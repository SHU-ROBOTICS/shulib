# G1 Part 1 — live progress log

> **Chunk:** G1 — `RobotConfig` / `Route` + `RobotBuilder`: a robot from data. **Part 1.**
> **Date:** 2026-09-18. **Tree at start:** `280dce8` (branch `shulib-v2`).
> **Executor model:** opus. **Brief:** [`G1-robot-config-and-builder.md`](G1-robot-config-and-builder.md).
> Appended in real time. Nothing in this chunk is committed by the executor.

---

## Order of work

1. This log; required reading; the clean-tree suite count at `280dce8`.
2. `include/shulib/config/robot_config.hpp` + `test/robot_config_test.cpp` (test 1);
   `include/shulib/config/route.hpp` + `test/route_test.cpp` (tests 2–3).
3. `include/shulib/hal/device_provider.hpp` + `include/shulib/hal/fake/fake_device_provider.hpp`.
4. `include/shulib/robot/robot.hpp` — the TANK path; `test/robot_builder_test.cpp` tests 4, 5, 8, 9, 10.
5. The X-drive path + tracking wheels + the GPS corrector; tests 6, 7.
6. All verification; the documentation contract; the prose review; the DoD with evidence; the final count.

**Standing constraints for this session (from the launch prompt):** the team lead is at the robot
with packages built from an earlier commit, so `src/`, the `Makefile`, `tools/src_build_gate.py`
and `docs/internal/chunks/R3a-BENCH-WORKSHEET.md` are untouchable. No commit, no push, no `git add`,
no worktree.

---

## Journal

### 1. Log created; baseline

**Clean-tree suite at `280dce8`:** `1306 cases / 1,625,161 assertions, 3 skipped — SUCCESS`.
Matches the number the launch prompt carried, so the tree is the one the brief was written against.

**One tree change before any code was written, and why.** The first `cmake --build build/test` went
RED on the *briefing* gate, not on a test: `tools/briefing_status.py` derives "INTERRUPTED CHUNK(S)"
from the set of `chunks/*-PROGRESS.md` files with no `-COMPLETED.md`, so creating this log made
`PROJECT-BRIEFING.md` stale *by existing*. Regenerated (`briefing_status.py generate`) — one line,
`G1` added to the interrupted list, which is true. `docs/internal/PROJECT-BRIEFING.md` is on the
documentation contract's touch list anyway. (`docs/internal/chunks/SESSION-2026-09-18-autonomous.md`
also shows as modified: that is the coordinator's launch entry, written after the git snapshot in
the prompt. Not mine, not touched.)

**Required reading, done in full before a line was written:** `RESUMING.md` (both named sections);
the brief §0–§8; `chassis/chassis.hpp` (standalone-promise section + the constructor);
`motion/motion.hpp` (`MotionDeps` + `validate()` + `tickHealthObservables`);
`chassis/robot_context.hpp`; `hal/drivetrain_description.hpp`; `src/main.cpp` READ ONLY
(the X-drive `Robot` ~335–424, `TankMotors` ~554, `tanklib::TankRobot` ~604–670);
`kinematics/{kinematics,tank,x_drive,h_drive,matrix_kinematics}.hpp`;
`localization/{localizer,pilons_odometry,drive_encoder_odometry,tracking_wheel,complementary_fusion,
gps_corrector,i_corrector,odometry}.hpp`; `hal/{motor,imu,gps,rotation,battery,clock,telemetry_sink,
motor_group,drive_geometry,absent_*,device_registry}.hpp`; `hal/fake/*`;
`sim/{drive_plant,scenario,degradation}.hpp`; `test/motion_test_rig.hpp`;
`test/chassis_facade_test.cpp` (the standalone case + `ChassisRig`);
`test/motor_group_plant_test.cpp` (`CoupledMembers`, `GroupRig`); `chassis/routine.hpp` (verbs only,
not touched); master plan §16.1–16.5 + §17; `docs/legacy-command-vocabulary.md`;
`.github/workflows/ci.yml` (the two greps); `tools/api_doc_tool.py`'s header; `test/README.md`;
`core/check.hpp` (the `SHULIB_PRECONDITION` macro takes a RUNTIME `const char*` — which is what lets
test 10's verdict message reach the thrown `PreconditionError` verbatim).

**Four measurements taken from that reading that changed the design (details in the rulings below):**

1. `sim::DrivePlant::wheelCommand()` returns *the primary motor's* commanded voltage when every
   member agrees and the members' MEAN otherwise (`drive_plant.hpp:396-411`). So a builder-made
   group that does not contain the plant's own primary `FakeMotor` would leave that primary at 0 V
   and the plant would drive on a mean that includes a silent motor. **Bit-identity therefore
   requires the provider to be able to serve a port with a CALLER-owned device** — the `bind*`
   hooks below.
2. `GroupRig` (the rig test 5 names) runs `PilonsOdometry` over the harness's tracking wheels, not
   `DriveEncoderOdometry`; the brief's §2 item 5 says the tank config is the *drive-encoder* one.
   Both readings are honoured — see ruling G1-R7.
3. `sim::DegradationModel`'s default `GpsTruth::hasFix` is `true` (`degradation.hpp:70`), so on the
   plant a configured GPS really does fold corrections. Test 7's "present ⇒ installed" half is
   therefore observable in the run, not only in the object graph.
4. `chassis::ChassisConfig` IS `constexpr`-default-constructible (measured with a throwaway TU), so
   `TuningConfig = chassis::ChassisConfig` does not cost `RobotConfig` its `constexpr`-ness.

### 2. The five headers

| File | What landed |
|---|---|
| `include/shulib/config/robot_config.hpp` | `RobotIdentity`, `DrivetrainKind`, `TankDrivetrain` (embedding `hal::DrivetrainDescription` BY VALUE), `XDriveDrivetrain`, `HDriveDrivetrain`, `DrivetrainConfig` (+ `kindOf/imuPort/cartridge/wheelCount/driveMotorCount`), `OdometryKind`, `TrackingWheelConfig`, `TrackingWheelsOdometry`, `OdometryConfig`, `GpsConfig`, `SensorsConfig`, `MechanismConfig`, `TuningConfig = chassis::ChassisConfig`, `PortUse` + `portUseWord`, `RobotConfig{valid, describe}`, `IRobotConfig`, `StaticRobotConfig`. Capacities as named constants (B6). |
| `include/shulib/config/route.hpp` | `kMaxWaypoints/kMaxMarkersPerWaypoint/kMaxMarkerArgs`, `WaypointMotion`, `HeadingMode`, `Constraints`, `Marker`, `Waypoint`, `Route{valid}`, `IRouteSource`, `StaticRoutes`. |
| `include/shulib/hal/device_provider.hpp` | `IDeviceProvider` — `motor(signedPort, cartridge)`, `imu(port)`, `gps(port, leverFwd, leverLeft)`, `rotation(port)`, port-less `clock/battery/telemetry/pacer`, `lastRefusal()`. |
| `include/shulib/hal/fake/fake_device_provider.hpp` | `FakeDeviceProvider` — fixed per-port storage, `bind*` hooks, `fake*` accessors, `refusePort/allowPort/isRefused`, `motorRequested/motorCartridge/gpsLeverForward/gpsLeverLeft`. |
| `include/shulib/robot/robot.hpp` | `BuildVerdict`, `validate(cfg, devices)`, `robot::Robot` (the whole graph, `chassis/localizer/faults/health/context/config/hasGpsCorrector/motorGroupCount/describe`). |

Tests: `test/robot_config_test.cpp` (12 cases), `test/route_test.cpp` (10),
`test/robot_builder_test.cpp` (14) — 36 cases in all.

### 3. Suite counts, and the ONE pre-existing number that moved

*(Measured on the FINAL tree, after the two cases and the extra checks that the mutation campaign
forced — §5. An intermediate reading of 1340 / 1,630,369 was taken before those landed.)*

| | cases | assertions |
|---|---|---|
| clean tree `280dce8` | 1306 | 1,625,161 |
| after G1 Part 1, whole suite | **1342** | **1,632,353** |
| after G1 Part 1, `-tce="G1 *"` (the pre-existing cases alone) | **1306** | **1,625,176** |
| ⇒ G1's own | **+36** | **+7,177** |

**Pre-existing cases moved by +15 assertions, and the cause is fully accounted for — no
pre-existing behaviour changed.** The in-tree mirror of the CI PROS-free guard
(`test/pros_adapter_fence_test.cpp`, case *"only hal/pros includes `<pros/>` — the in-tree mirror
of the CI guard"*) walks `include/shulib/**.hpp` recursively and spends exactly THREE assertions
per header outside `hal/pros/`: one `REQUIRE_MESSAGE` inside its `readFile` helper plus two
`CHECK_MESSAGE`s on the file's text. G1 adds five headers, none of them under `hal/pros/`, so the
case does 5 × 3 = 15 more assertions. Measured, not reasoned:

```
$ ./build/test/shulib_tests -tc="only hal/pros includes <pros/>*"   →  483 assertions
$ find include/shulib -name '*.hpp' | wc -l                          →  178
$ find include/shulib/hal/pros -name '*.hpp' | wc -l                 →   17
   3 * (178 - 17) = 483   ✔  and 3 * (173 - 17) = 468 at 280dce8, so the delta is exactly 15.
```

That case getting bigger IS the guard working: every new header is now scanned for a `pros/`
include. No other pre-existing case's count moved.


---

## 4. Rulings beyond the brief

Each one names the alternative and the reason, per the standing rule.

**G1-R1 — `DrivetrainConfig` / `OdometryConfig` are TAGGED STRUCTS, not unions or
`std::variant`.** All alternatives are stored and a `kind` field says which is read.
*Alternative:* a real union or `std::variant`, which is what "tagged choice" normally means and
which would save about thirty bytes. *Reason:* it would cost the type its AGGREGATE-ness, and
being an aggregate is exactly what lets a person — and G3's generator — write
`{.kind = …, .tank = {…}}` with designated initializers and have the compiler check every field.
Thirty bytes of static storage on a brain with megabytes, paid once, buys a `constexpr`-emittable
schema. The ignored members are documented as not read, not validated and not built.

**G1-R2 — `config::DrivetrainKind` is a NEW enum, not `hal::DrivetrainKind`.** *Alternative:*
extend `hal::DrivetrainKind` with `XDrive`/`HDrive`. *Reason:* that enum's values are written into
a persisted calibration record and it is append-only for that reason; it knows only `Tank` because
the calibrator can only measure a tank. The config vocabulary needs the two holonomic kinds TODAY
(src/main.cpp's X-drive graph has existed since R1a and must be expressible as data in Part 2), and
minting record values for kinds no record can ever carry would put unreachable states in a
serialization format. A tank config carries both: `config::DrivetrainKind::Tank` and the
description's own `hal::DrivetrainKind`, and `valid()` checks them against each other through the
description's own rules.

**G1-R3 — there is NO `robot::build()` free function; it is `validate(cfg, devices)` +
`Robot{cfg, devices}`.** The brief's §2 item 4 heading says "`robot::Robot` and `robot::build()`"
and §0's sketch writes `Robot robot{shulib::robot::build(config, devices)}`. *Reason:* that
spelling cannot compile. `Chassis` is non-copyable and non-movable (its scheduler is pinned by a
self-referential command-id stamp), so `Robot` is too, and a function returning one by value
depends on guaranteed-elision behaviour the brief's OWN ruling B3 explicitly rejects
("a non-movable object cannot be returned by value without guaranteed elision games the V5
toolchain should not be asked to play"). The brief's body wins over its heading. The DoD's "two
lines" is unaffected: `FakeDeviceProvider devices{…}; Robot robot{config, devices};`.

**G1-R4 — the F5 wheel-count cross-check has exactly ONE owner, and it is `robot::validate()`.**
`RobotConfig::valid()` deliberately does NOT refuse a holonomic wheel whose port is `0`.
*Alternative:* check it in both places, which reads safer. *Reason:* two owners for one rule is how
two checks come to disagree, and `0` already means "no port" everywhere else in this tree
(`drivetrain_description.hpp` says so in words). The practical consequence is the one that decided
it: a rule checked only in an unreachable place is dead code that no mutation can turn red. With
one owner, `validate()`'s check is reachable from a config that `valid()` accepts, and mutation M6
proves it. Both `valid()`'s documentation and `validate()`'s say where the other one is.

**G1-R5 — `DriveEncoders` odometry is REFUSED on a holonomic drivetrain.** Not in the brief.
*Reason:* `localization::DriveEncoderOdometry` reads exactly a LEFT and a RIGHT side and converts
each with its own geometry; an X-drive has neither, so the builder would have to invent two
`IMotor`s to construct one. The pairing is not merely unsupported, it is unrepresentable, and
refusing it in the data is cheaper than a precondition three objects deep.

**G1-R6 — the fake provider can serve a port with a CALLER-owned device (`bindMotor` and friends),
and that is load-bearing, not a convenience.** *Alternative:* the provider hands out only its own
fakes and the test attaches those to the plant. *Reason (measured):* `sim::DrivePlant::wheelCommand()`
reads the PRIMARY `FakeMotor`'s commanded voltage when the wheel's members agree and their MEAN
when they do not, and the primary is owned by the `SimHarness`. A group built only from
provider-owned fakes leaves that primary at 0 V, so the plant drives on a mean that includes a
motor nobody commanded — not a different robot, a wrong one. The IMU, GPS and tracking-wheel
encoders are plant-synthesized and bind for the same reason. Ruling B2 is preserved: the BUILDER
still owns no hardware; the `SimHarness`, which outlives the `Robot` in every rig, owns the bound
devices, exactly as `ProsDeviceProvider` will own everything it makes in Part 2.

**G1-R7 — test 5 is run BOTH ways, because the brief names two different tank compositions.**
§4's row 5 says "vs `motor_group_plant_test`'s `GroupRig`", which runs `PilonsOdometry` over the
harness's tracking wheels; §2 item 5 says the tank config is the DRIVE-ENCODER one. Both are built
(cases 5/5b for drive encoders — what `src/main.cpp`'s `TankRobot` actually is, and the graph Part 2
must replace — and 5c for `GroupRig`'s exact composition), plus 5d for the degenerate one-member
group. The longhand rigs are written out IN FULL in the test file rather than reusing a helper the
builder also uses: a shared helper would make the two sides agree by construction and prove nothing.
(`GroupRig` itself is in another translation unit's anonymous namespace and is unreachable, so it
had to be transcribed either way.)

**G1-R8 — the builder DERIVES the stall check's per-wheel geometry and `independentMotionSource`
for a tank, and leaves a holonomic robot's tuning untouched.** *Reason:* it reproduces what each
graph in `src/main.cpp` actually does — `tankChassisConfig(leftGeometry)` for the tank,
`Chassis chassis{deps, pacer}` (default config) for the X-drive — and it is the project's own
"discover the drivetrain's facts, do not make humans type them" rule applied to the one number the
builder already holds. A holonomic config carries no per-wheel scale (its odometry is the pods), so
there is nothing to derive and the builder invents nothing.

**G1-R9 — `hal/device_provider.hpp` FORWARD-DECLARES `motion::ITickPacer` instead of including the
motion layer.** *Alternative:* `#include "shulib/motion/motion_scheduler.hpp"`. *Reason:* that
header sits four layers above `hal/`, and including it from an L0 seam would invert the layering
for one reference. A reference to an incomplete type is all a declaration needs; the consumer
(`robot/robot.hpp`) already has the whole motion stack. The pacer belongs ON the seam — "advance the
world to the next control tick" is exactly as target-specific as "give me the motor on port 3".

**G1-R10 — `StaticRobotConfig` COPIES; `StaticRoutes` VIEWS.** Deliberately asymmetric.
*Reason:* size. One `RobotConfig` is a few hundred bytes and copying it makes the
decode-into-a-local-then-hand-out-a-reference bug impossible. A `Route` is roughly 5 KB, so a table
of them copied into a second buffer would double the largest static object on a brain with 128 KB
of user RAM — to protect against a lifetime bug that a `constexpr` table at namespace scope (the
only way either producer emits one) cannot have. Both choices are stated in the headers.

**G1-R11 — `Route::valid()` requires a non-empty NAME and at least one waypoint, and refuses a
verb/heading-mode mismatch.** Beyond the brief's list. *Reasons:* a route nobody can name is a
route `IRouteSource::route()` can never return; a route with no waypoints is nothing to execute;
and a `DriveTo` with `HeadingMode::Fixed` states a heading the verb computes from the approach and
then silently ignores, which is the quieter of the two mismatch bugs and therefore the one worth
naming out loud. Index 0 is NOT required to be a `Start` — a route that begins from wherever the
robot already is, is what every hand-written auton in this repo's history does, and requiring one
would force every importer to invent a pose it does not have.

**G1-R13 — `Robot` does NOT keep a handle to its provider.** It was written with one
(`hal::IDeviceProvider* devices_`) and it was removed before the chunk closed. *Reason:* nothing
read it. Every device the graph needs is taken during construction and held where it is used, so a
second handle is an unused member that reads like an ownership claim — and ruling B2 is precisely
that the builder owns no hardware. The provider must still outlive the `Robot`; that is a lifetime
contract stated in the class banner and the constructor, not a field. (`-Wunused-private-field` is
a clang warning and this project builds with g++ only, so nothing would have caught it; it was
removed on the argument, not on a diagnostic.)

**G1-R12 — the "two lines" test fences the host world above the two lines rather than spelling
`FakeDeviceProvider devices{h};`.** The brief's §4 row 8 gives that literal spelling. *Reason:* `h`
is a `sim::SimHarness`, and `hal/fake/` must stay SIM-FREE (the CI layering guard, and the brief's
own constraint). A constructor taking the harness would either include `shulib/sim/` — refused by
the guard — or hide the dependency behind a template, which removes the include without removing
the coupling. `SimHarness` also exposes neither a telemetry sink directly nor a pacer. The test
therefore builds the plant world first (the host's stand-in for "a robot exists and is plugged in",
which on the brain is `ProsDeviceProvider devices;` and nothing else) and marks THE TWO LINES
explicitly. The substance of the DoD sentence is unchanged; the spelling is not the brief's.

---

## 5. The mutation campaign — 15 runs, every one on a FRESH binary

Protocol for every row: edit the header, `touch` the three G1 test files (CMake's `/fast` target
skips dependency scanning — R3d Part 2 was bitten by exactly this), build
`--target shulib_tests/fast`, **require `Linking CXX executable shulib_tests` in the output before
trusting any result** (the harness `exit 8`s without it), run the target cases, record the OBSERVED
message, restore. A mutation not actually run does not count; two of these were run, came back
GREEN, and were the most useful results in the chunk.

| # | Mutation | Target | Observed |
|---|---|---|---|
| M1 | `RobotConfig::valid()`: the port-collision `claim()` never fires | config 1b/1c | **RED** — `CHECK( why.find("port 3") … )`, `…("drive wheel")`, `…("forward tracking wheel")`, and 11 more across all five collision sub-cases |
| M2 | drop the `DriveEncoders`-on-a-holonomic-drivetrain check | config 1f | **RED** — `CHECK( why.find("DriveEncoders") … )`, `…("x-drive")`, `…("TrackingWheels")`; 4 of 5 assertions failed |
| M3 | `Route::valid()`: a `Start` may sit anywhere | route 2b | **RED** — `CHECK( why.find("waypoint 2") … )`, `…("Start")` |
| M4 | drop the `Hold`-needs-a-duration check | route 2c | **RED** — `CHECK( why.find("waypoint 3") … )`, `…("Hold")`, `…("duration")` |
| M5 | `robot::validate()` skips the DEVICE pass entirely | builder 4 | **RED** — `CHECK_FALSE( v.ok )` plus the port/phrase checks, on the refused motor AND on all four of IMU/GPS/forward pod/lateral pod |
| M6 | drop the F5 wheel-count cross-check | builder 4 | **RED** — `CHECK( why.find("4 kinematic wheels") … )`, `…("3 motor ports")`, `…("drivetrain.ports")`. *(`v.ok` still went false — the 0 port then fails the DEVICE pass instead — which is why the message checks, not the bool, are the detector.)* |
| M7 | swap the two `MotorGroup`s in the kinematic wheel order | builder 5, 5b, 5c, 5d | **RED** — `REQUIRE( built.exit == longhand.exit )` in all four. **First run: only 1 of 4 red — see §5.1** |
| M8 | the builder stops deriving the stall check's per-wheel geometry | builder 5e | **RED** — `CHECK( stall.wheels[w].inchesPerRadian().value() == kPlantInchesPerRadian )` ×3. **First run: GREEN — see §5.1** |
| M9 | `independentMotionSource` hard-wired to `true` | builder 5e | **RED** — `CHECK_FALSE( stall.independentMotionSource )`. **First run: GREEN — see §5.1** |
| M10 | install the GPS corrector REGARDLESS of `sensors.gps.present` | builder 7 | **RED** — `CHECK_FALSE( built.robot.hasGpsCorrector() )` ×2 and `CHECK( …lastCorrection().source == "none" )`. *(Builder 6 stayed green and correctly so: a corrector over `AbsentGps` never proposes, so the PLANT cannot see it — which is exactly why test 7 exists.)* |
| M11 | drop the construction precondition in `Robot::prechecked()` | builder 10 | **RED** — `CHECK( e.what() == v.why )` and `…find("port 13")`, `…find("inches per radian")` |
| M12 | swap the forward/lateral tracking-wheel configs | builder 5c, 6, 6b | **RED** — `REQUIRE( built.exit == longhand.exit )` ×3 |
| M13 | `MotionDeps::motorGroups` handed over EMPTY | builder 5f | **RED** — `CHECK( …raiseCount(MotorGroupDisagree) >= 1 )`, `CHECK( …firstFault() == … )`. **First run: GREEN — see §5.1** |
| M14 | `wireMotors()` hands the provider `|port|` instead of the stored SIGNED port | builder 4 | **RED** — `CHECK( motorSignedPort(…) == kLeft[i] / kRight[i] )` ×5. **First run: GREEN — see §5.1** |
| M15 | the GPS corrector is built but the corrector count handed to the `Localizer` stays 0 | builder 6b, 7 | **RED** — `REQUIRE( built.ticks == longhand.ticks )` and `CHECK( …lastCorrection().source == "gps" )` |

Two mutations were rejected before they counted because the mutant did not COMPILE
(`-Werror=unused-parameter` on a dropped argument) — the harness refused to report a result
without a link line, and both were rewritten into forms that compile. That guard did its job.

### 5.1 The four mutations that came back GREEN, and what each one bought

This is the part of the campaign that was worth the time. Four mutants survived, every one of them
a genuinely wrong robot, and each exposed a hole in the TESTS rather than in the code.

**M7, first run — 1 of 4 cases red.** The comparison leg was a single `moveTo` straight along the
robot's own heading (a tank cannot strafe, so a laterally-offset target honestly exits `TimedOut` —
C1's drivetrain honesty, which is correct behaviour and not something to design around). But a
straight-line move commands BOTH sides the same voltage every tick, so swapping the two groups is
numerically almost a no-op. **Fix:** the comparison sequence is now `moveTo` **then `turnTo`**. A
rotation commands the two sides OPPOSITE voltages, which is what makes a side swap visible on the
first tick. Re-run: **4 of 4 red.** A bit-identity test is only as strong as the run it compares,
and that sentence is now in the test file above `runLeg()`.

**M8, first run — GREEN, and the reason is a trap worth naming.** The fixture's drivetrain used the
A2 plant's default 3.25 in wheel, so its inches-per-radian was 1.625 — which is *also* the default
of `hal::DriveGeometry` AND of every slot of `OdoStallCheckConfig::wheels` (the HA-14 stand-in). The
mutant dropped the derivation, the default took over, and the default happened to equal the value
being derived. **A fixture that shares a value with the library default cannot test that the value
was plumbed.** *Fix:* the fixture's plant now uses a 4.0 in wheel (`kPlantWheelDiameter`), so the
derived number is 2.0 and differs from every default; the reasoning is recorded at the constant.

**M8/M9 also needed a new case at all.** Even with a distinguishable number, a HEALTHY run never
trips the stall check, so neither mutant changes a single commanded volt and the bit-identity cases
are structurally incapable of seeing them. **New case `G1 builder 5e`** asserts the derived tuning
on the FACADE (`chassis().motionConfig().stall`) for the tank on drive encoders, the tank on
tracking wheels (where `independentMotionSource` is legitimately `true`), and the holonomic robot
(whose tuning is passed through untouched) — plus a check that a caller's own tuning still survives,
because "the builder adds, it does not replace". Both mutants now go red.

**M13, first run — GREEN.** Emptying `MotionDeps::motorGroups` changes nothing about a healthy run,
because a group's disagreement observable is only evaluated THROUGH that span and a healthy group
never disagrees. So the wiring was present and untested. **New case `G1 builder 5f`** freezes member
2 of the left wheel through the plant and requires `MOTOR_GROUP_DISAGREE` on the Robot's own fault
latch, with a healthy twin that raises nothing — `motor_group_plant_test`'s case 3, reached from a
config instead of from a hand-wired rig. M13 now goes red.

**M14, first run — GREEN, twice over.** First: a host fake CANNOT show a sign error by behaving
differently, because PROS applies the reversal inside the real adapter and a fake indexed by |port|
is identical either way. The sign is the single most dangerous number in a drivetrain description
(a side wired backwards drives the robot backwards, silently), so it needed a STRUCTURAL check:
`FakeDeviceProvider::motorSignedPort(port)` records the signed value of the most recent request, and
test 4 asserts every port came over as stored. Second: even with that, M14 stayed green, because the
check ran only after `validate()` — and M14 mutated `wireMotors()`, a *different* pass that asks the
provider again. The assertion now runs **twice**, once after `validate()` and once after the `Robot`
has wired itself, and that is written down in the test as the reason. M14 now goes red.

---

## 6. Verification — every gate, on the final tree

| Gate | Command | Result |
|---|---|---|
| Host suite | `cmake --build build/test && ./build/test/shulib_tests` | **1342 cases / 1,632,353 assertions, 3 skipped — SUCCESS** |
| Pre-existing cases alone | `./build/test/shulib_tests -tce="G1 *"` | **1306 / 1,625,176 — SUCCESS** (+15, accounted for in §3) |
| `src/` build gate — self-test | `python3 tools/src_build_gate.py self-test` | **OK (33 detector cases, each against a real build)** |
| `src/` build gate — check | `python3 tools/src_build_gate.py check` | **PASS — all 6 builds link**; `bench`, `xdrive`, `tank`, `tank-drive`, `tank-library`, `tank-calibrate`, each `exit 0, 4/4 TUs, 0 disallowed warnings, beacon checked` |
| ARM cross-compile, ALL headers | `arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror -Os -mcpu=cortex-a9 …` | **PASS — 178 headers** (173 at `280dce8` + G1's 5) |
| PROS-free guard | `grep -rnE '#\s*include\s*[<"]pros/' include/shulib \| grep -v '^include/shulib/hal/pros/'` | **empty** |
| Layering guard (sim) | `grep -rnE --exclude-dir=sim '#\s*include\s*[<"]shulib/sim/' include/shulib` | **empty** — `config/`, `robot/`, `hal/device_provider.hpp` and `hal/fake/fake_device_provider.hpp` are all sim-free; the plant attachment is done by the TEST |
| API reference | `api_doc_tool.py check-coverage` / `check-fresh` | **2253 public entities across 142 headers, all documented**; fresh |
| Briefing | `briefing_status.py generate` + `check` | **current** |
| C7 removability | `grep -rn "internal/\|chunks/\|RESUMING\|build-order" docs/*.md docs/guide docs/cookbook docs/api README.md` | **empty** |
| Doc staleness audit | (runs in the build) | **clean — 26 live docs** |

**`src/` was not modified.** `git status --short` shows no `src/` entry at any point, and the
six-build gate is byte-for-byte the same gate that was green at `280dce8`. Neither were the
`Makefile`, `tools/src_build_gate.py`, nor `docs/internal/chunks/R3a-BENCH-WORKSHEET.md`.

**One stale-binary trap was hit and is worth recording for the next session.** The mutation harness
restores the SOURCE but leaves the MUTANT BINARY on disk. A `briefing_status.py generate` run in
that window baked *"RED — 1 case(s) failing"* into `PROJECT-BRIEFING.md`, and the failure only
surfaced on the next `check`. The briefing was regenerated against the clean, green binary as the
last step. **Regenerate the briefing only immediately after a clean full build.**

---

## 7. Prose review (the logged step, per pace rule 4)

Grepped `docs/`, `docs/guide`, `docs/cookbook` and `README.md` (excluding the generated
`docs/api/`) for `RobotBuilder`, `RobotConfig`, `IRobotConfig`, `IRouteSource`, `Route`,
`standalone`, `two lines`, `no file`, `builder`, `config file`. Every hit was read.

### Stale, and FIXED

| Where | Was | Now |
|---|---|---|
| `docs/roadmap.md` M5 "Config ingestion" | `- [ ] IRobotConfig + RobotBuilder.from(profile) → a fully wired Chassis.` | `[~]` with the evidence, the actual spelling (`robot::Robot{config, devices}`) and the reason it is not `RobotBuilder.from`, and what is still OPEN (Part 2 + G3) |
| `docs/roadmap.md` "You are here" | ended at R3d Part 2 | a G1 Part 1 paragraph, ending *"`main.cpp` is NOT one line shorter yet"* |
| `docs/roadmap.md` §"the reciprocal half" | *"`IRobotConfig`, `IRouteSource`, `RobotBuilder`, `PathRunner`, … are **all unbuilt**"* | corrected, dated, with the first of them named as landed host-only and the rest still unbuilt |
| `docs/internal/build-order.md` G1 entry | no status line | a STATUS block: what landed, the bit-identity acceptance test, the `Robot{config, devices}` spelling ruling, and "Part 2 needs its own brief" |
| `docs/internal/build-order.md` first `**Next:**` | ended at T2's library half | one appended sentence for G1 Part 1; **R3d Part 3 as the bench session and everything else in that block are untouched** |
| `docs/vexbuilder-integration.md` "shulib's side" table | `IRobotConfig / IRouteSource / RobotBuilder \| ❌ \| G1` | `🚧` + what landed host-only + Part 2 open |

**`docs/vexbuilder-integration.md` was NOT on the brief's touch list, and the coordinator should
know I touched it anyway.** It was not on the "do not touch" list either. The row carried a bare
`❌` against types that now exist and are host-proven, which is a factually wrong public status
marker, and the standing rule is that docs move with the code. The edit is one table cell. If the
coordinator would rather keep the footprint to the briefed set, reverting that one line costs
nothing and changes no gate.

### Read and deliberately NOT changed

| Where | Why it is still true |
|---|---|
| `README.md:229` *"no config file or code generator is ever required"* | still exactly true — G1 ADDS an on-ramp; the standalone path is untouched and still pinned by its own file-free test. The README's front-page wiring paragraph changes when `main.cpp` does → **Part 2** |
| `docs/guide/08` (the wiring chapter) | same reason: it describes the longhand composition root, which is unchanged. Part 2 adds the builder as an alternative with the longhand kept as Tier 3 |
| `docs/shulib-v2-master-plan.md` §16.2 / §17 / decision #10 | §16.2's sketch is what this implements; it moves when G3 lands the file. Its `RobotBuilder.from(profile)` wording is the PLAN's, and the plan is not amended by an executor |
| `docs/faq.md` | nothing in it touches config ingestion (its one `no file` hit is about logging) |
| `docs/roadmap.md` F4 row, F11 row, WS9 row, the M5 badge (🎯), the "what shulib owes VexBuilder" table row A ("no — startable today") | all still true: G1 Part 2 IS startable today and ungated, M5 has six other open bullets, and F4's parenthetical about F7/F8 is unchanged |
| `docs/roadmap.md:1237` *"G1's RobotBuilder generates it"* (the §18.5 port map) | future tense and still future: `Robot::describe()` produces a banner, but `src/main.cpp`'s `portMapString()`/`tankPortMapString()` are still hand-built. **Part 2** |
| `docs/hardware-assumptions.md:2299` | "G1's RobotBuilder replace hand-kept port lists" — future tense, still future (Part 2) |
| `docs/vexbuilder-integration.md:160–161`, `:363`, `docs/guide/05:107`, `docs/guide/14`, `docs/guide/15` | the standalone/"two lines"/"no file" promises are all still true; `05:107`'s "config file" is a different sense of the phrase (a tuning value's documentation) |

### Listed for Part 2

`README.md`'s wiring paragraph; `docs/guide/08`'s "the wiring" section (builder as the on-ramp,
longhand kept as Tier 3); a `docs/guide/` chapter on describing a robot as data; `roadmap.md:1237`
and `hardware-assumptions.md:2299`'s port-map sentences; the master plan §16.2 wording once G3
lands the file.

---

## 8. Hardware-assumptions register — entries to add

**NONE. No new hardware belief was invented, and no HA number was minted.** Everything G1 Part 1
adds is either a DESIGN constant or a re-use of a number that already has a register entry:

- `kMaxWaypoints = 32`, `kMaxMarkersPerWaypoint = 4`, `kMaxMarkerArgs = 2`, `kMaxMechanisms = 8`,
  `kMaxMotorsPerMechanism = 4`, `kMaxPneumaticsPerMechanism = 2` — design constants (ruling B6).
  Nothing measured says any of them; a route longer than 32 is two routes. They are named
  constants with that reasoning in the header, and they are not hardware beliefs.
- Every physical number a `RobotConfig` carries is the CALLER's: the description's ports, scales and
  track width (already HA-135 / the R3d provenance chain), the X-drive's drive radius (HA-17 when
  `main.cpp`'s value is used), the tracking-wheel geometry (HA-12/13), the GPS lever arm (HA-10),
  the cartridge (human-stated by construction). G1 stores and validates them; it invents none.
- The one number G1 chose for itself is the TEST fixture's 4.0 in drive wheel, which exists
  precisely so the fixture does not share a value with a library default (§5.1). It describes no
  robot and is inside a test file.

**One thing a future register entry may eventually want, recorded here rather than minted:** the
builder's behaviour when a port is refused has only ever been exercised against
`FakeDeviceProvider::refusePort()`. What a real `ProsMotor` constructor does on a dead, mis-typed or
intermittently-answering port at boot is Part 2's `ProsDeviceProvider` and the bench, and HA-98's
read-back is the existing entry that covers it. No new number is warranted yet.

---

## 9. DoD — brief §5, with evidence

- [x] **`config/robot_config.hpp`, `config/route.hpp`, `hal/device_provider.hpp`,
      `hal/fake/fake_device_provider.hpp`, `robot/robot.hpp` — every public member documented; ARM
      gate green.** Evidence: `api_doc_tool.py check-coverage` → *2253 public entities across 142
      headers, all documented*; the ARM cross-compile of all **178** headers passes under
      `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror -Os` for cortex-a9. No
      `docs/internal/` path appears in any header (the C7 removability grep is empty).
- [x] **Tests 1–10 green; every mutation logged red; the whole pre-existing suite green and
      bit-identical.** Evidence: §5 (15 mutations, each with its observed message; four first-run
      GREENs and the four test strengthenings they forced); §3 (pre-existing cases +15 assertions,
      attributed line by line to one pre-existing case scanning five more headers — no pre-existing
      behaviour changed). Brief §4 coverage: 1 → `robot_config_test` 1a–1l; 2 → `route_test`
      2a–2h; 3 → `route_test` 3, 3b; 4 → `builder 4`; 5 → `builder 5/5b/5c/5d` (+5e, 5f);
      6 → `builder 6/6b`; 7 → `builder 7`; 8 → `builder 8`; 9 → `builder 9`; 10 → `builder 10/10b`.
- [x] **PROS-free and layering guards green; `src/` untouched (the six-build gate unchanged).**
      Evidence: §6. Both greps empty; `src_build_gate.py self-test` + `check` both PASS on six
      builds; `git status --short` never shows a `src/` entry.
- [x] **`G1-PROGRESS.md` complete with the prose-review log and the DoD evidence.** This file.
- [x] **No claim that `main.cpp` is cleaner yet.** `src/main.cpp` is byte-identical to `280dce8`.
      The roadmap's "You are here", the build-order status block and the build-order `Next:`
      sentence each say Part 2 owns the collapse; `robot/robot.hpp`'s banner says
      *"src/main.cpp is unchanged and still wires both of its graphs by hand — collapsing it is
      Part 2"*; the test file's banner says the same.

### What is NOT done, stated plainly

- **`hal::pros::ProsDeviceProvider` does not exist** — Part 2, deliberately out of scope.
- **`src/main.cpp` is unchanged.** Not one line shorter. Both graphs are still hand-wired.
- **No `Route` has ever been executed**, on a plant or on a robot. Part 1 pins the shape; the
  runner is G2.
- **No mechanism is constructed from a `MechanismConfig`.** Types only, as the brief scopes it.
- **`AbsentTagSource` / `AbsentVision` are unconditional** — there is no camera adapter to configure
  (R2), so `SensorsConfig` has no field for one.
- **NO HARDWARE CLAIM.** A `Robot` built from a config has driven the A2 host plant, through fakes,
  and nothing else. The library's motion stack has still never driven a robot.
- **Nothing was committed, pushed or `git add`ed.** No worktree was created.

---

## Part 2a — 2026-09-24 01:32:44 EDT

> **Part:** G1 Part 2a — the library half (the PROS device provider, absent drive ports in the
> builder, `Robot::portMap()`/`driveOdometry()`, `teleop::runDriverControl`). **Tree at start:**
> `018424c` (branch `shulib-v2`, clean). **Executor model:** opus. **Brief:**
> [`G1-part2-pros-provider-and-main.md`](G1-part2-pros-provider-and-main.md) §2a. The team lead's go:
> 2026-09-24. Appended in real time; every timestamp from a `date` call. Nothing here is committed
> by the executor. `src/`, `Makefile`, `tools/src_build_gate.py`, `.github/`, the worksheet: not
> touched.

### 2a.0 Log created (01:32:44 EDT)

Log section created before any code. Next: the required reading, then the clean-tree suite count.

### 2a.1 Required reading, and the baseline (01:40:20 EDT)

**Read in full before any code:** `RESUMING.md`; the brief (revision note, §0, §1, §2a in full,
§3 M1–M5, §4 tests 1–5, §5, §6, §7); Part 1's log above; `HANDOFF-remaining-work.md` §3;
`robot/robot.hpp` (all); `hal/device_provider.hpp`; `hal/fake/fake_device_provider.hpp`;
`hal/device_registry.hpp`, `hal/fake/fake_device_registry.hpp`, `hal/pros/device_registry.hpp`;
the adapters `hal/pros/{motor,imu,gps,rotation,tick_pacer,battery,clock,char_sink}.hpp`; the shim
`test/pros_shim/pros/{shim_control.hpp,apix.h,motors.hpp,imu.hpp,gps.hpp,rotation.hpp,rtos.hpp,
misc.hpp}`; `test/pros_adapter_fence_test.cpp` (it constrains any new `hal/pros/` header — see
below); `hal/motor_group.hpp` (the ABSENT member); `teleop/{driver_control,stick_mapping,
drive_modes,field_centric,input_shaping}.hpp`; `hal/controller.hpp`; `hal/fake/fake_controller.hpp`;
`localization/localizer.hpp` (`Quality`); `motion/motion_scheduler.hpp` (`ITickPacer`);
`calibration/boot_check.hpp` (`BootVerdict::isMissing`); `diag/session_info.hpp`;
`chassis/chassis.hpp` (`drive`, `cancel`, `deps`, `motionConfig`); `config/robot_config.hpp`
(the three drivetrain alternatives, the tracking-wheel and GPS configs);
`hal/drivetrain_description.hpp` (`sideOf`, `signedPortOf`, `portMagnitude`); `core/check.hpp`
(`PreconditionError` is a `std::logic_error`); `src/main.cpp` READ ONLY (`runTeleopLoop` :452-473,
`TankMotors` :554-596, `tankPortMapString` :678-722, the X-drive `portMapString` :432-440, the
graph-boundary catch :1060-1066); `test/robot_builder_test.cpp` (all); `test/driver_control_test.cpp`
(all); `test/guide_examples_test.cpp` (the ch. 8 cases); `test/CMakeLists.txt`;
`tools/api_doc_tool.py` (target globbing, `do_check_examples`).

**Baseline, measured on this tree (`018424c` + this log's append; configured hash
`v0.1.1-313-g018424c-dirty`):** `1358 test cases / 1,635,266 assertions, 3 skipped — SUCCESS`.
It matches the handoff's number exactly. Per-case assertion counts were dumped to a scratch file
(`--reporters=xml`) so every pre-existing case that moves can be named, not guessed.
`only hal/pros includes <pros/>*` = 492 = 3 × (181 headers − 17 under `hal/pros/`), the formula
Part 1 recorded.

**Facts measured from the reading that shape the design:**

1. `test/pros_adapter_fence_test.cpp` walks EVERY `hal/pros/*.hpp`: a file with no
   `#include "pros/…"` must contain no `#pragma GCC diagnostic` at all. So the provider header
   includes only other `shulib/hal/pros/` adapters, never a `pros/` header directly, and needs no
   fence of its own — the adapters' fences cover the vendored code.
2. The shim models the registry, a motor read-back failure (`MotorPortState::disconnected` makes
   `get_encoder_units()` answer `invalid`, which throws in `ProsMotor`'s constructor), and IMU
   calibration (`ImuPortState::resetCalls`). So every test-1 property is expressible through the
   shim as it stands — including an EMPTY IMU/rotation/GPS port: the registry answers
   `E_DEVICE_NONE` by default and the provider must refuse on that answer alone, because
   `ProsImu`'s and `ProsRotation`'s constructors would succeed. No brief assumption is false here.
3. `ProsMotor`'s precondition messages contain an EM DASH (UTF-8 `E2 80 94`). The brief says the
   provider's refusal carries "the adapter's message", and the task says refusal text must be
   plain ASCII. Both are honoured by copying the message through an ASCII filter (ruling below).
4. `FakeDeviceProvider::motorRequested()` is set only when a request is SERVED — a refused port
   never sets it. So "the absent port was never requested" cannot be proved with it alone (a
   builder that asked and ignored a refusal would pass). The fake gains a counter of EVERY
   `motor()` call per port, refused or not.
5. `src/main.cpp`'s `tankPortMapString()` prints `"<SOURCE> L <ports> R <ports> IMU<n> ipr
   <l>/<r> TW<w>"`, each side `%+d` space-separated and capped at 39 characters, the whole bounded
   to the 96-byte field. `Robot::portMap()` reproduces that byte for byte for a drive-encoder,
   GPS-less tank (so 2b's switch changes no session header), pinned by a literal in the test.

### 2a.2 First code pass (01:52:22 EDT)

Written: `include/shulib/hal/pros/device_provider.hpp` (new), `include/shulib/teleop/teleop_loop.hpp`
(new); `include/shulib/robot/robot.hpp` (`BuildOptions`, `validate(cfg, devices, options = {})`,
`writePortMap()`, `Robot{cfg, devices, options = {}}` with the options reaching `prechecked()` and
`wireMotors()`, `portMap()`, `driveOdometry()`, `describe()` naming absent ports, the banner's
stale "Part 2" sentences); `include/shulib/hal/fake/fake_device_provider.hpp`
(`motorRequestCount()`); `include/shulib/teleop/field_centric.hpp` + `driver_control.hpp` (the
`rezero()` finding — ruling G1-2a-R6 below).

Two compile checks before any test: the host test flags (`-Wall -Wextra -Wpedantic -Werror
-Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion`, shim on the path) — clean; and a
scratch TU that ODR-USES every new function the way Part 2b's `src/` will (`ProsDeviceProvider`
in `std::optional` storage, `validate` + `Robot` with options, `portMap`, `describe`,
`driveOdometry`, `runDriverControl` over a `ProsController`), compiled with
`arm-none-eabi-g++` and the `src/` build's own flags (`-Os -Wall -Wextra -Wformat=2 -Wno-psabi`,
common.mk's machine flags, the REAL vendored PROS headers) — **zero diagnostics**, so
`-Wformat-truncation` / `-Wformat-nonliteral` will not bite 2b from these headers.

**Not mine, noted so nobody mistakes it for this part's work:** `docs/internal/chunks/R3d-PROGRESS.md`
(+35 lines, "Part 3 — THE BENCH SESSION … coordinator's live log", started 01:36:21) and
`docs/internal/evidence/r3d-session-2026-09-24-census.log` (untracked) appeared in the working
tree while this part was running. They are the coordinator's bench-session record (packages built
from the committed `018424c` in a separate worktree, per landmine 9). This part does not touch
them. One consequence to watch: the briefing is regenerated LAST by this part, and whatever the
coordinator's files say at that moment is what the generator reads.

### 2a.3 Tests written; the suite green; the pre-existing numbers accounted for (02:07:22 EDT)

**27 new cases**, mapped to brief §4:

| Test | Where | Cases |
|---|---|---|
| 1 (provider) | `test/pros_device_provider_test.cpp` (new) | `ProsDeviceProvider 1a` same port ⇒ same object, constructed once (the shim's configuration counters do not move on the repeat) · `1b` EMPTY IMU / rotation / GPS / motor port ⇒ nullptr + the exact sentence, and the shim device never touched · `1c` wrong kind — a sweep putting ALL 13 `v5_device_e_t` values on the port for each of the 4 requests, only the right one served — plus four verbatim sentences · `1d` motor read-back failure ⇒ nullptr + the ADAPTER's message, ASCII, whole; a later request after the port answers constructs afresh · `1e` sign / cartridge / lever-arm / second IMU / second GPS ⇒ `PreconditionError` · `1f` calibration started once across 6 requests, never tared · `1g` out-of-range ports and a cartridge byte 0 refused without a single registry query · `1h` registry asked at construction only; a refusal is not cached · `1i` an injected registry replaces the owned one · `1j` the port-less handles · `1k` a whole `robot::Robot` over the REAL provider through the shim: every stored sign arrives at the device (−6000 mV on a negative port), `portMap()` byte-exact for robot two's shape, and ports 16/17 EMPTY ⇒ refused by name without the list, built with it, never opened, 3 of 5 present, `DEGRADED: absent 17,16` |
| 2 (absent ports) | `test/robot_builder_test.cpp` (appended) | `G1 2a builder A1` one absent port: validate passes, the Robot CONSTRUCTS, 4/5 present with the hole in slot 2, the port NEVER requested (`motorRequestCount`, which counts refused requests too), banner names it · `A2` default options = Part 1 exactly (three spellings driven bit-identical) · `A3` every new refusal, verbatim, from validate() AND the constructor (same sentence), refused before any device is asked for; one-member side; X-drive list; another dead port still refuses; a repeated entry |
| 3 (degraded on the plant) | same | `A4` three degraded shapes drive, keep an honest estimate, report honestly, raise no `MOTOR_GROUP_DISAGREE`, never request the dead ports, and differ from the healthy twin |
| 5 (portMap) | same | `A5` byte-identical to a TRANSCRIPTION of `tankPortMapString()` for every side width 1..21, TYPED and MEASURED; tank GPS/pods; X-drive = `portMapString()`'s format; H-drive; UNSET; absence not written into the map · `A6` truncation: one 21-port side capped at 39 chars, both sides 21 ⇒ the FIELD bound stops at 95 + NUL with a canary intact; caps 0 / null / 1 / 10 · `A7` `driveOdometry()` |
| 4 + 5 (loop) | `test/teleop_loop_test.cpp` (new) | `G1 2a loop 4a` step at tick k in the volts of tick k (and the release) · `4b` disabled ⇒ 0 V + Brake at the motors and return; disabled on tick 0 reads nothing · `4c` disconnected ⇒ bit-zero command from a controller that LIES while disconnected, zero reads while disconnected, all four axes read once per connected tick · `4d` re-zero once per press with that tick's heading, governing the same tick · `4d'` the non-finite re-zero (class half + loop half) · `4e` untrusted ⇒ fellBack, unrotated, latched after trust returns · `4f` tank sticks: RightY drives the right side · `4g` the period reaches the shaper; always Body (a robot facing +Y drives forward); `DriverLoopConfig{}` and a NaN period never run · `5` default profile vs `mapSticksToChassisSpeeds` over a 603-tick sweep of three axes: bit-identical at 0 and ±1 (9 points), elsewhere the gap equals `budget·d·(1−|x|)/(1−d)` to 1e-9 and never exceeds `budget·d` |

**Suite:** `1385 test cases / 1,638,655 assertions, 3 skipped — SUCCESS` (baseline 1358 / 1,635,266).
New cases: +27 / +3,381. **Pre-existing cases: +8 assertions, every one in the PROS fence file, and
no other pre-existing case moved** (per-case XML diff against the baseline dump):

| Pre-existing case | Before → after | Why |
|---|---|---|
| `fence: every adapter's suppression set is EXACTLY …` | 116 → 118 | one more `hal/pros/*.hpp` scanned: `readFile`'s REQUIRE + the "no pragma in a file with no direct pros include" CHECK |
| `fence: closes BEFORE shulib code …` | 97 → 98 | the new file's `readFile` REQUIRE (then `continue`: nothing to scope-check) |
| `adapters: PROS includes are QUOTED …` | 36 → 38 | the new file's `readFile` + its CHECK |
| `only hal/pros includes <pros/> …` | 492 → 495 | `teleop/teleop_loop.hpp` is one more header outside `hal/pros/`: 3 × (183 − 18) = 495 |

Part 1's bit-identity cases (`G1 builder 5/5b/5c/5d/6/6b`), `stick_mapping_test.cpp`, and T2's
`driver_control_test.cpp` / `drive_modes_test.cpp` / `input_shaping_test.cpp` did not move by one
assertion.

**One measurement that changed a test, recorded because it matters at the bench.** Test 3 was first
written to require the degraded robot to SETTLE. It does not, on this plant: one dead member of five
on a side ⇒ the heading-held 30 in `moveTo` times out 4.7 in off target; two dead on the right ⇒ 10.5
in; even one dead PER SIDE misses the settle window (0.6 in off) and its turn times out (measured with
a scratch case, since deleted). Cause: `sim::DrivePlant` models a silent member as dragging the wheel
to the MEAN of its members' volts (HA-137) — 80 % speed — while the controller's feedforward assumes
full authority. For N−1 powered motors in a voltage-limited train that model is PESSIMISTIC (a real
train loses far less free speed), so this is a statement about the model and the provisional
feedforward, not about robot two. The case now asserts what IS the library's property — it drives
(> 20 in), its estimate stays within 1 in / 0.03 rad of the ground truth, it reports Settled or
TimedOut (never a fault-cancel), raises no false fight, never opens the dead ports, and is not the
healthy robot — and says in its comment why it does not assert a settle. **Worth the coordinator's
attention given the 01:39 census (16 and 17 empty, 14 silent): a robot with dead drive members is a
different robot to the closed loop, and nothing on the host has measured by how much.**

**The stale-object trap, met once already:** the first run after the `field_centric.hpp` edit went red
on the fix's own test because `--target shulib_tests/fast` skips dependency scanning and the linker
kept an OLD inline `FieldCentric::rezero` from an unrecompiled object (an ODR pick, not a test bug).
From here on every build before a result is trusted touches EVERY `test/*_test.cpp` (a superset of
"every file that includes the edited header") and requires the link line.

### 2a.4 Rulings beyond the brief (each with the alternative and the reason)

**G1-2a-R1 — the re-zero edge runs BEFORE shaping, in the same tick, on the same heading sample.**
The brief's §2a item 5 lists `drive(Body)` and then "the loop's OWN ButtonEdge … calls
`shaper.rezero(heading.radians)`". *Alternative:* re-zero after `drive()`, as listed. *Reason:* the
press is "the way the robot faces NOW is away from me", and applying it after the tick is shaped makes
the press tick still show `LatchedUntilRezero` and rotate by the old zero, one tick late. Sampling the
heading once and re-zeroing first gives the press the same one-tick-latency property as the sticks,
and it has a second, measured benefit: the same tick's `shape()` re-latches on a bad heading, so the
loop cannot carry a non-finite zero forward even without R6. The signature is exactly the brief's.
Pinned by `loop 4d` (tick 5's state is `Active` with zero = tick 5's heading) and mutation L4b below.

**G1-2a-R2 — the re-zero button is read only while connected, like the axes.** Not stated in the
brief. *Alternative:* read it every tick. *Reason:* a disconnected controller's readings mean nothing
(HA-103), and feeding `false` to the edge while disconnected is the honest reading; the one
consequence (a button held across a drop registers once on reconnect) is the `ButtonEdge`'s documented
behaviour for a held button. Pinned by `loop 4c` (`buttonReads == 8`, zero reads while disconnected).

**G1-2a-R3 — `DriverLoopConfig`'s defaults are deliberately NOT a working config.** `tickPeriod`
defaults to 0 and `runDriverControl` preconditions it finite and > 0 before the first tick;
`rezeroButton` defaults to `L1` and its documentation says "state it". *Alternatives:* a 10 ms
default (a second, silent copy of `ProsTickPacer::kTickMs` in a PROS-free header, and a wrong value on
any other pacer); a constructor (loses the designated-initializer spelling every config in the tree
uses); no default for the button (an aggregate member with no initializer is indeterminate under
default-initialization). A zero period is the "nobody wrote this config" detector: `DriverLoopConfig{}`
and a NaN period are refused before `disabled()` is ever polled (`loop 4g`).

**G1-2a-R4 — `writePortMap(cfg, out, cap)` is a public FREE function; `Robot::portMap()` forwards to
it.** *Alternative:* only the member the brief names. *Reason:* the brief's test "a 21-port side
truncated not overflowed" is not constructible through a `Robot` — 21 ports on one side plus one on the
other collide, and `valid()` refuses the config. A free formatter over the config (which validates
nothing and formats what is stored) is the honest way to test the bound the brief asks for, and it is
what `Robot::portMap()` IS. Documented as such in both places.

**G1-2a-R5 — the port-map FORMATS are the ones `src/main.cpp` prints today, byte for byte.** Tank:
`tankPortMapString()`'s (source word, `%+d` space-separated sides each capped at 39 chars, `IMU<n>`,
`ipr l/r`, `TW`), pinned against a verbatim TRANSCRIPTION of that function for every side width 1..21
(`A5`) and both 21-wide (`A6`). X-drive: `portMapString()`'s (`FL BL BR FR ROTF ROTL GPS IMU`). Two
additions, both chosen so robot two's text is unchanged: a tank's `GPS<n>` and pods `ROTF<n> ROTL<n>`
are printed only when configured and go between the IMU and the geometry (ports before scales, so
truncation eats the geometry first); an X-drive's `GPS<n>` is omitted when there is no GPS (the invented
graph always had one). H-drive, which has no printer today, follows the X-drive's shape (`L R S`).
*Alternative:* a new format (e.g. `describe()`'s comma lists). *Reason:* Part 2b swaps the generator;
every session header since R3b would change shape under whoever reads the logs.

**G1-2a-R6 — `FieldCentric::rezero()` no longer accepts a non-finite heading (a T2 defect, fixed where
it lives).** Found while wiring the button: `rezero(NaN)` stored a NaN zero and CLEARED the latch, so
the next trusted tick rotated by `heading − NaN` ⇒ a NaN request ⇒ `Chassis::drive`'s precondition ⇒
the teleop task dies, ticks after the press. Now a non-finite heading keeps the old zero and SETS the
latch (`LatchedUntilRezero` on the panel until a press on a real heading). This narrows T2's ruling
T2f ("rezero clears the latch unconditionally") for non-finite headings ONLY; every finite path is
unchanged (T2's tests did not move by one assertion). `constexpr` is kept — `std::isfinite` is not
constexpr in C++20, so the test is `x − x == 0`, commented. *Alternative:* guard only inside
`runDriverControl` — rejected by the standing rule (a chunk that finds a flaw in an earlier chunk fixes
it there, not around it), and because any other caller of `DriveShaper::rezero` would keep the hole.
`driver_control.hpp`'s `rezero` doc says so. Pinned by `loop 4d'` (class half: inf, −inf, NaN).

**G1-2a-R7 — the provider serves ONE IMU and ONE GPS; another port for either is a PRECONDITION.** The
brief's storage list says one `std::optional` each. *Alternative:* refuse (nullptr). *Reason:* it is not
a wiring fault the robot can have — a config has one IMU port and one GPS port — it is two parts of one
program disagreeing, which is what preconditions are for; same for a second motor request with another
sign or cartridge, or the GPS with another lever arm.

**G1-2a-R8 — the registry is consulted when a device is MADE, never again for it; a refusal is not
cached.** *Alternative:* re-check on every request. *Reason:* the builder asks twice (validate, then the
constructor's passes); a cable flicker between the two would turn the second ask into `wireMotors()`'s
"the provider refused a motor validate() had obtained" precondition — a boot crash out of a flicker —
and would break the idempotence contract. Runtime loss is the adapter's sentinel screening and the
health layer's. A refused port IS re-asked, so a port plugged in after the first look is served (`1h`).

**G1-2a-R9 — the provider owns a `ProsDeviceRegistry` by default and has a constructor that borrows any
`IDeviceRegistry`.** The brief allows either. Both exist: `ProsDeviceProvider devices;` is the brain's
two-line spelling, and the borrowing form lets a host test program a census without the shim (`1i`).

**G1-2a-R10 — each construction is wrapped in `catch (const std::exception&)`, not only
`PreconditionError`.** The adapters throw through `SHULIB_PRECONDITION`, whose handlers (the default and
`src/main.cpp`'s) throw `PreconditionError`, a `std::logic_error`. Catching the base keeps a future
handler that throws another standard exception from unwinding the boot; nothing broader (`...`) is
caught, so a non-standard throw still surfaces.

**G1-2a-R11 — the adapter's message reaches the refusal through an ASCII filter.** `ProsMotor`'s
messages carry an em dash; the brief says the refusal carries "the adapter's message" and the task says
refusals are plain ASCII (they reach a screen with ASCII-only fonts). A UTF-8 lead byte becomes `-`, its
continuation bytes are dropped, a control byte becomes a space; bounded by hand. *Alternative:* edit
`motor.hpp`'s messages — rejected: it would not protect against the next adapter, and the refusal is
where the screen-safety property belongs. Pinned by `1c`/`1d`.

**G1-2a-R12 — `FakeDeviceProvider` gains `motorRequestCount(port)`, counting REFUSED requests too.**
The brief: "make the fake provider count requests if it cannot already". It could say "served or not",
not "asked or not" — a builder that asked for an absent port, got a refusal, and ignored it would have
looked innocent. Counted before any refusal check.

**G1-2a-R13 — `BuildOptions::isAbsent()` is a member; a repeated absent port is accepted.** Membership,
not a count: `{4, 4, 4}` is one absent port (`A3`). Refusing duplicates would be a new rule about the
boot check's output that nothing needs.

**G1-2a-R14 — the empty-side refusal names the SIDE and lists its ports** ("every LEFT drive port is
absent (ports 1 2 3 4 5) -- a side needs at least one motor that answers"), rather than one port: no
single port is the cause. The other two refusals name the one port.

**G1-2a-R15 — `portMap()` does NOT mark absent ports; `describe()` does.** The port map is what the
robot is WIRED to be (and it must stay byte-identical to today's for 2b); a dead port is the banner's
and the panel's to announce (`DEGRADED: absent 3,13`, printed only when there is one, so a complete
robot's banner is byte-for-byte Part 1's — `A2`).

**G1-2a-R16 — test 3 does not require the degraded robot to SETTLE** (measured in 2a.3: the plant's
mean-of-members model is pessimistic for a voltage-limited train). It asserts drive, honest estimate,
honest report, no false fight, never-opened dead ports, and a run different from the healthy twin's.

### 2a.5 The mutation campaign — every run on a FRESH binary (provider and builder rows logged 02:15:52 EDT)

Harness (scratchpad `mutate.py`, not committed): back the file up → replace EXACTLY ONE anchored
occurrence (abort if the anchor matches 0 or 2+ times) → `touch` EVERY `test/*_test.cpp` → `cmake
--build build/test --target shulib_tests/fast` → **require `Linking CXX executable shulib_tests`**
(no link line = result not trusted) → run the named cases → print each red case with its first error
verbatim → restore from the backup → `cmp` the restored file against the backup (byte-identical, every
row). Rows below are the OBSERVED output, trimmed to the first error of each red case.

| # | Mutation (file) | Observed |
|---|---|---|
| P1 | **skip the registry check** — `expectKind()` always true (`device_provider.hpp`) — *the brief's* | **RED, 5 cases.** `1b`: `CHECK( devices.imu(2) == nullptr ) … values: CHECK( 0x7ffd9360e200 == nullptr )` — an EMPTY IMU port came back LIVE, exactly the audit's defect 2, and the same for the rotation, GPS and motor subcases (11 errors); `1c`: `CHECK( got == (raw == req.served) ) … CHECK( true == false ) logged: req.name := motor, raw := 0`; `1h`, `1i` (an empty rotation port served); `1k`: `CHECK_FALSE( v.ok )` — the robot with ports 16/17 empty VALIDATED |
| P2 | **return a fresh device per request** — the "already made" branch skipped, so the slot is re-emplaced — *the brief's* | **RED, 3 cases.** `1a`: `CHECK( pros::shim::motorState(11).setGearingCalls == gearingCalls ) … CHECK( 2 == 1 )`. **Worth recording: `CHECK(m2 == m1)` stayed GREEN** — re-emplacing a `std::optional` reconstructs in the SAME storage, so the address check alone cannot see a fresh object; the shim's configuration counter is what catches it (which is why 1a asserts both). Also `1e` (conflicts no longer detected), `1h` |
| P3 | **drop the conflict check** (motor sign/cartridge) — *the brief's* | **RED.** `1e`: `CHECK_THROWS_AS( (void)devices.motor(11, MotorCartridge::Blue), PreconditionError ) did NOT throw at all!` and the same for `(-11, Green)` |
| P4 | calibration never started by the provider | **RED.** `1f`: `CHECK( pros::shim::imuState(2).resetCalls == 1 ) … CHECK( 0 == 1 )`; `1k` the same |
| P4b | calibration started on EVERY request | **RED, 3 cases.** `1a`/`1f`: `CHECK( devices.imu(2) == i1 ) THREW exception: "ProsImu::calibrate: already calibrated — a second reset() would re-zero …"`; `1k`: the Robot's second pass threw |
| P5 | the ASCII filter dropped (bytes copied raw) | **RED.** `1d`: `CHECK( why.find("did not accept degrees - wrong port") … )` and `CHECK( isAscii(devices.lastRefusal()) ) … CHECK( false )`, logged why := `… degrees — wrong port …` (the raw em dash) |
| P6 | the registry re-asked on EVERY request (motor) | **RED, 3 cases.** `1h`: `CHECK( devices.motor(12, MotorCartridge::Green) == m ) … CHECK( nullptr == 0x7ffce8221b40 )` — a made motor refused after its cable came out; `1g`, `1i` (query counts) |
| B1 | **drop the options in `prechecked()`** (`(void)options; validate(cfg, devices)`) — *the brief's* | **RED, 5 cases — the constructor throws, as the brief predicted.** `A1`: `REQUIRE_NOTHROW( robot.emplace(c, world.devices, options) ) THREW exception: "a LEFT drive motor cannot be made: port 3 has no motor"`; `A3`: `Robot's constructor accepted an absent list validate() had refused` (logged `absent port 7 …`); `A4`, `A5`; `1k`: `THREW … port 17: nothing plugged in (expected a motor)` |
| B2a | **request absent ports anyway** — `wireMotors()` asks for the absent port and ignores the answer — *the brief's* | **RED, 3 cases.** `A1`: `CHECK( world.devices.motorRequestCount(3) == askedWithoutList ) … CHECK( 2 == 1 )`; `A3` (`motorRequestCount(4) == 0` → `1 == 0`); `A4` ×2. Only the new counter can see this variant: the port was refused, so `motorRequested()` stayed false and nothing threw |
| B2b | the same in `validate()`'s device pass (LEFT side) | **RED, 4 cases.** `A1`: `REQUIRE( v.ok ) … REQUIRE( false ) logged: v.why := a LEFT drive motor cannot be made: port 3 has no motor`; `A3`, `A4`, `A5` |
| B3 | a non-drive absent port accepted | **RED.** `A3`: `CHECK_FALSE( v.ok ) … CHECK_FALSE( true ) logged: k.label := a pod port, not a drive port` (13 errors across the six non-drive cases) |
| B4 | an emptied side accepted | **RED.** `A3`: `CHECK_FALSE( v.ok ) … logged: k.label := the whole LEFT side` (66 errors: the MotorGroup's own precondition then fires with a DIFFERENT sentence, and devices were asked) |
| B5 | a list on an X-drive accepted | **RED.** `A3`: `CHECK( std::string{v.why} == "absent drive port 2 was given for a x-drive drivetrain -- …" ) … values: CHECK( absent port 2 is not one of this drivetrain's drive ports … == …)`. Note: the build is still REFUSED under this mutant — by the next check, with a misleading sentence (the X-drive's unused tank slot has no ports) — so the MESSAGE check is the detector, not the bool |
| B6 | the `DEGRADED` segment printed on a complete robot | **RED.** `A2`: `CHECK( std::string{ba}.find("DEGRADED") == std::string::npos ) … CHECK( 70 == 18446744073709551615 )` |
| L1 | **a one-tick buffer** — `drive(pending); pending = command;` (`teleop_loop.hpp`) — *the brief's* | **RED, 2 cases.** `4a`: `CHECK( firstMoved == kStep ) … CHECK( 6 == 5 )`, `CHECK( log[kStep].leftVolts > 1.0 ) … CHECK( 0 > 1 )`; `4g` |
| L2 | **skip the cancel** on disable — *the brief's* | **RED.** `4b`: `CHECK( c.rig.h.motor(0).commandedVoltage().value() == 0.0 ) … CHECK( 11.4 == 0 )` (and the right side, and both brake modes) |
| L3 | **poll the axes while disconnected** — *the brief's* | **RED.** `4c`: `CHECK( pad.readsWhileDisconnected == 0 ) … CHECK( 20 == 0 )`, `CHECK( pad.reads[a] == 8 ) … CHECK( 12 == 8 )`. **Worth recording: the four bit-zero command checks stayed GREEN under this mutant** — the shaper short-circuits on `connected = false` whatever it is handed, so only the lying, counting controller can see a loop that reads a dropped controller |
| L4 | **never read RightY** — *the brief's* | **RED, 2 cases.** `4f`: `CHECK( log[3].rightVolts > 1.0 ) … CHECK( 0 > 1 )`; `4c`: `CHECK( pad.reads[a] == 8 ) … CHECK( 0 == 8 ) logged: a := 3` |
| L4b | the re-zero AFTER `drive()` — the brief's LISTED order (ruling R1) | **RED.** `4d`: `CHECK( log[5].state.fieldCentric.reason == FieldCentricReason::Active ) … CHECK( 2 == 0 )` — the press tick still `LatchedUntilRezero` |
| L5 | re-zero on the button LEVEL | **RED.** `4d`: `CHECK( log[8].zero == doctest::Approx(0.5) ) … CHECK( 0.8 == Approx( 0.5 ) )` — the zero chased the heading while held |
| L6 | the trust verdict dropped (`true` always) | **RED, 2 cases.** `4e`: `CHECK( log[k].state.fieldCentric.fellBack ) … CHECK( false ) logged: k := 0`; `4d` |
| L7 | the shaper handed dt = 0 instead of the period | **RED.** `4g`: `CHECK( log[0].state.slewOutput.forward == doctest::Approx(period) ) … CHECK( 0 == Approx( 0.01 ) )` and `( 0.02 )` |
| L8 | `drive(…, Frame::Field)` | **RED — but only after the test was strengthened.** First run (5 ticks, "> 1 V"): `CHECK( t.leftVolts > 1.0 ) … CHECK( 0 > 1 )` on ONE tick only — the boot-window tick. Measured why (scratch case, deleted): after the 0.1 s settle window a Field-frame +X at a 90° heading still puts **kS = 1.2 V on both sides**, because the rotated forward component is cos 90° × 60 ≈ 4e-15 in/s rather than exactly 0 and kS is gated on the sign of the commanded wheel speed. The subcase now runs 20 ticks (past the settle window) and requires > 5 V (full stick in Body is ~11 V). Re-run: `CHECK( driven == 20 ) … CHECK( 0 == 20 )`. **The kS-on-a-rounding-residual behaviour is a pre-existing pipeline property, not this part's; reported, not chased.** |
| L9 | the tick-period precondition dropped | **RED.** `4g`: `CHECK_THROWS_AS( runDriverControl( … DriverLoopConfig{} … ), shulib::PreconditionError ) threw a DIFFERENT exception: "PlantPacer: pace cap exceeded — a wait failed to end"` (the unconfigured loop ran 200,000 ticks) and the same for the NaN period |
| R6 | `FieldCentric::rezero`'s non-finite guard removed (`field_centric.hpp`) | **RED.** `4d'`: `CHECK( fc.zeroRadians() == 0.7 ) … CHECK( inf == 0.7 ) logged: bad := inf`, `CHECK( fc.fellBack() ) … CHECK( false )`, plus the loop half's `CHECK( t.zero == 0.25 )`. T2's 38 cases (`-tc="T2*"`) stayed green under it — T2's suite never covered a non-finite re-zero |
| L4b+R6 | BOTH: the brief's listed order WITHOUT the class fix (run by hand, both files restored and `cmp`-confirmed) | **RED, and the crash is real:** `4d'`: `CHECK_NOTHROW( log = runTicks(…) ) THREW exception: "Chassis::drive: speeds must be finite"` — the teleop task would die ticks after a NaN-heading press. With either R1's order or R6's fix alone the loop is safe; both are kept |
| M1a | **drop the bound** in `writePortMap()` — *the brief's* (run on the one subcase whose buffer is large enough to overflow INSIDE the test's array; the other subcases' 4- and 16-byte buffers would have been genuine stack overruns) | **RED.** `A6` "both sides at 21 ports": `CHECK( std::strlen(buf) == shulib::diag::kMaxPortMapBytes - 1 ) … CHECK( 124 == 95 )`, `CHECK( buf[i] == '\xAB' ) … CHECK( 49 == -85 ) logged: i := 96` (32 errors — the whole canary tail) |
| M1b | the bound off by one (`n < cap`: no room for the NUL) | **RED.** `A6`: `CHECK( … == 95 ) … CHECK( 96 == 95 )`, `CHECK( buf[i] == '\xAB' ) … CHECK( 0 == -85 ) logged: i := 96` |
| M2 | the side format `%+d` → `%d` | **RED, 2 cases.** `A5`: `values: CHECK( MEASURED L 1 -2 3 -4 5 R -11 12 … == MEASURED L +1 -2 +3 -4 +5 R -11 +12 … )` (44 errors — every width against the transcription); `1k` |
| M3 | the pods dropped from the map | **RED.** `A5`: `values: CHECK( FL1 BL2 BR3 FR4 GPS9 IMU10 == FL1 BL2 BR3 FR4 ROTF6 ROTL7 GPS9 IMU10 )` |
| B7 | absent slots COMPACTED out of the group instead of kept as holes | **RED, 2 cases.** `A1`: `CHECK( left.memberCount() == 5 ) … CHECK( 4 == 5 )`, `CHECK( left.absentMask() == … ) … CHECK( 0 == 4 )`; `A4`: the banner lost `DEGRADED: absent 3 |` (the absent ports are read back from the slots) |

**30 distinct mutations (7 provider, 8 builder, 10 loop, R6, 4 port-map) plus the L4b+R6
combination — 32 runs counting L8's weak first run and a first P1 run made before the harness printed
per-case summaries (also red); every one RED on a freshly linked binary (logged 03:06:47 EDT); every
restore `cmp`-identical.** Two came back WEAK and changed the tests: **P2** (the
address check alone could not see a re-emplaced object — the configuration counter did) and **L8**
(red only on the boot tick until the subcase ran past the settle window with a 5 V threshold). One
brief mutation is ALSO only visible through a counter: **L3** (and **B2a**), which is why the counting
controller and `motorRequestCount()` exist. After the last restore: `touch test/*_test.cpp`, relink
(`Linking CXX executable shulib_tests`), whole suite **1385 / 1,638,666, SUCCESS** (the +11 over §2a.3's
figure is `4g`'s Body subcase growing from 5 ticks to 20).

### 2a.6 The documentation pass and the prose review (03:18:38 EDT)

**The brief's §2a documentation list, done:**

| Document | What changed |
|---|---|
| `docs/api/` + `mkdocs.yml` | regenerated (`api_doc_tool.py generate`): two new pages (`pros-device_provider.md`, `teleop_loop.md`), and `robot.md`, `driver_control.md`, `field_centric.md`, `stick_mapping.md`, `device_provider.md`, `motor_group.md`, `session_info.md`, the index and the all-entities page moved with their headers. Coverage: **2339 public entities across 147 headers, all documented** |
| `docs/guide/08-your-first-routine.md` | new subsection "The same wiring, in two lines" after Step 2: a `RobotConfig` for the chapter's own X-drive, `robot::validate` + `robot::Robot` over a provider, the routine run on `built.chassis()`; what a device provider is (the host fake here, `ProsDeviceProvider` on a brain, which asks the registry first); the longhand kept and named as the Tier-3 path; an honest note that no `src/` program is built this way and nothing built this way has run on a robot. The listing is quoted VERBATIM from the new compiled case **`guide-08d`** in `test/guide_examples_test.cpp` (which also asserts the routine settles on the ground truth and that a refused port is named); the chapter's case list and the test file's mapping comment gained `08d`. `check-examples`: **408 quoted lines, all verbatim** |
| `docs/guide/10-the-api.md` | after the teleop-shaped `drive()` loop: a paragraph pointing at `teleop::runDriverControl` (what it does each tick, what is injected and why), honest status — prose only, no signature restated (the chapter's own rule) |
| `docs/changelog.md` | a dated **"Added since 2.2 (2026-09-24)"** entry under API 2.2: every addition, the `FieldCentric::rezero` behaviour change with "Migration: none", and why there is no bump (nothing frozen grows; `IDeviceProvider` and `Robot` are unfrozen); the 2.2 entry's "the PROS device provider is not built yet" sentence gained a dated parenthetical instead of being rewritten |
| `docs/roadmap.md` | "You are here": a **G1 Part 2a LANDED** paragraph (host-only, no program uses it, not one line of `main.cpp` shorter) and the **Now** paragraph moved to 2b-then-2c with the bench session still pre-empting; the table row A ("Part 2 … is next" → 2a DELIVERED, 2b/2c next); the M5 WS9 bullet's OPEN clause; "the reciprocal half" paragraph's parenthetical; and the C5 session-header bullet's "G1's RobotBuilder generates it" gained a dated parenthetical (the generator exists, no program uses it until 2b) — the full rewrite of that sentence stays Part 2c's, as the brief lists |
| `docs/internal/build-order.md` | the first bold `**Next:**` is now **G1 Part 2b** (waits for the go; the bench session pre-empts), carrying the 2a landing; the previous block is kept as labelled history (its bold opener removed so the briefing tool still finds exactly one first `Next:`); G1's status line and its closing sentence; the phase table's G row; the "half built" risk line |
| `docs/hardware-assumptions.md` | **HA-141** registered — summary row, a full entry after HA-140 (claim, source with the `A4 register HA-141` tag now in `device_provider.hpp`, confidence **reasoned**, the settle step at 2b's first boot, blast radius "refuses rather than lies"), the status line "7 of **141** settled", the intro's per-part note, and the robot-two section's count (eighteen entries, seven reasoned). *First draft claimed a 2026-08-18 census had read the kinds correctly; I could not cite it, so it was cut to what `R3b-PROGRESS.md` §9 records (the IMU on port 2, 2026-09-10) and the census the coordinator logged today* |
| the briefing | regenerated LAST, after the final green build (§2a.7) |

**Documents touched OUTSIDE the brief's 2a list, each because 2a made a sentence in it false:**
`docs/vexbuilder-integration.md` (row 64: "*Part 2 (the PROS provider + `src/main.cpp`) is open*" —
the brief assigns this row to 2c, but its "the PROS provider is open" half became false today; one
cell, now "Part 2a (the PROS provider, host-tested only) landed 2026-09-24; Parts 2b/2c … are
open"); `docs/internal/HANDOFF-remaining-work.md` (§1's open-chunks line, §4's inventory of what
exists — 18 `hal/pros` headers, the loop, the builder's new members — L1's status line with the old
wording kept in a note, and §7 step 1; the handoff's own rule is that it is corrected in the commit
that notices it is wrong); `docs/internal/PROJECT-BRIEFING.md` hand-written half (the T and G rows of
the phase table and §13's heading and summary box — the generated block is the tool's).

**Headers whose COMMENTS changed because 2a made them false** (no code change in any of them except
`field_centric.hpp`, ruling R6): `hal/device_provider.hpp` (**the false sentence at :26**, "every
adapter's constructor throws on a dead port", replaced by what is true — only ProsMotor throws, the
provider asks the registry first, HA-141 — and the "(G1 PART 2)" table row); `robot/robot.hpp`
(:13-16 and :55-56 — the two-line sketch now names `hal::pros::ProsDeviceProvider` and says no
`src/` program uses it; the NO-HARDWARE paragraph); `hal/fake/fake_device_provider.hpp` (future
tense about the PROS provider, and `refusePort()`'s "stand-in for a ProsMotor constructor throwing on
an empty port" — the real provider refuses an empty port on the registry's answer); `diag/session_info.hpp`
(":47 G1 generates" and its banner note — the generator exists); `hal/motor_group.hpp` ("WHY BOOT":
it said the composition root catches the ProsMotor throw per port and hands the group a span with
holes — true of neither program that uses the group; now: the drive program catches and does not use
the group, Teleop's `TankMotors` and `robot::Robot` take the boot check's MISSING set);
`teleop/stick_mapping.hpp` (the doc comment on `mapSticksToChassisSpeeds` called it "what the library
teleop loop uses" — the library's loop now does NOT; comment only, the body and its pin untouched,
landmine 7); `teleop/driver_control.hpp` (`rezero`'s doc, R6).

**The prose review — the grep was RUN, and its hits read, before this paragraph was written.**
Pattern, over `docs/`, `README.md`, `include/` and `src/` (`.md`, `.hpp`, `.cpp`, `.h`):
`main\.cpp|TankRobot|runTeleopLoop|composition root|wiring|hand-wired|object graph|mapSticksToChassisSpeeds|deadband-only|Part 2|every adapter.s constructor throws`
(then a second pass for `ProsDeviceProvider|PROS provider|PROS device provider|device provider|runDriverControl|portMap()`
over the live docs). Every hit in a LIVE document was read:

- **README.md** (6 hits): lines 97/152 (the 2026-08-12 boot "constructed its whole object graph") —
  history, true; 238 "`src/main.cpp` is the complete, commented wiring example" — still true in 2a
  (2b rewrites that paragraph, per the brief); 265/268/282 — true. **Not touched** (2b/2c own README).
- **docs/guide/**: 03:244/384, 12:153/178, 13:38, 15:160 — generic uses of "wiring", true. 07:122
  (`src/main.cpp` wires the library) — true until 2c, which owns it. **14:331** ("the teleop loop in
  `src/main.cpp` and the bench drive program still call the older raw mapping") — still true of the
  PROGRAMS; guide 14 is 2b's, **not touched**. 08 — the new section's own hits, plus 50-53/156/167/217
  (the longhand is `src/main.cpp`'s shape) — still true.
- **docs/*.md**: faq 115, master plan 197/251 (the WHY document — not an executor's to amend),
  diagnostics-plan, legacy-command-vocabulary (legacy `main.cpp`), changelog 304/447 (history),
  docs/README 9 (history) — true. roadmap: 50 hits read; the stale ones are the five edits above; the
  rest are dated history (R1a's "deadband-only teleop" at :692, C7's rewire, M1/M2 clauses) and true.
  hardware-assumptions: HA-111 (:704-712, `src/main.cpp`'s invented map — 2c's), HA-112 (:736, the
  mapping `src/main.cpp`'s loop still calls — 2b updates its source lines), :963/1011/1061 (`src/`
  sources, unchanged), :2341 (the "builder will replace hand-kept port lists" row — 2c's) — all still
  true, **not touched**.
- **include/** (~150 hits): every "composition root" / "wiring" use is generic and true; the stale
  ones are the seven headers listed above. `robot.hpp:23/920` ("the same objects … as src/main.cpp's
  two hand-written graphs … TankRobot and X-drive Robot") — true while those graphs exist (2b/2c).
  `config/robot_config.hpp:71-72` ("the X-drive … must be expressible as data (Part 2)") — still a
  future statement (2c). `tick_pacer.hpp:5/27`, `char_sink.hpp:12`, `main.h` — history, true.
- **src/** (report only — `src/` is not this part's to edit, and it is byte-identical): no `src/`
  comment became FALSE, because `src/` describes itself and did not change. Two became AMBIGUOUS and are
  Part 2b's to replace: `src/main.cpp:1322` — the runtime banner "opcontrol(): the LIBRARY's teleop loop
  is live on robot two" means `runTeleopLoop`, while "the library's teleop loop" is now
  `teleop::runDriverControl`; and `src/main.cpp:1279-1281` ("The R1a teleop loop … is
  shulib::teleop::mapSticksToChassisSpeeds") — true of `runTeleopLoop`, which 2b deletes.
- **docs/internal/** chunk briefs, PROGRESS and COMPLETED logs (the bulk of the hit count —
  R3b-PROGRESS 67, the G1 Part 2 brief 53, SESSION-2026-09-18 40, …): records of what was true when
  written; **not edited**. The live internal documents (build-order, RESUMING, HANDOFF, the
  briefing's hand-written half) were read and corrected as listed.

### 2a.7 Verification — every gate, on the final tree (host build at 03:22:40 EDT; the rest immediately after)

Run in this order: `touch test/*_test.cpp` → `--target shulib_tests/fast` (link line seen) → whole
suite green → `briefing_status.py generate` (LAST, on that fresh green binary: suite 1,386 /
1,638,673, headers 183, HA 141 / next free HA-142, and the Next-up line now G1 Part 2b) → then the
full gated build and the rest:

| Gate | Command | Result |
|---|---|---|
| Host build + suite (03:22:40) | `cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests \| tail -6` | doc gates: coverage **2339 entities / 147 headers, all documented**; **408 quoted lines, all verbatim**; removability clean; **briefing current**; staleness audit self-test OK + clean. Suite **1386 cases / 1,638,673 assertions, 3 skipped — SUCCESS** |
| `src` build gate | `python3 tools/src_build_gate.py self-test && … check` | **self-test OK (33 detector cases); PASS — all 6 builds link** (`bench`, `xdrive`, `tank`, `tank-drive`, `tank-library`, `tank-calibrate`: exit 0, 4/4 TUs, 0 disallowed warnings, beacons checked — unchanged, as `src/` is) |
| ARM header gate | the RESUMING/HANDOFF command, to a scratch TU | **183 headers, exit 0** under `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror -Os`, cortex-a9 |
| PROS-free guard | `grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib \| grep -v '^include/shulib/hal/pros/'` | **empty** (and `teleop_loop.hpp` / `robot.hpp` include no `pros/` and no `shulib/sim/`) |
| sim guard | `grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib` | **empty** |
| API reference | `api_doc_tool.py generate && check-fresh`; `briefing_status.py check`; `check-removability` | fresh (exit 0); briefing current (exit 0); removability clean (exit 0) |
| `src/` untouched | `git diff --stat -- src Makefile tools .github` + `git status --short` on the same paths | **empty** — byte-identical |

**Final suite accounting** (per-case XML diff against the baseline dump): **1358 → 1386 cases (+28),
1,635,266 → 1,638,673 assertions (+3,407)** = 28 new cases (+3,399: 11 provider, 7 builder, 9 loop,
and `guide-08d`) + **8 in four pre-existing fence cases** (§2a.3's table: one more `hal/pros/` header
scanned, one more header outside it). No other pre-existing case moved; none disappeared.

### 2a.8 DoD — brief §5 "2a", with evidence

- [x] **§2a item 1 — `ProsDeviceProvider`:** `include/shulib/hal/pros/device_provider.hpp` (439
      lines). Fixed storage, no heap (`std::optional` per port); registry asked before ANY
      construction (`expectKind`); the four refusal shapes; per-construction `try`; one object per
      port; conflicts are preconditions; IMU calibration started once; `hal/device_provider.hpp:26`'s
      false sentence corrected. Evidence: cases `1a`–`1k`, mutations P1–P6.
- [x] **Item 2 — its tests through the shim:** `test/pros_device_provider_test.cpp` (EMPTY IMU /
      rotation / GPS / motor, every wrong kind, the read-back failure with the adapter's message,
      identity, the conflicts, calibrate-once, and the end-to-end `Robot` over the real provider).
- [x] **Item 3 — absent drive ports:** `robot::BuildOptions`; `validate(…, options)` and
      `Robot{…, options}` with the options reaching `prechecked()` AND `wireMotors()` (mutation B1 is
      the audit's defect 1, red); never requested (B2a/B2b); the three refusals naming the port
      (B3/B4/B5); no policy; default = Part 1 exactly (`A2` + Part 1's untouched bit-identity cases);
      `describe()` names absent ports (B6, B7).
- [x] **Item 4 — `portMap()` and `driveOdometry()`:** byte-identical to a transcription of
      `tankPortMapString()` for every side width, X-drive in `portMapString()`'s format, bounded by
      hand (M1a/M1b), always terminated; `driveOdometry()` (`A7`).
- [x] **Item 5 — `runDriverControl`:** `include/shulib/teleop/teleop_loop.hpp` (167 lines),
      PROS-free, the brief's exact signature; one ordering ruling (R1, re-zero before shaping).
      Evidence: loop cases `4a`–`4g`, `5`; mutations L1–L9, L4b, R6 and the L4b+R6 combination.
- [x] **Item 6 — tests 1–5:** §2a.3's table.
- [x] **Every public member documented; ARM gate green; PROS-free and sim-free guards:** §2a.7.
- [x] **Mutations logged red:** §2a.5 — 30 distinct, every one red on a freshly linked binary.
- [x] **Pre-existing suite unchanged except stated additions:** +8 assertions, four fence cases,
      explained line by line.
- [x] **`src/` byte-identical:** §2a.7.
- [x] **§2a's documentation:** §2a.6 (plus three documents outside the list, each named with why).
- [x] **The prose review:** §2a.6 — the grep run and its live hits read before it was written up.
- [x] **The log:** this section.

### What is NOT done, stated plainly

- **No program uses any of this.** `src/` is byte-identical; `src/main.cpp` is not one line shorter;
  "shulib Teleop" still wires its graph by hand and still runs `runTeleopLoop` over the raw mapping.
  That is Part 2b (and `main.cpp` as a dispatcher, Part 2c) — each waits for the team lead's go.
- **NO HARDWARE CLAIM.** `ProsDeviceProvider` has been exercised only through `test/pros_shim/`,
  which tests it against our BELIEF about PROS (HA-138's indexing, HA-141's kinds) and cannot test
  the belief. `runDriverControl` has driven only the A2 host plant; no driver has felt it. The
  library's motion stack has still never driven a robot.
- **The degraded-robot plant finding (§2a.3)** is a statement about the plant's pessimistic
  mean-of-members model, not about robot two — nothing measures a real train with a dead member yet.
- **The kS-on-a-rounding-residual observation (mutation L8)** — a Field-frame command perpendicular to
  a tank's heading yields ±kS (1.2 V) on both sides — is a pre-existing command-pipeline behaviour,
  reported here and NOT investigated or changed.
- **HA-141 is unsettled** — its settle step is 2b's first boot of "shulib Teleop" (IMU present, then
  pulled).
- **Nothing was committed, staged or pushed; no branch or worktree was created.** The coordinator's
  concurrent bench-session files (`R3d-PROGRESS.md`, `docs/internal/evidence/r3d-session-2026-09-24-*.log`)
  are in the working tree and are NOT this part's.

---

## Part 2b — 2026-09-26 00:32:44 EDT

> **Part:** G1 Part 2b — "shulib Teleop" rebuilt on the library (`src/boot_source.{hpp,cpp}`,
> `src/teleop_program.{hpp,cpp}`, the shared screen-text header, `main.cpp`'s tank-library branches
> forwarding). **Tree at start:** `b622d9b` (branch `shulib-v2`, clean). **Executor model:** opus.
> **Brief:** [`G1-part2-pros-provider-and-main.md`](G1-part2-pros-provider-and-main.md) §2b with the
> BINDING amendments A1-A19, §3, §4 rows 6 (A11) and 7, §5 DoD 2b (+A14), §6 landmines 1-11, §7.
> Appended in real time; every timestamp from a `date` call. Nothing here is committed, staged or
> pushed by the executor; `pros` is never run and `/dev/ttyACM*` never touched. No hardware claim for
> THIS version (landmine 8).

### 2b.0 Log created (00:32:44 EDT)

Log section created before any code. Next: the required reading, then the clean-tree baseline (A17).

### 2b.0a The first executor run STOPPED here (recorded 01:16:32 EDT)

**What happened, stated plainly:** the first executor for this part (launched 00:32 EDT) wrote the
section header and 2b.0 above and then stopped: its report was null, and between 00:32:49 and
01:14:59 nothing in `src/`, `include/`, `test/`, `tools/`, `Makefile` or `common.mk` changed (three
read-only reviewers measured it: `git diff --numstat` showed only this file, `src/main.cpp` stayed at
1,340 lines, no `src/boot_source.*` / `src/teleop_program.*` existed). **No code, test or document of
Part 2b was written by that run.** The `build/test/shulib_tests` binary dated 00:31:41 predates this
section and is not a logged baseline (whoever built it, its count is not recorded anywhere; 2b.1
measures the baseline afresh).

**This run** (a second executor, started 01:16 EDT on the same tree: `b622d9b` + the 17 lines above)
continues the SAME section rather than opening a second "Part 2b" header, so the section stays one
true record. Everything below 2b.0a is this run's.

**The team lead's go, as the tree and the coordinator's session record it:** on 2026-09-26 at
00:21 EDT the team lead answered the coordinator's four Part 2b questions — commit the 2026-09-25
log (`4611b22`), "Yes, switch" (A6), "Yes, just the status lines" (A7), "Accept it" (A16, the
documentation list's size) — and the coordinator launched this part at 00:32 EDT on those answers
and told him so ("Part 2b is now being built"); at 00:40 he answered "we have to get auton working
legitimately we arent going to rush" without objecting to the build. **No separate sentence of the
form "go on 2b" is recorded in the tree** (Part 2a's header carries "The team lead's go:
2026-09-24"). The coordinator confirms or corrects this before the commit.

**CONFIRMED (second review round, 2026-09-26, 2b.9):** the coordinator's task to the run that
answered the second review states "the team lead confirmed the go for 2b (2026-09-26)". His own
words are in the coordinator's session, not in the tree; HANDOFF L1 now records the confirmed go.

### 2b.1 Required reading, and the clean-tree baseline (A17) (01:35:16 EDT)

**Read in full before any code:** `RESUMING.md` (the standing rules; pace rules 1-8); the brief
(§0, §1, §2b with A1-A19, §3 M1-M5, §4 rows 6-7, §5, §6 landmines 1-11, §7); `R3d-PROGRESS.md`
"2026-09-25 evening"; the capture `docs/internal/evidence/r3d-session-2026-09-25-teleop.log.gz` (`zcat
| strings`, the first boot block, lines 76-106); `src/main.cpp` (all 1,340 lines);
`src/calibrate_program.cpp` :1-620 (the header, `printText`, the literal table); `src/chassis_table.hpp`;
`src/bench_r3a.cpp` :225-260 (its own sanitizer); `robot/robot.hpp`; `config/robot_config.hpp`;
`hal/pros/device_provider.hpp`; `teleop/teleop_loop.hpp`, `driver_control.hpp`; `calibration/boot_check.hpp`,
`calibrate_panel.hpp` :1-200 and `wrapWords`/`screenText`; `hal/drivetrain_description.hpp`;
`motion/motion_config.hpp`, `odo_stall_check.hpp` :100-200, `motion_scheduler.hpp` :220-252;
`diag/term_sink.hpp` (the 200-byte cut), `line_format.hpp` (`appendSanitized`), `session_info.hpp`;
`hal/fake/fake_device_provider.hpp`, `fake_device_registry.hpp`; `Makefile` :1-130; `tools/src_build_gate.py`
:1-260; `HANDOFF-remaining-work.md` §3.

**Baseline on this tree (`b622d9b` + this log section), measured by this run:**
`cmake --build build/test` (full rebuild, `Linking CXX executable shulib_tests`), then the suite:
**1425 test cases / 2,386,871 assertions, 3 skipped — SUCCESS**, exactly A17's number. `src/main.cpp`
= 1,340 lines (A17).

### 2b.2 The code (01:41-01:52 EDT)

Files, with before/after line counts (`wc -l`):

| File | Before | After | What |
|---|---:|---:|---|
| `src/main.cpp` | 1,340 | 722 | the tank-library branches of `initialize()`/`autonomous()`/`opcontrol()` forward; `TankMotors`, `TankRobot`, `tankChassisConfig`, `tankPortMapString`, `paintRefusal`, `drawHeader`, `TankPanel` and the boot block LEFT (425 + 208 lines removed); `attachFaultLatch` defined for every build; the X-drive graph, `runTeleopLoop`, `robot()`/`portMapString()` STAY (§2b item 3); stale header comments rewritten (A1, A4) |
| `src/boot_source.hpp` | — | 222 | NEW: the pure core `bootConfigFrom()`, `BootConfig` (absent ports behind `absent()`/`buildOptions()`), `countPortDisagreements()`, `BootOutcome` |
| `src/boot_source.cpp` | — | 166 | NEW: `decideBootSource(tag)` — the moved boot block + cross-check, printing as it decides (A9) |
| `src/teleop_program.hpp` | — | 51 | NEW: `teleopInitialize(beacon, bits)`, `teleopAutonomous()`, `runTeleop()`, `kTeleopAutonomousReason` |
| `src/teleop_program.cpp` | — | 777 | NEW: the program (see below for the composition count) |
| `src/screen_text.hpp` | — | 110 | NEW: `printText()` + `formatOf()` + `g_screenPrints`, moved VERBATIM (A13) |
| `src/fault_latch_hook.hpp` | — | 29 | NEW: `attachFaultLatch()` declared (A12) |
| `src/calibrate_program.cpp` | 1,381 | 1,335 | `printText` block removed (57 lines), `#include "screen_text.hpp"`, header comment names the new home; the "HAS NEVER RUN ON HARDWARE" paragraph corrected (prose review, 2b.5) |
| `src/chassis_table.hpp` | 429 | 442 | `tableToDescription()`'s provenance built by a hand-bounded copy (finding F-2b-1 below) |
| `src/bench_r3a.cpp` | 3,244 | 3,248 | A13: the sanitizer's stated reason rewritten to name `src/screen_text.hpp` and why the tester keeps its own (no code change) |

**Rulings made while building (each with the alternative and why):**

- **G1-2b-R1 — the absent ports are PRIVATE storage behind one door.** `BootConfig` keeps
  `absentPorts_`/`absentCount_` private; they leave only as `absent()` (a span bounded to the count) or
  `buildOptions()` (`{.absentDrivePorts = absent()}`). The brief's §2b item 1 lists them as public
  fields; A9 says the outcome "exposes `absent()`, used everywhere". *Alternative:* public fields plus a
  comment at each call site (the brief's original spelling) — rejected: the hazard (a span over the whole
  array hands `validate()` the zeros, "absent port 0") would then be one careless line away in every
  program that builds a Robot (Teleop now, COMP next), and the host build cannot see `src/*.cpp` call
  sites at all. `teleop_program.cpp` calls only `buildOptions()` (grep in 2b.3). *(Overstated as
  first written — the second review round's finding T1, 2b.9: the program called `BootOutcome`'s
  own forwarders, which no test built, so the tested door was not the program's door; and
  `absent().data()` still hands out the pointer. Fixed by ruling G1-2b-R8: the forwarders are gone
  and the program spells `boot.built.…`.)*
- **G1-2b-R2 — the core sets `identity.name` from `kChassis.robot`** ("2026 TANK CHASSIS (robot two)" in
  the tank build), bounded to 31 characters; team and season stay empty, so `robot.describe()` prints
  `[]` for the team. *Alternative:* leave `(unnamed)` (A15 permits it) — rejected: the per-robot name
  already lives in the one per-robot file, and a banner that names the robot is worth a line.
  *Alternative:* a literal "Robot Two" in the program — rejected: a second spelling of the robot's name.
- **G1-2b-R3 — storage is ONE function-local static of EMPTY optionals** (`TeleopState`: provider, then
  Robot, then controller, LCD, shaper). Landmine 1 forbids building at namespace scope and forbids a
  static whose constructor can throw; empty optionals cannot throw, and the Robot is emplaced once from
  `teleopInitialize()`. Function-local rather than namespace-scope for a second, measured reason: every
  `src/*.cpp` compiles in all six builds, and a namespace-scope object with a non-trivial destructor
  registers it from a static initializer the linker must keep. Measured after the gate: the calibrate
  package carries no `teleopInitialize`/`runTeleop`/`decideBootSource`/`TeleopPanel` symbol and none of
  the strings `TELEOP REFUSED AT BOOT`, `built on robot::Robot`, `CROSS-CHECK: the MEASURED` (all 0 hits);
  the tank-library package carries each.
- **G1-2b-R4 — the re-zero button is `A`.** No ruling picks one; the host tests use `A`; it is harmless in
  ArcadeBody. A named constant (`kRezeroButton`), stated in the boot line; the team may change it.
- **G1-2b-R5 — every long serial line is either a literal `static_assert`ed <= 200 bytes, or goes through
  `logWrapped()`** (split at spaces into <= 196-character pieces — `kLogPieceCols`; this line said 190
  until the second review round — the second and later prefixed `... `,
  via `calibration::wrapWords`), because `TermSink` cuts a message at 200 bytes (A10; four boot lines were
  cut on the brain on 2026-09-25). The library's `kNoIndependentStallSourceNote` (333 bytes; "332" until
  the second review round re-measured it) is printed in two pieces that way.
- **G1-2b-R6 — the refusal headline for anything the builder refuses is "the library graph REFUSED to
  build"**, with the builder's sentence (or the precondition's) as the reason. The old headline "an adapter
  REFUSED at construction" described a `ProsMotor` throwing inside `TankRobot`; the provider now catches
  that throw and names the port through `validate()`'s verdict, so the old words would be wrong.
- **G1-2b-R7 — the empty-record message.** Moving the boot block exposed that an OPEN but EMPTY record
  file printed `calibration record: NOT USED -- ` with nothing after the dash. The decision is unchanged
  (not decoded); `boot_source.cpp` now says `the calibration record /usd/shulib_drivetrain.bin is EMPTY (0
  bytes read) -- run shulib Calibrate`. The one message the move adds.

**Finding F-2b-1 (a flaw in an earlier chunk, fixed there — RESUMING "fix it there"):** the first
tank-library build failed the gate's policy with `-Wformat-truncation` in `src/chassis_table.hpp:333`
(`'%s' directive output truncated writing 197 bytes into a region of size 51`): `tableToDescription()`
formats `"TYPED TABLE (%s): %s"` into the description's 96-byte provenance, and with a constant table
the compiler can prove the truncation is certain. Until now the call was inlined only in `main.cpp`'s
library branch, where it happened not to warn; `src/boot_source.cpp` compiles in all six builds and
warned. Fixed at the root: the provenance is appended by hand, bounded by the buffer — the same bytes
the `snprintf` wrote (the text cut one short of 96, terminated). `test/chassis_table_test.cpp`'s
provenance checks (`TYPED TABLE`, the geometry provenance, the robot name) pass unchanged.

**Compile findings fixed before the first green build:** two static_asserts caught literals over budget
(`Adapter refused? Bench Tests (slot 3) diagnoses it.` was 51 SMALL characters -> "diagnoses."; the
Calibrate hint did not wrap into two SMALL rows -> "the record Teleop boots from."); one boot line was 214
bytes (the provisional-gains line — restored to the OLD line's own tail text, 190 bytes); `refuseWith()`'s
`"%s"` copy of a 320-byte verdict reason into a 192-byte `why` warned `-Wformat-truncation` -> precisions
`%.63s` / `%.191s`.

**The gate after the code (01:51 EDT):** `python3 tools/src_build_gate.py check` — all six builds exit 0,
**6/6 TUs compiled** each (four before: the gate globs `src/`), 2 allowed `-Wunused-function` (the
X-drive's `robot()`/`portMapString()`) in bench, tank, tank-drive, tank-library, tank-calibrate and 0 in
xdrive — the variant differential holds unchanged — 0 disallowed, hash and beacons asserted, PASS.

**Composition size (§0's measure, a 2c target, reported now; measured 02:29 EDT as CODE lines —
no comment or blank lines — with `awk` over the function ranges):** `teleop_program.cpp` is 777 lines,
583 of them code. The composition proper — the driver-control constants and `teleopProfile()` (6),
`TeleopState` + `state()` (15), `headingSample()` (7), `teleopInitialize()` / `teleopAutonomous()` /
`runTeleop()` (87) — is **115 code lines**. `logBootLines()` (the A10 boot lines the brief requires kept)
adds 74, and the boot-line literals and their static_asserts ~40 more; the rest is the panel and the
refusal screen. So §0's "composition <= 120" holds for the composition proper and does NOT hold if the
kept boot lines are counted as composition — stated both ways, not rounded in either direction.

### 2b.3 Test 6 (A11, and the review's findings), and its mutations (01:55-01:59 EDT)

`test/boot_source_test.cpp` (NEW, 10 cases / 227 assertions — "226" here until the second review round:
that was the count before 6c's `describe()` golden was added, which happened AFTER M1-M12 below had run;
the second round re-ran the whole campaign on the final file, 2b.9), including `src/boot_source.hpp` by relative
path the way `chassis_table_test.cpp` includes `src/chassis_table.hpp`:

| Case | Property |
|---|---|
| 6a | the census rebuilt from the capture reproduces the robot's own 2026-09-25 port check, WHOLE (`OK: all 8 recorded drive ports carry motors, IMU on port 2. NEW motors not in the calibration: 14. NOT CHECKED: ...`, capture :83-84) — so the fixture IS robot two |
| 6b | config valid; identity; tank; the description carried field by field (source MEASURED, every signed port incl. the unused tail, both ipr and the width with `==`, provenance); drive-encoder odometry; no GPS; no mechanisms; `tuning` == `ChassisConfig{}` in EVERY leaf, longhand (incl. `motion.stall.*` and every scheduler field), with a host `sizeof` pin (368 / 48 / 416) so a new field fails until listed; nothing absent |
| 6c | `robot.portMap()` == the capture's `[SES] ports` literal (:94, 76 bytes < the 96-byte field); the description's `describe()` == the capture's :95 text |
| 6d | the built Robot's `driveOdometry()` scales and width `==` 1.25442 / 1.36703 / 12.006; 4+4 members, 4+4 present (:96); the facade budgets/gains formatted as the program formats them == the capture's :105 text; `kV == 12/70`; the stall wiring by the BUILDER (no independent source, every slot the left geometry); no GPS corrector |
| 6e | port 13 (RIGHT, index 2) EMPTY -> Degraded; `absent()` == {13} (which, not just how many); `buildOptions()` carries exactly it; `validate()` passes with the fake refusing port 13 as the brain would; the Robot constructs with right 3/4, left 4/4; port 13 never requested; `describe()` has `DEGRADED: absent 13` |
| 6f | 18 (left, stored -18) and 12 (right, stored -12) EMPTY -> `absent()` == {18, 12}: magnitudes, left then right |
| 6g | a REFUSED verdict (IMU unplugged) -> the core throws `PreconditionError` |
| 6h | a TYPED description stays TYPED (source carried, port map `TYPED L -11 +12 ...`) |
| 6i | `countPortDisagreements(record, robot two's table)` == 10, as the robot printed (:100); symmetric; 0 against itself; 1 for one flipped sign; the ends of the range (ports 1 and 21) count |
| 6j | the robot's name is bounded to 31 characters; an empty name stays empty |

**Mutations — each RUN on a fresh binary** (scratchpad `mutate.py`: edit -> touch the test -> `cmake
--build build/test --target shulib_tests/fast` -> REQUIRE `Linking CXX executable shulib_tests` -> run
`-tc="G1 2b*"` -> restore -> relink -> green), every one observed RED and every restore relinked green:

| # | Mutation (in `src/boot_source.hpp`) | Killed by |
|---|---|---|
| M1 | `absent()` spans the WHOLE array (A11's "pass the whole absentPorts array") | 6b (`absent().empty()`), 6c (`validate()`), 6d (the constructor THREW "absent port 0 is not one of this drivetrain's drive ports"), 6e, 6f — 5 cases red |
| M2 | the core reads the LEFT side twice (the review's copy-paste class) | 6e (size 1 -> 0), 6f |
| M3 | the core records the SIGNED entry, not the magnitude | 6f (`-18`, `-12`; `validate()` refuses) |
| M4 | the core ports `tankChassisConfig()` (`stall.setAllWheels(left)`, `independentMotionSource = false`) | 6b ONLY, at `requireDefaultTuning`'s `stall.wheels[i].wheelRadius` — `effectiveTuning()` hides it from every Robot-level check, as the review said |
| M5 | a non-default `kV` (0.2) | 6b (`wheelFf.kV`), 6d (the :105 golden and `kV == 12/70`) |
| M6 | the LEFT scale copied into the right | 6b (`requireSameDescription`), 6c (both goldens), 6d (`rightInchesPerRadian() == 1.36703`) |
| M7 | the left scale round-tripped through `float` | 6b, 6d (`==`), incl. every stall slot; the error is 3.4e-8 relative (1.25442 -> 1.2544200420379639), 350x below doctest's default `Approx` epsilon (1.19e-5) — Approx would have passed it |
| M8 | the source hard-coded MEASURED | 6h |
| M9 | `buildOptions()` returns `{}` (the absent ports dropped at the door) | 6e, 6f (`validate()` asks the provider for the refused port) |
| M10 | the `usable()` precondition dropped | 6g |
| M11 | the cross-check stops at port 20 | 6i (the port-21 case) |
| M12 | the core picks tracking-wheel odometry | 6b (`valid()`), 6c, 6d, 6e, 6f, 6h |

**`src/teleop_program.cpp` spells the options only as `st.boot.buildOptions()`** — `grep -n
"absentPorts\|absentCount" src/teleop_program.cpp` prints nothing; the storage is private, so a program
cannot spell the hazard (the review's finding F4; its disposition is in 2b.8). *(Overstated — see
2b.2's R1 note and 2b.9, finding T1: since the second round the spelling is `st.boot.built.buildOptions()`,
the door test 6 goes through.)*

**The A13 move, shown.** `printText` / `formatOf` / `g_screenPrints` / the budget static_assert, before
(`git show b622d9b:src/calibrate_program.cpp`, lines 240-296) against after (`src/screen_text.hpp`):
`diff` shows exactly three changed code lines, each gaining the `inline` a header needs (`formatOf`,
`g_screenPrints`, `printText`), and one comment clause (`(header)` -> `(each program's own serial record
says where)`); the `printText` body after its signature is byte-identical (md5 `3ecaabd1...` both sides).
`git diff src/calibrate_program.cpp` has four hunks: the run-status paragraph (:14, the prose review's
edit, 2b.5), the header comment (:44), the `#include` (:124) and the removed block (:240-296) — no call
site changed. (This line said "three hunks" until the second review round.) `g_screenPrints` is now `shulib::bench::g_screenPrints`
and Calibrate's one reader (`emitLoopStats`) finds it by the same unqualified name; Bench Tests'
`g_screenPrints` lives in its own anonymous namespace and `bench_r3a.cpp` does not include the header, so the
two cannot clash (said in both files).

### 2b.4 The banner diff, three-way (A10) (02:02 EDT)

There is no hardware to run the new program on, so the NEW column is the source, and the new boot block
was DERIVED from it for robot two's 2026-09-25 record (scratchpad `newboot.py` mirrors every format and
`logWrapped()`'s split): the message lengths below are the new ones. *(Second review round: `newboot.py`
was a HAND-TYPED copy of the strings and was not re-run after `teleop_program.cpp`'s last edit at 02:29,
so row 22's length was stale. Every length below was re-derived MECHANICALLY from the final source —
literals extracted by name, formats and the split re-applied — by `r2/lens2.py` (2b.9); row 22 is
corrected, every other length reproduced.)* **Old** = `src/main.cpp` at `b622d9b`;
**capture** = `zcat docs/internal/evidence/r3d-session-2026-09-25-teleop.log.gz | strings`, first boot
block (lines 76-106 — 26 distinct `[R3B]`/`[SES]` lines, counted from that output); **new** = file:line.
"CUT" = the capture shows TermSink's 200-byte cut on the brain. Status: **SAID** (still said, maybe moved),
**REWORDED** (with the evidence that made the old words false), **SPLIT** (said whole now, in <= 200-byte
pieces), **DROPPED** (deliberately).

| # | Capture line | Old source | New source | Status |
|---|---|---|---|---|
| 1 | :76 `booted: shulib TELEOP build [shulib-robot-variant=tank-library] -- THE LIBRARY drives robot two through` | main.cpp:969 | teleop_program.cpp:676 (beacon passed from main.cpp's branch, A2) | SAID |
| 2 | :77 `Chassis::drive() over two hal::MotorGroups and drive-encoder odometry, built` | :970 | :677 `robot::Robot (G1 Part 2b): two hal::MotorGroups, drive-encoder odometry and` + `Chassis::drive(), built ...` | SAID, extended (it names the builder; the old words were not false — this cell said REWORDED, with no evidence line, until the second review round) |
| 3 | :78 `from the CALIBRATION RECORD on the SD card when there is one, else from` | :971 | :678-679 | SAID |
| 4 | :79 `src/chassis_table.hpp. Not yet run on hardware before this boot: the first` | :972 | :679 `... else from src/chassis_table.hpp.` + the A1 run-status line (:681, `kRunStatus` :125) | REWORDED — FALSE since :76-106 itself (eight starts on 2026-09-25); A1's sentence replaces it |
| 5 | :80 `run is wheels-up. The invented X-drive graph is NOT constructed.` | :973 | :679-680 `The invented X-drive graph is NOT constructed.` | "the first run is wheels-up" DROPPED (the previous version's first run was that run; this version's procedure is Station F's); the X-drive clause SAID |
| 6 | :81 `BUILD STAMP Sep 24 2026 17:42:12` | :974 | :682 | SAID |
| 7 | :82 `competition status at boot: 0x00 field=no disabled=no auton=no -- the loop runs only in DRIVER CONTROL, ...` | :978 (bits formatted at :980) | :684 (bits formatted in main.cpp's branch, A2) | SAID |
| 8 | :83-84 `calibration record: DECODED, port check OK -- OK: all 8 ... the first drive catches that` | :1013 | boot_source.cpp:99 | SAID (printf, not cut) |
| 9 | :85 `typed table: cannot describe a drivetrain -- UNSET: track width, external ratio ...` | :1029 | boot_source.cpp:113 | SAID |
| 10 | :86 `boot source: RECORD -- MEASURED calibration record on the card, verified against the ports` | :1037 | boot_source.cpp:122 | SAID |
| 11 | :87 `shulib-robot-variant=tank-library` (TermSink) | :1071 | teleop_program.cpp:720 | SAID |
| 12 | :88-90 `[SES] run start · build v0.1.1-319-g2780462 · routine "(none: teleop only; auto…` (routine CUT at 24 bytes) | :1082 `"(none: teleop only; autons land later)"` | :728 `"(none: teleop only)"` (19 bytes, whole) | REWORDED — A4/A14: no auton ever lands in Teleop; the old text promised one and was cut anyway |
| 13 | :91-93 `[SES] alliance - · side - · batt 13.10V` | :1083-1086 | :726-732 | SAID |
| 14 | :94 `[SES] ports MEASURED L -16 +17 -18 -20 R +11 -12 +13 +15 IMU2 ipr 1.2544/1.3670 TW12.01` | :1085 `tankPortMapString()` | :725 `r.portMap()` (robot.hpp `writePortMap`) | SAID — byte-identical, pinned by test 6c against this very line |
| 15 | :95 `library graph BUILT from the CALIBRATION RECORD on the SD card: MEASURED tank BLUE 600 rpm: L4/R4 ports, ipr L1.25442 R1.36703, W 12.006 in` | :1092 | :602-606 | SAID (A15: the description's own line) — its text pinned by test 6c |
| — | (new) `robot::Robot: 2026 TANK CHASSIS (robot two) [] tank MEASURED | L -16,17,-18,-20 R 11,-12,13,15 | ipr ... | GPS ABSENT tags ABSENT vision ABSENT` (193 B) | — | :608-612 | NEW (§2b item 2: "the boot banner from robot.describe()") — pinned by test 6c |
| 16 | :96 `4+4 slots behind two MotorGroups, 4+4 PRESENT -- median reads ...` | :1098 | :614-622 (counts from `chassis().deps().motorGroups`) | SAID |
| 17 | :97 `description provenance: shulib Calibrate Sep 24 2026 17:42:16 | ROBOT-MEASURED: ...` | :1107 | :623 | SAID |
| 18 | :98 `boot source: MEASURED calibration record on the card, verified against the ports` | :1109 | :625 | SAID |
| 19 | :99 `port check: OK: ... NOT CHECKED: two drive cables swapped WITH EACH OTHER look identical here -- the first dr…` CUT | :1114 | :630-634 `logWrapped` -> 191 B + `... first drive catches that` (28 B) | SPLIT |
| 20 | :100 `[WARN] CROSS-CHECK: the MEASURED record and the typed chassis table disagree about 10 port(s). ...` | :1134-1139 (TermSink WARN after the header) | boot_source.cpp:153 (printf, BEFORE the header, as it is decided — A9) | SAID, MOVED (the `[WARN]` level tag is not on a printf line; the word CROSS-CHECK carries it). The count 10 pinned by test 6i |
| 21 | :101 `[WARN] HARDWARE-UNVALIDATED: the library's motion stack has not driven a robot before this program; gains and budgets are PROVISIONAL ... (speeds…` CUT | :1143-1147 | :636-640 `HARDWARE-UNVALIDATED: ` + A1's sentence (188 B + 165 B) ; :642 `kProvisionalLine` (the old tail, whole, 193 B) | REWORDED (A1: "has not driven a robot before this program" is FALSE — :76-106 is that program driving it) + SPLIT |
| 22 | :102 `absent devices (deliberate): gps=ABSENT tags=ABSENT vision=ABSENT -- no GPS or camera on robot two; ...` | :1148 | :643 `kAbsentDevicesLine` (:181) `... -- robot two has no GPS, and its AI Vision camera (port 3) has no adapter yet; nothing is polled over an absent source` (184 B — "179 B" until the second review round re-derived it from the source) | REWORDED — "no GPS or camera on robot two" is FALSE of the robot since the AI Vision was mounted on port 3 (2026-09-24 census, R3d-PROGRESS); A14 lists "no camera". The three ABSENT words and "nothing is polled" SAID |
| 23 | :103 `[WARN] ODO_STUCK stall detection has NO INDEPENDENT motion source ... is NOT wired…` CUT | :1152 (`kNoIndependentStallSourceNote`, 332 B) | :646 `logWrapped` -> 194 B + 142 B | SPLIT (A10: the library note in two parts) |
| 24 | :104 `dead-port tolerance: AT BOOT ONLY (R3d Part 2) -- ... A port that dies AFTER boot is stil…` CUT | :1161-1165 | :656-657 `kDeadPortLine1` (164 B) + `kDeadPortLine2` (133 B) | SPLIT (the same words, two literals, each `static_assert`ed <= 200) |
| 25 | :105 `facade alive: strafeAuthority=0.00 (tank: 0.00); stall check canDetectStall=false (by config); budgets lin 60 in/s ang 6.00 rad/s wheel 60 in/s; ff kS 1.00 kV 0.1714 kA 0.0000` | :1166 (`"false (by config)"` a LITERAL) | :659-666 (`canDetectStall` now READ from `motionConfig().stall.independentMotionSource`) | SAID — the budgets/gains part pinned by test 6d against this line |
| — | (new) `driver control: teleop::runDriverControl + DriveShaper, the default profile (continuous deadband 0.05, no expo, no slew -- HA-112, unfelt); re-zero button A; field-centric: OFF (selectable)` (189 B) | — | :667-669 | NEW (M4/M5 said at boot). Since the second review round the button word and field-centric's state are formatted FROM their constants, and the profile's numbers are `static_assert`ed against the words (2b.9) — the bytes are unchanged |
| 26 | :106 `opcontrol(): the LIBRARY's teleop loop is live on robot two -- Chassis::drive(Body) over two MotorGroups; deadband-only mapping (HA-112)` | :1323-1325 | :759 `kOpcontrolLive` (:193, 174 B) `... teleop::runDriverControl, Chassis::drive(Body) over two MotorGroups; DriveShaper feel, default profile (HA-112)` | REWORDED — "deadband-only mapping" is FALSE once Teleop runs the DriveShaper (M4, A6: the team lead's "switch", 2026-09-26) |

**Old lines on paths the 2026-09-25 capture never took** (no capture column):

| Old source | New source | Status |
|---|---|---|
| :1017 `calibration record: NOT USED -- <why>` | boot_source.cpp:102 | SAID; plus the new EMPTY-file reason (ruling G1-2b-R7) |
| :1045 `WARNING: driving from the TYPED TABLE -- unverified since the rebuild; run shulib Calibrate (slot 4). Port check ...` | boot_source.cpp:128 | SAID |
| :752 `REFUSED AT BOOT: <headline> -- <why>` | teleop_program.cpp:361 | SAID |
| :1050 headline `the TYPED TABLE does not match the ports` | boot_source.cpp:132 | SAID (wrapped over two MEDIUM rows on screen; the old screen cut it at 34) |
| :1057 headline `NO USABLE DRIVETRAIN DESCRIPTION` + `%.80s | %.100s` | boot_source.cpp:140, :138 | SAID |
| :1066 headline `an adapter REFUSED at construction` | teleop_program.cpp:705, :712 `the library graph REFUSED to build` | REWORDED (ruling G1-2b-R6: the provider catches the adapter's throw now) |
| :1154 `DEGRADED: %u recorded drive port(s) are EMPTY and were NOT constructed; ... %.80s` | :647-654 (`logWrapped`; count from `boot.absent()`) | SAID |
| :1329 `opcontrol(): field DISABLED -- loop exited cleanly after cancel() (0 V + Brake)` | :770-774, + `; <n> screen prints so far` | SAID, extended (the PROS print-leak count, R3d Part 2F's rule) |
| :1262-1263 AUTONOMOUS state screen `No motion here yet: the library teleop drives only under driver control.` | main.cpp autonomous() `benchStateScreen("AUTONOMOUS", kTeleopAutonomousReason)` = `No motion here, ever: Teleop has no autonomous.` (47 chars, `static_assert`ed) | REWORDED — A4: no motion here, EVER, not "yet" |
| :1265-1267 `autonomous(): no motion until the teleop loop has been validated on hardware and the gains measured (R5)` | teleop_program.cpp:741-744 `autonomous(): No motion here, ever: Teleop has no autonomous.` | REWORDED — A4 (it promised motion after R5) |

**The screen, old against new** (every string now through `printText`; literals `static_assert`ed against
the SMALL 50 / MEDIUM 36 / header 24 budgets):

| Old (main.cpp) | New (teleop_program.cpp) | Status |
|---|---|---|
| :790 `shulib TELEOP` | header word (`text::kHeaderWord`) | SAID |
| :792 `THE LIBRARY drives: Chassis::drive() -- NEVER RUN BEFORE` | row 0 `THIS VERSION has not run on hardware (G1 2b)` (amber) | REWORDED (A1: "NEVER RUN BEFORE" is false of the program since 2026-09-25) |
| :794 `BUILD %s` (whole stamp at x=300) | `BUILD <time>` at x=340 on the bar (Calibrate's layout; the whole stamp is on serial) | SAID (shortened, with the static_asserts Calibrate uses) |
| :814 `cmd vx ... w ... NO CONTROLLER -> 0` | row `cmd vx %+6.1f in/s  w %+5.2f rad/s[  NO CONTROLLER]` | SAID (<= 48) |
| :818 `est x ... y ... hdg ... deg <quality>` | row `est x ... y ... hdg ... <quality>` ("deg" dropped to fit 50) | SAID |
| :826 `L ...V n/m present k disagree \| R ...` (88 chars — cut on the brain's 50-char budget) | two rows, `L %+5.1f V  n/m present  k disagree` and `R ...` | SPLIT (A3: L and R on separate rows) |
| :839 `hdg xcheck ... battery ... L/R travel ...` (86) | two rows: `hdg xcheck ... rad/tick  batt ... V`, `L/R travel ... in (last tick)` | SPLIT |
| :845 `FAULT latched: first %s, last %s, n=%d` | two rows `FAULT n=%d  first %s`, `      last %s` | SPLIT |
| :851 `faults: none` | `faults: none` | SAID |
| :855 `budgets lin ... | ff kS ... kV ... PROVISIONAL HA-45` | serial-only (boot lines :642, :659-666) | DROPPED from the screen (A3 permits: never changes) |
| :859 `FEEL differs from shulib Drive: ...` | serial-only (:642) | DROPPED from the screen (A3) |
| :860 `stall check: NO independent source (odometry IS the wheels)` | serial-only (:646) | DROPPED from the screen (A3) |
| :866 `TYPED TABLE -- unverified since the rebuild; run shulib Calibrate` | source rows 1-2, same words wrapped | SAID |
| :868 `MEASURED record | %.70s` (verdict reason) | source rows 1-2: `MEASURED record | <reason>` wrapped at word boundaries over two SMALL rows | SAID (Station G.6.2 quotes updated). Since the second review round a reason longer than two rows ends at its last whole sentence with ` ...` (`sourceRows()`, 2b.9) — robot two's DEGRADED row 2 no longer ends on a stray `NEW` |
| :871 `MEASURED record from the SD card, verified against the ports` | source rows 1-2, same words wrapped | SAID |
| :874 `table %s` (the port map, labelled `table` whatever the source) | serial-only (`[SES] ports`) | DROPPED from the screen — A3: the `table` label was false on a MEASURED record |
| (none) | row `driver ARCADE BODY  field-centric OFF` (mode + `DriveShaperState`) | NEW (§2b item 2) |
| :730 `shulib TELEOP -- REFUSED AT BOOT` | red bar `TELEOP REFUSED AT BOOT` | REWORDED (fits the 24-char header budget) |
| :733 `%.34s` headline / :735-741 `%.58s` x3 why | headline over two MEDIUM rows; why over four SMALL rows (+ `(the full text is on the serial log)` when cut) | SAID (A3: wraps at kSmallCols, not 58) |
| :744 `The library graph did NOT construct. Nothing is powered.` (56) | `Library graph NOT built. Nothing is powered.` (44) | SAID (reworded to fit) |
| :746-748 `Run shulib Calibrate (slot 4): the robot measures itself and / writes the answer to the SD card. This program boots from it.` | `Run shulib Calibrate (slot 4): the robot measures itself and saves the record Teleop boots from.` over two rows | SAID (reworded to fit) |
| :750 `An adapter refusal: Bench Tests (slot 3) diagnoses it.` (54) | `Adapter refused? Bench Tests (slot 3) diagnoses.` (48) | SAID (reworded to fit) |
| :751 `BUILD %s` | `BUILD <time>` on the red bar | SAID |
| LCD `LIB 12V Bxx.xV` / `hdg ... deg` / fault / `no controller 0V` / `DEGRADED see brain` / `ok` | the same three lines, same formats | SAID |

Rows the new panel paints, top to bottom (y = 32 + 16 n): run status; cmd; est; driver; L; R; hdg xcheck +
battery; L/R travel; fault 1; fault 2; source 1; source 2 — twelve SMALL rows, the last ending at y=224
(<= 235, `static_assert`ed). Live rows are recomputed every 20 ticks (200 ms) and repainted only when their
text or colour changed; the old panel printed ten rows every 100 ms.

### 2b.5 The prose review, widened (A14) — run BEFORE this was written (02:11-02:24 EDT)

**The greps, as run** (living documents only: `README.md`, `docs/*.md`, `docs/guide/`, the three
living internal documents `build-order.md` / `RESUMING.md` / `HANDOFF-remaining-work.md`, the worksheet,
and the comments of `include/`, `src/`, `test/`, `tools/`, `Makefile`, `.github/`; the per-chunk
`-PROGRESS` / `-COMPLETED` records and briefs are ledgers of their day and are not edited):

```sh
P='TankRobot|runTeleopLoop|mapSticksToChassisSpeeds|deadband-only|tankPortMapString|tankChassisConfig|TankMotors|TankPanel|NEVER RUN|never run|has not run|UNRUN|no camera|autons land|hand-wired|composition root'
grep -rnE "$P" README.md docs/*.md docs/guide docs/internal/{build-order,RESUMING,HANDOFF-remaining-work}.md docs/internal/chunks/R3a-BENCH-WORKSHEET.md include src test tools Makefile .github
grep -rnE "main\.cpp|Part 2b|Part 2\b|no program|No program|not yet|moves onto" include      # §5's terms, include/
grep -rnE "main\.cpp" README.md docs/*.md docs/guide                                            # §5's term, public docs
grep -rn '`table \|table L -\|"table ' docs src include test README.md                          # the panel label (A14)
```

Every hit was read. Counted per file first (58 files hit the first pattern when first run; 60 on a re-run at 02:31, after the fixes below and with this part's new files); the generic uses — "composition
root" as a concept, "never run" about a callback, "wiring" of a test, "no camera" about a robot with none —
are still true and left. **Fixed — FALSE after 2b (or found false in a file 2b edits), each with the A1
wording where it is a run status:**

| File | What was false | Now |
|---|---|---|
| `include/shulib/robot/robot.hpp` :13-19, :22-23, :71-75, :325-332, :346, :373, :690, :846-848, :866, :920 | "no program in src/ uses either line yet"; "src/main.cpp is unchanged and still wires both of its graphs"; `tankPortMapString()` / `TankMotors` / `tankChassisConfig()` / `TankRobot` in the present tense | "Teleop is built with exactly these two lines since G1 Part 2b ... that version has not run on hardware"; the retired functions in the past tense, with where the pin now lives |
| `include/shulib/hal/pros/device_provider.hpp` :72, :85-86, :328 | "No program constructs one yet: src/ is unchanged"; "both of src/main.cpp's graphs" | "Since G1 Part 2b one program constructs one ... no provider has yet made a device on a brain" |
| `include/shulib/teleop/teleop_loop.hpp` :51-52 | "No program calls this yet" | "Since G1 Part 2b Teleop drives through it ... that version has not run" |
| `include/shulib/teleop/stick_mapping.hpp` :133-136 (a doc comment only — landmine 7 forbids the BODIES) | "what Teleop's loop in src/main.cpp uses each tick today" | the X-drive loop's, until 2c; Teleop moved at 2b |
| `include/shulib/hal/motor_group.hpp` :85 | "Teleop (src/main.cpp's `TankMotors`)" | robot::Robot, which Teleop is built on since 2b |
| `include/shulib/diag/session_info.hpp` :26-27 | "no program in src/ uses the generator yet" | Teleop prints its port map from it since 2b |
| `include/shulib/config/robot_config.hpp` :9, :325 | "src/main.cpp's two graphs", "as both src/main.cpp graphs do today" | past tense |
| `src/calibrate_program.cpp` :17-18 | "THIS version ... HAS NEVER RUN ON HARDWARE" (false since 2026-09-24; its own runtime banner already said so) | "FIRST RAN on robot two the evening of 2026-09-24 ..." |
| `src/bench_r3a.cpp` :239-240 (A13) | "printText, re-implemented here because the two programs share no translation unit" | names `src/screen_text.hpp` and why the tester keeps its own |
| `src/main.cpp` header :1-3, :12-18 (A1), :22-26 (A4), :56-61, :66-68, :89-91, :109, :131, the handler's comment, `runTeleopLoop`'s banner, the `initialize()` / `autonomous()` (A4) / `opcontrol()` doc comments | "THE LIBRARY HAS STILL NEVER DRIVEN A ROBOT ... it has NOT run"; "autonomous() still commands NO motion: R3 owns ..."; "the tank graph below"; "BOTH object graphs call"; the #error's "carries geometry" (the table carries none) | A1's wording; A4's "no motion in ANY build ... never"; the new file named |
| `test/robot_builder_test.cpp` :16-21, :309, :391, :642, :695; `test/drivetrain_calibrator_test.cpp` :282; `test/boot_check_test.cpp` :406; `test/pros_device_provider_test.cpp` :397 | the transcribed `TankRobot` / `tankChassisConfig()` / "the shape src/main.cpp uses" in the present tense; "src/main.cpp is untouched and still wires both of its graphs" | past tense, with G1 Part 2b named (test NAMES left unchanged, so the logs that cite them stay true) |
| `tools/src_build_gate.py` :34-35, :102-110 | "the tank object graph in src/main.cpp"; "the library program builds its OWN tank graph ... its own helpers are guarded under SHULIB_RUNS_TANK_LIBRARY" | `src/teleop_program.cpp`, `robot::Robot`; main.cpp's branch only forwards; the new TUs are compiled everywhere and `--gc-sections` drops them |
| `Makefile` :86-91, :103, :125 | "the tank object graph in src/main.cpp"; Calibrate "UNRUN on hardware"; "only robot two's chassis table ... carries geometry" | the new file; "has run on robot two since 2026-09-24 (its recorded reproduction run is still owed)"; "a calibration record and a signed chassis table" |
| `README.md` :236-246, :273-274 | "`src/main.cpp` is the complete, commented wiring example ... robot two's graph is built from ..."; the tree line without Teleop | the builder as the on-ramp and `src/teleop_program.cpp` the worked example, the longhand kept as Tier 3; Teleop in the tree line |
| `docs/guide/07-getting-set-up.md` :122-125 (2c's chapter, but 2b made this sentence false — rule 3) | "`src/main.cpp` ... is the worked example of constructing a whole robot; the other files are ..." (no Teleop) | both worked examples named |
| `docs/hardware-assumptions.md` HA-112 source + confidence, HA-125 source, HA-131 source, HA-141 settle (A5) | "which src/main.cpp's loop and the drive program still call"; "The library has never been driven by a person"; the boot decision / the stall note "in src/main.cpp"; the pulled-IMU settle step | the X-drive loop, Drive and the tester call it, Teleop runs the feel layer; "the FEEL LAYER has never been driven by a person" (the previous Teleop was driven through the OLD mapping on 2026-09-25); `src/boot_source.cpp` / `src/teleop_program.cpp`; A5's two halves |
| `docs/internal/HANDOFF-remaining-work.md` :267-268 | "the library teleop loop in `src/main.cpp`; never run" | `robot::Robot` + the loop in `src/teleop_program.cpp`, "this version never run — the previous one drove robot two ... 2026-09-25" |
| `docs/internal/chunks/R3a-BENCH-WORKSHEET.md` Station G's closing note | "THIS VERSION OF 'shulib Calibrate' HAS NOT RUN" | it has run; the recorded reproduction run is owed; which run wrote the record is not provable |

**Hits in files 2b does NOT edit, or statements made false by the 2026-09-25 session rather than by 2b —
owned by the HELD documentation pass (A7; the team lead's "just the status lines", 2026-09-26), listed so
it is not lost:**

- `docs/roadmap.md` :237-250 — the dated R3b Parts 1-3 paragraph ends "**It has NOT run.** It cannot
  yet: the table's geometry is UNSET ..." — false of that program since 2026-09-25.
- `docs/guide/14-what-it-cannot-do-yet.md` :19-25 — lists 2026-09-17's "shulib Drive" drive as the last
  hardware event; the 2026-09-25 Teleop drive (the library's own graph, open-loop, under a driver) is not
  named. (The chapter heading "It has never driven a robot under its own control" is still TRUE and
  stays — A14.)
- `docs/internal/HANDOFF-remaining-work.md` :269-270 — "shulib Calibrate ... **never run**" (false since
  2026-09-24).
- `include/shulib/calibration/drivetrain_calibrator.hpp` :20 — "never run on a robot at all. No physical
  drivetrain has ever been described by it" (false since 2026-09-24/25).
- `include/shulib/localization/apriltag_corrector.hpp` :119 — "there is no robot, no camera" (robot two
  carries an AI Vision camera since 2026-09-24; there is still no adapter).
- `docs/internal/build-order.md` :1462 — COMP "waits for his go, after G1 Part 2b" (true, but the
  2026-09-25 ruling put 2c first; the Next block says so).
- `test/guide_examples_test.cpp` :166 / `docs/guide/08` :167 — "(they exist; src/main.cpp wires them)":
  still true (the X-drive graph), and the comment is inside a block the guide quotes VERBATIM (the example
  gate would fail on a one-sided edit) — left for 2c, which rewrites ch. 8's wiring section with ch. 7.
- **Deliberately left as they are (A14):** every "never driven a robot under its own control" line — still
  true, no library-commanded motion has run; the R3 sweep owns them.

### 2b.6 The documentation, in this part's (uncommitted) change (02:06-02:24 EDT)

| Document | What moved |
|---|---|
| `README.md` | the "Wiring a `Chassis`" paragraph (builder = on-ramp, `src/teleop_program.cpp` the worked example, longhand = Tier 3); the tree's `src/` line |
| `docs/guide/08-your-first-routine.md` :300-312 | "none of the programs in `src/` is built this way yet" -> Teleop is, this version unrun, the previous version drove on 2026-09-25; "on the simulator" -> "on the host test plant" |
| `docs/guide/10-the-api.md` :185-188 | "no program on the robot uses it yet" -> Teleop drives through it, unrun; "simulator" -> "host test plant" |
| `docs/guide/14-what-it-cannot-do-yet.md` | the driver-control bullet: only Teleop uses the feel layer, unrun; Drive and the tester keep the raw mapping (M4); no driver has felt it |
| `docs/guide/07-getting-set-up.md` | the `src/` sentence (2b made it false; 2c rewrites the chapter's `src/` description) |
| `docs/hardware-assumptions.md` | HA-112's source and confidence, HA-125's and HA-131's sources, **HA-141's settle step reworded (A5)** |
| `docs/roadmap.md` | the two Phase T rows (still `[~]`, now saying exactly M4), the M5 config row, the session-header port-map note ("until Part 2b"), the G1 register row, "You are here" (**A7: 2b landed, THIS version unrun; one sentence for the previous version's 2026-09-25 drive; Next = 2c, then COMP**), the dated G1 Part 1 / 2a paragraphs marked "true of that day" |
| `docs/vexbuilder-integration.md` :64 | "Parts 2b/2c ... are open" -> 2b landed (unrun), 2c open |
| `docs/internal/build-order.md` | **the first `**Next:**` = G1 PART 2c, then COMP** (A7), with a 2b-landed sentence and the old first sentence kept as history; G1's status (entry + table + the "half built" paragraph + the Part 1 note); T2's status line |
| `docs/internal/HANDOFF-remaining-work.md` | **L1** (2b landed, the team lead's 2026-09-26 answers, 2c waits) and **§7 item 1** (re-check 2c, then COMP) — A7's two additions; the §4 program line's location clause |
| `docs/internal/chunks/R3a-BENCH-WORKSHEET.md` | **Station F rewritten from the new code**: A1's status, **A8's three WHICH-END statements**, F.0 = "no record on the card means refused; the typed table cannot describe robot two" (the tape-and-type steps retired), the F.1 panel table from the new rows (budgets/feel/stall/port map now serial-only, with the capture's numbers), the dead-port paragraph (tolerated at boot), F.2 re-pointed (census only; the camera end; the ~8 % scale; F.2.6 the feel against the previous version); **Station G's Teleop quotes** — G.0.4 (the record's front), the LEFT OUT paragraph's NEW-motor line, G.6.1 (two source rows), G.6.2 (`3/4 present`), G.6.3 (`4/4 present 0 disagree`, `4+4 PRESENT`, budgets `lin 60 ... ang 6.00 ... wheel 60`, `ff kS 1.00 kV 0.1714`), **G.6.4 keeps "about 48"** + the 44 line, G.6.5 (the moot table comparison replaced), "What to send back" 5-6, the Calibrate closing note — each string checked against `src/teleop_program.cpp` / `src/boot_source.cpp` / `boot_check.hpp` |
| `Makefile`, `tools/src_build_gate.py` | the tank-library descriptions (and 2b.5's other fixes) |
| `docs/api/` + `mkdocs.yml` | regenerated (`api_doc_tool.py generate`: 7 pages moved — the `///` comments fixed in 2b.5) |
| `docs/internal/PROJECT-BRIEFING.md` | regenerated LAST (2b.8) |
| this log | this section |

**Deliberately NOT touched, with why:** `docs/changelog.md` (it records library API changes; 2b changes no
public surface — only `src/`, comments, and one bounded copy inside `src/chassis_table.hpp`); the master
plan (the WHY; no decision in it changes); `RESUMING.md` (its protocol is unchanged); the held 2026-09-25
documentation pass (A7; the list in 2b.5); guide ch. 7's and ch. 8's `src/` / wiring sections beyond the
one sentence 2b made false (2c's list). **A16:** the list is longer than rule 7's five — accepted by the
team lead on 2026-09-26.

### 2b.7 Review findings and their disposition (the three reviewers of the first run; 02:32 EDT)

Three read-only reviewers (robot behaviour, test strength, docs/process) reported 15 findings against the
tree the FIRST executor left (00:32-01:15 EDT). Each was re-checked against the tree by this run, and every
one was REAL. What was done:

| # | Lens / severity | Finding | Real? | Disposition |
|---|---|---|---|---|
| F1 | robot / blocker | No Part 2b change existed; the executor stopped at 2b.0 | REAL (re-measured: `git diff --numstat` = this file only, `src/main.cpp` 1,340 lines, no new files) | Recorded in 2b.0a; this run did Part 2b on the same tree (`b622d9b` + the 17-line header) — code 2b.2, test 2b.3, diff 2b.4, docs 2b.5-2b.6. Packages must still be built only from a COMMITTED tree (landmine 9) |
| F2 | robot / minor | The dirty log described a run in progress that had ended | REAL | 2b.0a says the first run stopped with no code and that this run continues the SAME section (no second header) |
| F3 | tests / blocker | No implementation, test 6, mutation table or banner diff | REAL | 2b.2, 2b.3 (10 cases, 12 mutations red), 2b.4 |
| F4 | tests / major | A11's `absent().size() == 1` does not check WHICH port | REAL — on robot two every port is a drive port, so a wrong port passes `validate()` | 6e: port **13 (RIGHT)** missing through the REAL `verifyAgainstRegistry` over a `FakeDeviceRegistry`, `absent()[0] == 13`, `buildOptions()` carries exactly it, the Robot built with right 3/4 and left 4/4, 13 never requested, `DEGRADED: absent 13`; 6f: two ports, negative entries, left-then-right order. The suggested copy-paste mutation is not expressible verbatim (the core walks both sides through one loop); its equivalent M2 (the LEFT side read twice) and M3 (the signed entry) are killed |
| F5 | tests / major | The "whole array" mutation only reached the header; `teleop_program.cpp`'s `BuildOptions` spelling was covered by nothing | REAL — the host build never compiles `src/*.cpp` | Fixed at the root (ruling G1-2b-R1): the storage is PRIVATE in `BootConfig`; the options leave only through the inline `buildOptions()`; `teleop_program.cpp` spells nothing else (`grep -n "absentPorts\|absentCount" src/teleop_program.cpp` prints nothing); test 6 calls `buildOptions()` and `validate()` with it; M1 and M9 killed |
| F6 | tests / major | `tuning == ChassisConfig{}` has no `operator==`; the likeliest bug (porting `tankChassisConfig()`) is hidden by `effectiveTuning()` | REAL | `requireDefaultTuning()` compares EVERY leaf longhand (`motion.stall.*` and every scheduler field included; no memcmp), with host `sizeof` pins so a new field fails until listed; 6d pins the Robot's budgets/gains against the capture's :105 text and `kV == 12/70`. M4 (the stall port) is killed ONLY by the longhand check; M5 (kV) by both |
| F7 | tests / major | Per-side scales must come from `driveOdometry()` and be compared with `==` | REAL | 6d: `leftInchesPerRadian() == 1.25442`, `right == 1.36703`, `trackWidth() == 12.006`; 6b field-by-field `==`. M6 (left into right) and M7 (a float round trip, 3.4e-8 relative — 350x under `Approx`'s epsilon) killed |
| F8 | tests / minor | The port-map golden must be the captured literal; add a TYPED case | REAL | 6c: the literal copied from capture :94 with a comment naming the file and line, from a description rebuilt from the capture's fields (W 12.006, ipr 1.25442/1.36703, the provenance); 6a first proves the fixture reproduces the robot's own port-check sentence; 6h: a TYPED description stays TYPED (M8 killed) |
| F9 | tests / minor | The A10 diff must carry the cut routine text and the "deadband-only" opcontrol line, three-way | REAL | 2b.4: 26 rows re-derived from `zcat ... | strings | sed -n 74,107p` (the first boot block is lines 76-106 of that output, not 80-135); row 12 (routine) and row 26 (opcontrol) REWORDED with the evidence; the four TermSink-cut lines SPLIT (rows 19, 21, 23, 24) |
| F10 | docs / blocker | Part 2b not in the tree | REAL | as F1/F3 |
| F11 | docs / major | The log read as in progress | REAL | as F2 |
| F12 | docs / major | None of §2b's documents updated; seven HEAD sentences false | REAL | (a) Station F's "It has never run" -> A1's status; (b) F.0 -> "no record on the card means refused; the typed table cannot describe robot two"; (c) F.1's `budgets lin 60 ang 3.0 wheel 70` row -> serial-only with the capture's `lin 60 ... ang 6.00 ... wheel 60`, and the false `table` row dropped (A3); (d) A8's three WHICH-END statements added, F.2.3 says the camera end; (e) G.6.3 -> `4/4 present 0 disagree`, `4+4 PRESENT`; (f) HA-141's settle step reworded (A5); (g) `main.cpp` :793, :972, :1144 are gone with the old program and their replacements are A1's words (2b.4 rows 4, 21, and the screen table). The pointers moved with the code (build-order Next, roadmap "You are here", HANDOFF L1 and §7 item 1); every prose file on the list edited (2b.6); the briefing regenerated LAST (2b.8) |
| F13 | docs / minor | The 2b header did not record the team lead's go | REAL | 2b.0a records what exists: his four answers at 00:21 EDT 2026-09-26 (switch, status lines only, accept the list, commit the log), the launch at 00:32, his "we arent going to rush" at 00:40 — and that NO sentence of the form "go on 2b" is recorded (from the coordinator's session transcript; the tree records only the decisions). **The coordinator confirms or corrects this before committing** |
| F14 | docs / minor | The test binary was relinked a minute before the log said "before any code" | REAL (unattributable from the tree) | 2b.0a says that binary is not a logged baseline; 2b.1 measured the baseline itself: 1425 cases / 2,386,871 assertions |
| F15 | docs / minor | A7's "pointers only" reads two ways; `b622d9b`'s body took the narrow one | REAL — the ambiguity is in the brief | This run read it the way the review recommends: §2b's documentation list plus HANDOFF L1 / §7 item 1 are ALL required; A7 limits only what 2b writes about the HELD 2026-09-25 pass. 2b.5 lists every statement left for that pass. **The commit body should say this, not "pointers only".** |

One more finding, this run's own: **F-2b-1** (`tableToDescription()`'s certain `-Wformat-truncation`,
fixed where it lives — 2b.2).

### 2b.8 Every gate, re-run on the final tree (02:33-02:40 EDT)

| Gate | Command | Result |
|---|---|---|
| Host build, relink required | `touch test/boot_source_test.cpp && cmake --build build/test --target shulib_tests/fast` | exit 0, **`[ 41%] Linking CXX executable shulib_tests`** |
| Host suite | `./build/test/shulib_tests` | **1435 test cases / 2,387,098 assertions, 3 skipped — SUCCESS** = the A17 baseline (1425 / 2,386,871) + test 6's 10 cases / 227 assertions, exactly: no pre-existing case gained or lost an assertion |
| src build gate self-test | `python3 tools/src_build_gate.py self-test` | `OK (33 detector cases, each against a real build)` |
| src build gate | `python3 tools/src_build_gate.py check` | **PASS** — six builds, each 6/6 TUs, 0 disallowed; the two allowed `-Wunused-function` are `src/main.cpp`'s `robot()` / `portMapString()` in all five tank/bench builds and none in xdrive (the variant differential unchanged); hashes and beacons asserted |
| ARM header gate | the `arm-none-eabi-g++ ... -Werror` compile of every `include/shulib` header | exit 0 (184 headers) |
| CI greps | `<pros/>` outside `hal/pros`; `shulib/sim/` outside `sim/` | both print nothing |
| API docs fresh | `python3 tools/api_doc_tool.py check-fresh` | exit 0 (after `generate`, 2b.6) |
| Removability | `python3 tools/api_doc_tool.py check-removability` | `removability: no public doc references docs/internal/` |
| Doc staleness | `python3 tools/doc_staleness_audit.py` | `clean (26 live docs, 0 numeric claims checked)` |
| Briefing | `python3 tools/briefing_status.py generate`, then `check` — LAST, after every relink above | see the line below |

Packages: none uploaded, none built for a robot from this tree (`pros` never run, `/dev/ttyACM*` never
touched); `bin/` and `.d/` removed after the gate runs. Nothing committed, staged or pushed; no branch or
worktree created.

**Briefing (02:39 EDT), the last step:** `python3 tools/briefing_status.py generate` -> `regenerated` (two
lines moved: **Next up** = "G1 PART 2c (main.cpp as a dispatcher), then COMP — each on its own go";
**Suite** = 1,435 cases / 2,387,098 assertions); `check` -> `generated block current`, exit 0. The full
`cmake --build build/test` then ran every doc gate (coverage 2592 entities / 148 headers, 408 verbatim
example lines, removability, the briefing, the staleness audit and its self-test) and exited 0.

**State at the end of this run:** Part 2b is COMPLETE in the working tree and UNCOMMITTED; THIS VERSION of
"shulib Teleop" has not run on hardware. The coordinator verifies (§7: read `teleop_program.cpp`'s
composition and `boot_source.cpp` against the old boot block; re-run 2b's banner diff; rebuild the
hardware packages from the COMMITTED tree after the commit and confirm the beacons), confirms the team
lead's go (2b.0a / F13), and commits with a body that names the held documentation pass (2b.5) as
deferred.

### 2b.9 The second review round, and its disposition (03:00-03:55 EDT)

Three read-only reviewers (robot behaviour, test strength, docs/process) reported 23 findings against
the 2b.0-2b.8 tree. **This run re-checked every one against the tree before acting** and fixed each real
one at its root, within §2b's scope; nothing is committed, staged or pushed, and nothing ran on a robot.
Numbers in brackets are the reviewers' order (robot R1-R3, tests T1-T7, docs D1-D13).

| # | Finding (severity) | Real? | What was done (file:line in the final tree) |
|---|---|---|---|
| R1 | Boot line says "re-zero button A" as a literal; nothing stops the button moving to a shoulder (minor) | REAL | `src/teleop_program.cpp`:146 `static_assert` that `kRezeroButton` is not L1/L2/R1/R2 (T3's claw buttons); :152 `buttonWord()` and the driver line now formats the button FROM the constant (`logBootLines`, same 189 bytes). Mutation S1 (A -> L1) fails the ARM build at :146 |
| R2 | The library's `DriverLoopConfig::rezeroButton` defaults to L1, T3's claw button (minor; Part 2a code) | REAL, not fixed in 2b (a public-API default is T3/COMP's decision; the finding itself says so) | The doc comment that called the defaults "deliberately not a working configuration" was FALSE for the button and is corrected (`include/shulib/teleop/teleop_loop.hpp`:75-81, `docs/api/teleop_loop.md` regenerated); the owed change (a refusing sentinel or a non-shoulder default) is recorded on HANDOFF L5 (T3). Teleop itself names A and is protected by R1's `static_assert` |
| R3 | The DEGRADED source row is cut at two rows with no marker; on robot two it ends on "NEW" (minor) | REAL (reproduced with wrapWords' own rules) | Ruling G1-2b-R10: `sourceRows()` (`src/teleop_program.cpp`:364) ends the two rows at the LAST WHOLE SENTENCE that fits with ` ...`, else at the last whole word that leaves room for it; `paintAll()` uses it (:611). Three `static_assert`s prove it at compile time on robot two's exact reason (`... driving` / `without them (left dead 0, right dead 1). ...`), on a fitting text, and on a text with no sentence end (:422-446). Mutations S3/S4 fail the build. F.1 row 11-12 and G.6.2 re-quoted |
| T1 | The program used `BootOutcome`'s forwarders, which no test built — A11's whole-array mutation survives on the program's path (major) | REAL — `grep BootOutcome test/*.cpp` was empty; `teleop_program.cpp` called `st.boot.config()` / `buildOptions()` / `absent()` | Ruling G1-2b-R8: the forwarders `config()`, `absent()`, `buildOptions()` and the caller-less `sourceWord()` are DELETED (`src/boot_source.hpp`:193-214); the program spells `st.boot.built.config` / `st.boot.built.buildOptions()` / `boot.built.absent()` (`teleop_program.cpp`:798, :855, :861), the door test 6 goes through — 6k/6l build a `BootOutcome` through the real decision flow and validate and construct the Robot through `built`. M1 and M9 (the whole array; the options dropped) now also go red in 6k/6l. The 2b.2/2b.3 claims ("one door", "cannot spell the hazard") are marked overstated in place |
| T2 | `decideBootSource()`'s decisions have no host coverage although fakes exist (minor) | REAL | Ruling G1-2b-R9: the whole flow moved into `src/boot_source.hpp` as `readRecord(IBlockSource&, path)` (:230) and `decideBootSourceFrom(record, IDeviceRegistry&, table, tableMissing, robotName, printLine)` (:296), PROS-free; `src/boot_source.cpp` is now a 20-line shim (card opened, read and CLOSED before the registry — the old order; printf under the tag). New cases 6k-6r (`test/boot_source_test.cpp`:616-822) run every row: record OK (reproducing the capture's :83-86 and :100 lines WORD FOR WORD, and no cross-check line when the table agrees), DEGRADED, a refused record with a usable table (no fallback; IMU and SWAPPED variants), no card + usable table, no card + unusable table (both reasons), an open EMPTY file (R7), a TRUNCATED record, a table failing its own port check. Mutations D1-D8 all red |
| T3 | `sizeof` pins cannot see a field added into padding (minor) | REAL (verified: A1/A2 below leave every size unchanged) | A MEMBER-COUNT pin on every struct `requireDefaultTuning()` walks (`test/boot_source_test.cpp`:220-255: a C++20 brace-initialisation arity probe; ChassisConfig 2, MotionConfig 12, FeedforwardGains 3, AxisGains 4, SettleConfig 3, OdoStallCheckConfig 6, DriveGeometry 2, MotionSchedulerConfig 4, LoopMonitorConfig 1, PlausibilityConfig 3); the size pins stay for a changed TYPE; the header comment says exactly this. A1 (a bool after `independentMotionSource`) and A2 (a uint32 after `abortFaultMask`) each fail ONLY the member-count pin |
| T4 | The hand-bounded provenance copy is never asserted where it cuts (minor) | REAL (6/`chassis table 1` checks only `strstr`; the cut path ran with only `valid()` checked) | `test/chassis_table_test.cpp`:483 "chassis table 3": byte-for-byte against a `std::string` rebuild of the old snprintf, at a short text, at EXACTLY 95 bytes, at 96 (cut by one, `[95] == '\0'`) and on the bench table this TU compiles. P1 (`n + 1` -> `n`, the NUL past the buffer) and P2 (`n + 2`) both red |
| T5 | The A10 NEW column and several logged numbers were not re-derived after the last edit (minor) | REAL — row 22 is 184 B (logged 179), R5's pieces are 196 (logged 190), the stall note is 333 (logged 332); row 2's REWORDED had no evidence | Re-derived MECHANICALLY by `scratchpad/r2/lens2.py` (literals extracted by name from the final source, formats and `logWrapped()`'s split re-applied): see the table below. 2b.2, 2b.4 corrected in place; row 2 relabelled "SAID, extended" |
| T6 | The mutation campaign ran on an earlier test file; 226 vs 227 (minor) | REAL | 2b.3 corrected (227; the `describe()` golden was added after M1-M12 ran). The WHOLE campaign re-ran on the final code (below), plus M13 — the reviewer's `describe()` separator mutation, killed by 6c's golden |
| T7 | Boot-line literals hard-code the button and the feel numbers (minor) | REAL | The button: R1. The feel: `driverLineMatchesProfile()` (`teleop_program.cpp`:184-195) `static_assert`s that `teleopProfile()` has deadband 0.05 and expo 0 on all three axes and no slew, the words the line says; mutation S2 fails the build. **Residual, stated:** the program's own format wrappers (`robot::Robot: %.220s`, the facade line's text around the numbers) are still compile-only — 6c/6d pin the library values they print, and the A10 diff is the check on the wrappers |
| D1 | Every "Next" pointer names 2c; the team lead's order is brake-mode fix -> T3 -> 2c -> COMP (major) | REAL — the ruling (2026-09-26, relayed to this run with the task) was recorded nowhere | `build-order.md`:632 first `**Next:**` = DEFECTS2 (the brake-mode fix; Teleop coasts on release), then T3 (claw pivots on 6/7, R1/R2 and L1/L2), then 2c, then COMP; a deviations row at :1900 (the ORDER only — DEFECTS2's own commit adds its entry and the chunk count, so the coordinator should drop "a deviations-table row ... and the order" from the DEFECTS2 brief's §3 or reword it); G1's rows (:946, :1482) and COMP's (:1462) say "after DEFECTS2 and T3"; `roadmap.md`:348 "Now"; HANDOFF L1 and §7 item 1; the briefing regenerated LAST |
| D2 | The briefing's hand-written §12/§13 still say 2b waits for the go (major) | REAL | `PROJECT-BRIEFING.md` §12's R row (the brain has enumerated since — sessions 2026-09-24/25), T row, G row; §13's heading and "Current as of 2026-09-26" box |
| D3 | A14 missed false lines in edited files, and applied its held rule inconsistently (major) | REAL, all seven | (a) `src/main.cpp`:68-70, :368 camera comments -> "no camera ADAPTER exists yet" (robot two has an AI Vision on 3); the X-drive's runtime line :599 likewise; (b) :582 routine `"(none: autons land at R3)"` -> `"(none: teleop only)"`; (c) `README.md`:116 Calibrate's run status; (d) HANDOFF §4's Calibrate "**never run**"; (e) HA-70's confidence (`hardware-assumptions.md`:1907); (f) `calibrate_program.cpp`'s "matching the tank graph's own helper" — see the note under this table; (g) the greps re-run case-insensitively with the added terms, below. **One rule, applied everywhere (ruling G1-2b-R13):** in a file 2b edits, a FALSE run-status line gets A1's correction (A14); a line inside a dated record is true of its day and is left or annotated as such; a NEW 2026-09-25 finding beyond run status waits for the held pass (A7) |
| D4 | Worksheet G.0.4's camera-end advice contradicts G.5.2 / G.2.2; Station F item 2 contradicts G.2.2 (major) | REAL | G.0.4 (:722) states the fact (the 2026-09-25 record's front is the camera end; a push from 15/16 reverses Teleop's forward) and NO recommendation ("there is no ruling"); G.5.2 (:854) gives both fronts' sides; G.6.3 (:903) "toward the end you pushed first in G.4"; Station F item 2 (:517-523) says the AGREES rows compare against the TABLE's front and that G.2.2 uses MOTOR WATCH differently |
| D5 | README says robot two's config falls back to "a measured port table" (minor) | REAL | `README.md`:240-245: the table is the fallback in general; robot two's cannot describe its drivetrain, so without a record Teleop refuses |
| D6 | Two Station F quotes differ from the code (minor) | REAL (`%+6.1f` pads; the refusal's `%.80s \| %.100s`) | LCD `hdg  +45.2 deg` (:582); F.0's second row (:549) quotes the screen's reason as cut at 100 characters ("... this robot was rew", no mark) and points to the `calibration record: DECODED, port check REFUSE -- ...` serial line for the whole sentence. The cut itself is the moved code's, unchanged — now PINNED by 6m (`ends_with("this robot was rew")`); widening it is listed below, not done |
| D7 | F.2.1's MOTOR WATCH table-front push and the 30 s shulib Drive check were dropped without a ruling (minor) | REAL | Ruling G1-2b-R12: the MOTOR WATCH push is RESTORED (:622), labelled as the TABLE's front (A8(2)), with what it checks and the derived expectation (the eight recorded ports AGREE, 19 empty, 14 written down); the 30 s shulib Drive run stays dropped, with its reason in the step: it proved the TABLE's signs for a Teleop that drove from the table, and slot 1 just before slot 2 puts two opposite "forward"s back to back |
| D8 | HA-141's summary row still settles on 2b's first boot alone (minor) | REAL | `hardware-assumptions.md`:228: "IMU half: G1 Part 2b's first boot; the provider's kind refusal: the first program that asks it for a GPS or rotation port / R3" |
| D9 | HANDOFF §1 lists G1 as "2b and 2c open" (minor) | REAL | HANDOFF §1: 2b landed 2026-09-26, build-gated, unrun; 2c open; T2's line: its wiring landed with 2b |
| D10 | Log numbers wrong: 226/227, three/four hunks, 179/184, and the go unrecorded (minor) | REAL, all four | 2b.3 (227; four hunks), 2b.4 (184 B), 2b.0a "CONFIRMED" and HANDOFF L1 — the go confirmed to this run by the coordinator (the task: "the team lead confirmed the go for 2b (2026-09-26)"); his own words are not in the tree |
| D11 | Held 2026-09-25 content pulled in beyond A7 (minor) | REAL for three items, not for two | MOVED OUT (held pass): the worksheet's and `calibrate_program.cpp`'s "the record Teleop booted from came from an unrecorded run" (both now say only what A14 needs: it has run, its reproduction run is owed); G.0.4's recommendation (D4); G.6.5's new question — G.6.5 is RETIRED instead (its old question presupposed a Teleop from the typed table, false for robot two: A14), "What to send back" 6 follows. KEPT, with the reason for the commit body: HA-112's confidence line (it corrects a FALSE "the library has never been driven by a person" with A1's own wording, which names the 2026-09-25 drive) and G.6.4's 44 line (A8 permits it verbatim) |
| D12 | Wording slips: "simulated hardware"; "the library's own code drove"; `robot.hpp` "exactly these two lines" and `static Robot r{…}` advice (minor) | REAL, all three | `README.md`:248 "the host test plant"; `roadmap.md`:345 "own object graph"; `include/shulib/robot/robot.hpp`:16-22 (validate, then `std::optional::emplace` from `initialize()`) and :49-55 (never a `static Robot`: namespace scope precedes the handler; a function-local one whose constructor throws is re-attempted) — landmine 1's reasoning, public; `docs/api/robot.md` regenerated |
| D13 | Station F says nothing about brake vs coast on release (minor) | REAL that nothing is said; the reviewer's predicted behaviour is NOT | The reviewer read `cancel()` -> `setBrakeMode(Brake)` and concluded a released stick "most likely brakes". The vendored PROS header says otherwise: `motor_move_voltage` "will not respect brake modes" (`include/pros/motors.h`:253), and `ProsMotor::setVoltage` only ever calls `move_voltage` (`hal/pros/motor.hpp`:126) — so in THIS build every library stop reaches the motors as plain 0 V, a coast (the DEFECTS2 defect). F.2.3 (:640) now says release COASTS (the team lead's ruling), why, and that the brake-mode fix keeps it so; F.2.7 (:667) says the disable's "0 V + Brake" coasts too until that fix. Stated as read from the code and the vendored header, not seen on the robot |

**D3(f), in full.** `src/calibrate_program.cpp`'s Blue-cartridge default said it matches "the tank graph's
own helper" — main.cpp's `gearsetOf()`, deleted by 2b (the provider's own `gearsetOf` refuses instead of
defaulting). A file 2b edits, now FALSE: corrected at `calibrate_program.cpp`:595-598 (the helper named
as retired, the provider's refusal named), a comment only.

**Rulings made in this round (each with the alternative and why):**

- **G1-2b-R8 — `BootOutcome` has no accessors; the program reads `built`.** *Alternative:* keep the
  forwarders and add a test that builds a `BootOutcome` (the reviewer's second option) — rejected: two
  doors to one hazard with one of them tested by a second, separate test is still two doors; deleting
  them leaves exactly the door test 6 walks through. A9's "the outcome exposes `absent()`" is met by
  `outcome.built.absent()` — the bounded span, from the outcome.
- **G1-2b-R9 — the decision flow is header-inline and PROS-free; the .cpp is a shim.** *Alternative:*
  keep the flow in the .cpp and leave it to reading (the brief scoped test 6 to the core) — rejected:
  "moved with every decision unchanged" then rests on nobody re-reading it, while `IBlockSource`,
  `FakeBlockSource`, `IDeviceRegistry` and `FakeDeviceRegistry` already exist for exactly this.
  *Alternative:* pass the `IBlockSource` into the flow — rejected: the old block CLOSED the card before
  asking the registry; `readRecord()` + the shim's scope keep that order. Printing goes through a
  caller-supplied line printer (the shim's `printf("[%s] %s\n", tag, line)`), so the bytes on serial
  are the old ones and a test can read them.
- **G1-2b-R10 — the source rows end at a whole sentence with ` ...`.** *Alternative:* a third row
  "more on serial" — impossible: the panel's twelve rows end at y=224 and a thirteenth fails the
  `kPanelLastRow` `static_assert`. *Alternative:* cut at the last word with ` ...` (the reviewer's
  first option) — kept only as the fallback: on robot two it would still end `... dead 1). NEW ...`.
- **G1-2b-R11 — the re-zero button is named from its constant and may never be a shoulder button.**
  *Alternative:* `static_assert(kRezeroButton == A)` beside the literal — rejected: it makes the team
  edit two places to change one choice.
- **G1-2b-R12 — F.2.1 keeps the MOTOR WATCH push (the TABLE's front) and drops shulib Drive's 30 s.**
  Reasons in D7's row.
- **G1-2b-R13 — one rule for false run-status lines** (D3's row).

**The mutation campaign — every run on a FRESH binary** (`scratchpad/r2/mutate2.py` +
`mutations2.json`; host: edit -> touch the including tests -> `--target shulib_tests/fast` -> REQUIRE
the `Linking CXX executable shulib_tests` line, or a compile failure counted as killed with its error
shown -> run -> restore -> relink -> green; src: edit -> `rm -rf bin .d` -> `make ROBOT=tank
PROGRAM=library` -> the build must fail -> restore -> rebuild green -> `rm -rf bin .d`). Host run
03:30:10-03:33:01 EDT, src run after it; **29 of 29 killed, every restore green**:

| # | Mutation | Killed by |
|---|---|---|
| M1 | `absent()` spans the WHOLE array (A11) | 6b, 6c, 6d (threw "absent port 0 ..."), 6e, 6f, **6k, 6l** — 7 cases |
| M2 | the core reads the LEFT side twice | 6e, 6f, **6l** |
| M3 | the SIGNED entry recorded | 6f |
| M4 | `tankChassisConfig()` ported into the core | 6b only (`stall.wheels[i].wheelRadius`) — `effectiveTuning()` hides it elsewhere, as before |
| M5 | a non-default kV | 6b, 6d |
| M6 | the LEFT scale into the right | 6b, 6c (both goldens), 6d, 6k |
| M7 | the left scale through `float` | 6b, 6d (every stall slot), 6k |
| M8 | the source hard-coded MEASURED | 6h, **6n, 6q** |
| M9 | `buildOptions()` returns `{}` | 6e, 6f, **6l** |
| M10 | the `usable()` precondition dropped | 6g |
| M11 | the cross-check stops at port 20 | 6i |
| M12 | tracking-wheel odometry | 6b, 6c, 6d, 6e, 6f, 6h, 6k, 6l |
| M13 | `robot.hpp` `describe()`: one ` \| ` becomes ` / ` (the reviewer's) | 6c's `describe()` golden |
| D1 | a TYPED boot keeps the record's pointer (`chosen = &fromTable` deleted) | 6n, 6q (`requireSameDescription`) |
| D2 | the chosen table is not re-verified | 6n (the warning line), 6p, 6q, 6r |
| D3 | the cross-check condition inverted | 6k, 6l, 6n, 6p, 6q |
| D4 | the EMPTY-file sentence removed (R7) | 6p |
| D5 | a zero-port cross-check printed (`> 0` -> `>= 0`) | 6k's agreeing-table half |
| D6 | a refused record's reason replaced by the table's | 6m (both subcases) |
| D7 | a table that fails its own port check is built anyway | 6r (the core's precondition throws) |
| D8 | a refused record falls back to a usable table | 6m, 6o |
| P1 | provenance bound `n + 1` -> `n` (the NUL past the buffer) | chassis table 3 (c), (d) |
| P2 | provenance bound `n + 1` -> `n + 2` | chassis table 3 (b), (c), (d) |
| A1 | `bool reportOnly` after `OdoStallCheckConfig::independentMotionSource` (tail padding) | COMPILE: only the member-count pin (`boot_source_test.cpp`:246) — the three size pins stayed silent, which is the reviewer's point proven |
| A2 | `uint32_t extraMask` after `MotionSchedulerConfig::abortFaultMask` (padding) | COMPILE: only the member-count pin (:250) |
| S1 | `kRezeroButton` = L1 | ARM COMPILE: `teleop_program.cpp`:146 |
| S2 | the profile's yaw deadband 0.08 under the driver line | ARM COMPILE: :195 |
| S3 | `sourceRows()` without the sentence search | ARM COMPILE: :428 (robot two's rows) |
| S4 | the ` ...` marker empty | ARM COMPILE: :428 and :443 |

The code these ran on is the final code: the only edits after the host run are two COMMENTS
(`src/boot_source.hpp`'s `kBootLineBytes` byte count; `test/boot_check_test.cpp`'s pointer to the flow)
and documentation.

**The A10 numbers, re-derived mechanically** (`scratchpad/r2/lens2.py`, on the final source; robot
two's 2026-09-25 record):

| Line | Bytes | Logged before |
|---|---:|---:|
| `kProvisionalLine` | 193 | 193 |
| `kAbsentDevicesLine` (row 22) | **184** | 179 |
| `kDeadPortLine1` / `kDeadPortLine2` | 164 / 133 | 164 / 133 |
| driver line (from constants) | 189 | 189 |
| `kOpcontrolLive` | 174 | 174 |
| `HARDWARE-UNVALIDATED: ` + `kRunStatus` (328) | 188 + 165 | 188 + 165 |
| `kNoIndependentStallSourceNote` | **333** -> 194 + 142 | "332" -> 194 + 142 |
| `port check: <OK reason>` | 191 + 28 | 191 + 28 |
| `robot::Robot: <describe()>` | 193 | 193 |
| log pieces (`kLogPieceCols`) | **196** | "190" |

**The widened greps, as run** (case-insensitive this time — 2b.5's were case-sensitive and missed
`NO camera`; living documents only, as 2b.5):

```sh
P='never been run|never run|has not run|not yet run|unrun|no camera|autons land|wiring|object graph|main\.cpp|Part 2\b|composition root|hand-wired|TankRobot|TankMotors|TankPanel|tankPortMapString|tankChassisConfig|runTeleopLoop|mapSticksToChassisSpeeds|deadband-only|simulator|simulated|`table |table L -|"table '
grep -rniE "$P" README.md docs/*.md docs/guide docs/internal/{build-order,RESUMING,HANDOFF-remaining-work,PROJECT-BRIEFING}.md docs/internal/chunks/R3a-BENCH-WORKSHEET.md include src test tools Makefile common.mk .github
```

750 hits in 144 files; 470 of them in the 37 files 2b edits, and every one of those was read.
Beyond D3(a)-(f), FIXED as false in an edited file: `src/chassis_table.hpp`:241, :255 (the
"composition root paints `buf` on the state screen" — only serial prints it, and did before 2b);
`roadmap.md`:248 "It has NOT run" (annotated as true of 2026-09-14, with A1's wording for since) and
:1594 "never been run against a physical SD card" (false since 2026-08-18/19); HA-125's source (the
decision lives in `boot_source.hpp` now; `describeMissingForDescription` feeds the `typed table:` line,
not the refusal); the `src/boot_source.cpp` pointers in `Makefile`:91, `main.cpp`:29/:206/:560 and
`test/boot_check_test.cpp`:406. Left as TRUE: "simulator"/"simulated" in text 2b did not write (the
house rule "host test plant" applies to what is written now); every generic "wiring", "object graph",
"composition root", "never run" (a callback, a task); every dated record (`build-order.md`:230, :798;
`roadmap.md`:237, :1434 — the last is the 2026-09-25 odometry evidence, a held-pass item).

**Owed to the HELD documentation pass (A7), added this round** (2b.5's list stands): `docs/guide/14`
:19-25 is incomplete, not false (it names 2026-09-17's drive, not 2026-09-25's); `roadmap.md`:1434
("not yet run on a robot" inside a dated DONE note — drive-encoder odometry ran on robot two on
2026-09-25); `README.md`:105-109 (the hardware list stops at 2026-09-17); the worksheet's and
`calibrate_program.cpp`'s record-provenance sentence (D11, moved out); `docs/guide/03`:249 and
`include/shulib/hal/absent_tag_source.hpp` are true and stay.

**For the commit body — deliberate scope beyond A7's pointers, with the reason:** HA-112's confidence
(A14: a FALSE line corrected with A1's wording); G.6.4's 44 line (A8); G.6.5 retired (A14: its
question presupposed a Teleop from the typed table); F.2.3/F.2.7's coast statements (the team lead's
ruling that Teleop coasts, and D13); the order ruling in `build-order.md`'s deviations table (D1).
Deferred: the held pass (2b.5 + this list); DEFECTS2's own entry; the `DriverLoopConfig` L1 default
(R2, HANDOFF L5).

**Listed, not done (each outside 2b's scope or a behaviour the move keeps):** the refusal's reason is
cut at 100 characters of a refused record's sentence, with no mark on the screen (moved unchanged from
`main.cpp`, now pinned by 6m; a wider cut would change the refusal text COMP will share — its brief
should decide); the program's format wrappers are compile-only (T7); the `DriverLoopConfig` default
(R2).

**Every gate, re-run on the final tree (03:50-04:01 EDT):**

| Gate | Command | Result |
|---|---|---|
| Host build, relink required | `touch test/*.cpp && cmake --build build/test --target shulib_tests/fast` | exit 0, `Linking CXX executable shulib_tests` |
| Host suite | `./build/test/shulib_tests` | **1444 test cases / 2,387,352 assertions, 3 skipped — SUCCESS** = 2b.8's 1435 / 2,387,098 + test 6k-6r (8 cases) and chassis table 3 (1 case, 9 assertions); test 6 is now 18 cases / 472 assertions (was 10 / 227) |
| src build gate self-test | `python3 tools/src_build_gate.py self-test` | `OK (33 detector cases, each against a real build)` |
| src build gate | `python3 tools/src_build_gate.py check` | **PASS** — six builds, 6/6 TUs each, 0 disallowed; the allowed `-Wunused-function` 2/0/2/2/2/2 (bench, xdrive, tank, tank-drive, tank-library, tank-calibrate) — the variant differential unchanged; hashes and beacons asserted. The calibrate package carries no `decideBootSource` symbol and no `CROSS-CHECK: the MEASURED` string; the tank-library package carries both, the EMPTY-file sentence and the `re-zero button` line (measured, then `bin/` and `.d/` removed) |
| ARM header gate | `arm-none-eabi-g++ ... -Werror` over every `include/shulib` header | exit 0 (184 headers) |
| CI greps | `<pros/>` outside `hal/pros`; `shulib/sim/` outside `sim/` | both print nothing |
| API docs | `api_doc_tool.py generate`, then `check-fresh` | exit 0 (`robot.md`, `teleop_loop.md` regenerated for this round's comments) |
| Removability | `api_doc_tool.py check-removability` | `removability: no public doc references docs/internal/` (a first draft of the `DriverLoopConfig` comment named HANDOFF; reworded before this ran) |
| Doc staleness | `python3 tools/doc_staleness_audit.py` | `clean (26 live docs, 0 numeric claims checked)` |
| Briefing | `briefing_status.py generate`, then `check` — LAST | `regenerated` (**Next up** = the brake-mode fix, then T3, then G1 PART 2c, then COMP; **Suite** = 1,444 cases / 2,387,352 assertions); `check` -> `generated block current`, exit 0. Then the full `cmake --build build/test` ran every doc gate |

Packages: none uploaded; the two measured above were built only to read their symbols, from this
working tree, and deleted (landmine 9 — a robot's packages come from the COMMITTED tree). Nothing
committed, staged or pushed.

**State at the end of this round:** Part 2b is COMPLETE in the working tree and UNCOMMITTED; THIS
VERSION of "shulib Teleop" has not run on hardware. The coordinator verifies (§7), and commits with a
body that names the scope beyond A7 above and the held pass as deferred.

### 2b.10 The COORDINATOR's verification (not the executor's), before the commit

- **The team lead's go for Part 2b, confirmed in his words (2026-09-26 ~01:55 EDT): "yes please do part
  2b?"** — after "I am ready" (2026-09-25 ~22:47, in answer to "Say go when you want Part 2b") and his four
  answers at 00:21. 2b.0a's inference is thereby confirmed, not corrected.
- **History of the run, stated plainly:** the first executor (00:32) stopped at 2b.0 with an API error on
  the service side (no code); the three first-round reviewers therefore reviewed nothing; the second
  executor (01:16) built the part; a SECOND, full review round (robot behaviour / test strength / docs,
  02:44-03:0x) ran on the real change — no blocker, no major on the robot lens — and its 23 findings were
  dispositioned in 2b.9.
- **Every gate, re-run by the coordinator** (scratchpad `verify.sh`, ~04:1x): host build with the doc gates
  inside it; suite `1444 | 1444 passed | 0 failed | 3 skipped`, `2387352 | 2387352 passed` (the
  executor's numbers exactly); `src_build_gate.py self-test` OK (33) and `check` PASS on all six builds,
  6/6 TUs each, 0 disallowed, hashes asserted, beacons checked; ARM header gate over 184 headers exit 0;
  both CI greps empty; `check-fresh` 0; briefing current; the removability grep empty.
- **Two hand mutations of `src/boot_source.hpp`** (touch `test/*.cpp`, the `/fast` target, the
  `Linking CXX executable shulib_tests` line required, `-sf="*boot_source_test.cpp"`, restored with `cp` and
  checked with `cmp`, relinked green at 1444/2387352): `absent()` spanning the WHOLE storage array (the
  "absent port 0" trap) → 7 of 18 boot-source cases red, 8 assertions; `absent()` dropping one port → 3
  red. Both caught.
- **Read by the coordinator:** `teleopInitialize` / `teleopAutonomous` / `runTeleop` (`src/teleop_program.cpp`)
  — the banner and competition line from the arguments main.cpp passes (A2); the boot decision printed
  before any adapter; `validate()` and the construction inside the graph-boundary `try`, into the
  `std::optional`s, once; `attachFaultLatch` after the Robot exists; `teleopAutonomous` logs one Warn and
  commands nothing (A4); the driver loop with the re-zero button A (static_assert: never L1/L2/R1/R2),
  ArcadeBody, field-centric off. The status pointers after this commit (build-order's first `Next:`,
  roadmap "Now") name the team lead's current order: DEFECTS2 → T3 → 2c → COMP.
- **Carried to T3 (rulings made after this part was briefed):** the pneumatic claw on A (2026-09-26 ~03:20)
  moves the re-zero button from A to **Y** — done at T3's merge, with the boot line (now formatted from the
  constant) following automatically; the library's `DriverLoopConfig::rezeroButton` default of L1 (a claw
  pivot's button) is owed a change with T3/COMP (HANDOFF L5).

---

## Part 2c — 2026-09-26 09:39:58 EDT

> **Part:** G1 Part 2c — `src/main.cpp` becomes a dispatcher; the invented X-drive becomes a
> `config::RobotConfig` constant built through `ProsDeviceProvider` + `robot::Robot`; the state screens
> move out; the src build gate's variant differential is redesigned. **Tree at start:** `ea58fbc` (branch
> `g1-2c`, an isolated worktree at `.claude/worktrees/g1-2c`, clean). **Executor model:** opus.
> **Brief:** [`G1-part2-pros-provider-and-main.md`](G1-part2-pros-provider-and-main.md) §2c (IN items
> 1-5 and its documentation list), §4 test 7's 2c part, §5 DoD 2c, §6 landmines (10-11 included), §7,
> and §2b's amendments A1-A19 for context. **The team lead's go:** "start all three" (2026-09-26 morning,
> recorded in `build-order.md`'s deviations table by `ea58fbc`: 2c, DIAG1 and COMP-a built in parallel in
> isolated worktrees while T3 finishes). Appended in real time; every timestamp from a `date` call.
> Nothing here is committed, staged or pushed by the executor; `pros` is never run and `/dev/ttyACM*`
> never touched. `G1-COMPLETED.md` is the coordinator's, not this run's. No hardware claim.

### 2c.0 Log created (09:39:58 EDT)

Log section created before any code. Next: re-check §2c against the tree at `ea58fbc`, then the baseline.

### 2c.1 §2c re-checked against the tree at `ea58fbc` (09:40-09:55 EDT) — what changed since it was written

§2c was written at the 2026-09-24 revision (`a2c9fff`); the 2026-09-26 re-check (A1-A19) covered 2b only.
Since then G1 Part 2b (`7f9b9a1`) and DEFECTS2 (`a8540e6`) landed. Read in full before any code:
`src/main.cpp` (735 lines), `src/teleop_program.{hpp,cpp}`, `src/fault_latch_hook.hpp`,
`src/screen_text.hpp`, `tools/src_build_gate.py`, `Makefile`, `robot/robot.hpp`,
`config/robot_config.hpp`, `hal/pros/device_provider.hpp`, `teleop/teleop_loop.hpp`, and the tests that
already pin the X-drive's builder path (`test/robot_builder_test.cpp` cases 6, 6b, A5).

| §2c item | At `ea58fbc` | Consequence |
|---|---|---|
| 1 state screens | `competitionBits`/`benchStateScreen` still in `main.cpp` :246-288, guarded by `SHULIB_STATE_SCREENS` (so the xdrive build emitted no unused-function). `teleop_program.cpp` calls NEITHER — main.cpp formats the bits and passes them (A2), and paints `kTeleopAutonomousReason` itself | the move needs NO change to `teleop_program.cpp` (T3's file): confirmed, none made |
| 2 X-drive as data | the graph is `main.cpp` :290-445 (`struct Robot`, `robot()`, `portMapString()`), its loop `runTeleopLoop` :460-492 — **DEFECTS2 added `coastDrive()` to it at merge** — and its branches :582-620, :677-680, :727-733. `teleop::DriverLoopConfig::releaseBrakeMode` (default Coast) exists since DEFECTS2. `robot::writePortMap()`'s X-drive format is `portMapString()`'s (2a, pinned by `robot_builder_test` A5). No `constexpr RobotConfig` instance exists in the tree (grep: comments only) | the X-drive's driver control goes onto `runDriverControl` with `releaseBrakeMode = Coast` stated; **NEW FINDING D-2c-1**: `robot::Robot` installs a GPS corrector for a config with a GPS; the hand-wired X-drive had none (`Localizer{clock, imu, odom, fusion}`) — the brief does not mention it (ruling G1-2c-R3 below) |
| 3 dispatcher | after 2b `main.cpp` forwards Teleop; the tester's, Drive's and Calibrate's boot banners are still INLINE in `initialize()` (:517-568), not forwarded | the banners move into their programs (`testerInitialize`, `driveInitialize`, `calibrateInitialize`) — "forwarding" is §2c.3's word (ruling G1-2c-R2) |
| 4 gate redesign | `XDRIVE_WIRING_DEAD_IN` :202, the differential :443-458, the header :91-115, `Makefile` :15-23 — as the brief says. **NEW FINDING D-2c-2 (load-bearing):** `main.cpp` selects the PROGRAM in one `#if` lattice (:127-144, `SHULIB_RUNS_*`, plus a nested `#if defined(SHULIB_PROGRAM_DRIVE)` at :155, :522, :709) and the BEACON in a SEPARATE chain (:226-244). The brief's premise — the beacon "already covers" a broken `#ifdef` — holds only for the beacon's own chain. MEASURED (scratchpad `old_lattice_probe.py`, the `ea58fbc` main.cpp built as xdrive with the LATTICE's `#if defined(SHULIB_ROBOT_XDRIVE_INVENTED)` misspelt): `exit 0`, gate verdict **PASS**, `runR3a` linked, the xdrive beacon present — the build was silently the bench tester and ONLY the retired differential saw it (2 `-Wunused-function` from `main.cpp` in an xdrive build). Removing the differential as the brief words it would have opened that hole | the redesign makes the beacon complete: ONE chain selects the program and the beacon in the same branch, enforced by a new source check (ruling G1-2c-R1) |
| 5 ≤ 300 lines + grep | — | met (2c.3) |
| docs | 2b already rewrote README's "Wiring a `Chassis`" paragraph and guide 07's `src/` sentence; guide 08's "(they exist; src/main.cpp wires them)" (:167, quoted VERBATIM from `test/guide_examples_test.cpp`) and :52/:156/:217 were LEFT for 2c by 2b.5 | added to 2c's documentation list |

Also re-checked: `chassis_table.hpp`:77 selects robot two's table on its OWN `#if defined(SHULIB_ROBOT_TANK_2026)`
— a second selection outside `main.cpp` that neither the retired differential (tank looked like bench to it) nor
the beacon can see. Pre-existing; NAMED in the gate's header as the residual and allowlisted by the new check
(not fixed — outside §2c). Nothing in §2c needs a decision only the team lead can make; every deviation is
recorded below with its alternative.

### 2c.2 Baseline on the clean tree (09:41-09:44 EDT)

`cmake -S test -B build/test && cmake --build build/test -j6` (full build, `Linking CXX executable
shulib_tests`), then the suite: **1464 test cases / 2,387,533 assertions, 3 skipped — SUCCESS** (DEFECTS2's
number). `python3 tools/src_build_gate.py check`: PASS, six builds, 6/6 TUs each, `-Wunused-function` 2/0/2/2/2/2
(bench, xdrive, tank, tank-drive, tank-library, tank-calibrate), 0 disallowed, beacons checked (1 m 34 s).
`src/main.cpp` = 735 lines.

### 2c.3 The code (09:55-10:05 EDT)

| File | Before | After | What |
|---|---:|---:|---|
| `src/main.cpp` | 735 | 275 | THE DISPATCHER: the `#error` checks; `enum class Program` and THE ONE CHAIN (`enum class Program` :89, the chain :110-128 — `kProgram` + `kVariantBeacon` in the same branch); `robotPreconditionHandler` :137 + `attachFaultLatch` :155; the five entry points forwarding to `shulib::bench::` (`initialize` :166 and `autonomous` :223 and `opcontrol` :257 as a `switch (kProgram)`; `disabled` :193 and `competition_initialize` :205, where only the X-drive differs, as an `if`). The X-drive graph, `robot()`, `portMapString()`, `runTeleopLoop`, the state screens, every `SHULIB_RUNS_*`/`SHULIB_STATE_SCREENS` macro and the three inline boot banners are GONE |
| `src/state_screens.{hpp,cpp}` | — | 54 + 47 | NEW: `competitionBits()` and `benchStateScreen()` moved from `main.cpp` (§2c.1), in `shulib::bench`; output unchanged (2c.4); every screen call's format is now the literal `"%s"` |
| `src/xdrive_config.hpp` | — | 75 | NEW: `kInventedXDrive`, an `inline constexpr config::RobotConfig` — the tree's first — with main.cpp's numbers; HA-111 in `identity.name` (§2c.2) |
| `src/xdrive_program.{hpp,cpp}` | — | 39 + 328 | NEW: `xdriveInitialize(beacon)` / `xdriveAutonomous()` / `runXdrive()` — `ProsDeviceProvider` → `validate()` → `Robot` emplaced once inside the graph-boundary `try` → `attachFaultLatch` → session header from `portMap()` → boot lines; `runDriverControl` (default profile ArcadeBody, re-zero Y, release COAST) with the controller fault display per tick; a refusal screen through `printText` |
| `src/bench_r3a.{hpp,cpp}` | 170 / 3,248 | 178 / 3,264 *(corrected at 2c.10 F8: this row said 181)* | `testerInitialize(beacon, bits)` — main.cpp's tester banner, verbatim; the header's X-drive sentence corrected |
| `src/drive_program.{hpp,cpp}` | 36 / 670 | 43 / 687 | `driveInitialize(beacon, bits)` — main.cpp's Drive banner, verbatim |
| `src/calibrate_program.{hpp,cpp}` | 27 / 1,335 | 35 / 1,358 | `calibrateInitialize(beacon, bits)` — main.cpp's Calibrate banner, verbatim (the stamp now this TU's `kBuildStamp`) |
| `src/fault_latch_hook.hpp` | 29 | 30 | comment: the X-drive is the second caller |
| `src/teleop_program.{hpp,cpp}`, `src/boot_source.*` | | | **NOT touched** (T3's files): the move did not force a change — Teleop calls neither state-screen function |
| `tools/src_build_gate.py` | 719 | 848 *(corrected at 2c.10 F8: this row said 720 → 849)* | the differential retired; step 0 (the one-chain source check); self-test cases 16-19; header rewritten (2c.7) |
| `Makefile` | | | the WARNFLAGS rationale rewritten; the xdrive line |
| `test/xdrive_config_test.cpp` | — | 321 | NEW: 3 cases (2c.7) |

**The §2c item 5 grep** — `grep -nE 'MotorGroup|Localizer|RobotContext|MotionDeps|screen_print' src/main.cpp`
prints nothing (count 0), comments included. `wc -l src/main.cpp` = **275** (≤ 300).

**Package contents, measured** (`arm-none-eabi-nm -C bin/hot.package.elf` after a single `make`, 09:58 EDT): the
`switch (kProgram)` on a constant folds — the xdrive package carries `xdriveInitialize`/`runXdrive` and NONE of
`teleopInitialize`, `runTeleop`, `runR3a`, `testerInitialize`, `runDrive`, `driveInitialize`, `runCalibrate`,
`calibrateInitialize`, `benchStateScreen`, `competitionBits`, `decideBootSource`; the tank-library package carries
the Teleop symbols, `benchStateScreen`, `competitionBits`, `decideBootSource` and no X-drive, tester, Drive or
Calibrate entry; the bench package carries `runR3a`/`testerInitialize`, the state screens, and no Teleop or X-drive
symbol. The dead cases of a switch on a `constexpr` are not emitted at `-Os`, so every build still links only its own
program.

**Rulings made while building (each with the alternative and why):**

- **G1-2c-R1 — ONE chain selects the program AND the beacon; the entry points `switch` on a C++ enum; a source check
  enforces it.** Finding D-2c-2 (2c.1) measured that the brief's premise was false for the tree as it stood: the beacon
  lived in a second chain, so retiring the warning-count differential alone would have left a broken program-lattice
  `#if` invisible to every detector. *Alternative A:* keep two chains and add `-Wundef` so a misspelt DERIVED macro
  (`#if SHULIB_RUNS_TANK_LIBRARYY`) warns — rejected: it does not see a misspelt DEFINE in the lattice (the probe's
  case: `defined(X_BROKEN)` is simply false, no warning), which is the class the differential existed for.
  *Alternative B:* derive the beacon from the lattice's macros (`#if SHULIB_RUNS_TANK_LIBRARY` selects the beacon) —
  rejected: still two layers of macros, and a typo in a macro USE stays silent. *Chosen:* the defines are read in
  exactly one chain whose every branch sets both `kProgram` and `kVariantBeacon`; below it, C++ — a misspelt
  enumerator is a compile error and an unhandled one is `-Wswitch`, which the gate's policy FAILS; and step 0 of
  `check` rejects any other selecting `#if` in `src/` (one named exception, `chassis_table.hpp`). Measured against the
  OLD `main.cpp`: step 0 reports 11 lines (both chains and the nested `PROGRAM_DRIVE` ifs).
- **G1-2c-R2 — the tester's, Drive's and Calibrate's boot banners move into their programs** (`testerInitialize`,
  `driveInitialize`, `calibrateInitialize`, each taking the beacon and the bits the way `teleopInitialize` does since
  A2). *Alternative:* keep them inline in `main.cpp` — it would still fit 300 lines (~310 estimated, tight), but §2c.3
  says the entry points FORWARD and "no boot logic"; a banner is the program's own words. The bytes are unchanged
  (2c.4). *Alternative:* one shared `boot_banners.cpp` — rejected: a program's run status belongs beside the program
  (Calibrate's banner carries Calibrate's run history).
- **G1-2c-R3 — the X-drive config declares its GPS PRESENT, so the builder adds a GPS corrector the hand-wired graph
  lacked (finding D-2c-1).** *Alternative:* `gps.present = false` to keep the old corrector-less graph — rejected: it
  drops `GPS9` from the port map and the GPS from the context, describing a different robot than HA-111's map; the
  builder's rule (GPS present ⇒ corrector) is the library's and `robot_builder_test` 6b pins that graph against a
  hand-wired one. Neither graph has booted on a robot over real adapters *(corrected at 2c.10 F5 — this said "No
  X-drive has booted either way"; the old graph over FAKE devices booted on a V5 on 2026-08-12)*. Said in
  `xdrive_config.hpp`'s header and pinned by test 2c-2.
- **G1-2c-R4 — `constexpr`, proven.** `inline constexpr config::RobotConfig kInventedXDrive{…}` compiled on the host
  and at `-Os` for the Cortex-A9 (a scratch probe first, 09:47); `test/xdrive_config_test.cpp` reads it in
  `static_assert`s, including a `constexpr` copy of the whole value. Mutation X9 (`constexpr` → `const`) fails that
  file's compile.
- **G1-2c-R5 — the X-drive's driver control: the library's DEFAULT profile (ArcadeBody), field-centric OFF, re-zero
  button `Y`, release mode `Coast` stated, not defaulted.** ArcadeBody reads the left stick's X as strafe (`left =
  -leftX`, `drive_modes.hpp`), exactly the old mapping's axes, so an X-drive keeps its strafe. *Alternative for the
  button:* `A`, Teleop's today — rejected: T3 gives robot two's `A` to the pneumatic claw and moves Teleop's re-zero to
  `Y`; choosing `Y` now means the two programs agree once T3 lands. A `static_assert` forbids a shoulder button, another
  pins the boot line's words to the profile, the button and the release mode.
- **G1-2c-R6 — the X-drive refuses AT ITS GRAPH BOUNDARY, on screen and serial, instead of throwing out of
  `initialize()`.** The old graph's adapters threw an uncaught precondition from a function-local static (main.cpp's
  comment called that throw "bench runbook step 1's measurement"). The provider now asks the registry first and
  `validate()` names the port; the program paints `X-DRIVE REFUSED AT BOOT` with the reason and "Every port in this
  build is INVENTED (HA-111)". *Alternative:* no screen, serial only (the X-drive never painted a screen) — rejected:
  §2c.2 says it "still says so", and a person who uploaded it sees the brain, not the log.
- **G1-2c-R7 — `benchStateScreen` keeps its own prints, converted to the literal-`"%s"` form, and does NOT go through
  `printText()`.** *Alternative:* route it through `printText()` — rejected: the SMALL budget (50) would cut three of
  its fixed lines (53-54 characters) and every reason `main.cpp` passes (66-126), changing what the brain shows; 2c
  changes nothing a robot shows. *Alternative:* move it verbatim (literal text as the format) — rejected: the standing
  rule is that only a literal `"%s"` reaches a screen print; no literal contains a `%`, so the pixels are identical.
- **G1-2c-R8 — the X-drive's heading source is written a second time** (the same three-part trust rule as Teleop's
  `headingSample`). *Alternative:* a shared helper — rejected FOR NOW: it would edit `src/teleop_program.cpp`, which
  T3 is changing in parallel. Owed as a follow-up once T3 lands (listed at the end).

### 2c.4 What each build SAYS — the source-level diff, old (`ea58fbc`) against new (10:15 EDT)

There is no hardware to run it on, so the diff is of the source strings. For the four programs that run on robot two
it is MECHANICAL: `scratchpad/bannerdiff.py` extracts every C string literal of each `printf` statement (adjacent
literals joined, comments stripped) from the old `main.cpp` block and from the new function, and compares:

| Build | Old | New | Result |
|---|---|---|---|
| bench / tank ("Bench Tests") | `main.cpp` :530-542 | `testerInitialize()`, `bench_r3a.cpp` | both statements IDENTICAL (banner; `competition status at boot`) |
| tank-drive ("shulib Drive") | `main.cpp` :523-528 + :537-542 | `driveInitialize()`, `drive_program.cpp` | both IDENTICAL — including the second line's `[R3A] ... the tester runs only in DRIVER CONTROL`, which this program has always printed (listed below, not changed) |
| tank-calibrate | `main.cpp` :550-566 | `calibrateInitialize()`, `calibrate_program.cpp` | all four statements IDENTICAL; the stamp argument is now this TU's `kBuildStamp` (`__DATE__ " " __TIME__`, as `kCalibrateBuildStamp` was) |
| tank-library ("shulib Teleop") | — | — | unchanged: `teleop_program.cpp` not touched; main.cpp still formats and passes the beacon and the bits |
| every build but xdrive — the state screens | `main.cpp` :259-287 | `state_screens.cpp` | `competitionBits`' format and the serial line IDENTICAL; the four fixed screen lines and `status bits %s` are the same TEXT, now passed through a literal `"%s"` (ruling G1-2c-R7) |
| every build — the `disabled()` / `competition_initialize()` / `autonomous()` reasons | `main.cpp` :630-673 | `main.cpp` :185-233 | the same five literals, the same program for each (incl. Drive's tester sentence) |

The X-drive build (this version never booted — the one X-drive boot on record is the old graph over FAKE devices,
2026-08-12, corrected at 2c.10 F5; every line below is source, none has been seen on a brain):

| # | Old (`main.cpp` at `ea58fbc`) | New (`xdrive_program.cpp`) | Status |
|---|---|---|---|
| 1 | `[R3B]` Info: the beacon (:584) | `xdriveInitialize` — `log(Info, "R3B", variantBeacon)` | SAID |
| 2 | the §18.5 session header, `routineId "(none: teleop only)"`, port map `portMapString()` (:592-598) | same fields; port map `robot.portMap()` | SAID — the port map text pinned equal to `portMapString()`'s by test 2c-2 |
| 3 | `[R1A]` "shulib v2 core wired over hal/pros adapters: X-drive kinematics + Pilons odometry + fused localizer + motion scheduler + Chassis facade" | `kWiredLine` "...wired by robot::Robot over hal::pros::ProsDeviceProvider (G1 Part 2c): X-drive kinematics + Pilons odometry + GPS corrector + ..." | REWORDED — the graph is built differently and gained the corrector (R3) |
| 4 | — | `[R1A]` "robot::Robot: <describe()>" (the text pinned in test 2c-2; 160 bytes with its prefix) | NEW |
| 5 | `[R1A]` Warn "HARDWARE-UNVALIDATED: adapters are host-tested against a PROS shim only; port map is invented (HA-111); R3 owns first motion" | `kUnvalidatedLine` "HARDWARE-UNVALIDATED: no X-drive has ever booted -- every port and sign in this build is INVENTED (HA-111); no library-commanded motion has run on any robot (R3b open)" | REWORDED — "host-tested against a PROS shim only" is FALSE since the adapters first ran on a brain (R1a's bench, 2026-08-13; robot two, 2026-09-17 and 09-25) |
| 6 | `[R3B]` absent devices (:610-612) | `kAbsentDevicesLine` | SAID, byte-identical |
| 7 | `[R1A]` "facade alive: strafeAuthority=%.2f (X-drive: 1.00)" | same format | SAID (test 2c-2 pins 1.00) |
| 8 | — | `[R1A]` the driver line (profile, button Y, COAST) | NEW |
| 9 | `[R1A]` opcontrol: "teleop drive loop live (deadband-only mapping, HA-112, shulib/teleop/stick_mapping.hpp; driver-feel shaping is chunk T2)" | `kOpcontrolLive` "...(teleop::runDriverControl, DriveShaper default profile ArcadeBody, release COAST; HA-112, unfelt)" | REWORDED — the old loop is gone; the old words would be false |
| 10 | `[R1A]` Warn autonomous: "no motion until R3 validates the stack on hardware (adapters landed at R1a; the register's R3 group is unsettled)" | `kAutonomousLine` "autonomous(): NO motion here, ever -- autonomous routines run only in the competition program's autonomous() (the team lead's ruling, 2026-09-25)" | REWORDED — the old line promised motion here later, which the 2026-09-25 ruling (A4) forbids |
| 11 | an uncaught `PreconditionError` out of `initialize()` on any real robot | `X-DRIVE REFUSED AT BOOT` screen + `[R1A] X-DRIVE REFUSED AT BOOT: <why> -- every port in this build is INVENTED (HA-111); nothing is powered` | NEW (ruling R6); the exact `<why>` on robot two is pinned by test 2c-3 |

Every fixed new line is `static_assert`ed ≤ 200 bytes (TermSink's cut).

### 2c.5 The prose review — run BEFORE this was written (10:05-10:25 EDT)

**The greps, as run** (living documents only, as 2b.5/2b.9: `README.md`, `docs/*.md`, `docs/guide/`,
`build-order.md`, `RESUMING.md`, `HANDOFF-remaining-work.md`, `PROJECT-BRIEFING.md`, the worksheet; and the comments of
`include/`, `src/`, `test/`, `tools/`, `Makefile`, `common.mk`, `.github/`; per-chunk logs and briefs are ledgers of
their day):

```sh
# 1. the words 2c retires, in code and comments
P='runTeleopLoop|portMapString|SHULIB_RUNS_|SHULIB_STATE_SCREENS|XDRIVE_WIRING_DEAD_IN|variant differential|warning-count|unused-function|benchStateScreen|competitionBits|kCalibrateBuildStamp|struct Robot|hand-wired|X-drive graph|X-drive wiring|x-drive graph|invented X-drive|main\.cpp'
grep -rnE "$P" include src test tools Makefile common.mk .github      # 136 hits outside the new files, all read
# 2. the same family, case-insensitive, in the living documents
P='main\.cpp|runTeleopLoop|portMapString|X-drive graph|X-drive wiring|invented X-drive|hand-wired|dispatcher|Part 2c|variant differential|unused-function|state screen|benchStateScreen|competitionBits|xdrive|constexpr RobotConfig|composition root'
grep -rniE "$P" README.md docs/*.md docs/guide docs/internal/{build-order,RESUMING,HANDOFF-remaining-work,PROJECT-BRIEFING}.md docs/internal/chunks/R3a-BENCH-WORKSHEET.md
# 3. the brief's §5 list, case-insensitive, everywhere above (461 hits; every one naming main, the X-drive, 2c or the retired loop read)
P='TankRobot|runTeleopLoop|composition root|wiring|hand-wired|object graph|mapSticksToChassisSpeeds|deadband-only|Part 2\b|Part 2c'
```

**FIXED — false after 2c, or found false in a file this part edits:**

| File | What was false | Now |
|---|---|---|
| `include/shulib/robot/robot.hpp` :84-86, :336-337, :575-576, :862-863 | "src/main.cpp still wires its invented X-drive graph by hand until … 2c"; "the X-drive graph in main.cpp still prints its own until 2c"; `portMapString()` "has"; "what main.cpp's X-drive graph does" | the X-drive built the same way since 2c; "retired … had"; "did until G1 Part 2c built it here" |
| `include/shulib/config/robot_config.hpp` :72-73 | the X-drive "must be expressible as data (Part 2)" | it is, since 2c — the tree's first `constexpr RobotConfig` |
| `include/shulib/diag/session_info.hpp` :26-29 | "the invented X-drive graph in src/main.cpp still builds its own by hand until G1 Part 2c" | no program builds a port map by hand |
| `include/shulib/hal/pros/device_provider.hpp` :329-330 | "(the invented X-drive's still does)" | until 2c, which builds it over this provider |
| `include/shulib/teleop/stick_mapping.hpp` :133-138 (a doc comment; landmine 7 forbids the BODIES) | `runTeleopLoop` "now called only by the invented X-drive graph"; the function "stays for the X-drive loop until G1 Part 2c" | the loop is deleted; no program calls `mapSticksToChassisSpeeds` (grep of `src/`: none); it stays for its pin |
| `src/bench_r3a.hpp` :7-11 | the X-drive's "every adapter ctor read-back would throw at boot" | the provider refuses its first port; it lives in `src/xdrive_program.cpp` |
| `src/fault_latch_hook.hpp` :9-13 | one caller | two (Teleop, the X-drive) |
| `test/robot_builder_test.cpp` :23, :731, :1375 | "src/main.cpp still wires its X-drive graph by hand"; "main.cpp's X-drive graph does"; `portMapString()` present tense | past tense, 2c named (test NAMES unchanged, as 2b did) |
| `test/guide_examples_test.cpp` :166 + `docs/guide/08` :167 (quoted VERBATIM — both edited identically; the verbatim gate passed) | "(they exist; src/main.cpp wires them); here it is the simulated robot" | "(they exist; a device provider supplies them); here it is the host test plant" (the house word; re-verified at 10:41: the full build's verbatim gate "408 quoted lines, all verbatim", suite unchanged) |
| `test/routine_signature_pin_test.cpp` :385, `test/f6_signature_pin_test.cpp` :418 | "main.cpp TODO(R1)" / "main.cpp carries a TODO(R1) to emit it" — FALSE since R1a (no such marker; `diag::SessionInfo` has no version field — read) | the token is meant for the header; no program prints it; the correction dated |
| `README.md` :246-249, :276-280, :291-292 | "`src/main.cpp` still wires its X-drive variant that way"; the tree's `src/` line; "`src/main.cpp` is the other deliberate PROS toucher" | no program wires a graph by hand; main.cpp only chooses; `src/` is the other toucher |
| `docs/guide/07-getting-set-up.md` :122-131 | main.cpp "wires the invented X-drive by hand, the worked example of constructing a whole robot" | main.cpp chooses; Teleop the worked example; `xdrive_config.hpp` a robot typed by hand |
| `docs/guide/08-your-first-routine.md` :50-53, :155-157, :216-218, :306-311 (2b left these for 2c) | "On a real robot this lives in `src/main.cpp`"; "piece for piece, the structure of `src/main.cpp`"; "`src/main.cpp` is the worked example"; "one program in `src/` is built this way" | the builder's graph; `src/teleop_program.cpp`; two programs, the X-drive's ports invented |
| `docs/guide/13-extending-the-library.md` :38 | "`src/main.cpp` … is the other deliberate PROS toucher" | `src/` |
| `docs/hardware-assumptions.md` HA-111 row + entry, HA-112 source, HA-128 source, "Port map, last season" row | "main.cpp still carries the invented map"; `src/main.cpp`'s X-drive loop calls the raw mapping; the state screens in `src/main.cpp`; "G1's RobotBuilder replace[s] hand-kept port lists" | `src/xdrive_config.hpp`, refusing at boot (shim-proven only); the X-drive runs the feel layer; `src/state_screens.*`; the generator named, the typed table still typed by hand |
| `docs/roadmap.md` M2 rewire bullet, session-header note, M5 config bullet, register row, M5 "You are here", the "Now" block | "main.cpp wires them"; "the invented X-drive graph in src/main.cpp still builds its own"; "OPEN: the main.cpp collapse (Part 2c)"; "Part 2c is next" | past tense / delivered; a "G1 Part 2c LANDED" paragraph; Next = T3 then COMP |
| `docs/internal/chunks/R3a-BENCH-WORKSHEET.md` :3-5, :1199-1201 | "The shipped `src/main.cpp` cannot boot on this robot"; the X-drive "would fault at boot" | the X-drive build refuses at boot naming the first port |
| `docs/internal/HANDOFF-remaining-work.md` §1 open chunks, §4 (`teleop_loop.hpp` "none of it wired into any program yet"; the xdrive line), L1, §7 item 1 | "2c open"; "none of it wired"; "2c waits for his go" | 2c landed; wired into Teleop (2b) and the X-drive (2c), neither run |
| `.github/workflows/ci.yml` :141-146 | the self-test's list of planted faults | + the chain cases |
| `Makefile` :15-28, :58 | the WARNFLAGS rationale (the X-drive helpers' warnings as evidence); "the invented X-drive wiring (…cannot boot on the bench bot)" | rewritten (§2c.4); refuses at boot on any robot |

**Read and deliberately NOT changed:** dated records that are true of their day (`roadmap.md` :160, :175, :198, :394,
:428, :737, :758, :886, :1050, :1376 and the "Status at C7" paragraph; `changelog.md` :459, :602); history in comments
("until G1 Part 2b", "the retired", `teleop_loop.hpp` :7-8/:23, `stick_mapping_test.cpp`, `teleop_loop_test.cpp` :401,
:513, `motor_group.hpp` :87, `tick_pacer.hpp`, `char_sink.hpp`); the generic concept uses of "wiring", "object graph",
"composition root" (absent_*.hpp, motion.hpp, odo_stall_check.hpp …); the master plan's L4 diagram (:197-198,
"src/main.cpp: initialize / autonomous / opcontrol; Composition root: builds HAL impls -> RobotContext -> Chassis") —
the WHY document's layering is unchanged (the application layer still owns the entry points and the composition, now
through `robot::Robot` from `src/`), and 2a/2b left it for the same reason. **Stale but not false, in files T3 is
editing, NOT touched here (listed for the coordinator):** `src/boot_source.cpp` :13 and `src/teleop_program.cpp` :61
say nothing there reads "src/main.cpp's SHULIB_RUNS_* macros" — those macros no longer exist (true, vacuously);
`include/shulib/teleop/teleop_loop.hpp` :60-63 names only Teleop as a caller (the X-drive is the second).

### 2c.6 The documentation, in this part's (uncommitted) change

| Document | What moved |
|---|---|
| `README.md` | the "Wiring a `Chassis`" paragraph's last sentence (no program wires by hand; `main.cpp` only chooses; the X-drive a `constexpr RobotConfig`); the tree's `src/` line; rule 1's "other deliberate PROS toucher" = `src/` |
| `docs/guide/07-getting-set-up.md` | its `src/` description (the brief's item) |
| `docs/guide/08-your-first-routine.md` | the four sentences 2b left for 2c (2c.5), incl. the verbatim-quoted comment, edited with `test/guide_examples_test.cpp` |
| `docs/guide/13-extending-the-library.md` | the one "other PROS toucher" sentence (same fix as README rule 1) |
| `docs/hardware-assumptions.md` | HA-111's row and entry (the map is `src/xdrive_config.hpp` now; refuses at boot, shim-proven only); HA-112's source; HA-128's source; the "Port map, last season" row's tail (the brief's item) |
| `docs/roadmap.md` | the "G1 Part 2c LANDED" paragraph and "Now" (Next = T3, then COMP); the M5 config bullet (still `[~]`: the `.vexbot` codegen, G3, is OPEN); the session-header port-map note; the M2 rewire bullet's "main.cpp wires them"; the register row; the M5 "You are here" parenthetical |
| `docs/vexbuilder-integration.md` :64 | the `IRobotConfig` / builder row: Part 2c landed, build-gated; G3 owns the codegen |
| `docs/internal/build-order.md` | the first `**Next:**` (T3, then COMP; DIAG1 and COMP-a in parallel) with a "G1 PART 2c LANDED" sentence after it; the previous first sentence kept as history; G1's entry and STATUS (LANDED; G1 closes with it — its completion record is written at merge); the phase table row; the risks list |
| `docs/internal/HANDOFF-remaining-work.md` | §1 open chunks, §4 (the xdrive line, `teleop_loop.hpp`'s wiring status), L1, §7 item 1 |
| `docs/internal/PROJECT-BRIEFING.md` | §12's G row and §13's box (hand-written); the generated block regenerated LAST (2c.7) |
| `docs/internal/chunks/R3a-BENCH-WORKSHEET.md` | the header note and the "Do not upload an old build" line (the X-drive refuses at boot) |
| `docs/api/` + `mkdocs.yml` | regenerated (`api_doc_tool.py generate`: `robot.md`, `robot_config.md`, `session_info.md`, `pros-device_provider.md`, `stick_mapping.md` moved — the `///` comments of 2c.5) |
| `Makefile`, `tools/src_build_gate.py`, `.github/workflows/ci.yml` | the WARNFLAGS rationale, the gate's header and self-test, the CI comment (§2c item 4) |
| this log | this section |

**Deliberately NOT touched, with why:** `G1-COMPLETED.md` — the coordinator's (the task); `docs/changelog.md` — it
records library API changes and 2c changes no public surface (only `src/`, tools, and comments in five headers);
the master plan — the WHY document, its layering unchanged (2c.5); `RESUMING.md` — its protocol and its gate
description are still true (six builds, `self-test` then `check`); `src/teleop_program.*` and `src/boot_source.*` —
T3's files, and the move forced no change in them (2c.1); `include/shulib/teleop/teleop_loop.hpp` — nothing in it
is false (2c.5; its "Teleop drives through it" is incomplete, not false). *(Corrected at 2c.10 F7: this said "T3's
directory, nothing in it false", but `stick_mapping.hpp` in the same directory WAS edited. The rule actually applied:
a FALSE sentence is fixed in this change even in a file a parallel part edits — comment-only, and listed for the
coordinator — and a stale-but-true one is left and listed.)* **Deferred, as the brief says:** the guide chapter on describing a robot as data waits for
G3, when a robot can come from a FILE — today it would teach typing a `RobotConfig` in C++, the audience the feature
exists to avoid (G2 deferred its routine chapter for the same reason). `xdrive_config.hpp` is readable as the
example meanwhile, and guide 07/08 point at it. **Size (rule 7):** the list runs past five documents, as 2b's did; most
entries are one-sentence status corrections that rule 3 requires in the same change.

### 2c.7 Tests and the mutation campaign (10:05-10:18 EDT)

`test/xdrive_config_test.cpp` (NEW, 3 cases / 106 assertions), including `src/xdrive_config.hpp` by relative path:

| Case | Property |
|---|---|
| (namespace scope) | `kInventedXDrive` is a CONSTANT EXPRESSION: `static_assert`s read its kind, its wheel counts, and a `constexpr` COPY of the whole value — the brief's "prove it with a static_assert" |
| 2c-1 | every field against a TRANSCRIPTION of `main.cpp`'s constants at `ea58fbc` (typed as literals, never read from the header under test): ports AND signs in the F5 order, GREEN, IMU 10, radius 7.0, pods 5/6 at 2.0 in and -3.0/-4.5 in, GPS 9 with a zero lever arm, no mechanisms, the HA-111 label whole in the 32-byte name; `valid()` |
| 2c-2 | built over a `FakeDeviceProvider`: every wheel asked for with its SIGNED port as stored and GREEN, no motor asked for on any other port; four single-motor wheels in the F5 order, no groups, no drive-encoder odometry; `strafeAuthority() == 1.0` (the old banner's check); the GPS corrector installed (R3) and the GPS live; the library's default tuning untouched (field by field against `ChassisConfig{}`); the port map `==` the transcribed `portMapString()` AND the literal `FL1 BL2 BR-3 FR-4 ROTF5 ROTL6 GPS9 IMU10`; `describe()` `==` its full text, ≤ 200 bytes with the program's prefix |
| 2c-3 | through the PROS shim and the REAL `ProsDeviceProvider`: robot two's census (radio 1, IMU 2, camera 3, motors 11-18/20) ⇒ `validate()` refuses with exactly `an X-drive wheel motor cannot be made: port 1: the radio (expected a motor)`, no port 1-4 motor configured, IMU 10 never reset, and the constructor throws the same sentence; the old comp bot's census (HA-111's measurement) ⇒ `port 1: nothing plugged in (expected a motor)`; a brain carrying exactly the invented devices ⇒ builds, IMU reset once, a +6 V command lands +6000 mV on ports 1 and 2 and -6000 mV on 3 and 4 (PROS's reversal, once), GREEN on all four, the port map, the corrector |

**Host mutations — every one on a FRESH binary** (`scratchpad/m2c/mutate.py` + `muts.json`: edit → touch the test →
`cmake --build build/test --target shulib_tests/fast` → REQUIRE `Linking CXX executable shulib_tests`, or a compile
failure counted as killed with its error → run `-tc="G1 2c*"` → restore byte-exact → relink → green). 10:09:22-10:10:46
EDT, **12 of 12 killed, every restore relinked green**; X1b/X5b by hand at 10:12:

| # | Mutation (in `src/xdrive_config.hpp`) | Killed by |
|---|---|---|
| X1 | BR's sign dropped (-3 → 3) | COMPILE: the constexpr copy's `static_assert` (:74) |
| X1b | X1 with that `static_assert` clause removed | 2c-1, 2c-2 (signed port), 2c-3 (-6000 mV) — 3 cases |
| X2 | FL and BL swapped | 2c-1, 2c-2, 2c-3 (robot two's refusal now names port 2, the IMU) |
| X3 | cartridge Blue | 2c-1, 2c-2, 2c-3 (the shim's gearing) |
| X4 | GPS declared absent | 2c-1, 2c-2 (map, corrector, describe), 2c-3 |
| X5 | lateral pod offset -4.5 → -3.0 | COMPILE: the constexpr copy's `static_assert` (:76) |
| X5b | X5 with that clause removed | 2c-1 only (no map or banner carries an offset — the transcription is its one guard) |
| X6 | drive radius 7.0 → 7.5 | 2c-1, 2c-2 (describe) |
| X7 | the HA-111 label dropped from the name | 2c-1, 2c-2 (describe) |
| X8 | a typed gain in the constant (`maxLinearSpeed` 50) | 2c-2 (the default-tuning subcase) |
| X9 | `constexpr` → `const` | COMPILE: "non-constant condition for static assertion" (:67) — the constexpr proof itself |
| X10 | the forward pod on port 7 | 2c-1, 2c-2, 2c-3 |
| X11 | the IMU on port 11 | 2c-1, 2c-2, 2c-3 |
| X12 | the GPS lever arm 1.0 forward | 2c-1, 2c-2 (the lever the provider was given) |

**src mutations — through the REAL gate** (`scratchpad/m2c/src_mutate.py`: plant exactly once → the gate's own
`run_build()` + `verdict()` on fresh `bin/`/`.d/`, or the full `check` → restore byte-exact; the full `check` after the
last restore: PASS). 10:12-10:17 EDT:

| # | Mutation | Result |
|---|---|---|
| **S1 — the brief's** | `main.cpp`'s chain: `#elif … defined(SHULIB_PROGRAM_CALIBRATE)` misspelt `_BROKEN` | full `check` **FAIL**, exit 1: `[tank-calibrate] … the beacon 'shulib-robot-variant=tank-calibrate' is NOT in bin/hot.package.elf` and `… carries the tank beacon … a tank build wearing a tank-calibrate label`; the other five builds pass. Restored; `check` PASS at 10:17:22 |
| S2 | `opcontrol()` forgets `Program::Drive` | tank-drive FAIL: `disallowed warning -Wswitch at src/main.cpp:258`. (It also reported the build hash missing from the ELF: with `runDrive` unreferenced, nothing in that package prints the hash — collateral of the plant, and itself a sign the build is not the program it claims) |
| S3 | the X-drive's release mode Brake | xdrive FAIL: `static assertion failed: kDriverLine names … COAST` (:113) |
| S4 | the X-drive's re-zero on L1 | xdrive FAIL: the shoulder-button `static_assert` (:92) and the driver line's (:113) |
| S5 | a refusal literal over the SMALL budget | xdrive FAIL: the screen-literal `static_assert` (:160) |
| S6 | a second selection (`#if defined(SHULIB_ROBOT_XDRIVE_INVENTED)`) in `xdrive_program.cpp` | step 0 reports `src/xdrive_program.cpp:75 selects on a variant/program define — only src/main.cpp's one chain may` |
| S7 — RESIDUAL PROBE | `chassis_table.hpp`'s `#if defined(SHULIB_ROBOT_TANK_2026)` misspelt | tank build **PASSES** (verdict ok): the tank build carries the bench bot's table and nothing sees it — the residual the gate's header names, measured; pre-existing (the retired differential could not see it either) |
| (probe, 2c.1) | the OLD `main.cpp`'s PROGRAM lattice misspelt for xdrive | gate verdict PASS — the hole the one-chain rule closes |

The self-test gained the permanent versions: case 16 (S1's class, on the tank-library branch), 17 (the xdrive
branch), 18 (an unhandled enumerator ⇒ `-Wswitch`), 19 (step 0 on the real tree, a second chain in `main.cpp`, a
selection in another file) — results in 2c.8.

**The gate's own detectors, mutated** (`scratchpad/m2c/gate_mutate.sh`: the gate edited, its full `self-test` run,
restored and `cmp`-checked; 10:25-10:34 EDT): **GM1** — step 0 neutered (`one_chain_problems()` returns `[]`) ⇒ the
self-test FAILS case 19 twice ("a second chain in main.cpp was not reported", "a selection in another src/ file was not
reported"); **GM2** — the expected-beacon-absent check neutered ⇒ the self-test FAILS cases 8, 10, 12, 14 and **16**
(the new chain case). Cases 7 and 17 still pass under GM2 on the OTHER half of the beacon check (the wrong build's beacon
present), as designed.

### 2c.8 Every gate, on the final tree (10:18-10:38 EDT)

| Gate | Command | Result |
|---|---|---|
| Host build, relink required | `touch test/*.cpp && cmake --build build/test --target shulib_tests/fast -j6` | exit 0, `[100%] Linking CXX executable shulib_tests` |
| Host suite | `./build/test/shulib_tests` | **1467 test cases / 2,387,639 assertions, 3 skipped — SUCCESS** = the baseline 1464 / 2,387,533 + the three 2c cases / 106 assertions exactly (no pre-existing case gained or lost an assertion; the four other test files changed only in comments) |
| src build gate self-test | `python3 tools/src_build_gate.py self-test` | `src build gate self-test: OK (45 detector cases, each against a real build)` (33 before: + cases 16-19) |
| src build gate | `python3 tools/src_build_gate.py check` | `src build gate: PASS (all 6 builds link; policy: errors fail, vendor warnings path-ignored, -Wunused-function allowed by category, all else fails; defines token-exact; one chain; beacons asserted)` — 8/8 TUs each, 0 `-Wunused-function` in every build (2/0/2/2/2/2 before), 0 disallowed |
| ARM header gate | the `arm-none-eabi-g++ … -Werror` compile of every `include/shulib` header | exit 0 (184 headers) |
| CI greps | `<pros/>` outside `hal/pros`; `shulib/sim/` outside `sim/` | both print nothing |
| API docs | `api_doc_tool.py generate`, then `check-fresh` | exit 0 |
| Removability | `api_doc_tool.py check-removability` | `removability: no public doc references docs/internal/` |
| Doc staleness | `doc_staleness_audit.py` | `clean (26 live docs, 0 numeric claims checked)` |
| Briefing — LAST | `briefing_status.py generate` (10:35, after the last relink), then the full `cmake --build build/test -j6` (every doc gate: coverage 2593 entities / 148 headers, 408 verbatim example lines, removability, the briefing, the staleness audit and its self-test) and `briefing_status.py check` | `regenerated` (Next up = T3, then COMP; Suite = 1,467 / 2,387,639); the build exit 0; `generated block current` |

`bin/` and `.d/` removed after the last gate run. Packages: none uploaded and none built for a robot (`pros` never run,
`/dev/ttyACM*` never touched); per landmine 9, a robot's packages come from the COMMITTED tree. Nothing committed,
staged or pushed.

### 2c.9 DoD — brief §5 "2c", with evidence

- [x] **§2c item 1** — the state screens are `src/state_screens.{hpp,cpp}`; `main.cpp` has no screen code (the item-5
      grep, 2c.3). Output unchanged (2c.4).
- [x] **§2c item 2** — `kInventedXDrive` in `src/xdrive_config.hpp`, `inline constexpr` (proved: 2c.7's
      `static_assert`s; X9 red), HA-111 in `identity.name`, built through `ProsDeviceProvider` + `Robot`
      (`src/xdrive_program.cpp`); it cannot boot on a real robot and says so (screen + serial; the sentence on robot two
      pinned by 2c-3). Driver control on `runDriverControl`, release Coast; `runTeleopLoop` REMOVED.
- [x] **§2c item 3** — `main.cpp` is the dispatcher (2c.3).
- [x] **§2c item 4** — the differential retired, the header's section rewritten ("THE VARIANT DIFFERENTIAL, RETIRED …"),
      the self-test updated (cases 3/7/10/12/14 reworded, 16-19 added), the `Makefile` rationale rewritten
      (`-Wunused-function` still allowed by category), the CI comment updated; **the mutation: S1, one `#ifdef`
      broken ⇒ `check` FAILS on the beacon; restored; PASS** (2c.7). Beyond the brief: step 0 (ruling R1), because the
      brief's premise did not hold for the two-chain tree (2c.1, measured).
- [x] **§2c item 5** — 275 lines; the grep prints nothing.
- [x] **Test 7's 2c part** — six builds green, beacons unchanged, `main.cpp` ≤ 300 with the grep, the `#ifdef`
      mutation.
- [x] Line counts before/after (2c.3); §2c's documentation (2c.6) — with the deferral the brief names; the prose
      review, run before it was written (2c.5); this log.
- [ ] **`G1-COMPLETED.md`** — the coordinator's (the task); not written here.

### What is NOT done, or needs the coordinator / the team lead

- **NO HARDWARE CLAIM.** No X-drive robot exists; this X-drive program has never booted (the one X-drive boot on record
  is the old hand-wired graph over FAKE devices, 2026-08-12 — corrected at 2c.10 F5); it has run only as source and through the PROS
  shim (2c-3). The four programs that run on robot two say exactly what they said (2c.4). Nothing here ran on a robot.
- **For the team lead (offered, not decided here):** (1) `-Wunused-function` is still ALLOWED by category while no build
  emits one — tightening it to a FAIL is one line (`ALLOWED_CATEGORIES`) and would close the gate's named "new dead
  function passes" hole; the brief ruled "stays allowed", so it stays. (2) The residual in the gate's header —
  `src/chassis_table.hpp`'s own `#if` (tank vs bench table) is invisible to every detector (measured, S7). Closing it
  (e.g. the table's robot name asserted in the ELF per build, or the table selected from main.cpp's chain) is outside
  §2c. (3) *(added at 2c.10, finding 7)* **The merge of T3 and this part needs a decision:** T3's
  `src/mechanism_table.hpp` is to select robot two's claws on its OWN `#if defined(SHULIB_ROBOT_TANK_2026)` (T3's brief
  §2 item 4), so on the merged tree step 0 reports it and `check` FAILS until one of: (a) allowlist it beside
  `chassis_table.hpp` as a second named residual — a broken `#if` there would give robot two's Teleop no claws with
  every check green; (b) close BOTH residuals: step 8 also asserts each table's identity per build (the table's own
  literal or symbol in the tank builds and not in the others), after which allowlisting the table files is safe; (c)
  select both tables from `main.cpp`'s chain. (a) is one line; (b) is the gate change that makes (a) safe.
- **For the coordinator at merge** *(rewritten at 2c.10 — findings 1, 7 and 8; the list it replaces named only (2)'s
  first two files)*: (1) `build-order.md`'s first `**Next:**`, `roadmap.md`'s "Now", HANDOFF L1/§1/§7 and
  `PROJECT-BRIEFING.md` §12/§13 are also edited by the parallel parts — expect conflicts there, and regenerate the
  briefing and `docs/api/` after merging. (2) EVERY file this part changed that a parallel part's brief also changes
  (the briefs read, 2c.10 F7): `include/shulib/robot/robot.hpp` — T3 (mechanisms) and COMP-a (`BuildOptions::pacer`);
  four comment-only hunks, :84-86, :337, :575-576, :863; `include/shulib/config/robot_config.hpp` — T3
  (`MechanismConfig`); one comment hunk, :73-74; `include/shulib/teleop/stick_mapping.hpp` — T3's directory (its briefs
  name `mechanism_binding.hpp` and `teleop_loop.hpp`, not this file); one doc-comment hunk, :133-138;
  `include/shulib/hal/pros/device_provider.hpp` — T3's pneumatic half (a digital-out member); one comment hunk,
  :329-330; `src/calibrate_program.cpp` — T3 item 6 (the claw-port filter); 2c ADDED `calibrateInitialize()` (the
  banner moved from main.cpp) at :1028-1050; `docs/hardware-assumptions.md` — T3 (claw entries), DIAG1 (HA-123) and
  COMP-a (HA-05, a new HA); eight hunks, all in HA-111/HA-112/HA-128 and the "Port map, last season" row;
  `docs/vexbuilder-integration.md` :64 — T3 (the mechanisms schema, other rows); `R3a-BENCH-WORKSHEET.md` :3-6 and
  :1201-1202 — T3 (Station J); `test/robot_builder_test.cpp` :23-24, :732-733, :1377 (comments) — T3 may add mechanism
  cases there (its brief does not name the file); `docs/api/` — regenerate, never hand-merge. (3) **A MERGE THAT
  WILL FAIL THE GATE UNLESS DECIDED:** T3's brief (§2 item 4) selects `src/mechanism_table.hpp` "by the same
  `SHULIB_ROBOT_TANK_2026` `#if`" as `chassis_table.hpp` — step 0's one-chain rule reports any such selection outside
  `main.cpp` except the one allowlisted file, so the merged tree's `check` FAILS on it (listed for the team lead
  below: allowlist it as a second named residual, or close both residuals). (4) The COMP brief §3.3 is stale against
  2c (the review's finding 1): it still has COMP-b add "the `SHULIB_RUNS_*` macros, the beacon chain" and, "if 2c has
  not yet replaced it", the variant differential's list; B3 settles the differential but not the macros. Proposed
  amendment text (a plan change — pace rule 2, so proposed, not made here): *"COMP-b adds the seventh program by: a
  `Program::Competition` enumerator; ONE branch of `main.cpp`'s chain (`#elif defined(SHULIB_ROBOT_TANK_2026) &&
  defined(SHULIB_PROGRAM_COMPETITION)` followed by `constexpr Program kProgram = Program::Competition;` and the beacon
  `shulib-robot-variant=tank-competition`) plus its `#error` checks; a case in EACH of the five entry-point switches
  (`-Wswitch` fails the gate until all five have one — its `disabled()`/`competition_initialize()` cases forward to
  C5's wait and the choice screen, never to `benchStateScreen`); in `tools/src_build_gate.py`, `VARIANTS`,
  `BUILD_ARGS`, `VARIANT_DEFINES`, `VARIANT_BEACONS`, `BUILD_PROGRAM` and `PROGRAM_ENTRY_SYMBOLS` (step 0's program
  table fails until the last two name its functions) and a self-test pair like cases 14/15. There are no
  `SHULIB_RUNS_*` macros and no variant differential since G1 Part 2c."* (5) `G1-COMPLETED.md` and G1's closing
  pointer move — IN THE SAME COMMIT as this part (the DEFECTS2 precedent, `a8540e6`), because this change already
  says "G1 is complete" / "G1 COMPLETE" in `build-order.md` :632, :948, :972, :1486 (twice), `roadmap.md` :348, :1742,
  :1957 and :1998 (table row A), `HANDOFF-remaining-work.md` :303 and `PROJECT-BRIEFING.md` :633 (§12) and :657
  (§13) — located by grep at 2c.10 — and without the record
  those sentences are false (RESUMING rule 5), and the regenerated briefing block says `OPEN CHUNK(S): G1` until it
  exists. (6) Rebuild the six packages from the committed tree and confirm the beacons (§7).
- **Owed follow-ups, listed not done (each would edit a file T3 is changing):** one shared heading-trust helper for
  Teleop and the X-drive (ruling R8); `src/boot_source.cpp` :13 and `src/teleop_program.cpp` :61 still name the
  retired `SHULIB_RUNS_*` macros (vacuously true); `teleop/teleop_loop.hpp` :60-63 names only Teleop as a caller.
- **Found, left as it was (2c changes nothing a robot shows):** "shulib Drive" prints the tester's boot line
  (`[R3A] competition status at boot: … the tester runs only in DRIVER CONTROL`) and paints the tester's AUTONOMOUS
  reason ("…the tester's DRIVE station lives under driver control") — both since R3b Part 0b, both carried over byte for
  byte. A one-line wording fix each, for whoever next edits Drive.
- **The deferred guide chapter** (describing a robot as data) waits for G3, as the brief says.

### 2c.10 Review findings and their disposition (started 11:20:43 EDT)

The review of this part returned eight findings (two major, six minor). Each one below is re-checked against the
worktree first, then fixed at the root inside §2c's scope, or left with the evidence for leaving it. Rows are
appended as the work lands; every gate is re-run on the final tree at the end (2c.11).

| # | Finding (severity) | Re-check | Disposition |
|---|---|---|---|
| F1 | `disabled()` / `competition_initialize()` outside the `-Wswitch` net; COMP would inherit the bench screens; the COMP brief §3.3 stale (major) | **CONFIRMED.** Both were `if (kProgram == Program::InventedXDrive) return;` then `benchStateScreen`; the gate's header claimed every entry point. The COMP brief §3.3 still has COMP-b add "the `SHULIB_RUNS_*` macros, the beacon chain" (B3 settles only the differential) | **FIXED at the root.** Both are `switch (kProgram)` with a case per program and no default, like the other three (`main.cpp`'s header and the gate's say "all five"). **Test:** self-test case 18 now requires each of the five PROS entry points to hold a `switch (kProgram)` AND the planted enumerator to draw a `-Wswitch` at each of those five lines — the claim is now measured, not stated. **Mutation SM1** (`disabled()` back to its `if`): the self-test FAILS case 18 (below). The COMP brief is NOT edited here — a brief change is a plan change (pace rule 2), and COMP-a works from that brief in parallel: the amendment text is the coordinator's item (4) in "What is NOT done" |
| F2 | The beacon vouches for the chain's branch only — a swapped `opcontrol()` case or a wrong enumerator under the right beacon passes (major) | **CONFIRMED.** `verdict()` checked only the hash and the beacons; step 0 only the prefix of the next line. Measured, see "The old gate on the two plants" below: both plants PASS steps 0-7 | **FIXED — step 8, the PROGRAM, from each package's symbol table.** `arm-none-eabi-nm -C --defined-only bin/hot.package.elf`: the build's program (`BUILD_PROGRAM`) must link every `shulib::bench::` name `main.cpp` uses for it (`PROGRAM_ENTRY_SYMBOLS`: its `*Initialize`, its opcontrol entry, its autonomous entry where it has one, Teleop's reason object) and must link NONE of another program's. It holds because a switch on a `constexpr` folds at `-Os`, no call crosses a TU inlined (no LTO — `common.mk`), and `--gc-sections` drops what nothing calls; if any of that changes, step 8 FAILS, it cannot pass silently. Step 0 gained a second source check, THE PROGRAM TABLE: the enum's enumerators = the table's programs, every build has one, every `shulib::bench::` name `main.cpp` uses (comments stripped — a single-pass tokenizer, because `src/*.cpp` in a `//` comment reads as a block-comment start to a naive regex, found while writing it) is one program's or shared, every table name is used — so COMP-b cannot add a program without telling the gate whose functions are whose. *Alternative (the review's):* one banner literal per program in the ELF — rejected: a banner lives in `*Initialize()` only, so mutation A (the `opcontrol()` calls swapped) keeps every banner right and would pass; it would also copy prose from five files into the gate, one of them T3's. *Alternative:* a table of function pointers chosen in the chain's branch — rejected: the pairing is still typed by hand, and `-Wswitch` is lost. **Named limit** (the gate's header): the four sentences `main.cpp` paints itself (DISABLED, FIELD CONTROL, the tester's and Calibrate's AUTONOMOUS) are its own literals, no program's symbol — a swap there shows the wrong sentence and passes. **Tests:** case 20 (mutation A: the Bench Tests build links `runDrive` and misses `runR3a` — both halves reported, NO beacon problem), case 21 (mutation B: `kProgram = Program::BenchTests` under the tank-library beacon — both halves, no beacon problem, steps 0's two checks silent), case 22 (the program table: an unknown enumerator, an unowned name, a name inside a comment NOT counted). **Gate mutations GM4/GM5/GM6** red (below). `nm` is `binutils-arm-none-eabi`, a `Depends` of the `gcc-arm-none-eabi` package CI installs (`apt-cache depends`, this machine); a missing `nm` FAILS the gate and the self-test, never skips |
| F3 | The X-drive's `static_assert`s check its constants, not the loop's configuration (minor) | **CONFIRMED** — `loopCfg` was built inside `runXdrive()` from `kRezeroButton`/`kReleaseMode`; a literal there, or a dropped field (the loop's default re-zero is L1), passed every assert | **FIXED.** `constexpr teleop::DriverLoopConfig kLoopConfig{…}` at namespace scope, passed to `runDriverControl` AS IS; the asserts (tick period > 0, re-zero not a shoulder button, the driver line's words) read ITS fields. **Mutations through the real xdrive build** (fresh `bin/`, restored byte-exact): F3a release Brake → FAIL (driver-line assert); F3b the re-zero field DROPPED (the L1 default) → FAIL (shoulder assert + driver-line assert); F3b2 re-zero L1 → FAIL (both); F3c the tick period dropped → FAIL (the new period assert). **Residual, run and SURVIVING as expected:** F3d — a different config built at the call (`teleop::DriverLoopConfig{… .releaseBrakeMode = Brake}` in place of `kLoopConfig`) → PASS; the asserts can only check the constant the call is written to pass. **(c) and (d) have NO test**, recorded as the review asks: deleting `attachFaultLatch(&r.faults())` or the `isReady()` clause of the heading trust passes everything — `src/*.cpp` is not compiled on the host, and Teleop has the same two lines untested; the owed shared heading-trust helper (ruling R8) is where a host test for (d) would live, once T3 lands |
| F4 | `SELECTS`' `#ifdef`/`#ifndef` forms never exercised (minor) | **CONFIRMED** (every case-19 plant was `#if defined(...)`). **AND WIDER:** C++23's `#elifdef`/`#elifndef` — accepted SILENTLY by this toolchain at `-std=gnu++20` with the project's warnings (measured: `arm-none-eabi-g++` 13.2.1, `#if 0 / #elifndef BAR` took the branch, exit 0, no diagnostic) — were not in `SELECTS` at all | **FIXED.** `SELECTS` covers `if|elif|ifdef|ifndef|elifdef|elifndef`; case 19 plants one of each (`#ifdef` and `#elifndef` in `bench_r3a.cpp`, `#ifndef` and `#elifdef` in `main.cpp`), each expected reported. **GM3** (the pattern narrowed back to `(if|elif)`): the self-test FAILS all four |
| F5 | "No X-drive has ever booted" contradicts the record; the runtime line printed it at the one boot where one had (minor) | **CONFIRMED** against `build-order.md` :782-790 (2026-08-12: the package from `d4fac9c` booted, built its whole object graph, printed `strafeAuthority=1.00`; the HAL fake-backed), `roadmap.md` :1399-1402 and `xdrive_config.hpp` :9 | **FIXED.** Worded to the record everywhere: no X-drive ROBOT exists; the one X-drive boot on record is the old hand-wired graph over FAKE devices, 2026-08-12 (it drove nothing); none is on record since R1a gave it real adapters; THIS version has never booted and cannot on any robot that exists. `kUnvalidatedLine`, printed only when the graph BUILT, now says only what is true at that boot: *"HARDWARE-UNVALIDATED: this X-drive graph is built on REAL adapters, but every port and sign in it is INVENTED (HA-111) -- nothing here was measured; no motion it commands is validated"* (183 bytes, under the 200 `static_assert`). Touched: `src/xdrive_program.{cpp,hpp}`, `src/main.cpp`, `src/xdrive_config.hpp` :32, `include/shulib/robot/robot.hpp` :86 ("so it refuses at boot on every robot that exists"; `docs/api/robot.md` regenerated), `test/xdrive_config_test.cpp` :25, `docs/hardware-assumptions.md` :774, and this log (2c.3 R3, 2c.4's X-drive heading, "What is NOT done" — each marked "corrected at 2c.10 F5") |
| F6 | A refused X-drive boot never printed its beacon (minor) | **CONFIRMED** — the beacon was logged through the provider's sink after `emplace`, on the success path only | **FIXED.** `xdriveInitialize(beacon, bits)` prints its banner FIRST, before the graph boundary, the way the other four programs do: `[R1A] booted: INVENTED X-DRIVE build [shulib-robot-variant=xdrive] -- every port and sign INVENTED` / `[R1A] (HA-111); on a robot without exactly those devices it refuses below.` and `[R1A] competition status at boot: <bits> -- the drive loop runs only in DRIVER CONTROL, enabled; this program paints no state screen.`; `main.cpp` passes `bootStatusBits()` as it does for the others; the later `[R3B]` sink beacon line is gone (the banner carries it). So `competitionBits()` is now linked into the xdrive package (`state_screens.hpp`'s gc sentence corrected). **Not tested:** the ORDER (source only — `src/` is not host-compiled); the beacon's presence in the package is step 6's, as before |
| F7 | The collision list is incomplete; the "T3's directory" reason applied inconsistently (minor) | **CONFIRMED, and wider than the review found** — reading T3's two briefs and COMP-a's and DIAG1's scopes against this part's file list: T3 also changes `src/calibrate_program.cpp` (its item 6; 2c added `calibrateInitialize()`), `hal/pros/device_provider.hpp` (the pneumatic half), the HA register, `vexbuilder-integration.md` and the worksheet; and T3's `src/mechanism_table.hpp` is to be "selected by the same `SHULIB_ROBOT_TANK_2026` `#if`" — which step 0 will REPORT at merge | **FIXED in the log.** The `stick_mapping.hpp` edit is KEPT (it corrected a sentence that 2c made false — rule 3 — comment only) and listed; the complete list, with line ranges, is the coordinator's items (2)-(3); 2c.6's reason for leaving `teleop_loop.hpp` corrected to the rule actually applied (a false sentence is fixed and listed; a stale-but-true one is left and listed) |
| F8 | Small inaccuracies; "G1 complete" before `G1-COMPLETED.md` (minor) | **CONFIRMED, all four** | (1) 2c.3's rows corrected in place (`bench_r3a.hpp` 178; the gate 719 → 848, measured at 2c.3's tree), marked. (2) `vexbuilder-integration.md` :64: "ungated" made explicit in the doc's own sense (:468 — G1 waits on nothing from VexBuilder); "build-gated" is the project's word for "the src build gate passes" and stays. (3) `state_screens.hpp`: three fixed lines (53, 53, 54 characters) and all four of `main.cpp`'s OWN reasons (66, 68, 76, 126); Teleop's own reason (48) fits 50. (4) "G1 is complete" KEPT, as the DEFECTS2 precedent does (`a8540e6` carried `DEFECTS2-COMPLETED.md` in the chunk's own commit): the coordinator's item (5) lists every location found by grep and says plainly that without the record in the SAME commit those sentences are false |

**The old gate on the two plants of finding 2, measured** (`scratchpad/f/oldgate.py`, 12:04 EDT: each plant ONE fresh
build, judged twice from the same build — steps 0-7 alone, then the full gate — and restored byte-exact):

| Plant | Build | Step 0 (both checks) | Steps 0-7 (the gate before this review) | Full gate |
|---|---|---|---|---|
| A — `opcontrol()`'s BenchTests and Drive calls swapped | bench | 0 problems | **PASS** | FAIL: `runR3a` not linked; the Drive program's `runDrive` linked |
| B — the tank-library branch assigns `Program::BenchTests` | tank-library | 0 problems | **PASS** | FAIL: Teleop's four names not linked; the tester's `testerInitialize` and `runR3a` linked |

**What each package links, re-measured on the final src** (`arm-none-eabi-nm -C --defined-only`, 12:06 EDT; the
`shulib::bench::` names only): bench and tank — `testerInitialize`, `runR3a`, the state screens, the chassis-table
helpers; tank-drive — `driveInitialize`, `runDrive`, the state screens; tank-library — `teleopInitialize`,
`teleopAutonomous`, `runTeleop`, `kTeleopAutonomousReason`, the boot source, the state screens; tank-calibrate —
`calibrateInitialize`, `runCalibrate`, the state screens; xdrive — `xdriveInitialize`, `xdriveAutonomous`,
`runXdrive`, `kInventedXDrive`, `competitionBits` (NEW since F6: `main.cpp` formats its boot bits), and NOT
`benchStateScreen`. No package links a name of another program — which is what step 8 now asserts on every run.

**Mutations of this round — every one observed red on a fresh build, restored byte-exact:**

| # | Mutation | Result |
|---|---|---|
| F3a / F3b / F3b2 / F3c | `kLoopConfig`: release Brake / the re-zero field dropped / re-zero L1 / the tick period dropped | the xdrive build FAILS on the `static_assert`s (11:41 EDT, `scratchpad/f/srcmut.py`) |
| F3d — RESIDUAL | a different config built at the `runDriverControl` call | PASSES, as named (the asserts check the constant, not the call) |
| GM3 | `SELECTS` narrowed to `(if|elif)` | self-test FAILS — case 19, all four directive-form plants unreported (11:42-11:46) |
| GM4 | step 8's "another program's name linked" half disabled | self-test FAILS — cases 20 and 21 (11:46-11:50) |
| GM5 | step 8's "own name missing" half disabled | self-test FAILS — cases 20 and 21 (11:50-11:54) |
| GM6 | step 0's program-table check returns nothing | self-test FAILS — case 22, both plants (11:54-11:59) |
| SM1 | `disabled()` back to its `if` (finding 1's shape) | self-test FAILS — case 18, "disabled() in src/main.cpp holds no `switch (kProgram)`" (11:59-12:03) |

(`scratchpad/f/gatemut.py`: each a FULL `self-test` run as a subprocess against the mutated file; every restore
checked byte-exact. The real self-test passes on the restored tree — 2c.11.) Each half of step 8 is tested ON ITS
OWN: GM4 leaves the other half firing and the self-test still fails, because cases 20 and 21 expect BOTH messages.

**This round's documentation** (the prose review: the greps run BEFORE this row was written — `ever booted|never
booted|has booted|booted either way` over `src`, `include`, `test`, `docs`, `README.md`; `made complete|G1
(is )?complete|G1 COMPLETE|G1 CLOSES` and `beacon` over the living documents; `xdriveInitialize\(beacon\)|kReleaseMode|
as an .if.|beacons checked|45 detector|self-test cases|step 8|symbol table` over the living documents, `src`,
`include`, `tools`, `test`, `Makefile`, `.github`; every hit read): `src/main.cpp` (the two switches, the header, the
chain's comment, the run status); `src/xdrive_program.{cpp,hpp}`; `src/xdrive_config.hpp` :32; `src/state_screens.hpp`;
`include/shulib/robot/robot.hpp` :86 → `docs/api/robot.md` regenerated; `test/xdrive_config_test.cpp` :25 (comment);
`tools/src_build_gate.py` (header steps 0 and 8, the tables, cases 18-22); `Makefile` (the rationale's last
sentence); `.github/workflows/ci.yml` (the self-test comment); `docs/hardware-assumptions.md` :774;
`docs/vexbuilder-integration.md` :64; `docs/roadmap.md` :355-358; `docs/internal/build-order.md` :632 and :1486;
`HANDOFF-remaining-work.md` :302-303; this log. **Left, and why:** 2c.3's and 2c.8's rows describe the tree BEFORE this
review (`disabled()` "as an `if`", "45 detector cases", "beacons checked") — they are true of their timestamps, and 2c.11
supersedes them; the COMP brief — a plan document, proposed not edited (coordinator item 4).

### 2c.11 Every gate, re-run on the final tree after the review (12:06-12:25 EDT)

| Gate | Command | Result |
|---|---|---|
| Host build, relink required | `touch test/*.cpp && cmake --build build/test -j6` (every doc gate inside) | exit 0, `[100%] Linking CXX executable shulib_tests` |
| Host suite | `./build/test/shulib_tests` | **1467 test cases / 2,387,639 assertions, 3 skipped — SUCCESS** — unchanged from 2c.8 (this round changed no host-compiled code; its one test-file edit is a comment) |
| src build gate self-test | `python3 tools/src_build_gate.py self-test` | `src build gate self-test: OK (73 detector cases, each against a real build)` (45 at 2c.8: + case 18's five per-entry-point checks, case 19's four directive forms, cases 20-22) |
| src build gate | `python3 tools/src_build_gate.py check` | `src build gate: PASS (all 6 builds link; policy: errors fail, vendor warnings path-ignored, -Wunused-function allowed by category, all else fails; defines token-exact; one chain; beacons asserted; programs asserted)` — `[program-table] 5 programs, 13 entry names, matching src/main.cpp`; 8/8 TUs each, 0 `-Wunused-function`, 0 disallowed |
| ARM header gate | CI's `arm-none-eabi-g++ … -Werror` over every `include/shulib` header | `TU includes 184 headers`, `ARM compile gate: CLEAN (all v2 headers)` |
| CI greps | `<pros/>` outside `hal/pros`; `shulib/sim/` outside `sim/` | both print nothing |
| API docs | `api_doc_tool.py generate`, then `check-fresh` | exit 0 |
| Removability | `api_doc_tool.py check-removability` | `removability: no public doc references docs/internal/` |
| Doc staleness | `doc_staleness_audit.py` | `clean (26 live docs, 0 numeric claims checked)` |
| Briefing — LAST | `briefing_status.py generate`, then the full host build again (every doc gate) and `briefing_status.py check` | `briefing status: regenerated`; the build exit 0 (`doc coverage: 2593 public entities across 148 headers, all documented`, `408 quoted lines, all verbatim`, the relink line); `briefing status: generated block current` (13:10 EDT, re-run after this row was written: current) |

`src/main.cpp` = **292 lines** (≤ 300); the §2c item-5 grep prints nothing. `bin/` and `.d/` removed after the last gate
run. Nothing committed, staged or pushed; `pros` never run; `/dev/ttyACM*` never touched; no hardware claim.

## Part 2c final: the team lead's rulings — 2026-09-26 13:38:40 EDT

> **Part:** G1 Part 2c's FINAL run — the team lead's two rulings of 2026-09-26 (~13:15), applied on top of
> Part 2c as built and reviewed (2c.0-2c.11 above). **Tree at start:** `80e2615` (branch `g1-2c-final`, an
> isolated worktree at `.claude/worktrees/g1-2c-final`: G1 Part 2c squashed onto `cc513a7`, which carries T3
> Part 1 — `src/mechanism_table.hpp` landed at `c845041`). **Executor model:** opus. **The rulings, as the
> coordinator relayed them:** RULING 1, "ONE CHAIN PICKS BOTH" — the build switches are read in ONE small
> shared header in `src/` whose every branch sets the program, its beacon AND the robot; `src/main.cpp`
> includes it and keeps its dispatch; the two tables choose the robot's table from that constant with no `#if`
> of their own; the gate's step 0 enforces it; a wrong robot inside a branch fails the gate through a beacon
> that names the robot. RULING 2, "`-Wunused-function` becomes a FAILURE". Appended in real time; every
> timestamp from a `date` call. Nothing committed, staged or pushed; `pros` never run; `/dev/ttyACM*` never
> touched; the main tree and every other worktree (T3 Part 2 builds in `.claude/worktrees/t3-part2`) never
> touched. **NOT touched by this run (other runs own them):** `src/teleop_program.*`, `src/boot_source.*`,
> `include/shulib/teleop/*`, `build-order.md`'s and `roadmap.md`'s status paragraphs, HANDOFF, the COMP brief,
> `G1-COMPLETED.md`. No hardware claim: nothing here has run on a robot.

### 2cF.0 Log created (13:38:40 EDT)

Log section created before any code. Next: the baseline on the clean tree at `80e2615`, then the design.

### 2cF.1 Baseline on the clean tree at `80e2615` (13:39-13:44 EDT)

Read first: the brief's §2c; this log's Part 2c (2c.0-2c.11, the coordinator's and the team lead's lists);
`src/main.cpp` (292 lines), `src/chassis_table.hpp`, `src/mechanism_table.hpp` (T3's, `c845041`),
`tools/src_build_gate.py` (1,126 lines), `test/chassis_table_test.cpp`, `test/mechanism_table_test.cpp`, and
the other tests that include either table (`boot_source_test`, `boot_check_test`, `drivetrain_description_test`,
`bench_menu_grid_test`, `pros_device_registry_adapter_test`, `xdrive_config_test` — by grep).

| Gate | Result |
|---|---|
| Host build (`cmake -S test -B build/test`, `cmake --build build/test -j8`) | exit 0, `[100%] Linking CXX executable shulib_tests`; every doc gate green (coverage 2669 entities / 149 headers, 437 verbatim lines) |
| Host suite | **1536 test cases / 2,393,994 assertions, 3 skipped — SUCCESS** (2c's 1467 + T3 Part 1's cases) |
| `src_build_gate.py check` | **FAIL, exit 1 — at step 0 only**, exactly the merge failure 2c predicted (2c.9 coordinator item 3): `[one-chain] src/mechanism_table.hpp:298 selects on a variant/program define — only src/main.cpp's one chain may` and the same for `:571`. All six builds link, 8/8 TUs each, **0 `-Wunused-function` in every build**, 0 disallowed, beacons and programs checked (1 m 19 s) |

So the tree this run starts from does not pass its own gate; Ruling 1 is what makes it pass, by removing both
tables' `#if`s rather than allowlisting them.

### 2cF.2 The design (13:44-14:05 EDT) — Ruling 1, "one chain picks both"

**`src/build_selection.hpp` (NEW, 156 lines) — the ONE place the build switches are read.** It holds the five
impossible-combination `#error` guards (moved verbatim from `main.cpp`), `enum class Program` (moved from
`main.cpp`'s anonymous namespace into `shulib::bench`), a NEW `enum class RobotId { BenchBot, RobotTwo,
InventedXDrive }`, and THE ONE CHAIN (`:107-131`): six branches, each setting exactly
`inline constexpr Program kProgram`, `inline constexpr RobotId kRobot` and `inline constexpr char
kVariantBeacon[]` — nothing else. Below the chain, `robotBeacon(RobotId)` (`:140-150`, a case per robot, no
default) DERIVES the robot beacon — `shulib-robot=bench-bot|robot-two|invented-xdrive` — and
`kRobotBeacon = robotBeacon(kRobot)`. The robot beacon is never typed in a branch: that is what lets it vouch for
the `kRobot` a branch set, which the branch's own typed `kVariantBeacon` cannot.

**`src/main.cpp` (292 → 252 lines)** includes it and keeps its dispatch: the guards, the enum and the chain are
gone (`grep -n "SHULIB_" src/main.cpp` prints nothing); `using shulib::bench::{kProgram, kRobotBeacon,
kVariantBeacon, Program}`; `initialize()` prints ONE new serial line before the program's boot —
`[BLD] robot tables: <robot beacon> (src/chassis_table.hpp, src/mechanism_table.hpp)` — which is what links the
robot beacon into the package (and tells a person reading the log which robot's tables the build carries). The
five `switch (kProgram)` are unchanged.

**The tables choose from `kRobot`, with no `#if` of their own.** `src/chassis_table.hpp`: every robot's table is
defined in every build under its own name — `kRobotTwoChassis` (robot two's, byte-identical to the old tank
branch), `kBenchBotChassis` (byte-identical to the old `#else`), and a NEW `kNoChassis` for the invented X-drive
(ruling F1 below); `chassisTableFor(RobotId)` (a case per robot, no default) is "the one place a robot becomes a
table"; `inline constexpr const ChassisTable& kChassis = chassisTableFor(kRobot);` and a restated
`static_assert(&kChassis == &chassisTableFor(kRobot))`. `src/mechanism_table.hpp`: T3's pattern was already
per-robot names; its `#if`/`#else` selector became `mechanismTableFor(RobotId)` + `kMechanisms =
mechanismTableFor(kRobot)`; its two `#if`'d static_asserts ("THE SELECTION, restated") became the same two checks
keyed on `kRobot` (`kRobot != RobotId::RobotTwo || (…robot two's, 2 pivots, 1 claw…)` and `kRobot ==
RobotId::RobotTwo || &kMechanisms == &kNoMechanisms`). **Every existing static_assert on the tables is kept**
(the ~~nine~~ EIGHT robot-two checks — 1-7 and 9 [corrected at 2cR.5: check 8 was build-pair only before this run] —
the build-pair checks, check 8, the restated selection); ONE is added: check 8 on robot
two's REAL pair (`mechanismsDisjointFromChassis(kRobotTwoMechanisms, kRobotTwoChassis)`) in every build — possible
only now that robot two's chassis table exists on the host. A misspelt robot in either table is a compile error
(`RobotId::RobotTwoo` names nothing); a robot added to `RobotId` and not to a table is `-Wswitch`.

**2c's known gap is closed by the same change.** 2c.1/2c.7 S7 named and measured it: `chassis_table.hpp`'s own
`#if defined(SHULIB_ROBOT_TANK_2026)` misspelt ⇒ a tank build carried the bench bot's table and PASSED every check.
That `#if` is gone, as are T3's two in `mechanism_table.hpp` (the merge failure of 2cF.1); the gate's step 0 has no
allowlist any more; and a wrong robot is now visible twice — from the source (step 0's chain table) and from the
package (the robot beacon).

**How the host compiles the tables without a robot define — the design, and the alternative rejected.**
*Chosen:* per-robot named constants defined in EVERY build (`kBenchBotChassis`, `kRobotTwoChassis`, `kNoChassis`,
`kRobotTwoMechanisms`, `kNoMechanisms`) + constexpr selection functions over the enum (`chassisTableFor`,
`mechanismTableFor`) + the build's selection (`kChassis`, `kMechanisms`) = selection(`kRobot`). The host includes
the tables — and through them `build_selection.hpp` — with no switch set, so the chain takes its `#else`: the host's
`kProgram`/`kRobot`/`kChassis`/`kMechanisms` are exactly the bench build's (`make` with no ROBOT). The host suite
does not lean on that: it reads every robot's table BY NAME and calls both selection functions for EVERY `RobotId`,
so the tables robot two's four builds select are tested byte for byte on the host — which closes a second, older gap
too: `test/chassis_table_test.cpp` could only RE-TYPE robot two's ports (a typo in the real table was invisible to
the suite, its own header said so). *Rejected (A):* a third header (`build_tables.hpp`) holding the selection, so
the tables stay selection-free and the host never sees `kChassis`/`kMechanisms` at all — rejected because the ruling
has the TABLES choose from the constant, the tables' own "restated selection" static_asserts would have to move away
from the tables they check, and every program's include would change for no gain the host tests need. *Rejected
(B):* defining a robot switch in one host test TU — the ODR violation `chassis_table_test.cpp` and T3's header both
describe (two definitions of `kChassis`, now also of `kProgram`/`kRobot`, across the suite). *Rejected (C):* a
`RobotId::Host` enumerator — a robot that is no build.

**Rulings made while building (each with the alternative):**

- **F1 — the X-drive build carries an UNSET chassis table (`kNoChassis`), not the bench bot's.** The enum forces a
  choice for `RobotId::InventedXDrive`. *Alternative:* map it to `kBenchBotChassis` — byte-identical to the old
  `#else`, but that is a build carrying ANOTHER robot's facts (HA-111's defect class, dormant only because no program
  in that build reads `kChassis`; step 8 proves none is linked). With every field UNSET, anything that ever read it
  would refuse by name. Measured: no runtime string of the xdrive package changes except the `[BLD]` line and its
  robot beacon (below). **One consequence for a file this run may not touch:** `src/boot_source.cpp` :14-16 says "In
  the bench and xdrive builds src/chassis_table.hpp's kChassis is the bench bot's table" — now FALSE for xdrive
  (listed for the coordinator, 2cF.8).
- **F2 — the robot beacon is DERIVED (`robotBeacon(kRobot)`) and printed by `main.cpp`, not typed in the branch and
  not folded into the variant beacon.** *Alternative (a):* a second typed literal per branch — rejected: typed next
  to `kRobot`, it could disagree with it exactly as the variant beacon can, and would vouch for nothing new.
  *Alternative (b):* compose one beacon from both enums (`…=tank-library` from `RobotTwo` + `Teleop`) — rejected: it
  changes the text every program's banner prints (and "shulib Teleop"'s banner is T3's file), and the branch would no
  longer "set its beacon". *Alternative (c):* keep it in the ELF without printing (`[[gnu::used, gnu::retain]]`) —
  rejected: a person reading the serial log could not see which robot's tables the build carries, and `retain` is a
  toolchain dependency the gate would have to trust. The cost: ONE new serial line at boot in all six builds, before
  each program's banner — the only runtime change in this run.
- **F3 — the robot beacon is one string per ROBOT, not per build** (the four tank builds share robot two's): it
  answers "whose tables", and the variant beacon already answers "which build".
- **F4 — `Program` and `RobotId` live in `shulib::bench`, the chain's constants are `inline constexpr`** (one
  definition across TUs; `main.cpp` brings the four names in with `using`). The enum is named `RobotId`, not
  `Robot`, beside `robot::Robot` (the library's builder).

Measured, the six packages before (`80e2615`) and after (`strings -a` and `nm -C --defined-only`, every build
from a fresh `bin/`): the ONLY new runtime strings are the `[BLD]` format and the build's own robot beacon; the old
`kChassis` table bytes are unchanged in the bench and tank packages (robot two's / the bench bot's table text
present exactly as before); every other difference is a symbol name in the debug info (`kChassis` →
`kRobotTwoChassis`/`kBenchBotChassis`, the anonymous `kVariantBeacon` → `shulib::bench::kVariantBeacon`). Each
package carries its own robot beacon and neither of the others'.

### 2cF.3 The gate, Ruling 2, and the first full runs (14:00-14:20 EDT)

**`tools/src_build_gate.py` (1,126 → 1,542 lines).** Step 0 is now THREE source checks. *THE ONE-CHAIN RULE*
(rewritten): a build switch named anywhere in the code of any `src/` file other than `SELECTION_HEADER`
(comments AND string literals stripped — so `#if`, `#ifdef`, `#ifndef`, `#elif`, `#elifdef`, `#elifndef`, a
laundering `#define` and plain code are all caught, and a switch named in a comment or a printed hint is not), or a
declaration of `kProgram`/`kRobot`/`kVariantBeacon` there, FAILS; `SELECTION_ALLOWLIST` is gone (no file is
exempt); in the header, every block reading a switch is an `#error` guard or THE ONE CHAIN — exactly one — whose
every branch, `#else` included, holds exactly the three constants once each and nothing else. *THE CHAIN'S TABLE*
(new): each branch is one build's row of the gate's tables (`VARIANT_BEACONS`, `BUILD_PROGRAM`, the new
`BUILD_ROBOT`), every build has exactly one branch, `RobotId`'s enumerators and `robotBeacon()`'s cases equal the new
`ROBOT_BEACONS`. *THE PROGRAM TABLE*: reads the `Program` enum from the header now; `BUILD_SELECTION_NAMES` lets
`main.cpp` name the four selection names it `using`s. **Step 6 gained its robot half**: the build's robot beacon
present, the other robots' absent, NUL-matched like the variant beacons.

**RULING 2 — `-Wunused-function` is a FAILURE.** `ALLOWED_CATEGORIES = frozenset()`; the header's policy table and
its "-Wunused-function IS A FAILURE" paragraph (replacing "AFTER G1 PART 2c" and "THE RESIDUAL HOLE" — the hole it
named, "a genuinely NEW dead function in src/ passes", is closed); the summary and PASS lines no longer print an
allowed category; self-test case 3 INVERTED (a planted dead function must FAIL, naming `-Wunused-function` and
`bench_r3a.cpp`, with nothing allowed); the `Makefile`'s WARNFLAGS rationale and the CI comment say so.
**DEVIATION FROM THE BRIEF, recorded as the ruling requires:** the G1 Part 2 brief §2c item 4 says "`-Wunused-function`
stays allowed by category" (and 2c followed it, offering the tightening in 2c.9's list for the team lead, item
(1)); the team lead's ruling of 2026-09-26 (~13:15) supersedes it. Nothing emits one: 0 in all six builds at
2cF.1's baseline and at every run since.

**The self-test (cases 16-25; 73 → 114 detector checks):** 3 inverted; 16/17 moved to the header's chain (17 now also
requires BOTH robot-beacon messages: the xdrive build fallen through to the bench bot's tables; 16 requires the robot
beacon NOT to fire — both of its branches are robot two's — so each detector is proven on its own); 18 plants the
unhandled `Program` in the header and requires `-Wswitch` at each of `main.cpp`'s five switch lines (now unmoved); 19
rewritten — the six directive-form plants in `main.cpp`/`bench_r3a.cpp` plus THREE forms (`#if defined`, `#ifdef`,
`#if 0`/`#elifdef`) in EACH TABLE, a laundering `#define` and a plain-code read in each table, a shadowing `kRobot`
in each table, a switch named in a comment and in a string (NOT reported), a second chain in the header in two
forms, a stray `#elifndef` in the header, a chain constant declared outside the chain, and a branch with no robot
and an extra line; 21 now also requires step 0's chain table to report the wrong program; NEW 23 (an unhandled
`RobotId` ⇒ `-Wswitch` in `build_selection.hpp`, `chassis_table.hpp` AND `mechanism_table.hpp`, and the chain table
reports it); NEW 24 (THE RULING'S CASE: the tank-library branch sets `RobotId::BenchBot` ⇒ the chain table reports it
from the source, and from the package ONLY the robot beacon fires — robot two's absent, the bench bot's present, no
variant-beacon message, no step-8 message); NEW 25 (the robot SPLIT into a chain of its own: step 0 reports the second
chain and the branches with no robot even though the split picks the right robots; and with the tank define misspelt
in the robot chain, the tank build carries the bench bot's robot beacon under the right variant beacon — only the
robot beacon sees it).

| Run | Command | Result |
|---|---|---|
| gate check (13:57-14:00) | `python3 tools/src_build_gate.py check` | `[one-chain] the build switches are read in src/build_selection.hpp only, by ONE chain setting program, robot and beacon in every branch; no allowlisted file` / `[chain-table] 6 builds, 3 robots, matching` / `[program-table] 5 programs, 13 entry names, matching`; six builds 8/8 TUs, 0 disallowed; **PASS** |
| gate self-test (14:05:40-14:11:27) | `python3 tools/src_build_gate.py self-test` | `src build gate self-test: OK (114 detector cases, each against a real build)` (5 m 47 s) |
| host build (14:13) | `touch test/*.cpp && cmake --build build/test -j8` | first attempt STOPPED at the briefing gate (`PROJECT-BRIEFING IS STALE` — the committed block said 1,533 cases, the baseline binary 1,536: stale at `80e2615`, not by this run); `briefing_status.py generate`, rebuild: exit 0, `[100%] Linking CXX executable shulib_tests` |
| host suite | `./build/test/shulib_tests` | **1543 cases / 2,394,206 assertions, 3 skipped — SUCCESS** = baseline 1536 / 2,393,994 + 7 new cases (211 assertions: `build selection` ×3 = 24, `chassis table: the selection` 12, `…robot two's REAL table…` 106, `…the invented X-drive's table…` 56, `mechanism table: the selection` 13) + 1 (the `REQUIRE(&kChassis == &kBenchBotChassis)` added to the bench-bot case) — every assertion accounted for |

**Tests added or changed (host):** `test/build_selection_test.cpp` (NEW): the host's `#else` pinned whole at compile
time and at run time (program, robot, both beacons; `kRobotBeacon` is `robotBeacon(kRobot)`), `robotBeacon()`'s three
texts pinned as literals, and no robot beacon a SUFFIX of another's (the NUL-matched search's real hazard — equality
included) or spelled like a variant beacon. `test/chassis_table_test.cpp`: the selection for every `RobotId`; robot
two's REAL table pinned to the brief's literals field by field (ports, signs, zero tails, cartridge, IMU, flags,
geometry), plus the two strings `boot_source_test.cpp` re-types (the name, `track width, external ratio
(motor->wheel)`), plus the hand-typed copy the break-it cases use pinned to the real table; the X-drive's UNSET table
(consistent; no port on any side; the drive refusal names `LEFT ports, RIGHT ports, cartridge`; no valid
description). `test/mechanism_table_test.cpp`: the selection for every robot, check 8 for EVERY robot's own pair,
a claw moved onto the real robot two's drive side refused; the disjointness helper now copies the REAL
`kRobotTwoChassis`. Comment-only: `test/boot_source_test.cpp` :161-165 (its reason for re-typing was false after
this change).

### 2cF.4 RESUMED AFTER A POWER LOSS (20:27 EDT) — what the interrupted run left, and what was checked

The run above was cut off by a power loss at about 14:35 EDT; nothing was committed. This entry is written by the
run that resumes it (executor model: opus), before any further change. The previous run's scratch files under
`/tmp` (its mutation scripts, run logs and the backups its mutation runs restored from) were WIPED by the reboot;
this run keeps its scratch files under `.claude/coordinator/scratch/2cfinal/` (reboot-safe) instead.

**The log stops at 2cF.3 (last written 14:15:20 EDT).** The worktree says the run went on after that, unlogged
(`stat` of every changed file, `bin/`, `build/` and `tools/__pycache__/`):

| Time (EDT) | What the tree shows |
|---|---|
| 14:01-14:10 | the four program headers' comments, the `Makefile`, guides 07/08, `roadmap.md`, `README.md`, `ci.yml`, `main.cpp` — the docs pass 2cF.3 mentions only in part (README, the guides and the roadmap row are not in the log) |
| 14:12 / 14:15 | `PROJECT-BRIEFING.md` regenerated (2cF.3's host-build row); this log last written |
| 14:18:00 / 14:18:20 | EVERY `test/*.cpp` touched at the same instant, then `shulib_tests` relinked — the fresh-binary step of a host mutation run |
| 14:22:18 / 14:23:46 | `src/chassis_table.hpp`, then `src/mechanism_table.hpp`, written — a mutation's plant or restore |
| 14:26:28-14:26:41 | ONE package built into `bin/` — a clean `tank` build: the `tank` variant beacon and robot two's robot beacon present, no other build's or robot's (NUL-matched, measured now) |
| 14:27:00.307 / .347 / .380 | `tools/src_build_gate.py` written, the gate IMPORTED as a module (its `.pyc` written — the recorded source mtime and size equal the file on disk now), then `src/build_selection.hpp` written: a script restoring two backups around a source-only (step 0) check |

Nothing was written after 14:27:00, so the last act on record is a RESTORE — but a restore from `/tmp` copies this
run cannot see, so the content was checked on its own merits, file by file, before anything was built:

- `git diff` of every tracked file read in full, and both untracked files (`src/build_selection.hpp`,
  `test/build_selection_test.cpp`) read whole: every change is one 2cF.2/2cF.3 describes. The chain's six branch
  conditions and five `#error` guards are `80e2615`'s `main.cpp` ones, byte for byte; each branch's robot is the
  build's (`xdrive` → `InventedXDrive`, the four tank branches → `RobotTwo`, `#else` → `BenchBot`); `robotBeacon()`'s
  three texts equal the gate's `ROBOT_BEACONS`; `chassisTableFor()` / `mechanismTableFor()` map each robot to its own
  table; `kChassis` / `kMechanisms` are selected from `kRobot`; no table value changed (the diff shows none).
- No `#if`, `#ifdef`, `#elif`, `#else`, `#endif` or `#define` in `src/chassis_table.hpp`, `src/mechanism_table.hpp` or
  `src/main.cpp`; no build switch named in any other `src/` file except in two comments.
- Every static_assert kept: `chassis_table.hpp` 0 → 4 (all new), `mechanism_table.hpp` ~~16 → 17~~ **12 → 13
  declarations** [corrected at 2cR.5: 16 → 17 was `grep -c static_assert`, which also counts four comment lines;
  `grep -cE '^\s*static_assert\('` gives 12 at `80e2615` and 13 now] (T3's 12 — its two `#if`'d "restated
  selection" asserts now two unconditional ones keyed on `kRobot` — plus check 8 on robot two's real pair).
- The gate read whole (1,546 lines — FOUR more than 2cF.3's "1,542"; which four lines is not recoverable, so the gate is
  treated as changed after its last logged self-test and the self-test is re-run on it below): `ALLOWED_CATEGORIES =
  frozenset()`; step 0 has no allowlist; step 6's robot half, the chain table and cases 3, 16-25 as 2cF.3 describes;
  no detector neutered.
- No stray files (`git status --ignored`: only `tools/__pycache__/`); `bin/`, `.d/` are the interrupted run's and
  are deleted by every gate build anyway.

**Verdict: no mutation found left applied** — every change is accounted for by 2cF.2/2cF.3 or by the docs pass the
tree shows. What the lost `/tmp` evidence cannot be replaced by is the previous run's mutation results: none of them
is claimed. The whole mutation set is re-run below on the FINAL code, each on a fresh binary or a fresh package build.
Next: the host build and the whole suite, then the gates, then the docs, then the mutations.

### 2cF.5 After the resume: the suite, the gate, the prose review, and two changes it found (20:27-20:36 EDT)

**The host build and the whole suite on the tree as found** (`cmake -S test -B build/test`;
`briefing_status.py generate`; `touch test/*.cpp && cmake --build build/test -j8`): exit 0, `[100%] Linking CXX
executable shulib_tests`, every doc gate green (coverage 2669 entities / 149 headers, 437 verbatim lines, removability,
the briefing current, the staleness audit clean over 27 live docs). **1543 test cases / 2,394,206 assertions, 3
skipped — SUCCESS: exactly 2cF.3's numbers**, so every host-compiled file is as it was when 2cF.3 was logged.
**`src_build_gate.py check`** (20:29:22-20:30:21): **PASS** — `[one-chain] … ONE chain setting program, robot and
beacon in every branch; no allowlisted file`, `[chain-table] 6 builds, 3 robots, matching`, `[program-table] 5
programs, 13 entry names, matching`; six builds 8/8 TUs, 0 disallowed.

**The prose review — the greps run BEFORE this row was written** (living documents: `README.md`, `docs/*.md`,
`docs/guide/`, `docs/faq.md`, `docs/cookbook/`, and in `docs/internal/` `RESUMING`, `build-order`, `HANDOFF-remaining-work`,
`PROJECT-BRIEFING`, `ORIENTATION`, `RELEASING`, `guide-maintenance`, `docs-publishing`, `verify/`, the worksheet; and the
comments of `include/`, `src/`, `test/`, `tools/`, `Makefile`, `common.mk`, `.github/`):

```sh
P='unused-function|allowed by category|ALLOWED_CATEGOR|residual|allowlist|one chain|#if chain|main\.cpp.s (one )?chain|chain in src/main|kVariantBeacon|kProgram|kRobot\b|RobotId|SHULIB_ROBOT_|SHULIB_PROGRAM_|CXXFLAGS_EXTRA|build_selection|chooses which program|only chooses|selected by the same define|same define|robot beacon|shulib-robot='
grep -rnE "$P" <the living documents>                      # every hit read
grep -rnE 'src_build_gate|src build gate|beacon|step 0|step 8|self-test' <the living documents>
grep -rnE "$P|chassis_table|mechanism_table|kChassis\b|kMechanisms\b" include src test tools Makefile common.mk .github
grep -rnE 'tank define|variant define|under (its|the) define|selected by the|#else of|bench bot.s table' <all of the above>
grep -nE 'booted:|first line|serial log|\[BLD\]|robot tables' <the worksheet, guides, README, docs/*.md>
```

| File | Finding | Disposition |
|---|---|---|
| `docs/internal/RESUMING.md` :136-138 | the `CXXFLAGS_EXTRA` history "survives only in comments … (`Makefile`, `src/main.cpp`)" — FALSE: that comment moved to `src/build_selection.hpp` with the guards | FIXED: names `src/build_selection.hpp`, and that `main.cpp` read the defines until this run |
| `docs/internal/build-order.md` :690-691 (GATE1's entry, not a status paragraph) | "`-Wunused-function` allowed by category with the residual hole stated" — GATE1's record, true of its day | KEPT, with a dated parenthetical: a FAILURE since this run, the team lead's ruling |
| `docs/internal/build-order.md` :1933 (the deviations table, the third variant) | "the beacon is the only detector that can see it" (a tank build carrying the bench bot's table) — measured FALSE at G1 Part 2c (S7: that build PASSED) | KEPT, with a dated correction naming the measurement and what closes it now |
| `docs/roadmap.md` :1264-1267 (GATE1's bullet — not "Now" or "You are here") | the previous run's inline note left "(the residual hole is stated in the tool)" reading as present tense; the tool no longer states one | REWORDED: "everything else fails (`-Wunused-function` excepted by category at GATE1 … a FAILURE … since G1 Part 2c's final run)" |
| `src/main.cpp` :33-35 | "The ROBOT column follows the same chain" — the table above it has no ROBOT column; and "the invented X-drive's" tables are an UNSET chassis table and no mechanisms | REWORDED (comment only) |
| `README.md` :250-257, :285-291; guides 07 :122-131, 08 :50-55; `Makefile`; `ci.yml` | the previous run's edits (the tree shows them at 14:09; 2cF.3 logs only the Makefile and CI) | READ against the tree: true |
| `docs/hardware-assumptions.md` :213, :753, :757 | "`src/main.cpp` … now a dispatcher and carries no port"; "the xdrive build still carries the invented map" | true; not changed |
| the worksheet :4, :422-423, :535, :753-756, :1010, :1492-1494, :896 | the `make` commands (unchanged by this run); no line says which serial line comes FIRST at boot, so the new `[BLD]` line falsifies nothing | not changed; an OPTIONAL check for the coordinator (2cF.8) |
| `Makefile` :69 | "as documented in src/main.cpp" — the history of the transposed flag, true of its day | not changed |
| **`src/teleop_program.hpp` :30-31, `src/teleop_program.cpp` :42-43** | "the BEACON stays a literal in main.cpp's #if chain" — FALSE since this run (it is `src/build_selection.hpp`'s chain) | NOT TOUCHED — T3 Part 2's files; listed for the coordinator (2cF.8) |
| **`src/boot_source.cpp` :12-16** | "nothing here reads src/main.cpp's SHULIB_RUNS_* macros" (vacuous since 2c) and "In the bench and xdrive builds … kChassis is the bench bot's table" — FALSE for xdrive since this run (`kNoChassis`, ruling F1) | NOT TOUCHED — listed (2cF.8) |
| `src/teleop_program.cpp` :61, `include/shulib/teleop/teleop_loop.hpp` ~~:60-63~~ :78-81 [re-pointed at 2cR.5: T3 Part 1 moved the text; :58-64 is the tick-order list] | 2c's follow-ups (the retired macros; the X-drive as a second caller) | NOT TOUCHED — listed (2cF.8) |
| `docs/internal/HANDOFF-remaining-work.md` :277, :302, :311 | "`src/main.cpp` only chooses which program a build is"; the gate's "program check over one chain"; the X-drive helpers' `-Wunused-function` | NOT TOUCHED — the coordinator's; listed (2cF.8) |

**Change 1, found by the review — the chain's robot constant is renamed `kRobot` → `kRobotId`.** The library already
documents the name for something else: `include/shulib/config/robot_config.hpp` :22 (and `docs/api/robot_config.md`)
teaches ``inline constexpr RobotConfig kRobot{…};`` as THE way to type a robot. With the chain's constant named
`kRobot`, a student (or G3a's generator) writing that idiom in `src/` fails step 0 ("declares one of THE ONE CHAIN's
constants" — the check is by name, namespace-blind), and inside `shulib::bench` it is a C++ redefinition. `kRobotId`
pairs with its type the way `kProgram` pairs with `Program`. *Alternative:* keep `kRobot` (the coordinator's
"e.g. kRobot") and change the library's documented idiom — rejected: it edits a public header's documentation to
make room for a `src/` name, and the doc is the older, public one (the decision-consistency rule: a new name must not
break what is already written). Mechanical (`\bkRobot\b` matches none of `kRobotBeacon`, `kRobotTwoChassis`,
`kRobotTwoMechanisms`): `src/build_selection.hpp`, both tables, `src/main.cpp` (comments), the three test files, the
gate (its regexes, messages, plants), and `build-order.md` :1933. **2cF.2 and 2cF.3 above keep the name they were written
with** — read `kRobot` there as `kRobotId`.

**Change 2, found re-reading the ruling — step 0 reads `include/shulib/` and `test/` too.** The ruling says the
switches are read in the one header ONLY; step 0 looked under `src/` only. Every `src/*.cpp` includes the library's
headers, which see the same `-D` flags, so a `#ifdef SHULIB_ROBOT_TANK_2026` in `include/shulib/` would be a second
selection no detector reads; and a host test that `#define`s a switch before including a table is the ODR violation
2cF.2 rejected as alternative (B) — two definitions of `kProgram`/`kRobotId`/`kChassis` across the suite, silently. Today
neither tree names a switch or declares a chain constant in code (grep: one hit, `robot_config.hpp` :22, inside a
comment). The vendored `include/pros/`, `include/liblvgl/` and `test/vendor/` are not read (third-party code that
never names our switches; the gate ignores them by path for warnings too). *Alternative:* leave step 0 at `src/`
and rely on the host `static_assert` in `test/build_selection_test.cpp` — rejected: that catches a switch reaching
the HOST build's flags, not a library header or a test TU reading one. The self-test gains a plant in each.

**The two changes, made (20:36-20:44 EDT):** the rename (`perl -pi -e 's/\bkRobot\b/kRobotId/g'` over the eight files
named above; afterwards `grep -rnE '\bkRobot\b'` outside this log and `docs/api/` prints nothing); step 0 split into
`_stray_selection_problems()` (one file) and `_selection_header_problems()` (the header), walking `ONE_CHAIN_ROOTS` =
`src/`, `include/shulib/`, `test/` (C/C++ extensions only, `test/vendor/` skipped); the header's step 0 paragraph, the
`check` summary line (`… (src/, include/shulib/, test/ searched) …`), `build_selection.hpp` :10-12 and :20-22, the
`Makefile` :56 and the CI comment say so. Self-test case 19 gains five checks: a switch in a library header
(`include/shulib/config/robot_config.hpp`) and in a host test (`test/build_selection_test.cpp`) REPORTED, a test that
`#define`s one REPORTED, a switch in `test/vendor/doctest.h` NOT read, and the library's `kRobot{…}` idiom planted in
`src/` NOT reported (the rename's reason, as a test).

| Run (after both changes) | Result |
|---|---|
| host build, fresh (`touch test/*.cpp`, 20:41:33) | exit 0, `[100%] Linking CXX executable shulib_tests` |
| host suite | **1543 cases / 2,394,206 assertions, 3 skipped — SUCCESS** (unchanged: the rename is a name) |
| gate `check` (20:42:31-20:43:55) | **PASS**; `[one-chain] the build switches are read in src/build_selection.hpp only (src/, include/shulib/, test/ searched), by ONE chain setting program, robot and beacon in every branch; no allowlisted file` |
| gate `self-test` (20:44:12-20:50:24) | **`src build gate self-test: OK (119 detector cases, each against a real build)`** — 114 at 2cF.3 + case 19's five |

A sha256 manifest of `src/`, `include/`, `test/`, `tools/`, `Makefile`, `.github/` (600 files) was taken at 20:50, before
the mutations (`scratch/2cfinal/manifest-before-mutations.txt`); the tree is compared to it after the last restore.

### 2cF.6 The mutation set, re-run on the FINAL code (from 20:51 EDT)

The harness: `.claude/coordinator/scratch/2cfinal/mutate.py` (reboot-safe: the original bytes go to `scratch/2cfinal/backup/`
and an `APPLIED.json` marker names the file and its sha256 BEFORE a plant is written; the marker goes only after the
restore is verified byte-exact; a harness that starts and finds a marker restores from the backup and stops). Kinds:
**host** — `touch test/*.cpp`, `cmake --build build/test --target shulib_tests/fast -j8`, REQUIRE `Linking CXX executable
shulib_tests` (a failed compile is recorded with its error as killed-by-compile), then the WHOLE suite; **builds** — step
0's three source checks and the gate's own `run_build()` + `verdict()` on fresh `bin/`/`.d/` for the builds named;
**check** — the full `src_build_gate.py check`; **self** — the full `self-test` against the mutated gate. Every result
below is from `scratch/2cfinal/mutations.jsonl`.

**Batch A — the tables' selection, the robot beacon, the host's branch (20:52-20:57; host names re-run 21:01-21:04).**
The first pass's failing-case NAMES were wrong (doctest prints a `TEST CASE:` header for MESSAGE/WARN output too, and
the harness listed every such case); its counts were right. The parser was fixed (a case failed only if its block has an
`ERROR:`/`FATAL ERROR:` line) and the six suite-killed mutations re-run; the names below are the re-run's. Between the
two passes the tree was compared to the manifest: identical, no marker left. (A caution the run made concrete: after
MC2's host compile failed, the binary left in `build/test/` was MB1's MUTATED one — the harness never ran it because the
link line was absent; the final gates relink from the restored tree.)

| # | Mutation (final code) | Host suite (fresh binary) | Package / step 0 |
|---|---|---|---|
| MT1 | `chassisTableFor()`: `RobotTwo` → `kBenchBotChassis` | **RED** 1 case: `chassis table: the selection …` | **tank FAILS**: `static assertion failed: ROBOT=tank must select robot two's SIGNED table` (`chassis_table.hpp:227`) |
| MT1b | MT1 AND that static_assert deleted | **RED** — the same case | tank PASSES (the table is unsigned, not a port clash) — **the host case is the mapping's independent guard**; the two are two ways to be caught, not one |
| MT2 | `kChassis = chassisTableFor(RobotId::BenchBot)` — the selection names a fixed, wrong enumerator | green — BY DESIGN (the host IS the bench build) | **xdrive and tank FAIL**: `kChassis must be chassisTableFor(kRobotId)` (:220) + the per-robot assert (:227 / :232) |
| MT3 | a misspelt enumerator, `RobotId::RobotTwoo` | **COMPILE ERROR**: `'RobotTwoo' is not a member of 'shulib::bench::RobotId'` (+ `-Wswitch`, an error on the host's `-Werror`) | **bench FAILS** — the same error |
| MT4 | robot two's REAL table: `-13` → `13` | **RED** 1 case: `chassis table: robot two's REAL table (kRobotTwoChassis) is the measured one …` | tank PASSES — no compiler can know a sign; **the gap 2cF.2 named (the suite could only re-type robot two's ports) is closed by that case** |
| MT5 | the X-drive's `kNoChassis` given a port | **RED** 1 case: `chassis table: the invented X-drive's table is UNSET in every field …` | — |
| MM1 | `mechanismTableFor()`: `RobotTwo` → `kNoMechanisms` | **RED** 1 case: `mechanism table: the selection …` | **tank-library FAILS**: `ROBOT=tank must select robot two's mechanism table` (`mechanism_table.hpp:598`) |
| MM2 | `kMechanisms = mechanismTableFor(RobotId::BenchBot)` | green — BY DESIGN | **tank-library FAILS** — the same assert |
| MM3 | `mechanismTableFor()`: `BenchBot` → `kRobotTwoMechanisms` | **COMPILE ERROR**: `static assertion failed: only robot two declares mechanisms` (:602, evaluated on the host) | **bench FAILS** — the same |
| MB1 | `robotBeacon()`: the bench bot's text → robot two's | **RED** 3 cases: the three `build selection:` cases | **step 0** chain table: `robotBeacon() returns {… 'BenchBot': 'shulib-robot=robot-two' …}, but this gate's ROBOT_BEACONS says …`; **bench FAILS**: `robot beacon 'shulib-robot=bench-bot' is NOT in` + `carries the robot beacon 'shulib-robot=robot-two'` |
| MC2 | the `#else` (bench) branch sets `RobotId::RobotTwo` | **COMPILE ERROR**: `build_selection_test.cpp:42 static assertion failed: the host sets no build switch, so it must take the chain's #else` | **step 0** chain table: `build bench's branch (src/build_selection.hpp:129) sets kRobotId = RobotId::RobotTwo, but this gate's BUILD_ROBOT says BenchBot`; **bench FAILS** on both robot-beacon halves |

**Batch B — package only (21:04-21:05).**

| # | Mutation | Result |
|---|---|---|
| MB2 | `main.cpp`'s `[BLD]` line deleted — the robot beacon no longer linked | step 0 silent (a `using` nobody uses warns nothing); **bench and tank-library FAIL**: `robot beacon … is NOT in bin/hot.package.elf`. The message then blamed only "the chain's branch … set another robot" — **the gate's message was corrected** to name both causes (`…; or src/main.cpp no longer prints kRobotBeacon, so nothing links it`); the self-test's expectations match on the unchanged prefix |
| MC3 | the xdrive branch sets `RobotId::RobotTwo` | **step 0** chain table: `build xdrive's branch (src/build_selection.hpp:109) sets kRobotId = RobotId::RobotTwo, but … InventedXDrive`; **xdrive FAILS**: the X-drive's robot beacon absent, robot two's present |
| D1 | RULING 2 on the REAL gate: a new dead function in `src/bench_r3a.cpp` | **bench FAILS**: `disallowed warning -Wunused-function at src/bench_r3a.cpp:3266 — 'void {anonymous}::mutation_d1_dead_function()' defined but not used` |

**Batch C — each through the FULL `src_build_gate.py check` (21:06-21:12).**

| # | Mutation | `check` |
|---|---|---|
| **MC1 — the ruling's case** | the tank-drive branch sets `kRobotId = RobotId::BenchBot` (right program, right variant beacon) | **FAIL, exit 1**: `[chain-table] build tank-drive's branch (src/build_selection.hpp:113) sets kRobotId = RobotId::BenchBot, but this gate's BUILD_ROBOT says RobotTwo`; `[tank-drive] … robot beacon 'shulib-robot=robot-two' is NOT in bin/hot.package.elf`; `[tank-drive] … carries the robot beacon 'shulib-robot=bench-bot'`. The package compiles and links — only the robot beacon (from the package) and the chain table (from the source) see it; the other five builds pass |
| **MP1** | `chassis_table.hpp` selects `kChassis` on its own `#if defined(SHULIB_ROBOT_TANK_2026)` again (the SAME tables — nothing else can see it) | **FAIL, exit 1**, step 0 ONLY: `[one-chain] src/chassis_table.hpp:214 names a build switch — only src/build_selection.hpp may read them … #if defined(SHULIB_ROBOT_TANK_2026)`; all six builds pass |
| **MP2** | `mechanism_table.hpp` selects `kMechanisms` on its own `#ifdef SHULIB_ROBOT_TANK_2026` again | **FAIL, exit 1**, step 0 ONLY: `[one-chain] src/mechanism_table.hpp:324 names a build switch …`; all six builds pass |
| **MP3 — 2c's S7 probe, re-run** | `chassis_table.hpp`'s OLD shape back, misspelt (`#if defined(SHULIB_ROBOT_TANK_2026_BROKEN)` → robot two's, `#else` → the bench bot's): at G1 Part 2c this build **PASSED** | **FAIL, exit 1, three ways**: step 0 (`src/chassis_table.hpp:214 names a build switch`); `static assertion failed: kChassis must be chassisTableFor(kRobotId)` (:224) in xdrive and all four tank builds; `ROBOT=tank must select robot two's SIGNED table` (:231) and `the X-drive build must select the UNSET table` (:236). **The known gap 2c named is closed** |
| **MS1** | the robot SPLIT out of the one chain into a second chain of its own (every robot still right) | **FAIL, exit 1** at step 0: `src/build_selection.hpp:129 opens a SECOND chain on the build switches, outside THE ONE CHAIN` and six `the branch … sets kRobotId 0 times`; all six builds pass |
| **MS2** | MS1 with the tank define misspelt in the robot's chain | **FAIL, exit 1**: step 0 as MS1, AND all four tank builds `robot beacon 'shulib-robot=robot-two' is NOT in …` / `carries the robot beacon 'shulib-robot=bench-bot'` — no variant-beacon message (the program's chain is right): the robot beacon is the one detector in the package that sees it |

**Batch D — the gate's own detectors, each a FULL `self-test` run against the mutated gate (21:11-21:34).**

| # | Gate mutation | Self-test |
|---|---|---|
| **MG1 — Ruling 2** | `ALLOWED_CATEGORIES = frozenset({"-Wunused-function"})` (re-allowed) | **RED, exit 1** — case 3: `a -Wunused-function in src/ PASSED the gate — the category is allowed again (the team lead ruled it a FAILURE)`, `the failure did not name -Wunused-function and the file: []`, `a warning was ALLOWED by category: [('src/bench_r3a.cpp', '3266', '-Wunused-function')]` |
| MG2 | step 6's robot-beacon half disabled | **RED** — cases 17 (both robot-beacon messages missing), 24 (`a tank-library build carrying the bench bot's tables PASSED`), 25 (`a split chain with a broken robot #if PASSED the tank build`) |
| MG3 | step 0's chain table returns nothing | **RED** — cases 21 (wrong program), 23 (unknown robot), 24 (wrong robot) |
| MG4 | step 0 reads `src/` only again | **RED** — case 19: the switch in `include/shulib/config/robot_config.hpp`, the one in `test/build_selection_test.cpp` and the test that `#define`s one, all unreported |
| MG5 | both tables allowlisted again | **RED** — case 19: all six directive-form plants, both laundering `#define`s, both plain-code reads and both shadowing `kRobotId`s in the two tables unreported |

**Every mutation of the required set was RED** — a table's selection naming the wrong enumerator (MT2, MM2: the robot
builds fail; MT1, MM1, MM3: host red or a host compile error; MT3: a compile error everywhere); a branch setting the wrong
robot (MC1: `check` FAILS; MC2, MC3); a planted `#if` in each table (MP1, MP2: step 0 FAILS); `-Wunused-function`
re-allowed (MG1: self-test red); the robot split across two chains again (MS1: step 0 FAILS; MS2: step 0 and the robot
beacon). **31 records, every restore verified byte-exact by sha256**, no `APPLIED.json` left, and at 21:34:21 the whole
tree (600 files) equals the manifest taken after the last intended edit (`manifest-final.txt`).

### 2cF.7 Every gate, on the FINAL tree (21:34-21:44 EDT)

After the last intended edit (the MB2 message, 21:05) and the last restore (21:33:35); the tree equals
`manifest-final.txt` (21:34:21).

| Gate | Command | Final line |
|---|---|---|
| src build gate self-test | `python3 tools/src_build_gate.py self-test` (21:34:52-21:39:18) | `src build gate self-test: OK (119 detector cases, each against a real build)` |
| src build gate | `python3 tools/src_build_gate.py check` (21:39:18-21:40:17) | `src build gate: PASS (all 6 builds link; policy: errors fail, vendor warnings path-ignored, no category allowed, every other warning fails; defines token-exact; one chain picks program and robot; variant and robot beacons asserted; programs asserted)` — six builds 8/8 TUs, 0 disallowed, 0 `-Wunused-function` |
| ARM header gate | CI's command over every `include/shulib` header | `TU includes 185 headers`, `ARM compile gate: CLEAN (all v2 headers)` |
| CI greps | `<pros/>` outside `hal/pros`; `shulib/sim/` outside `sim/` | `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core is sim-free` |
| API docs | `api_doc_tool.py self-test`, `check-coverage`, `check-fresh`, `check-examples`, `check-removability` | `api_doc_tool self-test: OK`; `doc coverage: 2669 public entities across 149 headers, all documented`; `check-fresh` exit 0 (no `include/` header changed, so no regeneration); `437 quoted lines, all verbatim`; `removability: no public doc references docs/internal/` |
| Doc staleness | `doc_staleness_audit.py self-test`, then the audit | `self-test: OK (10 detector/filter cases)`; `clean (27 live docs, 0 numeric claims checked)` |
| Host build + suite | `briefing_status.py generate`; `touch test/*.cpp && cmake --build build/test -j8` (every doc gate inside); `./build/test/shulib_tests` | exit 0, `[100%] Linking CXX executable shulib_tests`; **1543 test cases / 2,394,206 assertions, 3 skipped — SUCCESS** |
| Briefing — LAST | `briefing_status.py generate` after that binary, the host build again, `briefing_status.py check` (21:44:20) | `briefing status: regenerated`; build exit 0; `briefing status: generated block current` (the Suite line now reads 1,543 / 2,394,206) |

**The suite's arithmetic** (2cF.3's, unchanged since): 1536 cases / 2,393,994 at `80e2615` + 7 new cases / 211
assertions + 1 assertion (`REQUIRE(&kChassis == &kBenchBotChassis)`) = 1543 / 2,394,206. **The package** (`strings -a`,
the last build, tank-calibrate): `shulib-robot=robot-two`, `[BLD] robot tables: %s (src/chassis_table.hpp,
src/mechanism_table.hpp)`, `shulib-robot-variant=tank-calibrate` — and no other build's or robot's beacon (step 6, every
build). `src/main.cpp` = 253 lines; `src/build_selection.hpp` = 158; `tools/src_build_gate.py` = 1,608.

### 2cF.8 The rulings against the task, deviations, residuals, and the coordinator's list

> **SUPERSEDED for the coordinator by 2cR.7 below** (the review's findings: step 0's new rules, corrected
> counts and pointers, and the merge-collision list completed by a trial merge). Kept as written, with the two
> corrections marked in place.

**Ruling 1 — "one chain picks both" — done, with evidence:**

- [x] ONE small shared header, `src/build_selection.hpp` (158 lines): the five `#error` guards (:63-78), `enum class
      Program` (:83), `enum class RobotId` (:93), THE ONE CHAIN (:109-133 — six branches, each setting `kProgram`,
      `kRobotId` and `kVariantBeacon`, nothing else), `robotBeacon()` (:142) and `kRobotBeacon` (:156). `src/main.cpp`
      includes it and keeps its dispatch (253 lines; `grep -n "SHULIB_" src/main.cpp` prints nothing; the five `switch
      (kProgram)` unchanged).
- [x] The tables choose from `kRobotId` with NO `#if` of their own: `chassisTableFor()` / `kChassis`
      (`src/chassis_table.hpp` :200-214) and `mechanismTableFor()` / `kMechanisms` (`src/mechanism_table.hpp` :309-324);
      a misspelt robot is a compile error (MT3), a wrong enumerator in the selection fails the robot builds (MT2, MM2).
- [x] The switches are read in that header ONLY — step 0 (no allowlist; `src/`, `include/shulib/`, `test/`); a planted
      `#if`/`#ifdef`/`#elifdef` in either table FAILS it (self-test case 19; MP1, MP2); the header's chain passes.
- [x] A wrong robot INSIDE a branch fails: the ROBOT BEACON (a second beacon, derived from `kRobotId`, asserted per
      build at step 6) and step 0's chain table (self-test case 24; MC1, MC2, MC3).
- [x] Host tests read the REAL tables: robot two's chassis table (`kRobotTwoChassis`) and claws (`kRobotTwoMechanisms`)
      by name, both selection functions for every `RobotId` (MT1, MT4, MT5, MM1).
- [x] Every existing static_assert on the tables kept (2cF.4's count); one added (check 8 on robot two's real pair) and
      four on the chassis selection.
- [x] 2c's known gap (`chassis_table.hpp`'s own `#if`) is CLOSED by the same change — MP3, the S7 probe that PASSED at 2c,
      now fails three ways.

**Ruling 2 — "`-Wunused-function` becomes a FAILURE" — done:** `ALLOWED_CATEGORIES = frozenset()`; the gate header's
policy table and its "-Wunused-function IS A FAILURE" paragraph; self-test case 3 inverted; the `Makefile` WARNFLAGS
rationale and the CI comment; `build-order.md` :690-693 and `roadmap.md` :1264-1267 (GATE1's records) dated. D1 (a
dead function, the real gate) FAILS; MG1 (the category re-allowed) turns the self-test red.

**Deviations, each with its reason (and where it is logged):**

1. **From the G1 Part 2 brief, §2c item 4** ("`-Wunused-function` stays allowed by category") — superseded by the team
   lead's Ruling 2 of 2026-09-26 ~13:15 (2cF.3).
2. **The robot is named by a SECOND beacon, derived — not folded into the variant beacon** (the ruling allows "or a
   second beacon the gate asserts per build"): folding it in would change every program's banner text (one of them
   T3's file) and a typed beacon could not vouch for the `kRobotId` beside it (2cF.2, F2).
3. **`kRobotId`, not the ruling's example `kRobot`** — the library already documents `kRobot{…}` as the idiom for typing a
   `RobotConfig` (2cF.5, change 1).
4. **Step 0 reads `include/shulib/` and `test/` too**, beyond the `src/` the 2c gate read (2cF.5, change 2).
5. **The xdrive build carries an UNSET chassis table (`kNoChassis`)**, no longer the bench bot's (2cF.2, F1) — no
   runtime string of that package changes except the new line below.
6. **ONE new serial line at boot in all six builds**, before each program's banner:
   `[BLD] robot tables: shulib-robot=<bench-bot|robot-two|invented-xdrive> (src/chassis_table.hpp,
   src/mechanism_table.hpp)` — the only runtime change of this run; ASCII, a literal format, serial only (no screen).

No HA number was needed: nothing here is a belief about hardware (HA-163/HA-164 unused).

**Residuals, measured, not hidden:**

- MT1b — a wrong table inside `chassisTableFor()` WITH its per-robot `static_assert` also deleted passes every package
  check; the host case "chassis table: the selection" catches it. The two are independent guards by design.
- MT4 — a wrong VALUE in a table (a dropped sign) passes every package check; only the host test (robot two's REAL table,
  pinned to the brief's literals) catches it — which it could not before this run.
- Step 0 is textual: a switch spelled through token pasting (`SHULIB_##ROBOT_…`) is invisible to it. Nobody writes that
  by accident; stated so nobody trusts the check further.
- Unchanged from 2c: step 8 cannot see `main.cpp`'s own painted sentences swapped (the gate header says so).

**For the COORDINATOR at the final merge (none of these files was touched by this run):**

1. **False since this run, in files other runs own** — fix at merge:
   - `src/teleop_program.hpp` :30-31 and `src/teleop_program.cpp` :42-43: "the BEACON stays a literal in main.cpp's #if
     chain" → "…in src/build_selection.hpp's one chain".
   - `src/boot_source.cpp` :14-16: "In the bench and xdrive builds src/chassis_table.hpp's kChassis is the bench bot's
     table" → the bench build carries the bench bot's table, the xdrive build the UNSET `kNoChassis`; still harmless
     (no program in either asks for a boot source).
   - `src/boot_source.cpp` :12-13 and `src/teleop_program.cpp` :61 (2c's item, now sharper): "nothing here reads
     src/main.cpp's SHULIB_RUNS_* macros" → "nothing here reads a build switch (only src/build_selection.hpp may; the
     src build gate's step 0 fails one named here)".
   - `include/shulib/teleop/teleop_loop.hpp` ~~:60-63~~ :78-81 [re-pointed at 2cR.5] (2c's item): the X-drive is the second caller.
   - `docs/internal/HANDOFF-remaining-work.md` :277 — FALSE: "`src/main.cpp` only chooses which program a build is" →
     `src/build_selection.hpp` chooses the program AND the robot, `main.cpp` hands it the entry points; :302 — true but
     incomplete: the gate now also asserts a per-ROBOT beacon, reads the switches' one chain from
     `src/build_selection.hpp` (step 0 over `src/`, `include/shulib/`, `test/`), and fails `-Wunused-function`. (:311,
     "the variant differential requires the X-drive helpers' `-Wunused-function`", is in the paragraph HANDOFF keeps "as
     it was written before the go" — history; no change.)
2. **Merge collisions to expect:** `src/mechanism_table.hpp` — T3 Part 2's brief edits the `kTeleopRezeroButton` doc
   comment (~:140-148 at `c845041`); this run's hunks are the header (:3-12), the host-test paragraph (:81-89), the check
   list (:112-129), the include (:139), :236, the selection (:301-325) and the static_asserts (:576-605) — adjacent, not
   the same lines. T3 Part 2 reads `kMechanisms` (name and type unchanged). **Any `#if` on a build switch that T3 Part 2
   adds anywhere in `src/`, `include/shulib/` or `test/` FAILS step 0 on the merged tree**; its build's robot comes from
   `kRobotId`, never a define. `build-order.md` (:690-693, :1934) and `roadmap.md` (:1264-1267) carry this run's hunks
   beside the coordinator's status edits.
3. **The COMP brief §3.3** — 2c's proposed amendment (2c's coordinator item 4) is stale against this run. Proposed
   replacement text (a plan change: proposed, not made): *"COMP-b adds the seventh program by: a `Program::Competition`
   enumerator; ONE branch of `src/build_selection.hpp`'s chain (`#elif defined(SHULIB_ROBOT_TANK_2026) &&
   defined(SHULIB_PROGRAM_COMPETITION)` setting `kProgram = Program::Competition`, `kRobotId = RobotId::RobotTwo` and
   `kVariantBeacon = "shulib-robot-variant=tank-competition"`, nothing else) plus its `#error` guards; a case in EACH of
   `src/main.cpp`'s five entry-point switches; in `tools/src_build_gate.py`, `VARIANTS`, `BUILD_ARGS`,
   `VARIANT_DEFINES`, `VARIANT_BEACONS`, `BUILD_PROGRAM`, `BUILD_ROBOT` (`RobotTwo` — its robot beacon is robot two's,
   no new `ROBOT_BEACONS` entry) and `PROGRAM_ENTRY_SYMBOLS`, and a self-test pair like cases 14/15. The define is read
   in `src/build_selection.hpp` ONLY."*
4. **Status and records (the coordinator's):** `build-order.md`'s first `**Next:**` and `roadmap.md`'s "Now"/"You are
   here" — G1 Part 2c landed WITH the team lead's two rulings; **`G1-COMPLETED.md`** — this section's summary (the
   rulings, the six deviations, the residuals, the mutation table, the gates) beside 2c's; regenerate the briefing
   (`briefing_status.py generate` after the merged suite binary exists) and `docs/api/` after merging (this run
   changed no `include/` header).
5. **The G1 Part 2 brief §2c item 4** still says "stays allowed by category" — a plan document; annotate it or leave it
   with this log's deviation 1 as the record (the coordinator's call).
6. **After the commit (landmine 9):** rebuild the six packages from the COMMITTED tree; the gate asserts both beacons
   per build. At the first boot of each program the first line `initialize()` prints is now `[BLD] robot tables: …`
   (before the program's banner) — the worksheet could add it as an upload-identity check at G.1/H.0.1 (optional; not added here — T3 Part 2 / Station J
   may be editing the worksheet, and nothing in it is false).
7. **Nothing has run on a robot.** No package was uploaded; `pros` never run; `/dev/ttyACM*` never touched.

`bin/`, `.d/` and the interrupted run's `tools/__pycache__/` removed at the close (all gitignored). Nothing committed, staged or
pushed; `pros` never run; `/dev/ttyACM*` never touched; the main tree and every other worktree untouched. Scratch evidence
(the harness, `mutations.jsonl`, every run log, both manifests) is kept under `.claude/coordinator/scratch/2cfinal/`.

## Part 2c final: review findings and their disposition — 2026-09-26 22:48:44 EDT

> **Part:** the builder's pass over the independent review of Part 2c final (2cF.0-2cF.8 above): five findings (one
> major, four minor), each re-checked against the tree, fixed at the root within scope or answered with evidence.
> **Executor model:** opus. Appended in real time; every timestamp from a `date` call. Nothing committed, staged or
> pushed; `pros` never run; `/dev/ttyACM*` never touched; the main tree and every other worktree never touched; the
> files other runs own (2cF.0's list) never touched. Scratch evidence under `.claude/coordinator/scratch/2cfinal/`
> (reboot-safe), never `/tmp`.

### 2cR.0 Resumed: what this run found (22:48 EDT)

This pass was started by the same request as 2cF.4 ("my computer died before things finished"). What the tree says:
the machine has been up since 20:12:41 EDT (`uptime -s`), so no second power loss since 2cF.4; the last write to the
worktree is this log at 21:55:38 (2cF.8), and nothing under `src/`, `include/`, `test/`, `tools/`, `Makefile` or
`.github/` has changed since the manifest taken after the last intended edit: `sha256sum -c
scratch/2cfinal/manifest-final.txt` — all 600 files OK, and the same file set (no file added or removed). No
`APPLIED.json` marker in `scratch/2cfinal/`, no `bin/`/`.d/`/`__pycache__`. So no mutation is left applied and no
earlier attempt at THIS pass left anything: the review findings are untouched, and this pass starts from 2cF.8's tree.
Next: the host build and the whole suite on that tree, then the findings one by one.

### 2cR.1 The baseline on 2cF.8's tree, and each finding re-checked (22:49-23:05 EDT)

**Host build and whole suite on the tree as found** (`cmake -S test -B build/test`; `briefing_status.py generate`;
`touch test/*.cpp && cmake --build build/test -j8`; `./build/test/shulib_tests`; logs `scratch/2cfinal/r-*-0.log`):
exit 0, `[100%] Linking CXX executable shulib_tests`, every doc gate green; **1543 test cases / 2,394,206 assertions, 3
skipped — SUCCESS**, exactly 2cF.7's numbers.

Each finding re-checked against the tree BEFORE any change (the reviewer's evidence re-derived, not quoted):

- **F1 (major) — REAL.** `grep` of `src/` and `include/shulib/` for the five per-robot names and the two selection
  functions: outside the two tables they appear only in comments (`src/build_selection.hpp` :19, :50-51, :91-92). Nothing
  in the gate reads them: step 0 looks for switches and for the three chain constants, the chain table and step 6 key on
  `kRobotId`, step 8 on program symbols; the tables' static_asserts check `kChassis`/`kMechanisms` only; and
  `test/CMakeLists.txt` globs `*_test.cpp`, so no program TU is host-compiled. `bench_r3a.cpp` :158 `tableSideOf(kChassis,
  port)` → `kRobotTwoChassis` would change nothing any check reads. The reviewer is also right that it is NEW for the drive
  table: at `80e2615` each build defined one `kChassis` and no other chassis name. And the class is wider than the names
  it lists: `chassisTableFor(RobotId::RobotTwo)` / `mechanismTableFor(…)` in a program is the same defect by another spelling.
  T3 Part 2's worktree (read-only, `git --no-optional-locks`) names `kRobotTwoMechanisms` in THREE static_asserts —
  `src/teleop_program.cpp` :170, `src/boot_source.hpp` :143 and its new `src/teleop_claws.hpp` :374 — and its host test
  `test/teleop_claws_test.cpp` by name; any fix must keep all four legal.
- **F2 (minor) — REAL.** `.github/workflows/ci.yml` :118-119 still says "the stated residual hole live[s] in
  tools/src_build_gate.py's header" (the header states none now); the gate's :328 says "its branch of src/main.cpp's
  chain". 2cF.5's grep matched both and its table named neither — a miss of the logged review.
- **F3 (minor) — REAL, all four parts.** (a) `ONE_CHAIN_EXTS` has no `.c++` (the `Makefile`'s `CXXEXTS:=cpp c++ cc`)
  and no `.s`/`.S`. (b) `DECLARES_CHAIN_CONSTANT` cannot match `#define kRobotId RobotId::BenchBot`; traced: `kRobotBeacon`
  is derived in `build_selection.hpp` BEFORE a table's macro, so robot two's beacon stays, while `kChassis`, all four chassis
  static_asserts and (through the include) `mechanism_table.hpp` read the macro — a tank build with the bench bot's table
  and no claws, every check green. (c) No self-test plant uses `kRobotId{…}`. (d) :309-310 holds for GLOBAL host flags
  only; a per-source `COMPILE_DEFINITIONS` in `test/CMakeLists.txt` is seen by neither step 0 nor the static_assert.
- **F4 (minor) — REAL.** `git show 80e2615:src/mechanism_table.hpp | grep -cE '^\s*static_assert\('` = 12, now 13 (the
  16 → 17 of 2cF.4 is `grep -c static_assert`, which counts four comment lines); robot two's checks at `80e2615` were
  EIGHT (1-7 and 9; check 8 was build-pair only). `include/shulib/teleop/teleop_loop.hpp` :58-64 is the tick-order list;
  "Since G1 Part 2b "shulib Teleop" … drives through it" is :78-81.
- **F5 (minor) — REAL.** T3 Part 2's committed transport (`1a86103`, base `cc513a7`; `git --no-optional-locks diff
  --stat cc513a7 1a86103`) touches `test/mechanism_table_test.cpp` (+2/-1), `test/boot_source_test.cpp` (+377) and
  `src/mechanism_table.hpp` (53 lines) — hunks listed in 2cR.7 item 2.

Next: the fixes, in the gate first (F1, F3, F2's gate line), then the prose (F2's CI line, the tables' and the selection
header's comments), then the log corrections (F4, F5).

### 2cR.2 The fixes (23:05-23:21 EDT)

**F1 — a per-robot table named outside the tables FAILS step 0** (`tools/src_build_gate.py`: header "A ROBOT'S TABLES
BY NAME"; `per_robot_table_names()`, `_blank_static_asserts()`, the rule in `_stray_selection_problems()`; self-test
case 26). The per-robot names are DERIVED from `TABLE_FILES` (`src/chassis_table.hpp`, `src/mechanism_table.hpp`), never
listed in the gate: every `constexpr ChassisTable|MechanismTable <name>` they define, their selection functions
(`constexpr const …Table& <name>(RobotId …) {`) and every name those functions `return` — today `kBenchBotChassis`,
`kRobotTwoChassis`, `kNoChassis`, `chassisTableFor`, `kRobotTwoMechanisms`, `kNoMechanisms`, `mechanismTableFor`.
Fail-closed: a table file in which step 0 finds no selection function, no table or nothing returned is itself a
step-0 FAILURE (a rule over an empty set would check nothing, in green). The names may appear in code (comments and
literals stripped) only in the two table files, under `test/`, and inside a `static_assert(…)`'s parentheses — exactly
the parentheses: code after the closing one on the same line is read. Anywhere else under `src/` or `include/shulib/`
FAILS, naming the file, the line and the name. Reported under `[one-chain]`: naming a robot's table IS choosing a robot
outside the chain. The review's own mutations are self-test plants (the bench tester's IMU from `kRobotTwoChassis`,
shulib Drive's ports from `kBenchBotChassis`, Calibrate's drop list from `mechanismTableFor(RobotId::BenchBot)`), with
every derived name planted in a program, a library header, the legal forms (T3 Part 2's multi-line static_assert shape, a
comment, a string, a host test) NOT reported, a NEW robot's table covered the moment it exists (by the object pattern,
and separately by the selection function's `return`), and an unrecognised selection function FAILING.
*The design and its alternative:* the reviewer's alternative, a nested namespace (e.g. `bench::robots::`), makes a use
visible to `grep` but enforces nothing, and — measured against T3 Part 2's transport `1a86103` — would break its three
static_asserts (`src/teleop_program.cpp` :170, `src/boot_source.hpp` :143, `src/teleop_claws.hpp` :374) and its host test's
`using shulib::bench::kRobotTwoMechanisms;` at the merge (the decision-consistency rule). C++ access control (the tables as
private members, a friend accessor for the tests) was rejected for the same break, and because a program can still call
the selection function with a fixed robot — which the textual rule catches by name. **Measured on the merge:** step 0 run
over a SIMULATED merged tree (this worktree + T3 Part 2's `1a86103` for the files only it changes + a scratch `git
merge-file` of the three `src/`/`test/` files both change; `scratch/2cfinal/merged-tree/`, nothing written to either
worktree): `one_chain_problems` 0, `chain_table_problems` 0, `program_table_problems` 0 — T3 Part 2's three static_asserts
are legal as written. **What it does not cover (stated, not hidden):** a program that types ANOTHER robot's ports into a
table of its own (`constexpr ChassisTable kMine = {…}` in a program) — a second table, which the chassis table's own rule
("the ONE typed input about a drivetrain") forbids and nothing enforces; not new in this run, and no program does it
(grep); and `src/xdrive_config.hpp`'s `kInventedXDrive`, which is the X-drive PROGRAM's own `RobotConfig` (selected by
`kProgram` through `main.cpp`'s dispatch and step 8), not a per-robot table.

**F2 —** `.github/workflows/ci.yml` :118-120 now says "The warning policy (no category allowed since G1 Part 2c's final
run: every warning in our code fails) and the self-test live in tools/src_build_gate.py's header"; the gate's
BUILD_PROGRAM comment says `src/build_selection.hpp`'s chain. The CI self-test comment (:141-156) also names the new plants.

**F3 —** (a) `CXX_EXTS`/`ALL_EXTS` moved up to step 0's tables and `ONE_CHAIN_EXTS = ALL_EXTS + (".cxx", header
extensions)` — generated from the Makefile's mirror, so `.c++`, `.s` and `.S` are read (a `.c`/`.s`/`.S` file never
receives a switch — they ride in `EXTRA_CXXFLAGS` only — so a switch named there is a wrong answer in every build).
(b) `SELECTION_NAMES` (`kProgram`, `kRobotId`, `kVariantBeacon`, `kRobotBeacon`, `robotBeacon`, `Program`, `RobotId`,
`kChassis`, `kMechanisms`) plus the derived per-robot names may never be a macro: a `#define`/`#undef` of one, in ANY file
under the roots — the header, the tables and `test/` included — FAILS (`_macro_problems()`). (c) `DECLARES_CHAIN_CONSTANT`
also matches the `(…)` initializer form, and self-test case 27 plants the `{…}` and `(…)` forms in a table and in the
header outside the chain. (d) Step 0 reads the host build's CMake files (`CMakeLists.txt`, `*.cmake` under the roots;
`#` and `#[[…]]` comments stripped, a quoted `#` kept) — a per-source `COMPILE_DEFINITIONS` naming a switch FAILS; the
comment at the old :309-310 now says the host static_assert sees GLOBAL flags only, and why CMake is read. Case 27
plants each: a `.c++` and a `.S` probe file (created and removed), `#define kRobotId RobotId::BenchBot` after each table's
include, `#undef kChassis` in a program, `#define kRobotTwoChassis kBenchBotChassis` in `main.cpp`, `#define kProgram …`
in the header, `#define kMechanisms …` in a test, two CMake forms reported and two comment forms not.

**Prose made true with them:** `src/chassis_table.hpp` :13-16 and :215-217, `src/mechanism_table.hpp` :85-88 and :323-325,
`src/build_selection.hpp` :43-49 (WHAT VOUCHES FOR WHAT) and :57-58 (THE HOST) now say the per-robot names are the tables' and the tests'
only and that step 0 enforces it (comments only; no table value, no code line changed). The gate's header: step 0's
paragraphs, "cases 16-27". **F4/F5 in the log:** 2cF.2's "nine robot-two checks" and 2cF.4's "16 → 17" corrected in
place (struck through, the right number beside, marked `[corrected at 2cR.5]`); the `teleop_loop.hpp` pointer re-pointed
to :78-81 in 2cF.5's table and 2cF.8's item; 2cF.8 marked superseded by 2cR.7.

A development aid, NOT a gate result: `scratch/2cfinal/run_source_cases.py` runs self-test cases 19, 26 and 27 alone (no
build) against the module — 70 checks, 0 failures — and afterwards the tree differs from `manifest-final.txt` in exactly
the five files edited (`ci.yml`, the three `src/` headers, the gate): every plant restored. Next: the gates, then the
mutations.

### 2cR.3 The first full runs on the fixed tree (23:21-23:28 EDT)

| Run | Command | Result |
|---|---|---|
| briefing, host build, suite (23:21:48-23:22:25) | `briefing_status.py generate`; `touch test/*.cpp && cmake --build build/test -j8` | exit 0, `[100%] Linking CXX executable shulib_tests`, every doc gate green |
| host suite | `./build/test/shulib_tests` | **1543 cases / 2,394,206 assertions, 3 skipped — SUCCESS** (unchanged: this pass changed comments in `src/` and the gate, no test and no code a test reads) |
| gate `check` (23:22:25-23:23:26) | `python3 tools/src_build_gate.py check` | **PASS**; `[one-chain] … (src/, include/shulib/, test/ searched, CMake included), by ONE chain setting program, robot and beacon in every branch; no allowlisted file; no selection name a macro; 7 per-robot names named only in the tables, test/ and static_asserts`; six builds 8/8 TUs, 0 disallowed |
| gate `self-test` (23:23:26-23:27:55) | `python3 tools/src_build_gate.py self-test` | **`src build gate self-test: OK (157 detector cases, each against a real build)`** — 119 at 2cF.7 + 38 (case 26: 20, case 27: 18) |

A sha256 manifest of `src/`, `include/`, `test/`, `tools/`, `Makefile`, `.github/` (600 files) taken at 23:29 before the
mutations (`scratch/2cfinal/manifest-review-before-mutations.txt`).

### 2cR.4 The mutations, on the fixed code (from 23:30 EDT)

The harness: `scratch/2cfinal/mutate2.py` — `mutate.py` of 2cF.6 (the same reboot-safe backup + `APPLIED.json` marker,
restore verified by sha256) plus two kinds: a plant that CREATES a file (the marker names it; the restore deletes it and
asserts it gone) and `step0` (step 0's three source checks through the gate module, reloaded). `hostfull` builds the
`shulib_tests` target WITH cmake's build-system check, so a changed `test/CMakeLists.txt` is re-configured (the `/fast`
target would skip that and build the OLD configuration). Every record is in `scratch/2cfinal/mutations-review.jsonl`.

**Batch A — the review's evidence, and the step-0 mutations of 2cF.6 re-run on the changed step 0 (23:30:26-23:36:03).**

| # | Mutation (final code) | Package (fresh `bin/`) | Step 0 |
|---|---|---|---|
| R1 | the review's: `src/bench_r3a.cpp` :740 `ProsImu imu{kRobotTwoChassis.imuPort` | **bench PASSES** — the finding's claim, measured | **FAILS**: `src/bench_r3a.cpp:740 names kRobotTwoChassis, a per-robot table — a program reads the BUILD's tables …` |
| R2 | the review's: :158 `tableSideOf(kRobotTwoChassis, port)` | bench PASSES | **FAILS** at `:158`, the same message |
| R3 | the mirror in a tank build: `src/drive_program.cpp` :355 `tableSignedPort(kBenchBotChassis, p)` (on robot two, shulib Drive would hand `ProsMotor` the bench bot's unsigned entries — every measured `-` dropped on 11-18, and 0 for 19/20) | tank-drive PASSES | **FAILS** at `:355` naming `kBenchBotChassis` |
| R4 | by the selection function: `src/calibrate_program.cpp` :1129 `dropDeclaredMechanismPorts(mechanismTableFor(RobotId::BenchBot), …)` (robot two's claw pivots become drive candidates) | tank-calibrate PASSES | **FAILS** at `:1129` naming `mechanismTableFor` |
| R5 | F3(b), the review's: `#define kRobotId RobotId::BenchBot` after `chassis_table.hpp`'s include | **tank AND tank-library PASS** — every static_assert, both beacons, step 8: the finding's claim, measured | **FAILS**: `src/chassis_table.hpp:51 makes kRobotId a macro (#define) — a selection name is never a macro …` |
| R6 | F3(a): a new `src/mutation_probe.c++` reading `SHULIB_ROBOT_TANK_2026` (make compiles it: 9/9 TUs) | tank PASSES | **FAILS**: `src/mutation_probe.c++:1 names a build switch — only src/build_selection.hpp may read them` |
| R7 | F3(d): `test/CMakeLists.txt` gives `drivetrain_description_test.cpp` ALONE the tank switch (`COMPILE_DEFINITIONS`) — that TU's `kProgram`/`kRobotId`/`kVariantBeacon`/`kChassis` differ from every other TU's | **host suite GREEN** (re-configured, fresh link: 1543 / 0 failed) — the ODR split is silent, and `build_selection_test.cpp`'s static_assert (another TU) cannot see it | **FAILS**: `test/CMakeLists.txt:271 names a build switch in the host build — only src/build_selection.hpp may read them` |
| MP1 | (2cF.6) `chassis_table.hpp` selects on its own `#if` again | `check` exit 1, six builds pass | **FAILS** only there: `src/chassis_table.hpp:218 names a build switch …` |
| MP2 | (2cF.6) `mechanism_table.hpp` on its own `#ifdef` again | `check` exit 1, six builds pass | **FAILS** only there: `src/mechanism_table.hpp:326 …` |
| MS1 | (2cF.6) the robot split into a second chain | `check` exit 1 | **FAILS**: the second chain at `:135` and six branches `sets kRobotId 0 times` |
| MC1 | (2cF.6) THE RULING'S CASE: the tank-drive branch sets `RobotId::BenchBot` | `check` exit 1: tank-drive's robot beacon absent and the bench bot's present | chain table: `build tank-drive's branch (src/build_selection.hpp:119) sets kRobotId = RobotId::BenchBot, but … RobotTwo` |

R1-R7 are each a defect that every PACKAGE check passes — measured, not assumed — so step 0 is the one detector for them;
MP1/MP2/MS1/MC1 show the rewritten step 0 still catches what 2cF.6 proved it caught.

**R5, inspected (00:12 EDT, R5s; and an unmutated control).** What the tank-library package carries, from its bytes
(`bin/hot.package.elf`, fresh build each):

| Build | Gate verdict (steps 1-8) | robot two's chassis table (`2026 TANK CHASSIS (robot two)`) | the bench bot's (`TANK BENCH BOT (measured)`) | robot beacon |
|---|---|---|---|---|
| tank-library, unmutated (control; tank-calibrate the same) | ok | PRESENT | absent | robot two's |
| tank-library under `#define kRobotId RobotId::BenchBot` (R5s) | **ok** | **absent** | **PRESENT** | **robot two's** — derived before the macro |

So the review's F3(b) claim is measured: robot two's "shulib Teleop" would boot over the bench bot's chassis table
under robot two's robot beacon, and every package check passes; step 0's macro rule is the one detector (R5). (The claw
names are not linked in this tree's packages — T3 Part 2 wires them — so the "no claws" half is by the traced
`static_assert`/selection reading, not by a string.)

**Batch B — the gate's new detectors, each a FULL `self-test` against the mutated gate (23:36:03-00:11:32).**

| # | Gate mutation | Self-test |
|---|---|---|
| G1 | the per-robot-name rule disabled | **RED, exit 1** — case 26: every derived name planted in a program unreported, the review's three plants, the library header, the name after a static_assert, the new robot's table, the returned table |
| G2 | the static_assert exemption removed | **RED** — case 26: `a static_assert, a comment or a string was reported: ["src/bench_r3a.cpp:3266 names kRobotTwoMechanisms …` (T3 Part 2's form would fail the merged tree) |
| G3 | the macro rule disabled | **RED** — case 27: `#define kRobotId` in each table, `#undef kChassis`, `#define kProgram` in the header, `#define kMechanisms` in a test unreported (the `main.cpp` plant still drew the per-robot-name message, but not the macro one) |
| G4 | `ONE_CHAIN_EXTS` back to 2cF's typed tuple | **RED** — case 27: the `.c++` and the `.S` probes unreported |
| G5 | `DECLARES_CHAIN_CONSTANT` without the brace form | **RED** — case 27: `kRobotId{…}` and `kProgram{…}` in a table and in the header unreported |
| G6 | the host build's CMake not read | **RED** — case 27: both CMake plants unreported |
| G7 | the per-robot names not derived (an empty set, no problem) | **RED** — case 26: `the derived per-robot names miss one: []`, every plant unreported, `an unrecognised selection function did not FAIL step 0` |
| G8 | the selection function's returns not derived (objects only) | **RED** — case 26: `a table the selection function returns, named in a program, was not reported` |

**Every mutation of this pass was killed: 19 records + R5s (an inspection, no verdict expected), every restore verified
byte-exact by sha256 (R6's probe file removed and asserted gone), no `APPLIED.json` left, and at 00:12:54 the whole tree
(600 files, the same set) equals `manifest-review-before-mutations.txt`.**

### 2cR.5 Each finding's disposition

| # | Finding | Verdict | Disposition, with its evidence |
|---|---|---|---|
| F1 | major — a program can name another robot's table, and nothing notices | **REAL — FIXED at the root, in the gate** | Step 0's new rule "A ROBOT'S TABLES BY NAME" (2cR.2): the per-robot names, DERIVED from the table files (fail-closed), may appear in code only in the tables, `test/` and a `static_assert`'s parentheses. The review's two mutations and two more (R1-R4) each pass every package check and FAIL step 0; the gate's own detector proven load-bearing by G1 (disabled), G2 (the static_assert exemption removed), G7 (the names no longer derived) and G8 (the selection function's returns no longer read) — each turns the self-test red. T3 Part 2's three static_asserts pass it on a simulated merge. The alternative (a nested namespace) enforces nothing and would break T3 Part 2's code at the merge — rejected with that measured reason. |
| F2 | minor — two now-false sentences the logged prose review missed | **REAL — FIXED** | `ci.yml` :118-120 (the policy: no category allowed; the self-test) and the gate's BUILD_PROGRAM comment (`src/build_selection.hpp`'s chain). This pass's prose-review grep (2cR.6) re-ran 2cF.5's patterns plus the new ones and read every hit. |
| F3 | minor — step 0 gaps (`.c++`, a macro renaming `kRobotId`, the untested brace form, the CMake route) | **REAL, all four — FIXED** | (a) `ONE_CHAIN_EXTS` generated from `ALL_EXTS` (R6; G4 red). (b) no selection name may be a macro, in any file (R5 — the tank builds PASS every package check with the macro, measured; R5s shows what they carry; G3 red). (c) the `{…}` and `(…)` forms planted, in a table and in the header (G5 red). (d) step 0 reads the host build's CMake files (R7 — the host suite stays GREEN with the ODR split, measured; G6 red); the old :309-310 comment says "GLOBAL flags" and why CMake is read. |
| F4 | minor — two counts wrong, one pointer stale | **REAL — CORRECTED in the log** | 2cF.2 "nine" → EIGHT robot-two checks; 2cF.4 "16 → 17" → 12 → 13 declarations (both struck through in place, marked); the `teleop_loop.hpp` pointer → :78-81 in 2cF.5's table and 2cF.8's item. Every other pointer in 2cF.8 re-checked against this tree (2cR.7 carries the checked list). The conclusion the numbers supported (every static_assert kept) was and is true. |
| F5 | minor — the merge-collision list was incomplete | **REAL — COMPLETED by a trial merge** | 2cR.7 item 2: every file both runs change, with a `git merge-file` trial merge of each (base `cc513a7`, ours = this tree incl. 2c, theirs = T3 Part 2's `1a86103`), done in scratch — four textual conflicts, each with its resolution; seven files merge clean. |

### 2cR.6 The prose review for this pass — the greps run BEFORE this entry was written (23:17-23:20 EDT)

Files changed by this pass: `tools/src_build_gate.py`, `.github/workflows/ci.yml`, `src/build_selection.hpp`,
`src/chassis_table.hpp`, `src/mechanism_table.hpp` (comments only in the three `src/` headers), this log. The concepts:
the per-robot names and the rule over them, the macro rule, the extensions step 0 reads, the host CMake, the warning
policy's CI description, where the chain lives.

```sh
LIVING="README.md docs/*.md docs/guide docs/cookbook docs/internal/{RESUMING,build-order,HANDOFF-remaining-work,PROJECT-BRIEFING,ORIENTATION,RELEASING,guide-maintenance,docs-publishing}.md docs/internal/chunks/R3a-BENCH-WORKSHEET.md"
P='main\.cpp.s (one |#if )?chain|residual hole|stated residual|only name a program|the only name|per-robot (name|table)|kRobotTwoChassis|kBenchBotChassis|kNoChassis|kRobotTwoMechanisms|kNoMechanisms|chassisTableFor|mechanismTableFor|CXXFLAGS_EXTRA|\.c\+\+|COMPILE_DEFINITIONS|test/CMakeLists|a macro|#define kRobot'
grep -rnE "$P" $LIVING                                   # every hit read
grep -rnE "$P" include src test tools Makefile common.mk .github   # every hit read (test/vendor excluded)
grep -rnE "step 0|one-chain|chain table|ONE_CHAIN|residual|src build gate|src_build_gate" $LIVING Makefile common.mk .github
grep -nE "main\.cpp.s (one |#if )?chain|main\.cpp's|residual|three source checks|cases 16-2|ONE_CHAIN_EXTS|kRobot\b" tools/src_build_gate.py
```

| Hit | Finding | Disposition |
|---|---|---|
| `.github/workflows/ci.yml` :118-119 | "the stated residual hole" (F2) | FIXED (2cR.2) |
| `tools/src_build_gate.py` BUILD_PROGRAM comment | "src/main.cpp's chain" (F2) | FIXED |
| `tools/src_build_gate.py` header "step 0, three source checks" | the new rule is a fourth paragraph but part of the first check (it reports under `[one-chain]`) | the paragraph says so in its first line |
| `src/chassis_table.hpp` :11, :215; `src/mechanism_table.hpp` :85-86, :323; `src/build_selection.hpp` :43-44, :50-52 (as found; edited at :13-16, :215-217, :85-88, :323-325, :43-49, :57-58) | "the only name a program uses" was a convention; now enforced | each now names step 0 (2cR.2) |
| `README.md` :289, `docs/guide/07-getting-set-up.md` :130 | "the per-robot tables (every robot's ports; robot two's claws)" | true; not changed (they describe the files, not how a program reads them) |
| `docs/hardware-assumptions.md` :1670, :1865 | cite `kRobotTwoMechanisms` as HA-153's / HA-159's source | true; not changed |
| `docs/roadmap.md` :1265, `docs/internal/build-order.md` :691 | GATE1's record ("the residual hole stated in the tool"), dated by 2cF.5 | true of its day, already dated; not changed |
| `docs/internal/RESUMING.md` :136, `build-order.md` :698 | the `CXXFLAGS_EXTRA` history | true; not changed |
| `include/shulib/diag/build_info.hpp` :11, `docs/internal/PROJECT-BRIEFING.md` :563 | `test/CMakeLists.txt` injects `SHULIB_BUILD_HASH` | true (not a build switch: `READS_SWITCH` matches `SHULIB_ROBOT_*`/`SHULIB_PROGRAM_*` only, and step 0 reads the file clean) |
| `test/*_test.cpp` (chassis, mechanism, build selection) | name the per-robot tables | legal: `test/` is exempt by design, and their headers say why |
| `build-order.md` :1399-1450 "a macro" | a CONTROLLER macro (a button sequence) | unrelated |
| `src/teleop_program.hpp` :30-31, `src/teleop_program.cpp` :42-43, :61, `src/boot_source.cpp` :12-16, `include/shulib/teleop/teleop_loop.hpp` :78-81 | 2cF.8's items — still stale, still other runs' files | NOT TOUCHED; carried into 2cR.7 with T3 Part 2's line numbers too |

No public document describes step 0's rules (they live in the gate's header, which RESUMING, the `Makefile` and CI
point to), so none needed a change for F1/F3.

### 2cR.7 For the COORDINATOR at the final merge — supersedes 2cF.8's list (every pointer re-checked against this tree, 23:40 EDT; T3 Part 2's line numbers from its transport `1a86103`, read-only)

None of these files was touched by this run.

1. **False since this run (or since 2c), in files T3 Part 2 owns** — fix at the merge, in T3 Part 2's version of each file
   (the merged file is its):
   - `src/teleop_program.hpp` (here :30-31; T3P2 :35) and `src/teleop_program.cpp` (here :42-43; T3P2 :56-57): "the BEACON
     stays a literal in main.cpp's #if chain" → "…in src/build_selection.hpp's one chain".
   - `src/boot_source.cpp` (here and T3P2 :14-16): "In the bench and xdrive builds src/chassis_table.hpp's kChassis is the
     bench bot's table" → the bench build carries the bench bot's table, the xdrive build the UNSET `kNoChassis`; still
     harmless (no program in either asks for a boot source).
   - `src/boot_source.cpp` (here and T3P2 :12-13) and `src/teleop_program.cpp` (here :61-62; T3P2 :75-76): "nothing here reads
     src/main.cpp's SHULIB_RUNS_* macros" → "nothing here reads a build switch (only src/build_selection.hpp may; the src
     build gate's step 0 fails one named here)".
   - `include/shulib/teleop/teleop_loop.hpp` (here :78-81; T3P2 :92-95): "Since G1 Part 2b "shulib Teleop" … drives
     through it" — the X-drive (`src/xdrive_program.cpp`, since 2c) is the second caller.
   - `docs/internal/HANDOFF-remaining-work.md` :277 — FALSE: "`src/main.cpp` only chooses which program a build is" →
     `src/build_selection.hpp` chooses the program AND the robot, `main.cpp` hands it the entry points; :300-303 — true but
     incomplete: the gate also asserts a per-ROBOT beacon, reads the one chain from `src/build_selection.hpp`, fails a
     switch named anywhere else (`src/`, `include/shulib/`, `test/`, every extension make compiles, the host CMake), a
     selection name made a macro, and a program naming another robot's table; and it fails `-Wunused-function`. (:311 is
     history in the paragraph HANDOFF keeps "as it was written before the go"; no change.)
2. **Merge collisions — a TRIAL MERGE, not a guess** (`scratch/2cfinal/trial_merge.sh`: for every file both branches change,
   `git merge-file` of base `cc513a7`, ours = this worktree's tree (2c's `80e2615` + this run), theirs = T3 Part 2's
   `1a86103`; blobs by `git show`, merged in `scratch/2cfinal/trial-merge/`, neither worktree written). Eleven files are
   changed by both:
   - **Merge clean (7):** `src/mechanism_table.hpp` (T3P2's hunks at base :97-102 "WHO READS IT", :141-152
     `kTeleopRezeroButton`'s doc, :239 and :246 the entry doc, :471-479 `pneumaticStart/SafeCommand`'s docs — beside this
     branch's :6-8, :78-86, :115-121, :132, :230, :298-304, :555, :567-578; none overlaps), `test/mechanism_table_test.cpp`
     (T3P2 :834, a comment; this branch's header :10-17, `robotTwoChassis()` :84-104, :373-376, :393, :534),
     `test/boot_source_test.cpp` (T3P2 +377 incl. :86, :336-545 one-liners, :592-597, :820; this branch :161-163),
     `docs/hardware-assumptions.md`, `docs/internal/build-order.md`, `docs/roadmap.md`, the worksheet. **Semantically
     checked, not only textually:** step 0 over the simulated merged `src/`/`include/shulib/`/`test/` (2cR.2) reports
     nothing — T3 Part 2's new `src/teleop_claws.hpp` and its three `kRobotTwoMechanisms` static_asserts are legal.
   - **Conflict (4), each with its resolution:** `README.md` (~:281-285, the `src/` line of the tree layout): take THIS
     branch's (build_selection.hpp; main.cpp a dispatcher; the X-drive a `RobotConfig`) and add T3P2's "and its claw wiring"
     (`src/teleop_claws.hpp`). `docs/guide/07-getting-set-up.md` (~:122-128): T3P2's side still says `src/main.cpp`
     "wires the invented X-drive by hand" — FALSE since 2c; take this branch's paragraph and add T3P2's "Teleop's claw
     wiring (`src/teleop_claws.hpp`)". `docs/internal/HANDOFF-remaining-work.md` (~:601): 2c's WIP sentence vs T3P2's status
     sentence — the coordinator's status text, written at the merge. `docs/internal/PROJECT-BRIEFING.md` :113: generated —
     take either and REGENERATE after the merged suite binary exists.
   - Any `#if` on a build switch, `#define` of a selection name, or per-robot table named outside a static_assert that
     either branch adds anywhere under `src/`, `include/shulib/` or `test/` FAILS step 0 on the merged tree; run `python3
     tools/src_build_gate.py check` on it before the commit.
3. **The COMP brief §3 item 3** (`docs/internal/chunks/COMP-competition-program.md` :287-292, which T3P2 also edits
   elsewhere): 2c's proposed amendment is stale. Proposed replacement (a plan change: proposed, not made): *"COMP-b adds the
   seventh program by: a `Program::Competition` enumerator; ONE branch of `src/build_selection.hpp`'s chain (`#elif
   defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_COMPETITION)` setting `kProgram = Program::Competition`,
   `kRobotId = RobotId::RobotTwo` and `kVariantBeacon = "shulib-robot-variant=tank-competition"`, nothing else) plus its
   `#error` guards; a case in EACH of `src/main.cpp`'s five entry-point switches; in `tools/src_build_gate.py`,
   `VARIANTS`, `BUILD_ARGS`, `VARIANT_DEFINES`, `VARIANT_BEACONS`, `BUILD_PROGRAM`, `BUILD_ROBOT` (`RobotTwo` — no new
   `ROBOT_BEACONS` entry) and `PROGRAM_ENTRY_SYMBOLS`, and a self-test pair like cases 14/15. The define is read in
   `src/build_selection.hpp` ONLY, and the program reads robot two's tables as `kChassis`/`kMechanisms` ONLY (a per-robot
   name outside a static_assert fails step 0)."*
4. **G3a (the robot profile and its generator, next after this lands) — a constraint this run adds:** a generated table
   must keep the shapes step 0 derives the per-robot names from — each table a `constexpr ChassisTable`/`MechanismTable
   <name>` in `src/chassis_table.hpp`/`src/mechanism_table.hpp`, ONE selection function per file
   (`constexpr const …Table& <name>(RobotId …) {` returning each robot's table by name), programs reading `kChassis`/
   `kMechanisms` only, and no switch outside `src/build_selection.hpp`. A table file step 0 cannot derive FAILS
   (fail-closed); the G3a brief should say so.
5. **Status and records (the coordinator's):** `build-order.md`'s first `**Next:**` and `roadmap.md`'s "Now"/"You are
   here" — G1 Part 2c landed WITH the team lead's two rulings and this review; **`G1-COMPLETED.md`** — this log's Part 2c
   final summary (the rulings, 2cF.8's six deviations, the review's five findings and their fixes, the residuals, both
   mutation tables, the gates) beside 2c's; regenerate the briefing after the merged suite binary exists, and `docs/api/`
   after merging T3 Part 2's headers (this run changed no `include/` header).
6. **The G1 Part 2 brief §2c item 4** (`G1-part2-pros-provider-and-main.md` :424) still says "`-Wunused-function` stays
   allowed by category" — annotate it or leave it with 2cF.8's deviation 1 as the record (the coordinator's call).
7. **After the commit (landmine 9):** rebuild the six packages from the COMMITTED tree; at the first boot of each program the
   first line `initialize()` prints is `[BLD] robot tables: …` (before the program's banner) — the worksheet could add it
   as an upload-identity check (optional; T3 Part 2 edits the worksheet; nothing in it is false).
8. **Nothing has run on a robot.** No package was uploaded; `pros` never run; `/dev/ttyACM*` never touched.

### 2cR.8 Every gate, on the FINAL tree (00:14:57-00:21:35 EDT, 2026-09-27)

After the last code edit (the gate, 23:20) and the last restore (00:12:45); the tree equals
`manifest-review-before-mutations.txt` before and after these runs. Driver: `scratch/2cfinal/final_gates.sh`; log
`scratch/2cfinal/review-final.log`.

| Gate | Command | Final line |
|---|---|---|
| Briefing, before the build | `briefing_status.py generate` | `briefing status: regenerated` |
| Host build | `touch test/*.cpp && cmake --build build/test -j8` (every doc gate inside) | exit 0, `Linking CXX executable shulib_tests` present, `[100%] Built target shulib_tests` |
| Host suite | `./build/test/shulib_tests` | **`[doctest] test cases: 1543 \| 1543 passed \| 0 failed \| 3 skipped`; `assertions: 2394206 \| 2394206 passed`; `Status: SUCCESS!`** |
| Briefing, after the suite binary; the build again; check | `generate`; `cmake --build build/test -j8`; `briefing_status.py check` | `regenerated`; exit 0; `briefing status: generated block current` |
| API docs | `api_doc_tool.py self-test`, `check-coverage`, `check-fresh`, `check-examples`, `check-removability` | `api_doc_tool self-test: OK`; `doc coverage: 2669 public entities across 149 headers, all documented`; `check-fresh` exit 0 (no `include/` header changed — no regeneration); `doc example scan: 437 quoted lines, all verbatim (4 source files)`; `removability: no public doc references docs/internal/` |
| Doc staleness | `doc_staleness_audit.py self-test`; the audit | `self-test: OK (10 detector/filter cases)`; `doc staleness audit: clean (27 live docs, 0 numeric claims checked)` |
| CI greps | CI's two guards | `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core is sim-free` |
| ARM header gate | CI's command over every `include/shulib` header | `TU includes 185 headers`; `ARM compile gate: CLEAN (all v2 headers)` |
| src build gate self-test | `python3 tools/src_build_gate.py self-test` (00:16:09-00:20:36) | **`src build gate self-test: OK (157 detector cases, each against a real build)`** |
| src build gate | `python3 tools/src_build_gate.py check` (00:20:36-00:21:35) | **`src build gate: PASS (all 6 builds link; policy: errors fail, vendor warnings path-ignored, no category allowed, every other warning fails; defines token-exact; one chain picks program and robot; variant and robot beacons asserted; programs asserted)`** — `[one-chain] … no selection name a macro; 7 per-robot names named only in the tables, test/ and static_asserts`; six builds 8/8 TUs, 0 disallowed |

`bin/` and `.d/` removed at the close (gitignored); `build/test` kept (the host build). Nothing committed, staged or
pushed; `pros` never run; `/dev/ttyACM*` never touched; the main tree and every other worktree untouched (T3 Part 2's read
only through `git --no-optional-locks show/diff`, which writes nothing). Scratch evidence (both harnesses,
`mutations-review.jsonl`, every run log, the trial merge, the simulated merged tree, both manifests) is under
`.claude/coordinator/scratch/2cfinal/`.

---

## The coordinator's merge — G1 Part 2c final (2026-09-27, ~00:15–00:45)

Squash-merged from `63b8cd2` onto `f6274e1`; the record, the conflicts, the merge list's items and the
verification (suite 1659/1659; the gate's 157-case self-test and six-build check; two coordinator
mutations, one of them the review's surviving edit now failing step 0) are in
[`G1-COMPLETED.md`](G1-COMPLETED.md). **G1 is complete.** Nothing of 2b's or 2c's programs has run on a
robot.
