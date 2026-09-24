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
