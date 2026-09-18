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
