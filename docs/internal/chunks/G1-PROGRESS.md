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
