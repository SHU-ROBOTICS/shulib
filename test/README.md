# shulib host tests

Off-robot unit tests for the **pure C++20 core** (`math/`, `kinematics/`, `control/`, … — the layers
that contain **zero `pros/` includes**). Running the logic on a laptop, in seconds, is what lets us
catch bugs before they ever reach the V5. This build is **completely separate** from the PROS ARM
Makefile at the repo root.

## Build & run

```sh
cmake -S test -B build/test        # configure (once, or after adding files)
cmake --build build/test           # compile
./build/test/shulib_tests          # run all tests
# or:
ctest --test-dir build/test --output-on-failure
```

`build/` is gitignored, so this never pollutes the repo. A non-zero exit code means a test failed.

**The compile-refusal harness** (since the mechanism core's first part): the mechanism core refuses a
robot file's mistakes AT BUILD, each by name. Every named refusal a robot file can reach has a
translation unit in [`compile_refusals/`](compile_refusals/) that must FAIL to compile, naming its
reason. Two names have none, on purpose: `a_smart_port_is_used_twice` and
`a_three_wire_port_is_used_twice`, the "used twice" fallbacks no list can reach (each said so beside it
in `mechanism/wiring.hpp`). And every PLACE in the headers that calls a refusal is reached by a TU, not
only every name (checked by deleting each call in turn and seeing a TU go red — at the mechanism core's
first part, 148 of the 150 direct calls in the robot file's words, the other two being those fallbacks,
and all 11 calls made through a dispatcher, such as `refuse(specRefusal(...))` or the route-id
recorder; at its fourth part's review, 2026-10-27, all 36 of the driver file's and the auton's: the 31
direct calls in `mechanism/driver_inputs.hpp`, its one dispatcher call — the button-naming one — and
the `Routine`'s four refusal templates). Where two layers could both refuse with the same
name — a `wire(...)` check and `mechanisms()`' own re-check of each entry — the TU also pins the line
GCC must cite (`// expect-at:`). They are not part of `shulib_tests` — `ctest` builds each one on its
own, one at a time even under `ctest -j` (they share a lock on the build tree), which takes two to four
minutes for all of them (127 s and 153 s, measured 2026-10-15 at 195 TUs; 197 s at 251 TUs, 2026-10-27), and
`ctest --test-dir build/test -R compile_refusal --no-tests=error --output-on-failure` runs just those.
The TU's own `// expect:` line states its reason; [`compile_refusals/run_refusal.py`](compile_refusals/run_refusal.py)
is the judge (the reason must be in a diagnostic's message, never only in a path), and four
`harness_*` TUs prove the judge cannot be fooled (a TU that compiles, the wrong reason, the reason only
in a file name, the wrong line). A TU dropped into that directory is a test at the next configure, and
CI fails when the number of harness tests is not the number of TUs (a detached harness would otherwise
report "No tests were found" and pass). The suite and the harness compile with the same flags
(`shulib_test_flags`, in `CMakeLists.txt`). The spec's per-field CONDITIONS (both sides of a range,
each field of a pair) are pinned at run time, one property per case, by `mechanism_spec_test.cpp`.

**The build needs `python3`.** Before compiling anything it runs the documentation gates
(`tools/api_doc_tool.py`): a public entity anywhere under `include/shulib/` with no
documentation comment fails the build *naming that entity, its file and its line*; the generated reference in
`docs/api/` must match a fresh generation; every ```` ```cpp ```` line in the public
documentation must appear verbatim in a compiled example test (or in a team file a chapter quotes
— robot two's driver file, `src/team/robot_two_driver.hpp`, each listing of which must also be an
unbroken run of the file); and no public document may link into the development-process notes.
The library itself is header-only C++ and depends on none of
this — only the test build does, and deliberately: a gate you can forget to run is not a gate.

## The route builder's tests (since RMP1-A1)

The route builder (`tools/route_maker/`, a web page) has two halves of tests, and this suite holds the C++ one:

- **`route_maker_golden_test.cpp`** — every golden in [`goldens/route_maker/`](goldens/route_maker/) is a route
  (`<name>.route.json`, a project file) and the C++ the page's OWN exporter wrote for it (`<name>.cpp`, kept fresh
  by the page's node test) — except `gl0_empty.cpp`, the shared file's empty region as the contract with robot two's
  update writes it, kept by hand so the exporter is held to it rather than re-making it. Each export is `#include`d here inside a namespace that resolves its names the way the
  shared skills-route file does, compiled with the suite's flags, and RUN through `competition::runAuton` at its
  FIELD start on the host test plant (`route_maker_rig.hpp`). Behaviour is asserted only on robot x
  (`mechanism_fixtures.hpp`, numbers the test owns: a drive turned first and one not, reverse and turns, curves of 2
  and 16 points, every mechanism step, stop / carry on / skip, a wait across the 57 s cut, the home rule's twins,
  the caps' spellings and both tolerances); robots one and two are smoke runs. A tripwire holds that a robot
  building the shared route and declaring a cascade has its levels marked as its facts ledger's (robot two, once its
  update gives it robot one's cascade). It also pins the committed fixture descriptions
  (`goldens/route_maker/robots/{x,y,z}.json`) and CV1's curve table (`goldens/route_maker/curves.txt`) to what the
  C++ makes now — the page's node tests read them.
- **`shulib_route_maker`** (`route_maker_main.cpp`, its own CMake target, not part of `shulib_tests`) —
  `describe --robot one|two|x|y|z` writes a robot's description from its REAL table (`route_maker_describe.hpp`;
  the declaration's C++ name read from the line its `.name` recorded) and `curves` re-makes CV1's table. After a
  change that moves either, re-make the committed file and READ the diff:
  `build/test/shulib_route_maker describe --robot x > test/goldens/route_maker/robots/x.json` (and y, z), and
  `build/test/shulib_route_maker curves > test/goldens/route_maker/curves.txt`.

The page's own tests are node's: `node --test tools/route_maker/test/*.test.mjs` (the model — no browser), and, with a
browser, `ROUTE_MAKER_BROWSER=firefox node --test tools/route_maker/test/page/*.test.mjs` (see `tools/route_maker/README.md`).

## Conventions

- **One test file per unit:** every `*_test.cpp` in this directory is auto-discovered and compiled in.
- **`test_main.cpp`** provides the test runner's `main()` exactly once — leave it empty of tests.
- Include the unit under test from `include/shulib/...` and assert against it.

## The host field plant (the wall reset's oracle)

`include/shulib/sim/field_plant.hpp` (chunk L6-A1, 2026-10-29) is the host test plant's FIELD and its
SIDE DISTANCE SENSORS — test infrastructure, outside the generated API reference like the rest of
`sim/`. `sim::FieldModel` holds axis-aligned boxes with heights (opaque, or see-through for a clear
wall panel) and the floor; `FieldModel::override2026()` builds the season's field — the four walls
11.5 in tall at ±70.2047 in, each Toggle's body on top with its two brackets hanging to 10.6 in, the
Loaders on the red and blue walls, A12's start pieces against every wall — from the plant's OWN copy
of the drawing numbers, and `add` / `removeNamed` / `seeThroughWall` change it (a Pin, a robot, an object
behind a panel). `sim::SideSensorPlant` is one side sensor with a TRUE mount (forward, left, height,
facing — set separately from the mount the test hands the library, so a case can type it wrong): a
cone of rays (±18° by default, a 7 × 7 grid inside the circle; `raysPerAxis = 1` is the centre ray
alone) returning the NEAREST hit, or the hits' mean in `blend` mode; the GPS code strip's band
(9.06–11.44 in) with a per-colour bias and confidence scale; a bias, seeded noise, an update hold, a
latency delay line, drop-outs, a FROZEN mode and read faults (what `ProsDistance` does on PROS_ERR:
the last good distance, confidence 0, the count moving). Each `step(truePose)` publishes into a
`hal::fake::FakeDistance` the library reads; `rangeAt(truePose)` measures without publishing.

**The rule it keeps:** it includes nothing from `localization/` — the wall reset's tests grade the
library's geometry against this plant's own ray cast, so a sign error cannot hide in both
(`sim_field_plant_test.cpp` asserts the two copies of the field agree to 0.01 in). The tank rig that
puts it under a `Chassis` is `wall_reset_rig.hpp`; it takes an optional `sim::DegradationModel*` (e.g.
`sim::ImuHostileModel`) for an IMU that calibrates, drifts or jitters — the wall reset's NOT LIVE case
and its still-robot noise case run over one. The wall reset's log line is pinned byte for byte in
`wall_reset_line_test.cpp`, and guide 11 quotes those strings.

Since L6-A2 (2026-11-03) `wall_robot_rig.hpp` puts the same field under a whole `robot::Robot` built from a
robot file that DECLARES its side sensors (robot two's shape, sensors on ports 4 and 5), over the fake
device provider: the provider serves its own `FakeDistance` per port and has no bind hook for one, so
the rig's pacer copies each side plant's published reading — and its read-fault state — into the
provider's fakes after every plant step, and the Chassis's pacer is a `RunGuard`, so
`competition::runAuton` runs an auton on it and the everyday wall reset samples through the guard's
wait. `competition_wall_reset_test.cpp` builds its context by hand instead, so each side's sensor can
be a COUNTING wrapper ("nothing was read" is a number).

## Strict by design

The build uses `-Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion
-Wdouble-promotion`. **Warnings are errors** — a narrowing conversion or an implicit double→float is a
bug we want the compiler to catch, not us. doctest is included as a *system* header so the framework's
internals never trip these flags; only our code is held to them.

## How we write tests

Tests exist to **find bugs, not confirm the obvious** (see the
[Testing discipline](../docs/roadmap.md) in the roadmap). Each test targets a specific way the
logic could be wrong (edge case, boundary, sign flip, wrap-around, NaN); we prefer **invariants/
properties** swept across the input space over hand-picked points; and for load-bearing code we run a
**mutation check** — deliberately break the implementation and confirm a test goes red. A suite that
stays green while the code is wrong is theater.

Trivial checks (`1+1==2`) are only ever *harness self-checks* — proof the runner itself works (and can
fail) — and are retired once real tests exist.

## The camera's test doubles (since VIS1-A1)

Three pieces stand in for the V5 AI Vision camera on the host. Like every fake and the host test plant they
are outside the API reference (`tools/api_doc_tool.py` leaves `hal/fake/` and `sim/` out), so they are
documented here and in their own headers:

- **`hal::fake::FakeVisionFrames`** (`include/shulib/hal/fake/fake_vision_frames.hpp`) — a settable camera:
  `setNext(frame)` is what every `read()` returns until changed, `setFailing(true)` makes every read a
  `ReadFault` with NO detections, `setStatus(...)` is what `status()` answers (the Robot's boot line is
  tested through it in every state). It is its own `hal::IReadFaults`, like the adapter, so its count moves
  WITH the fault frames it returns; `FakeDeviceProvider::readFaults(port)` answers it only once
  `aiVision()` has served it. `FakeDeviceProvider::fakeCamera(port)` hands a test the provider's own.
- **The shim's `pros/ai_vision.h`** (`test/pros_shim/pros/`) — a programmable camera per port, for
  `hal::pros::ProsAiVision`. Where the vendored PROS header and its `libpros.a` disagree it answers as the
  BINARY does (a class name read returns 1 for every id and writes nothing for an id the camera does not
  name; a failed object read is `{127, 127, 0...}`; a failed colour read 127s and +Inf — `colourReadFails`
  makes the read-back fail while the write still lands; `codeObject()` makes a colour CODE's record). It
  starts ADVERSARIAL — tags and objects on (what Bench Tests station 12 leaves), a stray colour in slot 1, and
  VEX's Classroom model (`setOverrideModel(port)` for the right one) — and counts every call the adapter
  must never make. `pros::shim::resetAll()` resets it. It tests the adapter against our BELIEF about PROS;
  it cannot test the belief.
- **`sim::CameraPlant`** (`include/shulib/sim/camera_plant.hpp`), the host camera model — Toggle faces
  (seated or rolled), Pins and Cups (standing, lying, raised), plates and strips, seen through a TRUE
  camera a test sets apart from the believed mount, with jitter, drops and a frozen, unplugged or torn
  camera. It projects with its OWN code and includes nothing from `vision/`: a sign error in the library's
  geometry cannot cancel out in a test that uses both (`test/sim_camera_plant_test.cpp` holds its own
  anchors). Its banner lists what it does not model.
- **`test/camera_scene_rig.hpp`** (VIS1-A2) — the camera's meaning tests share it: the mount the library
  believes and the TRUE camera the host camera model looks through (moved inside the stated tolerances),
  the tracker's and the finder's numbers the brief's probes used (`tcfg()`, `fcfg()` — the team's own
  come from the bench), a seated Toggle on the +x wall, a robot parked square to it, Pins, and hand-made
  frames. Test-only (no `TEST_CASE` in it); its numbers are never a library default.
- **`sim::TogglePlant`** (`include/shulib/sim/toggle_plant.hpp`, since AF1S-A1), the host Toggle — three faces
  in a roll order the test picks; a stroke (a flipper's TRUE angle passing `rollDeg` outward with the robot's
  front within reach, or a passive bar's approach) rolls one face, two when over-rolling, none when sticky
  (an attached `sim::ArmPlant` is then stopped where it meets it), and leaves it stuck part-way when jammed;
  it seats after its seat time, counts contacts while its seated face is "ours", and answers the game's
  colour (`scoredColour()`: yellow while touched or unseated). It draws its TRUE faces into a
  `sim::CameraPlant` and includes nothing from `vision/`. The Toggle step's tests use it as their oracle
  (`test/toggle_visit_rig.hpp`; its own tests are `test/sim_toggle_plant_test.cpp`).
- **`test/mirrored_pair_check.hpp`** (MIRROR1-A) — THE PAIR CHECK, one function every user of it shares: a
  MIRRORED auton entry run twice through `competition::runAuton`, robot one as written and robot two mirrored
  from the reflected start, each on its own host test plant over the program's chain, compared pace by pace;
  it returns a VERDICT (never an assertion) that FAILS when the two robots' 15 in squares come within
  `kMirrorGapIn` (6 in — PROVISIONAL, A4 HA-260), naming the moment, both places and the drive motion each
  robot was in (robot one's with its target, in the route's numbers). The suite asserts it over robot two's
  registry (T9) and on planted routes (`test/mirrored_pair_test.cpp`, where a planted crash is a PASSING
  test of the check); a route preview or route maker calls the same function so the picture and the suite
  never disagree. A RELATIVE mirrored entry is skipped, by name.
- **`test/goldens/mirror/skills_mirror_vectors.txt`** (MIRROR1-A) — the skills mirror's worked vectors, one
  per line (`FIELD` and `RELATIVE`), worked by hand: `test/reflection_test.cpp` reads every line (and refuses
  a malformed one by its line number); a route maker that draws the mirror is to read the same file, so the
  two cannot disagree on the reflection.
- **`test/mirror_unchanged_runs.hpp`** (MIRROR1-A's T10) — the runner-level "nothing else changes" hash: three
  autons and a mechanism list ALL OWNED BY THE FILE (none of the team's, so a student's edit never moves it)
  run through `runAuton` on a weak-side tank and an X-drive, every record, log line and true pose folded into
  one FNV-1a hash per drivetrain, MEASURED against the merge base and pinned in
  `test/mirrored_auton_test.cpp`. Self-contained: it includes no `src/` file, not even through a rig (its
  weak side is a COPY of `auton_tester_rig.hpp`'s, because that rig includes robot two's mechanism list), so
  it compiles against the base: change it and re-measure on a clean archive of the base.
- **`test/toggle_robot_rig.hpp`** (AF1S-A2) — "the robot plugged in" for the Toggle step's EVERYDAY form: a
  REAL `robot::Robot` built from a robot file that declares the camera and its Toggle block (and a flipper arm
  and a stall-homed cascade, namespace `toggle_robot` — the cookbook recipe's robot), over the fake provider,
  run through `competition::runAuton` on the competition program's own chain (Chassis -> RunGuard ->
  MechanismService -> the world). The world steps the drive plant, the flipper's and the cascade's arm plants
  and one `sim::TogglePlant` per wall a test asks for, then renders the host camera model from the TRUE pose and
  hands that frame to the provider's own fake camera, so the Robot's feed reads exactly what the plant published.
  Its users: `test/toggle_visit_ctx_test.cpp`, `test/robot_toggle_camera_test.cpp`, cookbook-08a.

## The route preview (since RPV1-A)

`python3 tools/route_preview.py "<auton name>"`
([guide chapter 16](../docs/guide/16-testing-an-auton-on-the-bench.md#before-the-bench-previewing-a-route),
"Before the bench: previewing a route") runs
one of the team's autons through the Auton Tester's own sequence on the host test plant and draws it as one
offline page. Its program is a host target here, `shulib_route_preview` (built by the default target, outside
the suite's `*_test.cpp` glob, with both robots' autons files). `--robot one` runs robot one's list on fakes by
kind and lists every fact its ledger (`src/robots/robot_one_facts.hpp`) still OWES as a PLACEHOLDER — read from
the ledger, never copied. ALL its logic is in headers the suite calls directly:

- **`route_preview_run.hpp`** — `runPreview(args, out, err)` (arguments, the exact-name lookup including
  the names inside the registry's refusal sentences, the run, the page, the exit code) and `runProgram` (the
  same, with a failure of the preview itself said in words and exit 3); the RECORDER, a pacer layer put at
  `TesterStack::probe.inner` that stores the plant's TRUE pose after every pace and dates every log line to
  its pace (and tells the end action's steps from the routine's by the guard's own line, not by time); the
  scheduler's boundary observer; the NO-PACE STOP. `route_preview_main.cpp` only forwards `argv`.
- **`route_preview_model.hpp`** — what the page is drawn from, and THE FIELD CHECK (a 15 in square against
  `sim::FieldModel::override2026()`: THROUGH a wall or a Loader past a 1 in allowance fails, a TOUCH is a note).
- **`route_preview_render.hpp`** — the stdout table and the page, both from one `tableRows()`.
- **`fakes_by_kind.hpp`** — ANY robot's mechanism list on fakes, by kind (booted through the real
  `robot::MechanismParts` over the fake provider; an arm or lift on a `sim::ArmPlant` built from its OWN
  declaration, a flywheel likewise — each given an ASSUMED time constant where its declaration states no
  `kA`, because a plant with no inertia makes a speed loop overshoot every tick), bound into `TesterStack`
  through its last parameter, a `RigMechanisms` (`auton_tester_rig.hpp`). Left at its default, the rig is
  robot two's list over `RobotTwoFakes`, exactly as before (`route_preview_test.cpp`'s default-rig case pins
  four runs' truth and sink — one of them stepping every one of robot two's mechanisms — to fingerprints
  taken before the parameter existed, `route_preview_hash.hpp`).
- **`route_preview_test.cpp`, `route_preview_fixtures.hpp`** — the cases and their fixture autons.

**`hal::fake::FakeTelemetrySink::setOnLine(fn, ctx)`** — an opt-in hook called after each stored line with the
count of lines kept (default: none, so every other test is unchanged). The route preview's no-pace stop uses
it: an auton that keeps logging without ever pacing (a loop after a stop) would otherwise grow the sink until
it ran out of memory. The same hook notes the run guard's end-of-run line and every mechanism verdict logged
while an exception unwinds (which steps are the park's, and the step a run threw inside). A hook that throws
must only be set where every caller can carry the exception.

## Vendored dependency

[`vendor/doctest.h`](vendor/doctest.h) — **doctest v2.4.11**, a single-header test framework.
Source: <https://github.com/doctest/doctest> (pinned to release tag `v2.4.11`). License: **MIT**
(included at the top of the header). Chosen for zero install, fast compile, and clean support for the
strict/edge-case style above. Swapping it later is a single-file change.
