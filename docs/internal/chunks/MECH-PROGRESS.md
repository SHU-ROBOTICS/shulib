# MECH, the core — progress log (live)

- **Chunk:** MECH, the core mechanism library (brief:
  [`MECH-core-mechanism-library.md`](MECH-core-mechanism-library.md)) — six runs, A1–A6, each its own go,
  strictly in order (the brief's §2).
- **Executor model:** opus.
- Appended as the work goes, a heading per run. Every time in a section header is a `date` reading taken when
  the section was written, not an estimate written afterwards. **Nothing in this chunk has run on a robot.**
  Say "host test plant", never "simulator".

---

## MECH-A1 — the binding and the devices

- **Started:** Thu Oct  8 02:56:23 PM EDT 2026 (`date`).
- **Tree at start:** `bdcbc7c` (T4 Part 3 landed there — N12's precondition met), clean, in the git worktree
  `.claude/worktrees/mech-a1` (branch `mech-a1`) — NOT the main working tree.
- **Go:** the team lead, 2026-10-08: "Accept all, go" — N1–N13 all as recommended (a). The coordinator's
  workflow run launched MECH-A1 ONLY. Do not commit; no `pros` command; no `/dev/ttyACM*`. Mutation backups and
  probes under `.claude/coordinator/scratch/mech-a1/` (never `/tmp`, never the repository root).
- **Scope:** §2's A1 row — §1.2 `kinds.hpp`, §1.3 `spec.hpp`, §1.4 `wiring.hpp`, §1.5 `devices.hpp` + the
  read-fault seam (`hal/read_faults.hpp`, the fake, the three adapters' `IReadFaults`) + the five provider
  accessors in all four implementers + the config's sensor roles and `valid()` + `MotorMechanism::meanPosition`
  and the per-line `PneumaticMechanism` + `robot::MechanismParts` and the Robot forwarding to it; §1.16's A1 part
  (`route_ids.hpp`); §3.0 the compile-refusal harness and its CI step; §3.1's tests and mutations; §4.1's A1
  documents and §4.2's prose review; §4.3's HA-184 and HA-186 ONLY. NOTHING of A2–A6: no service, no
  behaviours, no `src/` file, no `tools/` file (`git diff --stat -- src/ tools/` must be empty at the end).
  Calibrate's declared-port drop and its push WATCH are CAL1's (N9): nothing of them is built here.

### 14:56 — start
- Read `docs/internal/RESUMING.md` (the chunk loop, the standards, pace rules 1–8) and the brief in full: its
  header (the go note), "Re-check 2026-10-04 — amendments" (what moved; N9–N13; the dispositions T1–T12, U1–U12,
  D1–D11 — these OVERRIDE the body), "Decisions for the team lead" (N1–N8), the design in plain words, §0–§9 and
  "Found while reading". This log created before any code.

### 14:57–15:06 — required reading, the overlay against the tree, and the baseline *(`date` 15:06)*
- **Read in the overlay** (`.claude/coordinator/scratch/brief-MECH-core/proposed4/`): `mechanism/{spec,kinds,wiring,
  route_ids,devices}.hpp`, `hal/read_faults.hpp`, `hal/fake/fake_read_faults.hpp`, and the overlay's diffs (against
  `tree_t43d`) of `hal/device_provider.hpp`, `hal/pros/{device_provider,distance,optical,digital_in}.hpp`,
  `hal/fake/fake_device_provider.hpp`, `hal/mechanism.hpp`, `config/robot_config.hpp`; `proposed_test3/
  robot_builder_test.cpp` (the two local providers' 10 lines); `p5/probe3_wiring.cpp` + `out5/probe3_wiring.txt`
  (P4's boot lines, which `MechanismParts`' tests reproduce); `team5/robot_x.hpp`, `team5/robot_two.hpp`; every
  `neg5/` TU and `neg5/expect.txt`, `neg5/run5.sh`.
- **Read in the tree at `bdcbc7c`:** `hal/device_provider.hpp` (whole), `hal/pros/device_provider.hpp` (whole),
  `hal/fake/fake_device_provider.hpp` (whole), `hal/mechanism.hpp` :180-344, `config/robot_config.hpp` :330-1030
  (`MechanismConfig`, `PortUse`, `valid()` whole), `robot/robot.hpp` :1170-1314 (the mechanism storage and builder)
  and its accessor block :633-737, `src/robots/mechanism_table.hpp` :150-330, `src/common/boot_source.hpp`
  :185-320 (today's conversion — the identity test's other side), `test/CMakeLists.txt` (whole),
  `.github/workflows/ci.yml` (whole), `tools/api_doc_tool.py` (target selection, exclusions, subsystem labels),
  `tools/briefing_status.py` and `tools/doc_staleness_audit.py` (headers).
- **The overlay's base vs mine (N12's re-verification).** `diff -rq tree_t43d/include <worktree>/include`: SEVEN
  headers differ, and of A1's files only `hal/pros/device_provider.hpp` — T4 Part 3's review round changed its
  banner (the one-writer guard paragraph, the competition program named) and `serialCopyDroppedLines()` (adds
  `logCopy_->contendedLines()`). None of the overlay's hunks touch those lines: the overlay's PROS-provider hunks are
  re-applied by hand onto `bdcbc7c`'s file (delta D1 below). The other A1 headers (`hal/device_provider.hpp`,
  `hal/pros/{distance,optical,digital_in}.hpp`, `hal/fake/fake_device_provider.hpp`, `hal/mechanism.hpp`,
  `config/robot_config.hpp`, `robot/robot.hpp`) and `test/robot_builder_test.cpp` are byte-identical to
  `tree_t43d`'s.
- **IDeviceProvider implementers at `bdcbc7c`** (`grep -rn "IDeviceProvider {"` over include/src/test/tools): FOUR —
  `FakeDeviceProvider`, `ProsDeviceProvider`, `test/robot_builder_test.cpp` :1782 `SilentRefusal` and :2160
  `SilentLine`. None in `src/` (so the pure virtuals cost `src/` nothing).
- **The brief's vendored citations, re-opened:** `include/pros/motors.h` :1255-1257 ("Gets the current limit ... The
  default value is 2500 mA") ✓; `include/pros/adi.hpp` :731-757 is `DigitalIn::get_value()`'s doc ("Gets the digital
  value (1 or 0) of a pin", :732), :705-729 `get_new_press()` ✓; `include/pros/rotation.hpp` :194-216 is
  `get_position()` ("current position in centidegrees"), :244-266 `get_angle()` ✓; `hal/pros/rotation.hpp` :79 binds
  `get_position()` ✓ (HA-186's premise).
- **Baseline at `bdcbc7c`** (`cmake -S test -B build/test`, then the build): the doc gate stopped on the briefing
  (this log's creation adds MECH to its open-chunk line) — `python3 tools/briefing_status.py generate`, rebuilt:
  **2253 cases / 2,783,389 assertions, 3 skipped, green** (build 28.7 s wall at -j8, suite 19.1 s). (The committed
  briefing says 2,783,363 assertions; the 26-assertion difference is in the tree as committed, not mine.)

### 15:06–17:06 — the library half written, every existing case green *(`date` 17:06)*
Copied from `proposed4/` as the starting point and re-written for the tree (every `PROTOTYPE` / "(proposed)" marker
stripped; `grep -rn PROBE_ include/` is empty — A1's slice never carried the switches). Files:
- NEW `include/shulib/hal/read_faults.hpp`, `hal/fake/fake_read_faults.hpp`, `mechanism/{spec,kinds,wiring,
  route_ids,devices}.hpp`, `robot/mechanism_parts.hpp`; `test/mechanism_fixtures.hpp` (robot x and robot two
  verbatim from §1.1, robot y = P5b's every-optional-word robot).
- CHANGED `hal/device_provider.hpp` (5 pure virtuals + a banner paragraph), `hal/pros/device_provider.hpp` (the
  overlay's hunks re-applied onto `bdcbc7c`'s file — D1), `hal/pros/{distance,optical,digital_in}.hpp` (`+ IReadFaults`,
  `faultedReads()` gains `override`), `hal/fake/fake_device_provider.hpp`, `hal/mechanism.hpp` (`meanPosition()`, the
  per-line `PneumaticMechanism`), `config/robot_config.hpp` (appended sensor roles + per-line levels, `PortUse::
  MechanismSensor`, `valid()`), `robot/robot.hpp` (owns a `MechanismParts`, every accessor forwards, `mechanismParts()`),
  `test/robot_builder_test.cpp` (the two local providers forward the five accessors — the overlay's 10 lines, wrapped).
- **Deltas from the brief / overlay, each with its reason** (numbered for the hand-back):
  - **D1** `hal/pros/device_provider.hpp`: the overlay's base (`tree_t43d`) lacked T4 Part 3's review changes (banner,
    `serialCopyDroppedLines()`); the hunks were applied by hand, both kept.
  - **D2** `PneumaticMechanism`: the overlay ADDED a loop to the single-level constructor (filling `safeLevels_`);
    landmine 10 says that constructor stays byte for byte, so it does — the per-line array is read only when
    `perLine_`. Named `kMaxPerLineLevels` (the overlay's `kMaxPneumaticLines` read as a capacity of lines, which it
    is not), plus `perLine()` and `safeLevel(i)` readers for tests and boot lines.
  - **D3** `MechanismSpec::autonOnly` removed (§1.3 says so); the stale "robot two: its kClawOpenCommand line" comment
    on `levelsSource` rewritten.
  - **D4 — a 61st spec refusal, `AStallSignatureNeedsLastingAbove0`** (appended after N10's never-trip pair, checked in
    the same loop, for EVERY stated signature). Found while reading: the overlay refused `lasting` 0 only on an arm's,
    a lift's and a flywheel's OWN `.stall` (as "UNSET"); a Power's optional signature and a HOMING signature with no
    `.lasting` passed. `StallDetector` trips on the first qualifying sample at persistence 0 (its own note,
    `manipulation/stall_detector.hpp` :40-42: "wrong for most"), and a motor starting from rest draws stall-grade
    current with its shaft still — so a stall-HOME would zero on its own spin-up spike and a Power's `stalls` step
    would succeed at once. Consistent with P6 (an unstated number is UNSET, refused). Alternative: leave it (F1 calls
    0 "legitimate for a mechanism with no spin-up"); rejected — the core's three users of a signature (a homing push,
    a `stalls` squeeze, a jam watch) all start from rest or from a command change. **For the coordinator: G3a mirrors
    `spec.hpp`'s list by sentence — it is 61, not 60.**
  - **D5** route-id helpers live in `namespace detail` (`appendRouteWords`, `refuseRouteId`, `addRouteId`) — the
    overlay's `detail_route` would have published them in the API reference (the doc tool excludes only `detail`);
    `add()` also checks `config::commandIdValid()` (F8's rule) beside the length.
  - **D6** `RoleState` (and a new `RoleFact`, one per role) live in `mechanism/devices.hpp`, which `wiring.hpp`
    includes — so `robot/mechanism_parts.hpp` records them without including the vocabulary (`kinds.hpp`, `wiring.hpp`);
    `mechanism::RoleState` keeps its name for every caller. `roleLine()` re-shaped so a 3-wire role names its line in
    EVERY state ("lift: home switch -> 3-wire B not asked (the mechanism is already ABSENT)" — the overlay printed
    "not checkable" for a line it never asked for); P4's lines for every other state are byte-identical.
  - **D7** `RolePort` gains `letter` so a bad LETTER (`homeSwitch('Z')`, `pieceSensor('Z')`) is refused as a bad 3-wire
    port, not "a smart port is not a number from 1 to 21".
  - **D8** `wiringProblem()` gains three refusals so it mirrors what `wire()`/`mechanisms()` refuse for a hand-built
    table: more than 8, two names alike, a position's level count vs the lines (U2). Route ids stay `routeIdsOf()`'s.
  - **D9** the fake provider: `fakeDistance/fakeOptical/fakeDigitalIn/fakeReadFaults/fakeLineReadFaults` (the house
    `fake*` naming beside `fakeMotor`/`fakeRotation`; the brief wrote `ownReadFaults`); `readFaults(port)` /
    `lineReadFaults(line)` answer ONLY where a sensor was SERVED (the overlay answered for any port) — the on-robot
    contract, so a builder reading a count for a sensor it never got fails on the host too; and BOTH one-direction
    preconditions (an input on a line made as an output AND the reverse — the overlay had only the first in the fake).
  - **D10** `RobotConfig::valid()` also refuses a HALF-STATED per-line level set (`perLineLevelsValid`): the field's
    note says all-or-nothing, and a half-stated set would build a line at an UNSET level, which `toLineValue()` reads
    as Low. The switch-line collision message mirrors the pneumatic one ("... (a limit switch)").
  - **D11** `MechanismParts` BORROWS its configs (a precondition: they outlive it) instead of copying them: the first
    build ran 2251/2253 — `robot_builder_test.cpp` :1639 and :2054 pin a built mechanism's `name()` as the very
    pointer into the ROBOT's own config copy. The Robot hands in that copy; MECH-A6's station keeps its configs in
    static storage. Alternative: copy, and change the two pins to compare against the parts' copy — rejected: the
    brief requires every existing case to pass unchanged, and the Robot already copies for the dangling-reference
    reason.
  - **D12** a refused HOME switch makes the mechanism ABSENT (it is how a switch-homed mechanism knows where it is),
    not "runs without"; unreachable on a brain (the provider refuses only a line outside 1..8, which `valid()` refuses
    first) — reachable with the fake's `refuseLine()`, and tested.
  - **D13** HA-182 (the starting values) is cited in `kinds.hpp`/`spec.hpp` as "the hardware assumption the brief holds
    as HA-182 (MECH-A3 registers it)" — the starting numbers are A1 data (a position spec refuses a 0 profile), but
    registering HA-182 is A3's (§4.3); no `PROVISIONAL (A4: HA-182)` label is written, so direction 1 of the register's
    reconciliation (no label without a registered ID) stays clean. **For the coordinator.**
- `python3 tools/api_doc_tool.py check-coverage`: "4458 public entities across 177 headers, all documented";
  `generate` wrote the 7 new pages (`devices`, `kinds`, `mechanism_parts`, `read_faults`, `route_ids`, `spec`,
  `wiring`) and updated 10 + `mkdocs.yml`'s nav (a new "Mechanism" group; the label is the tool's default for a new
  directory — `tools/` is not mine to edit).
- **Build (fresh briefing):** green. **Suite: 2253/2253, 2,783,911 assertions, 3 skipped** — every existing case,
  `robot_builder_test.cpp` included, after D11 (the first build: 2251/2253, the two name-pointer pins).

### 17:06–17:54 — the gates, two more holes, the tests *(`date` 17:54)*
- **ARM header gate** (RESUMING :107-112, every `include/shulib` header, 215 then): `rc=0`, clean.
- **`src` build gate** `python3 tools/src_build_gate.py check`: **PASS — all 7 builds link**, 11/11 TUs each, 0
  disallowed warnings (2 m 34 s). (`robot.hpp`, which `src/` includes, now includes `mechanism_parts.hpp` →
  `mechanism/devices.hpp`; `device_provider.hpp` three more seam headers.)
- **D14 — `PositionGeometryUNSET` (refusal 63)**, found while laying out the one-property cases: a `Lift` with no
  `.shape` carries geometry 0, which EVERY geometry check passes — with no `.tuning` it was refused for the WRONG
  reason (its starting profile scales to 0 in/deg: "the profile needs maxVelocity"), and with a stated
  `.tuning.maxSpeed` it was ACCEPTED, `angleDegToPosition` then answering 0 for every angle. Now refused right
  after the units check. **For the coordinator: G3a's mirror.**
- **D4 widened — `AStallSignatureNeedsAmpsAbove0` (refusal 62)**: a signature with `.amps` left out but
  `belowMotorRpm`/`lasting` written was read as "no signature" (`stated()` is amps > 0) — silently ignored on a Power
  and on a homing push; negative amps likewise. `StallSpec::written()` (any of the three) is now what the
  signature loop checks, then amps > 0, < 2.5, rpm > 0, lasting > 0. So `spec.hpp` holds **63 refusals** (the brief's
  60 + D4's two + D14). Generated from ONE table (`.claude/coordinator/scratch/mech-a1/gen/spec_refusals.py`) into
  the enum, the sentences, the named functions and the dispatch, so the four cannot disagree.
- **D15 — the per-line `PneumaticMechanism` was a footgun** (found by this run's own test): given a C ARRAY of safe
  levels, overload resolution prefers the single-level `(lines, bool, name)` constructor (array → pointer → bool is
  a standard conversion; array → `span` is user-defined), so `PneumaticMechanism hood(lines, safe, "hood")` would
  SILENTLY build the fan-out form with `safe = true`; only the test's braces made it a narrowing error. Fixed with an
  array-by-reference constructor template (an identity match) forwarding to the span form; the single-level
  constructor untouched (landmine 10). The test that found it now pins it.
- **Tests written** (all new files; robot z moved into `test/mechanism_fixtures.hpp` — two TUs defining it inline
  would record different files, an ODR violation): `test/mechanism_spec_test.cpp` (one property per case for EVERY
  refusal — the case FAILS if an enumerator is never produced —, the accepted boundaries, three "peel" chains that
  pin the check order, the sentences, the enum's numbers), `test/mechanism_kinds_test.cpp` (the words field by field
  for robots x, two and y, the starting values for Red/Green/Blue × 1:1/5:1/7:1 from first principles, `.tuning`,
  gearing's direction, where each declaration and each `wire(...)` was written — read from the fixture's text —,
  the conversions within 1e-12 and their clamps, the goals, BrakeWord), `test/mechanism_wiring_test.cpp` (robot two's
  projection IDENTICAL to today's `bootConfigFrom` field by field, the projection's signs / per-line levels /
  sensor fields, `declaredMotorPorts`, `clearOfDrive`, `wiringProblem`'s 18 mistakes, `roleLine` verbatim and
  bounded), `test/mechanism_route_ids_test.cpp` (robots x 17 / two 6 / y 18 in order with args, the stem rule, the
  four refusals with their `offending` text, 23 vs 24, 32 vs 33, `kMaxRouteIds == runner::kMaxCommands`),
  `test/mechanism_devices_test.cpp` (both providers' five accessors, every registry kind, the one-direction rule
  both ways, the read-fault seam through the REAL adapters over the shim, the "dead from boot = no object" proof,
  `FakeReadFaults`, `meanPosition`, the per-line pneumatic), `test/robot_config_mechanism_sensors_test.cpp` (P4b's
  five cases verbatim, every owner a sensor port collides with, one piece sensor, switch lines both orders, per-line
  all-or-nothing), `test/mechanism_parts_test.cpp` (P4's six census cases through the REAL `ProsDeviceProvider`, every
  boot line verbatim, a counting registry proving "not asked", the ABSENT rule, robot y over the fake with every
  device and count wired, the home/piece switch refusals, the per-line build, the Robot's forwarding object for
  object).
- **Suite: 2313/2313, 2,786,565 assertions, 3 skipped**; the MECH-A1 cases alone: **60 cases, 2622 assertions**.

### 17:54–18:38 — the compile-refusal harness, and every mutation run *(`date` 18:38)*
- **The harness (§3.0)** — `test/compile_refusals/`: **143 TUs** (140 refusals + 3 self-tests), each an
  `EXCLUDE_FROM_ALL` object target linked to `shulib_test_flags`, each a CTest (`compile_refusal_<stem>`) judged by
  `run_refusal.py`: PASS only when the build FAILS and every `// expect:` text appears in a diagnostic's MESSAGE (the
  text after `: error: ` / `: note: ` — never a path), and every `// expect-at: <line>` is a line of the TU GCC cites.
  **Delta D16:** the expected text lives IN the TU (`// expect:`), not as the brief's third CMake argument — one file
  per refusal, no CMake escaping of quotes (`operator""_deg`), more than one text or a location per TU possible. The
  three **self-tests** (WILL_FAIL) prove the judge cannot be fooled — run by hand, each fails for exactly its trap:
  `harness_a_tu_that_compiles` → "COMPILED: ... was NOT refused"; `harness_refused_for_another_reason` → "REFUSED FOR
  THE WRONG REASON" (it was refused with `target_is_outside_the_travel_limits`); `harness_target_is_outside_the_
  travel_limits` (its reason only in its own FILE NAME) → "REFUSED FOR THE WRONG REASON" (refused with
  `no_such_preset_on_this_mechanism`).
- **`test/CMakeLists.txt`**: the suite's include roots, `-iquote src` and defines moved onto ONE INTERFACE target,
  `shulib_test_flags`, which `shulib_tests` and every harness TU link (T11). **Proof it changed nothing:** the
  compile command of `robot_builder_test.cpp` from `compile_commands.json`, before and after: `diff` → IDENTICAL; a
  harness TU's command carries exactly those flags. `add_subdirectory(compile_refusals)`.
- **Two library changes the harness needed (D17):** `mechanisms()` re-checks each entry's spec (`refuse(specRefusal
  (w.spec))`) — an entry built BY HAND otherwise walked around every spec refusal, so the 16 reasons no declaration can
  produce (a kind byte, a units byte, a line count on a motor kind…) had no build-time path; and it range-checks a
  port (> 21) or a line (> H) before counting it (a hand-built port 22 was an out-of-bounds read inside a consteval
  function). The two "used twice" switch DEFAULTS are therefore unreachable (said in `wiring.hpp`).
- **Coverage, checked by script** (every `inline void <name>() {}` refusal of `spec/wiring/route_ids/kinds.hpp`
  against every TU's `// expect:`): **121 of 123** named refusal functions have a TU; the 2 without are the
  unreachable defaults above. First run: 2 TUs of mine wrong, both caught by the harness itself (`s01` reused a
  prefix that states `.name` → COMPILED; `n01` qualified the misspelt name, GCC says "is not a member of 'rx'" — a
  team's auton names it unqualified) — fixed. 26 TUs added after the coverage script found the port-NAMED variants
  2/3/5..21 and A/B/E..H untested and `write_an_arm_angle...` reachable with a constexpr unit (`arm.to(5_in)`).
  **`ctest -R compile_refusal`: 143/143** (63.3 s for 117; the 26 added 16.7 s).
- **CI** (`.github/workflows/ci.yml`): a step after "Run tests": `ctest --test-dir build/test -R compile_refusal
  --output-on-failure`; the file's header names it. `python3 -c "import yaml; yaml.safe_load(...)"` → parses.
- **MUTATIONS — every one RUN on a fresh build, observed RED, restored byte-identical (the driver checks it) and its
  timestamp bumped.** Driver: `.claude/coordinator/scratch/mech-a1/mutate.py`, list `mutlist.py`/`mutlist2.py`, one
  log per mutation in `scratch/mech-a1/mut/`. HARNESS side (`ctest -R`, the target rebuilt by the test itself):
  - **M0** `Arm::to()`'s limit check deleted → RED `compile_refusal_n03_target_is_outside_the_travel_limits`
    (`k_lift_past_top` stays green: only the arm's check was cut).
  - **M1** `mechanisms()` stops refusing a smart port twice → RED 22: `n10_a_smart_port_is_used_twice`,
    `w_sensor_on_a_motor_port`, `m1_port_twice_on_the_seventh_line`, and all 19 `w_smart_port_NN_named`.
  - **M6** `motors()` accepts a `char` → RED `n27_motor_port_letter` (refused for the WRONG reason: 'A' = 65 > 21).
  - **M38** `routeIdsOf` skips `findBuiltinCommand` → RED `n62_route_id_builtin`.
  - **M40** the stall pair's `< 2.5` → `<= 2.5` → RED `s59b_stall_at_2_5_amps` (`n58`, 3.0 A, stays green — why the
    exact-boundary TU exists).
  - **D4a** the `lasting > 0` check removed → RED `s61_power_stall_no_lasting`, `s61b_homing_stall_no_lasting`.
  - **D4b** the `amps > 0` check removed → RED `s62_stall_amps_left_out`.
  - **D14** the geometry check removed → RED `s63_lift_no_shape`, `s63b_lift_no_shape_tuned`.
  - **D17** `mechanisms()`' spec re-check removed → RED all 16 hand-built TUs (`s03 s05 s08 s09 s11 s12 s13 s18 s20
    s21 s29 s30 s33 s38 s43 s47`).
  - **U2** `wire()`'s levels-vs-lines check removed → RED `n42_one_level_two_lines`, `n43_two_levels_one_line` (n43
    then refused for the WRONG reason, "two positions have the same line levels" — exactly the brief's U2 finding).
  SUITE side (`-tc='MECH-A1*'` on a fresh build of the suite target):
  - **M2** `toMechanismConfig` drops a Reversed role's sign → RED the projection test ("a Reversed role is a NEGATIVE
    port") and parts P4.1 (`config(1).motorPorts[1] == -3`). **Said plainly:** robot two's identity test stays GREEN
    under M2 — robot two has no reversed role; the reversed-lift test is the net.
  - **M3** PROS `distance()` skips `expectKind` → RED 3: the kind sweep, parts P4.3, P4.4.
  - **M4** a refused ANGLE sensor treated as degrade-only → RED parts P4.3 and the M4 case.
  - **M5** `valid()` stops claiming sensor ports → RED P4b's five cases and the every-owner collision case.
  - **M7** the cartridge and safe-mode checks swapped → RED the CHECK-ORDER peel.
  - **M38 (run side)** → RED the built-in route-id case. **M40 (run side)** → RED the one-property table and the peel.
  - **M41** `readFaults()` never answers a distance sensor → RED 3: the seam case, "why the channel", parts P4.1.
    (First run: the seam case CRASHED on the null count, aborting the run — red, but untidy; the derefs are now
    behind `REQUIRE`s and the rerun fails cleanly.)
  - **M13** `applySafeState()` drives every line to line 0's level → RED the per-line pneumatic case.
  - **X1** the fake answers a count for ANY port (the overlay's behaviour) → RED the fake-provider case.
  - **X2** `MechanismParts` never fills `pieceFaults` → RED parts P4.1 and the robot-y case.
  - **X3** per-line levels never checked (D10) → RED the all-or-nothing case.
  - **X4** a refused PIECE sensor makes the mechanism ABSENT → RED parts P4.3, P4.4.
  - **X5** the PROS one-direction precondition removed → RED the `digitalIn()` case.
  - **X6** `meanPosition` reads one motor → RED the `meanPosition` case.
  - **X7** the per-line ARRAY constructor removed (D15) → RED AT BUILD: `narrowing conversion of '(const bool*)(&
    safe)' from 'const bool*' to 'bool'` — the footgun itself.
  - **X8** a refused HOME switch only degrades (D12) → RED the home/piece switch case.
  - **X9** the starting values read the gearing inverted (U1's class) → RED robot x's arm and the starting-values
    table.
  - **X11** the route-id stem keeps upper case → RED the stem case.
  - **X12** the projection ignores a non-uniform SAFE position → RED the projection case.
  - **X13** `wiringProblem` forgets the capacity (D8) → RED the mirror case.
  **A first suite round was NOT a result and is not counted:** its build was stopped by the doc gates before
  compiling (the briefing's suite count, then `check-fresh` — the API reference renders inline bodies), so M2, M3,
  M5, M41, X5, X6, X11, X12, X13 "failed at build" with no compiler error. The driver now builds the suite target's
  own `build.make` (header dependencies tracked — touching `route_ids.hpp` recompiled 6 TUs, then 0) and all were
  re-run above. Suite assertions are deterministic (2,787,144 three runs of one binary).

### 19:53 — RESUMED after the team lead's pause *(`date`: Thu Oct  8 07:53:01 PM EDT 2026)*
- The previous run was STOPPED at the usage limit (~18:45–18:50); the team lead resumed the same evening. This
  entry is the resuming executor's (opus), same worktree, same tip `bdcbc7c`.
- **What the tree holds, checked before anything else:** `git diff --stat -- src/ tools/ Makefile` → EMPTY (no gate
  self-test plant left by a killed run); `grep -rn PROBE_ include/` → empty; no "PROTOTYPE"/"(proposed)" in the new
  headers. **Every mutation restored:** all 31 logs under `scratch/mech-a1/mut/` end "RESTORED: byte-identical to the
  backup: True", and each mutated header is byte-identical to its LATEST backup — except `config/robot_config.hpp`,
  whose last backup (M5, 18:26) differs from the file by ONE prose hunk (:434-441, the per-line pneumatic paragraph
  of `MechanismConfig`'s note) written at 18:47, i.e. the docs pass, not a leftover mutant.
- **What the stopped run did AFTER its last log entry (18:38), unlogged — reconstructed from file times:** 18:40
  `test/mechanism_guide_example_test.cpp` (new); 18:41 the glossary; 18:42 the changelog; 18:42
  `docs/hardware-assumptions.md`; 18:44 `roadmap.md`, `REMAINING-WORK.md`, `HANDOFF-remaining-work.md`,
  `build-order.md`; 18:46 `cookbook/06-driver-control.md`, `faq.md`, `guide/10-the-api.md`,
  `vexbuilder-integration.md`; 18:47 `robot_config.hpp` (prose), `guide/13-extending-the-library.md`,
  `test/README.md`; 18:49 `README.md`; 18:49:43 `docs/api/` regenerated + `mkdocs.yml`. None of it is trusted
  until re-read below; no gate has run on it.
- **The plan from here:** a FRESH configure + build from source (a killed mutation run could have left a mutant
  binary), the suite and `ctest -R compile_refusal`; then re-read every document the stopped run touched against
  the code; then the prose review's greps (§4.2 + §4.1's re-check list), run and read BEFORE they are claimed; then
  every gate; then the hand-back.

### 19:53–20:25 — the fresh build, the harness, and the stopped run's docs pass re-read *(`date` 20:24)*
- **Fresh build from source** (`cmake -S test -B build/test`; `briefing_status.py generate`; `cmake --build
  build/test --target clean`; then the full build): rc 0, 33.9 s wall at -j8. The doc gates inside it: api_doc_tool
  self-test OK; "doc coverage: 4464 public entities across 177 headers, all documented"; "doc example scan: 537
  quoted lines, all verbatim (7 source files)"; removability clean; briefing self-test OK; staleness audit clean.
  **Suite: 2316/2316 passed, 2,787,160 assertions, 3 skipped** (32.7 s) — 2313 at 18:38 + `test/
  mechanism_guide_example_test.cpp`'s three cases (guide-10f..10h), written by the stopped run at 18:40.
  `briefing_status.py generate` again with the binary present → `check`: "generated block current".
- **`ctest --test-dir build/test -R compile_refusal -j8`: 143/143 passed, 62.1 s** (the RESOURCE_LOCK serialises
  them; -j8 changes nothing).
- **Every document the stopped run touched, re-read against the code** (each claim opened in the header it names):
  - `docs/changelog.md` — "Added since 2.4" at the top of `## API 2.4` (no bump). Claims checked: eight new headers
    (215 − 207 ✓), `setLevels`/`commandedLevel`/`perLine`/`safeLevel` exist (`hal/mechanism.hpp` :374-421),
    `specRefusal` has 63 reasons (63 enumerators ✓). **FIXED:** it listed "the mechanism seam F11" among "surfaces
    in no row of the Freeze Register" — F11 HAS a row (NOT FROZEN); re-worded: `RobotConfig`, `Robot` and
    `IDeviceProvider` are in no row, the mechanism seam is F11's, not frozen.
  - `docs/hardware-assumptions.md` — HA-184 and HA-186 rows + entries, the status count 171 → 173, the held line's
    note. Direction 2 of the register's reconciliation checked for both: every Source file named carries the ID
    (`kinds.hpp` :156/:307/:1141, `spec.hpp` :132/:173, `hal/pros/digital_in.hpp`'s new note; `hal/pros/
    rotation.hpp` is cited as carrying HA-11, not HA-186, and says so). **ADDED (prose review):** HA-158's Source
    now names `digitalIn()` too — the new accessor asks no registry and is tagged HA-158 (`hal/pros/
    device_provider.hpp` :493, :508).
  - `docs/guide/10-the-api.md` — the new section "Mechanisms — declared once, wired separately" (its listings are
    `test/mechanism_guide_example_test.cpp`'s lines; `check-examples` reads `test/*example*_test.cpp`). The boot
    lines it quotes are that test's assertions verbatim ✓; the route ids are guide-10g's ✓. **FIXED two:** its
    in-page link `#mechanisms-declared-once-wired-separately` → `#mechanisms--declared-once-wired-separately` (the
    em-dash heading's GitHub/pymdownx slug, the repo's own convention — e.g. guide 04's `#honesty-about-limits--…`;
    markdownlint MD051 flagged it); and its sample diagnostic pinned `wiring.hpp:598:52` — accurate today
    (measured: port 1 is :595:52, port 4 :598:52) but certain to go stale at A2–A4's first edit of `wiring.hpp`, so
    it now shows GCC's three lines with the library's line numbers elided (the user's line and the order of the
    notes are what the lesson teaches — measured with `cr_n10_a_smart_port_is_used_twice`).
  - `docs/guide/15-glossary.md` — declaration, role, wiring, piece sensor, "not checkable" (§4.1) and route id,
    read fault (the re-check's additions): all seven, each in alphabetical place; "mechanism" re-worded (the five
    kinds).
  - `docs/cookbook/06-driver-control.md`, `docs/faq.md`, `docs/guide/13-extending-the-library.md`,
    `docs/vexbuilder-integration.md` (:137, the gear-ratio sentence — `hal::DriveGeometry` is per side, :55-70 ✓),
    `README.md` (the directory map), `test/README.md` (the harness), `mkdocs.yml` (nav for the 7 new pages) — read;
    true to the code.
  - `config/robot_config.hpp` :434-441 — `MechanismConfig`'s note re-worded at 18:47 (the per-line form replaces T3
    addendum B2's never-built inverted-line sketch) — true to `PneumaticMechanism`'s per-line constructor and
    `MechanismParts::buildOne`.
  - The plan docs (shared — only MECH's lines touched): `build-order.md` (the first bold `Next:` gains MECH-A1 and
    "NEXT: MECH-A2"; MECH's own entry gains its A1 note), `roadmap.md` ("You are here" gains a MECH-A1 paragraph; a
    `[~]` MECH row), `REMAINING-WORK.md` row 12, `HANDOFF-remaining-work.md` (the data-layer inventory). **FIXED:**
    `build-order.md` said "63 build refusals by name" — counted: **123 named reasons** (`spec.hpp` 63, `wiring.hpp`
    47 — 18 reasons, the "used twice" pair through 29 port-named variants —, `route_ids.hpp` 4, `kinds.hpp` 9);
    re-worded with the split; the roadmap's sentence made exact the same way. They say "LANDED" — written for the
    tree after the coordinator's commit, as every chunk's do (pace rule 5).
- **Also fixed:** `test/compile_refusals/run_refusal.py`'s docstring named a self-test `harness_reason_only_in_
  the_path.cpp` that does not exist; it is `harness_target_is_outside_the_travel_limits.cpp` (the trap: refused for a
  misspelt preset, its stated reason only in its file name).

### 20:25 — the prose review (pace rule 4; the brief's §4.2 and §4.1's re-check list) — RUN, hits READ
Scope: `docs/` and `README.md`, `*.md`, excluding `docs/api/` (generated, gated by `check-fresh`) and
`docs/internal/chunks/` (dated records); the changelog's older entries are dated history too.
- **§4.2 A1** `grep -rnE "MechanismConfig|digitalOut|rotation\(|IDeviceProvider|mechanisms\(\)"` → hits in 12 files.
  Read every one: faq :55/:57/:234, HA :177-427 and :2425/:3185, roadmap :778/:1071, master plan :307/:315 are the
  IMU's `get_rotation()` (not ours); roadmap :306/:1869, vexbuilder-integration :64, HANDOFF :538-542, build-order
  :2147 are dated history; guide 13 :206 (`Robot::mechanisms()` hands you the list) and guide 10 :278 still true
  (the Robot forwards); cookbook 06 :201 is the stopped run's own edit; `MECH-PANEL-requirements.md` :99 is the
  requirements' "what exists" table as of their writing — stays. **Stale, FIXED:** `HANDOFF-remaining-work.md` :490
  ("`IDeviceProvider` has no `distance()` member") — it has one since this run; re-worded. **Incomplete, FIXED:**
  HA-158's Source (above).
- **§4.1 re-check** `grep -rnE "gearRatio|Speed\b|RobotMechanisms|faultedReads"` → `RobotMechanisms`: 0 hits.
  `gearRatio`: the `.vexbot` drivetrain field (vexbuilder-integration :108/:137/:735, roadmap :2125, the internal
  handoff prompt) — still has no reader, true. `Speed\b`: `maxLinearSpeed`/`maxAngularSpeed` and the legacy
  vocabulary — not the old `Speed` declaration; `MECH-PANEL-requirements.md` :204 names the kind SPEED, which the
  core keeps internally — stays. `faultedReads`: faq :22 (the stopped run's edit), HA :1013-1070 (R1a's adapter
  entries), the briefing :710, requirements :254 — all true.
- **Pace rule 4, per changed header** `grep -rnE "PneumaticMechanism|double.?coil|double solenoid|FakeDeviceProvider|
  ProsDeviceProvider|PortUse|mechanismAbsence|builtMechanismAt|pneumaticMechanismAt|ProsDistance|ProsOptical|
  ProsDigitalIn|meanVelocity|kMaxMechanisms"` → read every hit: faq :90/:110, guide 13 :139/:170, guide 10
  :288/:302, cookbook 06 :13/:107/:146/:198-209, guide 08 :298, README :248, roadmap F11/F14 rows and history,
  HANDOFF :165-674, build-order :564, HA :329/:1038/:1113/:1656/:2095 — all still true. **One stays, flagged for
  the coordinator:** `docs/vexbuilder-integration.md` :170-172 (the `.vexbot` contract §3.1) says a double-coil
  solenoid will get "a stated polarity" per port — the plan T3's addendum B2 sketched, which the per-line levels
  have replaced; but the contract §3.1 revision is the coordinator's note to VexBuilder (the brief's §6.4), not an
  executor's edit. Its "no such field exists today, so do not emit one" stays true.
- **Freeze Register (for the coordinator, not edited — a register change waits for the team lead, pace rule 2):**
  `hal::IReadFaults` is a new F4-additive sibling and `mechanism/*.hpp` a new vocabulary; neither has a row, and
  the register's own D2 lesson is that silence reads as "frozen". F11/F13/F14 each got a "NOT FROZEN" row when
  built. A row for each (NOT FROZEN; the vocabulary freezes as `.vexbot` schema at G3 via F7, §6.4) is the
  coordinator's to propose.

### 20:45–21:02 — one last fix, then every gate on the final tree *(`date` 21:01:49)*
- **Fixed after the first full gate round:** `robot/robot.hpp`'s new `#include "shulib/robot/mechanism_parts.hpp"`
  sat in the middle of the sorted `localization/` block; moved to the end of the block (after
  `motion/motion_scheduler.hpp`). It was made AFTER that round's `src` gate, so every gate below ran again on the
  final tree; the include order changes no compile (robot.hpp alone also compiles for ARM, -Werror).
- **The gates, final tree:**
  - Host build (strict -Werror, doc gates inside): rc 0. **Suite: 2316/2316, 2,787,160 assertions, 3 skipped**
    (`-tc='MECH-A1*'`: 60 cases, 2625 assertions; `guide-10f..h`: 3 cases, 12 assertions).
  - **`ctest -R compile_refusal`: 143/143 passed** (63.9 s): 140 refusal TUs + 3 harness self-tests (WILL_FAIL).
  - `python3 tools/src_build_gate.py check`: **PASS — all 7 builds link**, 11/11 TUs each, 0 disallowed warnings
    (147.8 s). `self-test`: **OK (211 detector cases, each against a real build)** (723.2 s). After both: `git status
    --short src Makefile tools` empty — no plant left.
  - ARM header gate (RESUMING :107-112): **215 headers, rc 0**.
  - CI greps: "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds: core is sim-free".
  - `api_doc_tool.py`: self-test OK; check-coverage "4464 public entities across 177 headers, all documented";
    check-fresh rc 0 (silent); check-examples "537 quoted lines, all verbatim (7 source files); 6 listings of
    whole-run files, each one unbroken run"; check-removability "no public doc references docs/internal/".
  - `doc_staleness_audit.py`: self-test OK (10 cases); "clean (29 live docs, 0 numeric claims checked)".
  - `briefing_status.py`: self-test OK (24 cases); check "generated block current" (regenerated after the
    build-order fix: MECH in the open-chunk line, 2316 / 2,787,160, 215 headers, 173 registered HA, 18 `[~]`).
  - The brief's §7 greps: `git diff --stat -- src/ tools/` EMPTY; `grep -rn PROBE_ include/` empty; no
    "PROTOTYPE"/"(proposed)" in `include/shulib` or the new tests; C7 removability grep empty; the register's
    direction-1 grep (`PROVISIONAL (A4` with no HA id) empty.
- **The tree's state:** NOTHING COMMITTED. Branch `mech-a1` at `bdcbc7c`, worktree dirty with MECH-A1's change only
  (`git status`: 38 modified, 22 untracked paths incl. `test/compile_refusals/` and `include/shulib/mechanism/`).
- **§2.1's definition of done:** the log ✓; suite + harness ✓ (CI runs both); ARM gate + PROS-free guard ✓; `src`
  gate check (and self-test) ✓; PROBE_/prototype grep ✓; every §3.1 test exists and every §3.1 mutation (M0–M7,
  M38, M40, M41) plus the run's own (D4a, D4b, D14, D17, U2, M13, X1–X9, X11–X13) was RUN and SEEN red, restored
  (17:54–18:38 above); docs in the same change, the prose review logged (20:25), C7 empty ✓; nothing committed ✓.
- **For the coordinator** (collected): (1) G3a mirrors `spec.hpp`'s list by sentence — it is **63**, not 60 (D4's
  two, D14); (2) HA-182 is cited in `kinds.hpp`/`spec.hpp` as "held, MECH-A3 registers it", with no `PROVISIONAL`
  label (D13); (3) `docs/vexbuilder-integration.md` :170-172's double-coil "stated polarity" plan is superseded by
  the per-line levels — the contract §3.1 revision is yours (§6.4); (4) no Freeze Register row for `hal::IReadFaults`
  or `mechanism/*.hpp` (silence reads as frozen — D2's lesson); (5) the plan docs say "MECH-A1 LANDED" for the tree
  after your commit; (6) main has since gained HA-189 and held HA-190..191 — this worktree's register (from
  `bdcbc7c`) shows "next free HA-189" and "Held … HA-187..HA-188"; the merge takes main's lines plus this run's
  HA-184/186 rows; (7) no file another in-flight chunk owns was touched (SER1's `char_sink.hpp`, `src/`,
  `tools/src_build_gate.py`, `tools/api_doc_tool.py`: untouched — `device_provider.hpp`'s `ProsCharSink usb_{}` line
  is unchanged by this run).

### MECH-A1 review findings and their disposition — started *(`date`: Thu Oct 15 09:55 AM EDT 2026)*
- Builder again (opus), same worktree, same tip `bdcbc7c`. The machine rebooted at 09:45; before anything else:
  `find . -newer docs/internal/chunks/MECH-PROGRESS.md` (outside `build/`) → EMPTY, so no file changed after this
  log's 21:03 entry (no partial fix, no leftover mutant); no process holds the worktree.
- Three reviewers (correctness, tests, docs) returned 25 findings. Each is re-checked below, fixed at the root or
  answered with evidence, and every gate re-run on the final tree.
- **Read before changing anything** (09:55–10:03, `date`): `wiring.hpp` whole, `spec.hpp` :200-560 and :1000-1302
  (`specRefusal` whole), `kinds.hpp` :1-1000, `robot_config.hpp` :425-520 and `valid()` :880-990,
  `robot/mechanism_parts.hpp` :180-407, `hal/mechanism.hpp` :260-439, `hal/pros/device_provider.hpp` :430-530, the
  six A1 test files, `test/compile_refusals/` (`CMakeLists.txt`, `run_refusal.py`, `refusal_common.hpp`, a TU of each
  family), `.github/workflows/ci.yml`, `test/README.md`, guide 10 :397-537, faq :18-26, glossary :165-175,
  vexbuilder-integration :165-175, the brief's §1.4, §2.1, §3.0, §3.1 and `neg5/expect.txt`. **Both scratch
  generators are still in step with the tree:** `gen/refusal_tus.py` regenerated into scratch → `diff -rq` against
  `test/compile_refusals/` EMPTY (143 TUs); `gen/spec.hpp.in` + `gen/spec_refusals.py` regenerated → IDENTICAL to
  `spec.hpp`. This round edits copies under `scratch/mech-a1/fix1015/gen/`, so the originals stay as they were.
- **10:04–10:08 (`date` 10:08:39) — the library fixes** (strict `-fsyntax-only` of `wiring.hpp`/`spec.hpp`/`robot_config.hpp`: clean):
  - `spec.hpp`, GENERATED from `fix1015/gen/` (`make_spec.py`: the template + the ONE table; the diff against the
    pre-fix file is exactly the hunks below): **six refusals appended (64–69)** —
    `PositionTheSettleTimeIsNegative`, `SpeedTheAtSpeedLastingIsNegative`, `PneumaticTheConfirmWindowIsNegative` (the
    three fields that shared another field's reason: findings 6 and, found while re-checking 6, the settle time beside
    the tolerance), `PositionTheHomingPushIsPast12Volts`, `PositionTheGravityHoldIsPast12Volts`,
    `PositionUnhomedVoltsIsNotIn0To12` (finding 7); each split condition checked right after its old partner, the
    gravity hold after the gravity kind, the push after "homing needs homeVolts", unhomedVolts after the home
    position. The preset, the home position and a linkage's limits are now "not inside" (`!(v >= min && v <= max)`),
    so a NaN is refused like any out-of-range number (the file's own idiom elsewhere).
  - `wiring.hpp`: `EntryProblem` + `entryProblem()` + `entryProblemText()` — the per-entry rules wire() enforces, as
    data, read by BOTH `wiringProblem()` (now: each entry's own rules first, then the cross-entry and drive checks)
    and `mechanisms()` (`detail::refuseEntry`, by wire()'s names) — finding 3's root fix; `indexOf(nullptr)` is -1
    (two hand-built entries were "the same mechanism"); `detail::sensors()` refuses a role written twice
    (`the_wiring_plugs_in_the_same_sensor_twice`, finding 1); `motors()` with five or more ports and `lines()` with
    three or more are refused BY NAME, and `motors()` / `lines()` with none fall into wire()'s count refusals
    (finding 5 and its zero-argument twin); a pneumatic wired with no line is refused by spec.hpp's
    `a_pneumatic_mechanism_needs_one_or_two_line_roles`, no longer by "not a letter A..H". Two new names for what only
    a hand-built entry can do: `the_wiring_names_another_mechanism_than_its_declaration`,
    `the_wiring_gives_a_different_number_of_lines_than_the_mechanism_has`.
  - `config/robot_config.hpp` `valid()`: a limit-switch piece sensor beside a distance or optical one is refused
    ("mechanism 'x' has a distance AND a limit-switch piece sensor -- a piece sensor is one device") — finding 4.
- **10:09–10:16 — the tests and the harness written** (`date` 10:16:00):
  - `test/mechanism_spec_test.cpp`: the three split fields re-expected to their own reasons; BOTH sides of the
    preset and home-position rules (finding 12); a NaN preset, home position and linkage pivot; the six new reasons
    with both sides and a NaN each; `boundariesAccepted` gains four motor roles, three stages, a 2-point table, a
    -maxRpm preset, 0 s lasting and settle, a home exactly at the top, ±12 V exactly for the push and the hold, 0 and
    12 for unhomedVolts (finding 15); the reachability bound moved to the last enumerator; numbers 64–69 pinned;
    three peels pin each new check's place in the order.
  - NEW `test/mechanism_hand_built.hpp`: one hand-built entry per `EntryProblem` (18, two rules reached two ways)
    plus a positive pair with `declared == nullptr` — the SAME constants the suite and the harness read (finding 3).
  - `test/mechanism_wiring_test.cpp`: the mirror's second direction (every hand-built entry: its spec valid,
    `entryProblem` names it, `wiringProblem` refuses it with its sentence; every enumerator has a row; every
    declaration-made entry of the four fixture robots passes; two hand-built entries compile as two mechanisms);
    a robot of exactly eight and a four-motor mechanism accepted at build and by the mirror; a collision through a
    piece SWITCH's line; an 8-entry hand table is no problem (finding 15).
  - `test/mechanism_kinds_test.cpp`: the DR4B's and the tilt's starting profile from first principles; EVERY
    `PositionTuning` and `SpeedTuning` field stated, each landing in its own field (finding 16); `lift.to(0_in)` and a
    0 s at-speed lasting compile.
  - `test/mechanism_parts_test.cpp`: a per-line hood whose SAFE position differs across its lines ("middle"), its
    `applySafeState()` and `safeCommand()` (finding 13); a two-motor lift whose FIRST motor is refused — its second
    motor and its switch NOT asked, the counting registry proving port 3 was never queried (finding 14).
  - `test/mechanism_devices_test.cpp`: idempotence as "not re-made" — no new registry query and a read-fault count
    that survives a second request, for distance, optical and digitalIn (finding 14); `meanPosition` over 1, 3 and 4
    motors (finding 17).
  - `test/robot_config_mechanism_sensors_test.cpp`: a limit-switch piece sensor beside a distance, beside an optical,
    and alone (finding 4).
  - The harness, GENERATED (`fix1015/gen/refusal_tus.py`): **194 TUs** (143 + 51). New: four sensor-written-twice
    TUs and the reviewer's port-hiding one (finding 1); a port used twice through the 2nd, 3rd and 4th motor role,
    the angle sensor, and a second line (findings 2, 9); five motor ports, three lines, `motors()`, `lines()`
    (finding 5); a piece sensor and a home switch the arm lacks, an unplugged angle sensor and piece sensor (finding
    10); the six new spec reasons (finding 6, 7 — the settle time by hand: `.tuning.settle` replaces only a value
    above 0, so no declaration reaches it); a preset below and a home above the range (finding 12); `arm.to(-1)`,
    `lift.to(-1_in)`, `flywheel.at(-4000)`, `lift["hihg"]`, `gears(12, 0)` (finding 11); 18 `hb_*` TUs over the shared
    hand-built entries (finding 3); `harness_wrong_location`, the location rule's self-test (finding 18). **Location
    pins added** where `mechanisms()`' own per-entry check would now raise the SAME name as `wire()` (n12, n13,
    w_extra_sensor, n42, n43 and the new unplugged/extra/`motors()`/`lines()` TUs): each is one `wire(...)` per line
    with `// expect-at:` on it, so a `wire()` that stopped refusing is red even though `mechanisms()` would still
    refuse — the masking the root fix of finding 3 would otherwise cause, caught before it shipped. n02, n06, n08 pin
    the team's line (finding 20).
- **10:16–10:27 — the build, the suite, the harness, the CI step** (`date` 10:27:42):
  - Fresh configure; `api_doc_tool.py generate` (the `///` comments changed) and `briefing_status.py generate`;
    build: first try RED at build — `test/mechanism_kinds_test.cpp`: a local `constexpr Flywheel` whose ADDRESS a
    `static constexpr` table takes is not a constant expression; made `static constexpr` (as the file's other
    declarations are); rebuilt rc 0. **Suite: 2321/2321, 2,787,687 assertions, 3 skipped** (+5 cases).
    **`ctest -R compile_refusal --no-tests=error`: 194/194** (101.6 s). Each of the four self-tests run by hand
    fails for exactly its own trap — the new one: "expected GCC to cite harness_wrong_location.cpp:6 (it cited lines
    [4])". Then "SIX presets" / "SIX speed presets" added to the accepted boundaries (S5 had been caught only by
    n63 being refused for the wrong reason); MECH-A1 cases 65/65.
  - **The CI step (finding 8)** now runs `ctest … --no-tests=error` AND requires the number of `compile_refusal`
    CTests to EQUAL the number of TUs in `test/compile_refusals/`. The step, verbatim as a script
    (`fix1015/ci_harness_step.sh`): real tree **rc 0** ("TUs: 194; their CTests: 194"). **G1** (`add_subdirectory
    (compile_refusals)` commented out, configured into a scratch build dir): the new step **rc 8** ("No tests were
    found!!!"); the OLD step on the same build **rc 0** — the reviewer's finding, reproduced. **G2** (the glob
    narrowed to `n0*.cpp`): 8 TUs ran and passed, the new step **rc 1** ("TUs: 194; their CTests: 8"). Both
    restored byte-identical (`cmp`), timestamps bumped. `yaml.safe_load` parses the workflow.
- **10:27–10:31 (`date` 10:31:18) — a mutation run that proved NOTHING, and why (recorded so it is never mistaken for a result):**
  the first driver (`fix1015/mutate2.py`) had TWO defects, both of which only ever report GREEN: (1) it read ctest's
  rc 8 as "no tests matched" — but ctest also exits 8 when tests FAIL, so every harness verdict was a false "NOT
  CAUGHT" (the logs show the truth, e.g. D1a: "COMPILED: … was NOT refused"); (2) after this round's reconfigure,
  `compiler_depend.make` was an empty placeholder, so the suite target's own `build.make` RECOMPILED NOTHING for a
  header mutation (each log: only "Linking CXX executable") and every suite verdict ran the unmutated code. The run
  was stopped at F7c2; the kill left F7c2's mutant in `spec.hpp` (found by `grep -rn MUTANT`), restored from its
  backup and proved equal to the generator's output (`cmp`); no other file carries a mutant. **Driver v3**
  (`fix1015/mutate3.py`, logs in `fix1015/mut3/`): "no tests" only on ctest's own "No tests were found"; the
  suite's `depend` step run before every build (compiler_depend.make 115 B → 3.1 MB); a header mutation that
  recompiles NOTHING is reported "INVALID", never green. Checked on two known cases before the run: D1a → harness
  RED (`w_sensor_twice_angle` COMPILED), R11 → suite RED AT BUILD (21 objects recompiled). All 82 re-run on v3.
- **D18 — what the brief's §3.0/§3.1 put in A1 that A1 cannot build yet (finding 22; logged now — the first run
  left it to the hand-back and a fixture comment):** the harness carries **43 of `neg5/`'s 66** (n01–n04, n06–n16,
  n23, n25–n30, n32, n35–n38, n41–n47, n52, n58–n63, m1, m3 — checked by `ls`). The **23 left out** test words that
  do not exist until later runs, so no A1 TU can compile up to their refusal: n05, n34, n39, n40, n53–n55 (the
  Routine's mechanism steps and `until.hpp`'s `seesPiece` — MECH-A2/A4), n17–n22, n24, n33, n48–n51, n56, n57, m2
  (the driver controls, `DriverFile`, the loop's re-zero button — MECH-A4/A5), n31 (the robot table file's
  `clearOfDrive` `static_assert` — MECH-A5; A1 pins `clearOfDrive` itself in the suite). Each joins the harness in
  the run that writes its words (§3.0: "grown by every run"). **The POSITIVE TU** (`neg5/p01.cpp` +
  `probe4_forms.cpp` + `team5/counted_steps.cpp`) is all Routine steps — MECH-A4's; A1's positive compile is the
  four fixture robots in `test/mechanism_fixtures.hpp` (every word, every form at its exact limit: `arm.to(160)`,
  `arm.to(0)`, `lift.to(24_in)`, `lift.to(0_in)`, `flywheel.at(±3600)`, robot y's every optional word), compiled
  into the suite. **The suffix pin** §3.1 asks for (`src/robots/mechanism_table.hpp:<line>`) needs robot two's
  table in the new words, which is MECH-A5's (`src/` is not A1's); A1 pins the SAME rule — a recorded path is
  pinned by its SUFFIX, host and brain differ — on `test/mechanism_fixtures.hpp` (`mechanism_kinds_test.cpp`,
  "WHERE each declaration and each wire(...) was written"), and A5 moves the pin to the table file.
- **10:33–10:57 — the mutations, on driver v3, and the call-site sweep** (`date` 10:57:04):
  - **All 82 of `fix1015/mutlist3.py` RED, 0 green, 0 "invalid", 82 restored byte-identical** (`fix1015/mut_run3.out`,
    one log each in `fix1015/mut3/`; every two-sided mutation red on BOTH sides). They are every mutation the
    tests reviewer listed as surviving — R1, R2, R3, R4, R5b, R6b, R7, R8, R9, R10, R11, L1, L2, S1–S9, P1–P6, K1–K7,
    G5 — each now RED; the correctness reviewer's two (R1 = "drop the angle sensor", R2 = "count only motors[0]");
    one per new rule (D1a–c, F3a–d, F4, F5a–e, F6a–c, F7a–f with one-sided variants, K8–K14, R2a–d, R4b, R6c, LL1–3,
    WC1); and the first run's M1, U2 and D17, re-run because `mechanisms()` changed. G1 and G2 (the CI step) are
    above. **The location pins did the work they were added for:** R3, R4, R4b, R5b, R6b, R6c, WC1 and U2 (n42) are
    red ONLY through `// expect-at:` ("expected GCC to cite w_extra_piece_sensor.cpp:5 (it cited lines [4])") —
    without the pins each would have been refused, by the same name, by `mechanisms()`' own check.
  - **Found by the per-condition audit (finding 12's "every multi-condition reason has a case per condition"):**
    "homing needs homeVolts" covers switch AND stall homing; only the switch side had a case. Added ("STALL homing
    with no push"); mutation **NHV** (drop the stall side) RED — and it would have survived before (a stall home at
    0 V passes every later check).
  - **THE CALL-SITE SWEEP** (`fix1015/sweep_sites.py`, finding 10's "count call sites, not names"): delete EACH call
    of a named refusal in spec/wiring/route_ids/kinds.hpp in turn; run every TU expecting that name; covered when one
    goes red. **First pass: 146 of 152.** Two "NO-TU" were the unreachable "used twice" fallbacks (expected). FOUR
    SURVIVED: `refuseEntry`'s two range refusals and the port loop's two range guards in `mechanisms()` — the same
    two names raised twice, so deleting EITHER copy was masked by the other (and F3a turned only 14 of the 18 hb TUs
    red). **Fixed at the root:** the port loop's range guards removed — `entryProblem()` already refused every port
    past 21 and line past H, and `mechanisms()` is consteval, so an out-of-bounds index stops the build (GCC:
    "array subscript value '22' is outside the bounds of array 'smart'") and can never read past the array. F3a
    re-run: **RED 18 of 18**. **Second pass: 148 of 150 covered, 150 restored byte-identical;** the two not covered
    are the "used twice" fallbacks, which no list can reach (each says so beside it). test/README and the changelog
    now state exactly that.
### MECH-A1 review findings and their disposition *(`date`: Thu Oct 15 11:16:14 AM EDT 2026)*
Three reviewers (correctness, tests, docs) returned 25 findings; the correctness lens's "port used twice is pinned
only for motors[0] and the piece sensor" and the tests lens's same finding are ONE (2 = 9 below). Every one was
re-checked against the tree first; every one was real. Each mutation named below ran on driver v3 (fresh objects —
the count of recompiled objects is in each log), was SEEN red and was restored byte-identical with its timestamp
bumped (`fix1015/mut3/<id>.log`; summary `fix1015/mut3/summary.txt`).

| # | Lens, severity | Finding | Disposition | Proof |
|---|---|---|---|---|
| 1 | correctness, major | a sensor role written twice: the last wins, the first port never checked — hiding a port used twice | **FIXED** in `detail::sensors()`: `the_wiring_plugs_in_the_same_sensor_twice` (a new named reason, wiring.hpp) | 5 TUs (`w_sensor_twice_{angle,piece,piece_switch,home,hides_a_port}` — the last is the reviewer's own port-4 case); D1a/D1b/D1c RED. No wiringProblem mirror: a `MechanismWiring` holds one field per role, so a table cannot express the duplicate — nothing for G3a's validator to mirror |
| 2 = 9 | correctness minor + tests major | "port used twice" pinned only via motors[0] and the piece sensor; R1, R2 survived | **FIXED (tests)**: TUs through the 2nd, 3rd, 4th motor role, the angle sensor, and a second LINE | R1, R2, R2a/b/c, R2d (piece), LL1 (2nd line), LL2 (home switch), LL3 (piece switch), M1 (re-run) all RED |
| 3 | correctness, minor | hand-built entries walk around mechanisms()' wiring checks; the mirror one way only; two hand-built entries refused as "the same mechanism" | **FIXED at the root**: `EntryProblem` / `entryProblem()` / `entryProblemText()` — the per-entry rules as data, read by BOTH `wiringProblem()` and `mechanisms()` (`detail::refuseEntry`, wire()'s names; two new names for what only a hand-built entry can do); `indexOf(nullptr)` is -1. The mirror's second direction: `test/mechanism_hand_built.hpp` (18 entries, one per rule, read by the suite AND by 18 `hb_*` TUs) | F3a RED 18/18 (after the sweep removed the port loop's redundant range guards; 14/18 before); F3b RED at build (two hand-built entries); F3c, F3d RED. **The fix's own trap, caught before it shipped:** with `mechanisms()` re-checking each entry, a `wire()` check that stopped firing is still refused by the SAME name at the list's first line — so the wire()-level TUs now pin their `wire(...)` line (`// expect-at:`); R3, R4, R4b, R5b, R6b, R6c, WC1, U2 each RED only through that pin ("expected GCC to cite …:5 (it cited lines [4])") |
| 4 | correctness, minor | valid() accepts a distance/optical piece sensor AND a piece switch; the switch then never built | **FIXED**: valid() refuses a limit-switch piece sensor beside either ("… has a distance AND a limit-switch piece sensor -- a piece sensor is one device") | 3 suite checks; F4 RED |
| 5 | correctness, minor | five motor ports / three lines a plain C++ error, though the names say otherwise | **FIXED**: `motors()` with 5+ and `lines()` with 3+ refused BY NAME; and their zero-argument twins (`motors()`, `lines()`) fall into wire()'s count refusals (`lines()` now names spec.hpp's `a_pneumatic_mechanism_needs_one_or_two_line_roles`, no longer "not a letter A..H") | 4 TUs; F5a–F5e RED |
| 6 | correctness, minor | two refusals blame the wrong field (negative confirmWithin → the move time; negative at-speed lasting → the band) | **FIXED**: two reasons appended (65, 66) — and a THIRD found while re-checking: a negative settle time was named as the tolerance (64) | one-property cases + 3 peels; TUs s64 (by hand: `.tuning.settle` replaces only a value above 0), s65, s66; F6a/b/c RED |
| 7 | correctness, minor | homeVolts, kG, unhomedVolts never range-checked; NaN passes the preset and home checks | **FIXED** (refused, not deferred to A3's clamp: a team writing 40 V for 4.0 V is told at build): reasons 67–69 — a homing push and a gravity hold within ±12 V (kG signed: elastics can pull harder than gravity), unhomedVolts 0..12; the preset, home and linkage checks are "not inside", so a NaN is refused | both sides + NaN cases; TUs s67, s68, s69, s69b; F7a/a2/b/b2/c/c2/d/e/f RED. **For the coordinator:** spec.hpp is now **69** reasons (G3a mirrors by sentence) |
| 8 | tests, major | the CI harness step passes with zero tests | **FIXED**: `--no-tests=error` + the CTest count must EQUAL the TU count | G1 (detached): new step rc 8, old step rc 0; G2 (glob narrowed): rc 1 ("194; 8") |
| 10 | tests, major | wire()'s sensor refusals pinned at one call site of three; docs say every refusal has a TU | **FIXED (tests + the claim made exact)**: 4 TUs (piece/home extra, angle/piece unplugged), each at its wire line; and a CALL-SITE SWEEP replaces the name count | R3, R4, R5b, R6b RED; the sweep: first pass 146/152 — four range refusals masked one another (fixed, above); second pass **148 of 150 call sites covered**, the two left being the unreachable "used twice" fallbacks |
| 11 | tests, major | constant-target checks pinned on one side; lift[] misspelt and gears driven unpinned | **FIXED (tests)**: `arm.to(-1)`, `lift.to(-1_in)`, `flywheel.at(-4000)`, `lift["hihg"]`, `gears(12, 0)`; `lift.to(0_in)` compiles in the suite | R7, L2, R8, R9, R10 RED; L1 RED at build |
| 12 | tests, major | two-sided spec refusals tested on one side | **FIXED (tests)**: a preset below the range and a home above it, in the table AND as TUs; every multi-condition reason now has a case per condition (the audit, every `\|\|` and two-sided range in `specRefusal` against the table — it found ONE more gap, "homing needs homeVolts" for STALL homing: case added, mutation NHV RED) | S1, S2 RED |
| 13 | tests, major | MechanismParts' per-line safe levels never asserted per line; P5 unpinned | **FIXED (tests)**: a hood safe at "middle" (High/Low): applySafeState → C High, D Low; `safeCommand()` = line 0's | P3, P5 RED |
| 14 | tests, minor | "not asked" for a later motor; PROS idempotence pinned only by address | **FIXED (tests)**: robot z's lift with its FIRST motor refused — motor 2 and its switch NOT asked, port 3 never queried; a second distance/optical/digitalIn request asks no registry and keeps the fault count | P4, P1, P2 RED |
| 15 | tests, minor | accepted boundaries unexercised | **FIXED (tests)**: 4 motors, 3 stages, 2-point table, ±maxRpm preset, 0 s lasting and settle, six presets (both kinds), an 8-mechanism list at build, an 8-entry hand table, a piece-switch line collision in the mirror | R11, S3, S4, S5, S5b, S6, S7, S8, S9 RED |
| 16 | tests, minor | starting values pinned for rotary arms / flywheels only; most `.tuning` fields unpinned | **FIXED (tests)**: the DR4B's and the tilt's profile from first principles; every PositionTuning and SpeedTuning field stated, each landing in its own field | K1–K14 RED |
| 17 | tests, minor | meanPosition tested with two motors only | **FIXED (tests)**: 1, 3 and 4 motors | P6 RED |
| 18 | tests, minor | no WILL_FAIL self-test of `expect-at` | **FIXED**: `harness_wrong_location` | by hand: "expected GCC to cite …:6 (it cited lines [4])"; G5 RED |
| 19 | docs, major | guide 10 says programs print the boot lines / use the read-fault count today | **FIXED**: guide 10's box, heading and both sentences; faq; glossary — robot two's programs BUILD their claws through MechanismParts (the Robot does) but print no role line until A5, and nothing reads the count until the service | re-read against `grep -rn "roleLine\|mechanismParts" src/` (0 hits) and robot.hpp :581 |
| 20 | docs, minor | guide 10: "a misspelt preset is refused ON your line" | **FIXED**: the example is `intake["fast"]`; the guide says a misspelt preset's error is the library's. Measured by `g++ -fsyntax-only` on each TU's own command: n06 `:4:44`, n08 `:4:38`, n41 `:4:93` — n02 `kinds.hpp:689:49`. n02, n06, n08 now pin line 4 | the harness |
| 21 | docs, minor | vexbuilder-integration's double-coil clause untrue at this commit | **FIXED** (a factual correction, not §6.4's contract revision: "this contract has no field for them yet, so do not emit one" stands) | re-read against robot_config.hpp's per-line fields |
| 22 | docs, minor | A1 work moved to A4/A5 without a line in the log | **FIXED**: delta **D18** above | `ls test/compile_refusals` |
| 23 | docs, minor | the fixture's `PROVISIONAL (HA-185)` names a held, unregistered ID | **FIXED**: the line now reads as a fixture value, HA-185 held for A5 | gate 26's grep (`PROVISIONAL (HA-18[1-6])` in include/ test/: none) |
| 24 | docs, minor | five out-of-list public docs edited with no logged reason | **FIXED**: one line per document below | — |
| 25 | docs, minor | "63 kinds of mistake in a declaration" overstates | **FIXED**: roadmap and build-order say 69 checks on one mechanism's spec, 52 of them a declaration can trip, 132 named reasons in all (69 + 50 + 4 + 9, counted by `grep -c "^inline void"`) | — |
| 26 | docs, minor | merge note: main rewrote the same plan-doc lines | **FOR THE COORDINATOR** (no merge happens in a worktree): take main's build-order `Next:` line, REMAINING-WORK row 12 and the register's header lines, apply this run's MECH-A1 sentences on top, and REGENERATE `PROJECT-BRIEFING.md` after the merge | — |

**Finding 24 — each out-of-list document and why it was touched** (pace rule 3):
- `docs/cookbook/06-driver-control.md` — its double-solenoid sentence linked to `MechanismConfig`'s note "says how
  it would be" (the inverted-line sketch); this run REWROTE that note (the per-line levels replaced the sketch), so the
  link's promise had to change with it.
- `docs/faq.md` — its "last good value" answer now names the read-fault seam this run added (`hal::IReadFaults`).
- `docs/guide/13-extending-the-library.md` — its one-line description of `PneumaticMechanism` would have been false
  (it commands one level OR, since this run, one level per line).
- `docs/vexbuilder-integration.md` — :137, a prose-review hit on `gearRatio`, stale since R3b (it said shulib had no
  gear ratio anywhere); :170-174, finding 21.
- `README.md` — the directory map gained `mechanism/`, a new public header directory.
- (`test/README.md` documents the harness; `mkdocs.yml` is `api_doc_tool.py generate`'s output — neither is a public
  prose document of §4.1's list.)

**Found while fixing, not fixed — for the coordinator (unnumbered):** a NEGATIVE `.tuning` number
(`.tuning.kP = -0.3`, `.tuning.settle = -0.1_s`, …) is silently replaced by the starting value
(`t.x > 0 ? t.x : start` — N7's "0 keeps the starting value" also swallows a negative), so a stated number is
ignored without a word; and `cosine(…, levelAt)`'s level angle and the gains are not range-checked at all. Refusing
them is more reasons for G3a to mirror, so it is put to the team lead rather than added here.

**The gates, final tree** (`fix1015/gates.sh`, one log each in `fix1015/gates/`; 10:57–11:16, every rc 0):
configure; `api_doc_tool.py generate`; `briefing_status.py generate`; CLEAN build (strict -Werror, doc gates inside),
37 s; **suite 2321/2321, 2,787,692 assertions, 3 skipped**; briefing generate (binary present) + **check "generated
block current"** + self-test (24 cases); **harness 194/194** (99 s; the CI step's count check "TUs: 194; their CTests:
194"); api_doc_tool self-test, **check-coverage "4503 public entities across 177 headers, all documented"**,
check-fresh (silent), check-examples "537 quoted lines, all verbatim", check-removability; doc_staleness_audit
self-test + "clean (29 live docs)"; the PROS-free and sim-free CI greps; **ARM header gate: 215 headers, rc 0**;
**`src_build_gate.py check`: PASS — every build links, 11/11 TUs each, 0 disallowed warnings** (169 s);
**`self-test`: OK, 211 detector cases** (690 s); `git diff --stat -- src/ tools/` and `git status --short src tools
Makefile` EMPTY; no `PROBE_` / "PROTOTYPE" / "(proposed)"; C7 removability grep empty; the register's direction 1
clean and no `PROVISIONAL (HA-181..186)` label anywhere in include/ or test/.

**The tree's state:** NOTHING COMMITTED. Branch `mech-a1` at `bdcbc7c`, worktree dirty with MECH-A1's change only.
New this round: `test/mechanism_hand_built.hpp`, 51 TUs (194 in all). §2.1's definition of done holds as at 21:02
on 10-08, with this round's tests, mutations (82 + NHV + F3a's re-run, all RED), the sweep and the gates added.

**For the coordinator** (this round's; the 10-08 list stands, its (1) now reading **69**):
1. **G3a mirrors `spec.hpp` by sentence: 69 reasons** (64–69 appended this round, numbers pinned). `wiring.hpp` has
   **50** named reasons (+3: the same sensor twice; two only a hand-built entry can raise) and a new run-time rule
   set, `EntryProblem` (17 values, sentences unchanged from `wiringProblem`'s) — 132 named reasons in all.
2. **Finding 26, the merge:** main rewrote the same plan-doc lines this worktree edits (build-order's first `Next:`,
   REMAINING-WORK row 12, the register's held and next-free lines, HANDOFF). Take main's lines, apply this run's
   MECH-A1 sentences on top, and regenerate `PROJECT-BRIEFING.md` after the merge (never keep either side's copy).
3. **Found while fixing, unnumbered:** a negative `.tuning` value is silently replaced by the starting value, and
   `cosine(…, levelAt)` and the gains are not range-checked — refusing them adds reasons G3a mirrors, so it is the
   team lead's call (above).
4. The vexbuilder-integration double-coil clause is corrected as a FACT (finding 21); the `.vexbot` contract
   revision (§6.4) is still yours.

### MECH-A1 review findings — the fix round RESUMED *(`date`: Thu Oct 15 12:27 PM EDT 2026)*
- The team lead paused all work on 2026-10-15 at ~11:15 near a usage limit; the coordinator relaunched this fix
  round as a continuation ("go slowly but please continue"). Same worktree, same tip `bdcbc7c`, nothing committed.
- **12:27–12:29 — the state the pause left** (checked before any change):
  - The section above ends at 11:16 with all 25 findings dispositioned and `fix1015/gates.sh` run to its end
    ("GATES DONE rc=0", 11:15:56; `fix1015/gates_run1.out`). No file in the worktree outside `build/` is newer than
    this log's 11:17:26 write (`find -newer`).
  - `git diff --stat -- src/ tools/ Makefile` and `git status --short src tools Makefile` EMPTY; the files the
    src gate's self-test plants into (`robot_config.hpp`, `test/CMakeLists.txt`, the vendored headers) carry no
    plant (their tails read; the vendored and `src/` files equal `HEAD`).
  - Every file the mutation tooling backed up (`fix1015/mut/`, `fix1015/mut3/`, `fix1015/sweep/`) EQUALS its newest
    backup byte for byte (kinds 21/21 backups, spec 31/31, the rest their newest; `wiring.hpp` equals its last 2 of
    70 — the earlier 68 predate the sweep's removal of the port loop's range guards, as logged). No `MUTANT` marker.
  - Both scratch generators still match the tree: `make_spec.py` → `spec.hpp` IDENTICAL; `refusal_tus.py` → the 194
    TUs, `diff -rq` empty (`fix1015/resume1015/`).
- **12:30–12:32 — the test binary relinked from source** (it could have been a mutant's link): configure,
  `briefing_status.py generate`, the old binary deleted, full build — **217 objects compiled** (every TU fresh),
  rc 0; **suite 2321/2321, 2,787,692 assertions, 3 skipped** — the gates' 11:15 numbers.
- **The finished work, re-verified from its own records** (`fix1015/mut3/summary.txt`): every one of the 82 + NHV +
  F3a re-run is RED with a fresh-object count or a red harness TU — none "invalid", none green. The files they
  mutated are unchanged since, EXCEPT `wiring.hpp` (the sweep's range-guard removal came after 30 of them): those
  are re-run below on the final tree.
- **A gap the finished round left — a dispatcher call is a call site too.** The call-site sweep deleted each
  DIRECT call `refused::<name>();`; it never deleted a DISPATCHER call (`refuse(specRefusal(…))`,
  `detail::refuseEntry(…)`, `refuseSmartPortTwice`, `refuseLineTwice`, `refuseRouteIds`, the route-id recorder
  `refuseRouteId`). The coordinator's verification copy (`scratch/mech-a1/verify/`, read as data) records one
  such deletion surviving every TU: `wire(Pneumatic)`'s own `refuse(specRefusal(x.spec))` — finding 3's fix
  re-checks every entry's spec in `mechanisms()` by the SAME name, so the wire()-level check can go unseen (the
  very masking this round pinned for the wiring checks, missed for the pneumatic spec). Plan: a DISPATCH-SITE
  SWEEP (`resume1015/sweep_dispatch.py`: 11 sites — wiring.hpp :911, :966, :1004, :1005, :1033, :1043, :1050;
  route_ids.hpp's four recorder calls — each deleted in turn, EVERY TU run, the suite too for the recorders),
  then a TU per survivor, then the sweep again.
- **12:33–12:56 — the dispatch-site sweep, first pass** (`resume1015/sweepd_run1.out`, a log per site in
  `resume1015/sweepd/`; every site restored byte-identical, `cmp` against the pre-sweep backups): **10 of 11
  COVERED** — `wireMotors`' spec check (m3, s04, s04b red), `mechanisms()`' spec re-check (17 red) and entry
  re-check (20 red), the port-twice (26) and line-twice (9) calls, `refuseRouteIds` (n60–n63), and the four
  route-id recorder calls (each its own n6x TU red AND the suite red, 7 objects recompiled). **SURVIVED:
  `wire(Pneumatic)`'s `refuse(specRefusal(x.spec))`** — 194 of 194 green: the verifier's reading, reproduced here.
- **12:56–12:58 — fixed (tests)**: a TU pinning the pneumatic spec check at its own `wire(...)` line,
  `m4_pneumatic_spec_at_its_wire_line` (two positions at the same level; the list puts one `wire(...)` per line and
  `// expect-at:` names the pneumatic's) — generated from the same scratch table (`fix1015/gen/refusal_tus.py`,
  one row added; regenerated into scratch, `diff -rq` → ONLY the new file), **195 TUs**. Judged by hand: "refused
  as expected … at line(s) [7]". **The sweep re-run on that site: COVERED** (m4 red, 1 of 195). By hand under the
  same mutant: "expected GCC to cite m4_pneumatic_spec_at_its_wire_line.cpp:7 (it cited lines [5])" — without
  `wire()`'s own check the refusal moves to the list's first line, which only the pin sees. Restored, `cmp` equal.
  `test/README.md` now says the sweep covers the 11 dispatcher calls too.
- **12:59–13:07 — EVERY mutation of the round re-run on the FINAL tree** (driver v3's copy `resume1015/mutate5.py`,
  logs in `resume1015/mut5/`, summary `resume1015/mut_run5.out`): `mutlist3` (82), NHV, F3a-after-guards and the
  new **PWSPEC** (`wire(Pneumatic)`'s spec check deleted) — **85 of 85 RED, 0 green, 0 "invalid", 0 not applied,
  85 restored byte-identical**; every suite verdict recompiled objects (7 to 49; five red at build), 46 harness
  verdicts red. This settles the 30 `wiring.hpp` mutations that predated the range-guard removal. Afterwards every
  mutated file `cmp`-equal to its pre-run state; no `MUTANT` marker anywhere in include/ or test/.
- **13:07–13:27 — EVERY gate on the final tree** (`resume1015/gates.sh` = the round's `gates.sh` writing to
  `resume1015/gates/`; **26 of 26 rc 0**): configure; `api_doc_tool.py generate`; `briefing_status.py generate`
  (before the build); CLEAN build (strict -Werror, the doc gates inside), 33 s; **suite 2321/2321, 2,787,696
  assertions, 3 skipped**; briefing generate again (binary present) + **check "generated block current"** +
  self-test (24 cases); **harness 195/195** (127 s; the CI step's count check "compile-refusal TUs: 195; their
  CTests: 195"); api_doc_tool self-test, **check-coverage "4503 public entities across 177 headers, all
  documented"**, check-fresh (silent), check-examples "537 quoted lines, all verbatim", check-removability;
  doc_staleness_audit self-test + "clean (29 live docs)"; the PROS-free and sim-free CI greps; **ARM header gate:
  215 headers, rc 0**; **`src_build_gate.py check`: PASS** (every build links, 11/11 TUs each, 0 disallowed
  warnings); **`self-test`: OK, 211 detector cases** (768 s); `git diff --stat -- src/ tools/` and
  `git status --short src tools Makefile` EMPTY; no `PROBE_` / "PROTOTYPE" / "(proposed)"; C7 removability clean;
  the register's direction 1 clean, no `PROVISIONAL (HA-181..186)` label in include/ or test/.
- **The +4 assertions (2,787,692 at 11:15 and 12:32 → 2,787,696), explained, not waved through:** two XML runs of
  the suite agree case by case (stable); with `m4_…cpp` moved out to scratch, ONE case differs —
  "P3-T3: the buttons left the mechanism table …" 1924 → 1920 — a source pin that reads every file under src/ and
  test/ for the retired names, four checks a file. The file was put back (`cmp` equal).
- **Finding 3 and finding 10, addendum** (the disposition table above stands; this completes both): the call-site
  claim now covers DISPATCHER calls — **direct calls 148 of 150 (the two unreachable "used twice" fallbacks),
  dispatcher calls 11 of 11** — and `wire(Pneumatic)`'s own spec check is pinned at its line by
  `m4_pneumatic_spec_at_its_wire_line` (PWSPEC RED). `test/README.md` says so.

**The tree's state (13:29, `date`):** NOTHING COMMITTED. Branch `mech-a1` at `bdcbc7c`, worktree dirty with
MECH-A1's change only (61 paths in `git status --short`, as at 11:16). New since the pause: ONE TU
(`test/compile_refusals/m4_pneumatic_spec_at_its_wire_line.cpp`, 195 in all) and three lines of `test/README.md`;
no header, no suite test and no public document changed. All 25 findings stand dispositioned as in the table
above; every mutation (85) RED on the final tree; both sweeps complete; every gate green on the final tree.

**For the coordinator** (the 11:16 list stands — 69 spec reasons, 50 wiring, 132 in all; finding 26's merge;
the unnumbered `.tuning` / `cosine` question; the `.vexbot` revision is yours). Added by the resumption:
1. **Your verification copy saw it first:** `scratch/mech-a1/verify/` (its PWSPEC: 0 of 194 red) found the one
   call the round's sweep could not see; reproduced here on the worktree, fixed by `m4`, and the sweep now covers
   every dispatcher call. Its suite verdict for PWSPEC ("RED AT BUILD (0 objects)") was the doc gates' line-number
   freshness stopping `all`, not a test — the driver here builds the suite target alone.
2. The harness grew by one TU (195); the CI step needs no change (it counts the TUs).
- 13:30 — after this log's last entries (the only file changed since the gates): `briefing_status.py check`
  "generated block current", `doc_staleness_audit.py` clean, `check-removability` and `check-fresh` rc 0.

### Verification-round findings and their disposition — started *(`date`: Thu Oct 15 01:54:21 PM EDT 2026)*
- A fresh verifier returned TWO findings (both minor, both documentation). Same worktree, same tip `bdcbc7c`,
  nothing committed. At 13:54 no file outside `build/` was newer than this log's 13:30 write (`find -newer`: empty).
- **Re-checked before any change** (the verifier's evidence, run here, not taken on trust):
  - **V1 (valid()'s own list).** `robot_config.hpp` :600-606 names only "a distance AND an optical piece sensor on
    one mechanism"; the refusal finding 4 added (:935, `switchLines[1]` beside `distancePort` or `opticalPort`)
    is in the code, the test (`robot_config_mechanism_sensors_test.cpp` :220-227), the `switchLines` field doc
    and the changelog, but NOT in valid()'s list nor in the `opticalPort` field doc (:497, "Never stated beside
    `distancePort`" — the switch omitted there too). `docs/api/robot_config.md` :1124 and :1424 copy both. REAL.
  - **V2 (the harness paragraph).** The two timings read from their logs: `fix1015/resume1015/gates/10_harness.log`
    :396 "127.47 sec" (`-j8`, but the TUs share a RESOURCE_LOCK so they run one at a time) and
    `verify/v3/logs/harness1.log` :396 "153.36 sec" — "under two minutes" is false at 195 TUs. Every named
    refusal counted against the TUs' `// expect:` lines (each `inline void <name>() {}` in the four headers):
    spec 69/69, kinds 9/9, route_ids 4/4, wiring 48/50 — no TU for `a_smart_port_is_used_twice` and
    `a_three_wire_port_is_used_twice` (the two unreachable fallbacks, wiring.hpp :495/:499, called at :774/:788).
    REAL. The blanket claim is in MORE places than the three the verifier named (grep `every refusal\|every named
    refusal\|two minutes\|every one has` over the tree, hits read): `test/README.md` :21 and :29;
    `test/CMakeLists.txt` :321; `test/compile_refusals/CMakeLists.txt` :3 ("One translation unit per BUILD-TIME
    refusal"); `test/compile_refusals/run_refusal.py` :6 ("every one has a TU here"); `.github/workflows/ci.yml`
    :5-6 and :108 ("Every one has a translation unit"); `docs/guide/10-the-api.md` :495; `docs/changelog.md`
    :102-103 ("every refusal AT BUILD by a translation unit"). `refusal_common.hpp` :9 ("every refusal of spec.hpp
    has a build-time test") is TRUE (69/69) and stays. The other "every refusal" hits are other chunks'
    run-time refusals (calibrators, drive program), not this harness.
- **13:54–13:57 — fixed at the root (documentation only; no code, no test changed):**
  - **V1** — `robot_config.hpp`: valid()'s "SINCE THE MECHANISM CORE" list now reads "two piece sensors on one
    mechanism (any two of a distance, an optical and a limit switch — a piece sensor is one device)"; the
    `opticalPort` field doc now says "Never stated beside `distancePort` or a piece switch (`switchLines[1]`) … valid()
    refuses two" (the second stale place the verifier did not name). The code is unchanged — its F4D/F4O mutations
    were RED last round — so no mutation is owed. `docs/api/` regenerated by the gates' `api_doc_tool.py generate`.
  - **V2** — the blanket "every refusal has a TU" made exact in all eight places: `test/README.md` (names the
    two fallbacks that have none, and why; the time claim replaced by the measured "two to three minutes … 127 s
    and 153 s, measured 2026-10-15 at 195 TUs", and why `-j` does not help: the TUs share a RESOURCE_LOCK,
    `compile_refusals/CMakeLists.txt` :44), `test/CMakeLists.txt`, `test/compile_refusals/CMakeLists.txt`,
    `run_refusal.py`'s docstring, `ci.yml` (the header comment and the step's comment; YAML re-parsed,
    `yaml.safe_load` OK; the step's commands and name unchanged), `docs/changelog.md` ("every refusal a robot file
    can reach AT BUILD … (all but two "used twice" fallbacks no list can reach)") and guide 10 ("Every refusal your
    robot file can reach has a test that fails to compile on purpose"). `refusal_common.hpp` :9 left: it speaks
    of spec.hpp only, where 69 of 69 names have a TU.
- **The prose review for this round (pace rule 4; RUN, hits READ before this line):** `grep -rn -i "piece sensor"`
  over docs/guide, docs/cookbook, faq, README, the register, vexbuilder-integration and the roadmap — 9 hits read:
  the glossary's (:147, "a distance sensor …, an optical sensor … or a limit switch" — one of the three,
  consistent; :179, a list of roles), guide 10 :503/:521/:526/:529, the roadmap :360, the register :1105/:1128 — none states the
  one-piece-sensor rule, none stale. `grep "valid()"` + sensor/mechanism over the same public docs: no hit. The
  harness grep is the one listed under V2 above (every hit read; the eight fixed, `refusal_common.hpp` true, the
  rest other chunks' run-time refusals). The MECH brief's §3.1 test list (:2205, "a distance AND an optical on one
  mechanism") is the coordinator's plan of record, not edited: the suite tests more than it lists.
- **13:56–14:15 — EVERY gate on the fixed tree** (`scratch/mech-a1/verify1015/gates.sh` = the last round's script
  writing to `verify1015/gates/`; **26 of 26 rc 0**, "GATES DONE rc=0"): configure; `api_doc_tool.py generate`
  (`docs/api/robot_config.md` :1124 and :1424 now carry the new wording, the old "distance AND an optical piece
  sensor on one" gone — grep count 0); briefing generate (before the build); CLEAN build, 34 s; **suite 2321/2321,
  2,787,696 assertions, 3 skipped** (the last round's figures: no test changed); briefing generate again + **check
  "generated block current"** + self-test (24 cases); **harness 195/195, 127.78 s** (inside the README's new
  "two to three minutes"; "compile-refusal TUs: 195; their CTests: 195"); api_doc_tool self-test, **check-coverage
  "4503 public entities across 177 headers, all documented"**, check-fresh (silent), check-examples "537 quoted
  lines, all verbatim", check-removability; doc_staleness_audit self-test + "clean (29 live docs)"; the PROS-free
  and sim-free CI greps; **ARM header gate: 215 headers, rc 0**; **`src_build_gate.py check`: PASS** (7 builds link,
  11/11 TUs each, 0 disallowed warnings); **`self-test`: OK, 211 detector cases** (689 s); `git diff --stat -- src/
  tools/` and `git status --short src tools Makefile` EMPTY; no `PROBE_` / "PROTOTYPE" / "(proposed)"; C7
  removability clean; the register's direction 1 clean. Afterwards the self-test's plant targets read clean
  (`robot_config.hpp` and `test/CMakeLists.txt` tails as written; no `MUTANT`/`PLANT` marker).

| # | Finding | Re-checked | Disposition |
|---|---|---|---|
| V1 | valid()'s list omits the switch-beside-a-smart-piece-sensor refusal | REAL (and `opticalPort`'s field doc too) | FIXED — both docs; `docs/api/` regenerated; check-fresh green |
| V2 | "under two minutes" false; "every named refusal has a TU" overstates | REAL (127 s / 153 s; 2 of 132 names without a TU; 8 places, not 3) | FIXED — all eight made exact; the measured time stated |

**The tree's state (14:15:58, `date`):** NOTHING COMMITTED. Branch `mech-a1` at `bdcbc7c`, worktree dirty with
MECH-A1's change only (61 paths in `git status --short`, as before this round — no path added). Changed this
round: `robot_config.hpp` (two doc comments, +2 lines), `docs/api/robot_config.md` (regenerated: the text and
the line anchors after :497; the generator rewrites `docs/api/`, `mkdocs.yml` and the briefing's generated block
whole — all already in MECH-A1's change set), `test/README.md`,
`test/CMakeLists.txt`, `test/compile_refusals/CMakeLists.txt`, `test/compile_refusals/run_refusal.py` (comments
and a docstring), `.github/workflows/ci.yml` (comments only), `docs/changelog.md`, guide 10, and this log. No code
and no test changed, so no mutation is owed; the 85 of the last round stand. (`find -newermt 13:54:30` also lists
`src/`, `Makefile`, the vendored headers and two `test/*_test.cpp` files: the src gate's self-test plants and
restores them — `git status --short` on each is EMPTY, so their content is `HEAD`'s.)
- 14:17 — after this log's last entries (the only file changed since the gates): `briefing_status.py check` "generated block current", `doc_staleness_audit.py` clean, `check-fresh` and `check-removability` rc 0; 61 paths in `git status --short`; `git diff --stat -- src/ tools/` EMPTY.

### The coordinator's merge verification — MECH-A1 (2026-10-15 14:20–15:00, main `03a8081` + the squash of `mech-a1`)
- **Audit before the merge:** no gate plant, mutant or probe marker in the worktree's added lines; `git diff --stat -- src/ tools/
  Makefile` empty; the transport commit `bcb1cdb`, then `git merge --squash mech-a1`. A stray `tree_build.log` (a 131-byte error
  log an agent had written into main's root at 01:40) was unstaged and deleted before the commit.
- **Four document conflicts:** the register (HA-184 and HA-186 rows beside main's HA-189; the count 174; MECH-A1's status
  paragraph placed after HA-189's, which landed first), REMAINING-WORK row 12 and build-order's `Next:` (A1 LANDED 2026-10-15;
  NEXT A2), the briefing (regenerated).
- **Gates on the merged tree** (`.claude/coordinator/logs/mech_a1_merge_verify.out`): host build with the doc gates; suite
  **2,321 | 2,321 passed, 3 skipped**; `src_build_gate.py` self-test OK and check **PASS, all 7 builds**; ARM header gate over
  **215** headers clean; CI greps empty; api check-fresh, the briefing, removability and the staleness audit clean; the
  compile-refusal harness **195 of 195** (80 s).
- **The coordinator's hand mutations** (`muts_mech_a1.json`, each on a fresh link, the whole suite, restored and hash-checked):
  **10 of 10 caught** — MA1–MA8 and MA10 RED (a refused piece sensor must not make the mechanism ABSENT; refused angle and home
  sensors must; the piece sensor's read-fault channel; NOT ASKED roles after an absence; `meanPosition`; the per-line safe
  state; the route words' case; the fake's read-fault count); **MA9** (the route words joined by `-`) is refused AT BUILD by the
  library's own route-id check (`refused::a_route_id_does_not_fit_…`, `route_ids.hpp` :263) — caught.

---

## MECH-A2 — the service, power, pneumatic, the guard's cut

- **Started:** Thu Oct  8 09:07:27 PM EDT 2026 (`date`).
- **Tree at start:** `85e0285` — a SNAPSHOT of MECH-A1's worktree taken the moment A1's BUILD finished (the team
  lead's SPEEDUP of 2026-10-08: the runs OVERLAP), on main `bdcbc7c`; clean, in the git worktree
  `.claude/worktrees/mech-a2` (branch `mech-a2`) — NOT the main working tree. MECH-A1 is in its review round in
  `.claude/worktrees/mech-a1`, NOT merged; the coordinator ports A1's final fixes into this tree before the merge.
- **Go:** the team lead, 2026-10-08: "Accept all, go" — N1–N13 all as recommended (a); the coordinator's workflow
  launched MECH-A2 ONLY. Do not commit; no `pros` command; no `/dev/ttyACM*`. Mutation backups and probes under
  `.claude/coordinator/scratch/mech-a2/` (never `/tmp`, never the repository root).
- **Scope:** §2's A2 row — §1.6 `until.hpp` and `service.hpp` (the slot, power, pneumatic, the service's seams,
  claimant, entry, `enterAutonomous`, leave, lookup; the evidence rule and the read-fault rule; the step
  preconditions); §1.8 the outcome appends (`manipulation/mechanism_outcome.hpp`) and N1 in `sequence/run_guard.hpp`
  with D5's wording; §1.12's plant scaffolding (`sim/mechanism_plant.hpp`); §3.2's tests and mutations; §4.1's A2
  documents (changelog — N1's BEHAVIOUR CHANGE; `docs/api/`; guide 11: the `[MECH]` lines, `MECHANISM_STALLED`,
  `mechanismCuts`); HA-183 ONLY. NOTHING of A3–A6: no position/speed behaviours, no `src/` or `tools/` file. A1's
  files are not edited except where A2's own scope requires an addition — each such touch logged under "A2 edits to
  A1 files (for the port)" below.

### 21:07 — start
- Read `docs/internal/RESUMING.md` (standing rules, pace rules 1–8), the brief in full (the go note, the
  amendments — which override the body —, N1–N13, §0–§9, "Found while reading"), and `## MECH-A1` above. This
  heading appended before any code.

### 00:24 (2026-10-15) — RESUMED after the team lead's pause *(`date`: Thu Oct 15 12:22:42 AM EDT 2026)*
- The previous run was STOPPED at 21:23 on 2026-10-08 (the team lead paused all work for the night); resumed
  2026-10-15 by the coordinator's workflow, same worktree, same base `85e0285`. This entry is the resuming
  executor's (opus).
- **What the tree holds, checked before anything else:** `git status` → only this log and
  `docs/internal/PROJECT-BRIEFING.md` modified (the briefing's generated block regenerated before a build — "Suite: not
  built"; it is regenerated again below); `git diff -- src/ Makefile` → EMPTY (no gate self-test plant left by the
  killed run); no new file in `include/` or `test/`. `scratch/mech-a2/` holds `configure.log` and `build0.log` (the
  stopped run's baseline build: `[100%] Built target shulib_tests`, no result logged). **So the stopped run did the
  start entry and a baseline build ONLY — no code. Nothing to keep or revert; MECH-A2 continues from "start".**
- **The plan from here:** a FRESH build from source and the suite (the baseline); the overlay's A2 slice read against
  this base; the code; the tests; the mutations; the docs; the prose review; every gate.

### 01:03 (2026-10-15) — RESUMED a second time *(`date`: Thu Oct 15 01:02:48 AM EDT 2026)*
- The 00:24 run above was itself STOPPED (its last file write 00:58:06, `test/sim_mechanism_plant_test.cpp`); it
  logged nothing after "the plan from here". This entry is the third executor's (opus), same worktree, same base
  `85e0285`. The machine is held awake for the run (a `systemd-inhibit --what=sleep:idle:handle-lid-switch` "Claude
  shulib builds" block, PID 8045, listed by `systemd-inhibit --list` at 01:02).
- **What the tree holds, checked first:** `git diff -- src/ Makefile` → EMPTY (no gate plant). `git status`: modified
  `manipulation/mechanism_outcome.hpp` (00:36), `sequence/run_guard.hpp` (00:41), this log; NEW (unlogged, by the
  00:24 run, reconstructed from file times) `mechanism/until.hpp` (00:36), `mechanism/service.hpp` (00:40),
  `sim/mechanism_plant.hpp` (00:42), `test/mechanism_service_rig.hpp` (00:43), `test/mechanism_service_test.cpp`
  (00:48), `test/mechanism_until_test.cpp` (00:49), `test/mechanism_service_power_test.cpp` (00:52),
  `test/mechanism_service_pneumatic_test.cpp` (00:52), `test/mechanism_service_evidence_test.cpp` (00:54),
  `test/run_guard_mechanism_cut_test.cpp` (00:57), `test/sim_mechanism_plant_test.cpp` (00:58). Its scratch: a
  baseline `suite1.log` at 00:24 (**2316/2316, 2,787,160 assertions, 3 skipped** — A1's final numbers, the base is
  sound) and `tu.sh` (one-TU syntax check with the suite's flags). No mutation backup exists in `scratch/mech-a2/`
  (none was run). **None of the unlogged code is trusted until re-read against the brief and built FRESH.**
- **The plan:** read every new/changed file against §1.6–§1.8, §1.12, §3.2 and the overlay's A2 slice; fix what is
  wrong (logged); a FRESH build and the suite; the remaining §3.2 tests; every mutation on a fresh binary; the docs;
  the prose review; every gate.

### 01:03–01:17 — the 00:24 run's code re-read, built FRESH, and four defects its never-run tests found *(`date` 01:17:29)*
- **Re-read against the brief** (§1.6–§1.8, §1.12, §3.2) and the overlay's A2 slice (`proposed4/shulib/mechanism/
  {until,service}.hpp`, `sim/mechanism_plant.hpp`, `manipulation/mechanism_outcome.hpp`, `sequence/run_guard.hpp`):
  the design is the brief's; the deltas are logged below (A2-D1 …). No `PROBE_` switch and no "PROTOTYPE"/"proposed"
  marker was copied (the overlay's A2 headers carry `PROBE_NO_MECH_CUT`, `PROBE_SETTLE_SPEED`,
  `PROBE_TIMEOUT_HOLDS_TARGET` — none is in the tree; `grep -rn PROBE_ include/` empty).
- **The first fresh build STOPPED at the doc gate** (`api_doc_tool` coverage): 32 "UNDOCUMENTED" — every member the
  00:24 run defined OUTSIDE its class (the overlay's layout) reads to the tool as an undocumented namespace-scope
  free function, and would have rendered on the API page as one. No other library header defines a member out of
  line. **A2-D1 (structure):** `service.hpp` restructured to the house style — every body inside its class; a slot
  reaches the clock, latch, sink and its robot's other slots through `detail::SlotHost` (owned by the service,
  pointed at by each slot), so no slot body needs the service's complete type. Behaviour unchanged but for A2-D2.
  Alternative: teach `tools/api_doc_tool.py` to skip `X::y(` definitions — rejected: `tools/` is not this run's
  (SER1 edits the tool; the brief's §7: `git diff -- src/ tools/` EMPTY). Pre-restructure copy kept at
  `scratch/mech-a2/pre-restructure/service.hpp`.
- **Built past the gate by the suite's `/fast` target first** (to see the tests before the restructure): 2392
  cases, **5 FAILED** — the 00:24 run's tests had never been built or run. Each was diagnosed, not loosened:
  1. `run_guard_mechanism_cut_test.cpp` (3 cases): "mechanism 'intake' cut ..." never found — the TEST read the
     wrong sink (the guard logs to the CHASSIS's telemetry; the rig built `MotionRig` with its own). Fixed in the
     rig: one sink for the chassis and the service (`MotionRig rig{kin, plantConfig(), &sink}`, the house pattern of
     `sequence_run_guard_test.cpp` :805).
  2. The D5 case expected the end action to return at T+4.0: wrong arithmetic — scoring returned at 0.9 s, so the
     end action STARTED there (before the 1 s deadline) and its 3 s step ran THROUGH the deadline to T+3.92 (the
     brief's own measured number for the tree's guard). Re-stated exactly: the end action starts before the
     deadline, returns after it, and the step ran its own bound (≥ 3.0 s, < one pace past).
  3. **A REAL PLANT DEFECT (A2-D3)** — `sim_mechanism_plant_test.cpp` "the BRAKE MODE at 0 V": Brake fell FASTER
     than Coast (3.6° vs 9.4°). Cause: Brake's volts were computed from the LAST step's speed (`−B·ω`), and with a
     memoryless joint (kA 0) the speed then flips sign every step once B/kV > 1 (here 3/1.2 = 2.5) — a ringing
     discrete loop, not damping. A stiff Hold on a fast joint rings the same way once S·dt/kV > 1. **Fixed: both
     solved IMPLICITLY** (the applied volts are `a0 − c·ω_next`: Brake c = B; Hold a0 = −S·(θ − held), c = S·dt;
     τ = kA/(kV + c); the ±12 V clamp re-solved), the flywheel's Brake the same; steady states unchanged. A new case
     pins it on a FAST joint (gear 1, kV 0.19: explicit gains 15.6 and 2.1) — at most one reversal, Hold at rest,
     Brake never faster than its damped steady speed.
  4. The same test's Coast expectation was wrong PHYSICS: an arm with its level at 90° does not fall to its −10°
     stop — the cosine load reaches the static band at θ* = 90 − acos(kS/kG) = 8.63°. Re-stated from that
     equation (and Brake approaches the same rest angle, slower).
- **A2-D2 (behaviour, found re-reading the tick):** a slot whose claim was RELEASED outside it (no `cancel()`, nobody
  took it — what T3's driver loop does to every leftover claim at its entry) went inert and left the motor at its
  last voltage, commanded by nobody (the half-safe failure in another coat). Now the slot puts it in its declared
  safe state on its way out (only when NOBODY holds the claim — a taken claim is the new owner's, untouched), the
  step CANCELLED "(its claim was released outside the service)". New case pins it, a step and a hold-to-run.
- **Fresh build** (`cmake --build build/test --target clean`, then the full doc-gated build; `docs/api/`
  regenerated for the gate — `service.md`, `until.md` new; `mechanism_outcome.md`, `run_guard.md` updated): rc 0,
  225 TUs. **Suite: 2394/2394 passed, 2,789,440 assertions, 3 skipped** (`-tc='MECH-A2*'`: 78 cases, 1976
  assertions).

### 01:17–01:26 — the remaining §3.2 tests, the harness's until.hpp TUs, and EVERY mutation run *(`date` 01:25:39)*
- **Tests added** (beyond the 00:24 run's files): `mechanism_service_power_test.cpp` "conditions OR-ed —
  `seesPiece | stalls`" (the jam first → SUCCEEDED by stall, no fault; the piece first → SUCCEEDED by the switch;
  `time | stalls` → by time) — §3.2's "alone AND OR-ed"; `mechanism_service_test.cpp` "a claim RELEASED outside the
  slot" (A2-D2); `sim_mechanism_plant_test.cpp` "Hold and Brake are IMPLICIT — a FAST joint never rings" (A2-D3).
- **The compile-refusal harness grows by until.hpp's refusal** (§3.0: "MECH-A2 adds until.hpp's") — four TUs, each
  pinned to its line (`// expect-at: 5`): `n39_sees_piece_on_an_arm` (the kind-has-none branch), `n40_stops_seeing_
  piece_on_a_pneumatic_with_none` (the declares-none branch), `u_sees_piece_on_a_power_with_none` (robot two's
  pivot), `u_stops_seeing_piece_on_a_flywheel`. `ctest -R "compile_refusal_(n39|n40|u_)"`: 4/4. The brief's n39/n40
  call `r.run(...)` (MECH-A4's Routine); here the condition alone — the refusal is the word's, not the step's.
  A1's `refusal_common.hpp` is NOT edited: each TU includes `until.hpp` itself.
- **ARM header gate** (RESUMING :107-112): 218 headers, rc 0; `service.hpp`, `until.hpp`, `sim/mechanism_plant.hpp`
  each alone, rc 0. **Sizes (ARM `-Os`, `sizeof`):** `MechanismService` 3 048 B (power + pneumatic only — the brief's
  8 048 B is the whole service with A3's slots), `PowerSlot` 224 B, `PneumaticSlot` 128 B.
- **MUTATIONS — 29, each RUN on a fresh build of the suite target (header dependencies tracked; the binary deleted
  after each so none can be reused), observed RED, restored byte-identical (the driver checks; timestamps bumped).**
  Driver: `scratch/mech-a2/mutate.py` (MECH-A1's, re-pointed), list `scratch/mech-a2/mutlist.py`, old/new text in
  `scratch/mech-a2/mutdefs/`, one log each in `scratch/mech-a2/mut/`. The brief's:
  - **M8** `cutClaimedMechanisms()` removed from both branches → RED 5: the deadline cut (P6), the disable cut, the
    abandon-inside-the-end-action cut, the F1 BEHAVIOUR CHANGE case, the retry count. **Its numbers re-measured (M8m,
    the deadline case alone): without the cut the 2 s step returns T+2.01 with 12.0 V·s past the cut, the 5 s step
    T+3.02 with 24.12 V·s, the hard stop FIRED and the end action FAILED; the disable case T+2.01 / T+3.02 with 18.0 /
    30.12 V·s; the in-end-action abandon T+3.92, 28.92 V·s; the F1 operation T+4.01 — the brief's P6 table and the
    checker's numbers, reproduced to the hundredth. With the cut: T+1.01, < 0.05 V·s.**
  - **M9** the tick skips `holdsClaim()` → RED 2: the claim TAKEN OVER, the claim RELEASED.
  - **M10** `pace()` ticks after `tickFromDriverLoop()` → RED 2: P8 (1200 ticks for 600), the seam case.
  - **M11** a leftover claim's `cancel()` called → RED 2: driver entry's spy, `enterAutonomous`'s spy.
  - **M12** a sensorless `grabs` not counted done → RED 2: robot two's claw, the missing-sensor grip.
  - **M13** `applySafeState()` drives every line to line 0's level (A1's `hal/mechanism.hpp`, restored) → RED: the
    hood's safe POSITION ("middle" = High, Low).
  - **M13b** a missing sensor read as "sees nothing" → RED: P13 (the false success returns).
  - **M35′** — the brief's M35 mutates `runAuton`'s service overload, which is MECH-A4's (it does not exist yet).
    Run here on the call's TARGET instead: `enterAutonomous()` drops nothing → RED 2: the D1 handover (269 of 269
    paces at 12 V) and the foreign-claim case. **For MECH-A4: M35 proper is yours.**
  - **M37** the abandon cut gated on `!inEndAction_` → RED: the abandon-inside-the-end-action case.
  - **M42** `seen()` ignores the read-fault count → RED 3: the moved-count case, R, the part-blind case. Re-run on
    R alone (M42r) to name its subcases: **R2, R3, R5, R8** red — the false verdicts return (R2, R3, R8 a false
    SUCCEEDED "stopped seeing", the motor never run; R5 a false TIMED_OUT, "it never came"); R4 stays green
    without the channel (its held NEAR value has confidence 0, which the evidence rule already reads as UNKNOWN —
    case J's net), R1, R6, R7 green as they should be.
  - **M43** a distance near at confidence 0 read as "gone" → RED 2: the evidence table, case J.
  The run's own: **M8b** the cut re-safes UNCLAIMED mechanisms → RED (HA-144: the counter's safes during scoring);
  **M37b** the DEADLINE cut inside the end action → RED 2; **MORD** the world advances before the service ticks →
  RED (the world's first step saw 0 V); **M9b** a released claim left unsafe (A2-D2 reverted) → RED; **M22a** the
  read-fault snapshot never taken at a tick's end (landmine 22) → RED 3; **MU1** `|` keeps the LEFT time (the
  overlay's operator) → RED 2; **MU2** two sensors not flagged → RED 2 (the until case, the step's precondition);
  **MP1** the evidence ignored on the tick the time is up → RED (the tie); **MP2** a missing sensor's verdict not
  counted (N8 (b)) → RED 2; **MP3** the hold-to-run's stall watch removed → RED; **MP4** `stalls` asked → still
  STALLED → RED 2; **MP5** volts not clamped to `maxVolts` → RED; **MP6** `stop()` repaints another owner's
  mechanism → RED; **MN1** START sent on EVERY driver entry → RED; **ML1** a disable makes a pneumatic safe → RED;
  **MD1** Brake explicit again (the plant, A2-D3) → RED 2; **MD2** Hold explicit again → RED; **MD3** a hard stop
  keeps the speed → RED 2.
  After the last: `grep -c MUTATION` over the five files touched → 0 each; `git diff --stat 85e0285 --
  include/shulib/hal/mechanism.hpp` → empty.
- **Rebuilt normally after the mutations** (`docs/api/` regenerated, the briefing regenerated, the doc-gated build):
  rc 0. **Suite: 2395/2395 passed, 2,789,466 assertions, 3 skipped** (`-tc='MECH-A2*'`: 79 cases, 1986 assertions).

### 01:26–01:41 — one more evidence hole, the documents, the prose review *(reconstructed from the file's mtime 01:41:24; no `date` was taken — the run was stopped; the header said "01:45" and "`date` at the entry's end" until the review round, 2026-10-15)*
- **A2-D12 (found re-reading `seen()`):** a reading that is NOT A NUMBER compared its way to an answer — an optical
  proximity of NaN fails `>=`, so it read "no piece" and would satisfy `stopsSeeingPiece`. The F4 seams promise finite
  readings (`hal/pros/optical.hpp` :52, :73), so no conforming adapter sends one; the service must still not turn
  "not a number" into evidence. Now UNKNOWN (a distance's or its confidence's NaN/Inf, an optical's NaN). Pinned in
  the evidence table case (four readings).
- **Documents (§4.1's A2 list, plus what the prose review found stale):**
  - `docs/changelog.md` — "Added and changed since 2.4 (2026-10-15)" at the top of `## API 2.4`, newest first: ADDED
    (until.hpp, service.hpp, the three verdicts, `mechanismCuts`, the plant), the **BEHAVIOUR CHANGE** (N1: an
    operation with a registered claimant now stops at the run's deadline, not at its own bound — with the numbers
    re-measured under M8), TESTED, HA-183. No version bump (A1's rule; API 2.5 is A4's). The claim "a `switch` over
    the enum ... none in the library or its programs did" was grepped first: the only `switch` over
    `MechanismOutcome` is `mechanismOutcomeName()`'s own.
  - `docs/guide/11-reading-the-diagnostics.md` — three new line entries under "Warnings, errors, and other lines":
    the `[MECH]` step verdict line field by field (every verdict word, every bracket the code writes; eight example
    lines in the tests' format — five verbatim from assertions, the rest with another mechanism's name or time),
    the service's leftover-claim line, the guard's `SEQ` mechanism-cut
    line with `RunGuardReport::mechanismCuts`; the `MECHANISM_STALLED` row widened (the service raises it once per
    stall, beside the step's line; a step that ASKED for `stalls` succeeds and raises nothing).
  - `docs/hardware-assumptions.md` — **HA-183** (row + entry + the MECH-A2 history paragraph; status 173 → 174; the
    held-numbers note now names HA-182 and HA-185 only). Reconciled both ways: every file HA-183 names carries the
    ID (`sim/mechanism_plant.hpp` 3, `sim_mechanism_plant_test.cpp` 1, `mechanism_service_power_test.cpp` 2); the
    direction-1 grep (`PROVISIONAL (A4` without an HA id) is empty.
  - `docs/api/` regenerated (`service.md`, `until.md` new; `mechanism_outcome.md`, `run_guard.md`, `README.md`,
    `all-entities.md` updated; `mkdocs.yml`'s nav gains the two pages under A1's "Mechanism" group).
  - Plan docs (only MECH's lines): `build-order.md` (the first bold `Next:` — "MECH-A2 LANDED 2026-10-15 … NEXT:
    MECH-A3"; MECH's own entry), `roadmap.md` ("You are here" gains a 2026-10-15 paragraph; the `[~]` MECH row gains
    A2), `REMAINING-WORK.md` row 12 ("A2 LANDED … A3 next"), `HANDOFF-remaining-work.md` (the inventory: the service;
    the guard's cut). They say "LANDED" for the tree after the coordinator's commit, as A1's do (pace rule 5).
    `PROJECT-BRIEFING.md`'s generated block regenerated (`briefing_status.py check`: current).
- **The prose review (pace rule 4; the brief's §4.2 A2 greps, then the run's own names) — RUN, every hit READ:**
  Scope: `docs/` and `README.md`, `*.md`, excluding `docs/api/` (generated, gated) and `docs/internal/chunks/`
  (dated records).
  - §4.2 A2: `grep -rnE "RunGuard|cancel-all|safeMechanisms|MechanismOutcome"` → 14 files. Read: cookbook 05 :73/:80
    and guide 09 :25/:268/:288 (`then()` accepts a `MechanismOutcome`, only `Succeeded` continues — still true; the
    three new values are non-successes and stop it too); cookbook 03 :115-150 (the guard's use; "frozen waits pay
    their remainder" — still true of a wait); diagnostics-plan :199 (D-8's history); guide 13 :154/:203/:258 (the
    cancel-all; the seam list); HA :2176 (abandon); `MECH-PANEL-requirements.md` :63-241 (the requirements as
    written — internal, stays); `PROJECT-BRIEFING.md` :137/:600 (generated / the briefing's history); the plan docs
    (MECH lines, mine). **Stale by omission after N1, FIXED:** `README.md` :65-68 (the guaranteed end of run now cuts
    every mechanism operation that registered its claim, not only the motion); `docs/guide/09-the-recipe-api.md`
    "The match clock" gains a bullet (a mechanism operation is cut too; the wait standing on it sees it end one pace
    later; an anonymous claim is reached only by the cancel-all); `docs/guide/13-extending-the-library.md` :207-213
    (a registered operation is cancelled the moment the deadline passes or the field disables, and again at the
    cancel-all); `docs/guide/12-when-things-go-wrong.md` (the guard's line list gains `mechanism '…' cut at the
    deadline`, and "a wait cannot be cut" gains its one exception).
  - The run's names: `grep -rnE "MechanismService|mechanism service|seesPiece|stopsSeeingPiece|orAfter|mechanismCuts|
    NOT_HOMED|ArmPlant|FlywheelPlant|mechanism_plant|until\.hpp|service\.hpp|enterAutonomous|enterDriverControl|
    leaveDriverControl"` → only this run's own new lines, the requirements doc (stays) and the plan docs.
    `grep -rnE "nothing runs|nothing reads a|its next parts|later parts|core's service"` → A1's dated changelog
    entry (history, stays), the HA register's MECH-A1 paragraph and section preamble (dated "In this part"; the
    preamble gains a "since MECH-A2" sentence — the service now READS a limit switch used as a piece sensor), cookbook
    06 :202 ("no driver control drives one yet; the core's later parts do" — still true: A4), vexbuilder-integration
    and roadmap :1920 (`.vexbot`, not ours).
  - `grep -rnE "\bclaimant\b"` → guide 12 :203 (the anonymous-claim line; true), roadmap F11 row (history). `grep -rnE
    "(six|five|seven) (verdicts|outcomes)"` → guide 11 :73, the MOTION outcomes (not ours).
  - **The register's consumers:** HA-154 (a pivot held into its stop is not watched) now says the service CAN watch a
    power mechanism's hold-to-run when it states a signature, and that robot two's pivots state none in the core's
    plan, so the claim stands for them; HA-157 names the service's `leaveDriverControl()` / `enterAutonomous()`
    (a pneumatic keeps its level); HA-184 names `Slot::seen()` (a switch piece sensor read against `pressedLevel`).
    `service.hpp` carries each ID at the line (HA-154 at `drive()`, HA-157 at both mode changes, HA-184 at
    `seen()`'s switch read).
  - **Not touched, deliberately:** `docs/guide/15-glossary.md` (no A2 entry in §4.1; "mechanism service",
    "done-condition" and "slot" would be the first — the mechanisms chapter, A3/A4, is where they are taught;
    **for the coordinator**); `docs/guide/10-the-api.md` (A1's mechanism section describes the declarations and the
    wiring, all still true; the service's API is A4's to teach with the steps that call it) *(**WRONG** —
    the review round found guide 10's box still saying "Nothing RUNS a mechanism through it yet" and "nothing
    reads it yet": this run's grep was case-sensitive and missed both. Corrected in "MECH-A2 review findings and
    their disposition", finding 19)*; the cookbook (A4's
    recipe). Pace rule 7: SEVEN public documents moved (changelog, guide 11, the register, `docs/api/` as listed;
    README, guides 09, 12, 13 as the prose review's fixes of statements N1 made incomplete) — the four fixes are each
    one paragraph and are the review's, not new scope.

### 09:54 (2026-10-15) — RESUMED a third time *(`date`: Thu Oct 15 09:54:32 AM EDT 2026)*
- The 01:03 run above was STOPPED after its prose-review entry (its last writes: this log 01:41:24, guide 11
  01:41:19); the laptop rebooted at 09:45 (`last -x reboot`). This entry is the fourth executor's (opus), same
  worktree, same base `85e0285`.
- **What the tree holds, checked first:** `git diff -- src/ Makefile` → EMPTY (no gate self-test plant);
  `git diff --stat 85e0285 -- src/ tools/` → EMPTY; `grep -rn PROBE_ include/ test/` → EMPTY. Every file the entries
  above name is present. **Not yet evidenced:** A2-D12 (`service.hpp` 01:38:54, the evidence test 01:38:38) was
  written AFTER the last build (`build11.log`, 01:32) and AFTER the mutation run (01:25) — no build, suite or mutation
  has seen it; `scratch/.../gate_check.log` (rc 0, 01:39) predates nothing it needs to but is re-run anyway; the
  gate self-test log is EMPTY (never ran to completion). Every mutation backup in `scratch/mech-a2/mut/` is checked
  against the tree before anything is built (a `MUTATION` grep over every file the list touches).
- **The plan from here:** the mutation-restore check; the brief re-read (the A2 row, §1.6–§1.8, §1.12, §2.1, §3.2,
  §4.1–§4.2, the amendments, N1–N13) against what the entries above claim; a FRESH build from source and the suite;
  A2-D12's own mutation; every gate; the prose-review greps re-run; the final report.

### 09:54–10:03 — the brief re-read, a FRESH build, A2-D12's mutations (one false GREEN caught), two small fixes *(`date` 10:02:50)*
- **Re-read** (this executor, in full): the brief's header and go, the amendments (which override the body), N1–N13,
  §0–§9 and "Found while reading"; the A2 row, §1.6–§1.8, §1.12, §2.1, §3.2, §4.1–§4.2, §7, landmines 1–24 against
  `service.hpp`, `until.hpp`, `sim/mechanism_plant.hpp`, the two appended headers and the eight test files. The
  deltas A2-D1 … D15 below stand as logged; nothing found that the entries above mis-state.
- **Mutation backups:** every `scratch/mech-a2/mut/*.orig` of `run_guard.hpp`, `until.hpp`, `mechanism_plant.hpp` and
  `hal/mechanism.hpp` is byte-identical to the tree; every `service.hpp` backup is identical to the others and differs
  from the tree ONLY by A2-D12's hunks and two HA-ID comments (HA-154 at `drive()`, HA-157 at `enterAutonomous()`)
  — `diff` read line by line. `grep -rn MUTATION include/ test/ src/` → only five older, unrelated comments.
- **FRESH build:** `briefing_status.py generate`; `cmake -S test -B build/test`; `--target clean`; the full doc-gated
  build — rc 0, **225 TUs compiled** (`build12.log`). **Suite: 2395/2395 passed, 2,789,470 assertions, 3 skipped**
  (`-tc='MECH-A2*'`: 79 cases, 1990 assertions — A2-D12's four NaN/Inf readings are the +4 over 01:25's 1986).
- **A2-D12's mutations — and a false GREEN, caught.** MNAN1 (the distance's non-finite guard → `if (false)`) and
  MNAN2 (the optical's `isfinite` → `if (true)`) first came back GREEN in 2 s each: **the driver's direct `build.make`
  call recompiled NOTHING** (`Building CXX` 0 in both logs) — a `--target clean` build leaves
  `CMakeFiles/shulib_tests.dir/compiler_depend.make` EMPTY (115 bytes) until a normal `cmake --build` runs the depend
  step, so a header mutation reaches no object file. Both runs are kept, renamed `*.invalid-no-recompile.log`, and
  count for nothing. One normal build (`compiler_depend.make` → 3.2 MB, `service.hpp` listed), then both re-run:
  **MNAN1 RED** (the evidence table, `:181` a NaN distance read YES, `:187` an infinite one NO), **MNAN2 RED** (`:179`
  a NaN proximity read NO) — 5 TUs recompiled each, restored byte-identical. *(The 01:20–01:25 campaign is not
  affected: every one of its logs shows its TUs recompiled — e.g. `M42.log`, `MP6.log`: 5 each.)*
- **A green hole closed (test added):** the confidence half of A2-D12's guard (`|| !std::isfinite(c)`) had no
  assertion of its own — a NaN confidence already fails `c > 0.0` by itself. The evidence table now also asserts an
  INFINITE confidence beside a near distance reads UNKNOWN (the seam promises [0, 1]). **MNAN3** (that half removed)
  → **RED** at `:188`; restored byte-identical.
- **A doc comment that was false, fixed (A2-D16):** `Slot::lastStep()` / `stepsBegun()` said "0 steps begun: lastStep()
  is a default report, not a verdict" — but a hold-to-run (`drive()`, not a step) that STALLS writes its STALLED into
  `lastStep()` with `stepsBegun()` still 0. The two comments now say so; the drive-stall case pins it
  (`stepsBegun() == 0`, `running()` false beside the STALLED). `docs/api/` regenerated (`service.md`); check-fresh 0.
- Rebuilt normally: rc 0. `-tc='MECH-A2*'`: **79 cases, 1993 assertions**, all passed.

### 10:03–10:26 — the harness mutation, the prose review re-run (two more stale-by-omission statements), every gate *(`date` 10:26:20)*
- **The until.hpp refusal's own mutation (§3.0: "a refusal no test can see is a refusal a mutation can delete")** —
  not in the 01:25 campaign: **MUH** `detail::needsPieceSensor` made a no-op → `ctest -R
  'compile_refusal_(n39|n40|u_)'` **RED 4/4** (each TU compiled, no reason named); restored byte-identical, the four
  green again.
- **The prose review, re-run by this executor (pace rule 4; every hit read):** §4.2's A2 grep `RunGuard|cancel-all|
  safeMechanisms|MechanismOutcome` (README.md + docs/, minus `docs/api/` and `docs/internal/chunks/`) — 14 files,
  the same hits as 01:26–01:45 plus this run's own lines; the run's names grep — only this run's lines, the
  requirements doc, the plan docs. **One more grep, for the claim N1 made incomplete:** `cannot be cut|can.t be
  cut|pays? (its|their) (own )?remain|frozen wait|runs? (on )?to its own (bound|timeout)` → cookbook 03 :144
  (`pause`/`waitFor` only — true), guide README :147 (F2's dated note — history), guide 12 :198 (already carries the
  exception), diagnostics-plan :202 (D-8's DELIVERED-at-F2 record — history, stays), guide 09 :321 (`pause()` and
  `waitFor()` — true; the mechanism bullet follows it), and **guide 14 :280 — STALE BY OMISSION, FIXED**: "Frozen
  waits pay their remainder. `pause`/`waitFor`/`waitUntil` …" now names the one exception (a wait on a mechanism
  operation that registered its claim ends one pace after the deadline — the guard cuts the operation), pointing at
  guide 09's section. Guide 06 ("Running out of match") and guide 17 ("The one wait an abandon cannot cut":
  `Chassis::wait()` / `Routine::pause()`) read — still true. **The Freeze Register's F12 row** (`docs/roadmap.md`
  :146) records every change to the guard's surface since F2 (COMP-a's abandon, COMP-t's `pacesTotal()`, its
  review's sweep) and did not record this one — **added**: "MECH-A2, 2026-10-15, made the latch also cancel every
  CLAIMED mechanism at the deadline and at an abandon — a BEHAVIOUR change, no signature change — and appended
  `RunGuardReport::mechanismCuts`". Pace rule 7: public documents moved are now EIGHT plus `docs/api/` (changelog,
  guide 11, the register — §4.1's; README, guides 09, 12, 13, 14 and the roadmap's F12 row — each one paragraph or
  clause, the prose review's fixes of statements N1 made incomplete).
- **GATES, final (the tree as handed back):**
  - fresh build earlier (`--target clean`, 225 TUs); the final doc-gated build rc 0 (`build16.log`); **suite
    2395/2395 passed, 2,789,473 assertions, 3 skipped** (`suite16.log`); `-tc='MECH-A2*'` 79 cases, 1993 assertions.
  - `ctest --test-dir build/test -R compile_refusal`: **147/147 passed** (`ctest_refusal2.log`).
  - ARM header gate (RESUMING :107-112, TU in `scratch/mech-a2/probes/all.cpp`): **218 headers, rc 0**; `service.hpp`,
    `until.hpp`, `sim/mechanism_plant.hpp`, `sequence/run_guard.hpp`, `manipulation/mechanism_outcome.hpp` each alone
    rc 0. `sizeof(MechanismService)` on ARM re-read through a failing `static_assert`: 3048 B.
  - `python3 tools/src_build_gate.py self-test`: **OK (211 detector cases, each against a real build)**, 14 min;
    `check`: **PASS (all 7 builds link …)**, every build `exit 0, 11/11 TUs compiled, 0 disallowed`. `git diff --
    src/ Makefile` EMPTY after both (the self-test's plants restored).
  - CI greps: "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds: core is sim-free".
  - `api_doc_tool.py` self-test OK; check-coverage "4583 public entities across 179 headers, all documented";
    check-fresh rc 0; check-examples "537 quoted lines, all verbatim (7 source files); 6 listings of whole-run files,
    each one unbroken run"; check-removability "no public doc references docs/internal/". `briefing_status.py`
    self-test OK (24 cases), check "generated block current" (regenerated before the build and again after the
    suite binary existed). `doc_staleness_audit.py` self-test OK (10 cases); the audit "clean (29 live docs …)".
  - `grep -rn PROBE_ include/` EMPTY; no "PROTOTYPE" / "(proposed)" / "proposed4" in any file this run wrote; the C7
    removability grep EMPTY; `git diff --stat 85e0285 -- src/ tools/` EMPTY.
- **Tree state:** nothing committed (`git log -1` = `85e0285`); 21 tracked files modified, 17 new (the code, tests,
  harness TUs and generated API pages listed in the entries above), no file outside this worktree touched but the
  scratch directory. MECH-A2's DoD (§2.1) is met as far as an executor can meet it; the coordinator's verification,
  the review round and the port of MECH-A1's final fixes come next.

### Deltas from the brief (MECH-A2), each with its reason
- **A2-D1** `service.hpp` defines every member INSIDE its class (the overlay defined ~30 out of line), a slot reaching
  the clock, latch, sink and its robot's other slots through `detail::SlotHost`. Reason: the doc gate reads a
  namespace-scope `X::y(` as an undocumented free function (and would render it as one); no library header defines a
  member out of line; `tools/` is not this run's to teach otherwise.
- **A2-D2** a slot whose claim was RELEASED outside it (nobody holding it) puts the mechanism in its declared safe state
  on its way out (the overlay went inert and left the motor at its last voltage). A TAKEN claim is untouched.
- **A2-D3** the plant's Hold and Brake (and the flywheel's Brake) are solved IMPLICITLY — the explicit form rang
  (Brake fell faster than Coast). Steady states unchanged; HA-183's text says so.
- **A2-D4** `Until::operator|` keeps the EARLIER of two times (the overlay kept the left one's: `time(3) | time(1)`
  was 3 s); `Until::conflicting()` ADDED — two different mechanisms' sensors in one condition (the overlay stored one
  name and silently dropped the other) are refused at the step.
- **A2-D5** the brief's §1.6 surface lists `PositionSlot`, `SpeedSlot`, `position(i)`, `speed(i)` and the `find()`
  overloads for `Arm`, `Lift`, `Flywheel`, `PositionGoal`, `SpeedGoal` in `service.hpp`: they are MECH-A3's behaviours
  (§2's A3 row), not built here. Until A3, `bind()` REFUSES a list holding an arm, a lift or a flywheel (a precondition
  naming MECH-A3) — never a slot that silently does nothing. **For MECH-A3: lift that refusal and its test case.**
- **A2-D6** the home switch's half of the read-fault rule, `Slot::homeSwitchPressed()`, and the R probe's case S (a
  switch dead from boot must not home the arm at bind) need a position slot and homing — MECH-A3's. Not built here;
  `bind()` refuses a home switch on a power or pneumatic mechanism. **For MECH-A3: S is yours.**
- **A2-D7** M35 mutates `runAuton`'s service overload (MECH-A4's); run here as M35′ on `enterAutonomous()` itself.
  **For MECH-A4: M35 proper.**
- **A2-D8** a piece sensor blind on EVEN ONE sample of a step ends it, at its bound, UNCONFIRMED not counted ("stopped
  answering: no evidence on N of M reads") — the brief says "a sensor that stops answering mid-step"; a sensor that
  answered NO again before the bound may still have missed the piece in its blind window, so TIMED_OUT ("it never
  came") would claim more than it knows. Both stop the chain; only the word differs.
- **A2-D9** surface added beyond the brief's verbatim: `Slot::stepsBegun()` (a default report is not a verdict),
  `Slot::pieceSensorPlace()`, `PneumaticSlot::kNoPosition` (its `commandedPosition()` reads the DEVICES back, so it is
  not `noexcept`), `MechanismService::bound()`, `Until::conflicting()`.
- **A2-D10** ONE step-line format for every verdict: `'<name>': <VERDICT>[ -- <what ended it>] in <s>s[ (<note>[;
  counted done])]` — the brief's examples spell some verdicts differently (`… ran 1.00 s`); guide 11 documents the one
  format field by field.
- **A2-D11** the guard's cut attempts every mechanism whatever one claimant throws, then rethrows the first (the
  `safeMechanisms()` rule of COMP-t's review), and logs the run's FIRST cut once (`SEQ`) — the brief's verbatim loop
  had neither.
- **A2-D12** a non-finite piece reading is UNKNOWN (above).
- **A2-D13** the brief's §3.2 "an unclaimed mechanism is not re-safed each pace (applySafeState counted: once, at the
  run's end)": run() has TWO cancel-alls (before and after the end action), both older than this run — the test pins
  2, and 0 during scoring.
- **A2-D14** the D1 handover's control reads 269 of 269 paces at 12 V without the call (the brief's probe: 257 of 269
  — its auton commanded something else meanwhile); with the call 0, as the brief.
- **A2-D15** the harness's until.hpp TUs state the condition alone (`seesPiece(rx::arm)`), not inside `r.run(...)`
  (MECH-A4's Routine).
- **A2-D16** (surface wording, 2026-10-15 09:54 run) `Slot::lastStep()` also carries the STALLED that ends a driver's
  hold-to-run (`drive()` is not a step, so `stepsBegun()` stays 0) — the brief's surface does not say what a
  hold-to-run's stall reports; the doc comments and a test now do.
- **A2-D17** (the review round, finding 3) a PNEUMATIC whose claim is RELEASED outside the service mid-step KEEPS its
  level (the step CANCELLED, said); a MOTOR mechanism is still made safe (A2-D2). Alternative: the safe position for
  every kind, as A2-D2 first did — rejected: every mode change keeps a pneumatic's level and its own verdict leaves it
  there (a claw must not drop what it holds); a release is not a request to be safe — the guard's reach (`cancel()`)
  is, and it still applies the safe state.
- **A2-D18** (finding 27) `REFUSED`'s documented meaning is "another owner holds the claim" only; the brief's §1.8
  second half ("or a route marker's args are wrong", N13 (a)(2)) names a refusal that does not exist until the route
  markers do (MECH-A4). **For MECH-A4:** widen `MechanismOutcome::Refused`'s doc comment, the changelog and guide 11's
  verdict list when a marker with bad args is refused as REFUSED.
- **A2-D19** (findings 4, 5) `Until` keeps the step's OWN sensor beside a NAMED one as a fact —
  `Until::readsOwnSensorToo()` (new surface) — and the STEP resolves it (one sensor when the name is its own
  mechanism's, refused as "two different mechanisms' piece sensors" otherwise); two different NAMES are still
  `conflicting()` at once. A declaration with no `.name` is stored as "" (never kSelf), refused at build by the words
  (`a_mechanism_has_no_name`, A1's) and at the step ("names a declaration with no .name"). Alternative: keep the
  second name and resolve both at run time — equivalent here, since the only unresolved pair is "self" + one name.
- **A2-D20** (finding 7) `~MechanismService` ends every slot still holding its claim (CANCELLED "its service was
  destroyed", the safe state, released) — the F1 operations' destructor rule; a device that throws is left, its claim
  STILL released, the other slots still ended. Alternative: state the lifetime as a precondition for MECH-A5's rig —
  rejected: a dangling registered claimant is the failure the claimant design exists to prevent, and bind() already
  requires the devices to outlive the service, which is exactly what the destructor needs.
- **A2-D21** (finding 8, and one found beside it) every `snprintf` field of the step line BOUNDED (`%.23s`, `%.11s`,
  `%.22s`, `%.180s`, the seconds clamped) so GCC proves no truncation at any -O; and `find(PneumaticGoal)` checks
  `kMaxPneumaticPositions` before the run-time count, so -O3 proves the index in range (`-Warray-bounds`).

### A2 edits to A1 files (for the port)
- **No A1 header, test, fixture or harness file was edited.** `include/shulib/hal/mechanism.hpp` was mutated (M13) and
  restored byte-identical (`git diff --stat 85e0285 -- include/shulib/hal/mechanism.hpp` empty). Four NEW TUs were
  added to A1's harness directory (`test/compile_refusals/{n39,n40,u_sees,u_stops}_*.cpp` — the CMake glob picks them
  up; `refusal_common.hpp`, `CMakeLists.txt` and `run_refusal.py` untouched).
- **A1's DOCUMENT lines touched** (the port merges them with A1's review): `docs/hardware-assumptions.md` — HA-184's
  *Source* gains `mechanism/service.hpp`; the "MECH-A1's additions" preamble gains one "(Since MECH-A2 …)" sentence;
  the register's held-numbers note (A1's sentence "HA-182, HA-183 and HA-185 … held") now names HA-182 and HA-185.
  `docs/internal/build-order.md` — A1's sentence in the first bold `Next:` ("host-only, nothing runs a mechanism
  through it yet … NEXT: MECH-A2 …") is rewritten into A2's; `docs/roadmap.md` — A1's "You are here" paragraph loses its
  last sentence ("Nothing runs a mechanism through it yet: next is MECH-A2 …"), A2's paragraph follows it;
  `docs/internal/HANDOFF-remaining-work.md` — A1's inventory clause "(MECH-A1, host-only, nothing runs through it
  yet)" is rewritten to add the service; `docs/internal/REMAINING-WORK.md` row 12's "A2 next".
- **What A2's code reads from A1's** (if A1's review renames any, the port re-points these): `MechanismSpec` (`name`,
  `kind`, `motorCount`, `lineCount`, `pieceSensor.{kind, seesWithinIn, seesProximity, pressedLevel}`,
  `power.{maxVolts, stall.{amps, speedRadS, time, stated()}}`, `pneumatic.{positions[].{name, levels, confirm},
  positionCount, start, actuationTime, confirmWindow}`), `nameOf`, `specRefusal`, `specProblem`, `Refusal::None`,
  `SensorKind`, `Confirm`, `MechanismWiring.{pieceSensorPort, pieceSwitch}`, `Line`, `lineLetter`,
  `MechanismTable.{specSpan(), wiring}`, `Devices` (every field), `PositionGoal`/`PneumaticGoal` fields,
  `hal::PneumaticMechanism::{setLevels, commandedLevel, lines}`, `hal::MotorMechanism::motors()`,
  `robot::MechanismParts::{build, devices, built}`, the fixtures' robots x / two / y, and `FakeDeviceProvider`'s
  `fakeMotor` / `fakeDistance` / `fakeOptical` / `fakeDigitalIn` / `fakeDigitalOut` / `fakeReadFaults` /
  `fakeLineReadFaults` / `refusePort` / `refuseLine`.
- **Added by the review round (2026-10-15, after the A1 port — the merge onto main meets these):**
  - **The A1 port itself is DONE in this tree** (the 14:50–15:02 entry: `git diff 85e0285 bcb1cdb` applied, seven
    overlaps resolved — build-order's first `Next:` line, the changelog's A1 heading A1's review renamed, this log, the
    roadmap merged clean, and four GENERATED files regenerated). A1's files are now `bcb1cdb`'s byte for byte.
  - **`mkdocs.yml`**: two nav lines (`Service: api/service.md`, `Until: api/until.md`) inside A1's `Mechanism:` group —
    INSIDE the `BEGIN/END GENERATED API NAV` block, written by `api_doc_tool.py generate`, never by hand: regenerate after
    any merge rather than merging the lines. **`docs/api/`** (`README.md`, `all-entities.md` list A1's entities beside
    A2's): regenerate after the merge too. **`PROJECT-BRIEFING.md`**'s status block: regenerate (finding 28's HA total
    and next free number among it).
  - **`docs/changelog.md`**: A2's section sits DIRECTLY above A1's entry heading — the one hunk of this port that
    conflicted (A1's review renamed that heading). A merge onto a main whose 2.4 section has grown again must keep A2's
    section on top (newest first) and A1's heading as main has it.
  - **A1-written PUBLIC text A2 now edits** (the review round's prose fixes): `docs/guide/10-the-api.md` — the
    mechanism section's "What exists, and what does not yet" box (A1's, as A1's review rewrote it) and the read-fault
    paragraph's last sentence ("nothing reads it yet"); `docs/faq.md` :18-27 (A1's review's read-fault sentence,
    "nothing reads that count yet"); `docs/guide/15-glossary.md` — the `read fault` entry (A1's, "its next part") and
    the `run guard` entry (older than A1). Each now says the service reads a PIECE sensor's count since the core's
    second part.
  - **A1's harness directory gains a FIFTH A2 TU**: `test/compile_refusals/u_sees_piece_on_a_nameless_declaration.cpp`
    (expects A1's own `a_mechanism_has_no_name`, raised by `until.hpp`'s `needsPieceSensor` — a CALL of A1's refusal,
    not an edit of `spec.hpp`). The CI step's TU = CTest count check covers it (200 = 195 + 5).
- **Added by the verification round (2026-10-15):** `docs/hardware-assumptions.md` — A1's top-of-register paragraph
  ("**MECH-A1 … added TWO** … nothing reads a switch or homes a mechanism yet … With them, 173.") gains the SAME
  "*(Since MECH-A2, 2026-10-15, …)*" sentence the section preamble carries (V1). A merge that rewords A1's paragraph
  keeps the sentence after it.

### 11:01 (2026-10-15) — MECH-A2 review findings and their disposition — START *(`date`: Thu Oct 15 11:01:36 AM EDT 2026)*
- The builder again (opus), same worktree, same base `85e0285`. Three reviewers (correctness, tests, docs) returned 29
  findings (overlaps deduplicated below). Each is RE-CHECKED against the tree before anything is changed; a real one is
  fixed at its root, one that is not is answered with evidence. The tree at start: as the 10:26 entry left it
  (`find include test -newer suite16.log` → nothing; only this log and the generated briefing changed since the
  build) — the reviewers worked in copies under `scratch/mech-a2/review-*/`, never in this worktree.

### 14:49 (2026-10-15) — the fix round RESUMED *(`date`: Thu Oct 15 02:49:54 PM EDT 2026)*
- The 11:01 fix run was STOPPED (the team lead paused all work at ~11:10 near a usage limit); the coordinator relaunched
  it as a continuation. This entry is the next executor's (opus), same worktree, same base `85e0285`, nothing
  committed. MECH-A1 has since LANDED on main (`fd36e07`), its final worktree change being `git diff 85e0285 bcb1cdb`.
- **What the stopped run left, checked before anything else:** its log section holds only the START line — NO
  finding dispositioned. But five files were changed after it (11:04–11:06): `mechanism/service.hpp`,
  `mechanism/until.hpp`, `sim/mechanism_plant.hpp`, `test/mechanism_service_rig.hpp`, `test/mechanism_service_test.cpp`
  — never built, tested or logged. Every other file equals the reviewers' copies (`diff -rq` of include/, test/,
  src/, docs/, tools/, README, mkdocs.yml against `scratch/mech-a2/review-correctness/a2/`; the three reviewers'
  copies agree with one another), so those copies ARE the pre-fix tree; the five pre-fix files are saved in
  `scratch/mech-a2/fix/prefix/`, the stopped run's versions in `scratch/mech-a2/fix/stopped/`, its diff in
  `fix/stopped-run-partial.diff` (441 lines). What it holds, read: the step line's bounded `snprintf` (finding 8);
  `Until` keeping the step's own sensor beside a name (`readsOwnSensorToo()`) and a nameless declaration stored as ""
  and refused (findings 4, 5); `~MechanismService` ending a slot that still holds its claim (finding 7) and the
  rig/fixture members reordered so the devices outlive the service; the ORDER comments at the two mode changes
  (finding 1, a comment only); a released PNEUMATIC keeping its level (finding 3, labelled "A2-D17" — renumbered
  below); the plant's Hold captured AFTER the step integrates (finding 2); one test (every mode change forgets a
  driver tick — findings 6/12). It is treated as a DRAFT: each piece is re-read against its finding and kept only
  with a test seen red under its mutation on a fresh binary.
- **Mutations left applied?** None. `grep -rn MUTATION include/ test/ src/` → only five older, unrelated comments; no
  `mut/` log or backup is newer than 10:05 (MUH, restored byte-identical then); the reviewers' drivers
  (`review-tests/rmut.py`, the correctness probes) wrote only to their own copies (`W = …/review-tests/src-copy`).
  `git diff --stat -- src/ Makefile tools/` → EMPTY (no gate self-test plant).
- **The plan:** (1) the A1 port, built and gated with the stopped run's edits PARKED (the pre-fix five in place), so
  the port is proven on its own; (2) the stopped run's edits back, each re-verified; (3) every finding not yet
  dispositioned, fixed at its root or answered; (4) every mutation on a fresh binary; (5) every gate on the final
  tree.

### 14:50–15:02 — THE A1 PORT: `git diff 85e0285 bcb1cdb` applied, every overlap resolved, built and gated with the stopped run's edits parked *(`date` 15:01:56 at the src gate's check)*
- **The diff** (`scratch/mech-a2/fix/a1-port.diff`, 7374 lines, 91 files; `bcb1cdb` = "MECH-A1 transport commit
  (worktree): built, reviewed (25 findings fixed), verified (2 doc fixes)", one commit on `bdcbc7c`). Overlap with
  this worktree's own changes, computed (`comm` of the two file lists): **seven files** — `docs/changelog.md`,
  `docs/internal/build-order.md`, `docs/internal/chunks/MECH-PROGRESS.md`, `docs/roadmap.md` (hand-written) and
  `docs/api/README.md`, `docs/api/all-entities.md`, `docs/internal/PROJECT-BRIEFING.md` (generated). The other 84
  (A1's `spec.hpp`, `wiring.hpp`, `robot_config.hpp`, the 52 new or changed harness TUs, `mechanism_hand_built.hpp`,
  the A1 tests, `ci.yml`, `test/README.md`, guide 10, the glossary, the faq, vexbuilder-integration, A1's API
  pages) applied cleanly (`git apply --exclude=<the seven>`, `--check` first); afterwards **each of the 84 is
  byte-identical to `bcb1cdb`'s** (`git show bcb1cdb:<f> | cmp`). A2's untracked files collide with none of A1's
  new ones.
- **The overlaps, each resolved by a three-way merge** (`git merge-file`, base `85e0285`; copies in
  `scratch/mech-a2/fix/merge/`):
  - `docs/roadmap.md` — merged clean (A1 reworded its own "Now (2026-10-08, later)" paragraph's count to "69 kinds
    of mistake in one mechanism's spec … 132 named reasons"; A2's removal of that paragraph's last sentence and
    A2's own paragraph sit beside it untouched). Read after the merge: both paragraphs whole and in order.
  - `docs/internal/build-order.md` — ONE conflict, the first bold `Next:` line both sides rewrote: A2's line kept
    (A2 LANDED, NEXT MECH-A3) with A1's one change applied on top ("123 named reasons: 63 in a declaration, 47 in
    the wiring" → "132 named reasons: 69 checks on one mechanism's spec — 52 of them a declaration can trip …, 50
    in the wiring").
  - `docs/changelog.md` — ONE conflict: A1's review renamed its own entry heading ("… (2026-10-08, later; its
    review round 2026-10-15) …"). A2's section kept above it, A1's new heading taken; A1's entry body (69 reasons,
    the harness wording) merged clean.
  - `docs/internal/chunks/MECH-PROGRESS.md` — both sides appended after A1's 21:01 entry: A1's four review-round
    sections placed FIRST (they belong to `## MECH-A1`), then `## MECH-A2` whole; `bcb1cdb`'s file is checked to
    be an exact prefix of the result.
  - `docs/api/*`, `mkdocs.yml`'s API nav, `PROJECT-BRIEFING.md`'s status block — GENERATED (between their
    `BEGIN/END GENERATED` markers): A2's copies kept and regenerated by `api_doc_tool.py generate` /
    `briefing_status.py generate` below (the nav's `Service` / `Until` lines are the generator's, inside A1's
    `Mechanism:` group).
- **What A1's review changed that A2's code relies on — checked, one by one:**
  - `specRefusal()` grew 63 → 69 reasons (64–69: a negative settle time, at-speed lasting or confirm window; a
    homing push, a gravity hold past ±12 V; `unhomedVolts` outside 0..12) and its range checks now refuse a NaN.
    `bindImpl` calls `specRefusal(s) == Refusal::None` / `specProblem(s)` — it inherits all 69 with no change.
    Every A2 test spec (the fixtures' and the hand-edited ones: `maxVolts = 13`, `pressedLevel`) still passes or
    fails exactly as asserted (the suite below).
  - `entryProblem()` / `EntryProblem` (wiring.hpp): the per-entry rules `mechanisms()` now re-checks. A2 reads a
    table's `wiring[i]` only to NAME a missing sensor's port in a log line; `bind(const MechanismTable&)` states
    "the robot's list (`mechanisms(...)`)", which `mechanisms()` has already held to those rules at build, and A1
    gives the run-time check of a hand-built table to the robot table file (`wiringProblem`, MECH-A5 —
    `wiring.hpp`'s header :37). **Not adapted, deliberately:** a second check in `bind()` would duplicate A5's, and
    the devices the service COMMANDS are checked against the spec in `bindImpl` already.
  - `MechanismParts`, `hal/mechanism.hpp`, `devices.hpp`, the read-fault seam: unchanged by A1's review
    (not in the diff). `valid()` now refuses a limit-switch piece sensor beside a distance or optical one — A2's
    `bindImpl` already refuses two piece sensors in one `Devices`; no A2 fixture states two.
  - The harness: A1's CI step now requires the CTest count to EQUAL the TU count; A2's four TUs (and this round's
    fifth, below) are counted by the same glob.
- **Built and gated with the stopped run's five edits PARKED** (the pre-fix files in place, so the port is proven
  on its own; `fix/gates.sh`, logs `fix/port-gates/`): configure; `api_doc_tool.py generate`; briefing generate;
  CLEAN build, **225 objects compiled**, rc 0; **suite 2400/2400 passed, 2,790,009 assertions, 3 skipped** — exactly
  A1's final 2321 / 2,787,696 plus A2's 79 / 2,313 (2,789,473 − 2,787,160); `-tc='MECH-A2*'` 79 / 1993; briefing
  generate + check "generated block current" + self-test (24); **harness 199/199** ("compile-refusal TUs: 199;
  their CTests: 199" — A1's 195 + A2's 4); api_doc_tool self-test, coverage "4622 public entities across 179
  headers, all documented", check-fresh, check-examples "537 quoted lines, all verbatim", removability;
  staleness audit self-test + clean; PROS-free and sim-free greps; **ARM header gate: 218 headers, rc 0**; no
  `PROBE_`/PROTOTYPE; C7 clean; the register's direction 1 clean; **`src_build_gate.py check`: PASS** (7 builds, 11/11
  TUs each, 0 disallowed). The src gate's self-test ran next (below).
- 15:02–15:16 — the src gate's **self-test: OK (211 detector cases, each against a real build)**, 829 s; afterwards
  every file it plants into (`robot_config.hpp`, `test/CMakeLists.txt`, `test/build_selection_test.cpp`,
  `test/chassis_table_test.cpp`, `test/vendor/doctest.h`, the vendored lvgl header) `cmp`-equal to `bcb1cdb`'s, and
  `git status --short src tools Makefile` EMPTY. **The A1 port is proven on its own: every gate green.**

### 15:02–15:28 — MECH-A2 review findings and their disposition *(`date`: Thu Oct 15 03:28:22 PM EDT 2026)*
- **The stopped run's five edits put back** (15:02) and each re-read against its finding before anything was built on
  it: kept, with these changes — the destructor now releases the claim even when the device throws (it did not); the
  ORDER comment's "puts a motor one SAFE" typo; the tick's released-pneumatic comment no longer tags HA-157 (a release
  is not a disable — HA-157 is the disable belief); its "A2-D17" label kept for the pneumatic rule (logged below,
  with REFUSED's half as A2-D18). Its one test (every mode change forgets a stale driver tick) kept as written.
- **Twenty-nine findings, three pairs and one triple the same** (6 = 11 + 12 + 14 — the correctness lens's R18, R2,
  R13 are the tests lens's R1, R5, R7; 9 = 26). Numbered in the order received. Every one was re-checked against the
  tree first; **every one was real** (17's premise in part: `mkdocs.yml`'s nav is GENERATED, see 9/26).
- **The build with every edit** (`fix/build1/`, 15:18–15:23): configure (the new TU globbed); `api_doc_tool.py
  generate`; briefing generate; CLEAN build, 225 objects, rc 0; **suite 2413/2413, 2,790,169 assertions, 3 skipped**
  (`-tc='MECH-A2*'`: **92 cases** — 79 + 13 new — **2141 assertions**); **harness 200/200**; every doc gate, the
  ARM gate (218 headers), the greps — all rc 0.
- **The mutations, on driver v3** (`scratch/mech-a2/fix/mutate3.py` — finding 17's fix, below; list
  `fix/mutlist_fix.py`; one log each in `fix/mut3/`; summary `fix/mut3/summary.txt`): **26 of 26 RED, 0 green, 0
  invalid, 26 restored byte-identical**, every suite verdict on a build that recompiled EVERY object depending on the mutated
  file (5 for `service.hpp`, 6 for `until.hpp`, 2 for the plant, 22 for `run_guard.hpp`), the harness verdict on
  ctest — and **M8r** (15:30, below) makes it **27 of 27**. **The first campaign (15:24) was STOPPED after six "INVALID" verdicts and counts for nothing**
  (`fix/mut3_run1_invalid/`): the new driver's dependents parser read `compiler_depend.make`'s trailing one-line rule
  per header as belonging to the last object (`xdrive_config_test.cpp.o`), so it demanded a sixth object that never
  depends on `service.hpp` — the INVALID rule did its job (no verdict rather than a false one). Parser fixed (only
  indented lines continue an object's list); the kill left no mutant (`service.hpp` `cmp`-equal to R14's backup,
  no `MUTATION` marker anywhere; its timestamp bumped).

| # | Lens, severity | Finding | Re-checked | Disposition | Proof (each mutation RED on fresh objects, restored) |
|---|---|---|---|---|---|
| 1 | correctness, major | a released FOREIGN claim's motor made safe only by statement order; reorder survives | REAL — both leftover tests asserted no motor | **FIXED (tests)**: both leftover tests seed pivot 7 at 12 V, Coast, before the mode change, and assert 0 V and Hold after; the ORDER comment at both calls | **R1a** (enterDriverControl reordered) RED; **R1b** (enterAutonomous) RED |
| 2 | correctness, major | the plant's Hold captured the angle one step BEFORE the command stopped | REAL — 17.55° back at 6 V, 35.54° at 12 V (the review's probe) | **FIXED** (the stopped run's): the capture after the step integrates (and in the jam branch). New case: a fast joint at 6, 12 and −9 V, then Hold, ends EXACTLY where it stopped (no gravity: no sag); HA-183's text says where Hold captures | **HOLDCAP** (capture one step behind) RED |
| 3 | correctness, minor | A2-D2 drove a released PNEUMATIC to its safe position mid-step | REAL | **FIXED + decided (A2-D17)**: a pneumatic keeps its level, as every mode change keeps it; a motor still made safe. New case on robot two's claw (open, mid-move, released → still open, CANCELLED, said) | **D17** (safe for every kind) RED; **D2m** (safe for none — A2-D2 re-checked after the split) RED |
| 4 | correctness, minor | `seesPiece \| stopsSeeingPiece(intake)` on the intake refused as two mechanisms | REAL | **FIXED (A2-D19)**: `Until::readsOwnSensorToo()`; the step accepts it on that mechanism and refuses it on any other. Until cases (both orders, joins, a third name) + a step case (runs on the roller and SUCCEEDS on its sensor; refused on the catapult, nothing commanded) | **MU3** (the flag dropped) RED; **MU4** (the step's check removed) RED; **MU8** (the reviewer's defect back) RED; **MU2r** (two NAMES not flagged, the 01:25 MU2 on the rewritten `\|`) RED; **MU1r** (keep the LEFT time) RED |
| 5 | correctness, minor | `seesPiece(nameless)` read the step's OWN sensor | REAL | **FIXED (A2-D19)**: a null name stored as ""; the words refuse it at build by A1's `a_mechanism_has_no_name`; the step refuses it by name. Until case, step case, and a fifth A2 harness TU (`u_sees_piece_on_a_nameless_declaration`, pinned at its line) | **MU5** (null stored as-is) RED; **MU6** (the build refusal gone) harness RED 1/5; **MU7** (the step's refusal gone: refused, but as "does not have") RED |
| 6 | correctness, minor | R2, R13, R18 survive | REAL | = 12, 14, 11 | (there) |
| 7 | correctness, minor | a destroyed service left a dangling claimant and the motor at its last volts | REAL | **FIXED (A2-D20)**: `~MechanismService` ends what it holds; releases the claim even when the device throws. Two cases: a step and a hold-to-run mid-flight (safe, released, no claimant left, another owner's claim untouched and never cancelled, the CANCELLED line); a device that THROWS (its claim still released, the next slot still ended, nothing escapes). Every fixture audited for the lifetime the destructor needs (the devices declared BEFORE the service: `Rig`, `HandPower` reordered by the stopped run; the evidence, cut and bind fixtures already were) | **DTOR** (ends nothing) RED 2; **DTORREL** (no release after a throw) RED |
| 8 | correctness, minor | the step line's `snprintf` warns `-Wformat-truncation` at -O1/-Os | REAL — reproduced on the pre-fix copy: -O1 5 warnings, -Os 1 | **FIXED (A2-D21)**: every field bounded. `fix/fmt_probe.sh` (the suite's flags without -Werror, -O1/-O2/-O3/-Os) over six TUs (the four service tests, the cut test, the guard's own): **0 warnings in all 24 compiles** (`fix/fmt_after.log`). **Found beside it:** -O3 `-Warray-bounds` in `find(PneumaticGoal)` (a constant goal index 9 past `positions[3]`, guarded only at run time) — on the pre-fix copy too; fixed by checking `kMaxPneumaticPositions` first; re-probed clean | the probe (not a mutation: a warning, not a behaviour) |
| 9 = 26 | correctness minor; docs minor | `mkdocs.yml` and the regenerated A1 API pages missing from the port list | REAL as a list omission; the premise "hand-edited" is not: the nav lines sit inside `BEGIN/END GENERATED API NAV`, written by `api_doc_tool.py generate` | **FIXED**: the port list now names `mkdocs.yml`'s generated nav, `docs/api/`, the briefing (regenerate, never merge by hand) and the changelog placement against A1's heading (the port's one conflict there) | the port itself (14:50–15:02) |
| 10 | tests, major | the throwing-claimant test passes with the cut's catch deleted (run()'s cancel-all masks it) | REAL | **FIXED (tests)**: a case where scoring CATCHES each throw and keeps waiting — at the first catch (1.00 s) the roller is already at 0 V and CANCELLED; run() then rethrows from its own cancel-all | **R3** (`throw;` in the catch) RED |
| 11 | tests, minor | the bind-time read-fault snapshot untested | REAL | **FIXED (tests)**: one failed read counted before bind, then healthy — no piece: TIMED_OUT "in 1.00s"; a piece: SUCCEEDED at elapsed 0 | **R18** RED |
| 12 | tests, minor | no mode change pinned to clear `driverTicked_` | REAL | **FIXED (tests)** (the stopped run's case): a stale driver tick, each mode change, then pace() ticks exactly once | **R2**, **R13dc**, **R14** each RED |
| 13 | tests, minor | the per-run first-cut log unpinned | REAL | **FIXED (tests)**: the retry test's second run cuts too — its count 1 (reset, not 5) and its own "cut at the deadline" line (2 over both runs); a third run cuts nothing | **R4** RED |
| 14 | tests, minor | the stall-window resets in `run()` and `drive()` invisible | REAL | **FIXED (tests)**: the catapult's motor held stall-grade BY HAND (2.4 A, 0 rpm): a hold-to-run trips after ≥ 20 ticks; let go, pressed again: its own ≥ 20; then two steps, each STALLED at ≥ 0.2 s; four trips | **R7**, **R8** each RED |
| 15 | tests, minor | a switch's `pressedLevel` never exercised | REAL | **FIXED (tests)**: a hand-built spec with `pressedLevel = false` — low YES, high NO, and a step ends on each as asked | **R6** RED |
| 16 | tests, minor | the bare-specs branch of `pieceSensorPlace` untested; a comment claims it | REAL | **FIXED (tests)**: robot y's catapult bound from bare specs, its switch missing — the place and the whole line; the false comment corrected | **R21** RED |
| 17 | tests, minor | any re-configure empties `compiler_depend.make`; a `build.make` mutation run links stale objects | REAL (the 10:03 run had met half of it) | **FIXED (the process)**: driver v3 runs the `depend` step before every build and refuses a verdict unless EVERY object depending on the mutated file was recompiled (INVALID otherwise); the binary deleted after every run; every gate run ends with a CLEAN full build. It caught its own parser bug as INVALID, never as a verdict (above) | 26 verdicts, each with its object count |
| 18 | tests, minor | the flywheel's Hold-at-0-V braking unpinned | REAL | **FIXED (tests)**: Hold decays below Coast and equals Brake | **FWHOLD** (Brake only) RED |
| 19 | docs, major | guide 10 says nothing runs a mechanism; the prose review logged it "all still true" | REAL — and a second sentence the reviewer did not name (:533 "nothing reads it yet"), and two more beyond guide 10 (faq :24-25, glossary "read fault") | **FIXED**: guide 10's box and read-fault paragraph, the faq, the glossary entry; the 01:26 entry's claim marked WRONG in place; the grep re-run case-insensitive (below) | the prose review below |
| 20 | docs, minor | the glossary's run guard omits the mechanism cut | REAL | **FIXED**: the same clause README carries; and guide 17's abandon paragraph (found by the `run guard` grep) | — |
| 21 | docs, minor | guide 11 states A4 behaviour as current; TIMED_OUT too narrow | REAL | **FIXED**: counted done in `StepReport::countedDone` terms, "from the core's auton steps on (its fourth part)", today's `then()` rule said; TIMED_OUT covers no-condition and no-stall; MECHANISM_STALLED's held-button clause qualified; `its service was destroyed` added to the CANCELLED brackets | — |
| 22 | docs, minor | "one pace late" claimed for the disable | REAL — MEASURED (`fix/probes/f22.cpp`): deadline returned 1.0100 (0.01 s late), disable 0.5200 (0.02 s), 0.1200 V·s past, both bounds | **FIXED**: the changelog says one pace after the deadline, two after a disable (and why); the disable test tightened to the measurement (0.52 s ± 0.5 %, 0.12 V·s ± 1 %) | **M8r** (the ABANDON cut removed, 15:30) RED 2: the tightened case and the in-end-action abandon |
| 23 | docs, minor | "a pneumatic keeps its level through a disable" without HA-157 | REAL | **FIXED**: "is sent nothing on a disable, so it keeps its level ([HA-157])" *(**superseded** — that wording read as a fact about the whole library and contradicted the guard's cancel-all; re-scoped to the service's mode changes by the verification round, V2)* | — |
| 24 | docs, minor | the changelog never says bind() REFUSES an Arm/Lift/Flywheel | REAL | **FIXED**: the preamble says so | — |
| 25 | docs, minor | the brief's cited lines not re-verified in the log | REAL | **FIXED**: the entry below | — |
| 27 | docs, minor | REFUSED's route-marker half dropped without a delta | REAL | **LOGGED: A2-D18** and its "For MECH-A4" hand-on | — |
| 28 | docs, minor | a section header's time not a `date` reading | REAL | **FIXED**: "01:26–01:41 (reconstructed from the file's mtime 01:41:24; no `date` was taken …)" | — |
| 29 | docs, minor | the HA total and next free number are relative to the A1 snapshot | REAL, and not this tree's to change | **FOR THE COORDINATOR** (below) | — |

- **The brief's cited lines, re-checked at `85e0285` (finding 25)** — every §0.2 citation A2's code depends on, read
  at `bdcbc7c` and at `85e0285`: `hal/mechanism.hpp` `ICancellable` :101, `IMechanism` :125, `applySafeState` :153,
  `tryClaim(claimant)` :178, `MotorMechanism` :209 — unchanged; **`MotorMechanism::motors()` :282 → :295 and
  `PneumaticMechanism` :295 → :313** (A1's per-line pneumatic and `meanPosition()` above them).
  `manipulation/mechanism_outcome.hpp` :33, :50; `manipulation/stall_detector.hpp` `StallConfig` :37;
  `sequence/run_guard.hpp` the waits ~:92, "FREEZES NOTHING" :163, `RunGuardReport` :241, `pace()` :312, `run()`
  :473, `cutActiveMotion` :653, `safeMechanisms` :701; `teleop/teleop_loop.hpp` the tick order :18-59 — all exact.
  A1's review changed none of these files (`git diff 85e0285 bcb1cdb --stat` names none of them).
- **The prose review for this round (pace rule 4 — every grep RUN, every hit READ, before this line); scope README.md
  and docs/**/*.md minus `docs/api/` and `docs/internal/chunks/`, CASE-INSENSITIVE this time:**
  - `nothing runs|nothing reads|next parts?|later parts?|core's service` → guide 10 :406 and :535 (this round's
    text), :511 (a marker's id is A4's — true); **faq :24-25 STALE, FIXED**; **glossary :172 ("read fault … its next
    part") STALE, FIXED**; the register :126-127 and :1114-1115 (A1's dated "In this part" paragraphs, each already
    followed by its "since MECH-A2" sentence — history, stays) *(**WRONG** for :126-127 — only :1114-1117, the
    section preamble, carried the sentence; the top-of-register paragraph had none and A2's HA-183 entry after it
    says nothing of the switch. Found by the verification round, fixed there: "Verification-round findings and their
    disposition", V1)*, :132 and :149 (A3's, A5's — true); cookbook 06 :202
    (A4's — true); the changelog's A1 entry (dated — stays); vexbuilder-integration, the master plan, roadmap :1921,
    the changelog's R5a / D-lines (other subjects).
  - `\brun guard\b` over the guides, cookbook, faq and README → **guide 17 :154 STALE BY OMISSION, FIXED** (the
    abandon also cancels every claimed mechanism operation in the same tick); glossary :186 FIXED (finding 20); guide
    11 :291/:316, guide 13 :18/:203, cookbook 03 :8/:109/:199, cookbook 06 :103/:112, guide 17 :39/:179/:199/:289 —
    read, true.
  - `two different mechanisms|conflicting\(|one mechanism's sensor|seesPiece|stopsSeeingPiece` → the register's
    HA-184 *Source* (true) and the roadmap's F11 row (another sense) — the condition words are taught only in the
    changelog (updated) until MECH-A4's steps.
  - `service (is |was )?destroyed|outlive the service|MechanismService` → guide 11 :290 (this round's).
  - `where the command stopped|holdStiffness|ArmPlant|FlywheelPlant|mechanism_plant` → **HA-183's *Claim* (:1186)
    amended**: Hold captures at the END of the last commanded step (finding 2); the changelog's and the roadmap's
    plant mentions — true.
  - `pace late|one pace|two paces|0\.02 s` → guide 09 :331, guide 14 :283, guide 12 :201 — all about the DEADLINE
    (one pace — measured, true) or "one pace later" after the cut (true for both); the changelog (this round's).
  - `keeps? (its|the) level|drop what it holds` → cookbook 06 :176/:212 (the driver toggle and HA-157 — true); the
    register's HA-157 row and entry (its *Source* names the service's two mode changes, which still carry the tag).
  - `TIMED_OUT|counted done|auton goes on|auton stops` → guide 11 only (this round's).
  - `bind\(\)|refuses a list|power and pneumatic` → the changelog (this round's), guide 10 (this round's), the register
    :1174 (A2's paragraph, true), roadmap :1727 (true); cookbook 06's `bind()` is T4's driver bindings (another
    function).
  - **Pace rule 7, this round:** public documents moved — the changelog, guides 10, 11, 15 (glossary) and 17, the faq,
    the register (HA-183's claim), `docs/api/` (regenerated); each edit one sentence to one paragraph, every one a
    review finding or a hit of its grep. **Not touched, deliberately:** README (its guard sentence already carries
    the cut), guides 09/12/13/14 (their "one pace" is the deadline's — measured true), the cookbook (A4's recipes).
- **For the coordinator (this round's):**
  1. **Finding 29, the merge:** this tree's register reads "7 of 174 settled", its briefing "174 registered … next
     free HA-189" — counted on `bdcbc7c` + A1 + A2. `shulib-v2` (`fd36e07`) already reads 174 with HA-189 registered
     and HA-190..191 held: after A2 merges, **175 registered, next free HA-192**. Take main's register header lines,
     apply A2's HA-183 row, entry and "MECH-A2 … added ONE" sentence on top (its running count "With it, 174" becomes
     175), and REGENERATE `PROJECT-BRIEFING.md`.
  2. **`mkdocs.yml`'s nav and `docs/api/` are generated** — regenerate after the merge (the port list says so).
  3. **The mutation driver lesson (finding 17) is worth carrying into every later run's driver:** `depend` first, and
     no verdict unless every dependent object was rebuilt — counted from the consolidated depend file with only
     INDENTED lines continuing an object's list.
  4. **For MECH-A4 (new): A2-D18** — REFUSED's route-marker half.

### 15:30–15:55 — every gate on the FINAL tree *(`date`: Thu Oct 15 03:56:24 PM EDT 2026)*
- One comment changed after the campaign (15:30): the step line's "the longest real line is under 200 characters"
  (never measured) replaced by its provable bound (bracket ≤ 177, line ≤ 254, buffer 320). A comment only — no
  mutation is owed; the first final-gate run was stopped at its build for it and restarted (`fix/final/`).
- **`fix/gates.sh final all`, 26 of 26 rc 0:** configure; `api_doc_tool.py generate`; briefing generate; CLEAN build,
  **225 objects**, rc 0; **suite 2413/2413 passed, 2,790,169 assertions, 3 skipped**; `-tc='MECH-A2*'` **92 cases, 2141
  assertions**; briefing generate (binary present) + **check "generated block current"** + self-test (24 cases);
  **harness 200/200** ("compile-refusal TUs: 200; their CTests: 200"); api_doc_tool self-test, **check-coverage "4623
  public entities across 179 headers, all documented"**, check-fresh (silent), check-examples "537 quoted lines, all
  verbatim", check-removability; doc_staleness_audit self-test + "clean (29 live docs)"; the PROS-free and sim-free CI
  greps; **ARM header gate: 218 headers, rc 0**; no `PROBE_` / PROTOTYPE / "(proposed)"; C7 removability clean; the
  register's direction 1 clean; **`src_build_gate.py check`: PASS** (7 builds link, 11/11 TUs each, 0 disallowed
  warnings); **`self-test`: OK, 211 detector cases** (949 s); **`git diff --stat 85e0285 -- src/ tools/` and
  `git status --short src tools Makefile` EMPTY.**
- After it: every file the self-test plants into `cmp`-equal to `bcb1cdb`'s; `service.hpp`, `until.hpp`,
  `sim/mechanism_plant.hpp`, `sequence/run_guard.hpp` each ALONE through the ARM compile (-Werror -Os): rc 0;
  `fix/fmt_probe.sh` over eight TUs (the four service tests, the until and plant tests, the cut test, the guard's
  own) at -O1/-O2/-O3/-Os: **32 of 32 compiles, 0 warnings** (`fix/fmt_final.log`); no `MUTATION` marker in any file
  the campaign touched, each `cmp`-equal to its last backup before the comment.
- **The tree's state:** NOTHING COMMITTED (`git log -1` = `85e0285`). 124 paths in `git status --short` (53
  modified, 71 untracked) — A2's own change, A1's final change ported (`bcb1cdb`'s files byte for byte), and this
  round's. New this round: 13 test cases across six test files, one harness TU
  (`u_sees_piece_on_a_nameless_declaration.cpp`), `Until::readsOwnSensorToo()`, `~MechanismService`'s body;
  docs: the changelog, guides 10, 11, 15, 17, the faq, the register's HA-183 claim, `docs/api/` regenerated, this
  log. Every finding is dispositioned above; every mutation of the round (27) RED on fresh objects; every gate
  green on the final tree. MECH-A2's §2.1 DoD holds as at 10:26, with the A1 port and this round added.

### 16:15 (2026-10-15) — Verification-round findings and their disposition *(`date` at the first backup: Thu Oct 15 04:21:23 PM EDT 2026; at this entry's writing: 04:33:10 PM)*
- The builder again, same worktree, same base `85e0285`, A1's final (`bcb1cdb`) already ported (14:50). A fresh
  verifier returned TWO findings, both minor, both docs. Each re-checked against the tree before any edit; **both
  REAL**, and the second wider than its suggested fix. Backups of the two code-side files before their edits:
  `scratch/mech-a2/vr2/{service.hpp,mechanism_service_test.cpp}.before`; gate logs `scratch/mech-a2/fix/vr2-gates/`.

| # | Finding | Re-checked | Disposition |
|---|---|---|---|
| V1 | the 15:02 prose review said the register's :126-127 paragraph is "already followed by its 'since MECH-A2' sentence"; it is not | **REAL.** `sed -n 122,130p docs/hardware-assumptions.md`: A1's top-of-register paragraph ends "nothing reads a switch or homes a mechanism yet (the core's service, its next parts). With them, 173." and the next text is A2's own "MECH-A2 … added ONE — HA-183" entry, silent on the switch. Only the "MECH-A1's additions" section preamble (then :1114-1117, now :1113-1119) carried the sentence; the 01:26 entry said exactly that (only the preamble gains it), and the 15:02 entry mis-stated it | **FIXED.** The section preamble's sentence, word for word, after "With them, 173." (:127-129): "*(Since MECH-A2, 2026-10-15, the service reads a limit switch used as a PIECE sensor — HA-184 matters from there; homing, and so HA-186 and the home-switch half of HA-184, from MECH-A3.)*" — each half re-verified first: `service.hpp` :226-227 reads `pieceSwitch->state() == spec_->pieceSensor.pressedLevel` (tagged HA-184); the brief's §2 A3 row (:2127) owns "every homing moment". The 15:02 entry's false claim marked **WRONG** in place (pace rule 4); the touch of A1's paragraph added to "A2 edits to A1 files (for the port)" |
| V2 | finding 23's new changelog wording "a pneumatic is sent nothing on a disable" reads as a fact about the library and contradicts the guard's cancel-all | **REAL — and the suggested replacement ("the mode changes send a pneumatic nothing") is itself false.** (a) `RunGuard::run()` performs `cancelAll()` → `safeMechanisms()` → `applySafeState()` on EVERY listed mechanism at the end of EVERY run (its steps 3 and 5 — not only on an abandon), and `competition_program.cpp` :695-701 lists every present mechanism, robot two's claw included; guide 17 :157-158 says so. (b) A2's own cut: `Slot::cancel()` (`service.hpp` :268) → `stopBehaviour(true)` applies the safe state for EVERY kind — pinned by "MECH-A2 pneumatic: the guard's safe state is the declared safe POSITION on every line -- claimed mid-step and unclaimed". (c) `enterDriverControl()` sends a pneumatic its START on the FIRST entry (`PneumaticSlot::onDriverEntry`, :758-761) — pinned by "MECH-A2 driver entry: the claw goes to START on the FIRST entry only …". Only `leaveDriverControl()` and `enterAutonomous()` send a pneumatic nothing (`stopBehaviour(kind != Pneumatic)`, :905, :922) | **FIXED** (changelog :57-63): "None of the three puts a pneumatic in its safe state: `leaveDriverControl()` (a disable) and `enterAutonomous()` send it nothing, and `enterDriverControl()` only its START position, on the first entry (T4's ruling: the claw starts closed) — so through a disable the service leaves a pneumatic at its level ([HA-157]). The run guard is another matter: `run()`'s cancel-all at the end of every guarded auton still puts every LISTED mechanism, a pneumatic included, in its safe state, as before, and its new cut (below) puts a pneumatic whose step has not given its verdict yet in its safe position too." **One clause drafted and DROPPED:** "only while nobody holds its claim" (`onDriverEntry`'s `!mech_->claimed()`) — unreachable: `enterDriverControl()` runs `releaseLeftover()` and `stopBehaviour()` on the same slot just before it, which between them release every claim, so the clause would describe no behaviour (its removal is an equivalent mutant; no test is owed). The 15:02 table's finding-23 row annotated **superseded** |

- **Beside V2, the same over-general phrase in A2's own code (fixed, no behaviour changed):** the tick's
  released-claim comment (`service.hpp` :1121-1127) said a pneumatic keeps its level "as every mode change keeps
  it" — now "no mode change makes one safe either (driver control's FIRST entry sends its START, nothing else
  does)"; the test case at `mechanism_service_test.cpp` :518 renamed "… KEEPS its level -- no mode change makes one
  safe either" (run under its new name: 1 case, 11 assertions; the old name now matches nothing). A `//` comment
  and a test NAME: no `///` changed, no mutation owed; the full rebuild below recompiled both.
- **The prose review for this round (pace rule 4 — every grep RUN, every hit READ, before this line); scope README.md
  and docs/**/*.md minus `docs/api/` and `docs/internal/chunks/`, CASE-INSENSITIVE:**
  - `nothing reads|reads? a switch|homes? a mechanism|reads? a limit switch` → the register :126-129 (V1, FIXED) and
    :1113-1119 (the section preamble under :1111, already carrying it), :1131 and :1137 (HA-184's *Source* and *Settle* — true);
    vexbuilder-integration :140/:213/:259/:737, roadmap :1921, the master plan :692, the changelog :611,
    build-order :1744 (`.vexbot`, `.shupaths`, the scale, the wire — other subjects).
  - `sent nothing|commands? nothing|keeps? its level` (the verifier's grep, widened) and `through a disable|on a
    disable|keeps what it holds|at its level` → the changelog :53 and :60 (this round's text); the changelog :1060 and
    :1076 (T3's hold and toggle bindings — driver control, true in their scope), roadmap :395/:1837 and HANDOFF :462
    (T3's toggle — driver control), guide 17 :145/:277 (the driver loop's end — true), HANDOFF :315 and build-order
    :2155 (`autonomous()` before COMP — history); **HA-157's row :335, its *Source* :2075/:2081, cookbook 06 :212-213
    and guide 14 :233 — pre-A2, over-general since COMP: for the coordinator (below).** A2's own HA-157 sentence
    (:2077-2078, "the mechanism service keeps the same rule … `leaveDriverControl()` and `enterAutonomous()`") names
    the service and its two mode changes — true as scoped.
  - The same greps over A2's `///` text (`docs/api/service.md`, `run_guard.md`): `enterAutonomous()`'s and
    `leaveDriverControl()`'s docs and the file banner's released-claim rule — each scoped to its own call, true.
  - **Pace rule 7, this round:** public documents moved — the changelog (one paragraph of A2's entry), the register
    (A1's top paragraph, one sentence). `docs/api/` regenerated (no `///` changed; `check-fresh` silent).
- **For the coordinator (unnumbered; pre-A2 text, not this run's lines):** HA-157's *Source* (:2075, "the library
  commands NOTHING to a pneumatic claw on a disable"), cookbook 06 :213 ("The library commands nothing on a
  disable") and guide 14 :233 ("because the library commands the claw nothing on a disable") are over-general since
  COMP (2026-10-02): in "shulib Competition", a disable DURING a guarded auton makes the watch abandon the run, and
  `run()`'s cancel-all then writes the claw's SAFE level while the brain is disabled — a write HA-157's half (2)
  governs, and a consumer HA-157's *Source* does not name (guide 17 :157-158 already states the behaviour). Not a
  new hardware belief: an existing one with an unnamed consumer. A2's cut adds no new write there (the cancel-all
  that follows reaches every listed mechanism anyway).
- **Every gate on the round's tree** (`fix/gates.sh vr2-gates all`, 16:22–16:47, **26 of 26 rc 0**): configure;
  `api_doc_tool.py generate`; briefing generate; CLEAN build, **225 objects**, rc 0, 0 warnings; **suite 2413/2413,
  2,790,169 assertions, 3 skipped**; `-tc='MECH-A2*'` **92 cases, 2141 assertions**; briefing generate (binary
  present) + check "generated block current" + self-test (24 cases); **harness 200/200**; api_doc_tool self-test,
  check-coverage (4623 entities, 179 headers), check-fresh (silent), check-examples (537 lines), check-removability;
  doc_staleness_audit self-test + "clean (29 live docs)"; the PROS-free and sim-free greps; **ARM header gate 218
  headers, rc 0**; no `PROBE_` / PROTOTYPE / "(proposed)"; C7 clean; the register's direction 1 clean;
  **`src_build_gate.py check` PASS** (7 builds link, 0 disallowed warnings); **`self-test` OK, 211 detector cases**
  (884 s); **`git diff --stat 85e0285 -- src/ tools/` and `git status --short src tools Makefile` EMPTY** (a
  `src/programs/bench_r3a.cpp` seen modified at 16:32:59 mid-run was the self-test's own plant, restored by it — the
  final gate's check is after it). **After this entry was written** (16:48): api generate, briefing generate +
  check, check-fresh / -examples / -coverage / -removability, the staleness audit, C7 — all rc 0
  (`scratch/mech-a2/vr2/post-log-gates.txt`). One slip: that re-run's per-command output went to a throwaway file in
  the session's /tmp scratchpad (against the run's scratch rule), deleted at once; its verdict lines are in the
  scratch file above.
- **The tree's state:** NOTHING COMMITTED (`git log -1` = `85e0285`); 124 paths in `git status --short` (53
  modified, 71 untracked — the same paths as at 15:55). Changed this round: `docs/hardware-assumptions.md` (V1),
  `docs/changelog.md` (V2), `include/shulib/mechanism/service.hpp` (one `//` comment),
  `test/mechanism_service_test.cpp` (one case name), this log; `docs/api/`, `mkdocs.yml`'s generated nav and the
  briefing regenerated. Both findings dispositioned; no behaviour changed, so no mutation was owed.

### The coordinator's merge verification — MECH-A2 (2026-10-15 21:50–22:45, main `2506067` + the squash of `mech-a2`)
- **Audit before the merge:** the 37 mutation logs under `scratch/mech-a2/mut/` all say restored, and no mutation's new
  text is in its file; no gate plant; no stray root file; `git diff --stat -- src/ tools/` empty. The transport commit
  `a4cb685`, then `git merge --squash mech-a2`.
- **The merge base was not A1's final tree** (A2 was built on the snapshot `85e0285`, whose parent is `bdcbc7c`), so git's
  own three-way merge saw A1 on both sides: 15 files conflicted. Every document was re-merged with **`bcb1cdb` (A1's
  final worktree) as the base** — against it A2's changes are pure additions (this log +741 −0, the changelog +95 −0) —
  and every file git merged by itself was checked to equal that re-merge byte for byte (it did). Hand-resolved: the
  changelog (A2's entry first, dated as merged, then SER1's); the register (**177 rows, "7 of 177 settled", next free
  HA-192** — finding 29's arithmetic plus SER1's HA-190/191; A2's paragraph after SER1's, "With it, 177"; A1's paragraph
  given A2's italic note); REMAINING-WORK row 12 (A1's date from main, "A2 LANDED" from A2, word by word);
  build-order's `Next:` (A2 LANDED; NEXT A3 — and the line's stale "the sensors come after G3", corrected to the team
  lead's ruling of 2026-10-14: the distance sensors and the camera's objects BEFORE the panel); this log (A1's merge
  record, then A2's section). `docs/api/`, `mkdocs.yml`'s nav and the briefing regenerated (findings 29 and the port
  list's note 2).
- **Gates on the merged tree** (`.claude/coordinator/logs/mech_a2_merge_verify.out`): host build with the doc gates;
  suite **2,445 | 2,445 passed, 3 skipped** (A2's 2,413 + SER1's 32); `src_build_gate.py` self-test OK and check
  **PASS, all 7 builds**; ARM header gate over **218** headers clean; CI greps empty; api check-fresh, the briefing,
  removability and the staleness audit clean; the compile-refusal harness **200 of 200** (105 s).
- **The coordinator's hand mutations** (`muts_mech_a2_main.json`, on `mechanism/service.hpp`, `sequence/run_guard.hpp`,
  `mechanism/until.hpp`; each on a fresh link, the whole suite under a 4 GiB / 300 s cap, restored and hash-checked):
  **13 of 15 caught at first** — MB1 (a near reading with confidence 0 counted), MB2 (the read-fault rule never firing),
  MB3 (an optical NaN read as "no piece"), MB6 (an asked stall reported STALLED), MB7 (`enterAutonomous` releasing the
  leftover claim after the safe state), MB8 (a released motor mechanism left at its last voltage), MB9 (a pneumatic's
  START on every driver entry), MB10 (the read-fault counts never snapshotted), MB11 (the abandon not cutting a claimed
  mechanism), MB12 (a cut counted, not cancelled), MB13 (the cut count not reset per run), MB14 (a throwing cancel
  swallowed), MB15 (two joined times keeping the later) RED. **Two SURVIVED — real gaps, now closed:**
  - **MB4** (`blind > 0` → `blind > 1`): nothing pinned the boundary — ONE blind sample in a whole step. New case
    "ONE blind sample in a whole step is enough for UNCONFIRMED" (`test/mechanism_service_evidence_test.cpp`):
    "no evidence on 1 of 101 reads".
  - **MB5** (`elapsed >= time` → `>`): the case "a time EQUAL to the bound ends SUCCEEDED by time" never landed on the
    equality — with 0.01 s ticks the elapsed time always overshoots it (0.01 has no exact binary form), so it passed
    either way. New case on an exact 1/128 s clock (`test/mechanism_service_power_test.cpp`) that also asserts the
    elapsed time EQUALS the bound, so the edge is provably reached.
  Re-run (`muts_mech_a2_main_b.json`): **MB4 and MB5 RED**, each at the new case's lines; restored tree **2,447 | 2,447**.
  **15 of 15.** The lesson is carried into MECH-A3's run: an equality edge must be tested on an exact clock.
- **Hand-ons:** A2-D18 (REFUSED's route-marker half) is MECH-A4's; the unnumbered HA-157 note (three pre-A2 sentences,
  "the library commands nothing on a disable", over-general since COMP) is fixed in its own documentation commit after
  this one.

---

## MECH-A3 — position and speed

- **Started:** Thu Oct 15 10:44:23 PM EDT 2026 (`date`).
- **Tree at start:** `fbaf729` (main: MECH-A1 `fd36e07`, SER1 `2506067`, MECH-A2 `98392b2` merged and verified, then
  the HA-157 scope note), clean, in the git worktree `.claude/worktrees/mech-a3` (branch `mech-a3`) — NOT the main
  working tree.
- **Go:** the team lead, 2026-10-08: "Accept all, go" — N1–N13 all as recommended (a), the runs strictly in order,
  each after the last LANDS; and 2026-10-15: "keep going with the original roadmap but move faster — that does not
  mean skip steps". The coordinator's workflow launched MECH-A3 ONLY. Do not commit; no `pros` command; no
  `/dev/ttyACM*`. Mutation backups and probes under `.claude/coordinator/scratch/mech-a3/` (never `/tmp`, never the
  repository root). **Memory safety (2026-10-15):** every test binary — the suite, a filter, every mutation, every
  looping probe — runs as `(ulimit -v 4194304; timeout 600 <cmd>)`; a mutant killed by the cap or the timeout counts
  as CAUGHT (said which).
- **Scope:** §2's A3 row — §1.7's POSITION and SPEED behaviours in `mechanism/service.hpp` (every homing moment and
  its zero log, the move and hold loops, reached in units, both limit rules with the closed-loop soft stop, the
  clamps, N2's hold-where-stopped, the stall watch on every behaviour, the speed loop); N7's starting values in
  `kinds.hpp` and the library formatter of their boot line (beside `roleLine`); §1.12's arm, lift and flywheel models
  tuned in `sim/mechanism_plant.hpp`; `bind()` STOPS refusing Arm/Lift/Flywheel (A2-D5's precondition — removed with
  its text and every doc that says so); §3.3's tests and mutations; §4.1's A3 documents (changelog, `docs/api/`,
  HA-182 ONLY, the guide's NEW mechanisms chapter); §2.1's DoD. NOTHING of A4–A6: no auton steps, no driver loop, no
  `src/` or `tools/` file (`git diff --stat fbaf729 -- src/ tools/` EMPTY at the end).

### 22:44 — start
- Read `docs/internal/RESUMING.md` (standing rules, pace rules 1–8), the brief in full (the go note, the amendments —
  which override the body —, N1–N13, §0–§9, "Found while reading"), and `## MECH-A1` / `## MECH-A2` above (A2's deltas
  A2-D1…D21, its hand-ons: A2-D5 — lift `bind()`'s refusal; A2-D6 — the home switch's read-fault half and case S;
  A2-D18 is A4's, not this run's; the merge record's MB5 lesson — an equality edge on an exact binary clock; finding
  17's — a mutation verdict only when EVERY dependent object was recompiled). This heading appended before any code.

### 22:44–23:05 — the baseline, and the overlay's A3 slice read against the tree *(`date` 23:05)*
- **Baseline at `fbaf729`** (`cmake -S test -B build/test`; `rm -f build/test/shulib_tests`; `briefing_status.py
  generate`; the full doc-gated build at -j8, 36 s wall; the suite under `ulimit -v 4194304; timeout 600`): **2447/2447
  passed, 2,792,078 assertions, 3 skipped** — main's merged numbers (the coordinator's MB4/MB5 cases included).
  Driver: `scratch/mech-a3/build.sh` (logs `scratch/mech-a3/logs/base.*`).
- **Read in the overlay** (`proposed4/shulib/mechanism/service.hpp` :255-330 the two slots' surface, :720-1052 their
  bodies; `sim/mechanism_plant.hpp`; `kinds.hpp`'s starting values), the probes `p5/probe2_position.cpp`,
  `probe2_speed.cpp`, `adv_control.cpp`, `adv_softstop.cpp`, `adv_extra.cpp`, `adv_stallsig.cpp` and their `out5/`
  outputs. **The overlay's base is not this tree's:** A2 rewrote the slot (A2-D1: every body inside its class, the
  `detail::SlotHost`; `beginStep(bound, t)` / `finishStep(t, …)`; `stopBehaviour` / `resetBehaviour`), the plant
  (A2-D3: Hold and Brake implicit; Hold captured at the END of the commanded step), and the starting values already
  live in `kinds.hpp` (A1's `detail::positionStart`, the flywheel's in `Flywheel::spec()`), so A3's slice is
  re-written onto A2's architecture, not copied — the deltas below (A3-D1 …).

### 23:05–23:17 — the behaviours written, the plant extended (no constant of it tuned — A3-D19, the review's correction of this heading, which said "tuned"), the tests, the suite green *(`date` 23:17:29)*
- **Code** (each file's banner says what it holds now):
  - `include/shulib/mechanism/service.hpp` — `PositionSlot` and `SpeedSlot` (the brief's §1.6 surface: `moveTo`,
    `moveToPreset`, `holdHere`, `home`, `jog`, `release`, `homed`, `position`, `target`, `softStops`, `jointDeg`;
    `spinUp`, `spinUpPreset`, `stop`, `spinning`, `rpm`, `atSpeed`); `Slot::homeSwitchPressed()` with the home
    switch's half of the read-fault rule and its snapshot; `Slot::stalledAfterStep()`; the three HA-182 constants
    (`kHoldIntegralLimitVolts` 3 V, `kSpeedIntegralWindow` 0.10, `kSpeedIntegralLimitVolts` 3 V); the service's
    `position(i)`, `speed(i)`, `find(const Arm&)`, `find(const Lift&)`, `find(const Flywheel&)`,
    `find(const PositionGoal&)`, `find(const SpeedGoal&)`; **`bind()` no longer refuses an Arm, a Lift or a
    Flywheel** (A2-D5's precondition and its text removed) and checks their devices instead (an angle sensor
    exactly as declared, a home switch exactly when homed by switch, a home switch's count only with a switch).
  - `include/shulib/mechanism/kinds.hpp` — the starting values factored so a boot line can name them:
    `detail::unitsPerOutputDeg()`, `detail::speedStart()`, `positionStartingValues()`, `speedStartingValues()`;
    HA-182 tagged at each; the numbers themselves unchanged (A1's `positionStart`).
  - `include/shulib/mechanism/wiring.hpp` — `startingValuesLine()` beside `roleLine()` (N7's boot line).
  - `include/shulib/mechanism/spec.hpp` — refusal **70** appended (A3-D3 below) and HA-182's wording.
  - `include/shulib/sim/mechanism_plant.hpp` — `RotationMount` (an angle sensor absolute within one turn at
    power-up, HA-186's belief modelled, either way round), `SwitchMount` (a limit switch at an angle, at a chosen
    pressed level — HA-184's belief the test's to choose), `setWedgeDeg()` / `clearWedge()` (a stop BELOW the joint:
    the piece wedged under the arm, D10).
- **The A3 probe first** (`scratch/mech-a3/probes/p2.cpp`, output `p2.txt`): the brief's P2a, P2b, P2c and P3 re-run on
  THIS code and A2's plant — **reproduced to the hundredth**: P2a 143/144 SUCCEEDED in 0.69..3.15 s, overshoot ≤ 7.22°
  (≤ 3.59° with kVx ≥ 1), final ≤ 2.55°, the one miss Green 7:1 kVx 1.00 kS 0.6 kG 2.4 → 90 TIMED_OUT at 2.38 s; P2b
  1.66 / 1.80 / 2.41° worst, −1.25 / −1.18 / −0.86 after 2 s; P2c STALLED at 0.65 s and 1.00 s; N2 (a 10 s signature)
  TIMED_OUT at 3.01 s then mean 1.68 V, ≤ 0.37 A, held at 59.7°; P3 20/27, 20/21 reachable, the same table row for row.
  ONE number moved: P2b's "nobody ticking" control sags **2.87°** (the brief 3.05°) — A2-D3's implicit Hold. The
  ablation probe (`abl.cpp`): held level under plant kG 1.2 vs the controller's 1.6, kS 0.3 — **+3.20° without the
  hold's integral, +0.001° with it**.
- **Tests** (new: `test/mechanism_position_rig.hpp`, `test/mechanism_position_test.cpp` 29 cases,
  `test/mechanism_speed_test.cpp` 7, 3 plant cases appended to `test/sim_mechanism_plant_test.cpp`; changed:
  `test/mechanism_service_test.cpp` (A2's two "MECH-A3" bind refusals replaced by a pointer to their successors),
  `test/mechanism_spec_test.cpp` (refusal 70's cases); new harness TU `test/compile_refusals/s70_absolute_homing_whole_
  turn.cpp`). Thresholds are the probe's numbers plus the brief's stated margins, quoted beside each.
- **Fresh build** (`build.sh b1`): the doc gate stopped it first (`docs/api/` stale — regenerated; the build script now
  runs `api_doc_tool.py generate` every time). First suite: **7 cases red**, each diagnosed — 6 were the TESTS' errors
  (a verdict's error read one world step late; a never-homed arm "not moved" though it sags under its own Hold; robot
  y's wrist's starting line mis-computed by hand — 3:1 Green is 300 deg/s, not 450; a bind case that tripped an
  earlier precondition; a reversed-sensor case wired to read +40; a shot read one tick early); **one was the code's
  (A3-D9)**: on a homing step's success the slot zeroed and switched to HOLD but commanded nothing that tick, so the
  motor kept the homing push one more tick past its zero — it now commands the hold in the same tick. The switch-home
  test then asserts what is physical: the zero within one tick's travel of the switch (measured 0.23°), and held at it
  after 3 s (measured −0.06° at 2.5 s).
- **Suite (`b3`): 2486/2486 passed, 2,795,436 assertions, 3 skipped**; `-tc='MECH-A3*'` **39 cases, 3193 assertions**.
  **Harness: `ctest -R compile_refusal` 201/201** (95 s; A1's 195 + A2's 5 + s70).

### 23:17–23:40 — EVERY mutation run on fresh objects; four new tests the green ones asked for *(`date` 23:40)*
- **Driver:** `scratch/mech-a3/mutate.py` — MECH-A2's v3 (the coordinator's finding 17 carried: the suite target's
  `depend` step BEFORE every build, and no verdict unless EVERY object depending on the mutated file was recompiled
  — counted from the consolidated `compiler_depend.make`) plus two additions: the LINK checked (the binary newer
  than the mutated file, else INVALID) and the suite run under the memory cap (`ulimit -v 4194304; timeout 600`;
  a mutant killed by either would count CAUGHT, said which — none was). Multi-site mutations allowed (X15d). Lists
  `mutlist.py`, `mutlist2.py`, `mutlist3.py`; one log and one byte-identical backup each in `scratch/mech-a3/mut/`;
  every file restored byte-identical (47 of 47 `restored=True`) and its timestamp bumped. Filter `-tc='MECH*'`
  (every MECH case, A1's and A2's included).
- **The brief's §3.3, each RED on a fresh build** (the failing cases named):
  - **M14** the hold loop's kI 0 → RED 11 (P2a, P2b, the integral ablation, P2d ×2, the bottom soft stop, case C,
    D4, P2e, P2h, the past-the-top clamp).
  - **M15** the speed integral's 10 % window removed → RED 2 (P3's overshoot, the window's ablation).
  - **M16** the soft stop removed → RED 3 (D2 at both limits, the bottom soft stop, case C) — **at BOTH limits:**
    **M16top** (the top alone) RED (D2's UP pushes); **M16bottom** (the bottom alone) RED 3 (D2's DOWN, the bottom
    soft stop, case C). *(The first M16 was red only at BUILD — `-Werror=unused-variable` — no verdict; re-written to
    compile, `if (false && (up || down))`, and re-run. X1wrap the same.)*
  - **M17** a move before homing proceeds → RED (the never-homed case's NOT_HOMED).
  - **M18** a computed target not clamped → RED (P2d: the clamp line and the target).
  - **M19** absolute homing zeroes where it is → RED (P2h).
  - **M20** the gravity term's sign flipped → RED 8.
  - **M21** N2 reverted (a timed-out move holds its unreached target) → RED 2 (P2c's N2 case: the current; the
    past-the-top case).
  - **M36** "reached" in converted degrees → RED (D4: the table-top).
  - **M39** the hold's stall watch removed → RED (case A).
- **The run's own** (each RED): X2sign (a reversed absolute sensor's sign ignored — the overlay's formula; a sensor
  reading 180 HIDES it, 2 × 180 being a whole turn, so a case reading 100 was added), X1wrap (the turn never chosen),
  X3rests (home() re-zeroes a rests-at-stop arm where it stands — the overlay), X4nothomed (a NOT_HOMED move leaves a
  jog at its last volts — the overlay), X5jog0 (a push before homing with no unhomedVolts holds the claim at 0 V —
  the overlay), X6swwatch (no stall watch on a homing push by switch), X7fresh / X22snapshot (the home switch's
  read-fault rule / its snapshot), X8tick (the homing success's same-tick hold, A3-D9), X9clamp (a hold target not
  clamped), X10seq (a stall after a step bumps no command number), X11spinwatch (a flywheel watched only during its
  step), X12spinclamp, X13stopdist (the soft stop entered AT the limit, not at the stopping distance — RED 2),
  X14profile (below), X15d (below), X17bind (A2-D5's refusal back — RED 35), X18unitsfind, X19goalpreset, X20angle,
  X21movewatch (the MOVE's stall watch removed — RED 3: P2c, case F, P2h's mismatch), X23unhomedcap; the plant's
  P1wrap, P2top, P3wedge; the spec's **S70** (refusal 70 removed — suite RED and **harness RED**
  `compile_refusal_s70_absolute_homing_whole_turn`) and S70b (a spool's travel mis-scaled); W1line (the boot line
  marks the wrong numbers); K1units (a spool's starting scale doubled — RED 3, A1's words test among them).
- **Found GREEN, each answered:**
  - **X14profile** ("reached" stops waiting for the plan's end) was GREEN: with the starting tolerance the arm lags
    its plan and is never inside 1.5° before the plan ends. **A test was added** that can see it — a team's coarse
    `within` 30° and no settle time: the verdict must still wait for the plan (measured: SUCCEEDED at the plan's end
    + one tick). Re-run: **RED**.
  - **X15fresh** (the hold loop not reset at the plan's end) is GREEN and stays green: EQUIVALENT — `moveTo()`
    resets the hold loop too, and the move loop carries no integral, so either reset alone keeps the hold fresh.
    **X15d** removes BOTH (the integral of a loaded hold carried into the next move): measured first on a temporary
    copy (`probes/x14.cpp`: after the move SUCCEEDED the arm rose to 37.6° above its 30° target, against ≤ 0.1° on the
    true code); **a test was added** ("a move after a LOADED hold starts its hold FRESH"); X15d **RED 2**.
  - **M18b** (the output-degree clamp of a move's target removed, M18's position clamp kept) is GREEN: EQUIVALENT —
    the target is already clamped to the travel in its units, and `positionToAngleDeg` is monotonic with the limits'
    degrees computed by the same function, so the degree clamp can never act. Kept as a guard; said so here.
- **Tests added in this round:** the coarse-tolerance REACHED case, the loaded-hold FRESH case, the reversed sensor
  reading 100, "the hold from the verdict's own tick" (`volts > 0` after a switch-home — what made X8tick red), and
  **case F** (a switch reading "pressed" while truly at 60 → homed there → a move to "140" STALLED at 1.70 s at the
  hard stop — the brief's number to the hundredth; HA-184's net) — and the n58 justification in case A (the plant
  never reads more than 2.5 A held at its stop — max 2.500 A — so a 2.5 A or larger signature can never trip, and
  `specRefusal` refuses a 3.0 A one).
- **Numbers re-measured for the documents** (`probes/nums.cpp` → `nums.txt`, the true code): case A 1.66 s at
  ≥ 2.4 A of 30 s (STALLED 1.77 s after the load), case H STALLED at 0.51 s, D2 1.26 / 0.97, 2.06 / 1.76, 4.06 /
  2.12° past, the bottom soft stop 18.32 / 17.94 / 16.06°, a jog into an obstacle STALLED 0.95 s in, case F 1.70 s —
  the brief's every number reproduced.
- **Suite after the round (`b7`): 2492/2492 passed, 2,795,475 assertions, 3 skipped**; `-tc='MECH-A3*'`: 42 cases
  (32 position, 7 speed, 3 plant) + the chapter's 3 (`guide-18a..c`).

### Deltas from the brief (MECH-A3), each with its reason
- **A3-D1** the overlay's `PositionSlot` / `SpeedSlot` (`proposed4/shulib/mechanism/service.hpp` :261-361, :720-1052) were
  RE-WRITTEN onto A2's architecture, not copied: every body inside its class (A2-D1's house style — the doc gate reads an
  out-of-line `X::y(` as an undocumented free function), the `detail::SlotHost`, `beginStep(bound, t)` /
  `finishStep(t, …)` / `stopBehaviour` / `resetBehaviour`. Not one `PROBE_` switch was carried (`PROBE_SETTLE_SPEED`,
  `PROBE_TIMEOUT_HOLDS_TARGET` — the overlay's A3 half; `grep -rn PROBE_ include/` empty), no "PROTOTYPE"/"(proposed)"
  marker. The behaviour is the overlay's except where a delta below says otherwise — and the brief's P2a, P2b, P2c, P3,
  D2, case A, case F, case H numbers came out IDENTICAL to the hundredth (the probe entry above).
- **A3-D2** absolute homing (the overlay's `zeroAbsolute()`, :798-801) computed `joint = sgn·reading + home − s0`, wrong
  for a sensor declared `Reversed` unless the stated reading is 0 (robot y's wrist, `absoluteSensor(0, 180)` reversed,
  would read −390° at −30°), and never chose the reading's TURN (a sensor reading 350 at the arm's 0 put the arm at 20°
  at −340°). Now `joint = home + sgn·(reading − s0)`, the turn chosen nearest the middle of the travel. Both pinned
  (P2h's subcases), both mutations RED (X1wrap, X2sign).
- **A3-D3 — a 70th refusal in `spec.hpp`** (A1's file — the chunk that finds a flaw fixes it there):
  `PositionAbsoluteHomingTravelSpansAWholeSensorTurn` / `position_absolute_homing_needs_a_travel_shorter_than_one_turn_
  of_its_angle_sensor`, appended (its enum value 70 pinned). An angle sensor absolute within ONE turn (HA-186) cannot
  home a travel of a whole output turn or more — a spool lift's rotation sensor turns 5.1 times over 24 in on a 1.5 in
  spool, and the service would pick a turn, maybe the wrong one, and drive into a stop. Checked with a constexpr span
  (`detail::outputTravelDeg`: an arm's limits, a spool's arc, a table interpolated, a linkage ≤ 180° by its formula's
  domain). Its compile-refusal TU `s70_absolute_homing_whole_turn.cpp`; three one-property cases and three accepted
  boundaries in `mechanism_spec_test.cpp`; the test's `last` bound moved to it. **For the coordinator: G3a's validator
  mirrors `spec.hpp`'s list by sentence — it is 70 now** (and A1's counted totals in the plan docs — "69 checks …
  132 named reasons" — are A1's dated numbers; today's are 70 and 133).
- **A3-D4** `home()` on a mechanism that RESTS AT ITS STOP keeps its BOOT zero — SUCCEEDED, `(it rests at its stop at
  boot: its zero is kept)`, held where it is; the overlay re-zeroed it where it stood (the "zero where it stands" class
  the brief rejected for the absolute sensor — X3rests RED). An absolute one is READ AGAIN (its zero log says whether it
  moved). `neverHomed` is a PRECONDITION ("it has no way to find its zero"), not the overlay's REFUSED — A2-D18 keeps
  REFUSED for "another owner holds the claim", and a step that can never be met is refused where it is made (D7's rule).
- **A3-D5** a NOT_HOMED move applies the declared safe state; the overlay (`stopBehaviour(false)` after its state was
  reset) left a running pre-homing push at its last volts (X4nothomed RED).
- **A3-D6** a push before homing with `.unhomedVolts` 0 is a STOP (the safe state, no claim held); the overlay held the
  claim and commanded 0 V every tick (X5jog0 RED). `jog(0 V)` is `holdHere()`.
- **A3-D7** a homing push BY SWITCH is watched by the mechanism's stated signature (N10: "every behaviour … homing by
  STALL keeps its own"), STALLED `(… while homing: its home switch never read pressed)`; the overlay watched none.
- **A3-D8** the zero logs: the brief's lines kept (`homed -- <x> deg from where its encoders read zero at boot`, `homed
  again -- its zero moved <x> deg`), each number SIGNED, to its tenth, never "-0.0"; "angle sensor" for a sensor-read
  mechanism; an absolute home's own line (`homed -- its angle sensor reads 170.0 deg, so it is at 10.00 deg`); the
  re-home Warn's threshold is the tolerance converted on BOTH sides of the home position (at a table's end one side
  converts to 0 — D4's class; the overlay used one side).
- **A3-D9** a homing step's success commands the HOLD on the verdict's own tick (found by this run's own first test
  run: the overlay switched to HOLD but commanded nothing, so the motor kept the homing push one more tick past its
  zero) — X8tick RED.
- **A3-D10** `find(const Arm&)` / `find(const Lift&)` check the units (an `Arm` is never a same-named lift);
  `find(const PositionGoal&)` / `find(const SpeedGoal&)` return the slot only when this robot's mechanism HAS the goal's
  named preset (the pneumatic goal's position-name check, A2's, extended) — X18, X19 RED.
- **A3-D11** surface beyond the brief's verbatim: `SpeedSlot::targetRpm()`; `positionStartingValues()`,
  `speedStartingValues()` (what a spec runs with no `.tuning` — the boot line's comparison); `detail::unitsPerOutputDeg()`
  and `detail::speedStart()` (the starting values factored, unchanged); the three HA-182 constants
  (`kHoldIntegralLimitVolts`, `kSpeedIntegralWindow`, `kSpeedIntegralLimitVolts` — the brief's numbers, now named);
  `Slot::stalledAfterStep()` (protected); `sim::RotationMount`, `sim::SwitchMount`, `ArmPlant::setWedgeDeg()` /
  `clearWedge()`.
- **A3-D12** the boot line (`startingValuesLine()`, in `wiring.hpp` beside `roleLine()`): the brief's example words, plus
  the ramp's time ("in 0.25 s"), fixed digits, each number a team's `.tuning` changed marked "(tuned)", and three
  tails (all starting / some / none). Its numbers come from comparing the spec against `positionStartingValues()`: a
  team that states exactly a starting value reads as untuned — harmless, said here.
- **A3-D13** P2c's N2 case: the brief's hand-built 2.6 A signature is refused at `bind()` since A1 (n58), so the test
  states a VALID signature too slow for the 3 s bound (lasting 10 s) — the same N2 behaviour, the same numbers. The
  brief's "a hand-built 3.0 A signature never trips" is pinned as a property in case A (the plant held at its stop at
  12 V reads exactly 2.500 A and never more; `specRefusal` refuses 3.0 A), not a 30 s run of an unbindable spec.
- **A3-D14** the speed integral's ablation ("from the start it overshoots > 150 rpm") is a REFERENCE loop in the test —
  the window is a constant (HA-182), not a `.tuning` field — run on the same plant with the same gains; M15 (the window
  removed from the library) turns the library's own < 60 rpm assertion red.
- **A3-D15** the rig's encoders read zero where the plant starts (`encoderZeroDeg = startDeg` — a TEST choice, not a
  belief about V5 motors: the service zeroes a rests-at-stop mechanism at whatever its encoders read at bind, and
  nothing in the library assumes where a motor's count starts); the brief's probes left them at the joint's 0. D10's verbatim line uses
  the probes' convention explicitly (`encoderZeroDeg = 0`, so "+25.0 deg").
- **A3-D16** A2-D5's precondition removed WITH its text and its two test lines; `bind()` checks the new kinds' devices
  instead (an angle sensor exactly as declared on a built position or speed mechanism, a home switch exactly when it
  homes by switch, a home switch's count only beside a switch). The A2 changelog entry's "bind() REFUSES …" is dated
  history: annotated "(part 3, above, lifted it)", not rewritten.
- **A3-D17** *(measured at the resumption, 2026-10-18)* `sizeof(MechanismService)` on ARM `-Os` is **9 448 B**, the
  brief's S3 8 048 B; `PositionSlot` 568 B (S3: 536). The 1 400 B are A2's layout (one `std::optional` array of 8 per
  kind: 8 × (568 + 208 + 224 + 136) B plus their flags) carrying A3's larger slots — the A2-D21 bounded log
  buffers live on the stack, not in the slot. Static storage on a brain with megabytes of it: no change made; the
  number is here so A5 (which puts the service in each program) starts from the measured one.
- **A3-D18** *(2026-10-18)* the changelog's part-3 entry no longer names "the brief", "the live log" or "N10" (above):
  a public document a release ships without `docs/internal/` must not lean on it, even unlinked. *(Scope, MECH-A3's
  review, 2026-10-22: the rule covers the public text A3 itself writes — the changelog AND its own new doc comments,
  which reach `docs/api/`. The review found 8 such `docs/api/` lines ("the brief's P3", "N7", "N2", "N10"); the fix
  round reworded them, and `git diff fbaf729 -- docs/api/` now adds no line naming the brief or a decision but one —
  `kinds.hpp`'s banner "(N7)", A1's citation on a line whose tail A3 rewrote. A1's and A2's own names in their doc
  comments — `docs/api/kinds.md` 2 at the base and now, `service.md` 9 and 9 — are their chunks' wording, left.)*
- **A3-D19** *(the review, 2026-10-22)* **no constant of the host test plant was TUNED.** The brief's A3 row, §1.12
  ("MECH-A3 tuned") and §4.3 (HA-183's owner "A2 (A3 tunes)") planned it; the brief's P2a, P2b, P2c, P3, D2 and cases
  A, F, H reproduced to the hundredth on A2's plant (the 23:05–23:17 probe entry), so none needed it. A3 EXTENDED the
  plant (`RotationMount`, `SwitchMount`, the wedge) and changed no `ArmPlantConfig` or `FlywheelPlantConfig` default
  (`git diff fbaf729 -- include/shulib/sim/mechanism_plant.hpp`). The 23:05 heading that said "the plant tuned" is
  corrected in place (it says so); HA-183's entry says it in its Blast radius.
- **A3-D20** *(the review)* **THE SOFT STOP, REBUILT** (the brief: a push past a limit "HELD AT THE LIMIT by the
  closed-loop hold (P+I+D + gravity)", entered at the stopping distance). The first version let go every tick the
  hold had the arm back inside the limit — the full push for a tick, the hold's integral reset, another soft stop
  counted (25 in a 60 s push at 2×, 8 V), and a 2× arm drifting +1.59° in 15 s instead of settling — and its hold's
  P on the 40° a fast arm still had to go pushed it toward the limit HARDER than the driver (8.70 V for an 8 V
  push). Now, in `tickJog()`: a push toward a limit that could no longer stop before it at the smooth move's own
  acceleration (its speed TOWARD the limit only) is taken over and LATCHED for the rest of that push — let go only
  when the push turns away (or a new command ends it): ONE soft stop per push. The MOVE loop (P + D and gravity, no
  integral to wind up on the way) brings it toward the limit; a FRESH hold loop (P + I + D and gravity) holds it
  there from the moment it reaches the limit or the time a stop at the move's acceleration would take (its speed
  then over that acceleration); the output toward the limit is CAPPED at the push's own volts. Measured over 3
  cartridges (Red, Green, Blue 5:1) × plant kG 1×/1.5×/2× × both limits × 4/8/12 V (`fix/probes/p_ss_grid.cpp`; re-run
  at the resumption on the final code, `fix/resume/vcur/grid_hard.txt`): worst **8.32°** past (Blue, 2×, 12 V down —
  the reviewed version 10.11°), **2.50°** with the gravity right (4.56°), Green's D2 arm 3.67° at 2× (4.06°), never
  harder toward the limit than the push; chapter 18 now quotes the whole table. **The alternative the review named
  first was BUILT and REJECTED:** a deceleration RAMP from the arm's speed to the limit at the move's own
  acceleration, tracked by the move loop with the full feedforward (kS, kV·v, kA·a, gravity), then a fresh hold — the
  stopped run's variant `fix/variants/v1`, and independently this resumption's `fix/resume/vramp` (the same numbers to
  the hundredth): WORSE everywhere it matters — **10.44°** worst, the Blue 2× arm meeting its limit at 219 deg/s, and
  Green's D2 arm 4.13° past with its gravity RIGHT where the chosen design gives 0.02° (`fix/resume/vramp/grid_hard.txt`):
  with the starting kA 0, the P + D lags a plan that decelerates at the move's own rate, so the arm arrives fast.
  The stopped run's other variants: v0 (the reviewed code, byte-identical), v2 and v3 (a latched hold from the
  entry, uncapped and capped) all 10.11° worst; v4 (the move loop, then the hold) 8.32° — the one kept.
- **A3-D21** *(the review)* **a lift's move plan in ONE scale, its STEEPEST.** The brief: "a lift's inches converted at
  the move's start: exact for a spool, a conservative scale for a linkage or a table". The first version converted
  with the move's MEAN slope while the starting speed was derived with the STEEPEST — so near a linkage's top the
  degree plan asked up to 3.9× the free speed (a saturated push, not a trapezoid). Now `PositionSlot::degPerUnit()` —
  the inverse of `detail::unitsPerOutputDeg()`, the one the starting speed uses — scales the plan's speed and
  acceleration AND the soft stop's stopping distance (`accelDeg_`), and a measured table's scale is its steepest
  segment, not its first (`unitsPerOutputDeg()`, A1's function, fixed where it lives). A plan is never faster in
  units than stated anywhere on the travel, and is slower where the shape is flatter (near a linkage's top) — the
  conservative side, which is what the brief's word meant.
- **A3-D22** *(the review)* **`PositionGoal::units`** (a new field, set by the `Arm` and `Lift` words; the byte 0, a
  goal built by hand, finds nothing) — `find(const PositionGoal&)` finds only a mechanism of the goal's units, and,
  for a named preset, only when this robot's preset of that name holds the SAME value (`find(const SpeedGoal&)` the
  same for a flywheel's named speed). A3-D10's claim (a goal of another robot's same-named mechanism never moves this
  one to a stranger's value) is now true for a same-named lift's goal and a same-named preset at another value. A4
  is the lookup's first caller (landmine 6: an auton compiled into another robot's build).
- **A3-D23** *(the review)* **the homing signature: no new build refusal; a diagnosable line instead.** The review's
  first suggestion — refuse a homing signature above |homeVolts| / 12 × 2.5 A (n58's never-trip class) — was NOT
  built: that bound is HA-93's INVENTED current shape (the host test plant draws |V − kV·ω| × 2.5 / 12), and a real V5
  motor that draws more at a low push would make the build refuse a signature that works — a refusal must rest on a
  fact (n58's 2.5 A is the motor's own limit), not on a belief. Instead: a stall-home that runs out of time logs what
  its push drew and what its signature needs (`(it never stalled against its stop: it drew at most 0.62 A, its
  homing signature needs 2.00 A)` — `homePushAmps_`, the most current over the push); chapter 18 gives the homing
  signature its own rule with a worked number (below what the push draws when it stalls; 0.45 A for a 3 V push to
  start — the host test plant's, checked on the bench); and A1's robot y `swing` fixture states 0.45 A (its 1.5 A
  could never trip: every `home()` TIMED_OUT — the chunk that finds a flaw fixes it there, in
  `test/mechanism_fixtures.hpp` and A1's words test). HA-93's entry notes chapter 18's reliance on its shape.
  *(Its line's number was REDEFINED by A3-D26, the verification round: `homePushAmps_`, the most over the whole
  push, counted the behaviour before the push and a swing the push braked.)*
- **A3-D24** *(the review)* the boot line compares the smooth move's ramp by its TIME (a stated `maxSpeed` alone keeps
  the starting 0.25 s ramp — the acceleration follows the speed — so the ramp is not marked "(tuned)" nor counted),
  and labels the flywheel's `kI` "(V per output rad)" beside `kV` and `kP`'s "(V per output rad/s)".
- **A3-D25** *(the review's prose pass)* `include/shulib/control/trapezoid_profile.hpp` — outside §4.1's list —
  said nothing in the tree instantiates a `TrapezoidProfile`; A3's moves do. Its banner, `ProfileConstraints` and the
  class doc now name the mechanism core's position moves as its one consumer (the drive's motions still none), and
  guide 14's "No motion profiles on the main verbs yet" is scoped to the drive (pace rule 4: a stale sentence fixed
  where it is found). `docs/api/trapezoid_profile.md` regenerated.
- **A3-D17, re-measured after the review** *(2026-10-22)*: `sizeof(MechanismService)` on ARM `-Os` is **9 640 B**
  (was 9 448), `PositionSlot` **592 B** (568) — the soft stop's latch direction and its approach clock, and the
  homing push's most current (`fix/resume/probes/sizes.cpp`, the same failing-template read). **Re-measured after the
  verification round** (2026-10-22, `vfix/probes/sizes.cpp`): unchanged, **9 640 B** and **592 B** — the new flag
  sits in the padding after `softStops_`.
- **A3-D26** *(the verification round, 2026-10-22)* **a stall-home's TIMED_OUT line counts the PUSH'S OWN current,
  WHEN STOPPED.** The verifier's finding V-1: the first homing tick samples BEFORE the push acts, so A3-D23's
  `homePushAmps_` (the most over every homing tick) carried the previous behaviour's current — a jog pinned at 8 V
  just before printed 1.67 A against a 1.50 A signature beside "it never stalled". Its suggested fix (skip the first
  tick) was built, and was NOT ENOUGH: `vfix/probes/p_brake.cpp` showed the push's OWN current at speed contradicts
  the line the same way — a home begun while the arm still swings up brakes it at 1.29 A (1.42 A after 30 ticks of
  jog), the push's stopped current 0.62 A. The number a homing signature is sized by is what the push draws when
  STOPPED — no faster than the signature's own speed, the only samples its current test could have tripped on. So:
  `homeStoppedAmps_` records only samples after the push's first command (`homePushed_`, set by it) AND at or below
  `homeStall.speedRadS` (the detector's own `<=` test on the same `meanVelocity()` read, now read once per tick for
  both); `home()` resets both. The line says `it drew at most 0.62 A when stopped, its homing signature needs 2.00
  A`; a push that never got that slow says `it never slowed to its homing signature's speed -- did it reach its
  stop?` instead of a number (its record stays < 0). The homing DETECTOR still takes the first tick's sample, as
  every stall watch does (each tick samples, then commands): a stall-grade sample from the behaviour before can open
  its window one tick early, and it stays open only if the push is itself stall-grade from its first acting tick
  on — so a home can trip at most ONE tick sooner than its own samples alone would make it (an arm already at its
  stop); left as it is, consistent with every watch, and said here rather than hidden. Docs: chapter 18's homing-signature rule, guide
  11's TIMED_OUT notes, the changelog's part-3 entry, HA-93's note, `home()`'s doc comment (`docs/api/service.md`).
- **Not done here, on purpose:** the driver toggle's 5 s at-speed window and station 13's numbers are A4's and A6's
  (HA-182's entry says they add theirs); A2-D18 (REFUSED's route-marker half) is A4's.

### 23:25–23:38 — the documents, and the prose review (pace rule 4) — every grep RUN and its hits READ before this line
- **Documents (§4.1's A3 list):** `docs/changelog.md` — "Added and changed since 2.4 (2026-10-15) — part 3" at the top of
  `## API 2.4`, newest first (no bump); A2's entry's "`bind()` REFUSES …" annotated "(part 3, above, lifted it)" (dated
  history, not rewritten). `docs/hardware-assumptions.md` — **HA-182** (row, entry under a new "MECH-A3's addition"
  section, the status 177 → **178**, the MECH-A3 paragraph after A2's; the held-numbers sentence now holds HA-185 only
  of the core's) and AMENDED **HA-183** (Source: the A3 tests that run through the plant; the plant's new mounted
  sensor, switch and wedge; its blast radius with the A3 numbers), **HA-184** (Source: `Slot::homeSwitchPressed()`; the
  nets now BUILT and measured: case F STALLED at 1.70 s, case S), **HA-186** (Source: the absolute zero in `service.hpp`,
  refusal 70, the plant's `RotationMount` which models — never proves — the belief). `docs/api/` regenerated
  (`kinds.md`, `service.md`, `spec.md`, `wiring.md`, `README.md`, `all-entities.md`). **The guide's NEW chapter 18**
  (`docs/guide/18-mechanisms.md` — 18 was the next free number: G3a Part 1 has not landed): the five kinds and what each
  does, the member ORDER per kind, units and tooth-count gearing, limits and the soft stop (with the measured overshoots
  to keep the limits short of the stops by), every homing moment and which to choose and what each TRUSTS, what a move,
  a hold and a spin do and what "held" costs (N2), the jam-signature recipe, the starting values and `.tuning` (the
  order to tune in), the honest limit (a starved service), and reading a refusal (chapter 10's lesson, and this
  chapter's refusals by name); its listings compiled in `test/mechanism_chapter_example_test.cpp` (`guide-18a..c`,
  `check-examples` holds them verbatim); its row in `docs/guide/README.md` (and the maintenance note's chapter list),
  its `mkdocs.yml` nav line (outside the generated API block). Plan docs, MECH's lines only: `build-order.md` (the first
  bold `Next:` — "MECH-A3 LANDED … NEXT: MECH-A4"; MECH's own entry), `roadmap.md` ("You are here" gains the A3
  paragraph — A2's last sentence moved into it — and the `[~]` MECH row gains A3), `REMAINING-WORK.md` row 12,
  `HANDOFF-remaining-work.md` (the inventory; the docs-owed list's "mechanism chapter" marked begun). They say
  "LANDED" for the tree after the coordinator's commit, as A1's and A2's did (pace rule 5).
- **§4.2's A3 grep** `\bhold\b|\bholds\b|gravity|homing|travel limit` (README.md and docs/**/*.md minus `docs/api/` and
  `docs/internal/chunks/`, case-insensitive): 37 files, ~600 hits — narrowed to the lines that are about a MECHANISM
  (`arm|lift|flywheel|mechanism|homed|preset|service|MECH|the core`) and every one of those read: guide 10 :404/:437/
  :479/:511, guide 11, the guide README (this run's); HANDOFF :163/:451/:522/:677 (F3-era plans — history), **:787 the
  docs-owed "mechanism chapter (L8)" — marked begun**; cookbook 01 :91 (`pause` is not `hold` — the chassis, true);
  PROJECT-BRIEFING (generated / history); roadmap F6/F10/F12/F14 rows, :711/:736/:830/:932 (history), **:1734 F3's
  `liftToLevel (homing + … sag-comp PID hold)` — an F3 primitive the core now provides generically; a plan change, so
  FOR THE COORDINATOR (the Q28 re-scope's)**; `MECH-PANEL-requirements.md` (the requirements as written — stays);
  guide 14 :230/:402 (true); vexbuilder-integration :151 (true); REMAINING-WORK rows 12–14 (row 12 this run's);
  guide 13 :147/:157/:176 and guide 06 :90 ("an UNSCREENED dead port reads pressed … a lift driving into its hard stop"
  — why the adapter screens; true, and the core's read-fault rule now also refuses a held level as a home); master plan
  :413/:579/:585/:655 (history); HA register (this run's entries; HA-92's loaded-lift Hold — true).
- **§4.1's A3 re-check grep** `soft stop|stall signature|belowRpm`: guide 11 (this run's), REMAINING-WORK :255
  (history), requirements :186/:266 (stay), HA :1246/:1290 (this run's) and :2059 (HA-154: the service's watch on a
  power hold-to-run — true). `belowRpm`: 0 hits.
- **The run's own names** `MECH-A3|next part|later part|third part|refuses a list|power and pneumatic( mechanisms)? only|
  nothing homes|homes? a mechanism|NOT_HOMED|starting value|no program builds|arrive with|its only reader`: **STALE,
  FIXED** — `docs/faq.md` :25 (the service reads a HOME switch's count since the third part), guide 10's box (:400-409:
  "the position and speed behaviours … are the core's next parts"), guide 10 :533-539 ("a home switch's arrives with the
  position behaviours, its only reader"), roadmap :367-368 (A2's "next is MECH-A3"), build-order's `Next:` and MECH
  entry. Read and TRUE: guide 10 :513, :539 (no program builds the service — true), cookbook 06 :202 (a three-position
  pneumatic in driver control — A4's, true), HA :143-145 / :155 / :1138-1141 (A1's and A2's dated paragraphs, each
  followed by its "since …" note), the changelog's older entries (dated), vexbuilder-integration :427 (another chunk).
- **Stale BY OMISSION, found by reading around the hits, FIXED:** `README.md`'s directory map (`mechanism/` named only the
  declarations — no service since A2) now names the service and its four kinds; `docs/guide/14-what-it-cannot-do-yet.md`'s
  "The mechanism layer exists" section never mentioned the mechanism core at all (A1/A2 too) — one bullet added (what
  exists, host-only, what does not); `docs/guide/11-reading-the-diagnostics.md` lists every bracket the service writes
  (A2's rule) — A3's new ones added (`reached`, NOT_HOMED's three, STALLED's "while holding / held to move / spinning /
  homing", TIMED_OUT's "holds where it stopped / keeps spinning / home switch never read pressed / never stalled",
  CANCELLED's `released`, the home step's two SUCCEEDED brackets, the zero-log and clamp lines, `MECHANISM_STALLED`'s
  row); `docs/guide/15-glossary.md` gains **homing**, **mechanism service**, **soft stop**, **stall signature**,
  **starting values** (A2 deferred its glossary words to "the mechanisms chapter"); the roadmap's MECH row said "its 63
  build refusals" (A1's first count — 69 after its review, 70 now).
- **Pace rule 7:** public documents moved — §4.1's (the changelog, the register, `docs/api/`, chapter 18 with its README
  row and nav line) and the prose review's fixes (guides 10, 11, 14, 15, the faq, README's map, the roadmap's MECH row),
  each one sentence to one paragraph. Not touched, deliberately: the cookbook (A4's recipe), guide 13 (true), guide 17
  and 16 (A5's).

### 23:40–23:58 (2026-10-15) — what ran after the last entry, read back from the scratch files at the resumption
- *(Written at the resumption, from the files' own timestamps and contents — the run was stopped before it logged
  these.)* `scratch/mech-a3/logs/srcgate_selftest.log` (ended 23:49:20): **`src build gate self-test: OK (235
  detector cases, each against a real build)`**, 15 min 36 s. `fmt_probe.sh` (A2's finding 8 carried: the suite's
  flags without `-Werror`, at -O1/-O2/-O3/-Os, over the eight mechanism TUs) → `logs/fmt_probe.log` (23:51). One
  warning, at -O1/-O2/-O3 only: `wiring.hpp:1102` `'%.23s' directive output truncated writing 21 bytes into a region
  of size 8` — `roleLine()` (A1's), reached from `mechanism_wiring_test.cpp`:503, A1's DELIBERATE truncation case
  (`roleLine(small, 8, "a very long mechanism", …)`): GCC sees the constant and the 8-byte cap and says the truncation
  the test asks for will happen. Neither file is changed by this run; the suite builds at -O0 and the robot at -Os,
  where it does not fire. Not a defect — recorded, not changed. `startingValuesLine()` and every A3 TU: 0 warnings.
- `scratch/mech-a3/gates/final/` (23:51–23:58): every gate rc 0 — the configure, `api_doc_tool generate`, the
  briefing pre/post, a CLEAN build (229 objects), **the suite 2,795,475 assertions passed**, `-tc='MECH-A3*,guide-18*'`
  3228 assertions, the briefing check and self-test, the harness 201/201 (201 TUs), the api self-test / coverage /
  fresh / examples / removability, the staleness self-test and audit, the PROS-free and sim-free greps, the ARM header
  gate (218 headers, rc 0), no PROBE_ / PROTOTYPE, the C7 grep, the register's direction-1 grep, and `src_build_gate.py
  check` PASS (all 7 builds). The second self-test (`22_src_gate_selftest.log`) was EMPTY — killed by the pause.

### 22:33 (2026-10-18) — RESUMED *(`date`: Sun Oct 18 10:33:08 PM EDT 2026)*
- The team lead paused all work at ~00:15 on 2026-10-16 and resumed 2026-10-18 ~22:35; the coordinator audited this
  worktree at the pause (every file equal to its newest mutation backup under `scratch/mech-a3/mut/`; `service.hpp`
  differs from X23's backup only by a later doc comment, 23:49 — `position()`'s, re-read: true).
- Checked first: `git diff --stat fbaf729 -- src/ tools/ Makefile` EMPTY, `git status --short src tools Makefile`
  EMPTY — no gate self-test plant left by the killed run. The binary on disk (`build/test/shulib_tests`, 23:51:59)
  is NOT trusted: the resumption rebuilds clean from source (`gates.sh resume`: `--target clean`, the binary deleted,
  the full build) before any number below.
- What is left, read against §2.1's DoD and §3.3/§4.1: the fresh-build verification, the gates (the src gate's
  self-test again), the ARM size of the whole service (A2 measured 3 048 B for power + pneumatic; the brief priced the
  whole at 8 048 B — `probes/sizes.cpp` was written at 23:51 and never run), a read of §3.3's list against the tests,
  and the hand-back.

### 22:33–22:37 (2026-10-18) — the resumption's fresh build: every non-src gate green *(`date` 22:37:20)*
- `scratch/mech-a3/gates.sh resume all` (logs `scratch/mech-a3/gates/resume/`): configure; `api_doc_tool generate`;
  the briefing; `--target clean`; the binary deleted; **a full build, 229 objects compiled** (rc 0); **the suite
  under `ulimit -v 4194304; timeout 600`: 2492/2492 test cases passed, 2,795,475 assertions, 3 skipped**;
  `-tc='MECH-A3*,guide-18*'` **45 cases, 3228 assertions**; the briefing post / check / self-test (24 cases); the
  harness **201/201** (201 TUs, 201 CTests); api self-test, coverage (4672 entities, 179 headers), fresh, examples
  (565 quoted lines), removability; the staleness self-test and audit (clean); "library is PROS-free"; "layering
  holds: core is sim-free"; the ARM header gate **218 headers, rc 0**; no `PROBE_` / PROTOTYPE / "(proposed)"; the
  C7 grep empty; the register's direction-1 grep. The same numbers as 2026-10-15's last run: nothing drifted.
- **A rule broken by MY gate script, found reading its log, and why it was harmless:** `gates.sh` (written 10-15)
  ran `briefing_status.py generate` (step 03) BEFORE deleting the binary (step 04b) — the coordinator's memory rule
  says `rm -f build/test/shulib_tests` FIRST, because the briefing runs that binary UNCAPPED. Step 03 took 18 s: it
  ran the binary on disk, linked 2026-10-15 23:51:59 by `gates/final`'s CLEAN build of the restored tree (every
  mutation run by `mutate.py` deletes its binary after its suite — `os.remove(bin_)` on every path — and the last
  mutation ended 23:32), so it was the true code, and it passed. The 10-15 `build.sh` has the right order (rm, then
  the briefing). Fixed for every later run: `scratch/mech-a3/gates2.sh` deletes the binary first (`gates.sh` was
  left untouched while it ran — bash reads a running script as it goes).
- **The mutation verdicts still stand for this tree** (checked, not assumed): every `service.hpp` backup in
  `scratch/mech-a3/mut/` (36 of them, M14 … X23) is byte-identical to the others, and the tree's `service.hpp`
  differs from them only by `position()`'s doc comment (above); `mechanism_plant.hpp`, `spec.hpp`, `kinds.hpp` and
  `wiring.hpp` are md5-identical to their backups (P1wrap/P2top/P3wedge, S70/S70b, K1units, W1line). Every verdict
  line in `mut/summary.txt` says "all N dependents" recompiled and the link checked. *(The driver runs the suite
  target's `depend` step through its `build.make`, then the build — not a whole `cmake --build` — and refuses a
  verdict unless EVERY object `compiler_depend.make` lists as depending on the mutated file was recompiled and the
  binary re-linked after it: finding 17's failure is that check's to catch, and none came out INVALID.)*
- **Sizes, ARM `-Os`** (`probes/sizes.cpp` through a failing template, run now): **`MechanismService` 9 448 B**,
  `PositionSlot` 568 B, `SpeedSlot` 208 B, `PowerSlot` 224 B, `PneumaticSlot` 136 B (A2: 3 048 / 224 / 128 B — the slot
  base gained the home switch's read-fault snapshot). **A3-D17** below: the brief priced the whole service at 8 048 B
  and a `PositionSlot` at 536 B (S3).
- **§3.3 read against the tests, item by item:** P2a (+ finding 4's "cannot follow ends TIMED_OUT"), P2b (with its
  control), P2c (the robot file's signature, the 1.6 A / 3 rpm / 0.15 s one, N2's case — A3-D13), P2d (the limit,
  the clamp once, D2's held push, never homed), P2e (every homing moment), P2f (the lift), P2h (absolute, reversed,
  the mismatch STALLED), the integral's ablation and the fresh hold, P3 (27 plants, the 20 % shot, the window's
  ablation — A3-D14), N10 (case A, case H, the jog into an obstacle, P2h's mismatch, the 3.0 A property — A3-D13; the
  Power hold-to-run's is A2's `mechanism_service_power_test.cpp`:594), D2 (both limits, 1×/1.5×/2×, the release's
  hold target, N2 clamped, case C), D4 (rotary, spool, linkage top, table ends — case G 1.91 s), D10; the mutations
  M14–M21, M16 at both limits, M36, M39. Each has its case; each mutation its RED line. P2g is §3.1's (A1).
- **Public-doc wording fixed (A3-D18 below):** the changelog's part-3 entry said "the brief's P2a", "Every mutation
  of the brief's §3.3 run and seen red (the live log)" and "(N10)" — names that mean nothing outside
  `docs/internal/` (which a release drops; no earlier changelog entry names a brief, a decision number or a log). Now
  "144 moves over …", "Every rule's mutation was run and turned its test red." (A2's own words) and no "(N10)".
  Re-run after: api check-fresh / check-examples (565 lines) / check-removability, the staleness audit, the C7 grep
  — all clean. Guide 18, guides 11/14/15, the faq and README grepped for `\bN[0-9]+\b|\bD[0-9]+\b|A[1-6]-D[0-9]+|the
  brief|live log`: no hit (the register's "case F" / "case S" name its tests, and "the core's brief" was its wording
  before this run).
- **Looked at and left:** three code lines past 120 columns (`service.hpp` :806 and :1730, the `SpeedGoal` find;
  `wiring.hpp` :1161, 121) — no formatter or gate enforces a width, and A2's own tests and `wiring.hpp` carry longer lines
  (`mechanism_service_test.cpp` 14 code lines past 120 at the base); re-wrapping would change verified files for no
  behaviour. The -O1..-O3 `-Wformat-truncation` note on A1's `roleLine()` test (above): not a defect, not this run's.

### 22:40–22:45 (2026-10-18) — the landing dates made true for the commit (pace rule 5)
- The plan docs said "MECH-A3 LANDED 2026-10-15" — written the night it was built, but it lands at the coordinator's
  commit, after this resumption. Now **"LANDED 2026-10-18 (built 2026-10-15)"** in `build-order.md` (the first bold
  `Next:` — "MECH-A3 LANDED 2026-10-18 (built 2026-10-15, verified at its resumption 2026-10-18)" — and MECH's own
  entry), `roadmap.md` ("You are here": "Now (2026-10-18): MECH-A3, the third part (built 2026-10-15), has LANDED";
  the `[~]` MECH row), `REMAINING-WORK.md` row 12; the changelog's heading "(2026-10-18; built 2026-10-15)" (A2's
  form); guide 10's box ("since its third (MECH-A3, 2026-10-18)"), guide 14 ("2026-10-15 to 2026-10-18, three of its
  six parts"), chapter 18's header ("MECH-A1 and A2, 2026-10-15; A3, 2026-10-18"). **For the coordinator: if the
  commit lands on another day, these say 2026-10-18.** Left as the BUILD date, on purpose: the register's "since
  MECH-A3 (2026-10-15)" source notes and its section heading (its row says "built 2026-10-15"), HANDOFF's "chapter
  18, MECH-A3, 2026-10-15" (when the chapter was written), and the code comments naming the run by its date
  (`kinds.hpp` :47, `service.hpp`'s banner, `sim/mechanism_plant.hpp`'s).
- `PROJECT-BRIEFING.md` regenerates from `build-order.md`'s pointer: regenerated by the final gate run below.

### 22:37–22:58 (2026-10-18) — the src gates, then the final gates on the final tree *(`date` 22:58)*
- From `gates.sh resume` (the tree before the doc-only edits above — none of them reaches `src/`: no `src/` file and
  no header `src/` includes was changed after it; `src/` reaches the mechanism layer only through
  `robot/mechanism_parts.hpp` → `mechanism/devices.hpp`, which includes none of A3's four headers):
  **`src_build_gate.py check`: PASS (all 7 builds link …)**, every build `exit 0, 11/11 TUs compiled, 0 disallowed`
  (170 s); **`self-test`: OK (235 detector cases, each against a real build)** (852 s); `git diff --stat fbaf729 --
  src/ tools/` and `git status --short src tools Makefile` EMPTY after both (no plant left).
- `gates2.sh final2 build` on the FINAL tree (logs `scratch/mech-a3/gates/final2/`; the binary deleted FIRST, then
  the briefing, a clean build): configure; api generate; **229 objects compiled**; **suite 2492/2492 passed, 2,795,475
  assertions, 3 skipped** (capped); `-tc='MECH-A3*,guide-18*'` 45 cases, 3228 assertions; briefing regenerated
  (`PROJECT-BRIEFING.md` now carries "MECH-A3 LANDED 2026-10-18 …" from `build-order.md`'s pointer), check
  (current) and self-test (24); **harness 201/201** (201 TUs); api self-test / coverage (4672 entities, 179
  headers) / fresh / examples (565 lines) / removability; staleness self-test (10) and audit (clean, 30 docs);
  PROS-free; sim-free; **ARM header gate 218 headers, rc 0**; no PROBE_ / PROTOTYPE; C7 empty; direction-1 clean.

### §2.1's DoD — MECH-A3, checked against the tree *(2026-10-18 22:58)*
*(Superseded by the re-check at the end of "MECH-A3 review findings and their disposition", below — the review
changed the code, the tests, the numbers and this list's hand-ons; this block is the 10-18 record, kept.)*
- [x] This log carries the run's heading, its tip (`fbaf729`), each step as it happened (and the 10-15 23:40–23:58
      steps read back from their files at the resumption), every mutation with its observed red, the prose review's
      greps and hits.
- [x] The host suite builds and passes (2492/2492, 2,795,475 assertions, 3 skipped — memory-capped); `ctest -R
      compile_refusal` 201/201.
- [x] The ARM header gate (218 headers, rc 0); the CI PROS-free and sim-free greps.
- [x] `src_build_gate.py check` PASS, all seven builds, 0 disallowed warnings; `self-test` OK (235).
- [x] `grep -rn "PROBE_" include/` empty; no copied file says PROTOTYPE / "(proposed)" (gate 24).
- [x] Every §3.3 test exists (the item-by-item read above) and every §3.3 mutation was RUN on a fresh build of every
      dependent object, observed RED and restored byte-identical (M14–M21, M16 at both limits, M36, M39) — plus the
      run's own 30 (X…, P…, S70/S70b, W1line, K1units); the three GREEN ones each answered (X14profile → a test added,
      then RED; X15fresh EQUIVALENT, X15d RED; M18b EQUIVALENT, kept as a guard). None killed by the cap or the
      timeout.
- [x] Docs in the same change (§4.1's A3 list: changelog, `docs/api/`, HA-182 only — HA-183/184/186 amended in
      their Source notes, no other number registered —, chapter 18 with its README row and nav line); the prose review
      logged (above); the C7 grep empty.
- [x] Nothing committed; no `pros`; no `/dev/ttyACM*`; `git diff --stat fbaf729 -- src/ tools/` EMPTY. **The tree:**
      30 files modified + 6 new (`docs/guide/18-mechanisms.md`, `test/compile_refusals/s70_absolute_homing_whole_turn.cpp`,
      `test/mechanism_chapter_example_test.cpp`, `test/mechanism_position_rig.hpp`, `test/mechanism_position_test.cpp`,
      `test/mechanism_speed_test.cpp`), uncommitted, in the worktree `.claude/worktrees/mech-a3`.
- **For the coordinator** (unnumbered; no hardware belief outside HA-182 was met):
  - **Files main has changed since `fbaf729`** (CAL1 Part A1 `ceb17c0`, SER1's record `fd55ffc`) that this run also
    changed: `docs/changelog.md` (both add a "since 2.4" entry at the top of `## API 2.4`), `docs/internal/build-order.md`
    (the first bold `Next:` is ONE long line both edit — a certain conflict: keep CAL1's sentence and A3's "MECH-A3
    LANDED 2026-10-18 … NEXT: MECH-A4 …, then A5–A6"), `docs/roadmap.md`, `docs/internal/REMAINING-WORK.md`,
    `docs/internal/HANDOFF-remaining-work.md`, `README.md` (different hunks), `mkdocs.yml` (the generated API block on
    main; this run's guide nav line), and the generated `docs/api/README.md`, `docs/api/all-entities.md`,
    `docs/internal/PROJECT-BRIEFING.md` — regenerate those three after the merge (api_doc_tool generate; the briefing
    after `rm -f build/test/shulib_tests` and a fresh build). `test/CMakeLists.txt`: NOT touched (the new tests and
    the harness TU are picked up by its globs).
  - The landing dates say **2026-10-18**; change them if the commit lands another day (the list in the 22:40 entry).
  - G3a's validator mirrors `spec.hpp`'s refusals by sentence: it is **70** now (A3-D3).
  - The roadmap's F3 `liftToLevel (homing + … sag-comp PID hold)` (`docs/roadmap.md` :1734 at the 10-15 read) is a
    primitive the core now provides generically — a plan change, the Q28 re-scope's to make.
  - `sizeof(MechanismService)` is 9 448 B on ARM (A3-D17) — for A5, which puts one in each program.
  - The mutation driver's build step is the suite target's `build.make` with its `depend` step run first, verified
    per mutation (every dependent object recompiled, the link checked) — not a whole `cmake --build`; finding 17's
    failure would have shown as INVALID, and none did.

### MECH-A3 review findings and their disposition *(the fix round: started 2026-10-22 ~01:20, STOPPED ~02:16 by the session limit; RESUMED `date`: Thu Oct 22 08:43:31 AM EDT 2026)*
- **Three reviewers** (correctness, tests, docs) returned 33 findings, numbered below in the order received; two
  overlap (13 ⊃ 1's test half; 14 is 9–13's survivors in the changelog's words). Every one was re-checked against the
  tree; **every one was real** (33's premise has since moved — see 33).
- **The stopped run, read back from its files** (it logged nothing but this log's 23:05 heading fix): code and test
  edits 01:36–02:07 (`service.hpp`, `kinds.hpp`, `wiring.hpp`, `control/trapezoid_profile.hpp`, A1's
  `test/mechanism_fixtures.hpp` and `mechanism_kinds_test.cpp`, both A3 test files); six fresh builds and capped suites
  `fix/logs/f1…f6` (f6, 02:07:57: **2514/2514**); soft-stop design variants `fix/variants/v0…v4, v4n, wt` (01:38–01:55);
  a copy of the final tree, `fix/muttree`, on which `fix/mut.py` re-ran every mutation 02:09–02:16 (`fix/mut/summary.txt`:
  50 verdicts) until the stop killed it mid-`Napproach` (its log empty); the documents 02:11–02:15.
- **Checked FIRST at the resumption:** the live log had no disposition section (grepped); `git status` and `git diff
  --stat fbaf729 -- src/ tools/ Makefile` — src/, tools/ and the Makefile untouched, no gate plant; every worktree file
  `cmp`-equal to its newest backup in `fix/mut/` (`service.hpp` = `Napproach.service.hpp.orig`, `kinds.hpp` = K1units's,
  `spec.hpp` = S70b's, `wiring.hpp` = W1line's, the plant = P3wedge's) — the mutations ran in `fix/muttree`, never in
  the worktree; **`fix/muttree`'s `service.hpp` still held the killed `Napproach` mutant** (`if (!holdLoop_) {`) —
  restored from its backup, timestamp bumped, its binary deleted; `fix/muttree` then equal to the worktree file by file
  (`diff -rq`, build dirs aside) once the stopped run's later doc edits were copied in. The worktree's binary was never
  trusted: the final gates deleted it first, then the briefing, then a clean build.
- **The reviewers' probes re-run on the final code** (`fix/resume/probes/*.now.txt`, each under the memory cap):
  p_softstop2 Blue 2× 12 V down 10.11 → **8.32°**, 1× 4.56 → **2.50°**; p_blue the most toward the limit **8.00 V** for an
  8 V push (was 8.70); p_softstop3 a 60 s push: **one soft stop, ±0.00° from 10 s on** in all nine (was 25–37 stops,
  +1.45° drift); p_linkage the joint never above **66 of 86 deg/s** (Red) / **135 of 171** (Green) — 77–79 %;
  p_stallhome the 2.0 A recipe still TIMED_OUT (by design, A3-D23) and now says `it drew at most 0.62 A, its homing
  signature needs 2.00 A`; p_findgoal every stranger's goal **(nil)**; p_bootline `in 0.25 s,` unmarked; p_absturn −12°
  still believed 348° and STALLED at 0.70 s (documented, finding 5); p_restsstop the limit-at-the-stop arrivals
  0–19 deg/s. The stopped run's own measurement probes (`p_measure`, `p_measure2`) reproduced byte for byte; `p_ring`,
  `p_bottom` as the tests quote.
- **Found at the resumption, fixed:** (a) **R10 SURVIVED** the stopped run's test for it ("a re-home by STALL from a
  loaded hold"): its hold had been loaded 1 s and still sagged at 9.6 deg/s, so the homing detector's first sample was
  not stall-grade and a window left open closed on it — the case could not see the reset (`fix/resume/probes/r_rehome.cpp`
  traced it: with the hold SETTLED, 1.08 A at rest, the mutant re-zeroes at once, "its zero moved +59.5 deg"). The
  subcase now ticks the loaded hold 4 s and REQUIRES both halves of the stall-grade sample (current AND speed); R10
  re-run: **RED**, and the twelve other mutations whose RED had named that test case re-run on it: all **RED**
  (`fix/resume/mlist_resume3.py`). (b) `Nfindvalue` and `M16` were BUILD failures (`-Werror=unused`), no verdict —
  re-written to compile, both **RED**. (c) the rests-at-stop test said its matched-gravity arm "never" meets its stop:
  it meets it at 0.0–0.2 deg/s (never at 6 V) — comment corrected (`p_measure2`).
- **The mutations, all on the FINAL code** (`fix/mut.py` and `fix/resume/mut.py`: one mutation at a time in
  `fix/muttree`; a FULL `cmake --build build/test` — its depend step and the doc gates — line-preserving so the doc
  gate's line numbers hold; a verdict only when EVERY object depending on the mutated file was recompiled (8 for
  `service.hpp`, 16 for `kinds.hpp`/`wiring.hpp`/`spec.hpp`, 5 for the plant) and the binary re-linked after it; the
  `MECH*,guide-18*` filter, then the whole suite, under `ulimit -v 4194304; timeout 600`; restored in `finally`,
  byte-identical, timestamp bumped; the binary deleted after each): **98 distinct mutations — 96 RED, 2 GREEN and
  EQUIVALENT** (M18b, X15fresh — as at 10-15: a clamp that can never act, a reset another reset already makes); none
  killed by the cap or the timeout; every `restored=True`. Lists: `fix/mlist_final.py` (the brief's §3.3, the run's
  own, the fix round's `N…`, the reviewers' `R…`), `fix/resume/mlist_resume*.py`; summaries `fix/mut/summary.txt`
  (02:09–02:16, 50 lines) and `fix/resume/mut/summary.txt` (08:54–09:04, 63 lines).
- **The soft stop's ramp, built again** (`fix/resume/vramp`): finding 1's first suggestion, independently re-written
  at the resumption — the same grid as the stopped run's v1 to the hundredth (10.44° worst). A3-D20 records why it was
  rejected.

| # | Lens, severity | Finding | Disposition | Proof (mutations on the final code, each RED unless said) |
|---|---|---|---|---|
| 1 | correctness, major | the soft stop measured on one arm at 8 V; Blue / 12 V 7.8–10.1° past; its hold pushed harder than the driver | **FIXED in part, the rest MEASURED and DOCUMENTED** (A3-D20): latched, the move loop then a fresh hold, capped at the push — worst 8.32° (Blue 2× 12 V down), 2.50° with gravity right; the ramp tried and worse. Chapter 18's table (3 cartridges × 3 gravity errors), the rule that it grows with arrival speed and gravity error, a move's own overshoot up to 10.27° on the Blue arm; HA-183's numbers. New case: the soft stop on a SLOW, MEDIUM and FAST arm at 4/8/12 V (54 pushes; ≤ 8.32 + 1°, matched ≤ 2.50 + 1°, never harder than the push, one soft stop per push) | Ncap, Napproach, Napproachend, Nfreshhold, Nlatch, M16, M16top, M16bottom, X13stopdist, R7stopquarter |
| 2 | correctness, major | a homing signature by the jam recipe can never trip at −3 V; robot y's `swing` states 1.5 A | **FIXED, a different way** (A3-D23): no refusal (it would rest on HA-93's invented current shape); the TIMED_OUT line says what the push drew against what the signature needs; chapter 18's homing-signature rule (0.45 A for a 3 V push to start), guide 11, the glossary; `swing` 0.45 A (A1's fixture and words test); HA-93's entry. New case: the 2.0 A home TIMED_OUT with its line, and robot y's swing HOMES | Nhomeamps |
| 3 | correctness, major | a linkage's or a table's plan up to ~4× the free speed (mean slope against a steepest-slope speed) | **FIXED** (A3-D21): one scale, the steepest — the plan, the soft stop's distance, a table's steepest segment. New case: 8 moves on a DR4B, the measured table and a table steepest at its top — never a saturated tick, the joint ≤ 80 % of free, ≥ 70 % where it cruises | Nscale, Ntable, R44linkscale (A1's words test) |
| 4 | correctness, minor | the limit hold chattered: let go inside the limit, the full push a tick, the integral reset, another soft stop | **FIXED** (A3-D20's latch). New cases: a 20 s push stays ONE soft stop and converges within 0.1°; D2 asserts `softStops() == 1` | Nlatch, R6softcount, R30turnaway |
| 5 | correctness, minor | absolute homing's turn has no margin: an arm resting > (360 − travel)/2 outside its limits is read a turn off | **DOCUMENTED, not refused**: the build sees the travel, never where an arm rests at power-up; the jam signature is the net (STALLED 0.70 s). Chapter 18's rule with worked numbers (160° → ±100°; 340° → ±10°); HA-186's entry. New subcase: the one-turn window (−90 read right; −110 believed 250, STALLED at its stop) | R49floor, R49ceil |
| 6 | correctness, minor | "limits short of the stops" cannot hold at a rests-at-stop / stall-homed end | **DOCUMENTED + TESTED**: chapter 18's "At a home end the limit IS the stop" with the measured arrivals (0.2 / 13.5 / 35 °/s). New case: the plant's stop AT the limit — met at ≤ 35.3 + 5 deg/s, never STALLED, held | M16bottom, R7stopquarter |
| 7 | correctness, minor | `find(PositionGoal)` ignored units: a same-named lift's goal moved this arm | **FIXED** (A3-D22): `PositionGoal::units`; a named preset must match in value too. New case (and the speed one) | Nfindunits, Nfindvalue, X18unitsfind, X19goalpreset |
| 8 | correctness, minor | the boot line marked a speed-only ramp "(tuned)"; the flywheel's kI labelled per rad/s | **FIXED** (A3-D24). Boot-line case: speed only, acceleration only, a declaration's `.maxSpeed` | Nrampmark, NkIunit, W1line |
| 9 | tests, major | absolute homing tested only below the middle — floor for round survived | **FIXED (tests)**: an arm at 150 (0..160) and robot y's reversed wrist at +60 | R49floor, R49ceil, X1wrap |
| 10 | tests, major | kS, kA and a lift's constant gravity never exercised | **FIXED (tests)**: a tuned kS (±0.5 V, both directions, exactly) and kA; a lift held by its gravity term alone (kI 0: within 0.01 in, 3.0 V at rest); the speed loop's kS (the reverse-spin case) | R4possign, R5kA, R38constgrav, R39level |
| 11 | tests, major | removing either 3 V clamp, clearing (not freezing) the speed integral, carrying it into a new spin — all survived | **FIXED (tests)**: a move blocked 5 s (≤ 1.9 A, then ≤ 9.8° over — unclamped STALLED); a wheel dragged short (≤ 300 rpm over — unclamped 540); a shot's recovery ≤ 0.50 s (cleared 0.57); a re-spin ≤ 50 rpm over (carried 172.5) | R36holdcap, R42holdcaplow, R2spdcap, R2bspdcaplow, R40spdclear, R32spdintreset, R41wide, R41narrow |
| 12 | tests, major | REACHED's settle window not tested as contiguous | **FIXED (tests)**: a ringing arm (2 exits after its plan) — the verdict a full settle time after the LAST entry | R1tolreset, R1bsettle |
| 13 | tests, major | the soft stop tested on one arm: a quarter distance, an unconverted lift acceleration, a per-tick count, the direction-blind distance all survived | **FIXED (tests)**: the cartridge grid (1); a spool lift at both limits (the push untouched until near, then held within 0.05 in); a fall away from the top when the push flips up is the driver's own; a push turned away is let go at once, back up a NEW soft stop | R7stopquarter, R51accel, R6softcount, R22stopdir, R30turnaway |
| 14 | tests, major | the changelog said "Every rule's mutation was run and turned its test red" against six survivors | **FIXED**: every survivor RED on the final code (9–13, 15–22) and the sentence narrowed to what was shown — "98 mutations of these rules, the review's own among them … 96 turned a test red, and the other two change nothing the code can do (each shown equivalent)"; the TESTED paragraph names the review's new cases; the DoD below matches | (the table's every row) |
| 15 | tests, minor | the re-home Warn's threshold: one side, 10× — both survived | **FIXED (tests)**: a stall-homed arm wedged 2° (Warn) and 1° (Info); a measured table homed at its bottom end (1.25° one side, 0 the other: 1° is Info) | R8tolside, R9warn10 |
| 16 | tests, minor | the stall-watch and step-clock resets untested (R10–R16, R18) | **FIXED (tests)**: every command restarts them — a new jog, a move from a loaded hold, `holdHere()`, a switch-home from a pinned jog, a re-home by stall from a SETTLED loaded hold, a move's first tick — on exact 1/128 s ticks (the 65th stall-grade sample is the trip) | R10 (on the resumption's fixed subcase), R11–R16, R18 |
| 17 | tests, minor | `find(Lift)`'s units and `find(SpeedGoal)`'s preset untested | **FIXED (tests)**: a lift named "arm" on an arm rig; a same-named flywheel's missing preset and its preset at another speed | R37findlift, R19speedgoal |
| 18 | tests, minor | the speed boot line's "(tuned)" never tested | **FIXED (tests)**: one tuned (kP) and all four, verbatim | R20bspdmark |
| 19 | tests, minor | no reverse spin | **FIXED (tests)**: ±2000 rpm on a 10 %-slow plant both SUCCEEDED (the window opens backward) | R3spdwinneg |
| 20 | tests, minor | `jog(0 V)` == `holdHere()` untested | **FIXED (tests)**: the target where it was, the gravity hold commanded, the claim held | R35jog0 |
| 21 | tests, minor | a tautological assertion (`target() == Approx(target())`) | **FIXED**: the release's real claim — the target is where the arm read, clamped to the limit | — |
| 22 | tests, minor | a reversed sensor's zero log, and `>=` at the homing and spin-up bounds, unpinned | **FIXED (tests)**: the reversed wrist's line verbatim; both bounds on 1/128 s ticks (TIMED_OUT on the third, `elapsed == 2/128`) | R28abslog, R23homegt, R24spdgt |
| 23 | tests, minor | the 10-15 mutation driver ran no whole `cmake --build`; two test files changed after the last mutation | **SUPERSEDED**: every mutation re-run on the FINAL tree with a full build (above). What the 23:39 edits of `mechanism_position_rig.hpp` and `mechanism_speed_test.cpp` changed cannot be read back — no copy of either exists from between 23:32 and 23:39 — so it is recorded as unknown, not guessed; the reviewers' own full-build re-run of 18 (`review-gates/run3.out`) was all RED | (all) |
| 24 | docs, major | HA-182 "printed at boot and replaced by `.tuning`" false for its three library constants, and nothing prints the line yet | **FIXED**: the claim split wherever it is made — the register (the MECH-A3 paragraph, the row, Source, a two-part Contained with the wider blast radius), `service.hpp`'s banner and the constants' docs, `kinds.hpp`'s banner, `startingValuesLine()`'s doc, chapter 18 (a paragraph on the three LIBRARY constants; "clamped to 3 V" out of the `.tuning` table), the changelog; "printed at boot" → "worded for a boot line the programs print from MECH-A5; none does yet" (chapter 18, glossary, roadmap, build-order → the briefing). Whether a team should tune the three is a design question: for the coordinator | — |
| 25 | docs, minor | MECH-A4's driver-loop behaviour in the present tense | **FIXED**: the changelog, chapter 18 (twice), the glossary, `service.hpp`'s banner — "from the core's driver controls on (its fourth part)" | — |
| 26 | docs, minor | "LANDED 2026-10-18" already wrong | **FIXED for a commit TODAY**: 2026-10-22 in the changelog heading, guides 10, 14, 18, roadmap ×2, build-order ×2, REMAINING-WORK (the 22:40 entry's list); the briefing regenerates. If the commit lands another day, these change again (for the coordinator) | — |
| 27 | docs, minor | "the models tuned" not done, unlogged | **FIXED**: A3-D19; the 23:05 heading corrected in place; HA-183's entry | — |
| 28 | docs, minor | homing "switch or stall" built as both while F14 and master-plan question A say open | **HANDED ON** (Q26's plan commit is the coordinator's): below | — |
| 29 | docs, minor | kI's unit, `.cartridge` "common", flywheel kS 0 missing from HA-182 | **FIXED**: chapter 18's rows (kI per output rad), the boot line, "`.cartridge` (every kind with motors)", "kS 0" in HA-182's row and entry | NkIunit |
| 30 | docs, minor | chapter 18's "Reading a refusal" had no example; guide 10's list lacked refusal 70 | **FIXED**: refusal 70's note chain in chapter 18 — GCC 13.3's real output for `fix/refusal/your_robot.cpp` (re-compiled at the resumption: `your_robot.cpp:7:9: in 'constexpr' expansion of 'shulib::mechanism::wire<RolePort>(wrist, …)'` … `spec.hpp:1064:103: error: call to non-'constexpr' function 'void shulib::mechanism::refused::position_absolute_homing_needs_a_travel_shorter_than_one_turn_of_its_angle_sensor()'`), trimmed of the `wireMotors` expansion and the template arguments, as it says; guide 10's list gains the clause | — |
| 31 | docs, minor | `mechanism_kinds_test.cpp`'s stale "the brief holds as HA-182" | **FIXED** | — |
| 32 | docs, minor | A3-D18's rule broken by A3's own doc comments reaching `docs/api/` | **FIXED**: A3's doc comments reworded (no "the brief's P3", N2, N7, N10); A3-D18 scoped (its note) | — |
| 33 | docs, minor | the merge list left out main's uncommitted plan update | **REAL, then OVERTAKEN**: main committed it as `71b4764` (HA-192..HA-203 held for VIS1); the hand-on below lists main's commits since `fbaf729` and the overlaps | — |

- **The A3 tests now** (the final build): `MECH-A3 position` **47** cases (was 32), `MECH-A3 speed` **14** (7), `MECH-A3
  plant` 3, the chapter's `guide-18a..c` 3 — **67 cases, 3925 assertions** under `-tc='MECH-A3*,guide-18*'`.
- **Docs moved in this round (pace rule 3):** `docs/changelog.md` (part 3's header, the position and speed entries, the
  library constants, the boot line, the TESTED paragraph), `docs/hardware-assumptions.md` (the MECH-A3 paragraph,
  HA-182's row and entry, HA-183's Blast radius, HA-186's Source, HA-93's note), `docs/guide/18-mechanisms.md` (the
  common fields; units — the steepest scale; the soft stop and its table; a home end's limit; the one-turn rule; the
  homing signature; REACHED unbroken; the starting-values table and the library constants; the honest limit; the
  refusal example), guides 10, 11, 14, 15, `docs/api/` (regenerated: `kinds.md`, `service.md`, `wiring.md`,
  `trapezoid_profile.md`, `README.md`, `all-entities.md`), `roadmap.md`, `build-order.md`, `REMAINING-WORK.md`,
  `PROJECT-BRIEFING.md` (regenerated), this log. **Not touched, deliberately:** the cookbook (A4's recipe), guide 13
  (true), guides 16/17 (A5's), README's map (true), `docs/faq.md` (true), HANDOFF (its A3 lines true), the brief (the
  coordinator's), `MECH-PANEL-requirements.md` (the requirements as written).
- **Prose review (pace rule 4) — every grep RUN and its hits READ before this line** (README.md and docs/**/*.md
  minus `docs/api/` and `docs/internal/chunks/`): `4\.06` 0 hits; `7\.22` chapter 18 :132 and HA-183 :1249 (both true —
  the Red/Green grid's); `stopping distance` guide 14 :431, changelog :497, HA :4217/:4219, roadmap :1211 (the drive's —
  not this); `held at the limit|HELD AT THE LIMIT|held AT` glossary (this round's), roadmap :372 and build-order :632
  ("a push held at the limit" — true as a summary), HA-154 (a claw — not this); `mean slope|converted at the
  move|first segment|conservative (scale|estimate)` 0 hits; `never stalled against its stop` chapter 18, guide 11,
  changelog (this round's); `per rad/s|per output rad` chapter 18 :272/:273 and the changelog (this round's), HA
  :1221/:1225 (the plant's kV and brake damping — true), :1278 (true); `PositionGoal` changelog (this round's);
  `no consumer|nothing in the tree instantiates|isn.t wired|not wired into` changelog :1583/:1592 (a dated BREAKING
  entry — history), guide 14 :351 (this round's); `soft stop` changelog, guide 11 :340, glossary, guide 14 :220,
  roadmap :1741, build-order :1621, chapter 18, `MECH-PANEL-requirements.md` :186/:266 (all true or this round's);
  `homing signature|homeByStall` guide 11, changelog, glossary, chapter 18, HA :4054 (this round's) and changelog
  :303 (A1's words — true); `steepest` chapter 18 and the changelog (this round's); `trapezoid` chapter 18, the
  changelog, guide 14, roadmap :867/:1188 (A1's control list — true), the master plan (history),
  `MECH-PANEL-requirements.md` :97/:168 (true); `printed at boot|print(s|ed)? (it|them) at boot|boot line` — every
  MECH hit scoped (24), the rest other programs' boot lines (true).
- **Gates on the final tree** (`fix/resume/gates.sh`, logs `fix/resume/gates/build1/` and `…/src1/`): see the DoD
  re-check below.

### §2.1's DoD — MECH-A3, re-checked after the review's fix round *(2026-10-22, `date` 09:34:18)*
- [x] This log carries the run's heading, its tip (`fbaf729`), each step as it happened, the 10-15 / 10-18 steps
      read back at their resumptions, and the review's 33 findings with their dispositions — the stopped fix run read
      back from its files and this resumption's own checks (above).
- [x] The host suite, a CLEAN build (229 objects) of the final tree, its binary deleted first: **2514/2514 passed,
      2,796,172 assertions, 3 skipped** (`ulimit -v 4194304; timeout 600`); `-tc='MECH-A3*,guide-18*'` **67 cases,
      3925 assertions**; `ctest -R compile_refusal` **201/201** (201 TUs). (`fix/resume/gates/build1/`, 09:05–09:10.)
- [x] The ARM header gate (**218** headers, rc 0); the CI PROS-free and sim-free greps.
- [x] `src_build_gate.py check` **PASS** (all seven builds, 11/11 TUs each, 0 disallowed warnings); `self-test` **OK
      (235** detector cases, each against a real build); `git diff --stat fbaf729 -- src/ tools/` and `git status
      --short src tools Makefile` EMPTY after both (`fix/resume/gates/src1/`, 09:11–09:33).
- [x] No `PROBE_` / PROTOTYPE / "(proposed)"; the C7 grep empty; the register's direction-1 grep clean.
- [x] Every §3.3 test exists, and every mutation — the brief's §3.3 (M14–M21, M16 at both limits, M36, M39), the run's
      own, the fix round's and the reviewers' — was RUN on the FINAL code with a full build of every dependent object,
      the link checked: **98, 96 RED, 2 GREEN and shown EQUIVALENT** (M18b, X15fresh); none killed by the cap or the
      timeout; every file restored byte-identical.
- [x] Docs in the same change (the round's list above, §4.1's A3 list as before: the changelog, `docs/api/`, HA-182
      only — HA-183/184/186 amended in their Source notes, HA-93's note, no other number registered —, chapter 18 with
      its README row and nav line); the prose review logged (above); `api_doc_tool` self-test / coverage (4673
      entities, 179 headers) / fresh / examples (565 lines) / removability; the staleness self-test and audit clean;
      the briefing regenerated after the binary existed, `check` current, self-test OK.
- [x] Nothing committed; no `pros`; no `/dev/ttyACM*`. **The tree:** 34 files modified + 6 new (the 10-18 list plus
      `include/shulib/control/trapezoid_profile.hpp`, `docs/api/trapezoid_profile.md`, A1's
      `test/mechanism_fixtures.hpp` and `test/mechanism_kinds_test.cpp`), uncommitted, in `.claude/worktrees/mech-a3`.
- **For the coordinator** (unnumbered; the 10-18 list above stands except where this one says otherwise):
  - **Main has moved since `fbaf729`**: `ceb17c0` (CAL1 Part A1), `fd55ffc` (SER1's record), `71b4764` (VIS1's brief
    answered — HA-192..HA-203 HELD); and main's working tree holds uncommitted edits (`HANDOFF-remaining-work.md`,
    `RELEASING.md`, `test/CMakeLists.txt`, `tools/briefing_status.py`, `tools/doc_staleness_audit.py`,
    `tools/release.py`, a new `tools/suite_run.py`). Files both main (to `71b4764`) and this run changed: `README.md`,
    `docs/changelog.md`, `docs/hardware-assumptions.md` (main's held line now ends `HA-192..HA-203` — after the merge
    the briefing's next free is **HA-204**, not this tree's HA-192 — and main's 2026-10-18 K.7.3 note sits after
    SER1's paragraph, beside where this run's MECH-A3 paragraph follows A2's), `docs/internal/build-order.md` (the first
    bold `Next:` line: a certain conflict — keep both sentences), `docs/internal/HANDOFF-remaining-work.md` (also
    uncommitted on main), `docs/internal/REMAINING-WORK.md`, `docs/roadmap.md`, `mkdocs.yml`, and the generated
    `docs/api/README.md`, `docs/api/all-entities.md`, `docs/internal/PROJECT-BRIEFING.md` — regenerate those after the
    merge (`api_doc_tool.py generate`; the briefing only after `rm -f build/test/shulib_tests` and a fresh build).
    `test/CMakeLists.txt`: NOT touched by this run (its globs pick up the new tests).
  - **The landing dates say 2026-10-22** (the changelog heading, guides 10, 14, 18, roadmap ×2, build-order ×2 — its
    MECH entry adds "reviewed and fixed 2026-10-22" —, REMAINING-WORK): change them again if the commit lands another
    day.
  - **Q26's plan commit:** homing by switch AND by stall is BUILT (P2e; chapter 18's which-to-choose table), so the
    roadmap's F14 row (:148, "the lift-homing switch-or-stall decision … is still unanswered"), roadmap :839 ("the
    still-open lift-homing switch-or-stall answer") and the master plan's open question A (:585, "the scoring-primitives
    chunk … which writes the homing routine") are stale until that commit records it. F3's `liftToLevel` (roadmap
    :1748 now) as before.
  - **Two design questions this round surfaced and did NOT decide:** (a) whether a team should be able to tune the
    three LIBRARY constants (the hold's integral clamp, the speed integral's window and clamp) per mechanism — today
    only an edit to `service.hpp` changes them, for every mechanism (HA-182's Contained says so); (b) whether, once
    HA-93 is measured on a real motor, a homing signature its push cannot reach should be REFUSED at build (A3-D23
    declined it while that current shape is invented).
  - **A hardware belief leaned on outside HA-182** (unnumbered, as the go asks): chapter 18's homing-signature advice
    — a 3 V homing push draws about 3/12 of the 2.5 A limit at its stop, so state 0.45 A to start — rests on HA-93's
    invented current shape (|V − kV·ω| × 2.5 / 12). It is worded as the host test plant's number, to be checked on the
    bench; HA-93's entry now names the reliance. No HA number was registered for it.
  - `sizeof(MechanismService)` is **9 640 B** on ARM `-Os` (A3-D17 re-measured; was 9 448) — for A5.
  - G3a's validator mirrors `spec.hpp`'s refusals by sentence: still **70** (the review added none).
  - The mutation driver now runs a whole `cmake --build` per mutation (line-preserving, so the doc gate holds) — the
    10-18 hand-on's last bullet is superseded. Its scratch: `.claude/coordinator/scratch/mech-a3/fix/` (the stopped
    run) and `fix/resume/` (this resumption); `fix/muttree` is a scratch copy of the final tree, every mutated file
    restored and `cmp`-equal to the worktree's.

### 10:04 (2026-10-22) — Verification-round findings and their disposition — started *(`date`: Thu Oct 22 10:04:52 AM EDT 2026)*
- A fresh verifier returned **five minor findings** (V-1 … V-5 below: the homing note's first-tick current, the soft
  stop latch's clear in `resetBehaviour()`, an Arm `clampedTo()` goal's units, the changelog's missing CHANGE for a
  measured table's starting speed, guide 10's "only numbers" sentence). Each is re-checked against the tree before it
  is fixed; the disposition table follows when the round's gates are green. Scratch: `.claude/coordinator/scratch/
  mech-a3/vfix/`.

### 10:04–10:50 (2026-10-22) — Verification-round findings and their disposition *(`date` at this entry's writing: Thu Oct 22 10:45:37 AM EDT 2026)*
- **Re-checked FIRST, each against the tree** (every one REAL): V-1, the verifier's probe re-run under the memory cap
  (`verify/probes/p_v_homeamps.bin`): after a jog pinned at 8 V, `it drew at most 1.67 A, its homing signature needs
  1.50 A` beside "it never stalled" — reproduced; V-2, `p_v_latch_real.bin` / `p_v_latch_v2.bin`: the second push's
  first tick 8.00 V and 2 soft stops on the real header, **1.58 V** and 1 on the mutant — reproduced; V-3, the tests
  read: `mechanism_kinds_test.cpp` checked no goal's `.units`, and no position case found an ARM's `clampedTo()`;
  V-4, `git show fbaf729:include/shulib/mechanism/kinds.hpp` — "for a table its first segment's" — and the changelog's
  CHANGED section had two bullets, neither this; V-5, guide 10 :452-456 read — the "only numbers" sentence as quoted.
- **V-1 went wider than its suggestion — A3-D26 above.** Skipping the first tick alone was built in a probe first and
  left the same contradiction from the push's OWN current: `vfix/probes/p_brake.cpp` (a home begun while the arm still
  swings up) — the push brakes the swing at 1.29 A (1.42 A after 30 ticks of jog), 1.00–1.10 A after its first
  tick, while its current when STOPPED is 0.62 A. The line now counts only the push's samples at or below its
  signature's speed, and says `it never slowed to its homing signature's speed -- did it reach its stop?` when there
  are none; `vfix/probes/p_newnote.cpp` on the fixed header: 0.62 A when stopped after a pinned jog (twice), after a
  braked swing and on a wedge; the never-slowed line for a 0.15 s home that could not reach its stop.

| # | Finding (minor, each) | Disposition | Proof (mutations on the final code — each RED unless said) |
|---|---|---|---|
| V-1 | the homing TIMED_OUT note counted the PREVIOUS behaviour's current (first tick); its reset unpinned | **FIXED at the root** (A3-D26): the push's own samples only (`homePushed_`, set by its first command), and only when stopped (the detector's own speed test); a never-slowed line; `home()` resets both. Tests: the existing note case's text; a NEW case — a jog pinned at 8 V before each of TWO homes (0.62 A each time), a home that brakes an upward swing (0.62, not 1.29), a home stopped on a wedge then a 0.15 s home that never slows (the never-slowed line, no "drew at most"). Docs: chapter 18, guide 11, the changelog, HA-93's note, `home()`'s doc comment | V1homeampsreset (the verifier's V1, re-anchored), V1bpushedreset, V1cfirsttick, V1dslow, V1enever, V1fsetpushed, V1gvelsource |
| V-2 | the soft stop latch's clear in `resetBehaviour()` unpinned | **FIXED (tests)**: a push held at the top ENDED by `release()`, by a move, and by a STALLED (a 0.5 A signature; the arm jammed short of the top after the latch, `... while held to move`) — then a move away and a push the same way: its first tick commands **8.0 V exactly**, `softStops() == 2`, held within 0.5° of the limit. The code was right | V2resetlimitdir — RED on all THREE ends (`end := 0, 1, 2`). V2bholdatclear (`holdAt()`'s clear removed) **GREEN, EQUIVALENT**: `holdAt()`'s two callers are `beginHold()` (after `resetBehaviour()`) and a move's TIMED_OUT (Move mode); `limitDir_` is read only in `tickJog()`, reached only through `jog()`'s Mode::Jog, which passes `resetBehaviour()` whenever it enters from another mode — and `holdAt()` always leaves Mode::Hold. Kept as a defensive clear |
| V-3 | an Arm `clampedTo()` goal's `units` unpinned | **FIXED (tests)**: `find(kGreen5.clampedTo(175.0))` and a same-named arm's both find this arm; A1's words test checks the units of six goals (the arm's preset, `to()`, `clampedTo()`; the lift's `to()`, preset, `clampedTo()`) | V13armclampunits — RED in BOTH tests |
| V-4 | the changelog did not list a measured table's STARTING speed changing (first → steepest segment) | **FIXED**: a CHANGED bullet — `Lift::spec().position.maxVelocity` / `maxAcceleration` (and so the boot line's cruise) from the steepest segment, the same or faster (2.5× on the test's kTilt2); a stated `.tuning` value, a spool's and a linkage's unchanged | — (A3-D21's own test, `mechanism_position_test.cpp` kTilt2's `maxVelocity`) |
| V-5 | guide 10: "the only numbers the library supplies" missed the three library constants | **FIXED**: scoped "PER MECHANISM", and the three loop constants named as the library's own, not in `.tuning` (chapter 18) | — |

- **Every earlier mutation re-run on THIS round's code** (`vfix/mlist_all.py`: the 98 of `fix/mlist_final.py` and
  `fix/resume/mlist_resume*.py`, the later definition winning) — `tickHome()` and `home()` changed, so no verdict is
  carried over unchecked: **94 RED, 2 GREEN and EQUIVALENT** (M18b, X15fresh — as before). `Nhomeamps` is gone with
  its line (superseded by V1cfirsttick / V1fsetpushed); `R11homewatchreset` named `homePushAmps_` in its context and
  was re-anchored (`vfix/mlist2.py`, R11homewatchreset2): **RED**. **This round's own:** 10, 9 RED, V2bholdatclear
  EQUIVALENT. **In all: 107 distinct mutations on the final code — 104 RED, 3 GREEN and shown EQUIVALENT**; none
  killed by the cap or the timeout; every one `restored=True`; every verdict with ALL dependents recompiled (8 for
  `service.hpp`, 16 for `kinds.hpp`/`spec.hpp`/`wiring.hpp`, 5 for the plant) and the binary re-linked, by a whole
  `cmake --build` (`vfix/mut.py` = `fix/resume/mut.py` re-pointed; `vfix/mut/summary.txt`, 10:16–10:33). The tree:
  `fix/muttree`, rsynced from the worktree before its baseline build (10:15:52; 229 objects, the filter green) and
  again at 10:18 (the docs edited since), before the 96; after the last, equal to the worktree file by file but for this log.
- **The A3 tests now:** `-tc='MECH-A3*,guide-18*'` **68 cases, 3969 assertions** (was 67 / 3925: one new case; the
  latch subcase, the find() and words checks added to existing ones).
- **Docs moved in this round (pace rule 3):** `docs/changelog.md` (part 3's homing line, a CHANGED bullet, TESTED and
  its counts), `docs/guide/18-mechanisms.md` (the homing-signature rule: the line's number is the push's when stopped,
  the never-slowed line), `docs/guide/11-reading-the-diagnostics.md` (TIMED_OUT's homing notes), `docs/guide/10-the-api.md`
  (the "only numbers" sentence), `docs/hardware-assumptions.md` (HA-93's note: "when stopped"), `docs/api/`
  (regenerated: `home()`'s doc comment, line numbers), `docs/internal/PROJECT-BRIEFING.md` (regenerated), this log
  (A3-D23's note, A3-D26, A3-D17 re-measured). **Not touched, deliberately:** the glossary (its `homeByStall` sentence,
  "a gentle push stalls at a small current", is true), roadmap / build-order / REMAINING-WORK / HANDOFF (their A3
  lines name no count and stay true — read), `test/CMakeLists.txt` (no new file), the brief (the coordinator's).
- **Prose review (pace rule 4) — each grep RUN and its hits READ before this line** (README.md, docs/**/*.md minus
  `docs/api/` and `docs/internal/chunks/`, and `include/shulib`): `drew at most` — guide 11 :290, the changelog :52,
  chapter 18 :203, `service.hpp` :1121 (all this round's); `what the push drew|push drew|its push drew` — guide 11,
  chapter 18, the changelog, HA-93 :4057, `service.hpp` :847/:1293 (this round's); `never slowed` — the same four
  (this round's); `first segment` — the changelog's new bullet only (A1's words gone, A3-D21); `steepest` — chapter
  18 :92/:213, the changelog :37/:108, `service.hpp` :66/:810/:1164, `kinds.hpp` (all true); `only numbers|library
  supplies` — guide 10 only (this round's); `library supplies|library's numbers|library's own numbers|replaced one
  by one|one by one with|\.tuning replaces` — chapter 18 :16/:296, the changelog :78, HA :161/:1263, `service.hpp`
  :90 (each already split by finding 24 — true); `turns away|turned away|or ends|push ends` — the changelog :62,
  `service.hpp` :1053/:1057 (true), HA :1157 (a homing push by switch — not this); `homePushAmps|homeStoppedAmps|
  homePushed_` — `service.hpp` and this log's A3-D23 (now annotated); `98 mutation|96 turned|67 cases|3925` — none
  outside this log.
- **Gates on the final tree** (`vfix/gates.sh final all`, logs `vfix/gates/final/`): the binary deleted, the briefing
  generated, a CLEAN build (**229 objects**), the suite **2515/2515 passed, 2,796,224 assertions, 3 skipped** (capped,
  20 s), the A3 filter as above, the briefing generated after the binary existed, `check` current, self-test OK (24);
  `ctest -R compile_refusal` **201/201** (201 TUs); `api_doc_tool` self-test OK, coverage (4673 entities, 179
  headers), fresh, examples (565 lines), removability; the staleness self-test OK and the audit clean; PROS-free,
  sim-free; the ARM header gate (**218** headers, rc 0); no `PROBE_` / PROTOTYPE / "(proposed)"; C7 clean; the
  register's direction 1 clean; `src_build_gate.py check` **PASS** (seven builds, 11/11 TUs each, 0 disallowed
  warnings); `self-test` **OK (235** detector cases, each against a real build, 10:41–10:56); `git diff --stat fbaf729
  -- src/ tools/` and `git status --short src tools Makefile` EMPTY after both (the self-test's planted `Makefile`
  edit was seen mid-run, 10:50, and is gone — the gate restores it). Every gate rc 0 (`vfix/gates/final/summary.txt`).

### §2.1's DoD — MECH-A3, re-checked after the verification round *(2026-10-22, `date` 10:57)*
- [x] The log carries the verification round's start (10:04), its five findings re-checked and disposed (above), the
      A3-D26 delta and A3-D23's note, and the re-measured size (A3-D17).
- [x] The host suite on a CLEAN build (229 objects), its binary deleted first: **2515/2515 passed, 2,796,224
      assertions, 3 skipped** (capped); `-tc='MECH-A3*,guide-18*'` **68 cases, 3969 assertions**; `ctest -R
      compile_refusal` **201/201**.
- [x] The ARM header gate (218 headers, rc 0); the CI PROS-free and sim-free greps; `src_build_gate.py` check PASS and
      self-test OK (235); src/, tools/ and the Makefile untouched since `fbaf729`.
- [x] Every mutation on the FINAL code with a whole build of every dependent object, the link checked: **107 — 104
      RED, 3 GREEN and shown EQUIVALENT** (M18b, X15fresh, V2bholdatclear); none killed by the cap or the timeout;
      every file restored byte-identical.
- [x] Docs in the same change (the round's list above); the prose review logged (above); `api_doc_tool` self-test /
      coverage / fresh / examples / removability; the staleness self-test and audit clean; the briefing regenerated
      after the binary existed, `check` current — and `check`, the staleness audit and C7 re-run after this entry
      was written (below).
- [x] Nothing committed; no `pros`; no `/dev/ttyACM*`; no `src/` file. **The tree:** 34 files modified + 6 new —
      the same 40 files as at 09:34 (`git status --short` against `fix/resume/a3_changed.txt`: no difference) —,
      uncommitted, in `.claude/worktrees/mech-a3`.
- **For the coordinator** (unnumbered; the 09:34 list stands except where this says otherwise):
  - **V-1 was fixed wider than the verifier suggested** (A3-D26): the line's number is the push's own current WHEN
    STOPPED, and a push that never slowed says so — the narrow fix (skip the first tick) left the push's own braking
    current (1.29 A at speed) in the same contradiction. The homing DETECTOR still takes the first tick's sample, like
    every stall watch: a home can trip at most one tick sooner when the arm is already at its stop (A3-D26 says why
    it was left).
  - The mutation count everywhere it is stated (the changelog's TESTED, the DoD) is now **107 / 104 / 3**.
  - No new hardware belief; no HA number registered (chapter 18's 1.67 A / 1.29 A are the host test plant's,
    labelled so).
- **Re-run after this entry was written** (10:59): `briefing_status.py check` current (capped); the staleness audit
  clean (30 live docs); C7 clean; `api_doc_tool` check-fresh and check-removability OK; `git status --short src tools
  Makefile` and `git diff --stat fbaf729 -- src/ tools/` EMPTY.

### The coordinator's merge verification — MECH-A3 (2026-10-22, main `c09e022` + the squash of `mech-a3`)
- **Audit before the merge:** every file equals its newest mutation backup under `scratch/mech-a3/mut/` (one older probe backup,
  `probes/service.hpp.bak` of 2026-10-15, predates later edits — not a mutation); `git diff --stat fbaf729 -- src/ tools/ Makefile`
  empty; no stray root file; no merge in progress. The transport commit `36fbbe8`, then `git merge --squash mech-a3` (base `fbaf729`).
- **Five conflicts:** `docs/api/README.md`, `all-entities.md` and the briefing (regenerated); the changelog (A3's entry first, then
  CAL1 Part A1's and the earlier ones); build-order's `Next:` line (a word-level three-way merge against `fbaf729` — main's CAL1, plan
  and tooling clauses and A3's clause all kept). The roadmap carries A3's "Now" paragraph (from the run).
- **Gates on the merged tree** (`.claude/coordinator/logs/mech_a3_merge_verify.out`): host build with the doc gates (the bounded suite
  run's, the briefing's and the staleness audit's self-tests all OK); suite **2,545 | 2,545 passed, 3 skipped**; `src_build_gate.py`
  self-test OK (235 cases) and check **PASS, all 7 builds**; ARM header gate over **223** headers clean; CI greps empty; api
  check-fresh, the briefing, removability and the staleness audit clean; the compile-refusal harness **201 of 201** (A3's S70 added).
- **The coordinator's hand mutations** (`muts_mech_a3_main.json`, on `mechanism/service.hpp`; each on a fresh link, the whole suite
  under a 4 GiB / 300 s cap, restored and hash-checked): **10 of 10 caught** — MC1 (a move before homing not refused NOT_HOMED —
  caught by an abort), MC2 (a clamped target's Warn not logged), MC3 (N2: out of time it holds the TARGET), MC4 (the soft stop ignores
  the stopping distance), MC5 (the speed integral's cap removed), MC6 (a reverse flywheel's kS sign), MC7 (absolute homing never picks
  the turn nearest the middle — A3-D2), MC8 (the homing note counts samples from before the push — V-1), MC9 (the speed output clamped
  at 6 V), MC10 (the hold target not clamped inside the travel limits) RED. Restored tree 2,545 | 2,545.

---

## MECH-A4 — autons, the driver loop, the direction judgement, the route binder

- **Started:** Thu Oct 22 05:24:28 PM EDT 2026 (`date`).
- **Tree at start:** `43ac361` (main: MECH-A1 `fd36e07`, SER1 `2506067`, MECH-A2 `98392b2`, CAL1 Part A1 `ceb17c0`,
  MECH-A3 `43ac361` merged and verified; the plan at `71b4764`, the tooling at `c09e022`), clean, in the git worktree
  `.claude/worktrees/mech-a4` (branch `mech-a4`) — NOT the main working tree.
- **Go:** the team lead, 2026-10-08: "Accept all, go" — N1–N13 all as recommended (a), the runs strictly in order,
  each after the last LANDS; 2026-10-15/22: "move faster — that does not mean skip steps"; "we are running out of time
  but that doesnt mean we should cut corners". The coordinator's workflow launched MECH-A4 ONLY. Do not commit; no
  `pros` command; no `/dev/ttyACM*`. Mutation backups and probes under `.claude/coordinator/scratch/mech-a4/` (never
  `/tmp`, never the repository root). **Memory safety (2026-10-15):** every test binary — the suite, a filter, every
  mutation, every looping probe — runs as `(ulimit -v 4194304; timeout 600 <cmd>)`, the suite through
  `python3 tools/suite_run.py green` where a verdict is enough; a mutant killed by the cap or the timeout counts as
  CAUGHT (said which); the briefing is regenerated only after `rm -f build/test/shulib_tests`.
- **Reading (the team lead's ruling of 2026-10-22 for this run):** the brief's header and answers, the re-check's
  amendments (which override the body), N1–N13, the Decisions, §6 coordination, §7, §8 landmines, §9 and "Found while
  reading" IN FULL; every section, row and test list naming MECH-A4 or its files (§1.1's auton and driver forms, §1.9,
  §1.10, §1.11, §1.16, §2's A4 row, §2.1, §3.0, §3.4, §4.1–§4.3); the rest by heading. One reviewer reads the whole.
- **Scope:** §2's A4 row — §1.9 (the Routine's mechanism steps, ONE eager call per step, the constructor and the
  refusal templates, `MechanismStepFailed`, `RoutineResult::mechanismVerdict`, `AutonContext::mechanismService`, the
  `runAuton` service overload — API 2.5); §1.10 (`mechanism/driver_inputs.hpp`, the loop's one body + the service
  overload — every T4 Part 2a protection test unchanged); §1.11 `mechanism/direction_check.hpp` (the judgement; its
  page is MECH-A6); §1.16's `runner/mechanism_routes.hpp` and the registry's doc; N4's "superseded" notes. `src/`:
  ONE `case` each in `src/programs/{competition_run,auton_tester_run}.hpp`, nothing else. Hand-ons carried: **M35
  proper** (A2-D7: `runAuton`'s service overload calls `enterAutonomous()`); **A2-D18** (REFUSED's route-marker half:
  `MechanismOutcome::Refused`'s doc comment, the changelog, guide 11's verdict list); A3's "not done here" (the driver
  toggle's 5 s at-speed window is HA-182's, A4's to add); A3-D4 (`home()` on `neverHomed` is a precondition); A3-D22
  (A4 is `find(PositionGoal)`'s first caller). NOTHING of A5–A6 (robot two's tables and programs; station 13).
- **In flight beside this run (not touched here):** CAL1 Part A2a (`calibration/calibrate_program.hpp`,
  `hal/pros/calibrate.hpp`); VIS1-A1/A2 (`hal/vision_frames.hpp`, `hal/pros/ai_vision.hpp`, `hal/device_registry*`,
  `hal/device_provider*`, `hal/fake/fake_device_provider.hpp`, `config/robot_config.hpp`, `config/camera_config.hpp`,
  `robot/robot.hpp`, `vision/`, `sim/camera_plant.hpp`); the L6 and IMU1 briefs (L6 adds a Routine step AFTER this run:
  A4's additions to `routine.hpp` stay in their own delimited block).

### 17:24 — start
- Read `docs/internal/RESUMING.md` (standing rules, pace rules 1–8), the brief as ruled above, and `## MECH-A1` /
  `## MECH-A2` / `## MECH-A3` above (their deltas and every "For MECH-A4" hand-on). This heading appended before any
  code.

### 17:24–18:08 — the baseline, the overlay's A4 slice against the tree, the code, the tests *(`date` 18:07:54)*
- **Baseline at `43ac361`** (`cmake -S test -B build/test`; `rm -f build/test/shulib_tests`; the briefing regenerated; the
  full doc-gated build at -j8, 232 objects, 45.9 s wall; the suite under `ulimit -v 4194304; timeout 600`): **2545/2545
  passed, 2,798,203 assertions, 3 skipped** — main's merged numbers. Driver: `scratch/mech-a4/build.sh` (logs
  `scratch/mech-a4/logs/`).
- **Read in the overlay** (`proposed4/shulib/`: `chassis/routine.hpp`, `competition/{auton_registry,match_runner}.hpp`,
  `teleop/teleop_loop.hpp`, `runner/{command_registry,manifest,mechanism_routes}.hpp`, `mechanism/{driver_inputs,
  direction_check}.hpp`; `proposed4_src/`), the probes `p5/probe3_auton.cpp`, `probe3_driver.cpp`, `probe3_direction.cpp`,
  `probe3_robot_two.cpp`, `probe5_routes.cpp`, `probe4_forms.cpp`, `neg5/` and their `out5/` outputs. **The overlay's base is
  not this tree's** (A2 and A3 rewrote the service, A3-D22 added `PositionGoal::units`, T4 Part 3 rewrote a comment block of
  `auton_registry.hpp` after the overlay was taken): A4's slice is re-written onto the tree, not copied — the deltas below
  (A4-D1 …). No `PROBE_` switch exists in the A4 slice; no "PROTOTYPE"/"(proposed)" marker was carried.
- **Code** (each file's banner says what it holds now): `chassis/routine.hpp` (the MECH-A4 block: the context constructor,
  the steps, the four refusal templates, `lastMechanism()`, the appended `MechanismStepFailed` and `mechanismVerdict`, the
  auton's words); `competition/auton_registry.hpp` (`AutonContext::mechanismService`, appended with a default; the word);
  `competition/match_runner.hpp` (ONE body `detail::runAutonImpl`, COMP-a's overload forwarding with no service, the service
  overload — `enterAutonomous()` first); `mechanism/driver_inputs.hpp` (NEW); `teleop/teleop_loop.hpp` (ONE body
  `detail::driverLoop`, the service overload, step 6½, the driver file's words); `teleop/mechanism_binding.hpp` (N4's
  "superseded" notes); `mechanism/direction_check.hpp` (NEW); `runner/command_registry.hpp` (`CommandEntry::doc`, the
  four-argument `on()`, the banner's stale "asynchronous mechanism operations are chunk T3's" refreshed — the brief's
  "Found while reading"); `runner/manifest.hpp` (a registered id's doc); `runner/mechanism_routes.hpp` (NEW);
  `version.hpp` (API **2.5**); `src/programs/competition_run.hpp` :560 and `auton_tester_run.hpp` :272 (ONE `case` each).
- **The suite after the library half, before any new test:** 2545/2545 green (every existing case — T4 Part 2a's
  protections, COMP-a's runAuton, the F8/F10 pins, the manifest golden — unchanged and passing).
- **Tests** (new): `test/mechanism_auton_rig.hpp` (robot x on the A2 tank plant + the §1.12 models, the program's pacer
  chain, the control; robot two), `test/mechanism_driver_fixtures.hpp` (the driver files' controls, verbatim),
  `mechanism_auton_run_test.cpp` (P1 through the REAL runAuton — **measured: drive T+0.92..T+2.72, the arm's worst |error|
  1.16°, the intake 0 V at T+2.00 with the drive running, lift 9.70 in and flywheel 3000 rpm at the drive's end, 180 paces /
  180 service ticks** — the brief's P1 to the hundredth; P5-run; the control; the deadline cutting a blocking step; D1/M35;
  the leftover claim; a refused entry; the guard's span; COMP-a), `mechanism_auton_steps_test.cpp` (every form; the verdict
  policy, each verdict; ABSENT's three sentences; REFUSED; the start forms; waitUntilDone; the frozen constructor;
  MechanismFailed kept), `mechanism_driver_loop_test.cpp` (P5d, P8, the order pin, both-buttons / a drop, N10's stall
  supersede, P2i, U5, P9c, the loop's and bind()'s preconditions, the lists), `mechanism_direction_check_test.cpp` (P7:
  every finding verbatim, the generated path, changeHere's sources, the pneumatic answers, the sign, noise, the
  preconditions), `mechanism_routes_test.cpp` (P14 through the REAL PathRunner in the REAL runAuton; robot two's route; nine
  bad-arg markers; a failing step; every action; both manifests byte-exact; the doc; the binder's ids),
  `mechanism_step_forms_test.cpp` (the positive TU, compiled into the suite, the counted autons run). Edited:
  `routine_signature_pin_test.cpp` (`MechanismStepFailed == 5`, `mechanismVerdict`'s type, the floor 5),
  `f8_signature_pin_test.cpp` (`CommandEntry::doc`, the four-argument `on()`, the floor 5), `competition_run_test.cpp` (the
  word), `auton_tester_run_test.cpp` (the word and the longest line — A4-D9). **51 MECH-A4 cases, 517 assertions, green.**
- **The harness** (`test/compile_refusals/`, grown by 45 TUs → 246): the Routine's four named `static_assert`s (n05, n53,
  n54, n55) and three kind variants; every `refused::` function of `driver_inputs.hpp` — the twelve buttons by name (n17 R1,
  m2 Right multi-line, `d_button_*_twice` the other ten), the cast byte, n18–n22, n24, n33, n34, n48–n51 and their edge
  variants (a hold at 0 V and at −6 V, two presets to one target, a toggle on a constant speed, a homing with no time
  limit, a homing on a never-homed mechanism, autonOnly with a control, 25 controls, the four kind refusals). **45/45 PASS**
  (`ctest -R compile_refusal_(…)`); the judge caught two of the first draft refused for the WRONG reason (`Control` not
  a word — fixed to `mech::Control`). n56, n57 (DriverFile) and n31 stay MECH-A5's (A1's D18).

### 18:08–18:38 — the documents (§4.1's A4 share, pace rule 3) and the prose review (pace rule 4) — every grep RUN and its hits READ before this line *(written during mutation run 1, which touched only headers; `date` 18:38)*
- **Written:** `docs/changelog.md` — the **API 2.5** section at the top (ADDED: the steps, the context constructor,
  `MechanismStepFailed`, the four refusal sentences, `AutonContext::mechanismService` and `runAuton`'s service overload,
  the driver file's controls and `runDriverControl`'s service overload, the route binder, the F8-additive per-id doc, the
  direction check's judgement, the words; CHANGED (documentation only): REFUSED's second meaning — A2-D18's hand-on —, T3's
  bindings superseded (N4); "what you must do"); `include/shulib/version.hpp` (2.5 and its history paragraph);
  `docs/api/` regenerated (three new pages: `driver_inputs.md`, `direction_check.md`, `mechanism_routes.md`; `mkdocs.yml`'s
  GENERATED nav by the tool); **the mechanisms chapter** (`docs/guide/18-mechanisms.md`: the banner's "what exists"; four
  new sections — *In an auton — one line per step* (the steps table, the START forms, P1's measured numbers, the verdict
  policy, the stop line, the build-time sentences, the conditions refused at the step, the frozen constructor's
  precondition, the never-ticked service), *On the driver's buttons* (the controls, the most recent input wins, a press
  must be seen, every build refusal with U5's, the loop's entry / tick / disable), *As route markers* (the one-call
  registration, what blocks, the args, REFUSED, the docs), *The direction check — what it judges*; *The honest limit*
  rewritten with D11's measured 11.74 V / 2.45 A and "a `then()` action must not block"; the jam section's driver clause
  in the present tense; *Reading a refusal*'s list); **guide 10** (the bindings' "superseded" note, the mechanism
  section's "what exists" box, the route-id sentence, the direction check's sentence, a new *In an auton, on the
  buttons, on a route* map); **the cookbook's recipe** (NEW `docs/cookbook/07-mechanisms.md`, "An arm with presets, held
  while the robot drives" — A4-D17); **`docs/vexbuilder-integration.md` §4.2.3** (the doc field's new meaning, every
  mechanism's derived ids with robot two's six manifest lines verbatim, their args, what blocks, why a picker derives
  the list; §4.3's row); **guide 11** (A2-D18: REFUSED's route-marker half; the routine's mechanism stop line and its two
  own brackets; the binder's REFUSED line; the counted-done sentence and `MECHANISM_STALLED`'s driver clause in the
  present tense); **HA-182** (the driver toggle's 5 s window, A3's hand-on — entry and row); the internal pointers
  (`build-order.md`'s MECH entry and its first bold `Next:`, `roadmap.md`'s "Now" paragraph, the MECH row, the F8 and
  F10 rows, the button-bindings row; `REMAINING-WORK.md` row 12; `HANDOFF-remaining-work.md`) — each says BUILT, its
  review, verification and merge to come, never "landed".
- **The greps** (`grep -rnF <pat> docs/ README.md`, public documents; `docs/internal/` and the generated `docs/api/`
  read separately): §4.2's A4 set — `Routine` (127 hits), `then(` (62), `AutonContext` (7), `runAuton` (26),
  `runDriverControl` (34), `MechanismBinding` (10), `ToggleBinding` (12) — and §4.1's re-check additions — `hold(` (14),
  `toggle(` (6), `MechanismFailed` (6), `CommandRegistry` (10), `manifest` (35); plus `fourth part`, `next part`,
  `auton step`, `mechanism step`, `route marker`, the chapter's old title, `2.4`. **Stale, FIXED:** guide 9 :18-24 ("used
  for real, twice" — now three times, with 2.5's additions and the steps' unfrozen status), :148 (the stop-cause list
  lacked `MechanismStepFailed` and `mechanismVerdict`), :270 (a declared mechanism needs no `then()`); guide 14 :217-224
  ("three of its six parts", "the auton steps, driver controls … are its next parts") and :409-416 (mechanism buttons
  — the driver controls now exist); the glossary's `route id` (a marker runs the step) and `step (recipe)` (a mechanism
  step); the guide index's chapter-18 row; cookbook 06 (T3's two recipes — a "superseded, not removed" note, N4); the
  README's source layout (`mechanism/`, `runner/`, `teleop/`); `runner/command_registry.hpp`'s banner;
  `mechanism/route_ids.hpp`'s and `mechanism/service.hpp`'s future-tense sentences; guide 11 and guide 10 (above).
  **Read, true, left:** guide 3 :220/:391, guide 16 :24/:43/:92/:141, guide 17 (every `runAuton` hit — COMP-a's call,
  unchanged; guide 16's quoted auton builds `Routine r{ctx.chassis, …}`, the team's file, which MECH-A5 changes with the
  file — the brief's U3), cookbook 01 :32, guide 9 :104 (`startAt`), guide 10 :194-222 (the loop's drive half,
  unchanged), the hardware register's T3 entries (HA-157's, the ToggleBinding entries — true of T3's path, which stays),
  the changelog's dated entries (history), `legacy-command-vocabulary.md` :162, the roadmap's T2/T4 rows. Every
  `hold(` hit is the drive's `Routine::hold` / `Chassis::hold` (unchanged); `manifest` hits outside the VexBuilder
  contract and guide 10 are the drive-protection and SD-log meanings or dated history.

### Deltas from the brief (MECH-A4), each with its reason
- **A4-D1** the overlay's A4 slice (`proposed4/`) was RE-WRITTEN onto the tree, not copied: A2 and A3 rebuilt the service
  (A2-D1's every-body-inside-its-class, A3-D10/D22's `find()` by kind, units and preset VALUE, `PositionGoal::units`), and
  T4 Part 3 rewrote a comment block of `auton_registry.hpp` after the overlay was taken (only the overlay's MECH additions
  were carried). No `PROBE_` switch, no "PROTOTYPE"/"(proposed)" marker (`grep -rn PROBE_ include/` empty).
- **A4-D2** (the route binder) a `<lift>_home` marker looks its mechanism up as a LIFT, by the spec's units (the overlay
  built an `Arm` stub for every home: since A3-D10 the service never finds a lift through an arm's declaration, so every
  lift's home marker would have ended ABSENT — RT4 RED); a preset marker's goal carries its `units` (A3-D22; RT5 RED).
- **A4-D3** `waitUntilDone` collects only the step THIS routine last began on that mechanism (`startedSeq_`, the slot's
  `stepsBegun()` at the begin): none begun — a precondition naming the start forms; one REPLACED from outside since —
  CANCELLED, said. The overlay collected whatever `lastStep()` held: a stale verdict of an earlier run's step, or the
  verdict of the command that replaced it (R8, R9 RED).
- **A4-D4** `stop()` reports what it did: the slot's `stop()` bumps the command number exactly when it acts, so a stop that
  did nothing is REFUSED (another owner holds the claim) or ABSENT (not built), each with a MECH line in the step-line
  format, and the chain stops. The overlay recorded SUCCEEDED when another owner held the claim (R6b RED).
- **A4-D5** a step whose mechanism the service cannot find says one of THREE sentences: the service is not bound; no
  mechanism of that name on this robot (the brief's verbatim); or one of that name that is not what the step names
  (another robot's same-named declaration — a different kind, units, preset or position; landmine 6, A3-D22). The
  overlay said "not on this robot" for all three (R7 RED).
- **A4-D6** `runAuton`'s service overload calls `enterAutonomous()` INSIDE the one body, right after the two
  preconditions and BEFORE the refusals (the overlay called it before the body, i.e. before the preconditions, and logged
  a second, generic Warn — the service's own `releaseLeftover` line already names each mechanism): a refused entry still
  stops what driver control left (tested); no duplicate line (MR2 RED). No `bound()` precondition: an UNBOUND service has
  no mechanisms, and every mechanism step then ends ABSENT saying "the mechanism service is not bound" while the drive
  steps run — a precondition would have cost a match's whole auton for a boot problem the boot already reports.
- **A4-D7** the driver loop's entry: the overlay's extra generic `TEL` Warn for a released claim dropped — the service
  logs one `MECH` line per mechanism, by name (guide 11 documents both lines).
- **A4-D8** a START form stops the chain only on ABSENT, REFUSED or NOT_HOMED — the brief's words; the overlay stopped on
  anything but Running/Succeeded (equivalent at a start; written to the brief) (R4c, R4d, R4b RED).
- **A4-D9** `test/auton_tester_run_test.cpp` edited (not in the A4 row's list): the Auton Tester's four-line render
  checked with the new LONGEST stop word ("mechanism step FAILED") and its `routineStopWord` for the new cause pinned —
  the src line A4 adds was otherwise untested (S2 RED with it).
- **A4-D10** `driver_inputs.hpp` beyond the overlay: `Control::units` (an arm's or a lift's control); `DriverInputs::bind()`
  checks every control against the robot the SERVICE is bound to — the kind, the units, a named preset at its VALUE, a
  constant inside the limits / free speed, a pneumatic position by NAME, a homing method — and binds ONCE (the overlay
  bound by name only: a driver file built for another robot's same-named mechanism would command it — D6, D13 RED); the
  homing words take an `Arm` or a `Lift` only, and are refused BY NAME on the other kinds
  (`homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift`; the overlay's template took any declaration and misread a
  power mechanism's unset homing as "never homed"); a cast byte names
  `a_control_names_a_button_that_is_not_one_of_the_twelve` (the overlay reported it as "a button does two jobs"); the
  overlay's unused `a_hold_on_a_flywheel_or_a_pneumatic` and its generic `a_button_does_two_jobs` dropped (every
  `refused::` function has a harness TU — §3.0); **a toggle on a flywheel's CONSTANT speed is refused** (the brief: a
  toggle works "a flywheel's NAMED speed"; the overlay accepted `toggle(flywheel.at(3000))`), the refusal renamed
  `a_toggle_needs_a_two_position_pneumatic_or_a_flywheel_named_speed` (n22's expected text is its prefix; D8 RED);
  a time limit must be finite (`detail::realBound`); `enter()` / `tick()` preconditions (bound).
- **A4-D11** the driver toggle's at-speed window is the public `mechanism::kToggleAtSpeedWindowSeconds` (5 s, HA-182 —
  A3's hand-on: its entry and row now carry it); the overlay's was private.
- **A4-D12** `direction_check.hpp`: preconditions (a motor kind for `judgeDirection`, a pneumatic for `judgePneumatic`; a
  finite, non-zero command; the page's thresholds finite and above 0; the positions in range); a motor "against its
  command" and an angle sensor's reversal are judged only at or past the move threshold — encoder noise below it is not a
  fight (the overlay flagged any negative product; DC5 RED); every `snprintf` precision bounded so the 384-byte verdict
  provably fits (GCC's `-Wformat-truncation` refused the overlay's at -O2 — the brief's verbatim sentences are unchanged;
  a path is cut at 130–160 characters).
- **A4-D13** the FREEZE line: the brief calls the mechanism steps additive growth of F10 but does not say to freeze them.
  Pinned: `MechanismStepFailed == 5` (an append-only enum's values must be pinned) and `RoutineResult::mechanismVerdict`'s
  type (a field of a frozen return type); the F10 and F8 floors moved to 5. NOT pinned, and said so in `routine.hpp`'s
  banner, the pin file and the F10 row: the steps, the context constructor and `lastMechanism()` — the mechanism seam's,
  unfrozen like `then()` until the core runs on a robot (Q26's "MECH on hardware" trigger). The coordinator may rule
  otherwise; pinning them later is additive.
- **A4-D14** the two constrained templates use named concepts (`chassis::detail::RoutineContext`,
  `MechanismDeclaration`): the doc gate reads a `requires requires(...) {...}` clause between the `///` and the
  declaration as an undocumented member (measured: the first build failed DOC COVERAGE on both).
- **A4-D15** the words gain `TankSticks` beside `ArcadeBody` / `ArcadeField` (the third `DriveMode` a team may choose).
- **A4-D16** §3.4's "Every step form ... each stop the chain as `MechanismFailed`" is the 10-06 body's word; the
  re-check's `MechanismStepFailed` is built (the amendments override the body), and a test pins `!= MechanismFailed`
  (R12 RED).
- **A4-D17** the docs: chapter 18's title widened ("— in autons, on buttons, on routes"), and the cookbook recipe got a
  NEW chapter, `docs/cookbook/07-mechanisms.md` (+ one `mkdocs.yml` nav line, + the cookbook README's row and source
  note): chapter 6 is driver control on T3's bindings, chapters 1–5 are drive recipes, and the recipe needs both an auton
  and a driver file. Its listings and chapter 18's new ones are quoted from a new example test,
  `test/mechanism_auton_example_test.cpp` (`check-examples` reads `test/*example*_test.cpp`).
- **A4-D18** the binder's REFUSED line buffer is 256 bytes (the overlay's 160 could not hold its bounded format).
- **A4-D19** `MechanismRoutes::doc(i)` (an observer for a test or a tool) and `kMechanismRouteDocBytes`.
- **A4-D20** a blocking wait that ran out puts the verdict FIRST in the routine's stop-line bracket — `(TIMED_OUT: the
  mechanism service never ticked -- it is not in the chassis pacer's chain)` — so a stop line always carries its verdict;
  the brief's sentence is kept verbatim inside it.
- **A4-D21** `MechanismOutcome::Refused`'s doc comment widened (A2-D18's hand-on: a route marker whose args are wrong) —
  `manipulation/mechanism_outcome.hpp`, a header outside the A4 row's list, named by the hand-on; the changelog and
  guide 11 too.
- **A4-D22** the rig builds robot x's devices by HAND over the §1.12 models (the brief's probes did), not through
  `robot::MechanismParts` (A2's `mechanism_service_rig.hpp`) — the plants must hold the fake motors, and the brief's
  P1/P14 numbers are reproduced to the hundredth on exactly the probes' wiring.
- **A4-D23** stale prose fixed where found (pace rule 4): `runner/command_registry.hpp`'s banner ("asynchronous mechanism
  operations are chunk T3's" — the brief's "Found while reading"), `mechanism/route_ids.hpp`'s and
  `mechanism/service.hpp`'s future-tense MECH-A4 sentences (A1's and A2/A3's files — one sentence each), guide 9's
  "used for real, twice" and its stop-cause list, guide 14's mechanism bullet, the glossary's `route id` and
  `step (recipe)`, cookbook 06's T3 recipes (a "superseded" note, N4), the guide index's chapter 18 row, the README's
  source layout.
- **A4-D24** `DriverInputs::boundTo(svc)` and the loop's precondition that its inputs are bound to the SERVICE it is
  handed (the overlay checked `bound()` only: inputs bound to another service would command mechanisms the loop never
  ticks — BT RED). Found while writing the mutation list.
- **A4-D25** two tests beyond §3.4's list, each asked for by a mutation that compiled but needed its own case: a START
  form's ABSENT and REFUSED starts (R4c, R4d), a control of another KIND at bind (D13); and the direction check's
  "exactly the move angle while drawing" edge (DC4 survived run 1 — the `still` flag feeds only the jam rule, whose edge
  needs a drawing motor; DC4b re-run RED).

### 18:10–18:55 — EVERY mutation, on a fresh build of every dependent object *(`date` 18:55)*
- **The driver** is MECH-A3's (`scratch/mech-a4/mutate.py`, re-pointed): the suite target's `depend` step before every
  build, EVERY object that depends on the mutated file recompiled (fewer = INVALID, never a verdict), the link checked
  newer than the mutation, the WHOLE suite run under `ulimit -v 4194304; timeout 600`, the binary deleted after each run,
  the file restored byte-identical in a `finally` and its timestamp bumped; harness mutations judged by `ctest -R
  compile_refusal_<…>`. Backups `scratch/mech-a4/mut/` (per mutation) and `mut/orig*/` (every mutated file, with md5 —
  `md5sum -c` OK after every run). Lists: `mutlist.py` (53), `mutlist2.py` (13), `mutlist3.py` (9). None was killed by
  the cap or the timeout.
- **75 mutations: 68 RED by a TEST's verdict (suite or harness — the re-runs R3b, R4c, R4d, R6b, D6b, DC4b among
  them), 2 RED by a test's own compile-time assertion (D11b: the positive TU; D12: a test's `static_assert`), 4 RED AT
  BUILD by a warning only — NOT counted as caught; each re-written to compile and re-run RED (R3, R4, R6, D6) — and 1
  GREEN that asked for a test (DC4: the case added, re-run RED as DC4b).** The brief's: **M22** RED (the sensorless-grab, robot two's route and the counted
  autons), **M23** RED (P5d, N10's stall supersede), **M24** RED (P8's order pin, the same-tick press), **M25** RED (the
  sign), **M35 proper** RED (D1: 0 of N paces at 12 V with the call; the CONTROL overload reproduces the failure, >50),
  **M44** RED (robot two's route), **M45** RED (the bad-arg markers: the step's own precondition would throw), **M46**
  RED (the motor findings, verbatim).

  | id | file | the mutation | verdict — the case(s) that turned red |
  |---|---|---|---|
  | M22 | routine.hpp | stops on a counted UNCONFIRMED | RED: steps/sensorless grab, P14d, forms/counted |
  | R2 | routine.hpp | continues on ANY UNCONFIRMED | RED: steps/an UNCONFIRMED NOT counted |
  | R3 → R3b | routine.hpp | the never-ticked sentence never chosen | R3 RED AT BUILD (`ticks0` unused — not a verdict); **R3b** (compiles) RED: P1 CONTROL |
  | R4 → R4c, R4d | routine.hpp | a START's ABSENT / REFUSED start continues | R4 RED AT BUILD (`-Wparentheses`); **R4c, R4d** RED: steps/a START form whose START fails (A4-D25) |
  | R4b | routine.hpp | a START's NOT_HOMED start continues | RED: steps/NOT_HOMED |
  | R5 | routine.hpp | a blocking step does not wait | RED: 14 cases (P1 …) |
  | R6 → R6b | routine.hpp | a stop that did nothing → SUCCEEDED | R6 RED AT BUILD (unused parameter); **R6b** RED: steps/REFUSED |
  | R7 | routine.hpp | another robot's goal → "not on this robot" | RED: steps/a goal of ANOTHER robot |
  | R8 | routine.hpp | waitUntilDone collects a replaced step | RED: steps/waitUntilDone |
  | R9 | routine.hpp | a move not noted as started | RED: P1, steps/START forms |
  | R10 | routine.hpp | mechanismVerdict never written | RED: 11 cases |
  | R11 | routine.hpp | the wait's slack 0.25 → 0 | RED: 12 cases (P1 …) — the slack is load-bearing |
  | R12 | routine.hpp | a failing step recorded as MechanismFailed | RED: 7 cases |
  | M35 | match_runner.hpp | runAuton's service overload: no enterAutonomous() | RED: D1/M35 proper, the leftover claim, the refused entry |
  | MR2 | match_runner.hpp | enterAutonomous() moved into the run | RED: D1/M35 proper, the refused entry |
  | MR3 | match_runner.hpp | the guard handed an EMPTY span | RED: 6 cases (P1, the guard's reach …) |
  | MR4 | match_runner.hpp | the context without the service | RED: 6 cases |
  | M23 | driver_inputs.hpp | the SUPERSEDED state removed | RED: P5d, N10's stall supersede |
  | D2 | driver_inputs.hpp | a button down at entry is a press | RED: P5d |
  | D3 | driver_inputs.hpp | homeAtDriverStart re-homes what was homed | RED: P2i |
  | D4 | driver_inputs.hpp | a disconnect reads every button up | RED: a press must be SEEN (the reconnect) |
  | D5 | driver_inputs.hpp | both buttons: forward wins | RED: both buttons = neither |
  | D6 → D6b | driver_inputs.hpp | bind() accepts a stranger's preset value | D6 RED AT BUILD (unused parameter); **D6b** RED: bind checks against the SERVICE's robot |
  | D7 | driver_inputs.hpp | U5's rule removed | RED (harness): n48 |
  | D8 | driver_inputs.hpp | a toggle on a CONSTANT speed compiles | RED (harness): d_toggle_on_a_constant_speed |
  | D9 | driver_inputs.hpp | Left and Right swapped in the naming | RED (harness): d_button_Left_twice, m2 |
  | D10 | driver_inputs.hpp | a 0 s time limit accepted (EQUALITY) | RED (harness): n34, d_home_at_start_with_no_time_limit |
  | D11 | driver_inputs.hpp | a hold at 0 V accepted (EQUALITY) | RED (harness): d_hold_at_0_volts |
  | D11b | driver_inputs.hpp | a hold at exactly 12 V refused (EQUALITY) | RED at build — **the positive TU's own assertion**: `mechanism_step_forms_test.cpp:50` (`kLimits`' 12 V hold) refused, the failure IS that test's check (every form at its exact limit compiles) |
  | D12 | driver_inputs.hpp | avoids() blind to a hold's BACK | RED at build — **the test's own `static_assert`** (`mechanism_driver_loop_test.cpp`: `!rx::kControls.avoids(Button::R2)`) |
  | D13 | driver_inputs.hpp | bind() accepts another KIND | RED: bind refuses another KIND (A4-D25) |
  | BT | teleop_loop.hpp | inputs bound to ANOTHER service accepted | RED: the loop refuses inputs bound to another service (A4-D24) |
  | M24 | teleop_loop.hpp | the service ticked AFTER perTick | RED: P8 (`tickedBeforePanel`), the same-tick press |
  | L1 | teleop_loop.hpp | no leaveDriverControl() on a disable | RED: P5d |
  | L2 | teleop_loop.hpp | no enterDriverControl() at entry | RED: P9c (the claw's START) |
  | L3 | teleop_loop.hpp | a T3 binding beside the service | RED: the loop's preconditions |
  | L4 | teleop_loop.hpp | a control on the re-zero accepted | RED: the loop's preconditions |
  | M25 | direction_check.hpp | the command's sign ignored | RED: the SIGN |
  | M46 | direction_check.hpp | "nothing / something else" → OTHER way | RED: the motor findings, verbatim |
  | DC3 | direction_check.hpp | exactly the draw current not drawing (EQUALITY) | RED: the thresholds are EDGES |
  | DC4 → DC4b | direction_check.hpp | exactly the move angle is still (EQUALITY) | **DC4 GREEN — not caught**: the `still` flag feeds only the jam rule, whose edge needs a DRAWING motor; the case "exactly the move angle WHILE drawing" added; **DC4b** RED |
  | DC5 | direction_check.hpp | noise below the threshold is a fight | RED: encoder NOISE |
  | DC6 | direction_check.hpp | the angle sensor judged backwards | RED: the ANGLE SENSOR |
  | DC7 | direction_check.hpp | exactly the move angle "neither moved nor drew" (EQUALITY) | RED: the thresholds are EDGES |
  | M44 | mechanism_routes.hpp | a counted UNCONFIRMED → Unconfirmed | RED: P14d |
  | M45 | mechanism_routes.hpp | the time-limit check removed | RED: P14b/c (the 0 s step throws) |
  | RT3 | mechanism_routes.hpp | the volts unchecked | RED: P14b/c |
  | RT4 | mechanism_routes.hpp | a lift's home as an ARM's | RED: P14, every action |
  | RT5 | mechanism_routes.hpp | a lift preset's goal in DEGREES | RED: every action |
  | RT6 | mechanism_routes.hpp | a doc drifts | RED: the MANIFESTS |
  | RT10, RT11 | mechanism_routes.hpp | ±12 V refused (EQUALITY) | RED: P14 (+12), P14d (−12) |
  | RT12, RT13 | mechanism_routes.hpp | 0 ms accepted (EQUALITY) | RED: P14b/c |
  | RT7 | manifest.hpp | the doc ignored | RED: the MANIFESTS, the DOC |
  | RT8 | command_registry.hpp | a quoted doc accepted | RED: the DOC |
  | RT9 | command_registry.hpp | the four-argument on() drops the doc | RED: the MANIFESTS, the DOC |
  | S1 | competition_run.hpp | the new cause's word drifts | RED: `request (R23)` |
  | S2 | auton_tester_run.hpp | the new cause's word drifts | RED: `run (23)` (A4-D9's added check) |
  | V1 | version.hpp | "2.4" left with minor 5 | RED: the F6, F8, F10 pins |
  | H1–H4 | routine.hpp | each refusal template's sentence lost or its kind dropped | RED (harness): n05 + r_run_on_a_pneumatic, n53, n54, n55 |
  | H5–H9 | driver_inputs.hpp | whileHeld(hood) / n19 / autonOnly+control / 25 controls / a preset on never-homed — each refusal removed | RED (harness): n24, n19, d_auton_only_with_a_control, d_more_controls_than_fit, n50 |
- **The equality edges** (the coordinator's lesson from MB5 / CM13 — a test must land ON the edge): the hold's 0 V and
  12 V (D11, D11b), the 0 s time limit (D10), a marker's ±12 V and 0 ms (RT10–RT13), the direction check's two
  thresholds (DC3, DC4b, DC7) — each landed on exactly, each RED. No time-based edge is A4's own: the step bounds and
  the 5 s toggle window are the service's (A2/A3, tested there on exact clocks).

### 18:56–19:13 — every gate on the FINAL tree *(`date` 19:13:16)*
- **The host suite, a CLEAN build** (`cmake --build build/test --target clean`, then `scratch/mech-a4/build.sh final`: the
  API pages generated, the binary deleted, the briefing generated, the full doc-gated build — **239 objects, 47.96 s**
  wall at -j8 —, the suite capped, the briefing generated again): **2605/2605 passed, 2,799,223 assertions, 3 skipped**
  (the base: 2545/2545, 2,798,203). `python3 tools/suite_run.py green`: **green — 2605 cases, 2,799,223 assertions**.
  MECH-A4's own cases (`-tc='MECH-A4*,guide-18c*,guide-18d*,guide-18e*,cookbook-07a*'`): **61 cases, 548 assertions**.
- **The compile-refusal harness**, every TU: `ctest --test-dir build/test -R compile_refusal --no-tests=error -j1` —
  **246/246 passed** (116 s); TUs 246 = CTests 246 (CI's count rule).
- **`python3 tools/src_build_gate.py self-test`**: OK (235 detector cases); **`check`: PASS — all 7 builds link**, 0
  disallowed warnings (each 11/11 TUs, 8 vendor-ignored warnings). `git diff --stat 43ac361 -- src/ tools/ Makefile`:
  the two `case` lines only (`src/programs/auton_tester_run.hpp` +1, `src/programs/competition_run.hpp` +1).
- **The ARM header gate** (RESUMING's command over every `include/shulib` header, -Werror, arm-none-eabi-g++ 13.2.1):
  **226 headers, rc 0, no output** (223 at the base + the three new).
- **The CI greps**: the PROS grep and the `sim/` grep empty; `grep -rn PROBE_ include/` empty; no "PROTOTYPE" /
  "(proposed" in an A4 file.
- **`api_doc_tool`**: self-test OK; check-coverage — 4,959 public entities across 186 headers, all documented;
  check-fresh OK; check-examples — 603 quoted lines verbatim (9 source files), 6 whole-run listings each unbroken;
  check-removability OK. **`briefing_status.py`** self-test OK, check current. **`doc_staleness_audit.py`** self-test
  OK, audit clean (31 live docs). **C7** removability grep empty.
- **Landmine 15 (routine.hpp now includes `mechanism/service.hpp`) — measured:** the clean host build 232 objects /
  45.86 s → 239 / 47.96 s (+7 objects — the new test files — and +2.1 s); a TU of `routine.hpp` alone at -O2 0.42 →
  0.50 s; the ARM header gate 1.78 → 1.87 s. Not noticeable: the fallback (the steps over a forward-declared service) is
  not needed.
- **Sizes on ARM -Os** (a failing-template read of `sizeof`): `runner::MechanismRoutes` **4,804 B** and
  `CommandRegistry` **1,156 → 1,284 B** (the brief's S3, exactly); `mechanism::DriverInputs` 176 B,
  `mechanism::DriverControls` 1,544 B (a driver file's `inline constexpr`), `chassis::Routine` 48 → 112 B (on the stack
  of an auton).

### §2.1's DoD — MECH-A4, checked against the tree *(2026-10-22 19:13)*
- [x] `MECH-PROGRESS.md` carries this run's heading (written FIRST, 17:24), its tip (`43ac361`), each step as it happened,
      every mutation with its observed red, the prose review's greps and hits.
- [x] The host suite builds and passes (2605/2605, clean build) AND `ctest -R compile_refusal` passes (246/246).
- [x] The ARM header gate passes (226 headers); the CI PROS-free and sim-free greps are empty.
- [x] `python3 tools/src_build_gate.py check` — all seven builds link, zero disallowed warnings; `self-test` OK.
- [x] `grep -rn "PROBE_" include/` empty; no copied file says "PROTOTYPE" or "proposed".
- [x] Every §3.4 test exists (P1 through the REAL runAuton; every step form; COMP-a unchanged; P5d, P2i, P8, P9c; P7's
      verdicts verbatim and `changeHere`'s sources; P14 through the REAL PathRunner; the stop cause and the `== 5` pin;
      U5's n48–n50 and the restart off a switch; D6's three answers; the both-paths and re-zero preconditions; every T4
      Part 2a protection test unchanged and passing) and every §3.4 mutation was RUN, seen red and restored (M22–M25,
      M35 proper, M44–M46), with 67 more of this run's own.
- [x] Docs in the same change (above), the prose review logged (above), the C7 grep empty.
- [x] Nothing committed; no `pros`; no `/dev/ttyACM*`. **The tree:** uncommitted in `.claude/worktrees/mech-a4` (branch
      `mech-a4`): 50 files modified (`git diff --stat 43ac361`: 2,756 insertions, 430 deletions — the API pages and
      the briefing among them) and 61 new (the three headers, three API pages, the cookbook chapter, nine test files, 45
      compile-refusal TUs).

### For the coordinator (unnumbered)
- **No hardware belief outside HA-182..HA-186 was needed, and no HA number was registered.** HA-182 gained the driver
  toggle's 5 s window (A3's hand-on); HA-185 stays held for A5.
- **The FREEZE line (A4-D13) is a call the brief left open:** the mechanism steps, `Routine(ctx, name)` and
  `lastMechanism()` are added to F10's class but NOT pinned, stated in the header, the pin file and the F10 row; only
  `MechanismStepFailed == 5` and `mechanismVerdict`'s type are pinned. Pinning the steps later is additive.
- **A latent COMP-b line-length gap, NOT A4's to fix (src/ is A5's):** `src/programs/competition_run.hpp`'s routine-stop
  line (`kCompetitionLineBytes` 200) reaches **214** bytes in its worst case (a 24-character step name, the counts at
  `INT_MAX`, the longest fault string) with A4's new word "mechanism step FAILED" — and was already **209** with
  "mechanism FAILED" / "motion CANCELLED" before A4 (computed from the format, not run): the line is cut (its `cut`
  flag set), never overflowed. `test/competition_run_test.cpp`'s worst-case case leaves `cause` at None and the step
  name empty, so it never meets it. A realistic stop line is ~163 bytes. The Auton Tester's render was checked with the
  new word (A4-D9) and fits.
- **Measured on the host test plant while writing the N10 test (not a defect, the team's number):** a 6 V hold-to-run
  pushing an arm into an obstacle draws 1.25 A (the plant's invented current shape, HA-93/HA-183) — under the recipe's
  2.0 A signature, so it never trips and the motor sits at that current while the button is held. Chapter 18's recipe
  is for a 12 V jam; a team that drives an arm at 6 V into something learns its signature is above that push. Worth a
  sentence in DOCS3 or A6's bench page (a held button's current is visible there), not a library change.
- **A route `Hold` leg of 0.1 s exits TIMED_OUT** (seen while writing a test; 0.3 s settles) — PathRunner / `Chassis::hold`
  semantics, F8's, unchanged here; noted, not chased.
- **Files touched outside the A4 row's list, each logged:** `manipulation/mechanism_outcome.hpp` (A2-D18's hand-on),
  `mechanism/service.hpp` (two comment sentences), `mechanism/route_ids.hpp` (one banner sentence),
  `test/auton_tester_run_test.cpp` (A4-D9), `mkdocs.yml` (ONE nav line by hand — the cookbook chapter — plus the
  tool-generated API nav), `README.md`'s source layout, cookbook 06 / README, guides 9 / 11 / 14 / 15 / README,
  `docs/hardware-assumptions.md` (HA-182). **No file of an in-flight chunk was touched** (CAL1 Part A2a's, VIS1's —
  `hal/device_provider*`, `config/robot_config.hpp`, `robot/robot.hpp`, … — unchanged). `routine.hpp`'s additions sit in
  two delimited blocks (`MECH-A4: the mechanism steps — BEGIN/END` and `… machinery — BEGIN/END`) plus the appended
  members, for L6's later step.
- **The shared files** (`test/CMakeLists.txt` untouched — the suite and the harness glob; the changelog's API 2.5 section
  added at the top, above 2.4's; `mkdocs.yml` as above): a merge onto a main whose changelog grew keeps 2.5 on top.
- **Re-run after the entries above** (19:15): the `src` gate's self-test left `config/robot_config.hpp` and eight more
  headers with a new mtime (its planted cases restore the content: `git diff 43ac361` shows none of them), so the
  staleness audit no longer trusted the binary; `build.sh final2` rebuilt all 239 objects (45.6 s) — **2605/2605,
  2,799,223 assertions, 3 skipped** —, then `briefing_status.py check` current, `doc_staleness_audit.py` clean, C7 empty.

### MECH-A4 review findings and their disposition — started *(`date`: Tue Oct 27 07:51:26 PM EDT 2026)*
- Three reviewers (correctness, tests, docs) returned 30 findings (one blocker). The worktree's A4 files were
  snapshotted before any edit (`scratch/mech-a4/fixround/tree_before_fixround.tgz`, 111 files). Each finding is
  re-checked, fixed at the root or answered with evidence below; every gate re-run on the final tree.

### MECH-A4 review findings and their disposition *(the fix round: 2026-10-27, `date` 19:51:26 at the snapshot; this entry written as each step finished)*
- **How it ran.** Every finding re-checked against the worktree before any edit (the code read, the reviewers' probes
  read; `scratch/mech-a4/fixround/tree_before_fixround.tgz` = the 111 A4 files before the round). Root fixes in the
  library; a test for every finding, each placed in the file of its topic under a "MECH-A4's review round" banner;
  then EVERY fix and every survivor the reviewers named mutated on a fresh build of every dependent object
  (`fixround/mutate_fr.py` = `mutate.py` re-pointed to `fixround/mut/`, `fixround/fr_list.py`), each seen red and
  restored byte-identical; then the per-PLACE deletion check (test/README's claim) over A4's refusal calls; then
  every gate on the final tree.

**Correctness (6).**
- **C1 (BLOCKER) — a controller DROP fired every one-button control whose button was up. REAL — confirmed by reading:**
  `pressedNow()` returned `edge_[i].update(connected ? … : true)`, and `hal::ButtonEdge::update` reports a rising edge
  whenever the last level was up (`hal/controller.hpp` :111-114). So robot two's claw flipped OPEN on a drop, robot x's
  three arm presets and its flywheel toggle fired, and kAltControls' `homeOnPress` began a homing push. A regression
  against the overlay (`proposed4/…/driver_inputs.hpp` :353/:366/:374: `if (!connected) { (void)edge_[i].update(true);
  break; }`) introduced by A4-D1's rewrite and logged nowhere. **FIXED at the root** — `pressedNow()` records the
  unknown level as HELD and returns false on a disconnected tick (the overlay's form; T3's `forgetLevel()` is the same
  rule). **Test:** "a controller DROP with every button UP fires NOTHING" through the REAL `runDriverControl` — robot
  two's claw line and command number unchanged through a drop and its reconnect, then a real A press still opens it;
  robot x's arm/flywheel command numbers unchanged and Right still presses after the reconnect; kAltControls' lift
  begins no step and the hood's three presets do not fire. **FR-C1** (the collapsed expression restored) RED;
  **FR-D4** (the drop recorded as "up") RED on the existing reconnect case.
- **C2 (major) — U5's driver-start home CANCELLED by the mechanism's own hold or preset. REAL** — `jog()` / `moveTo()`
  replace a running step ("replaced by a newer command"), unhomed with no `.unhomedVolts` the jog then commands
  nothing, and nothing re-issues the home: the dead buttons U5 refuses at build. **FIXED — the reviewer's rule (a),
  A4-D26:** while the driver-start home that `enter()` began is still running (its slot's command number unchanged
  since and no verdict yet, `startHomeRuns()`), that mechanism's OWN controls wait — a hold held during it is
  SUPERSEDED (nothing until its buttons are seen released after the home), a preset's or a homeOnPress's press is
  consumed and dropped; other mechanisms are untouched; any other running step (a preset's move after the home) is
  still replaced at once (Q8). **Rule (b) not taken** (re-issue the home on the next touch while unhomed): it changes
  "once at entry" and, on a broken switch, re-pushes into the stop on every press; (a) costs at most the home's own
  time limit. Stated in the header as THE ONE EXCEPTION to "the most recent input wins" (Q8). **For the coordinator:**
  it is a rule beside Q8 the brief did not state — overrulable; the tests say what it does. **Tests:** the reviewer's
  four touches (nothing; L1 held THROUGH entry; L2 tapped at 0.20 s; L1 at 0.50 s) — homed by 3.0 s, no "replaced by a
  newer command" on the lift, L1 later raises it; a hold held from before entry to 3.0 s does nothing after the home
  until released and pressed again; Up (preset) and Left (homeOnPress) during the home dropped, Up after it goes to
  'high', and L2 during THAT move wins at once (the move replaced — the exception is only the driver-start home's).
  **FR-U5a … FR-U5e** RED.
- **C3 (minor) — "a button already down at entry does nothing" is wrong for a hold. REAL (words only).** The code is
  T3's rule (a hold's button is a level: down at entry it drives at once — `mechanism_binding.hpp` :59-60), kept.
  **FIXED** per control in `driver_inputs.hpp`'s header and `enter()`'s doc, chapter 18 and the changelog's 2.5
  entry: a toggle's, a preset's and a homeOnPress's press must be SEEN; a hold's button is a level. The EN1 case below
  pins the hold's half at a re-entry.
- **C4 (minor) — U5 accepts `.unhomedVolts` as "a way to home", but a jog never homes. REAL** — the service zeroes at
  bind (`service.hpp` :765/:769) and in a homing step (:1102) only; `tickJog` unhomed commands the clamped volts
  (:1043-1045). A preset beside it ends NOT_HOMED all match after a restart off the home. **FIXED — A4-D27, the
  reviewer's first option:** `.unhomedVolts` answers for a mechanism that is only HELD; a PRESET on a switch- or
  stall-homed mechanism needs `homeAtDriverStart` or `homeOnPress` (same refusal name; its doc comment says why). The
  second option (a jog onto a pressed switch zeroes it) not taken: it changes A3's service and trusts a switch read
  mid-push for a zero. **For the coordinator:** this TIGHTENS the brief's U5 text (which lists `.unhomedVolts`
  unconditionally) — overrulable; no fixture, brief listing or robot two's file is affected (robot two's pivots are
  Power; robot x's and robot y's homed mechanisms carry homing controls). **Tests:** `n48d_preset_beside_unhomed_volts`
  (refused) and the positive TU's new `forms::u5` lists (a switch-homed lift and a stall-homed arm with `.unhomedVolts`,
  HELD only, compile). **FR-C4** RED (harness: n48d), **FR-D14** RED at build — the failure IS the positive TU's own
  check (`mechanism_step_forms_test.cpp`'s `kHeldOnlyControls` refused, its `static_assert` non-constant).
- **C5 (minor) — a GENERATED robot file's sensor / mechanism-wide / pneumatic sentences in C++ words with no
  regenerate step. REAL. FIXED — A4-D29:** every finding that names a role's place has a generated branch when that
  role's `source` is set — SensorReversed ("at <path> make it true|false, then regenerate, rebuild and upload"),
  MechanismReversed for motors ("in the generated robot file, flip EVERY motor role's reversed (and the angle
  sensor's) -- motor 1's is at <path> -- then regenerate…"), the pneumatic's levels ("at <path> swap each position's
  levels, then regenerate…"), and FightOrJam's two-motor sentence too (the reviewer did not list it; the same defect).
  **Test:** one generated sentence pinned per finding, verbatim. **FR-G1 … FR-G5** RED.
- **C6 (major) — "make motor 1 Reversed" for a BACK-DRIVEN mechanism. REAL** — a V5 motor reverses command and encoder
  together (landmine 17), so a motor reading against its command is a fight only when a mate moves WITH its own.
  **FIXED — A4-D28:** a new finding `DirectionFinding::BackDriven`, APPENDED after the brief's verbatim eight (their
  values unchanged; nothing is frozen here): when every motor that moved went against its command (one motor, or all
  alike), "'<name>' was turned AGAINST its command by something stronger than the jog (gravity or a load): every motor
  that moved went the same wrong way, so no declaration is wrong -- jog it harder, or from another pose, and check
  again" — no line named. "Moved" uses the page's threshold (a mate's noise is not "with"). **Tests:** the one-motor
  arm and the two-motor lift verbatim, a DOWN jog driven up, a mate EXACTLY at the move angle with its command (a fight
  — the equality edge landed on), a mate below it (back-driven). **FR-BK1 … FR-BK3** RED. **For the coordinator:**
  MECH-A6's page shows a ninth finding.

**Tests (12).**
- **T1 — a wait shorter than its step (the second TIMED_OUT sentence). REAL gap. Test:** the reviewer's probe A
  (`startRun(…3_s).waitUntilDone(intake, 0.5_s).move(…)` with no piece: stopped at 2, TIMED_OUT in `mechanismVerdict`
  AND `lastMechanism()`, the sentence verbatim, "never ticked" absent, one skipped, the step still running).
  **FR-WU3, FR-V1, FR-V8b** RED.
- **T2 — skipping after a stop, per step form. REAL gap. Test:** probe B (all twelve forms after a NOT_HOMED stop:
  skipped 12, completed 0, no pace, every slot's command number unchanged, the stop's step and verdict not overwritten).
  **FR-SK1 … FR-SK7** RED.
- **T3 — driver-control branches never run. REAL gaps. Tests:** the toggle OFF before any disable (probe C, plus the
  named speed); kAltControls' hood presets → each position's line levels; a flywheel preset's named and constant
  speeds; a preset's time limit (TIMED_OUT at 0.25 s, elapsed in [0.25, 0.27)); the driver-start home's (a switch that
  never reads pressed: TIMED_OUT at 1.5 s); the toggle's window (a flywheel whose named speed its plant cannot reach:
  TIMED_OUT at `kToggleAtSpeedWindowSeconds`, the wheel still spinning, the constant pinned `== 5.0`).
  **FR-TG2, FR-D1, FR-D2, FR-DB1, FR-DB2** RED; **FR-TW1** (5 → 4 s) RED at build — the failure IS the test's own
  `static_assert(kToggleAtSpeedWindowSeconds == 5.0)`; **FR-TW2** (the toggle's USE of it replaced by a literal 4 s)
  RED at run time, the window check.
- **T4 — bind()'s rules untested. REAL gap. Test:** one stranger per rule (a Lift named "arm"; a two-position "hood"
  toggled; a flywheel toggle at a "shoot" of 2500; a constant 170° on a 0..180 stranger; a constant 4000 rpm under a
  4200 rpm stranger; a pneumatic position "open" at index 1; a homing control bound to a never-homed "arm"), each
  `CHECK_THROWS` and `!bound()`. **FR-BD3 … FR-BD9** RED.
- **T5 — build-time rules that compiled when broken. REAL gaps. TUs:** `n48b_stall_homed_arm_held_no_way`,
  `n48c_switch_homed_lift_presets_only`, `d_home_on_press_on_a_used_button` (Left named), `d_preset_with_an_infinite_bound`;
  the positive `forms::u5` lists (C4). **FR-D12, FR-D13, FR-D15, FR-CT1** RED (harness), **FR-D14** RED at build (the
  positive TU).
- **T6 — an ABSENT mechanism's control binds; an UNBOUND service still runs the auton. REAL gaps. Tests:** robot x
  with its flywheel ABSENT binds kControls, R1 drives the arm, X's toggle says ABSENT; `runAuton`'s service overload
  with a never-bound service: RoutineRan, the drive step completed, the arm step ABSENT with the not-bound sentence.
  **FR-AB1, FR-AB3** RED.
- **T7 — `_until_gone` never fired; the ms conversion unchecked. REAL gaps. Test:** `intake_until_gone [-6, 2000]`
  with a piece leaving at 0.6 s: SUCCEEDED "stopped seeing a piece", elapsed in (0.5, 1.0); `intake_run [12, 500]`:
  "time in 0.50s", elapsed in [0.5, 0.52). **FR-T1, FR-T2** RED.
- **T8 — the 0 s waitUntilDone edge masked. REAL. Test:** the 0 s bound after a START on the same mechanism (the
  message checked), the step still collectable after. **FR-V5** RED.
- **T9 — spinUp's `computed`, set()'s noteStarted. REAL gaps. Tests:** `spinUp(flywheel.clampedTo(5000.0))` clamps
  to 3600 rpm and says so (the host test plant's 5 %-slow flywheel then times out — the step RAN, never threw);
  `set(hood["up"]).waitUntilDone(hood, 1_s)` succeeds. **FR-V6, FR-NS1** RED.
- **T10 — the Warn level unpinned. REAL gap.** The rig gains `RobotX::countAt(level, subsystem, needle)`. **Tests:**
  "not on this robot", a REFUSED stop and an ABSENT stop, and the binder's REFUSED marker line, each at Warn.
  **FR-LV1, FR-LV2, FR-LV3** RED.
- **T11 — four direction branches. REAL gaps. Tests:** a generated FORWARD motor ("make it true"), a FORWARD
  hand-written sensor (robot z's wrist → `rotationSensor(Reversed)`), a still motor drawing EXACTLY the draw current
  beside a moving mate (AwaitingPerson; half of it → DidNotMove), a pneumatic told its FIRST position going to the
  other. **FR-C3, FR-C4s, FR-C5, FR-C7** RED.
- **T12 — a superseded hold through a drop; enter()'s hold reset. REAL gaps. Tests:** N10's stall with L1 held, a
  drop and a reconnect with L1 still down — 0 V and one STALLED line; R1 held across a disable — the second entry drives
  at once. **FR-D10, FR-EN1** RED.

**Docs (14).**
- **D1 (major) — A4-D13's freeze scope written as decided, citing a trigger the register does not carry. REAL**
  (`grep -c 'MECH on hardware' docs/roadmap.md` at base: 0; the register's F11 trigger: "F3's scoring primitives, on
  hardware"). **FIXED — worded as OPEN in all six places** (`routine.hpp`'s banner, the F10 row, `version.hpp`'s 5,
  the changelog's 2.5 heading block, guide 9's stability box, the pin file's comment): not pinned meanwhile; the
  proposal (not yet — they take the unfrozen mechanism seam's types, as `then()` does) and the alternative (L9's
  additive path, pinned the day they landed) both stated; pinning later is additive. Not pinned the L9 way: that would
  be the decision itself. **For the coordinator: the team lead's ruling is needed before the merge** (pace rule 2).
- **D2 (major) — the jam section implied a held button always trips. REAL** (the 0.95 s is a 12 V push,
  `mechanism_position_test.cpp` :1172-1174; at 6 V the plant's stall current is 6 × 2.5/12 = 1.25 A < 2.0 A).
  **FIXED** in this change: chapter 18's jam section ("A gentler push may never trip it"), its driver section (a
  pointer), and cookbook 07 (a watch-out under the 6 V recipe).
- **D3 — "a 2.4 program needs nothing". REAL. FIXED:** the changelog's 2.5 "What you must do" and heading block,
  `version.hpp`'s 5 and guide 9 now say a `switch` over every `RoutineStopCause` needs a `MechanismStepFailed` case
  (A2's precedent for an appended enumerator; shulib's two programs each gained one).
- **D4 — "all eight fields". REAL. FIXED:** nine (`mechanismVerdict` appended at MECH-A4, its type pinned) in
  `routine.hpp`'s banner and the F10 row.
- **D5 — future tense left standing. REAL. FIXED:** chapter 18 :113 ("`whileHeld` — below"), cookbook 06's per-line
  sentence (A2's hand-on — the core's driver controls now drive such positions; no program until its fifth part),
  guide 9 ("every step spelling and every observer but the ones listed as not frozen"; "One exception" became the
  not-frozen list).
- **D6 — HA-157 not cited (landmine 11). REAL. FIXED** in chapter 18 and the changelog's 2.5 entry. **Also found while
  fixing (the same class, unflagged):** the press-must-be-SEEN rule rests on HA-147 (the connected flag and the
  buttons come back together — T3's ToggleBinding cites it); chapter 18, `driver_inputs.hpp`'s header and HA-147's
  entry (Source + Contained) now say so.
- **D7 — the VexBuilder paragraph lacked "host-only". REAL. FIXED:** a status note in §4.2.3 (host-tested only, no
  program registers the ids until the core's fifth part, the six lines from a fixture's copy of robot two's claws, the
  manifest file's place the panel's).
- **D8 — guide 11 lacked the third ABSENT sentence. REAL. FIXED** in guide 11 (what it means: the boot never bound the
  service; drive steps before it ran; the chain stopped at the first mechanism step) and the changelog.
- **D9 — no G3a hand-on for the changed refusal vocabulary. REAL. FIXED:** the unnumbered coordinator note below.
- **D10 — the log omitted steps. REAL. FIXED here:** (a) landmine 24's re-read — `git log 43ac361 --
  docs/internal/MECH-PANEL-requirements.md`: last `bdcbc7c`; `git diff 2fce38a 43ac361` touches only :101-102 and
  :277-280 (the controller screen's wording) — §4 item 7b (:497) is unchanged since the brief's tip; (b) the HA
  register reconciled both ways over A4's fourteen headers — direction 1 (every `HA-NNN` they cite is a registered row:
  HA-05, -32, -50, -51, -53, -103, -104, -112, -145, -146, -147, -154, -157, -161, -167, -175, -176, -182, -184, -186 —
  all present), direction 2 (HA-182's Source names `driver_inputs.hpp`, which carries HA-182; HA-147's Source now
  names it too, and it carries HA-147); (c) the cited lines (A4-D30): the AutonContext comment the brief cites at
  :221 is at :224 at the base; the brief's `competition_run.hpp` :559, `auton_tester_run.hpp` :271 and
  `competition_run_test.cpp` :1370 are the existing `MechanismFailed` lines, and A4's lines sit BESIDE them at :560,
  :272 and :1371 — placements, not drift; (d) the deltas' order: A4-D23 moved before D24.
- **D11 — HA-182's row put the window in "Settled by". REAL. FIXED:** the belief column lists
  `kToggleAtSpeedWindowSeconds`; "Settled by" says how (timing a real flywheel's spin-up).
- **D12 — test/README's harness claim stale. REAL. FIXED by running the check** (below) and updating the counts.
- **D13 — public build dates. REAL, the merge's:** the coordinator re-dates at the commit (A3's form); the list is in
  the coordinator's notes below.

**For the coordinator (unnumbered), from the review round.**
- **THREE rulings wanted before the merge** (each built as recommended, each overrulable, each with its tests): (1) the
  FREEZE line (D1 — now worded OPEN in all six places; the team lead's call, pace rule 2); (2) the driver-start home's
  exception to "the most recent input wins" (C2, A4-D26 — rule (a)); (3) U5 tightened: `.unhomedVolts` answers for a
  HELD-only mechanism, a preset needs a homing control (C4, A4-D27 — tightens the brief's U5 text).
- **G3a's validator mirror (the brief's §6.3, its §1.14 re-check):** mirror A4's refusal NAMES and SENTENCES, not the
  brief's 17. Against the overlay: `a_button_does_two_jobs` DROPPED (every button is named:
  `button_<L1…A>_does_two_jobs`), `a_hold_on_a_flywheel_or_a_pneumatic` DROPPED (unused — `whileHeld` on either is
  refused by name), `…_flywheel_with_a_preset` RENAMED `a_toggle_needs_a_two_position_pneumatic_or_a_flywheel_named_speed`
  (and a toggle on a flywheel's CONSTANT speed is refused — A4-D10), ADDED
  `homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift` and
  `a_control_names_a_button_that_is_not_one_of_the_twelve`; 29 functions, 18 reasons (the twelve buttons one). And
  U5's rule as A4-D27 states it (a preset needs a homing control whatever its `.unhomedVolts`). Plus bind()'s run-time
  checks against the service's robot (A4-D10) — a panel cannot see those at generation, the boot does.
- **MECH-A6's page:** the judgement has a NINTH finding, `BackDriven` (A4-D28: no line named — "jog it harder, or
  from another pose"), and every role-naming finding has a generated-file sentence (A4-D29).
- **MECH-A5:** robot two's driver file's `toggle(claw, Button::A)` inherits the C1 fix (a drop never flips the claw);
  HA-147's Station J.9.5 now settles the service path too.
- **The landing dates (D13), for the commit — A3's form "(<landing>; built 2026-10-22)":** `docs/changelog.md` the 2.5
  heading ("Added (2026-10-22)"); `docs/guide/18-mechanisms.md` :12 ("A3 and A4, 2026-10-22"); `docs/guide/10-the-api.md`
  :409-411 ("MECH-A4, 2026-10-22"); `docs/guide/14-what-it-cannot-do-yet.md` :217 ("2026-10-15 to 2026-10-22") and
  :422; `docs/cookbook/06-driver-control.md` :21; `docs/vexbuilder-integration.md` :352, :362 and the §4.3 table row
  ("since 2026-10-22"); `include/shulib/version.hpp` :103; `docs/roadmap.md` the F8 and F10 rows ("MECH-A4, 2026-10-22"),
  :385 and :1765 ("BUILT 2026-10-22 … to come" → LANDED); `build-order.md`'s first bold `Next:` and MECH entry,
  `REMAINING-WORK.md` row 12, `HANDOFF-remaining-work.md`. Left as the BUILD date on purpose (A3's rule): the register's
  "since MECH-A4, 2026-10-22" source notes (HA-182 :1283) and the code banners naming the run by its date
  (`driver_inputs.hpp` :5, `direction_check.hpp` :4, `mechanism_routes.hpp` :5, `teleop_loop.hpp` :114,
  `mechanism_binding.hpp` :7).

**Deltas from the brief added by the review round** (numbered on from A4-D25): **A4-D26** the driver-start home's
exception (C2); **A4-D27** U5: `.unhomedVolts` answers for a HELD-only mechanism, a preset needs a homing control
(C4 — tightens the brief's U5 text); **A4-D28** `DirectionFinding::BackDriven`, appended after the brief's eight
(C6); **A4-D29** a generated-file sentence for every role-naming finding, FightOrJam's included (C5); **A4-D30** the
cited lines (D10 (c)); **A4-D31** A4-D13's freeze line re-worded OPEN (D1) — A4-D13 stands as the proposal, no
longer as a decision.

**The mutations — 62, every one RED, every file restored byte-identical** (`fixround/run_fr.out`, `run_extra.out`;
backups and per-mutation logs `fixround/mut/`; `md5sum -c` OK after each list; none killed by the cap or the
timeout). Each verdict came from a full build of every dependent object (16 for `driver_inputs.hpp`, 33 for
`routine.hpp`, 14 for `match_runner.hpp`, 2 for `mechanism_routes.hpp`, 1 for `direction_check.hpp`) and the WHOLE
suite under the 4 GiB / 600 s cap, or the harness for a build-time rule:

| ids | what | verdict — the case(s) that turned red |
|---|---|---|
| FR-C1, FR-D4 | the blocker restored; a drop recorded "up" | the drop case; the existing reconnect case |
| FR-U5a–e | the driver-start home's wait removed (hold / preset / homeOnPress), widened to any step, never recorded | the two U5 cases |
| FR-C4, FR-D12, FR-D13 | U5: the old rule; presets only; stall-homed | harness: n48d, n48c, n48b |
| FR-D14 | U5's accepting side refuses a held-only list | RED at build — the positive TU's own check (`forms::u5`) |
| FR-D15, FR-CT1 | an infinite limit; homeOnPress's button not reserved | harness: d_preset_with_an_infinite_bound, d_home_on_press_on_a_used_button |
| FR-TG2, FR-D1, FR-D2, FR-DB1, FR-DB2, FR-TW2 | toggle never off; pneumatic preset → 0; flywheel preset ×0.5; preset / start-home limits ×10; the toggle's window used as 4 s | the toggle-off, presets and time-limit cases |
| FR-TW1 | the window constant 5 → 4 s | RED at build — the test's own `static_assert` |
| FR-BD3–BD9 | bind()'s seven rules, each removed | the stranger-per-rule case |
| FR-AB1, FR-AB3 | bind() refuses an ABSENT one; runAuton refuses an unbound service | the absent-binds case; the unbound-runAuton case |
| FR-D10, FR-EN1 | a superseded hold released by a drop; enter() keeps a hold's state | the drop / re-entry case |
| FR-WU3, FR-V1, FR-V8b | a short wait recorded SUCCEEDED; its verdict not in lastMechanism(); its sentence never chosen | probe A's case |
| FR-SK1–SK7 | each step form's skip removed | probe B's case (SK1, SK3, SK7 also P1 CONTROL) |
| FR-V5, FR-V6, FR-NS1 | the 0 s edge; spinUp's computed; set()'s noteStarted | the 0 s case; the computed / set case |
| FR-LV1, FR-LV2, FR-LV3, FR-RS3 | the three Warn lines lowered; the not-bound sentence never chosen | the Warn cases; the unbound cases |
| FR-T1, FR-T2 | until_gone → sees; every marker time doubled | the until_gone / 0.50 s case |
| FR-BK1–BK3 | back-driven removed; its equality edge; noise as "with" | the back-driven case |
| FR-G1–G5, FR-C3 | each generated branch never taken; a generated value backwards | the generated case |
| FR-C4s, FR-C5, FR-C7 | a Forward sensor's word; DidNotMove's equality edge; a pneumatic told 0 | the each-way case |

**The equality edges** (the coordinator's lesson): the new guard's "a mate at exactly the move angle" (FR-BK2), the
draw current in DidNotMove (FR-C5), the 0 s wait on a STARTED step (FR-V5) — each landed on exactly, each RED. The
time limits above are scaling mutants, checked by intervals ([0.25, 0.27), [1.5, 1.52), [5.0, 5.02)); their `>=`
edges are the service's, pinned on exact clocks in A2/A3.

**The per-PLACE deletion check (D12) — 36 of 36 RED** (`fixround/fr_places.py`, `run_places.out`): each of the 31
direct `refused::` calls in `driver_inputs.hpp`, its dispatcher call (`refuseButtonTwice`, 15 TUs red) and each of
the Routine's four refusal templates' `static_assert`, deleted in turn; the harness over A4's 50 TUs went red every
time. (The ids' line numbers are the file's before the HA-147 paragraph — add 3.) `test/README.md` now says so,
with the harness's new time.

**Every gate on the final tree** (`date` at the end below):
- **The host suite, a CLEAN build** (`cmake --build build/test --target clean`, then `build.sh frclean`: the API pages
  generated, the binary deleted, the briefing generated, the full doc-gated build — 239 objects, 35.3 s at -j8 —, the
  suite capped, the briefing generated again): **2625/2625 passed, 2,799,429 assertions, 3 skipped** (before the
  round: 2605, 2,799,223). `python3 tools/suite_run.py green`: green — 2625 cases, 2,799,429 assertions. MECH-A4's own
  cases (`-tc='MECH-A4*,guide-18c*,guide-18d*,guide-18e*,cookbook-07a*'`): **81 cases, 720 assertions** (51 → 61 at
  the build, + 20 from the round).
- **The compile-refusal harness**, every TU: **251/251 passed** (197 s); TUs 251 = CTests 251.
- **`src_build_gate.py self-test`**: OK (235 detector cases); **`check`: PASS — all 7 builds link**, 0 disallowed
  warnings; `git diff --stat 43ac361 -- src/ tools/ Makefile`: the two `case` lines only.
- **The ARM header gate**: 226 headers, rc 0, no output. **The CI greps**: PROS and `sim/` empty; `PROBE_` 0; no
  "PROTOTYPE" / "(proposed" in an A4 file.
- **`api_doc_tool`**: self-test OK; check-coverage 4,960 public entities across 186 headers, all documented;
  check-fresh OK (the pages regenerated: `direction_check.md`, `driver_inputs.md` changed); check-examples 603 quoted
  lines verbatim; check-removability OK. **`briefing_status.py`** self-test OK, check current.
  **`doc_staleness_audit.py`** self-test OK, audit clean. **C7** empty.

**The prose review (pace rule 4) — every grep RUN and its hits READ before this line.** For the changed concepts:
`homeAtDriverStart` (10 hits: guide 10 :577 — a word list, true; chapter 18 :403, :417, :435, :450, :461 — updated;
cookbook 07 :65 — updated; changelog :82, :87, :97 — updated), `unhomedVolts` beside a homing word (the changelog
:87 — updated), "press must be seen" / "already down" (chapter 18 — updated; cookbook 06 :184 and guide 10 :296 — T3's
ToggleBinding, still true, HA-147 already beside them; the changelog :1477 — 2.3's history), the direction check
(guide 10 :543, :584, guide 14 :224-226, README :287, roadmap :390/:1768 — none lists the findings: true), "fight" /
"drags it" (no public hit beyond chapter 18 and the changelog — both updated), the freeze words ("not frozen",
"eight fields", "MECH on hardware": the six D1 places updated; HA-182/HA-183's "MECH on hardware" is a settling
phase's name, not a freeze trigger — left), `MechanismStepFailed` "What you must do" (changelog, `version.hpp`, guide
9 — updated). Internal pointers moved to "the review's fix round done 2026-10-27; verification, three rulings and
merge to come": `build-order.md`'s first bold `Next:` and its MECH entry, `roadmap.md`'s "Now" and the MECH row,
`REMAINING-WORK.md` row 12, `HANDOFF-remaining-work.md` :289.

**Files the round touched** — library: `mechanism/driver_inputs.hpp`, `mechanism/direction_check.hpp`,
`chassis/routine.hpp` (its banner only), `version.hpp` (comments); tests: `mechanism_driver_loop_test.cpp`,
`mechanism_auton_steps_test.cpp`, `mechanism_auton_run_test.cpp`, `mechanism_routes_test.cpp`,
`mechanism_direction_check_test.cpp`, `mechanism_step_forms_test.cpp`, `mechanism_auton_rig.hpp` (`countAt`),
`routine_signature_pin_test.cpp` (a comment), five new TUs, `test/README.md`; docs: the changelog, guides 9 / 11 /
18, cookbooks 06 / 07, `vexbuilder-integration.md`, `hardware-assumptions.md` (HA-182's row, HA-147's entry),
`roadmap.md` (F10 row, "Now", the MECH row), `docs/api/` (regenerated), the internal pointers above. No file of an
in-flight chunk touched; nothing committed; no `pros`; no `/dev/ttyACM*`.
- **Entry written** (`date`: Tue Oct 27 09:32:56 PM EDT 2026); the final re-check after the src gate (whose self-test re-times nine headers) follows.
- **The final re-check** (`build.sh frpost` after the src gate: the API pages regenerated, the binary rebuilt — **2625/2625, 2,799,429 assertions, 3 skipped**; `gates.sh frpostg`: every gate above green again, `briefing_status.py check` current, the staleness audit clean, the ARM gate rc 0). The tree: 51 files modified from `43ac361` (3,229 insertions, 446 deletions — the API pages and the briefing among them) and 66 new (the round added five TUs; `test/README.md` is the one newly modified file). `date`: Tue Oct 27 09:36:22 PM EDT 2026.

### MECH-A4 — Verification-round findings and their disposition — started *(`date`: Tue Oct 27 10:00:17 PM EDT 2026)*
- A fresh verifier returned **three findings** (VR-1 major: the C1 fix's "recorded as held" half is unpinned — a
  button pressed DURING a drop and still down at the reconnect; VR-2 minor: `driver_inputs.hpp` cites A4-D26 for the
  U5 tightening, which is A4-D27; VR-3 minor: guide 14 still promises the whole recipe layer frozen and names only
  `then()` as open). Each is re-checked against the tree before any edit; the disposition follows when the round's
  gates are green. Scratch: `.claude/coordinator/scratch/mech-a4/vround/`.

### MECH-A4 — Verification-round findings and their disposition *(this entry written as each step finished; `date` at its first line: Tue Oct 27 10:15:12 PM EDT 2026)*
- **Re-checked FIRST, each against the tree (every one REAL):** VR-1 — `pressedNow()` read (`driver_inputs.hpp`
  :719-725: both halves present, the code RIGHT), and every reconnect case read: "a press must be SEEN" (:247-272, X
  down from tick 10 — BEFORE the drop, so the edge already reads "held" and the recording line changes nothing) and
  the fix round's C1 case (:598-693, every button UP through the drop); none presses a button DURING the drop. The
  verifier's probe (`verify/probes/press_during_drop.cpp`) read: its two cases are that gap. A TEST gap on the half
  of the rule the header, chapter 18 :426-428 and the changelog :94-95 state ("still down when a dropped controller
  comes back, is not a press"). VR-2 — `grep -rn "A4-D2[6-9]" include test docs | grep -v MECH-PROGRESS`: the one
  hit, `driver_inputs.hpp` :496, cites A4-D26 (the driver-start home's exception) for U5's tightening, A4-D27.
  VR-3 — guide 14 :201-206 ("Both API tiers are frozen … routines written today will not need rewriting") and
  :254-261 ("One related piece stays deliberately open: `then()`") read; A4's diff to that file touched only :217
  and :421-427. **A sibling, found by this round's prose review:** guide 10's banner (:14-25) promises "every
  signature and documented behavior in this chapter changes only with a major API-version bump", and A4 added to
  that chapter the section "In an auton, on the buttons, on a route" naming the steps, the second constructor and
  `lastMechanism()`; the driver loop carves itself out of the banner (:210 then, :212 now, "The banner at the top of this chapter
  does not cover the loop"), the mechanism section (MECH-A1's, extended by A2–A4) never did.

| # | Finding | Disposition | Proof (each mutation on a fresh build of all 16 objects that depend on `driver_inputs.hpp`, the WHOLE suite under the 4 GiB / 600 s cap, restored byte-identical — `vround/mutate_vr.py` = `fixround/mutate_fr.py` re-pointed to `vround/mut/`, `vround/vr_list.py`, `vround/run_vr.out`) |
|---|---|---|---|
| VR-1 (major) | the C1 fix's "recorded as held" half unpinned: a button pressed DURING a drop, still down at the reconnect | **FIXED (tests); the code was right.** A NEW case in `test/mechanism_driver_loop_test.cpp`, "MECH-A4 (verification): a button pressed DURING a controller drop and still down at the reconnect is NOT a press -- released and pressed again, it is", through the REAL `runDriverControl`, each button UP before the drop (tick 30), pressed during it (35), still DOWN at the reconnect (40) and to tick 55, released at 58, pressed again at 60: robot two's A (claw line still closed and its command number unchanged at 55; at 70 open, command number +1); robot x's X and Right on `kControls` (arm and flywheel command numbers unchanged, target 0, not spinning at 55; at 75 the target 95 and spinning); robot x's Left and Right on `kAltControls` (no homing push begun, no hood preset at 55; at 75 one push begun, one hood command, 'middle' sent — C high, D low). 17 assertions | **VR-1** (the verifier's VM1: the recording line deleted) RED — the new case; run again on the new case alone (`VR-1detail`) 13 of its 17 checks red, in ALL THREE blocks (the four green: the line before the drop, robot x's target at 75 — the spurious "score" already sent it there —, the hood's two lines at 75 — the spurious and the real press both send 'middle'). **VR-1b** (the drop records what the controller READS — 0 while disconnected on a real one, HA-103, and on the fake) RED — the new case and "a press must be SEEN". **FR-C1** and **FR-D4** re-run RED — each now in two cases (the fix round's and the new). None killed by the cap or the timeout; `md5sum -c` OK after the list |
| VR-2 (minor) | `driver_inputs.hpp` :496 cites A4-D26 for U5's tightening | **FIXED:** A4-D27 (a `//` comment — no generated page moves) | — |
| VR-3 (minor) | guide 14 promises the whole recipe layer frozen and names only `then()` as open | **FIXED, with its sibling:** guide 14 :201-207 — frozen "all but the pieces the register leaves outside them" (for the recipe layer `then()`, its log lines' wording, and — not yet ruled — its mechanism steps), "routines written today against the frozen surfaces"; :256-263 — "What stays open in the recipe layer": `then()` as before, AND the mechanism steps (`r.move` / `r.run` / `r.set` / `r.spinUp` / `r.home` / `r.stop`, the START forms, `waitUntilDone`), the second constructor and `lastMechanism()` not yet ruled, the F10 row giving both readings, not pinned meanwhile, an auton using them may need an edit if their spelling moves before the ruling; :266 "both API tiers but for the pieces named above". **Guide 10** (the sibling): the banner ends "Two parts of this chapter are outside this banner, and each says so: the driver loop (under `drive()`) and the mechanisms (the auton steps among them)"; the mechanism section's opening box gains the carve-out (the mechanism words, the service, the driver controls, the route binder and the direction check are in no register row — the seam beneath them is F11's, NOT frozen —; whether the steps, the second constructor and `lastMechanism()` join F10 is not yet ruled; the two pieces the route binder appended to the FROZEN F8 — `CommandEntry::doc`, the four-argument `on()` — ARE pinned, `f8_signature_pin_test.cpp` :343, :367). The wording follows the six D1 places (guide 9's box, the F10 row, the changelog's 2.5 block) | — (prose) |

- **RESUMED** *(`date`: Thu Oct 29 11:50:22 AM EDT 2026)*: the 2026-10-27 round stopped DURING its gates (the
  compile-refusal harness at 189/251 in `vround/harness.log`; no gate verdict was written above). The same three
  findings came back from the verifier; each was re-checked against the tree at this resumption before anything
  else: VR-1's case is at `test/mechanism_driver_loop_test.cpp` :695 (read in full — every captured value starts at
  a sentinel that fails its CHECK if the capture never runs), VR-2's comment reads A4-D27 (`grep -rn "A4-D2[6-9]"
  include test docs | grep -v MECH-PROGRESS` → only `driver_inputs.hpp` :496), VR-3's guide 14 and guide 10 edits
  are in the tree (`git diff 43ac361` read). The mutated header's md5 equals `vround/pre_mut.md5`, and the test
  file's mtime (22:01:45) predates every mutation run (22:01–22:07), so the table's verdicts are on these files.
  What follows: one more sibling from the prose review, VR-1 re-run in this session, and every gate.
- **VR-1 re-run in THIS session** (`vround/vr_list_resume.py`, id `VR-1r`; the verifier's VM1 exactly: `if
  (!connected) { (void)edge_[i].update(true); return false; }` → `if (!connected) { return false; }`): a fresh build
  of all 16 objects that depend on `driver_inputs.hpp` (the depend step first; 16 of 16 recompiled; the binary newer
  than the mutant), the WHOLE suite under the 4 GiB / 600 s cap — **RED**: 2626 cases, 2625 passed, 1 failed, the one
  failure the new case ("a button pressed DURING a controller drop …"); not killed by the cap or the timeout.
  Restored byte-identical (md5 `b01fe37c…` before and after, = `vround/pre_mut.md5`), its timestamp bumped.
- **One more sibling of VR-3, from this resumption's prose review** (every grep RUN and its hits READ before this
  line). Searched the public docs for a freeze promise: `need rewriting|both tiers are now stable|both API
  tiers|FROZEN at BOTH|whole recipe layer|recipe layer is frozen|One related piece|stays deliberately open`, then
  "frozen" beside routine / recipe / auton / tier / F10. Hits: guide 10 :16 and :25 (the banner — carved out at
  :26-27 by the earlier pass: true); guide 14 :201, :207, :266 (VR-3's edits: true); guide 9 :10-13, :28 (D1's box:
  true); guide 10 :423 (the mechanism section's carve-out: true); `hardware-assumptions.md` :477 (`startAt` frozen:
  true); `roadmap.md` :486 (the M-milestone record of the 2026-08-12 freeze — a dated record, left), the changelog
  :2123 (the 2026-08-12 entry — history, left), the guide README :131-137 and :154 (maintenance-log items "Executed
  2026-08-12" / "2026-08-13" — history, left). **One live promise A4 made untrue:** `docs/vexbuilder-integration.md`
  :74, the status table's "The HAL, motion, localization, sequencing, both auton-authoring tiers | ✅ | done and
  frozen" — now "done and frozen (but for the pieces the Freeze Register leaves outside: the recipe layer's `then()`
  and — not yet ruled — its mechanism steps, row F10; the mechanism ids of 4.2.3 are a robot's, never frozen)".
- **A process incident of this resumption, found and undone before any verdict was taken** (for the coordinator,
  unnumbered — not a hardware belief): I first ran `src_build_gate.py self-test` under `timeout 600`; under this
  machine's shared load (CAL1-A2a's review was building beside it) the self-test needs ~16 minutes, so `timeout`
  SIGTERMed it at 12:06:40 — in the middle of a plant. SIGTERM skips Python's `finally`/`__exit__`, so its
  byte-exact restore never ran: **`src/build_selection.hpp` was left with the tank-drive branch's `kProgram =
  Program::Drive` rewritten to `Program::BenchTests`** (found by `git diff --stat 43ac361 -- src/`, which showed a
  third file). The `check` that followed ran ~25 s on that planted tree and was interrupted by me (SIGINT); NO verdict
  of it is used. The planted copy is kept (`vround/build_selection.hpp.planted-by-killed-selftest`); the file was
  restored from `HEAD` (A4 never edits it — every earlier gate's `git diff --stat 43ac361 -- src/` lists only the two
  `case` lines) and touched; the tree re-counted: 51 modified, 66 new — exactly the fix round's counts. Nothing that
  ran before 11:56:40 (the build, the suite, the harness) saw a plant. The self-test and `check` were then re-run
  under `timeout -s INT 3600` (SIGINT raises `KeyboardInterrupt`, so a kill still runs the restores). **The lesson:
  never wrap the src gate's self-test in a SIGTERM timeout, and never one shorter than ~20 minutes on a loaded
  machine — use `timeout -s INT`, and read `git diff --stat 43ac361 -- src/` after it.**
- **Status pointers moved** (A4's own lines only): `build-order.md`'s first bold `Next:` ("… its gates finished
  2026-10-29 …, the freeze wording in guides 10 and 14 and the integration doc") and its MECH entry (:1626),
  `roadmap.md`'s "Now (2026-10-29)" and the MECH row (:1765), `REMAINING-WORK.md` row 12, `HANDOFF-remaining-work.md`
  :289; `PROJECT-BRIEFING.md`'s generated block regenerated from them (on the fresh final binary).

**Every gate on the FINAL tree** (`date` at the end below):
- **The host suite** (`build.sh vr3`: the API pages generated, the binary deleted, the briefing generated, the full
  doc-gated build, the suite capped, the briefing generated again): **2626/2626 passed, 2,799,446 assertions, 3
  skipped** (before the round: 2625, 2,799,429 — the new case's 17). After the src gate (whose self-test re-times
  headers): `build.sh vr3post` — 239 objects recompiled — the same 2626 / 2,799,446. `python3 tools/suite_run.py
  green`: green both times, 2626 cases, 2,799,446 assertions. MECH-A4's own cases
  (`-tc='MECH-A4*,guide-18c*,guide-18d*,guide-18e*,cookbook-07a*'`): **82 cases, 737 assertions** (81 / 720 + the
  new case).
- **The compile-refusal harness**, every TU (`ctest --test-dir build/test --output-on-failure`, capped): **251/251
  compile refusals passed** (and the suite as CTest #1) — 252/252, 212.6 s; TUs 251 = CTests 251
  (`vround/harness_resume.log`). The 2026-10-27 run of it, stopped at 189/251, is superseded.
- **`src_build_gate.py self-test`**: OK (235 detector cases, each against a real build; 15 m 39 s);
  **`check`: PASS — all 7 builds link**, 0 disallowed warnings (2 m 32 s); `git diff --stat 43ac361 -- src/ tools/
  Makefile`: the two `case` lines only.
- **The ARM header gate**: 226 headers, rc 0, no output. **The CI greps**: PROS and `sim/` empty; `PROBE_` 0; no
  "PROTOTYPE" / "(proposed" in an A4 file.
- **`api_doc_tool`**: self-test OK; check-coverage 4,960 public entities across 186 headers, all documented;
  check-fresh OK (no `///` changed this round — VR-2's is a `//` comment); check-examples 603 quoted lines verbatim;
  check-removability OK. **`briefing_status.py`** self-test OK, check current. **`doc_staleness_audit.py`** self-test
  OK, audit clean. **C7** empty. (Run twice — `gates.sh vr3g`, then `vr3g2` after the pointer edits and the
  briefing's regeneration: the same.)

**Files this round touched** — tests: `test/mechanism_driver_loop_test.cpp` (the new case, :695); library:
`mechanism/driver_inputs.hpp` (:496, a `//` comment); docs: guide 14 (:201-207, :256-268), guide 10 (the banner
:26-27, the mechanism section's box), `vexbuilder-integration.md` (:74), the pointers above, this log. No file of an
in-flight chunk touched; `src/` as at the fix round (the two `case` lines); nothing committed; no `pros`; no
`/dev/ttyACM*`. The tree: 51 files modified from `43ac361`, 66 new. `date`: Thu Oct 29 12:31:04 PM EDT 2026.

### The coordinator's merge verification — MECH-A4 (2026-10-29, main `73044ca` + the squash of `mech-a4`)
- **The team lead's three rulings (2026-10-29, asked at the merge):** (1) the freeze line — the mechanism steps, `Routine r{ctx,
  name}` and `lastMechanism()` stay OPEN (unpinned) for now; pinning later is additive (A4-D13 stands as the proposal, A4-D31);
  (2) the driver-start home's exception to "the most recent input wins" — ACCEPTED (A4-D26, C2); (3) U5 tightened — a preset needs a
  homing control, `.unhomedVolts` answers for a held-only mechanism — ACCEPTED (A4-D27, C4). Recorded in the roadmap (the A4 "Now"
  paragraph, the F10 row), build-order, REMAINING-WORK row 12 and HANDOFF.
- **Audit before the merge:** every file equals its newest mutation backup (version.hpp differs from V1's only by the fix round's later
  doc comment); `src/` changed by exactly the two `case` lines the row allows; no stray root file; no merge in progress. The transport
  commit `1fe8f52`, then `git merge --squash mech-a4` (base `43ac361`).
- **Two conflicts:** the briefing (regenerated) and build-order's `Next:` line (a word-level three-way merge against `43ac361`).
- **Gates on the merged tree** (`.claude/coordinator/logs/mech_a4_merge_verify.out`): host build with the doc gates; suite **2,626 |
  2,626 passed, 3 skipped**; `src_build_gate.py` self-test OK and check **PASS, all 7 builds**; ARM header gate over **226** headers
  clean; CI greps empty; api check-fresh, the briefing, removability and the staleness audit clean; the compile-refusal harness **251
  of 251**.
- **The coordinator's hand mutations** (`muts_mech_a4_main.json`; each on a fresh link, the whole suite under a 4 GiB / 300 s cap,
  restored and hash-checked): **7 of 7 caught** — MD1 (a counted UNCONFIRMED stops the auton), MD2 (runAuton never hands over —
  `enterAutonomous` not called: M35 proper), MD3 (the service ticked before the driver's controls are read), MD4 (a route marker
  with a 0 ms limit not refused), MD5 (a START form's NOT_HOMED does not stop the chain), MD6 (a dropped controller's button read as
  a press — C1), MD7 (a preset press during the driver-start home not dropped — A4-D26) RED. Restored tree 2,626 | 2,626.

---

## MECH-A5 — robot two and the programs

- **Started:** Thu Oct 29 02:12:30 PM EDT 2026 (`date`).
- **Tree at start:** `4691757` (main: MECH-A4 `46c874c`, its landing dates `4691757`; the plan at `73044ca`, `ec3754f`,
  `2ef1f3f` — G3a's brief answered; CAL1 Part A1 `ceb17c0`; the tooling at `c09e022`), clean, in the git worktree
  `.claude/worktrees/mech-a5` (branch `mech-a5`) — NOT the main working tree.
- **Go:** the team lead, 2026-10-08: "Accept all, go" — N1–N13 as recommended; his three rulings at A4's merge
  (2026-10-29: the steps' freeze stays OPEN; A4-D26 the driver-start home's exception; A4-D27 a preset needs a homing
  control); THE SPRINT RULING (2026-10-29, "by the 21st"): the panel (G3a) makes and drives a robot with its mechanisms
  and draws routes by 2026-11-21; MECH-A5 and G3a Part 1 start NOW, side by side; NOTHING IS SKIPPED. The coordinator's
  workflow launched MECH-A5 ONLY. Do not commit; no `pros` command; no `/dev/ttyACM*`. Mutation backups and probes under
  `.claude/coordinator/scratch/mech-a5/` (never `/tmp`, never the repository root). **Memory safety (2026-10-15):** every
  test binary runs as `(ulimit -v 4194304; timeout 600 <cmd>)`; a mutant killed by the cap or the timeout counts as
  CAUGHT (said which); the briefing is regenerated only after `rm -f build/test/shulib_tests`.
- **Order change (the sprint):** CAL1 has NOT landed before this run (only its Part A1) — the brief's own fallback
  applies (N9): A5 re-points today's drop call in `calibrate_program.cpp` at the new list (one line) and CAL1 maps the
  core's list later; `declaredMotorRole()` is not built; CAL1's typed half, if met, stays untouched.
- **G3a's ask (its RC5, D4 accepted 2026-09-29):** the driver file's `// [driver file two]` block ENDS BEFORE THE FEEL
  (`kFeel`, `kLoop` hand-written, outside it), so G3a's `controls` region wraps only the WHICH BUTTON heading and the
  `kControls` statement. (The brief's §1.1 draft put the feel inside the block — a delta, logged below.)
- **Reading (the team lead's ruling of 2026-10-22):** the brief's header and answers, the re-check's amendments, N1–N13,
  the Decisions, §6, §7, §8 landmines IN FULL; every section/row/test list naming MECH-A5 or its files (§1.1's robot-two
  blocks and the count, §1.13 entire, §1.16, §2's A5 row, §2.1, §3.5, §4.1–§4.3); the rest by heading. MECH-A1..A4's
  entries of this log grepped for every `A5` hand-on.
- **In flight beside this run (not touched here):** CAL1 Part A2a (`calibration/calibrate_program.hpp`,
  `hal/pros/calibrate.hpp`, `calibration/calibrate_panel.hpp`, `test/calibrate_program_*`); VIS1-A1 (`hal/device_registry*`,
  `hal/device_provider*`, `config/robot_config.hpp`, `config/camera_config.hpp`, `robot/robot.hpp`, `hal/vision_frames.hpp`,
  `hal/pros/ai_vision.hpp`, `sim/camera_plant.hpp`, `test/pros_shim/pros/shim_control.hpp`); IMU1 Part A
  (`hal/pros/imu.hpp`, …); L6-A1 next; G3a Part 1 (worktree g3a-p1: `tools/robot_tool.py`, robot two's robot file,
  `[chassis]` markers in `src/robots/chassis_table.hpp`, `src/build_selection.hpp`, the freshness gate and a `[generated]`
  step in `tools/src_build_gate.py`) — shares ONLY `tools/src_build_gate.py` (A5 edits step 0's per-robot functions) and
  `test/CMakeLists.txt` with this run.

### 14:12 — start
- `docs/internal/RESUMING.md` read (standing rules, pace rules 1–8). The brief read as ruled above. `## MECH-A1` ..
  `## MECH-A4` grepped for `A5`: the hand-ons are (1) n56, n57 (DriverFile's two `static_assert`s) and n31 (the robot
  table file's `clearOfDrive` `static_assert`) join the harness here (A1's D18, A4's :2530); (2) A1's recorded-path pin
  moves to the table file (:527-529); (3) `sizeof(MechanismService)` 9 640 B on ARM `-Os` (A3-D17 re-measured) — one per
  program; (4) HA-182's line is "worded for a boot line the programs print from MECH-A5" — A5 prints it; (5) guide 16's
  quoted auton builds `Routine r{ctx.chassis, …}` — A5 changes the team's file; (6) HA-185 held for A5; (7) the latent
  COMP-b line-length gap in `competition_run.hpp`'s routine-stop line (214 > 200 bytes worst case, cut not overflowed) —
  src/ is A5's; (8) robot two's `toggle(claw, Button::A)` inherits A4's C1 fix (a controller drop never flips the claw);
  HA-147's Station J.9.5 settles the service path too; (9) A4-D26 / A4-D27 as accepted.

### 14:12–14:38 — the baseline, then the src/ half written *(`date` at this entry: Thu Oct 29 02:37:50 PM EDT 2026)*
- **Baseline at `4691757`** (`scratch/mech-a5/`): host build 36 s; the suite through `tools/suite_run.py show` **2,626
  cases, 0 failed, 3 skipped, 2,799,446 assertions**; `src_build_gate.py check` **PASS, all 7 builds link** (2 m 27 s).
- **The brief's file:line citations, re-checked at this base** (the brief was written against `tree_t43d`): `calibrate_
  program.cpp`'s drop call is **:1138-1139** (the brief: :1129-1130); `mechanism_table.hpp`'s old check 4 was :393-402,
  the drop :448-489, the "only robot two" pin :522-523; `boot_source.hpp`'s `mechanismTableProblem` :130, the two
  `toMechanismConfig` :202-222, `mechanismConfigIndexOfPneumatic` :159, the declared-port rule :451-471;
  `driver_panel.hpp`'s `motorClawBootLine`/`pneumaticClawBootLine` :152-202 and `clawRowText` :277; `guarded_run.hpp`'s
  `stopAfterProgramError` :83-97; `autons.cpp`'s Routine :53; `competition_program.cpp`'s chain :680-683 (the brief:
  :640-644), its claw lines :785-796; `auton_tester_program.cpp`'s chain :472-475; `drive_program.cpp`'s run :695 (now
  :698); `xdrive_program.cpp` :54 and :239 (the brief: :237); `driver_controls.hpp`'s users exactly the brief's T6 list.
  **No `src/team` exception exists in step 0 at this base** (the brief's T3 described one): `_stray_selection_problems`
  (:811-836) exempts only the table files, `test/` and a `static_assert` — the driver file named robot two's list only
  inside its `static_assert`s. So the per-robot rule ADDS the own-team-file exception rather than narrowing one.
- **Written (src/):** `src/robots/mechanism_table.hpp` (rewritten: robot two's region exactly as §1.1, in `namespace
  robot_two`; `kNoMechanisms`; `mechanismTableFor()` a case per robot, no default; `kMechanisms`; `mechanismsClearOf()`
  over the library's ONE rule `mechanism::clearOfDrive` — every smart role and the IMU; `clearOfItsDrive()` refusing BY
  NAME; `isDeclaredMechanismPort()` + `dropDeclaredMechanismPorts()` re-implemented over the new list; the selection pins
  by ADDRESS, no counts); `src/common/driver_heading.hpp` (NEW — `driverHeading()` moved, body unchanged);
  `src/common/driver_file.hpp` (NEW — `DriverFile<kControls, kFeel, kLoop>`, its two `static_assert`s, the brain's tick
  supplied, `bind(service)`, `run(robot, service, pacer, disabled, perTick)`, `releasedLeftoverAtEntry()` for FREED);
  `src/team/robot_two_driver.hpp` (rewritten — the button map in words KEPT, `// [driver file two]` around the WHICH
  BUTTON heading and `kControls` ONLY, the feel and the loop after it, `using RobotTwoDriver = DriverFile<…>;`);
  `src/common/driver_controls.hpp` DELETED; `src/common/boot_source.hpp` (`bootConfigFrom` through
  `mechanism::toMechanismConfig` in the list's order; `mechanismTableProblem`, both old `toMechanismConfig`s,
  `mechanismConfigIndexOfPneumatic`, `toMotorCartridge`/`toSafeBrakeMode`/`toPneumaticLevel`/`copyMechanismName`/
  `mechanismNameFits` and the two `static_assert`s REMOVED; the declared ports from `mechanism::declaredMotorPorts`; the
  declared-port rule over every smart ROLE — motors in T3's words byte for byte, sensors named by mechanism and role);
  `src/common/guarded_run.hpp` (`ChainGap`, `pacerChainMissing(chassis, guard, service, watch)` naming the first layer
  that did not pace, `pacerChainProven` over it; `stopAfterProgramError` also cancels each present mechanism's registered
  claimant); `src/common/driver_panel.hpp` (the mechanism boot lines, `routesBootLine`, the claw row reading the service);
  the three programs (the service in static storage, in the chain guard → service → watch for the two auton programs and
  around the raw pacer for Drive, bound over `robot.mechanismParts().devices()` inside the boot's try; the route registry +
  `MechanismRoutes` in the two auton programs, registered after the bind, handed to `runAuton`; the boot lines; the FREED
  word from the service); `src/programs/{competition_run,auton_tester_run}.hpp` (the `runAuton` service overload with the
  registry; `CompetitionParts::present` → `mechanisms` + `commands`; `runTesterOnce(…, service, commands, …)`);
  `src/programs/xdrive_program.cpp` (its include and one comment); `src/team/autons.cpp` (`Routine r{ctx, …}`, the table
  include, `using namespace robot_two;`, a comment on mechanism steps — no auton steps a mechanism: G4).
- **`calibrate_program.cpp` is UNCHANGED** — N9's fallback said "one line": `dropDeclaredMechanismPorts(kMechanisms, …)`
  keeps its signature over the new type and `kMechanisms` IS the new list, so the call (and its pinned text) did not move.
- **All seven `make` variants compile and link** (each `make clean` first; 8 vendor-ignored warnings each, none of ours) —
  after one fix: a `-Wformat-truncation` in the pneumatic declaration line (a 64-byte buffer; 80 now, its sum commented).

### 14:38–15:09 — the tests over the src/ half; the suite GREEN *(`date` at this entry: Thu Oct 29 03:09:39 PM EDT 2026)*
- **Rewritten / new tests** (each case names the bug it catches): `test/mechanism_table_test.cpp` (rewritten — robot two's
  declarations and wiring pinned literally; the claw's levels on ONE `.positions` line; WHERE each declaration and
  `wire(...)` line was written, pinned by the suffix `src/robots/mechanism_table.hpp` — A1's D18 pin moved here; the
  region byte for byte the fixture's; the selection for every robot by ADDRESS; `isDeclaredMechanismPort` and the
  Calibrate filter over the new list; THE DRIVE CHECK over every smart role incl. sensors on drive ports and the IMU, a
  3-wire line numbered like the IMU's port never a collision; the Calibrate call site pin UNCHANGED);
  `test/boot_source_test.cpp` (6s–6x: robot two's derived config written literally; a list whose kinds are
  interleaved lands in the LIST's order; NEW 6x: a mechanism SENSOR port on a record's / typed table's drive port
  refuses the boot by mechanism and role); `test/driver_heading_test.cpp` (NEW — the heading cases moved from
  `driver_controls_test.cpp`, which is DELETED with its types; the ONE-copy pin; the retired header's grep);
  `test/robot_two_driver_test.cpp` (rebuilt on the service; NEW **M5a**: the same long script — every button alone and
  in pairs, A short and held, a drop with R1 held and A pressed during it, a disable mid-hold, three entries — through
  T3's bindings exactly as T4 bound them and through the driver file: ports 6/7 (mV, brake() and move counts,
  stopped, brake mode) and line A (every write) **IDENTICAL on all 159 ticks**; T9b the C1 drop fix inherited; T10
  refusals now PreconditionErrors by sentence; the region pin for G3a's ask); `test/driver_panel_claws_test.cpp`
  (rewritten for the mechanism boot lines word for word, every control word over robot x's fixture controls, the
  ROUTES line, the claw row from the service, FREED per entry); `test/guarded_run_test.cpp` (G1 — `pacerChainMissing`
  names each gap, the P1 control case FAILS; NEW G2b — the stop cancels the slot: no re-command at the next tick);
  `test/robot_two_claws_example_test.cpp` (NEW — the claws auton and the claws ROUTE through the REAL tester run,
  each step's verdict asserted; a marker with no time limit REFUSED; a typo id unknown); the rigs
  (`test/auton_tester_rig.hpp`, `test/competition_rig.hpp`: guard → service → watch over robot two's REAL list on
  counting fakes, the route registry registered); the source pins (`auton_tester_source_test`, `competition_source_test`,
  `drive_program_source_test`: chain order, service-before-Robot, bind-after-Robot-inside-the-try, registerInto after
  bind, `pacerChainMissing`, the boot lines, the unbuild order); `competition_run_test` R7/R8/R12/R18 and
  `auton_tester_run_test` (18b) re-pointed at the rig's counting fakes (18b now cancels a STEP IN FLIGHT — the slot is the
  claimant; a pre-run foreign claim is released by `enterAutonomous()` by design, D1); the route-auton case now RUNS
  (the tester passes the registry); `competition_watch_test` W6, `auton_abort_watch_test` 13b, `mechanism_wiring_test`
  P9a, `drive_protection_test` (one string), `mechanism_fixtures.hpp` (robot two's region now byte for byte the real
  one: the `.moveTime` comment), `mechanism_driver_fixtures.hpp` (its banner).
- **Two findings while writing them:** (1) a mechanism NAME is at most ~18–19 characters in any `mechanisms(...)` list
  — its route ids must fit F8's 23 (`claw_pivot_six_lef_stop`), so the config's 23-character field (which the old
  boot refusal guarded) can no longer be reached; the 6t case pins the longest one carried whole, and **G3a's panel
  needs this limit for its name field** (for the coordinator). (2) `DriverInputs::bind` checks a control's mechanism
  exists on the service's robot, but not that every mechanism of that robot has a control — T4's binder refused "the
  robot declares 'intake', and no control names it" at boot; **`DriverFile::bind` now checks coverage first** (a
  PreconditionError naming the problem, nothing bound), so T10's four refusals all hold (logged as a delta).
- Doc gates met on the way: cookbook 06's robot-two section re-quoted (N6's three listings, the unbroken-run rule —
  G3a Part 4's `example-source` directive has not landed); guide 16's quoted 40 in auton (`Routine r{ctx, …}`); two
  history lines annotated for the staleness audit's dead paths (`build-order.md` T4 status, `roadmap.md` T4 row:
  "(deleted at MECH-A5 …)").
- **Suite: 2,620 cases, 0 failed, 3 skipped, 2,800,303 assertions** (`build.sh b7`: the stale binary removed and the
  briefing regenerated first; the run capped at 4 GiB / 600 s).

### 15:09–15:31 — the docs in the same change; the src gate's self-test; the suite again *(`date` at this entry: Thu Oct 29 03:31:28 PM EDT 2026)*
- **`tools/src_build_gate.py self-test`: OK — 249 detector cases, each against a real build** (14 m 9 s; it plants in
  the worktree and restored every file — `git status` after it lists only this run's own changes). Case 26's
  `DRIVE_PLANT` re-pointed (the drive program's bind names `mechanismTableFor(RobotId::BenchBot)` for the plant); NEW
  case 28b: the robot namespaces derived from the table file (`{"robot_two": {"kRobotTwoMechanisms"}}`), then the brief's
  nine planted cases (`out5/gate_cases.txt`), each alone — REFUSED: `robot_two::claw` in a program, `using namespace
  robot_two` in a program, `src/common/` re-opening `namespace …robot_two`, robot THREE's driver file naming robot two's
  list qualified and unqualified, a chassis table named in robot two's own team file, a team file of NO robot saying
  `using namespace robot_two`; PASSING: robot two's own second team file naming its list and namespace, a program's
  `static_assert` naming it — then the auton file through `TEAM_FILE_ROBOT` (passes), and a planted robot THREE in the
  table file derived and its namespace refused in a program. The real tree: `one_chain_problems() == []`.
- **The docs (same change):**
  - `docs/hardware-assumptions.md`: **HA-185 registered** (index row after HA-184; the long entry in a new "MECH-A5's
    addition" section after MECH-A3's; the status 178 → **179**, a MECH-A5 paragraph; the held-numbers sentence now holds
    HA-181 only — the `**Held numbers**` line itself is unchanged: its rule lets a landed number stay in its range, as
    A1–A3 left HA-181..HA-186). **HA-159's text:** the index row and the entry's claim, source, settle and blast radius
    now say "swap `Low` and `High` on the claw's `.positions` line" (`kClawOpenCommand` named only as "until MECH-A5").
    Re-pointed Sources: HA-147 (robot two's claw on `DriverInputs` since A5), HA-153 (the declarations and their `wire`
    lines; `Forward` / `Reversed`), HA-154 (the 12 V is the driver file's `whileHeld` now; the boot's no-stall line
    quoted), HA-156 (the declarations' `.safe = Hold`; `whileHeld` through the service; the M5a differential), HA-161
    (`driver_controls.hpp` deleted — the static-storage rule stands in `driver_file.hpp`), HA-182 (the boot line is
    printed since A5, for a position or speed mechanism — robot two has none).
  - `docs/internal/chunks/R3a-BENCH-WORKSHEET.md`: J.3.1 (the eight `[MECH]` pivot lines and the two ABSENT lines, the old
    one-line form kept as history), J.3.3 (FREED's serial line is the service's `[WARN][MECH]` one), J.4's fix (the
    declaration's `{Forward}` → `{Reversed}`), J.9's intro and J.9.1 (the claw's three lines), J.9.3 (swap `Low` and
    `High`), **NEW J.9.8** (HA-185: the slow-motion video, full and low air), "What to send back" items 3 and 7 (item 3 no
    longer asks for the banner's claw sentence — J.3.2 was RETIRED at T4 Part 3; a stale item fixed on the way), K's
    boot-line paragraph, K.2.3, M.2.2 (the tester's eight lines and its `ROUTES:` line), M.9.1, Station M's header
    ("a route auton is neither station's" — both programs register the route ids now).
  - Guide 16 (the registry paragraph rewritten — both programs pass the service and a registry holding the robot's
    route ids; **NEW section "The claws in an auton"**: `claws_example::grabAndPivot` quoted from
    `test/robot_two_claws_example_test.cpp` — held verbatim by the example scan — each step's verdict, the route ids as
    markers, the tester's boot lines; the host-test paragraph: the claws safe at the end, the `ctx.chassis` source pin);
    guide 17 (the chain with the service, the boot phase row, the `[MECH]` lines and `ROUTES:`, the chain proof's three
    links, "No route autons" → "Route autons fire only the mechanisms' ids"); guide 18 (five parts in; the starting-values
    line printed since A5; a pointer to guide 16's claws auton); guide 10 (two "no program builds a service yet"
    sentences); guide 14 (two); guide 15 glossary (starting values); guide 11 (the `TEL` release line: a program on the
    service says it with the `MECH` line); guide 07 (`src/common/`'s and `src/robots/`'s descriptions); cookbook 06 (one
    "until the core's fifth part" note) and 07 (its intro); `README.md` (the Mechanisms bullet names the core; the
    `src/robots/` and `src/common/` layout lines); `src/team/README.md` (`driver_file.hpp` / `driver_heading.hpp`; the
    per-robot namespace rule); `docs/changelog.md` (an entry at the top of 2.5: "the mechanism core, part 5 … — no API
    change"); `docs/roadmap.md` (a "Now" paragraph, the MECH row's A5 line); `docs/internal/build-order.md` (the MECH
    entry's A5 paragraph; the first bold Next: line's MECH clause); `docs/internal/REMAINING-WORK.md` row 12;
    `docs/internal/HANDOFF-remaining-work.md` (one parenthesis on the library list; two `driver_controls.hpp` mentions
    annotated "deleted at MECH-A5").
  - `include/shulib/mechanism/{wiring,kinds,service,route_ids}.hpp`: four "MECH-A5 adds it / no program does yet"
    comments made present tense (one `///` — `startingValuesLine`'s); `python3 tools/api_doc_tool.py generate` re-run
    (`docs/api/{kinds,route_ids,service,wiring}.md` regenerated).
  - `test/autons_test.cpp`: its banner (the rig is the program's whole chain since A5); (1b) also asserts robot two's
    mechanisms SAFE after every registered auton; **NEW (1c)**: `src/team/autons.cpp` builds every `Routine` on `ctx`
    (never `ctx.chassis`), says `using namespace robot_two;` once and includes the table — the brief's U3 source pin.
- **`build.sh b6`: rc 0** — doc coverage 4,960 entities / 186 headers; **doc example scan 591 quoted lines, all verbatim
  (10 source files); 6 whole-run listings, each one unbroken run**; removability clean; the staleness audit clean.
- **Suite (`tools/suite_run.py green`, capped): green — 2,621 cases, 2,800,327 assertions.** `ctest -R compile_refusal`:
  **255/255 passed** (n31 ×2, n56, n57 among them, each "refused as expected" for its own reason).

### 15:31–15:45 — the mutations, each observed red; the final gates *(`date` at this entry: Thu Oct 29 03:45:23 PM EDT 2026)*
- **How each ran.** A C++ mutation: the file backed up under `scratch/mech-a5/mut/`, ONE exact replacement, a FRESH
  full host build (`build.sh`: the stale binary removed, the briefing regenerated, the whole `cmake --build build/test`
  — doc gates, suite, everything), then the named cases from that binary, capped (`ulimit -v 4194304; timeout 600`);
  the file restored in a `finally` (identical to its backup, checked) and its timestamp bumped. A refusal-harness
  mutation: its CTest, which builds the TU's own object target (`run_refusal.py` → `cmake --build --target cr_<stem>`),
  so it compiled against the mutant. A gate mutation: case 28b's OWN lines, extracted from the self-test's source and
  run against the mutated gate module (`scratch/mech-a5/gate_case_runner.py`), with every plant restored. A mutation
  that changes a line a document quotes changed the quote too (`mutate_multi.py`), so the TEST — not the doc gate —
  is what judged it. Scripts: `scratch/mech-a5/{mutate.py,mutate_multi.py,mut_run.sh,batch1.sh}`; logs `mut/*.out`.
- **Every one RED (none caught by the cap or the timeout):**

| # | Mutation (file) | Caught by (observed) |
|---|---|---|
| M26 | `bootConfigFrom` loses the per-line levels — every `pneumatic{Start,Safe}ByLine` zeroed after the projection (`src/common/boot_source.hpp`) | **6y RED** (4 checks: start E/F, safe E/F read 0); **6s GREEN under it** — robot two's one-line claw cannot see it, as the brief said (6y is A5-D11) |
| M27 | the own-team-file exception admits EVERY per-robot name (`per_robot = frozenset()`) — chassis tables included (`tools/src_build_gate.py`) | **28b RED** (2 of 16): "a chassis table in robot two's own team file — expected REFUSED"; robot three's file naming robot two's list |
| M28 | the chain proof ignores the service (its `ticks()` check removed) (`src/common/guarded_run.hpp`) | **G1 RED** (3): the service left out → `None` instead of `Service`; `pacerChainProven` true |
| M29 | `mechanismsClearOf` checks MOTOR roles only (`src/robots/mechanism_table.hpp`) | **"THE DRIVE CHECK" RED** (3: an angle sensor on a drive port, a piece sensor on one, a distance sensor on the IMU port) **and the harness: `compile_refusal_n31_a_sensor_on_a_drive_port` FAILED** (it compiled) |
| M33 | the namespace rule skips `using namespace` (`tools/src_build_gate.py`) | **28b RED** (3): a program's `using namespace robot_two`; a no-robot team file's; the auton file's `using namespace robot_three` |
| M34 | the exception admits EVERY robot's mechanism lists, not the file's own robot's (`tools/src_build_gate.py`) | **28b RED** (1): robot three's team file naming robot TWO's list — **it SURVIVED the brief's nine planted cases** (all 14 earlier expectations green under it); the two robot-three expectations were added for it (A5-D9) |
| M47a | the rig forgets `routes.registerInto(commands)` (`test/auton_tester_rig.hpp`) | **"claws route" RED**: 0 markers fired, 4 unknown ids, no verdict lines |
| M47b | the Auton Tester forgets `st.routes->registerInto(*st.commands);` (`src/programs/auton_tester_program.cpp`) | **the tester's chain/registry pin RED** (3: the call's count, `registerInto(`'s count, the routes-before-proof order) |
| MA | `DriverFile::bind`'s coverage check disabled (`src/common/driver_file.hpp`) | **T10 RED**: "the driver file bound a robot its controls are not for" (the extra-intake list) |
| MB | FREED from the service's LIFETIME count (`releasedBefore_ = 0`) (`src/common/driver_file.hpp`) | **robot_two_driver's FREED case RED** (2: still FREED after the next entry; its row) and **driver_panel_claws' "FREED for THAT entry" RED** (1) |
| MC | `stopAfterProgramError` no longer cancels the claimant (`src/common/guarded_run.hpp`) | **G2b RED**: the pivot still claimed, no cancel, and port 6 back at 12 V at every later tick |
| MD | the boot's declared-port rule skips SENSOR roles (`src/common/boot_source.hpp`) | **6x RED** (every row: the boot not refused, no headline, the config built with 7 mechanisms) |
| ME | robot two's driver file swaps pivot 7's buttons (`whileHeld(clawPivot7, Button::L2, Button::L1, …)`; cookbook 06's quote changed with it) | **M5a RED**: port 7 at −12000 mV where T3's bindings gave +12000, tick after tick |
| M1c | the team's 40 in auton built as `Routine r{ctx.chassis, …}` (guide 16's quote changed with it) | **autons (1c) RED** (2); (1b) GREEN under it — the auton has no mechanism step, which is exactly why the source pin exists |
| MF | DriverFile's re-zero `static_assert` made `true \|\| …` | **`compile_refusal_n57_a_control_on_the_rezero_button` FAILED** ("COMPILED … was NOT refused") |
| MG | DriverFile's tick `static_assert` made `true \|\| …` | **`compile_refusal_n56_a_tick_that_is_not_the_brains` FAILED** ("COMPILED: … was NOT refused; it must be refused naming: [\"leave .tickPeriod out of the driver file: the loop runs at the brain's tick\"]") |

- **The final gates (the restored tree; `git status` lists only this run's files):**
  - `build.sh final1`: rc 0. **Suite: 2,622 cases, 0 failed, 3 skipped, 2,800,355 assertions** (`suite_run.py green`;
    the binary itself, capped, the same). **`ctest -R compile_refusal`: 255/255 passed; TUs 255 = CTests 255.**
  - `api_doc_tool.py` self-test OK; check-coverage 4,960 entities / 186 headers; check-fresh rc 0; check-examples 591
    quoted lines verbatim, 6 whole-run listings unbroken; check-removability clean. `doc_staleness_audit.py` self-test OK,
    the audit clean (31 live docs). `briefing_status.py` self-test OK; generated from the FRESH binary, then **check:
    current**.
  - CI greps: no `pros/` include outside `hal/pros/`; no `shulib/sim/` include in the core; **`PROBE_`: 0**.
  - **ARM header gate: 226 headers, `arm-none-eabi-g++ … -Werror -mfloat-abi=softfp`: CLEAN.**
  - `src_build_gate.py` self-test and check: re-run after case 28b grew (below).

### Deltas from the brief (MECH-A5), each with its reason
- **A5-D1** robot two's driver file: `// [driver file two]` .. `// [/driver file two]` wraps ONLY the WHICH BUTTON heading
  and the `kControls` statement — the feel (`kFeel`) and the loop (`kLoop`) are hand-written AFTER it (§1.1's draft put
  them inside). G3a's ask (its RC5, D4 accepted 2026-09-29): its generator owns the controls region and never the feel.
  Pinned by `test/robot_two_driver_test.cpp`'s region case (the markers, and `kFeel`/`kLoop` after the closing one).
- **A5-D2** the driver file's header KEEPS the button map in words (the left stick .. Y, the coast, "no presets"):
  the team reads it before the code; it is prose, not compiled. **For G3a:** the words describe the controls, so a
  generated controls region needs a `control words` region beside it (or the words go stale on the first edit made
  in the panel) — G3a's Part 1 decides; nothing here generates them.
- **A5-D3** `DriverFile::bind` checks COVERAGE first — every mechanism the service has is named by a control
  (`autonOnly(...)` included), else a PreconditionError naming the problem and nothing bound. `DriverInputs::bind`
  (A4) checks only the other way (each control's mechanism exists on the service's robot); T4's binder refused "the
  robot declares 'intake', and no control names it" at boot, and T10's four refusals must all hold. (Mutation MA.)
  *(Superseded at the review by A5-D21: the refusal is a `false` with a reason naming the mechanism, not a throw.)*
- **A5-D4** the boot's declared-port rule covers every SMART ROLE, not only the motors: an angle sensor or a piece
  sensor on a drive port of the record / typed table refuses the boot too (new headline "a MECHANISM SENSOR port is a
  DRIVE port in the record/typed table", the sensor named by mechanism and role). A MOTOR role keeps T3's sentences
  byte for byte (Station J's and the worksheet's quotes; the generic wording is L9 Part B's). The build-time
  `mechanismsClearOf` covers every role already (the library's `clearOfDrive`); the boot had to match it, or a record
  could put a drive motor where the table's check never looked. (Test 6x; mutation MD.)
- **A5-D5** `pacerChainProven` gained the service as the brief says AND a sibling, `pacerChainMissing(...) -> ChainGap`,
  that names the FIRST layer an idle tick did not reach (Guard, Service, Watch, NotAttached), so a program's refusal
  says which link is missing (`chainGapWords`) — the brief's single "pacer chain is WRONG" sentence could not tell a
  service left out from one never bound. The programs call `pacerChainMissing`; the bool is the `== None` of it.
  (G1; mutation M28.)
- **A5-D6** `stopAfterProgramError(chassis, present)` also CANCELS each present mechanism's registered claimant (the
  service's slot) after `applySafeState()` — without it the slot re-commands the step it was running at the next tick
  (a pivot back to 12 V after a program error). Each call in its own try. (G2b; mutation MC.)
- **A5-D7** FREED is per ENTRY: `DriverFile::releasedLeftoverAtEntry(service)` compares the service's lifetime count
  (`leftoverClaimsReleased()`) with its value when THIS `run()` began — the service's count alone would say FREED
  forever after one release; T3's flag was per entry. (Mutation MB.)
- **A5-D8** `src/programs/calibrate_program.cpp` UNCHANGED: N9's fallback said "re-point the drop call (one line)", but
  `dropDeclaredMechanismPorts(kMechanisms, …)` keeps its signature over the new type and `kMechanisms` IS the new
  list, so the call and its pinned text did not move. CAL1's facts mapping (N9 proper) is NOT done — CAL1 Part A2a had
  not landed in this branch's base; `declaredMotorRole()` is not built.
- **A5-D9** the src gate: no `src/team` exception existed in step 0 at this base (the brief's T3 described one), so the
  per-robot rule ADDS a robot's-own-team-file exception (`_team_file_robot`: `src/team/<ns>_*`, or the reviewed
  `TEAM_FILE_ROBOT` map — `autons.{hpp,cpp}` → `robot_two`) rather than narrowing one; it admits only that robot's
  MECHANISM lists, never a chassis table or a selection function. Case 28b gained two expectations past the brief's
  nine: robot THREE's own team file naming its own list passes and naming robot TWO's is refused — the mutation that
  widened the exception to every robot's lists (M34) passed all nine planted cases.
- **A5-D10** the claws auton and the claws ROUTE (U3's "a case that registers a claws auton AND a claws route") live in a
  NEW `test/robot_two_claws_example_test.cpp`, not in `test/autons_test.cpp`: guide 16 quotes the auton, and the
  documentation build's verbatim rule reads `test/*example*_test.cpp`. `autons_test.cpp` gained (1b)'s mechanisms-safe
  check and (1c), the `Routine r{ctx, …}` source pin.
- **A5-D11** test 6y (a two-line pneumatic whose start and safe differ per line) added for M26: robot two's one-line
  claw cannot see per-line levels (6s stays green under M26, as the brief predicted), and no boot test had one.
- **A5-D12** robot two's claws in the boot use TWO tags no more: every program prints the mechanism lines with tag
  `MECH` (one block, `mechanismBootLines`), and the auton programs' `ROUTES:` line with their own tag (`CMP`, `ATS`);
  "shulib Drive" registers no routes (it runs no auton) and prints no `ROUTES:` line.
- **A5-D13** the HA register's `**Held numbers**` line is unchanged (HA-181..HA-186 stays): its own rule lets a number
  that lands as a row stay in its range, as A1–A3 left it; the status paragraph's held sentence names HA-181 only.
- **A5-D14** the worksheet's "What to send back" item 3 no longer asks for "the banner's claw sentence" — J.3.2 was
  RETIRED at T4 Part 3 and the item was stale; fixed on the way, in the lines this run rewrote.
- **A5-D15** the changelog entry is a "no API change" entry under 2.5's heading (nothing under `include/shulib/` moved
  but comments) — the format COMP-b's entry used.

### The prose review (MECH-A5) — the greps run FIRST, every hit read *(2026-10-29, 15:40–15:50)*
- **§4.2's A5 grep** `mechanism_table|kClawOpenCommand|driver_controls|HoldControl|ToggleControl|DriverBindings|claw pivot|
  robot_two_driver|Calibrate.*declared` — hits after the fixes: `README.md` 1, `docs/guide/` 22, `docs/cookbook/` 8,
  `docs/vexbuilder-integration.md` 5, `docs/changelog.md` 5, `docs/roadmap.md` 6, `docs/hardware-assumptions.md` 42,
  `src/team/README.md` 2, `test/README.md` 1; internal: the worksheet 28, HANDOFF 13, build-order 15, REMAINING-WORK 1.
  **Fixed on reading:** guide 11's `TEL` release line (a program on the service says it with the `MECH` line — added);
  guide 07's `src/common/` / `src/robots/` lines (the driver file's plumbing, the mechanism lines, the core's words);
  HA-153's claim (`+6` / `+7` → `.motors = {Forward}`, the old form as history); HANDOFF :481 (a FUTURE plan naming
  `driver_controls.hpp` as the macros' binder — now says the file was deleted and names `driver_file.hpp`) and :611
  (annotated). **Stay, with why:** the roadmap's :420/:434 and the HA register's MECH-A3 status line (:163) are dated
  "Now (date)" / per-part history paragraphs (indexed, not rewritten — the DOCS3 rule); build-order :1492/:1501 and
  REMAINING-WORK row 8 are T4's own history (:1492 already annotated "deleted at MECH-A5"); changelog :164 (2.5's A4 entry
  quoting robot two's manifest line — true) and :1371 (2.2's entry — history); `docs/internal/MECH-PANEL-requirements.md`
  :101 (a planning table of 2026-09-28 listing what existed then); the worksheet's remaining hits are its new lines or the
  old forms quoted as "until MECH-A5"; every guide / cookbook / vexbuilder hit names a file that exists or quotes a line
  the code prints today (read against `test/driver_panel_claws_test.cpp`'s pins and `mechanism_routes_test.cpp`'s
  manifest).
- **Second grep** `driverHeading|ctx\.chassis|kMechanisms` (public docs): guide 3 (18 :389 "the old way" — true; 16
  :281/:330 — this run's), changelog 1 (2.5's A4 entry on the frozen constructor — true), roadmap 1 (this run's).
- **Forward references** (`MECH-A5|fifth part` with yet / will / next / "from … on" / "adds it" / "moves onto"), public
  docs + `include/shulib` + `src/team/README.md`: **fixed** — guide 10 (two), guide 14 (two), guide 15, guide 18 (two),
  cookbook 06 and 07, `docs/vexbuilder-integration.md` §4.2.3's status ("no program registers them yet" → registered by
  the competition program and the Auton Tester, written to no manifest), HA-182's entry, and four header comments
  (`wiring.hpp`'s `startingValuesLine` `///` — the API reference regenerated —, `kinds.hpp`, `service.hpp`,
  `route_ids.hpp`). **Stay:** roadmap :391 and the HA register's :163 (dated history paragraphs, as above).
- **Read whole, as a student would:** guide 16's new section and its listing against the example test; guide 17's boot
  paragraph and its `text` block against the pinned lines; the worksheet's J.3.1, J.9 and J.9.8; the HA-185 entry; the
  changelog entry (corrected on reading: "shulib Drive" has no guard — its driver loop ticks the service; the chain claim
  now names only the two auton programs).

### Definition of done (MECH-A5) — evidence
- [x] §1.13's file list: the table (declarations in `robot_two`, `mechanismTableFor` a case per robot, the drive check
  over every role by name — n31 ×2), `boot_source` (projection, sensors in the declared-port rule), N6's `driver_file.hpp`
  + robot two's driver file (`[driver file two]` before the feel — A5-D1), `driver_heading.hpp`, `driver_controls.hpp`
  deleted (grep test), `autons.cpp` (`Routine r{ctx, …}`, `using namespace robot_two;` — (1c)), the service + route
  registry in each program (chain, proof naming the gap, boot lines, `ROUTES:`, claw row from the service), the tester
  rig, the gate's per-robot rule (28b), HA-185 + HA-159's text.
- [x] **The claws EXACTLY as today on every input**: M5a — the same 159-tick script through T3's bindings as T4 bound them
  and through the driver file: ports 6/7 (mV, brake() and move counts, stopped, brake mode) and line A (every write)
  identical; T1–T10, T9b (a drop is never a press); mutation ME red.
- [x] **Every T4 Part 2a protection test green, its assertions unchanged** — ONE source pin's search string re-pointed
  in `drive_protection_test.cpp` (A5-D19, for the coordinator's acceptance; this line said "unchanged" beside the
  re-point until the review, R-11): the driver file's `kLoop` is now `inline constexpr DriverLoopConfig kLoop{` at
  namespace scope, where it was a `static constexpr teleop::DriverLoopConfig kLoop{` member; the pin's assertions — no
  protection field in the driver file's `kLoop` — unchanged; the suite green.
- [x] **The src gate PASSES all 7 builds** (check, 15:28 and the final run below); the self-test OK (249 cases, then
  re-run after 28b grew).
- [~] **CAL1's facts mapping (N9)**: NOT done — CAL1 Part A2a landed on main (`e5a69b5`) AFTER this branch's base
  (`4691757`), says "src/ is untouched; robot two moves onto it in Part B"; the brief's fallback holds (Calibrate's drop
  reads the new list; A5-D8). The mapping belongs to CAL1 Part B.
- [x] Docs in the same change; the prose review above; every delta logged (A5-D1..D15); every mutation observed red.

### Notes for the coordinator (MECH-A5)
1. **Files shared with another in-flight chunk:** ONLY `tools/src_build_gate.py` (G3a Part 1's too). A5 changed: the
   header's step-0 paragraph ("…AND A ROBOT'S MECHANISM NAMESPACE, PER ROBOT"); NEW `TEAM_FILE_ROBOT`, `TEAM_DIR_REL`,
   `_NAMESPACE_OPEN`, `_MECHANISM_TABLE_OBJECT`, `_robot_namespaces()`, `_team_file_robot()`,
   `_robot_namespace_problems()`; `_stray_selection_problems(repo, rel, per_robot, namespaces)` (the own-team-file
   exception); `one_chain_problems()` (calls both); self-test case 26's `DRIVE_PLANT` re-pointed; NEW case 28b.
   `test/CMakeLists.txt` NOT touched (the new tests are globbed). Untouched: every CAL1 A2a, VIS1-A1 and IMU1 file,
   `src/robots/chassis_table.hpp`, `src/build_selection.hpp`, `src/programs/calibrate_program.cpp`.
2. **The merge onto main** (this branch is from `4691757`; main has `d8fdb8b` and `e5a69b5` since): expect conflicts in
   `README.md`, `docs/changelog.md` (both entries go at the top of 2.5 — A5's is "no API change"), `docs/roadmap.md`,
   `docs/internal/{build-order,REMAINING-WORK,HANDOFF-remaining-work}.md`; REGENERATE after it: `docs/api/`
   (`api_doc_tool.py generate` — `README.md` / `all-entities.md` differ) and the briefing (from a fresh binary). The
   suite's count changes with CAL1's cases.
3. **N9 (CAL1's facts mapping) is CAL1 Part B's now** — A2a "left src/ untouched; robot two moves onto it in Part B".
   A5 left `calibrate_program.cpp` unchanged: `dropDeclaredMechanismPorts(kMechanisms, …)` reads the new list (A5-D8;
   `test/mechanism_table_test.cpp` pins the call site unchanged). Part B maps `kMechanisms` (a `mechanism::MechanismTable`;
   `mechanism::declaredMotorPorts()` is the library's helper) into `CalibrateFacts` and deletes the drop.
4. **For G3a:** (a) a mechanism NAME is at most **18** characters for a Power / motor kind (its `<name>_stop` route id
   must fit F8's 23) and shorter for a pneumatic (`<name>_<longest position>` ≤ 23) — `mechanisms(...)` refuses a longer
   one at build by name; the panel's name field needs that rule (6t pins the 18). (b) `// [driver file two]` wraps ONLY
   the WHICH BUTTON heading and `kControls` (A5-D1); the header's button words are KEPT (A5-D2) — G3a needs a
   `control words` region or the words go stale. (c) `// [robot two mechanisms]` is one contiguous block, byte for byte
   `test/mechanism_fixtures.hpp`'s (pinned).
5. **NOT fixed here — the latent COMP-b line gap A4 handed on:** `src/programs/competition_run.hpp`'s routine-stop line
   is still **214 bytes** with its NUL in its worst case (re-computed from the format: 10-digit request and counts at
   `INT_MAX`, a 24-character step name, "mechanism step FAILED", a 78-character faults text) against
   `kCompetitionLineBytes` 200 — CUT (the record's `cut` flag set), never overflowed; a
   realistic line is ~163. A fix changes words `test/competition_run_test.cpp` and Station K quote; left for the
   coordinator to brief or rule.
6. **The register:** HA-185 registered (179 entries); HA-159 amended; the next free number is unchanged (HA-216).
   The worksheet gained **J.9.8** (HA-185: a phone's slow-motion video of the claw, full and low air).
7. **G4 held:** no auton in `src/team/autons.cpp` steps a claw; the claws auton guide 16 teaches lives in
   `test/robot_two_claws_example_test.cpp` (a host test), so the team writes its own.
8. **Scripts** (`.claude/coordinator/scratch/mech-a5/`): `build.sh`, `mutate.py`, `mutate_multi.py` (a mutation that
   changes a quoted line changes its quote too), `mut_run.sh`, `batch1.sh`, `gate_case_runner.py` (one self-test case
   against the gate on disk); every mutation's log under `mut/`.

### The src gate, final *(`date` at this entry: Thu Oct 29 04:00:53 PM EDT 2026)*
- **`src_build_gate.py self-test`: OK — 251 detector cases, each against a real build** (13 m 51 s; 249 before case
  28b gained robot three's two expectations). **`check`: PASS — all 7 builds link** (bench, xdrive, tank, tank-drive,
  tank-calibrate, tank-autontest, tank-competition: 11/11 TUs each, 8 vendor-ignored warnings, 0 disallowed; 2 m 32 s).
  Every plant restored (`git status` lists only this run's files). The fast doc gates re-run after the last doc edits
  (`docs/vexbuilder-integration.md` §4.2.3's status, HA-185's index row): coverage, fresh, examples (591 lines; 6
  whole-run listings unbroken), removability, the staleness audit and the briefing check — all clean / current.
- **MECH-A5 BUILT — waiting on its review.** Not committed (the coordinator commits).

### MECH-A5 review findings and their disposition *(started — `date`: Thu Oct 29 06:03:32 PM EDT 2026)*
- Three reviewers (correctness, tests, docs) returned 19 findings; each re-checked against this worktree before any
  fix. The dispositions follow, one per finding (R-1 .. R-19 in the order the coordinator listed them), then the gates.

**R-1 (correctness, minor) — the gate's per-robot namespace rule missed four valid spellings.** REAL: confirmed by
reading `_robot_namespace_problems` (one physical line at a time, three spellings) and by mutation GN1 below — the old
rule restored passes exactly the reviewer's four shapes (an alias, a `using namespace` split over two lines, a re-open
with its brace on the next line, a qualified name split before its `::`) and a `#define` of the name. **Fixed at
the root:** `_robot_namespace_problems` no longer lists spellings — the rule is the namespace's NAME as a whole C++
identifier (`\b<ns>\b`, comments and literals stripped, static_asserts blanked, per logical line: an identifier
cannot span a line). Every way to reach a namespace spells its name, so the alias, the split forms, a `#define`, an
attribute or `inline` on a re-open are all refused by the one rule. The header paragraph, the function's docstring,
`src/robots/mechanism_table.hpp`'s banner and `src/team/README.md` say so. Case 28b gained eight plants: REFUSED — an
alias, a split `using namespace`, a fully qualified `using namespace shulib::bench::robot_two;` at global scope, an
Allman re-open in `src/common/`, a qualified name split before `::`, a `#define` of the name, a `src/common/` file
NAMED like robot two's team files (R-5); PASSING — identifiers that only BEGIN with the name (`robot_twofold`,
`robot_two_count`). The real tree still passes (`one_chain_problems() == []`, the case's last expectation). The
alternative the reviewer suggested — whole-file matching plus an alias pattern — was rejected: it is still a list of
spellings (it misses a `#define`, `namespace [[deprecated]] robot_two {`, `namespace a::inline robot_two {`). Its cost:
an unrelated identifier spelled exactly like a robot's namespace is refused too — none exists in `src/` or
`include/shulib/`, and a name equal to a robot's namespace is one a program should not use. (A5-D20.)
*(Overclaimed — the verification round's VR-1, below: "every way … spells its name" missed a splice inside the
name, a digit separator and a raw string on the name's line, and a name pasted together with `##`.)*

**R-2 (correctness, minor) — a refused boot destroyed the Robot before the mechanism service.** REAL, confirmed by
reading: `~MechanismService` (`service.hpp` :1595-1611) calls `holdsClaim()` on every slot, which reads the slot's
`hal::MotorMechanism` / `PneumaticMechanism` — objects inside the Robot's `MechanismParts` — and the service was bound
with the Robot's fault latch and clock (`bind()`'s rule: they "must outlive the service"). Both unbuild lambdas
(`competition_program.cpp`, `auton_tester_program.cpp`) reset the Robot first. **Fixed:** the service is reset BEFORE
the Robot; the Robot's, the guard's and the watch's destructors are defaulted (read nothing), so the guard naming the
released service as its inner pacer for one statement is harmless (nothing paces during an unbuild). The two comments
now say what each destructor reads (the old "the service's own end touches no device" was true but beside the point).
`test/competition_source_test.cpp`'s unbuild pin PINNED THE WRONG ORDER — re-pointed, with the reason; the Auton
Tester had no unbuild pin — one added to `auton_tester_source_test`'s chain case. "shulib Drive" was already right
(`refuseBuilt()` resets the service first). Undefined behaviour no host test can observe (the static `optional`'s
storage stays), so the pins are the guard; mutations FRU1/FRU2 below.

**R-3 (correctness, minor) — "shulib Drive"'s driver-file refusal lost its headline and its mechanism's name.** REAL:
T4's binder refused with its own headline and a reason naming the mechanism; A5's `DriverFile::bind` threw a
PreconditionError the boot's catch showed as "the library graph REFUSED to build" with a generic sentence — and Station
F's table promised the old headline. **Fixed at the root (T4's contract restored):** `DriverFile::bind(service, why,
cap)` returns `bool`; FALSE with the reason in `why` NAMING THE MECHANISM, and nothing bound, in T4's sentences — "a
control names '<name>', which the robot does not declare", "'<name>' is a <kind> mechanism on this robot: its control
is for a <kind> one", "the robot declares '<name>', and no control names it" — and the library's own checks
(`DriverInputs::bind`: a preset, position, speed, units or homing that is not this robot's) caught and reported in
the library's sentence. A PROGRAM's mistake (a second bind; a service not bound yet) stays a PreconditionError — the
graph headline is right for those. The programs refuse with their T4 headlines again: "shulib Drive" `the driver file
REFUSED this robot` (Station F's row, now true — amended to name `driver_file.hpp` and the library-sentence case), the
competition program `the driver's controls REFUSED` (its T4 words; no document quotes them). Tests: T8 pins both
program-mistake preconditions by sentence; T10 pins all five refusals by their exact reasons (a NEW three-position-claw
case reaches the library's check), `CHECK_NOTHROW`, nothing bound, and a refusal with no buffer; the two source pins
re-pointed (`refuseBuilt(` counts back to T4's 4 in each program). The duplication this adds (steps 1's two checks
repeat two of `DriverInputs::bind`'s preconditions) is deliberate and bounded: the library's preconditions stay the
authority (step 3 still runs them), the driver file's are the team-facing words. (A5-D21, superseding A5-D3's "a
PreconditionError naming the problem".)

**R-4 (correctness, minor) — each claw press in driver control now prints a MECH verdict line.** REAL — confirmed
and pinned by a NEW test (`robot_two_driver_test`, "MECH-A5 review: in driver control each A press is a claw STEP that
logs its verdict"): through robot two's real driver file on the fake provider (10 ms a tick), A at ticks 10, 100, 200,
203 and the run ending at 230 log exactly `[INFO] 'claw': SUCCEEDED -- time in 0.50s`, `[WARN] 'claw': UNCONFIRMED --
time in 0.50s (no piece sensor to confirm it; counted done)`, `[WARN] 'claw': CANCELLED in 0.03s (replaced by a newer
command)`, `[WARN] 'claw': CANCELLED in 0.26s (driver control ended)` — and R1 held for 40 ticks logs nothing (a
hold-to-run is not a step). **Disposition: documented and pinned, not silenced.** A step's verdict line is A4's
library rule for every step, and a driver's toggle / preset / homing press IS a step by A4's design (one service, one
path); silencing driver presses is a library behaviour change outside A5's row. The lines are now expected in the
docs: guide 18's `toggle` bullet, guide 11's verdict entry, worksheet J.9.3 (the line after each press, "ONE line per
press?"), J.9.4 (one line while held; two quick presses → `CANCELLED (replaced …)`), K.2.5 (one per press of A, none
for the pivots, in serial and `comp_log.txt`). **For the team lead** (coordinator note 10): whether a match's driver
presses should log — each close is a `[WARN]` line on serial and the card. (A5-D22.)

**R-5 (tests, minor) — 28b missed two edges (G10, G21 survived).** REAL. Fixed with R-1: the `src/common/
robot_two_helpers.hpp` plant (a file named like robot two's team files outside `src/team/`) and the fully qualified
global `using namespace` plant. G10 re-run: **RED**. G21 as written no longer applies — the rule has no
qualified-`using` alternative to drop; its plant stands, and GN2 (the rule narrowed to qualified names only) is its
analogue: **RED**.

**R-6 (tests, minor) — the competition program's route path had no behavioural test (R22 survived).** REAL. NEW
`competition_run_test` **(R1r)**: the claws route (the example test's: `claw_open` at the start; `claw_closed`,
`claw_pivot_6_run` 12/400, `claw_pivot_7_run` −12/400 at the end of a 20 in drive) registered as a match auton through
the stack's `extraAutons`, chosen through the real menu, served through `serveAutonRequest`: `FINISHED`, `RouteRan`,
4 markers fired, 0 unknown, line A commanded open during the route, pivot 6 at +12 V and pivot 7 at −12 V, every
mechanism and the drive safe at the end. The rig's `ProbeLayer` gained an `onPace` hook (a case's own look at the world
each pace). R22 and R3 (`runAuton` handed `nullptr` for the registry) re-run: **both RED behaviourally** (R3 also red
on P5's pin).

**R-7 (tests, minor) — "each call in its own try" untested with a throwing safe state AND a claimant (R1 survived).**
REAL. G2b gained a `ThrowingMechanism` (its `applySafeState()` throws) holding a `Counting` claimant, first in the
list: its safe state tried once, its claimant STILL cancelled once, its claim released. R1 re-run: **RED**.

**R-8 (tests, minor) — the boot's rule for motor roles beyond the first untested (R2 survived).** REAL. 6x gained robot
y's dr4b (motors 3 and 4) with its SECOND motor on the record's drive port 13: refused, the CLAW headline, `port 13 is
both a claw (dr4b) and a drive motor in the record -- recalibrate: …`, nothing built. R2 re-run: **RED**.

**R-9 (tests, minor) — the drive check never tested on the LAST left port (R21 survived).** REAL. THE DRIVE CHECK gained
the left side's LAST entry (−15) and the right side's FIRST (+20) (the right's last, +16, and the left's first, −11,
were there). R21 re-run: **RED**.

**R-10 (tests, minor) — the chain proof's "first gap in that order" unpinned with two gaps (R11 survived).** REAL. G1
gained a guard paced over the PLANT (neither the service nor the watch in its chain): `ChainGap::Service`, never
`Watch`. R11 re-run: **RED**.

**R-11 (tests, minor) — one T4 Part 2 source pin's search string changed.** Re-checked: true, and necessary —
`git diff 4691757 -- test/drive_protection_test.cpp` is that one search string (`static constexpr
teleop::DriverLoopConfig kLoop{` → `inline constexpr DriverLoopConfig kLoop{`): N6's driver file states `kLoop` at
namespace scope (DriverFile takes it as a template argument), so the old text cannot exist; the pin's assertions (no
protection field in the driver file's `kLoop`) are unchanged. **For the coordinator's explicit acceptance** — logged as
A5-D19; the DoD line reworded so it no longer says "unchanged" beside the re-point.

**R-12 (docs, major) — status pointers stale.** REAL. Rewritten: `build-order.md`'s first bold `Next:` (its "NEXT:
MECH-A5 (CAL1 before it) … then A5–A6" clause → A5 built and in review, then A6; G3a Part 1 beside A5 under the PANEL
SPRINT; CAL1 maps the core's list after A5 — N9's fallback), its MECH entry's `Next run:` line and A5 paragraph,
`REMAINING-WORK.md`'s "Where things stand" and row 12, `roadmap.md`'s "Now (2026-10-29, later)" paragraph (now names
the next step), `HANDOFF-remaining-work.md` :288/:291. The merge with main's `d8fdb8b` (the sprint's own wording) still
has to reconcile these lines (note 2). The briefing regenerated from a fresh binary.

**R-13 (docs, major) — the prose review missed sentences this change made false.** REAL, all eight — fixed: (1)
`docs/faq.md` ("no program builds the service yet" → robot two's three programs build it since the fifth part); (2)
HA-161's quote of `drive_program.cpp` ("the bindings by address" → "the inputs", the old word as history); (3) HA-161's
claim rule restated (the service's slots DO take a claim; every mode change ends each slot's behaviour and releases its
claim, and releases and says a claim another owner left; T3's bindings took none — history); (4) HA-157's Source (the
service's `leaveDriverControl()` and `teleop_loop.hpp`; T3's ToggleBinding as history; `robot_two_driver_test` M5a pins
robot two's claw); (5) HANDOFF's re-zero paragraph (DriverFile's `static_assert` — n57 — and the service overload's
`DriverInputs::avoids()`; `validBindings()` for the superseded overloads); (6) `teleop_loop.hpp` — its callers
(robot two's driver file through `DriverFile::run`, the service overload) and `DriverLoopConfig`'s `///` (both
preconditions; `docs/api/teleop_loop.md` regenerated); (7) guide 14's present-tense T3 sentence → past; (8)
`src/team/README.md` and cookbook 06 ("the call into the loop" is `driver_file.hpp`'s). Also found: guide 14's "four of
its six parts" → five; HANDOFF :288's "no program builds one yet" → "until MECH-A5". The widened greps are logged in
the prose-review addendum below.

**R-14 (docs, minor) — `src/team/autons.cpp` misstated a claw step's verdict.** REAL — reworded: opening SUCCEEDS once
its move time has passed; a GRAB (`closed`, marked `grabs`) with no sensor ends UNCONFIRMED and is counted done.

**R-15 (docs, minor) — guides 16/17 showed no route auton with mechanism markers; the changelog said guide 16 did.**
REAL. Guide 16 now QUOTES the claws route's waypoint and marker lines from `test/robot_two_claws_example_test.cpp`
(held verbatim by the example scan — 600 quoted lines now, was 591), with its verdicts; guide 17's route bullet points
at it (and says the host suite runs one through the competition program's own request path — R1r). The changelog line
is true as written.

**R-16 (docs, minor) — deltas missing from the A5-D list.** REAL — A5-D16..D19 added below (the declaration line; the
ROUTES line's text; the tester rig's list; the drive-protection pin), and this round's A5-D20..D22.

**R-17 (docs, minor) — nothing handed L9 Part B the rule A5 rewrote.** REAL — coordinator note 9 added below.

**R-18 (docs, minor) — no landing-date list.** REAL — the list is below (A4's D13 form).

**R-19 (docs, minor) — two comments overclaimed.** REAL. `wiring.hpp`'s `startingValuesLine` `///` → "a program prints
it at boot for each position or speed mechanism of its list, after that mechanism's role and control lines" (robot two
declares none, so no boot prints one; `docs/api/wiring.md` regenerated); `mechanism_table.hpp`'s banner → "mistakes the
mechanism core refuses at build, each by name (GCC names your line on the error or in its first note — guide 18,
'reading a refusal')" (the brief's T5 retraction; no count).

**Deltas from the brief added by the review round** (numbered on from A5-D15):
- **A5-D16** every mechanism's boot block begins with a DECLARATION line (`driver_panel.hpp`
  `mechanismDeclarationLine()`: its kind, cartridge, each motor's direction and its stop — or a pneumatic's start,
  safe and each position's levels) beyond §1.13's role, control, homed, no-stall and starting-values lines. Reason: a
  role line names a port, not what is on it; J.9's polarity station needs the claw's levels AS DECLARED and J.4 a
  pivot's direction, and without the line a student must open the robot file. Rejected: the brief's list only.
  Quoted by the worksheet (J.3.1's four per pivot, J.9.1, M.2.2) and guides 16/17; pinned by `driver_panel_claws_test`.
- **A5-D17** the `ROUTES:` line reads `(claw_pivot_6_run .. claw_open); 26 of the registry's 32 left for the team's
  own` where §1.13 wrote `…` and no tail: `driver_panel.hpp`'s own rule is every line ASCII (:62, :101 — a `…` is three
  bytes of UTF-8), and the tail is the number a team adding its own commands (and G3a's panel) needs. Rejected: the
  brief's text verbatim.
- **A5-D18** the tester rig (`test/auton_tester_rig.hpp`) binds ROBOT TWO's list, not "the build's list over the §1.12
  models" (§1.13 / U3): on the host the build's list is `kNoMechanisms` (the bench selection), so a rig over it runs
  every team auton with no mechanism at all — the team's autons are robot two's (`TEAM_FILE_ROBOT`), and a claw step
  must meet robot two's declarations; robot two has no arm, lift or flywheel for the §1.12 models to stand for. Said
  in the rig's banner (:11-18).
- **A5-D19** `test/drive_protection_test.cpp` :2480's search string re-pointed (R-11) — the one change to a T4 Part 2a
  protection test; assertions unchanged; for the coordinator's acceptance.
- **A5-D20** (review) the src gate's per-robot rule is the namespace's NAME as an identifier, not three spellings (R-1).
  *(Verification round, VR-1: its "in any spelling" / "misses none" overclaimed — step 0 now reads the text as C++'s
  phases 2–3 leave it (`_splice`: line splices joined with NOTHING; numbers whole; raw strings whole), and its documents
  state the real limit: a name the preprocessor ASSEMBLES with `##` is beyond any text rule.)*
- **A5-D21** (review) `DriverFile::bind` returns a refusal NAMING the mechanism (`bool` + `why`) and the programs' T4
  headlines are back (R-3); supersedes A5-D3's PreconditionError for a robot the controls are not for.
- **A5-D22** (review) a driver's toggle press logs its step verdict — documented and pinned, not silenced (R-4).

**The review round's mutations — each observed RED, none caught by the cap or the timeout.** A C++ mutation: ONE
exact replacement through `scratch/mech-a5/mutate.py` (the file backed up under `scratch/mech-a5/mut/`, restored in a
`finally`, identical to its backup — checked — and its timestamp bumped), then `mut_run.sh`: the stale binary removed,
the WHOLE `cmake --build build/test` (every object depending on the mutated file recompiled — the count below), then
the named cases from THAT binary, capped (`ulimit -v 4194304; timeout 600`). A source pin reads the file on disk at run
time, so a program file the host build never compiles (0–3 objects) is judged by the pin on the mutant. A gate
mutation: case 28b's own lines against the mutated gate module (`gate_case_runner.py`), every plant restored. Script
`scratch/mech-a5/fix/{batch_fix.py,gate_mut.py}`; logs `scratch/mech-a5/mut/FR*.out`, `scratch/mech-a5/fix/*.out`.

| # | Mutation (file; objects rebuilt) | Caught by (observed) |
|---|---|---|
| GN1 | the namespace rule back to three spellings matched per line (`tools/src_build_gate.py`) | **28b RED** (5 of 24): the alias, the split `using`, the Allman re-open, the split qualified name, the `#define` |
| GN2 | the rule narrowed to qualified names `<ns>::` (G21's analogue) | **28b RED** (10 of 24) |
| GN3 | the trailing `\b` dropped (a prefix, not the name) | **28b RED** (1): the PASS control (`robot_twofold`, `robot_two_count`) reported |
| G10 | `_team_file_robot`'s `src/team/` check dropped | **28b RED** (1): `src/common/robot_two_helpers.hpp` admitted |
| FR1 | the program-error stop's safe state and claimant cancel in ONE try (`src/common/guarded_run.hpp`; 9) | **G2b RED**: `safeThrowsClaimant.cancels == 1` and the claim released both fail |
| FR2 | the boot's rule reads each mechanism's FIRST motor only (`src/common/boot_source.hpp`; 16) | **6x RED** (4): the dr4b's second motor on port 13 booted, no headline, 7 mechanisms built |
| FR21 | the drive check's left loop stops one short (`src/robots/mechanism_table.hpp`; 17) | **THE DRIVE CHECK RED**: `onLeftLast` (−15) clear |
| FR11 | the chain proof's service and watch checks swapped (`src/common/guarded_run.hpp`; 18) | **G1 RED**: the two-gap chain named `Watch` |
| FR22 | the competition rig's `routes.registerInto(commands)` → `(void)commands` (`test/competition_rig.hpp`; 9) | **R1r RED** (5): 0 markers fired, ids unknown, the claw never opened, neither pivot moved |
| FR3 | `serveAutonRequest` hands `runAuton` `nullptr` for the registry (`src/programs/competition_run.hpp`; 3) | **R1r RED** (not `FINISHED`, not `RouteRan`) and **P5 RED** (the hand-off pin) |
| FRU1 | the competition unbuild resets the Robot before the service (`competition_program.cpp`; 3) | **pins (P2) RED** (the unbuild pin) |
| FRU2 | the same in the Auton Tester (`auton_tester_program.cpp`; 0 — not host-compiled) | **the tester's chain pin RED** (its NEW unbuild pin) |
| FRD1 | `DriverFile::bind`'s kind check removed (`src/common/driver_file.hpp`; 2) | **T10 RED**: 'claw pivot 7 a pneumatic' reached the library's check — its reason no longer names the mechanism |
| FRD2 | `DriverFile::bind`'s try around the library's checks removed (2) | **T10 RED** (3): the three-position claw THREW (`CHECK_NOTHROW`), no `false`, no reason |
| FRD3 | "shulib Drive" refuses a driver file with the library-graph headline (`drive_program.cpp`; 2) | **P3-T3 RED** (the boot pin) |
| FRD4 | the coverage refusal's reason no longer names the mechanism (2) | **T10 RED**: 'an extra intake' — `the robot declares a mechanism no control names` |
| FRL1 | every step verdict logged at `Info` (`include/shulib/mechanism/service.hpp`; 51) | **the claw-press log case RED** (3 of 4 lines: `[INFO]` where `[WARN]`) |

**The prose review, widened (R-13), the greps run FIRST and every hit read** *(2026-10-29, the fix round)*:
- `no program (builds|runs|binds|registers|prints)|bindings by address|take(s) no claim|call into the … loop` over the
  public docs, `src/`, `include/`, `test/README.md` and the live internal docs: changelog 3 (A3's entry and two older
  "no program runs it yet" — history, dated), cookbook 06 2 (the reworded "Use this when" and the "Why it is written"
  bullet — true), `driver_file.hpp` 1 (true), `src/team/README.md` 2 (reworded), roadmap 1 and the HA register 4 (dated
  per-chunk history: HA-117's, R5a's, a 2.x entry — and HA-161's reworded rule), `mechanism_binding.hpp` 2 (T3's own
  bindings — true of them), HANDOFF 1 and build-order 1 (R5a's history, reworded :288), `session_info.hpp` 1 and
  `service.hpp` 1 (other subjects — true).
- `validBindings|ToggleBinding|MechanismBinding|claw bindings|driver_controls|teleop_claws`: guide 10 4 and cookbook 06
  10 (the superseded bindings' own sections, marked "Superseded, not removed" — true), guide 14 2 (now past tense),
  changelog 10 (history), roadmap 3 and REMAINING-WORK 1 and HANDOFF 3 (annotated history; HANDOFF's re-zero
  paragraph rewritten), the HA register 7 (HA-147's and HA-156's "until MECH-A5" history, HA-157/HA-161 rewritten),
  `teleop_loop.hpp` 24 (its T3 overloads — true; its callers and the re-zero `///` rewritten), `src/team/README.md` 1
  (history).
- `REFUSED this robot|controls REFUSED|library graph REFUSED|DriverFile::bind|PreconditionError naming`: the two
  programs' refusals (each read), `driver_file.hpp`'s own sentences, the worksheet's F table (row rewritten, the
  library-graph row true), `drive_program.cpp`'s ORDER AT BOOT comment (corrected: the driver file's mismatch is its
  own refusal, not a precondition), `autons.hpp` :32 (another subject).
- `UNCONFIRMED|counted done` in guides 11/16/18, the worksheet and `autons.cpp`: every hit now says a SENSORLESS GRAB
  is the counted-done case (R-14's comment fixed; the others were already right).
- `four of its six|unbuild|using namespace robot_two|re-open`: guide 14's count fixed; the unbuild comments are the two
  rewritten; the namespace hits are the gate's, the banner's and the README's rewritten sentences.

**The landing dates, for the commit (A4's D13 form — "(<landing>; built 2026-10-29)", or the landing date where a
document dates A5 as IN):** `docs/changelog.md` :19 (the entry's heading date); `docs/guide/18-mechanisms.md` :12 ("A4
and A5, 2026-10-29"); `docs/guide/16-testing-an-auton-on-the-bench.md` :38; `docs/guide/14-what-it-cannot-do-yet.md`
:219 ("2026-10-15 to 2026-10-29, five of its six parts"), :227 and :434; `docs/guide/10-the-api.md` :416;
`docs/cookbook/06-driver-control.md` :27; `docs/cookbook/07-mechanisms.md` :8; `docs/vexbuilder-integration.md` :371;
`src/team/README.md` :10; `docs/hardware-assumptions.md` :170 (the status paragraph: "built 2026-10-29" → add the
landing); `docs/roadmap.md` the "Now (2026-10-29, later)" paragraph and the MECH row (:1780, "A5 BUILT … landing waits
on its review" → LANDED); `build-order.md`'s first bold `Next:` and the MECH entry's A5 paragraph and `Next run:`;
`REMAINING-WORK.md` "Where things stand" and row 12; `HANDOFF-remaining-work.md` :291. **Left as the BUILD date on
purpose (A3's rule):** the register's "since MECH-A5, 2026-10-29" source notes (HA-182 :1297, HA-185's section heading
:1325, HA-147 :2067, HA-159 :2333, HA-161 :2394), the worksheet's "since MECH-A5, 2026-10-29" station notes (they name the
build a station tests), and the code banners naming the run by its date (`driver_file.hpp` :4, `driver_heading.hpp`
:5, `driver_panel.hpp` :9/:98, `guarded_run.hpp` :15/:20, `competition_run.hpp` :65, `auton_tester_run.hpp` :31,
`drive_program.cpp` :62, `mechanism_table.hpp` :4, `robot_two_driver.hpp` :6, `tools/src_build_gate.py` :225, the
test banners).

**Notes for the coordinator, added by the review round:**
9. **L9 Part B — carry A5's rewritten rule.** A5 rewrote the boot's declared-port rule (`src/common/boot_source.hpp`,
   step 5) over every SMART ROLE: a MOTOR role keeps T3's words byte for byte ("port N is both a claw (<name>) and a
   drive motor in the record -- recalibrate: …", headline "a CLAW port is a DRIVE port in the record / typed table"),
   a SENSOR role (an angle sensor, a distance or optical piece sensor) has new words ("port N is both <name>'s <role>
   and a drive motor in the record -- check its wire(...) line in src/robots/mechanism_table.hpp; then recalibrate
   (slot 4)", headline "a MECHANISM SENSOR port is a DRIVE port in the record / typed table"). `mechanismTableProblem`,
   both old `toMechanismConfig`s and their two `static_assert`s are GONE from `src/`. L9's brief (:433-440) still plans
   a motor-only `DeclaredMotorPorts` and says `mechanismTableProblem` stays in `src/` — both stale. The MECH brief's
   §6.1 ask ("that derivation must include A1's sensor ports") is met at the collision rule (step 5 reads every smart
   role); the port check's declared list stays the MOTOR ports (`mechanism::declaredMotorPorts()` — a sensor is never a
   NEW motor in the census). L9 Part B's `BootWording` must carry both headlines and both sentence shapes, and its
   re-check should re-read step 5 at its tip.
10. **A ruling the team lead may want (R-4):** since A5, each driver press of robot two's claw logs its step verdict
   0.5 s later — a close is a `[WARN][MECH] … UNCONFIRMED … counted done` line, on serial and in `comp_log.txt` during
   a match. It is A4's library rule (every step says its verdict) and is now documented (guides 11/18, J.9.3/J.9.4/
   K.2.5) and pinned; silencing driver presses would be a library change (`driver_inputs.hpp` / `service.hpp`), not
   made here.
11. **The unbuild order (R-2)** was a real lifetime defect in the competition program and the Auton Tester (UB on a
   refused boot after the bind; benign in practice); fixed and pinned in both. The `DriverFile::bind` signature changed
   (R-3: `bool bind(service, why, cap)`): a driver FILE never calls it — the programs do (`if (!st.driver.bind(
   *st.mechanisms, why, sizeof why)) { refuseBuilt(<the program's headline>, why); return; }`), so G3a Part 5's
   generated controls region does not move; G3a Part 4 (which edits the three programs and adds `DriverFor<RobotId>`)
   builds on this form at A5's tip.

**The gates, final (the restored tree; `git status` lists only this run's files)** *(`date` at this entry: Thu Oct 29
07:05 PM EDT 2026)*:
- `build.sh final2` (the stale binary removed, the briefing regenerated as unbuilt, the whole `cmake --build
  build/test`, 51 objects): rc 0. **Suite (`tools/suite_run.py green`, capped 4 GiB / 600 s): green — 2,624 cases,
  2,800,491 assertions** (2,622 before the round: + the claw-press log case, + R1r). The briefing regenerated from that
  fresh binary, then **`briefing_status.py check`: current**; its self-test OK.
- **`ctest -R compile_refusal`: 255/255 passed** (TUs 255 = CTests 255).
- `api_doc_tool.py` self-test OK; check-coverage 4,960 entities / 186 headers; **check-fresh rc 0** (`docs/api/`
  regenerated for the two `///` edits: `teleop_loop.md`, `wiring.md`); check-examples **600 quoted lines** verbatim (10
  source files; 6 whole-run listings unbroken); check-removability clean. `doc_staleness_audit.py` self-test OK, the
  audit clean (31 live docs) — after one fix on the way: HA-161's re-wrapped sentence put the deleted
  `driver_controls.hpp` on a line without its "deleted at" note (the audit's dead-path rule caught it; re-joined).
- CI greps: no `pros/` include outside `hal/pros/`; no `shulib/sim/` include in the core; `PROBE_`: 0.
- **ARM header gate: 226 headers, `arm-none-eabi-g++ -std=gnu++20 … -Wshadow -Werror -Os -mcpu=cortex-a9
  -mfpu=neon-fp16 -mfloat-abi=softfp`: CLEAN.**
- **`src_build_gate.py self-test`: OK — 259 detector cases, each against a real build** (251 + case 28b's eight new
  plants; ~15 min; every plant restored). **`check`: PASS — all 7 builds link** (bench, xdrive, tank, tank-drive,
  tank-calibrate, tank-autontest, tank-competition: 11/11 TUs each, 8 vendor-ignored warnings, 0 disallowed).
- **MECH-A5's review round DONE — every finding re-checked; 18 fixed at the root, R-11 left for the coordinator's
  acceptance (A5-D19).** Not committed (the coordinator commits).

### MECH-A5 — Verification-round findings and their disposition *(started — `date`: Thu Oct 29 07:44:27 PM EDT 2026)*
- A fresh verifier returned ONE finding (VR-1, minor: R-1's fix claims the gate's namespace rule catches the name "in
  ANY spelling" and "misses none", but a backslash line splice inside the name and macro token pasting both compile
  and pass). Re-checked first, against this worktree; the disposition, the fix and the gates follow when done.

**VR-1 (minor) — R-1's fix claimed the namespace rule sees the name "in ANY spelling" and "misses none"; two
compiling spellings passed.** *(`date` at this entry: Thu Oct 29 08:10:52 PM EDT 2026)* **REAL**, re-checked first:
- **The re-check.** `scratch/mech-a5/vfix/probe.py` plants each spelling under `src/programs/` of a COPY of this
  worktree (`vfix/copy/`) and counts what the namespace rule and the stray-selection rule report. Before the fix: the
  splice `robot_\` + `two::claw` 0 / 0 and `CAT(robot_, two)` 0 / 0 — the verifier's two, confirmed. The same probe
  found THREE MORE of the class, each compiling (`vfix/evasions.cpp`, `g++ -std=gnu++20 -Wall -Wextra`, returning the
  expected 28): a digit separator on each side of the name on ONE line (`int a = 1'0; … robot_two::claw; … 2'0` — the
  character-literal pattern paired the two apostrophes and blanked the code between them as a literal's text, 0 / 0;
  the same with `0xF'F` / `0b1'0`), and a raw string holding a quote before the name on its line (`R"(")"`, with and
  without a delimiter — the plain-literal pattern ended at the inner quote and the next `"` opened a literal that
  swallowed the name, 0 / 0). Robot two's LIST split by a splice (`kRobotTwo\` + `Mechanisms`) passed the
  per-robot table rule too (the same reading). The other way: a `//` comment ending in a backslash was REPORTED,
  though C++ continues the comment over the next line. Not gaps (each probed): a UCN inside the name
  (`robot_\u0074wo` and its `\U` / `\u{}` forms — g++ and arm-none-eabi-g++ both refuse "universal character … is
  not valid in an identifier"), a trigraph `??/` (ignored in gnu++20 — the split name does not compile), a digraph
  `%:define R2 robot_two` (the name is written out — refused).
- **The root cause.** Step 0 read the text in a different ORDER and with a coarser lexer than C++: comments and
  literals stripped BEFORE splices were joined, the joins a SPACE, no pp-numbers, no raw strings. Every step-0 rule
  reads through the same two helpers (build switches, chain constants, per-robot tables, the namespace rule, the
  include root, the printed paths), so the fix is the READING, not the namespace rule.
- **Fixed at the root (`tools/src_build_gate.py`).** (1) `_splice` (new) is C++'s translation phase 2: each line
  splice — a backslash ending a line, blanks after it allowed as GCC allows them — deleted with NOTHING between, each
  deleted line break put back after its logical line ends so every line keeps its number. (2) `_strip_comments`
  splices FIRST (C++'s order), so a comment continued by a splice is a comment and a literal continued by one is one
  literal. (3) `_logical_lines` joins by `_splice` — with nothing (the finding's step 1), for any text handed to it.
  (4) `_CXX_TOKENS` gained C++'s pp-number (a digit separator's `'` opens no literal) and the raw string literal
  (`(u8|u|U|L)?R"delim( … )delim"`), each anchored at a token's start; `_strip_comments` keeps a number as it is and
  blanks a raw string to `""` keeping its line breaks; `_literal_runs` (the printed-path rule) reads a raw string's
  own text. (5) The documents state the REAL limit, in the header (step 0's reading; the per-robot paragraph's "THE
  LIMIT"), `_robot_namespace_problems`' and `_splice`'s docstrings, the 28b comment ("THE NAME, WHEREVER IT IS
  WRITTEN"), `src/robots/mechanism_table.hpp`'s banner and `src/team/README.md`: a name the PREPROCESSOR assembles
  (`##`, or its digraph `%:%:`) is never written out and no text rule sees it; a file step 0 does not read is not
  read; and the one place the reading differs from C++'s — C++ undoes splices inside a raw string's text, `_splice`
  does not, so a splice inside a raw string's closing `)delim"` ends it early here. (6) A5-D20 notes the change;
  R-1's paragraph is annotated "overclaimed — VR-1" (history kept).
- **The real tree, old reading against new** (`vfix/diffread.py`, `vfix/diffcount.py`: every one of the 793 files
  step 0 reads, both modes): every rule still reports NOTHING (`one_chain_problems`, `include_root_problems`,
  `printed_path_problems`: `[]` before and after). 28 files read differently, each now right: 16
  (`test/pros_shim/*`) hold a string literal continued by a splice, and the old reading dropped its line break — every
  later line was reported ONE LINE SHORT (`test/pros_shim/main.h`'s `#endif` is line 14; the old reading said 13);
  `src/build_selection.hpp`'s, `check.hpp`'s and `trace.hpp`'s multi-line directives and the F6/F8/F10 pin macros
  join with no extra blank; `pros_device_provider_test.cpp` and `pros_platform_adapter_test.cpp` use digit
  separators (`advanceUs(2'500'000)` — the old reading had `2''000`, `500` blanked as a "literal"); five tests' raw-string
  regexes (`R"(…)"`) were mis-paired. So the digit-separator gap was LIVE in this tree's reading, in test/ (read by
  the build-switch rule).
- **The tests (adversarial).** Case 28b: seven new plants and two direct checks — 33 expectations (was 24). REFUSED:
  the name split INSIDE it by a splice; the same with blanks after the backslash; the name between two
  digit-separated numbers on one line; the name after a raw string holding a quote; robot two's LIST split by a splice
  (the per-robot table rule). PASSING: a `//` comment a splice runs on over the next line; the name only inside a
  multi-line raw string. Direct: `_logical_lines` joins with nothing on the line the logical line starts; a spliced
  literal shifts no later line and a number keeps its digits. Case 30: a dead path inside a raw string holding a quote
  is reported (it passed the old gate). Token pasting is NOT planted: it is the stated limit — a plant would pin a gap
  as behaviour. Cases 28b and 30 GREEN on the fixed gate (`gate_case_runner.py`: 33 and 7 expectations).
- **Not done, said.** Refusing `##` (and `%:%:`) under `src/` would close the last text gap — a NEW rule on the
  team's code, a decision for the team lead; offered in the notes, not made. A file outside ONE_CHAIN_ROOTS —
  `include/main.h`, PROS's project header, which a team may edit — is read by NO step-0 rule; pre-existing since SRC1,
  every rule alike; noted for the coordinator, not changed here.

**VR-1's mutations — each observed RED on an `expect` (none crashed; none caught by the cap or the timeout).**
`scratch/mech-a5/vfix/gate_mut.py`: each mutation a set of exact replacements in `tools/src_build_gate.py` (backed up
under `vfix/mut/`, restored in a `finally`, checked identical, timestamp bumped; `md5sum -c vfix/gate.fixed.md5` OK
after the run), then case 28b's and case 30's own lines against the mutated module (`gate_case_runner.py`, capped
`ulimit -v 4194304; timeout 600`). Log `vfix/mut/gate_mut.out`.

| # | Mutation | Caught by (observed) |
|---|---|---|
| VM0 | the WHOLE reading back to the pre-verification gate (`_logical_lines`, `_CXX_TOKENS`, `_strip_comments`, `_literal_runs`) | **28b RED** (9 of 33: both direct checks, all five REFUSED plants, both PASS plants over-reported) and **30 RED** (the raw-string path) |
| VM1 | `_splice` joins with a SPACE (the finding's premise) | **28b RED** (4): `_logical_lines`' direct check, the split name, the split name with blanks, the split LIST |
| VM2 | `_strip_comments` does not splice first | **28b RED** (2): the spliced literal shifts the later lines; the continued `//` comment reported |
| VM3 | `_logical_lines` does not splice | **28b RED** (1): its direct check |
| VM4 | no pp-number (a digit separator opens a literal again) | **28b RED** (2): the digit-separator plant; the number lost its digits |
| VM5 | no raw string | **28b RED** (2): the quote-holding raw string; the multi-line raw string over-reported — and **30 RED** |
| VM6 | `_literal_runs` ignores a raw string | **30 RED** (the raw-string path) |
| VM7 | a raw string's text not blanked | **28b RED** (1): the multi-line raw string reported |
| VM8 | a number blanked like a literal | **28b RED** (1): `int k = 11; int j = 22;` |
| VM9 | the splice pattern without blanks after the backslash | **28b RED** (1): the blanks plant |

- **An interrupted run, cleaned up.** My first `src_build_gate.py self-test` of this round was STOPPED (by me, two
  minutes in, to finish the limit sentences before the gates ran on them) and left its first plant —
  `static int gate1_selftest_unused_variable = 42;` at the end of `src/programs/bench_r3a.cpp` (a file this run does
  not change). Found by the plant grep, restored with `git checkout`, timestamp bumped; no other plant
  (`gate1_selftest|kSelfTest|zz_self_test` over `src include test`: none).

**VR-1's prose review — the greps run FIRST, every hit read** *(before any sentence above claimed it)*:
`any spelling|misses none|cannot span a line|every spelling|continuations joined|literals stripped` over `*.py *.hpp
*.cpp *.md *.txt`: the gate's header :267 ("comments and string literals stripped, as above" — true, "above" is the
rewritten reading sentence), :796 and :954 (true: literals are stripped), :2621 (case 28's selection function — another
subject), the 28b comment's "THE NAME, IN ANY SPELLING" (REWORDED: "WHEREVER IT IS WRITTEN"), R-1's paragraph in this
log (ANNOTATED, history kept), the SER1 and G1 logs (history), `auton_tester_program.cpp` :35 (another subject — true).
The four documents the finding named were rewritten with the real limit (above); `mechanism namespace|robot's
namespace|namespace rule` over `docs/`: build-order :1646 (no completeness claim), the MECH and G3a briefs (plans —
no claim of the rule's reach). No guide, cookbook or the changelog describes the gate's reading. `259 detector`: only
this log (history; the count is now 269).

**VR-1's gates, final (the restored tree; `git status` lists only this run's files)** *(`date` at this entry: Thu Oct 29 08:35:15 PM EDT 2026)*:
- `vfix/build.sh v2` (the stale binary removed, the briefing regenerated as unbuilt, `cmake -S test -B build/test`, the
  WHOLE `cmake --build build/test -j8`, 240 objects — the banner edit and the src gate's fresh builds): rc 0, no
  doc-gate error. **Suite (`tools/suite_run.py green`, capped 4 GiB / 600 s): green — 2,624 cases, 2,800,491
  assertions** (unchanged: this round changed no C++ code). The briefing regenerated from that fresh binary, then
  **`briefing_status.py check`: current**; its self-test OK. **`ctest -R compile_refusal`: 255/255 passed.**
- `api_doc_tool.py` self-test OK; check-coverage 4,960 entities / 186 headers; check-fresh rc 0 (no `///` changed);
  check-examples 600 quoted lines verbatim (10 source files; 6 whole-run listings unbroken); check-removability clean.
  `doc_staleness_audit.py` self-test OK, the audit clean (31 live docs).
- CI greps: no `pros/` include outside `hal/pros/`; no `shulib/sim/` include in the core; `PROBE_`: 0.
- **ARM header gate: 226 headers, `arm-none-eabi-g++ -std=gnu++20 … -Wshadow -Werror -Os -mcpu=cortex-a9
  -mfpu=neon-fp16 -mfloat-abi=softfp`: CLEAN.**
- **`src_build_gate.py self-test`: OK — 269 detector cases, each against a real build** (259 + 28b's seven plants and
  two direct checks + case 30's raw-string plant; 20:09–20:29; every plant restored). **`check`: PASS — all 7 builds
  link** (bench, xdrive, tank, tank-drive, tank-calibrate, tank-autontest, tank-competition: 11/11 TUs each, 8
  vendor-ignored warnings, 0 disallowed).

**Notes for the coordinator, added by the verification round:**
12. **The shared gate file.** VR-1's fix is in step 0's SHARED reading, not only the per-robot functions:
   `_splice` (new), `_logical_lines`, `_CXX_TOKENS`, `_strip_comments`, `_literal_runs`, the header's step-0 reading
   sentence and the per-robot paragraph, case 28b and one plant in case 30. G3a Part 1 also edits
   `tools/src_build_gate.py` (its freshness gate and `[generated]` step): whichever lands second rebases; a G3a rule
   that reads C++ through `_strip_comments` / `_logical_lines` gets the corrected reading (splices joined with nothing,
   numbers and raw strings whole, line numbers right after a spliced literal).
13. **Two offers, not made (each a decision beyond VR-1):** (a) refuse `##` and `%:%:` under `src/` — it closes the one
   text gap the rule states (a name pasted together); a new rule on the team's code, the team lead's call. (b) Step 0
   reads no file outside ONE_CHAIN_ROOTS — `include/main.h` (PROS's project header, which a team may edit) could hold
   `using namespace shulib::bench::robot_two;` or a build switch and no step-0 rule would see it; pre-existing since
   SRC1, every rule alike.
- **VR-1 DONE — re-checked REAL, fixed at the root (the reading), three more spellings of its class found and fixed,
  the limit stated where the claims were; every gate green.** Not committed (the coordinator commits).

### The coordinator's merge verification — MECH-A5 (2026-10-30 08:45–09:30, main `6798aa5` + the squash of `mech-a5`)
- **Audit before the merge:** `audit_wt.py mech-a5` — 29 backed-up files, 0 suspicious; the two files newer than their
  backups (`src/robots/mechanism_table.hpp` against V9b, `tools/src_build_gate.py` against VM9) diffed by hand: the
  verification-fix round's doc-comment and splice-rule edits, no live mutant. No gate plant; no stray root file. The
  transport commit `d37fa79`, then `git merge --squash mech-a5` (base `4691757`).
- **Four conflicts:** the briefing (regenerated); the changelog — A5's "no API change" entry dated as landed
  ("2026-10-30 (built 2026-10-29)") at the top of `## API 2.5`, above IMU1's and CAL1-A2a's; the register — the status
  line **7 of 182** (main's 181 with IMU1's three, plus A5's HA-185; 182 index rows counted), IMU1's "added THREE"
  paragraph then A5's "added ONE" ending "With it, 182", and both new sections (IMU1's, then A5's); build-order — A5's
  MECH status rewritten as LANDED 2026-10-30 (N9 "is CAL1 Part B's"), the sprint's line kept after it in the past tense,
  and the `Next:` line by a word-level three-way merge against `4691757` (both sides' additions present), its "A5
  BUILT … in review" phrases then made LANDED. Auto-merged but stale, corrected: the roadmap's A5 "Now" paragraph and
  its F-row line, HANDOFF :297, REMAINING-WORK's top status and row 12 — all said "built, waiting on its review".
- **Gates on the merged tree** (`.claude/coordinator/logs/mech_a5_merge_verify.out`): host build with the doc gates;
  suite **2,721 | 2,721 passed, 3 skipped** (main's 2,723, less A5's two retired driver-controls cases, plus its
  new ones — the same net as its own run against its base); `src_build_gate.py` self-test OK and check **PASS, all 7
  builds**; ARM header gate over **228** headers clean; CI greps empty; api check-fresh, the briefing, removability and
  the staleness audit clean; the compile-refusal harness **255 of 255**; no plant, no stray root file.
- **The coordinator's hand mutations** (`muts_mech_a5_main.json` + `_b`; each on a fresh link, the whole suite under a
  4 GiB / 300 s cap, restored and hash-checked): **7 of 7 caught** — MA1 (pivot 6's buttons swapped), MA2 (pivot 7 at
  11 V, not full power), MA3 (the program-error stop never cancels a mechanism's claimant — A5-D6), MA4 (the
  program-error stop skips the safe state), MA5 (the drive check forgets the IMU's port), MA6 (the boot's
  declared-port rule forgets an angle sensor — A5-D4) RED; **MA7** as first cut (the right side's loop bound `0`)
  broke its own build (`-Werror=type-limits`) — NOT counted; re-cut as **MA7b** (the drive check skips the last
  right-side port) → RED at `test/mechanism_table_test.cpp` :601. Restored tree green.
- **Logged, not fixed (A5's note 5):** COMP-b's routine-stop line is 214 bytes in its worst case against
  `kCompetitionLineBytes` 200 — CUT with its `cut` flag set, never overflowed; a realistic line is ~163. A small item
  for a later brief or ruling.
- **Next:** MECH-A6 (station 13 and the direction check page). G3a Part 5 (robot two's mechanisms and controls in the
  tool, D7's flip first) can start once G3a Part 1 merges and the panel amendment is answered.

---

## MECH-A6 — station 13 and the direction check page

- **Started:** Fri Oct 30 10:19:10 AM EDT 2026 (`date`).
- **Tree at start:** `81eb666` (main: MECH-A5 landed there; IMU1 Part A `6798aa5`, CAL1 Part A2a `e5a69b5`, the
  PANEL SPRINT plan `d8fdb8b`), clean, in the git worktree `.claude/worktrees/mech-a6` (branch `mech-a6`) — NOT the
  main working tree.
- **Go:** the team lead, 2026-10-08: "Accept all, go" — N1–N13 as recommended (N5 (a): the 3 × 5 menu); his rulings
  at A4's merge (2026-10-29); THE PANEL SPRINT (2026-10-29): the panel by 2026-11-21, the Nov 23 competition work
  keeps its place, and the panel brief's D20 puts MECH-A6 FIRST among the five job slots (station 13 is how robot
  two's mechanisms are brought up and checked on the robot before 2026-11-23). NOTHING IS SKIPPED. The coordinator's
  workflow launched MECH-A6 ONLY. Do not commit; no `pros` command; no `/dev/ttyACM*`. Mutation backups and probes
  under `.claude/coordinator/scratch/mech-a6/` (never `/tmp`, never the repository root). **Memory safety
  (2026-10-15):** every test binary runs as `(ulimit -v 4194304; timeout 600 <cmd>)`; a mutant killed by the cap or
  the timeout counts as CAUGHT (said which); the briefing is regenerated only after `rm -f build/test/shulib_tests`.
- **Scope:** §2's A6 row — §1.14 entire: the 3 × 5 menu (N5), station 13's readings in declared units, its six
  gates, the direction check page with its three answers, RECORD lines, HA-184/186's settles — in
  `src/programs/bench_r3a.{hpp,cpp}` ONLY; `test/bench_menu_grid_test.cpp` and new host tests of the station's pure
  parts; documents: `R3a-BENCH-WORKSHEET.md` (station 13, its steps, what each verdict means), the mechanisms
  chapter's bench page section, the changelog. Every existing bench station unchanged (proved below).
- **Reading (the team lead's ruling of 2026-10-22):** RESUMING's standing rules; the brief's header and answers, the
  re-check's amendments and dispositions IN FULL, N1–N13, the Decisions, §6, §7, §8 landmines IN FULL; every
  section/row/test list naming MECH-A6 or its files (§1.11, §1.14, §2's A6 row, §2.1, §3.6, §4.1–§4.3); the rest by
  heading. `## MECH-A1` .. `## MECH-A5` of this log grepped for every `A6` hand-on.
- **In flight beside this run (not touched here):** G3a Part 1 (`tools/robot_tool.py`, `src/robots/chassis_table.hpp`'s
  `[chassis]` markers, `src/build_selection.hpp`, `tools/src_build_gate.py`'s `[generated]` step,
  `docs/guide/19-your-robot-file.md`); CAL1 Part A2b (`calibration/calibrate_program.hpp`'s typed half, the golden
  harness, guide 12); L6-A1 (`localization/*`, `chassis/wall_reset_call.hpp`, `sim/field_plant.hpp`); VIS1-A1 (the
  camera device layer). LATER, planned around this run's `bench_r3a.*`: VIS1-B (station 12's functions), L6-B
  (station 11's functions, a NEW `src/programs/bench_wall.hpp`), IMU1 Part B (station 2's functions) — this run keeps
  its edits to the menu, the grid and station 13, and names every function it touched (below).

### 10:19 — start; the A6 hand-ons found by the grep
- `docs/internal/RESUMING.md` read (standing rules, pace rules 1–8). The brief read as ruled above. `## MECH-A1` ..
  `## MECH-A5` grepped for `A6` / `station 13` / `bench page` / `toMechanismConfig`. The hand-ons:
  1. **A1-D11:** `MechanismParts` BORROWS its configs — "MECH-A6's station keeps its configs in static storage".
  2. **A3 (:1927):** station 13's numbers are A6's to add to HA-182's entry (the jog volts and time, the "did not
     move" angle, the "drawing" current).
  3. **A4 (:2820):** a 6 V hold-to-run pushing an arm into an obstacle draws 1.25 A on the host test plant — under
     the recipe's 2.0 A signature — "worth a sentence in DOCS3 or A6's bench page (a held button's current is
     visible there)".
  4. **A4-D28 (:2914, :3024):** the judgement has a NINTH finding, `BackDriven` ("jog it harder, or from another
     pose" — no line named); the page shows it.
  5. **A4-D29 (:3024):** every role-naming finding has a generated-file sentence (a role's `source` set): the page
     prints `judgeDirection`'s / `judgePneumatic`'s sentence as it comes, so a generated file's path reaches the
     screen unchanged.
  6. **A5:** the mechanism boot lines (`src/common/driver_panel.hpp`'s `mechanismDeclarationLine`, `roleLine` over
     `MechanismParts`' facts) and the projection `mechanism::toMechanismConfig` — the station shows each role's
     line in the same words; the src gate's step 0 (per robot): a program names `kMechanisms` only.
  7. **A5's merge record:** next is MECH-A6; nothing else handed on.

### 14:20 (2026-10-30) — RESUMED after the account's usage limit *(`date`: Fri Oct 30 02:16:05 PM EDT 2026)*
- **Where the stopped run stood** (read from this log and the worktree, never assumed): this heading, the start
  entry and the hand-on list above — nothing else. `git status`: only `docs/internal/PROJECT-BRIEFING.md` (the
  briefing regenerated with the binary removed — "Suite: not built") and this log; `git diff -- src/ Makefile`
  EMPTY (no gate self-test plant, no mutant). The scratch directory held `build.sh` (the safe build: the binary
  removed FIRST, the briefing regenerated, the whole host build), `cmake_config.out`, and `baseline.out` — the
  baseline build `Terminated` mid doc-gates when the limit cut the run; `mut/` empty (no mutation was ever run).
  Nothing to restore, nothing to revert.
- **The fresh baseline** (`build.sh baseline`: rc 0 — every TU compiled from source, the binary relinked) and
  the suite under the cap (`ulimit -v 4194304; timeout 600`): **2,721 | 2,721 passed, 0 failed, 3 skipped;
  2,937,848 assertions** — the A5 merge record's count exactly.
- **Re-read for the resumption** (the team lead's 2026-10-22 ruling): RESUMING's standing rules; the brief's
  header, the re-check's amendments, N1–N13 and the Decided-here lists IN FULL; §1.11, §1.14, §2's A6 row, §2.1,
  §3 intro and §3.6, §4.1–§4.3, §6, §7, §8 IN FULL; the hand-ons above re-grepped (A4-D28's `BackDriven`, A4-D29's
  generated-file sentences, A1-D11's static storage, A3's HA-182 numbers, A4's 1.25 A note, A5's boot lines).
  The code read before writing: `src/programs/bench_r3a.{hpp,cpp}` (the menu, its grid, station 10's six gates,
  stations 11/12's live panel), `test/bench_menu_grid_test.cpp`, `mechanism/direction_check.hpp`,
  `robot/mechanism_parts.hpp`, `mechanism/{spec,wiring,devices}.hpp`'s projection, `roleLine` and the units
  conversions, `hal/mechanism.hpp`, `src/common/driver_panel.hpp`'s mechanism boot lines,
  `src/robots/mechanism_table.hpp`, `teleop/coupled_side_monitor.hpp`, `manipulation/stall_detector.hpp`,
  `hal/pros/device_provider.hpp`, `hal/fake/fake_device_provider.hpp`, `test/mechanism_fixtures.hpp`.

### 14:26–15:05 (2026-10-30) — the code, the tests, every mutation on a fresh binary *(`date` at this entry: Fri Oct 30 03:05 PM EDT 2026)*
- **What was written, and where (the A6 row's files ONLY — `src/programs/bench_r3a.{hpp,cpp}`, the two tests):**
  - **`src/programs/bench_r3a.hpp`** (PROS-free, the R2a precedent): `menu_grid::kMenu` → `{13, 3, 6, 38, 153, 36, 4}`
    (N5 (a): five rows of 36 px, 38 + 5 × 40 − 4 = 234), `kMenuSlots = 15`; the inside-a-button offsets for a 36 px
    button (line 1 MEDIUM at +2, line 2 **SMALL** at +22 — two MEDIUM lines need 44 px — `kSmallPitch = 13`, the dot at
    +28 r 5); `kAnswerRow4` / `kAnswerRow4Hit` (a three-position pneumatic's four answers); and **NEW
    `namespace mech_station`** — the page's stated numbers (HA-182's page entries), `projectConfigs()`,
    `unitsReading()` / `unitsText()`, `temperatureText()`, `motorText()`, `switchText()`, the piece evidence
    (`Evidence`, `distanceEvidence()`, `opticalEvidence()` — the service's positive-evidence rule, mirrored for the
    page), `FaultWatch`, `Pad`, `Phase`, `PromptText`, `wrapText()`, and **`class Station`**: the six gates, the
    direction check, the readings (`panelLines()`) and the RECORD lines (`record()`), over `robot::MechanismParts`.
    It includes `common/driver_panel.hpp` for `mechanismKindWord()` (the boot lines' kind words — not copied).
  - **`src/programs/bench_r3a.cpp`**: **functions touched** (for VIS1-B / L6-B / IMU1 Part B, who plan around this
    file): `drawMenu()` (line 2 drawn SMALL; the header's "READ-ONLY except 10 + 13"), the `kMenu[]` table (entry 13),
    `kMaxMenu`/`g_verdict` (13), `kBtnLine2Cols` (SMALL budget), `testerInitialize()` (ONE serial line added — the
    banner named only the DRIVE station as powered), `runR3a()` (the splash's and the `#` banner's "except" sentences);
    two static_asserts (`kLineH == menu_grid::kSmallPitch`); comments only in the file banner and station 10's header
    (its "ONLY code path that calls setVoltage" note grows to two, as §1.14 says); **NEW**: `namespace mech13`
    (`Built` / `built()` / `provider()` / `station()` — function-local statics, once per power cycle;
    `rawTemperature()`, `prompt()`, `ExitGuard`, `drawAnswer4()`) and **`mechanismsStation()`**. The include of
    `robots/mechanism_table.hpp` (`kMechanisms` only — the per-robot rule) and `hal/pros/device_provider.hpp`.
    **Untouched:** every function of stations 1–12 (`census`, `reportImu`, `motorWatch`, `driveStation`, …,
    `distanceStation`, `aiVisionStation` and their helpers) — proved by the function diff below.
  - **`test/bench_menu_grid_test.cpp`** (rewritten where the grid changed): 15 slots inside 240 px, no overlaps, the
    exact rectangles; stations 1–12 at their row and column with their columns' x pixels unchanged, and R2a's 3 × 4
    spec reproduced by the helper pixel for pixel against its stated rows; the 3 × 5 hit test against an
    independently re-typed oracle at every pixel of −2..481 × −2..241; the empty fifth-row slots dead (and counted);
    the gap above 13 belongs to 10; the button's inside (SMALL line 2, the dot, 11 / 13 characters); the four-answer
    row. **`test/bench_mech_station_test.cpp`** (NEW, 23 cases): the gates one by one, the pneumatic, the check's
    flow and its at-once findings, the readings, RECORD, the build, NEXT, the wrap, the prompts' widths.
- **Built:** `build.sh` (the binary removed first, the whole host build) — the station's header and the cpp compile
  clean on the host warning set (−Wformat=2 included) and in the ARM `make ROBOT=tank` and `make` (bench) objects
  (`bin/programs/bench_r3a.cpp.o`: no warning but the vendored liblvgl one every build carries). **Suite under the cap:
  2,749 | 2,749 passed, 3 skipped; 2,938,908 assertions** (base 2,721: +28 cases).
- **The clock is WHOLE MILLISECONDS** (`Station::tick(pad, nowMs)`, `pros::millis()`): a first cut kept seconds as
  doubles, and `(a + 400) / 1000.0 − a / 1000.0` can land an ulp under 0.4 — an equality edge the tests must land on
  exactly (the coordinator's lesson); the stall detector is fed the time since the jog began, so its window's edge is
  exact too.
- **Mutations** (`scratch/mech-a6/mutate.py`: ONE exact replacement, the binary REMOVED, `cmake --build --target
  shulib_tests`, both dependent TUs seen recompiling in every build output — `bench_menu_grid_test.cpp.o`,
  `bench_mech_station_test.cpp.o` — the station and grid cases run under `ulimit -v 4194304; timeout 600`, the file
  restored in a `finally` and on SIGTERM/SIGINT, its timestamp bumped). A first run (24) at 14:54; the FINAL run (29)
  at 15:00–15:01 on the final header (`mut/run_final.out`), all 29 backups of the last run byte-equal to the restored
  file, then a fresh build (b4) and the whole suite green again:
  - **M30** (the dead-man ignored: `held = pad.connected`) → RED, 4 cases (gate 3, the valve, the pneumatic check,
    the abandon); **M31** (the ceiling starts at 12 V) → RED, 9 cases (gate 4: `12 == 3`); **M32** (a valve fired
    without its confirm) → RED, 3 cases; **M48** (the stall signature not watched) → RED (gate 5's stall case).
  - MA6-1 (the release safes nothing) RED · MA6-2 (the fight cut also cuts a back-drive) RED · MA6-3 (over-current
    at 2.4 A itself) RED · MA6-4 (the jog's maximum a tick late) RED · MA6-5 (the check's jog a tick long) RED · MA6-6
    (NO arms) RED · MA6-8 (the sensor's mounting not applied) RED · MA6-9 (the gearing upside down) RED · MA6-10 (an
    infinite temperature shown as a number) RED · MA6-11 (the exit safes nothing) RED · MA6-12 (NEXT leaves it
    powered) RED · MA6-13 (no CLEAR needed) RED · MA6-14 (the release safes the untouched too) RED · MA6-15 (fired
    with L1 let go) RED · MA6-16 (a distance AT its threshold is a piece) RED · MA6-17 (the switch inverted) RED ·
    MA6-18 (a cut during the check throws the jog away) RED · MA6-19 (the menu's rows 37 px) RED (2 grid cases) ·
    MA6-20 (a jog starting on a HELD arrow) RED (5 cases) · MA6-22 (the screen not asking for the switch's three
    readings) RED · MA6-23 (an answered verdict not counted) RED.
  - **MA6-7 and MA6-21 SURVIVED — equivalent mutants, logged, re-cut:** each removed one term (`!built(i)`, `cut`)
    from the guard `if (!cleared_ || !built(i) || mech_[i].cut) return;` that sits AFTER the per-press refusals; every
    press is refused before it and an unbuilt or cut mechanism is never armed or jogging, so the term is
    defence in depth that no reachable state reaches. Re-cut to remove the refusal AND the term: **MA6-7b → RED**
    (gate 1's case: the unbuilt pivot asked for a confirm), **MA6-21b → RED** (a cut lift powered again on a fresh
    press). The terms are kept (a powered station's guard), and said here.

### Every existing bench station unchanged — the proof *(2026-10-30 15:08)*
- `git diff -U0 src/programs/bench_r3a.cpp` — its 26 hunks, each placed: the file banner's comments (2), two
  `#include`s, the `kLineH == menu_grid::kSmallPitch` static_assert, `kMaxMenu` / `g_verdict` (13), station 10's
  HEADER COMMENT (2 hunks: "THE FIRST STATION THAT POWERS", the "ONLY code path" note grown to two — the lines above
  `namespace drive`, which git's context labels `motorWatch`), the NEW station 13 block (after `aiVisionStation`'s
  closing brace), `MenuItem`'s comments, the menu table's entry 13, the grid comment, `kBtnLine2Cols`, `drawMenu()`,
  `testerInitialize()`, `runR3a()`. No hunk falls inside a station function.
- **Mechanically** (`scratch/mech-a6/fn_diff.py`, every top-level definition of the base `81eb666` file against
  this one): **48 definitions; 43 byte-identical** — `census`, `reportImu`, `reportMotors`, `motorWatch`,
  `driveStation` and the whole `namespace drive`, `measureLoopRate`, `reportPlatform`, `probeSdCard`, `screenRuler`,
  `distanceStation`, `aiVisionStation` and the whole `namespace sensors` (LiveLine, drawRowButton, TapWatch, the
  distance and camera helpers), `tImu`, `tAll`, `emit`/`emitf`/`emitS`, the screen helpers, `Grid`/`gridFor`/
  `gridCell`, `panelSafe`, `menuLiteralsFit` … — and **5 changed: `MenuItem` (comments), `kMenu` (entry 13 added),
  `drawMenu`, `testerInitialize`, `runR3a`** (the menu and the banners, none a station); added `mech13`,
  `mechanismsStation`. (One-line and multi-line-signature functions — `tCensus`, `twoButtonPrompt`, `screenText`,
  `hitTest` — the detector does not split out; the hunk list above shows none of them touched.)

### Deltas from the brief (MECH-A6), each with its reason
- **A6-D1 — the station's pure parts are `bench_r3a.hpp`'s `namespace mech_station`** (the A6 row writes
  `bench_r3a.{hpp,cpp}` only; the R2a precedent keeps that header PROS-free). It now includes the library's
  mechanism headers, `robot/mechanism_parts.hpp`, `teleop/coupled_side_monitor.hpp`, `manipulation/stall_detector.hpp`
  and `common/driver_panel.hpp` (for `mechanismKindWord()` — the boot lines' kind words, used, not copied); so
  `src/main.cpp` (every build) and the two bench test TUs compile them. All 7 builds were gated with it (below).
- **A6-D2 — gate 2 is stricter than "before the first volt or the first line in a power cycle":** stage 1 (CLEAR) is
  asked at EVERY entry, before anything is made — making a pneumatic's 3-wire line DRIVES it to its start level
  (HA-119), so the build itself waits for that YES (once per power cycle: `MechanismParts` builds once); stage 2 is
  asked before a motor mechanism's first volt in each ENTRY (station 10 asks its wheels-up question at every entry
  too) and before EVERY pneumatic fire, naming its position (a cylinder cannot be stopped mid-stroke). A YES only
  ARMS a motor mechanism; it moves on a fresh press (an arrow held through the prompt does not drive — MA6-20).
- **A6-D3 — gates 3 and 6 safe every mechanism the station POWERED, not every one built.** A mechanism the station
  never commanded is never commanded: a pneumatic whose safe position is not its start (robot y's `grip`: start
  'loose', safe 'shut') would otherwise MOVE on the first L1 release with nobody asking — against "never move one
  without the operator's touch". The guard and DONE call the same `Station::exit()`.
- **A6-D4 — the fight cut cuts a FIGHT** (some of a mechanism's motors persisted against the others), never every
  motor alike against the command: that is a BACK-DRIVE (A4-D28's finding), not a fight; station 10 cuts a whole
  side because a whole drive side opposite means a wrong front. A one-motor mechanism has no fight rule (the
  evaluator would read a sagging arm as "opposite"). The 2.4 A cut, its own stall signature and the 3 s jog still
  apply. MA6-2 is its mutation.
- **A6-D5 — a cut during the check's jog ENDS the jog and the judgement runs over what it saw** (a two-motor fight's
  250 ms cut lands inside the 0.4 s jog: the check then names the fighting motor, MotorFights — abandoning it would
  throw the one finding away). MA6-18.
- **A6-D6 — the page's numbers** (HA-182's page entries, §4.3): the check's + jog 0.4 s at the ceiling (started by
  L1 + A, its volts fixed at its start); "did not move" 0.035 rad (~2° at the motor); "drawing" 0.40 A; a jog's
  maximum 3 s; station 10's 2.4 A / 250 ms and its 3 → 12 V ceiling. All invented, said on screen and in HA-182.
- **A6-D7 — the menu's second line is SMALL** (two MEDIUM lines need 44 px; a 36 px button has 35 inside its
  edges): line 1 MEDIUM at +2 (11 characters), line 2 SMALL at +22 (13), the dot at +28 r 5. Every station's button
  words are unchanged; 13's are `13 MECH` / `POWERS` (logged `13 MECHANISMS (POWERS)`), red stripe and edge.
- **A6-D8 — "stations 1–12 at their numbers and places … pixel-for-pixel against the 3 × 4 grid it replaces"**
  (§3.6) built as: stations 1–12 keep their ROW and COLUMN and their columns' x pixels (their y pixels cannot stay:
  N5 (a)'s rows are 36 px); the generalized helper still reproduces R2a's 3 × 4 grid pixel for pixel from its spec
  (an oracle typed from its stated rows); and the new grid's hit test agrees with an independently typed oracle at
  every pixel on and around the panel.
- **A6-D9 — "in its declared units, from power-up"** for a position mechanism takes the power-up pose as its
  declared HOME (`angleDegToPosition(positionToAngleDeg(home) + travel)`): for an arm or a spool lift that is home +
  travel (the §3.6 numbers, 360° and π × 1.5 in), for a linkage or a table the height the service itself would
  believe at boot (`restsAtStop`). The screen says it: `(power-up = home 0.0; 5.00:1)`.
- **A6-D10 — temperature is read RAW** (`pros::c::motor_get_temperature`, PROS_ERR_F = infinity → `unknown`): the
  adapter holds its last good value and seeds 20 °C, so its `temperature()` could never say "unknown".
- **A6-D11 — the piece evidence is MIRRORED for the page** (`distanceEvidence` / `opticalEvidence`, the service's
  `Slot::seen()` rule): the station builds no service to ask. The drift risk is named in its comment and its test
  pins the same edges (AT the threshold: no piece; near with confidence 0: no evidence; MA6-16).
- **A6-D12 — a controller is NOT required to enter** (station 10 refuses without one): the readings by hand are
  worth having without one; nothing can be powered without it, and the gates line says so.
- **A6-D13 — the serial boot banner gains ONE line** (`testerInitialize()`: it said only the DRIVE station powers),
  and the splash, the menu header and the `#` banner name 13 — their comment's "bytes unchanged" corrected.
- **A6-D14 — the page shows ONE mechanism at a time** (NEXT; seven reading lines, the gates line, the status line); a
  NEW verdict takes the reading lines until L1 is pressed again, NEXT, or 20 s (the log keeps it).
- **A6-D15 — the worksheet station is "Station P"** (N is IMU1 Part B's; L appears in an old T4 brief note).
- **A6-D16 — the clock is whole milliseconds** (above): the edges exact, on the host and the brain alike.
- **A6-D17 — no `MECH-COMPLETED.md` yet:** A6 is BUILT, not closed — §4.1's "at A6's close" is after its review and
  its verification; the status pointers say "A6 BUILT, in its review" (build-order's first `Next:` and its MECH
  entry, the roadmap's "Now" and MECH row, REMAINING-WORK row 12, HANDOFF).

### The documents, and the prose review (pace rule 4) — every grep RUN and its hits READ before this entry *(2026-10-30 15:05–15:12)*
- **The A6 row's documents:** `docs/internal/chunks/R3a-BENCH-WORKSHEET.md` — Station A (the menu's 3 × 5 picture,
  13's row in the table, "buttons 10 and 13" are the red ones, the "EXCEPT" sentence) and **NEW Station P** (before
  J: its gates, P.0–P.8 step by step, the screen's strings copied from the code, **what each verdict means** — a
  table of the nine findings and the line each names — the HA-184/186 steps for the robot that first declares a
  switch or a sensor, what to send back), and J.4's pointer to P; `docs/guide/18-mechanisms.md` — the intro (the
  sixth part is in) and **NEW section "The bench page — Bench Tests station 13"** (the readings in declared units,
  the six gates, the check, "it saves nothing"; A4's hand-on: a held button's current is the number a jam signature
  is sized by — 1.25 A at 6 V on the host test plant, under a 2.0 A signature); `docs/changelog.md` — an entry at the
  top of `## API 2.5` ("Built 2026-10-30 … no API change").
- **The register** (`docs/hardware-assumptions.md`, own lines only): HA-182's row and entry (the page's numbers,
  §4.3's "A6 adds the page's"), HA-184's and HA-186's Source and Settle (the built page; Station P's P.7.1 / P.7.2),
  HA-153's Settle (Station P first), a MECH-A6 status paragraph ("added NONE … Still 182"); and SER1's HA-189 note
  (the menu "was" full — the ruling stands). No new HA; the held line untouched.
- **Status pointers (own lines):** build-order's first bold `Next:` (MECH clause → "MECH-A6's review, verification and
  merge (A6 BUILT …), then the core's completion record") and its MECH entry (an A6 paragraph) and the SER1 note's
  "(3 × 4 … until MECH-A6 made it 3 × 5)"; the roadmap's "Now" and the MECH row's A6 line (`[~]` until A6 lands);
  REMAINING-WORK row 12; HANDOFF's library list and its chapter-18 note.
- **Prose-review greps** (over `docs/` and `README.md`, the API pages excluded):
  1. §4.2's A6 grep `Bench Tests|station 1[0-3]|3 × 4|3 x 4|twelve stations|twelve buttons` (the chunk logs filtered):
     FIXED — the worksheet's Station A ("twelve buttons, three across and four down", the 4-row picture, "the only one
     that powers motors"); the register's HA-189 note ("the bench program's menu is full — twelve stations…": now
     "was full", with MECH-A6's note — the team lead's K.7.3 ruling stands); build-order :1741 (the same SER1 fact,
     dated). LEFT, history or still true: build-order :663 (R3b Part 0's landing record, 2026-09-10: "station 10 DRIVE —
     the ONE station that powers motors" — true then), the roadmap's R3b/R2a "Now" records (:193–:226, :462), guide 07's
     program list, the glossary's "the tester", REMAINING-WORK rows 15/18c/24 and :181/:274, HANDOFF's session history,
     ORIENTATION's program list, the register's HA-136/166/215/HA-04 rows (other subjects), the IMU1/L6/VIS1/G3a
     briefs' plans around `bench_r3a.*` (A6 kept to the menu, the grid and station 13, as they plan).
  2. `bench page|sixth part|core.s last|station 13|MECH-A6`: FIXED — guide 10 :555 ("the bench page that runs it is the
     core's last" → it exists, linked to chapter 18's section) and :572 ("settled at the bench page" → station 13,
     built, not yet run); guide 14 :228 ("the bench page … is its sixth" → since its sixth and last, built, never run);
     README :293 (the mechanism line names station 13); HANDOFF :823 ("A6 adds the bench page" → added); the register
     HA-153 :2210 ("once it lands"). LEFT: the changelog's A1/A4 entries (:377, :416 — history), guide 18 :176/:182/:187/
     :514 (true now: the page reads them), MECH-PANEL-requirements (the requirements, history), REMAINING-WORK :71
     (true), the G3a/VIS1/L6/IMU1 briefs (plans that name A6 as the owner of the menu and station 13 — true).
- **C7 removability** (`grep -rln "internal/\|chunks/\|RESUMING\|build-order" README.md docs --include=*.md | grep -v
  "^docs/internal/"`): EMPTY. The guide's new section links only to chapter 18 itself and the register.

### 15:10–15:14 — three defects found re-reading the station, each test written FIRST and seen RED on the unfixed header *(`date` 15:13:54)*
While the first src-gate run held `src/` (its self-test plants into `bench_r3a.cpp`, so nothing in `src/` was edited
under it), a re-read of `Station::tick()` / `confirm()` found three ways something could move with nobody pressing:
- **A6-F1 — a controller that comes back resumed a jog.** The press edges were fed `false` while the controller was
  gone, so L1 + UP still held at the reconnect read as a NEW press: an armed pivot jogged again by itself. **Fix:** an
  edge is updated only on a connected controller (a press is a release-then-press SEEN). Test `station 13: nothing
  resumes by itself …` — RED on the unfixed header (`3 == 0`, live).
- **A6-F2 — a hold-to-run jog resumed after a check.** A check started with UP still held (A pressed mid-jog) left
  `jogDir_` set; once the check ended, the held arrow drove again without a press. **Fix:** `startCheck()` clears the
  jog. Same test, its second half — RED (`3 == 0`).
- **A6-F3 — L1 let go DURING a confirm prompt kept a fired claw fired.** `confirm()` set `heldWas_` from L1 at the
  answer, so the release that happened while the prompt blocked the ticks was never seen: a claw fired 'open', a
  second fire asked, L1 let go, YES → "NOT fired" — and the claw stayed OPEN with nobody on the dead-man. **Fix:** a
  confirm answered with L1 not held IS the dead-man's release: whatever the station powered goes safe there and then
  (logged). Test `station 13: L1 let go DURING a confirm prompt …` — RED on the unfixed header (the line still HIGH).
- Two screen fixes for the same edit: the header bar redrawn after the CLEAR prompt (the prompt erases the whole
  panel — station 10 redraws it the same way), and the station's opening lines cut to the scrolling log's 54 columns
  (`rule()`'s title too: the header shows 30).

### 15:24–15:32 — the first src-gate run, the three fixes applied, the FINAL mutation run *(`date` 15:31:29)*
- **The first `tools/src_build_gate.py` run** (on the tree before the three fixes; nothing in `src/` edited while it
  ran — `sha256sum` of `bench_r3a.{cpp,hpp}` and the `Makefile` equal before and after): **self-test OK (269 detector
  cases, each against a real build; 21 min)**; **check PASS — all 7 builds link** (bench, xdrive, tank, tank-drive,
  tank-calibrate, tank-autontest, tank-competition: each exit 0, 11/11 TUs, 8 vendor-ignored warnings, **0
  disallowed**, hash and beacons asserted).
- **The fixes applied** (A6-F1..F3 and the two screen fixes, above) — `bench_r3a.hpp`'s `tick()` / `startCheck()` /
  `confirm()`, `bench_r3a.cpp`'s `mechanismsStation()` only; every literal line of the station's scrolling log then
  checked at ≤ 54 columns by a script (four were longer: the "no mechanism" path, the failed-build path and the exit's
  "nothing is saved" — each split). ARM object (`make ROBOT=tank bin/programs/bench_r3a.cpp.o`): no warning of ours.
- **Fresh build (b8) and the whole suite under the cap: 2,751 | 2,751 passed, 3 skipped; 2,938,941 assertions.**
- **The FINAL mutation run** (`mut/run_final2.out`, 15:29–15:30, 32 mutants on THIS header — its backup byte-equal to the
  file after the run): **30 RED, 2 equivalent (MA6-7, MA6-21 — re-cut as MA6-7b / MA6-21b, both RED)**: everything
  listed above again, plus **MA6-24** (the edges fed while the controller is gone) RED, **MA6-25** (a check leaving the
  jog set) RED, **MA6-26** (L1 let go during a prompt not taken as the release) RED. Every mutant's build recompiled
  BOTH dependent TUs (`bench_menu_grid_test.cpp.o`, `bench_mech_station_test.cpp.o`), the binary removed before each.
- The final src-gate run (self-test + check) started 15:31 on the final tree — below.

### 12:50 (2026-11-03) — RESUMED after the team lead's pause *(`date`: Tue Nov  3 12:50:11 PM EST 2026)*
- **Why it stopped:** the 15:31 (2026-10-30) final src-gate run was cut by the team lead's pause (~15:40, he
  travelled); the laptop restarted on 2026-11-03. `scratch/mech-a6/gate2_selftest.out` is EMPTY (0 bytes, 15:31) —
  the run was killed before its first line, so it proved nothing and is re-run below.
- **Checked before anything else** (never assumed): `git status` — the A6 files above and nothing else (no stray
  include-root directory, no file outside the A6 set); `git diff -- src/ Makefile` — `bench_r3a.{cpp,hpp}` ONLY;
  `sha256sum` of `bench_r3a.{cpp,hpp}` and the `Makefile` EQUAL to `scratch/mech-a6/src_hashes_before_gate2.txt`
  (taken just before the killed run), and a grep of every file the gate's self-test plants into (`bench_r3a.cpp`,
  `robot_config.hpp`, `build_selection_test.cpp`, `test/CMakeLists.txt`, `chassis_table_test.cpp`, the vendored
  `lv_obj_style.h`, the `Makefile`, `drive_program.cpp`, `calibrate_program.cpp`, `boot_source.hpp`) for its plant
  strings: EMPTY — no gate plant left. All 32 mutation backups under `mut/` byte-equal to the header: no mutant left.
- **Main has moved** since the base: `git log 81eb666..shulib-v2` is ONE commit, the panel plan `eab0f89` (plan
  documents only — `git diff --stat 81eb666 shulib-v2` over `bench_r3a.*`, the grid test and `test/CMakeLists.txt`
  is empty); IMU1 Part A `6798aa5` is IN the base (an ancestor of `81eb666`, checked); VIS1-A1 is staged on main,
  not committed. None of it is merged here; this run stays on its base `81eb666`, as launched.
- **Next:** the binary removed and a fresh whole build, the suite under the cap, then the final gates in order.

### 12:50–12:58 (2026-11-03) — a CLEAN rebuild, the suite, and every gate but the src gate on the final tree *(`date` 12:58:18)*
- **A clean rebuild, not trusted from the old objects:** `cmake -S test -B build/test` (rc 0); `cmake --build
  build/test --target clean` (0 objects left), then `build.sh r2_clean` — the binary removed first, the briefing
  regenerated reading "not built", **246 objects compiled from source** (`scratch/mech-a6/r2_clean.out`), rc 0, no
  warning. No object of a killed mutation run can have reached this binary.
- **The suite under the cap** (`ulimit -v 4194304; timeout 600`): **2,751 | 2,751 passed, 0 failed, 3 skipped;
  2,938,941 assertions** — the 15:30 (2026-10-30) count exactly. `python3 tools/suite_run.py green` (the bounded
  run): `suite run: green — 2751 cases, 2938941 assertions`.
- **The compile-refusal harness:** `ctest --test-dir build/test -R compile_refusal --no-tests=error` — `100% tests
  passed, 0 tests failed out of 255`; 255 TUs, 255 CTests.
- **The ARM header gate** (every `include/shulib` header, −Werror, Cortex-A9 softfp; its TU under
  `scratch/mech-a6/all_headers_r2.cpp`): `TU includes 228 headers` / `ARM compile gate: CLEAN (all v2 headers)`.
- **The CI greps:** `library is PROS-free`; `layering holds: core is sim-free`; `grep -rn "PROBE_" include/`
  EMPTY; the C7 removability grep EMPTY; no `PROTOTYPE` / `proposed` in the four A6 files.
- **The documentation gates:** `api_doc_tool.py` self-test OK; check-coverage `5060 public entities across 188
  headers, all documented`; check-fresh rc 0 (nothing under `include/shulib/` changed — the docs/api pages need no
  regeneration); check-examples `600 quoted lines, all verbatim`; check-removability `no public doc references
  docs/internal/`; `briefing_status.py` self-test OK; `doc_staleness_audit.py` self-test OK and `clean (31 live
  docs…)`.
- **The briefing:** `briefing_status.py check` FAILED once as expected (the briefing still said "Suite: not built"
  — `build.sh` regenerates it with the binary removed); regenerated from THIS clean link (`build/test/shulib_tests`,
  12:51:41), then `check`: `generated block current` (rc 0). Its "Suite" line: 2,751 / 2,938,941.
- **The src gate** (`self-test`, then `check`) started 12:58 on this tree, `sha256sum` of every changed and
  planted-into file taken first (`scratch/mech-a6/r2_hashes_before_gate.txt`); nothing under `src/`, `include/`,
  `test/` or the `Makefile` is edited while it runs.

### 12:59–13:00 (2026-11-03) — the prose review RE-RUN on the final tree; one stale sentence the first pass missed *(`date` 13:00:12)*
- **Re-run, every hit read** (over `README.md` and `docs/`, the API pages and the PROGRESS/COMPLETED history
  excluded): §4.2's A6 grep narrowed to the menu's facts (`3 × 4|3 x 4|twelve stations|twelve buttons|the ONE station`),
  a new grep `kMaxMenu|kMenu\{12|menu is full|menu.{0,20}full|twelve (stations|buttons)`, and the second one
  (`bench page|sixth part|core.s last|station 13`).
- **FIXED (missed by the 2026-10-30 pass):** `R3a-BENCH-WORKSHEET.md` :2310 (Station K's K.7.4 note on HA-189) still
  said "the bench program's menu is full, twelve stations in its 3 × 4 grid" — a live instruction the team reads at
  the bench, and the register's twin sentence (HA-189, :3168) WAS fixed. Now "was full then … MECH-A6 made it 3 × 5
  on 2026-10-30 for station 13 — two slots free — and the ruling below stands", the register's words.
- **LEFT, each read:** build-order :663 (R3b Part 0's landing record — true then) and :1741 (already dated "until
  MECH-A6 made it 3 × 5"); the changelog :45 (A6's own "it was 3 × 4"); the MECH brief's N5 :553-554 / §3.6 :2389 /
  §4.2 :2451 (the brief itself); R2a's brief and log, SER1's log :687 and record :61, T4's and R3b's logs (history);
  COMP-t's brief :428 and G3a's brief :2082 ("twelve buttons" = the CONTROLLER's twelve buttons, another subject);
  guide 18 :176/:182/:187/:514 (true: the page reads them; it IS the core's last part); the register :1167/:1238
  (true); the roadmap's MECH row :1789/:1804/:1811 (the A1/A4/A5 records' "still [~]" — history; :1812 is A6's
  entry); REMAINING-WORK :71 (true); the L6 brief :5/:1934-1935/:2197 and build-order :1815/:1825 (plans naming A6
  as the owner of the menu and station 13 — true).
- **NOT MINE TO EDIT — for the coordinator:** the IMU1 brief (`IMU1-imu-rate-once-per-tick.md` :733-734, IMU1 Part B's
  live plan) says "no new menu button (the 3 × 4 menu is full — `kMaxMenu` = 12, ~:329)". After A6: the menu is 3 × 5
  with two slots free and `kMaxMenu` = 13 at :339. Its plan (station 2's functions only, no new button) is
  unaffected; the reason it gives is stale. Its line references into `bench_r3a.cpp` move (measured against the base
  file, `scratch/mech-a6/bench_r3a.cpp.base`): `reportImu` :736 → :745 (+9: the banner, includes, the static_assert
  and `kMaxMenu` above it), `distanceStation` :2490 → :2501 and `aiVisionStation` :2636 → :2647 (+11: station 10's
  header comment grew by 2), `tImu` :2914 → :3345 (+431: station 13's block sits above it).

### 12:58–13:29 (2026-11-03) — the src gate on the FINAL tree, then the suite and the briefing once more *(`date` 13:28:40)*
- **`python3 tools/src_build_gate.py self-test`** (12:58–13:19): `src build gate self-test: OK (269 detector cases,
  each against a real build)`, rc 0. **`check`** (13:19–13:24): **`src build gate: PASS (all 7 builds link …)`**, rc 0
  — bench, xdrive, tank, tank-drive, tank-calibrate, tank-autontest, tank-competition: each `exit 0, 11/11 TUs
  compiled, warnings: 8 vendor-ignored, 0 disallowed, hash asserted (v0.1.1-399-g81eb666-dirty)`. Output under
  `scratch/mech-a6/r2_gate_{selftest,check}.out`. This replaces the killed 15:31 (2026-10-30) run.
- **No plant left:** `sha256sum -c` of `scratch/mech-a6/r2_hashes_before_gate.txt` (every changed file plus every file
  the self-test plants into) — ALL EQUAL after the gate; `git status` the A6 set only; `bench_r3a.cpp` ends at its
  closing namespace brace.
- **Rebuilt after the gate** (the self-test restores its plants byte-exact but bumps their timestamps, so the audit
  read the old binary as older than `boot_source.hpp` and nine more — correctly not trusted): `build.sh r2_final` —
  the binary removed first, **246 objects recompiled**, rc 0 — and the suite under the cap: **2,751 | 2,751 passed,
  0 failed, 3 skipped; 2,938,941 assertions**. The briefing regenerated from THAT link; `briefing_status.py check`:
  `generated block current`; `doc_staleness_audit.py`: `clean (31 live docs…)` (the binary trusted now).
- **The docs gates once more** after the worksheet fix: `check-fresh` rc 0, `check-examples` `600 quoted lines, all
  verbatim`, `check-removability` clean, the C7 grep EMPTY.
- `git diff --stat -- src/ tools/ include/ test/CMakeLists.txt mkdocs.yml Makefile`: `src/programs/bench_r3a.{cpp,hpp}`
  ONLY (the new test file is picked up by the suite's glob; `test/CMakeLists.txt` untouched).

### §2.1's DoD — MECH-A6, checked against the tree *(2026-11-03 13:29)*
- [x] This log carries the run's heading, its tip (`81eb666`), each step as it happened (two stops, both logged), every
  mutation with its observed red (32 on the final header, `mut/results_153025.json`: 30 RED, MA6-7 / MA6-21 equivalent
  and re-cut RED as MA6-7b / MA6-21b), the prose review's greps and hits (2026-10-30, and re-run 2026-11-03).
- [x] The host suite builds and passes (2,751 | 2,751; 2,938,941) and `ctest -R compile_refusal` passes (255/255).
- [x] The ARM header gate CLEAN (228 headers); the CI PROS-free and layering guards pass.
- [x] `src_build_gate.py check` PASS — all 7 builds link, 0 disallowed warnings; `self-test` OK (A6 runs it).
- [x] `grep -rn "PROBE_" include/` EMPTY; no `PROTOTYPE` / `proposed` in the A6 files.
- [x] Every §3.6 test exists (the grid: 15 inside 240 px, no overlaps, 1–12 at their places, R2a's grid reproduced;
  the station: every gate, the check's flow, RECORD, the readings in declared units — 360° / π × 1.5 in —, the
  switch's three states, the three answers into `judgeDirection` / `judgePneumatic`, the stall signature's cut), and
  M30, M31, M32 and M48 were RUN, observed RED and restored.
- [x] Docs in the same change (§4.1's A6 list, the internal status pointers), the prose review logged, the C7 grep
  EMPTY.
- [x] Nothing committed: the worktree holds the A6 change uncommitted on `mech-a6` at `81eb666` (16 files modified,
  one new: `test/bench_mech_station_test.cpp`). **MECH-A6 is BUILT; its review, fixes and verification follow
  (A6-D17: no `MECH-COMPLETED.md` until then).**

### MECH-A6 review findings and their disposition *(started — `date`: Tue Nov  3 01:50:50 PM EST 2026)*
- Three reviewers (correctness, tests, docs) returned 31 findings; each is re-checked against this worktree before
  any fix. The dispositions follow, one per finding (R-1 .. R-30 in the order the coordinator listed them), then the
  gates. *(Written as each step finishes.)* (The coordinator's list holds 31 entries: 10 correctness, 13 tests, 8 docs.)
- **(Correction to the line above:** the findings are numbered **R-1 .. R-31** in the coordinator's order — R-1..R-10
  correctness, R-11..R-23 tests, R-24..R-31 docs; "R-30" was a slip.)

### 15:49 (2026-11-03) — RESUMED: the stopped fix run's state, checked before anything else *(`date`: Tue Nov  3 03:49:36 PM EST 2026)*
- **Why it stopped:** the team lead dropped to ONE job at 93 % of the session limit (~14:50); the old session ended
  with only the "started" heading above written — **no finding was dispositioned in this log**, so every one is
  re-checked below, none taken as done.
- **What the stopped run left (found, not assumed):** `git status` — the A6 set and nothing else (no file the src
  gate's self-test plants into — `robot_config.hpp`, `build_selection_test.cpp`, `test/CMakeLists.txt`,
  `chassis_table_test.cpp`, `lv_obj_style.h`, the `Makefile`, `drive_program.cpp`, `calibrate_program.cpp`,
  `boot_source.hpp` — is modified: no gate plant). Against the reviewers' snapshot (`scratch/mech-a6/review-tests/tree`,
  whose header is byte-equal to the 2026-10-30 final header and to every A6 mutation backup under `mut/`), only
  `bench_r3a.hpp` (+~400 lines) and `bench_r3a.cpp` (+~120) differ — the stopped run's code edits, saved as
  `scratch/mech-a6/fix2/{hpp,cpp}_partial.diff` and READ IN FULL; the tests, the docs and this log untouched since
  12:50 (but the briefing, regenerated). No mutation string in them: the stopped run's edits ARE the partial fixes
  for R-1..R-10 (each reviewed below), no mutant. The reviewers' own mutations ran only in their copy of the tree.
- **The binary trusted for nothing:** removed, the briefing regenerated reading "not built", the build — and then
  the header's timestamp bumped and BOTH test TUs recompiled from source (`fix2_b1`, `fix2_b2`). On that fresh
  binary the stopped run's code failed SIX existing station cases (its announcement changed the check's timing; its
  per-mechanism NEXT reset moved the prompts case's ceiling) — expected: the run stopped before its tests.
- **The stopped run's code, reviewed line by line and KEPT, with four corrections of mine:** (1) a FIRE did not mark
  the position known again after an unknown readback (`fire()` now sets `pneuKnown`); (2) an unknown readback's first
  fire went to `(safe + 1) % n`, not to the safe position its own log line promised (`tick()` now targets `safe`);
  (3) `kMaxPanelLines` was 10, short of a declaration with every role (4 motors + units + 3 angle + 2 home + 2 piece
  switch = 12) — now computed from `kMaxMotorRoles`, and the cpp `static_assert`s its buffer against it; (4) two
  lines that could never run were removed rather than left as equivalent mutants: `safeOne()`'s `pneuKnown = true`
  (only a FIRED pneumatic is ever live, and `fire()` sets it) and `startCheck()`'s `jogDir_ = 0` (A is now refused
  while a jog is live, and no hold-to-run is set without being live — it replaced the A6 mutant MA6-25, which this
  round's N15 succeeds). Also: the HA-186 prompt `  RECORD it at 2 marks: power-cycled, then restarted` was 52
  characters, 2 past the panel's 50 (found by this round's paging test) — now `… power-cycled, restarted`; and the
  RECORD header line (`[R3A] mech REC n: …`, cpp-only) names the new invented number, `announce 1.00 s`.

### MECH-A6 review findings — each re-checked, its disposition *(2026-11-03 15:49–16:20; every test below was seen
RED on its mutant, each mutant on a fresh build of BOTH dependent objects — the mutation table follows)*
**Correctness (R-1..R-10) — every one REAL, reproduced from the reviewer's evidence against the 10-30 header:**
- **R-1 (re-entry: a pneumatic believed at its START).** REAL — `Station()` set `pneuPos = start`; the parts outlive
  the Station. FIXED: `markCleared()` reads each pneumatic's lines back (`commandedLevel`, the DEVICE readback) and
  matches a declared position; one left elsewhere says `'grip' holds 'shut' (an earlier entry left it there), not its
  start 'loose'`; a readback matching none says so (`position: UNKNOWN …`) and its first fire goes to the SAFE
  position. Test: `station 13 re-entry (R-1)` — robot y's grip fired 'shut', made safe, re-entered: the page reads
  HIGH and 'shut', the first FIRE names 'loose' and moves the line; robot x's hood set to an undeclared C-low/D-high,
  re-entered: UNKNOWN, then 'down'. Mutants N1, N1b, N2, N3 red.
- **R-2 (maxVolts ignored).** REAL. FIXED: `voltsFor(i)` = min(ceiling, `power.maxVolts`) for a POWER mechanism (the
  only kind that declares one — `kinds.hpp` :693, (0, 12]); the jog and the check use it; the confirm's foot, the
  gates line, the R1 event and RECORD say it. Test `(R-2)` on a 6 V power mechanism: 3 V at a 3 V ceiling, 6 V at 6,
  9 and 12 (both ways), the check at 6 V; robot two's pivots (12 V) never shown capped. N7–N10 red.
- **R-3 (lines over 255 characters).** REAL (robot two's real table: 8 over). FIXED: every Station log line and
  RECORD line goes through `logPieces()` — at most 255 characters (`kLineChars`; the cpp `static_assert`s it equal to
  `hal::pros::kSerialLineBytes − 1`), broken at spaces, each later piece starting `[R3A] mech CHECK '<name>'
  (cont.) ` / `[R3A] mech REC n '<name>' (cont.) `, so the worksheet's grep keeps every piece and a path stays whole.
  Test `(R-3)` on robot two's REAL table (its host path is LONGER than the brain's `./src/…`): nine verdicts — every
  line ≤ 255 and tagged, each fix whole in its CHECK and REC text, the path whole on one line, some line actually
  split; plus `logPieces` at 255 exactly / 256 / a 300-character word / null. N20–N23 red.
- **R-4 (the ceiling carried across NEXT).** REAL. FIXED: `next()` puts it back to 3 V and logs it. Test `(R-4)`.
  N11 red. The worksheet's "a claw at a time, at 3 V" is now TRUE as written.
- **R-5 (only the last verdict survives).** REAL. FIXED: each `Mech` keeps its latest verdict and answer; RECORD
  prints the mechanism's own; `anyFinding()` drives the menu dot (`setVerdict`) and the exit summary prints every
  mechanism's. Test `(R-5)` (pivot 6 REVERSED then pivot 7 OK: both in RECORD, FAIL; pivot 6 re-checked OK: no
  finding; an abandoned check leaves the last standing). N12–N14 red; the cpp's dot pinned (C4 red).
- **R-6 (L1 let go during a blocking prompt).** REAL — `twoButtonPrompt` waits on a touch only. FIXED in the cpp:
  `mech13::promptHeld()` (twoButtonPrompt's layout; stations 4 and 10's function untouched) polls L1 every tick and
  CLOSES as `LetGo` the moment it is let go (or the controller drops); `confirm(false, false)` then grants nothing
  and makes safe what was powered; `confirm(true, false)` no longer ARMS a motor mechanism either. Tests `(R-6)` and
  the pin; N29, N30, C2, C3 red. Guide 18's "that tick" is now true (within one 10 ms poll).
- **R-7 (a check started mid-jog).** REAL. FIXED two ways: A is REFUSED while the mechanism is live (`REFUSED A: …
  is being jogged`); and the check now starts with a 1 s ANNOUNCEMENT, nothing driven — at its last 0.4 s a settle
  window samples every motor, and one that turned ≥ 0.035 rad by the jog's start is still moving (a coast, a sag): the
  check is abandoned, nothing judged. Test `(R-7, R-30)`: the refusal (the minus jog runs on, never reversed), nothing
  driven t0..t0+0.99 s, the jog at t0+1.00 s exactly, the settle edges (EXACTLY 0.035 rad: abandoned; 1 µrad under:
  the jog; a whole radian BEFORE the window's sample at t0+0.60 s: the jog; 0.035 rad at t0+0.61 s: abandoned). N15–N19,
  N31–N34 red. (A6-D18, A6-D19 below.)
- **R-8 (7 reading lines).** REAL. FIXED: `panelLines()` is asked for `kMaxPanelLines`; `readingsPage()` pages them
  at the cpp's `kShown` (= `Station::kShownLines`, pinned), `shown − 1` a page with a footer `-- page 1/2: TAP the
  readings for more --`; a tap on the lines steps the page (a verdict and the question page the same way). Test
  `(R-8)` on a 4-motor lift with an angle sensor and a home switch: 10 lines, the switch and its question on page 2,
  every line ≤ 50 columns. N24, N25, C5 red.
- **R-9 (the CLEAR's YES).** REAL. FIXED: `clearText()` says `a cylinder MOVES` (and `'<name>' -> '<start>' MOVES`)
  when a pneumatic's start drives a line High on the first CLEAR, and on a later entry (parts made) `nothing moves on
  YES` / `made earlier this power cycle: no line is driven`. Test `(R-9)`; N5, N6 red.
- **R-10 (which angle number absoluteSensor takes).** REAL — `zeroAbsolute()` matches `reads` against the RAW sensor
  (its mounting applied to both sides) and the spec refuses a `reads` outside [0, 360). FIXED: a panel line
  `  absoluteSensor(at, reads): reads = 180.0` (`absoluteReadsDeg()`: the raw reading wrapped into [0, 360), to its
  tenth, never "-0.0") and the same number named in RECORD. Test `(R-10, R-29)` — robot y's wrist (declared
  `absoluteSensor(0, 180)`, Reversed) with its sensor at 180 raw reads 180.0 (the declaration's own number; the
  service homes that very pose correctly in `mechanism_position_test.cpp`), at −90 raw reads 270.0. N26–N26c red.

**Tests (R-11..R-23) — every survivor re-checked against the reviewer's mutant; each now has its test, every mutant
re-cut on THIS header and seen red on a fresh build (R1b..R21 in the table):**
- **R-11** hold-to-run: `(R-11)` — UP let go with L1 held: 0 V, HOLD applied, `jog released -- safe`; DOWN too. R1b red.
- **R-12** the question holds the tick: `(R-12)` — FRESH presses of UP, DOWN and A during a motor check's question,
  and of UP and A during the claw's: nothing driven, still asking, no new fire. The pneumatic case's old comment
  ("presses while the question waits") was a held button, not a press — rewritten as fresh presses. R2 red.
- **R-13** the observation's baselines and the peak: `(R-13)` — the motor at 1.0 rad first (DID NOT MOVE, Δ exactly
  0); robot y's sensor at 1.0 rad first (Δ +0.2 as mounted); 1.0 A for ten ticks then 0 (FIGHT OR JAM, peak 1.0).
  R3, R4, R5 red.
- **R-14** nothing resumes through NEXT: `(R-14)` — `kArmOnly` (NEXT wraps to itself) and NEXT onto an armed pivot 7,
  UP held five ticks: 0 V; a fresh press then moves it. R19 red.
- **R-15** the answer labels: `(R-15)` — each slot's two label lines paired with the finding that slot is judged. R11 red.
- **R-16** any motor, consecutive: `(R-16)` — the lift's SECOND motor (port 3) cut at its 25th tick naming
  `motor 2 (port 3)`; 24 above, one dip, 24 above: not cut, the 25th after the dip is. R6, R7 red.
- **R-17** an at-once verdict counted: every at-once case now checks `verdictCount() == 1`. R21 red.
- **R-18** the temperature's port: `(R-18)` — a stub answering 31 °C for port 6 only: `31C` on pivot 6's panel and
  RECORD, `unknown` on pivot 7's. R13, R14 red.
- **R-19** RECORD's angle, cut and pneumatic lines: `(R-19)`. R15, R17, R18 red.
- **R-20** the read-fault seed: `(R-20)` — the lift's switch and the intake's distance sensor fail, recover, the
  station is RE-ENTERED (`Bench::reenter()`): the first paint reads `released` / `no piece`. R9, R10 red.
- **R-21** "did not move" from below: `(R-21)` — 0.0349 rad at 0.1 A is DID NOT MOVE. R8 red.
- **R-22** the stall window at a small clock: `(R-22)` — `Bench{robotX(), 40}`, the press at 70 ms, the cut at 570 ms
  exactly. R12 red.
- **R-23** the cpp's own safety steps: the last case, `pin (R-23, R-6, R-8, R-5)`, reads `mechanismsStation` and
  `promptHeld` (the `serial_line_test.cpp` :1048 precedent): the guard before the CLEAR prompt, ONE `.parts.build(`
  after it, `markCleared()` after that; the confirm through `promptHeld` and L1 read AFTER it into `confirm()`; never
  the touch-only prompt for a confirm; `kShown = ms::Station::kShownLines`; the panel asked for `kMaxPanelLines` and
  paged; the dot from `anyFinding()`; `promptHeld` polling L1 inside its wait before any touch. Cpp mutants C1–C6 red
  (C6 builds the parts BEFORE the CLEAR under another spelling — the pin counts `.parts.build(` and sees two). The
  stage-1 test's comment now says the Station never builds the parts (the cpp does, pinned); the worksheet's P.3
  heading says which order is pinned by reading the source and that only the bench run proves the page.

**Docs (R-24..R-31):**
- **R-24 (the worksheet's "ONE station that powers").** REAL — my 10-30 and 11-03 passes listed neither :308 nor :1436.
  FIXED: Station D's warning (one of the TWO stations that power: 10 DRIVE and 13 MECHANISMS, both red), the DO NOT
  list (10 and 13 named; a new "do not run 13 MECHANISMS with a hand where a claw can reach"), Station J's programs
  paragraph (station 13 makes 3-wire A at its start level and fires it only on a confirm) and J's banner (their first
  run FROM THE CONTROLLER; Station P is their first powered check — run it first). The grep re-run below.
- **R-25 (HA-184's settle for a piece switch).** REAL. FIXED IN CODE (not by narrowing the documents): a switch piece
  sensor gets the ask line on the panel and its own RECORD line (`piece switch 3-wire E reads N (pressedLevel 1;
  HA-184), read faults N -- RECORD it pressed, released and with its cable OUT`). Test `(R-25)` on robot y's catapult;
  N27, N28 red. The worksheet's P.7.1, guide 18, the changelog and HA-184 (Source and Settle) now say home switch OR
  piece switch — true of the code.
- **R-26 (MECH-COMPLETED.md deferred).** Accepted: A6-D17 is SUPERSEDED. `docs/internal/chunks/MECH-COMPLETED.md`
  written now (BUILT, not landed — the T4 / COMP-b precedent), from this log: the six runs and their commits (A1
  `fd36e07`, A2 `98392b2`, A3 `43ac361`, A4 `46c874c`, A5 `81eb666`), what is proven and where, what has not run
  (nothing on a robot; HA-182..186 and HA-153/159 open; "MECH on hardware" the freeze trigger for F11/F12), the deltas
  index, what is handed on. **For the coordinator, explicitly:** its A6 landing line, the roadmap's MECH rows `[~]` →
  `[x]`, build-order's MECH entry and first bold `Next:`, and REMAINING-WORK row 12 flip AT THE MERGE — main has moved
  since `81eb666` (`eab0f89`, the panel plan, rewrote those files), so they are written against the merged tree, not
  this worktree's. Here they say "A6 BUILT, in review; its record written". The briefing lists `MECH` among the
  completed records from now on (regenerated; it broke the doc gate inside the first mutation run — below).
- **R-27 (register follow-through).** REAL. FIXED: HA-159's "the one program that drives its line" (stale since A5's
  Competition driver control, and now station 13) rewritten; its Settle names Station P (P.5) first, then J.9; its
  index row and HA-153's say "worksheet Station P (Bench Tests' station 13), then Station J"; the A6 status paragraph
  names HA-159 and the HA-189 note it edited.
- **R-28 (two public sentences read as run).** REAL. FIXED: README's library tree ("which Bench Tests' station 13
  MECHANISMS runs on the robot — built, not yet run on one"), guide 10 :556 ("built and not yet run on a robot").
- **R-29 (guide 18's angle sentence).** REAL. FIXED with R-10: guide 18 names the RAW reading as the number
  `absoluteSensor(at, reads)` takes and why "as mounted" is not; the test comment that shared the ambiguity rewritten;
  the worksheet's P.7.2 quotes the new panel line.
- **R-30 ("ONLY '<name>' should move" not said before the jog).** REAL. FIXED IN CODE by R-7's announcement — the line
  is shown for a whole second with nothing driven before the jog (`announceText()` on the panel, with a countdown; the
  gates line `CHECK: watch it -- keep L1 held`) — so the brief's order holds as written; worksheet P.4.4 and guide 18
  describe it.
- **R-31 (status pointers, the index, the counts).** REAL. FIXED: REMAINING-WORK :23 ("A6 BUILT … in its review — its
  fix round 2026-11-03, `MECH-COMPLETED.md` written"); the guide index's chapter-18 row adds the bench page. The
  log's counts, corrected here: `test/bench_mech_station_test.cpp` had **25 cases at 15:05 and 27 at the 10-30 final**
  (the "23 cases" at :4190 was wrong; the grid file holds 12, untouched since that final); the 10-30 FINAL mutation run
  (`mut/results_153025.json`, 36 filtered cases) read **M31 10 failed, M32 4, MA6-20 6** — not the 15:00 run's 9 / 3 /
  5 that :4207/:4216 restate. **Now: 51 station cases, 12 grid cases.**

### Deltas from the brief added by the review round (A6-D18 onward), each with its reason
- **A6-D18 — the direction check ANNOUNCES first:** A shows "ONLY '<name>' should move -- watch it" with NOTHING driven
  for `kAnnounceMs` = 1 s, then the 0.4 s "+" jog; a motor that turned ≥ `kMinMoveRad` in the announcement's last
  0.4 s abandons the check (a coast would be judged as the jog). The brief's "the page says …, then one short jog"
  literally; the 1 s and the reuse of 0.035 rad are INVENTED (HA-182 amended).
- **A6-D19 — A is refused while a jog runs** (a check starts from rest; R-7).
- **A6-D20 — the ceiling is per mechanism SHOWN:** NEXT puts it back to 3 V (gate 4's gentle first motion for every
  mechanism; R-4). Arming still persists per entry.
- **A6-D21 — a POWER mechanism's declared `maxVolts` caps the page** (the service's own rule; §1.14 does not name it;
  R-2). A position or speed mechanism declares no such limit.
- **A6-D22 — the stage-2 confirm watches the dead-man** (`promptHeld`, a copy of `twoButtonPrompt`'s layout — that
  function is stations 4 and 10's and stays untouched; R-6).
- **A6-D23 — a log line longer than 255 characters goes as tagged `(cont.)` pieces** (SER1's one-write rule; R-3).
- **A6-D24 — the readings are PAGED** (A6-D14's "seven reading lines" superseded: seven SHOWN, a footer and a tap
  when there are more; R-8).
- **A6-D25 — each mechanism keeps its own latest verdict this entry** (RECORD, the exit summary, the menu dot; R-5).
- **A6-D26 — a pneumatic's position is read back at each CLEAR** (unknown → its first fire to the safe position; R-1).
- **A6-D27 — the CLEAR says what its YES moves** (R-9). **A6-D28** — the angle sensor's `absoluteSensor(at, reads)`
  line (R-10). **A6-D29** — a piece switch is asked HA-184's three readings (R-25). **A6-D30** — the HA-186 prompt
  shortened to fit 50 columns. **A6-D31** — `MECH-COMPLETED.md` in this change; A6-D17 superseded (R-26).

### The mutations — 92, every one on a fresh build of BOTH dependent objects, every one RED *(16:08–16:18)*
- Driver `scratch/mech-a6/fix2/mutate.py` (one exact replacement that must match ONCE; the file backed up under
  `fix2/mut/`; the binary REMOVED; `cmake --build … --target shulib_tests`; the verdict refused unless the output shows
  `bench_menu_grid_test.cpp.o` AND `bench_mech_station_test.cpp.o` compiled; the station + grid + pin cases under
  `ulimit -v 4194304; timeout 600`; a survivor would get the whole suite; restored in `finally` and on SIGTERM, its
  timestamp bumped). The list is `fix2/muts.json`: the reviewers' R1b–R21 re-cut on this header, this round's N1–N34
  and C1–C6, and the A6 set (`muts_final2.json`) — M32, MA6-6 and MA6-23 re-cut where the header moved, MA6-7/MA6-21
  dropped for their red re-cuts MA6-7b/MA6-21b, MA6-25 retired (its line removed, N15 its successor).
- **Run 1 (`fix2/mutrun1.out`): 56 red, then a BUILD FAILED on everything after N33** — not a mutant's: my own doc
  edits during the run broke the build's doc gates (a roadmap line naming "MECH-COMPLETED" — the removability rule
  refuses `-COMPLETED` in a public doc — and, once that was reworded, the briefing made stale by the new record). The
  driver counts a failed build as NOTHING ("re-cut, not counted"). Stopped by SIGTERM (it restored the header; all four
  files `sha256sum`-equal to `fix2/hashes_before_mut.txt`), the binary removed, a clean build, the briefing
  regenerated, and **run 2 (`fix2/mutrun2.out`, `mut/results_161813.json`) ran the 36 not yet red: all 36 red.**
  Total **92 / 92 RED**, none killed by the cap or the timeout. No doc was edited during run 2.
- The cpp mutants (C1–C6): no host object compiles `bench_r3a.cpp` (the pins READ it at run time), so "recompiled:
  none needed" is the truth for them, and each was seen red by the pin's own assertion.

### The prose review (pace rule 4) — the greps RUN and their hits READ before this entry *(16:00–16:20)*
- **§4.2's A6 grep** (`Bench Tests|station 1[0-3]|3 × 4|twelve stations` over `README.md` and `docs/`, the API pages
  and the PROGRESS/COMPLETED history excluded): 300 hits in 32 files. Read for what THIS round changed — which stations
  power, how the check starts, the ceiling, the readings: the briefs of VIS1 (39), G3a (35), CAL1 (21), IMU1 (17), L6
  (16), T4, SAFE-SAVE, COMP-t, G1, R3d, R3b, R2a, R5a, the session log and ORIENTATION name Bench Tests' stations as
  plans or history — none states what this round changed; the worksheet's (31), the register's (29), build-order's,
  the roadmap's, REMAINING-WORK's, HANDOFF's, the changelog's, guides 07/10/14/15/18/README's and README's read — the
  stale ones FIXED above (R-24, R-28, R-31).
- **The R-24 concept grep** (`read-only|the ONE station|only station|powers motors|powers anything|nothing in shulib
  drives|never drives|DRIVE station commands|…power…`): FIXED the worksheet :308/:311, :1435-1440, :2363 (R-24);
  worksheet :14 already true (A6); LEFT: build-order :663 (R3b Part 0's landing record — history), R3b's brief :135/:373
  (history), R2a's brief (stations 11/12 ARE read-only), L6's :474 (station 11's read-only claim — true), every other
  hit a different subject (a "read-only query", a read-only research pass, the panel's read-only blocks).
- **This round's own facts** — `power-cycled, then restarted` (0 left), `ONLY '…' should move` (guide 18, the
  changelog, the worksheet — the announcement; the brief :453/:2012 — the brief), `nothing moves yet` (the worksheet,
  true), `as mounted` (guide 18, the changelog, HA-186, worksheet P.7.2 — all now name the RAW number; HA :442 another
  sensor), `seven … lines|kShown` (only G3a's other subject), `last verdict|the last check` (0), `at 3 V` (the
  worksheet's Station P/J now TRUE per mechanism; R3b/R3d/T4/D.1.5 other stations), `maxVolts` (guide 18 :52 the
  field table; the new sentences).
- **NOT MINE TO EDIT — for the coordinator:** G3a's brief (in flight, worktree `g3a-p1`) :1647/:1655/:1931 says
  "Station J.9 settles" HA-159 — Station P (P.5) now checks it first (HA-159's Settle, amended here). And, still, the
  IMU1 brief's "the 3 × 4 menu is full" (logged 11-03 13:00).
- **Found while testing, LEFT (pre-existing A6 behaviour, not a finding; for the coordinator):** a distance piece
  sensor's panel line can run past the panel's 50 columns (`piece p9: 393.66in conf 1.00 -> no piece (within 2.00in)`
  is 56): the panel cuts the "(within …)" restatement of the declared threshold; the evidence word is never cut, and
  RECORD carries the line whole. An optical one's `NOT ANSWERING` form reaches 52. Not fixed here: outside the 31
  findings, and it changes text the worksheet quotes.

### Every existing bench station unchanged — re-proven *(16:12)*
- `fn_diff.py` (every top-level definition) — base `81eb666` → now: **48; 43 byte-identical, 5 changed (`MenuItem`,
  `kMenu`, `drawMenu`, `testerInitialize`, `runR3a` — A6's menu and banners, as on 10-30)**, added `mech13`,
  `mechanismsStation`. The review round alone (the 10-30 final cpp → now): **50; 48 identical; changed ONLY `mech13`
  and `mechanismsStation`.** Every function this round touched: in `bench_r3a.hpp`'s `mech_station` — new
  `absoluteReadsDeg`, `logPieces`, `ReadingsPage`, `readingsPage`, `pageFooter`; `Station`'s constants
  (`kLineChars`, `kLineBytes`, `kTextBytes`, `kMaxEvents`, `kShownLines`, `kMaxPanelLines`), `markCleared`,
  `startsHigh`, `clearText`, `next`, `voltsFor`, `capped`, `confirmText`, `confirm`, `tick`, `answer`,
  `verdictPanelText`, `hasVerdictFor`/`verdictFor`/`answeredFor`/`anyFinding`/`anyVerdict`, `exit`, `panelLines`,
  `announceText`, `gatesText`, `record`, `pushLine`/`vEvent`/`event`/`eventCont`, `keepVerdict`, `safeOne`,
  `startCheck`, `stepAnnounce`, `startJog`, `finishCheck`, `abandonCheck`, `fire`, `leaveSelected`, `Mech`'s fields;
  in `bench_r3a.cpp` — `mech13::Held`, `mech13::promptHeld`, a `static_assert`, and `mechanismsStation`. Nothing
  outside station 13 (VIS1-B, L6-B and IMU1 Part B's stations 12, 11 and 2 untouched).

### 16:20–16:55 (2026-11-03) — the first src-gate run FAILED; the fix; two fixed line budgets; ALL mutations again *(`date` 16:55:18)*
- **The host gates first, on the fixed tree:** `build.sh fix2_final1` (binary removed, briefing regenerated, the
  whole build with the doc gates) rc 0; the suite under the cap **2,775 | 2,775 passed, 0 failed, 3 skipped;
  2,943,528 assertions**; `suite_run.py green` the same; the briefing regenerated from that link and `check`ed current.
- **`src_build_gate.py self-test` (16:23) FAILED at its baseline:** `[tank], [tank-drive], [tank-calibrate],
  [tank-autontest], [tank-competition] disallowed warning -Wformat-truncation= at src/programs/bench_r3a.hpp:1541 —
  '%s' directive output may be truncated writing up to 511 bytes into a region of size 256` — `pushLine()`'s
  `snprintf(event_[k], kLineBytes, "%s", line)` fed by `logPieces()`' 512-byte piece buffer (a piece never passes 255,
  but GCC's optimised ARM builds cannot see that; the host build does not warn). Stopped with SIGINT (the self-test
  restores its plants in `finally`/`__exit__`, which SIGINT runs); all 13 changed and planted-into files `sha256sum`-
  equal to `fix2/hashes_before_gate.txt`; `git status` the A6 set only. FIXED: `pushLine()` COPIES at most
  `kLineChars` (`memcpy` + terminator) — no format, no warning. Its mutant N35 (the terminator dropped: a short line
  shows the tail of the longer one its slot held) is caught by a new EXACT check in `(R-11)` (the ARMED line held
  that slot); the `min(…, kLineChars)` bound is unreachable by construction (every piece ≤ 255) — a bound against an
  overrun, not a behaviour, so its only mutant (to `kLineBytes − 1`, the same number) would be equivalent: not cut.
- **Two fixed line budgets found while the gate ran** (the R-3 / R-8 class — a sentence's tail dropped without a
  word): the exit summary wrapped each verdict into a FIXED 10 lines of 52 (`wrapText` stops silently at its limit;
  a 383-character sentence with its prefix is ~440, 9–11 lines) — now `logPieces()` with no cap, pinned (C7 red); and
  the live page's buffer for a wrapped verdict or question, 14 lines of 50 for up to 459 characters — now 20.
- **ALL 94 mutations again on this final header** (`fix2/mutrun3.out`, `mut/results_165512.json`; the 92 above plus
  N35 and C7): **94 / 94 RED**; the 87 header mutants each with BOTH dependent objects recompiled (the driver refuses
  a verdict otherwise), the 7 cpp mutants read at run time by the pin; none killed by the cap or the timeout; the
  four files `sha256sum`-equal after (`fix2/hashes_before_mut3.txt`). No file was edited while it ran.

### RESUMED again (2026-11-05) — what the second stop left, checked before anything else *(`date`: Thu Nov  5 12:22:35 AM EST 2026)*
- **Why:** the 16:55 (2026-11-03) entry above is the stopped run's last; its final src-gate run (`fix2/gate2_check_first.out`,
  17:00, `check` PASS, rc 0, then the `self-test`, whose `fix2/gate2_selftest.out` is EMPTY) was cut while the
  self-test held a plant. Nothing after 16:55 was logged, so the gates below are re-run, none taken from that run.
- **A GATE PLANT WAS LEFT, found and restored:** the `Makefile` (mtime 2026-11-03 17:08:15) held the self-test's case
  for the tank define — `override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_TANK_2026_BROKEN` in place of `…_TANK_2026` (the
  `_replace` at `tools/src_build_gate.py` :2087; the kill skipped its restore). Its copy kept as
  `fix2/Makefile.planted_found_1105`; restored with `git checkout -- Makefile` (HEAD's hash = the pre-gate hash
  `63ce7a46…` in `fix2/hashes_before_gate2.txt`), timestamp bumped. **For the coordinator:** its audit did not list
  the `Makefile` — any tank build from this worktree between 2026-11-03 17:08 and this restore was made with no robot defined (none is known to have been made; the gate below rebuilds every variant from source).
- **Every other file checked:** `sha256sum -c fix2/hashes_before_gate2.txt` (the four A6 code/test files and every file
  the self-test plants into) — ALL OK after the restore; `sha256sum -c fix2/hashes_before_mut3.txt` — the four files
  equal to the tree the 94 mutations ran on (no mutant; their verdicts stand for this source); `git status` (with
  ignored files) — the A6 set, `MECH-COMPLETED.md` and the station test only; no stray include-root folder.
- **The dispositions above (R-1 .. R-31) were all written by the stopped run;** none is redone. Spot-checked in the
  tree: the worksheet's Station D / J / DO NOT sentences (R-24), README and guide 10 (R-28), HA-159 (R-27),
  REMAINING-WORK :23 and the guide index (R-31), `MECH-COMPLETED.md` (87 lines, R-26); the test counts 51 / 12 (R-31).
- **Next:** the binary removed, a fresh whole build, the suite under the cap, every gate on this final tree.

### 00:21–00:58 (2026-11-05) — every gate on the FINAL tree *(`date`: Thu Nov  5 12:58:55 AM EST 2026)*
- **A clean rebuild:** `cmake -S test -B build/test` rc 0; `--target clean`; `build.sh fix3_clean` — the binary removed
  first, the briefing regenerated reading "not built", **246 objects compiled from source**, rc 0, no warning.
- **The suite under the cap** (`ulimit -v 4194304; timeout 600`): **2,775 | 2,775 passed, 0 failed, 3 skipped;
  2,943,529 assertions** (one more than 16:20's 2,943,528: N35's EXACT check in `(R-11)`, added after that run);
  `suite_run.py green`: `green — 2775 cases, 2943529 assertions`.
- **The CI greps:** `library is PROS-free`; `layering holds: core is sim-free`; `PROBE_` in `include/` 0; `PROTOTYPE` /
  `proposed` in the four A6 code/test files 0.
- **The ARM header gate** (CI's exact command; TU `scratch/mech-a6/all_headers_fix3.cpp`): `TU includes 228 headers`,
  `ARM compile gate: CLEAN (all v2 headers)`.
- **The compile-refusal harness:** `ctest -R compile_refusal --no-tests=error` — `100% tests passed … out of 255`;
  255 TUs, 255 CTests.
- **The documentation gates:** `api_doc_tool.py` self-test OK; check-coverage `5060 public entities across 188
  headers, all documented`; check-fresh rc 0 (no `///` under `include/shulib/` changed — `docs/api` needs no
  regeneration); check-examples `600 quoted lines, all verbatim`; check-removability clean; `briefing_status.py`
  self-test OK and `check` current; `doc_staleness_audit.py` self-test OK and `clean (31 live docs…)`.
- **`src_build_gate.py self-test`** (00:29–00:51): `OK (269 detector cases, each against a real build)`, rc 0.
  **`check`** (00:51–00:54): **`src build gate: PASS (all 7 builds link …)`**, rc 0 — bench, xdrive, tank, tank-drive,
  tank-calibrate, tank-autontest, tank-competition each `exit 0, 11/11 TUs compiled, warnings: 8 vendor-ignored, 0
  disallowed`. Output `scratch/mech-a6/fix3_gate_{selftest,check}.out`. **No plant left:** `sha256sum -c
  fix3_hashes_before_gate.txt` (15 files: the four A6 code/test files, every file the self-test plants into, the
  vendored `doctest.h`, `build_selection.hpp`) ALL OK; `git status --ignored` the A6 set only.
- **Rebuilt after the gate** (its restores bump timestamps): `build.sh fix3_final` — 246 objects recompiled, rc 0; the
  suite under the cap **2,775 | 2,775; 2,943,529**, rc 0; the briefing regenerated from THAT link, `check` current;
  the staleness audit, check-fresh, check-examples and check-removability once more — all clean.
- **Every existing bench station unchanged, re-proven** (`fn_diff.py`, base `81eb666` → now): 48 definitions, 43
  byte-identical, changed only `MenuItem`, `kMenu`, `drawMenu`, `testerInitialize`, `runR3a` (A6's menu and banners);
  added `mech13`, `mechanismsStation` — as at 16:12.
- **The mutations are not re-run:** the four files they mutate are `sha256sum`-equal to `fix2/hashes_before_mut3.txt`,
  the tree the 94 / 94 RED run (`mut/results_165512.json`, each header mutant with both dependent objects recompiled)
  ran on; nothing under `src/` or `test/` has changed since.
- **The fix round is complete:** R-1 .. R-31 each dispositioned above (every one REAL and fixed; none declined), the
  deltas A6-D18 .. A6-D31 logged, the prose review logged. Nothing committed. **For the coordinator:** the Makefile plant
  found and restored at this resumption (above); the items marked "NOT MINE TO EDIT" and "Found while testing, LEFT"
  above stand.

### MECH-A6 — Verification-round findings and their disposition *(`date` at this entry: Thu Nov  5 01:56:34 AM EST 2026)*
- A fresh verifier returned ONE finding (minor: R-7's settle check watches EVERY motor, but every settle case drives a
  one-motor mechanism, so its mutant V19 — the check loop cut to `m < 1` — survived the whole suite). Re-checked first,
  on this worktree; the fix, its mutations and every gate follow. Scratch: `scratch/mech-a6/vround/`.

**VR-1 (minor) — the settle window's "every motor" untested past motor 1.** **REAL** (a test gap; the code is right):
- **The re-check.** `bench_r3a.hpp` `stepAnnounce()` samples, then checks, `motors[m]` for every `m` below
  `motors.size()` and `kMaxMotorRoles`. Every settle case in `(R-7, R-30)` moved robot x's ONE-motor arm or robot two's
  one-motor pivot, so nothing failed when motor 2 was ignored — the verifier's V19 SURVIVED (its
  `verify/mutrun.out`). The SAMPLING loop had the same gap (my V19s below): skipping motor 2's sample leaves its stored
  angle at 0, which a motor resting at 0 hides. Robot two (one motor per pivot) is unaffected today; a lift or a
  mirrored pair is not.
- **FIXED IN THE TEST ONLY** (`test/bench_mech_station_test.cpp`, the end of the `(R-7, R-30)` case — no source line
  changed; `bench_r3a.{hpp,cpp}` `sha256sum`-equal to the tree the 94 / 94 RED run used, `fix2/hashes_before_mut3.txt`):
  robot x's LIFT (motors on ports 2 and 3, 3 mirrored), armed, A pressed, and ONLY its second motor (port 3) moved at
  t0 + 1.00 s. Port 3 starts at `kMinMoveRad`, not 0, so a skipped sample reads as a move; the jump goes to
  `2 × kMinMoveRad`, and `REQUIRE(atEdge - start == kMinMoveRad)` proves first that the equality edge lands EXACTLY (a
  start of 2.0 or 0.5 rad would not: `(p + 0.035) - p != 0.035` in doubles for every start tried). EXACTLY
  `kMinMoveRad`: the check abandoned, `CHECK 'lift': still MOVING when its jog was due (motor 2 turned +0.035 rad in
  the last 0.40 s) -- let it settle, then press A again; nothing judged` logged word for word, no line naming motor 1,
  0 V on ports 2 and 3 at every tick, no verdict, port 2 never moved. 1 µrad under: the jog (`Jogging`, volts on the
  lift, no "still MOVING"). The case's BUG CAUGHT line names the gap. Case counts unchanged (51 station / 12 grid);
  the suite **2,775 | 2,775 passed, 3 skipped; 2,943,552 assertions** (+23 over 2,943,529).
- **The mutations — 4 / 4 RED**, each on a fresh build with BOTH dependent objects recompiled (driver
  `vround/mutate.py`, a copy of `fix2/mutate.py` with only its scratch folder changed; backups under `vround/mut/`;
  restored in `finally` and on SIGTERM, timestamps bumped; `vround/mutrun.out`): **V19** (the check loop `m < 1`, the
  verifier's) — red by the NEW assertions (:1653 phase, :1654 volts 6 V, :1664 named); **V19s** (the SAMPLE loop
  `m < 1`) — red by :1664 and the 1 µrad-under case (:1668–:1670: abandoned where the jog was due); **V19e** (`>=` →
  `>`) — red by the old motor-1 edges AND the new motor-2 edge (:1653/:1654, so that edge is exact); **V19n** (the
  message's `m + 1` → `m == 0 ? 1 : m`, which names motor 2 as "motor 1") — red by :1662/:1664. None killed by the cap
  or the timeout. After the run `sha256sum -c vround/hashes_before_mut.txt` — all four files OK.
- **Every gate on the final tree** (`vround/`): `cmake -S test -B build/test` rc 0; `build.sh vround_final` (the binary
  removed first, the briefing regenerated reading "not built"), rc 0; the suite under the cap rc 0 (figures above);
  `suite_run.py green` `green — 2775 cases, 2943552 assertions`; the briefing regenerated from that link, `check`
  current. CI greps: `library is PROS-free`, `layering holds: core is sim-free`, `PROBE_` in `include/` 0,
  `PROTOTYPE` / `proposed` in the four A6 files 0. ARM header gate (CI's exact command, TU `vround/all_headers.cpp`):
  228 headers, CLEAN. `ctest -R compile_refusal` 255 / 255. `api_doc_tool.py` self-test OK, check-coverage 5,060
  entities all documented, check-fresh rc 0 (no `///` changed — `docs/api` needs no regeneration), check-examples 600
  lines verbatim, check-removability clean; `briefing_status.py` self-test OK; `doc_staleness_audit.py` self-test OK and
  clean. **`src_build_gate.py self-test`: `OK (269 detector cases …)`, rc 0; `check`: `src build gate: PASS (all 7
  builds link …)`, rc 0** — bench, xdrive, tank, tank-drive, tank-calibrate, tank-autontest, tank-competition each
  `exit 0, 11/11 TUs compiled, … 0 disallowed`. No plant left: `sha256sum -c vround/hashes_before_gate.txt` (15 files,
  the `Makefile` among them) ALL OK; `git status --ignored` the A6 set only. Rebuilt after the gate (246 objects), the
  suite again 2,775 | 2,775; 2,943,552, rc 0; the briefing regenerated and `check` current; the staleness audit,
  check-fresh, check-examples, check-removability clean once more.
- **Every existing bench station unchanged** (`fn_diff.py` base `81eb666` → now): 48 definitions, 43 byte-identical;
  changed only `MenuItem`, `kMenu`, `drawMenu`, `testerInitialize`, `runR3a`; added `mech13`, `mechanismsStation` — as
  before (no source changed this round).
- **No delta from the brief** (a test only). **The prose review** (pace rule 4): the new test lines and this entry
  grepped for the retired name of the host test plant, `on the robot`, `proven`, `robot claims` — 0 hits; no live doc describes the settle check
  past robot two's pivot (the worksheet's P.4.4 at `R3a-BENCH-WORKSHEET.md` :1364 is about the pivot and stays true),
  so no document changes. Nothing committed.

### The coordinator's merge verification — MECH-A6, the core's LAST run (2026-11-06, main `858b675` + the squash of `mech-a6`)
- **Audit before the merge:** `audit_wt.py mech-a6` — 2 backed-up files, 0 suspicious; the header newer than its backup
  diffed by hand (581 lines of the fix round's code, not a mutant); no "(self-test plant" line in `src/` or the Makefile;
  no stray root file. The transport commit `e70328f`, then `git merge --squash mech-a6` (base `81eb666`).
- **Five conflicts:** the briefing (regenerated); the changelog — A6's entry at the top of `## API 2.5`, dated "2026-11-06
  (built 2026-10-30)"; the guide's index — A6's longer chapter-18 row and A2b's chapter-19 row; the register — A6 added
  NONE (amended HA-182/184/186/153/159/189), so its paragraph ends "Still 203"; build-order's `Next:` line by a word-level
  three-way merge against `81eb666` (both sides present). "A6 BUILT … in its review" phrases made LANDED in build-order,
  REMAINING-WORK (the status and row 12), HANDOFF, the roadmap (its Now paragraph and checklist) and `MECH-COMPLETED.md`
  (written by A6's fix round; its header and A6 row now say landed).
- **Gates on the merged tree** (`.claude/coordinator/logs/mech_a6_merge_verify.out`, re-run 2026-11-06 after a pause):
  host build with the doc gates; suite **3,015 | 3,015 passed, 3 skipped**; `src_build_gate.py` self-test OK and check
  **PASS, all 7 builds**; ARM header gate over **240** headers clean; CI greps empty; doc gates clean; the compile-refusal
  harness **255 of 255**; no plant, no stray root file.
- **The coordinator's hand mutations** (`muts_mech_a6_main.json`, on `src/programs/bench_r3a.hpp`; fresh link, 4 GiB / 300 s
  cap, restored and hash-checked): **6 of 6 caught** — MX1 (a dropped controller's held L1 still counts as held), MX2 (a
  motor mechanism's first volt needs no confirm — gate 2), MX3 (letting go of L1 during a confirm keeps what the station
  powered — A6-F3), MX4 (the volts ceiling starts at 12 V — gate 4), MX5 (a YES armed without L1 held), MX6 (letting go
  of L1, or a dropped controller, leaves powered mechanisms powered) RED. Restored tree green.
- **THE MECH CORE IS COMPLETE** (A1–A6). What follows for mechanisms, by the team lead's order of 2026-11-05: the
  cascade — MECH-S-A (the current cap, building), TUNE1 (live tuning), and the cascade's declaration once its facts arrive.
