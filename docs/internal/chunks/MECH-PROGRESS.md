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
