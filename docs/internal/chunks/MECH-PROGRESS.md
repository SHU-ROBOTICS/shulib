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
