# MECH-S, a string lift's safety — progress log (live)

- **Chunk:** MECH-S, a string lift's SAFETY (brief:
  [`MECH-S-TUNE1-string-lift-safety-live-tuning.md`](MECH-S-TUNE1-string-lift-safety-live-tuning.md)) — the
  current cap (MECH-S-A) and the slack watch (MECH-S-B1/B2), each its own go (the brief's D10, §2.1). TUNE1's runs
  keep their own log.
- **Executor model:** opus.
- Appended as the work goes, a heading per run. Every time in a section header is a `date` reading taken when the
  section was written. **Nothing in this chunk has run on a robot.** Say "host test plant", never "simulator".
  **Exceptions (the fresh verifier, 2026-11-05):** three headers break that rule and say so in the header — 00:40
  (estimated), 15:50 (estimated; the pass's first `date` was 16:09 and its draft `review-fix/log_draft.md` was
  last written at 16:10, so the true time is between about 15:52 and 16:10) and the review pass's gates entry
  (recovered from file times as 17:12, first written as an estimated 17:20). Every header from the verification
  round on is a `date` reading.

---

## MECH-S-A

- **Tree at start:** `ef6162e` (main; the team lead's orderings of 2026-11-03 and the MECH-S/TUNE1 brief answered),
  clean, in the git worktree `.claude/worktrees/mech-s-a` (branch `mech-s-a`) — NOT the main working tree.
- **Go:** the team lead, 2026-11-03: "Accept all, go" on D1–D13; the lift is ROBOT ONE's 4-stage CASCADE, strings
  both ways. The team's dates (2026-11-05): competition NOV 21; the CASCADE first. The coordinator's workflow run
  launched MECH-S-A ONLY (not TUNE1, not MECH-S-B, not the panel). Do not commit; no `pros` command; no
  `/dev/ttyACM*`. Mutation backups and probes under `.claude/coordinator/scratch/mech-s-a/` (never `/tmp`, never the
  repository root). Every test binary under `(ulimit -v 4194304; timeout 600 …)`.
- **Scope (§1.1):** NEW `hal/motor_current_limit.hpp`; CHANGED `hal/pros/motor.hpp`, the PROS shim,
  `hal/fake/fake_motor.hpp`, `hal/device_provider.hpp` + every implementer, `mechanism/devices.hpp`,
  `robot/mechanism_parts.hpp`, `mechanism/spec.hpp`, `mechanism/kinds.hpp`, `mechanism/service.hpp` (slot
  constructors + a once-a-second tick step), `sim/mechanism_plant.hpp`; tests CA1–CA15; documents §3.1's MECH-S-A
  list; HA ids for this run from HA-219..HA-227 (§4).

### 00:22 (2026-11-05, `date`) — start
- Read `docs/internal/RESUMING.md` (standing rules, pace rules 1–8) and, per the team lead's 2026-10-22 reading
  ruling, the brief's header and answers block, "## Re-check — amendments" and its "Continuation" in full, the
  Decisions D1–D12 in full, the plain words, §0.2–§0.4, §1.1 (MECH-S-A), §1.6, §2.0–§2.3 (MECH-S-A's tests CA1–CA15
  and mutations MA1–MA12), §3.1–§3.4, §4, §5, §6 and §7–§8 in full; the other runs' sections by heading only. This log
  created before any code.

### 00:40 (2026-11-05, estimated — no `date` was taken at the time) — the code, first pass
- NEW `include/shulib/hal/motor_current_limit.hpp` — `hal::IMotorCurrentLimit` (`setCurrentLimit` → bool,
  `currentLimit()` read back) and `hal::kDefaultMotorCurrentLimit` (2.5 A). `IMotor` untouched.
- `FakeMotor` and `ProsMotor` implement it; the PROS shim gains `set_current_limit` / `get_current_limit` with
  knobs; `hal/motor_conversion.hpp` gains `motorCurrentLimitToMilliamps` (the ONE A→mA — **delta**: the brief put
  the conversion in `ProsMotor`; the house rule is "convert exactly once, at the edge", in motor_conversion.hpp).
- `IDeviceProvider::currentLimit(port) noexcept` (non-const — the limiter is written through); every implementer:
  `ProsDeviceProvider`, `FakeDeviceProvider` (+ a `bindMotor(port, FakeMotor&)` overload that binds the limiter;
  a bare `IMotor&` binding answers nullptr), `test/calibrate_program_rig.hpp`'s `CountingProvider`, and the FOUR
  test providers the tip's grep found beyond the brief's three: `robot_builder_test.cpp`'s `SilentRefusal`,
  `SilentLine`; `robot_camera_test.cpp`'s `SilentCameraRefusal`, `ThrowingCamera` (landmine 14).
- `mechanism/devices.hpp`: `Devices::currentLimits[kDevicesMaxMotors]` (a local 4, static_asserted against
  `config::kMaxMotorsPerMechanism` in `robot/mechanism_parts.hpp` and `kMaxMotorRoles` in `service.hpp` — devices.hpp
  includes nothing above hal/). `MechanismParts` fills them.
- `spec.hpp`: `MechanismSpec::currentCapAmps` (last member); refusals 71 `ACurrentCapMustBeAbove0AndAtMost25A`, 72
  `AStallSignatureMustBeBelowTheCurrentCap`, checked LAST in `specRefusal`.
- `kinds.hpp`: `struct CurrentCap { double amps; }`, `.currentCap` after `.stall` on Power (before its `maxVolts`),
  Arm, Lift, Flywheel.
- `service.hpp`: the boot cap (Slot's constructor), the once-a-second re-check (a loop at the head of `tickImpl`),
  a bind precondition on `currentLimits`, two library constants (`kCapSignatureMarginAmps` 0.2,
  `kCapRecheckSeconds` 1.0 — HA-223/HA-220).
- `sim/mechanism_plant.hpp`: the ArmPlant limiter (bypassed at the default; bisection on the applied volts so
  |V − kV·ω| at the END speed is inside the limit's volts; a load the limit cannot carry gets the far end's step,
  its reading clamped). **Delta:** the FlywheelPlant has no limiter (no CA test needs one).
- ARM header gate over every header: exit 0.

### 00:55 (2026-11-05, `date`) — tests, documents, the suite green
- **Tests added** (33 new cases at this first green run; a 34th — bind's guard on `currentLimits`, the mutation MX3's net — was added just after it, before the mutations):
  `test/mechanism_current_cap_test.cpp` (NEW: CA2, CA4, CA5, CA7, CA8, CA9 up AND down, CA10 ×3, CA14 ×2,
  CA15, the bind guard), `test/motor_current_limit_test.cpp` (NEW: CA12 — the fake provider, a bound
  FakeMotor vs a bare IMotor, the PROS provider over the shim, the calibrate rig's forwarding),
  `test/mechanism_spec_test.cpp` (CA1–CA3: 9 one-property cases, 8 boundary cases, a peel, numbers 71/72,
  sentences verbatim), `test/sim_mechanism_plant_test.cpp` (CA6 ×5: the ef6162e reversal pinned byte for byte
  — digest `0xeb8a40335b400333` and seven hexfloat samples, taken by the scratch probe `p_reversal.cpp`
  built against ef6162e's plant AND the new one, output `cmp`-identical — the stop reading, the TORQUE, the
  Hold brake, a jam), `test/pros_motor_adapter_test.cpp` (CA11 ×2), `test/fake_motor_test.cpp` (×3),
  `test/motor_conversion_test.cpp` (×1), `test/mechanism_parts_test.cpp` (×2),
  `test/mechanism_chapter_example_test.cpp` (guide-18g), `test/compile_refusals/s71_*`, `s72_*` (CA13).
  `test/mechanism_position_rig.hpp` now wires each rig motor as its mechanism's limit (so every position and
  speed test reads limits at bind, as a robot does) — `Wiring::noCurrentLimits` for the opposite.
- **Measured on the host test plant** (MESSAGE lines): CA8 STALLED uncapped at 3.21 s and capped at 3.21 s
  (the brief's P10: 3.22 s); CA15 the 3 V lift holds at 0.5625 A, and with the 2.5 V piece ends STALLED.
- Suite: `{'cases': 2994, 'failed': 0, 'skipped': 3, 'assertions': 2956421}` (baseline at ef6162e built in
  this worktree: 2961 cases — one of them red only because the spec test's `last` enumerator moved, fixed).
  `ctest -R compile_refusal`: 257/257 (s71, s72 "refused as expected" with their names).
- **Documents:** guide 18 (the member-order table, the common fields, the jam recipe's 2.0 A amended under a
  cap, NEW "Current caps — how hard a string is pulled" after "Jam signatures" — the four currents' ORDER, V1's
  piece-on-a-hold, the string's ceiling (HA-222), the momentum it cannot limit, today's invisible snap, the
  boot/re-check/no-cap lines — and the two new refusal names); guide 11 (the cap's lines and the no-cap Info);
  guide 15 glossary ("current cap"); guide 10 (the signature rule: also below the cap — a stale line found by
  the prose review); `docs/changelog.md` (top of `## API 2.5`); `docs/hardware-assumptions.md` (HA-219..HA-223
  rows + entries, the Held line, the total 203 → 208, HA-183 amended for the limiter); `docs/roadmap.md` (a Now
  paragraph); `docs/internal/REMAINING-WORK.md` row 20e (one clause); the API pages and `mkdocs.yml` (generated:
  one nav line for the new header); `PROJECT-BRIEFING.md` (generated).
- **Deliberately NOT touched:** `build-order.md` (its MECH row :848 lists the chunks, no status; the pointer
  moves are the coordinator's at the commit), `HANDOFF-remaining-work.md` (:685 is dated history), MECH-PANEL
  Q15 and G3a's §6 (the plan commit's, §3.3 — the coordinator's), guide 14 (the slack watch's chapter-14
  sentences are MECH-S-B's, §3.1), `tools/robot_tool.py` (G3a Part 5 adopts `.currentCap` — re-check #1).

### 01:23 (2026-11-05, `date`) — the mutations: 21 run, 21 RED, every file restored byte for byte
- Driver: `.claude/coordinator/scratch/mech-s-a/mutate.py` — one exact-text change per mutant (the anchor must
  occur once), backup under `scratch/mech-s-a/backups/`, the file restored in a `finally` AND on
  SIGTERM/SIGHUP/SIGINT with its mtime bumped, then compared to its backup (all 21 "identical"; `grep -rn MUTANT
  include src test` empty). Each run under `ulimit -v 4194304; timeout 600`. **A first attempt was invalid and is
  not counted:** it built `--target shulib_tests`, which depends on the doc gates — the gates ran the PREVIOUS
  binary and stopped the build (MA1 "BUILD-BROKEN", 0 objects; stopped by me during MA2's build, MA2's file
  restored identical by the signal handler). Re-run on `--target shulib_tests/fast` (the target alone), from a
  binary rebuilt clean first (`rm` then build: 55 objects).
- Every verdict below is a RUNTIME doctest assertion (none is a compile-time trip); every mutant built (none
  re-cut); the objects recompiled per mutant are the dependents of the mutated header (28–67).

| Mutant | Change | Caught by (filter) | Verdict |
|---|---|---|---|
| MA1 | the no-cap Info line dropped | CA4 D12 (b), CA14 1.8 A | RED (2 cases, 5 assertions) |
| MA2 | the cap written to motor 0 only | CA4 | RED — `m12.currentLimit() == 1.2` |
| MA3 | `>=` → `>` in signature-below-cap | spec one-property (the AT-the-cap cases) | RED |
| MA4 | the plant ignores the limit | CA6 ×5, CA7 | RED (6 cases, 14 assertions) |
| MA5 | A → mA × 100 | CA11 | RED — `currentLimitMa == 1200` |
| MA6 | the re-check never re-writes | CA10 | RED |
| MA7 | a new refusal inserted mid-order | the numbers test (70/71/72) | RED |
| MA8 | the boot line's total not × motors | CA4 (the line verbatim) | RED |
| MA9 | a no-cap motor written every boot | CA4, CA14 (write counts) | RED (3 cases) |
| MA10 | the re-check re-writes ANY changed value | CA10's 1.8 A case | RED |
| MA11 | the plant limiter active at the default | CA6's ef6162e reversal (digest + samples) | RED (34 assertions) |
| MA12 | the within-0.2 A Warn dropped | CA2, CA15 | RED |
| MX1 | the plant clamps the reading, not the torque | CA6 (the torque case, the pushed volts) | RED (3 of 5) |
| MX2 | the boot "reads back" the request's echo | CA5 | RED |
| MX3 | bind's current-limit guard removed | the bind-guard case | RED |
| MX4 | the fake provider answers a motor never served | CA12 | RED (4 cases) |
| MX5 | `ProsMotor::currentLimit` echoes, never reads | CA11 | RED |
| MX6 | the parts give every motor motor 1's limit | the parts case | RED |
| MX7 | the re-check only for ACTIVE slots | CA10 ×3 | RED |
| MX8 | the cap's range check lets a NaN through | spec one-property (the NaN cap) | RED |
| MX9 | the 0.2 A edge moved PAST the tolerance (`< 0.2 − 1e-9`) — relabelled by the review: NOT the plain exclusive edge, which the 1e-9 tolerance made equivalent; superseded by E1–E3 (whole mA) in the review section | CA2's exact-0.2 case | RED |

### 01:23 (2026-11-05, `date` when the writing began) — every delta from the brief (against ef6162e), with its reason
1. **The log's name** is `MECH-S-PROGRESS.md` (the coordinator's task), not the brief's `MECHS-PROGRESS.md`.
2. **The A→mA conversion lives in `hal/motor_conversion.hpp`** (`motorCurrentLimitToMilliamps`, beside HA-97's
   mA→A), not inside `ProsMotor`: the file's rule is "convert exactly once, at the edge … the adapter CALLS these".
   Its own test pins 1.2 A → 1200 mA and the rounding.
3. **`IDeviceProvider::currentLimit(port)` is `noexcept`** (readFaults' precedent: a lookup that never makes).
4. **Seven implementers, not three:** the tip's grep found four more test providers (`robot_builder_test.cpp`'s
   `SilentRefusal`, `SilentLine`; `robot_camera_test.cpp`'s `SilentCameraRefusal`, `ThrowingCamera`) — each forwards.
5. **`FakeDeviceProvider::bindMotor(port, FakeMotor&)`** — a NEW overload, so a bound fake answers its limit; a bare
   `IMotor&` binding answers nullptr (never a cast). The brief did not say how a bound motor answers.
6. **`mechanism::kDevicesMaxMotors`** (4) in `devices.hpp` — that header includes nothing above hal/; two
   static_asserts tie it to `config::kMaxMotorsPerMechanism` and `kMaxMotorRoles`.
7. **`MechanismSpec::currentCapAmps` is the LAST member** (no positional aggregate of it exists; a generated file
   that writes one by order stays valid).
8. **The bind precondition** (a limit only for a built motor mechanism's own motors) — NEW, not in §1.1; MX3 is its
   mutant.
9. **When a motor will not take its cap, the boot prints NO "set and read back" line** — only the per-motor Warns
   (the Info line would otherwise claim what is false). §1.1 said "One Info line per capped mechanism" and "a Warn
   when the motor would not take it".
10. **The 0.2 A margin's edge is INCLUSIVE** (a signature exactly 0.2 A under the cap boots with the Warn): D2's
    ordering is strict ("signature < cap − 0.2 A"), so exactly 0.2 under breaks it. CA2 named 0.19 / 0.21 only;
    the test lands on the exact edge too (1.2 − 1.0, with a 1e-9 tolerance for the binary arithmetic), and MX9
    (the edge made exclusive) turns it red. **[Superseded by the review — see "MECH-S-A review findings": 1.2 − 1.0
    is 0.19999999999999996, not the edge, and the plain exclusive mutant survived; the margin is now judged in
    whole milliamps, exact, with a second edge pair 2.5/2.3.]** CA15's V1 lift (1.0 under 1.2) therefore boots with the Warn — pinned.
11. **The re-check's state per motor:** a motor whose BOOT write left it at the default is re-written each second
    silently (its boot Warn said it); a re-write that reads back the cap clears the state, so a LATER loss Warns
    again — one Warn per loss (tested: two losses, two lines).
12. **No "`.slack` lift with no cap" Warn** — `.slack` does not exist until MECH-S-B1 (§1.1 says "after MECH-S-B1").
13. **D5 and the team's cascade (strings both ways, homed by a gentle stall DOWN into its bottom stop):** the cap is
    the MOTOR's, so it protects a homing push in EITHER direction — CA9 is extended with a −6 V stall-home DOWN
    into the bottom stop under a 1.0 A cap (uncapped it pushes 1.25 A; capped ≤ 1.00 A after contact; SUCCEEDED,
    homed). Nothing in MECH-S-A refuses a downward stall-home. **For the coordinator:** D5 (a)'s refusal
    ("homeByStall pushing DOWN on a lift that declares `.slack`", MECH-S-B1) was written for a gravity-return lift
    (P5: the spool pays out string past the bottom); on a both-ways lift the down-string carries the push, so the
    refusal would forbid the team's own homing — the answers block already says D5 is re-checked before
    MECH-S-B's go; this run confirms the cap half needs no change.
14. **The host test plant:** the FlywheelPlant has NO limiter (no CA case needs one); a load the limit cannot carry
    even back-driven gets the −12 V (or +12 V) end's step with its READING clamped at the limit (the plant has no
    mass to fall with — said in its banner and in HA-183); a jammed joint and the Hold brake at 0 V are limited too.
15. **CA14 through the fake provider**, against a twin whose provider answers no limit (each claw motor bound as a
    bare `IMotor`) — every sink entry equal; the PROS-shim pins of robot two's programs (robot_two_driver_test,
    the competition and tester rigs) re-ran green unchanged with the shim's 2500 mA default.
16. **CA8's timing:** the test asserts the capped STALLED time within 0.25 s of the uncapped one (measured 3.21 s
    both; P10's 3.22 s), not an equality — the cap could legitimately slow the approach.
17. **The guide's example lift is `stringLift` / "string lift"**, compiled verbatim as `guide-18g` in
    `test/mechanism_chapter_example_test.cpp` (a file §1.1 does not list) — the check-examples gate requires it.
18. **`test/mechanism_position_rig.hpp` wires each rig motor as its mechanism's limit by default** (a file §1.1
    does not list): so every existing position and speed test reads limits at bind as a robot does, and the
    whole suite re-ran green — the no-cap read path is exercised everywhere.
19. **Public documents: six** (guide 18, 11, 10, 15, the changelog, the HA register) plus the roadmap's Now —
    rule 7's "about five": guide 10's signature line was a stale fact otherwise (found by the prose review).
20. **The HA register:** the Held line's MECH-S/TUNE1 range shrinks to HA-224..HA-227; the total 203 → 208; HA-183
    amended by one sentence (the plant's limiter).

### 01:23 (2026-11-05, `date` when the writing began) — the prose review (rule 4), run before this entry was written
- §3.4's greps: `grep -rn -i simulator docs/guide include/shulib/{mechanism,hal,sim}` — 10 hits, every one
  pre-existing in guides 05/07/08/13/14 (none in a file this run touched; `git diff | grep -i simulator` empty)
  **[WRONG as logged — the review re-ran it: 14 hits; corrected in the review section below]**;
  `grep -rn 'internal/\|chunks/\|RESUMING' docs/guide docs/cookbook README.md` — empty;
  `grep -rnoE 'HA-(219|22[0-7])' include src test` — HA-219..HA-223 each with code sites (motor_current_limit.hpp,
  pros/motor.hpp, motor_conversion.hpp, kinds.hpp, spec.hpp, service.hpp, mechanism_plant.hpp, fake_motor.hpp, the
  shim, the tests), HA-224..HA-227 none (held, not registered). Every new log line's format in guide 11 equals
  the code's (each line pinned verbatim in `mechanism_current_cap_test.cpp`).
- Per changed file, the docs and README grepped for its name and its concepts ("five screened",
  `faultedReads`, `IDeviceProvider`, "current limit", "2500 mA", "2.5 A", `mechanism_plant`, `ArmPlant`,
  `FakeMotor`, `motor_conversion`, `Devices`): hits read — guide 10 :605 (the signature rule: AMENDED — also
  below a stated cap); guide 12 :251 and guide 17 :396 (a DRIVE motor's 2.5 A default — still true, untouched);
  faq :22 and the master plan :583 (the generic screened-read idiom — still true); guide 13 :233
  (`motor_conversion.hpp and friends` — still true); HA-183 (the plant's current model: AMENDED);
  `docs/vexbuilder-integration.md` :64 and roadmap :307/:2016 (dated history of `IDeviceProvider` — untouched).

### 01:42 (2026-11-05, `date`) — every gate, on the final tree (after the mutations, from a clean rebuild)
- After the mutations: `bind()`'s doc comment amended (a stated cap is WRITTEN at bind — "Nothing is commanded"
  was no longer the whole truth; its preconditions list gains the limits' guard); API pages regenerated; the
  binary REMOVED and rebuilt (`shulib_tests/fast`), THEN `briefing_status.py generate`, then the full build with
  the doc gates (exit 0).
- `python3 tools/suite_run.py green` → `suite run: green — 2995 cases, 2956439 assertions` (3 skipped).
- `ctest -R compile_refusal` → `100% tests passed, 0 tests failed out of 257`.
- ARM header gate (every `include/shulib` header, 241, one TU, `-Werror`) → exit 0.
- CI greps (pros/ outside hal/pros; shulib/sim/ outside sim) → empty. F4 (`hal/motor.hpp`), F6/F10
  (`chassis.hpp`, `routine.hpp`) diffs → empty. `git diff --stat -- src/ Makefile` → empty; no probe name leaked.
- `src_build_gate.py self-test` → `OK (269 detector cases, each against a real build)`; `check` → `PASS (all 7
  builds link …)`; `git diff -- Makefile src/ | grep -c 'self-test plant'` → 0.
- `api_doc_tool.py` self-test OK; check-coverage "5528 public entities across 198 headers, all documented";
  check-fresh exit 0; check-examples "651 quoted lines, all verbatim (12 source files)"; check-removability "no
  public doc references docs/internal/".
- `doc_staleness_audit.py` → clean; `briefing_status.py check` → "generated block current".
- **For the coordinator:** files another in-flight chunk also edits — `test/calibrate_program_rig.hpp` (CAL1-B
  re-reads it: one forwarding member), `docs/hardware-assumptions.md`, `docs/changelog.md`, `mkdocs.yml` (one
  generated nav line), `docs/roadmap.md`, `docs/internal/REMAINING-WORK.md` (one clause), and
  `mechanism/kinds.hpp` (AF1-A's `Arm::pieceSensor` after `.angleSensor`; this run's `.currentCap` after
  `.stall` — the Arm's member order at the tip must be re-read by whichever lands second). `hal/device_provider.hpp`
  gained a pure virtual: an implementer added by L6-A2 or VIS1-A2 must answer `currentLimit` (landmine 14). No
  file of MECH-A6, IMU1-B, L6-A2 or VIS1-A2 was touched. **Nothing here has run on a robot.**
- **MECH-S-A is BUILT, not closed:** three reviewers, fixes, a fresh verifier and the coordinator's checks follow;
  no `-COMPLETED.md` and no pointer move until then.

### 15:50 (2026-11-05, estimated — the first `date` of this pass was taken at 16:09) — MECH-S-A review findings and their disposition
- Three reviewers (correctness, tests, docs): 20 findings, deduped to 18 below (the correctness lens's
  whole-milliamp finding and the tests lens's half-milliamp finding are one; the correctness lens's
  written-0 finding and the docs lens's are one). Each re-checked against the tree before it was fixed.
- **A build lesson found on the way (for the coordinator):** after this pass's re-configure
  (`cmake -S test -B build/test`), the `shulib_tests/fast` build recompiled only the 8 objects whose `.cpp`
  had changed — `CMakeFiles/shulib_tests.dir/compiler_depend.make` was 2 lines long (a `/fast` target skips
  CMake's depend step, which is what fills it). A test failing for a change I HAD made (the shim case below
  still read the old comparison) caught it. Fixed by running the target's depend step
  (`make -f CMakeFiles/shulib_tests.dir/build.make CMakeFiles/shulib_tests.dir/depend`: 74 337 lines), then a
  rebuild: 255 objects. This pass's mutation driver (`scratch/mech-s-a/review-fix/mutate2.py`) therefore
  checks, for every mutant, that EVERY object `compiler_depend.make` lists as depending on the mutated file
  was recompiled, and refuses a verdict otherwise. The first pass's 21 verdicts recorded 28–67 recompiled
  objects each (the dependency file was whole then); they stand.

| # | Finding (lens) | Re-check | Disposition |
|---|---|---|---|
| 1 | A cap on a half-milliamp edge is misjudged: 2.4995 A warned as two equal numbers and re-written every second; 1.2005 A warned and never re-checked; 0.0004 A sent as 0 mA and reported "set and read back" (correctness; the tests lens's 1.2345 A finding is the same) | REAL: the service compared doubles to ±0.5 mA, ProsMotor sends `lround(A·1000)` | **FIXED at the root:** every comparison the service makes is in the DEVICE's unit, whole milliamps — the cap as `hal::motorCurrentLimitToMilliamps` sends it (the one A→mA; `service.hpp` now includes `motor_conversion.hpp`), a read-back rounded to the mA (a number no motor reports is −1); `sameAmps` is gone. A cap above 0 but under 0.001 A is refused at build by a NEW appended refusal **73**, `ACurrentCapMustBeAtLeastOneMilliamp` (`a_current_cap_must_be_at_least_0_001_A_a_motor_holds_whole_milliamps`), checked after 71 and before 72 (the cap's own fault first); the preconditions of `motorCurrentLimitToMilliamps`, `FakeMotor::setCurrentLimit` and the interface's contract move to "from 0.001 A". Tests: over the PROS shim, 2.4995/1.2005/1.2345 A each sent as 2500/1201/1235 mA, "set and read back" at Info, no Warn, ONE write each in ten seconds, the 1201 mA one re-written after a power loss; spec cases 0.0009, 0.0004, the smallest positive double refused, 0.001 exactly accepted; the peel (73 before 72); s73 compile refusal; the conversion's and the fake's backstops (0.0004, 0.0009). |
| 2 | A written `.currentCap = {.amps = 0}` (or −0.0) is silently "no cap", but guide 18, the changelog, the roadmap, the spec test's banner and the brief's CA1 say the build refuses a cap at or below 0 (correctness; docs) | REAL — a conflict inside the brief: §1.1 defines `CurrentCap{amps = 0.0}`, "0 = none stated"; CA1 says "refuses a written cap ≤ 0" | **FIXED in the words, delta 21:** 0 means none stated — the library's house rule for every optional number in a declaration (a `StallSpec` of three zeros is "no signature" too; a sentinel default would make the cap the only field that differs, and `CurrentCap` would carry a second representation G3a's generator must mirror). Guide 18, the changelog, the roadmap, `kinds.hpp`'s and `spec.hpp`'s comments and the spec test's banner now say "below 0 … a cap written as 0 is no cap"; a spec case pins −0.0 as accepted (no cap). |
| 3 | A program that commands mechanisms without binding the service never applies a stated cap — MECH-A6's station 13 is one (correctness) | REAL as a coordination point; nothing in MECH-S-A's tree is wrong (every program at the base that builds mechanisms binds the service: Competition, Drive, Auton Tester) | **No change here; for the coordinator** (below): whichever of MECH-A6 and MECH-S-A lands second must make station 13 write a stated `.currentCap` before it commands a capped mechanism, or say on its page that it runs uncapped. |
| 4 | Nothing tests that an Arm's `.currentCap` reaches its spec; deleting that line survives (tests) | REAL (`grep -rn currentCap test/` had no Arm) | **FIXED:** a case over all FOUR kinds (an Arm, a Lift, a Power, a Flywheel, each a distinct cap): `spec().currentCapAmps`, the spec accepted, and the same spec with its signature AT the cap refused by 72 — and an Arm bound through the rig: its motor capped, its boot line verbatim. K1 (the Arm's copy deleted) now RED. |
| 5 | MX9 does not prove the inclusive 0.2 A edge; the plain exclusive mutant and removing the tolerance both survive (tests) | REAL: 1.2 − 1.0 = 0.19999999999999996 is inside the margin either way. (The reviewer's S1, `<=` → `<` with the 1e-9 kept, is in fact EQUIVALENT under that tolerance — no pair of typed amps lands within 1e-9 above 0.2 — and a 2.5/2.3 pair alone would not have separated it.) | **FIXED at the root:** the margin is judged in whole milliamps (`capMa − sigMa <= 200`), exact; a second edge lift, cap 2.5 / signature 2.3 (0.20000000000000018 in doubles), boots with the Warn beside the 1.2/1.0 one. E1 (`<=` → `<`), E2 (amps, no tolerance) and E3 (amps, exclusive) each RED; MX9 relabelled in the first pass's table ("edge moved past the tolerance", superseded). |
| 6 | Nothing pins the re-check to once a second: every tick (R4) or 1.1 s (R3) survives; CA10's "lost just AFTER a check" comment is wrong (tests) | REAL | **FIXED:** `FakeMotor` counts `currentLimit()` reads (`currentLimitReads()`, its own test); a new CA10 case records the ticks on which the service read the capped motor over ten seconds — ONE read per check, the first at tick 99–100, each next 100–101 ticks later (the 10 ms clock's sum) — then loses the limit on the tick right after a check: no read and no write 0.98 s later, re-written by 1.01 s. CA10's comment corrected (the second loss is ~0.36 s after a check). R4, R3 and FR1 (reads not counted) RED. |
| 7 | A motor that keeps refusing its cap has no test: a Warn every second survives (tests) | REAL | **FIXED:** CA5's refused subcase paces 5 s — still ONE line, 5–6 writes (the boot's and one per check), still 2.5 A — then the port accepts: capped within a check, no new line; a LATER loss is one new Warn. R1 (`if (true)`) RED. |
| 8 | (the tests lens's half-milliamp finding) | the same as #1 | FIXED with #1 (1.2345 A is the shim case's third motor). |
| 9 | The plant's reading clamp in `publish()` is never exercised (tests) | REAL | **FIXED:** a plant case — a 6 V load held by 3 V under a 0.3 A limit, where even −12 V leaves 5.7 V of torque: every sample reads exactly the limit, applies −12 V, and falls at the −12 V step's speed; uncapped the same reads its true 1.19 A. P1 (the clamp removed) RED. |
| 10 | Guide 18 and HA-223 say "at least 0.2 A below", but the boot warns at exactly 0.2 A (docs) | REAL | **FIXED:** "MORE than 0.2 A below" in guide 18 (the jam recipe and the four-currents bullet: "exactly 0.2 A below warns too"), guide 11, HA-223's row and entry, `kinds.hpp`'s comment and the changelog. |
| 11 | Guide 18's V1 example says "held … without a word", but that configuration boots with the margin Warn; its "~1.1 A" is not this run's evidence (docs) | REAL — and the number was WRONG for this tree: pinned, the hold with the piece rises to the CAP, 1.2 A (a capped motor stalls at its cap; the brief's V1 probe read 1.08 A on its own plant) | **FIXED:** CA15 now records the most the hold drew with the piece before the trip and CHECKs it above the 1.0 A signature and at or under the 1.2 A cap (MESSAGE: 1.2 A); the guide says the boot warned, the hold sat at 0.56 A (CA15's measured 0.5625 A) and the piece raised it "to the cap's 1.2 A". |
| 12 | HA-154 still says nothing in shulib changes the claw pivot's 2500 mA; the prose review did not record the register's "2500 mA" hits (docs) | REAL | **FIXED:** HA-154's row and entry: unchanged while the pivot states no `.currentCap` (robot two's state none); since MECH-S-A a stated cap lowers it (HA-219). The register hits are recorded in the prose review below. |
| 13 | Guide 18's "honest limit" still says a starved service leaves a mechanism "not limited" (docs) | REAL | **FIXED:** one clause — except a stated `.currentCap`, which the motor itself keeps (believed, not measured — HA-219), linked to the section. |
| 14 | The logged `simulator` grep (10 hits) does not match a re-run (14) (docs) | REAL — my count was wrong | **FIXED:** re-run and recorded below; the original line is marked wrong in place. Still no NEW hit. |
| 15 | REMAINING-WORK row 20e, edited by this run, still says an output rotation sensor is "optional and stronger" (D6 found it false) and "not sized" (docs) | REAL; §3.3 gives both corrections to the PLAN commit (REMAINING-WORK :110), and the coordinator's rule for shared plan docs is "touch only your own lines" | **Flagged for the coordinator** (below), not edited: my clause stays the only change to that row. |
| 16 | Guide 18's recipe says to measure the hold and the peak but gives no way before the Tune program (docs) | REAL | **FIXED:** "Where the hold and the peak come from" — no page of the library shows a mechanism motor's current yet; live tuning (a later part) will; until then a PROS program of the team's own can print `get_current_draw()` (mA), or start from the order and the boot's warnings and treat the numbers as unmeasured. |
| 17 | The plant's reversal inaccuracy is in no guide chapter though §1.1 says "the guide's chapter on the plant says so" (docs) | REAL: no guide chapter describes the plant's internals (`grep -rn 'ArmPlant\|mechanism_plant\|reversal' docs/guide` — none) | **Delta 22 (logged, not a guide edit):** the plant's banner and HA-183's amendment carry it; a student never meets the plant's reversal, and inventing a plant chapter is outside MECH-S-A's documents (§3.1). |
| 18 | Guide 11 states as fact "the motor keeps its cap meanwhile", "a loss over and over is a loose cable or a brown-out", and "will push with less than its full 2.5 A" (docs) | REAL | **FIXED:** each hedged with its register id (HA-219, HA-220, HA-219); the glossary's "enforced by the motors themselves" too. |
| 19 | The glossary entry is out of order; guide 18's opening status box does not mention caps; delta 20 gives no reason for the HA split (docs) | REAL | **FIXED:** the entry moved after "cross-track error"; the box names current caps and what is still to come (the bench page, the snapped-string watch, live tuning); delta 20's reason added below. |

- **Deltas from the brief added by this pass** (numbered after the first pass's 20):
  21. **A cap WRITTEN as 0 (or −0.0) is no cap**, never refused — §1.1's `CurrentCap{amps = 0.0}`, "0 = none
      stated", over CA1's "refuses a written cap ≤ 0" (the two cannot both hold without a sentinel; the house
      rule for optional numbers decided it). Negative caps are refused (71).
  22. **The plant's reversal inaccuracy is said in the plant's banner and HA-183, in no guide chapter** (§1.1
      said "the guide's chapter on the plant" — there is none).
  23. **A NEW refusal, 73, `ACurrentCapMustBeAtLeastOneMilliamp`** (not in the brief's §1.1): a V5 limit is
      whole milliamps, so a cap under 0.001 A would be sent as 0 mA. Appended after 72, checked between 71
      and 72. The interface's, the conversion's and the fake's preconditions move to "from 0.001 A".
  24. **Every cap comparison is in whole milliamps** (§1.1 did not say how a read-back is compared): the cap as
      `motorCurrentLimitToMilliamps` sends it, the read-back rounded to the mA, the 0.2 A margin as 200 mA —
      inclusive. **Corrected by the verification round (below):** the signature is NOT rounded to the nearest
      mA (it is never sent) — it is the first whole-mA reading that trips it (`tripMilliamps`); only then is
      the 200 mA edge exact for every signature. `service.hpp` now includes `hal/motor_conversion.hpp`.
  25. **`FakeMotor::currentLimitReads()`** — a read counter, so a test can pin the re-check's pace.
  - **Delta 20's reason (the HA split, asked by the review):** §4 reserves HA-219..HA-227 for the chunk
    without assigning them per run. MECH-S-A registered HA-219..HA-223 because each is cited by its own
    decisions — HA-219 (the motor enforces the limit) by D1 (a), HA-220 (a limit outlives a program / a
    power loss resets it) and HA-221 (VEXos's sharing) by D12 (b) and the re-check, HA-222 (pull per amp,
    momentum) by D2's recipe, HA-223 (the margin) by D2 (a)'s boot Warn — and none of HA-224..HA-227 is
    cited by anything but the slack watch (MECH-S-B) and `restsAtStop`'s check, which are not this run's.

### 16:28 (2026-11-05, `date` at the end of the run) — the review's mutations: 17 run, 17 RED, every file restored byte for byte
- Driver `.claude/coordinator/scratch/mech-s-a/review-fix/mutate2.py` (one exact-text change each, the anchor
  unique; backup under `review-fix/backups/`; restored in a `finally` and on SIGTERM/SIGHUP/SIGINT, mtime
  bumped, then `cmp`-identical to its backup — all 17 "identical"; `grep -rn MUTANT include src test` empty).
  Each mutant: the WHOLE suite binary (no filter) under `ulimit -v 4194304; timeout 600`; a verdict only
  when every object `compiler_depend.make` lists as depending on the mutated file was recompiled (the count
  in brackets is recompiled / dependents). None broke its build (none re-cut); every verdict is a RUNTIME
  doctest assertion — the one compile-time net, s73, was checked separately (below). The failing cases were
  read from each run log (`review-fix/redcases.py`).

| Mutant | Change | Caught by | Verdict |
|---|---|---|---|
| U1 | the boot compares doubles again (±0.5 mA) | the PROS-shim whole-mA case | RED (55/55) |
| U2 | the re-check compares doubles again | the PROS-shim case (re-written every second) | RED (55/55) |
| U3 | refusal 73 removed | spec one-property (0.0009, 0.0004, denorm) and the peel | RED (66/66) |
| U4 | 73's floor at half a mA (`< 0.0005`) | spec one-property (0.0009 accepted) and the peel | RED (66/66) |
| U5 | the conversion's floor back to `> 0` | the conversion test (0.0004, 0.0009 backstops) | RED (77/67) |
| E1 | the margin `<=` → `<` (the real exclusive edge, in mA) | CA2 (m020 and m2523) and CA15's pinned Warn | RED (67/55) |
| E2 | the margin in amps with no tolerance | CA2 (m2523: 0.20000000000000018 > 0.2) | RED (55/55) |
| E3 | the margin in amps, exclusive | CA2 (m020: the edge itself) | RED (55/55) |
| R4 | the re-check every tick (`t < nextCapCheck_` dropped) | the PACE case; CA5's refusing motor (a write every tick) | RED (55/55) |
| R3 | `kCapRecheckSeconds` 1.0 → 1.1 | the PACE case (gaps 110); the shim case's re-write in 1.02 s | RED (55/55) |
| R1 | a Warn on every re-check of a lost limit | CA5's refusing motor (one line in 5 s) | RED (55/55) |
| FR1 | FakeMotor's reads not counted | its own read-counter case; the PACE case | RED (120/112) |
| K1 | Arm::spec() drops `.currentCap` (the reviewer's survivor) | the four-kinds case | RED (128/66) |
| P1 | the plant's reading clamp removed (the reviewer's survivor) | the unreachable-load plant case | RED (67/12) |
| MA6r | the re-check never re-writes (MA6 re-cut: its line changed) | CA10 ×3, CA5, the shim case | RED (56/55) |
| MA9r | a no-cap motor written at boot (MA9 re-cut) | CA4 ×2, CA14 ×2 | RED (55/55) |
| MX2r | the boot "reads back" the echo (MX2 re-cut) | CA5 | RED (55/55) |

- The first pass's MA6, MA9 and MX2 anchors no longer exist (their lines were rewritten by the whole-mA fix):
  re-cut on the new code as MA6r, MA9r, MX2r, each RED. MX9's anchor is gone too — E1–E3 replace it.
  The reviewer's S1 (`<=` → `<` with the 1e-9 kept) is no longer expressible: the tolerance is gone.
- **The compile-time net (s73):** with refusal 73's check removed (backup `review-fix/backups/U3ct__spec.hpp`,
  trap-restored, `cmp` identical), `ctest -R s73` FAILED with the judge's own verdict — "COMPILED: …s73_…
  was NOT refused; it must be refused naming: ['a_current_cap_must_be_at_least_0_001_A_a_motor_holds_whole_milliamps']"
  — the test's assertion, not a stray compile error.

### 16:45 (2026-11-05, `date`) — the prose review for the review's changes (rule 4), run before this entry
- §3.4's greps re-run on this tree: `grep -rn -i simulator docs/guide include/shulib/{mechanism,hal,sim}` —
  **14 hits** (the first pass logged 10, wrongly): guides 05 :81, 07 :114/:194, 08 :404, 09 :91, 13 :45/:143,
  14 :289/:307/:316/:539, and the three sim banners `mechanism_plant.hpp:7`, `field_plant.hpp:4`,
  `camera_plant.hpp:4` (each a "never simulator" rule). All present at `ef6162e`; `git diff | grep '^+.*simulator'`
  empty — none new. `grep -rn 'internal/\|chunks/\|RESUMING' docs/guide docs/cookbook README.md` — empty.
- The changed concepts grepped across the public docs and the headers: "at least 0.2 A" / "at least ~0.2" —
  none left; "cap at or below" / "(0, 2.5]" / "above 0 and at most 2.5" — left only where true (refusal 71's
  own sentence and enum comment: a WRITTEN, non-zero cap; `spec.hpp`'s comment; the spec test's comment);
  "within 0.2 A" — guide 11's pinned log line and `service.hpp`/`spec.hpp` comments (inclusive: true);
  `a_current_cap_must` — guide 18 and the changelog list all three names.
- **The register's "2500 mA" hits, read and recorded** (the review's finding 12): `hardware-assumptions.md`
  :215 (MECH-S-A's own block), :402 and :2612 (HA-154 — AMENDED), :416 (HA-172) and :3860 (its entry), :420
  and :3444 (HA-176) — each about the DRIVE motors' limit, which the service never touches (only mechanism
  motors are capped): still true, untouched; :457–:459 and :1784/:1792/:1794/:1809/:1815 (HA-219..HA-221 —
  this run's own); :2633 (HA-154's source quote — true).
- Guide 18's new paragraphs read against the code: the V1 numbers (0.5625 A hold, the 1.2 A with the piece)
  are CA15's MESSAGE lines of this tree; "exactly 0.2 A below warns too" is CA2's m020/m2523; "under 0.001 A"
  is refusal 73's sentence.

### For the coordinator (from the review pass)
- **Station 13 (MECH-A6) and a stated cap:** only `MechanismService` writes a cap (at bind, D1 (a)). Every program
  at the base that builds mechanisms binds it, but MECH-A6's Bench Tests station 13 commands mechanisms
  directly and binds no service — there a stated `.currentCap` would NOT be applied (the motor stays at 2.5 A,
  usually, with no line). Whichever of MECH-A6 and MECH-S-A lands second must make station 13 write a stated
  cap before it commands a capped mechanism (or say on its page that it runs uncapped). Robot one's cascade is
  the first capped mechanism, and the team's first priority.
- **REMAINING-WORK row 20e:** its existing cell still says snap detection has "an output rotation sensor
  optional and stronger" (D6/P3 found a sensor on the spool's shaft blind to a snap) and "not sized" (the
  brief sized the runs) — §3.3 gives both corrections to the plan commit; this run did not edit them (its
  clause is its only change to the row).
- **Refusal 73 is new** (not in the brief's §1.1): G3a Part 5, which mirrors the refusal sentences, must mirror
  it too.
- **The `/fast` build lesson** (above): after a re-configure, run the target's depend step (or a non-`/fast`
  build) before trusting a `/fast` build to recompile a header's dependents; the coordinator's verifier should
  check `compiler_depend.make` is not empty before counting a mutation verdict.

### 17:12 (2026-11-05, RECOVERED from file times, no `date` was taken: the full build's log `review-fix/full_build2.log` 17:10:00, this log's last write 17:12:39; first written as "17:20, estimated") — every gate re-run on the final tree (after the review's fixes and mutations, from a clean rebuild)
- `api_doc_tool.py generate` (the `///` comments changed); the binary REMOVED, `shulib_tests/fast` rebuilt (263
  objects — the src gate had touched src/), THEN `briefing_status.py generate`, then the full build with the doc
  gates: exit 0.
- `python3 tools/suite_run.py green` → `suite run: green — 3000 cases, 2956645 assertions` (3 skipped; 5 new
  cases since the first pass: the pace, the four kinds, the PROS-shim whole-mA case, the FakeMotor read
  counter, the plant's unreachable load).
- `ctest -R compile_refusal` → `100% tests passed, 0 tests failed out of 258`; TUs 258 = CTests 258 (s73 new).
- ARM header gate (every `include/shulib` header, 241, one TU, `-Werror`) → exit 0. CI greps (pros/ outside
  hal/pros; shulib/sim/ outside sim) → empty. `git diff --stat -- include/shulib/hal/motor.hpp src/ Makefile`
  → empty.
- `src_build_gate.py self-test` → `OK (269 detector cases, each against a real build)`; `check` → `PASS (all 7
  builds link …)`; `git diff -- Makefile src/ | grep -c 'self-test plant'` → 0.
- `api_doc_tool.py` self-test OK; check-coverage "5530 public entities across 198 headers, all documented";
  check-fresh exit 0; check-examples "651 quoted lines, all verbatim (12 source files)"; check-removability
  "no public doc references docs/internal/". `briefing_status.py` self-test OK, check "generated block
  current"; `doc_staleness_audit.py` → clean.
- **MECH-S-A is BUILT and REVIEWED, not closed:** a fresh verifier and the coordinator's checks follow; not
  committed; no `-COMPLETED.md`, no pointer move. **Nothing here has run on a robot.**

### 18:04 (2026-11-05, `date`) — Verification-round findings and their disposition
The fresh verifier returned two minor findings. Each was re-checked against the code before any change.

| # | Finding | Re-check | Disposition |
|---|---|---|---|
| 1 | The 0.2 A margin rounded the SIGNATURE to its nearest mA (`milliamps()`, `lround`), but a signature is never sent: the StallDetector trips on a whole-mA reading `>=` the exact double, so a signature's device edge is the FIRST tripping mA, not its nearest. 0.9994 A under 1.2 A was silent (nearest 999, margin 201), though the detector trips it only at 1000 mA, exactly 200 under. The header and delta 24 claimed "the edge is exact" for every comparison. | REAL. Confirmed from the code: every detector is fed `maxCurrent()`, one motor's reading, and ProsMotor's `current()` is `get_current_draw()` (whole mA) / 1000 through `motorMilliampsToCanonical`. A bare `ceil(a·1000)` is not exact either: 2.007 · 1000 is 2007.0000000000002 in doubles (ceil says 2008, the detector trips at 2007), and one ulp above 0.938 A times 1000 is 938.0 (ceil says 938, but 938 mA reads 0.938 A, BELOW it, so it trips at 939). | **FIXED at the root:** new `detail::Slot::tripMilliamps(a)` (`service.hpp`), the least whole mA `r` with `motorMilliampsToCanonical(r) >= a` (the detector's own comparison through the one mA→A conversion: a ceil, then corrected down and up). The margin line is now `capMilliamps() - tripMilliamps(sig) <= 200`. The cap and read-back comparisons are unchanged. The header section, the margin comment, guide 18 (the four-currents bullet: "it judges as the motor reads … type a signature in whole milliamps and the arithmetic is the plain one"), guide 11, HA-223's entry, the changelog, `kinds.hpp`'s `///` comment (API pages regenerated) and delta 24 (marked corrected) say so. New test case "CA2 (verifier)": the detector itself shown tripping at exactly the mA each fixture names (and not one below), then five lifts on the 200 mA edge: 0.9996 A and 0.9994 A under 1.2 A warn, 0.999 A does not (201), 2.007 A under 2.208 A does not (a bare ceil warns falsely), one ulp above 0.938 A under 1.139 A warns (a bare ceil is silent). |
| 2 | The log's preamble says every section-header time is a `date` reading; the 15:50 entry and the gates entry ("17:20, estimated — `date` not taken") are estimates. | REAL. File times recovered: the review pass's last full build log (`review-fix/full_build2.log`) 17:10:00 and this log's last write before this round 17:12:39, so the gates entry was written by 17:12, not 17:20. For 15:50, the review-tests reviewer's last output was 15:52, the pass's first `date` was 16:09, and the disposition draft (`review-fix/log_draft.md`) was last written at 16:10. | **FIXED:** the gates entry's header now reads 17:12, "RECOVERED from file times" with both mtimes and its first label. The 15:50 header is kept as written (no file fixes it more closely), and the preamble now lists the three exceptions (00:40, 15:50, the gates entry) with their bounds. Every header from this round on is a `date` reading. |

**The verification round's mutations** (driver `.claude/coordinator/scratch/mech-s-a/vround/vmut.py`, copied from the
verifier's: one exact-text change, `make shulib_tests/fast` with `compiler_depend.make` non-empty (73128 lines),
the whole suite under `ulimit -v 4194304; timeout 600`, then the new case alone for its failing lines; the file
restored in `finally` and on SIGTERM/SIGHUP/SIGINT, its mtime bumped, `cmp`-identical to the backup under
`vround/backups/`). Each run recompiled 55 objects, all 55 dependents of `service.hpp`:

| Mutation | Change | Caught by | Verdict |
|---|---|---|---|
| V1 | the margin back to the signature's NEAREST mA (`milliamps(sig)`, the code before this round) | the new case: `s9994` (0 Warns, 1 expected) and `s938u` (0, 1 expected); 2 assertions, 1 case | RED |
| V2 | `tripMilliamps` without its downward correction (a bare ceil that can overshoot) | the new case: `s2007` (a false Warn: 1, 0 expected) | RED |
| V3 | `tripMilliamps` without its upward correction (a bare ceil that can undershoot) | the new case: `s938u` (0 Warns, 1 expected) | RED |

All three are runtime assertion failures, none a build failure, none a cap or timeout kill. The existing CA2 edges
(1.0 under 1.2, 2.3 under 2.5, 1.01, 0.99) all land on whole mA, so they pass with and without the change.

**The prose review for this round (rule 4), grep run before this entry:** `grep -rn "0\.2 A\|200 mA\|kCapSignatureMargin"`
over `docs/guide`, `docs/*.md`, `include/shulib`, REMAINING-WORK, the roadmap and the briefing. Changed: guide 18
:326, guide 11 :358, HA-223 :1836, changelog :63, `kinds.hpp` :218, `service.hpp` :119–126 and :579–581.
Checked and left alone: guide 18 :263 (points to the section that now says how it is judged), :320 and :332
(1.0 A under 1.2 A is whole mA and still warns), HA-223's table row :461 and its claim :1834 (the motor-side
claim, unchanged), roadmap :461 (a one-line summary, still true for whole-mA signatures), `spec.hpp` :53 ("warns
within 0.2 A", unchanged), the changelog's :67 constant line, and `service.hpp` :136 and :509 (the order and the
constant, unchanged).

### 22:08 (2026-11-10, `date`) — the verification round RESUMED (the earlier run stopped at the session's end): state checked, both findings re-verified, every gate re-run on the final tree
- **What was found on resuming** (the round's 18:04 entry was already written; its gates were not): `git diff --
  src/ Makefile` empty; `include/shulib/mechanism/service.hpp` `cmp`-identical to its last mutation backup
  (`vround/backups/V3__service.hpp`); every other file any mutation round backed up (`spec.hpp`,
  `mechanism_plant.hpp`, `motor_conversion.hpp`, `fake_motor.hpp`, `fake_device_provider.hpp`,
  `mechanism_parts.hpp`) identical to its latest backup, and `kinds.hpp` / `hal/pros/motor.hpp` differing from
  theirs only by the later rounds' logged `///` wording (diffed). No mutant, no plant.
- **Both findings re-verified, not redone:** finding 1's fix read again (`tripMilliamps`, the margin line, the
  header and margin comments) and the "CA2 (verifier)" case read against it; the prose-review grep of 18:04
  re-run (`0\.2 A|200 mA|kCapSignatureMargin` over the guides, `docs/*.md`, `include/shulib`, REMAINING-WORK,
  the briefing): every hit says the same as the 18:04 entry records. Finding 2: the preamble's exceptions
  paragraph and the 17:12 header stand; this header is a `date` reading.
- **An incident in this pass, and its root cause (for the coordinator):** the first `src_build_gate.py
  self-test` was run under `timeout 1200`; with three other worktrees' self-tests running at once it overran,
  `timeout` killed it (rc 124), and the tool — which has NO SIGTERM handler — left its plant in
  `src/build_selection.hpp` (every `kRobotId` line of the one chain DELETED). **`git diff -- Makefile src/ | grep
  -c 'self-test plant'` said 0**: a deletion plant carries no marker, so that grep cannot see it — check `git
  status -- src/ Makefile` instead. The `check` that followed ran on the planted file and FAILED ([one-chain]:
  "sets kRobotId 0 times") — that result was discarded. The plant was copied to
  `vresume/build_selection.hpp.plant-left-by-timeout` and the file restored with `git checkout` (this branch
  never changes `src/`), its mtime bumped; then self-test and check were re-run with NO timeout (the gate is not
  a test binary; it took 28.5 min for self-test with the other worktrees' builds beside it, 4.5 min for check).
- **Every gate on the final tree** (logs under `.claude/coordinator/scratch/mech-s-a/vresume/`):
  - `api_doc_tool.py generate`; `rm -f build/test/shulib_tests`; `cmake -S test -B build/test`; `shulib_tests`
    built; `briefing_status.py generate` (binary present); the full build with the doc gates: exit 0.
  - `src_build_gate.py self-test` → `OK (269 detector cases, each against a real build)`; `check` → `PASS (all 7
    builds link …)`; afterwards `git status --short -- src Makefile` empty and the plant grep 0.
  - After the src gate (it touches `src/`): the binary REMOVED and relinked (263 objects), then the
    regenerate / full build / suite / refusals / doc gates below, on the tree holding this entry.
  - `python3 tools/suite_run.py green` (under `ulimit -v 4194304; timeout 600`) → `suite run: green — 3001
    cases, 2956665 assertions` (the 3000 of 17:12 plus "CA2 (verifier)"); the MECH-S-A cases alone: 39 cases,
    611 assertions, all passed.
  - `ctest -R compile_refusal` → `100% tests passed, 0 tests failed out of 258` (TUs 258 = CTests 258).
  - ARM header gate (every `include/shulib` header, 241, one TU, `-Werror`) → exit 0. CI greps (pros/ outside
    hal/pros; shulib/sim/ outside sim) → empty. `git diff --stat -- include/shulib/hal/motor.hpp src/ Makefile`
    → empty.
  - `api_doc_tool.py` self-test OK; check-coverage "5530 public entities across 198 headers, all documented";
    check-fresh exit 0; check-examples "651 quoted lines, all verbatim (12 source files)"; check-removability
    "no public doc references docs/internal/". `briefing_status.py` self-test OK, check "generated block
    current"; `doc_staleness_audit.py` → clean.
  - Finished 22:16 (`date`).
- **MECH-S-A: the verification round is CLOSED on the builder's side** — both findings fixed at the root and
  re-verified, 3 mutations RED (18:04), every gate green on the final tree. Not committed; no `-COMPLETED.md`, no
  pointer move; the coordinator's checks follow. **Nothing here has run on a robot.**

## MECH-S-A — the coordinator's merge verification *(2026-11-10/11)*

- **Squash** of `mech-s-a` (base `ef6162e`) onto `16c01e0` (L6-A2). Conflicts resolved by hand: guide 18's opening (MECH-A6's
  bench-page sentence kept, the cap sentence after it, "still to come" no longer lists the bench page), the register (MECH-A6's
  "added NONE" paragraph kept, MECH-S-A's "added FIVE" after it; HA-153's row is main's — Station P — and HA-154's is MECH-S-A's),
  the roadmap (MECH-S-A's "Now" paragraph after L6-A2's, dated 2026-11-10), the changelog (MECH-S-A's entry at the top of
  `## API 2.5`, dated "2026-11-10 (built 2026-11-05)"), and the generated `docs/api/` pages and briefing (regenerated).
  REMAINING-WORK row 20e: MECH-S-A LANDED 2026-11-10.
- **A semantic conflict the build would have caught:** L6-A2's two new test providers (`test/robot_wall_sensors_test.cpp`,
  `OddDistance` and `RecordingProvider`) lacked MECH-S-A's new pure virtual `IDeviceProvider::currentLimit`; both now forward
  it (the recording provider does NOT record it — a limit is a view of a motor already asked for, so T13's request order is
  unchanged).
- **Gates on the merged tree** (`.claude/coordinator/scratch/mech_s_a_merge_verify.sh`, every binary capped): the suite green,
  3,138 cases; `src_build_gate.py` self-test OK and check PASS (all 7 builds link); the ARM header gate CLEAN (243 headers);
  the doc gates and the briefing check clean; the compile-refusal harness 258 of 258; no plant, no stray root file.
- **The coordinator's own mutations** (`muts_mech_s_a_main.json`, `_b.json`): **6 of 6 RED** — MS1 a stated cap never
  written, MS2 an accepted write counted as taken whatever reads back, MS3b the "set and read back" line printed after a
  refusal (MS3, `if (true)`, broke its own build under -Werror — an unused `allTook` — and was re-cut, not counted), MS4 the
  re-check never run, MS5 the plant's limiter never engaged, MS6 a stall signature AT the cap accepted. The restored tree
  relinked and ran green.
- **MECH-S-A LANDED.** Nothing has run on a robot. TUNE1-A builds on a snapshot of MECH-S-A's pre-review build; its merge
  re-checks every place it depends on the cap path (its live log lists them).
