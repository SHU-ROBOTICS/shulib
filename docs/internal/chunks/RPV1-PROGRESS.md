# RPV1 — the route preview: live progress log

<!-- release: between parts — RPV1-A (A0+B1) landed 2026-11-14 and RPV1-S1 (the switch to robot one) 2026-11-15; A1/A2 and B2 follow (declared by the coordinator 2026-11-14; S1's words written for its merge) -->

> Appended in real time (RESUMING's chunk loop). The brief is
> [`RPV1-route-preview.md`](RPV1-route-preview.md); its answers block, R1–R13 and D1–D13 (as recommended) govern.

## RPV1-A (A0+B1)

**Executor:** `opus` (Claude Opus 5.5), a workflow subagent. **Worktree:** `.claude/worktrees/rpv1-a`, branch
`rpv1-a`, base `d33d25d` (CURVE1-A, API 2.7). **Go:** the team lead, 2026-11-11 ~00:20, "Go: A0+B1 before 21st
(Recommended)" — A0 and B1 as ONE run. Scratch: `.claude/coordinator/scratch/rpv1-a/` (never `/tmp`).
Do not commit. Every binary runs as `(ulimit -v 4194304; timeout 600 …)`.

### Launch state (2026-11-12)

- `git log -1` = `d33d25d`; tree clean except the untracked brief.
- **ROB1-A has NOT landed** (`shulib-v2` tip is `d33d25d`; no `src/robots/robot_one_facts.hpp`). So B1 is built and
  tested against the test fixture robot with a lift (`test/mechanism_fixtures.hpp`), and `--robot one` answers
  "robot one has no build yet" (RP25) until ROB1-A lands.
- **Baseline suite** (after `briefing_status.py generate` with the suite unbuilt — the untracked brief made the
  briefing stale — and a full build): `suite_run.py green` → **green — 3372 cases, 3779373 assertions**. (A first
  run read BROKEN, signal 11: the background build had compiled half the objects before the fake sink's hook landed
  and half after — two layouts of one class. A rebuild of every dependent object made it green; recorded so nobody
  reads that crash as the tree's.)

### RP35's BEFORE fingerprints (captured over the BASE tree, before the rig changed)

`git archive d33d25d` → `scratch/rpv1-a/base-tree/`; probe `scratch/rpv1-a/probe/base_hash_probe.cpp`, which includes
the same `test/route_preview_hash.hpp` and `test/route_preview_fixtures.hpp` the suite uses (copied beside it), built
against the base tree's headers, run under `(ulimit -v 4194304; timeout 600 …)`:

```
40in forward: truth b777dc5fd6086413 sink a9f6e719ff29c0cd paces 494 lines 6 records 1090 word FINISHED
TEST 24in turn back: truth 9834ef8786009c0a sink 05467b932ab6683e paces 858 lines 6 records 1821 word FINISHED
skills long: truth 4182eaa13ccb3663 sink fdcb43740365e43d paces 6050 lines 133 records 12219 word CUT BY DEADLINE
```

### Build log (2026-11-12/13)

1. **The fake sink's line hook** — `include/shulib/hal/fake/fake_telemetry_sink.hpp`: `setOnLine(fn, ctx)`, called
   after each stored line; default none. `api_doc_tool.py` excludes `/fake/` from the API reference ("test doubles —
   documented for test authors in test/README.md"), so no `docs/api/` page moves; `check-fresh` / `check-coverage`
   green after the edit (6062 entities, all documented).
2. **B1's rig change, IN PLACE** — `test/auton_tester_rig.hpp`: `MechanismWorldLayer`, `MechanismBinding`, a last
   defaulted `TesterStack` parameter, `listOf()`. Default = robot two's list over `RobotTwoFakes`, exactly as before.
   **RP35 passes on the first build**: the three fingerprints captured over the base tree match bit for bit.
3. **`test/fakes_by_kind.hpp`** (NEW) — any list booted through the REAL `robot::MechanismParts` over the FAKE device
   provider (test/mechanism_service_rig.hpp's recipe), ArmPlant / FlywheelPlant built from the declaration's own
   numbers. First probe: robot x's lift to "high" took 4.01 s and TIMED_OUT against the fixture's 4 s bound — exactly
   the declaration's trapezoid (3.99 s) + its 0.1 s "reached" window; the fixture's bounds were raised to 6 s.
4. **`test/route_preview_model.hpp`, `test/route_preview_render.hpp`, `test/route_preview_run.hpp`,
   `test/route_preview_main.cpp`** (NEW), **`test/CMakeLists.txt`** (one target, `shulib_route_preview`).
   First smoke run: "40in forward" 1.80 s, "TEST 24in turn back" placed at (-48,-48,90) 5.44 s — P10's ideal numbers.
   The long skills fixture: CUT BY DEADLINE, scoring ended T+57.36 s — P2's number exactly.
5. **`test/route_preview_test.cpp`, `test/route_preview_fixtures.hpp`, `test/route_preview_hash.hpp`** (NEW).
   First run: 23/27; fixed (a) the loop sentence's step number — the STOPPED line is logged after the last pace, so it
   is the first line of the flood, not of the recorded log (foldLoop reads both); (b) RP31's fixture drove sideways
   (a tank cannot) — re-aimed; (c) RP33 found the region marker inside a banner comment first — anchored to
   line starts; (d) RP24's bound (above). Then 27/27.
6. **`tools/route_preview.py`** (NEW) — the launcher; self-test 1 failure first: `git describe` here reads
   `v0.1.1-415-gd33d25d-dirty` (a tag exists), so the check now asks that the stamp NAME HEAD's short SHA.
7. **The page, looked at** (Firefox headless screenshots, `scratch/rpv1-a/shots/`): two fixes from looking —
   stacked step numbers at a revisited spot became unreadable on the 160-step skills fixture (now one label per
   spot, "1,7,13,19 +6", all in its tooltip), and after a THROUGH the step's own colour was drawn over the faded part
   (now solid only up to the THROUGH sample; the headline also says "but it goes THROUGH the right wall at T+1.87 s").
   A legend line was added under the field.
8. **Suite after the tests landed:** `suite_run.py green` → green — 3399 cases (3372 + 27).

### Deltas from the brief (each with its reason)

| # | Brief says | Built | Why |
|---|---|---|---|
| Δ1 | §1.1: two new headers, `route_preview_run.hpp` (logic) and `route_preview_render.hpp` | a THIRD, `test/route_preview_model.hpp` — the Model and THE FIELD CHECK | the renderer then depends on neither the rig nor the run (the run includes the renderer to write the page; the renderer needs the Model); the field check is pure geometry a test reaches without a run |
| Δ2 | §1.2: "`test/auton_tester_rig.hpp` — CHANGED IN PLACE … A new `FakesByKind`" | the rig gains only `MechanismWorldLayer` + `MechanismBinding` + the defaulted last parameter; `FakesByKind` lives in its own `test/fakes_by_kind.hpp` | keeps the change to a rig ~3,000 tests include to ~40 lines; FakesByKind needs `robot::MechanismParts`, the fake provider and `sim/mechanism_plant.hpp`, which no other user of the rig should compile. Reuse, not a fork: FakesByKind boots the list through the REAL `MechanismParts` over the FAKE provider (test/mechanism_service_rig.hpp's recipe) |
| Δ3 | §1.1 / D6: THROUGH = "past a wall's inner face" | THROUGH = past a wall's face OR more than the allowance into a **Loader** | RP29 (c) requires it ("the same drive 1.5 in further → THROUGH" — 1.5 in into a Loader is 2.24 in short of the wall); a Loader stands on the floor and is solid. Toggles (body above the wall, brackets hanging from 10.6 in) stay TOUCH-only. ~~Said in guide 16 and on every page~~ — **corrected in the review round (2026-11-13):** when this row was written guide 16's TOUCH bullet did NOT say a Toggle is touch-only (it read as if a deep Toggle overlap were something more); it now says "or overlaps a Toggle by any amount … the preview never calls it THROUGH". Every page's outline sentence names only "a wall or a Loader" as THROUGH |
| Δ4 | D2 (a): "drives, turns and mechanism steps" | drives, turns, **curves** and mechanism steps | CURVE1-A landed in the base (§1.6: "a curve is a motion — numbered like any drive"); a curve is drawn from the plant's poses, never a spline |
| Δ5 | RP8 (c): runAuton's own refusal via "a route entry with no registry" | via an entry handed a period its stop does not fit (runAuton's same step-1 refusal), through `foldRun` | the tester rig ALWAYS passes its command registry, so the route form cannot be refused from the rig; the period form is the same refusal block in `match_runner.hpp` and reaches the same fold |
| Δ6 | RM25: "one step counter across all routines" | re-cut: "the routine's NAME dropped from the quoted lines" | in A0 the tool keeps no step counter per routine — the numbers on the page's RTN lines are the LIBRARY's (D2 (a)); the only way A0 can merge two numberings is to drop the name that tells them apart |
| Δ7 | §3.1: glossary "'host test plant' (amended …)" | ADDED a "host test plant" entry | the glossary had no such entry (only "plant" and "host-side"). **Review round:** the first wording defined the whole plant as the preview's IDEAL one ("no walls, pieces or other robots") and contradicted chapter 5 (:196, lag and a weak side on the host test plant); it now defines the plant generally (ideal unless a test adds lag, a weak side, slip or noise; walls, pieces or a camera's view only where a test puts them) and says the preview runs it IDEAL |
| Δ8 | §1.1 exit codes: "2 usage" | no name at all → the names listed on stderr + usage, exit 2; `--list` → the names on stdout, exit 0 | D7 says "No name: it lists every auton"; a bare call is still a usage error for a script, `--list` is the asked-for listing |
| Δ9 | §1.1 launcher: "runs it with RLIMIT_AS … imported from tools/suite_run.py" | imports the constants AND `suite_run._child_setup` (the one place the limits are applied) and runs its own Popen | `suite_run.run()` captures stdout to a temp file and returns only its last 64 KiB; the preview's table must reach the terminal whole. Same bounds, same child setup, nothing copied |
| Δ10 | D3 / §1.1: `--place -48,-48,90` | the guide writes `--place=-48,-48,90`; the launcher also accepts `--place VALUE` (it joins them) | argparse reads a value starting with '-' as an option. ~~the C++ program takes either~~ — **FALSE when written** (the program matched only the bare word `--place`; `--place=0,0,0` was "unknown flag", exit 2 — the review's correctness finding). Since the review round the program splits EVERY `--flag=value` at its first '=' (the usage line says so; RP37: `--robot=`, `--place=`, `--out=` give the same run as the spaced form; `--list=yes` is a usage error) |
| Δ11 | §1.1: page contents | added a legend line, and "every line the library logged from the arming on" at the foot | the colours must be readable without this brief; the full log is what a student reads next when a sentence is not enough. The no-pace stop's flood is NOT in that log (it is the page's own "looping lines" section, the first 200) |
| Δ12 | (none) | the test cases write their pages under `build/route_preview_test/` (gitignored) | never `/tmp`, never the repository root |
| Δ14 (review round) | D5 (a): "each step's number where it started AND ended, with its time from the auton's start"; R4: "the start step is a 'started' mark; a `waitUntilDone` step shows the verdict it collected" | one visible number per step, where it BEGAN (spots revisited share one label, "1,7,13,19 +6"); where it ENDED and every time are in the tooltips — each number's (its name, T+ start → T+ end, its verdict), each motion's end outline ("ended here, T+…: SETTLED"), each background mechanism's diamond ("started here in the background, T+…; its verdict … arrived at T+…") and its new DOT where the verdict arrived; the table carries both times. A `waitUntilDone` is not drawn | a second visible number at a step's end sits on the next step's start (a drive ends where the next step begins) — on the 160-step skills fixture the labels were already unreadable with ONE number per step (Build log 7). A `waitUntilDone` starts no motion and raises no step count, so from below (D2 (a)) the preview cannot see it until A2's listener; guide 16 says so ("The `waitUntilDone` that collects it is not drawn") |
| Δ15 (review round) | §1.1: exit codes 0 / 1 / 2; R12c: `route_preview_main.cpp` is `return runPreview(args, std::cout, std::cerr);` | a fourth code, **3 — the tool itself failed** (the page could not be written; the preview threw; the launcher: the program did not build); `main` calls `runProgram` (runPreview, with an exception escaping it said in words and exited 3) | 2 means "a usage error, nothing written"; a broken tool exiting 2 could not be told from a typo by a script or by A1's CI check (the review's finding). `main` still only forwards argv; `runProgram` is in the header and tested (RP38) |
| Δ16 (review round) | §1.2: FakesByKind "moves exactly as its declared numbers predict"; a flywheel "likewise" | an ASSUMED time constant where a declaration states no `kA` — 0.05 s for an arm or lift, 0.2 s for a flywheel (`test/mechanism_auton_rig.hpp`'s lift and flywheel, and `sim::FlywheelPlantConfig`'s default), `kA = τ·kV`. A flywheel's words cannot state `kA` (`SpeedTuning` holds kS, kV, kP, kI), so every declared flywheel runs on it, and its steps' PLACEHOLDER note names it. Robot x's arm-plant numbers and the declared profile still set an arm's time (RP24 unchanged) | a declaration's `kA` is the controller's acceleration FEEDFORWARD (starting value 0), not the mechanism's inertia; a plant built with kA = 0 is memoryless, and a speed loop that reads one tick late on it diverges whenever kP/kV > 1 (e[k+1] = −(kP/kV)·e[k]; robot x's flywheel: 1.57) — the false red TIMED_OUT the review found (2058 of 3000 rpm at a constant 6.86 V). No real mechanism has no inertia. Not "exact": RP24's window is [profile, profile + 0.35 s] |
| Δ17 (review round) | §1.2's B1 note "… TUNE1 measures the real one"; the brief's robot-one sentence naming ROB1-A | "… 'shulib Tune' sets the real ones on the robot"; "robot one has no build yet: its mechanism list and its autons file are not in the code yet (…)" | a student reads these lines: no chunk tag on any page, table or terminal line. TUNE1 landed in the base as the program "shulib Tune", which tunes live rather than timing a move |
| Δ18 (review round) | (none) | the program REMOVES the auton's last page before it runs it, and writes the new one to `<page>.part` renamed into place; the launcher says "No page was written for this run" after a timeout or a signal | an earlier run's page left in place after a killed run reads as this run's (the review's finding). RP38 kills a forked run of a silent loop with its alarm: the old page and any `.part` are gone |
| Δ13 | B1: robot one "switched the day ROB1-A lands" | ROB1-A has NOT landed (base `d33d25d`): `kRobotOne` has no list and no register function → RP25's sentence, exit 2. The fixture robot x is exercised through the SAME `PreviewRobot` path the switch will use | the switch is then two fields of `kRobotOne` (`table = &robot_one::kRobotOneMechanisms`, `registerAutons = &registerRobotOneAutons`) plus `src/team/robot_one_autons.cpp` in the CMake target — for the coordinator after ROB1-A merges (see "For the coordinator") |

### RP35 strengthened after mutation RB1 SURVIVED (2026-11-13)

The first mutation run's **RB1** (the rig's DEFAULT devices changed: `two{true || absentPivot7}`, pivot 7 ABSENT
by default) stayed GREEN: none of RP35's three fingerprinted runs touches pivot 7, and binding an absent device logs
nothing. A real gap, closed: (1) a fourth fingerprinted run, `rp_fixture::everyMechanism` (the claw, pivot 6, pivot
7, the claw, a drive), its BEFORE fingerprint captured over the base tree by the same probe (fixture header copied
beside it): `every mechanism: truth ad9b850f2b3bc459 sink 633cf69544aee734 paces 629 lines 10 records 1360 word
FINISHED`; (2) the default stack's bound slots asserted directly — robot two's three names, in order, every one
built (the base probe printed the same: `slot 0 'claw pivot 6' built 1`, `slot 1 'claw pivot 7' built 1`,
`slot 2 'claw' built 1`). The base probe's first three fingerprints re-printed unchanged (`scratch/rpv1-a/probe/
base_hash2.txt`). RB1 is re-run on the final tree (below).

Two more case changes from reading the first mutation log: **RM10**'s red was the test case THROWING (the registry
built to throw) rather than an assertion — RP8 now runs both refused lookups under `REQUIRE_NOTHROW`, so the red is
the test's own assertion; **RM22**'s red showed the forked child's escaped `std::bad_alloc` landing back in the
test runner (the child printed its own doctest summary) — the child's body is now wrapped so it always ends with
`std::_Exit` (77/78 returned, 79 threw), never inside doctest.

### Interruptions and renames (2026-11-13, logged as they happened)

- **A second mutation run was STOPPED one mutation in, by me, and restored.** I had edited `docs/guide/README.md`
  (chapter 16's index row) while it ran; that file is a doc-gate input, so the next mutant build would have re-run
  the doc gate against a briefing whose suite line no longer matched (the cases gained assertions) — every later
  mutant would have failed to BUILD and been mis-read as RE-CUT. The driver was ended with SIGTERM: its handler
  restored `test/route_preview_run.hpp` (RM1's file) — `cmp` against its backup: identical. Lesson kept: no edit to
  any doc-gate input while a mutation run is in flight. (My `pkill -f` pattern also matched — and ended — my own
  shell; it did NOT match the coordinator's own mutation run in the main tree, `.claude/coordinator/mutate.py`,
  which was checked alive afterwards and never touched.)
- **`tester_rig::MechanismBinding` renamed `tester_rig::RigMechanisms`** (the rig, FakesByKind, route_preview_run,
  test/README.md): the library already has a public `teleop::MechanismBinding` (driver buttons, guide chapter 10,
  cookbook 6) — the prose review's grep for the new name found it. A test-only type must not share a library
  type's name. Earlier lines of this log keep the old name as history.

### Tests added (`test/route_preview_test.cpp`, 27 cases; A0's RP list + B1's)

RP1 (×2: the tap/plant-distance/page check; the truth-not-estimate check under a +5° IMU), RP2, RP3, RP6, RP8 (a, b, c),
RP9, RP10, RP11, RP13, RP14, RP15, RP16, RP17, RP18, RP27 (forked child), RP28 (robot two's pivot; robot x's intake
failing in the background), RP29 (a)–(d), RP31, RP32, RP33, RP34; B1: RP24, RP25, RP26, RP35, and "B1: FakesByKind
builds each plant from the declaration's own numbers". A1's (RP7, RP12, RP19, RP20, RP21, RP27b, RP30), A2's (RP4,
RP5) and B2's (RP22, RP23) are not this run's.

### Mutations — FINAL run on the final tree (`scratch/rpv1-a/mutate.py`, log `mutations.final.log`)

Every mutation: backed up under `scratch/rpv1-a/backups/`, applied (each replacement matched exactly once), the
suite binary REBUILT (every dependent object recompiled — the log lists them: 1 TU for the preview's headers; all 9
TUs that include `auton_tester_rig.hpp` for RB1/RB2, checked against the compiler's `.d` files), its filter run under
`ulimit -v 4194304; timeout 600`, the file RESTORED in a `finally` (and on SIGTERM/SIGHUP) with its timestamp bumped.
None broke its own build (no RE-CUT). **29 / 29 RED, each by the test's own assertion:**

| # | Mutation | Red case — the assertion that fired |
|---|---|---|
| RM1 | recorder samples BEFORE `inner->pace()` | RP1 — "sample 0 is not the tap's" |
| RM2 | path from the ESTIMATE | RP1 ×2 — tap mismatch; truth-not-estimate |
| RM3 | SVG y not flipped | RP1, RP2 — "page point 0 (130, 130) is not the sample's (130, 610)" |
| RM4 | plant not placed at the FIELD start | RP2 (and RP29) — first sample not at -48 |
| RM5 | `--place` heading sign | RP3 — placed end ≠ placement ∘ bare end |
| RM8 | skipped steps drawn with the stop's pose | RP6 — `steps.size() == 3` |
| RM9 | T+ from the clock's zero | RP1, RP6, RP11, RP13 |
| RM10 | Throw-mode registry | RP8 — `REQUIRE_NOTHROW(a = run({"too long"}))` (an assertion, not a bare throw) |
| RM11 | throw drawn FINISHED | RP9 |
| RM12 | case-insensitive match | RP10 — `r.code == 2` |
| RM13 | field check on the centre | RP11 — `samples[throughAt].x < face` |
| RM14 | the wall at 72 in | RP11 — `cornerPast(throughAt - 1) <= 1.0` |
| RM15 | refused wall reset green | RP14 — mark Amber |
| RM16 | caption dropped | RP15 |
| RM17 | wall-clock stamp in the page | RP17 — `a.page == b.page` |
| RM18 | no HTML escaping | RP18 |
| RM20 | launcher's bounds not applied (`preexec_fn=None`) | launcher `self-test` — 4 FAIL lines (address space, output, CPU, core) |
| RM21 | MECH lines attributed to the open step | RP28 — the drive's sentence must be empty |
| RM22 | no-pace stop never reached | RP27 — the child's exit status ≠ 1 (it ended in `std::bad_alloc`) |
| RM23 | allowance 0 | RP29 — (a) is not a TOUCH |
| RM24 | stamp from `SHULIB_BUILD_HASH` | RP34 |
| RM25 (re-cut, Δ6) | routine name dropped from quoted lines | RP31 |
| RX1 (added) | UNCONFIRMED drawn green | RP13 |
| RX2 (added) | a Loader only touchable (Δ3 undone) | RP29 (c) — `REQUIRE(c.through)` |
| RX3 (added) | lookup ignores the refusal sentences | RP8, RP32 |
| RX4 (added) | `--robot` ignored | RP25, RP26 |
| RX5 (added) | FakesByKind starts a lift at its top stop | RP24, the B1 plant case |
| RB1 (B1) | the rig's DEFAULT devices changed (pivot 7 absent) | RP35 — the slots' `built()` ×4, AND the "every mechanism" fingerprint (`paces`, `truth`, `sink`) — re-run with every assertion captured (7 errors) |
| RB2 (B1) | FakesByKind's plant 3× weaker than declared | RP24 — the lift no longer FINISHES in its declared time |

The FIRST run (`mutations.run1.log`, before the post-run edits) had 26 / 27 red; **RB1 SURVIVED** — fixed as logged above.

### Gates (final tree)

- `suite_run.py green` → **green — 3399 cases, 3779816 assertions** (baseline 3372; +27).
- `briefing_status.py` generated with the suite unbuilt, the build, generated again; `check` → current.
- `api_doc_tool.py` self-test OK; check-coverage 6062 entities all documented; check-fresh exit 0; check-examples 762
  lines verbatim; check-removability clean. `doc_staleness_audit.py` self-test OK, run clean.
- ARM header gate: 249 headers in one TU (scratch `all.cpp`), `arm-none-eabi-g++ … -Werror` exit 0.
- CI greps: library PROS-free; core sim-free.
- `src_build_gate.py self-test` → OK (306 detector cases); `check` → PASS, all 8 builds link; `git diff d33d25d --
  src/ Makefile` EMPTY; `git diff d33d25d -- include/` = `hal/fake/fake_telemetry_sink.hpp` only.
- The brief's own: `route_preview.py "40in forward"` exit 0 (18,382-byte page), `"TEST 24in turn back" --place=…`
  exit 0 (30,313 bytes, stamped `v0.1.1-415-gd33d25d-dirty · autons file 2f45cb1053f1`), `"no such auton"` exit 2,
  names listed, nothing written; `./build/test/shulib_route_preview "40in forward"` exit 0, "commit not stamped";
  RP17 by hand (`cmp` of two launcher runs) identical; `grep -c 'http\|simulator'` 0 on every page; "IDEAL host test
  plant" on every page; C1's loop through the REAL program on a scratch autons copy (`scratch/rpv1-a/c1/`): exit 1,
  0.02 s, 12.6 MB, page written, "after step 32 the auton keeps looping without letting time pass …";
  `route_preview.py self-test` ok (0 failures).

### Prose review (rule 4) — greps run, hits read

- `simulator` in every new file and every page: none (the rule sentences now say "the project's one name for it";
  RP15 builds the word from two halves). Old changelog line 2964 is not this run's.
- `lower bound`: none in docs/guide, the changelog, the new files. `exact|guarantee` near "time": two glossary hits,
  both about the end action / watchdog (not times); "fakes that move EXACTLY as their declarations predict" was
  softened to "fakes built from their declarations' own numbers" (page, guide 16, RP26) — RP24 shows the lift within
  [profile, profile + 0.35 s], which is the declaration's prediction plus its "reached" window, not "exactly".
- `robot two` + drive/track in guide 16: every claim about the drive says "stand-in".
- `internal/|chunks/|RESUMING` in the public docs touched: none (check-removability agrees).
- ~~`R[CPM][0-9]` in the public docs and the one header touched: none.~~ **FALSE when written** (the docs reviewer,
  2026-11-13): `test/README.md` — one of §3.1's public docs — carried "`route_preview_test.cpp`'s RP35 pins three
  runs" at :163 (and "three" was stale too: RP35 pins FOUR runs since RB1). The grep was not run over
  `test/README.md` before this line was written — a rule-4 breach. Fixed in the review round: the sentence says "the
  default-rig case pins four runs' truth and sink — one of them stepping every one of robot two's mechanisms"; the
  rig's banner (a test header, not a public doc) says four. Re-run, hits read: below.
- Names: `auton_tester_rig`, `FakeTelemetrySink`, `TesterStack`, `route.preview`, `fakes_by_kind`,
  `MechanismBinding`, `shulib_route_preview` over docs/, README.md, test/README.md — every hit read; the only stale
  one was the NAME CLASH with `teleop::MechanismBinding` (renamed, above). `docs/diagnostics-plan.md` :79 (the fake
  sink as "recording double") still true. Guide index (`docs/guide/README.md` :24) chapter 16's row extended to
  name the preview. Root README lists no tools: untouched.
- Deliberately NOT touched: `docs/roadmap.md`, `docs/internal/build-order.md` (status pointers are the
  coordinator's at merge), `docs/hardware-assumptions.md` (no row — §4: the outline's row comes with A1's suite
  check), `mkdocs.yml` (no new page), `docs/api/` (fakes are outside the API reference).

### For the coordinator

- **Files other in-flight chunks also edit** (each change minimal, my own lines only): `test/auton_tester_rig.hpp`
  (AF1S-A1 may touch it — my change is the banner paragraph, `RigMechanisms`/`MechanismWorldLayer`, the defaulted
  last constructor parameter, the bind/`probe.inner` branch, `listOf`); `test/CMakeLists.txt` (one target appended
  after `add_test`; ROB1-A will add `src/team/robot_one_autons.cpp` to the suite's `target_sources` — different
  lines); `docs/changelog.md` (an entry at the top of `## API 2.7`, "No API change"; ~~STEP1-B adds its own~~ —
  corrected 2026-11-13: STEP1-B OPENED `## API 2.8` above 2.7 on main, so at the rebase this entry moves to the top
  of the newest section, still "No API change" — see the review round's D3 below);
  guide 14 / 15 / 16, `docs/guide/README.md` :24, `test/README.md`; `docs/internal/PROJECT-BRIEFING.md` (GENERATED —
  regenerate after the merge, suite unbuilt first).
- **The ROB1-A switch** (not done: ROB1-A had not landed at `d33d25d`): in `test/route_preview_run.hpp`'s
  `kRobotOne`, set `.table = &shulib::bench::robot_one::kRobotOneMechanisms`, `.registerAutons =
  &shulib::bench::registerRobotOneAutons`, a `provisional` table for robot one's OWED facts if the page should name
  them, add `src/team/robot_one_autons.cpp` to `shulib_route_preview`'s sources, and turn RP25 into "robot one runs"
  (RP26 on robot one's list). UNVERIFIED until then: the cascade is homed by STALL; FakesByKind attaches no switch
  for it and relies on the ArmPlant's hard stop and current to trip the homing signature — not exercised here (robot
  x's lift is switch-homed).
- **HA register:** no row (§4: the outline's row lands with A1's suite check, HA-251 reserved there).
- **The brief's §7 says "the src gate: 7 builds"**; the tree at `d33d25d` has 8 (Tune) — the gate ran all 8.
- **Reviewers:** this executor had no agent tool, so the three reviewers and the fresh verifier are the workflow's
  next steps; nothing here was reviewed by anyone but me.
- **Nothing committed.** Scratch: `.claude/coordinator/scratch/rpv1-a/` (probes, backups, mutation logs, screenshots).

## RPV1-A review findings and their disposition

Three reviewers (correctness, tests, docs) returned 27 findings (2026-11-13, ~01:40–02:05; their scratch:
`scratch/rpv1-a/review-correctness/`, `review-tests/`, `review-docs/`). Overlaps deduped below: C5 = D4 (D5's
numbers), T6 = D10 (curves), C7 ⊂ D2 (RP35's "three").

### Resumption log

- **First fix run** (2026-11-13 ~02:10–02:30, then idle): STOPPED by the team lead at 98 % of the usage limit
  (~03:40). It left its edits in the worktree (scripts in `scratch/rpv1-a/fix/edits/`: `run_header.py`,
  `run_cli.py`, `render_svg.py`, `launcher.py`, `tests_existing.py`, `docs1.py`, `docs2.py`) and new cases RP36–RP44
  and three B1 cases, built once (02:30, exit 0) — but logged NOTHING here and never ran the suite, a mutation or a
  gate on them.
- **Resumed** 2026-11-13 10:50 EST (`date`). Audit before trusting anything:
  - **No mutation left applied:** every one of the first run's 29 mutations (`scratch/rpv1-a/mutate.py`'s list) checked
    by its own text — the mutant text absent from its file — `scratch/rpv1-a/fix/residue_check.py`: 29 checked, 0
    residue. (A `cmp` against `backups/` cannot answer it: the first fix run edited those files after the backups.) The
    reviewers mutated only their own scratch copies (`review-*/tree/`, by their drivers' headers).
  - **No src-gate plant:** `git diff -- src/ Makefile` empty; `git status --ignored -- src/ Makefile include/` shows
    only `include/shulib/hal/fake/fake_telemetry_sink.hpp` (this run's hook).
  - **The binary relinked from source:** `rm -f build/test/shulib_tests build/test/shulib_route_preview`, the
    briefing generated with the suite unbuilt, a full build (exit 0; 9 objects recompiled — the TUs including
    `auton_tester_rig.hpp`, whose banner I re-wrapped — every other object current by its `.d`), generated again,
    rebuilt: `suite_run.py green` → **green — 3412 cases, 3780129 assertions** (3399 + the first fix run's 13 cases).
- **This run** then re-checked every finding against the code (below), finished what was not done, and found two
  more gaps of its own (the "inside" rule still missed a throw from ABOVE the recorder; N32, a survivor in the test
  reviewer's own log that no finding raised).

### Dispositions

**Correctness (8).**

| # | Finding | Disposition | Evidence |
|---|---|---|---|
| C1 major | an instantly-failing step that stops the route is labelled "park", number 0 | **FIXED at the root (first fix run; verified here):** the end action is decided by ORDER — the sink's line hook notes, as the guard's `[SEQ] end-of-run action starting` line is stored, every mechanism begin so far (`noteBegins()` first) and the count of motion boundaries; a MECH-line step (no slot) is the park's only if its line comes after that line. Never by a time window | RP36: (a) RP31's helper — `intake` ABSENT is **step 3**, not park; exactly one park step (the brake); the table row `3 \| intake`, the page's `<tr data-step="3">`, "step 3: 'intake' is not on this robot…" on the page and stdout; (b) robot x's lift auton on robot two — `lift` ABSENT is **step 1**; (c) robot y's never-homed `tilt` NOT_HOMED at once is **step 3**; (d) the other side — an entry's OWN end action that commands the claw: its claw and brake ARE the park (2 park steps). Mutations F1–F5 |
| C2 minor | every PROGRAM ERROR says "scoring ended T+0.00 s" | **FIXED (first fix run):** a throw's summary says "it threw at T+x s of the N s period" (the last pace's time); "scoring ended" only for a run the guard reported | RP9 asserts the exact clause on stdout and that "scoring ended" is on neither the page nor stdout. F6 |
| C3 minor | "PROGRAM ERROR inside step N" for a drive the deadline cut before the throw | **FIXED, then made exact here.** The first fix run used the recorder's in-pace flag — right for (a) and for a throw from below the recorder, but a throw from ABOVE it (a pacer layer, the library between paces) while a motion ran still read "after step N". Now: the boundary observer notes `std::uncaught_exceptions() > 0` for each boundary — the scheduler's `WaitUnwindGuard` and the guard's `RunUnwindGuard` are destructors, so the motion running at a throw is recorded DURING the unwind (`Step::openAtThrow`); a deadline-cut motion is recorded with nothing in flight. Inside that motion (or "inside the park (the end action)"); else, from inside a pace, "during a wait"; else "after step N" | RP9b: (a) the cut-then-throw fixture → "after step 1", never "inside", the cut drive amber and NOT `openAtThrow`; (b) an IMU that throws below the recorder mid-drive → "inside step 1", the path cut short (0.5 < x < 30); (c) NEW — a pacer layer ABOVE the recorder throws mid-drive → "inside step 1", `openAtThrow`, CANCELLED (the in-pace rule said "after step 1" here). F7, N2 |
| C4 minor | Δ10 says the program takes `--place=X,Y,DEG`; it rejected it | **FIXED (first fix run):** `runPreview` splits every `--flag=value` at its first '='; the usage line says so; Δ10 corrected (struck, with the date) | RP37: `--robot=review --place=-48,-48,90` and the spaced form give the same stdout; `--robot=x --out=FILE` writes FILE; `--list=yes` exit 2. F8 |
| C5 = D4 minor | R4's "started" mark / `waitUntilDone` verdict and D5 (a)'s end numbers and times — unlogged deviations | **PARTLY FIXED, the rest LOGGED (Δ14):** a background mechanism now has its diamond as the "started" mark and a dot where its verdict arrived; every number, outline, diamond and dot carries its times and result in its tooltip; the end NUMBER is not drawn and `waitUntilDone` is not drawn — Δ14 gives both reasons; guide 16 says both | RP39: the step-1 number's pixel and tooltip ("step 1 drive: T+0.00 → …, SETTLED"), robot x's background intake's "started here in the background, T+0.00 s; its verdict TIMED_OUT arrived at T+…" and its `mechend failed` dot. F19 |
| C6 minor | after a 300 s timeout or a crash an earlier page stays and reads as this run's | **FIXED (first fix run):** the program removes the auton's last page before it runs it (a found name or a refused one), and writes the new page to `.part`, renamed into place; the launcher's timeout and signal sentences add "No page was written for this run …" (Δ18) | RP38: a forked child running the silent loop is killed by its alarm — the old page is gone and no `.part` is left; the launcher's self-test reads both sentences through a stub. F9, L1 |
| C7 minor | two places still say RP35 pins three runs | **FIXED (first fix run):** `test/README.md` ("the default-rig case pins four runs' …", no tag) and the rig's banner ("four runs"); this run re-wrapped the banner's over-long line | `grep -rn "three runs\|RP35" test/README.md test/auton_tester_rig.hpp`: the banner's "four runs" only |
| C8 minor | a broken preview exits 2, the usage code | **FIXED (first fix run):** exit **3** — the page could not be written, the preview threw (`runProgram`), the launcher's build failed (Δ15); documented in guide 16's exit codes and the launcher's docstring | RP38: a page under a FILE (not a directory) → 3 with "could not write the page"; a robot with no list → `runPreview` throws `PreconditionError`, `runProgram` → 3 with "the preview itself failed: …"; the launcher's self-test: a program that did not build → 3, nothing run. F11, F12, L2 |

**Tests (9).**

| # | Finding | Disposition | Evidence |
|---|---|---|---|
| T1 major | the launcher's real path is untested (N40–N45 survived) | **FIXED (first fix run):** `main()` is now `self-test` or `run(argv)`, and the self-test runs `run()` end to end with a STUB program (`build_fn` and the binary injected): the stub writes its argv and its rlimits; the self-test checks the address-space, output, CPU and core bounds in the stub (each the bound `suite_run` applies under the caller's own), `--place=X,Y,DEG` arriving as `--place X,Y,DEG`, `--stamp` = the commit read now naming HEAD, `--autons-hash` = robot two's file's, `--robot one` passed through with NO hash while robot one's file is absent (never robot two's), the exits 1/2/3 passed back, a stub that outlives a 1 s timeout → 1 + the timeout and no-page sentences, a SIGKILLed stub → 1 + "killed by signal 9" + no-page, a failed build → 3 with nothing run. `main` itself is the two-line dispatch; nothing else in it to mutate | N40–N45, L1, L2, RM20 (below) |
| T2 major | FakesByKind's flywheel cannot reach its target (a false TIMED_OUT); no test runs a speed mechanism | **FIXED at the root (first fix run; the root cause re-derived here):** a declaration's `kA` is the acceleration FEEDFORWARD (starting value 0), not the inertia; a plant built with kA = 0 is memoryless, so a speed loop reading one tick late diverges for kP/kV > 1 (robot x: 1.57) — a plant artifact no real flywheel has. FakesByKind gives an ASSUMED τ where none is stated (Δ16, HA-183's invented first-order constant); this run found a Flywheel's words cannot state `kA` at all (`SpeedTuning`), so EVERY declared flywheel runs on it — the PLACEHOLDER note, guide 16 and the banner now say so ("a flywheel's declaration has no number for it") | new B1 case: robot x's flywheel to "shoot" through `previewEntry` → SUCCEEDED in ≥ 0.1 s (its at-speed window) and ≤ τ·ln(3000/60) + 0.15 s, its note naming the assumed 0.2 s. N27, N28, F13, F14 |
| T3 major | the page and table content mostly unasserted | **FIXED (first fix run):** RP39 checks each thing against a value computed INDEPENDENTLY (its own pose formatter and pixel transform): row 1's seven cells, start before end, "to" is the end; one start and one end outline per motion; the step-1 number's pixel and tooltip; the timeline's park (658.0) and hard-stop (706.0) lines and its 30 s tick; the summary's "of the 30 s period"; the failed step's `seg failed` in ITS group, a good step's `seg ok`; the THROUGH ring at the THROUGH sample and the faded part starting there with exactly the remaining samples; a TOUCH ring; a failed mechanism's sentence on stdout | N1, N16–N20, N25, N46–N49 |
| T4 minor | field-check branches and edges untested | **FIXED (first fix run):** RP40 on hand-made paths: every wall 0.5 in past (TOUCH) and 1.5 in past (THROUGH) its INNER face, depth equal to the overshoot; the equality edge landed on EXACTLY (`nextafter` until the penetration is exactly 1.0 → TOUCH; the next double past → THROUGH); a 45° square whose bounding box overlaps a Loader and whose square does not → no note; the start pieces driven over twice → one GREY note per piece (2 Cups, 1 Pin) | N6, N7, N7b, N8, N24 |
| T5 minor | the background equality edge never landed on (N22) | **FIXED (first fix run):** RP41's fixture, a drive straight into a claw: `REQUIRE(drive.s1 == claw.s0)`, not background, no "(in the background)", no span | N22 |
| T6 = D10 minor | no curve through the preview | **FIXED (first fix run):** RP42 — `curveTo` then a drive: one Curve step labelled "curve", number 1, SETTLED; its segment is EXACTLY the samples between its start and end through the independent transform; its × at its end target; the path really curves (> 1 in off its chord) | N15 |
| T7 minor | a throw inside a step, cut marks, --place validation, the ignored notice, substring lookup, the flood in the log, the any-starting-value rule | **FIXED (first fix run; one assertion sharpened here):** RP9b (N2); RP43 — every CANCELLED step of the cut skills fixture is Cut, none Failed (N9, N10); RP37 — `1,2,3x`, `nan,0,0`, `0,inf,0`, `1,2`, `a,b,c`, `` → exit 2, nothing written; a FIELD auton with `--place` says it is ignored (N13, N14, N23); RP32 — `long`, `too`, `oo long` → exit 2 "no auton named" (N38); RP27 — the page's log under 300 lines (N4); RP44 — a lift stating only kP is hatched, one stating kP, kV, speed and tolerance is not (N11). **N12 would still have survived** (robot x's flywheel is hatched by its assumed inertia anyway): RP44 now holds robot x's flywheel note EQUAL to the full sentence naming HA-182 and the assumed τ, a flywheel stating kP and kV names the τ but not HA-182, and on the default rig names nothing | N2, N4, N9–N14, N23, N38 |
| T8 minor | B1's constants case leaves kA / encoderZeroDeg unchecked; robot one's shape has no fixture | **FIXED (first fix run; one gap closed here):** a constants case on an arm homed at 30° (level 45°, stated kS and kA): kS, kA (× rad→deg), levelDeg, cosine gravity, the stops, startDeg and **encoderZeroDeg = 30**; robot x's lift and flywheel kA = τ·kV. A robot-y case: the STALL-homed swing homes and moves, the linkage dr4b moves, and a cascade-like lift declared in the test (stall-homed, MEASURED shape, a current cap — ROB1-A's shape) homes and reaches L1 within [profile, profile + 0.5 s]. **Added here:** N32 (the piece switch reads a piece) SURVIVED in the test reviewer's own log but no finding named it — robot y's three piece sensors (optical, limit switch, distance-confirmed grab) never see a piece: the roller and the catapult run "until a piece" TIME OUT, the grip is UNCONFIRMED | N29, N30, N32, N32b, N32c, RX5, RB2 |
| T9 minor | `tools/suite_run.py` still says "The suite does not fork" | **FIXED (first fix run):** its header names RP27's and RP38's forked children, their 2 GiB bounds and alarms (60 s, 1 s), and that each is waited for. **This run's grep found a second copy:** `docs/internal/RELEASING.md` :103 "— it does not fork" — fixed to name the two cases (an internal doc, not a gate input) | prose review below |

**Docs (10).**

| # | Finding | Disposition | Evidence |
|---|---|---|---|
| D1 major | the glossary's "host test plant" defines the whole plant as the preview's ideal one; chapter 5 contradicts it | **FIXED (first fix run):** the general definition (ideal unless a test adds lag, a weak side, slip or noise; walls, pieces or a camera's view only where a test puts them); the preview runs it IDEAL; "not a floor or a ceiling" kept; Δ7 updated | read beside chapter 5 :196 and chapter 19 :14 ("the suite's model of a robot") — consistent |
| D2 major | the prose review's "no internal tags" claim was false; two RP35 descriptions stale | **FIXED:** the tag and "three" (C7); the false line in the prose review is struck with the reason (a rule-4 breach — the grep was not run over `test/README.md`) | the struck line above; the re-run grep below |
| D3 major | STEP1-B landed on main during this run (API 2.8, triggers); placement, guide 16, §1.6's tick and STEP1-B's new lines unreflected | **NOT fixable in this tree — handed to the coordinator with evidence.** This worktree's base is `d33d25d`: it has no `atPercent`, no `## API 2.8`, no trigger lines, so a case or a guide sentence naming them would not compile or would describe an API the tree lacks. **Probe on main's tree instead** (`scratch/rpv1-a/merged/`: `git archive shulib-v2` at `5f6633b` with this run's preview files overlaid, each copied only when its checksum equalled the pre-mutation one; `trigger_probe.cpp`, robot two): a trigger that FIRES (`r.atPercent(50).startRun(clawPivot6, 6_volt, 0.4_s).moveTo(…)` then `waitUntilDone`) is drawn as the R4 background span from its FIRING pose (20.4 in, T+0.34) to its verdict (SUCCEEDED, T+0.75), numbered from below between the drives — §1.6's tick in all but name; a trigger that NEVER fires (`afterDistance(50_in)` on a 40 in drive) shows the library's own lines — "trigger after 50.0 in on 'claw pivot 7' did not fire: the drive was 40.0 in" (grey) and "STOPPED at step 2 (claw pivot 7): mechanism NOT STARTED (the trigger never fired: the robot did not reach the mark)" (red, the ✕ on the field). So nothing breaks at the merge. **At the rebase (coordinator):** (1) move this changelog entry to the top of `## API 2.8` (still "No API change"); (2) add the trigger form to guide 16's background-start sentence (`r.atPercent(60).startMove(lift["high"], 2_s)` is drawn the same way, from where it fired); (3) turn the probe's two autons into a case (and consider the "did not fire" line amber, not grey — it is a warning when no `waitUntilDone` follows); (4) re-run the suite on the merged tree. The PROGRESS coordinator note ("STEP1-B adds its own") is corrected above | `scratch/rpv1-a/merged/out/fires.txt`, `never.txt` and their pages |
| D4 = C5 | D5's end numbers and times | Δ14 (above) | — |
| D5 minor | the "which steps are counted" sentence was incomplete | **FIXED (first fix run):** counted — every motion (drives, curves, turns, brakes, holds) and mechanism step; not counted — pauses, waits (`waitFor`, `waitUntilDone`), `then(...)` and `startAt` — on the page (`kNumberingLine`), guide 16, the model's banner and the changelog. **This run added strafes** (`Routine::strafeTo` starts a StrafeTo motion, counted like any) — see "after the mutation run" below | the four texts read together |
| D6 minor | guide 16 implied a Toggle can be gone THROUGH | **FIXED (first fix run):** the TOUCH bullet — "or overlaps a Toggle by any amount (… the preview never calls it THROUGH)"; Δ3's false "said in guide 16" struck with the date | guide 16's TOUCH bullet; `boxRole` (Toggles → Touch) |
| D7 minor | `fakes_by_kind.hpp` still said "moves EXACTLY as its declared numbers predict" | **FIXED (first fix run):** "its time is the declared profile plus the controller's 'reached' window, NOT exact (RP24 …)" | grep `-i exactly` over the new files: only "exactly as the tester runs it" (not a time claim) |
| D8 minor | chunk tags in student-facing output (ROB1-A, TUNE1) | **FIXED (first fix run):** "…are not in the code yet…", "'shulib Tune' sets the real ones on the robot" (Δ17) | grep for `ROB1\|TUNE1\|RPV1\|G1 ` inside string literals of the preview's files: none |
| D9 minor | the outline's 15 in and 1 in hard-coded in the page sentence | **FIXED (first fix run):** `outlineLine()` prints both from `kPreviewFootprintIn` / `kThroughAllowanceIn` (`trimmed`); RP15 asserts that sentence, formatted from the two constants, on every page. (A literal that happened to equal the constant would pass — inherent; the code is read, not trusted) | RP15 |
| D11 minor | exit-code docs missed three cases; "Hatched" vs "dashed" | **FIXED (first fix run):** guide 16 and the launcher's docstring: 1 includes a bound's cut (nothing drawn), 2 an unknown name lists the names / an unknown robot the robots, 3 the build, the page or the preview failed; guide 16 says "Dashed (a dashed diamond, `[PLACEHOLDER]` in the table)", the changelog "dashed as PLACEHOLDER" | guide 16 "Exit codes"; `grep -i hatch docs/` — no hit in this run's text |

### Mutations — the review round, on the final tree (`scratch/rpv1-a/fix/mutate2.py`, log `fix/mutations2.log`)

The driver: each mutation backed up under `scratch/rpv1-a/backups2/`, applied (each replacement matching exactly once —
`--check` first: 88/88), the suite binary rebuilt, **every object whose compiler `.d` file names the mutated file
checked as recompiled** (1 TU for the preview's and FakesByKind's headers; all 9 for RB1's `auton_tester_rig.hpp`;
a missing one would void the verdict — none did), EVERY route preview case run under `ulimit -v 4194304; timeout 600`,
the file restored in a `finally` and on SIGTERM/SIGHUP with its timestamp bumped; launcher mutations run its
self-test under the same caps. Checksums of every mutated file taken before the run equal after it; `git diff --
src/ Makefile` empty. **88 mutations in 90 runs (N2 and F7 twice), 90 RED, each by a test's own assertion** (RM10's by RP8's `REQUIRE_NOTHROW`, as
designed; the rest by CHECK/REQUIRE lines, or the self-test's FAIL lines). None RE-CUT, none VOID.

- **The first run's 29, re-run** (RM1–RM5, RM8–RM18, RM20–RM25, RX1–RX5, RB1, RB2; RM12 re-cut — the matched block now
  begins with `removeOldPage()`): all RED.
- **The test reviewer's survivors, re-cut for the final tree**, each with the case that now catches it: N1 (hard-stop
  at the park time) RP39 · N2 (no "inside") RP9b · N4 (the flood in the page's log) RP27 · N6 (world axes only) RP40 ·
  N7, N7b (left / blue wall at its outer face) RP40 · N8 (start pieces amber) RP40 · N9 (a cut mechanism red) RP43 ·
  N10 (a cut motion red) RP9b (a) · N11 (ALL starting values) RP44 · N12 (a Speed mechanism never on its starting
  values) RP44 — **only after this run's exact-note assertion**; it would have survived the first fix run's checks ·
  N13, N14, N23 RP37 · N15 RP42 · N16, N17, N18, N19, N20 RP39 · N22 RP41 and RP36 · N24 RP40 (the equality edge,
  landed on) · N25 RP39 and RP36 · N27, N28 the B1 flywheel case · N29, N30 the B1 constants case · N32 (raised here),
  N32b, N32c the robot-y piece sensors · N38 RP32 · N40–N45 the launcher's self-test (N40, the program run unbounded:
  under a caller's 4 GiB `ulimit` the address-space line alone cannot tell — the stub inherits 4 GiB either way — so
  the output, CPU and core lines are what fire; recorded so nobody reads the AS line as the catch) · N46–N49 RP39.
- **The fixes' own logic:** F1 (the begins not noted at the end-action line), F2 (the end action by TIME — the
  reviewed defect), F3 (a MECH-line step always the park's), F4 (the routine's last motion counted as the park's),
  F5 (an entry's own end action's claw never the park) — RP36 (and RP6, RP13, RP11 for F4); F6 RP9; F7 (no boundary
  ever seen during an unwind) RP9b (b), (c), (d); F8 RP37; F9 RP38; F11, F12 (exit 2 again) RP38; F13 (the flywheel
  with no inertia — the reviewed false TIMED_OUT) the B1 flywheel and constants cases; F14 (an arm with none) the
  constants case; F16 (the flywheel's assumed τ never named) RP44 and the flywheel case; F19 (no verdict dot) RP39;
  **F20** (a throw inside the END ACTION worded as inside a routine step) RP9b (d) — run in a second batch after (d)
  was added, with N2 and F7 again (all RED); L1 (the killed run's no-page sentence dropped), L2 (a failed build exit
  2) the launcher's self-test.
- **One false start, not counted:** the first try at the second batch built nothing — my guide 16 edit re-ran the
  doc gate against a briefing whose suite line predated the piece-sensor case, so all three "RE-CUT"; the file was
  restored (checksum equal), the briefing regenerated around a fresh link, and the batch re-run (above).

**After the run, two edits outside every mutated line:** "strafes" added to the counted motions (`kNumberingLine` in
`route_preview_render.hpp`, the model's banner, guide 16, the changelog — `Routine::strafeTo` starts a StrafeTo
motion, counted like any other; no test reads that literal), and RP9b (d) (new assertions only). The final gates
below ran on that tree.

### Prose review (rule 4), the review round — greps run on the final text, hits read BEFORE this was written

- **The brief's §3.3 list** over the new files, guide 14/15/16, guide README, `test/README.md`, the changelog entry and
  the touched header: `simulator` — none (and none in any page under `build/route_preview_test/`, nor "http");
  `lower bound` — none; `exact|guarantee` near a time — three hits, all text older than this run (glossary :260 the
  end action, :357 the watchdog; guide 16 :132 the hard stop at 30 s); `robot two` in guide 16's new section — the
  side sensors, the claw's 0.5 s, and "**stand-in drive** … not robot two's drive": every drive claim says stand-in;
  `internal/|chunks/|RESUMING` — none; `\b(RC|RP|RM)[0-9]` in the public docs AND `fake_telemetry_sink.hpp` — **none**
  (this time `test/README.md` was in the grep; its section heading "(since RPV1-A)" follows the file's own "(since
  VIS1-A1)" — a chunk name, not a test tag).
- **This round's concepts:** `kA` over guide/cookbook — guide 18 :448 already says "the robot file cannot say a
  flywheel's", matching the new wording; `host test plant` over the guide — chapter 19 :14 ("the suite's model of a
  robot") and chapters 5, 6, 9, 11, 12 (lag, weak sides) agree with the new glossary entry; exit codes — guide 16 and
  the launcher's docstring both list 0/1/2/3 the same way, the changelog says "0 / 1 / 2 / 3"; `hatch` — no hit in this
  run's prose (the word survives only as the CSS class); `does not fork|never forks` — `tools/suite_run.py` (fixed by the
  first fix run) **and `docs/internal/RELEASING.md` :103, fixed here** (found by this grep, not by a reviewer);
  `three runs|RP35` — only the rig banner's "four runs"; chunk tags inside string literals of the preview's files
  (`ROB1|TUNE1|RPV1|G1 `) — none; `-i exactly` in the new files — "exactly as the tester runs it" only.
- **Documents touched this round:** guide 16 (reflow; the flywheel's `kA`; strafes; "If your code throws"), the
  changelog entry (reflow; the throw's placement; every flywheel's assumed τ; strafes), `docs/internal/RELEASING.md`
  :103, this log. **Deliberately not touched:** guide 18 (its flywheel `kA` sentence is already right); the HA register
  (the assumed τ values are instances of HA-183's invented first-order constant, τ = kA/kV — no new belief; if the
  coordinator reads it otherwise, it is a row for A1's batch beside HA-251); `docs/roadmap.md` / `build-order.md` (the
  coordinator's at the merge); `docs/api/` (no `///` changed under `include/`; check-fresh agrees).

### Gates — the review round's final tree (2026-11-13 11:40–12:55 EST)

- **Cases:** the route preview now has **40** (the first run's 27; the first fix run's RP36, RP9b, RP37–RP44 and three
  B1 cases; this run grew RP9b by (c) and (d), RP44 by the exact flywheel notes, the robot-y case by the piece
  sensors — no new TEST_CASE): `-tc="*route preview*"` → 40 passed, 753 assertions.
- `suite_run.py green` → **green — 3412 cases, 3780154 assertions** (3399 + 13), on a fresh link: binaries removed,
  the briefing generated with the suite unbuilt, built, generated again, rebuilt — done twice, the second time
  after the src gate's self-test re-dated `src/` files with its byte-exact restores.
- `briefing_status.py` self-test OK (38 cases); `check` → current. `doc_staleness_audit.py` self-test OK (16 cases);
  run → clean, the suite numbers derived (an earlier run of it, between the two src-gate runs, distrusted the
  binary as older than the restored `src/` files — the relink above answered it).
- `api_doc_tool.py` self-test OK; check-coverage 6062 entities across 206 headers, all documented; check-fresh exit 0
  (no `///` changed under `include/`, so no `generate`); check-examples 762 quoted lines verbatim; check-removability
  clean.
- **ARM header gate:** 249 headers in one TU (`scratch/rpv1-a/fix/all_headers.cpp`), CI's exact flags, `-Werror` →
  exit 0. **CI greps:** the library PROS-free outside `hal/pros/`; the core sim-free.
- **Compile refusals:** `ctest -R compile_refusal --no-tests=error` → 258/258 passed; 258 TUs = 258 CTests.
- **src gate:** `self-test` → OK (306 detector cases, each against a real build; 12:14:58 → 12:44:27 — a first try
  under my own 1800 s `timeout` was KILLED at it on a machine loaded by two other sessions' gate runs, exit 124,
  and left nothing behind); `check` → **PASS, all 8 builds link**. `git diff -- src/ Makefile` EMPTY and `git status
  --ignored -- src/ Makefile` EMPTY after each; `git diff d33d25d -- include/` = `hal/fake/fake_telemetry_sink.hpp`
  only.
- **The launcher:** `route_preview.py self-test` → ok, 35 checks, 0 failures (the stub runs included). End to end
  through the real launcher on the team's autons (`scratch/rpv1-a/fix/e2e/`): `"40in forward"` exit 0 (18,889 bytes;
  its table `1 | drive | (0.0, 0.0) 0.0° | (39.8, 0.0) 0.0° | 0.00 | 1.80 | SETTLED`, then the park);
  `"TEST 24in turn back" --place=-48,-48,90` exit 0 (31,312 bytes, stamped `v0.1.1-415-gd33d25d-dirty · autons file
  2f45cb1053f1`); `"no such auton"` exit 2, the two names listed, nothing written; `--list` exit 0;
  `"40in forward" --robot one` exit 2 with the no-build sentence (no chunk tag); RP17 by hand — two launcher runs,
  `cmp` identical; every page: 0 "http"/retired-word hits, "IDEAL host test plant" once.
- **Mutations:** above — 88 mutations, 90 runs, 90 RED, all restored.

### For the coordinator (the review round adds)

- **At the rebase onto main (`5f6633b` or later):** (1) this changelog entry to the top of `## API 2.8`, still "No API
  change"; (2) guide 16's background-start sentence gains the trigger form; (3) the merged-tree probe's two autons
  (`scratch/rpv1-a/merged/trigger_probe.cpp`: a trigger that fires, one that never does) become a case, and the
  "did not fire" RTN line may deserve amber; (4) the suite on the merged tree. Where main touched this run's shared files
  (`git diff d33d25d shulib-v2`, read hunk by hunk): `docs/changelog.md` (its `## API 2.8` section — item 1);
  `test/README.md` (13 lines at :140 — its camera section; this run's new section starts after it, so the two meet
  at :141–:144: a context conflict at worst); guide 14 (hunks at :263 and :402 — this run's bullet is at :62);
  `docs/guide/README.md` (a chapter-20 row after row 19 — this run edits row 16); `docs/internal/RELEASING.md`
  (:139 — this run's sentence is :103). `test/auton_tester_rig.hpp`, `test/CMakeLists.txt`,
  `fake_telemetry_sink.hpp`, `tools/suite_run.py`, guide 15 and guide 16: untouched on main. Guide 18 :448 on main
  still says "the robot file cannot say a flywheel's" (kA) — this run's wording agrees.
- **HA:** no new row — the assumed time constants are HA-183's invented τ = kA/kV (your call if you read it as a new
  belief; it would sit beside HA-251 in A1's batch).
- **Exit 3** is new against the brief's 0/1/2 (Δ15) — A1's CI check should treat 3 as "the tool broke", never as a
  route's failure.
- **Reviewers and a fresh verifier** for this round are the workflow's next steps; nothing here was reviewed by anyone
  but me. **Nothing committed.**
- **A slip, corrected:** this run first wrote the five `api_doc_tool.py` gate logs to the session's scratchpad under
  `/tmp` (the task forbids `/tmp`); they were moved to `scratch/rpv1-a/fix/gates/` and nothing of this run is left
  there. Every other probe, log and backup of this round is under `scratch/rpv1-a/` (`fix/`, `backups2/`, `merged/`).

## Verification-round findings and their disposition

A fresh verifier returned four minor findings (its scratch: `scratch/rpv1-a/verify/v2/` — probe `probe/vprobe.cpp`,
its mutations `vmuts_own.py`, log `vmut.log`). This round's scratch: `scratch/rpv1-a/vfix/` (checksums of every file
before an edit: `vfix/before.md5`).

### Log (appended as it happened)

- 2026-11-13 16:24 EST — started. `git diff -- Makefile src/` empty; the tree as the review round left it (the suite
  binary 12:51). Each finding re-checked against the code before any edit (below).
- 16:24–16:32 — every finding re-checked against the code first: F1 by reading (`MotionLog` is the only thing
  that notes an unwind, and only for motion boundaries; a mechanism step's CANCELLED line comes from
  `RunGuard::RunUnwindGuard`'s cancel-all → `Slot::cancel()` → `finishStep`, during the unwind, and nothing
  looked at it) and the verifier's `out/claw150.txt` (`#2 claw pivot 6 verdict=CANCELLED ... open=0`); F2 by
  `grep -n "during a wait" test/route_preview_test.cpp` (no hit); F3 by reading `previewEntry`'s `onNoPace`
  (`std::_Exit(1)` after a writer whose result was dropped) and the verifier's `out/forever.txt` (rc=1, "could not
  write the page"); F4 by reading the B1 constants case (`l.kA == Approx(kAssumedJointTauS * l.kV)`, derived
  from the constant). **All four REAL.** Edits, then the new cases on a mini binary (`vfix/mini.sh`:
  `route_preview_test.cpp` compiled fresh against the worktree): 40/40, 846 assertions — except (i) below,
  which FAILED first (4 assertions) and found a fifth defect.
- **A fifth defect, found by RP9b (i) before its fix:** a mechanism step was marked "in the background" (the
  table's "(in the background)", the page's "started here in the background" tooltip) when ANOTHER MECHANISM
  overlapped it, though the robot did not drive: `foldRun`'s overlap loop ran over `steps`, which by then held
  the mechanism steps already placed as well as the motions. Measured: pivot 6 run blocking while pivot 7 (started
  earlier) still ran, no drive → `CHECK_FALSE(is[2].background)` failed, "inside step 2" instead of 3, the table
  cell "(in the background)". Fixed at the root (only a motion counts as driving); the four assertions pass.
- **A sixth, a false comment, corrected (no behaviour):** the sort's comment said a start form then its drive is
  "the only way two steps begin on one pace". Two start forms back to back also begin on one pace, and two
  mechanisms begun on one pace are numbered in their robot's LIST order (`noteBegins` reads the slots in order),
  which may not be the code's. Not fixable from below (a step's begin logs nothing; A2's listener ends it) — so it
  is now SAID (the comment, guide 16's "Why the numbers ...") and PINNED (RP28's new block: `startRun(pivot 7)`
  then `startRun(pivot 6)` → pivot 6 is step 1, pivot 7 step 2, same sample).
- 16:35–16:46 — the mutation run (below), the build, the fast gates (`vfix/gates.sh`, logs `vfix/gates/`), the
  launcher's end-to-end runs (`vfix/e2e/`); 16:46 guide 16's Amber bullet corrected ("names the step it happened
  inside" → "says where it happened": in (h) below a background pivot is amber, cut by a throw the PROGRAM ERROR
  line places "during a wait"); 16:46 `src_build_gate.py self-test` started. **The session then ended** (~17:02).
- **RESUMED 2026-11-13 23:31 EST** (`date`: Fri Nov 13 11:31:58 PM EST 2026). Found and fixed before anything
  else:
  - **A src-gate self-test plant LEFT APPLIED:** `git diff -- src/ Makefile` showed `src/build_selection.hpp`'s
    tank-drive beacon as `"shulib-robot-variant=tank-library"` (mtime 17:02:40) — self-test 12's
    `_replace(SELECTION_PATH, ...)` (`tools/src_build_gate.py` :2367), killed mid-plant when the session ended, so
    its byte-exact restore never ran. Restored with `git checkout -- src/build_selection.hpp` (a fresh write: the
    next build recompiles everything that includes it); `git diff -- src/ Makefile` then empty. No
    `src_build_gate` process was left running (`ps`).
  - **Every file the mutation driver backed up compared against its backup** (`vfix/backups/`, and
    `vfix/pre_mutation.md5`): `route_preview_run.hpp` and `fakes_by_kind.hpp` IDENTICAL; guide 16 differs from its
    pre-mutation copy only by the 16:46 Amber edit above (`diff` read: four lines, no mutation's text).
  - **One stale sentence the 16:46 edit had not reached:** the page's own legend still said "the PROGRAM ERROR
    line names the step" — false for (h) (the amber pivot is not named: the line says "during a wait, after step
    2"). Now "says where it happened", as guide 16 (`route_preview_render.hpp`'s legend).
  - The suite binary relinked from source before any result was trusted: `rm -f build/test/shulib_tests`, the
    briefing regenerated (reads it as unbuilt), a full build, the briefing regenerated again (Gates, below).

### Dispositions

| # | Finding | Disposition | Evidence |
|---|---|---|---|
| V1 minor | C3 not exact for a throw during a BLOCKING mechanism step ("after step N" / "during a wait, after step N") | **REAL — FIXED at the root.** The sink's line hook now notes the index of every MECH line stored while an exception is in flight (`Recorder::mechLinesInUnwind`); `foldRun` marks the mechanism step that line closes (`endedByUnwind`), and a step so ended is `openAtThrow` when the robot did NOT drive during it — a mechanism the robot drove during was started in the background and the code had moved on past it (without that rule, (h) below would say "inside" a background pivot). The run's header (WHERE A RUN THREW) states the rule and its one limit from below: a background start followed by a pause or a `waitUntilDone` with no drive between reads as inside it (from below it IS a blocking step: the code waiting while it runs). The legend's amber now says "cut short (by the deadline, or by a throw: the PROGRAM ERROR line says where it happened)", guide 16's Amber bullet likewise (both first said "names the step" — false for (h); corrected at 16:46 and in the resumption, log above); guide 16's "inside a step" names mechanism steps and excludes one the robot drove during | RP9b (e), (f): `r.run(clawPivot6, 6_volt, 3_s)` after a drive, thrown into from ABOVE and from BELOW the recorder (`ThrowWhenStoppedLayer`, 5 paces after the drive) → exactly "PROGRAM ERROR inside step 2 (counted from below)", pivot 6 CANCELLED, `openAtThrow`, not background, cut in < 0.5 s; the drive SETTLED, not open. (h) and (i) below. Mutations VF1–VF5 |
| V2 minor | the "during a wait" branch had no case (VM1 survived) | **REAL — FIXED (a case).** | RP9b (g): drive, claw opened (SUCCEEDED), `pause(1_s)`, a throw from BELOW the recorder with nothing running → exactly "PROGRAM ERROR during a wait, after step 2 (counted from below)", the claw not open; the same from ABOVE (never inside a pace) → exactly "PROGRAM ERROR after step 2 (counted from below)" — the other side of the documented rule. (h): pivot 7 started in the background, a drive during it, a pause, the throw → "during a wait, after step 2", pivot 7 CANCELLED by the unwind but background and NOT open. VM1 re-run RED |
| V3 minor | the no-pace stop exits 1 when its page cannot be written | **REAL — FIXED at the root.** `PreviewOptions::onLoop` returns whether the page was written (runPreview's returns `writeOutputs`'s result); the stop ends with `std::_Exit(written ? kExitProblem : kExitToolFailed)` (the exit constants moved above `PreviewOptions`). The run's header (THE NO-PACE STOP), the launcher's docstring and the field's comment say 1 or 3 | RP38's new forked child: RP27's "forever" with `--out` under a FILE → exit **3**, no page, and its stderr (redirected to a file in the child: the stop ends the process) holds "route preview: could not write the page to …/forever.html". VF6 (the reviewed defect), VF7 (the writer's result dropped) RED |
| V4 minor | nothing ties guide 16's "0.05 s for an arm or lift" to `kAssumedJointTauS` (VM3 survived) | **REAL — FIXED, both ways.** Rather than a literal (which pins the code to the test, not to the docs), the B1 constants case now reads guide 16 and `fakes_by_kind.hpp`'s banner as prose (comment markers dropped, whitespace squeezed: a reflow is not a failure) and requires each sentence to quote the constants in their shortest exact form (`std::to_chars`, never a rounded one): guide 16 "the fake assumes it (0.05 s for an arm or lift, 0.2 s for a flywheel)" and "a flywheel spins up on an ASSUMED 0.2 s time constant"; the banner "kAssumedJointTauS (0.05 s;" and "kAssumedWheelTauS (0.2 s;". A constant changed without its sentences, or a sentence without its constant, fails | VM3 (0.05 → 0.15) and VM3b (0.2 → 0.25) RED; VD1 (guide 16 alone says 0.15) and VD2 (the banner alone says 0.08) RED |
| V5 (found here) | a blocking mechanism step marked "in the background" when another MECHANISM overlapped it | **FIXED at the root** (log above): only a motion is driving | RP9b (i): pivot 7 started, a 0.2 s pause, a blocking pivot 6 → pivot 6 not background, no "(in the background)" cell in any table row, "PROGRAM ERROR inside step 3"; pivot 7 (no drive during it) also open — the header's limit — and the LATER-begun of the two is where it threw. VF4, VF5 |
| V6 (found here) | the sort comment's "the only way two steps begin on one pace" | **Comment corrected; the limit SAID and PINNED** (log above) | RP28's new block ("two starts") |
| V7 (found in the resumption's prose review) | guide 16 "Which robot" said the fake assumes a time constant "(0.05 s for an arm or lift, 0.2 s for a flywheel), and the page says so" — the page says so only for a FLYWHEEL: `placeholderFor` names the assumed τ for a Speed mechanism alone (an arm's time is mostly its declared profile — `fakes_by_kind.hpp`'s banner, the review round's decision), so a tuned lift with no `kA` is drawn with no note | **REAL — the GUIDE corrected** (the decision stands; the sentence over-claimed it): "A flywheel's spin-up time is mostly that number, so its steps are dashed and the page names it; an arm or a lift moves mostly in the time its declared speeds predict, so its assumed number is not marked." The pinned clause ("the fake assumes it (0.05 s for an arm or lift, 0.2 s for a flywheel)") kept whole | Both halves were already pinned by RP44 (the tuned lift with no `kA` → no note; the tuned wheel → "with an ASSUMED 0.2 s wheel time constant") — now shown to bite: VR1 (an arm or lift with no `kA` marked) and VR2 (the wheel's τ never named) RED; VD1r (the pinned clause, re-cut on the edited paragraph) RED |
| V8 (found in the resumption) | the page's legend: amber "cut short (by the deadline, or by a throw: the PROGRAM ERROR line names the step)" — false for RP9b (h): a background pivot cut by a throw is amber while the line says "during a wait, after step 2"; guide 16 had been corrected at 16:46, the legend had not | **Wording corrected** (`route_preview_render.hpp`): "... the PROGRAM ERROR line says where it happened", as guide 16. **No case added:** a legend sentence is fixed wording with no logic behind it, and pinning one string to another tests the typing, not the tool; the prose review (below) is its guard | every page of the end-to-end runs carries the new legend once and the old one nowhere (Gates, below) |

### Mutations — the verification round (`scratch/rpv1-a/vfix/mutate3.py`, log `vfix/mutations3.log`)

The driver: each mutation backed up under `vfix/backups/`, applied (each replacement matching exactly once —
`--check` first: 20/20), the MINI binary rebuilt (`vfix/mini.sh`: `rp_test.o` removed and recompiled from the
worktree every time — the only object in it that includes a mutated file; `c++ -M` of `autons.cpp` and
`test_main.cpp` names none of them, logged in the run's first line), `rp_test.o` checked newer than the mutation
(all 18 code mutations: yes), EVERY route preview case run under `ulimit -v 4194304; timeout 600`, the file restored
in a `finally` and on SIGTERM/SIGHUP/SIGINT with its timestamp bumped. Checksums of the three mutated files before
the run (`vfix/pre_mutation.md5`) equal after it; `git diff -- src/ Makefile` empty. **20 mutations, 20 RED, each
by a test's own CHECK/REQUIRE line** (none killed by a cap, none RE-CUT):

- **This round's fixes:** VF1 (a MECH line during an unwind never noted) RP9b (e)/(f) — "after step 2" again;
  VF2 (a mechanism the robot drove during counted) RP9b (h) — "inside step 1"; VF3 (every mechanism verdict counted
  as ended by the unwind) RP9b (g) — the SUCCEEDED claw read as "inside step 2"; VF4 (background judged against
  other mechanisms again — V5) RP9b (i); VF5 (the FIRST open step, not the last-begun) RP9b (i) — "inside step 2";
  VF6 (the no-pace stop exits 1 whatever its write said — the reviewed defect) and VF7 (the loop writer's result
  dropped) RP38's new child — exit 1, not 3.
- **The verifier's:** VM1 (the "during a wait" branch never taken — SURVIVED for the verifier) RP9b (g) and (h);
  VM2 (every motion open) RP9 and RP9b; VM3 (`kAssumedJointTauS` 0.05 → 0.15 — SURVIVED for the verifier) the B1
  constants case's guide and banner lines; VM3b (`kAssumedWheelTauS` 0.2 → 0.25) the same, RP44 and the flywheel case.
- **The docs side of V4:** VD1 (guide 16 alone says 0.15 s) and VD2 (the banner alone says 0.08 s) — the B1 constants
  case's guide line, its banner line. **VD1 was first VOID and is re-run:** a docs mutation needs no rebuild, so the
  driver ran the binary left by the mutation before it — VM3b's MUTANT (its errors were VM3b's flywheel lines). The
  driver now rebuilds from the clean tree before a docs mutation; re-run: RED by the guide line alone (857/858).
- **The review round's C3 / C8 family, re-run around this round's edits:** F7, N2, F20 RP9b; F9, F11, F12 RP38;
  RM22 RP27 and RP38.

### Prose review (rule 4), the verification round — greps run on the final text after the resumption, hits read BEFORE this was written

Changed this round: `route_preview_run.hpp`, `route_preview_render.hpp`, `route_preview_test.cpp`,
`route_preview_fixtures.hpp`, `route_preview_model.hpp`, `tools/route_preview.py`, guide 16,
`fake_telemetry_sink.hpp` and `test/README.md` (comments only, the resumption). Searched `docs/`, `README.md`,
`test/README.md`, the launcher (the RPV1 brief and log excluded):

- `PROGRAM ERROR` → guide 16 :190 (the Auton Tester's words), :399 (Amber — now "says where it happened"), :468
  (the throw paragraph: inside a drive, turn or mechanism step, never one the robot drove during; during a wait;
  after the last step that began — matches WHERE A RUN THREW); guide 17 :248, :367 (the competition program's word —
  not the preview's); the rest are other chunks' history records. The legend (`route_preview_render.hpp`) said
  "names the step" — fixed (V8).
- `during a wait|inside step|inside a step|after the last step|counted from below|in the background)` → guide 16
  :471 (true); changelog :37 "placed inside a step, during a wait, or after a step" (true; general enough to hold
  for a mechanism step — not changed); the other hits are unrelated (build-order, T3, TUNE1 logs).
- `could not write the page|keeps looping|no-pace|exit 3|kExitToolFailed|EXIT_TOOL_FAILED` → guide 16 :462 (the
  page is written ... the command fails) and its exit-code paragraph (3: "the page could not be written") — true
  for the stop now; the launcher's docstring :19 ("exit 1 with the page written, or 3 when the page could not be
  written") and :27–:32 — true; `test/README.md` :153 — true; `docs/api/motion_scheduler.md` :1384 unrelated.
- `assumed … time constant|kAssumed|0.05 s for|ASSUMED` → guide 16 :406 (Dashed: the flywheel's 0.2 s — true),
  :450 (**over-claimed for an arm or lift — fixed, V7**); `test/README.md` :162, changelog :46 — true; guide 15
  :176 unrelated.
- `list order|one pace|Why the numbers` → guide 16 :431–:438 (two starts on one pace numbered in the list's order —
  true, pinned by RP28); every "one pace" elsewhere is the deadline's (guides 09/12/14, changelog) — unrelated.
- `setOnLine|line hook|FakeTelemetrySink` → `fake_telemetry_sink.hpp`'s banner and `test/README.md` :170 named only
  the no-pace stop as the hook's use; the recorder also notes the guard's end-of-run line through it (the review
  round) and now every MECH line during an unwind. Both now say so (one sentence each; a `//` banner, not a `///`
  comment — `check-fresh` exit 0, no `generate`). Changelog :42, roadmap :1295/:1552, guide 09 :176,
  `diagnostics-plan.md` :79, `docs/api/*` — true, untouched.
- `route_preview_render|legend` → `test/README.md` (the render header's line — true) and the master plan (an
  unrelated legend). `amber` in the other guides → the competition program's and Calibrate's screens, unrelated.
- **Not touched, deliberately:** the changelog (its RPV1 entry is general and stays true: "inside a step, during a
  wait, or after a step", "3 is the tool's own failure"); guides 14, 15 and the guide README (nothing they say moved).

### Mutations — the resumption (`scratch/rpv1-a/vfix/resume/mutate4.py`, log `vfix/resume/mutations4.log`)

`mutate4.py` imports `mutate3.py`'s machinery unchanged (backups under `vfix/resume/backups/`; exact-once
replacements, `--check` 9/9; `rp_test.o` removed and recompiled from the worktree each time and checked newer than
the mutation — all 8 code mutations: yes; `c++ -M` of the other two mini objects names no mutated file; every route
preview case under `ulimit -v 4194304; timeout 600`; restore in a `finally` and on SIGTERM/SIGHUP/SIGINT, timestamp
bumped; a docs mutation run on a binary rebuilt from the CLEAN tree first). Baseline first: the mini binary from
fresh objects, 40/40, 858 assertions.

- **A first run was VOIDED** (`mutations4.VOID-during-srcgate.log`): it was started while `src_build_gate.py
  self-test` ran, and that self-test plants into `src/build_selection.hpp`, `src/robots/*_table.hpp` and `src/team/`
  — all in the mini binary's include closure — and into `test/vendor/doctest.h`. A mini object compiled during a
  plant is not this tree's. Stopped with SIGTERM after 5 (its restore ran: the three files' checksums equal
  `vfix/resume/pre_mutation.md5`); none of its verdicts is counted. Re-run whole after the src gate finished.
- **The re-run (00:12–00:14), 9 mutations, 9 RED, each by a test's own CHECK** (none killed by a cap, none RE-CUT):
  VR1 (an arm or lift with no `kA` marked) RP44 :2075 `placeholderFor(r, "lift").empty()`; VR2 (a flywheel's τ never
  named) RP44 :2079 and :2085, the robot-x flywheel case :2109; VD1r (guide 16 says 0.15 s, on the edited paragraph)
  the B1 constants case :2190, alone (857/858); VF1 RP9b (e)/(f) :1541 "after step 2" for "inside step 2", :1550;
  VF2 RP9b (h) :1585 "inside step 1", :1591; VF3 RP9b (g) :1562 "inside step 2", :1567; VF6 RP38's new child :1758
  (exit 1, not 3); VM1 RP9b (g) :1562 and (h) :1585 ("after step 2" without "during a wait"); VM3 (0.05 → 0.15) the
  B1 constants case :2190 and :2194. After: the three files' checksums equal `pre_mutation.md5`; `git diff -- src/
  Makefile` empty.
- **The verifier's own probe, re-run on the final tree** (`verify/v2/probe/vprobe.cpp` copied unchanged to
  `vfix/resume/probe/`, built from fresh objects, outputs in `vfix/resume/probe/out/`): `layer "claw throw" 150` →
  "PROGRAM ERROR inside step 2", `#2 claw pivot 6 verdict=CANCELLED t0=1.24 t1=1.5 open=1 ... bg=0` (was "after step
  2", open=0); `imu "claw throw" 150` → "PROGRAM ERROR inside step 2", open=1 (was "during a wait, after step 2");
  `run "wait forever"` → "PROGRAM ERROR during a wait, after step 1" (unchanged — the branch worked; now pinned);
  `run "forever" --out out/notadir/forever.html` → **exit 3** (was 1), "route preview: could not write the page to
  out/notadir/forever.html", no page.

### Gates — the verification round's final tree (2026-11-14 00:14–00:20 EST; the src gate 2026-11-13 23:39–00:12)

- **src gate** (run first: its self-test re-dates `src/` files): `self-test` → OK (306 detector cases, each against a
  real build; 23:39:36 → 00:08:09, beside another worktree's src-gate run; my `timeout` sends SIGINT so a cut run
  still restores); `check` → **PASS, all 8 builds link** (00:12:40). `git diff -- src/ Makefile` and `git status
  --short --ignored -- src/ Makefile` EMPTY after each.
- **The suite, relinked from source:** both binaries removed, the briefing generated (unbuilt), configured, built
  (286 objects recompiled — the src gate's re-dated files and the mutation restores), the briefing generated again.
  `suite_run.py green` → **green — 3412 cases, 3780259 assertions**; `-tc="*route preview*"` → **40 passed, 858
  assertions**.
- `api_doc_tool.py` self-test OK; check-coverage 6062 entities across 206 headers, all documented; check-fresh exit 0;
  check-examples 762 quoted lines verbatim; check-removability clean. `briefing_status.py` self-test OK (38 cases),
  `check` current. `doc_staleness_audit.py` self-test OK (16 cases); run clean.
- **ARM header gate:** 249 headers (= `find`), CI's exact flags, `-Werror` → exit 0. **CI greps:** PROS-free outside
  `hal/pros/`; the core sim-free.
- **Compile refusals:** `ctest -R compile_refusal --no-tests=error` → 258/258 passed; 258 TUs.
- **The launcher:** `route_preview.py self-test` → ok, 35 checks, 0 failures. End to end (`vfix/resume/e2e/`):
  `"40in forward"` exit 0 (18,959 bytes; `1 | drive | (0.0, 0.0) 0.0° | (39.8, 0.0) 0.0° | 0.00 | 1.80 | SETTLED`),
  run twice → `cmp` identical (RP17 by hand); `"TEST 24in turn back" --place=-48,-48,90` exit 0 (31,382 bytes, stamped
  `v0.1.1-415-gd33d25d-dirty · autons file 2f45cb1053f1`); `"no such auton"` exit 2, both names listed; `--list` exit
  0; `--robot one` exit 2 with the no-build sentence. Every page: 0 "http", "IDEAL host test plant" once, the new
  legend once, the old one nowhere.
- **Scope:** `git diff d33d25d -- include/ src/ Makefile` = `include/shulib/hal/fake/fake_telemetry_sink.hpp` only
  (24 added comment/hook lines; no robot program includes it). **Nothing committed.**

### For the coordinator (the verification round adds)

- **The session that ran this round ended mid-gate at ~17:02** (the plant's mtime is 17:02:40; the cause is not on
  record — the machine was later rebooted, up 24 min at 23:53) and left the src gate's self-test 12 plant in
  `src/build_selection.hpp`. A gate killed by SIGTERM or SIGKILL skips its `with`-block restores (SIGINT does not:
  this round's `timeout -s INT`). Restored from git and logged above; worth a line in the crash-recovery checklist:
  **`git diff -- src/ Makefile` in every worktree after a session ends mid-gate**.
- **Two findings of my own** (V7, V8) beside the verifier's four, both wording, both fixed; V5 and V6 (the stopped
  run's) stand as logged.
- **ROB1-A has not landed** (`main` at `c778c11`): `--robot one` still refuses by name; the switch to robot one's
  list waits for it, as the brief says.
- The four findings' fixes, V5, V7 and V8 were reviewed by nobody but me; a fresh verifier is the workflow's next step.

## RPV1-A — the coordinator's merge verification *(2026-11-14)*

- **Squash-merged from `rpv1-a`** (transport `6d4143c`, base `d33d25d`) onto `49822e5` (ROB1-A and the homing fix
  landed since). Only the generated briefing conflicted (regenerated). The changelog entry moved from the top of
  `## API 2.7` to the top of `## API 2.8` (no API change, no bump). **Robot one's wording corrected at the merge:**
  ROB1-A landed the day before, so "until its robot file and its autons file land" was no longer true — guide 14,
  guide 16, the changelog entry and `test/route_preview_run.hpp`'s message and comments now say robot one's files
  are in the code but the preview is not switched to them yet (RP25's checked words, "robot one has no build yet",
  kept). The switch is a queued small run.
- **The stopped session's plant:** the fix2 resumption found `src/build_selection.hpp` still carrying the src gate
  self-test's case-12 plant (the laptop restarted mid-self-test, ~17:02 on 11-13) and restored it with `git checkout`;
  the coordinator's audit (`audit_wt.py`: 0 backed-up files, 0 suspicious) and `git diff -- Makefile src/` (empty)
  confirmed it before the transport commit.
- **Gates on the merged tree** (`.claude/coordinator/scratch/rpv1_a_merge_verify.sh`, log
  `.claude/coordinator/logs/rpv1a_merge_verify.out`): the suite green (**3,701 cases**); `src_build_gate.py` self-test
  OK and check **PASS — all 14 builds**; the ARM header gate exit 0; `ctest` **289 of 289**; the API reference fresh;
  the briefing current; the staleness audit clean; the CI and removability greps empty; no plant, no stray root file.
- **The coordinator's own mutations** (`muts_rpv1_a_main.json`, log `.claude/coordinator/logs/muts_rpv1_a_main.out`):
  **3 of 3 RED** — MP1 the pose recorded BEFORE each pace instead of after (8 cases), MP2 a left/right wall's
  penetration measured from the wrong face (9 cases), MP3 the field check's outline shrunk from 15 in to 12 in (3
  cases). The restored tree relinked and ran green.
- **RPV1-A LANDED.** Host test plant only; nothing has run on a robot. Next: the switch to robot one (its list on
  fakes by kind, its OWED numbers marked PLACEHOLDER); the in-page PREVIEW is RMP1-A2e/A2u's (WebAssembly).

## RPV1-S1 — switch the preview to robot one

**Executor:** `opus` (Claude Opus 5.5), a workflow subagent. **Worktree:** `.claude/worktrees/rpv1-s1`, branch
`rpv1-s1`, base `9befdb3` (RPV1-A on main). **Go:** the team lead's go for this run, relayed by the coordinator
2026-11-15. Scratch: `.claude/coordinator/scratch/rpv1-s1/` (never `/tmp`). Do not commit. Every binary runs as
`(ulimit -v 4194304; timeout 600 …)`. Host test plant only.

### Launch state (2026-11-15)

- `git log -1` = `9befdb3`; tree clean. ROB1-A is in the base (`src/robots/robot_one_facts.hpp`,
  `robot_one::kRobotOneMechanisms`, `src/team/robot_one_autons.cpp`); the preview still refuses `--robot one`
  (RP25, "robot one has no build yet").
- ROB1-C2 (robot one's real mechanisms and typed facts) builds beside this run: nothing here may assume today's list
  (the cascade only) or today's owed facts.
- **Baseline** (briefing regenerated with the suite unbuilt, a full build, the suite through `suite_run.py`):
  **green — 3701 cases, 3863418 assertions.**
- Read before writing: the brief's header, answers, R1–R13, D1–D13, §1.2, §2.2–2.4 (B1's rows), §3.2–3.3, §6, §7, §8;
  this log's RPV1-A record and the merge verification; `src/robots/robot_one_facts.hpp` (its `kRobotOneFacts` —
  "THE COUNT (… RPV1-B's hatch …)"), robot one's region of `src/robots/mechanism_table.hpp`,
  `src/team/robot_one_autons.cpp`. ROB1-C2's worktree was READ (never edited) to see what changes tonight: new
  mechanisms "right toggle", "left toggle", "claw", "flipper", "ramp"; facts typed and added, named e.g. "the right
  side toggle's port" — so a fact's name does NOT reliably name its mechanism (Δ1 below).

### Build log (2026-11-15)

1. `test/route_preview_run.hpp`: `kRobotOne` gets `table = &robot_one::kRobotOneMechanisms`, `registerAutons =
   &registerRobotOneAutons`, and two new `PreviewRobot` fields, `ledger` (a span of the ledger's own
   `FactSummary` list) and `ledgerFile`; `owedFacts(robot)` reads the ledger at run time and keeps every fact not
   typed, in the ledger's order; `askedModel` and `refusedModel` put that list on the Model. `noBuild` and its
   check are gone (Δ2). The banner's robot-one paragraph says what it does now, in three lines.
2. `test/route_preview_model.hpp`: `Model::owed`, `Model::ledgerFile` (kPreviewFootprintIn untouched — MIRROR1-A).
3. `test/route_preview_render.hpp`: `owedLine(m)`; the table prints it and one `  [PLACEHOLDER] <fact>` line per
   owed fact under the stamp; the page an amber dashed block (`class="owed"`) with one `<li>` per fact.
4. `test/CMakeLists.txt`: `src/team/robot_one_autons.cpp` joins `shulib_route_preview`.
5. `tools/route_preview.py`: the self-test's robot-one branch now REQUIRES robot one's file's hash (present, not
   robot two's); the docstring gains the `--robot one` line.
6. Tests (`test/route_preview_test.cpp`): RP25 rewritten as robot one's RUN test; RP25b, RP25c new; RP38 gains a
   robot with no autons file; the six fixture robots lose `.noBuild = ""`.
7. Probe: `shulib_route_preview "40in forward" --robot one --place=-48,-48,90` → exit 0, FINISHED, scoring ended
   T+1.80 s, the table lists all 17 owed facts (`scratch/rpv1-s1/one40.html`, 20,163 bytes). RP25b's cascade page
   (final form, mid-travel = 20 in): home 0.00–0.32 s SUCCEEDED, move 0.33–10.70 s SUCCEEDED (declared profile
   20 in at 2 in/s: 10.25 s + the 0.1 s window), both hatched by B1's HA-182 rule.
8. **ROB1-C2 probe** (`scratch/rpv1-s1/c2probe/`, `build.sh`): two scratch trees — `mine` (`git archive 9befdb3`
   include/src + this run's test/docs/tools) and `tree` (the same with ROB1-C2's uncommitted include/ and src/ files
   overlaid as of ~20:25: driver_inputs.hpp, driver_panel.hpp, drive_program.cpp, chassis_table.hpp,
   mechanism_table.hpp, robot_one_facts.hpp, robot_one_autons.cpp, robot_one_driver.hpp) — each built (test_main +
   route_preview_test + both autons files, the suite's flags) and run `-tc='route preview*'`: **42/42 green in
   both.** On ROB1-C2's tree the robot line names six mechanisms (cascade, right toggle, left toggle, claw, flipper,
   ramp), the list shows **12** owed (its typed facts dropped off by themselves), the cascade's mid-travel move took
   0.70 s (its typed blue 1:1). A hint only: ROB1-C2 was still in flight.

### Deltas from the brief and the task (each with its reason)

| # | Asked | Done | Why |
|---|---|---|---|
| Δ1 | "its PROVISIONAL list … the page and the table mark them" | the OWED facts are marked ROBOT-WIDE: a block at the top of the page and of the table, one line per fact; per-step hatching keeps B1's rules (the cascade, on the library's starting values, carries the HA-182 note on every step) | a fact's name does not name its mechanism reliably (ROB1-C2's "the right side toggle's port" vs the declaration "right toggle"), so a per-step attribution would be a guess that under-marks; the whole list is honest and needs no mapping |
| Δ2 | — | `PreviewRobot::noBuild` and its check removed; a robot with no register function is a precondition (runProgram: exit 3, its words) | no robot lacks a build now; dead text otherwise. RP38 proves the precondition (MS9) |
| Δ3 | "its PROVISIONAL list" | new fields `ledger` + `ledgerFile`; `provisional` (per mechanism, each with an HA row) stays robot two's | a ledger fact has no HA row and no mechanism; the ledger's own `kRobotOneFacts` ("THE COUNT … RPV1-B's hatch") is the source, read at run time, so a fact typed tonight drops off by itself |
| Δ4 | RP25 → a robot-one run test | RP25 (robot one's own file, every auton drawn, the marks), RP25b (the cascade through a test-local auton: the starter autons step no mechanism), RP25c (the rule on a hand-made ledger) | today all 17 facts are owed, so only a fixture ledger can show a typed fact dropping off; RP26 stays on robot x |
| Δ5 | "exit codes as B1" | RP25 requires each of the team's robot-one autons DRAWN (exit 0 or 1, page written, never 2 or 3); exit 0 is required of RP25b's cascade run; 2 for an unknown name, 0 for `--list` | the team's skills route lands in that file this week; failing the suite on a route's content is A1's suite check (D8), not this run's |
| Δ6 | — | the RPV1-A changelog entry's "switching the preview to it is queued" gains "— done 2026-11-15, the entry above" | history kept; a reader of the older entry is pointed at the newer one |
| Δ7 | — | guide 21 not touched | not stale (it never mentions the preview); the preview's home is chapter 16 |
| Δ8 | — | a page with no `--out` goes to `build/route_preview/<key>/<name>.html` for every robot but the first (`defaultPagePath`); robot two's path unchanged | FOUND running the launcher end to end: robot one's starter autons share robot two's names, so `"40in forward" --robot one` overwrote robot two's `build/route_preview/40in forward.html`. Guide 16, the launcher's docstring and `--out` help, the changelog say where robot one's pages go. RP25 checks both paths (end to end since the review round, F4); MS12 |
| Δ9 | (task) "a test that names a mechanism names the cascade only" | RP25b moves the cascade to MID-TRAVEL (`cascade.to((bottom + top) / 2)`), never a named level | ROB1-C2 (read, 2026-11-15 ~20:20) types the levels as NONE ("manual for now"): `cascade[facts::lv(1).name]` — this run's first form — would not compile after it lands. Mid-travel is inside the range whatever the ledger says |

### Tests added or changed (`test/route_preview_test.cpp`; suite 3701 → 3703: −1 old RP25, +3)

- **RP25** robot one runs its own autons on its own list — the registry is built from `registerRobotOneAutons`
  ITSELF (not through the preview); every auton in it run through `runPreview --robot one`: drawn, the summary
  carries ITS kind and schedule (robot two's same-named copies are MATCH 27/29, robot one's SKILLS 57/59), the robot
  line names robot one's list (iterated from `kRobotOneMechanisms`) on fakes by kind, and `checkMarks`: every ledger
  fact's `<li>` on the page and `[PLACEHOLDER]` line in the table present iff it is OWED, the count equal to the
  ledger's own `kRobotOneOwedCount` (a self-comparison that could not fail — replaced in the review round, F8), the
  header line on both; `--list` (0) names each; an unknown name → 2, no page,
  names `src/team/robot_one_autons.cpp`; `--robot three` → 2; the default page paths (robot one's under `one/`,
  robot two's unchanged — Δ8; the helper only — end to end since the review round, F4).
- **RP25b** robot one's cascade (the real declaration, found BY NAME in the list; a test-local auton naming only
  the cascade): homed and moved to MID-TRAVEL (Δ9), both SUCCEEDED, exit 0, the move within [declared trapezoid,
  +0.5 s] computed from the declaration's own `maxVelocity`/`maxAcceleration` and the ledger's two stops; a REFUSED robot-one auton (stop past 60 s)
  → exit 1 with the marks; the same auton on robot two's list stops "'cascade' is not on this robot".
- **RP25c** the list is the ledger's OWED facts in order (a 4-fact ledger, two typed → exactly the two owed); none
  owed → no block on either output; robot two has none (that check read a test fixture and could not fail —
  replaced in the review round by robot two's real page, F1).
- **RP38** (+3 checks) a robot with no register function → exit 3, "a robot with no autons file".

### Mutations (`scratch/rpv1-s1/mutate.py`, log `scratch/rpv1-s1/mutations.log`) — each on a FRESH binary (the old one removed, the suite TU's object and the binary checked newer than the mutant), the named cases run under `ulimit -v 4194304; timeout 600`, restored in a `finally` and on SIGTERM/SIGHUP/SIGINT, timestamp bumped

| # | Mutation | Result |
|---|---|---|
| MS1 | `owedFacts` lists every fact, typed too | RED — RP25c |
| MS2 | robot one with no ledger | RED — RP25, RP25b |
| MS3 | `refusedModel` without the owed list | RED — RP25b |
| MS4 | the table without the `[PLACEHOLDER]` lines | RED — RP25, RP25b, RP25c |
| MS5 | the page without the `<li>` facts | RED — RP25, RP25b, RP25c |
| MS6 | robot one's table = robot two's list | RED — RP25, RP25b |
| MS7 | robot one's register = robot two's `registerAutons` | RED — RP25 (its kind/schedule check, added before this run: robot two's autons have the same names, so a names-only check — RP25's first form — was REASONED to miss it; not run against that form) |
| MS8 | robot one on the rig's default fakes (`defaultRig = true`) | RED — RP25, RP25b |
| MS9 | the no-autons-file precondition removed | RED — RP38, CRASHED (SIGSEGV, exit −11): caught |
| MS10 | the owed count off by one | RED — RP25, RP25b, RP25c |
| MS11 | the launcher hashes robot two's file for `--robot one` | RED — the self-test's "--autons-hash is robot ONE's file's" |
| MS12 | every robot's default page on the first robot's path (Δ8) | RED — RP25. (First cut `std::string{}` for the subdirectory left `first` unused: -Werror, BUILD FAILED → re-cut as `first = !robots.empty()`) |

Three full runs (19:17, 20:16, and the FINAL 21:22 on the final tree — RP25b's mid-travel form, Δ8, the owed
line's final wording): **all 12 RED** with the same failing cases each time; after each run the mutated files'
SHA-256 equal the pre-mutation ones (`pre-mutation*.sha256`), the restored tree relinked (the binary removed
first) and ran green; `git diff -- Makefile src/` carries no '(self-test plant'; no untracked file under
src/ include/ test/.

### Prose review (rule 4) — greps run on the final text, hits read BEFORE this was written

- **§3.3's greps over every changed file's ADDED lines and the robot-one pages** (`one40.html`, RP25b's
  `one-cascade.html`, `one-refused.html`): `simulator` 0 (pages 0 each); `lower bound` 0; `exact|guarantee` one hit,
  a test comment ("Robot one exactly as the program knows it") — not about time; `internal/|chunks/|RESUMING|R[CPM][0-9]`
  in the public docs' added lines: 0 hits (corrected in the review round, F11: this line first said "one hit, the
  changelog's '(chunk RPV1, the switch to robot one)'" — that was a hit of a `chunk` grep, not of this pattern; the
  changelog's own convention, as the entries below say "chunk RPV1, part A", "chunk ROB1"), no test tag; no `R[CPM][0-9]` tag in an
  added header line.
- **"no build yet"** across docs/, README.md, test/, tools/ (outside docs/internal/): `docs/robot-facts-form.md:84`
  ("17 stops no build yet" — the facts form, about builds, not the preview: untouched); `docs/changelog.md` the RPV1-A
  entry's sentence — kept as history with "— done 2026-11-15, the entry above" (Δ6).
- **By file name** (`route_preview_run|render|model|test`, `shulib_route_preview`): test/README.md only — its section
  now says the target carries both robots' autons files and what `--robot one` lists. **`route_preview.py`**: guides
  14, 15, 16, the changelog, test/README — 14 and 16 updated; guide 15's entry is generic (no robot named), untouched.
- **By concept** (`robot_one_autons`, `robot_one_facts`, `--robot one`, `PLACEHOLDER`): guide 21 :16, :36, :235
  (robot one's files, the host suite on placeholder numbers — true, never mentions the preview: untouched, Δ7);
  `docs/robot-facts-form.md`, `docs/hardware-assumptions.md` (the facts and HA rows — nothing about the preview);
  guide 16 :365, :372, :410-412, :457-460 and the changelog — this run's own lines, re-read.
- **Wording corrected after the mutation run:** the owed line said "the preview runs on their placeholders" — not
  true of a fact the preview never reads (the camera's port and mount); now "wherever the preview needs one, it uses
  the placeholder until it is typed", and guide 16 and the changelog say the same.

### Gates — the final tree (2026-11-15 21:2x EST)

- **The suite** (`briefing_status.py generate` with the binary removed, a full build, then the suite):
  `suite run: green — 3703 cases, 3863609 assertions` (baseline 3701: −1 old RP25, +3).
- **The src gate:** `check` → `src build gate: PASS (all 14 builds …)`, exit 0 (20:11–20:26; `src/` identical to
  the base). `self-test`: the FIRST run hit this run's own 50-min bound (`timeout -s INT 3000`; four gates were
  running on the laptop at once, load ~10) at its last stage — SIGINT, its `with`-block restores ran: `git diff --
  src/ Makefile` empty, no plant, no untracked file; RE-RUN with a 7000 s bound → `src build gate self-test: OK (377
  detector cases, each against a real build)`, exit 0 (~20:27–21:18). Plant checks after each: 0 '(self-test plant',
  no untracked file under src/ include/ test/.
- **The ARM header gate:** every include/shulib header (255) in one TU with the robot's flags → CLEAN.
  `git diff 9befdb3 -- include/ src/ Makefile` → EMPTY (this run touches neither).
- **The CI greps:** library PROS-free; core sim-free.
- **api_doc_tool:** self-test OK; check-coverage "6386 public entities across 211 headers, all documented";
  check-fresh exit 0; check-examples "860 quoted lines, all verbatim (18 source files)"; check-removability "no public
  doc references docs/internal/". No `///` in include/ changed: docs/api not regenerated (check-fresh agrees).
- **briefing_status.py** self-test OK (38 cases), check "generated block current"; **doc_staleness_audit.py**
  self-test OK, the audit "clean (34 live docs …)".
- **The launcher, end to end** (fresh `build/route_preview/`): `"40in forward"` 0 → `build/route_preview/40in
  forward.html`; `"TEST 24in turn back"` 0; `"40in forward" --robot one --place=-48,-48,90` 0 →
  `build/route_preview/one/40in forward.html` (20,219 bytes); `"TEST 24in turn back" --robot one` 0 →
  `one/TEST 24in turn back.html` (33,004 bytes); `"no such auton"` 2 (robot two's names); `"no such auton" --robot one`
  2 (names `src/team/robot_one_autons.cpp`); `--list --robot one` 0. Every page: 0 "http", 0 "simulator", "IDEAL
  host test plant" once. `route_preview.py self-test` ok (0 failures).
- **Reviewers:** this executor has no agent tool — the three reviewers, the fix round and the coordinator's checks
  are the workflow's next steps; nothing here was reviewed by anyone but me. **Nothing committed.**

### For the coordinator

- **Files another in-flight run also edits:** `docs/changelog.md` (the new entry is at the top of `## API 2.8`;
  MIRROR1-A opens `## API 2.9` — move this entry to the top of the newest section at the merge, still "No API
  change"); `docs/guide/14-what-it-cannot-do-yet.md` and `16-…` (MIRROR1-A's staged hunks are at guide 16 :128 and
  in guide 14 elsewhere; this run's are guide 14 :71-72 and guide 16 :365-380, :410-412, :457-460);
  `test/README.md` (:161-164; MIRROR1-A adds 20 lines elsewhere); `test/CMakeLists.txt` (:329-330 only).
  `test/route_preview_model.hpp`: two Model fields added; `kPreviewFootprintIn` untouched (MIRROR1-A reads it).
  No `src/` file and no ROB1-C2 file touched.
- **ROB1-C2:** its in-flight src (read at ~20:25) passes all 42 preview cases (Build log item 8). Its levels are typed
  NONE, which is why RP25b moves the cascade to mid-travel (Δ9). After it lands, robot one's page lists 12 owed
  facts, not 17 — by itself.
- **`src/robots/robot_one_facts.hpp`'s count banner** ("THE COUNT (the Drive boot line, RPV1-B's hatch, the tests)")
  names the use this run makes of `kRobotOneFacts`; left as it is (a src file, ROB1-C2's).
- **Robot one's pages now go to `build/route_preview/one/`** (Δ8) — a behaviour the brief did not ask for, found end
  to end; your call if you would rather a filename suffix.
- **Status pointers** (roadmap "You are here", build-order `Next:`, the brief's B1 line "it switches to robot one the
  day G1 lands") are not edited here; this log's between-parts marker is kept.

## RPV1-S1 review findings and their disposition

**Who:** three reviewers (correctness, tests, docs; their probes under `scratch/rpv1-s1/review-*`), 17 findings,
deduplicated to 14 below; the fix round by the builder (`opus`), 2026-11-15 ~21:45–23:00. Each finding was
re-checked before it was fixed. No test case added or removed (3703); nothing committed.

| # | Finding (lens, severity) | Disposition |
|---|---|---|
| F1 | No test looks at robot two's REAL page for the owed block: RM1/SB (robot two given robot one's ledger) and RM2 (the table's guard removed) survived; RP25c's last check read a test fixture, so it could not fail (correctness major, tests major — the same gap) | **FIXED.** RP25 now runs robot two's own "40in forward" through `rp::kRobots` (in the default-path block, F4): `owedFacts(rp::kRobotTwo)` empty, no `PLACEHOLDER numbers:` in the table, no `class="owed"` on the page, no empty line after the stamp. RP25c's fixture check removed. RM1/SB, RM2 and RM2b (the page's guard removed) **RED** |
| F2 | Robot two's page gained an unused `.owed{…}` CSS rule, so it was not byte-identical (correctness minor) | **FIXED.** The block's style is inline on the block; the shared style sheet is the base's again. Robot two's "40in forward" and "TEST 24in turn back", each with and without `--place` (stamp and hash fixed), against the base program (`review-correctness/base`): **all four pages and all four tables byte-identical** (the table's `--out` path normalised) — `scratch/rpv1-s1/fix-cmp2/` |
| F3 | The changelog entry sits at the top of `## API 2.8`; main now opens `## API 2.9`, and the RPV1-A entry's "the entry above" would then point at the wrong entry (correctness minor, docs minor) | **The move is the coordinator's, at the merge** (this worktree's base has no 2.9 section): the entry goes to the top of `## API 2.9`, still "No API change". **The pointer is fixed here:** it names the entry by its title ("done 2026-11-15: \"the route preview previews robot one\""), so it stays true wherever the entry sits |
| F4 | Δ8's default page path was never run end to end: SA (runPreview back to the old path) survived; RP25 only called the helper (tests major) | **FIXED.** RP25 runs robot two's and robot one's "40in forward" with NO `--out`, with `build/route_preview_test/default-paths/` as the working directory (a guard puts the old one back, also when a REQUIRE throws): robot two's page is at `build/route_preview/40in forward.html` and says ROBOT TWO, robot one's at `build/route_preview/one/40in forward.html` and says ROBOT ONE. The two direct calls of the helper are gone (the run covers both). SA **RED**; MS12 still RED |
| F5 | No ledger with exactly ONE owed fact: SC (both guards `size() > 1`) survived (tests minor) | **FIXED.** RP25c adds `{typed, one owed}`: "PLACEHOLDER numbers: 1 of this robot", its `[PLACEHOLDER]` line, its `<li>`. SC **RED** |
| F6 | "In its order" was checked on `owedFacts()` only: SD (both renderers reversed) survived (tests minor) | **FIXED.** RP25c checks the order in the table and on the page. SD **RED** |
| F7 | A hand-copied ledger survived (SE), against "never copied" (tests minor) | **FIXED.** RP25 checks that robot one's ledger IS the ledger: `kRobotOne.ledger.data() == std::data(kRobotOneFacts)`, and the same size. SE (today's 17 facts copied by hand) **RED** |
| F8 | checkMarks' count compared the ledger's count with itself (tests minor) | **FIXED.** checkMarks counts what the program WROTE: the `[PLACEHOLDER]` fact lines in the table and the `<li>`s inside the owed block, each equal to the ledger's owed count; with nothing owed, no block. SF (the table lists a fact twice) and SG (the page does) **RED**; MS10 still RED (the header line) |
| F9 | A fact typed "chosen, not measured" leaves the list and gets no per-step mark: after ROB1-C2, robot one's claw, front-toggle and ramp steps draw on an unmeasured 0.5 s, unmarked, while robot two's claw says HA-185 (docs major) | **CONFIRMED — the root fix is not in this run's files.** Re-checked on ROB1-C2's live tree (read only, ~22:00): the claw's, front toggle's and ramp's stroke times are `typed(0.4_s, 0.5_s, …, "chosen, not measured -- … (robot two's claw's provisional time, HA-185)")`, and the cascade's stall signature and cap, the side toggles' cap and the homing stall are typed the same way. `Fact` carries its `source`, but `FactSummary` — the only list, `kRobotOneFacts` — keeps name, stage and typed, so the preview cannot tell a chosen value from a measured one without a hand-made list (forbidden). The fix needs `src/robots/robot_one_facts.hpp`, ROB1-C2's file, in flight: `FactSummary` gains the source (or a `measured` flag) in `summary()`; then the preview lists OWED and CHOSEN facts, each labelled, and RP25c holds a chosen fact ON the list. The per-step mark is the brief's R11 path: robot one's declarations mark a chosen number `// PROVISIONAL (HA-…)`, and a robot-one `provisional` table is held to those comments as RP33 holds robot two's — it names mechanisms that exist only after ROB1-C2. **Done here:** guide 16 (:414-415), guide 14 (:72-73) and the changelog say plainly that a value typed "chosen, not measured" counts as typed and the page does not mark it. **For the coordinator:** the follow-up below |
| F10 | §1.2's per-step HA-182 mark on robot one's cascade was not tested: RD1 (no per-step mark on a robot with a ledger) survived (docs minor) | **FIXED.** RP25b computes the rule from the cascade's OWN declaration (`positionStartingValues`: any of kP, kV, maxVelocity, tolerance still the library's) — not a literal, so a tuning ROB1-C2 or TUNE1 types moves the expectation with it — and checks every cascade step's note for HA-182 and the table's `SUCCEEDED [PLACEHOLDER]` count. RD1 **RED** |
| F11 | The prose review as logged was incomplete: no `robot two` / stand-in grep; a logged hit could not come from the pattern named; robot two's page sizes missing (docs minor) | **FIXED.** The greps re-run on the final text (below); the wrong line in the S1 record is corrected in place, marked "(corrected in the review round, F11)"; the S1 record's other claims this review disproved are marked in place too (F1, F4, F8) |
| F12 | The new comments broke the team's one-line rule (docs minor) | **FIXED.** The S1 test cases have no comment block (their titles say it); each helper's comment one line; `defaultPagePath`'s one line; the banner's robot-one part one line |
| F13 | RPV1-A's "stall homing not exercised" note not closed; RP25b only homed from the bottom stop (docs minor) | **FIXED, beyond logging.** RP25b's cascade auton homes AGAIN from mid-travel: SUCCEEDED, 10.71–23.94 s on today's placeholders (1.04–1.83 s on ROB1-C2's live tree; the 0.32 s home at the stop is 0.30 s of stall signature plus a tick), and the test holds the second home longer than the first — it travelled down before it stalled. RH1 (fakes by kind with no bottom stop: the push never stalls) **RED**. Not isolated by a mutation: the "longer than the first" check alone (a plant that stalls in mid-air needs a change to the library's sim plant). Still unverified: anything about a REAL cascade's stall currents (host test plant only) |
| F14 | The between-parts marker still said "the switch to robot one (queued)" (correctness minor, docs minor) | **FIXED.** Its reason now reads "RPV1-A (A0+B1) landed 2026-11-14 and RPV1-S1 (the switch to robot one) 2026-11-15; A1/A2 and B2 follow …"; `release.py`'s `between_parts_reason` reads it back whole. The coordinator confirms the date at the merge |

### Mutations — the review round (`scratch/rpv1-s1/mutate_fix.py`, log `mutations-fix.log`, 22:01–22:04)

Every mutation on the FINAL tree: all its anchors matched before anything was written; the binary removed; rebuilt;
EVERY suite object whose depfile names a mutated file (one: `route_preview_test.cpp.o`) checked newer than the
mutation, and the binary; the named cases under `ulimit -v 4194304; timeout 600`; restored in a `finally` and on
SIGTERM/SIGHUP/SIGINT, timestamps bumped. The reviewers' survivors and this run's MS1–MS12 together — **23 of 23
RED**, none survived, none failed to build:

| # | Mutation | Result |
|---|---|---|
| RM1/SB | robot two carries robot one's ledger | RED — RP25 |
| RM2 | the table's guard removed | RED — RP25 |
| RM2b | the page's guard removed | RED — RP25, RP25c |
| SA | the default page path unwired in runPreview | RED — RP25 |
| SC | one owed fact shows no block (both guards) | RED — RP25c |
| SD | both renderers list the facts in reverse | RED — RP25c |
| SE | a hand-copied ledger (today's 17 facts) | RED — RP25 |
| SF | the table lists the first owed fact twice | RED — RP25, RP25b |
| SG | the page lists the first owed fact twice | RED — RP25, RP25b |
| RD1 | no per-step HA-182 mark on a robot with a ledger | RED — RP25b |
| RH1 | no bottom stop on fakes by kind: a stall home never stalls | RED — RP25b |
| MS1 | every fact listed, typed ones too | RED — RP25c |
| MS2 | robot one with no ledger | RED — RP25, RP25b |
| MS3 | a refused page without the owed list | RED — RP25b |
| MS4 | the table without the owed facts | RED — RP25, RP25b, RP25c |
| MS5 | the page without the owed facts | RED — RP25, RP25b, RP25c |
| MS6 | robot one previewed on robot two's list | RED — RP25, RP25b |
| MS7 | robot one previewed with robot two's autons file | RED — RP25 |
| MS8 | robot one on the rig's default (robot two's) fakes | RED — RP25, RP25b |
| MS9 | no check for a robot with no autons file | RED, CRASHED (SIGSEGV, exit −11) — RP38 |
| MS10 | the owed count off by one | RED — RP25, RP25b, RP25c |
| MS11 | the launcher hashes robot two's file for `--robot one` | RED — the launcher's self-test |
| MS12 | robot one's default page on robot two's path | RED — RP25 |

After the run: the five mutated files' SHA-256 equal the pre-run ones (`pre-mutation-fix.sha256`); the binary
removed and the tree rebuilt and run green (Gates, below); `git diff -- Makefile src/` carries no '(self-test plant';
no untracked file under src/ include/ test/.

### Prose review — the review round (greps run on the final text, hits read before this was written)

- **§3.3's greps over the ADDED lines** of every changed public doc, test and tool file (333 lines,
  `scratch/rpv1-s1/fix-added.txt`) and the four launcher pages: `simulator` 0 (pages 0 each); `lower bound` 0;
  `exact|guarantee` 2, both test comments ("exactly as the program knows it", "exactly the ledger's OWED facts") — not
  about time; `internal/|chunks/|RESUMING` 0; `R[CPM][0-9]` 0 in the public docs and 0 in added header lines — only
  the test file's case names (RP3, RP10, RP25, RP25b, RP25c, RP26, RP35); `chunk` 1, the changelog's "(chunk RPV1,
  the switch to robot one)", the changelog's own convention.
- **`robot two|robot one|stand-in|drive`** over the public docs' added lines: each sentence that says where robot one
  runs says "the same stand-in drive" (guide 16 :460-461, the changelog :24-26); guide 14's robot-one sentence sits
  inside the entry that opens "on a stand-in drive"; no claim about robot two's drive was added.
- **`chosen, not measured`** across the public docs: guide 16 :414 and guide 14 :73 (this round's); guide 21 :82,
  :110, :136 and `docs/robot-facts-form.md` :43, :83 — the ledger's own rule ("a number chosen at the bench … is
  typed with the source `chosen, not measured`"), consistent with the new sentences; none says the preview marks
  such a value — untouched.
- **This round's names:** `defaultPagePath`, `class="owed"`, `.owed{` in public docs and tools: 0.
  `route_preview/one` / `under one/`: guide 16 :380, the changelog :29, the launcher :13 and :82 — true.
  "the entry above" in the changelog: only :2998, an unrelated 2026-09 entry.
- **Page sizes** (the launcher, a fresh `build/route_preview/`, stamped with the commit and the autons hash): robot
  two's "40in forward" **18,959 bytes**, "TEST 24in turn back" **31,563**; robot one's 20,221 and 33,006. Robot two's
  are 62 bytes under the reviewer's 19,021 / 31,625: the `.owed` rule (F2). Every page: 0 "http", 0 "simulator",
  "IDEAL host test plant" once; the owed block once on robot one's, never on robot two's.

### Gates — the review round's final tree (2026-11-15 ~22:05–23:00 EST)

- **The suite** (`briefing_status.py generate` with the binary removed, a configure, a full build — the doc gates
  ran — then `suite_run.py green`): **green — 3703 cases, 3863638 assertions** (S1's build: 3703 / 3863609). The
  briefing regenerated after it: its Suite line reads 3,703 / 3,863,638. Route preview cases: 42/42.
- **ROB1-C2's live src** (read only, copied ~22:05 into `scratch/rpv1-s1/c2probe/tree2`: the base's include/ src/
  test/, this run's test files and guides, ROB1-C2's uncommitted include/ and src/; `c2probe/build.sh`): **42/42
  green.** The robot line names six mechanisms (cascade, right toggle, left toggle, claw, flipper, ramp); **12** owed
  facts; the cascade's three steps all hatched HA-182 (its tuning still the library's), the re-home 0.79 s against
  0.32 s at the stop. (The first run failed one case only because the probe tree had no `docs/guide/16-…` for a
  source pin; with the guides copied, green.)
- **Robot two unchanged:** four pages and four tables byte-identical to the base program's (F2).
- **The ARM header gate:** 255 headers in one TU, CI's flags, `-Werror` → CLEAN. **The CI greps:** PROS-free,
  sim-free. `git diff 9befdb3 -- include/ src/ Makefile` → EMPTY.
- **api_doc_tool:** self-test OK; check-coverage "6386 public entities across 211 headers, all documented";
  check-fresh exit 0; check-examples "860 quoted lines, all verbatim (18 source files)"; check-removability "no
  public doc references docs/internal/". No `///` comment in include/ changed: docs/api not regenerated.
- **briefing_status.py** self-test OK (38 cases), check "generated block current"; **doc_staleness_audit.py**
  self-test OK, the audit "clean (34 live docs …)".
- **The launcher end to end:** `"40in forward"` 0, `"TEST 24in turn back"` 0 (robot two's pages where they were);
  `"40in forward" --robot one --place=-48,-48,90` 0 and `"TEST 24in turn back" --robot one` 0 (under `one/`);
  `"no such auton"` 2 (names `src/team/autons.cpp`), with `--robot one` 2 (names `src/team/robot_one_autons.cpp`);
  `--list --robot one` 0; `route_preview.py self-test` ok (0 failures).
- **The src gate:** `check` → `src build gate: PASS (all 14 builds …)`, exit 0 (22:09–22:22); `self-test` →
  `src build gate self-test: OK (377 detector cases, each against a real build)`, exit 0 (22:22–23:08). After both:
  0 '(self-test plant' in `git diff -- Makefile src/`, no untracked file under src/ include/ test/, `git diff 9befdb3
  -- include/ src/ Makefile` empty.
- **Nothing committed.**

### For the coordinator (adds to the list above)

- **F9's follow-up** (a proposal, not built — it needs ROB1-C2's file): after ROB1-C2 lands, (1) `FactSummary`
  gains the fact's `source` (or `bool measured`) in `summary()`; (2) the preview's list becomes the facts nobody
  measured — OWED and "chosen, not measured", each labelled — with RP25c holding a chosen fact on it; (3) robot one's
  chosen stroke times get `// PROVISIONAL (HA-…)` on their declarations and a robot-one `provisional` table held to
  them as RP33 holds robot two's, so a robot-one claw step says what robot two's says. Until then the docs say the
  gap plainly (guide 16, guide 14, the changelog).
- **The changelog** (F3): move the entry to the top of `## API 2.9` at the merge; its back-pointer already names it.
- **ROB1-C2 also edits** `docs/changelog.md` and `docs/guide/21-…` (its `git status`, read ~22:00): the changelog
  will need a hand merge.
- **The between-parts marker** (F14): its reason now names S1 with today's date — confirm or correct it at the merge.

## RPV1-S1 — the coordinator's merge verification *(2026-11-16)*

- **Squash-merged from `rpv1-s1`** (transport `af4fb8a`, base `9befdb3`) onto `3c29b73` (MIRROR1-A, the Tune label fix and
  ROB1-C2 landed since) — in the coordinator's INTEGRATE worktree, because the team builds and uploads from the main tree
  now. Only the generated briefing conflicted (regenerated); the changelog entry moved to the top of `## API 2.9`.
- **The src gate's self-test, cases 53 and 55 — stale fixtures, fixed in this merge (`tools/src_build_gate.py`):** both
  assumed robot one's Bench Tests still REFUSES on owed Stage A facts; since 2026-11-16 (ROB1-C2 + the right toggle's port)
  Bench Tests and Calibrate LINK. Case 53 now drops `needs()` on Drive and expects it to refuse the RECORDED camera facts
  too; case 55 builds `ROBOT=one PROGRAM=drive` for its refusal half. Verified by hand first (worktree post-3c29b73): a
  linked Calibrate's package is REMOVED by a refused Drive build. Follow-up: both cases should plant their own owed fact,
  so they hold once every fact is typed.
- **Gates** (`scratch/integrate_verify.sh rpv1s1`, then the self-test re-run): suite green (**3,764 cases**); src gate
  self-test **OK (377 cases)** and check **PASS — all 14 builds**; ARM header gate exit 0; ctest compile refusals green;
  doc gates and briefing current; no plant, no untracked file.
- **The coordinator's mutations** (`muts_rpv1_s1_main.json`): **3 of 3 RED** — MS-C1 every fact listed as PLACEHOLDER,
  typed ones too (3 cases); MS-C2 robot one previewed on robot two's list (2); MS-C3 robot one's previews run robot two's
  autons (1). Restored tree green.
- **RPV1-S1 LANDED:** `python3 tools/route_preview.py "<auton>" --robot one` previews robot one's own autons on its own
  mechanisms, every still-owed fact marked PLACEHOLDER from the ledger itself. Host test plant only.
