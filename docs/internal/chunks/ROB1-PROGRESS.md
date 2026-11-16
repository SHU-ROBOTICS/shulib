# ROB1 — robot one (and its twin) in the code — live progress log

<!-- release: between parts — ROB1-A and ROB1-C2 landed (robot one's mechanisms, 2026-11-16); Stage B from station 13, ROB1-BT (robot two as the twin) and ROB1-C follow (declared by the coordinator 2026-11-16) -->

> The brief: [`ROB1-robot-one-and-twin.md`](ROB1-robot-one-and-twin.md) (it calls the runs G1-A / G1-B / G1-T / G1-C /
> G1-F; filed as `ROB1-*` because "G1" is a finished chunk — `G1-PROGRESS.md` is NOT this log). Appended in real time.
> Host- and build-gated only: **robot one does not exist in our records; nothing has run on it.** Every placeholder
> number is OWED until the builder's facts are typed (G1-F).

## ROB1-A

- **Executor:** `opus` (a workflow subagent), worktree `.claude/worktrees/rob1-a`, branch `rob1-a`, base `b71f838`
  (main `49d1a22` — MECH-S-A, L6-A2, IMU1, MECH core — plus CAL1-B's squash, staged and uncommitted on main). Not
  committed (the coordinator commits). Never `pros`, never `/dev/ttyACM*`.
- **Scope:** G1-A ONLY (§1.1), D1–D13 as recommended, the re-check's R1–R11 over the body. Not G1-B, G1-C, G1-T, G1-F.
- **The brief's line numbers** were opened at `16c01e0`; this run's base is `b71f838` (CAL1-B's squash on top of
  MECH-S-A). Every delta from the brief against THIS base is logged below under "Deltas".

### Log
- **Read first:** RESUMING.md's standing rules; the brief's header, answers block, D1–D13, §0.2–0.3, §1.1, §2.0–2.3,
  §3, §4, §5, §6, §7, §8, §9 and the Re-check R1–R11 in full; the probes P2/P3/P3b/P3c/P4/P5 in
  `.claude/coordinator/scratch/brief-G1/` (read, never edited).
- **Code, first pass (in order):** `src/build_selection.hpp` (`RobotOne`, five branches, the guards widened with robot
  two's words kept, `robotBeacon()`'s case); `Makefile` (`ROBOT=one`, the four PROGRAM guards accept `one`, THE
  VARIANT STAMP before `-include ./common.mk` — P5's lines, with an explanatory banner); NEW
  `src/robots/robot_one_facts.hpp` (the ledger: 17 facts — 10 Stage A, 5 Stage B, 2 Recorded); `chassis_table.hpp`
  (`kRobotOneChassis`, every field UNSET, below robot two's/`kNoChassis`; the case; the restatement);
  `mechanism_table.hpp` (robot one's region and its two fact checks ALL BELOW robot two's region — the ledger's
  `#include` too, after closing and re-opening the namespace; the case; the drive check; the restatement); NEW
  `src/team/robot_one_driver.hpp`, `robot_one_autons.{hpp,cpp}`, `src/common/this_robot.hpp`,
  `src/common/robot_words.hpp`; the four programs/commons switched to `ThisRobotDriver` /
  `registerThisRobotsAutons()` / per-robot literals; `driver_panel.hpp`, `boot_source.hpp` take a last defaulted
  `Words`; `test/CMakeLists.txt` compiles `robot_one_autons.cpp`.
- **First host build:** green compile; the suite RED on exactly the §0.3 pins (11 cases: `registerAutons(` ×3 in
  `auton_tester_source_test`, ×2 in `competition_source_test`, `RobotTwoDriver` ×6, the competition body's claw
  sentence) — each re-pinned with the same force (count 1 of the new name AND count 0 of the old and of
  `registerRobotOneAutons(`; the sequence and position checks on the new name; `"MECH ABSENT"` added to Drive's
  shared-words list beside `"CLAW ABSENT"`).
- **First ARM probe (a scratch copy under `.claude/coordinator/scratch/rob1-a/typedcopy`, every `= owed(` typed as the
  gate's copy will):** all five robot-one builds LINK; one `-Wformat-truncation` (robot one's dropped-autons line, 10
  bytes longer path) — fixed by shortening robot one's literal ("NOT on the menu"); robot two's literal untouched.
  On the real tree `make ROBOT=one` refuses with exactly the 10 Stage A lines; `PROGRAM=drive` with the 15 A+B lines.
- **Tests written (host):** `test/robot_one_facts_test.cpp` (T2, T3), `robot_one_tables_test.cpp` (T1, D5, T8, T9,
  [R5]), `robot_one_driver_test.cpp` (T11 + the MECH row), `robot_one_autons_test.cpp` (T12), `robot_one_flipper_forms_test.cpp`
  (T13), `this_robot_test.cpp` (T1's driver/autons halves, R7's words, T7); robot-one cases appended to
  `boot_source_test.cpp` (T10); `build_selection_test`, `chassis_table_test`, `mechanism_table_test` gain RobotOne.
  Compile refusals s74–s82 (typed without a source; the gate-copy source; typed = placeholder, scalar and struct;
  a misquoted check; an undeclared non-drive motor; a flipper kind without its arm; the cascade past its top; the
  pneumatic form without its move time).
- **Found while testing (each fixed at the cause):** (1) the brief's P3 homing push -3 V can never reach the 1.0 A
  placeholder signature on the host test plant (2.5 A at 12 V stalled) — the placeholder is -6 V (MECH-S-A's CA9
  uses -6 V for the same reason); (2) robot one's fact checks re-cut as consteval refusals BY NAME
  (`refused::a_motor_the_devices_screen_lists_is_not_a_drive_motor_and_is_not_declared_in_robot_ones_list`,
  `refused::the_flippers_typed_kind_does_not_match_robot_ones_list`) so a compile-refusal case exercises THE
  table file's function, and a source pin proves it is applied to the real ledger; (3) the region banner quoted
  the marker comments literally, so a marker search found the banner — re-worded.
- **Suite:** `suite_run.py green` — 3173 cases, 3033008 assertions; the nine new compile refusals pass.
- **[Not logged before the stop, found in the tree at resumption]** the gate's ROB1-A work (`tools/src_build_gate.py`:
  the five rows in every table; step 0's team-selection rule and the ledger-source rule; step 8's robot-separation
  half; THE FACTS PROOF — `facts_verdict` for the real-tree rows and `facts_copy_proof` in `build/src-gate-facts/`;
  self-test cases 42–47; cases 16 and 21's plant targets re-pointed at robot two's lines because robot one's branches
  read the same program defines); a `check` at ~04:16 PASSED (12 builds: 7 link, robot one's 5 refuse exactly their
  owed sets — 10 for `one`/`one-calibrate`, 15 for the other three — and all 5 link in the typed copy; 7 min 04 s); a
  first `self-test` died at case 16's old plant target (fixed in the tree since); the docs (guide 21, the facts form,
  guides 07/16/README, changelog, HA register, roadmap, REMAINING-WORK, worksheet, mkdocs nav) drafted; T6's fourteen
  packages built (`scratch/rob1-a/t6/`), NOT yet compared; the mutation drivers written (`scratch/rob1-a/mut/`), NOT run.
- **RESUMED 2026-11-11 07:44 EST** (`date`) after the 5-hour usage stop at ~04:30. Checked first: `git diff -- Makefile
  src/` carries no `(self-test plant` or other plant marker; no mutation backup outside `scratch/rob1-a/mut/`; the T6
  `change/` tree's `src/`, `include/`, `Makefile`, `common.mk` are identical to the worktree's (`diff -rq`). Fresh host
  build from source (`rm -f build/test/shulib_tests`; the briefing regenerated before and after): `suite_run.py
  green` — 3173 cases, 3033008 assertions.
- **T6 — robot two's seven packages, base `b71f838` vs the change (07:50–07:58).** Built by
  `scratch/rob1-a/t6/run_t6.sh` (both trees `git archive`-style copies with `GIT_CEILING_DIRECTORIES` so no build
  hash enters either; every `make` capped). **Delta from the brief's method:** the two trees sit at `t6/base/` and
  `t6/change/`, not one path in turn — harmless here because no package carries a source path (`strings -a` set
  differs only in the timestamps), said so it is not hidden. **FIRST COMPARE — A FINDING:** robot two's Drive and
  Competition machine code CHANGED: the first cut made `clawRowText`, `controllerWord` and `controllerScreenLines`
  templates with a defaulted `Words` (the brief's §1.1 "last, defaulted parameter"), and GCC then INLINED
  `clawRowText` (287 insns) and `controllerScreenLines` into `runDrive` and `DriverPanel::view` where the base calls
  them (per-function disassembly modulo addresses: Drive `runDrive` 1118 → 1444 insns, two functions gone;
  Competition `view` 270 → 539); `.rodata` sizes differed (Drive 68479 → 68430; Competition 80356 → 80320; xdrive
  38638 → 38622, a 16-byte `"ABSENT"` initializer). **Fixed at the cause (a delta from §1.1's mechanism, not its
  intent):** robot two's three functions are back to the base text, byte for byte; robot one's words are an
  OVERLOAD with an explicit `Words` argument that wraps them (`clawRowText(…, Words)` replaces the first four letters;
  `controllerWord(f, Words)` swaps `CLAW ABSENT` for the robot's word; `controllerScreenLines(f, Words)` re-writes
  line 2); each program picks on `ThisRobotWords::value.claw` — `?:` in Drive and Competition's LCD, `if constexpr`
  in Competition's `view` (the `?:` on a class prvalue cost 3 instructions there). `decideBootSourceFrom`'s defaulted
  `Words` (boot_source.hpp) is KEPT: its packages' code did not change. Source pins re-pinned with the same force
  (`drive_program_source_test`: the two LCD/row sequences, `count("controllerScreenLines(") == 2` AND
  `count("controllerScreenLines(f, ThisRobotWords{})") == 1`; `competition_source_test`: three sequences).
  **FINAL COMPARE (all seven rebuilt from the fixed tree):** `.bin` sizes identical for all seven; per-function
  disassembly modulo addresses: 0 differing functions in all seven (`tools/t6_funcs.py`); `strings -a` sets differ
  ONLY in `__DATE__ __TIME__` / `BUILD hh:mm:ss` strings; `.data` masked compare 0 bytes in all seven; `.rodata`
  masked compare (P6's `t6_rodata.py`) 107 / 725 / 729 / 765 / 646 / 761 / 761 bytes (bench / xdrive / tank / drive /
  calibrate / autontest / competition) — every run inside `typeinfo for …`, `vtable for …`, `typeinfo name for …`,
  `_PROS_COMPILE_TIMESTAMP*` or a timestamp string (`tools/t6_where.py`): the typeinfo/vtable OBJECTS are REORDERED
  (four TUs that include `robot_words.hpp`/`this_robot.hpp` — boot_source, drive, competition, auton tester — emit
  their COMDAT sections in a different order: `readelf -S` of their `.o`; the linker keeps the first copy). To prove
  a REORDER and not a change, a stronger compare (`scratch/rob1-a/tools/t6_symbolic.py`): every aligned word pointing
  into the image replaced by the C string or the symbol it points at, every named object compared by name, the
  anonymous bytes as a multiset of NUL-terminated pieces — result for all seven: **the only differences are
  `_PROS_COMPILE_TIMESTAMP`, `_PROS_COMPILE_TIMESTAMP_INT`, `kBuildTimeText` and the timestamp strings**. Report:
  `scratch/rob1-a/t6/t6_report.txt`. Robot one's literals reach no robot-two package (the strings sets).
- **Prose review (2026-11-11 ~08:00), the brief's §3 greps, every hit read:** (1) `grep -rn -i "only robot two\|robot
  two's autons are\|ROBOT=tank" docs/guide docs/*.md README.md src/` — 42 lines in 21 files, each read. Now FALSE
  and fixed (comments only, no line added or removed): `src/common/boot_source.cpp:49-50` ("robot two's claws in the
  ROBOT=tank builds, none elsewhere" → robot one's cascade in ROBOT=one's), `src/common/driver_panel.hpp:424`
  ("every build but ROBOT=tank" → "…ROBOT=tank and ROBOT=one"), `src/robots/calibrate_facts.hpp:62` ("Calibrate
  only for ROBOT=tank" → "ROBOT=tank|one"); `build_selection.hpp:92`'s "only robot two and robot one have a
  driver file" is the new, true message. Left as TRUE of robot two (each names robot two's builds, none says
  "only"): README.md:207, guide 16 :52, guide 17 :27 (**the command line only — its next sentence, :31-32, "`make`
  does not notice that the `PROGRAM=` word changed", was FALSE once the stamp landed and was not read: found by
  the docs reviewer, fixed in the review-fix round**), `src/main.cpp`'s build list (:24-36 — main.cpp is not G1-A's,
  TUNE1-B1 edits it), the four program headers' `make ROBOT=tank …` lines, chassis/mechanism tables' robot-two
  lines, bench_r3a (landmine 11), roadmap history. (2) `grep -rn -i "simulator" docs/guide/21-* docs/robot-facts-form.md`
  — empty. (3) `grep -rn "G1-PROGRESS" docs/internal/chunks/ROB1-*` — hits only THIS log's header note ("is NOT this
  log") and the brief quoting its own check; `G1-PROGRESS.md` itself unchanged (`git diff --stat` empty). Also read
  for staleness: `grep -rn -i "seven builds"` → `.github/workflows/ci.yml:146` (the gate's build list comment) gained
  "since ROB1-A TWELVE"; guide 09's "all seven" is a recipe listing (unrelated); roadmap's are history. Guide 21's
  "seven builds are byte for byte what they were" re-worded to what T6 proves (same machine code and constant data,
  compared function by function and object by object).
- **Docs finished at resumption:** `src/team/README.md` (§3 item 5 — robot one's two files, which file is which
  robot's, the gate rule); `docs/internal/build-order.md` (the "Next:" line, one ROB1-A sentence at its end);
  `tools/src_build_gate.py`'s HEADER (robot one's five rows in THE BUILDS; a section ROBOT ONE'S FACTS PROOF with
  step 0's ROB1-A rules, step 8's robot half and self-test cases 42–47 — the code cited "header, …" sections that
  did not exist yet); `.github/workflows/ci.yml` (one comment).
- **§7's `grep -n consteval src/robots/robot_one_facts.hpp` must be empty:** it found the banner's own warning
  ("NEVER a consteval function") — re-worded ("compile-time-only (immediate) function"); now empty.
- **GM15b ([R5], T6's own mutation), on the T6 `change/` COPY (never the worktree):** one blank line added above
  `// [robot two mechanisms]`, `ROBOT=tank PROGRAM=drive` rebuilt: per-function disassembly 0 differing, `strings`
  set unchanged (only timestamps) — **the constant-data compare turns RED**: `t6_symbolic.py` names
  `robot_two::clawPivot6`, `clawPivot7`, `claw` and `kRobotTwoMechanisms`, each with its recorded line moved by one
  (0x73 → 0x74 …); the masked `t6_rodata.py` count moves 765 → 789. Restored (the file equals its backup).
- **MUTATIONS — host side** (`scratch/rob1-a/mut/run_host.py`, driver `mutate.py`: ONE exact replacement, backup
  under `scratch/rob1-a/mut/`, restored in `finally` and on SIGTERM/SIGHUP/SIGINT, mtime bumped; every build a
  `cmake --build` of the whole suite, so every object that includes the mutated file recompiles; the suite binary is
  DELETED before each build and after each restore, so the doc gates — which run before the relink — never run a
  mutant; every test run `ulimit -v 4194304; timeout 600`). Results (`scratch/rob1-a/mut/results.txt`):
  - GM1 `robotBeacon(RobotOne)` → robot two's — **CAUGHT (test assertion)** `build_selection_test.cpp:69`.
  - GM2 `mechanismTableFor(RobotOne)` → `kNoMechanisms` — **CAUGHT (test)** `mechanism_table_test.cpp:379`.
  - GM3 the dispatcher's RobotOne case → robot two's `registerAutons` — **CAUGHT (test)** `this_robot_test.cpp:58`.
  - GM4 one fact's check deleted — **CAUGHT (test)** `robot_one_facts_test.cpp:221` (one check per fact).
  - GM5 a literal `40_in` in the cascade's `.range` — **CAUGHT (test)** `robot_one_facts_test.cpp:314` (the region
    scan names the literal).
  - GM6 `typed(…, "")` — **CAUGHT by the ledger's own static_assert, by name** ("a typed fact needs a real source");
    the T2 doctest is not reached (the build stops first) — said so per the coordinator's lesson.
  - GM7 a check's message misspelt — **CAUGHT by the ledger's own static_assert, by name** ("the check below a fact
    must quote its name verbatim").
  - GM7b the `same()` refusal deleted — **CAUGHT (compile-refusal case red)** s76 "COMPILED … was NOT refused".
  - GM7c the `gate copy` refusal deleted — **CAUGHT (compile-refusal case red)** s75.
  - GM8 Stage B needed by Bench Tests — **CAUGHT by a static_assert, by name** ("Bench Tests and Calibrate must
    build on Stage A alone"; the T3 doctest pins the same table but is not reached).
  - GM9 Stage A not needed by Calibrate — **CAUGHT by a static_assert, by name** ("Stage A is needed by all five").
  - GM11 the OWED check's `||` → `&&` — **CAUGHT: every build fails** (the host build prints `robot one: OWED --`).
  - GM14 robot two's row word `CLAW` → `MECH` — **CAUGHT (test)** `this_robot_test.cpp:102`.
  - GM15 robot two's boot label re-worded — **CAUGHT by boot_source.hpp's static_assert, by name** ("robot two's
    boot label is kDeclaredMotorLabel, verbatim").
  - GM16 robot one's cascade reads `robot_two::clawPivot6.cartridge` — **CAUGHT (test)** `robot_one_tables_test.cpp:279`.
  - GM18 the homing push's sign flipped — **CAUGHT: the library's refusal, by name**
    (`position_the_homing_push_points_away_from_its_home_end`).
  - GM19 one port for the two-motor cascade — **CAUGHT: the wiring's refusal, by name**
    (`the_wiring_gives_a_different_number_of_motors_than_the_mechanism_has`).
  - GM20 Calibrate's facts take one motor per mechanism — **CAUGHT (test)** `robot_one_tables_test.cpp:235` (T9).
  - GM20b the non-drive-motor check deleted — **CAUGHT (compile-refusal case red)** s79.
  - GM21 robot one's label = robot two's — **CAUGHT (test)** `boot_source_test.cpp:1305`.
  - GM21b robot one's headline arm returns robot two's literal — **CAUGHT (test)** `boot_source_test.cpp:1301`.
  - GM22 two levels' buttons swapped in the driver file — first cut **BUILD-BROKE (doc gate: guide 21 quotes the
    lines verbatim), NOT counted**; **re-cut** (`run_recut.py`: the driver file AND guide 21's quoted copy mutated
    alike, both restored) — **CAUGHT (test)** `robot_one_driver_test.cpp:179` (the map's preset at each d-pad button
    is the ledger's level i).
  - GM23 the 40 in's kind Match — **CAUGHT by the LIBRARY'S REGISTRY REFUSAL, not the test's assertion**
    (relabelled by the review-fix round: the test case THREW at `robot_one_autons_test.cpp:57` because the registry
    refuses a 59 s hard stop in a 30 s match period; the kind check never ran). **Re-cut as GM23r** in the
    review-fix round below (Match with 27 s / 29 s times, inside the period).
  - GM24 the pneumatic form's `.moveTime` removed — first cut **BUILD-BROKE (doc gate), NOT counted**; **re-cut**
    (example test + guide 21's copy) — **CAUGHT: the library's refusal, by name**
    (`pneumatic_actuationtime_must_be_more_than_zero`).
  - **New, for the T6 fix's overloads:** GMx1 robot one's row-word `memcpy` removed — **CAUGHT (test)**
    `robot_one_driver_test.cpp:284` (row starts `MECH <port>`); GMx2 the LCD-word swap's condition broken —
    **CAUGHT (test)** `this_robot_test.cpp:140`; GMx3 `controllerScreenLines(f, Words)`'s line-2 rewrite removed —
    **CAUGHT (test)** `this_robot_test.cpp:141`.
  - Not this run's: GM17 (G1-T), GM26 (G1-B). GM10, GM12, GM13, GM13b, GM25, GM27, GM28 are the gate side (below).
- **MUTATIONS — gate side** (`scratch/rob1-a/mut/run_gate.py`; each loads a FRESH copy of the gate module and
  runs the gate's own functions; every build is a fresh `make` with `bin/`/`.d/` deleted by `run_build`):
  - GM10 the OWED check's `needs(kProgram, …)` term → `true` — **CAUGHT (gate verdict)**: `one` "refused naming the
    wrong facts — named and not owed by this program: [the camera's port, the hold volts, …]". (The checks are ONE
    macro, so the mutation drops `needs` for every fact at once — the brief's "a Stage B fact's" is per fact; the
    verdict names each extra fact either way.)
  - GM12 robot one's driver file without the cascade's control — **CAUGHT (gate copy proof)**: the COPY's `one`
    build fails with the library's `a_mechanism_has_no_driver_control_mark_it_autonOnly_if_that_is_meant` (the
    real-tree rows never reach it — they refuse on OWED first: the copy's builds are real).
  - GM13 the proof's typing pointed at the REPO's ledger (the backup restores it) — **CAUGHT (gate copy proof)**: 66
    problems, the last "the repository's src/robots/robot_one_facts.hpp CHANGED during the copy's proof" (re-run
    with every problem written, `mut/copy_proof_problems.txt`); the real ledger after: 17 `owed(`, 0 `typed(`, the
    refusal on.
  - GM13b the `finally`'s deletion of the copy removed — **CAUGHT**: after an interrupted proof
    `build/src-gate-facts/` is left (case 46's expectation, run by the driver).
  - GM25 `ROBOT=one` setting the TANK define — **CAUGHT (gate verdict)**: "build one LINKED although its program
    needs 10 owed fact(s)".
  - GM27 the stamp's comparison always equal — **CAUGHT (case 47's checks)**: "the PROGRAM switch compiled 0 of 12;
    a refused robot-one build left bin/hot.package.bin".
  - GM28 the stamp written but `bin/` not deleted — **CAUGHT (case 47's checks)**: the same two failures.
  - After: `git diff -- Makefile src/` carries no plant marker (the only "(self-test plant" string is self-test
    case 45's own text in `tools/src_build_gate.py`); no `build/src-gate-facts/`; every backup restored (`.bak`
    files are under `scratch/rob1-a/mut/` only).
- **Host suite after every mutation (fresh build, briefing regenerated before and after):** `suite_run.py green` —
  3173 cases, 3033012 assertions; `ctest -R compile_refusal` — 267 of 267 passed.

### Deltas from the brief (against THIS base, `b71f838`)
1. **The words reach the panel as OVERLOADS, not a defaulted parameter** (§1.1 `this_robot.hpp` / R7): robot two's
   `clawRowText`, `controllerWord`, `controllerScreenLines` keep their base text; robot one's take an explicit
   `Words`. WHY: T6 — the defaulted-template form changed GCC's inlining in robot two's Drive and Competition
   (logged above). `decideBootSourceFrom` keeps the defaulted `Words` (its packages did not change).
2. **The words travel as a TYPE** (`WordsOf<RobotId>`, `ThisRobotWords = WordsOf<kRobotId>`), with `wordsFor(RobotId)`
   underneath as the brief says: a value parameter is not a constant expression inside the function, so the
   `?:` between two literal formats (landmine 18) could not be decided at compile time.
3. **Each fact's check is ONE macro**, `ROBOT_ONE_CHECK(fact, "name")` (four `static_assert`s: the name quoted
   verbatim, OWED-and-needed, a real source, typed-but-still-the-placeholder), not four hand-written lines per
   fact. So §7's `grep -c static_assert` = `grep -c ' owed(\| typed('` does not hold literally (6 vs 21, banner text
   included); its intent holds as `grep -c '^ROBOT_ONE_CHECK('` = 17 = the facts, and T2 pins exactly one check per
   fact, naming it.
4. **17 facts** (10 Stage A, 5 Stage B, 2 Recorded) — the brief's boot line said "31" as an example. Drive's boot
   line: `robot one: 17 facts, 0 typed (…); 17 still owed, none needed by this program` (robot one's builds only).
5. **The homing push placeholder is −6 V**, not P3's −3 V (logged above: −3 V never reaches the 1.0 A signature on
   the host test plant).
6. **The flipper forms test is `test/robot_one_flipper_forms_example_test.cpp`** (brief: `…_forms_test.cpp`): the
   `*example*_test.cpp` name puts it in `api_doc_tool`'s EXAMPLE_SOURCE_GLOBS, so guide 21's three paste-ready
   blocks are checked VERBATIM against the code that compiles and runs them (GM24's first cut showed it working).
7. **HA:** HA-252..HA-255 registered; **HA-256 stays HELD** for G1-T (the twin's belief has no code to act on until
   then); **HA-257 stays HELD for G1-B** (changed by the review-fix round — first cut "released unused": the belief
   "each robot keeps its own SD card" is the procedure guide 21 gives until G1-B's record names its robot, and G1-B
   may miss the freeze; registered then, released when G1-B lands).
8. **T6's two trees at two paths** (`t6/base/`, `t6/change/`), not one path in turn — no package carries a path
   (the strings sets), so harmless; and T6 adds a **symbolic** constant-data compare beside P6's masked one,
   because ROB1-A REORDERS typeinfo/vtable objects in robot two's packages and the masked compare cannot tell a
   reorder from a change (it reports 646–765 bytes; the symbolic compare reports only the timestamps).
9. **Public documents: NINE touched** (eight at the first report; the review-fix round added guide 17, whose
   `rm -rf bin .d` sentence the stamp made false — the prose review had missed it), over rule 7's ~5 (the brief
   flagged six): guide 21 (new), the facts form (new), guide 07, guide 16, guide 17, the guide index
   (`docs/guide/README.md`: chapter 21's row AND its maintenance notes — chapter 21 quotes
   `src/team/robot_one_driver.hpp`, a third team file the verbatim scan covers), the changelog,
   `src/team/README.md`, the roadmap's "you are here" — plus `mkdocs.yml`'s nav. Flagged for the team lead.
10. **Files another chunk owns, touched minimally:** `src/robots/calibrate_facts.hpp` (CAL1-B's, in main since
    e458dc6) — ONE comment's words (:62), no line moved; `docs/internal/build-order.md` — one sentence appended to
    its "Next:" line; `.github/workflows/ci.yml` — one comment; `test/CMakeLists.txt` — the one `target_sources`
    line and its comment. **Not touched:** `src/main.cpp` (its build-list comment names robot two's builds only —
    TRUE, and TUNE1-B1 edits that file), `src/programs/bench_r3a.*` (G1-C's), `include/shulib/`.
11. **The gate's real-tree half IS the five rows** ([R11]'s preferred form — not built twice): `check` gives a
    robot-one row `facts_verdict`, which accepts "refused naming exactly its owed set, no package left".
12. **§7's `robot_one` grep was not empty at the second stop** (Drive's facts line, the panel's row-budget
    assert) — fixed by moving the line's words to `this_robot.hpp` and reading the table through
    `mechanismTableFor(RobotId::RobotOne)`; a test now mirrors the grep. **§7's `RobotTwoDriver` grep has one hit
    by design:** `src/common/driver_file.hpp:9`, a base comment (not G1-A's file), true of robot two.
- **Gate self-test, first full run since the resumption (08:19–09:04, 44 min 35 s; another worktree's self-test ran
  beside it):** EXIT 1 with ONE failure — case 28b pinned the table file's robot namespaces as robot two's alone
  (`{"robot_two": {"kRobotTwoMechanisms"}}`); the tree now has robot one's. **Re-pinned with the same force** — the
  exact two-robot set `{"robot_two": {…}, "robot_one": {"kRobotOneMechanisms"}}` (an equality, not a subset), a
  comment saying why. Every other case, 42–47 included, passed on that run.
- **Gate `check` (09:05–09:15, 10 min 09 s):** `src build gate: PASS (all 12 builds link or, robot one's, refuse naming
  exactly their owed facts (5 refused; every one links in the gate's typed copy); …)` — `one` / `one-calibrate`
  refuse naming exactly their 10 Stage A facts, `one-drive` / `one-autontest` / `one-competition` their 15 (A + B);
  in `build/src-gate-facts/` "17 owed facts typed … robot one's 5 builds link and pass every check"; the copy gone
  after.
- **[Not logged before the second stop, found at resumption]** the gate `self-test` re-run AFTER case 28b's re-pin
  (09:15–09:49, 34 min 40 s): `src build gate self-test: OK (300 detector cases, each against a real build)`, EXIT 0
  (`scratch/rob1-a/selftest4.log`). The source files' mtimes after 09:15 are the self-test's own plant/restore bumps
  (`git status` shows no change to `src/main.cpp`, `include/`, `src/team/autons.cpp`, `test/vendor/`).
- **RESUMED 2026-11-11 22:37 EST** (`date`) after the second usage stop (~09:40). Checked first: `git diff -- Makefile
  src/` and the untracked `src/` files carry no `(self-test plant`, `kSelfTest…` or `static_assert(true, "x")` plant;
  every mutation backup lives under `scratch/rob1-a/mut/` only; no gate/suite/make process running. Left to do: a
  fresh build from source, the suite, §7's remaining gates (the ARM header gate, the CI greps, the api_doc_tool
  five, doc_staleness_audit, briefing check) and the brief's own greps, then the report.
- **Fresh build at resumption (22:40):** `rm -f build/test/shulib_tests`, briefing regenerated, `cmake --build
  build/test -j8 --clean-first` (every object recompiled from source, 1 min 57 s), briefing regenerated:
  `suite_run.py green` — 3173 cases, 3033012 assertions; `ctest -R compile_refusal` 267 of 267. The ARM header gate
  (244 lines: 243 headers + main) clean; the CI PROS-free and sim-layering greps empty; `api_doc_tool` self-test /
  check-coverage (5542 entities, 200 headers) / check-fresh / check-examples (723 lines, 14 files) /
  check-removability exit 0; `doc_staleness_audit.py` clean; `briefing_status.py check` current.
- **§7's own greps (22:50) — A FINDING:** `git diff --stat b71f838 -- include/shulib` empty; robot two's three team
  files unchanged; robot two's mechanism region identical to the base's; `grep consteval` on the ledger empty; but
  `grep -rn 'robot_one' src … | grep -v 'src/robots/\|src/team/robot_one_\|src/common/this_robot.hpp'` was NOT
  empty: `src/programs/drive_program.cpp:643-645` (Drive's facts boot line read `robot_one_facts::kRobotOne…Count`)
  and `src/common/driver_panel.hpp:465` (the row-budget `static_assert` named `robot_one::kRobotOneMechanisms`). The
  gate's step 0 passed them (it fails the `robot_one` NAMESPACE; the ledger's identifier is `robot_one_facts`, and
  the panel's line is a static_assert), so nothing caught it before. **Fixed at the cause:** the line's words moved
  to `src/common/this_robot.hpp` — `robotOneFactsLineFor(out, size, facts, owed)` (the counts as parameters, so
  the host can test a ledger with facts typed) and `robotOneFactsLine(out, size)` over the ledger; Drive calls
  `robotOneFactsLine(line, sizeof line)` inside its `if constexpr (kRobotOneBuild)`; the panel's static_assert
  reads `mechanismTableFor(RobotId::RobotOne)` (mechanism_table.hpp's banner allows mechanismTableFor() inside a
  static_assert). Self-test case 42's comment re-worded to match (comment only). The `RobotTwoDriver` grep's one hit
  is `src/common/driver_file.hpp:9`, a COMMENT present at the base (it shows how robot two's driver type is
  declared — true; not G1-A's file) — left, logged as a delta.
- **New tests (`test/this_robot_test.cpp`):** "this robot (§7)" — the brief's grep, line for line, over every file
  under src/programs and src/common (this_robot.hpp exempt), raw text so a comment counts as the grep counts it;
  "this robot (D3)" — the facts line against counts taken HERE from `kRobotOneFacts`' summaries one by one, its
  return = the line's length, ≤ 200 bytes (one TermSink line), a 12-byte buffer cut and NUL-terminated with the
  full count returned, and `robotOneFactsLineFor(…, 17, 5)` / `(…, 17, 0)` exactly (robot one's ledger has 0 typed,
  so its own line cannot tell the facts slot from the owed slot — the parameterised form can).
- **Mutations (`scratch/rob1-a/mut/run_resume3.py`, the same driver: whole-suite rebuild, binary deleted before
  and after, capped, restored in `finally`/on signals):** GMy1 Drive's old direct `robot_one_facts::` read put back
  — **CAUGHT (test)** `this_robot_test.cpp:236` (§7); GMy2 the panel's assert back on `robot_one::kRobotOneMechanisms`
  (compiles on the host) — **CAUGHT (test)** `:236`; GMy3 the typed slot printing the owed count — **CAUGHT (test)**
  `:262`; GMy4 the owed slot printing the facts count (equivalent on today's ledger: 17 = 17) — **CAUGHT (test)**
  `:273` by the parameterised case; GMy5 the ledger wrapper passing the owed count as the facts count — **CAUGHT
  (test)** `:262`.
- **T6 RE-RUN on the final tree (22:57–23:02):** `t6/change/` re-synced from the worktree (`src/`, `include/`,
  `Makefile`, `common.mk`; `diff -rq` empty after — it had also missed the 08:00 prose-review comment edits), robot
  two's seven packages rebuilt (`run_t6.sh change`), compared to the same base packages (`scratch/rob1-a/t6/
  t6_report3.txt`): `.bin` sizes identical for all seven (71756 / 128972 / 153796 / 175792 / 107988 / 239564 /
  221452); per-function disassembly 0 differing in all seven (255 / 452 / 439 / 625 / 409 / 711 / 798 functions);
  `.data` masked 0 bytes in all seven; the masked `.rodata` compare 110 / 727 / 733 / 768 / 648 / 763 / 764 bytes
  (the COMDAT reorder logged above); the SYMBOLIC compare: the only differing objects are `_PROS_COMPILE_TIMESTAMP`,
  `_PROS_COMPILE_TIMESTAMP_INT` and (Drive, Competition) `kBuildTimeText`, the only unclaimed strings the
  timestamps; `strings -a` sets differ in nothing but timestamp strings (0 other lines, all seven); no `robot one` /
  `robot_one` string in robot two's Drive package.
- **[Not logged before the third stop, found at resumption]** the gate `check` on the final tree (22:59–23:08, 9 min
  09 s; `scratch/rob1-a/gate5.log`): `src build gate: PASS (all 12 builds link or, robot one's, refuse naming exactly
  their owed facts (5 refused; every one links in the gate's typed copy); …)`, EXIT 0. A `self-test` started at 23:08
  (`selftest5.log`, empty) was CUT by the team lead's pause and left a plant in the Makefile
  (`# (self-test plant: drive append dropped)` in place of the `PROGRAM=drive` append line) — the COORDINATOR found
  and restored it on 2026-11-12 (~20:21, the Makefile's mtime).
- **RESUMED 2026-11-12 20:21 EST** (`date`) after the team lead's pause (~14:00 on 11-11 by the coordinator's count;
  this log's last entry is 23:08). Checked first: `grep -rn 'self-test plant\|kSelfTest\|static_assert(true, "x")'
  Makefile src/ common.mk` empty; `git diff b71f838 -- Makefile` is exactly ROB1-A's (the `PROGRAM=drive` append
  present); the worktree's `src/`, `include/`, `Makefile`, `common.mk` are IDENTICAL to T6's `change/` copy (`diff
  -rq`, `diff`) — so the coordinator's restore equals the T6-verified Makefile and the tree is the one T6 and gate5
  checked; no `build/src-gate-facts/`; every mutation backup under `scratch/rob1-a/mut/` only; no gate/suite/make
  process of this worktree running (the one running build is the main tree's). Left: a fresh host build and the
  suite, the src gate `self-test` IN FULL and `check` again, §7's other gates, the report.
- **Fresh host build at the third resumption (20:23):** `rm -f build/test/shulib_tests`, briefing regenerated,
  `cmake --build build/test -j8 --clean-first` (1 min 12 s), briefing regenerated: `suite_run.py green` — **3175
  cases, 3033058 assertions** (the 3173 above + the two 22:50 cases "this robot (§7)" and "(D3)"); `ctest -R
  compile_refusal` 267 of 267 (190.6 s).
- **Gate `self-test` IN FULL (20:27–20:57, 30 min 10 s; nothing else touched the tree):** `src build gate self-test: OK
  (300 detector cases, each against a real build)`, EXIT 0 (`scratch/rob1-a/selftest6.log`). After it: no plant
  marker in `Makefile`/`src/`/`common.mk`; `src/`, `include/`, `Makefile` still identical to T6's `change/` copy.
- **Gate `check` (20:58–21:05, 7 min 39 s):** PASS — the 7 robot-two/bench/xdrive builds link; `one` / `one-calibrate`
  refuse naming exactly their 10 Stage A facts, `one-drive` / `one-autontest` / `one-competition` their 15; all 5
  link in `build/src-gate-facts/` (17 owed facts typed there only), the copy gone after (`gate6.log`).
- **§7's other gates (21:06–21:12):** the PROS-free and sim-layering greps empty; the ARM header gate CLEAN (244 lines:
  243 headers + main); `api_doc_tool` self-test OK / check-coverage (5542 entities, 200 headers) / check-fresh /
  check-examples (723 lines, 14 files) / check-removability exit 0; `briefing_status.py` self-test OK and check
  current; `doc_staleness_audit.py` self-test OK — its audit first said "test binary older than
  robot_one_facts.hpp and 11 more" (the self-test's restore bumps), so the suite was rebuilt (binary deleted first,
  briefing regenerated before and after; suite green 3175 / 3033058) and the audit re-run: clean, binary trusted.
- **The brief's §7 greps, re-run on the final tree:** `grep -rn 'robot_one' src --include=*.hpp --include=*.cpp |
  grep -v …` empty; `grep -n consteval` on the ledger empty; `grep -c '^ROBOT_ONE_CHECK('` 17; `git diff --stat
  b71f838 -- include/shulib src/team/autons.* src/team/robot_two_driver.hpp src/main.cpp` empty; the `RobotTwoDriver`
  grep's one hit is `src/common/driver_file.hpp:9` (delta 12). The prose-review grep (1) still 42 lines, unchanged
  since 08:00's reading; (2) `simulator` in guide 21 / the facts form: 0; (3) `G1-PROGRESS` only in this log's header
  and the brief.
- **ROB1-A's build run is COMPLETE** (host- and build-gated; nothing has run on robot one). Left for the coordinator:
  the three reviewers, a fresh verifier, the coordinator's checks, the commit.

### ROB1-A review findings and their disposition (the review-fix round, 2026-11-12 ~21:20 EST onward)

Three reviewers (correctness, tests, docs) returned 27 findings; deduplicated, 22. Each was re-checked against the tree
before anything was changed. **Every one was real; none is dismissed.** Each is fixed at its cause (or, F6, accepted
explicitly and written into the procedure). Numbered here F1–F22; the reviewers' RV ids are kept.

**MAJOR**
- **F1 — `needs()` failed open for a program it does not name (correctness + tests + docs, three reviewers).** Confirmed:
  `needs(p, Stage::B)` was `p == Drive || p == AutonTester || p == Competition`, so TUNE1-B1's staged `Program::Tune`
  (which homes the cascade, jogs it at `.unhomedVolts`, moves it to its first and last preset and records its hold) would
  build robot one on Stage A alone; the gate's `STAGE_PROGRAMS` duplicated the rule and `expected_owed()` treated an
  unknown program as Stage A only, so the gate would AGREE. **Fixed at the cause:** a new `needsStageB(Program)` — a case
  per PROGRAM, no `default` (a new program is `-Wswitch`: a host compile ERROR under `-Werror`, a gate FAILURE), only
  Bench Tests and Calibrate false, the after-switch return `true` (fail closed); `needs()` calls it. The gate: the
  classification is explicit (`STAGE_B_PROGRAMS`, `STAGE_A_ONLY_PROGRAMS`), `expected_owed()` raises on an unclassified
  program, and a new step-0 check `stage_table_problems()` reads `src/build_selection.hpp`'s `Program` enum and the
  ledger's switch back, case by case, and fails a missing case, a `default`, a disagreement with `STAGE_PROGRAMS`, or an
  unclassified robot-one row (self-test case 48). Host test: "EVERY Program is classified" walks the enum from the
  header's TEXT (so Tune is seen the day it is added) and checks every enumerator's case and value, a value past the
  last needs Stage B, and `needs()` holds no second `p == Program::` list. **For the coordinator (TUNE1-B1):** whichever
  of ROB1-A / TUNE1-B1 lands second must add `case Program::Tune:` to the TRUE group of `needsStageB()`, `"Tune"` to the
  gate's `STAGE_B_PROGRAMS`, `one` to `MECHANISM_ROBOTS`, and a `one-tune` row expecting all 15 Stage A + B facts — the
  host build (-Wswitch), the T3 test and step 0 each fail until it does. Not done here: `Program::Tune` does not exist
  at this run's base.
- **F2 — a ledger copied back from `build/src-gate-facts/` linked robot one's Drive (correctness).** Confirmed by reading:
  `type_every_fact()` flipped `kRefuseGateCopySource = true` → `false` IN THE LEDGER'S TEXT, so the copied-back file
  carried its own off-switch (the reviewer's probe: `make ROBOT=one PROGRAM=drive` exit 0, a 177136-byte package).
  **Fixed at the cause:** the switch is no longer in the ledger's text. The ledger reads a MARKER FILE —
  `#if __has_include("../../build/ROBOT-ONE-GATE-COPY-ONLY")` — relative to its own directory; the gate writes the
  marker at the COPY's root only (`facts_copy_proof()`), never edits the switch, and step 0 fails the marker in the
  repository's `build/` (`ledger_source_problems()`). A copied-back ledger (or all of the copy's `src/`) brings no
  marker, so the real tree refuses every "gate copy" source at make — in EVERY build. Self-test case 49: the copy's
  own typed ledger written over the repository's and `make ROBOT=one PROGRAM=drive` run DIRECTLY must refuse by name
  with no package left; the marker planted in the repo must fail step 0. Host test: the `#if/#else` pinned verbatim,
  each value stated once. Why `__has_include` and not a `-D` on the copy's make line (the reviewer's first option):
  the gate's header forbids EXTRA_CXXFLAGS on the command line (GATE1 §4.1) and step 0 treats every `SHULIB_*` macro
  as a build switch outside the one chain; a file probe adds no switch and no make variable.
- **F3 — robot one's 40 in could drive any distance (tests, RV1).** Confirmed. New T12 case: the 40 in ends within
  0.5 in of (40, 0) and 1.2° of 0 by the plant's true pose AND the end estimate, FINISHED, 1 of 1 steps; the
  acceptance run ends within 0.6 in of its start, reversed (±3°), 4 of 4 — robot two's bounds (autons_test (3), (4)).
- **F4 — guide 17 still said `make` does not notice a `PROGRAM=` change (docs).** Confirmed: guide 17 :31-32 was false
  since the stamp; the prose review read :27 only. Reworded like guide 16; the prose-review entry above annotated.
- **F5 — the facts form had no item for two Stage B facts (correctness + docs).** Confirmed: the homing push and the
  unhomed volts gate Drive/AT/Competition and no item asked for them; "the rest of 17" gated nothing. **Fixed:** item 15
  (C8) is now three voltages — (i) hold, (ii) the homing push (negative = down, gentle), (iii) before homing — each
  measured or "chosen, not measured"; item 12 (C5) says FOUR TO SIX levels; the footer lists exactly the ledger's
  Stage A and B items (17 stops no build yet — it does once the flipper's block and facts are added); the C-section
  intro names the five items Bench Tests needs; the worksheet's answer table gains C8 (ii) and (iii); the ledger's
  comments name C8 (i)/(ii)/(iii).
- **F6 — the order skipped typing the drive cartridge and IMU into `kRobotOneChassis` (docs).** Confirmed (Calibrate
  refuses an unset cartridge; the boot refuses a tank with no IMU). **Fixed:** guide 21's file table and order step 1,
  the worksheet's R1.1 and the form's footer say A2 and A4 are typed into `kRobotOneChassis` with its provenance, before
  Calibrate. **Consistency fix found doing it:** T2's "robot one's chassis table holds only UNSET zeros" would have
  FAILED the day the IMU port was typed — it now allows `.imuPort` (and the cartridge, an enum) and requires the
  provenance to stop saying "nothing typed" once either is typed; every other number stays an UNSET zero.
- **F7 — guide 21 and the form said both "jog to the top stop" and "never jog into it" (docs).** Confirmed. One
  instruction now, in guide 21 (order step 3 → the last section), the form's item 14 and the worksheet R1.2/R1.4: the
  ceiling stays at 3 V (never R1); jog up at 3 V to just short of the top stop and stop; close the last bit by hand
  only if the gearing turns freely by hand; otherwise tell the coordinator and the top row waits for the capped
  station (ROB1-C). Station 13's own cut-outs named (two motors fighting, 2.4 A / 250 ms, the stall signature, 3 s).

**MINOR**
- **F8 — a pasted flipper block reaches a motor with invented numbers (correctness + docs).** Confirmed: the ledger holds
  only the flipper's KIND; guide 21 said "its numbers typed into the facts file first" as if facts existed. `make` does
  not run T2's region scan (the host suite does). **Fixed in the docs, not the ledger:** guide 21's flipper section is
  now three steps — each of the block's numbers becomes an OWED fact with its check and stage FIRST (listed: a motor
  arm's port, cartridge, teeth, stall, cap, range at Stage A; its angles / a pneumatic's stroke time at Stage B), then
  the block is pasted reading `facts::…`, and the suite (not `make`) is what fails a literal left behind; the ledger's
  banner says the same. Why not add owed flipper facts now: D6 — nothing about the flipper is declared until the
  builder names its kind, and the facts differ by kind. **For G1-F's estimate:** turning a block into its ledger form is
  designing new facts, checks, `kRobotOneFacts` entries and count pins — more than the brief's ½ hour; logged so it is
  not trusted.
- **F9 — a G1-F typing slip about robot one breaks robot two's builds too (correctness).** Confirmed (robot one's region
  and fact checks compile in every build). **Accepted, written into the procedure:** the ledger's banner, guide 21
  ("After each typing, build robot two too") and the worksheet R1.1 say the twelve gate builds are re-run after each
  typing; the likeliest slip (fewer than four levels) is closed in the form (F5).
- **F10 — Drive's field-DISABLED line said robot two's claws on robot one (correctness + docs).** Confirmed. Chosen on
  `ThisRobotWords::value.claw` between robot two's literal (verbatim) and robot one's ("each motor mechanism made safe,
  each pneumatic left as it was"), as Competition does; the source pin re-pinned to both literals in order.
- **F11 — robot one's program words were untested (tests, RV10).** Confirmed. New case "this robot (R7): EVERY per-robot
  choice": walks every `kRobotOneBuild ? a : b` in Drive (6: three named, three literal) and every literal
  `ThisRobotWords::value.claw ? a : b` in Drive, Competition and the Auton Tester (1 each), reading both operands from
  the source — robot one's side first and saying nothing of robot two (no "robot two", `robot_two`, "claw",
  `src/team/autons.cpp`), robot two's second; a named pair is `kXOne : kXTwo` and each definition's text is read too.
- **F12 — D5 did not pin the stall's rpm or lasting (tests, RV2; RV3 indirect).** D5 now checks `stall` and `homeStall`
  speed (rpm → rad/s) and time against their facts, that the three stall placeholders differ (so a cross-wire shows),
  the gravity kind Constant with `kG` = the hold volts, and `homePosition` = the bottom row.
- **F13 — `same()` was tested one way and only for double/Teeth (tests, RV6/RV7).** Every value type now: same as itself,
  and different when any ONE field differs — int, double, enum, Length, Time, const char*, Teeth, PortList (incl. the
  count), Pair<int>, Pair<Motor>, Rows (each row, both fields), Levels (each slot, name and height), CameraMount (each
  field). Not added: a new compile-refusal case for Rows (s83) — the numbering is shared with chunks in flight, and the
  static_asserts prove `same(Rows)`, which is the half s76/s77 do not already prove.
- **F14 — T2's number scan missed WORD facts (tests, RV4/RV5).** A bare-word scan: every identifier in robot one's region
  not reached through `.`, `::` or `->` must be one of the declaration's structural words (`Lift`, `cascade`, `gears`,
  `measured`, `Hold`, `facts`, …); `.cartridge = Red` / `.motors = {Forward, Reversed}` fail, and the scan is shown not
  blind on both.
- **F15 — the flipper-kind check was tested one way (tests, RV8/RV9).** Static asserts: "not on the robot yet" and a
  passive bar are refused against both flipper forms; an inches Lift named "flipper" satisfies none of MotorArm,
  Pneumatic, PassiveBar.
- **F16 — GM23 mislabelled (tests).** Relabelled above (the registry's refusal, not the test's); re-cut as GM23r.
- **F17 — the beacon-prefix test compared strings typed in the test (tests).** It now reads every `kVariantBeacon`
  literal out of `src/build_selection.hpp` (12, five of them robot one's) and checks prefixes both ways from those.
- **F18 — `robot_words.hpp`'s banner described the replaced design (tests + docs).** Reworded: TWO forms — the
  defaulted `Words` (boot source, dropped-autons line) and the explicit-`Words` OVERLOADS (the panel's three), and why
  (T6's inlining finding).
- **F19 — HA-255's containment named the cap at station 13, which applies none until ROB1-C (docs).** Reworded: at
  station 13 the 3 V ceiling and its cut-outs; the cap only in programs that bind the service.
- **F20 — HA-257 released while guide 21 acts on its belief (docs).** HA-257 is HELD for G1-B (delta 7 changed).
- **F21 — public overclaims (docs).** Guide 07: "every still-owed fact … that its program needs"; guide 21, the
  changelog and the ledger banner: no OTHER robot's or program's package is left, with D3's same-variant exception;
  the changelog: "reported to be a tank like robot two (HA-252, not yet seen)"; guide 21: "the seven robot-two, bench
  and X-drive packages".
- **F22 — deltas not logged; the guide index's maintenance notes stale (docs).** Deltas 13–14 below; the guide index now
  counts chapter 21 among the code chapters and `src/team/robot_one_driver.hpp` among the quoted team files (delta 9).

### Deltas from the brief (added by the review-fix round)
13. **A sixth new source file, `src/common/robot_words.hpp`** (§1.1 puts the per-robot words in `this_robot.hpp`): the
    words are data only, light enough for `boot_source.hpp` and `driver_panel.hpp`, whose host tests link no team file;
    `this_robot.hpp` includes it and selects the driver file and autons.
14. **Robot one's two fact checks are `consteval` refusal functions in the table file** (P4 had a `static_assert`):
    `robotOneNonDriveMotorsDeclared` / `robotOneFlipperKindMatches` call a refusal BY NAME, so a compile-refusal case
    (s79, s80) exercises the table file's own function. Safe against landmines 2/3: each is called directly in a
    `static_assert`, never under a short-circuited `||`, and is meant to fail EVERY build when the typed facts disagree
    with the list (robot one's region compiles everywhere) — not to be inert outside robot one's builds.
15. **The gate copy's switch is a marker FILE read by `__has_include`** (R4's text said the copy turns the refusal off;
    the first cut did it in the ledger's text — F2).
16. **`needsStageB()` — a case per program** (D3's stage table was written per stage — F1).
17. **T2's chassis-table rule allows the typed IMU port and cartridge** (F6).

### The review-fix round's mutations (`scratch/rob1-a/fix/run_fix.py`, `run_gate_fix.py`; results in `fix/results.txt`)
Same driver as before (`mut/mutate.py`: one exact replacement, backup under scratch, restored in `finally` and on
SIGTERM/SIGHUP/SIGINT, mtime bumped; the suite binary deleted before each build and after each restore; every whole-suite
rebuild recompiles every object that includes the mutated file; every test run `ulimit -v 4194304; timeout 600`).
**Found in the driver itself, and fixed:** its `suite()` counted ANY non-zero exit as caught — RV4/RV5's first cut used a
test filter with an apostrophe ("robot one's"), the shell quoting broke, and both read "CAUGHT" with no assertion. Neither
is counted; `run_fix.py` now uses a STRICT check (caught only on a doctest `ERROR:` line), and both were re-run.
(Every earlier `mut/results.txt` "CAUGHT (test assertion)" line was re-read: each names a real `file:line: ERROR:`.)
- RV1 the 40 in → 20 in — **CAUGHT (test)** `robot_one_autons_test.cpp:135` (truth x within 0.5 of 40).
- RV2 `.belowMotorRpm` ← the amps fact — **CAUGHT (test)** `robot_one_tables_test.cpp:134`; RV2b `.lasting` ← the amps
  fact — **CAUGHT (test)** `:135`; RV3 the gravity ← the unhomed-volts fact — **CAUGHT (test)** `:146` (`kG`).
- RV4 `.cartridge = Red` / RV5 `.motors = {Forward, Reversed}` typed into the region (re-run, strict) — **CAUGHT (test)**
  `robot_one_facts_test.cpp:472` (the bare-word scan) each.
- RV6 `same(Rows)` → false — **CAUGHT by the test's own static_assert** `robot_one_facts_test.cpp:420` (`same(rows,
  rows)`); RV7 `same(PortList)` → false — **the test's static_assert** `:406`; RV6b `same(Level)` ignoring the name —
  **CAUGHT (test)** `:437`.
- RV8 "not on the robot yet" always matches — **the test's own static_assert**
  `robot_one_flipper_forms_example_test.cpp:98`; RV9 `isArm` without Degrees — **the test's static_assert** `:112`.
- RV10 `false ? kRunStatusOne : kRunStatusTwo` — **CAUGHT (test)** `this_robot_test.cpp:313` (6 choices); RV10b the
  absent-devices operands swapped — `:323`; RV10c robot one's boot banner saying "robot two's" — `:332`; RVfd robot one's
  field-disabled literal saying claws — `:353`.
- RVT1 `needs()` back to `p == Drive || …` — **CAUGHT (test)** `robot_one_facts_test.cpp:239`; RVT2 Drive moved to the
  "A only" group — `:238`; RVT3 the fail-closed return → false — `:244`; RVT4 one case deleted (what an unclassified new
  program looks like) — first cut's expected text was wrong ("Wswitch"; GCC prints `-Werror=switch`), re-cut: **CAUGHT
  by the host COMPILER, not a test assertion** — `robot_one_facts.hpp:116: enumeration value 'Competition' not handled in
  switch [-Werror=switch]` (the point of the switch; the T3 test's count would catch it if the build went on).
- GM23r (GM23 re-cut: Match, 27 s / 29 s) — **CAUGHT (test assertion)** `robot_one_autons_test.cpp:74`
  (`forty.kind == AutonKind::Skills`).
- **Gate side.** P-copyback (a PROBE, no mutation — the reviewer's scenario on the fixed tree): the copy's typed ledger
  over the repository's, `make ROBOT=one PROGRAM=drive` run directly — **REFUSED**: exit 2, 17 "a typed fact needs a real
  source" lines, no `bin/hot.package.bin`. GMc1 the repository's `#else` value → false (the old design's effect) —
  **CAUGHT by case 49's check** (make exit 0, a package left). GMc1's first cut (`#if 1`) was refused earlier by
  `type_every_fact()` itself (the ledger no longer reads the marker) — a detection, but not case 49's; re-cut, said so.
  GMc2 the copy proof writing no marker — **CAUGHT (gate copy proof)**: the copy's builds refuse "a typed fact needs a
  real source". GMs1 `STAGE_B_PROGRAMS` without InventedXDrive — **CAUGHT (step 0's stage check)**. GMs2
  `expected_owed()` guessing again — **CAUGHT (case 48's check)**. GMs3 the `default` check removed — **CAUGHT (case
  48's check)**.

### Re-run after the fixes
- **T6 re-run** (`t6/change/` re-synced from the worktree — `diff -rq` empty — then `run_t6.sh change`; report
  `t6/t6_report4.txt`): robot two's seven packages against the base: `.bin` sizes identical (71756 / 128972 / 153796 /
  175792 / 107988 / 239564 / 221452); per-function disassembly 0 differing in all seven; `.data` masked 0 bytes; the
  symbolic compare's only differences the timestamps. Drive's new `?:` keeps robot two's literal only.
- **Host:** `rm -f build/test/shulib_tests`, briefing regenerated, `cmake --build build/test -j8 --clean-first`, briefing
  regenerated: `suite_run.py green` — **3178 cases, 3033472 assertions** (+3 cases: Program classification, the end
  poses, every per-robot choice); `ctest -R compile_refusal` **267 of 267** (267 TUs).
- **ARM header gate** CLEAN (244 lines: 243 headers + main); the PROS-free and sim-layering greps empty; `api_doc_tool`
  self-test / check-coverage / check-fresh / check-examples (723 lines, 14 files) / check-removability exit 0;
  `briefing_status.py` self-test OK, check current (next free HA-258 — HA-257 held); `doc_staleness_audit.py` self-test
  OK, audit clean.
- **Prose review of the round (rule 4; the greps run and every hit read before this line):** (1) `grep -rln
  "kRobotOneChassis\|robot_one_facts\|needsStageB\|ROBOT-ONE-GATE-COPY\|field DISABLED\|robot_words" docs/ README.md
  src/team/README.md` (docs/api and ROB1's own files excluded) — the form, guide 21, the worksheet, the changelog, the
  HA register, `src/team/README.md` (true), and history in G1-PROGRESS / T4-PROGRESS (`field DISABLED …` as a prefix —
  still true; history not rewritten). One more FALSE line found and fixed: the form's opening "Every answer here
  becomes one fact in `robot_one_facts.hpp`" (2 and 4 go into the drive table; several answers are no fact) → "Most
  answers …; the footer says which answer stops which build". (2) `grep -rn -i "only robot two\|rm -rf bin .d"
  docs/guide README.md` outside guides 16/17 — empty. (3) The register's `field DISABLED` reference
  (`hardware-assumptions.md:3073`) quotes the prefix only — still true. (4) `grep -rn "Competition ONLY\|rest of 17\|
  jogged into\|seven packages\|no package"` over docs/src/tools — every remaining hit is history or a gate message that
  is true (the gate's "no package left" lines are about refused builds after a variant change).
- **Files touched by the round:** `src/robots/robot_one_facts.hpp` (needsStageB, the marker read, banner, comments),
  `src/programs/drive_program.cpp` (one `?:`), `src/common/robot_words.hpp` (banner only), `tools/src_build_gate.py`
  (stage check, marker, cases 48–49, header), tests (`robot_one_facts_test`, `robot_one_tables_test`,
  `robot_one_autons_test`, `robot_one_flipper_forms_example_test`, `this_robot_test`, `drive_program_source_test`); docs
  (guides 07, 17, 21, the guide index, the form, the changelog, the HA register, the worksheet, this log). NOT touched:
  `include/shulib/`, robot two's tables/team files, `src/main.cpp`, `Makefile`, `test/CMakeLists.txt`.
- **Gate `self-test` IN FULL (21:50–22:18; other worktrees' self-tests ran beside it):** `src build gate self-test: OK
  (312 detector cases, each against a real build)`, EXIT 0 (`scratch/rob1-a/selftest7.log`; 300 → 312 with cases 48
  and 49). After it: no `(self-test plant`, `(probe)` or `GateRob1NewProgram` in `Makefile`/`common.mk`/`src/`; no
  `build/ROBOT-ONE-GATE-COPY-ONLY`, no `build/src-gate-facts/`; `src/` identical to T6's re-synced `change/` copy.
- **Gate `check` (22:19–22:27):** PASS, EXIT 0 (`gate7.log`) — the 7 robot-two/bench/xdrive builds link; `one` /
  `one-calibrate` refuse naming exactly their 10 Stage A facts, `one-drive` / `one-autontest` / `one-competition` their
  15; in `build/src-gate-facts/` (the marker written there only) "17 owed facts typed … robot one's 5 builds link and
  pass every check"; the copy gone after.
- **Last:** the form's opening line was corrected after the doc gates' first pass, so `api_doc_tool` check-fresh /
  check-examples and `briefing_status.py check` were re-run (exit 0 / current); the self-test's restore bumps made the
  staleness audit distrust the binary, so the suite was rebuilt (binary deleted first, briefing regenerated before and
  after): `suite_run.py green` — 3178 cases, 3033472 assertions; the audit clean, binary trusted. Not committed.

## ROB1-A — integration onto d33d25d

- **Executor:** `opus` (a workflow subagent), worktree `.claude/worktrees/rob1-int`, branch `rob1-int`, base **`d33d25d`**
  (main: STEP1-A + API 2.6 `196cdfc`/`1ed0e26`, TUNE1-A `6baef55`, TUNE1-B1 "shulib Tune" `c0a7c7e`, CURVE1-A `d33d25d`).
  Started 2026-11-13 00:0x EST. Not committed (the coordinator commits). Never `pros`, never `/dev/ttyACM*`. Scratch:
  `.claude/coordinator/scratch/rob1-int/` only.
- **The job:** ROB1-A's own diff (`scratch/rob1a-own-diff.patch`, `git diff b71f838 <its transport>`) applied with
  `git apply --3way`, every conflict resolved keeping BOTH sides' meaning, then THE SEMANTIC MERGE neither side could do:
  robot one has a mechanism table, so it gets "shulib Tune" (TUNE1-B1's own banner, `tune_program.hpp`: "a robot that
  registers a mechanism table adds its ROBOT value to the Makefile's MECHANISM_ROBOTS, a `<robot> && SHULIB_PROGRAM_TUNE`
  branch … its define to that file's Tune guard, and a `<robot>-tune` build to tools/src_build_gate.py's tables").
  ROB1-A's design is NOT changed.

### Log
- **Read first:** RESUMING.md's standing rules; the brief's header, answers block, D1–D13, §6 (coordination), §7
  (checks), §8 (landmines) and the Re-check R1–R11 in full; the TUNE1/Tune mentions (§6's "TUNE1-B1 … if before, G1-A
  carries it to robot one"); this log's ROB1-A section in full (F1's instruction to whichever lands second).
- **STEP 1 — `git apply --3way` (00:05):** 54 files applied cleanly (3-way or direct); 7 conflicted, exactly as the
  coordinator listed: `.github/workflows/ci.yml` (1 region), `docs/guide/07-getting-set-up.md` (1),
  `docs/hardware-assumptions.md` (5), `docs/internal/PROJECT-BRIEFING.md` (1, regenerated), `docs/roadmap.md` (1),
  `src/build_selection.hpp` (2), `tools/src_build_gate.py` (13). `git apply` warned "2 lines add whitespace errors"
  (ROB1-A's own lines; the gates decide).
- **The conflicts, each resolved keeping BOTH sides' meaning (00:05–00:30):**
  - `src/build_selection.hpp` (2): THE RULE's switch list names `SHULIB_ROBOT_ONE_2026` AND `SHULIB_PROGRAM_TUNE`;
    the beacon comment says "the other thirteen" and names both prefix families (`…=tank` of the five
    `…=tank-<program>`, `…=one` of robot one's five).
  - `tools/src_build_gate.py` (13): VARIANTS / BUILD_ARGS / VARIANT_DEFINES / VARIANT_BEACONS / BUILD_ROBOT /
    BUILD_PROGRAM carry `tank-tune` AND robot one's five (and, STEP 2, `one-tune`) — the order: robot two's eight
    as at `d33d25d`, then robot one's six; the header's THE BUILDS lists `tank-tune` and robot one's six; "WHAT
    `check` DOES" names both; TUNE1-B1's `elf_defined_names()`/`tuning_problems()` AND ROB1-A's
    `ROBOT_FOREIGN_*`/`robot_separation_problems()` are kept, each with its own `return problems`; step 8 calls
    all three (`program_problems`, `tuning_problems`, `robot_separation_problems`) from one `elf_bench_names()`
    read. **The self-test cases COLLIDED** — TUNE1-B1 landed its cases 42–49 and ROB1-A had numbered its own
    42–49: TUNE1-B1's keep 42–49 (they are committed and cited); **ROB1-A's are RENUMBERED 50–57** (every
    `# 4N` heading and every `"4N: …"` message, mechanically, the diff read line by line — only case-number
    lines moved), and every citation of them in the gate's header and code re-pointed (42→50 … 49→57).
    TUNE1-B1's cases 46 and 48 plant on the Makefile's `MECHANISM_ROBOTS:=tank\n` — re-aimed at
    `MECHANISM_ROBOTS:=tank one\n` (STEP 2 changes that line; `_replace` asserts exactly one occurrence, so the
    old target would have failed the self-test, not passed it).
  - `.github/workflows/ci.yml` (1): the comment names Tune's eighth build AND robot one's six — FOURTEEN.
  - `docs/guide/07-getting-set-up.md` (1): "shulib Tune" in robot two's list AND robot one taking "the same six".
  - `docs/hardware-assumptions.md` (5): the Held line keeps STEP1's HA-217/218 AND ROB1's HA-256/257; the count
    214 + 4 = **218**; the paragraphs STEP1-A (209), CURVE1-A (214), ROB1-A (now "With them, 218"); the table rows
    HA-216, HA-246..250 then HA-252..255; the detail sections STEP1-A, CURVE1-A, then ROB1-A's four under a new
    heading "### ROB1-A's addition — robot one in the code (2026-11-11)" (at ROB1-A's base its items sat under
    MECH-S's heading; two headings landed in between, so without one they would have read as CURVE1's).
  - `docs/roadmap.md` (1): TUNE1-A/B1, STEP1-A and CURVE1-A's "Now" paragraphs kept, ROB1-A's after them, dated
    2026-11-13 and naming its six programs.
  - `docs/internal/PROJECT-BRIEFING.md` (1): regenerated by `briefing_status.py generate` (never hand-merged).
- **STEP 2 — THE TUNE JOIN (robot one gets "shulib Tune"), the four build lines TUNE1-B1's banner names:**
  `Makefile` `MECHANISM_ROBOTS:=tank one`; `src/build_selection.hpp` a `SHULIB_ROBOT_ONE_2026 &&
  SHULIB_PROGRAM_TUNE` branch (`Program::Tune`, `RobotId::RobotOne`, `"shulib-robot-variant=one-tune"`) after
  `one-competition`, and `&& !defined(SHULIB_ROBOT_ONE_2026)` in the Tune guard; the gate's `one-tune` row in
  all six tables. **And the classification ROB1-A's F1 left for whichever landed second:** `case Program::Tune:`
  in the TRUE group of the ledger's `needsStageB()`, `"Tune"` in the gate's `STAGE_B_PROGRAMS`. Without it the
  host build fails (`-Werror=switch`), the T3 test fails (it walks the enum's text and demands every program but
  Bench Tests and Calibrate need Stage B), and the gate's step 0 fails (`stage_table_problems`).
- **A DELTA FROM THE COORDINATOR'S TASK TEXT, logged for the coordinator (not a change to ROB1-A's design — the
  task text and ROB1-A's design disagreed, and ROB1-A's was kept):** the task said `one-tune` "like robot one's
  Bench Tests, needs only Stage A facts (robot_one_facts.hpp needs(): Stage A for every program) and so REFUSES
  naming exactly its owed Stage A facts". ROB1-A's design says the opposite, explicitly: F1 (three reviewers)
  — "whichever of ROB1-A / TUNE1-B1 lands second must add `case Program::Tune:` to the TRUE group of
  `needsStageB()`, `"Tune"` to the gate's `STAGE_B_PROGRAMS` … and a `one-tune` row expecting all 15 Stage A + B
  facts"; the T3 host test enforces it for any program; and the reason holds in TUNE1-B1's code
  (`include/shulib/mechanism/tune_program.hpp`'s banner: HOME homes the cascade with the homing push, the
  stick jogs it at `.unhomedVolts` before homing, TEST A / TEST B move a position mechanism "to its first and
  last preset", the report reads the hold) — every one of the five Stage B facts reaches a motor in Tune. Built
  on Stage A alone, a robot-one Tune upload would drive the cascade on placeholder rows, levels, homing push and
  volts: exactly what the facts ledger (D3) exists to stop. ("Stage A for every program" is true of `needs()`'s
  Stage A; Stage B is `needsStageB()`.) The cost, said plainly: Tune cannot be used to FIND the first hold volts
  or homing push — those are typed first as "chosen, not measured" (the form's item 15 already allows it) and
  then tuned with Tune and re-typed from its COPY. If the team lead rules Tune Stage-A-only, it is ONE line moved
  in `needsStageB()` and one name moved between the gate's two sets (plus T3's `measures` rule and the docs).
  STEP 4's third mutation is cut to match the design kept: `one-tune` NOT needing Stage B → red.
  **It also supersedes the brief's §2.0 item 15** (the form's C8: the hold volts from "TUNE1's page if it reaches
  robot one; otherwise chosen with the coordinator") — with Tune needing Stage B, the FIRST hold volts (and homing
  push) cannot come from Tune's page: they are typed "chosen, not measured", and Tune then tunes them and its COPY
  is re-typed into the facts (guide 21's "Tune's COPY on robot one"). The coordinator's ruling stands open
  (added by the review-fix round, 2026-11-13).
- **The build count:** the task text said "13 builds (robot two 7 + Tune for robot two = 8; robot one 5 + one-tune
  = 6 …)"; 8 + 6 = **14**, and fourteen is what the gate runs (VARIANTS has 14 rows).
- **Robot one's Tune, everywhere ROB1-A lists its programs:** the gate's step 8 live-tuning half (`tuning_problems`
  keys on `BUILD_PROGRAM` — `one-tune` is "Tune", so in the typed copy it must link live tuning and no chassis;
  robot one's match programs must link a chassis symbol) and robot half (`ROBOT_FOREIGN_*` by `BUILD_ROBOT`) —
  generic, nothing to add; the variant stamp — generic (`PROGRAM=` is in the stamp; checked by hand below);
  the facts' program list (`test/robot_one_facts_test.cpp`: `kFivePrograms` → `kRobotOnePrograms` with Tune,
  a static_assert that Tune needs Stage B, Tune in the owed-count loop's Stage-B set, `owedFor(Tune) == a + b`);
  `test/this_robot_test.cpp` (the beacons read from the chain: 14, robot one's 6); `test/build_selection_test.cpp`
  (TUNE1-B1's `makeValueOf()` gains `RobotOne → "one"` — required by its own `-Wswitch` — and the
  MECHANISM_ROBOTS case pins `one` and exactly two robots); guides 07 and 21, the guide index, the facts form,
  the worksheet, ci.yml, the changelog, the roadmap, REMAINING-WORK, build-order, RESUMING, HANDOFF.
- **Found by the merge (a sentence each side wrote true, false together):** guide 18's Tune build says "The
  `rm -rf bin .d` matters for the same reason as the Auton Tester's: `make` does not see that `PROGRAM=`
  changed" — TUNE1-B1 wrote it after ROB1-A's base; ROB1-A's variant stamp makes it false (ROB1-A's F4 fixed
  the same sentence in guide 17). Reworded like guides 16/17, and it names `ROBOT=one` and robot one's refusal.
- **A NEW TEST for the join** (`test/tune_program_test.cpp`, appended — TUNE1-B1's file, no chunk in flight
  edits it): "Tune (ROB1-A's integration): over ROBOT ONE's own table …" — the SAME Tune program over
  `robot_one::kRobotOneMechanisms` on the host test plant (the plant built from the declaration, as
  `robot_one_tables_test.cpp`'s `cascadePlant()`): READY; HOME through stage 2's GO pushes DOWN (never harder
  than the facts file's push, never above the cap) onto the bottom row and logs `'cascade': HOME SUCCEEDED`;
  TEST A reaches the first level ("floor"); TEST B drives it UP toward the last ("L3", 24 in); and across the
  whole session the provider is asked ONLY for the cascade's two ports (8, 9 — the placeholders), no other smart
  port, no 3-wire line. **A FINDING it measured (for the coordinator; TUNE1-B1's design, not changed):** on the
  facts file's placeholder gearing (red cartridge, 12:60 — 120 °/s at the output) TEST B floor → L3 (1080° at
  the output) needs ~9 s of travel, so its limit is Tune's 10 s cap (`kTuneTestMostSeconds`,
  `tunePositionBound`) and the verdict is **TIMED_OUT at 887.8° of 1080°**. The case pins it honestly (SUCCEEDED
  when the bound is under the cap, TIMED_OUT at the cap, and driven up more than half the way either way). If
  the real cascade is that slow over its range, Tune's TEST B on its top level can never succeed — the team
  learns its real gearing on Nov 14.
- **First host build (00:30) and suite:** `rm -f build/test/shulib_tests`, briefing regenerated, `cmake --build
  build/test -j8` (1 min 11 s), briefing regenerated: `suite_run.py green` — **3405 cases, 3781604 assertions**
  (before the new Tune case); `ctest -R compile_refusal` **267 of 267** (267 TUs, s74–s82 ROB1-A's; no number
  collides with main's s61–s73).
- **Gate `check` (00:34–00:47):** `src build gate: PASS (all 14 builds link or, robot one's, refuse naming
  exactly their owed facts (6 refused; every one links in the gate's typed copy); …; live tuning only in Tune,
  Tune with no chassis; …)`, EXIT 0 — the 8 robot-two/bench/xdrive builds link; `[tune-row]` clean (MECHANISM_ROBOTS
  = the robots with mechanisms, each with its tune build); `one` / `one-calibrate` refuse exactly their 10 Stage A
  facts, `one-drive` / `one-autontest` / `one-competition` / **`one-tune`** exactly their 15; in
  `build/src-gate-facts/` "17 owed facts typed … robot one's 6 builds link and pass every check" (so the typed
  `one-tune` links live tuning and no chassis, and carries no robot-two word); `[chain-table] 14 builds, 4 robots`.
  (`scratch/rob1-int/gate_check1.log`.)
- **The ARM header gate** CLEAN (250 lines: 249 headers + main, `-Werror`); the PROS-free and sim-layering greps
  empty; `git diff --stat d33d25d -- include/shulib` EMPTY (no library change).
- **T6 / [R5] — robot two's (and bench's, xdrive's) EIGHT packages, `d33d25d` vs the integrated tree (00:36–00:40;
  `scratch/rob1-int/t6/`, both trees copies under `GIT_CEILING_DIRECTORIES` so no build hash enters either, every
  make capped; report `t6/t6_report.txt`):** `.bin` sizes identical for all eight (71756 / 130004 / 153940 /
  178284 / 108136 / 246860 / 228332 / 120288 — bench / xdrive / tank / drive / calibrate / autontest / competition
  / tune); per-function disassembly modulo addresses **0 differing functions** in all eight; `.data` masked 0
  bytes; the masked `.rodata` compare 100–761 bytes, and the SYMBOLIC compare (ROB1-A's `t6_symbolic.py`) names
  ONLY `_PROS_COMPILE_TIMESTAMP`, `_PROS_COMPILE_TIMESTAMP_INT` and (Drive, Competition) `kBuildTimeText` — the
  COMDAT reorder ROB1-A logged, nothing else; `strings -a` of each `.bin` differs ONLY in timestamp strings, and no
  `.bin` carries a robot-one string (`robot one`, `robot_one`, `one-tune`, `=one`: 0). (`strings` of the `.elf`
  differs — its DWARF names `robot_one_facts.hpp`, `robot_one`; the ELF is not uploaded, the `.bin` is.) Robot
  two's Tune package is unchanged too, robot one's Tune branch notwithstanding.
- **[R1] the stamp, by hand, in the T6 change copy:** `make ROBOT=tank` (13 files) → `make ROBOT=tank
  PROGRAM=tune` with no rm prints `shulib: the build changed …`, compiles 13 of 13, and its package carries the
  `tank-tune` beacon; the same again compiles 0; then `make ROBOT=one PROGRAM=tune` prints the stamp line, exits
  2 with exactly 15 `robot one: OWED --` names (`grep -o … | sort -u | wc -l` = 15: the 10 Stage A + the 5 Stage
  B), and `bin/hot.package.bin` is GONE.
- **STEP 4 — the semantic merge's mutations** (`scratch/rob1-int/mut/mutate_int.py`: ONE exact replacement,
  backup under `scratch/rob1-int/mut/`, restored in `finally` AND on SIGTERM/SIGHUP/SIGINT, mtime bumped; gate
  halves in a FRESH python importing the worktree's gate, every build `run_build()` — `bin/`/`.d/` deleted
  first, a fresh package build; host halves: the suite binary DELETED, `cmake --build` (every object including
  the mutated file recompiled by cmake's header dependencies), a doctest filter under `(ulimit -v 4 GiB; timeout
  600)`, the binary deleted again after the restore; results `mut/results.txt`):
  - **IM1 — `MECHANISM_ROBOTS` without `one`:** **CAUGHT (gate, step 0's [tune-row])** "robot 'one' has a
    mechanism table but the Makefile's MECHANISM_ROBOTS does not name it"; **CAUGHT (gate, the one-tune row)**
    make exit 2 at the Makefile's `$(error) PROGRAM=tune needs a ROBOT whose mechanism table is not empty — 'one'
    …`, the facts verdict naming all 15 owed and unnamed; **CAUGHT (test)** `test/build_selection_test.cpp:155`
    (`listedRobots == withMechanisms`).
  - **IM2 — robot one's Tune branch dropped from the chain:** **CAUGHT (gate, the one-tune build)** — it builds
    as robot one's Bench Tests and refuses with 10, "owed and not named: [the 5 Stage B facts]"; **CAUGHT (gate,
    step 0's chain table)** "build one-tune: 0 branches of THE ONE CHAIN carry its beacon"; **CAUGHT (test)**
    `test/this_robot_test.cpp:95` (`REQUIRE(beacons.size() == 14)`: 13).
  - **IM3 — robot one's Tune NOT needing Stage B (the ledger: `case Program::Tune:` moved to the false group;
    the coordinator's third mutation re-cut to the design kept, see the delta above):** **CAUGHT (gate, the
    exact-owed-list check)** "build one-tune refused naming the wrong facts — owed and not named: [hold volts,
    homing push, levels, rows, volts before homing]"; **CAUGHT (gate, step 0's stage check)** "Program::Tune: the
    ledger's needsStageB() says false, this gate's STAGE_PROGRAMS says true"; **CAUGHT (host build: the test's
    own static_assert, by name)** "shulib Tune needs Stage B" (`test/robot_one_facts_test.cpp`).
  - **IM3b — the GATE's side: `"Tune"` moved from `STAGE_B_PROGRAMS` to `STAGE_A_ONLY_PROGRAMS` (what the task
    text described):** **CAUGHT (gate, the exact-owed-list check)** "named and not owed by this program: [the 5
    Stage B facts]"; **CAUGHT (gate, step 0's stage check)** "the ledger's needsStageB() says true, this gate's
    STAGE_PROGRAMS says false".
  - **IM4 — the Tune guard without robot one's term:** **CAUGHT (gate, the one-tune build)** "refused with an
    error that is not an OWED fact: src/build_selection.hpp:108 — #error \"PROGRAM=tune needs a ROBOT …\"".
  - **IM5 — the gate's `one-tune` row dropped from VARIANTS alone:** **CAUGHT (gate, step 0's [tune-row])**
    "robot 'one' is in MECHANISM_ROBOTS but this gate runs 0 Tune build(s) for it".
  - **IM6 — robot one's homing push weakened to −2 V (it can no longer reach the 1.0 A signature on the plant):**
    **CAUGHT (test)** `test/tune_program_test.cpp:2168` (`REQUIRE(…homed())`) — the new Tune-over-robot-one case.
  - **The driver's own misses, said so (none counted):** the FIRST run's host halves of IM1 and IM2 used a
    doctest filter with an apostrophe ("the Makefile's", "robot one's"), the shell quoting broke and they read
    "SURVIVED" with no test run (ROB1-A's driver met the same) — re-run with `shlex.quote` and a "matched no case
    = NOT RUN" guard; the second run's IM2-host and IM6 read "SURVIVED" because the driver counted only `ERROR:`
    lines and a failed `REQUIRE` prints `FATAL ERROR:` — re-run with both counted. Every verdict above is from the
    final runs (`run2.log`, `run3.log`).
  - After every run: `git diff -- Makefile src/` carries no `(self-test plant`; each mutated file equals its
    pre-mutation text (the driver asserts it); no suite binary left.
- **Gate `self-test` IN FULL (00:51–01:29; vis1-a2's self-test ran beside it in its own worktree):** `src build
  gate self-test: OK (349 detector cases, each against a real build)`, EXIT 0 (`scratch/rob1-int/selftest1.log`)
  — TUNE1-B1's cases 42–49 (46 and 48 re-aimed at `MECHANISM_ROBOTS:=tank one`) and ROB1-A's renumbered 50–57
  all pass. After it: no `self-test plant`, `kSelfTest`, `GateRob1NewProgram` or `static_assert(true, "x")` in
  `Makefile`/`common.mk`/`src/`; no `build/ROBOT-ONE-GATE-COPY-ONLY`, no `build/src-gate-facts/`; `src/`
  identical to the T6 change copy taken before it.
- **THE FINAL TREE, every gate (01:30–01:52):** `rm -f build/test/shulib_tests`, briefing regenerated, `cmake
  --build build/test -j8 --clean-first`, briefing regenerated: **`suite_run.py green` — 3406 cases, 3783633
  assertions** (3405 + the Tune-over-robot-one case); `ctest -R compile_refusal` **267 of 267**, TUs 267 = CTests
  267; **gate `check` PASS** — "all 14 builds link or, robot one's, refuse naming exactly their owed facts (6
  refused; every one links in the gate's typed copy)", `[tune-row]` and `[chain-table] 14 builds, 4 robots`
  clean (`gate_check2.log`); `api_doc_tool` self-test OK / check-coverage (6062 entities, 206 headers) /
  check-fresh / check-examples (776 lines, 16 files) / check-removability — all exit 0; `briefing_status.py`
  self-test OK, check current (218 registered, next free **HA-258**, ROB1 among the open chunks);
  `doc_staleness_audit.py` self-test OK, audit clean (binary trusted). The ARM header gate and the CI greps as
  above (no header changed since).
- **The brief's §7 greps on the final tree:** `git diff --stat d33d25d -- include/shulib` empty;
  `git diff d33d25d -- src/team/robot_two_driver.hpp src/team/autons.hpp src/team/autons.cpp` empty; robot two's
  region (`[robot two mechanisms]` … `[/robot two mechanisms]`) identical to `d33d25d`'s AND at the same lines
  (its end marker on :135 in both — landmine 19); `grep -n consteval` on the ledger empty; `grep -c
  '^ROBOT_ONE_CHECK('` 17; the `RobotTwoDriver\|registerAutons(` grep's one hit `src/common/driver_file.hpp:9` (ROB1-A's
  delta 12, a base comment); the `robot_one` grep outside its own files empty; `G1-PROGRESS.md` unchanged;
  `make ROBOT=one PROGRAM=drive` and `PROGRAM=competition` (in the T6 change copy, so the worktree's gate was
  never disturbed) exit 2 naming 15 owed facts each, `bin/hot.package.bin` absent after; the repository's
  ledger after every gate run: 17 `owed(` declarations, 0 `typed(` declarations.
- **Prose review (rule 4 — each grep RUN and every hit READ before this line was written):**
  (1) `grep -rn "MECHANISM_ROBOTS\|PROGRAM=tune\|one-tune" docs/ README.md src/team/README.md` (outside docs/api and
  the TUNE1/ROB1/MECH-S records) — 13 hits: guide 21's two (new, true), HANDOFF :322-323 (was "today robot two" —
  FALSE, fixed), the changelog's two (ROB1-A's, new; TUNE1-B1's :156, history, true), guide 18 :527 (the
  command line — true; its next sentence was FALSE, fixed above), RESUMING :118 (the gate's list — updated to
  fourteen), guide 07 :148 (true), roadmap :521-535 (TUNE1-B1's paragraph — "robot one gets it by registering
  its table plus the four build lines" — true, now done; ROB1-A's paragraph names it), README :208 (true).
  (2) `grep -rn "needsStageB\|STAGE_B_PROGRAMS" docs/ README.md` — guide 21 :88 only (true: "lists every
  program by name"). (3) `grep -rn "case 4[2-9]\|cases 4[2-9]"` over docs/tools/test/src (outside the TUNE1 and
  ROB1 logs) — every hit is TUNE1-B1's own case (gate :174, :426, :698, :1262, :4091; changelog :158) — true.
  (4) `grep -rn -i "robot one's five\|five programs\|five builds\|twelve builds\|all twelve\|the gate's twelve"`
  over the live docs, src, tools, test, ci, Makefile — remaining hits are history (build-order :644/:1925,
  roadmap :245, REMAINING-WORK :221) or TRUE ("`…=one` [is a prefix] of robot one's five `…=one-<program>`",
  gate :135, `build_selection.hpp` :139; "all twelve" buttons). (5) `grep -rn "Drive, the Auton Tester and
  Competition\|Drive, the Auton Tester, Competition\b"` — every hit now names Tune (form :81, guide 21 :83/:154,
  HA register :2070, worksheet :2722, ledger :37/:409). (6) `grep "rm -rf bin .d\` matters\|does not see that
  \`PROGRAM=\`\|make\` does not notice"` over docs/README (outside chunk records) — EMPTY after the guide 18 fix.
  (7) "simulator" in the files this run touched — only `tune_program_test.cpp`:11's own "never 'simulator'".
  NOT re-read (ROB1-A's own prose review covers them and this run did not touch their subject): guides 16, 17,
  the facts form outside its footer, `src/team/README.md`.
- **Stale counts NOT this run's, left (for the coordinator):** `src/programs/xdrive_program.cpp:35,37`,
  `src/common/fault_latch_hook.hpp:17`, `src/common/boot_source.cpp:12`, `src/programs/auton_tester_program.cpp:62`
  say "all seven builds" (eight since TUNE1-B1, fourteen now) — comments about every `.cpp` compiling into every
  build, wrong since TUNE1-B1 landed; editing them moves no package byte but they are not this merge's lines.
  `docs/hardware-assumptions.md:495` (`d33d25d`:487, STEP1-A's): a lone `>` line between the HA-223 and HA-216 table rows
  ends the table, so HA-216 and HA-246..HA-255 render as plain text in mkdocs — main's defect, carried as is
  (ROB1-A's four rows follow CURVE1-A's below it).
- **ROB1-A's integration is COMPLETE** (host- and build-gated; nothing has run on robot one or on robot two's
  Tune). Not committed. Left for the coordinator: the review, the Stage B ruling on Tune (above), the commit.

### ROB1-INT review findings and their disposition (the review-fix round, 2026-11-13 ~02:20 EST onward)

Three reviewers (correctness, tests, docs) returned 13 findings; deduplicated, eleven. Each was re-checked against
the tree before anything was changed. Scratch: `.claude/coordinator/scratch/rob1-int/` (`mut-review/`, `t6/change2*`,
`gates_rf.log`, `selftest_rf.log`, `gate_check_rf.log`). Not committed. Never `pros`, never `/dev/ttyACM*`.

- **RF1 — stale robot-one/robot-two build counts in comments and docs (correctness #1; docs #8 (a)–(e)). REAL, fixed.**
  `src/robots/chassis_table.hpp:211` "the five ROBOT=one builds" → six; `src/robots/mechanism_table.hpp:142`
  (landmine 19's warning, BELOW robot two's region) "all five of robot two's tank packages" → six (tank-tune
  records robot two's lines too); guide 21 :42 "the seven robot-two, bench and X-drive packages" → eight (T6
  compared eight); `.github/workflows/ci.yml`'s header "eight since TUNE1-B1 … three robot variants" →
  fourteen with robot one's six; RESUMING's "Variant selection is" sentence now names `PROGRAM=tune` and
  `ROBOT=one` with its five `PROGRAM=` values. Each src edit is a word inside its line (no line added or removed —
  landmine 19; R5 re-run below). **Found by this round's own grep, same class, fixed:** `src/main.cpp:49-52` said
  which tables "the bench build … the six tank builds … the xdrive build" carry and left robot one's six out —
  re-worded in the SAME four lines.
- **RF2 — the lone `>` in the HA register's table (correctness #2; docs #13). REAL, fixed.** `docs/hardware-assumptions.md`
  :495 deleted (STEP1-A's stray line, `d33d25d`:487 — `git blame` → `196cdfc`). Rendered with Python-Markdown
  (`tables`): HA-216, HA-246, HA-250, HA-252, HA-255 all `<td>` cells; between HA-223 and HA-255 no `</table>`, no
  `<blockquote>`, no `&gt;` row. A deletion only: no number or text changed (the briefing's 218 holds).
  **Not fixed, for the coordinator:** STEP1-A left a second lone `>` at `docs/hardware-assumptions.md`:1927 (just above
  "### STEP1-A's addition"), after a list — it renders an empty blockquote, no ROB1 content near it.
- **RF3 — robot one's TEST B to its top level ends TIMED_OUT under Tune's 10 s (correctness #3). REAL; TUNE1-B1's
  design, NOT changed — recorded.** Reproduced (TIMED_OUT at 887.81° of 1080° after 10 s). Guide 21's Tune
  paragraph now says it: on the placeholder gearing TEST B from the bottom to the last level needs ~9 s of travel
  alone, ends TIMED_OUT short of it HOLDING where it stopped, and a second TEST B finishes the move (the strengthened
  host case below proves exactly that: the second TEST B SUCCEEDED in 3.5 s, the plant at 1080.6°); "with the real
  gearing typed, check it again". **For the coordinator / TUNE1:** whether `kTuneTestMostSeconds` should scale with a
  mechanism's travel.
- **RF4 — step 8's ROBOT HALF had no self-test case (tests #4). REAL; fixed with a test, the design untouched.** The
  integration hand-merged step 8's three calls, so proving the merge kept the third is the integration's to do. New
  **self-test case 58**, `_self_test_robot_half(expect)` (its own function so a probe can run exactly it), called from
  `do_self_test()` after 57: (a) every detector of `robot_separation_problems()` on synthetic ELF contents — eight
  builds over all four robots, each foreign symbol and each foreign word — and two clean controls (each robot's OWN
  registration, beacon and driver path pass); (b) every robot a build names has a non-empty
  `ROBOT_FOREIGN_SYMBOLS` AND `ROBOT_FOREIGN_STRINGS` row; (c) ONE real build through `verdict()`: robot two's
  `src/team/autons.cpp` reaching `registerRobotOneAutons` behind a `static volatile` false → `tank-autontest` FAILS
  step 8 naming `links shulib::bench::registerRobotOneAutons … (step 8, ROB1-A)`, and the plant raises no warning.
  The gate's header lists case 58 (and its summary line now says TUNE1-B1's cases are 42-49, not 42-48 — case 49 is
  TUNE1-B1's review case). Mutations RM1/RM2 below.
- **RF5 — the Tune-over-robot-one host case proved less than its title (tests #5). REAL, fixed (test only; TUNE1's
  design untouched).** `test/tune_program_test.cpp`'s case rewritten: (a) the facts file's cap is asserted IN BOTH
  MOTORS (`fakeMotor(8|9).currentLimit() == kCascadeCapAmps`) after READY, before anything moves, and again after
  HOME — the peak-current check stays but is labelled what it is (the plant reads at most the limit); (b) TEST A is
  no longer a move of nothing: the stick jogs the cascade up first (to 146.9° on the placeholders; a `REQUIRE` that it
  is more than four tolerances above the floor), then TEST A brings it DOWN — SUCCEEDED, the plant at the floor's
  row; (c) TEST B is repeated (at most three TESTs) until SUCCEEDED, each attempt: driven UP, a TIMED_OUT only after
  the WHOLE 10 s and really short of the level, the next one starting where the last HELD (within tolerance);
  then the PLANT — never the program — within half an inch (22.5°) of L3's row, and still there a second later.
  The expected verdict is no longer chosen by the code under test: the cap is checked by arithmetic the program does
  not do (2 × distance / the output's free speed + 0.5 s ≥ 10 s ⇒ `tunePositionBound` = the cap), and every verdict
  is backed by the plant's own position. Run: TEST A from 146.905° SUCCEEDED at 0; TEST B (1) TIMED_OUT after 10 s,
  0 → 887.81°; TEST B (2) SUCCEEDED after 3.5 s, 888.98 → 1080.61°. Mutations HM1/HM2 below.
- **RF6 — the suite's assertion count depends on the checkout (tests #6). Pre-existing, not this integration's; NOT
  root-caused.** The reviewer's evidence holds at `d33d25d` as well (+4 in a clone at a longer path). Not chased
  here: the reviewer's clone binary is gone, and a second full build in another path was not worth this round's
  budget. A lead, unverified: the mechanism declarations record `std::source_location::file_name()` — an ABSOLUTE
  path on the host (`include/shulib/mechanism/spec.hpp`, `kinds.hpp`, `wiring.hpp`) — so a path ~33 characters
  longer can change a cut or wrapped line some test counts per line. CI is not affected (it runs the briefing check
  before it builds: the suite line is exempt with no binary, `tools/briefing_status.py` "THE SUITE LINE IS EXEMPT").
  **For the coordinator:** regenerate the briefing in the commit checkout after a fresh build (rm the binary,
  build, generate), as the memory rule already says; this worktree's count is 3406 cases / 3,784,664 assertions.
- **RF7 — robot one's Tune COPY had no written home (docs #7, major). REAL; documented, and the design decision
  logged for the coordinator.** Re-checked: robot one's region has no `.tuning`; every member reads `facts::…`; T2
  (`test/robot_one_facts_test.cpp`) fails any literal number there; COPY prints whole members
  (`include/shulib/mechanism/live_tuning.hpp:1079-1129`: `.stall = {…}`, `.currentCap = {…}`, `.range = {…}`,
  `.gravity = constantGravity(…_volt)`, `.home = homeByStall(…, …_volt, {…})`). **Guide 21** gains "Tune's COPY on
  robot one: each number goes to its fact" — a table mapping `.gravity` → `kCascadeHoldVolts`, `.home`'s push →
  `kCascadeHomingPushVolts`, `.stall` → `kCascadeStallAmps` / `kCascadeStallRpm` / `kCascadeStallTime`, `.currentCap`
  → `kCascadeCapAmps`, typed by the coordinator with a source like "shulib Tune COPY, <date>"; and what has NO fact
  yet — a `.tuning` block, a narrowed `.range`, a `.home` whose signature differs from `.stall` (robot one's
  declaration uses ONE signature, `facts::cascadeStall()`, for both) — kept from `/usd/tune_log.txt` and given to the
  coordinator. **Guide 18**:590 points robot one's readers there. **THE DECISION OWED (coordinator):** whether robot
  one's `.tuning` (and a narrowed range, and a separate homing signature) become ledger facts, or T2 allows a marked
  `.tuning` block. Until it is made, robot one's first Tune session can tune and type back the hold volts, homing
  push, stall signature and cap — not its gains or profile.
- **RF8 — HANDOFF said robot one has no build variant (docs #9). REAL, fixed.** `docs/internal/HANDOFF-remaining-work.md`
  :124-127 (robot one's line in the robots list) and Q3 (:730-732): robot one has its own six builds since ROB1-A;
  the drive cartridge and IMU port are typed into `kRobotOneChassis` from the form (A2, A4) before Calibrate, which
  refuses an unset cartridge by name; Q3's variant half marked ANSWERED by ROB1-A, the cartridge still owed.
- **RF9 — one-tune needs Stage B, against the task text AND brief §2.0 item 15 (docs #10). Not a defect; kept,
  coordinator's ruling.** One line added under the delta above: it also supersedes item 15's "TUNE1's page if it
  reaches robot one" route for the FIRST hold volts. `grep -rn "TUNE1's page\|if it reaches robot one"` over docs/,
  src/, README.md outside the ROB1 brief and this log: EMPTY (no live doc routes the first hold volts through Tune).
- **RF10 — two status lines said "not merged" (docs #11). REAL, fixed for the tree AFTER the commit (rule 5).**
  REMAINING-WORK row 39 → "ROB1-A LANDED 2026-11-13 (built 2026-11-11; integrated onto `d33d25d`)"; build-order's
  first bold `Next:` → "2026-11-13: ROB1-A LANDED (built 2026-11-11, integrated onto `d33d25d`; …)" — the form
  CURVE1-A's lines use (no hash). **If the commit is not made on 2026-11-13, the coordinator corrects the date.**
- **RF11 — guide 21's "where a robot lives" left out build_selection.hpp's guards (docs #12). REAL, fixed.** Its row
  now adds the robot's define (`SHULIB_ROBOT_ONE_2026`) in each `PROGRAM=` guard above the chain — and, with
  mechanisms, in the Tune guard — "without it that build stops at an `#error` naming why".
- **Found while re-checking, for the coordinator (ROB1-A's reviewed text, not this round's to change):** guide 21:32
  says Calibrate "cannot discover the drive **cartridge** or the **IMU's port**". The cartridge half is true; the IMU
  half is not what Calibrate does — `include/shulib/calibration/calibrate_program.hpp:12` lists "the IMU's port" among
  what is discovered on the robot, its census counts the registry's IMUs (:538-557), and `imuPortStated` is "optional
  cross-check (the registry decides)" (:172). Typing the IMU port before Calibrate is still right for robot one (guide
  21's order step 1: a real IMU on the placeholder mechanism ports fails every build), so only the reason is wrong.
  Also still open from the integration: the "all seven builds" comments (`src/programs/xdrive_program.cpp:35,37`,
  `src/common/fault_latch_hook.hpp:17`, `src/common/boot_source.cpp:12`, `src/programs/auton_tester_program.cpp:62`) —
  TUNE1-B1's stale count, not this merge's lines.

#### The review-fix round's mutations (`scratch/rob1-int/mut-review/mut_review.py`; results `mut-review/results.txt`)

ONE exact replacement each, backup in `mut-review/`, restored in `finally` AND on SIGTERM/SIGHUP/SIGINT, mtime
bumped, the restored text asserted equal to the pre-mutation text. Gate side: a FRESH python imports the worktree's
gate and runs case 58 exactly (`_self_test_robot_half`), its build `run_build()` (bin/ and .d/ deleted — a fresh
package build). Host side: the suite binary DELETED, `cmake --build` (every dependent object recompiled — HM2
rebuilt `tune_panel_test.cpp.o` and `tune_program_test.cpp.o`), the doctest filter `Tune (ROB1-A*` under
`(ulimit -v 4 GiB; timeout 600)`, `ERROR:` and `FATAL ERROR:` both counted; the binary deleted again after the
restore. After every mutation `git diff -- Makefile src/` carries no `(self-test plant`.
- **CONTROL** — case 58 on the real gate: GREEN (0 failures).
- **RM1 — step 8's `problems += robot_separation_problems(robot, linked, elf_bytes)` DELETED (the reviewers' SM1,
  which survived `check` and the whole self-test):** **CAUGHT** — "58: robot one's autons registration linked into
  ROBOT TWO's Auton Tester PASSED" and "58: the failure did not name robot one's autons registration".
- **RM2 — robot two's `ROBOT_FOREIGN_SYMBOLS` row emptied:** **CAUGHT** — "58: build tank-tune with
  ['shulib::bench::registerRobotOneAutons'] … PASSED", "58: robot RobotTwo (build tank) has no ROBOT_FOREIGN_SYMBOLS
  or ROBOT_FOREIGN_STRINGS row" (×6 builds), and the real build's.
- **HM1 — robot one's `.currentCap = {.amps = 2.5}` (the reviewers' HM1, which the old case survived):** **CAUGHT** by
  the case — `tune_program_test.cpp:2164/2165` (both motors, before anything moves) and `:2184/2185` (after HOME).
- **HM2 — Tune's TEST B aimed at the next-to-last preset (`presets[presetCount - 2]`; its NAME left the last's):**
  **CAUGHT** — `:2244` (the plant not at L3 after a SUCCEEDED) and `:2255` (not there a second later). The name check
  alone would not have seen it. *(HM1 and HM2 ran at ~02:49 on the case as it stood then; the case was edited again at
  03:00 — the last level's row found by height, `levelDeg`, not `row[3]` — so both were RE-RUN on the final case at the
  resumption below.)*
- A race of this round's own, said so: one full-suite run taken (for RF6) WHILE the gate-side mutation run had case
  58's plant live in `src/team/autons.cpp` failed `this_robot_test.cpp:405/406` (T7: robot two's team file naming
  robot one) — the host suite catching the plant; that run was discarded, and no suite runs beside a self-test below.

#### The fix round STOPPED and RESUMED (2026-11-13 10:53 EST)

The team lead stopped all work at ~03:40 (98 % of the usage limit) while this round's full gate `self-test` ran
(launched ~02:58; `scratch/rob1-int/selftest_rf.log` empty); resumed after the reset. Main moved to `5f6633b` meanwhile
(STEP1-B API 2.8, VIS1-A2, AF1S-A1) — NOT rebased here: the coordinator rebases at the merge. Scratch for the
resumption: `.claude/coordinator/scratch/rob1-int/resume/`. Not committed; never `pros`, never `/dev/ttyACM*`.

- **The audit first (10:53).** This section's RF1–RF11 read; `git status`; `git diff -- src/ Makefile`.
  - **A LIVE PLANT, found and removed.** `src/main.cpp`'s `opcontrol()` had Bench Tests calling
    `shulib::bench::runDrive()` and Drive calling `runR3a()` — the gate self-test's **case 20** plant (`swapped_to`,
    `tools/src_build_gate.py` ~:3252), written at 03:24:39 by the stopped self-test and never restored: the stop ended
    python inside its `with _replace(...)` block. It carries **no `(self-test plant` marker**, so the marker grep — the
    coordinator's audit and this round's own after-run check — cannot see it. Restored by ONE exact replacement (the
    planted text asserted present exactly once), the as-found file kept as `resume/main.cpp.as-found-case20-plant`,
    mtime bumped; after it `git diff --cached -- src/main.cpp` is empty (the index = `d33d25d`'s dispatch) and `git diff
    -- src/main.cpp` is RF1's comment alone. The other files the stopped self-test touched (`Makefile` 03:18,
    `src/build_selection.hpp` and `src/robots/mechanism_table.hpp` 03:24 — restored by their cases) equal the index /
    their pre-mutation backups; no `build/ROBOT-ONE-GATE-COPY-ONLY`, no `build/src-gate-facts/`.
  - **Every mutation backup compared with its file:** HM1 (`mechanism_table.hpp`), HM2 (`tune_program.hpp`), IM1
    (`Makefile`), IM2/IM4 (`build_selection.hpp`), IM3/IM6 (`robot_one_facts.hpp`) IDENTICAL; RM1/RM2/IM5
    (`src_build_gate.py`) differ only by this round's own later edits (the header's case list at 02:58; IM5's older
    backup also lacks case 58) — step 8's three calls are at :2108-2110 and robot two's `ROBOT_FOREIGN_SYMBOLS` row is
    there: no mutation residue. The reviewers' backups (`review-correctness/*.bak*`) are theirs, not this worktree's —
    the files they name equal the integration's backups above.
  - **A FINDING for the coordinator (the gate's self-test tooling — GATE1 onward — not changed here):** the self-test
    restores a plant ONLY through `with` blocks (`_replace` / `_transform` / `_newfile` `__exit__`), and the gate sets
    no signal handler, so SIGTERM/SIGHUP end python WITHOUT unwinding them: a stopped self-test leaves its live plant.
    And 19 `(self-test plant` markers stand against ~86 plant blocks, so the marker grep is no audit. Two remedies to
    weigh: a SIGTERM/SIGHUP handler that raises `SystemExit` (the `with` blocks then restore), and an audit that
    compares `src/ Makefile common.mk` with the index or a snapshot instead of grepping for the marker. (This
    resumption's own gate run snapshots them first and compares after each half.)
- **The test binary relinked from source:** `rm -f build/test/shulib_tests`, briefing regenerated, `cmake --build
  build/test -j8 --clean-first` (290 objects), briefing regenerated; `suite_run.py green` — **3406 cases, 3784664
  assertions** (10:56).
- **Fixed at the resumption (found by re-reading RF1's own edit):** RESUMING's "Variant selection is" sentence had left
  d33d25d's parenthetical "(`PROGRAM=library`, the library program "shulib Teleop", retired at T4 Part 3)" hanging
  after robot one's facts file, where it read as if that file concerned `PROGRAM=library`; moved back after "the
  competition program", where d33d25d had it, and the sentence's new half re-punctuated (no word of meaning changed).
- **Re-verified — each finding's evidence observed again on the final tree:**
  - **RF1:** the prose grep the reviewer asked for, widened — `grep -rnoiE "\b(five|six|seven|eight|nine|ten|eleven|
    twelve|thirteen|fourteen|[0-9]+)\b[^.;]{0,50}\b(builds?|packages?)\b"` over docs/ (outside docs/api and the chunk
    records), README.md, src/, tools/, test/, .github/, Makefile, common.mk (`resume/prose_counts2.txt`, 60 hits, every
    one READ): TRUE now — guide 21 :39/:42/:116, RESUMING :119, HANDOFF :306/:731, `main.cpp` :50, `build_selection.hpp`
    :203 ("the six tank builds share robot two's" robot beacon), `robot_one_facts.hpp` :24/:66/:70, `chassis_table.hpp`
    :211, `this_robot_test.cpp` :95, ci.yml :150-152, the gate :2136-2146, the changelog :19/:60; HISTORY, dated and
    true as dated — the roadmap's and build-order's chunk paragraphs, REMAINING-WORK's landed rows, PROJECT-BRIEFING
    :110 (COMP-t's "seventh build"), HANDOFF :55 (its 2026-09-24 gate list) and :667, the gate :163/:366-367/:460,
    the HA register :3747, ORIENTATION :84 (its dated table: 193 headers, 1,808 tests); STALE and not this merge's
    (already listed above): the "all seven builds" comments in `boot_source.cpp` :12, `auton_tester_program.cpp` :62,
    `fault_latch_hook.hpp` :17, `xdrive_program.cpp` :35/:37.
  - **RF2:** the register re-rendered with Python-Markdown 3.5.2 (`tables`): ONE table from :285 to :504; HA-216,
    HA-223, HA-246, HA-250, HA-252..HA-255 each a `<td>`; no `<blockquote>`, no `&gt;` row. The file's other lone `>`
    lines (:9, :27, :38, :43, :218, :227, :234, :244) sit inside blockquotes; :1927 is the one listed above.
  - **RF3 / RF5 (the case, alone, on the fresh binary):** "TEST A from 146.905 deg" → SUCCEEDED at 0; "TEST B (1):
    TIMED_OUT after 10 s … from 0 to 887.81 deg of 1080"; "TEST B (2): SUCCEEDED after 3.5 s … from 888.978 to 1080.61";
    1 case, 3060 assertions, 0 failed.
  - **RF5's mutations RE-RUN** (the case was edited at 03:00:58, AFTER HM1/HM2 ran — see the note above; the same
    driver, `resume/mut/mut_resume.py`, backups in `resume/mut/`; each on a FRESH binary — deleted, `cmake --build`
    recompiling every object that includes the mutated header — restored = backup byte for byte, mtime bumped, the
    binary deleted after): **HM1 CAUGHT** — `tune_program_test.cpp:2164/:2165` (the cap in both motors before anything
    moves) and `:2184/:2185` (after HOME); **HM2 CAUGHT** — `:2258` (`std::abs(endDeg - levelDeg) <= tolDeg` after the
    SUCCEEDED) and `:2269` (not there a second later). `(self-test plant` lines after each: 0.
  - **RF4:** RM1/RM2 stand as observed — `_self_test_robot_half` has not changed since they ran (the gate differs from
    their backups only in the header docstring); case 58 inside the FULL self-test below.
  - **RF6–RF11:** re-read on the tree. RF7's table: each fact it names is in the ledger (`kCascadeHoldVolts`,
    `kCascadeHomingPushVolts`, `kCascadeStallAmps` / `…Rpm` / `…Time`, `kCascadeCapAmps`); robot one's declaration has
    no `.tuning`, `.range = {facts::cascadeBottom(), facts::cascadeTop()}`, and `.stall` and `.home` both use
    `facts::cascadeStall()` — so "the library's starting gains and profile, and the range from the bottom row to the
    top row" holds; guide 18's anchor `#the-tune-program--tuning-on-the-brain-screen` is its "### The Tune program —
    tuning on the brain screen", and guide 18 says the 10 s cap (:566). RF8/RF10/RF11 as written above.
- **Every gate on the final tree (10:53–),** `resume/`:
  - the suite as above; then, after RF5's re-run mutations (which delete the binary): `rm -f`, briefing regenerated,
    `cmake --build`, briefing regenerated, `suite_run.py green` — **3406 cases, 3784664 assertions**
    (`resume/suite2.log`); `briefing_status.py check` current, run right after it;
  - `ctest -R compile_refusal` **267 of 267**, TUs 267 = CTests 267; the ARM header gate CLEAN (249 headers, 250
    lines, `-Werror`); the PROS-free and sim-layering greps empty; `api_doc_tool` self-test OK, check-coverage (6062
    entities, 206 headers), check-fresh, check-examples (776 lines, 16 files), check-removability — all exit 0;
    `briefing_status.py` self-test OK (38 cases + `suite_run.py`'s 41), check current; `doc_staleness_audit.py`
    self-test OK, audit clean (`resume/gates.log`, 10:56–11:00);
  - **gate `check` PASS** (11:12–11:20, `resume/gate_check.log`): "all 14 builds link or, robot one's, refuse naming
    exactly their owed facts (6 refused; every one links in the gate's typed copy)"; `[chain-table] 14 builds, 4
    robots`; `[tune-row]` clean; `one` / `one-calibrate` refuse exactly their 10, `one-drive` / `one-autontest` /
    `one-competition` / `one-tune` exactly their 15; `[facts-copy] 17 owed facts typed in build/src-gate-facts/ only;
    robot one's 6 builds link and pass every check`; `src/ Makefile common.mk` equal to the snapshot after it;
  - **gate `self-test` IN FULL** (11:20–11:58, `resume/gate_selftest.log`, nothing else running in the worktree):
    `src build gate self-test: OK (376 detector cases, each against a real build)`, EXIT 0 — 349 before case 58,
    plus case 58's 27 (8 synthetic detectors, 2 clean controls, 14 builds' `ROBOT_FOREIGN_*` rows, 3 on the real
    `tank-autontest` plant); no `SELF-TEST FAILURE` line. After it: `src/ Makefile common.mk` EQUAL to the snapshot
    taken before `check` (`diff -rq` / `cmp` — not only the marker grep), `(self-test plant` lines 0, no
    `build/ROBOT-ONE-GATE-COPY-ONLY`, no `build/src-gate-facts/`.
  - **[R5] / T6 — robot two's (and bench's, xdrive's) EIGHT packages:** `t6/change2-*` were built at 02:56 from a copy
    whose `src/ include/ firmware/ Makefile common.mk project.pros` are byte-identical to the final tree (`diff -rq` /
    `cmp` at the resumption: the plant above was written at 03:24, after the copy), compared with `d33d25d`'s
    `t6/base-*` by ROB1-A's own tools (`resume/t6_compare.sh` → `resume/t6_compare.txt`): `.bin` sizes identical for
    all eight; per-function disassembly modulo addresses **0 differing** in all eight; `.data` masked 0 bytes; the
    symbolic `.rodata` compare names ONLY `_PROS_COMPILE_TIMESTAMP`, `_PROS_COMPILE_TIMESTAMP_INT` and (Drive,
    Competition) `kBuildTimeText`, every anonymous-run difference a build time; `strings -a` of each `.bin`, timestamps
    masked and SORTED, **0 differing lines** in all eight (unsorted, only ROB1-A's logged COMDAT reorder); no robot-one
    string in any of them.
  - **[R1] the stamp, by hand in that copy** (`resume/r1_stamp.txt`): `make ROBOT=tank` (13 compiled) → `make
    ROBOT=tank PROGRAM=tune` with no rm prints `shulib: the build changed (ROBOT=tank PROGRAM=tune) -- removing bin/
    and .d/ so every file recompiles`, compiles 13, its package carries `shulib-robot-variant=tank-tune`; the same
    again: no stamp line, 0 compiled; `make ROBOT=one PROGRAM=tune` prints the stamp line, exits 2 naming exactly 15
    `robot one: OWED --` facts (the 10 Stage A + the 5 Stage B), and `bin/hot.package.bin` is gone.
  - **[R4] the typed-copy rehearsal:** the gate's `[facts-copy]` above; the repository's ledger after every gate run:
    17 `owed(` declarations, 0 `typed(` declarations.
  - **The brief's §7 greps:** `git diff --stat d33d25d -- include/shulib` empty; robot two's team files
    (`robot_two_driver.hpp`, `autons.hpp`, `autons.cpp`) unchanged from `d33d25d`; robot two's region identical to
    `d33d25d`'s line for line AND at the same line numbers (its end marker on :135); `consteval` in the ledger: 0;
    `ROBOT_ONE_CHECK(`: 17; `G1-PROGRESS.md` unchanged.
- **Last, after the self-test (11:59–12:03):** its restores bump the mtimes of the files it planted, so the staleness
  audit then distrusted the binary ("older than src/team/autons.cpp and 11 more") — the source equal to the snapshot,
  only the times newer. So once more: `rm -f build/test/shulib_tests`, briefing regenerated, `cmake --build` (290
  objects), briefing regenerated, `suite_run.py green` — **3406 cases, 3784664 assertions**; `briefing_status.py check`
  current; `doc_staleness_audit.py` clean with the binary trusted; `api_doc_tool` check-fresh and check-removability
  exit 0 after this log's edits (`resume/docgates_final.log`). **The review-fix round is COMPLETE** — every finding
  dispositioned above, every gate green on the final tree. Not committed.
- **For the coordinator, from the resumption:** (1) the gate self-test's restore-on-signal gap above; (2) main is at
  API 2.8 now (STEP1-B) — ROB1-A's changelog entry was placed at the top of `## API 2.7` per the task (no version bump:
  src/ only); at the rebase it belongs at the top of whatever section is current then; (3) RF10's "LANDED 2026-11-13"
  needs the commit's real date if it is not today; (4) RF6 stands — regenerate the briefing in the commit checkout
  after a fresh build (rm the binary, build, generate).

#### Verification-round findings and their disposition (2026-11-13 ~12:20 EST onward)

A fresh verifier returned two findings, both minor, against the review-fix round above. Each was re-checked against the
tree before anything was changed. Scratch: `.claude/coordinator/scratch/rob1-int/verify-fix/` (`mut_vf.py` and its
`results.txt`; `gates.sh` / `gates.log`; `srcgate.sh` / `srcgate.log` / `gate_check.log` / `gate_selftest.log`;
`suite*.log`; `prose_grep.txt`). Not committed. Never `pros`, never `/dev/ttyACM*`.

- **VF1 — case 58 proved only the SYMBOL half of step 8's robot-half call; passing `b""` for the ELF's bytes at the
  call site survived. REAL, fixed (the test only; the gate's checks untouched).** Re-checked: the call
  `problems += robot_separation_problems(robot, linked, elf_bytes)` (`tools/src_build_gate.py`:2110, :2112 after this
  round's two header lines) has two inputs;
  case 58's part (c) expected only the `links shulib::bench::registerRobotOneAutons` problem (the `linked` half), and
  part (a) calls the function directly, never through the call site — so RF4's "the call site is proven" (the gate's
  header) held for `linked` alone, and an ELF-words check disconnected at the call site turned nothing red. The plant
  DOES put the words in the package: the planted `tank-autontest` ELF the last round's self-test left in `bin/` (11:58)
  holds `src/team/robot_one_autons.cpp` in `.debug_str` (its compile unit's DWARF name — `-g` is in common.mk's
  MFLAGS), and the file's name in `.debug_line` and `.strtab`. **Fix:** part (c) also expects a problem carrying
  `carries 'src/team/robot_one_autons.cpp'`, `another robot's words` and `(step 8, ROB1-A)`; (c)'s comment and the
  header's case-58 sentence now say the case proves BOTH of the call's inputs, and since when. **Mutations**
  (`verify-fix/mut_vf.py`: ONE exact replacement in the gate, backup in `verify-fix/`; case 58 EXACTLY
  (`_self_test_robot_half`) in a FRESH python under `(ulimit -v 4 GiB; timeout 1500)`, whose SIGTERM/SIGHUP/SIGINT
  raise `SystemExit` so the gate's own `with` plant of `src/team/autons.cpp` unwinds; the gate restored in `finally`
  AND on those signals, its mtime bumped, the restored text asserted equal; each build `run_build()` — `bin/` and
  `.d/` deleted, a fresh package build; after each, `src/ Makefile common.mk` compared with a snapshot taken before
  the first, and the marker grep):
  - **CONTROL** (the fixed case on the real gate): GREEN, 28 checks (RF4's 27 + this one).
  - **RM3 — the call site's `elf_bytes` → `b""` (the verifier's survivor): CAUGHT** — "58: the failure did not name
    robot one's autons FILE PATH (step 8's robot half, the ELF's words — the call site's elf_bytes)", the problems
    list holding only the symbol line.
  - **RM1 — the call DELETED (re-run on the new case): CAUGHT** — 3 failures: PASSED; no registration named; no file
    path named.
  - **RM4 (this round's own) — robot two's `ROBOT_FOREIGN_STRINGS` row loses `src/team/robot_one_autons.cpp`** (the row
    kept non-empty, so (b) passes, and no (a) row for robot two names that word): **CAUGHT** — by the new expect
    alone. So it pins that row entry as well as the call site's input.
  - After each: `src/ Makefile common.mk` EQUAL to the snapshot; `(self-test plant` lines 0; the gate's call line and
    robot two's row each present exactly once again.
- **VF2 — guide 21's COPY table sends a `.stall` into the three stall facts without saying they are HOME's signature
  too. REAL, fixed (docs only).** Re-checked: `src/robots/mechanism_table.hpp`:181 and :185-186 — `.stall =
  facts::cascadeStall()` and `.home = homeByStall(…, kCascadeHomingPushVolts, facts::cascadeStall())`;
  `include/shulib/mechanism/live_tuning.hpp`:411-424 — `StallAmps` / `StallLasting` (`st->amps`, `st->time`) and
  `HomeStallAmps` / `HomeStallLasting` (`s.position.homeStall.…`) are separate tunables; :1121 — COPY prints `.home`
  only when `homeVolts` or `homeStall` changed. So a session that tuned only the stall signature prints `.stall` and
  no `.home`, and typing it moves HOME's signature to one HOME never ran at — and the exception "a `.home` whose stall
  signature differs from `.stall`" cannot cover it (there is no `.home` line). **Guide 21** gains a paragraph under
  the table: the three stall facts are HOME's signature too; after typing a `.stall` changed on its own, jog the
  cascade partway up in Tune and run HOME once, relying on it only when it ends SUCCEEDED with the cascade resting on
  its bottom stop — an `.amps` above what the push draws at the stop ends TIMED_OUT, never homed, and a stall-home
  trusts the first stall (linked to chapter 18's homing section, which says both); tuned apart, both lines go to the
  coordinator. The link's anchor follows the site's slugify (`pymdownx.slugs.slugify`, `case: lower`: each space a
  hyphen, the em dash dropped → `#homing--every-moment-and-which-one-to-choose`, the form the existing
  `#the-tune-program--tuning-on-the-brain-screen` links use).
  - **For the coordinator (ROB1-A's design, not changed here):** robot one's ONE signature has to meet the rules
    chapter 18 gives for TWO different signatures — the jam recipe (the heaviest hold < the signature < the cap −
    0.2 A; the form's C6 (i) asks for it that way) AND a homing signature below what the homing push draws at its stop
    (chapter 18: "A homing push is gentle, so its stall is SMALL — `homeByStall`'s own signature is not the jam
    recipe"; MECH-PROGRESS A3-D23 is the same class). The form's C6 (ii) measures at a 3 V press on the bottom stop,
    and the facts file's placeholder push is −6 V for exactly this reason (`robot_one_facts.hpp`:434-438). On the
    real robot that window can be empty; RF7's owed decision (a separate homing signature as a fact) is where it
    closes.
- **Prose review (the greps run and every hit read BEFORE this line; `verify-fix/prose_grep.txt`, 61 lines):**
  (1) `case 58|robot half|_self_test_robot_half|ROBOT_FOREIGN_STRINGS|robot_separation_problems` over docs/ (outside
  docs/api and this log), README.md and .github: no live doc describes step 8's robot half — the gate's header is its
  doc (updated above); the hits are build-order :936 ("robot half-built", another subject) and G1's records of step 6's
  robot half (history). (2) `homing signature|HOME's signature|cascadeStall|kCascadeStall|Tune's COPY|COPY on robot
  one` over docs/, README.md, src/robots and src/team: guide 11 :333-336 and guide 18 :208-218 (the TIMED_OUT line and
  the sizing rule — what the new paragraph says, agreeing); guide 18 :446 and :609 (Tune's live members; the cap
  refusal — agreeing); the changelog, HA register and roadmap hits and the MECH / TUNE1 / MECH-S-TUNE1 records
  (history, agreeing); `robot_one_facts.hpp` :509-513 ("the declaration's `.stall` AND its homing's" — agreeing);
  `docs/robot-facts-form.md` C6 — the note for the coordinator above, not changed.
- **Every gate on the final tree (12:31–13:27, `verify-fix/`):**
  - the suite: `rm -f build/test/shulib_tests`, briefing regenerated, `cmake -S test -B build/test`, `cmake --build`,
    briefing regenerated, `suite_run.py green` — **3406 cases, 3784664 assertions** (`suite.log`; this round changed no
    source the suite compiles: `src/ Makefile common.mk` equal to the resumption's snapshot `resume/snap/`, `git diff
    --stat d33d25d -- include/` empty);
  - `gates.sh` (`gates.log`, 12:32–12:36): the PROS-free and sim-layering greps empty; the ARM header gate CLEAN (249
    headers, 250 lines, `-Werror`); `ctest -R compile_refusal` **267 of 267**, TUs 267 = CTests 267; `api_doc_tool`
    self-test OK, check-coverage (6062 entities, 206 headers), check-fresh, check-examples (776 lines, 16 files),
    check-removability — all exit 0 (no `///` comment changed, so no `generate`); `briefing_status.py` self-test OK
    (38 + `suite_run.py`'s 41), check current; `doc_staleness_audit.py` self-test OK, audit clean;
  - **gate `check` PASS** (`gate_check.log`, 12:36–12:46): all 14 builds — robot two's, bench's and xdrive's 8 link,
    robot one's 6 refuse exactly their owed facts (`one` / `one-calibrate` 10, `one-drive` / `one-autontest` /
    `one-competition` / `one-tune` 15) and link in the gate's typed copy; `[chain-table] 14 builds, 4 robots`;
    `[tune-row]` clean; `[facts-copy] 17 owed facts typed in build/src-gate-facts/ only`;
  - **gate `self-test` IN FULL** (`gate_selftest.log`, 12:46–13:20, nothing else running in the worktree): `src build
    gate self-test: OK (377 detector cases, each against a real build)` — the resumption's 376 + VF1's one; no
    `SELF-TEST FAILURE` line;
  - after `check` and after the self-test: `src/ Makefile common.mk` EQUAL to the snapshot taken before them
    (`diff -rq` / `cmp`, and that snapshot equal to the one taken before VF1's mutations), `(self-test plant` lines 0,
    no `build/ROBOT-ONE-GATE-COPY-ONLY`, no `build/src-gate-facts/`; the self-test's restores bumped mtimes in
    `src/`, `include/` and `test/` with no content change (`git diff -- include/ test/CMakeLists.txt
    test/vendor/doctest.h …` empty);
  - so once more after it: `rm -f`, briefing regenerated, `cmake --build` (290 objects), briefing regenerated,
    `suite_run.py green` — **3406 cases, 3784664 assertions** (`suite2.log`); `gates.sh` again (`gates2.log`): every
    line as above, the staleness audit clean with the binary trusted;
  - **[R5] / T6, [R1], [R4] stand:** this round changed no file a package is built from (`src/ Makefile common.mk`
    byte-identical to the resumption's snapshot, against which T6's eight-package compare and R1's stamp were
    evidenced; `include/` equal to `d33d25d`), and R4's typed-copy rehearsal is `check`'s `[facts-copy]` above; the
    repository's ledger after every gate run: 17 `owed(` declarations, 0 `typed(`.
- **The verification round is COMPLETE** — both findings real and fixed (VF1 by a test, VF2 in guide 21), one note for
  the coordinator (VF2's one-signature window); every gate green on the final tree. Not committed.

## ROB1-A — the coordinator's merge verification *(2026-11-13)*

- **Squash-merged from `rob1-int`** (ROB1-A integrated onto `d33d25d`, with robot one's Tune) onto `a27e2bc` (STEP1-B,
  VIS1-A2, AF1S-A1 and the CI pin landed since). Code applied cleanly; `.github/workflows/ci.yml` merged cleanly WITH
  the `ubuntu-24.04` pin. Documents resolved by hand: the guide index and `mkdocs.yml`'s nav (chapter 20 the camera,
  chapter 21 adding your robot by hand), the roadmap's "Now", build-order's "Next" (grafted), the register (7 of
  **227** settled; ROB1's HA-252..255 added, HA-256/257 held; the index sorted), the changelog (robot one's entry at
  the top of `## API 2.8`; no library change, no bump). This log declared **between parts** (ROB1-B, ROB1-C and the
  twin queued).
- **Gates on the merged tree** (`.claude/coordinator/scratch/rob1_a_merge_verify.sh`, log
  `.claude/coordinator/logs/rob1a_merge_verify.out`, 15:56–16:57): the suite green (**3,661 cases, 3,862,526
  assertions**); `src_build_gate.py` self-test OK (377 detector cases) and check **PASS — all 14 builds** (robot two's,
  bench's and xdrive's 8 link; robot one's 6 refuse naming exactly their owed facts and link in the gate's typed
  copy); the ARM header gate exit 0; `ctest` **289 of 289**; the API reference fresh; the briefing current; the
  staleness audit clean; the CI and removability greps empty; no plant, no stray root file.
- **The coordinator's own mutations** (`muts_rob1_a_main.json`, log `.claude/coordinator/logs/muts_rob1_a_main.out`):
  **3 of 3 caught** — MR1A robot one dropped from the Makefile's `MECHANISM_ROBOTS` (RED: `build_selection_test.cpp`
  :155, `listedRobots == withMechanisms`); MR1B robot one's Tune moved to the Stage-A-only group (caught AT COMPILE TIME
  by the test's own assertion — `test/robot_one_facts_test.cpp` :193, `static_assert(needs(Program::Tune, Stage::B), "shulib
  Tune needs Stage B: …")`; the harness reports it NO-LINK, and the coordinator rebuilt the mutant by hand to confirm the
  failure is that assertion and nothing else); MR1C the cascade's placeholder homing push weakened to −2 V (RED, 2
  cases). The restored tree relinked and ran green (3,661 cases).
- **One merge slip found and fixed before the commit:** the register's Held line had lost HA-257 (ROB1's, held for
  G1-B — the integration branch had it); restored, and the briefing regenerated (next free **HA-258**).
- **ROB1-A LANDED.** Nothing has run on robot one; every placeholder number is OWED until the builder's facts are
  typed. Next: ROB1-BT (the record names its robot + robot two as the twin, with the route maker's shared route file).
  Known follow-up (the RMP1 brief's re-check R1): guide 21 and `robot_one_autons.cpp`'s banner must say to wait for the
  cascade's home to FINISH before its first move — a coordinator commit right after this one.
- **The homing follow-up (the coordinator, 2026-11-13 late):** guide 21 and `robot_one_autons.cpp`'s banner now say to
  WAIT for the cascade's home to finish (`r.waitUntilDone(cascade, 2_s)`, or `r.home(cascade, 3_s)`, which waits) before
  its first move — a move or a trigger's move sent while it is still homing REPLACES the home and stops NOT_HOMED (the
  RMP1 brief's re-check R1; its probes `p6_one-starthome-trigger` (stopped at step 2, NOT_HOMED) and
  `p6_one-starthome-wait` (homed, then reached its level)). Gates (`scratch/rob1a_homing_verify.sh`): suite green (3,661
  cases), src gate check PASS (14 builds), check-examples / check-fresh / check-coverage, staleness audit and briefing
  check all exit 0; no plant.

## ROB1-C2 — robot one's real mechanisms and its first typed facts *(2026-11-15, the coordinator's builder)*

> Started 2026-11-15 ~19:40 in the worktree `rob1-c2` (branch `rob1-c2`, from `9befdb3`), on the team lead's go
> ("we need to have the bot coded tonight"). The builder's answers (2026-11-15 ~18:40-19:25) are the coordinator's
> relay in the run's task. The between-parts marker above is removed while this run builds (the coordinator
> restores it). Host- and build-gated only: **nothing has run on robot one.**

- Read: RESUMING's standing rules; the brief's header, D1-D13, §6, §8, the re-check table; this log's tail; guide 21;
  the facts form; the ledger, the table's robot one region, the driver file and their five tests.
- **A library gap found first (the builder's map needs it):** `whileHeld` took TWO buttons only; "L1 will be the
  right side and when I hold it will run" is ONE button, one way, with the d-pad left free. Added the one-button
  hold to `include/shulib/mechanism/driver_inputs.hpp` (`whileHeld(const Power&, button, volts)`, `Control::oneWay`;
  `controls()` takes its one button only; `tickHold` never reads a back button for it; `bind()` accepts it on a
  power mechanism only; a 3-argument call on an Arm / Lift / Pneumatic / Flywheel is refused by name). The panel's
  boot line says `hold-to-run L1 (one way), 12 V`. API reference regenerated. **Robot two's packages change by
  this** (the library's hold tick and the panel's line are compiled into them; its two-button holds never take
  the new branch) — the coordinator's T6-style compare would see it.
- **A second gap: the Drive screen's mechanism row.** Six mechanisms need 98 characters in the full row; the build
  asserts robot one's fits 50 (`driver_panel.hpp`). Added a COMPACT row used only by robot words that are not the
  claw words AND only when the full row cannot fit (`compactRowText`, `compactRowWorstLength`,
  `mechanismRowWorstLength`): `MECH 10H34 9B31 2B30 CLAW:C FLIP:O RAMP:U` (47 worst). Robot two's row code is
  untouched; the build's own static_assert reads the row its words print.
- **The ledger (32 facts; was 17):** typed from the builder (source "the team's builder, 2026-11-15, read off the
  robot"): B6 {10, 1, 9, 2}; the cascade's ports {10, 1}, directions {Forward, Reversed} ("same as the placeholder,
  checked"), BLUE, teeth 1:1 (direct); the flipper's kind Pneumatic; the side toggles' ports {9, 2} and GREEN; the
  claw 'A', the front toggle 'B', the ramp 'C'; the cascade's levels NONE ("for now it will be manual"). Chosen,
  not measured: the coordinator's cascade cap 2.0 A and signature 1.5 A / 5 rpm / 0.3 s; this run's side-toggle
  cap 1.5 A and each piston's stroke time 0.5 s. Still OWED: the right side toggle's port (A); the cascade's rows
  (now TWO: bottom and top), hold volts, homing push (placeholder moved -6 V -> -8 V: the host plant's stall
  current at -6 V is 1.25 A, under the typed 1.5 A signature), unhomed volts; each side toggle's direction; each
  piston's opening level (B); the camera (Recorded). New helpers: `leftTogglePort()` (the other side-toggle port,
  0 = refused by the wiring), `unpoweredPosition()` (each piston's start/safe = its Low position).
- **The drive table:** `kRobotOneChassis.cartridge = Blue`, provenance the builder's. The IMU is NOT typed:
  Calibrate DISCOVERS it from the registry (`calibrate_program.hpp` :538-550; a typed port is only the
  `imuPortStated` cross-check) — ROB1-A's guide 21, the facts form and two tests said it had to be typed; fixed
  there (a flaw in an earlier chunk, fixed in place).
- **The driver file:** the builder's map exactly (R1/R2 cascade, A claw, B front toggle, X ramp, L1 right toggle,
  L2 left toggle, Y re-zero, d-pad free), the side toggles at 12 V (chosen; the cap bounds a held stop), the
  driver-start home kept as the ONE `[delete-able]` line. Drive's boot line no longer says "a DEFAULT".
- **Levels rule (the brief's "four to six"):** it existed only because the default driver file bound four
  presets. Now the levels may be EMPTY; the rule kept wherever a preset is bound: `cascade[facts::lv(i).name]` on
  an unnamed slot is refused by name (`test/compile_refusals/s83`).
- **Tests changed/added:** `robot_one_facts_test` (counts 32/17/13/2, the structural words, same() for char/bool/
  Motor, the ROB1-C2 typing: every builder fact typed, every typed source "the team's builder, 2026-11-15" or
  "chosen, not measured", Stage A owes only the right toggle port; the typed values); `robot_one_tables_test`
  (six mechanisms, 1:1, the chassis' typed cartridge and discovered IMU, T8 moves to 1/4 and 3/4 of the range,
  T9 four declared motors, a new case for the side toggles and pistons); `robot_one_driver_test` (the builder's
  map re-pinned, the banner's `[the map]`, the host run with L1/L2 one way and the motors' signed ports, a pistons
  case: every line MADE at Low and A/B/X each flip only theirs, the compact row, the one-way boot line);
  `robot_one_flipper_forms_example_test` (forms keep, robot one's real flipper = the pneumatic form, AF1S's pair
  check takes `flipper["open"]` / `flipper["closed"]`); `tune_program_test` (robot one's six-mechanism table;
  targets by TuneTargets' rule; only declared ports and lines asked); `mechanism_driver_loop_test` (the one-button
  hold); `boot_source_test` T10 and `mechanism_table_test`'s claw-level case (re-scoped: robot one's real ports
  1 and 2, the second scan read the banner's marker mention); refusal TUs `d_one_button_hold_on_an_arm`,
  `d_one_button_hold_button_twice`, `s83_robot_one_preset_on_an_unnamed_level`; `s79` moved from port 10 (now the
  cascade's) to 11.
- **Mutations** (in a copy of this tree, `.claude/coordinator/scratch/rob1-c2/mtree`, driver `mutate.py`, log
  `mutations.log`; each a fresh `shulib_tests` link, restored in `finally`/on signals, timestamp bumped): MC1 tickHold
  reads the back button for a one-way hold -> RED (held/heldAgain/reconnected); MC2 the one-way boot line removed
  -> RED; MC3 compact row never chosen -> RED (9 assertions); MC4 compact letter always H -> RED; MC5 right/left
  toggle wiring swapped -> RED; MC6 the claw starts in its powered position -> RED (line made High); MC7 B6 drops
  port 2 -> RED; MC8 a typed source not the builder's or "chosen" -> RED; MC9/MC10 (first cut broke the build at
  the doc gate's verbatim scan — re-cut with guide 21's listing changed with it) claw/flipper buttons swapped ->
  RED, R1/R2 swapped -> RED; MC11 the chassis cartridge Green -> RED; MC12 (first cut `return rest;` broke the build
  on unused parameters — re-cut) unpoweredPosition inverted -> caught AT COMPILE TIME by the test's own
  static_asserts (`robot_one_tables_test.cpp` :415-416; the build stops at the first failing translation unit — no
  `-k` — so whether any OTHER test would also have caught it is not shown: corrected by the review round); MC13 the right toggle's cap removed
  -> RED; MC14 a level typed -> the s83 refusal harness RED. Equivalent, not run: `takes()`'s `!c.oneWay` guard
  (a one-way control's `back` repeats its button, so the guard cannot change an answer).
- **Gates on the final tree** (logs in `.claude/coordinator/scratch/rob1-c2/`): host build exit 0 (doc gates inside);
  `suite_run.py green` — **green, 3,708 cases, 3,862,183 assertions** (`suite-final.log`); the compile-refusal harness
  **292 of 292** (`ctest-final2.log`; 289 + the three new TUs; `s79` re-pointed first — it was the one red);
  `src_build_gate.py check` **PASS — all 14 builds** (`srcgate2.log`): robot two's, bench's and xdrive's 8 link;
  robot one's Bench Tests and Calibrate refuse naming exactly ONE fact ("the right side toggle's port"); its Drive,
  Auton Tester, Competition and Tune refuse naming exactly TEN (that one; the cascade's rows, hold volts, homing
  push, volts before it is homed; the right and left side toggles' run directions; the claw's, the front toggle's
  and the ramp's opening level); all six link in the gate's typed copy (12 owed facts typed there only);
  `self-test` OK (377 detector cases; `gate-self2.log` — a first run under a 20-minute `timeout` was killed and left
  no plant); the ARM header gate over all 255 headers exit 0; the CI greps and the removability grep empty;
  api_doc_tool check-fresh / check-examples (860 lines verbatim) / self-test / check-coverage (6,390 entities) /
  check-removability exit 0; the staleness audit clean; `briefing_status.py check` current; `git diff -- Makefile
  src/` holds no self-test plant. Not committed.
- **Prose review (grepped, hits read):** `whileHeld` across docs/ (guide 10, 14, 18, 21, cookbook 06/07 — guide 18
  gained the one-button bullet; the rest list the two-button form correctly); "robot one" / `robot_one` / `ROBOT=one`
  across docs/ and `src/team/README.md` (the README's DEFAULT-map paragraph and "OWED until typed" rewritten; the
  roadmap's ROB1 "Now" paragraph — its own lines — no longer says "a default"; guide 07, 16, 18 and guide README
  still true); "four to six" / "IMU's port" / "12:60" / "8 and 9" / "fewer than four" / "flipper's kind" (guide 21
  and the facts form fixed; guide 18 :848's 12:60 is its own example; guide 19's IMU row already says discovered);
  "claw row" / `MECH` / `kRobotOneChassis` / `imuPort` / "facts file" (no stale hit). Not touched, with why:
  `build-order.md` / the roadmap's "You are here" (the coordinator's status pointers at the merge); guide 20's
  flipper words (generic — "the flipper's kind" is a Step 0 number there, still true).


## ROB1-C2 review findings and their disposition *(2026-11-15 ~21:00-22:30, the coordinator's builder)*

> Three reviewers (correctness, tests, docs) returned 26 findings; deduplicated to 22 below. Each was re-checked
> against the tree before it was fixed or answered. Mutation driver `scratch/rob1-c2/mutate_r.py` (the run's
> `mutate.py`, its own log `mutations-r.log`), in `mtree/` re-synced from the worktree first (`diff -rq` of
> `src/ include/ test/` empty before and after); each a fresh `shulib_tests` link, restored in `finally` / on
> signals, timestamp bumped. Host- and build-gated only: **nothing has run on robot one.** Not committed.

**1. The homing push can never reach the 1.5 A signature (correctness major + tests major + docs major — one
finding).** REAL, re-derived: `homeByStall(…, facts::cascadeStall())` shared the jam signature; on the host test
plant (HA-183 / HA-93's invented shape, 2.5 A × |V| / 12 at stall) a push must exceed 7.2 V to draw 1.5 A, so the
form's gentle −3 V could never end a home (the review's probe B: TIMED_OUT, 9 assertions). The same class was
ruled on in MECH (MECH-PROGRESS :2194, chapter 18 :204-212: "a homing signature is not the jam recipe"). **Fixed at
the root:** a separate fact, `kCascadeHomingStallAmps` (Stage B; typed 0.45 A, "chosen, not measured -- the
coordinator's ROB1-C2 run, 2026-11-15", chapter 18's starting value for a gentle push), read through a new
`cascadeHomingStall()` (its own amps, the jam signature's rpm and time) by the cascade's `homeByStall`. The homing
push's placeholder goes back to ROB1-A's −6 V (the −8 V move existed only to reach 1.5 A on the plant). The
coupling itself is ROB1-A's — its −6 V placeholder already worked around a shared 1.0 A placeholder signature that
the form's −3 V could not reach either — so it is fixed where it was made. The push's comment now states the rule (the push must draw more than the homing signature: past 2.2 V on the plant). The plant is NOT
changed (the tests lens's alternative): it models a real motor's current, and the declaration — not the plant — was
wrong. Docs: the facts form's C8 (ii) (the constraint beside the −3 V example), guide 21 (the Tune COPY table gains
the homing amps' row; the "three stall facts are HOME's signature too" paragraph rewritten; "No fact holds these
yet" narrowed to a homing rpm / time that differs; the Stage B row; the cascade's row), the ledger's Stage B banner,
the cascade's banner, the worksheet. Tests: D5 and T8 check the declaration's homing amps are the new fact's and T8
that the HOMING signature was reached; the facts test pins 0.45 A, homing < jam, rpm / time shared. **For the
coordinator: 0.45 A is a value this run chose** (the task gave the jam pair only) — confirm or replace it.

**2. The facts test pinned tonight's two source wordings on EVERY typed fact (correctness major + tests major).**
REAL: the review's probe C (rows typed "station 13 RECORD and a tape, the team's builder, 2026-11-16") and PR2
(`shulib Tune COPY, 2026-11-17` — guide 21's own example) both went red. **Fixed:** the two-prefix rule applies to
the test's own `given` set (the facts typed on 2026-11-15) only; every other typed fact needs a date
(`20\d\d-\d\d-\d\d`) and never "guess". Guide 21 says so. The ledger's own rules (`realSource`, "gate copy") are
unchanged.

**3. Robot two's packages change; T6 not run (correctness minor + docs major).** REAL. **T6 RUN** (below). The
roadmap's "Robot two is unchanged." → "Robot two behaves as before."

**4. Two interpretations recorded as the builder's words; "ROB1-C2's builder" reads as the team's (correctness
minor).** REAL. `kFlipperKind`'s source → "chosen, not measured -- the coordinator, 2026-11-15, from the builder's
answer 'B will be the Forward Toggle pistons': the front toggle … is the autons' flipper"; `kCascadeDirections`'
source → "… the two motors FACE EACH OTHER; port 10 taken as Forward and port 1 as Reversed by the coordinator,
station 13 confirms …"; every "ROB1-C2's builder" → "the coordinator's ROB1-C2 run" (the side-toggle cap, the three
stroke times, the driver file). Also the `FlipperKind` doc comment, the table's banner, the flipper-forms test's
header, guide 21, the facts form's intro, the changelog, the worksheet.

**5. The cascade's 12 V not marked chosen (correctness minor).** REAL (ROB1-A's default, kept). Marked in the
driver file's comment block under the controls (the banner now says THREE numbers nobody gave); the quoted code
block in guide 21 is unchanged (its verbatim gate holds) and a paragraph below it names the three.

**6. `ControlKind::Hold` / `words::whileHeld` comments said "two buttons" (correctness minor).** REAL. Both
reworded ("two buttons ± volts, or one button + volts on a power mechanism (oneWay)"); `docs/api` regenerated.

**7. The screen says FLIP for the front toggle (correctness minor).** Real, LEFT for tonight as the review advises:
the flipper check finds the mechanism by the literal name "flipper"; renaming needs that check changed (a flag, not
a name) — a decision for later. Guide 21's row legend now says "`FLIP` is the front toggle". **The team should be
told.**

**8. Stale "robot one does not exist in this project's records" (correctness minor + docs minor).** REAL. Reworded
to "its facts are typed from its builder's answers (2026-11-15), not measured" in `src/team/robot_one_autons.cpp`,
`src/robots/mechanism_table.hpp` (the banner's last line), `src/programs/drive_program.cpp` (`kRunStatusOne` — the
line robot one's Drive PRINTS) and `test/robot_one_autons_test.cpp`. Also: guide README's row 21 ("its default
controls" → "the builder's button map" — the first run's log wrongly said the README was still true); the roadmap's
ROB1 paragraph (its lead, HA-261 added, "Next with it" rewritten); REMAINING-WORK row 39 and Q3 (answered: BLUE).
Not changed, on purpose: the changelog's 2026-11-13 entry and the HA register's ROB1-A paragraph (dated history,
true when written), the ledger's "Every one OWED on 2026-11-11: robot one did not exist in our records" (dated).

**9. The compact row's moving / hot / unreadable states untested; robot one's full row unpinned (tests minor ×2).**
REAL. A new case in `robot_one_driver_test`: R1 held → ` 10F99` (120 °C), R2 held → ` 10R99` (99.6 °C), the side
toggles ` 9B00` (−5 °C, 0.4 °C) and ` 2B--` (NaN) — temperatures set only for the drawing and put back; and the
cascade-only list drawn in robot one's words begins `MECH ` and equals the claw-words row from character 4 (ROB1-A's
GM14 pin, restored). Seen on the host: `MECH 10F99 9B00 2B-- CLAW:C FLIP:C RAMP:U`.

**10. The one-button refusal pinned for the arm only (tests minor).** REAL. Three TUs added:
`d_one_button_hold_on_a_lift`, `…_on_a_pneumatic`, `…_on_a_flywheel`.

**11. `bind()`'s one-way refusal untested (tests minor; the first run had said only a hand-built Control reaches
it — true, and a test CAN hand-build one).** REAL. A case in `mechanism_driver_loop_test`: robot x's own controls
copied, the arm's hold flagged `oneWay` → bind throws `PreconditionError`, not bound (the unflagged copy binds).

**12. Drive's robot-one driver line unpinned (tests minor).** REAL. Pinned in `drive_program_source_test` (the
P3-T3 (T4-3) case, from the `kRobotOneBuild ?` choice through the string) and "a DEFAULT" absent from the code.

**13. MC12b's "nothing else failed" over-claims (tests minor).** REAL: the driver's build has no `-k`. The first
run's line is corrected in place to "the build stops at the first failing translation unit … not shown".

**14. The worksheet's Station R1 stale (docs major).** REAL. Rewritten: R1.0 answered (no IMU question — Calibrate
discovers it), R1.1 typed except the right toggle's port (the "8 and 9" warning gone), R1.2 runs each side toggle
(briefly, uncapped) and fires each piston, R1.4 the bottom and top stops only and the homing push's rule; the answer
table filled with every 2026-11-15 answer and its source, and rows G21-G24 added.

**15. Tonight's goal for Drive missed (docs major).** TRUE, and the first run's log did not say so. On the real
tree Drive, the Auton Tester, Competition and Tune wait on TEN owed facts: the right toggle's port; the cascade's
rows, hold volts, homing push and unhomed volts; each side toggle's direction; the claw's, the front toggle's and the
ramp's opening level. The goal named only the rows and the directions. **Cause:** the three voltages are Stage B,
"measured, or chosen with the coordinator" (form C8), and the task chose only the cap and the signature — this run
does not invent the rest (landmine 17); the opening levels are station 13's by the task's own item (4), which Step 4's
goal contradicts. **Decisions left to the coordinator:** (a) type chosen starting values for the hold volts, the
homing push (it must draw more than 0.45 A stalled) and the unhomed volts; (b) rule whether the opening levels gate
Drive (for driver control they change nothing physical — each piston starts unpowered, Low, and each press flips it —
only the names an auton reads) or only the programs that run autons.

**16. Facts form internally inconsistent (docs minor).** REAL. Section C's intro now lists what Bench Tests
waits for (21's ports and cartridge, the side toggles' cap, 22-24's letters); the footer says the side toggles' cap
is chosen with the coordinator (item 21 does not ask it); item 14 worded for a direct-drive spool (guide 21's and
the worksheet's matching "gearing turns freely" too).

**17. "Safe" at 12 V, and station 13 runs the side toggles uncapped (docs minor).** REAL: a cap reaches the motors
only when a program's mechanism service binds; station 13 binds none (D13). Guide 21: the side-toggle row says
"chosen, not measured, to bound the push … not by station 13"; the station-13 warning covers the side toggles
(briefly, never held against a stop); the order on the robot's step 3 says it. The ledger's cap comment, the
driver file's comment and the table's side-toggle comment say the same.

**18. "Press = open" depends on an owed fact (docs minor).** REAL. Guide 21, under the controls table: the first
press moves each piston from where it rests unpowered; if station 13 finds one rests open (or down), tell the
coordinator — the builder's map then conflicts with "nothing moves at power-up".

**19. Deltas only in the first run's report (docs minor).** Recorded here: (a) Drive's goal — item 15; (b) the
three stroke times were TYPED as chosen 0.5 s although Step 2's owed list named them: the task's item (4) allowed "a
chosen starting stroke time … with its source", so they gate no build now and the bench times them later; (c) the
side toggles' "when I let go it will stop" is read as `.safe = Brake` (a stop that resists back-driving; Coast would
let a toggle drift) — the builder's word was "stop", Brake is this run's reading; (d) the front toggle named
"flipper" — item 7; (e) the `bind()` refusal — now tested (item 11); (f) robot one's Tune TEST B on the host test
plant: SUCCEEDED after 1.7 s against the first attempt's 1.766 s limit (`tune_program_test.cpp` :2272) — a thin
margin on the placeholder rows; the review round's probe PRC (rows typed 0 in / 0 deg, 38 in / 1700 deg) kept it
green, but watch it when station 13's rows are typed.

**20. Merging onto main (docs minor) — for the coordinator, not done here (this tree's base is `9befdb3`):**
move the changelog entry to the top of `## API 2.9`; the HA register's "7 of 228" and the ROB1-C2 paragraph's
"With it, 228" become 231 (MIRROR1-A's HA-258..260); regenerate the briefing after rebuilding (next free HA-262);
RMP1's brief "next free is HA-261" goes stale; re-run `robot_one_flipper_forms_example_test` on the merged tree
(MIRROR1-A changed `toggle_visit.hpp`).

**Mutations and probes (`mutations-r.log`; each RED on a fresh link unless a probe):** RV1 the compact row always
chosen → RED (`mechRow.substr(4) == clawRow.substr(4)`); RV2 the row word's `memcpy` removed → RED (`starts_with("MECH
")`); RV3 F / R swapped → RED (2); RV4 the clamp removed → RED (3); RV5 `!c.oneWay &&` removed from `bind()`'s check
→ RED (did NOT throw); RV6 the one-button refusal's requires-clause cut to `Arm` → the harness RED, 3 of 4 (lift,
pneumatic, flywheel — refused unnamed; the arm's still named); RV7 Drive's robot-one line reverted to "a DEFAULT --"
→ RED (2); MH1 the home on `cascadeStall()` again → RED (11: D5's homing amps, T8 Succeeded / homed, Tune's HOME);
MH2 `cascadeHomingStall()` reading the jam amps → RED (11); MC8r a given fact's source re-worded → RED (`builder ||
chosen`); MC8b a later fact typed with no date → RED (the date check); MC8c a source "guessed …" → RED. Probes that
must stay GREEN (the review's false alarms, now accepted): PR2 the hold volts typed `shulib Tune COPY, 2026-11-17`;
PRC the rows typed `station 13 RECORD and a tape, the team's builder, 2026-11-16`; PR3 the homing push typed −5 V;
PRB the placeholder push −3 V — all four GREEN over every robot-one and Tune case (38 cases each).

**T6 — robot two's (and bench's, xdrive's) EIGHT packages, `9befdb3` vs this tree** (`scratch/rob1-c2/t6r/`: both
trees built in turn at ONE path, `work/`, P0's method, `GIT_CEILING_DIRECTORIES`, every make capped; `t6r_report.txt`,
`t6r_strings.txt`, `t6r_symbolic.txt`; the change copy's `src/ include/ firmware/ Makefile common.mk` = the
worktree's, `diff -rq` empty):
- **bench, xdrive, tank (Bench Tests), calibrate, autontest:** `.bin` sizes identical (71756 / 130004 / 153940 /
  108136 / 248396); text / data identical; per-function disassembly modulo addresses **0 differing**; `.data` masked
  0 bytes; `.rodata` masked 5-8 bytes (the build timestamps); `strings -a` of each `.bin` the same but for timestamps
  (xdrive: one code-byte "string" gaining a tab).
- **tune:** `.bin` 119944 both; one "differing function", `__screen_erase_veneer` (a linker veneer, 7 instructions
  both — its target's order); `.rodata` masked 7 bytes and the symbolic compare only the timestamps.
- **tank Drive and Competition: CHANGED, as the first run said they would.** `.bin` 177940 → 178080 and 229864 →
  230000; text +138 bytes each (`arm-none-eabi-size`: 177929 → 178067, 229840 → 229978); `.rodata` +38 bytes —
  NOT provable by the masked compare (size differs), so explained by hand with the symbolic compare: the ONLY
  rodata additions are the string `%.23s: hold-to-run %s (one way), %d V` (37 characters + NUL) and the timestamps
  (one `CSWTCH.235` "difference" is the tool's word grouping at a moved odd address: its 30 raw bytes are identical).
  The functions: `DriverInputs::matchesThisRobot` now OUT of line (97 instructions — the `!c.oneWay` term; the base
  inlined it into `bind`, 157 → 71); `mechanismControlLine` 153 → 169 (the one-way boot line); `tickHold` 192 → 194
  (the `!h.oneWay` guard on the back button); `runDrive` 1110 → 1113 and `runCompetition` 990 → 993 (`ldrb [r3,#57];
  cmp; bne` — the inlined `takes()` reading `oneWay` before a control's back button, plus literal-pool offsets moved
  by 8); `__competition_get_status_veneer` (order). Robot two never sets `oneWay` — its driver file
  (`src/team/robot_two_driver.hpp`, `git diff 9befdb3` empty, as are `autons.hpp` / `autons.cpp`) holds two
  four-argument `whileHeld`s, which leave the field at its `false` default — so no new branch is taken: **robot two behaves as before; its two driver packages
  are 138 bytes larger.** T6 (a): `src/robots/mechanism_table.hpp` lines 1-138 identical to the base (robot two's
  region ends at :135).

**Prose review (grepped, hits read):** the brief's three — `only robot two\|robot two's autons are\|ROBOT=tank`
over docs/guide, docs/*.md, README.md, src/: 46 hits (robot two's build commands in guides 07/16/17/18/21, the
roadmap, the changelog, `src/build_selection.hpp`'s refusals — "only robot two and robot one have a driver file",
still true — `src/main.cpp`, the programs' banners): none stale; `simulator` in guide 21 and the facts form: 0;
`G1-PROGRESS` in `ROB1-*`: only this log's header note and old entries quoting the check, and the brief. This round's
own: "homing signature" / "ONE signature" / "same three facts" / "HOME's signature" across docs (guide 21 :164-180
rewritten; chapter 18 :204-218, guide 11 :336-339, the HA register :5402 already say a homing signature is its own,
0.45 A to start — consistent); "does not exist in this project's records" (fixed in the four files above; dated
history kept); "ROB1-C2's builder" (0 left); "safe to hold" (0 left); "gearing turns freely" (guide 21, the form,
the worksheet — all reworded); "default controls" (guide README fixed); "two buttons" in the header and docs/api
(fixed). Not touched, with why: guide 18 (its flipper words are generic; the one-button bullet already there),
`build-order.md` and the roadmap's "You are here" (the coordinator's pointers at the merge).

**Gates on the final tree** (logs in `scratch/rob1-c2/`): host build exit 0 (`build-final-r.log`, the doc gates
inside); `suite_run.py green` — **green, 3,710 cases, 3,862,242 assertions** (`suite-final-r.log`; 3,708 + the two
new cases); the compile-refusal harness **295 of 295** (`ctest-r.log`; 292 + the three new TUs; TUs 295 = CTests
295); `src_build_gate.py check` **PASS — all 14 builds** (`srcgate-r.log`): robot two's, bench's and xdrive's 8 link;
robot one's Bench Tests and Calibrate refuse naming exactly ONE fact; Drive, the Auton Tester, Competition and Tune
exactly TEN (item 15's list); all six link in the gate's typed copy (12 owed facts typed there only); `self-test` OK
(377 detector cases, `gate-self-r.log`, one run under a 58-minute timeout); `git diff -- Makefile src/ common.mk`
holds no `(self-test plant` (the self-test's restores bumped `src/main.cpp`'s and others' times only: `src/
include/ test/` are byte-identical to `mtree/`'s, so the suite was rebuilt and re-run after it); no
`build/src-gate-facts/` left; the ARM header gate over all 255 headers exit 0; the CI PROS-free and sim-layering
greps and the removability grep empty; api_doc_tool self-test / check-coverage (6,390 entities) / check-fresh /
check-examples (860 lines verbatim) / check-removability exit 0; briefing_status self-test and `check` current
(regenerated after a fresh link); the staleness audit clean (`cheap-gates-final.log`). Not committed.

## ROB1-C2 — the coordinator's merge verification *(2026-11-16, ~01:40)*

- **Squash-merged from `rob1-c2`** (transport `116bf65`, base `9befdb3`) onto `28a7497` (MIRROR1-A and the Tune label fix
  landed since). Conflicts: the register (both additions — MIRROR1-A's HA-258..260 and this run's HA-261; HA-255 now
  **reported**; **7 of 231** settled), the generated API pages and the briefing (regenerated). The changelog entry moved to
  the top of `## API 2.9` (the one-button `whileHeld` is additive in an unfrozen layer — no bump).
- **Typed at the merge by the coordinator:** `kRightTogglePort = 9` — the builder, 2026-11-15: "9 is right side and left is
  2" (equal to its placeholder, so its source says "same as the placeholder, checked"). **Bench Tests (`one`) and
  Calibrate (`one-calibrate`) now LINK on the real tree**; Drive, the Auton Tester, Competition and Tune refuse naming
  exactly their 9 Stage B facts (station 13's rows, the toggles' run directions, the pistons' open levels, the three cascade
  volts).
- **Gates — the FAST set for tonight's upload** (`scratch/rob1c2_merge_fast.sh`, log `logs/rob1c2_merge_fast.out`, plus a
  rebuild after the briefing was regenerated first): suite green (**3,762 cases**); `src_build_gate.py check` **PASS — all
  14 builds** (4 robot-one builds refused as they must, Bench Tests and Calibrate linking); the ARM header gate exit 0;
  `ctest` compile refusals **295 of 295**; api check-examples / fresh / coverage / removability; staleness; briefing current;
  no plant, no untracked file. **NOT yet run on the merged tree (run right after this commit, logged below when done):**
  the src gate's self-test and the coordinator's own hand mutations. The builder's 12+ mutations (MC1–MC9b…) were red on
  its tree.
