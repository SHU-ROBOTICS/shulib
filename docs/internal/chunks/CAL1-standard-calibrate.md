# CAL1 — the standard Calibrate, shipped with the library: a team states its drive kind, its cartridge and its mechanisms' ports, and gets the same, correct Calibrate; the front is the end that leads the push, with a flip check; SAFE-SAVE B1's safe save inside it

> **THE TEAM LEAD'S ANSWERS (2026-10-08)** to "Decisions for the team lead", in his words: **"Accept, but add
> D6's check."**
> - **D1 (a) — DECIDED, as recommended:** the split — a PROS-free `calibration::CalibrateProgram` over seams, plus a
>   thin PROS shell in `hal/pros/calibrate.hpp`; the CI rule unchanged.
> - **D2 (b) — DECIDED, as recommended:** FRONT FLIPPED holds the save; USE NEW (save as pushed) or RUN AGAIN (save
>   nothing); KEEP OLD withdrawn.
> - **D3 (a) — DECIDED, as recommended:** one `CalibrateFacts`, the team's 21-line file, robot two's facts DERIVED
>   from its tables.
> - **D4 (a) — DECIDED, as recommended:** the watch is CAL1's, reads through PROS's C API (HA-187), and a declared
>   port that turned HOLDS the save (MECHANISM PORT TURNED).
> - **D5 (d) — DECIDED, as recommended:** three parts, A1, A2 and B, each its own go.
> - **D6 (a) — DECIDED, NOT the recommended (b):** a FIRST record (no usable record on the card) on a robot whose
>   facts carry TYPED drive ports is checked against those typed ports. Ruling (1)'s "the previous record" becomes
>   "the previous record, else the typed ports". Designed and probed in this brief (the Re-check log's "D6 (a)"
>   entry; §1.1, §1.4, §2; `d6a/`); (b) is kept in D6 as the alternative not taken, with its cost.
> - **The go to build** stands from his ruling (2) of 2026-10-06: CAL1 is built right after T4 Part 3 lands,
>   beside the MECH core, and lands before MECH-A5 (the MECH core brief's N9, accepted 2026-10-08:
>   `docs/internal/chunks/MECH-core-mechanism-library.md`).

> **Execution brief (decided 2026-10-08 — the answers above; built after T4 Part 3 lands).** Written 2026-10-06 (evening) and 2026-10-07 from a read-only
> pass over the tree that T4 Part 3 leaves: `.claude/worktrees/t4-3`'s WORKING tree (HEAD `6abcac8`, main `fab1ebb`
> merged and staged, uncommitted), with R5a Part A (`083e232`) laid over it. That tree was copied to
> `.claude/coordinator/scratch/brief-CAL1/tree/` on 2026-10-06 at 17:02 (`tree_manifest.sha256`). **Re-checked
> 2026-10-07 against the live worktree** (`diff -rq`): since the copy, Part 3 changed only documents
> (`T4-PROGRESS.md`, `PROJECT-BRIEFING.md`, a new `T4-COMPLETED.md`) and one comment block in
> `competition_program.cpp` (:66-71); no file this brief cites moved. **Re-checked again 2026-10-08 (~09:25)**
> against the live worktree (`sha256sum -c tree_manifest.sha256`, then a `cmp` of every file this brief cites): T4
> Part 3's three reviews ran on 2026-10-07 and its review-fix run (started 23:51, still editing at 09:31) has changed
> six library headers (`diag/fault.hpp`, `diag/loop_monitor.hpp`, `diag/log_copy_sink.hpp`,
> `motion/motion_scheduler.hpp`, `teleop/coupled_side_monitor.hpp`, `hal/pros/device_provider.hpp`), seven `src/`
> files (the tester, Drive, Competition, `drive_panel.hpp`, `dropped_autons.hpp`, `log_copy_line.hpp`,
> `team/autons.hpp`), their tests and documents. **None is a file this brief cites for Calibrate:**
> `calibrate_program.{hpp,cpp}`, `src/main.cpp`, `screen_text.hpp`, `build_selection.hpp`, `mechanism_table.hpp`,
> the `Makefile`, `src_build_gate.py`, `api_doc_tool.py`, `ci.yml`, `calibrate_panel.hpp`, `record_store.hpp`, the
> fence test and `mechanism_table_test.cpp` are byte-identical to the copy. `device_provider.hpp`'s change is the log
> copy's one-writer guard (comments and `serialCopyDroppedLines()`); the standard Calibrate's shell makes
> `ProsDeviceProvider{registry}` with no log copy (`proposed3/…/hal/pros/calibrate.hpp` :123). **Re-probed with those
> 2026-10-08 files overlaid** (`recheck_1008/`): the ARM header gate passes over all 214 headers (its `int x = d;`
> control fails); the draft's 19-line team project re-links a V5 package (the revision's 21-line file: P19); and the real `src_build_gate.py check` over the end
> state (`tree_gate/` + the thirteen changed files, as of ~09:25 — Re-check R9) — §0.6 P14. **Every file:line below was opened in the copy**
> (`~:N` = within a few lines). The plan commit `fda5351` (CAL1's build-order entry and the rulings) was read too.
> Whoever launches A1 and A2 re-reads `calibrate_program.cpp`, `src/main.cpp` and the gate as Part 3 LANDS (§5).
>
> **Status: DECIDED (D1–D6, the team lead, 2026-10-08: D1–D5 as recommended, D6 (a)). The go to build is ruling
> (2) of 2026-10-06: after T4 Part 3 lands. Nothing is built.**
>
> **The team lead's words this brief holds (2026-10-06):** "we need to standardize this calibrate program so that
> we can push it with the library so people who want to use the vscode extension can have the correct calibrate
> program to actually use the extension"; and, on the panel's front field, "itll be hard to say which way is the
> front through the panel ... maybe we just get it from the calibrate?" — "Yes, with a flip warning". **His rulings
> the same day (binding, not re-asked):** (1) the robot's FRONT is never asked in the panel or typed: the end that
> LEADS Calibrate's push is the front, carried by the calibration record, WITH A FLIP WARNING when a new record's
> front is opposite to the previous record's; (2) CAL1 is built right after T4 Part 3 lands, beside the mechanism
> core, and ABSORBS SAFE-SAVE Part B1; (3) only Calibrate is standardized now (the Auton Tester and Bench Tests are
> PKG1's to decide). Also held: the runtime-discovery ruling (2026-09-10: a human states only what cannot be
> measured), and his 2026-10-04 "the new programs are kinda confusing looking and look hard to write" (§1.1 shows the
> team's file verbatim, compiled, counted).
>
> **Runs:** three executor runs (A1, A2, B — §2; the re-check split the draft's Part A, D5 (d)), each its own go,
> each in an isolated worktree (`.claude/worktrees/cal1-a1`, `cal1-a2`, `cal1-b`, branch of the same name, from the
> tip at launch). Below, "Part A" means A1 and A2 together wherever the text does not split them. **Executor
> model: `opus`. Do not commit.** Live log: **`docs/internal/chunks/CAL1-PROGRESS.md`, created FIRST** by A1,
> appended as you go, a heading per part. **Hardware Assumptions numbers: HA-187 ONLY** (the register's Held line,
> `docs/hardware-assumptions.md` :39, holds HA-187..HA-188 for CAL1; HA-188 is not needed and is RELEASED — §3.3).
> A second belief ⇒ stop and tell the coordinator. Say "host test plant", never "simulator". **Nothing here has
> run on a robot, and nothing in this brief runs on one.**
>
> **Probes** (evidence, never project code) — `.claude/coordinator/scratch/brief-CAL1/`: `tree/` (the copy above);
> `proposed3/` (**Part A as §1 writes it**: the new library files and the ADDITIVE panel change, the calibrator
> untouched — an overlay of `tree/`); `proposed2/` (Part A + Part B's removal of the WHICH END question: the end
> state); `proposed/` (the first draft both correct — §0.6 says what changed); `shim_extra/pros/` (the host shim
> pieces the probes needed and the suite lacks: a `screen.h` that records every draw call, the C competition calls
> of `misc.h`, an `rtos.hpp` with an advance hook); `harness/` (**the golden harness**: it runs a Calibrate PROGRAM — today's
> `src/programs/calibrate_program.cpp`, or the proposed one — on the A2 host test plant THROUGH THE PROS SHIM, robot
> two's shape, with a scripted operator that pushes and taps only what the SCREEN shows; `build_old.sh`,
> `build_new.sh`, `run_all.sh`); `golden_old/`, `golden_new/` (each scenario's serial log, screen draw log and the
> record it saved), `rerun_old/`, `rerun_new/` (the same, re-run 2026-10-07: byte-identical), `ctl/` (the timing
> control); `tree_a/` (`tree/` + `proposed3/`'s headers, `src/` untouched — the tree Part A leaves) and `tree_gate/`
> (`tree/` + `proposed2/` + robot two's glue — the tree Part B leaves), each its own scratch git, where the real
> `make`, the real `src_build_gate.py check`, the ARM header gate and the whole host suite ran; the rest is named
> where it is used. **Re-check (2026-10-08): `amend/` supersedes them wherever the re-check changed the design** —
> `amend/p3r/` (the revised Part A form), `amend/p2r/` (the revised end state), `amend/harness2.cpp` and its runs,
> `amend/tree_a2/`, `amend/tree_end/`, `amend/gate_selftest/` (§0.6 P15–P22). **D6 (a) (2026-10-08, afternoon):
> `d6a/` supersedes `amend/` wherever D6 (a) changed the design** — `d6a/p3r/` and `d6a/p2r/` (the two library forms
> with the typed-port check), `d6a/harness3.cpp` and its runs `d6a/run_new/`, `d6a/run_end/`, `d6a/p3r_old/`,
> `d6a/tree_a3/`, `d6a/tree_end3/`, `d6a/team_proj3/`, `d6a/team3/`, `d6a/boot_probe/`, `d6a/facts_probe/`,
> `d6a/screen_probe/` (§0.6 P23–P31). `amend/` is unchanged; the D6 (a) probes read it and never wrote it.
> **`d6a/v2/` (2026-10-08, ~14:40–15:45): the continuation's re-verification** of every D6 (a) claim (each probe
> re-run from its source with the final harness) and the same proofs on T4 Part 3 AS LANDED (`bdcbc7c`,
> `d6a/landed_a/`, `landed_end/`, `landed_ctl/`) — §0.6 P32, the Re-check log's D6 (a) entry.
> **`d6a/v3/` (2026-10-08, evening): the check of D6 (a)** — five findings, all real, fixed at the root and
> re-probed (`v3/p3r/`, `v3/p2r/` = the two library forms with the fixes; `v3/harness4.cpp` and its runs; the
> parity probe; `mutate_v3.py`; `v3/landed_a3/`, `landed_end3/`, `team_proj4/` = T4 Part 3 as landed + the
> fixes) — §0.6 P33–P37, the Re-check log's "The check of D6 (a)" entry. **`v3/` supersedes `d6a/p3r/`,
> `p2r/` and `harness3.cpp`.** `v3/recheck/` (2026-10-08, ~19:50–20:40): that entry's continuation — the goldens
> and all 32 mutations re-run from source (identical), the runner's seed refusal, and the end state's `self-test`
> re-run after the usage stop cut it off.

## Re-check — amendments (2026-10-08, two checkers' findings, each re-verified)

> Two checkers (a "tree" lens and a "safety" lens) read this draft on 2026-10-08. Their 18 findings are
> taken below one by one, each re-verified against the probe copy and the live worktree. A finding that is
> real is fixed at the root and the fix re-probed. A finding that is not real says why, with evidence.
> Re-check probes live in `.claude/coordinator/scratch/brief-CAL1/amend/` (named per finding). The body
> of the brief is amended to match. Where an amendment and older text disagree, the amendment wins and
> the older text is marked. **What changed for the team lead (at the top of "Decisions"):** D2 now recommends
> (b), hold then USE NEW or RUN AGAIN, and KEEP OLD is withdrawn; D4's watch now HOLDS the save; D5 now recommends
> three parts (A1 ~1–1¼, A2 ~1½–2, B ~1½–2; ~4¼–5½ in all); D6 is new (a first record on a robot with typed drive
> ports; recommended: state the gap — **the team lead decided (a), the typed-port check, the same day: the "D6 (a)"
> entry near the end of this log; the sizes above are superseded there, and again by "The check of D6 (a)" after
> it — A1 ~1¼–1⅜, A2 ~2½–2⅞, B ~1⅞–2⅛, ~6–6½ in all**). **Live worktree re-check at 10:12:** `sha256sum -c tree_manifest.sha256` over
> `.claude/worktrees/t4-3`. The only changes are T4 Part 3's fix-run files (now also
> `competition/auton_registry.hpp`, `auton_menu.hpp`, `driver_panel_test.cpp` and others) and R5a Part
> A's overlay. No Calibrate file, `src/main.cpp`, `screen_text.hpp`, `build_selection.hpp`, the
> `Makefile`, `src_build_gate.py`, `mechanism_table.hpp`, `calibrate_panel.hpp` or `record_store.hpp`
> changed.

### Dispositions, one per finding

**R1 (tree, major): Part B breaks `src_build_gate.py self-test`. REAL; fixed and re-probed.**
- *Verified.* `CALIBRATE_PLANT` is `src/programs/calibrate_program.cpp` (`tools/src_build_gate.py` :1503).
  Self-test case 26 plants on `dropDeclaredMechanismPorts(kMechanisms,` (:2207-2208). `_replace`
  asserts exactly one occurrence (:1556-1569). The anchor occurs once in the copy and 0 times in the
  24-line glue. Loading the end state's gate and entering that plant gives
  `AssertionError: plant target must occur exactly once in …/calibrate_program.cpp` (reproduced).
  P4 and P14 ran `check` only.
- *Fix (§1.8, §2.B, §3.2, §9.2 amended).* Part B re-points case 26's Calibrate plant at the file that now
  names the build's tables:
  - `CALIBRATE_PLANT = src/robots/calibrate_facts.hpp`, anchor `calibrateFactsFor(kChassis, kMechanisms)`
    → `calibrateFactsFor(kChassis, mechanismTableFor(RobotId::BenchBot))`. That file is not one of
    `TABLE_FILES` (:451-452), so step 0 still reports a per-robot name there.
  - The docstring's `tank-calibrate` paragraph (:39-40) says "the standard Calibrate
    (`include/shulib/hal/pros/calibrate.hpp`) with this build's facts (`src/programs/calibrate_program.cpp`,
    `src/robots/calibrate_facts.hpp`)".
  - :174's example "`mechanismTableFor(RobotId::BenchBot)` in Calibrate" becomes "… in Calibrate's
    facts".
  - `tools/` joins §3.2's prose-review grep.
- *Re-probed* (`amend/gate_selftest/`: `recheck_1008/tree_gate` plus that one edit, its own scratch git
  `bd75019`): `python3 tools/src_build_gate.py self-test` gives **`OK (211 detector cases, each against a
  real build)`, 11 m 12 s** (`amend/gate_selftest.log`). Part B's gates line in §2.B grows from ~¼ to
  ~⅓ day (the self-test's wall time plus the edit). Part B's size is still ~1½–2 days.

**R2 (tree, major): D2 (a) changed ruling (1) without saying so. REAL; D2's recommendation is now (b), "hold,
then USE NEW or RUN AGAIN", the ruling's own letter. KEEP OLD and `flipFront` are withdrawn. Re-probed.**
- *Verified.* Ruling (1) is "the end that LEADS Calibrate's push is the front … WITH A FLIP WARNING". The
  brief marked it binding. The draft's KEEP OLD saved `flipFront(measured)`, a record whose front is the end
  that TRAILED the push. Its screen's LARGE line was `WHICH IS FRONT?` (`proposed3/…/calibrate_panel.hpp`
  :450-451, drawn in `final_new/flip-keep.screen.log`). The brief's own C1 and §4 reject exactly that: "the
  front ASKED".
- *R10 makes it worse than a wording problem.* The records cannot tell a flip from a left/right cable swap,
  and on a swapped robot KEEP OLD reverses every auton (measured below).
- *Fix.*
  - **D2 now recommends (b)** (Decisions, rewritten). A FRONT FLIPPED check holds the save on a screen
    headed `NOTHING SAVED YET`. **USE NEW** saves the run as pushed (the end that led IS the front, ruling
    (1)); **RUN AGAIN** discards it. A field disable or a power-off writes nothing.
  - The screen never asks which end is the front. It states what the records say ("the card calls the end
    you led BACK") and what each touch means (R10's words). Ruling (1) holds without exception: every saved
    record's front is the end that led its push.
  - `flipFront` and KEEP OLD leave the design. `FrontChoice` keeps `None`/`New`. C6 (KEEP OLD's provenance)
    is withdrawn. C1 and §4's rejection now agree with the screen.
  - The draft's (a) is kept in D2 only as an explicit request to amend ruling (1), with R10's measured cost.
- *Re-probed* (`amend/p3r/`, the revised Part A library; `amend/harness2.cpp`; `amend/run_new/`; robot two's
  shape on the A2 host test plant, through the PROS shim):
  - `flip-again`: run 2 led with the other end, so it was HELD, then RUN AGAIN. Run 3 led with the front,
    SAME FRONT, saved. The card drives **10 of 10** motors forward on their own side.
  - `flip-new`: the same wrong push, then USE NEW. Saved as pushed: **0 of 10** (the warned-of case).
  - `flip-disable`: held, then the field disables. Both files are byte-identical to run 1's record, and the
    line reads `the field disabled the robot on the held FRONT FLIPPED screen -- NOTHING SAVED: the card
    keeps its record`.
  - Today's program on the revised Part A panel: **33 of 33** output files byte-identical to `final_old/`
    (`amend/p3r_old/`). The pure-addition claim holds for the revision.
  - The seven comparable scenarios classify with **residual 0** (`amend/classify2.log`; one new reason M,
    R11's words). The records of `one`, `slip18` and `curved` are byte-identical to today's.

**R3 (tree, minor): worksheet citations wrong; M.0.2's "wait for B1" rule missing from Part B. REAL; fixed.
One more error found and fixed.**
- *Verified* in the copy and the live worktree. The live worksheet has 2,111 lines; the copy has 2,097.
  - Station 6 is at :284 in both. The draft's :265 is the controller station's "Do this even if nothing
    else gets done".
  - The copy's :849-850 are the `5/5 SPIN` and `SAVED` rows of Station G's screen table. The interim
    false sentence ("A run that says NOT SAVED may already have ERASED the record … Copy the record off
    the card first (G.0.1)") is copy :876-882, live :884-890. G.0.1 is copy :738-744, live :746-752.
  - The WHICH END IS FORWARD box is live :452-461. M.0.2 is live :1596-1598 ("do it only after SAFE-SAVE
    B1 is in the build"). HANDOFF's S1b repeats "copy the record off the card first, G.0.1" (live
    `HANDOFF-remaining-work.md` :724).
- *Found while re-verifying: the draft was wrong about :354 and :616/:622.* Live :354 is D.1.2 and live
  :622 is F.2.1. Both are **Bench Tests' MOTOR WATCH** question ("WHICH WAY DID YOU PUSH IT?" → FRONT
  FIRST). That question STAYS under CAL1 (§1.8, §4, ruling (3)), so those lines must NOT be reworded.
  The draft's §3.2 listed them for change. The box's item 1 ("Forward … is the end that LED the
  Calibrate push saved in the record") is already true under CAL1 and stays. Items 2-3 are MOTOR WATCH's
  and the boot's and also stay. The box gains one sentence: Calibrate now checks a new record's front
  against the card's.
- *Fix (§3.2 amended).* Line numbers are no longer cited. Part B starts from a later tip, so §3.2 names each
  place by its anchor, with the live number as of 2026-10-08 10:12 in brackets:
  - Station G: the whole station, rewritten from the code (G.0.1 [:746-752]; G.4's RUN AGAIN paragraph
    [:884-890]; the screen table [:851-859]; the serial list [~:985-1000]);
  - Station 6 [:284] (the card holds the record AND its backup);
  - **M.0.2 [:1596-1598]**: "copy … first, and only after B1" becomes "the card keeps a verified backup
    (CAL1); copying the record off first is still a good habit";
  - the WHICH END IS FORWARD box [:452-461] gains its one sentence; D.1.2 and F.2.1 stay as written;
  - **HANDOFF's S1/S1b line [:724]** joins Part B's internal edits.
- `off the card` joins §3.2's prose-review grep.

**R4 (tree, minor): Part B's documents omit regenerating `docs/api/`. REAL; fixed.**
- *Verified.* The copy's `docs/api/drivetrain_calibrator.md` documents `Answer::FrontFirst` (:389) and
  `CalibrationReport::backFirst` (:1541). `docs/api/calibrate_panel.md` documents `PanelScreen::WhichEnd`
  (:64) and `PanelButtonId` (:73). Part B removes or renames each of these, so `check-fresh` (§6), which
  compares the whole directory byte for byte (`api_doc_tool.py` :1798), fails until they are regenerated.
- *Fix.* §3.2 and §9.2's Part B documents column list "`docs/api/` regenerated". The changed pages are
  `drivetrain_calibrator.md`, `calibrate_panel.md`, `calibrate_program.md` (Part A's page; Part B removes
  the `FrontRule` arguments) and `front_check.md` (R2 removes `flipFront`), plus `all-entities.md` and the
  index.

**R5 (tree, minor): P13's "pure addition" evidence lacked Part A's shim changes, and test 9's check contradicted the
brief. REAL; P13 re-run with the shim pieces in the tree; test 9 and the DoD reworded.**
- *Verified.* P13's `tree_a/test/pros_shim/pros/` had no `screen.h` or `misc.h`, and its `rtos.hpp` was
  today's. Yet §1.2 item 8 changes that EXISTING shim header (the advance hook). So "`git diff --stat --
  test/` shows only NEW files" could not hold. The DoD's "the two changed headers" disagreed with §1.2/§9.2,
  which change one library header.
- *Re-probed* (`amend/tree_a2/`, its own scratch git). `tree_a` plus the REVISED Part A library plus the shim
  pieces as project code in `test/pros_shim/pros/`:
  - `screen.h`, `misc.h` and the C-API `motors.h` (R6), each with the project's `#error` +
    `SHULIB_HOST_PROS_SHIM` guard;
  - `rtos.hpp` with the advance hook, whose behaviour is unchanged while no hook is installed.
  - Results (P13's method; the doc gate bypassed by commenting one `add_dependencies` line, as P13 did):
    - the WHOLE host suite: **2,231 of 2,231 cases** (`amend/suite_a2_run.log`). Assertions are 2,784,574,
      against 2,783,067 pristine; the rise is the scans that walk the extra headers and literals.
    - today's program on the revised panel: **33 of 33** files byte-identical (R2).
    - the real `src_build_gate.py check`: **PASS**, all 7 builds, 0 disallowed (`amend/gate_check_a2.log`).
    - the ARM header gate: passes over all 214 headers, and its control fails (`amend/arm.log`,
      `arm_neg.log`).
- *Fix (§2.A test 9, DoD).* Test 9 now reads: "the whole existing suite passes; NO existing test TU is edited.
  The only existing test files Part A may modify are `test/pros_shim/pros/rtos.hpp` (the hook; behaviour
  identical while none is installed) and `test/CMakeLists.txt` (only if a new test needs a define: its
  sources are globbed, :79)". The DoD names ONE changed library header (`calibrate_panel.hpp`) and the one
  changed shim header.

**R6 (tree, minor): HA-187 (1) was an avoidable belief; the MECH core reads through the C API. REAL; the watch
reads through PROS's C API now. Re-probed; HA-187 re-worded.**
- *Verified.* The MECH core's 10-06 draft read the watch with `pros::c::motor_get_position`, "never a
  ProsMotor" (the checker's cite, `briefs/MECH-core.md` :1268-1276). Its text as re-checked at ~10:45 hands the
  watch to CAL1 (its N9) and keeps the rule "never construct a `ProsMotor` for a declared port with a gearset;
  CAL1's HA-187 holds the read" (its landmine 8). The draft's `ProsCalibrateIo::readShaftDegrees` built a
  `pros::Motor{port}` and so depended on that constructor writing nothing. The draft's §5 also called the
  core's landmine 8 "already built"; it is not.
- *Measured* (`amend/objdump/`): disassembling the vendored `firmware/libpros.a` `vdml_motors.c.o`, the
  three C reads call only `registry_validate_binding`, `registry_get_device`,
  `port_mutex_take`/`port_mutex_give`, `__errno` and one firmware getter each: `vexDeviceMotorPositionGet`,
  `vexDeviceMotorGearingGet`, `vexDeviceMotorEncoderUnitsGet` (`amend/objdump/calls.txt`). No `*Set` call,
  no motor object.
- *Fix (§1.2 item 7, D4, §3.3, §5, landmine 6, mutation 18).*
  - `readShaftDegrees` reads `pros::c::motor_get_position(port)`, converted by `motor_get_encoder_units(port)`
    and `motor_get_gearing(port)`. No `pros::Motor` and no `ProsMotor` is constructed.
  - The shim gains those three reads (`test/pros_shim/pros/motors.h`). They write nothing and count their calls.
  - HA-187 (1) is now "the three C reads write nothing" (reasoned from that disassembly: the firmware getters
    are closed code). Claims (2) and (3) are unchanged.
  - Mutation 18 becomes "the watch's read constructs `pros::Motor{port, gearset}` → the shim's gearing
    counter". A source pin: `hal/pros/calibrate.hpp` names no `pros::Motor`, `pros::v5::Motor` or
    `ProsMotor`.
  - §5 says the core's landmine 8 is built by CAL1 Part A.
- *Re-probed.*
  - The C-API adapter compiles on the host with the suite's flags.
  - It compiles under the ARM header gate (214 headers) against the REAL `include/pros/motors.h`.
  - It links into robot two's `tank-calibrate` package and the team project.
  - In every scenario the shim counts 0 gearing, units, voltage and brake-mode writes to the declared
    ports (6 and 7, or 6 and 12 in the typo runs), and 2 C reads per push (`amend/run_new/*.stderr.log`,
    `amend/run_typo_counters/`).

**R7 (tree, minor): Part B mutation 4 (`printText` back to its own sanitizer) has no test that would fail;
"THE one sanitizer" overstates. REAL; a test added, the words corrected.**
- *Verified.* `src/common/screen_text.hpp` includes `pros/screen.h`. The only host pins are counts of
  `printText(` in three source tests (`drive_program_source_test.cpp` :311, `competition_source_test.cpp`
  :286, `auton_tester_source_test.cpp` :342), and none of them runs it. `bench_r3a.cpp` keeps its own
  sanitizer at its 54-column width (`screen_text.hpp` :26-32).
- *Feasibility probed* (`amend/printtext_probe/`): a host TU that includes the end state's
  `common/screen_text.hpp` through the probe's recording `shim_extra/pros/screen.h` compiles under the
  suite's flags with `-Werror`. Running it, `printText(Small, …, "50% done — a\tb € end")` records
  `print S 8 180 pen=ffffff eraser=000000 '50? done - a?b ? end'`. A 26-letter LARGE line is cut to its 18,
  and `g_screenPrints` counts 2.
- *Fix (§2.B amended).* Part B adds test 6, `test/screen_text_test.cpp` (NEW). It runs `printText` through
  Part A's shim `screen.h` and checks: the sanitizer's cases (`'%'`, a control byte, an em dash, a 3-byte
  character, the cut at each size's budget, the pad); the recorded call order (eraser, pen,
  `print_at(…, "%s", …)`); and the counter. Mutation 4 now has a test that fails. C3 and §1.2 item 1 say
  "the screen's one sanitizer **outside Bench Tests**".

**R8 (tree, minor): SAFE-SAVE landmine 12's test-helper consolidation has no owner once B1 is absorbed.
REAL; assigned to SAFE-SAVE B2.**
- *Verified.* SAFE-SAVE's landmine 12 (`SAFE-SAVE-calibrate-save-and-teleop-label.md` :1253-1256) says
  consolidating the copied `codeOnly` and robot two's fixture "happens only after both have landed, in B1
  or B2, and is logged". In the live worktree `codeOnly` is used in two places:
  `test/mechanism_table_test.cpp` (9 uses) and its copy in `test/record_store_platform_scan_test.cpp`
  (31). Robot two's fixture is in `test/boot_source_test.cpp`. CAL1 Part B removes only the Calibrate
  call-site pin, and `codeOnly` keeps another user in that file (:336).
- *Disposition (§5 amended).* **The consolidation is B2's.** B2 edits the boot source and its test, the
  file that owns robot two's fixture. It is the last SAFE-SAVE part, so both copies have landed by then.
  CAL1 Part B logs in `CAL1-PROGRESS.md` that it leaves both copies in place, and why. Merging one
  helper in Part B and the other in B2 would touch the same files twice.

**R9 (tree, minor): small citation and count errors. REAL; corrected in place.**
- "Build-order row 18b" (D5, §5) is `docs/internal/REMAINING-WORK.md` :86. Corrected.
- "18 references" to robot two's tables (the computed task's count, repeated in §0.1) is wrong. The copy has
  **11 occurrences of `kChassis`/`kMechanisms` on 10 lines**: :1087, :1130, :1138, :1146, :1155 (two),
  :1160, :1163, :1180, :1227, :1238. §0.1 now lists :1238 too.
- §1.4's tie row: P7's log has `onlyNow 1` (`probe_front_save/probe.log` :11). The row now shows it.
- 1,388 vs 1,387: the program is 1,388 lines in Part 3's tree (the copy and the live worktree, `wc -l`)
  and 1,387 on main (`git show HEAD:…`). §0.1 and §9.2 say "1,388 (1,387 on main)". The glue's comment
  keeps 1,387; Part B writes the count of the file it replaces.
- The header's "thirteen changed code files" is now "as of ~09:25". Later fix-run edits are handled by §5's
  re-check protocol, not by a count. At 10:12 more files had changed (above), none of them Calibrate's.

**R10 (safety, major): FRONT FLIPPED cannot tell a flip from a left/right cable swap, and its words steered
the operator to the reversing choice. REAL, reproduced through the program on the plant; fixed (with R2).**
- *Verified, by reasoning.* Swap cable a (LEFT, sign s) with cable b (RIGHT, sign t) at the brain. Port b
  then reads (LEFT, s). After a push led by the other end it reads (LEFT, −t). On a mirror-mounted tank
  every such pair has s = −t, so the two records are identical. Robot two's pairs 16/11, 17/12, 18/13 and
  20/15 are all mirror-signed (the checker's `check-safety/swap_probe.log`).
- *Verified, measured through the program.* The harness's `swap` scenario: the card seeded with run 1's
  record, the bundles swapped (11-15 ↔ 16-20), the CAMERA end leading.
  - The program measures LEFT −16 +17 −18 +19 −20 | RIGHT +11 −12 +13 −14 +15. That is the same signed
    record as the other-end push in `flip-new`; the two files differ only in 6 timing bytes.
  - `compareFronts` gives FRONT FLIPPED, 10 of 10. USE NEW is right here: **10 of 10** motors forward on
    their own side.
  - What the draft's KEEP OLD would have saved (the seed's ports and signs, `swap-keep-old-equivalent`)
    drives **0 of 10**. On a correctly pushed but re-cabled robot, KEEP OLD reverses every auton.
  - Today's program saves the swap correctly when answered FRONT FIRST. The draft added this failure path.
- *Fix.*
  - D2 (b) (R2): no KEEP OLD.
  - The held screen states what the RECORDS say, never what the operator did. It names both causes and ties
    each touch to what the operator did (drawn, `amend/run_new/swap.screen.log`):
    - header `FRONT FLIPPED`; LARGE `NOTHING SAVED YET`;
    - MEDIUM `the card calls the end you led BACK`;
    - SMALL `RUN AGAIN: you led with the BACK by mistake`, `USE NEW: you led with the FRONT (cables
      swapped?)`, amber `USE NEW after a wrong push reverses every auton`;
    - boxes USE NEW and RUN AGAIN. RUN AGAIN is a choice on this screen (the draft's view lacked it).
  - The serial lines say the records cannot tell the causes apart.
  - A field disable while held prints `… on the held FRONT FLIPPED screen -- NOTHING SAVED: the card keeps
    its record` (measured, `flip-disable`).
  - The cable-swap pair joins §1.4's table and test 3 (verdict FLIPPED, marked "the records cannot tell this
    from a flip"). The `swap` scenario (USE NEW right) and `flip-again` (RUN AGAIN right) join the goldens
    (§2.A test 10). Guide 12's FRONT FLIPPED row explains both causes. Landmine 3 matches.

**R11 (safety, major): an undeclared mechanism motor is a drive candidate and can be powered up to 9 V; the draft
never stated or tested it. REAL, measured. Fixed by statement, words and tests. The pre-pulse hold is NOT added
(why below).**
- *Verified in the calibrator.* Every motor the registry reports, minus the declared ports, is a candidate and is
  built through the provider. That writes the DRIVE cartridge, then 0 V + Coast. A candidate whose shaft moved
  past 5° in the push is a drive port and is pulsed (`drivetrain_calibrator.hpp` :53-59, :1163-1240). One that
  did not move "is never touched again": it is never powered, though it keeps receiving 0 V + Coast as an idle
  member.
- *Measured* (`amend/run_new/undeclared`). Robot two's plant plus two UNDECLARED motors:
  - Port 8 does not move. It gets one gearing write and 0 V commands only, **max |V| 0.0, never powered**. The
    new DONE line names it.
  - Port 9 is an intake roller the push turns (0.6 of the wheel) and that spins freely when powered. It is
    pulsed **3.0, 6.0 and 9.0 V, 1.19 s in all** (own shaft +242.7°, +630.9°, +1051.5°, yaw 0). It is then
    LEFT OUT as "decoupled", and the run SAVES a correct 10-port record (10 of 10 forward).
  - So the record is safe, but a mechanism is powered next to a person and the words blamed "a slipping gear".
  - A mechanism that cannot spin freely ends ROLLED or STILL (refused), with "is it on the FLOOR?".
- *Fix (§0.4, §1.1, §1.3, §1.4, §2.A, the team file, the chapter):*
  - **The rule is stated everywhere a team reads.** List EVERY motor that is not a drive motor: an unlisted
    motor is a drive candidate, and one the push turns is POWERED at 3, 6 and 9 V like a drive motor. The team
    file's `.mechanisms` comment says so in two lines (§1.1; the file is now 21 lines).
  - **The words.**
    - With nothing declared, the banner says so in three lines: `no mechanism port declared -- … EVERY motor is
      a drive candidate: one that turns with the push is POWERED (3, 6, 9 V). Declare every mechanism motor …,
      or unplug it first`.
    - At DONE each candidate that did not turn gets `port 8 did not turn with the push: not a drive motor,
      never powered -- if it is a mechanism, declare it in the robot's facts`.
    - The LEFT OUT line adds "or a MECHANISM motor -- declare it in the robot's facts".
    - A pulse refusal (ROLLED, STILL) on a port adds `if port N is a MECHANISM motor, declare it …`.
    - All measured in `undeclared` and `slip18`. The `slip18` line is classified as reason M.
  - **Tests (§2.A test 5b):** the `undeclared` scenario, asserting port 8's 0 V/Coast and no non-zero command,
    and port 9's three pulse levels and its LEFT OUT line.
  - **G3a (§5):** until its Part 5, a generated robot declares no mechanism (`kNoMechanisms`). Its Calibrate
    prints the "every motor is a drive candidate" banner, and its bring-up instructions must say to unplug any
    mechanism motor before Calibrate, or Part 2 generates the profile's mechanism ports.
- *Not added: a hold before the pulses on a "moved" candidate whose push travel is far below the drive's.* No
  ratio separates a back-driven mechanism from a drive member. The roller in `undeclared` turned 0.6 of the
  drive, and robot two's own port 18 runs ~20 % short (HA-130). Any ratio would be a new invented threshold in
  the calibrator, which Part A does not touch. It would also miss a mechanism geared to turn MORE than the
  drive. The calibrator already ends such a port safely (LEFT OUT or refused, never in the record, measured).
  The root guard is the declared facts, now stated loudly where a team writes them. Logged in §4 as rejected,
  with this reason.

**R12 (safety, major): the watch's WARNING did not stop the save, so a drive motor declared as a mechanism replaced a
good record with one a motor short. REAL, measured; D4 now HOLDS the save. Re-probed.**
- *Verified.* The draft's `watchReport` only printed, and `finishDone` saved on SAME FRONT whatever the watch
  found. `saveRecordSafely` then refreshes the backup with the new bytes (`record_store.hpp` :891-893), so the
  good record leaves both files.
- *Measured, the draft's path* (`amend/run_new/watch-typo-save`, where the operator chooses SAVE, which is what
  the draft did unasked): the facts declare drive port 12 as "claw pivot 7" (a typo for 7), and the card holds
  run 1's 10-port record. Port 12 turned +1692.4° (the drive turned 1692°). The front check said SAME (9 shared, 1
  only on the card). The save writes **L4/R5, nine ports**, over the good record.
- *Fix (D4 rewritten).* A declared port that turned past `movedThreshold` HOLDS the save on a second held screen.
  `movedThreshold` is the push's own 5°, the same rule that makes a candidate a drive port, so no new threshold
  is added.
  - Header `MECHANISM PORT TURNED`; LARGE `NOTHING SAVED YET`.
  - MEDIUM: the evidence, `port 12: 1692 deg, drive 1692 deg`.
  - SMALL: `the card's record DRIVES with port 12` (when it does; else `it is declared a mechanism: not in this
    record`), `SAVE: it is a mechanism the push turns -- keep it`, amber `RUN AGAIN = save nothing: a drive
    motor? fix facts`.
  - Boxes SAVE and RUN AGAIN. The serial lines say what to do in full (stop, fix the facts or the wiring,
    rebuild, upload).
  - After a FRONT FLIPPED answered USE NEW, this screen follows if a declared port also turned (one question
    per screen).
  - After SAVE, the finished screen's NOTE is `PORT 12 KEPT AS A MECHANISM -- see the log`.
- *Re-probed.*
  - `watch-typo-again`: held, RUN AGAIN, held again, then the field disables. The card's two files are
    **byte-identical to the seed**. Zero replace operations while held is asserted by §2.A test 2 on
    `FakeFileStore`; the harness uses a real directory and cannot count them. The line reads `… on the held
    MECHANISM PORT TURNED screen -- NOTHING SAVED`.
  - `watch` (claw 6 turned on an empty card): held, then SAVE. Saved 10/10, NOTE `PORT 6 KEPT AS A MECHANISM`.
  - Every other scenario: both declared ports "did not turn", and nothing held.
- *Alternatives (D4):*
  - the draft's warn-only, whose cost is measured above;
  - refuse outright: a floor-touching intake could never be calibrated declared;
  - hold only on the card's evidence or on a travel ratio: a new invented ratio (HA-188), and a first record
    with a mistyped port would still save silently.

**R13 (safety, major): a holonomic team could not state its drive kind, so the promised NOT A TANK refusal was
unreachable. REAL; `.drive` is now the config vocabulary. Re-probed.**
- *Verified.* `hal::DrivetrainKind` (the record's, append-only) has only `Tank` (`hal/drivetrain_description.hpp`
  :62-64). The draft typed `CalibrateFacts::drive` with it, so `hal::DrivetrainKind::XDrive` does not compile
  (the checker's `check-safety/kind_probe.cpp`). `config::DrivetrainKind` (Tank, XDrive, HDrive;
  `config/robot_config.hpp` :78-82) is the vocabulary robots state, and its own comment says it is
  "DELIBERATELY NOT `hal::DrivetrainKind`". `calibration/` already includes `config/robot_config.hpp`
  (`drive_characterization.hpp`), so no new layer edge.
- *Fix (§1.1, §1.3 item 3, §9.3).*
  - `CalibrateFacts::drive` is `std::optional<config::DrivetrainKind>`. The program maps Tank to the record's
    `hal::DrivetrainKind::Tank`.
  - Any other kind refuses BY NAME before anything is made: `NOT A TANK -- the facts say x-drive; this procedure
    measures a TANK drivetrain only` / `a holonomic drive's calibration is not built yet -- do not run this on it`.
  - The team file reads `.drive = config::DrivetrainKind::Tank`.
  - Robot two's facts state Tank: a `ChassisTable` is a tank's, and the chain builds Calibrate only for
    `ROBOT=tank` (`build_selection.hpp` :89-90, a compile-time `#error`).
  - §5 tells G3a Part 2: its general form of that guard must keep "Calibrate builds only for a robot whose
    generated kind is Tank", or the generated facts carry the generated kind. Without that, a generated X-drive
    with a stated cartridge would be calibrated as a tank. The reasoned failure the checker names (a SAVED
    "tank" record of an X-drive) is not measured, and the chapter says why an X-drive must not run Calibrate.
- *Re-probed* (`amend/team2/`, `amend/run_new/facts-xdrive`):
  - an X-drive team's file (`.drive = config::DrivetrainKind::XDrive`) COMPILES;
  - the old type is now a compile error naming `std::optional<shulib::config::DrivetrainKind>`;
  - through the program on the plant, the X-drive facts paint `NOT A TANK` and print the line above. The
    refusal precedes every `devices.motor()` call in `run()`, and §2.A test 6 asserts the provider's request
    count is 0. The harness counted 0 writes on the declared ports.

**R14 (safety, minor): after KEEP OLD, the screen and the "copy this line" instruction showed the measurement, not the
record saved. REAL for the draft; dissolved by R2.**
- *Verified.* The draft's finished view painted `cal.result()` (`calibrate_panel.hpp` :841-849), while KEEP OLD
  saved `flipFront(result)`. The plant's equal side scales (1.625 both) hid the swap of L and R.
- *Disposition.* KEEP OLD is withdrawn (R2), so what is saved is always `cal.result()` as measured. The screen,
  the `MEASURED tank …` line and the card now say the same thing by construction. Confirmed: in every `amend/`
  scenario the harness's decoded card line equals the program's `MEASURED` line.
- *Kept from the fix:* §2.A test 1 gains one scenario with UNEQUAL side scales, so a future mutation that swaps
  L and R anywhere between the measurement and the card turns a golden red. The plant has ONE wheel diameter
  (`DrivePlantConfig::driveWheelDiameter`, `sim/drive_plant.hpp` :151). The rig therefore scales one side's
  REPORTED shaft position by robot two's measured ratio (1.36703 / 1.25442 ≈ 1.09). To the program that looks
  like a smaller wheel on that side: the push, the pulses and the spin all read it.

**R15 (safety, minor): the watch's 5° WARNING told a student to "check the facts", which could lead them to un-declare
a real mechanism. REAL; reworded, with the evidence shown. No new threshold.**
- *Verified.* The draft's line was `… it may be a DRIVE motor -- check the robot's facts and the wiring`
  (`proposed2/…/calibrate_program.hpp` :667-670). Removing a declaration makes the motor a candidate, which is
  R11's path to being pulsed.
- *Fix (with R12):*
  - The serial line shows the evidence beside the drive's and never suggests un-declaring: `WARNING: port 6 is
    declared as claw pivot 6 but TURNED WITH THE PUSH (+1692.4 deg; the drive turned 1692 deg) -- the save waits
    for you`.
  - The held screen tells each case apart by what to do: `SAVE: it is a mechanism the push turns -- keep it`
    and `RUN AGAIN = save nothing: a drive motor? fix facts`.
  - The serial says in full: "If it is a mechanism the push turns (an intake on the floor), touch SAVE and keep
    it declared. If it is a DRIVE motor, touch RUN AGAIN … fix the robot's facts or the wiring".
  - All measured in `amend/run_new/watch`, `watch-typo-*`.
- *Not added: a comparison threshold against the drive's median travel.* The numbers are shown side by side for
  the operator to judge (`port 6: 1692 deg, drive 1692 deg`). A ratio rule would be a new invented belief, and
  R11 explains why none separates the cases. HA-188 stays released.

**R16 (safety, minor): a first record (empty or new card) showed no screen note; the "no typed-table check" exclusion
was justified by robot two only. REAL; the NOTE added (re-probed), and the typed-port question goes to the team
lead as D6.**
- *Verified.* §1.4 gave the NOTE only to "does not decode" and CANNOT TELL. The `card :` line is serial only.
- *Fix 1 (re-probed).* A first record's finished screen carries the amber NOTE `FIRST RECORD -- the end you
  pushed first is FRONT` (49 of 50 columns, in `kAll`'s compile-time budget check). It is drawn in every
  empty-card scenario (`amend/run_new/one.screen.log` :255). The serial `front check:` line words the three
  no-check cases apart (§1.4); the draft's "no usable record" for an absent card is gone (`nocard`: `front
  check: NOT DONE -- no SD card in the brain`).
- *Not done: the `card :` line on the starting screen.*
  - The starting screen shows only while the census runs. The IMU-wait view that follows already fills its five
    rows (`calibrate_panel.hpp` ~:740-755: HANDS OFF, IMU WARMING UP, the seconds, the front rule, NEXT) and lasts
    ~2 s (HA-23).
  - The NOTE gives the same fact where the operator can still act (RUN AGAIN), and the serial `card :` line
    gives it at the start.
- *Fix 2 — the typed-table exclusion's justification, re-checked* (`amend/table_probe/`):
  - Robot two's typed table is not a drivable description (`valid()` false: track width and ratio UNSET), so
    its drive never boots from it (R17). The panel never asks drive ports (MECH-PANEL-requirements §4.1).
  - A team exposed would need typed ports AND typed geometry (a valid table), autons written while driving from
    it, and a first Calibrate pushed from the other end. No robot in the tree is exposed today.
  - G3a's D10 still permits typed ports, so the gap is real for such a team. Whether CAL1 closes it is **D6**
    (new, top): recommended (b), state the gap; (a) is a ~¼-day check against the typed sides. **The team lead
    decided (a) the same day** — the "D6 (a)" entry below; it costs ~½ day as designed.
  - §4's OUT entry now covers every robot with typed ports, not only robot two. *(Superseded by D6 (a): §4 now
    states only the residual gap — a robot with neither a record nor typed ports.)*

**R17 (safety, minor): P8 overstates the boot. REAL that P8 overstates; the "runs backwards" consequence is NOT
real for robot two today. Reworded with evidence; B2 ordered.**
- *Verified.* P8's "the boot falls to the backup's EARLIER record" was measured with the library's
  `loadRecord` (`probe_front_save/probe.log` S6). That is not the drive program's boot. The drive program's
  boot reads only the record file (`src/common/boot_source.cpp` :38-40). When the file does not decode it
  takes the typed table only if that table is a valid description (`boot_check.hpp` `chooseBootSource`,
  :420-447).
- *The checker's consequence, re-checked* (`amend/table_probe/`): robot two's typed table is **not** a
  drivable description today. `tableToDescription(kRobotTwoChassis).valid()` is **false** ("LEFT inches per
  radian is 0 -- must be finite and > 0 (0 = UNSET)"): its track width and external ratio are UNSET. So on
  the crash-mid-write path, robot two's drive program **refuses to boot** ("no calibration record on the
  card, and the typed table cannot describe a drivetrain either; run shulib Calibrate"). It does not drive
  backwards behind a TYPED TABLE warning.
  A robot whose typed table IS valid (a G3a D10 team that types ports and geometry) would drive from the
  table. That table's front may differ from the record's, which R16 covers. Today's `"wb"` save is worse on
  this path: no backup at all.
- *Fix.*
  - §0.6 P8 now says "`loadRecord` falls to the backup", followed by the drive boot's real behaviour until
    B2 (above).
  - §5 orders SAFE-SAVE B2 directly after CAL1 Part B, before R5a Part C (build-order already puts B2 after
    COMP-b, which has landed).
  - The risk until then is accepted and stated: a refused boot, not a reversed robot, for robot two.
  - Part B's Station G words say a SAVE UNCONFIRMED run leaves the drive program unable to boot until
    Calibrate saves again or B2 lands.

**R18 (safety, minor): small silent traps in the team file and the facts checks. REAL; fixed and re-probed.**
- *Verified.*
  - `factsProblem` skipped port 0 even for a NAMED entry, so `{0, "intake"}` silently left the intake a candidate.
  - It returned a generic sentence, though §1.1 promised to name the entry.
  - TOO FEW MOTORS said "re-seat the drive cables" even when the shortage came from declaring drive ports as
    mechanisms.
- *Fix (§1.1, §1.3 item 3; re-probed in `amend/run_new/facts-*`):*
  - `factsProblem` returns `FactsProblem {why, entry}`. A NAMED entry whose port is 0 refuses. The refusal names
    the entry. Measured: `REFUSED: FACTS REFUSED -- a mechanism is named but its port is 0 -- state its port:
    mechanisms[2] = {0, "intake"}`.
  - When any declared port was dropped, the TOO FEW MOTORS fix names them. Measured: `declared as mechanisms: 6 7
    11 12 13 14 15 16 17 18 19 -- is any a DRIVE motor? else re-seat the drive cables`.
  - The `.mechanisms` comment is R11's.
  - Robot two's `static_assert` reads `factsProblem(kCalibrateFacts).why == nullptr`. It compiles in the end
    state, where the `src` gate PASSES (P18).
  - Tests §2.A 6 gain the named-port-0 case and the TOO FEW naming. Each refusal still happens before any device
    is requested.

### D6 (a) — the team lead's answer, designed and probed (2026-10-08, afternoon)

> **The answer** (the header block): "Accept, but add D6's check" — D1–D5 as recommended, **D6 (a)**: a FIRST record on
> a robot whose facts carry TYPED drive ports is checked against those typed ports; ruling (1)'s "the previous record"
> becomes "the previous record, else the typed ports". The go to build stands (ruling (2): after T4 Part 3 lands;
> before MECH-A5, the core's N9). This entry is the design and its evidence; the body is amended to match (below).
> Probes in `.claude/coordinator/scratch/brief-CAL1/d6a/`, built from COPIES of `amend/`'s `p3r/`, `p2r/`,
> `harness2.cpp`, `tree_a2/`, `tree_end/` and `team_proj2/`; `amend/` and the repository were only read. **Live
> worktree re-check:** `.claude/worktrees/t4-3` is now committed (`a82b992`, T4 Part 3's transport commit); the eleven
> files D6 (a) reads (`chassis_table.hpp`, `boot_check.hpp`, `drivetrain_description.hpp`, `boot_source.{hpp,cpp}`,
> `robot_two_driver.hpp`, `robot_config.hpp`, `mechanism_table.hpp`, `calibrate_panel.hpp`, `calibrate_program.cpp`,
> `record_store.hpp`) are byte-identical to the probe copy.

**The design (D6; §1.1, §1.4).**
- **The facts:** `CalibrateFacts::typedDrive`, a `TypedDrivePorts {left[21], right[21]}` of SIGNED ports (0 = an
  unused slot), appended LAST and empty by default — the team's 21-line file is unchanged and still compiles and
  links (P31). The spelling is the chassis table's `.left = {-11, 12, -13, 14, -15}`; `typedDrivePortsFrom()` takes a
  `config::RobotConfig` tank's description. `factsProblem` gains the typed lists' checks (outside 1..21, twice by
  |port| on one side or across both, one side only), naming `typedDrive.left[i]`/`.right[i]` through a new
  `FactsProblem::list`.
- **The convention, proved by a probe, not by reading** (P24): `typedDriveDescription(facts)` equals the boot's
  `tableToDescription(table)` sides bit for bit (three robots), and a drive the boot builds from robot two's typed
  table (made valid) puts the same millivolts on all 10 ports, forward and turning, as a drive built from the record
  the standard Calibrate saved with a SAME verdict against those ports — and the opposite forward millivolts on all
  10 for a record the check calls FLIPPED. "SAME as the typed ports" means "the boot would drive the way this push
  measured".
- **The rule:** the card's record (as `loadRecord` chooses it) whenever one decodes — the typed ports then change
  nothing (P23); with no usable record (no file, OR files that do not decode — **decided: the typed ports, because
  the boot treats a record that does not decode as absent and drives from the typed table when it is valid**,
  `boot_check.hpp` :439-442), the typed ports when the facts carry them; else FIRST RECORD. **No card: no check**
  (nothing can be saved). Same verdicts and strict majority.
- **The words:** every line, screen and NOTE that says "the card" says "your typed drive ports" when they were the
  reference; the FRONT FLIPPED screen's causes include "the typed ports are wrong — fix them"; USE NEW / RUN AGAIN
  only; NOTEs `FRONT MATCHES YOUR TYPED DRIVE PORTS` (SAME) and `FIX YOUR TYPED DRIVE PORTS -- see the log` (after
  USE NEW); "the card keeps its record" only when it had one (which also corrects the revision's words on one
  untested path — a MECHANISM PORT TURNED hold discarded on an empty card with no typed ports, §1.4); the MECHANISM
  PORT TURNED evidence names the typed ports when they were the reference. Five new literals in `kAll`, all within
  budget (P26).
- **Robot two:** Part B's facts map `kChassis`'s typed ports (tableToDescription's rule; a signed table only). Its
  typed table is FLIPPED against its real 09-25 record, so its first Calibrate on an EMPTY card holds on FRONT
  FLIPPED when the camera end leads, until G3a's D7 flips the table — USE NEW is right then, and the record it
  saves is SAME as the 09-25 record, 8 of 8 (P24). Station G and the chapter say so (§3.2). With a record on the card
  (its normal case) nothing changes (P23). **SUPERSEDED by the check (C1, below):** that mapping pointed robot two's
  empty-card check at the wrong end — a 15/16-first push saved the reversed front under `FRONT MATCHES YOUR TYPED
  DRIVE PORTS` — and robot two's table is not one the boot can drive from. The facts now take a table's ports only
  when the boot would drive from it (`withTypedDriveFrom`: `tableToDescription(t).valid()`), so robot two carries
  none and its empty card gets the FIRST RECORD note, as in the revision.

**Measured (each in §0.6):** P23 the goldens — the fifteen: 50 of 69 files byte-identical (every record; `nocard`,
`swap`, `watch-typo-*` whole), the other 19 differ only by D6's lines, residual 0; `first-untyped` = the revision's
`one`; `typed-card-wins` = its untyped control. P24 the boot's vocabulary (above). P25 the end state: 143 of 143.
P26 the screen strings: all PASS (36/36, 50/50, 48, 36, 41 of 50; the runtime line ≤ 35). P27 the facts checks: every
`static_assert`, and three refusals through the program. P28 today's program on the D6 panel: 33 of 33. P29 the
host suite: 2,231 of 2,231 (+229 and +62 assertions in two literal/line scans, attributed by a control build). P30
the `src` gate check PASS on the Part A tree and the end state (7 builds each, 0 disallowed), the end state's
self-test OK (211 cases), the ARM header gate on both (214 headers; control fails), robot two's package carries the
typed words and none of the old. P31 the team file: byte-identical, host-clean, V5-linked. **Mutations:** A2's
28–42 and B's 8–10, each applied to the probe's draft and run: **all 18 killed** (`d6a/mut/mutate.log`,
`mutate2.log`, `mutB.log`).

**What it changed in the brief:** the header (the answers; the status); "Decisions" (each D marked DECIDED; D6
rewritten as (a)'s design with (b) the alternative not taken; D3's and D5's notes); the plain words; §0.2-§0.6
(G3a D7/D10, the panel; the typed convention; §0.4's optional row; §0.5's D6 goldens; P23–P31); §1.1 (the facts
type, the typed refusals, the optional line, robot two's mapping), §1.2 (items 5-6, the panel's additions), §1.3
(items 5, 8), §1.4 (the rule, the words, five verdict rows, the screens, the NOTE order), §1.7, §1.8; §2.A (sizes,
the rig, tests 1-3, 6, 10, 11, mutations 28-42, the DoDs), §2.B (size, tests 1-2, mutations 8-10, the DoD); §3.1
(the chapter, guide 12, the changelog), §3.2 (Station G's sentence, the chapter's); §4 (the residual gap; three
rejected alternatives); §5 (MECH-A5 keeps the mapping; G3a D7, D10, the panel; the plan's size); §7 (landmines 4, 5,
21-23); §8 (three findings); §9.1-§9.3. Nothing else changed: D1-D5 stand as decided.

**Sizes:** ~½ builder-day added (A1 +~1/16, A2 +~⅜, B +~⅛) — twice the draft's "~¼ plus tests" (§9.2 says why).
**A1 ~1–1¼, A2 ~1⅞–2⅜, B ~1⅝–2⅛; ~4¾–6 in all with the extra go** (was ~4¼–5½). **SUPERSEDED by the check
(C5):** those ranges sat below their own items; re-derived, A1 ~1¼–1⅜, A2 ~2½–2⅞, B ~1⅞–2⅛, ~6–6½ in all.

**Continuation — every claim above re-verified (2026-10-08, ~14:00–15:45; `d6a/v2/`).** The run that wrote this entry
was stopped by the coordinator's usage limit right after its 13:25 save. A second run re-checked each claim against
its probe, re-running each probe from its source rather than reading its log:
- **The drafts predate every run that cites them** (`p3r/` last edited 12:15, `p2r/` 12:26; goldens 12:19 and 12:26);
  `tree_a3/`, `tree_end3/` and `team_proj3/` carry exactly those files (`cmp`).
- **`harness3.cpp` was edited AFTER the goldens** (13:25: the `watch-typo-empty-untyped` scenario of §1.4). Rebuilt from
  it: Part A's form and the end state each reproduce `run_new/` byte for byte (**143 of 143** each); the classifier's
  totals are unchanged, residual 0; `watch-typo-empty-untyped` reproduces `run_extra/` exactly. P24's full log is
  byte-identical on re-run; P26 and P27 re-compile with the suite's flags and `-Werror` (P27 over both forms); P28
  33 of 33; **all 18 mutations killed again in one pass** (`v2/mutate_rerun.log`; B8 and B9 again at compile time
  in the parity probe).
- **Precision fixed in §0.5/§0.6 (no number changed):** the control's 79 files omit `swap-keep-old-equivalent`'s two
  card copies (`amend/run_new/` kept none); "50 of 69" compares `watch-typo-*`'s stderr with
  `amend/run_typo_counters/`; the `card :` count (14) includes the three `facts-*` refusals outside the fifteen;
  `first-untyped` equals `one` with the card directory normalized and the harness's scenario-name line aside; the
  mutation logs name B's 8–10 MB9–MB11.
- **One claim had not been measured as written, and now is:** the chapter's
  `typedDrivePortsFrom(kRobot.drivetrain.tank.description)` had been compiled with a bare `hal::DrivetrainDescription`
  (`team3/main_config.cpp`). `v2/team_config_real.cpp` spells it over a real `inline constexpr config::RobotConfig`
  tank: host-clean (suite flags, `-Werror`) over both forms, and a V5 package as a project's only source (P31). §1.1
  and test 11 now cite it, and test 11 types a real `RobotConfig`.
- **T4 Part 3 LANDED during this run** — `bdcbc7c` on `shulib-v2`, 2026-10-08 14:52. Every Calibrate file this brief
  cites, the eleven D6 (a) reads, the header's unchanged list and the tests CAL1 re-pins are byte-identical to the
  probe copy (`cmp`, the main checkout). The landed tree differs from the copy only in T4's fix-run files: seven
  library headers (`auton_registry.hpp`, `fault.hpp`, `log_copy_sink.hpp`, `loop_monitor.hpp`, `device_provider.hpp`,
  `motion_scheduler.hpp`, `coupled_side_monitor.hpp`), seven `src/` files and fourteen tests, none Calibrate's — five
  of those headers and four of the `src/` files changed again after the 09:25 overlay (`recheck_1008/`). §5's
  protocol therefore asks no re-run, but **P32 re-ran the proofs on the landed tip anyway**: the goldens 143 of 143
  on both forms, today's program 33 of 33, the whole suite (2,253 of 2,253 — T4's fix run added 22 cases — with every
  assertion change in the file-walking scans, against a control build of the landed tree), the `src` gate PASS on
  both (7 builds, 0 disallowed), the end state's self-test OK (211 cases), the ARM header gate on both (214), and
  robot two's package with the typed words — every one passing.
- **What the continuation changed in the brief:** the header's probe list (`d6a/v2/`); §0.5 (the precision above, the
  re-run line); §0.6 (the title; P23–P31's re-verified results; P32 new); §1.1 (the config form's citation); §2.A
  (test 11's real `RobotConfig`; the mutations' re-run log and names); §2.B (the MB names); §5 (T4 Part 3 landed);
  this paragraph. No design, decision, word or size changed.

**Not settled here (for the coordinator / the team lead)** — **SUPERSEDED by the check's list (below): #1's numbers
were below A2's own items, and #3 no longer holds (D7 alone does not give robot two the typed check):**
1. **A2's upper end (~2⅜) is past the ~2-day line** D5 used to split Part A. Kept in A2 (the check lives in
   `finishDone`; a fourth part costs ~¼ to save ~⅜); the team lead may still ask for the split.
2. **MECH-A5 must keep the typed-port mapping** when it rewrites `calibrateFactsFor` (§5) — the core's brief does not
   say so yet; the coordinator carries it.
3. **G3a's D7 and CAL1 Part B: whichever lands second** rewords Station G's and the chapter's robot-two sentence (§5).
   If D7 lands first, robot two's empty-card camera-end push gives SAME and the sentence is not needed.
4. **A typed drive port that is also a declared mechanism** is not refused by the facts (§8) — not D6's; the watch
   holds on it, with the typed ports as evidence.

### The check of D6 (a) — five findings, each re-verified (2026-10-08, evening)

> One checker read the D6 (a) amendment and ran its own probes (`check-d6a/`). Its five findings are taken below
> one by one: each re-verified against the checker's evidence and the probes, then fixed at the root and
> re-probed. **All five are real.** Probes in `.claude/coordinator/scratch/brief-CAL1/d6a/v3/`: `p3r/` and `p2r/`
> (the two library forms, copies of `d6a/`'s with the fixes; `patch_v3.py` applies C3/C4 to both),
> `harness4.cpp` (`harness3.cpp` + the check's scenarios), `build.sh`, `run.sh`, `cmp.sh`, the runs `run_new/`
> and `run_end/` (43 scenarios), `compare_v3.log`, `classify_v3.log`, `parity_probe/`, `mutate_v3.py` +
> `mutate_v3.log`, and `landed_a3/`, `landed_end3/`, `team_proj4/` (T4 Part 3 as landed, `bdcbc7c`, + the fixes;
> scratch gits `230ccf2`, `a83f659`) with `gates_v3.sh` and its logs. `amend/`, `check-d6a/` and `d6a/` outside
> `v3/` were only read; the repository was only read. **What changes for the team lead:** robot two's Calibrate no
> longer checks its front against its typed table — no boot can drive from that table — so its empty card gets the
> revision's FIRST RECORD note (C1); and the sizes, re-derived from their own items, are **A1 ~1¼–1⅜, A2 ~2½–2⅞,
> B ~1⅞–2⅛, ~6–6½ in all** (C5). A2 is past D5's ~2-day line at both ends, so its split goes to him (not settled #1).

**C1 (major): robot two's typed table pointed its empty-card check at the wrong end; Station G's sentence contradicted
itself; §1.7 and §5 were false. REAL; fixed at the root — the facts take a table's ports only when the boot would
drive from it — and re-probed.**
- *Verified.*
  - Robot two's table types the 15/16 end as its front (`src/robots/chassis_table.hpp` :93-98, :136:
    "front = the 15/16 end"). Its 2026-09-25 record and its driver use the camera end (`src/team/robot_two_driver.hpp`
    :16-17: "Forward is the end the calibration record calls the front (the camera end, …)").
  - Under the D6 (a) draft's mapping, on an EMPTY card a 15/16-first push is SAME against the typed ports and saves
    with `FRONT MATCHES YOUR TYPED DRIVE PORTS` (`d6a/run_new/one.screen.log`; `typed-flip-again` the same after RUN
    AGAIN). That record is FLIPPED 8 of 8 against the real 09-25 record (`d6a/boot_probe/boot_probe.log` :52, :55).
    The push the brief called right (camera end first) is the one that held. The check rewarded the wrong push and
    stopped the right one.
  - Station G's planned "Touch RUN AGAIN only if the 15/16 end led by mistake" can never apply: a 15/16-led push
    gives SAME and never shows FRONT FLIPPED. The sentence also left out the 15/16-first case.
  - §1.7 ("exactly as the boot drives only from a signed table") and §5 ("the same rule the boot uses (it drives only
    from a signed, valid table)") were false. The mapping tested only `signsMeasured`. The boot drives from the table
    only when `tableToDescription(kChassis).valid()` (`boot_check.hpp` :403, :439-442; `src/common/boot_source.hpp`
    :498). Robot two's is not valid ("LEFT inches per radian is 0", P21). So no boot drives robot two from its table,
    no auton is tuned on it, and the hazard D6 (a) closes cannot arise on robot two.
- *The root fix.* The reference is the table the boot would drive from, decided by the boot's own function.
  - `calibrateFactsFor` (constexpr, under the `static_assert`) maps no drive ports any more.
  - A new `withTypedDriveFrom(facts, table)` in `src/robots/calibrate_facts.hpp` sets `typedDrive` from
    `tableToDescription(table)` when it is `valid()`, and to none otherwise.
  - The glue runs `hal::pros::runCalibrate(withTypedDriveFrom(kCalibrateFacts, kChassis), kBuildStamp)` (27 lines).
  - No copy of the boot's rule remains, so landmine 22's drift risk is gone and "exactly as the boot" is literally
    true.
  - Robot two, the bench bot and the X-drive carry NO typed ports. A robot whose table becomes valid gets the typed
    check with no Calibrate line written: G3a D10's generated robot, or robot two once its track width and ratio are
    typed (HA-125, HA-126).
  - A team's own `.typedDrive` is unchanged (it is the team's statement, §1.1).
- *Options weighed.*
  - The checker's (b), G3a's D7 before Part B, fixes neither §1.7 nor §5. D7 flips the ports, not the geometry, so the
    check would still run against a table nothing drives from.
  - Its (c), documents only, leaves the check misdirecting robot two.
  - Its (a) is this fix. It needs no new decision: D6 (a) covers "a robot whose facts carry typed drive ports", and
    which tables become facts was this brief's design, made after his answer (the D6 (a) entry).
- *Measured (`d6a/v3/`).*
  - **Robot two's runs are the revision's again.** Its 19 untyped scenarios against `amend/run_new/`: of the 81
    files with a counterpart, 80 are byte-identical (`watch-typo-*`'s stderr compared with
    `amend/run_typo_counters/`, as in D6 (a)'s control). The 81st, `nocard.serial.log`, differs by C3's one `card :`
    line. `swap-keep-old-equivalent`'s two card copies have no counterpart, as before. `first-untyped` equals `one`
    (the scenario-name line aside).
  - **The side-by-side proof against today's program:** residual 0 in all seven comparable scenarios, every count
    equal to `amend/classify2.log` (`v3/classify_v3.log`).
  - **The typed scenarios.** They now type their ports. `typed-first` is d6a's `one`, typed through
    `withTypedDriveFrom` over robot two's table made valid (P24's numbers). Against `d6a/run_new/`, 56 of 60 files are
    byte-identical; the other 4 are `typed-first`'s name line, `typed-undecodable(-flip)`'s reason lines (C3) and
    `typed-watch-typo`'s difference lines (C4).
  - **The parity probe** (`v3/parity_probe/`, the suite's flags and `-Werror`, over both forms). Every RobotId's table
    was checked, and robot two's table made valid and then broken one fact at a time (track width, ratio, wheel, IMU
    unset, IMU on a drive port, cartridge unset, unsigned, a port on both sides). `withTypedDriveFrom` gives
    `tableToDescription`'s sides exactly when `valid()`, and none otherwise; every result passes `factsProblem`.
    **PASS, 12 of 12.**
  - **Mutations** (`mutate_v3.log`):
    - B8, the D6 (a) draft's mapping (validity ignored), is killed by 14 of robot two's goldens and the parity probe.
    - B9 (sides swapped) and B10 (the mapping dropped) are killed by 8 typed goldens and the parity probe.
    - B11 (the glue skips the mapping) survives every golden. On robot two it is equivalent (its table is not valid),
      so Part B test 4's source pin kills it.

**C2 (minor): "the discard words follow the card, not the reference" had no golden. REAL; fixed and re-probed.**
- *Verified.*
  - Only `watch-typo-empty-untyped` pinned the rule, and it was not among test 10's 26 scenarios, nor named by tests
    2 or 5.
  - The checker's X1 (the words keyed on `typedRef`) changes no file of the 33 goldens (`check-d6a/mutate_extra.log`).
  - The scenario's name matched `run_scenarios.sh`'s `watch-typo-*` seeding pattern. Its card stayed empty only
    because `d6a/run_extra/` held no `one` record, so the `cp` failed silently.
- *Fix.*
  - The scenario is renamed `empty-watch-untyped`, which no seeding pattern matches.
  - It is a golden of test 10 and a case of test 5; mutation 43 (X1) is added.
  - `v3/run.sh` now REFUSES to run a seeded scenario without its seed (`NO SEED -- not run`), so a seeded card can
    never turn into an empty one by accident. With C1, robot two's own facts are untyped, so this IS robot two's
    empty-card discard.
- *Measured.* `empty-watch-untyped` reproduces `d6a/run_extra/watch-typo-empty-untyped` byte for byte (the name line
  aside). Mutation 43 is killed by it and by no other scenario.

**C3 (minor): an undecodable card holding only a backup printed an empty reason; a discard with NO card spoke of the
card. REAL; fixed and re-probed.**
- *Verified.*
  - `check-d6a/run_prec/bakonly-garbage-one.serial.log` prints `front check: the card's record does not decode () --
    …`. The draft printed `earlier.primary.why`, which is empty when only the backup opened. The revision's untyped
    NOT DONE branch did the same.
  - **Also found here:**
    - An EMPTY record file (opened, 0 bytes: a crash mid-write) gives an empty reason too. `RecordFileRead::why` is ""
      unless the file held bytes (`record_store.hpp` :618-619).
    - A reason may be 191 bytes long (`kFileReasonBytes`). Inside a ~150-character line, it could be cut by the
      240-byte line buffer.
  - With no card, a MECHANISM PORT TURNED discard said `the card is unchanged (it holds no usable record)`
    (`check-d6a/conv2/nocard-typo-again.serial.log`).
  - With no card, the `card :` line at start said `no usable record yet` — words of the kind landmine 5 forbids
    (no card ≠ no record).
- *Fix.*
  - The does-not-decode lines, typed and untyped, no longer carry the reason themselves.
  - Below them comes one line per file that OPENED and did not decode, naming it:
    `  shulib_drivetrain.bak: magic is not "SHDC" -- this is not a calibration record`, or
    `  shulib_drivetrain.bin: EMPTY (0 bytes read)`.
  - With no card, the `card :` line says `NO SD CARD in the brain -- nothing can be saved, and the FRONT is not
    checked`.
  - With no card, a discarded held run says `NOTHING SAVED: … there is no SD card in the brain`.
- *Measured* (new goldens):
  - `bakonly-garbage-untyped`: NOT DONE, the `.bak` line, NOTE `FRONT NOT CHECKED`.
  - `bakonly-garbage-typed`: the `.bak` line, then SAME against the typed ports.
  - `undecodable-untyped`: both files' lines.
  - `emptyfile-untyped`: `EMPTY (0 bytes read)`.
  - `nocard-watch-again`: both discards name the missing card.
  - `typed-nocard`: equal to `nocard` but for its name line — no card, no check, typed or not.
  - Mutations 44–49 are each killed: 44 (X3, typed), 45 (the untyped FIRST decided from the record file alone),
    46 (the backup's line skipped), 47 (an EMPTY file given the decoder's empty reason), 48 (the no-card discard
    words), 49 (the no-card `card :` line).

**C4 (minor): `FRONT MATCHES YOUR TYPED DRIVE PORTS` was painted when some typed ports disagreed, and guide 12 said
"nothing to do". REAL; fixed and re-probed.**
- *Verified.* `check-d6a/conv_run.log` shows four cases, each SAME with the NOTE
  `FRONT MATCHES YOUR TYPED DRIVE PORTS` (`check-d6a/conv/w0-onewrongsign-front.screen.log` :255):
  - one typed sign wrong (9 agree, 1 sign only);
  - two typed signs wrong;
  - 3 of 10 typed from the other end;
  - a side short a motor.

  A typed port with a wrong sign fights its side-mates in anything that still drives from the typed ports.
- *Fix.*
  - `FRONT MATCHES YOUR TYPED DRIVE PORTS` only when every port in either list agrees: no sign-only, side-only or
    flipped port, and none in one list only.
  - Otherwise the NOTE is `FRONT MATCHES, SOME TYPED PORTS DIFFER -- see log` (49 of 50, in `kAll`). The serial log
    names each port, under `front check: SAME FRONT, but your typed drive ports and this push differ at N port(s) --
    check each before anything drives from the typed ports:`:
    - `  port 12: typed LEFT -12, this push LEFT +12 -- the SIGN differs`;
    - `… -- the SIDE differs`;
    - `… -- the side AND the sign differ`;
    - `  port 9: typed RIGHT +9, not in this push's record (LEFT OUT, declared, or did not turn)`;
    - `  port 16: this push RIGHT +16, not in your typed drive ports`.
  - Guide 12's row is reworded, and the new NOTE gets its own row.
  - Against the card's record nothing changes: that record measured this robot, and the boot uses the new one.
- *Measured.*
  - `typed-partial` (12's sign wrong, 16 not typed, 9 typed but not a drive motor): SAME — 8 agree, 1 sign only, 1 in
    each list only — then the three lines and the new NOTE.
  - `typed-watch-typo` now names port 12 (typed, but declared).
  - Mutations 50 (the D6 (a) draft's rule), 51 (the lines dropped) and 52 (a sign-only port called a side
    difference) are each killed.
  - **One more from the checker's own log:** X2 (`hasTypedDrive` reads the LEFT side only) survived its run, because
    `facts-typed-oneside` types only the LEFT. A RIGHT-only list then skips `factsProblem`'s typed checks and runs as
    untyped. `facts-typed-rightonly` (refused: `the LEFT is empty: typedDrive.left[0] = 0`) is added to test 6, and
    mutation 53 (X2) is killed by it. Its X4 is mutation 54 (killed by `typed-tie`, `typed-nocommon`).

**C5 (minor): A2's stated size was below the sum of its own items. REAL; every part re-derived.**
- *Verified.*
  - A2: §2.A's items are ½ + ¼ + ¼ + ½ + ¼ + ¼–½ = **2–2¼** before D6 (a) (stated ~1½–2). Plus D6 (a)'s ⅜ that is
    2⅜–2⅝ (stated ~1⅞–2⅜).
  - A1, checked the same way: ¼ + ¼ + ⅛ + ⅛ + ¼ + ¼ = 1¼, plus 1/16 = **1 5/16**, above its stated ~1–1¼. Its
    "inside the range" was false too.
  - B: ¼–⅓ + ½ + ¼ + ½ + ⅓ + ⅛ = **~2** (1 23/24–2 1/24), inside its stated ~1⅝–2⅛.
- *Fix: each part's range is now its own items' sum, rounded outward to the eighth.*
  - **A1 ~1¼–1⅜.**
  - **A2 ~2½–2⅞:** 2–2¼, plus D6 (a)'s ⅜, plus this check's ~3/16 — C2–C4's code ~1/16, their goldens and cases
    ~1/16, mutations 43–54 ~1/16 — is 2 9/16–2 13/16.
  - **B ~1⅞–2⅛.** C1 changes what B's D6 item holds, not its ⅛: a 4-line mapping instead of a copy, a source pin, a
    simpler Station G sentence, and the chapter's robot-two sentence gone.
  - With the extra go's ~¼: **~6–6½ builder-days in all** (the parts' rounded ranges add to ~5⅞–6⅝). Was ~4¾–6.
- A2 is now past D5's ~2-day line at both ends, with or without D6 (a) — not settled #1.

**The proofs re-run on T4 Part 3 as landed with the fixes** (`d6a/v3/`, §0.6 P33–P37) — every one passing:
- the end state equals the Part A form in all 43 scenarios (185 of 185 files, `compare_v3.log`);
- the ARM header gate over all 214 headers on both trees (its `int x = d;` control fails);
- the `src` gate `check` on both, all 7 builds, 0 disallowed warnings (`gate_check_landed_a3.log`,
  `gate_check_landed_end3.log`);
- `make ROBOT=tank PROGRAM=calibrate` on the end state links (text 80.55 KB; 8 warnings, all vendored `liblvgl`), and
  the package carries the check's words and none of the old question's;
- the 21-line team file, byte-identical, links a V5 package as a project's only source (`team_proj4/`);
- the WHOLE host suite on the Part A form: 2,253 of 2,253 cases, every assertion change in two literal/line scans
  (+52, +30), no verdict changed;
- today's program on the fixed panel: 33 of 33 files byte-identical;
- the end state's `src` gate `self-test`: **OK (211 detector cases, each against a real build), 11 m 52 s**
  (`gate_selftest_end3.log`; the first run was cut off by the usage stop and left one plant in `build_selection.hpp`
  — restored with `git checkout` and the whole self-test re-run; the tree is clean after it,
  `gate_selftest_end3.after_status` is empty).

**What the check changed in the brief:**
- the header's probe list;
- the Re-check intro and the D6 (a) entry, where its robot-two design, its sizes and its "not settled" are marked
  superseded;
- this entry;
- "Decisions": D5's and D6's notes;
- the plain words;
- §0.2, §0.4, §0.5 (the check's runs), §0.6 (P33–P37);
- §1.1 (the facts' mapping, the glue), §1.2, §1.3 item 8, §1.4 (the no-card and undecodable words, the exact and
  partial NOTEs, the discard words, the verdict rows, the NOTE order), §1.7, §1.8;
- §2.A (the sizes, tests 1, 2, 5, 6 and 10, mutations 43–54, the DoD) and §2.B (the size, tests 1, 2 and 4,
  mutations 8–11);
- §3.1 (guide 12), §3.2 (Station G's sentence, the box, the chapter's robot-two sentence withdrawn);
- §4, §5, §7 (landmines 5, 22, 23), §8, §9.

**Not settled (supersedes the D6 (a) entry's list):**
1. **A2 (~2½–2⅞) is past the ~2-day line D5 used to split Part A, at both ends.** For the team lead:
   - (i) keep A2 as one go, D5 (d) as decided;
   - (ii) split it in two. The extra go adds ~¼, so ~6¼–6¾ in all.
     - **A2a, the program with no typed reference (~1¾–1⅞):** the move onto the seams, the shell, the panel's
       additions and held screens, the watch, the facts checks, the rig, tests 1, 4–6, 8b and 9 and their mutations,
       and C2's and C3's untyped words.
     - **A2b, D6 (a)'s typed reference and the proof through the real PROS shell (~¾–1):** the typed facts and
       words, C4, the golden test's 34 scenarios, the team file's example test, the chapter, guide 12, the changelog
       and HA-187.

   By D5's own line, (ii). The team lead decides.
2. **MECH-A5** (the coordinator carries it): its rewrite of `calibrateFactsFor` touches the mechanism half only. The
   typed half is now the glue's `withTypedDriveFrom(kCalibrateFacts, kChassis)`, which it must leave in place (Part B
   test 4's source pin; mutation B11).
3. **G3a's D7 alone no longer changes robot two's Calibrate.** Robot two gets the typed check only once its table is
   a VALID description: D7's flip AND the track width and ratio (HA-125, HA-126). Until then Station G's sentence
   (§3.2) holds. Whichever change makes the table valid rewords it. If the geometry were typed before D7, the check
   would compare against the 15/16 end — the end the boot would then drive as forward — so D7 should land first or
   with it (§5).
4. (Unchanged) a typed drive port that is also a declared mechanism is not refused by the facts (§8).

**Continuation — the fix step finished, every disposition re-verified (2026-10-08, ~19:50–20:40; `d6a/v3/recheck/`).**
The run that wrote C1–C5 was stopped by the coordinator's usage limit at ~18:49, mid-way through the end state's
`self-test` and before §7 and §9 were amended. A second run re-checked each finding against the checker's evidence
and each fix against its probe, re-running the probes rather than reading their logs, then finished the brief:
- **C1 — REAL, the fix holds.** Re-read: `chassis_table.hpp` :93-98 and :136 (front = the 15/16 end);
  `robot_two_driver.hpp` :16-17 (forward = the camera end, the 09-25 record); `d6a/run_new/one` (empty card, typed
  ports) SAME 10 of 10 and painted `FRONT MATCHES YOUR TYPED DRIVE PORTS`; `boot_probe.log` step 5, `one` and
  `typed-flip-again` FLIPPED 8 of 8 against the 09-25 record; `boot_check.hpp` and `boot_source.hpp` :498 drive from
  a table only when `valid()`. The fix: `landed_end3/src/robots/calibrate_facts.hpp` :65-69 calls the boot's own
  `tableToDescription(c).valid()`; the glue calls it (:23). `v3/run_new/one` now prints `this is the FIRST` and paints
  `FIRST RECORD -- the end you pushed first is FRONT`, and its finished screen carries RUN AGAIN, so Station G's
  recovery (RUN AGAIN, camera end, FRONT FLIPPED against the record just saved, USE NEW) is the `flip-new` path.
  The parity probe's 12 cases print as P34 says. The classifier's counts equal `amend/classify2.log` line for line
  (`diff`). Robot two's `nocard` differs from the revision's only by C3's `card :` line (`diff`).
- **C2 — REAL, the fix holds.** `check-d6a/mutate_extra.log`: X1 changed none of the 33 goldens. The old runner
  (`d6a/run_scenarios.sh` :10-13) copies the seed with an unchecked `cp`, and `d6a/run_extra/` held no `one` record.
  Run against an empty seed directory, `v3/run.sh` prints `watch-typo-again: NO SEED (…) -- not run` and `swap: NO
  SEED …`, and writes no file (`recheck/noseed.log`).
- **C3 — REAL, the fix holds.** `check-d6a/run_prec/bakonly-garbage-one.serial.log` :129 printed `does not decode ()`.
  `check-d6a/conv2/nocard-typo-again.serial.log` :135 and :214 claimed the card's contents with no card. The v3 goldens
  print each opened file's line (`shulib_drivetrain.bak: magic is not "SHDC" …`, `shulib_drivetrain.bin: EMPTY (0
  bytes read)`), and with no card `NO SD CARD in the brain …` and `there is no SD card in the brain`.
- **C4 — REAL, the fix holds.** `check-d6a/conv_run.log` shows the four SAME cases (one or two signs wrong, a side
  short), and `w0-onewrongsign-front.screen.log` :255 shows the NOTE. `v3/run_new/typed-partial` gives SAME, the
  three port lines and `FRONT MATCHES, SOME TYPED PORTS DIFFER -- see log`, 49 columns (counted), and `kAll` lists it
  (`calibrate_panel.hpp` :547 in `v3/p3r/`).
- **C5 — REAL; the sizes stand as re-derived.** Re-added from §2.A's and §2.B's items: A1 1 5/16, A2 2–2¼ → 2⅜–2⅝
  with D6 (a) → 2 9/16–2 13/16 with the check, B 1 23/24–2 1/24. The total ~6–6½ is the outward rounding of the
  unrounded sums plus the extra go (6 1/12–6 5/12); the parts' rounded ranges add to ~5⅞–6⅝. The checker's
  proposed ~2⅜–2⅝ and ~5¼–6¼ omitted A1's own shortfall, B's re-derivation and the check's own ~3/16. **§9.1 and §9.2
  still carried the old sizes**, and are now amended.
- **Re-run, not read.** Both harnesses were rebuilt from `v3/p3r/` and `v3/p2r/`, and all 43 scenarios were re-run:
  **185 of 185 files byte-identical to `v3/run_new/` and to `v3/run_end/`**. All 32 mutations were re-run from
  `mutate_v3.py` (`recheck/mutate_rerun.log`): **every verdict is identical** to `mutate_v3.log` (31 killed by the
  goldens, each by the same scenarios; B11 survives them and fails the source pin; no anchor missed). The suite's
  XML was compared case by case with `d6a/v2/suite_landed_a.xml`: 2,253 cases on both, 0 failures, and only two
  cases changed their count, `calibrate_panel_test` P9 +52 and the retired-Teleop scan +30 (P37). The package size
  (80.55 KB), its 8 warnings (all `liblvgl`) and the team file's byte-identity were re-read from the logs.
- **The interrupted `self-test`.** The stop cut the end state's `self-test` mid-plant. It left case 25's plant (the program and the robot split across two chains) in
  `landed_end3/src/build_selection.hpp`: the `kRobotId` lines moved out of the chain, found by `git status`. The
  plant was restored with `git checkout`, and the whole `self-test` re-run: **OK (211 detector cases, each against a
  real build), 11 m 52 s, the tree clean after it**. P36's and C5's pending marks now carry the result.
- **Completed in the brief by this continuation** (C1–C5's list above claimed them; they were not yet written):
  - §7 landmine 5 (the no-card and backup-only words, mutations 44–49);
  - landmine 22 (the copy is gone: `withTypedDriveFrom` calls the boot's own function; the parity case and the
    source pin stay);
  - landmine 23 (the discard words follow the card's contents; the exact/partial NOTE; seeds are required);
  - D3's note (robot two maps no typed ports);
  - the Decisions intro's old D5 sizes, marked superseded;
  - §1.1's facts file, ~77 lines;
  - §9.1's D3, D5 and D6 rows (the final sizes, the split question, the C1 mapping, the check's evidence);
  - §9.2's heading, its re-derivation block and its three rows (sizes, the 27-line glue, the ~77-line facts file,
    34 goldens, the sixth literal, the parity and source-pin tests, the withdrawn chapter sentence, Station G's
    corrected sentence).
- **Not a finding, and already logged:** the harness's host `-O1` build warns `-Wformat-truncation` at
  `calibrate_panel.hpp`'s `snprintf(out, cap, "%s", buf)`. That code is unchanged from the repository (:643-644),
  and §8 already logs the warning.

## Decisions for the team lead — ALL SIX DECIDED (2026-10-08: D1–D5 as recommended, D6 (a))

> **Decided 2026-10-08** ("Accept, but add D6's check"): D1 (a), D2 (b), D3 (a), D4 (a) and D5 (d) as recommended;
> D6 (a), not the recommended (b). Each D below is marked DECIDED; D6 is rewritten as the design of (a), with (b)
> kept as the alternative not taken and its cost. The text below the marks is the evidence the team lead decided on,
> amended only where D6 (a) forces it (D3's facts type, D5's sizes).
>
> Six questions the rulings leave open. Each carries a recommendation, its measured evidence, and the alternatives
> with their costs. What follows from the rulings without a new choice is listed after them ("Consequences, stated")
> so it is seen, not asked. **Changed by the 2026-10-08 re-check (above):**
> - **D2's recommendation is now (b)**: hold, then USE NEW or RUN AGAIN. KEEP OLD is withdrawn: it amended ruling
>   (1), and on a robot whose drive cables were swapped between the sides it reverses every auton (measured).
> - **D4's recommendation now HOLDS the save** when a declared mechanism motor turned with the push (R12). The draft
>   only warned, and a mistyped drive port silently replaced a good record with one a motor short (measured).
> - **D6 is new**: a first record on a robot that has typed drive ports (R16) — decided (a), the typed-port check.
> - D3's team file is now 21 lines, 5 of them comments. The drive kind is the config vocabulary, so an X-drive
>   is refused by name (R13), and the mechanisms line says what an unlisted motor risks (R11).
> - **D5 now recommends THREE parts.** With the re-check's additions Part A grew to ~2½–3 builder-days, past
>   the ~2-day line, so it splits: A1 (the seams, ~1–1¼), A2 (the program, ~1½–2) and B (robot two, ~1½–2).
>   That is ~4¼–5½ in all (§9.2). *(Sizes SUPERSEDED: with D6 (a) and re-derived from their own items by the check
>   of D6 (a), A1 ~1¼–1⅜, A2 ~2½–2⅞, B ~1⅞–2⅛, ~6–6½ in all — D5 below, §9.2.)*

**D1 — Where a PROS-dependent standard program lives in the library. DECIDED (a), as recommended (the team lead,
2026-10-08): split it — a PROS-free program in `calibration/`, run on the host's fakes, plus a thin PROS shell in
`hal/pros/`.**
- **The constraint.** CI allows `#include "pros/…"` only under `include/shulib/hal/pros/`, path-anchored
  (`.github/workflows/ci.yml` :37-62: "a gate's exclusion list is where its holes live"). Today's program
  names `pros::` 32 times inline — the screen, the touch, the competition status, the delay — and prints with
  `printf` (`calibrate_program.cpp` :283-575, :1010-1019, :1276-1277, :192-195).
- **(a) RECOMMENDED — the split** (§1.2):
  - `calibration/calibrate_program.hpp` — `calibration::CalibrateProgram`: EVERYTHING the program decides (the
    census, the refusals, the calibrator, the touch, the serial record, RUN AGAIN, the front check, the safe save,
    the watch), moved from today's file in its order. It touches the world only through seams: the existing
    `hal::IDeviceProvider` (motors, the IMU, the clock, the pacer), `hal::IDeviceRegistry` (the census) and
    `hal::IFileStore` (the card, SAFE-SAVE Part A); a NEW `hal::IBrainScreen` (the brain's screen and touch: six
    calls); and a four-call `calibration::CalibrateIo` (a serial line, the competition status, a plain wait, the
    watch's raw read).
  - `calibration/calibrate_screen.hpp` — the painter and the program's own screens, moved verbatim from
    `calibrate_program.cpp` :207-575, drawing through `IBrainScreen`.
  - `hal/pros/calibrate.hpp` (~130 lines) — the shell: it composes `ProsDeviceProvider`, `ProsDeviceRegistry`,
    `ProsFileStore`, `ProsBrainScreen` and a `ProsCalibrateIo`, and gives a team five one-line entry functions.
  - **Why:** the CI rule stays as it is; and the whole program runs in the host suite on `hal::fake` devices and the
    A2 plant — the painter's repaint rules, the touch arming, the held save, the flip — where today only the
    PROS-free pieces it calls (`viewFor`, `TouchGate`, the calibrator) are host-tested.
  - **Measured** (§0.6): the real `make ROBOT=tank PROGRAM=calibrate` links robot two's Calibrate built this way;
    the real `src_build_gate.py check` PASSES all 7 builds, 0 disallowed warnings, once its program table names
    Calibrate's three new entry functions; the ARM header gate passes over all 214 headers; a team project whose
    ONLY source is the team file (§1.1; 21 lines since the re-check) compiles and links a V5 package
    (`team_proj/`; re-linked P19); with Part A's form of it in, the WHOLE host suite — 2,231 cases — passes
    unchanged (P13; re-run for the revision with the shim pieces in place, P16), and in the end state 206 of the
    210 test TUs compile untouched, the other 4 being C1's re-pins (P11; re-measured for the revision as P18: the
    same 4, plus `dropped_autons_test`, which fails on the probe tree's mismatched T4 pair, not on CAL1); the
    fence test and SAFE-SAVE's platform scan pass (P12).
    Today the census filter's call site cannot even be compiled on the host: `test/mechanism_table_test.cpp`
    :786-861 pins it as TEXT because "runCalibrate() is a whole PROS program … it cannot be compiled, let alone run,
    here"; under (a) it RUNS in the suite.
- **(b) The whole program as one file under `hal/pros/`.** Cost: `hal/pros/` is the adapter tree — each file
  adapts one device and the fence test reads every one (`test/pros_adapter_fence_test.cpp` :71-120); a 1,000-line
  program there is host-testable only through the PROS shim, which has no screen at all today (no `pros/screen.h`
  in `test/pros_shim/pros/`), so its painter and touch would stay untested or need the shim's growth anyway.
- **(c) A new `include/shulib/programs/` folder that may include PROS.** Cost: a second CI exemption — exactly the
  widening the CI comment warns against — and the same shim-only testing as (b).

**D2 — When the card's record calls the end that led this push its BACK (REWRITTEN by Re-check R2/R10).
DECIDED (b), as recommended (the team lead, 2026-10-08); under D6 (a) the same hold, with its own words, applies
when the TYPED ports are the reference (§1.4). RECOMMENDED (b): the ruling's own letter. The save WAITS on a FRONT FLIPPED screen with two big choices, USE NEW
(save as pushed) and RUN AGAIN (save nothing). Nothing is written until one is touched. The draft recommended (a),
KEEP OLD; that is withdrawn.**
- **The check** (§1.4): before anything is saved, the program reads the card's record the way the boot will choose
  it (`calibration::loadRecord`: the record file, else its backup) and compares, port by port, the drive ports
  BOTH records list: same side and same sign = agree; the other side AND the other sign = flipped. FRONT FLIPPED when
  the flipped ports are a strict majority; SAME FRONT when the agreeing ones are; otherwise CANNOT TELL (no port in
  common, or a tie) — saved as measured, with an amber line to check the robot drives the right way. The record
  carries no "front" field and needs none: its signs and sides ARE the front.
- **What FLIPPED can and cannot mean (R10, measured).** The records cannot tell "the BACK led by mistake" from
  "the FRONT led and the LEFT and RIGHT drive cables were swapped between the sides". On a mirror-mounted tank both
  give the same signed record. Through the program on the plant, the swapped robot pushed camera-first measures
  LEFT −16 +17 −18 +19 −20 | RIGHT +11 −12 +13 −14 +15, exactly the other-end push's record (`amend/run_new/swap`,
  `flip-new`). Only the operator knows which happened. So the screen states what the records say, never what the
  operator did, and ties each touch to one cause.
- **(b) RECOMMENDED — hold, then USE NEW or RUN AGAIN.**
  - USE NEW saves the run AS PUSHED. The end that led is the front (ruling (1)): right after a cable swap or a
    deliberate change of front, wrong after a wrong push. RUN AGAIN discards the run and starts a new one, right
    after a wrong push. A field disable or a power-off while it waits writes NOTHING.
  - The screen (drawn, `amend/run_new/swap.screen.log`): header `FRONT FLIPPED`; LARGE `NOTHING SAVED YET`;
    MEDIUM `the card calls the end you led BACK`; SMALL `RUN AGAIN: you led with the BACK by mistake` / `USE NEW:
    you led with the FRONT (cables swapped?)` / amber `USE NEW after a wrong push reverses every auton`; boxes USE
    NEW and RUN AGAIN.
  - The front is never asked, and every saved record's front is the end that led its push.
  - **Measured** (§0.5, `amend/`): today's program, pushed with the other end leading and answered FRONT FIRST,
    SAVES the flipped record silently (`final_old/flip-keep`). The revised program holds:
    - `flip-again`: RUN AGAIN, then a front-led run 3 → SAME FRONT, saved, **10 of 10** motors forward;
    - `swap`: USE NEW → **10 of 10**;
    - `flip-new`: USE NEW after the wrong push → 0 of 10, the case the amber line warns of;
    - `flip-disable`: both files byte-identical to before.
  - Thirteen record pairs (`probe_front_save/`) give the verdicts the rule says. Robot two's REAL 2026-09-25
    record against its typed table is included (FRONT FLIPPED, 8 of 8 — G3a's D7 finding, reproduced).
  - Cost: a mistaken push costs a whole second run (~56 s on the plant's timing, ~45 s of it powered pulses and
    spins, plus a second push).
- **(a) The draft's KEEP OLD (save the measurement turned to the card's front, `flipFront`) — only as an explicit
  amendment of ruling (1).** It departs from the ruling twice: a saved front that TRAILED the push, and a choice of
  front on the brain's screen.
  - Measured cost (R10): on a correctly pushed robot whose drive cables were swapped between the sides, what KEEP
    OLD would save drives **0 of 10** motors forward (`amend/run_new/swap-keep-old-equivalent`). The checker's
    `check-safety/swap_probe` gives −8/8 on robot two's real record.
  - Even with words tied to the operator's action, it adds a reversing choice to a screen whose other choice
    (RUN AGAIN) already recovers from the only case KEEP OLD helps. Not recommended.
- **(c) Warn after saving.** Cost: by then the old record is gone — the safe save refreshes the backup with the new
  bytes (SAFE-SAVE step 3) — so recovery is a rerun, and an operator who misses the warning reverses every auton:
  the hazard the ruling exists to stop.

**D3 — What a team writes, and how robot two states its facts. DECIDED (a), as recommended (the team lead,
2026-10-08). RECOMMENDED (a): one `CalibrateFacts` with three fields a team fills (drive kind, cartridge, mechanism
ports) and five one-line entry points (§1.1, 21 lines verbatim, compiled for the host and the V5); robot two's facts
are DERIVED from its two tables, never typed twice.** *(D6 (a) adds one OPTIONAL member, `typedDrive`, after the
others: the team file does not name it and stays byte-identical — re-compiled and re-linked, §0.6 P31 — and robot
two's facts map it from `kChassis`, §1.1.)* *(The check of D6 (a), C1: a repository robot's facts take its table's
drive ports only when the boot could drive from that table — `withTypedDriveFrom(kCalibrateFacts, kChassis)` in the
glue, `tableToDescription(kChassis)` when `valid()`. Robot two's table is not valid (P21), so its facts carry none;
the team file is still unchanged — P36's `team_proj4/`.)*
- **(a) RECOMMENDED.** The team's file is §1.1, verbatim: **21 lines, five of them comments** (Re-check: the
  draft's 19 lines plus the two-line rule on `.mechanisms`, R11). Every fact a person states is on its own line.
  The kind is `config::DrivetrainKind`, so an X-drive is refused by name (R13). An unset kind or cartridge REFUSES
  before anything is powered (no default is invented). Re-linked as a V5 package's only source and compiled on
  the host with `-Werror` (`amend/team_proj2/`, `amend/team2/`).
  Robot two's `src/robots/calibrate_facts.hpp` maps `kChassis` and `kMechanisms` into the same struct, so a
  generated robot (G3a Part 2) gets its Calibrate from its tables with no Calibrate line written for it.
- **(b) One macro, `SHULIB_CALIBRATE_PROGRAM(kFacts)`, that defines all five entry points.** 9 lines instead of
  21. Cost: it hides PROS's five entry points, which every other program a team writes spells out; a student who
  reads the file cannot see what runs when.
- **(c) Robot two states its facts literally, like any team.** Cost: its cartridge and its claw ports are then
  typed twice (the tables and the facts) and can drift; a `static_assert` could pin them equal, but that is a
  second copy kept honest by a check rather than one copy.

**D4 — Calibrate's WATCH of the declared mechanism ports (G3a §5 item 7): IN CAL1 (AMENDED by Re-check R12/R15/R6).
DECIDED (a), as recommended (the team lead, 2026-10-08). RECOMMENDED (a): the standard program reads every declared mechanism motor at the start and the end of the push,
never commanding it. If any turned with the push, the SAVE WAITS on a MECHANISM PORT TURNED screen: SAVE (keep it
declared; the record has no such port) or RUN AGAIN (save nothing).**
- **Why it is needed** (G3a re-check #10): with no typed drive ports (G3a D10 (a)), a mechanism role wired to a
  real drive motor passes every build check; Calibrate drops that port as declared (`calibrate_program.cpp`
  :1129-1131), the record never carries it, and the robot drives one motor short while that motor answers the
  mechanism's buttons. The watch is the only catch.
- **(a) RECOMMENDED — in CAL1, and it HOLDS.**
  - The read writes nothing. `CalibrateIo::readShaftDegrees` uses PROS's C API by port (`motor_get_position`,
    `motor_get_encoder_units`, `motor_get_gearing`), with no motor object of any kind (R6, the MECH core's design).
    The vendored reads call only firmware getters (disassembled). There is no brake mode and no command. Its
    belief is **HA-187** (§3.3).
  - "Turned" is the push's own `movedThreshold`, the rule that makes a candidate a drive port, so no new
    threshold is added.
  - A port that turned **holds the save**. The screen shows the evidence, `port 12: 1692 deg, drive 1692 deg`,
    and, when true, `the card's record DRIVES with port 12`. The words never suggest un-declaring a mechanism
    (R15).
  - **Measured** (`amend/run_new/`):
    - `watch-typo-save`: drive port 12 declared as a claw, the card holding a 10-port record. The draft's
      warn-only design would have saved **L4/R5 over the good record, both files**.
    - `watch-typo-again`: held, RUN AGAIN, held, field disable → **the card byte-identical to before**.
    - `watch`: a claw the push turned, SAVE → saved, NOTE `PORT 6 KEPT AS A MECHANISM`.
    - Every other scenario: "did not turn", nothing held. The shim counts ZERO gearing, units, voltage and
      brake-mode writes to ports 6 and 7 in every run.
- **(a′) The draft: the watch decides nothing (a WARNING line and a NOTE).** Cost: measured above. A mistyped
  drive port silently replaces a good record with one a motor short, and the claw's buttons then drive a wheel.
- **(a″) Hold only on strong evidence** (the card's record drives with the port, or its turn is at least half the
  drive's). Cost: a new invented ratio (HA-188). A first record with a mistyped port would still save silently.
- **(a‴) Refuse outright.** Cost: a mechanism the push turns (an intake on the tile) could never be calibrated
  while declared. The team would have to lift it or unplug it every time.
- **(b) Leave it to the MECH core**, whose draft puts it in MECH-A5, inside `src/programs/calibrate_program.cpp`
  (`briefs/MECH-core.md` §1.13, §6.2 item 7). Cost: that file becomes a 24-line shim at CAL1 Part B; MECH-A5 (a
  `src/` run) would have to edit library code, and every team's standard Calibrate would lack the watch until
  then. Under (a) MECH-A5's Calibrate hunk shrinks to feeding robot two's facts (the mechanism and role names) —
  the coordinator amends the core's brief (§5).

**D5 — The parts, their order and their size (AMENDED by the re-check: Part A grew past two days and is split).
DECIDED (d), as recommended (the team lead, 2026-10-08): three parts, each its own go.** *(D6 (a) adds ~½
builder-day — A1 +~1/16, A2 +~⅜, B +~⅛ — re-estimated from the D6 (a) probe's draft: A1 ~1–1¼, A2 ~1⅞–2⅜, B
~1⅝–2⅛, ~4¾–6 in all with the extra go. A2's upper end is now above the ~2-day line that split Part A; see §9.2 and
the D6 (a) entry's "not settled".)* ***(The check of D6 (a), C5: those ranges sat below their own items. Re-derived
from them, with the check's ~3/16 in A2: A1 ~1¼–1⅜, A2 ~2½–2⅞, B ~1⅞–2⅛, ~6–6½ in all. A2 is past the ~2-day line
at both ends, so whether to split it goes to the team lead: the check's "not settled" #1, §9.2.)***
- **A1** — the seams, a library ADDITION: the screen seam and its sanitizer, the fake, the PROS screen adapter, the
  painter and the program's screens, the front check, the shim pieces.
- **A2** — the standard Calibrate itself, a library ADDITION: the program, the PROS shell, the panel's additive
  change, the rig, the goldens, the team file's example test, HA-187, the chapter.
- **B** — robot two moved onto it, the WHICH END question removed, the side-by-side proof, the robot-facing
  documents.
- No `src/` file is touched in A1 or A2, and every existing test passes unchanged through both.
- **(d) RECOMMENDED** (§2):
  - Sizes: **A1 ~1–1¼, A2 ~1½–2, B ~1½–2 builder-days**, plus ~¼ for the extra go, verification and commit:
    **~4¼–5½ in all**.
  - That is above the plan's rough "~2–3 days (B1's ~1 included)" (`REMAINING-WORK.md` row 18b, :86). The draft
    probes found the screen seam, the shim's screen, a golden test through the real PROS shell, 31 missing doc
    lines and the four test files C1 re-pins. The re-check added the second held screen, the C-API watch read,
    the undeclared-motor words, the facts checks, nine more golden scenarios and the gate's self-test edit.
  - Every part starts once T4 Part 3 has LANDED (ruling (2); its reviews may still touch
    `calibrate_program.cpp`). A2 follows A1, and B follows A2.
  - **Why A1 and A2 can be pure additions** — measured, not assumed:
    - Today's program and today's tests switch over the panel's `PanelButtonId` (`calibrate_program.cpp`
      :704-713, `test/calibrate_panel_test.cpp` :119-126). A NEW button id would be a `-Wswitch` warning: a `src`
      gate FAILURE and a host `-Werror` failure.
    - A2 therefore adds no button id. A held screen's left box (USE NEW / SAVE) IS today's `Front` button
      relabelled, which `TouchGate` reads as `SaveHeld` on a held screen. Its RUN AGAIN is today's `RunAgain`
      id.
    - Every other panel change is an APPENDED enumerator nothing switches on (`SaveOutcome::Held`,
      `PanelScreen::Held`, `TouchAction::SaveHeld`, the new `HoldReason`), or a DEFAULTED parameter whose
      default is today's screen (`FrontRule::AskedAfterPush`, `FinishFacts{}`).
    - The library program passes `FrontRule::ByPush` and answers the calibrator's `FrontFirst` itself the tick
      DONE is accepted, so the WHICH END view is never painted. The calibrator is not touched before B.
  - **Probed for the revision** (`amend/tree_a2/`, `amend/p3r/`; the draft's P13 on `proposed3/` is superseded),
    with the revised library AND the shim pieces in `test/pros_shim/` and `src/` untouched:
    - the real `src` gate PASSES (7 builds, 0 disallowed warnings, `amend/gate_check_a2.log`);
    - the ARM header gate passes (214 headers);
    - the WHOLE host suite passes unchanged: **2,231 of 2,231 cases** (assertions 2,783,067 → 2,784,574 in the
      scans that walk more files);
    - today's program on the revised panel prints, paints and saves BYTE-IDENTICALLY (**33 of 33** files);
    - the revised Part A program and the revised end state (`amend/p2r/`, C1's removal applied) are
      byte-identical, serial, screen and card, in all fifteen scenarios (**69 of 69** files, `amend/run_end/`).
  - **Part B** then moves robot two onto it and deletes what nothing uses any more: the question
    (`Answer::FrontFirst`/`BackFirst`, the step, `backFirst`, the `WhichEnd` view, the `FrontRule` parameter, the
    auto-answer) — the pattern SAFE-SAVE's D4 used for `BootReadsBackup`: a one-part parameter, removed by the part
    that no longer needs it.
- **(a) Two parts, A1 and A2 as one Part A** (the draft's recommendation). Cost: Part A ~2½–3 builder-days in one go.
  That is over the ~2-day line for one go; it saves the ~¼ day of a third verification and commit.
- **(b) Part A also swaps robot two** (the library, C1 and the swap in one part; Part B only pins and documents).
  Cost: Part A ~3–3½ days, and pace rule 3 then puts robot two's documents (Station G, the FAQ, guide 07, HA-139)
  into Part A beside the library's (the chapter, guide 12, the changelog, the register) — ~9 documents, almost twice
  rule 7's five.
- **(c) One part.** Cost: ~11 documents in one pass and ~4–5 days in one go; the golden proof would land in the
  same change as the library it checks.

**D6 — A FIRST record on a robot that has TYPED drive ports (NEW, Re-check R16). DECIDED (a) (the team lead,
2026-10-08: "Accept, but add D6's check") — NOT the brief's recommended (b). A first record (no usable record on the
card) on a robot whose facts carry typed drive ports is checked against those typed ports. Ruling (1)'s "the previous
record" becomes "the previous record, else the typed ports".** Designed below and in §1.1/§1.4; probed in `d6a/`
(§0.6 P23–P31).
- **The gap (a) closes.** With no record on the card there was nothing to compare the front with, so the first
  Calibrate's front was whatever end led its push. A team whose typed table is a VALID description drives from it
  before calibrating (`boot_check.hpp` `chooseBootSource`, :439-442: no decoded record and a valid table →
  `UseTable`). If it tuned autons while driving from the table and then pushed its first Calibrate from the other
  end, those autons reversed without a FLIPPED warning.
- **(a) DECIDED — the design** (the builder needs no further answer):
  - **The facts carry the typed ports, optionally** (§1.1): `CalibrateFacts::typedDrive`, a `TypedDrivePorts`
    `{left[21], right[21]}` of SIGNED ports, 0 = an unused slot, appended after every other member. Absent by
    default, so the team's 21-line file is byte-identical and compiles (re-compiled with the suite's flags and
    re-linked as a V5 package's only source: P31). The vocabulary is the one a team already types: the chassis
    table's `.left = {-11, 12, -13, 14, -15}`, which the boot's `tableToDescription()` carries unchanged into
    `hal::DriveSideDescription::signedPorts`; a team on a `config::RobotConfig` passes
    `typedDrivePortsFrom(kRobot.drivetrain.tank.description)`. **"SAME FRONT as the typed ports" means exactly "the
    boot, driving from those ports, turns every motor the way this push measured" — measured, not read** (P24: robot
    two's typed table, made valid, through `decideBootSourceFrom` → `UseTable` → `robot::Robot` over the real
    `ProsDeviceProvider` and the shim → `RobotTwoDriver`, drives all 10 ports at the same millivolts as the record
    the standard Calibrate saved when its verdict was SAME, forward and turning; against a FLIPPED record, the
    forward stick reverses all 10).
  - **The rule — which reference** (§1.4): the card's record as `loadRecord` chooses it (the record file, else its
    backup) whenever there is one — the typed ports then change nothing (P23: `typed-card-wins` byte-identical to its
    untyped control); with **no usable record** — no file, **or files that exist but neither decodes** — the typed
    ports, when the facts carry them; else the FIRST RECORD note as before. **Decided for the undecodable card: the
    typed ports.** Reason: a record that does not decode is no record to the boot too — `chooseBootSource` treats
    "not decoded" exactly as "absent" and drives from the typed table when it is valid (:439-442) — so the typed
    ports are the front the robot drives with now; the decode failure is still printed. **No card: no check**, typed
    or not (nothing can be saved, so there is no front to protect; today's words). Same verdicts, same strict
    majority (`compareFronts`, unchanged).
  - **The words** (§1.4): every line, screen and NOTE that says "the card" says "your typed drive ports" when they
    were the reference. FRONT FLIPPED against them holds the save with USE NEW / RUN AGAIN only (no KEEP OLD —
    ruling (1), D2 (b)); its causes now include "the typed ports are wrong — fix them"; a SAME verdict gets its own
    NOTE `FRONT MATCHES YOUR TYPED DRIVE PORTS`; USE NEW gets `FIX YOUR TYPED DRIVE PORTS -- see the log`; `FIRST
    RECORD` only with neither a record nor typed ports. Every new string passes the screen's rule, measured (P26).
  - **Robot two** (Part B): its facts map `kChassis`'s typed ports (tableToDescription's rule: a signed table's
    entries as typed). Its typed table is FLIPPED against its real 2026-09-25 record (P7 row 3; G3a's D7), so **its
    first Calibrate on an EMPTY card holds on FRONT FLIPPED against the typed ports until G3a's D7 flips the table —
    USE NEW is right when the camera end led** (the record it then saves is SAME FRONT, 8 of 8, as the real 09-25
    record: P24). Station G and the chapter say so (§3.2). Its normal case — a record on the card — is unchanged:
    the seeded goldens are byte-identical (P23). **SUPERSEDED by the check (C1):** the facts take a table's drive
    ports only when the BOOT would drive from that table — `withTypedDriveFrom(facts, table)`:
    `tableToDescription(table)` when `valid()`, the boot's own function and test (`chooseBootSource`). Robot two's
    table is not valid (no track width or ratio, P21), so no boot drives it from that table and no auton is tuned on
    it. The draft's mapping made its empty-card check reward the wrong push: a 15/16-first push saved the reversed
    front under `FRONT MATCHES YOUR TYPED DRIVE PORTS`. Robot two therefore carries no typed ports. Its empty card
    gets the revision's FIRST RECORD note (its goldens equal `amend/run_new/`'s, P33), and Station G tells the
    operator to push the camera end first (§3.2). A robot whose table IS valid gets the typed check with no line
    written for it (§1.7).
  - **Further words from the check (C3, C4):** an undecodable card names each file and its reason on its own line. A
    card that is absent says so in the `card :` line and in a discard. SAME against the typed ports paints `FRONT
    MATCHES YOUR TYPED DRIVE PORTS` only on an exact match; otherwise it paints `FRONT MATCHES, SOME TYPED PORTS
    DIFFER -- see log`, and the log names each port (§1.4).
  - **Cost, re-estimated from the probe's draft:** ~½ builder-day (A1 +~1/16, A2 +~⅜, B +~⅛), twice the ~¼ the
    draft offered *(the check adds ~3/16 in A2, and re-derives every part from its items: §9.2)*. The draft's ¼ did not price the undecodable-card rule, the hold's words per reference (RUN AGAIN,
    the field disable, the MECHANISM PORT TURNED evidence line), the facts checks on the typed lists, the eleven
    golden scenarios, the parity test, or the eighteen mutations (A2's 28–42, B's 8–10).
- **(b) The alternative not taken — state the gap** (§4, the chapter, Station G): the FIRST RECORD note and a
  chapter sentence telling a team with typed ports to push the end its autons call the front. Cost: none in code;
  but a team with a VALID typed table that pushed its first Calibrate from the other end would reverse every auton
  tuned on the table with only an amber note to say so — the exact hazard ruling (1)'s flip warning exists for. No
  robot in the tree is exposed today (robot two's typed table is not a valid description, P21), which is why the
  brief recommended (b); the team lead chose to close it.

**Consequences of the rulings, stated (not asked):**
- **C1 — the WHICH END question leaves the library (Part B).** Ruling (1) makes the end that leads the push the
  front, so a question on the brain's screen would be the front ASKED. The held screens ask no such question
  either (Re-check R2): they offer only "save as pushed" or "save nothing". Part A already never paints it (D5);
  Part B REMOVES it once robot two's program no longer answers it: `Answer::FrontFirst`/`BackFirst`, the step
  `PushAwaitingFront`, `CalibrationReport::backFirst`, the panel's `WhichEnd` screen, the `FrontRule` parameter and
  the auto-answer; `PanelButtonId::Front` becomes `SaveHeld` and `Back` is removed (Re-check R2; the draft renamed
  both to `KeepFront`/`NewFront`). The steps read 1/4..4/4; the push screen says PUSH FRONT FIRST and "the end that
  leads the push is FRONT". `calibration/` is in no Freeze Register row (`docs/roadmap.md` :127-145), so this is a
  changelog BEHAVIOUR CHANGE in an unfrozen layer, not a major bump. Four test files re-pin (§2.B). Measured for the
  revision (`amend/tree_end/`): 205 of the 210 test TUs compile against the end state. The 4 CAL1 ones are exactly
  C1's re-pins (`calibrate_panel_test`, `drivetrain_calibrator_test`, `median_nonfinite_test`,
  `drive_characterizer_test` through `calibration_test_rig.hpp`). The fifth, `dropped_autons_test`, fails on the
  probe tree's mix of T4 Part 3's 10-08 header with its 10-06 test: T4's pair, not CAL1's.
- **C2 — robot two keeps its slot, beacon and gate line.** `make ROBOT=tank PROGRAM=calibrate`, slot 4, the variant
  beacon `shulib-robot-variant=tank-calibrate` (`src/build_selection.hpp` :138-141) and the gate's `tank-calibrate`
  row are unchanged; `src/programs/calibrate_program.{hpp,cpp}` stay as a 27-line glue (24 before the check of D6 (a)) so `main.cpp`'s dispatch
  keeps its shape; its three state-screen cases call the library's (`main.cpp` :180, :203, :242), and the gate's
  `PROGRAM_ENTRY_SYMBOLS` (`tools/src_build_gate.py` :473-484) names them (measured: without that line step 0
  FAILS naming each).
- **C3 — what leaves `src/common/`:** only the sanitizer (the screen's one sanitizer OUTSIDE Bench Tests, whose
  `bench_r3a.cpp` keeps its own at its 54-column width — Re-check R7). `printText()`'s body becomes `hal::screenSafeLine()` in
  the new `hal/brain_screen.hpp`; `printText()` itself and `g_screenPrints` stay in `src/common/screen_text.hpp`,
  because the drive, competition and X-drive programs and `screen_paint.cpp` paint through them (measured: all 7
  builds pass with `printText` calling the library's sanitizer).
- **C4 — HA-187 only;** HA-188 is released. **C5 — the save is B1's**, word for word
  (`formatSaveReport(report, BootReadsBackup::No, …)`, `record_store.hpp` :744-916). **C6 — WITHDRAWN by Re-check
  R2** (KEEP OLD's provenance): nothing is ever saved but the measurement as pushed, so the provenance is always the
  calibrator's.

## The design in plain words (for the team lead)

- **A team gets Calibrate by writing one short file** (§1.1): "our drive is a tank, our drive motors are blue,
  our mechanisms are on ports 6 and 7". Nothing else is typed. The program is the library's, so every team — and
  the VS Code panel, which reads the record Calibrate saves — gets the same, correct one.
- **Robot two's Calibrate becomes that file's twin**, reading the cartridge and the claw ports from its two
  tables. Same slot (4), same build line, same serial tag `[R3D]`, the same screens and words — except where the
  rulings change them, and every one of those differences is listed (§0.5): it was run side by side with today's
  program on the host test plant, ten scenarios (eight of them comparable line by line), and every differing line
  has a named reason.
- **The front is the end you push first.** The WHICH END FIRST? screen is gone; the push screen says PUSH FRONT
  FIRST and "the end that leads the push is FRONT". Before saving, Calibrate compares the new record with the one
  on the card — or, when the card holds no usable record, with the drive ports the team TYPED, if its facts carry
  them (D6 (a), decided 2026-10-08). A robot's typed table counts only when its drive program could drive from it
  (the check of D6 (a)): robot two's cannot yet (no track width or ratio), so robot two's first record gets the
  FIRST RECORD note, and its operator pushes the camera end first. If the card's record (or the typed ports) calls
  the end that led its BACK, it does NOT save. A FRONT FLIPPED screen
  offers USE NEW (save as pushed: right if the FRONT led and the drive cables were swapped between the sides, or
  the front was changed on purpose) or RUN AGAIN (save nothing: right if the BACK led by mistake). The records
  cannot tell those apart, so the screen never guesses (Re-check R2/R10, measured on the host test plant). Today's
  program, pushed backwards by mistake and answered FRONT FIRST, saves a reversed robot without a word — the host
  test plant shows it.
- **The save is SAFE-SAVE's** (B1): a failed save can never erase the good record, and the words say what the
  card measurably holds.
- **Calibrate watches the mechanism ports during the push** (never powering them). If one turned like a drive
  motor, it does NOT save until the operator says SAVE (it is a mechanism the push turns) or RUN AGAIN (it is a
  drive motor: fix the facts). This is the only catch for a mechanism wired to a drive port when no drive ports
  are typed (Re-check R12).
- **A team lists EVERY motor that is not a drive motor.** A motor left out is a drive candidate, and one the push
  turns is powered at 3, 6 and 9 V like a drive motor (measured, Re-check R11). The team file says so, and so does
  Calibrate's banner when nothing is declared.
- **Three parts, ~6–6½ builder-days** (A1 ~1¼–1⅜, A2 ~2½–2⅞, B ~1⅞–2⅛; re-derived from their own items by the
  check of D6 (a); the earlier ~4¾–6 sat below them; whether A2 splits is the team lead's): the library's seams (A1), then the
  standard program itself (A2), each changing no program and leaving every existing test passing (measured); then
  robot two moved onto it, the old question deleted, the side-by-side proof and the robot-facing documents (B).

## 0. Why — what exists, re-read in the tree T4 Part 3 leaves (every line opened)

### 0.1 Calibrate today: library logic, a robot-two program around it
- **The logic is already library code** (`include/shulib/calibration/`, 7,455 lines with R5a Part A):
  `drivetrain_calibrator.hpp` (the tick-driven calibrator: the push, the pulses, the spin; 1,978 lines),
  `calibrate_panel.hpp` (`viewFor()` — every word, button and place on the screen — and `TouchGate`; 1,016),
  `calibration_record.hpp` (the 180-byte record, refuse-never-misread; 340), `record_store.hpp` (SAFE-SAVE Part A's
  safe save and load, and B1's words, `formatSaveReport`; 916), `boot_check.hpp` (the record against the registry;
  449).
- **The PROGRAM is robot two's** — `src/programs/calibrate_program.cpp` (1,388 lines in Part 3's tree, 1,387 on main; `.hpp` 35):
  - the PROS calls: the screen and touch (:283-345, :511-575, :1010-1019), the competition status (:558-575,
    :1276-1277), `pros::delay` (:1252, :1375), `printf` (:192-195, :1044-1057);
  - robot two's tables, 11 references on 10 lines (Re-check R9; the draft said 18): `kChassis.cartridge`
    (:1138, :1146, :1180, :1227, :1238), `kChassis.imuPort` (:1155 twice, :1160, :1163), `kChassis.robot`
    (:1087), `kMechanisms` (:1130);
  - `src/common/screen_text.hpp`'s `printText()` and `g_screenPrints` (:139; :255-259 says why they moved there);
  - robot-two words in the banner and refusals: "robot two MEASURES ITS OWN DRIVETRAIN" (:1045, :1069), the
    2026-09-24 history (:1048-1050, :1072-1074), "(Bench Tests, slot 3)" (:1152, :1171, :1199, :1217),
    "restart slot 4" (:471), "type it into src/robots/chassis_table.hpp" (:1142);
  - the WHICH END question it answers on the screen (:1337-1339, `emitFront` :801-818) — ruling (1) removes it;
  - today's save, `saveAndReadBack()` (:631-685), with SAFE-SAVE's defect: its `"wb"` open erases the good record
    before a byte is written, and its NOT SAVED text claims the opposite (:904-906; SAFE-SAVE brief §0.1). Part B1
    replaces it (absorbed here).
- **What calls it:** `src/main.cpp` :154-156 (`calibrateInitialize`), :279-280 (`runCalibrate`); its three
  state-screen cases share Bench Tests' and Drive's `benchStateScreen` (:178-183, :201-207, :242-246). The chain
  picks it at `src/build_selection.hpp` :138-141 (beacon `shulib-robot-variant=tank-calibrate`), refuses it for any
  robot but `tank` at :89-90 ("takes the cartridge from robot two's chassis table").
- **The tests that read it as TEXT:** `test/mechanism_table_test.cpp` pins the census filter and its log line
  (SAFE-SAVE landmine 9; the MECH core's §1.13 re-points them) — Part B re-points them at the library (§2.B).

### 0.2 What the plan and the other briefs ask of CAL1
- **SAFE-SAVE Part B1** (its brief §1 "Part B1", §2 "Part B1", §3.2): Calibrate saves through
  `calibration::saveRecordSafely` over a default `ProsFileStore`; `saveAndReadBack()` is REMOVED; the panel is
  told Saved iff the report is Saved; the serial outcome is `formatSaveReport(report, BootReadsBackup::No, …)`; the
  banner, the `record file:` line and `calibrateInitialize`'s banner name both files; `finishStopped`'s "Nothing was
  written to the card by this run" stays; a source pin; three mutations; Station G and Station 6; HA-139's settle and
  blast radius; guide 12's two rows. **All of it is CAL1's** (§1.5, §2, §3). B2 (the boot's fallback) stays
  SAFE-SAVE's: until it lands, the words say the drive program does not read the backup.
- **R5a Part C** (its §1 Part C, D7) adds GAINS and REST to Calibrate AFTER CAL1: a start screen offering
  CALIBRATE / GAINS / REST when the card's record decodes and passes the registry check, and GAINS / REST beside
  RUN AGAIN after SAVED; its own save words (`formatGainsSaveReport`). **CAL1 builds neither** and leaves the place
  (§1.6): a run KIND the program switches on (one value, `Drivetrain`), the finished screen's buttons as a list,
  and the program's motor construction reusable for a run made from the record's ports.
- **G3a** (amended 2026-09-29; front ruled 2026-10-06): Part 2 generates a new robot's bring-up builds, Calibrate
  included. Under CAL1 that needs NO Calibrate line: `src/programs/calibrate_program.cpp` and
  `src/robots/calibrate_facts.hpp` read the BUILD's `kChassis`/`kMechanisms`, which Part 2 already generates;
  Part 2's chain branch, Makefile row and gate row are all a generated robot's Calibrate needs (§1.7). G3a's
  Calibrate hunks (the banner from `kChassis.robot`, the front's words, the front `static_assert`) are superseded:
  the banner already prints the facts' name, and there are no front words (ruling (1)). Its §5 item 7, the WATCH,
  is D4. Its D7 (flip robot two's typed table to the camera end) still keeps the no-record fallback agreeing with
  the record; CAL1 does not touch the typed table. **Under D6 (a) a typed table is also Calibrate's reference on a
  card with no usable record — when the boot could drive from it** (`tableToDescription(t).valid()`: the check of
  D6 (a), C1). Robot two's cannot (its track width and ratio are UNSET, P21), so its first Calibrate on an empty
  card gets the FIRST RECORD note whatever D7 decides; D7's flip matters to Calibrate only once that table is also
  valid (§5). *(Superseded: "until D7 lands robot two's first Calibrate on an empty card holds on FRONT FLIPPED
  when the camera end leads".)* Its D10 (a) (a NEW robot's drive ports OPTIONAL — still the team lead's to answer)
  means a generated robot MAY type them; when its generated table is a VALID description, the glue's
  `withTypedDriveFrom` puts them into Calibrate's facts (§1.7).
- **MECH-PANEL-requirements §4.1** (amended 2026-10-06): the panel's new-robot page asks name, kind, cartridge — the
  front is Calibrate's. §4.5: the panel shows the record read-only, with its disagreement warning. The panel asks no
  drive ports, so a panel-generated robot carries no typed ports and its first Calibrate gets the FIRST RECORD note
  (D6 (a); §5).

### 0.3 The front, as the record carries it
The record has no front field (`calibration_record.hpp` layout :160-178; G3a landmine 10): it carries each drive
port's SIDE and SIGN, both relative to the push's leading end (`drivetrain_calibrator.hpp` — the push captures the
signs, the pulses assign sides by the yaw's sign). Push with the other end leading and every port changes side AND
sign: robot two's real 2026-09-25 record (camera end, L −16 +17 −18 −20 | R +11 −12 +13 +15,
`test/boot_source_test.cpp` ~:111-126) against its typed table (15/16 end, `src/robots/chassis_table.hpp`
:126-128) is exactly that, 8 of 8 (`probe_front_save/`). So the flip check needs only the two records — or, with no
usable record on the card, the new record and the typed drive ports (D6 (a)), which carry each port's side and sign
in the same convention: the boot's `tableToDescription()` copies a signed table's entries unchanged into the
description's sides, and a drive built from those turns every motor exactly as one built from a record that agrees
with them (measured through the boot's own path, §0.6 P24).

### 0.4 What a team must state, and nothing else (the runtime-discovery ruling)
| Fact | Who states it | Why |
|---|---|---|
| the drive KIND | the team | stated in the config vocabulary (`config::DrivetrainKind`: Tank, XDrive, HDrive — Re-check R13); Calibrate measures a tank; a holonomic drive's calibration is not built (R3d §2 OUT) — it refuses anything else by name (`NOT A TANK -- the facts say x-drive`, measured) |
| the drive motors' CARTRIDGE | the team | unobservable from inside: every insert turns the rotor the same and the firmware divides by what it was told (R3d §2, rejected #2) |
| the MECHANISM motor ports — EVERY motor that is not a drive motor | the team | the registry reports a motor's kind, never its job; a declared port is never a candidate, never geared, never powered (T3's rule, `calibrate_program.cpp` :1119-1135). **An UNDECLARED motor is a drive candidate: one the push turns is pulsed at 3, 6 and 9 V like a drive motor** (measured, Re-check R11) |
| the drive ports, their directions, the sides | MEASURED | the push and the pulses |
| *(optional)* the drive ports a team ALREADY TYPES for its drive program, signed, per side | the team, only if it has them | never used to calibrate: with no usable record on the card they are the front check's reference (D6 (a), §1.4); a repository robot's come from its table only when the boot could drive from it (`withTypedDriveFrom`, the check of D6 (a)) — robot two's cannot yet, so it has none; a panel-generated robot has none |
| the FRONT | MEASURED | the end that leads the push (ruling (1)) |
| the distance scale, the track width | MEASURED | the push along a tape; the spin |
| the IMU's port | MEASURED | the registry; a stated port is an optional cross-check only (robot two's) |

### 0.5 Robot two's Calibrate, today's against the proposed — side by side on the host test plant
**The method** (`harness/`): each program — today's `src/programs/calibrate_program.cpp`, or robot two's glue over
`proposed2/`'s library program — is compiled with the PROS SHIM and run on the A2 plant in robot two's shape (two
wheels, five coupled members a side, members 11, 13, 15, 17, 19 wired backwards, claw pivots on 6 and 7, the IMU
on 2, the radio on 1). The plant steps every 10 ms of shim time; a scripted operator pushes the robot 48 in and taps
whatever button the SCREEN shows (it reads the draw calls, never the program's state) after a 0.3 s reaction; a
second run uses RUN AGAIN; the field then disables the robot. Outputs: the serial log, every draw call, the files on
the card. `harness/classify.py` aligns the two serial logs with every number masked and puts EVERY differing line
under one named reason; a line no rule claims is a RESIDUAL and fails the comparison.

| Scenario | What it does | Serial lines old → new | Differing lines by reason (G generic words · F the front · S the save · W the watch · T timing) | Residual | Draw calls old → new | Record saved |
|---|---|---|---|---|---|---|
| `one` | one clean run | 138 → 139 | G 19 · F 19 · S 7 · W 8 · T 14 | **0** | 298 → 273 | **byte-identical** (`.bin`); `.bak` = `.bin` |
| `clean` | two runs (RUN AGAIN) | 218 → 221 | G 19 · F 30 · S 10 · W 10 · T 32 | **0** | 557 → 506 | 6 bytes differ: the last bits of ipr L and W, and the CRC (T) |
| `slip18` | port 18 slips → LEFT OUT | 135 → 136 | G 19 · F 19 · S 7 · W 8 · T 14 | **0** | 287 → 262 | **byte-identical** |
| `curved` | run 1 curves (STOPPED PUSH CURVED), run 2 straight | 167 → 170 | G 19 · F 25 · S 7 · W 10 · T 24 | **0** | 377 → 352 | **byte-identical** |
| `nocard` | no SD card | 137 → 137 | G 19 · F 19 · S 10 · W 8 · T 14 | **0** | 298 → 273 | none (NOT SAVED both) |
| `noimu` | no IMU → refused | 48 → 44 | G 19 · F 3 · S 4 · W 4 | **0** | 18 → 17 | none |
| `disable` | the field disables 8 s into the pulses | 99 → 98 | G 19 · F 18 · S 4 · W 8 · T 8 | **0** | 135 → 111 | none |
| `flip-keep` | run 2 led with the OTHER end; new: KEEP OLD *(the draft's design — withdrawn, Re-check R2; see the re-check paragraph below)* | 218 → 229 | G 19 · F 40 · S 10 · W 10 · T 32 | **0** | 557 → 531 | **old: the FLIPPED record, saved silently**; new: the kept front — 5 bytes from today's BACK FIRST answer (`oldback`): ipr's and W's last bits, the CRC |
| `flip-new` | the same; new: USE NEW | — → 227 | (new only) | — | — → 531 | the flipped record, chosen (5 bytes from today's silent one: last bits, CRC) |
| `watch` | the push turns declared port 6 | — → 139 | (new only) | — | — → 273 | saved; `WARNING: port 6 is declared as claw pivot 6 but TURNED WITH THE PUSH (+1692.4 deg)` |

**Every reason, and why it is right** (the classifier's rule table, `harness/classify.py` :24-90, is the full list):
- **G — robot two's words become the standard program's** (19 lines in every scenario: the banner and boot
  banner). "robot two MEASURES ITS OWN DRIVETRAIN" → "the robot …"; the 2026-09-24 run history leaves the banner (it
  is R3d's history, kept in the progress log and Station G, not a property of the library's program); "(table says
  2)" → "(stated 2)"; "(the chassis table is NOT consulted …)" → "(no typed port list is consulted …)"; the
  X-drive sentence goes; "(Bench Tests, slot 3)" and "restart slot 4" go from the refusal screens and lines, since a
  team's project may have no Bench Tests and Calibrate cannot know its own slot (§5: PKG1 may add a stated "where to
  diagnose" fact). The leading blank line of `calibrateInitialize`'s banner goes.
- **F — ruling (1).** The WHICH END screen, its touch and its `front:` line go; the push screen says PUSH FRONT
  FIRST and "the end that leads the push is FRONT"; the steps renumber 1/4..4/4; the PUSH TABLE header says the
  leading end is the front; the banner and the push instructions say it; the `card :` line and the `front check`
  line (§1.4); in `flip-keep` the FRONT FLIPPED screen, its words, the KEEP OLD touch and the turned LEFT/RIGHT
  lines; the finished header `SAVED -- FRONT KEPT` (the draft's; in the revision, the held screens' lines — the
  re-check's `classify2.py` rule table).
- **S — SAFE-SAVE B1, absorbed.** The banner and the `record file:` line name the backup; SAVED is B1's two lines
  (written, RE-READ, compared, re-read at the end; the backup re-reads identical); NOT SAVED is B1's sentence plus
  what the card measurably holds — today's false "(A record saved by an EARLIER run, if any, is still on the
  card …)" is gone.
- **W — the watch and the declared port's name.** "port 6: claw pivot (declared)" → "port 6: claw pivot 6
  (declared)" (the name the facts carry: the table's entry name); the two `watch` banner lines; one `watch:` line
  per declared port at DONE.
- **T — timing on the harness's 10 ms grid.** The same line with different numbers: a pulse's yaw at its deciding
  level (−6.31° or −6.47° — whether the 0.4 s pulse spans 40 or 41 plant steps), the loop's tick and print counts,
  the touch release counts, a live number at the moment of a disable. **The control** (`ctl/`): today's program run
  against ITSELF with the operator 0.1 s slower gives T lines only (8 in `one`, 18 in `clean`, residual 0) and a
  record 5 bytes different in the same last bits — the proposed program, which skips the WHICH END step's ~0.8 s,
  moves the same way. The screen's printed TEXT differs only by F and G (`final_*/*.screen.log`: the WHICH END
  screen's texts gone; PUSH FRONT FIRST, the front rule, the FRONT FLIPPED screen's texts and the `SAVED -- FRONT
  KEPT` header, the two refusal footers, and one live number caught at the disable).

**Found by the comparison and fixed in the design (in the re-check's `amend/p3r/`):** with NO card, the front check said
"no usable record on the card -- this is the FIRST" (`final_new/nocard`) — false, the card is absent, not empty.
§1.4 gives the three cases their own words.

**Re-check (2026-10-08): the revised design, side by side again** (`amend/harness2.cpp`, `amend/p3r/` = the
revised Part A form, `amend/p2r/` = the revised end state, `amend/run_new/`, `amend/run_end/`,
`amend/classify2.py`). The table above is the DRAFT's, and its `flip-keep`/`flip-new` rows describe the withdrawn
KEEP OLD. For the revision:
- **Comparable scenarios against today's program: residual 0 in all seven** (`amend/classify2.log`). The counts
  match the draft's except `slip18`, which has M 2: the LEFT OUT line's "or a MECHANISM motor" words (R11).
- The records of `one`, `slip18` and `curved` are byte-identical to today's. `clean`'s differs in 6 bytes, the
  same T class as before.
- Serial lines and draw calls (new):

  | Scenario | Serial | Draws | Card |
  |---|---|---|---|
  | `one` | 139 | 274 | saved |
  | `clean` | 221 | 507 | saved |
  | `slip18` | 136 | 262 | saved |
  | `nocard` | 137 | 273 | none |
  | `curved` | 170 | 353 | saved |
  | `noimu` | 44 | 17 | none |
  | `disable` | 98 | 111 | none |
- **New scenarios (new program only):**

  | Scenario | What it does | Serial / draws | Card at the end |
  |---|---|---|---|
  | `flip-again` | run 2 led with the other end → HELD → RUN AGAIN → run 3 front-led → SAME FRONT | 304 / 749 | run 3's record, **10 of 10** motors forward |
  | `flip-new` | the same wrong push → USE NEW | 229 / 532 | saved as pushed, **0 of 10** (the warned-of case) |
  | `flip-disable` | the same → HELD → the field disables | 221 / 516 | **both files byte-identical to run 1's record** |
  | `swap` | seeded card; LEFT/RIGHT cable bundles swapped; the CAMERA end leads → FRONT FLIPPED 10 of 10 → USE NEW | 147 / 298 | **10 of 10**; the signed record equals `flip-new`'s (6 timing bytes differ) |
  | `swap-keep-old-equivalent` | the same, the field disables on the held screen: the card keeps what KEEP OLD would have saved | — | **0 of 10** — KEEP OLD reverses a swapped robot (R10) |
  | `watch` | the push turns declared claw 6 (empty card) → MECHANISM PORT TURNED → SAVE | 146 / 299 | saved 10/10; NOTE `PORT 6 KEPT AS A MECHANISM` |
  | `watch-typo-again` | drive port 12 declared as "claw pivot 7"; seeded 10-port card → HELD (`the card's record DRIVES with port 12`) → RUN AGAIN → HELD → field disable | 218 / 502 | **both files byte-identical to the seed** |
  | `watch-typo-save` | the same → SAVE (what the draft did unasked) | 145 / 288 | L4/R5: the good record replaced by one a motor short — the operator's informed choice now |
  | `undeclared` | undeclared motors on 8 (still) and 9 (an intake the push turns, free-spinning) | 148 / 286 | 10/10; port 8 never powered (max 0.0 V); port 9 pulsed 3, 6, 9 V (1.19 s), then LEFT OUT |
  | `facts-port0`, `facts-toofew`, `facts-xdrive` | `{0, "intake"}`; drive ports 11-19 declared; `.drive = XDrive` | 44 / 18, 55 / 19, 44 / 19 | none: FACTS REFUSED / TOO FEW MOTORS naming the declared ports / NOT A TANK, before any motor is made |
- **The end state equals Part A's form:** `amend/run_end/` (C1's removal applied) is byte-identical to
  `amend/run_new/` in all fifteen scenarios (69 of 69 files).
- The declared ports get 0 gearing, units, voltage and brake-mode writes and 2 C reads per push in every run.
  That includes port 12 when the typo declares it (`amend/run_typo_counters/`). A port that becomes UNDECLARED in a
  typo scenario (7) is a candidate: it gets the drive cartridge (one gearing write) and 0 V commands only, max
  |V| 0.0.
- *Probe artefact:* the probe's long card root cuts the path in the SAVED line (`/…/amend/card/shulib_drivetrai`).
  On the brain it is `/usd/shulib_drivetrain.bin`, as `final_new/` shows.

**D6 (a) (2026-10-08, afternoon): the goldens again, with robot two's typed drive ports in its facts** (`d6a/`:
`p3r/` and `p2r/` with the typed check, `harness3.cpp` = `amend/harness2.cpp` plus the D6 scenarios, its runs
`run_new/` and `run_end/`, `classify_d6.py`). The facts are Part B's — `calibrateFactsFor(kChassis, kMechanisms)`
with the typed mapping — so the empty-card scenarios now compare against LEFT −11 +12 −13 +14 −15 | RIGHT +20 −19
+18 −17 +16. The probe's card directory is `d6a/cards1/` (11 characters, as `amend/card/` is), normalized in every
comparison. *A control first:* `amend/harness2.cpp` over an unchanged copy of `amend/p3r/`, built and run from
`d6a/` (`ctl_run/`), reproduces `amend/run_new/` byte for byte (79 files; the other 2, `watch-typo-*`'s stderr, equal
`amend/run_typo_counters/`, the harness's later counter lines; `swap-keep-old-equivalent`'s two card copies have no
counterpart, as `amend/run_new/` did not keep them — re-counted in the continuation, `d6a/v2/`).
- **The fifteen goldens (69 files) against the revision's (`amend/run_new/`):** 50 byte-identical — every `.bin` and
  `.bak`, every stderr (the FORWARD scores and the shim's counters; `watch-typo-*`'s compared, as in the control,
  with `amend/run_typo_counters/`), and all of `nocard`, `swap`, `watch-typo-again`,
  `watch-typo-save`: **the normal case, a record on the card, is unchanged**. The other 19 (the empty-card runs'
  serial and screen logs, and `noimu`'s and `disable`'s serial) differ ONLY by D6's lines — **residual 0**
  (`classify_d6.log`; re-run on `d6a/v2/run_new/`: the same totals, residual 0): the `card :` line (14 — the eleven
  empty-card goldens' and, outside the fifteen, the three `facts-*` refusals', whose serial logs differ by that line
  alone), the FIRST line replaced by `checking against your typed drive ports
  instead`, the `typed drive ports:` list and the counts line, SAME FRONT 10 of 10 (9 each), and the NOTE `FIRST
  RECORD …` → `FRONT MATCHES YOUR TYPED DRIVE PORTS` (8). In `clean`, `flip-*` only run 1 changes: runs 2 and 3
  compare against run 1's record, as before.
- **New scenarios** (robot two's facts unless said; "forward" is the harness's score against the PLANT's front, which
  is the end robot two's typed table calls front — §8):

  | Scenario | What it does | Serial / draws | Card at the end |
  |---|---|---|---|
  | `first-untyped` | `one` with NO typed ports (the team file's shape) | 139 / 274 | **byte-identical to `amend/run_new/one`** — serial, screen, card (FIRST RECORD; the card directory normalized, the harness's scenario-name line in stderr aside) |
  | `typed-flip-new` | empty card; the other end leads (robot two's real camera end) → FRONT FLIPPED against the typed ports, 10 of 10 → USE NEW | 150 / 299 | saved as pushed; the `FIX YOUR TYPED DRIVE PORTS` line and NOTE; vs the real 09-25 record: SAME, 8 of 8 (P24) |
  | `typed-flip-again` | the same → RUN AGAIN (`the card is unchanged (it holds no usable record)`) → run 2 table-end-led → SAME | 226 / 516 | run 2's record, forward 10 of 10 |
  | `typed-flip-disable` | the same → the field disables on the held screen | 141 / 282 | **no file** — `… NOTHING SAVED: the card is unchanged (it holds no usable record)` |
  | `typed-card-wins` | card seeded with `one`'s record; the typed ports FLIPPED (G3a D7's table); front-led | 139 / 273 | **serial, screen and card byte-identical to `typed-card-wins-untyped`** (the same with no typed ports): the card decides |
  | `typed-undecodable` | two 180-byte garbage files; front-led | 141 / 274 | `does not decode (magic is not "SHDC" …) -- checking against your typed drive ports instead`, SAME, saved over them |
  | `typed-undecodable-flip` | the same, the other end leading → held → USE NEW | 150 / 299 | saved as pushed, NOTE `FIX YOUR TYPED DRIVE PORTS` |
  | `typed-tie` | typed LEFT −11 +12 −13 −16 +17 −20, RIGHT −14 +15 +18 −19 | 142 / 274 | CANNOT TELL (5 agree, 5 flipped), saved, NOTE `FRONT NOT CHECKED` |
  | `typed-nocommon` | typed LEFT −3 +4, RIGHT +5 −8 | 142 / 274 | CANNOT TELL (0 shared), saved, NOTE `FRONT NOT CHECKED` |
  | `typed-watch-typo` | empty card; drive port 12 declared → SAME 9 of 9 → MECHANISM PORT TURNED (`your typed ports DRIVE with port 12`) → RUN AGAIN → held again → disable | 222 / 502 | **no file**; both RUN AGAIN and the disable say `the card is unchanged` |
  | `facts-typed-dup`, `-range`, `-oneside` | 11 on both sides; −22; the RIGHT side empty | 44 / 18 each | none: `FACTS REFUSED` naming `typedDrive.right[4] = 11`, `typedDrive.left[2] = -22`, `typedDrive.right[0] = 0`, before any motor is made |
- **The end state equals Part A's form again:** `d6a/run_end/` (C1's removal applied to the D6 library) is
  byte-identical to `d6a/run_new/` in all 33 scenarios — **143 of 143 files** (the fifteen goldens' 69, the D6
  scenarios' 60, `facts-port0/-toofew/-xdrive` and `swap-keep-old-equivalent`'s 14).
- **Re-verified (continuation, `d6a/v2/`):** `harness3.cpp` was edited after these runs (13:25: the
  `watch-typo-empty-untyped` scenario, §1.4). Rebuilt from the final harness, Part A's form and the end state each
  reproduce `d6a/run_new/` byte for byte (**143 of 143** each, `v2/run_new/`, `v2/run_end/`), and so they do over T4
  Part 3 AS LANDED (`bdcbc7c`): **143 of 143** each again (P32).

**The check of D6 (a) (2026-10-08, evening): the goldens again — robot two untyped, the typed cases typed**
(`d6a/v3/`: `p3r/`, `p2r/`, `harness4.cpp`, `run_new/`, `run_end/`, `compare_v3.log`, `classify_v3.log`; built
against T4 Part 3 as landed, `d6a/landed_ctl/`; the card directory `d6a/cards3/`, 11 characters, normalized). The
paragraph above describes the D6 (a) draft. Its "robot two's typed drive ports in its facts" is withdrawn by C1:
robot two's facts are the glue's `withTypedDriveFrom(kCalibrateFacts, kChassis)`, which carries no typed ports
(its table is not valid).
- **Robot two's 19 untyped scenarios equal the revision's** (`amend/run_new/`). Of the 81 files with a counterpart,
  80 are byte-identical; `nocard.serial.log` differs by C3's `card :` line. The side-by-side classifier against
  today's program gives residual 0 in all seven comparable scenarios, with `amend/classify2.log`'s counts
  (`v3/classify_v3.log`). So the D6 lines the paragraph above counts are gone from robot two's runs.
- **The typed scenarios now type their ports.** Case 8 is robot two's table made valid (P24's track 12.5 in and
  ratio 1.0) through `withTypedDriveFrom`: LEFT −11 +12 −13 +14 −15 | RIGHT +20 −19 +18 −17 +16, the ports the draft
  derived. `typed-first` is the draft's `one`. Against `d6a/run_new/` (the fourteen typed and `facts-typed-*`
  scenarios), 56 of 60 files are byte-identical. The other 4 are `typed-first`'s name line, `typed-undecodable` and
  `-flip`'s per-file reason lines (C3), and `typed-watch-typo`'s difference lines (C4: port 12 typed but declared).
- **The check's scenarios:**

  | Scenario | What it does | Serial / draws | Card at the end |
  |---|---|---|---|
  | `typed-first` | the draft's `one`: empty card, the typed ports' end leads | 141 / 274 | saved; SAME 10 of 10; NOTE `FRONT MATCHES YOUR TYPED DRIVE PORTS` |
  | `typed-partial` | typed LEFT −11 −12 −13 +14 −15, RIGHT +20 −19 +18 −17 +9 (12's sign wrong, 16 untyped, 9 not a drive motor) | 145 / 274 | saved; SAME (8 agree, 1 sign only, 1 in each list only), the three port lines, NOTE `FRONT MATCHES, SOME TYPED PORTS DIFFER -- see log` (C4) |
  | `empty-watch-untyped` | renamed `watch-typo-empty-untyped`: empty card, no typed ports, drive port 12 declared → held → RUN AGAIN → held → disable | 216 / 502 | **no file**; both discards: `the card is unchanged (it holds no usable record)` (C2) |
  | `undecodable-untyped` | two 180-byte garbage files, no typed ports | 141 / 274 | saved; `NOT DONE -- the card's record does not decode` + a line per file; NOTE `FRONT NOT CHECKED` (C3) |
  | `bakonly-garbage-untyped` | only the `.bak`, 180 bytes of garbage, no typed ports | 140 / 274 | saved; NOT DONE + `  shulib_drivetrain.bak: magic is not "SHDC" …`; NOTE `FRONT NOT CHECKED` (C3) |
  | `bakonly-garbage-typed` | the same, typed ports (case 8) | 142 / 274 | saved; `does not decode -- checking against your typed drive ports instead` + the `.bak` line; SAME (C3) |
  | `emptyfile-untyped` | the record file EMPTY (0 bytes), no backup | 140 / 274 | saved; NOT DONE + `  shulib_drivetrain.bin: EMPTY (0 bytes read)` (C3) |
  | `nocard-watch-again` | no SD card; drive port 12 declared → held → RUN AGAIN → held → disable | 216 / 502 | none; `card : NO SD CARD in the brain …`; both discards: `there is no SD card in the brain` (C3) |
  | `typed-nocard` | no SD card, typed ports (case 8) | 137 / 273 | none; equal to `nocard` but the name line: no card, no check (mutation 40) |
  | `facts-typed-rightonly` | only the RIGHT side typed | 44 / 18 | none: `FACTS REFUSED -- typed drive ports must list BOTH sides -- the LEFT is empty: typedDrive.left[0] = 0` (mutation 53) |
- **The end state equals Part A's form:** `v3/run_end/` = `v3/run_new/` in all 43 scenarios, **185 of 185 files**.

### 0.6 The probes — what was run, and what each showed (re-run 2026-10-07; P14 re-checked 2026-10-08; P15–P22 the 2026-10-08 re-check's, `amend/`; P23–P31 D6 (a)'s, `d6a/`, each re-verified in `d6a/v2/`; P32 on T4 Part 3 as landed; P33–P37 the check of D6 (a), `d6a/v3/`)
| Probe | What | Result |
|---|---|---|
| **P1 the goldens** (`harness/`, `final_old/`, `final_new/`, `golden_classified.txt`) | §0.5, ten scenarios, both programs | every differing serial line classified, **residual 0** in all eight comparable scenarios; saved records as §0.5's table; the 2026-10-06 run's logs (`golden_old/`) reproduce byte for byte (`rerun_old/`) |
| **P2 the timing control** (`ctl/`) | today's program against itself, the operator 0.1 s slower | T lines only; record differs in the last bits — the same class as old→new |
| **P3 robot two's build** (`tree_gate/`, `gate_make_calibrate.log`) | the real `make ROBOT=tank PROGRAM=calibrate` with robot two's glue over `proposed2/` | links; ONE warning in our code at first — `-Wformat-truncation` on KEEP OLD's provenance mark (the gate would FAIL it) — removed with the mark (C6) |
| **P4 the src gate** (`gate_check.log`, `gate_check2.log`) | the real `python3 tools/src_build_gate.py check` on `tree_gate/` | first run: all 7 builds link, 0 disallowed warnings, but step 0 FAILS: `src/main.cpp uses shulib::bench::calibrateAutonomous, which belongs to no program in this gate's PROGRAM_ENTRY_SYMBOLS` (and Connected, Disabled). With the three names added to the Calibrate row: **PASS, all 7 builds, 2 m 20 s** |
| **P5 the ARM header gate** (`arm_all.cpp`) | RESUMING's command over all 214 headers of `tree_gate/include/shulib` | **passes**; a planted `int x = d;` in the same TU fails it (the negative control) |
| **P6 the team's file** (`team/main.cpp`, `team_proj/`, `team_make.log`) | §1.1's 19 lines as the ONLY source of a PROS project (the tree's Makefile, firmware and headers; no `src/` but it) | **links a V5 package** (`bin/hot.package.bin`); the one warning is vendored `liblvgl` via `main.h`; on the host, with the suite's flags (`-Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion`) and the shim, it compiles clean |
| **P7 the front check** (`probe_front_save/probe.log`) | `compareFronts` on 13 record pairs; `flipFront` (withdrawn — Re-check R2) | all 13 as the rule says (§1.4's table); `flipFront(flipFront(d))` encodes byte-identical to `d` |
| **P8 the save paths** (`probe_front_save/probe.log`) | load → check → hold → `saveRecordSafely` over `FakeFileStore`, then a reboot's `loadRecord` | eight cases (§2.A): while HELD, zero replace operations; KEEP OLD saves the turned record and the boot reads it as SAME FRONT (KEEP OLD withdrawn — Re-check R2); a cut write restores the earlier record; a crash mid-write is SAVE UNCONFIRMED and the LIBRARY's `loadRecord` falls to the backup's EARLIER record — **the drive program's boot does not, until SAFE-SAVE B2** (it reads only the record file; robot two's typed table is not a valid description, so it REFUSES to boot — Re-check R17, `amend/table_probe/`); a damaged record file is checked against the BACKUP |
| **P9 the watch writes nothing** (the shim's counters, every scenario; `objdump/`) | the declared ports' gearing, units, voltage, brake-mode and `brake()` calls; `pros::v5::Motor::Motor` disassembled from the vendored `firmware/libpros.a` (`vdml_motors.cpp.o`) | 0 of each on ports 6 and 7 in all ten new runs; the constructor calls `set_gearing` only when the gearset is not `invalid` and `set_encoder_units` only when the units are not — with neither given it stores the port and returns (HA-187) |
| **P10 the screen seam's sanitizer** (`tree_gate/src/common/screen_text.hpp`) | `printText()` reduced to `hal::screenSafeLine()` + the PROS print | all 7 builds pass (P4) with drive, competition, X-drive and `screen_paint.cpp` painting through it |
| **P11 the host suite on the overlay** (`build_suite_gate/`, `suite_build2.log`, `suite_syntax.log`) | CMake's own commands for all 210 test TUs (`compile_commands.json`), each syntax-checked against `tree_gate/`; the doc gates the suite's build runs first | **206 of 210 compile; the 4 that do not are exactly C1's** (`drivetrain_calibrator_test`, `calibrate_panel_test`, `median_nonfinite_test`, and `drive_characterizer_test` through `calibration_test_rig.hpp`'s `Answer::FrontFirst`). The doc gate STOPS the build twice: first `api_doc_tool: unterminated declaration ... kCompetitionDisabled = 1u << 0` (the tool reads `<<` as a template bracket — fixed with `0x01`/`0x02`/`0x04`), then **31 undocumented public entities** (29 screen literals in `calibrate_screen.hpp`, the two deleted special members) — the executor documents them or moves the literals into `detail` |
| **P12 the fence and the platform scan** (`scan/`) | `test/pros_adapter_fence_test.cpp` and `test/record_store_platform_scan_test.cpp` built alone over `tree_gate/` | **both pass** (11 cases, 1,280 assertions); with the FIRST draft's unfenced `hal/pros/calibrate.hpp` swapped in, the fence test FAILS: `calibrate.hpp: no fence push` |
| **P13 Part A as a pure addition** (`proposed3/`, `tree_a/`, `gate_check_a.log`, `build_suite_a/`, `build_suite_t/`, `xml_*.xml`, `p3_old/`, `p3_new/`) | the library files with the ADDITIVE panel and the calibrator untouched, over the tree with `src/` unchanged: the real `src` gate; the ARM header gate; the whole host suite (CMake's own compile and link commands — the doc gate bypassed, P11's 31 being Part A's to document) against the pristine copy's; today's program and the library program through the golden harness | **gate PASS (7 builds, 0 disallowed); ARM gate passes (214 headers); suite 2,231 / 2,231 cases on BOTH trees** (assertions 2,783,067 → 2,784,251: seven scans that walk more files and literals, every one passing — the panel's literal check, the retired-Teleop scan, SAFE-SAVE's scan, the two fence tests, the quoted-include and the PROS-include mirrors); **today's program on Part A's panel: 24 of 24 output files byte-identical** to `final_old/`; **the library program on Part A's panel: 34 of 34 byte-identical** to `final_new/` (the end state's) |
| **P14 the 2026-10-08 re-check** (`recheck_1008/`: `include/` + `arm_all.cpp` + `arm_neg.cpp`, `team_proj/` + `team_make.log`, `tree_gate/` + `gate_check.log`) | T4 Part 3's review-fix run's thirteen changed code files (six library headers, seven `src/` files, as of ~09:25) overlaid on `tree_a/`'s headers, on the team project and on the end state `tree_gate/` (its own scratch git, committed `57c2fed`) | **ARM header gate passes** (214 headers; the `int x = d;` control fails, exit 1); **the 19-line team project re-links** `bin/hot.package.bin` (one warning, vendored `liblvgl`); **`src_build_gate.py check` PASS** — all 7 builds link, 0 disallowed warnings, beacons asserted, 6 programs / 20 entry names matching `src/main.cpp`, 2 m 19 s |
| **P15 the gate's self-test on the end state** (`amend/gate_selftest/`, `amend/gate_selftest.log`; Re-check R1) | `recheck_1008/tree_gate` + case 26's Calibrate plant re-pointed at `src/robots/calibrate_facts.hpp`; unmodified, the plant dies on `_replace`'s exactly-once assertion (reproduced) | **`self-test: OK (211 detector cases, each against a real build)`, 11 m 12 s** |
| **P16 the revised Part A as a pure addition** (`amend/tree_a2/`, `amend/p3r/`, `suite_a2_*.log`, `gate_check_a2.log`, `arm.log`, `p3r_old/`; R2, R5) | the REVISED library (held screens, C-API watch read, facts changes) + the shim pieces as project code in `test/pros_shim/pros/` (`screen.h`, `misc.h`, `motors.h`, `rtos.hpp`'s hook), `src/` unchanged | **suite 2,231 / 2,231** (assertions 2,784,574); **src gate PASS** (7 builds, 0 disallowed); **ARM gate passes** (214 headers; control fails); **today's program on the revised panel: 33 of 33 files byte-identical** to `final_old/` |
| **P17 the revised goldens** (`amend/harness2.cpp`, `amend/run_new/`, `amend/classify2.py`; R2, R10-R12, R16, R18) | robot two's glue over `p3r/` on the A2 plant through the shim: the seven comparable scenarios plus `flip-again`, `flip-new`, `flip-disable`, `swap`, `swap-keep-old-equivalent`, `watch`, `watch-typo-again`, `watch-typo-save`, `undeclared`, `facts-*`; the harness decodes the card and scores a FORWARD command on the plant's wiring | residual 0 in all seven comparable; §0.5's re-check tables |
| **P18 the revised end state** (`amend/p2r/`, `amend/tree_end/`, `amend/run_end/`, `gate_check_end.log`, `make_calibrate_end.log`; R2) | C1's removal applied to the revision (`PanelButtonId::Front` → `SaveHeld`, `Back` removed, `FrontRule` gone, the draft's C1 calibrator); the end-state tree with case 26 re-pointed and Part A's shim pieces | goldens **69 of 69 byte-identical** to `run_new/`; **src gate PASS** (7 builds, 0 disallowed, 2 m 51 s); `make ROBOT=tank PROGRAM=calibrate` links, and its package carries `FRONT FLIPPED`, `MECHANISM PORT TURNED` and `MEASURES ITS OWN DRIVETRAIN`, but no `WHICH END FIRST`, `KEEP OLD` or `WHICH IS FRONT`; **205 of 210 test TUs compile**: the 4 CAL1 failures are exactly C1's re-pins, and the fifth (`dropped_autons_test`) is the probe tree's T4 10-08 header against its 10-06 test |
| **P19 the revised team file** (`amend/team2/`, `amend/team_proj2/`, `team2_make.log`; R11, R13) | §1.1's 21 lines as a PROS project's only source; on the host with the suite's flags and `-Werror`; an X-drive variant; the old kind type | **links a V5 package** (one warning, vendored `liblvgl`); host compile clean; the X-drive file compiles (so NOT A TANK is reachable); `hal::DrivetrainKind::Tank` no longer converts |
| **P20 the C-API watch read** (`amend/objdump/`; R6) | `vdml_motors.c.o`'s `motor_get_position`, `motor_get_gearing`, `motor_get_encoder_units` disassembled | each calls only the registry check, the port mutex, `__errno` and one `vexDeviceMotor*Get`; no `*Set` |
| **P21 robot two's typed table** (`amend/table_probe/`; R16, R17) | `tableToDescription(kRobotTwoChassis).valid()` | **false**: "LEFT inches per radian is 0" (track width and ratio UNSET) — robot two's drive never boots from its typed table |
| **P22 `printText` on the host** (`amend/printtext_probe/`; R7) | the end state's `screen_text.hpp` through the recording shim `screen.h`, suite flags, `-Werror` | compiles and runs: `'50? done - a?b ? end'`, a LARGE line cut to 18, the counter 2 |
| **P23 the D6 (a) goldens** (`d6a/harness3.cpp`, `d6a/run_new/`, `d6a/ctl_run/`, `d6a/classify_d6.py` + `.log`) | robot two's glue over `d6a/p3r/` (its facts with the typed ports) on the A2 plant through the shim: the 19 earlier scenarios and 14 new ones (§0.5); a control first (`amend/harness2.cpp` over unchanged `amend/p3r/` from `d6a/`) | the control reproduces `amend/run_new/` byte for byte (79 files, `watch-typo-*`'s stderr against `amend/run_typo_counters/`); **the fifteen goldens: 50 of 69 files byte-identical, every record among them; the other 19 differ only by D6's lines, residual 0**; `first-untyped` = `amend/run_new/one`; `typed-card-wins` = its untyped control (the scenario-name line of the harness's stderr aside). **Re-verified** (`d6a/v2/`): rebuilt from the final `harness3.cpp`, 143 of 143 against `d6a/run_new/`, the classifier's totals unchanged, residual 0 |
| **P24 the boot's vocabulary** (`d6a/boot_probe/boot_probe.cpp` + `.log`; suite flags, `-Werror`) | (0) `typedDriveDescription(calibrateFactsFor(t, m))` vs `tableToDescription(t)` for robot two's, the bench bot's and the X-drive's tables; (1-3) robot two's typed table made VALID (track width 12.5, ratio 1.0; ports and signs untouched) and its G3a-D7 flip, booted with no record through `decideBootSourceFrom` → `UseTable` → `robot::Robot` over the real `ProsDeviceProvider` and the shim → `RobotTwoDriver`, and each saved record (`one`, `typed-flip-new`, `flip-new`, `typed-flip-again`) booted `UseRecord`; a forward stick, then a turn; every port's millivolts; (4-5) `compareFronts` against the typed ports and against robot two's REAL 2026-09-25 record | (0) robot two L5/R5 byte-identical, 0 ports differing; bench bot and X-drive: none, none. (2) the typed table drives **10 of 10** motors forward, as `one`'s record does; `typed-flip-new`'s and `flip-new`'s records **0 of 10**, as the D7 table does. (3) **typed table vs `one`: forward 10/10 ports at the same mV, turn 10/10**; vs `typed-flip-new`: forward 0/10, turn 10/10 (a flipped front turns the same way, §8). (4) typed vs `one` SAME 10/10; vs `typed-flip-new` FLIPPED 10/10. (5) 09-25 vs typed table FLIPPED 8/8 (P7 row 3 reproduced); **09-25 vs the D7 table SAME 8/8; 09-25 vs `typed-flip-new`'s record SAME 8/8** — USE NEW after a camera-end push saves robot two's real front. **Re-verified:** re-run from the records' directory (`d6a/v2/run_new/`, byte-identical to `d6a/run_new/`), the full log is byte-identical (`v2/boot_probe.rerun.full.log`) |
| **P25 the end state** (`d6a/p2r/`, `d6a/run_end/`) | C1's removal applied to the D6 library (the same edit as `p3r`'s, the p3r→p2r difference unchanged) | **143 of 143 files byte-identical** to `d6a/run_new/` (33 scenarios); **re-verified** with the final harness (`v2/run_end/`: 143 of 143) |
| **P26 the screen strings** (`d6a/screen_probe/` + `.log`) | the five new literals through `screenText` at COMPILE time and at run time; `your typed ports DRIVE with port N` for N = 1..21; controls: a 51-column SMALL line and a `%` | `kFlipWhatTyped` 36 of 36 (MEDIUM), `kFlipNewTyped` 50 of 50, `kFlipWarnTyped` 48 of 50, `kNoteTypedSame` 36 of 50, `kNoteFixTyped` 41 of 50 — all PASS; the runtime line at most 35 of 50, 0 failures; both controls refused at compile time; `panelLiteralsFit()` holds with the five in `kAll`. **Re-verified:** re-compiled with the suite's flags and `-Werror`, the log byte-identical (`v2/screen_probe.log`) |
| **P27 the facts checks** (`d6a/facts_probe/facts_probe.cpp`; `d6a/run_new/facts-typed-*`) | `factsProblem`, `hasTypedDrive`, `typedDriveDescription`, `typedDrivePortsFrom` as `static_assert`s: the team file's facts, robot two's derived facts, the bench bot's and the X-drive's, a duplicate on one side and across sides, 22 / −22 / −128, one side typed, 0 slots, a mechanism problem first, the config round trip; and the three refusals through the program | every `static_assert` holds; through the program: `FACTS REFUSED` naming the entry and its value, before any motor is made. **Re-verified:** re-compiled with the suite's flags and `-Werror` over BOTH library forms (`v2/facts_probe`, `facts_probe_end`): every `static_assert` holds |
| **P28 today's program on the D6 panel** (`d6a/build_old.sh`, `d6a/p3r_old/`) | today's `src/programs/calibrate_program.cpp` over `d6a/p3r/`'s `calibrate_panel.hpp`, the nine old scenarios | **33 of 33 files byte-identical** to `final_old/` (nocard's one line compared with the probe's card directory normalized); **re-verified** (`v2/p3r_old/`: 33 of 33) |
| **P29 the host suite** (`d6a/tree_a3/` = `amend/tree_a2/` + the two D6 headers, scratch git `42fdec3`; `build_suite_a3/`, `suite_a3_run.log`; control `tree_a2ctl/`, `suite_a2ctl.xml`, `suite_a3.xml`) | the WHOLE suite, CMake's own build (doc gate bypassed as P13/P16), and a control build of `amend/tree_a2` to attribute the assertion change | **2,231 of 2,231 cases** (3 skipped, as before); assertions 2,784,574 → 2,784,865: +229 in `calibrate_panel_test` P9 (the literal scan walks the five new literals) and +62 in the retired-Teleop scan (more lines) — every other case's count identical (re-read case by case from the two XMLs in the continuation) |
| **P30 the gates** (`d6a/gate_check_a3.log`, `gate_check_end3.log`, `gate_selftest_end3.log`, `arm_a3.log`, `arm_neg_a3.log`, `arm_end3.log`, `make_calibrate_end3.log`, `strings_end3.log`; `d6a/tree_end3/` = `amend/tree_end/` + the D6 library and facts, scratch git `b3bd1f1`) | the real `src_build_gate.py check` on the Part A tree and on the end state; `self-test` on the end state (case 26's anchor `calibrateFactsFor(kChassis, kMechanisms)` is still there once); the ARM header gate on both; `make ROBOT=tank PROGRAM=calibrate` and `arm-none-eabi-strings` | **check PASS on both, all 7 builds, 0 disallowed warnings** (3 m 44 s, 3 m 54 s); **self-test OK (211 detector cases), 13 m 25 s**; **ARM gate passes over all 214 headers on both, its `int x = d;` control fails**; the package links (text 76.19 → 78.81 KB; 8 warnings, all vendored `liblvgl`) and carries `checked against your typed drive ports`, `FRONT MATCHES YOUR TYPED DRIVE PORTS`, `typed ports: the end you led is BACK`, `FIX YOUR TYPED DRIVE PORTS`, but no `WHICH END FIRST`, `KEEP OLD` or `WHICH IS FRONT` (every log re-read in the continuation; the ARM TUs hold 214 `#include`s each) |
| **P31 the team file** (`d6a/team_proj3/`, `team3_make.log`; `d6a/team3/`) | §1.1's 21 lines, unchanged, as a PROS project's only source over the D6 headers; on the host with the suite's flags and `-Werror`; two variants with typed ports (the `.typedDrive` line; `typedDrivePortsFrom` of a config description) | **byte-identical file; links a V5 package** (`bin/hot.package.bin`, text 73.76 → 76.40 KB; one warning, vendored `liblvgl`); **host compile clean**; both typed variants compile clean with their `static_assert`s. **Re-verified** (`d6a/v2/`): the file equals §1.1's and §9.3's listings byte for byte (extracted from this brief); all three re-compile clean on the host over BOTH library forms (`p3r`, `p2r`). `main_config.cpp` passed a bare `hal::DrivetrainDescription`, so the chapter's spelling was probed literally: `v2/team_config_real.cpp`, a real `inline constexpr config::RobotConfig kRobot` tank and `.typedDrive = calibration::typedDrivePortsFrom(kRobot.drivetrain.tank.description)` — host-clean over both forms, and **links a V5 package** as a project's only source (`v2/team_proj_cfg/`, text 76.39 KB; one warning, vendored `liblvgl`) |
| **P32 T4 Part 3 AS LANDED** (`bdcbc7c`, 2026-10-08 14:52; `d6a/landed_ctl/` = the commit, `landed_a/` = it + `tree_a3/`'s CAL1 files (Part A's form, `src/` untouched), `landed_end/` = it + `tree_end3/`'s CAL1 files only — never its 09:25 overlay of T4's files; scratch gits `e294e22`, `5d9fd64`; logs in `d6a/v2/`) | the continuation's launch-time re-run of every proof D6 (a) could break, on the tip CAL1 now builds from: the golden harness built against the landed tree (both forms) and today's program; the WHOLE suite on `landed_a/` against a control build of `landed_ctl/` (the same doc-gate bypass); `src_build_gate.py check` on both and `self-test` on the end state; the ARM header gate on both (its control); `make ROBOT=tank PROGRAM=calibrate` and `arm-none-eabi-strings` on the end state | **goldens 143 of 143 byte-identical to `d6a/run_new/` on BOTH forms** (`v2/run_new_landed/`, `run_end_landed/`); **today's program on the D6 panel 33 of 33** (`v2/p3r_old_landed/`); **suite 2,253 of 2,253 on both** (3 skipped; T4's fix run added 22 cases), assertions 2,783,363 → 2,785,161, every change in the scans that walk more files and literals (the panel's literal check +937, the retired-Teleop scan +786, the buttons scan +12, five fence cases +56, SAFE-SAVE's scan +7) — no case's verdict changed; **`check` PASS on both, all 7 builds, 0 disallowed warnings** (2 m 49 s, 2 m 32 s); **`self-test` OK (211 detector cases), 12 m 13 s**; **ARM gate passes over all 214 headers on both, its `int x = d;` control fails**; the package links (text 78.80 KB; 8 warnings, all vendored `liblvgl`) and carries the typed words and none of `WHICH END FIRST`, `KEEP OLD`, `WHICH IS FRONT` |
| **P33 the check's goldens** (`d6a/v3/`: `harness4.cpp`, `build.sh`, `run.sh`, `cmp.sh`, `run_new/`, `run_end/`, `compare_v3.log`, `classify_v3.log`; built against `d6a/landed_ctl/` = `bdcbc7c`) | robot two's glue (facts = `withTypedDriveFrom(kCalibrateFacts, kChassis)`: no typed ports) over `v3/p3r/` and `v3/p2r/` on the A2 plant through the shim: D6 (a)'s 33 scenarios + `typed-first` (the typed ports from robot two's table made valid, P24's numbers), `typed-partial`, `empty-watch-untyped`, `undecodable-untyped`, `bakonly-garbage-untyped`/`-typed`, `emptyfile-untyped`, `nocard-watch-again`, `typed-nocard`, `facts-typed-rightonly` (43) | **end state = Part A form, 185 of 185 files**; robot two's 19 untyped scenarios = `amend/run_new/` (80 of the 81 files with a counterpart; `nocard`'s serial differs by C3's `card :` line); side-by-side vs today's program residual 0 in all seven, `amend/classify2.log`'s counts; the typed scenarios vs `d6a/run_new/` 56 of 60 (the 4: a name line, C3's reason lines ×2, C4's lines); `empty-watch-untyped` = `d6a/run_extra/`'s; every new scenario as §0.5's check table says |
| **P34 the mapping's parity** (`d6a/v3/parity_probe/` + `.log` ×2; the suite's flags, `-Werror`, both forms) | `withTypedDriveFrom(calibrateFactsFor(t, m), t)` against `tableToDescription(t)` and its `valid()`: every RobotId's table; robot two's table made valid, then broken one fact at a time (track width, ratio, wheel, IMU unset, IMU on drive port 13, cartridge unset, unsigned, port 11 on both sides) | **PASS, 12 of 12**: typed ports = the boot's sides exactly when `valid()`, none otherwise (robot two, the bench bot and the X-drive: none); every result passes `factsProblem` |
| **P35 the mutations** (`d6a/v3/mutate_v3.py` + `mutate_v3.log`, `mutate_v3_m41.log`) | D6 (a)'s A2 28–42 re-anchored on `v3/p3r/`, the check's 43–54, Part B's mapping 8–11; each on a fresh copy, built into the harness, run on ALL 43 scenarios against `v3/run_new/` (B's also through the parity probe) | **31 of 32 killed by the goldens, each naming every scenario that differs**; B8 (the D6 (a) draft's mapping) by 14 of robot two's goldens and the parity probe; **B11 (the glue skips the mapping) survives every golden — equivalent on robot two, whose table is not valid — and fails Part B test 4's source pin** |
| **P36 the gates on T4 Part 3 as landed + the fixes** (`d6a/v3/landed_a3/` = `d6a/landed_a` + the two changed headers, scratch git `230ccf2`; `landed_end3/` = `d6a/landed_end` + the two headers, the facts and the glue, `a83f659`; `team_proj4/`; `gates_v3.sh`) | the ARM header gate on both (its control); `src_build_gate.py check` on both; `make ROBOT=tank PROGRAM=calibrate` + `arm-none-eabi-strings`; the 21-line team file as a V5 project's only source; `self-test` on the end state | **ARM gate passes over all 214 headers on both, the `int x = d;` control fails**; **`check` PASS on both, all 7 builds, 0 disallowed warnings** (2 m 50 s, 2 m 41 s); the package links (text 78.80 → 80.55 KB; 8 warnings, all vendored `liblvgl`) and carries `FRONT MATCHES, SOME TYPED PORTS DIFFER`, `NO SD CARD in the brain`, `there is no SD card in the brain`, `EMPTY (0 bytes read)` and the D6 words, none of `WHICH END FIRST`, `KEEP OLD`, `WHICH IS FRONT`; **the team file links a V5 package** (byte-identical file; text 77.58 KB; one warning, vendored `liblvgl`); **`self-test` OK (211 detector cases), 11 m 52 s** — re-run whole after the first run was cut off by the usage stop and left one plant in `src/build_selection.hpp` (restored by `git checkout`; the tree is clean after the re-run) |
| **P37 the suite and today's program** (`d6a/v3/build_suite_a3/`, `suite_a3_run.log`, `suite_a3.xml`; `p3r_old/`, `p3r_old.log`) | the WHOLE host suite on `landed_a3/` (CMake's own build, the doc gate bypassed as P13/P16/P29/P32), compared case by case with `d6a/v2/suite_landed_a.xml`; today's program over `v3/p3r/`'s panel, the nine old scenarios | **2,253 of 2,253 cases** (3 skipped); assertions 2,785,161 → 2,785,243: +52 in `calibrate_panel_test` P9 (the literal scan walks `kNoteTypedPartial`), +30 in the retired-Teleop scan (more lines) — every other case identical, no verdict changed; **today's program 33 of 33 files byte-identical** to `d6a/v2/p3r_old_landed/` (itself = `final_old/`) |

**What the probes still lack against §1 (the executor builds these; the probes did not need them):** §1.6's
`RunKind`; the host rig on `hal::fake` and every test of §2 (the probes ran through the PROS shim); the 31
undocumented public entities P11 found (plus the re-check's new ones: `HoldReason`, `FinishFacts`,
`FactsProblem` and the held screens' literals). The draft's `proposed2/`/`proposed3/` are superseded by
`amend/p2r/`/`amend/p3r/`, which carry §1.4's three no-record cases, the NOTE line, the held screens, the facts
checks and the C-API read; and they by `d6a/p2r/`/`d6a/p3r/` for D6 (a)'s typed check (P23–P31); and those by
`d6a/v3/p2r/`/`v3/p3r/` for the check's fixes (P33–P37). D6 (a)'s probes
also lack the host rig: the typed cases ran through the PROS shim, and §2.A's tests 2 and 6 re-state them on the
fakes.

## 1. Scope — IN

### 1.1 What a team writes — VERBATIM, compiled, counted (D3)
**A team's Calibrate is this one file** (its project's `src/main.cpp`, or the Calibrate build's entry file; PKG1
decides where a starter project keeps it — §4). **21 lines, 5 of them comments** (Re-check R11/R13: the draft's
19 lines, the kind now `config::DrivetrainKind`, and the two-line rule for `.mechanisms`). Compiled and linked into
a V5 package as the ONLY source of a PROS project (P6; re-linked as P19, `amend/team_proj2/`), and on the host with
the suite's flags and `-Werror` (`amend/team2/`):

```cpp
// Our robot's Calibrate. We state only what the robot cannot measure about itself;
// shulib Calibrate finds everything else (which ports drive, their directions, the sides,
// the FRONT -- the end you push first -- the track width and the wheel scale).
#include "main.h"
#include "shulib/hal/pros/calibrate.hpp"

using namespace shulib;

constexpr calibration::CalibrateFacts kOurRobot{
    .drive = config::DrivetrainKind::Tank,
    .cartridge = hal::MotorCartridge::Blue,  // the insert colour of the DRIVE motors
    // EVERY motor that is not a drive motor. A motor left out is a drive candidate,
    // and one the push turns gets powered (3, 6, 9 V) like a drive motor.
    .mechanisms = {{6, "left claw"}, {7, "right claw"}},
};

void initialize() { hal::pros::calibrateInitialize(); }
void disabled() { hal::pros::calibrateDisabled(); }
void competition_initialize() { hal::pros::calibrateConnected(); }
void autonomous() { hal::pros::calibrateAutonomous(); }
void opcontrol() { hal::pros::runCalibrate(kOurRobot); }
```

- **What each line is.** The three facts are the only things a person states (§0.4). `.drive` is the kind the
  team HAS (Tank, XDrive, HDrive); only Tank is measured. `.mechanisms` lists EVERY smart motor that is not a drive
  motor (a 3-wire solenoid is not a motor and is never a candidate), with the name the log and the screen will use.
  Anything left out is a drive candidate, and one the push turns is pulsed at 3, 6 and 9 V (R11, measured). The five entry points are PROS's own, each one call. Nothing else is configurable from
  here: every threshold is the calibrator's (HA-136), the screen is the library's.
- **The file is unchanged by D6 (a)** (measured, §0.6 P31: byte-identical, compiled on the host with the suite's
  flags and `-Werror`, re-linked as a V5 package's only source). The typed drive ports are an OPTIONAL member the
  file does not name. A team whose drive program already types its drive ports MAY add one line after
  `.mechanisms`, in the chassis table's own spelling (compiled, `d6a/team3/main_typed.cpp`):
  `.typedDrive = {.left = {-11, 12, -13, 14, -15}, .right = {20, -19, 18, -17, 16}},` — or, from a
  `config::RobotConfig`, `.typedDrive = calibration::typedDrivePortsFrom(kRobot.drivetrain.tank.description),`
  (compiled with a real `inline constexpr config::RobotConfig kRobot` tank and linked as a V5 package's only source,
  `d6a/v2/team_config_real.cpp`, P31; the `kRobot` must be a constant expression, as `src/robots/xdrive_config.hpp`'s
  is, because `kOurRobot` is `constexpr`). Calibrate never calibrates from them: with no usable record on the card
  they are the front check's reference (§1.4). A team's line is taken as stated — the team is saying its drive
  program drives from those ports; only a repository robot's table is filtered by the boot's own test (below).
- **What a team gets wrong, and what it then sees** (each a host test, §2.A):
  - kind or cartridge not stated → the red `CANNOT START` screen headed `DRIVE KIND UNSET` or `CARTRIDGE UNSET`
    before any motor is made, with the fix ("state it in the robot's facts, rebuild, upload"); a kind other than
    Tank → `NOT A TANK -- the facts say x-drive …` (R13, measured: `amend/run_new/facts-xdrive`);
  - a mechanism NAMED with port 0 (a placeholder), a port outside 1..21, or a port listed twice → `FACTS
    REFUSED`, naming the entry: `mechanisms[2] = {0, "intake"}` (`factsProblem()` returns `{why, entry}`, also
    usable in a `static_assert`, as robot two's does; R18, measured);
  - drive motors listed as mechanisms until fewer than two candidates remain → `TOO FEW MOTORS`, whose fix names
    the declared ports: `declared as mechanisms: 6 7 11 … -- is any a DRIVE motor? else re-seat the drive cables`
    (R18, measured);
  - a mechanism motor NOT listed → it is a candidate. If the push does not turn it: `port N did not turn with the
    push: not a drive motor, never powered -- if it is a mechanism, declare it`. If the push turns it, it is pulsed
    3, 6 and 9 V and then LEFT OUT (free-spinning) or refused (loaded). Either line now adds "or a MECHANISM motor
    -- declare it" (R11, measured: `amend/run_new/undeclared`);
  - a declared port with no motor on it → a census line `port N: <name> (declared) -- the census found NO MOTOR
    there`, and the run goes on;
  - a port declared as a mechanism that is really a DRIVE motor (a mistyped port, or a mechanism role wired to a
    drive cable) → the push turns it, the WATCH names it at DONE, and **the save WAITS** on MECHANISM PORT TURNED
    (D4 as amended, R12). SAVE keeps it declared and saves without it; RUN AGAIN saves nothing and the card keeps its
    record. Measured: `watch-typo-again` leaves the card byte-identical. With no record on the card and typed ports
    that drive with that port, the held screen says `your typed ports DRIVE with port 12` (D6 (a), measured:
    `d6a/run_new/typed-watch-typo`; again in `d6a/v3/run_new/`, where the serial log also names port 12 as typed
    but not in this push's record — the check of D6 (a), C4);
  - **(D6 (a)) typed drive ports with a port outside 1..21, a port listed twice (on one side, or once on each — by
    |port|, `tableConsistent()`'s rule), or only ONE side typed** → `FACTS REFUSED`, naming the entry, before any
    device is made. Measured (`d6a/run_new/facts-typed-*`): `a typed drive port is listed twice (one side, or
    both): typedDrive.right[4] = 11`; `a typed drive port is not a smart port (1..21): typedDrive.left[2] = -22`;
    `typed drive ports must list BOTH sides -- the RIGHT is empty: typedDrive.right[0] = 0`, and (the check of D6
    (a)) `… -- the LEFT is empty: typedDrive.left[0] = 0` (`d6a/v3/run_new/facts-typed-rightonly`). A mechanism
    problem is reported first (P27).
- **The facts type** (`calibration/calibrate_program.hpp`), all members defaulted, so a team names only what it
  states (designated initializers; an omitted member is never `-Wmissing-field-initializers`, P6):
  ```cpp
  struct MechanismPort { std::uint8_t port = 0; const char* name = nullptr; };   // 0 = unused
  inline constexpr std::size_t kMaxMechanismPorts = hal::kMaxSmartPorts;         // 21: never refuses a real robot
  struct TypedDrivePorts {                     // D6 (a): the chassis table's spelling, signed, 0 = an unused slot
      std::int8_t left[hal::kMaxSmartPorts]{};
      std::int8_t right[hal::kMaxSmartPorts]{};
  };
  struct CalibrateFacts {
      std::optional<config::DrivetrainKind> drive{};   // UNSET refuses; Tank is the only kind measured
      std::optional<hal::MotorCartridge> cartridge{};  // UNSET refuses: never a default cartridge
      MechanismPort mechanisms[kMaxMechanismPorts]{};
      const char* robot = nullptr;                     // optional: a name for the log
      std::uint8_t imuPortStated = 0;                  // optional cross-check; the registry decides
      TypedDrivePorts typedDrive{};                    // optional (D6 (a)): the reference with no record; LAST
  };
  enum class FactsList : std::uint8_t { Mechanisms, TypedLeft, TypedRight };      // which list `entry` indexes
  struct FactsProblem { const char* why = nullptr; std::size_t entry = 0; FactsList list = FactsList::Mechanisms; };
  [[nodiscard]] constexpr FactsProblem factsProblem(const CalibrateFacts&) noexcept;      // mechanisms, then typed
  [[nodiscard]] constexpr bool hasTypedDrive(const CalibrateFacts&) noexcept;             // any non-zero entry
  [[nodiscard]] constexpr hal::DrivetrainDescription typedDriveDescription(const CalibrateFacts&) noexcept;
  [[nodiscard]] constexpr TypedDrivePorts typedDrivePortsFrom(const hal::DrivetrainDescription&) noexcept;
  ```
  The two optional members `robot` and `imuPortStated` exist for robot two (its banner prints its name, and its
  typed IMU port is a printed cross-check today, `calibrate_program.cpp` :1155-1163); a team leaves them out.
  **`typedDrive`** (D6 (a)) is optional for every team; a repository robot's is filled by the glue's
  `withTypedDriveFrom` only from a table the boot can drive from — robot two's is not one yet, so it carries none
  (the check of D6 (a), C1). It is appended LAST, so a
  designated initializer that names the earlier members in order still compiles. `typedDriveDescription()` fills
  only a description's two sides (zero slots skipped, the order kept) — exactly the sides `compareFronts` reads;
  `typedDrivePortsFrom()` is its inverse for a team on a `config::RobotConfig`. All four are `constexpr` and
  allocation-free; probed at compile time (`d6a/facts_probe/`, P27).

**Robot two's Calibrate** (Part B) — the same library call with facts derived from its tables, so nothing is
typed twice (D3):
- `src/programs/calibrate_program.cpp` becomes **27 lines** (P3, P4 built it; 24 before the check of D6 (a) gave
  `runCalibrate` its typed-port line — `d6a/v3/p2r/`, built in P36):
  ```cpp
  // calibrate_program.cpp — "shulib Calibrate" for the robot this build carries (slot 4): the
  // standard Calibrate (include/shulib/hal/pros/calibrate.hpp, chunk CAL1) with this build's facts
  // (src/robots/calibrate_facts.hpp). Until CAL1 this file WAS the program (1,387 lines).
  
  #include "programs/calibrate_program.hpp"
  
  #include "robots/calibrate_facts.hpp"
  #include "shulib/hal/pros/calibrate.hpp"
  
  namespace shulib::bench {
  namespace {
  /// This build's stamp: the banner prints it whole, the screen's header its time.
  constexpr const char* kBuildStamp = __DATE__ " " __TIME__;
  }  // namespace
  
  void calibrateInitialize(const char* variantBeacon, const char* /*competitionStatusBits*/) {
      hal::pros::calibrateInitialize(variantBeacon, kBuildStamp);
  }
  void calibrateDisabled() { hal::pros::calibrateDisabled(); }
  void calibrateConnected() { hal::pros::calibrateConnected(); }
  void calibrateAutonomous() { hal::pros::calibrateAutonomous(); }
  void runCalibrate() {
      const calibration::CalibrateFacts facts = withTypedDriveFrom(kCalibrateFacts, kChassis);
      hal::pros::runCalibrate(facts, kBuildStamp);
  }
  
  }  // namespace shulib::bench
  ```
  and `calibrate_program.hpp` declares those five (21 lines, comments included).
- `src/robots/calibrate_facts.hpp` (new, ~77 lines with `withTypedDriveFrom` — ~70 before the check of D6 (a) —,
  PROS-free so the host suite reads it for every robot):
  `calibrateFactsFor(const ChassisTable&, const MechanismTable&)` → `CalibrateFacts` (kind
  `config::DrivetrainKind::Tank` — a `ChassisTable` is a tank's, and the chain builds Calibrate only for
  `ROBOT=tank`, `build_selection.hpp` :89-90, Re-check R13; the table's cartridge, `Unset` staying unset; every
  motor entry's port magnitude and name; the table's `robot` name and typed `imuPort`), and `inline constexpr
  CalibrateFacts kCalibrateFacts = calibrateFactsFor(kChassis, kMechanisms);` with
  `static_assert(calibration::factsProblem(kCalibrateFacts).why == nullptr, …)` (R18). It names only `kChassis`/`kMechanisms` — the build's selection — so the `src` gate's step 0 is
  satisfied (P4) and a generated robot's Calibrate needs nothing here (§1.7). **And (D6 (a), as the check of D6 (a)
  corrected it, C1) `withTypedDriveFrom(facts, table)`**: `facts` with `typedDrive` set from the description the
  BOOT would drive from with no record on the card — `tableToDescription(table)`, the boot's own function — only
  when that description is `valid()`, the boot's own test (`chooseBootSource`: no decoded record and a valid table
  → `UseTable`, `boot_check.hpp` :439-442), and none otherwise. It is not `constexpr` (neither function is), so the
  glue applies it where the program starts; `kCalibrateFacts` and its `static_assert` stay as they are. No copy of
  the boot's rule exists to drift; Part B's facts test still pins it against `tableToDescription` for every robot's
  table (§2.B test 2; measured: P34, 12 of 12). **Robot two's table is not valid (its track width and ratio are
  UNSET, P21), so its facts carry NO typed ports** — no boot drives it from that table, and its first Calibrate on
  an empty card gets the FIRST RECORD note, as in the revision (P33). *(The D6 (a) draft copied every SIGNED table's
  ports, valid or not, so robot two carried LEFT −11 +12 −13 +14 −15 | RIGHT +20 −19 +18 −17 +16 — the 15/16 end,
  opposite its 2026-09-25 record — and its empty-card check rewarded the wrong push: withdrawn, the check's C1.)*

### 1.2 The library (Parts A1 and A2) — eight files, every one host-tested; `amend/p3r/` is the draft of each (re-check; `proposed3/` superseded), `d6a/p3r/` where D6 (a) changed it (`calibrate_program.hpp`, `calibrate_panel.hpp`), `d6a/v3/p3r/` where the check of D6 (a) changed those two
**Which part builds which (D5 (d), Re-check):** A1 = items 1-5 and 8 (the seams, the painter, the front check,
the shim pieces); A2 = items 6-7 and the panel's additive change (the program, the shell, the held screens).
**D6 (a), split the same way:** A1 — `front_check.hpp`'s header sentence (the reference may be the typed ports) and
test 3's two typed-table pairs (`compareFronts` itself is unchanged); A2 — the typed facts and their checks, the
reference rule and every word of §1.4's typed branch in `calibrate_program.hpp`, `FinishFacts::typedPorts` and the
five literals in `calibrate_panel.hpp`, tests 2, 6, 10 and 11's typed cases, mutations 28-42; B — robot two's
mapping in `src/robots/calibrate_facts.hpp`, its parity test, mutations 8-10, and the Station G and chapter
sentences (§3.2). **The check of D6 (a), split the same way:** A2 — the per-file reason lines, the no-card words
(the `card :` line and the discards), the exact/partial typed NOTE and the per-port lines, the sixth typed literal
(`kNoteTypedPartial`), the check's golden scenarios and test cases, mutations 43-54; B — `withTypedDriveFrom` (the
mapping through the boot's own `tableToDescription`/`valid()`) and the glue's line, the parity test's
valid/invalid cases, mutations 8-11 (11: the source pin), and Station G's robot-two sentence (the chapter's is
withdrawn, §3.2). A1 is unchanged.
Each new header: PROS-free unless it is under `hal/pros/` (and then behind the fence: `#pragma GCC diagnostic push`,
EXACTLY `-Wshadow` and `-Wsign-conversion` ignored, the `pros/` includes, `pop`, before `namespace shulib` —
`test/pros_adapter_fence_test.cpp` :71-140 reads every `hal/pros/*.hpp`; the first draft lacked it), sim-free,
allocation-free, every public member `///`-documented (the coverage gate), internals in a namespace named exactly
`detail` (`tools/api_doc_tool.py` :543-548 hides only that name; the first draft's `program_detail` /
`screen_detail` would have reached the reference).

1. **`include/shulib/hal/brain_screen.hpp` (NEW, ~125 lines)** — `hal::IBrainScreen`: `fillRect`, `outlineRect`,
   `eraseRect`, `eraseAll`, `print(size, x, y, pen, eraser, text, cols, padTo)`, `touch()` → `ScreenTouch {error,
   releaseCount, x, y}`, `prints()`. Not one of F4's ten (an additive device seam, as `ILineDisplay` is). And
   **`hal::screenSafeLine()` — THE sanitizer, moved verbatim** from `src/common/screen_text.hpp`'s `printText()`
   body (`'%'` and control bytes → `?`, a UTF-8 en/em dash → `-`, any other non-ASCII character → one `?`, cut at
   `cols`, padded to `padTo`, never past `cols`): the screen's one rule outside Bench Tests (Re-check R7), now the library's. `print()` MUST pass its
   text through it, so a caller can never hand VEXos a format character.
2. **`include/shulib/hal/fake/fake_brain_screen.hpp` (NEW, ~105 lines)** — records every call as one text line
   (the same shape as the probe shim's, `shim_extra/pros/screen.h`), scripts touches, counts prints.
3. **`include/shulib/hal/pros/brain_screen.hpp` (NEW, ~90 lines, fenced)** — `hal::pros::ProsBrainScreen`: each
   call is today's PROS sequence (`screen_set_pen`/`set_eraser` then draw; `print_at(…, "%s", line)`; the touch with
   `E_TOUCH_ERROR` reported as an error), as `calibrate_program.cpp` :283-345 and :1010-1019 make them now.
4. **`include/shulib/calibration/calibrate_screen.hpp` (NEW, ~385 lines)** — the palette (with today's contrast
   notes), `CalibratePainter` (today's `PanelPainter`, :352-446, unchanged rules: a full repaint on a change of
   screen or place, a colour or button change at once, a live number at most every 0.2 s — PROS leaks each print —
   and in-place updates padded, never erased), and the program's own screens (starting, cannot start, not driver
   control, program fault, the competition-state screen) with every literal checked against its budget at compile
   time, as today's `text::kAll` (:452-509) is.
5. **`include/shulib/calibration/front_check.hpp` (NEW, ~110 lines)** — `FrontVerdict {Same, Flipped,
   Undetermined}`, `FrontCheck` (the counts), `compareFronts(earlier, now)` (§1.4). **No `flipFront`** (Re-check
   R2): nothing is ever turned to another front. Its header says what FLIPPED can and cannot mean: a left/right
   cable swap gives the same record as the other end leading (R10). **(D6 (a), A1)** It also says that `earlier`
   is the card's record or, with none, a description holding only the TYPED drive ports — `compareFronts` reads
   only sides and signs, so it needs no change; test 3 gains robot two's typed table against the plant's front-led
   record (SAME, 10 of 10) and against the other-end record (FLIPPED, 10 of 10) — P24.
6. **`include/shulib/calibration/calibrate_program.hpp` (NEW, ~1,050 lines; +~160 with D6 (a), +~65 with the check of D6 (a) — 1,351 → 1,417 in `d6a/v3/p3r/`; the probe's draft
   goes 1,189 → 1,351 lines, `d6a/p3r/`)** — `MechanismPort`,
   `CalibrateFacts`, `factsProblem()`, `declaredMechanism()`; **(D6 (a)) `TypedDrivePorts`, `FactsList`,
   `hasTypedDrive()`, `typedDriveDescription()`, `typedDrivePortsFrom()` (§1.1) and the reference rule and its
   words (§1.4)**; the three competition bits (`0x01` disabled, `0x02`
   autonomous, `0x04` connected — written as hex, landmine 9); `CalibrateIo` (four calls: `line(text)` — one serial
   line WITHOUT its tag; `competitionStatus()`; `delayMs(ms)`; `readShaftDegrees(port, &deg)` — the watch's read,
   HA-187); `CalibrateDevices {IDeviceProvider&, IDeviceRegistry&, IFileStore&, IBrainScreen&, CalibrateIo&}`;
   `CalibrateProgram(facts, devices, buildStamp)` with `run()`; `calibrateBootBanner(io, label, status, stamp)`
   for `initialize()`; `calibrateStateScreen(screen, io, state, why)` for the other three entry points. **`run()` is
   today's `runCalibrate()` (:1061-1386) in its order** — banner, census, the declared-port drop, the stated facts'
   refusals, one motor per port POSITIVE (now `devices.motor(port, cartridge)`: the provider's registry check and
   its per-device catch replace the inline `try`), the IMU (the provider starts its calibration once, HA-108), the
   calibrator, the touch baseline from a good read, the 10 ms loop with the field-disable exit, RUN AGAIN as a new
   calibrator and a new gate, the program-fault catch, and the all-stop guard on every C++ exit — with the changes
   §1.3 lists, and no others.
7. **`include/shulib/hal/pros/calibrate.hpp` (NEW, ~130 lines, fenced)** — `hal::pros::ProsCalibrateIo`
   (`printf("[R3D] %s\n")` + `fflush` — the tag every worksheet and capture reads; `competition_get_status()`;
   `pros::delay`; the watch's read through PROS's C API by port — `pros::c::motor_get_position(port)` converted by
   `motor_get_encoder_units(port)` and `motor_get_gearing(port)`, NO motor object of any kind (Re-check R6: the
   vendored reads call only firmware getters, P20), false when PROS answers `PROS_ERR_F` or an unknown unit; its
   includes are `pros/misc.h`, `pros/motors.h`, `pros/rtos.hpp`), and the team's five entry functions:
   `calibrateInitialize(label = nullptr, stamp = __DATE__ " " __TIME__)`, `calibrateDisabled()`,
   `calibrateConnected()`, `calibrateAutonomous()`, `runCalibrate(facts, stamp = …)` — the last composes
   `ProsDeviceRegistry`, `ProsDeviceProvider{registry}`, `ProsFileStore`, `ProsBrainScreen`, `ProsCalibrateIo` on its
   own frame and runs the program. A `static_assert` ties `calibrate_program.hpp`'s three competition bits to
   PROS's `COMPETITION_*` values. Its header says: include it in ONE translation unit (its default stamp is that
   unit's `__DATE__`/`__TIME__`).
8. **The shim** (`test/pros_shim/pros/`): a `screen.h` that RECORDS every draw call and scripts the touch (the
   probe's `shim_extra/pros/screen.h`, with the `#error` + `SHULIB_HOST_PROS_SHIM` gate every shim header carries —
   the fence test checks it); the competition status calls (`misc.h`); the three C-API motor READS (`motors.h`:
   `motor_get_position`, `motor_get_encoder_units`, `motor_get_gearing`, over the same per-port state as the shim's
   `pros::Motor`, writing nothing and counting calls — R6; `amend/shim_extra2/pros/motors.h`); a time-advance hook in
   `rtos.hpp` (behaviour unchanged while none is installed); and a host `main.h` beside the shim that declares PROS's five entry points with C linkage, as
   `include/main.h` :42-48 does (P6's `shim_extra_team/main.h`), so the team file compiles in the suite exactly as
   written. They exist for the two adapters' host tests, the team file's compile (§3.1) and Part B's golden test;
   A2's program tests do not need them (they run on `hal::fake`).

**Changed library file in A2 — `calibration/calibrate_panel.hpp`, ADDITIONS ONLY** (Re-check revision, `amend/p3r/`;
the draft's `proposed3/` form is superseded). **New enums:**
- `FrontRule {AskedAfterPush, ByPush}`;
- `FrontChoice {None, New}` (no `Kept`, R2);
- `HoldReason {None, FrontFlipped, MechanismTurned}`.

**A new struct**, `FinishFacts {front, hold, holdLine1, holdLine2, note, typedPorts}`: what the program tells the
finished and held screens. `typedPorts` (D6 (a), default `false`) says the FRONT FLIPPED screen's reference was the
TYPED drive ports, so the view paints their words (§1.4).

**APPENDED enumerators**, none of which anything switches on: `PanelScreen::Held`, `SaveOutcome::Held`,
`TouchAction::SaveHeld`.

**Two DEFAULTED parameters** on `viewFor()` and `TouchGate::view()`: `FrontRule rule = AskedAfterPush` and
`const FinishFacts& finish = FinishFacts{}`. Every existing call is therefore today's screen.

**Under `ByPush`:** the headers 1/4..4/4, `PUSH FRONT FIRST`, the rule line "the end that leads the push is FRONT"
during the IMU wait, and `touch DONE -- then stand clear`. The WHICH END view never appears.

**On `Held`, the held views (§1.4):** FRONT FLIPPED or MECHANISM PORT TURNED.
- Their left box is today's `PanelButtonId::Front`, relabelled USE NEW or SAVE. A NEW id would be a `-Wswitch`
  failure in today's program (:704-713) and in `test/calibrate_panel_test.cpp` (:119-126). `TouchGate::touch`
  reads it as `SaveHeld` on a held screen only.
- The right box is today's `PanelButtonId::RunAgain` at `kBackBox`. `TouchGate` already turns it into `RunAgain`
  in `Phase::Done`, and the program's RUN AGAIN already discards the held run.
- The amber SAVED header `SAVED -- NEW FRONT` follows a USE NEW.
- **(D6 (a))** With `finish.typedPorts`, the FRONT FLIPPED view swaps its three reference lines for the typed
  ports' (`kFlipWhatTyped`, `kFlipNewTyped`, `kFlipWarnTyped`); the header, `NOTHING SAVED YET`, `kFlipAgain` and
  the two boxes are the same.

**The NOTE row** (§1.4). Seventeen new literals join `kAll`'s compile-time budget check (`amend/p3r/`), among them
`NOTHING SAVED YET`, the FRONT FLIPPED and MECHANISM PORT TURNED lines, `SAVE`, and the NOTE `FIRST RECORD -- the end
you pushed first is FRONT`. **D6 (a) adds five more** (`d6a/p3r/`, measured against `screenText` in P26):
`kFlipWhatTyped` `typed ports: the end you led is BACK` (MEDIUM, 36 of 36), `kFlipNewTyped` `USE NEW: you led with
the FRONT -- fix typed ports` (SMALL, 50 of 50), `kFlipWarnTyped` `USE NEW reverses autons tuned on the typed ports`
(48 of 50), and the NOTEs `kNoteTypedSame` `FRONT MATCHES YOUR TYPED DRIVE PORTS` (36 of 50) and `kNoteFixTyped` `FIX
YOUR TYPED DRIVE PORTS -- see the log` (41 of 50). **The check of D6 (a) adds a sixth** (`d6a/v3/p3r/`, C4):
`kNoteTypedPartial` `FRONT MATCHES, SOME TYPED PORTS DIFFER -- see log` (SMALL, 49 of 50), the NOTE when the front
matched the typed ports but some port did not; `kNoteTypedSame` is now painted only on an exact match.

Today's program on this panel is 33 of 33 files byte-identical (P16; re-run with D6 (a)'s panel, P28: 33 of 33; with
the check's, P37: 33 of 33).
**`drivetrain_calibrator.hpp` is not touched in A1 or A2.**

**Changed library files in Part B — C1, the removal** (`proposed2/` is that end state):
- `calibration/drivetrain_calibrator.hpp` — `Answer::FrontFirst`/`BackFirst`, `Step::PushAwaitingFront`,
  `CalibrationReport::backFirst` and `applyFront(bool)`'s inversion removed; `finishPushCapture()` goes straight to
  the pulses; the push's prompt words; `forward` in the spin's sign is always +1 (`proposed2/` diff: 36 lines out,
  8 in; 1,978 → 1,950). `Answer` loses its two middle enumerators (`Done, FrontFirst, BackFirst, Abort` →
  `Done, Abort`, :202-207): it is never persisted (the record stores no answer), so renumbering `Abort` is safe —
  the header says so; nothing on the card changes.
- `calibration/calibrate_panel.hpp` — the `WhichEnd` screen, its words and `kHeaderWhichEnd` go; `FrontRule` and
  the `AskedAfterPush` branch go (`ByPush` is the only behaviour; the `1/5..5/5` headers and `PUSH IT FORWARD`
  leave with it); `PanelButtonId::Front` is renamed `SaveHeld` and `Back` is REMOVED (Re-check R2: a held screen's
  right box is `RunAgain`); `kFrontBox`/`kBackBox` become `kChoiceLeftBox`/`kChoiceRightBox` (:153-156 — still in
  the MIDDLE, never in the button row). Probed as `amend/p2r/` (P18): goldens 69 of 69 unchanged.
- `calibration/calibrate_program.hpp` — the auto-answer and the `FrontRule::ByPush` arguments go.

### 1.3 What `run()` changes from today's program — the complete list (every other line is moved, not changed)
1. **The words that name robot two** become the facts' (§0.5 G): the banner's "the robot", `robot : <facts.robot
   or (not stated)>`, the IMU line's "(stated N)", the census header, the refusals' fixes ("state it in the robot's
   facts"), the refusal footer ("restart it", "the census is on the serial log"); the 2026-09-24 history and the
   X-drive sentence leave the banner.
2. **The census's declared-port drop reads the facts** (`declaredMechanism(facts, port)`), names each by its facts
   name — `port 6: claw pivot 6 (declared) -- not a candidate, not touched` — and adds `port N: <name> (declared) --
   the census found NO MOTOR there` for a declared port the registry did not report.
3. **The stated facts' refusals** (`factsProblem` naming the entry, kind unset, not Tank naming the stated kind,
   cartridge unset) — before any motor is made, each with its own LARGE headline; TOO FEW MOTORS names the declared
   ports when any was dropped (Re-check R13, R18).
4. **Motors through the provider** (`devices.motor(port, cartridge)`), POSITIVE ports, a refusal naming the port
   from `lastRefusal()`; **the IMU through the provider** (its calibration started once, there).
5. **The front (ruling (1), D2, C1)**: the push words; no WHICH END (in A2 the program answers the
   calibrator's `FrontFirst` itself the tick DONE is accepted, before any view is painted — Part B deletes the
   question and that line); at DONE the `front:` line says the leading end is the front; before the save, §1.4's
   front check — against the card's record, else (D6 (a)) the typed drive ports; FRONT FLIPPED holds the save
   (`SaveOutcome::Held`) until USE NEW (save as pushed) or RUN AGAIN
   (discard); a field disable while held writes nothing and says so (Re-check R2, R10). Nothing is ever turned to
   another front.
6. **The save (B1, §1.5)** replaces `saveAndReadBack()`.
7. **The watch (D4, §1.4)**: armed when the PUSH phase begins, judged at DONE (after the push table), one line per
   declared port, a WARNING (with the drive's travel beside it) for a port that moved past `movedThreshold`, and
   then the MECHANISM PORT TURNED hold (Re-check R12, R15): SAVE or RUN AGAIN; after a USE NEW it follows the FRONT
   FLIPPED hold, one question per screen.
8. **The serial `card :` line** at start (what the card holds, one line, from `loadRecord`) — the operator knows
   before pushing whether a front check will run. With no usable record on a present card and typed ports in the
   facts (D6 (a)): `card       : no usable record yet -- this run's FRONT is checked against your typed drive
   ports`; with NO card (the check of D6 (a), C3): `card       : NO SD CARD in the brain -- nothing can be saved,
   and the FRONT is not checked` (the revision said `no usable record yet` there — no card is not "no record",
   landmine 5); otherwise the revision's words.
9. **The loop's competition read** is `io.competitionStatus()` against the three bits; its touch is
   `screen.touch()`; its waits `io.delayMs()`; its prints count `screen.prints()`.
10. **The undeclared-motor words (Re-check R11):** with no mechanism declared, three banner lines say every motor is
    a drive candidate and one the push turns is POWERED (3, 6, 9 V), "declare every mechanism motor, or unplug it
    first"; at DONE one line per candidate that did not turn (`port N did not turn with the push: not a drive
    motor, never powered -- if it is a mechanism, declare it in the robot's facts`); the LEFT OUT line adds "or a
    MECHANISM motor -- declare it in the robot's facts"; a ROLLED or STILL refusal on a port adds `if port N is a
    MECHANISM motor, declare it …`.

### 1.4 The front check and the watch — the rules, the words, the screen
**The front check** (`finishDone`, after the MEASURED lines, before any write). **THE REFERENCE (D6 (a), decided
2026-10-08): the card's record as `loadRecord` chooses it; with no usable record on the card, the TYPED drive ports
when the facts carry them (`hasTypedDrive`); else none.** In order:
- read the card's record as the boot would (`loadRecord(card)`: the record file, else the backup);
- **no card** (`card.cardPresent()` false) → `front check: NOT DONE -- no SD card in the brain (nothing can be
  saved either)`; save (it will say NOT SAVED — no card). **Typed ports or not**: nothing can be saved, so there is
  no front to protect (measured unchanged: `d6a/run_new/nocard` byte-identical to `amend/`'s with robot two's typed
  ports in the facts; in the check's runs, `d6a/v3/run_new/typed-nocard` equals `nocard` but its name line). *(The
  check of D6 (a), C3:)* the `card :` line at start already says `NO SD CARD in the brain -- nothing can be saved,
  and the FRONT is not checked`, and a held run discarded with no card says `there is no SD card in the brain`
  (below);
- **a record** (either file decodes) → `compareFronts(record, measured)` and one line with every count and the
  verdict (§0.6 P7's shape), naming the file it compared against (`shulib_drivetrain.bin` or `.bak`). **The typed
  ports change nothing here** — not a line, not a draw, not a byte (measured: `d6a/run_new/typed-card-wins`, a card
  holding the front-led record and the facts' typed ports FLIPPED, is byte-identical to its untyped control: serial,
  screen and card — P23). Then the verdicts below;
- **(D6 (a)) no usable record — no file, or files that exist but neither decodes — and typed ports in the facts** →
  the typed ports are the reference (`typedDriveDescription(facts)`):
  - `front check: the card holds no record yet -- checking against your typed drive ports instead` (no file), or
    `front check: the card's record does not decode -- checking against your typed drive ports instead` (files that
    do not decode — **decided: the typed ports, because a record that does not decode is no record to the boot
    either: `chooseBootSource` treats "not decoded" as "absent" and drives from the typed table when it is valid,
    `boot_check.hpp` :439-442**), followed by **one line per file that OPENED and did not decode, naming it** (the
    check of D6 (a), C3): `  shulib_drivetrain.bin: magic is not "SHDC" -- this is not a calibration record`,
    `  shulib_drivetrain.bak: …`, or `  shulib_drivetrain.bin: EMPTY (0 bytes read)`. *(The D6 (a) draft printed
    the record file's reason in parentheses: empty when only the backup opened, or when a file was EMPTY, and
    liable to be cut by the 240-byte line. Measured: `v3/run_new/typed-undecodable`, `bakonly-garbage-typed`.)*;
  - `typed drive ports: LEFT -11 +12 -13 +14 -15 | RIGHT +20 -19 +18 -17 +16` (the list as compared);
  - `front check against your typed drive ports: N drive port(s) in both -- A agree, F flipped, S sign only, D side
    only; T in the typed ports only, R in this run's only: <VERDICT>`;
  - then the verdicts below, in the typed ports' words. **SAME, exactly** — every port in either list agrees, none
    sign-only, side-only or flipped, none in one list only — adds the NOTE `FRONT MATCHES YOUR TYPED DRIVE PORTS`
    (measured: `d6a/v3/run_new/typed-first`, `typed-undecodable`). **SAME with any difference** (the check of D6 (a),
    C4) adds `front check: SAME FRONT, but your typed drive ports and this push differ at N port(s) -- check each
    before anything drives from the typed ports:` and one line per differing port — `  port 12: typed LEFT -12,
    this push LEFT +12 -- the SIGN differs`, `… -- the SIDE differs`, `… -- the side AND the sign differ`,
    `  port 9: typed RIGHT +9, not in this push's record (LEFT OUT, declared, or did not turn)`, `  port 16: this push
    RIGHT +16, not in your typed drive ports` — and the NOTE `FRONT MATCHES, SOME TYPED PORTS DIFFER -- see log`
    (measured: `typed-partial`). A typed port with a wrong sign fights its side-mates in anything that still drives
    from the typed ports, so "matches" alone would hide it (the checker's probe: one sign wrong of ten painted
    FRONT MATCHES under the D6 (a) draft);
- **no usable record and no typed ports** — today's two cases, unchanged (measured: `d6a/run_new/first-untyped`
  byte-identical to `amend/run_new/one`; since the check of D6 (a), robot two's own `one` is this case again,
  `v3/run_new/one` = `amend/run_new/one`):
  - **a card with no record** (neither file opened) → `front check: the card holds no record yet -- this is the
    FIRST: the end that led the push is the FRONT`; save; the NOTE says `FIRST RECORD -- the end you pushed first is
    FRONT` (Re-check R16; drawn in `amend/run_new/one.screen.log`);
  - **a card whose files do not decode** (opened, refused) → `front check: NOT DONE -- the card's record does not
    decode; the end that led the push is the FRONT -- check the robot drives the right way before an auton`, then
    the per-file lines above (the check of D6 (a), C3: the revision put the record file's reason in the line, empty
    for a backup-only card or an EMPTY file); save; the finished screen's NOTE says `FRONT NOT CHECKED -- see the log`
    (measured: `v3/run_new/undecodable-untyped`, `bakonly-garbage-untyped`, `emptyfile-untyped`);
- **the verdicts** (against the card's record, or the typed ports):
  - SAME FRONT → save;
  - CANNOT TELL → `front check: CANNOT TELL (no drive port in common, or no majority) -- saved as measured; check the
    robot drives the right way before an auton`; save; the NOTE says `FRONT NOT CHECKED -- see the log`;
  - FRONT FLIPPED → five lines (Re-check R2/R10, as `amend/run_new/swap.serial.log` prints them): `FRONT FLIPPED --
    the card's record calls the end that LED this push its BACK.` / `Either the BACK led by mistake (touch RUN
    AGAIN: nothing is saved), or the FRONT led` / `and the drive cables were swapped between the sides, or the front
    was changed on` / `purpose (touch USE NEW: saved as pushed). The records cannot tell these apart.` / `USE NEW
    after a wrong push reverses every auton. NOTHING IS SAVED YET.`; the loop stats; `SaveOutcome::Held`.
    USE NEW → `front: USE NEW -- the end pushed first THIS time is the FRONT (saved as pushed)` and the save (or,
    if a declared port also turned, the MECHANISM PORT TURNED hold next). RUN AGAIN → `RUN AGAIN on a held screen
    -- NOTHING SAVED: this run is discarded, the card keeps its record` and a new run. A field disable →
    `the field disabled the robot on the held FRONT FLIPPED screen -- NOTHING SAVED: the card keeps its record`.
  - **FRONT FLIPPED against the TYPED ports (D6 (a))** → the same hold, USE NEW and RUN AGAIN only (no KEEP OLD —
    ruling (1), D2 (b)), five lines in the typed ports' words, whose causes include the typed ports being wrong
    (as `d6a/run_new/typed-flip-new.serial.log` prints them): `FRONT FLIPPED -- your typed drive ports call the
    end that LED this push its BACK.` / `Either the BACK led by mistake (touch RUN AGAIN: nothing is saved), or the
    FRONT led` / `and the typed ports are wrong, or the drive cables were swapped since they were typed` / `(touch
    USE NEW: saved as pushed -- then fix the typed ports). The push cannot tell` / `these apart. USE NEW reverses
    every auton tuned on the typed ports. NOTHING IS SAVED YET.` USE NEW → the same `front: USE NEW …` line, then
    `front: now FIX YOUR TYPED DRIVE PORTS -- they call this FRONT the BACK, and anything that still drives from
    them drives this robot backwards`, the NOTE `FIX YOUR TYPED DRIVE PORTS -- see the log`, and the save (or the
    MECHANISM PORT TURNED hold next). **RUN AGAIN and a field disable say what the card holds, which here is no
    usable record:** `RUN AGAIN on a held screen -- NOTHING SAVED: this run is discarded, the card is unchanged (it
    holds no usable record)` / `the field disabled the robot on the held FRONT FLIPPED screen -- NOTHING SAVED: the
    card is unchanged (it holds no usable record)` (measured: `typed-flip-again`, `typed-flip-disable`). The same
    words follow a MECHANISM PORT TURNED hold on a card with no usable record (`typed-watch-typo`): "the card
    keeps its record" is printed only when the card held one. *(The condition is the card's contents, not the
    reference, so it also corrects one path the revision worded wrongly and no golden ran: a MECHANISM PORT TURNED
    hold on an empty card with NO typed ports, discarded by RUN AGAIN or a disable, printed "the card keeps its
    record" (`amend/p3r/…/calibrate_program.hpp` :509-512, :569-570, unconditional). It now says "the card is
    unchanged (it holds no usable record)" — measured, `d6a/run_extra/watch-typo-empty-untyped`. No other path's
    words change: P23.)* **With NO card** (the check of D6 (a), C3) the discard words are `NOTHING SAVED: this run is
    discarded, there is no SD card in the brain` / `… NOTHING SAVED: there is no SD card in the brain` (measured:
    `v3/run_new/nocard-watch-again`; the D6 (a) draft said "the card is unchanged" there). The empty-card,
    no-typed-ports path is a golden now (`empty-watch-untyped`, the renamed scenario, C2).
- **The verdict rule** (`compareFronts`), over the drive ports BOTH records list, matched by |port|: same side and
  sign = agree; other side AND other sign = flipped; same side, other sign = re-mounted (`signOnly`); other side,
  same sign = moved across (`sideOnly`); a port in one record only decides nothing and is counted. FLIPPED iff
  `2 × flipped > shared`; SAME iff `2 × agree > shared`; otherwise UNDETERMINED. **Probed** (P7):

  | Pair (earlier = the card's) | shared | agree | flipped | other | → |
  |---|---|---|---|---|---|
  | the same record | 8 | 8 | 0 | — | SAME |
  | the other end led | 8 | 0 | 8 | — | FLIPPED |
  | robot two's 09-25 record vs its typed table | 8 | 0 | 8 | 2 table-only | FLIPPED |
  | a port re-cabled (13 → 9) | 7 | 7 | 0 | 1 + 1 one-only | SAME |
  | a side short a motor (15 LEFT OUT) | 7 | 7 | 0 | 1 one-only | SAME |
  | flipped AND a side short | 7 | 0 | 7 | 1 one-only | FLIPPED |
  | one motor re-mounted (12's sign) | 8 | 7 | 0 | 1 signOnly | SAME |
  | one cable across sides (13) | 8 | 7 | 0 | 1 sideOnly | SAME |
  | another robot's card | 0 | — | — | 8 + 4 one-only | CANNOT TELL |
  | a tie | 8 | 4 | 4 | 1 this run's only | CANNOT TELL |
  | flipped, one moved across, one re-cabled | 7 | 0 | 6 | 1 sideOnly, 1 + 1 one-only | FLIPPED |
  | 3 of 8 flipped | 8 | 5 | 3 | — | SAME |
  | one motor a side, flipped | 2 | 0 | 2 | — | FLIPPED |
  | **the LEFT and RIGHT drive cables swapped, the right end leading** (Re-check R10) | 8 | 0 | 8 | — | FLIPPED — **the records cannot tell this from a flip** (`check-safety/swap_probe.log`; through the program, `amend/run_new/swap`: 10 of 10 flipped) |
  | **(D6 (a), earlier = the TYPED ports)** robot two's typed table vs the plant's front-led push (`d6a/run_new/one`; since the check, `v3/run_new/typed-first`, its table made valid) | 10 | 10 | 0 | — | SAME (NOTE `FRONT MATCHES YOUR TYPED DRIVE PORTS`) |
  | robot two's typed table vs the other end leading (`typed-flip-new`) | 10 | 0 | 10 | — | FLIPPED → held |
  | typed ports half flipped: LEFT −11 +12 −13 −16 +17 −20, RIGHT −14 +15 +18 −19 (`typed-tie`) | 10 | 5 | 5 | — | CANNOT TELL → saved, NOTE `FRONT NOT CHECKED` |
  | typed ports with no port in common: LEFT −3 +4, RIGHT +5 −8 (`typed-nocommon`) | 0 | — | — | 4 typed-only, 10 this run's only | CANNOT TELL → saved, NOTE `FRONT NOT CHECKED` |
  | robot two's typed table vs a run with port 12 declared (`typed-watch-typo`) | 9 | 9 | 0 | 1 typed-only | SAME, then the MECHANISM PORT TURNED hold (since the check: the serial log names port 12 as typed, not in this push) |
  | **(the check of D6 (a), C4)** typed LEFT −11 −12 −13 +14 −15, RIGHT +20 −19 +18 −17 +9 (`typed-partial`) | 9 | 8 | 0 | 1 signOnly; 1 typed-only, 1 this run's only | SAME, the three port lines, NOTE `FRONT MATCHES, SOME TYPED PORTS DIFFER -- see log` |
- **No `flipFront`** (Re-check R2): the draft's KEEP OLD turned a measurement to the card's front; it is withdrawn,
  and nothing is ever saved but the measurement as pushed.
- **The FRONT FLIPPED screen** (amber header `FRONT FLIPPED`; `amend/p3r/` `calibrate_panel.hpp`, drawn in
  `amend/run_new/swap.screen.log`): LARGE amber `NOTHING SAVED YET`; MEDIUM amber `the card calls the end you led
  BACK`; SMALL `RUN AGAIN: you led with the BACK by mistake`, `USE NEW: you led with the FRONT (cables swapped?)`,
  and in amber `USE NEW after a wrong push reverses every auton`; two big boxes USE NEW (left) and RUN AGAIN
  (right), greyed for the arming delay like every new button. No question about which end is the front (ruling
  (1)); no ABORT (nothing is powered); a field disable or a power-off ends the program with nothing written.
  **Against the TYPED ports (D6 (a); `FinishFacts::typedPorts`; drawn in `d6a/run_new/typed-flip-new.screen.log`):**
  MEDIUM amber `typed ports: the end you led is BACK`; SMALL `RUN AGAIN: you led with the BACK by mistake`, `USE
  NEW: you led with the FRONT -- fix typed ports`, amber `USE NEW reverses autons tuned on the typed ports`; the
  same header, LARGE line and boxes. After USE NEW the SAVED screen is `SAVED -- NEW FRONT` with the NOTE `FIX YOUR
  TYPED DRIVE PORTS -- see the log`.
- **The MECHANISM PORT TURNED screen** (Re-check R12; amber header; drawn in `amend/run_new/watch-typo-save.screen.log`):
  LARGE amber `NOTHING SAVED YET`; MEDIUM amber `port 12: 1692 deg, drive 1692 deg` (the port's turn beside the
  drive's median push travel); SMALL `the card's record DRIVES with port 12` when it does — **or, when the typed
  ports were the reference (D6 (a)), `your typed ports DRIVE with port 12` when they do** (serial: `Your typed drive
  ports DRIVE with port 12: saving now makes a record with NO port 12.`; measured, `typed-watch-typo`) — else `it is
  declared a mechanism: not in this record`; `SAVE: it is a mechanism the push turns -- keep it`; amber `RUN AGAIN = save
  nothing: a drive motor? fix facts`; boxes SAVE (left) and RUN AGAIN (right). SAVE → `watch: SAVE -- port 12 stays
  a declared mechanism; this record has no port 12` and the save; RUN AGAIN and a disable as on FRONT FLIPPED.
- **The finished screen's NOTE** — ONE amber SMALL line, when there is something to check, the first that applies:
  `PORT N KEPT AS A MECHANISM -- see the log` (after a SAVE on MECHANISM PORT TURNED); `FIX YOUR TYPED DRIVE PORTS
  -- see the log` (after USE NEW against the typed ports, D6 (a)); `FRONT NOT CHECKED -- see the log` (does not
  decode with no typed ports; CANNOT TELL against either reference); `FRONT MATCHES, SOME TYPED PORTS DIFFER -- see
  log` (SAME against the typed ports with any port differing — the check of D6 (a), C4); `FRONT MATCHES YOUR TYPED
  DRIVE PORTS` (SAME against the typed ports, every port agreeing, D6 (a)); `FIRST RECORD -- the end you pushed first
  is FRONT` (**only with neither a record nor typed ports**; Re-check R16). It takes
  the third SMALL row; a calibrator warning that needs two rows is then cut to one with ` ...` (`wrapWords`' rule),
  and the full text stays on serial. Every fixed word fits its budget at compile time; the runtime lines are cut
  by the painter.
**The watch** (D4): when the calibrator enters the PUSH phase, read every declared port (`io.readShaftDegrees`); at
DONE, after the push table, read each again: a port that moved more than `movedThreshold` (5°, the push's own
number, HA-136 — no new constant) prints `WARNING: port N is declared as <name> but TURNED WITH THE PUSH (+X deg;
the drive turned Y deg) -- the save waits for you` (Re-check R15: never "check the facts") and HOLDS the save on
MECHANISM PORT TURNED (R12); one that did not, `watch: port N (<name>) did not turn with the push (+X deg)`; one
that could not be read either time, `watch: port N (<name>) could not be read -- not judged`. With no declared
port, three banner lines say there is nothing to watch AND that every motor is a drive candidate (R11).

### 1.5 The save — SAFE-SAVE Part B1, absorbed (its brief §1 "Part B1", word for word where it applies)
- `save(d)` = `saveRecordSafely(card, d)` (`record_store.hpp` :587-606) over the `IFileStore` the program was given
  (`ProsFileStore` on the brain); the serial outcome is `formatSaveReport(report, BootReadsBackup::No, emit)`
  (:744-916) — B2 flips the argument when the boot reads the backup; the panel is told `SaveOutcome::Saved` iff
  `report.outcome == SaveResult::Saved` (NotSaved and Unconfirmed both show NOT SAVED, SAFE-SAVE D5); then the two
  follow-ups today prints (`COPY THE 'MEASURED tank ...' LINE ABOVE DOWN …` after SAVED; `THE RESULT IS ON THE
  SCREEN AND IN THIS LOG ONLY. Copy it down NOW.` otherwise — without today's false parenthesis).
- `saveAndReadBack()` is GONE; nothing in the program opens a `ProsBlockSink` or `ProsBlockSource` for the record.
- The banner, `calibrateBootBanner` and the `record file:` line name BOTH files (`kRecordFileName`,
  `kRecordBackupFileName`) and say the order (a verified backup first) — B1 item 1.
- `finishStopped()` keeps "Nothing was written to the card by this run." (still true).
- **Probed** (P8): the eight paths of §2.A's save table give B1's words and the library's `loadRecord` reads
  what they claim. **(Re-check R17)** The drive program's boot reads only the record file until SAFE-SAVE B2. On
  the SAVE UNCONFIRMED path robot two's drive therefore refuses to boot until Calibrate saves again, because its
  typed table is not valid. Station G says so.

### 1.6 The clean place for R5a Part C (GAINS and REST) — left, not built
- `CalibrateProgram` keeps one private `enum class RunKind : std::uint8_t { Drivetrain = 1 }` that `run()` switches
  on where a run is started and where its outcome is printed; R5a Part C appends `Gains`, `Rest` (append-only).
- The finished screen's buttons are built as a LIST in `viewFor()` (RUN AGAIN today); Part C appends GAINS and REST
  after SAVED (its D7).
- A start screen is NOT added now (Part C adds CALIBRATE / GAINS / REST when the card's record decodes and passes
  `verifyAgainstRegistry` with the declared ports — which `CalibrateFacts.mechanisms` already supplies).
- The motor construction is one private function from a port list and the cartridge, POSITIVE ports — Part C calls
  it with the record's ports (its C.1).
- The record's stored CRC (Part C's `LoadedRecord::storedCrc`) is not CAL1's; CAL1 does not edit `record_store.hpp`.

### 1.7 What a NEW robot's Calibrate needs from G3a Part 2 — nothing Calibrate-specific
With CAL1, `src/programs/calibrate_program.cpp` and `src/robots/calibrate_facts.hpp` read the BUILD's `kChassis`
and `kMechanisms`; G3a Part 2 already generates a new robot's tables, its chain branch, its Makefile row and its gate
row. So a generated robot's Calibrate is those generated lines and no others: no front words, no front
`static_assert`, no banner hunk (G3a §1.13's three Calibrate hunks are superseded — §5 tells the coordinator).
G3a Part 2 still owns the general form of `build_selection.hpp`'s "PROGRAM=calibrate needs ROBOT=tank" guard
(:89-90) and the Makefile's; CAL1 only rewords both messages (the cartridge now comes from "this robot's chassis
table", not "robot two's"). **Two conditions on that general form (Re-check, §5):** it must keep Calibrate building
only for a robot whose generated kind is Tank, or carry the generated kind into the facts so `NOT A TANK` refuses
(R13); and a generated robot's MECHANISM ports must reach its mechanism table before its first Calibrate — or its
bring-up says to unplug every mechanism motor first — because an undeclared motor is a drive candidate that the
push can get pulsed (R11; until G3a Part 5 a generated robot carries `kNoMechanisms`). **(D6 (a))** A generated
robot's typed drive ports reach Calibrate with no line written for them: the glue's `withTypedDriveFrom` takes them
when its generated table is a VALID description — `tableToDescription(t).valid()`, exactly the test the boot uses
before it drives from a table (`chooseBootSource`). A generated table with signed ports but no geometry is not
valid, and gives none, as robot two's does (the check of D6 (a), C1: the D6 (a) draft took any SIGNED table, and so
"exactly as the boot" was false). A robot made in the panel types no drive ports (MECH-PANEL §4.1), so its first
Calibrate gets the FIRST RECORD note.

### 1.8 Robot two onto the standard Calibrate (Part B) — every `src/` file it touches
- `src/programs/calibrate_program.cpp` → the 27-line glue (§1.1; 24 before the check of D6 (a)); `calibrate_program.hpp` → its five declarations.
- `src/robots/calibrate_facts.hpp` (NEW, §1.1; ~77 lines with `withTypedDriveFrom`, the check of D6 (a)'s form of the
  typed-port mapping; the glue is 27 lines).
- `src/main.cpp` — Calibrate's three state-screen cases (:178-183, :201-207, :242-246) call
  `calibrateDisabled()`, `calibrateConnected()`, `calibrateAutonomous()`; Bench Tests and Drive keep
  `benchStateScreen`. (The library's competition-state screen says what `benchStateScreen` says, in the
  library's painter: P1's `disable` scenario shows both.)
- `src/common/screen_text.hpp` — `printText()`'s body becomes `hal::screenSafeLine(out, sizeof out, text,
  calibration::budgetFor(size), padTo)` + the same three PROS calls and the counter (C3; P10 built all 7 builds on
  it); its header names the library's sanitizer as the one rule.
- `src/build_selection.hpp` :89-90 and `Makefile` :176-178 — the refusal MESSAGES only: "takes the cartridge
  from this robot's chassis table" (G3a Part 2 owns the guards' general form).
- `tools/src_build_gate.py` :477 — `PROGRAM_ENTRY_SYMBOLS["Calibrate"]` gains the three entry names (P4 measured
  the failure without them). **(Re-check R1)** Self-test case 26's Calibrate plant moves with the code it
  plants on: `CALIBRATE_PLANT` (:1503) becomes `src/robots/calibrate_facts.hpp`, and its anchor (:2207-2208)
  becomes `calibrateFactsFor(kChassis, kMechanisms)` → `calibrateFactsFor(kChassis,
  mechanismTableFor(RobotId::BenchBot))`. Without this edit `self-test` dies on `_replace`'s
  exactly-once assertion. The docstring's `tank-calibrate` paragraph (:39-40) and :174's example are
  reworded (R1). Then `self-test` is re-run: it builds against real builds, ~11 min. Probed: `OK (211 detector
  cases)`, `amend/gate_selftest.log`.
- `src/robots/mechanism_table.hpp` — `DroppedMechanismPorts` and `dropDeclaredMechanismPorts()` (:450-489) are
  DELETED: their only caller was `calibrate_program.cpp` :1129-1130 (grep, §0.1); the library's facts-based drop
  replaces them, and their test cases move to Part A's drop tests (§2.A). `motorEntryCount`, `absPort` and the rest
  stay (the drive, competition and boot-source code use them).
- **Nothing else in `src/`.** Bench Tests' MOTOR WATCH keeps its own FRONT/BACK question (`bench_r3a.cpp`
  :1200-1210, :1462-1464) — ruling (3) leaves Bench Tests to PKG1, and its answer is in memory only, never a record (§4).

## 2. The parts — scope, tests (adversarial), mutations, DoD

### 2.A Parts A1 and A2 — the standard Calibrate, as library ADDITIONS (no `src/` file; every existing test unchanged; ~1¼–1⅜ and ~2½–2⅞ builder-days, re-derived from their items by the check of D6 (a))
**(Re-check: the draft's single Part A, ~2–2½ days, grew to ~2½–3 with the re-check's additions and is split —
D5 (d). Every test and mutation below says which part builds it; the rig and the goldens are A2's.)**

**A1 builds** §1.2 items 1-5 and 8: the screen seam and `screenSafeLine`, the fake screen, `ProsBrainScreen`, the
painter and the program's screens (`calibrate_screen.hpp`), `front_check.hpp`, and the shim pieces (`screen.h`,
`misc.h`, the C-API `motors.h`, `rtos.hpp`'s hook, the host `main.h`). **Size, where it goes:** the seam, the fake,
the adapter and the sanitizer (~¼); the painter and the screens moved from today's program (~¼); the front check
(~⅛); the shim pieces (~⅛); tests 3, 7, 8a and A1's half of 9, their mutations (~¼); the doc lines for A1's
public entities (P11's 29 screen literals are here), the changelog, `docs/api/`, the gates (~¼); **(D6 (a))**
`front_check.hpp`'s header sentence and test 3's two typed-table pairs (~1/16). **~1¼–1⅜** — the items sum to
1 5/16 (the check of D6 (a), C5: the earlier "~1–1¼", and its "inside the range", were below the items' own sum).

**A2 builds** §1.2 items 6-7 and the panel's ADDITIVE change (the calibrator untouched); §1.3–§1.6 inside
`CalibrateProgram`, with the auto-answer of §1.3 item 5. **Size, where it goes:** the move of ~1,000 program lines
into the seams (~½ — the re-check's `amend/p3r/` is a working draft, not project code: every line is re-derived
and re-read against the landed file); the panel's additions and the two held screens (~¼); the C-API watch, its
hold, the undeclared-motor words and the facts checks (~¼); the host rig and its scenarios, the re-check's nine
included (~½); the golden test and the team file's example test (~¼); mutations, HA-187, the chapter, guide 12, the
changelog, the gates (~¼–½); **(D6 (a), +~⅜, re-estimated from `d6a/p3r/`)** the typed facts, their checks, the
reference rule and its words in the program and the panel (~⅛ — the probe's draft is +162 program lines and +18
panel lines); test 2's typed cases, test 6's typed refusals, test 11's typed line and eleven more golden scenarios
(~⅛); mutations 28–42 (~⅛); the chapter's paragraph, guide 12's three rows and the changelog's lines (inside the
docs line); **(the check of D6 (a), +~3/16, re-estimated from `d6a/v3/p3r/`)** the per-file reason lines, the
no-card words, the exact/partial typed NOTE and the per-port lines, the sixth literal (~1/16 — +66 program lines,
+3 panel lines); the check's eight golden scenarios and the test cases of tests 2, 5 and 6 (~1/16); mutations 43–54
(~1/16); guide 12's reworded and new rows (inside the docs line). **~2½–2⅞** — the items sum to 2–2¼ + ⅜ + 3/16 =
2 9/16–2 13/16 (the check of D6 (a), C5: the earlier "~1½–2" and "~1⅞–2⅜" were below the items' own sum). **Both
ends are past the ~2-day line that split Part A** — whether A2 splits is the team lead's (the check's "not
settled" #1, §9.2).

**Both start** when T4 Part 3 has LANDED (its reviews may still edit `calibrate_program.cpp`, the file A2 moves —
re-read it as landed, and move THAT text). A2 starts after A1 lands.

**The host rig** (`test/calibrate_program_rig.hpp`, NEW, A2): the A2 plant in robot two's shape (the probe harness's
`wire()`, `amend/harness2.cpp`), served through `hal::fake::FakeDeviceProvider` (`bindMotor`/`bindImu` onto the
plant, its pacer stepping the plant 10 ms), a `FakeDeviceRegistry`, a `FakeFileStore`, a `FakeBrainScreen`, and a
`FakeCalibrateIo` whose `competitionStatus()`, `readShaftDegrees()` and scripted OPERATOR run inside the pacer: the
operator pushes the robot and taps only what the FAKE SCREEN has drawn (the probe's rule: it reads the draw calls,
never the program), so a button the program never shows can never be pressed. The rig can also: swap the LEFT and
RIGHT cable bundles; add undeclared motors (still, or turned by the push and free-spinning when powered); seed the
card — with a record, or **(D6 (a)) with two files that do not decode** (the probe's 180 bytes of `x`); scale one
side's reported shaft position (unequal side scales, R14); and override the facts, **typed drive ports included**.

**Tests** — each names the property it would fail without, and the part that builds it:
1. **(A2) The scenarios** (one test each, robot two's facts — **as Part B derives them, with NO typed drive ports:
   its table is not a valid description, so `withTypedDriveFrom` gives none (the check of D6 (a), C1); on an empty
   card the `card :` and `front check` lines and the NOTE are the revision's FIRST RECORD words** — *(the D6 (a)
   draft gave robot two its table's typed ports here, withdrawn)*): one clean run; two runs (RUN AGAIN); port 18 slipping
   (LEFT OUT); a curved first push (STOPPED) then a straight one; no card; no IMU (refused, nothing made); the field
   disabling mid-pulses (every motor 0 V + Coast on the same tick; nothing written); **one with UNEQUAL side scales
   (R14)**. Each asserts VERBATIM the serial lines that carry a decision: the `card :`, census, drop, `front`,
   `front check`, `watch`, MEASURED, SAVED / NOT SAVED lines. Each also asserts the saved record's bytes, and that
   the card's decoded description equals the printed MEASURED line.
2. **(A2) The front holds** (Re-check R2, R10):
   - a run pushed with the other end leading, against a card holding the first run's record, is HELD: the FRONT
     FLIPPED view and ZERO replace operations in `FakeFileStore`'s log;
   - USE NEW → the record file equals `encodeRecord(measured)`, saved AS PUSHED;
   - RUN AGAIN → nothing written, and a new run whose front-led push gives SAME FRONT and saves;
   - a field disable while HELD → the card's files byte-identical to before, and the `… on the held FRONT FLIPPED
     screen -- NOTHING SAVED` line;
   - **the cable-swap scenario**: seeded card, bundles swapped, the right end leading → FRONT FLIPPED → USE NEW →
     a FORWARD command on the saved record drives every motor forward on its own side (the probe's score: 10 of
     10);
   - the first record on an empty card with its NOTE (facts WITHOUT typed ports); the no-card words; the
     do-not-decode words with the NOTE (no typed ports); CANNOT TELL (a card holding another robot's record) with the
     NOTE;
   - **(the check of D6 (a), C3) the cards whose reason the revision lost**, no typed ports: only the `.bak`, and it
     does not decode → NOT DONE and the line `  shulib_drivetrain.bak: <reason>`, NOTE `FRONT NOT CHECKED`; the
     record file EMPTY → `  shulib_drivetrain.bin: EMPTY (0 bytes read)`; both files undecodable → one line each;
     and **no card** → the `card :` line `NO SD CARD in the brain -- nothing can be saved, and the FRONT is not
     checked`;
   - a damaged record file with a good backup → checked against `.bak`, and the line names it;
   - **(D6 (a)) the typed ports as the reference**, each asserting the `card :`, the two `front check` lines, the
     `typed drive ports:` list, the verdict's words, the view's lines and the NOTE verbatim, and the card's files —
     the typed ports given through `withTypedDriveFrom` over robot two's table made VALID (track width 12.5 in,
     ratio 1.0: P24's numbers, the probe's case 8), unless a case says otherwise:
     - **agreeing** (robot two's typed ports, the front-led push, empty card) → SAME 10 of 10, saved, NOTE `FRONT
       MATCHES YOUR TYPED DRIVE PORTS`;
     - **(the check of D6 (a), C4) agreeing in front but not in every port** (LEFT −11 −12 −13 +14 −15, RIGHT +20 −19
       +18 −17 +9) → SAME (8 agree, 1 sign only, 1 in each list only), the `SAME FRONT, but … differ at 3 port(s)`
       line and one line per port (12 the SIGN, 9 not in this push's record, 16 not typed), saved, NOTE `FRONT
       MATCHES, SOME TYPED PORTS DIFFER -- see log`;
     - **flipped** (the other end leading, empty card) → HELD on FRONT FLIPPED in the typed ports' words, ZERO replace
       operations; USE NEW → saved as pushed, the `FIX YOUR TYPED DRIVE PORTS` line and NOTE; RUN AGAIN → nothing
       written and `the card is unchanged (it holds no usable record)`, then a front-led run 2 → SAME, saved; a field
       disable while held → no file on the card and the same words;
     - **a half-flipped tie** (LEFT −11 +12 −13 −16 +17 −20, RIGHT −14 +15 +18 −19: 5 agree, 5 flipped) → CANNOT TELL,
       saved, NOTE `FRONT NOT CHECKED`;
     - **no port in common** (LEFT −3 +4, RIGHT +5 −8) → CANNOT TELL, saved, NOTE `FRONT NOT CHECKED`;
     - **a card record present AND the typed ports flipped against it** → the card decides (SAME, saved), and the
       whole run — serial, view sequence, card bytes — EQUALS the same run with no typed ports (the typed ports change
       nothing);
     - **card files present that do not decode**, typed ports agreeing → the `does not decode -- checking against
       your typed drive ports instead` line and one line per file naming it, SAME, saved over them; typed ports
       flipped → HELD; **only the `.bak`, undecodable** (the check of D6 (a), C3) → the same, with the `.bak`'s line
       only (`  shulib_drivetrain.bak: magic is not "SHDC" …`) — never an empty reason;
     - **no card**, typed ports present → today's `NOT DONE -- no SD card` words, no typed line, no hold, and the
       run equals the untyped no-card run;
     - **a declared drive port** (12) on an empty card with typed ports → the MECHANISM PORT TURNED view's `your typed
       ports DRIVE with port 12`, and RUN AGAIN → `the card is unchanged`.
3. **(A1) `compareFronts`:** §1.4's fourteen pairs, the cable swap included and marked "the records cannot tell
   this from a flip", each with its literal counts and verdict (written in the test, never computed by the code
   under test). **(D6 (a))** Two more, with `earlier` a description holding only robot two's typed sides: against
   the plant's front-led record, SAME (10 shared, 10 agree); against the other-end record, FLIPPED (10 of 10) — P24.
4. **(A2) The save (B1):** the program's save through `FakeFileStore` faults, at minimum P8's eight paths, KEEP
   OLD's withdrawn. The panel says SAVED iff the report is Saved (a Saved, a NotSaved with the restore, an
   Unconfirmed after a crash with the card reported gone). The serial words are `formatSaveReport`'s with
   `BootReadsBackup::No`. Nothing but `saveRecordSafely` writes the record (the log has no replace outside it).
5. **(A2) The watch and its hold** (Re-check R12, R15):
   - a declared port the push turns → the WARNING line with the drive's travel, the MECHANISM PORT TURNED view and
     ZERO replace operations while held; SAVE → saved without it, NOTE `PORT N KEPT AS A MECHANISM`; RUN AGAIN or a
     disable → the card unchanged;
   - **a drive port declared as a mechanism over a card whose record drives with it** → the view's `the card's
     record DRIVES with port N` line, and RUN AGAIN leaves both files byte-identical (the probe's
     `watch-typo-again`);
   - **(the check of D6 (a), C2/C3) the discard words follow the CARD, not the reference:** the same typo on an
     EMPTY card with NO typed ports → held, RUN AGAIN → `NOTHING SAVED: this run is discarded, the card is unchanged
     (it holds no usable record)`, then held again, a field disable → `… NOTHING SAVED: the card is unchanged (it
     holds no usable record)`, no file (the probe's `empty-watch-untyped`); and with NO card → both say `there is
     no SD card in the brain` (`nocard-watch-again`);
   - **flipped AND a declared port turned** → FRONT FLIPPED first; USE NEW → MECHANISM PORT TURNED next (its
     buttons re-armed); nothing written until both are answered;
   - one that does not turn → "did not turn", no hold; one the read refuses → "could not be read -- not judged",
     no hold;
   - no declared port → the three "nothing to watch / every motor is a drive candidate" lines;
   - in EVERY scenario the declared ports' fakes show zero `setVoltage`, `setBrakeMode` and constructions (the
     provider is never asked for them).
   **5b. (A2) Undeclared mechanism motors (Re-check R11):** port 8 still → one `port 8 did not turn with the push`
   line, the drive cartridge, 0 V + Coast and never a non-zero command; port 9 turned by the push → pulsed at 3, 6
   and 9 V (the levels asserted), then LEFT OUT with the "or a MECHANISM motor" words. With port 9 LOADED instead
   (it cannot spin) → the ROLLED/STILL refusal and the `if port 9 is a MECHANISM motor` line.
6. **(A2) The facts** (Re-check R13, R18): each refusal happens before the provider is asked for ANY device (its
   request count is 0) and paints its headline. The refusals: a NAMED port-0 entry, a port out of range, a port
   twice (each naming `mechanisms[i]`), kind unset, `config::DrivetrainKind::XDrive` and `HDrive` (`NOT A TANK`
   naming the kind), cartridge unset. Also: TOO FEW MOTORS names the declared ports when any was dropped; a declared
   port with no motor gives the census line and the run continues; the drop keeps the census order (the deleted
   `dropDeclaredMechanismPorts`' cases, moved here: an empty facts list, both pivots dropped, a declared port the
   census did not list). **(D6 (a)) The typed lists:** a port outside 1..21 (22, −22, −128), a port twice on one side,
   a port once on each side (by |port|), only one side typed — the LEFT only, and **the RIGHT only** (the check of
   D6 (a): `hasTypedDrive` must read both sides, mutation 53) — each `FACTS REFUSED` naming `typedDrive.left[i]` /
   `typedDrive.right[i]` with its value, before any device is requested; a mechanism problem is still reported first;
   0 slots are skipped (`{0, -21, 0, 12}` is valid and compacts to −21 +12); the compile-time cases of `factsProblem`,
   `hasTypedDrive`, `typedDriveDescription` and `typedDrivePortsFrom` as `static_assert`s (the probe's,
   `d6a/facts_probe/facts_probe.cpp`).
7. **(A1) The painter and the screens** (on `FakeBrainScreen`): the five repaint rules (a change of screen repaints
   everything; a colour change at once; a live number at most every 0.2 s; an in-place change padded to the old
   length; a vanished button erased); every literal within its budget (compile time); the sanitizer's cases
   (`'%'`, a control byte, an em dash, a 3-byte character, the cut, the pad) — moved from wherever they are pinned
   today (Part B's test 6 re-runs them through `printText`).
8. **The adapters through the shim:** **8a (A1)** `ProsBrainScreen` makes today's PROS calls in today's order (the
   recorded shim log for one fill, one outline, one print, one touch error). **8b (A2)**
   `ProsCalibrateIo::readShaftDegrees` reads through the C API, writing NOTHING (the shim's gearing, units,
   voltage and brake counters stay 0; its C-read counter counts 2 per push), and converts degrees, rotations and
   counts by gearing. A source pin: `hal/pros/calibrate.hpp` names no `pros::Motor`, `pros::v5::Motor` or
   `ProsMotor` (R6). `calibrateInitialize` and the state screens print their lines.
9. **The additions change nothing that exists:** **(A1, A2 each)** the WHOLE existing suite passes, and NO existing
   test TU is edited. The only existing test files a part may modify are `test/pros_shim/pros/rtos.hpp` (A1: the
   hook; behaviour identical while none is installed) and `test/CMakeLists.txt` (only if a new test needs a define:
   its sources are globbed, :79) — Re-check R5, measured as P16: 2,231 of 2,231. **(A2)** The panel's defaults are
   today's screens: a test renders every phase with the defaulted `viewFor()` and with `FrontRule::AskedAfterPush`
   and `FinishFacts{}` spelled out, and compares the views field for field.
10. **(A2) The golden test through the REAL PROS shell** (`test/calibrate_golden_test.cpp`, NEW — the re-check's
   harness, `amend/harness2.cpp`, made project code):
   - The program is composed exactly as `hal::pros::runCalibrate` composes it — `ProsDeviceRegistry`,
     `ProsDeviceProvider`, `ProsBrainScreen`, `ProsCalibrateIo` — except that the card is
     `ProsFileStore{<a temp directory>}` (the adapter's own host-test constructor). A source pin checks
     `runCalibrate` builds those same five, with the default card.
   - Robot two's facts are written LITERALLY (Tank, BLUE, {6 "claw pivot 6", 7 "claw pivot 7"}, the robot's name,
     IMU 2, **and NO typed drive ports** — what Part B derives, its table not being valid: the check of D6 (a), C1;
     the D6 (a) draft typed them here). The typed scenarios add LEFT −11 12 −13 14 −15 | RIGHT 20 −19 18 −17 16 (the
     probe's case 8) or their own lists. It runs through the shim, on the plant.
   - Scenarios: `one`, `clean`, `slip18`, `curved`, `nocard`, `noimu`, `disable`, `flip-again`, `flip-new`,
     `flip-disable`, `swap`, `watch`, `watch-typo-again`, `watch-typo-save`, `undeclared`; **(D6 (a)) `typed-first`
     (an empty card, the typed ports' end leading — it replaces the draft's `first-untyped`, which is now robot two's
     own `one`), `typed-flip-new`, `typed-flip-again`, `typed-flip-disable`, `typed-card-wins` and its untyped
     control `typed-card-wins-untyped` (asserted EQUAL), `typed-undecodable`, `typed-undecodable-flip`, `typed-tie`,
     `typed-nocommon`, `typed-watch-typo`; (the check of D6 (a)) `typed-partial`, `empty-watch-untyped`,
     `undecodable-untyped`, `bakonly-garbage-untyped`, `bakonly-garbage-typed`, `emptyfile-untyped`,
     `nocard-watch-again`, `typed-nocard`** — 34 scenarios, the facts overridden per scenario as the probe's
     `d6a/v3/harness4.cpp` does. **A seeded scenario never runs without its seed** (the check's C2: the draft's
     runner seeded by name pattern and let a missing seed leave the card empty; `v3/run.sh` refuses), and no
     empty-card scenario's name matches a seeding pattern.
   - Each one's serial log, draw log and card bytes equal committed goldens. The host is deterministic, and the T
     lines are pinned as they print.
   - These goldens are the end state's: the re-check measured them byte-identical with and without C1's removal
     (P18: 69 of 69), and D6 (a)'s again (P25: the 15 scenarios' 69 files, the D6 scenarios' 60, all 143 of
     the 33 probe scenarios), and the check's again (P33: 185 of 185, the 43 probe scenarios), so Part B must leave
     them unchanged.
11. **(A2) The team file's example test** (`test/calibrate_team_file_example_test.cpp`, §3.1): §1.1's 21 lines
   verbatim, compiled through the shim and the host `main.h`, plus one case that `factsProblem(kOurRobot).why` is
   null and the three facts read back (and `hasTypedDrive(kOurRobot)` is false — the file is unchanged by D6 (a)).
   **(D6 (a))** One more case holding the chapter's typed-port line (`.typedDrive = {.left = {...}, .right =
   {...}}`) and the `typedDrivePortsFrom(kRobot.drivetrain.tank.description)` form — over a real `inline constexpr
   config::RobotConfig kRobot` tank in the test, so the chapter's spelling is the test's — in their own facts
   objects, each `factsProblem(…).why == nullptr` — so `check-examples` finds every line the chapter quotes (both
   compiled clean with the suite's flags: `d6a/team3/main_typed.cpp`, `d6a/v2/team_config_real.cpp`).

**Mutations** (fresh binary each; the stale-binary recipe, HANDOFF §3 — `touch` the includers, build
`shulib_tests`, REQUIRE `Linking CXX executable shulib_tests`, observe RED, restore, relink, GREEN). The part that
owns each is in brackets.
1. [A1] `compareFronts` FLIPPED on `>=` instead of `>` → the tie pair.
2. [A1] `signOnly` counted as flipped → the re-mounted pair.
3. [A1] `compareFronts` compares signs only, ignoring sides → the re-mounted and cable-across pairs (re-check:
   replaces the draft's `flipFront` mutation).
4. [A2] USE NEW saves anything but the measurement as pushed (e.g. keeps the card's record) → the USE NEW byte
   test and the `swap` golden.
5. [A2] The front check skipped (save at once) → the HELD test (a replace appears).
6. [A2] RUN AGAIN from a held screen saves the held run first → the disable/RUN AGAIN byte tests (re-check:
   replaces the draft's KEEP OLD mutation).
7. [A2] The check compares against the record FILE only (`.bak` ignored) → the damaged-file case.
8. [A2] The no-card branch prints the FIRST words → the no-card golden.
9. [A2] The panel told Saved on an Unconfirmed report (B1 mutation 1).
10. [A2] `BootReadsBackup::Yes` passed (B1 mutation 2).
11. [A2] `saveAndReadBack`'s direct write left beside the safe save (B1 mutation 3; a source pin finds a second
    writer).
12. [A2] The watch commands a declared port (`setVoltage(0)`) → the counters.
13. [A2] The watch's threshold made 0 → "did not turn" becomes a WARNING and a HOLD in the clean run (the clean
    golden).
14. [A2] A declared port built as a candidate (the drop removed) → the provider's request count for port 6.
15. [A2] A refusal moved after the first `devices.motor()` → the request count 0.
16. [A1] The painter's live-number throttle removed → the repaint test.
17. [A1] `screenSafeLine` passes `'%'` → the sanitizer case.
18. [A2] The watch's read constructs `pros::Motor{port, gearset}` → the shim's gearing counter and the source pin
    (R6).
19. [A1/A2] A `program_detail` namespace instead of `detail` → `api_doc_tool check-coverage` names the leak (or
    `check-fresh`).
20. [A2] `viewFor`'s `rule` default flipped to `ByPush` → today's `calibrate_panel_test.cpp` goes red (the WHICH
    END cases), and so does test 9's comparison.
21. [A2] The auto-answer removed → the clean scenario stops at the WHICH END view (the operator finds no live
    button and the run times out, named).
22. [A2] `TouchGate::touch` reads Front as `Answer::FrontFirst` on a held screen → the HELD test (a USE NEW touch
    saves nothing and the view does not change).
23. [A2] The MECHANISM PORT TURNED hold removed (warn only) → `watch-typo-again`: the card changes.
24. [A2] USE NEW on FRONT FLIPPED saves without asking the pending MECHANISM hold → test 5's flipped-and-turned
    case.
25. [A2] The "did not turn" DONE line dropped → test 5b.
26. [A2] `factsProblem` skips a named port-0 entry → test 6.
27. [A2] The not-Tank check removed → test 6's X-drive case (the calibrator then starts on an X-drive's facts).

**(D6 (a)) Mutations 28–42 — each applied to the probe's draft (`d6a/mut/mutate.py`), built into the golden harness
and run: every one KILLED, on the scenario named (`d6a/mut/mutate.log`, `mutate2.log`; all eighteen — these fifteen
and B's 8–10, which the logs call MB9–MB11 — re-run in ONE pass with the final harness in the continuation:
`d6a/v2/mutate_rerun.log`, 18 of 18 killed, B8 and B9 also against the parity probe, `v2/fp_MB9`, `fp_MB10`).** The
builder's test for each is in brackets after the arrow. **The check of D6 (a) re-ran all fifteen on `d6a/v3/p3r/`
against ALL 43 scenarios (`d6a/v3/mutate_v3.py`, `mutate_v3.log`): every one KILLED.** Since robot two's own facts
are now untyped (C1), the kills that named robot two's `one` or `nocard` now come from `typed-first` (29, 31, 32,
37, 41) and `typed-nocard` (40); 38 is also killed by `empty-watch-untyped`.
28. [A2] The typed ports checked even when the card holds a record → `typed-card-wins` (its serial differs from the
    untyped control) [test 2, card-and-typed; golden].
29. [A2] The typed ports ignored (no D6) → `typed-flip-new` saves without holding; `one`'s lines [test 2, flipped].
30. [A2] Files that do not decode NOT checked against the typed ports → `typed-undecodable(-flip)` [test 2].
31. [A2] `typedDriveDescription` drops the signs → `one`: CANNOT TELL instead of SAME [test 2, agreeing].
32. [A2] `typedDriveDescription` swaps the sides → `one`: CANNOT TELL [test 2, agreeing].
33. [A2] `factsProblem` misses a port listed once on EACH side → `facts-typed-dup` runs [test 6].
34. [A2] `factsProblem` allows |port| > 21 → `facts-typed-range` runs [test 6].
35. [A2] `factsProblem` allows one side typed → `facts-typed-oneside` runs [test 6].
36. [A2] The typed FLIPPED hold prints the card's words (serial), or `FinishFacts::typedPorts` left false (the view
    paints the card's words) → `typed-flip-new` serial / screen [test 2, flipped].
37. [A2] SAME against the typed ports gives no NOTE → `one`'s screen [test 1, test 2].
38. [A2] A discarded held run always says "the card keeps its record" → `typed-flip-again`, `typed-flip-disable`
    [test 2, flipped].
39. [A2] The MECHANISM PORT TURNED evidence ignores the typed reference → `typed-watch-typo` [test 2].
40. [A2] No card: the typed ports consulted anyway → `nocard` [test 2, no card; golden].
41. [A2] The `card :` line ignores the typed ports → `one`'s serial [test 1].
42. [A2] USE NEW against the typed ports: no FIX line and no NOTE → `typed-flip-new` [test 2, flipped].

**(The check of D6 (a)) Mutations 43–54 — each applied to `d6a/v3/p3r/`, built into the golden harness and run on
ALL 43 scenarios: every one KILLED, each by the scenarios named (`d6a/v3/mutate_v3.log`).**
43. [A2] (the checker's X1) The discard words keyed on the REFERENCE, not the card → `empty-watch-untyped` only
    [test 5, the empty-card discard; golden].
44. [A2] (X3) Typed branch: "no record yet" decided by the record FILE alone → `bakonly-garbage-typed` [test 2,
    backup-only].
45. [A2] Untyped: the FIRST decided by the record FILE alone → `bakonly-garbage-untyped` [test 2, backup-only].
46. [A2] The per-file lines skip the backup → `bakonly-garbage-*`, `undecodable-untyped`, `typed-undecodable(-flip)`
    [test 2].
47. [A2] An EMPTY file printed with the decoder's (empty) reason → `emptyfile-untyped` [test 2].
48. [A2] No card: the discard words ignore it ("the card is unchanged") → `nocard-watch-again` [test 5].
49. [A2] No card: the `card :` line says "no usable record" → `nocard`, `nocard-watch-again`, `typed-nocard`
    [test 1, test 2].
50. [A2] A partial typed match painted `FRONT MATCHES YOUR TYPED DRIVE PORTS` (the D6 (a) draft's rule) →
    `typed-partial`, `typed-watch-typo` [test 2, partial].
51. [A2] The per-port difference lines dropped (the NOTE kept) → `typed-partial`, `typed-watch-typo` [test 2].
52. [A2] A SIGN-only port reported as a SIDE difference → `typed-partial` [test 2].
53. [A2] (X2) `hasTypedDrive` reads the LEFT side only → `facts-typed-rightonly` runs untyped instead of refusing
    [test 6].
54. [A2] (X4) CANNOT TELL against the typed ports paints the MATCHES NOTE → `typed-tie`, `typed-nocommon` [test 2].

**A1's DoD:**
- [ ] §1.2 items 1-5 and 8 written, every public member documented; `git diff --stat -- src/` EMPTY;
- [ ] tests 3 (with D6 (a)'s two typed-table pairs), 7, 8a and 9 green; the suite's counts logged before and after —
  every pre-existing case passing, none edited;
- [ ] mutations 1-3, 16, 17 and 19 observed RED on fresh binaries, each logged with the failing line;
- [ ] the CI guards (PROS-free outside `hal/pros/`, path-anchored; no `sim/` in the core), the fence test (the new
  shim headers carry the guard), the ARM header gate over EVERY header, `api_doc_tool` self-test / coverage /
  check-fresh / check-examples / check-removability, `doc_staleness_audit.py`, the briefing regenerated LAST;
- [ ] `src_build_gate.py check` run once (A1 touches no `src/` file);
- [ ] docs per §3.1 (A1's share); `CAL1-PROGRESS.md` created FIRST.

**A2's DoD:**
- [ ] §1.2 items 6-7 and the ONE changed library header (`calibrate_panel.hpp`, additions only), every public
  member documented; `git diff --stat -- src/` EMPTY;
- [ ] tests 1, 2, 4, 5, 5b, 6, 8b, 9, 10 and 11 green; the suite's counts logged before and after;
- [ ] mutations 4-15, 18 and 20-54 RED on fresh binaries, logged (28–42: D6 (a); 43–54: the check of D6 (a));
- [ ] the same gates as A1. `src_build_gate.py check` shows `src/` builds unchanged: the changed
  `calibrate_panel.hpp` is in today's program's includes (P16 measured PASS, and today's program 33 of 33
  byte-identical);
- [ ] HA-187 in the register; docs per §3.1; the PROGRESS log carries every ruling the executor made beyond this
  brief, with its alternative;
- [ ] (D6 (a), as the check amended it) the 34 golden scenarios committed; `typed-card-wins` asserted equal to its
  untyped control and `typed-nocard` to `nocard`; every new screen string — the six typed literals included —
  inside `kAll`'s compile-time check.

### 2.B Part B — robot two on the standard Calibrate, the WHICH END question removed, the side-by-side proof, the documents (~1⅞–2⅛ builder-days, re-derived from its items by the check of D6 (a))
**Builds** §1.8 and C1's removal (§1.2 "Changed library files in Part B"). **Starts** after A2 lands.
**Size:** the glue, the facts, `main.cpp`, `screen_text.hpp` and its new host test, the messages, the gate row and
self-test case, the deleted drop (~¼–⅓);
C1's removal and its four re-pinned test files (~½); the side-by-side proof (~¼ — the harness exists); the
documents — Station G rewritten from the code, Station 6, the FAQ, guide 07, HA-139, the changelog's BEHAVIOUR
CHANGE (~½); gates and verification (~⅓, Re-check R1: the gate's self-test edit and its ~11 min run); **(D6 (a),
+~⅛)** the typed-port mapping in `calibrate_facts.hpp` (8 lines in the probe), the facts test's parity case,
mutations 8–10, and robot two's sentence in Station G and the chapter. *(The check of D6 (a), C1/C5: the mapping is
now `withTypedDriveFrom` through the boot's own function — 4 lines, `d6a/v3/p2r/` — plus the glue's line; the
parity case gains the valid/invalid table cases; mutations 8–11, 11 a source pin; Station G's sentence is simpler
and the chapter's robot-two sentence is withdrawn — the same ~⅛. The items sum to ~2 (1 23/24–2 1/24), so B is
**~1⅞–2⅛**; the earlier ~1⅝–2⅛ contained the sum but was not derived from it.)*

**The side-by-side proof** — BEFORE deleting today's program, the executor captures its goldens with the probe's
method (`harness/` — build today's program against the shim, run the nine scenarios) into
`test/golden/calibrate/old/`, then builds robot two's glue over A2 and runs the same scenarios, and runs
`amend/classify2.py`'s rule table (moved into the test as data; reasons G, F, S, W, M, T): **every differing line
has a reason, residual 0** (the re-check measured it for the seven comparable scenarios, P17), and the records
compare as §0.5's re-check paragraph says. The rule table and the reasons go into `CAL1-PROGRESS.md` and the
changelog's entry; the OLD goldens are deleted with the old program (they described a program that no longer
exists) — the NEW ones stay as the pin.

**Tests:**
1. **`test/calibrate_golden_test.cpp`** (A2's test 10) — its facts now come from
   `withTypedDriveFrom(calibrateFactsFor(kRobotTwoChassis, kRobotTwoMechanisms), kRobotTwoChassis)` — the glue's own
   composition (one line); **every golden UNCHANGED** — which proves at once that robot two's derived facts equal
   the literal ones (NO typed drive ports, its table not being valid: the check of D6 (a), C1), and that C1's
   removal changed nothing anyone sees or the card holds (measured: `d6a/run_end/` = `d6a/run_new/`, 143 of 143;
   with the check's form `d6a/v3/run_end/` = `v3/run_new/`, 185 of 185, the facts derived from the tables in both).
2. **`test/calibrate_facts_test.cpp` (NEW)** — `calibrateFactsFor` over robot two's tables gives
   `config::DrivetrainKind::Tank`, BLUE,
   mechanisms {6 "claw pivot 6", 7 "claw pivot 7"} (the table's names), robot "2026 TANK CHASSIS (robot two)",
   IMU 2; over the bench bot's and the X-drive's tables, an UNSET cartridge (both refuse at Calibrate, as today —
   and the Makefile builds Calibrate for neither); a pneumatic entry never becomes a facts port; every robot's
   derived facts pass `factsProblem(…).why == nullptr` (R18). **(D6 (a), as the check of D6 (a) corrected it, C1)
   The typed ports — `withTypedDriveFrom(calibrateFactsFor(t, m), t)`:** for every `RobotId`, its typed sides equal
   `tableToDescription(t)`'s bit for bit when `tableToDescription(t).valid()`, and are EMPTY otherwise — the parity
   that makes "SAME as the typed ports" mean "the boot, which would drive from this table, drives the way this push
   measured". Today every robot's is empty (robot two: "LEFT inches per radian is 0"; the bench bot: unsigned; the
   X-drive: no cartridge — asserted with the reason, so a table that becomes valid turns this case red, and its
   author re-reads Station G's robot-two sentence, §3.2). Also, literally in the test: robot two's table made valid (track width 12.5, ratio 1.0) carries
   LEFT −11 12 −13 14 −15 | RIGHT 20 −19 18 −17 16; and with each one fact undone again — track width, ratio, wheel,
   IMU unset, IMU on a drive port, cartridge unset, unsigned, a port on both sides — none; every result passes
   `factsProblem` (measured: `d6a/v3/parity_probe/`, 12 of 12 — P34).
3. **`test/mechanism_table_test.cpp`** — the Calibrate call-site pin (:786-861: "runCalibrate() is a whole PROS program … it
   cannot be compiled, let alone run, here") is REMOVED (the source reader's own case, :772, stays while
   `codeOnly` has other users in the file — :336 reads the table with it) — the call site now RUNS (A2's tests 1, 5,
   6) — and the deleted drop's cases are gone with it (moved, A2). Logged: the pin, what replaced it, where.
4. A source pin over the glue (B1's, reworded): `src/programs/calibrate_program.cpp` names `runCalibrate` with
   `withTypedDriveFrom(kCalibrateFacts, kChassis)` (the check of D6 (a): on robot two the mapping gives nothing, so
   only this pin sees it dropped — mutation 11); no `ProsBlockSink`/`ProsBlockSource`, no `pros::` call, no
   `printf`.
5. **C1's re-pins:** `drivetrain_calibrator_test.cpp` (its test 5 "BackFirst inverts every sign" becomes "a push
   led with the other end puts every port on the other side with the other sign, and swaps the sides' scales,
   against the front-led run's LITERAL ports and scales" — STRONGER: a property of the whole run, not of one sign
   flip; there is no `flipFront` to compute it with, Re-check R2), `calibrate_panel_test.cpp` (its WHICH END
   cases go; `PanelButtonId::Front`/`Back` → `SaveHeld`), `median_nonfinite_test.cpp`, `calibration_test_rig.hpp`
   (P18: the only four CAL1 TUs of the 210 the removal breaks); each edit logged with what it pinned before and
   after; A2's test 9 (the defaulted view equals `AskedAfterPush`) is deleted with the parameter.
6. **`test/screen_text_test.cpp` (NEW, Re-check R7)** — `printText` run through A1's recording shim `screen.h`:
   the sanitizer's cases (`'%'`, a control byte, an em dash, a 3-byte character, each size's cut, the pad), the
   recorded call order (eraser, pen, `print_at(…, "%s", …)`) and `g_screenPrints`; the test mutation 4 turns red
   (feasibility probed: `amend/printtext_probe/`).

**Mutations:** 1. The glue passes a facts struct with the cartridge cleared → the golden (refused). 2. `main.cpp`'s
Calibrate disabled case calls `benchStateScreen` → the gate's step 8 (program symbols) or the golden's state
screen. 3. `calibrateFactsFor` drops a mechanism entry → the facts test and the `one` golden's census line.
4. `printText` reverts to its own sanitizer with `'%'` passed → the sanitizer case through `printText`.
5. The removed WHICH END branch left reachable (the push view shows it after DONE) → the golden test's `one`
(the screen log gains the WHICH END screen). 6. The calibrator made to keep each port's side on an other-end push
(flipping signs only) → the re-pinned test 5 goes red against its literal ports; if it does not, the test is
rewritten until it does. 7. Self-test case 26's Calibrate plant left on `calibrate_program.cpp` → `self-test` dies
on `_replace`'s assertion (the R1 failure, logged as the reason for the edit). **(D6 (a), each run on the probe:
`d6a/mut/`, where they are named MB9, MB10 and MB11; re-run in `d6a/v2/mutate_rerun.log`; **re-written for the
check of D6 (a)'s `withTypedDriveFrom` and run on ALL 43 scenarios plus the parity probe, `d6a/v3/mutate_v3.log`**)**
8. The mapping ignores `valid()` (the D6 (a) draft's rule: robot two typed from its undrivable table) → the facts
test's parity case, and 14 of robot two's goldens (`one`, `clean`, … — FRONT MATCHES instead of FIRST RECORD;
killed, and the parity probe fails 8 cases). 9. The mapping swaps the sides → the parity case and the literal ports,
and 8 typed goldens (`typed-first`: CANNOT TELL instead of SAME; killed). 10. The mapping dropped → the parity case
(the made-valid table types none), and 8 typed goldens (`typed-first`, `typed-flip-new`: FIRST RECORD and no hold;
killed). 11. The glue runs `kCalibrateFacts` without `withTypedDriveFrom` → test 4's source pin (it survives every
golden: on robot two, whose table is not valid, it is equivalent; measured).

**Part B's DoD:**
- [ ] the side-by-side proof logged: every scenario, every reason, residual 0; the timing control re-run;
- [ ] tests 1–6 green; mutations 1–11 RED (8–11: D6 (a), as the check of D6 (a) re-wrote them); the suite's counts;
- [ ] `src_build_gate.py self-test && check`: all 7 builds link, 0 disallowed warnings, the beacons unchanged
  (`tank-calibrate` still `shulib-robot-variant=tank-calibrate`);
- [ ] the package for `ROBOT=tank PROGRAM=calibrate` carries the library's strings (`MEASURES ITS OWN
  DRIVETRAIN`, `FRONT FLIPPED`, `MECHANISM PORT TURNED`, D6 (a)'s `checked against your typed drive ports`, and the
  check's `FRONT MATCHES, SOME TYPED PORTS DIFFER` and `NO SD CARD in the brain`) and not the old ones (`WHICH END
  FIRST?`, nor the draft's `KEEP OLD`) — `arm-none-eabi-strings` (P18 measured exactly this; P30 with D6 (a); P36
  with the check's fixes);
- [ ] docs per §3.2; the status pointers true after the commit; `CAL1-COMPLETED.md`.

## 3. Documentation (same change as the code — pace rule 3; prose review logged — rule 4)

### 3.1 Parts A1 and A2 (A1: the changelog's seams entry and `docs/api/`; A2: the rest — 5 public documents in all, plus the nav's two index lines)
- **`docs/guide/<next>-calibrating-your-robot.md` (NEW)** — the number is the next free at A2's tip (18 unless
  G3a Part 1 has taken it; mkdocs nav and `docs/guide/README.md` gain the row). For a TEAM, in this order: what
  Calibrate measures and what you state (§0.4's table); **your file** (§1.1, VERBATIM — bound by `check-examples` to
  a COMPILED copy: A2 adds `test/calibrate_team_file_example_test.cpp` — the 21 lines verbatim, compiled by
  the host suite through the shim and the host `main.h`, plus one case that checks `factsProblem(kOurRobot).why`
  is null and the three facts read back; its name matches `check-examples`' source glob `test/*example*_test.cpp`
  (`tools/api_doc_tool.py` :1836), so every line the chapter quotes must be a line of it; if G3a's
  `<!-- example-source: … -->` directive has landed by then, the listing names the file too); the run
  (IMU still → PUSH FRONT FIRST, 48 in along a tape, the end that leads is the front → stand clear: the pulses
  → the spin → SAVED); what each finish means (SAVED, NOT SAVED, SAVE UNCONFIRMED — B1's words; the backup file;
  "to reset, delete BOTH files on a laptop"); FRONT FLIPPED and its two choices, with BOTH causes the records
  cannot tell apart — a wrong push (RUN AGAIN) and drive cables swapped between the sides or a deliberate change
  of front (USE NEW) — Re-check R10; MECHANISM PORT TURNED and its two choices (R12); **list EVERY motor that is
  not a drive motor — an unlisted one may be powered at 3, 6 and 9 V; with nothing declared, unplug the
  mechanism motors first (R11)**; an X-drive must not run Calibrate, and the facts refuse it by name (R13); **the
  typed drive ports (D6 (a)): a team whose drive program already types its drive ports MAY state them
  (`.typedDrive`, the one line or the `typedDrivePortsFrom` form — both quoted from test 11); with no usable record
  on the card, Calibrate checks the front against them; FRONT FLIPPED against them means the BACK led by mistake
  (RUN AGAIN) or the typed ports are wrong / the cables moved since (USE NEW, then fix the typed ports), and USE NEW
  reverses every auton tuned on the typed ports; SAME paints `FRONT MATCHES YOUR TYPED DRIVE PORTS` only when every
  port agrees, else `FRONT MATCHES, SOME TYPED PORTS DIFFER` with each port on the serial log (the check of D6 (a));
  a robot with neither a record nor typed ports gets only the FIRST RECORD note, so push its first Calibrate with the
  end its autons call the front**; "nothing in
  this program has run on a robot in this form" (the calibrator ran on robot two in its R3d Part 2F form,
  2026-09-24, cited by date). No `docs/internal` path (C7).
- **`docs/guide/12-when-things-go-wrong.md`** — rows: "Calibrate says FRONT FLIPPED" (both causes, which to touch
  for each — Re-check R10); "it says MECHANISM PORT TURNED" (SAVE if it is a mechanism the push turns, RUN AGAIN
  and fix the facts or wiring if it is a drive motor — R12); "the finished screen says FRONT NOT CHECKED"; "it says
  FIRST RECORD" (R16); **(D6 (a)) "it says FRONT FLIPPED and `typed ports: the end you led is BACK`" (RUN AGAIN after
  a wrong push; USE NEW when the front led, then fix the typed ports); "the NOTE says FRONT MATCHES YOUR TYPED DRIVE
  PORTS" (nothing to do: the first record agrees with every typed port — the check of D6 (a), C4: painted only on
  an exact match); **"the NOTE says FRONT MATCHES, SOME TYPED PORTS DIFFER" (the front is right, but the serial log
  names each port where the typed ports and this push differ — a wrong sign, a side, a port in one list only: fix
  each typed port it names, or anything that still drives from them fights itself; a typed port missing from this
  push may still be right — it was LEFT OUT, declared, or did not turn: check the run's LEFT OUT and watch lines)**; "the NOTE says FIX YOUR TYPED DRIVE PORTS" (they call
  the saved front the back: correct them, or anything that still drives from them drives backwards)**; "LEFT OUT / a pulse refusal names a port that is a mechanism" (declare it — R11); "FACTS
  REFUSED / NOT A TANK / TOO FEW MOTORS" (R13, R18); and B1's two (SAFE-SAVE §3.2): "it says SAVE UNCONFIRMED", "it
  says NOT SAVED and the record file is damaged".
- **`docs/changelog.md`** — "Added since 2.4 — the standard Calibrate" (A1 its seams, A2 the rest): the seven
  headers, the panel's additions (each new enumerator, `HoldReason`, `FinishFacts` with its `typedPorts`, the two
  defaulted parameters — every existing call unchanged; the five typed-port literals), the facts' optional
  `typedDrive` and its helpers (D6 (a)), the shim's screen, competition calls and C-API motor reads.
  **No behaviour change and no version bump**: additive, in no Freeze Register row (the T4 Part 3 and SAFE-SAVE
  entries' rule); today's programs paint and print exactly as before (P16: 33 of 33). Host only — nothing here has
  run on a robot.
- **`docs/hardware-assumptions.md`** — HA-187's index row and entry (§3.3) in a new "CAL1's addition" section; the
  status paragraph's count; the Held line (:39) drops `HA-187..HA-188` for "HA-187 (registered by CAL1); HA-188
  released unused — never reissued" in the register's own words. HA-136's text is unchanged (no threshold moves).
- **`docs/api/`** regenerated in each part (new pages for each new public header; `calibrate_panel` changed in A2;
  `drivetrain_calibrator` is not touched before B).
- Internal: `docs/internal/build-order.md` (CAL1's status line), `CAL1-PROGRESS.md`, the briefing regenerated.
- **Prose review, logged** (grep `docs/`, `README.md` and the worksheet — read every hit BEFORE writing that it
  was done): `shulib Calibrate`, `calibrate_program`, `viewFor`, `TouchGate`, `IBrainScreen`, `screenSafeLine`,
  `printText`, `record_store`, `saveRecordSafely`, `front`. A1 and A2 change no program, so a hit describing robot
  two's Calibrate is still true; log it as Part B's.

### 3.2 Part B (5 public documents, README only if false; D6 (a)'s robot-two sentence in A2's chapter WITHDRAWN by the check of D6 (a))
- **`docs/changelog.md`** — "Changed since 2.4 — the WHICH END question removed": **BEHAVIOUR CHANGE** (C1:
  `Answer::FrontFirst`/`BackFirst`, `CalibrationReport::backFirst`, `PanelScreen::WhichEnd`, `FrontRule` removed;
  `PanelButtonId::Front` renamed `SaveHeld` and `Back` removed — Re-check R2; the front is the end that leads the push); **What you
  must do**: a program that answered `FrontFirst`/`BackFirst` answers nothing now, and one that names `Front`
  uses `SaveHeld` (`Back` is gone; none outside this repository is known). No major bump: `calibration/` is in no Freeze
  Register row (`docs/roadmap.md` :127-145).
- **`docs/internal/chunks/R3a-BENCH-WORKSHEET.md`** (amended by Re-check R3: places by anchor, live line numbers of
  2026-10-08 in brackets) — Station G rewritten FROM THE CODE (the screens' words as `viewFor()` paints them:
  §1.4's held screens, the NOTE, the push words, the 1/4..4/4 headers; the serial lines as `run()` prints them;
  B1's outcomes — and, Re-check R17, that a SAVE UNCONFIRMED run leaves robot two's drive unable to boot until
  Calibrate saves again or SAFE-SAVE B2 lands; the two files and "delete both to reset"; G.0.1's interim "copy the
  record off the card first"
  [:746-752] (SAFE-SAVE §3.0) and G.4's RUN AGAIN paragraph [:884-890] rewritten now that the save is safe);
  Station 6 [:284] (the card holds the record and its backup); **Station M's M.0.2 [:1596-1598]** (its "only
  after SAFE-SAVE B1 is in the build" is stale once Part B lands); the WHICH END IS FORWARD box [:452-461]
  gains one sentence (Calibrate checks a new record's front against the card's — or, with no record on the card,
  against typed drive ports the drive program could drive from: D6 (a); robot two's table is not one yet, so its
  first record is checked against nothing). **Station G also carries, where the operator reads the finished
  screen's NOTE (D6 (a), as the check of D6 (a) corrected it, C1):** "With NO record on the card, nothing checks
  robot two's front. Calibrate checks a first record against typed drive ports only when the drive program could
  drive from them, and robot two's typed table cannot: its track width and gear ratio are not typed. Push with the
  CAMERA end leading — the front its 2026-09-25 record and its driver use. The 15/16 end, which
  `src/robots/chassis_table.hpp` calls the front, is its BACK. The finished screen says `FIRST RECORD -- the end you
  pushed first is FRONT`. If the 15/16 end led by mistake, that record is reversed: touch RUN AGAIN and push with
  the camera end leading. The screen then says FRONT FLIPPED against the record just saved; touch **USE NEW** (you
  are changing the front back on purpose). With a record on the card (the normal case), the card's record decides."
  Whoever makes robot two's table a valid description — G3a's D7 flip together with the track width and ratio
  (HA-125, HA-126) — rewords this sentence in the same change: from then on Calibrate checks an empty card's first
  push against the table (§5). *(The D6 (a) draft's sentence — "stops on FRONT FLIPPED … touch USE NEW … Touch RUN
  AGAIN only if the 15/16 end led by mistake" — is WITHDRAWN. It contradicted itself: a 15/16-led push never
  showed that screen. It also left out the 15/16-first push, which the draft SAVED under `FRONT MATCHES YOUR TYPED
  DRIVE PORTS`.)* **D.1.2 [:354] and F.2.1
  [:622] stay as written:** they are Bench Tests' MOTOR WATCH question, which CAL1 keeps (§1.8, §4). Written
  from the code, never from this brief.
- **`docs/hardware-assumptions.md` HA-139** (B1, SAFE-SAVE §3.2): the settle — after a SAVED run both files are 180
  bytes and byte-identical (`cmp`); the blast radius names the library's save and the boot's load; the 2026-09-25
  evidence (a Calibrate-written record decoded at eight boots) by date.
- **`docs/faq.md`** :160-173 — "the programs do not use it yet: until shulib Calibrate saves through it …" becomes
  true to the tree: Calibrate saves through it; the boot's fallback to the backup is still to come (SAFE-SAVE B2).
- **`docs/guide/07-getting-set-up.md`** :143 — "shulib Calibrate" is the library's standard Calibrate, robot two's
  facts in `src/robots/calibrate_facts.hpp`, and the chapter of §3.1 for a team's own.
- ~~**The chapter of §3.1 (D6 (a), one sentence, Part B …)**~~ **WITHDRAWN by the check of D6 (a) (C1):** robot two
  carries no typed ports, so a team's chapter has nothing to say about it. Its instruction is Station G's (above).
  The chapter keeps D6 (a)'s team-facing text (§3.1).
- **`README.md`** :304 — only if the prose review finds the programs list false or incomplete.
- **`docs/api/`** regenerated (Re-check R4): the removed and renamed entities (`drivetrain_calibrator.md`,
  `calibrate_panel.md`, `calibrate_program.md`, `front_check.md`, `all-entities.md`, the index) — `check-fresh`
  fails until it is.
- Internal: HANDOFF §5's "copy the record off the card first" item closed, and its S1/S1b line (live :724,
  "copy the record off the card first, G.0.1") reworded (Re-check R3); `build-order.md` (CAL1 complete;
  SAFE-SAVE's row: B1 done by CAL1; the Next block; the deviations row's status); `REMAINING-WORK.md`;
  `CAL1-COMPLETED.md`; the briefing regenerated LAST; the status board republished by the coordinator.
- **Prose review, logged** (grep `docs/`, `README.md`, the worksheet, `include/`, `src/`, `test/`, `tools/` — Re-check R1): `WHICH END`,
  `FRONT FIRST`, `BACK FIRST`, `FRONT or BACK`, `front or back`, `FrontFirst`, `BackFirst`, `backFirst`,
  `PushAwaitingFront`, `FrontRule`, `AskedAfterPush`, `kFrontBox`, `kBackBox`, `1/5`, `5/5`, `PUSH IT FORWARD`,
  `saveAndReadBack`, `calibrate_program`, `dropDeclaredMechanismPorts`, `claw pivot (declared)`, `restart slot 4`,
  `robot two MEASURES`, `screen_text`, `benchStateScreen`, `shulib_drivetrain.bak`, `off the card` (Re-check R3). Bench Tests' own MOTOR WATCH
  question stays (§8); the changelog's past sections stay as written (:949 is 2.2's history).

### 3.3 HA-187 — the text to add (copy it; the coordinator renumbers only if HA-187 is taken at merge) — AMENDED by Re-check R6/R12
Index row:
`| HA-187 | Calibrate's WATCH reads a DECLARED mechanism motor without changing it: PROS's C reads motor_get_position(), motor_get_encoder_units() and motor_get_gearing() by port write nothing and construct no motor object; the position, converted by the units and gearing they report, gives the shaft's turn between two reads of one push | **reasoned** (1) from the vendored libpros.a disassembled 2026-10-08; **invented** (2) and (3); none measured | Station G |`

Entry: **HA-187 — the declared-port watch reads a mechanism motor and changes nothing.**
*Claim:*
1. The three C reads write nothing to the motor. In the vendored `firmware/libpros.a` (`vdml_motors.c.o`),
   `motor_get_position`, `motor_get_gearing` and `motor_get_encoder_units` each call only PROS's registry check
   (`registry_validate_binding`, `registry_get_device`), the port mutex and `__errno`, plus ONE firmware getter
   (`vexDeviceMotorPositionGet`, `…GearingGet`, `…EncoderUnitsGet`). There is no `*Set` call and no motor object.
   The getters are closed firmware, so "a Get writes nothing" is reasoned, not seen.
2. On a motor this program never configured, `motor_get_position()` returns the shaft's position in the units
   `motor_get_encoder_units()` reports, scaled by the gearing `motor_get_gearing()` reports, whatever
   configuration the motor then has. Both stay the same between the watch's two reads (the start and the end of
   one push), so the difference is the shaft's turn, within the reported gearing's scale. The watch's 5°
   threshold sits far below a 48 in push's ~1,700° for any cartridge.
3. Reading a motor's position does not disturb it (a claw left where it was stays there).

*Source:* the disassembly above (2026-10-08, `arm-none-eabi-objdump -dr`, brief-CAL1 `amend/objdump/calls.txt`);
`include/pros/motors.h`; `ProsCalibrateIo::readShaftDegrees` in `hal/pros/calibrate.hpp`.

*Confidence:* (1) **reasoned**; (2) and (3) **invented**.

*Settle:*
- (2) and (3) at Station G: with robot two's claw pivots declared, turn one by hand during the push. The watch
  names that port and only that port, the save waits on MECHANISM PORT TURNED, and the other pivot has not moved.
- (1) needs a read of a pivot's gearing and units before and after a Calibrate run, with no other program in
  between. That is an extension of Bench Tests' raw probes, unowned (like HA-166's settle (b)). The design does
  not depend on it.

*Blast radius if wrong:*
- (1) a getter that wrote would leave the claw's motor with some configuration until the next program configures
  it (every program that drives the claws writes its own gearset at construction, `ProsMotor`). No motion results,
  and nothing is commanded.
- (2) a missed watch line, or a false MECHANISM PORT TURNED hold. The hold costs one touch (SAVE), and a missed one
  is the draft's state (no watch).
- (3) a claw that drifts while read: not expected of a read.

**Contained:** one program's serial lines, one held screen and one NOTE.

## 4. OUT, and explicitly rejected
**OUT (with the owner):**
- The Auton Tester and Bench Tests as library programs — ruling (3), **PKG1**. That includes Bench Tests' MOTOR
  WATCH, which still asks FRONT FIRST / BACK FIRST (`bench_r3a.cpp` :1200-1210, :1462-1464): its answer lives in memory for
  one power cycle and never reaches a record; whether it follows ruling (1) is PKG1's call (§8).
- Where a starter project keeps the team file, and how a team uploads Calibrate beside its match program —
  **PKG1** (§1.1 says only that it is one file).
- The boot's fallback to the backup and `BootReadsBackup::Yes` — **SAFE-SAVE B2** (now one token in the LIBRARY
  program, §5).
- GAINS and REST — **R5a Part C** (§1.6 leaves the place); every program's gains line — R5a Part D.
- The guards' general form for a generated robot, and the panel's calibration page — **G3a Parts 2 and 3**.
- A holonomic drive's calibration — R3d §2 OUT (spring, with R3c); the standard program refuses it by name (`NOT A
  TANK -- the facts say x-drive`, reachable since `.drive` is the config vocabulary — Re-check R13).
- **The residual gap after D6 (a): a robot with NEITHER a usable record on the card NOR typed drive ports in its
  facts.** Its first Calibrate has nothing to compare the front with, so the end that led is the front (ruling (1))
  and only the FIRST RECORD note says so. Such a robot has nothing typed that could drive it backwards — no table to
  tune autons on — so the hazard D6 closed (autons tuned on a typed table, reversed by a first record from the
  other end) cannot arise there; a robot made in the panel is this case (it types no drive ports, MECH-PANEL
  §4.1). **So is robot two, since the check of D6 (a) (C1):** its typed table cannot be driven from (no track
  width or ratio), so no boot drives it from that table and no auton is tuned on it; its facts carry none, and
  Station G tells its operator which end to push (§3.2). The only program that drives robot two from that table
  is Bench Tests' DRIVE station, a bench station that never runs an auton (§8). The chapter tells such a team to push its first Calibrate with the end its autons call the front. (D6 (a)
  closed the gap for every robot whose facts carry typed ports — decided 2026-10-08; the brief's (b), "state the
  gap", is the alternative not taken, D6.)
- A front check when the brain has NO card — none, typed ports or not (D6 (a)): nothing can be saved, so there is no
  front to protect.
- Any threshold (HA-136's) or screen budget (HA-136's estimates); the record's layout, version and golden.
- Any run on a robot, and R3d's close (its reproduction run) — the worksheet's Station G after Part B.
- The mechanism VOCABULARY (roles, wiring): CAL1's facts take a port and a name; mapping roles to them is the MECH
  core's (MECH-A5, §5).

**Explicitly rejected:**
- **Keeping WHICH END as a confirmation** ("is the leading end the front?") — the front would be asked on the
  screen, against ruling (1). (The held screens are not this: they offer "save as pushed" or "save nothing", and
  appear only when the records disagree — Re-check R2.)
- **KEEP OLD (saving the measurement turned to the card's front)** — Re-check R2/R10: it amends ruling (1), and
  on a robot whose drive cables were swapped between the sides it reverses every auton (measured: 0 of 10). D2 (a)
  keeps it only as an explicit amendment request.
- **A hold before the pulses on a "moved" candidate whose push travel is far below the drive's** — Re-check R11:
  no ratio separates a back-driven mechanism from a drive member (the probe's roller turned 0.6 of the drive;
  robot two's port 18 runs ~20 % short, HA-130); it would be a new invented threshold inside the calibrator, which
  A1/A2 do not touch; the calibrator already ends such a port safely (LEFT OUT or refused, never in the record —
  measured); the root guard is the declared facts, now stated where a team writes them.
- **A watch threshold relative to the drive's travel** — Re-check R15: the numbers are shown side by side for the
  operator instead; a ratio would be a new invented belief (HA-188 stays released).
- **The `card :` line on the starting screen** — Re-check R16: that screen shows only during the census, and the
  IMU-wait view after it fills its five rows for ~2 s; the finished screen's NOTE carries the fact where the
  operator can act.
- **A `front` field in the record** — a version bump every uploaded program refuses (`calibration_record.hpp` ~:229-
  233, SAFE-SAVE D1 (c)); the signs and sides already carry the front, and the check needs nothing more (§0.3).
- **Saving first and warning after** — D2 (c).
- **(D6 (a)) The typed ports checked even when the card holds a record** (or both checked, either able to hold) — the
  ruling's order is the previous record first: the record measured THIS robot, the table is older (the boot's own
  rule, `boot_check.hpp` :413-415), and a disagreement between the two is the boot's CROSS-CHECK line's business, not
  a reason to hold a run the record agrees with. Measured: the typed ports change nothing when the card decides
  (`typed-card-wins` = its untyped control).
- **(D6 (a)) "Files that do not decode" kept as NOT DONE when typed ports exist** — the boot treats a record that does
  not decode as absent and drives from the typed table when it is valid (`chooseBootSource`, :439-442), so the typed
  ports are exactly the front the robot drives with; checking against them is strictly more information.
- **(The check of D6 (a)) The typed ports of a table the boot cannot drive from** (the D6 (a) draft's mapping: any
  SIGNED table). Such a table is no front anything drives with. On robot two it pointed the check at the wrong end:
  a 15/16-first push saved the reversed front under `FRONT MATCHES YOUR TYPED DRIVE PORTS`, and the right push
  held (C1). The reference is now exactly the table the boot would use (`withTypedDriveFrom`: `valid()`).
- **(The check of D6 (a)) Keeping that mapping and ordering G3a's D7 before Part B instead** — D7 flips the ports,
  not the geometry. The table would still not be one the boot drives from, so §1.7's "exactly as the boot" would
  still be false, and the check would still run against a table nothing drives. Documents alone (the checker's
  (c)) would leave the misdirection in place.
- **(The check of D6 (a)) `FRONT MATCHES` on any SAME verdict against the typed ports** — a wrong sign on one typed
  port of ten still gives SAME, and the port fights its side-mates in anything that drives from the typed ports.
  The NOTE now says which (C4).
- **(D6 (a)) The typed ports printed in the banner, or a separate typed-port line on every run** — it would change
  every run's log, the seeded (normal) case included; the typed ports are printed only when they are the reference
  (the `typed drive ports:` line), so a robot with a record on its card sees byte-identical output (measured).
- **A second copy of the declared-port drop** in robot two's code — one rule, the library's (the deleted
  `dropDeclaredMechanismPorts`, §1.8).
- **A typed "slot" or "where to diagnose" fact now** — robot two loses "(Bench Tests, slot 3)" from its refusal
  lines (§0.5 G); PKG1 can add a stated fact if teams need it.

## 5. Coordination — who re-reads whom (the coordinator's, at this brief's commit and at each launch)
- **T4 Part 3** lands first. Its three reviews ran on 2026-10-07 and its review-fix run was still editing the
  worktree at 2026-10-08 09:31 (re-checked at 10:12: still no Calibrate file changed — the Re-check header); A2
  re-reads `src/programs/calibrate_program.cpp`, `src/main.cpp`,
  `src/common/screen_text.hpp` and `tools/src_build_gate.py` AS LANDED and moves that text. The probes ran on the
  worktree as of 2026-10-06 17:02 (re-checked 2026-10-07: only documents and one comment block had changed; re-checked
  2026-10-08 ~09:25: the fix run's thirteen code files are none of Calibrate's, and the ARM gate, the team project and
  the `src` gate re-ran with them overlaid — the header, §0.6 P14). If the fix run's LATER edits reach any file the
  header lists as unchanged, the launcher re-runs P16 (`amend/tree_a2/`: the suite, the `src` gate, today's program
  through the harness) before A1 starts. **LANDED 2026-10-08 14:52 as `bdcbc7c` on `shulib-v2`** (seen by the D6 (a)
  continuation): no file the header lists as unchanged moved, so no re-run is owed; the proofs were re-run on the
  landed tip with D6 (a)'s library anyway (§0.6 P32, all passing). A1's launcher still re-reads the four files above
  at A1's tip.
- **The MECH core** (`briefs/MECH-core.md`, a draft waiting for the team lead's go). **Re-checked 2026-10-08 ~10:45:**
  its own re-check already adopted this split as its **N9** (recommended (a)): CAL1 owns the drop and the watch;
  MECH-A5 edits no Calibrate code and only rewrites `calibrateFactsFor` to read the core's list (one fact per motor
  role, `{wiring[i].motors[r], specs[i].name}` — a two-motor lift gives two facts with one name); `declaredMotorRole`
  is not built; order CAL1 before MECH-A5; its landmine 8 now reads "never construct a `ProsMotor` for a declared
  port with a gearset … CAL1's HA-187 holds the read". So the draft's list of amendments for the coordinator is
  mostly DONE in that brief. **What remains for the coordinator to carry into it (Re-check):**
  - D4 as amended: the watch HOLDS the save (MECHANISM PORT TURNED) — the core's text says the watch "names" a
    port; a sentence that it holds;
  - R6: the watch reads through PROS's C API (no motor object at all), which is stricter than its landmine 8;
  - CAL1 needs no per-role name and no `role` field: the hold and the WARNING name the PORT, which identifies the
    motor; the mechanism's name per role (N9's mapping) is enough. The draft's "claw pivot 6 motor 1" request is
    WITHDRAWN;
  - `dropDeclaredMechanismPorts` is DELETED by CAL1 Part B (§1.8) — consistent with N9;
  - **(D6 (a)) MECH-A5's rewrite of `calibrateFactsFor` must KEEP the typed-port mapping** (§1.1: a signed table's
    drive ports into `typedDrive`, tableToDescription's rule). It reads the core's list for the MECHANISM half only;
    the drive half still comes from `kChassis`. Part B's parity case (§2.B test 2) and the empty-card goldens turn
    red if it is lost (mutation B10, killed in the probe). **As the check of D6 (a) changed it (C1):** the typed
    half is no longer inside `calibrateFactsFor`. It is `withTypedDriveFrom(facts, kChassis)` in the same file,
    applied by the glue. MECH-A5 rewrites the mechanism half and must leave `withTypedDriveFrom` and the glue's call
    in place. Part B test 4's source pin catches the call dropped (mutation 11; on robot two no golden can), and the
    parity case catches the function broken (mutations 8–10);
  - **order:** whichever of CAL1 Part B and MECH-A5 lands second rebases `src/robots/calibrate_facts.hpp` (N9 says
    CAL1 first);
  - shared documents (the changelog, the register, build-order, `docs/api/`) are merged by the coordinator. The
    core holds HA-182..HA-186; CAL1 holds HA-187.
- **G3a** (its D6′–D10 still the team lead's): Part 2's three Calibrate hunks are superseded (§1.7) — the banner
  from `kChassis.robot` (the facts' name already does it), the front's words and the front `static_assert`
  (ruling (1)), and `ChassisTable::front` — the coordinator strikes them from its §1.13, parts table and §5 item 5.
  Its §5 item 7 (the watch) is CAL1's (D4), so its Part 5's fallback ownership of the watch (re-check #10, "+~½",
  G3a ~:527) is released and Part 5's size drops by that half day; its landmine 22 then reads "the watch exists
  (CAL1)" rather than "until that exists". Part 3's calibration page reads the two files (`.bin`, `.bak`) and, if
  it ever shows a flip, mirrors `compareFronts` as ONE rule pinned against the C++ (its decoder's pattern) — and
  says what FLIPPED cannot tell (a cable swap, Re-check R10).
  **(Re-check R11)** Until G3a Part 5 a generated robot declares no mechanism (`kNoMechanisms`, G3a ~:440), so its
  Calibrate treats every motor as a drive candidate (and its banner says so): G3a Part 2 either generates the
  profile's mechanism ports into the mechanism table, or its bring-up instructions tell the team to unplug every
  mechanism motor before Calibrate. **(Re-check R13)** G3a Part 2's general form of the "Calibrate needs a tank"
  guard (`build_selection.hpp` :89-90, Makefile :176-178) must keep Calibrate building only for a robot whose
  generated kind is Tank — or the generated facts carry the generated kind, so `NOT A TANK` refuses — never a
  generated X-drive calibrated as a tank.
  **(D6 (a), decided 2026-10-08)**
  - **G3a's D7** (flip robot two's typed table to the camera end) ~~now also decides what robot two's FIRST Calibrate
    on an EMPTY card says~~ — **SUPERSEDED by the check of D6 (a) (C1):** D7 alone no longer changes Calibrate.
    Robot two's facts take the typed ports only once its table is a VALID description (`withTypedDriveFrom`), which
    needs D7's flip AND the track width and ratio (HA-125, HA-126; both UNSET today, P21). If the geometry were typed
    first, the boot itself would drive an empty card with the 15/16 end forward. Calibrate would then hold the
    camera-end push on FRONT FLIPPED and, after USE NEW, say FIX YOUR TYPED DRIVE PORTS. That is correct against
    that table, but it is the reversal G3a's D7 exists to remove. So **D7 should land before or with the
    geometry**. Once both land, the camera-end push gives SAME FRONT and
    `FRONT MATCHES YOUR TYPED DRIVE PORTS` (measured: the D7 table against the record a camera-end push saves, SAME
    8 of 8 — P24). The change that makes the table valid rewords Station G's robot-two sentence in the same change
    (§3.2), and Part B's facts test (§2.B test 2) turns red then by design.
  - **G3a's D10** (a NEW robot's drive ports OPTIONAL, still the team lead's): a generated robot whose generated table
    is a VALID description gets the typed check with no Calibrate line (§1.7). One that types no ports, unsigned
    ports, or signed ports without the geometry gets the FIRST RECORD note — the same rule the boot uses (it drives
    only from a signed, valid table; since the check of D6 (a) that is literally its own test, `valid()`).
  - **The panel** asks no drive ports (MECH-PANEL §4.1), so a robot made there carries none and its first Calibrate
    gets the FIRST RECORD note — §4's residual gap. Part 3's calibration page, if it ever compares a typed file's
    ports with the record, makes the comparison Calibrate now makes (mirror `compareFronts`, as above).
- **SAFE-SAVE**: B1 is CAL1's (§1.5). Its B2 item "Calibrate's `formatSaveReport` call flips to
  `BootReadsBackup::Yes`" now edits `include/shulib/calibration/calibrate_program.hpp`, not a `src/` file — the
  coordinator amends B2's file list. **(Re-check R8)** Landmine 12's consolidation of the copied `codeOnly`
  (`mechanism_table_test.cpp` / `record_store_platform_scan_test.cpp`) and robot two's fixture
  (`boot_source_test.cpp`) is **B2's**. CAL1 Part B leaves both copies and logs that. **(Re-check R17)**
  Order B2 directly after CAL1 Part B, before R5a Part C. Until it lands, the drive program's boot reads
  only the record file (§0.6 P8 as amended).
- **R5a Part C** edits `calibration/calibrate_program.hpp` and `calibrate_panel.hpp` (the library program), not
  `src/programs/calibrate_program.*` — the coordinator amends its parts table and file lists; it starts from §1.6's
  `RunKind` and the finished screen's button list.
- **The plan**: `REMAINING-WORK.md` row 18b's (:86) size becomes **~6–6½ builder-days** (three parts, A1 ~1¼–1⅜, A2
  ~2½–2⅞, B ~1⅞–2⅛, with the extra go — re-derived from their items by the check of D6 (a); the earlier ~4¾–6 and
  ~4¼–5½ sat below them), or ~6¼–6¾ if the team lead splits A2 (the check's "not settled" #1). The deviations row
  stays as written. The status board shows CAL1's parts and that D1–D6 are decided.

## 6. Checks (the coordinator re-runs every one; each part)
```sh
cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -6
python3 tools/src_build_gate.py self-test && python3 tools/src_build_gate.py check   # Part B; A1, A2: check once
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > /tmp/all.cpp
echo "int main(){return 0;}" >> /tmp/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/all.cpp -o /dev/null -Iinclude
python3 tools/api_doc_tool.py self-test && python3 tools/api_doc_tool.py check-coverage \
  && python3 tools/api_doc_tool.py check-fresh && python3 tools/api_doc_tool.py check-examples \
  && python3 tools/api_doc_tool.py check-removability
python3 tools/doc_staleness_audit.py
python3 tools/briefing_status.py generate && python3 tools/briefing_status.py check   # LAST
```
Plus: the two CI greps exactly as `.github/workflows/ci.yml` spells them (PROS only under `hal/pros/`, path-anchored;
no `sim/` in the core); **A1, A2:** `git diff --stat -- src/` empty; the team file built as a PROS project's only
source (P6's method: a copy of the tree with `src/` replaced by the file, `make`) links; **Part B:** the side-by-side
proof's log re-run by the coordinator from the committed harness, residual 0; `arm-none-eabi-strings` on the
`tank-calibrate` package (§2.B DoD); and Part B's beacon unchanged.

## 7. Landmines
1. **The front is in the signs AND the sides.** A check on signs alone calls a re-mounted motor a flip; a check
   on sides alone calls a moved cable a flip. Both, per port, by majority over the ports BOTH records list — and a
   tie is CANNOT TELL, never a guess (§1.4's pairs pin each).
2. **FLIPPED is what the RECORDS say, never what the operator did** (Re-check R10, replacing the draft's KEEP OLD
   landmine). A LEFT/RIGHT cable swap with the right end leading gives the same signed record as the other end
   leading; only the operator knows which happened. So the screen states the records' verdict and ties each touch
   to a cause; it never "corrects" the measurement, and nothing is ever turned to another front (no `flipFront`
   — ruling (1): every saved front is the end that led its push).
3. **Nothing is written while HELD.** The hold is a `SaveOutcome`, not a phase of the calibrator, and there are
   two reasons (`HoldReason`): FRONT FLIPPED, then MECHANISM PORT TURNED — one question per screen, in that order.
   RUN AGAIN (the held screen's right box) or a field disable discards the measurement and says so — the card
   keeps its record. Tests count replace operations (zero) and compare the card byte for byte.
4. **The front check reads the card BEFORE the save, and the save reads it again.** Two reads of one card in one
   program on one task: no race. But a card that misreads a GOOD record as absent at the check makes this run a
   "first record" with no flip check — the same two-fault limit SAFE-SAVE's D1 names (HA-139). Say so in the
   header; do not add a third read to "fix" it. (With typed ports in the facts, D6 (a), such a run is checked
   against them instead.)
5. **No card ≠ no record.** `cardPresent()` false is "front check NOT DONE"; a card with neither file is "the
   FIRST"; files that do not decode are "NOT DONE — check the robot drives the right way". The first draft printed
   "the FIRST" with no card (§0.5). With typed ports in the facts (D6 (a)), the last two become the typed check;
   "no card" stays NOT DONE (mutation 40). **(The check of D6 (a), C3)** No card is said as no card everywhere: the
   `card :` line (`NO SD CARD in the brain -- nothing can be saved, and the FRONT is not checked`) and a discarded
   held run (`there is no SD card in the brain`), never "no usable record" or "the card is unchanged" (mutations 48,
   49). "No record yet" means NEITHER file opened — a card holding only a backup, or an EMPTY record file, is "does
   not decode" (mutations 44, 45) — and every file that opened and did not decode gets its own line with its name
   and its reason, `EMPTY (0 bytes read)` for a 0-byte file (mutations 46, 47).
6. **The watch WRITES NOTHING.** Never a `ProsMotor` for a declared port (it writes the gearset at construction —
   T3's rule for a claw), never a `pros::Motor` at all, never `setBrakeMode`, never a command; PROS's C reads by
   port are the one read path (HA-187 (1), Re-check R6). A mutation constructs a `pros::Motor` with a gearset and the
   shim's counter and the source pin must go red.
7. **The fence.** Every `hal/pros/*.hpp` that includes `pros/` carries EXACTLY the `-Wshadow` + `-Wsign-conversion`
   fence before `namespace shulib` (`test/pros_adapter_fence_test.cpp`); the first draft's two new adapters had
   none.
8. **`namespace detail`, exactly.** `api_doc_tool` hides only a namespace NAMED `detail` (:543-548); a
   `program_detail` publishes its internals and fails coverage. The library's `calibration::detail` already
   exists (`calibrate_panel.hpp` :563, `record_store.hpp` :332) — the new helpers join it with names that cannot
   collide.
9. **The doc tool parses ONE declaration per statement and reads `<` as a bracket.** `1u << 0` in an initializer
   stops the whole host build at the doc gate (P11: "unterminated declaration"); write the three competition bits
   as `0x01`, `0x02`, `0x04`. Every public literal needs its `///` line, or lives in `detail` (P11: 31).
10. **The `src` gate's step 0 knows every program's entry names.** Calibrate's three new ones go into
   `PROGRAM_ENTRY_SYMBOLS` in the same change (P4: without them the gate FAILS naming each).
11. **`calibrate_program.cpp` is compiled into EVERY build** (the gate's rule, `src/` landmine 8 of SAFE-SAVE):
    the glue's five functions are external, so no `-Wunused-function`; the library header it includes is inline
    and costs only the Calibrate build's code.
12. **Only a literal `"%s"` reaches a PROS screen call**, and every screen literal is checked against its budget at
    compile time (today's `text::kAll` rule, moved). `IBrainScreen::print` sanitizes; nothing else may print.
13. **Every exit leaves the motors at 0 V + Coast** — the RAII all-stop, the field-disable poll every tick, the
    program-fault catch — moved, not rewritten; a test per exit (§2.A 1).
14. **The provider is the one owner of the motors** and asks the registry before it constructs (HA-141); the
    program never builds a `ProsMotor` itself (the first draft's `CalibrateDevices` keeps the registry only for the
    census).
15. **The host cannot see the brain's card**: SAFE-SAVE's HA-166 governs the save (no rename, no delete, `"wb"`
    empties at the open). CAL1 adds no file operation outside `saveRecordSafely` and `loadRecord`.
16. **"Host test plant", never "simulator"; no hardware claim.** The calibrator ran on robot two in its R3d Part 2F
    form (2026-09-24, one run stopped, one finished unrecorded); THIS program has not run on a robot. Every
    document says so, by date.
17. **Copy, then delete, today's program's goldens** (Part B): capture them BEFORE `calibrate_program.cpp` is
    replaced, or the side-by-side proof has nothing to compare against.
18. **An undeclared motor is a drive candidate** (Re-check R11). The program cannot tell a mechanism from a drive
    member except by the team's facts; one the push turns is pulsed at 3, 6 and 9 V (measured). Never "fix" this
    with a travel ratio inside the calibrator (rejected, §4) — say it where the team writes its facts, in the
    banner when nothing is declared, at DONE for every still candidate, and in the LEFT OUT and pulse-refusal lines.
19. **The held screens reuse today's ids in A2** (`PanelButtonId::Front` relabelled, `RunAgain` at `kBackBox`);
    a NEW id there is a `-Wswitch` failure in today's program and tests. Part B renames `Front` → `SaveHeld` and
    removes `Back` only once robot two's program no longer switches over them (P18 measured the rename leaves the
    goldens unchanged).
20. **Self-test plants move with the code they plant on** (Re-check R1): when a part deletes or rewrites text a
    `src_build_gate.py self-test` case plants on, re-point that case in the same change and run `self-test`, not
    only `check`.
21. **(D6 (a)) The typed ports are the reference ONLY with no usable record on a present card.** The card's record
    first (as `loadRecord` chooses it); the typed ports only when there is none — absent OR undecodable; no card, no
    check. A typed check that runs while the card holds a record would hold runs the record agrees with (mutation
    28). And the typed ports never CALIBRATE anything: they are compared, never copied into a record.
22. **(D6 (a)) One vocabulary for a sign and a side.** The facts' typed ports are the chassis table's signed entries
    AS TYPED, by `tableToDescription()`'s rule (a signed table only; never negated, never re-sided, never from an
    unsigned table). That is what makes "SAME as the typed ports" mean "the boot drives the way this push measured"
    (P24). ~~The mapping is a copy (the boot's function is not `constexpr`), so Part B's parity case pins it to
    `tableToDescription` for EVERY robot — do not delete it as redundant.~~ **As the check of D6 (a) changed it
    (C1):** a repository robot's typed ports come from `withTypedDriveFrom(facts, table)`, which CALLS the boot's
    own `tableToDescription(table)` and takes its sides only when `valid()` — the test the boot makes before it
    drives from a table (`chooseBootSource`). There is no copy left to drift. Never map a table the boot cannot
    drive from (any SIGNED table was the D6 (a) draft's rule: on robot two it rewarded the 15/16-first push with
    `FRONT MATCHES YOUR TYPED DRIVE PORTS`). Part B's parity case still pins the function for EVERY robot, valid and
    broken one fact at a time (mutations 8–10), and test 4's source pin keeps the glue's call (mutation 11, which no
    golden on robot two can see) — do not delete either as redundant.
23. **(D6 (a)) The words follow the reference.** Every line, screen and NOTE that names "the card" names "your typed
    drive ports" when they were compared; "the card keeps its record" is printed only when the card held one (else
    "the card is unchanged (it holds no usable record)"); `FIRST RECORD` only with neither a record nor typed ports.
    A golden per path pins each (§2.A test 10's D6 scenarios). **As the check of D6 (a) amended it:** the discard
    words follow the card's CONTENTS, never the reference (C2: `empty-watch-untyped`, the one scenario that tells
    them apart, is one of test 10's 34 goldens and a test 5 case; mutation 43); with no card they say there is none (C3, landmine 5);
    `FRONT MATCHES YOUR TYPED DRIVE PORTS` only when EVERY port in either list agrees — any sign-only, side-only or
    flipped port, or a port in one list only, paints `FRONT MATCHES, SOME TYPED PORTS DIFFER -- see log` and the
    serial log names each port (C4; mutations 50–52). A seeded scenario never runs on an empty card: the rig seeds
    by scenario, and a missing seed stops the run (C2).

## 8. Found while reading and probing (not CAL1's to fix unless named above)
- (Re-check) The probe tree `recheck_1008/tree_gate` mixes T4 Part 3's 2026-10-08 `src/common/dropped_autons.hpp`
  with its 2026-10-06 `test/dropped_autons_test.cpp`, so that test does not compile there (P18). It is T4's
  pair, consistent in the live worktree, and not CAL1's.
- (Re-check) `boot_check.hpp`'s `chooseBootSource` drives from a VALID typed table when no record decodes; robot
  two's is not valid (P21), so its drive REFUSES on that path — the reason R17's consequence is a refusal, not a
  reversal.
- Bench Tests' MOTOR WATCH still asks WHICH WAY DID YOU PUSH IT? — FRONT FIRST / BACK FIRST (`bench_r3a.cpp`
  :1200-1210), and its DRIVE station's refusal tells the operator to answer it (:1462-1464). Both are MOTOR WATCH's
  own question about its own in-memory capture, not Calibrate's, so nothing there becomes false at CAL1; whether
  Bench Tests follows ruling (1) is PKG1's (ruling (3)).
- `calibrate_panel.hpp` :644's `std::snprintf(out, cap, "%s", buf)` draws `-Wformat-truncation` from the host
  `g++ -O1 -Wall` (the probe harness's build); the suite builds without optimization and the ARM `-Os` gate is
  silent on it, so nothing fails today. Logged, not changed.
- Robot two's 2026-09-25 record (camera end) and its typed table (15/16 end) are FLIPPED, 8 of 8, by CAL1's own
  rule — G3a's N1 finding, reproduced with the library's check (P7). The first standard-Calibrate run on robot two
  is compared against the CARD's record, so pushing with the camera end leading gives SAME FRONT.
- The register's Held line uses "never reissued" for held numbers; releasing HA-188 therefore leaves a gap (§3.1
  words it).
- (D6 (a)) **The probe harness's "camera end" label is its own, not the robot's.** `harness2.cpp`/`harness3.cpp` call
  `pushDir = +1` "the FRONT (camera) end" and score FORWARD against it; on the plant that end is the one robot two's
  TYPED table calls front (the 15/16 end: `one` vs the typed table is SAME 10 of 10). Robot two's REAL camera end is
  the other one (its 2026-09-25 record is FLIPPED against the typed table, P7 row 3). This brief's statements about
  the real robot use the 09-25 record (P24 step 5), never the label; the builder's rig should name the ends by the
  table, not by the camera.
- (D6 (a)) **A typed drive port that is ALSO a declared mechanism is not refused by the facts.** D6 (a) does not
  need it: the census drops the declared port, the typed reference counts it "typed only", and the watch HOLDS when
  the push turns it, now with `your typed ports DRIVE with port N` as the evidence (`typed-watch-typo`). A
  facts-level refusal (`factsProblem`) would be a cheaper, earlier catch for a robot that types its ports — not
  added here (not D6's, and it would change the `watch-typo-*` scenarios on robot two's facts); logged for the MECH
  core's wiring checks or PKG1.
- (D6 (a)) Observed in P24: a FLIPPED record turns the robot the SAME way for a turn stick (10 of 10 ports at the
  same millivolts) and reverses only straight driving — a reversed front is a sign flip of the forward axis, and a
  spin about the centre is symmetric under it. So a reversed front shows first on the first straight move, not on a
  turn.

## 9. Summary for the team lead

### 9.1 The decisions (§ "Decisions for the team lead") — ALL DECIDED by the team lead, 2026-10-08 ("Accept, but add D6's check")
| # | Question | Decided | Evidence |
|---|---|---|---|
| **D1** | Where a PROS-dependent standard program lives | **DECIDED (a)**, as recommended — split: a PROS-free `calibration::CalibrateProgram` over seams + a ~130-line PROS shell in `hal/pros/calibrate.hpp`; the CI rule unchanged | `src` gate PASS (7 builds), ARM gate (214 headers), a team project of one file links, the suite unchanged — re-run for the revision (P16, P18, P19) |
| **D2** *(changed)* | The card's record calls the end that led this push its BACK | **DECIDED (b)**, as recommended — hold the save on FRONT FLIPPED: **USE NEW** (save as pushed) or **RUN AGAIN** (save nothing); the screen states what the records say and ties each touch to a cause; nothing written until a touch. The draft's KEEP OLD is withdrawn — kept only as an explicit amendment of ruling (1) | today's program saves the reversed robot silently; the records cannot tell a wrong push from a LEFT/RIGHT cable swap (through the program on the plant: the same signed record); on the swapped robot USE NEW → 10 of 10 forward, KEEP OLD's record → 0 of 10; RUN AGAIN then a front-led run → 10 of 10; a disable while held → the card byte-identical (P17) |
| **D3** | What a team writes; how robot two states its facts | **DECIDED (a)**, as recommended — one `CalibrateFacts` (kind in the config vocabulary, cartridge, EVERY non-drive motor's port; D6 (a)'s optional `typedDrive` appended LAST) + five one-line entry points — **21 lines, 5 comments, unchanged by D6 (a)**; robot two's facts DERIVED from its tables — its typed drive ports only when its table is one the boot can drive from (`withTypedDriveFrom`, the check of D6 (a), C1), which robot two's is not yet, so it carries none | links a V5 package as the only source, host `-Werror` clean; an X-drive's file compiles and is refused by name (P19); byte-identical, re-compiled and re-linked with D6 (a) (P31) and with the check's fixes on T4 Part 3 as landed (P36) |
| **D4** *(changed)* | Calibrate's watch of the declared mechanism ports | **DECIDED (a)**, as recommended — in CAL1, a read through PROS's C API that writes nothing (HA-187), and a declared port that turned **HOLDS the save** (MECHANISM PORT TURNED: SAVE or RUN AGAIN, the evidence on screen); the MECH core's brief amended | the draft's warn-only would replace a good 10-port record with a 9-port one (`watch-typo-save`); the hold leaves the card byte-identical (`watch-typo-again`); 0 writes to the declared ports in every run; the C reads disassembled (P17, P20) |
| **D5** *(changed)* | Parts, order, size | **DECIDED (d)**, as recommended — three parts: **A1** the seams (**~1¼–1⅜ d**), **A2** the standard program (**~2½–2⅞ d** with D6 (a) and its check), **B** robot two, the WHICH END removal, the proof, the robot-facing documents (**~1⅞–2⅛ d**); **~6–6½ in all** with the extra go — each part's range re-derived from its own items by the check of D6 (a), C5 (stated earlier as ~4¾–6 with D6 (a), ~4¼–5½ before it; both sat below the items). **A2 is past the ~2-day line that split Part A, at both ends: whether it splits is the team lead's** — (i) one go, as decided; (ii) A2a, the program with no typed reference (~1¾–1⅞), then A2b, D6 (a)'s typed reference and the proof through the real PROS shell (~¾–1): ~6¼–6¾ in all (§9.2; the check's "not settled" #1) | the revised Part A form leaves today's program 33 of 33 byte-identical and the suite 2,231 of 2,231 (P16; again with D6 (a): P28, P29; with the check's fixes on T4 Part 3 as landed, 33 of 33 and 2,253 of 2,253: P37); the end state's goldens equal Part A's, 69 of 69 (P18; with D6 (a) 143 of 143 over 33 scenarios, P25; with the check's fixes 185 of 185 over 43, P33) |
| **D6** *(new)* | A first record on a robot with TYPED drive ports | **DECIDED (a)** — NOT the recommended (b): with no usable record on the card (none, or none that decodes), the front check compares against the facts' typed drive ports (optional `typedDrive`, the chassis table's signed spelling); same verdicts and hold (USE NEW / RUN AGAIN), the typed ports' own words and NOTEs — `FRONT MATCHES YOUR TYPED DRIVE PORTS` only when every typed port agrees, else `FRONT MATCHES, SOME TYPED PORTS DIFFER -- see log` with each differing port on the serial log; no card, no check (and the words say there is no card); a repository robot's facts take its table's drive ports only when the boot could drive from that table (`withTypedDriveFrom`: `tableToDescription(t).valid()`, the boot's own test) — robot two's cannot yet (no track width or ratio), so it carries none, its empty card gets the FIRST RECORD note, and Station G says to push the camera end first (the check of D6 (a), C1: the draft's mapping of any SIGNED table pointed robot two's empty-card check at the wrong end); ~½ builder-day (the draft said ~¼), plus the check's ~3/16 in A2. (b), "state the gap", is the alternative not taken | through the boot's own path, the typed table drives every port exactly as the record it agrees with (P24); the typed ports change nothing when the card holds a record (`typed-card-wins` = its control, P23); robot two's 19 untyped scenarios equal the revision's (80 of the 81 files with a counterpart; `nocard`'s one `card :` line is C3's) and the side-by-side residual is 0 in all seven (P33); the mapping equals the boot's sides exactly when `valid()`, every robot and robot two's table broken one fact at a time, 12 of 12 (P34); 32 mutations — 31 killed by the 43 goldens, B11 (the glue skips the mapping, equivalent on robot two) by Part B test 4's source pin (P35); every gate on T4 Part 3 as landed (P36, P37); the goldens (185 of 185, both forms) and all 32 mutation verdicts reproduced on re-run (`v3/recheck/`) |

Consequences stated, not asked (C1–C5; C6 withdrawn): the WHICH END question leaves the library (in Part B, after
A2 never paints it; `PanelButtonId::Front` → `SaveHeld`, `Back` removed); robot two keeps slot 4, its beacon and
its gate line; only the sanitizer leaves `src/common/` (the one outside Bench Tests); HA-187 only (HA-188 released);
the save is SAFE-SAVE B1's words.

### 9.2 The parts, their sizes, and every file each touches (amended by the re-check: three parts; by D6 (a), 2026-10-08: ~½ day added — A1 +~1/16, A2 +~⅜, B +~⅛; by the check of D6 (a): ~3/16 more in A2 and every part re-derived from its own items — A1 ~1¼–1⅜, A2 ~2½–2⅞, B ~1⅞–2⅛, ~6–6½ builder-days in all with the extra go)

> **D6 (a)'s re-estimate, honestly:** the draft priced (a) at "~¼ builder-day plus tests". Built in the probe, it
> is ~½: the program and panel code (+162 and +18 lines in the probe's draft, ~⅛), the typed cases in tests 2, 6 and
> 11 and eleven golden scenarios (~⅛), eighteen mutations (A2's 28–42, B's 8–10; ~⅛ in A2, inside B's ~⅛), and the
> documents' sentences. **A2's range, ~1⅞–2⅜, now reaches past the ~2-day line D5 used to split Part A.** Kept in A2
> (the decided D5 (d)): the typed check is inside `finishDone`, which A2 writes, and its tests run on A2's rig;
> splitting it out would add a fourth go and verification (~¼) to save ~⅜. The team lead may still ask for the split.
> **SUPERSEDED by the check of D6 (a) (C5):** "~1⅞–2⅜" sat below A2's own items, and so did A1's range.
>
> **The check of D6 (a)'s re-derivation (C5) — each part's range is its own items' sum, rounded outward to the
> eighth:**
> - **A1:** ¼ + ¼ + ⅛ + ⅛ + ¼ + ¼ = 1¼, plus D6 (a)'s 1/16 = 1 5/16 → **~1¼–1⅜** (§2.A).
> - **A2:** ½ + ¼ + ¼ + ½ + ¼ + ¼–½ = 2–2¼; plus D6 (a)'s ⅜ = 2⅜–2⅝; plus the check's ~3/16 (C2–C4's code, their
>   goldens and cases, mutations 43–54, ~1/16 each) = 2 9/16–2 13/16 → **~2½–2⅞** (§2.A).
> - **B:** ¼–⅓ + ½ + ¼ + ½ + ⅓ + ⅛ = 1 23/24–2 1/24 → **~1⅞–2⅛** (§2.B; C1 changes what its D6 item holds, not
>   its ⅛).
> - **In all:** the unrounded sums plus the extra go's ~¼ are 6 1/12–6 5/12 → **~6–6½** (the parts' rounded ranges
>   add to ~5⅞–6⅝).
> - **A2 is past D5's ~2-day line at both ends**, with or without D6 (a) — for the team lead (the check's "not
>   settled" #1): (i) keep A2 as one go, D5 (d) as decided; (ii) split it — **A2a**, the program with no typed
>   reference (~1¾–1⅞: the move onto the seams, the shell, the panel's additions and held screens, the watch, the
>   facts checks, the rig, tests 1, 4–6, 8b and 9 and their mutations, C2's and C3's untyped words), then **A2b**,
>   D6 (a)'s typed reference and the proof through the real PROS shell (~¾–1: the typed facts and words, C4, the
>   golden test's 34 scenarios, the team file's example test, the chapter, guide 12, the changelog, HA-187) —
>   ~6¼–6¾ in all. By D5's own line, (ii).

| Part | Size | Starts | Code and tests | Documents |
|---|---|---|---|---|
| **A1 — the seams, a library addition** | ~1¼–1⅜ builder-days (its items, D6 (a)'s 1/16 included, sum to 1 5/16 — the check of D6 (a), C5; was "~1–1¼, inside the range") | T4 Part 3 LANDED | NEW: `include/shulib/hal/brain_screen.hpp`, `hal/fake/fake_brain_screen.hpp`, `hal/pros/brain_screen.hpp`, `calibration/calibrate_screen.hpp`, `calibration/front_check.hpp` (D6 (a): its header says `earlier` may be the typed ports; test 3's two typed-table pairs); `test/pros_shim/pros/screen.h`, `misc.h` (competition calls), `motors.h` (the three C-API reads — Re-check R6), a host `main.h`; `test/front_check_test.cpp`, `test/brain_screen_test.cpp`, `test/pros_brain_screen_adapter_test.cpp`. CHANGED: `test/pros_shim/pros/rtos.hpp` (the advance hook; behaviour unchanged with none installed); `test/CMakeLists.txt` only if a define is needed. NO `src/` file; NO existing test TU edited | `docs/changelog.md` (Added — the seams), `docs/api/` regenerated; internal: `build-order.md`, `CAL1-PROGRESS.md` (created FIRST), the briefing |
| **A2 — the standard Calibrate, a library addition** | ~2½–2⅞ builder-days (items 2–2¼ + D6 (a)'s ~⅜ + the check's ~3/16 — C5; was "~1⅞–2⅜"); **past the ~2-day split line at both ends: split or not is the team lead's (A2a ~1¾–1⅞ + A2b ~¾–1)** | A1 landed | NEW: `include/shulib/calibration/calibrate_program.hpp` (D6 (a): `TypedDrivePorts`, `CalibrateFacts::typedDrive`, `FactsList`, `hasTypedDrive`, `typedDriveDescription`, `typedDrivePortsFrom`, the reference rule and its words; the check of D6 (a): the per-file reason lines, the no-card words, the exact/partial typed NOTE and the per-port lines), `hal/pros/calibrate.hpp`; `test/calibrate_program_rig.hpp`, `test/calibrate_program_test.cpp`, `test/pros_calibrate_adapter_test.cpp`, `test/calibrate_golden_test.cpp` + its goldens (34 scenarios: fifteen + D6 (a)'s eleven + the check's eight), `test/calibrate_team_file_example_test.cpp` (+ the typed-port line's case). CHANGED (additions only): `include/shulib/calibration/calibrate_panel.hpp` (D6 (a): `FinishFacts::typedPorts`, five literals; the check: a sixth, `kNoteTypedPartial`). NO `src/` file; NO existing test TU edited | NEW guide chapter "Calibrating your robot" (+ `mkdocs.yml` nav, `docs/guide/README.md`), `docs/guide/12-when-things-go-wrong.md`, `docs/changelog.md` (Added), `docs/hardware-assumptions.md` (HA-187, the Held line), `docs/api/` regenerated; internal: `build-order.md`, `CAL1-PROGRESS.md`, the briefing |
| **B — robot two on it; the WHICH END question removed; the proof** | ~1⅞–2⅛ builder-days (its items, D6 (a)'s ~⅛ included, sum to ~2: 1 23/24–2 1/24 — C5; was "~1⅝–2⅛") | A2 landed | `src/programs/calibrate_program.cpp` (1,388 — 1,387 on main — → 27 lines, its facts `withTypedDriveFrom(kCalibrateFacts, kChassis)`), `src/programs/calibrate_program.hpp`, NEW `src/robots/calibrate_facts.hpp` (~77 lines: `withTypedDriveFrom` takes a table's typed drive ports only when `tableToDescription(t).valid()` — none for robot two today; the check of D6 (a), C1), `src/main.cpp` (three cases), `src/common/screen_text.hpp` (the sanitizer call), `src/build_selection.hpp` and `Makefile` (two messages), `tools/src_build_gate.py` (one table row; self-test case 26's Calibrate plant re-pointed at `calibrate_facts.hpp`; the docstring's :39-40 and :174 — Re-check R1), `src/robots/mechanism_table.hpp` (the dead drop deleted); `include/shulib/calibration/drivetrain_calibrator.hpp`, `calibrate_panel.hpp`, `calibrate_program.hpp` (C1's removal; `PanelButtonId::Front` → `SaveHeld`, `Back` removed); `test/drivetrain_calibrator_test.cpp`, `test/calibrate_panel_test.cpp`, `test/median_nonfinite_test.cpp`, `test/calibration_test_rig.hpp` (C1's re-pins), `test/mechanism_table_test.cpp` (the call-site pin removed), `test/calibrate_golden_test.cpp` (facts from the tables, goldens unchanged), NEW `test/calibrate_facts_test.cpp` (D6 (a): the typed ports and their parity with `tableToDescription` for every robot — as the check amended it, `valid()` tables only, and robot two's table made valid then broken one fact at a time), NEW `test/screen_text_test.cpp` (Re-check R7), the glue's source pin (it must name `withTypedDriveFrom(kCalibrateFacts, kChassis)`: mutation 11) | `docs/changelog.md` (BEHAVIOUR CHANGE), `docs/faq.md`, `docs/guide/07-getting-set-up.md`, `docs/hardware-assumptions.md` (HA-139), `README.md` (if false), `docs/api/` regenerated (Re-check R4); ~~the chapter's one robot-two sentence (D6 (a))~~ WITHDRAWN by the check (C1); internal: the worksheet's Station G (with the robot-two empty-card sentence as the check corrected it: push the camera end first, FIRST RECORD, and what to do if the 15/16 end led — §3.2), Station 6, M.0.2 and the WHICH END IS FORWARD box's sentence (Re-check R3), HANDOFF §5 and its S1/S1b line, `build-order.md`, `REMAINING-WORK.md`, `CAL1-COMPLETED.md`, the briefing; the status board (the coordinator) |

### 9.3 The team's file — VERBATIM, 21 lines (5 comments), compiled for the V5 and the host (§1.1, P6; re-linked P19; unchanged by D6 (a), re-compiled and re-linked P31)
```cpp
// Our robot's Calibrate. We state only what the robot cannot measure about itself;
// shulib Calibrate finds everything else (which ports drive, their directions, the sides,
// the FRONT -- the end you push first -- the track width and the wheel scale).
#include "main.h"
#include "shulib/hal/pros/calibrate.hpp"

using namespace shulib;

constexpr calibration::CalibrateFacts kOurRobot{
    .drive = config::DrivetrainKind::Tank,
    .cartridge = hal::MotorCartridge::Blue,  // the insert colour of the DRIVE motors
    // EVERY motor that is not a drive motor. A motor left out is a drive candidate,
    // and one the push turns gets powered (3, 6, 9 V) like a drive motor.
    .mechanisms = {{6, "left claw"}, {7, "right claw"}},
};

void initialize() { hal::pros::calibrateInitialize(); }
void disabled() { hal::pros::calibrateDisabled(); }
void competition_initialize() { hal::pros::calibrateConnected(); }
void autonomous() { hal::pros::calibrateAutonomous(); }
void opcontrol() { hal::pros::runCalibrate(kOurRobot); }
```
