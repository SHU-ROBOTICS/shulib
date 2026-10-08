# T4 — COMPLETED (Part 0 2026-09-27, Part 1a 2026-09-27, Part 2a 2026-10-01; Part 3, with Parts 1b and 2b folded in, LANDED 2026-10-08 (built 2026-10-04/06, reviewed 2026-10-07, fixed and verified 2026-10-08; host- and build-gated; NOT run on a robot)

> The driver program — the handoff's L5b, briefed 2026-09-26/27 on the team lead's request ("rather than
> putting the claw and stuff in teleop please make a driver program and also put anything driver under the
> official opcontrol as well with of course the fully finished library code running it"), and built in parts,
> each on his go. Written FROM [`T4-PROGRESS.md`](T4-PROGRESS.md) (the live log: every step, every stopped run and
> its resumption, each review round, every delta with its reason, the mutation tables, every gate line). Brief:
> [`T4-driver-program.md`](T4-driver-program.md) (its header notes — Part 1b folded into Part 3, Part 2 split into
> 2a and 2b, 2b folded into Part 3, Part 3 started early in two steps — and its 2026-09-28 re-check amendments
> govern Parts 2 and 3). Part 3's executor: Opus 5.5 (1M context), in the isolated worktree `.claude/worktrees/t4-3`
> from `7daf809`, over several runs (each STOPPED run is recorded in the log with what it left and how the next
> resumed). **Nothing was committed by the executor** beyond the early step's transport commit `6abcac8` the
> coordinator asked for; the late step's tree is an open merge of main (`fab1ebb`) into it, for the coordinator.

## What T4 delivered

| Part | Landed | What |
|---|---|---|
| 0 — the slots and the plan | 2026-09-27, `9e86a3c` (docs only) | the final slot map (R6): 1 "shulib Drive", 2 "shulib Auton Tester", 3 "Bench Tests", 4 "shulib Calibrate", 5 HELD, 6 "shulib Competition" |
| 1a — the driver file | 2026-09-27, `8d89df7` | `src/team/robot_two_driver.hpp` (which button does what, the feel, `bind()`, `run()`) and its generic half `src/common/driver_controls.hpp` (the binder BY NAME, five checks, ONE heading rule) |
| 1b — one owner | SKIPPED 2026-09-28, folded into Part 3 | — |
| 2a — the drive protections, the library half | 2026-10-01, `7daf809` | inside `teleop::runDriverControl` for every caller: a drive motor whose readings FREEZE is dropped (250 ms), the drive stops at more than two dead for the rest of the run, a fight or more than 2.4 A for 250 ms cuts both sides for 1 s; `hal::MotorGroup::markAbsent`, `FrozenReadingDetector` with guard G4, `evaluateRuntimeDegradation`; HA-175/176 |
| 2b — the program half | folded into Part 3 (2026-10-01), its late step | the PROTECTION row, the chooser's REFUSED / CUT / dead words, COMP's four-argument `perTick`, the driver file's protections paragraph (P2-18), Station K's push check |
| 3 — the new "shulib Drive"; "shulib Teleop" retired | BUILT 2026-10-04/06 (early step `6abcac8`; late step in the open merge), reviewed 2026-10-07, the findings fixed 2026-10-08; NOT committed | below |

## Part 3 — what it built (both steps; the log's "Part 3" entries)

- **"D, again" — the encoder check** (the team lead's ruling, 2026-10-01): in `teleop::FrozenReadingDetector`
  (`include/shulib/teleop/coupled_side_monitor.hpp`) a side-mate counts as evidence that the train turns ONLY while
  its ENCODER moved in the last 5 reads. A plain wall marks nobody; a hard push whose speed reports settle unevenly
  gets ONE 1 s fight cut instead (P2-19e re-pinned over the verification's whole grid); port 18's gaps (P2-21c/d)
  stay until its slipping gear is fixed. HA-175, guide 12, the monitor's WHAT IT CANNOT SEE and the changelog say
  the new rule.
- **The new "shulib Drive"** (slot 1, `make ROBOT=tank PROGRAM=drive`, beacon `tank-drive`, serial tag `DRV`):
  `src/programs/drive_program.{hpp,cpp}`, content REPLACED — the thin program around the driver file. Its boot in
  Teleop's order (the provider WITH its SD copy, `validate`, the Robot's `emplace` and `st.driver.bind()` in ONE try,
  `unbind()` first in its catch); a refusal paints the shared refusal screen (`paintRefusalScreen`,
  `DRIVE REFUSED AT BOOT`) and `opcontrol()` paints it again and idles; driver control is `st.driver.run(...)` and
  nothing else. Its pages are pure and host-tested (`src/programs/drive_panel.hpp`): **MAIN** (the PROTECTION row,
  the controller row, DETAILS), **DETAILS** (R5's four — the run status, the two source rows, the claw row, the
  fault row — then the command, the estimate, the feel, the heading cross-check and the battery) and **MORE** (the
  two groups, the travel, the protections' counters, the last cut, the peaks); a touch flips on the RELEASE edge
  after the arming delay; only the visible page is painted, and only what `driveRepaint()` marks changed, at most
  5 Hz; the controller's three lines through the ONE chooser and the line writer.
- **"shulib Teleop" RETIRED**: `src/programs/teleop_program.{hpp,cpp}` and `teleop_claws.hpp` deleted;
  `PROGRAM=library` an `$(error)` naming the retirement; `SHULIB_PROGRAM_LIBRARY` on the gate's RETIRED list
  (step 0 fails it anywhere; its beacon too); every runtime string naming Teleop or "(slot 2)" re-worded; every
  comment naming it re-worded or kept as true history. **Seven builds** (bench, xdrive, tank, tank-drive,
  tank-calibrate, tank-autontest, tank-competition); **five programs in five slots** (1 Drive, 2 Auton Tester,
  3 Bench Tests, 4 Calibrate, 6 Competition).
- **The Auton Tester to slot 2** (N4): every document COMP-t wrote; no code string names a slot.
- **Part 1b, folded:** the buttons left `src/robots/mechanism_table.hpp` (`forward`/`reverse`/`volts`/`toggle`,
  `kTeleopRezeroButton`, checks 4-7); the interim tie (`controlsMatchTable`) retired; the claw boot lines take the
  driver file's controls and find each entry's buttons BY NAME; the driver file's header says "change it HERE and
  nowhere else". The driver file is the ONLY place buttons live.
- **Part 2b, folded (the late step):** the PROTECTION row (`protectionRowText`) and the chooser's `REFUSED see
  brain` / `CUT see brain` and line 1's `dead <ports>` (`addProtectionFacts`) in the shared
  `src/common/driver_panel.hpp`, which also received the claw row, the fault row, the LCD word and the claw boot
  lines from `teleop_claws.hpp`; the competition program's driver page shows the PROTECTION row FIRST and its
  `perTick` takes the fourth argument; the driver file's PROTECTIONS paragraph, §2 verbatim, kept true by P2-18;
  cookbook 06's re-quote; worksheet Station K's protections line and **K.8 THE PUSH CHECK** (required before the
  competition program drives a match — N1).
- **N5 (a) with Decisions A and B — an SD copy of the log, events and summaries only:** `diag::LogCopySink`
  (`log()`/`summarize()` to both, `emit()`/`wantsRecord()` to the primary only) and `ProsDeviceProvider`'s copy
  constructor (`serialCopyFile`, `ProsFileMode`, a mount root for the host tests) with four read-only accessors.
  "shulib Drive" copies to `drive_log.txt` (one file per boot), the competition program to `comp_log.txt`
  (APPENDED across boots, Decision B); each says where ONCE (`src/common/log_copy_line.hpp`). **The coordinator's
  ruling of 2026-10-04: those two programs ONLY** — not the Auton Tester (COMP-t's own SD run log), not Calibrate
  (it constructs no provider), not the X-drive (it refuses at boot on every robot).
- **The team lead's C5 ruling of 2026-10-02 ("drop just that auton"), given for the competition program:**
  `competition::OnRefusal` and `AutonRegistry(periods, onRefusal = Throw)` — under `DropEntry` a refused entry
  registers NOTHING, its sentence is kept and counted, and `add()` returns, so the students' `registerAutons()` goes on
  to its next entry (the file is untouched); `AutonMenu` names each dropped entry on its status line in red, in turn;
  the competition program builds its registry with `DropEntry` and logs every dropped sentence
  (`src/common/dropped_autons.hpp`). **The Auton Tester does the same** (the same registration path) — the
  COORDINATOR's extension of the ruling (the late step's computed task: "the same for the Auton Tester if it shares
  the registration path"), not the team lead's; `REMAINING-WORK.md` §2 row 10 asked him to confirm or reject it — **confirmed by the team lead 2026-10-08: "yes auton tester can work that way too"**. The
  default is bit-identical for every other caller.
- **`main.cpp`'s final form:** the five entry points switch over the six programs (the Drive case with its own
  `autonomous()` sentence, `kDriveAutonomousReason`; no Teleop), the build table and the run-status notes.

## Deltas from the brief (Part 3), each with its reason — the log's entries carry each in full

1. **The BUILT claw line ends `-- buttons: src/team/robot_two_driver.hpp`**, not a list of programs: the brief
   allows a reword only where the words named Teleop, and this one did (`-- in shulib Teleop only`, then COMP-b's
   `-- in shulib Teleop and shulib Competition`); a list of programs went stale the day a program was added, where
   the buttons live stays true. Worksheet J.3.1 and K.2.3 re-quoted.
2. **A third page, MORE** — the brief's own fallback (DETAILS gets a second page when R5's four and the rest do not
   fit one).
3. **The page model is its own file, `src/programs/drive_panel.hpp`**, with a pure repaint plan, `driveRepaint()`:
   MAIN/DETAILS/MORE are this program's alone; what is shared (the PROTECTION row, the controller row, the chooser)
   stays in the shared header and is handed in as rows.
4. **The SD copy's provider argument is a NEW constructor** (the default one byte-for-byte unchanged), with a third
   optional argument (a mount root, so the copy is host-tested on the same open/write path) and four read-only
   accessors (so a program can say a refusing card ONCE) — additive, not in the brief's exhaustive list.
5. **Two new shared `src/common/` headers**, each because two programs would otherwise type the same words:
   `log_copy_line.hpp` (the `log copy:` boot line — "shulib Drive" and the competition program) and
   `dropped_autons.hpp` (C5's boot lines — the competition program and the Auton Tester).
6. **C5 is a LIBRARY change** (`competition::OnRefusal`, `AutonRegistry`'s defaulted constructor argument,
   `refusedCount()`/`refusal(i)`/`onRefusal()`, `kMaxAutonRefusals`; `AutonMenu`'s red status line,
   `kMenuRefusedPrefix`, `kMenuRefusalTurnSeconds`), where §5 allowed Part 3 library TEXT only: the ruling post-dates
   the brief, and the students' file must stay untouched — while `add()` throws, the first refused entry aborts
   `registerAutons()` and loses every later entry, whatever the program does around it. Additive; the default is
   bit-identical; no version bump (`competition/` is in no Freeze Register row).
7. **The Auton Tester drops a refused entry too** (the computed task's "if it shares the registration path" — it
   does: the same `registerAutons()`); a student fixing one auton must still be able to test the others. This
   CHANGES the tester's behaviour (its PICK page's status line names a dropped entry in red, its boot logs `DROPPED`
   lines) although the brief's §5 lists COMP-t's program OUT and the review's lens expected it unchanged — the
   coordinator's instruction and extension of the team lead's C5 ruling, recorded as such (the T4-3 review, finding 4),
   for the team lead to confirm.
8. **The SD copy for "shulib Drive" and the competition program ONLY** (the coordinator's ruling of 2026-10-04 on
   the early step's open question).
9. **The gate's retirement pair takes the retired cases' numbers (12-13)**, so 14+ keep theirs; the re-pointed
   plants each move to a LIVE switch (logged one-for-one in the early step's 08:58 entry).
10. **`test/source_pins.hpp` is the ONE copy of the source-pin helpers** (COMP-b's file; the early step's own copy
    moved into it).

## Tests (adversarial), and the mutations that proved them

- **The encoder check:** `coupled_side_monitor_test` P2-10b ×2 (the boundary: a mate whose encoder held 4 reads is
  still evidence, at 5 it is not; a wall whose reports settle unevenly), `drive_protection_test` P2-19d / P2-19e
  re-pinned over the verification's whole grid (nobody marked; per row exactly ONE fight cut or none; at 2.45-2.5 A
  the over-current cuts come first). Mutations EC1-EC3, all red.
- **The SD copy:** `log_copy_sink_test` LC-1..LC-6 (LC-6: 200 real `Chassis::drive()` ticks put no record line on the
  copy), `pros_device_provider_test` 1n-1q (through the shim and a scratch card), `log_copy_line_test` LCL1-LCL2.
  Mutations LC1-LC4 (LC1 the brief's named one), PV1-PV3, LLC1-LLC2, all red.
- **"shulib Drive":** `drive_panel_test` (P3-T2: MAIN / DETAILS / MORE exactly, every row's words and tone, the pager
  — the RELEASE edge, the arming delay, a press held across a flip —, `driveRepaint()`), `drive_program_source_test`
  (P3-T3: the ONE try, `bind` and `unbind` in place, `run` once, the refused path repaints and idles, nothing of the
  old program, the copy only by its bare name, the shared painters only, the controller through the ONE chooser with
  the protections' state, the cut length from the config, the RETIRED names nowhere in code or string). Mutations
  DP1-DP7 (DP1, DP2 the brief's named), MN1 (named: a leftover Teleop include — the suite AND the ARM build red),
  PG1-PG4 (PG1 named: the flip on a PRESS), DV1-DV2, LDR1-LDR3, LDP1-LDP3, all red.
- **The buttons in the driver file only:** `mechanism_table_test`'s button cases retired with their map; the claw
  boot lines read the CONTROLS (`driver_panel_claws_test` P3-T5: swapped R1/R2 print swapped; an ABSENT pivot names
  its buttons from the controls; an entry no control names prints `?`). Mutations MT1 (named), CL1-CL3 (CL2 the
  brief's named; CL3 survived, two checks added, then red), all red.
- **Part 2b:** `driver_panel_test` D6 (the PROTECTION row, every mode, words and tone, worst cases; a 0.5 s and a 2 s
  cut added when LPR2 survived), D7 (the chooser's full order, all 21 pairs), D8 (`addProtectionFacts` only SETS;
  line 1's dead list cut at 19); `competition_screen_test` S11/S12 (the PROTECTION row first); `competition_source_test`
  P19 (the fourth argument, `kCutSeconds` from the config); P2-18 (the driver file's paragraph against the config).
  Mutations LCH1 (**the brief's named "CUT ranked below a fault"**), LCH2-LCH3, LPR1-LPR6, LAF1-LAF3, LCL1, LCS1,
  LCP3, LCP5, LPP1 (**the brief's named "2.5 A"**), LPP2, all red.
- **C5:** `auton_registry_test` "C5 DropEntry" ×2, `auton_menu_test` "C5 the selector names each DROPPED entry …",
  `dropped_autons_test` DA1-DA3 (DA3 added when LDA2 survived), `autons_test` (2b) (the REAL students' file),
  `competition_source_test` P18, `auton_tester_source_test` "the tester's registry DROPS …" — each fails on the
  unchanged code (the API does not exist there; the pins name a call the unchanged programs lack: LCP1 and LAT1 put
  the unchanged form back and turn them red). Mutations LRG1-LRG4, LMN1-LMN4, LDA1-LDA3, LCP1-LCP2, LAT1-LAT2, all red;
  **LRG5 EQUIVALENT** (a slot past `refusedCount()` is never written, so it reads `""` either way).
- **Decision B:** `competition_source_test` P17 (the provider's copy, APPENDED, said once in the shared words) and
  P9's mode count. Mutation LCP4, red.
- **The gate (P3-T1):** `src_build_gate.py self-test` runs every plant for real — the RETIRED pair (case 12: the
  library switch planted in `src/`, `build_selection.hpp` and the host CMake; case 13: `make ROBOT=tank
  PROGRAM=library` stops at the `$(error)` naming the retirement), the NUL match, step 8, and every re-pointed plant
  through `_replace`'s exactly-once assertion.

## The review round (2026-10-07) and its fixes (2026-10-08) — the log's "T4-3 review findings and their disposition"

Three reviewers (correctness, tests, docs) returned 17 findings; all 17 were real (one pair the same finding), and
each was fixed at its root or, where the call is not the executor's, put to the coordinator. The ones that changed
code:
- **A slow loop wrote a line and a card flush into EVERY overrunning tick** (the major one): `LOOP_OVERRUN` per tick
  through the Robot's latch, and the scheduler's attribution line, reached the competition program's SD copy one
  write and one flush each, inside the already-late tick. Now a slow loop is one EPISODE — logged on its first slow
  tick (`diag::FaultLatch::raiseRepeat()` for the rest: counted, tallied and stamped exactly as before, no line; the
  run's first fault always said), ended by a healthy tick or `reset()` (`LoopMonitor::overrunEpisodes()`); the
  attribution line goes with it. An intermittent overrun still says each slow stretch (stated). Host test 1r runs the
  reviewer's probe through the REAL provider and Robot: one line on the card where there were 50. A BEHAVIOUR CHANGE
  in the changelog; HA-60 amended.
- **The card copy has ONE writer at a time:** `diag::LogCopySink` takes a try-lock for the copy — a line that finds
  another task mid-copy stays on serial, is counted into `serialCopyDroppedLines()`, the card's next line says how
  many it missed there (`[DIA] copy: <n> line(s) left off here ...`), and nobody waits on a card write.
  Serial's own two-task exposure (since COMP-b) is the coordinator's. **One case ends the copy** (stated and
  pinned at the verification, LC-9): a task PROS deletes mid-copy (HA-161 (a), reasoned — both driver programs
  log a line right as their loop sees the disable) never releases the guard, so the card's copy ends for the
  rest of that boot, every later line counted and on serial, with no gap line. That is the chosen failure:
  releasing it would hand the next writer, the driver loop included, whatever the deleted task left half-done
  on the card (unmeasured), where it could block.
- **A boot refused after the provider is built says why in the copy** (`logBootRefusal()`, through the provider's
  sink before it is released) — in both programs; before, "shulib Drive"'s card held an empty file.
- **The copy's boot line tells NO SD CARD from a card that would not take the file** (`logCopyOpenFrom()`, the card
  asked once).
- **"shulib Drive"'s row building and generated boot lines moved into `src/programs/drive_panel.hpp`'s pure,
  host-tested functions** (`driveSourceRows`, `DriveLiveFacts`/`driveLiveRows`, `driveLineToPaint`,
  `driveProtectionsLine`, `driveDegradedBootLine`, `driveBootLevel`) — the ten edits that survived the review now
  fail it.
- **The dropped-autons line** no longer reads as an empty menu, and names driver control only in the competition
  program.
- **Tests closed:** `refusal()`'s cap by address; the menu's turn clamp past eight drops; D6's frozen-and-warnings
  order; the 2 s turn by `static_assert`; D8's REFUSED and CUT kept.
21 new cases (19 from the stopped run of the fix round, then LCL5 — a refusal's over-long word — and LC-8 — the
copy's gap line): **57 mutations, all red** (five first red only at the build under `-Werror` were re-aimed and
observed red in the suite). The documents' half — the C5 credit for the
tester (the coordinator's extension, confirmed by the team lead 2026-10-08), HA-143's and HA-161's sources (and HA-161's stale
`src/main.cpp` citation), REMAINING-WORK §11's "below 2.5 A" corrected to the 2.4 A limit, HA-154 named as a belief
in chapter 17 and F.6.2, and the full prose review the late step had not run over `docs/` — is in the log.

**The verification round (2026-10-08)** returned four minor findings, each fixed or stated (the log's "T4-3
verification-round findings and their disposition"): the card copy's guard held by a task PROS deleted mid-line
(above — KEPT as the chosen failure, now stated in the header, the provider, both programs, HA-161, chapter 17,
F.7.2, K.7.2 and the changelog, and pinned by LC-9 with real threads); LC-8 now pins the gap line IMMEDIATELY
before the summary (one ordered list); a refusal with no provider is painted WITH its serial line in both
programs (`!logged`, read before the release); the dropped-autons line says `(the first 8 named below)` past the
cap instead of "each" (its clause under a precision, `%.23s`, and a `static_assert` on the one-digit cap — the
`src/` gate's `-Wformat-truncation` caught the first version). One new case (LC-9); **13 mutations, all red**.

## Documents (Part 3; the prose review is in the log, run and read before each entry claimed it)

README (the layout block, the worked example); guides 07, 08, 10, 12, 14, 16 (slot 2; the dropped auton), 17 (the
PROTECTION row and the chooser, the boot lines, the two SD files, C5); cookbook 06 (the re-quote, the protections
sentence, "change it HERE and nowhere else"); `faq.md`; the changelog (API 2.4: `LogCopySink`, the provider's copy
constructor, the encoder check as a BEHAVIOUR CHANGE, `OnRefusal` and the menu's red line — no version bump);
`hardware-assumptions.md` (HA-175 re-worded to the encoder check, HA-153..159's program text, HA-176's K.8, the test
file renamed); the bench worksheet (Station E a pointer, Station F REWRITTEN for the new "shulib Drive", G, J moved
to slot 1, M to slot 2, K's protections line, the dropped autons and **K.8 THE PUSH CHECK**, the slot map, the DO
NOT list); the COMP brief's B15 and B18 notes (C5); RESUMING, ORIENTATION, HANDOFF, REMAINING-WORK, build-order,
the roadmap (own lines); `docs/api` regenerated; the briefing regenerated last. **The review's fixes moved:** the
changelog (the once-per-episode BEHAVIOUR CHANGE, `raiseRepeat`, `overrunEpisodes`, the copy's one writer), guide 11
(the fault and attribution lines once per slow stretch), `diagnostics-plan.md` D-3, guides 14, 16 and 17 (the C5
credit, the card's writes, HA-154 as a belief, the refusal in the copy), the register (HA-60's second consumer,
HA-143, HA-161, the port-map row, the T4 Part 3 status sentence), the worksheet (C's DISAGREES, F.1's refusal, F.6.2,
F.7.2, K.2.2, K.7.2), REMAINING-WORK (§2 row 10 asks the team lead about the tester's drop; §10; §11's 2.4 A),
HANDOFF, build-order, the roadmap (T4's own lines and the T2 rows' "shulib Drive"); the full prose review is logged.

## Not done here — and not claimed

- **Nothing of T4 has run on a robot.** The new "shulib Drive"'s first run is worksheet Station F (F.0.1 first: an
  OLD package in slot 1 drives toward the other end), the claws' Station J, the competition program's Station K —
  and **K.8's push check is required before the competition program drives a match** (N1; HA-176). Every number
  the protections use is unmeasured (HA-132, HA-175, HA-176).
- *(Done since: the coordinator's verification and the commit, 2026-10-08 — the log's "The coordinator's merge
  verification" entry.)*
- **SAFE-SAVE B2's `BACKUP see brain`** is not in the chooser (B2 has not landed; its place, after CLAW ABSENT, is
  kept).
- **Port 18's gaps** (P2-21c/d) stay until its slipping gear is fixed, as ruled.

## For the coordinator

- **Two tasks write one log sink in the competition program** (the late step's finding, confirmed by the review; no
  HA number taken): the auton task, PROS's phase tasks and the opcontrol task's driver loop all log through the
  provider's sink. **The SD copy is now guarded** (the review's fix: `diag::LogCopySink` takes a try-lock for the
  copy, so only one task ever writes the card's `FILE*` and `BlockCharSink`'s counter; a line that finds the copy busy
  stays on serial and is counted). **SERIAL is not** — its exposure predates T4 (COMP-b): whether PROS's newlib
  serializes two tasks' writes to stdout is measured nowhere and stated in no register entry. An entry (a new HA
  number is the coordinator's) or a lock is the coordinator's call.
- **`pp_pre_t.txt` / `pp_post_t.txt`** (empty, at the repository root) arrived with main (`fab1ebb`, SAFE-SAVE
  Part A), not with T4.
- **At the merge, flip each status line to LANDED with the commit** (each says BUILT / reviewed / fixed today):
  build-order's first `Next:` (:632) and T4's status (:1490); the roadmap's T4 row (:1794) and its "You are here"
  "Now (2026-09-28)" paragraph — the WHOLE paragraph, :348-:393 (the range was :348-:362 until the verification
  round): its T4-false clauses are :349-:350 ("slot 7 until the driver program replaces Teleop"), :360-:362
  ("and so will the driver program … "shulib Teleop" retires when that lands") and :368-:375 ("shulib Teleop" still
  COASTS on release; T3's sentence, "shulib Teleop" now drives robot two's claws … the panel's two fault rows became
  one); HANDOFF :464 ("not yet merged") and :622; REMAINING-WORK §2 row 10 (:46) and the plan table (:233); this
  record's title, the Part 3 row and "Not done here"; then regenerate the briefing.
- **The team lead's confirm-or-reject of the Auton Tester's drop** (REMAINING-WORK §2 row 10) — the coordinator's
  extension of his C5 ruling: **confirmed 2026-10-08**.
- **The card copy that ends at a deletion** (the verification's V1; `diag/log_copy_sink.hpp`, "A writer that never
  comes back"; HA-161's consequence line): kept as the chosen failure. The two ways past it are the coordinator's.
  (1) A guard that recovers from a dead holder needs a NEW belief — that `task_get_state()` reads DELETED for a
  task PROS deleted, never a later task's state — and it would re-expose the next writer to whatever the deleted
  one left half-done. (2) One card-writer task that alone touches the file, fed by the others, removes the case,
  and with it the card write inside a tick (HA-60). Worksheet F.7.2 and K.7.2 now ask whether a card copy stopped
  at a disable while serial ran on — the first sighting of how often it happens.
- **REMAINING-WORK §11's answered cell was corrected** (the fight cut is under the 2.4 A over-current limit, not
  "below 2.5 A"; the ruling itself untouched) — the computed task carried the same wrong number.
- **`docs/internal/MECH-PANEL-requirements.md` :104 and :280** (another chunk's document, not edited): the
  controller screen's rows are "used by the fault display in Drive and Teleop" — Teleop is gone; "shulib Drive"'s
  and the competition program's since T4 Part 3.
- **An intermittent overrun** still logs each slow stretch (stated in `loop_monitor.hpp`, HA-60, the changelog): a
  hysteresis that merges near episodes is the coordinator's option if Station K's copy shows a loop slow every few
  ticks.

## The gates on the final tree (the review-fix round's last run, 2026-10-08 10:21–10:37; the log's final entry has every line)

- Host: `docs/api` regenerated, the briefing regenerated, the FULL build with the doc gates, the suite `2170 | 2170
  passed | 0 failed | 3 skipped`, `assertions: 2776627` (the late step's 2149 / 2776350, plus the review round's 21
  cases); the briefing regenerated again after the binary.
- `src_build_gate.py self-test` OK (211 detector cases, each against a real build); `check` PASS — all **7** builds
  link, 0 disallowed warnings, every beacon and program asserted.
- The ARM header gate over all 202 `include/shulib` headers; both CI greps (PROS-free, sim-free).
- `api_doc_tool` check-fresh / check-examples (521 quoted lines, 6 whole-run listings) / self-test / check-coverage
  (3356 entities, 165 headers) / check-removability; `doc_staleness_audit.py` clean and its self-test; the
  briefing's self-test and check.
