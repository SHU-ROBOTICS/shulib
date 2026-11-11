# CAL1 — COMPLETED (Part B built 2026-11-05, LANDED 2026-11-11; host- and build-gated, NOT run on a robot)

> The standard Calibrate, shipped with the library: a team states its drive kind, its cartridge and its mechanisms'
> ports, and gets the same, correct Calibrate; the front is the end that leads the push, with a flip check;
> SAFE-SAVE B1's safe save inside it. The team lead's rulings of 2026-10-06 and his answers of 2026-10-08 (D1–D5 as
> recommended, D6 (a)); A2 split 2026-10-08; Part B's go and his order of 2026-11-05 (the autons before Nov 21:
> Part B is what puts robot two on the library's Calibrate, whose recorded rerun is expected to fix its ~8 % distance
> error — not yet measured).
> Written FROM [`CAL1-PROGRESS.md`](CAL1-PROGRESS.md) — every part's build, review rounds, verification rounds and
> the coordinator's merge records. Brief: [`CAL1-standard-calibrate.md`](CAL1-standard-calibrate.md) (its Re-check,
> D6 (a) and "The check of D6 (a)" entries override its body). Executor model: Claude Opus 5.5 (`opus`), each part in
> its own worktree (`cal1-a1`, `cal1-a2a`, `cal1-a2b`, `cal1-b`).

## The one-sentence honest status

Robot two's "shulib Calibrate" (slot 4) IS the library's standard Calibrate — a 27-line glue over facts derived
from its own tables — and the WHICH END question is gone from the library: the end that leads the push is the
front, checked against the card's record before a safe save; every piece is proven on the host test plant and in
all seven `src` builds, and NONE of it has run on a robot in this form.

## The four parts

| Part | Landed | What |
|---|---|---|
| A1 | 2026-10-15 | the seams: `hal::IBrainScreen` + `hal::screenSafeLine()`, the painter (`calibrate_screen.hpp`), the front check (`front_check.hpp`, `compareFronts`), the host shim's screen / competition / clock-hook pieces |
| A2a | 2026-10-29 (built 10-18–22) | `calibration::CalibrateProgram` (robot two's program moved onto the seams, PROS-free), `CalibrateFacts`/`factsProblem()`, the front check's FRONT FLIPPED hold, the watch's MECHANISM PORT TURNED hold, SAFE-SAVE B1's save, the PROS shell `hal/pros/calibrate.hpp` |
| A2b | 2026-11-03 (built 10-30) | D6 (a): typed drive ports as the reference with no usable record; the program through its real PROS shell in 34 golden scenarios; the team's 21-line file compiled and run; guide chapter 19; HA-187 |
| **B** | **LANDED 2026-11-11** (built 2026-11-05) | robot two onto it (the glue, `src/robots/calibrate_facts.hpp` — N9's mapping —, `main.cpp`'s state screens, `printText()` on the library's sanitizer, the gate's row and case 26, `dropDeclaredMechanismPorts()` deleted); C1 — the WHICH END question removed from the library; the side-by-side proof; the documents |

## Part B — built

- `src/programs/calibrate_program.{hpp,cpp}` — 22 + 27 lines (was 35 + 1,397); `src/robots/calibrate_facts.hpp` NEW;
  `src/main.cpp` (Calibrate's `disabled()`/`competition_initialize()`/`autonomous()` call the library's state
  screens); `src/common/screen_text.hpp` (`printText()` = `hal::screenSafeLine()` + the three PROS calls + the
  counter); `src/build_selection.hpp` / `Makefile` (the refusal messages); `src/robots/mechanism_table.hpp`
  (the drop deleted); `tools/src_build_gate.py` (`PROGRAM_ENTRY_SYMBOLS`' Calibrate row + three names; case 26's
  plant on `calibrate_facts.hpp`).
- C1 in `calibration/drivetrain_calibrator.hpp`, `calibrate_panel.hpp`, `calibrate_program.hpp` — the changelog's
  BEHAVIOUR CHANGE (no version bump: `calibration/` is in no Freeze Register row).
- One calibrator fix the removal exposed: the yaw rate is read every tick of the push (`updateStatus()`), so DONE's
  first rest tick does not read a rate differentiated over the whole push (IMU1's T11, the root cause in the live
  log's B.2; the broader fix — no short-circuit in `atRest()` — measured and rejected: it moves every pulse's rest
  timing and ten golden records).
- One screen fix the removal exposed (the review round, finding 1): DONE now leads straight to the PULSES, whose
  ABORT filled the button row — DONE's box included — from the first frame, so a reflexive second tap aborted the
  run, against `calibrate_panel.hpp`'s own double-tap rule. The PULSES' ABORT now stays in its slot until the first
  pulse drives a motor (`viewFor()`'s SideAssign case); P3/P4/P5 pin it, every golden's screen and serial log moved
  only by that (one redraw and one `screen:` line per run, the print count +1; no card byte), classified residual
  0 (`scratch/cal1-b/fix2/classify_abort.py`).
- Tests: `test/calibrate_facts_test.cpp` (F1–F6, F5b), `test/screen_text_test.cpp` (X1–X3) NEW; re-pinned
  `calibration_test_rig.hpp`, `drivetrain_calibrator_test.cpp` (test 5 against both runs' literal ports),
  `median_nonfinite_test.cpp`, `calibrate_panel_test.cpp`, `calibrate_panel_additions_test.cpp` (N1 deleted),
  `mechanism_table_test.cpp` (the filter and the call-site pin removed), the golden rig and test (facts derived);
  `curved`'s goldens moved one tick (T).

## The evidence

Suite 2,968 cases green after the review round (2,966 at the build; + F6, F5b); `src_build_gate.py` self-test OK (269 cases) and check PASS (7 builds); the `tank-calibrate`
package's strings; ARM gate over 240 headers; every doc gate. See the live log's Part B sections B.1–B.6: the side-by-side proof (seven comparable scenarios, every differing
line classified G/F/S/W/M/T, residual 0; records identical or T-class; the timing control; a negative control), the
first measurement (all 34 goldens unchanged with the facts derived and C1 removed), the mutations, every gate's
final line, and the prose review; then "CAL1-B review findings and their disposition" (16 findings: the PULSES'
ABORT fix, `mechanismFactsAre()`'s compile-time check of robot two's mechanism facts, F5's per-function pins of
`main.cpp`, the corrected times and documents, each mutation re-run with every dependent object recompiled).

## Open, handed on

- **For the team lead (FIXED in the review round, his to overrule):** DONE leads straight to the PULSES; their
  ABORT now stays bottom right until the first pulse drives a motor, so the second tap is harmless, as the
  panel's own rule requires — the build's first pass had left it aborting the run and flagged it as a design
  choice.
- **Robot two's recorded Calibrate rerun** (worksheet Station G; copy both files off the card first; HA-187's settle
  steps G.4b.1–2) — the run expected to fix the ~8 % distance error (not yet measured). The program has never run
  on a robot.
- **SAFE-SAVE B2** (the boot's fallback to the backup; `formatSaveReport`'s `BootReadsBackup::Yes` in
  `calibration/calibrate_program.hpp`), ordered directly after CAL1 Part B; its consolidation of the two `codeOnly`
  copies and robot two's boot fixture (Re-check R8).
- **G3a Part 5** (the typed table's flip) with the track width and ratio (HA-125, HA-126) makes robot two's table a
  valid description: from then on Calibrate checks an empty card's first push against it — whoever lands it rewords
  Station G.0.4's two bullets and turns `calibrate_facts_test.cpp` F4 red by design.
- **R5a Part C** (GAINS and REST) edits `calibration/calibrate_program.hpp` and `calibrate_panel.hpp`, not `src/`.
- `src/programs/bench_r3a.cpp` :238 cites the old program's header (another chunk's file in flight).
