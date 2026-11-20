# S13UX — progress log

Brief: [`S13UX-station-13-made-plain.md`](S13UX-station-13-made-plain.md). Executor: opus (subagent), worktree
`.claude/worktrees/s13ux` (branch `s13ux`, from `ee5a1f5`). Host only; nothing committed by the executor.

## S13UX-A

- 2026-11-19 — log created first; reading station 13 (`src/programs/bench_r3a.{hpp,cpp}`) and its tests.
- Baseline (ee5a1f5, built in this worktree): suite 3,855 cases, 0 failed, 3 skipped (`suite_run.py show`).
- Read: station 13's pure parts (`bench_r3a.hpp` `mech_station`), its live page (`bench_r3a.cpp` `mech13::`,
  `mechanismsStation`), the robot-one log of 2026-11-17 (the 0x0f state screen; L1 never held while UP and R1 were
  pressed; the 3 s cut followed by "leave and re-enter" REFUSED lines), the service's current cap
  (`mechanism/service.hpp` `bootCurrentCap`).
- **D5 is reachable from `src/` with no `include/` change**: `robot::MechanismParts::devices()[i].currentLimits[m]`
  (MECH-S-A) is each motor's `hal::IMotorCurrentLimit`, and the spec carries `currentCapAmps`. Built.
- Code (header): `Pad::field`; `CutKind`; `doNowText` (D2, replaces `gatesText`); `limitText` (the status line:
  volts + cap); the lift guide (`guideSteps`, `guideStep`, `guideLines`, `stepName`), `recorded()` /
  `recordShown()` / `recordText()` (D3); the D4 re-arm in `tick()`; `applyCap` (D5) at arming and every jog/check;
  `beginJog`/`endJog` (D2's "did not move", judged only for a jog of 0.4 s or more); plain `motorText` (no
  radians), `unitsText` (output deg first, RECORD's digits), `switchText` (no HA number), the confirm's foot line.
- Code (cpp): the D2 line under the title, readings below it, the status line = `limitText` (or RECORD's note);
  the title shows RECORD's big line; events go to serial/SD only; `pad.field` from `competition_get_status()`'s
  CONNECTED bit; the intro and boot lines on the panel in plain words, the engineering lines log-only
  (`mech13::logOnly`); `st.recorded(records)` after RECORD.
- Tests: 9 new cases (S13UX D2 ×2, D2/D4, D4, D3 ×3, D5, the verdict case split out of the old gates-line case);
  existing expectations moved to the plain words. Two adversarial width checks went RED on the first build and
  fixed the code: "Pair the controller to the brain (it shows no link)" was 51 columns, and RECORD's big line for
  a power mechanism cut its own name ("right togg") — now "…: it has no link" and "REC 4: right toggle in the log".
- Docs moved with the code: guide 21 (step 3 of "The order on the robot"; the toggles' row: station 13 now writes
  the cap; "What robot one does not do yet": the uncapped-station bullet became the capped rule), guide 18 ("The
  bench page": the what-to-do line, plain readings, the cap, the D4 cut rule, the lift's guide, RECORD's big line),
  the robot facts form (C7's top stop), `hardware-assumptions.md` (HA-255's containment; HA-184's page wording),
  `roadmap.md` (ROB1's "Next with it"), `changelog.md` (an API 2.9 entry, no API change), the bench worksheet
  (Station P's note, P.2's table, P.3's cut rule, P.4; R1.2 and R1.4). Two stale code comments ("station 13 binds
  none, so there it is uncapped"): `src/robots/mechanism_table.hpp` (robot one's region; the twin test compares
  code with comments stripped) and `src/robots/robot_one_facts.hpp` (kSideToggleCapAmps) — FLAGGED for the
  coordinator (neither is listed as another chunk's).
- Suite after the tests: `suite_run.py green` — 3,864 cases (3,855 − the old gates-line case + its verdict half +
  9 new), 0 failed.
- Gates so far: src build gate `check` PASS (14 builds; 0 disallowed warnings); ARM header gate CLEAN (259); CI
  greps clean; api_doc_tool self-test / check-coverage / check-fresh / check-examples / check-removability OK;
  briefing_status self-test + check OK; doc_staleness_audit self-test + run clean.

### Deltas from the brief (each with its reason)

1. **D2's field line** is `Controller says FIELD: unplug the cable or re-pair`, not the brief's 58-column example —
   the 50-column budget. It is read from `competition_get_status()`'s CONNECTED bit (the brain's own status; the
   link to the controller is `isConnected()`). What the code can tell: robot one's 0x0f was a DISABLED state, in
   which PROS ends `opcontrol()` and the state screen (`src/common/state_screens.cpp`, not this chunk's) is what
   shows — station 13 is not running then. While it runs, a field/switch link is said, ahead of "hold L1".
2. **No-link line** `Pair the controller to the brain: it has no link` — the brief's wording was 51 columns (the
   width test went red on it).
3. **D3's big line** is `REC 2: cascade output +3759.04 deg`: RECORD's own tag and the serial line's own digits
   (`%+.2f`, captured inside `record()` for the shown mechanism, so it is the SAME number), in the title slot,
   MEDIUM — the largest font on the live page (LARGE holds 18 columns: no name and number fit). The name is cut
   first, never the number.
4. **The lift's L1-held line** is `UP raises, DOWN lowers; A checks its way (L1 held)` (the brief's example named
   the mechanism): the name is in the title, and the check (guide 21's step 2) belongs on the same line.
5. **Layout**: the "what to do now" line sits directly under the title (it replaced the gates line); the status
   line is a reading (`3 V now (R1 for more); cap 2.0 A`) or RECORD's note, never the latest log line (D1).
   The intro's engineering lines and each mechanism's boot lines go to the log only; the panel shows one plain
   line per mechanism (`1. 'cascade' found` / `NOT FOUND: check its cable`); a refused role still marks the
   menu's dot CHECK, as before.
6. **"Did not move"** is judged only for a jog of 0.4 s (kCheckJogMs) or more — a tap is not a failure — and is
   cleared by R1 (the ceiling it was judged at changed), NEXT, or the next jog.
7. **D5 is applied at the mechanism's arming (stage-2 YES) and at every jog/check start**, written and READ BACK
   in whole mA as the service does; nothing is written before the first confirm; a motor that lost power (HA-220)
   is re-written at the next jog. A cap is a limit, never a gate (the service's rule): a motor that will not take
   it is said (log + a red `NOT SET` status line) and still runs. The cap is believed to stay in the motor after
   the station (HA-220, unverified; every program writes the same declared number anyway).
8. **A new log line** `[R3A] mech guide: REC n = '<name>' step k of N, <step>` beside each RECORD on a lift, so the
   coordinator knows which RECORD was which step; the RECORD lines themselves are byte-for-byte unchanged (D6).
   The guide starts at step 1 at each entry (a Station is new per entry).
9. **Readings** drop the motor speed (rad/s — D1's radians) and the `(power-up = home …; 5.00:1)` wording; a lift's
   line leads with RECORD's `output … deg`. The confirm's foot line no longer says "INVENTED (HA-182)"; the switch
   line no longer says "(if HA-184)" (both stay in the log / RECORD).
10. **Two files outside the brief's list** had a now-false comment ("station 13 binds none, so it is uncapped"):
    `src/robots/mechanism_table.hpp` (robot one's region) and `src/robots/robot_one_facts.hpp` — comment-only.

### Tests added (`test/bench_mech_station_test.cpp`) and the mutants that turn them RED

Each mutant: applied by `.claude/coordinator/scratch/s13ux/mutate.py` (restores in `finally` and on
SIGTERM/SIGHUP/SIGINT, bumps the restored file's timestamp, touches the test file so every dependent object is
rebuilt), a FRESH binary built (`build.make` direct), the station 13 filter run under `ulimit -v 4194304` +
`timeout 600`. All restored (checksums matched before and after).

| Mutant | What it breaks | Seen |
|---|---|---|
| M1 | D4: a FIGHT classified as a jog-time cut | RED — "S13UX D2/D4: each cut…", "S13UX D4: a jog-time cut RE-ARMS…" |
| M1b | D4: the let-go marking and the re-arm ignore the cut's kind | RED — "S13UX D4…" (5 assertions) |
| M2 | D4: the re-arm without L1 let go first | RED — "S13UX D4…" (6) |
| M2b | D4: a controller DROP counted as a let-go | RED — "S13UX D4…" (3) |
| M3 | D2: the field-link line dropped | RED — "S13UX D2: ONE 'what to do now' line…" (2) |
| M3b | D2: bench_r3a.cpp never reads the field link | RED — "pin (S13UX)…" (2) |
| M4 | D3: the big line reads the inches, not the output degrees | RED — "S13UX D3: RECORD's big line…" (2) |
| M4b | D3: the big line rounds differently from the serial line | RED — "S13UX D3: RECORD's big line…" (2) |
| M5 | D5: the cap never written | RED — "S13UX D5…" (7) |
| M6 | D2: a short tap judged "did not move" | RED — "S13UX D2: a jog that did NOT move…" (1) |
| M7 | D3: the guide never moves on | RED — both "S13UX D3" guide cases (11) |

New cases: S13UX D2 (every state's line, each ≤ 50 columns), S13UX D2 (did not move: 0.4 s rule, R1 clears,
"check its cables" at the most), S13UX D2/D4 (each cut kind's line), S13UX D4 (re-arm: L1 held throughout, a
drop, release-then-hold, nothing resumes, a fresh 3 s; a fight and an over-current never re-arm; re-entry still
clears a fault), S13UX D3 ×3 (robot one's cascade with no levels; a lift with six levels and a 15-character
level; the big line equal to the serial line's number, the name cut first, cleared by power or NEXT, other kinds'
words), S13UX D5 (robot one's table: nothing written before the confirm, 2.0 A / 1.5 A after, a lost cap
re-written at the next jog, the absent right toggle never written, a refused cap said and still running, a
mechanism with no cap never written), pin (S13UX) (bench_r3a.cpp: the field read before the tick, the D2 line and
the big line painted, `recorded()` after the RECORD lines, events never on the status line). The old gates-line
case became the verdict case.

### Prose review (run, hits read, BEFORE this entry)

1. `grep -rn -i -E 'station 13|13 MECHANISMS' docs/ README.md` (outside `docs/internal`): fixed guide 21 (step 3, the
   toggles' row, "What robot one does not do yet"), guide 18 ("The bench page"), the facts form (C7), HA-255's
   containment, roadmap (ROB1's "Next with it"), changelog (new entry). Read and still true, unchanged:
   `docs/README.md`:21 (the first-week picture), guide 22 :78/:80/:302 (levels recorded at station 13), guide 10
   :856/:872 and guide 14 :251 (the station exists, not run on a robot), `README.md`:296, `docs/api/direction_check.md`
   :431 and `docs/api/mechanism_parts.md`:296 (generated from include/, untouched), the HA rows and entries that name
   station 13 as where a belief is settled.
2. `grep -rn -E 'bench_r3a\.(hpp|cpp)' docs/ README.md`: HA-182 (the constants — unchanged), HA-184 (its page
   wording — FIXED, :1628 and :1637), HA-186 (the RECORD line — unchanged), `where-your-code-goes.md`,
   `api/brain_screen.md`, `api/pros-device_registry.md` (other parts of the file) — no change.
3. `grep -rn -E 'gatesText|gates line|ceiling 3V|m1 p6|power-up = home|\(if HA-184\)|leave and re-enter|UNCAPPED|
   uncapped|does not apply the (cascade.s )?cap|CONTROLLER NOT CONNECTED|binds none' docs/ README.md`: live hits
   fixed (HA-184 ×2; the worksheet's P.2, P.3, P.4.1, P.4.3, P.7.1, R1.2, R1.4 and Station P's note); history
   records (MECH-PROGRESS, ROB1-*, MECH-S-*, other chunks' logs) left as written — records are indexed, not rewritten.
4. `grep -n -i -E 'never press R1|R1 to raise|at 3 V|uncapped|ceiling' docs/guide/*.md docs/robot-facts-form.md
   docs/README.md README.md`: clean (only unrelated "ceiling"s and the facts form's C6 (ii) 3 V press — true).
5. `grep -rn 'station 13 binds none' src/ include/ docs/ README.md`: the two src comments fixed; none left outside
   history records.

### Final gates (2026-11-19, the first hand-over — CORRECTED at the review round: the real order and times)

- Host: the suite binary linked 16:25 → `suite_run.py green`: **3,864 cases, 3,886,719 assertions, green**; the
  build's doc gate stamped 16:28 — BEFORE guide 21's last edit (16:39), so that gate never ran on the final guide
  21 (the reviewer re-ran it on a copy of the final tree: clean). The src gate `check` ran 16:23 and 17:29, its
  `self-test` 17:15. Superseded by the review round's final gates (below).
- `src_build_gate.py self-test` (under `timeout -s INT`): **OK (398 detector cases, each against a real build)** —
  it ran in its copy; no plant left (`git diff -- Makefile src/` has no "(self-test plant"; no untracked file in
  src/include/test).
- `src_build_gate.py check`: **PASS** (14 builds; 6 refuse naming exactly their owed facts and link in the typed copy).
- ARM header gate: CLEAN (259 headers). CI greps: PROS-free, sim-free.
- api_doc_tool self-test / check-coverage (6,485 entities) / check-fresh / check-examples / check-removability: OK;
  briefing_status self-test + check: OK; doc_staleness_audit self-test + run: clean.
- Not done here (host only): no `pros`, no upload, nothing run on a robot.

### S13UX-A review findings and their disposition (2026-11-19, ~17:55–19:25)

Three reviewers (correctness, tests, docs); 22 findings, each re-checked against the code before it was acted on.
All real; none rejected. Code: `bench_r3a.hpp` (`doNowText`, `limitText`, `recorded`, `resumeGuide`, the
still-moving note) and `bench_r3a.cpp` (the line's colour; a lift's guide kept in `mech13::Built`).

| # | Finding | Disposition |
|---|---|---|
| 1 | (major) "did not move — press R1" ignored the cap, a stop and the kind | FIXED. R1 is offered only when the mechanism states a cap that HOLDS and is not a power one, worded for a stop: `Did not move at 3 V. Not at a stop? R1 for 6 V`; the cap NOT SET: `Did not move at 3 V, cap NOT SET: check cables`; otherwise `Did not move at 3 V: at a stop, or check cables`. The NOT SET status line no longer offers R1: `3 V now; cap 2.0 A NOT SET: keep 3 V` (guide 21's rule, on the panel) |
| 2 | (minor) the FIELD line outranked every state though the station works with it on | FIXED. Said only when nothing else needs saying (below the cut, the prompts, L1, a jog, the outcomes), worded `Controller says FIELD: unplug the field cable` ("re-pair" dropped — that cured 11-17's DISABLED state, which never reaches the station); amber, no longer red; the test's comment corrected |
| 3 | (minor) a check cut mid-jog that still asks said "DONE" | FIXED. The question outranks the cut |
| 4 | (minor) two silent dead-ends: a check abandoned as still moving; an arrow held after the re-arm | FIXED. `It was still moving: wait, then press A again` (until the next jog, check or NEXT); `Let go of UP, then press it again` (any arrow held from before: a re-arm, a drop, through a confirm) |
| 5 | (minor) RECORD on an ABSENT lift moved its guide on | FIXED. The guide (and its log line) moves on only for a RECORD that read the built lift |
| 6 | (minor) a flywheel's big line rounded differently from its serial line | FIXED. `%+.4f rpm`, the serial line's digits; guide 18 says what the big line shows (a lift's or an arm's `output … deg`, a flywheel's rpm) |
| 7 | (major) the pin checked computing, not painting (N3, N4, N5, N12, N15, N25 survived) | FIXED. The pin now requires the place, both paints, the status paint, the `if (st.recordShown())` adjacency for the title and the status, the log-only declaration line, the refused role's `setVerdict(Sev::Warn)`; all six RED |
| 8 | (major) D5's read-back and the check's cap re-write untested (N10, N8) | FIXED. `currentLimitTakesInstead` → NOT SET; a lost cap re-written at the check's start; both RED |
| 9 | (major) the 3 s cut's "did not move" untested (N11) | FIXED. Robot one's own case (a 3 s jog that never moved, the re-arm, the advice); RED |
| 10 | (minor) N1, N7, N24 | FIXED. N7 (NEXT round trip) and N24 (one of the cascade's two motors turning) RED. N1's code is gone: a power mechanism is never offered R1 now (F1c below) |
| 11 | (minor) kMaxPanelLines' guide term unguarded; a comment called kTall the widest lift | FIXED. `kWide` (4 motors, an angle sensor, a home switch, 6 levels): 19 lines, the last the switch's RECORD question; N6 RED; the comment fixed |
| 12 | (minor) N2, N9, N18 | FIXED. Robot x's arm has no guide (N2), a fire ends the big line (N9), the claw's status line names its SAFE position while fired (N18); all RED |
| 13 | (minor) PROGRESS's final-gates order and times wrong; the doc gate stale for guide 21 | FIXED. The entry corrected (above); the build's doc gate re-run on the final tree (final gates below) |
| 14 | (minor) the briefing's count is path-dependent | NOTED for the coordinator (below) |
| 15 | (major) the top stop's likely STALL cut resets the guide; no doc says RECORD first | FIXED in docs AND code: guide 21 step 3, the facts form C7, the worksheet's R1.4 and P.3.5: `It STALLED` at a stop IS the stop — RECORD first, then DONE and open 13 again. A lift's stall line says it: `It STALLED: at a stop? RECORD, then DONE, open 13`. Its guide is kept across entries while Bench Tests runs (`mech13::Built::guide`, handed back by `resumeGuide` after the CLEAR, kept at the exit): every RECORD reads the same motor counts, so the next entry's RECORD is labelled its real step |
| 16 | (major) HA-182 still said the page says "invented" on its screens | FIXED. "the log says so (its screens did until S13UX, 2026-11-19)" |
| 17 | (minor) prose review logged "never run on a robot" lines as true | FIXED. Station 13 ran on robot one 2026-11-17 (139 `mech REC` lines in its log): guide 18's bench page, guide 10 (×2), guide 14 and README.md now say so. The earlier verdict (item 1 of the prose review above) was wrong |
| 18 | (minor) guide 21 "no soft stops before Stage B" | FIXED. "(it has no soft stops: nothing is homed there)" |
| 19 | (minor) "R1 is safe" rests on HA-219, unmeasured | FIXED. Guide 21 and the changelog: "The motor holds its cap (HA-219), so R1 is safe…"; delta 7 cites HA-220 |
| 20 | (minor) switchText's comment kept "(if HA-184)" | FIXED |
| 21 | (minor) §1's failure 1 not raised as open | NOTED for the coordinator (below) |
| 22 | (minor) roadmap: stale "Next with it", S13UX out of date order | FIXED. "robot two's station 13 answers … — robot one's were typed 2026-11-17"; S13UX after ROB1-BT |

**Mutants, this round** (`.claude/coordinator/scratch/s13ux/mutate2.py`: applied, every dependent test TU touched,
a FRESH binary built, the station 13 filter under 4 GiB + `timeout 600`, restored in `finally` and on
SIGTERM/SIGHUP/SIGINT with its timestamp bumped, checksums matched after): the reviewers' N2, N3, N4, N5, N6, N7,
N8, N9, N10, N11, N12, N15, N18, N24, N25 — all RED. This round's own: F1a (R1 with no cap stated — SURVIVED the
first run: every capless fixture in the test was a power one; robot x's arm and lift added, then RED), F1b (R1
with the cap NOT SET), F1c (R1 to a power mechanism), F1d (the NOT SET status line offering R1), F2a/F2b (the
field line over "hold L1" / over a cut), F2c (the field line dropped), F3 (a cut over the question), F3b (a
lift's stall says only DONE), F4a–F4d (the still-moving note never set / not cleared by a jog / by NEXT; the
held arrow unsaid), F5 (an ABSENT lift moves its guide), F6 (a flywheel's big line in whole rpm), F7a (the
resumed step unclamped), F7b/F7c (the cpp never hands back / keeps the guide) — all RED. The first round's M1–M7
re-run on the final tree (M4b's anchor moved to `%+.4f rpm`): all eleven RED.

**For the coordinator:**
- §1's failure 1 (the 0x0f FIELD CONNECTED controller, 2026-11-17) is NOT changed by S13UX: it was a DISABLED
  state, in which `opcontrol()` has ended and the boot's state screen shows (`src/main.cpp`,
  `src/common/state_screens.cpp` — outside this chunk). Station 13's FIELD line covers only a field link while
  enabled. The state screen says to unplug a cable; on 11-17 re-pairing fixed it, and the worksheet's DO NOT
  (HA-128: a start from the controller's own Run menu arms a pretend match) is a possible cause its words do not
  name — unconfirmed.
- The briefing's assertion count depends on the source path's length (ee5a1f5): after the merge, regenerate it
  in the main tree (`rm -f build/test/shulib_tests`, build, `briefing_status.py generate`), as ee5a1f5 did.
- `src/robots/mechanism_table.hpp` and `src/robots/robot_one_facts.hpp` carry a comment-only change (delta 10).

**Prose review, this round (run, hits read, BEFORE the final gates):**
1. `grep -rn -E 'press R1 for|re-pair|check its cables|unplug the cable|R1 for more\); cap 2.0 A NOT SET|DONE, then
   open 13 again' docs/ README.md` (history records aside): the one hit, the worksheet's P.2 row, quotes the
   FIGHT line (still true); every old line already replaced (guide 18, the changelog, the worksheet's P.2).
2. `grep -rn -i -E 'run on a robot|run on the robot|on a robot yet|its first run' docs/*.md docs/guide/ README.md`
   (station 13's lines) and `… 'not (yet )?run|never run' docs/hardware-assumptions.md`: guide 18, guide 10 ×2, guide
   14, README.md, HA-184's and HA-186's *Settle* fixed (it ran on robot one 2026-11-17, which declares no switch or
   rotation sensor); the changelog's 2026-10-30 entry and the roadmap's A6 landing record (2026-11-06) are dated
   records, left as written.
3. `grep -rn -E 'says so on its screens|soft stops before Stage B|\(if HA-184\)' docs/ README.md src/`: clean
   (HA-182, guide 21 and switchText's comment fixed).
4. `grep -rn -i 'R1 is safe' docs/ README.md`: guide 21 and the changelog, each now citing HA-219.
5. `grep -rn 'switched off\|brain is off' docs/`: the first wording of the guide's carry-over ("until the brain is
   switched off") was wrong — the steps live in Bench Tests' memory, so they last while Bench Tests runs; fixed in
   guide 21, guide 18, the facts form, the changelog, the worksheet and the roadmap.

### Final gates, the review round (2026-11-19, the tree as handed over — times from the files)

- Host: `rm -f build/test/shulib_tests` → briefing generated → `cmake -S test -B build/test` → `cmake --build
  build/test -j8` (18:16; its doc gate passed, the suite line not compared without a binary) → `suite_run.py green`
  (18:17): **3,866 cases, 3,886,991 assertions, green** (3,864 + the still-moving case + the guide carry-over case)
  → briefing generated again → `cmake --build` again (18:20): the doc gate passed WITH the binary (briefing current,
  staleness clean, check-nav / check-tone / first_week / build_to_program / check-register OK). Every public doc
  edit of this round predates that 18:20 stamp.
- `src_build_gate.py self-test` under `timeout -s INT` (18:20–19:09): **OK (398 detector cases, each against a real
  build)**, in its copy; `git diff -- Makefile src/` has no "(self-test plant"; no untracked file in src/include/test.
- `src_build_gate.py check` (19:09–19:23): **PASS** (14 builds; 6 refuse naming exactly their owed facts and link in
  the typed copy; 0 disallowed warnings).
- ARM header gate: CLEAN (259 headers). CI greps: PROS-free, sim-free.
- api_doc_tool self-test / check-coverage (6,485 entities) / check-fresh / check-examples / check-removability: OK;
  briefing_status self-test + check: OK; doc_staleness_audit self-test + run: clean.
- The briefing here reads 3,866 / 3,886,991 at THIS path; regenerate it in the main tree after the merge (above).
- Not done here (host only): no `pros`, no upload, nothing run on a robot.

### Verification-round findings and their disposition (2026-11-19, ~19:30–)

A fresh verifier returned three findings (all minor; safety unaffected). Each was re-checked against the code
before it was acted on; all three are real; none rejected.

| # | Finding | Re-check | Disposition |
|---|---|---|---|
| V1 | DOWN on a pneumatic says `Let go of DOWN, then press it again` — a dead end; robot one's (and two's) ramp line `UP fires it to 'down'` invites DOWN | Real: `doNowText`'s held-arrow branch had no kind test, and `tick()` fires a pneumatic only on `plusPress \|\| checkPress` | FIXED. A pneumatic with DOWN held (UP not) says `DOWN does nothing here: UP fires it` (35 columns); UP held still says `Let go of UP, …`; a motor mechanism's DOWN line is unchanged. Guide 21 step 3: fire each piston with **UP** (the ramp too) |
| V2 | After a RECORD at a stall the line still says RECORD; one extra RECORD mislabels every later step for the whole run (the carry-over removed the only reset) | Real: the stall line never changed after RECORD; `recorded()` moved the step on for EVERY RECORD that read the lift; robot one's 11-17 log has two RECORDs with nothing between (REC 1 and REC 2 at `pos +0.000 rad`, the second entry) | FIXED at the root, in three parts. (a) A lift RECORDed while cut says `RECORDed: now DONE, then open 13 again` (`Mech::recordedCut`, set by a RECORD that read it while cut, cleared by every new cut). (b) **A step moves on only for a RECORD after a jog**: `GuideMark {step, jogged}` — `drive()` marks it jogged; a RECORD with nothing jogged since the last step's RECORD is THAT step again, logged `[R3A] mech guide: REC n = '<name>' step k of N, <step> -- AGAIN (not jogged since)`, and moves nothing. The whole mark carries over (`mech13::Built::guide` is a `GuideMark`; `guideMark()` kept at the exit, `resumeGuide(i, mark)` after the CLEAR), so jogged-then-left-then-RECORDed is a new step and a re-entry's first RECORD with nothing jogged is not. (c) Docs: to start the steps over, stop the program and run Bench Tests again (guide 21, guide 18, the facts form's C7, the changelog, the worksheet's P.2 table, P.3.5 and R1.4) |
| V3 | `Did not move at 9.125 V: at a stop, or check cables` is 51 columns (a power mechanism's `%.4g` maxVolts) | Real: `%.4g` prints up to 9 characters over (0, 12] V (`0.0001234`) | FIXED. Volts on the panel print bare when whole, else to 0.1 V — the log's own `%.1f` (`decimals()`), at most 4 characters for every legal value: the three did-not-move lines AND the status line (`9.1 V now (the most); cap 2.5 A`), so the page agrees with itself. The widest is now 50 (`Did not move at 12.0 V: at a stop, or check cables`) |

**Deltas this round:**
11. **V2's "again" rule is the jog, not the position.** A RECORD moves the guide on only if the lift was powered
    (a jog or a check) since the last step's RECORD. Every step after the first is reached by a jog, so this is
    exactly guide 21's order; a position rule would stick on a level declared AT a stop and needs a threshold.
    A level declared at a stop is reached by jogging into it (it does not move; the RECORD is then a new step).
12. **The guide's carry-over holds the whole mark** (`GuideMark`, not the step alone): without `jogged`, a lift
    jogged in one entry and RECORDed in the next (a fault cut between) would be called "again".
13. **The status line's volts follow V3's format too** (`9.1 V now`, not `9.125 V now`), so one page never shows
    two spellings of one number; whole volts unchanged (`3 V now (R1 for more); cap 2.0 A`). The confirm's own
    foot line (MECH-A6's `%.4g V now, its most %.4g V`) is a different screen within its own budget — unchanged.

**Tests (`test/bench_mech_station_test.cpp`):** two new cases — "S13UX D2: DOWN on a pneumatic …" (robot one's
ramp: its idle line, three fresh DOWN presses fire nothing and each says so, the cascade's DOWN line unchanged, UP
still asks) and "S13UX D2: the volts fit the panel …" (a capped power mechanism at 9.125 V, 0.0001234 V and 11.96 V:
the did-not-move and status lines, the cap holding and NOT SET, each ≤ 50 columns). Extended: the ONE-line case
(the claw with DOWN held, with both held — inside its width loop); the cut case (a lift's line after its RECORD,
through a NEXT round; robot x's arm unchanged; robot one's 3 s cut RECORDed, re-armed, then STALLED: the new cut's
line asks for RECORD again); the D3 cases (RECORDs with nothing jogged between are AGAIN — exact log lines; arming
is not a jog; a jog moves it on; AGAIN at the top stop; the carry-over of `{step, jogged}` both ways; the levels
case jogs between RECORDs); the pin (`b.guide[i] = st.guideMark(i);`).

**Mutants, this round** (`.claude/coordinator/scratch/s13ux/vr/mutate3.py`: applied, every dependent test TU
touched, a FRESH binary built, the station 13 filter under 4 GiB + `timeout 600`, restored in `finally` and on
SIGTERM/SIGHUP/SIGINT with its timestamp bumped; the three files' checksums matched after; no build broke) — **27 of
27 RED**: P1 (the pneumatic DOWN line removed), P2 (without the kind test), P3 (said with UP held too); S1 (a RECORD
at a cut never noted), S2 (a new cut keeps an earlier RECORD), S3 (the RECORDed line for a non-lift), S4 (a lift's
stall says only DONE — F3b re-cut); G1 (a RECORD with nothing jogged since moves on), G2 (a jog never marks the
guide), G3 (`resumeGuide` drops `jogged`), G4 (an AGAIN still moves on), G5 (a new step keeps `jogged`), G6 (an
ABSENT lift's RECORD moves it on — F5 re-cut), G7 (`resumeGuide` unclamped — F7a re-cut), G8 (the guide never moves
on — M7 re-cut), G9 (the cpp keeps only the step), G10 (the cpp never keeps the mark — F7c re-cut), G11 (the cpp
never hands it back — F7b re-run); W1 (non-whole volts to 3 decimals: `9.125 V`, 51 columns — the width check AND
the exact lines RED), W2 (the same in the did-not-move lines only); re-runs on the code beside this round's: F1a,
F1b, F1c, F1d, F4d, M1, M2.

**Prose review, this round (run, hits read, BEFORE the final gates):**
1. `grep -rn -E 'each RECORD (moving|moves)|moved on by each RECORD|each RECORD mov' docs/ README.md`: one live
   hit, the worksheet's P.2 table ("each RECORD moves it on") — FIXED (after a jog; AGAIN otherwise).
2. `grep -rn -E 'Let go of DOWN|DOWN does nothing' docs/ README.md`: the worksheet's P.2 line-2 list — the DOWN
   line added beside `Let go of UP, …`; guide 21 step 3 now says to fire each piston with UP (the ramp too).
3. `grep -rn -E 'RECORDed: now DONE|stop the program and run Bench Tests|start (the steps|them) over|nothing
   jogged' docs/ README.md`: guide 21, guide 18, the facts form, the changelog, the worksheet (P.2, P.3.5, R1.4) —
   each read: the same rule in each, "stop the program" (the house phrase, guides 11 and 18).
4. `grep -rn -E 'carr(y|ies) over|keeps your place|while Bench Tests runs' docs/*.md docs/guide/ README.md
   <worksheet>` (the guide's steps): every hit now pairs the carry-over with the AGAIN rule or the way to start
   over, except the roadmap's one-line summary (:580, "kept while Bench Tests runs" — still true).
5. `grep -n -i -E 'each RECORD|power cycle|guideStep|%\.4g' src/programs/bench_r3a.{hpp,cpp}
   test/bench_mech_station_test.cpp` (code comments): the pin's comment ("kept for the power cycle") FIXED; the
   other "power cycle" comments are about the parts (true); `%.4g` remains only in MECH-A6's confirm foot line
   (delta 13).

**Resumed, 2026-11-19 ~20:00.** This round's first run stopped at ~19:55, during its src gate self-test. Nothing of
s13ux's was still running, and the next self-test cleared the interrupted copy. Before any gate, each finding was
checked again against the code as it is now, not taken from the table above:
- V1: the pneumatic DOWN branch sits above the held-arrow line, and `tick()` still fires a pneumatic on UP or A only.
- V2: `recordedCut` is set only by a RECORD that read the lift while it was cut, and `cutMech` clears it. The AGAIN
  rule is in `recorded()`, and `drive()` marks `jogged`. `mech13::built()` is a function-local static, so "stop the
  program and run Bench Tests again" really does start the steps over.
- V3: `decimals()` prints whole volts bare, others to 0.1 V. The R1 line's `more` is always whole (3/6/9/12 V).
Each fix has a test: the ramp's three DOWN presses; the RECORDed line; AGAIN at both stops and across entries; the
9.125 / 0.0001234 / 11.96 V widths. The three code and test files' checksums equal the mutation-tested tree's
(`vr/premut.md5`), so `mutate3.py`'s 27 of 27 RED stand for this tree. No mutation ran during the src gate (its
`check` builds `src/`). The prose review's five greps were run again and their hits read: unchanged from the list
above. MECH-A6's confirm foot line (`%.4g`, delta 13) is 46 columns at 9.125 V and fits for any maxVolts of
0.01 V or more.

### Final gates, the verification round (2026-11-19, the tree as handed over: times from the logs in `vr/f_*`)

- Host: `rm -f build/test/shulib_tests`, then the briefing generated, `cmake -S test -B build/test`, `cmake --build
  build/test -j8` (20:04), and `suite_run.py green` under 4 GiB + `timeout 600`: **3,868 cases, 3,887,073
  assertions, green** (3,866 + the two new cases). Then the briefing generated again and `cmake --build` run again.
  The doc gate passed WITH the binary: api_doc_tool self-test, suite_run self-test, briefing current, staleness clean,
  check-nav (263), check-tone (48), first_week, build_to_program, check-register (232).
- `src_build_gate.py self-test` under `timeout -s INT 3600` + 4 GiB (20:07 to 21:03, beside rmp1-d's): **OK (398
  detector cases, each against a real build)**, run in its copy. `git diff -- Makefile src/` has no "(self-test
  plant", and there is no untracked file in src/include/test (checked after the self-test and again after `check`).
- `src_build_gate.py check` (21:03 to 21:18): **PASS** (14 builds; 6 refuse naming exactly their owed facts and link
  in the typed copy; 0 disallowed warnings).
- ARM header gate: CLEAN (259 headers, CI's exact command). CI greps: PROS-free, sim-free; `include/` untouched.
- api_doc_tool self-test / check-coverage (6,485 entities, 215 headers) / check-fresh / check-examples /
  check-removability: OK. docs/api needs no regeneration: no `///` comment under `include/` changed, and `bench_r3a.hpp`
  is outside the tool's scope. briefing_status self-test + check: OK. doc_staleness_audit self-test + run: clean.
- The briefing here reads 3,868 / 3,887,073 at THIS path: regenerate it in the main tree after the merge.
- Not done here (host only): no `pros`, no upload, nothing run on a robot. Nothing committed.
