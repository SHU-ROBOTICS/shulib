# TUNE1 — live tuning — PROGRESS (live log, appended as the work goes)

<!-- release: between parts — TUNE1-A and TUNE1-B1 landed (6baef55, c0a7c7e); TUNE1-B2 (the copy-back) is after the panel; nothing is building (declared by the coordinator 2026-11-13 for the first release since August) -->

Brief: [`MECH-S-TUNE1-string-lift-safety-live-tuning.md`](MECH-S-TUNE1-string-lift-safety-live-tuning.md)
(its answers block, D8, D9, §1.2, §2.2's TUNE1-A list, §3.1, the re-check amendments — #7 and #14 override the body).

## TUNE1-A

**Run:** live tuning's LIBRARY half (§1.2: D8's rules, D9's `[TUNE]` block). Executor: `opus` (a subagent of the
workflow; the team lead's go of 2026-11-03 "Accept all, go", the order of 2026-11-05: the cascade first — MECH-S-A, then
this run, then TUNE1-B1). Worktree `.claude/worktrees/tune1-a`, branch `tune1-a`, base **`2bc30ab`** — a SNAPSHOT of
MECH-S-A's BUILD (in review, uncommitted on main) over `ef6162e`. Host only: nothing here has run on a robot. Not
committed (the coordinator commits).

### 1. Started (2026-11-05)
- Read: `RESUMING.md` (standing rules); the brief's header + answers, D1–D12 in full, §1.2, §2.1–2.3 (TUNE1-A), §3.1,
  §3.2, §3.4, §4 (no HA id is assigned to TUNE1-A — none registered), §6.2, §7, §8, both re-check sections.

### 2. Resumed (2026-11-10, `date`: Tue Nov 10 20:56 EST) — the first run stopped at the 5-hour limit (2026-11-05 ~18:30)
- Found in the tree: `include/shulib/control/pid.hpp` (+23: `Pid::setGains` bumpless, `Pid::config()`) and
  `include/shulib/mechanism/service.hpp` (+182: `Slot::rederiveSlot()`, `rederiveKind()` per kind, `refreshDetector`,
  `capInMotors_`, `PositionSlot::holding()`, `MechanismService::rederive()` / `canRederive()`), this log's §1. No
  `live_tuning.hpp`, no test, no doc yet. Scratch: `probes/p_tochars.cpp` (+ two binaries) — a `std::to_chars`
  shortest-round-trip probe for the copy block.
- `git diff -- Makefile src/`: EMPTY (no gate self-test plant left). Every mutation backup the coordinator found was
  restored; no mutation had been run yet. Rebuilt from source before trusting anything (below).
- Read again (this run): `RESUMING.md`'s standing rules (§3–§Pace); the brief's header + answers, both re-check
  sections, D7–D9, D11–D12, §1.2, §2.2's TUNE1-A list, §2.3, §3.1, §3.4, §4, §6.2, §7, §8 in full; the rest by heading.
- REBUILT FROM SOURCE (2026-11-10 ~21:00, a fresh `build/test`: `cmake -S test -B build/test`, the API pages and the
  briefing regenerated first — the first run's `/// ` comments had made `check-fresh` stale, which stopped the build
  at its doc gate): the build clean, `python3 tools/suite_run.py green` → **green — 3001 cases, 2956687 assertions**
  (the partial work changes no behaviour yet: nothing calls it). The partial work is KEPT — read line by line against
  D8/§1.2 and found right, with three additions below (§3).

### 3. Built (2026-11-10, ~20:56–21:28 — CORRECTED by the review round: the first text said ~21:00–23:00; the files' times are the record)
- `include/shulib/mechanism/live_tuning.hpp` NEW — `LiveTuning` (the table COPY the service runs; `attach`,
  `apply`, `step`, `undo`, `reset`, `copyBlock`), `tuningRefusal` / `tuningProblem`, `Tunable`, `StepRule`,
  `Parameter`, `parametersOf`, `offered`, `tunableValue` / `setTunable` / `tunableUnit`, `steppedValue`, `Applied`,
  `LiveRefusal` / `liveRefusalText`.
- `service.hpp` (the first run's region, kept) + this run: `PositionSlot::heldButtonMoving()` and
  `PositionSlot::takesRange(candidate)` — the ONE check live tuning asks before it writes and `rederiveKind()`
  asserts after (re-check #7), so the two cannot disagree; the precondition now calls it.
- Tests: `test/mechanism_live_tuning_rig.hpp` (NEW — a rig binding the service over a LiveTuning's table; not in
  the brief's file list: the three test files share it), `test/mechanism_live_tuning_test.cpp`,
  `test/mechanism_tuning_rules_test.cpp`, `test/mechanism_tuning_block_test.cpp`, 4 cases appended to
  `test/pid_test.cpp`. Suite: **green — 3041 cases, 2964748 assertions** (+40 cases).

### 4. Mutations (2026-11-10 21:28–21:39 — CORRECTED by the review round: run 1 `mut_run1.log` 21:28:23–21:34:23, run 2 (the re-cuts) `mut_run2.log` 21:34:39–21:38:51; the first text said ~23:00–23:40) — every one OBSERVED red, the file restored
Driver: `.claude/coordinator/scratch/tune1-a/mutate.py` (backup under `scratch/tune1-a/mut/`, ONE exact replacement,
the suite binary removed and rebuilt from source — make's header dependencies recompile every object including the
file —, the tests `-sf="*tuning*,*pid_test*"` under `ulimit -v 4194304; timeout 600`, the file restored in `finally`
and on SIGTERM/SIGHUP/SIGINT, its timestamp bumped). Logs `mut_run1.log`, `mut_run2.log`.
- **Re-cut, never counted (first run):** MT1, MT3, MT6, MT10, MX1, MX2, MX3 broke their own build — not by
  `-Werror` but by the doc gate's `check-fresh` (the API pages carry each entity's LINE; a mutation that adds or
  removes a line makes them stale); MT9 and MX6 by `-Werror=unused-but-set-variable`. Each re-cut keeps the line
  count and every variable used, and was run again (second run).
- **Verdicts (all CAUGHT — red by a test's assertion, no kill, no timeout):**
  MT1 the PIDs' gains not refreshed → TA1 kP x4 (P6's +0.0000 V) and TA1 bumpless · MT2 `minDeg_`/`maxDeg_` not
  narrowed → TA1's 6 V jog · MT3 the live bounds skipped (build check only) → TA3 live bounds, TA3 "only a changed
  number", TA5 random walk · MT4 `Pid::setGains` keeps the accumulator → pid_test bumpless + TA1 bumpless · MT4b the
  speed integral not rescaled → TA8 flywheel kI · MT5 undo restores the NEIGHBOURING record → TA4, TA1 cap, TA10 ·
  MT6 the derived re-derive loop dropped → TA10 (maxAcceleration), TA5 · MT7 rederive allowed mid-step → TA2, TA4's
  undo-waits · MT8 `takesRange`'s hold check dropped → TA7 (both cases, by `CHECK_FALSE(r.ok)`, not a throw) · MT9
  the block states every number → TA10 (NOT kV), TA5 · MT10 a x1.25 step on a zero kA → TA9 (both), TA4.
  Extra (this run's own): MX1 a changed stall signature never rebuilt → TA1 stall, TA8 (both) · MX2 a live cap
  never written → TA1 cap, TA4, TA8 · MX3 a held button not clamped to a lowered maxVolts → TA8 power · MX4 no undo
  record pushed → TA4, TA1 cap, TA10 · MX5 numbers printed to 6 significant figures → TA5 (long numbers, random
  walk), TA10 · MX6 a stall's rpm read back by the plain inverse → TA5, TA10 · MX7 rederive allowed inside a tick →
  TA2 inside-a-tick.
- After both runs: `grep -rn "// MT\|// MX" include` empty; `git diff --stat -- Makefile src/` empty.
- **Record gaps (found by the review round, kept honest here):** `muts.json` and `muts_recut.json` were BOTH
  rewritten at 21:34:34, after run 1 — run 1's original texts of the nine entries that broke their build
  are lost (MT9's entry reads the same in both files now); both runs filtered 49 cases, and two cases
  added at 21:39 (the guide-11 quoting test, the guide-18h example test) were in neither run. The
  reviewer re-ran all 18 verdicts on the final tree: all CAUGHT. From the review round on, every run's
  mutation texts are their own file, never rewritten (§9).

### 5. Documents (pace rule 3) and the prose review (rule 4) — the greps RUN before this was written
- Public (§3.1, four, under rule 7's ~5): `docs/guide/18-mechanisms.md` — the intro's "what exists" (live tuning's
  library in; the Tune program still to come), the current-cap section's "live tuning, a later part, will report"
  → "the Tune program (live tuning's next part)", and the NEW "### Or on the robot: live tuning" at the end of the
  Tuning section (wiring, what is live, the press rules, the refusals, undo/reset, the COPY block, re-tune after a
  cartridge/gearing/spool change); `docs/guide/11-reading-the-diagnostics.md` — two entries (the `MECH` live-tuning
  lines, the `TUNE` copy block; DELTA: §3.1 gives guide 11's `[TUNE]` lines to TUNE1-B1, but the lines are this
  run's code, so they are documented in the same change — the brief's own §3.4 rule "every new log line's format in
  guide 11 equals the code's (a test pins it)"); `docs/changelog.md` (top of `## API 2.5`); the API pages
  regenerated (`docs/api/live_tuning.md` new; `pid.md`, `service.md`, `README.md`, `all-entities.md`, `mkdocs.yml`).
- Internal: this log; `REMAINING-WORK.md` row 20f (one clause); `roadmap.md` — a "Now (2026-11-10)" paragraph after
  MECH-S-A's; `PROJECT-BRIEFING.md` through the tool. **Not touched, deliberately:** `build-order.md` (no TUNE1-A
  chunk entry exists to move; its `**Next:**` pointer is the coordinator's at the merge), `HANDOFF-remaining-work.md`
  (MECH-S-A did not touch it either), the HA register (no HA id is assigned to TUNE1-A — §4; none used), the
  glossary (§3.1 does not list it for T-A; its "starting values … until `.tuning` replaces them" stays true —
  nothing persists).
- Greps run and hits read: `grep -rn -i "live tuning|tuned live|retuned|rederive|setGains|LiveTuning|live_tuning|
  \[TUNE\]|shulib Tune" docs/guide docs/cookbook docs/*.md README.md` → guide 18 :23 (fixed), :343 (fixed), changelog
  :22 (MECH-S-A's entry, history — kept), roadmap :465 (MECH-S-A's "next is TUNE1-A" — true when written; the new
  paragraph follows it). `grep -rn -i "pid\.hpp|control::Pid|copies its gains|gains.*at bind|…"` docs/guide,
  cookbook, README → no hit. `grep -rn "service\.hpp|MechanismService"` guide 18/10 → guide 10 :720 (the auton
  context — unaffected). Glossary `tun` → :108, :263 (true). Guide 14 and 07, README: no live-tuning claim to
  correct. §3.4: `simulator` — 0 added lines in the diff (the rig's one hit is the house rule "never 'simulator'");
  `internal/|chunks/|RESUMING` in docs/guide, cookbook, README → empty; `HA-(219|22[0-7])` in this run's files → none
  (none assigned); probe names in include/test → empty. Every new log line in guide 11 is pinned by
  `mechanism_live_tuning_test.cpp`'s guide test (and guide 18's text block, member for member).

### 6. Deltas from the brief (against base 2bc30ab) — each with its reason
1. `apply(index, Change)` → `apply(index, Tunable, value)` + `step(index, Tunable, ±1, fine)`: the page presses
   (`step`) and a test or tool sets values (`apply`); a `Change` struct would be the same two fields.
2. `undo()` is GLOBAL (the last change, any mechanism — the sentence names it), as §1.2 writes it.
3. The stall detectors are REBUILT only when their signature changed; an unchanged one is KEPT, its window with it
   (§1.2 says "rebuilt (reset)"): a re-derive for a gain must not restart a stall half-way to tripping (a delayed
   STALLED). A changed one starts fresh (TA1's stall case pins the fresh window).
4. A live bound judges only a number that DIFFERS from the robot file's: a file the build accepted (P9's 500 in/s)
   would otherwise lock out every change. Pinned (TA3 "only a number that differs").
5. kV's live bound is kLiveKvMostTimesStarting (2) × its starting value — the brief names the test ("kV past the free
   speed's") but no number: INVENTED, labelled so in the header.
6. Two live rules the brief did not list, both found while building: a homing push never turned around
   (`HomingPushTurned` — the build lets a MID-travel home push either way) and a stated cap never removed by a
   change (`CapRemoved` — 0 A is NO cap, the motor's full 2.5 A); and a step never removes a cap (a press at its
   last 0.05 A is refused by staying put). Plus `HeldButtonMoving`: a travel limit change while a held button moves
   it (the soft stop would meet a limit it was not braking for) — part of `takesRange()`.
7. A step's result is ROUNDED (a factor's to six significant figures, an additive one to the micro-unit): ×1.05
   repeated would otherwise print 17-digit numbers; 0.1 − 0.05 − 0.05 is exactly 0. ×1.25 then ÷1.25 may not return
   bit-exactly — UNDO does.
8. The `[TUNE]` member lines carry no mechanism name (the begin/end lines do): with the name, the longest homing
   line could pass 200 bytes; without it the bound is 197 by construction.
9. `.tuning` lines are printed only when a `.tuning` number changed live — then the WHOLE `.tuning` (stated +
   changed + derived-differs); otherwise the file's own `.tuning` stays as written (non-tuning numbers never move a
   derived one).
10. The `retuned live` line prints numbers shortest-round-trip (`kP 0.25 -> 0.3125`), not the brief's example's
    fixed `0.250 -> 0.312`.
11. A cap REMOVED live writes each motor back to the 2.5 A default and says so (one Info, or a Warn per motor that
    did not go back) — D12 (b)'s "Tune restores every limit IT changed", done in the library's re-derive; a cap
    added live goes through MECH-S-A's own boot path.
12. Files beyond §1.2's list: `test/mechanism_live_tuning_rig.hpp` (the three test files' shared rig) and
    `test/mechanism_live_tuning_example_test.cpp` (check-examples needs chapter 18's ```cpp lines compiled). The
    slack signature (D8's live set) is NOT live: it does not exist until MECH-S-B1.
13. Host vs robot: a `_s` / `_in` / `_volt` literal reads through long double — 80 bits on the host, 64 (= double)
    on the V5's ARM — so on the host a printed number is read back by double rounding; the tests read the block with
    `strtod` (as the ARM build reads it). Said in the header.

14. (review round) `MechanismService::rederive(index, what = nullptr)`: the brief says `rederive(std::size_t index)
    → bool`; the build adds `what`, the change in words its ONE Info line prints as given (live tuning gives
    `kP 0.25 -> 0.3125`; nullptr says "its numbers re-read").
15. (review round) Rule 3 reaches WHERE IT IS, not only where it holds: holding nothing and homed, a travel
    limit change that leaves where it is further outside the limits than those in force is refused
    (`LiveRefusal::LeavesOutWhereItIs`, a new value; "it is at 19.95 in, outside the new limits: move it
    inside first"). The brief's words ("the target the mechanism is holding now") were followed literally and
    left the jump re-check #7 forbids two presses away: STOP, narrow, HOME → −11 V at once.
16. (review round) A live bound on kS, kA, kV and the top speed refuses a number only when it is ALSO further
    past than the robot file's own: a file's number past a bound steps back toward it (delta 4's promise,
    "a file the build accepted can always be tuned", was false for a page with only [−] and [+]).
17. (review round) The copy block's `.tuning` set is decided by MINIMALITY, not by "the file states it":
    the live-changed numbers, then, one at a time in the derivation's order, the first number whose
    derivation from the set so far differs. The first build guessed "stated" as "differs from the
    all-default starting value", which over-stated (a derived acceleration from a stated top speed) and
    under-stated (a file's number at exactly its starting value). The spec holds values, not statements, so
    a number the file states AT its worked-out value cannot be kept: it is left out, said in the header
    and guide 18 (the cost: after a cartridge or gearing change it follows the derivation).
18. (review round) The block is a PATCH, member by member: `.stall`, `.range`, `.gravity`, `.home`,
    `.currentCap` appear only when changed live (the file's own stay as written), and `.tuning`, when it
    appears, replaces the file's whole. TUNE1-B2's import must therefore MERGE member by member, never
    replace the declaration.
19. (review round) A current cap changed live STAYS in the motors after the program unless RESET or UNDO
    writes the file's back (HA-220; D12 (b)'s next program reads it and writes nothing). The first text
    (header, guide 18) said leaving the program restores every number — corrected; nothing does it at the
    program's end (LiveTuning lives in static storage). TUNE1-B1's page must tell the student (D12 (b)).

### 7. Where this run DEPENDS on MECH-S-A's build (in review — the coordinator re-checks these at the merge)
`Slot::bootCurrentCap()` (called by `rederiveSlot()` for a cap added or changed, or a signature changed under a
cap), `capState_` / `CapState::Ok`, `milliamps()`, `defaultMilliamps()`, `clampAmps()`, `motorPlace()`,
`Devices::currentLimits`, `hal::IMotorCurrentLimit`, `hal::kDefaultMotorCurrentLimit`, `recheckCurrentCap()` (and, since the review
round, the fact that `bootCurrentCap()` sets EVERY motor's `capState_` — why rederiveSlot()'s removal path no
longer touches it), the 0.2 A margin Warn's text `'…': its stall signature (X A) is within 0.2 A of its current
cap (Y A): …` (pinned by the review round's margin case),
(reads the spec's cap each second — a live cap is re-checked, a removed one not), `MechanismSpec::currentCapAmps`,
the three cap refusals and their order in `specRefusal` (TA3 pins them), `kinds.hpp`'s `CurrentCap` (the copy
block's `.currentCap = {.amps = …}`), the boot line `'…': current cap X A per motor (N motor[s]: Y A in all) -- set
and read back` (pinned by TA1's cap case), the FakeMotor limit knobs (`refuseCurrentLimit`, `currentLimitWrites`),
and the rig's per-motor limits (the position rig's pattern).

### 8. Gates (2026-11-10 ~21:42–22:15 — CORRECTED by the review round: the first text said 2026-11-11; `srcgate.log` 21:42–22:15 — final tree of THAT round; every binary under `ulimit -v 4194304` + `timeout`)
- Suite: `python3 tools/suite_run.py green` → **green — 3043 cases, 2964789 assertions** (base 3001 → +42).
- `ctest -R compile_refusal` → 100% tests passed, 0 failed out of 258.
- ARM header gate (every `include/shulib` header, one TU, `-Werror`, cortex-a9 softfp) → exit 0, 243 lines.
- CI guards: no `pros/` include outside `hal/pros/`; no `shulib/sim/` include outside `sim/` → both empty.
- `src_build_gate.py self-test` → OK (269 detector cases); `check` → PASS (all 7 builds link); `git diff -- Makefile
  src/` empty, no self-test plant.
- `api_doc_tool.py`: self-test OK; check-coverage all documented (5639 entities, 199 headers); check-fresh exit 0;
  check-examples 654 quoted lines verbatim (13 source files); check-removability clean.
- `doc_staleness_audit.py` clean; `briefing_status.py check` current (regenerated with the binary removed, then
  again after it existed). `robot_tool.py`: not in this base (G3a Part 1 unmerged) — N/A.
- Host only: nothing here has run on a robot. Not committed.

### 9. TUNE1-A review findings and their disposition (2026-11-10, `date` 23:11 to 2026-11-11 00:23 EST; three reviewers — correctness, tests, docs; 17 findings, deduplicated below)
Every finding re-checked against the code before acting; each fixed at the root or answered with evidence.
Scratch: `.claude/coordinator/scratch/tune1-a/` (`mutate_v3.py`, `muts_run3_review_round.json` — its own
file, never rewritten — `mut_run3_review_round.log`, `probes/fixed_point.cpp`).

**C1 (major, correctness) — a range narrowed past where it IS, not holding, then the next hold jumps. REAL —
FIXED.** Reproduced the reviewer's reading (`takesRange` judged only `holding()`; `holdAt()` clamps
`jointDeg()`). Root fix in `PositionSlot::takesRange()` (the ONE check both `apply` and `rederiveKind`'s
precondition use): homed, neither holding nor under a held button, a candidate is refused when it leaves
`jointDeg()` FURTHER outside its limits than the limits in force do (distance outside, new > old) — so a
widening, or a change that leaves an already-outside mechanism no further out, is never refused (a lift
resting a hair under its 0 in limit can still have its top narrowed). New `LiveRefusal::LeavesOutWhereItIs`
("it is at 19.95 in, outside the new limits: move it inside first"). Delta 15. Tests (TA7 review): STOP at
20 in → narrow to 12 refused (CHECK_NOTHROW, spec byte for byte) → HOME holds where it is (|V| < 3, the
probe's −11 V gone); the equality edge LANDED exactly (a max whose degrees ARE `jointDeg()` accepted, one ulp
under refused, both REQUIREd); the lower limit while released; already outside (carried onto the 24.5 in
hard stop past the 24 in limit): the far end accepted, the near end refused. Guide 18, the header, the
service's comments say "now or at the next hold".

**C2 (minor) — a file's number past a live bound could never be stepped back. REAL — FIXED.** kS, kA, kV and
the top speed are refused only when past the bound AND further past than the robot file's own number
(`pastAbove` / `pastBelow0` in `tuningRefusal`). Delta 16. The old pin "499 refused on a 500 in/s file"
inverted (499 accepted, 501 refused). New TA3 case: a file with 10 in/s, kV 0.1, kS −0.5 — every [−] on the
top speed accepted down under the free speed, [+] and one ulp past 10 refused; kV [−] 0.08 accepted, [+]
refused; kS [+] −0.45 accepted, [−] and one ulp under −0.5 refused. GainNotAbove0 needs no such rule: a
declaration cannot carry a gain ≤ 0 (`positionStart` replaces it by the starting value).

**C3 + D1 (minor + major — the copy block's "stated" set). REAL — FIXED at the root.** Both were one bug: the
block guessed "the file states it" as "it differs from the ALL-DEFAULT starting value" — over-stating a
derived acceleration (`.tuning = {.maxSpeed = 5}` printed `.tuning.maxAcceleration = 20`) and under-stating a
number stated at its start. The spec holds values, not statements, so "stated" cannot be known; the block
now decides by MINIMALITY: the live-changed numbers, then, ONE AT A TIME in the member order (the
derivation's own — only the acceleration hangs on another number, the top speed, before it), the first
number whose derivation differs from the live one. A number so added must be stated; none that would
re-derive the same is. Consequence, said honestly in the header and guide 18 (reviewer C3's option (a)): a
number the file states AT its worked-out value is left out — the block rebuilds the same spec today, and
that number follows the derivation after a cartridge or gearing change. Delta 17. Tests (TA10 review):
the maxSpeed-only file + a kP press → exactly `kP`, `maxSpeed` (no acceleration), rebuilt field for field;
a Green 12:36 file stating kV = 0.03 (its start) and kP 0.4 → kP kept, kV absent, rebuilt identical; the
header's example block pinned line for line. Two mutants guard it: MZ6 (add every differing number per
round, the overshoot) and MZ8 (the old guess restored) — both CAUGHT.

**D2 (major) — "leaving the program restores the robot file's numbers" is false for a live cap. REAL —
FIXED.** The header and guide 18 now say: one exception, the cap is in the motors and a motor keeps it after
the program (HA-220); the next program reports it but writes nothing (D12 (b)); RESET or UNDO back to the
file's cap before leaving — nothing does it at the program's end (a static object's destructor is no place
to drive a motor). Guide 11 says a cap changed live and never reset is what the next program's "a limit it
found" line reports. Delta 19, with TUNE1-B1's duty (the page must say it — D12 (b)).

**D3 (minor) — format/example text.** (a) `rederive()`'s example now `kP 0.25 -> 0.3125` and says `what` is
printed as given. (b) The header's example REPLACED by a real run, pinned by a test (the cascade, kP [+]
and 2 in/s: `0.3125`, `2`, `23.561944901923447`), shortest form, its begin line `<its file>:<its line>`.
(c) `src/robots/robot_one.hpp` does not exist: guides 11 and 18 now show `src/robots/mechanism_table.hpp:41`
(the file robot declarations live in) and say it is an example line; the guide-quoting test updated. (d)
"±5 %" → "×/÷ 1.05 (about 5 %)" in guide 18, the header and `StepRule::Percent`.

**D4 (minor) — deltas missing from §6.** Added: 14 (`rederive`'s `what`), 17 (minimality — not "differs from
start"), 18 (the block is a PATCH member by member; TUNE1-B2's import must MERGE).

**D5 (minor) — build-order and HANDOFF untouched.** Both stale claims are DATED history ("added 2026-10-30 …
not yet briefed"; "(2026-10-30: … briefs next)") — kept, not rewritten (DOCS3's rule: history indexed, not
rewritten); each now carries a minimal dated note after it: build-order :855 `[then: briefed and answered
2026-11-03; MECH-S-A built 2026-11-05 and TUNE1-A 2026-11-10, host-only — …PROGRESS.md]`, HANDOFF's G3a entry
a `(2026-11-10: …)` note. No TUNE1 chunk ENTRY exists in build-order to move (§3.2's "chunk entries" have
none for MECH-S/TUNE1) — the coordinator adds one at the merge if wanted. MECH-S-A may touch the same
:855 line: whichever lands second rebases.

**D6 (minor) — the prose grep missed "live tuner" (H3). REAL — logged and cross-referenced.** Re-ran
`grep -rn -i "live tuner|live tuning|tuned live|robot_one\.hpp|panel can import|import of the copied|
nothing persists|leaving the program|can always be tuned|never moved by a change|±5 %"` over docs/guide,
docs/cookbook, docs/*.md, README.md and the two headers; hits read: `docs/vexbuilder-integration.md:71`
("On-brain live tuner ❌ H3") — still literally true (no program exists; H3 is the drive's, over H1's wire),
left for the coordinator; `docs/roadmap.md:2224` (WS10's "live tuner") — H3's workstream, true;
`docs/shulib-v2-master-plan.md:514` ("Cut from v1 … on-robot live tuner") — a dated design record, kept;
build-order's H3 entry now notes `(2026-11-10: TUNE1 … builds this for MECHANISMS …; H3 remains the DRIVE's
numbers)`. Every other hit is this run's own text, read and true after the fixes.

**D7 (minor) — panel-dependent text vs the team lead's latest word ("we dont need the panel. we just need to
be able to fully program the bot for skills auton").** My own lines no longer promise a panel import: guide
18 "**Write them in by hand**, replacing your `.tuning = {...}`"; the changelog "the copied numbers are
written into the robot file by hand". NOT mine to change: REMAINING-WORK row 20f's pre-existing "copy the
numbers into the panel" and "before the panel lands", and TUNE1-B2's place in the plan — **the coordinator's
ruling with the team lead** (flagged).

**T1 (major) — live kP/kD on the MOVE loop, kD anywhere (MY1, MY2). FIXED.** Three-twin cases (`splitAfter`):
until the first tick two twins differ all three are identical, so the differences are the changed gain's
share of ONE shared input and their RATIO is ΔkP_a/ΔkP_c exactly (1e-9) — kP on the move (first differing
tick ≤ 3, inside the profile), kD on the move and on the hold's next tick.

**T2 (major) — rule 3's lower limit, undo, reset (MY7, MY8, MY14, MY18, MY20). FIXED.** On the new
switch-homed fixture (its home at 12 in, so its minimum can be raised past a hold): min raised past a hold
at 8 in refused with its sentence; the LOWER equality edge exactly (8 in accepted, one ulp above refused —
the degrees' inequality REQUIREd); a held button + a min change refused; narrow → widen → hold at 18 → undo
refused (CHECK_NOTHROW, LeavesOutHeldTarget, the record kept), accepted after a move inside; RESET during a
step refused with "'cascade': RESET refused -- …".

**T3 (major) — switch-homed / noGravity never tested (MY12, MY13, MY16, MY22).** Fixture `switchLift`
(homeBySwitch(12_in, 6_volt), noGravity, Coast; the rig wires a FakeDigitalIn the plant presses at
`switchAt(12)`): its `parametersOf()` pinned (homing push, no homing signature, no gravity hold); gravity hold
and homing amps refused NotLive; its `.home = homeBySwitch(12_in, 6.05_volt)` pinned EXACTLY and read back;
it joins the random walk. The parser now must consume each WHOLE line (`scanAll`, `%n`) — MY16's `_volts`
had parsed. **"Stall-less" is impossible**: the build refuses a position spec without a stated stall
(`spec.hpp` `PositionTheStallSignatureIsUNSET`), and `rederive` requires a spec the build accepts — so
**MY22 is EQUIVALENT** (`p.stall.stated()` is always true there; it SURVIVED the full suite, as expected).

**T4 (major) — takes-effect gaps (MY3, MY4, MY5, MY9, MY15, MY6). FIXED but MY6.** (a) A lowered acceleration:
the soft stop takes over earlier than its twin's. (b) Homing signature: both twins at 0.3 A (this plant's
−2 V push draws 0.42 A at most: the file's 0.6 A never trips — measured), one with a 0.5 s lasting: it
succeeds 29–31 ticks later. (c) An unchanged signature keeps its window: a kP press half-way trips at the
SAME tick as the twin. (d) A stall raised to 1.35 A under a 1.5 A cap: the HA-223 margin Warn, exactly once.
(e) A flywheel's kI changed twice between ticks: still ΔkI × this tick's increment. **MY6 is EQUIVALENT:** the
speed integral is clamped to ±3 V/kI at every accumulation, so |kI_old·I| ≤ 3 always, and the rescaled
term kI_new·(kI_old·I/kI_new) is the same term — the post-rescale clamp can trim at most an ulp, and the next
update clamps again before the term is used beyond that. Kept as a guard; SURVIVED, as expected.

**T5 (minor) — pid_test's "unchanged kI bit for bit" cannot tell the guard (MY11). The reviewer's fix is NOT
POSSIBLE, with evidence:** every accumulator a Pid can hold is fl(w/kI) — `update()`'s back-calculation,
`setGains`' rescale, or 0 — and fl(w/k) is a fixed point of x → fl(fl(k·x)/k): `probes/fixed_point.cpp`, 0
of 20,000,000 random (w, k) pairs moved (and every kI/error the reviewer's idea suggests, searched in
Python, likewise). So MY11 is EQUIVALENT; the guard states the intent; the test's comment now says so.

**T6 (minor) — rederive's spec precondition (MY17), roundSix ≥ 1e6 (MY19). FIXED:** a `pos_rig::ArmRig` (its
slot runs the rig's own mutable spec) with tolerance 0 → `rederive` throws PreconditionError;
roundSix(1234567.8) == 1234570, (−98765432.1) == −98765400, (0.000123456789) == 0.000123457.

**T7 (minor) — the mutation record. FIXED:** §3, §4, §8's times corrected from the files (§4's record gaps
written in); this round's texts in their own file; the whole table re-run on the final tree (below).

**Found while fixing (not raised): MY10** (the reviewer's run, not a finding — `capState_[m] = Ok` in the cap
removal path) SURVIVED for a reason: the once-a-second re-check runs only under a stated cap, and a cap stated
again goes through `bootCurrentCap()`, which sets every motor's `capState_`. A dead store — REMOVED, with a
comment saying why (§7 records the dependency on MECH-S-A's `bootCurrentCap()`).

**Mutations, review round** (`mutate_v3.py`: `make shulib_tests/fast` — no doc gate, so a line shift cannot
break a build —, header dependencies recompile every dependent object, the COUNT recorded: service.hpp 59,
pid.hpp 111, live_tuning.hpp 4 = its four test files; filtered `*tuning*,*pid_test*` (71 cases), a survivor
re-run on the FULL suite; every binary under `ulimit -v 4194304; timeout 600`; restored in `finally` and on
SIGTERM/SIGHUP/SIGINT, timestamps bumped). Run 23:26:05–23:41:46, 49 mutations, none broke its build:
- the first build's 18 on the FINAL tree (MT8, MT9 re-cut for the changed code): all CAUGHT;
- the reviewers' MY1–MY5, MY7–MY9, MY12–MY21: all CAUGHT (red by a test's assertion — each verdict's failing
  case is in the log; none by the cap or the timeout); MY6, MY11, MY22: SURVIVED the full suite (3063/3063)
  — EQUIVALENT, argued above; MY10: its line removed;
- new MZ1 (the where-it-is clause dropped), MZ2 (`>` → `>=`), MZ3 (judged against the new limits alone), MZ4
  (the top speed/kV toward rule dropped), MZ5 (kS/kA's), MZ6 (all differing numbers per round), MZ7 (the
  refusal named LeavesOutHeldTarget), MZ8 (the old "stated" guess), MZ9 (the sentence says "holds"), MZ10
  (the homed check dropped): all CAUGHT.
- After: `grep -rn "// MT\|// MX\|MY[0-9]*:\|MZ" include` empty; `git diff -- Makefile src/` empty.

**Gates, review round (final tree, 2026-11-10/11):** API pages regenerated; binary removed → briefing →
build (doc gate in it) → `python3 tools/suite_run.py green` → **green — 3063 cases, 2966969 assertions**
(3043 → +20) → briefing regenerated with the binary present. `ctest -R compile_refusal` 258/258. ARM header
gate (243 lines, every `include/shulib` header) exit 0. CI guards (no `pros/` outside `hal/pros/`, no
`shulib/sim/` outside `sim/`) empty. api_doc_tool self-test OK, check-coverage 5640 entities / 199 headers
all documented, check-fresh exit 0, check-examples 654 lines verbatim, check-removability clean;
doc_staleness_audit clean; briefing_status check current. **src gate: see the next bullet.**
- **A self-test plant WAS left and removed:** my first `src_build_gate.py self-test` ran under
  `timeout 600`, which killed it mid-run (the gate takes ~30 min) and left `SelfTestUnhandled,  ///<
  self-test plant (case 18)` in `src/build_selection.hpp`; found by `git diff -- Makefile src/`, restored by
  `git checkout -- src/build_selection.hpp` (its only change), timestamp bumped. Re-run with a 3000 s
  limit (`srcgate_review.log`): self-test OK (269 detector cases, exit 0); check PASS (all 7 builds link, exit 0); `git diff -- Makefile src/` EMPTY after it — no plant.

Host only: nothing here has run on a robot. Not committed.

### 10. Verification-round findings and their disposition (2026-11-11, `date` 00:30–01:30 EST; a fresh verifier, two findings, both REAL)

**V1 (major) — the C1 fix's where-it-is clause made `rederive()`'s precondition read a LIVE sensor a second
time. REAL, FIXED AT THE ROOT.** Re-checked: `PositionSlot::takesRange()` reads `jointDeg()` (the motors'
`meanPosition()` or the angle sensor — on PROS, device data VEXos updates in the background) and ran twice:
in `LiveTuning::stateRefusal()` before the write, and as `rederiveKind()`'s `SHULIB_PRECONDITION` after
`commit()` had already copied the candidate into the live table. The verifier's probe
(`verify/probe_toctou.cpp`, a motor wrapper whose reading moves 0.01 rad per read) threw out of `apply()`
with the live table at the new limit, the slot not re-derived, and no record — reproduced from its log.
The log's earlier text (§2, §9) and the header's claim "the two can never disagree" held only while the rule read slot state.
- **The fix — ONE decision, made by the service, and a refusal instead of a throw:** a new protected
  virtual `Slot::takesSpecNow()` (true; `PositionSlot` overrides it with `takesRange(spec_->position)`).
  `MechanismService::rederive()` asks it ONCE, after its own checks and before anything is re-derived,
  and returns false (nothing re-derived, nothing said) when it fails. `rederiveKind()`'s precondition is
  REMOVED (nothing in it reads the sensor; it runs only after that one read said yes). `LiveTuning::commit()`
  on a false: the old spec back byte for byte, an undo's record kept, nothing pushed, and the reason from
  a new private `lateRefusal(i)` — the slot's STATE only (step / tick, holding, held button, else where it
  is), no second sensor read deciding it — in the sentence the same request would have got before the
  write (a change through `refuse()`, so "it is at %.2f in, outside the new limits"; undo / RESET their own
  "undo refused -- …" / "RESET refused -- …"). The verifier's option (a), not (b): (b) would leave a
  direct caller of `rederive()` able to narrow past where it is unguarded.
- **Why not one read in total:** live tuning still asks first, for the reason and its sentence; the
  service's read is the one that counts. They can now disagree only by a reading that moved between them,
  and that is a clean refusal.
- **A reset can never be refused late by where it is** (the robot file's range is never narrower than the
  live one, so `takesRange` cannot fail on it after a step / tick check passed) — the reset branch of the
  late sentence is reached only by a step or tick state that cannot change between the two checks either;
  no test reaches it, and none can without a fault-injecting service. Said here, not hidden.
- **Comments corrected:** `takesRange`'s ("can never disagree" → the two calls can disagree when a
  reading moves; the service then refuses), `rederiveKind`'s (no longer a precondition — why), `rederive`'s
  (the third refusal reason; "a caller that wrote the spec itself must put the old one back on false"),
  `commit`'s ("it cannot after stateRefusal()" → the service's answer is the one that counts).
- **DELTA 20 from the brief:** §1.2 (re-check #7) says "`rederive` ASSERTS it, a precondition". It is now a
  REFUSAL (false), for the reason above. Re-check #7's rule is unchanged: no re-derive moves a hold.
  STEP1-B's check functions are untouched (this is not a start-command precondition — §1.5).
- **Docs in the same change:** guide 18 (the refusal list: where it is is read again as the change is
  applied; a lift still coasting across the new limit gets the same refusal, nothing changed or recorded),
  the changelog's `rederive` entry (the third refusal reason), docs/api regenerated (`service.md`,
  `README.md`, `all-entities.md`, `mkdocs.yml`).
- **Tests (adversarial, `mechanism_live_tuning_test.cpp`, the verifier's wrapper as `DriftingMotor`):**
  (1) an `apply` whose reading crosses the new limit between the two reads — `CHECK_NOTHROW`, not ok,
  `LeavesOutWhereItIs`, the sentence's head and tail, the spec BYTE FOR BYTE, `changed` false, no record, no
  `retuned live` line, and the slot NOT re-derived (its next hold begins past the refused limit, which a
  re-derived slot would clamp to); (2) an `undo` refused the same way — undo's own sentence exactly, the
  record kept (depth 2, the value 23), then taken once the reading is back; (3) a DIRECT `rederive()` on a
  spec its caller narrowed past the hold with kP ×4 — `CHECK_NOTHROW`, false, no line, and the very next
  command equal TO THE BIT to a twin's (neither the gain nor the limits re-derived), the target still 20 in.

**V2 (major) — the flywheel copy block's minimality loop was untested (the verifier's V17b survived the full
suite). REAL, FIXED.** Re-checked: the only flywheel fixture states no `.tuning`, so the loop never had a
stated number to add. Added to `mechanism_tuning_block_test.cpp` "TA10 (verification round)": a Blue 72:12
flywheel whose file states `.tuning = {.kS = 0.3, .kI = 0.5}` (kI's start is 0.2 — REQUIREd, so 0.5 is a
stated number), one [+] on kP: the block is EXACTLY `.tuning.kS = 0.3`, `.tuning.kP = 0.0625`,
`.tuning.kI = 0.5` (member order; no kV — derived, untouched), read back through `rebuiltFlywheel` to the
live spec field by field; and the fixture flywheel's same press is `.tuning.kP = 0.0625` alone, round-tripped.

**Mutations, verification round** (`vround/mutate_v4.py` — v3 plus several edits in one file; backups under
`vround/mut/`; `make shulib_tests/fast` with header dependencies, the recompiled COUNT recorded; filter
`*tuning*,*pid_test*` (75 cases); every binary under `ulimit -v 4194304; timeout 600`; restored in `finally`
and on SIGTERM/SIGHUP/SIGINT, timestamps bumped). Run 00:54:06–00:55:52 (`vround/vmut_run.log`), 10
mutations, none broke its build, ALL CAUGHT, each red by a TEST'S ASSERTION (none by the cap or the timeout;
no compile-time check involved):
- VR1 THE PRE-FIX CODE (the takesSpecNow refusal removed, the precondition put back) — 59 objects; 3 new
  cases red (the two drift cases and the direct-rederive case, by their `CHECK_NOTHROW`);
- VR2 the refusal in `rederive()` removed — 59; 3 red. VR3 `PositionSlot::takesSpecNow()` → true — 59; 3 red;
- VR4 `lateRefusal` → `InsideATick` — 59 (live_tuning.hpp: its dependents); 2 red. VR5 the undo record not
  kept — 4; 1 red. VR6 the old spec not put back — 4; 2 red. VR7 a change's refusal not through `refuse()`
  — 4; 1 red. VR8 `lateRefusal`'s where-it-is → `LeavesOutHeldTarget` — 4; 2 red. VR9 undo's sentence
  says RESET — 4; 1 red;
- VR10 THE VERIFIER'S V17b (`break;` before `SpeedTuning t{};`) — 4; the new flywheel case red.
- After: `grep -rn "VR[0-9]|// MT|// MX|MY[0-9]*:|MZ[0-9]" include` empty; `git diff -- Makefile src/` empty.
  (VR4's 59 vs VR5–VR10's 4: VR4 ran first after the service.hpp mutations, so the restored service.hpp's
  59 dependents rebuilt with it.)

**Prose review (rule 4) — greps RUN before this was written:** `grep -rn -i "rederive|takesRange|takesSpecNow"
docs/guide docs/cookbook docs/*.md README.md` → only the changelog's entries (:46–:55, updated). `grep -rn -i
"can never disagree|cannot disagree|PositionSlot: a precondition|it cannot after|PositionSlot::rederive:"`
over include, docs/api, docs/guide, the changelog → no hit in any mechanism file or page (the other hits are
other headers' own, unrelated). Guide 18's refusal list read whole — its new sentence is true of the code
(the service's read; the same `LeavesOutWhereItIs` refusal; nothing recorded). No HA id used.

### 11. TUNE1-A review findings and their disposition — RESUMED after the stop (2026-11-11, `date` Wed Nov 11 02:00:16 AM EST to ~03:00 EST)
The fix run was stopped by the team lead's pause (2026-11-11 ~00:30, usage credits) and resumed after it.
Scratch for this part: `.claude/coordinator/scratch/tune1-a/resume/` (its own driver `mutate_v5.py` —
v4 with its own backup dir `resume/mut/` —, its own table `muts_resume_v2.json`, `mut_run_resume.log`,
`srcgate_resume.log`, `probes/`).

**State found on resuming.** Every one of the 17 findings already had a disposition (§9), and the
verification round (§10) had run; what had NOT been recorded was the final tree's gates after §10's fix.
- No mutation left applied: `include/shulib/mechanism/service.hpp` and `live_tuning.hpp` are byte for
  byte the last backups the verification-round driver took (`vround/mut/VR3.service.hpp.orig`,
  `VR10.live_tuning.hpp.orig`), `include/shulib/control/pid.hpp` the review round's `mut3/MY11.pid.hpp.orig`;
  `vround/vmut_run.log` ends `restored_and_rebuilt: true`; `grep -rnE "VR[0-9]+|// MT|// MX|MY[0-9]+:|MZ[0-9]+"
  include` empty.
- No src-gate self-test plant: `git diff -- Makefile src/` EMPTY (0 lines).
- The test binary was removed, the briefing regenerated (it read the binary as unbuilt), the suite rebuilt
  and relinked from source, then the suite run and the briefing regenerated — before any result below.

**Re-verified on a FRESH build (the reviewers' own probes, recompiled against the final headers in
`resume/probes/`, every binary under `ulimit -v 4194304; timeout 600`):**
- C1 — `review-correctness/probe1.cpp`: the narrowing to 12 in after STOP at 19.95 in is now REFUSED
  ("'cascade': upper limit 24 -> 12 refused -- it is at 19.95 in, outside the new limits: move it inside
  first"); HOME then holds at 19.950 in with +1.0 V (the reviewer's −11 V gone). PROBE1b/1c now stop at
  their own `REQUIRE(... apply(...).ok)` — the narrowing they need is refused, which is the fix.
- C2 — `probe3.cpp`: six [−] on the 10 in/s top speed and three [−] on kV 0.1, every one accepted.
- C3 / D1 — `review-docs/probe_stated.cpp`: `.tuning = {.maxSpeed = 5}` + one [+] on kP → exactly
  `.tuning.kP = 0.3125`, `.tuning.maxSpeed = 5`, "copy ends -- 2 lines" (no derived acceleration).
  `probe2.cpp`: its random walk (1671 applied changes) round-trips; PROBE2b's stated-at-start kV is left
  out, as §9 decided (option (a)) and guide 18 / the header now say.

**The WHOLE mutation table re-run on the final tree** (the 49 of §9 + the 10 of §10 = 59; `make
shulib_tests/fast` with header dependencies, the recompiled count recorded; filter `*tuning*,*pid_test*`
= 75 cases; a filter survivor re-run on the FULL suite; restored in `finally` and on SIGTERM/SIGHUP/SIGINT,
timestamps bumped). Run 02:03:38–02:22:14 EST. **54 CAUGHT, each red by a test's assertion** (none by the
memory cap or the timeout, none by a compile-time check, none broke its build), **5 SURVIVED the full
suite (3067/3067)**:
- **MY17 RE-CUT as MY17r:** §10's fix moved the line after `rederive()`'s spec precondition (it now
  precedes `if (!s.takesSpecNow())`), so the old text no longer occurred; MY17r drops the same
  precondition → CAUGHT ("rederive's precondition: a spec the build refuses …").
- MY6, MY11, MY22: SURVIVED, EQUIVALENT — as argued in §9 (unchanged).
- **MY14 and MY18: SURVIVED here, though CAUGHT in §9's run — EQUIVALENT NOW, because of §10's fix, not a
  hole.** Re-checked in the code: since §10, `commit()` asks the service, and on its refusal puts the old
  spec back byte for byte, keeps an undo's record (`--undoCount_` then `++undoCount_`, the record's memory
  never written) and says `lateRefusal(i)`'s reason in the SAME sentence form (`"'%.23s': %s refused --
  %.180s"` with "undo" / "RESET" — character for character the early sentences). MY14 (reset's early state
  check forced to None) during a step: `rederive()` returns false at `canRederive()` before anything else
  (nothing re-derived, no line), `lateRefusal` gives StepRunning → "'cascade': RESET refused -- …", the
  same. MY18 (undo checked against the live spec, not the record): the service's `takesSpecNow()` refuses
  the record's narrower range by the HOLD rule, which reads slot state (`toDeg_` against the limits), not
  the sensor — so the late path gives LeavesOutHeldTarget with the undo sentence and the record kept, the
  same. The tests that caught them in §9 ("TA4 (review): an UNDO that would narrow a range past the hold
  …", "TA4 (review): a RESET while a step runs …") still pin every observable; the early check is kept
  because it gives the reason before anything is written (a change's sentence needs it, and it costs
  undo/reset no re-derive call). Defence in depth, said here rather than hidden.
- After: `grep -rnE "VR[0-9]+|// MT|// MX|MY[0-9]+:|MZ[0-9]+" include` empty; `git status` shows only this
  run's files; `git diff -- Makefile src/` empty.

**No code or document changed in this resumption** (so no prose review was due beyond §10's; no HA id used).

**Gates, final tree (2026-11-11, 02:00–02:59 EST):** binary removed → briefing → build → `python3
tools/suite_run.py green` under `ulimit -v 4194304; timeout 600` → **green — 3067 cases, 2967073
assertions** (twice: before the mutation run and again after it, each on a freshly relinked binary) →
briefing regenerated, `briefing_status.py check` current. `ctest -R compile_refusal` 258/258. ARM header
gate (243 lines, every `include/shulib` header, `-Werror`, cortex-a9 softfp) exit 0. CI guards (no `pros/`
outside `hal/pros/`; no `shulib/sim/` outside `sim/`) empty. `api_doc_tool.py` self-test OK, check-coverage
5640 entities / 199 headers all documented, check-fresh 0, check-examples 654 lines verbatim,
check-removability clean (each exit 0). `doc_staleness_audit.py` clean (exit 0). `src_build_gate.py
self-test` OK (269 detector cases, exit 0, 02:28–02:54) and `check` PASS (all 7 builds link, exit 0,
→02:59), run under `timeout 3000`; `git diff -- Makefile src/` after it: 0 lines, no plant.

**Left for the coordinator (unchanged from §9):** D5 — build-order's TUNE1 chunk entry, if wanted, at the
merge; D6 — `docs/vexbuilder-integration.md:71`'s H3 row; D7 — REMAINING-WORK row 20f and TUNE1-B2's
place in the plan, the team lead's ruling on the panel; §7 — every dependency on MECH-S-A's cap path.

Host only: nothing here has run on a robot. Not committed.

## TUNE1-A — the coordinator's merge verification *(2026-11-11)*

- **Applied as its OWN diff** (`2bc30ab..` its transport commit), not a squash: TUNE1-A was built on a snapshot of MECH-S-A's
  PRE-REVIEW build, so a squash would have re-applied the pre-review cap. `git apply --3way` onto `1ed0e26` (API 2.6): every
  code file applied cleanly; conflicts only in documents — guide 18's opening (the live-tuning sentence added after the
  cap's), REMAINING-WORK row 20f (TUNE1-A LANDED), the roadmap (its "Now" paragraph after STEP1-A's, re-worded LANDED), the
  changelog (TUNE1-A's entry at the top of `## API 2.6`, dated "2026-11-11 (built 2026-11-10)"; a stray pre-review copy of
  MECH-S-A's heading the patch carried was removed), and the generated `docs/api/` pages and briefing (regenerated). No minor
  bump: `Pid::setGains`/`config()` are in C1's layer, not the Freeze Register; the rest is new.
- **§7's dependencies on MECH-S-A re-checked against the cap AS LANDED (`49d1a22`)**: `bootCurrentCap()` still sets every
  motor's `capState_` (Ok, LostWarned or OtherWarned — a missing limit included), the margin Warn's text is unchanged (its
  test is now judged by `tripMilliamps`, which TUNE1-A's pinned cases also pass), the boot line, `recheckCurrentCap()`, the
  refusal order and `CurrentCap` are as TUNE1-A used them. The suite on the merged tree is the proof.
- **Gates on the merged tree** (`.claude/coordinator/scratch/tune1_a_merge_verify.sh`, every binary capped): the suite green;
  `src_build_gate.py` self-test OK and check PASS (all 7 builds link); the ARM header gate CLEAN (244 headers); the doc gates
  and the briefing check clean; the compile-refusal harness 258 of 258; no plant, no stray root file.
- **The coordinator's own mutations** (`muts_tune1_a_main.json`): **4 of 4 RED** — MU1 a gain change keeping the accumulator
  (the I-term jumps), MU2 rederive allowed while the slot's step runs, MU3 a narrowing past the held target accepted, MU4
  undo 15 deep. The restored tree relinked and ran green.
- **TUNE1-A LANDED.** Nothing has run on a robot. TUNE1-B1 ("shulib Tune") merges next, as its own diff the same way.

## TUNE1-B1

**Run:** "shulib Tune" on the brain (§1.3: D7 (a), D8 (a); re-check #3, #7, #8, #10, #14 override the body).
Executor: `opus` (a subagent of the workflow; the team lead's go of 2026-11-03 "Accept all, go" on D1–D13; the
team's situation of 2026-11-10: first competition NOV 21, skills only; robot one built ~Nov 14 with a 4-stage string
CASCADE, its numbers unknown until built — THIS PROGRAM IS HOW THE TEAM TUNES IT the week of Nov 14–18, so it must
work for any robot G1 registers and its TEST / HOME / COPY flow must be fast and clear for a non-programmer).
Worktree `.claude/worktrees/tune1-b1`, branch `tune1-b1`, base **`662f252`** — a SNAPSHOT of TUNE1-A's BUILD (in its
fix round) on a snapshot of MECH-S-A's build (`2bc30ab`) on `ef6162e`. Host only: nothing here has run on a robot.
Not committed (the coordinator commits).

### 1. Started (2026-11-10)
- Read: `RESUMING.md`'s standing rules; the brief's header + answers block, both re-check sections (the table and
  the continuation), D7, D8, D9, D12 in full, §1.2–§1.4, §2.0, §2.1, §2.2's TUNE1-B1 list (TB1–TB12, MB1–MB8), §2.3,
  §3.1–§3.4, §4 (NO HA id is assigned to TUNE1-B1 — the "Settled by" column names the Tune program as the
  instrument for HA-219/220/222/223, but registers nothing for this run: none registered), §5, §6.2, §7, §8 in full;
  the rest by heading (the team lead's ruling of 2026-10-22 on how to read a brief). This log's TUNE1-A sections.
- `git status` at start: clean at `662f252`.

### 2. Built (2026-11-10 23:45 – 2026-11-11 ~01:30)
- NEW `include/shulib/mechanism/tune_panel.hpp` — the pure screen model: `TunePage` (CLEAR = stage 1, TUNE,
  CONFIRM = stage 2, REPORT, STRING CHECK — EMPTY, EXITED, STOPPED), `TuneButton`, `TuneView`, `TuneFacts`,
  `TuneReport`, `tuneView()`, `tuneHit()`, `tuneLabelCols()`, the `tune_layout` rows; over `ui/touch_panel.hpp`'s
  budgets (HA-136).
- NEW `include/shulib/mechanism/tune_program.hpp` — `TuneProgram` (PROS-free): `bootBanner()`, `run()`, `step()`,
  `enter()`, `copy()`; `TuneIo`, `TuneDevices`; the pure `tuneTargets()`, `tunePositionBound()`,
  `tuneReportFrom()`, `tuneTestLabel()`; the `kTune*` constants (the INVENTED ones labelled so).
- NEW `include/shulib/hal/pros/tune.hpp` — `ProsTuneShell` (ProsDeviceProvider with `/usd/tune_log.txt` in
  `ProsFileMode::Append`, ProsBrainScreen, the master ProsController, `ProsTuneIo`), `tuneShell()`,
  `tuneInitialize()`, `runTune()`; a static_assert of the three competition bits against PROS's.
- NEW `src/programs/tune_program.{hpp,cpp}` — `shulib::bench::tuneInitialize()` / `runTune()` over `kMechanisms`,
  and the static_assert backstop `kProgram != Program::Tune || kMechanisms.count > 0`.
- The build: `Makefile` (MECHANISM_ROBOTS:=tank; the ROBOT-generic `PROGRAM=tune` row; the unknown-PROGRAM list),
  `src/build_selection.hpp` (the guard, `Program::Tune`, the tank-tune branch — beacon
  `shulib-robot-variant=tank-tune`; the switch list and "six tank builds" comments), `src/main.cpp` (a Tune case
  in all five entry points; the header's program table), `tools/src_build_gate.py` (the tank-tune row in every
  table; step 0's `[tune-row]` check; step 8's live-tuning half — symbols AND precondition bytes, no chassis in
  Tune, a chassis in each match program; self-test cases 42-48), `.github/workflows/ci.yml` (two comments: eight
  builds).
- Tests: NEW `test/tune_panel_test.cpp` (5 cases), NEW `test/tune_program_test.cpp` (16 cases),
  `test/build_selection_test.cpp` (+2 cases, one comment word). Suite green — 3086 cases (base 3063, +23).
- Found while building, fixed before the tests were final: the screen's STOP was greyed for 0.5 s after GO
  (the arming delay) — STOP is now never greyed (a reflexive tap on it only stops); a stick jog left running
  when a tap changed the mechanism (its release would have stopped the NEW mechanism) — any tap but STOP/EXIT
  ends a jog first, and TEST/HOME are refused while the stick is pushed; step 8's symbol check alone MISSED a
  planted LiveTuning use (`-Os` inlined every member: self-test case 44 red on its first run, 2026-11-11
  ~00:40) — it now also matches the bytes of LiveTuning's precondition messages (`LiveTuning::…`, which survive
  inlining; no other header writes that text in a string); a TB4 independent computation that used the plant's
  START angle as the zero (the lift sags 0.4 in onto its stop before READY; a rests-at-stop lift is zeroed where
  it is at bind) — the test now takes the plant's angle at the READY tick.

### 3. Tests (ADVERSARIAL) and the mutation each turns red — OBSERVED (2026-11-11 00:51–00:54, `mut_run1.log`)
Driver `.claude/coordinator/scratch/tune1-b1/mutate.py` (texts in `muts.json`, written once): backup under
`scratch/tune1-b1/mut/`, ONE exact replacement with the line count kept (the API pages stay fresh — no re-cut was
needed), the suite binary removed, the briefing regenerated over no binary, `cmake --build` (make's header
dependencies recompiled BOTH TUs that include the mutated headers — `tune_panel_test.cpp.o`,
`tune_program_test.cpp.o`, the only includers, each log shows both), the named tests under `ulimit -v 4194304;
timeout 600`, the file restored in `finally` and on SIGTERM/SIGHUP/SIGINT with its timestamp bumped. Every
verdict below is a TEST ASSERTION failing (no kill, no timeout, no build break):
- **TB1** (`tune_panel_test.cpp`, every page × kind × units × 9 numbers incl. NaN/inf × busy × built, the
  23-character name, a sparse variant) — **MB2** EXIT's rect to x 480 → `onScreen(x.rect)` red.
- **TB2** (each of the 12 buttons mid-TEST: CANCELLED, 0 V, Hold, in ONE tick; a held button refuses TEST) —
  **MB1** the button loop stops before A → red at the CANCELLED / 0 V checks.
- **TB3** (a refused [−] on settle shows its sentence and changes nothing, byte for byte; during a test only
  STOP is armed; a tap on [+] is not read) — **MX7** STOP greyed while arming → red (with the panel's
  arming case).
- **TB4** (the report vs. the test's own computation from the PLANT and the fake motors, three plant
  mismatches × up and down; a real overshoot seen; 100 hold samples) — **MB3** overshoot from the start → red;
  **MX5** settle from the first sample → red.
- **TB5** (robot two's own table: only ports 6, 7 and line A asked for, through READY, a TEST, COPY, ‹ ›,
  EXIT; robot two's drive ports by name) — **MB4** (re-cut of "the program builds the Robot" into what it
  would observably do first: `make()` also asks the provider for port 1) → `motorRequestCount(port) == 0` red.
- **TB7** (a disconnect mid-TEST and VEXos DISABLED each release in one tick; `step()` false; re-entry
  carries on; TEST, HOME, STRING CHECK each refused with no controller and with a field, the sentences pinned
  whole) — **MB6** the dropped controller not a cause → red; **MX1** no stop on DISABLED → red; **MX6** the
  field refusal dropped → red (the sentence).
- **TB8** (nothing asked before READY — motors, lines, inputs — however long it waits; READY makes the claw's
  line once; the first motion waits for GO; BACK moves nothing; asked once per mechanism) — **MB7** the
  mechanisms made at the first tick → red; **MX2** stage 2 skipped → red.
- **TB9** (the limit = the test's own trapezoid × 2 + settle + 0.5; TIMED_OUT AT it within one tick on a
  blocked plant; EXACTLY 10.0 s on a slow live top speed) — **MX3** the plan factor 1 → red.
- **TB10** (EXIT writes back an ADDED cap (to the 2.5 A default) and a LOWERED stated one; leaving a page
  writes back a stated one; RESET; the fake's write counts) — **MB8** EXIT writes nothing → red; **MX4**
  leaving the page writes nothing → red. **TB10b** (the shell over the PROS shim: two boots' COPY blocks both
  in `tune_log.txt`, Created then Appended) — **MX9** Truncate → red.
- **TB11** (a piece on the plant: hold 0.85 A over a 0.6 A signature → the warning, in the log and on the
  REPORT page; none on a light hold; a 0.6 A cap at the peak → the cap warning) — **MX8** the stall warning
  never set → red.
- **TB6 / MB5** (the gate): self-test case 44 plants a LiveTuning use in the competition program. MB5 — the
  non-Tune branch of `tuning_problems()` reports nothing — run as case 44 alone
  (`scratch/tune1-b1/case44.py`, real builds): the planted build PASSES step 8 → case 44 RED; restored (the
  negative control, the real competition build, PASSES with the check in).
- **TB12**: the gate's self-test cases 45 (`make ROBOT=bench|xdrive PROGRAM=tune` → the Makefile's $(error),
  nothing compiled), 46 (MECHANISM_ROBOTS out of step with the table file), 47 (a robot named in the tune row),
  48 (the C++ guard behind a wrong list); the host half in `build_selection_test.cpp` (MECHANISM_ROBOTS ==
  the robots whose `mechanismTableFor()` is not empty; no `$(ROBOT)` comparison in the row).
- Also: COPY (0 lines on an untouched mechanism; `.tuning.kP = 0.3125` after one [+]); the stick (deadband,
  stage 2, 6 V, holds when let go, a range change refused while it moves, TEST refused while pushed); a tap on
  › ends the jog on the OLD mechanism first; EXIT / TUNE AGAIN.
- After the runs: `grep` of every mutant text in the three headers → 0; `git diff -- Makefile src/` carries no
  plant.

### 4. Deltas from the brief (against base 662f252), each with its reason
1. **The ROBOT-generic row's `$(error)` needs a list:** a Makefile cannot read C++, so `MECHANISM_ROBOTS:=tank`
   (beside the ROBOT block) names the robots with mechanisms; the tune ROW names none. Drift is closed three
   ways: the gate's step 0 `[tune-row]` derives the set from `mechanismTableFor()` and the empty tables and
   fails a mismatch; the host test does the same from the tables themselves; `src/build_selection.hpp`'s guard
   and `tune_program.cpp`'s static_assert stop a Tune build of a robot with no branch / no mechanisms. **What
   robot one's G1 adds for Tune** (no Tune file changes): its ROBOT value in `MECHANISM_ROBOTS`, a
   `<robot one> && SHULIB_PROGRAM_TUNE` branch in the chain (the one-chain rule makes every branch name its
   robot) and its define in the guard, and the gate's rows for a `<robot>-tune` build — the same rows it adds
   for every program.
2. **The task:** Tune runs on PROS's OPCONTROL task (the standard Calibrate's pattern, which §1.3 names), not a
   task of its own: the host PROS shim cannot construct a task, and Tune moves nothing unless driver control
   runs with no field — on a disable or autonomous it releases everything and returns; the program lives in
   static storage and carries on when PROS calls opcontrol() again (`enter()` runs the service's driver-entry,
   clearing a flag a deleted task left).
3. **The stick (NEW, not in §1.3's button list):** D8 (a) says "a student jogs it to the right height and reads
   the number off", and nothing on the page could. The LEFT stick (never a button — every button is a STOP)
   jogs the shown position/power mechanism at ≤ 6 V (INVENTED), deadband 0.15 (INVENTED), behind stage 2.
4. **STOP is never greyed** (the arming delay exempts it; found by TB3 — the page a test starts on must be
   stoppable at once).
5. **Step 8 checks EVERY non-Tune build**, not only the three match programs §1.3 names (a superset); and it
   reads precondition BYTES as well as symbols (case 44's finding above). The "no chassis" half has a positive
   control: Drive, Competition and the Auton Tester must each link a chassis symbol.
6. **Leaving a page restores the cap by `LiveTuning::apply(file's cap)`** (an undoable change); if refused (a
   live signature above the file's cap) the page change is refused with the sentence ("COPY, then RESET").
   EXIT falls back to RESET when that apply is refused. The program keeps no limits of its own: the
   library's re-derive writes the motors (MECH-S-A's path, TUNE1-A's delta 11).
7. **TEST targets beyond the brief's position rule** (INVENTED, labelled): a flywheel's first/last preset or ½ /
   9⁄10 of its top speed, its spin limit 5 s, stopped after its report; a power mechanism's ±maxVolts for 0.5 s
   (limit 1.0 s). HOME's limit 10 s (the TEST cap reused). Position A/B: the FIRST and LAST preset (the brief
   says "two presets").
8. **The report's hold current is the MEAN** over the second after the verdict (the brief: "the HOLD current
   over the second after it settles"); a hold that ENDS inside that second (a stall signature tripping on it —
   V1's case) is reported as such with the stall warning, hold n/a.
9. **Log tag `TUNEPG`** for the program's own lines (the COPY block keeps LiveTuning's `TUNE`, so TUNE1-B2's
   import reads only the block).
10. **Files beyond §1.3's list:** `docs/api/service.md` regenerated — the BASE's page was one line stale against
    its own `service.hpp` (every change is a `:814 → :815` line number; TUNE1-A's area — nothing of mine);
    `docs/guide/16-…` (one sentence: slot 7 is Tune's now, where an old brain may hold the old tester) — a 6th
    public document over rule 7's ~5, a one-sentence factual fix. `test/CMakeLists.txt` NOT touched (globbed).
11. **No HA id** (§4 assigns none to TUNE1-B1); the program cites HA-220 (the cap stays in the motor — MECH-S-A's
    row) and HA-136 (the budgets); HA-220's "Settled by" could now name Tune (Step 0.4) — MECH-S-A's row, not
    edited.

### 5. Where this run DEPENDS on TUNE1-A's and MECH-S-A's builds (in review — re-check at the merge)
TUNE1-A: `LiveTuning` (ctor over the declared table, `table()`, `declared()`, `count()`, `spec()`, `attach()`,
`changed()`, `apply()`, `step()`, `undo()`, `reset()`, `copyBlock()`, `undoDepth()`), `Applied` (`ok`, `why`,
`sentence`), `Tunable::CurrentCap/StallAmps/MaxSpeed/RangeMax`, `parametersOf()`, `Parameter` (`id`, `name`,
`rule`), `StepRule`, `tunableValue()`, `tunableUnit()`, `detail::liveStall()`, `liveRefusalText()`,
`LiveRefusal::SettleIs0/HeldButtonMoving`; `PositionSlot::holding()` / `heldButtonMoving()`; the refusal
sentences' words the tests pin (`'<name>': TEST A refused -- …` is mine; the settle refusal text is read from
`liveRefusalText()`); the cap-removed write-back to the 2.5 A default (TB10's "added cap" case). MECH-S-A:
`Devices::currentLimits`, the FakeMotor `currentLimit()` / `currentLimitWrites()`, the plant's limit, `.currentCap`.
~~**A finding for TUNE1-A's fix round:** a cap cannot be ADDED by presses …~~ **WITHDRAWN by the review
round (§10, C3): FALSE** — no cap reads as the motor's 2.5 A (`live_tuning.hpp` :733): [−] from none sets
2.45 A (accepted), [+] asks 2.55 A (refused as above the motor's most). Nothing for TUNE1-A to fix.

### 6. Resumed (2026-11-11, `date`: Wed Nov 11 02:00 EST) — the first run was STOPPED at ~00:57 (the team lead stopped all work ~00:30, usage credits low; resumed ~01:30)
- Found: the build, the tests and the mutations (§2–§3) done and logged; documents edited but NOT logged (no
  prose-review entry yet); the final gate run not recorded (`scratch/tune1-b1/gate_check_final.log` empty).
- Checked before trusting anything: `git diff -- Makefile src/` carries NO self-test plant (`self-test plant`,
  `self_test_directive_form`, `gate1_selftest`, `SelfTestUnhandled`, `tunePlant` — 0 hits; the strings appear
  only in `tools/src_build_gate.py`'s own self-test cases, as they must); every mutation backup restored (the
  coordinator's audit; each mutant text grepped again below); the suite binary removed and rebuilt from source.
- Rebuilt from source (the suite binary removed; the three test objects that include this run's headers —
  `tune_panel_test`, `tune_program_test`, `build_selection_test` — deleted and recompiled): `suite_run.py green`
  — 3086 cases, as §2 recorded.

### 7. The resumption's work (2026-11-11 02:00–02:30)
- **Found missing and ADDED — §3.4's "every new log line's format in guide 11 equals the code's (a test pins
  it)":** the first run wrote guide 11's `TUNEPG` entry but no test held it. NEW case in
  `test/tune_program_test.cpp` — "Tune: EVERY TUNEPG line guide 11 quotes is one the program logs" — runs
  the program through the boot, READY, a change, TEST B to 'stack 3' and its report, every STOP cause, a
  refusal, a cap written back, EXIT, the three warnings and a HOME run to SUCCEEDED, and holds each of guide
  11's 12 quoted lines (verbatim in the guide, backticked) to a logged line of the same SHAPE — names
  (`'lift':`) → 'N', the beacon → [B], numbers → # — every word, separator, unit and plural the
  program's; the two quoted with "…" up to the cut; and every STOP cause the guide names. Writing it found
  **(a)** the guide's cause list lacked `autonomous started` (the program logs it) — added to guide 11 and
  held; **(b)** no test ran HOME to its verdict (the stall-homed fixture `mid home lift` TIMES OUT on the
  host test plant: its −2 V push draws 0.42 A, under its 0.6 A homing signature — a fixture fact, not a
  program bug; the program reported `HOME TIMED_OUT in 10.01 s, its push's peak 0.42 A`, which is the number
  a signature is sized by) — the new case runs a cascade homed by stalling DOWN at −4 V (robot one's plan)
  to SUCCEEDED.
- **Found untested and ADDED — delta 8's "a hold that ENDS inside that second":** NEW case "Tune: a hold
  that ENDS inside the second after the move settled …" (a 0.3 s stall signature at 0.6 A under a piece's
  hold): the move SUCCEEDED, the hold STALLED inside the second → hold n/a (−1), the stall warning set, the
  ENDED line logged once and the mean's warning NOT logged.
- Mutations (driver `mutate.py`, texts in `muts2.json`, `mut_run2.log`, 02:06:18–02:07:02; both including
  TUs recompiled in each log; every verdict a TEST ASSERTION, no kill, no build break; each file restored —
  the mutant texts grepped after: 0):
  - **MG1** the TEST line's "time limit" → "limit" → the guide-11 case red (`loggedWhole`).
  - **MG2** the screen STOP's cause "STOP on the screen" → "STOP on screen" → the guide-11 case red (the
    cause check).
  - **MH1** the cut hold's `holdAmps = -1` dropped → the hold-ENDED case red (`r.holdAmps < 0.0`).
  - **MH2** the ENDED line never written → the hold-ENDED case red (2: the line missing; the mean's
    warning written instead).
- Documents finished: guide 18's Tune section — one paragraph re-wrapped (three over-long lines; no word
  changed); guide 11 — `autonomous started`; `build-order.md` :855 (the count's bracket: TUNE1-B1 built
  2026-11-11) and `HANDOFF-remaining-work.md` :685–688 (the 2026-11-10 note: TUNE1-B1 built; TUNE1-B2
  after the panel) — §3.2's internal list, which the first run had not reached. NOT touched, with why:
  `docs/hardware-assumptions.md` (no HA id is this run's; HA-219's entry :1785 still says "*Settle:* the
  Tune program (a later part of MECH-S/TUNE1)" — now built; MECH-S-A's row, FOR THE COORDINATOR);
  `roadmap.md`'s "You are here" and `build-order.md`'s `**Next:**` (the coordinator's pointer moves at the
  merge); the glossary (§3.1 gives TUNE1-B1 no glossary term).
- **The documents this run changed** (rule 3; public: 6 — a 6th over rule 7's ~5, delta 10): guide 18 (the
  Tune section; the intro; "where the hold and the peak come from"; live tuning's opening), guide 07 (the
  program list), guide 11 (the `TUNEPG` lines), guide 16 (slot 7, one sentence), README (two program lists),
  the changelog (top of `## API 2.5`); generated: `docs/api/{tune_panel,tune_program,pros-tune}.md`, the API
  README and index, `docs/api/service.md` (delta 10), `mkdocs.yml` (three nav rows); internal: this log,
  `REMAINING-WORK.md` row 20f, `roadmap.md` (a Now paragraph), `build-order.md`, `HANDOFF-remaining-work.md`,
  `PROJECT-BRIEFING.md` (the tool's); `ci.yml` (two comments).

### 8. The prose review (rule 4) — the greps RUN (2026-11-11 ~02:20) before this was written, their hits read
- §3.4: `simulator` in the added lines and the new files → one hit, the test file's own house rule ("never
  'simulator'"); `internal/|chunks/|RESUMING` in docs/guide, docs/cookbook, README → empty;
  `HA-(219|22[0-7])` in this run's files → HA-220 only (tune_program.hpp :42, the test :776 — MECH-S-A's
  row exists); probe names (`PROBE_|p_string|p_cap|SlackWatch`) in include/shulib, test → empty; guide 11's
  lines → held by the new case (§7).
- Per changed source file, its name and concepts in docs/ and README: `seven builds|five programs|five
  slots|six tank|five tank` → roadmap :353 and :2010 only, both dated history; `slot 7|shulib Tune|Tune
  program|PROGRAM=tune` → guide 18, 16, 07, 11, README (all this run's, read: current) and
  hardware-assumptions :1785, :1828, :1840 (HA-219/222/223's "Settle" lines name the Tune program as the
  instrument — true; :1785's "a later part" is stale, flagged above); `valid values|PROGRAM=<each>` in the
  guides and README → guide 16 :52, 17 :27, 07 :144–145 (each its own program's line, current);
  `live tun|on the brain screen` outside guides 18 and 11 → guide 07 :147 and README :334 (this run's).
  `MECHANISM_ROBOTS`, `tank-tune`, `[tune-row]` → no public doc names them (the Makefile, the gate and this
  log only; guide 18 says "a robot with no mechanisms is refused by `make`, naming why" — true).

### 9. Gates — the final tree (2026-11-11 02:09–03:20; every test binary under `ulimit -v 4194304` + `timeout`)
- `python3 tools/suite_run.py green` → `suite run: green — 3088 cases, 3317253 assertions` (base 3063: +25 —
  §2's 23 and §7's two), on a binary removed and relinked after the src gate.
- `ctest -R compile_refusal` → `100% tests passed, 0 tests failed out of 258`.
- The ARM header gate (every `include/shulib` header, 245, in ONE TU, `-Werror`, cortex-a9) → compiles clean.
- The CI guards (`pros/` outside `hal/pros/`; `shulib/sim/` outside `sim/`) → both empty.
- `python3 tools/src_build_gate.py self-test` → `src build gate self-test: OK (289 detector cases, each against
  a real build)`; `check` → `src build gate: PASS (all 8 builds link; … live tuning only in Tune, Tune with no
  chassis; …)` (`scratch/tune1-b1/gate_selftest_final.log`, `gate_check_final.log`). After them `git diff --
  Makefile src/` carries no plant (`self-test plant|self_test_directive_form|gate1_selftest|SelfTestUnhandled|
  tunePlant` → 0) and `git status` shows no vendored file changed (the self-test left
  `include/liblvgl/core/lv_obj_style.h` with a new timestamp, its content unchanged).
- §7's by hand: `rm -rf bin .d && make -s ROBOT=tank PROGRAM=competition` → its beacon in the ELF; `nm -C
  --defined-only | grep -c 'LiveTuning\|tune_program'` → **0**, the `LiveTuning::` precondition bytes → 0;
  `PROGRAM=tune` → 26 LiveTuning symbols, 0 `shulib::chassis::`. (A FIRST try without `rm -rf bin .d`
  read 26 in "competition": make had relinked nothing after the gate's last build, Tune's — the package was
  Tune's (its competition beacon count 0). Exactly the stale-`bin/` trap guide 18 warns of; the clean
  build is the result.)
- `api_doc_tool.py` self-test OK; check-coverage `5878 public entities across 202 headers, all documented`;
  check-fresh rc 0; check-examples `654 quoted lines, all verbatim`; check-removability `no public doc
  references docs/internal/`. No `///` comment changed in the resumption (tests and documents only).
- `doc_staleness_audit.py` → clean (32 live docs); `briefing_status.py check` → generated block current
  (regenerated over no binary before the build, and again after it).
- `git diff -U0 HEAD` of `hal/motor.hpp`, `chassis/routine.hpp`, `chassis/chassis.hpp`, and TUNE1-A's
  `live_tuning.hpp`, `service.hpp`, `control/pid.hpp` → no line; `bench_r3a.*` and `test/CMakeLists.txt` →
  untouched; `src/` → `build_selection.hpp`, `main.cpp` and the new `programs/tune_program.*` only.
- `tools/robot_tool.py` does not exist at this base (G3a-P1 not merged here) — not run.

### 10. TUNE1-B1 review findings and their disposition (2026-11-11, `date` at start: Wed Nov 11 03:08 EST)
Three reviewers (correctness, tests, docs) returned 27 findings. Each was RE-CHECKED against this tree
(the reviewers' probes in `scratch/tune1-b1/review-*` read, the code read at the line named) before any fix.
Overlaps are folded: the cap write-back family (C2, D2, T-refused, C5) is fixed at ONE root (below, 10.2).
**All 27 are real; 25 fixed at the root; 2 are the coordinator's (D-HA219, and the ruling on D-generic —
documents corrected meanwhile).** Nothing below claims a robot: host test plant only.

#### 10.1 Correctness
- **C1 (major) — a hold cut short / stopped drew "no hold: it did not settle" and "stall 0.60 A not above
  the hold -1.00 A".** REAL (read: `finishTest` set −1; the REPORT view printed the sentinel). FIX: `TuneReport`
  gains `holdEnded` / `holdStopped`; the page says `peak … A; hold ENDED (the stall tripped on it)` +
  `WARNING: stall … A trips on the hold: raise it`, or `hold n/a: stopped in its second`, or `(no hold: the
  move did not succeed)`; the −1 is never printed as a number. Tests: the hold-ENDED case now reads the
  DRAWN page (no "-1.00", no "did not settle"); "a STOP inside the hold second" (page + flags); a pure panel
  case over every hold reason.
- **C2 (major) — EXIT silently RESET every live number when a cap could not go back; TUNE AGAIN said "kept".**
  REAL (PROBE2 reproduced in the review case below). FIX (10.2): EXIT first COPIES every mechanism changed
  since its last COPY, then restores caps; a RESET it needs is logged (`EXIT: 'x': RESET to the robot file's
  numbers -- … (its numbers were COPIED above)`), counted on the EXITED page (`N RESET to the file: a
  signature >= its cap.`) and in TUNE AGAIN's sentence ("… but N mechanism(s) RESET at EXIT (each one's block
  is in the log)"). Not "refuse EXIT": a refused EXIT leaves a raised limit in a motor (HA-220), the worse
  hazard; copying first loses nothing. Test: "EXIT with a cap that CANNOT go back alone …" (kP tuned, the
  block BEFORE the RESET line, the motor at the file's 0.6 A; and the same on a mechanism not shown).
- **C3 (major) — guide 18 and §5's "finding for TUNE1-A" were FALSE: [+] from no cap asks 2.55 A (refused),
  [−] gives 2.45 A** (`live_tuning.hpp` :733, unchanged at 662f252 — I misread the step rule). FIX: guide 18
  says "press −: the first press sets 2.45 A; + from none … is refused"; the TUNE page's rule line for an
  unstated cap says `file: none; - starts a cap at 2.45 A` (`TuneParamFacts::capNone`); the test comment
  corrected; **§5's finding for TUNE1-A is WITHDRAWN** (struck below). Test: "a cap the robot file does not
  state STARTS with [-] at 2.45 A …" (the page line, [+] refused, [−] → 2.45 A in the motor). The guide
  sentence now DEPENDS on TUNE1-A's step rule — added to §5.
- **C4 (minor) — READY made the mechanisms (a pneumatic MOVES) with no controller / with a field.** REAL
  (`press(Ready)` unconditional). FIX: READY greyed with why in red (no controller, a field or switch, a
  button held — `TuneFacts::buttonHeld`), CLEAR shows the status lines; `press()` refuses too (a touch read
  on the tick the controller drops: `READY refused -- … ; nothing was made`). Test: "READY makes NOTHING …"
  (three causes + the backstop; the request log stays EMPTY).
- **C5 (minor) — UNDO on another page re-wrote a cap a page-leave had put back; the safety line looked only
  at the shown one.** REAL. FIX: the invariant "only the SHOWN mechanism carries a live cap": an UNDO that
  reaches another mechanism's cap going back puts it back at once and says so (`'x': UNDO reached its cap
  going back when you left its page -- it stays the file's (change it on its page)`) — UNDO stops there on
  that page (LiveTuning has no accessor for the top record's mechanism; TUNE1-A's file, not edited); the
  safety line reads `TuneFacts::anyCapChanged` (EVERY mechanism). Test: "an UNDO that reaches ANOTHER
  mechanism's cap …" (motor 2 stays 1.5 A; a cap changed on a mechanism not shown → the safety line).
- **C6 (minor) — long status sentences cut on the two 50-column lines.** REAL. FIX: the page shows the
  status WITHOUT the shown mechanism's leading `'name': ` (the title has it; `screenStatus()`); the
  can't-leave sentence is short and whole on screen (`can't leave: a signature >= the file's cap 1.50 A /
  -- lower it, or RESET (your numbers are COPIED)`); the log keeps the whole story in lines within the
  log's 200 bytes (LiveTuning's own sentence on its own line). COPY's sentences reworded to fit
  (`'x': COPY -- N lines to the terminal and /usd/tune_log.txt: put them in the robot file`). LIMIT: a
  LiveTuning refusal longer than ~100 characters after the name is still cut on screen (TUNE1-A's
  sentences; the terminal has them) — the rule words now come first. Tests: COPY's two drawn lines pinned;
  the can't-leave page lines pinned.
- **C7 (minor) — a power TEST's report said "never settled" / "no hold: it did not settle".** REAL. FIX:
  a power run's REPORT has no settle and no hold line (`peak … A`). Tests: the panel's power case; "TEST A
  and TEST B on robot two's claw pivot …" reads the drawn page.
- **C8 (minor) — REPORT put AGAIN where TUNE's STOP was.** REAL. FIX: REPORT has STOP in the SAME cell
  (x 109–349), never greyed (stops, back to TUNE); AGAIN 0–89, BACK 369–479. Tests: the panel case sweeps
  TUNE's STOP hit box over REPORT (only STOP or nothing); "AGAIN repeats the SAME test; the REPORT page's
  STOP …" taps it on the report's first frame.
- **C9 (minor) — a stick held over ‹ › jogged the newly shown (confirmed) mechanism.** REAL. FIX: the stick's
  EDGE is tracked on EVERY page each pass and a jog STARTS only on a fresh push (centred, then pushed) —
  also closes a stick held through a button stop, GO, TUNE AGAIN or another page. GO's sentence says "let
  the left stick centre, then push it". Test: "a stick HELD while > changes the mechanism …" (two homed,
  both-confirmed lifts).

#### 10.2 The cap write-back family (C2, C5, T-refused, D-copy-first) — one root
The tuned cap left the live numbers when a page was left or at EXIT, unseen. Now `restoreCap()` COPIES the
mechanism's numbers first whenever no COPY carries them (`uncopied(i)`: changed, and not byte-equal to what
its last COPY wrote — `copiedAs_`), says so in the status, and only then applies the file's cap; a refused
page-leave keeps the page with the short sentence; EXIT copies every uncopied mechanism, then restores (RESET
fallback, counted). A delta against D12 (b) (logged as delta 12): the write-back is still "on leaving a page
and on EXIT"; what is new is the COPY before it and the refusal's wording.

#### 10.3 Tests
- **T1 (major) — the stick on a POWER mechanism and after EXIT untested (R06, R13, R23).** FIX: "on a POWER
  mechanism (robot two's claw pivot) the stick drives it at 6 V and STOPS it in the tick it is let go; on one
  whose most is 4 V, at 4 V" (also R07); "the stick moves NOTHING on the EXITED, REPORT and CONFIRM pages".
- **T2 (major) — the refused write-backs untested (R11, R24).** FIX: the two EXIT/leave cases of 10.1 C2/C6.
- **T3 (major) — HOME barely tested (R05, R16).** FIX: "HOME from MID-TRAVEL on a stall-homed cascade" (the
  logged peak == the most current the TEST's own recording saw, formatted the same; homed; then TEST B and
  COPY in the same session — the cascade's bench flow); "a HOME whose push never trips … TIMED_OUT AT
  kTuneHomeSeconds, within a tick".
- **T4 (minor) — R01, R09, R18.** FIX: TB4 asserts no cap warning (and no line) on the uncapped cascade;
  the STOP-in-hold case; the AGAIN case.
- **T5 (minor) — R12, R08, R15, R07 (and R02).** FIX: "tuneTargets for EVERY kind" (pure: two presets, one,
  a flywheel's, a power run's ±, a 4 V pivot); the claw's TEST A +12 V / TEST B −12 V, both SUCCEEDED; "a
  FLYWHEEL's TEST … STOPPED after it" (kFlyAndPivot: 0 V and not spinning after the report).
- **T6 (minor) — equality edges (R03, R10).** FIX: a pure report case with the hold's mean EXACTLY the
  signature (warns) and 1e-7 above (does not), the cap EXACTLY the peak; the stick at EXACTLY
  `kTuneJogDeadband` (nothing) and 1e-4 past it (stage 2 asks).
- **T7 (minor) — gate step 8's other detectors unexercised.** FIX: self-test case 49 (pure, synthetic ELF
  names/bytes): Tune linking a chassis / a Robot; Tune with no live tuning (and the precondition BYTES
  counting); Competition / Drive carrying a LiveTuning member / its bytes; each match program with no
  chassis; the clean ones and Calibrate PASS.
- **T8 (minor) — [tune-row] missed `$(filter tank,$(ROBOT))` and a quoted ifneq.** REAL (reproduced: both
  missed). FIX: the row may read `$(ROBOT)` (or `${ROBOT}`) ONLY in the one list filter and its `$(error)`'s
  words; case 47 now plants the filter, quoted and `${ROBOT}` forms (each FAILS step 0); the host test reads
  the row the same way.
- **T9 (minor) — `build_selection_test.cpp` hard-coded the robot list (G1 would turn the suite red).** REAL.
  FIX: `makeValueOf(RobotId)` — a case per robot, no default (-Wswitch, an error under -Werror), the loop
  over every RobotId value — so G1 gets a compile error naming the line to add, never a silent miss; added
  to G1's list (§4 delta 1, amended below).
- **T10 (minor) — MB5 not reproducible from the logs.** REAL. FIX: `scratch/tune1-b1/gate_mut.py` runs a
  gate mutation's named self-test case blocks VERBATIM (cut from the mutated gate's own `do_self_test()`, its
  own `expect()`), with the exact old/new text in `gate_muts.json` and each log; MB5 re-run below.

#### 10.4 Documents
- **D-generic (major) — the Tune build is not ROBOT-generic in build_selection.hpp or the gate.** REAL as a
  document truth: robot one needs FOUR build lines (MECHANISM_ROBOTS, its chain branch, its guard term, its
  `<robot>-tune` gate build) — the one-chain rule (every branch types its beacon; the team lead's "one chain
  picks both") and the gate's per-build tables make the last three per-robot by design. NOT restructured
  (a generic branch would break the chain rule and step 0's per-build check; CAL1-B is editing the same
  files). FIX: the library header, `src/programs/tune_program.hpp` and the roadmap now list the four lines and
  that the gate fails a robot missing any; §4 delta 1 amended with T9's host test line. **FOR THE
  COORDINATOR: a ruling — accept delta 1, and send G1's brief the checklist (MECHANISM_ROBOTS; the
  `<robot> && SHULIB_PROGRAM_TUNE` branch; the guard term; the gate rows; `makeValueOf`'s case).**
- **D-copy-first (major) — no document said the tuned cap leaves the live numbers.** FIX: 10.2 in code, and
  guide 18's cap paragraph: "COPY before you leave a page whose cap you tuned … Tune COPIES that mechanism's
  numbers to the log first when you have not … write the cap from that block".
- **D-counts (minor) — stale counts in files this run edited; the prose grep was case-sensitive and missed
  src/, the Makefile and tools/.** REAL. FIX: Makefile (PROGRAM list gains tune; "Six programs in six
  slots"; "fourth to eighth entries (…, tank-tune)"; "a build is … `tune` for any robot in
  MECHANISM_ROBOTS"), `src/main.cpp` :47 (six tank builds), `src/build_selection.hpp` :130 (the other SEVEN,
  `…=tank-tune`), the gate's step-6 docstring (`tank-tune`, the other seven). **FOR THE COORDINATOR (§7 —
  not this run's files):** "seven builds" in `src/common/fault_latch_hook.hpp` :17, `src/common/boot_source.cpp`
  :12, `src/programs/xdrive_program.cpp` :35 and :37, `src/programs/auton_tester_program.cpp` :62 (now eight).
- **D-internal (minor) — HANDOFF and RESUMING.** FIX: HANDOFF :303 "eight builds since TUNE1-B1" and shulib
  Tune in its program list; RESUMING :118 (one sentence: `ROBOT=tank PROGRAM=tune` … eight builds — the
  coordinator's protocol file, edited minimally because its gate sentence describes the gate this run
  changed). HANDOFF :656 "(seven builds)" is inside a dated T4 note — left as history.
- **D-HA219 (minor) — HA-219 is stale in TWO places.** CONFIRMED: `docs/hardware-assumptions.md` :457 (the
  table row, "the Tune program (a later part)") and :1785 (the entry's *Settle*). MECH-S-A's rows — **FOR THE
  COORDINATOR at the merge: both, and optionally HA-220's "Settled by" (Step 0.4) naming Tune.**
- **D-copy-each (minor) — COPY copied only the shown mechanism (D9 (a) says each changed).** REAL. FIX:
  `copyChanged()` — the COPY button writes ONE block for EVERY mechanism changed (the shown one's 0-line
  block when none is); a summary sentence when several. Test: "COPY writes ONE block for EVERY mechanism
  changed (D9 (a))". Guide 18 says so.
- **D-guide18 (minor).** (a) guide 18 :25's "still to come: the bench page that runs the direction check"
  is false on main (MECH-A6, 5120dad) — **a MERGE POINT with MECH-A6: take main's sentence and keep this
  run's Tune clause** (not edited here: main's text is not in this base). (b) the cap sentence — rewritten
  (C3), and now listed in §5 as depending on TUNE1-A's step rule.

#### 10.5 Deltas added by the review round (against base 662f252)
12. **Cap write-back with COPY first** (10.2) — a strengthening of D12 (b)'s write-back, and EXIT copies.
13. **COPY = every changed mechanism** (D9 (a) read literally; the first build copied the shown one).
14. **The stick needs a fresh push** for every jog start (C9) — stricter than the first build's edge rule.
15. **READY refused** without a controller / with a field / a button held (re-check #7's "nothing moves
    without a connected controller" applied to READY, which moves a pneumatic).
16. **The REPORT page carries a STOP** (C8). 17. **The status on screen drops the shown name** (C6).
18. **The UNDO cap invariant** (C5). 19. **Self-test case 49** and case 47's three new forms (T7, T8).

#### 10.6 §5 amended — where this run depends on TUNE1-A (re-check at the merge)
Added: `steppedValue()`'s Amps rule (a cap from none: [−] → 2.45 A, [+] → 2.55 A refused — guide 18, the
TUNE page's `capNone` line and the review test pin it); `Applied::why` / `Applied::build` and
`Refusal::AStallSignatureMustBeBelowTheCurrentCap` (the can't-leave sentence's reason); `LiveTuning::undo()`
being global (C5's invariant exists because of it). **WITHDRAWN: §5's "finding for TUNE1-A's fix round"
(a cap cannot be added by presses) — it was false (C3).**

#### 10.7 The review round's mutations — OBSERVED (2026-11-11 03:27–03:38)
Driver `scratch/tune1-b1/mutate.py` (as §3: backup, ONE exact replacement keeping the line count, the suite
binary removed, the briefing regenerated over no binary, `cmake --build`, the named tests under `ulimit -v
4194304; timeout 600`, restored in `finally` and on SIGTERM/SIGHUP/SIGINT, the timestamp bumped). Texts in
`scratch/tune1-b1/muts_review.json` / `muts_review2.json`; results `mut_review_round.log` / `_round2.log`;
per-mutant logs `mut/RV*.log`. EVERY object depending on the mutated file was recompiled (each header mutant's
log shows both includers, `tune_panel_test.cpp.o` and `tune_program_test.cpp.o` — the only ones; the
`build_selection_test.cpp` mutant recompiled it; the two Makefile mutants needed no object — the test reads the
Makefile at run time — and relinked). Every verdict is a TEST ASSERTION failing (no kill, no timeout):
- The reviewers' survivors, now CAUGHT: RV01 (cap `>= 0`), RV02 (stall `>= 0`), RV03 (stall `<` hold — the
  equality case), RV05 (HOME's peak never taken), RV06 (power jog never stopped), RV07 (power stick cap
  dropped — SURVIVED the first run: at full deflection the service's own clamp hid it; the test now also
  checks HALF deflection = 2 V, then CAUGHT), RV08 (flywheel not stopped), RV09 (STOP in the hold → partial
  mean), RV10 / RV10b (deadband 0.3; `>=` — both edges), RV11 (EXIT without the RESET fallback), RV12 (TEST B
  forward), RV13 (power stick drives nothing), RV15 (one-preset target), RV16 (HOME limit 1 s), RV17 (power
  run 5 s), RV18 (AGAIN always A), RV23 (stick on every page), RV24 (a refused leave moves on).
- This round's own code: RVN1 (READY's backstop refusal dropped), RVN2 (READY not greyed when blocked), RVN3
  (REPORT's STOP stops nothing), RVN4 (REPORT's STOP greyed while arming), RVN5 / RVN6 (`holdEnded` /
  `holdStopped` never set), RVN7 (a power report's settle line drawn), RVN8b (a jog starting with no fresh push
  — the first cut RVN8, `if (false)`, BROKE ITS OWN BUILD (-Werror: `edge` unused): re-cut, never counted),
  RVN10 (leaving a page without COPY first), RVN11 (EXIT without COPY), RVN12 (the UNDO cap invariant
  dropped), RVN13 (safety line on the shown cap only), RVN14 (COPY of the shown one only), RVN15 (the screen
  keeps the name — the COPY lines no longer fit), RVN16 (no `capNone` hint), RVN19 (the host test's robot
  map wrong), RVN20 / RVN21 (a `$(filter tank,$(ROBOT))` / quoted `ifneq` row planted in the Makefile → the
  host test RED).
- After the runs: every mutated file's ORIGINAL text present exactly once (scripted); `git diff -- Makefile
  src/` carries no plant.

**The gate's mutations** — `scratch/tune1-b1/gate_mut.py` (T10's fix: the named self-test case blocks run
VERBATIM from the mutated gate's own `do_self_test()`, its `expect()`; old/new texts in `gate_muts.json` and in
each `mut/GM-*.log`; the gate restored after each, `cmp` with the backup clean), 03:36–03:38,
`gate_mut_run.log`:
- GM0-control (a comment changed): cases 44, 47, 49 — no failing expectation (the negative control; case 44's
  real competition builds PASS with the check in).
- **MB5 re-run — GM-MB5** (`for n in tuning[:3]:` → `tuning[:0]:` — the non-Tune branch reports nothing):
  CAUGHT, 4 failing expectations, the first `44: live tuning linked into the COMPETITION program PASSED step
  8` (case 44's real build with the planted LiveTuning), and case 49's two match-program expectations.
- GM-T1 (Tune's chassis detector `chassis[:0]`): CAUGHT by case 49 (`49: a Tune ELF linking
  shulib::chassis:: PASSED step 8`). GM-T2 (`if not tuning:` → `if False:`): CAUGHT by 49. GM-T3 (the match
  programs' chassis control off): CAUGHT by 49 (3). GM-R1 (the [tune-row] `or reads` dropped): CAUGHT by case
  47 (3 — the filter, quoted and `${ROBOT}` forms).

#### 10.8 Gates — the review round's final tree (2026-11-11 03:40–04:20; every test binary under `ulimit -v 4194304` + `timeout`)
- `python3 tools/suite_run.py green` → `suite run: green — 3112 cases, 3695601 assertions` (§9's 3088: +24 —
  the review cases), on a binary removed and rebuilt from source (the briefing regenerated over no binary,
  then again after).
- `ctest -R compile_refusal` → 258 of 258 passed. The ARM header gate (every `include/shulib` header, 245, one
  TU, `-Werror`, cortex-a9) → clean. The CI guards → both empty.
- `src_build_gate.py self-test` → `OK (306 detector cases, each against a real build)` (§9's 289: +17 — case
  47's three forms, case 49's fourteen); `check` → `PASS (all 8 builds link; … live tuning only in Tune, Tune
  with no chassis; …)` (`scratch/tune1-b1/gate_selftest_review.log`, `gate_check_review.log`). After them:
  `git diff -- Makefile src/` → no plant (0 hits of the five plant words); `git status` → no vendored file,
  `bench_r3a.cpp` not modified (the self-test's case-41 plant restored).
- By hand: `rm -rf bin .d && make ROBOT=tank PROGRAM=competition` → its beacon in the ELF, LiveTuning /
  tune_program symbols **0**, the `LiveTuning::` bytes **0**; `PROGRAM=tune` → 26 LiveTuning symbols, 0
  `shulib::chassis::`. `bin/` and `.d/` removed after.
- `api_doc_tool.py` self-test OK; check-coverage `5889 public entities across 202 headers, all documented`;
  check-fresh rc 0 (the API pages regenerated with every build: `///` comments changed in both headers);
  check-examples `654 quoted lines, all verbatim`; check-removability clean. `doc_staleness_audit.py` clean;
  `briefing_status.py check` current. `tools/robot_tool.py` does not exist at this base — not run.
- The brief's diffs: `hal/motor.hpp`, `chassis/routine.hpp`, `chassis/chassis.hpp`, TUNE1-A's `live_tuning.hpp`,
  `service.hpp`, `control/pid.hpp` → no line; `bench_r3a.*` and `test/CMakeLists.txt` untouched; `src/` →
  `build_selection.hpp`, `main.cpp`, `programs/tune_program.*` only.
- The prose review (rule 4), RUN 04:20 before this line: `simulator` in added lines/new files → the test
  file's house rule only; `internal/|chunks/|RESUMING` in docs/guide, README → empty; `HA-(219|22[0-7])` →
  every id an existing row (this run cites HA-220 only); probe names → empty; the stale counts, now
  CASE-INSENSITIVE over Makefile, src/, tools/, include, .github, docs/guide, README, RESUMING, HANDOFF, the
  roadmap (`other six|seven builds|five programs|five slots|five tank|fourth to seventh|robot-two build is`)
  → only the five src/ comments listed for the coordinator (10.4), HANDOFF :659 and roadmap :353 / :2012
  (dated history); `from none is 0.05|[+] from none` → only this round's corrected texts; the old sentences
  (`COPY: 0 lines`, `did not settle`, `your live numbers are kept`) → only tests' BUG-CAUGHT comments and
  the TUNE AGAIN sentence where it is now true (no RESET at EXIT).
- After the gate (its restores bump timestamps): the suite binary rebuilt again, `suite_run.py green` → 3112 cases, 3695601 assertions; the staleness audit then trusted the binary (clean); the briefing current; check-fresh rc 0.
- **Do not commit** (the coordinator commits). Places this round depends on TUNE1-A / MECH-S-A: §5 + 10.6.

### 11. Verification-round findings and their disposition (2026-11-11, `date` at start: Wed Nov 11 07:44:57 EST)
**Resumed.** A first run of this fix round was STOPPED (the 5-hour limit, ~04:40). Its state at 07:45, checked
before any edit: the live log had NO disposition of these findings; `tune_program.hpp`, `tune_panel.hpp`,
`build_selection_test.cpp`, the Makefile and `tools/src_build_gate.py` were each byte-identical (`cmp`) to every
mutation backup of the same size under `scratch/tune1-b1/mut/` and `scratch/tune1-b1/verify/mut/` (the
verifier's) — no mutant left applied, no edit of the stopped run in the tree; `git diff -- Makefile src/` → 0
plant words (the 7 hits in `tools/src_build_gate.py`'s diff are the gate's own self-test strings, the code that
plants and restores); `bench_r3a.cpp` unmodified. The test binary was removed and relinked from source before
any suite result below. So this round starts at the findings, none done.

The fresh verifier returned 4 findings; each RE-CHECKED against the code at the line named (its probes
`scratch/tune1-b1/verify/probe_v.cpp`, `probe_v5.cpp` read). **All 4 are real; all 4 fixed at the root.**

- **V-major — C5's fix did not hold: an UNDO on another page that undid a mechanism's RESET left the raised cap
  in a motor not shown, and said "it stays the file's".** REAL (read: `pressTune`'s UNDO applied the file's cap
  AFTER the undo and set the sentence whatever `back.ok` said; the RESET is an undoable record whose `before`
  carries the raised cap AND the 1.8 A signature, so that apply is refused). ROOT: the first fix let the undo
  happen and then tried to repair it. The verifier's suggestion (COPY + RESET after it) would push a new record
  for every undo popped — the depth never falls, finding V2's loop again. FIX — decide BEFORE the undo:
  LiveTuning's records are private (TUNE1-A's file, not edited), so the program now tracks WHOSE every record
  is: every change it makes goes through `tracked()` / `undoTracked()` (step, RESET, the cap write-back and its
  RESET fallback, undo — the only LiveTuning mutators it calls), which keep `undoWho_` (the mechanism, and its
  current cap BEFORE the change = what undoing it brings back) in step with LiveTuning's own push / oldest-falls-
  off-past-16 / pop — a record is pushed exactly when the change committed (`a.ok` and the spec's bytes moved:
  LiveTuning's `commit()` records exactly those). `undoWaitsOn()`: when the next undo is ANOTHER mechanism's and
  its cap-before is not the file's, UNDO **waits** — nothing changes, one Warn line
  (`'x': UNDO waits on another page -- the next undo would change its current cap (only its own page may change
  a cap): show it (< >) and UNDO there`), status `UNDO waits: next is 'x''s cap -- UNDO it on its page`. The
  after-the-fact re-apply is REMOVED (unreachable now; kept, it would be dead, untestable code). On that
  mechanism's own page the same UNDO runs (the cap SHOWN, the safety line amber), and leaving it then follows
  the page-leave rule (written back, or "can't leave"). Tests: "an UNDO on another page that would undo a
  mechanism's RESET of a RAISED cap … WAITS" (the verifier's exact presses: motor 2 stays 1.5 A, depth and
  tracked depth unchanged, the sentence, the safety line grey; on its page the RESET undoes to 2.2 A and the
  page is then not left); "the program tracks WHOSE every undo record is …" (past the 16: the OLDEST falls off —
  a sequence where keeping the newest instead writes 1.45 A into motor 2 — and a refused [+] records nothing).
- **V-minor — UNDO could never get past another mechanism's cap write-back; the explanation was cut.** REAL
  (the re-apply pushed one record for every one popped: depth stuck at 3; two log lines a press; the 118-char
  sentence cut at "(cha"). FIX: the same root — a waiting UNDO changes nothing (depth stays, ONE log line a
  press), the screen sentence is 64 characters with this name and whole at the LONGEST (23-character) name
  (pinned: `tuneWrap` of the longest form = the sentence). The page does NOT grey UNDO: a greyed button carries no
  why, and the student who presses it must read where to go — the press is what explains it. UNDO stays global
  (D8 (a)): a record of another mechanism that does not touch its cap still undoes from any page (test). Guide
  18 says UNDO waits and how to get past it (its page, UNDO there, come back); guide 11 lists the new log line.
  Test: "an UNDO whose next undo would change ANOTHER mechanism's cap … WAITS" (kP on the cascade FIRST, the
  verifier's V2 presses; three presses change nothing, one line each; the two DRAWN lines pinned —
  `UNDO waits: next is 'mid home lift''s cap -- UNDO` / `it on its page`; then on its page the write-back and
  the [−] undo; back on the cascade its kP undoes to the file's, depth 0, tracked 0).
- **V-minor — a STOP after the stall had tripped on the hold dropped the WARNING from the page.** REAL (read:
  `finishTest` tested `holdStopped` first; `logReport` keyed on `holdCut_`). FIX: `holdCut_` first — the hold
  ENDED and its warning stand, `stoppedBy` is kept (the page's "stopped by" line); `holdStopped` only when the
  hold had not ended. `logReport` now keys on the report's own `holdEnded` (the page's flag), the WARNING on
  `warnStallNotAboveHold` — page and log can no longer disagree. Guide 18: "— even when you press STOP after it
  ended; a stop before the hold ends says the hold is n/a". Test: "a STOP AFTER the stall tripped on the hold
  keeps the hold ENDED and its WARNING on the page …" (the verifier's V1: trip, 0.2 s, a controller button;
  flags, the log's one WARNING, the drawn `hold ENDED`, `WARNING: stall 0.60 A trips on the hold: raise it`,
  `stopped by a controller button`, no `hold n/a`).
- **V-minor — T10's gate mutation driver could run the WRONG mutant through Python's bytecode cache.** REAL
  (`importlib` file loader + `tools/__pycache__`, validated by whole-second mtime and size; GM-T1 and GM-MB5
  are the same length). FIX in `scratch/tune1-b1/gate_mut.py` (the original kept as `gate_mut.py.v1`): the
  mutated gate is COMPILED FROM ITS TEXT (`exec(compile(text, GATE, "exec"))` into a fresh module named as the
  gate file — no import, no cache), `sys.dont_write_bytecode = True`, and `tools/__pycache__/src_build_gate*.pyc`
  deleted before and after each mutation. **For the coordinator's re-check of §10.7: use this form.** Re-run
  07:56–07:57 (`gate_mut_verify_run.log`; the review round's logs kept in `mut/review-round-GM/`):
  GM0-control no failing expectation (case 44's real builds PASS); GM-MB5 CAUGHT (4, first `44: live tuning
  linked into the COMPETITION program PASSED step 8`); **GM-T1, GM-T2, GM-T3, GM-R1 all in the SAME second
  (07:57:40) and each reports ITS OWN expectations** — GM-T1 `49: a Tune ELF linking shulib::chassis:: PASSED
  step 8`, GM-T2 `49: a Tune ELF with no live tuning PASSED step 8`, GM-T3 `49: a tank-competition ELF linking no
  chassis PASSED step 8` (3), GM-R1 `47: a tune row naming a robot as 'ifeq ($(filter tank,$(ROBOT)),)' PASSED
  step 0` (3) — the same-second run that misled the old driver. No `.pyc` of the gate left after; the gate `cmp`
  clean with its backup.

#### 11.1 Deltas added by this round (against base 662f252)
20. **UNDO waits** when its next undo would change another mechanism's cap (replaces the review round's
    re-apply, delta 18) — and `TuneProgram::undoTrackedDepth()` (a host-test accessor; API page regenerated).
21. **A hold that ENDED outranks a later stop** in the report.
§5 / 10.6 amended — this round DEPENDS on TUNE1-A's `LiveTuning` recording EXACTLY the changes that move the
spec's bytes (`commit()`: a change, a RESET of a changed mechanism; an undo pops; a refused or "unchanged"
apply, an "already the file's" RESET and a `rederive()` refusal record nothing) and on `kLiveUndoDepth`'s
oldest-falls-off ring (`push()`). If TUNE1-A's fix round changes either, `tracked()` must follow — the test
"tracks WHOSE every undo record is" turns red on a mismatch (its `inStep()` after every press).

#### 11.2 This round's mutations — OBSERVED (2026-11-11 07:54–07:56, `mut_verify_round.log`)
`scratch/tune1-b1/mutate.py` as §10.7 (backup, ONE exact replacement keeping the line count, the suite binary
removed, the briefing regenerated over no binary, `cmake --build`, the tests under `ulimit -v 4194304; timeout
600`, restored in `finally` and on SIGTERM/SIGHUP/SIGINT, the timestamp bumped). Texts in
`muts_verify.json`; per-mutant logs `mut/VF*.log`, each showing BOTH includers of the header recompiled
(`tune_panel_test.cpp.o`, `tune_program_test.cpp.o`). Every verdict a TEST ASSERTION failing (no kill, no
timeout, no build broken):
VF1 (`undoWaitsOn()` always null — the wait gone): CAUGHT, 25 (3 cases). VF2 (waits on its OWN page too): CAUGHT,
11. VF3 (waits on EVERY other mechanism's record — UNDO made page-local): CAUGHT, 4 (the D8 (a) case). VF4
(`tracked()` records the wrong changes): CAUGHT, 24. **VF5 (past 16 the NEWEST is overwritten instead of the
oldest falling off — the count still in step): CAUGHT, 4 — motor 2 at 1.45 A, by the ring-order sequence**.
VF6 (an undo never pops the tracking): CAUGHT, 6. VF7 (the stop outranks a hold that ENDED again): CAUGHT, 7.
VF8 (the log's WARNING inverted): CAUGHT, 2 (both hold-ENDED cases' log counts). VF9 (two log lines a waiting
press): CAUGHT, 4. VF10 (the tracking's top read at the bottom of the ring): CAUGHT, 19. After the run every
mutated file `cmp`-identical to its backup.

#### 11.3 The prose review (rule 4) — the greps RUN 08:02, before this was written, their hits read
`simulator` in the added lines / new files → only the test file's own house rule and this log's earlier
entries; `internal/|chunks/|RESUMING` in guides 11 and 18 → empty; the OLD sentence (`UNDO reached`, `it stays
the file's`, `puts it back again|at once`) in include, src, docs/guide, the changelog, README, test → one hit,
a test's BUG-CAUGHT comment quoting it; HA ids in the added guide/header lines → HA-220 only (an existing row);
the new screen and log sentences match guides 18 and 11 (guide 11's line is pinned by the test that collects
every TUNEPG line the guide quotes). The changelog's Tune entry gains the UNDO rule and `undoTrackedDepth()`.
Guide 18's cap paragraph was re-wrapped around the new sentences (no other words changed).

#### 11.4 Gates — the final tree (2026-11-11 07:58–08:53; every test binary under `ulimit -v 4194304` + `timeout`)
- First, before the src gate: the binary removed, the briefing regenerated over none, `cmake --build`, the
  briefing again → `suite_run.py green` → `3116 cases, 3696825 assertions` (§10.8's 3112: +4 — five
  verify cases, the review round's C5 case replaced).
- `src_build_gate.py self-test` (07:59–08:36, slowed by another worktree's self-test running beside it) →
  `OK (306 detector cases, each against a real build)`; `check` (→08:42) → `PASS (all 8 builds link; … live
  tuning only in Tune, Tune with no chassis; …)` (`scratch/tune1-b1/gate_selftest_verify.log`,
  `gate_check_verify.log`). After them: `git diff -- Makefile src/` → 0 plant words (with `tunePlant`); `git
  status` → nothing new, `bench_r3a.cpp` and the vendored tree unmodified; no gate `.pyc` in `tools/__pycache__`.
- By hand: `rm -rf bin .d && make -s ROBOT=tank PROGRAM=competition` → LiveTuning / tune_program symbols **0**;
  `PROGRAM=tune` → 26 LiveTuning symbols, 34 TuneProgram, **0** `shulib::chassis::`. `bin/` and `.d/` removed
  after (a `bin/` and `.d/` found at the start of this round, left by the verifier's case-44 runs, went too).
- After the gate (its restores bump timestamps): `build.sh` (binary removed, briefing over none, API pages,
  build, briefing again) → `suite_run.py green` → **3116 cases, 3696825 assertions**; `ctest -R
  compile_refusal` → 258 of 258; the ARM header gate (245 headers, one TU, `-Werror`, cortex-a9) → CLEAN; the
  CI guards → both empty; `api_doc_tool.py` self-test OK, check-coverage `5890 … all documented` (+1:
  `undoTrackedDepth`), check-fresh rc 0 (the API pages regenerated: a `///` comment changed), check-examples
  `654 quoted lines, all verbatim`, check-removability clean; `doc_staleness_audit.py` clean;
  `briefing_status.py check` current.
- The brief's diffs against 662f252: `hal/motor.hpp`, `chassis/routine.hpp`, `chassis/chassis.hpp`, TUNE1-A's
  `live_tuning.hpp`, `service.hpp`, `control/pid.hpp`, `bench_r3a.*`, `test/CMakeLists.txt` → no line. This
  round's files: `tune_program.hpp`, `tune_program_test.cpp`, guides 11 and 18, the changelog, the API pages
  (`tune_program.md`, `all-entities.md`, `README.md`, `mkdocs.yml` — generated), this log.
- **Do not commit** (the coordinator commits). Places this round depends on TUNE1-A: 11.1 (LiveTuning's
  recording rule and its ring), with §5 and 10.6.

## TUNE1-B1 — the coordinator's merge verification *(2026-11-12)*

- **Applied as its OWN diff** (`662f252..` its transport commit — it was built on a snapshot of TUNE1-A's build) onto
  `6baef55` (TUNE1-A landed). Code: one conflict, `src/main.cpp`'s `disabled()` and `competition_initialize()` — CAL1-B had
  given Calibrate its own state screens, so `Program::Tune` joins the bench's shared case and Calibrate keeps its own.
  Documents: guide 07's program list (Calibrate's chapter-19 link kept, Tune added), guide 18's opening, REMAINING-WORK row
  20f (TUNE1-A and TUNE1-B1 LANDED), this log (TUNE1-A's later sections, then TUNE1-B1's), the changelog (Tune's entry at
  the top of `## API 2.6`, dated "2026-11-11 (built 2026-11-11)"; a stray copy of TUNE1-A's old heading the patch carried
  removed); HA-219's settle note ("the Tune program (a later part…)" → landed). No minor bump (no frozen surface).
- **Gates on the merged tree** (`.claude/coordinator/scratch/tune1_b1_merge_verify.sh`; its first run was cut by the team
  lead's pause in the src-gate step, re-run in full 2026-11-12): the suite green; `src_build_gate.py` self-test OK and
  check PASS (**all 8 builds link** — Tune is the eighth); the ARM header gate CLEAN; the doc gates and the briefing check
  clean; the compile-refusal harness 258 of 258; no plant, no stray root file.
- **The coordinator's own mutations** (`muts_tune1_b1_main.json`): **4 of 4 RED** — MV1 a held controller button no longer
  releasing the mechanisms, MV2 a disconnected controller no longer releasing them, MV3 autonomous starting not ending Tune,
  MV4 a TEST move allowed 20 s. The restored tree relinked and ran green.
- **TUNE1-B1 LANDED — "shulib Tune" is on main.** Nothing has run on a robot. Robot one gets Tune from ROB1-A by adding its
  ROBOT to `MECHANISM_ROBOTS`, a Tune branch in the chain and the gate's rows (the gate's `[tune-row]` check enforces it).

## The READY label fix — the coordinator *(2026-11-15)*

- Found by the DOCS3 amendment's screen-picture probe: `TuneButtonView::label` was `char[16]`, so "READY - I AM CLEAR" (18) drew as "READY - I AM CL". Now `char[24]`; `tune_panel_test` pins READY's label whole and every one of the 24 `b.button(...)` label literals under the box (a source pin, so a later label is held too). Gates: suite green (3,753), src gate check PASS (14 builds), ARM header gate, check-examples / fresh / coverage, staleness, briefing. Mutation MT-L1 (the box back to 16) RED.
