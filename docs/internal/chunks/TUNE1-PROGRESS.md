# TUNE1 — live tuning — PROGRESS (live log, appended as the work goes)

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
