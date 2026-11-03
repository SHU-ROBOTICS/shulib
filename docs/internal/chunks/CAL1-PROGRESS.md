# CAL1 — live progress log

> Appended in real time by the executor of each part. Brief:
> [`CAL1-standard-calibrate.md`](CAL1-standard-calibrate.md) — the team lead's answers (2026-10-08):
> D1–D5 as recommended, D6 (a), A2 split into A2a and A2b. A heading per part.
>
> Nothing in CAL1 has run on a robot. "Host test plant", never "simulator".

---

# Part A1 — the seams (a library ADDITION)

> **This run builds Part A1 ONLY** (§9.2's A1 row, §2.A's A1 items): `hal/brain_screen.hpp` (the
> screen seam and its one sanitizer outside Bench Tests), `hal/fake/fake_brain_screen.hpp`,
> `hal/pros/brain_screen.hpp`, `calibration/calibrate_screen.hpp` (the painter and the program's
> screens), `calibration/front_check.hpp` (D6 (a)'s `earlier` may be the typed ports), the shim pieces
> (`screen.h`, `misc.h`, `motors.h`'s three C-API reads, a host `main.h`, `rtos.hpp`'s advance hook),
> and tests 3, 7, 8a and A1's half of 9. NOT A2 (the program, the PROS shell, the panel's additions),
> NOT B (robot two). HA numbers: none (HA-187 is A2's to register).
>
> Executor: **Claude Opus 5.5 (1M context)** (subagent of the coordinator's workflow run), working in
> the git worktree `.claude/worktrees/cal1-a1` (branch `cal1-a1`, from `aaea302`). The executor does
> not commit, push, run `pros`, or touch the serial port. Scratch and backups:
> `.claude/coordinator/scratch/cal1-a1/` (never `/tmp`, never the repository root).

## A1.0 Start (2026-10-08 20:33)

- Worktree clean at `aaea302`. Read RESUMING.md's standing rules and the brief in full (3,171 lines):
  the answers block, every Re-check entry (R1–R18), the D6 (a) entry and its continuation, "The check of
  D6 (a)" (C1–C5) and its continuation, the Decisions (all DECIDED), §0–§9 and the landmines.
- This log was created FIRST, before any code.

## A1.R Resumption (2026-10-15 00:26 EDT, `date`)

- **Why a resumption.** The run that wrote A1.0 was STOPPED by the team lead's pause of all work on
  2026-10-08 at 21:23 and resumed 2026-10-15. Executor of this resumption: **Claude Opus 5.5 (1M
  context)**, the same worktree and branch (`cal1-a1`, from `aaea302`).
- **An honest gap in this log.** The stopped run appended NOTHING after A1.0 (20:33), though it wrote
  every A1 file, ran 21 mutations and regenerated `docs/api/` (file times 20:44–21:21). Its work is
  reconstructed below from the evidence it left — the files' contents and times, its scratch logs
  under `.claude/coordinator/scratch/cal1-a1/` (`baseline_*`, `build1.log`, `suite1-3.log`,
  `mut/mutations.log`, `mut/a1_mutations.json`, `mut/run_mut.py`) — and every result it claims is
  RE-RUN in this resumption on a fresh build, never taken from those logs. Where this log says
  "the stopped run", it means a claim read from its evidence; where it says "re-run", the result was
  produced again here.
- **State found** (`git status`, `git diff -- src/ Makefile`): `src/` and the `Makefile` UNCHANGED
  (0 diff lines) — no gate self-test plant was left by a killed run; no `bin/` or `.d/`. Modified:
  `test/pros_shim/pros/rtos.hpp` (the hook), `docs/api/README.md`, `docs/api/all-entities.md`,
  `mkdocs.yml`, `docs/internal/PROJECT-BRIEFING.md`. New: the five headers, the four shim pieces, the
  three tests, four `docs/api/` pages. NOT yet done: `docs/changelog.md`, `build-order.md`, the prose
  review, the gates' final runs, this log.
- **Mutation backups:** every `mut/M*.bak` was restored by the runner (each run line says
  `cmp with backup: identical`); the coordinator also found them restored. Each file's mtime is its
  last restore (21:08–21:10), after which no source file changed — so the mutation run was on the
  final sources. Still, a killed run may have left a mutant binary linked: this resumption rebuilds
  from source before trusting any result.
- **SER1:** `hal::pros::serialLine()` is NOT at this base (grep of `include/` and `src/`: no hit).
  A1 writes NO serial line (grep of A1's five headers: no `printf`/`puts`/`fflush`; the fake's
  `snprintf` calls only format its record). Hand-over logged for A2: `ProsCalibrateIo::line()` must be
  ONE call to `serialLine()` if SER1 has landed by then.

## A1.1 What the stopped run built (2026-10-08 20:44–21:21) — reconstructed from its files

**The drafts it worked from, and which won.** The brief names three draft trees, newest wins
(`.claude/coordinator/scratch/brief-CAL1/`): `amend/p3r/`, `d6a/p3r/`, `d6a/v3/p3r/`. Compared in
this resumption (`md5sum`, `wc -l`): for A1's five headers ALL THREE ARE BYTE-IDENTICAL
(`calibrate_screen.hpp` 385 lines, `front_check.hpp` 107, `hal/brain_screen.hpp` 124,
`fake_brain_screen.hpp` 106, `hal/pros/brain_screen.hpp` 89) — the D6 (a) work and its check changed
none of them. So every A1 header comes from `d6a/v3/p3r/` (= the other two). The shim pieces have
their own drafts: `screen.h` and `misc.h` from `amend/shim_extra2/pros/` (byte-identical to the
older `shim_extra/pros/`), `motors.h` from `amend/shim_extra2/pros/` (its only draft), the
`rtos.hpp` hook from `amend/shim_extra2/pros/rtos.hpp` (a whole-file draft; applied here as a change
to the repository's own `rtos.hpp`), the host `main.h` from `shim_extra_team/main.h`. **Every probe
switch stripped:** each draft's `#error "probe shim: …"` became the project's shim guard, and the
"PROBE (brief-CAL1) … NOT project code" comments are gone.

**Its deltas from the drafts, each with its reason** (read from `diff` against `d6a/v3/p3r/` and the
shim drafts, comments aside; the stopped run logged none of them). *(Corrected 2026-10-15 by the
review's R4, below: items 5 and 6 first listed, as deltas, words the draft already had — `diff` of
the two headers' `string_view k` lines is EMPTY. They are reworded here, the words moved to the
second list, and nothing else in this section changed.)*
1. `calibrate_screen.hpp` — the helpers in `calibration::detail` are renamed so they cannot collide
   with that namespace's existing members in `calibrate_panel.hpp` / `record_store.hpp` or be misread
   as `src/`'s `bench::printText` (landmine 8): `printText`→`printPanelText`, `sizeOf`→`screenSizeOf`,
   `textColour`/`barColour`/`barTextColour`→`panel…`, `kHeaderWordX/Y`, `kBuildX/Y`→`kPainter…`.
2. `printPanelView()` (new): the draft passed a `std::string_view`'s `.data()` on as a C string. The
   own screens take string views (`paintCannot`'s header, headline, footer), which need not be
   NUL-terminated, so the view is copied into a terminated buffer first (mutation M9 pins it).
3. The header's build text is cut at `kBuildTimeCols` (14, `BUILD hh:mm:ss`): today's program
   `static_assert`s its own literal `"BUILD " __TIME__`; the library's is a runtime argument, so the
   cut is what keeps the layout `static_assert` (moved from the program) true (M10). The draft cut it
   at SMALL's 50.
4. Every own screen (`paintStarting`, `paintCannot`, `paintNotDriverControl`, `paintStateScreen`)
   marks the painter's record stale (`valid_ = false`): they paint over the view, so the next
   `render()` must repaint in full (M8). In today's program they are free functions after which
   nothing renders again; as painter members they need it. The draft did not set it.
5. The competition-state screen's colours are named (`detail::kStateGround/kStateWord/kStateText`),
   the values of `src/common/state_screens.cpp`; the draft wrote them as literals (`0xF0C000`,
   `0x000000`, `0xFFFFFF`, d6a/v3/p3r :296-307).
6. *(Not a delta — see the correction above.)* The general words and the three new refusals are the
   draft's, kept as drafted; they are in the list that follows.
7. `front_check.hpp` — a guard (`detail::frontSidesReadable`): a description whose side `count` runs
   past 21 would make `sideOf`/`signedPortOf` read past `signedPorts`; it now gives UNDETERMINED with
   every count 0. Proven at COMPILE time (F7's `static_assert`s — a constant evaluation that read
   past the array would not compile; M4, M23). The header gained A1's D6 (a) sentence (`earlier`
   may be the typed drive ports; neither description need be `valid()`). `<cstring>` dropped (unused).
8. `fake_brain_screen.hpp` — `rect()` takes `int16_t` (the draft's `int` parameters needed a narrowing
   at every call); two unused members removed. `hal/brain_screen.hpp`, `hal/pros/brain_screen.hpp`
   — comments only (the sanitizer's code is the draft's and `printText`'s, byte for byte; a note on the
   one call-order difference: each `outlineRect` sets its pen, where the program sets it once for two
   outlines — the same pixels).
9. The shim: `screen.h` gained `calls` (every C call in order, `set_pen`/`set_eraser` and
   `print_at`'s FORMAT included — the only way test 8a can see the order and the literal `"%s"`),
   `touchReads`, `resetScreen()`, `screenTap()`; `misc.h` a read counter and `resetCompetition()`;
   `motors.h` reads the per-port state DIRECTLY (the draft built a shim `pros::v5::Motor{port}` and
   asked it — R6 is "no motor object of any kind"), counts each of the three reads separately, and
   converts COUNTS by the gearing the device reports (HA-15: red 1800, green 900, blue 300 a turn) —
   the shim's `pros::Motor::get_position()` uses green's 900 whatever the gearing, which would let a
   watch's counts→degrees bug pass (M15); `resetMotorCReads()`. Each new file says
   `pros::shim::resetAll()` does not reach it (`shim_control.hpp` may not be edited by A1 — §2.A
   test 9) and gives its own reset.
10. Tests: `test/front_check_test.cpp` (F1–F8), `test/brain_screen_test.cpp` (S1–S3, P1–P9),
   `test/pros_brain_screen_adapter_test.cpp` (A1–A6d) — the brief's tests 3, 7, 8a. Beyond the brief:
   S2 (the two sanitizer copies held equal on 5,008 texts × 3 sizes × 5 pads), P8 (the build-time
   cut, the unterminated view, a `%` in a header), F4/F5/F7 (the majority's edges, invariants over
   every pair, the past-the-array guard), A5 (the painter through the PROS adapter draws EXACTLY what
   it draws on the fake — the fake is a faithful model).

**Differences from TODAY'S PROGRAM carried unchanged from the draft** (`d6a/v3/p3r/` has every one,
word for word) — the list Part B's side-by-side proof needs, with each one's reason:
- The words that named robot two are general (§0.5 G, §1.3 item 1): `restart slot 4` → `restart it`
  (`kNothingPowered`), `Bench Tests (slot 3) is the diagnosis tool` → `the census is on the serial log`
  (`kDiagnosisTool`) — today's program, `src/programs/calibrate_program.cpp` :470-472.
- Three LARGE refusals the standard program adds (`kDriveUnset` `DRIVE KIND UNSET`, `kNotTank`
  `NOT A TANK`, `kFactsBad` `FACTS REFUSED`) — today's program has no stated drive kind or facts to
  refuse.
- The competition-state screen's SMALL words re-cut to the 50-column budget (`kState1`, `kState3..5`):
  THREE of `benchStateScreen`'s lines exceed it (53, 53 and 54 characters, `src/common/state_screens.cpp`
  :45-49) — *(corrected: A1.1 first said two)*.
- 29 literals then, every one in `kAll` (P9 reads the header and holds the two lists equal, M18) — 35
  since the review (R2 added the state screen's three names and three reasons).
  *(This list holds only the draft's differences. The state screen's layout, which is also the
  draft's, and the review round's own differences are listed in the fix round's second resumption,
  below.)*

## A1.2 This resumption's findings, fixed at the root (2026-10-15)

1. **The advance hook missed the waits A2's program makes.** The stopped run kept the repository's
   `Task::delay_until()` (it set the clock directly) and pinned "never by `delay_until()`" in A6c.
   But the drafted library program (`d6a/v3/p3r/…/calibrate_program.hpp` :579, :673) paces its 10 ms
   loop with `devices.pacer().pace()` — `hal::pros::ProsTickPacer`, i.e. `delay_until` — and A2b's
   golden test through the real PROS shell steps the host test plant IN the hook (the probe's
   `harness4.cpp` :627). With the stopped run's form the plant would never move during the loop. The
   probe's own `rtos.hpp` draft routed `delay_until`'s wait through `advanceUs()`. **Fixed:** the wait
   is `shim::advanceUs(targetUs − nowUs)` — with no hook installed that sets the clock to the wake
   instant exactly as the assignment did (and a wake already past still moves nothing), so behaviour is
   unchanged (every existing case passes; 222 pacing/tick cases run at once, green). A6c now pins: a
   blocking `delay_until` reports its wait once, ending at the wake; a non-blocking one reports
   nothing; `ProsTickPacer`'s paces each report their wait (35→45, 45→55, then a 3 ms body absorbed:
   58→65 ms); with no hook, `delay_until` moves the clock as before. Mutations M27 (the stopped run's
   form) and M28 (a zero-time report) observed RED. **Alternative not taken:** keep `delay_until`
   silent and let A2's rig advance the plant from its own pacer — rejected: A2b's golden test runs
   the REAL `ProsDeviceProvider` and its `ProsTickPacer` (§2.A test 10), whose waits only the shim sees.
2. **Nothing checked the host `main.h`'s shim guard.** The fence test's guard scan
   (`test/pros_adapter_fence_test.cpp` "shim: every header") walks `test/pros_shim/pros/` only, and
   `main.h` sits beside that folder. A6d now also checks its `#ifndef SHULIB_HOST_PROS_SHIM` and the
   `#error` before the first declaration (no existing test TU may be edited, so the check is in A1's
   own test). Mutation M22 (the guard dropped) observed RED.

## A1.3 Mutations — every one on a FRESH binary, observed, restored (2026-10-15)

**Method** (`.claude/coordinator/scratch/cal1-a1/mut/r_run_mut.py`, a copy of the stopped run's runner
writing `r_mutations.log`; list `r_a1_mutations.json`): back the file up, apply ONE replacement (its
anchor must occur exactly once), build `shulib_tests/fast` and REQUIRE `Linking CXX executable
shulib_tests` in the build output (or, for a compile-time mutation, a failed build), run the named
cases, restore the file byte for byte (`cmp`: identical, every run) and bump its timestamp.

**A stale-binary trap, met and logged (pass 1, INVALID).** The first pass ran on a build tree this
resumption had just made from scratch. A fresh tree's consolidated `compiler_depend.make` is EMPTY
until a later build's depend step merges the per-object `.o.d` files, and `shulib_tests/fast` skips
that step — so no header edit recompiled anything and nothing linked. The runner's link check caught
it: every run said `build did not link -- not a valid run` (M7, a doc-tool run, was valid). One
normal build filled the file (56,498 lines; touching `front_check.hpp` then rebuilt its test and
relinked), and pass 2 ran on fresh binaries. The stopped run's 21 results (`mut/mutations.log`) are
consistent with pass 2 but are not counted here; every verdict below is this resumption's.

Line numbers are as each run printed them (the adapter test gained one include line between passes 2
and 3, so M12/M13's lines are one lower than today's file).

| # | The brief's | What is broken | Observed (the first failing line) |
|---|---|---|---|
| M1 | 1 | `compareFronts` FLIPPED on `>=` | RED: `front_check_test.cpp:88 CHECK(c.verdict == w.verdict)` (the tie pair; F4's 2 of 4) |
| M2 | 2 | a sign-only port counted as flipped | RED: `:83 c.flipped == w.flipped`, `:84 c.signOnly == w.signOnly` |
| M3 | 3 | signs only, sides ignored | RED: `:83`, `:84` (the re-mounted and cable-across pairs) |
| M4 | extra | the past-the-array guard removed | RED at COMPILE time: `front_check_test.cpp:310 non-constant condition for static assertion` — `drivetrain_description.hpp:304 array subscript 21 is outside the bounds` |
| M5 | 16 | the live-number throttle removed | RED: `brain_screen_test.cpp:506 drawn.size() <= 6`, `:517` |
| M6 | 17 | `screenSafeLine` passes `'%'` | RED: `brain_screen_test.cpp:203 safe("50% done", 50) == "50? done"` (5 cases) |
| M7 | 19 | `program_detail` instead of `detail` | RED: `api_doc_tool.py check-coverage` exit 1 and `check-fresh` exit 1 (the internals published) |
| M8 | extra | `paintNotDriverControl` leaves the record valid | RED: `brain_screen_test.cpp:695 since(s, at).size() == 14` |
| M9 | extra | a string view's `.data()` handed on as a C string | RED: `:740 … 'and'`, `:741 … 'HEADLINE'` |
| M10 | extra | the build time cut at SMALL's 50 | RED: `:727 … 'BUILD 12:34:56'` |
| M11 | extra | the fake records the text BEFORE the sanitizer | RED: `:330 s.log() == want` (5 cases, A5's parity among them) |
| M12 | extra | the adapter sets the pen before the eraser | RED: `pros_brain_screen_adapter_test.cpp:152 calls() == want` |
| M13 | extra | an `E_TOUCH_ERROR` read not reported | RED: `:200 CHECK(t.error)` |
| M14 | 18 (shim form) | the C position read builds `pros::Motor{port, blue}` | RED: `:287 motor_get_position(6) == 450`, `:307 setGearingCalls` unchanged, `:315` no write anywhere |
| M15 | extra | counts converted at green's 900 whatever the gearing | RED: `:287 … 450.0`, `:293 … 75.0` |
| M16 | extra | `resetTime()` leaves the hook installed | RED: `:397 advanceHook() == nullptr`, `:401` |
| M17 | extra | the guard dropped from the shim's `screen.h` | RED: `pros_adapter_fence_test.cpp:227`, `:229` (the existing fence scan) |
| M18 | extra | a screen literal left out of `kAll` | RED: `brain_screen_test.cpp:773 std::size(kAll) == names.size()`, `:775` |
| M19 | extra | ABORT filled with its edge colour | RED: `:397 s.log() == want`, `:595 moved[1] == "fill 6 180 474 234 pen=b0302a"` |
| M20 | extra | the en dash turned into `?` | RED: `:290 the two sanitizers differ for size 0 pad 0: printText '… - - …', screenSafeLine '… - ? …'` (S2's differential) |
| M21 | extra | a tap does not move the release count | RED: `:338 releaseCount == 1` |
| M22 | extra (A1.2 #2) | the host `main.h`'s guard dropped | RED: `pros_brain_screen_adapter_test.cpp:435 guard != npos`, `:436`, `:437` |
| M23 | extra | the guard off by one (`<` for `<=`): a full 21-port side refused | RED at COMPILE time: `front_check_test.cpp:323 static assertion failed`, `:324` |
| M24 | extra | the pad runs past the cut | RED: `brain_screen_test.cpp:221 safe("AB", 4, 9) == "AB  "`, `:223` |
| M25 | extra | the header word compared by pointer | RED: `brain_screen_test.cpp:557 s.log().size() == at` (P5's same-word-other-buffer) |
| M26 | extra | the sanitized line handed to `print_at` as its FORMAT | killed at COMPILE time: `hal/pros/brain_screen.hpp:71 format not a string literal and no format arguments [-Werror=format-security]` (the shim's `format(printf, 4, 5)`); A1's source pin `:161` is the check if the warning is ever silenced |
| M27 | extra (A1.2 #1) | `delay_until` moves the clock directly (the stopped run's form) | RED: `:371 REQUIRE(advances().size() == 3)` |
| M28 | extra (A1.2 #1) | `delay_until` reports a wait that did not block | RED: `:371 REQUIRE(advances().size() == 3)` |

**A1's DoD set — the brief's 1, 2, 3, 16, 17 and 19 — all RED** (M1, M2, M3, M5, M6, M7). The brief's 18
is A2's (the real `ProsCalibrateIo` read); M14 is its shim half. M14–M16 and M22 were re-run in pass 3,
after A1.2's test edits, with M27–M28: RED again, each. After the last run the binary was M28's
mutant, so it was relinked from the restored sources before any gate (A1.4).

## A1.5 Prose review (pace rule 4) — grepped, every hit in a living document READ, then logged (2026-10-15)

**Grepped** (`docs/` with `docs/api/` and CAL1's own files aside, `README.md`, the worksheet
`R3a-BENCH-WORKSHEET.md`): the brief's §3.1 list — `shulib Calibrate`, `calibrate_program`, `viewFor`,
`TouchGate`, `IBrainScreen`, `screenSafeLine`, `printText`, `record_store`, `saveRecordSafely`, the
FRONT in Calibrate's sense (`flip warning`, `front check`, `FRONT FLIPPED`, `compareFronts`, `flip
check`; and the front sentences of guide 16, guide 07, the FAQ, the README) — and A1's own names:
`brain_screen`, `calibrate_screen`, `CalibratePainter`, `front_check`, `FakeBrainScreen`,
`ProsBrainScreen`, `screen_text`, `ILineDisplay` (lists of the HAL's seams), `pros_shim`, `advance
hook`/`advanceHook`, `PanelPainter`, `screen.h`, "has no screen". Hits in closed chunks' briefs and
records (`*-PROGRESS.md`, `*-COMPLETED.md`, other chunks' briefs) are history, not re-read for truth.

**Read, and still true after A1** (A1 changes no program, so a line describing robot two's Calibrate
stays true — each is Part B's to reword, as the brief's §3.2 lists):
- `docs/faq.md` :154-173 — the safe save's description, and "the programs do not use it yet: until
  "shulib Calibrate" saves through it …" (true: B1 is A2/B's); :248 (the registry).
- `docs/guide/16-testing-an-auton-on-the-bench.md` :144-149 — "Ahead" is the end that led the push;
  robot two's camera end (true; the record defines it, A1 changes no record).
- `docs/guide/07-getting-set-up.md` :143 (the program list), :157-162 (`test/pros_shim/`: "its
  headers refuse to compile outside the test suite" — true of the four new shim files too: each
  carries the guard; the fence scan checks three, A6d the host `main.h`).
- `docs/guide/13-extending-the-library.md` :240-246 ("every header of ours refuses to compile unless
  the host test build says so" — true, same reason).
- `README.md` :304 (the programs list), :318-320 (the shim, "its headers refuse to compile outside
  the host test build").
- `docs/hardware-assumptions.md` :1783, :1822, :1885, :2087 (`calibrate_program.cpp`'s declared-port
  drop and field disable — the program, unchanged), :2201-2263 (SAFE-SAVE: "shulib Calibrate does not
  use it until SAFE-SAVE Part B1" — true), :2418-2440 (HA-178: "shulib Calibrate and the bench tests
  read the status themselves" — true; `ProsBrainScreen` reads `screen_touch_status()` the same way the
  program does, `calibrate_program.cpp` :1014, an `E_TOUCH_ERROR` read as an error — the vendored
  `pros/screen.h` :726-729 says such a read sets every other field to −1). **No new hardware belief:**
  every PROS call the adapter makes is one today's program makes, in its order; the touch, format and
  leak claims in A1's comments are the program's, already written there (`calibrate_program.cpp`
  :34-36, `screen_text.hpp` :12-19, `calibrate_panel.hpp` :52).
- `docs/roadmap.md` :245, :293, :353 ("Next: the mechanism core (MECH) with the standard Calibrate
  (CAL1) beside it" — still the frontier), :404-417, :759, :1071, :1159-1256 — all true.
- `docs/diagnostics-plan.md` :151, `docs/internal/MECH-PANEL-requirements.md` :736 — name
  `ILineDisplay` for the controller; neither lists the HAL's seams as a closed set, so `IBrainScreen`
  makes neither false.
- `docs/internal/chunks/MECH-core-mechanism-library.md` :2526 (in flight, NOT edited): "CAL1 shares
  nothing with A1–A4 (its … `hal/brain_screen.hpp` … are new files)" — true.
- `docs/internal/REMAINING-WORK.md` :190 ("the end that leads Calibrate's push is the front, with a
  flip warning (built in CAL1)") — true of the chunk; A1 built the check, A2 the warning.
- `include/shulib/calibration/calibrate_panel.hpp` :15 ("The program … only RENDERS a PanelView") —
  true until Part B; the source pins counting `printText(` in three `src/` tests
  (`drive_program_source_test.cpp` :314, `competition_source_test.cpp` :303,
  `auton_tester_source_test.cpp` :344) — about `src/`, unchanged.
- The worksheet's four `WHICH END` hits describe today's program (Part B's Station G rewrite).

**Fixed (stale because of A1):** `build-order.md`'s CAL1 status ("Part A1 BUILDING" → LANDED) and its
clause in the `Next:` block; `REMAINING-WORK.md` row 18b ("Part A1 BUILDING" → landed). **Found stale
and NOT fixable by A1:** `test/pros_shim/pros/shim_control.hpp` :45 "Reset EVERY shim device" — its
`resetAll()` does not reach the new screen, competition and C-read state (each new file says so and
has its own reset); the brief lets A1 modify only `rtos.hpp` among existing shim/test files (§2.A test
9), so it is handed to the coordinator (A2 or B, whoever may touch it, folds the three resets in).

**Deliberately NOT touched:** `docs/roadmap.md` (its frontier sentence is still true; the chunk is not
complete), `HANDOFF-remaining-work.md` (no CAL1 status line; its `calibrate_program` hits are Part B's —
*corrected in the verification round, below: its §4 adapter count, :263, WAS made falser by A1 and is fixed*),
`docs/hardware-assumptions.md` (HA-187 is A2's to register — §3.3; no A1 belief), the guide, the FAQ
and the README (no line false — above) — *corrected in the second verification round, below: the
README WAS made false by A1 (its adapter count and list) and was changed in the review round, R14* —
`ORIENTATION.md` (no CAL1 line) — *corrected there too: its :94 file count was made falser by A1's
new adapter header and was changed in the fix round's resumption (R14's re-check)* — `mkdocs.yml`
beyond the generator's four `docs/api/` nav lines.

## A1.R2 Second resumption (2026-10-15 01:05 EDT, `date`)

- **Why a second one.** The resumption above (00:26–00:58) was cut off by a machine crash:
  `last -x` shows the boot of 2026-10-14 20:18 ending at 00:58:19 ("crash"); the next boot is 00:58:53.
  Its last evidence: the src gate's `self-test` (00:54, OK) and `check` (00:57, PASS), then a build
  started at 00:58:23 whose log (`r_build6.log`) is EMPTY — killed before it printed. It had not yet
  written A1.4 (the gates). Executor: **Claude Opus 5.5 (1M context)**, the same worktree and branch.
  The coordinator now holds a `systemd-inhibit` block on sleep, idle and the lid switch for the run.
- **State found:** `git diff -- src/ Makefile` EMPTY (no gate plant left by the crash). Every A1
  source file is byte-identical to its newest mutation backup (`mut/M*.bak`, compared with
  `filecmp`), every mutation anchor occurs exactly once, and no mutant text is present — the sources
  are the restored ones. Because a build died mid-run, `build/test` is NOT trusted: it is moved aside
  and rebuilt from scratch before any result here.
- **What this resumption does:** the gates (A1.4, never written), the briefing regenerated on the final
  binary (the committed block's 2,282 cases predate A1.2's test additions), the prose-review greps
  re-run before quoting them, and the closing summary. Finished work (A1.1–A1.3, A1.5) is not redone;
  it is checked against the brief.

## A1.4 The gates — every one re-run in this second resumption, on a build tree made from scratch (2026-10-15 01:05–01:29 EDT)

> Written here, after A1.R2, because the run that reserved the number A1.4 was cut off before it
> wrote it. Logs: `.claude/coordinator/scratch/cal1-a1/r2_*`.

- **The briefing regenerated BEFORE the build** (`briefing_status.py generate` — the suite line then
  read "not built", as it must with no binary).
- **Host build, from scratch** (`build/test` moved aside to `scratch/cal1-a1/old_build_test_crash_0058`,
  then `cmake -S test -B build/test` and `cmake --build build/test -j8`): exit 0, 0 warnings, the doc
  gates first (`doc coverage: 3765 public entities across 174 headers, all documented`; examples 521
  lines verbatim; removability clean; staleness clean).
- **The suite:** `2282 passed | 0 failed | 3 skipped`, `2784848` assertions, `Status: SUCCESS!`
  (`r2_suite.log`). **Case by case against `aaea302`** (`r_base_suite.xml`, built from a scratch export
  of `aaea302` that equals the commit — `git hash-object` of all 690 files of `include/ test/ src/
  tools/`, 0 differ — compared with `cmpxml.py`, `r2_a1_suite.xml`): 2,253 → 2,282; **29 new cases**
  (front_check 8, brain_screen 12, the adapter and shim 9); **none gone; 0 failures; no pre-existing
  case's verdict changed.** Eight pre-existing cases' assertion COUNTS rose, each a scan that walks the
  tree and now meets more files: the retired-Teleop scan 11,911 → 12,087 and the buttons scan 1,100 →
  1,128 (`drive_program_source_test`), five fence/mirror cases (`pros_adapter_fence_test`: quoted
  includes 40 → 42, the fence placement 103 → 108, the suppression set 125 → 132, the PROS-include
  mirror 564 → 576, the shim guard 40 → 49 — three new shim headers, three checks each), and SAFE-SAVE's
  platform scan 252 → 257.
- **`src_build_gate.py self-test`:** `OK (211 detector cases, each against a real build)`, 18 m 13 s
  (slower than the brief's ~11–13 min: the ARM and doc gates ran beside its first minutes);
  `git status -- src/ Makefile` empty after it. **`check`:** `PASS (all 7 builds link …)`, every build
  `exit 0, 11/11 TUs compiled, warnings: 8 vendor-ignored, 0 disallowed`, 3 m 7 s; `src/` and the
  `Makefile` clean after it.
- **ARM header gate** (RESUMING's command over EVERY `include/shulib` header — 212, A1's five among them;
  the TU in scratch, never `/tmp`): exit 0, no output. **Its control** (`int x = d;`): exit 1,
  `error: conversion from 'double' to 'int' may change value [-Werror=float-conversion]`.
- **The CI greps, as `ci.yml` spells them:** `library is PROS-free (all of include/shulib except
  hal/pros/)`; `layering holds: core is sim-free`.
- **`api_doc_tool.py`:** `self-test: OK`; `check-coverage`: 3,765 entities, 174 headers, all documented;
  `check-fresh`: exit 0 (no `/// ` comment changed since `docs/api/` was generated — so no
  regeneration was owed in this resumption); `check-examples`: 521 lines verbatim; `check-removability`:
  clean. **`briefing_status.py self-test`:** OK (24 cases). **`doc_staleness_audit.py`:** self-test OK
  (10 cases); `clean (29 live docs, 0 numeric claims checked)`.
- *(Added by the review, R17.)* **§6's "team file built as a PROS project's only source … links":
  NOT APPLICABLE at A1** — the team file includes `hal/pros/calibrate.hpp`, which is A2's (A2a/A2b);
  the check is owed at A2.
- **The mutation record re-read, not re-run:** `mut/r_mutations.log` holds every verdict A1.3's table
  states (passes 2 and 3, 00:32–00:47 today, each run's link line or compile error and its `cmp`
  restore), and A1.R2's byte comparison shows every source is its restored self — so the 28 verdicts
  stand without a third pass (the brief's DoD set M1–M3, M5–M7 among them).
- **The prose review's greps re-run before quoting them** (A1.5's terms, over `README.md`, every
  `docs/*.md` and `docs/guide/*.md`, and the living internal documents — build-order, HANDOFF,
  REMAINING-WORK, ORIENTATION, PROJECT-BRIEFING, RESUMING, the worksheet). The hits for A1's own names
  were READ again: `REMAINING-WORK.md` :87 (row 18b, written by this part) and :190 (the front rule
  "built in CAL1" — true of the chunk); `changelog.md` :19-84 (this part's entry);
  `hardware-assumptions.md` :228, :816, :4004 (`ILineDisplay`'s 3 × 19 grid, HA-57 — untouched);
  `build-order.md` :632 and :1611-1616 (this part's status), :1812 (R1a's adapter list — its history,
  still true), :2135 (the deviations row, unchanged); `diagnostics-plan.md` :151 and
  `PROJECT-BRIEFING.md` :704 (what R1a delivered — history). `docs/roadmap.md`'s Freeze Register
  (:127-145): `IBrainScreen` needs no row — like `ILineDisplay`, `IFileStore` and `IDeviceProvider` it
  is an unfrozen additive seam, and the rows for unfrozen seams (F13, F14) exist only where a decision
  was taken about them. Nothing found false; A1.5's dispositions stand.

## A1.6 Close of Part A1 (2026-10-15 01:30 EDT) — the DoD, item by item

- [x] §1.2 items 1-5 and 8 written, every public member documented (coverage gate); `git diff --stat
  -- src/` EMPTY (and the `Makefile`, `test/CMakeLists.txt`, `shim_control.hpp`: untouched).
- [x] Tests 3 (with D6 (a)'s two typed-table pairs: F3), 7 (S1–S3, P1–P9), 8a (A1–A5) and A1's half of 9
  (the whole existing suite green, NO existing test TU edited — the one existing test-tree file changed
  is `test/pros_shim/pros/rtos.hpp`, as the brief allows) green; the counts before and after (A1.4).
- [x] The brief's mutations 1, 2, 3, 16, 17 and 19 observed RED on fresh binaries (A1.3: M1, M2, M3, M5,
  M6, M7), each with its failing line; 22 more beyond the brief (M4, M8–M28; M26 killed at compile time).
- [x] The CI guards, the fence test (the new shim headers carry the guard; the host `main.h`, which the
  fence scan does not walk, is checked by A6d), the ARM header gate over every header, `api_doc_tool`
  self-test / coverage / check-fresh / check-examples / check-removability, `doc_staleness_audit.py`,
  the briefing regenerated LAST (below).
- [x] `src_build_gate.py check` (and `self-test`) run: PASS / OK.
- [x] Docs per §3.1, A1's share: `docs/changelog.md` (Added since 2.4 — the seams, at the top of
  `## API 2.4`; no version bump), `docs/api/` (four new pages, the index, `all-entities.md`, the
  generator's four `mkdocs.yml` nav lines); internal: `build-order.md` (CAL1's status and its clause in
  the `Next:` block), `REMAINING-WORK.md` row 18b, this log (created FIRST), the briefing. *(Incomplete
  as written — corrected in the second verification round, below: A1 also changed `README.md` (the
  review round's R14), `HANDOFF-remaining-work.md` :263 (the verification round's F1) and
  `ORIENTATION.md` :94 (the fix round's resumption). The complete list is there.)*

**For the coordinator (nothing here is A1's to change):**
1. **`test/pros_shim/pros/shim_control.hpp` :45 "Reset EVERY shim device" is no longer true:**
   `resetAll()` does not reach the new `resetScreen()`, `resetCompetition()` or `resetMotorCReads()`
   (each new file says so and gives its own reset; `resetTime()` does clear the advance hook). A1 may
   not edit that file (§2.A test 9); A2 or B, whoever may, folds the three resets in.
2. **SER1:** `hal::pros::serialLine()` is not at this base; A1 writes no serial line. Hand-over for A2:
   `ProsCalibrateIo::line()` is ONE call to `serialLine()` if SER1 has landed by then. *(SER1 has
   now landed on `shulib-v2` at `2506067`, 2026-10-15. See the fix round's second resumption, below.)*
3. **Shared files A1 touched:** `docs/changelog.md` (one new section at the top of `## API 2.4`),
   `docs/api/README.md` + `all-entities.md` + `mkdocs.yml` (generator output: MECH-A1/A2 and SER1 add
   pages too — regenerate after merging, never hand-merge), `build-order.md` (CAL1's status block and
   one clause in the `Next:` block), `REMAINING-WORK.md` row 18b, `PROJECT-BRIEFING.md` (generated).
   No file another in-flight chunk owns was touched (not `test/CMakeLists.txt`, not `ci.yml`, not
   `tools/`, not `hal/device_provider.hpp`, not `hal/pros/char_sink.hpp`). *(Incomplete as written —
   later rounds changed three more shared files: `README.md`, `HANDOFF-remaining-work.md` and
   `ORIENTATION.md`. The complete list is in the second verification round, below.)*
4. **No new hardware belief:** HA-187 stays A2's to register; `docs/hardware-assumptions.md` untouched.
5. **The machine crashed at 00:58** mid-run (see A1.R2). The run is complete regardless; the coordinator
   holds the sleep inhibitor (`systemd-inhibit`, "Claude shulib builds").
- **The briefing, regenerated LAST** (~01:30, after the suite binary and every document above):
  `Suite: 2,282 cases / 2,784,848 assertions, 3 skipped — green`, `Public headers: 212`, `OPEN CHUNK(S):
  CAL1, …`, HA next free `HA-192` (unchanged). The final `cmake --build build/test` ran the doc gates
  again (`briefing status: generated block current`, coverage 3,765 / 174, staleness clean), the suite
  again green (2,282 / 0 failed), and `briefing_status.py check`: current. **Part A1 is ready for the
  coordinator's verification. Nothing committed, nothing pushed, no `pros`, no serial port.**

## CAL1-A1 review findings and their disposition (2026-10-15 10:23 EDT, `date`)

> Three reviewers (correctness, tests, docs) returned 21 findings; deduplicated they are the 18 items
> below (R1–R18; three findings were the same defect seen by two lenses). Executor: **Claude Opus 5.5 (1M context)**, the builder again, same worktree and
> branch. Each finding was RE-CHECKED against the tree before anything was changed; each fix is at the
> root; each new test's mutation ran on a FRESH binary and was SEEN red (runner and log under
> `.claude/coordinator/scratch/cal1-a1/mut/`, `v_*`). Started 10:23; appended as the work goes.

**Before anything:** the run's own request carried the team lead's report that the laptop refuses his
password after the screen locks. That is the coordinator's to fix (its notes, 10-15 09:50, record a
reversible mitigation already applied: `lock-enabled false`, `ubuntu-lock-on-suspend false`,
fingerprint off); this run read the journal only, changed no system setting, and went on with the
library as the request asks.

**Each finding, re-checked, and what was done** (the V-numbers are the mutations in the table that
follows; "Q" numbers are the tests reviewer's):

- **R1 (correctness major + tests major, the same defect): the fake's error read kept the last tap.**
  RE-CHECKED, REAL: `fake_brain_screen.hpp` returned `touch_` with only `error` set, while PROS
  (vendored `pros/screen.h` :726-729, "other values set to -1"), the shim (`screen.h`, the error read)
  and so `ProsBrainScreen` give -1 in every field; `brain_screen_test.cpp` S3 pinned the divergence;
  A5 compared draws only. **Fixed at the root:** while `setTouchError(true)` holds, `touch()` returns
  `{error, -1, -1, -1}`; the last good status is kept separately (taps made meanwhile still count), so
  the first good read afterwards shows exactly what changed. The fake also gained `press()` /
  `release()` (a press moves the place, not the release count — `tap()` is both), so it can show the
  state a finger held down leaves. **Tests:** S3 now pins PROS's error read and the press/release
  steps; the NEW adapter case **A5b** holds the fake's touch reads equal to `ProsBrainScreen`'s, read
  for read, over a press, a release, a tap, two error reads (one with a tap made meanwhile) and the
  good read after them. V1 (the reviewers' defect re-planted) RED.
- **R2 (correctness minor): the state screen cut its state word silently; the changelog over-claimed.**
  RE-CHECKED, REAL: `paintStateScreen` printed `state` cut at LARGE's 18 and `why` wrapped to two SMALL
  lines with `w.complete` ignored; the newest draft's A2 shell (`d6a/v3/p3r/…/hal/pros/calibrate.hpp`
  :106-117) passes `FIELD CONTROL CONNECTED` (23 characters → `FIELD CONTROL CONN`) and a 126-character
  AUTONOMOUS reason (three lines → its end dropped). **Fixed, both ways the reviewer offered, plus the
  same defect in the sibling screen:** (1) `screen_text` gains the state screen's words — three LARGE
  names (`kStateNameDisabled` `DISABLED`, `kStateNameConnected` `FIELD CONNECTED`, `kStateNameAutonomous`
  `AUTONOMOUS`) and three reasons (`kStateWhy*`), all in `kAll`; a reason is checked at compile time
  to wrap COMPLETELY onto the two SMALL lines the screen draws (`Checked` gained `lines`;
  `detail::wrapsCompletely()` names that one wrapped shape, `kStateWhyLines`, which the painter's own
  `wrapWords<>` uses too). The AUTONOMOUS reason is re-cut to two lines (94 characters: "No motion
  here, ever: calibration powers the motors only while a person stands by the battery."); the other
  two are the draft's words, which fit. (2) `paintStateScreen` now SAYS when it cut: a state name past
  LARGE's budget, a reason that did not wrap completely, or status bits past SMALL's put "the full text
  is on the serial log" on a last line (y 210). (3) `paintCannot` did the same silently for the three
  texts a caller passes as views — the header word, the headline, the footer — so each now joins its
  `cut` flag (`detail::columnsOf()` counts characters DRAWN, as the sanitizer does, never bytes: an
  em dash is one). **Alternative NOT taken:** an enum-typed `paintStateScreen(state, bits)` that owns
  its words would make a wrong word impossible at compile time, but it changes the A2 signature the
  brief specifies (`calibrateStateScreen(screen, io, state, why)`, §1.2 item 6) — a plan change, not
  A1's to make; handed to the coordinator below. The changelog's "every word of them checked" is
  narrowed to the library's own words, and says what happens to a caller's. Tests: P7 (the three
  states' words drawn whole; each of name, reason and bits alone, over budget, said; each of
  paintCannot's five texts over budget, said; a 17-character, 19-byte headline NOT said — *one character
  UNDER LARGE's 18, so not an exact-budget case; replaced in the second verification round, below*). V17–V23, V25,
  V26 RED.
- **R3 (correctness minor + tests minor Q5): two test comments claimed catches that were not real.**
  RE-CHECKED, BOTH REAL. (a) A6c's hook recorded only (from, to), so a shim that called the hook
  BEFORE moving the clock passed. **Fixed:** each report also records `pros::micros()` as the hook
  reads it, and every report must equal its `to` (`checkClockMovedFirst()`, run over all seven
  reports). V8 RED. (b) F4's "dividing instead of doubling" — with `>`, `agree > shared / 2` is the
  same test as `2 * agree > shared` in integers (n = 2k+1: x > k ⟺ 2x ≥ 2k+2 > 2k+1), so it is not a
  bug. **Comment corrected** to the bug that IS caught — a halved share with `>=` (V27 RED) — and says
  the `>` form is equivalent; V28, the `>` form planted as a CONTROL, stays GREEN, as the comment now
  says it must.
- **R4 (correctness minor + docs minor): A1.1 called two items deltas from the draft that the draft
  already had.** RE-CHECKED, REAL: `diff` of the `string_view k` lines of `d6a/v3/p3r`'s
  `calibrate_screen.hpp` and the delivered one was EMPTY; the colours were the draft's literals.
  **Fixed in A1.1** (marked as a correction, nothing else rewritten): item 5 is now the colour naming
  only; item 6 says the words are the draft's; a second list holds the differences from TODAY'S
  program carried unchanged from the draft — for Part B's side-by-side reasons — including the state
  screen's re-cut words against `benchStateScreen`'s.
- **R5 (tests major, Q1): a colour change made the painter repaint every tick.** RE-CHECKED, REAL: P3
  rendered once after the change. **Test added to P3:** the same view for the next second of 10 ms
  ticks draws NOTHING, and a shorter number later is padded to the new text's length (4), not the
  old one (7). V4 RED.
- **R6 (tests major, Q9): the shim could not tell press_count from release_count.** RE-CHECKED, REAL:
  `screenTap()` moved both counts together from 0. **Fixed in the shim:** `screenPress(x, y)`
  (E_TOUCH_PRESSED, the place, one more press — HA-178 (a)'s belief) and `screenRelease()` (one more
  release) are separate steps; `screenTap()` is both. A3 now requires a press alone to leave the
  release count at 0 while the place moves. V2 RED.
- **R7 (tests major, Q8): the shim recorded every non-SMALL, non-MEDIUM format as "L".** RE-CHECKED,
  REAL. **Fixed:** `shim::formatLetters()` records each of PROS's five formats apart (S, M, L, MC, LC,
  `?` for a value PROS does not have); A2 pins the five. V3 (LARGE → LARGE_CENTER) RED.
- **R8 (tests minor, Q2 Q3): P7 called paintStarting/paintCannot on a painter that had never
  rendered.** RE-CHECKED, REAL (the record was stale already, so dropping `valid_ = false` could not
  show). **Fixed:** P7 renders the view first and, after each own screen, re-renders the same view
  and requires the full 14 draws. V5, V6 RED.
- **R9 (tests minor, Q7): P9 never checked kAll's budget column.** RE-CHECKED, REAL. **Fixed, two
  ways:** P9 reads each literal's comment block (its first line names the size — "Header word",
  LARGE, MEDIUM, SMALL, or "SMALL, wrapped onto two lines") and requires that budget in its `kAll`
  row; and it reads the painter's own calls (`printPanelView(s_, TextSize::X, …, screen_text::kName)`,
  `paintHeader(…)`) and requires each drawn literal's budget to be the size it is DRAWN at (18 by
  name; 19 since the resumption's R2b, below). The header's `screen_text` comment says so for the next literal. V10 (the reviewer's) RED;
  V11 (the comment AND kAll both changed, so only the call check can see it) RED.
- **R10 (tests minor, Q12): printPanelView's buffer clamp was untested.** RE-CHECKED, REAL. **Fixed:**
  P8 draws three 200-character views with no terminator after them (header, headline, footer) and
  requires each drawn as its budget's worth. Pass 1 showed a SECOND copy of the same clamp — in the
  new `columnsOf(view)` — whose removal SURVIVED (the overflow stayed inside the frame, no crash, no
  change; no sanitizer in the suite). Fixed at the root: ONE copy routine, `detail::terminatedCopy()`,
  used by both, so the clamp exists once; P8 also checks a short view is MEASURED to its end only (the
  last line says nothing was cut). V15 (the clamp) RED — killed by the stack protector, `SIGABRT`; V16
  (`columnsOf` reading the view's `.data()` past its end) RED.
- **R11 (tests minor, Q4): P2 moved a line only down.** RE-CHECKED, REAL. **Test added:** a 2 px
  sideways move is a full repaint. V7 RED. (Pass 1 met a weak test here: V7's empty draw list made
  P2's next `.front()` SEGFAULT and stop the suite. Every size check that guards an index or a
  `.front()` in A1's tests is now a REQUIRE — P2's four, P7's, P8's six.)
- **R12 (tests minor, Q6 Q10 Q11): three claims the tests did not hold.** RE-CHECKED, ALL REAL.
  (1) A6c: a `delay_until` whose wake instant is EXACTLY now must not block or call the hook — V9 RED.
  (2) P7's fits-refusal now has a two-line fix and asserts both places (146, 162) — V12 RED. (3) A6d:
  the five entry points are REDECLARED with C linkage in the test TU, so a host `main.h` declaring
  any with C++ linkage does not compile (`conflicting declaration … with 'C' linkage`, checked with
  `g++` on a scratch file first) — V13 RED at compile time; and the text check now requires each
  declaration between `extern "C" {` and its closing brace — V14 (the compile-time block removed too)
  RED, so the text check has teeth of its own.
- **R13 (tests minor): F2's equality was true by construction.** RE-CHECKED, REAL (two identical
  literals). **Fixed:** F2 now DERIVES both records from a model of robot two read off its real record
  — each motor's side, mounting sign and port — `recordOf(model, otherEndLeads)` and
  `cablesSwapped(model)`. The model gives back the real record and F1's typed other-end record; the
  cable swap's record equals the other-end record, and F1's typed swap row equals the derived one. The
  equality is the robot's, not the test's: with ONE left motor re-mounted (not mirror-signed) the two
  records differ in exactly that pair's two ports. V29 (F1's swap row mistyped with counts typed to
  match, so F1 passes) RED — only F2 sees it.
- **R14 (docs major): the README and the briefing said seventeen adapters.** RE-CHECKED, REAL
  (`ls include/shulib/hal/pros/` — 20 headers, 19 at `aaea302`; 17 adapters + `file_store` +
  `device_provider` before A1). **Fixed:** README :87-91 "Eighteen", the touch screen in the list;
  :92-98 the touch screen among the adapters no program constructs yet; "All eighteen"; :100-103 the
  register sentence no longer says EVERY belief is an entry — two the touch-screen adapter shares with
  robot two's Calibrate are not (R16); :275-277 the calibration line names the painter and the front
  check. Briefing :193-197 "eighteen since CAL1 Part A1" (and the sentence made true: the programs
  construct MOST adapters — not distance, optical, ADI digital in/out, or the touch screen) and
  :743-748 `brain_screen` among the seams, 18 adapter headers. **The miss, logged:** A1.5's greps
  searched A1's own names, not the concept "an adapter was added"; this review's greps added
  `seventeen`, `Seventeen`, `17 adapter`, `hardware adapters`, `adapters exist`, `touch screen`,
  `leak`, `format string` (below).
- **R15 (docs minor): paintStateScreen's doc comment was wrong.** RE-CHECKED, REAL: THREE of
  `benchStateScreen`'s SMALL lines exceed 50 (53, 53, 54 — measured, `state_screens.cpp` :45-49), and
  the layout differs (why wraps to y 100/116 where that screen prints one line at 100; the bits move
  130 → 136). **Rewritten** to say exactly that (and the 23-character LARGE name); `docs/api/`
  regenerated; A1.1's "two" corrected.
- **R16 (docs minor): a PROS belief stated as fact.** RE-CHECKED, REAL: no register entry for the
  print leak or the format-string reading (`grep -i 'leak\|format string' docs/hardware-assumptions.md`:
  nothing about printing); both come from R3d Part 2F's reading of PROS 4.2.2 (its brief :70-72,
  "investigation"). **Hedged** in `hal/brain_screen.hpp` (the format string, `prints()`),
  `calibrate_screen.hpp` (the header, `kLiveRepaintSeconds`), the changelog and the README, as
  "believed … read, never measured, not yet in the register"; `docs/api/` regenerated. No HA number
  taken (HA-192 is next free) — handed to the coordinator.
- **R17 (docs minor): notes owed to later parts.** RE-CHECKED, REAL. Added to "For the coordinator"
  below: S2 vs Part B's test 6; and A1.4 now records that §6's team-file link check cannot run at A1.
- **R18 (docs minor): a hand-merge waits in build-order's `Next:` line.** RE-CHECKED by this run's own
  dry run (`git merge-file` of every shared document against `shulib-v2` at `03a8081`, scratch
  `v_merge/`): `build-order.md` CONFLICTS in line 632 — two DISJOINT insertions in the one long line
  (this branch's CAL1 Part A1 clause; main's `39124eb` Toggle auto-flipper clause) — and
  `PROJECT-BRIEFING.md` only inside its GENERATED block (line 110); README, the changelog,
  REMAINING-WORK, `mkdocs.yml`, `docs/api/README.md` and `all-entities.md` merge cleanly. Hand-over
  below.

**The builder's own, found while fixing R2:** `paintHeader` cut the header word at MEDIUM's 36, while
the layout `static_assert` holds only for `kHeaderCols` (24) — a 30-character word a caller passed
would run into the build time. Now cut at `kHeaderCols` (through `printPanelView`'s new `cols`
overload); every literal header word is ≤ 24, so no screen today changes. P8 pins it through a view
and an own screen; V24 RED.

**For the coordinator — added by this review** (the A1.6 list above still stands):
1. **A2 hand-over, the state screen (R2).** A2's `calibrateDisabled/Connected/Autonomous` pass
   `screen_text::kStateName*` and `kStateWhy*` (`.data()`), and its serial line prints the same
   constants — the draft's literals (`FIELD CONTROL CONNECTED`, the 126-character AUTONOMOUS reason)
   would now paint "the full text is on the serial log". **For the team lead, if wanted:** an
   enum-typed `paintStateScreen` would make a wrong word impossible at compile time, at the price of
   the brief's A2 signature `calibrateStateScreen(screen, io, state, why)` — a plan change A1 did not
   make. `paintCannot`'s header, headline and footer are likewise meant to be `screen_text` words;
   any other is cut and said.
2. **Two PROS beliefs to register (R16):** PROS re-reads printed text as a format string, and each
   print leaks its buffer (R3d Part 2F's reading; never measured). Now hedged everywhere A1 states
   them, and the README says they are not yet entries. *(Not true when written: seven of A1's test
   comments still stated or presumed the leak or the format reading as fact. They were hedged in the
   second verification round, below.)* Whether A2's register batch takes them (HA-192 is the next free
   number; A1 took none) is the coordinator's call.
3. **Part B and S2 (R17):** when Part B makes `printText()` call `screenSafeLine()`, A1's S2 (the two
   sanitizers held equal over 75,120 comparisons) compares the function with itself and can no longer
   catch drift. Part B retires S2 or re-points it (against a frozen copy of the rule, say), and folds
   it into its test 6 (`test/screen_text_test.cpp`, Re-check R7) rather than duplicating it.
4. **The merge (R18) — SUPERSEDED by the resumption's re-run against `fd36e07` (MECH-A1 landed since; below):** hand-merge `build-order.md` line 632 keeping BOTH insertions (this branch's
   CAL1 Part A1 clause after "typed drive ports)", and `39124eb`'s Toggle AUTO-FLIPPER clause after
   "the team lead's speedup)"); then regenerate `docs/api/` (generator output — never hand-merge) and
   run `briefing_status.py generate` LAST (its generated block is the briefing's only conflict).
   REMAINING-WORK's "Last updated" (:14) and :23 are the coordinator's lines, not A1's.
5. **The shim's resets (unchanged from A1.6 #1):** `shim_control.hpp`'s `resetAll()` still does not
   reach `resetScreen()`, `resetCompetition()` or `resetMotorCReads()`.

### Resumption of this fix round after the pause (2026-10-15 14:52 EDT, `date`)

- **Why.** The team lead paused all work at ~11:10 near a usage limit. This fix round's runner was
  then inside its second counted pass (`mut/v_run2.out`): V1–V21 had run, each on a fresh binary
  and each restored and `cmp`-checked; **V22** (`paintCannot` does not say a cut FOOTER) had been
  APPLIED at 11:06:15 and its binary linked (`build/test/shulib_tests`, 11:06:17), with no verdict
  written and no restore. Executor: **Claude Opus 5.5 (1M context)**, the builder again, same worktree.
- **The restoration, as found.** The COORDINATOR restored `include/shulib/calibration/calibrate_screen.hpp`
  from the run's own `V22.0` backup (identical to the `V21.0` pre-mutation backup) and bumped its
  timestamp (14:50:25). This resumption checked it: every A1 file equals its pass-2 checksum
  (`mut/v_pre2_md5.txt`, taken 10:57 before pass 2: 11 of 11 `OK`) and its newest mutation backup
  (`filecmp`, byte for byte). The two whose newest backups are older (`hal/brain_screen.hpp`, `M24`;
  the shim's `screen.h`, `M17`) differ from them only by this round's own R16 hedges and R6/R7 shim
  steps — read line by line, no mutant text. `git diff -- src/ Makefile` EMPTY: no gate plant left.
- **The binary on disk is the V22 MUTANT** (linked 11:06:17, after the mutation and before any
  restore) — NOT trusted. `build/test` is moved aside (`scratch/cal1-a1/old_build_test_v22_mutant`)
  and rebuilt from scratch after the briefing is regenerated; V22–V29 run again on the restored tree
  (pass 1's V22–V29, 10:52–10:54, predate this round's later edits to `calibrate_screen.hpp`,
  `brain_screen_test.cpp` and `screen.h`, so they do not count).
- **The dispositions above (R1–R18) were written before the pause** and are re-verified against the
  tree here, not rewritten; what this resumption found is logged below.
- **The binary rebuilt from source first** (14:52–14:53; logs `scratch/cal1-a1/w_*`): `briefing_status.py
  generate`, then `cmake -S test -B build/test` and `cmake --build build/test -j8` on the EMPTY tree:
  exit 0, 0 warnings; the suite on that binary `2283 passed | 0 failed | 3 skipped`, `2785288`
  assertions (`w_suite.log`) — 30 cases over `aaea302`'s 2,253 (A5b is the review's one new case).

**What the resumption found, and did:**

- **Its first try at V22–V29 (14:54) was NOT a valid run, and the runner said so.** A build tree made
  from scratch had not yet merged its compilers' header dependencies (`compiler_depend.make` named
  `calibrate_screen.hpp` 0 times — the Makefile generator merges them at the next FULL build, and
  the runner's `shulib_tests/fast` target skips that step), so `/fast` rebuilt nothing for a mutated
  header: V22, V24, V27, V28 "did not link — NOT A VALID RUN", and V25/V26, expecting a compile
  failure, saw "exit 0" with nothing compiled. V23's anchor was pass 1's `columnsOf`, since split in
  two (NOT RUN, anchor count 0). V29 (a `.cpp`) WAS valid: linked, RED. **Fixed:** one full `cmake
  --build` merged the dependencies (3 lines, and it recompiled the three test TUs over the bumped
  headers); V23's anchor moved to today's `columnsOf(const char*)` (the mutant counts bytes);
  the runner now refuses a compile-failure run that recompiled nothing (`mut/v_run_mut.py`). The
  re-run (14:57–14:59, `mut/v_run4.out`): V22–V24 and V27 RED, V25/V26 RED at compile time (the
  `allFit()` `static_assert`), V28 GREEN as its control must be.
- **The briefing was generated once from a MUTANT binary — caught, regenerated last.** After V29
  the binary on disk was V29's link; the doc gate's briefing check failed against it, and the
  `generate` that followed wrote "RED — 1 case(s) failing" into the generated block. The final
  sequence below rebuilds from scratch and regenerates the briefing AFTER the clean suite, so the
  committed block is the clean binary's. (Lesson, for the runner's next user: a mutation pass leaves
  the LAST mutant linked; nothing reads the binary after a pass until a clean build has run.)
- **R2b — a sibling of R2 the fix round missed: `paintNotDriverControl()` cut its status bits
  silently.** RE-CHECKED against the claims R2 wrote: the header ("Nothing is cut silently") and
  the changelog ("Text a caller passes — … the status bits … — … a screen that cut or dropped any
  of it says so on its last line") were false for this screen: `bits` is a caller's `const char*`,
  printed SMALL and cut at 50 with no word. (The draft's own caller builds at most 49 characters —
  `"status bits " + competitionWords()`, d6a/v3/p3r `calibrate_program.hpp` :236-242, :693-699 — so
  no screen today loses text; the promise was still the API's.) **Fixed at the root:** when
  `columnsOf(bits)` exceeds `kSmallCols`, a last SMALL line at y 204 says "the full text is on the
  serial log" (`kMoreOnSerial`, already in `kAll`); the header's sentence names the screen *(its "Nothing is
  cut silently" was itself an over-claim — narrowed in the verification round, below)*. **Test:**
  P7 — 50 characters exactly is drawn whole and NOT said (11 draws); 51 is cut and said (12, the
  last at y 204). P9's count of literals drawn by name rose 18 → 19 (the line above marks it).
  **V30** (the line never drawn) RED; **V31** (`>=` — said at exactly the budget) RED.
- **R14, re-checked with the concept greps the fix round named but did not log** *(they missed
  HANDOFF :263's "18 headers" — found and fixed in the verification round, below)* (run 15:00, over
  `README.md`, `docs/*.md`, `docs/guide/*.md` and the living internal documents — build-order,
  HANDOFF-remaining-work, REMAINING-WORK, ORIENTATION, PROJECT-BRIEFING, RESUMING; each hit READ):
  - `seventeen`: one hit, `PROJECT-BRIEFING.md` :193 — the history sentence ("seventeen since R3d
    Part 2 … eighteen since CAL1 Part A1"), true.
  - `17 adapter`: none.
  - `hardware adapters` / `adapters exist`: `README.md` :87 (Eighteen — true: 20 headers, minus
    `file_store` and `device_provider`); :118, `docs/README.md` :10/:37, `faq.md` :40, guide 08 :38,
    guide 13 :28, guide 14 :34, master plan :10, roadmap :1452, build-order :800 — none counts; all
    true; `changelog.md` :1452 — history.
  - `eighteen`: `README.md` :87/:98, `PROJECT-BRIEFING.md` :194 — A1's, true; `hardware-assumptions.md`
    :1106 — the register's own count of entries, not adapters. **And `ORIENTATION.md` :94 "the V5
    hardware layer (eighteen files)" — FALSE, and missed by both the A1.5 review and the fix round:**
    written at COMP-t (`0f56301`, 2026-09-29) when `hal/pros/` held 18 files, stale since SAFE-SAVE
    added `file_store.hpp` (`fab1ebb`, 19), and A1's `brain_screen.hpp` makes 20 (`git ls-tree` at
    each commit). **Fixed: "twenty files"** — a shared document's line, changed because A1's file
    makes it falser; recorded for the coordinator below.
  - `touch screen` / `touchscreen`: `README.md` :89/:93/:95 and `PROJECT-BRIEFING.md` :195/:196/:746
    (A1's, true); `changelog.md` :37 (A1's); `hardware-assumptions.md` :1205 (another program's
    words — untouched).
  - `format string` / `leak` (R16): `README.md` :102-103 and `changelog.md` :52-54, :66 — A1's,
    hedged. One unhedged statement remains OUTSIDE A1's lines: `build-order.md` :632, the R3d Part 2F
    clause's "(a live number at most 5 Hz: PROS leaks every print)" — not A1's to change; it joins
    the coordinator's R16 item. The rest are other subjects (guide 16 :137, a pose leak in the
    register, the API-doc leak in build-order :325).
- **R18, re-run against TODAY'S `shulib-v2`** — `fd36e07` (MECH-A1 landed after the fix round's dry
  run against `03a8081`). `git merge-file` of every file both branches changed since `aaea302`
  (scratch `w_merge/`): `README.md`, `REMAINING-WORK.md`, `mkdocs.yml` merge cleanly;
  **`docs/changelog.md` now CONFLICTS** — both branches added a section at the top of `## API 2.4`
  (MECH-A1's "the mechanism core, part 1" and A1's): keep BOTH, A1's first (newest), one blank line
  between (proven: `w_merge/changelog.resolved.md`, headings in order A1 → MECH-A1 → T4 Part 3 →
  …); **`build-order.md` :632** conflicts as before, now with MORE on main's side — `fd36e07`'s
  "MECH-A1 LANDED 2026-10-15 …" clause and its rewritten "NEXT: MECH-A2 …" clause beside
  `39124eb`'s Toggle AUTO-FLIPPER clause — all DISJOINT from A1's one insertion after "typed drive
  ports)": main's line plus A1's clause resolves it (proven: `w_merge/build-order.resolved.md`
  differs from main's file in that line only, and holds all three); `PROJECT-BRIEFING.md` conflicts
  only inside its GENERATED block; `docs/api/README.md` and `all-entities.md` conflict in generator
  output. The hand-over (item 4 above, marked superseded) is restated in the list at the end.
- **The re-verification of R1–R18, finding by finding:** every disposition above was read against
  the tree and holds — the fake's error read and `press()`/`release()` (`fake_brain_screen.hpp`),
  the shim's `screenPress`/`screenRelease`/`formatLetters` (`screen.h`; HA-178 (a), cited for the
  press count, is in the register at :340 as stated), A3/A5b/A6c/A6d (`pros_brain_screen_adapter_test.cpp`),
  P2/P3/P7/P8/P9 and S3 (`brain_screen_test.cpp`), F2's model and F4's comment (`front_check_test.cpp`),
  R15's comment against `state_screens.cpp` :36-49 (53, 53, 54 characters re-measured; why at 100,
  bits at 130), R16's hedges, R17's two notes. Every new test's mutation is in the table below, RUN
  AGAIN on the final code tree.

**The mutations of this fix round — the counted pass, every one on the FINAL code tree** (pass 4,
15:05:18–15:17:01, `mut/v_run5.out`; the runner backs up each touched file, applies the mutant,
builds `shulib_tests/fast` and REQUIRES the link line — or, for a compile-time mutant, a failed
build that compiled something — runs the WHOLE suite, restores byte for byte (`cmp`: identical,
every one) and bumps the timestamp; 28 fresh links + 3 compile failures; after the pass every A1
file matches its pre-pass checksum, `mut/v_pre3_md5.txt`). Pass 2 (10:57–11:06, V1–V21) and pass 3
(V22–V29) gave the same verdicts on the tree before R2b. *(The runner's labels in `v_mutations.json`
use the reviewers' pre-merge numbering — e.g. V7 says "R12/Q4" — the finding column here is this
log's.)*

| V | Finding | The mutant | Verdict | Caught by |
|---|---|---|---|---|
| V1 | R1 | the fake's error read keeps the last tap (only the flag set) | RED | S3, A5b |
| V2 | R6 | the adapter reports `press_count` as the release count | RED | A3, A5b |
| V3 | R7 | LARGE mapped to `E_TEXT_LARGE_CENTER` | RED | A2, A5 |
| V4 | R5 | an in-place repaint never records the new line (`have = want` removed) | RED | P3 |
| V5 | R8 | `paintCannot` leaves the record valid | RED | P7 |
| V6 | R8 | `paintStarting` leaves the record valid | RED | P7 |
| V7 | R11 | `samePlace` ignores x | RED | P2 |
| V8 | R3a | the advance hook fires BEFORE the clock moves | RED | A6c |
| V9 | R12(1) | `delay_until` calls the hook when the wake instant is exactly now | RED | A6c |
| V10 | R9 | `kAll` checks MEDIUM `kFindingDevices` at SMALL's budget | RED | P9 |
| V11 | R9 | its comment AND `kAll` both say SMALL; the painter draws it MEDIUM | RED | P9 (the call check) |
| V12 | R12(2) | `paintCannot`'s second fix line 18 px down, not 16 | RED | P7 |
| V13 | R12(3) | `opcontrol` declared after the host `main.h`'s `extern "C"` block | RED (compile: "conflicting declaration … with 'C' linkage") | A6d's redeclarations |
| V14 | R12(3) | the same, A6d's compile-time block removed too | RED | A6d's text check |
| V15 | R10 | `terminatedCopy` without its clamp | RED (`SIGABRT`, the stack protector) | P8 |
| V16 | R10 | `columnsOf(view)` reads the view's `.data()` past its end | RED | P8 |
| V17 | R2 | the state screen does not say a cut state NAME | RED | P7 |
| V18 | R2 | … a cut REASON | RED | P7 |
| V19 | R2 | … cut STATUS bits | RED | P7 |
| V20 | R2 | `paintCannot` does not say a cut HEADLINE | RED | P7 |
| V21 | R2 | … a cut HEADER word | RED | P7 |
| V22 | R2 | … a cut FOOTER | RED | P7 |
| V23 | R2 | `columnsOf` counts BYTES, not characters drawn | RED | P7 (the 17-character, 19-byte headline — *now five exact-budget texts, each over its budget in bytes: the second verification round*) |
| V24 | builder's own | the header word cut at MEDIUM's 36, not `kHeaderCols` | RED | P8 |
| V25 | R2 | `kStateWhyAutonomous` = the draft's 126-character reason | RED (compile: `allFit()`) | the `static_assert` |
| V26 | R2 | `kStateNameConnected` = `FIELD CONTROL CONNECTED` | RED (compile: `allFit()`) | the `static_assert` |
| V27 | R3b | a halved share with `>=` | RED | F1, F4, F6 |
| V28 | R3b CONTROL | a halved share with `>` — the same test in integers | GREEN, as it must be | — |
| V29 | R13 | F1's typed swap row mistyped, its counts typed to match | RED | F2 (the derivation) |
| V30 | R2b | `paintNotDriverControl` never says cut bits | RED | P7 |
| V31 | R2b | … says "cut" at exactly the budget (`>=`) | RED | P7 |

**For the coordinator — added by the resumption** (the lists above still stand, except item 4 of
the review's, superseded here):
1. **The merge onto `shulib-v2` at `fd36e07`** — *SUPERSEDED by the second resumption's re-run
   against `2506067`, where SER1 has landed (below)* (re-run; `scratch/cal1-a1/w_merge/`): (a)
   `docs/changelog.md` — keep BOTH new sections at the top of `## API 2.4`, CAL1 A1's first, one blank
   line before MECH-A1's heading (`changelog.resolved.md` shows it); (b) `build-order.md` :632 — take
   main's line and insert A1's one clause (", and its Part A1 LANDED 2026-10-15 (… A2a next — [live
   log](chunks/CAL1-PROGRESS.md))") after "D6 (a): a first record checked against typed drive ports)"
   (`build-order.resolved.md`); (c) `docs/api/README.md`, `all-entities.md`, `mkdocs.yml` — generator
   output: regenerate (`api_doc_tool.py generate`) after merging, never hand-merge; (d) run
   `briefing_status.py generate` LAST, after the merged suite binary exists (the briefing's only
   conflict is its generated block). README, REMAINING-WORK merge cleanly.
2. **A shared document's line changed:** `docs/internal/ORIENTATION.md` :94, "the V5 hardware layer
   (eighteen files)" → "(twenty files)" — stale since SAFE-SAVE (`fab1ebb`, 19 files), and A1's
   `brain_screen.hpp` makes 20. `shulib-v2` has not changed ORIENTATION since `aaea302`, so it merges
   cleanly; MECH-A1 added no `hal/pros/` file (19 there), so the merged tree has 20.
3. **R16's register question gains one line outside A1's:** `build-order.md` :632's R3d Part 2F
   clause says "PROS leaks every print" as fact; hedge it when the register decision is taken.
4. **For whoever runs the next mutation pass:** a build tree made from scratch must have one FULL
   `cmake --build` before `shulib_tests/fast` sees a header change (the dependency merge), and a pass
   leaves its LAST mutant linked — rebuild before any suite result or briefing is trusted.
5. **The restoration of V22 was checked, not assumed:** `calibrate_screen.hpp` equal to its `V22.0`
   and `V21.0` backups and to the pass-2 checksum; the V22 mutant binary moved aside
   (`scratch/cal1-a1/old_build_test_v22_mutant/`), never trusted.

**Every gate, re-run on the FINAL tree after the pass** (15:18–15:43 EDT; logs `scratch/cal1-a1/f_*`):
- **Host build from scratch** (`build/test` moved to `scratch/cal1-a1/old_build_test_after_pass4/`,
  the briefing regenerated first, then `cmake -S test -B build/test` and `cmake --build`): exit 0, 0
  warnings; the doc gates in it — `api_doc_tool self-test: OK`, coverage `3773 public entities across
  174 headers, all documented`, examples 521 lines verbatim, removability clean, briefing self-test OK,
  staleness self-test OK and `clean (29 live docs …)`.
- **The suite:** `2283 passed | 0 failed | 3 skipped`, `2785296` assertions, `Status: SUCCESS!`
  (`f_suite.log`). **Case by case against `aaea302`** (`cmpxml.py r_base_suite.xml f_a1_suite.xml`):
  2,253 → 2,283; **30 new** (brain_screen 12, front_check 8, the adapter and shim 10); **none gone;
  0 failures; no existing case's verdict changed**; the same eight scans' assertion counts rose as
  A1.4 lists (the retired-Teleop scan now 11,911 → 12,099, the test files having grown).
- **The briefing regenerated AFTER that suite** (`2,283 cases / 2,785,296 assertions, 3 skipped —
  green`, `Public headers: 212`, `OPEN CHUNK(S): CAL1, …`); a second `cmake --build` ran the doc gates
  against it (`generated block current`); `briefing_status.py check`: current.
- **`src_build_gate.py self-test`:** `OK (211 detector cases, each against a real build)`, 827.5 s;
  **`check`:** `PASS (all 7 builds link …)`, every build `exit 0, 11/11 TUs compiled, warnings: 8
  vendor-ignored, 0 disallowed`, 185.3 s; `git status -- src/ Makefile` EMPTY after each.
- **After the src gate** (its builds touch shared timestamps), one more `cmake --build` (it recompiled
  every TU) and the suite again: `2283 passed | 0 failed`, `2785296` assertions; `briefing_status.py
  check`: current — so the briefing's block is the binary on disk.
- **ARM header gate** (RESUMING's command over every `include/shulib` header — 212, A1's five among
  them; the TU in scratch): exit 0, no output. **Control** (`int x = d;`): exit 1, `error: conversion
  from 'double' to 'int' may change value [-Werror=float-conversion]`.
- **CI greps** as `ci.yml` spells them: `library is PROS-free (all of include/shulib except
  hal/pros/)`; `layering holds: core is sim-free`.
- **`api_doc_tool.py`** run explicitly: `self-test` OK, `check-coverage` 3,773 / 174, `check-fresh`
  exit 0 (`docs/api/` regenerated at 15:02 after R2b's `///` change — `calibrate_screen.md` :808
  carries the new sentence), `check-examples` 521 verbatim, `check-removability` clean.
  **`doc_staleness_audit.py`:** self-test OK; clean. **`briefing_status.py`:** self-test OK; check
  current.
- **`git diff -- src/ Makefile`:** EMPTY. `test/CMakeLists.txt`, `shim_control.hpp`, `ci.yml`,
  `tools/`: untouched. No existing test TU edited.

**Close of this fix round (15:43 EDT, `date`).** All 18 review findings dispositioned (R1–R17 fixed
at the root, R18 a merge hand-over, re-run against `fd36e07`); the resumption added R2b (fixed,
V30/V31) and the ORIENTATION count (fixed). 31 mutations, all on the final code tree, each as
expected. **Nothing committed, nothing pushed, no `pros`, no serial port; no new hardware belief
(the two PROS beliefs of R16 stay hedged and handed over, no HA number taken).**

## Verification-round findings and their disposition (2026-10-15 15:56–16:50 EDT, `date`)

> A fresh verifier returned two findings, both minor. Executor: **Claude Opus 5.5 (1M context)**,
> the builder again, same worktree and branch. Each finding was RE-CHECKED against the tree before
> anything changed; the new test's three mutations ran on FRESH binaries and were SEEN red (runner
> `mut/v_run_mut.py`, list `mut/x_mutations.json`, output `mut/x_run.out` and `mut/x_run2.out`, log
> `mut/v_mutations.log`); every gate was re-run (logs `scratch/cal1-a1/g_*`, `h_*`, `i_*`). The
> earlier entries this round corrects carry an italic marker pointing here; nothing else in them
> was rewritten.

- **F1 (minor): R14 was not fully fixed — `HANDOFF-remaining-work.md` :263 said "Adapters
  (`hal/pros/`, 18 headers)" and listed neither `file_store` nor `brain_screen`.** RE-CHECKED, REAL:
  `git ls-tree --name-only aaea302 include/shulib/hal/pros/` lists 19 (`file_store.hpp` among
  them, from SAFE-SAVE Part A, `fab1ebb`, 2026-10-04); the worktree has 20; the line is the same at
  `aaea302` and at `shulib-v2` (`fd36e07`). Its convention counts every header (it lists the device
  provider), so the true number is 20. HANDOFF's own header says "if this file is wrong about the
  tree … this file must be corrected in the same commit that notices it", and A1's file makes the
  line falser — the same case as ORIENTATION :94. **Fixed:** "(`hal/pros/`, 20 headers)", and the
  list gains "(SAFE-SAVE Part A, 2026-10-04) the SD store `ProsFileStore` built over the block sink
  and source, and (CAL1 Part A1, 2026-10-15) the brain's touch screen `ProsBrainScreen`" (both class
  names checked in their headers). A1.5's "Deliberately NOT touched … HANDOFF" and the resumption's
  R14 entry carry a correction marker. **Why the miss:** the concept searches looked for words
  (`seventeen`, `eighteen`, `17 adapter` …), never for a NUMERAL beside `hal/pros`. **The searches
  added, and run** (over `README.md`, `docs/*.md`, `docs/guide/*.md` and the six living internal
  documents; GNU `command grep` — see the note below; each hit READ):
  - `hal/pros` within 40 characters of `[0-9]+ (header|file|adapter)s?`, either order: master plan
    :235 (no count — "only files that include <pros/*>"); HANDOFF :263 (now 20 — fixed).
  - a count (`fourteen`…`twenty-one`, `14`–`22`) within 30 characters before `adapters`, `adapter
    headers`, `headers` or `files`: `README.md` :87 ("Eighteen hardware adapters" — true by its own
    count, which leaves out `file_store` and the device provider: 20 − 2); `PROJECT-BRIEFING.md`
    :747 ("18 adapter headers" — the same convention, true); `ORIENTATION.md` :94 ("twenty files",
    fixed in the resumption); HANDOFF :263; the rest are not `hal/pros` counts (master plan :573/:574
    are decision-table row numbers; changelog :1389 a 2026-08-14 heading; build-order :644, :845,
    :2151 — a date, a plan table, `src/`'s 22 files in SRC1's history).
  - The fix round's word searches re-run with GNU grep (`seventeen`, `17 adapter`, `hardware
    adapters`, `adapters exist`, `eighteen`, `nineteen`, `touch ?screen`, `format string`): the same
    hits the resumption logged, plus HANDOFF :267 (this fix's own words).
  - **Note for whoever greps next:** this shell's `grep` is a function that runs `ugrep` with
    `--ignore-files`; with `-niE` and `\b…\b` it returned NOTHING for `ORIENTATION.md` :94 ("twenty
    files"), which matches (GNU grep finds it). This round's searches use `command grep`. The fix
    round's and the resumption's word searches had no `\b`; re-run with GNU grep, they give the same
    hits, so nothing they reported is in doubt.
  - **Found and NOT changed (handed over):** HANDOFF §4's "Calibration:" list (:270-271) names only
    `drivetrain_calibrator`, `calibration_record` and `boot_check`. It states no count and is an
    incomplete list, not a false one — it already lacked SAFE-SAVE's `record_store.hpp` and R5a's
    `feedforward_fit.hpp` / `drive_characteriz*.hpp` / `rest_capture.hpp`. Adding only A1's two
    headers would make a partial list look complete, and the rest are other chunks' lines, so it is
    left for the coordinator (below).
  - **The merge:** `git merge-file` of HANDOFF against `fd36e07` (base `aaea302`; scratch
    `g_merge/`) — CLEAN. Main's own edits (:272-280, MECH-A1's data-layer sentence) are disjoint, and
    the merged file holds both.
- **F2 (minor): a new over-claim from the fix round — "Nothing is cut silently."** (the
  `calibrate_screen.hpp` file comment, copied into `docs/api/calibrate_screen.md`). RE-CHECKED, REAL:
  `render()` cuts a view's header word at `kHeaderCols` (`paintHeader`), each line at its size's
  budget and each button label at MEDIUM's (both through `printPanelText`), and the header cuts the
  build time at `kBuildTimeCols`. None of these says anything on screen, and P8 pins the header-word
  and build-time cuts as silent (`hw.log().size() == 4`; the 7-entry `paintStarting` log). The
  sentence was absent from the draft (`d6a/v3/p3r`) and from `M25`'s backup, and first appears in
  the fix round's `V10.0`. Today no screen loses text: `calibrate_panel_test`'s `checkText` checks
  every view its runs reach (header word ≤ `kHeaderCols`, each line ≤ `budgetFor(size)`, each label
  ≤ 12). **The defect is the promise, so the promise is fixed — narrowed to what is true:** an own
  screen that cuts a CALLER's text says so. Two cuts are NOT said, because they are clamps that keep
  the layout whole: `render()`'s (header word, lines, labels) and the build time's (on every screen
  that has a header). The text says why `viewFor()`'s views are not expected to reach them. The
  constructor's and `render()`'s own `///` comments now say "without a word on screen"; `docs/api/`
  is regenerated. **Alternative NOT taken:** making `render()` and the constructor announce their cuts
  would add a line to the panel's layout (it has buttons at y 180–234) and change behaviour that P8
  pins, to guard against views the panel's own test already proves fit. **The changelog** said
  nothing false before, but read beside its previous sentence it implied the header cuts were said.
  It now names the two kinds of cut that are not said. Its first wording in this round was itself
  wrong ("those cuts … are NOT said" took in the header word, which `paintCannot()` DOES say — P7,
  V21). This was caught on re-reading and corrected before any gate counted: "`render()`'s of a view's
  header word, lines and button labels … `paintCannot()` does say a header word it cut".
  - **A test for the newly documented claim** (a doc statement no test held, so a test was owed): P8
    gains a `render()` of a 30-character LARGE line, a 45-character MEDIUM line and a 40-character
    button label. They are drawn as 18, 36 and 36 characters (the label 4 px into its box, since 40 ×
    13 px is wider than 295 px), and the log holds exactly 10 draws, with no "full text" line. P8's
    name is unchanged (the case-by-case comparison keys on it); the file's test list says it.
    Assertions 2,785,296 → 2,785,302 (P8 26 → 32; no other case changed — `cmpxml.py`).

| V | Finding | The mutant | Verdict (16:26–16:28, then again on the final tree 16:44–16:46) | Caught by |
|---|---|---|---|---|
| V32 | F2 | `render()` cuts a view's line at SMALL's 50 whatever its size | RED, RED | P8 (the LARGE and MEDIUM lines) |
| V33 | F2 | a button label cut at SMALL's 50, not MEDIUM's 36 | RED, RED | P8 (the label) |
| V34 | F2 | `render()` SAYS a cut line ("the full text is on the serial log" at y 218) | RED, RED | P8 (`REQUIRE` 10 draws); P9 (19 drawn by name) |

Each one was run by the runner: a backup, the mutant applied (anchor count 1), `shulib_tests/fast`
built with the link line REQUIRED, the WHOLE suite run, the file restored byte for byte (`cmp`:
identical) and its timestamp bumped. After each pass both touched files matched their pre-pass
checksums (`mut/x_pre_md5.txt`, `mut/x_pre2_md5.txt`: OK). The second pass ran because the
comments were reworded after the first; the code anchors had not changed.

**A slip in the gate sequence, caught:** after the first pass the binary on disk was V34's mutant.
`cmake --build` (16:29) then FAILED in the doc-gate target, because the briefing's suite line no
longer matched (`RED — 1 case(s) failing`). That target runs before the suite target, so nothing
was rebuilt and the briefing was NOT written. `--target shulib_tests` failed the same way. The
`shulib_tests/fast` target rebuilt the two TUs over the restored header, and the suite was green
again. The tree was then rebuilt from scratch (below). This is the resumption's lesson #4 from the
other side: after a pass, rebuild through `/fast` first, because the full build's doc gates read the
binary the pass left behind.

**Every gate, re-run** (the final code tree from 16:40; the src gate ran 16:07–16:24, after F1/F2's
first edits):
- **Host build from scratch** (16:35; `build/test` moved to `scratch/cal1-a1/old_build_test_after_xpass/`
  — the one before it, `old_build_test_before_vround/`, at 16:00 — briefing generated first): exit 0,
  0 warnings; its doc gates passed (`api_doc_tool self-test: OK`, coverage 3,773 / 174, 521 lines
  verbatim, removability clean, briefing self-test OK, staleness clean). The suite: `2283 passed | 0
  failed | 3 skipped`, `2785302` assertions. Against `aaea302` (`r_base_suite.xml`): 30 new, none
  gone, 0 failures. Against the fix round's final run (`f_a1_suite.xml`): the only change is P8, 26 →
  32 assertions.
- **After the final rewording** (16:40, comments only): one more full build (it recompiled the two
  TUs that include `calibrate_screen.hpp`), the suite green (2,283 / 2,785,302), the case-by-case
  comparison unchanged. After V32–V34's second pass: `/fast` (the same two TUs, linked), the suite
  green, a full build (doc gates: `generated block current`).
- **The briefing** was regenerated after the suite (`2,283 cases / 2,785,302 assertions, 3 skipped —
  green`, `Public headers: 212`). `briefing_status.py check`: current. `self-test`: OK.
- **`src_build_gate.py self-test`:** `OK (211 detector cases, each against a real build)`, 13 m 55 s;
  **`check`:** `PASS (all 7 builds link …)`, each `exit 0, 11/11 TUs compiled, warnings: 8
  vendor-ignored, 0 disallowed`, 3 m 30 s; `git status -- src/ Makefile` EMPTY after each. The later
  edits do not reach it: they are a test TU and comments in `calibrate_screen.hpp`, and no file
  under `src/` includes `calibrate_screen.hpp`, `hal/brain_screen.hpp` or `front_check.hpp` (`grep
  -rl` over `include src test` finds only A1's own headers and its three tests).
- **ARM header gate** (all 212 `include/shulib` headers, TU `scratch/cal1-a1/h_arm_all.cpp`) on the
  final tree: exit 0. **Control:** `int x = d;` gives `error: conversion from 'double' to 'int' may
  change value [-Werror=float-conversion]`, exit 1.
- **CI greps** as `ci.yml` spells them: `library is PROS-free (all of include/shulib except
  hal/pros/)`; `layering holds: core is sim-free`.
- **`api_doc_tool.py`:** `self-test` OK; `check-coverage` 3,773 / 174; `check-fresh` exit 0
  (`docs/api/` regenerated after each `///` change — `calibrate_screen.md` carries the narrowed
  text); `check-examples` 521 verbatim; `check-removability` clean. **`doc_staleness_audit.py`:**
  self-test OK; clean (29 live docs).
- `git diff -- src/ Makefile test/CMakeLists.txt .github tools/ test/pros_shim/shim_control.hpp`:
  EMPTY. No existing test TU edited; `brain_screen_test.cpp` is A1's own.

**The prose review of this round (pace rule 4).** Each new sentence was read against the code. The
header comment, the constructor's and `render()`'s `///`, and the changelog's sentence were checked
against `paintHeader`, `printPanelText`, `paintButton` and P7/P8. One error was found and fixed: the
changelog's first wording, above. In HANDOFF :263, each name and date was checked (`ProsFileStore`
in `file_store.hpp`:59, `ProsBrainScreen` in `brain_screen.hpp`:37, `fab1ebb` dated 2026-10-04).
The test's comments were checked against its own draws (x 10 = 6 + 4; 10 draws). Searched for
other cut claims (`cut silently`, `nothing is cut`, `never cut`, `no silent`; GNU grep over
`README.md`, `docs/`, `include/` and A1's test): in A1's files, only this log's history entries.

**For the coordinator — added by this round** (the lists above still stand):
1. **A second shared document's line changed:** `HANDOFF-remaining-work.md` :263 — "18 headers" →
   "20 headers", plus `ProsFileStore` (SAFE-SAVE) and `ProsBrainScreen` (CAL1 A1) in its list. It
   merges cleanly onto `fd36e07` (proven, `scratch/cal1-a1/g_merge/`).
2. **The changelog's resolution, re-proven with this round's text:**
   `g_merge/changelog.resolved.md` (A1's section first, then MECH-A1's; it differs from main's file
   only by additions) supersedes `w_merge/changelog.resolved.md`.
3. **HANDOFF §4's "Calibration:" list** is incomplete since SAFE-SAVE and R5a, and now A1
   (`record_store`, `feedforward_fit`, `drive_characteriz*`, `rest_capture`, `front_check`,
   `calibrate_screen`). It states no count, so nothing in it is false. Completing it touches other
   chunks' lines, which is the coordinator's call.
4. **Tooling note:** this shell's `grep` wraps `ugrep --ignore-files`. On a `\b`-bounded `-iE`
   pattern it missed a matching line (above). Use `command grep` for concept searches.

**Close of the verification round (16:50 EDT, `date`).** Both findings fixed at the root: F1 the
count and list (plus the search that would have caught it), F2 the over-claim narrowed to the
truth, with a test (V32–V34 RED). **Nothing committed, nothing pushed, no `pros`, no serial port; no
new hardware belief; no HA number taken.**

## The fix round's second resumption, after the freeze (2026-10-15 21:48–22:30 EDT, `date`)

> **Why.** The session ended twice (about 17:40 and 20:33). At about 20:34 the laptop froze: a
> runaway mutant test binary in ANOTHER tree (the coordinator's SER1 run on main) grew to 58 GB.
> This run was asked to finish every finding not yet dispositioned, re-verify the logged ones, re-run
> every gate and log the resumption. Executor: **Claude Opus 5.5 (1M context)**, the builder again,
> same worktree and branch.
>
> **The memory rule (2026-10-15) was followed throughout.** Every run of `build/test/shulib_tests`
> was capped as `(ulimit -v 4194304; timeout …)`, and so was every tool that runs it:
> `briefing_status.py generate` and `check`, and `cmake --build`, whose doc-gate step runs
> `briefing_status.py check`. So were every probe and the src gate. Nothing was killed by the cap
> or the timeout. Logs are under `scratch/cal1-a1/z_*`.

**What the earlier sessions left.** This log ended at 16:50, when the verification round closed.
Every finding (R1–R18), the resumption's R2b and the verification round's F1/F2 had a disposition,
and every mutation (V1–V34) was logged as run on a fresh binary. `find -newermt "2026-10-15 16:50"`
over the worktree (`build/` left out) finds only this log, so neither later session changed a file
here. The 20:33 session left one script, `scratch/cal1-a1/y_resume/cmp_backups.py`, with no output.

**The audit, before anything was built.**
- `cmp_backups.py`, run as-is, compares each file with its NEWEST mutation backup. Nine were
  IDENTICAL. Two DIFFER, and both diffs were read line by line:
  - `hal/brain_screen.hpp` against `M24`: only R16's two hedges.
  - The shim's `screen.h` against `M17`: only R6's `screenPress`/`screenRelease` and R7's
    `formatLetters`.
  - (`M7.calibrate_screen.hpp.bak` is an orphan with no JSON record. It is older than the
    calibrate_screen backups that have records, the newest being `V34.0`, identical.)
- Checksums against the last passes' records:
  - `calibrate_screen.hpp` and `brain_screen_test.cpp` match `mut/x_pre2_md5.txt`, taken before the
    final V32–V34 pass.
  - The other 11 A1 files match `mut/v_pre3_md5.txt`, taken before pass 4.
  - `calibrate_screen.hpp` against `V31.0` (the pass-4 tree) differs in COMMENTS only (three
    hunks, the verification round's F2 wording).
- `git diff -- src/ Makefile`: EMPTY. **No mutant and no gate plant was left applied.**

**The binary was rebuilt from source before any result.** The binary on disk was linked at
16:46:11 by the `/fast` relink after V32–V34's second pass, so it was not trusted. Steps:
1. `build/test` moved to `scratch/cal1-a1/old_build_test_resume3`.
2. `briefing_status.py generate` with no binary (the briefing reads it as unbuilt).
3. `cmake -S test -B build/test`, then `cmake --build` (21:50:49–21:51:35): exit 0, 0 warnings.
   The doc gates in it passed.

**The suite on that binary:**
- `2283 passed | 0 failed | 3 skipped`, `2785302` assertions; 19.5 s; peak RSS 57,984 KB (`/usr/bin/time -v`).
- Case by case (`cmpxml.py`) against the verification round's final run (`i_suite2.xml`): IDENTICAL
  (0 new, 0 gone, 0 changed).
- Against `aaea302` (`r_base_suite.xml`): 30 new, none gone, 0 failures. The same eight source
  scans' counts rose, as A1.4 lists.

Then the briefing was regenerated after the suite. The second `cmake --build` reported the doc
gates' `generated block current`, and `briefing_status.py check` reported current.

**Re-verifying the dispositions.** Since the last mutation pass, the code tree has changed only by
the comments above and P8's added assertions. Every logged mutation was observed on a fresh binary,
so none was re-run. That would redo finished work. As an INDEPENDENT check, the reviewers' OWN
probes were recompiled against the final tree in `scratch/cal1-a1/z_probe/` and run capped
(`probes.out`). Each now shows the fixed behaviour:

- **R1** (`r2/probe/touch_fidelity.cpp`): the fake's error read is `error=1 release=-1 x=-1 y=-1`,
  the same as the PROS adapter's. A baseline taken from an error read now finds **phantom taps:
  fake 1, PROS adapter 1**; the review measured fake 0, adapter 1. **`probe_touch.cpp`:** a naive
  reader "sees a tap" on BOTH; the review saw `no` on the fake and `YES` on the adapter.
- **R2** (`r2/probe/state_cut.cpp`, the draft's words): `FIELD CONTROL CONNECTED` is still drawn as
  `FIELD CONTROL CONN`. The 126-character AUTONOMOUS reason still loses its end. Both screens now end
  with `print S 10 210 … 'the full text is on the serial log'`; the review saw no such line.
  `DISABLED`, whose words fit, has no such line.
- **R5/Q1** (`probe_q1.cpp`): 0 prints in 100 unchanged ticks after a colour change.
- **R3a/Q5** (`probe_q5.cpp`): in the hook, `micros()=25000, to=25000 -> the clock had moved`.

**Probe: the painter against TODAY'S program** (the correctness review's `painter_diff.cpp`, with its
verbatim copy of today's painter). The probe feeds the same random sequences to both painters: views,
times, invalidations and own screens. It then compares the shim's draw logs, and the C-call logs with
redundant pen/eraser sets dropped, after the one sanctioned word mapping.
- **As written** (`painter_diff.out`, `painter_diff2.out`), seeds 1–8 × 20,000 steps, 735,512 draws:
  - every difference is ONE class, a header word cut at `kHeaderCols` (24) where today's program cuts
    at MEDIUM's 36. There are 3,813 such draws; a classifier counts each (`painter_diff2.cpp`), and
    the "other" count is **0**;
  - draw counts and print counts are equal.
- **With the probe's one over-long header word made 24 characters** (`painter_diff3.cpp`, `.out`), the
  same 8 seeds give draws `same=1`, calls `same=1`, and equal prints on every seed.
- So outside its header-word cut, the library's painter draws, byte for byte, what today's program
  draws, in the same call order.
- The probe never passes `paintCannot` an over-long header word, headline or footer, and never calls
  the state screen. Those differences are known by construction and listed below for Part B.

**R18, re-run against TODAY'S `shulib-v2`.** SER1 landed there as `2506067` (committed 21:53).

How the trial merge was built (`scratch/cal1-a1/z_merge/`, script `merge.sh`):
- `git archive shulib-v2` was extracted into a scratch tree, with no git metadata.
- A1's new files were copied in, as were its two files that main has not changed (`ORIENTATION.md`,
  the shim's `rtos.hpp`).
- `git merge-file` was run with `aaea302` as base on every file both branches changed.
- No name collides: main added nothing under `test/pros_shim/`, and none of A1's new paths.

The merge results:
- **CLEAN:** `README.md`, `HANDOFF-remaining-work.md`, `REMAINING-WORK.md`, `mkdocs.yml`.
- **CONFLICT, `docs/changelog.md`:** three sections now sit at the top of `## API 2.4`. Resolved as
  A1's, then SER1's, then MECH-A1's: newest landing first, one blank line between. The resolved file
  differs from main's only by A1's added lines.
- **CONFLICT, `build-order.md` :632:** resolved as main's line with A1's one clause inserted after
  "D6 (a): a first record checked against typed drive ports)" (the anchor occurs once). The result
  differs from main's file only in that line and in A1's CAL1 status block.
- **CONFLICT, `PROJECT-BRIEFING.md`:** only inside the generated block (regenerate it).
- **CONFLICT, `docs/api/README.md` and `all-entities.md`:** generator output (regenerate it).

**The merged tree, gated:**
- `api_doc_tool.py generate`, then `self-test` OK.
- `check-coverage`: 4,624 entities / 181 headers. This includes SER1's new rule, which fails on any
  name the reader cannot read.
- `check-fresh`: exit 0. `check-examples`: 541 lines verbatim. `check-removability`: clean.
- A1's four `docs/api/` pages come out of main's new generator BYTE-IDENTICAL to the worktree's.
- The briefing was regenerated after the suite. `briefing_status.py` self-test OK; `check`: current.
- `doc_staleness_audit.py`: self-test OK; clean.
- Host build: 0 warnings. Suite: **2383 passed | 0 failed | 3 skipped**, 2,791,532 assertions. That is
  main's 2,353 plus A1's 30, and all 30 of A1's cases have the same assertion counts as in the
  worktree.
- ARM header gate: all 220 headers, exit 0; the control fails as it must.
- CI greps: both pass.
- *(The tree's first build had no git identity: git was fenced off with
  `GIT_CEILING_DIRECTORIES`, so nothing touched the main repository. Two cases failed BY DESIGN,
  `session_header_test`'s hash case and `run_report_e2e_test`'s C5 case: "build MISSING". A throwaway
  repository inside the scratch tree gave it a hash, `2379d27`, and those runs are the ones counted.)*

**SER1 is on `shulib-v2` now.** A1 writes no serial line: none of its five headers calls
`printf`/`puts`/`stdout`, and the fake's `snprintf` is a buffer write that SER1's gate allows. The
hand-over stands, now as a fact: A2's `ProsCalibrateIo::line()` is ONE call to
`hal::pros::serialLine()` (`hal/pros/char_sink.hpp` on `2506067`). SER1's src-gate step 9 (no stdio
printer in a src/ object) does not reach A1, because no src/ file includes an A1 header.

**Every gate on the worktree's final tree** (logs `z_*`, `z_wt_gates/`):
- **Host build from scratch, and the suite:** above.
- **CI greps:** `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core is sim-free`.
- **ARM header gate:** 212 headers (TU `z_wt_gates/arm_all.cpp`), exit 0, no output. **Control**
  (`int x = d;`): exit 1, `error: conversion from 'double' to 'int' may change value
  [-Werror=float-conversion]`.
- **`api_doc_tool.py`:** `self-test` OK; `check-coverage` 3,773 / 174; `check-fresh` exit 0;
  `check-examples` 521 verbatim; `check-removability` clean.
- **`briefing_status.py`:** `self-test` OK; `check` current.
- **`doc_staleness_audit.py`:** self-test OK; clean (29 live docs).
- **`src_build_gate.py self-test`:** `OK (211 detector cases, each against a real build)`,
  22:04:45–22:18:59.
- **`check`:** `PASS (all 7 builds link …)`, 22:18:59–22:21:59. Every build was `exit 0, 11/11 TUs
  compiled, warnings: 8 vendor-ignored, 0 disallowed`.
- `git status` was the same list before and after the src gate. `git diff -- src/ Makefile
  test/CMakeLists.txt include/shulib/config tools .github`: EMPTY.
- The coordinator's own src gate self-test was running in the MAIN tree at the same time (pid
  70452, cwd `/home/gonzei/projects/shulib`). It was a separate tree, so it did not interfere.
- No host build ran here while this gate's self-test was planting. (Its plants include
  `test/CMakeLists.txt`, `include/shulib/config/robot_config.hpp` and `src/`.)
- **After the src gate** (its restores bump timestamps): one more `cmake --build` (22:23), which
  recompiled all 212 TUs with 0 warnings, and its doc gates passed. The suite again: `2283 | 0
  failed`, `2785302` assertions, case by case IDENTICAL to the run above. `briefing_status.py
  check`: current (22:26:49). The briefing's block therefore describes the binary on disk.
- No existing test TU was edited. `shim_control.hpp`, `ci.yml` and `tools/` are untouched.

**For the coordinator, added by this resumption** (the lists above still stand, except the
resumption's item 1, marked superseded):
1. **The merge onto `shulib-v2` at `2506067`**, proven in `scratch/cal1-a1/z_merge/tree`:
   (a) `docs/changelog.md`: A1's section, then SER1's, then MECH-A1's, at the top of `## API 2.4`.
   (b) `build-order.md` :632: main's line plus A1's one clause after "typed drive ports)".
   (c) `docs/api/README.md`, `all-entities.md`, `mkdocs.yml`: run `api_doc_tool.py generate`;
   never hand-merge them.
   (d) `PROJECT-BRIEFING.md`: run `briefing_status.py generate` LAST, after the merged suite binary
   exists. README, HANDOFF and REMAINING-WORK merge cleanly. Expect 2,383 cases.
2. **Part B's side-by-side list: the differences from TODAY'S program the A1.1 list does not hold.**
   - **The draft's state-screen layout:** `why` wrapped onto two SMALL lines at y 100/116, where
     `benchStateScreen` prints one unwrapped line at 100 that VEXos clips. The status bits are at
     y 136 (today 130). Every line goes through the sanitizer and its budget, where today's calls
     `screen_print_at` directly.
   - **The review round's own differences:**
     - (a) the header word is cut at `kHeaderCols` 24, where today's program cuts at MEDIUM's 36.
       This is the only difference the probe above finds, and no view today carries a word over 24.
     - (b) `paintCannot` turns its last line into "the full text is on the serial log" when it cut a
       HEADER word, headline or footer. Today's program says so only for a cut reason or fix.
     - (c) `paintNotDriverControl` adds a line at y 204 for status bits past 50. Today's program
       builds at most 49, so this line is never reached.
     - (d) the state screen's own words: `FIELD CONNECTED` for today's `FIELD CONTROL CONNECTED`,
       which VEXos clips at the screen's edge; the AUTONOMOUS reason re-cut to 94 characters; and a
       cut said at y 210.
3. **A capping gap the coordinator's TODO does not name:** it covers `briefing_status.py generate`,
   but `check` derives the suite line from the binary too (`suite_counts()`). `check` runs as
   `cmake --build`'s doc-gate step whenever `build/test/shulib_tests` exists. A mutant binary left
   linked is therefore run, UNCAPPED, by the next plain `cmake --build` as well. Until
   `suite_counts()` caps itself, wrap `cmake --build` in the cap too, or remove the binary first.
4. **Disk:** seven moved-aside build trees, about 1.1 GB, are under `scratch/cal1-a1/old_build_test_*`;
   all are evidence only. The trial-merge tree is under `z_merge/`, with its build and its throwaway
   `.git` inside (`.claude/` is git-ignored in the main repository).

**Close of the second resumption (22:30 EDT, `date`).** No finding was left without a disposition.
All 18 review findings, R2b, F1 and F2 were re-verified: the reviewers' own probes show every
reported defect gone, and the painter is byte-identical to today's outside its one intended cut.
Every gate passed on a binary rebuilt from source, and the merge onto today's `shulib-v2` was proven
green. After this entry was written, `check-fresh`, `check-removability`, `briefing_status.py check`
and the staleness audit were run again: all exit 0. **Nothing committed, nothing pushed, no `pros`,
no serial port. No new hardware belief and no HA number taken. No test binary ran uncapped.**

## Verification-round findings and their disposition — the second verification round (2026-10-15 22:46 EDT, `date`)

> A fresh verifier returned three findings, all minor (its log:
> `scratch/cal1-a1/verify/f3/vmut.log`). Executor: **Claude Opus 5.5 (1M context)**, the builder
> again, same worktree and branch. Each finding is RE-CHECKED against the tree before anything
> changes; each fix is at the root; every mutation runs on a FRESH binary, capped, and is SEEN red.
> Scratch for this round: `scratch/cal1-a1/k_vr2/`. The earlier entries this round corrects carry an
> italic marker pointing here; nothing else in them is rewritten. Appended as the work goes.

**The state found (22:46).** `git status` is the list the second resumption left. The sources equal
the tree its last gates ran on. `git diff -- src/ Makefile` is EMPTY. No mutant and no gate plant are
applied.

**The method this round.** The worktree itself is never mutated. `k_vr2/sync.sh` copies the
worktree's `include/`, `test/` and `src/` into `k_vr2/mtree/` and checksums them. For each mutant,
`k_vr2/vmut.py` does the following:
- backs up the file and applies the mutant (the anchor must occur exactly once);
- builds a FRESH binary, compiling `test_main` plus A1's three test TUs from source with the suite's
  own flags (`-Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion
  -Wdouble-promotion`, C++20);
- runs it as `(ulimit -v 4194304; timeout 600 …)`;
- restores the file byte for byte in `finally` (`cmp`-checked) and bumps its timestamp.

Those three TUs are the only ones that include or read `calibrate_screen.hpp` (`grep -rl` over
`include src test tools`), so the binary holds every case a mutant here can reach. No mutant was
killed by the cap or the timeout. Logs: `k_vr2/vmut_pre.log` (before the fixes), `vmut_post.log`
(after them) and `vmut_final.log` (the final tree, after the last comment change).

- **V1 (minor): four of the review round's new "say it was cut" checks had no exact-budget test, and
  P7's comment claimed one it did not have.** RE-CHECKED, REAL, and wider than reported.
  - **Before the fixes**, on fresh binaries: N1–N4 (`>=` for `>` on the headline, the state name,
    the header word and the footer) were all GREEN, as the verifier found.
  - **A fifth sibling the finding did not name:** N10, the same `>=` on the STATE screen's status
    bits (`paintStateScreen`), was GREEN too. Every test's state-screen bits were 48 characters or
    fewer, or far over 50.
  - **The other side of each boundary was untested as well:** a check one past the budget (`>
    budget + 1`, so a text cut by ONE character says nothing) was GREEN for all five (N1p, N2p, N3p,
    N4p, N10p). Every over-budget test text was several characters over.
  - **Control:** N11p, the same `+ 1` on the not-driver-control screen's bits, was RED. V31's pair
    (50 and 51 characters) already pins both sides there. So the method sees red.
  - **P7's comment:** `FRONT — UNCHECKED` is 17 characters and 19 bytes, and LARGE's budget is
    18. It caught the byte count (V23), not the exact budget. This is the same defect class as
    the review round's R3: a comment claiming a catch that was not real.
  - **The `>` checks in the library were right, so no library code changed; the tests were
    missing.** P7 now tests each of the five texts a caller passes that a screen checks against a
    budget, at EXACTLY that budget and at one character over:
    - paintCannot's header word (24), headline (18) and footer (50);
    - the state screen's name (18) and status bits (50).
  - At the budget, each is drawn whole and the last line is "the census is on the serial log" (or
    no "full text" line on the state screen). One over, each is cut back to the same characters
    and said.
  - Each exact text carries an em dash (one character drawn, three bytes), so each is also over its
    budget in BYTES and pins V23. Each text's drawn length is REQUIREd equal to its budget
    constant, so a mis-sized edge case fails loudly instead of testing nothing (N14 below). That
    REQUIRE caught this round's own first draft: a 51-character "50-character" bits text.
  - P7's comment and the file's test list now say what is caught. The R2 entry and V23's table
    row, above, carry a marker.
- **V2 (minor): the compile-time word check had no negative test.** RE-CHECKED, REAL, and wider than
  reported.
  - **Before the fixes**, all GREEN: N5 (the wrapped branch returns true), N6 (`wrapsCompletely`
    ignores the wrap), N7 (a one-line literal measured against its own length).
  - **Also GREEN before the fixes**, the same class:
    - N5b: the wrapped branch drops its ASCII and percent check.
    - N6b and N6c: any width, or any line count, taken as the state screen's shape.
    - N12: `allFit()` walks the list without looking.
    - N13: the header's `static_assert` removed outright.
  - Each would have left every literal TODAY fitting, with no red. The next over-long word would
    then fail only at P9's runtime loop, not at the build the header promises.
  - **Fixed, three ways:**
    - **(a) `screen_text::allFit` now takes the list it checks** (`template <std::size_t N>
      allFit(const Checked (&list)[N])`). The header's assertion is `static_assert(allFit(kAll),
      …)`, so the loop can be tested on a list with a bad entry. This changes A1's own
      not-yet-released function: nothing outside A1 calls it (`grep`: only the header and P9;
      `src/programs/calibrate_program.cpp`'s `allFit()` is that program's own). The changelog names
      no `allFit`, so it is unchanged. The `///` comment says what the host test holds, and
      `docs/api/` is regenerated.
    - **(b) P9 gains 15 `static_assert`s**, evaluated by the compiler, so a weakened check fails to
      BUILD the test:
      - one line: exactly the budget fits (`EIGHTEEN CHARACTER`), and one over, a `%` or a
        non-ASCII byte does not;
      - wrapped: `kStateWhyDisabled` fits, and three refusals — the draft's 126-character
        AUTONOMOUS reason, a wrapped reason with a `%`, and the shapes no screen draws (MEDIUM's
        width, three lines), through both `fits()` and `detail::wrapsCompletely()`;
      - lists: a good one passes, and one with a bad LAST entry and one with a bad FIRST entry are
        both refused.
    - **(c) A runtime source pin:** the header holds `static_assert(allFit(kAll), "calibrate_screen:
      …` at the START of a line, so removing it or commenting it out turns P9 red.
  - **Residual, stated:** an assertion disabled by the preprocessor (`#if 0` around it) still passes
    the text pin. Guarding that would need a compile-refusal harness. One exists, MECH-A1's
    `test/compile_refusals/`, but it is another chunk's and is wired through `test/CMakeLists.txt`,
    which A1 does not edit. So the pin is the proportionate guard here.
- **V3 (minor): the log still called the README and ORIENTATION untouched, and its "hedged
  everywhere" claim was false.** RE-CHECKED, REAL.
  - **The progress log, corrected with markers** (nothing else in those entries rewritten):
    - A1.5's "Deliberately NOT touched" now marks the README (changed in R14) and ORIENTATION :94
      (changed in the fix round's resumption).
    - A1.6's DoD documents line and its coordinator item 3 now say they are incomplete and point to
      the complete list below.
    - The fix round's coordinator item 2 ("hedged everywhere A1 states them") is marked as not true
      when written.
  - **The root of item 2's falsehood is fixed, not narrowed.** The verifier named two test comments.
    A grep of A1's 13 files (`leak`, `format`, `choke`, `conversion with`) found SEVEN that stated
    or presumed a PROS belief as fact:
    - `brain_screen_test.cpp`: S1 (a `%` "read as a conversion"), S3 ("the robot would choke on"),
      P2 ("PROS leaks every print"), P3 ("the leak the throttle exists to stop"), P4 ("PROS 4.2.2
      leaks every print's buffer"), P5 ("(a leak)");
    - `pros_brain_screen_adapter_test.cpp`: A4 ("a program's leak report").
  - Each now says BELIEVED and points to the header that states the belief and its source.
  - **Checked and left unchanged:**
    - the shim's `screen.h` :23-24 ("print_at formats with vsnprintf, as PROS hands the formatted
      text on"): this is PROS's DOCUMENTED API (vendored `include/pros/screen.h` :645, `\param text
      Format string`), not R16's belief that VEXos re-reads the formatted text;
    - `hal/pros/brain_screen.hpp` :9 ("no text a caller passes is ever read by VEXos as a format"):
      a property of the literal `"%s"` call, true whichever way the belief falls.
  - Item 2 is now true of A1's headers, its tests, its shim pieces and its public pages.
    `build-order.md` :632's R3d clause stays the one unhedged line outside A1's, as the resumption's
    coordinator item 3 says.

**The documents A1 touched, complete** (for pace rule 3's commit body; `git status` at 22:55):
- **New:** this log. `docs/api/brain_screen.md`, `calibrate_screen.md`, `front_check.md` and
  `pros-brain_screen.md` (generated).
- **Changed, public:**
  - `docs/changelog.md`: "Added since 2.4 (2026-10-15)", at the top of `## API 2.4`;
  - `docs/api/README.md`, `docs/api/all-entities.md`, and four `mkdocs.yml` nav lines (generated);
  - `README.md` :87-103 (eighteen adapters with the touch screen; the two unregistered PROS
    beliefs — R14, R16) and :275-277 (the calibration line).
- **Changed, internal:**
  - `build-order.md`: CAL1's status block, and one clause in :632's `Next:` line;
  - `REMAINING-WORK.md`: row 18b;
  - `HANDOFF-remaining-work.md` :263: 20 headers, `ProsFileStore` and `ProsBrainScreen` (F1);
  - `ORIENTATION.md` :94: "twenty files" (the resumption);
  - `PROJECT-BRIEFING.md`: the generated block, and :193-197 and :746-748 (the adapter counts, R14).
- **Deliberately NOT touched:**
  - `docs/roadmap.md`: its frontier sentence is still true;
  - `docs/hardware-assumptions.md`: HA-187 is A2's, and no HA number was taken;
  - the guide, the FAQ, `RESUMING.md` and the brief;
  - other chunks' files: `test/CMakeLists.txt`, `ci.yml`, `tools/`, `shim_control.hpp`, `src/` and
    the `Makefile`.

**The mutations of this round** (each verdict on a fresh binary; "before" is the tree as the
verifier found it, 22:48–22:49; "final" is the final tree, 22:59–23:00, after the last `///`
change; the "after" pass at 22:53–22:54 gave the same verdicts):

| ID | The mutant | Before the fixes | Final tree | Caught by |
|---|---|---|---|---|
| C0 | CONTROL: a comment added | GREEN | GREEN | — |
| N1 | `paintCannot`: headline `>=` | GREEN | RED | P7 (exact pair) |
| N1p | … headline `> budget + 1` | GREEN | RED | P7 |
| N2 | `paintStateScreen`: state name `>=` | GREEN | RED | P7 |
| N2p | … state name `> budget + 1` | GREEN | RED | P7 |
| N3 | `paintCannot`: header word `>=` | GREEN | RED | P7 |
| N3p | … header word `> budget + 1` | GREEN | RED | P7 |
| N4 | `paintCannot`: footer `>=` | GREEN | RED | P7 |
| N4p | … footer `> budget + 1` | GREEN | RED | P7 |
| N10 | `paintStateScreen`: status bits `>=` (the fifth sibling) | GREEN | RED | P7 |
| N10p | … status bits `> budget + 1` | GREEN | RED | P7 |
| N11p | CONTROL of the method: the not-driver-control bits `> budget + 1` | RED | RED | P7 (V31's pair) |
| N5 | `fits()`: the wrapped branch returns true | GREEN | COMPILE-FAIL | P9's `static_assert`s |
| N5b | … the wrapped branch drops its ASCII/`%` check | GREEN | COMPILE-FAIL | P9 |
| N6 | `wrapsCompletely` ignores the wrap | GREEN | COMPILE-FAIL | P9 |
| N6b | … accepts any width | GREEN | COMPILE-FAIL | P9 |
| N6c | … accepts any line count | GREEN | COMPILE-FAIL | P9 |
| N7 | `fits()`: one line measured against its own length | GREEN | COMPILE-FAIL | P9 |
| N12 | `allFit` walks without looking | GREEN | COMPILE-FAIL | P9 (the lists) |
| N12b | `allFit` checks only the FIRST entry | — (new form) | COMPILE-FAIL | P9 (bad last) |
| N12c | `allFit` checks only the LAST entry | — (new form) | COMPILE-FAIL | P9 (bad first) |
| N13 | the header's `static_assert` removed | GREEN | RED | P9 (the source pin) |
| N13b | … commented out | — | RED | P9 (the pin, at a line start) |
| N14 | TEST-SIDE: P7's exact headline back to the old 17 characters | — | RED | P7's length `REQUIRE` |
| V17–V22, V30, V31 | the review rounds' P7 mutants, re-run because P7 changed | — | RED, each | P7 |
| V23 | `columnsOf` counts BYTES | — | RED | P7 (every exact text) |
| V25, V26 | an over-budget literal in `kAll` | — | COMPILE-FAIL | the header's `static_assert(allFit(kAll))`, :358 |

- Every COMPILE-FAIL was read: each failed at the intended assertion, either P9's block
  (`brain_screen_test.cpp` :1155-:1179) or the header's :358. None failed on anything else.
- After each pass, `mtree/` matched its pre-pass checksums (`md5sum -c`). `RESTORE MISMATCH` occurs
  0 times.
- The worktree's `include/ test/ src/` equals the final pass's tree file for file
  (`mtree_md5_final.txt`).
- The pre-fix pass's first try had three malformed mutants, all reported as such. N4's replacement
  dropped a `;`, and N6b/N6c tripped `-Werror=unused-parameter`. They were corrected and re-run, and
  their GREEN verdicts above are from the corrected runs.

**The merge, re-proven against TODAY'S `shulib-v2` (`fd55ffc`).** Three commits have landed since
the second resumption's proof at `2506067`:
- MECH-A2 (`98392b2`);
- the HA-157 register line (`fbaf729`);
- SER1's completion record (`fd55ffc`).

This round also changed a public header: `allFit`'s signature, a template now. SER1 changed
`tools/api_doc_tool.py` on main, so the page had to be checked against main's generator.

**How the trial merge was built** (`k_vr2/merge/`, scripts `merge.sh` and `resolve.py`):
- `git archive shulib-v2` was extracted into a scratch tree.
- `git merge-file` was run, with `aaea302` as base, on every file both branches changed.
- A1's new files, and the two it alone changed, were copied in.

**The merge results:**
- **CLEAN:** `README.md`, `REMAINING-WORK.md`, `mkdocs.yml`.
- **CONFLICT, NEW since the last proof — `HANDOFF-remaining-work.md` :263-267.** MECH-A2 rewrote the
  sentence before the adapter count ("F12 unfrozen; since MECH-A2 its deadline and an abandon also
  cut a claimed mechanism"). Resolved as main's lines with A1's count, "20 headers", and A1's two
  list entries. Main still has 19 `hal/pros/` headers (`git ls-tree`; MECH-A2 added none), so the
  merged tree has 20.
- **CONFLICT, `docs/changelog.md`:** resolved as A1's section first, then main's: MECH-A2, SER1,
  MECH-A1, newest landing first. The resolved file differs from main's only by A1's 85 added lines.
- **CONFLICT, `build-order.md` :632:** resolved as main's line with A1's one clause after "typed
  drive ports" (the anchor occurs once). The result differs from main's file only in that line and
  in A1's CAL1 status block.
- **CONFLICT, `PROJECT-BRIEFING.md`:** only inside the generated block. A1's two hand-written count
  passages (:193-197, :746-748) merged cleanly.
- **CONFLICT, `docs/api/README.md` and `all-entities.md`:** generator output.

**The merged tree, gated** (a throwaway git repository inside the scratch tree gave the build a hash,
`b4acb21`, as in the last proof; `GIT_CEILING_DIRECTORIES` kept git off the main repository):
- **`docs/api/`:** main's `api_doc_tool.py generate` writes A1's four pages BYTE-IDENTICAL to the
  worktree's, the templated `allFit` included. Gates: `self-test` OK; `check-coverage` 4,744
  entities across 183 headers; `check-fresh` exit 0; `check-examples` 541 verbatim;
  `check-removability` clean.
- **Host build:** 0 warnings.
- **Suite:** **2477 passed | 0 failed | 3 skipped**, 2,794,052 assertions. That is main's 2,447 plus
  A1's 30, and all 30 of A1's cases have the same assertion counts as in the worktree.
- **The briefing**, regenerated after the suite: `check` current; `self-test` OK.
- **Staleness:** self-test OK; clean.
- **ARM header gate:** all 223 headers, exit 0; the control fails as it must.
- **CI greps:** both pass.

**A slip, logged:** one command in this round wrote a scratch list to `/tmp/x` (a sorted `git diff
--name-only`), against the rule that scratch lives only under `scratch/cal1-a1/`. It was deleted at
once; it held only file names. Every other file this round wrote is under `k_vr2/` or the worktree.

**Every gate on the worktree's final tree** (logs under `k_vr2/`). Every test binary and every tool
that runs one ran capped as `(ulimit -v 4194304; timeout …)`: the suite, the mutants,
`briefing_status.py`, `cmake --build` (its doc-gate step runs the binary), the src gate. Nothing was
killed by the cap or the timeout.
- **The build, from scratch** (22:55–22:56):
  - `build/test` was removed of its binary, then moved to `scratch/cal1-a1/old_build_test_vr2`;
  - the briefing was generated with no binary, so it read "not built";
  - `cmake -S test -B build/test` and `cmake --build`: exit 0, 0 warnings;
  - the doc gates in the build passed: `api_doc_tool self-test` OK, coverage 3,773 / 174, 521 lines
    verbatim, removability clean, briefing and staleness self-tests OK, staleness clean.
- **The suite:** `2283 passed | 0 failed | 3 skipped`, `2785337` assertions; 18.5 s; peak RSS
  58,948 KB (`k_suite.log`, `k_suite.time`).
  - Case by case (`cmpxml.py`) against the second resumption's final run (`i_suite2.xml`): 0 new,
    0 gone. Only P7 changed (49 → 83 assertions) and P9 (160 → 161, the source pin; its 15 new
    checks are compile-time).
  - Against `aaea302` (`r_base_suite.xml`): 30 new, none gone, 0 failures. The same eight source
    scans' counts rose, as A1.4 lists.
- **The briefing** was regenerated after the suite (`2,283 cases / 2,785,337 assertions, 3 skipped —
  green`, `Public headers: 212`). The second `cmake --build` reported `generated block current`.
- **`docs/api/`:** regenerated after each `///` change to `allFit`. `calibrate_screen.md` :721-724
  carries the template signature and the new comment.
- **CI greps, ARM gate, doc tools** (`k_vr2/gates/`, the same script as the second resumption,
  23:00–23:01):
  - CI greps: `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core
    is sim-free`;
  - ARM header gate: all 212 headers, exit 0, no output. Control (`int x = d;`): exit 1, `error:
    conversion from 'double' to 'int' may change value [-Werror=float-conversion]`;
  - `api_doc_tool.py`: `self-test` OK; `check-coverage` 3,773 / 174; `check-fresh` exit 0;
    `check-examples` 521 verbatim; `check-removability` clean;
  - `briefing_status.py`: `self-test` OK; `check` current. `doc_staleness_audit.py`: self-test OK;
    clean (29 live docs).
- **`src_build_gate.py self-test`:** `OK (211 detector cases, each against a real build)`,
  23:01:38–23:13:01. **`check`:** `PASS (all 7 builds link …)`, 23:13:01–23:15:21. Every build was
  `exit 0, 11/11 TUs compiled, warnings: 8 vendor-ignored, 0 disallowed`.
  - Nothing was built in the worktree while the self-test was planting. The trial merge ran in its
    own scratch tree.
  - `git status --short` was identical before and after (`k_status_before_srcgate.txt`). `git diff
    -- src/ Makefile test/CMakeLists.txt include/shulib/config tools .github
    test/pros_shim/shim_control.hpp`: EMPTY.
- **After the src gate** (its restores bump timestamps), one more `cmake --build` (23:15), which
  recompiled all 212 TUs with 0 warnings. Its doc gates passed (`generated block current`).
  - The suite again: `2283 | 0 failed`, `2785337` assertions, case by case IDENTICAL to the run
    above (`k_suite3.xml`).
  - `briefing_status.py check`: current. The briefing's block therefore describes the binary on disk.
- No existing test TU was edited: `brain_screen_test.cpp` and `pros_brain_screen_adapter_test.cpp`
  are A1's own. `shim_control.hpp`, `ci.yml` and `tools/` are untouched.

**The prose review of this round (pace rule 4)** — grepped first, every hit read:
- **The old `allFit()` signature:** over every `*.md`, `*.hpp`, `*.cpp` and `*.py` outside `build/`
  and `docs/api/`, the hits are the new header and P9; this log's V25/V26 rows and its pass-4 note,
  which are history of that time; and `src/programs/calibrate_program.cpp`'s own `allFit()`, robot
  two's program, unrelated.
- **The exact-budget claims** (`17-character`, `19-byte`, `exactly the budget`, `EXACTLY its budget`,
  `at exactly`) over this log, the changelog, the README, the header and the test:
  - the R2 entry and the V23 row now carry markers;
  - V31's row (`>=` at exactly the budget) is true;
  - the test's new comments match its checks: each REQUIRE ties a text to its budget constant.
- **PROS beliefs** (`leak`, `choke`, `conversion with`, `format string`) over A1's 13 files, the
  README and the changelog, after the hedges: every remaining hit is a continuation line of a
  sentence that says BELIEVED or "read from the SDK and not yet entered". The two left unchanged on
  purpose are explained under V3.
- **Touched-file claims** (`untouched`, `NOT touched`, `no line false`, `hedged everywhere`) in this
  log: the four false ones carry markers. The rest are true: `hardware-assumptions.md`,
  `shim_control.hpp`, `tools/` and `ci.yml` are all unchanged (`git status`).
- **The new `///` on `allFit`** was read against P9: it claims only what P9's assertions hold. A
  first wording, "the check cannot be weakened without a red", over-claimed: the `#if 0` residual
  above shows it. It was narrowed to "a check weakened to pass everything fails to build that test"
  before any gate counted.

**For the coordinator — added by this round** (the lists above still stand, except the second
resumption's item 1, superseded here):
1. **The merge onto `shulib-v2` at `fd55ffc`** (proven in `k_vr2/merge/tree`; `resolve.py` does each
   step):
   - (a) `docs/changelog.md`: A1's section, then MECH-A2's, SER1's and MECH-A1's, at the top of
     `## API 2.4`.
   - (b) `build-order.md` :632: main's line plus A1's one clause after "typed drive ports".
   - (c) **NEW:** `HANDOFF-remaining-work.md` :263-267: main's sentence (MECH-A2's run-guard
     clause) with A1's "20 headers" and its two list entries.
   - (d) `docs/api/README.md`, `all-entities.md`, `mkdocs.yml`: run `api_doc_tool.py generate`;
     never hand-merge them.
   - (e) `PROJECT-BRIEFING.md`: run `briefing_status.py generate` LAST, after the merged suite binary
     exists.
   - README and REMAINING-WORK merge cleanly. Expect 2,477 cases and 223 public headers.
2. **For A2:** `screen_text::allFit` now takes its list. A2's new literals join `kAll` and are checked
   by the header's `static_assert(allFit(kAll), …)`, so A2 calls nothing. A literal's comment must
   still begin with its size: P9 reads it.
3. **The one guard left at test time, by choice:** an `#if 0` around the header's assertion passes
   P9's text pin. A compile-refusal case (MECH-A1's harness, `test/compile_refusals/`) would close it.
   That harness is another chunk's and `test/CMakeLists.txt` is shared, so whether to add one is the
   coordinator's call.
4. **Disk:** this round added `k_vr2/` (394 MB: the mutation copy and the trial merge with its build
   and its throwaway `.git`) and `old_build_test_vr2/` (144 MB). The scratch directory is now 4.8 GB.
   All of it is evidence only.

**Close of the second verification round (23:18 EDT, `date`).** All three findings were real and
fixed at the root, and each reached further than reported:
- **V1:** a fifth sibling (the state screen's status bits) and the other side of every boundary,
  all now pinned by exact-budget pairs.
- **V2:** the guard's whole logic is now tested at compile time, `allFit`'s loop included, and its
  presence in the header is pinned.
- **V3:** the log is corrected and the seven unhedged test comments are hedged, so the claim they
  falsified is now true.

18 mutations that were GREEN before the fixes are now RED or COMPILE-FAIL. Four more (N12b, N12c,
N13b, N14) exist only against the new code or test, and all four are caught. The review rounds' P7
mutants were re-run, all RED, and the controls behaved as they must. Every gate passed on the worktree, and the merge onto today's
`shulib-v2` was re-proven green. **Nothing committed, nothing pushed, no `pros`, no serial port. No
new hardware belief and no HA number taken. No test binary ran uncapped.**
After this entry was written, `check-fresh`, `check-removability`, `briefing_status.py check` and the
staleness audit were run again (`k_vr2/k_final_*.log`): all exit 0.

## The coordinator's merge verification — CAL1 Part A1 (2026-10-15 23:10–23:58, main `fd55ffc` + the squash of `cal1-a1`)
- **Audit before the merge:** no gate plant (`git diff --stat -- src/ tools/ Makefile` empty in the worktree); the 38
  mutation definitions under `scratch/cal1-a1/mut/` checked against their files — none still applied (the second
  verification round mutated scratch copies only); no stray root file; the verifier's one `/tmp/x` slip already
  deleted. The transport commit `28011c7`, then `git merge --squash cal1-a1` (base `aaea302`, A1's real base).
- **Six conflicts:** `docs/api/README.md`, `all-entities.md` and the briefing (regenerated); the changelog (A1's entry
  first, dated as merged, then MECH-A2's and SER1's); HANDOFF :263 and build-order's `Next:` — each a word-level
  three-way merge against `aaea302`, so main's sentence (MECH-A2's cut; A2 LANDED) and A1's clause ("20 headers";
  "CAL1 Part A1 LANDED") both stand. The roadmap gained the part's "Now" paragraph (A1 had none; MECH's parts each
  have one).
- **Gates on the merged tree** (`.claude/coordinator/logs/cal1_a1_merge_verify.out`): host build with the doc gates;
  suite **2,477 | 2,477 passed, 3 skipped** (main's 2,447 + A1's 30); `src_build_gate.py` self-test OK and check
  **PASS, all 7 builds**; ARM header gate over **223** headers clean; CI greps empty; api check-fresh, the briefing,
  removability and the staleness audit clean; the compile-refusal harness **200 of 200**.
- **The coordinator's hand mutations** (`muts_cal1_a1_main.json`, on `hal/brain_screen.hpp`, `calibration/front_check.hpp`,
  `hal/pros/brain_screen.hpp`, `calibration/calibrate_screen.hpp`; each on a fresh link, the whole suite under a 4 GiB /
  300 s cap, restored and hash-checked): **12 of 12, one after a new
  test** — CM1 (DEL passing the sanitizer), CM2 (an em dash as '?'), CM3 (one '?' per continuation byte), CM7 (a port
  only in NOW counted as only in EARLIER), CM9 (an agree TIE read SAME), CM10 (the pen set before the eraser), CM11
  (an `E_TOUCH_ERROR` read not an error), CM12 (LARGE drawn MEDIUM) RED; CM5 (the cut ignoring the buffer's capacity)
  killed by an abort (caught); **CM6** (the sign ignored) and **CM8** (a full side of `kMaxSmartPorts` unreadable) refused AT
  BUILD by `test/front_check_test.cpp`'s own compile-time checks (:436 "record vs other end is FLIPPED"; :433-434 the full
  side) — confirmed by compiling that file alone against a mutated scratch copy, so the failure is the test's, not the
  mutation's. **CM13 SURVIVED** (`now - lineAt_ < kLiveRepaintSeconds` → `<=`): P4 stepped on exact binary times (0.125 s,
  0.25 s) but never ON the 0.2 s edge. Added to P4 (`test/brain_screen_test.cpp`): a paint at 0.0 and a change at exactly
  `kLiveRepaintSeconds` must repaint ("at most every" — never sooner, never later). Re-run: **CM13 RED** at the new line;
  restored tree 2,477 | 2,477. The same lesson as MECH-A2's MB5: an equality edge needs a test that lands on it.
- **Accepted, not closed:** the second verification round's one open gap — an `#if 0` around the header's
  `static_assert(allFit(kAll), …)` would still pass P9's text check. P9 proves `allFit` itself both ways at compile
  time; disabling the header's assertion is a deliberate edit, not a slip; a compile-refusal case for it would edit
  the shared harness for a sabotage case. Recorded here so it is not silent.

---

# Part A2a — the standard Calibrate with NO typed reference (a library ADDITION)

> **This run builds Part A2a ONLY**, as the team lead's split (2026-10-08, "Split it") defines it
> (the brief's check of D6 (a), "Not settled" #1 (ii); §9.2): the move of today's program onto A1's
> seams in the NEW `include/shulib/calibration/calibrate_program.hpp` (PROS-free, D1 (a)), the PROS
> shell `include/shulib/hal/pros/calibrate.hpp`, the panel's additions and the two held screens, the
> watch, the facts checks, the host rig, tests 1, 4–6, 8b and 9 and their mutations, and C2's and C3's
> untyped words. NOT A2b's: no `TypedDrivePorts` / `CalibrateFacts::typedDrive` / typed words, no C4,
> no golden test's 34 scenarios, no team-file example test, no chapter, no guide 12, no HA-187 (A2b
> registers it). NOT B's: no `src/` file (`git diff --stat ceb17c0 -- src/` must be EMPTY at the end).
>
> Executor: **Claude Opus 5.5 (1M context)** (subagent of the coordinator's workflow run), working in
> the git worktree `.claude/worktrees/cal1-a2a` (branch `cal1-a2a`, from `ceb17c0`). The executor does
> not commit, push, run `pros`, or touch the serial port. Scratch and backups:
> `.claude/coordinator/scratch/cal1-a2a/` (never `/tmp`, never the repository root). Every test binary
> runs capped: `(ulimit -v 4194304; timeout 600 …)`.

## A2a.0 Start (2026-10-18 22:33 EDT, `date`)

- Worktree clean at `ceb17c0` (`git status`). Read RESUMING.md's standing rules, the brief in full
  (3,171 lines: the A2 SPLIT note, the answers block, every Re-check entry R1–R18, the D6 (a) entry and
  its continuation, "The check of D6 (a)" C1–C5 and its continuation, the Decisions, §0–§9, the
  landmines) and this log in full (Part A1, its review rounds, every hand-on to A2, the coordinator's
  merge record).
- This heading was appended FIRST, before any code.
- **Baseline** (22:40–22:50): `briefing_status.py generate` with no binary, then `cmake -S test -B
  build/test` and `cmake --build build/test -j8` (the build capped as `(ulimit -v 4194304; timeout
  1800 …)` — its doc-gate step runs the binary): exit 0, 0 warnings. The suite, capped: **2,477
  passed | 0 failed | 3 skipped, 2,794,057 assertions**, peak RSS 61,264 KB (`scratch/cal1-a2a/logs/
  b0_suite.xml`, `b0_suite.time`) — main's count after the coordinator's A1 merge. This XML is the
  case-by-case baseline for test 9.
- **What the split leaves A2a, read against the brief's §2.A** (each a ruling of this run, logged):
  - The split names "tests 1, 4–6, 8b and 9". **Test 2 (the front holds) is in neither list**, but
    its UNTYPED half tests code that is A2a's by the same sentence — the held screens, the FRONT
    FLIPPED hold, USE NEW / RUN AGAIN / the disable, and C3's untyped words (backup-only, EMPTY,
    both undecodable, no card) — so A2a builds test 2's untyped cases and their mutations (4–8);
    its typed cases are A2b's. Without them, the hold and C3's words would land untested.
  - Mutations, by the same reading: 4–15, 18, 20–27 and 43, 45–49 (the untyped ones among the
    check's) are A2a's; 28–42, 44 and 50–54 are typed (A2b's). 19 is re-run for A2a's new headers.

## A2a.R Resumption (2026-10-22 00:45 EDT, `date`)

- **Why a resumption.** The run that wrote A2a.0 was STOPPED by the team lead's pause of all work
  (~23:30 on 2026-10-18, the session limit) and resumed 2026-10-22 ~00:45. Executor of this
  resumption: **Claude Opus 5.5 (1M context)**, the same worktree and branch (`cal1-a2a`, from
  `ceb17c0`). As with A1's resumption, the stopped run appended NOTHING after A2a.0 (22:33); its work
  is reconstructed below from the files it left (times 22:42–23:03) and its scratch
  (`scratch/cal1-a2a/logs/`, `cmp/`, `smoke/`). Every result it implies is RE-RUN here on a fresh
  build, never taken from those files.
- **State found** (`git status`, `git diff -- src/ Makefile tools/ test/CMakeLists.txt test/pros_shim
  .github`): those paths UNCHANGED (0 diff lines) — no gate self-test plant left by a killed run.
  Modified: `include/shulib/calibration/calibrate_panel.hpp` (the additions), `docs/api/README.md`,
  `all-entities.md`, `calibrate_panel.md`, `mkdocs.yml` (two nav lines), `PROJECT-BRIEFING.md`
  (generated), this log. New: `include/shulib/calibration/calibrate_program.hpp` (1,482 lines),
  `include/shulib/hal/pros/calibrate.hpp` (156), `test/calibrate_program_rig.hpp` (888),
  `test/calibrate_program_test.cpp` (835: T1a–T1j, T2a–T2i, T4, T4b), `docs/api/calibrate_program.md`,
  `docs/api/pros-calibrate.md` — 13 files, the coordinator's count. No mutation backup exists under
  `scratch/cal1-a2a/` (the run had not reached its mutations), and `build/test/shulib_tests` does NOT
  exist (no binary, so no mutant can be linked); the build tree is rebuilt from source before any
  result here.
- **What the stopped run's scratch shows** (read, not trusted): `logs/b1_build.log` (22:58) — a build
  that linked after the panel and program existed; `smoke/` — serial and screen logs of 17 rig
  scenarios (22:55–22:58), from a scratch driver whose source it did not keep; `cmp/` — `m_*` (those
  serial logs) beside `p_*` (the brief's probe goldens, `brief-CAL1/d6a/v3/run_new/`) for eleven
  scenarios, i.e. it had begun comparing the library program with the probe's. No conclusion of it
  was written down; this resumption re-does the comparison from source (A2a.3).
- **Not yet written when it stopped:** tests T5 (the watch and its hold), T5b (undeclared mechanism
  motors), T6 (the facts), 8b (`test/pros_calibrate_adapter_test.cpp`, which `hal/pros/calibrate.hpp`
  and the test file's header already name), 9 (`test/calibrate_panel_additions_test.cpp`, named
  likewise); every mutation; the changelog, build-order, REMAINING-WORK, roadmap; the prose review; the
  gates. Finished work is not redone; each file is checked against the brief as it is read.

## A2a.1 What the stopped run built (2026-10-18 22:42–23:03) — reconstructed from its files

**The drafts it worked from.** The newest draft wins (the brief's header): `brief-CAL1/d6a/v3/p3r/`
for `calibrate_program.hpp`, `calibrate_panel.hpp` and `hal/pros/calibrate.hpp`. `diff` of each
delivered file against it (`scratch/cal1-a2a/logs/draft_vs_wt_program.diff`, comments aside) gives the
list below; the stopped run logged none of it.

**The split applied (A2b's, removed from the draft):** `TypedDrivePorts`, `CalibrateFacts::typedDrive`,
`FactsList`, `FactsProblem::list`, `hasTypedDrive`, `typedDriveDescription`, `typedDrivePortsFrom`,
the typed branch of the front check and every typed word (the card line's typed sentence, the typed
FLIPPED lines, `FIX YOUR TYPED DRIVE PORTS`, the per-port difference lines, `emitTypedDifferences`),
`Held::typedRef`, `FinishFacts::typedPorts` and the panel's six typed literals. With none of them the
reference is the card's record only, and every "the card" word is the only word.

**Its deltas from the draft, each with its reason:**
1. `fillBuildTime()` writes the header's build time into the PROGRAM's own `buildTime_` — the draft's
   `buildTime()` returned a function-`static` buffer shared by every program in the image (a second
   `CalibrateProgram`, or `calibrateStateScreen`'s painter, would overwrite the first's header).
2. A stated cartridge byte outside RED/GREEN/BLUE is refused by name (`CARTRIDGE UNSET`, "the stated
   cartridge (byte N) is not RED, GREEN or BLUE") before any device — the draft let it reach the
   calibrator's precondition, which throws out of `run()` past every screen.
3. The watch and the still-candidate lines judge only an ACCEPTED push (`if (cal->phase() ==
   Phase::SideAssign)`): the draft printed them after a push the calibrator refused at DONE — a WARNING
   that "the save waits for you" on a run that can never save.
4. `TouchGate::view()` restarts its arming clock when the HOLD's question changes, not only when
   `layoutKey()` does: FRONT FLIPPED's USE NEW and MECHANISM PORT TURNED's SAVE share one box, one id
   and one key, so under the draft a reflexive second tap on USE NEW landed on a LIVE SAVE.
   `recordLayout()` prints a `screen:` line on the same change (the draft printed none for the second
   question).
5. The finished screen's NOTE: the draft dropped a two-row calibrator warning's second row silently;
   the brief (§1.4) says "cut to one with ` ...`" — done, through `wrapWords<kSmallCols - 4, 1>` and a
   new literal `kWarningCut` (in `kAll`).
6. `FRONT NOT CHECKED -- see the log` is a panel literal (`kNoteNotChecked`, in `kAll`'s compile-time
   budget check) — the draft wrote it inline in the program, unchecked.
7. The `PORT N KEPT AS A MECHANISM -- see the log` NOTE's widest form (port 21) is checked against the
   SMALL budget at compile time (`detail::kProgramWidestKeptNote`).
8. The words: the `record file:` line and the boot banner say "a verified backup X first" (the brief's
   §1.5, B1 item 1: "name BOTH files and say the order"; the draft said "backup X"); the watch's banner
   line says "and the save waits" (D4 as amended: the watch HOLDS).
9. The census's drop, the motor construction and a run's start are named private functions
   (`dropDeclared`, `buildMotors` — §1.6's one construction R5a Part C reuses — `startRun`), and §1.6's
   `RunKind` is the switch where a run starts and where its outcome is printed.
10. `snprintf` widths bounded where a caller's text enters a fixed buffer (`%.60s` a mechanism's name,
    `%.150s` the boot label, `%.100s` the stamp, `%.40s`/`%.130s` the state screen's words).

**The shell** (`hal/pros/calibrate.hpp`) is the draft's with SER1 applied: `line()` is ONE
`serialWrapped("R3D", text)` call (the brief's `printf` + `fflush` is gone — SER1's hand-over), the
competition bits are `static_assert`ed against PROS's, and the watch's read is the C API's by port.

## A2a.2 This resumption's findings, fixed at the root (2026-10-22)

1. **The facts were checked AFTER the declared-port drop read them.** With a port listed twice, the
   drop printed the second entry as `port 6: b (declared) -- the census found NO MOTOR there` — false,
   port 6 HAS a motor — and only then refused. **Fixed:** `factsProblem()` runs right after the census
   count line, before `dropDeclared()`; a refused list prints no `(declared)` line at all (T6a). The
   census still prints (it is diagnosis). The probe's `facts-port0` loses its two (true) drop lines;
   no other golden changes. **Alternative not taken:** checking before the census — it would hide the
   census from a run whose only fault is a typed list.
2. **The watch's "turned" was not the calibrator's test.** The draft (and the stopped run) used
   `|moved| > threshold`; the calibrator's `movedOnPush()` — the rule that makes a candidate a drive
   port, which the brief (D4, R12) says the watch shares — is a FINITE travel whose size REACHES the
   threshold (`>=`). **Fixed:** `hal::isUsableReading(moved) && std::abs(moved) >= threshold`; a
   reading that is not a number is "could not be read -- not judged", never "did not turn" (T5g lands
   on the edge exactly, in binary — the coordinator's MB5/CM13 lesson; T5h the NaN).
3. **`calibrateStateScreen()`'s line could be cut, and the host compiler said so** (`-Wformat-truncation`,
   first seen when test 8b instantiated it): `%s` of the status words into a 240-byte line beside two
   bounded texts. **Fixed:** `%.40s` (the words are at most 37). The src gate's `-Wformat-truncation`
   would have refused it at Part B; the ARM header gate cannot see it (it emits no unused inline
   function).

## A2a.3 The library program on the A2a rig against the brief's probe goldens (2026-10-22)

`scratch/cal1-a2a/probecmp/compare.cpp` (scratch, not project code) runs the library program on
`test/calibrate_program_rig.hpp` for the 23 UNTYPED scenarios of the probe harness
(`brief-CAL1/d6a/v3/harness4.cpp`, its facts overrides and `run.sh`'s seeds) and `cmp.py` compares each
serial and screen log with `d6a/v3/run_new/` (before A2a.2's fixes; `out1/`, `cmp2.log`). Normalized:
only the environment (the probe's boot banner, which its harness printed first; the build hash and
stamp; the probe's long card path). **Draw counts equal in 22 of 23** (`disable` 111 → 110: the disable
lands on another 10 ms tick). **Serial lines: every difference is one of** — T (timing: the same line,
other digits — a pulse's yaw at 40 or 41 plant steps, tick and print counts; 97 lines in all); the
record-file line and the watch banner line (A2a.1 #8: two lines per scenario that reaches them); and
`curved`'s two watch lines for its REFUSED first push (A2a.1 #3). Screen: only the NOTE's " ..." cut
(A2a.1 #5, `slip18`, `undeclared`) and timing. **Residual 0** — so the rig reproduces the probe's
plant, operator and screen, and the library program the probe's draft, except the deltas above.

**Re-run after A2a.2's fixes** (`out2/`, `cmp3.log`): the same classes, plus `facts-port0`'s two drop
lines (A2a.2 #1: a refused list now prints no drop line). Every other scenario's serial and screen logs
are byte-identical to `out1/` (`cmp`, all 22) — the `>=` watch and the bounded state line change no
output on these scenarios.

## A2a.4 The tests (the brief's §2.A, A2a's share; 2026-10-22)

The stopped run left T1a–T1j, T2a–T2i, T4 and T4b (all green on the fresh build, re-read against the
brief). This resumption added the rest, and three cases the mutations below showed were owed:
- **`test/calibrate_program_test.cpp`** (40 cases, on `test/calibrate_program_rig.hpp`):
  - **T1** (test 1) the scenarios on robot two's facts written literally (no typed ports — Part B's
    derivation): T1a one run (every decision line verbatim, the record's bytes, encode∘decode, the
    MEASURED line = the card, the screens 1/4..4/4 and the FIRST RECORD note, the claws untouched and
    read twice), T1b RUN AGAIN (SAME against run 1), T1c port 18 LEFT OUT (the mechanism words; the
    NOTE's " ..." cut), T1d a curved push then a straight one (no watch line for the refused push, no
    write before run 2's DONE), T1e no card, T1f no IMU, T1g a disable mid-pulses (exits on the tick it
    is seen), T1h unequal side scales (R14), T1i a PROGRAM FAULT, T1j touch errors at start, **T1k two
    programs alive at once, each its own build time (new: kills X10)**.
  - **T2** (test 2's UNTYPED half — the stopped run's ruling, A2a.0): T2a FRONT FLIPPED held, nothing
    written, RUN AGAIN, SAME on run 3; T2b USE NEW saves as pushed (0 of 10 forward, the warned-of
    case); T2c a disable while held; T2d the cable swap (10 of 10 with USE NEW); **T2j a discarded hold
    leaves nothing held behind (new: kills X13)**; T2e–T2g files that do not decode (both, the backup
    alone, an EMPTY file); T2h CANNOT TELL; T2i a damaged record file read through its backup.
  - **T4** (test 4) the save under seven planted card faults: the program's card operations equal a
    replay of two `loadRecord()` and one `saveRecordSafely()` op for op, its words `formatSaveReport(No)`'s
    in order, SAVED iff Saved; **T4b** a source pin (one writer, no stdio printer, no PROS).
  - **T5** (test 5) T5a a claw the push turns (WARNING, held, nothing written until the SAVE tap —
    the rig's tap marks — the KEPT note); T5b the typo over a card that drives with it (byte-identical
    card after RUN AGAIN and a disable); T5c the same on an EMPTY card (C2's words); T5d with no card
    (C3's words); T5e SAVE on the typo (the informed choice); T5f FRONT FLIPPED then MECHANISM PORT
    TURNED, the box re-greyed so the reflexive tap lands on nothing, no write before the second answer;
    T5g the exact threshold, a hair under it, 2°; T5h a refused read and a NaN read; T5i nothing
    declared (the three banner lines); **T5j in EVERY scenario (15) the declared ports never asked for,
    commanded or braked, each read once per push begun and once more per push accepted (new)**.
    **T5b-1/2** (test 5b) undeclared 8 (geared, never powered) and 9 (pulsed at exactly 3, 6 and 9 V,
    LEFT OUT); 9 loaded → PULSE_STILL with the "declare it" line.
  - **T6** (test 6) T6a the list's refusals (named port 0, 22, 255, a port twice) before ANY device and
    before any drop line; T6b kind unset, x-drive, h-drive, cartridge unset, a byte that is no
    cartridge; T6c TOO FEW MOTORS naming the declared ports (and the plain fix with none dropped);
    T6d the drop keeps the census order (the deleted `dropDeclaredMechanismPorts`' cases: an empty list,
    both pivots, a declared port with no motor — the run goes on, its watch "not judged"); T6e
    `factsProblem()` at COMPILE time, both ways (15 `static_assert`s).
- **`test/pros_calibrate_adapter_test.cpp`** (test 8b, 7 cases, through the host PROS shim): PC1 the
  watch's C read (degrees, rotations, counts by red/green/blue; false for PROS_ERR_F, an unknown unit,
  counts with no gearing; NOT ONE write on any port); PC2 `line()` one tagged write per line, a 400-char
  line in two tagged pieces, a null text; PC3 the status byte and `pros::delay`; PC4 the boot banner
  verbatim; PC5 the three state screens (painted whole, one line each); PC6 `runCalibrate()`'s own
  composition over the real adapters (a refusal painted and printed, every write tagged, no motor
  written); PC7 the source pin (no `pros::Motor` / `v5::Motor` / `ProsMotor`, one `serialWrapped`, no
  stdio). fd 1 is captured with a copy of SER1's socket helper (that file is SER1's; not edited).
- **`test/calibrate_panel_additions_test.cpp`** (test 9 and the panel's half of the holds, 6 cases):
  N1 the defaulted `viewFor()`/`TouchGate::view()` equal `AskedAfterPush` + `FinishFacts{}` field for
  field over 119,957 views of five runs; N2 ByPush never paints WHICH END; N3 the held views' words,
  places and boxes, one `layoutKey()` for both questions; N4 the left box is `SaveHeld` only on a held
  screen in Done; N5 the arming clock restarts on a change of question (and only then); N6 the NOTE and
  the " ..." cut, `SAVED -- NEW FRONT`.
- **The rig** gained (this resumption): an exact-angle claw, a NaN read, a claw with no motor reading
  false, tap marks (card operations at each tap), and `program()` (a program constructed, not run).
- No existing test TU was edited (`git status test/`: four new files only); `test/CMakeLists.txt`
  untouched (its sources are globbed).

## A2a.5 Mutations — every one on a FRESH binary, capped, SEEN red, restored (2026-10-22 01:09–01:29)

**Method** (`scratch/cal1-a2a/mut/run_mut.py`, list `a2a_mutations.json`, output `pass1.out` and
`mutations.log`): back up each touched file under `scratch/cal1-a2a/mut/` (never /tmp); apply each edit
(its anchor must occur exactly once — every anchor was checked against the live files when the list was
written); build `shulib_tests/fast` and REQUIRE the link line AND that every object whose depfile names
the mutated file was rebuilt after the edit (the coordinator's lesson: a verdict counts only when every
dependent object was recompiled — 2 objects for the program and the shell, 20 for the panel); run the
WHOLE suite capped (RLIMIT_AS 4 GiB, 600 s); restore byte for byte in `finally` (`cmp`: identical, all
41) and bump the timestamp. After the pass every A2a file matched its pre-pass checksum
(`mut/pre_md5.txt`, `md5sum -c`: 7 of 7 OK), and the binary — the last mutant's link — was thrown
away: a clean rebuild from source followed (A2a.6). **No mutant was killed by the cap or the timeout**;
M13 and M21 hang the operator (no live button) and are caught by the rig's own 900 s deadline, i.e. by
the tests' `REQUIRE_FALSE(rig.timedOut)`.

| # | Brief | The mutant | Verdict | Caught by (first failing line) |
|---|---|---|---|---|
| M4 | 4 | USE NEW saves the measurement turned to the card's front (KEEP OLD) | RED | T2b :519 `sidesOf(d) == kOtherEndSides`; T2d, T5a, T5e, T5f |
| M5 | 5 | the front check skipped (FLIPPED never holds) | RED | T2a, T2b, T2c, T2j, T5f |
| M6 | 6 | RUN AGAIN on a held screen saves the held run first | RED | T2a (replaces while held), T5b, T5c, T5j |
| M7 | 7 | the check reads the record FILE only | RED | T2i |
| M8 | 8 | no card prints the FIRST words | RED | T1e, T5d |
| M9 | 9 | the panel told Saved on Unconfirmed (B1 m1) | RED | T4 :828/:832 (the crash case's header and follow-up) |
| M10 | 10 | `BootReadsBackup::Yes` (B1 m2) | RED | T4 (words ≠ the replay's), T4b |
| M11 | 11 | a direct record write beside the safe save (B1 m3) | RED | T4 (ops ≠ the replay's), T4b (`.replace(`) |
| M12 | 12 | the watch asks the provider for a declared port and commands 0 V | RED | T1a, T5a, T5b, T5f, T5h, T5j, T6d |
| M13 | 13 | the watch's threshold 0 | RED | 27 cases: every unmoved claw "turned" → held → `REQUIRE_FALSE(rig.timedOut)` |
| M14 | 14 | the declared-port drop removed | RED | T1a (`motorRequestCount(6) == 0`), T1d, T5a, T5b, T5e, T5f, T5h, … (10) |
| M15 | 15 | the drive-kind refusal moved after `buildMotors()` | RED | T6b :1300 `counting.requests == 0`; PC6 (the real adapters wrote gearing) |
| M18 | 18 | the watch's read constructs `pros::Motor{port, blue}` | RED | PC1 (`setGearingCalls`), PC7 (the pin) |
| M19 | 19 | `program_detail` instead of `detail` | RED | `api_doc_tool check-coverage`: `ProgramRun`'s members undocumented |
| M20 | 20 | `viewFor()`'s rule default flipped to ByPush | RED | today's `calibrate_panel_test` P1–P4 (the WHICH END cases), N1, N2 |
| M21 | 21 | the auto-answer removed | RED | 33 cases: the run stops at a screen with no live button → `timedOut` |
| M22 | 22 | `TouchGate::touch` reads the held left box as `FrontFirst` | RED | N4 :367, N5; T2b, T2d, T5a, T5e, T5f (9) |
| M23 | 23 | the MECHANISM PORT TURNED hold removed (warn only) | RED | T5a–T5e, T5g |
| M24 | 24 | USE NEW saves without the pending MECHANISM question | RED | T5f :1063 (no MECHANISM PORT TURNED screen) |
| M25 | 25 | the "did not turn" DONE line dropped | RED | T5b, T5b-1, T6d |
| M26 | 26 | `factsProblem()` skips a NAMED port-0 entry | RED — **compile time, by the TEST's own assertion**: `calibrate_program_test.cpp:1505 static assertion failed` (T6e's `namedZero`), not the mutation's own breakage |
| M27 | 27 | the not-Tank check removed | RED | T6b (x-drive, h-drive) |
| M43 | 43 (A2a's untyped form) | the discard words ignore the card's contents | RED | T5c :994/:996 |
| M45 | 45 | the FIRST decided by the record FILE alone | RED | T2f |
| M46 | 46 | the per-file lines skip the backup | RED | T2e, T2f |
| M47 | 47 | an EMPTY file printed with the decoder's (empty) reason | RED | T2g |
| M48 | 48 | no card: the discard words ignore it | RED | T5d |
| M49 | 49 | no card: the `card :` line says "no usable record" | RED | T1e, PC6 |
| X1 | A2a.1 #4 | the arming clock not restarted on a change of held question | RED | N5 :401/:402, T5f |
| X2 | A2a.1 #4 | no `screen:` line for the second held question | RED | T5f :1063 |
| X3 | A2a.2 #2 | the watch's `>=` made `>` (the exact edge) | RED | T5g (exactly the threshold) |
| X4 | A2a.2 #2 | the NaN guard dropped | RED | T5h (the NaN read) |
| X5 | A2a.2 #1 | the facts checked AFTER the drop | RED | T6a (a `(declared)` line before the refusal) |
| X6 | A2a.1 #3 | a REFUSED push judged by the watch | RED | T1d, T5j (one read too many) |
| X7 | A2a.1 #5 | the NOTE drawn over the warning's second row | RED | T1c, N6 |
| X8 | A2a.1 #2 | the cartridge byte not checked | RED | T6b (the calibrator's precondition throws out of `run()`) |
| X9 | SER1 | the shell's `line()` back to `printf` + `fflush` | RED | PC2 (one 407-byte write, not two tagged pieces), PC7 |
| X10 | A2a.1 #1 | the build time in a function-`static` buffer | RED | T1k :455/:456 |
| X11 | shell | `kCompetitionAutonomous` 0x08 | RED — **compile time, by the SHELL's own guard**: `hal/pros/calibrate.hpp:61 static assertion failed: calibrate_program.hpp's competition bits must be PROS's` (a header guard, not a test assertion) |
| X12 | landmine 15 | a third card read before the save | RED | T4 :805 (`a.size() == b.size()`: one read more than the replay) |
| X13 | test 2/5 | a discarded hold's state kept | RED | T2j :582/:583 ("on the held" printed after a STOPPED run) |

**41 of 41 RED** — the brief's A2a set (4–15, 18, 19, 20–27, 43, 45–49: 28) and 13 of this run's own.
Three tests were written BECAUSE a planned mutant had no test that could see it (T1k for X10, T2j for
X13, T5j's read count for X6), each before the pass ran.

## A2a.6 Prose review (pace rule 4) — grepped, every hit in a living document READ, then logged (2026-10-22)

**Grepped** (GNU `command grep` — A1's note: this shell's `grep` wraps `ugrep`) over `README.md`,
`docs/*.md`, `docs/guide/*.md` and the living internal documents (build-order, HANDOFF, REMAINING-WORK,
ORIENTATION, PROJECT-BRIEFING, RESUMING, the worksheet): the brief's §3.1 list — `shulib Calibrate`,
`calibrate_program`, `viewFor`, `TouchGate`, `IBrainScreen`, `screenSafeLine`, `printText`,
`record_store`, `saveRecordSafely`, the front (`front check`, `FRONT FLIPPED`, `WHICH END`) — and
A2a's own names: `CalibrateProgram`, `CalibrateFacts`, `hal/pros/calibrate`, `runCalibrate`,
`calibrateInitialize`, `ProsCalibrateIo`, `HoldReason`, `FinishFacts`, `FrontRule`, `MECHANISM PORT`,
`HA-187`, `standard Calibrate`, `still to come`, `Part A2`; and the concept that bit A1 (R14, F1): a
COUNT of `hal/pros` headers (`hal/pros` within 40 characters of a numeral or a number word before
`header`/`file`/`adapter`, and `twenty`/`nineteen`/`eighteen`/`hardware layer`).

**Read, and still true after A2a** (A2a changes no program, so a line describing robot two's Calibrate
stays true — each is Part B's to reword, as the brief's §3.2 lists): `docs/faq.md` :154-175 (the safe
save; "the programs do not use it yet: until "shulib Calibrate" saves through it" — no program uses the
library's Calibrate yet) and :252; `docs/hardware-assumptions.md` :39 (HA-187..HA-188 HELD — A2b
registers HA-187), :1910, :1949, :2016, :2224 (`src/programs/calibrate_program.cpp`'s drop and
disable — the program, unchanged), :2338-2400 (SAFE-SAVE: "shulib Calibrate does not use it until
SAFE-SAVE Part B1" — true of the program), :2565 (HA-178), and the R5a rows naming Calibrate's GAINS
step (Part C's); `docs/roadmap.md` :245, :293, :353, :374-380 (history and the frontier), :442, :451,
:1197-1294 (R5a's); `docs/guide/07-getting-set-up.md` :143 (the program list), guide 16 :145-146 and
guide 17 :346 (robot two's front, the record's — unchanged); HANDOFF :117 (robot one's ports are
discovered), :296 (the program's path), :712; ORIENTATION :104 (history); the worksheet's `WHICH END`
hits (:452, :540, :624, :865 — robot two's program: Part B's Station G rewrite; D.1.2 and F.2.1 stay as
written, the brief's R3).

**Fixed (made false, or left stale, by A2a's new header):**
- `docs/internal/HANDOFF-remaining-work.md` :264-269 — "(`hal/pros/`, 20 headers)" → 21, and the list
  names the standard Calibrate's shell (not an adapter).
- `docs/internal/ORIENTATION.md` :94 — "the V5 hardware layer (twenty files)" → "twenty-one files".
- `README.md` :96-98 — the touch screen's adapter "is the library's standard Calibrate's, still to
  come" → "…, which no program builds yet" (the program exists now; no program builds it); :277-280 —
  `calibration/`'s line names the standard Calibrate program and its shell. README :88's "Eighteen
  hardware adapters" stays true (its count leaves out the device provider and the SD store, and the
  shell is not an adapter).
- `docs/internal/build-order.md` :1649 (CAL1's own block) — the program's "1,387 lines" was SER1's
  stale count: "1,387 lines when CAL1 was added; 1,397 since SER1".

**Found lost, and restored — for the coordinator:** `PROJECT-BRIEFING.md` :192-197 and :743-748 —
A1's R14 fix ("eighteen since CAL1 Part A1 … construct most of them"; "the brain's touch screen
`brain_screen` (CAL1 Part A1) … 18 adapter headers") is on branch `cal1-a1` but NOT on `ceb17c0`: the
A1 merge kept main's text there (the briefing's generated block conflicted; these hand-written lines
went with it). Restored from `git show cal1-a1:docs/internal/PROJECT-BRIEFING.md`, and the 18-adapter
sentence now also names the device provider, the SD store and A2a's shell as not adapters.

**Deliberately NOT touched:** `docs/hardware-assumptions.md` (HA-187 is A2b's to register — the split;
A2a takes no HA number and found no NEW hardware belief: the shell's C reads are HA-187's claim, its
serial line SER1's HA-189/190/191, its composition the adapters' existing rows); the guide (the chapter
and guide 12 are A2b's); the FAQ (true, above); the worksheet (Part B's); `RESUMING.md`; the brief;
`test/pros_shim/pros/shim_control.hpp` (§2.A test 9: an existing test-tree file A2 may not edit — its
`resetAll()` still does not reach A1's three resets; the new tests call each themselves); other chunks'
files.

## A2a.R2 Second resumption (2026-10-22 08:44 EDT, `date`)

- **Why.** The first resumption (A2a.R, 00:45) was STOPPED at ~01:45 when the team lead dropped to one job
  near a usage limit; resumed ~08:50 the same day. Executor: **Claude Opus 5.5 (1M context)**, the same
  worktree and branch. Its last log entry is A2a.6 (01:37); its scratch shows it then ran the ARM header
  gate, the CI greps and the api gates (`scratch/cal1-a2a/gates/`, 01:34), linked a scratch team project
  (`team_proj/`, 01:35) and STARTED `src_build_gate.py self-test` (`gates/srcgate.start` 01:35:20, its log
  empty) — the stop cut it. None of that is taken from the files: every gate is re-run below.
- **State found:** `git diff -- src/ tools/ Makefile test/CMakeLists.txt test/pros_shim .github` is 0 lines —
  the killed self-test left NO plant in `src/` or the `Makefile`. `git status` lists the 22 files A2a.R
  ended with (14 modified, 8 new), the coordinator's count. The code's mtimes (01:26–01:29) are the
  mutation pass's restores; the coordinator found every backup under `mut/` equal to its file.
  `build/test/shulib_tests` EXISTS and may be the last mutant's link or a stale one: removed before any
  result, and rebuilt from source.
- **Fresh build and suite (08:44–08:46):** `briefing_status.py generate` with no binary; `cmake -S test -B
  build/test`; `cmake --build build/test -j8 --clean-first` capped (`ulimit -v 4194304; timeout 3000`):
  exit 0, 232 objects compiled, 0 warnings (`scratch/cal1-a2a/logs/g_clean_build.log`). The suite, capped:
  **2,530 passed | 0 failed | 3 skipped, 2,929,054 assertions**, 38 s, peak RSS 61,852 KB (`logs/g_suite.xml`,
  `g_suite.time`). Against A2a.0's baseline (`b0_suite.xml`, case by case, `cmpxml.py`): 53 new cases (40 + 7 +
  6), none gone, 0 failures; the only pre-existing cases whose COUNTS changed are scanners that read every
  header (`calibrate_panel_test` P9 +795 — `kAll`'s new literals; the retired-Teleop and mechanism-table
  scans; the PROS fence's four; SAFE-SAVE A's platform scan) — every one passing, none edited.
- **A finding about A2a.5's evidence (08:52).** `md5sum -c mut/pre_md5.txt` now: 6 of 7 OK,
  **`test/calibrate_program_test.cpp` FAILED** — it was edited at 01:29:36, AFTER the pass's own post-check
  (01:28). The pass's cited failing lines sit 7 lines lower now from line ~120 on (e.g. T2b's
  `sidesOf(d) == kOtherEndSides` :519 → :526, `checkDeclaredUntouched` :120 → :127): 7 lines were added in
  the file's first 120 lines (its header comment or helpers), and no copy of the pre-edit file exists to say
  which. So A2a.5's 41 verdicts were measured against a test file that is not the one in the tree. **The pass
  is RE-RUN in full on the current files** (A2a.7), not taken from `mutations.log`.
- **A gap found while re-reading the watch (08:55).** Every rig scenario starts both claws' readings at
  0 deg, so two wrong watches would pass the whole suite: one that judges a claw's READING instead of its
  travel since the push began (`moved = now`), and one that judges a port whose BASELINE read failed
  against a baseline of 0 (the `run.watchRead[k] &&` guard dropped) — the rig's refused port refuses every
  read, so its DONE read never answers. On a robot a claw's encoder holds whatever the last program left.
  A test is added for both (T5k, A2a.7) BEFORE the re-run, and the two mutations join the list (X14, X15).

## A2a.7 The watch's two gaps closed, and EVERY mutation re-run on the files as they stand (2026-10-22 09:15–09:41)

- **The gap, proven before the test** (`scratch/cal1-a2a/mut2/mutations.log`, "survival check", 09:17, on a
  fresh link of the CURRENT tests): **X14** (`const bool read = d_.io.readShaftDegrees(m.port, now);` — the
  `run.watchRead[k] &&` guard dropped) **SURVIVED: GREEN, 2,530 of 2,530.** X15 (`const double moved = now;` —
  the reading, not the travel) was already RED, but only by accident: T5b's port 12 is a DRIVE motor whose
  position is not zero when run 2's push begins (`countPrefix(L, kMechWarn12) == 2`, :963) — no claw case.
- **The rig** (`test/calibrate_program_rig.hpp`, a new A2a file): `Scenario::clawStartDeg[2]` (the claws'
  readings when the program starts — :141, set at :347-348; `clawExactDeg` is now TRAVEL from it, :770-771,
  `0.0 + x` exactly, so T5g's edge is unchanged) and `Scenario::unreadableAtPushPort` (the watch's FIRST read on
  that port refused, every later read answers — :144, :846-847, counted in `shaftReads_` :816). Its header
  comment names both.
- **T5k** (`test/calibrate_program_test.cpp` :1247; the file's header comment names it): claws parked at
  +1234.5 and −987.25 deg (exact binary fractions) that the push does not turn → both `did not turn with the
  push (+0.0 deg)`, no WARNING, no hold, SAVED, each read twice, untouched; the parked claw on 6 turned by the
  push → T5a's `kMechWarn6` (+1692.4 deg, the travel — never 2926.9, the reading) and T5a's held line; port 7's
  push-start read refused, its DONE read answering −987.25 → `could not be read -- not judged`, never "did not
  turn", no WARNING, no hold, SAVED. Green on the first run.
- **Pass 2 — the WHOLE list, 43 mutations** (A2a.5's 41 + X14, X15; `mut2/a2a_mutations_pass2.json`, the same
  runner copied to `mut2/run_mut.py`, its backups and log under `mut2/`, pass 1's files left as they were):
  checksums of the seven A2a files taken first (`mut2/pre_md5_pass2.txt`); each mutant built on a fresh link
  with every dependent object REBUILT after the edit (2 objects for the program and the shell, 20 for the
  panel), the whole suite run capped (RLIMIT_AS 4 GiB, 600 s), the file restored in `finally` (`cmp`:
  identical, all 43), timestamp bumped. After the pass `md5sum -c` 7 of 7 OK. **No kill by the cap or the
  timeout**; M13 and M21 hang the operator and are caught by the rig's 900 s deadline (`REQUIRE_FALSE(
  rig.timedOut)`), as in pass 1.

| # | The mutant | Verdict (pass 2, 09:19–09:40) | First failing line (the cases) |
|---|---|---|---|
| M4 | USE NEW saves the measurement turned to the card's front | RED | T2b :527 `sidesOf(d) == kOtherEndSides` (T2b, T2d, T5a, T5e, T5f) |
| M5 | the front check skipped | RED | T2a :485 `L[check + 1] == kFlip1` (T2a, T2b, T2c, T2j, T5f) |
| M6 | RUN AGAIN on a held screen saves the held run first | RED | T2a :479 `timedOut` (T2a, T5b, T5c, T5j) |
| M7 | the check reads the record FILE only | RED | T2i :681 |
| M8 | no card prints the FIRST words | RED | T1e :350 (T1e, T5d) |
| M9 | the panel told Saved on Unconfirmed | RED | T4 :836 |
| M10 | `BootReadsBackup::Yes` | RED | T4 :832 (T4, T4b) |
| M11 | a direct record write beside the safe save | RED | T4 :813 `a.size() == b.size()` (T4, T4b) |
| M12 | the watch asks for and commands a declared port | RED | T1a :128 (8 cases, T5k among them) |
| M13 | the watch's threshold 0 | RED | T1a :170 `timedOut` (28 cases) |
| M14 | the declared-port drop removed | RED | T1a :187 (11 cases) |
| M15 | the drive-kind refusal after `buildMotors()` | RED | T6b :1367 `counting.requests == 0` (T6b, PC6) |
| M18 | the watch's read constructs `pros::Motor{port, blue}` | RED | PC1 :213 (PC1, PC7) |
| M19 | `program_detail` | RED | `check-coverage`: `ProgramRun`'s members undocumented |
| M20 | `viewFor()`'s rule default ByPush | RED | N1 :251 `tally.differing == 0` (N1, N2, today's P1–P4) |
| M21 | the auto-answer removed | RED | T1a :170 `timedOut` (34 cases) |
| M22 | the held left box read as `FrontFirst` | RED | N4 :367 (10 cases) |
| M23 | MECHANISM PORT TURNED warns only | RED | T5a :915 (T5a–T5e, T5g, T5k) |
| M24 | USE NEW skips the pending MECHANISM question | RED | T5f :1071 |
| M25 | the "did not turn" DONE line dropped | RED | T5b :986 (T5b, T5b-1, T6d) |
| M26 | `factsProblem()` skips a NAMED port 0 | RED — **at compile time, by the TEST's own assertion**: `calibrate_program_test.cpp:1572:47: static assertion failed` = T6e's `static_assert(factsProblem(namedZero).why != nullptr && …)`, not the mutation's own breakage |
| M27 | the not-Tank check removed | RED | T6b :1367 |
| M43 | the discard words ignore the card's contents | RED | T5c :1002 |
| M45 | the FIRST decided by the record FILE alone | RED | T2f :624 |
| M46 | the per-file lines skip the backup | RED | T2e :610 (T2e, T2f) |
| M47 | an EMPTY file with the decoder's reason | RED | T2g :644 |
| M48 | no card: the discard words ignore it | RED | T5d :1024 |
| M49 | no card: the `card :` line says "no usable record" | RED | T1e :347 (T1e, PC6) |
| X1 | the arming clock not restarted on a new held question | RED | N5 :401 (N5, T5f) |
| X2 | no `screen:` line for the second held question | RED | T5f :1071 |
| X3 | the watch's `>=` made `>` | RED | T5g :1117 (exactly the threshold) |
| X4 | the NaN guard dropped | RED | T5h :1156 |
| X5 | the facts checked AFTER the drop | RED | T6a :1412 |
| X6 | a REFUSED push judged by the watch | RED | T1d :335 (T1d, T5j) |
| X7 | the NOTE over the warning's second row | RED | N6 :447 (N6, T1c) |
| X8 | the cartridge byte not checked | RED | T6b :1417 (threw: the calibrator's precondition) |
| X9 | the shell's `line()` back to `printf` + `fflush` | RED | PC2 :141 (PC2, PC7) |
| X10 | the build time in a function-`static` buffer | RED | T1k :463 |
| X11 | `kCompetitionAutonomous` 0x08 | RED — **at compile time, by the SHELL's own guard** (`hal/pros/calibrate.hpp:61:26 static assertion failed: calibrate_program.hpp's competition bits must be PROS's`): a header guard, not a test assertion — it is the guard this mutation exists to test |
| X12 | a third card read before the save | RED | T4 :813 |
| X13 | a discarded hold's state kept | RED | T2j :590 |
| **X14** | **the watch judges a port whose BASELINE read failed** | **RED (SURVIVED before T5k)** | **T5k :1294 `timedOut`** — the WARNING holds a run nobody answers |
| **X15** | **the watch judges the claw's READING, not its travel** | RED | T5b :964 and now **T5k :1259** (the parked claws) |

**43 of 43 RED**, on the files as they stand: the brief's A2a set (4–15, 18, 19, 20–27, 43, 45–49: 28) and
15 of this run's own (X1–X15). **Pass 1 (A2a.5) is SUPERSEDED by this table**: its verdicts were measured on a
test file that was edited after it (A2a.R2), so they are kept as history, not cited.

- **After the pass:** the binary (X15's link) thrown away; `briefing_status.py generate` with no binary;
  `cmake --build build/test -j8 --clean-first` capped: exit 0, 232 objects, 0 warnings; the suite capped:
  **2,531 passed | 0 failed | 3 skipped, 2,929,086 assertions**, 24 s, peak RSS 62,064 KB (`logs/h_suite.xml`,
  `h_suite.time`); against A2a.0's baseline 54 new cases (41 + 7 + 6), none gone, 0 failures, the same eight
  scanner counts changed as at 08:46; the briefing regenerated over that binary, and a second build's doc gates
  all pass (`logs/h_build2.log`).

## A2a.8 The gates — every one re-run in this resumption (2026-10-22 08:44–09:45)

- **Host build** (`cmake -S test -B build/test`, `cmake --build build/test -j8`, the doc-gate step included):
  exit 0, 0 warnings, twice from clean (08:44, 09:41).
- **The suite:** 2,531 | 2,531 passed, 3 skipped (A2a.7).
- **`src_build_gate.py self-test`:** `src build gate self-test: OK (235 detector cases, each against a real
  build)`, 08:49–09:08; **`check`:** `src build gate: PASS (all 7 builds link; …)` — each of bench, xdrive,
  tank, tank-drive, tank-calibrate, tank-autontest, tank-competition `exit 0, 11/11 TUs compiled, warnings: 8
  vendor-ignored, 0 disallowed` (`gates/g_srcgate_selftest.log`, `g_srcgate_check.log`). `git status` before and
  after both identical (no plant left); `git diff --stat ceb17c0 -- src/` EMPTY.
- **The ARM header gate** (RESUMING's command, over EVERY header; the TU under `gates/`, never /tmp): exit 0
  over **225** headers, 0 output lines; its control (`int x = d;`) fails as it must. Re-run at 09:45 on the
  final tree: the same.
- **The CI greps**, as `ci.yml` spells them: `library is PROS-free (all of include/shulib except hal/pros/)`;
  `layering holds: core is sim-free`.
- **The compile-refusal harness** (`ctest -R compile_refusal`, CI's step): `100% tests passed, 0 tests failed
  out of 200`; 200 TUs, 200 tests.
- **`api_doc_tool`:** self-test OK; `doc coverage: 4842 public entities across 185 headers, all documented`;
  check-fresh exit 0; `doc example scan: 541 quoted lines, all verbatim (7 source files); 6 listings of
  whole-run files, each one unbroken run`; `removability: no public doc references docs/internal/`.
- **`doc_staleness_audit.py`:** `clean (29 live docs, 0 numeric claims checked)`.
- **`briefing_status.py check`:** `generated block current` (the suite line 2,531 / 2,929,086, 225 headers).
- **The team file as a PROS project's only source** (§6, P6's method): `git archive` of the committed tree
  (`ceb17c0`, so no gate plant could be copied), A2a's three headers overlaid (`cmp`: identical), `src/` = the
  brief §9.3's 21 lines verbatim (extracted from the brief, `diff`: identical to the stopped run's copy) —
  `make`: exit 0, `bin/hot.package.bin` 80,940 bytes (text 79.04 KB), ONE warning, `liblvgl`'s own
  (`scratch/cal1-a2a/team_proj2/make.log`); its symbols carry `motor_get_position`, `motor_get_encoder_units`,
  `motor_get_gearing` and `competition_get_status` (the shell's C reads) and the program's own functions.
- **A slip, corrected:** the final api re-run at 09:45 wrote five one-line outputs under the session's
  scratchpad in /tmp — against this run's rule; deleted at once, nothing read from them but their last lines.

## A2a.9 Close of Part A2a (2026-10-22 09:46 EDT) — the DoD, item by item (A2's DoD as the split leaves it to A2a)

- [x] §1.2 items 6-7 (`calibration/calibrate_program.hpp`, `hal/pros/calibrate.hpp`) and the ONE changed
  library header (`calibrate_panel.hpp`, additions only), every public member documented (coverage, above);
  `git diff --stat ceb17c0 -- src/` EMPTY.
- [x] Tests 1, 4, 5, 5b, 6, 8b and 9 green, and test 2's untyped half (A2a.0's ruling); the suite's counts
  before (2,477) and after (2,531) logged, every pre-existing case passing, none edited. Tests 2's typed half,
  10 and 11 are A2b's (the split).
- [x] Mutations 4–15, 18, 19, 20–27, 43, 45–49 RED on fresh binaries, each logged with its failing line
  (A2a.7, pass 2); 28–42, 44 and 50–54 are typed (A2b's). 16 and 17 are A1's.
- [x] The same gates as A1 (A2a.8); `src_build_gate.py check` shows `src/` builds unchanged with the changed
  `calibrate_panel.hpp` in today's program's includes; the team file links as a PROS project.
- [~] HA-187 — A2b's to register (the split); A2a found NO new hardware belief (the shell's C reads are
  HA-187's claim; its serial line SER1's). Docs per §3.1's A2a share (A2a.6); the PROGRESS log carries every
  ruling beyond the brief with its reason (A2a.0, A2a.1, A2a.2, A2a.R2, A2a.7).
- [~] The 34 golden scenarios, `typed-card-wins` and `typed-nocard` — A2b's (the split). Every new screen
  string of A2a is inside `kAll`'s compile-time check (the six typed literals are A2b's), the kept NOTE's widest
  form checked in the program (`detail::kProgramWidestKeptNote`).

**For the coordinator:**
1. **Shared files this part touches (only its own lines):** `docs/changelog.md` (one entry at the top of
   `## API 2.4`), `mkdocs.yml` (two nav lines), `docs/roadmap.md` (one "Now" paragraph), `docs/internal/
   build-order.md` (CAL1's status block :1622-1631, the `Next:` block's CAL1 clause at :632, :1649's line
   count), `HANDOFF-remaining-work.md` :264 and :268-271, `REMAINING-WORK.md` :87 (18b), `ORIENTATION.md` :94,
   `README.md` :97-98 and :279-280, `PROJECT-BRIEFING.md` (generated block :110-116, and A1's two lost hand-written passages restored at
   :193-197 and :746-749 — A2a.6),
   `docs/api/README.md` and `all-entities.md` (generated). MECH-A3 edits the changelog, `mkdocs.yml` and
   likely the plan docs too: merge word by word; regenerate the briefing and `docs/api/` after the merge.
2. **Main moved under this worktree:** `docs/internal/RESUMING.md` on main now runs the suite through
   `tools/suite_run.py` (the coordinator's uncommitted tool fix); this worktree branches from `ceb17c0` and has
   neither — every suite run here was capped by hand as the task says.
3. **A2a.5 is superseded** (A2a.R2, A2a.7): pass 1's verdicts were taken on a test file edited after the pass;
   pass 2 is the evidence. X14 was a real gap (survived), now closed by T5k.
4. **Nothing committed, nothing pushed, no `pros`, no serial port.**

## CAL1-A2a review findings and their disposition (2026-10-22 18:02 EDT, `date`)

- **Started** ~17:45 (reading; this heading was appended at 18:02, after the first code edits — the log came late
  by those minutes, not by any result) on the worktree as A2a.9 left it (`git status`: 14 modified, 8 new; `git diff --stat ceb17c0 --
  src/` empty). Executor: **Claude Opus 5.5 (1M context)**. Three reviewers (correctness, tests, docs) returned 16
  findings; overlaps deduplicated below into 13 dispositions (R1/V2 and the tests lens's "two ports" are one; R24/V3 and
  V26 are one; R4 is its own). Each was RE-CHECKED against the code before anything changed; the reviewers' probes
  and mutant logs (`scratch/cal1-a2a/review-*/`) were read, not trusted — every verdict below is re-measured here.

### The findings, each re-checked, and what was done

**1. Two declared ports turned: the hold showed ONE port's evidence and SAVE answered for all (correctness R1,
tests V2 — major; REAL, reproduced).** Re-checked in the code: `watchReport` kept only the lowest turned port
(`turnedPort`), `turnedCount` was read only as `> 0`, and the held screen's lines, `onCard`, the DRIVES line, the
SAVE line and the NOTE were all that one port's. Reproduced on the pre-fix tree with the correctness reviewer's own
probe binary (`fix/probes/prefix_two_turned.out`): an intake on 6 and drive port 12 typed as a mechanism over a
card whose record drives with 12 — the screen says port 6 "is declared a mechanism: not in this record", no DRIVES
line anywhere, SAVE, and the card goes from `LEFT -11 +12 -13 +14 -15` to `LEFT -11 -13 +14 -15` in BOTH files.
**Root cause:** SAVE is a judgment about ONE port ("it is a mechanism the push turns -- keep it"), but the program
asked it once for all of them. **Fixed at the root — one question per port** (`calibrate_program.hpp`): the watch
keeps EVERY declared port that turned, in the facts' order (`ProgramRun::turnedPort[]/turnedDeg[]`); `Held` keeps,
per question, whether the card's record drives with THAT port (`onCardOf[]`, judged at DONE against the record the
check read); `holdMechanism()` asks the next one with its own turn and its own DRIVES verdict; `resolveHeld()` asks
the next after each SAVE and saves only after the LAST; RUN AGAIN or a disable on any question discards the run.
With several, the first says how many (`MECHANISM PORTS TURNED -- 2 declared ports turned with the push: each is
asked on its own screen, one at a time. NOTHING is saved until every one is answered SAVE; RUN AGAIN on any of them
saves nothing.`) and each says which it is (`MECHANISM PORT TURNED (1 of 2) -- ...`); the NOTE after several SAVEs
is `N PORTS KEPT AS MECHANISMS -- see the log` (widest form `21 PORTS ...` checked against the SMALL budget at
compile time, `detail::kProgramWidestKeptManyNote`). **One port: every word unchanged** (the brief's goldens).
The panel learns the question's port (`FinishFacts::holdPort`, an addition with a 0 default) so `TouchGate::view()`
restarts the arming clock on a new PORT as it does on a new reason — without it (`layoutKey()` sees no text), a
reflexive second tap after SAVE on port 6 would land on port 12's LIVE SAVE, the very hazard A2a.1 #4 closed for
FRONT FLIPPED → MECHANISM; `recordLayout()` prints a `screen:` line per question likewise.
**Alternatives not taken:** (a) one screen with the "priority" port (on-card first) and a "+N more -- see the log"
count (the reviewers' first suggestion) — the screen has no free row or column (the MEDIUM line already runs 34 of
its 36 columns for `port 12: -1692 deg, drive 1692 deg`, the LARGE `NOTHING SAVED YET` 17 of 18, the boxes start at
y 72), so the other ports' evidence would live on serial only, and SAVE
would still answer for a port whose evidence the operator never saw; (b) on-card ports asked first — every port is
asked before anything is saved, so the order changes no outcome, and the facts' order is the WARNING lines' order.
**Proof:** T5l (the reproduced hazard: SAVE on 6, then port 12's question shows `the card's record DRIVES with
port 12` on the screen and serial, RUN AGAIN, then a disable on run 2's first question — both files byte-identical,
no replace), T5m (SAVE on both: the reflexive tap after the first SAVE lands GREYED, no replace before the second
deliberate SAVE, the record as measured, the NOTE `2 PORTS KEPT`), T5n (the facts' order: 12 listed first is asked
first and RUN AGAIN ends it; two claws on an empty card: two questions, no DRIVES line), N5b (the panel: a new
`holdPort` restarts the clock, the same port does not, FRONT FLIPPED reads none); the rig gained `clawTurnsToo`
and `heldMechNext[]` (per-question answers). **For the brief (the coordinator's, not edited here):** §1.4 and D4
word the hold for ONE port; proposed Re-check note — "R19 (CAL1-A2a review, 2026-10-22): with several declared
ports turned, each is its own MECHANISM PORT TURNED question in the facts' order (its turn, whether the card's
record drives with IT), the boxes re-greyed between, nothing saved until the last SAVE; one port's words unchanged.
A2b's golden scenarios are single-port and unaffected."

**2. The AUTONOMOUS exit untested; the held and finished lines said "the field disabled" on autonomous
(correctness R24, tests V3 and V26 — REAL).** Re-checked: the exit tested either bit, but every rig end set exactly
`0x01`, so dropping the autonomous bit (V3) or requiring `status == 0x01` (V26, which a real field's `0x05` would
run past) survived; the PROGRAM FAULT idle exited on the disabled bit only (an autonomous start left it idling until
PROS deleted the task — harmless, everything stopped, but not the documented "returns"); and the reviewer's probe on
the pre-fix tree (`fix/probes/prefix_auton.out`) prints `the field disabled the robot on the held FRONT FLIPPED
screen` beside `status bits 0x06 ... disabled=no auton=yes`. **Fixed:** ONE test for "driver control ended"
(`detail::programDriverControlEnded()`: either bit) used by the loop AND the fault idle; the held and finished lines
name what happened from the status (`detail::programDriverControlEndWords()`: the disabled bit → `the field disabled
the robot`, the brief's words, unchanged; else `autonomous started`). The abort's own reason (`the field disabled
the robot, or autonomous started`) is today's program's words, kept: the status-bits line after it says which.
**Alternative not taken:** the reviewer's "word every line as the abort does ('… or autonomous started')" — it would
change the brief's held/finished disable words (A2b's goldens) and say "or" where the program KNOWS which. **The
rig:** `Scenario::endStatus` (the status every end of driver control sets), and a timed-out run now always adds the
disabled bit, so a program that honours one bit only fails on `timedOut` instead of hanging the suite. **Tests:** T1m
(0x02, 0x06 and 0x05 mid-pulses: exits on the tick, STOPPED, the bits line verbatim, every motor 0 V + Coast,
nothing written), T1n (autonomous after a finished run: its own words; a PROGRAM FAULT's idle returns on 0x06 within
one idle wait), T2k (autonomous on the held FRONT FLIPPED screen: its own words, the card byte for byte).

**3. C2's discard words over files that OPENED and do not decode (correctness R4 — REAL gap, code correct).**
Re-checked: `cardHadRecord = earlier.from != Which::None` is right, and the reviewer's pre-fix probe prints the right
words (`fix/probes/prefix_undecodable.out`), but no test discarded a hold over such files, so a mutant counting
opened files as a record survived. **Fixed (test):** T5o — the typo over two undecodable files, and over an
undecodable backup alone: RUN AGAIN and a disable both say `the card is unchanged (it holds no usable record)`,
`keeps its record` never appears, no DRIVES line, the files byte for byte as seeded.

**4. "The card's record DRIVES with port N" never asserted ABSENT over a record without N (tests V1 — REAL).**
Re-checked: T5f's held-screen check was a prefix that stopped before line 2; T5a's absence check ran on an empty
card. **Fixed (test):** T5f asserts the whole held `screen:` line (`... / it is declared a mechanism: not in this
record | [SAVE] [RUN AGAIN]`) and no `DRIVES with port` on serial or the screen; T5l asserts the DRIVES line present
for 12 and absent for 6 in one run.

**5. A held run's saved bytes not pinned to the measurement (tests V30, V31 — REAL).** Re-checked: T2b's
`encoded(decoded(bin)) == bin` holds for any bytes that decode, and `describe()` leaves out the IMU port and the
provenance. **Fixed (test):** `checkRecordIsTheMeasurement(rig, nth)` pins the saved record FIELD BY FIELD to what
that run PRINTED before any save — kind, cartridge, IMU port 2, source RobotMeasured, the provenance equal to the
run's `provenance:` line (and its `shulib Calibrate <stamp> | ROBOT-MEASURED: ` prefix), both signed sides equal to
its `AS MEASURED` lines, both scales to six places and the width to four as printed, the backup equal — in T1a, T2b,
T5a, T5e and T5m.

**6. Refusal edges never landed on (tests V9, V12 — REAL).** **Fixed (test):** T6c's edge — exactly two drive motors
in the census starts the calibrator (`calibrator started: 2 candidate(s)`, no REFUSED); T6c-2 — two IMUs refused
`the registry reports 2 IMUs`, before any device; three motors with one refused by the provider = exactly two made,
which starts (the post-construction edge, V7's).

**7. Untested refusal and warning paths (tests V4–V8, V11, V15 — REAL).** **Fixed (tests):** T6f (a stated IMU port
that differs: the WARNING verbatim and the registry's port used; none stated; the same one); T6c-2 (a motor the
provider will not make: the WARNING naming it and nine candidates; one made of three: `MOTORS REFUSED`, the IMU
never asked for, nothing powered; the IMU not made: `IMU REFUSED -- port 2 has no IMU`, motors made and never
powered, all 0 V + Coast); T5b-3 (the IMU's heading held — wheels up: `PULSE_ROLLED port 11` with the "declare it"
line — the rig's `Scenario::headingHeld`); T1a (the `push live:` lines: the first tick's, the change, two 5 s
keepalives and READY, verbatim, five in all); T1l (motors left at 5 V/Hold and −7 V/Brake by "the last program" are
0 V + Coast at the moment the IMU is asked for — its calibration starts as it is made — through the rig's new
`CountingProvider::onImu` hook). V15 was nearly equivalent before T1l: the calibrator's `start()` runs its own
`safeState()`, so the only window the program's own all-stop covers is the IMU request — which T1l now observes.

**8. The drop's false "NO MOTOR there" lines unseen (tests V10 — REAL).** **Fixed (test):** T1a asserts no `the
census found NO MOTOR there` line on robot two's facts (both claws plugged).

**9. Landmine 4's header sentence missing (docs — REAL, an instruction left undone and unlogged).** **Fixed:** the
FRONT CHECK block of `calibrate_program.hpp`'s header now says THE LIMIT — the check and the save read the card
separately; a misread GOOD record at the check makes a FIRST record with no flip check, SAFE-SAVE's two-fault limit
(HA-139; `record_store.hpp`, THE LIMIT); no third read is added; a FIRST RECORD note means "the check READ no
record". `docs/api/calibrate_program.md` regenerated (check-fresh: exit 0).

**10. "Today's program 33 of 33 byte-identical" not re-measured on A2a's panel (docs — REAL: the DoD line was ticked
on an argument).** **Measured** (`fix/p33/`, `build.sh`, `run.sh`): `git archive ceb17c0` (so no worktree file can
leak in), TODAY's `src/programs/calibrate_program.cpp` built twice with the brief's golden harness
(`brief-CAL1/harness/harness.cpp`, read-only; flags as `d6a/build_old.sh`, its card overlay; `shim_extra` dropped —
A1 put those shim pieces into `test/pros_shim`) — once over ceb17c0's `calibrate_panel.hpp` (`old_h`), once over
A2a's (`new_h`; the overlay is the panel as it stands after this round's fixes, its md5 equal to pass 3's
pre-checksum), the nine scenarios P28/P37 ran (`clean one slip18 nocard curved noimu flip-keep oldback disable`),
each from an empty card, capped: **33 of 33 files byte-identical** — `nocard.serial.log` after normalizing the card
directory's name in its one NOT SAVED line (`card_old` / `card_new`), exactly the brief's own normalization (P28).
(For the record: 32 of 33 also equal P37's `d6a/v2/p3r_old_landed/`, the pre-SER1 program's — the harness reads the
serial through the same path either way; nocard differs only by the directory.) A2a.9's DoD clause is now measured,
not argued.

**11. Two split rulings unlogged (docs — REAL).** Logged here as rulings of A2a (A2a.0 is history and is not
rewritten): **(1) the changelog entry** — the split gives "the changelog" to A2b (brief "Not settled" #1 (ii), §9.2),
but A2a adds public API, and the launch prompt asked for the `since 2.4` entry when it does; A2a WROTE it, and A2b
EXTENDS that entry with §3.1's typed items (TypedDrivePorts, the typed words) rather than writing a second one.
**(2) Test 8b's "C-read counter counts 2 per push"** — that count is through the real PROS shell across a push,
which needs A2b's golden harness: it is A2b's. Until then PC1 counts the shim's C reads per call (and that NOT ONE
write happens on any port) and T5j counts the watch's reads per push begun and per push accepted on the rig.

**12. HANDOFF's Calibration line (docs — REAL, older than A2a, made inconsistent by it).** **Fixed:**
`HANDOFF-remaining-work.md` :273 now names `calibrate_panel.hpp`, (SAFE-SAVE Part A) `record_store.hpp`, (CAL1 A1)
`calibrate_screen.hpp` and `front_check.hpp`, and (CAL1 A2a) `calibrate_program.hpp` — host-only, no program builds
it yet.

**13. Main moved further than A2a.9 said (docs — REAL).** Measured (`git log ceb17c0..shulib-v2`): **four** commits —
`71b4764` (VIS1's answers; the register's Held line gains HA-192..HA-203; 70 chunks), `c09e022` (`tools/suite_run.py`;
`briefing_status.py` and `doc_staleness_audit.py` changed), `43ac361` (MECH-A3: the changelog, `mkdocs.yml`, README,
`docs/api/`, the plan docs), `73044ca` (L6/IMU1 answered: Held HA-204..HA-215, so the next free number is now
**HA-216**; **71** chunks; build-order and the register). Files BOTH sides touched (merge word by word, touch only
A2a's lines): `README.md`, `docs/changelog.md` (A2a's entry goes at the top of `## API 2.4`, above MECH-A3's — newest first,
and A2a merges after it), `mkdocs.yml`, `docs/api/README.md` and `all-entities.md` (generated: regenerate after
the merge, never hand-merge), `HANDOFF-remaining-work.md`, `PROJECT-BRIEFING.md` (regenerate with MAIN's
`briefing_status.py`, after the merged build), `REMAINING-WORK.md` (18b's row; add "Part A2a landed 2026-10-22" to
its "Where things stand" sentence about CAL1, which main now words), `build-order.md` (its `**Next:**` line: keep
main's text and add A2a's CAL1 clause), `docs/roadmap.md`. The worktree's briefing says "37 of 69" and "next free is
HA-192": true at ceb17c0, stale against main — regenerate at the merge. A2a touches none of `hardware-assumptions.md`,
`RESUMING.md`, `test/CMakeLists.txt` or `tools/`.

**14. Three wording slips (docs — REAL).** **Fixed:** (1) the changelog: "its box greyed again so a second tap cannot
answer it" → each new question's boxes "greyed again for the arming delay, so a reflexive second tap cannot answer
it"; (2) `hal/pros/calibrate.hpp` :36 "not the belief" → "not the belief itself" (as its siblings; regenerated into
`docs/api/pros-calibrate.md`); (3) `FrontChoice::None` → "not asked: no FRONT FLIPPED hold — the same front, the
card's first record, or a front that could not be checked (files that do not decode, or CANNOT TELL)".

**Not raised, re-checked anyway:** the tests reviewer's V13 (RUN AGAIN without `painter_.invalidate()`) and V14 (RUN
AGAIN without its all-stop) also survived. V14 is EQUIVALENT: the next line starts a new calibrator whose `start()`
runs `safeState()` (0 V + Coast on every candidate) before any tick. V13 is equivalent in what the screen SHOWS (the
painter redraws only what changed, and a correct diff draws the same final screen); the full repaint is a guard
against a stale cache, not an observable difference on the host screen. Both re-run in pass 3 below for the record.

### Mutation pass 3 — the WHOLE list again, on the files as they stand after the fixes (2026-10-22 18:05–18:52; 3b 18:53–19:03)

**Method:** pass 2's runner (`scratch/cal1-a2a/fix/mut3/run_mut.py`, a copy; list `pass3.json`, log `mutations.log`,
backups under `fix/mut3/`, never /tmp) — each mutant on a FRESH link with every object depending on the mutated file
REBUILT after the edit (2 objects for the program and the shell, 20 for the panel), the WHOLE suite run capped
(RLIMIT_AS 4 GiB, 600 s), the file restored in `finally` (`cmp`: identical, 77 of 77), its timestamp bumped;
`md5sum -c` of the seven A2a code and test files before the pass: 7 of 7 OK after it. Before the pass, the unmutated
suite: 2,543 / 2,543 (`fix/suite_pre_mut.out`). **77 mutations:** pass 2's 43 (four anchors re-cut for the reworked
hold — M23, M24, X1, X2 — the same bug, on the new code), the tests reviewer's 19 that apply (V2's code no longer
exists: Y2/Y3 are its forms on the new code; V1, V3 and V26 re-cut onto the new code), the correctness reviewer's R4,
and 14 of this round's own (Y1–Y12, Y15, Y16) on the per-port questions and the end of driver control.
**Pass 3b:** the rig's deadline then changed (a timed-out run now ends with exactly 0x01 — under pass 3, V26 made the
rig's "0x05 | 0x01" status never equal 0x01, so the suite HUNG and was killed by the 600 s timeout: caught, but by
the cap, not an assertion); the five mutants whose verdict involves the deadline (M13, M21, V3, V26, Y12) were re-run
on the final rig, 7 of 7 files `md5sum -c` OK after: all RED by a test's assertion.

| # | The mutant | Verdict | First failing line (the cases) |
|---|---|---|---|
| M4 | USE NEW saves the measurement turned to the card's front | RED | T2b :698 (T2b, T2d, T5a, T5e, T5f, T5m, T5n) |
| M5 | the front check skipped | RED | T2a :656 (T2a, T2b, T2c, T2k, T2j, T5f) |
| M6 | RUN AGAIN on a held screen saves the held run first | RED | T2a :650 (T2a, T5b, T5c, T5j, T5l, T5n, T5o) |
| M7 | the check reads the record FILE only | RED | T2i :876 |
| M8 | no card prints the FIRST words | RED | T1e :419 (T1e, T5d) |
| M9 | the panel told Saved on Unconfirmed | RED | T4 :1031 |
| M10 | `BootReadsBackup::Yes` | RED | T4 :1027 (T4, T4b) |
| M11 | a direct record write beside the safe save | RED | T4 :1008 (T4, T4b) |
| M12 | the watch asks for and commands a declared port | RED | T1a :138 (11 cases) |
| M13 | the watch's threshold 0 | RED (3b) | T1a :224 `timedOut` (33 cases in pass 3) |
| M14 | the declared-port drop removed | RED | T1a :241 (16 cases) |
| M15 | the drive-kind refusal after `buildMotors()` | RED | T6b :1800 `counting.requests == 0` (T6b, PC6) |
| M18 | the watch's read constructs `pros::Motor{port, blue}` | RED | PC1 :213 (PC1, PC7) |
| M19 | `program_detail` | RED | `check-coverage`: `ProgramRun`'s members undocumented (a tool's verdict) |
| M20 | `viewFor()`'s rule default ByPush | RED | N1 :254 (N1, N2, today's P1–P4) |
| M21 | the auto-answer removed | RED (3b) | T1a :224 `timedOut` (42 cases in pass 3) |
| M22 | the held left box read as `FrontFirst` | RED | N4 :370 (14 cases) |
| M23 | MECHANISM PORT TURNED warns only | RED | T5a :1110 (T5a–T5e, T5g, T5k–T5o) |
| M24 | USE NEW saves without the pending MECHANISM questions | RED | T5f :1268 |
| M25 | the "did not turn" DONE line dropped | RED | T5b :1182 (T5b, T5b-1, T6d) |
| M26 | `factsProblem()` skips a NAMED port 0 | RED — **at compile time, by the TEST's own assertion**: `calibrate_program_test.cpp:2147:47: static assertion failed` = T6e's `static_assert(factsProblem(namedZero).why != nullptr && …)`, not the mutation's own breakage |
| M27 | the not-Tank check removed | RED | T6b :1800 `counting.requests == 0` |
| M43 | the discard words ignore the card's contents | RED | T5c :1198 (T5c, T5o) |
| M45 | the FIRST decided by the record FILE alone | RED | T2f :819 (T2f, T5o) |
| M46 | the per-file lines skip the backup | RED | T2e :805 (T2e, T2f) |
| M47 | an EMPTY file with the decoder's reason | RED | T2g :839 |
| M48 | no card: the discard words ignore it | RED | T5d :1220 |
| M49 | no card: the `card :` line says "no usable record" | RED | T1e :416 (T1e, PC6) |
| X1 | the arming clock restarted on neither a new reason nor a new port | RED | N5 :404 (N5, N5b, T5f, T5m) |
| X2 | no `screen:` line for a new held question | RED | T5f :1268 (T5f, T5l, T5m, T5n) |
| X3 | the watch's `>=` made `>` | RED | T5g :1320 (exactly the threshold) |
| X4 | the NaN guard dropped | RED | T5h :1359 |
| X5 | the facts checked AFTER the drop | RED | T6a :1845 |
| X6 | a REFUSED push judged by the watch | RED | T1d :404 (T1d, T5j) |
| X7 | the NOTE over the warning's second row | RED | N6 :500 (N6, T1c) |
| X8 | the cartridge byte not checked | RED | T6b :1850 |
| X9 | the shell's `line()` back to `printf` + `fflush` | RED | PC2 :141 (PC2, PC7) |
| X10 | the build time in a function-`static` buffer | RED | T1k :532 |
| X11 | `kCompetitionAutonomous` 0x08 | RED — **at compile time, by the SHELL's own guard** (`hal/pros/calibrate.hpp:61:26 static assertion failed: calibrate_program.hpp's competition bits must be PROS's`): a header guard, not a test assertion — the guard this mutation exists to test |
| X12 | a third card read before the save | RED | T4 :1008 |
| X13 | a discarded hold's state kept | RED | T2j :785 |
| X14 | the watch judges a port whose baseline read failed | RED | T5k :1497 |
| X15 | the watch judges the claw's reading, not its travel | RED | T5b :1160 (T5b, T5k, T5l) |
| V1 | "the card's record DRIVES" whenever the card holds ANY record | RED (survived before this round) | T5f :1279 (T5f, T5l) |
| V3 | the loop ends on a disable only (autonomous runs on) | RED (survived) (3b) | T1m :591 `timedOut` (T1m, T1n, T2k) |
| V4 | the stated-vs-registry IMU port WARNING dropped | RED (survived) | T6f :2048 |
| V5 | the IMU refusal painted as MOTORS REFUSED | RED (survived) | T6c-2 :2021 |
| V6 | a motor the provider refuses dropped silently | RED (survived) | T6c-2 :1968 |
| V7 | after construction ONE motor is enough (`n < 1`) | RED (survived) | T6c-2 :1949 |
| V8 | the "declare it" line after PULSE_STILL only | RED (survived) | T5b-3 :1784 |
| V9 | TOO FEW below THREE (a two-motor tank refused) | RED (survived) | T6c :1941 (the edge) |
| V10 | the drop forgets what it found (false "NO MOTOR there") | RED (survived) | T1a :244 |
| V11 | no `push live:` line ever | RED (survived) | T1a :254 |
| V12 | TWO IMUs accepted | RED (survived) | T6c-2 :1800 `counting.requests == 0` (the refusal helper) |
| V13 | RUN AGAIN without `painter_.invalidate()` | **GREEN — equivalent** (not raised; finding "Not raised", above) | — |
| V14 | RUN AGAIN without its all-stop | **GREEN — equivalent**: the new calibrator's `start()` runs `safeState()` first | — |
| V15 | no 0 V + Coast before the calibrator | RED (survived) | T1l :565 |
| V16 | the state screen's state and reason swapped | RED | PC5 :325 |
| V20 | the watch judges SIGNED travel | RED | T5f :1268 (a claw turned backwards) |
| V26 | the exit only on a status EQUAL to 0x01 | RED (survived) (3b) | T1m :591 `timedOut` (0x05) — in pass 3, before the rig's fix, the 600 s TIMEOUT (caught by the cap) |
| V30 | a held save writes IMU port 3 | RED (survived) | T2b :175 `d.imuPort == 2` (T2b, T5a, T5e, T5m) |
| V31 | a held save overwrites the provenance | RED (survived) | T2b :177 (T2b, T5a, T5e, T5m) |
| R4 | opened files count as "the card keeps its record" | RED (survived) | T5o :1706 |
| Y1 | SAVE on the first question saves at once | RED | T5l :1566 (T5l, T5m, T5n) |
| Y2 | only the FIRST turned port kept (one question for all) | RED | T5l :1557 (T5l, T5m, T5n) |
| Y3 | only the LAST turned port kept (R1/V2 on the new code) | RED | T5l :1557 (T5l, T5m, T5n) |
| Y4 | each question shows question 1's DRIVES verdict | RED | T5l :1567 (T5l, T5m) |
| Y5 | each question names question 1's port | RED | T5l :1566 (T5l, T5m, T5n) |
| Y6 | `TouchGate` ignores `holdPort` (no re-grey per port) | RED | N5b :444 (N5b, T5m) |
| Y7 | no `screen:` line per port | RED | T5l :1569 (T5l, T5m, T5n) |
| Y8 | the several-ports NOTE names one port | RED | T5m :1632 (T5m, T5n) |
| Y9 | the count line before several questions dropped | RED | T5l :1557 (T5l, T5n) |
| Y10 | the "(k of n)" marker dropped | RED | T5l :1557 (T5l, T5n) |
| Y11 | the end words always "the field disabled the robot" | RED | T1n :618 (T1n, T2k) |
| Y12 | the fault's idle ends on a disable only | RED (3b) | T1n :630 `timedOut` |
| Y15 | a later question shows the first slot's travel | RED | T5l :1566 (T5l, T5m, T5n) |
| Y16 | "on the card" judged against the first port's side | RED | T5l :1567 (T5l, T5m) |

**75 of 77 RED** (the two GREEN, V13 and V14, equivalent — reasoned above; neither was a finding). Every mutant the
reviewers saw survive (V1, V3–V12, V15, V26, V30, V31, R4, and R1/V2 in its new forms Y2/Y3) is RED, each first by
a test written for it this round. Pass 2 (A2a.7) stays as history: its verdicts were on the files before this round.

## CAL1-A2a review findings and their disposition — the SECOND review round (2026-10-29 12:52 EDT, `date`)

- **Resumed** 2026-10-29 12:45 EDT (`date`) after the stop of 2026-10-22 (~19:15: the first fix round's
  `src_build_gate.py check` was killed mid-run — its log is 0 bytes — when the team lead dropped to one job; the
  week's limit followed). Executor: **Claude Opus 5.5 (1M context)**. This heading is written BEFORE any edit.
- **The worktree as found** (`git status`): 14 modified, 8 new; `git diff --stat ceb17c0 -- src/ Makefile tools/`
  EMPTY. **No mutant or plant left behind:** `md5sum -c fix/mut3/pre_md5_pass3b.txt` — 7 of 7 A2a code and test
  files OK; `calibrate_program.hpp` equals its newest backup (`Y12`), `calibrate_panel.hpp` its (`Y6`), the shell its
  (`X9`), each `cmp` identical; the reviewers mutated COPIES (`review-*/tree`), never the worktree. The test binary on
  disk (linked 2026-10-22 18:56) is REMOVED before any result here (`rm -f build/test/shulib_tests`): every number
  below is from a binary linked in this resumption.
- **Main has moved** (`43ac361` MECH-A3, `73044ca` L6/IMU1's plan, `c09e022` the bounded suite run, `71b4764`):
  none touches this part's files; `tools/suite_run.py` is not in this worktree (it branches from `ceb17c0`), so the
  suite runs here capped by hand (`ulimit -v 4194304; timeout 600`) and once through a COPY of main's
  `suite_run.py` in scratch pointed at this worktree's binary (below).
- Three reviewers (correctness r3, tests r2, docs r2) returned **18 findings**; the dispositions follow, each
  re-checked against the code first.

### The findings, each re-checked, and what was done

Each was re-checked in the code before anything changed; every "survived" below was READ in the reviewer's own
mutant log (`review-correctness/r3/mutlogs/`, `review-tests-r2/mut/mine2.out`, `mine_b.out`: a fresh link, the
suite capped, `2543 | 2543 passed`) and every fix is proved by pass 4 (below) on fresh binaries. Words a test now
pins were read off a scratch probe first (`fix2/probes/dump.cpp`, run capped; outputs `fix2/probes/out_*.txt`),
never computed by the code under test in the test.

**1. The port list after a refused motor was unpinned (correctness, major; R19 — REAL gap, code correct).**
`buildMotors()` compacts `ports` beside the made motors (`ports[built] = port`); T6c-2's refusals both refused port
20, the LAST, where an uncompacted list reads the same, and both ended before any save. Probe (`out_mid13.txt`, the
real code): `drive ports, signed: -11 +12 +14 -15 +16 -17 +18 -19 +20  (9 to pulse, ...)`, saved `LEFT -11 +12 +14
-15 | RIGHT +16 -17 +18 -19 +20`, 9 of 9 forward — the reviewer's figures exactly. **Fixed (test):** T6c-2's new
subcase refuses the MIDDLE port 13 and runs to SAVED: the WARNING, `calibrator started: 9 candidate(s)`, the signed
line, the record verbatim, `checkRecordIsTheMeasurement`, forward 9 of 9, port 13 asked for once and never
commanded, every motor stopped. R19 RED (pass 4).

**2. The RAII all-stop was untested on the exit only it covers (correctness, minor; R54 — REAL).** MOTORS REFUSED
returns between `buildMotors()` and the program's own `stopAll()`, so only `AllStopGuard` stops a made motor there;
the subcase's port 11 started at 0 V + Coast and could not tell. **Fixed (test):** port 11 is left at 5 V and HELD
by "the last program" (T1l's premise) before the run; after `run()` it is 0 V + Coast, and its powered-command count
is exactly the inherited one (the program powered nothing). R54 RED.

**3. No run had the field's "connected" bit set (correctness, minor; R7 — REAL).** The rig's status started at 0;
T1m's three statuses all END driver control. **Fixed:** `Scenario::startStatus` (the rig's status at the start).
**T1o**: a whole run under 0x04 reaches SAVED (the four steps, `===== RUN 1 ENDED: DONE =====`, the record field by
field, no STOPPED), and the field's disable 0x05 ends it with `status bits 0x05 field=yes disabled=yes auton=no`; a
PROGRAM FAULT under 0x04 idles to 60 s and returns on 0x05 within one idle wait. R7 RED.

**4. When the watch takes its baseline was unpinned (correctness, minor; R1 — REAL).** The tests reviewer's Q10 (arm
at the IMU step ONLY) was RED in their run, but only through the RUN AGAIN runs' read counts (T1b's
`watchReads[6] == 4`, T1d, T5b … — `mine2.out`) — no test saw a FIRST run's baseline taken early, which is why R1
(armed at the IMU step, never re-armed at the push) survived. The rig moved a claw only while pushing.
**Fixed:** `Scenario::clawSettlesBeforePush` — a declared claw's reading moves +30 deg at t = 1 s, in the IMU
warm-up (the IMU is ready at 2 s, so before any push). **T5p**: `watch: port 6 (claw pivot 6) did not turn with the
push (+0.0 deg)`, no WARNING, no hold, SAVED, two reads — and the premise read back after the run (the claw IS at
+30.0 deg). R1 and Q10 RED.

**5. Serial and screen words claimed a save that had not happened (correctness, minor — REAL, both).**
(1) **The touch line.** `-> SAVE AS PUSHED` was printed for every left-box touch on a held screen, including a USE
NEW with a MECHANISM question pending and every SAVE but the last — touches that save nothing. **Fixed at the root
(`calibrate_program.hpp`):** `heldAsksNext()` (FRONT FLIPPED with a MECHANISM question to come, or a MECHANISM
question that is not the last) words the touch by what it DOES — `-> USE NEW -- the next question follows; nothing
is saved yet` / `-> SAVE -- the next question follows; nothing is saved yet`; the touch that saves keeps `-> SAVE
AS PUSHED`. **Checked against the brief's goldens before choosing:** all six `SAVE AS PUSHED` lines in
`brief-CAL1/d6a/v3/run_new/` (`flip-new`, `swap`, `watch`, `watch-typo-save`, `typed-flip-new`,
`typed-undecodable-flip`) are touches that save at once, so A2b's goldens are untouched. Tests: T5f (USE NEW then the
MECHANISM question: the new words, and exactly one `SAVE AS PUSHED` — the last SAVE), T5l (SAVE on 6 with 12 to come:
the new words, no `SAVE AS PUSHED` in a run that saved nothing), T5m (the first SAVE's new words, the last SAVE's
`SAVE AS PUSHED`, one in all). (2) **The FIRST RECORD note under NOT SAVED.** The note was set before the save and
painted whatever its outcome. **Fixed:** `finishDone()` drops it when the save is not `Saved` (a local
`firstRecord`, set only on the FIRST branch); the serial keeps the check's own words (`this is the FIRST`), which
are true — the check READ no record. **T4c**: an empty card whose first record-file write is cut short (the dry run
finds the write; the control run paints the note): NOT SAVED, the save's own words (`it read back 30 of 180 bytes`,
`the next boot will REFUSE until a run saves`), no SAVED header, NO `FIRST RECORD` drawn. No golden combines NOT
SAVED with the note (grepped: none). The header's FRONT CHECK block and the changelog say it. Mutants Z1–Z5 RED.
**Alternative not taken** for (2): a distinct NOTE ("NO RECORD -- the next boot refuses") — a new panel literal
in a part whose panel changes are additions under a field-for-field test, for words the save's own serial lines and
the red header already carry.

**6. The program's stack on the opcontrol task was stated nowhere (correctness, minor — REAL: undocumented).**
**Re-measured on the FINAL headers** (`fix2/stack/`: the team project `fix2/team_proj4` — `git archive ceb17c0`, the
three A2a headers overlaid, `cmp` identical, the 21-line team file — compiled with the team project's flags plus
`-fstack-usage -fcallgraph-info=su`; the reviewer's `maxpath.py`, copied): deepest in-TU path **22,464 B** (opcontrol
8 → runCalibrate 8,712 → `CalibrateProgram::run` 10,272 → record 264 → finishDone 1,144 → save 464 →
saveRecordSafely 1,184 → SafeSave …), and `sizeof` on ARM: ProsDeviceProvider 7,288, DrivetrainCalibrator 7,712,
CalibrateProgram 1,360 — the reviewer's figures to the byte. PROS's default task stack is
`TASK_STACK_DEPTH_DEFAULT 0x2000` words (`include/pros/rtos.h:72`, 32 KiB): ~70 % before newlib's and VEXos's frames,
which GCC cannot see. **Fixed (docs):** a THE STACK paragraph in `hal/pros/calibrate.hpp`'s header (regenerated into
`docs/api/pros-calibrate.md`) and a clause in the changelog. **Alternative not taken:** the provider in static
storage — a static local outlives the call, and opcontrol() is entered again after every disable → enable on a
field, so a second run would get the first run's provider and every device it made (today's one-run-per-entry
contract changes), to buy margin on a stack that fits; if the frame grows (R5a Part C's GAINS and REST runs), that
is the place to weigh it. No hardware belief is added: the 32 KiB is PROS's source, the 22,464 B GCC's.

**7. HA-187's blast radius: the held screen's degrees depend on the gearing the motor REPORTS (correctness, minor,
for the coordinator — a refinement of a REGISTERED-NEXT belief, not a new one; no stop).** Re-checked:
`readShaftDegrees()` converts counts by the gearing the declared motor reports, and nothing in this program gears
that motor; the drive's travel comes from motors geared to the STATED cartridge. If an ungeared declared port reports
another gearing than its insert, the MEDIUM `port N: X deg, drive Y deg` line — the operator's evidence for SAVE vs
RUN AGAIN — is off by the gearing ratio (a BLUE drive motor reporting green shows ~⅓ of the drive's travel, which
reads as "not a drive motor" and invites SAVE). Detection is unaffected (5 deg against ~1,700). The brief's HA-187
(2) already says "within the reported gearing's scale"; its blast radius names only a missed watch line or a false
hold. **Handed to A2b** (it registers HA-187): add this to (2)'s blast radius, and settle it at Station G by reading
an unconfigured claw's `motor_get_gearing` before any program gears it. No code change here: the host shim cannot
model it, and re-wording the screen is a brief-level change (D4's held screen).

**8. PROGRAM FAULT's own all-stop was invisible (tests, major; Q1 — REAL).** T1i's fault fired on a motor's FIRST
powered command (nothing else powered) and its checks ran after `run()` returned (the guard had run). **Fixed (rig +
test):** `Scenario::faultPortInSpin` (the motor throws on its first powered command once the SPIN screen shows),
`RecordingMotor::beforeThrow` (the world read at the instant of the fault), `FakeIo::onDelay` (the world read as each
wait starts). **T1p**: the fault fires on port 20 mid-spin with **9** motors powered (asserted — the premise), and at
the fault idle's FIRST 250 ms wait every drive motor is 0 V + Coast; it idles until the disable at 200 s. Q1 RED.

**9. The fix round's gates were not closed (tests, major — REAL).** `fix/gates/srcgate_check.log` is 0 bytes: the
`check` was killed with the session on 2026-10-22 ~19:15. The first round's other gate outputs are cited in the close
of that round, below; every gate is re-run on THIS round's final tree (the gates, below), the src gate's `check`
included and completed.

**10. The shell's three "cannot be read" guards were each untested alone (tests, minor; S1–S3 — REAL).** The shim's
position read failed in all three of PC1's cases, so only the first guard was ever reached. **Fixed (shim + test):**
three dropout knobs on the shim's per-port state (`failPositionReads`, `failUnitsReads`, `failGearingReads` in
`test/pros_shim/pros/motors.hpp`; consumed one call at a time by the three C reads in `motors.h`, whose header now
says a read moves only a test's own knob) — a dropout landing between two reads of one port, as
`refuseSetBrakeMode` models for writes. PC1 reaches each guard ALONE (a failed position read with DEGREES units; a
finite COUNT read whose gearing read fails; a finite reading whose units read fails), checks each knob consumed, and
that the next read of the port answers −1692.4. S1, S2, S3 RED.

**11. A failed DONE read was untested (tests, minor; Q13 — REAL).** **Fixed:** `Scenario::unreadableAtDonePort` (the
baseline read answers; every later read is refused) and T5k's fourth subcase — port 6 parked at +1234.5: `could not
be read -- not judged`, no `did not turn`, no WARNING, nothing held, SAVED, both reads MADE. Q13 RED.

**12. Nine serial-record and touch behaviours with no test, and a weak assertion (tests, minor; Q2–Q9, Q14 — REAL,
each).**
- **Q2** (one touch-baseline try): **T1j-2** — touch errors for 30 ms only, inside the retry's 20 × 5 ms: no WARNING,
  and the first tap acts.
- **Q3** (RUN AGAIN's gate on the start's baseline) and **Q9** (run 2 numbered 1): **T1b** — `===== RUN 2 ENDED:
  DONE =====`, `RUN 1 ENDED` once, the RUN AGAIN release (`release=2`) logged exactly once, run 2's DONE is
  `release=3`.
- **Q4/Q5** (the push-live edges `>=` made `>`): the decision is now ONE function,
  `detail::programPushLogDue(changed, sinceLast, min, keepalive)` (both edges inclusive, documented), called by
  `record()` with the two constants; **T1s** lands on both edges EXACTLY (0.25 and 5.0 are exact in binary; a
  `static_assert` ties the constants to them) and a hair under each (`std::nextafter`), plus "nothing changed at the
  minimum is no reason" — the coordinator's equality-edge lesson. T1a's five verbatim `push live:` lines pin that the
  program uses it (Z6, the two intervals swapped, RED). Q4 and Q5 are re-cut onto the function.
  **[Corrected by the third verification round, below:** T1a's lines pin the two interval VALUES only — not the
  program's inclusive edges, nor that `record()` calls the function (W13d, W13e GREEN). **T1t** now lands the
  PROGRAM's own lines on both edges exactly.**]**
- **Q6** (the worst gap never measured): T1a's `loop:` line verbatim (`5496 ticks, worst gap between two ticks 10
  ms ...; 218 screen prints so far`), and **T1q** — `Scenario::stallAt/stallSteps`, one 50 ms tick in the push:
  `worst gap between two ticks 50 ms`.
- **Q7** (the config line's values swapped): T1a asserts the three `config:` lines verbatim, and that there are three.
- **Q8** (a tap the calibrator no longer accepts logged as "no button there"): unreachable on the rig until now — the
  view and the touch read share a tick and nothing moves between them, yet on a brain `pushReady()` reads the motors
  LIVE, and the robot can move between the paint and the read. **Fixed (rig + test):** the program's screen is now
  `Rig::seam` (`SeamScreen`: forwards every call to the FakeBrainScreen, whose log the tests read, and calls
  `onTouchRead` before each read) and `Rig::jostle(port, rad/s)`. **T1r**: at the first DONE tap's read a drive member
  turns, so `touch: ... release=1 on DONE -> ignored -- the calibrator no longer accepts it`; the next tap
  `release=2 ... -> ANSWER DONE`; one answer in all; saved.
- **Q14** (the card line always names the backup): **T2d** asserts `card       : shulib_drivetrain.bin holds
  MEASURED tank ... -- this run's FRONT is checked against it` verbatim (T2i keeps the backup's).
- **T2b's weak amber check:** it now also asserts the header WORD's own print, `print M 8 4 pen=101418
  eraser=e8b23a 'SAVED -- NEW FRONT'` (a header word is printed on its own band's colour), and that it is never
  printed on green.

**13. The review-fix round was not closed in this log (docs, major — REAL).** Closed below ("The first fix round's
close"), with this round's own close after it.

**14. Seven places said A2a LANDED 2026-10-22 (docs, minor — REAL).** Re-dated in the form MECH-A1's line uses:
`LANDED 2026-10-29 (built 2026-10-18–22, reviewed and fixed 2026-10-22–29)` — `build-order.md` :632 and :1622, the
roadmap's "Now" (:382), `REMAINING-WORK.md` :87, the changelog's heading (:19), HANDOFF's shell line (:269); the
briefing's "Next up" regenerates from :632. **For the coordinator: if the merge is NOT on 2026-10-29, re-date those
six and regenerate the briefing.** First-round fix #13's "add 'Part A2a landed 2026-10-22'" is superseded by this.

**15. `run()`'s doc comment named only the disable (docs, minor — REAL).** Now: "Run calibrations until driver
control ends — the field's disable, or autonomous starting — or return at once when no run can start (a refusal,
painted and printed) (header)", matching the class comment and the shell's `runCalibrate()`; `docs/api/`
regenerated.

**16. The changelog's ADDED list left out public additions (docs, minor — REAL).** Added: the shell's
`hal::pros::calibrateStateScreen(state, why)`; `CalibrateProgram::kPushLogMinSeconds` / `kPushLogKeepaliveSeconds`;
all 23 new `calibration::panel_text` literals by name (the four step headers, the two held headers and
`kHeaderSavedNew`, `kPushFrontFirst`, `kNothingSavedYet`, `kNextTouchDoneClear`, `kFrontRule`, `kFlip*` ×4, `kMech*`
×3, `kLabelNew`, `kLabelSave`, `kNoteFirst`, `kNoteNotChecked`, `kWarningCut` — the list `git diff ceb17c0` of the
panel prints); and this round's behaviours (the FIRST RECORD note only over a save that succeeded; the stack, stated).
`detail::programPushLogDue` is internal (`detail::` is not in `docs/api/`, checked) and is not listed.

**17. HANDOFF's Calibration inventory left out R5a Part A's four headers (docs, minor — REAL, older than A2a, fixed
in passing).** HANDOFF :274-278 now ends "and (R5a Part A) `drive_characterization.hpp`, `drive_characterizer.hpp`,
`feedforward_fit.hpp` and `rest_capture.hpp` (the GAINS and REST measurements; Calibrate runs them in R5a Part C)" —
each header's own first lines read to confirm the words.

**18. The shell's API page sent a team to an internal brief (docs, minor — REAL).** `hal/pros/calibrate.hpp` now says
"(21 lines; CAL1's next part adds the guide chapter that shows it whole)" — no brief named; regenerated into
`docs/api/pros-calibrate.md` (grep: `the CAL1 brief §1.1` appears in no living document). **Hand-on to A2b:** when
the chapter lands, name it in that sentence and regenerate `docs/api/`.

**Not raised, closed anyway.** The tests reviewer's own run shows two more survivors it did not raise —
**Q11** (after SAVE / USE NEW the new question's view NOT put up at once) and **Q12** (after an answer the new
step's view NOT put up at once): GREEN for them. Both are documented behaviour ("the new view goes up NOW, not on the
next tick"), and the rig's marks carry exact times: T1a now asserts the `3/4 PULSES` header is drawn at the DONE
tap's own instant, T5f that the MECHANISM PORT TURNED header is drawn at the USE NEW tap's own instant (both times
the same `steppedUs × 1e-6` double, so `==` is exact; a tick later is +0.01). Q11 and Q12 RED.

### Mutation pass 4 — the WHOLE list again, on the files as they stand after this round's fixes (2026-10-29 13:11–13:26)

**Method:** pass 3's runner made to drive one of FOUR independent copies (`fix2/mut4/run_mut4.py`; list `pass4.json`;
outputs `shard1..4.out`; backups `bak/tree1..4/`, never /tmp). Each copy is a shared-object clone of the repository
checked out at `ceb17c0` with the worktree's files rsynced over it (`git describe`: the same
`v0.1.1-387-gceb17c0-dirty`) and its own `cmake` build tree — so no copy can touch the worktree or another copy. Before
the pass: `md5sum -c pre_md5_pass4.txt` (the nine A2a code, test and shim files) OK in every copy, and each copy's
unmutated suite `2551 | 2551 passed`. Each mutant on a FRESH link, every object whose depfile names the mutated file
rebuilt after the edit (checked per mutant), the whole suite capped (RLIMIT_AS 4 GiB, 600 s), the file restored in
`finally` (`cmp`: identical, 105 of 105 restores), its timestamp bumped; after the pass, the nine files `md5sum -c` OK
in all four copies. **A false start, logged:** the first launch (13:10) gave 104 × "build did not link — NOT A VALID
RUN" in seconds — a copy whose first build ran only `shulib_tests/fast` never runs CMake's `depend` step, so its
`compiler_depend.make` was empty and no header edit rebuilt anything; the runner's own guard refused every verdict
(none was counted), each copy's `depend` step was then run (`compiler_depend.make` 63,069 lines), a header touch shown
to rebuild, and the pass re-launched (`*.invalid-nodeps.*` kept). **104 mutations:** pass 3's 77 (V11 re-cut onto
`programPushLogDue`; M19 also renames the one test reference to `detail::`, so its verdict stays the TOOL's), the
tests reviewer's 17 (Q1–Q3, Q6–Q14, S1–S3 verbatim; Q4/Q5 re-cut onto the new function), the correctness reviewer's
four survivors (R1, R7, R19, R54, recovered from their probe headers and the findings), and six of this round's own
(Z1–Z6). The shell's adapter test was edited once AFTER the copies were synced — its header COMMENT only (PC1's
description); every verdict on it stands, and the final suite below runs the final file.

| # | The mutant | Verdict | First failing line (cases) |
|---|---|---|---|
| M4 | USE NEW saves the measurement TURNED to the card's front (sides swapped, signs negated: the … | RED | :894 (T2b, T2d, T5a, T5e, T5f, T5m, … 7 in all) |
| M5 | the front check skipped: FLIPPED never holds (save at once) | RED | :852 (T2a, T2b, T2c, T2k, T2j, T5f) |
| M6 | RUN AGAIN from a held screen saves the held run first | RED | :846 (T2a, T5b, T5c, T5j, T5l, T5n, … 7 in all) |
| M7 | the check reads the record FILE only (a backup's record treated as none) | RED | :1079 (T2i) |
| M8 | the no-card branch prints the FIRST words | RED | :455 (T1e, T5d) |
| M9 | the panel told Saved on an Unconfirmed report (B1 mutation 1) | RED | :1234 (T4) |
| M10 | BootReadsBackup::Yes passed (B1 mutation 2) | RED | :1230 (T4, T4b) |
| M11 | a direct write of the record beside the safe save (B1 mutation 3: saveAndReadBack's writer) | RED | :1211 (T4, T4b, T4c) |
| M12 | the watch COMMANDS a declared port (asks the provider for it, setVoltage(0)) | RED | :147 (T1a, T5a, T5b, T5f, T5h, T5j, … 12 in all) |
| M13 | the watch's threshold made 0 | RED | :233 (T1a, T1b, T1c, T1d, T1e, T1h, … 40 in all) |
| M14 | a declared port built as a candidate (the drop removed) | RED | :250 (T1a, T1d, T5a, T5b, T5e, T5f, … 17 in all) |
| M15 | a refusal moved after the first devices.motor(): the drive kind checked after the motors are… | RED | :2091 (T6b, PC6) |
| M18 | the watch's read constructs pros::Motor{port, gearset} (a motor object that WRITES the gearing) | RED | adapter :213 (PC1) |
| M19 | the program's internals in `program_detail`, not `detail` (the reference publishes them) | RED (tool) | `check-coverage`: `ProgramRun`'s members undocumented |
| M20 | viewFor()'s rule default flipped to ByPush | RED | calibrate_panel_additions_test.cpp:254 (N1, N2, P1, P2, P3, P4) |
| M21 | the auto-answer removed (WHICH END left to the operator, who is never shown it) | RED | :233 (T1a, T1b, T1c, T1d, T1e, T1g, … 50 in all) |
| M22 | TouchGate::touch reads the held screen's left box as Answer::FrontFirst | RED | calibrate_panel_additions_test.cpp:370 (N4, N5, N5b, T2b, T2d, T5a, … 14 in all) |
| M23 | the MECHANISM PORT TURNED hold removed (warn only) | RED | :1350 (T5a, T5b, T5c, T5d, T5e, T5g, … 11 in all) |
| M24 | USE NEW saves without asking the pending MECHANISM PORT TURNED | RED | :1511 (T5f) |
| M25 | the 'did not turn' DONE line dropped | RED | :1422 (T5b, T5b-1, T6d) |
| M26 | factsProblem() skips a NAMED port-0 entry (runtime) | RED — at compile time | T6e's own `static_assert` (`calibrate_program_test.cpp:2468 static_assert`) |
| M27 | the not-Tank check removed | RED | :2091 (T6b) |
| M43 | the discard words ignore the card's contents: always 'the card keeps its record' | RED | :1438 (T5c, T5o) |
| M45 | the FIRST decided by the record FILE alone (a backup alone is 'no record') | RED | :1022 (T2f, T5o) |
| M46 | the per-file lines skip the backup | RED | :1008 (T2e, T2f) |
| M47 | an EMPTY file printed with the decoder's (empty) reason | RED | :1042 (T2g) |
| M48 | no card: the discard words ignore it | RED | :1460 (T5d) |
| M49 | no card: the `card :` line says there is no usable record | RED | :452 (T1e, PC6) |
| X1 | TouchGate's arming clock NOT restarted when the hold's question changes | RED | calibrate_panel_additions_test.cpp:404 (N5, N5b, T5f, T5m) |
| X2 | recordLayout() prints no `screen:` line when the held question changes | RED | :1511 (T5f, T5l, T5m, T5n) |
| X3 | the watch's 'turned' strict (>), not the calibrator's >= (the EXACT edge) | RED | :1568 (T5g) |
| X4 | the watch's NaN guard dropped | RED | :1607 (T5h) |
| X5 | the facts checked AFTER the drop (the stopped run's order) | RED | :2136 (T6a) |
| X6 | a REFUSED push judged by the watch (the draft's form) | RED | :440 (T1d, T5j) |
| X7 | the NOTE drawn over the warning's second row (no ' ...' cut) | RED | calibrate_panel_additions_test.cpp:500 (N6, T1c) |
| X8 | the stated cartridge byte not checked (reaches the calibrator's precondition) | RED | :2141 (T6b) |
| X9 | the shell's line() back to printf + fflush (the brief's pre-SER1 form) | RED | adapter :141 (PC2) |
| X10 | the build time in a function-static buffer shared by every program (the draft's) | RED | :582 (T1k) |
| X11 | the competition bits disagree with PROS's (kCompetitionAutonomous 0x08) | RED — at compile time | the shell's guard (`hal/pros/calibrate.hpp` static_assert) |
| X12 | a third card READ before the save (an extra loadRecord) | RED | :1211 (T4) |
| X13 | a held run's RUN AGAIN keeps the held state (held_ not cleared) | RED | :988 (T2j) |
| X14 | the watch judges a port whose BASELINE read failed (against a baseline of 0) | RED | :1745 (T5k) |
| X15 | the watch judges the claw's READING, not its travel since the push began | RED | :1400 (T5b, T5k, T5p, T5l) |
| V1 | MECHANISM PORT TURNED says the card's record DRIVES with the turned port whenever the card h… | RED | :1523 (T5f, T5l) |
| V3 | the loop no longer exits when AUTONOMOUS starts (disable only) | RED | :641 (T1m, T1n, T2k) |
| V4 | the stated-vs-registry IMU port WARNING never printed | RED | :2369 (T6f) |
| V5 | an IMU the provider refuses is painted as MOTORS REFUSED | RED | :2342 (T6c-2) |
| V6 | a registry MOTOR the provider refuses is dropped SILENTLY (no WARNING naming it) | RED | :2259 (T6c-2) |
| V7 | after construction ONE motor is enough to start (n < 1) | RED | :2240 (T6c-2) |
| V8 | the 'declare it' line after PULSE_STILL only, never after PULSE_ROLLED | RED | :2075 (T5b-3) |
| V9 | TOO FEW MOTORS below THREE (a two-motor tank refused) | RED | :2232 (T6c) |
| V10 | the drop forgets what it found: every declared port ALSO printed 'the census found NO MOTOR … | RED | :253 (T1a) |
| V11 | no 'push live:' serial line ever | RED | :276 (T1a) |
| V12 | TWO IMUs accepted (the refusal only for none) | RED | :2091 (T6c-2) |
| V13 | RUN AGAIN no longer invalidates the painter | **GREEN — equivalent** (as pass 3) |  |
| V14 | RUN AGAIN no longer stops every motor before the new calibrator | **GREEN — equivalent** (as pass 3) |  |
| V15 | the declared starting state (0 V + Coast before the calibrator) removed | RED | :615 (T1l) |
| V16 | calibrateStateScreen paints the reason where the state goes and the state where the reason goes | RED | adapter :348 (PC6) |
| V20 | the watch judges SIGNED travel (a claw turned backwards by the push is not 'turned') | RED | :1510 (T5f) |
| V26 | the loop exits only on a status EQUAL to 'disabled' (a field-connected disable 0x05 runs on) | RED | :641 (T1m, T1o) |
| V30 | a held run's save (USE NEW / SAVE) writes the measurement with its IMU port changed (2 -> 3) | RED | :184 (T2b, T5a, T5e, T5m) |
| V31 | a held run's save writes the measurement with its provenance overwritten | RED | :186 (T2b, T5a, T5e, T5m) |
| R4 | files that OPENED and do not decode count as 'the card keeps its record' | RED | :1997 (T5o) |
| Y1 | SAVE on the first MECHANISM PORT TURNED question saves at once (the other turned ports never… | RED | :1854 (T5l, T5m, T5n) |
| Y2 | the watch keeps only the FIRST declared port that turned (one question for all) | RED | :1843 (T5l, T5m, T5n) |
| Y3 | the watch keeps only the LAST declared port that turned (the reviewers' R1/V2, on the new code) | RED | :1843 (T5l, T5m, T5n) |
| Y4 | each question shows the FIRST question's 'the card's record DRIVES' verdict | RED | :1855 (T5l, T5m) |
| Y5 | each question names the FIRST question's port | RED | :1854 (T5l, T5m, T5n) |
| Y6 | TouchGate ignores holdPort: a new port's question is NOT re-greyed | RED | calibrate_panel_additions_test.cpp:444 (N5b, T5m) |
| Y7 | recordLayout() ignores holdPort: no `screen:` line for a new port's question | RED | :1857 (T5l, T5m, T5n) |
| Y8 | after SAVE on several questions the NOTE names one port | RED | :1923 (T5m, T5n) |
| Y9 | the count line before several questions dropped | RED | :1843 (T5l, T5n) |
| Y10 | the '(k of n)' marker dropped: every question worded as if it were the only one | RED | :1843 (T5l, T5n) |
| Y11 | the end-of-driver-control words always say 'the field disabled the robot' | RED | :668 (T1n, T2k) |
| Y12 | the PROGRAM FAULT idle exits on a disable only (the pre-review form) | RED | :680 (T1n) |
| Y15 | a later question shows the first port's travel slot (its own turn lost) | RED | :1854 (T5l, T5m, T5n) |
| Y16 | the per-question 'on the card' judged against the FIRST port's side | RED | :1855 (T5l, T5m) |
| Q1 | PROGRAM FAULT: the catch block's own stopAll removed (motors left as they were through the w… | RED | :775 (T1p) |
| Q2 | touch baseline: ONE try instead of 20 (no retry after an error read) | RED | :567 (T1j-2) |
| Q3 | RUN AGAIN: the new gate based on the program's START baseline, not the release that asked fo… | RED | :379 (T1b) |
| Q6 | loop stats: worst gap never measured | RED | :270 (T1a, T1q) |
| Q7 | config line: movedThreshold and straightBound swapped | RED | :261 (T1a) |
| Q8 | touch log: a tap on a live button the calibrator no longer accepts logged as 'no button there' | RED | :812 (T1r) |
| Q9 | RUN AGAIN: the per-run record keeps run number 1 (RUN 2 ENDED printed as RUN 1) | RED | :375 (T1b) |
| Q10 | the watch's baseline taken when the IMU step begins, not when the push begins | RED | :392 (T1b, T1d, T5b, T5c, T5d, T5j, … 10 in all) |
| Q11 | SaveHeld: the new question's view NOT put up at once (next tick instead) | RED | :1541 (T5f) |
| Q12 | Answer: the new step's view NOT put up at once (next tick instead) | RED | :342 (T1a) |
| S1 | shell: the isfinite(raw) guard removed | RED | adapter :227 (PC1) |
| S2 | shell: the counts-with-unknown-gearing guard removed (degrees = raw*360/0) | RED | adapter :233 (PC1) |
| S3 | shell: an unknown encoder unit answers the raw reading as degrees | RED | adapter :238 (PC1) |
| Q13 | the watch ignores a FAILED read at DONE (judges against a stale 0.0) | RED | :1762 (T5k) |
| Q14 | the start's card line always names the BACKUP as the record's file | RED | :962 (T2d) |
| Q4 | push live keepalive >= made > (re-cut onto programPushLogDue) | RED | :829 (T1s) |
| Q5 | push live min interval >= made > (re-cut onto programPushLogDue) | RED | :827 (T1s) |
| R1 | the watch's baseline taken at the IMU wait (armed on WaitImu, not re-armed at the push) | RED | :1781 (T5p) |
| R7 | driver control ends on ANY status bit (the connected bit too) | RED | :703 (T1o) |
| R19 | the port list not compacted after a refused motor | RED | :2323 (T6c-2) |
| R54 | the RAII all-stop guard's destructor emptied | RED | :2306 (T6c-2) |
| Z1 | the touch line always 'SAVE AS PUSHED' (the next-question words never said) | RED | :1510 (T5f, T5l, T5m) |
| Z2 | USE NEW with a MECHANISM question pending read as saving | RED | :1510 (T5f) |
| Z3 | the LAST MECHANISM SAVE read as asking a next question (< made <=) | RED | :1357 (T5a, T5f, T5m) |
| Z4 | the FIRST RECORD note kept over a failed save | RED | :1314 (T4c) |
| Z5 | the FIRST RECORD note dropped over a save that SUCCEEDED | RED | :347 (T1a, T1b, T1c, T4c) |
| Z6 | the push-live intervals passed swapped (keepalive as the minimum) | RED | :279 (T1a) |

**102 of 104 RED** — every one the reviewers saw survive (R1, R7, R19, R54, Q1–Q9, Q11–Q14, S1–S3) by a test written
for it this round, and this round's own Z1–Z6. Line numbers are `calibrate_program_test.cpp`'s unless named. **M26**
is red at compile time by the TEST's own assertion (T6e's `static_assert` at :2468 — not the mutation's breakage);
**X11** by the shell's own guard, the guard it exists to test (a header guard, not a test); **M19** by the tool
(`check-coverage`: `ProgramRun`'s members undocumented once they leave `detail`). **V13 and V14 GREEN — equivalent**,
as in pass 3 (V14: the new calibrator's `start()` runs `safeState()` before any tick; V13: the painter redraws only
what changed and a correct diff draws the same final screen). Pass 3 (above) stays as history: its verdicts were on
the files before this round.

### The first fix round's close (its gates, as they ran 2026-10-22 18:55–19:15; recorded 2026-10-29)

That round (above, "CAL1-A2a review findings and their disposition", 2026-10-22) ended with its mutation pass 3 and
was stopped during its gates; its record was never closed (finding 13). What its gate outputs under
`scratch/cal1-a2a/fix/gates/` show, read 2026-10-29:
- **Host build from clean** (`cmake.log`, `clean_build.log`, 18:55–18:56) and the doc-gate step (`build2.log`:
  `doc staleness audit: clean (29 live docs …)`, both targets built).
- **The suite** (`final_suite.out`, capped, 18:56): `2543 | 2543 passed | 0 failed | 3 skipped`, `2929659`
  assertions — 2,477 at the A2a base, 2,531 at A2a.9, +12 that round.
- **The ARM header gate** (`arm_ci.out`, 18:59): exit 0 over 225 headers, 0 output lines; its control fails
  (`float-conversion`); the CI greps both hold.
- **api_doc_tool** (19:01): self-test OK; coverage `4843 public entities across 185 headers, all documented`;
  check-fresh empty (exit 0); examples `541 quoted lines, all verbatim`; removability clean.
- **Compile refusals** (`refusal.out`, 19:01): `100% tests passed, 0 tests failed out of 200`.
- **The team file as a PROS project** (`team_proj3`, 19:02): `hot.package.bin` 81,432 bytes.
- **`src_build_gate.py self-test`** (19:15): `OK (235 detector cases, each against a real build)`, exit 0.
- **`src_build_gate.py check`: NEVER COMPLETED** — its log is 0 bytes, killed with the session. The panel header that
  round changed IS in today's src Calibrate's includes, so this was the one gate with no evidence; THIS round's
  `check` (below) runs on a panel header byte-identical to that round's (`calibrate_panel.hpp` md5 `8348f3fe…`, the
  same in `fix/mut3/pre_md5_pass3b.txt` and now) and is the evidence for both.
- **Its prose review was never logged.** Done now, for both rounds' terms together (below).
- **Where it stopped:** after the src gate's self-test, 2026-10-22 ~19:15; nothing ran after it until this
  resumption, and the worktree was exactly pass 3b's files (checked at the top of this section).

### The gates — every one re-run on THIS round's final tree (2026-10-29 12:58–13:32; outputs `fix2/gates/`)

- **Host build** (`cmake -S test -B build/test`, `cmake --build build/test`, the doc-gate step included;
  `build_final.log`): exit 0, no warning. The briefing regenerated before the build (no binary) and again after the
  suite binary existed — each time capped (`ulimit -v 4194304; timeout 900`), the binary always linked from this
  round's sources (removed first at 12:52 and again before the final link).
- **The suite** (`final_suite.out`/`.xml`, capped): **`2551 | 2551 passed | 0 failed | 3 skipped`, `2929908`
  assertions**, 22.0 s, 62.7 MB peak. Against the first round's 2,543 (`fix/gates/final_suite.xml`, compared case by
  case, `cmp_xml.py`): **0 gone, 8 new** (T1j-2, T1o, T1p, T1q, T1r, T1s, T4c, T5p), 0 failing; the assertion counts
  that changed are the edited cases (T1a, T1b, T2b, T2d, T5f, T5k, T5l, T5m, T6c-2, PC1) and one source scan
  (`drive_program_source_test.cpp` P3-T3, 13,583 → 13,585: it counts per string literal under `include/shulib`, and
  this round added two). Against the A2a base: 2,477 → 2,551, **74 new, every pre-existing case passing, none edited**
  (A2a.9's record for 2,477 → 2,531; the first round's and this round's edits are to A2a's own files only).
  **Through main's bounded runner** (a scratch COPY of `tools/suite_run.py` — this worktree predates it — pointed at
  this worktree's binary, `fix2/suiterun/`): `suite run: green — 2551 cases, 2929908 assertions`, exit 0.
- **`src_build_gate.py self-test`** (started 13:10:37): `src build gate self-test: OK (235 detector cases, each against a
  real build)`, exit 0. **`check`** (to 13:29:49, `srcgate_check.log`): **`src build gate: PASS (all 7 builds link;
  …)`**, exit 0 — bench, xdrive, tank, tank-drive, tank-calibrate, tank-autontest, tank-competition each `exit 0, 11/11
  TUs compiled, warnings: 8 vendor-ignored, 0 disallowed, hash asserted`, and the stdio object check. **No plant left:**
  `git status --porcelain` identical before and after; of the 24 changed files' checksums only the three I edited
  during the run differ (the changelog's contributor note, the regenerated briefing, PC1's header comment) — no
  header under `include/` was edited while it ran. `git diff --stat ceb17c0 -- src/ Makefile tools/`: EMPTY.
- **The ARM header gate** (`arm_and_ci.sh`, its TU under `fix2/gates/`): exit 0 over **225** headers, 0 output lines;
  the control (`int x = d;`) fails with `-Werror=float-conversion`. **The CI greps:** `library is PROS-free (all of
  include/shulib except hal/pros/)`; `layering holds: core is sim-free`.
- **The compile-refusal harness** (`ctest -R compile_refusal`): `100% tests passed, 0 tests failed out of 200`.
- **api_doc_tool:** self-test OK; `doc coverage: 4843 public entities across 185 headers, all documented`;
  check-fresh exit 0 (`docs/api/` regenerated after every `///` change: `run()`'s comment, the shell's header);
  `doc example scan: 541 quoted lines, all verbatim (7 source files)`; `removability: no public doc references
  docs/internal/`. **`doc_staleness_audit.py`:** `clean (29 live docs, 0 numeric claims checked)`.
  **`briefing_status.py check`:** `generated block current` (suite 2,551 / 2,929,908; 225 headers).
- **The team file as a PROS project** (`fix2/team_proj4`: `git archive ceb17c0`, the three A2a headers as they stand
  overlaid, `cmp` identical, the 21-line team file): `make` exit 0, `bin/hot.package.bin` **81,604 bytes** (text 79.69
  KB), ONE warning (liblvgl's own); its symbols carry `motor_get_position`, `motor_get_encoder_units`,
  `motor_get_gearing`, `competition_get_status` and the program's own functions. The STACK measurement (finding 6)
  re-run on these final headers: 22,464 B, unchanged.

### Prose review (pace rule 4) — BOTH fix rounds' terms, grepped over `docs/` and `README.md` (chunk logs aside), every hit in a living document READ (2026-10-29 13:20–13:32)

- **The first round's terms:** `holdPort` (the changelog's panel paragraph, true; `docs/api/` generated);
  `MECHANISM PORTS TURNED` and `(k of n)` (no living doc — serial words, in the tests); `PORTS KEPT AS MECHANISMS` (the
  changelog, true); `autonomous started` and `programDriverControlEnded` (none; the changelog's "the field's disable,
  or the start of autonomous … its words naming which" is true); `arming` (the changelog's re-greyed boxes, true).
- **This round's terms:** `SAVE AS PUSHED` and `next question follows` — no living doc quotes the touch line (none to
  fix); `FIRST RECORD` — the changelog (now says the note needs a save that succeeded), build-order :1640 (robot two's
  empty card gets the note — still true), `docs/api/` (generated); `kPushLog*`, `calibrateStateScreen`, `22,464` — the
  changelog and generated pages, true; `TASK_STACK_DEPTH` — guide 11 and `pros-char_sink.md` name the MIN depth of
  another task (unrelated, true); `programPushLogDue` and the rig's new options — no doc (test and `detail::`);
  the shim's three knobs — the changelog's new contributor note; `the CAL1 brief §1.1` — gone from every living
  doc; `LANDED 2026-10-22` / `Now (2026-10-22)` — gone (finding 14); `drive_characterizer` — HANDOFF now names it
  with the other three, and the hits elsewhere (roadmap, build-order, the register, `docs/api/`) were already true.
- **The touch-line and FIRST-note changes against A2b's goldens:** grepped `brief-CAL1/d6a/v3/run_new/` — six
  `SAVE AS PUSHED` lines, all touches that save at once (unchanged words); no golden pairs NOT SAVED with FIRST
  RECORD. Recorded for A2b in the hand-on below.

### This round's close (2026-10-29 13:32 EDT, `date`) — A2's DoD as the split leaves it to A2a, re-ticked

- [x] §1.2 items 6-7 and the ONE changed library header (`calibrate_panel.hpp`, additions only — unchanged this round,
  md5 `8348f3fe…` as in pass 3), every public member documented (coverage, above); `git diff --stat ceb17c0 -- src/`
  EMPTY.
- [x] Tests 1, 4, 5, 5b, 6, 8b and 9 green, and test 2's untyped half; the suite 2,477 → 2,551 (above), every
  pre-existing case passing, none edited.
- [x] Mutations: **pass 4** (above) — 102 of 104 RED on fresh binaries, the two GREEN equivalent and reasoned; every
  survivor any reviewer named is RED. Pass 2 (A2a.7) and pass 3 are history. 28–42, 44 and 50–54 are typed (A2b's);
  16 and 17 are A1's.
- [x] The same gates as A1, and `src_build_gate.py check` COMPLETED on this tree (above); the team file links as a
  PROS project.
- [~] HA-187 — A2b's to register; this round found NO new hardware belief: finding 7 widens HA-187 (2)'s blast
  radius (handed on below), and the stack figures are GCC's and PROS's source, not a belief about the brain.
- [~] The 34 golden scenarios, `typed-card-wins`, `typed-nocard` — A2b's. Every new screen string of A2a is inside
  `kAll`'s compile-time check; this round added no screen string (the touch line is serial; the FIRST note is
  dropped, not changed).

**Hand-ons to A2b (added this round):** (1) HA-187 (2)'s blast radius gains the MEDIUM evidence line's degrees under a
reported gearing that differs from the insert (finding 7) — Station G reads an unconfigured claw's
`motor_get_gearing` before any program gears it; (2) when the guide chapter lands, name it in
`hal/pros/calibrate.hpp`'s "21 lines" sentence and regenerate `docs/api/` (finding 18); (3) the golden harness will
see the touch line's new words only in scenarios with two held questions in one run (none of the brief's 34 has
one) — no golden changes; (4) A2b EXTENDS the changelog entry (its typed literals by name, as finding 16 did for
A2a's).

**For the coordinator (refreshed):**
1. **Shared files this part touches (only its own lines, as they stand now):** `docs/changelog.md` :19–132 (one
   entry), `mkdocs.yml` :287 and :313, `docs/roadmap.md` :382–394, `docs/internal/build-order.md` :632 (the `Next:`
   block's CAL1 clause), :1622–1632 (CAL1's status), :1650, `HANDOFF-remaining-work.md` :264, :268–271 and :274–278,
   `REMAINING-WORK.md` :87, `ORIENTATION.md` :94, `README.md` :97–98 and :279–280, `PROJECT-BRIEFING.md` (generated
   :110, :113, :115; hand-written :193–197 and :746–749), `docs/api/README.md` and `all-entities.md` (generated), and
   NEW this round **`test/pros_shim/pros/motors.h` and `motors.hpp`** (the three dropout knobs: additions with 0
   defaults, the C reads' header comment).
2. **Main moved further than the first round said:** besides `71b4764`, `c09e022`, `43ac361` and `73044ca`, main now
   has **`46c874c` (MECH-A4 — API 2.5)** and **`ec3754f`** (the ordering ruling of 2026-10-29). Neither touches A2a's
   code or test files (`git diff --stat ceb17c0 shulib-v2` over them: nothing); both touch shared docs. **The
   changelog's placement changes:** main's changelog now opens with `## API 2.5`, so A2a's entry belongs at the top
   of `## API 2.5` as **"Added since 2.5 (…)"** — not under 2.4 (this worktree, on `ceb17c0`, still says 2.4; A2a is
   additive, no bump). Regenerate `docs/api/` and the briefing (with MAIN's `briefing_status.py`, after the merged
   build) — the worktree's briefing says "next free is HA-192" and its own chunk count, both stale against main
   (HA-216).
3. **The landing date (finding 14):** six lines say `2026-10-29` — `build-order.md` :632 and :1622, `roadmap.md`
   :382, `REMAINING-WORK.md` :87, `changelog.md` :19, `HANDOFF-remaining-work.md` :269. If the merge is on another
   day, re-date those six and regenerate the briefing (its "Next up" copies :632). The first round's fix #13 ("add
   'Part A2a landed 2026-10-22'") is superseded.
4. **The brief's Re-check note** proposed by the first round (one question per turned port) still waits for you;
   this round proposes none.
5. **Nothing committed, nothing pushed, no `pros`, no serial port.** Scratch: `fix2/` (gates, probes, mut4 with its
   four copies under `mut4/tree1..4` — deletable after the merge, 1.1 GB with their build trees).

**Where work stopped:** this round is complete at 13:32 EDT 2026-10-29 — every finding dispositioned, pass 4 run,
every gate green on the final tree. The worktree is uncommitted (16 modified tracked files — the two shim headers new this round — and 8 new); the next step is
the coordinator's review and merge.

## Verification-round findings and their disposition — the third verification round (2026-10-29 14:12 EDT, `date`)

- **Started** 2026-10-29 14:12 EDT (`date`), on the worktree exactly as the second review round closed it (13:32).
  Executor: **Claude Opus 5.5 (1M context)**. This heading is written BEFORE any edit. A fresh verifier returned ONE
  finding (minor); it is re-checked below, then fixed and every gate re-run.

### The finding, re-checked, and what was done

**V3-1. Q4/Q5 held only on the new function: the PROGRAM's own push-live edges were unpinned (minor — REAL).**
- **Re-checked, reproduced.** On the second round's final tree (pass 4's copy `fix2/mut4/tree1`, rsynced from the
  worktree, 0 files differing), the verifier's two call-site mutants, each on a fresh link with both dependent
  objects rebuilt, the suite capped (RLIMIT_AS 4 GiB, 600 s), restored `cmp`-identical (`fix3/mut/repro_before.out`):
  **W13d** (the call replaced by the pre-round inline test, both edges `>`) **GREEN**, `2551 | 2551 passed`;
  **W13e** (`std::nextafter(now - run.lastPushLogAt, 0.0)` passed) **GREEN**, the same. Finding 12's Q4/Q5 bullet
  claimed more than its tests held: T1a's five lines pin the two interval VALUES (Z6), not the inclusive edges, and
  not that `record()` calls `programPushLogDue` at all.
- **Why no test could see it** (a scratch probe, `fix3/probes/pushlive.cpp`, run capped; outputs `out_*.txt`): the
  program reads the plant's `FakeClock`, which ADDS 0.01 every step, so its instants sit a hair off the decimals —
  T1a's first push line is at `2.0000000000000013` s and the count-change line 26 ticks later at `2.2599999999999958`
  (the 25th tick's gap was a hair UNDER 0.25 s), and the keepalive 501 ticks after `3.7399999999999642` at
  `8.7499999999998579` (the 500th tick's gap was a hair UNDER 5 s). No gap lands on an edge, so `>` and `>=` print
  the same lines.
- **Fixed at the root — the program's own lines now land on both edges exactly (no library change).** The rig
  gains, as additions with defaults that leave every earlier scenario byte-identical (`test/calibrate_program_rig.hpp`):
  `Scenario::exactClockOffsetUs` — the PROGRAM's clock read the brain's way, ProsClock's `micros() x 1e-6` over the
  rig's steps (`ExactClock`, handed to the program through `CountingProvider::clockOverride`; the plant, the IMU's
  warm-up and the operator keep the plant's clock). It is exact at every quarter second: `double(1e-6)` is within
  4.53e-23 of 1e-6 (relative 4.5e-17, under the 2^-54 = 5.55e-17 relative half-ulp of any double), so `n x 1e-6`
  rounds to `n/1e6` whenever that is a double — and the gap between two exact instants is exact.
  `Scenario::pushWaitSec` — the operator waits on the PUSH screen, hands off, before pushing. `FakeIo::lineAt` — the
  program clock's instant of every serial line. **T1t** (`calibrate_program_test.cpp`): on the exact clock, hands off
  for 5 s — the push's first line at exactly **2.0** s (REQUIRED: the premise every instant stands on); nothing
  changed, the keepalive at exactly **7.0** s (`0 of 10 moved`), not a tick later; the panel first shows `10 of 10
  turned` at 7.03 s (REQUIRED inside (7.0, 7.25): the count changed INSIDE the minimum) and its line waits for
  exactly **7.25** s, not a tick before or after; then far enough, a keepalive and READY verbatim, `RUN 1 ENDED:
  DONE`, the card's record equal to the MEASURED line, every motor stopped. The instants were read off the probe
  first (`out_0_5.txt`), never computed by the code under test in the test. Of the verifier's two options this is
  the stronger: it pins the program's BEHAVIOUR at both edges, so any implementation with an exclusive edge goes red
  — inline, through the function, or through a nudged argument — where a source pin would hold only this one
  spelling of the call.
- **Found while re-checking (a test comment, REAL): T1a called the `travel 6.3 rad` line "a keepalive".** The probe
  shows it 1.48 s after the line before — it is the push reaching FAR ENOUGH (one wheel turn, 6.28 rad): a CHANGE.
  The comment now says so (T1a's assertions unchanged).
- **The header's claim stands as written:** `programPushLogDue`'s "BOTH edges inclusive (exactly the interval is
  due)" is now true of the program's lines and tested there (T1t), as well as on the function (T1s). No `///` comment
  changed, so `docs/api/` is untouched (check-fresh, below).

### Mutation pass 5 — the push-live edges at the program, on fresh binaries (2026-10-29 14:18–14:23)

Pass 4's runner, copied (`fix3/mut/run_mut5.py`; list `pass5.json`; output `pass5.out`; backups
`fix3/mut/bak/tree1/`, never /tmp), on pass 4's copy `tree1` rsynced from the worktree as it now stands (0 files
differing; `md5sum` of the three files recorded first, `pre_md5_pass5.txt`). Its unmutated suite first:
`2552 | 2552 passed`. Each mutant on a FRESH link with every object whose depfile names the mutated file rebuilt after
the edit (checked by the runner), the whole suite capped (RLIMIT_AS 4 GiB, 600 s), the file restored in `finally`
(`cmp`: identical, 10 of 10), its timestamp bumped; afterwards `md5sum -c` OK and each file `cmp`-identical to the
worktree's.

| Id | The mutation | Verdict | Caught at (`calibrate_program_test.cpp`) |
|---|---|---|---|
| W13d | (verifier) the call replaced by the pre-round inline test, BOTH edges `>` | RED | :881 (T1t: the keepalive at 7.01, not 7.0), :887–888 |
| W13e | (verifier) the call passes `std::nextafter(now - run.lastPushLogAt, 0.0)` | RED | :881, :887–888 (T1t) |
| W13f | Q4 at the PROGRAM: the call inline, the keepalive alone `>` | RED | :881, :888 (T1t) |
| W13g | Q5 at the PROGRAM: the call inline, the minimum alone `>` | RED | :888 (T1t: the change line at 7.26, not 7.25) |
| W13ctl | CONTROL: the call inline, both edges `>=` — a correct program | **GREEN, as it must be** | — (2552 / 2552: T1t pins behaviour, not the call's spelling) |
| Q4 | the keepalive `>` inside `programPushLogDue` | RED | :831 (T1s), :881, :888 (T1t) |
| Q5 | the minimum `>` inside `programPushLogDue` | RED | :829 (T1s), :888 (T1t) |
| Z6 | the two intervals passed swapped | RED | :281–285 (T1a), :875 (T1t) |
| K1 | (rig) the exact clock never handed to the program | RED | :878 — T1t's PREMISE (`lineAt[live[0]] == 2.0`): on the plant's clock the test fails loudly, it does not pass vacuously |
| K2 | (rig) the operator's wait on the PUSH screen ignored | RED | :875 (T1t: 5 push lines, not 6) |

**9 of 9 RED and the control GREEN.** Every verdict is the suite's own assertion (no compile-time catches in this
pass). The line numbers are the worktree's (the copy is `cmp`-identical).

### The gates — every one re-run on this round's final tree (2026-10-29 14:16–14:37; outputs `fix3/gates/`)

- **Host build** (`cmake -S test -B build/test`, `cmake --build build/test -j8`, the doc-gate step included;
  `build1.log`): exit 0, no warning. The binary was REMOVED first (`rm -f build/test/shulib_tests`), the briefing
  regenerated before the build (no binary) and again after the suite binary existed — each capped
  (`ulimit -v 4194304; timeout 900`).
- **The suite** (`suite.out`, capped): **`2552 | 2552 passed | 0 failed | 3 skipped`, `2929945` assertions**, 20.6 s,
  61 MB peak. Case by case against the second round's 2,551 (`cmp_xml.out`): **0 gone, 1 new (T1t, 37 assertions),
  none changed** (T1a's edit is a comment). Through main's bounded runner (a scratch COPY of `tools/suite_run.py`
  pointed at this worktree's binary, `fix3/suiterun/`): `suite run: green — 2552 cases, 2929945 assertions`, exit 0.
- **`src_build_gate.py self-test`**: `OK (235 detector cases, each against a real build)`, exit 0. **`check`**
  (14:18–14:35): **`src build gate: PASS (all 7 builds link; …)`**, exit 0 — each of the seven `exit 0, 11/11 TUs
  compiled, warnings: 8 vendor-ignored, 0 disallowed, hash asserted`, programs and stdio printers checked. **No plant
  left:** `git status --porcelain` identical before and after; of the changed files' checksums only the briefing
  (regenerated) and this log differ — no header under `include/` was edited while it ran.
  `git diff ceb17c0 -- src/`: EMPTY (0 lines).
- **The ARM header gate** (`arm_and_ci.sh`): exit 0 over **225** headers, 0 output lines; the control fails with
  `-Werror=float-conversion`. **The CI greps:** `library is PROS-free (all of include/shulib except hal/pros/)`;
  `layering holds: core is sim-free`.
- **The compile-refusal harness** (`ctest -R compile_refusal`): `100% tests passed, 0 tests failed out of 200`.
- **api_doc_tool:** self-test OK; `doc coverage: 4843 public entities across 185 headers, all documented`;
  check-fresh exit 0 (no `///` comment changed this round; re-run after the last edit); `doc example scan: 541 quoted
  lines, all verbatim`; `removability: no public doc references docs/internal/`. **`doc_staleness_audit.py`:**
  `clean (29 live docs, 0 numeric claims checked)`. **`briefing_status.py check`:** `generated block current` (suite
  2,552 / 2,929,945).

### Prose review (pace rule 4) — this round's terms, grepped over `docs/` and `README.md` (chunk logs aside) (2026-10-29 14:30)

`T1s`, `T1t`, `exactClock`, `ExactClock`, `pushWaitSec`, `lineAt` — no living doc (test infrastructure). `push live`,
`push-live`, `keepalive`, `kPushLog*`, `programPushLogDue`: the changelog's sentence ("a changed count at most that
often, an unchanged one at least that often") — true, and it states no edge; `docs/api/` (generated, unchanged);
the R3a worksheet's `push live: ...` (robot two's src program, not this one — true). Nothing to fix.

### This round's close (2026-10-29 14:37 EDT, `date`)

- [x] The one finding dispositioned: REAL, reproduced (W13d, W13e GREEN on the old tree), fixed in the tests at the
  root — the PROGRAM's lines land on both edges (T1t) — and both mutants RED on fresh binaries, with Q4 and Q5 also
  cut at the program (W13f, W13g RED) and a correct inline program GREEN (the control).
- [x] Finding 12's Q4/Q5 bullet carries a pointer to this correction; its other text is left as history.
- [x] Every gate green on the final tree; `git diff ceb17c0 -- src/` EMPTY; no library header changed this round
  (`calibrate_program.hpp` md5 `13aaf900…`, as at the second round's close).
- [~] No new hardware belief. One observation for the coordinator, not this part's: the probe, built once at `-O1`,
  met GCC's `-Wformat-truncation` on `calibrate_panel.hpp`'s `snprintf(out, cap, "%s", buf)` in `panelChips` — code
  already in A1's base (`ceb17c0` :644); the host suite (no `-O`) and every ARM gate and build (`-Os`) are clean, so
  it trips only a host build at `-O1` or above with `-Werror`.

**For the coordinator:** files this round touched — `test/calibrate_program_rig.hpp` (the exact clock, the push wait,
`lineAt`; additions with defaults), `test/calibrate_program_test.cpp` (T1t; T1a's comment; the file header's T1
line), this log, and the regenerated `PROJECT-BRIEFING.md` (suite 2,552 / 2,929,945). No shared plan doc's lines
changed. Suite now **2,552** cases (the earlier close's "2,477 → 2,551" is +1). Scratch: `fix3/` (probes, mut,
gates, suiterun). Nothing committed, nothing pushed, no `pros`, no serial port.

**Where work stopped:** this round is complete at 14:37 EDT 2026-10-29; the next step is the coordinator's review and
merge.

## The coordinator's merge verification — CAL1 Part A2a (2026-10-29 14:40–15:45, main `d8fdb8b` + the squash of `cal1-a2a`)
- **Audit before the merge:** `audit_wt.py cal1-a2a` — 8 backed-up files, 0 suspicious (every mutated file equals its
  backup; the rest exist only as scratch copies); `git diff --stat ceb17c0 -- src/ tools/ Makefile` empty in the
  worktree; no stray root file. The transport commit `05a210e`, then `git merge --squash cal1-a2a` (base `ceb17c0`,
  A2a's real base).
- **Six conflicted paths:** `docs/api/README.md`, `all-entities.md` and the briefing (regenerated); the changelog —
  A2a's entry was written when 2.4 was the newest API, but it lands after MECH-A4 opened **2.5**, so it moved to the
  top of `## API 2.5` as "Added since 2.5" (its text says "no version bump", still true); REMAINING-WORK — row by row
  against `ceb17c0`: A2a changed only row 18b, main the rest (the sprint's and G3a's rows), so 18b is A2a's and every
  other row main's; build-order — the CAL1 status (A2a's LANDED line first, then the sprint's line of the same day
  with "A2a (verifying)" → "A2a landed the same day", then A2a's form of the A1 line without its stale "A2a next")
  and the `Next:` line (a word-level three-way merge against `ceb17c0`; both sides' additions present).
- **Gates on the merged tree** (`.claude/coordinator/logs/cal1_a2a_merge_verify.out`): host build with the doc gates;
  suite **2,701 | 2,701 passed, 3 skipped** (main's 2,626 + A2a's 75); `src_build_gate.py` self-test OK and check
  **PASS, all 7 builds**; ARM header gate over **228** headers clean; CI greps empty; api check-fresh, the briefing,
  removability and the staleness audit clean (its "binary older than `config/robot_config.hpp`" note is an mtime the
  src gate's self-test touched — the file's content is unchanged); the compile-refusal harness **251 of 251**.
- **The coordinator's hand mutations** (`muts_cal1_a2a_main.json` + `_b`; each on a fresh link, the whole suite under a
  4 GiB / 300 s cap, restored and hash-checked): **9 of 9 caught** — CA3 (a FLIPPED front never holds the save), CA4
  (USE NEW skips a pending MECHANISM PORT TURNED question), CA5 (a failed save reported as saved), CA6 (the watch's
  threshold edge made strict), CA7 (whether the card's record drives the turned port, inverted), CA8 (a BLUE
  cartridge refused) RED; **CA1** (a port listed twice not refused) and **CA2** (port 22 accepted) refused AT BUILD by
  the test's own compile-time checks (`test/calibrate_program_test.cpp` :2536–:2541 — the test's `static_assert`s,
  confirmed by the compiler's first error, not the mutation's own breakage); **CA9** as first cut (the shell's
  `line()` printing "") broke its own build (`-Werror=unused-parameter`) — NOT counted; re-cut as **CA9b** (the text
  dropped while the pointer is still read) → RED at `test/pros_calibrate_adapter_test.cpp` :263/:312/:317. Restored
  tree 2,701 | 2,701.
- **The sprint's order (the team lead, 2026-10-29):** MECH-A5 builds now, before CAL1's A2b and B; whichever of B and A5
  lands second rebases (the MECH brief's N9 fallback). Next for CAL1: **Part A2b** (`make_cal1_a2b_args.py`, base =
  this commit), when a slot frees after the session reset.
- **For later (not this part's):** the third verification round's builder saw GCC's `-Wformat-truncation` in
  `calibrate_panel.hpp`'s `panelChips` (A1's code) when built at `-O1` only; the suite build and every ARM build are
  clean. Logged for Part A2b's or B's look, not fixed here.

---

# Part A2b — D6 (a)'s typed reference and the proof through the real PROS shell (a library ADDITION)

> **This run builds Part A2b ONLY**, as the team lead's split (2026-10-08, "Split it") defines it (the brief's
> check of D6 (a), "Not settled" #1 (ii); §9.2): the typed facts and words (`TypedDrivePorts`,
> `CalibrateFacts::typedDrive`, `FactsList`, `hasTypedDrive`, `typedDriveDescription`, `typedDrivePortsFrom`, the
> reference rule and every word of §1.4's typed branch, `FinishFacts::typedPorts` and the six typed literals), C4
> (the exact/partial typed NOTE and the per-port lines), the golden test's 34 scenarios through the REAL PROS shell
> (`test/calibrate_golden_test.cpp`), the team file's example test (`test/calibrate_team_file_example_test.cpp`),
> the chapter, guide 12, the changelog (EXTENDING A2a's entry) and HA-187. NOT B's: no `src/` file
> (`git diff --stat e5a69b5 -- src/` must be EMPTY at the end); `withTypedDriveFrom` and robot two's mapping are B's.
>
> Executor: **Claude Opus 5.5** (subagent of the coordinator's workflow run), working in the git worktree
> `.claude/worktrees/cal1-a2b` (branch `cal1-a2b`, from `e5a69b5`). The executor does not commit, push, run `pros`,
> or touch the serial port. Scratch and backups: `.claude/coordinator/scratch/cal1-a2b/` (never `/tmp`, never the
> repository root). Every test binary runs capped: `(ulimit -v 4194304; timeout 600 …)`.

## A2b.0 Start (2026-10-29 20:38 EDT, `date`)

- Worktree clean at `e5a69b5` (`git status`). Read RESUMING.md's standing rules; the brief by the team lead's ruling
  of 2026-10-22 (header and answers, every Re-check R1–R18, the D6 (a) entry and its continuation, "The check of D6
  (a)" C1–C5 and its continuation, the Decisions D1–D6, §1.1–§1.4, §2.A's tests 1–11 and mutations 1–54 and both
  DoDs, §3.1, §3.3, §4, §5, §6, §7, §8, §9); this log's Part A2a in full where it hands on (A2a.0–A2a.9, both
  review rounds, the third verification round, the coordinator's merge record).
- This heading was appended FIRST, before any code.
- **The hand-ons A2b carries** (each taken up below, by number): (1) R19 — several turned declared ports are each
  their own question, in the facts' order; A2b's goldens are single-port; (2) the held and finished lines keep
  "the field disabled" (the status-bits line says which); (3) test 8b's "C-read counter counts 2 per push" through
  the REAL shell is A2b's (the golden harness); (4) the changelog's A2a entry (under API 2.5) is EXTENDED, not a
  second entry; (5) the touch-line and FIRST-note changes: no golden changes expected; (6) HA-187 registered here,
  with A2a's finding 7 in (2)'s blast radius; (7) `-Wformat-truncation` in `calibrate_panel.hpp`'s `panelChips` at
  `-O1` — look, fix if cheap, else log; plus A2a fix-2 finding 18 — name the chapter in `hal/pros/calibrate.hpp`'s
  "21 lines" sentence and regenerate `docs/api/`.

## A2b.R Resumption (2026-10-30 08:33 EDT, `date`)

- **Why.** The run that wrote A2b.0 (2026-10-29 20:38) was STOPPED by the account's 5-hour usage limit at ~21:10;
  the laptop was off overnight and restarted 08:28 on 2026-10-30. Executor of this resumption: **Claude Opus 5.5**
  (the coordinator's workflow run), the same worktree and branch (`cal1-a2b`, from `e5a69b5`).
- **State found** (`git status`, `git diff -- src/ Makefile`): two paths changed — this log (A2b.0 appended) and the
  generated `PROJECT-BRIEFING.md` (regenerated with no binary: "Suite: not built"); `src/` and the `Makefile` carry
  NO gate self-test plant (0 diff lines); no mutation backup under `scratch/cal1-a2b/` (`mut/` and `probe/` are
  empty). The stopped run's scratch (`logs/b0_*`, 20:42–20:44) shows only a baseline: the briefing regenerated, a
  host build (`build rc=0`) and a suite run (`suite rc=0`). Its counts are NOT taken from those files: the binary is
  removed and rebuilt from source before any result here.
- **So nothing of A2b's code was written**: every item in A2b.0's list is still to do, from the top.

## A2b.1 Baseline, and the library's typed reference (2026-10-30 08:37–08:50)

- **Baseline** (the binary removed, the briefing regenerated with none, `cmake -S test -B build/test`, the build
  capped): exit 0, no warning; the suite through `tools/suite_run.py green`: **2,701 cases, 2,935,334 assertions,
  3 skipped** — main's count after the A2a merge (`scratch/cal1-a2b/logs/r0_*`, the XML case by case in
  `r0_suite.xml`).
- **`calibration/calibrate_program.hpp`** — the brief's §1.1 typed facts, from the draft `brief-CAL1/d6a/v3/p3r/`
  (the split had removed them, A2a.1), each line re-read against A2a's file as it stands: `TypedDrivePorts`
  (`left[21]`, `right[21]`, signed, 0 = unused), `CalibrateFacts::typedDrive` appended LAST, `FactsList`,
  `FactsProblem::list`, `hasTypedDrive()` (either side), `factsProblem()`'s typed checks after the mechanisms' (a
  port outside 1..21 by |port| — so −128 too; twice on one side or once on each, by |port|, the SECOND entry named;
  one side only), `typedDriveDescription()` (zero slots skipped, the order kept), `typedDrivePortsFrom()`; the
  FACTS REFUSED line names `typedDrive.left[i]`/`.right[i]` and its value; the `card :` line's typed words; the
  front check's typed branch (no usable record on a present card — neither file opened, or files that opened and do
  not decode — AND typed ports: the per-file lines, the `typed drive ports:` list, the counts line, CANNOT TELL /
  SAME exact / SAME partial with C4's per-port lines (`emitTypedDifferences`) / FLIPPED); the typed FRONT FLIPPED
  hold's five lines; `Held::typedRef`; each MECHANISM PORT TURNED question's evidence judged against the REFERENCE
  (`onCardOf[k]` from the typed ports when they were it — A2a's per-port questions kept); its typed screen and
  serial words; USE NEW's FIX line and NOTE; `FinishFacts::typedPorts`. The header's FRONT CHECK block now states
  the typed reference (it said "A2b adds …").
- **`calibration/calibrate_panel.hpp`** (additions only, plus one behaviour-preserving line — below): the six
  typed literals (`kFlipWhatTyped`, `kFlipNewTyped`, `kFlipWarnTyped`, `kNoteTypedSame`, `kNoteFixTyped`,
  `kNoteTypedPartial`) in `kAll`'s compile-time budget check; `FinishFacts::typedPorts` (default false); the FRONT
  FLIPPED view swaps its three reference lines when it is set. Header and `holdLine2`'s comment updated.
- **Rulings beyond the brief (each logged here):**
  1. `FinishFacts::typedPorts` is set only while a hold SHOWS (`held_.held && held_.typedRef`) — the draft set it
     from `typedRef` alone. The view reads it only on a held FRONT FLIPPED screen, so no output differs; it keeps a
     stale flag from ever reaching a later screen. **Alternative:** the draft's form — equivalent on every path
     today.
  2. The typed NOTEs (`FRONT MATCHES …`, `… SOME TYPED PORTS DIFFER …`, `FIX YOUR TYPED DRIVE PORTS …`) stay under
     a NOT SAVED header; A2a drops only `FIRST RECORD` there (it names a record the run made — A2a's review round
     2, finding 5). The typed NOTEs describe the COMPARISON of this measurement with the typed ports, which is true
     whether or not the save succeeded. **Alternative not taken:** drop every NOTE on NOT SAVED — it would hide a
     FIX YOUR TYPED DRIVE PORTS that is still true.
  3. Hand-on (7), the `-Wformat-truncation` in `panelChips` (A1's code): **looked, fixed (cheap)**. Reproduced with
     `g++ 13.3 -Wall -O1` on a one-function probe (`scratch/cal1-a2b/probe/wft/`): `calibrate_panel.hpp:753 '%s'
     directive output may be truncated writing up to 267 bytes into a region of size 96` (and 51) — at `-O1` ONLY
     (`-O0`, `-O2`, `-O3`, `-Os`: silent). The line copies the chips buffer inside `if (used <= kMediumCols)`; it is
     now `snprintf(out, cap, "%.*s", static_cast<int>(used), buf)` — exactly `used` characters, which is exactly
     what `"%s"` copied (the buffer holds `used` characters in that branch). Re-probed: silent at every level. A
     behaviour-preserving edit to a non-additive line of the panel (its brief rule is "additions only"): logged as a
     delta; mutation P1 below pins that the chips line is tested.
- **The build** (09:00): `docs/api/` regenerated (`calibrate_panel.md`, `calibrate_program.md`, `README.md`,
  `all-entities.md`); the suite, unchanged tests only: **2,701 / 2,701 passed**, assertions 2,935,334 → 2,935,704
  (+370: the literal and line scans that walk `include/shulib`).

## A2b.2 The golden test through the REAL PROS shell — and its goldens checked against the brief's (2026-10-30 08:50–08:58)

- **`test/calibrate_golden_rig.hpp`** (new) — the probe harness (`brief-CAL1/d6a/v3/harness4.cpp`) made project
  code: robot two's plant stepped from the shim clock's advance hook, the operator reading the shim screen's draw
  log, the program composed EXACTLY as `hal::pros::runCalibrate()` composes it (`ProsDeviceRegistry`,
  `ProsDeviceProvider{registry}`, `ProsBrainScreen`, `ProsCalibrateIo`) except the card, `ProsFileStore{<temp
  dir>/card/}`; `calibrateInitialize(label, stamp)` first (initialize() before opcontrol()); fd 1 redirected to a
  file for the run (the shell's line() is one write() on fd 1, SER1). Captures: the serial (normalized: the card's
  directory → `<card>/`, the git hash → `<hash>` — the only two things not the program's), the draw log, both card
  files, the shim's per-port C-read counts, configuration/command counters, largest |mV| and powered steps, and the
  probe's FORWARD score of the saved record. Per-run state (the probe's `static pulsesSeenAt`) moved into the
  operator; the probe's `exit(3)` deadline became a flag the test fails on (and a disable so the program returns).
- **`test/calibrate_golden_test.cpp`** (new) — the brief's 34 scenarios by the probe's knobs, robot two's facts
  LITERALLY with NO typed ports (the typed scenarios add the probe's lists — case 8 = robot two's 2026-10-08 typed
  table, written literally, never read from `src/`: G3a's D7 flips that table), the seeds by the probe's rules (a
  seeded scenario takes `one`'s saved record, REQUIRED to exist and decode first — C2). Per scenario: the three
  goldens (a missing golden FAILS; a mismatch names its first differing line), every serial line `[R3D] `-tagged,
  the declared ports never configured or commanded (gearing/units/voltage/brake-mode/brake() counters 0, |mV| 0),
  the WATCH's C reads — **2 per push it judged, 1 per push refused at DONE, 0 with no push (hand-on (3): test 8b's
  "C-read counter counts 2 per push" through the REAL shell)** — the units read paired with each position read and
  no gearing read (rotations, the shim's default), the typo's port 7 geared once and never powered, the undeclared
  8 never powered and 9 pulsed to exactly 9,000 mV, and the FORWARD score. Plus: `typed-card-wins` EQUALS its
  untyped control and `typed-nocard` EQUALS `nocard` (serial, screen, card); the goldens directory holds exactly the
  34 × 3 files; a source pin that `runCalibrate()` composes the same five (the card at its DEFAULT root) and the rig
  the same, the card's root its one difference. `SHULIB_GOLDEN_OUT=<dir>` writes the actual files beside the
  comparison (the case still compares and still fails) — nothing regenerates itself.
- **The expectations a golden cannot hold were written from the probe's own counters** (`run_new/*.stderr.log`:
  `C-API reads`, `drives N of M`), re-derived: the one difference is `curved` — 3 reads, not the probe's 4, because
  A2a.1 #3 judges only an ACCEPTED push (run 1's refused push is read once, at its start).
- **The goldens, before they were committed** (`scratch/cal1-a2b/gold1/` ← `SHULIB_GOLDEN_OUT`; `cmp/classify.py`,
  `cmp/classify2.log`): every file compared with `brief-CAL1/d6a/v3/run_new/` (the probe's card path → `<card>/`,
  its hash `probe` → `<hash>`; its card binaries as the same hex text). **66 of 102 files byte-identical** (all 34
  card files; 32 of 34 screen logs). Every other differing line has a named reason — **residual 0**:
  101 serial lines W8 (A2a.1 #8: "a verified backup … first" in the banner and the `record file:` line, the watch
  banner's "and the save waits"); 56 serial lines E (the probe's card path was cut at 95 characters,
  `<card>/shulib_drivetrai`; this rig's short root prints it whole); 2 screen lines C5 (A2a.1 #5: `slip18` and
  `undeclared`'s finished-screen warning cut to one row with " ..." under the NOTE); 2 serial lines R3 (A2a.1 #3:
  `curved`'s refused push gets no watch lines). **No timing line differs** — the real shell reproduces the probe's
  plant to the digit — and no typed line differs: every D6 (a) and check-of-D6 (a) scenario prints, paints and
  saves what the brief's probe did. Hand-on (5) confirmed: no golden pairs NOT SAVED with FIRST RECORD, and no touch
  line changed (none of the 34 has two held questions in one run).
- Then copied into **`test/goldens/calibrate/`** (102 files, 1.3 MB) and re-run: **38 / 38 cases, 936 assertions**
  (2.2 s, 25 MB peak); order-independent (a seeded scenario alone, `--order-by=rand`), and inside the whole suite.

## A2b.3 The typed tests and the team file's example test (2026-10-30 08:58–09:01)

- **`test/calibrate_program_typed_test.cpp`** (new, on A2a's rig — no existing test TU edited): **T7a** agreeing in
  every port (SAME, `FRONT MATCHES YOUR TYPED DRIVE PORTS`, never FIRST RECORD); **T7b** partial (8 agree, 1 sign
  only, 1 each list only: the three per-port lines verbatim, the partial NOTE); **T7b-2** the SIDE and side-AND-sign
  lines (13 typed on the other side: side only; 14 other side and sign: flipped — "8 agree, 1 flipped, … 1 side
  only", SAME); **T7c** flipped on an empty card — the five typed lines, the typed screen (and NONE of the card's
  three lines drawn), nothing written until USE NEW, the FIX line and NOTE, `SAVED -- NEW FRONT`, the record as
  pushed, FORWARD 0 of 10; **T7d** RUN AGAIN → "the card is unchanged (it holds no usable record)", never "keeps
  its record", run 2 SAME and saved; **T7e** a disable while held → the same words, NO file; **T7f** the 5/5 tie
  and no port in common → CANNOT TELL, `FRONT NOT CHECKED`; **T7g** a record on the card with typed ports FLIPPED
  against it → the whole run EQUALS the untyped run (every line, draw and byte); **T7h** undecodable files with typed
  ports (both files; flipped → held and USE NEW; the backup alone; an EMPTY record file) — each opened file's own
  line, then the typed ports; **T7i** no card → EQUALS the untyped no-card run, no typed line; **T7j** drive port 12
  declared with typed ports driving it → `your typed ports DRIVE with port 12` (screen and serial), its per-port
  line, the discard words; **T7k** typed FRONT FLIPPED then a claw's MECHANISM question → the touch line's "next
  question" words, FIX at USE NEW, and the NOTE `PORT 6 KEPT …` over `FIX …` (§1.4's order). **T6g** the seven typed
  refusals (−22, 22 right, −128, 11 twice on a side, 11 on each side, LEFT only, RIGHT only) each FACTS REFUSED
  naming the entry and value before ANY device, and a mechanism problem first; **T6h** zero slots skipped through
  the program; **T6i** the four functions at COMPILE time (17 `static_assert`s, counted).
  Expected strings read off the probe's goldens; T7b-2's and T7k's `-1692` were reasoned (and two of my first
  expectations were WRONG — T7b-2's "0 flipped" and T7k's "+1692": both corrected to the plant's truth, by
  reasoning about the check's rule and the claw following the backward-turning left wheel, not by reading the
  program's output).
- **`test/calibrate_team_file_example_test.cpp`** (new): the 21 lines VERBATIM at the top of the file (they are the
  file's program: `main.h` resolves to the shim's; the five entry points are defined here, and no other TU defines
  them); the facts read back, usable, NO typed ports (compile and run time); the chapter's two typed spellings in
  their own facts — the one line, and `typedDrivePortsFrom(kRobot.drivetrain.tank.description)` over a real
  `inline constexpr config::RobotConfig` tank that is itself `valid()` (the brief's probe config was not: its sides
  carried no inches per radian and no width — completed here, so the example is a drive program's real config);
  and the five entry points RUN through the shim: the banner (default label), the three state lines, and
  `opcontrol()` running the library program on an empty brain with no card — the census, then `REFUSED … the
  registry reports 0 IMUs`, `CANNOT START` painted, no motor touched.
- **The suite** (09:01): **2,757 cases, 2,937,329 assertions, green** (`suite_run.py green`) — 2,701 + 56 new.

## A2b.4 The documents (2026-10-30 09:05–09:20) — the brief's §3.1, A2b's share

- **`docs/guide/19-calibrating-your-robot.md` (NEW)** — the next free chapter number at this base (18 is MECH-A3's;
  **for the coordinator:** G3a Part 1, in flight, adds no guide chapter by its file list — if anything lands a 19
  first, renumber this one and its three links). For a team, in the brief's order: what it measures and what you
  state (§0.4's table, the optional typed row included), YOUR FILE — the 21 lines VERBATIM, bound by
  `check-examples` to `test/calibrate_team_file_example_test.cpp`, with what a team gets wrong and what it sees;
  list EVERY motor that is not a drive motor (R11; unplug them with nothing declared); one run (1/4 IMU … SAVED,
  RUN AGAIN to check); what each ending means (SAVED, NOT SAVED, SAVE UNCONFIRMED — B1's words; both files; delete
  BOTH to reset); FRONT FLIPPED with BOTH causes (R10, qualified "on a tank whose two sides are mirror images");
  MECHANISM PORT TURNED (R12, one screen per port); the typed drive ports (both spellings quoted from test 11; the
  exact and partial NOTEs; FLIPPED against them; "a robot with neither … gets only FIRST RECORD: push with the end
  its autons call the front"); X-drive refused (R13); "nothing in this program has run on a robot in this form",
  the calibrator on robot two 2026-09-24, by date. No `docs/internal` path (`check-removability`: clean).
  **A dated status line** ("As of 2026-10-30 no program in this repository runs the standard Calibrate yet; robot
  two's own … moves onto it in CAL1's next part") — **for Part B: reword it when robot two moves on** (the brief's
  §3.2 list does not name the chapter; this sentence is new).
- **`docs/guide/12-when-things-go-wrong.md`** — a new section, "Calibrate held its save, or left a note": the
  brief's rows (FRONT FLIPPED, both causes; FRONT FLIPPED against the typed ports; MECHANISM PORT TURNED; FRONT NOT
  CHECKED; FIRST RECORD; FRONT MATCHES YOUR TYPED DRIVE PORTS — "nothing to do", only on an exact match; FRONT
  MATCHES, SOME TYPED PORTS DIFFER — C4's words, the LEFT OUT/declared/did-not-turn caveat; FIX YOUR TYPED DRIVE
  PORTS; a LEFT OUT or refused pulse naming a mechanism; FACTS REFUSED / NOT A TANK / … / TOO FEW MOTORS; B1's SAVE
  UNCONFIRMED and NOT SAVED-with-a-damaged-record-file, worded from `formatSaveReport(No)`'s own lines).
- **`docs/guide/README.md`** — the chapter's row; the maintenance note's "16–18" → "16–19" and its history sentence.
  **`mkdocs.yml`** — one nav line (`19 — Calibrating your robot`), after 18's.
- **`docs/changelog.md`** — A2a's entry under `## API 2.5` EXTENDED (hand-on (4)), not a second entry: the heading's
  dates, the "written when it was built" note (Parts A2a and A2b), a sub-bullet for the typed reference's behaviour,
  the typed facts' API by name (`TypedDrivePorts`, `CalibrateFacts::typedDrive`, `FactsList`, `FactsProblem::list`,
  `hasTypedDrive()`, `typedDriveDescription()`, `typedDrivePortsFrom()`), the panel's `FinishFacts::typedPorts` and
  the six typed literals by name, HA-187 linked where the watch's belief was "registered by CAL1's next part", "What
  you must do" pointing at chapter 19, and the contributors' note (the golden test, `test/goldens/calibrate/`,
  `SHULIB_GOLDEN_OUT`, the example test, the chips line's `"%.*s"`). Still "No version bump — additive".
- **`docs/hardware-assumptions.md`** — HA-187 (§3.3's text, AMENDED by R6/R12) in a new section "CAL1's addition"
  before Group R4, its index row between HA-186 and HA-189, the status paragraph's CAL1 Part A2b sentence and
  "Status: 7 of 179"; the Held line drops `HA-187..HA-188` for `HA-188 (CAL1's second number, released unused when
  CAL1 registered HA-187 — never reissued)` — parsed by `briefing_status.py`'s `HELD_LINE` exactly as before (the
  next free number stays HA-216). **Hand-on (6) worded in:** (2)'s blast radius now carries A2a's finding 7 — the
  held screen's EVIDENCE line depends on the gearing the motor REPORTS (a BLUE drive motor reporting green shows
  about ⅓ of the drive's travel, inviting SAVE; detection unaffected) — and Station G's settle reads an unconfigured
  claw's `motor_get_gearing` before any program gears it.
- **Headers (their `///`-free banners, and so `docs/api/` regenerated):** `hal/pros/calibrate.hpp`'s "21 lines;
  CAL1's next part adds the guide chapter" → names chapter 19 (A2a fix-2 finding 18's hand-on), and its "the belief
  CAL1 registers as HA-187 (in Part A2b)" → "the belief is HA-187 in docs/hardware-assumptions.md"; the same in
  `calibrate_program.hpp`'s WATCH block. `docs/api/` regenerated (`pros-calibrate.md`, `calibrate_program.md`,
  `calibrate_panel.md`, `README.md`, `all-entities.md`).
- **Internal:** `build-order.md` — CAL1's status block gains a 2026-10-30 line (Part A2b BUILT — "it lands at the
  coordinator's verification and merge"; B next), the `Next:` line's CAL1 clause the same, and the CAL1 entry's
  "HA-187/HA-188 reserved" → registered/released; `REMAINING-WORK.md` row 18b; `roadmap.md` a "Now (2026-10-30)"
  paragraph after A2a's (A2a's left as history). **For the coordinator:** "BUILT … lands at the merge" is written
  in four places (build-order ×2, roadmap, REMAINING-WORK) — make them LANDED with the merge's date then.
- **Deliberately NOT touched:** `README.md` (its Calibrate lines — "the library's standard Calibrate, which no
  program builds yet", `calibration/`'s line — are still true; README is Part B's "only if false"); `docs/faq.md`
  (:154-176, :253 — robot two's program and the safe save, unchanged and true); guide 07 (the program list — Part
  B's); the worksheet (Station G — Part B's); `HANDOFF-remaining-work.md` (its inventory line for
  `calibrate_program.hpp` is still true; A2b adds no header); `ORIENTATION.md` (no `hal/pros` count moved);
  `RESUMING.md`; the brief (its Re-check notes are the coordinator's); `test/pros_shim/` (no shim change was needed:
  A1's `main.h`, `screen.h`, `misc.h`, C reads and advance hook carried the golden test as they stand);
  `test/CMakeLists.txt` (the new tests are globbed; `SHULIB_SOURCE_DIR` already defined); every other chunk's
  files — `git diff --stat e5a69b5 -- src/ tools/ Makefile test/CMakeLists.txt test/pros_shim .github` is EMPTY.

## A2b.5 Mutations — every one on a FRESH binary, capped, SEEN red, restored (2026-10-30 09:06–09:27)

**Method** (`scratch/cal1-a2b/mut/run_mut.py` — A2a pass 5's runner, run on the worktree itself; lists
`make_list.py` → `pass1.json`, `make_list_b.py` → `pass1c.json`/`pass1b2.json`; outputs `pass1.out`,
`pass1c.out`, `pass1b2.out`; backups under `mut/bak/`, never /tmp): each anchor checked to occur EXACTLY once
before the pass (`make_list.py`: 33 mutations, 0 bad anchors); each mutant built with `shulib_tests/fast`,
REQUIRING the link AND every object whose depfile names the mutated file rebuilt after the edit (5 objects for the
program and the shell: `calibrate_golden_test`, `calibrate_program_test`, `calibrate_program_typed_test`,
`calibrate_team_file_example_test`, `pros_calibrate_adapter_test`; the panel's ~25); the WHOLE suite run capped
(RLIMIT_AS 4 GiB, 600 s); the file restored in `finally`, `cmp`-identical, its timestamp bumped. After every pass
`md5sum -c` of the touched files against `mut/pre_md5.txt`: **OK, 4 of 4**. No mutant was killed by the cap or the
timeout. A compile-time catch is counted only where the compiler's FIRST errors are a TEST's `static_assert`
(printed by the compile_fail mode, `pass1c.out`) — and each of those mutants was ALSO run with T6i's
`static_assert`s turned into run-time `CHECK`s (a `#define static_assert(...) CHECK(static_cast<bool>(...))` inserted
before T6i, in the mutant's own edit list and restored with it — `pass1b2.out`; a first form that replaced the word
with `CHECK(` broke on doctest's "expression too complex" for `&&` chains, `pass1b.out`, and was NOT counted), to
see which RUN-TIME tests catch it too.

| # | Brief | The mutant | Verdict | Caught by |
|---|---|---|---|---|
| M28 | 28 | the typed ports checked even when the card holds a record | RED | golden `typed-card-wins`, the equality case, T7g |
| M29 | 29 | the typed ports ignored (no D6 (a)) | RED | 22 cases: every typed golden, T7a–T7k |
| M30 | 30 | files that do not decode NOT checked against the typed ports | RED | goldens `typed-undecodable(-flip)`, `bakonly-garbage-typed`, T7h |
| M31 | 31 | `typedDriveDescription` drops the signs | RED — **compile time, by the TEST's assertion**: T6i's description `static_assert`s (`calibrate_program_typed_test.cpp` :662–:667); at run time 24 cases (`typed-first` and every typed golden, T7*) |
| M32 | 32 | `typedDriveDescription` swaps the sides | RED — compile time, T6i (:662–:667); at run time 24 cases |
| M33 | 33 | `factsProblem` misses a port once on EACH side | RED — compile time, T6i's `across` (:649); at run time T6g ("11 on each side") |
| M34 | 34 | `factsProblem` allows \|port\| > 21 (typed) | RED — compile time, T6i's `past` (:646); at run time T6g (−22, 22, −128) |
| M35 | 35 | `factsProblem` allows one side typed | RED — compile time, T6i's `leftOnly`/`rightOnly` (:638, :642); at run time T6g |
| M36a | 36 | the typed FRONT FLIPPED hold prints the CARD's words (serial) | RED | goldens `typed-flip-*`, `typed-undecodable-flip`, T7c, T7h |
| M36b | 36 | `FinishFacts::typedPorts` left false | RED | the same goldens' screen logs, T7c |
| M36c | 36 | the panel's FRONT FLIPPED view ignores `typedPorts` | RED | the same (5 cases) |
| M37 | 37 | SAME (exactly) against the typed ports gives no NOTE | RED | goldens `typed-first`, `typed-undecodable`, `bakonly-garbage-typed`, `typed-flip-again`; T7a, T7d, T7h, T6h |
| M38 | 38 | a discarded held run always says "the card keeps its record" | RED | goldens `typed-flip-again`, `typed-flip-disable`, `typed-watch-typo`, `empty-watch-untyped`, … (9) |
| M39 | 39 | the MECHANISM PORT TURNED evidence ignores the typed reference | RED | golden `typed-watch-typo`, T7j |
| M40 | 40 | no card: the typed ports consulted anyway | RED | golden `typed-nocard`, the equality case `typed-nocard` IS `nocard`, T7i |
| M41 | 41 | the `card :` line ignores the typed ports | RED | 13 cases (every typed golden with a card, T7a, …) |
| M42 | 42 | USE NEW against the typed ports: no FIX line, no NOTE | RED | goldens `typed-flip-new`, `typed-undecodable-flip`; T7c, T7h, T7k |
| M43 | 43 (typed form) | the discard words keyed on the REFERENCE, not the card | RED | golden `empty-watch-untyped`; T5c, T5o |
| M44 | 44 | typed branch: "no record yet" decided by the record FILE alone | RED | golden `bakonly-garbage-typed`; T7h |
| M50 | 50 | a partial typed match painted FRONT MATCHES (the D6 (a) draft's rule) | RED | goldens `typed-partial`, `typed-watch-typo`; T7b, T7b-2, T7j |
| M51 | 51 | the per-port lines dropped (the NOTE kept) | RED | the same |
| M52 | 52 | a SIGN-only port reported as a SIDE difference | RED | golden `typed-partial`; T7b |
| M53 | 53 | `hasTypedDrive` reads the LEFT only | RED — compile time, T6i's `rightOnly` (:641–:642); at run time T6g ("the right only" runs untyped instead of refusing) |
| M54 | 54 | CANNOT TELL against the typed ports paints the MATCHES NOTE | RED | goldens `typed-tie`, `typed-nocommon`; T7f |
| X20 | ruling 1 | CONTROL: `typedPorts` from `typedRef` alone (the draft's form) | **GREEN, as reasoned** (equivalent: the view reads it only on a held FRONT FLIPPED screen) | — |
| X21 | hand-on 7 | `panelChips` copies one character short | RED | 33 cases: every golden's screen log (the PULSES chips line), the panel tests |
| X22 | test 10 | `runCalibrate()`'s card given a root | RED | the golden test's source pin |
| X23 | 8b / hand-on 3 | the shell's watch read makes TWO position reads | RED | 35 cases: every golden's C-read count ("2 per push" through the real shell), PC1 |
| X24 | 11 | `typedDrivePortsFrom` puts the RIGHT's ports on the left | RED — compile time, by the TESTS' assertions: T6i's inverse (:667) and the team file's `factsProblem(kOurRobotFromConfig).why == nullptr` (`calibrate_team_file_example_test.cpp` :98) — namespace scope, so no run-time form |
| X25 | 6 | the FACTS REFUSED line names the other side | RED | T6g |
| X26 | 6 | the typed range refuses port 21 (off by one) | RED — compile time, T6i's `edge` and `gappy` (:644, :660); at run time T6i only (no program run types port 21) |
| X27 | 6 | a typed duplicate names the FIRST entry | RED — compile time, T6i's `lastSlot` (:655); at run time T6g, T6i |
| X29 | 10 | the `typed drive ports:` line prints the RIGHT as the LEFT | RED | 21 cases: every typed golden, T7* |

**32 of 32 RED and the control GREEN** — the brief's typed set (28–42, 44, 50–54) and 43 in its typed form, plus 9
of this run's own. **Not re-run, with reasons:** 1–3, 16, 17 (A1's, unchanged code); 4–15, 18, 20–27, 45–49 (A2a's,
whose code A2b did not change — every one of them was RED at A2a's merge, and the 34 goldens add catches,
never remove them); 19 (`detail` namespace — A2b adds no `detail` code).

## A2b.6 Prose review (pace rule 4) — grepped, every hit in a living document READ, then logged (2026-10-30 09:20–09:40)

**Grepped** (GNU `command grep`) over `README.md`, `docs/*.md`, `docs/guide/`, `docs/cookbook/` and the living
internal documents (build-order, HANDOFF-remaining-work, ORIENTATION, REMAINING-WORK, PROJECT-BRIEFING, RESUMING),
`docs/api/` aside (generated, check-fresh): A2b's names — `typedDrive`, `typed drive port`, `HA-187`, `HA-188`,
`A2b`, `golden`, `calibrate_team_file`, `Calibrating your robot`, `chapter 19`, `19-calibrating`,
`-Wformat-truncation`, `panelChips`, `FRONT MATCHES`, `FIX YOUR TYPED`, `FIRST RECORD` — and the phrases A2a left
pointing at this part: `next part`, `still to come`, `registers as HA-187`, `registered by CAL1's next part`,
`CAL1's next part adds`, `in Part A2b)`; and the counts a register entry moves (`178`, `of 178`, `HA-216`, a guide
chapter count).
**Read, and true after A2b:** the changelog's API 2.4 seams entry (:421 "its Parts A2a and A2b", :501 "comes in the
next parts" — history, written when built); the changelog's other "next part" hits (the MECH core's, unrelated);
`hardware-assumptions.md` :89/:169/:380/:381/:2537–2559 (HA-177..180's own text; "With it, 178." is MECH-A3's
history line, followed now by A2b's "With it, 179."); `roadmap.md` :403 and :406–417 (A1's and A2a's "Now"
paragraphs — history, A2b's added after them); guide 18 :17 and guide 11 :713 ("still to come" — the MECH core's and
H1's); `build-order.md` :1676 (the brief's answers — robot two gets FIRST RECORD: still true); the FAQ :160–176 and
:253–255 (the safe save and robot two's program — unchanged).
**Fixed (made false, or left stale, by A2b):** `hal/pros/calibrate.hpp` ("CAL1's next part adds the guide chapter";
"registers as HA-187 (in Part A2b)") and `calibrate_program.hpp` ("the belief CAL1 registers as HA-187, in Part
A2b") — now chapter 19 and HA-187 by name, `docs/api/` regenerated; the changelog's A2a entry (the watch's belief
"registered by CAL1's next part", "What you must do … come with CAL1's next part"); `build-order.md` :1690 (CAL1's
entry: "HA-187/HA-188 reserved" → registered/released) and CAL1's status block and `Next:` clause;
`REMAINING-WORK.md` row 18b; the register's Held line and status count; the guide README's chapter list and its
maintenance note.
**Deliberately NOT touched** (A2b.4's list): README, the FAQ, guide 07, the worksheet, HANDOFF, ORIENTATION, RESUMING,
the brief, the shim, `test/CMakeLists.txt`, every other chunk's files.

## A2b.7 The gates — every one on the final tree (2026-10-30 09:28–09:58; outputs `scratch/cal1-a2b/gates/`)

- **Host build** (the binary removed, the briefing regenerated with none, `cmake -S test -B build/test`, the full
  build with its doc-gate step, capped): exit 0, 0 warnings — re-run after the last header edit (09:52, below).
- **The suite** (`tools/suite_run.py green`): **`suite run: green — 2757 cases, 2937329 assertions`** (3 skipped;
  30.2 s, 53.8 MB peak). Case by case against the A2b baseline (`logs/r0_suite.xml` → `gates/suite.xml`): **0 gone,
  56 new** (38 `calibrate_golden_test`, 15 `calibrate_program_typed_test`, 3 `calibrate_team_file_example_test`),
  **0 failing**, 3 changed — all literal/file scans: `calibrate_panel_test` P9 (4,789 → 5,089: the six literals)
  and two `drive_program_source_test` P3-T3 scans (+70, +16: the new strings and files).
- **`briefing_status.py`** regenerated with the binary, then `check`: `generated block current` (suite 2,757 /
  2,937,329; 228 headers; HA: 179 registered, next free HA-216).
- **api_doc_tool:** self-test OK; `doc coverage: 5078 public entities across 188 headers, all documented`;
  check-fresh exit 0 (0 lines; `docs/api/` regenerated after every `///`/banner change); `doc example scan: 623
  quoted lines, all verbatim (10 source files)` — chapter 19's three listings included; `removability: no public doc
  references docs/internal/`. **`doc_staleness_audit.py`:** `clean (32 live docs, 0 numeric claims checked)`.
- **The ARM header gate** (`gates/arm_and_ci.sh`, its TU under `gates/`): exit 0 over **228** headers, 0 output
  lines; the control fails (`-Werror=float-conversion`). **The CI greps:** `library is PROS-free (all of
  include/shulib except hal/pros/)`; `layering holds: core is sim-free`.
- **The compile-refusal harness** (`ctest -R compile_refusal`): `100% tests passed, 0 tests failed out of 251`.
- **`src_build_gate.py self-test`** (09:34–09:49): `OK (235 detector cases, each against a real build)`, exit 0.
  **`check`**: `src build gate: PASS (all 7 builds link; …)` — each of the seven `exit 0, 11/11 TUs compiled,
  warnings: 8 vendor-ignored, 0 disallowed, hash asserted` — run at 09:52 and AGAIN at 09:56 after the last header
  edit (the gate's step 0 reads `include/shulib` text). **No plant left:** `git status --porcelain` and the md5 of
  every file under `src/ include/ test/ tools/` and the `Makefile` identical before and after each run.
  `git diff --stat e5a69b5 -- src/ Makefile tools/`: EMPTY.
- **The team file as a PROS project** (`scratch/cal1-a2b/team_proj/`: `git archive e5a69b5`, the three changed
  headers overlaid and `cmp`-identical, `src/` replaced by the 21 lines — `cmp`-identical to the example test's
  first 21): `make` exit 0, ONE warning (liblvgl's own), `bin/hot.package.bin` **85,104 bytes** (A2a's 81,604:
  +3.5 KB, the typed code), text 83.10 KB; its strings carry the typed words (21 hits), its symbols the four C calls.
- **The last edit** (09:52, after the first src-gate run): `calibrate_program.hpp`'s THE LIMIT paragraph gains
  landmine 4's typed clause ("With typed drive ports in the facts, such a run is checked against THEM instead —
  still not against the record the card holds") — a comment; `docs/api/` regenerated, and the build, the suite,
  the briefing, every api/doc gate, the ARM gate, the src gate's `check` and the team project re-run after it, all
  as above. (The src gate's `self-test` ran before it: its detectors plant into `src/`, which includes no changed
  header — `grep`: no `src/` file includes `calibrate_program.hpp` or `hal/pros/calibrate.hpp`.)

## A2b.8 Close of Part A2b (2026-10-30 09:59 EDT, `date`) — A2's DoD as the split leaves it to A2b

- [x] The typed facts, words and C4 in `calibrate_program.hpp` (A2b.1), `FinishFacts::typedPorts` and the six
  literals in `calibrate_panel.hpp` (additions, plus the chips line's behaviour-preserving `"%.*s"` — a logged
  delta), every public member documented (coverage, above); `git diff --stat e5a69b5 -- src/` EMPTY.
- [x] Tests 2 (typed half: T7a–T7k), 6 (typed: T6g–T6i), 10 (the 34 goldens through the real PROS shell, the two
  equalities, the 8b C-read count — hand-on (3)) and 11 (the team file) green; the suite 2,701 → 2,757, every
  pre-existing case passing, none edited (`git status test/`: new files only).
- [x] Mutations 28–42, 43 (typed form), 44, 50–54 RED on fresh binaries, logged with their catches (A2b.5), plus 8 of
  this run's own and a GREEN control; compile-time catches are the TESTS' `static_assert`s, each also shown caught at
  run time where a run-time form exists.
- [x] The same gates as A1/A2a (A2b.7), `src_build_gate.py` self-test and check; the team file links as a PROS
  project.
- [x] HA-187 in the register (index row, entry, status count, Held line; HA-188 released) with A2a's finding 7 in
  (2)'s blast radius — hand-on (6). Docs per §3.1 (A2b.4), the prose review logged (A2b.6).
- [x] The 34 golden scenarios committed (`test/goldens/calibrate/`, 102 files), checked against the brief's probe
  goldens first (residual 0, A2b.2); `typed-card-wins` asserted equal to its untyped control and `typed-nocard` to
  `nocard`; every new screen string — the six typed literals included — inside `kAll`'s compile-time check.
- **Hand-ons, each closed:** (1) R19 — no golden needs the multi-port wording (all 34 are single-port; T7k's
  two-question run is on the rig); (2) the goldens keep "the field disabled" words — residual 0; (3) test 8b's
  "2 per push" through the real shell — the golden test (and mutant X23); (4) the changelog EXTENDED; (5) no golden
  touch line or NOT SAVED/FIRST RECORD pair changed — confirmed by the comparison; (6) HA-187 (2)'s blast radius;
  (7) `-Wformat-truncation` — fixed (A2b.1 ruling 3, mutant X21); A2a fix-2 finding 18 — chapter 19 named in
  `hal/pros/calibrate.hpp`.
- **No new hardware belief.** HA-187 is the brief's own; nothing found here needed another.

**For the coordinator:**
1. **Shared files this part touches (only its own lines):** `docs/changelog.md` (A2a's entry under `## API 2.5`
   :19–172, extended), `mkdocs.yml` (one nav line after guide 18's), `docs/guide/README.md` (row 19, the maintenance
   note), `docs/hardware-assumptions.md` (:39 Held line, :44 status, :170–176, :387 the index row, the new section
   before Group R4), `docs/roadmap.md` (a "Now (2026-10-30)" paragraph after A2a's), `docs/internal/build-order.md`
   (`Next:` line's CAL1 clause, CAL1's status block, :1690), `REMAINING-WORK.md` row 18b, `PROJECT-BRIEFING.md`
   (generated), `docs/api/` (generated). **No file another in-flight chunk owns was edited** — not the shim
   (VIS1/IMU1's `shim_control.hpp`, `imu.hpp`), not `config/robot_config.hpp` (VIS1), not `src/`, not `tools/`.
2. **Merge order:** IMU1 Part A (`6798aa5`, `hal/pros/imu.hpp`, the shim's `imu.hpp`) is on main after this base. The
   golden test runs the REAL `ProsImu` over the shim, so after the merge re-run `-tc='golden*'`: a change in what the
   IMU adapter answers within one tick would move the goldens' timing lines (reasoned unlikely: the shim's IMU value
   changes only between 10 ms steps). If they move, compare with `SHULIB_GOLDEN_OUT` and classify before copying.
   VIS1 (in flight) edits `config/robot_config.hpp`: the team file test's `kRobot` names `.identity` and
   `.drivetrain` by designated initializer — still valid as long as their order holds.
3. **"BUILT … lands at the merge"** is written in four places (build-order :1643 block and the `Next:` clause, the
   roadmap's "Now (2026-10-30)", REMAINING-WORK 18b) and the changelog heading says "its typed drive ports built
   2026-10-30": make them LANDED with the merge's date, and regenerate the briefing (its "Next up" copies `Next:`).
4. **Guide chapter 19** is the next free number at this base; if anything lands a 19 first, renumber (the chapter,
   the guide README row, `mkdocs.yml`, `hal/pros/calibrate.hpp`'s sentence, the changelog's and roadmap's links).
5. **For Part B:** chapter 19's dated status line ("As of 2026-10-30 no program in this repository runs the standard
   Calibrate yet; robot two's own … moves onto it in CAL1's next part") — not in the brief's §3.2 list; reword it
   when robot two moves on. The golden test's facts are written literally; Part B's §2.B says the goldens are the
   end state's and must stay unchanged when its facts come from the tables.
6. **The brief's proposed Re-check note R19** (A2a's) still waits for you; A2b proposes none. **The goldens are
   1.3 MB** of text (102 files) under `test/goldens/calibrate/`.
7. Scratch: `scratch/cal1-a2b/` (`gold1/`, `cmp/`, `mut/`, `gates/`, `team_proj/` — deletable after the merge).
   **Nothing committed, nothing pushed, no `pros`, no serial port.**

**Where work stopped:** Part A2b is complete at 09:59 EDT 2026-10-30 — every item built, tested, mutated and gated;
the next step is the coordinator's review and merge.

## CAL1-A2b review findings and their disposition (2026-10-30 14:50 EDT, `date`)

> Executor: **Claude Opus 5.5** (subagent of the coordinator's workflow run), the same worktree and branch
> (`cal1-a2b`, from `e5a69b5`). Twenty-one findings from three reviewers (correctness, tests, docs), deduplicated
> below to eighteen.
> Nothing committed, nothing pushed, no `pros`, no serial port; scratch and backups under
> `scratch/cal1-a2b/fix1/`.

### Resumption of this fix round (2026-10-30 14:48 EDT, `date`)

- **Why.** A first fix run for these findings was STOPPED by the account's 5-hour usage limit late in the morning of
  2026-10-30; this run resumes it ~14:48.
- **State found** (`git status`, `git diff -- src/ Makefile`, file times): no file in the worktree (outside the build
  trees `build/`, `bin/`, `.d/`) is newer than A2b.8's close (09:59) — the stopped run wrote NOTHING to the worktree
  and no disposition to this log. `git diff --stat e5a69b5 -- src/ tools/ Makefile test/CMakeLists.txt test/pros_shim
  .github`: EMPTY (no gate self-test plant). **No live mutant:** every backup under `scratch/cal1-a2b/mut/bak/`
  (57 files) has the md5 of its file as it stands — except `calibrate_program.hpp`'s 46, which differ from the file
  ONLY by the 09:52 THE LIMIT comment (A2b.7's last edit; `diff`: lines 66–68, a comment). The reviewers' trees are
  their own copies (`review-*/tree`, `review-tests/r2/mergetree`), not this worktree.
- **The binary on disk** (`build/test/shulib_tests`, 09:53) is not trusted: it is removed and relinked from source
  before any result below.

### The findings, each re-checked, and what was done (2026-10-30 14:50–15:40)

Twenty-one findings arrived; three pairs say the same thing (2 = 14's first half; 4 = 7; 5 = 11), so eighteen
distinct items. Each was re-checked on the tree before anything changed; **every one was REAL** — none is
dispositioned "not a defect". Numbers below are the findings' order in the review.

**1. The golden test fails with a long temporary directory — macOS's default (correctness, minor — REAL).**
Re-checked: `TMPDIR=<139 characters>` on the relinked binary, `golden one`: `REQUIRE( 188 <= 95 )` … "the temporary
directory's path is too long" — 1 failed (`fix1/logs`, the first probe). macOS's TMPDIR
(`/var/folders/xx/<28>/T/`, 49 characters) + `shulib_calgold_<5-digit pid>/card/` + the 21-character file name =
96 > 95: every scenario fails there, from the environment, not the code. **Fixed at the root**
(`test/calibrate_golden_test.cpp`, `workDir()`): the work directory is made ONCE per process by `mkdtemp` (unique,
so two suites side by side never share it — the pid name was only probably unique), named `scg-XXXXXX`, under this
machine's temporary directory when the card root fits there, else under `/tmp`, whose length no environment variable
sets; `kMaxCardRoot` (74 = 95 − 21, `static_assert`ed) is the one limit, used by both the choice and the guard.
The guard `REQUIRE` stays (and a new one names a work directory that could not be made). macOS's case now fits in
TMPDIR itself: 49 + `scg-XXXXXX` + `/card/` = 65 ≤ 74. **Measured:** the same 139-character TMPDIR → all 38 golden
cases pass (the root fell back to `/tmp`); a 72-character TMPDIR (89 > 74) → falls back too, and passes; TMPDIR
unset (this machine, CI) → `/tmp/scg-…`; no `scg-*` directory is left behind after any run (`ls`). The goldens are
unchanged (the root is normalized away). **Alternative not taken:** `chdir` into the work directory with a relative
root `card/` — process-global state for the whole suite, and "card/" as the normalization key would also match
other text.

**2 + 14 (first half). HA-187's Contained line said "no other program reads a motor this way" — FALSE (correctness,
docs; minor — REAL).** Re-checked: `src/programs/bench_r3a.cpp` :975/:977 (`motor_get_position(raw)`,
`motor_get_gearing(raw)`, the raw motor report) and :1135/:1160/:1184 (the motor watch's position reads) — Bench
Tests reads motors by port through the same C calls; the entry's own settle (1) calls them "Bench Tests' raw
probes". **Fixed** (`docs/hardware-assumptions.md`, the entry's Contained): the brief's sentence kept, then "In
Calibrate the read is one function (`readShaftDegrees`). It is not the only reader: Bench Tests' raw motor report and
its motor watch read motors by port through the same C calls …, so a getter that wrote would touch their reads too —
still with no motion and nothing commanded." The blast radius is now stated, not understated.

**14 (second half) + 20. Edits to the brief's verbatim text, and two deltas, not logged as rulings (docs, minor —
REAL).** Logged here, each with its alternative (A2b.0–A2b.8 are history and are not rewritten):
- *Ruling 4 — HA-187's text, four edits to §3.3's copy.* (a) the Source line drops the brief's internal
  `brief-CAL1 amend/objdump/calls.txt` (a scratch path: `check-removability` refuses a public document naming
  internal material, and the file is not in the repository) and names the shell's function and the host test
  instead; (b) Claim (2) adds "(the push's own test for a drive port, HA-136)" — the 5° threshold's own register
  entry; (c) blast radius (2)'s "is the draft's state (no watch)" → "is the state before the watch existed" — "the
  draft" means nothing to a register reader; (d) hand-on (6)'s blast-radius paragraph and Station G's gearing read
  (A2b.4), and now this round's Contained sentence. **Alternative:** copy §3.3 verbatim — a removability failure
  and two sentences that point at nothing.
- *Ruling 5 — the typed cases' ports written literally.* The brief's §2.A test 2 gives the typed cases' ports
  "through `withTypedDriveFrom` over robot two's table made VALID"; `withTypedDriveFrom` is Part B's (§1.6) and does
  not exist at A2b's tree, and robot two's table is G3a's to flip (D7). So T7*, T6* and the golden typed scenarios
  write the brief probe's case-8 ports literally (`LEFT -11 +12 -13 +14 -15 | RIGHT +20 -19 +18 -17 +16`, the test
  headers say so). **Alternative:** a local stand-in for `withTypedDriveFrom` in the test — Part B's function
  written early, untested against the real one, and the test's ports read from `src/` (which G3a will change).
  Part B's §2.B re-derives the golden facts from the tables and requires the goldens unchanged (A2b.8 item 5).
- *Ruling 6 — the Held line.* The brief's §3.1 text is "HA-187 (registered by CAL1); HA-188 released unused — never
  reissued"; A2b first wrote "HA-188 (CAL1's second number, released unused when CAL1 registered HA-187 — never
  reissued)". Both put the token `HA-187` in the line `briefing_status.py`'s `held_numbers()` reads as HELD numbers,
  so a REGISTERED number read as held (harmless today — only `next_free` uses the set — but the line no longer meant
  its label). **Now:** "HA-188 (CAL1's second number, released unused — never reissued)"; `held_numbers()` on the
  new line: 187 gone, 188 present, next free still HA-216 (the briefing's check, below). **Alternative:** the
  brief's text verbatim — the same parser problem.

**3. A C4 edge untested: only pushed ports the team did not type (correctness, minor — REAL).** Re-checked: no test
has `onlyNow` as the only difference (T7b and `typed-partial` combine it with others). **Fixed:** T7b-3 — the typed
lists one motor short (`LEFT -11 +12 -13 +14 | RIGHT +20 -19 +18 -17 +16`): the counts line ("9 agree, … 0 in the
typed ports only, 1 in this run's only: SAME FRONT"), the header "differ at 1 port(s)", the one line "  port 15: this
push LEFT -15, not in your typed drive ports", the SAVED line next, the PARTIAL note drawn and the exact one NOT.
Mutant R15 RED (below).

**4 + 7. T7k never checked the typed FRONT FLIPPED words, on the serial or the screen (correctness, tests; major —
REAL).** Re-checked: T7k checked the touch line, FIX and the mechanism screen only; no golden pairs a typed flip with a
turned claw. **Fixed:** T7k now checks the five hold lines (`kTypedFlip`) right after the verdict, no "the card's
record calls", the `screen:` line in the typed words, the three typed draws, and none of the card's three. Mutants
R14 (`typedPorts` false with a question to come) and T05 (the hold in the CARD's words with a question to come) RED.

**5 + 11. The typed facts' slot edges: the last slot alone, and a duplicate in ADJACENT slots (correctness, tests;
minor — REAL).** Re-checked: `lastSlot` is decided at slot 0, T6g's same-side duplicate pairs slots 0 and 4.
**Fixed:** T6i gains seven `static_assert`s (the last slot alone on both sides — typed, usable, described; on the
left only and on the right only — refused naming the other side; an adjacent duplicate on each side — refused at
entry 1); T6g a run-time case (`{-11, 11, -13, 14, -15}` → "…: typedDrive.left[1] = 11"); T6h-2 a program run with
ONLY slot 20 typed on both sides (the typed list `LEFT -11 | RIGHT +20`, "2 agree … 8 in this run's only", the
partial NOTE, never FIRST RECORD). Mutants T04 and T06 RED at compile time (the TEST's assertions) and at run time.

**6. A record recovered from the BACKUP, with typed ports present, untested (tests, major — REAL).** Re-checked: T7g
and `typed-card-wins` seed both files good; A2a's T2i (the backup case) has no typed ports. **Fixed:** T7g-2 — the
record file `garbage(180)`, the backup `frontLedRecord()`, typed ports flipped against it — the whole run EQUALS the
untyped run (every line, draw and byte), no "typed drive ports" line, the BACKUP's check SAME, no FRONT FLIPPED.
Mutant T07 RED. (Not added as a golden: the directory holds exactly the brief's 34 scenarios, and the rig's
equality is the same proof.)

**8. The MECHANISM evidence against the typed ports, tested on the LEFT side only (tests, minor — REAL).**
**Fixed:** T7j-2 declares port 18 (typed RIGHT +18): its per-port line, the MECHANISM line, "Your typed drive ports
DRIVE with port 18: saving now makes a record with NO port 18.", the screen's `your typed ports DRIVE with port 18`
and never "it is declared a mechanism: not in this record", the disable's words, no file. Mutant T08 RED.

**9. The per-port lines' bounds, ports 1 and 21, untested (tests, minor — REAL).** **Fixed:** T7b-4 types 1 (left)
and 21 (right) besides the pushed ten: "differ at 2 port(s)", then exactly the port 1 and port 21 lines, then SAVED.
Mutants T01 (`p < 21`) and T02 (`p = 2`) RED.

**10. Ruling 2 (typed NOTEs kept under NOT SAVED) pinned by no test (tests, minor — REAL).** **Fixed:** T7l — A2a's
T4c fault (a short write, 30 bytes, at the record file's first replace, found by a dry run whose NOTE is REQUIRED
drawn) on two typed runs: every port agreeing (FRONT MATCHES YOUR TYPED DRIVE PORTS) and flipped then USE NEW (FIX
YOUR TYPED DRIVE PORTS) — each NOT SAVED (the save's own line, the header, its red draw; never SAVED / SAVED -- NEW
FRONT), the typed NOTE drawn, never FIRST RECORD. Mutant T03 RED.

**12. Both checks on `runCalibrate()` tested presence only (tests, minor — REAL).** **Fixed:** the golden test's
source pin compares `runCalibrate()`'s body EXACTLY (its nine lines, the five, the program, ONE run); the team file
test counts the program's banner and `[R3D] REFUSED: ` exactly once each and requires the refusal's fix line to be
the run's last. Mutant D01 (`program.run()` twice) RED — by the pin and by the team file test.

**13. Chapter 19 misstated the backup after SAVED (docs, major — REAL).** Re-checked: `record_store.hpp` :35 (step 3
`put(K, bytes)` refreshes the backup); `test/goldens/calibrate/flip-new.card.txt` (both files identical after USE
NEW); the FAQ :163 says so. **Fixed** (:39-40 and "What the ending means"): three steps, each read back; after
SAVED the backup holds the NEW record too (the log's "The backup … re-reads identical") — copying it back cannot
undo a USE NEW after a wrong push; only a step that fails leaves the earlier record in one file, and the log says
which (a SAVED whose backup refresh failed: "The backup … does NOT hold this record -- it holds …", `record_store.hpp`
:813 — so the chapter does not claim "both files" unconditionally); "never erases the good record" is now
"designed so that one failure part-way never erases … rests on beliefs … no robot has tested yet (HA-166)".

**15. "ADDITIONS ONLY" over a non-additive panel line; today's program not re-measured on it (docs, minor — REAL).**
**Fixed:** `calibrate_panel.hpp`'s A2b header bullet says "additions, plus ONE behaviour-preserving edit" and names
it (no internal path — the first wording named this log and `check-removability` refused it, as it should;
`docs/api/` regenerated). **Measured** (`fix1/p33/`, A2a's method — `build.sh`, `run.sh`, `compare.out`):
`git archive e5a69b5`, TODAY's `src/programs/calibrate_program.cpp` built with the brief's harness twice — over
e5a69b5's panel (`old_h`) and over A2b's final panel (`new_h`, md5 `eb353d8e…` = the worktree's) — the nine
scenarios from an empty card, capped: **33 of 33 files identical** (32 byte-identical; `nocard.serial.log` after
normalizing the card directory's name in its one NOT SAVED line, exactly A2a's). The old panel's build printed
GCC's `-Wformat-truncation` (`calibrate_panel.hpp:753`) and the new one's did not — hand-on (7) seen once more at
`-O1`. The changelog's "paints, prints and saves exactly as before" is now measured.

**16. The prose review skipped the brief's terms, the changed files' names and the worksheet (docs, minor — REAL).**
Done and logged below ("Prose review"). The stale comment the reviewer found — `test/pros_calibrate_adapter_test.cpp`
:7, "the one CAL1 registers as HA-187 (in Part A2b)" — is in an existing test file A2b may not edit (§2.A test 9):
**handed to Part B** (below).

**17. FIRST RECORD from the wrong end — the recovery was incomplete (docs, minor — REAL).** **Fixed** in chapter 19
and guide 12: RUN AGAIN, push with the other end leading; Calibrate then shows FRONT FLIPPED against the record just
saved — touch USE NEW (changing the front back on purpose), not RUN AGAIN, which keeps the reversed record (the
brief's Station G sentence, in the team's terms).

**18. Chapter 19 omitted the typed-list refusals (docs, minor — REAL).** **Fixed:** one bullet in "What you get
wrong": a port outside 1..21, a port listed twice (one side, or once on each — `-11` and `11` are one port), or one
side typed → FACTS REFUSED naming the entry and value, e.g. `typedDrive.right[4] = 11` (T6g's own message).

**19. Screen and log quotes with '—' where the brain prints '--', and a misquoted serial line (docs, minor — REAL).**
**Fixed:** chapter 19 (`FRONT NOT CHECKED -- see the log`, `SAVED -- NEW FRONT`, `FIRST RECORD -- …`, `FRONT
MATCHES, SOME TYPED PORTS DIFFER -- see log`) and guide 12 (the four NOTE rows); the pulse-refusal sentence now
quotes the LEFT OUT line's "(a slipping gear? or a MECHANISM motor -- declare it in the robot's facts)" and the
refusal's own line `if port N is a MECHANISM motor, declare it in the robot's facts: a motor that is not declared is
a drive candidate` (`calibrate_program.hpp` :1646, :1669). `grep` over the docs A2b touched for a quoted literal with
an em dash (`NOT CHECKED —`, `FIRST RECORD —`, `SAVED — NEW`, `DIFFER —`, `DRIVE PORTS —`): none left (two hits
elsewhere are prose, not quotes).

**21. Hand-off gaps (docs, minor — REAL).** Added to "For the coordinator" and "For Part B" below: (a) main's register
is at 182 (`git show shulib-v2:docs/hardware-assumptions.md` :44, "Status: 7 of 182"; IMU1 "With them, 181" :178,
MECH-A5 "With it, 182" :183, both after e5a69b5) — A2b's "7 of 179" / "With it, 179." become **183** at the merge,
the status sentences in landing order (IMU1, MECH-A5, then CAL1 A2b), and main's Held line still reads
`HA-187..HA-188`; (b) HA-187's two Station G settle steps are in no worksheet step; (c) the goldens' path.

### Prose review (pace rule 4) — the brief's §3.1 terms and the changed files' names, grepped, every hit READ (2026-10-30 15:00–15:10)

**Grepped** (GNU `command grep -F`) over `README.md`, `docs/*.md`, `docs/guide/*.md`, `docs/cookbook/*.md`, the
living internal documents (build-order, HANDOFF-remaining-work, ORIENTATION, REMAINING-WORK, PROJECT-BRIEFING,
RESUMING) and **the worksheet** (`R3a-BENCH-WORKSHEET.md`): the brief's §3.1 list — `shulib Calibrate`,
`calibrate_program`, `viewFor`, `TouchGate`, `IBrainScreen`, `screenSafeLine`, `printText`, `record_store`,
`saveRecordSafely`, the front (`front check`, `FRONT FLIPPED`, `WHICH END` — A2a's reading of "front") — and every
file A2b changed by name: `calibrate_program`, `calibrate_panel`, `hal/pros/calibrate`, `pros-calibrate`,
`calibrate_golden`, `calibrate_team_file`, `calibrate_program_typed`, `shulib_calgold`, `temp_directory`. 139 hits
outside chapter 19 and guide 12 (those two were read whole for findings 13 and 17–19) — `fix1/logs/prose_hits.txt`.
**Read, and true after A2b** (A2b changes no program: every line about robot two's "shulib Calibrate" describes
`src/programs/calibrate_program.cpp`, unchanged, and is Part B's to reword as §3.2 lists): README :279-280 (the
standard Calibrate and its shell), :322 (the program list); the FAQ :161 (the safe save's three steps — true, and
what chapter 19 now says), :176 ("until shulib Calibrate saves through it" — robot two's program), :253;
`hardware-assumptions.md` :116, :654, :1344, :1633/:1645 (the panel's screen model — A2b added literals inside its
budget check, no budget moved), :1738, :1760, :2012, :2054, :2121, :2329, :2443-2505 (SAFE-SAVE: "does not use it
until Part B1" — robot two's), :2670, :2700, :2910, :3026, :3184-3213 (HA-187 — A2b's, fixed above), :3329, :4029,
:4190, :4219; roadmap :245, :293, :401, :414, :421 (A2b's paragraph), :476, :485, :1231, :1266, :1328; the master
plan :274; guide 07 :143, guide 16 :145, guide 17 :346, the guide README :27; HANDOFF :119, :274-280 ("no program
builds it yet" — still true), :307, :582, :731; ORIENTATION :85, :104; REMAINING-WORK :101; the briefing :110 (its
"Next up" copies build-order's pointer); the changelog — A2b's extended entry :19-171 (its :81 "a verified backup
first … never erases a good one" is A2a's merged text, true of the order; history, not rewritten), the 2.4 seams
entry :418-478, SAFE-SAVE's :936-986, and the older :656, :865, :1535, :1728, :1779-1850 (history); build-order
:632-642 (pointers and history), :1506, :1551, :1647-1686 (CAL1's block — A2b's lines and A2a's), :1855,
:2160-2164, :2334-2338; the worksheet :22, :27, :452, :492-540, :624, :722-865, :1030, :1354, :1862 — robot two's
program and Station G, all Part B's rewrite (§3.2), none made false by A2b. The test files' names occur only in
chapter 19 (:74, its link), the changelog (:163, :168) and HA-187 (:3213) — true. `shulib_calgold` and
`temp_directory` occur in no public document (one internal hit, IMU1's brief :78, about OTHER tests' fixed temp
paths — the mkdtemp directory here is unique per process, so it adds no such collision).
**Fixed this round:** chapter 19 and guide 12 (findings 13, 17, 18, 19), HA-187 and the Held line (2/14, 20),
`calibrate_panel.hpp`'s header bullet (15; `docs/api/` regenerated).
**Not touched, with reasons:** `test/pros_calibrate_adapter_test.cpp` :7 (an existing test A2b may not edit —
handed to Part B, below); the FAQ, README, guide 07, the worksheet (true; Part B's); the changelog's A2a sentences
(history).

### For the coordinator and for Part B (added by this round; A2b.8's list stands)

**For the coordinator (the merge):**
1. **The register count** (finding 21a): main is at **"Status: 7 of 182"** — IMU1 (+3, "With them, 181") and
   MECH-A5 (+1, "With it, 182") landed after `e5a69b5` and inserted their status sentences where A2b inserted its
   own. At the merge A2b's "Status: 7 of 179" and "With it, 179." become **183**, its sentence AFTER MECH-A5's
   (landing order), and main's Held line's `HA-187..HA-188` becomes this branch's `HA-188 (CAL1's second number,
   released unused — never reissued)` (ruling 6: no registered number in the held line). The next free number stays
   HA-216 (check it with `briefing_status.py check` on the merged tree).
2. **The goldens after IMU1** (A2b.8 item 2) still apply; nothing here changed a golden (the card root is
   normalized away).

**For Part B:**
1. **HA-187's settle at Station G** (finding 21b): the register's index row and entry name Station G, but no
   worksheet step does it and §3.2's Station G list does not name it. Part B's Station G rewrite adds both steps:
   (2)+(3) with the claw pivots declared, turn ONE by hand during the push — the watch names that port and only it,
   the save waits on MECHANISM PORT TURNED, the other pivot has not moved; and read an unconfigured claw's
   `motor_get_gearing` before any program gears it (it sets the evidence line's scale — HA-187 (2)'s blast radius).
2. **The goldens' path** (finding 21c): A2b's goldens are `test/goldens/calibrate/` (plural — the brief's §2.B
   side-by-side proof says `test/golden/calibrate/old/`, singular). The golden test REQUIRES
   `test/goldens/calibrate/` to hold exactly the 34 × 3 files, so an `old/` directory placed UNDER it fails that
   case. Put the old program's goldens beside it (e.g. `test/goldens/calibrate-old/`), or extend the directory test
   — and delete them with the old program, as the brief says.
3. **A stale comment in an existing test** (finding 16): `test/pros_calibrate_adapter_test.cpp` :7 still says "the
   one CAL1 registers as HA-187 (in Part A2b)" — now false (HA-187 is registered). Part B re-pins tests; reword it
   there ("the belief is HA-187 in docs/hardware-assumptions.md", the headers' words).

### Mutation pass — the reviewers' surviving mutants, re-cut on this tree, each on a FRESH binary (2026-10-30 15:02–15:09)

**Method** (`scratch/cal1-a2b/fix1/mut/`: `make_list.py` → `pass1.json`, `pass2.json`; `run_mut.py` — A2b.5's
runner, its backups under `fix1/mut/bak/`, never /tmp; outputs `pass1.out`, `pass2.out`): 14 mutations, every
anchor checked to occur EXACTLY once first (0 bad); each built with `shulib_tests/fast`, REQUIRING the link and
every object whose depfile names the mutated file rebuilt after the edit (5 for the program header, 3 for the
shell); the WHOLE suite capped (RLIMIT_AS 4 GiB, 600 s) — no mutant was killed by the cap or the timeout; each file
restored in `finally`, `cmp`-identical, its timestamp bumped; after both passes `md5sum -c` of the touched files
against `fix1/mut/pre_md5.txt`: **OK, 3 of 3**. A compile-time catch counts only where the compiler's errors are the
TEST's `static_assert`s (printed — `pass1.out`); each such mutant was ALSO run with T6i's `static_assert`s made
`CHECK`s (A2b.5's `#define`, in the mutant's own edit list and restored with it) to show the run-time catch. Before
this round's tests, every one of these survived the reviewers' whole-suite runs (2,757 / 2,757 green).

| # | Finding | The mutant | Verdict | Caught by |
|---|---|---|---|---|
| R15 | 3 | C4's exact NOTE when `differing == onlyNow` | RED | T7b-3 (5 checks), T6h-2 |
| R14 | 4 | `typedPorts` false while a MECHANISM question is to come | RED | T7k (the `screen:` line, the three typed draws, the card's three) |
| T05 | 7 | the typed hold in the CARD's words when a question is to come | RED | T7k (the five hold lines, "the card's record calls") |
| T04 | 5/11 | `hasTypedDrive` never reads slot 20 | RED — **compile time, by the TEST's assertions** (`calibrate_program_typed_test.cpp` :888, :894–:895, :899–:900: `lastOnly`, `lastLeftOnly`, `lastRightOnly`); at run time T6h-2 (no typed check printed) and T6i |
| T06 | 11 | the same-side duplicate scan starts two slots on | RED — **compile time, by the TEST's assertions** (:904 `adjacent`, :907 `adjacentRight`); at run time T6g (the adjacent case runs instead of refusing: devices asked for, motors commanded) and T6i |
| T07 | 6 | typed ports take over whenever the RECORD FILE does not decode | RED | T7g-2 (the typed ports, flipped against the push, become the reference, so the typed run HOLDS on FRONT FLIPPED, which its scripted operator never answers — `REQUIRE_FALSE(typed.timedOut)`, the rig's own deadline, not the cap) |
| T08 | 8 | the MECHANISM evidence judges the LEFT side only | RED | T7j-2 (the evidence line, the screen's line, "not in this record" drawn) |
| T01 | 9 | the per-port lines stop at port 20 | RED | T7b-4 |
| T02 | 9 | the per-port lines start at port 2 | RED | T7b-4 |
| T03 | 10 | the typed NOTEs dropped under NOT SAVED (ruling 2 reversed) | RED | T7l |
| D01 | 12 | `runCalibrate()` runs the program twice | RED | the golden test's exact source pin; the team file test (banner and REFUSED once each) |
| X22r | (A2b.5's X22) | `runCalibrate()`'s card given a root — re-run because its catch, the source pin, was rewritten | RED | the exact source pin |

**14 of 14 RED** (two of them at compile time by the tests' own assertions, and RED at run time too). The finding-1
fix is shown by the TMPDIR probes above (it is the test's own environment, not library logic); finding 15's by the
33-of-33 measurement.

### Second resumption of this fix round, after the team lead's pause (2026-11-03 12:49 EST, `date`)

- **Why.** The team lead paused all work at ~15:40 on 2026-10-30 (he travelled). The run above was stopped after
  its mutation pass, before it logged its gates or closed the round. The laptop restarted on 2026-11-03; this run
  resumed at 12:49 EST (`date`; EST since 2026-11-01). Scratch under `scratch/cal1-a2b/fix1r/`.
- **State found** (`git status`, `git diff -- src/ Makefile`, md5s, file times):
  - the worktree holds A2b's changes only (18 modified files; 6 untracked: chapter 19, the golden rig and test, the
    typed test, the team file test, `test/goldens/`). `git diff --stat e5a69b5 -- src/ tools/ Makefile
    test/CMakeLists.txt test/pros_shim .github`: EMPTY.
  - **No live mutant:** `md5sum -c fix1/mut/pre_md5.txt` OK, 3 of 3 (the program header `f4cd06a2…`, the shell
    `401c7982…`, the typed test `5e060af0…`), and each of the 16 backups under `fix1/mut/bak/` has the md5 of its
    file as it stands.
  - **No gate plant:** the stopped run's src gate `self-test` and `check` had FINISHED (10-30 15:17–15:35,
    `fix1/gates/times.txt`), and its after-snapshot (1,116 files under `src/ include/ test/ tools/` and the
    `Makefile`) matches the tree now (`md5sum -c --quiet`: all OK). The files newer than this log (`Makefile`,
    `src/main.cpp`, `test/CMakeLists.txt`, …) are newer only by their restore's timestamp bump: `git status` lists
    none of them.
  - **What the stopped run did after its mutation pass and did not log:** the gates, 10-30 15:10–15:35
    (`fix1/gates/`: the suite green with 2,763 cases; the src gate's `self-test` OK and `check`; ARM over 228
    headers; api_doc_tool; the staleness audit; the briefing), and a rebuild of the team project at 15:04. The tree
    has not changed since (above). Those results are NOT cited here: every gate is re-run below, on a fresh binary.
  - **main has moved** since the base (IMU1 Part A `6798aa5`, MECH-A5 `81eb666`, the panel plan `eab0f89`; VIS1-A1
    staged in main's tree). Of the files A2b changes, main's commits touch only shared documents
    (`git diff --name-only e5a69b5 shulib-v2` against A2b's list): the changelog, the register, build-order, the
    roadmap, REMAINING-WORK, the briefing and the two generated `docs/api/` indexes. In each, A2b changed only its own
    lines. VIS1-A1's staged change also touches `mkdocs.yml`. The merge reconciles all of these (see "For the
    coordinator", below).
- **The binary** (`build/test/shulib_tests`, 2026-10-30 15:11) was removed and relinked from source (below) before
  any result.

### Re-verification of the logged dispositions (2026-11-03 12:52–13:00)

- **Each finding's fix was re-read in the tree**, not taken from the log above:
  - (1) `workDir()`'s `mkdtemp` and `kMaxCardRoot` (the golden test :319–354, the two `REQUIRE`s :394–401).
  - (2) HA-187's Contained ("It is not the only reader", the register :3240).
  - (3) T7b-3. (4 and 7) T7k's five hold lines, its typed draws, and the absence of the card's three.
  - (5 and 11) T6i's new `static_assert`s (the typed test :888–907), T6g's adjacent case and T6h-2.
  - (6) T7g-2. (8) T7j-2. (9) T7b-4. (10) T7l.
  - (12) the exact source pin (the golden test :608–633) and the team file test's counts.
  - (13) chapter 19 :39–40 and "What the ending means". (15) the panel's header bullet: a `diff` against A2b.5's
    backup shows the bullet only, so no code line changed in this round, in any of the three headers.
  - (17) chapter 19's FIRST RECORD paragraph and guide 12's row. (18) chapter 19's typed refusal bullet. (19) the
    `--` quotes. (20) rulings 4–6 and the Held line (the register :39). (21) the hand-ons above.
- **The mutation pass above was not re-run.** Its outputs (`fix1/mut/pass1.out`, `pass2.out`) already show every
  mutant on a FRESH binary:
  - each run-time mutant carries the runner's own line "fresh binary: linked, every dependent object rebuilt (5: … /
    3: …)";
  - each compile-time mutant shows the compiler's errors, which are the TEST's `static_assert`s (the typed test
    :888–:907);
  - each was observed RED and restored `cmp`-identical.

  The files they mutated, and the tests that caught them, are byte-identical to the files those runs used
  (`pre_md5.txt` OK; the golden test and the team file test last changed at 14:54, before the pass began at 15:02).
- **Finding 1's probes, re-run on this round's fresh binary** (`fix1r/probe/`, capped, `strace -f -e
  trace=mkdir,mkdirat`):
  - TMPDIR set to a 146-character directory: `golden*` **38 of 38 passed**. `mkdtemp` made `/tmp/scg-NVjoYy`, and
    nothing was made under the long TMPDIR, so the fallback was taken.
  - TMPDIR unset: 38 of 38, made under `/tmp` (`/tmp/scg-yew8dw`), so the branch where the root fits was taken.
  - No `scg-*` directory was left after either run, nor after the whole suite (`ls`).

  A TMPDIR short enough to hold the root cannot be made under this run's scratch directory, which is itself 65
  characters (65 + 17 > 74). macOS's 49-character TMPDIR is shown by the arithmetic: 49 + `scg-XXXXXX` +
  `/card/` = 65 ≤ 74.
- **Finding 15's measurement still holds:** its overlay panel (`fix1/p33/overlay/…/calibrate_panel.hpp`) has md5
  `eb353d8e…`, the same as the panel as it stands.
- **The team file as a PROS project, rebuilt** (`team_proj/`: the three headers `cmp`-identical to the worktree's,
  `src/main.cpp` `cmp`-identical to the example test's first 21 lines; `bin/main.cpp.o` and the hot package removed
  first, capped):
  - `make` exit 0, with ONE warning (liblvgl's own);
  - `bin/hot.package.bin` is **85,108 bytes**, text 83.11 KB; its strings carry the typed words (21 hits).
  - A2b.7 measured 85,104 bytes. The four bytes are the build-hash string, which this scratch project's Makefile
    takes from `git describe --dirty` of the repository around it (`v0.1.1-400-geab0f89-dirty` now). No code
    changed.

### The gates — every one re-run on this round's final tree (2026-11-03 12:52–13:25 EST; outputs `fix1r/`)

- **The host build.** First the binary was removed, `cmake -S test -B build/test` run, and the briefing regenerated
  with no binary. Then the full `cmake --build build/test -j8`, capped: exit 0, **245 objects compiled** (a full
  rebuild), **0 warnings**, linked 12:53:09. Its doc-gate step reported:
  - api_doc_tool self-test OK; coverage 5,078 entities across 188 headers; 623 quoted lines verbatim;
  - removability clean;
  - the self-tests of `suite_run.py` (41 cases), `briefing_status.py` (38) and the staleness audit (16) OK.

  The briefing was regenerated again once the binary existed.
- **The suite** (`(ulimit -v 4194304; timeout 600 python3 tools/suite_run.py green)`): **`suite run: green — 2763
  cases, 2937505 assertions`** (3 skipped, as on main; 12:53:45–12:54:12). Compared case by case with A2b.7's
  `gates/suite.xml` (the XML from the same binary, capped, `fix1r/logs/suite.xml`):
  - **0 gone, 0 failing.**
  - **6 new**, all in the typed test: T7b-3, T7b-4, T7g-2, T7j-2, T7l and T6h-2.
  - **39 changed**, each by a test this round changed:
    - the 34 golden scenarios, +1 assertion each (`runScenario()`'s new work-directory `REQUIRE`);
    - the source pin, 14 → 8 (one exact compare in place of six finds);
    - T7k, 21 → 35; T6g, 287 → 323; the team file test, 52 → 57;
    - the panel's P9, 5,089 → 5,093. P9 checks every line of `calibrate_panel.hpp` for `%%`, and the header bullet
      is four lines longer.
- **`briefing_status.py check`:** `generated block current` (suite 2,763 / 2,937,505; 228 headers; HA: 179
  registered, next free HA-216).
- **api_doc_tool** (each exit 0):
  - `self-test` OK; `check-coverage`: 5,078 public entities across 188 headers, all documented;
  - `check-fresh`: 0 lines;
  - `check-examples`: 623 quoted lines, all verbatim (10 source files), and 6 listings of whole-run files, each one
    unbroken run;
  - `check-removability`: no public doc references `docs/internal/`.
- **`doc_staleness_audit.py`:** `clean (32 live docs, 0 numeric claims checked)`, with its cases and assertions
  derived from the fresh binary.
- **The ARM header gate** (`fix1r/gates/arm_and_ci.sh`, its translation unit under `fix1r/gates/`): exit 0 over
  **228** headers, 0 output lines. Its control fails as it should (`-Werror=float-conversion`).
- **The CI greps** (as `.github/workflows/ci.yml` :60 and :76 spell them): `library is PROS-free (all of
  include/shulib except hal/pros/)`; `layering holds: core is sim-free`.
- **The compile-refusal harness** (`ctest -R compile_refusal`, capped): `100% tests passed, 0 tests failed out of
  251`.
- **`src_build_gate.py`** (`fix1r/gates/srcgate.sh`, 13:00:36–13:19:45):
  - `self-test`: `OK (235 detector cases, each against a real build)`, exit 0 (13:00–13:16).
  - `check`: `src build gate: PASS (all 7 builds link; …)`, exit 0 (13:16–13:19). Each of the seven builds: `exit 0, 11/11 TUs
    compiled, warnings: 8 vendor-ignored, 0 disallowed, hash asserted`.
  - **No plant left:** the md5 of every file under `src/ include/ test/ tools/` and of the `Makefile`, plus
    `git status --porcelain`, were taken before and after the two runs: **identical** (1,116 files). They are also identical to the
    stopped run's after-snapshot of 2026-10-30, file for file, with the same `git status`.
  - `git diff --stat e5a69b5 -- src/ Makefile tools/`: EMPTY.
- **The team project:** rebuilt above (`make` exit 0, 85,108 bytes).
- **Relinked after the src gate.** The src gate's restores bump file timestamps (`include/shulib/config/robot_config.hpp`
  and 8 more), so the staleness audit then distrusted the binary ("older than … so it was not trusted"), although
  the snapshot shows no content changed. So the binary was removed again, the briefing regenerated with none, the
  full build re-run (exit 0, 245 objects, 0 warnings), and the briefing regenerated. The suite was then re-run:
  **`suite run: green — 2763 cases, 2937505 assertions`** (13:22–13:24). No `scg-*` directory was left behind.
- **After this log's last edit,** the documentation gates were re-run (`check-fresh`, `check-examples`,
  `check-removability`, the staleness audit, `briefing_status.py check`), on the relinked binary. Each exited 0:
  `check-fresh` printed 0 lines; 623 quoted lines verbatim; removability clean; the audit `clean (32 live docs, …)`
  with its counts derived from the binary; the briefing `generated block current`.

### Prose review (pace rule 4) for this resumption

This resumption changed **no source file and no public document**. Its only edits are to this log, plus the
briefing's regeneration, which leaves it unchanged (`git diff`: the suite and HA lines as the stopped run left them).
The proof is the after-snapshot: every file under `src/ include/ test/ tools/` and the `Makefile` matches the
stopped run's (`md5sum -c`: all OK, above), and `git status` lists the same files. The prose review of the round is
the one logged above ("Prose review … 2026-10-30 15:00–15:10"), and nothing since has changed what it read.

### For the coordinator — updated by this resumption (items 1 and 2 above stand, except as said here)

1. **The register count has moved again.** Item 1 above says 183. That is right only for main at `eab0f89`
   ("Status: 7 of 182"). VIS1-A1 is staged in main's tree at **"Status: 7 of 194"** (twelve entries, HA-192..HA-203,
   its sentence "With them, 194." the last). So at the merge:
   - A2b's "Status: 7 of 179" and "With it, 179." become **main's count + 1**: 195 if VIS1-A1 lands first, 183 if
     it does not;
   - A2b's sentence goes after the last one landed;
   - the Held line still becomes A2b's `HA-188 (CAL1's second number, released unused — never reissued)`.
     VIS1-A1's staged Held line still lists `HA-192..HA-203` as held; that is VIS1's to settle, not A2b's.
   - The next free number stays HA-216. Check it with `briefing_status.py check` on the merged tree.
2. **`mkdocs.yml` is touched by both A2b and VIS1-A1's staged change**: A2b adds chapter 19's nav line, so keep
   both.
3. **The landing date.** Five places say A2b is BUILT on 2026-10-30 and that it lands at the merge:
   - `build-order.md` :632 (the `Next:` pointer — the briefing's "Next up" regenerates from it) and :1643;
   - the roadmap's "Now" :419;
   - `REMAINING-WORK.md` :101;
   - the changelog's 2.5 heading :19 ("its typed drive ports built 2026-10-30").

   At the merge, re-date them in A2a's form: `LANDED <merge date> (built 2026-10-30, reviewed and fixed
   2026-10-30–11-03)`. Then regenerate the briefing.

### This round's close (2026-11-03 13:24 EST, `date`)

- **Findings:** the review sent twenty-one; deduplicated, they are **eighteen**. Every one was re-checked and found
  **REAL**, and every one is dispositioned above:
  - fixed at the root, in the tests, the documents or the register;
  - or handed on with its reason: `test/pros_calibrate_adapter_test.cpp` :7 (an existing test A2b may not edit) and
    HA-187's Station G steps both go to Part B.

  None was dispositioned "not a defect".
- **Mutations:** the reviewers' surviving mutants were re-cut on this tree. **14 of 14 are RED** on fresh binaries:
  two at compile time by the tests' own `static_assert`s (and RED at run time too), and none killed by the cap or
  the timeout.
- **Gates:** every one is green on the final tree (above).
- **Scope:** `git diff --stat e5a69b5 -- src/` is EMPTY, so Part B's files are untouched.
- **Status:** nothing committed, nothing pushed, no `pros`, no serial port.
- **Next:** the coordinator's verification and merge (above), then Part B.

## The coordinator's merge verification — CAL1 Part A2b (2026-11-03 15:55–17:15, main `59d93bb` + the squash of `cal1-a2b`)
- **Audit before the merge:** `audit_wt.py cal1-a2b` — 5 backed-up files, 0 suspicious; no gate plant in `src/`, `tools/`
  or the Makefile (also grepped for a "(self-test plant" line); no stray root file. The transport commit `114caa0`, then
  `git merge --squash cal1-a2b` (base `e5a69b5`).
- **Seven conflicts:** `docs/api/README.md`, `all-entities.md` and the briefing (regenerated); the changelog — A2b EXTENDED
  A2a's 2.5 entry (hand-on 4), so the entry keeps its place under the four newer ones and its heading takes A2b's words,
  dated "its typed drive ports 2026-11-03, built 2026-10-30"; the register — the status **7 of 203** (main's 202 + HA-187;
  203 index rows, sorted, HA-187 present), A2b's "added ONE" paragraph last, "With it, 203"; the roadmap — A2b's "Now"
  paragraph after IMU1's, VIS1-A1's and L6-A1's, re-worded LANDED 2026-11-03; build-order's `Next:` line by a word-level
  three-way merge against `e5a69b5` (both sides' additions present). "BUILT … lands at the coordinator's merge" phrases made
  LANDED in build-order (the Next line and the CAL1 status) and REMAINING-WORK 18b.
- **No semantic conflict:** A2b edited no existing test file; `test/calibrate_program_rig.hpp` keeps main's
  `CountingProvider::aiVision()` (the VIS1-A1 merge fix).
- **Guide chapter 19** is A2b's ("Calibrating your robot"); G3a Part 1's robot-file chapter, merging next, was renumbered
  20 in its worktree before its merge.
- **Gates on the merged tree** (`.claude/coordinator/logs/cal1_a2b_merge_verify.out`): host build with the doc gates; suite
  **2,961 | 2,961 passed, 3 skipped** (the goldens re-run on IMU1's real `ProsImu`, as A2b asked); `src_build_gate.py`
  self-test OK and check **PASS, all 7 builds**; ARM header gate over **240** headers clean; CI greps empty; api
  check-fresh, the briefing, removability and the staleness audit clean; the compile-refusal harness **255 of 255**.
- **The coordinator's hand mutations** (`muts_cal1_a2b_main.json`; fresh link, 4 GiB / 300 s cap, restored and
  hash-checked): **5 of 5 caught** — CB3 (the turned port judged against the card when the typed ports were the
  reference) and CB5 (the typed ports win over a usable record on the card — D6 (a)'s order) RED; CB1 (a port typed on
  both sides not refused), CB2 (one side typed, the other empty, accepted) and CB4 (typed port 22 accepted) refused AT
  BUILD by `test/calibrate_program_typed_test.cpp`'s own `static_assert`s (:865, :873, :876, :895 — the test's
  assertions, confirmed by compiling each mutant, not the mutation's own breakage). Restored tree green.
- **Next for CAL1:** Part B (robot two onto the standard Calibrate; CAL1's facts mapping N9; the dated status line in
  chapter 19) — the auton library, ahead of the panel (the team lead's 2026-11-03 order).
