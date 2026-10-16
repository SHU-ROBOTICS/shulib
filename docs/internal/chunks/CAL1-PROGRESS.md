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
