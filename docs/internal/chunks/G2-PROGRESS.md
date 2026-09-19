# G2 — PROGRESS (live log)

**Chunk:** G2 — `PathRunner` + the command-id registry + the manifest → freeze F8
**Date:** 2026-09-18
**Tree at start:** `e9ba69f docs(internal): G2 brief -- the path runner, the command-id registry, the manifest, the F8 freeze`
**Executor model:** opus
**Brief:** [`G2-path-runner-and-registry.md`](G2-path-runner-and-registry.md)

> This log is appended in real time. It is the honest record of exactly how far the work got,
> every ruling taken beyond the brief, every mutation actually RUN with its OBSERVED message, the
> prose-review log, and the DoD with evidence.

---

## Order of work

1. This log; the clean-tree suite count (expect 1342 cases / 1,632,353 assertions at `fe8e270`).
2. `route.hpp` additions — `config::kMaxMarkerIdBytes = 24`, the G2 validity rules in `Route::valid()`
   (additive only) + test 13.
3. `include/shulib/runner/command_registry.hpp` + tests 1–2.
4. `include/shulib/runner/path_runner.hpp` + tests 3–10 — **test 3's `bearingTo` bit-identity FIRST**,
   it is the acceptance test.
5. `include/shulib/runner/manifest.hpp` + test 11.
6. The F8 pin (`test/f8_signature_pin_test.cpp`) + `version.hpp` + the Freeze Register row + the
   integration document's §4.1–4.3 schema (test 12 with the deliberate mutation).
7. All verification; the documentation pass; the prose review; the DoD with evidence; the final count.

---

## Log

### Step 0 — orientation (start)

- `git log --oneline -1` → `e9ba69f`; `git status --porcelain` → clean.
- Read `docs/internal/RESUMING.md` in full.

- **Clean-tree count CONFIRMED** from the binary built at the clean tree:
  `[doctest] test cases: 1342 | 1342 passed | 0 failed | 3 skipped` /
  `[doctest] assertions: 1632353 | 1632353 passed | 0 failed` — **green**.
- Note on the first `cmake --build`: it FAILED before the link, on the briefing staleness gate —
  creating this PROGRESS log made `G2` an "INTERRUPTED CHUNK" in the generated block. That is the
  doc-gate-before-link ordering the brief warns about, firing on this file's own creation. The
  briefing is regenerated as part of the documentation pass (and again LAST, after the final
  mutation is restored and the suite is green, so no mutant's numbers can be baked in).

### Required reading — done, in order

`RESUMING.md`; the brief §0–§8; `config/route.hpp`, `chassis/chassis.hpp` (verbs, lines 230–470),
`chassis/routine.hpp` (in full — `driveTo`/`face`/`bearingTo`, the error policy, `then()`'s
adaptation), `sequence/run_guard.hpp`, `manipulation/mechanism_outcome.hpp`,
`hal/telemetry_sink.hpp`, `hal/char_sink.hpp`, `hal/fake/fake_char_sink.hpp`,
`hal/fake/fake_telemetry_sink.hpp`, `diag/line_format.hpp`, `core/check.hpp`, `math/angle.hpp`,
`test/f6_signature_pin_test.cpp`, `test/routine_signature_pin_test.cpp`, `test/route_test.cpp`,
`test/motion_test_rig.hpp`, `test/chassis_routine_test.cpp`, `test/robot_builder_test.cpp`,
`include/shulib/version.hpp`, `docs/vexbuilder-integration.md` §4, `docs/legacy-command-vocabulary.md`
§3–5, `docs/roadmap.md` (F6/F8/F10 rows, M5, "You are here"), `docs/internal/build-order.md` (the G2
entry + the first `Next:` block), `.github/workflows/ci.yml`, `tools/api_doc_tool.py`'s header,
`test/README.md`.

**What the reading changed in the plan** (each recorded as a ruling below): `route_test.cpp`'s
`baseRoute()` already carries `reverse = true` on a `DriveTo`, and every marker id in the tree is
already lower-snake — so the G2 validity rules can be written at their honest width without
invalidating any route the tree builds today.

---

## ⚠ INTERRUPTED by the coordinator at 19:39 on 2026-09-18 — NOT verified, NOT committed

The team lead is rebooting the machine into another OS, so the coordinator stopped this executor
mid-chunk. Its last words: *"Now the `version.hpp` bump and test 13 in `route_test.cpp`."* What is
on disk (`git status`): `include/shulib/runner/` (new), `include/shulib/config/route.hpp` (modified),
`test/command_registry_test.cpp`, `test/f8_signature_pin_test.cpp`, `test/manifest_test.cpp`,
`test/path_runner_test.cpp`, and this log. **None of it has been through the coordinator's
verification** (no gate run, no hand mutations, no read of the load-bearing code), and the
executor's own DoD checklist above this line is incomplete. Treat every claim above as the
executor's, unverified.

**How to resume (RESUMING.md, "Where things stand"):** the tree is dirty on purpose. Either
(a) launch a fresh G2 executor with the brief and tell it to CONTINUE from this log — read this
file first, keep what builds, finish the DoD, run every mutation again on the final code — then the
coordinator verifies and commits; or (b) `git checkout -- include/shulib/config/route.hpp && git
clean -fd include/shulib/runner test/command_registry_test.cpp test/f8_signature_pin_test.cpp
test/manifest_test.cpp test/path_runner_test.cpp docs/internal/chunks/G2-PROGRESS.md` and start G2
from the brief. Do NOT commit what is here as it stands.

---

## RESUMED 2026-09-18 (evening) by a second executor

**Tree at resume:** `d9d0ee3 docs(internal): session close-out 2026-09-18 -- G2 interrupted mid-chunk, tree left dirty on purpose`
**Model:** opus. **Mandate:** treat every claim above as UNVERIFIED; keep what is correct, fix what is
not, finish the chunk, re-run EVERY mutation against the FINAL code. Do not commit.

### R0 — inventory of the first executor's tree (before any edit)

**What the first executor had left on disk** (`git status --short`, unchanged from the close-out):

| File | State found | Verdict |
|---|---|---|
| `include/shulib/config/route.hpp` (M) | `kMaxMarkerIdBytes`, `commandIdView`, `commandIdValid`, `BuiltinCommand` + `kBuiltinCommands` + `kBuiltinCommandCount` + `findBuiltinCommand`, `markerIdView`, and four new `valid()` rules | **Sound, kept.** The built-in SPEC living in `config/` (not `runner/`) is the right call and is argued in the header: `valid()` checks arity, the manifest publishes it, F8 freezes it; only the DOING is the runner's. The reverse would make `config/` depend on `runner/` |
| `include/shulib/runner/command_registry.hpp` (237 lines) | complete: `Handler`, `CommandEntry`, `CommandRegistry` with six preconditions and the `then()`-shaped adaptation | **Sound, kept unchanged.** |
| `include/shulib/runner/path_runner.hpp` (458 lines) | complete: `RunStopCause`, `RunReport`, `PathRunner::run` + the built-ins + D1's error policy + `bearingTo` | **Sound, kept unchanged.** `bearingTo` is `Routine::bearingTo`'s expression character for character (verified side by side: `routine.hpp:445-453` vs `path_runner.hpp:398-406` — same pose read, same delta order, same `1e-9` hypot precondition, same `atan2(dy, dx)`) |
| `include/shulib/runner/manifest.hpp` (111 lines) | complete | **Sound, kept unchanged.** |
| `test/command_registry_test.cpp` (259) | tests 1–2, complete | kept (one lint fix) |
| `test/path_runner_test.cpp` (694) | tests 3–10 + 10b | kept; **test 4's tank arm was WRONG — see D3** |
| `test/manifest_test.cpp` (116) | test 11 + 11b + 11c | kept unchanged |
| `test/f8_signature_pin_test.cpp` (492) | test 12, complete | kept unchanged |
| `test/route_test.cpp` | **UNTOUCHED — test 13 was never written** | written now |
| `include/shulib/version.hpp` | **UNTOUCHED — never bumped** | bumped now |
| `docs/` (all of it) | **UNTOUCHED — the whole documentation pass was still to do** | done now |

Its own log stopped after "required reading"; there is no step-by-step record, no mutation, no gate
run and no DoD evidence in it. Everything above was re-derived by reading the files against the
brief, not taken from the log.

### R1 — what was MISSING against the brief

§2 items 1–7 were all present in CODE. Missing entirely: `version.hpp`'s bump (§2 item 5), test 13
(§4), the integration doc's frozen schema (§2 item 6), the whole of §6's documentation contract, and
every mutation in §4.

### R2 — DEFECTS found in what existed

Three, all found by the first full build + run of the suite, which the first executor never reached.

**D1 — `version.hpp` was never bumped, and the F8 pin's own case fails red on it.**
`test/f8_signature_pin_test.cpp:489` CHECKs `kApiMinor >= 2`; the tree carried `kApiMinor = 1`.
Fixed: `kApiMinor = 2`, `kApiVersionString = "2.2"`, with the note naming every additive item AND
the one relaxation (D2). This is the interruption point the close-out named, so it is the one
"defect" that is really just unfinished work.

**D2 — `Route::valid()` refused `MoveToPose` + `HeadingMode::Free`, which the brief's ruling R6,
its §2 item 3 mapping table and its test 9 all require to be LEGAL.** G1 wrote one bundled rule
("MoveToPose or TurnTo with Free — that verb needs a heading"), committed, with a committed test
(`route_test.cpp` 2h) asserting the refusal. The first executor wrote the runner's R6 branch
(`path_runner.hpp:230-232`) and test 9 for it, but never touched the rule — so test 9 threw
`PathRunner::run: route 'free' waypoint 0 is a MoveToPose with headingMode Free`, and R6 was
unreachable dead code. **This is the one ruling of this resumption that goes beyond "additive
rules only" and it is flagged for the coordinator — see RULING R6-SPLIT below.**

**D3 — test 4's tank arm asserted something the library deliberately does not do, and was red.**
Measured, not guessed: `stoppedAt=1, reason=MotionFailed, lastExit=TimedOut, legsRun=2`. The
X-drive acceptance route's first moving leg is a `DriveTo` to a point ~34° off the start heading,
and on a tank `Chassis::moveTo` translates along the heading it servos to — D12/C1's honesty, "the
library never sequences a turn you did not ask for". `chassis_routine_test.cpp`'s tank cases carry
exactly this shape already ("the AUTHOR plans the turn"). The first executor's test asserted the
tank would arrive anyway, which would only have passed if the runner had silently planned a turn
the facade refuses to plan — i.e. the test was asking for the bug.
Fixed by making the finding the test: a `tankAcceptanceRoute()` that carries its two turns as
waypoints (computed at AUTHORING time from the nominal path, which is all a path editor has),
plus a third subcase pinning that the un-authored route fails in the DEFINED way (stop at
waypoint 1, TimedOut, 3 skipped, drive safed) rather than in some new way.

### R3 — the fixes, and the suite accounting (MEASURED, not inferred)

Fixes applied (each above): D1 `version.hpp` → 2.2; D2 the R6 split in `route.hpp` +
`route_test.cpp` 2h rewritten; D3 `path_runner_test.cpp` test 4 re-authored. Plus test 13
(`route_test.cpp` 2j, which did not exist), and one lint fix in two files: the
`"\xc3\xa9" "clair"` literal-concatenation spelling (a `-Wstring-concatenation` diagnostic in an
array initializer) became the unambiguous octal `"\303\251clair"` in both `route_test.cpp` and
`command_registry_test.cpp`.

**Suite accounting, measured by building the tree BOTH ways** (G2's files moved aside, `route.hpp`
/ `version.hpp` / `route_test.cpp` restored from `d9d0ee3`, `cmake -S test -B build/test`
re-run, the binary rebuilt and run under `--reporters=xml`; then restored, md5-verified, rebuilt,
re-run). Per-TEST-CASE assertion counts were diffed, not just the totals:

- **baseline at `d9d0ee3`: 1342 cases / 1,632,353 assertions — green.** (Exactly the number the
  brief predicted.)
- **final: 1358 cases / 1,635,290 assertions — green.** +16 cases, +2,937 assertions.
- **Exactly ONE pre-existing test case moved, and it is the one the brief predicted:**
  `only hal/pros includes <pros/> — the in-tree mirror of the CI guard`, **483 → 492 (+9)** —
  three assertions per new header × the three `runner/` headers. Every other pre-existing case is
  byte-identical in count.
- One pre-existing case was RENAMED rather than moved: `G1 route 2h` (7 assertions) is now
  `G1 route 2h: TurnTo refuses Free, MoveToPose accepts it (R6); DriveTo refuses Fixed`
  (8 assertions) — the R6 split, D2 above. It shows in the diff as one removal + one addition.
- The 17 added cases: 2 registry, 3 manifest, 9 runner (3, 4, 5, 6, 7, 8, 9, 10, 10b), the F8 pin
  case, the renamed 2h, and `G2 route 2j` (test 13, 52 assertions).

### R4 — MUTATIONS, all re-run against the FINAL code

**A process finding first, because it invalidates a sloppy reading of the protocol.** The first
pass of M11 (reorder the built-ins) came back **GREEN**, and it was not because the test is weak.
`config::kBuiltinCommands` is an `inline constexpr` variable: every TU that includes `route.hpp`
emits its own definition and the linker keeps ONE. Touching only `manifest_test.cpp` rebuilt that
one object; the four other G2 TUs kept their pre-mutation copies, the linker picked one of those,
and the golden compared the ORIGINAL bytes and passed. The brief's wording — *"touch the test
file(s) INCLUDING the edited header"* — is exactly this hazard, and it is worth writing down
because the failure mode is a mutation that reports itself as "the test does not catch this",
which is the most expensive wrong answer a mutation run can give.
**Every mutation below therefore touches all five TUs that include the G2 headers**
(`route_test`, `command_registry_test`, `manifest_test`, `path_runner_test`,
`f8_signature_pin_test`) plus the mutated header, and every one requires the
`Linking CXX executable shulib_tests` line before it is run. Full transcript:
`mutations.log` in the executor's scratchpad; the observed messages are quoted below.

| # | Mutation | Where | Observed |
|---|---|---|---|
| M1a | drop the duplicate-id precondition | `command_registry.hpp` | RED: `CHECK_THROWS_AS( r.on("intake_in", intakeIn), PreconditionError ) did NOT throw at all!` + `CHECK( r.count() == 2 )` → `CHECK( 4 == 2 )` |
| M1b | widen the id charset to `[A-z0-9_ -]` | `route.hpp` | RED **AT BUILD TIME**, and by the freeze: `f8_signature_pin_test.cpp:251: error: static assertion failed: F8 FREEZE VIOLATION: the frozen shape of commandIdValid refusing uppercase (the no-escaping-ever rule) changed …` (+ the space and hyphen pins) |
| M2 | flip the `bool` adaptation | `command_registry.hpp` | RED: `CHECK( fire(r, marker("f")) == MechanismOutcome::Unconfirmed )` → `CHECK( 1 == 2 )`, **and the cross-tier pin**: `CHECK( registrySucceeded == routine.ok() )` → `CHECK( false == true )` at `k := 1` |
| M3 | `atan2(dy,dx)` → `atan2(dx,dy)` | `path_runner.hpp` | RED: the acceptance test, `REQUIRE( a.ticks == b.ticks )` → `REQUIRE( 458 == 469 )`; test 10 also fell over |
| M5a | fire the markers BEFORE the leg | `path_runner.hpp` | RED: `CHECK( firedAtY > 22.0 )` → `CHECK( -12 > 22 )` — the marker fired at the START pose |
| M5b | carry on past a non-Settled leg | `path_runner.hpp` | RED, 17 assertions across two cases: `CHECK( fired == 0 )` → `2 == 0`, `stoppedAt` `-1 == 1`, `legsSkipped` `0 == 3`, `driveIsSafed` false |
| M6a | an unknown id STOPS the route | `path_runner.hpp` | RED: `CHECK( report.ok() )` false, `legsRun 2 == 5`, `known 0 == 1` — the guarantee broken exactly where the doc promises it |
| M6b | an unknown id is SILENT | `path_runner.hpp` | RED: `CHECK( warnCount(r.telemetry, "no handler") == 2 )` → `CHECK( 0 == 2 )` |
| M7 | skip `cancel()` on a stop | `path_runner.hpp` | RED in BOTH stop paths (motion and marker), 7 assertions: `CHECK( driveIsSafed(r.rig.h) )` → `CHECK( false )` for every `MechanismOutcome` |
| M9 | `Free` on a MoveToPose means 0 rad | `path_runner.hpp` | RED: `REQUIRE( a.ticks == b.ticks )` → `REQUIRE( 163 == 156 )` (the rig starts at 90°, which is what makes zero visible) |
| M10 | drop the π on a reversed DriveTo | `path_runner.hpp` | RED: `CHECK( …errorTo(Angle::radians(Angle::kPi))… < 0.10 )` → `CHECK( 3.14159 < 0.1 )` |
| M11 | reorder the built-ins | `route.hpp` | RED **twice over**. Stage A (the reorder alone): the build fails — `f8_signature_pin_test.cpp:263: error: static assertion failed: F8 FREEZE VIOLATION: the frozen shape of built-in 0 is "wait_ms" (the manifest's order is part of its golden) changed …` Stage B (the two F8 pins relaxed so the GOLDEN is the one that speaks): `CHECK( sink.text() == expected )` red, `brake` and `wait_ms` transposed in the diff |
| M13 | drop the built-in ARITY rule from `valid()` | `route.hpp` | RED: `CHECK( why.find("built-in id takes") != npos )` for `wait_ms` and `hold_ms` (7 assertions) |
| M13b | drop the reverse-belongs-to-DriveTo rule | `route.hpp` | RED: 10 assertions, every one of the five non-DriveTo verbs |
| **F8-A** | `PathRunner::run` GAINS `noexcept` | `path_runner.hpp` | Build fails: `f8_signature_pin_test.cpp:456: error: static assertion failed: F8 FREEZE VIOLATION: PathRunner::run GAINED noexcept. A change to a frozen signature's noexcept shape is BREAKING IN EITHER DIRECTION …` — the INVERSE pin the F6/F10 lesson exists for, proven to fire |
| **F8-B** | `CommandRegistry::count()` returns `int` | `command_registry.hpp` | Build fails: `f8_signature_pin_test.cpp:371: error: static assertion failed: F8 FREEZE VIOLATION: the frozen shape of CommandRegistry::count() const noexcept -> std::size_t changed …` |

**Green with no mutation surviving:** 14 mutations run (12 from the brief's §4 table + the two F8
signature mutations), **14 red, 0 green.** The one that first came back green (M11) is written up
above as the ODR finding, not buried. Every file restored byte-for-byte afterwards (`diff -rq`
against the pre-mutation copies: no output), relinked, and re-run green.

Two diagnostic-quality fixes made while the mutation reds were being read: `CAPTURE(b.id)` and
`CAPTURE(mechanismOutcomeName(o))` printed a POINTER (`0x5fdfff548596`) rather than the string,
which made three mutation reds unreadable about WHICH built-in or WHICH outcome failed. Wrapped in
`std::string{}` in `route_test.cpp`, `path_runner_test.cpp` and `command_registry_test.cpp`; the
reds above already show the fixed form (`… := UNCONFIRMED`). M13/M13b were re-run after the fix.

### R5 — RULINGS taken beyond the brief (the coordinator should read this section first)

**R6-SPLIT — `Route::valid()` was RELAXED for `MoveToPose` + `HeadingMode::Free`. This is the one
thing here that is not "additive fields/rules only", and it changes a committed G1 test.**

*The conflict.* G1 committed one bundled rule — *"a `MoveToPose` or `TurnTo` with `Free` needs a
heading"* — with a committed test asserting it (`route_test.cpp` 2h, subcase "MoveToPose with Free").
The G2 brief contradicts it in three independent places: §2 item 3's mapping table ("`HeadingMode::Free`
→ the heading argument is the CURRENT estimate's heading (hold what you have)"), **ruling R6** (a
numbered ruling with a named rejected alternative), and **test 9**, which exists for nothing else.
§4's test-13 list also names only *"Free on TurnTo"* as the heading rule `valid()` gains, which only
parses if `MoveToPose` + `Free` is meant to become legal.

*What was chosen, and why.* The brief wins. R6 is a deliberate ruling written AFTER G1 landed, with
its alternative recorded; the bundled G1 rule reads like one verb's reasoning applied to two verbs
that are not alike. `TurnTo` + `Free` is a turn to nowhere — there is no motion left. `MoveToPose` +
`Free` has an honest fallback the other does not: the heading the estimate already carries. *"Go
there, keep facing where I am facing"* is an authoring idiom with no other spelling in this
vocabulary, and G1's own `HeadingMode::Free` comment already gestured at it ("the approach bearing,
**or whatever the robot already has**").

*Why doing it NOW is the cheap direction.* Relaxing a validity rule is ADDITIVE — every route that
validated under 2.1 still validates under 2.2 — while TIGHTENING one is breaking. So the cost of
relaxing at the freeze is zero and the cost of getting it wrong later is a major bump. `TurnTo`
keeps the refusal, unchanged, and the rewritten 2h now pins the DIFFERENCE between the two verbs
rather than their sameness, which is the property that is actually load-bearing.

*The alternative, recorded.* Keep G1's rule, delete R6, delete test 9, and leave the runner's Free
branch as dead code — coherent (`Free` would then mean only "the runner computes the approach
bearing", i.e. `DriveTo` only) and strictly smaller. It was rejected because it deletes a ruling the
brief argues for rather than answering it, and because `moveTo` always commands a heading, so
somebody has to choose one; "the one you already have" is the only choice that asks the drivetrain
for nothing it was not asked for.

**If the coordinator disagrees, the reversal is three edits** (`route.hpp`'s rule + its two doc
comments, `route_test.cpp` 2h, `path_runner_test.cpp` test 9) and the F8 pin does not move.

---

**R-MANIFEST-FIELDS — the manifest line carries `minArgs`/`maxArgs`/`doc`, not the brief's `args`.**
Brief §2 item 4 sketches `{"id":"intake_in","builtin":false,"args":0}`. What shipped is
`{"id":…,"builtin":…,"minArgs":…,"maxArgs":…,"doc":…}`. Reason: `log` takes 0 **to** 2 args, so a
single `args` number cannot describe the built-ins the manifest exists to publish, and a picker that
must render "0, 1 or 2 fields" needs the range. `doc` carries the built-ins' one-line meanings so the
picker can label a button without shipping its own copy of the vocabulary. Both are the additive
direction (a field may be appended later; a field that means the wrong thing cannot be fixed later).
Kept from the first executor, who made this call; recorded here because the brief's sketch differs.

**R-TANK-AUTHORING — a tank route is authored with its turns, and the test says so.** See D3. This is
not a G2 decision so much as D12/C1's drivetrain honesty becoming visible one tier up, but it is the
first time the data layer has had to state it, so it is stated: in `path_runner_test.cpp`'s banner,
in the new `tankAcceptanceRoute()`'s comment, and in `build-order.md`'s G2 entry.

**R-NO-NEW-PUBLIC-SURFACE — nothing was added to `chassis.hpp` or `routine.hpp`.** Both are FROZEN
(F6/F10) and both were read, not edited. `bearingTo` is duplicated into `path_runner.hpp` and pinned
equal by test rather than shared, because `Routine::bearingTo` is private and exposing it would
change a frozen surface to save eight lines.

### R6 — the documentation pass, and the prose review (the part the gates cannot see)

**Touched, with what changed:**

| Document | What moved |
|---|---|
| `docs/vexbuilder-integration.md` | **§4.1–4.3 rewritten as the FROZEN contract.** New: the verb table (what each `motion` means, which `headingMode` it takes, which fields it reads); the units/zero-sentinel table (in/s, rad/s, s; `heading` in DEGREES in the file, the one exception, with the reason); the **rename table** proposal→frozen (`"locked"`→`"fixed"`, `"tangent"`→`motion:"drive"`, `args:{}`→`args:[int,int]`, the added `"start"`/`"drive"`/`"hold"`); the **`"arc"` gap stated as a gap**; the id charset rule; the four built-ins with their arities; **the manifest's real bytes**; the unknown-id guarantee restated as implemented-and-tested; §4.3.1 answering `NONE`; §4.3.2 stating honestly that a route has run on the host plant only. Also the status rows in §0/§1/§2, the §7 risk table (three rows RETIRED, struck through rather than deleted) and §8's work lists |
| `docs/roadmap.md` | the **F8 register row → ✅ LOCKED 2026-09-18**, with the frozen list, the additive list, what is deliberately NOT frozen, the R6 change, and the pin test + golden + version mechanism named; M5's "Path authoring" bullets (runner / registry / manifest `[x]`, the `.vexbot` READER stays `[ ]`, with "profiled" corrected); M5's DoD (F8 locked, and a sentence saying the DoD itself is **not** met); **"You are here"**; the reciprocal-list row B; the two-things-that-unblock paragraph; the `NONE` joint decision, answered |
| `docs/legacy-command-vocabulary.md` | **not on the brief's list — see the note below.** The `NONE` row's fate cell, the §4 "For G2's manifest" paragraph and §5's first gap entry, all of which named G2 as the owner of an OPEN decision that G2 has now closed |
| `docs/internal/build-order.md` | the G2 entry (what landed, plus two corrections to the entry's own wording — "profiled per-segment execution" was never G2's, and "WARN fault" was wrong in kind); the FIRST bold `Next:` block — "G2 BRIEFED AND LAUNCHED" replaced with what is now true, everything else in that block left alone |
| `include/shulib/version.hpp` | `kApiMinor` 1 → **2**, `kApiVersionString` → **"2.2"**, with a note naming every additive item and the R6 relaxation |
| `docs/api/` + `mkdocs.yml` | regenerated: 3 new pages (`command_registry.md`, `manifest.md`, `path_runner.md`), a Runner section in the nav, `route.md` and `version.md` updated, the index and `all-entities.md` refreshed |
| `docs/internal/PROJECT-BRIEFING.md` | regenerated LAST, after the final mutation was restored and the suite was green, so no mutant's numbers could be baked in |

**`legacy-command-vocabulary.md` was not on the brief's touch list and was touched anyway.** It is a
public document that said, in three places, *"G2 must decide boundary-marker representation"* — a
decision G2 made and FROZE. Leaving it would have left the tree stating both that the question is
open and that the answer is pinned by a compile-time test. The edits are surgical (three passages,
struck-through rather than deleted so the history reads), and the G4 half of each gap is left open
because it really is still open. Flagged here because it is a deviation from the contract.

**NOT touched, with why:**

- **`README.md` and `docs/guide/`** — the guide chapter this deserves is *"Your first data-driven
  routine"*, and it cannot honestly be written until G3 can LOAD one from a file: today the only way
  to get a `Route` is to type it in C++, which is the audience this feature exists to avoid. Writing
  it now would document a workflow no non-coder can follow. **Owner: G3/G4.** (The guide's three
  "command id" hits are a DIFFERENT term — the per-motion serial number in the diagnostics — and are
  correct as they stand; `15-glossary.md` defines that sense explicitly, so there is no collision to
  fix, though G4's guide chapter should say which "command id" it means.)
- **`docs/hardware-assumptions.md`** — G2 asserts no belief about hardware. Nothing in it moved.
- **`docs/shulib-v2-master-plan.md`** — §16.3 is the sketch this chunk implements, and the plan is
  the WHY document; its `runner.on(...)` spelling (now `registry.on(...)`) is the one wart, and the
  master plan is explicitly outside this chunk's contract. **Left for the coordinator to rule on.**
- **`src/`, the `Makefile`, `tools/src_build_gate.py`, `R3a-BENCH-WORKSHEET.md`** — forbidden, and
  untouched: `git status` shows nothing under `src/`.

**Prose review, logged per changed/added source file.** Grepped `docs/` and `README.md` for
`PathRunner`, `runner.on`, `command-id`, `command id`, `manifest`, `F8`, `paths[]`, `markers`,
`code_template`; read every hit. Fixed: everything in the table above, plus one sentence in
`roadmap.md`'s reciprocal-half paragraph that still read "`PathRunner`, the id manifest … are still
unbuilt" AFTER the new "You are here" had been inserted below it (a stale line left standing by its
own replacement — exactly the failure the pace rules describe). Listed and deliberately left: the
master plan's hits, the guide's three unrelated "command id" hits, `roadmap.md`:327 (the motion
command-id stamping, a different subject), `roadmap.md`:1770/1805 (forward-looking M6/WS8 lines that
are still true).

### R7 — DoD, with evidence

- [x] **`runner/command_registry.hpp`, `runner/path_runner.hpp`, `runner/manifest.hpp`; every public
      member documented; ARM gate green; PROS-free and sim-free.**
      Doc coverage gate: `doc coverage: 2307 public entities across 145 headers, all documented`.
      ARM: `arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow
      -Werror -Os -mcpu=cortex-a9 …` over ALL 145 headers — exit 0. Both CI greps empty.
- [x] **Tests 1–13 green; every mutation logged red; the pre-existing suite green with pre-existing
      cases moved by zero assertions except the CI-guard mirror.**
      1358/1358 cases, 1,635,290/1,635,290 assertions, 0 failed. 14 mutations, 14 red, 0 survived
      (R4). Exactly one pre-existing case moved: the CI-guard mirror, +9 for three headers (R3).
- [x] **F8 frozen**: `test/f8_signature_pin_test.cpp` (compile-time, naming F8, mutation-proven four
      ways); Freeze Register row F8 `✅ LOCKED 2026-09-18` with the frozen list, the additive list and
      the not-frozen list; `version.hpp` 2.1 → **2.2** with the note; the schema frozen in
      `vexbuilder-integration.md` §4.1–4.3.
- [x] **`src/` untouched; the six-build gate green.** `git status --short` shows nothing under
      `src/`. `src build gate self-test: OK (33 detector cases…)`; `src build gate: PASS (all 6
      builds link…)` — bench / xdrive / tank / tank-drive / tank-library / tank-calibrate, each
      `exit 0, 4/4 TUs compiled, 0 disallowed warnings`.
- [x] **`G2-PROGRESS.md` complete; no hardware claim.** Every header, every test banner, the roadmap
      row, the build-order entry and §4.3.2 of the integration doc say the same thing in the same
      words: **a `Route` has executed on the A2 host plant only. The library's motion stack has never
      driven a physical robot.**

### R8 — final state

**NOTHING COMMITTED. NOTHING STAGED. NO PUSH. NO WORKTREE.** `git status --short` at close:

```
 M docs/api/README.md              M docs/internal/build-order.md
 M docs/api/all-entities.md        M docs/legacy-command-vocabulary.md
 M docs/api/route.md               M docs/roadmap.md
 M docs/api/version.md             M docs/vexbuilder-integration.md
 M docs/internal/PROJECT-BRIEFING.md   M include/shulib/config/route.hpp
 M mkdocs.yml                      M include/shulib/version.hpp
 M test/route_test.cpp
?? docs/api/command_registry.md    ?? include/shulib/runner/
?? docs/api/manifest.md            ?? test/command_registry_test.cpp
?? docs/api/path_runner.md         ?? test/f8_signature_pin_test.cpp
?? docs/internal/chunks/G2-PROGRESS.md  ?? test/manifest_test.cpp
                                   ?? test/path_runner_test.cpp
```

`docs/internal/chunks/SESSION-2026-09-18-autonomous.md` is also modified — **that is the
coordinator's own resume entry, not this executor's**, and it was left alone.

**Open / partial, under-claimed deliberately:**
- **No hardware.** A route has run on the host plant only.
- **`docs/guide/` has no data-driven-routine chapter.** Deferred to G3/G4 with the reason above; M5's
  DoD is correspondingly NOT ticked.
- **The `.vexbot` reader stays `[ ]`** — G3 parses the schema this chunk froze.
- **The master plan's `runner.on(...)` spelling** is now `registry.on(...)` in the code. One wart,
  outside this chunk's documentation contract, flagged for the coordinator rather than fixed.
- **R6-SPLIT (R5) is the one ruling that needs a coordinator decision**, because it relaxes a G1 rule
  and rewrites a committed G1 test.
