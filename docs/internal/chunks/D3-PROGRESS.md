# D3 — PROGRESS LOG (live)

Chunk: **D3 — the recipe cookbook + generated API reference**
Branch: `shulib-v2`, starting commit `5287279`, tree clean.
Baseline at start: suite **690 cases / 916,050 assertions**, ARM gate clean at 104 headers.

This file is appended to continuously, timestamped. It is the recovery record if the
session is interrupted. Newest entries at the bottom.

---

**19:31:43** — D3 started. Confirmed branch `shulib-v2`, HEAD `5287279`
("docs(chunks): D3 brief — the cookbook, generated docs, and Routine's freeze"),
working tree clean. Progress log created as the first action per the brief.

**19:31:43** — Plan of attack, in the order the brief mandates:
1. Required reading (brief → `routine.hpp` in full → D1 §2/§6 → D2 §2/§2.D →
   `chassis.hpp` → `version.hpp` → `f6_signature_pin_test.cpp` → guide docs idiom →
   `RESUMING.md`).
2. Establish baseline (build + suite + guards + ARM gate) before touching anything.
3. **Cookbook first** — write real recipes against `Routine`, record every awkwardness.
4. `Routine` critique from what the cookbook found.
5. Doc-coverage gate + generator.
6. **Only then** rule A1 (freeze), and B1–B4.

**19:38:12** — Baseline confirmed independently before touching anything:
`cmake --build build/test` clean, suite **690 passed / 0 failed / 3 skipped,
916050 assertions**. Matches the brief's stated baseline exactly.

**19:38:12** — Required reading done: the D3 brief; `routine.hpp` in full (484 lines,
12 steps + 4 observers + the STATUS banner that says it is deliberately outside F6);
`chassis.hpp` (frozen F6 banner, 20 members, 3 types); D1-COMPLETED §2 (the nine-item
facade critique — the format my `Routine` critique must imitate) and §6 (the twelve
`Routine` decisions D1–D12); D2-COMPLETED §2/§2.D (docket format + mapping table) and
§6/§10 (the two holes: noexcept blindness on non-overloaded members, and the
`300_s`-for-`300_ms` call site that passed every outcome assertion); `version.hpp`
(breaking = kApiMajor + migration note; additive = kApiMinor); `f6_signature_pin_test.cpp`
(36 pins, concept-templated-on-the-class, compound `{ call } noexcept` requirement, one
negative pin); `guide-maintenance.md` (three rules: compile every example, link don't
restate, write for someone who doesn't know yet); ch. 09 + ch. 10 + `guide_examples_test.cpp`
(the verbatim-quoting idiom); `RESUMING.md`; `verify-d2.sh` + verify/README ("a red gate
is a question, not a verdict").

Key mechanical facts I will need:
- The drift scan is a python block in `verify-d2.sh` §9 — it currently scans only
  `docs/guide/*.md`. The brief requires the scan to cover **all** of `docs/`.
- `Routine`'s public surface: ctor, 4 deleted copy/move ops, dtor, 12 steps
  (`startAt`, `moveTo`, `driveTo`, `strafeTo`, `turnTo`, `face`, `followTrajectory` ×2,
  `brake`, `hold`, `pause`, `waitFor`, `then`), 4 observers (`ok`, `result`,
  `lastTrajectory`, `chassis`). Plus `RoutineResult` (8 fields) and `RoutineStopCause`
  (4 enumerators).
- `noexcept`-carrying `Routine` members (the D2 hole #1 trap): ctor, `ok()`, `result()`,
  `lastTrajectory()`, `chassis()`. Any pin must use the compound-requirement detector.

**19:47:05** — Architecture decided on evidence (rulings will be written up in
`D3-COMPLETED.md` §docket B; recording the evidence here as it was gathered):
- **B1 evidence:** `doxygen` is **NOT installed** on this machine (`which doxygen` → empty),
  so it is a genuinely new toolchain dependency. It also has no native markdown emitter
  (needs doxybook/moxygen on top), and — decisive — its coverage warning
  (`WARN_IF_UNDOCUMENTED`) is a *different* notion of "documented" from whatever a
  generator extracts. A custom extractor gives **one parser** feeding **both** the
  reference and the coverage gate, so "documented" means exactly one thing.
  `python3` is 3.12.3 and already used by the verify harnesses.
- **B4 evidence:** committed output + a build-time regeneration check. A committed
  reference is browsable on GitHub with no toolchain (a student on a phone), and it is
  the web-portable artifact the publish path needs.
- The drift scan and the generated reference would collide (a generated signature in a
  ```cpp fence is not an "example"). Resolution: `docs/api/` is covered by the
  **regeneration check** instead — which is strictly stronger, because it compares the
  whole directory byte-for-byte against a fresh run, so a hand-written file dropped in
  `docs/api/` fails. The scan asserts that partition rather than carrying a loophole.

**19:47:05** — Starting the **cookbook** (brief's mandated order: cookbook → critique →
gate → freeze). Writing `test/cookbook_examples_test.cpp` first — every recipe is a real
compiled routine that runs against the plant, and the markdown quotes it verbatim.
Recipes planned, chosen to stress `Routine` the way a real VEX U auton would:
skeleton · two-goal side run with a composed sub-sequence · trajectory sweep + partial
recovery · bail-out on a failed grab · attempt-and-continue · alliance-partner wait ·
match-clock budget · tank routine · H-drive lateral budget · mixed-tier with the verdict
honored · relocalize mid-routine.

**19:56:40** — `test/cookbook_examples_test.cpp` written and **green**: 11 cases,
95 assertions, first build clean under `-Werror` after two trivial fixes (a `const`
qualifier on the rig accessor, one unused using-decl). Every recipe runs against the
plant and is graded on ground truth.

**19:58:10** — **Timing bounds calibrated, not guessed.** Forced every clock bound to
`< 0.0` and read the real simulated elapsed off the failure messages, then set each
bound with honest headroom:
| case | measured | bound |
|---|---|---|
| 01a skeleton | 1.65 s | 4.0 s |
| 01b two-goal side run | 7.52 s | 12.0 s |
| 03b budgeted (tight) | 5.30 s | 9.0 s |
| 03b budgeted (generous) | 8.19 s | > 6.0 s and < 13.0 s |
| 04a tank routine | 5.88 s | 10.0 s |
| 04b H-drive lateral | 1.46 s | 4.0 s |
The generous-budget case gained a **lower** bound too, so the pair proves the optional
goal really costs time — otherwise "the budget check works" would be provable by a
routine that never did anything.

**19:58:10** — **FINDINGS ALREADY BANKED from writing the recipes** (full critique to
follow; these are the ones the code itself demonstrates):
1. **No whole-chain deadline.** Per-step timeouts only. Budgeting a 15 s match needs the
   clock, and the clock is only reachable through the **Tier-3 seam**
   (`chassis.deps().ctx->clock()`) — a Tier-2 concept needing Tier-3 plumbing, which is
   exactly the smell D1 §2.2 raised for `pause` and D2 fixed by adding `wait()`.
2. **No `.finally()`** (D1 D11 deferred it). "Always park" cannot live in a chain that
   might stop, so every serious recipe ends up with a **second `Routine`** for the park —
   and must save `r.result()` first, because the second chain has its own counters.
3. **A failure you want to survive must be swallowed silently.** `then()` stops on
   `false`, so "attempt the grab, keep sweeping" must return `void` and stash the outcome
   in the author's own variable. Asserted cost: the routine layer logs **nothing** about
   the miss (`routineWarnings(log) == 0` in cookbook-02b).
4. **A bare facade call mid-chain is invisible to the chain.** `chassis.moveTo()` that
   times out leaves `ok()` **true** and the chain drives on from the wrong place.
   `then([&]{ return chassis.moveTo(...); })` is the fix and it works — but nothing
   makes the author reach for it. Both halves pinned in cookbook-05a.
5. **`exit == Running` after a non-motion stop.** Documented, and still reads as "still
   going" to a newcomer looking at a failed routine.
6. **`startAt` is the only re-seed spelling**, so the mid-routine relocalization idiom is
   spelled "startAt" in the middle of a routine (cookbook-05b).
7. **`then()`'s `name` defaults to `"action"`**, so an unnamed action reports
   `STOPPED at step 7 (action)` — the default makes the transcript worse in exactly the
   case where a transcript matters.

**20:11:30** — Cookbook written: `docs/cookbook/` — README index + 5 files
(`01-getting-there`, `02-when-a-step-fails`, `03-timing-and-partners`,
`04-drivetrains`, `05-mixing-tiers`), 14 recipes. Every listing quoted verbatim.

**20:11:30** — **Drift scan extended to ALL public docs, and it found two pre-existing
holes** (verify/README's rule applied: a red gate is a question — both predate D3):
1. **`README.md`'s front-page example was never compiled.** Every other public code
   block has been verbatim-quoted from a compiled test since C8; the README's was swept
   **by hand** at D2's retype — the exact failure mode the rule exists to remove. The
   first thing a stranger reads was the one example nothing checked.
   **Fixed where it lives** (rule 4): new `test/readme_example_test.cpp`, case
   `readme-a`, block quoted verbatim including its `#include`/`using` lines, graded on
   ground truth (4 motions, < 1 in, < 2°). Green.
2. **`docs/shulib-v2-master-plan.md`** carried a ```cpp block for `runner.on(...)` — a
   sketch of `PathRunner`, which **does not exist**. It cannot be compiled, so it cannot
   be a ```cpp block under this rule. Re-fenced as ```text with one sentence saying why
   (design, not code you can write today).
The scan's source set is now a **glob** — `test/*example*_test.cpp` — so a future
examples file is covered the moment it exists (same self-maintaining principle as the
ARM gate's generated header list). **Scan now reads 343 quoted lines, 0 not verbatim.**

**20:26:40** — **FINDING #8, and it is a real bug, not an awkwardness.**
`Routine::lastTrajectory()` on a routine that has never run a trajectory returned a
**value-initialized** `TrajectoryResult` — `{exit = Settled, completedLegs = 0,
totalLegs = 0}` — and `succeeded()` reads that as **SUCCESS**. Verified with a
standalone probe: `virgin TrajectoryResult.succeeded() = 1 (legs 0/0)`.

Why it is a lie only here: at the facade, `followTrajectory` requires at least one
waypoint (precondition) and always returns a populated result, so `succeeded()` is
correct there and `TrajectoryResult` is **F6-frozen** and must not change. The recipe
layer creates the member at construction, before any trajectory exists — so the
default value is the layer's own problem.

The case that would actually mislead someone: a `followTrajectory` step that got
**SKIPPED** after an earlier stop. `if (r.lastTrajectory().succeeded())` takes the
success branch on a trajectory that never ran.

**Fixed in the layer that owns it** (rule 4), with zero surface change: the member now
initializes to `{.exit = control::ExitReason::Running}` — the project's own "no verdict
here yet" convention, already used by `RoutineResult::exit` and `CompletedMotion` for
exactly this. `succeeded()` is now honestly false on a virgin routine. Header comment
explains both halves. Pinned by a new case in `test/chassis_recipe_test.cpp`
("D1/D3 trajectory: a routine with no trajectory does not claim a good one"), which
covers virgin / after-non-trajectory-steps / after-a-skip.

Suite after the fix: **703 cases / 916,157 assertions, all green** (was 690 / 916,050).

**20:41:05** — `tools/api_doc_tool.py` written (one parser, four subcommands:
`generate` / `check-coverage` / `check-fresh` / `self-test`). Self-test **OK** —
13 fixture assertions covering: types found, enumerators as members, `///` above vs
`///<` beside, **a member must not inherit its neighbour's comment**, private members
excluded, the `= delete` run rule and its non-leak, overloads not collapsed, `const` /
`noexcept` / default arguments preserved in the rendered signature, a constructor
rendered without its init list, the banner carried, and **determinism** (render twice,
generate twice, byte-identical).

Parser bug found and fixed while self-testing (worth recording, it is the sort of thing
that would have produced a plausible-looking wrong reference): "a declaration ends at
the first `;`" cuts in the wrong place for a one-line inline function
(`int f() const { return x; }` — the `;` is inside the body), rendering the signature
with the body glued on. Termination is now the first `;`, `{`, or constructor-init `:`
that is outside parentheses, angle brackets, string literals, and `::`.

**20:41:05** — **FIRST RUN OF THE COVERAGE GATE ON THE REAL HEADERS: 16 UNDOCUMENTED
PUBLIC MEMBERS**, on the surface that is about to be frozen forever. This is the
strongest possible evidence that the gate is not decoration — the generated reference
would have been *silently incomplete* and would have looked finished:

| Undocumented | Where |
|---|---|
| `class Chassis` itself | chassis.hpp:222 |
| `class Routine` itself | routine.hpp:153 |
| `MotionOptions::validate()` | chassis.hpp:197 |
| `TrajectoryResult::succeeded()` | chassis.hpp:217 |
| `Chassis::scheduler() const` (the const overload only) | chassis.hpp:469 |
| `Chassis` copy/move ctors + assignments + dtor (5) | chassis.hpp:236–240 |
| `Routine` copy/move ctors + assignments + dtor (5) | routine.hpp:161–165 |
| `RoutineResult::cause` | routine.hpp:146 |

Note the shape of what it caught: the two most-read classes in the library have no
one-line "what am I", the const `scheduler()` overload is undocumented while its
non-const twin is documented (a reader could reasonably conclude the const one does
something else), and `TrajectoryResult::succeeded()` — the member whose default value
was finding #8's bug — had no documentation at all.

`chassis.hpp` is FROZEN. Adding `///` comments changes **no signature**, so this is not
a freeze violation; the F6 pin will be re-run to prove it.

**21:22:40** — **MUTATION CAMPAIGN: 45 executed, build-gated. THREE GREEN HOLES FOUND
AND CLOSED.** Runner: `break → (regenerate) → BUILD (hard gate) → run → OBSERVE →
restore`, restoring from **file snapshots** rather than a reverse substitution — the
first run of the runner corrupted `chassis.hpp` because a reverse substitution cannot
undo a DELETION (its anchor is the empty string). Caught immediately by
`check-coverage`, restored by hand, runner rewritten. Logging it because it is exactly
the class of mistake the campaign discipline exists to surface.

**Doc-coverage gate — 7/7 RED, each naming the right member:**
D1 `Chassis::pose` · D2 `Routine::hold` · D3 a NEW undocumented member
(`Routine::stepsSoFar`) · D4 a removed trailing `///<` (`RoutineResult::steps`) ·
D5 the `= delete` run's shared comment (`Routine::operator=`) · D6 a member added
directly under a documented one (`Chassis::poseX` — the inherit-the-neighbour hole) ·
D7 the class declaration itself (`class Chassis`).

**Freshness — 3/3 RED:** F1 hand-edited generated markdown · F2 a header `///` changed
without regenerating · F3 a hand-written file dropped into `docs/api/`.

**Generator fidelity — 6 + 9 RED.** G1–G4 (drop `const` / `noexcept` / default
arguments / collapse an overload set) are caught by the tool's **self-test** at the
build gate, naming the exact rendered signature. G6 (non-deterministic output) likewise.
**G5 (template head dropped) was NOT caught by the self-test** — the fixture had no
template member. Gap closed: the fixture gained a template member and an assertion, and
the reverted fix re-fails.
Then P1–P9 isolate the **C++ fidelity pin** by mutating the *committed markdown* and
running the existing binary (the pin reads the docs at runtime), which is the real-world
shape: a generator regression whose wrong output got committed. All nine RED with the
fidelity test as the **sole** failing case — dropped `const`, dropped `noexcept`,
dropped default argument, a lost overload, a lost template head, `units::Time` retyped
back to a bare double, a member vanished from the index, two overloads merged in the
index, and the header prose dropped while the signature stayed right.

**Recipe behaviour — 12/12 RED:** R1 `then()` ignores `false` · R2 `pause()` delegates
half the duration · R3 `skipIfStopped` disabled · R4 `lastTrajectory_` back to
value-initialized (RED by exactly the new D1/D3 case) · R5 `startAt()` stops seeding ·
R6 `recordStop` stops safing the drive · R7 bearing loses the live pose · R8 the
cookbook's `lateralBudget` ignores `strafeAuthority` · R9 the budget comparison inverted
· R10 `strafeTo` silently becomes a `moveTo` · R11 the composed `scoreAt` stops chaining
· R12 `then()` stops honouring an `ExitReason` verdict.

### THE THREE GREEN HOLES (the most valuable output of this chunk)

**HOLE #1 — the verbatim rule was never mechanized.** (probes H1 and H6, both GREEN.)
The anti-rot rule "every ```cpp line in the docs appears verbatim in a compiled test"
has existed since C8 and is the mechanism this whole chunk was told to *extend rather
than compete with* — and it lived **only in `docs/internal/verify/verify-d2.sh`**. It
was never in the build and never in CI. A drifted example passed everything. H6 made
the point exactly: a chapter's `.hold(300_ms)` retyped to `.hold(300_s)` — D2's own
green hole, in the documentation this time — **built clean and passed the whole suite**.
**Closed:** `api_doc_tool.py check-examples`, wired into the build gate and CI, scanning
all public markdown against a *glob* of `test/*example*_test.cpp`. Both probes now RED.

**HOLE #2 — the removability property was never mechanized either.** (probe H7, GREEN.)
C7 established that no public document may reference `docs/internal/`, because that
directory is dropped at the squash-merge to `main`. Enforcement was a grep in an
internal script. A public doc could link into the development log and stay green until
the link broke in published docs. **Closed:** `check-removability`, same wiring. RED now.

**HOLE #3 — the coverage gate accepted EMPTY documentation.** (probe H3.) A bare `///`
with no text satisfied `bool(self.doc)`; the member passed and the reference rendered an
empty paragraph. The gate reproduced, inside itself, the precise "nothing looks wrong"
failure it exists to prevent. Verified in isolation: with a bare `///` above
`Routine::ok`, `check-coverage` exited **0**. **Closed:** documentation must contain
non-whitespace. Proven both ways — the empty `///` now names `Routine::ok`, and
reverting the one-line fix fails the self-test with "an EMPTY /// must not count as
documented".

**21:58:20** — **A1 RULED: `Routine` FREEZES at D3.** Reasoning (full write-up in the
record): the cookbook wrote fourteen recipes across three drivetrains and needed **zero**
changes to the surface; every awkwardness it found is **additively** fixable under
`version.hpp`'s policy, so freezing forecloses none of them; and leaving it unfrozen past
D3 would force the guide to carry an indefinite hedge, which is the contradictory-claims
state D2's constraint 5 exists to prevent.
**A2: a NEW register row (F10), not an F6 amendment** — the recipe layer is a strict
client of the facade, a different tier that can version independently, and amending F6
would retroactively blur what F6 promised on its own lock date.
**A3: yes, the pin extends** — `test/routine_signature_pin_test.cpp`, 37 pins.
**A4 exclusions, stated out loud:** `then()` is entirely outside F10 (placeholder shape
until F1/F3), and the stop/skip log line **wording** is outside (the behaviour is in).

**21:58:20** — **F10 PIN MUTATION CAMPAIGN: 16/16 RED, every one naming F10.** Run by
compiling the pin TU alone, because the doc gates run first in the full build and would
otherwise mask the pin under test. Includes **five `noexcept` drops on NON-overloaded
members** (`ok`, `result`, `lastTrajectory`, `chassis`, the constructor) — D2's hole #1,
not re-opened: the compound `{ call } noexcept` requirement is the sole detector for all
five. Also red: a renamed step, a retyped `hold`, a dropped default argument, a
const-callable step, a **renumbered** `RoutineStopCause`, two field type changes, a
movable `Routine`, a lost brace overload, and an added bare-double `pause` (the negative
pin — typed time as a frozen *semantic*, not just a signature).

**21:58:20** — Documentation sweep, atomic: `routine.hpp` banner → **FROZEN F10** with the
exclusions; `chassis.hpp`'s F6 banner updated (Routine "unfrozen until D3" → "froze at D3
as F10"); Freeze Register gains row F10 (ordered by milestone, after F9) and F6's row
updated; roadmap "you are here" + M7 checkboxes with cited evidence (`[x]` cookbook,
`[~]` generated docs — **nothing is published**); ch. 09 notice → frozen + the two doc
gaps the cookbook found (read `cause` before `exit`; a bare direct call mid-chain is
invisible to the chain); ch. 10 → the B2 division rule stated explicitly; ch. 14 → the
"recipe spellings can change" limitation FELL, replaced by the one thing still open
(`then()`); guide README → the three-documents table + the mechanized verbatim rule;
root README → cookbook and reference links.

**The removability gate caught my own edit** while I was writing it: a roadmap sentence
saying "build-order's next chunk" tripped the four-term grep. Rephrased. Worth recording
— the gate's first catch was the person who wrote it.

Suite: **707 cases / 916,222 assertions, green.**

**22:31:05** — **HOLE #4, found by the D3 verify harness — in a gate I had just written.**
`docs/api/chassis.md:546` contained `docs/internal/chunks/C4-COMPLETED.md` — a **public**
document linking into the development log, the exact C7 violation `check-removability`
exists to catch, and my own gate said green. Cause: I excluded `docs/api/` from the public
set because it is generated and covered by `check-fresh`. That exclusion is right for the
*verbatim* scan and wrong for removability — **freshness proves the file matches the header,
not that the header is fit to publish**, and the reference reproduces each header's banner
in full, so an internal path in a `//` comment lands in a published page.
**Closed twice over:** (a) the source — `chassis.hpp`'s banner now names the development log
without a path, matching `routine.hpp`'s existing spelling (comment-only; no signature
touched); (b) the gate — `docs/api/` is now scanned for removability while still being
skipped by the verbatim scan, with the asymmetry and its reason written into the tool.
Proven: planting `docs/internal/chunks/D2-COMPLETED.md` back into the header banner,
regenerating, and running the gate → **RED at `docs/api/chassis.md:545`**. Restored.

The general lesson, and it is the same one D2 §10.1 wrote about the noexcept pin: **a gate's
exclusion list is where its holes live.** Both of this chunk's gate holes (empty `///`
accepted; generated docs unscanned) were in what the gate declined to look at, not in what
it looked at wrongly.

**22:58:40** — **D3 COMPLETE.** `D3-COMPLETED.md` written (980 lines; the bar was 570–654).
`docs/internal/verify/verify-d3.sh` written and run: **MECHANICAL GATES PASSED** — every
section green, including the two proofs the brief demanded be re-derived from scratch
(a planted undocumented member fails NAMING `Routine::startAt`; a planted `noexcept` drop
fails NAMING F10).

Final state:
- suite **707 cases / 916,222 assertions**, green (from 690 / 916,050)
- GUARD1 + GUARD2 pass; ARM gate clean at **104 headers**
- removability four-term grep **empty**; drift scan **343 quoted lines, 0 non-verbatim**
- `docs/api/` byte-identical across two generations
- **nothing committed** — HEAD is still `5287279` (the D3 brief), 0 commits since

Honest partials, both `[~]` in the roadmap with the remainder named:
1. **Nothing is published.** No website, no Pages config. The reference is web-portable and
   the publish path is written up in `docs/internal/docs-publishing.md`; that is not the
   same thing as published.
2. **No new reader has read the cookbook cold.** The DoD clause needs a person. Owner now
   named (programming chair, next onboarding) rather than left hanging as C8 left it, and
   the verify harness asks for it so it cannot be quietly skipped.
