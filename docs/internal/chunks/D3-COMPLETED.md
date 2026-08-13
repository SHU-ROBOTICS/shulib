# Chunk D3 — COMPLETED

> **The recipe cookbook, the generated API reference, the doc-coverage gate, and the `Routine`
> freeze.** Phase D, chunk 3 of 3 — **Phase D closes here.**
> Brief: `D3-cookbook-and-generated-docs.md`. Live log: `D3-PROGRESS.md`.
> Branch `shulib-v2`, worked from `5287279`, **nothing committed** — everything is in the
> working tree.

---

## 0. The one-paragraph version

The cookbook was written first, as a critical consumer of `Routine`, and it worked: fourteen
recipes across three drivetrains, every one compiled and run against the plant, needing **zero
changes** to the recipe surface — while producing an eight-item critique and finding one real
**bug** (`lastTrajectory()` reported a *successful* trajectory on a routine that never ran one).
Only then did `Routine` freeze, as its own Freeze Register row **F10**, with `then()` explicitly
excluded and 37 compile-time pins holding it. The generated reference (`docs/api/`) and its
coverage gate landed alongside; the gate's **first run found 16 undocumented public members on
the surface about to be frozen forever**. The mutation campaign — 61 mutations across five
groups — found **four green holes**, three of them in enforcement that had existed only as
convention: the guide's verbatim anti-rot rule and the C7 removability property were run by an
internal script and by **neither the build nor CI**, and the new coverage gate itself accepted an
empty `///` as documentation. All four are closed and proven. Nothing is published, because
there is no website; that half of the roadmap line stays `[~]` with the remainder named.

**Suite: 707 cases / 916,222 assertions, green** (from 690 / 916,050). Both CI guards pass; ARM
gate clean at 104 headers; the removability gate is empty; the drift scan reads **0 non-verbatim
lines across 343 quoted lines** in all public documentation.

---

## 1. What was ruled, built, and changed

### Ruled

| # | Question | Ruling |
|---|---|---|
| A1 | Does `Routine` freeze at D3? | **YES** — after the cookbook, never before |
| A2 | Amend F6, or a new row? | **New row F10** |
| A3 | Does the signature pin extend? | **YES** — 37 pins, 16 mutations, all red and named |
| A4 | What is excluded? | **`then()` entirely**, plus the stop/skip log *wording* |
| B1 | Doxygen or a custom extractor? | **Custom extractor** — one parser feeds the reference *and* the gate |
| B2 | Chapter 10 vs the generated reference | **Ch. 10 = how to think; the reference = what exists.** Stated as a rule in both |
| B3 | Where does the cookbook live? | **`docs/cookbook/`**, its own document |
| B4 | Committed output or built on demand? | **Committed**, with a build-time regeneration check |

### Built

| Artifact | Where | Size |
|---|---|---|
| The cookbook | `docs/cookbook/` (index + 5 chapters, 14 recipes) | 6 files |
| Its compiled examples | `test/cookbook_examples_test.cpp` | 11 cases / 95 assertions |
| The generated API reference | `docs/api/` (index + 2 pages) | 1,385 lines, generated |
| The documentation tool | `tools/api_doc_tool.py` | 5 subcommands |
| The fidelity pin | `test/api_reference_fidelity_test.cpp` | 3 cases / 62 assertions |
| The `Routine` freeze pin | `test/routine_signature_pin_test.cpp` | 37 pins + 1 case |
| The README example, finally compiled | `test/readme_example_test.cpp` | 1 case / 4 assertions |
| The publish path | `docs/internal/docs-publishing.md` | — |
| Verification harness | `docs/internal/verify/verify-d3.sh` | 12 sections |

### Changed (and why)

- **`include/shulib/chassis/routine.hpp`** — the STATUS banner flips to FROZEN (F10) with the
  exclusions stated; `lastTrajectory()`'s initial value fixed (§8.1, a real bug); doc comments
  added for the class, the deleted-special-member run, and `RoutineResult::cause` (the coverage
  gate found them).
- **`include/shulib/chassis/chassis.hpp`** — **comment-only.** Doc comments added for `class
  Chassis`, the deleted-special-member run, `MotionOptions::validate()`,
  `TrajectoryResult::succeeded()` and the const `scheduler()` overload; the F6 banner's
  `Routine` exclusion updated; and one internal path removed from the banner (§8.5). **No
  signature was touched** — the F6 pin is still green, re-verified.
- **`test/chassis_recipe_test.cpp`** — one new case pinning §8.1's fix, added in the layer that
  owns it (rule 4).
- **`test/CMakeLists.txt`, `.github/workflows/ci.yml`** — the four documentation gates, wired
  where they cannot be forgotten.
- **The documentation sweep** — roadmap (F10 row, F6 row, "you are here", M7 checkboxes), guide
  chapters 07 / 09 / 10 / 14 and the guide README, `guide-maintenance.md`, the root README,
  `test/README.md`, and one re-fenced block in the master plan.

---

## 2. THE `ROUTINE` CRITIQUE — A1's input packet

> **The chunk's real job**, in the shape of `D1-COMPLETED.md` §2: what it was like to build on
> `Routine` as its second consumer. Each item: what was hit, the code that shows it, and a
> recommendation. Ordered by how much the freeze ruling should care.
>
> **The frame that makes this critique different from D1's.** D1's items were mostly
> *now-or-never*: a facade freeze was one chunk away and several fixes would have become
> breaking migrations. Almost nothing here is now-or-never, and that is the single most
> important input to A1. Under `version.hpp`'s policy a **new member is additive** — so a chain
> deadline, a `.finally()`, an `.attempt()`, an alias for `startAt` can all arrive later without
> a major bump. The question the freeze actually had to answer was narrower: *is any of these
> awkwardnesses only fixable by changing something that exists?* The answer is no, once — and
> that one case (2.2) is handled by excluding it from the freeze rather than by changing it.

### 2.1 There is no whole-chain deadline, and the clock is Tier-3 — ACCEPT, ADDITIVE LATER

A `Routine` has per-step timeouts and nothing else. If an early leg burns six seconds fighting a
defender, every later leg still gets its full budget, and the routine runs past the buzzer. VEX
autonomous is fifteen seconds; this is not a corner case, it is the shape of the problem.

Writing the recipe for it (`cookbook-03b`) showed the cost precisely. The routine has to be
broken into phases with an explicit elapsed-time check between them, the unconditional park has
to live outside the chain — and reading the clock at all needs this:

```cpp
Time clockNow(Chassis& chassis) { return chassis.deps().ctx->clock().now(); }
```

`deps()` is the **Tier-3 seam**. A Tier-2 concept reaching through it is exactly the smell D1
§2.2 raised for `pause()` and D2 fixed by adding `wait()`.

**Recommendation: accept at the freeze; add later, additively, and only when someone asks
twice.** Two shapes are additive and neither is foreclosed: `Chassis::clock()` (an accessor,
minor bump) and `Routine::elapsed()` / `Routine::deadline(Time)`.

**The one thing that must be said now, because it is the part a freeze can get wrong:** if a
deadline is ever added, it must be **opt-in and inert by default**. A `deadline()` that silently
clamped every subsequent step's timeout would change the behaviour of existing routines — a
*breaking* change wearing an additive costume. Whoever builds it should read this paragraph
first. **Rejected: add `elapsed()`/`deadline()` now** — the brief forbids inventing capability
to make a recipe read nicely, the semantics deserve their own design (does a blown deadline stop
the chain? does it count as a step? does it interact with `skipped`?), and a freeze chunk is the
worst possible place to invent semantics.

### 2.2 `then()` is doing three unrelated jobs — EXCLUDE FROM THE FREEZE *(rules A4)*

The cookbook used `then()` for three things D1 did not anticipate all of:

1. **mechanism glue** (`then([&]{ intake.release(); }, "release")`) — the intended use;
2. **verdict-honoring mixed-tier calls** (`then([&]{ return chassis.moveTo(...); }, "approach")`)
   — the *fix* for 2.4, and it works only because D1's `ExitReason` arm exists;
3. **deliberate failure-swallowing** (`then([&]{ outcomes.push_back(intake.grab()); })`) — the
   only way to express "attempt this, keep going" (2.5).

Three jobs on one placeholder is a signal. Add the ergonomic wart — `name` defaults to
`"action"`, so a routine with four unnamed actions logs `STOPPED at step 7 (action)` four
indistinguishable ways, exactly when a transcript matters — and the case is clear.

**Recommendation: exclude `then()` from the freeze entirely, and say so out loud.** Not
"freeze the name and leave the contract open": the whole member, out. Mechanisms do not exist
(F1/F3 build them); its accepted return types and its `name` default were chosen before there
was anything real to plug in. **Rejected: freeze it with `name` made required** — that fixes the
wart but locks the *return-type contract*, which is the part F1/F3 are most likely to want back;
and making a parameter required is the kind of change that is free today and breaking forever
after, which is precisely why it should not be spent on a placeholder. **Also rejected: freeze
it as-is** — silence would have read as a promise about code nobody has written.

### 2.3 There is no `.finally()`, and every serious recipe wants one — ACCEPT, ADDITIVE LATER

Two of fourteen recipes needed "do this whatever happens": park after a failed grab
(`cookbook-02a`), park at the end of a budgeted auton (`cookbook-03b`). Both end up with a
**second `Routine`** and a saved result:

```cpp
const RoutineResult failure = r.result();
Routine bail{chassis, "grab-or-bail/fallback"};
bail.driveTo(-48_in, -48_in, {.timeout = 6_s}).brake({.timeout = 1.5_s});
return failure;
```

This is D1's D11 deferral (`"deliberately left OUT of the surface until someone real asks"`).
**D3 is someone real asking** — twice out of fourteen.

**Recommendation: accept for the freeze; `.finally(action)` is additive.** The reason not to
build it now is not cost, it is that its semantics are genuinely non-obvious and each choice is
load-bearing: does it run after a stop (yes, presumably) — but does its own failure re-stop an
already-stopped chain? does it count in `steps`? does it clear `ok()`? A freeze chunk inventing
four answers under time pressure is how a surface acquires a wart it can never remove.
**What D3 did instead: document the second-chain idiom**, so teams do not each invent a
different one (`docs/cookbook/02-when-a-step-fails.md`). **Rejected: build `.finally()` now.**

### 2.4 A bare facade call mid-chain is invisible to the chain — ACCEPT, DOCUMENT LOUDLY

The sharpest footgun found, and it is not a spelling problem. Chapter 9 says mixing tiers is
"fully supported", which is true and incomplete:

```cpp
const ExitReason direct = c.chassis.moveTo(Pose2d{60_in, 0_in, 0_deg}, {.timeout = 0.3_s});
CHECK(direct == ExitReason::TimedOut);
r.brake({.timeout = 1.5_s});
CHECK(r.ok());              // the chain is cheerful…
CHECK(r.result().steps == 2);  // …and never counted the failed move
```

`ok()` stays **true** while the robot is somewhere it was never supposed to be, and every
following step runs from there. Everything the recipe layer does about failure applies to steps
only.

**Recommendation: accept — this is the tier model, not a defect — and close the documentation
gap, which D3 did.** The fix is `then()`-wrapping (2.2's second job) and it already works; what
was missing was anyone saying so. Chapter 9 now carries the warning and the fix; the cookbook
shows both forms side by side with a compiled test holding each true, so neither the warning nor
the advice can go stale alone. **Rejected: have `Routine` detect it** (compare
`scheduler().motionsStarted()` between steps) — it couples the chain to the scheduler's
counters, produces false positives on any `drive()` loop, and would make a *silent* guess about
whether a call the author made deliberately was a mistake.

### 2.5 A failure you want to survive must be swallowed silently — ACCEPT, ADDITIVE LATER

"Sweep three goals; a miss on the middle one is disappointing, not fatal." `then()` stops the
chain on `false`, so the only expressible form returns `void` and stashes the outcome:

```cpp
Routine& attemptGrab(Routine& r, Intake& intake, std::vector<bool>& outcomes) {
    return r.then([&intake, &outcomes] { outcomes.push_back(intake.grab()); },
                  "attempt-grab");
}
```

The cost is real and the cookbook asserts it: `routineWarnings(log) == 0` — **the routine layer
says nothing about the miss.** There is no way to record "this step failed, continue".

Worse, the tolerant and intolerant forms differ by the word `return`. `{ outcomes.push_back(...); }`
and `{ return intake.grab(); }` are one keyword apart and behave oppositely.

**Recommendation: accept for the freeze; `.attempt(action, name)` is additive** — a step that
records a failure (a new `RoutineResult` field with a 0 default, itself additive) without
stopping. Do not build it now: the brief forbids inventing capability, and the swallow idiom is
honest once documented. **Rejected: make `then()` continue on `false` and add a strict variant**
— that inverts the safe default, and a routine that keeps driving after a failure is the exact
behaviour D1's error policy exists to prevent.

### 2.6 `result().exit` reads `Running` after a non-motion stop — ACCEPT, WITH A READING RULE

After a failed action or a timed-out wait, `exit` is `Running`, which to a first-year reads as
"still going" on a routine that has stopped. `cookbook-02a` pins it deliberately.

**Recommendation: accept.** It is `CompletedMotion`'s existing "no verdict here yet" convention,
and consistency across three layers beats local clarity in one struct — a fourth spelling of
"nothing yet" would be worse. **What D3 did: made the reading rule explicit** in the header
(`RoutineResult::cause` now says *read this BEFORE `exit`*), in chapter 9, and in the cookbook.
**Rejected: add `ExitReason::None`** — that is a `control::` change rippling through C1/C2/C5
for a Tier-2 readability nit, and it would re-mean an enum used in three layers.

### 2.7 `startAt` is the only re-seed spelling — ACCEPT, WITH THE NAME EXPLAINED

D1's D12 predicted this: `startAt` is "the honest place for the relocalization idiom later". The
cookbook wrote that recipe (`cookbook-05b`) and the prediction held — the *behaviour* is exactly
right, and the *name* reads wrong in the middle of a routine.

**Recommendation: accept.** Renaming is breaking and would touch every routine and chapter for a
one-line documentation cost; adding a `.setPose()` alias means two spellings for one action in a
teaching API, which is worse than one slightly-wrong name until somebody asks twice. Both remain
additively available. **What D3 did:** the cookbook says *"read it as 'the robot is at', not
'the robot begins at'"*. **Rejected: rename to `seedPose`/`relocalize`** (breaking, for a name);
**rejected: add an alias now** (surface bloat at a freeze).

### 2.8 `pause` vs the facade's `wait` — ACCEPT, and this one is *not* the obvious call

`Routine::pause` is a pure delegation to `Chassis::wait` (D2 made it one). Identical semantics,
different name — and it is the *only* step where that is true: `startAt`/`face`/`driveTo` differ
deliberately (field vocabulary), `waitFor`/`waitUntil` differ deliberately (different failure
semantics). The header's own claim — *"dropping from Tier 2 to Tier 3 is deleting the `Routine`
object and keeping every verb"* — is not literally true for this one step.

The tempting fix is to rename `pause` → `wait` while it is still free. **Rejected, and this is
the ruling that most deserved its reasoning written down:** it would put `wait(2_s)` and
`waitFor(pred, 2_s)` next to each other in the same API, reading as two forms of one verb, when
one can never fail and the other **stops the chain**. Same-family names with opposite failure
semantics are a more dangerous confusion than a name that differs across tiers — and the tier
difference costs exactly one line in chapter 9's table, which already carries it.
**Recommendation: accept `pause`, and keep the reason on the record**, because "make the names
match" is the kind of tidiness that will be proposed again by someone who has not thought about
`waitFor`.

### 2.9 What was genuinely good to build on

Honesty cuts both ways, and these D1/D2 shapes did exactly what their decision logs claimed:

- **Eager execution (D1's D1)** is why every mixed-tier recipe "just worked". A direct call
  between two steps needs no explanation because there is nothing to splice into. It is also
  why the composed helper `Routine& scoreAt(Routine&, ...)` is possible at all — a deferred
  chain would have needed a splice API.
- **Stop / safe / skip / report (D1's D2)** meant **not one recipe wrote safety code.**
  `cookbook-02a`'s `if (r.ok())` is a *strategy* branch, not a safety branch: by the time it
  runs, the drive is already at 0 V under brake. That is precisely what D1 claimed and it held.
- **`then()` accepting `ExitReason` (D1's D7)** was built for mechanisms and its first real use
  was something else entirely — it is the whole fix for 2.4. Seams pay off in unplanned ways.
- **Run-time bearing computation in `face`/`driveTo` (D1's D4)** is what makes tank recipes
  robust to a leg ending slightly short.
- **`lastTrajectory()` (D1's M13)** was needed exactly once, in exactly the way D1 predicted, and
  could not have been added later without the member existing.
- **Typed time (D2's B1)** made the thousand-fold mistake unwritable in fourteen recipes.
- **Non-copyable / non-movable (D1's D9)** never once got in the way; `Routine&`-taking helpers
  cover composition completely.

**The headline A1 should hear:** fourteen recipes, three drivetrains, eleven test cases —
**zero changes to the surface were required.** The critique above is real, and every item in it
is additive.

---

## 3. THE DECISION DOCKET — every ruling, with its rejected alternative

### 3.A The `Routine` freeze

#### A1 — `Routine` FREEZES at D3

Ruled on three things, checked rather than assumed:

1. **The consumer test passed.** The cookbook is a genuine second consumer — fourteen recipes,
   three drivetrains, real VEX U shapes (multi-goal side run, bail-out, partner wait, tank
   routine, mixed tiers) — and required **zero** surface changes. That is the same evidence
   D1 produced for the facade, and D2 acted on it.
2. **Nothing in the critique is foreclosed.** §2's items are additive to a member; the one
   genuine now-or-never (`then()`) is handled by exclusion, not by a change.
3. **Not freezing has a cost that was easy to overlook.** Chapter 9 currently tells readers
   *"hold that last part loosely until D3"*. Leaving `Routine` unfrozen past D3 means that hedge
   becomes indefinite — a documented promise with no date, which is the two-contradictory-claims
   state D2's constraint 5 exists to prevent, and which quietly discourages teams from writing
   against the easier tier at all.

**Rejected: defer to Phase F′** (the competition routines, the real mass consumer). It is a fair
argument and it loses to the same reasoning that governed F6: Phase F′ does not exist and has no
date, additive growth covers what it will want, and a surface that waits for its *largest*
consumer waits forever. If F′ finds a genuine reshape, that is what a major bump plus a
migration note is for — and this record is the evidence for whether the wait would have helped.

#### A2 — A NEW register row (F10), not an F6 amendment

`Routine` is a **strict client** of the facade, one tier up, and the two version independently:
an additive `Routine::deadline()` implies nothing about `Chassis`, and a facade addition implies
nothing about the recipe layer. A separate row also keeps each freeze's own lock date, member
list, and consumer story legible — a reader in two years can ask "what exactly did F6 promise on
2026-08-12" and get an answer that has not been edited by a later chunk.

**Rejected: amend F6.** It would retroactively blur F6's own promise, it would assert that the
two surfaces always move together (they do not), and it would mean the F6 row's carefully
written `Routine` *exclusion* had to be deleted rather than resolved — losing the record of a
decision that was correct.

#### A3 — The pin extends: 37 compile-time pins in `test/routine_signature_pin_test.cpp`

D2's own sentence: *a freeze without one is a comment.* Coverage was cross-checked
member-by-member against the header: the constructor (both spellings + `noexcept`),
non-copy/move, all eleven steps, all four observers, all eight `RoutineResult` fields, all four
`RoutineStopCause` **values**, the terse defaulted spellings, and three **negative** pins.
Additive extension stays legal by construction (`static_cast` overload selection, never
`decltype(&member)`). §7 has the design; §10 has the 16 mutations.

#### A4 — What is EXCLUDED, said out loud

- **`then()`** — entirely (§2.2). Named in the register row, in `routine.hpp`'s banner, in
  chapter 9, in chapter 14, and in a prominent comment in the pin file explaining that its
  absence there is deliberate.
- **The stop/skip log line WORDING.** The *behaviour* is frozen — one `Warn` naming the routine
  and the failing step, one `Info` per skipped step — because a routine that stops silently is
  the failure the policy exists to prevent. The exact sentence is not: tests assert it, teams do
  not parse it, and freezing prose would make a typo fix a major version bump.

Silence in a freeze reads as "frozen too" (D2's A2 lesson), so both exclusions are stated in
every place the freeze is stated.

### 3.B The documentation architecture

#### B1 — A custom extractor, not Doxygen

Decided on evidence, and the decisive point is not the one the brief expected.

*Against Doxygen:* it is a new toolchain dependency (verified: `which doxygen` → not installed
on the development machine), it has no native markdown emitter (needs doxybook/moxygen on top —
two dependencies, not one), and its house style fights this repo's plain-English voice.

*The decisive point:* **the generator and the coverage gate must agree, exactly, on what "a
public member" and "documented" mean.** With Doxygen they are two tools with two notions —
Doxygen's `WARN_IF_UNDOCUMENTED` versus whatever a markdown converter emits — and two notions
disagree eventually, silently, in the direction that looks fine. One parser makes the agreement
structural. `tools/api_doc_tool.py` is ~1,050 lines including its own self-test, and it is code
we own and can be blamed for.

**Rejected: Doxygen** (above). **Rejected: no generator, hand-maintained reference** — that is
the staleness this chunk exists to fight. **The honest cost of the ruling:** the extractor is
*not* a C++ parser and must never pretend to be one. It understands this repository's house
comment style and two named headers; pointed at anything else it is required to **fail loudly**
rather than guess — and it does: a public nested type raises with a message saying the tool must
be extended deliberately.

#### B2 — Chapter 10 teaches; the generated reference enumerates

The rule, now stated in *both* documents and in `guide-maintenance.md`:

> Chapter 10 is **how to think about the API** — when to reach for a verb, what it does when
> things go wrong, the gotchas, worked idioms. `docs/api/` is **exactly what exists** — every
> member, its precise signature, its documentation. Chapter 10 names verbs and arguments
> conversationally and **never restates a signature**. If the two disagree, the reference is
> right, because nobody typed it.

That last clause is the operative one: it tells a reader what to do at the moment of conflict,
which is the only moment the rule matters. **Rejected: fold chapter 10 into the reference** —
prose about *when* to use a verb is not derivable from its signature, and a generated page in
the guide's voice is a signature list wearing a hat. **Rejected: leave the relationship
implicit** — two documents that can disagree, with no stated authority, is the rot this chunk
exists to prevent.

#### B3 — The cookbook lives in `docs/cookbook/`, not as more guide chapters

A cookbook is read **out of order, mid-task**; a manual is read **in order, while learning**.
Merging them means either the guide grows chapters nobody reads front-to-back, or the cookbook
inherits a reading order it does not have. Separate documents also let each keep its own shape:
every recipe is *use this when / the recipe / why it is written this way / watch out for*, which
would be a jarring format inside the guide.

**Rejected: `docs/guide/16..20`** — see above. **The real cost of separating, named honestly:**
a second place to keep true, and a second place a reader can fail to find. Both are mitigated,
not eliminated: the guide README now opens with a three-document table saying which question
each answers, the root README links all three, and chapters 9 and 10 point at the cookbook. The
anti-rot mechanism is *shared*, not duplicated — the same verbatim scan covers both.

#### B4 — Generated output is COMMITTED, with a build-time regeneration check

Committed, because a reference that does not exist until someone runs a tool is not a reference:
a student browsing the repository on a phone in the pits must be able to read it, and GitHub
renders it as-is. The staleness that committing invites is answered mechanically —
`check-fresh` regenerates into a temporary directory and compares **the whole directory**
byte-for-byte, so a stale page, a missing page, *and* a hand-written file dropped into
`docs/api/` all fail the build with the exact command to fix it.

That whole-directory comparison is what lets the verbatim scan skip `docs/api/` without leaving
a loophole: every public document is covered by exactly one mechanism, and the partition is
enforced rather than promised. **Rejected: build on demand** (the reference would not exist for
most readers). **Rejected: commit without a check** (that is the stale-doc failure, with extra
steps).

---

## 4. The cookbook as a critical consumer — what it actually proved

Fourteen recipes in six files, chosen as things a real VEX U auton does rather than as things
the API happens to do well:

| Recipe | What it stresses |
|---|---|
| The skeleton | The minimum honest routine: seed, work, park |
| A reusable scoring step | Composition — `Routine&` in, `Routine&` out |
| A two-goal side run | A full auton inside a match window (7.5 s measured) |
| A waypoint sweep | `followTrajectory` + per-leg options |
| Bail out and park somewhere useful | The error policy, and the missing `.finally()` |
| Attempt something and keep going | The absence of a tolerant step |
| Recover from a broken sweep | `lastTrajectory()`'s partial progress |
| Wait for your alliance partner | `pause`, clock-asserted |
| Wait for a condition, with a deadline | `waitFor`'s stop-on-timeout |
| Wait, but go anyway | The `(void) waitUntil` inside `then()` idiom |
| Fit the match window | The missing chain deadline; the Tier-3 clock |
| A tank routine | `face` + `driveTo`, drivetrain honesty |
| Budget a sideways leg | `strafeAuthority()` at authoring time |
| Call the full API mid-routine / make it count | The mixed-tier footgun and its fix |
| Re-seed the estimate mid-routine | `startAt`'s second life |

**Every recipe runs.** Not "compiles" — runs, against the A2 plant, graded on the simulator's
ground truth, which the estimator cannot see.

**Timing claims are asserted against the simulated clock, and the bounds were measured, not
guessed.** Every clock bound was calibrated by forcing it to `< 0.0`, reading the real elapsed
time off the failure message, and setting the bound with honest headroom:

| Case | measured | bound |
|---|---|---|
| `cookbook-01a` skeleton | 1.65 s | < 4.0 s |
| `cookbook-01b` two-goal side run | 7.52 s | < 12.0 s |
| `cookbook-03b` budgeted, tight | 5.30 s | < 9.0 s |
| `cookbook-03b` budgeted, generous | 8.19 s | **> 6.0 s** and < 13.0 s |
| `cookbook-04a` tank routine | 5.88 s | < 10.0 s |
| `cookbook-04b` H-drive lateral | 1.46 s | < 4.0 s |

The generous-budget case has a **lower** bound as well, which is the interesting one: without
it, "the budget check works" would have been provable by a routine that never did anything.
This is D2's hole #2 (`300_s` for `300_ms`) taken as a standing instruction rather than a
one-off fix.

---

## 5. The generated reference

`docs/api/` is three files, all generated: an index (every member, alphabetical, linked) and one
page per documented header. Each page carries, for every public member: the **rendered
signature**, its documentation, its kind, and a line-numbered link into the header — plus the
header's full design banner, reproduced because a reference that lists signatures teaches nobody
*why*.

Three properties were designed in, each because a specific failure was foreseeable:

- **Determinism.** No timestamps, no dict-order dependence, source-ordered members. Without it
  B4's regeneration check would fail at random and be switched off within a week. Proven by
  self-test (render twice, generate twice, byte-identical) and by the verify harness.
- **Unique anchors.** Overloaded members share a name, and a name-derived anchor sends every
  link to the first one. `scheduler()` and `scheduler() const` get distinct anchors and distinct
  labels. A reference whose links go to the wrong member is worse than one with no links.
- **Explicit `<a id>` anchors, CommonMark only, relative links.** Heading-derived anchors differ
  between GitHub, MkDocs, Docusaurus and mdBook. The output renders identically everywhere,
  which is what "web-portable" has to mean if nothing is hosted yet.

**One parser bug found while self-testing, worth recording** because it would have produced a
plausible-looking wrong reference rather than an obvious failure: "a declaration ends at the
first `;`" cuts in the wrong place for a one-line inline function (`int f() const { return x; }`
— the `;` is inside the body), gluing the body onto the signature. Termination is now the first
`;`, `{`, or constructor-init `:` outside parentheses, angle brackets, string literals and
`::` — and a `{` on a declaration with no parameter list is treated as a brace *initializer*, so
`units::Time timeout{0.0}` keeps its default instead of rendering as `units::Time timeout`.

---

## 6. The doc-coverage gate — and what it found on its first run

The gate is the piece that makes "generated" mean "complete". Generation alone does not: a
generator faithfully extracts whatever comments exist, so a member added with no comment is
silently absent and the reference still looks finished. Nothing looks wrong, which is worse than
a stale document.

The rule, stated precisely because a fuzzy rule is a loophole:

> A public member is documented if a `///` block appears immediately above it (no blank line),
> or a `///<` comment sits on its last line. A **run of `= delete` / `= default` special
> members** may share one comment placed above the run. Any other declaration **consumes** the
> pending comment, so a member added directly below a documented one does **not** inherit its
> neighbour's documentation. Documentation must contain non-whitespace.

The last two clauses are each there because a mutation proved they had to be (§10, D6 and H3).

### What it found the first time it ran — 16 undocumented public members

On the surface about to be frozen forever:

| Undocumented | Where |
|---|---|
| `class Chassis` itself | `chassis.hpp:222` |
| `class Routine` itself | `routine.hpp:153` |
| `MotionOptions::validate()` | `chassis.hpp:197` |
| `TrajectoryResult::succeeded()` | `chassis.hpp:217` |
| `Chassis::scheduler() const` — the const overload only | `chassis.hpp:469` |
| `Chassis` copy/move ctors, assignments, destructor (5) | `chassis.hpp:236–240` |
| `Routine` copy/move ctors, assignments, destructor (5) | `routine.hpp:161–165` |
| `RoutineResult::cause` | `routine.hpp:146` |

Look at the *shape* of that list. The two most-read classes in the library had no one-line "what
am I". The const `scheduler()` overload was undocumented while its non-const twin was documented
— a reader could reasonably have concluded it does something different.
`TrajectoryResult::succeeded()`, the member whose default value turned out to be a bug (§8.1),
had no documentation at all. **None of this was visible before the gate existed, and all of it
was about to be frozen.**

All 16 are now documented. `chassis.hpp` is FROZEN and the additions are **comment-only** — no
signature was touched, and the F6 pin was re-run green to prove it.

---

## 7. The F10 pin

37 pins, one uniform macro whose failure text names F10, the lock date, and the exact procedure
for an *intended* break. It inherits both of D2's hard-won lessons and re-proves them:

- **Every member pin is a concept templated on the class.** In a non-dependent context an
  invalid `static_cast` inside a requires-expression is a hard error and the named assert never
  speaks; the bar is "fails the build **naming** the freeze", and "fails the build" is not that.
- **Every `noexcept`-carrying pin pairs the cast with `{ call } noexcept`.** D2's campaign hole
  #1: for a *non-overloaded* member the compiler accepts a cast that *adds* `noexcept`, so the
  cast alone cannot see it being dropped. `Routine` has **five** such members and not one is
  overloaded — `ok`, `result`, `lastTrajectory`, `chassis`, and the constructor — so an
  exact-cast-only pin would have been decoration on the entire observability surface. All five
  drops are red, with the compound requirement as the sole detector (§10, N1–N5).

**Three negative pins**, because some frozen things are semantics rather than signatures:
`hold(0.3)` and `pause(0.2)` must stay uncompilable (typed time — adding an implicit conversion
would move no member-pointer type while reopening exactly the bug D2's retype killed), and a
step must stay non-callable on a `const Routine&` (every step mutates the chain's counters; a
const-callable step would be a silent lie about that).

**`RoutineStopCause`'s numeric values are pinned, not just its names**, because the enum is
append-only and a *re-meaning* — inserting an enumerator in the middle — is invisible at every
call site while changing what every stored value means.

---

## 8. Findings

### 8.1 A REAL BUG: a routine with no trajectory reported a successful one

`Routine::lastTrajectory()` returned a value-initialized `TrajectoryResult` until a trajectory
ran — `{exit = Settled, completedLegs = 0, totalLegs = 0}` — and `succeeded()` reads that as
**success**. Verified with a standalone probe:
`virgin TrajectoryResult.succeeded() = 1 (legs 0/0)`.

Why it is a lie only at this layer: `Chassis::followTrajectory` requires at least one waypoint
(precondition) and always returns a populated result, so `succeeded()` is correct there — and
`TrajectoryResult` is **F6-frozen** and must not change. The recipe layer creates the member at
*construction*, before any trajectory exists, so the default value is this layer's problem.

The case that would actually mislead someone: a `followTrajectory` step **skipped** after an
earlier stop. `if (r.lastTrajectory().succeeded())` takes the success branch on a trajectory that
never ran.

**Fixed in the layer that owns it (rule 4), with zero surface change:** the member initializes to
`{.exit = control::ExitReason::Running}` — the project's own "no verdict here yet" convention,
already used by `RoutineResult::exit` and `CompletedMotion` for exactly this. Pinned by a new
case in `test/chassis_recipe_test.cpp` covering virgin / after-non-trajectory-steps / after-a-skip;
mutation R4 (revert the initializer) is red by exactly that case and nothing else.

### 8.2 GREEN HOLE #1 — the verbatim anti-rot rule was never mechanized

**Found by mutations H1 and H6, both GREEN.** The rule "every ```` ```cpp ```` line in the docs
appears verbatim in a compiled test" has existed since C8. It is the mechanism this chunk was
told to *extend rather than compete with*. It lived **only** in
`docs/internal/verify/verify-d2.sh` — never in the build, never in CI.

H6 makes the point better than any argument: a chapter's `.hold(300_ms)` retyped to
`.hold(300_s)` — **D2's own green hole, this time in the documentation** — built clean and
passed the entire 707-case suite.

**Closed:** `api_doc_tool.py check-examples`, wired into the build and into CI, scanning all
public markdown against a **glob** of `test/*example*_test.cpp` (a gate you must remember to
update when adding a file is a gate that rots — the ARM gate's generated header list, again).
Both probes now RED.

**And it found two real gaps immediately, both pre-existing:**

1. **`README.md`'s front-page example was never compiled.** Every other public code block has
   been verbatim-quoted from a compiled test since C8; the README's was swept **by hand** at D2's
   retype — the exact failure mode the rule exists to remove, on the first thing a stranger
   reads. Fixed where it lives: `test/readme_example_test.cpp`, case `readme-a`, the block
   quoted verbatim including its `#include`/`using` lines, graded on ground truth.
2. **`docs/shulib-v2-master-plan.md`** carried a ```` ```cpp ```` block for `runner.on(...)` — a
   sketch of `PathRunner`, which does not exist. It cannot be compiled, so it cannot be a
   `cpp` block under this rule; re-fenced as `text` with one sentence saying why.

### 8.3 GREEN HOLE #2 — the removability property was never mechanized either

**Found by mutation H7, GREEN.** C7 established that no public document may reference
`docs/internal/`, because that directory is dropped at the squash-merge to `main`. Enforcement
was a grep in an internal script. A public doc could link into the development log and stay
green until the link broke in published documentation — where a reader, not a maintainer, finds
it. **Closed:** `check-removability`, same wiring. RED now.

Its first catch was the person who wrote it: a roadmap sentence saying *"build-order's next
chunk"* tripped the four-term grep during this chunk's own documentation sweep.

### 8.4 GREEN HOLE #3 — the coverage gate accepted EMPTY documentation

**Found by hole probe H3.** A bare `///` with no text satisfied `bool(self.doc)`; the member
passed and the reference rendered an empty paragraph. **The gate reproduced, inside itself, the
precise "nothing looks wrong" failure it exists to prevent.**

Verified in isolation (a bare `///` above `Routine::ok`, `check-coverage` exit **0**). Closed:
documentation must contain non-whitespace. Proven both directions — the empty `///` now names
`Routine::ok`, and reverting the one-line fix fails the self-test with *"an EMPTY /// must not
count as documented"*.

### 8.5 GREEN HOLE #4 — the generated reference leaked an internal path into public docs

**Found by the D3 verify harness, in a gate written twenty minutes earlier.**
`docs/api/chassis.md:546` contained `docs/internal/chunks/C4-COMPLETED.md`: a **public**
document linking into the development log — precisely the C7 violation §8.3's new gate exists to
catch — while that gate reported green.

Cause: `docs/api/` was excluded from the public set because it is generated and covered by
`check-fresh`. That exclusion is right for the *verbatim* scan and wrong for removability —
**freshness proves the file matches the header, not that the header is fit to publish**, and the
reference reproduces each header's banner in full, so an internal path in a `//` comment lands
in a published page.

**Closed twice over:** the source (`chassis.hpp`'s banner now names the development log without
a path, matching `routine.hpp`'s existing spelling — comment-only) and the gate (`docs/api/` is
now scanned for removability while still skipped by the verbatim scan, with the asymmetry and
its reason written into the tool). Proven: planting the path back into the header banner,
regenerating, and running the gate → RED at `docs/api/chassis.md:545`.

### 8.6 The general lesson from #3 and #4

Both gate holes were in **what the gate declined to look at**, not in what it looked at wrongly.
D2 §10.1 said the analogous thing about the noexcept pin (*an exact-type cast is not an
exact-type test*). The D3 version: **a gate's exclusion list is where its holes live.** Every
`skip`, `continue`, or "covered elsewhere" in a checker deserves the same scrutiny as its
matching logic — and a note saying which other mechanism covers the excluded case, so the claim
can be audited instead of assumed.

### 8.7 A self-test gap, found by mutation

Mutation G5 (the generator drops a template head) was **not** caught by the tool's self-test —
the fixture had no template member. It was caught downstream, but "caught downstream" is how a
regression reaches a release. Closed: the fixture gained a template member with a default
argument and an assertion; the reverted fix now fails the self-test.

### 8.8 A process find: the mutation runner corrupted a header

The first runner restored files by reverse text substitution, which cannot undo a **deletion**
(its anchor is the empty string) — it raised mid-restore and left `chassis.hpp` missing a doc
comment. Caught immediately by `check-coverage` (the gate under test catching the tool testing
it), restored by hand, runner rewritten to restore from **file snapshots**. Recorded because it
is exactly the class of mistake the campaign discipline exists to surface, and because the
detection path is worth noticing.

---

## 9. Test inventory (16 new cases; every case names its bug in-file)

**`test/cookbook_examples_test.cpp` (11 cases / 95 assertions)** — every recipe, run against the
plant, graded on ground truth: the skeleton; the two-goal side run with a composed scoring step;
the waypoint sweep plus a broken sweep's partial result; bail-out (both branches — the grab
working and the grab missing, with transcript assertions and the parked-drive check); attempt-and-
continue (including the asserted *silence*); the partner wait (three forms, all clock-bounded);
the match budget (tight and generous, bracketing the branch); the tank routine; the H-drive
lateral budget (naive budget times out, authority-aware one settles); mixed tiers (footgun and
fix); mid-routine re-seeding (estimate moves, ground truth does not).

**`test/api_reference_fidelity_test.cpp` (3 cases / 62 assertions)** — the committed markdown's
rendered signatures against the headers, chosen as the shapes a naive extractor gets wrong: a
const overload beside its non-const twin, `noexcept` accessors, default arguments, a template
member, a field initializer, a brace-list overload, a deleted special member, and typed time.
Plus the index's completeness and its distinguishable overload labels. It **fails** rather than
skips when it cannot open the reference.

**`test/routine_signature_pin_test.cpp` (37 pins + 1 case)** — §7.

**`test/readme_example_test.cpp` (1 case / 4 assertions)** — the front-page example, compiled and
graded, closing §8.2's first gap.

**`test/chassis_recipe_test.cpp` (+1 case)** — §8.1's fix, pinned in the layer that owns it.

**`tools/api_doc_tool.py self-test` (13 fixture assertions, run every build)** — types found;
enumerators as members; `///` above vs `///<` beside; **a member must not inherit its neighbour's
comment**; **an empty `///` must not count**; private members excluded; the `= delete` run rule
*and its non-leak*; overloads not collapsed; `const` / `noexcept` / default arguments preserved;
a template head preserved; a constructor rendered without its init list; the banner carried; and
determinism twice over.

---

## 10. Mutation campaign (61 executed: break → build-gate → run → OBSERVE → restore)

The runner hard-gates on build success and restores from file snapshots (§8.8).

**Doc-coverage gate — 7/7 RED, each naming the right member**

| # | Mutation | Result |
|---|---|---|
| D1 | remove the `///` from `Chassis::pose` | **RED** — names `Chassis::pose` |
| D2 | remove the `///` from `Routine::hold` | **RED** — names `Routine::hold` |
| D3 | **add a new public member with no doc** | **RED** — names `Routine::stepsSoFar` |
| D4 | remove a trailing `///<` | **RED** — names `RoutineResult::steps` |
| D5 | remove the `///` above the `= delete` run | **RED** — names `Routine::operator=` |
| D6 | **add a member directly under a documented one** | **RED** — names `Chassis::poseX` |
| D7 | remove the `///` from the class declaration | **RED** — names `class Chassis` |

**Freshness — 3/3 RED:** hand-edited generated markdown; a header `///` changed without
regenerating; a hand-written file dropped into `docs/api/`.

**Generator fidelity — 15 RED.** G1–G4 (drop `const` / `noexcept` / default arguments / collapse
an overload set) and G6 (non-deterministic output) are caught by the tool's **self-test** at the
build gate, naming the exact rendered signature. G5 exposed §8.7. Then **P1–P9 isolate the C++
fidelity pin** by mutating the *committed markdown* and running the existing binary — the
real-world shape of the failure, a generator regression whose wrong output got committed. All
nine RED with the fidelity test as the **sole** failing case: dropped `const`, dropped
`noexcept`, dropped default argument, a lost overload, a lost template head, `units::Time`
retyped back to a bare double, a member vanished from the index, two overloads merged in the
index, and the header prose dropped while the signature stayed right.

**Recipe behaviour — 12/12 RED:** `then()` ignores `false`; `pause()` delegates half the
duration; `skipIfStopped` disabled; `lastTrajectory_` back to value-initialized (RED by exactly
the new case); `startAt()` stops seeding; `recordStop` stops safing the drive; the bearing loses
the live pose; the cookbook's `lateralBudget` ignores `strafeAuthority`; the budget comparison
inverted; `strafeTo` silently becomes a `moveTo`; the composed `scoreAt` stops chaining;
`then()` stops honouring an `ExitReason`.

**Hole probes — 7 run, 3 GREEN (§8.2–8.4), all closed and re-proven RED.**

**The F10 pin — 16/16 RED, every one naming F10.** Run by compiling the pin TU alone, because
the doc gates run first in the full build and would otherwise mask the pin under test.

| # | Mutation | Pin that fired |
|---|---|---|
| N1 | `ok()` loses `noexcept` (non-overloaded) | `Routine::ok() const noexcept -> bool` |
| N2 | `lastTrajectory()` loses `noexcept` | `...lastTrajectory() const noexcept` |
| N3 | `result()` loses `noexcept` | `...result() const noexcept` |
| N4 | `chassis()` loses `noexcept` | `...chassis() noexcept` |
| N5 | the constructor loses `noexcept` | `Routine's constructor staying noexcept` |
| N6 | a step is renamed | `Routine::startAt(const Pose2d&)` |
| N7 | `hold()` retyped to a bare double | `Routine::hold(units::Time, ...)` |
| N8 | a default argument dropped | the defaulted spellings |
| N9 | a step becomes const-callable | `Routine::turnTo(...)` |
| N10 | `RoutineStopCause` **renumbered** | `MotionFailed == 1` |
| N11 | `stoppedAt` int → long | `RoutineResult::stoppedAt : int` |
| N12 | `Routine` becomes movable | `Routine non-movability` |
| N13 | `waitFor`'s name default dropped | the defaulted spellings |
| N14 | a bare-double `pause` added | the negative pin (typed time) |
| N15 | `stoppedName` → `string_view` | `RoutineResult::stoppedName : const char*` |
| N16 | the brace overload renamed away | `followTrajectory({...}) (brace form)` |

N1–N5 are the D2-hole-#1 shape — five `noexcept` drops on non-overloaded members — and the
compound requirement is the sole detector for all five.

---

## 11. The documentation sweep, atomic

`routine.hpp`'s banner → **FROZEN F10** with both exclusions. `chassis.hpp`'s F6 banner → the
`Routine` exclusion now says *froze at D3 as F10* instead of *unfrozen until D3*. Freeze Register
→ row **F10** (placed after F9, milestone order) and F6's row updated. Roadmap → "you are here"
(both tiers frozen; D3's paragraph; next phase), M7 checkboxes with cited evidence, and the M7
DoD annotated with the honest status of **both** its clauses. Chapter 9 → the notice rewritten to
*frozen*, plus the two documentation gaps the cookbook found (§2.4, §2.6) and a pointer to the
cookbook. Chapter 10 → B2's rule, stated as a rule. Chapter 14 → the "recipe spellings can still
change" limitation **fell**, replaced by the one thing still open (`then()`). Chapter 7 and
`test/README.md` → the `python3` prerequisite and what it is for. The guide README → the
three-document table and the now-mechanized verbatim rule. `guide-maintenance.md` → the D3 row,
the new rules, the "which document owns what" table, and the checklist updates. The root README
→ cookbook and reference. `docs/internal/docs-publishing.md` → new (§13).

Gate greps across every location: **0** stale "unfrozen until D3" claims; **0** stale
"not frozen" claims; the four-term removability grep **empty**.

---

## 12. What we now know for certain, and what we do not

### Known, with evidence

- **`Routine` survived a second consumer with zero changes** — 14 recipes, 3 drivetrains, 11
  cases, all green (§4).
- **The coverage gate catches its own omission**, by name, in seven distinct forms including
  the two subtle ones (a member inheriting its neighbour's comment; an empty `///`) — §10.
- **The generator does not lie about the shapes that matter** — 9 isolated fidelity mutations,
  each caught solely by the fidelity pin (§10).
- **The generator is deterministic** — self-test and harness, byte-identical.
- **The F10 freeze is mechanical**, 16/16 mutations red and named, including all five
  non-overloaded `noexcept` drops (§7, §10).
- **Every public code block in the repository is now machine-checked** against a compiled test,
  in the build and in CI (§8.2). 343 quoted lines, 0 non-verbatim.
- **The library is untouched by all of this.** `include/shulib` never mentions the tool; the
  PROS robot `Makefile` gained nothing; ARM gate clean at 104 headers.

### NOT known, stated plainly

- **Nobody new has read the cookbook.** The DoD clause *"a new reader can write a routine from
  the cookbook alone"* needs a person who has not read the code, and none was available. C8 left
  the same clause open with no owner; D3 does not repeat that — see §14.
- **Nothing is published.** No website, no Pages configuration. The reference is web-portable
  and the publish path is written down; that is not the same thing (§13).
- **No recipe has ever run on a robot.** Everything here is graded on the A2 plant. R3 owns
  "it works".
- **The critique's additive claims are reasoned, not demonstrated.** `.finally()`,
  `.attempt()`, a chain deadline — each is argued to be additive under `version.hpp`'s policy.
  The argument is sound but nobody has built one; if a future chunk finds that a deadline
  *cannot* be added without changing an existing member's behaviour, this record is where the
  reasoning it should attack lives (§2.1's opt-in warning is the load-bearing part).
- **The extractor is not a C++ parser.** It handles two headers in one house style. Pointed
  elsewhere it will fail — loudly, by design, but it will fail.

---

## 13. The publish path, and the honest `[~]`

`docs/internal/docs-publishing.md` is new and carries: why the output is already web-portable
(CommonMark only, explicit anchors, relative links); three concrete options (GitHub Pages with a
sketched workflow and a list of what does not exist yet; publishing the markdown as-is, which
works on a phone in the pits and is the right answer if nobody wants to own a site; VexBuilder
hosting it, which is real but downstream of work that does not exist); and the one rule that
must survive whichever is chosen — **the publishing job must generate or verify freshness**,
because a stale published reference is the same failure at its most authoritative-looking
moment, and `check-fresh` is one line.

The roadmap's M7 line is `[~]`, not `[x]`, and names the remaining half. **Do not read this
chunk as having published anything.**

---

## 14. The new-reader clause, addressed honestly

The DoD says *"a new reader can write a routine from the cookbook alone."* **D3 did not close
it, and cannot.** The cookbook was written by the same process that wrote the library; the only
evidence that matters is a person who has not read the code, given the cookbook and asked to
produce a routine.

What D3 *can* claim, precisely: every recipe compiles, runs, and does what its prose says
(asserted, including timing); every listing is quoted verbatim from the code that proves it; and
the "watch out for" sections were written from the awkwardnesses actually hit while writing the
recipes, not imagined afterwards. That is a necessary condition for the clause and not a
sufficient one.

**Named owner, because C8 left this open with none:** the programming chair, at the next
new-member onboarding — hand over `docs/cookbook/README.md` and nothing else, ask for a
left-side auton, and record where they get stuck. The verify harness ends by asking for exactly
this, so it cannot be quietly skipped.

---

## 15. Freeze Register note (documentation contract #6)

**F10 — Public `Routine` API — ✅ LOCKED 2026-08-12, M7.** The row enumerates the surface by
group (construction, the eleven steps, the four observers, the two types, the documented
semantics), names the exclusions (`then()`, log wording), states *why it is a separate row from
F6*, and carries the enforcement pointers (`test/routine_signature_pin_test.cpp`,
`include/shulib/version.hpp`) so a reader of the register can reach the mechanism from the
promise in one hop.

**F6 is unchanged as a contract.** Its row was edited in one respect only: the `Routine`
exclusion now reads *"they froze at D3 as their own row F10"* instead of *"unfrozen until D3's
cookbook"*. The comment-only additions to `chassis.hpp` (§6) touched no signature, and the F6
pin was re-run green to prove it.

Order was honored, as at D2: the cookbook consumed the surface first, the critique was written,
the coverage gate landed, and **only then** did A1 rule and the badge flip.

---

## 16. Verification (actually run, outputs as observed)

```
$ cmake --build build/test -j"$(nproc)"
[  1%] shulib doc gates: coverage, freshness, verbatim examples, removability
api_doc_tool self-test: OK
doc example scan: 343 quoted lines, all verbatim (3 source files)
removability: no public doc references docs/internal/
[  1%] Built target shulib_doc_gates
[100%] Built target shulib_tests

$ ./build/test/shulib_tests | tail -4
[doctest] test cases:    707 |    707 passed | 0 failed | 3 skipped
[doctest] assertions: 916222 | 916222 passed | 0 failed |
[doctest] Status: SUCCESS!

$ grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib   -> GUARD1 PASS
$ grep -rnE --exclude-dir=sim '... shulib/sim/' include/shulib           -> GUARD2 PASS

$ arm-none-eabi-g++ -std=gnu++20 ... -c /tmp/all.cpp -o /dev/null -Iinclude
   headers: 104        ARM PASS

$ grep -rnE 'internal/|chunks/|RESUMING|build-order' README.md test/README.md \
      docs/*.md docs/guide/*.md
   (empty)             REMOVABILITY PASS
```

Per-group counts: `cookbook-*` 11 cases / 95 assertions; `*doc fidelity*` 3 / 62; `readme-*`
1 / 4. Baseline at chunk start: 690 cases / 916,050 assertions.

`docs/internal/verify/verify-d3.sh` re-derives all of the above from scratch plus: the library
never mentions the tool, the robot Makefile is untouched, the determinism diff, **a planted
undocumented member** (must fail naming `Routine::startAt`), **a planted `noexcept` drop** (must
fail naming F10), the F10 row's completeness and its `then()` exclusion, no stale
"unfrozen until D3" claims, and the publish honesty check. It ends by printing what it cannot
mechanize — including "read a recipe cold" (§14).

---

## 17. Deliberately left for later (named handoffs)

- **`.finally()` / `.attempt()` / a chain deadline** — §2.1, §2.3, §2.5. Additive; the deadline
  carries a warning about what would make it *breaking*.
- **`Chassis::clock()`** — additive, minor bump, would retire §2.1's Tier-3 reach.
- **`then()`'s final shape** — F1/F3, when mechanisms exist. Read §2.2 first.
- **Publishing** — §13. Infrastructure, not documentation.
- **The new-reader validation** — §14, owner named.
- **Documenting more headers** — adding one to `TARGETS` in the tool is the whole cost; the
  gate then covers it. Deliberately not done here: gating a header nobody reviewed would claim
  a coverage guarantee this chunk did not earn.

---

## 18. DoD checklist (brief §Definition of Done)

- [x] **Generated API reference exists, covers the F6 surface + `Routine`, deterministic** —
  `docs/api/` (index + 2 pages, 1,385 lines), determinism proven by self-test and harness.
- [x] **Doc-coverage gate lands and is proven** to fail on an undocumented public member —
  7 mutations, each naming the member; the harness re-proves it from scratch. It found 16 real
  omissions on its first run (§6).
- [x] **Generator fidelity pinned** — `test/api_reference_fidelity_test.cpp`, 3 cases /
  62 assertions; 9 isolated mutations, fidelity pin the sole detector.
- [x] **Cookbook written; examples compiled, quoted verbatim, drift scan zero** — 14 recipes,
  11 cases / 95 assertions; scan reads 343 lines / 0 non-verbatim across all public docs, and
  the scan is now in the build and in CI.
- [x] **A written critique of `Routine`** — §2, eight items, each with a recommendation.
- [x] **A1 ruled** — frozen, register row F10 + a 37-pin signature pin, 16 mutations red.
- [x] **B1–B4 ruled** with reasoning and rejected alternatives — §3.B.
- [~] **Publish path documented; `[~]` if nothing is hosted** — documented in full (§13);
  **nothing is hosted**, and the roadmap says so.
- [~] **The new-reader clause addressed honestly** — §14. Not closed; owner named; the harness
  asks for it.
- [x] **Suite green; both guards pass; ARM gate passes; removability holds** — §16.

**Not committed. Not pushed.** Everything is in the working tree on `shulib-v2`.
