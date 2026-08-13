# The shulib guide

The user guide for shulib, written for a new team member who wants to write autonomous routines
and is not (yet) a robotics expert. **Start at [Chapter 1](01-what-is-this.md)** and read in
order — each chapter assumes the ones before it and says so in its header.

| # | Chapter | What it is |
|---|---|---|
| 1 | [What is this?](01-what-is-this.md) | VEX U, the autonomous problem, what shulib does — and its honest status |
| 2 | [The field and coordinates](02-the-field-and-coordinates.md) | Coordinates, pose, field-relative vs robot-relative |
| 3 | [Knowing where you are](03-knowing-where-you-are.md) | Odometry, drift, what each sensor knows, fusion |
| 4 | [Drivetrains](04-drivetrains.md) | Holonomic vs tank vs H-drive; kinematics; strafe authority |
| 5 | [Getting there: control](05-getting-there.md) | PID without anxiety, feedforward, settling, the watchdog |
| 6 | [How things fail](06-how-things-fail.md) | Drift, slip, dead sensors, brownout — and the fault philosophy |
| 7 | [Getting set up](07-getting-set-up.md) | Build, run the tests, read the output, the repo layout |
| 8 | [Your first routine](08-your-first-routine.md) | The full tutorial, every line explained |
| 9 | [The recipe API](09-the-recipe-api.md) | Tier 2: a routine as a chain of steps, and what happens when one fails |
| 10 | [The API, as prose](10-the-api.md) | Every verb: what, when, failure behavior, gotchas |
| 11 | [Reading the diagnostics](11-reading-the-diagnostics.md) | The transcript line by line; every fault code |
| 12 | [When things go wrong](12-when-things-go-wrong.md) | Symptom-first troubleshooting |
| 13 | [Extending the library](13-extending-the-library.md) | The layers, the testing bar, adding drivetrains/motions |
| 14 | [What it can't do yet](14-what-it-cannot-do-yet.md) | The honest limits, each linked to its tracking doc |
| 15 | [Glossary](15-glossary.md) | Every term, one sentence |

## The other two documents

The guide is one of three, and they answer different questions. Reaching for the wrong one is
the most common way to waste an afternoon.

| Document | Answers | Read it |
|---|---|---|
| **This guide** | *How does any of this work?* | Once, in order, while learning |
| **[The cookbook](../cookbook/README.md)** | *How do I write the routine I am writing right now?* | Out of order, mid-task |
| **[The API reference](../api/README.md)** | *What exactly exists, and what is its exact spelling?* | When you need a signature |

The reference is **generated from the headers** by a tool, so it cannot disagree with the code,
and a public member that ships with no documentation fails the build by name. That means the
guide and the cookbook never need to restate a signature — and must not: a signature copied by
hand is a fact that will go stale.

---

## Maintenance (read before editing the guide)

### How it's organised

Numbered files, one topic each, so updating a topic means editing one file. Chapters 1–6 are
concepts (no code) and should rot slowly; 7–12 touch the code and rot fast — check them first
when the API moves. (Chapter 9 was a deliberately reserved slot from the start; the Tier-2
recipe chapter filled it at chunk D1 as an *added file*, with nothing renumbered — the
pattern to reuse.) Add future chapters either into a reserved slot or at the end (before the
glossary stays the glossary — renumber only as a last resort, in one commit, updating every
cross-link).

### Where the code examples live — the anti-rot rule

**Every code listing in the guide compiles and runs in
[`test/guide_examples_test.cpp`](../../test/guide_examples_test.cpp).** The test cases are named
for their chapters (`guide-08a` → chapter 8, and so on), and the guide quotes their listings
*verbatim*. The rule, in both directions: change code there → update the chapter's listing;
change a listing → update the test. A mismatch is a bug even if both sides work. CI compiles
and runs the file on every commit, so an example that stops building turns the suite red — the
guide cannot silently rot below the compiler's line of sight. Prose *claims* about behavior are
held as assertions in the same cases wherever practical.

**The verbatim half of that rule is now checked by a machine.** Until 2026-08-12 it was checked
only by a person running an internal script — so a listing could drift from the test that
compiles it and pass both the build and CI. (It was found by deliberately introducing the
drift: a chapter's `300_ms` retyped to `300_s` built clean and passed the whole suite.) The
build and CI now run a scan that requires every non-blank line inside a ```` ```cpp ```` block in
any public document to appear verbatim in a compiled example test, and the example sources are
matched by a glob, so a new examples file is covered the moment it exists. The same run also
checks that no public document links into the development-process notes.

**Where this applies:** `docs/guide/`, `docs/cookbook/`, `docs/*.md`, and the repo `README.md`.
It deliberately does *not* apply to `docs/api/`, which is generated in full and checked by a
stronger rule instead — its whole directory must be byte-identical to a fresh generation, so a
hand-written file dropped there fails too. Every public document is covered by exactly one of
the two mechanisms.

Transcript excerpts (chapters 8 and 11) come from real runs:

```sh
SHULIB_GUIDE_PRINT=1 ./build/test/shulib_tests -tc='guide-*'
```

regenerates them. Chapter 11's line formats are additionally byte-pinned by the diagnostics
tests named in that chapter — if formats change, those tests change, and chapter 11 must
follow.

### Facts policy: link, don't restate

Test counts, register sizes, milestone statuses, freeze states — the guide links to the
[README](../../README.md), [roadmap](../roadmap.md),
[Hardware Assumptions Register](../hardware-assumptions.md), and
[diagnostics plan](../diagnostics-plan.md) rather than copying numbers. Where a number genuinely
serves the prose (a transcript, a "57 as of 2026-08-10"), it's dated. If you find an undated
copied fact, that's a bug: link it or date it.

Two structural rules inherited from the repo: the guide is **public documentation** — it may
link only to other public documents (this `docs/` level, the repo README, headers, tests),
never to development-process notes that exist only on a working branch (bare chunk ids like
"D1" or "R1" are fine as vocabulary; the roadmap gives them context). And examples must
respect the CI guards (no PROS includes, no sim includes from core) since they compile inside
the test suite.

### When the API changes, check in this order

1. `test/guide_examples_test.cpp` — does it still compile and pass? (CI answers this for you.)
2. Chapter 10 (the API chapter), then chapter 8's
   listings and transcripts, then chapter 11 if any output format moved. **The `Chassis` API
   is frozen (F6, 2026-08-12):** a deliberate change to a frozen signature is a *breaking*
   change — it rides a major version bump with a migration note
   ([`include/shulib/version.hpp`](../../include/shulib/version.hpp)) and an update to the
   signature pin ([`test/f6_signature_pin_test.cpp`](../../test/f6_signature_pin_test.cpp));
   the chapters follow the change, never lead it.
3. Chapter 14 when a limitation falls (that's the *good* kind of doc rot — celebrate, then
   delete the paragraph and its glossary entries if any).
4. ✅ Executed 2026-08-12 (D3): the recipe spellings froze as register row **F10**, so chapter
   9's stability notice and chapter 14's "recipe spellings" bullet were rewritten the way D2's
   freeze retired the API notices. Both now say *frozen*, and both name the one deliberate
   exception — `then()`, the mechanism seam, which stays unfrozen until F1/F3 build the
   mechanisms it exists for. A change to a frozen `Routine` spelling is a *breaking* change and
   rides the same procedure as F6's, updating
   [`test/routine_signature_pin_test.cpp`](../../test/routine_signature_pin_test.cpp).
5. The [generated reference](../api/README.md) needs no step at all — it regenerates from the
   headers and the build fails if the committed copy is stale. If you changed a `///` comment,
   run `python3 tools/api_doc_tool.py generate` and commit what it writes.

### Voice

Written the way a patient senior member explains things sitting next to someone. Plain
sentences; no marketing. Define every term at first use (and mirror it in the glossary). Prefer
a concrete number to an abstraction, and a reason to a rule. Admit what's hard and what's
unverified — this library has never driven a robot, and the guide says so wherever it matters.
If a sentence sounds like a tagline, delete it.
