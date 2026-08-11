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
| 9 | *(reserved)* | The recipe API's chapter, when it ships (chunk D1) — see below |
| 10 | [The API, as prose](10-the-api.md) | Every verb: what, when, failure behavior, gotchas |
| 11 | [Reading the diagnostics](11-reading-the-diagnostics.md) | The transcript line by line; every fault code |
| 12 | [When things go wrong](12-when-things-go-wrong.md) | Symptom-first troubleshooting |
| 13 | [Extending the library](13-extending-the-library.md) | The layers, the testing bar, adding drivetrains/motions |
| 14 | [What it can't do yet](14-what-it-cannot-do-yet.md) | The honest limits, each linked to its tracking doc |
| 15 | [Glossary](15-glossary.md) | Every term, one sentence |

---

## Maintenance (read before editing the guide)

### How it's organised

Numbered files, one topic each, so updating a topic means editing one file. Chapters 1–6 are
concepts (no code) and should rot slowly; 7–12 touch the code and rot fast — check them first
when the API moves. **Chapter number 9 is deliberately vacant**: it's reserved for the Tier-2
recipe API chapter (chunk D1), so that adding it is *adding a file*, not renumbering eleven.
Add future chapters either into a reserved slot or at the end (before the glossary stays the
glossary — renumber only as a last resort, in one commit, updating every cross-link).

### Where the code examples live — the anti-rot rule

**Every code listing in the guide compiles and runs in
[`test/guide_examples_test.cpp`](../../test/guide_examples_test.cpp).** The test cases are named
for their chapters (`guide-08a` → chapter 8, and so on), and the guide quotes their listings
*verbatim*. The rule, in both directions: change code there → update the chapter's listing;
change a listing → update the test. A mismatch is a bug even if both sides work. CI compiles
and runs the file on every commit, so an example that stops building turns the suite red — the
guide cannot silently rot below the compiler's line of sight. Prose *claims* about behavior are
held as assertions in the same cases wherever practical.

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
2. Chapter 10 (the API chapter — it says of itself which parts are unfrozen), then chapter 8's
   listings and transcripts, then chapter 11 if any output format moved.
3. Chapter 14 when a limitation falls (that's the *good* kind of doc rot — celebrate, then
   delete the paragraph and its glossary entries if any).
4. When F6 freezes (D2): soften the stability warnings in chapters 10 and 14.
5. When D1 lands: write chapter 9 (the recipe API), add its examples as `guide-09*` cases,
   update chapter 8's closing pointer and this README's table.

### Voice

Written the way a patient senior member explains things sitting next to someone. Plain
sentences; no marketing. Define every term at first use (and mirror it in the glossary). Prefer
a concrete number to an abstraction, and a reason to a rule. Admit what's hard and what's
unverified — this library has never run on a robot, and the guide says so wherever it matters.
If a sentence sounds like a tagline, delete it.
