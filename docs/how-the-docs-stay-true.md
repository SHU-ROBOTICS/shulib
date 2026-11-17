# How these pages stay true

Documentation that drifts is worse than none, because people trust it. These rules are for anyone
editing these pages. Most of them are enforced by the build, not by anyone remembering:

| What | Checked by |
|---|---|
| Every code listing compiles and runs, and the page quotes it word for word | `tools/api_doc_tool.py check-examples` |
| The API reference matches the code, and nothing public is undocumented | `tools/api_doc_tool.py check-fresh`, `check-coverage` |
| No public page links into the development notes | `tools/api_doc_tool.py check-removability` |
| Every public page is in the site's menu exactly once | `tools/doc_readability.py check-nav` |
| Limits are plain facts: no page opens with a status disclaimer (the exact words are in the tool) | `tools/doc_readability.py check-tone` |
| On the site, every register row has its own link (`hardware-assumptions.md#ha-136`) | `tools/prepare_site.py check-register` |
| The front page's first-week picture matches guide 21's order | `tools/docs_figures/first_week.py check` |
| "Where your code goes"'s picture matches that page's tables | `tools/docs_figures/build_to_program.py check` |
| A number a page states matches the repository; a file a page names exists | `tools/doc_staleness_audit.py` |

All of them run in every build of the test suite and in CI.

## How it's organised

Numbered files, one topic each, so updating a topic means editing one file. The file names keep
their numbers as stable ids — tests, links and bookmarks use them — but the site's menu is by
task and shows titles only. [How it works](guide/README.md) lists every page by its old number.
The concept pages (1–6) carry no code and should rot slowly; the pages that touch the code rot
fast — check them first when the API moves.

A new page takes the next number that is free in **every** tree — the working branch and every
branch in flight — and joins the menu in `mkdocs.yml` in the same change (`check-nav` fails the
build until it does). Renumber only as a last resort, in one commit, updating every cross-link.

## Where the code examples live — the anti-rot rule

**Every code listing in the guide is compiled and run** — in
[`test/guide_examples_test.cpp`](../test/guide_examples_test.cpp), in another `test/*example*_test.cpp`
file, or in one of the team's files named below — and the guide quotes it *verbatim*. In
`test/guide_examples_test.cpp` the cases are named for their chapters (`guide-08a` →
[chapter 8](guide/08-your-first-routine.md), and so on). The rule, in both directions: change the
code → update the chapter's listing; change a listing → update the code. A mismatch is a bug even if
both sides work. CI compiles and runs those files on every commit, so an example that stops building
turns the suite red — the guide cannot silently rot below the compiler's line of sight. Prose *claims* about
behavior are held as assertions in the same cases wherever practical.

**The verbatim half of that rule is checked by a machine.** Until 2026-08-12 it was checked
only by a person running an internal script — so a listing could drift from the test that
compiles it and pass both the build and CI. (It was found by deliberately introducing the
drift: a chapter's `300_ms` retyped to `300_s` built clean and passed the whole suite.) The
build and CI now run a scan that requires every non-blank line inside a ```` ```cpp ```` block in
any public document to appear verbatim in a compiled source: a compiled example test, or one of
the team's files under `src/team/` that a chapter quotes (today: robot one's driver file,
`src/team/robot_one_driver.hpp`, which the [driver-control recipes](cookbook/06-driver-control.md) quote,
and robot two's autons file,
`src/team/autons.cpp`, which [chapter 16](guide/16-testing-an-auton-on-the-bench.md) and
[chapter 17](guide/17-the-competition-program.md) quote — the host suite compiles it and runs
every auton in it), or one of the route builder's compiled goldens
([chapter 21](guide/21-adding-your-robot-by-hand.md) quotes robot one's map as the builder gave it from
`test/goldens/route_maker/controls_one_builders_map.hpp`, so the team's own changes to its driver file never fail
the scan) — never the rest of `src/`. A listing of a team file must also be one unbroken run of the file's lines, so
a line added to the file inside a part a chapter shows fails the scan too. The sources are matched by globs, one per kind
of file, so a new examples file is covered the moment it exists, and a kind that matches no file
fails the scan. The same run also checks that no public document links into the
development-process notes.

**Where this applies:** `docs/guide/`, `docs/cookbook/`, `docs/*.md`, the repo `README.md` and
`test/README.md`.
It deliberately does *not* apply to `docs/api/`, which is generated in full and checked by a
stronger rule instead — its whole directory must be byte-identical to a fresh generation, so a
hand-written file dropped there fails too. Every public document is covered by exactly one of
the two mechanisms.

Transcript excerpts ([chapter 8](guide/08-your-first-routine.md) and
[chapter 11](guide/11-reading-the-diagnostics.md)) come from real runs:

```sh
SHULIB_GUIDE_PRINT=1 ./build/test/shulib_tests -tc='guide-*'
```

regenerates them. [Chapter 11](guide/11-reading-the-diagnostics.md)'s line formats are
additionally byte-pinned by the diagnostics tests named in that chapter — if formats change,
those tests change, and [chapter 11](guide/11-reading-the-diagnostics.md) must follow.

## Facts policy: link, don't restate

Test counts, register sizes, milestone statuses, freeze states — the guide links to the
[README](../README.md), [roadmap](roadmap.md),
[Hardware Assumptions Register](hardware-assumptions.md), and
[diagnostics plan](diagnostics-plan.md) rather than copying numbers. Where a number genuinely
serves the prose (a transcript, a "57 as of 2026-08-10"), it's dated. If you find an undated
copied fact, that's a bug: link it or date it.

Two structural rules inherited from the repo: the guide is **public documentation** — it may
link only to other public documents (this `docs/` level, the repo README, headers, tests),
never to development-process notes that exist only on a working branch (bare chunk ids like
"D1" or "R1" are fine as vocabulary; the roadmap gives them context). And examples must
respect the CI guards (no PROS includes, no sim includes from core) since they compile inside
the test suite.

## When the API changes, check in this order

1. `test/guide_examples_test.cpp` — does it still compile and pass? (CI answers this for you.)
2. [Chapter 10](guide/10-the-api.md) (the API chapter), then
   [chapter 8](guide/08-your-first-routine.md)'s listings and transcripts, then
   [chapter 11](guide/11-reading-the-diagnostics.md) if any output format moved. **The `Chassis`
   API is frozen (F6, 2026-08-12):** a deliberate change to a frozen signature is a *breaking*
   change — it rides a major version bump with a migration note
   ([`include/shulib/version.hpp`](../include/shulib/version.hpp)) and an update to the
   signature pin ([`test/f6_signature_pin_test.cpp`](../test/f6_signature_pin_test.cpp));
   the chapters follow the change, never lead it.
3. [Chapter 14](guide/14-what-it-cannot-do-yet.md) when a limitation falls (that's the *good*
   kind of doc rot — celebrate, then delete the paragraph and its glossary entries if any).
4. ✅ Executed 2026-08-12 (D3): the recipe spellings froze as register row **F10**, so
   [chapter 9](guide/09-the-recipe-api.md)'s stability notice and [chapter 14](guide/14-what-it-cannot-do-yet.md)'s "recipe spellings"
   bullet were rewritten the way D2's freeze retired the API notices. Both now say *frozen*, and
   both name the one deliberate exception — `then()`, the mechanism seam, which stays unfrozen
   until F1/F3 build the mechanisms it exists for. A change to a frozen `Routine` spelling is a
   *breaking* change and rides the same procedure as F6's, updating
   [`test/routine_signature_pin_test.cpp`](../test/routine_signature_pin_test.cpp).
5. ✅ Executed 2026-08-13 (F1): the mechanism layer landed, so [chapter 9](guide/09-the-recipe-api.md)'s `then()` section
   was rewritten around the compiled operation idiom (`guide-09d` — the example set is now
   09a/b/c/d), [chapter 13](guide/13-extending-the-library.md) gained "Extension 3: building a
   mechanism", and [chapter 14](guide/14-what-it-cannot-do-yet.md)'s placeholder section fell (celebrated, then replaced with the
   host-only status). The old `chassis.moveTo(p).then(intake.in)` spelling was corrected
   everywhere it was quoted — it was never valid C++ (verbs return `ExitReason`; a member
   function is passed as a lambda) — and [chapter 14](guide/14-what-it-cannot-do-yet.md)'s "no cookbook yet" line, stale since D3
   shipped one, was fixed. `then()` and the mechanism seam remain unfrozen (register row F11)
   until F3.
6. ✅ Executed 2026-08-13 (F2): the run-scoped deadline + guaranteed end-of-run action landed
   (`sequence/run_guard.hpp`), so [chapter 6](guide/06-how-things-fail.md) gained "Running out
   of match" (the failure class no per-step bound can see), [chapter 9](guide/09-the-recipe-api.md) gained the match-clock
   section (the frozen waits' exact lateness bound and the `guard.waitFor` idiom, compiled as
   `sequence-02`), [chapter 12](guide/12-when-things-go-wrong.md) gained the `SEQ`-transcript
   triage section, and [chapter 14](guide/14-what-it-cannot-do-yet.md) gained the guarantee's boundary — quoted VERBATIM from the
   project record, because that claim is the one most likely to be over-quoted and the accurate
   version is narrower than the word "guaranteed". The compiled examples live in
   `test/sequence_example_test.cpp` (sequence-01/02/03). The sequence layer is NOT frozen
   (register row F12) — [chapter 14](guide/14-what-it-cannot-do-yet.md)'s freeze paragraph still lists only F6/F10.
7. The [generated reference](api/README.md) needs no step at all — it regenerates from the
   headers and the build fails if the committed copy is stale. If you changed a `///` comment,
   run `python3 tools/api_doc_tool.py generate` and commit what it writes.

## Voice

Written the way a patient senior member explains things sitting next to someone. Plain
sentences; no marketing. Define every term at first use (and mirror it in the glossary). Prefer
a concrete number to an abstraction, and a reason to a rule. A limit is a plain fact, said where
it applies or in [What it cannot do yet](guide/14-what-it-cannot-do-yet.md) — never a status
disclaimer at the top of a page, and a result from the host test plant says so where it is shown.
If a sentence sounds like a tagline, delete it.
