# DOCS3-A — the BEGINNER TRACK, "Start here: coding your robot with shulib": six walkthrough pages, each built around ONE complete example program that a first-year student can read top to bottom, every line commented in plain words (what it does and why), every example COMPILED AND RUN by the host suite on the host test plant with its result asserted, every quoted line held to the compiled source by the documentation gate — and the site's front door pointing a beginner to it first

## ANSWERS to the navigation and readability amendment (the team lead, 2026-11-15 ~20:30)

- **"Go, accept all (Recommended)"** — D13–D22 as the amendment's re-check recommends (D13 the ten-tab task menu, no file renamed; D14 one shape per kind of page; D15 the readability rules and the tone rule global from N1; D16 brain-screen pictures from the code; D17 the run order N1 first; D18–D22 as recommended, D20 Bench Tests = page 23). **DOCS3-N1 GO** (each later run gets its go). Context the same evening: 'we need to redo all the docs so that everything is more readable'; extra usage bought ('we can do everything').

## Amendment 2026-11-13 — navigation and readability brought forward

> **Status: DRAFT for the team lead — WAITS for his answers to D13–D22 (§A.11), then a go per run.** Written
> 2026-11-15 by the coordinator's brief writer from a READ-ONLY pass. **"main" below = branch `shulib-v2` at
> `9befdb3`** (the main worktree; git's own `main` is the release branch, at `c778c11`). Every `file:line` was
> opened at `9befdb3` (`git show 9befdb3:<path>`, or the snapshot `scratch/brief-DOCS3A/amend/snap/`, a
> `git archive` of it) unless marked *(staged)* — MIRROR1-A's squash, staged in the main worktree's index while this
> was written (65 files: `docs/cookbook/09-one-route-two-robots.md`, guides 02/10/11/14–17/20, `mkdocs.yml` +1
> line). **Probed, not argued:** 13 probes in `.claude/coordinator/scratch/brief-DOCS3A/amend/` (§A.10), every binary
> and script under `(ulimit -v 4194304; timeout 300 …)`, the C++ one with the suite's exact flags. Body sections this
> amendment changes are marked **(amended 2026-11-13)**. Say "host test plant", never "simulator".
>
> **RE-CHECKED 2026-11-15 late (a checker's 15 findings, every one re-verified by this writer from the trees and
> worktrees, READ-ONLY; probes in `scratch/brief-DOCS3A/amend/recheck2/` and `recheck3/`).** All 15 real (two of them
> partly — an overstated word, a miscount); fixed in place, each change marked **[K1]–[K15]** (K15's six small items
> as [K15a]–[K15f]); plus four this writer found (**[S4]–[S7]**). The list with the evidence, the decisions as now recommended and the runs with honest dates:
> "## Re-check — the navigation and readability amendment" at the END. Main is still `9befdb3`; the in-flight
> worktrees were read at ~19:30–20:00 (ROB1-C2, ROB1-BT, AF1S-A2, RMP1-A1, RPV1-S1).

### A.0 The team lead's words, verbatim (newest first)

- **2026-11-15 ~20:05** (the request this run answers): *"we can do everything dont worry im going to buy usage and
  everything we will get it all done"* — up to FIVE jobs; READ1 back before the 21st; docs phase 1 starts before
  the 21st in its own slot and releases as soon as it lands (NOTES 11-15 ~20:05).
- **2026-11-15** (the team lead and the builder, after opening the code): *"we need to redo all the docs so that
  everything is more readable"*; *"there are soooooo many comments in every file it makes things so hard to read like
  the mechanism table"*; *"even the main.cpp is terrible with the amount of comments"*; and for a starter route
  *"minimum comments just as simple as possible for people getting introduced to the library"*. (The code's comment
  cleanup is chunk READ1, separate — §A.9.)
- **2026-11-13, on guide 18:** *"guide 18 is a wall of text we will need to fix that"*.
- **2026-11-13, after seeing the first pictures live:** *"definitely better with the visuals but the entire docs is
  just filled with things that can be explained simpler and with more visuals rather than GIANT paragraph walls of
  text. so eventually we need to fix it all"*.
- **2026-11-13, on the guide:** *"holy this user guide needs a lot of work like visuals. it is just complete walls of
  text its so so bad"*; for the field: *"i would really just find a picture of the actual field"* (done since:
  guide 2's real field, `d7b290a`).
- **2026-11-13, on the front page's status paragraph:** *"remove all this 'honest part' bs that is so unnecessary you
  know"* (the front page's paragraph went in `055ab0b`; ~10 guide pages still carry the same kind).
- **2026-11-13:** *"i really want the docs to be easier to navigate through and understand"* → offered DOCS3-B's
  navigation and readability brought forward and joined to the beginner track, he chose **"Next slot, ahead of the
  rest (Recommended)"**.
- **2026-11-13:** *"really take your time on the documentation it is incredibly important"*; *"people need fully
  updated docs"*.

### A.1 What is wrong today — each claim re-checked at `9befdb3`

Measured by probes A1 and R1 over the 33 READER pages (the front page, the FAQ, the facts form, guides 01–21 and their
README, the cookbook): **11,316 lines, 64,585 words of prose, 2 pictures** (both guide 2's).

| # | Claim | At `9befdb3` |
|---|---|---|
| W1 | "No front door — `docs/README.md` opens on a coder's page" | **Partly out of date.** The front page was rewritten 11-13 (`52313b7`, `055ab0b`): it opens with "What do you want to do?" (`docs/README.md` :9-63). What is still wrong: six asks, each a sentence of links, not the eight a reader brings (no "set up a new robot", "check the robot on the bench", "teach or mentor", "maintain the library"); a "Reference material" list (:95-105) no reader of this page needs; and `docs/guide/README.md` :3-5 still says "**Start at Chapter 1** and read in order". |
| W2 | 20+ chapters in one flat list, in build order | **True.** `mkdocs.yml` :119-141: 22 entries titled "1 — …" to "21 — …"; the cookbook :143-151 a second flat list; FAQ, changelog and "Project" after the generated API block (:394-404). |
| W3 | Internal history in the reader's way | **True.** On the 33 reader pages: **99 chunk ids, 163 HA numbers, 66 ISO dates** in the body (probe A1). Worst: guide 18 :9-25 (a 17-line opening box: "MECH-A1 and A2, 2026-10-15; A3, 2026-10-22; A4 and A5, 2026-10-29 … since the fifth part … MECH-A6, built 2026-10-30"); `guide/README.md` :55-62 ("at chunk COMP-t … COMP-b … MECH-A3 … CAL1 … ROB1 … chunk D1"); guide 10 :693-703; guide 14 (25 chunk ids, 45 HA numbers, 25 dates); `faq.md` :41 ("since T4 Part 2"), :179 ("SAFE-SAVE's Part B2"). |
| W4 | Status-disclaimer preambles and "honest" framing | **True.** Preambles: guide 01 :72-80 ("## The honest part"), 12 :20-22 ("A scoping note, honestly: …"), 16 :12-15 ("Nothing in this chapter has run on a robot yet…"), 18 :9-25, cookbook 04 :3-5 ("…where that honesty changes your code"), 08 :36 ("One honest note before we start…"). **[K6]** Also in a page's opening (before its first H2), found by probe T2 (`recheck3/t2_preamble.py`): guide 17 :15, guide 18 :30, guide 21 :13, cookbook 07 :6; and deeper: guide 07 :186. The words, counting **honesty** too (`\bhonest(y\|ly)?\b`, outside fences and inline code): **116 on public pages under `docs/` (71 on reader pages) + 8 in `README.md`**, and two more inside quoted code comments (guide 08 :67, guide 09 :197 — from `test/guide_examples_test.cpp` :112, :481); guide 04 :78 is a HEADING ("## Honesty about limits — …") with an inbound anchor link from guide 16 :232. Generated API pages: **79 (94 with "honesty") in 49 pages**, copied from header comments under `include/`. No program prints the word (no string literal under `src/`/`include/` holds it). |
| W5 | Long "what this is NOT" parts | **True** (e.g. guide 16 :17 "## What the tester is, and what it is not"; guide 17 :347, :394; guide 21 :283). |
| W6 | Inconsistent page shapes | **True.** 19 guides open with a "> **Covers** / Read this if / Assumes" box; guide 18 and 20 open with a "Read chapter 10 first" box; the cookbook opens with prose; no two pages put their limits under the same heading. |
| W7 | Cookbook and guides overlap without saying which to read | **True.** `cookbook/README.md` :11-14 sends a first-timer to chapters 8 and 9; guide 18 and cookbook 07 both teach `r.move(…)`; guide 18 :699 and cookbook 06 both teach the driver's buttons. |
| W8 | Walls of text | **True, measured** (probe R1, the rules of §A.4): **every one of the 33 reader pages fails at least one rule — 1,375 findings.** 287 paragraphs over 80 words. Guide 18 alone: 50 paragraphs over 80 words (longest 422), a 1,061-word run of prose with no picture, list, table or code, sentences of median 35 words (longest 274), no picture. 529 "Chapter N" references name pages by a number the reader cannot see in a task-organised menu. |

### A.2 The front page — "What do you want to do?" (run DOCS3-N1)

Plain words for the builder; every ask a link; one picture; nothing a reader of this page does not need. The exact
draft (the run may tighten wording, not add paragraphs):

```text
# shulib

shulib is the code that runs Seton Hall's VEX U robots. It knows where the robot is, drives it there,
runs its lifts, claws and flippers, and tells you what happened.

## What do you want to do?

| I want to…                        | Go to                                                                   |
| Learn to code our robot            | Start here — six short pages, one example each        (row added by A1) |
| Drive the robot                    | Build and upload the programs · Buttons for your mechanisms · Match day |
| Write a skills route               | The route builder · Your first route, step by step · The field ·       |
|                                    |   The steps of a route · Try it before the field     [K2: RMP1-A1's two |
|                                    |   pages lead — placed by whichever of N1 and RMP1-A1 merges second]     |
| Add or tune a mechanism            | Declare a mechanism · Tune it on the robot · Protect a string lift      |
| Set up a new robot                 | Add your robot · The robot facts form · Check every motor and sensor ·  |
|                                    |   Calibrate the drive                          [K9: guide 21's order]  |
| Check the robot on the bench       | Bench Tests (P1c-2; until then guide 21 "The order on the robot") ·     |
|                                    |   Try a route on the bench                                              |
| Understand an error or a log line  | Reading the log · When things go wrong · FAQ                            |
| Teach or mentor                    | How it works (the ideas in order, before any code) · Glossary           |
| Maintain the library               | Extend the library · How these pages stay true · API reference ·        |
|                                    |   Changelog · Roadmap                                                   |

[picture: a new robot's first week, drawn FROM guide 21's "The order on the robot" — the Devices screen →
 Bench Tests and Calibrate → measure at station 13 → Drive, the Auton Tester, Competition, shulib Tune]   [K9]

## Why this library exists        [K15c] the section as it stands at `9befdb3` :66-77: its FIRST paragraph is the
                                   approved D11 wording, verbatim; the second (the three questions) becomes a
                                   three-row list, its words kept
## How these pages stay true      [K15c] today's three bullets (:84-90, all true at N1), each one line, + a link to
                                   the new "How these pages stay true" page (below); P1a adds "brain-screen pictures
                                   are drawn by the screens' own code" AT ITS MERGE, never before
```

Each link is to the page's title in the new menu (§A.3); deep links use anchors that exist today (probed: zero broken
anchors at `9befdb3`, §A.10 P-M2). The "Reference material" list goes (the Project tab holds those pages). **(amended
2026-11-13)** D11's sentence already landed (`055ab0b`); A1 no longer edits `docs/README.md` except its one row.

**[K9] The first-week picture's source.** Its figure script reads the bold step names of guide 21's numbered list
under "## The order on the robot" (`9befdb3` :175-191; the same four steps in ROB1-C2's rewrite, *(worktree
`rob1-c2`)* :183-198) and the manifest records the digest of THAT list, not of the whole file — so the picture and the
page cannot disagree (landmine 5), and only a change to the order forces a redraw. The Robot → Set up menu uses the
same order.

**[K15e] The guide README's Maintenance section** (`9befdb3` :49-171: how the guide is organised, the anti-rot rule,
"link, don't restate", the API-change check order, voice) is for maintainers and has no place on a "How it works"
landing. N1 moves it, words kept, to a new public page **`docs/how-the-docs-stay-true.md`** ("How these pages stay
true", Project tab; the front page's section links it). Nothing links into its subsections (`git grep` for
`README.md#` + any of its anchors at `9befdb3`: none). Two COMMENTS point at it and N1 updates both in the same change:
`tools/api_doc_tool.py` :1945-1946 and `test/guide_examples_test.cpp` :5 (a test comment — not robot code).

### A.3 The site organised by reader and task — the exact menu, and nothing moves

**Decision (D13): no existing file moves or is renamed. The menu and the page titles drop chapter numbers; the
FILE names keep them as stable ids.** Why, measured:
- **Six tests open guide files by path** (§A.6 table); ~900 links point into the guide and cookbook (143 with an
  anchor); the published site's URLs (`/guide/18-mechanisms/`) are already in people's bookmarks. **These carry the
  decision on their own.**
- **[K15b] And one robot-code line:** `include/shulib/mechanism/service.hpp` :144 names
  `docs/guide/11-reading-the-diagnostics.md` (the generated `docs/api/service.md` :1728 repeats it) — the ONLY numbered
  doc file the robot code names (`git grep 'docs/(guide|cookbook)/[0-9]' 9befdb3 -- include src`: one hit). The other
  paths the code names (`docs/changelog.md`, `docs/robot-facts-form.md`, `docs/hardware-assumptions.md`,
  `docs/cookbook/`) carry no number, so dropping numbers would not touch them. A docs run never edits `src/` or
  `include/` (and both freeze ~Nov 18).
- **Numbers cost the reader nothing once they are out of sight**: the menu shows titles; a URL is not read.

So **no redirects are needed**, and none are added (mkdocs-redirects was installed in the probe venv to cost the
alternative; it is NOT recommended — renaming would break the tests, links and bookmarks above). **[K2] New pages get
the next number free in EVERY tree** — main, the staged index and each in-flight worktree (landmine 15): **22 is
taken** by RMP1-A1's `guide/22-making-your-first-route.md` *(worktree `rmp1-a1`, untracked)*, so Bench Tests is
**`guide/23-bench-tests.md`** (P1c-2; no brief or worktree names 23 — grep of `briefs/`, `drafts/` and the five
worktrees, 2026-11-15); a run re-checks the number at its start. **[K12] A split keeps the chapter's number and adds
words, never a letter**: `18-mechanisms-on-buttons.md`, `18-mechanisms-in-autons.md`, `18-mechanisms-tuning.md`,
`18-mechanisms-current-caps.md`, `18-mechanisms-rules.md` — the test-case markers `guide-18a`…`guide-18h` already use
those letters for LISTINGS (`guide-18d` is the driver controls, `guide-18g` the current cap, `guide-18h` live tuning —
`test/mechanism_*_example_test.cpp`), so lettered files would say "18d" for two different things. `-` sorts before `.`,
so the five sort beside `18-mechanisms.md`, and every existing glob (`docs/guide/*.md` in `tools/api_doc_tool.py`
:1916, `tools/doc_staleness_audit.py` :62, :68, `test/CMakeLists.txt` :219-221) already covers them — no tool edit.
The markers keep their names (they name listings, and no tool reads the letter); P1a adds to each example test's
mapping comment which file each marker's listing now lives in. The beginner track's new folder `docs/start/` still
needs §1.6 rule 1's three glob edits (unchanged).

**The menu** (probe P-M3 built it under `--strict` — with stub pages for the track and the split, without the
not-yet-written page 23, two labels worded slightly differently: exit 0; the only lines printed were INFO for three API pages
that MIRROR1-A *(staged)* adds and whose generated block the probe did not regenerate — and the nav check below
caught exactly those three). **[K2] Re-built over the FUTURE tree** (re-check probe T7, `recheck2/nav/`: main +
MIRROR1-A's staged pages + AF1S-A2's `cookbook/08-skills-toggles.md` and `api/toggle_camera.md` + RMP1-A1's
`route-builder.md` and `guide/22-making-your-first-route.md` + stubs for the track, the split and page 23, the API block
regenerated): **check-nav 275/275, none missing, none twice; `mkdocs build --strict` with `anchors: warn` exit 0** —
with the route builder's two app files STUBBED (`route-builder/app.html`, `shulib-route-builder.html`, 52 bytes each:
RMP1-A1's `build.py` makes the real ones; without them the build fails — [K3], §A.8). **Ten tabs**, each a reader's
task: RMP1-A1 makes the route builder a top-level tab, approved with its brief ("Go, accept all", NOTES 11-14), and
this menu keeps it. **Probed in a browser** (Playwright's own Firefox from RMP1-A1's capped setup, under
`ulimit -v 4194304`; `recheck2/shots.mjs`, `shots/`): at 1280 px the ten tabs end at 861 px of a 1216 px bar; at 1440
px likewise; at 400 px the tab bar hides and the drawer lists the ten in order. The API block between the generator's
markers (`mkdocs.yml` :153, :392; `api_doc_tool.py` :1474-1476) is untouched; `navigation.indexes` (:59) makes a bare
first entry a section's landing page:

```yaml
nav:
  - Home: index.md
  - Start here:                                   # A1 adds these 7 lines; the tab does not exist before A1
      - start/README.md
      - Set up and build: start/01-set-up-and-build.md
      - Your robot file: start/02-your-robot-file.md
      - Driving and turning: start/03-driving-and-turning.md
      - Write a skills route: start/04-writing-a-skills-route.md
      - Mechanisms in autons and on buttons: start/05-mechanisms.md
      - Test before the field: start/06-testing-before-the-field.md
  - Route builder:                                # [K2] RMP1-A1's tab (its own nav line), with its guide page
      - route-builder.md
      - Your first route, step by step: guide/22-making-your-first-route.md
  - Robot:
      - Set up:                                   # [K9] guide 21's "The order on the robot"
          - Build and upload the programs: guide/07-getting-set-up.md
          - Add your robot: guide/21-adding-your-robot-by-hand.md
          - The robot facts form: robot-facts-form.md
          - Check every motor and sensor: guide/23-bench-tests.md      # new page, P1c-2 [K2: 23, not 22]
          - Calibrate the drive: guide/19-calibrating-your-robot.md
      - Drive and compete:
          - Buttons for your mechanisms: cookbook/06-driver-control.md
          - Match day — the competition program: guide/17-the-competition-program.md
  - Routes:
      - The field and coordinates: guide/02-the-field-and-coordinates.md
      - The steps of a route: guide/09-the-recipe-api.md
      - Driving, turning and curves: guide/05-getting-there.md
      - Where the robot is, and wall resets: guide/03-knowing-where-you-are.md
      - The camera and the Toggle: guide/20-the-camera.md
      - Try a route before the field: guide/16-testing-an-auton-on-the-bench.md
      - Recipes:
          - cookbook/README.md
          - Getting there: cookbook/01-getting-there.md
          - When a step fails: cookbook/02-when-a-step-fails.md
          - Timing and partners: cookbook/03-timing-and-partners.md
          - Routes for a tank: cookbook/04-drivetrains.md
          - Mixing the two styles: cookbook/05-mixing-tiers.md
          - Skills routes and the Toggles: cookbook/08-skills-toggles.md # [K2] AF1S-A2 (worktree, untracked)
          - One route, two robots: cookbook/09-one-route-two-robots.md   # (staged) MIRROR1-A
  - Mechanisms:
      - Declare a mechanism: guide/18-mechanisms.md
      - Put it on buttons: guide/18-mechanisms-on-buttons.md         # P1a's split [K12: no letters]
      - Use it in an auton: guide/18-mechanisms-in-autons.md         # P1a
      - Tune it on the robot: guide/18-mechanisms-tuning.md          # P1a
      - Protect a string lift: guide/18-mechanisms-current-caps.md   # P1a
      - Rules and refusals: guide/18-mechanisms-rules.md             # P1a (a REFERENCE page, §A.4)
      - Recipes: cookbook/07-mechanisms.md
  - Troubleshoot:
      - Reading the log: guide/11-reading-the-diagnostics.md
      - When things go wrong: guide/12-when-things-go-wrong.md
      - What it cannot do yet: guide/14-what-it-cannot-do-yet.md
      - FAQ: faq.md
  - Learn:
      - guide/README.md                            # becomes "How it works": the concept pages in order, and
      - What is this?: guide/01-what-is-this.md    #   every page by its old chapter number, for mentors
      - Drivetrains: guide/04-drivetrains.md
      - How things fail: guide/06-how-things-fail.md
      - A routine built by hand: guide/08-your-first-routine.md
      - The API in words: guide/10-the-api.md
      - Glossary: guide/15-glossary.md
# BEGIN GENERATED API NAV … (unchanged: the "API reference" tab) … # END GENERATED API NAV
  - Project:
      - Extend the library: guide/13-extending-the-library.md
      - How these pages stay true: how-the-docs-stay-true.md     # [K15e] new: the guide README's Maintenance part
      - Changelog: changelog.md
      - Roadmap: roadmap.md
      - Architecture and decisions: shulib-v2-master-plan.md
      - Hardware assumptions: hardware-assumptions.md
      - Diagnostics plan: diagnostics-plan.md
      - VexBuilder integration: vexbuilder-integration.md
      - Legacy command vocabulary: legacy-command-vocabulary.md
```

**[K2] Every public page is placed exactly once — and the menu is built from MAIN AT N1'S MERGE, not from this
list.** At `9befdb3`: 10 root pages, guide README + 01–21, cookbook README + 01–07, the API block; *(staged)* cookbook
09. Pages other runs add land in their own order: AF1S-A2's cookbook 08 (~Nov 17), RMP1-A1's tab and guide 22 (~Nov
19–20), P1a's five split pages, P1c-2's page 23, A1's track, N1's `how-the-docs-stay-true.md`. **Whichever of two runs
merges SECOND places the other's pages** (the coordinator's serial merge): if a page-adding run lands before N1, N1's
merge places its page where this list shows it; if after, that run's merge moves its own nav line from the old menu
into N1's. **`check-nav` (in the doc gate from N1 on) fails the merge that leaves any public page out of the menu or
names a file that does not exist** — so a page an earlier merge added cannot be dropped silently. N1's menu lists only
files that exist (a nav entry naming no file fails `--strict`). Before P1a lands, the Mechanisms tab holds guide 18
and cookbook 07 only.

**How each thing named in the task keeps working:**

| Thing | What happens |
|---|---|
| Links between pages | No file moves, so none breaks. **[K4] An anchor belongs to ONE file:** `18-mechanisms.md#x` resolves only if `x` is on guide 18 (re-check probe T8, `recheck3/cross_nostub.out`: a link from guide 12 to `18-mechanisms.md#moved-tune` with the id only on the split page → "does not contain an anchor", exit 1; with the id on guide 18 → exit 0). So the rule is: **(1) a rewrite WITHIN a page** that removes an anchor keeps it as an invisible `<a id="old-anchor"></a>` where the content went (probe P-M3); **(2) a SPLIT** leaves, on the OLD page, each moved section's old id on a VISIBLE one-line pointer — `<a id="the-tune-program--tuning-on-the-brain-screen"></a>Moved to [Tune it on the robot](18-mechanisms-tuning.md).` — grouped in a short "Moved from this page" list at the bottom, so an old link (a bookmark, the changelog, a page not yet re-pointed) lands on a line that says where to go, never on a page that silently lacks the content (`mkdocs.yml` :73-76's "degraded-but-not-obviously-broken" failure); **(3) the reader pages that link a moved section are RE-POINTED** to the new page in the split's own change — for guide 18 at `9befdb3`: **31 of the 34 anchored inbound links** (the Tune section 8, the auton section 6, the buttons section 5, current caps 5, live tuning 4, route markers 2, the bench page 1; the other three — tuning's starting values, jam signatures, homing — stay on "Declare a mechanism"): **25 on reader pages** — the front page 2, guides 07 (1), 09 (4), 10 (3), 11 (8), 14 (1), 16 (1), 21 (3), cookbook 06 (2) — a link each; the **6 in the LEDGERS** (changelog 3, roadmap 3) keep their links and land on the pointer line; and guide 18's own 11 in-page links to those sections become links to the new pages. **P1a's check lists every inbound anchored link to guide 18 and its new target** (a table in its log, re-run at its merge over main as it then is). |
| A link that does break | `mkdocs.yml` gains `validation: links: anchors: warn` (N1). Probed: **0 broken anchors today**; a planted broken anchor fails `--strict` (§A.10 P-M2). **[S4] Never run the strict build with `-q`:** mkdocs' `--strict` counts WARNING records, and `-q` drops them first — re-check probe T9 (`recheck3/quiet_masks.out`): the same broken anchor builds with **exit 0 under `-q`**, exit 1 without (landmine 14). **[K13] The anchor check already catches a broken link in flight:** ROB1-BT renamed cookbook 06's "A whole driver file" heading, and `cookbook/README.md` :102 still links `#a-whole-driver-file-robot-twos` *(worktree `rob1-bt`)* — exit 1 with `anchors: warn`; today's `mkdocs.yml` (anchors at mkdocs' default, INFO) would publish it silently. Told to the coordinator for ROB1-BT's continuation. Links into `README.md` anchors (GitHub) are not checked by mkdocs: N1's tone sweep fixes the one it breaks (guide 14 :84 → `README.md#how-verified-is-it-honestly`, heading `README.md` :142), and guide 16 :232's link to guide 04's "## Honesty about limits" heading (that one IS checked). |
| `check-examples`' file list | Unchanged (`api_doc_tool.py` :1916 `PUBLIC_DOC_GLOBS`, :1947-1948, :1972): split pages are under `docs/guide/`; listings may move between pages freely (membership is per line). `docs/start/` per §1.6 rule 1. |
| `tools/prepare_site.py` | One addition ([K7], N1): the register's row anchors, added to `site_src/` only. Otherwise unchanged: it copies `docs/` minus `internal/` (:53, :157-176) and renames only the root README (:190-193); `docs/start/README.md` is a section page like `guide/README.md` (probed: the menu with `start/README.md` as a bare first entry builds). New pictures under `docs/assets/` are copied as files. |
| `api_doc_tool.py`'s nav markers | Unchanged: the generated block stays between the markers at the top level; hand edits are outside it; `check-fresh` compares only the block. **[K10]** N1 changes only three wording strings the generator prints on every API page (:1405, :1614, :1641-1642: "user guide", "chapter 10" → "How it works", "The API in words") and regenerates `docs/api/`. |
| `doc_staleness_audit.py`'s folders | Unchanged for split pages (`docs/guide/*.md`, :62, :68); `docs/start/*.md` per §1.6 rule 1. The new readability tool's self-test asserts every reader folder in `PUBLIC_DOC_GLOBS` is also in the audit's `PATH_GLOBS`, so the two lists cannot drift (the P3 hole's cause). |
| Tests that open guide files | §A.6's table: each pinned sentence or line stays verbatim; a pin whose text moves to a split page has its PATH changed in the same change (two pins, both P1a). |
| Chapter numbers | Out of the menu and the titles (N1 renames the menu labels; each page's `# 18 — …` title loses its number when its run rewrites it). **[K11] Of the 529 "Chapter N"/"Ch. N" on reader pages, 224 are plain text, not a link** (re-check count over `snap/`: the glossary 147, the guide README 29, the cookbook README 8, ~40 across the guides) — a reader told "see Chapter 14" would find no 14 in the menu. **N1 turns every UNLINKED one into a link to that page** (the glossary's and the two READMEs' by its own rewrite of those pages; the ~40 others by a one-off script mapping N → file, its diff read in review), so every number a reader meets is clickable from N1 on; the LINKED ones (305) keep their text until the page's own run converts it (rule C). The "How it works" landing also lists every page by its old number, for mentors. The test-case markers (`guide-18c`, `cookbook-06a`) keep their names — they name LISTINGS ([K12]). |
| The changelog's links | It is a ledger: its links stay as written; every anchor they use stays alive — on the same page by an invisible `<a id>`, or, after a split, on the old page's visible "Moved to" line ([K4]); the anchor check holds them. |
| A page in the folder but not in the menu | mkdocs `--strict` only prints INFO for it (measured, and `api_doc_tool.py` :1483-1486 says so). **New `check-nav` (N1):** every public page in the menu exactly once, no menu entry naming no file. Probe N2: `9befdb3` 253/253; the probe tree's 3 missing pages caught; re-check T7: the future tree 275/275. **[K15d] Standard library only**, like every doc tool today (no tool under `tools/` imports `yaml`; the probe used PyYAML, and `mkdocs.yml` :81 carries a `!!python/object/apply` tag a safe loader refuses): it reads the `nav:` block line by line (`- Title: path`, `- path`, `- Title:` + an indented list, comments) and REFUSES any line it does not understand, naming it — a new YAML form fails loudly, never misread. `site_check.py` (N2), which runs inside the docs venv where PyYAML is present, cross-checks that parse against PyYAML's. |

### A.4 One page layout, and the readability rules that hold it

**The layout (D14), for every TASK page** (how to do one job — the Routes, Mechanisms, Robot and most Troubleshoot
pages):

```text
# <the job, in plain words — no number>
<What it is for: ONE paragraph, at most 60 words — who needs it, and when.>
<a picture of the thing's shape or place>
## How to use it          numbered steps, one or two lines each; a picture per step where there is a screen,
                          a spot on the field or a part of the robot
## A complete example     the whole listing (compiled and run, as today), then a short table: line → what it
                          does; then "what you will see" (the output, as its test asserts it)
## What it cannot do      plain facts, one line each; where a number is an estimate, a link to the register
## More detail            links: the rules and refusals, the API reference page, the recipe, the glossary
<details class="history" markdown><summary>History</summary> chunk ids, landing dates, HA numbers </details>
```

**[K5] Page TYPES — one shape per kind of page, declared per page in the tool's `CONVERTED` list** (a page with no
type cannot be converted; the reviewer reads the choice). One fixed set of four H2s cannot fit a page that is a
COLLECTION (cookbook 01 has five recipe H2s, `9befdb3` :11, :49, :100, :139, :173; cookbook 07 two; guide 12 nine
symptom H2s, :32-282) or a catalogue whose headings tests PIN (`wall_reset_line_test.cpp` :286-287 finds
"## The wall reset (WALL)" and the next "\n## "; `toggle_visit_line_test.cpp` :183-185 the same for
"## The Toggle visit (TGL)" — both in guide 11). So rule L depends on the type:

| Type | Shape (rule L) | Pages |
|---|---|---|
| **TASK** | the layout above: H2s in order How to use it / A complete example / What it cannot do / More detail; first paragraph ≤ 60 words | 18 (declare), 18-mechanisms-on-buttons, -in-autons, -tuning, -current-caps; 03, 05, 09; 16, 17, 19, 20, 23; 07, 21 |
| **RECIPES** | first paragraph ≤ 60 words saying what the recipes are for; then each H2 ONE recipe: a line naming (linking) its task page, then a complete listing; a closing "More detail" H2 allowed | cookbook 01–09 |
| **TROUBLESHOOT** | each H2 a symptom in the reader's words, then numbered steps; a closing "More detail" H2 | guide 12 |
| **LEARN** | what it is for / the idea in a picture / how it works / what it cannot do / more detail | 01, 02, 04, 06, 08, 10, 13 |
| **REFERENCE** | no L (its headings are entries — pinned ones stay legal); every other rule applies once converted | 11 (the log catalogue), 14, 18-mechanisms-rules, the glossary, the FAQ |
| **LEDGER** | no L, no H (their subject IS history); T only | changelog, roadmap, register, master plan, diagnostics plan, VexBuilder, legacy vocabulary |
| **WALKTHROUGH** | §1.4's frame (read in order, once); every rule but L | `docs/start/` |

**Which pages, and when:** N1 — none converted (the front page has its own shape, §A.2; the two READMEs become short
landing pages; `how-the-docs-stay-true.md` is moved text) but rule T applies to every public page from N1 (below); P1a —
18, its four TASK splits, 18-mechanisms-rules (REFERENCE), cookbook 07 (RECIPES); P1b-1 — 02, 03, 05; P1b-2 — 09,
cookbook 01–05, 08, 09; P1c-1 — 16, 17, cookbook 06; P1c-2 — 19, 20, the new 23; phase 3 — 07, 21 (TASK), 12
(TROUBLESHOOT), 11, 14, FAQ, glossary (REFERENCE), 01, 04, 06, 08, 10, 13 (LEARN). Guide 02 moved from TASK to LEARN
(it teaches the frame; its how-to is a single step list inside "how it works"). Runs and dates: §A.7.

**The rules (D15) — a new tool, `tools/doc_readability.py`, with `check-nav` and `check-tone` (N1, global) and
`check-readability` (N2, converted pages) and `self-test`; probe R1 prototypes rules H, T, C, P, R, V and A** (`scratch/brief-DOCS3A/amend/r1_readability.py`; its self-test
`r1_selftest.py`: a good page passes and each of the 12 cases fires on its rule — `r1_selftest.out`); S was measured
(probe S-len), L is specified here only. On a CONVERTED page:

| Rule | What fails | Why this number |
|---|---|---|
| **H** history | a chunk id, an HA number or an ISO date in the BODY — outside fenced blocks, the closing History box, and inline code that is MORE than a bare id ([K7]: a span holding ONLY an id or date — `` `MECH-A6` ``, `` `HA-136` `` — fails; a quoted log line or sample comment holding one is the program's words and passes; measured over `snap/`: two spans hold an id or date today, guide 11 :559 `walls L4 R5(ABSENT)` and guide 21 :149 `shulib Tune COPY, 2026-11-17`, neither bare, so the tightening is free); and a History box that is not the page's last block | the team lead's "internal history out of the reader's way". **[K7] The exact pattern, in the tool as `CHUNK_IDS`:** the LONG ids — a name of three or more capitals, or one with a hyphen, then an optional part (`-A6`, `-B`, `-S-A`, `-t`) and an optional lower-case letter: `MECH`, `MECH-S`, `TUNE1`, `STEP1`, `CURVE1`, `AF1`, `AF1S`, `ROB1`, `RPV1`, `RMP1`, `VIS1`, `CAL1`, `COMP`, `IMU1`, `SER1`, `DIAG1` (so `DIAG1b`), `DEFECTS1–2`, `DOCS1–3`, `GATE1`, `SRC1`, `TANK1`, `MIRROR1`, `READ1`, `ALLY1`, `SAFE-SAVE` — plus `chunk <X>`, `phase R<n>`, `<id> Part <n>`, `HA-<n>`, ISO dates. **The SHORT ids are NOT machine-matched**, each listed in the tool's `SHORT_IDS` with the word it collides with: `R1`/`R2` are controller buttons (cookbook 06 :65, :160), `A2`, `A4`, `C1`–`C8` the facts form's field ids (guide 21 :32, :181), `L4` a wall reading in a log line (guide 11 :559), and `D1`–`D3`, `E1`–`E4`, `F1`–`F2`, `G1`–`G3a`, `H1`, `L6`/`L9`, `R1a`–`R5a`, `T2`–`T4` likewise too short to tell from a reader's word; the logged review grep reads them (guide 14 :201 "until R4", :212 "at D3", :537 "at F1"; guide README :140 "F1/F3"). **Its self-test** lists every file-name prefix under `docs/internal/chunks/` (59 at `9befdb3`, one of them `SESSION`, a session log, listed as not a chunk) and fails on any that `CHUNK_IDS` does not match and `SHORT_IDS` does not list — a new chunk name cannot slip past (it runs on the working branch, where `docs/internal/` exists). The probe prototype matched some short ids and missed suffix forms (`DIAG1b`); the run writes THIS pattern, not the prototype's. **Estimates link the register ROW:** `[an estimate](../hardware-assumptions.md#ha-136)` — the number only in the URL, which rule H (case-sensitive `HA-`) does not read; the anchors are below. |
| **T** tone | **[K6] (a)** the word, `\bhonest(y\|ly)?\b` (case-insensitive — "honesty" too: 6 on reader pages + 1 in `README.md`), ANYWHERE on the page, fenced listings included; **(b)** a status-disclaimer sentence in a page's OPENING (before its first H2): `ha(s\|ve) (not\|never) (yet )?(run\|driven\|been (run\|tested\|driven))`, `never driven`, `nothing (in this chapter \|here )?has run`, `not (yet )?(been )?run on a robot` | the 11-13 ruling ("remove all this 'honest part' bs"): no status preambles, limits as plain facts in the page's own "What it cannot do" or guide 14. **Global from N1** (every public page, converted or not), except `docs/api/` (generated from `include/` comments, frozen — a header-comment pass after the 21st removes the 79 (94 with "honesty") there, then the exception goes). (a) covers fences because only two fenced lines hold the word, both COMMENTS in `test/guide_examples_test.cpp` (:112, :481, quoted at guide 08 :67, guide 09 :197) — N1 rewords both comments and re-quotes both lines in the same change; no program prints the word. (b) fires on 5 pages at `9befdb3` (re-check probe T2, `recheck3/t2_preamble.out`: guides 16 :12, 17 :15, 18 :30, 21 :13, cookbook 07 :6); the same pattern anywhere else on a page (34 hits today, e.g. guide 07 :186, guide 01 :74 under "## The honest part") is a LOGGED review grep — each hit must sit in a "What it cannot do" part, guide 14, `README.md`'s "How verified" part, or say where a result is shown that it is the host test plant's. |
| **C** numbers | "Chapter N" / "Ch. N" | pages are named by title in a menu with no numbers |
| **P** paragraph | a paragraph over **80 words** (≈ 5 lines on the site) | 287 of today's paragraphs are over it; guide 18's longest is 422 |
| **R** run | over **250 words** of prose between two pictures, lists, tables or code blocks — **headings do NOT end a run** | so short sections of pure prose still count as a wall (the self-test's case "R run across headings") |
| **S** sentence | a sentence over **40 words** | guide 18: median 35, longest 274; guide 9: median 24 (probe S-len, `s_sentences.out`) |
| **V** picture | no picture on the page, unless it says why: `<!-- no-picture: <reason> -->` (the reason is read in review — the staleness audit's in-document opt-out style, `doc_staleness_audit.py` :99-105) | "VISUALS ARE REQUIRED" |
| **A** alt text | a picture whose alt text is under 3 words | every image with alt text |
| **L** layout | **[K5]** a page whose headings do not follow its TYPE's shape (the type table above): a TASK page whose H2s are not, in order, "How to use it", "A complete example", "What it cannot do", "More detail"; a RECIPES H2 with no task-page link or no listing; a TROUBLESHOOT H2 with no numbered steps; any TASK/RECIPES/LEARN page whose first paragraph is over 60 words. REFERENCE and LEDGER pages have no L | one shape per kind of page; pinned headings (guide 11's two) stay legal |

**How a page becomes "converted":** `doc_readability.py` holds the list (`CONVERTED`, each page with its TYPE), and
each run adds its pages to it in the same change that rewrites them; at the end of phase 3 the list is replaced by
"every reader page", so a new page is held from its first commit. **Rule T is global from N1 ([K6]; N1 creates the
tool with `check-nav` and `check-tone`; N2 adds `check-readability`).** LEDGER pages are exempt from H — their subject
IS history — and the exemption is in the tool with its reason beside it (the house rule: an exclusion states why).

**[K7] The register's row anchors — added at SITE BUILD, not in the source.** The register has 227 `| HA-` rows and no
per-row anchor (`grep -c` over `snap/`: 227, 0), so an estimate link today lands at the top of a 5,756-line page. N1
adds one pass to `tools/prepare_site.py` (which already rewrites the copied pages): each row's first cell gains
`<a id="ha-NNN"></a>`, in `site_src/` only. Why not in the source: every robot-code run edits register rows (ROB1-C2
+38 lines, ROB1-BT +29 *(worktrees)*) and a 227-line anchor edit would conflict with each. Probed (re-check T6,
`recheck2/reg_ok.out`, `reg_bad.out`): with the anchor in a table cell, a link to `#ha-136` builds under `--strict` +
`anchors: warn` (exit 0); a link to a row that does not exist fails (exit 1). On GitHub (the raw `docs/`) the link
lands on the page top — said in the pass's comment; the site is where readers read. `prepare_site.py`'s self-test
gains a case (a row without its anchor fails).

**[K6] `guide-maintenance.md` :97** (the "R3 — first real robot run" sweep) names where the "never driven a robot under
its own control" sentence lives (the README, the orientation chapter, the setup chapter, the can't-do-yet chapter).
After N1's tone sweep the orientation and setup chapters no longer carry it as a preamble; N1 rewrites that row: the
sentence lives in guide 14, `README.md`'s "How verified" part and each page's "What it cannot do" lines — grep the
same words there. (N2 still owns rules 2–3, below.)

**History moves to:** the changelog first (every landing already has an entry), the register (beliefs), and only
where a maintainer needs a page-local pointer, a closing `<details class="history" markdown>` box of ≤ 5 lines
(probe P-M3: renders on the site; GitHub renders `<details>` natively — the reason `mkdocs.yml` :89-96 already uses
plain `<details>` for the API pages).

**The existing gates that require or use those references — kept true:**
- **No gate REQUIRES an HA number, chunk id or date in a reader page.** Measured: no tool under `tools/` and no test
  asserts one (the HA readers are `briefing_status.py` :147-169 and `doc_staleness_audit.py` :137, both on the
  register only). Test COMMENTS cite pages for limits ("the stated limit (banner, guide 14, HA-235)",
  `test/toggle_visit_edges_test.cpp` :751) — they stay true because the LIMIT stays on the page as a plain fact; the
  number moves to the History box ("register: HA-235") so a maintainer still finds it.
- **The program's own words keep their numbers.** Output quoted in a fenced block or inline code is exempt: the route
  preview prints `HA-182` placeholders (`test/route_preview_test.cpp` :1089) and Bench Tests prints `HA-213: …`
  (`test/bench_imu_timing_test.cpp` :1192) — a page quoting them quotes the program.
- **`check-removability` and `prepare_site.py`'s leak screen** (`api_doc_tool.py` :2003; `prepare_site.py` :196-209)
  still scan the WHOLE page, History box included: a box names chunk ids and dates, never an internal path.
- **The staleness audit** skips a numeric claim that carries a chunk tag (`doc_staleness_audit.py` :93, :257-258);
  stripping chunk ids could expose such claims. Measured: **0 lines** on reader pages where the tag is the only
  shield — nothing new will fire.
- **`guide-maintenance.md` rule 2** (:47-50) says "write it *as of* a date" — it becomes "as of <Month YYYY>" in
  words (rule H refuses ISO dates in the body); rule 3 (:61-64) gains "link the glossary on first use" and the
  layout; and the "one mechanism" rule (:41-45) is about example COVERAGE (`check-examples` / `check-fresh`) — the
  readability tool is not a coverage mechanism, so it is not a third one.
- **Glossary on first use:** N1 gives every glossary term an anchor (`<a id="pose"></a>` — probed, resolves under the
  anchor check), so pages can link a term; that a term is linked at its FIRST use is read in review (a logged grep),
  not machine-checked — "first use" is too fuzzy for a gate that must not cry wolf.

### A.5 The pictures — required, drawn by scripts, checked; brain screens drawn by the screens' own code

- **Every picture comes from a script in the repo** (`tools/docs_figures/`, as guide 2's two do today), never edited
  by hand. **Probe F1:** both of today's figures re-render byte-identical to the committed files.
- **A figure manifest** `tools/docs_figures/figures.json`: each published image → its generator, its inputs (with
  their SHA-256), its output's SHA-256, and a credit where the source is not ours (the field: the game manual, already
  printed in the image — `field_coordinates.py` :97). **`check-figures`** fails on: an image under `docs/assets/`
  missing from the manifest (`banner.png` is declared hand-made, the team's logo, used by `README.md` :1); an input
  or output whose hash changed without the manifest (a figure not regenerated, or edited by hand); an SVG whose
  re-render differs from the file. It needs no Pillow (hashes, and SVG text), so it runs in CI. Wired into the doc
  gate (`test/CMakeLists.txt` :270-317; its DEPENDS gain `docs/assets/*` and `tools/docs_figures/*`), `ci.yml`
  :96-104 and `pages.yml` before the build.
- **Brain-screen pictures (D16) — rendered from the code. Probe S1 PROVED the path for "shulib Tune":** a host
  program drove `mechanism::TuneProgram` (PROS-free, `include/shulib/mechanism/tune_program.hpp` :389 — it draws
  through `hal::IBrainScreen`) over `hal::fake::FakeBrainScreen` to four pages (READY, TUNE, CONFIRM, REPORT); the
  fake's record (`fill/rect/erase/print …`, `fake_brain_screen.hpp` :12-17) was replayed into a 480 × 240 picture
  (PNG and SVG) — the REPORT page shows "to 0.00 in: SUCCEEDED in 0.35 s (limit 1.09 s)", the peak and hold currents,
  AGAIN / STOP / BACK, exactly as the code lays them out. 352 assertions pass; both outputs re-render byte-identical.
  (Lessons kept for the run: capture a page a few ticks after it flips, and after its buttons ARM — the first
  captures showed REPORT before its paint and CONFIRM with GO/BACK still grey.)
- **The probe found a real screen defect — REPORTED, not fixed (landmine 8; robot code):** Tune's first page draws
  its READY button as **"READY - I AM CL"**. The label is "READY - I AM CLEAR" (18 characters,
  `include/shulib/mechanism/tune_panel.hpp` :508) but a button's label is stored in `char label[16]` (:109) and
  copied cut to 15 (:459); the button has room for 32 (`tuneLabelCols`, :471-474). No test caught it (the record:
  `scratch/brief-DOCS3A/amend/screens/screen_1_ready.log` lines 14, 17; the picture `tune_1_ready.png`). It is the
  only Tune label over 15 characters. **For the coordinator, before the Nov 18 freeze:** a one-line fix (a longer
  buffer or a 15-character label) in a robot-code run, with a test that every label fits its buffer — the picture
  tool's first catch, and the reason pictures come from the code.
- **The tool:** `test/docs_screens_test.cpp` — one case per picture, `docs-screens: <name>`, drives the program to
  the page and compares the screen's draw record with the committed `tools/docs_figures/screens/<name>.draws`; a
  changed screen turns it red, saying "the screen changed — run `python3 tools/docs_figures/screens.py --update`"
  (which runs those cases with an output folder set, as `SHULIB_GUIDE_PRINT` works, and re-renders). 
  `tools/docs_figures/screens.py` replays a record into `docs/assets/screens/<name>.svg` (SVG: deterministic text,
  no Pillow); `check-figures` re-renders and compares. Two links, both machine-held: **code → record** (the suite),
  **record → picture** (the doc gate). A robot-code change that alters a screen fails its own build until the
  pictures are regenerated — which is what "regenerated when a screen changes" means.
- **Which screens, and how exact:**

| Screen | Model / painter | Picture |
|---|---|---|
| Tune (READY, TUNE, CONFIRM, REPORT; HOME and after COPY) | `mechanism/tune_panel.hpp` + `tune_program.hpp`, over `IBrainScreen` | exact — **probed (S1)** for READY, TUNE, CONFIRM, REPORT; HOME and COPY are the same program, unprobed |
| Calibrate | `calibration/calibrate_panel.hpp` + `calibrate_screen.hpp`, over `IBrainScreen` | exact (same seam; rig `test/calibrate_program_rig.hpp`) — unprobed |
| The auton selector, Auton Tester, Competition | `competition/auton_menu.hpp` / `src/programs/auton_tester_panel.hpp`, `competition_screen.hpp`; painted by `src/common/screen_paint.cpp` through `pros::c::screen_*` | the host PROS shim records the same words (`test/pros_shim/pros/screen.h` :8-12). **Probe S2: `screen_paint.cpp` compiles on the host against the shim** (suite flags, exit 0); running it is unprobed — stop rule ½ day, then draw from the menu model's view instead |
| Bench Tests | `src/programs/bench_r3a.cpp` paints DIRECTLY (**202** `pros::c::screen_*` calls); only its pure parts are host-built (`bench_r3a.hpp` :127-139 the button grid, :625 the readings page) | **from the pure model only** (the grid, the page texts) — exact once Bench Tests paints through `IBrainScreen`, a robot-code change after the 21st |

  The text cells use the panels' character budgets (`ui/touch_panel.hpp` :47-58), themselves ESTIMATES (HA-136) — so
  a picture is the layout the code computes, in a stand-in mono font; no page claims more.
- **Path and curve pictures come from real runs:** the route preview's renderer (`test/route_preview_render.hpp`
  :1-16: "THE PATH IS THE PLANT'S POSES … two runs of the same tree write the same bytes") draws the field, the path
  the host test plant drove and each step — so a curve on a page is the curve the library drove. Feeding a docs
  example's run into `renderSvg` is unprobed (P1b-1's first step, stop rule ½ day).
- **Shape pictures** (a lift's range, presets and home; the four currents' order; the wall reset; the Toggle's look /
  push / back spots; the first-week flow) are small SVG scripts like `field_heading.py`, from the numbers in the
  compiled examples, never typed twice. **[K9]** The first-week flow reads its steps from guide 21's "The order on the
  robot" list (§A.2), so the two cannot disagree.
- **[K8] Each picture pipeline a run depends on is probed FIRST, with a stop rule, and its time is in the run's
  size:** P1a's screen tool (S1 proved it — ~1 day); P1b-1's path pictures from the route preview's `renderSvg` (½–1
  day, stop rule ½ day: then the curves are drawn from the example's planned points by a figure script, said on the
  page as "the planned path"); P1c-1's Auton Tester and selector screens (S2 compiled the painter only — ½–¾ day, stop
  rule ½ day: then from the menu model's view); P1c-2's Calibrate screens (same seam as Tune, unprobed — ½ day) and
  Bench Tests' from its pure model (¼ day).

### A.6 The tests that pin guide text — what each run must keep

| Test (`test/…`) | Pins | Who touches it |
|---|---|---|
| `mechanism_current_cap_test.cpp` :872-877 | guide 18's sentence "the heaviest **hold** (with a game piece) < the stall signature < the cap − 0.2 A" | P1a: the sentence moves to `18-mechanisms-current-caps.md` [K12]; the test's path changes in the same change |
| `mechanism_live_tuning_test.cpp` :531, :596-601 | guide 18's COPY block as one `text` fence (begin line `…src/robots/mechanism_table.hpp:41`) | P1a: moves to `18-mechanisms-tuning.md` [K12]; path changed with it |
| `mechanism_live_tuning_test.cpp` :562-595; `tune_program_test.cpp` :1052, :1206-1252 | guide 11's MECH/TUNE/TUNEPG lines in backticks | phase 3 (guide 11); kept verbatim |
| `toggle_visit_line_test.cpp` :183-185; `wall_reset_line_test.cpp` :286-287 | guide 11's headings "## The Toggle visit (TGL)" and "## The wall reset (WALL)" (each found, then the next `\n## `) and the 12 + 6 lines under them | phase 3: those two headings and lines stay — **legal under rule L because guide 11 is a REFERENCE page** ([K5]) |
| `route_preview_test.cpp` :2189-2192 | two sentences of guide 16 ("the fake assumes it (… s for an arm or lift, … s for a flywheel)") | P1c-1: kept verbatim (RPV1-S1 and ROB1-BT edit guide 16 first — each keeps them; §7 re-runs the test) |

None of the pinned text holds a chunk id, HA number or date, so rule H never forces a pin to move.

### A.7 The runs, their order, sizes and dates (D17) — five job slots — **re-done at the re-check ([K1], [K3], [K8], [K14])**

**The phases, as the team lead set them, with one change (D17):** the NAVIGATION ships in phase 1's first release,
because the task's point (5) needs it first ("so the beginner track lands into its final home") and the team lead's
11-13 answer put it "ahead of the rest". Phase 1 = the menu, the front page, the tone sweep and the pages people need
to code autons; phase 2 = the beginner track; phase 3 = everything else. **[K6] The tone sweep moved INTO N1** (it
was N2's): it is mechanical and it was the team lead's FIRST ask (11-13), so the first reorganised release must not
still carry ~70 "honest" and five status preambles. **[K3] The site-build machinery moved OUT of N1 into N2**, so N1
does not touch `ci.yml`/`pages.yml`, which RMP1-A1 is rewriting.

**[K8] One rate for every page rewrite:** ~3,000 words of today's prose per builder-day — P1a's own number (7.6k
words in ~2½ days once its screen tool's ~1 day is taken out) — with each run's tooling and picture pipelines sized on
top (§A.5's stop rules). **NOT measured:** P1a records the words it rewrote each day in its log, and the coordinator
re-dates every later run from P1a's first full day. Words: probe A1 at `9befdb3` and over the FUTURE tree
(`recheck2/a1_future.out`: + MIRROR1-A's staged pages, AF1S-A2's cookbook 08, the in-flight edits as of 19:50). At that
rate the old P1b (~13.6k–14.9k words + an unprobed pipeline) was ~5½–6½ days and the old P1c (~14.3k + four screen
sets + a new page) ~6½–7, not 3–4 — **each is split in two** so the halves run in parallel after the 21st.

| Run | What | Owns (a run in flight edits only its own files; see "shared files" below) | Words | Size (builder-days) + review |
|---|---|---|---|---|
| **DOCS3-N1** — what the reader sees first | the menu (§A.3) + `anchors: warn`; the front page (§A.2) and its first-week picture (from guide 21's list, [K9]); `guide/README.md` → the "How it works" landing (with the old-number index), its Maintenance part → `how-the-docs-stay-true.md` ([K15e]); `cookbook/README.md` → "Recipes: which to read"; glossary anchors; the 224 unlinked "Chapter N" → links ([K11]); **the tone sweep** — rule T (a) and (b) on every public page but `docs/api/` (~124 words, 5 openings, guide 01's "The honest part", guide 04's heading + guide 16 :232's link, `README.md`'s heading + guide 14 :84, the two test comments re-quoted, `guide-maintenance.md` :97) ([K6]); `tools/doc_readability.py` with `check-nav` (standard library) + `check-tone` + `self-test`, wired into the doc gate; the register's row anchors in `prepare_site.py` ([K7]); `api_doc_tool.py`'s three "user guide"/"chapter 10" strings + `docs/api/` regenerated ([K10]) | `mkdocs.yml`, `docs/README.md`, the two READMEs, the glossary, `how-the-docs-stay-true.md`, the swept lines, `test/guide_examples_test.cpp` :5, :112, :481 (comments only), `tools/doc_readability.py`, `tools/prepare_site.py`, `api_doc_tool.py` :1405, :1614, :1641-1642, :1945-1946, `docs/api/` (regenerated), `guide-maintenance.md` :97, the doc-gate block | 1.6k (landings) + the sweep | **~2–2½ + ½** |
| **DOCS3-N2** — the checks | `tools/site_check.py`, the ONE definition of the site build, which `pages.yml` and a new CI "site" job both call (§A.8); `tools/docs-requirements.txt`, the full resolved pin set; `check-readability` (rules H C P R S V A L with the page types, §A.4) in `doc_readability.py`, a self-test case per rule that fails when the rule is removed; `guide-maintenance.md` rules 2–3, the briefing's doc map, the changelog | `tools/site_check.py`, `tools/docs-requirements.txt`, `ci.yml`, `pages.yml`, `doc_readability.py`'s readability half, `guide-maintenance.md` rules 2–3 | — | **~2 + ½** |
| **DOCS3-P1a** — guide 18 | **guide 18 first among the page rewrites** (the team lead's order): split by job into `18-mechanisms.md` (declare) + `-on-buttons`, `-in-autons`, `-tuning` (numbered steps, a SCREEN PICTURE at each: READY, HOME, TEST, the REPORT, COPY), `-current-caps` (protect a string lift), `-rules` (REFERENCE) ([K12]); each opening with a picture; cookbook 07 to RECIPES; the screen-picture tool (§A.5) + `figures.json` + `check-figures`; the moved anchors' pointer lines and the 25 inbound links re-pointed ([K4]); the two pins' paths (§A.6). **Base: main after ROB1-C2 AND ROB1-BT merge** (both edit guide 18, [K1]) — or, if a slot frees first, main as it is, and the coordinator ports ROB1-BT's 8 guide-18 hunks onto the split pages at P1a's merge from P1a's "where each old section went" table | `docs/guide/18*.md`, `cookbook/07`, the 25 link lines, `tools/docs_figures/*`, `docs/assets/screens/`, `test/docs_screens_test.cpp`, the two pin tests, the example tests' mapping comments | 7.6k | **~3½ + ~1** |
| **DOCS3-A1** — the beginner track | §1 as amended: its gate rules (§1.6, `api_doc_tool.py`), the practice rig, pages 4 → 3 → 5 → 2 → 1 → 6, page 6's route preview now IN A1 (RPV1-A merged, `9befdb3`); its 7 menu lines + the front page's row | `docs/start/`, `test/start_*`, `api_doc_tool.py`'s gate rules | (new) | **~4–5 + ~1** (D19 (a) saves ~¼) |
| **DOCS3-P1b-1** — the field and driving | guides 02 (LEARN), 03, 05 — path pictures from the route preview's renderer (§A.5, probed first, stop rule ½ day) | those pages, their figure scripts | 7.7k | **~3–3½ + ~1** |
| **DOCS3-P1b-2** — the steps of a route, and recipes | guide 09; cookbook 01–05, 08, 09 (RECIPES) | those pages | 7.2k | **~2½–3 + ~1** |
| **DOCS3-P1c-1** — testing and match day | guides 16, 17, cookbook 06; the Auton Tester's and the selector's screen pictures (§A.5) | those pages, their screen cases | 8.1k | **~3½ + ~1** |
| **DOCS3-P1c-2** — calibrate, camera, bench | guides 19, 20, a NEW `guide/23-bench-tests.md` (D20, [K2]); Calibrate's and Bench Tests' screen pictures; the Toggle's look/push/back spots | those pages, their screen cases | 5.8k + the new page | **~3¼ + ~1** |
| **DOCS3-A2** | the track's late landings (AF1S-A2's everyday Toggle form) | `docs/start/` | — | ~½ |
| **DOCS3-P3a/b/c** | phase 3, after the 21st: Learn (01, 04, 06, 08, 10, 13, the glossary's rewrite — 16.8k, ~5½), Troubleshoot and reference (11, 12, 14, FAQ — 8.0k, ~2½), the rest (07, 21, the facts form, `README.md`, `test/README.md` — 3.4k + the READMEs, ~1½–2; the ledgers get T only, in N1) | by tab | ~28k | **~9½–10 in total + ~3** (was "6–8": [K8]) |

**In total:** ~33–37 builder-days of docs work + ~10 of review. Phase 1 alone (N1, N2, P1a, P1b-1/2, P1c-1/2) is
~20–22 + ~6.

**[K1] The slot arithmetic — every run before the 21st, from NOTES (11-15, :1045–1063) and the worktrees (read
~19:30–20:00).**

*Running now (five slots):* **ROB1-C2** (robot one's mechanisms and typed facts; merges tonight, NOTES :1047–1051,
:1057); **RPV1-S1** — NOT queued, as the first draft said: it was launched and BUILT today (worktree `rpv1-s1`,
`RPV1-PROGRESS.md` :654-765: the build log, 11 mutations (MS1–MS11), the prose review; files edited to 19:23) and its
src-gate self-test was running at ~20:00 → merge ~Nov 16; **RMP1-A1** (cont; lands Thu–Fri, Nov 19–20, NOTES
:1055–1056); **AF1S-A2** (to ~Nov 17, aim: the Wed freeze, :1056); **this re-check** (tonight). MIRROR1-A's agent job
is done — its squash is staged in the main worktree's index for the coordinator's gates and merge.

*Queued before the 21st, all robot code or the tooling the 21st needs, so all ahead of docs* (the 11-12 answer "Go,
after robot code"):
- **the starter route** — after ROB1-C2 merges, "tonight, short" (:1052, :1056, :1057);
- **ROB1-BT** (cont) — after ROB1-C2 merges, ~1–1½ days → ~Nov 17 (:1047, :1057);
- **MIRROR1-B** — small, after MIRROR1-A's merge, BEFORE the Wed freeze (:1059);
- **the Tune label fix** — one line and a test, before the freeze, the coordinator's or a small run (:1063, §A.5);
- **READ1** — its brief (~½ day, still to write), then its build after ROB1-BT (~1–1½) and review (~½) (:1053–1054,
  :1057). It edits `src/robots/*` and `src/main.cpp`, so it is freeze-bound, and a build starting ~Nov 17 merges
  ~Nov 18½–19 — **at or past the Nov 18 freeze: the coordinator's call**, flagged here because §A.9's order depends
  on it;
- **RMP1-B** (Controls tab + Driving feel) — the first free slot after RMP1-A1's BUILD step finishes; lands with A1
  Thu–Fri (:1060);
- RPV1-S1's merge; then docs (:1057).

*The usage limit:* **the weekly limit was hit at ~19:55 ("resets Nov 18, 10pm", NOTES :1062); the team lead is buying
extra usage ("KEEP PUSHING"). Every date below assumes the extra usage stays on. Without it, no agent run moves until
Nov 18 ~22:00 and NOTHING of the docs lands before the 21st.**

*Slot by slot* (one builder-day ≈ one calendar day; no usage stop):

| Slot | Nov 15 night | Nov 16 | Nov 17 | Nov 18 (freeze) | Nov 19 | Nov 20 |
|---|---|---|---|---|---|---|
| 1 | ROB1-C2 → merge | ROB1-BT (cont) | ROB1-BT → merge; READ1 build | READ1 build, review | READ1 merge → **A1** | A1 |
| 2 | RPV1-S1's gates | RPV1-S1 merge; the starter route; MIRROR1-B | MIRROR1-B → merge (+ the label fix if not the coordinator's) → **P1a** (pm) | P1a | P1a | P1a |
| 3 | RMP1-A1 | RMP1-A1 | RMP1-A1 | RMP1-A1 | RMP1-A1 → merge (Thu–Fri) | → **P1b-1** |
| 4 | AF1S-A2 | AF1S-A2 | AF1S-A2 → merge; **RMP1-B** (once A1's build is done) | RMP1-B | RMP1-B | RMP1-B → merge |
| 5 | this re-check | READ1's brief (½) → **N1** (~midday) | N1 | N1 built → review | N1 merged → release; **N2** | N2 |

If RMP1-B is ready before AF1S-A2 frees slot 4, it takes slot 2 on Nov 17 and **P1a starts ~Nov 18** instead.

- **RECOMMENDED (D17 (b), changed from (a)) — the menu first, guide 18 in the very next docs slot:**
  - **N1 ~Nov 16 midday → built ~Nov 18 → merged and released ~Nov 19** (one usage stop: ~Nov 20 — still before the
    21st); one release with RMP1-A1's if they land within a day (D21).
  - **P1a ~Nov 17 pm (or ~Nov 18) → built ~Nov 21 → merged ~Nov 22** (~Nov 23 if it starts Nov 18 or one stop hits it).
  - **N2 ~Nov 19 → merged ~Nov 21½**, on main after RMP1-A1 where the timing allows (§A.8).
  - **A1 ~Nov 19 → merged ~Nov 24–25.** **P1b-1 ~Nov 20 → ~Nov 24; P1b-2 and P1c-1 ~Nov 20–21 → ~Nov 24–25;
    P1c-2 ~Nov 22 → ~Nov 26.** A2 ~Nov 26. Phase 3: three runs from ~Nov 25–27 → ~Dec 1–4; with stops, early to
    mid December.
  - **Releases:** the menu, the front page and the tone sweep ~Nov 19; guide 18 ~Nov 22; the checks ~Nov 21½ (with
    whichever release is next); the beginner track, the route pages and the bench pages ~Nov 24–26; phase 3 in December.
  - **Said plainly: before the 21st, only N1 can land — and only if the extra usage stays on and the robot-code queue
    above clears on time. Guide 18 starts before the 21st, in its own slot, and lands after it (~Nov 22) in EVERY
    order.** Each usage-limit stop adds about a day to every run it hits.
  - Why it changed from (a): since the first draft the pre-21st queue grew by four runs (the starter route, MIRROR1-B,
    the label fix, READ1's brief and build), RPV1-S1 already took a slot, and the weekly limit is hit. Guide 18 cannot
    reach the students before the competition in any order; the menu keeps a one-stop margin only if it goes first.
    The team lead's 11-13 answer was literally "navigation, next slot, ahead of the rest", and "guide 18 first" was
    his order among the page rewrites — (b) keeps both.
- (a) **Guide 18 first** (the first draft's recommendation): P1a ~Nov 16 midday → ~Nov 21 (the competition day — no
  use to it); N1 ~Nov 17 pm → ~Nov 20, and one stop pushes the menu to the 21st.
- (c) **The beginner track's page 4 first** (A1 in the first docs slot, cut line Nov 20 noon, every other track page a
  guide-post): the skills-route walkthrough could reach the students ~Nov 20–21; the menu ~Nov 20; guide 18 ~Nov 23.

**How this joins the beginner track (the task's point 5).** One brief, separate runs, each its own go. The menu and
the front page land BEFORE the track (N1 ~Nov 19; A1 ~Nov 24–25), so the track lands into its final home: N1's
menu has no "Start here" tab (a tab of empty pages would be a placeholder), and A1 adds the tab's 7 lines and the
front page's first row at its merge. The track's pages obey the readability rules (all but L) from their first
commit and LINK the task pages for detail — they never restate a rule a task page owns (landmine 5's reason: two
copies disagree within a week). A1's own gate rules stay in `api_doc_tool.py` (§1.6), N1/N2's in the new
`tools/doc_readability.py`. A1 builds while N2 is in flight; it merges after both.

**Shared files — who edits which page, and the base each docs run starts from ([K1], [K3]).** In-flight runs edit
docs pages (worktrees, ~19:30–20:00): **guide 18** — ROB1-C2 (+4: the one-button `whileHeld`), ROB1-BT (17+/13−, 8
hunks: robot two's claws retired, the twin); **guide 16** — ROB1-BT (113 lines), RPV1-S1 (16); **guide 17** — ROB1-BT
(63), MIRROR1-B (to come); **guide 19** — ROB1-BT (44); **guide 20** — AF1S-A2 (41); **guide 21** — ROB1-C2 (207),
ROB1-BT (88); **guides 10, 11, 14** — AF1S-A2, ROB1-BT, RPV1-S1, RMP1-A1; **the glossary** — AF1S-A2 (+13); **cookbook
README** — AF1S-A2 (+6), ROB1-BT (its broken link, [K13]); **cookbook 06** — ROB1-BT (117); **the facts form** —
ROB1-C2, ROB1-BT; **guide README** — RMP1-A1 (+1); **`mkdocs.yml`** — MIRROR1-A *(staged)* +1, AF1S-A2 +2, RMP1-A1 its
tab and guide 22; **`ci.yml` (+96) and `pages.yml` (+42)** — RMP1-A1. The rules:
- **A docs run starts on main AFTER the robot-code runs that edit its pages have merged**, where the order allows:
  P1a after ROB1-C2 and ROB1-BT; P1c-1/P1c-2 after ROB1-BT, MIRROR1-B and READ1 (they start after the 21st anyway).
  Where a slot frees first, the coordinator PORTS that run's page edits at the docs run's merge, from the docs run's
  "where each old section went" table.
- **N1's small edits on many pages** (the sweep's words, the 224 chapter links, the README anchor) conflict only line
  by line; the coordinator re-runs N1's deterministic chapter-link script on the merged text and re-reads the sweep's
  patterns (`check-tone` fails any word a later merge brings back).
- **`ci.yml`/`pages.yml` are N2's, after RMP1-A1** (§A.8). **`api_doc_tool.py`:** A1's gate rules + N1's three strings
  and one comment — different regions, serial merge, `docs/api/` regenerated at each merge (the tool is
  deterministic). **`prepare_site.py`:** N1 alone. **`doc_readability.py`:** N1 creates it, N2 extends it. **The
  doc-gate block of `test/CMakeLists.txt`:** N1 (check-nav, check-tone), N2 (check-readability), P1a (check-figures,
  the screens test), A1 (`docs/start/` globs) — each adds its lines at its merge. **`docs/README.md`:** N1; A1's row,
  P1a's bullet and RMP1-A1's two links (if it merges after N1) at their merges.
- `mkdocs.yml`'s nav: whichever of two runs merges second places the other's pages (§A.3); `check-nav` fails a merge
  that leaves one out.

**[K14] The progress log — one chunk, one file, one heading per run, created by the plan commit.** The plan commit
creates `docs/internal/chunks/DOCS3-PROGRESS.md` WITH the between-parts marker in its first 40 lines
(`tools/release.py` :48-49, :151-159 — a release FAILS on a PROGRESS file without it) and ONE heading per run, in this
order — N1, P1a, N2, A1, P1b-1, P1b-2, P1c-1, P1c-2, A2, P3a, P3b, P3c — each with a one-line placeholder. **A run
writes ONLY under its own heading.** Probed (re-check T11, `recheck3/mergelog/`): three runs editing their own sections
in three copies merge cleanly (`git merge-file`, exit 0 twice); runs that each append at the END of one file conflict
(exit 1) — which is what "a heading per run" in a file each run creates would have done. The marker stays on main
throughout (main never holds a run's in-progress log; each builds in its own worktree); the coordinator updates its
reason at each merge.

**The run frame (unchanged from §1):** executor model `opus`; do not commit; every binary under
`(ulimit -v 4194304; timeout 300 …)`; `src/` and `include/` untouched (§7's first check). **Hardware Assumptions:
still NONE** — the pictures show what the code draws and computes; the screen budgets they use are already HA-136's.

**Releases (D21):** a docs release per landing (two landings within a day share one), each through `tools/release.py`'s
full check; "docs-only" means the docs run changed nothing under `src/` or `include/` (§7's first line) — the release
still carries whatever else main holds (the students' route edits allowed after the freeze, RMP1 D15).

### A.8 How the site build is checked — one definition, probed

The laptop has no mkdocs (`python3 -c "import mkdocs"` fails, re-checked 11-15). **Probe P-M1:** a venv under the
scratch folder with CI's pins (`pages.yml` :65: `mkdocs==1.6.1 mkdocs-material==9.5.44`) installed in about a minute;
`prepare_site.py <out>/site_src` then `mkdocs build --strict` over a copy of `mkdocs.yml` (its `docs_dir: site_src`,
:23) built today's 253 pages in ~8 s, exit 0. **P-M2:** with `validation: links: anchors: warn` added, still exit 0
(no broken anchor today); one planted bad anchor → "Aborted with 1 warnings in strict mode!", exit 1. **P-M3:** the
§A.3 menu → exit 0. **[S4] Never pass `-q`:** with `-q` the same planted anchor builds with exit 0 (re-check T9,
`recheck3/quiet_masks.out`) — `--strict` counts WARNING records and `-q` drops them first.

**[K3] What RMP1-A1's merge changes.** Its `docs/route-builder.md` links `route-builder/app.html` and
`route-builder/shulib-route-builder.html`, files its `pages.yml` builds BEFORE mkdocs —
```text
cmake -S test -B build/test
cmake --build build/test --target shulib_route_maker -j4
python3 tools/route_maker/build.py --out site_src/route-builder --describe build/test/shulib_route_maker
```
— and checks AFTER it (its PG1 step: `site/route-builder/app.html` and `shulib-route-builder.html` present, `app.html`
carries `<meta name="shulib-app" content="route-builder">`, the docs page links the app, no laptop or runner path).
Without those files the strict build FAILS (re-check T5: RMP1-A1's worktree through its own `prepare_site.py`, then
`mkdocs build --strict` → exit 1, "contains a link 'route-builder/app.html', but the target is not found" ×2;
`recheck2/build_norb.out`). So any site check that copies today's `pages.yml` goes red the day RMP1-A1 merges.

**N2 adds `tools/site_check.py` — the ONE definition of the site build** (one command, every binary under the suite's
bounds, imported from `tools/suite_run.py` as `route_preview.py` does):
1. make or reuse `build/docs-venv` from `tools/docs-requirements.txt`;
2. `prepare_site.py` into `build/site_src`;
3. **when `docs/route-builder.md` exists** (a fact of the tree, not a switch), the route builder into
   `build/site_src/route-builder/` by RMP1-A1's three commands above (reusing the suite's `build/test` when present) —
   a missing `tools/route_maker/build.py` then FAILS, never skips;
4. `mkdocs build --strict -d build/site` — never `-q`;
5. the route builder's PG1 checks, moved from `pages.yml`;
6. `check-nav`, plus a cross-check of its standard-library parse against PyYAML's inside the venv ([K15d]);
7. `pages.yml` :86-92's internal-path grep over the built site.

**`pages.yml`'s build job and a new CI "site" job (`ci.yml`, every push) both run `python3 tools/site_check.py
--required`**, so local = CI = publish by construction — there is no second copy of the steps to drift. (The CI job
may pass `--route-builder <dir>` to use the route-builder job's artifact instead of rebuilding; the publish job builds
its own.) `pages.yml` keeps checkout, setup-python, the toolchain install, the `site_check.py` call, CNAME, upload and
deploy. `site_check.py`'s self-test fails if `pages.yml`'s build job gains a `run:` step that is none of those — so a
later publish step cannot slip past the local check. Today the strict build runs only on pushes to the release branch
(`pages.yml` :13-21), so a broken menu on the working branch is found at release time — the most expensive moment.

**[K3] Pins:** `tools/docs-requirements.txt` holds the FULL resolved set, not two pins. CI today installs the two and
lets ~26 others float (pymdown-extensions, Markdown, Pygments, …): the probe venv resolved pymdown-extensions 10.21.3,
Markdown 3.11.3 and regex 2026.11.15 — released the day of the probe — and also holds mkdocs-redirects, which CI does
not install. The run makes a CLEAN venv from the two pins, `pip freeze`s it, and pins any package released in the
last two weeks to its previous release.

**[K3] Order with RMP1-A1.** RMP1-A1 started first and edits `ci.yml` (+96: the route-builder and
route-builder-browsers jobs) and `pages.yml` (+42: path triggers, the ubuntu-24.04 pins, the route-builder build, PG1)
*(worktree `rmp1-a1`)*. N1 no longer touches either file. **N2 starts on main after RMP1-A1's merge (Thu–Fri) when
the timing allows, and moves RMP1-A1's two `pages.yml` steps into `site_check.py` itself; if N2 starts first it
rebases onto RMP1-A1 before its own merge; if N2 must MERGE first, the coordinator moves the two steps at RMP1-A1's
merge** — the steps are written above. If pip cannot reach the network the tool says **SKIPPED — the site was not
built**, loudly, and exits non-zero under `--required` (never a silent pass). External links (the GitHub links
`prepare_site.py` writes, `README.md` anchors) are not checked — that needs the network and the release branch; said
in the tool's banner.

### A.9 READ1 (the code's comment cleanup) and the docs — how both keep `check-examples` green

READ1 shortens comments in `src/team/*`, `src/robots/*` and `src/main.cpp` and splits the editable parts out. The doc
gate quotes `src/team/*_driver.hpp` and `src/team/autons.cpp` VERBATIM (`api_doc_tool.py` :1947-1948), each listing
as one unbroken run (:1972). **Probe Q1 — the listings READ1 would break:** 7 listings, 57 quoted lines, **41 of them
carry a comment**:

| Page (block) | File | Lines / with a comment |
|---|---|---|
| cookbook 06 (5, 6, 7) | `robot_two_driver.hpp`, `robot_one_driver.hpp`, `robot_two_driver.hpp` | 4/3, 5/2, 1/0 |
| guide 16 (1, 2) | `autons.cpp` | 21/16, 15/10 |
| guide 17 (1) | `autons.cpp` | 3/3 |
| guide 21 (1) | `robot_one_driver.hpp` | 8/7 |

- **READ1 re-quotes those listings in its own change.** Its build fails on the first changed comment, naming the page
  and line ("fix the SOURCE first, then re-quote" — the gate's own words); READ1 never edits code toward a page.
- **Paths:** a file READ1 renames or splits fails the staleness audit's dead-path check on every public page naming
  it in backticks (`doc_staleness_audit.py` :267-277) — today guide 21 (7), guide 16 (4), guide 17 (2), cookbook 06
  (2), guide 08 (2), the register (39), the roadmap (25), the changelog (4). READ1 updates the live pages; a ledger
  line says "moved to …" (the audit's ABSENT_ON_PURPOSE words, :109).
- **[K15f] That count was `src/team/*` only. READ1 also touches `src/robots/*` and `src/main.cpp`,** named in backticks
  on (re-check count over `snap/`) guide 07 (4), guide 08 (3), guide 21 (6), `README.md` (2), and guides 11, 18, 19,
  cookbook 06, the facts form, `vexbuilder-integration.md` (1 each), plus the ledgers (register 37, roadmap 22,
  changelog 6). **And guide 21 :53-56 and :97-99 quote `robot_one_facts.hpp` code in ```` ```text ```` fences that NO
  gate checks** (`check-examples` reads only `cpp` fences; the audit only backticked paths). So **READ1's rule is a
  grep, not the audit:** for every file it renames, splits or re-comments, it greps every public page (and `README.md`)
  for the file's name AND for each quoted line of it in any fence, and updates them in the same change; its log lists
  each hit read. Guide 11's and 18's COPY example names
  `src/robots/mechanism_table.hpp:41` (an example line pinned by `mechanism_live_tuning_test.cpp` :588, :597; the
  `:41` keeps it out of the path check) — if robot declarations leave that file, READ1 changes the pin and both pages
  together.
- **Order:** READ1 merges BEFORE the docs runs that rewrite pages quoting its files (P1c-1: 16, 17, cookbook 06; P1c-2:
  19; phase 3: 07, 08, 21), so those quote the cleaned files. **N1, N2 and A1 quote none of READ1's files** (A1's
  listings come from `test/start_*`); **P1a's guide 18 names `src/robots/mechanism_table.hpp` once** (the COPY
  example's `:41`, above) — if READ1 moves the declarations, READ1 changes that pin and the page together, whichever
  of the two merges second. If a docs run is in flight when READ1 merges, its merge re-quotes what the gate names and
  what READ1's grep listed.
- **The docs never lean on a comment banner:** pages explain in their own words; a listing of a team file starts at
  its first code line and shows no comment block longer than one line — the same taste READ1 applies.

### A.10 The probes (`.claude/coordinator/scratch/brief-DOCS3A/amend/`, 2026-11-15; all under the memory cap)

| Probe | Files | Result |
|---|---|---|
| A1 measure | `a1_measure.py`, `a1_head.out/.json` (over `snap/` = `9befdb3`) | the W3/W4/W8 numbers |
| A2 chunk-id hits | `a2_hits.py`, `a2.out` | short ids collide: R1/R2 buttons, A2/A4/C1 form fields |
| R1 readability | `r1_readability.py`, `r1_head.out`; `r1_selftest.py`, `r1_selftest.out` | 33/33 pages fail, 1,375 findings; 12/12 self-test cases right |
| N2 nav | `n2_check_nav.py`, `n2.out` | `9befdb3`: 253 pages, all once; probe tree: the 3 unlisted pages caught |
| P-M1 site | `venv/`, `site0/build.out` | strict build of today's site, ~8 s, exit 0 |
| P-M2 anchors | `site_anchor/`, `p_anchor.out`, `p_anchor2.out` | 0 broken anchors; a planted one fails strict |
| P-M3 menu | `navtree/`, `p_nav.out` | §A.3's menu + stub pages: exit 0; an `<a id>` glossary anchor resolves; the History box renders |
| S1 Tune screens | `screens/probe_tune_screens.cpp`, `replay_screen.py`, `s1.out`, `tune_*.png/.svg` | 4 pages from the code; 352 assertions; byte-identical re-render; **found READY's label cut to "READY - I AM CL"** |
| S2 painter | `screens/screen_paint_host.o` | `src/common/screen_paint.cpp` compiles on the host shim |
| F1 figures | `figs/` | both of today's figures re-render byte-identical |
| Q1 READ1 | `q1_read1_exposure.py`, `q1.out` | 7 listings, 57 lines, 41 with a comment |
| S-len sentences | `s_sentences.out` | guide 18 median 35 words, p90 77, longest 274 |
| staleness shield | `shield.out` | 0 reader-page lines where a chunk tag hides a numeric claim |
| **Re-check (2026-11-15 late)** | `recheck2/`, `recheck3/` | |
| T1 future words | `recheck2/a1_future.out/.json` (A1's measure over the future tree) | phase sizes at one rate ([K8]) |
| T2 openings | `recheck3/t2_preamble.py`, `t2_preamble.out` | rule T (b): 5 pages with a status sentence before their first H2; 34 hits anywhere ([K6]) |
| T5 route builder | `recheck2/nav/nav/site2`, `build_norb.out` (+ the checker's `checker/rmp/build.out`) | RMP1-A1's tree without the app's files: `--strict` exit 1 ([K3]) |
| T6 register anchors | `recheck2/reg_ok.out`, `reg_bad.out` | `<a id="ha-136">` in a table cell resolves; a missing row fails ([K7]) |
| T7 future menu | `recheck2/navhead.yml`, `apiblock.yml`, `navtail.yml`, `nav/`, `navcheck.out`, `build.out` | 275/275 pages once; `--strict` + `anchors: warn` exit 0 (app files stubbed) ([K2]) |
| T8 cross-file anchors | `recheck3/x/`, `cross_nostub.out`, `cross_stub.out` (and `recheck2/cross*.out`) | an id on the split page does NOT satisfy a link to guide 18 (exit 1); a pointer line on guide 18 does (exit 0) ([K4]) |
| T9 quiet flag | `recheck3/quiet_masks.out` | `-q` hides a strict failure (exit 0 vs 1) ([S4]) |
| T10 tabs in a browser | `recheck2/shots.mjs`, `shots.out`, `shots/*.png` — Playwright's own Firefox, under `ulimit -v 4194304` | ten tabs fit at 1280 px (end at 861 of 1216 px); at 400 px the drawer lists all ten ([K15a]) |
| T11 one log, many runs | `recheck3/mergelog/` | pre-made per-run headings merge cleanly; appends at the end conflict ([K14]) |

**UNVERIFIED (each run's first step, with a stop rule):** running the selector painter on the host (S2 compiled only,
½ day); Calibrate's screens (same seam as Tune, unprobed); feeding a docs example into the route preview's
`renderSvg` (½ day); the real route-builder app inside the docs build (T7 stubbed its two files; RMP1-A1's own PG1
step is the proof); how long the rewrite of ~64,000 words really takes (one stated rate, not measured — P1a measures
it); the speed of five parallel jobs before a usage stop; **whether the extra usage stays on** (every date assumes
it). **[K15a] Now verified:** ten tabs fit at 1280 px and the drawer reads well at 400 px (T10) — a run can screenshot
its pages the same way (RMP1-A1's capped setup, `tools/route_maker/README.md` :32-37).

### A.11 New and changed decisions (for the team lead, plain words) — **re-checked 2026-11-15 late**

**In one breath:** the docs get a menu by task (Start here, Route builder, Robot, Routes, Mechanisms, Troubleshoot,
Learn…), a front page that asks "What do you want to do?", one simple shape for each kind of page, pictures
everywhere (brain screens drawn by the code itself), and checks that refuse walls of text, internal history, the word
"honest" and status disclaimers. **Before the 21st, only the first piece can land: the menu, the front page and the
removal of every "honest" and status disclaimer (~Nov 19; ~Nov 20 after one usage stop) — and only if the extra usage
stays on and the robot-code work ahead of it finishes on time. Guide 18 starts before the 21st and lands after it
(~Nov 22).** The beginner track, the route pages and the bench pages land ~Nov 24–26; everything else in December.

- **D13 — the menu, by task; nothing moves.** **Ten tabs** ([K2]): Home · Start here · Route builder · Robot · Routes ·
  Mechanisms · Troubleshoot · Learn · API reference · Project (the Route builder tab is the route builder's own,
  already approved; ten fit a laptop screen — checked in a browser). Titles lose their chapter numbers; files keep
  them, so no link, test or bookmark breaks; every "Chapter N" that is not yet a link becomes one in the first piece,
  so no reader is left looking for a number. **Recommended.** (b) Also rename the files — not recommended: six tests
  open guide pages by name, ~900 links and people's bookmarks use them ([K15b]: the robot code itself names only one
  numbered page).
- **D14 — one shape per kind of page** ([K5]): a "how do I…" page has: what it is for (one short paragraph), a picture,
  how to use it (steps), a complete example, what it cannot do, more detail. Recipe pages are lists of recipes, each
  naming its "how do I…" page; the troubleshooting page is a list of symptoms with steps; reference pages (the log
  catalogue, the glossary, the FAQ, the rules) keep a reference shape. **Recommended.**
- **D15 — the history goes, and checks keep it out.** Chunk names, dates and register numbers move to the changelog (or
  a small folded "History" box at the bottom); an estimate links straight to its row in the register. A checker fails a
  rewritten page with a paragraph over 80 words, more than 250 words of text with no picture, list, table or code, a
  sentence over 40 words, or no picture. **The word "honest" (and "honesty") and the "this has not run on a robot yet"
  openings go from EVERY page in the first piece, and a check keeps them out from then on** ([K6]); real limits stay as
  plain facts in each page's "What it cannot do" and in "What it cannot do yet". **Recommended.**
- **D16 — brain-screen pictures are drawn by the screens' own code**, and a test fails when a screen changes until
  the pictures are redrawn. Proven for Tune. Bench Tests can only be drawn from its layout until its drawing code is
  moved (robot code, after the 21st). **Recommended.**
- **D17 — order and dates (CHANGED).** Since the first draft, four more robot-code jobs joined the queue before the
  freeze (the starter route, MIRROR1-B, the Tune label fix, READ1), the route preview's switch already took a slot, and
  the weekly usage limit was hit — so every date assumes the extra usage you are buying stays on. **(b) Now
  recommended: the menu first** — it starts ~Nov 16 midday in the first slot robot code does not need, and goes live
  ~Nov 19 (~Nov 20 after one usage stop, still before the 21st); **guide 18 starts in the very next slot (~Nov 17)
  and lands ~Nov 22.** (a) Guide 18 first (the first draft's pick): guide 18 ~Nov 21 — the competition day, no use to
  it — and the menu ~Nov 20, which one stop pushes to the 21st. (c) The beginner track's skills-route page first: that
  page ~Nov 20–21, the menu ~Nov 20, guide 18 ~Nov 23. **Guide 18 lands after the 21st in every order;** the menu can
  only land before it with a margin if it goes first. After the 21st: the beginner track ~Nov 24–25, the route pages
  ~Nov 24–25, the bench pages ~Nov 25–26, everything else ~Dec 1–4 (later with usage stops).
- **D18 — guide 8** (the X-drive "first routine") moves to Learn as "A routine built by hand" and is rewritten
  later; the beginner track's page 4 is the first route a student reads. **Recommended.**
- **D19 — comments in the beginner track's examples (reopens D8).** The builder asked for "minimum comments just as
  simple as possible for people getting introduced to the library", and you both found heavy comments hard to
  read. **Recommended:** short comments only where a line is not obvious, and every line explained in the page's
  walkthrough instead (the checker then requires every line of the program to appear in some explained piece, and
  no comment longer than one line). (b) Keep D8: a comment on every line.
- **D20 — a new page, "Bench Tests: check every motor and sensor"** (page 23 — the route builder's guide took 22),
  because "check the robot on the bench" has no page today. **Recommended.**
- **D21 — a docs release each time a docs run lands**, through the full release check. **Recommended.**
- **D22 — guide or cookbook?** Each "how do I…" page's complete example IS the recipe; the cookbook pages stay as
  quick-copy collections, each naming its page. Whether to fold the cookbook away is decided later. **Recommended.**
- **Not a decision — heads-ups the checks found (for the coordinator):** (1) "shulib Tune"'s first screen shows its
  green button as **"READY - I AM CL"** (the word CLEAR is cut off by a too-small text box in the code) — a one-line
  code fix for a robot-code run before the Nov 18 freeze, not a docs run (§A.5). (2) **ROB1-BT renamed a heading in
  cookbook 06, and the cookbook's index still links the old name** (`cookbook/README.md` :102 →
  `#a-whole-driver-file-robot-twos`) — a broken link today's site build would publish without a word; ROB1-BT's
  continuation should fix it ([K13]). (3) READ1's build, after ROB1-BT, merges at or just past the Nov 18 freeze
  (§A.7) — it edits robot files, so that is the coordinator's call to make with you.

---

> **THE TEAM LEAD'S ANSWERS (2026-11-12 late):** **"Go, after robot code (Recommended)"** — D1–D12 as recommended after the
> re-check; DOCS3-A1 starts ~Nov 14 in the slot ROB1-C frees, AFTER the route preview; robot code is never delayed; page 4
> first; anything unfinished at noon Nov 18 becomes a page of links; the track ships ~Nov 19–20 as a docs-only publish
> (a third release — authorized with this go, docs only). **D11's front-page wording: "Yes, use that wording"** — "Our
> robots have been X-drive and H-drive, which LemLib cannot drive at all — that is why this exists. This season both of our
> robots are tanks, and shulib drives them as well." (and the :28 sentence as proposed).

> **THE TEAM LEAD'S RULING (2026-11-12), as given to this writer (BINDING):**
> THE CHUNK: DOCS3-A — the first part of DOCS3 (the documentation redo, REMAINING-WORK row 19b, ~5–8 days, "after the
> panel"). THE TEAM LEAD, 2026-11-12: "how much more is left till we can update the full docs so people can use the most up
> to date docs and learn how to code with our library with helpful fully commented beginner friendly examples" → offered a
> split, he chose "Split: beginner track now": DOCS3-A, a BEGINNER TRACK "Start here: coding your robot with shulib", lands
> before the first competition (2026-11-21; published with the release at the ~Nov 18 code freeze — docs change no robot
> code, so a docs run may build Nov 15–18 and even after the code freeze); DOCS3-B (the full reorganisation: one front door
> per reader, navigation, the API reference's layout, re-checking every claim) AFTER the 21st. The team's situation: no
> panel; SKILLS ONLY; BOTH new robots run the skills route; robot one (4-stage string cascade, a flipper, one front AI
> Vision camera, side distance sensors, a tank) built ~Nov 14; the STUDENTS write the skills route and driver choices (G4 —
> the docs teach, they never write the team's route for them). THE TRACK, ~6 pages, each a complete WALKTHROUGH built
> around ONE complete example program that a beginner can read top to bottom, EVERY line commented in plain words (what it
> does and WHY), and every example COMPILED AND RUN by the host suite on the host test plant (the repo's check-examples gate
> requires quoted code to be verbatim from compiled sources — find how guides 09/18/19 do it: test/*_example_test.cpp, the
> 'guide-NNx' markers, tools/api_doc_tool.py check-examples) so no example can go stale: (1) set up and build (PROS, make
> ROBOT=… PROGRAM=…, uploading, the slots, what each program is); (2) your robot file by hand (robot one's facts file and
> declarations — ROB1, in flight: the OWED-facts scheme; the cascade as a Lift with measured rows, homed by stalling down
> under a current cap; the flipper's three kinds); (3) driving and turning in an auton (moveTo/driveTo/turnTo/face,
> timeouts, per-step tolerances STEP1-A, "never stuck": the watchdog, the stall check, the 60 s clock, why a tank cannot
> slide sideways and what to do); (4) WRITING A SKILLS ROUTE STEP BY STEP — the centrepiece: a full 60 s skills auton built
> up in stages (start pose, a wall reset L6-A2, onFail carryOn/skipTo/label STEP1-A, cascade actions partway through a
> drive STEP1-B triggers, a curve CURVE1 with the run-up rule and the one-line switch back to straight legs, the blind
> two-push Toggle flip and the checked check-and-flip AF1-S), and how to read its result; (5) mechanisms and the cascade on
> the controller and in autons (presets, holds, homing, the current cap; tuning on the robot with "shulib Tune" TUNE1-B1
> and pasting the COPY block back); (6) testing before the robot is on the field (the Auton Tester, the competition
> program's screen, the route preview RPV1 if it has landed, reading the logs). Several features are IN FLIGHT and land
> ~Nov 13–17 (STEP1-B, CURVE1-A, AF1S-A1, ROB1-A, RPV1) — the brief must say which pages wait for which merge, and how the
> track ships honestly if one slips (that page says "coming" and links the current reference, never a guessed API). Robot
> two (src/robots/*) is today's only worked example; robot one arrives with ROB1. Also in scope: the guides' entry page
> (docs/guide/README.md) and the site nav (mkdocs.yml) pointing a beginner to the track FIRST; the existing guides 01–19
> stay as the reference (DOCS3-B reorganises them later); the camera guide clash (two chapter-19 files: CAL1's and
> VIS1-A2's — the coordinator renumbers the camera's at its merge; the track must link by the final name).

> **(amended 2026-11-13) D1–D12 were ANSWERED 11-12 late (the block above); the navigation and readability
> amendment at the top adds D13–D22, and its §A.7 replaces the runs and dates below.** The 11-12 status, kept:
> **Status 2026-11-12 (late evening): DRAFT for the team lead's go — WAITS for his answers to D1–D12 below, then a go per
> run (§1.1). Nothing is built. RE-CHECKED the same night: 13 checker findings, all re-verified, 12 real and one partly (a
> line number); plus 3 found by this writer. The body is corrected in place, each change marked **[F1]–[F13]** / **[S1]–[S3]**;
> the list, with the evidence, is "## Re-check — amendments" at the end.** Written by the coordinator's brief writer from a READ-ONLY pass. **Tree: `c0a7c7e`**
> (branch `shulib-v2`, TUNE1-B1 committed); the main worktree's INDEX carries **CURVE1-A's squash, STAGED** (40 files,
> gates running — §6.1). **Every `file:line` below was OPENED at `c0a7c7e`** (`git show c0a7c7e:<path>`) unless marked
> *(staged)* (the main worktree's index — CURVE1-A) or *(worktree `x`)* (an in-flight run, read only, never edited).
> **Probed, not argued:** three probes in `.claude/coordinator/scratch/brief-DOCS3A/` (§0.5), every binary under
> `(ulimit -v 4194304; timeout 300 …)`, compiled with the suite's exact flags (`build/test/compile_commands.json`).
>
> **Executor model: `opus`. Do not commit. Live log `docs/internal/chunks/DOCS3-PROGRESS.md`, created FIRST** (one chunk,
> DOCS3; a heading per part — A1, A2; B later — and the release preflight's between-parts marker, §6.4). Say "host test
> plant", never "simulator" (the track itself too — §3.4 greps it). **No hardware claim anywhere:** the library has never
> driven a robot under its own control (`docs/internal/PROJECT-BRIEFING.md` :184-187); every result the track shows is the
> host test plant's, and every page says so once, in plain words. **(amended 2026-11-13)** — said in place, where a result is shown ("On
> the host test plant this prints: …"), never as a status preamble or an "honest" paragraph (the team lead's tone
> rule; rule T, §A.4). **Hardware Assumptions: NONE** (§4). **No robot code:**
> DOCS3-A changes nothing under `src/` or `include/` (§7's first check) — which is what lets it merge after the freeze.
> **Memory rule (2026-10-15):** every binary — probes, the suite, any test — runs as `(ulimit -v 4194304; timeout 300 …)`;
> the suite only through `python3 tools/suite_run.py`.

---

## DECISIONS FOR THE TEAM LEAD (a recommendation for each; nothing below decides them silently)

**D1 — Where the track lives.** **(amended 2026-11-13)** Still `docs/start/`, as the SECOND tab of §A.3's menu (after
Home); there is no "User guide" tab once DOCS3-N1 lands — the guide's pages sit in task tabs. A1 adds its 7 menu lines.
- **(a) RECOMMENDED: a new section, `docs/start/`, its own first tab on the site ("Start here"), before the User guide.**
  A beginner opens the site and the first thing they see is the track. Costs: three of our tools list the documentation
  folders by name, so each must be told about the new folder — otherwise the new pages would NOT be checked at all (probe
  P3 proved it: a made-up line in `docs/start/` passes the example gate silently; the same line in `docs/guide/` fails it).
  That is about half a day, done first, with a test that the new folder is checked.
- (b) Guide chapters 22–27. No tool changes, but the track would sit after 21 reference chapters and the guide's own page
  says "start at Chapter 1 and read in order" — a beginner would never find it first. Chapter numbers are also moving
  right now (20 held for the panel, 21 = robot one, the camera's 19 to be renumbered).
- (c) The cookbook. Wrong kind of book: the cookbook is read out of order mid-task; the track is read in order once.

**D2 — How many pages, and their order.**
- **(a) RECOMMENDED: six pages plus a one-screen welcome page, in the order of the ruling:** 1 set up and build → 2 your
  robot file → 3 driving and turning → 4 writing a skills route → 5 mechanisms and the cascade → 6 testing before the
  field. (Page 5 after page 4 on purpose: page 4 uses the cascade's presets in one line each; page 5 is where a student goes
  to make those presets right on the real robot.)
- (b) Five pages (1 and 2 merged). Shorter, but page 2 (the robot file, the OWED facts) is the page robot one's
  programmers will live in during robot week; it deserves its own.
- (c) Seven (page 4 split into "building it" and "reading its result"). **[F13]** Page 4 WILL pass ~500 lines: six
  growing complete programs plus their walkthroughs and output come to **~700–900 lines** (sized in §1.4 page 4). (a)
  stays recommended — one route read as one story, the site's right-hand contents jumps to each stage, and each stage's
  walkthrough shows only the lines that stage ADDS — but if the team lead prefers two shorter pages, (c) costs nothing
  extra and is decided now, not mid-run.

**D3 — One growing example, or one per page.**
- **(a) RECOMMENDED: one PRACTICE ROBOT everywhere (D4); page 4's skills route GROWS in stages on that one page — each
  stage a complete program of its own, compiled and run and checked; pages 1, 3, 5 and 6 each have their own small
  complete example on the same robot.** A student can open any page without having typed the others, and a change to page
  3's example cannot break page 4.
- (b) One example that grows across all six pages. Reads like a story, but every page depends on every earlier page, and
  one change ripples through the book.
- (c) Unrelated examples per page. Loses the "it is all one robot" feeling.

**D4 — Which robot the examples use.**
- **(a) RECOMMENDED: a PRACTICE ROBOT declared in the track's own example files, shaped exactly like robot one** (a tank,
  a 4-stage string cascade as a Lift with measured rows, homed by stalling down under a current cap, a flipper arm, side
  distance sensors, a front camera), **with round, obviously made-up numbers and a comment on every one saying "your
  robot's number comes from Calibrate / Tune / the tape".** Page 2 then shows, side by side, where the SAME lines live
  for robot one (its facts file and its mechanism list, from ROB1) — by link, not by copying.
  **(amended 2026-11-13)** Robot two is now robot one's build (the team lead, 11-13: "IT IS LITERALLY THE SAME ROBOT
  BUILD JUST A MIRRORED ROUTINE"; ROB1-BT), so the practice robot is shaped like both; the reasons below about
  placeholders still hold. The 11-12 text: Why not the real robots: robot two has claws, no cascade, flipper or camera; robot one's file is full of placeholders
  (OWED facts) that change as the builder types them during robot week — an example built on them would change its
  results mid-week and would show placeholders as if they were robot one's numbers.
- (b) Robot one's real declarations. Exactly the team's robot, but see above; and its file is written for the robot, not
  for a first reader (`facts::kCascadeTeeth.value().motor`, *(worktree `rob1-a`)* `src/robots/mechanism_table.hpp`
  :174-191).
- (c) Robot two. Real and merged, but it has none of the cascade robot's parts.

**D5 — The reading level.**
- **(a) RECOMMENDED: a first-year student who knows a little C++** — variables, functions, `if`, and that `r.driveTo(…)`
  calls `driveTo` on `r`. No robotics background assumed; every word the library uses (pose, heading, preset, settle,
  stall) defined in one sentence where it first appears, and in the glossary.
- (b) A student who has never programmed. Would need a C++ course inside the track; out of reach by Nov 18.

**D6 — How much C++ the track explains.**
- **(a) RECOMMENDED: exactly what its examples use, in a small "The C++ on this page" box at first use:** `#include`;
  `using namespace`; a function that takes the robot (`AutonContext& ctx` — "the robot, handed to your auton"); options by
  name (`{.timeout = 3_s}`); unit words (`24_in`, `90_deg`, `2_s`); `constexpr` declarations (page 2); and a lambda only
  where a step needs one (`r.then([&] { … })` for the wall reset — "a small piece of code handed to the step, run when the
  step's turn comes"). Nothing else: no templates, classes or spans.
- (b) A separate C++ primer page. More to write and keep true; a box at first use is read, a primer is skipped.

**D7 — How close page 4's route comes to a real skills run.**
- **(a) RECOMMENDED: a PRACTICE ROUTE on the real field** — it uses real walls (for the wall reset) and a real Toggle (for
  the flip), and it uses every feature once, but its order and choices are made for teaching, not scoring (for example:
  one wall reset, one curve, one cascade lift on the way, one Toggle). Each stage ends with "your turn: what would you do
  here?" — the questions a student answers when writing the team's own route, never the answers.
- (b) A plausible real skills plan. Rejected: that would be writing the team's route for them.
- (c) A made-up field with no Override parts. Rejected: the wall reset and the Toggle step only work against real walls
  and Toggles.

**D8 — "Every line commented": how it is kept true.** **(amended 2026-11-13) REOPENED by D19 (§A.11)** — the
builder's 11-15 "minimum comments just as simple as possible for people getting introduced to the library". If D19
(a) is chosen, rule 3 (§1.6) changes as D19 says; if (b), D8 stands as written below.
- **(a) RECOMMENDED: a machine rule for PRESENCE, a person for QUALITY.** The documentation gate refuses any code line in a
  track listing that has no comment OF ITS OWN — on the line, or on a comment line directly above it (at least three
  words). **[F1]** A line that merely continues a commented line does NOT count: in the first draft it did, and the checker
  showed one comment then covered a whole 17-line cascade declaration (re-check probe R1: the strict rule finds 15 of
  those 17 lines uncommented). Only lines of pure punctuation (`}`, `};`) are exempt. Whether a comment says what AND why
  is checked by the prose review, logged line by line for every listing. Probe P2 measured today's chapters by the looser
  rule: **216 of 338 code lines in eight chapters have no comment** (more under the strict rule) — so the rule is real
  work, and it applies to the track only (DOCS3-B decides about the rest).
- (b) A person only. Cheaper; rots the first time someone edits an example in a hurry.

**D9 — The runs, and when they take a job slot.** **(amended 2026-11-13) SUPERSEDED by §A.7 (D17):** five job slots
since 11-15 ~20:05; the 11-12 arithmetic below (three slots, A1 in ROB1-C's slot ~Nov 14) did not happen — the
session lost ~38 h after the 11-14 usage stop. A1's build order and its cut-line idea stand; its dates are §A.7's. **[F5, S1, S3] — rewritten at the re-check: the first draft's dates
did not fit the three-job limit, and its sizes were low.**

*The slot arithmetic (MAX 3 JOBS — the team lead, 11-11 ~13:15; the slot order he set 11-12, NOTES.md :984).* Running
now: the STEP1-B fix round, the VIS1-A2 fix round, and this brief. Next in his order: **RPV1-A0+B1** (~2½ builder-days
+ ~1 review, RPV1 brief :41-44), **ROB1-C** (~½, G1 brief :282), **AF1S-A2** (~1¼–1½, AF1S brief :719; it cannot start
before VIS1-A2 and AF1S-A1 are merged, ~Nov 13–14). Those three fill all three slots from ~Nov 13. The first slot that
frees is ROB1-C's, **~Nov 14 at the earliest** (~Nov 15 after one usage-limit stop). DOCS3-A1 cannot run as a fourth job.

*The honest size of A1 (re-summed — [S1]: the first draft's own parts, ½ + ¾–1 + 1¾–2¼ + ¼, add to 3¼–4, not the
3–3¾ it printed).* The checker's findings add work: the gate's two new rules and the stricter comment rule (+¼–½,
[F1, F2, F4]), the Tester-run examples on the existing rig (+¼, [F3]), page 4's six growing listings at ~700–900 lines
(+½, [F13]). **A1 ≈ 4¼–5¼ builder-days + ~1 review.** From ~Nov 14 that is **built ~Nov 18½–19½, merged ~Nov 19½–20½
— after the Nov 18 freeze release**, before any usage-limit stop (each ≈ +1 day).

- **(a) RECOMMENDED (changed): two runs, and A1 builds IN A FIXED ORDER WITH A CUT LINE.** **DOCS3-A1** takes ROB1-C's
  slot (~Nov 14), BESIDE RPV1 and AF1S-A2, never ahead of them (RPV1 has an `include/` hook and AF1S-A2 is robot code —
  both ride the freeze; docs do not). It builds in this order: the gate rules → the practice robot's rig (its probe first,
  §1.5) → **page 4** → page 3 → page 5 → page 2 → page 1 → page 6 → the welcome page and the front door. **At the cut line,
  Nov 18 at noon,** every page not finished becomes a short guide-post page (what the page will teach, and links to the
  guide chapters that teach it today — no listing), and the run finishes the front door and stops. Review ~1 day →
  merge ~Nov 19 → **one docs-only publish, ~Nov 19–20 (D10 (a))**. Page 4 — the one the students need in robot week — is
  built first, so it is the page most sure to ship. **Say it plainly: the track most likely reaches the students on
  ~Nov 19–20, not with the Nov 18 freeze release, and only one or two days before the competition.** It joins the freeze
  release only if no stop hits and the builder is fast. **DOCS3-A2** (~½–1): the late landings — the checked Toggle flip
  (AF1S-A2), the route preview (RPV1), the camera chapter's final link, and any page the cut line turned into a
  guide-post — after the 21st unless A1 merges early.
- (b) One run for everything. It would have to wait for the last in-flight feature (AF1S-A2 / RPV1, ~Nov 17+), and
  would not merge before the 21st.
- (c) Trimmed A1 from the start: pages 3, 4 and 5 in full with the gates and the rig; pages 1, 2 and 6 as guide-posts to
  chapters 7, 16, 17 and 21. **[S3] Re-sized: ~3½–4 builder-days, not the 2–2½ first printed** — the gates and the rig
  alone are ~1½–2. (a)'s cut line reaches the same result only if time runs short, so (c) is worse than (a).
- (d) Give A1 a slot ahead of RPV1-A0+B1. Lands the track ~1½–2 days sooner (~Nov 18). Costs: the route preview later,
  and RPV1's `include/` hook (RPV1 brief :52) closer to the freeze. Not recommended: the team lead's 11-12 order puts RPV1
  first, and the preview is what lets the students check the route they write — but it is his trade to make.

**D10 — Publishing.** **(amended 2026-11-13) SUPERSEDED by D21** (a docs release each time a docs run lands).
- **(a) RECOMMENDED: the track publishes with the freeze release (Nov 18) if it is merged by then; otherwise — the
  likely case, [F5] — ONE docs-only publish before the 21st (~Nov 19–20)** — allowed only when `src/` and `include/` are
  unchanged since the freeze release (§7) and every gate is green.
- (b) Only with the freeze release; anything later waits for after the 21st.

**D11 — What is OUT (DOCS3-B, after the 21st), and one small exception.** **(amended 2026-11-13) DONE and partly
superseded:** the approved wording landed 11-13 (`52313b7`, `055ab0b`; `docs/README.md` :67-78 at `9befdb3`), so A1
does not edit those lines; and DOCS3-B's navigation, front door and readability are brought forward (§A.2–§A.7).
- **(a) RECOMMENDED:** OUT — reorganising guides 1–21 and the cookbook, one front door per reader, the API reference's
  layout, re-checking every claim, the "never driven" sweep, rewriting chapter 8. **The one exception, IN A1: the front
  page's sentence "Ours are X-drive and H-drive"** (`docs/README.md` :28, :32-33 — both robots are tanks this season) is
  corrected in the same edit that adds the track's "Start here" link, because a beginner reads it just above the link.
  **[F12] It is the page's stated reason for the library, so the exact words are for the team lead to approve:**
  - :28 today: "…same time and independently — our robots slide sideways, so the library is built around that rather
    than treating it as an afterthought)" → **proposed:** "…same time and independently — an X-drive or H-drive robot
    can slide sideways, so the library is built around that rather than treating it as an afterthought; it drives tank
    robots too)".
  - :32-33 today: "Most VEX teams use LemLib, which drives only tank robots. Ours are X-drive and H-drive, so LemLib
    cannot drive them at all. That is why this exists." → **proposed:** "Most VEX teams use LemLib, which drives only tank
    robots. Our robots have been X-drive and H-drive, which LemLib cannot drive at all — that is why this exists. This
    season both of our robots are tanks, and shulib drives them as well."
  - **For the team lead to confirm:** that "our robots have been X-drive and H-drive" is true of the team's earlier
    robots (this writer read it from the page, not from a record), and that he wants the reason kept rather than cut.
    Nothing else on the front page changes in A1.
- (b) Leave even that for DOCS3-B.

**D12 — Chapter 8, "Your first routine".** **(amended 2026-11-13)** It moves to the Learn tab as "A routine built
by hand" (D18) and is rewritten in phase 3; the one line at its top stays. It is an X-drive tutorial that builds the whole robot by hand and slides
sideways (`test/guide_examples_test.cpp` :108-133, :169-215) — not how a team auton is written today (`src/team/autons.cpp`
:66-70).
- **(a) RECOMMENDED: one line at its top** — "Coding your team's robot? Start with the Start here track; this chapter shows
  the machinery underneath" — and nothing else until DOCS3-B.
- (b) Leave it untouched. A beginner who follows the guide's "start at Chapter 1" still lands there.

---

## The design in plain words (for the team lead)

- **What a student gets:** a "Start here" tab with a welcome page and six pages. Each page is one walkthrough around ONE
  complete program they can read top to bottom; every line has a comment that says what it does and why.
- **Why it cannot go stale:** every program shown is a real file in the test suite. The suite compiles it, RUNS it on the
  host test plant, and checks what the page says happens (where the robot ends, which steps finished, what the log says).
  The documentation gate checks that every line on the page is copied exactly from that file, that the page shows the
  WHOLE program (nothing skipped), that every code line carries its OWN comment, that every output line the page shows
  is one the test checks, and that no code block on the page escapes those checks. **[F1, F2, F4]** If anyone changes
  the library and an example stops working, the build goes red before the site does. (What the gate cannot judge — that
  a comment explains WHY, that a test really asserts the output it names — is a person's review, logged.)
- **The robot in the examples** is a practice robot shaped like robot one, with round made-up numbers; page 2 shows where
  robot one's real numbers go and how the build refuses until they are typed.
- **The centrepiece (page 4)** builds a 60-second practice skills route in stages: a start, a wall reset, what to do when
  a step fails, the cascade starting partway through a drive, a curve (with its one-line switch back to straight legs), and
  the Toggle — blind two-push first, the camera-checked step when it lands — then how to read the run's result. Each stage
  ends with questions for the student; the team's real route stays theirs to write.
- **If a feature is late:** its section says "coming" and links today's reference — never a guess at code that does not
  exist yet.
- **(amended 2026-11-13) What it costs now:** §A.7 — A1 ~4–5 builder-days + review, after the menu (DOCS3-N1), most
  likely merged ~Nov 22–23 with five job slots. The 11-12 text: **What it costs:** two runs (~4¼–5¼ and ~½–1 builder-days), beside the robot-code runs, never ahead of them. **[F5]**
  With three job slots, the first run can start ~Nov 14 and most likely reaches the site ~Nov 19–20 as a docs-only
  publish, not with the Nov 18 release. It builds page 4 first; on Nov 18 at noon any unfinished page becomes a short
  page of links instead (D9).
- **(amended 2026-11-13)** The menu, the front page and the rewrite of every other page are no longer "later":
  they are runs N1, N2, P1a–c and phase 3 of this brief (§A.7). The 11-12 text: **What it does not do:** reorganise the
  existing 19–21 chapters, the cookbook or the API reference (DOCS3-B, after the
  21st).

---

## 0. Why — what exists, what a beginner meets today, and the probes (every line cited was OPENED at `c0a7c7e`)

### 0.1 Why this chunk, and why now

Robot one is built ~Nov 14; robot week is Nov 14–21; the students write the skills route and the driver choices in that
week. The library they write it with grew fast (STEP1-A per-step tolerance and on-failure, L6-A2 the everyday wall reset,
MECH-S-A the current cap, TUNE1-A/B1 live tuning, CURVE1-A curves staged, STEP1-B partway triggers and ROB1-A robot one in
flight), and every one of those features was documented where its chunk landed — in the reference chapters, for a reader
who already knows the library. No page shows a student, start to finish, how to write a skills route with it.

### 0.2 What a beginner meets today (honestly)

| Where | What it says | Why it hurts a beginner |
|---|---|---|
| `docs/README.md` :42-46 **(amended 2026-11-13: rewritten 11-13 — now "What do you want to do?", :9-63 at `9befdb3`; what is still wrong: §A.1 W1)** | "New to this? Read the user guide in order." | The front door sends a beginner to 19 chapters (~8,000 lines; `wc -l docs/guide/*.md`) |
| `docs/README.md` :26-33 | "our robots slide sideways …", "Ours are X-drive and H-drive" | **Wrong this season**: both robots are tanks (D11) |
| `docs/guide/README.md` :3-5 | "Start at Chapter 1 and read in order" | Chapters 1–6 are concepts with no code; the first code is chapter 8 |
| `docs/guide/08-your-first-routine.md` :1-40 + `test/guide_examples_test.cpp` :108-133, :169-215 | The "full tutorial": an X-drive built by hand (`SimHarness`, `PilonsOdometry`, `Localizer`, `MotionDeps`, a pacer), `strafeTo` | Not how a team auton is written (`Routine r{ctx, "…"}` inside `src/team/autons.cpp` :66-70); a tank cannot `strafeTo` (D12) |
| `docs/cookbook/README.md` :11-14 | "never written a routine? start with Chapter 8 and Chapter 9" | Points back to the X-drive tutorial |
| `docs/guide/16-testing-an-auton-on-the-bench.md` :67 ("The team's file"), :340 ("Writing the next auton") | The real shape of a team auton, on robot two's claws | The closest thing to the track today — but bench-first, no skills route, no cascade |
| *(worktree `rob1-a`)* `src/team/robot_one_autons.cpp` :50-51 | "THE SKILLS ROUTE GOES HERE (the students')" | The place the students will write — no page walks them there |
| Probe P2 (§0.5) | 216 of 338 code lines in guide 08, 09, 16, 17, 18 and cookbook 01, 02, 07 carry no comment | The ruling's "fully commented" is not today's style |

### 0.3 How the documentation is kept true today — and the three holes for this track

- **The gate.** `tools/api_doc_tool.py` `do_check_examples()` (:2020-2116): every non-blank line inside a ```` ```cpp ````
  block of a public document must appear VERBATIM (whitespace-stripped) in a compiled source — `EXAMPLE_SOURCE_GLOBS`
  (:1947-1948): `test/*example*_test.cpp`, `src/team/*_driver.hpp`, `src/team/autons.cpp`. For the WHOLE-RUN kinds
  (`WHOLE_RUN_SOURCE_GLOBS` :1972, the two team files) a listing must also be one unbroken run of the file — and the
  comment at :1964-1969 says the example tests were deliberately left OUT of that rule ("a recipe may still leave out
  lines of its test"). It runs in the build (`test/CMakeLists.txt` :273, inside the doc-gate stamp, whose inputs are
  globbed at :213-219) and in CI (`.github/workflows/ci.yml` :86-92). The test files carry the chapter markers by test
  name (`guide-08a` … — `test/guide_examples_test.cpp` :1-19; `test/routine_on_fail_example_test.cpp` :1-8, :122;
  `test/mechanism_chapter_example_test.cpp` :79-139).
- **Hole 1 — a new folder is NOT scanned.** `PUBLIC_DOC_GLOBS` (:1916) lists `docs/*.md`, `docs/guide/*.md`,
  `docs/cookbook/*.md`, `docs/api/*.md` only. Probe P3: a line found in no source FAILS in `docs/guide/` and PASSES in
  `docs/start/` (`scratch/brief-DOCS3A/p3.out`). The same list is repeated in `tools/doc_staleness_audit.py` :62
  (`LIVE_GLOBS`) and :68 (`PATH_GLOBS`) and in the build's doc-gate inputs (`test/CMakeLists.txt` :216-218).
- **Hole 2 — "verbatim" is not "complete".** Membership lets a listing drop lines; for example tests even the whole-run
  rule does not apply (:1964-1969). The ruling asks for a COMPLETE program a beginner reads top to bottom: a listing that
  silently skips a line of the program (a `return r.result();`, a registration line) would pass today.
- **Hole 3 — nothing checks comments.** "Every line commented" has no machine anywhere.
- **The one-mechanism rule.** `docs/internal/guide-maintenance.md` :41-45: "Every public document is covered by exactly
  one of the two mechanisms; do not add a third." So the track's new rules are written as RULES INSIDE `check-examples`
  (the same scan, stricter for `docs/start/`), not a third mechanism (§1.6, landmine 9).

### 0.4 What each page can stand on (merged at `c0a7c7e`, staged, or in flight)

| Page | Needs | State on 2026-11-12 |
|---|---|---|
| 1 Set up and build | `make ROBOT=… PROGRAM=…`, the slots, the programs (`Makefile` :74-166; `tools/src_build_gate.py` :33-75 the build list); guide 7 | merged for robot two (`ROBOT=tank`); **`ROBOT=one` is ROB1-A's** *(worktree `rob1-a`: `Makefile`, `src/build_selection.hpp`)*; Tune for robot one needs its `MECHANISM_ROBOTS` entry at ROB1-A's merge (NOTES 11-11) |
| 2 Your robot file | the five mechanism kinds, `measured()` rows, `homeByStall` (`include/shulib/mechanism/kinds.hpp` :322-326), `.currentCap` (:211-223), guide 18; robot one's facts file and OWED scheme | kinds merged; **robot one's file, facts form and chapter 21 are ROB1-A's** *(worktree `rob1-a`: `src/robots/robot_one_facts.hpp`, `docs/guide/21-adding-your-robot-by-hand.md`, `docs/robot-facts-form.md`, `test/robot_one_flipper_forms_example_test.cpp`)* — DONE, merge next after CURVE1-A |
| 3 Driving and turning | `Routine` verbs (`include/shulib/chassis/routine.hpp` :414 startAt, :436 driveTo, :486 face, …), `.positionTolerance`/`.headingTolerance` (`chassis/chassis.hpp` :248, :262 — STEP1-A), the TANK1 steering (`src/team/autons.cpp` :55-65), the 60 s clock (`competition/auton_registry.hpp`; `src/team/autons.hpp` :26-27) | **all merged** |
| 4 Skills route | start pose; `competition::resetFromWall` (`competition/auton_wall_reset.hpp` :1-40, :105); `onFail`/`carryOn`/`skipTo`/`label` (`routine.hpp` :283-302, :663, :697); partway triggers; `curveTo` + `kUseCurves` switch + the run-up rule; the Toggle | wall reset, onFail **merged**; **curves STAGED** (*(staged)* `routine.hpp` :546-573, `test/cookbook_examples_test.cpp` :261-296, `docs/guide/05-getting-there.md` :189-205); **triggers = STEP1-B** *(worktree `step1-b`: `atPercent`/`afterDistance`/`afterTurning` … `startMove`, `test/routine_trigger_example_test.cpp`)*, fix round running; **checked flip = AF1S-A1 (explicit form) DONE, merge after VIS1-A2; the everyday form `checkAndFlipToggle(AutonContext&, …)` = AF1S-A2, queued** (`briefs/AF1S-skills-toggle-check-flip.md` :284, :562; *(worktree `af1s-a1`)* `competition/toggle_visit.hpp` :1-16); the blind two-push flip needs only mechanism steps — **merged** |
| 5 Mechanisms and the cascade | presets, holds, homing, the cap (guide 18 :160-370); the driver file's controls (guide 18 :694; `src/team/robot_two_driver.hpp`; cookbook 06); "shulib Tune" and the COPY block (guide 18 :425-600; `Makefile` :166) | **merged** (robot one's driver file = ROB1-A) |
| 6 Testing before the field | the Auton Tester (guide 16), the competition program's screen (guide 17 :178), reading the logs (guide 11), the route preview | Tester, Competition **merged**; **route preview = RPV1-A0+B1, approved, not started** (`briefs/RPV1-route-preview.md` :3-4) |

The camera chapter: VIS1-A2 *(worktree `vis1-a2`)* adds `docs/guide/19-the-camera.md`, colliding with CAL1's
`19-calibrating-your-robot.md`; ROB1-A takes 21; G3a-P1 holds 20 unmerged (NOTES 11-12 ~23:xx: "camera = 22?"). **The
coordinator renames it at VIS1-A2's merge; the track links it only after that merge, by the name on main** (landmine 6).

### 0.5 The probes (`.claude/coordinator/scratch/brief-DOCS3A/`, run 2026-11-12; every binary under the caps)

- **P1 — a fully commented beginner example compiles under the suite's flags, RUNS, is asserted, and a mutant turns it
  red** (`p1_start_example_test.cpp`, `p1.out`). Compiled with the suite's exact command (`-Werror -Wshadow -Wconversion
  …`) against `test/mechanism_auton_rig.hpp`'s robot x (a 12-in tank, a two-motor lift), run through the real
  `competition::runAuton`: driveTo with a timeout, turnTo with `.headingTolerance`, `onFail(carryOn)` + `.positionTolerance`,
  a lift preset, `return r.result();` — **every statement line with a trailing plain-words comment, no warning**. 6/6
  assertions pass (ok, 4 steps, 4 completed, the plant's true pose at (24, 24) within 1.5 in, facing 90° within 0.1 rad).
  **The flipped-turn mutant: 3 assertions red.** And the first draft FAILED on the plant: its lift bound, 2 s, was a guess
  — the run said `'lift': TIMED_OUT in 2.01s` and the routine STOPPED at step 4. The asserted run caught a number a
  compile-only example would have published (landmine 2).
- **P2 — the "fully commented" rule, prototyped** (`p2_comment_check.py`, `p2.out`): a code line is covered by its own
  trailing comment, by a comment line directly above, or as the continuation of a covered statement; closing-brace lines
  are exempt. P1's listing: 0 of 7 uncovered; with one comment removed: 1 uncovered, exit 1. **Today's chapters
  (`c0a7c7e`): guide 08 28/66, 09 47/62, 16 18/31, 17 12/17, 18 27/63; cookbook 01 34/34, 02 47/55, 07 3/10 — 216 of 338
  uncovered.** Most of 09's are chained continuation lines (`.moveTo(…)` on its own line) — which is why the track writes
  one step per line, each ending in `;` and a comment (as `src/team/autons.cpp` :68 and P1 do). **[F1] The continuation
  clause is DROPPED from the track's rule** (§1.6 rule 3): it let one comment cover a whole multi-line declaration. Re-check
  probe R1 (`scratch/brief-DOCS3A/recheck/strict_comment_check.py`, `r1.out`): under the strict rule P1's listing still
  passes (0 of 7), its one-comment-removed copy still fails (1 of 7), and the checker's cascade (one comment) fails 15 of
  17, its chained trigger 5 of 7.
- **P3 — the new folder is unscanned** (`p3.py`, `p3tree/`, `p3.out`): `api_doc_tool.py`'s own `do_check_examples()`, run
  over a scratch tree (its `REPO` pointed there; the repository untouched): `r.driveTo(99_in, 0_in);` in
  `docs/guide/99-bad.md` → exit 1, named; the same line only in `docs/start/01-bad.md` → `0 quoted lines … exit 0`.
- **Not probed (UNVERIFIED — the run's first step, §1.5):** the practice robot's composite rig (a Robot with side
  sensors + a cascade on a lift plant with a bottom stop for stall homing under the cap + a flipper arm + the run guard
  and service, through `runAuton` with a FIELD start). Every part exists and is tested separately (§1.5); their
  composition is not.

---

## 1. Scope — IN (every new and changed file, by run)

### 1.1 The runs — two, each its own go, each verified and committed on its own

**(amended 2026-11-13)** The table below is the 11-12 plan. Now: A1's preconditions are ALL merged (CURVE1-A
`d33d25d`, ROB1-A `d684aa7`, STEP1-B `8df58f3`, VIS1-A2 `2c3aa34`, AF1S-A1 `5f6633b`, RPV1-A `9befdb3`); the camera
chapter's final name is `guide/20-the-camera.md`; page 6's route preview moves INTO A1 (RPV1-A merged); A2 keeps only
AF1S-A2's everyday Toggle form and any page a cut line left. Slots, order and dates: §A.7.

| Run | What | Starts | Size |
|---|---|---|---|
| **DOCS3-A1** | the gate rules (§1.6); the practice robot and its rig (§1.5); `docs/start/` README + pages 1–6 on everything merged on its base, **in D9 (a)'s order, with its Nov 18 noon cut line** [F5]; the front door (§1.3) | **[F5]** ROB1-C's slot, ~Nov 14 at the earliest (all three slots are taken by RPV1-A0+B1, ROB1-C and AF1S-A2 from ~Nov 13 — D9), and only once main carries **ROB1-A and STEP1-B** — rebases onto CURVE1-A's commit (staged now); if VIS1-A2/AF1S-A1 merge during the build, the coordinator ports them in at A1's merge | **[S1, F5] ~4¼–5¼ builder-days + the review loop (~1)** → merged ~Nov 19½–20½ uncut, ~Nov 19 with the cut line |
| **DOCS3-A2** | the late landings, each only once MERGED on main: page 4's checked Toggle step (AF1S-A2's everyday form; until then AF1S-A1's explicit form is NOT taught — §1.4 page 4), page 6's route preview (RPV1-A0+B1), the camera chapter's link, robot one's Tune line if it lands after A1, **any page A1's cut line left as a guide-post** | after A1 merges and at least one of AF1S-A2 / RPV1 has merged; **most likely after the 21st** [F5] | ~½–1 (more if pages were cut) |

DOCS3-B (after the 21st) is a separate brief.

### 1.2 New files (A1 unless marked)

- `docs/start/README.md` — the welcome page (~60–90 lines): who it is for (D5), what you need (a laptop, the repository,
  PROS — page 1), the six pages and what each leaves you able to do, the practice robot in one picture-in-words, **how the
  examples are kept true** (compiled, run, checked — one paragraph), and the honest status line (host test plant only).
  **(amended 2026-11-13)** No status line: the host-test-plant fact is said where results are shown (rule T, §A.4);
  the page obeys every readability rule but L.
- `docs/start/01-set-up-and-build.md` … `06-testing-before-the-field.md` — the six pages (§1.4), each ~200–400 lines
  (**[F13] page 4 ~700–900**).
- `test/start_practice_robot_example.hpp` — the practice robot's declarations (the cascade, the flipper, the side
  sensors' block, the camera block when A2 teaches it), every line commented; quoted on page 2 as its "robot file".
  (A header, not a `_test.cpp`, because pages 3–6's tests include it; the gate's new kind glob covers it — §1.6.)
- `test/start_rig.hpp` — the practice robot's host rig (§1.5). Not quoted; test-only.
- `test/start_01_first_auton_example_test.cpp`, `test/start_03_driving_example_test.cpp`,
  `test/start_04_skills_route_example_test.cpp`, `test/start_05_mechanisms_example_test.cpp`,
  `test/start_06_testing_example_test.cpp` — each page's complete example(s) between BEGIN/END region markers (§1.6),
  and the TEST_CASEs `start-NNx: …` that run them (§2). Page 2's checks live in `test/start_02_robot_file_example_test.cpp`
  (the declarations compile and every rule the library checks passes; the cascade homes by stall under its cap on the
  plant; each preset reached; **[S2] and its own region `start-02c` declaring the flipper's other two kinds** — §1.4
  page 2).
- **[F6] Names inside the test binary.** The suite already links `src/team/autons.cpp` (`shulib::bench::registerAutons`,
  `test/CMakeLists.txt` :90) and, since ROB1-A, `src/team/robot_one_autons.cpp` (`shulib::bench::registerRobotOneAutons`,
  *(worktree `rob1-a`)* `test/CMakeLists.txt` :93). Page 1's complete autons file would collide with them, so every start
  example lives in **`namespace shulib::start`** (the practice robot's mechanisms in `shulib::start::practice`), and page
  1's registration function is **`registerPracticeAutons`**. Page 1 says in one line which two names differ in the
  students' real file (`namespace shulib::bench`, `registerRobotOneAutons`, `using namespace robot_one;`) — everything
  else is the same shape. (Quoting `robot_one_autons.cpp` itself was considered and rejected: it is ROB1's placeholder
  file the students fill in, its two entries are pinned by ROB1-A's test, and it would change under the page every time
  the students edit it.)
- `docs/internal/chunks/DOCS3-PROGRESS.md` (first), and at A2's end the between-parts marker (§6.4).

### 1.3 Changed files (A1 unless marked)

- **(amended 2026-11-13)** `docs/README.md` — DOCS3-N1 owns the front page (§A.2); A1 adds ONE row ("Learn to code
  our robot → Start here"). The 11-12 text: `docs/README.md` — "Start here" (:42-46): the track FIRST, then the guide/cookbook/reference; the two stale sentences
  (:26-33, D11). Nothing else.
- **(amended 2026-11-13)** `docs/guide/README.md` — N1 rewrites it as the Learn tab's "How it works" landing; A1 adds
  one link to the track. The 11-12 text: `docs/guide/README.md` — one line above the table (:3-5): "New to shulib and coding your team's robot? Start with the
  **Start here** track"; the maintenance section (:62-93) names `docs/start/` and its stricter rules in one paragraph.
- `docs/guide/08-your-first-routine.md` — one line at the top (D12 (a)).
- `docs/cookbook/README.md` — :11-14 points a first-timer at the track instead of chapter 8. **(amended 2026-11-13)** N1
  rewrites the page ("Recipes: which to read"); A1 only adds the link.
- **(amended 2026-11-13)** `mkdocs.yml` — A1 adds the 7 "Start here" lines into N1's menu, second after Home (§A.3).
  The 11-12 text: `mkdocs.yml` — a "Start here" tab between `Home` and `User guide` (:113-120): `start/README.md` + the six pages. No
  other nav change (the generated API block :151-383 is the tool's).
- `tools/api_doc_tool.py` — §1.6 (globs, the five track rules [F1, F2, F4, F8], the self-test fixtures). **Its banner comments updated in
  the same change.**
- `tools/doc_staleness_audit.py` — `docs/start/*.md` into `LIVE_GLOBS` (:62) and `PATH_GLOBS` (:68), with its self-test.
- `test/CMakeLists.txt` — `docs/start/*.md` **and `start_*_example.hpp`** [F7] into the doc-gate inputs (:213-221 — the
  existing `*example*_test.cpp` glob does not match the practice robot's header, which page 2 quotes), so an edit to a
  track page or to the header re-runs the gate (same reason as the comment there). The comment above the glob updated.
- `test/README.md` — the start example files in its example-file list (:50 area).
- `docs/internal/guide-maintenance.md` — rule 1 (:23-45) gains the track's five rules, inside the one mechanism.
- `docs/changelog.md` — one "Documentation" entry: the track; **no API version change** (nothing under `include/`).
- `docs/internal/REMAINING-WORK.md` (row 19b split into DOCS3-A/-B, the coordinator's plan commit — §3.3),
  `PROJECT-BRIEFING.md` (the doc map), `docs/roadmap.md` (one line).

### 1.4 The pages — what each teaches, its example, what it waits for, and its "coming" form

**(amended 2026-11-13)** The track keeps this frame (not §A.4's task layout, rule L) and obeys rules H, T, C, P, R,
S, V and A from its first commit; each page opens with a picture (the practice robot, the field, the route).
Every page has the same frame: **"By the end of this page you can …"** (one sentence); **the complete program** (one
listing = the whole BEGIN/END region); **walking through it** (the listing again in small pieces, each with its why —
pieces are from the same region and pass the same gate); **what you will see** (the run's log lines and result, as the
test asserts them — quoted in ```` ```text ````, and those lines are ASSERTED in the test, landmine 3 — **[F4] held by
the gate's rule 4**, §1.6); **when it goes
wrong** (the two or three failures a beginner meets, each with the log line that names it); **your turn** (what the
student decides for the team's robot); **read more** (links into guides 5–21 and the API reference). A "The C++ on this
page" box at first use (D6).

**Page 1 — Set up and build** (A1; robot-one lines need ROB1-A merged — it is a precondition of A1's start).
Teaches: getting the repository, the two builds (host suite `cmake`/`suite_run.py`; robot `make ROBOT=one PROGRAM=…`),
uploading with PROS to a slot, what each program is ("shulib Drive", "Calibrate", "Auton Tester", "Competition", "Tune",
"Bench Tests"), and which to run first on a new robot (Calibrate → Tune → Auton Tester). Its example: **a complete
first autons file** (the includes, the namespace, one auton that drives 24 in, `registerPracticeAutons` with a SKILLS
entry on the 57/59 s schedule — the shape of *(worktree `rob1-a`)* `src/team/robot_one_autons.cpp` :1-72; **[F6]** the
two names that differ in the real file are stated in one line, §1.2), run through the Auton Tester's own run
(`test/auton_tester_rig.hpp`'s `TesterStack`) and asserted FINISHED. **[F3]** This works because the auton only DRIVES:
the Tester's rig is robot two's — it builds no Robot (so no wall sensors) and binds robot two's mechanisms
(`auton_tester_rig.hpp` :153-167, :270-310; `auton_wall_reset.hpp` :27-28 says so). The page says so in one sentence
("on the host test plant the Tester runs this on a stand-in robot; your robot's Tester runs it on yours"). **The `make` lines are ```` ```sh ````
and checked by §2's T-sh rule** (each is a build the src build gate knows). Slot numbers: link guide 7 / chapter 21's
table, never restate (they moved before — `Makefile` :118-166 history).

**Page 2 — Your robot file** (A1, after ROB1-A). Teaches: what a robot file is; the five kinds in one sentence each (link
guide 18 :33); the practice robot's **cascade** line by line — a Lift, its motors facing each other, its gearing, its
**measured rows** (why a string's height per turn changes as it winds — HA-254 by link), `.currentCap` (why a string is
pulled gently), the stall signature, `homeByStall` (lower gently into the bottom stop at boot), the presets as stack
levels; **the flipper's three kinds** (motor arm, pneumatic, passive bar — the practice robot declares the arm; **[S2]
the other two are declared in page 2's own region `start-02c`** (compiled, each checked against the library's rules by
its TEST_CASE, every line commented) — not quoted from ROB1-A's `test/robot_one_flipper_forms_example_test.cpp`: rule 2
(§1.6) allows a track listing only as a run of a `start-NNx` region, and that file is chapter 21's to change; page 2
LINKS it as robot one's version); the side sensors' block and the camera's (link, A2). Then **"Robot one's real file"**: where the same lines live
(`src/robots/robot_one_facts.hpp`, robot one's region of `src/robots/mechanism_table.hpp`), **OWED facts** — the build
refuses, naming the fact, until it is typed — and the facts form (`docs/robot-facts-form.md`), all by LINK to chapter 21
(no second copy of ROB1's text; landmine 5). Its tests: the declarations compile (every library rule), the cascade homes
by stalling into the plant's bottom stop under its cap and reaches every preset; a mutant (the homing push's sign flipped)
turns it red.

**Page 3 — Driving and turning** (A1; all merged). Teaches: the field frame (x ahead, y left, angles counterclockwise —
`src/team/autons.cpp` :10-13), `moveTo`/`driveTo`/`turnTo`/`face`/`reverseTo`, timeouts ("never hang"), per-step
tolerances (`.positionTolerance`, `.headingTolerance` — what "close enough" costs and buys), **why a tank cannot slide
sideways and what to do** (face, then drive; TANK1's steering back onto the line; a curve — page 4), and **"never
stuck"**: every step ends — its own timeout, the watchdog, the stall check, the 60 s clock's end action and hard stop —
each in one paragraph with the log word it prints. Its example: a P1-like auton (face → drive → turn → a tolerance → a
`carryOn` leg) run through `runAuton` with its true pose asserted; a second short example shows a step that times out
and the routine's honest STOPPED line (asserted). Mutant: the turn's sign (P1's).

**Page 4 — Writing a skills route, step by step** (the centrepiece). One practice route, **six stages, each a complete
program** (its own region and its own TEST_CASE), on a SKILLS entry (60 s; end action 57 s, hard stop 59 s):
1. **The start** — a FIELD start pose (why FIELD, not RELATIVE: the wall reset and the Toggle need field coordinates —
   `auton_wall_reset.hpp` :20-24), two legs, the result. *(merged)*
2. **A wall reset** — `r.then([&] { return competition::resetFromWall(ctx, {…}).continues(); }, "left wall x")` after a
   leg along a wall; what it fixes (one axis), when it refuses and why the route goes on. The rig gives the robot a drift
   so the reset is LOAD-BEARING: the test asserts the estimate's error after the reset is small AND that without it the
   later leg ends measurably off (landmine 2). *(merged)*
3. **When a step fails** — `onFail(carryOn)`, `onFail(skipTo("…"))`, `label("…")`; the rig makes one step fail on
   purpose (a sensorless grab, a blocked leg) and the test asserts the skip. *(merged)*
4. **The cascade on the way** — `r.atPercent(60).startMove(cascade["level 2"], …).driveTo(…)` then `waitUntilDone`
   (STEP1-B's spellings EXACTLY as merged — landmine 1); not on a curve (*(staged)* `routine.hpp` :62: triggers do not
   cover curves at first). **Waits on STEP1-B's merge** (a precondition of A1's start, so it is IN A1 unless STEP1-B slips —
   then "coming", below).
5. **A curve** — `curveTo` with the **one-line switch** (`kUseCurves`, OFF until the curve passes its robot session —
   *(staged)* `test/cookbook_examples_test.cpp` :261-296) and the **run-up rule** in plain words (a 6-in sideways miss
   needs ~2½ ft of run-up — *(staged)* `docs/guide/05-getting-there.md` :189-205, LINKED, not restated as a law of this
   robot — landmine 4). The test runs the stage BOTH ways (switch on and off) and asserts both end at the goal.
   **Waits on CURVE1-A's commit** (staged now).
6. **The Toggle** — first the **blind two-push flip** a student can write today: drive square to the Toggle, push and
   retract the flipper twice (`move(flipper["flip"], …)`, `move(flipper["back"], …)`), back out. Its test asserts the
   flipper's strokes (and, once AF1S-A1's `sim::TogglePlant` is on main, the host Toggle's colour after the pushes).
   Then **the camera-checked step** — `checkAndFlipToggle` — **ONLY in its everyday form over `ctx` (AF1S-A2), in A2.**
   AF1S-A1's explicit form needs camera, guard and service handles an auton does not have (*(worktree `af1s-a1`)*
   `toggle_visit.hpp` :14-16, AF1S brief :484 "No auton can reach the camera"), so it is NOT taught to beginners.
Then **reading the result**: the run's `RTN`/`MECH`/`[WALL]`/`TGL` lines and the result line, as the stage-6 test asserts
them, and what each failure word means. Every stage ends with **"your turn"** questions (D7).
**[F13] Its length, honestly: ~700–900 lines** (six complete programs growing from ~25 to ~90 code lines, each shown
whole once; each stage's walkthrough shows ONLY the lines that stage adds, as pieces of that stage's region — so no line
is explained six times; output and "when it goes wrong" per stage). The prose review reads each stage's NEW lines; the
lines carried over from the previous stage are byte-identical (rule 2 holds both to their regions), so their review is
not repeated. **Before A1 merges, the team lead reads page 4's practice route and confirms in writing that it is not a
route the team would run** (§2.3) — the guard against writing the team's route (landmine 7) gets a named reviewer.

**Page 5 — Mechanisms and the cascade, on the controller and in autons** (A1; all merged). Teaches: presets and holds;
homing at boot and on request; the current cap; the driver file's controls (a button per level or up/down — the shape of
`src/team/robot_two_driver.hpp`; robot one's default from ROB1-A by link); **tuning on the robot with "shulib Tune"** —
the pages, RESET/COPY/EXIT, and **pasting the COPY block back** into the robot file (guide 18 :491-600). Its examples: (a)
the cascade in an auton (home, move to a level, hold while driving, a stall that stops it) through `runAuton`, asserted;
(b) the controls over a fake controller (the cookbook 06 pattern), asserted; (c) **[F10] restated:** a tuning session
(through `mechanism::LiveTuning`, as `test/mechanism_live_tuning_example_test.cpp` :33 does) changes two numbers and
prints its COPY block; the page shows that block as ```` ```text ```` (its lines are `.tuning.kP = …`-style lines and
begin/end lines — guide 18 :494-501 — NOT declaration lines, so they are output, held by rule 4), then shows, as a
checked ```` ```cpp ```` region, the cascade's declaration REWRITTEN BY HAND from those numbers (the student's paste,
`.tuning = {.kP = …, …}`); the test asserts that the rewritten declaration builds to the SAME derived spec as the one
that ran tuned (the COPY promise as guide 18 states it: "the SMALLEST set that builds to exactly what ran"; compared on
every tuned member at least — UNVERIFIED whether TUNE1-A's tests already have a whole-spec comparison to reuse; the
builder looks first in `test/mechanism_live_tuning*_test.cpp`). Mutant: a
control's button changed → red.

**Page 6 — Testing before the robot is on the field** (A1; the preview section A2). Teaches: the Auton Tester (pick,
RUN, abort, read the result against the tape — guide 16 by link for detail), the competition program's screen and phases
(guide 17 :178), the SD card and the serial log (guide 11 by link), a checklist before a skills run, and **the route
preview** once RPV1 merges. **[F3] Its examples, changed at the re-check:** (a) page 4's full route registered as a
SKILLS entry and run through `competition::runAuton` on the PRACTICE robot's rig (walls, cascade, flipper — the only rig
that has them), the result word and the per-step lines asserted; (b) the Auton Tester's own run (pick, RUN, abort,
result) shown with a DRIVE-ONLY practice auton on the existing `TesterStack` (robot two's stand-in; it builds no Robot,
so it has no walls and none of the practice robot's mechanisms — `auton_tester_rig.hpp` :270-310), the page saying so in
one sentence. A practice-robot Tester chain (walls + the practice mechanisms + the abort watch) is NOT built: it is
unprobed and would add ~½–1 day; RPV1-B1 generalises `TesterStack` to any robot's mechanisms (RPV1 brief :356) but still
builds no Robot, so the wall reset would still refuse there. Mutant: one step's bound too short → STOPPED, red.

**THE "COMING" FORM (any section whose feature is not merged on the run's base).** A box: "**Coming:** <feature> is being
built and is not in the library yet. Until it lands, see <the current reference: guide chapter / cookbook recipe on
main>." — **no code listing, no spelling of the unmerged API, no guessed signature.** **[F2] No backticks at all inside
the box** — an unmerged name in inline code (`` `atPercent` ``) is a guessed API too; the gate refuses a "Coming:" box
holding a backtick (§1.6 rule 5). The A2 run (or the coordinator at a merge) replaces it. §2's T-coming check holds this.
A page A1's cut line leaves unfinished (D9) is a GUIDE-POST: the page's title, "By the end of this page you can …", a
"Coming:" box, and links into the guide — no listing.

### 1.5 The practice robot and its host rig (`test/start_practice_robot_example.hpp`, `test/start_rig.hpp`)

**The robot.** A tank (12-in track, as every tank rig's `TankKinematics{Length{12.0}}` — `test/mechanism_auton_rig.hpp`
:146), on robot two's drive-port shape over the fake provider (`test/wall_robot_rig.hpp` :45-50); side sensors left and
right (`wall_robot_rig.hpp`'s walls block); a **cascade** `Lift`: two motors facing each other, `measured(…)` rows,
`.currentCap`, a stall signature below the cap, `homeByStall` into the bottom stop, presets named for stack levels; a
**flipper** `Arm` (stow/flip/back); the camera block in A2. Every number round and commented "made up for the practice
robot — yours comes from …". **Never robot one's placeholders, never robot two's numbers presented as the cascade's.**

**The rig** composes parts that exist and are tested separately — it invents no plant:
- the Robot with walls and `runAuton` over a run guard: `test/wall_robot_rig.hpp` :1-20, :197-199;
- a mechanism service bound over a declared list with plants: `test/mechanism_auton_rig.hpp` :144-200 (`RobotX`: two
  `ArmPlant`s, the service, the guard in the chain);
- a lift plant with hard stops for stall homing: `include/shulib/sim/mechanism_plant.hpp` (`ArmPlantConfig`
  `minStopDeg`/`maxStopDeg`), stall homing under a cap already tested (`test/mechanism_current_cap_test.cpp`);
- a drift for the wall-reset stage: a weak side (`test/auton_tester_rig.hpp` :81 `WeakSide`, robot two's ~9 %);
- in A2, the camera and Toggle plants: *(worktree `af1s-a1`)* `test/toggle_visit_rig.hpp` (once merged).
**Step 1 of A1 is a probe of this composition** (one auton through `runAuton` on it: a FIELD start, a wall reset that
seats, the cascade homed by stall and moved to a level). **[F3]** This rig runs every example that needs walls or the
practice mechanisms, page 6's full route included; the Auton Tester's own run is shown only with a drive-only auton on
the existing `TesterStack` (§1.4 pages 1 and 6) — no third rig. If the parts do not compose in **~¾ day**, STOP and tell the
coordinator — the fallback (each page-4 stage on the rig its feature already has; the wall-reset stage on
`wall_robot_rig`, the cascade stages on a `RobotX`-like rig) breaks "one robot everywhere" and is the team lead's call.

### 1.6 The gate — five track rules inside `check-examples`, and the folder (A1, FIRST) — **[F1, F2, F4, F7, F8] re-written at the re-check**

In `tools/api_doc_tool.py`, each with a self-test fixture that FAILS when the rule is removed. **All five are rules inside
the one `check-examples` scan** (one mechanism — landmine 9), stricter for `docs/start/`:
1. **The folder.** `docs/start/*.md` into `PUBLIC_DOC_GLOBS` (:1916) — and therefore into `check-removability` (which
   reads the same list, :2008-2017). The same folder into `doc_staleness_audit.py` (:62, :68) and the build's inputs
   (`test/CMakeLists.txt` :213-221, with `start_*_example.hpp` — [F7]).
2. **Whole and complete.** New kinds: `test/start_*_example_test.cpp` and `test/start_*_example.hpp` into
   `EXAMPLE_SOURCE_GLOBS` (:1947) only (the first overlaps the existing `test/*example*_test.cpp`, so de-duplicate the
   `sources` list). **[F7] NOT into `WHOLE_RUN_SOURCE_GLOBS`** (left exactly as it is): there it would make an unrelated
   guide or cookbook listing whose lines all happen to occur in a start file (`return r.result();`) fail unless they
   also occur there in the same order (:2076-2081) — and the region rule below is stronger for the track anyway.
   **The globs and the first start files land in ONE change** — the scan fails when a glob matches no file (:2042-2047).
   The **complete-region rule**, for `docs/start/` only: each source region `// ═══ start-NNx BEGIN — runs: <name>` …
   `// ═══ start-NNx END` (marker lines outside the region, never quoted) must appear on its page (`docs/start/NN-*.md`)
   as at least one listing EQUAL to the whole region (every non-blank line, in order); every listing in `docs/start/`
   must be a contiguous run of ONE region (pieces in the walkthrough allowed, mixing regions not, a line from outside
   every region not). **[F8] Tied to a RUN, not a name:** `<name>` on the BEGIN marker must be defined inside the region
   (the auton function, or for the header's regions the declared object, e.g. `cascade`), and the body of a `TEST_CASE`
   named `"start-NNx: …"` in a start example file must use that name — so a region whose function no test calls fails
   the gate. Habit on top, not instead: a region's auton functions sit in an anonymous namespace, so the suite's
   `-Wall -Werror` refuses one nothing calls.
3. **(amended 2026-11-13: per D19 — if (a), this rule becomes: every line of a region appears in at least one
   walkthrough piece on its page, and no comment in a track listing is longer than one line / 70 characters; if (b),
   as written)** **Fully commented** (D8), for `docs/start/` listings only — **[F1] the strict rule**: every code line covered by its
   OWN trailing `//` comment of ≥ 3 words, or by a comment-only line of ≥ 3 words directly above it. **No continuation
   exemption** (a line that continues a commented line needs its own comment — re-check probe R1). Punctuation-only lines
   exempt (`}`, `};`, `});`); a `//` inside a string literal does not count. Because rule 2 makes every listing a run of
   its region, the rule holds the SOURCE region too.
4. **[F4] Output is checked, not copied by habit.** Every non-blank line of a ```` ```text ```` block on
   `docs/start/NN-*.md` must appear inside a string literal in the body of a `start-NN…` TEST_CASE of that page's start
   test file(s) (plain and raw literals, escapes decoded). A part of a line that varies (a time) is written `…` on the
   page; each `…`-separated fragment of ≥ 4 characters must then appear. Re-check probe R2 measured how far today's
   habit is from this: guide 18's text lines 17 of 18 NOT in any literal of its tests, guide 09's 2 of 2 — so this is
   a new discipline, from the track's first page. What the gate cannot see — that the literal is used in an ASSERTION,
   not merely present — is a line in the prose review's log for each text block.
5. **[F2] No code block escapes.** In `docs/start/`, every fenced block's info string must be exactly `cpp`, `text` or
   `sh` (a fence with a title, a space, `c++`, `~~~`, or no info string is refused, naming the line) — the scan's regex
   (:2068) only sees blocks that begin exactly ```` ```cpp ```` + newline, and the checker's probe proved a titled block
   and a ```` ``` cpp ```` block holding lines found in no source pass today (`checker/t.py`: "0 quoted lines … exit 0").
   For EVERY public document (not only the track) a fence whose info string starts `cpp`/`c++` but is not exactly `cpp`
   is refused — zero such fences exist in public docs today (measured, `git grep` at `c0a7c7e`), so this closes the hole
   everywhere at no cost. And a "**Coming:**" paragraph or box in `docs/start/` holding a backtick is refused (§1.4).
Each failure message says what to fix and where (the house style of :2093-2116: "fix the SOURCE first, then re-quote").

---

## 2. Tests, mutations, and the definition of done

### 2.1 Tests (each tries to break ONE promise; graded on the host test plant's ground truth, never the estimate alone)

| # | Test | Bug it catches |
|---|---|---|
| T1 | each `start-NNx` TEST_CASE runs its region's auton through `competition::runAuton` on the practice robot's rig (**[F3]** or, for page 1's auton and page 6's Tester section only — DRIVE-ONLY autons — the Tester's run on the existing `TesterStack`) and asserts: the run outcome, `routine.ok` (or the exact stop the page says), `steps` and `completed`, the plant's TRUE end pose within the page's stated tolerance, each mechanism step's verdict word from its `MECH` line | an example that compiles but does not do what the page says (P1's 2 s lift) |
| T2 | every log line or result line a page quotes in ```` ```text ```` is asserted by substring in its TEST_CASE (the guide's `countOf` style — `guide_examples_test.cpp` :79-86); **[F4] its presence in the TEST_CASE is the gate's rule 4**, its use in an assertion the logged review | a page showing output the code no longer prints |
| T3 | page 4 stage 2: estimate error after the reset < the page's number AND the same route without the reset ends ≥ the page's "off by" number (two runs, same rig, same drift) | a wall reset that does nothing, taught as if it mattered |
| T4 | page 4 stage 5: the curve stage with `kUseCurves` true AND false, both end at the goal; the false run uses only straight legs (no curve line in the log) | the "one-line switch back" that does not switch |
| T5 | page 2: the practice robot's list passes every library rule (it compiles), the cascade HOMES by stall into the bottom stop with its cap applied (read back, as `mechanism_chapter_example_test.cpp` :139 `guide-18g` does) and reaches each preset | a declaration that compiles but cannot home |
| T6 | page 5: controls over a fake controller (each button → its preset; release → hold); **[F10]** a second declaration in the region, written by hand from the COPY block's numbers, builds to the same spec as the one that ran tuned (§1.4 page 5 (c)); the COPY lines themselves are output (```` ```text ````, rule 4) | a control, a COPY or a paste-back that drifted from the page |
| T-sh | every `make …` line in a ```` ```sh ```` block of `docs/start/` names a `ROBOT`/`PROGRAM` pair in `tools/src_build_gate.py`'s build list (a check in the same scan, or a small test reading both) | a page telling a student to run a build that the Makefile refuses |
| T-coming | no `docs/start/` listing contains an identifier that is not declared on main (by construction: rule 2 binds every line to a compiled region; **[F2] rule 5 makes "every listing" mean every fenced block — no titled or odd fence escapes — and refuses backticks in a "Coming:" box**; a "coming" box has no listing) — plus a check in §7 that every "Coming:" box links a file that exists. Left to a person: an unmerged name written as plain prose words (no backticks) — the §7 hand check reads every box | a guessed API on a page |
| TG1–TG11 | the gate's self-tests: a docs/start listing line edited → fails; a listing skipping a region line → fails; a region with no full listing → fails; a region whose `runs:` name no `start-NNx` TEST_CASE body uses → fails **[F8]**; a covered line's comment removed → fails; **TG5b [F1]: a multi-line declaration with ONE comment → fails**; `docs/start/*.md` removed from `PUBLIC_DOC_GLOBS` → the self-test fails (the P3 hole cannot reopen); **TG7 [F2]: a ```` ```cpp title="x.cpp" ```` block, a ```` ``` cpp ```` block and a `~~~cpp` block, each holding a line in no source → fail**; **TG8 [F2]: a backtick inside a "Coming:" box → fails**; **TG9 [F4]: a text-block line edited so it is in no literal of the page's test → fails**; **TG10 [F7]: an existing guide listing whose lines also occur, out of order, in a start file → still PASSES** (the whole-run rule was not widened); TG11: a listing mixing two regions → fails | each rule silently not running, or a rule biting where it should not |

### 2.2 Mutations (at least one per page; each must turn `python3 tools/suite_run.py green` or the doc gate RED; recorded in `muts_docs3_a1.json`)

| # | Mutant (in the example SOURCE, re-quoted where needed) | Red because |
|---|---|---|
| MS1 | page 1: the entry's kind `Skills` → `Match` | the 57/59 s schedule is refused for a 30 s period (registration names it) / the Tester's menu assertion |
| MS2 | page 2: the cascade's homing push sign flipped | T5: never reaches the bottom stop |
| MS3 | page 3: the turn's sign (P1's mutant) | T1: end pose, `ok`, `completed` |
| MS4a–f | page 4, one per stage: (a) the FIELD start → RELATIVE; (b) the wall reset line removed; (c) `skipTo("x")` → a label that is not written; (d) the trigger's percent 60 → 100 (the cascade starts at the end); (e) `kUseCurves` true with the curve's end pose moved; (f) one flipper push removed | (a) the wall reset refuses `not a field start` (T1/T3); (b) T3; (c) the precondition names it; (d) the asserted cascade height at the drive's 70 % mark; (e) T4; (f) the asserted stroke count |
| MS5 | page 5: a control's button changed | T6 |
| MS6 | page 6: one step's bound halved | the result word STOPPED, not FINISHED |
| MG1–MG4 | the gate: a quoted line edited on a page; a quoted line deleted from a listing; a comment deleted from a region line; `docs/start/*.md` dropped from the globs | check-examples (rule 2 / rule 3) / the self-test |
| MG5–MG8 **[F1, F2, F4, F8]** | the comment on a continuation line of page 2's cascade deleted (the line above keeps its own); page 4 stage 1's fence changed to ```` ```cpp title="stage1.cpp" ````; one word of a page-3 ```` ```text ```` line changed; the `runs:` name's call removed from its TEST_CASE (the call moved to a helper the test does not run) | rule 3 / rule 5 / rule 4 / rule 2's tie |

### 2.3 Definition of done (each run)

**(amended 2026-11-13; re-checked 2026-11-15 late)** Also, for every docs run of this brief (N1, N2, P1a, P1b-1/2,
P1c-1/2, A1, A2, phase 3): `python3 tools/doc_readability.py self-test`, `check-nav`, `check-tone` (from N1),
`check-readability` (from N2); `check-figures` once P1a lands; the strict site build with the anchor check — `python3
tools/site_check.py --required` once N2 lands, before that §7's scratch-venv build, **never with `-q`** ([S4]); every
page the run converted added to `CONVERTED` with its TYPE ([K5]); every removed anchor kept — an invisible `<a id>` on
the same page, or, after a split, a visible "Moved to" pointer line on the OLD page plus the reader pages' links
re-pointed, listed in the log ([K4]); the run's pages read once at ~400 px and once at 1280 px, screenshots in the log
(Playwright's own Firefox under the cap, as RMP1-A1 does — [K15a]); the run's own heading in `DOCS3-PROGRESS.md`, and
nothing written under another run's ([K14]).

The suite green through `tools/suite_run.py` (count recorded); every `start-NNx` case passes and is listed in the
progress log with what it asserts; the doc gates green (self-test, coverage, fresh, examples with the five track rules,
removability, briefing, staleness); `mkdocs build --strict` on a `prepare_site.py` copy (the publish path —
`.github/workflows/pages.yml`; the exact commands in §7 [F11]) with the new tab, no warnings; every mutation red and
restored; the prose review (§3.4) logged, **including, for every ```` ```text ```` block, the assertion that uses its
literal [F4]**; `git diff --stat <base> -- src/ include/` EMPTY; every page read top to bottom once by a reviewer playing
a first-year student, findings logged and fixed; **[F13] the team lead has read page 4's practice route and said it is
not a route the team would run** (the coordinator asks him with the page's built copy before the merge; his answer
logged); **[F5] the cut line's outcome logged** (which pages are full, which are guide-posts). A2 the same for what it
touches.

---

## 3. Documents — in the same change (pace rule 3), the prose review logged (rule 4)

### 3.1 Public
A1: `docs/start/` (7 pages), `docs/README.md`, `docs/guide/README.md`, `docs/guide/08-…` (one line),
`docs/cookbook/README.md`, `mkdocs.yml`, `docs/changelog.md`, `test/README.md`. A2: the pages it completes, the changelog.
(Pace rule 7's "≤ ~5 public" is exceeded by A1 by its nature — the track is the deliverable; the non-track edits are
one to three lines each.)

### 3.2 Internal, every run
`DOCS3-PROGRESS.md` (live; every probe, test, mutation and the prose review); `guide-maintenance.md` (rule 1);
`PROJECT-BRIEFING.md`'s doc map; at A2's end the between-parts marker (§6.4).

### 3.3 The plan commit (the coordinator's, on the team lead's go, before A1)
`REMAINING-WORK.md` row 19b → DOCS3-A (before the 21st, this brief) / DOCS3-B (after); `build-order.md`'s queue; this
brief copied to `docs/internal/chunks/DOCS3-beginner-track.md` with the answers.

### 3.4 The prose review's greps (each run runs them over `docs/start/` and logs every hit it read)

**(amended 2026-11-13)** The long chunk ids, HA numbers and dates are now machine-held (rule H, §A.4) and `honest`
by rule T; the review still greps the SHORT ids (`\b[ACDEFRT][0-9]\b`, which the machine cannot tell from buttons and
form fields) and every other grep below, on every page a run converts — not only `docs/start/`.
`simulat` (must be zero — "host test plant"); **[F9]** `never driven|has run on|ran on|drove|was tested on` (each a
true, dated statement — narrowed from `on the robot`, which the ruling's own words "tuning on the robot" would hit);
**[F9]** `\b(STEP1|CURVE1|AF1S|L6|ROB1|TUNE1|RPV1|MECH-[A-Z]|VIS1)-?[A-Z0-9]*\b` (chunk ids — zero on the track: a
beginner does not know them; `MECH-[A-Z]`, not bare `MECH`, because `MECH` is the mechanism LOG TAG the pages quote —
`include/shulib/mechanism/service.hpp` :448); `\bTier\b|facade|scheduler|provider|seam` (each defined in the sentence or removed); `robot one'?s .* (is|are) [0-9]`
(no robot-one number stated as fact — landmine 4); `G4|LLM|AI-written` (zero — §8 landmine 7).

---

## 4. Hardware Assumptions — NONE

DOCS3-A states no new belief about hardware: every result is the host test plant's and says so; every number about a
real robot is LINKED to the register or the chapter that owns it (the run-up rule's distances, the cap's range, HA-254's
string winding), never restated as a property of the team's robot. **A belief a page needs that no register entry
covers ⇒ STOP and tell the coordinator** (it gets a number at the plan commit, not in the run).

---

## 5. OUT, and explicitly rejected

**(amended 2026-11-13)** DOCS3-B's front door, navigation and page rewrites are now IN this brief (§A.2–§A.7: runs N1,
N2, P1a–c, phase 3). Still OUT: any edit under `src/` or `include/` (so the 79 "honest" in generated API pages and the
Bench Tests painter wait for a code pass after the 21st), renaming files (D13 (b)), and the robot panel's pages.

- **DOCS3-B** (after the 21st): one front door per reader; reorganising guides 1–21, the cookbook and the API reference's
  layout; re-checking every claim; the "never driven" sweep (`REMAINING-WORK.md` :179); rewriting chapter 8; applying the
  "fully commented" rule outside `docs/start/`.
- **Any library or robot code** (`src/`, `include/`) — a library defect an example exposes is REPORTED, never fixed or
  worked around in the example (landmine 8).
- **The team's real skills route, driver choices or robot one's numbers** — rejected (D7 (b); landmine 7).
- **AF1S-A1's explicit Toggle form as a beginner listing** — rejected (§1.4 page 4).
- **The panel, ALLY1 (alliance colour), L4 (drive to a piece), match autons** — out of this competition's scope.
- **A C++ primer page** (D6 (b)); **guide chapters for the track** (D1 (b)).

---

## 6. Coordination — every in-flight run (re-checked 2026-11-12 ~23:30; every worktree read only)

**(amended 2026-11-13)** The 11-12 picture below is history: everything in §6.1 has merged except AF1S-A2 (running)
and MIRROR1-A (staged). The state on 11-15 and the order of the docs runs: §A.7; READ1: §A.9; the shared files of
§6.2 now include `tools/doc_readability.py` (N1, then N2) and `tools/docs_figures/` (P1a first, then P1b/P1c add
scripts).

### 6.1 In flight
- **CURVE1-A** — squash STAGED on main (gates running); API 2.7. A1 bases on its commit. Touches `mkdocs.yml` (API nav
  +2), guide README (:27), cookbook 01/README, guide 05/09/10/11/14/15 — A1 edits the guide README and cookbook README
  AFTER it (no overlap in lines).
- **ROB1-A** — DONE; merge next. Touches `Makefile`, `mkdocs.yml` (chapter 21), `docs/guide/README.md`, guide 07/16/17,
  `test/CMakeLists.txt`. **A precondition of A1's start** (page 1's `ROBOT=one`, page 2's robot one). Robot one's Tune
  (`MECHANISM_ROBOTS`) is added at its merge (NOTES 11-11) — page 1/5 name it only if on main, else A2.
- **STEP1-B** — fix round running; API 2.8 at merge. Touches guide 09/10/11/14/18, cookbook 07/README, `mkdocs.yml`.
  **A precondition of A1's start** (page 4 stage 4); if it slips past ~Nov 15, A1 starts without it and stage 4 is a
  "coming" box filled by A2.
- **VIS1-A2** — fix round running; merge renames `19-the-camera.md` (landmine 6). Page 4/2 link the camera chapter only
  after it.
- **AF1S-A1** — DONE; merges after VIS1-A2 (its diff on VIS1-A2's code). Brings `sim/toggle_plant.hpp` (T1's Toggle colour
  for the blind flip) and `cookbook-08a` (explicit form; NOT quoted by the track).
- **Queued:** RPV1-A0+B1 (approved), ROB1-C, AF1S-A2 — ahead of DOCS3-A in the team lead's 11-12 order (NOTES :984),
  and with MAX 3 JOBS they take all three slots from ~Nov 13; **[F5]** A1 takes ROB1-C's slot when it frees (~Nov 14
  earliest — D9's arithmetic). A2 waits for AF1S-A2 and RPV1.
- **Release #1** (after VIS1-A2/AF1S-A1 + the release-check fix) carries no track; **Release #2 at the freeze** carries
  A1 only if merged by then — **[F5] more likely A1 reaches the site through D10's docs-only publish, ~Nov 19–20.**

### 6.2 Shared files — who writes, who waits
`mkdocs.yml`, `docs/guide/README.md`, `docs/cookbook/README.md`, `test/README.md`, `test/CMakeLists.txt`,
`docs/changelog.md`: every in-flight run above touches some; **A1 rebases at its merge onto whatever merged during its
build** (the coordinator's usual serial merge). `tools/api_doc_tool.py`: only A1 (the coordinator's release-check fix is
`tools/release.py`). The generated API nav block in `mkdocs.yml` (:151-383) is never hand-edited.

### 6.3 An API that changes at its merge
A1 quotes ONLY what is on its base. If a run merges during A1's build and renames a spelling A1 quotes (STEP1-B's trigger
words; CURVE1's `curveTo`), the merge fails A1's gate (rule 2: the line no longer compiles in the region → the region's
test fails to build) — it cannot publish a stale spelling; the coordinator re-runs A1's affected stage at the merge.

### 6.4 The release preflight
`tools/release.py` :93-108 FAILS a release on any `*-PROGRESS.md` without its `*-COMPLETED.md`. DOCS3 stays open until
DOCS3-B, so `DOCS3-PROGRESS.md` must carry the coordinator's planned **between-parts marker** (NOTES 11-11: the
release-check fix, before Release #1). **If that fix has not landed when A1 merges, the freeze release FAILS** — the
coordinator lands the fix first (it is already ordered before Release #1).

---

## 7. Checks — the coordinator re-runs every one after each run (never the report's word)

**(amended 2026-11-13; re-checked 2026-11-15 late)** Add, after the doc gates: `python3 tools/doc_readability.py
self-test && python3 tools/doc_readability.py check-nav && python3 tools/doc_readability.py check-tone` (from N1) and
`check-readability` (from N2); once P1a lands `python3 tools/docs_figures/screens.py check` and `check-figures`; and,
once N2 lands, `python3 tools/site_check.py --required` in place of the hand-made venv steps below (the full pin set,
the route builder when `docs/route-builder.md` exists, the same strict build plus the anchor check). Until N2, the
hand-made build below — with `validation: links: anchors: warn` in the copied `mkdocs.yml`, without `-q`, and, once
RMP1-A1 has merged, RMP1-A1's three route-builder commands (§A.8) run into the scratch `site_src/route-builder/`
first.

```sh
cd /home/gonzei/projects/shulib
git diff --stat <base> -- src/ include/                      # MUST print nothing: no robot code
cmake --build build/test -j4                                 # builds the suite AND runs the doc gates (stamp)
python3 tools/suite_run.py green                             # bounded suite run; count recorded
(ulimit -v 4194304; timeout 300 ./build/test/shulib_tests -tc='start-*')   # the track's cases, each listed
python3 tools/api_doc_tool.py self-test && python3 tools/api_doc_tool.py check-examples
python3 tools/api_doc_tool.py check-removability && python3 tools/doc_staleness_audit.py self-test
# [F11] the publish path, as .github/workflows/pages.yml runs it (prepare_site.py site_src, then mkdocs build --strict
# beside mkdocs.yml, whose docs_dir is site_src) — in a scratch folder, never the repo root:
S=/home/gonzei/projects/shulib/.claude/coordinator/scratch/docs3a-site; rm -rf "$S"; mkdir -p "$S"
cp mkdocs.yml "$S/mkdocs.yml"
# mkdocs is NOT installed on this laptop (checked 11-12): first, once, a scratch venv with CI's pins —
#   python3 -m venv "$S/../mkdocs-venv" && "$S/../mkdocs-venv/bin/pip" install mkdocs==1.6.1 mkdocs-material==9.5.44
#   and run the build below with "$S/../mkdocs-venv/bin/mkdocs".
(ulimit -v 4194304; timeout 300 python3 tools/prepare_site.py "$S/site_src")
(ulimit -v 4194304; timeout 300 mkdocs build --strict -f "$S/mkdocs.yml" -d "$S/site")   # no warnings
grep -rn 'simulat' docs/start/ ; grep -rnE 'G4|LLM' docs/start/ test/start_*   # both empty
grep -rn 'Coming:' docs/start/                               # each box's link opened by hand; listed in the log
# the mutations: muts_docs3_a1.json through .claude/coordinator/mutate.py (MS1–MS6, MS4a–f, MG1–MG8), each red, restored
```

---

## 8. Landmines

1. **A guessed API.** Writing a page from a brief, a chapter or memory instead of from main's header. Every listing is
   copied FROM a compiled region, never typed into the page (rule 2 enforces it); an unmerged feature gets a "coming"
   box with no code (§1.4). STEP1-B's trigger words and AF1S-A2's everyday form are the two most likely to be guessed.
2. **An example that only compiles.** P1's first draft compiled cleanly and FAILED on the plant (`'lift': TIMED_OUT in
   2.01s`). Every region has a TEST_CASE that RUNS it and asserts the page's claims (T1, rule 2's TEST_CASE check). A
   feature that does nothing on the rig (a wall reset with no drift to fix) is a hidden version of this: T3.
3. **Output the code no longer prints.** Every quoted log line is asserted (T2); output is copied from the test's run
   (`SHULIB_GUIDE_PRINT`-style), never written by hand. **[F4]** The gate's rule 4 holds every text line to a literal in
   its page's test; today's chapters do not keep this habit (probe R2: guide 18, 17 of 18 lines in no literal), so the
   track cannot lean on habit either.
4. **Robot-specific numbers presented as truths.** The practice robot's numbers are made up and each says so; host-plant
   times are "on the host test plant" (RPV1 brief :34-38: plant times are neither a floor nor a ceiling); the run-up
   rule, the cap's range, the string's winding are LINKED to their owners. Never "robot one's cascade reaches level 2 in
   1.2 s".
5. **Two copies of robot one's text.** Chapter 21 (ROB1-A) owns the OWED scheme and robot one's file; page 2 links it. A
   second explanation would disagree with it within a week of facts being typed.
6. **The camera chapter's name.** Two chapter-19 files exist across worktrees; the merged name is decided at VIS1-A2's
   merge. Link it only from main; `mkdocs build --strict` and the staleness audit's path check catch a wrong one.
7. **The line between teaching and writing the team's route.** The track teaches HOW (each feature, on a practice route
   whose choices are made for teaching); the students decide WHERE, WHAT and in WHICH ORDER. No page shows a plausible
   real skills plan; nothing is pasted into `src/team/robot_one_autons.cpp` (ROB1-A's test pins its two entries). The
   public pages and example comments teach in plain words ("your route is yours to write") and carry no rule numbers or
   authorship notes (the team lead's standing preference, 2026-06-19 — §3.4's last grep).
8. **A library defect found while writing an example.** It will happen (examples exercise combinations no test has).
   Never bend the example around it, never fix the library in a docs run (the freeze): STOP, log the smallest failing
   example in the progress log, tell the coordinator; the page ships the stage as "coming" if it cannot be fixed in time.
9. **A third coverage mechanism.** `guide-maintenance.md` :41-45. The track rules live inside `check-examples`, applied
   to `docs/start/` — one scan, stricter there.
10. **The P3 hole reopening.** A future edit drops `docs/start/` from a glob and every page goes unchecked silently. TG6
    pins it in the self-test.
11. **[F1] Multi-line statements and the comment rule.** The first draft said chained calls DEFEAT the rule; under its
    continuation clause they actually PASSED it with one comment (the checker's probe), and so did a whole multi-line
    declaration. The clause is gone (§1.6 rule 3): every line of a chain or a declaration needs its own comment. The
    track still writes one step per line, each ending `;` with its comment (chapter 9's 47 uncovered lines are mostly
    chains) — it reads better, and a chain of five steps would need five comments anyway.
12. **Overlong comments.** "Every line commented" in a 110-column file pushes comments off the screen on a phone. Put the
    why on the line ABOVE when it does not fit (the rule allows it); keep listings ≤ ~100 columns.
13. **The freeze.** A1 changes no robot code, but its merge rides the same release protocol: the release check, the src
    build gate's package comparison, and §7's empty `src/ include/` diff.
14. **[S4] A quiet build that passes.** `mkdocs build --strict -q` exits 0 over a broken anchor (re-check T9): `-q`
    drops the WARNING records `--strict` counts. No script, CI step or hand check in this brief passes `-q`.
15. **[K2] A page number taken in another tree.** The first draft gave Bench Tests the number 22, which RMP1-A1's
    worktree already used (untracked, so `git grep` on main could not see it) — the two-chapter-19 problem of
    landmine 6 again. Before naming a new page: `ls` the folder in main, the staged index and EVERY in-flight
    worktree, and grep `briefs/` and `drafts/`.
16. **[K4] An anchor kept on the wrong page.** An `<a id>` satisfies only links to the file it is in. A split that
    moves a section's id to the new page leaves every old link (`18-mechanisms.md#…`) broken, and one left on the old
    page with no words lands the reader where the content is not. The old page keeps a VISIBLE "Moved to" pointer
    holding the old id; the reader pages are re-pointed in the same change.

---

## 9. Summary

**(amended 2026-11-13)** For the runs, sizes and dates now, read §A.7; for the decisions, §A.11. The table below is
the 11-12 summary of A1 and A2 alone.

| | |
|---|---|
| **What** | A "Start here" tab: a welcome page + six walkthrough pages, each around one complete, line-by-line commented program on a practice robot shaped like robot one; page 4 builds a 60 s practice skills route in six stages |
| **Kept true by** | every program compiled AND run by the suite on the host test plant with its claims asserted; the gate (inside `check-examples`) requiring every quoted line verbatim, every program shown WHOLE and tied to a test that runs it, every code line with its OWN comment, every output line present in its page's test, no fenced block escaping the scan; one mutation per page (13 + 8 gate) red |
| **Runs** | **[S1, F5] DOCS3-A1 ~4¼–5¼ builder-days** + review loop (~1): gates (¾–1), rig (¾–1, its composition probed first), the Tester-run examples (¼), pages (2¼–2¾, page 4 ~700–900 lines), front door (¼); built in D9 (a)'s order with a Nov 18 noon cut line. **DOCS3-A2** ~½–1 (more if pages were cut): the checked Toggle step (AF1S-A2), the route preview (RPV1), the camera link |
| **Waits for** | A1: ROB1-A and STEP1-B merged (CURVE1-A's commit); D-answers; **ROB1-C's slot (~Nov 14 earliest — MAX 3 JOBS, D9)**. A2: AF1S-A2 and/or RPV1 merged |
| **Honest dates** | **[F5]** A1 starts ~Nov 14 (~Nov 15 after one usage-limit stop) → page 4 first → cut line Nov 18 noon → reviewed, merged ~Nov 19 → **docs-only publish ~Nov 19–20 (D10)**, one or two days before the competition; in the Nov 18 freeze release only if nothing slips. Uncut, A1 would merge ~Nov 19½–20½ before any stop. Page 4's checked flip and page 6's preview ship as "coming"; A2 most likely after the 21st |
| **If slots are short** | the cut line IS the plan (D9 (a)); D9 (c) re-sized to ~3½–4 days and no longer better; D9 (d) (A1 ahead of RPV1, ~1½–2 days sooner) is the team lead's trade |
| **Hardware Assumptions** | none |
| **Robot code** | none (`src/`, `include/` unchanged — checked) |
| **Unverified** | the practice robot's composite rig (A1's step 1, stop rule ~¾ day); how long six pages of plain-words prose really take with the review (sized from guide 16/18's sizes, not measured); STEP1-B's and AF1S-A2's final spellings (A1 quotes only what is merged); the gate's rules 4 and 5 and the region tie (prototyped only as probes R1/R2 and the checker's `t.py`, not written into the tool); whether a whole-spec comparison exists for T6; `mkdocs` is NOT installed on this laptop (checked: `python3 -c "import mkdocs"` fails) — §7's site build needs it in a scratch venv first; the front page's "our robots have been X-drive and H-drive" (D11, the team lead's to confirm) |

---

## Re-check — amendments (2026-11-12, late; every finding re-verified by this writer, not taken on the checker's word)

Re-check probes: `.claude/coordinator/scratch/brief-DOCS3A/recheck/` — `strict_comment_check.py` + `r1.out` (R1),
`text_rule_probe.py` + `r2.out` (R2, over a `c0a7c7e` snapshot in `recheck/snap/`); the checker's own probes re-run
from `scratch/brief-DOCS3A/checker/`. Every run under `(ulimit -v 4194304; timeout 300 …)`.

| # | Finding (severity) | Re-verified | Disposition — where the body changed |
|---|---|---|---|
| F1 | One comment covers a whole multi-line declaration (major) | **REAL.** Re-ran `p2_comment_check.py` on `checker/cascade_probe2.md`: "0 uncovered of 17" with a single comment; `cont_probe.md` "0 of 7". R1's strict rule: the same files 15/17 and 5/7 uncovered; P1's listing still 0/7, its mutant still 1/7. | Continuation exemption DROPPED. D8, plain words, §0.5 P2, §1.6 rule 3, TG5b, MG5, landmine 11 (which had it backwards: chains passed the rule, they did not defeat it). |
| F2 | A titled or spaced fence escapes the gate (major) | **REAL.** Re-ran `checker/t.py` on `checker/t/`: "0 quoted lines, all verbatim … check-examples exit: 0" for a ```` ```cpp title=… ```` and a ```` ``` cpp ```` block; regex at `api_doc_tool.py` :2068; `mkdocs.yml` :84-86 has `pymdownx.highlight`/`superfences`. Public docs today: ZERO non-exact cpp fences (`git grep` at `c0a7c7e`; the only two hits are in `docs/internal/`). | New rule 5: in `docs/start/` only `cpp`/`text`/`sh` fences; everywhere, a cpp-ish fence must be exactly `cpp` (free today); no backtick inside a "Coming:" box. §1.4 coming form, §1.6, T-coming (prose-only names left to the §7 hand check, said plainly), TG7, TG8, MG6. |
| F3 | The Tester rig cannot run the practice route (major) | **REAL.** `auton_tester_rig.hpp` :153-167 (robot two's fakes), :270-310 (`TesterStack`: a bare `Chassis`, robot two's list bound, no Robot); `auton_wall_reset.hpp` :27-28: "the host test plant's tester rig, which builds no Robot, has none". | Took the checker's second option: the full route runs through `runAuton` on the practice rig (page 6 (a)); the Tester's own run is shown only with DRIVE-ONLY autons on the existing `TesterStack` (pages 1 and 6 (b)), each page saying so. No third rig; +¼ day. §1.4 pages 1, 6; §1.5; T1. A practice-robot Tester chain was rejected as unprobed (~½–1 day); RPV1-B1 generalises `TesterStack`'s mechanisms but still builds no Robot (RPV1 brief :356), so even then the wall reset would refuse. |
| F4 | ```` ```text ```` output is held by habit only (major) | **REAL.** `do_check_examples()` scans only cpp; R2 measured the habit at `c0a7c7e`: guide 18 — 17 of 18 text lines in no literal of its example tests; guide 09 — 2 of 2. | New rule 4 inside `check-examples` (same scan): every text line (or each `…`-fragment ≥ 4 chars) inside a string literal in a `start-NN…` TEST_CASE body of that page's test. Its use in an ASSERTION stays a logged review item, said plainly. Plain words, §1.4 frame, §1.6, T2, TG9, MG7, §2.3, landmine 3. |
| F5 | Dates and size do not fit MAX 3 JOBS (major) | **REAL.** NOTES :980 (MAX 3), :984 (slot order); RPV1-A0+B1 ~2½ + ~1 (RPV1 brief :41-44), ROB1-C ~½ (G1 brief :282), AF1S-A2 ~1¼–1½ (AF1S brief :719). Those three hold all three slots from ~Nov 13; the first to free is ROB1-C's, ~Nov 14. **Plus [S1]:** the brief's own parts summed to 3¼–4, not the 3–3¾ printed. | D9 rewritten with the arithmetic; A1 re-sized **4¼–5¼ + ~1 review**; recommendation changed to a fixed build order (page 4 first) with a **Nov 18 noon cut line** and the **docs-only publish ~Nov 19–20** stated as the likely path; new option (d) (A1 ahead of RPV1) offered, not recommended. D10, plain words, §1.1, §6.1, §9. |
| F6 | Page 1's autons file would collide in the test binary (minor) | **REAL.** `test/CMakeLists.txt` :90 links `src/team/autons.cpp` (`shulib::bench::registerAutons`); *(worktree `rob1-a`)* :93 links `robot_one_autons.cpp` (`registerRobotOneAutons`). | Names fixed in the brief: `namespace shulib::start`, `shulib::start::practice`, `registerPracticeAutons`; page 1 says in one line which names differ in the real file. Quoting `robot_one_autons.cpp` itself rejected (placeholder file, pinned entries, changes under the page). §1.2, §1.4 page 1. |
| F7 | The build misses the header; widening the whole-run rule has side effects (minor) | **REAL.** Doc-gate inputs `test/CMakeLists.txt` :213-221 match `*example*_test.cpp` only; whole-run ownership at `api_doc_tool.py` :2076-2081; empty-glob failure :2042-2047. | `start_*_example.hpp` added to the build inputs; `WHOLE_RUN_SOURCE_GLOBS` LEFT AS IS (the region rule does the work for the track); globs and the first start files in one change. §1.3, §1.6 rules 1–2, TG10 (a guide listing still passes). |
| F8 | A region is tied to its test only by name (minor) | **REAL** — rule 2 checked only a TEST_CASE name. | The BEGIN marker names what the region runs (`runs: <name>`); the gate requires the name defined in the region AND used in a `start-NNx` TEST_CASE body; anonymous-namespace auton functions as habit (`-Wall -Werror`, `test/CMakeLists.txt` :22-24). §1.6 rule 2, TG (region tie), MG8. |
| F9 | Two review greps cannot pass (minor) | **REAL.** `MECH` is the log tag (`mechanism/service.hpp` :448, `"MECH"`); page 5's own ruling wording is "tuning on the robot". | `MECH-[A-Z]` in the chunk-id grep; the claim grep narrowed to `never driven\|has run on\|ran on\|drove\|was tested on`. §3.4. |
| F10 | T6's "COPY equals the declaration line" cannot be written (minor) | **REAL.** Guide 18 :494-501: COPY prints `.tuning.kP = 0.3125`-style lines plus begin/end lines. | T6 and page 5 (c) restated: COPY is output (```` ```text ````, rule 4); a hand-rewritten declaration in a checked region must build to the same spec as the tuned run. Whether a whole-spec comparison already exists to reuse: UNVERIFIED. |
| F11 | §7's site build was a placeholder (minor) | **REAL.** `pages.yml` :62 `prepare_site.py site_src`, :70 `mkdocs build --strict`; `mkdocs.yml` :23 `docs_dir: site_src`. **And `mkdocs` is not installed here** (`import mkdocs` fails). | §7 spelled out: copy `mkdocs.yml` into a scratch folder, `prepare_site.py <scratch>/site_src`, `mkdocs build --strict -f <scratch>/mkdocs.yml`, all under the caps, in a scratch venv with CI's pins (mkdocs 1.6.1, material 9.5.44). |
| F12 | D11's "two sentences" rewrites the front page's reason for the library (minor) | **REAL in substance; its line number is WRONG**: "our robots slide sideways" is at `docs/README.md` **:28**, as the brief said, not :27 (`git show c0a7c7e:docs/README.md \| grep -n`: 28, 32, 33). | D11 now gives the exact old and proposed wording for the team lead to approve, and asks him to confirm the claim about earlier robots. |
| F13 | Page 4 is longer than ~500 lines; the route-guard has no named reviewer (minor) | **REAL** — six whole programs plus walkthroughs plus output. | Page 4 sized ~700–900 lines; walkthroughs show only each stage's NEW lines; D2 (c) offered as a free choice now, (a) still recommended; the team lead reads page 4's route before the merge and says it is not one the team would run (§2.3). D2, §1.2, §1.4 page 4, +½ day in §9. |
| S1 | *(this writer)* A1's printed size did not equal its own parts | ½ + ¾–1 + 1¾–2¼ + ¼ = 3¼–4, printed 3–3¾ | Folded into F5's re-size. |
| S2 | *(this writer)* Page 2 planned to quote ROB1-A's flipper-forms test, which rule 2 (a listing must be a run of a `start-NNx` region) forbids | §1.4 page 2 vs §1.6 rule 2 contradicted each other | Page 2's own region `start-02c` declares the two other flipper kinds; ROB1-A's file is LINKED as robot one's version. §1.2, §1.4 page 2. |
| S3 | *(this writer)* D9 (c)'s 2–2½ days was too low | its gates and rig alone are ~1½–2 | Re-sized ~3½–4; no longer better than (a)'s cut line. |

### The decisions as now recommended (plain words, for the team lead)

- **D1** a new "Start here" tab, before the User guide (unchanged).
- **D2** six pages plus a welcome page; page 4 is long (~700–900 lines) but reads as one story. Say if you would rather
  it be two pages — it costs nothing extra if decided now.
- **D3–D7** unchanged: one practice robot shaped like robot one with made-up numbers; page 4's route grows in stages;
  reader = a first-year student who knows a little C++; C++ explained in small boxes; a practice route, not a real plan.
- **D8** the machine now checks that every code line has ITS OWN comment (the first draft let one comment cover a block).
- **D9 (changed)** with three job slots, the docs run can start ~Nov 14 at the earliest, in the slot ROB1-C frees. It is
  ~4¼–5¼ days of building, so it builds page 4 first, then 3, 5, 2, 1, 6; at noon on Nov 18 any unfinished page becomes
  a short page of links. **Most likely the track reaches the site ~Nov 19–20, one or two days before the competition,
  not with the Nov 18 release.** If you want it sooner, (d) puts it ahead of the route preview (~1½–2 days sooner) —
  your trade; not recommended.
- **D10** a docs-only publish before the 21st, only if no robot code changed since the freeze release.
- **D11 (changed)** please approve the exact front-page wording in D11 and confirm "our robots have been X-drive and
  H-drive" is true.
- **D12** one line at the top of chapter 8 (unchanged).

### The runs, sizes and honest dates

- **DOCS3-A1** ~4¼–5¼ builder-days + ~1 review. Starts ~Nov 14 (~Nov 15 after one usage-limit stop), only once
  ROB1-A and STEP1-B are merged. Cut line Nov 18 noon → merged ~Nov 19 → docs-only publish ~Nov 19–20. Uncut and with
  no stop it would merge ~Nov 19½–20½; each usage-limit stop adds about a day, and one stop late on Nov 19–20 could
  push it past the 21st — the cut line is what keeps it before.
- **DOCS3-A2** ~½–1 (more for any page the cut line left as links): the checked Toggle step, the route preview, the
  camera chapter's link. Most likely after the 21st.

### What stays unverified

The practice robot's composite rig (A1's first step, ¾-day stop rule); rules 4 and 5 and the region tie exist only as
probes (R1, R2, the checker's `t.py`), not in the tool; whether T6 can reuse a whole-spec comparison; `mkdocs` is not
installed here, so the site build has never been run by this writer; how long the plain-words prose and its review take
(sized, not measured); STEP1-B's and AF1S-A2's final spellings; the front page's claim about earlier robots.

---

## Re-check — the navigation and readability amendment (2026-11-15 late; every finding re-verified by this writer, not taken on the checker's word)

Re-check probes: `.claude/coordinator/scratch/brief-DOCS3A/amend/recheck2/` and `recheck3/` (T1–T11, §A.10), over
the snapshot of `9befdb3` (`amend/snap/`), the main worktree's staged index, and the in-flight worktrees read ONLY
(`rob1-c2`, `rob1-bt`, `af1s-a2`, `rmp1-a1`, `rpv1-s1`, ~19:30–20:00); NOTES.md :1045–1063. Every binary under
`(ulimit -v 4194304; timeout …)`, the browser included.

| # | Finding (severity) | Re-verified | Disposition — where the brief changed |
|---|---|---|---|
| K1 | The slot plan leaves out pre-21st runs; "guide 18 ~Nov 20" not honest (major) | **REAL.** NOTES :1052/:1056 (starter route, "tonight, short"), :1059 (MIRROR1-B before the Wed freeze), :1060 (RMP1-B before the 21st), :1063 (the label fix); RPV1-S1 BUILT, not queued (worktree `rpv1-s1`, `RPV1-PROGRESS.md` :654-765: build log, MS1–MS11, prose review; its src-gate self-test running at ~20:00); READ1 still needs its brief. Guide 18 is edited by ROB1-C2 (+4) and ROB1-BT (17+/13−, 8 hunks). And **[S5]** the weekly limit was hit at ~19:55 (NOTES :1062). | §A.7 re-done: every run listed, a slot-by-slot table, P1a's base (after ROB1-C2 and ROB1-BT, or the coordinator ports ROB1-BT's hunks), dates re-derived, the extra-usage assumption stated; D17's recommendation changed to (b) menu first, with plain words that guide 18 lands after the 21st in every order. |
| K2 | In-flight pages missing from the menu; 22 taken (major) | **REAL.** `cookbook/08-skills-toggles.md` *(af1s-a2, untracked)*; `route-builder.md` + `guide/22-making-your-first-route.md` *(rmp1-a1, untracked)*, and RMP1-A1's own nav adds a top-level "Route builder" tab. Re-built the menu over the future tree (T7): 275/275, strict + anchors exit 0 with the app's files stubbed. | §A.3: ten tabs (the Route builder tab kept — approved with RMP1), cookbook 08 under Routes → Recipes, Bench Tests = 23, "whichever merges second places the other's pages", check-nav fails a dropped page; §A.2: the route builder leads "Write a skills route"; D13, D20; landmine 15. |
| K3 | The strict build and the planned CI job fail once RMP1-A1 merges; ci.yml/pages.yml ownership; loose pins (major) | **REAL.** T5: RMP1-A1's tree without the app's two files → exit 1 ×2 warnings; RMP1-A1's `pages.yml` +42 (cmake, `shulib_route_maker`, `build.py`, PG1, ubuntu-24.04) and `ci.yml` +96. The probe venv: 30 packages, pymdown-extensions 10.21.3, regex 2026.11.15, and mkdocs-redirects (not in CI). | §A.8 rewritten: `site_check.py` is the ONE definition, called by `pages.yml` and the CI site job; the route builder built when its page exists (the three commands written out); PG1 moved in; full pin set; the site machinery moved from N1 to N2, which starts after RMP1-A1's merge or rebases onto it; §A.7 shared files. |
| K4 | Old anchors cannot live "where the content went" across files (major) | **REAL.** T8: link to `18-mechanisms.md#moved-tune` with the id on the split page → exit 1; with it on guide 18 → exit 0. Counted: 31 of the 34 anchored inbound links target moved sections (the checker said ~29) — 25 on reader pages, 6 in the changelog and roadmap; plus guide 18's own 11 in-page links. | §A.3's rule rewritten (same page: invisible id; split: a visible "Moved to" pointer on the old page + the 25 reader links re-pointed; ledgers keep theirs), P1a's check lists every inbound anchored link and its target; §2.3; landmine 16. |
| K5 | Rule L's four headings clash with pinned headings and collection pages (major) | **REAL.** `wall_reset_line_test.cpp` :286-287, `toggle_visit_line_test.cpp` :183-185 (guide 11's two headings); cookbook 01's five recipe H2s, cookbook 07's two, guide 12's nine symptoms; 18-rules and the FAQ listed both ways. | §A.4: page types (TASK, RECIPES, TROUBLESHOOT, LEARN, REFERENCE, LEDGER, WALKTHROUGH) declared in `CONVERTED`; rule L per type; guide 11, 14, 18-rules, FAQ, glossary = REFERENCE (the pins stay legal); §A.6; D14. |
| K6 | Rule T misses "honesty", fences unclear, preambles unchecked, the sweep comes late (major) | **REAL** (one word overstated: `guide-maintenance.md` :97 does not REQUIRE the sentence — it lists where it lives, which the sweep makes untrue). "Honesty": 6 on reader pages + 1 in `README.md` (`9befdb3`), incl. cookbook 04 :4 and guide 04 :78's heading (linked from guide 16 :232). Fences: exactly two hits, both test comments (`guide_examples_test.cpp` :112, :481); no program prints the word. T2: openings on guides 16, 17, 18, 21 and cookbook 07. | Rule T (a) `\bhonest(y\|ly)?\b` everywhere incl. fences, the two comments reworded and re-quoted; (b) the opening check + a logged grep elsewhere; **the sweep and `check-tone` moved INTO N1** (global from N1); guide-maintenance :97 rewritten by N1; §A.1 W4 counts; D15. |
| K7 | Rule H's holes; estimate links point at a 5,756-line page (minor) | **REAL.** The prototype strips backtick spans first; 2 spans hold an id/date today, neither bare; 227 `\| HA-` rows, 0 anchors; the prototype matches short ids (and `L4` is a word in a log line, guide 11 :559) and misses `DIAG1b`. | Rule H's exact pattern, `CHUNK_IDS` + `SHORT_IDS` (with each collision), a self-test against the 59 prefixes under `docs/internal/chunks/`; bare-id inline code fails; register row anchors added by `prepare_site.py` at site build (T6: resolves; a missing row fails), in N1. |
| K8 | Phase sizes inconsistent (minor) | **REAL.** At P1a's own rate (~3k words/day once its tool is out): old P1b 13.6k–14.9k words ≈ 5½–6½ days, old P1c 14.3k ≈ 6½–7, phase 3 ~28k ≈ 9½–10 (was 6–8). | One stated rate, measured by P1a and used to re-date; P1b and P1c each split in two; every picture pipeline probed first with a stop rule (§A.5); total ~33–37 + ~10 review. |
| K9 | The first-week picture and the Robot menu disagree with guide 21 (minor) | **REAL.** Guide 21 :175-191 (and ROB1-C2's :183-198): Devices screen → Bench Tests (+ Calibrate) → station 13 → Drive, Auton Tester, Competition, Tune. | The picture is drawn FROM guide 21's list (its digest in the manifest); the Set up menu and the front page row in that order. |
| K10 | Generated API pages keep "user guide" / "chapter 10" (minor) | **REAL.** `api_doc_tool.py` :1405, :1614, :1641-1642; no in-flight run edits the tool. | N1 changes the three strings (and the :1945-1946 comment) and regenerates `docs/api/`; A1 keeps the gate-rule region — different regions, serial merge. |
| K11 | 224 unlinked "Chapter N" stop resolving (minor) | **REAL.** 529 total, 305 in link text, 224 plain (glossary 147, guide README 29, cookbook README 8, ~40 elsewhere). | N1 makes every unlinked one a link (its own rewrites + a one-off script, diff reviewed); labels carry no numbers; the Learn landing keeps an old-number index. |
| K12 | Split letters collide with test markers (minor) | **REAL.** `guide-18a`…`guide-18h` name listings (18d = driver controls, 18g = current cap, 18h = live tuning); no tool reads the letter. | Split files named with words (`18-mechanisms-on-buttons.md` …), which sort beside guide 18 and match every glob; markers keep their names; the tests' mapping comments say where each listing now lives. |
| K13 | ROB1-BT carries a broken anchor today's gates miss (minor) | **REAL.** *(rob1-bt)* cookbook 06's heading renamed ("A whole driver file: the robots' …", :235); `cookbook/README.md` :102 still links `#a-whole-driver-file-robot-twos`; strict + anchors → exit 1. | Told to the coordinator (§A.3 table, A.11 heads-up); it argues for `anchors: warn` early — it lands with N1, the first docs slot. |
| K14 | One shared progress log for parallel worktrees (minor) | **REAL.** T11: per-run sections pre-made in one file merge cleanly (exit 0 twice); appends at the end conflict (exit 1). | The plan commit creates `DOCS3-PROGRESS.md` with the between-parts marker and one heading per run; a run writes only under its own (§A.7, §2.3). |
| K15 | Six smaller accuracy items (minor) | **(a) REAL** — RMP1-A1's capped Playwright Firefox exists; T10 ran it under the cap: ten tabs fit at 1280 px, the 400 px drawer lists all ten. **(b) REAL** — only `service.hpp` :144 names a numbered doc file. **(c) REAL** — the screen-picture bullet would ship before any picture; "Why this library exists" is two paragraphs (:66-77), D11's wording the first. **(d) REAL** — the probe imports `yaml`; no tool does; `mkdocs.yml` :81 has a `!!python` tag. **(e) REAL** — guide README :49-171; pointed at by `api_doc_tool.py` :1945-1946 and `guide_examples_test.cpp` :5; nothing links its subsections. **(f) REAL** (counts differ by a few by method) — `src/robots`/`main.cpp` named on guides 07, 08, 11, 18, 19, 21, cookbook 06, `README.md`, the facts form; guide 21 :53-56, :97-99 quote `robot_one_facts.hpp` in `text` fences. | (a) §A.3, §A.10, §2.3; (b) §A.3, D13; (c) §A.2; (d) §A.3 (a standard-library parser that refuses unknown lines; PyYAML cross-check in `site_check.py`); (e) §A.2: a new "How these pages stay true" page in Project, the two comments updated by N1; (f) §A.9: READ1 greps every public page for each file's name and quoted lines, any fence. |
| S4 | *(this writer)* `-q` hides a strict failure | T9: exit 0 with `-q`, exit 1 without, same broken anchor | §A.3 table, §A.8, §2.3, §7, landmine 14. |
| S5 | *(this writer)* The weekly usage limit | NOTES :1062: hit ~19:55, "resets Nov 18, 10pm"; extra usage being bought | Every date in §A.7/D17 assumes extra usage; without it nothing of the docs lands before the 21st. |
| S6 | *(this writer)* READ1 meets the freeze | READ1 edits `src/robots/*` and `src/main.cpp`; its build follows ROB1-BT (~Nov 17) → merge ~Nov 18½–19 | Flagged for the coordinator (§A.7, A.11 heads-up 3); it decides with the team lead. |
| S7 | *(this writer)* "P1a quotes none of READ1's files" was not quite true | guide 18's COPY example names `src/robots/mechanism_table.hpp:41`, pinned by `mechanism_live_tuning_test.cpp` | §A.9: whichever of READ1 and P1a merges second changes the pin and the page together. |

### The decisions as now recommended (plain words, for the team lead)

- **D13** a menu by task with **ten** tabs — Start here, Route builder, Robot, Routes, Mechanisms, Troubleshoot, Learn,
  API reference, Project, and Home; nothing renamed, so no link or bookmark breaks; every "Chapter N" becomes a link.
- **D14** one shape per KIND of page (how-to pages, recipe lists, troubleshooting, reference), not one shape forced on
  all.
- **D15** internal history and the word "honest" go; the "honest" words and the "has not run on a robot yet" openings
  go from every page in the FIRST piece, and checks keep them out.
- **D16** brain-screen pictures drawn by the code itself (unchanged).
- **D17 (changed)** the menu first: live ~Nov 19 (before the 21st with a day to spare); guide 18 starts ~Nov 17 and
  lands ~Nov 22 — after the competition whichever goes first. All dates assume the extra usage stays on.
- **D18, D19, D21, D22** unchanged. **D20** the Bench Tests page is page 23 (22 is the route builder's).
- **For the coordinator, not the team lead:** the Tune button label, ROB1-BT's broken cookbook link, READ1 meeting the
  freeze.

### The runs, sizes and honest dates (one builder-day ≈ one calendar day; extra usage on; no usage stop)

| Run | Size + review | Starts | Merged / released |
|---|---|---|---|
| N1 — menu, front page, tone sweep | ~2–2½ + ½ | ~Nov 16 midday (after READ1's brief) | ~Nov 19 (~Nov 20 after one stop) |
| P1a — guide 18 | ~3½ + 1 | ~Nov 17 pm (~Nov 18 if RMP1-B takes the slot) | ~Nov 22 (~Nov 23) |
| N2 — the checks | ~2 + ½ | ~Nov 19 (after N1; on main after RMP1-A1 where it can) | ~Nov 21½ |
| A1 — the beginner track | ~4–5 + 1 | ~Nov 19 (after READ1) | ~Nov 24–25 |
| P1b-1 / P1b-2 — routes | ~3–3½ + 1 / ~2½–3 + 1 | ~Nov 20 / ~Nov 20–21 | ~Nov 24 / ~Nov 24–25 |
| P1c-1 / P1c-2 — bench and match day | ~3½ + 1 / ~3¼ + 1 | ~Nov 20–21 / ~Nov 22 | ~Nov 25 / ~Nov 26 |
| A2 | ~½ | after A1 | ~Nov 26 |
| P3a/b/c | ~9½–10 + ~3 in total | ~Nov 25–27 | ~Dec 1–4; early to mid December with stops |

Before the 21st: N1 only. Each usage-limit stop adds about a day to every run it hits; without the extra usage, no
agent run moves until Nov 18 ~22:00.

### What stays unverified

The rewrite rate (~3k words a builder-day is P1a's planned number, not a measurement — P1a measures it); the picture
pipelines not yet probed (the route preview's `renderSvg` on a docs example; the selector painter running on the host;
Calibrate's screens); the real route-builder app inside a docs build (T7 stubbed its two files); whether ROB1-BT,
MIRROR1-B, READ1 and RMP1-B finish when NOTES expects; whether the extra usage stays on; the speed of five parallel
jobs before a usage stop; the practice robot's composite rig (A1's first step, unchanged).
