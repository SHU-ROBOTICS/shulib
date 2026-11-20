# DOCS3 — progress log (the docs redo)

<!-- release: between parts — DOCS3 runs land one at a time (N1 first); the coordinator updates this reason at each merge -->

Brief: [`DOCS3A-beginner-track.md`](DOCS3A-beginner-track.md) (its amendment §A.7 defines the runs).
One heading per run, in the brief's order ([K14]). **A run writes only under its own heading.**

## DOCS3-N1

Executor: Claude Opus 5.5 (subagent), worktree `.claude/worktrees/docs3-n1`, branch `docs3-n1` from `e9bcee7`.
Started 2026-11-15. Scratch: `.claude/coordinator/scratch/docs3-n1/`. Host only; nothing under `src/` or `include/`.

### Read first
RESUMING.md's standing rules; the brief's ANSWERS block, the amendment (A.0–A.11), the re-check (K1–K15, S4–S7),
§6 coordination, §7 checks, §8 landmines, and every N1 row.

### Built (2026-11-15, in order)
- **Menu** (`mkdocs.yml`): the task tabs. At N1 they are Home · Robot · Routes · Mechanisms · Troubleshoot · Learn ·
  API reference · Project — **eight, not ten**: "Start here" (A1) and "Route builder" (RMP1-A1) have no files on
  this base, and N1's menu lists only files that exist (§A.3 [K2], §A.7 "N1's menu has no Start here tab"). Each
  joins with its pages, placed by whichever run merges second. `validation: links: anchors: warn` added.
- **`tools/doc_readability.py`** (new): `check-nav` (stdlib line parser, refuses any menu line it cannot read),
  `check-tone` (rule T (a) + (b), every public page but `docs/api/`, plus `README.md`, `test/README.md`),
  `self-test` (34 planted cases + the tree's lists: PUBLIC_DOC_GLOBS ⊆ the audit's PATH_GLOBS, every folder of
  public pages ⊆ PUBLIC_DOC_GLOBS). First run on the base: check-nav 257/257 OK; check-tone 132 findings — the
  word on 29 pages + README.md, and rule (b) on 6 openings (guides 16, 17, 18, 21, cookbook 07 = probe T2's five,
  **plus the master plan :7**, a LEDGER page — T applies to ledgers too).
- **Front page** `docs/README.md` (§A.2; the D11 paragraph verbatim; the three questions as a list; three
  one-line bullets + the new page's link; "Reference material" gone) and **its first-week picture**
  `docs/assets/first-week.svg`, drawn by `tools/docs_figures/first_week.py` FROM guide 21's "The order on the
  robot" (each step's bold programs with their slots, else its first bold words); `first_week.py check` fails when
  the page and the picture disagree ([K9]).
- **`docs/guide/README.md` → "How it works"** (the ideas in order, which kind of page, every page by its old
  number); its Maintenance part moved to **`docs/how-the-docs-stay-true.md`** (Project tab), with a
  table of what each check holds at its top ([K15e]) — words kept except two rewrites, logged in review round 2
  (R2-δ2).
- **`docs/cookbook/README.md` → "Recipes: which to read"** (every recipe table kept; the assumptions as three
  short points; the stale "no `Lift` type, ever" bullet gone — guide 18 has `Lift`).
- **Tone sweep** (126 exact replacements, each asserted unique — scratch `tone_sweep.py`): every "honest",
  "honesty", "honestly" on public pages outside `docs/api/`; the six opening status lines; guide 01's
  "## The honest part" → "## How it is tested" (stale lines gone: "the robot doesn't exist yet", "the adapters
  are the next phase"); guide 04's heading + guide 16's link to it; `README.md`'s "How verified is it?" + guide
  14's link to it; guide 12's scoping note; guide 07's status paragraph → a pointer to guide 14; the two test
  comments (`test/guide_examples_test.cpp` :112, :481) reworded and re-quoted in guides 08 and 09. Renamed
  headings checked for inbound links first: only guide 04's (guide 16 :233) and `README.md`'s (guide 14 :84)
  had any; both re-pointed. No `<a id>` kept for a renamed "honest" heading — the id itself would hold the
  word that check-tone refuses.
- **"Chapter N" links** ([K11]): scratch `chapter_links.py` (deterministic, idempotent — re-run gives 0):
  42 unlinked "Chapter N" on reader pages → `[Chapter N](file)`; the glossary's 147 "[Ch. N]" → "→ [Title](file)"
  by the menu's titles; ranges by hand (guides 01, 10, 12). Re-scan: 0 unlinked (one line-based false positive,
  cookbook 07 :3, is a two-line link).
- **Glossary anchors**: 135 terms, `<a id="term"></a>`, ids unique (a collision takes the full term:
  `frame-a-cameras`).

- **`tools/prepare_site.py`**: the register's row anchors in the SITE COPY only ([K7]) — `anchor_rows()` gives each
  `| HA-NNN |` first cell `<a id="ha-NNN"></a>`; the copy step fails if any row is left without one or one is
  anchored twice; new `self-test` (5 cases, "a row without its anchor fails" among them). Site build: 230 rows
  anchored.
- **`tools/api_doc_tool.py`** ([K10]): the three generated strings ("user guide", "chapter 10" → "How it works",
  "The API in words", "Recipes") and the :1945 comment; `docs/api/` regenerated (215 pages, 1 line each, + README).
- **Test comments only** (`test/guide_examples_test.cpp` :5 → `docs/how-the-docs-stay-true.md`; :112, :481 lose
  the word) — re-quoted in guide 08 :64 and guide 09 :197; check-examples green (919 lines verbatim).
- **Doc gate** (`test/CMakeLists.txt`): `doc_readability.py self-test`, `check-nav`, `check-tone`,
  `docs_figures/first_week.py check`, `prepare_site.py self-test`; DEPENDS gain the tool, the figure script, the
  SVG and `prepare_site.py`. (`ci.yml`'s explicit doc step is NOT touched — K3: RMP1-A1 and N2 own it; CI runs the
  new checks through the build's doc-gate target.)
- **Internal**: `guide-maintenance.md` :6, :97 (the R3 row, [K6]), :147, :185; `docs-publishing.md` (one row:
  a hand-written page joins the menu, check-nav holds it); `PROJECT-BRIEFING.md` (two sentences: the reading
  checks, the register anchors + "never `-q`"); the changelog (one entry under API 2.9, no API change).

### The status pattern beyond the openings — the logged review grep (rule T (b) elsewhere)
`STATUS` over every tone page, outside the opening: **103 hits** (scratch `status_elsewhere.out`) — ledgers 70
(roadmap 31, changelog 27, register 10, VexBuilder 2: dated history records, LEDGER = T only, kept), guide 14 (9,
the place for them, kept), `README.md` 4 (:86, :102-103 in "What it is NOT yet" — kept; :338 inside the tree
listing's text fence, a status aside — P3c's rewrite). Reader pages, each read:
- **fixed (section-level status lines):** guide 03 :273 and guide 10 :463 (`*Host- and build-gated, NOT run on a
  robot.*` opening the wall-reset sections), guide 10 :714 (the sentence closing the mechanisms box), guide 16 :43
  ("Like the tester, it has not run on a robot yet.").
- **kept — a limit stated in place, or naming where a result comes from:** cookbook 07 :125 (a "watch out" limit
  with what is unmeasured), guide 02 :229 (numbers "read on paper"), guide 09 :425 (with its register entry
  HA-167), guide 10 :138 (motor lag unmeasured), guide 11 :473, :544, :622 (each says the lines are examples /
  the host test plant's, pinned by tests), guide 17 :262 (the mirror's limits), :554 (under "What it does not do
  yet"), guide 18 :619 ("tested on the host test plant"), :887 (the bench page's first run).
- **kept, for the page's own run:** guide 08 :313 (an aside on "shulib Drive", phase 3), guide 10 :853, :869
  (station 13 asides, phase 3), guide 18 :422 (P1a rewrites guide 18). faq :180 is not a status line
  ("a Calibrate build you have not run before").

### Mutations (scratch `mutate_n1.py`, `mutations.out`; each a FRESH run under `ulimit -v 4194304; timeout 600`,
### restored in a finally, backups in `scratch/docs3-n1/backups/`, the restored file's mtime bumped)
| # | Mutation | Command | Result |
|---|---|---|---|
| M1 | the "must be indented" refusal → `pass` | self-test | **survived — EQUIVALENT**: the next line (`ITEM` needs leading spaces) still refuses the line ("not a menu line"); M1b removes both |
| M1b | that refusal AND `ITEM`'s indent requirement | self-test | RED — "nav: an unindented item is refused" |
| M2 | the "not in the menu" loop emptied | self-test | RED — "a page missing from the menu fails" |
| M3 | `count > 1` → `> 2` | self-test | RED — "a page listed twice fails" |
| M4 | "names no page" check off | self-test | RED |
| M5 | `honest(?:y\|ly)?` → `honest` | self-test | RED — "'honestly' in inline code fails" |
| M6 | `re.I` dropped | self-test | RED — "'HONESTLY' fails" |
| M7 | fence tracking in `opening()` off | self-test | RED — "a '## ' inside a code block does not end the opening" |
| M8 | emphasis not stripped | self-test | RED |
| M9 | `never driven` alternative dropped | self-test | RED |
| M10 | `not … run on a robot` alternative dropped | self-test | RED |
| M11 | the "folder outside PUBLIC_DOC_GLOBS" check emptied | self-test | RED |
| M12 | the title first-character check off | self-test | RED — "a title with a YAML anchor or tag is refused" |
| M13 | "section has no pages" off | self-test | RED |
| M14 | `site_path` keeps `README.md` | check-nav (real tree) | RED — index.md names no page; docs/README.md not in the menu |
| M15 | `first_week.py`: `(slot N)` misread | first_week check | RED |
| M16 | guide 21's step 3 renamed (data) | first_week check | RED — "no longer matches guide 21's 'The order on the robot'" |
| M17 | `anchor_rows` anchors nothing | prepare_site self-test | RED |
| M18 | `unanchored_rows` returns `[]` | prepare_site self-test | RED |
| M19 | `HA_ROW` loses `^` (matches an id in a later cell) | prepare_site self-test | RED |
| M20 | `mkdocs.yml`: FAQ's line deleted | check-nav | RED |
| M21 | FAQ listed twice | check-nav | RED |
| M22 | FAQ's line names `faq-gone.md` | check-nav | RED |
| M23 | "honest" planted in guide 02 | check-tone | RED |
| M24 | a status line planted in guide 02's opening | check-tone | RED |
| M25 | THROUGH THE BUILD: `docs/guide/99-orphan.md`, in no menu | `cmake --build build/test` | RED — exit 2 at `doc_gates.stamp`, "check-nav: docs/guide/99-orphan.md: not in the menu"; removed → rebuild exit 0 |
| M26 | site config without `anchors: warn`, a broken glossary anchor planted | strict mkdocs build | exit 0 (INFO only) — so the line is load-bearing: with it (probe P3) exit 1 |
Restored tree after the runner: self-test, check-nav, check-tone, first_week check, prepare_site self-test all green.

### Site build (scratch venv, CI's pins mkdocs 1.6.1 + mkdocs-material 9.5.44; never `-q`)
`prepare_site.py` → 262 files, 258 pages stamped, 230 register rows anchored; `mkdocs build --strict` exit 0, no
WARNING, no "not included in the nav" INFO. Planted-link probes (`anchor_probes.out`): P1 `#ha-136` exit 0; P2
`#ha-99999` exit 1; P3 glossary `#no-such-term` exit 1; P4 the same with `-q` **exit 0** (landmine 14 reproduced);
P5 glossary `#pose` exit 0. The built HTML: no internal path (pages.yml's grep, empty).

### Screenshots (Playwright 1.49.1 in `~/shulib-render-tmp/docs3-n1/venv`, its own Firefox 132 in `…/browsers`,
### under the cap; the http server and the browser stopped by PID after)
`scratch/docs3-n1/shots/`: `front-1280.png`, `front-400.png`, `menu-drawer-400.png`, `learn-how-it-works-1280.png`,
`robot-tab-1280.png`. At 1280 the eight tabs end at 670 px of a 1280 px bar; at 400 the tab bar hides and the
drawer lists the eight in order (Home, Robot, Routes, Mechanisms, Troubleshoot, Learn, API reference, Project).

### Deltas from the brief (each with its reason)
1. **Eight tabs at N1, not ten.** "Start here" (A1) and "Route builder" (RMP1-A1) have no files on `e9bcee7`;
   N1's menu lists only files that exist (§A.3) and §A.7 says N1 has no "Start here" tab. Each joins with its pages.
2. **This log was created by N1** with the between-parts marker and one heading per run ([K14]'s layout) — the
   plan commit that was to create it is not on this base.
3. **The first-week picture is held by `first_week.py check` in the doc gate** — `figures.json`/`check-figures`
   are P1a's and do not exist yet. The check compares the whole SVG re-drawn from guide 21's list (stronger than a
   digest of the list); P1a folds the script into its manifest.
4. **Six openings, not five:** the master plan :7 (a LEDGER) also opened with "has never driven" — now a link to
   guide 14's list.
5. **Four section-level status lines removed beyond the openings** (guide 03 :273, guide 10 :463 and :714, guide 16
   :43) — the logged grep's criterion; the rest kept with a disposition (above).
6. **No invisible `<a id>` for the renamed "honest" headings** — the id itself would carry the word check-tone
   refuses; only two inbound links existed (guide 16 → guide 04, guide 14 → `README.md`) and both were re-pointed.
7. **Guide 18's menu title is "Declare, drive and tune a mechanism"** until P1a splits it ("Declare a mechanism"
   would mislabel the whole page today).
8. **The glossary's "[Ch. N]" became "→ [Title](file)"** (its own rewrite, [K11]); elsewhere "Chapter N" kept its
   text and became a link.
9. **check-tone reads every folder under `docs/` by glob** (not PUBLIC_DOC_GLOBS), plus `README.md` and
   `test/README.md` — a new folder cannot escape it; the self-test separately holds PUBLIC_DOC_GLOBS ⊆ the
   audit's PATH_GLOBS and every page folder ⊆ PUBLIC_DOC_GLOBS.
10. **`prepare_site.py self-test` is new** (there was none) and runs in the doc gate.
11. **Small stale facts fixed where N1 was already editing:** the cookbook README's "no `Lift` type, ever",
    guide 21's "(Chapter 20 is held for 'Your robot file' …)", guide 01's "the robot doesn't exist yet".
12. **Not added (other runs' lines):** the front page's "Learn to code our robot" row (A1) and the route builder's
    two links (RMP1-A1).
13. **`ci.yml`/`pages.yml` untouched** ([K3]); CI runs the new checks through the build's doc-gate target.
14. **[K15d] probe:** check-nav's stdlib parse vs PyYAML's in the docs venv — 258 = 258 paths, same order
    (scratch `yaml_crosscheck.py`).
15. **Reviewers:** this executor has no way to start sub-agents, so the three reviewers and their fix round are
    the workflow's/coordinator's next step; nothing here claims them.

### Prose review (rule 4) — the greps, run and read BEFORE this entry (scratch `prose_review_greps.out`, `s34_greps.out`)
- `doc_readability` → how-the-docs-stay-true :11-12, the changelog entry, the briefing, guide-maintenance :98 — all
  written in this run, true. `prepare_site` → the briefing :890 (updated), RELEASING.md :39 (the leak screen, still
  true). `first_week|first-week` → front page :21, the briefing — true.
- `user guide` → README.md :367 ("the user guide", the folder — still true; README is P3c's), roadmap :692, :2200,
  changelog, HANDOFF (ledgers/history, kept). `guide README|guide/README` → guide-maintenance :6/:148/:185 (updated),
  :97 (an executed D2 row, history, kept), docs-publishing :51 (an example path, true), changelog :3411 (ledger).
- The comments that pointed at the guide README's Maintenance part: `test/guide_examples_test.cpp` :5 and
  `api_doc_tool.py` :1945 — both re-pointed. `test/cookbook_examples_test.cpp` :6 ("keep in sync with
  docs/cookbook/README.md") — still true (the README keeps every recipe table).
- §3.4's greps over N1's four landings (front page, both READMEs, how-the-docs-stay-true): `simulat` 0;
  `never driven|has run on|ran on|drove|was tested on` 0; the chunk-id pattern 0; `G4|LLM|AI-written` 0;
  `robot one's … is N` 0; `honest` 0; `Tier|facade|scheduler|provider|seam` 2 and the short ids 12 — all in
  how-the-docs-stay-true's MOVED history list (F1/F2/D3 executed items, "the mechanism seam"; words kept, a
  maintainer page); every "Chapter N" there is a link.

### Gates — final lines (worktree, after the last edit)
- `cmake --build build/test -j8` exit 0 (doc gate: coverage 6435 entities/214 headers; examples 919 lines verbatim;
  removability clean; briefing current; staleness clean; check-nav 258/258; check-tone 44 pages; first_week OK;
  prepare_site self-test 5/5).
- `python3 tools/suite_run.py green` → `suite run: green — 3752 cases, 3883624 assertions`.
- `src_build_gate.py check` → `src build gate: PASS (all 14 builds …)`; `self-test` → `OK (377 detector cases …)`
  (the first self-test hit MY 3000 s `timeout` — exit 124, no output — with four worktrees' gates sharing the CPU;
  re-run under `timeout 7200`: exit 0).
- ARM header gate (259-line TU in scratch) exit 0; CI greps: PROS-free, sim-free.
- `api_doc_tool.py` self-test / check-coverage / check-fresh / check-examples / check-removability: OK;
  `doc_staleness_audit.py` self-test + run: OK; `briefing_status.py` regenerated before the build (binary removed
  first) and after it, then `check` in the build: current.
- `git diff --stat e9bcee7 -- src/ include/`: empty.

### DOCS3-N1 review findings and their disposition (fix round, 2026-11-15)
Three reviewers, five findings after the overlaps were merged. Scratch for this round: `scratch/docs3-n1/fix1/`.
The reviewers' runner and plants: `scratch/docs3-n1/review-gates/`.

1. **check-tone missed two status openings (major) — REAL, fixed.** Guide 19 :13 ("Nothing in this program has
   run on a robot in this form … Treat everything below as …") and cookbook 06 :11 ("No driver has used either on
   a robot yet."). The brief's four shapes could not see them.
   - Guide 19: both sentences gone. The facts stay: the host test plant, the 34 recorded runs, the calibrator's
     earlier run on robot two (2026-09-24), HA-136. *(Corrected in review round 2: "this program, in this form,
     has not run on a robot" was dropped, not kept — now in guide 19's "One run" and guide 14.)* Cookbook 06: the sentence gone. The limit stays where it
     applies, in guide 14 :650 and guide 10 :302.
   - `STATUS` widened. It now matches "nothing in this chapter/program/page/recipe/section (or here) has (yet) run,
     been run or been tested", "no driver/robot/team/brain/one has used, run, driven, felt, tried or started",
     "nobody has …", and "not (yet) (been) run/tried/tested/used/driven on a/the robot/brain" (a superset of the
     brief's fourth shape). Both real sentences are self-test cases.
   - Two suggested shapes were **not** adopted: bare "never run on a robot" and "not (yet) on a robot". Both fire
     in the register's opening (:83, :175). Those are dated LEDGER records, and history records are indexed, not
     rewritten. No reader page's opening uses either shape. A self-test case pins this ("a dated record is not a
     status line").
   - A wide-net scan of every opening (`fix1/opening_scan_before.out`, 95 sentences that pair robot, brain, driver
     or hardware with a negation, each read) found no other preamble. Guide 12's "reasoned …, not yet seen on a
     field" stays: it says where the hardware symptoms come from. Guide 20's "none of them exists yet" is about
     later features.
   - `how-the-docs-stay-true.md` :12 is now true. In the changelog, "six pages" became "eight pages".
   - **The logged review grep was re-run with the wider `STATUS`** (`fix1/status_elsewhere2.out`): 108 lines.
     The 99 that the old shapes hit are as logged above (103 before the four section lines were fixed). Nine are
     new to the wider shapes, and all are kept: guide 10 :226 (the driver loop part, which states its own limits)
     and :302 (the bindings part), guide 11 :740 (where a reading comes from), guide 14 :637 and :650 (the place
     for them), the register's :1126 and :4595, and the roadmap's :343 and :2069 (ledgers).
2. **The command paths were untested (major) — REAL, fixed.** `check_nav(mkdocs, docs)` and
   `tone_pages`/`check_tone(docs, repo, extra)` now take their roots, with the real ones as defaults. The self-test
   builds a scratch repository (`tempfile`) and asserts the following. A clean tree exits 0 ("5 pages",
   "4 public pages"). A plant in each of `docs/README.md`, `docs/faq.md`, a nested guide page, `README.md` and
   `test/README.md` exits 1 and names `file:line`. A plant in `docs/api/` or `docs/internal/` is not read. An
   orphan page at the top level, nested, or in `api/` exits 1 and is named. An internal page needs no menu line.
   A menu the tool cannot read exits 1. G1–G5 are now RED.
3. **The register's anchors were checked only on a 3-row sample (minor) — REAL, fixed.** The copy's per-page
   step is now one function, `prepare_page()`. The new **`prepare_site.py check-register`** (in the doc gate) runs
   the real register through it. It compares the result with a count made another way (`register_rows`: lines
   whose first cell is an HA id) and prints "230 register rows, each anchored once". The copy itself now fails
   when no register row was anchored, which closes G20's silent "0 anchored". Nine new self-test cases. Docs moved
   in step: the briefing's sentence, one row of how-the-docs-stay-true's table, and the doc-gate comment (already
   true).
4. **No gate on the working branch runs the strict site build (minor) — REAL, not N1's to fix.** [K3]: N2 owns
   the one site-build script, and RMP1-A1 rewrites `ci.yml`/`pages.yml`. **For the coordinator:** until N2 lands,
   run the strict build by hand at every merge into `shulib-v2`, starting with ROB1-BT, RMP1-A1 and AF1S-A2. Use
   CI's pins (mkdocs 1.6.1, mkdocs-material 9.5.44) and never `-q`. This round's build: exit 0, no WARNING.
5. **Eleven smaller surviving mutants (minor) — REAL, fixed.** Each one now has its own self-test case:
   "hasn't" (both apostrophes), "Nothing here has run", "has not yet run", "has not been tested", "_never_
   driven", a `~~~` block holding "## ", a quote split so that it matches only when joined, a trailing empty
   section, a top-level key after `nav:`, a title with a space before its ':', and an unquoted line with a
   trailing comment. G13 (equivalent in effect) and G11 (only the first word on a line named) now have cases too.
   The self-test now has 66 cases. A parse failure inside it counts as a failed case, not a crash.

**Mutations, fix round** (`fix1/mutate_fix.py`, `fix1/mutations_fix.out`). Each runs as a fresh process for every
gate command (doc_readability self-test, check-nav, check-tone; first_week check; prepare_site self-test and
check-register), under `ulimit -v 4194304; timeout 600` with `TMPDIR` in scratch. Each file is restored in a
`finally` from `fix1/mut_backups/`, its mtime is bumped, and the restore is asserted.
- The reviewers' G1–G20, re-aimed at the fixed code, plus G3b (check-nav's glob): **all RED.**
- New: N1–N9 and N12 (each new `STATUS` branch, `TONE_EXEMPT`, `UNPUBLISHED`, check-register's three exits and
  its count) **RED**. D1 and D2 (guide 19's and cookbook 06's old openings put back) **RED** at check-tone. D3 (a
  register row duplicated) **RED** at check-register.
- **Survived, each with its reason:**
  - G21 (the "tree's own lists agree" case forced to `[]`): a check of real data, equivalent on a correct tree.
    The synthetic list cases hold its logic.
  - N10 (the copy's new "no row anchored" guard removed): the doc gate never runs the copy, which only pages.yml
    does. The same cause, the register renamed or gone, fails check-register (G20: RED).
  - N11 (check-register's dispatch line removed): the gate command then copies the site into a folder named
    `check-register`, and that copy still runs `prepare_site.py`'s register guards. It loses only the second count.
    The runner's copy landed in the worktree root and was removed. The copy's command line is pages.yml's (N2,
    RMP1-A1).
  - D3's first form (a no-break space before the closing bar) is equivalent: both patterns still read the row,
    and the anchor works. It was replaced by the duplicated row.

**Gates, fix round** (worktree, after the last edit):
- `rm -f build/test/shulib_tests`, then the briefing was generated, the build ran, and the briefing was generated
  again. `cmake --build build/test -j8` exited 0. The doc gate printed: coverage 6435/214, examples 919 verbatim,
  removability clean, self-test 66 cases, check-nav 258, check-tone 44, first_week OK, prepare_site self-test 14/14,
  check-register 230.
- `api_doc_tool.py` self-test, check-coverage, check-fresh, check-examples and check-removability: OK.
  `doc_staleness_audit.py` self-test and run: clean (35 live docs). `briefing_status.py check`: current.
- `python3 tools/suite_run.py green`, run twice: `green — 3752 cases, 3883624 assertions`. The briefing was then
  regenerated, and `check` reports it current.
- A note for the coordinator, not caused by N1: with `TMPDIR` set to a long path (this scratch folder), two
  `ProsFileStore` cases fail (`test/pros_file_store_adapter_test.cpp` :181, :184, :229). The adapter's path buffer
  cuts the path short (`…/shulib_safesave_file` vs `…/shulib_safesave_filestore/shulib_drivetrain.bin`). So the
  suite runs here used the default temp folder. Only the doc tools' self-tests used the scratch `TMPDIR`.
- `src_build_gate.py check`: PASS (all 14 builds). `self-test`: OK (377 detector cases). The tree was restored
  byte-exact after its plants; the build and the suite above ran after it finished.
- ARM header gate (259-line TU, `fix1/all.cpp`): exit 0. CI greps: PROS-free, sim-free. `git diff --stat e9bcee7
  -- src/ include/`: empty.
- Strict site build (the scratch venv, CI's pins, no `-q`): `prepare_site.py` gave 262 files and 230 rows
  anchored; `mkdocs build --strict` exited 0, with no WARNING and no "not included in the nav". The built HTML
  holds 230 `ha-` ids and no internal path.
- The front page and the menu did not change in this round, so the screenshots stand.

**Prose review, fix round** (`fix1/prose_review_greps.out`, run and read before this entry):
- `check-register`: the briefing :263, how-the-docs-stay-true :13 and the doc gate. All are new and true.
- `six pages|six openings|five openings`: no hits left outside the brief.
- Guide 19's removed words ("in this form", "something a robot has shown") remain only in ledgers (changelog
  :587, :1157), internal records, and `include/` comments with their generated `docs/api/` pages (frozen; exempt).
- No link points into either page's opening. Cookbook 06's three anchors are its own H2s, and they are unchanged.

### DOCS3-N1 review findings and their disposition (round 2, 2026-11-16)
Three reviewers, 22 findings (two pairs overlapped: the register's citation, the removed anchors). Each was
re-checked before any fix. Scratch: `scratch/docs3-n1/fix2/`.

**Fixed in the tools**
1. **`_honest_` passed check-tone (major) — REAL.** `\b` counts `_` as a letter. `HONEST` now uses letter
   lookarounds (it still skips "dishonest"). Master plan :747 `_Honest caveat:_` → `_Caveat:_`. Cases: `_honest_`,
   `_Honest caveat:_`, `__honestly__`, `*honesty*`, `snake_honest_case`; "dishonest" passes.
2. **Guide 20's opening; STATUS too narrow (major) — REAL.** Guide 20 :16's "No AI Vision sensor has run this
   code." is gone (guide 14 :421 keeps it). STATUS is now six named shapes (`STATUS_SHAPES`): "has not run",
   "never driven", "nothing has run" (nothing / none / neither / nobody / "no" + 1–3 words, then has or have, yet or
   ever, been, run / driven / tested / used / tried), "yet to run", "not run on a robot" (+ hardware), "untested".
   Every reviewer plant is caught. Scan of every opening (`fix2/shapes_scan.py`): three hits — guide 20, the
   register's opening, `src/team/README.md` :9 (see 11). A looser rule (a negation and a verb in one sentence,
   `fix2/open_scan.py`) gave 23 sentences, 20 of them false, so it was not used.
   - **The register's opening (a LEDGER) rewritten, as the master plan's was (delta 4):** "No *v2* robot exists …
     no control loop has ever closed and nothing has driven" was stale (both robots exist; a driver drove robot two
     on 2026-09-17) → "What has run on a robot, and what has not, is listed in What it cannot do yet". Guide 14
     keeps the 2026-08-13 adapter fact. **For the coordinator:** the reviewer left this to you; the wider check
     needs it.
   - Changelog: "eight pages" → "ten pages" (guide 20 and the register).
5. **STATUS alternatives untested (major) — REAL.** `WITNESSES`: one sentence per word of every shape (41). Each
   must match its own shape and no other, and must fail in an opening. Mutants S1a–S6: all 48 RED.
6. **`first_week.py` had no self-test (major) — REAL.** New `self-test`, 10 cases on a scratch guide and picture:
   names and slots, a later section not read, write-then-check passes; a renamed step, a stale or missing picture,
   a step with no bold name, no heading, no numbered step and a missing guide each fail. `check()` and `write()`
   take their paths. It runs in the doc gate before `check`. F1–F7 RED.
7. **Opening detection (minor) — REAL.** Cases: an H3 does not end the opening; a page with no H2 is read whole;
   an indented fence holding "## " (R15 too). R11, R15, R16 RED.
8. **check-nav's YAML-refused lines (minor) — REAL.** Cases: each reserved first character (`{ [ ! & * | > % @`
   and the backtick); a quoted title not followed by ':'; a quoted title holding ' #' is read. R12–R14 RED.
9. **The doc gate missed new folders (minor) — REAL.** `test/CMakeLists.txt` now globs every `docs/**/*.md` but
   `internal/` (GLOB_RECURSE, CONFIGURE_DEPENDS). Through-build probe E1, the reviewer's: a registered
   `docs/start/01-hello.md` → build exit 0; a plant in it → exit 2 at the doc gate, check-tone naming it; restored
   byte-exact → exit 0 (`fix2/e1a.out`, `e1b.out`, `e1c.out`).
10. **check-register's second count was not independent (minor) — REAL.** `register_rows` is now wider than
    `HA_ROW`: any table line (spaces before the bar allowed) whose first cell holds an HA id. Real register: both
    230. Cases: a bold id and a leading-space row fail; a row anchored with another row's id fails (this catches
    P1). P1, P3b, P3c RED.
12. **A self-test could fail and exit 0 (minor) — REAL.** In all three tools, `self-test` first runs itself with
    one planted failing case (output hidden) and fails unless that run exits 1. R17, P2, F5 RED.

**Fixed in the pages**
3/14. **Removed anchors (minor, twice) — REAL.** Every id the three landing rewrites removed is back. The built
   site has all 30 (Playwright, `fix2/shots2.py`). `git grep` for each, internal included: no inbound link.

   | Page | Old ids | Now |
   |---|---|---|
   | front page | `shulib-documentation`; the six task ids, `look-something-up`, `reference-material`; `how-these-documents-stay-true` | invisible `<a id>`: the first paragraph; the matching table row ("Understand an error", "Maintain the library" for the two); the first bullet of "How these pages stay true" |
   | guide README | `the-shulib-guide`; `the-other-two-documents` | the first paragraph; the "Which kind of page" table |
   | guide README | `maintenance-read-before-editing-the-guide` and its five subsections | a VISIBLE "**Maintenance** … moved to How these pages stay true" line ([K4] (2), a split); the new page keeps the five headings, so their ids live there too |
   | cookbook README | `the-shulib-cookbook`, `how-to-use-a-recipe`; `the-recipes`, `1--getting-there` … `9--one-route-two-robots`; `the-three-things-every-recipe-assumes`; `what-is-not-in-here` | the paragraphs; each recipe group's table; item 1 of "What every recipe assumes"; "Not in here" |
   | eight "honest" headings (guides 01, 04, 06, 09, 18, VexBuilder ×2, README) | — | not kept (delta 6: the id holds the word); `git grep`: no link left |
4/16. **The register cited "the fidelity boundary" (minor, twice) — REAL.** :2710, :5705, :5724 now cite by place
   ("the boundary note that opens `motor_model.hpp`, :7–17").
13. **Guide 19's and guide 21's limits were dropped (major) — REAL.** Guide 19's "One run" now says "This program,
    in this form, has not yet run on a robot. Its calibrator ran on robot two in an earlier form on 2026-09-24",
    with a link to guide 14. Guide 14's first section gains two bullets: the standard Calibrate, and robot one (no
    census, reading or photo; placeholder numbers on the host test plant). Guide 21's "What robot one does not do
    yet" opens with the robot-one fact. The fix-round log line is corrected.
15. **"words kept" was untrue (minor) — REAL.** R2-δ2 below; the log line is corrected.
16. **how-the-docs-stay-true.md (minor) — REAL in part.** Fixed: the anti-rot paragraph (guide listings live in
    15 `*example*_test.cpp` files and the team files, not one file), "Where this applies" (+ `test/README.md`), and
    a table row for `first_week.py check`. **Left for N2:** :83's ISO-dated example and :88-89's "bare chunk ids …
    fine". They copy guide-maintenance.md rules 2–3, which N2 owns; changing one copy now makes two (landmine 5).
    **For the coordinator:** add `how-the-docs-stay-true.md` to N2's files.
17. **Cookbook README, "quoted from its test" (minor) — REAL.** Now "quoted word for word from the compiled file
    it runs in — a test, or the team's own driver file".
18. **The front page did not link guide 14 (minor) — REAL.** "What it cannot do yet" joins the "Understand an
    error or a log line" row, in the Troubleshoot tab's order (R2-δ1).
19. **HANDOFF's old list (minor) — REAL.** The S1 sentence now points at guide-maintenance's R3 row.
20. **Screenshots (minor) — REAL.** `fix2/shots/`: the front page, How it works, Recipes, How these pages stay
    true and the glossary, each at 1280 and 400 px, plus the drawer at 400. No page scrolls sideways (scrollWidth =
    the viewport at both widths). The tabs and the drawer show the eight, in order.
21. **Two unlinked "chapter N" (minor) — REAL.** `README.md` :68 and `test/README.md` :179 are links now.

**For the coordinator (not N1's to change)**
11. **`src/team/README.md` :9 opens with "neither has run on a robot yet".** STATUS catches it now (a self-test
    case quotes it). N1 may not touch `src/`; the run that rewrites that opening adds the file to `TONE_EXTRA`.
22. **Main moved (3c29b73: ROB1-C2 and the Tune label fix).** At the merge: keep N1's removal of guide 21's
    opening and of its "(Chapter 20 is held …)" line. ROB1-C2's new opening ("Nothing in this chapter has run on
    robot one yet") fails check-tone; its facts (Stage A typed 2026-11-15; no census, reading or photo) update the
    robot-one bullets in guide 21 and guide 14, which N1 wrote with `e9bcee7`'s facts. Redraw the picture
    (`first_week.py`; step 3 was renamed), link guide 18 :721, guide 21 :22 and the facts form :8 and :65, then run
    every N1 gate.

**Found in this round, not by a reviewer:** round 1's sweep left "an a MOTION_TIMEOUT" in the register (:3647,
from "an honest MOTION_TIMEOUT"). Fixed. A check of the ten base sites where "a" or "an" came before the word, and
a scan of the sweep's diff (`fix2/article_scan.py`), found no other.

**Deltas, round 2**
- R2-δ1: the front page gains one link (guide 14) in an existing row. §A.2 allows tightening, and the sweep moved
  every limit to guide 14.
- R2-δ2: round 1 rewrote the moved Maintenance text in two places. "How it's organised" lost its chunk-history
  aside and the stale "chapter 20 is held for the robot panel" line, and gained the rule "a new page takes the
  next number free in every tree; check-nav holds the menu". "Voice" swapped "admit what's unverified — it has
  never driven a robot …, and the guide says so wherever it matters" for the tone rule. Both follow D15.
- R2-δ3: the register's opening rewritten (2).
- R2-δ4: STATUS is six named shapes with a witness table, not one pattern.

**Mutations, round 2** (`fix2/mutate2.py`; results in `mutations2.out` and `mutations2_rerun.out`). Each runs 7
commands as fresh processes (doc_readability self-test, check-nav, check-tone; first_week self-test, check;
prepare_site self-test, check-register) under `ulimit -v 4194304; timeout 600`. Each file is restored in a
`finally` from `fix2/mut_backups/`, its mtime bumped and the restore asserted; all 7 files cmp-identical after.
- RED: H1; S1a–S1l, S2, S3a–S3r, S4a–S4d, S5a–S5l, S6 (48); R11–R19; F1–F7; P1, P2, P3b, P3c; D1–D4 (guide
  20's, the register's and guide 21's openings, and the master plan's `_Honest caveat:_`, put back → check-tone).
- Survived, equivalent: R20, F8, P4 (the planted-failure guard removed; it acts only when the exit is broken).
  P3a's target (an `lstrip()` with no effect) was removed from the code.
- Through the build: E1 (9).

**Prose review, round 2** (`fix2/prose_review_greps.out`, read before this entry): `first_week` → the briefing
:261 (updated: the self-test, and each self-test checks its exit) and the doc gate; `check-register|register_rows`
→ the briefing :262, true; `eight|nine|ten pages` → the changelog only; `fidelity|honesty boundary` → no public
hit (internal history only); "every code listing in the guide" → only how-the-docs-stay-true (fixed); "No *v2*
robot exists" → T3-PROGRESS :2213 (history); the reading checks in guide-maintenance :98, docs-publishing :134
and the briefing → true.

**Round 2, second pass (2026-11-16): re-checked, gates run.** The workflow sent the same 22 findings again, and
the round-2 entry above had no gates. Scratch: `scratch/docs3-n1/fix3/`. Each finding was re-checked in the tree,
not taken from this log:
- **Tools (1, 2, 5, 7, 8, 11):** every reviewer sentence and plant through the tool (`fix3/probe_sentences.py`):
  24 sentences, 2 openings and 3 menu lines, 0 missed; the two sentences that must pass, pass.
- **Pages (2, 3/14, 4/16, 13, 15–21):** each read in the diff against `e9bcee7`. True as logged.
- **Removed ids, every public page** (`fix3/removed_ids.py`, the site's own slugify): 7 ids are gone, all of
  them "honest" headings (delta 6). Every other old id is still served. The two old links to them
  (guide 16 → guide 04, guide 14 → `README.md`) are still in other worktrees' copies of those lines, RPV1-S1,
  RMP1-A1 and AF1S-A2 among them. Either one, back after a merge, fails check-tone, because the id holds the word.
- **One fix:** the register :5680 "exactly today's stated state" (round 1's word for "honest") → "exactly
  today's state". The sweep's word diff (`fix2/honest_wdiff.out`) was read again; no other replacement reads badly.
- **Mutations again** (`fix3/mutate3.py` = round 2's runner; `fix3/mutations3.out`): 76 mutants, 73 RED;
  R20, F8 and P4 survive (equivalent, as above); the tree green after.
- The front page and the menu did not change in this pass, so `fix2/shots/` stands.

**Gates, round 2** (worktree, after the last page edit; `fix3/gates.sh`, `fix3/gates.out`, one file per gate):
- `src_build_gate.py self-test`: OK (377 detector cases), then `check`: PASS (all 14 builds); `src/`, `include/`
  and `Makefile` clean after. Everything below ran after it.
- ARM header gate (258 headers, `fix3/all_headers.cpp`): exit 0. CI greps: PROS-free, sim-free.
- `rm -f build/test/shulib_tests`; briefing generated; `cmake --build build/test -j8` exit 0, no warning; the doc
  gate printed coverage 6435/214, examples 919 verbatim, removability clean, staleness clean, self-test 159 cases,
  check-nav 258, check-tone 44, first_week self-test 10 + check OK, prepare_site self-test + check-register 230.
  Briefing generated again (binary fresh, under the cap); the rebuild exit 0.
- `python3 tools/suite_run.py green` (under the cap): `green — 3752 cases, 3883624 assertions`. Compile refusals:
  289 ctests for 289 TUs, all pass.
- Each run by hand as well, exit 0: `api_doc_tool.py` self-test, check-coverage, check-fresh, check-examples,
  check-removability; `briefing_status.py` self-test, check (current); `doc_staleness_audit.py` self-test and
  run; the three reading commands, both `first_week.py` commands, both `prepare_site.py` commands.
- Strict site build (scratch venv, mkdocs 1.6.1 + mkdocs-material 9.5.44, no `-q`): 262 files, 230 rows anchored;
  `mkdocs build --strict` exit 0, no WARNING, no "not included in the nav"; 230 `ha-` ids in the built register;
  no internal path in the HTML; the old landing ids checked in the HTML are all there.
- `git diff --stat e9bcee7 -- src/ include/`: empty.

**Prose review, this pass** (`fix3/prose_review_greps.out`, read before this entry): "stated state" → only a test
comment (`test/pros_digital_adapter_test.cpp` :32, unrelated, not N1's); HA-56's other mentions (diagnostics plan
:162, register :291, :370) do not use the changed words.

## DOCS3-P1a

*(not started)*

## DOCS3-N2

*(not started)*

## DOCS3-A1

*(not started)*

## DOCS3-P1b-1

*(not started)*

## DOCS3-P1b-2

*(not started)*

## DOCS3-P1c-1

*(not started)*

## DOCS3-P1c-2

*(not started)*

## DOCS3-A2

*(not started)*

## DOCS3-P3a

*(not started)*

## DOCS3-P3b

*(not started)*

## DOCS3-P3c

*(not started)*

## DOCS3-N1 — the coordinator's merge verification *(2026-11-17)*

- **Squash-merged from `docs3-n1`** (transport `578248a`, base `e9bcee7`) onto `e8fa3de` in the coordinator's INTEGRATE
  worktree (ROB1-C2, RPV1-S1 and RMP1-A1 landed since). Conflicts resolved: the guide index (N1's menu-labelled table, plus
  chapter 22 → "Route builder"), guide 14 (RPV1-S1's robot-one preview words kept, N1's two limits added), guide 21's
  opening (N1's tone fix, ROB1-C2's typed facts), the changelog (N1's entry at the top of `## API 2.9`), the briefing
  (regenerated), and `mkdocs.yml` (N1's task menu, plus a **Route builder** tab: `route-builder.md` and guide 22, per K2:
  the run that merges second places the other's pages).
- **Merge steps the run asked for:** `chapter_links.py` re-run on the merged text (3 new "Chapter N" links; a second run
  links 0); `first_week.py` redrew the picture (guide 21's order had changed with ROB1-C2) and its check passes.
- **Gates** (`integrate_verify.sh docs3n1`): suite green (**3,778 cases**); src gate self-test OK and check PASS (14
  builds); ARM header gate; doc gates incl. **check-nav: 260 public pages, each in the menu once**, check-tone (46 pages),
  first_week; briefing current. **The strict site build, as pages.yml runs it** (prepare_site → the route builder built
  into site_src → `mkdocs build --strict`, CI's pins, no `-q`): exit 0, no warnings; **PG1: site/route-builder/app.html
  present.**
- **The coordinator's mutations:** MD1 guide 22 dropped from the menu → check-nav RED; MD2 "honest" planted in a page →
  check-tone RED. Restored, both green.
- **DOCS3-N1 LANDED.** Next: a docs release (the route builder + the menu).

## main.cpp's explanations move to the docs — the coordinator *(2026-11-17, at the team lead's word)*

- **The team lead, 2026-11-17:** "the comments are helpful so i dont want to get rid of them" but "the main.cpp file's
  comments should just like go in the docs so that people can know what to put in there but that their personal ones
  arent obscured". The first piece of READ1 (team-facing files: a short header, the code, the why in the docs).
- **`docs/where-your-code-goes.md` (new, menu: Robot → Set up):** the folders (what is in each, when to change it), the
  six programs (build, name on the brain, slot, program file, the team's and robot's files each uses), what to change
  where, what `src/main.cpp` does, and adding a program (the enum and chain, the five switches, the gate's table). Linked
  from the front page's "Drive the robot" row and guide 07's `src/main.cpp` sentence.
- **Its picture, `docs/assets/build-to-program.svg`,** drawn by `tools/docs_figures/build_to_program.py` FROM the page's
  two tables (the Auton Tester row; build_selection's and main.cpp's "What is in it" words) — `check` and a 12-case
  `self-test` (a planted failing case must exit 1) run in the suite's doc gates beside first_week's, listed in "How these
  pages stay true".
- **`src/main.cpp`: 316 → 183 lines, the code byte-for-byte the same with comments stripped** (checked with the gate's own
  `_strip_comments`, whitespace folded). Kept: a 3-line header pointing at the page, and one-liners where a line is not
  obvious (the precondition handler first; never a routine in `autonomous()`). Dropped, not moved: the "what has run" status
  block (stale: robot one has run library motion since 2026-11-17) and the build-history notes (the chunk records hold
  them). The gate's main.cpp plants (cases 18–22) match code lines, untouched.

## Set up your robot first — guide 23 (2026-11-19, the team lead's ask)

- **The team lead, 2026-11-19:** "please make a tutorial for the calibrate and bench test app usage so that people make
  sure to do that before using the route builder". For the builders, before Saturday Nov 21.
- **`docs/guide/23-set-up-your-robot-first.md` (new, menu: Robot → Set up, first):** what you need; 1 upload Bench Tests
  (slot 3) and Calibrate (slot 4) — `make ROBOT=one` / `ROBOT=tank` and the `pros upload` lines; 2 Bench Tests — tiles 1,
  4, 5, 2, then station 13 (its controls, each mechanism's direction check, the cascade's bottom-stop / level / top-stop
  RECORDs and RECORD's big `output … deg` line, the side toggles, the pistons, every self-stop line and its one action)
  and a "what to send" table keyed to the facts form's items; 3 Calibrate — one run (1/4 IMU, 2/4 PUSH 48 in front
  first then DONE, 3/4 PULSES finding the sides, 4/4 SPIN, SAVED, RUN AGAIN for a second agreeing run), FRONT FLIPPED,
  MECHANISM PORT TURNED, every STOPPED word with its do-line (from `calibrate_panel.hpp`'s `stoppedWords`), CANNOT START,
  NOT SAVED; 4 check in shulib Drive (slot 1), with the controller map picture; 5 the route builder; "If something goes
  wrong". **Written to tonight's programs:** CAL2's Calibrate (the pulses that find the sides, FINDING LEFT, SIDES
  UNCLEAR, the follower SIGN MISMATCH) and S13UX's station 13 (the "what to do now" line, the 3 s cut re-armed by L1
  released and held, the fault cuts' DONE-then-reopen, the cap applied and shown on the bottom line, RECORD's big
  line, the lift's numbered steps) — the page must land after both.
- **Its picture, `docs/assets/setup-first.svg`,** drawn by `tools/docs_figures/setup_first.py` FROM the page's own
  `## N. Title` steps and the one-line italic caption under each (numbered in order; a title or caption too long for its
  box refused) — `check` and a 15-case `self-test` (a planted failing case must exit 1; each scratch case seen failing)
  in the suite's doc gates beside build_to_program's (COMMAND lines and DEPENDS), listed in "How these pages stay true".
  Rendered once in Chromium and looked at: five boxes, numbered circles, the last (the route builder) green.
- **"Do this first"** lines at the top of `docs/route-builder.md` and guide 22; the front page's "Set up a new robot" row
  leads with it; guide/README.md row 23.
- **Checks (in the worktree, capped):** doc_readability check-nav / check-tone / self-test OK; setup_first self-test and
  check OK; first_week check OK; build_to_program check OK; api_doc_tool check-removability OK; doc_staleness_audit
  clean.
