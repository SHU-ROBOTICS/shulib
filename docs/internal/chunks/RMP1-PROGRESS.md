# RMP1 — PROGRESS (live log)

Brief: [`RMP1-route-maker-page.md`](RMP1-route-maker-page.md) (the coordinator's copy of record, placed uncommitted in
this worktree; it lands with RMP1-A1). Executor model: `opus`. Nothing committed and nothing pushed by the executor.
Host and browser only: nothing here has run on a robot; every run of the library is the host test plant.

## RMP1-A1

**Base:** worktree `.claude/worktrees/rmp1-a1`, branch `rmp1-a1`, at `49822e5` (main: API 2.8, ROB1-A, the homing fix).
Started 2026-11-14.

**Order of authority read (the team lead's 2026-10-22 ruling on what to read):** the header; `## ANSWERS` (2026-11-14:
"Go, accept all" — D1–D20 as recommended; D15 (a); D17 a docs release at each route-builder merge; D19 the builder stays
open to the 21st; D20 the COORDINATOR pushes `rmp1-a1`); `## Amendment 2026-11-13 (late)` A.1–A.12 in full (A.12's
W1–W16 override A.1–A.11); `## Re-check — amendments` R1–R19 in full; the Decisions D1–D20 in full; §0.2 (what exists);
§1.1 (as amended) and §1.4 (the interaction model) in full; §2.2–§2.5 in full; §3, §4, §5, §6 (coordination, the §6.3
contract, §6.4) and §7, §8 (landmines 1–30) in full; §1.2, §1.3, §0.3, §0.4, §2.1 and §9 by heading and skim.

### Log
- Created this log first. Read `docs/internal/RESUMING.md`'s standing rules.
- **§2.5.1 the screenshot path, FIRST (A.5's recipe):** Playwright 1.64.1 pinned in `~/shulib-render-tmp/rmp1-a1/pw/`
  (`npm install`, 2 packages), its OWN Firefox 157 from the shared `PLAYWRIGHT_BROWSERS_PATH=~/shulib-render-tmp/pw-browsers`,
  `NODE_OPTIONS=--disable-wasm-trap-handler`, `TMPDIR=~/shulib-render-tmp/rmp1-a1/tmp`, under `(ulimit -v 4194304; timeout 120 …)`:
  a test page captured (`shots/proof.png`, 11 KB) in seconds; `browser.close()` in a `finally`; `ps` after it shows no
  firefox / geckodriver / playwright process. The snap Firefox and geckodriver are not used.
- **The model (pure ES modules, `tools/route_maker/src/model/`):** `numbers.mjs` (W7: round at edit time to 3
  decimals, ONE formatter), `geometry.mjs`, `curve.mjs` (a line-for-line port of `motion/curve_path.hpp`'s
  CurvePath + `checkCurve` + `appendCurveCheckWords`; float arc tables via `Math.fround`), `route.mjs` (the step
  kinds, pure edits, `plannedPoses`), `checks.mjs` (RC1–RC3, RC15–RC21, RC24–RC26, path walk for RC17/RC25 on
  every robot's list), `export.mjs` (D4's region; GL0 = §6.3 byte for byte — extracted from the brief's own
  lines 2156–2181 into `test/goldens/route_maker/gl0_empty.cpp`, exporter output equal on the first run),
  `region.mjs` (W12), `project.mjs`, `undo.mjs`, `field.mjs`, `library_words.mjs`.
- **The C++ side (host only, `test/`):** `route_maker_describe.hpp` (one robot's description from its REAL table;
  the declaration's C++ name read from the `.name` line `std::source_location` recorded — no path written),
  `route_maker_robots.hpp` (one, two, x, y, z), `route_maker_curves.hpp` (CV1: 200 seeded curves — 105 Ok,
  25 StartBehind, 5 TightBend, 25 TooClose, 24 NoDirection, 8 AtRobot, 8 DoublesBack), `route_maker_main.cpp`
  (`shulib_route_maker describe|curves`; CMake target added at the END of `test/CMakeLists.txt`),
  `route_maker_rig.hpp` (robot x / one / two stacks at a FIELD start), `route_maker_golden_test.cpp`.
  First full build: `-Werror` clean. `shulib_tests -tc="RMP1*"`: 13 cases, 113 assertions, green after two
  golden corrections, both RECORDED here as found: (1) gl3b's first 16-point wave (amplitude 5 in, k/2) timed
  out on the host test plant inside a 5.6 in bend — a fixture choice, replaced by a gentler wave (3 in, k/4);
  (2) the hood's END state reads "down" because the guard safes it after the run — the test now reads the
  service's `'hood': SUCCEEDED` line instead of the lines' end state. GL1b (the 10° leg not turned first)
  stopped at step 1 MotionFailed, as re-check P8 measured.
- **Node tests (the model, `tools/route_maker/test/*.test.mjs`): 61 pass** — numbers (NF1), curve (CV1: all 200 curves'
  verdict, why, point and the library's sentence EXACT on the first run, positions < 0.001 in), checks (RC1–RC3,
  RC15–RC21, RC24–RC26, each with its edge), export (GL0, goldens fresh, UN1, UN3, order, turn-first), region (FA1–FA4
  direct), host (FA1–FA4 through `files.mjs` over the in-memory host), project (RT1–RT5, undo), library words (every
  quoted sentence found in its header), field (FP1, FP2, snapping). Node 20 runs EVERY `.mjs` under a `test/` folder
  given as a directory (measured in scratch `nodedir/`), so the goldens writer moved to `tools/route_maker/goldens.mjs`
  (`--write`), the helpers export only, and the page tests skip unless `ROUTE_MAKER_BROWSER` is set.
- **The field picture** (`tools/docs_figures/route_maker_field.py` → `tools/route_maker/assets/field.webp` 68 KB +
  `field.json`): the transform is MEASURED (the perimeter rail's inner edge on all four walls: 7.624 px/in, origin
  (570, 573.5)) — `field_coordinates.py`'s hand 7.5 px/in put the red wall ~1.5 in off, past FP1's 1 in. The centre
  goal's dark body (found independently) maps to (0.06, 0.19) in. FP1's "Toggle places" are the C++'s own definition
  (`TogglePlace`: the wall's inner face at its centre); the yellow Toggle bars in this render sit ~5 in OUTSIDE the
  wall, so they are not the landmark.
- **The page** (`src/app.html`, `style.css`, `browser_check.js`, `ui/*.mjs`, `host/*.mjs`) and **`build.py`** (SITE form
  `app.html` + `src/`, ONE FILE `shulib-route-builder.html` with the modules bundled — each module in its own function
  scope, so no name can collide — robots baked from `shulib_route_maker describe`; RG1 and the no-URL check on every
  output file and inside every decoded data: URI).
- **Page tests in Playwright's Firefox 157 (A.5's recipe), 6 pass:** WG5 (both exact policies, `data:` favicon,
  marker), WG6 (no request leaves either form), a route built by clicking (start, drive with turn-first ON by default,
  curve, cascade level → RC25 → "Home it first" with an EMPTY time limit → typed → no error; undo/redo; the export
  drawer's code; written into an in-memory CRLF folder: a non-builder region refused until "Overwrite", nothing
  outside it moved, CRLF kept; robot two's line found), the ONE FILE over `file://` (RC1 through the wall, PREVIEW
  disabled with its words, Export refused while an error stands), Save → `route_maker.json` download that reads back,
  the old-browser refusal (Firefox with WebAssembly off), and the §2.5.2 screens. Bugs the page tests found and fixed:
  a select-mode click on an empty field panned instead of placing the start; `replaceChildren` does not flatten arrays
  (the inspector printed "[object HTMLDivElement]"); a missing level read "no preset 'undefined'".
  No browser or driver process left (`ps` after each run).
- **CI** (`.github/workflows/ci.yml`, appended at the end — two jobs, A.4): `route-builder` (ubuntu-24.04: node 22,
  `node --test tools/route_maker/test/`, `shulib_route_maker` built alone, `build.py`, the artifact) and
  `route-builder-browsers` (needs it; ubuntu-24.04 chromium + firefox, ubuntu-24.04-arm chromium + firefox,
  windows-latest msedge + chrome + firefox + Playwright's pinned chromium; CR1 first; the model's node tests on each
  OS; `npm ci`; the page tests; the screenshots uploaded per row). **NOT RUN: a run needs a push, the
  coordinator's (D20).** `pages.yml`: both jobs pinned `ubuntu-24.04` (W14), the paths filter, the describe build
  and `build.py --out site_src/route-builder`, and PG1 after mkdocs (app.html, the one file, the marker, the docs
  page's link, RG1). The whole build job was run LOCALLY with the pinned mkdocs 1.6.1 + material 9.5.44 in a
  throwaway venv (`~/shulib-render-tmp/rmp1-a1/venv`; scratch `pages_local.sh`): `mkdocs build --strict` clean,
  PG1 and the internal-docs guard pass; `site/`, `site_src/` removed after.
- **Mutations** (scratch `mutate.py`: one exact change, the file backed up under scratch `backup/`, restored in a
  `finally` and on SIGTERM/SIGHUP, the restored file touched; every source and golden md5-checked against a manifest
  after each batch — all restored). Every one RED:
  RM15 (RC1 without the square's half-width) → RC1/RC2/RC3 red · RM21 (a running startHome counted finished) →
  RC25 red · RM19 (the path walk ignores skips) → RC17 and RC25 red · RM23 (curve.mjs without the 90° chord rule) →
  CV1 red · RM30 (LF written into a CRLF file) → FA3 red (direct AND through the host) · RM34′ (the export prints a
  4th decimal; `route_lines.mjs` is A2e's) → NF1 red · RM7 (save drops onFail) → **SURVIVED at first**: RT1
  compared a RE-SAVED golden with itself (idempotence, not fidelity) — RT1 now reads each file's own bytes; re-run:
  RT1 red · RM16 (a newer schema loaded) → RT4 red · RM6 (the transform off by one tile) → FP1 red · RM24 (the turn
  written AFTER the drive; goldens re-made, the suite rebuilt — the golden TU recompiled, GL1/GL2 fail by
  BEHAVIOUR: the legs time out off their lines) → GL1 + GL2 red · RM10 (the wait as `r.pause`; re-made, rebuilt) →
  GL13 red (`floorFired`: the frozen pause ran past the 59 s hard stop) · RM26 (a turn cap through `_deg`) → UN1 red
  (node), and, re-run against the C++ alone after GL19's drive got a 270 deg/s cap (a 90 deg/s cap does not wrap,
  so GL19 could not see it — fixed): GL19 red, the library refusing `MotionOptions: maxAngularSpeed must be finite
  and >= 0` (the test case threw — the library's own refusal is the oracle) · RM29 (the ONE FILE's policy without
  `'wasm-unsafe-eval'`) → WG5 red · RM33 (the SITE policy without `'self'`) → WG5 AND the SITE-form clicking test
  red (the app's module blocked: the page never became ready) · RM36 (the app written as `index.html`, build.py's
  own guard removed) → `mkdocs build --strict` red first (the docs page's link to app.html), and with strict off,
  PG1 red ("site/route-builder/app.html is missing") · RMU1 (Export allowed while a step has an error) → the ONE
  FILE page test red.
- **Prose review (rule 4), the brief's §3 greps, RUN and read:** `simulat` 0 hits; `honest` 1 — the developer
  README's heading, renamed; `obviously` 0; `just ` 3, all adverbs in code comments ("just past", "just after",
  "what just happened"); `simply` 0; `easy` 0; `precondition` 3, all in code comments naming the rule (never in the
  page's words — the page quotes the library's sentence in its "library:" line only); `radian` 4, all in model
  comments (UN2: the page shows degrees); `http` in the built page: one, `http://www.w3.org/2000/svg` (the SVG
  namespace, in the ONE FILE's bundled dom.mjs). The doc staleness audit flagged five lines naming
  `src/team/route_maker.json` and `src/team/skills_route.cpp` (files a student's save and robot two's update
  create): reworded so each says what WRITES or ADDS them (its rule) — clean.
- **Deltas from the brief (each with its reason):**
  1. **Not in A1, by A.7:** triggers, the wall reset, the mirror view, the time bar (D12, `timeline.mjs`), the
     Toggle steps, "who runs it" — and with them RC4–RC14, RC22, RC23, GL14, GL15 (D4's full listing has a reset
     and a trigger), GL16/GL16b, GL17, and §7's A1 mutations RM1, RM4, RM9, RM14 (they move to A1b with their
     features). SM1–SM4, WG1–WG4, WG7, WG8 are A2e/A2u's.
  2. **GL18 runs on robot x's SWITCH-homed lift started 2 in up** (its switch following the plant), not robot y's
     stall-homed arm: the base has no auton rig for robot y (RPV1-A's `fakes_by_kind.hpp` is unmerged); a
     switch-homed mechanism is inside RC25 too. Its four twins: no home → NOT_HOMED at step 1; startHome then move
     → NOT_HOMED at step 2; "home it first" → ok; "home it while the first leg drives" → ok.
  3. **HS9 deferred** (every export written into a scratch tree, the suite and src gate there): there is no
     `src/team/skills_route.cpp` on the base (ROB1-BT). Instead the golden test compiles every export with the
     suite's flags against the REAL robot tables (robot one's cascade through its facts ledger; robot two's
     current claws) and runs it. HS9 belongs to the coordinator's end-to-end check after ROB1-BT (§7).
  4. **NF1 in A1** = every stored number on the 0.001 grid and the export printing exactly `fmt()` of it
     (`route_lines.mjs` is A2e's file — §1.1); RM34 as written lands with it; A1's equivalent (the export printing
     a 4th decimal) observed red.
  5. **No `refusals.json`:** the quoted sentences live in `model/library_words.mjs`, and a node test reads the
     headers (adjacent C++ literals joined) and fails when one is gone — the same guard without a generated file.
  6. **Robots one and two are NOT committed descriptions** (R11: a facts edit would redden a gate): `build.py
     --describe` runs `shulib_route_maker` at build; only the fixture robots x, y, z are committed (the suite pins
     them). The C++ name of each declaration is read from the line its `.name` recorded (`std::source_location`)
     — no hand table to drift, no path written.
  7. **Robot two's checks start when the shared file does:** robot two's list is the claws on this base, so a
     cascade step would be red on it; `build.py` marks `sharedRoute.present` when `src/team/skills_route.cpp` has
     its markers (§6.3: then BOTH packages compile it) and the page checks robot two's list from then on, saying
     so once ("robot two is checked once its update is in this build").
  8. **The field transform is measured** (7.624 px/in, origin (570, 573.5) on the turned source) — the docs
     figure's hand 7.5 px/in put the red wall ~1.5 in off, past FP1's 1 in; FP1's Toggle landmarks are the C++'s
     own `TogglePlace` (the wall's inner face at its centre) — the render's yellow Toggle bars sit ~5 in outside.
  9. **No start-tile snap target:** nothing in the C++ states the start tiles; snapping is the 12 in grid and a
     wall flush. **D3's `seen`** is not written: the page shows the build it was made from; a description that
     can differ from the page's arrives with a helper (A.6).
  10. **Page-own rules** (stricter than the library, each said as the page's): a drive or face target under 1 in
      from the planned position (the library throws at exactly 0); a label of printable ASCII ≤ 29 without `"` or
      `\`; volts outside −12…12. Every one an ERROR with plain words, no invented number.
  11. **UX choices:** after a step is added the tool returns to Select (the new step is selected, its details on
      the right); the coach's third line says Export (no PREVIEW in A1); the turn-first face takes the robot's
      own options (the step's care is the drive's); one toast at a time.
  12. **The ONE FILE** is `shulib-route-builder.html`; the SITE form `app.html` + `src/` (+ `robots.json`).
  13. **A test hook in the page**, `window.routeBuilder` (a read-only copy of the route and its issues, and
      `useMemoryFolder` for the file layer's page test: the real folder dialog is the manual Windows check's).
  14. **guide 16** not touched: its "or press PREVIEW" line is A2u's. **The status pointers** (`roadmap.md`,
      `build-order.md`) are the coordinator's at the merge (shared plan docs).
- **Self-review before the gates:** a curve finished by a DOUBLE-click added its last point twice (a pointer
  event's `detail` is 0 on pointerdown, so the second click of the double-click was a new point on the same spot →
  RC26 TooClose). Fixed: a click within 1 in of the last draft point finishes the curve instead; the clicking page
  test now finishes a curve with `dblclick` and asserts 2 points; mutation RMU2 (the guard removed) → that test red.
  Also: Alt+Up / Alt+Down made global (the guide says "the selected step"), with a page-test case.

### Resumed 2026-11-15 18:52 EST (`date`) — the previous run was stopped by the account usage limit (~04:00 11-14)
- Read this log and RESUMING.md's standing rules again. `git status`: the work above, nothing committed; `git diff --
  src/ Makefile` EMPTY (no gate self-test plant left; the scratch `srcgate-selftest.log` / `srcgate-check.log` show the
  stopped run's last gates — self-test OK 377 cases, check PASS 14 builds — at ~02:31 11-14). Every mutation backup in
  scratch `backup/` byte-equal to its live file; the manifest taken before the last batch matches every file but
  `ui/app.mjs` and `ui/step_list.mjs`, the two the self-review above changed after it (the double-click guard,
  Alt+Up/Down). No browser or driver process running (`ps`).
- A FRESH build before trusting anything: `build/test/shulib_tests` and `shulib_route_maker` deleted, the briefing
  regenerated, `cmake --build build/test --clean-first` (every object recompiled, so no mutant object can survive).
- **The fresh build** (`--clean-first`, 1 min 07 s, 0 warnings): `suite_run.py green` — **3674 cases, 3862737
  assertions** (the base's 3661 + RMP1's 13). `node --test tools/route_maker/test/` — 61 pass. `build.py` — the site
  folder and the one file. The page tests in Playwright's Firefox 157 — 6 pass. So the stopped run's work stands as
  logged above.
- **Found and fixed on resuming (each a real defect, each with its test and its mutation):**
  1. **A drive's END point could not be dragged** (the guide says "drag any point"): the NEXT step's planned path
     (a curve starts where a drive ends) and the step-number circle were drawn over the handle and took the pointer,
     so the drag became a pan. Now ONLY the handles take the pointer (`style.css`: `.g-plan, .g-ghost, .g-turn,
     .g-draft, .g-heading, .g-grid { pointer-events: none; }`, and the pills too). Test: the clicking page test drags
     the drive's end (-12, -36) → (-12, -24) and undoes it. **RMU4** (`.g-plan` dropped from that rule) → RED on
     "the drive's end dragged (not the field panned)" (actual -36). Found by `elementFromPoint` at the handle:
     `polyline.g-plan`.
  2. **Step numbers hid under each other**: every step drew its own circle at the spot it ends, so at a curve's end
     where a level is raised and the robot waits, only "5" showed, and the start's robot covered a home's "1". Now
     the steps that end at one spot share ONE pill ("3–5"; "3, 4"), drawn last, coloured by the worst mark (red over
     amber) — the error/warn badge classes existed in the code with NO style. Test: the clicking route's pills read
     exactly `['1', '2', '3, 4']`. **RMU3** (one pill per step) → RED (`'3', '4'` against `'3, 4'`).
  3. **The turn-first hint said "15 deg" from a literal**; it now prints the robot's own `maxSteerDeg` (15 for both
     robots today, so nothing on screen changed).
  4. **Contrast under 4.5:1** (§2.5.3) in the LIGHT theme: the amber chip/badge text `--cut` #b86e00 on `--warn-soft`
     (3.65:1) and the green "Ready to export" `--ok` #1f8f4e on `--ok-soft` (3.64:1) — computed by the WCAG formula.
     `--cut` → #975a00 (5.09:1), `--ok` → #187540 (5.07:1); the dark theme was already ≥ 4.9:1. The error toast's
     `#fff` (3:1 on the dark theme's red) → `var(--on-accent)`.
  5. **Polish:** "No steps yet" was a GREEN check chip (green reads as done) — now neutral; the mode hint ("Click the
     field where robot one starts") showed as a dark sliver behind the first-run coach card — hidden while the card
     says the same thing.
- **The screens (§2.5.2), redone on a READY route** (start on the red tiles facing blue, a home with its time, a
  drive turned first, a curve, a cascade level, a wait — "Ready to export"), light and dark at 1440 × 900 and
  1024 × 768, plus two R6 screens: a drive with "turn to face it first" OFF (RC24 red, 26.6° off, the library's
  sentence, the fix button) and a curve with a refused point (the library's verdict: point 1 doubles back). A first
  attempt put the start flush against the red wall — the page said, correctly, that turning there puts a corner
  2.4 in into the wall (RC1); the start moved to the grid's y = −60. The test first expected a "first point behind"
  verdict; the library (CV1-pinned port) says "point 1 doubles back" for that point — the test now asserts the
  library's verdict. 12 screenshots kept in scratch `shots-final/`; the guide's 7 (`docs/assets/route-builder/`,
  1200 × 750, 256 colours, ~160–220 KB each — the repository's PNG convention): first-run, turn-first (NEW),
  curve-refused (NEW), home-first, route (now the ready route), export, route-dark.
- **Docs:** guide 22 gains the turn-first picture (R1/R6: "a picture each" — the home-first one existed), the
  refused-curve picture, and what the number pills mean; `docs/route-builder.md` no longer claims "Mac and
  Chromebooks" (not tested: it says which browsers CI runs on each OS); the changelog's heading likewise. **NEW
  internal page** [`RMP1-A1-manual-checks.md`](RMP1-A1-manual-checks.md): A.4's Windows-laptop check cut to A1 (no
  PREVIEW; Export writes into the folder only after ROB1-BT), §2.5.4's non-coder card (ends at Export), and §2.5.3's
  UX lens for the reviewers. `pages.yml`: the internal-docs guard's comment had been left ABOVE the new PG1 step —
  moved back to its own step.
- **Prose review (rule 4), this session's greps, RUN and read** over guide 22, `docs/route-builder.md`, the new
  checks page, `tools/route_maker/` and `test/route_maker_*`: `simulat` 1 — `route_maker_rig.hpp`'s comment quoting
  the rule itself, reworded ("Everything here runs on the HOST TEST PLANT"); `honest` 1 — the checks page quoting
  §2.5.3's rule; `obviously`/`simply`/`easy` 0; `just ` 4, all code-comment adverbs; `precondition` 6 — comments
  and one test naming the rule; `radian` 11 — model comments, a test and the rig; `http` in the built pages: only
  `http://www.w3.org/2000/svg`.
- **Deltas added this session:** 15. guide 22's file is `22-making-your-first-route.md` and the page says "route
  builder" (the team lead's name, A.12) where the brief's §3 says `22-the-route-maker.md` / "route maker"; the
  C++ region's comments still say "route maker" because they are §6.3's CONTRACT text (GL0 byte for byte, ROB1-BT
  writes the same) — renaming them is a contract change for the coordinator (both chunks at once), not A1's.
  16. A.4's manual check and §2.5.4's card are a separate internal page, not inside this log, so the team lead can
  be handed one short page.
- **`node --test` takes FILES now, everywhere** (ci.yml's three steps, both READMEs, `package.json`, the page test's
  header): `node --test tools/route_maker/test/*.test.mjs` and `.../test/page/*.test.mjs`, expanded by bash (every
  CI row runs bash, Windows included). Why: Node 20 (this laptop) takes a directory but no glob; Node 22 (CI's pin)
  reads its positional arguments as GLOBS, and a bare directory there is not a promise this run could measure (no
  Node 22 here) — a list of files means the same to both. Measured on Node 20: the file form runs **60** model tests,
  all pass; the directory form's "61" counted `helpers.mjs` as a test file (it has none) — so the model has 60 tests,
  not the 61 logged above. `package.json`'s `test` is now bare `node --test` (Node's default patterns from
  `tools/route_maker/`: 61 entries = the 60 + the page suite, skipped without a browser). **Delta 17:** the brief's
  DoD writes `node --test tools/route_maker/test/` — the same tests, by file.
- **The page tests are 7 now, and none skips in a real browser:** the two §2.5.2 screen tests assert behaviour too
  (the ready route has no error and says "Ready to export"; RC24's red + fix; RC26's verdict in the library's words),
  so they run on every row; only their 3.5 s settle-before-a-screenshot waits need `ROUTE_MAKER_SHOTS` (CI sets it).
  Firefox 157 here: 7 pass, 0 skipped, ~13 s.
- **The src gate ran BESIDE two other trees' gates** (`ps` + `/proc/<pid>/cwd`: one in the main tree, started 19:02,
  and one in worktree `rpv1-s1`, 19:21 — neither this run's; mine is the one whose cwd is this worktree), so it ran
  slower than the stopped run's; each builds in its own tree.
- **The gates, final (2026-11-15 ~20:20), on the tree as it stands:** suite (`suite_run.py green`, rebuilt after the
  src gate re-touched `src/` timestamps) **green — 3674 cases, 3862737 assertions**; `node --test
  tools/route_maker/test/*.test.mjs` **60 pass**; the page tests in Playwright's Firefox **7 pass, 0 skipped**;
  `build.py --out build/route_maker_site` made both forms (app.html 119791 B + `src/`; the one file 338519 B);
  pages.yml's build run locally (pinned mkdocs 1.6.1 + material 9.5.44, scratch `pages_local.sh`): `mkdocs build
  --strict` clean, PG1 and the internal-docs guard pass (`site/`, `site_src/` removed after); `src_build_gate.py
  self-test` **OK (377 detector cases)**, `check` **PASS (14 builds)**; `git diff -- src/ Makefile include/` EMPTY
  and no "(self-test plant" anywhere; the ARM header gate exit 0 (255 headers); the CI greps (PROS-free, sim-free)
  hold; `api_doc_tool` self-test OK, check-coverage (6386 entities, 211 headers), check-fresh exit 0,
  check-examples (860 lines), check-removability clean; `briefing_status.py` self-test OK and check current (the
  briefing regenerated with the built suite: 3,674 cases; RMP1 listed open); `doc_staleness_audit.py` self-test OK
  and clean; `ctest -R compile_refusal` **289/289**, TUs 289 = CTests 289. **No API change** (nothing under
  `include/`); **no HA row** (§4). **NOT RUN: the CI run on the pushed branch — the coordinator's (D20).**
- **Processes:** every Playwright Firefox this run started was closed by its test's `finally`/`after`; `ps` after
  each run shows none under `~/shulib-render-tmp/` (the snap Firefox processes on this laptop are the desktop's
  own, started before this run — not touched). No driver was used.
- **Stopping here for the review** (three reviewers, fixes, a fresh verifier, the coordinator's checks, then the
  Windows CI on the pushed branch). Nothing committed, nothing pushed.

### RMP1-A1 review findings and their disposition (2026-11-15 ~20:40 – 22:30 EST, `date`)
Three reviewers (correctness, tests-and-gates, docs) returned 34 findings; the ones that overlap (the export with a
missing number; the download way's hand-edit refusal; the Save test's skip) are answered once. Each was RE-CHECKED in
the code before it was touched (the reviewers' probes in scratch `review-correctness/probes/` and their mutation
logs in `review-gates/logs/` re-read), then fixed at the root or answered with evidence. Every fix has a test, and
every test was proven by a mutation observed RED (the table after the list). Nothing committed, nothing pushed.

**Correctness**
1. **Export did nothing on a step missing a number** (major) — REAL (`exportRegion` ran before the error gate; `fmt`
   throws on `undefined`). FIXED: `export_drawer.mjs` checks the errors FIRST and makes the code only from a route with
   none (and a `try` turns any throw into the refusal's own words, never a dead button); a second Ctrl+E while the
   drawer is open is ignored. The Export button stays clickable (a disabled button cannot carry the "Go there" links —
   delta 18) and now carries a red COUNT of the errors and a tooltip naming the first three. Guide 22 no longer says
   "disabled". Page test "an error stands": a wait with no seconds, a fresh level move, and the state "Home it first"
   leaves — each listed, no code, no page error, "Go there" lands on the step.
2. **A rename rewrote §6.3's marker lines and `e.name`** (major) — REAL (ROB1-BT's `skills_route_test.cpp` pins
   `"skills"` in both). FIXED: A1 offers NO rename — the top bar and the inspector SHOW the name ("Route skills"; "both
   robots' code registers the skills route by this name, so it stays"); RC21 gains the contract rule (any name but
   `"skills"` is an ERROR, `contract: true`, with the one-click fix **Name it "skills"**, so a hand-edited file cannot
   export new markers). `route.ROUTE_NAME`. Delta 19. Node: RC21's library rules unchanged and the contract rule with
   its fix; page: no name input anywhere.
3. **Unknown step kinds and fields were half-read** (major) — REAL (`during` loaded with zero issues and its trigger
   left out of the C++; `wallReset` made Export throw). FIXED at the root, in `project.mjs`: `newerParts()` refuses BY
   NAME — "This project has a step field "during" (route "skills", step 2), which this route builder cannot write: it
   was made by a NEWER route builder. Nothing was opened …" — any step kind not in `STEP_KINDS`, any key outside A1's
   vocabulary on a route, a step, a `care`, a curve point or the start, an `onFail` other than stop / carryOn /
   {skipTo}, a non-skills route, a schedule that is not a number, and a SECOND route (A1 edits one; saving would have
   dropped the rest). A top-level block (RMP1-B's `robots.<id>`) is still kept. In memory, an unknown kind is also an
   RC19 ERROR (defence). RT7 (nine cases, D3's own `during` and `wallReset` samples among them); a page test opens a
   `during` file and is refused, the page untouched.
4. **Save → open was not the identity for a hole** (major) — REAL (an emptied curve x became `NaN` → `null` → 0; a
   typed-over care field was dropped). FIXED: `route.mjs` never stores a non-finite number — a curve point's missing
   coordinate, a start's missing part and a `NaN` care value are left OUT (a hand-edited text care value is kept AS
   TEXT so RC19 names it); `null` in a file is a hole, never 0. The inspector no longer stores unreadable text at all:
   "2s" stays in the box, marked, with "Not a number: type digits, like 2.5 (in s)", and nothing is stored. RT6 (an
   emptied point, a NaN care field and start part, a hand-edited `null`: reopened exactly, the error still there, the
   second save byte-identical); page: "2s" refused in place.
5 + 20. **The download way called the builder's own export a hand edit, and its "Overwrite" did not exist** (major,
   correctness and tests) — REAL (probe p5 and G25/G26 re-read). FIXED: ONE hand-edit choice (`askHandEdit`: Keep their
   change / Overwrite) for both ways of writing; after a download export the project is marked **Unsaved changes** and
   the drawer says why and offers **Save route_maker.json** (the hash then reaches the file). Host test: day 1 save →
   export → save, day 2 open that file → export is NOT a hand edit; a real hand edit refused, nothing downloaded,
   Overwrite works. Page test (Firefox — the branch only exists where the folder cannot be written; the skip says so):
   pick a hand-edited file → refused → Overwrite → the file back with only the region changed, CRLF kept → Save asked
   and the saved project carries the hash.
6. **"Download for offline use" was a plain link** (major) — REAL (rendered `<a class="md-button" href=…>`). FIXED:
   `{ .md-button download="shulib-route-builder.html" }` (attr_list is on) and a "Save link as" line; **PG1 now greps
   the rendered link for `download=`** (pages.yml). Run locally with the pinned mkdocs 1.6.1 (scratch `pages_local2.sh`
   now READS PG1 and the internal-docs guard out of pages.yml instead of a hand copy): rendered
   `<a class="md-button" download="shulib-route-builder.html" href="shulib-route-builder.html">`, PG1 passes.
7. **Robot two described without a ledger** (major) — REAL, and not fixable on this base (robot two has no cascade and
   no `robot_two_facts::lv` until ROB1-BT). PREPARED for the merge that lands second: a C++ TRIPWIRE — "a robot that
   builds the shared route and declares a cascade marks its levels as its facts ledger's" — passes on this base (robot
   two has no cascade) and goes RED at ROB1-BT's merge until `route_maker_robots.hpp` gives robot two its
   `.ledgerLevel`; `route_maker_robots.hpp` says so at robot two's entry. Node: a ledger move passes on [one, two]
   both carrying the ledger, and is refused on robot two without it. **For the coordinator** (below).
8. **Care field names were not G3a R.4's** (major) — REAL. FIXED before any team file exists: `maxSpeedInPerS`,
   `maxTurnDegPerS` (route.mjs, export.mjs, inspector.mjs, the tests, `gl19_caps.route.json` re-made).
9. **RC18 quoted run-time preconditions instead of the BUILD refusals (R9)** (minor) — REAL. FIXED:
   `library_words.mjs` entries `noPreset` / `noPosition` carry `ident` (`no_such_preset_on_this_mechanism`,
   `no_such_position_on_this_mechanism`) and `twin` (`k_misspelt_lift_preset.cpp`, `n23_misspelt_position.cpp`); the
   page shows "the build stops: <name>"; a test holds each name DEFINED in `kinds.hpp`, CALLED there, and its twin's
   `// expect:` line naming exactly it.
10. **The turn-first comment gave the drive's caps to the turn** (minor) — REAL. FIXED in the words (delta 11's choice
    kept): "First it turns to face the point (at the robot's own turn settings); the drive: at most 2 s, turning at
    most 60 deg/s." The inspector's section says "(the drive)" and why when turn-first is on. Export test pins it.
11. **The draft claimed unsaved work after a Save; Save overwrote route_maker.json blind** (minor) — REAL. FIXED in
    `app.mjs` + `files.mjs`: the text last saved or opened is kept beside the draft, and the draft is announced only
    when it differs ("Saved" otherwise); a connected (or remembered) folder's `route_maker.json` is READ and compared at
    connect — the same: "Saved"; an empty page: the folder's opens; two different routes: the page ASKS (its own
    dialog, `dom.ask`); Save compares the folder's file with what the page last read or wrote and never replaces a
    changed one without asking (Open the folder's / Replace / Cancel); the quiet save after a folder export does not
    replace it either (the drawer says so). Host test: changed → refused, then forced; unchanged and deleted → written.
    Page tests: the draft across two reloads; the folder compared at connect and Save's three answers.
12. **Robot one's OWED level heights shown as facts** (minor) — REAL. FIXED: `describe` writes `"owed": true` on every
    ledger level while `robot_one_facts::kCascadeLevels` is owed (`RobotSource.ledgerOwed`), and the Level list says
    `level 2: "L2" (not measured yet)`. C++: the mark exactly while the fact is owed; page: the text.
13. **After a turn-first drive the brain's step numbers no longer match the page's** (minor) — REAL (no library call
    takes a page step's number; an API change is not A1's). FIXED in the words: every comment from the first turn-first
    drive on says the brain's number where it differs ("The brain's log calls the turn step 2 and the drive step 3.";
    "The brain's log calls this step 4."), and guide 22's new "Reading the code" says what `STOPPED at step N` counts.
    Export test over a five-step route with two turn-first drives.
14 + 24. **No browser test opened a file; Save skipped on the Chromium rows for a reason that is not true** (minor,
    both lenses) — REAL. FIXED: the Save test runs on EVERY row (no folder connected, Chrome and Edge download too) and
    OPENS the file back through the page's own file chooser — the route AND its export byte-identical — and opens a
    newer file, refused. Measured locally in BOTH engines (below): it passes in Playwright's Chromium.

**Tests and gates**
15. **RC1's slack and turn sweep pinned at neither edge** — REAL. FIXED: (a) the grid points either side of the 1 in
    slack (63.704 → 0.99928 in allowed, 63.705 → 1.00028 in refused); (b) ON the edge, exactly: a robot whose footprint
    (2 × (wall + 1 − 62), the test's own number) puts x = 62's corner EXACTLY 1 in past — allowed ("more than 1 in" is
    THROUGH) — and 62.001 refused; (c) a QUARTER turn 7.6 in from the red wall (both ends fit, the sampled sweep goes
    3.0 in through) and the same through a face. G3c (slack 0.9995, inside the grid bracket) SURVIVED (a) — no grid
    point lies in [0.99928, 1.00028) — which is why (b) was added; then RED.
16. **RC25's failed-wait branch and its "still homing" fix untested** — REAL. FIXED: `[startHome, waitUntilDone
    (carry on), move]` → a WARNING at step 3 naming the path; the "still homing" fix applied → `[startHome,
    waitUntilDone, move]`, its time limit empty, no RC25 left.
17. **RC19's motion options untested; two options never compiled** — REAL. FIXED: every option −1 refused, 0 allowed,
    "fast" refused; `toleranceDeg` 180 refused, 179.999 allowed; an option on a mechanism step refused, `onFail` on it
    allowed. GL19's drive carries `toleranceIn: 2` and its turn `toleranceDeg: 5`: compiled with the suite's flags and
    RUN — the robot settles within them (and the code says `.positionTolerance = 2_in`, `.headingTolerance = 5_deg`).
18. **GL0 pinned the exporter only to itself** — REAL. FIXED: `gl0_empty.route.json` REMOVED (kept in scratch
    `fp3/gl0_empty.route.json.removed`), so `goldens.mjs --write` never re-makes `gl0_empty.cpp`; its sha256 is pinned
    in `export.test.mjs` (re-checked against the brief's lines 2156–2181, extracted again: identical); a new test
    compares the exporter with the REAL `src/team/skills_route.cpp` region the day ROB1-BT puts it there (skipped, with
    its reason, on this base). The golden count is 17 route files now (the tests' `>= 18` became `>= 17`).
19. **FP1 could not see a wrong scale** — REAL. FIXED: FP3 — the picture's own TILE SEAMS (six tiles wall face to
    wall face, 23.40 in each), measured by `route_maker_field.py` from the two tile greys (each seam's crossing in six
    bands, the wall transform only saying where to look ±15 px), recorded in `field.json` (`tileSeamsPx`, the WebP
    unchanged: md5 equal before and after). Each seam maps within 0.09–0.43 in of its tile line, the outer spans
    93.86 / 93.94 in against 93.6; under G30's scale the x span is 92.14 in — RED.
20. See 5.
21. **The page tests saw neither the picture nor the drag's undo entry (RM8)** — REAL. FIXED: WG5 decodes the field
    `<image>`'s href in the page (`data:image/webp`, 1138 × 1138, drawn 1138 wide); after the drag, one Ctrl+Z must give
    back EXACTLY the route before the drag (the reorder before it untouched), and redo brings the drag back. RM8 run
    (P3): RED.
22. **"The robot's own" footprint and steering never varied** — REAL. FIXED: an 18 in robot steering within 8 deg:
    the wall edge moves 1.5 in (62.204 / 62.205), a 10 deg leg is an ERROR with the library's "(steers within 8.0
    deg)"; and RC24 ON its edge — a robot whose `maxSteerDeg` IS the leg's own computed offset, to the last bit: a
    warning, not an error (the library refuses only above it).
23. **Smaller rules** — all six FIXED with one test each: TightBend a WARNING; a name with `"` refused with its own
    words; the same RC25 on [one, two] read ONCE; BEGIN and END naming different routes refused (`findRegion` and
    `spliceRegion`); schemaVersion 0, −1 and 1.5 refused; Duplicate leaves no RC16.
24. See 14.
25. **library_words pinned only the tail of RC24's sentence** — REAL. FIXED: three entries, each ONE whole literal of
    `move_to_pose.hpp` (`lits.includes`, not a substring), and the page's sentence is joined from them
    (`tankNotSteeredWords`).

**Docs**
26. **§2.5.2 screens only partly taken** — REAL. FIXED: the screen tests now run EVERY A1 screen in light and dark at
    1440 × 900 and 1024 × 768 when `ROUTE_MAKER_SHOTS` is set (CI sets it on every row; without it they run once, light
    1440): first run, the start placed, the ready route, a drive not turned first, a refused curve, a cascade move with
    no home, the home just added (its empty time limit), Export listing the errors, the folder not connected, Copy only
    (no skills-route file yet AND robot two's line missing — today's state), no markers, a hand edit (keep / overwrite),
    a newer project refused — plus the download way's hand edit, Save, and the old-browser box. 55 screenshots, kept in
    scratch `shots-review/`; each was asserted on, and seven were viewed here.
27. **The Windows checklist** — REAL. FIXED: step 3 now chooses the level and types its time limit (and says what the
    Export count means); step 7 now asks the team lead, in the DOWNLOADED file in Edge, whether "Connect your shulib
    folder" is offered and whether Save writes — the brief's A.10 / landmine 23 question — and names what CI records.
    The page test now RECORDS it on every row (`t.diagnostic`) and asserts the page offers exactly what the browser
    allows; Playwright's Chromium over file:// here: `isSecureContext=true, showDirectoryPicker=true` (so the one file
    offers the folder there; Edge's real write stays the manual step).
28. **Delta 2's reason stale on today's main; shared-doc lines move at the rebase** — REAL. Delta 2 re-logged below;
    the rebase items are in "For you".
29. **D4's level name and §1.4's robot picker** — REAL, both logged as deltas 20 and 21 (below).
30. **Guide 22's untrue sentences** — REAL, all five FIXED: "can stop short" with P8's real numbers (up to 8° arrived,
    10° stopped short); the refused curve captioned as the library's own verdict ("point 1 doubles back"); "robot one's
    file declares it a tank" (HA-252 is not settled); robot two's mirrored line in the conditional ("Once robot two's
    update is in, one line … the export page checks"); Safari "should do the same, but no test runs it" (and its own row
    in `route-builder.md`, "not tested"). Also: the home-first caption no longer claims a library sentence RC25 does not
    have.
31. **G4 line, today's export picture, the start and Save pictures, how to read the code** — REAL, all FIXED: "The
    route in the pictures is an example of how to click, not a route for the robot"; `export.png` retaken in TODAY's
    state (the folder connected, no skills-route file yet — Copy only — and robot two's line missing), captioned as what
    Edge or Chrome shows, no toast over it; NEW `start.png` and `saved.png`; NEW "Reading the code" (the comment, the
    calls and units, the level through the facts, the brain's numbers, the entry). It is prose with inline code, NOT a
    `cpp` listing: the first draft quoted an export as a fenced block and the doc gate refused it ("NOT VERBATIM": a
    public listing must be a line of a compiled example or team file) — the exporter's text is pinned by the goldens,
    not by a guide quote, so the guide points at the picture instead of copying lines that could drift.
32. **Guide 14's list missed three** — FIXED: "driving through points", "a step of your own C++", "exporting a route to
    VexBuilder".
33. **D17's publish vs RELEASING's between-parts rule** — REAL. Written into RELEASING.md's new section as an OPEN
    question with the two ways out — the coordinator's (and the team lead's) to decide before A1's merge; not decided
    here.
34. **Stale comments** — FIXED: `goldens.test.mjs` → `export.test.mjs` (route_maker_golden_test.cpp, goldens.mjs);
    ci.yml's header names the two route-builder jobs.
35. **Guide 2's field figure off by about 1 in** — REAL (`field_coordinates.py` still `PX_PER_IN = 7.5`, centre (565,
    570), on main `e9bcee7` too). NOT fixed here: it is guide 2's figure (an earlier chunk's file and a public picture),
    and the fix belongs THERE — handed to the coordinator in "For you" with the measured numbers.

**Mutations of this round** (scratch `mutate2.py`: one or more exact edits, backups under `backup2/`, restored in a
`finally` and on SIGTERM/SIGHUP/SIGINT, each restored file touched and md5-checked; goldens backed up and restored
whole; every node run under `ulimit -v 4 GiB` + `timeout`; logs in `mut2-logs/`). **55 mutants, every one RED, each by
the intended test** (its failing line read in the log):
- the reviewers' survivors, now caught: G1 (maxSteer literal) and G11 (size literal) — the 18 in / 8 deg robot; G2
  (`>=` at maxSteer) — RC24 ON its edge; G3 (slack 0), G3b (1.09), G3d (1.0005) — the grid bracket; **G3c (0.9995)
  SURVIVED the bracket** → the exact-edge test → RED; G3e (`>=` at the drive's end) — the exact edge; G4 (no 180 rule),
  G4b (`> 180`) — the 180 / 179.999 pair; G5, G6 — RC19's options; G7 — the failed wait; G8 — TightBend a warning;
  G9 — read once; G10 — the quote; G12 — mismatched markers; G13 — schemaVersion 0; G14 — the "still homing" fix;
  G18 — the quarter turn; G25, G26 — the download way's day 2; G28 — Duplicate; G30 — FP3's span; X_GL0 (the empty
  region reworded, goldens re-made) — GL0's pinned hash and contract text;
- the fixes' own: N1 (no name contract) — RC21/§6.3; N2 (newer parts ignored) — RT7; N3 / N4 / N5 (a point, a care
  value, a start part stored as NaN) — RT6; N6 (the brain's numbers off) — the brain's numbers + the goldens' freshness;
  N7 (the caps said of the turn) — the turn-first wording; N8 (R9's name wrong) — R9; N9 (one RC24 piece reworded) —
  the pieces, RC24's sentence and the quoted-sentences test; N10 (Save never compares) — the host test; N11 (one seam
  7 px off) — FP3;
- the page (Playwright's Firefox, the site rebuilt from the mutant each time): P1 (no picture) — WG5 ("Invalid image
  request"); P3 = §2.3's RM8 (the drag not in the undo history) — "one undo puts back exactly the route before the drag";
  F1 (the export made before the error gate, the reviewer's bug) — the drawer never appears; F1b (no error gate) — the
  try's refusal is not the error list; F5 (no Overwrite in the download way), F5b (no Save prompt after it) — the
  download test; F12 (the draft always announced) — the draft test; F13 (the folder never compared), F13b (Save never
  compares) — the folder test; F14 ("2s" stored) — the "Not a number" assertion; F15 (an owed height shown) — "(16 in)"
  against "(not measured yet)"; F17 (no count on Export) — the count; F18 (Open ignored) — Save and Open; F19 (a rename
  input put back) — "no name input";
- pages.yml: PG1 (the plain link back) — `::error::the docs page's 'Download for offline use' link has no download
  attribute`, from the local pages build;
- the C++ (the suite target rebuilt and RE-LINKED each time — `rm -f build/test/shulib_tests` first, so no earlier link
  answers; the briefing held at "not built" for the series): C1 (owed not described) — `:387` the owed CHECK; C2 (robot
  one's ledger removed) — the TRIPWIRE's own CHECK at `:417` (and `:385`, `:387`); C3 (heading tolerance ×20, goldens
  re-made) — `:344` the code AND `:347` the robot's heading (behaviour); C4 (position tolerance ×10) — `:343` AND
  `:346` the robot's position (behaviour).
- **Re-cut, not counted:** the first C++ round "failed" at the BUILD — the doc gate refused a `cpp` listing in guide 22
  (item 31) and then the briefing (stale against a linked binary). Neither is the test's assertion, so the four were
  re-cut after the listing became prose and with the binary removed before each build; the counts above are the re-run.

**Deltas added in this round:**
18. **Export is not `disabled` while a step has an error** (§1.4 says disabled, its tooltip listing the first ones
    with "go to step"): a disabled button cannot be focused or clicked, so its "go to step" links could not be used.
    Instead the button carries the error COUNT and a tooltip naming the first three, and pressing it (or Ctrl+E) shows
    ONLY the list, each with Go there — never code from a route with an error.
19. **The route's name is not editable in A1** (§1.4's top bar shows "the project's name"; RC21 assumed a free name):
    §6.3's markers and `e.name` are `"skills"`, which both robots' code registers and ROB1-BT pins. The name is shown;
    RC21 refuses any other with a one-click fix. A free name is a contract change (both chunks, the coordinator's).
20. **D4's "the comment carries the name the page showed" for a level** — not followed: the comment says `level 2
    (facts::lv(2) in the robot's facts)`, not `"L2"`. The export depends on the ROUTE alone (its header's rule, which the
    hand-edit hash and the goldens rely on); the level's name lives in the robot's facts and would make a facts rename
    change the region (a "hand edit" the next export refuses) — the very thing `facts::lv(i)` is there to prevent.
21. **No robot picker (§1.4)**: A1 writes ONE route, robot one's, mirrored for robot two; the top bar's "Robot one" chip
    says so and "The robots" in the inspector says which robots it is checked on. A picker comes with a route of robot
    two's own (later).
22. **The page's error toast stays 9 s** (4.2 s before): a refusal such as "made by a newer route builder" must be
    readable before it goes.
23. **Re-log of delta 2 (its reason was stale):** GL18 runs on robot x's SWITCH-homed lift because THIS BRANCH's base
    (49822e5) has no auton rig for robot y. RPV1-A's `test/fakes_by_kind.hpp` merged on main at 9befdb3 (2026-11-14,
    during this run), so at the rebase onto main GL18's four twins can move to robot y's STALL-homed `swing`
    (`homeByStall`, the method robot one's and robot two's cascade use — R1's case) through it. On this base the
    library rules the twins exercise (a move REPLACES a running home; an un-homed move stops NOT_HOMED) are the same for
    both methods; only the successful home differs. Owed at the rebase (For you).

**Measured beside the reviews:**
- **Playwright's Chromium runs here** (the headless shell, `chromium_headless_shell-1248`, already in the shared
  `pw-browsers`): it dies with SIGTRAP at launch under `ulimit -v` (it reserves more address space than 4 GiB), so it
  ran under a cgroup cap of the same size on its real memory instead — `systemd-run --user --scope -p MemoryMax=4G -p
  MemorySwapMax=0` (the 2026-10-15 rule's purpose: a runaway cannot freeze the laptop). **11 pass, 2 skipped** (the
  download way and the old-browser box are Firefox's, each skip saying why). The developer README says how.
- **Firefox:** 13 pass (one variant); with the screenshots, 25 pass (every screen × 4) in 3 min 31 s.
- The model: **79 tests — 78 pass, 1 skipped** (GL0 against the real skills-route file: not on this base).

**Prose review (rule 4), run and read** over guide 22, `route-builder.md`, guide 14, the changelog entry, both
READMEs, `tools/route_maker/` and the manual-checks page, for the changed names and concepts: `maxSpeedInS` /
`maxTurnDegS` 0; `goldens.test` 0; `turning to face it first` (the old comment) 0; `rename` 5 — the guide's and the
README's "does not rename", checks.mjs's rule, and `renameRoute` (its comment now says the page does not offer it in
A1); `route's name` 5 — all say it is shown, not edited; `disabled` — real disabled controls, guide 14's unrelated
HA-157 lines, and ONE stale comment (`checks.mjs` `firstError`, "Export is disabled") — reworded; `Safari` 6 — the two
docs now say untested, `browser_check.js` states the floor, `files_download.mjs` the host it serves; `half-read` 0;
`simulat` / `honest` — none in this run's files (the hits are the repository README's own); `obviously` / `simply` /
`easy` 0; `just ` — code-comment adverbs and the README; `precondition`, `radian` — model comments and tests naming the
rule. `http` in the built pages: the SVG namespace only (RG1 green in the build).

**The gates, final (2026-11-15 ~22:30), on the tree as it stands** — after every mutant restored (each backup
byte-equal to its live file), `build/test/shulib_tests` removed, the briefing regenerated, `cmake --build build/test`
(0 warnings), the briefing regenerated again with the built suite and the build re-run (the doc gate green):
`suite_run.py green` — **3675 cases, 3862746 assertions** (the base's 3661 + RMP1's 14); `node --test
tools/route_maker/test/*.test.mjs` — **79: 78 pass, 1 skipped**; `build.py` — both forms (app.html 120345 B + `src/`;
the one file 363369 B, after the last two comment fixes — the model and Firefox page runs repeated on it); the page tests — **Firefox 13 pass; Chromium 11 pass, 2 skipped**; the pages build locally
(pinned mkdocs 1.6.1 + material 9.5.44): `mkdocs build --strict` clean, PG1 (with its new download check) and the
internal-docs guard pass; the CI guards (PROS-free, sim-free) and the doc gates — `api_doc_tool` self-test OK,
check-coverage (6386 entities, 211 headers), check-fresh, check-examples (860 lines, 18 sources), check-removability
clean; `briefing_status.py` self-test OK and check current; `doc_staleness_audit.py` self-test OK and clean; the ARM
header gate exit 0 (255 headers); `ctest -R compile_refusal` **289/289**; `src_build_gate.py self-test` **OK (377
detector cases)** and `check` **PASS (14 builds; robot one's 6 refuse naming exactly their owed facts and link in the
gate's typed copy)** — both re-run after the incident below, each followed by `git diff -- Makefile src/`: EMPTY; then
the suite REBUILT (the gate's restores re-touch `src/`; 307 objects) and re-run: green, 3675 cases; the briefing check
current. **No API change** (nothing under `include/` or `src/`); **no HA row**.
- **An incident, caught before any result was taken:** the first `src_build_gate.py self-test` of this round ran beside
  FOUR other trees' gates (af1s-a2, docs3-n1, rpv1-s1, af1s-a2's review copy — `ps` + `/proc/<pid>/cwd`; none of them
  this run's) and hit its `timeout 1800` — whose SIGTERM ends Python WITHOUT running the tool's `finally`, so it LEFT A
  PLANT: `src/build_selection.hpp` changed (8 lines in, 14 out — the `kRobotId` line of each variant removed; not marked "(self-test plant", so the
  marker grep alone would not have seen it; `git diff -- src/` did). The `check` that had just started on that planted
  tree was stopped (SIGINT) and its result thrown away; the file restored with `git checkout -- src/build_selection.hpp`
  (this branch never edits `src/`) and touched; nothing of mine had been built from `src/` in between (the suite, the
  ARM gate and the refusals ran before it). Both re-run with `timeout -s INT` (the tool's `finally` runs on SIGINT).

**For you (the coordinator):**
1. **D20: push `rmp1-a1`** so the Windows CI runs — the page tests now take every A1 screen in four variants on every
   row (ROUTE_MAKER_SHOTS is set there): about +4 min a row. Read each row's `the ONE FILE over file:// in <browser>:
   isSecureContext=…, showDirectoryPicker=…` line (msedge and chrome are the A.10 question's CI half).
2. **The rebase onto main (e9bcee7: RPV1-A, MIRROR1-A, API 2.9):** this run's changelog entry moves from the top of
   `## API 2.8` to `## API 2.9`; re-read the shared lines both sides touched (guide 14, guide README, `test/README.md`,
   `test/CMakeLists.txt`, `RELEASING.md`, `mkdocs.yml`); GL18's four twins move to robot y's stall-homed `swing`
   through the now-merged `test/fakes_by_kind.hpp` (delta 23).
3. **At ROB1-BT's merge (whichever lands second):** give robot two its `.ledgerLevel` (`robot_two_facts::lv`) in
   `test/route_maker_robots.hpp` — the tripwire goes red until then; the robot-two pin ("claw pivot 6") changes; the
   GL0-against-the-real-file test stops skipping and must pass; HS9 (every export into a scratch tree, the suite and
   the src gate there) is yours (delta 3).
4. **RELEASING.md's OPEN paragraph:** D17's publish at A1's merge vs the between-parts rule — decide (a) or (b) before
   the merge, and write it there.
5. **Guide 2's figure is off by about 1 in near the walls** (an earlier chunk's flaw, to be fixed THERE):
   `tools/docs_figures/field_coordinates.py` uses `PX_PER_IN = 7.5` and centre `(565, 570)` on the turned source; the
   measured transform is **7.6241 px/in, origin (570.0, 573.5)** (`tools/route_maker/assets/field.json`, its walls at
   35 / 1105 / 38 / 1109 px; FP1 and the new FP3 hold it) — the red wall ~1.5 in off. Reuse field.json's transform
   there and re-render `docs/assets/field-coordinates.png`.
6. **The manual Windows check** (`RMP1-A1-manual-checks.md`): step 7 now asks for the folder write from the DOWNLOADED
   file in Edge — the one thing no test here can do.
7. **Polish for RMP1-C:** at 1024 wide the step drawer, when open, covers the right end of the field's toolbar (Snap);
   the toolbar scrolls, and closing the drawer shows it — not a defect, a layout to improve.
- **Processes:** every Playwright Firefox and Chromium this round started was closed by its test's `after` / `finally`;
  `ps` after each run shows none under `~/shulib-render-tmp/`. No driver used. The other trees' src gates seen running
  (af1s-a2, docs3-n1, rpv1-s1, af1s-a2's review copy) are theirs — not touched.
- **Stopping here** for the fresh verifier and the coordinator's checks. Nothing committed, nothing pushed.

### Verification-round findings and their disposition (2026-11-15 ~23:30 – 2026-11-16 ~01:30 EST, `date`)
A fresh verifier returned two findings. Each was RE-CHECKED on my own build of the tree before anything was touched
(the verifier's probes copied to scratch `vround/probes/`, pointed at `build/route_maker_site`), fixed at the root,
given a test, and the test proven by a mutation observed RED (below). Nothing committed, nothing pushed.

1. **A project file whose start has a hole crashed the page, and the saved draft kept it from starting** (major) —
   REAL. Reproduced in Playwright's Firefox for both `"x": null` and a start with `x` left out: "Not opened: fmt: not a
   finite number (undefined)…" although the file HAD been opened, then after a reload "The route builder did not start"
   and a page error. **The root:** review fix 4 made a start with a hole a stored state of the model (RT6), but the page
   still assumed a whole start — the step list printed it with `fmt` (the throw), the field drew the robot and its knob
   from it (NaN coordinates), a curve being clicked was drawn from it, and `plannedPoses` planned every step from a pose
   with no `x`; and Open (and the folder's file at connect) KEPT the route — the draft and "last saved" — before drawing
   it, while `main()` had no way back from a draft it cannot draw. **FIXED:**
   - `route.mjs` `startPose()`: the start as a pose only when x, y and the heading are all numbers; `plannedPoses`
     plans from it, so a start missing a part plans nothing (as no start does) and RC3 is its one error.
   - `words.mjs` `startTitle()`: `?` for a missing part (the same `pt()` / `n()` the steps use); `step_list.mjs` draws
     the start through it, and the start card's accessible name now says the start and "(has an error)".
   - `field_view.mjs`: the robot and its knob, and the start of a curve being clicked, come from `startPose()` only.
   - `app.mjs`: the field's hint says "Robot one's start is missing a number: select the start (the top of the list)
     and type it". **Found while fixing, the same root:** `setStart` took a typed part only when all three were numbers,
     so a start missing TWO parts could never be filled in from the inspector; it now takes each part as it is typed
     into a start already missing parts, and an emptied box still never MAKES a hole.
   - `app.mjs` `adoptProject()`: Open and the folder's file are DRAWN first and kept (the draft, "last saved") only once
     on screen; a route the page cannot draw puts everything back (the project, the history, the folder's base, the
     selection, the save state), draws again and says "Not opened: this page cannot show the route in that file (…).
     The page is as it was." (or "src/team/route_maker.json in the folder was not opened: this page cannot show its
     route (…)"). **Found while fixing:** at connect the page had already taken the folder's text as Save's base, so
     after a failed adoption Save would have replaced a teammate's file the page never showed — the base is now `null`
     there and Save ASKS; and "Opened src/team/route_maker.json from the folder." is said only when it was.
   - `main()`: a draft that cannot be drawn opens a NEW route, clears the draft and says "Your route from last time
     could not be shown (…), so a new route is open. Your project file is not changed: Open your route_maker.json to
     carry on." A failure with no draft is the page's own and still shows "The route builder did not start".
   - Two edits made and then REVERTED as equivalent (no behaviour differs, so not claimed as fixes): `fieldClick`'s
     `from` (a missing x gives `bearingDeg` NaN, and the `Number.isFinite(leg)` guard already left turn-first off) and
     the heading knob's drag (the knob is drawn only for a whole start).
   - Guide 22: the checks table's "The start" row (a missing number, shown as `?`, the robot drawn once typed) and the
     draft paragraph (a draft the page cannot show: a new route, and it says so).
   **Tests:** node — "RC3: a start MISSING a part" (each of x, y, heading left out: exactly one error, RC3; nothing
   planned — `[from, to]` null for both steps; the drive and the face are placed 0.5 in from where a hole read as 0
   would put the robot, so a hole taken as 0 would ALSO raise RC1; `startTitle` with `?`); page — "a project file whose
   start is missing a number … opens" (the verifier's suggested test, and more: `?` in the list, the red edge, RC3 in the
   inspector, NO robot drawn, the hint; a reload STARTS and shows it "Saved"; x typed → whole, drawn, no error; a second
   file with TWO parts missing and no step: a curve clicked there is drawn through its two points only (no NaN), Esc;
   then x kept while y is still missing, y typed → whole; an emptied x refused); page — "a route the page cannot draw
   (a fault the TEST puts in)": no input reaches the defensive paths once the hole is drawn, so an init script makes
   the n-th `<ol>` the page creates throw (the step list is one). Open → the error, the page's own route, "Unsaved
   changes", and the browser's stored draft and "saved" read AT ONCE (a closed tab or a crash runs no `beforeunload` to
   put them right); a reload → the page's route; a draft that cannot be drawn → the page starts, a new route, the
   toast; the folder's file at connect → its error, the empty page kept, nothing stored, and Save ASKS (Cancel leaves
   the file). The verifier's own probe, re-run on the fixed build: both cases "Opened the project.", the reload ready,
   no page error.

2. **No rename in the page, and a name RC21's other rules refuse got no fix** (minor) — REAL (the verifier's
   `v3_names.mjs` re-run: `NONE`, `a"b`, 27 characters, empty, `50%` all `fix=NONE`). **FIXED** in `checks.mjs`: any name
   but `"skills"` puts **Name it "skills"** on its RC21 error, whichever rule raised it (the library's rules are one
   if / else chain, so there is at most one such error), and the page's own sentences no longer ask for a name the page
   cannot take ("use 1 to 29 …" → "it is 1 to 29 …"; "pick another name" → "so no route can have that name"; the
   library's own quoted sentence still says "pick another name" — it is the header's text, shown as the library's).
   Guide 22's name row: "a project file with any other name, even one the brain could not show, offers Name it
   "skills"". Node: eight names (`NONE`, `a"b`, `a\b`, empty, `50%`, 26 mirrored, 30 not mirrored, `café`): one RC21,
   the library's (not the contract's), the fix offered, no "pick another" / "use 1 to", and after the fix no RC21. The
   probe re-run: `fix=nameSkills` for all six. **Seen in the UX look at that screen (a screenshot of a file named
   `NONE`):** the inspector's name line under the error still said `"NONE" — both robots' code registers the skills
   route by this name, so it stays` — false for any name but `"skills"` (it was so for `"skills A"` too). It now says
   `"NONE" — not "skills", the name both robots' code registers the skills route by: Name it "skills" above.` Page test
   "a project file with a name the page does not take": open a `NONE` file → one error, RC21, in the page's words, the
   name line, **Name it "skills"** clicked → named, no error, the top bar and the line say "skills".

3. **Found on the way (not a verifier finding): a visit with no edit was announced as unsaved work.** Probe
   `vround/probes/empty_draft.mjs`: open the page, reload — "Draft kept in this browser" and "Your unsaved route from
   last time is back" for an EMPTY route (`beforeunload` keeps the new project as the draft, and nothing was saved to
   compare it with). FIXED: a draft that is only a new, empty project (a visit with no edit, or every edit undone) is
   nothing to bring back — the page starts as "New route". The draft page test now begins with that reload.
4. **NF1's helper `everyNumberStored` counted a hole as a number off the grid** (it pushed `undefined` for a start's
   or a curve point's missing part) — test-only, and no test ran it on a route with a hole before; it now skips a part
   left out (a stored NaN is still refused). Node test "NF1: a HOLE … stores no number".

**Mutations of this round** (scratch `vround/mutate3.py`: exact edits, backups under `vround/backup3/`, restored in a
`finally` and on SIGTERM/SIGHUP/SIGINT, each restored file touched and md5-checked — and the whole set md5-compared
with a list taken before the run: byte-equal; every node run under `ulimit -v 4 GiB` + `timeout`; for every page
oracle the site REBUILT from the mutant; logs in `vround/mut3-logs/`). **20 mutants, every one RED by the intended
assertion** (its failing line read in the log; the page's line numbers are those of the test file when each ran —
the name test was added after the first series, so later lines moved):
- V1 (the step list back to `fmt` on the start) — page `:498` "Opened the project." never comes (the open is put back);
  V1b (`startTitle` without `?`) — node `checks.test.mjs:137`; V2 (the field draws a start with a hole) — page `:503`
  "not drawn"; V3 (the plan from a start with a hole) — node `:135` "no pose planned from it"; V13 (no hint) — page
  `:504`; V16 (a curve drawn from a start with a hole) — page `:529` "the two clicked points only";
- V4 (Open keeps the draft BEFORE drawing — the verifier's order) **SURVIVED the first form of the test**: the page's own
  `beforeunload` re-wrote the draft from the restored route before the reload's check. The test now reads the stored
  draft and "saved" at once → RED at page `:583` "the draft is still the page's route: kept only once drawn". V5 (no
  put-back) — page `:579`; V6 (no fallback for a draft) — page `:592` "the page started"; V11 (the folder's base kept
  after a failed adoption) — page `:613` (Save does not ask); V12 ("Opened … from the folder" always) — page `:603`;
- V8 (`setStart` back to all-three) — page `:534` "the first part kept while the other is still missing"; V8b (an
  emptied box makes a hole) — page `:541`;
- V9 (the empty draft announced) — page `:467` "New route";
- V7 (the fix only on the contract's error) — node `checks.test.mjs:289` "Name it "skills" is offered"; V7b ("pick
  another name" back) — node `:290` "words the page cannot act on"; V7p (V7 again, under the PAGE's name test) — page
  `:558`, the button is not there; V17 (the name line always "so it stays") — page `:556`;
- V10a / V10b (`everyNumberStored` back to the old start / curve-point code) — node `numbers.test.mjs` "holes are not
  numbers off the grid". **Re-cut, not counted:** the first V10 also pushed a curve point's OPTIONAL heading (harsher
  than the old code) and went red on two unrelated NF1 tests too; it was re-cut into the two exact old forms and re-run.

**Prose review (rule 4), run and read** over this round's files (guide 22, `route-builder.md`, guide 14, the changelog,
`tools/route_maker/`, the manual-checks page, `test/README.md`): `pick another name` 1 — the library's own quoted text
(`library_words.mjs`), the page's sentence no longer says it; `use 1 to` 1 — the test's `doesNotMatch`; `Not opened` /
`could not be shown` — the new toasts and their tests, each read; `missing a number` 6 — the hint, guide 22, the tests;
`did not start` 2 — `main()`'s own; `simulat` / `honest` — guide 14's and the changelog's own lines and the checklist's
"no "honest" disclaimers" line, none of this round's; `obviously` / `simply` 0; `easy` 1 (the changelog's, not this
round's).

**The gates, final (2026-11-16 ~01:30), on the tree as it stands** — after every mutant restored (the whole set
md5-compared with lists taken before each series: byte-equal), the site rebuilt from the restored tree:
`node --test tools/route_maker/test/*.test.mjs` — **81: 80 pass, 1 skipped** (GL0 against the real skills-route file:
not on this base); the page tests on the final build — **Firefox 16 pass; Chromium (Playwright's, under the cgroup cap)
14 pass, 2 skipped** (the download way and the old-browser box are Firefox's); `build.py` — both forms (app.html
120345 B + `src/`; the one file 368453 B); the pages build locally (pinned mkdocs 1.6.1 + material 9.5.44):
`mkdocs build --strict` clean, PG1 and the internal-docs guard pass; the CI guards (PROS-free, sim-free) and the doc gates
— `api_doc_tool` self-test OK, check-coverage (6386 entities, 211 headers), check-fresh, check-examples (860 lines, 18
sources), check-removability clean; `briefing_status.py` self-test OK and check current; `doc_staleness_audit.py`
self-test OK and clean; the ARM header gate exit 0 (255 headers); `ctest -R compile_refusal` **289/289** (289 TUs);
`src_build_gate.py self-test` **OK (377 detector cases)** and `check` **PASS (14 builds; robot one's 6 refuse naming
exactly their owed facts and link in the gate's typed copy)**, each followed by `git status -- Makefile src/`: EMPTY;
then `build/test/shulib_tests` removed, the briefing regenerated, the suite REBUILT (307 objects: the gate's restores
re-touch `src/`), the briefing regenerated with the built suite and the build re-run (0 warnings, the doc gate green),
`suite_run.py green` — **3675 cases, 3862746 assertions**; the briefing check current. **No API change** (nothing under
`include/` or `src/`); **no HA row**.
- **An incident, caught before any result was taken (the coordinator's lesson, again):** the first `src_build_gate.py
  self-test` of this round ran ALONE (no other tree's gate in `ps`) and still passed its `timeout -s INT 1800` — it needs
  about 34 minutes here (the re-run: 00:33:43 → 01:07:56). GNU `timeout` (without `--foreground`) sends its signal to
  the tool AND to its whole process group, so Python took TWO interrupts: the second landed INSIDE the plant's own
  restore (`_plant.__exit__`), and a plant was LEFT — `src/programs/competition_program.cpp` + 2 lines
  (`using GateRob1Driver = shulib::bench::RobotTwoDriver;`, no "(self-test plant" marker, so the marker grep alone shows
  0; `git status -- src/` showed it). Restored with `git checkout -- src/programs/competition_program.cpp` (this branch
  never edits `src/`) and touched; nothing had been built from `src/` since the suite build before it (the ARM gate
  compiles `include/` only; the refusals ran before it; only the node tests ran during it). Both gates were re-run with
  `timeout --foreground -s INT 3600` (one signal, to the tool only) — passed as above, `src/` clean after each.
- **A Chromium flake, once:** the FIRST full Chromium page run of this round failed the existing test "a route built by
  clicking" at the redo right after an undo (`:354` then, `steps[0].boundS` undefined, expected 3) — nothing this round
  touched (undo, redo and the keyboard are unchanged). Not reproduced in 3 runs of that test alone and **19 full runs
  since** (15 in a row in one loop). The likeliest cause is focus: the redo is skipped when the focus is in an input
  (the "typing" rule), and "Home it first" focuses its time-limit box from a `setTimeout`; a late timer under a cold
  first launch would explain it, but it was not caught in the act. For you: watch that line on the CI rows.

**Processes:** every Playwright Firefox and Chromium of this round was closed by its test's `after` / the probes'
`finally` (`ps` after the last run: none of Playwright's under `~/shulib-render-tmp/`; the `/snap/firefox` processes in
`ps` are the laptop's own desktop Firefox, not started here and not touched). No driver used.

**For you (the coordinator), from this round:**
1. The src gate self-test needs ~34 min alone here: a `timeout 1800` kills it, and a plain `timeout -s INT` interrupts
   it twice (the process group), which can leave a plant WITHOUT its marker — use `timeout --foreground -s INT` with
   room (3600), and read `git status -- Makefile src/`, not only the marker grep.
2. Watch "a route built by clicking"'s redo line on the CI's Chromium-engine rows (the flake above).
3. Polish for RMP1-C (seen in the UX look, not a defect): the empty box of a start's missing number could carry the
   red "bad" mark the time-limit boxes get, beside the RC3 error that already names it.
- **Stopping here** for the coordinator's checks. Nothing committed, nothing pushed.
