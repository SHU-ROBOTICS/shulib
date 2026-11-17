# RMP1 — PROGRESS (live log)

<!-- release: between parts — RMP1-A1, RMP1-B (Controls + feel) and RMP1-A2e (the PREVIEW engine) landed 2026-11-16/17; RMP1-A2u (the PREVIEW screen) is building (declared by the coordinator 2026-11-17) -->

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

## RMP1-A1 — the coordinator's merge verification *(2026-11-16)*

- **Squash-merged from `rmp1-a1`** (07d17b7, base 49822e5; its Windows/Linux/ARM CI green on the pushed branch, run
  43816687690) onto `531c3fd` in the coordinator's INTEGRATE worktree. Conflicts: the guide index (chapter 22 added
  beside chapter 21), the changelog (the entry moved to the top of `## API 2.9`), the briefing (regenerated).
- **Four of its own tests met ROB1-C2's robot one, fixed in the merge (test-only):** (1) the robot-one smoke rig faked
  only the cascade → now RPV1's `FakesByKind` over robot one's whole list, with a no-op inner pacer (its `pace()` calls
  `inner->pace()` — the first try SEGFAULTED the suite without it); (2) `robots/x,y,z.json` described api 2.8 →
  regenerated (only that line differed); (3)+(4) two tests assumed robot one's cascade has ledger levels — its levels
  are NONE (manual) since ROB1-C2 → each NAMED level is checked when it exists, and the tripwire skips an empty preset
  list. The smoke golden route moved off `cascade[facts::lv(2).name]` to the claw and the right toggle
  (`goldens.mjs --write`: only `smoke_robot_one.*` changed). **Consequence, told to the team lead:** until robot one's
  levels are recorded at station 13, the builder can home its cascade but not move it to a level.
- **Gates:** suite green (**3,778 cases**); full build with the doc gates; compile refusals 295/295; api check-examples /
  fresh / coverage / removability; staleness; briefing current; the src gate self-test (377) and check (14 builds) and
  the ARM header gate on this merge's first pass; `node --test` **80 pass**; `build.py` built the site form and the one
  file. **The coordinator's JS mutations** (`muts_rmp1_a1_js.json`, runner `scratch/rmp1a1/jsmut.py`): **3 of 3 RED** —
  MJ1 RC25 dropped, MJ2 turn-first exported without its `face()`, MJ3 several marker pairs accepted.
- **RMP1-A1 LANDED.** Publish with the next docs release (D17).

## RMP1-B

**Base:** worktree `.claude/worktrees/rmp1-b`, branch `rmp1-b`, at `07d17b7` (RMP1-A1's finished branch, pushed; main has
moved to `3c29b73` — ROB1-C2's robot one map and the one-button `whileHeld` read from there with `git show`). Started
2026-11-16. Executor model: `opus`. Nothing committed and nothing pushed by the executor.

**Order of authority read:** `## ANSWERS` (Go, accept all; D15 (a) the driver files' controls/feel regions stay open;
D17, D19, D20); the amendment A.1–A.12 (A.7's RMP1-B row); R1–R19; the Decisions D1–D20 in full; §1.2 (RMP1-B), §1.4's
Controls-tab parts, §2.2's RMP1-B list (CT1–CT31, CT-HS, CT21–CT23, FE1–FE4), §2.3 (RM3, RM5, RM17), §2.4, §2.5, §3's
RMP1-B row, §5, §6 (with §6.3 item 5 and §6.4), §7 and §8 (landmines 1–30) in full; the rest by heading.

### Log
- Created this heading first. Read `docs/internal/RESUMING.md`'s standing rules.
- **The base this diff is made against (the coordinator applies it onto A1 merged with main):** the controls golden
  must compile robot one's REAL map (ROB1-C2's one-button `whileHeld`, main `3c29b73`), and §6.4's re-points edit
  main's `test/robot_one_driver_test.cpp`, so the work cannot be built on `07d17b7` alone. So the INDEX holds A1
  merged with main — `git merge-tree --write-tree 07d17b7 3c29b73`, read in with `git read-tree -u -m HEAD`, nothing
  committed, no MERGE_HEAD — with its three conflicts resolved and STAGED: `docs/guide/README.md` (main's row 21, A1's
  row 22), `docs/changelog.md` (A1's route-builder entry, then main's RPV1 entry), `docs/internal/PROJECT-BRIEFING.md`
  (main's lines + RMP1 in the open list; regenerated later). That staged tree is `b02ff5bb33d0c5bff21aa722f41b54673c659494`
  (saved in scratch `BASELINE_TREE`). **Everything UNSTAGED (and every untracked file) is RMP1-B's own diff**; the final
  patch is written to scratch `rmp1-b.diff` against that tree.
- **A1 onto main does not build as merged — three integration faults, fixed in the BASELINE (staged), not in this
  diff** (saved as scratch `a1-onto-main-merge-fixes.diff`; the coordinator's merge needs them or its own): (1)
  `smoke_robot_one` moved the cascade to `facts::lv(2)` — ROB1-C2 typed robot one's levels as NONE, so the export
  no longer compiled (`no_such_preset_on_this_mechanism`); the smoke route now homes, drives, `set`s the claw open,
  curves and waits (re-made by `goldens.mjs --write`); (2) A1's rig gave robot one's service ONE device (the cascade)
  for SIX mechanisms (`MechanismService::bind: one Devices per spec`) — `OneFakes` now has all six in table order; (3)
  the fixture descriptions said API 2.8 (main is 2.9: MIRROR1-A) — re-made; and robot one's description test /
  the ledger tripwire assumed a level exists — each now holds only once one does. After them: `shulib_tests
  -tc="RMP1*"` 14/14 green, node 81 pass 1 skip. The baseline tree is now `97d48bacf87eb62917d380140826454837370139`.
- **The model (pure, node-tested):** `src/model/feel.mjs` (the library's `continuousDeadband` / `expoCurve` /
  `shapeAxis` in the C++'s order; `checkFeel` = `AxisShaping::valid()` plus the page's stricter ramp rule; the
  `feel` region written and read back), `src/model/controls.mjs` (the controls as data; `checkControls` — one rule
  per build refusal of `controls()` in its order, the two kinds' missing-target refusals, DriverFile's re-zero; the
  `controls` and `control words` regions written and read back; G3a's per-kind save shape with `at`; the edits),
  `src/model/driver_file.mjs` (the three regions found — by markers, or by their SHAPE for reading only —, read,
  written all-or-none with the hand-edit hashes and the file's own line endings). **The round trip, probed first on
  main's own files:** robot one's and robot two's driver files at `3c29b73` read and re-made BYTE-IDENTICAL, all
  three regions (one bug found by it: the last control written `)));`).
- **The markers (§6.3 item 5) are NOT on main:** the page READS today's files by shape and refuses to write
  (`noMarkers`, Copy offered). The fixture is "the marked file": `test/goldens/route_maker/driver_{one,two}_marked.hpp`
  = main's file + four marker lines — `// [control words <id>]` / `// [/control words <id>]` around the header's
  stick-and-button map (from `//   left stick` to the `let go of the sticks` line) and `// [feel <id>]` /
  `// [/feel <id>]` around THE FEEL heading through kLoop's `};`. **The coordinator must give ROB1-BT exactly these
  four lines** (they are one table, `markers()` in `driver_file.mjs`).
- **FE1:** `shulib_route_maker feel` (new `test/route_maker_feel.hpp`) writes the 9 630-point table — **byte-identical
  to the brief's probe P2's `p2_golden.txt`** (`cmp`); committed as `test/goldens/route_maker/feel_curve.txt`.
- **D10:** `src/common/driver_file.hpp` gains `static_assert(Feel.valid(), "the feel is out of range: ...")` and its
  fixture `test/compile_refusals/rm01_a_feel_out_of_range.cpp`. (Lands only if B merges before the ~Nov 18 freeze —
  the brief's D10; it is one separable hunk.)
- **The CT agreement:** `tools/route_maker/controls_cases.mjs` — 39 configurations on robots x and y and one forced
  robot w (a never-homed arm with no unhomed volts, which no fixture has), one per build refusal: every
  `refused::` name of `driver_inputs.hpp` (30 since ROB1-C2's one-button hold), all twelve "does two jobs", both
  missing-target names by kind, the re-zero and the feel static_asserts. `goldens.mjs --write` makes each a
  compile-refusal TU `test/compile_refusals/rm_ct_*.cpp` whose `// expect:` is the PAGE's refusal: **ctest -R
  compile_refusal_rm: 40/40 refused with the page's name** (+ n57, d_button_not_one_of_the_twelve). The 13th button
  cannot come from the picture: its twin is MECH-A4's `d_button_not_one_of_the_twelve.cpp` (node checks it). Node:
  the page refuses each case with exactly that one name (the 25-control case may add "two of a kind", as the build
  does); a coverage test fails when the library grows a refusal with no case.
- **CT21 / FE2 / FE3 in the suite:** `test/route_maker_controls_test.cpp` (new) includes three page exports on the
  fixture robots (`controls_*.hpp`, every form, a block-form kFeel, Brake and Hold) through DriverFile, and
  `feel_valid_rows.hpp` (25 static_asserts, each the page's answer). Built clean with `-Werror`.
- **RESUMED 2026-11-16 08:55 EST** (`date`) after the usage stop at ~03:00. Checked first: `git diff -- src/ Makefile`
  holds only D10's `driver_file.hpp` hunk (no `(self-test plant` line); `git ls-files -o --exclude-standard src
  include test` lists only this run's own new fixtures and goldens; the index is still the baseline tree
  `97d48bac…` (`git write-tree` = scratch `BASELINE_TREE`). Rebuilt from source before trusting anything: `rm -f
  build/test/shulib_tests`, the briefing regenerated, `cmake --build build/test --clean-first` (a killed mutation could
  have left a mutant OBJECT behind a restored source). Found already written by the stopped run but not yet logged:
  the Controls tab (`ui/controls_tab.mjs`, `ui/feel_tab.mjs`, `ui/controls_export.mjs`), the tab in `app.mjs`,
  `build.py`'s baked driver files, `files.mjs`'s `readTeam` / `writeTeam`, the page tests
  (`test/page/controls.page.test.mjs` + `harness.mjs`) and a first set of screenshots (02:57). Not yet done: §6.4's
  re-points (T11, GM22), CT-HS, the docs, the mutations, the reviews.
- **Fresh state after the resume (09:00):** the clean rebuild 0 warnings; `suite_run.py green` — 3778 cases,
  3 882 904 assertions; node (model) 100 pass 1 skip; `build.py --out build/route_maker_site --describe` made the site
  and the one file; the Controls page tests in Playwright's Firefox: **5/5** (`controls.page.test.mjs`).
- **A1's OWN page tests are RED on the merged base (6 of 16), not by this run:** `page.test.mjs` tests 3, 5 and 11
  (and the three that share `readyRoute`) select "a cascade level" (`select[name="preset"]` index 3) — robot one's
  levels became NONE in ROB1-C2 (main `3c29b73`), so the select has no level and times out. It is the fourth
  A1-onto-main integration fault (with the three above); it is A1's test design (a level on robot one), so it is
  left to the coordinator's A1 merge — not fixed in this diff. **For the coordinator.**
- **§6.4 — the pin tests re-pointed (`test/robot_one_driver_test.cpp`, main's ROB1-C2 file):** a student's map
  change (D15 keeps it open; the page writes it) turned SIX of its seven cases red, not two: T11's map, GM22, BOTH
  host tests (they pressed R1/R2/L1/L2/A/B/X by name), the compact-row review case (R1/R2) and the boot-line case
  (L1/L2, `seen == 2`); the seventh, the compact row's SMALL-budget case, names no button and stayed green. *(Count
  corrected in the review round: this line first said "FIVE" and listed six.)* The brief's §6.4 names only T11 and GM22 (written before ROB1-C2 added the others) — **a delta,
  logged:** every one now reads its buttons from `kControls` (the team's) and keeps its behaviour checks; T11 pins the
  RULES (re-zero free; `preset(cascade` only through `facts::lv(`; each side toggle on its own port); GM22 is
  structural in C++ (every button `kControls` takes has ONE row in the header's map; every other button row says
  FREE; the d-pad not FREE while one of its buttons has a job; `(start)` iff a driver-start home), the rows' ORDER
  against `kControls` CT23's (node). *(Corrected in the review round: this said "the WORDS against `kControls`
  staying CT23's" — wrong: CT23 reads the map's words back from the same file, so no test judges them; see the
  review round's finding 17.)* The boot line is also pinned on a one-button hold made in the test (L1 on the
  right toggle, 12 V) so it never depends on the map. Before: `edits/robot_one_driver_test.cpp.before-6.4`; the
  edit script `edits/repoint_6_4.py`. `-tc="robot one driver*"`: 7/7, 102 assertions.
- **UX fixes found on the 02:57 screenshots (re-taken, opened):** (1) a free button's menu opened at the page's
  top-left corner, over the controller — the click re-draws the controller, so the anchor the menu measured was
  detached (`getBoundingClientRect` = 0); now measured on the NEW button and placed beside it, and the page test
  asserts it opens beside and level with the button; (2) the menu repeated each mechanism's name on every action —
  now grouped under each mechanism (each item's accessible name still "claw: Go to one position…"); (3) the
  controller's labels were cut at 28 characters ("L2 · left toggle: hold to r…") and ~8 px tall — 14 px, up to 32
  characters, the drawing widened; (4) the d-pad's names overflowed their circles — arrows; (5) the "In the button
  map" box clipped its second line.
- **The ramp's words were WRONG, and are fixed:** the page said "how quickly the robot may speed up"; the library's
  slew limits the request "in either direction" (`slew_limiter.hpp`), so with a ramp the robot does NOT stop the
  instant the sticks are let go. `rampWords(4)` = "full speed from rest in 0.25 s, and a stop from full speed takes
  as long" (FE4 re-pointed; the feel comment it writes says the same); guide 22 says it.
- **CT-HS run** (scratch copy `scratch/rmp1-b/cths/tree`, rsync of the worktree; `cths/make_cths.mjs` writes the
  page's exports through `driver_file.mjs` into both driver files — robot one: the claw's toggle A → Up (the "d-pad
  FREE" line removed as the page offers), the cascade's hold swapped (R2 up / R1 down), the feel at dead zone 0.1,
  curve 0.6, ramp 4/s; robot two: the same feel). It found THREE things that stop a student's export reaching the
  robot — **for the coordinator, each outside this run's files:**
  1. **`src/programs/drive_program.cpp:265-281` — EVERY robot build stops on ANY feel but the library's default**
     (`static_assert(driverLineMatchesFeel(), "the driver line says deadband 0.05, no expo, no slew: reword it…")`,
     the boot's driver line being fixed text). The src gate: all 10 driver-file builds `ERROR … static assertion
     failed` (`cths/srcgate-first.log`). The Driving feel cannot be used until the line is MADE from `kFeel`. A
     tested patch (NOT in this diff — `src/programs` is a robot program, the brief allows B only D10's line):
     `scratch/rmp1-b/drive-line-feel.patch` (a `feelLine()` from the profile, logged as its own boot line right
     after the driver line; the static_assert gone; `drive_program_source_test.cpp`'s pin re-pointed and a pin that
     the line is made from `kFeel`). It is library-freeze material: it must land BEFORE the ~Nov 18 freeze, with the
     team lead's OK, or the D15 (a) feel region is open in name only.
  2. **The doc gate (`api_doc_tool` check-examples, the whole-run rule) stops the suite's BUILD** after any change to
     a quoted driver-file run: guide 21 :261-269 quotes robot one's `kControls` whole; cookbook 06 :247-261 robot
     two's `kControls`, `kFeel` and `kLoop`. So after the freeze every student export needs those listings
     re-quoted (the coordinator's step; done in the scratch tree by `cths/requote.py` to get past it), or the two
     chapters move to quoting an example test. ROB1-BT rewrites robot two's driver file (cookbook 06) anyway.
  3. **`test/robot_two_driver_test.cpp` T5 and T6 pin robot two's feel as the library's default** (T5: the ports
     equal a direct run with `DriverProfile{}`; T6: let go → 0 V the same tick — false under a ramp, which slows the
     stop): a feel export on robot two turns both red (480 + 55 assertions). ROB1-BT's file (it rewrites robot two's
     driver test with robot two's map) — re-point: the ports equal a direct run with the FILE's `kFeel`; let go → 0 V
     within `1/ramp` s.
  With robot one's export (controls and feel), robot two's feel left as it is, guide 21 re-quoted and the drive line
  patch applied, the scratch suite and src gate are re-run below.
- **CT-HS, re-run** (robot one's export — controls and feel —, robot two's feel left as it is, guide 21 re-quoted,
  the drive line patch applied; the scratch briefing regenerated with the binary removed, then built): build 0
  warnings, **suite green — 3778 cases, 3 882 885 assertions** (robot one's re-pointed tests pass the team's
  changed map). The src gate on it: below.
- **Mutations — the model (node), each applied, run fresh, RED observed, restored by
  `scratch/rmp1-b/mut/mutate.py` (restore in a `finally` and on SIGTERM/SIGHUP/SIGINT; `all restored: True`):**
  RM3 `expoCurve` without `(1 − expo)` → FE1 red; RM5 the check misses a button used twice → the CT agreement red;
  RM17 the ramp readout per tick (÷100) → FE4 red; the ramp words without "a stop takes as long" → FE4 red; the dead
  zone applied AFTER the curve → FE1 red; `axisValid` refusing an expo of exactly 1 → FE2 red; the re-zero rule off
  → the CT agreement red; the splice eating the END marker → CT22, "writes only between the markers" and the
  mixed-endings test red; CRLF dropped → both line-ending tests red. (`mut/node-run.log`.)
- **Mutations — the page and the C++ (same driver):** the menu measured on the drawing a click replaced (the old
  bug) → the "free button" page test RED in Firefox (first run SURVIVED: its "beside it" check let a menu at the far
  left pass — tightened to a gap of at most 24 px, then RED); GM22's map row for X deleted → GM22 RED; the ramp's
  toggle moved X → Up with the map still saying "d-pad FREE" → GM22 RED (each on a fresh `shulib_tests/fast` link —
  the doc gate would refuse the mutant driver file first, finding 2 above —, restored, re-linked, 7/7 green).
- **CT-HS, the src gate on the scratch tree** (the drive line patch applied): **all 14 builds pass** — bench, xdrive,
  tank and its five programs, `one` and `one-calibrate` exit 0 with 0 disallowed warnings; `one-drive`,
  `one-autontest`, `one-competition`, `one-tune` REFUSED as they must (exactly their 9 owed facts). Its one FAIL is
  the facts-copy proof, which copies `git ls-files` — the scratch copy is not a git checkout (it sits under the
  ignored `.claude/`), so the copy was empty; the worktree's own src gate runs that proof. Without the patch the
  same gate failed all 10 driver-file builds (finding 1).
- **The re-point is what makes a team map pass (observed):** the PRE-§6.4 `robot_one_driver_test.cpp`
  (`edits/…before-6.4`) on the scratch tree's exported map: **6 of 7 robot-one cases RED** (T11's map, GM22, both
  host tests, the compact row, the boot line); the re-pointed file on the same map: 7/7, 102 assertions.
- **Reviewer 1 (the `code-review` skill, "high", forked; read-only; it ran the model in node under the cap) — 10
  findings + 1 note, each re-checked here and fixed, each fix with a test that FAILS without it (mutated, RED,
  restored — `mut/review-run.log`):**
  1. *(reproduced)* a one-line `kFeel` with an axis carrying BOTH a dead zone and a curve could not be read back
     (the split at `, .` also split inside the axis's braces) → `topFields()` splits only outside braces; test:
     every feel the page writes over a 64-feel grid reads back to the same lines (both forms) — mutation R-topFields
     RED.
  2. *(reproduced, the worst)* a feel or map the page could not read still got a base hash, so the next export
     replaced the team's numbers with the defaults WITHOUT asking → a region not read gets no base (Keep /
     Overwrite is asked); test + mutation R-read-flag RED.
  3. GM22 (C++) skipped A, L1 and L2 (the loop's `b == c.back` test caught the FIRST button when `back` repeats it
     or defaults to A) → the buttons checked one by one; mutations: the A row deleted RED, the L1 row deleted RED
     (below, after the src gate).
  4. the page writes a combined map key ("R1 / R2" — robot two's file has them) that the C++ GM22 did not read →
     GM22 now reads keys as the page does (`/`, `,`, "d-pad"), and states the rule both ways: a row is FREE exactly
     when its buttons have no job and are not the re-zero.
  5. `keep()` rewrote a LOCKED robot's project block (one from a newer builder) → skipped; page test (a project
     whose robot two has a field this builder does not write; robot one edited and saved; robot two's block
     byte-equal) + mutation R-locked-keep RED.
  6. connecting the folder after an edit replaced the edit with today's file and dropped undo → only an unchanged
     robot is re-read, under every history snapshot; page test (edit, connect, the edit and its undo still there)
     + mutation R-folder-reread RED.
  7. *(reproduced)* a flywheel toggle moved to a piston kept its speed (`toggle(claw["shoot"], …)`, which the build
     refuses) → a mechanism change drops the old target, and a form the new mechanism cannot take becomes one it can;
     `controlCode` writes a speed only on a flywheel; test + mutation R-mech-change RED.
  8. the map's FREE line could name the RE-ZERO button → `checkMap(state, rezero)`; test + mutation R-map-rezero RED.
  9. reuse: the tab re-typed `REGIONS` / `regionKey` → imported.
  10. efficiency: every drag of the "try it" dot rebuilt the whole graph, and each pane re-ran every check → the drag
      moves only the dot, its line and the readout; the checks run once per state; page test (the graph is the SAME
      element after a dot move) + mutation R-probe-remount RED.
  Note: the reader took `facts::lv(<one digit>)` only → any index; test (level 12).
- **Reviewer 2 (`code-review`, "high", forked; the C++ half of the agreement): it re-ran all 39 `rm_ct_*` + `rm01`
  (40/40 refused with the page's name, no second reason) and FE1 byte for byte, then found 10 — each re-checked
  and fixed:**
  1. the `rm_ct_*.cpp` freshness test compared bytes, but `test/compile_refusals/` has no LF `.gitattributes`, so
     every Windows row would read CRLF → the test normalises `\r\n` (no repository-wide attribute: landmine 19).
  2. *(reproduced)* a project file's numbers were judged unrounded (0.9996 passed, the export printed 1, the build
     refuses) → `drivingFromSaved()` and `fromSaved()` round as the export prints (W7); two tests.
  3. *(reproduced)* a preset on a POWER mechanism: the page said `no_such_preset_on_this_mechanism`, the build
     `a_power_mechanism_has_no_presets_run_it_with_r_run` → the page names the build's; a new CT case
     `rm_ct_preset_on_a_power_mechanism.cpp` (refused by the build with that name — ctest) and the coverage test now
     also demands a case for kinds.hpp's three target refusals.
  4. *(verified)* FE1's grid never lands on a dead zone's edge or a -0: `<=` → `<` survived → new FE1b golden
     `feel_edges.txt` (490 lines: each dead zone's edge and the doubles either side, ±0, the rails, dead zones and
     curves past their limits) made by `shulib_route_maker feel-edges`, compared by the suite and by node
     (`Object.is`); the `<` mutation now RED (FE1b); FE1's own 9 630-point file is unchanged (`cmp`).
  5. `pages.yml`'s path filter missed the driver files the page now bakes → `src/team/*_driver.hpp` added (A1's
     file, one line — logged as a delta).
  6. the team's driver text was baked raw, so a URL or a path in a student's comment would fail HS8/RG1 and stop the
     site's build → baked base64 (the team's text, not the builder's output), decoded in the tab.
  7. `hexDouble` wrote a subnormal with a hidden 1 and an infinity as an out-of-range literal → fixed (throws for
     an infinity); test round-trips subnormals through `fromHex`.
  8. the Controls status bar claimed "every refusal is the build's own", but the page also refuses unfinished
     controls, a negative ramp and a stale map line → the claim removed.
  9. two comments still counted DriverFile's checks as two → fixed (`driver_file.hpp` header, `driver_heading_test.cpp`).
  10. `ROBOTS()` re-read the fixture JSON per call → read once.
- **Reviewer 3 (`code-review`, "high", forked; it read the WHOLE brief and the live log, then probed the page in
  Playwright's Firefox): 10 findings — each re-checked and fixed, each with a page or node test that is RED without
  the fix (`mut/review3-run.log`, all 8 mutations RED, restored):**
  1. *(probed)* picking a job left the menu open (a second pick made a button do two jobs) → it closes (T-popover-close).
  2. *(probed)* a project with an unknown drive mode threw and drew no controller → the drawing falls back, the mode
     is the feel's error (new page test; T-mode-fallback).
  3. the page said "Ready to export" for a feel the build refuses (CT-HS finding 1) → **while `drive_program.cpp`
     still holds `driverLineMatchesFeel` (build.py bakes `feelDefaultOnly`), the page refuses any feel but the
     default with the static_assert's own words** — a build refusal like every other; it lifts itself the day the
     drive line patch lands (the page tests branch on the same fact). Guide 22 and the changelog say it as a plain
     fact. Tests: node (the rule, with the static_assert's sentence quoted from the source) and page
     (T-feel-default-only, T-default-only-rule).
  4. *(probed)* every re-draw dropped the keyboard focus to `<body>` → focus kept on the same button or field;
     Escape on the menu returns to its button; closing Export returns to the Export button (T-focus).
  5. *(probed)* the controller's targets were 17–21 px → an invisible 32 px+ target per button (the d-pad and face
     buttons spread so none overlaps), and the tab, segment, chip and small buttons raised to `--hit` (T-hit).
  6. G4: a two-button hold took "the first free button" when its partner was taken → the natural partner only when
     free, else the second button is the student's to choose ("unfinished" until then — never exported as a one-way
     hold); a changed "what it does" keeps the button the student chose (T-back-null).
  7. no hover help on the controller; `?` did nothing on this tab → each button carries its whole job as an SVG
     `<title>`; `?` opens help, which now speaks of the Controls tab.
  8. "Go there" missing for a mechanism with no button and for a feel error; the guide said "exactly what the build
     refuses" → both offered; the guide says the page also asks for what is unfinished.
  9. the graph's `role="img"` hid the "try it" slider; its readout announced on every pointer move; a smooth scroll
     ignored reduced motion → `role="group"`, the slider's `aria-valuetext`, no live region, reduced motion honoured.
  10. *(probed)* a word typed in the feel's number boxes gave no words (the ramp's was silently dropped), and the
      boxes stayed enabled on a locked robot → "Not a number: type digits, like 0.25." and disabled (T-nan-words).
- **Documents (pace rule 3; ≤ ~5 public):** guide 22 — the intro (three parts) and a new part "The driver's buttons
  and the driving feel" (what each button does, the kinds table, what the page refuses, the driving feel with the
  team lead's own example, the ramp slowing a stop too, the default-only fact, Export of the three regions) with five
  screenshots (`docs/assets/route-builder/controls*.png`, `feel.png`, Playwright's Firefox, light 1440); guide 18 —
  its driver-controls section links to it, and states D10's refusal; the changelog (API 2.9, 2026-11-16 entry);
  `docs/route-builder.md` and the guide README's row 22 (one sentence each). Small consequential edits: guide 21
  :230 links its "the map is yours" to the Controls tab; cookbook 06 :295 said DriverFile adds "the last two checks"
  — now three (prose only; its listings untouched). Internal: `test/README.md` (the Controls tab's C++ half, FE1b),
  `tools/route_maker/README.md` (the new modules and rules). NOT touched, deliberately: `mkdocs.yml` (no new page —
  guide 22 is in the nav; DOCS3-N1 owns the menu), guide 16/14 (nothing of B's applies), `docs/api/` (no `///`
  changed — `check-fresh` clean).
- **Prose review (rule 4), greps RUN and read before writing this:** "simulat", "honest", "obviously", "just ",
  "simply", "easy", "precondition", "radian" over B's six page modules and every added docs line — no hit but the
  rule's own "never simulator" line in `route_maker_controls_test.cpp`; `http` in the built page's six new modules: 0
  (the page test harness's `http://127.0.0.1` server is test code). Docs grepped for `driver_file.hpp`, `DriverFile`,
  `robot_one_driver_test`, "two checks": cookbook 06 :295 was stale (fixed); guide 7 :156, the roadmap and
  hardware-assumptions :2363 ("every line is MADE at Low" — still pinned by the re-pointed piston test) are true.
- **The C++ mutations re-run with every dependent object REBUILT (a lesson learned here):** `make shulib_tests/fast`
  (used to skip the doc gate a mutant driver file trips) does NOT recompile on a changed header — the first
  re-zero-on-the-d-pad run "SURVIVED" with 0 objects rebuilt. The driver now deletes every object whose depfile
  names `robot_one_driver.hpp` (2: `robot_one_driver_test`, `this_robot_test`) before linking, and prints the count:
  B-GM22-row, B-GM22-dpad, R-GM22-A-row, R-GM22-L1-row, R-GM22-rezero-on-dpad — **all RED, "recompiled: 2" each**
  (`mut/gm22-rebuilt-run.log`; the 09:26 GM22 verdicts are superseded by these). Afterwards both objects deleted
  again, the binary removed, the briefing regenerated, a full normal build.
- **Final gates (started ~10:45; this bullet written 10:58, as they finished — the "11:15" first written here was not
  a record; corrected in the review round):** build 0 warnings; `suite_run.py green` — **green, 3779 cases, 3 882 879 assertions**;
  node (model) **109 pass, 1 skip**; page tests in Playwright's Firefox — **B's 7/7**, A1's 10/16 (its 6 are the
  robot-one level select above, nothing else); `ctest -R compile_refusal` **336/336** (+`rm_ct_preset_on_a_power_mechanism`);
  `src_build_gate.py check` **PASS** (14 builds; robot one's 4 refuse exactly their 9 owed facts; the facts copy
  links all 6); `src_build_gate.py self-test` **5 FAILURES, cases 53 and 55, robot one's owed Stage A facts** —
  the SAME five as the coordinator's `logs/srcgate_selftest.rob1c2.log` (ROB1-C2 on main), not from this diff (B's
  only `src/` change is D10's `driver_file.hpp` hunk); the coordinator's `integrate1` log is OK, so its fix is in the
  integration; no plant left (`git diff -- Makefile src/` 0 "(self-test plant", no untracked file under `src/`
  `include/`); ARM header gate exit 0 over 258 headers (no `include/` change); the two CI greps clean; api_doc_tool
  check-fresh / check-examples (919 lines verbatim) / check-coverage / check-removability / self-test clean;
  `doc_staleness_audit` clean; `briefing_status.py check` current.
- **B's diff:** `scratch/rmp1-b/rmp1-b.diff` against the baseline tree `97d48bac…` (A1 + main + the four
  integration fixes in `a1-onto-main-merge-fixes.diff`); file list `rmp1-b.files`. Nothing committed, nothing pushed.

### RMP1-B review findings and their disposition (2026-11-16 ~12:15 – 13:30 EST, `date`)

Three reviewers (correctness, tests, docs) returned 31 findings (two pairs overlap: 8 = 26, and 4 = 19). **Found
first:** a stopped earlier pass at these fixes (files dated 11:28–11:29, after the reviews and after this log's
10:58 entry) had edited seven files (`controls.mjs`, `feel.mjs`, `driver_file.mjs`, `controls_tab.mjs`, `app.mjs`,
`controls_export.mjs`, `controls_cases.mjs`) with no test, no golden re-made and no log line. Each of those edits was
re-read against its finding before it was kept; two were changed (the stale-words check, below), and every one now
has a test that is RED without it. Every mutation verdict below is on a fresh run (node), a freshly built site
(page) or a rebuilt tree (the doc gate); the drivers restore on exit and on SIGTERM/SIGHUP/SIGINT and printed
`all restored: True` (`scratch/rmp1-b/fix/mut/`: `mutate2.py`, `node-run.log`, `page-run.log`, `doc-glob-mut.log`).

1. **The feel cannot be exported (blocker) — REAL, not fixable inside RMP1-B's files.** `drive_program.cpp`'s
   `static_assert(driverLineMatchesFeel(), …)` stops every robot build on any feel but the default; the page refuses
   the same with the build's own words. The fix is in `src/programs`, which the brief does not give B. **For the
   coordinator, before the ~Nov 18 freeze, with the team lead's OK:** `scratch/rmp1-b/drive-line-feel.patch`,
   RE-CUT (finding 30): repository-relative paths (`git apply --check` clean from the worktree root) and its doc
   edits bundled — guide 22's feel paragraph and the feel picture's caption, the changelog's last sentence, guide 14's
   limit. **Verified on the CT-HS tree** (`git apply` from a subdirectory of the main repository excludes every path,
   so it was applied with `patch -p1` there; the main tree checked untouched): with robot one's export carrying a
   dead zone of 0.1, a curve of 0.6 and a ramp of 4 /s, the build (doc gates included) and **the suite green, 3780
   cases, 3 882 953 assertions**; the site built from it says `feelDefaultOnly = False`, so the page lifts its rule by
   itself. After landing it: re-take `feel.png` (the picture still shows the refusal) and re-run the src gate (its
   14 builds passed with the C++ half on 2026-11-16 morning, `cths/srcgate.log`). *(Re-cut again at the resumption,
   below: the changelog hunk's context moved when A1 merged.)*
2. **False hand-edit alarm after Save and re-open — REAL (reproduced by the reviewer's probe), FIXED.** `keep()`,
   when it first takes a robot into the project, now saves the hashes the driver file was READ at into
   `project.exports` (never over an export's). Page test "saved, closed and opened again" (session 1: edit, Save;
   session 2: the same folder, Export → written, no Keep/Overwrite). Mutation F2-no-read-base RED.
3. **Words contradicting the code — REAL, FIXED (the earlier pass's rule kept, its check changed).** (a) A control's
   new JOB (form, mechanism, target, volts, time limit, one or two buttons) now gets the page's comment AND map rows;
   a button move keeps the team's words, re-keyed. (b) `checkWords` (new): the team's own words that name a button
   now belonging to ANOTHER mechanism (or none), or one of the control's own buttons whose role changed, are an error
   (`wordsStale`, fix **Use the page's words**); a note naming such a button, or a mechanism whose controls changed,
   is a warning listed in Export. Compared against the controls the FILE's words were written for (the driver file
   as read — baked, or today's from the folder; kept per robot, so a project re-opened still knows them) — the
   earlier pass compared against the state loaded, which a re-opened project had already changed. Two false
   positives found by CT-HS and fixed: R1/R2 swapped within the cascade (still the cascade's: "R1 / R2 wait" stays
   true) and the page's own default comment naming its own buttons. Node tests: the flipper's toggle moved to the
   claw, the driver-start home made a button home, robot two's 12 V hold made 6 V (no "full power" left — finding
   20), the cascade on L1/L2 with the toggles on R1/R2 (the `(start)` row flagged, fixed by one click), the 12 V
   note flagged, the swap not flagged, a team comment "R1 lifts" flagged after a swap, "A held" not read as A.
   Mutations F3-job-words, -words-off, -notes-off, -role-not-mech, -judge-default-words: all RED.
4. **Guide 21's listing stops the suite's build on any export — REAL (reproduced), FIXED for guide 21.** Guide 21
   now quotes robot one's map AS THE BUILDER GAVE IT from a new compiled CT21 golden,
   `test/goldens/route_maker/controls_one_builders_map.hpp` (made by `goldens.mjs` from the marked fixture, so CT22
   holds it to the builder's file); `api_doc_tool.py`'s `EXAMPLE_SOURCE_GLOBS` gains
   `test/goldens/route_maker/controls_*.hpp`, with its reasoning, and its self-test a pass case and a drift case. The
   whole-run rule still holds the listing to the live file while the two agree. **CT-HS on the DELIVERED
   configuration** (robot one: the claw A → Up with the FREE row removed, the cascade's hold swapped, Brake; the
   default feel; no docs re-quoted, no patch — `fix/make_cths2.mjs`): build green (6 whole-run listings — guide 21's
   now matched by the golden), **suite green, 3780 cases, 3 882 929 assertions**; the src gate on it: all 14 builds as
   required (robot one's four refuse exactly their 9 owed facts), only the facts-copy proof unable to run in a
   non-git copy (as before). Mutation: the glob removed on that tree → `NOT VERBATIM … guide/21` (RED); restored →
   clean. **For the coordinator:** cookbook 06 quotes robot two's `kControls`, `kFeel` and `kLoop` the same way —
   ROB1-BT (in flight, it rewrites that file) should quote a robot-two golden made the same way, or every robot-two
   export will need a re-quote. Documents: guide README's scope sentence, `guide-maintenance.md`, `test/README.md`.
5. **A1's six page tests time out on robot one's empty Level list — REAL (not caused by B), FIXED as the FIFTH
   A1-onto-main integration fix, in the BASELINE (staged), not in B's diff:** `readyRoute` sets the claw open where it
   chose a level (as the smoke route already does); the "route built by clicking" test keeps its level part while
   robot one has a level (`ONE_LEVELS`, read from the built site) and otherwise puts the claw in the move's place;
   the "[R1] cascade move with no home" test chooses a level only when there is one. Firefox 16/16, Chromium 14 + 2
   skipped (A1's own Firefox-only skips). Appended to `scratch/rmp1-b/a1-onto-main-merge-fixes.diff` (the hunk alone:
   `fix/a1-fix5-page-test.diff`); **the baseline tree is now `9ea9b10e9ae9a79260f4d8b584b3fccc7db12df9`**
   (`BASELINE_TREE`). *(Superseded at the resumption, below: A1 merged as `e8fa3de` WITHOUT this fix, so it is now
   part of B's diff.)*
6. **Overlapping or nested markers accepted — REAL, FIXED** (the earlier pass's check kept): `findRegions` refuses
   parts that overlap or nest (Copy only). Node test: a nested `control words` END, and an END marker twice.
   Mutations F6-overlap and F6-end-twice (`e.length >= 1`, the reviewer's survivor) RED.
7. **Undo acts on a robot not shown — REAL, FIXED** (the earlier pass's per-robot history kept): one history per
   robot; Undo/Redo and their buttons follow the robot shown. Page test (edit one, show two: Undo disabled, Ctrl+Z
   changes nothing; back on one, Ctrl+Z undoes). Mutation F7-undo-every-robot RED.
8. = 26. **The saved shape is a superset of G3a's — REAL, LOGGED as a D14 delta (no code change):** the `controls`
   block keeps G3a's per-kind lists and, beyond them, `at` (the place in the statement: per-kind lists lose the
   order the build reads) and `banner` (the control's map rows) per entry, and `table`, `head`, `notes`, `layout`,
   `banner` at its top; the `driving` block adds `words` {comment, rezero, release}. Why: CT22's byte-identical round
   trip of the team's own words and layout through Save/Open. **For G3a's re-plan (its inheritance list):** accept
   these keys or map them onto its per-entry `words` and its `notes`; `fromSaved` refuses any key it does not know,
   so nothing is dropped silently either way.
9. = 23. **"Use the library's defaults" / a release change rewrote the feel's comment and dropped HA-112 — REAL,
   FIXED** (the earlier pass's rule kept): the comment is re-made only when a number or the mode it describes changed,
   and the page's own comment says "numbers no driver has felt yet (HA-112)" while any axis sits at the library's
   default. Node test (defaults on a default feel: nothing changes; Brake: comment and HA-112 kept; a forward curve:
   the page's comment, still HA-112 for turn; every axis set: no HA-112). Mutations F9-HA112-gone,
   F9-comment-always-dropped RED.
10. **`pages.yml` missed `drive_program.cpp` — REAL, FIXED** (one path line; A1's file).
11. **CT21 only on fixture robots — REAL, FIXED:** three goldens on robot one's REAL declarations, compiled through
    DriverFile in `route_maker_controls_test.cpp` (new case): `one_every_form` (a two-button hold at exactly 12 V, a
    home on a button AND at driver start, each piston's position by press, a toggle, a two-button hold on a side
    toggle, autons only), `one_holds_only` (the stall-homed cascade with unhomed volts, held, no home) and
    `one_builders_map` (finding 4). Their counts checked.
12. **The menu picked the second button and the piston's position — REAL, FIXED** (the earlier pass's menu kept, the
    form select's partner guess removed too): each menu item says in full what it picks ("Move while held: Up up,
    Down down"; "Go to "open" on one press", one per position; a flywheel's speeds by name); a pair whose partner is
    taken reads "…, a second button you pick" and leaves it unset ("unfinished"). Page test G4. Mutation
    F12-menu-guesses-position RED. The two page tests that clicked the old item re-pointed.
13. **Splice order untested — REAL, FIXED:** node test writing a map that SHRINKS (the FREE row out, two rows in) in
    LF and CRLF — each region equal to its export, every line outside unchanged, markers included; the page's export
    test now changes the map's length too and compares each written part with the drawer's. Mutation
    F13-splice-top-down RED in node and in the page.
14. **The download way untested — REAL, FIXED:** Firefox page test — pick the marked file, the download has the
    change only between the markers; a hand-edited file asks, and Keep downloads nothing; the tab's words never say
    "connect a folder" there. Mutations F14-D1 (the picked text downloaded), F14-D2 (overwrite forced) and
    F23-connect-everywhere RED. (Firefox is a CI row on Linux and Windows; Chromium skips it, as A1's own.)
15. **The CT agreement covered names, not conditions — REAL, FIXED:** nine cases added (`rm_ct_*` twins made by
    `goldens.mjs --write`, **all refused by the build with the page's name: `ctest -R compile_refusal_rm` 50/50**): a
    two-button hold on the flywheel, homeAtDriverStart on a motor, a toggle on an arm, two presets to one piston
    position, the re-zero on a hold's BACK button, 12.001 V, exactly 24 controls (refused only for two-of-a-kind), a
    turn curve and a strafe dead zone out of range. Pass side: `one_every_form` (12 V) and `one_holds_only` (N10). The
    agreement now compares the SET of names (24 hood entries repeat one name). Mutations N4, N5, N6, N7, N8, N10, N12,
    N15 all RED. The (name × condition) enumeration the reviewer offered as "better" is not built: each condition of
    controls() now has a case, by reading driver_inputs.hpp's branches.
16. **Feel checks on forward only — REAL, FIXED:** FE2's rows run on all three axes; rows below the default dead zone
    (0, 0.04, 0.049) are not the default; a ramp of Infinity is refused. Mutations N1, N2, N3 RED.
17. **The log's "the WORDS are CT23's" — REAL, CORRECTED:** CT23 reads the map's words back from the same file, so no
    test judges the team's words; the log line, CT23's title and GM22's comments now say so. Not added: a check that
    a row names its mechanism (robot one's own rows would fail it — "the RIGHT side toggle" is not "right toggle").
    What guards words now: finding 3's rules on every change the page makes.
18. **route_preview self-test not logged; no fresh verifier; a time window written early — REAL.**
    `python3 tools/route_preview.py self-test`: **ok (0 failure(s))** (12:44, `fix/route_preview_selftest.log`). The
    fresh verifier is the step after this fix round — not run by the builder. The "Final gates (10:45–11:15)" line
    corrected (written at 10:58).
19. = 4. Guide 22's "the robot's tests run the new map" removed ("Then build as usual: the build is the final judge
    of every line" — true for a robot-one export now, CT-HS above).
20. **"The map always says what the code does" — REAL, FIXED in code (3) and reworded:** guide 22 now says the rows
    follow the code, what the page stops on, and that words it cannot judge are the team's to keep true.
21. **The export picture showed the error drawer — REAL, FIXED:** the page test now takes it after Ctrl+Z (the
    file's own map): the three parts, each with Copy. All 20 Controls screenshots re-taken (Firefox; light and dark,
    1440 and 1024; `scratch/rmp1-b/shots/`, opened: export light-1440 and dark-1024, refused, menu); the five docs
    images replaced. The refused picture's caption now says "B turns red" (it is one red label).
22. **Docs said the tab opens with today's map; Firefox was told to connect — REAL, FIXED:** guide 22 says it opens
    with the driver file as published (today's with the folder connected) and the project's once changed; its
    project-file section says the project holds the controls and feel; `route-builder.md`'s "What it knows" names the
    driver files and its browser table the driver-file export; in a browser without folder writing the tab says
    "Export checks the file you pick". The tab also re-reads today's files when a remembered folder reconnects and
    after a project opens (two call sites, `app.mjs` and `reload()`).
24. **"The same numbers as the robot's code" — REAL, FIXED:** "the library's C++, checked on the host" (guide 22,
    changelog).
25. **"Refuses exactly what the build refuses" — REAL, FIXED:** the changelog and `tools/route_maker/README.md` say
    what the build refuses, plus what is unfinished or would leave the map wrong (no twin for those).
27. **A1 files touched, not all logged — REAL, LOGGED (delta):** `app.mjs` (the Controls tab, its undo/redo routing,
    PREVIEW hidden on it, the status bar, `window.routeBuilder.controls`, the driver files re-read after a folder or
    project), `build.py` (the driver files baked, base64; `feelDefaultOnly`), `files.mjs` (`readTeam`, `writeTeam`,
    `pickText`, `offerDownload`), `style.css` (the controller, the graph, the menu), `help.mjs` (the Controls help),
    `goldens.mjs` (the CT twins, the CT21 goldens, FE2's rows), `test/route_maker_describe.hpp` (an
    `unhomedVolts` field on every arm and lift — the baked `robots.json` gains it, so the three fixture descriptions
    `robots/{x,y,z}.json` were re-made), `test/route_maker_main.cpp` (`feel`, `feel-edges`), `pages.yml` (two paths),
    guide 22 and `route-builder.md` (A1's pages); outside A1: `tools/api_doc_tool.py` (finding 4). The brief's
    golden layout (`controls_*.json` → `*.hpp` compiled by `route_maker_golden_test.cpp`) became
    `tools/route_maker/controls_cases.mjs` → `rm_ct_*.cpp` and `controls_*.hpp`, compiled by a new
    `test/route_maker_controls_test.cpp`: one data file for the page's node test and the C++ twins.
28. **§6.4's third T11 rule (the map equal to the project file's) — DEFERRED, with the reason:** a teammate's hand
    edit of the driver file is legal (G4, D9), so a suite test demanding equality would turn red on every legal
    hand edit — and on every project saved before its export; the disagreement it guards is caught where it matters,
    at the next export (the hashes: Keep / Overwrite). The coordinator may rule otherwise. Found while weighing it:
    once a project holds a robot's controls, the tab has no "read the driver file again" — a teammate's hand edit
    can be kept or overwritten, not taken in (remove the robot's block from `route_maker.json` to re-read). For the
    coordinator.
29. **The log's "FIVE of its cases" — REAL, CORRECTED:** six of seven red; the compact row's SMALL-budget case stayed
    green.
30. **The patch failed `git apply` — REAL, FIXED:** see 1.
31. **Guide 14; cookbook 06 — REAL, FIXED / FLAGGED:** guide 14's route-builder paragraph states the Controls tab's
    limits (the default feel only in this version, marker lines needed, no slow mode while a button is held — the
    patch drops the first). The cookbook 06 hunk (:295, "the last two checks" → three) stays; **for the coordinator:
    fold it into ROB1-BT's re-quote of that chapter.**

**Prose review (rule 4), greps RUN and read before writing this:** `EXAMPLE_SOURCE_GLOBS` / `check-examples` /
"compiled example test" over `docs/`, `README.md`, `test/README.md`, `tools/*.md` → guide README :83, 
`guide-maintenance.md` :31-37, `test/README.md` :51 stated the old source set (all three updated);
`docs-publishing.md` :135 and `build-order.md` :632 only name the check (true). `pages.yml` → `RELEASING.md` :145
(true). The removed or changed claims ("Go to one position", "the map the driver file has now", "always says what
the code does", "the robot's code computes", "exactly where the build refuses", "refuses exactly") → no hit left
outside `docs/internal/chunks/`. `driver_one_marked`, `controls_one_`, `wordsStale`, `checkWords` → only the new
lines. The same words-to-avoid grep as before over every line added this round: no hit.

#### Resumed 2026-11-16 22:50 EST (`date`): the fix round continued after the team lead's pause

**Found.** All 31 findings already had a disposition above. The stopped run had reached its final gates. Suite, ctest
(345/345), node, page and the doc gates were green at 12:47–12:54. Its src gate self-test ended **exit 1** at 13:45, on
cases 53 and 55: robot one's Bench Tests links since ROB1-C2, and the old baseline lacked `531c3fd`, which updated those
two cases. Its check never ran. Nothing was left applied: no `(self-test plant` in `git diff -- Makefile src/`,
nothing untracked under `src/` or `include/`, both mutation backup folders (`mut/backups`, `fix/mut/backups`) empty.
The only `src/` change is B's own (`src/common/driver_file.hpp`, the feel `static_assert`). The test binary (12:47)
was relinked from source: removed, briefing regenerated, full build, briefing regenerated again.

**The tree moved onto A1 as merged.** `shulib-v2` now carries A1 (`e8fa3de`, 12:14, on `531c3fd`), so the old
baseline (`9ea9b10`, this run's own A1-onto-main) was replaced. The index is now `e8fa3de`'s tree (`2b3d949`,
`BASELINE_TREE`); the working tree is `e8fa3de` + B's diff, built three-way from a snapshot of B (`fix/rebase/`:
`B_TREE`, `b-on-9ea9b10.patch`, `b-files.tgz`, the old `BASELINE_TREE`). Five files changed on both sides: the
changelog (B's entry kept at the top of API 2.9, above A1's), this log (the coordinator's merge section kept before
`## RMP1-B`), guide 14 and `test/README.md` (clean, read: no sentence contradicts), the briefing (regenerated). The
tree was checked equal to the planned one (`c981818`) before any gate ran.

- **Finding 5 is now in B's diff.** `e8fa3de`'s `page.test.mjs` is A1's as pushed. On this tree, in Firefox, it fails
  the same six tests (`pass 10, fail 6`, `fix/resume/page-a1-as-merged-ff.log`), so shulib-v2's route-builder
  browser rows stay red until this lands. The fix is unchanged; it is now an A1-file edit in B's diff, and item 27's
  list gains `tools/route_maker/test/page/page.test.mjs`. `a1-onto-main-merge-fixes.diff` is retired: fixes 1–4 are
  the coordinator's own in `e8fa3de`.
- **Findings 1 and 30: the patch re-cut again.** Its changelog hunk's context moved (A1's entry now follows B's).
  Same edits; `git apply --check` is clean on this tree. The previous cut is kept as
  `fix/resume/patch-recut/drive-line-feel.patch.on-9ea9b10`.

**Re-verified on the final tree** (logs in `fix/resume/`):
- Full build 22:54–22:56, doc gates in it. Suite **green, 3782 cases, 3 883 278 assertions** (`e8fa3de` alone:
  3778). ctest **346/346**, 23:02–23:07: all 345 compile refusals, the 50 `rm*` twins among them.
- ARM header gate (258 headers): exit 0. CI greps: PROS-free, sim-free, LF.
- api_doc_tool self-test, check-fresh, check-examples (919 lines, 7 whole-run listings), check-coverage (6439
  entities) and check-removability; doc staleness clean; briefing check current.
- `route_preview.py self-test`: ok (0 failures).
- Node: **114 pass, 1 skip** (GL0's real file, which ROB1-BT adds). Page: Firefox **27/27**; Chromium **24 + 3
  skipped** (the Firefox-only branches).
- **CT-HS on the delivered configuration, on this tree.** The scratch tree was re-synced from the worktree, then
  `make_cths2.mjs` wrote robot one's export: the claw A → Up with the FREE row removed, the cascade's hold swapped,
  Brake, the default feel; no docs re-quoted, no patch. Build green (6 whole-run listings); suite **green, 3782
  cases, 3 883 276 assertions**. The doc-glob mutation on that tree is RED (`NOT VERBATIM … guide/21`); restored, it
  is clean.
- **Every fix-round mutation re-run on this tree** (`mutate2.py`, 23:11–23:13): **28 of 28 RED**, `all restored:
  True`. F2 and F14-D2 took 33 s each, the rest under 4 s. The site was rebuilt from the restored sources afterwards.
- **The src gate** (`fix/resume/run_srcgate3.sh`, 23:12 to 00:22 on 2026-11-17): self-test **OK, 377 detector
  cases** (cases 53 and 55 pass on this tree), run under `timeout -s INT`; check **PASS, all 14 builds**. Robot one's
  four refuse naming exactly their owed facts, and its 6 builds link in the gate's typed copy. Before and after the
  check: no `(self-test plant` in `Makefile`/`src/`, nothing untracked under `src/` or `include/`.

**Prose review for this step** (greps run and read): the removed-claims grep was re-run. "Go to one position" hits
only `controls.mjs:40`, the form's name in the right pane's "What it does" list, where the position is then picked
from "Choose one" (the menu itself names each position). No other hit outside `docs/internal/chunks/`.

### Verification-round findings and their disposition (2026-11-17 from 00:45 EST, `date`)

A fresh verifier returned six findings (two major, four minor). All six are REAL. Each was reproduced first, with
the verifier's own probes (`verify/probes/`) or an instrumented one, then fixed at the root. Every fix has a test
that is RED without it. Every gate was re-run on the final tree. Scratch for this round: `scratch/rmp1-b/vr/`.

**First, the base moved (finding 6).** `shulib-v2` gained DOCS3-N1 (`26ee38e`), `1bcde4e` and `b3146d4` after A1
merged, so B's diff no longer applied. B was rebased onto `b3146d4`, the same way as at the last resumption:
- B snapshotted as a tree (`vr/B_TREE_before_vr`; files in `vr/b-files-before-vr.tgz`; patch in
  `vr/b-on-e8fa3de.patch`).
- Eleven files changed on both sides. Each was merged three-way with `git merge-file` (`vr/m3/`).
- The index moved with `git read-tree -m -u 2b3d949 4076a77`. The index is now `b3146d4`'s tree, `4076a77`
  (`BASELINE_TREE`; the old one is in `vr/OLD_BASELINE_TREE`).
- Checked: no path differs from B's snapshot except paths `shulib-v2` changed.

Three files conflicted:
- **`docs/changelog.md`:** B's entry first, then DOCS3-N1's, then A1's (newest first).
- **`docs/guide/README.md`:** taken as `shulib-v2` has it. DOCS3-N1 made row 22 a short "Make your first route |
  Route builder" row, so B's longer row-22 sentence no longer has a place. B's scope sentence (finding 4 of the
  review) moved with DOCS3-N1 to `docs/how-the-docs-stay-true.md`, and is re-applied there. That page now says chapter
  21 quotes robot one's map from `controls_one_builders_map.hpp`, not from the driver file.
- **`docs/internal/PROJECT-BRIEFING.md`:** regenerated.

`tools/api_doc_tool.py` merged cleanly: B's glob and self-test cases sit on DOCS3-N1's version, and its comment
points at `how-the-docs-stay-true.md`. Guide 14 and 18, guide 21, cookbook 06, `guide-maintenance.md`,
`test/README.md` and this log merged cleanly; their merged hunks were read.

- **New with DOCS3-N1, one link:** the docs front page (`docs/README.md`) "Drive the robot" row links "Set the
  buttons and the stick feel by clicking" to guide 22's Controls part. That part's anchor is the one guides 18 and 21
  already link to. `mkdocs` is not installed here; `pages.yml`'s strict build checks the anchor.
- **The drive-line patch, re-cut again** (findings 1/30 of the review). Its changelog hunk's trailing context is now
  DOCS3-N1's heading. `patch --fuzz=2` was applied to copies (`vr/patch/`), and the timestamps were stripped. The
  edits are identical (the `+`/`-` lines compared), and `git apply --check` is clean on this tree. The previous cut
  is `vr/patch/drive-line-feel.patch.on-e8fa3de`. Its C++ half is unchanged: `src/programs/drive_program.cpp` is
  byte-equal at `e8fa3de` and `b3146d4`, so the CT-HS verification of 2026-11-16 holds.

1. **Words the team types after a remap were refused; the one fix dropped `[delete-able]` (major).** REAL, FIXED.
   **Reproduced:** `start_row.mjs` and `words.mjs` gave `wordsStale` on "L1 / L2 wait" and on "L1 raises it, L2 lowers
   it". **Root cause:** every word that was not the page's was judged against the driver file as read. So a line the
   team wrote AFTER the remap was judged against jobs it was never written for.
   **The fix (`controls.mjs`):** each LINE is judged against the jobs it was written for:
   - a line the team types here: the jobs when it was typed (`editControl` records them in `wordsFor`, for new lines
     only; a line left untouched in an edited row keeps its baseline);
   - the file's own line: the driver file as read;
   - any other (a project's, opened again): the project as opened (`checkWords`' new `opened`; the tab passes
     `loaded`).
   The page's own lines are never judged. The error now says which part is stale ("its map row "(start)" names
   R1, R2"). **Use the page's words** replaces only the stale comment or rows: a stale row is replaced by the page's
   row for the same key, or the whole map part when no key matches. So the (start) row is fixed and
   "[delete-able] home it at the first driver start" is kept.
   **Tests (node):**
   - the truthful (start) row and true comment and rows on the moved hold: no error;
   - only one of the row's two "R1 / R2" lines fixed: the other still flagged;
   - the team's TRUE words going stale when the jobs change AFTER they were typed: flagged;
   - the one-click fix keeping the comment;
   - `wordsFor` never saved;
   - a project opened again judged against itself, not the file;
   - the old "R1 lifts" case now judged against the FILE as `before`.
   **Tests (page, Firefox):** the six-select remap in the browser, its (start) error, the one-click fix (comment
   kept), Ctrl+Z, then the row typed true: no error. A project holding a remapped map and a typed row opens with no
   error. **Mutations RED:** V1, V2, V3, V4, V7, V7b (node), V11, V12 (page).
   **Known limit, for G3a's re-plan:** `wordsFor` is not saved (the D14 shape keeps no per-line jobs). So a line
   typed in one session and made stale before Save is judged, after reopening, against the project as opened, where
   it is already stale. It is no longer flagged then. Lines from the driver file are always judged against the file.
2. **Connecting the folder on a page with no route discarded every unsaved Controls change (major).** REAL, FIXED.
   **Reproduced by its two halves:** `syncWithFolder` treats an empty route as an empty page, and `reload()` rebuilt
   every robot's history.
   **The fix:**
   - `reload(keepChanged)` (`controls_tab.mjs`) keeps a robot changed here one Ctrl+Z away (a commit, as A1 does for
     the route). With `keepChanged` and no controls of its own in the opened project, it keeps the robot as it is and
     writes it into the project.
   - `syncWithFolder`'s empty-route path passes `keepChanged`.
   - Its question says "route and Controls" (titled "a different project") when the page holds Controls, and that
     Ctrl+Z brings this page's back.
   - `readFolder` swaps an unchanged robot's present for today's file and keeps its past (`historyOf` dropped it).
   - Found on the way: A1's `toast()` shows one at a time, so the Controls note is folded into the open's own toast
     ("Opened … from the folder. Your Controls changes for robot one are kept …"). Open does the same.
   - A robot the opened project holds read only cannot get its changes back. The toast says so.
   - Found on the screenshots of this state (`~/shulib-render-tmp/rmp1-b/shots-vr/`, light and dark): with the
     folder connected and the robot changed here, the right pane still said "Connect your shulib folder to read
     today's". It now says the robot was changed here, so today's file is not read in, and that Export asks before
     replacing anything that differs. This was true before this round too (the review's finding 6 skipped a changed
     robot), and it is now asserted in the page test (mutation V17).
   **Page test (Firefox, 3 scenarios):**
   - the verifier's flow: a fresh page, Brake, connect a folder whose project has a start and no Controls. Brake
     kept, the route open, Save writes both.
   - a folder project with robot one's own Controls: they open, and Ctrl+Z brings Hold back;
   - a page WITH a route: asked with the new words; "Open the folder's route" gives today's file; Ctrl+Z gives Hold.
   **Mutations RED:** V8, V9, V10, V13, V14, V15.
3. **Guide 22's "move it to another button and yours are kept" was false for Left / Right (minor).** REAL, FIXED in
   code, so the sentence is now true. One combined row is keyed "Left/Right" when "Left / Right" is too long for the
   key column. That is `pairKey`, used by the writer (`rekey`) AND the reader (`parseControlWords`), which accepted
   only "A / B" before. A page change that still has to replace the team's rows toasts that it did (the comment-only
   toast missed it).
   - **Found while fixing:** the page's `keyButtons` split keys at "and" too, and GM22's C++ does not. So a
     hand-written "Up and Down" row was two buttons to the page and none to the suite. It now splits at "/" and ","
     only, as the C++ does. A node test pins the C++ source's `find_first_of("/,"`. GM22's comment names
     "Left/Right".
   - **Node tests:** robot two's combined row moved to Left/Right, Right/Down, Up / Down and Down / Left keeps the
     team's lines and reads back onto its control through a real splice; plus the key-split agreement.
   - **Mutations RED:** V5, V6.
4. **The README still claimed a twin for every build refusal; finding 27's A1 list missed the README (minor).**
   REAL, FIXED. `tools/route_maker/README.md` names the one build refusal with no compile-refusal twin: any feel but
   the default, while "shulib Drive" names the default feel at start (`drive_program.cpp`'s `static_assert`). It is
   pinned by its sentence read from that source in `feel.test.mjs`. **Finding 27's list of A1 files touched gains:**
   - `tools/route_maker/README.md`;
   - this round's `app.mjs` (`fix()`'s focus, and `adoptProject` / `adoptFolderText` / `syncWithFolder` /
     `openProj`);
   - `tools/route_maker/test/page/page.test.mjs` (the focus assertion).
   Outside A1 and B, DOCS3-N1's `docs/how-the-docs-stay-true.md` (the scope sentence) and `docs/README.md` (one link).
5. **A1's re-pointed page test failed now and then in Chromium at the redo (minor).** REAL. **ROOT CAUSE FOUND, in
   A1's `app.mjs`:** `fix()` ("Home it first") focused the new home's time-limit box in a `setTimeout(…, 0)`.
   - **What went wrong:** in Chromium that timer sometimes ran only after the test's later keys (Playwright's input
     events went first). It moved the focus into the box between the Ctrl+Z and the Ctrl+Shift+Z. The page ignores
     shortcuts while a box is being typed in, so the redo was lost.
   - **Probe:** `vr/flake/probe.mjs`, the test's steps looped with a capture of every keydown, focus and `focus()`
     call with its stack. With A1's code it failed 2 of 40 and 3 of 60. Every failure shows the box taking the focus
     right before the redo's `z`; in the 60-run, which captured the stack, that is `focus()` from `app.mjs:259`.
   - **The fix:** the box is focused at once, since `edit()` has already drawn it. Then 0 of 80, and 0 of 100 on the
     final site. The verifier's "one more page round trip" hid it by giving the timer time.
   - **Test:** the test now clicks "Home it first" and reads the focused element in the same task. Mutation V16
     (the deferred focus restored) is RED every time.
   - **Chromium:** the whole page suite ran 5 times, 27 pass and 3 skipped (the Firefox-only tests) each.
6. **B's diff did not apply onto `shulib-v2` after DOCS3-N1 (minor).** REAL, DONE HERE rather than left to the merge:
   see "the base moved" above. The coordinator now applies B onto `b3146d4`. The verifier's merge steps are all done:
   - the scope edit is carried into `how-the-docs-stay-true.md`;
   - `api_doc_tool.py` is merged;
   - B's changelog entry is on top;
   - the briefing is regenerated;
   - the patch is re-cut.

**Mutations this round** (`vr/mut/mutate3.py`, the review round's driver with new cases: each mutant is one edit,
run fresh, and restored in a `finally` and on SIGTERM/SIGHUP/SIGINT; the page mutants each on a freshly built site,
Firefox): **17 of 17 RED, `all restored: True`** (`vr/mut/mut-run.log`). No C++ logic changed this round: only
GM22's comment in `test/robot_one_driver_test.cpp`. So no C++ mutation applies; that object was rebuilt by the clean
build.

**Gates on the final tree** (logs in `vr/gates/`):
- **Briefing and build:** the binary was removed, the briefing regenerated, then `cmake --build --clean-first`
  (317 objects, 0 warnings, the doc gates in it: 919 lines verbatim, 7 whole-run listings). The briefing was
  regenerated again after.
- **Suite and ctest:** suite **green, 3782 cases, 3 883 278 assertions** under the 4 GiB cap. **ctest 346/346**
  (01:09–01:13).
- **Node:** **117 pass, 1 skip** (GL0's real file). The site was rebuilt from the restored sources.
- **Page:** Firefox **30/30**. Chromium (cgroup cap) **27 + 3 skipped, five runs**.
- **Doc tools:** `api_doc_tool` self-test, check-fresh, check-examples, check-coverage (6439 entities) and
  check-removability clean. `doc_staleness_audit` clean. `briefing_status.py check` current. `doc_readability`
  check-nav (260 pages), check-tone (46) and self-test (159 cases) OK. `route_preview.py self-test` ok.
- **ARM and CI greps:** ARM header gate exit 0 (258 headers). CI greps: PROS-free, sim-free, LF.
- **The src gate:** below.

**Prose review (rule 4), greps run and read before writing this:**
- "page's words", "Use the page", "different route", "another button", "Ctrl+Z brings", "one Ctrl+Z", "Home it
  first", "re-keyed", "keyed" over `docs/`, `README.md`, `tools/route_maker/README.md` and `test/README.md` (not
  `docs/internal/chunks/`, not `docs/api/`). Hits: guide 22 :83 ("Home it first", still true; the box takes the
  keyboard as before, now reliably), :169-170 (true now, finding 3) and :191 (rewritten). The rest were unrelated
  (cookbook 06 :163, build-order, hardware-assumptions).
- `docs/route-builder.md` :36-40 ("with your shulib folder connected … reads today's driver file"): still true for
  a robot not changed here.
- `tools/route_maker/README.md`: no sentence on undo, connect or words besides :58-63 (rewritten).
- The words-to-avoid grep ("simulat", "honest", "obviously", "just ", "simply", "easy", "precondition", "radian",
  "http"). Its first pass found one of mine: guide 22's "replaces just that comment or row", now "only". The other
  hits were DOCS3-N1's own front-page line and this rule's line. That pass diffed a tree against the working tree,
  which does not see B's untracked files (`controls.mjs`, `controls_tab.mjs`, the tests). So it was re-run against
  a tree of everything (`vr/added-lines.txt`, 517 added lines): only this rule's own lines hit.
  `doc_readability check-tone` is clean.

#### Resumed 2026-11-17 03:06 EST (`date`) — the run above was stopped during the src gate's `check`

**Found on resuming:**
- The index is still `b3146d4`'s tree (`4076a77`), and `shulib-v2` is still `b3146d4`. B is the working tree.
- No src-gate plant: `git diff` (staged and not) over `Makefile src/` has no "(self-test plant"; nothing untracked
  under `src/` or `include/`.
- No mutant left: every backup folder under `scratch/rmp1-b/` is empty, and each of `mutate3.py`'s 18 edits finds
  its original text once and its mutant text nowhere. No Playwright browser was left running.
- Where it stopped: the self-test passed at 02:01 (377 cases). The `check` started then and its log is empty.
- **One correction to the round above:** "17 of 17 RED" was V1–V16 plus V7b. V17, named under finding 2, had been
  added to the driver after that run and never run. It is run below, then re-cut as V17b after the fix below.

**Re-verified:** the verifier's probes re-run on this tree (`verify/probes/`, `vr/resume/start_row_stale.mjs`):
- the true "L1 / L2 wait" (start) row and the true words on the moved hold: no error;
- **Use the page's words**, called as the tab calls it, fixes only the (start) row and keeps
  "[delete-able] home it at the first driver start";
- robot two's combined row moved to Left / Right: kept, keyed "Left/Right".
The Chromium redo probe (`vr/flake/probe.mjs`): 0 of 100.

**New, found re-verifying finding 2, FIXED:**
1. **Guide 22 said "Connecting the folder or opening a project never loses a change made here." That was false.** A
   project whose controls for that robot this page cannot read (a newer builder's) opens that robot read only, and
   `reload` drops the change. The toast said so ("… are read only here, so your changes there are gone"), but no
   test covered it.
   - **The guide:** it now names that one exception.
   - **Its refusal table:** the "A newer project file" row now says a robot's Controls this page cannot read open
     read only (only a step, or a field of the route, refuses the file).
   - **Page test:** "Controls changed here, then a folder project whose controls for that robot this page cannot
     read …". Mutation **V18** (the change said to be one Ctrl+Z away) is RED.
2. **The right pane said "You changed it here" when the robot was not changed.** Found on that state's screenshots.
   It happened for a robot whose change was gone, and for any unchanged robot whose folder has no driver file, or one
   it cannot read. Finding 2's note had assumed that a robot still read from the page, with the folder connected, was
   always a changed robot.
   - **The fix (`controls_tab.mjs`):** `readFolder` records why today's file was not read in, and the pane says that:
     - the folder has no such file;
     - the file was not read in, with the reason;
     - it was changed here.
   - **Test:** the same page test asserts both robots' notes.
   - **Mutations:** V17b (the "changed here" note dropped), V19 (the old always-"changed here" note) and V20 (an
     unreadable file called missing) are RED.
   - **Screenshots:** `~/shulib-render-tmp/rmp1-b/shots-resume/` (light and dark), looked at.

**Mutations this resumption** (`vr/resume/mut/mutate4.py`, `mutate5.py`: `mutate3.py`'s driver, each page mutant on
a freshly built site, Firefox, restored in a `finally` and on SIGTERM/SIGHUP/SIGINT; each mutant was checked to
parse with `node --check`, so each RED is the test's):
- V17 (on the code before the fix) RED;
- then V18, V17b, V19 and V20, all RED;
- `all restored: True` both times.
So this round now has 22 mutations: V1–V17, V7b, V17b and V18–V20, all RED (V17b replaces V17 on the fixed code).

**The base moved again (finding 6, re-verified: its "done here" no longer held).** `shulib-v2` is now `b4392d0`.
Five commits landed after `b3146d4`: the Calibrate thresholds fix (`7867517`), AF1S-A2 (`d882e11`), MIRROR1-B
(`dc17349`) and their two docs commits. B's diff failed `git apply` there on `docs/changelog.md` and
`docs/internal/PROJECT-BRIEFING.md`. So B was rebased onto `b4392d0` the same way as before (scratch:
`vr/resume/rebase/`):
- **Snapshot:** B's tree `8058ee1` (`B_TREE_before_rebase`), its files (`b-files-before-rebase.tgz`), its diff
  (`b-on-b3146d4.diff`).
- **Four files changed on both sides,** each merged three-way with `git merge-file` (`m3/`):
  - `docs/changelog.md` conflicted. B's entry is first, then `shulib-v2`'s in its own order. Checked: the merged file
    is `shulib-v2`'s plus exactly B's 32 lines.
  - `docs/guide/14-what-it-cannot-do-yet.md` and `test/README.md` merged cleanly. B's hunks were carried
    line-identical and read.
  - `docs/internal/PROJECT-BRIEFING.md` was taken as `shulib-v2` has it, then regenerated after the build.
- **The move:** the index went with `git read-tree -m -u 4076a77 85b44e3`, after those four files were reset to the
  index. The index is now `b4392d0`'s tree (`85b44e3`).
- **Checked:** the 57 paths where B's tree differs from its snapshot are all paths `shulib-v2` changed.
- **The new docs** in those commits make no claim about the route builder, the driver file or its feel (grepped).
- **The drive-line patch was re-cut again.** Its changelog hunk's trailing context is now MIRROR1-B's heading. It was
  applied with `patch --fuzz=2` to copies (`vr/resume/patch/`). Every `+`/`-` line is identical, and
  `git apply --check` is clean. The previous cut is `vr/resume/patch/drive-line-feel.patch.on-b3146d4`. Its C++ half
  is unchanged: `drive_program.cpp` and `drive_program_source_test.cpp` are untouched by the five commits.
- **The coordinator now applies B onto `b4392d0`.**

**A1's files touched this resumption:** guide 22 only (already on finding 27's list). `controls_tab.mjs` and
`controls.page.test.mjs` are B's.

**Gates on the final tree** (rebased onto `b4392d0`; logs in `vr/resume/gates2/`, and before the rebase in
`vr/resume/gates/`). Every gate below also passed on the tree before the rebase. Its counts were: suite 3782 cases,
919 verbatim lines, 6439 entities, 260 menu pages, 46 tone pages and 258 headers; the other counts were equal.
- **Briefing and build:** the binary was removed, the briefing regenerated, then `cmake --build --clean-first`
  (319 objects, 0 warnings). The doc gates ran in it: 995 lines verbatim, 7 whole-run listings. The briefing was
  regenerated again after.
- **Suite:** green, **3823 cases, 3 884 522 assertions** (`shulib-v2`'s 3 819, plus B's 4), under the 4 GiB cap.
- **ctest:** 346/346.
- **Node:** 117 pass, 1 skip (GL0's real file).
- **Page:** Firefox **31/31**. Chromium (cgroup cap), five runs: **28 pass, 3 skipped** (the Firefox-only tests)
  each. The Chromium redo probe: 0 of 100.
- **Doc tools:**
  - `api_doc_tool`: self-test, check-fresh, check-examples, check-coverage (6478 entities) and check-removability,
    all clean;
  - `doc_staleness_audit`: clean;
  - `briefing_status.py check`: current;
  - `doc_readability`: check-nav (262 pages), check-tone (47) and self-test (159 cases) OK;
  - `route_preview.py self-test` and `first_week.py check`: OK.
- **ARM and CI greps:** ARM header gate exit 0 (259 headers). CI greps: PROS-free, sim-free, LF, and no CR in B's
  files.
- **The src gate:**
  - self-test OK (377 cases) under `timeout -s INT`;
  - then `check` PASS (14 builds);
  - no "(self-test plant" in `git diff -- Makefile src/`, before or after `check`; nothing untracked under `src/` or
    `include/`.
  Both passed on each tree. After it, the binary was removed and relinked (319 objects), and the suite was green
  again (3823).
- **B's diff:** `vr/resume/make_diff.sh` writes `scratch/rmp1-b/rmp1-b.diff` and `rmp1-b.files`. It is checked with
  `git apply --cached --check` onto `b4392d0`, and applies.

**Prose review (rule 4), greps run and read before writing this:**
- **"never loses", "are kept", "is kept", "stays one", "one Ctrl+Z"** over guide 22, `docs/route-builder.md`, the
  route-builder README, the docs front page, guides 18 and 21, and cookbook 06. One false hit: guide 22 :150. It is
  rewritten (new finding 1).
- **"You changed it here", "not read in", "today's file", "today's driver file", "Your folder has no"** over `docs/`,
  `README.md` and the two READMEs (not `docs/internal/chunks/`, not `docs/api/`):
  - guide 22 :148 and `docs/route-builder.md` :39 ("reads today's file"): true when the folder holds it, and the page
    now says why when it does not;
  - nothing quotes the pane's sentence.
- **"read only", "newer route builder"** over guide 22, `docs/route-builder.md` and the route-builder README: guide
  22's refusal table, row "A newer project file", is reworded.
- **The words-to-avoid grep** over this resumption's added guide lines: no hit. `doc_readability check-tone` is clean.

## RMP1-B — the coordinator's merge *(2026-11-17)*

- **Source:** the `rmp1-b` worktree as built, reviewed (three code-review passes), fixed, verified and fixed again
  (workflow `wf_85869230-05f`, 7/7), rebased by its run onto `b4392d0`; carried as `rmp1-b-on-main` (`99f2230`, its
  tree on parent `b4392d0`) and squash-merged in `integrate` onto main `279c297` (ROB1-BT had landed since).
- **The drive-line feel patch** (`scratch/rmp1-b/drive-line-feel.patch` — the team lead, 2026-11-16: "Yes, put it in"):
  `driverLineMatchesFeel()` and its `static_assert` gone; "shulib Drive" prints its feel on its own boot line,
  `feelLine()` — made from the driver file's `kFeel`. Its second hunk was re-cut by hand: ROB1-BT had reworded robot
  two's driver line (now "robot one's map -- the team's"); both robots' lines end "its feel (the next line)".
- **Conflicts (4):** the changelog (B's entry on top); cookbook 06 (ROB1-BT's robot-one text + B's "three checks");
  `how-the-docs-stay-true.md` (chapter 21 quotes B's golden, the driver-control recipes the live robot-one file); the
  briefing (regenerated). Guide 22's "until robot two's update" lines made present tense (it has landed).
- **Fixed at the merge:** GM22 (`test/robot_one_driver_test.cpp`) read 0 rows from robot one's real driver file —
  ROB1-BT's `[the map]` heading is two lines; B's golden has one. Its reader now starts after the route builder's
  `// [control words …]` marker (as the page reads it), else `[the map]`.
- **Gates on the merged tree:** the suite green through `suite_run.py` (**3,850 cases, 3,885,928 assertions**); node
  **120/120**; page tests (Playwright 1.64.1, its own browsers) **Firefox 31/31**, **Chromium 28 + 3 skipped** (Chromium
  must run under a cgroup cap — `systemd-run --user --scope -p MemoryMax=6G` — an address-space `ulimit -v` kills it at
  launch); src gate **self-test OK (398)** and **check PASS**; `ctest` **353/353**; ARM header gate 0 (259); CI greps
  clean; api fresh; the briefing current; staleness clean; the strict site build (prepare_site → the route builder →
  `mkdocs build --strict`) exit 0, no warning.
- **Mutations: 3 of 3 RED** — MR-1 robot one's map loses its L2 row (GM22); MR-2 Drive's boot prints no feel line
  (the drive program's source test); MR-3 (by hand) `DriverFile`'s feel range check removed → 4 compile refusals
  (rm01, rm_ct_a_feel / strafe / turn) fail; restored, byte-identical.
- **Open (for the coordinator / the team lead):** (1) ~~cookbook 06 quotes robot one's LIVE driver file~~ — PROBED
  after the merge (a student's change to `kControls` in the live file): `check-examples` still passes — every listing
  is also an unbroken run of B's `controls_one_builders_map.hpp` golden. Only cookbook 06's prose ("the build fails
  when a quoted line changes in the file") was stale: reworded in the next commit. (2) `test/robot_twin_test.cpp` holds the two
  driver files' marked regions equal: an export on ONE robot turns the host suite red until the other robot's file
  gets the same change — the Controls tab should write both, or say so. (3) The side-toggle mirror (ROB1-BT's
  finding 1) for the route export.

## RMP1-A2e — the preview ENGINE (the real library compiled to WebAssembly)

Executor: Claude Opus 5.5 (the coordinator's `opus` subagent), worktree `.claude/worktrees/rmp1-a2e` (branch
`rmp1-a2e`, from `e8fa3de`). Scope: A.7's A2e row. Not committed, not pushed; host only.

### Log
- 2026-11-16 12:15 EST — started: read RESUMING's standing rules, the brief's header, ANSWERS, amendment A.1–A.12, R1–R19, D1–D20, §6, §7, §8 and every A2e row.
- 2026-11-16 ~12:40 EST — built: `route_lines.mjs` (the page's route → the lines), `test/route_maker_runner.hpp` (the
  runner + the record writer), `route_maker_main.cpp` gains `preview` (and, in the test module only, `golden`/`goldens`),
  `test/route_maker_goldens.cpp` (WG1's compiled twins), `tools/route_maker/wasm/CMakeLists.txt`, the WG8 golden. The
  pinned emsdk is the probes' (6.0.13), used READ-ONLY (`EM_FROZEN_CACHE=1`, `TMPDIR=build/tmp`). First facts: the
  module builds with ZERO warnings under `-Werror` in ~9 s (307 MB); GCC 13 caught one `-Wsign-conversion` Clang did
  not (W14's two compilers, in action — fixed). Sizes: `preview.wasm` 513 KB (218 KB gzip), glue 10 KB; the one file
  694 KB (base64 — `-sSINGLE_FILE_BINARY_ENCODE=0`: Emscripten 6's default is the UTF-8 "binary" encoding P-W2 did
  not recommend). Dropping iostreams from the runner saved 85 KB; reading stdin only natively dropped the glue from
  55 KB to 10 KB (no file system). **WG1 on WG8: the runner and the compiled export — byte-identical.** Native vs
  WebAssembly on WG8: every step, line and time equal; poses differ in the last bits (e.g. a target heading
  0.21755411913400838 vs …840), as A.2 says.
- Emscripten's Node glue sets `process.exitCode` from main's return: the test helper restores it after each
  `callMain` (else a refused-lines case made the whole test FILE fail with exit 2). For A2u: irrelevant in a Worker.
- ~12:45 — **A REAL DEFECT the first sweep found, fixed at its root: `callMain` leaks its arguments on the module's
  C stack.** WG2s's 300 routes on ONE instance died at route 56 ("memory access out of bounds"); seed 56 alone ran
  fine. Bisected: any route repeated ~35–53 times on one instance crashed. A debug build (`-sASSERTIONS=2
  -sSTACK_OVERFLOW_CHECK=2`) named it: "stack overflow (Attempt to set SP …)". Emscripten 6.0.13's `callMain`
  (postamble.js :23-100) `stackAlloc`s argv and the lines on the 64 KB C stack and NEVER restores it — it is made
  to be called once; P-W1f called it twice. The page's Worker will call it once per PREVIEW, so a student's ~20th
  PREVIEW would have corrupted the module's memory silently. Fix: `src/host/engine_module.mjs` — ONE wrapper the
  tests AND A2u's Worker use — saves and restores the stack pointer around every call (`stackSave`/`stackRestore`
  exported); the module links with a 1 MB stack (a run needs 16–32 KB: probed at 8/16/32 KB over every golden and
  300 sweep routes — 16 KB overflowed on gl1, 32 KB ran all 319) and `-sSTACK_OVERFLOW_CHECK=2`, so an overflow
  ABORTS instead of corrupting a record (~5 % slower: 319 routes 2.85 s vs 2.71 s; +24 KB). Pinned by a test (400
  PREVIEWs on one instance with a 4 KB argument) and mutation M1.
- The sweep generator (`native.test.mjs`'s `sweepRoute`) tuned so it tests what W2 is about — long closed-loop runs:
  8–24 steps, mostly carry on, a wait on a never-started mechanism (a THROW) kept but rare. Over seeds 1–300: 4,716
  page steps, 3,699 steps from below, every verdict kind (ok 2,795, failedWentOn 700, skipped 614, jumped 456,
  failed 88, notReached 57, threw 6), 12.2 s of auton per route on average.
- WG8's last drive (turned first, `skipTo("end")`) now FAILS (0.35 s) so the skip is taken — without that, a runner
  that forgot the onFail before a turn-first DRIVE survived WG1 (it only shows when that drive fails; M3).
- Gates so far: native + module build clean; suite green **3,779** (+1: WG8 compiled with GCC `-Werror` and run on
  robot x — `test/route_maker_golden_test.cpp`); `node --test tools/route_maker/test/` 85 pass, 12 skipped (the
  browser and engine tests, by their env vars); the engine's 10 node tests and the browser WG3 (Playwright's Firefox
  157, A.5's recipe) green; ARM header gate exit 0 (259 lines); PROS-free and sim-layering greps clean;
  api_doc_tool self-test / coverage / fresh / examples / removability, staleness audit + self-test, briefing
  self-test + check — all clean; `git diff -- src/ Makefile include/` EMPTY.
- ~13:10–14:10 — **Mutations** (driver `.claude/coordinator/scratch/rmp1-a2e/mutate.py`: each file backed up under
  that folder, restored in a `finally` and on SIGTERM/SIGHUP/SIGINT, its mtime bumped; every dependent build made
  FRESH — the module, the native program or the suite — before its test; everything under the 4 GiB cap). The last
  round, after the final edits, all as expected:
  RM27 startMove run as `move` → WG1 RED, and SM RED (a start form must take no time); RM28 a preset goal built
  `computed = true` → RED in the new suite case (`route_maker_runner_test.cpp`: the runner's goal equals the
  compile-time `m["name"]`, field for field) — it is INVISIBLE to WG1 by construction (`computed` matters only for
  a target past the travel limits, and no preset can be one), so the brief's "RM28 → WG1 red" could not hold; RM32
  `opt.onLoop` unset → WG7 RED; RM35 two numbers read inside one call's argument list → WG2 RED natively (GCC
  evaluates right to left), and the same mutant leaves WG1 GREEN (all WebAssembly — W8's point, observed); RM37 the
  prefix map dropped → RG1 RED; RM22 (re-cut for the runner) a page step's span one pace late → SM and SM1–SM3 RED;
  RM34 the lines print a 4th decimal → NF1 (the lines) RED; and M1 the wrapper leaves the C stack → "hundreds of
  times" RED; M2 turn-first without its face → WG1; M3 the turn-first DRIVE without its onFail → WG1 (after WG8's
  last drive was made to fail); M4 the cut called a failure → SM3; M5 the brain's numbers one low → SM1; M6 a
  negative literal read positive → WG1; M7 a jumped step called skipped → SM1; M8 a refused line run anyway → the
  refusals test. **Two survived the first round and were fixed by strengthening the tests, not by loosening:** RM27
  (WG8 had no drive between a start form and its wait — three motions inserted, and SM now pins a start form's
  zero-length span) and RM28 (the suite case above).
- **A lesson for the coordinator (the mutation driver tripped it once):** building `shulib_tests` runs the doc
  gates, whose briefing check RUNS the existing suite binary — after a suite mutant the next build's gate ran the
  MUTANT (under the inherited 4 GiB cap) and failed the build, leaving the mutant linked. The driver now unlinks
  `build/test/shulib_tests` before every suite build (the rule's own advice); the tree was rebuilt and re-verified
  (suite green 3,781, briefing check current).
- **WG2s, the full sweep (seeds 1–3,000):** 128 s; **0 differences** in any step, verdict, pace, time, line or exit
  code; poses at most **2.89e-7 in and 1.07e-8 rad** (bounds 1e-5 / 1e-6: ~35× and ~93×) — the brief's checker
  measured 3.5e-7 / 2.5e-8 on its own sweep. Those 3,000 routes: 47,772 page steps, 36,184 steps from below,
  12.2 s of auton each on average; ok 27,426, failedWentOn 6,848, skipped 7,150, jumped 4,528, failed 1,000,
  notReached 750, threw 70 (no route reached the 57 s cut — SM3 and gl13 cover it). CI runs seeds 1–300 (max there
  4.3e-12 in).
- **WG3 in a browser** (`test/page/engine.test.mjs`, Playwright's Firefox 157 here; every CI row's browsers): the
  module in a MODULE Worker through `engine_module.mjs` gives every golden's record byte for byte as Node does.
  Observed red on one changed byte in the stored records (Node and browser), and on a harness that altered one
  browser record (a temporary copy, removed).
- **Deltas from the brief, each with its reason:**
  1. *The lines carry a mechanism's `.name`, not its C++ name* (`route_lines.mjs` maps one to the other from the
     robot's description): the C++ name is read from the source line the declaration recorded, which the module
     cannot read. A ledger level travels as its preset's name (the goal `m[facts::lv(i).name]` builds is that
     preset's).
  2. *The module's `describe` refuses* ("runs in the native program only") instead of printing descriptions with
     null C++ names; the page's descriptions are the native program's, baked at its build (A.3.2 (b)). Embedding the
     robot files in the module would put source comments where the site's internal-docs guard and RG1 scan.
  3. *The record of WG1's compiled twin has `"pageSteps": null`* (nothing dates its steps): WG1 compares the two
     records byte for byte with that ONE line set aside; the page-step spans are pinned by SM instead (motion spans
     exact against the motions, the steps tiling the run, a start form taking no time, the verdicts and the brain's
     numbers on SM1–SM3).
  4. *SM3's "every later step not reached"* is the library's own word: each later step is `skipped` (it logs
     "skipped — stopped at step N"); `notReached` is kept for a step the run never got to (a throw, the no-pace stop).
  5. *A1b's kinds (triggers, the wall reset) are NOT in the runner*: neither is "trivially the same mapping" (the
     trigger chain is a template over the leg kinds) and A1's model and exporter have neither, so no golden could
     hold WG1 for them; the runner refuses them by name (`cannot run a "wallReset" step`). A1b adds them to the
     exporter and the runner together, with their goldens.
  6. *RM28's oracle is a suite case*, not WG1 (above). *RM22 and RM35 re-cut* for the runner (a span one pace late;
     a stateful reader inside one call).
  7. *`engine_module.mjs` is new* (not in A.6's list): the ONE way to run an instance, because `callMain` must have
     its stack restored after every call (the defect above) — A2u's Worker builds on it, never on bare `callMain`.
  8. *`-sSTACK_SIZE=1MB -sSTACK_OVERFLOW_CHECK=2` and `-sSINGLE_FILE_BINARY_ENCODE=0`* beyond A.3.3's flags
     (measured above); the browser floor is unchanged (no new feature).
  9. *The module's `.wasm` is 537 KB (221 KB gzip)*, above the probe's 419 KB (-Os): the runner, the record writer
     and robot one's autons; iostreams were dropped from the runner (−85 KB) and stdin from the module (glue
     55 → 12 KB).
- **Prose review (rule 4), run BEFORE writing this:** `grep -rn 'shulib_route_maker\|route_maker_main\|route_maker_describe\|route_maker_golden_test\|engine\.mjs\|WebAssembly\|preview engine' docs/ README.md`
  (outside the chunk files) → one hit, `docs/internal/RELEASING.md` :146 (a publish builds the builder from
  `shulib_route_maker describe`) — still true (A2e does not put the module in the site; A2u does). `grep -n -i
  PREVIEW` over `docs/route-builder.md`, guides 14, 16, 22 → guide 22 :166 lists PREVIEW among what the builder
  does not do yet — still true until A2u; guide 16 is RPV1's command — unchanged. The brief's word greps
  ("simulat", "honest", "obviously", "just ", "simply", "easy", "precondition", "radian", `http`) over every new and
  changed file: no hit in this run's text (one pre-existing "stated honestly" in `ci.yml` :147, not this run's;
  "radians" only as API names in a test and the README's unit of `theta`). The built module: no URL, no path
  (`/home/`, `/Users/`, `C:\`, `/runner/`, `.claude`), no internal-doc name.
- **Documents:** touched — `tools/route_maker/README.md` (the engine: build, the pin, the outputs, the wrapper, the
  lines, the record's schema 1), `test/README.md` (the runner, WG1–WG8, SM, RG1; WG8 in the golden test),
  `docs/internal/PROJECT-BRIEFING.md` (regenerated), this log. Deliberately NOT touched: `docs/changelog.md` and
  every public page (no user-visible change — PREVIEW reaches the page in A2u, whose entry says so); `roadmap.md` /
  `build-order.md` (RMP1's runs are not tracked there; the coordinator's at the merge); `pages.yml` and
  `build.py` (putting the module in the site is A2u's); `engine.mjs` (A2u wires it).
- Added after the last full round: an engine test that robot one previews on ITS OWN list (the robot line names
  cascade, both toggles, claw, flipper, ramp) with its ledger's OWED facts marked (11 today, each a name in
  `src/robots/robot_one_facts.hpp`), and a fixture robot with none — its mutation M9 (robot one without its ledger)
  is run after the src gate (the gate plants into `src/`, which the module build compiles).
- **2026-11-16 22:50 EST (`date`: 22:50:43) — RESUMED** after the team lead's pause (~12:30). Found: a src gate
  self-test PLANT left in `src/programs/bench_r3a.cpp` (case 37's `ser1SelfTestFlush`, NO marker comment — the killed
  gate never restored it) → restored with `git checkout`, its mtime bumped; `git diff -- src/ Makefile` then EMPTY and
  no untracked file under `src/ include/ test/` but this run's own. The mutation driver's four backups equal the
  current files (no mutant left). Rebuilt from source: `build/wasm` deleted, the suite and native binaries unlinked,
  the briefing regenerated before and after; host build 0 warnings; suite green **3,781**; module 0 warnings (14 s;
  `preview.wasm` 537,336 bytes, as before); `node --test tools/route_maker/test/` 85 pass / 13 skipped; the engine's
  11 node tests green (WG2 on the goldens: max 5.33e-14 in / 6.88e-15 rad; WG2s 300 routes: 4.32e-12 in /
  1.14e-13 rad). Continuing from where it stopped: the src gate (self-test, check), M9, the remaining gates.
- **2026-11-16 22:59 – 11-17 00:30 EST — the rest of the gates, M9, and a pre-existing break found on main.**
  - **The src gate self-test: OK (377 detector cases)** — 52 min: SIX self-tests ran on this laptop at once
    (integrate, af1s-a2, mirror1-b, rmp1-b, rob1-bt2, this one). At case 37 after 44 min, my own `timeout 3600`
    would have interrupted it near its end (an interrupt is how the morning's plant was left), so I killed ONLY that
    `timeout` wrapper (SIGKILL; the gate ran on under its own PID) and waited on the PID. Afterwards `git diff -- src/
    Makefile include/` EMPTY, no "(self-test plant", no untracked file under `src/` or `include/`, the `test/` diffs
    exactly this run's (99+/15−, as at the resumption). **`check`: PASS (14 builds).**
  - **M9 (robot one previewed without its ledger) → RED** in "any robot's own list" (the driver: the module rebuilt
    fresh, restored, then the module, the native program and the suite rebuilt from the restored source: ok). Every
    other mutation's verdict stands from the 13:02 round (the runner has not changed since: its file equals the
    driver's last backup).
  - After M9's rebuild: the engine's 11 node tests green again (same maxima); WG3 in Playwright's Firefox green; suite
    green **3,781**; briefing regenerated, check current; the ARM header gate exit 0 (259 lines); PROS-free and
    sim-free greps clean; api_doc_tool self-test OK, check-coverage (6,439 entities, 214 headers), check-fresh exit 0,
    check-examples (919 lines), check-removability clean; staleness self-test OK and clean; compile refusals
    **295/295** (TUs 295); `route_preview.py self-test` ok; `node --test tools/route_maker/test/` 85 pass / 13
    skipped; §7's site checks (no URL, `app.html` and no `index.html`, the SITE policy once, the `data:` favicon in
    both pages, RG1 on the site and the `.wasm`, `eol: lf`) all hold.
  - **A PRE-EXISTING BREAK ON MAIN (not this run's), for the coordinator:** §7's Firefox page suite fails 16 of 29 —
    13 are `locator.selectOption` timeouts on `.pane.right select[name="preset"]` (e.g. `page.test.mjs` :335,
    `selectOption({ index: 3 })` on robot one's cascade), 3 a screenshot protocol error after them. Robot one's
    cascade bakes with `presets: []` (its levels are NONE since ROB1-C2), so no level can be picked. **The base
    `e8fa3de`, built in a scratch copy (`git archive`), fails the same tests the same way** (19 of 28 there; its
    `robots.json` equals this run's but for the build stamp) — the A1 merge re-ran the suite and `node --test`, not
    the page tests, on robot one's level-less cascade. It will turn this branch's CI browser rows red. NOT fixed here:
    the page tests are A1's (RMP1-B is editing A1's shell beside me), and the fix needs a ruling (a fixture robot with
    levels baked for the tests, or the page's own behaviour for a cascade with no levels). Evidence:
    `scratch/rmp1-a2e/base/page-base.log`; this tree's `build/tmp/page.log`.
  - README: the wrapper returns `{rc, text, record, words}` (it said `{rc, record, words}`).
- **Stopping here for the review** (three reviewers, fixes, a fresh verifier, the coordinator's checks, then the CI on
  the pushed branch). Nothing committed, nothing pushed.

### RMP1-A2e review findings and their disposition
- 2026-11-17 started: three reviewers' 22 findings (correctness, tests, docs), deduplicated to 17; each re-checked
  against the tree before anything was changed. Nothing committed, nothing pushed.
- **The findings, each re-checked, and what was done** (C = correctness, T = tests, D = docs; overlaps merged):
  1. **CI's browser rows are red before this run (C blocker, D major, T minor).** REAL, NOT this run's: A1's page
     tests pick a cascade level robot one no longer has (ROB1-C2: its levels are NONE). RMP1-B's worktree already
     carries the fix in A1's file (`page.test.mjs`: `ONE_LEVELS`, the claw set open in the move's place), so it is
     not made twice here. This run's part: the browser WG3 is now **its own CI step, before the page's**
     (`ci.yml`, "The preview engine in <browser> on <os>"; the page step no longer sets `ROUTE_MAKER_WASM`, so
     `engine.test.mjs` runs once), so each row's engine result reads on its own. **For the coordinator:** the page
     step stays red on every row until RMP1-B's page fix merges — merge it first, or read this branch's engine
     steps alone and rule on A.4's "every row green".
  2. **Robot one's owed facts and PLACEHOLDER marks had no real guard (C major, T major).** REAL. The test now reads
     every `owed(<placeholder>, Stage::<s>, "<name>")` from `src/robots/robot_one_facts.hpp` itself (comments
     aside: 11 today, never typed into the test), asserts the list is not empty and the record's `owed` is exactly
     it, and that the cascade's step carries RPV1's PLACEHOLDER note while the claw's two steps carry none.
     (A step's PLACEHOLDER is RPV1's "starting values" rule, `placeholderFor`, not a ledger fact's name — so the
     test checks the note RPV1 writes, not a fact name.) Mutants N1 (the ledger emptied, its file name kept), N2
     (`owed` written empty) and R1 (placeholders blanked): see the mutation round below.
  3. **ROB1-BT2 would break the engine's link (C major, D major).** REAL. **Fixed at its root, not by a longer
     list:** the engine never runs the team's autons — the runner gives every RPV1 robot row its own register
     function — so `src/team/` was linked only to satisfy those rows' two function pointers. Both targets now link
     `test/route_maker_no_team_autons.cpp` instead (the two register functions, each throwing
     `std::logic_error` if ever called). So: ROB1-BT2's `skills_route.cpp` needs no edit here at any merge order;
     the students' freeze-open files never enter Emscripten's `-Werror` gate (W14's list stays `include/`,
     `src/robots/`, the host test plant); `preview.wasm` 537,336 → 535,355 bytes. **Still ROB1-BT2's merge, named
     for the coordinator:** `route_maker_goldens.cpp` compiles `smoke_robot_two.cpp` under robot two's namespace
     exactly as A1's `route_maker_golden_test.cpp` :102 does — the same edit at the same merge (A1's suite test
     needs it anyway); robot two's ledger in `robotSource("two")` (A1's warning, `route_maker_robots.hpp` :38-42);
     and the module build plus the engine's node tests in that merge's done list (it changes `src/robots/`, W14).
  4. **describe read a robot's files from the working directory first (C minor, D minor).** REAL. A relative name
     (the prefix map's) is now opened from `SHULIB_SOURCE_DIR` only — the working directory only in a build that
     defines none. Pinned by a new native test: describe run from a decoy tree (robot one's files copied, every
     declared name changed) and from `/` gives exactly what it gives from the repository root (mutant N3, the
     reviewed code: RED). Logged as a W10 delta (below).
  5. **Coverage: WG2s on robot x only, 2 decimals; no ledger level end to end (C minor).** REAL for the sweep:
     WG2s now also sweeps robots one and two (their baked descriptions; a third as many routes each, 3-decimal
     numbers), and every module record in it is held to SM's span rule too (`spansHold`, now in `engine.mjs`) —
     500 routes in CI. Robot x's routes are unchanged: seeds 1–3,000 give the same lines as the generator the
     report measured (checked). The ledger level: NOT possible today — robot one's cascade has no levels, and no
     fixture robot has a ledger; **handed on:** the run that types the cascade's levels (station 13) adds a robot-one
     golden that moves to a ledger level, so WG1 covers `facts::lv(i).name`.
  6. **Nothing checked what the record says (T major).** REAL. A new test checks the record against the route
     itself: the start in `samples[0]` (x ≠ y, a 30° heading in radians), a sample every 10 ms, the drive ending at
     its (x, y), the drive's target, the page step's brain numbers, park and stop set to 55.5 / 58.25, a throw
     (`threw`, "PROGRAM ERROR", its sentence, `threw` then `notReached`, no samples or brain numbers), and a drive
     through the right wall (a TOUCH then the THROUGH, `throughAt` its sample). Mutants R2–R6, R11, R12.
  7. **`preview()` never called by a test (T minor).** REAL. A new test: a refusal (rc 2, record null, its words),
     then a good route on the same instance (rc 0, empty words, the record parsed and equal to its text). Mutants
     J1–J3. (The test helper's `instance()` now hands out `.preview` too, keeping Node's exit code.)
  8. **The registry's refusal path untested (T minor).** REAL. A new test: `park=59 stop=57` and an empty name —
     rc 1, `refused`, "REFUSED", the registry's sentence, no samples, every page step `notReached` with no brain
     numbers. Mutant R7.
  9. **Two refusal rules, `threw`, `lib` of an unreached step (T minor).** REAL. The refusal table gains `face=2`,
     points on a drive, and a curve without points; the throw case is item 6's; WG7 asserts `lib` null on its step
     never reached. Mutants R5, R6, R9, R10.
  10. **SM4 neither done nor logged (T minor, D minor).** REAL (a log gap). Delta 10 below.
  11. **The mirrored (two-robot) run (D major).** REAL: no run's row owns the engine half. NOT built here — delta 11.
  12. **The browser test let every warning pass; A1's handoff not acted on (D minor).** REAL. ONE list now,
      `test/page/allowed_warnings.mjs`, holding Firefox's sentence EXACTLY as Firefox 157 logs it (observed: a
      Worker's warning reaches the page's console wrapped as `[JavaScript Warning: "<sentence>"]`, once per
      instance); `engine.test.mjs` fails on any other warning or error. Firefox green with the one warning allowed;
      Playwright's Chromium green with none; one word changed in the list's sentence turns the Firefox run RED
      (mutant N4, by hand: so the check does see the Worker's warnings). NOT edited: A1's `page.test.mjs` (RMP1-B
      is editing it) and A2u's `preview.test.mjs` (its own copy of the sentence, matched with `includes`) — **for
      the coordinator:** at A2u's merge both import this list (the page first loads the module in A2u).
  13. **Putting the module into the site is handed to nobody by name (D minor).** REAL as a log gap; the work is
      already A2u's in fact: its worktree's `build.py` gains `--engine` (the Worker script, `preview.wasm`, the one
      file's module inline) and its `pages.yml` +25 lines. Delta 12 below names the pieces.
  14. **The coordination note missed RMP1-B's files (D minor).** REAL. Corrected in delta 13.
  15. **The prose-review record was narrower than claimed (D minor).** REAL. Re-run with the full pattern; recorded
      below exactly.
  16. **The README's engine section (D minor).** REAL, all four: (a) the engine is "behind the page's PREVIEW
      (which RMP1-A2u wires into the page)"; (b) rc 3 documented, and `.preview()` THROWS when the module aborts
      (the instance is then dead); the guidance "after a non-zero rc make a fresh instance" was too broad (~~rc 1 is
      also a plain failed drive~~ — WRONG, corrected in the verification round below: a failed drive is rc 0; rc 1 is
      also a throw, a THROUGH or a refusal, none of which harms the instance) — now: after the no-pace stop
      (`looped`) the instance runs on correctly (WG7) but
      a fresh one frees that run's heap; (c) the overflow count measured for the shipped 1 MB stack, the wrapper's
      restore removed: 2,224 PREVIEWs of a 335-byte route, 679 of the longest golden's 1.4 KB ("some hundreds"),
      in the README and `engine_module.mjs`; (d) `test/README.md` names `route_maker_runner_test.cpp`.
  17. **The describe change was unlogged (D minor).** REAL. Delta 14 below; `test/README.md`'s `shulib_route_maker`
      bullet now says the kept names are relative and opened from the build's tree, and that it links
      `route_maker_no_team_autons.cpp`.
- **Deltas added by the review** (numbered on from the nine above):
  10. *SM4 is not in this run*: robot two's numbering with a "robot one only" block needs a route that can SAY
      "robot one only" (A1b's "who runs it" blocks; A1's model has none) and the two-robot run (delta 11). It lands
      with whichever run adds the two-robot run's engine half — the coordinator names it.
  11. *The two-robot PREVIEW's engine half is in no run's row, and is NOT built here.* D6 wants a mirrored route's
      PREVIEW to run robot one's entry AND `competition::mirrored(...)` on robot two, with MIRROR1-A's pair check;
      the amendment dropped RPV1's `--pair` ("the runner records the run itself"), and A.7 gives "the two-robot
      PREVIEW (when MIRROR1-A has merged)" to A2u while the runner is this run's file. Today the lines carry no
      mirror word, `previewRobot()` runs one robot, and the record holds one run. Not built now because both files
      it rests on are being changed beside it: MIRROR1-B (branch `mirror1-b`, `a71f16d`) rewrites
      `test/mirrored_pair_check.hpp` (+209/−), and ROB1-BT2 rewrites robot two (its claws retired) and first
      registers the mirrored skills route. **What the owner adds** (for the coordinator's ruling — A2u by its row,
      or an engine follow-up): a `mirror=two` word on the route line (from `mirroredFor`); the runner runs the entry
      on robot one and the mirrored entry on robot two; `comparePair`'s verdict (the first moment within
      `kMirrorGapIn`, its sentence); the record carrying both runs and the verdict (schema 2, or a `mirror` member
      — A2u's playback reads it); WG1's twin through the export's mirrored line, GL16's pass and fail goldens, SM4,
      and their mutations.
  12. *The module into the site is A2u's, named:* `build.py` (the module's files in the SITE form, the one file's
      module inline, RG1 on the decoded module), `pages.yml` (the pinned emsdk, cached; the module build), PG1's
      `.wasm` check after `mkdocs build`, and RELEASING.md's "press PREVIEW once". A2u's worktree already has
      `build.py --engine` and `pages.yml` +25; the coordinator checks PG1 and RELEASING.md are in its args.
  13. *The coordination list, corrected* (the report said "no file owned by an in-flight chunk" — wrong). Shared
      with **RMP1-B**: `test/route_maker_main.cpp` (it adds `feel` / `feel-edges`; this run restructured `main`
      into `run()` — a certain conflict, and RMP1-B's `route_maker_feel.hpp` then compiles into the module too, so
      whichever merges second rebuilds the module and runs the engine's tests), `test/route_maker_describe.hpp`
      (it adds `unhomedVolts`; this run changed `identAt`), `test/README.md`, `tools/route_maker/README.md`
      (neighbouring rows of the layout table), this log. Shared with **A2u**: `tools/route_maker/README.md`, this
      log. Shared with the earlier list: `ci.yml`, `test/CMakeLists.txt`, `test/route_maker_golden_test.cpp`, the
      briefing. **Main has moved** since the base: `26ee38e` (DOCS3-N1), `1bcde4e` (the between-parts marker atop
      this log), `b3146d4` (the briefing).
  14. *describe opens a kept file name from the build's tree* (A1's `route_maker_describe.hpp`): W10's prefix map
      made the names relative (`test/../src/robots/…`), so `identAt` opens them under `SHULIB_SOURCE_DIR` (set on
      `shulib_test_flags`: the repository root) — from any working directory, never another tree's files.
  15. *The engine does not compile `src/team/`* (finding 3): `test/route_maker_no_team_autons.cpp` in both link
      lists. Before this run `shulib_route_maker` linked `route_maker_main.cpp` alone; the first round had added
      the two team files to satisfy RPV1's rows (unlogged then) — now neither.
- **The mutation round** (the same driver, `scratch/rmp1-a2e/mutate.py`, 37 mutations: every first-round one again,
  since the tests moved, plus the review's; each file backed up there, restored in a `finally` and on signals, its
  mtime bumped; the module, the native program or the suite rebuilt FRESH before each test; all under the 4 GiB
  cap; log `scratch/rmp1-a2e/mutations-review-round.log`). **35 as expected:** every first-round verdict holds
  (RM27, RM27b, RM28, RM32, RM35 red and RM35b green as designed, RM37, RM22, RM34, M1–M9); and now RED — N1 the
  ledger emptied (file name kept), N2 `owed` written empty, R1 placeholders blanked (all in "any robot's own
  list"); R2 park/stop swapped, R3 a target's x/y swapped, R4 THROUGH written TOUCH, R5 a throw called failed,
  R11 `threw` always false, R12 the samples' x/y swapped (the record-content test); R6 `lib` [0, 0] for a step
  never reached (that test and WG7); R7 the registry's refusal never taken (its own test); R9 `face=2` and R10 the
  curve-points rule dropped (the refusals); J1–J3 (the `preview()` test); N3 the reviewed `identAt` (the describe
  test). By hand: N4 above. After the round the driver rebuilt the module, the native program and the suite from
  the restored source; the five backed-up files equal the tree's.
  **Two are EQUIVALENT, with the evidence:** R8 (a step begun but never ended, outside a loop, called "looping")
  stayed green on all 15 engine tests — no run can make it: the routine runs synchronously inside `previewEntry`,
  so every reached step returns (dated) or throws (dated in the catch), except in `onLoop`'s record, where `looped`
  is true. R13 (`sampleAt` without its 5 ms rule) stayed green on SM and the 500-route WG2s with spans — a probe
  (`scratch/rmp1-a2e/fix/probe_r13.cpp`, the runner compiled natively) over 5,019 routes (the full sweep and every
  golden) found 156,306 page-step times, and `t + armedAt` equals the clock EXACTLY at every one, so the rule never
  changes a sample. It is kept because it is `foldRun`'s own rule for a motion's start and end: the page steps and
  the motions are dated by one rule, whatever the arithmetic.
- **WG2s, the full sweep (5,000 routes: seeds 1–3,000 on robot x, 1–1,000 on each of robots one and two), 210 s:**
  0 differences in any step, verdict, pace, time, line or exit code; poses at most **1.15e-6 in and 2.08e-7 rad**,
  both from ONE route, robot two's seed 159 (robot one's worst: 2.65e-7 in / 3.64e-8 rad, seed 22; robot x's
  unchanged, 2.89e-7 / 1.07e-8). The bounds hold, but **robot two's heading margin is ~4.8×, not the ~40× W2 set
  from robot x's sweep** — for the coordinator (W2's remedy is a replaced seed only when a run shows a flip; none
  does). CI runs seeds 1–300 / 1–100 / 1–100: 2.65e-7 in, 3.64e-8 rad.
- **Gates on the final tree:** host build 0 warnings; the briefing regenerated before the build (the suite binary
  unlinked) and after; suite green **3,781** (`suite_run.py green`); the module rebuilt from an empty `build/wasm`,
  0 warnings, `preview.wasm` 535,355 bytes (219,931 gzip), the one file 725,884; the site rebuilt (robots one's and
  two's descriptions unchanged); the engine's node tests **15 pass** (WG1, the robot list, WG3, WG7, the stack, SM,
  SM1–SM3, RG1, the refusals, the record's content, the registry, `preview()`, WG2, WG2s, describe); the model's
  `node --test tools/route_maker/test/*.test.mjs` 83 pass, 1 skipped (A1's GL0-on-the-real-file, as before);
  WG3 in Playwright's Firefox 157 and Chromium, green; the ARM header gate exit 0 (258 headers); PROS-free and
  sim-free greps clean; `api_doc_tool` self-test OK, check-coverage (6,439 entities, 214 headers), check-fresh exit
  0, check-examples (919 lines), check-removability clean; briefing self-test OK and check current; staleness
  self-test OK and clean; `ci.yml` parses (YAML).
- **Prose review (rule 4), the EXACT pattern run, BEFORE writing this:** `grep -rn -i
  'shulib_route_maker\|route_maker_main\|route_maker_describe\|route_maker_golden_test\|route_maker_runner\|route_maker_goldens\|route_maker_no_team\|route_lines\|engine_module\|engine\.mjs\|allowed_warnings\|wg8\|WebAssembly\|preview engine\|emsdk\|Emscripten\|wasm'
  docs/ README.md --include=*.md`, the chunk files aside → ONE hit, `docs/internal/RELEASING.md` :146 (a publish
  builds the builder from `shulib_route_maker describe`) — still true. Also read: `docs/guide/22` :166 and guide
  14 :551 (PREVIEW not done yet — true until A2u), `docs/route-builder.md` :18 (Save and Export — unchanged). (The
  first round's logged pattern left out the new files' names and emsdk/Emscripten/wasm; this one is what ran.)
- **Documents touched by the review:** `tools/route_maker/README.md` (the engine section: who wires PREVIEW, the
  team's autons not compiled, the allowed warnings, rc 3, the throw, the overflow count, the no-pace stop's heap),
  `test/README.md` (the describe and link change, the runner's suite case, the new engine tests), the comments of
  `engine_module.mjs`, `ci.yml`, both CMake files, the briefing (regenerated), this log. Deliberately not touched:
  the public docs and the changelog (nothing a student sees changes until A2u), `page.test.mjs` and A2u's files
  (above).

### Resumed 2026-11-17 03:07 EST (`date`) — the fix round's run was stopped by the usage limit (~02:00)
- **Found on resuming:** all 17 findings already dispositioned above, and the mutation round complete (37 mutations,
  each observed on a fresh build: the driver rebuilds the module, the native program or the suite before every
  test, the suite binary unlinked first). Every file the driver backed up (`scratch/rmp1-a2e/backup/`, five) and
  `allowed_warnings.mjs` (N4's) equal the tree's. The run was cut inside `src_build_gate.py self-test` (started
  01:27; the gate list above does not include the src gate). It left one plant, `src/programs/zz_self_test.cpp`
  (01:59), which was already moved out to `scratch/rmp1-a2e/LEFTOVER-zz_self_test.cpp`. Every other file it touched
  has its content back (mtimes only): `git diff -- Makefile src/ include/` is empty, no "(self-test plant" in the
  diff, and `git ls-files -o --exclude-standard src include test` lists only this run's six new test files.
- **Rebuilt from source before trusting anything:** the suite binary unlinked, the briefing regenerated, configured,
  built (0 warnings), the briefing regenerated; suite green **3,781**. The module rebuilt from an empty `build/wasm`
  (0 warnings, `preview.wasm` 535,355 bytes, the one file 725,884 — as above); the site rebuilt.
- **A NEW DEFECT the browser gate found, fixed:** `test/page/engine.test.mjs` opened its HTTP server, launched the
  browser, and only then entered the `try` whose `finally` closes the server. When the browser will not launch, the
  test fails in 0.5 s but the open server keeps Node alive until it is killed. Seen here: Playwright's Firefox not on
  the default path → `timeout 600` killed it (rc 124). In CI, a Windows row whose runner lacks its Edge or Chrome
  channel would hang that step to the job's 6-hour limit instead of failing. Now the launch is inside the `try` and
  the server is closed either way: the same missing browser → rc 1 in 0 s. A1's `page.test.mjs` launches before it
  serves, so it cannot hang this way.
- **Re-run on the final tree:** the model's tests 83 pass, 1 skipped; the engine's 15 pass (native set, records
  written); the other rows' form (records read, no native program) 12 pass, 3 skipped (WG2, WG2s, describe need the
  native program); WG3 in Playwright's Firefox 157 (under `ulimit -v`, `PLAYWRIGHT_BROWSERS_PATH` the shared one) and
  Chromium (under `systemd-run` `MemoryMax=4G`), green against the Linux records, Firefox with its one allowed warning
  and Chromium with none. N4 re-run on this tree (one word of the allowed sentence changed): Firefox RED, the list
  restored and compared.
- **Gates:** the ARM header gate (258 headers) exit 0; the PROS-free and sim-free greps clean; compile refusals 295
  of 295 (295 TUs); `api_doc_tool` self-test OK, check-coverage (6,439 entities, 214 headers), check-fresh exit 0,
  check-examples (919 lines), check-removability clean; briefing self-test OK and check current; staleness self-test
  OK and clean; the src gate run last, under `timeout -s INT`, its self-test **OK (377 detector cases)** (03:29–04:09,
  beside two other worktrees' self-tests) and its check **PASS** (14 builds; robot one's 4 refuse, each naming
  exactly its owed facts) — after it, `git diff -- Makefile src/ include/` empty and no untracked file under `src/`
  or `include/`. `ci.yml` parses. Nothing committed, nothing pushed.
- **Prose review (rule 4) for this round's one edit**, BEFORE writing this: `grep -rn -i
  'engine\.test\|PLAYWRIGHT_BROWSERS_PATH' docs/ README.md test/README.md tools/route_maker/README.md --include=*.md`,
  the chunk files aside → `tools/route_maker/README.md` :36 (the laptop's browser rule) and :59 (how to run the
  test). Both still true; nothing documents the hang.

### Verification-round findings and their disposition (2026-11-17 04:39 – 05:43 EST, `date`)
- 04:39 started: the fresh verifier's three findings (all minor), each re-checked against the tree before anything
  is changed. Nothing committed, nothing pushed.
- **1. The README's rc 1 "a failure" (and "the page tests' one list") — REAL.** Re-checked: the runner returns RPV1's
  `exitCode` — 1 for a throw, a THROUGH or no tester record (`route_preview_run.hpp` :787), a refusal (:590, :900)
  or the no-pace stop (:830) — so a failed drive is rc 0 (the verifier's probe: "ROUTINE STOPPED" and "DONE, 1
  FAILED", both rc 0; the runner's own comment, `route_maker_runner.hpp` :759, was right). Fixed: the README's rc 0
  is "it ran — also when a step failed, was skipped or jumped, or the 57 s cut ended it (`word`, each page step's
  `verdict`)" and rc 1 "threw, went THROUGH a wall or a Loader, was refused and never ran, or hit the no-pace stop";
  the same in `engine_module.mjs`'s `preview()` comment and in `route_maker_main.cpp`'s exit codes (the same vague
  "the record says a problem"). The contract is now PINNED: SM1–SM3 assert rc 0 for the stop, the carry-on, the
  skip and the cut (mutant V1 below). Item 16(b)'s reason corrected in place (struck through). The warnings: the
  README and `allowed_warnings.mjs`'s header now say `engine.test.mjs` uses the list today, and `page.test.mjs` (its
  own empty list still) and A2u's test import it at A2u's merge.
- **2. One CI result hiding another — REAL, and wider than reported.** Any red step in a browser row skipped every
  later one: the browser WG3 hid the page, AND the Node WG3 step (also this run's, before `npm ci`) hid both browser
  steps. At the JOB level an engine failure in `route-builder` (the module's build, WG1/WG2/WG2s/WG7) skipped all
  eight rows — A1's model and page on every OS — and A1's model step, first in `route-builder`, hid the engine.
  Fixed at the root in `ci.yml`: each test step runs whatever another test did and waits only for its own inputs.
  `route-builder-browsers` runs `if: !cancelled()`; its downloads (`site`, `engine`), `npm ci` and the Playwright
  install run `!cancelled()`; the Node WG3 step needs the site and the engine (it reads the site's `robots.json`),
  the browser WG3 those plus npm and the browser, the page the site, npm and the browser — a test whose input is
  missing is SKIPPED, not failed. In `route-builder` the model's tests moved last (`!cancelled()`), Emscripten's
  install runs `!cancelled()` and the module's build needs only it; the engine's tests keep the default (they need
  the native program, the site and the module), and the engine artifact is uploaded only after they pass, so no row
  compares against a failing run's records. Checked here: the YAML parses and every condition reads as intended
  from the parsed file; GitHub's own evaluation is NOT provable on this laptop (coordinator list, 9).
- **3. Two handoffs only in the log — REAL.** A tripwire in `module.test.mjs`'s robot-one test: every preset of
  robots one and two, listed, must be `[]` — else "a real robot has levels now: add a golden on it that moves to a
  ledger level, so WG1 runs the export's facts::lv(i) (RMP1-PROGRESS, RMP1-A2e review item 5)". Any preset, not
  only a `ledger`-marked one, on purpose: a level on a real robot is what makes the golden owed. It runs in
  `route-builder` and in every row's Node step. Mutants V2 and V3 below. Both handoffs are in the list below.
- **Mutations** (`scratch/rmp1-a2e/fix2/mutate_v.py`; the runner backed up under `fix2/backup/`, restored in a
  `finally` and on SIGTERM/SIGHUP/SIGINT, mtime bumped; the module rebuilt FRESH before the test and again after the
  restore; 4 GiB cap): **V1** a failed page step made rc 1 (the old README's claim) — RED: SM1–SM3 ("a failed,
  skipped, jumped or cut step is rc 0") and WG1 (the compiled twin's rc differs). **V2** robot one's cascade given a
  ledger level, **V3** robot two's first mechanism given a plain preset (each in a copy of the built site's
  `robots.json`, read at test time through `ROUTE_MAKER_SITE` — nothing to build) — both RED with the tripwire's
  message. After: the runner equals its copy taken before the mutations; the module rebuilt (`preview.wasm` 535,355
  bytes, the one file 725,884 — unchanged).
- **A laptop lesson (not this run's code):** the suite's two ProsFileStore cases fail (3 checks) when `TMPDIR` is a
  long absolute path — the store keeps a path in `hal::kStorePathBytes` (96) bytes, and
  `<this worktree>/build/tmp/shulib_safesave_filestore/shulib_drivetrain.bin` is longer, so it is cut at 95. Seen
  here once (the suite and the briefing run with the Emscripten `TMPDIR` still exported): re-run without it, green;
  the briefing regenerated without it. The README's `TMPDIR=build/tmp` is for Emscripten's build only.
- **Re-run on the final tree:** host build 0 warnings (the briefing regenerated before it with the suite binary
  unlinked, and after); suite green **3,781** (`suite_run.py green`); compile refusals 295 of 295 (295 TUs); the
  module rebuilt from an empty `build/wasm` (0 warnings); the site rebuilt (`robots.json` unchanged); the engine's
  node tests **15 pass** (native set, records written); the other rows' form 12 pass, 3 skipped; WG3 in Playwright's
  Firefox 157 and Chromium, green; the model's 83 pass, 1 skipped; the ARM header gate exit 0 (258 headers); the
  PROS-free and sim-free greps clean; `api_doc_tool` self-test OK, check-coverage (6,439 entities, 214 headers),
  check-fresh exit 0, check-examples (919 lines), check-removability clean; briefing self-test OK and check
  current; staleness self-test OK and clean; `ci.yml` parses.
- **The src gate, run last under `timeout -s INT`** (04:50–05:42, beside three other worktrees' gates): self-test
  **OK (377 detector cases)**, check **PASS** (14 builds; robot one's 4 refuse, each naming exactly its owed facts).
  After it: `git diff -- Makefile src/ include/` empty, no "(self-test plant" anywhere, and `git ls-files -o
  --exclude-standard src include test` lists only this run's six new test files.
- **Prose review (rule 4), run BEFORE writing this:** `grep -rn -i` over `docs/ README.md test/README.md
  tools/route_maker/README.md --include=*.md`, the chunk files aside, for the alternatives: says a problem · rc 1 ·
  \`rc\` 0 · rc 0 · exit code(s) · allowed_warnings · allowed console · ALLOWED_WARNINGS · route-builder-browsers ·
  before the page · ledger level · !cancelled · tripwire · TMPDIR → this round's own lines (`tools/route_maker/README.md` :62, :77; `test/README.md`
  :95, :98), read again: true. Also: `tools/route_maker/README.md` :66 (`TMPDIR=build/tmp`, relative and short —
  true), :88 (a ledger level by its preset's name — true), :126 (CI names both jobs — true); `test/README.md` :69 (A1's
  suite tripwire on marked levels — a different tripwire, true); `docs/changelog.md` :198 (RPV1's exit codes, no
  meaning given — true). **One FALSE line, not this run's:** `docs/guide/16-testing-an-auton-on-the-bench.md` :485
  calls RPV1's exit 0 "drawn and nothing wrong", but a route whose drive fails also exits 0 there (the same
  `exitCode` rule) — a public doc of RPV1's, left for the coordinator (list, 10). The rest are other tools' exit codes
  or other tripwires.
- **Documents touched by this round:** `tools/route_maker/README.md` (rc 0 / rc 1, the warnings list),
  `test/README.md` (SM's rc 0, the tripwire), the comments of `engine_module.mjs`, `route_maker_main.cpp`,
  `allowed_warnings.mjs` and `ci.yml`, this log (item 16(b) corrected in place). Not touched: the public docs and the
  changelog (nothing a student sees changes), `page.test.mjs` (RMP1-B's), guide 16 (above).
- **For the coordinator — every handoff of this run, in one place:**
  1. The browser rows' page step is red on every row until RMP1-B's `page.test.mjs` fix (`ONE_LEVELS`) merges:
     merge it first, or rule on A.4's "every row green" reading this branch's engine steps alone (item 1).
  2. ROB1-BT2's merge: `route_maker_goldens.cpp` compiles `smoke_robot_two.cpp` under robot two's namespace as A1's
     golden test does; robot two's ledger in `robotSource("two")`; the module build and the engine's node tests in
     that merge's done list (item 3).
  3. **NEW — the ledger-level golden:** the run that types a real robot's levels (robot one's cascade, station 13 —
     a ROB1 or MECH-S run, which does not read this log) adds a golden on that robot that moves to a ledger level,
     so WG1 runs the export's `facts::lv(i)`. The tripwire turns `module.test.mjs` red the moment a level is typed;
     put this in that run's brief (item 5; this round's 3).
  4. **NEW — at A2u's merge:** `page.test.mjs` and A2u's PREVIEW test import `test/page/allowed_warnings.mjs`
     (exact sentences, Firefox's Worker wrapping handled) and drop their own lists (item 12).
  5. The two-robot PREVIEW's engine half and SM4: in no run's row — rule A2u or an engine follow-up (deltas 10, 11).
  6. A2u's site pieces: `build.py --engine`, `pages.yml`, PG1's `.wasm` check, RELEASING.md's "press PREVIEW once" —
     check they are in its args (delta 12).
  7. Files shared with RMP1-B (`route_maker_main.cpp` — a certain conflict; `route_maker_describe.hpp`; both READMEs;
     this log) and A2u (`tools/route_maker/README.md`, this log); whichever merges second rebuilds the module and runs
     the engine's tests (delta 13).
  8. WG2s: robot two's heading margin is ~4.8×, not the ~40× W2 assumed — the bounds hold; W2's remedy only on a flip.
  9. **NEW — CI's step conditions** are proven only on GitHub: on the pushed branch, read that every row shows both
     engine steps and the page step with a result; the first run with a red engine step must still show the page's.
  10. **NEW, not this run's:** guide 16 :485's "0 drawn and nothing wrong" (RPV1's tool) is false for a failed drive
      — for RPV1's owner or DOCS3.
  11. **NEW, a laptop lesson:** run the suite and `briefing_status.py generate` with no long `TMPDIR` exported (above).

## RMP1-A2e — the coordinator's merge *(2026-11-17)*

- **Source:** the `rmp1-a2e` worktree (base `e8fa3de`) as built, reviewed, fixed, verified and fixed again (workflow
  `wf_e3a4d5e3-045`, 7/7); transport `e87f24c`; squash-merged in `integrate` onto main `e568b9e` (ROB1-BT and RMP1-B
  had landed since). No file under `src/`, `include/` or the Makefile changed.
- **Conflicts (5):** `test/route_maker_main.cpp` (B's `feel | feel-edges` and A2e's `preview` both kept, in the header
  and in `usage()`); `route_maker_golden_test.cpp` (A2e's WG8 case + ROB1-BT's renamed smoke case); the route
  builder's README (B's Controls rows + A2e's engine rows); this log (both sections); the briefing (regenerated).
  A2e's handoff notes for ROB1-BT were already met at that merge (robot two's ledger in `robotSource("two")`;
  `smoke_robot_two` on robot two's real list — `route_maker_goldens.cpp` compiles it under `robot_two`).
- **Gates on the merged tree:** the suite green through `suite_run.py` (**3,853 cases, 3,885,990 assertions**); the
  module rebuilt from empty with the pinned emsdk 6.0.13 (`EM_FROZEN_CACHE=1`): 0 warnings, `preview.wasm` 539,903
  bytes; the engine tests (module vs the native twin: WG1–WG3, WG2s, WG7, WG8) **15/15**; the model tests 125 pass
  (16 skipped without the engine's variables); page tests **Firefox 32/32** (A2e's WG3 in the browser included),
  **Chromium 29 + 3 skipped**; `ctest` **353/353**; api fresh / examples / coverage / removability; the briefing
  current; staleness clean; check-nav / check-tone OK; `ci.yml` parses; route_preview self-test ok; the strict site
  build exit 0, no warning. The src gate and the ARM header gate are B's merge's (this merge changes nothing they read).
- **Mutation: MA-1 RED** — the merged `route_maker_main.cpp`'s `preview` dispatch renamed: the engine's native test
  fails 2 of 3; restored byte-identical, rebuilt, 3/3.
- **Open (A2e's own list, carried):** A2u imports `test/page/allowed_warnings.mjs` at its merge; the engine half of the
  two-robot PREVIEW (SM4) has no owner yet; the first run that types a cascade level adds a ledger-level golden (the
  module test's tripwire); guide 16 :485's "exit 0 = drawn and nothing wrong" is loose (a failed drive also exits 0).

## The top bar at laptop widths — the coordinator, after the RMP1-A2e merge *(2026-11-17)*

- **Found by CI** on `rmp1-merged` (run 44080274676): the two Linux Firefox rows failed "an error stands" — the
  disabled PREVIEW button lay over Export ("intercepts pointer events"). Windows rows and the local Firefox passed:
  the runner's fonts are wider. **Probed locally:** at 1440 px the bar was full to the pixel; at 1280 px (and 1101)
  Export was pushed off the screen (x 1261, width 34) — the bar needs ~1,430 px since the Controls tab joined it.
- **Fix (`style.css`, `app.mjs`):** the top bar's items keep their size (`flex-shrink: 0`); only the save-state chip
  and the route's name may shrink (cut with "…"); below 1440 px the folder button keeps its icon (its words become
  its `aria-label`) and the "Save downloads · Export copies" chip is not drawn; below 1280 px the "Route builder"
  words and the "Robot one" chip go too. Guide 22 points at both places to connect the folder.
- **Test (page.test.mjs, new):** at 1101, 1180, 1280, 1366, 1440, 1536 and 1920 px, on both tabs, PREVIEW and Export
  whole, on the screen and apart, the route's name whole, nothing running past the bar. **RED on the old page**
  (1101 px: Export at x 1261 in a 1101 px window), green with the fix. Firefox 32 + 1 skipped, Chromium 29 + 4
  skipped, model 125/125. Screenshots: `~/shulib-render-tmp/coord-probe/fix-*.png` (1101–1536, light and dark).
- **CI's second round (run 44085346344):** the new test caught the route's name cut on the runners (Chromium at
  1280 px: 105 > 88; Linux Firefox at 1536 px: 105 > 100 — the runners' text is ~7 % wider than this laptop's: the
  name is 105 px there, 98 here). Breakpoints widened: 1600 px (the folder words, the "Save downloads" chip) and
  1366 px (the "Route builder" words, the "Robot one" chip). **Probed** with the bar's text widened by letter-spacing:
  every width from 1101 to 1920 fits up to +16 % (+0.1 em); +22 % first cuts the name at 1601 px. And Windows Firefox
  failed two older tests on a browser warning, "127.0.0.1 has been classified as a bounce tracker" — Firefox's
  bounce-tracking protection reacting to the tests' own server, not the page: the three page-test launchers start
  Firefox with `privacy.bounceTrackingProtection.mode = 0`. Locally Firefox 33/33 (the engine's WG3 included),
  Chromium 30 + 3 skipped.
