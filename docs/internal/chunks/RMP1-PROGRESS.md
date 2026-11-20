# RMP1 — PROGRESS (live log)

<!-- release: between parts — RMP1-A1, RMP1-B, RMP1-A2e and RMP1-A2u (PREVIEW) landed 2026-11-16/17; RMP1-C (named routes, robot two on the field, 'both robots together?') is briefed (declared by the coordinator 2026-11-17) -->

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

## RMP1-A2u — PREVIEW in the page (the screen)

Executor: Claude Opus 5.5 (the coordinator's `opus` subagent), worktree `.claude/worktrees/rmp1-a2u` (branch
`rmp1-a2u`, from `1bcde4e`). Scope: A.7's A2u row — the Worker (progress, cancel), playback with the timeline
scrubber, the robot on the real field with its path beside the dashed plan, the step list lit from the record's
spans, the two-robot PREVIEW (robot two runs the mirror) with the 6 in band and the pair check's verdict, PLACEHOLDER
facts marked; guide 22's PREVIEW section. Built against a STAND-IN engine (recorded schema-1 records) behind
`engine.mjs`; RMP1-A2e (the real module) is building beside this run. Not committed, not pushed; host and browser
only — nothing here runs on a robot.

### Log
- 00:25 EST 11-17 — Created this section first. Read RESUMING's standing rules; the brief by the 10-22 ruling: the
  header, ANSWERS, A.1–A.12 in full, R1–R19, D1–D20, §1.1 and §1.4 (every A2/A2u row), §2.2–§2.5's A2 rows, §3, §6.1–§6.2,
  §8 (landmines 1–30) in full; the rest by heading. Read A2e's worktree (never edited): the runner's record writer
  (`test/route_maker_runner.hpp`), `engine_module.mjs`, `route_lines.mjs`, its README's schema-1 table and its log.
- **§2.5.1 the screenshot path, FIRST (A.5):** Playwright 1.64.1 (`npm ci` in `tools/route_maker`, cache and TMPDIR
  under `~/shulib-render-tmp/rmp1-a2u/`), its own Firefox 157 from the shared `PLAYWRIGHT_BROWSERS_PATH`, under
  `(ulimit -v 4194304; timeout …)` with `NODE_OPTIONS=--disable-wasm-trap-handler`: the built ONE FILE captured over
  `file://` (`shots/proof.png`) in seconds; `browser.close()` in a `finally`; no Playwright process left (`ps`).
- **A scratch tree for REAL fixtures** (`scratch/rmp1-a2u/tree` = `git archive 1bcde4e` + A2e's 22 changed and new
  files copied read-only at 00:26; none of them differs between A2e's base `e8fa3de` and `1bcde4e`): A2e's native
  `shulib_route_maker` built there (GCC, the suite's flags) — its `preview` writes the records the stand-in plays.
- **Fixtures recorded (00:37–00:40):** six routes on robot one (`ready`, `failed`, `carry`, `cut`, `pair_ok`,
  `pair_fail`) through A2e's native program; the two pairs' robot-two runs and verdicts through a SCRATCH prototype of
  the runner's pair mode (`scratch/rmp1-a2u/pair_probe.cpp`: the entry `competition::mirrored(…, kSkillsMirror)`
  through RPV1's `previewEntry` on robot two, and MIRROR1-A's `checkMirroredPair`). **A REAL FINDING for the
  coordinator (RPV1's file, not this run's): `route_preview_run.hpp`'s `plantStartFor` seats a MIRRORED entry's plant
  at robot ONE's start (`effectiveStart().pose()`), while the library re-seats the estimate at the reflected start —
  so robot two's true path is wrong (it drove from (-36, -60) toward mirrored targets). With `competition::seatedStart(e)`
  there (patched in a scratch copy only), robot two starts at (60, 36) facing 180° and every pace is robot one's
  reflected to 1e-6 (test `playback.test.mjs`). The pair check also segfaulted on the runner's routine outside
  `previewEntry` (its `state().stack` is null there) — the probe's scratch copy guards it.**
- **Built:** `src/model/playback.mjs` (the record → what the screen draws), `src/worker.mjs` (the Worker's loop),
  `src/host/engine.mjs` (the page's side: progress, Cancel, the 20 s stop, a new Worker after a failure),
  `src/host/engine_backend.mjs` (the real backend: A2e's `engine_module.mjs` + `route_lines.mjs`; inert until A2e
  lands), `src/ui/preview.mjs` (the controller, keys), `src/ui/timeline.mjs` (the dock and its scrubber); field
  layer, lit step cards; `build.py --engine DIR`. The stand-in: `test/standin_engine.mjs` (recorded runs through the
  page's own `worker.mjs`, injected by a test hook like A1's `useMemoryFolder`).
- **The seam PROVEN end to end (00:59–01:05):** a scratch overlay (my `tools/route_maker` + A2e's `engine_module.mjs`
  and `route_lines.mjs`, A2e's built module copied read-only, md5 `3c8e8ccf…` = its `build/wasm/preview.wasm`) built
  with `build.py --engine`: the site form (module Worker) and the ONE FILE (1.19 MB, blob Worker over `file://`)
  both run the REAL module through the page's own Worker in Firefox 157, and the record equals the native fixture —
  every field exact, poses within 1e-5 in / 1e-6 rad (WG2's bounds); 13/13 PREVIEW page tests green there.
- Gates so far: host build 0 warnings; suite green **3,778** (no C++ touched); briefing regenerated before and after
  (no change) and `check` current.
- **Gates (01:00–01:45):** src gate self-test **OK (377 cases)**, no plant left (`git status -- Makefile src/ include/`
  clean, no untracked file under `src include test`); ARM header gate exit 0 (259 lines); the PROS-free and sim-layering
  greps clean; api_doc_tool self-test / coverage / fresh / examples / removability, briefing self-test + check,
  staleness audit + self-test, doc_readability check-nav / check-tone — all clean.
- **A1's page tests on this base: 6 RED before this run touched anything** (the same 6 on a base-built site, logged in
  `scratch/rmp1-a2u/baseline_page.log`): `readyRoute` and three more pick robot one's cascade level `selectOption({index:
  3})`, and robot one's cascade has NO levels since ROB1-C2 (`robots.json`: `presets: []`). Not fixed here — RMP1-B's
  worktree carries the fix in `page.test.mjs` (`ONE_LEVELS`, `setClaw`), a file this run does not touch. With screenshots
  on, 2–3 more die of a Firefox `page.screenshot` protocol error after those timeouts — on the base too (19 red there).
- **Mutations (each applied alone, the site rebuilt for page tests, RED observed, restored in a `finally` and on
  SIGTERM/SIGHUP/SIGINT, mtime bumped; driver `scratch/rmp1-a2u/mutate.py`, backups beside it):** RM31 (one byte of
  the browser check's module) → the PREVIEW page tests RED; M1 the worker keeps its instance after rc ≠ 0; M2 the stop
  keeps the Worker; M3 Cancel keeps it; M4 poses not interpolated; M5 the library's lines one step off; M6 a jumped step
  called done; M7 a zero-length step "plays"; M8 a run never put away; M9 a refused route run; M10 robot two drawn from
  robot one's poses; M11 the band twice as wide; M14 an answer for another run taken; M15 cards not lit; M16 the run's key
  unsorted; M17 the pair mark at the wrong moment; M18 robot two never asked — all RED. M12 / M13 (build.py's
  `</script` and glue-export refusals) RED in the A2e overlay (their test needs A2e's modules). **Two SURVIVED the first
  round and were equivalent: M7 (`s.t1 > s.t0` was redundant beside `t < s.t1`) and M19 (the cache told one-robot and
  two-robot runs apart, but a page's baked robots never change) — both conditions deleted, M7 re-cut at `t < s.t1` → RED.**
- **Review (the code-review skill, high, its reviewers; then a FRESH verifier pass, medium) — findings and disposition:**
  1. A step seen from below on the pace two page steps share counted for both (PLACEHOLDER and failed sentences on the
     wrong card) — REAL → `ownersOf`: [first, last), a zero-length step owns ONE step of its family; tests + M20, M25,
     M26 RED. 2. A synchronous Worker throw left PREVIEW "running" forever — REAL → caught in `engine.mjs` and
     `preview.mjs`; test + M21 RED. 3. The 20 s stop counted the engine's download — REAL → the load gets 60 s
     (`LOAD_MS`), each robot's run 20 s; test + M24 RED. 4. ci.yml builds the site without `--engine`, so the REAL-engine
     page test is skipped in CI — REAL, A2e's ci.yml hunk → for the coordinator (below). 5. A new instance after rc 1 —
     NOT CHANGED: the brief (A.3.10) says after ANY non-zero return, and a failed step is rc 0 (the fixtures). 6.
     pages.yml needs A2e's files — by A.7's order (A2e merges first); for the coordinator. 7. Esc did not close the error
     dock — REAL → fixed; M22 RED. 8. Enter on a step card did not seek — REAL → fixed; M23 RED. 9. Robot two sized by
     robot one — REAL (cosmetic) → each robot by its own description (the record's `robot`). 10. Per-frame element
     building — REAL → nose points computed, the play icon swapped only on a change, the driven path appended forward
     (binary search; reset on a seek back: M29 RED). Verifier's new ones: the zero-length ownership (done above), the
     pair run's two 20 s stops (words now "a run did not finish in 20 s"), `capabilities().preview` disagreeing with the
     engine (one `previewAvailability`), build.py tracing back on an incomplete engine folder (refused by name; M30
     RED), the one file's engine kept twice (the element removed once its blob exists), `busy` unused (removed), the run
     layer keyed by the route (M27 SURVIVED: equivalent — every new run clears the layer; the run id dropped, the test
     kept).
- **03:10 EST 11-17 — RESUMED** (the 5-hour usage limit stopped the first run ~02:00). Checked first: `git diff --
  src/ Makefile` empty and no untracked file under `src include test` (no gate plant left); every mutation in
  `scratch/rmp1-a2u/mutate.py` absent from the tree (each `old` text present once, or the code rewritten since — M8,
  M19, M20, M27 — and none of their `new` texts present). Rebuilt from source (the briefing generated with the suite
  binary unlinked first): host build 0 warnings; suite green **3,778**; briefing regenerated after — no change;
  `check` current. Site rebuilt; the model 105 pass / 2 skipped; Firefox: PREVIEW 12 pass / 1 skipped (the
  real-engine test: no `--engine` here), A1's page tests the SAME 6 red (robot one's cascade has no levels).
  A2e's worktree re-read (read only): `engine_module.mjs` now documents rc 3 (the program failed, no record) and a
  THROW when the module aborts (that instance dead) — `worker.mjs` already handles both (no record → the words; a
  throw → a new instance); the record's schema 1 unchanged (`route_maker_runner.hpp` identical to the 00:26 copy).
- **The new order for this run (the coordinator, 11-17): FIX A1's 6 red page tests HERE, both halves** — (1) the
  page, honest for a cascade with NO levels: its menu offers home (and nothing it cannot do) and says "No levels yet:
  record them at Bench Tests station 13" instead of an empty list; (2) the page tests that move to a level run on a
  FIXTURE robot with levels (robot x, `test/goldens/route_maker/robots/x.json`), and a new test pins the no-levels
  wording on robot one. This supersedes the 01:00 entry's "not fixed here".
- **03:20–03:29 — the cascade with NO levels, both halves.** (1) THE PAGE: `checks.mjs` — RC18 on an arm or lift with
  no levels says "Robot one's cascade has no levels yet: record them at Bench Tests station 13." (`LEVELS_FROM`; the
  library's "no preset" sentence beside it when a level is named anyway — an older project file), never "Choose which
  level"; `words.mjs` — `mechanismActions(kind, levels)` drops "Move to a level" when there are none; `app.mjs` (RMP1-B's
  shell — 1 import name, 3 lines in the mechanism menu, 2 in the test hooks; logged below) — the menu says "No levels
  yet: record them at Bench Tests station 13." where the move was; `inspector.mjs` — no empty level list (the step's
  RC18 says why). (2) THE TESTS: `page.test.mjs` — the four tests that move to a level (an error stands; built by
  clicking; §2.5.2 R1 ×4 variants) put a FIXTURE robot in robot one's place first (`window.routeBuilder.useRobots`, new
  in `app.mjs`): `ledgerRobot()` from `test/helpers.mjs` — robot x's committed description with a cascade of
  facts-ledger levels, marked owed. **Delta from the order ("robot x"):** robot x's own lift has NAMED presets, so A1's
  "not measured yet" and `facts::lv(` assertions could not run on it; `ledgerRobot()` is robot x's description with
  robot one's cascade as it was (the node tests' own fixture, R8), so every A1 assertion still runs. `readyRoute` (the
  docs' pictures) stays on the REAL robot one: the claw set open in place of the level (RMP1-B's `setClaw`, the same
  helper, so the merge is small). NEW test `§2.5.2 robot one's cascade has NO levels` (×4 variants): the cascade's menu
  is exactly [the no-levels sentence, Home it, Wait until it is done], the claw's has no such sentence; a project from
  before (a move to level 2) opens with NO level list, ONE error — RC18 with the plain words — and Export lists it.
  NEW node test RC18 "no levels" (a move and a startMove with a level named; a home and a wait not refused; one level →
  "Choose which level"). Also: A1's one-file test now expects PREVIEW enabled on a site built with `--engine` (it said
  "comes next" unconditionally — it would have gone red at A2e's merge); `app.mjs`'s re-bake clears its cached checks
  (`derived = null` — the hook changed the robots under a cached route).
  **Result: A1's page tests 17/17 green in Firefox (were 6 red), with screenshots on 32/32 + PREVIEW's 15/16 (1
  skipped: the real engine), 0 page errors.** The model 106 pass / 2 skipped.
- **Docs for it (rule 3):** guide 22 step 5 "Work a mechanism" (the claw; the cascade's no-levels note; the home rule
  kept for when it has levels); the intro, step 3's pill sentence, the code reading, the reference's level note and
  the checks table; the changelog entry's "What changed"; `RMP1-A1-manual-checks.md` (steps 3, 4, 7 and the card: the
  claw, PREVIEW, offline PREVIEW); `tools/route_maker/README.md` (the levels robot). **Pictures re-taken** (their step
  4 was "cascade: to level 2"): `route.png`, `route-dark.png`, `saved.png`, `export.png`, `turn-first.png`,
  `curve-refused.png`; NEW `mechanism-menu.png`; `home-first.png` DELETED (robot one cannot reach that screen; the
  rule is described in words). 1200 × 750, 256 colours (`scratch/rmp1-a2u/to_docs.py`: byte-identical method to the
  first round's — `preview.png` re-made from its source is the same 176,875 bytes).
- **03:30–03:50 — mutations on the FINAL code (every dependent rebuilt: the site re-made for each page mutation; driver
  `scratch/rmp1-a2u/mutate.py`, logs `mutlogs_final/`, results `mutations_final*.json`):** RM31, M1–M11, M14–M18,
  M20–M26, M29, M30, RM13 (NEW: PREVIEW runs a remembered route, not the one on screen — the brief's amended RM13 →
  the page tests red), N1–N5 (NEW, the no-levels half: the menu offers a move with no levels; its sentence dropped; its
  sentence on every mechanism; the empty level list back; RC18's no-levels branch dropped), M32–M34 (NEW, below) —
  **every one RED**. M12, M13, M33, M34 also in the overlay with A2e's modules (RED). M8 and M20 re-cut to the
  rewritten code; M19 and M27 dropped (their code is gone, as logged above). **A DRIVER FAULT, found and fixed:** a
  mutation whose command runs two test files (`a; r=$?; b || r=1; exit $r`) went through a second `bash -c` inside
  double quotes, so `$?` and `$r` were expanded by the OUTER shell and the exit was 0 — M5, M6 and N5 printed "GREEN"
  beside their own "not ok" lines. Now ONE shell (`timeout 900 bash -c <cmd>`); the three re-run: RED.
- **Playwright's CHROMIUM here (under `systemd-run … MemoryMax=4G`), the first Chromium run of PREVIEW — TWO REAL
  FINDINGS, both would have been red on the Windows Edge / Chrome rows:** (1) **Chromium refuses a MODULE Worker made
  from a `blob:` URL on a page opened from disk** (`file://`, the one file; a probe — `probe/workers.html` — shows a
  classic blob Worker answering in both browsers, a module one only in Firefox): the ONE FILE's PREVIEW said "the
  preview engine did not load". **Fixed:** the engine's Worker is CLASSIC in both forms (`engine.mjs`), and `build.py`
  makes the module's glue a classic script — `import.meta.url` → `self.location.href` (in a Worker, its own script's
  URL: what `import.meta.url` is in a module Worker), any other `import.meta` refused by name. This is the brief's
  A.10 open item ("a Worker from a blob: URL in the one file on file:// … Edge and Chrome still open") — answered for
  Chromium, and needing no fallback to the page's own thread. Tests: `engine.test.mjs` pins the classic Worker (M32),
  `build_engine.test.mjs` the rewrite and the refusal (M33, M34). (2) **A test race:** `openFixture` waited for the
  route's STEP COUNT, and `pair_fail` and `pair_ok` both have 3 steps — in Chromium PREVIEW was pressed before the
  file was read, so it played the previous route. Now it waits for the route itself.
  **Chromium after the fixes: 27 pass / 3 skipped** (the real engine; two Firefox-only tests). Firefox: 29 / 1.
- **The seam, re-proven with A2e's CURRENT module (copied read-only at 03:29: `preview.wasm` md5 `483d0232…`, its
  `engine_module.mjs`, `route_lines.mjs`; overlay `scratch/rmp1-a2u/integ2/`, site `integ2_site/`, the one file 1.20 MB):
  Firefox 30/30 and Chromium 28/28 (+2 Firefox-only skipped) — THE REAL ENGINE in the SITE form and the ONE FILE over
  file://, the record equal to the native fixture (every field exact; poses within 1e-5 in / 1e-6 rad), A1's tests
  with PREVIEW enabled.** (The overlay's three `library_words` node tests fail there only because the overlay has no
  `include/`; in the worktree they pass.)
- **04:15–04:30 — REVIEW ROUND 2 (the code-review skill, high, its reviewers, on this worktree's whole diff).** (A first
  invocation reviewed the MAIN tree's branch, not this worktree — its ten findings are about `shulib-v2`'s calibrate,
  Toggle and pair-check code, NOT this run's; not verified or acted on here; for the coordinator.) Findings and
  disposition, each checked against the code:
  1. `pages.yml` always builds `--engine`: a publish before A2e merges fails — REAL (the order "A2e first" was only in
     the log; the coordinator may merge A2u first) → the engine is built and PG1's engine check run only when
     `tools/route_maker/wasm/CMakeLists.txt` exists; both branches of the snippet run here.
  2. CI never runs the real module through the page — REAL, but A2e's `ci.yml` hunk: its `route-builder` job builds
     the site BEFORE the module → for the coordinator (the seam below).
  3. The browser check still demands MODULE Workers — REAL since the classic Worker → `browser_check.js` probes a
     classic blob Worker (`worker`), its comment says so; NEW page test WG4 [W16] (a browser whose `Worker` throws is
     told, the builder opens) + M35 RED. (W16's "module Workers" premise changed with the classic Worker.)
  4. `LEVELS_FROM` names Bench Tests station 13 for every robot — NOT CHANGED: station 13 MECHANISMS is Bench Tests'
     own station for every robot's mechanisms (MECH-A6), and the wording is the coordinator's order.
  5. `poseAt` blends headings without wrapping — NOT A DEFECT: schema 1's samples are UNWRAPPED (README), measured on
     every fixture: largest pace-to-pace change 0.06 rad, robot two's from 3.142 to 3.926 without a jump.
  6. A pair verdict `checked: false` and `ok: false` shown as "too close" — CANNOT HAPPEN: `PairVerdict::ok` is true
     unless a checked comparison found the gap (`test/mirrored_pair_check.hpp` :305, :323).
  7. `worker.onerror` stops the Worker with no run pending — KEPT: an error the Worker did not catch leaves it in an
     unknown state; the next PREVIEW makes a new one (A.3.10's "a new Worker after any failure").
  8. Guide 22 called "About this run" the ⓘ button; it is drawn as ? — REAL → fixed.
  9. `field_view.mjs` repeats the corner-to-points code — REAL (simplification) → `square()` uses `cornersOf`.
  10. The no-levels rule in four places — REAL → ONE predicate `noLevels(m)` (`checks.mjs`) used by RC18 and the menu.
  Mutations re-cut and re-run on the reviewed code: N1–N6 (N6 NEW: `noLevels` ignores the kind → the claw gets the
  sentence), M35, RM31 (its file changed), M10, M11, M29 (`field_view.mjs` changed) — all RED. Model 106 pass / 2
  skipped; Firefox 30 pass / 1 skipped.
- **04:30–04:45 — A FRESH VERIFIER (the code-review skill, medium, a new run on the whole diff).** Findings and
  disposition: (1) `poseAt`'s heading wrap — NOT A DEFECT (the samples are unwrapped: above). (2) **A record with NO
  paces crashed the frame** (`poseAt` on an empty list) — REAL: RPV1's `refusedModel` writes exactly that when the
  registry refuses the entry (`samples: []`, `word: REFUSED`, `why`) → `preview.mjs` says "The library refused this
  route before the robot moved (REFUSED): <the library's sentence>" and plays nothing; a robot-two record with no
  paces drops robot two with its sentence. NEW stand-in option `refuse` (a record shaped as `refusedModel` writes it)
  and page test; M36 RED. (3) The cache key covers labels and ignores re-baked robots — NOT CHANGED: a label changes
  where a skip lands (a different run), and the robots change only through the tests' hooks (a page's baked robots
  never do). (4) **After robot one's run returned non-zero, a pair request re-made the whole module just to find the
  engine has no pair mode** — REAL (a ~1 MB reload and a "Starting…" for nothing; today's backend has no pair) → the
  pair mode is asked of the instance that ran robot one; worker test + M38 RED. (5) `pages.yml` builds the module
  without a build type → assertion paths and URLs — NOT A DEFECT: A2e's `CMakeLists.txt` defaults to Release with
  `-ffile-prefix-map`, and the overlay site built that way passed build.py's RG1 and URL checks. (6) "as ci.yml builds
  it" — the comment now names A2e's job. (7) Module-only code outside the glue's function (top-level await) would
  pass build.py — REAL as a guard (A2e's glue has none: its one `await import("node:module")` is inside
  `createPreview`) → refused by name unless the glue begins with `async function createPreview(`; M39 RED. (8) **The
  dock's height was measured only when it redrew: a narrower window wraps its rows and the field went under it** —
  REAL (measured: 157 px at 1440, 138 at 1100, 215 at 700) → a `ResizeObserver` keeps `--dock-h`; asserted in the
  failed-step test at 700 px; M37 RED. (9) Robot one taken by position (`routeRobots(...)[0]`): for a hand-edited
  project naming a robot this build lacks, robot two was sent as robot one — REAL → robot one is the page's own
  `app.robot()` (the one its checks ran on), robot two the other one; M18 re-cut, RED. *No test reaches the
  unknown-robot case (the stand-in keys runs by route, robot included) — the fix is by reading.* (10) The one file's
  engine element removed after the first run left `engineSource()` empty for any later caller — REAL (latent: only
  startup calls it today) → the element stays (~1 MB of text twice in memory, which is nothing beside a robot's
  correctness).
  **After the fixes:** model 106 pass / 2 skipped; Firefox 31 pass / 1 skipped; Chromium 29 pass / 3 skipped; the
  overlay on A2e's module Firefox 32/32, Chromium 30 + 2 Firefox-only skipped (THE REAL ENGINE in both forms in both
  browsers); mutations M1, M18, M30, M33, M34, M36–M39 RED on the final code.
- **04:45–05:00 — the WHOLE-brief pass (no reviewer subagent can be started from this run: the coordinator's
  "one reviewer reads the WHOLE brief" was done here, by reading D5, §1.4's states and A.3.10 in full against the
  page).** ONE GAP FOUND: §1.4's "a preview that … went THROUGH a wall … (RPV1's sentences, the step marked)" — the
  page listed a field note only in "About this run", in RPV1's below-numbering. **A REAL THROUGH, recorded:** a route
  the page ACCEPTS (no check refuses it) whose real run goes through the right wall — start (-36, -36) facing 0, a
  curve through (30, -36), (60.5, -10), (60.5, 30): the follower swings 1.01 in past the wall's face at 2.28 s (found
  by `scratch/rmp1-a2u/probe/find_through.mjs` over 20 routes — every other THROUGH it found, the page refused
  first; recorded by A2e's native program, `make_fixture_through.mjs` → `test/fixtures/preview/through.preview.json`,
  rc 1). Now `playback.mjs` marks the page step whose span holds the THROUGH pace red — "Went THROUGH the right wall",
  and the line "On the robot it hits the right wall at 2.28 s (1.01 in past its face, more than the 1 in allowed): the
  host test plant has no walls." (a TOUCH or a piece marks nothing, as RPV1). Node test (the real fixture; a TOUCH;
  the shared pace's owner) + page test; M40 (not marked), M41 (a TOUCH taken for one), M42 (the shared pace to the
  earlier step) RED. Guide 22 and the changelog say it. New screens: `preview-through`, `preview-refused`.
  **Final runs: model 107 pass / 2 skipped; Firefox 32 pass / 1 skipped (PREVIEW in all four variants with
  screenshots: 18 pass / 1 skipped); Chromium 30 pass / 3 skipped.**
- **PROSE REVIEW (rule 4), run BEFORE this entry was written — the exact patterns:** (1) `grep -rn -i -E
  'engine\.mjs|engine_backend|worker\.mjs|playback\.mjs|preview\.mjs|timeline\.mjs|field_view|step_list|inspector\.mjs|words\.mjs|checks\.mjs|capabilities\.mjs|build\.py|app\.html|help\.mjs|standin|stand-in engine|PREVIEW|Move to a level|no levels|station 13|module Worker|classic Worker|import\.meta|pages\.yml'
  docs/ README.md --include=*.md`, the chunk files and `docs/api/` aside → 31 files; the route-builder ones read in
  full (guide 22, guide 14 :67 and :558–564, guide 16 :358–363, `docs/route-builder.md`, RELEASING :143–160, the
  changelog's entry); the rest are RPV1's route preview (`tools/route_preview.py`), station 13's facts form and the
  library's own uses of the words — none says anything about the route builder. (2) `grep -rn -E 'PREVIEW|route
  builder|route-builder|Route builder' docs/ README.md --include=*.md` (same exclusions) → guide 16 :362, guide 14
  :67 / :558–564, `docs/route-builder.md`, RELEASING :143–155, the guide index — all current. (3) `grep -rn -i "comes
  next|not yet in the route builder|no PREVIEW|PREVIEW (playing"` → only guide 22's "Not yet in the route builder"
  list, which no longer names PREVIEW. (4) `grep -rn "home-first"` → none left (the picture is deleted). `docs/route-builder.md`
  :36–38 ("their mechanisms, their levels, how each one homes") stays true. **Fixed by this review:** guide 22's step 5
  and the ⓘ/? button (above), `RMP1-A1-manual-checks.md` (steps 3, 4, 7 and the card). **Deliberately NOT touched:**
  `roadmap.md` / `build-order.md` (status pointers are the coordinator's at the merge), A1's changelog entry (history),
  `docs/api/` (no `///` comment changed: generated docs unchanged, `check-fresh` exit 0).
- **FINAL GATES (05:00–05:20, on the final tree):** host build 0 warnings (no C++ changed); suite green **3,778**
  (`suite_run.py green`); briefing `check` current (regenerated before and after the build at 03:10 — no change);
  src gate self-test **OK (377 detector cases)** (a first try under `timeout -s INT 1500` was cut at 25 min while
  rob1-bt2's self-test shared the CPU — interrupted inside a real build, nothing planted: `git diff -- src/ Makefile
  include/` empty, no untracked file under `src include test`; re-run under 7000 s, OK) and `check` **PASS (14
  builds)**; ARM header gate exit 0 (259 lines); PROS-free and sim-free greps clean; api_doc_tool self-test /
  coverage (6,439 entities, 214 headers) / fresh / examples / removability; briefing self-test; staleness self-test
  and audit clean; doc_readability check-nav (260 pages) / check-tone / self-test; the site: no URL but the XML
  namespaces, `app.html` and no `index.html`, RG1 clean; the new fixture `eol: lf`.
- **Lines in files other in-flight runs own (for the coordinator's merge):** `app.mjs` (RMP1-B's shell): the header
  comment; 2 imports (`createPreview`; `LEVELS_FROM`, `noLevels`); 3 lines in the Mechanism menu (`mechanismActions(m.kind,
  !noLevels(m))` and the no-levels sentence); `app.refresh` + `app.preview` (2 lines); the PREVIEW button →
  `app.preview.topbar()`; `app.preview.sync()` at the end of `render()`; the test hooks (3 lines: `rebake`, `useRobots`).
  `style.css`: 78 added lines (the dock, the run layer, the lit cards), nothing changed. `page.test.mjs` (A1's file;
  RMP1-B carries ITS OWN fix for the same red tests — `ONE_LEVELS` conditionals and the same `setClaw`): this run's fix
  is the coordinator's order (the level tests on the fixture robot, robot one's no-levels test) — take this one and
  drop B's `ONE_LEVELS` branches; B's other hunk there (the time limit taking focus at once) is independent.
  `files.mjs`: not touched.
- **THE SEAM for the coordinator at A2e's merge:** (1) `ci.yml`'s `route-builder` job builds and uploads the site
  BEFORE A2e's module steps: move the site's `build.py` call after "The preview engine" step and add `--engine
  build/wasm`, so every browser row runs the REAL engine through the page (`preview.test.mjs`'s "THE REAL ENGINE",
  site + one file; A1's one-file test then expects PREVIEW on). (2) Nothing else: `engine_backend.mjs` imports A2e's
  `engine_module.mjs` and `route_lines.mjs` by their paths; `pages.yml` builds the engine whenever
  `tools/route_maker/wasm/` exists. (3) When the runner gains its pair mode, the backend's `pair(route, robots)` is the
  one place to add (README "PREVIEW").
- **For the coordinator, not this run's files:** RPV1's `plantStartFor` seats a MIRRORED entry's plant at robot ONE's
  start (00:37 entry); the pair check segfaults outside `previewEntry` (same entry); the misdirected first review's ten
  findings on `shulib-v2` (04:15 entry). Not committed, not pushed; host and browser only.

### RMP1-A2u review findings and their disposition (2026-11-17 from 05:40 EST, `date`)
- 05:40 — Started. Three reviewers' 24 findings (correctness 7, tests 8, docs 9; overlaps deduped below). Each is
  re-checked against the code before any change; dispositions follow as they are made.
- 05:45–06:05 — **The model and the screen (C2–C6, T2–T6):** each finding re-checked in the code first; all REAL.
  - **C3 `ownersOf` guessed same-pace mechanism steps** — REAL, and reproduced on a REAL run: NEW fixture `two_starts`
    (`startRun rightToggle` then `startHome cascade` on one pace; RPV1 lists cascade first, by slot): the old code put
    the cascade home's PLACEHOLDER on the toggle's card. Now a mechanism step goes BY NAME (`playbackOf(result, {one,
    two})`: each page step's mechanism `.name` from the robot's description, `preview.mjs`), only to a page step naming
    it; two starting one mechanism on one pace take nothing (and never fall through to a drive's span — a case the
    first cut got wrong, caught by its own test). `MOTION_KINDS` / `MECHANISM_KINDS` now imported from `route.mjs`.
  - **C2 robot two's own result never shown** — REAL → `playbackOf` keeps robot two's verdict on a page step where it
    went otherwise and not done (`step.two`); the headline is the worse of the two with both words when they differ;
    the card says "Robot two: …" with its line; the dock's line says it; About this run has a robot-two part (owed
    facts, field notes, its full log). Page test where only robot two fails (stand-in option `two`).
  - **C6 after a THROUGH** — REAL → the steps after it are grey "After the THROUGH: the robot would not get here"
    (RPV1's own idea, `route_preview_render.hpp` :383), the run is drawn faded after `throughAt` (`g-run-after`, the
    robot at 40 %), the headline reads as RPV1's ("FINISHED, but THROUGH the right wall"), a THROUGH in the park is
    said in the dock; the "1 in allowed" number dropped (the record does not carry it).
  - **C5 RPV1's from-below step number in About this run** — REAL → `noteWords(f)` names the PAGE's step (`f.at`,
    the span holding the pace), or "after the last step"; never `f.step`.
  - **C4 Left passed over a step under 50 ms** — REAL → the 50 ms "previous" grace only while PLAYING; paused, every
    start is a stop; a span is at least 3 px wide (`max(3px, …)`).
  - **T3 threw / looping** — a REAL `threw` run recorded (NEW fixture `threw`: a wait on a mechanism never started —
    the page refuses it first, RC15, so it is a node test); looping in the runner's shape from a real record.
  - **T2 the cache test** — REAL → the stand-in counts RUNS (`standinRun`), asserted 1 after the undo and PREVIEW.
  - **T5** → two worker tests (robot two's rc 1 → a new instance; robot two after robot one's rc 1 runs on a NEW one).
  - **T1, T4, T6** → page tests for the drag (pauses, follows, Space plays on), the speeds (a frame through the new
    `frameAfter` hook: 50 ms = 0.05 × speed s), every span's state and place, the playhead and time, the budget bar,
    the pair mark's place, the driven / whole path against the record's thinned paces, the front's tip, the
    too-close squares at the verdict's poses, "Done in t1 − t0", Space from the end, Space held for a pan, a stale run
    pausing, the dock's PLACEHOLDER line, Esc cancelling, arrows in a box, a never-reached step's card, reduced
    motion, a schema-2 record, robot two refused, no steps, the error's step selected from another.
  - **TWO NEW UX DEFECTS the new tests found:** (1) the dock's foot row did not wrap, so robot two's long headline
    squeezed the step's sentence to one letter a line — the dock grew to 2,569 px (probe
    `scratch/rmp1-a2u/probe2/unstable.mjs`) → the row wraps, the sentence keeps 260 px; (2) **the scrubber moved
    while the run played**: the sentence under it changed height with each step's words → it is two lines, always
    (the whole sentence on hover) — pinned by the track's box at a long and a short sentence.
  - **D9's file name** — `test/engine.test.mjs` renamed `test/preview_worker.test.mjs` (A2e adds
    `test/page/engine.test.mjs`).
  - **T8 the recorder** — committed: `tools/route_maker/test/fixtures/record.mjs` (the routes, the command; it reads
    A2e's `route_lines.mjs`, so it runs where A2e has landed — run here in the scratch tree with A2e's native program:
    `ready` re-recorded BYTE-EQUAL to the committed one). Every fixture carries `madeBy`; the two pair fixtures say
    the PROTOTYPE and RPV1's patched `plantStartFor`. A node test pins each fixture's route to the recorder's.
  - Firefox: PREVIEW 19 pass / 1 skipped (the real engine), A1's 17/17; node 114 pass / 2 skipped.
- 06:10–06:25 — **C1 + D1 (one finding, two lenses): the two-robot PREVIEW cannot happen with the real engine** —
  REAL. Re-checked: `engine_backend.mjs` returns only `preview`; A2e's runner has no pair mode (grep `pair` in its
  `route_maker_runner.hpp` and `engine_module.mjs`: none); no run in A.7 owns one. So the page's line, guide 22, the
  changelog and guide 14 promised what robot two's update alone cannot deliver. **Fixed:** the page says "PREVIEW
  plays robot one: robot two's mirrored run is not in this build of the route builder yet." (the worker: "…not in
  this preview engine yet."); guide 22's Robot two paragraph is the plain fact plus the rule that keeps the robots
  apart (guide 17's "Both robots at once" — the route preview command was NOT pointed to: it seats a mirrored entry
  at robot one's start, the 00:37 finding); the changelog's "Both robots" bullet is gone (one sentence in the
  PREVIEW bullet); guide 14's route-builder limits name it first. **`docs/assets/route-builder/preview-pair.png`
  DELETED** (it showed a prototype's run: landmine 1, §3's "every step a real screenshot"); the pair screen's
  pictures stay as the run's evidence only (`~/shulib-render-tmp/rmp1-a2u/shots-review/`). The README now fixes
  the pair mode's CONTRACT — `pair(route, robots) -> {rc, record, verdict}`, robot two's record from its seated
  start, and the verdict's JSON members the page reads (`checked`, `ok`, `gapIn`, `leastGapIn`, `leastGapAt`,
  `failAt`, `failGapIn`, `one`, `two`, `sentence` — `PairVerdict`'s own names, `test/mirrored_pair_check.hpp`
  :303-317). **For the coordinator: A2u's two-robot row is met against the STAND-IN only. The runner's pair mode
  needs an owner and a go, with RPV1's `plantStartFor` → `seatedStart` fix and the pair check's guard outside
  `previewEntry` (00:37 entry).**
- **C7 the seam unproven on Windows** — REAL (the seam is the coordinator's at A2e's merge). Made SELF-ENFORCING
  here: in CI (`CI` set), a tree that has `tools/route_maker/wasm/CMakeLists.txt` must have built its site with
  `--engine` — THE REAL ENGINE test FAILS instead of skipping (proven: a planted `wasm/CMakeLists.txt` in the scratch
  overlay with an engine-less site → "this tree has the engine … but the site was built without it"; without `CI`,
  skipped). So the merge that brings A2e and A2u together cannot leave every browser row silently skipping it.
  Still for the coordinator: the `ci.yml` move (the site's `build.py` after "The preview engine" step, `--engine
  build/wasm`), `preview.test.mjs` importing A2e's `test/page/allowed_warnings.mjs` (A2e's README says it at A2u's
  merge; it is not in this tree), and confirming THE REAL ENGINE ran on the windows-latest msedge / chrome / firefox
  rows.
- **T7 the REAL test red on an API bump** — REAL → it compares with the native twin's record MADE AT TEST TIME when
  the program is there (`ROUTE_MAKER_NATIVE`, or `build/test/shulib_route_maker`; A2e's `route_lines.mjs` writes its
  lines), else with the fixture less `api`, `owed`, `robotLine`, `ledgerFile`, `notices`. Proven in a fresh overlay
  (`scratch/rmp1-a2u/integ3/`: this tree + A2e's CURRENT `engine_module.mjs` / `route_lines.mjs`, its built module
  md5 `483d0232…`, copied 06:15): native → pass; fixture → pass; a native twin with ONE member changed
  (`probe2/native_wrong.sh`, park 57 → 58) → RED "site: park".
- **D8 the docs true only if A2e merges first** — REAL. Round 2 had made `pages.yml` publish WITHOUT the engine when
  `tools/route_maker/wasm/` is absent; that branch is reachable only in the order A.7 forbids, and it would publish
  docs that say PREVIEW works on a page that says "comes next". **Reversed, at the root:** a first step "PREVIEW's
  engine is in this commit" refuses by name ("merge RMP1-A2e first"), the engine is always built, PG1 always checks
  both forms. Both outcomes run here by shell (this worktree: the error, rc 1; A2e's: present; PG1 on the overlay's
  site: ok; on this worktree's engine-less site: "engine missing", rc 1). `pages.yml` runs only on `main`, so no
  branch CI is affected.
- **D2 Chromium on this laptop** — REAL as a process point: the accepted answer is D18 (a) (Chromium in CI only), §5
  lists "Chromium on this laptop" as OUT, and D18 (b) (a `systemd-run … MemoryMax=4G` cap) needs the team lead's OK.
  `grep -n "MemoryMax\|systemd-run\|D18 (b)" .claude/coordinator/NOTES.md` (run 06:40, read only) finds ONE line —
  :1110, the coordinator's own RMP1-B merge (11-17 ~05:52: "Chromium needs systemd-run -p MemoryMax, not ulimit
  -v") — and no record of the team lead's OK for D18 (b). This run's earlier Chromium numbers
  (03:30–04:45 entries) are therefore OUTSIDE the accepted D18 — the evidence for Chromium, Edge and Chrome stays the
  CI rows; the one real defect those runs found (a module Worker from a `blob:` URL on `file://`) is fixed and pinned
  by node tests (M32–M34) that need no Chromium. **No Chromium ran in this round.** For the coordinator: A1's
  README (`tools/route_maker/README.md` :36-38, kept by this run) documents the `systemd-run` practice — record
  D18 (b) with the team lead's OK, or that line goes.
- **D3 two §1.4 / D5 items not logged** — REAL. (1) "open the full report" (D5): there is no RPV1 page in the browser
  (the record carries RPV1's content); **built here as far as the record goes**: About this run now lists, for each
  robot, its owed facts, its notices, its field notes in the page's step numbers, and EVERY line the library printed
  (a closed `<details>`, "Every line the library printed for robot one (N)"). RPV1's table and its drawn report are
  not in the page — a DELTA for the coordinator. (2) "the engine failed to load — PREVIEW greyed with the reason"
  (§1.4): KEPT ENABLED, with the reason in the dock and "press PREVIEW to try again" — a DELTA, logged with its
  reason: on the site form a load can fail for a dropped connection, which a second press fixes; greyed, only a
  reload would. A one file whose engine is damaged says so every press. For the coordinator to acknowledge.
- **D4 §2.5.3 in the dock** — REAL → the speed buttons are `var(--hit)` tall with hover help ("Play at half speed",
  … — asserted); the pair mark is a 32 px target with its 14 px dot drawn inside; the 10 px and 11 px sizes are
  `var(--t-xs)`; the menu note's inline `padding:0 10px` is `.menu-note` in `style.css` (`app.mjs`, RMP1-B's shell:
  that one line changed — logged for the merge). Found while capturing the new states: **the toasts sat on the dock's
  words** (the "Opened the project." toast over "Running the route…") → the toasts rise above the dock
  (`--dock-room`), asserted in the progress test.
- **D5 the word list** — REAL (not logged) → run now, BEFORE this line: `git diff 1bcde4e -U0` of guide 22, guide 14,
  guide 16, the changelog, `route-builder.md`, the README and `src/`, plus every new `src/` file, `+` lines only,
  through `grep -i -E 'simulat|honest|obviously|just |simply|easy|precondition|radian|https?://'` → ONE hit,
  "just behind it" in a `playback.mjs` comment → reworded ("less than 50 ms behind it"); re-run: none.
- **D6 screenshots of most states missing** — REAL → a second §2.5.2 test per variant captures running (progress,
  Cancel), stopped at 20 s, the engine that did not load, refused, a THROUGH (its step, then the end, faded), the
  one-robot line, the stale dock and the too-old browser — light and dark, 1440 and 1024. At 1024 the step list is a
  drawer: the test opens it (and closes it again before using the dock).
- **D7 the usability card and A.4 step 3 changed** — REAL as an acceptance delta: robot one's cascade has no levels
  (ROB1-C2), so `RMP1-A1-manual-checks.md`'s card and step 3 use the claw; the card therefore no longer meets RC25's
  "home it first" unaided. **A DELTA to §2.5.4 and A.4 for the coordinator to acknowledge**; the cascade card comes
  back when station 13 records levels (the manual-checks file says so). Robot one has no other arm or lift to carry
  a home-first check today.
- **D9 overlaps with A2e** — REAL → this run's `test/engine.test.mjs` is renamed `test/preview_worker.test.mjs`
  (A2e adds `test/page/engine.test.mjs`); the README conflict and the warning list are added to the merge list below.
- **For the coordinator's merge, added by this round** (beside the 05:00 list): `app.mjs` (RMP1-B's shell) — the
  menu note's inline style became `class: 'hint menu-note'` (one line); `style.css` — this round's lines are in the
  PREVIEW block only, plus `.toasts`' `bottom` (`calc(52px + var(--dock-room, 0px))`) and `.step-run`'s wrap;
  `tools/route_maker/README.md` — A2e ALSO changes it (+72/-2): A2u's "PREVIEW in the page" section sits after A1's
  laptop paragraph, A2e's after the module's build — keep both, and A2e's schema-1 table is the record's (A2u's names
  the pair verdict only); `test/page/preview.test.mjs` keeps a private `ALLOWED_WARNINGS` until A2e's
  `test/page/allowed_warnings.mjs` is in the tree — import it at the merge (A2e's README says so); `docs/guide/22`
  — the coordinator's NOTES (:1110, read only) say RMP1-B's merge into `integrate` made guide 22's "until robot two's
  update" lines present-tense: this round REWROTE the PREVIEW section's Robot two paragraph (robot two is not in
  PREVIEW, whatever the build) — take this round's paragraph. Robot two's update being in `integrate` means the page
  there ASKS for the pair, and the worker's line ("…not in this preview engine yet") is what it shows.
- 06:45–07:35 — **MUTATIONS on this round's code** (driver `scratch/rmp1-a2u/mutate_round3.py`: ONE exact replacement,
  the site rebuilt for every page mutation, its test under the 4 GiB cap and `timeout 900`, restored in a `finally`
  and on SIGTERM/SIGHUP/SIGINT, mtime bumped, restoration asserted; backups `scratch/rmp1-a2u/backups_r3/`, logs
  `mutlogs_r3/`, results `mutations_round3*.json`; every RED's failing test read — each the one aimed at). **86 cut:**
  the reviewers' R1–R32 survivors re-cut to the fixed code; this round's own (C2a–d, C3a–d, C4a–c, C5, C6a–d, T2,
  UX1–UX3); and EVERY earlier mutation on a file this round changed, re-run on the final code (M1, M4–M11, M15–M18,
  M22, M23, RM13, M36, M38, M42, N2, N3), the seven whose code was rewritten re-cut to it (M20, M25, M26, M29, M37,
  M40, M41). **First pass: 73 RED, 6 SURVIVED, 7 not applicable (their re-cuts ran).** The survivors, each re-checked:
  - **R15 (the nose never drawn) and R16 (the faint whole path empty) — a TEST FAULT:** `near()` walked the DRAWN
    points, so an empty polyline compared nothing. Now it asserts the counts first → both RED.
  - **C3b — a DRIVER FAULT, caught by reading its log:** the cut left an unbalanced `)` — "RED" was a file that would
    not load ("not ok 1 - …/playback.test.mjs"), not a test catching it. Re-cut (`            );`) → RED on the
    by-name test.
  - **C4c (a short step's span) — EQUIVALENT:** the span's 3 px left edge (`border-left`) already draws a 20 ms step
    (measured: 3 px); the reviewer's "clamps to 0, cannot be seen" missed the edge. The `max(3px, …)` added for it is
    REMOVED (less code); the TIMELINE test now pins a 20 ms span ≥ 3 px (`cut` with its first span made 20 ms through
    the stand-in's `patch`), and C4c re-cut as "the edge removed" → RED.
  - **UX1 (the two-line sentence) — a MUTATION FAULT, then a TEST GAP:** removing only `height` leaves
    `-webkit-line-clamp: 2` clamping; and the long headline chip set the row's height. Fixed at the root: the
    headline chip is at most 240 px wide (it wraps, the words keep ~500 px at 1440), the robot-two test pins the
    words' height and the track's box between a two- and a three-line sentence, and UX1 re-cut as the whole clamp
    removed → RED.
  - **UX2 (the foot row never wraps) — EQUIVALENT at the widths the brief tests (1440, 1024, the 700 px check):** the
    squeeze was the chip's `nowrap`; with the chip wrapping and the words' 260 px basis nothing squeezes. The row's
    wrap stays (narrower windows put the chip and the bar under the sentence).
  - **C4b (`step()` forgets it was playing) — NOT TESTED at page level, said plainly:** the 50 ms "previous" while
    playing is a timing of a running animation (a press within 3 frames); the arithmetic is pinned by the node test
    (C4a RED). The wiring is one argument.
  **After the fixes, every survivor re-run: R15, R16, C3b, C4c (edge), UX1 RED; UX2 equivalent; C4b untested (above).**
- **The tests lens, finding by finding:** T1 (9 survivors) → R2–R5, R28–R32 RED; T2 (the cache) → R1 + T2 RED; T3
  (threw / looping) → R8, R9 RED (a REAL threw run); T4 (the drawing) → R14–R17 RED; T5 (robot two's P-W1f) → R7,
  R20 RED; T6 (12 survivors) → R6, R10–R13, R18, R19, R21–R26 RED; T7 → the native compare (above); T8 → the
  committed recorder and `madeBy` (above).
- **Pictures:** guide 22's `preview.png` and `preview-failed.png` re-made from this round's shots (the dock and the
  cards changed: the speeds' height, the two-line sentence, the PLACEHOLDER chip under the words); `mechanism-menu.png`
  re-made (the note's padding); `preview-pair.png` deleted (above). A1's other pictures show nothing this round
  changed. Every PREVIEW state in four variants: `~/shulib-render-tmp/rmp1-a2u/shots-review/` (52 files).
- **PROSE REVIEW for this round (rule 4), run BEFORE this line:** `grep -rn -i -E "preview-pair|both robots, once|
  engine\.test\.mjs|publishes the builder without|PREVIEW (says it )?comes next|plays both robots|About this
  run|After the THROUGH|every line the library" docs/ README.md tools/route_maker/README.md --include=*.md` (the chunk
  files and `docs/api/` aside) → guide 16 :388 (RPV1's own page — true), guide 22 :115 and :121 (this round's), the
  README's "comes next" (true for a build without `--engine`). `grep -n -i "PREVIEW\|route builder"
  docs/internal/RELEASING.md` → :143-151: its PG1 sentence was STALE for the engine-required publish → **fixed**
  (the refusal by name, PG1 checking both forms). `docs/route-builder.md` :7-9 (done, failed or skipped) and guide 16
  :358-363 read in full: true. The word list (D5 above): clean. **Deliberately NOT touched:** `roadmap.md`,
  `build-order.md` (the coordinator's pointers at the merge), `docs/api/` (no `///` changed; `check-fresh` 0),
  `test/README.md` (no suite-side change: the fixtures, the recorder and the stand-in are the route builder's, in its
  README).
- **FINAL GATES (06:55–07:30, on the final tree):** host build 0 warnings (no C++ changed); suite green **3,778**
  (`suite_run.py green`); briefing regenerated with the suite binary unlinked first, then after the build — no change
  — and `check` current; src gate self-test **OK (377 detector cases)** (under `timeout -s INT 7000`, `nice`) and
  `check` **PASS (14 builds)**, `git diff -- Makefile src/ include/` EMPTY and no untracked file under `src include
  test` after both; ARM header gate exit 0 (259 lines); the PROS-free and sim-free greps clean; api_doc_tool
  self-test / coverage / fresh / examples / removability; staleness audit + self-test; doc_readability check-nav (260
  pages) / check-tone / self-test; `pages.yml` parses; the built site names no URL but SVG's namespace; the fixtures
  and the recorder `eol: lf`. **Node: 114 pass / 2 skipped. Firefox: PREVIEW 26 pass / 1 skipped with every variant's
  screenshots (the skip: the real engine, not in this tree), A1's 32/32 with screenshots. THE SEAM, with the final
  code and A2e's current module (overlay `integ3`, `preview.wasm` md5 `483d0232…`): PREVIEW 21/21 — THE REAL
  ENGINE in both forms against the native twin's record made at test time — and A1's 17/17.** No Chromium this
  round (D2). No browser or driver process left (`ps`).
- **Not done here, for the coordinator:** an owner and a go for the runner's PAIR MODE (with RPV1's `plantStartFor`
  fix and the pair check's guard) — until then the two-robot PREVIEW is met against the stand-in only; the `ci.yml`
  seam and `allowed_warnings.mjs` at A2e's merge (now enforced: the REAL test fails in CI without it); the deltas to
  acknowledge — D3 (the full report as far as the record goes; PREVIEW kept enabled after a load failure), D7 (the
  usability card on the claw), D2 (D18 (b) on this laptop). Not committed, not pushed; host and browser only.

### Verification-round findings and their disposition (2026-11-17 from 07:48 EST, `date`)
- 07:48 — Started. A fresh verifier's four findings (all minor): V1 a short step still hidden on the timeline (C4c's
  "equivalent" measured the span's own box, not what lies over it); V2 THE REAL ENGINE test in CI always falls back to
  the fixture, which still compares what robot one's facts write; V3 the time below zero on the first frame ("-0.00 s",
  "Between steps"); V4 the one-robot line last in About this run, out of view in its screenshots. Each re-checked in
  the code before any change; dispositions follow as they are made.
- 07:48–07:56 — **Each finding re-checked, then fixed at the root** (a browser probe first, on the code as the verifier
  found it — `scratch/rmp1-a2u/probe3/vis.mjs`: which step's span is ON TOP at every quarter pixel of the track, the
  spans made hit-testable for the probe only):
  - **V1 — REAL, and wider than reported.** The 20 ms step showed 0.25 px (its 3 px edge under the next span, a later
    sibling). Every step that took NO time was worse: carry's step 3 (skipped over) and two_starts' two starts showed
    **0 px** — each a 2 px tick at the same moment as the next step's span, which covered it whole. C4c's "equivalent"
    was wrong: it measured the short span's own box, not what lies over it. **Fixed** (`timeline.mjs`): the spans are
    drawn longest first, so a span shorter than its edge lies OVER the next one's start, never under it; on a tie the
    worse verdict on top (a failed start beside an ok one on one pace shows red). No CSS, no z-index: the playhead and
    the pair mark stay above. Probe after: the 20 ms step 3 px, carry's step 3 2 px, a two_starts tick 2 px; crop
    `probe3/cut_track.png` shows step 1's green edge before step 2's amber. **Test** (TIMELINE): what SHOWS, not the
    box — `shown(page)` samples `elementFromPoint` every 0.25 px with the spans hit-testable for the test; carry's step
    3 ≥ 1.75 px, cut's 20 ms step 1 ≥ 2.75 px, two_starts' step 1 (patched to "failed, and went on") ≥ 1.75 px, and the
    step after each still > 100 px. Guide 22's timeline paragraph: "a step too short to see at this scale is a thin
    mark at its moment". One of two zero-length steps on one pace stays hidden under the other (the worse shows): at
    this scale they are one moment; the step list names both.
  - **V2 — REAL.** shulib-v2's `ci.yml` (`git show shulib-v2:.github/workflows/ci.yml`, read only): the browser rows
    download only the `route-builder` and `route-builder-engine` artifacts and build no native program, so THE REAL
    ENGINE test would always take the fixture path in CI, whose run is robot one's facts' — and the README's "less the
    members robot one's facts … write" was not true. **Fixed:** `test/fixtures/record.mjs --out DIR` (the same recorder
    as the committed fixtures) writes the native twin's records into DIR; the test takes, in order, the native program
    (made now), `ROUTE_MAKER_NATIVE_RECORDS` (DIR, made by CI's Linux job from this tree), and only OFF CI the committed
    fixture (its comment now says plainly that this path goes red on a facts change); in CI (`MUST`) with neither it
    FAILS by name, so the merge cannot leave the rows on the fixture. Also compared now: the field notes (depth within
    1e-5), which no branch compared before. README paragraph rewritten. **Proven in a fresh overlay**
    (`scratch/rmp1-a2u/integ4/`: this tree + A2e's `engine_module.mjs` / `route_lines.mjs` as copied at 06:15, its
    module `integ3_wasm/`, A2e's native twin `tree/build/…`; site `integ4_site/` built `--engine`; script
    `integ4_v2.sh`, logs `integ4_logs/`): the recorder's `--out` record's result equals the committed fixture's; (a) the
    native program → pass; (b) `CI=1` + the records → pass; (c) the records with `park` 57 → 58 → RED "site: park";
    (d) one pace 0.01 in off → RED "site: pace 100"; (e) `CI=1` with neither → RED "in CI the native twin's record
    must be here …"; (g) the folder without the file → RED "… is missing"; (f) off CI with neither → the fixture, pass.
    The `wasm/CMakeLists.txt` that makes `MUST` true was planted in the OVERLAY only, removed by a trap.
    **The ci.yml half is the coordinator's (A2e's job, not in this tree)** — added to the seam: a draft applied to a
    copy of shulib-v2's `ci.yml`, parsed by PyYAML, is `scratch/rmp1-a2u/ci_seam/ci.seam.diff`: the site built AFTER
    the engine with `--engine build/wasm` (the 05:00 seam), then `node tools/route_maker/test/fixtures/record.mjs
    build/test/shulib_route_maker build/route_maker_site/robots.json --out build/wasm/native ready`,
    `build/wasm/native/*.json` added to the `route-builder-engine` upload, and `ROUTE_MAKER_NATIVE_RECORDS:
    build/wasm/native` in the browser rows' page step.
  - **V3 — REAL** (probe: a frame stamped 20 ms before the last → t = −0.02, "−0.02 s", "Between steps"). The cause is
    two clocks: `play()` reads `performance.now()`, and a frame's stamp is when the frame BEGAN, which can be earlier.
    **Fixed** (`preview.mjs` `tick`): dt is at least 0 — time never runs back. Probe after: t = 0, "0.00 s", the step
    playing named. **Test** (TIMELINE): `frameAfter(-20)` at Home → t 0, "0.00 s /", "Step 1: …". The screenshots
    re-taken (below).
  - **V4 — REAL.** The one-robot line was the last item of About this run, under robot one's 11 owed facts, out of the
    panel's 32vh at 1440 and 1024. **Fixed** (`timeline.mjs`): it is the second line, under the host-test-plant line
    (a refused robot two's line is the same member, so it moves too). **Test:** the one-robot test and the §2.5.2
    other-states test (all four variants, before the shot) assert the line is in the panel's view as it opens
    (`aboutLineInView`). Guide 22: "the first line of About this run says so".
  - Runs after the fixes: the model 114 pass / 2 skipped; Firefox PREVIEW 20 pass / 1 skipped (the real engine: not
    in this tree), A1's 17/17.
- 07:57–08:15 — **MUTATIONS on this round's final code** (driver `scratch/rmp1-a2u/mutate_round4.py`, round 3's machinery:
  ONE exact replacement, the site rebuilt for every page mutation, its test under the 4 GiB cap and `timeout 900`,
  restored in a `finally` and on SIGTERM/SIGHUP/SIGINT, mtime bumped, restoration asserted; backups `backups_r4/`,
  logs `mutlogs_r4/`, results `mutations_round4.json`; every RED's failing assertion read). **44 cut:** the new
  V1a (no order: spans in step order) → "carry: step 3 shows undefined px"; V1b (no tie-break) and V1d (the better
  verdict on top) → "two_starts: step 1 shows undefined px"; V1c (the longest on top) → carry's step 3; V3 (dt not
  clamped) → "-0.02 !== 0"; V4 (the line last again) → "About this run says it first, in view" — all RED. Re-run on
  the final code because their file changed (`timeline.mjs`, `preview.mjs`) or their catching test did (TIMELINE,
  the one-robot test): R1–R6, R10–R13, R19, R21–R26, R28–R32, C2d, C3d, C4c, C6d, UX1, old M8, M9, M10, M17, M18,
  M22, RM13, M36 and re-cut M37 — all RED, each on its own assertion (C4c, the span's edge removed, is now caught by
  what SHOWS: "cut: step 1 shows undefined px"). C4b stays GREEN, untested at page level as round 3 said (the
  50 ms "previous" while playing is a timing of a running animation; its arithmetic is pinned by C4a). Old M37 not
  applied (its code was rewritten; the re-cut ran). No mutant left (`grep` of the three fixed lines: each present
  once).
  **V2's mutations, in the overlay only** (`integ4_mut.py`: the overlay's copy of `preview.test.mjs` or
  `record.mjs`, restored in a `finally` and on signals, the overlay's committed fixture byte-compared after;
  `mutations_round4_v2.json`): V2a (the records folder ignored) → case b RED "in CI the native twin's record must be
  here"; V2d (the recorder ignores `--out`) → case b RED "… ready.preview.json is missing". The two controls: with
  the CI guard removed (V2b) case e turns GREEN, and with the records read but the fixture compared (V2c) case d
  turns GREEN — so the guard is what fails e, and the record is what is compared in d.
- **Pictures:** every PREVIEW state in four variants re-taken, `~/shulib-render-tmp/rmp1-a2u/shots-verify/` (52, Firefox
  26 pass / 1 skipped). Changed against `shots-review/`: the `about-*` and `one-robot-*` shots (the one-robot line
  now second, in view: `one-robot-dark-1024` reads "0.00 s / 4.88 s", "Step 1: Drive to (-36, -36)" and the line
  under the host-test-plant paragraph) and `running-*` (the spinner's angle). Guide 22's `preview.png` and
  `preview-failed.png` re-made from the new shots by `to_docs.py` are BYTE-IDENTICAL to the committed ones (the method
  checked: the review round's shot re-makes the committed `preview.png` exactly) — no picture in the docs changes.
- **PROSE REVIEW (rule 4), run 07:58, BEFORE this line:** `grep -rn -i -E "About this run|says so in one line|too short
  to see|thin mark|ROUTE_MAKER_NATIVE|record\.mjs|native twin|re-record|less the members|fixture less|robot two's
  mirrored run is not" docs/ README.md tools/route_maker/README.md --include=*.md` (the chunk files and `docs/api/`
  aside) → guide 22 :102, :115, :121 (this round's and true), the changelog :32 (true), the README :87–103 (this
  round's paragraph; :87–90's re-record rule true). `RMP1-A1-manual-checks.md` :21 ("Drag the timeline …") true.
  **Fixed by it:** the README's claim that the fixture path skipped what robot one's facts write (V2). **Deliberately
  NOT touched:** `roadmap.md`, `build-order.md` (the coordinator's), `docs/api/` (no `///` changed), `ci.yml` (A2e's
  job; the seam's draft is in scratch, above).
- **FINAL GATES (08:16–09:00, on the final tree; `scratch/rmp1-a2u/gates_v.sh`, logs `gates_v/`):** the suite binary
  unlinked, configure, briefing generated (reads it as unbuilt), host build **0 warnings** (no C++ changed), briefing
  generated again — no tracked change; suite green **3,778** (`suite_run.py green`, under the 4 GiB cap); briefing
  `check` and self-test; api_doc_tool self-test / coverage (6,439 entities, 214 headers) / fresh / examples /
  removability; staleness self-test + audit; doc_readability check-nav / check-tone / self-test; the PROS-free and
  sim-free greps clean; ARM header gate exit 0 (259 lines); src gate self-test **OK (377 detector cases)** (`timeout
  -s INT 7000`, `nice`) and `check` **PASS (14 builds)**; after both, `git diff -- Makefile src/ include/` EMPTY and no
  untracked file under `src include test`. Node 114 pass / 2 skipped; Firefox PREVIEW 26 pass / 1 skipped with every
  variant's screenshots (the skip: the real engine, not in this tree), A1's 17/17; V2's overlay cases as above. No
  Chromium this round (D2). No browser, driver or test process left (`pgrep`).
- **Added to the coordinator's seam at A2e's merge** (beside the 05:00 and 06:45 lists): the `ci.yml` draft
  `scratch/rmp1-a2u/ci_seam/ci.seam.diff` (the site after the engine, `--engine build/wasm`; the native twin's record
  of `ready` by `record.mjs --out build/wasm/native`; that folder in the `route-builder-engine` upload;
  `ROUTE_MAKER_NATIVE_RECORDS: build/wasm/native` in the browser rows' page step). Without it THE REAL ENGINE test
  fails by name in CI — on purpose. Not committed, not pushed; host and browser only.

## RMP1-A2u — the coordinator's merge *(2026-11-17)*

- **Source:** the `rmp1-a2u` worktree (base `1bcde4e`) as built, reviewed and verified (workflow `wf_6e780623-8bf`, 7/7);
  transport `65e71f7`; squash-merged in `integrate2` onto main `9d51074` (RMP1-B, RMP1-A2e, ROB1-BT and the top-bar fixes
  had landed since). The team lead, 2026-11-17: "PUSH AND MERGE EVERYHTING".
- **Conflicts (5):** app.mjs's top bar (A2u's `app.preview.topbar()`); page.test.mjs (A2u's `LEVELS` fixture robot and
  B's `ONE_LEVELS`, the Firefox prefs kept); the route builder's README (B's rows + A2u's engine and PREVIEW sections);
  the changelog and this log (both). A duplicated `setClaw` helper removed.
- **Reconciled after the merge** (a helper agent, verified here by re-running every suite): two PRODUCT bugs from the
  merge — PREVIEW was drawn on the Controls tab (`ui.tab === 'route' ? app.preview.topbar() : null`, app.mjs:702), and
  A2u's timeline span class `.seg` collided with B's segmented controls (the robot / drive-mode / axis pickers could not
  be clicked) → the spans are `.tl-seg` (style.css:270-277, timeline.mjs:34); seven tests pinned a world that no longer
  exists (robot two IS in the build; robot one's cascade has no levels; every PREVIEW runs robot two too) — re-pinned
  without dropping coverage (the one-robot path re-baked as a build without robot two; the level tests on both twins).
- **Gates:** the suite green (3,853); engine 15/15 (module vs native); model 156/156; page tests **Firefox 55/55**,
  **Chromium 52 + 3 skipped**; the strict site build WITH the engine (worker.mjs + preview.wasm published) exit 0, no
  warning; api fresh / examples / coverage / removability; check-nav / check-tone; staleness; both workflows parse.
- **Open:** the stand-in engine's "About this run" says "no recorded run of robot two" for one-robot fixtures (real
  use says "robot two's mirrored run is not in this preview engine yet") — cosmetic; the two-robot PREVIEW (SM4) still has
  no owner.

## After the merge — CI on Windows Firefox, and a symlink the merge committed *(2026-11-17, the coordinator)*

- **Windows Firefox failed two PREVIEW page tests on fd290c3.** (1) `preview.test.mjs` started Firefox WITHOUT the
  bounce-tracker pref the other three launchers pass (`privacy.bounceTrackingProtection.mode: 0`), so Windows Firefox's
  "127.0.0.1 has been classified as a bounce tracker" warning failed the no-warning check — fixed (9eac085), the two tests
  pass locally in Firefox. (2) `page.goto` timed out at 30 s once on the Windows runner ([light-1440] other states; the
  [dark-1440] run of the same test passed) — a slow runner; watched on the next run, not changed.
- **The A2u merge (88c8513) committed `tools/route_maker/node_modules` — a symlink to the coordinator's worktree.** The
  ignore rule `node_modules/` matches a folder only, not a symlink. Git treats ignored files as expendable, so checking the
  commit out replaced the integrate worktree's real install with the symlink (pointing at itself); on any other machine
  it is a dangling link (on Windows without symlinks, a FILE where npm wants a folder). Untracked; the rule is now
  `node_modules` (folder, file or link). No other symlink is tracked (`git ls-files -s`, mode 120000: none).

## RMP1-C — named routes, robot two on the field, "both robots run this together?", plain mechanism steps and small moves

- **Run by:** an executor subagent (Opus 5.5), worktree `.claude/worktrees/rmp1-c` (branch `rmp1-c`, base `88c8513`).
  Brief: `RMP1-C-named-routes-and-the-twin.md` (D1–D10). Host and browser only; nothing committed or pushed.
- **Started** 2026-11-17 (this log appended FIRST, before any code).
- **Resumed** 2026-11-17 (date read: `Tue Nov 17 2026`, the afternoon) by a second executor run, after the weekly usage
  limit stopped the first ~11:05 (its reviewers never started). Found: 51 files changed + 10 new, no src-gate self-test
  plant (`git diff -- Makefile src/` has no `(self-test plant`; no untracked file under src/include/test but the run's
  own goldens and `test/mirrored_pair_numbers.hpp`), the last suite run RED (4 cases). Every test object deleted and
  rebuilt from source before any result was trusted.
- **What the first run had built (kept, checked by a fresh build):** the C++ plumbing (`src/team/skills_route.{hpp,cpp}`:
  `Mirror`, `Together`, `TeamRoute`, `TeamRobot`, `teamRoutes()`, `registerRoutes()`, `addTeamRoutes()`; both autons
  files' one line), the pair check's `PairOptions::together` (D6) and its two numbers on their own
  (`test/mirrored_pair_numbers.hpp`, baked into the robots' descriptions with the skills mirror, the menu's size, the
  mirrored name budget and each arm's/lift's `homeAt`), the model (`lift.mjs` D7, `pair.mjs` D4/D5, the named-route
  edits in `project.mjs`, the checks RC21/RCT1/RCP1/RCL1/RCL2, the N-route export, `forward`/`backward` D9, the lines'
  `moveTo`/`startMoveTo` and the runner's `toGoal`), the C++ goldens GL20–GL22 and their suite cases, and half of the
  page (the inspector's "where it goes" and robot-two section, the words, `dom.mjs`'s menu and name box). Its last suite
  run was red (4 cases); on the fresh build the suite was green — it had fixed them before it stopped.
- **What this run added:** the switcher (`src/ui/routes_menu.mjs`, the top bar), robot two on the field and the
  too-close outlines (`field_view.mjs`), the mechanism menu grouped by name (`app.mjs`), the Straight tool (**F**), the
  nudge keys, Export of every route (`export_drawer.mjs`, `files.mjs`: both autons files' lines checked), the old tests
  moved to the new contract, `named_routes.test.mjs` (11 cases), `named_routes.page.test.mjs` (3 cases + 4 screens), a
  runner case (`RMP1-C runner`), the docs. Two bugs of the first run's found by the new tests and fixed: `freeName` past
  25 characters, and a small move after a step with a hole planned from NaN (now "not known"; the field never drew NaN).
  One found in the page: a start with a hole crashed the inspector's robot-two line (`fmt(NaN)`) — fixed (`startPose`).
- **Deltas from the brief (each with its reason):**
  1. **D3 asked ON THE ROUTE, not in a dialog.** The first run's modal popped on the first click of the field and
     stopped every flow; the question is the start's red issue with both answers one click away, the "Robot two"
     section's Yes/No (highlighted until answered), a toast, and at 1100 px and below the step pane opens to show it.
     No default: Export waits for it (RCT1). PREVIEW does NOT wait for it (the robot runs the same route either way).
  2. **PREVIEW and "one at a time":** the brief's landmine says PREVIEW plays one robot; it does with today's engine
     (`eng.pair` absent). For a route answered "No", the page drops a pair verdict from the run and says so, so a
     future two-robot engine can never contradict the page. Nothing else in PREVIEW changed.
  3. **"No levels yet" note gone from the mechanism menu:** the cascade now has its plain steps; "Move to a level" is
     offered only when levels exist, and a level move in an old file keeps its red words (RC18).
  4. **`skillsRoute()` is no longer declared in `skills_route.hpp`** — each route's entry lives in the region's anonymous
     namespace (several routes, one file). Nothing outside the region used it but the two autons lines this run
     replaced (grep: `docs/api/field_mirror.md`, `override_mirror.md`, cookbook 09 and guide 17 use `skillsRoute()` as
     the LIBRARY's generic example — still valid, left; guide 17 now points at the team's lines).
  5. **The markers keep the name "skills"** whatever the routes (the region is one; renaming it would be a contract
     change for nothing).
- **Tests added (each seen RED against a mutant, below):** node — `named_routes.test.mjs` (D1 set edits + undo, freeName,
  the old project, D2 the N-route export and the C++ stems, D3 the question, D5 the team lead's start too close /
  3 in out clear / mid-path / never / the one-click fix is the LEAST move, D7 the height tracker and refusals, D7/D8 the
  menu, D9 small moves), the RC21 rewrite in `checks.test.mjs`, FA4 for both autons lines (`region.test.mjs`,
  `host.test.mjs`), RT7's two new refusals and the named-routes project (`project.test.mjs`); C++ — `RMP1-C runner`
  (`route_maker_runner_test.cpp`: `toGoal` equals the compile-time `m.to(...)`, refused past the travel); the first
  run's GL20–GL22 and `RMP1-C D6` ×2 kept; browser — `named_routes.page.test.mjs` (the switcher, robot two + the
  question + too close + its fix, the cascade's steps + the claw's buttons + Straight + the nudge keys; 4 screens).
- **Mutations (every one observed RED, restored, its file's timestamp bumped; C++ each on a fresh link, the binary
  removed before each build so the build's doc gate never ran a mutant):** M1 duplicate names case-sensitive, M2 the
  start not pair-checked, M3 the pair check ignoring "one at a time", M4 "down" adding, M5 the list's Together, M6 back
  up driving forward, M7 "skills" deletable, M8 either robot's line accepted, M9 freeName past 25, M10 two routes one C++
  stem, M11 a small move from a NaN pose, M12 a lift past its top, M13 "up" from an unknown height, M14 the 15th route
  allowed, M15 a pneumatic's buttons unnamed, M16 only the first route exported, M17 no question for an old project —
  all 17 CAUGHT (node). C1b robot one mirrored (re-cut: C1 broke its own build, `robot` unused), C2 an unmarked route
  mirrored, C3 the C++ pair check ignoring `together`, C4 the runner's lift past its top, C5 the runner's arm in inches,
  C6 the gap retyped (5 in: the fixture descriptions' freshness case fails — the page's number IS the C++'s), C7b no
  `homeAt` in the description (re-cut: C7's build failed at the doc gate on C6's still-linked mutant — the driver now
  removes the binary first) — all 7 CAUGHT. B1 Shift nudging 1 in, B2 robot two not drawn, B3 a taken name accepted,
  B4 Up moving down — all 4 CAUGHT (Chromium, each against its own scratch site).
- **Prose review (greps run, hits read, BEFORE this line):** `skills_route` and `robot_one_autons|team/autons` over
  `docs/` + `README.md` (internal excluded) — fixed guide 07 :129, guide 16 :5, :73-75, :372, guide 21 :33-34, README
  :331; guide 17 gains the team's lines; `docs/roadmap.md` :577 a dated note (history, left); `docs/api/field_mirror.md`,
  `override_mirror.md`, cookbook 09, guide 17's example — the LIBRARY's generic `skillsRoute()` example (left).
  `route builder|route maker` over the same — guide 14 :563-566 fixed (named routes; "the mirrored robot drawn" no longer
  missing); guide 09, 18, `how-the-docs-stay-true.md`, `api/routine.md` checked, nothing stale. `Mechanism menu|Move to
  a level|Set a position|named \`skills\`` — guide 22 rewritten (steps 2, 3, 5, 7, "Reading the code", new step 9, the
  reference tables, the keyboard, the project file); `docs/route-builder.md`, the changelog (top of API 2.9). Pictures:
  `docs/assets/route-builder/{mechanism-menu,export}.png` re-taken, `{robot-two,routes-menu}.png` new (Playwright's
  Firefox, light 1440).
- **Found for the coordinator:** `tools/route_maker/node_modules` is a TRACKED symlink at the base 88c8513 (into the
  `integrate` worktree); this run's `npm ci` replaced it and it was put back (`git checkout`) — shulib-v2's 531ffd4
  already untracks it. Files this run changed that shulib-v2 also changed since 88c8513: `docs/changelog.md`, guides 07
  and 21, `PROJECT-BRIEFING.md`, `RMP1-PROGRESS.md`, `test/robot_twin_test.cpp`. `test/robot_one_autons_test.cpp`
  (the robot-one setup's) was touched by the first run: its pin moved with `robot_one_autons.cpp`'s line.
- **Resumed again** 2026-11-19 (date read: `Thu Nov 19 03:33 PM EST 2026`) by a third executor run, after the weekly
  limit stopped the second (~14:55 11-17) after its log through "Found for the coordinator" but before its gates. Found:
  no src-gate self-test plant (`git diff -- Makefile src/` has no `(self-test plant`; untracked under src/include/test
  only the run's goldens and `test/mirrored_pair_numbers.hpp`); every mutation backup equal to its restored file; its
  last Chromium page run red (D7–D9 :243, 1 !== 2) and an unlogged browser mutant B5 (the field's handles never
  shrinking) begun 14:38 for the fix it made in `field_view.mjs`; its src-gate self-test interrupted. `build/test`
  deleted and rebuilt from source before any result below was trusted.
- **Third run: nothing failed, nothing re-done.** Every gate below ran on the fresh build and passed; the second run's
  work is kept whole. One mutant finished: **B5** (the field's handles never shrink when zoomed:
  `handleR = () => inPx(1.6)`) — CAUGHT in Chromium (D7–D9: two points 1 in apart), restored (same md5), its own
  scratch site. The second run's Chromium red (D7–D9 :243) was its last run BEFORE that `handleR` fix: green now.
- **Found for the coordinator (third run):** at 1024 px the top bar's "Unsaved changes" chip is cut to "Uns" with no
  "…" (the switcher keeps its width; the chip is the words that give way, as the top bar's rule says; `.chip` is a
  flex box, so `text-overflow` draws no "…") — a polish item, not changed here. The engine was built with the shared
  emsdk read-only (`EM_FROZEN_CACHE=1`, `TMPDIR=build/tmp`, its git tree clean after).

### Gates (2026-11-19, `date`: 15:45–16:43 EST; every one on the fresh build)

- Build: `rm -rf build/test`, configure, `briefing_status.py generate` (unbuilt), `cmake --build build/test -j8` →
  exit 0; its gates: `doc coverage: 6485 public entities across 215 headers, all documented`, `check-nav: OK — 262
  public pages`, `check-tone: OK — 47 pages`, `first_week: OK`.
- Suite (`python3 tools/suite_run.py green`, 4 GiB cap): `suite run: green — 3859 cases, 3886114 assertions`.
- Briefing regenerated on the built binary: `Suite: 3,859 cases / 3,886,114 assertions, 3 skipped — green`;
  `briefing status: generated block current`; self-test `OK (38 cases …)`.
- Node (`node --test tools/route_maker/test/*.test.mjs`): `# tests 167 # pass 167 # fail 0`.
- Page, Firefox (`test/page/*.test.mjs`, `build/route_maker_site`): `# tests 86 # pass 84 # fail 0 # skipped 2` (WG3 in
  the browser and THE REAL ENGINE: both need the module — run below). Chromium: `# tests 86 # pass 81 # fail 0
  # skipped 5` (the same two, the download way ×2 — Chrome writes the folder — and the old-Firefox refusal).
- The engine (Emscripten 6.0.13, `tools/route_maker/wasm`): built, exit 0; `node --test test/wasm/*.test.mjs` with the
  native twin: `# tests 15 # pass 15` (WG1 compiles GL20–GL22's exported C++ into the module); `page/engine.test.mjs`
  (WG3): Firefox `# pass 1`, Chromium `# pass 1`; the page suite in Firefox on a site built `--engine`: `# tests 86
  # pass 86 # fail 0 # skipped 0`.
- ARM header gate: `TU includes 259 headers` → `ARM compile gate: CLEAN (all v2 headers)`. CI greps: `library is
  PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core is sim-free`. `git diff -- include/`: empty.
- Compile refusals: `100% tests passed, 0 tests failed out of 353`; 353 TUs.
- `api_doc_tool`: self-test `OK`; check-coverage as above; check-fresh exit 0; check-examples `988 quoted lines, all
  verbatim`; check-removability `no public doc references docs/internal/`. `doc_staleness_audit.py`: self-test `OK (16
  …)`; `clean (37 live docs, 0 numeric claims checked)`.
- src gate `check`: `src build gate: PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly
  their owed facts (10 refused; every one links in the gate's typed copy); …)`; `self-test` (under `timeout -s INT`):
  `src build gate self-test: OK (398 detector cases, each against a real build)`. After it: `git diff -- Makefile src/`
  has no `(self-test plant`; untracked under src/include/test only GL20–GL22 and `test/mirrored_pair_numbers.hpp`.

### RMP1-C review findings and their disposition (2026-11-19 from 17:06 EST, `date`)

Three reviewers (correctness, tests, docs) returned 24 findings; overlaps merged below (the skills toggle: correctness
#7a + docs #3; the retyped 6 in: correctness #5 + docs #6; the briefing line: tests #9 + docs #8). Each was re-checked
against the tree before its disposition was written.

**Correctness**

1. **PREVIEW refused a "together" route the page finds too close (RCP1) — REAL, fixed.** Re-checked: `preview.mjs`
   exempted RCT1 only, so the A2u fixture `pair_fail` answered Yes was refused. Robot one's run does not depend on the
   gap. Now `checks.mjs` exports `ROBOT_TWO_ONLY` (RCT1, RCP1, and RCM1 below), and PREVIEW waits for none of them.
   Page test `RMP1-C: a route both robots run TOGETHER …` (`preview.test.mjs`; the stand-in answers the same route
   answered Yes, `standinScript(…, {together})`). Mutant B6 (RCP1 not exempted) below. **Correction to delta 2
   above:** "Nothing else in PREVIEW changed" was wrong: the RCT1 exemption was a change, and it was incomplete.
2. **Route names clashing with the other autons — REAL, fixed.** Re-checked: `nameProblem` and the menu box compared
   only the project's routes. `shulib_route_maker` does not compile `src/team/` (on purpose), so the names cannot be
   read from the real registration there. Instead `test/route_maker_describe.hpp` holds `kOtherAutonNames`
   (`"40in forward"`, `kAcceptanceRunName`) and bakes them as `"otherAutons"`. The new suite case `RMP1-C: the other
   autons` holds that list equal to both robots' real registries minus `teamRoutes()`. The page refuses those names
   with capitals ignored, in RC21 and in the name box, quoting the library's "the name is already registered".
   `freeName` skips them, and the route cap is `maxAutons − otherAutons.length` (`OTHER_AUTONS = 2` is gone). The
   fixture descriptions x/y/z were re-made (`describe`), and the diff read: one line each.
3. **The too-close fix pushed a wall-flush start into the wall — REAL, fixed.** `startAwayFromLine` now finds the
   least move whose component away from the line clears the gap, inside a box that keeps every corner no further
   past a wall's face than the start's already was. Against a wall the move slides along it, and the words say
   **Slide the start N in along the blue wall, away from the diagonal**. With no such move, no one-click fix is
   offered and the words still say which way. The team lead's start is unchanged (0.8 in, straight off the line).
   Node case: flush on the blue wall at x = −50, −52, −58; no error after the fix; never into the wall; and no
   direction (every 0.1°, bisected) clears it by less.
4. **Export's errors unnamed for the route on the field — REAL, fixed** (`app.at()`). The page test now puts an error
   on the route on the field AND on another: both are listed under their names.
5. **Numbers retyped in JS — REAL, fixed.** The Yes button reads `too-close check: ${pairCheck.gapIn} in`.
   `freeName(routes, robot, base)` takes the budget from `robot.mirroredAutonNameMaxChars`; a node case uses a
   12-character budget. Mutation cannot see the label while the gap is 6 (the page bakes its robot): checked by
   reading the code.
6. **The nudge keys acted on a stale curve point — REAL, fixed.** `select(sel, point = null)` resets the point;
   only `selectHandle` passes one, and a dragged handle is now selected too (`field_view.mjs`). Page case: a point of
   curve 1 picked, curve 2 picked in the list → the arrows move nothing; a dragged point of curve 2 is the one
   nudged.
7. **(a) "skills" could be unmirrored (merged with docs #3) — REAL, fixed: the page keeps it mirrored.** ROB1-BT's
   ruling (robot two runs the skills route mirrored) stands, and (1d)'s `≥ 1` is that ruling's pin. The toggle is
   disabled on the skills route ("always, for the skills route"). A project file whose skills route is unmirrored
   gets the new RCM1 error with **Mirror it for robot two**. The coordinator may reverse this by relaxing (1d)
   instead. **(b) robot two's heading untested — REAL, fixed:** a node case reflects headings the mirror changes
   (0→−90, 30→−120, 90→180, −135→45), worked by hand. Page case D4: robot two's square, front tip and path ends at
   heading 30 → −120, compared with `squareCorners` of the hand-reflected pose.

**Tests**

8. **(1d)'s use of the answer untested — REAL, fixed.** The per-entry logic moved into `checkTeamRoutes()`
   (autons_test.cpp). (1d) calls it on `teamRoutes()`. The new `autons (1d')` runs it on a planted TEST route on the
   team lead's start, three ways: Yes fails, "4.35 in apart"; No is skipped by name; NotAnswered is reported. The
   side defect is fixed too: `PairOptions::answered`, and the sentence now says "nobody has said yet …".
9. **The page's sampling untested (J1–J3, J5) — REAL, fixed.** New node case: a straight drive and a curve across
   the diagonal, a turn in place 45→135, and a face-first turn 45→−45, each with both ends clear; each must give RCP1
   at its step. The same drive, already facing along the line, stays clear.
10. **The fix on one side only (J4)** — the mirror image of the team lead's start is now asserted: under 1.5 in,
    x + y down.
11. **"Go down only N" never exercised (J9)** — home → up 6 → down 10: the fix is down 6, and RCL2 clears.
12. **Lift-tracker edges (J6–J8)** — startHome → wait → up 6 is known, clean, and exported `lift.to(6_in)`; an owed
    ledger level has no height (null), so "up" after it is "from where?"; up 0 and down 0 are refused (RCL1).
13. **deleteRoute's shift (J10)** — a 4-route set; the last route deleted keeps the one before it on the field.
14. **The runner's arm at 0 (C4)** — `arm.to(0)` added.
15. **Page and suite agreement on a moving route — REAL gap, closed.** New golden **GL23**
    (`gl23_pair_near_the_diagonal`, a TEST route on robot x, mirrored, Yes): it drives toward the diagonal and
    stops 12 in off it. The page's plan gives a least gap of 9.0 in (node case). The C++ case runs
    `checkMirroredPair` on the host test plant: ok, checked, least 9.48 in (each robot stops about 0.24 in short;
    the page is the stricter one), held within 1 in. GL23 is in the WG list (`route_maker_goldens.cpp`), so the
    engine was rebuilt for WG1/WG2. The optional "a golden the page flags" cannot be exported (the export refuses
    an error), so it is not made.
16. **The briefing's count depends on the build path (merged with docs #8)** — real, not a defect of this tree.
    Logged for the coordinator below: regenerate in the main tree after the merge.

**Docs**

17. **D6's docs (guide 17, cookbook 09, HA-260, field_mirror) — REAL, fixed** except the library header. Guide 17
    point 3 now says the check runs for "every mirrored route the team says both robots run at the SAME time
    (`Together::Yes`)", that a No route is skipped by name, and that an unanswered field-start route fails the
    suite. Guide 17 :100 is now plural. Cookbook 09 says "when both robots run the route at the same time". HA-260's
    *Source:* now names `test/mirrored_pair_numbers.hpp`. `include/shulib/competition/field_mirror.hpp:43`
    (generated into `docs/api/field_mirror.md:236`) still says "every mirrored entry": that is library text, and
    there is no include/ change in this run, so it is logged below for the coordinator. The pair-check grep is
    added to the prose review (below).
18. **Guide 22's "0 to 40 in" — REAL, fixed:** the number is dropped ("the step says the range"). The page reads
    the range from the description, so it shows 0–20 after the merge with fd290c3. Logged below.
19. **The changelog's "what you must do" — REAL, fixed:** "export once more … every region written before this
    update lacks the list `kTeamRoutes` and no longer compiles". It adds the together question and NotAnswered.
20. **Guide 22 step 2 (four points) — REAL, fixed.** (a) "No: the too-close check is off (a tank's half-turn step
    is still refused)". (b) The second button is named, and so is the slide variant. (c) The caption now reads "a
    start 0.8 in too close". (d) "the page checks that its plan keeps …", with chapter 17's slow-first advice, also
    in `route-builder.md` and the changelog. The picture `robot-two.png` was re-taken (the skills toggle and the
    Yes label changed).
21. **The inspector's typed "6 in"** — see 5.
22. **Leftover wording — REAL, fixed:** guide 17 :100, guide 21 :8 / :34 / :366, `route-builder.md` :28,
    `src/team/autons.hpp` :6, guide 07 :129–131, and guide 22's export.png caption ("each robot's addTeamRoutes
    line"). The name words are now "plain keyboard characters … no %, \" or \\" (the box) and "letters, digits,
    spaces, punctuation" (RC21).
23. **Merge items not logged — REAL:** added below.

**Mutations of this round.** Each mutant was observed red and then restored, its file's timestamp bumped. Driver:
`scratch/rmp1-c/fix/mut.py`, which restores in `finally` and on SIGTERM/SIGHUP/SIGINT. Node: the full node suite per
mutant. C++: a fresh link per mutant (the binary removed first, so the doc gate never ran a mutant), then the
named cases; the whole build was re-made from the restored sources after. Browser: Chromium, each mutant on its own
scratch site, with the file restored before the page test ran.
- Node, all CAUGHT. The reviewers' J1–J10 (J4 re-cut onto the new `side` line), J11 (robot two's heading flipped),
  N1 (the other autons case-sensitive), N2 (not checked), N3 (freeName's budget retyped 25), N4 (freeName ignoring
  the other autons), N5 (the cap retyped − 2), N6 (the fix ignoring the walls), N8 (skills unmirrored allowed), N9
  (the slide words lost). N7 (the slide longer than the least) first SURVIVED: the "1 % less" check measured only
  the move's part away from the line. The flush case now compares the move against the least over every direction
  (0.1° apart, bisected), and N7 is CAUGHT on a re-run, with N6 again.
- C++, all CAUGHT. C1 `o.together = true` and C2 `= false` (both red in (1d')). C3 first did not build (`field`
  unused, -Werror) and was re-cut as C3b (`… && false`), CAUGHT: (1d') :317 `unanswered`. C3c `o.answered = true`
  (the sentence). C4 the runner's arm at 0 refused. C5 the description's "40in forward" drifting ("the other
  autons" red). C6 the pair check's square 12 in (GL23's agreement red).
- Browser, all CAUGHT. B6 PREVIEW waiting for RCP1 (the preview page case), B7 `app.state().at` back (D1: the
  current route unnamed), B8 `select` keeping a stale point and B9 a dragged handle not picked (D9 nudge), B10 the
  skills toggle enabled (D3–D5), B11 robot two's heading flipped (D4: the square drawn wrong).
- After the runs: every mutant's `old` text is back in its file, exactly once. `git diff -- Makefile src/` has no
  `(self-test plant`. Untracked under src/include/test: only GL20–GL23 and `test/mirrored_pair_numbers.hpp`.

**Prose review of this round (greps run and hits read BEFORE this line).**
- `pair check|mirrored_pair_check|mirrored_pair_numbers|(1d)|Together::|kMirrorGapIn` over `docs/` + `README.md`
  (internal excluded). Fixed: guide 17 :238–256, cookbook 09 :110–113, HA-260 :2354. Read and left: guide 14 :547
  (true for a Yes route), `docs/api/chassis.md` :1061 (the leg, still true), guide 22 :199 (true), changelog :414
  (MIRROR1's dated entry: history). `docs/api/field_mirror.md` :236 is generated from the library: logged below.
- `every mirrored entry|For every mirrored|mirrored entry robot two` — the hits are the same ones.
- `Letters, digits|0 to 40 in|kept 6 in apart|Runs it mirrored|turns the mirror off` over docs + the page's
  src. Fixed: guide 22 :54 (the skills route always mirrored).
- `6 in apart|keeps them|keeps the two` — fixed `route-builder.md` :6 and changelog :30; HA-260's own wording left
  (it states the claim it registers).
- `PREVIEW` near `error|red|fix` in guide 22 — fixed :148 and :158 (robot two's red waits only for Export).
- `40in forward|acceptance run take|at most 14` — fixed guide 22 :229–233. Guides 16/17 quote the auton itself
  (still true).
- `skills route both robots share|the skills route mirrored|each one marked` — fixed guides 07, 17 and 21, and
  `src/team/autons.hpp`.
- The picture `robot-two.png` was re-taken (Playwright's Firefox, light 1440), because the skills toggle and the
  Yes label changed. The other three pictures show nothing that changed.

**Found for the coordinator (this round).**
- `include/shulib/competition/field_mirror.hpp:43` (and its generated `docs/api/field_mirror.md:236`) still says
  "The host suite runs every mirrored entry robot two registers TOGETHER". Since D6 it checks only the routes
  answered Yes. That is library text, and this run has no include/ change, so it is a known divergence for the
  next include/ change.
- At the merge onto shulib-v2:
  - `docs/where-your-code-goes.md` (34eb853) :3, :13 and :42 say "the skills route both robots share" / "change
    the skills route": they need the plural edit README :331 got.
  - `PROJECT-BRIEFING.md`'s suite line conflicts. Here it is 3,862 / 3,886,152 at this worktree's path; the
    assertion count depends on the source path's length (ee5a1f5). Regenerate in the main tree after the merge:
    `rm -f build/test/shulib_tests`, build, then `briefing_status.py generate`.
  - Robot one's cascade travel becomes 0–20 in (fd290c3). Guide 22 no longer states a number; the page reads the
    range from the description.
- The skills route is kept mirrored in the page (correctness #7a / docs #3). If the coordinator would rather let it
  be unmirrored, relax autons (1d)'s `verdicts.size() >= 1` and drop RCM1.
- The engine tests (`test/wasm/*.test.mjs`) cannot run under `ulimit -v`: a WebAssembly instance reserves more
  address space than 8 GiB ("Out of memory: Cannot allocate Wasm memory"). They ran under `systemd-run --user
  --scope -p MemoryMax=6G` instead, a real cap.

### Gates after the review fixes (2026-11-19, `date`: 17:55–18:37 EST; every one on the final tree)

- Build: `rm -rf build/test`, configure, then `briefing_status.py generate` (on an unbuilt tree), then `cmake --build
  build/test -j8` → exit 0. Its gates: `doc coverage: 6485 public entities across 215 headers, all documented`,
  `check-nav: OK — 262 public pages`, `check-tone: OK — 47 pages`, `first_week: OK`. The doc gates were re-run
  (`--target shulib_doc_gates`) after the last guide edit: all the same.
- Suite (`python3 tools/suite_run.py green`, 4 GiB cap): `suite run: green — 3862 cases, 3886152 assertions`
  (+3: (1d'), GL23, the other autons).
- Briefing, regenerated on the built binary: `Suite: 3,862 cases / 3,886,152 assertions, 3 skipped — green`.
  `check`: `generated block current`.
- Node (`node --test tools/route_maker/test/*.test.mjs`): `# tests 173 # pass 173 # fail 0`. Re-run after the last
  comment edit: the same.
- The engine (Emscripten, `EM_FROZEN_CACHE=1`, `TMPDIR=build/tmp`, the shared emsdk's git tree clean after): built,
  exit 0. `node --test test/wasm/*.test.mjs` with the native twin, under `MemoryMax=6G`: `# tests 15 # pass 15`
  (WG1/WG2 now include GL23).
- Page, on the site re-built from the final tree (`otherAutons` baked). Firefox with the screens: `# tests 89 # pass
  87 # skipped 2`. Chromium with the screens: `# tests 89 # pass 84 # skipped 5` (the same skips as before). On a
  site built `--engine`: Firefox `# tests 62 # pass 61 # skipped 1` (THE REAL ENGINE passed); `page/engine.test.mjs`
  (WG3) passes 1 of 1 in Firefox and in Chromium. Screens: `~/shulib-render-tmp/rmp1-c/shots-fix/` (light/dark, 1440
  and 1024).
- ARM header gate: `TU includes 259 headers` → `ARM compile gate: CLEAN`. CI greps: `library is PROS-free …`,
  `layering holds: core is sim-free`, and the route builder's files are LF. `git diff -- include/` is empty.
- `ctest -R compile_refusal`: `100% tests passed, 0 tests failed out of 353`.
- `api_doc_tool`: self-test OK, check-coverage as above, check-fresh exit 0, check-examples `988 quoted lines, all
  verbatim`, check-removability `no public doc references docs/internal/`. `doc_staleness_audit.py`: self-test
  `OK (16 …)`, then `clean (37 live docs, 0 numeric claims checked)`.
- src gate `check`: `src build gate: PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly
  their owed facts (10 refused; every one links in the gate's typed copy); …)`.
- src gate `self-test` (under `timeout -s INT`, in its own copy): `src build gate self-test: OK (398 detector cases,
  each against a real build)`. After it: `git diff -- Makefile src/` has no `(self-test plant`, and the only
  untracked files under src/include/test are GL20–GL23 and `test/mirrored_pair_numbers.hpp`.
- Not run here: the Windows/Linux CI (the coordinator's, after the verifier).

### Verification-round findings and their disposition (2026-11-19 from 18:53 EST, `date`)

The fresh verifier returned 3 findings (all minor). Each was re-checked against the tree first; all three are real.

1. **The arrows moved a point of a curve nobody picked (after a new curve, a delete, a move, a duplicate) — REAL,
   fixed at the root, wider than reported.** Re-checked: `edit(r, sel)` set `ui.selected` and kept `ui.point`. The
   verifier's suggested fix (reset only when the selection *differs*) is not enough: deleting the picked curve
   selects `min(i, n − 2)`, which is often the SAME index, now holding the next curve. Two more paths had the same
   defect: an **undo/redo** (the step at the selected index can be another one), and the inspector's **point
   delete** (`update`, the point indices shift). Fix (`app.mjs`): one `pick(sel, point = null)` is now the only code
   that writes `ui.selected` / `ui.point` (grep: two assignments, both in it). `edit()` with a selection drops the
   point; only the nudge (no selection passed) keeps it; `select`, `editSet`, `switchRoute`, `adoptProject` and the
   start-up fallback go through `pick`. Undo/redo go through `travel()`, which drops the point and, when the undo
   shows ANOTHER route, the selection too (it was an index into the other route: undoing a new route left the
   skills route's start selected). Page case D9 grows (`named_routes.page.test.mjs`): a point picked, then a new
   curve drawn; the picked curve deleted (the next curve takes the same index); an undo that puts another curve at
   the selected index; an undo that shows the skills route again (nothing of it selected). Each: the arrow moves
   nothing. Mutants V1–V4 below.
2. **A drag at ≤ 1100 px slid the step drawer over the field — REAL (a regression from fix 6), fixed.**
   `select(sel, point, open = true)`; `selectHandle(…, open)`; `field_view.mjs` passes `!drag.moved`. A drag picks
   the step and its point without opening the drawer; a click with no movement opens it, as at the base. New page
   case `D9 at 1024 px`: after a drag, `.app` has no `data-right`, the dragged step is the one nudged; a click on
   it opens the drawer. Mutants V5 (the drag opens it) and V6 (the click does not).
3. **Guide 16 and the brain's boot line still said "the skills route both robots share" — REAL, fixed (the brain
   line too).** Guide 16 :136–138 is plural ("The route builder's routes are not in the autons file: both robots
   take them from …"). The boot line (`src/common/dropped_autons.hpp:84`) is now `(the route builder's routes are
   fixed in %.40s)`: the file it names is the same; the words now fit every route it holds. Its pin
   (`test/this_robot_test.cpp:219`), guide 17 :395–396 (which quotes the line), the `///` comments in
   `dropped_autons.hpp` and `src/common/robot_words.hpp` (`routeFilePath`), and the changelog's RMP1-C entry ("The
   boot's dropped-autons line now says …"; the banner now says "and one boot line") follow. Neither src/common file
   is in the coordinator's in-flight list (`boot_source.*` is); the assertion count is unchanged (a string). The
   wider grep also fixed: guide 16 :271/:273 ("the route builder's routes `using namespace this_robot;`", "In a
   route robot two runs MIRRORED"), the page's Help text ("Build your routes … the Route button at the top"),
   `region.mjs`'s `/** */`, and two test comments (`route_preview_model.hpp:169`, `route_maker_golden_test.cpp:470`).
   Mutant V7 below.

**Mutations of this round.** Driver `scratch/rmp1-c/vfix/mut.py` (the fix round's, restoring in `finally` and on
SIGTERM/SIGHUP/SIGINT, timestamp bumped). Browser: each mutant on its own scratch site built from it, the file
restored before Chromium ran `named_routes.page.test.mjs` under `MemoryMax=6G`; re-run with the failing assertion
printed (`vfix/mut_detail.log`). C++: the binary removed, a fresh link, the named case; then the whole build re-made
from the restored sources (exit 0).
- V1 `edit` keeps the point → CAUGHT: D9 "a new curve drawn: nothing moved".
- V2 the verifier's weaker fix (reset only when the selection differs) → CAUGHT: D9 "the picked curve deleted:
  nothing moved" (the same index).
- V3 `travel` keeps the point → CAUGHT: D9 "after an undo: nothing moved".
- V4 `travel` keeps a selection across routes → CAUGHT: D9 "nothing of the skills route selected".
- V5 the drag opens the drawer → CAUGHT: "a drag opens no drawer over the field".
- V6 the click opens no drawer → CAUGHT: expected `'open'`.
- V7 the boot line back to "the skills route both robots share" → CAUGHT: `this robot (R7)` :222 / :223, 4 of 31
  assertions red.
- After the runs: each mutant's `old` text is in its file exactly once and its `new` text nowhere. `git diff --
  Makefile src/` has no `(self-test plant`.

**Prose review of this round (greps run and hits read BEFORE this line).**
- `skills route both robots share|the skills route mirrored|each one marked|shared skills route|the skills route's
  entry` over docs, README, src, test and the page's src (internal excluded). Fixed: guide 16 :136–138 and guide 17
  :395–396 (the boot line), and the comments listed in 3. Read and left (true of the skills route itself): guide 22
  :54, `checks.mjs` :269, `this_robot_test.cpp` :66, `robot_twin_test.cpp` :347, `robot_one_autons_test.cpp` :65 (a
  case name: the skills route, as written, on robot one). History: changelog :139 (ROB1-BT's dated entry). Not
  mine to edit: `src/robots/mechanism_table.hpp` :236 (the coordinator's in-flight robot setup), logged below.
- `Build the skills route|file the skills route` over the page and docs: fixed (Help, `region.mjs`); no hit left.
- `arrow key|the arrows|drawer|narrow window|1024` over guide 22, `route-builder.md` and the changelog: guide 22 :82
  now says how a curve's point is picked for the arrows ("click or drag it on the field"); :393's table and the
  changelog's line stay true. No doc describes the drawer on a drag.

**Found for the coordinator (this round).**
- `src/robots/mechanism_table.hpp:236` says "The shared skills route (src/team/skills_route.cpp) is compiled into
  BOTH robots' builds": still true, but the file now holds every route the builder exports. src/robots/ is your
  in-flight area, so it is left.
- At the merge: `docs/where-your-code-goes.md` (main) — the same plural edit as before; and if its text or any
  other main-only page quotes the boot line, it is now `(the route builder's routes are fixed in
  src/team/skills_route.cpp)`.

### Gates after the verification-round fixes (2026-11-19, `date`: 19:03–20:28 EST; every one on the final tree)

- Build: `rm -f build/test/shulib_tests`, `briefing_status.py generate` (unbuilt), `cmake --build build/test -j8` → exit
  0: `doc coverage: 6485 public entities across 215 headers, all documented`, `check-nav: OK — 262 public pages`,
  `check-tone: OK — 47 pages`, `first_week: OK`. Re-run (`--target shulib_doc_gates`) after guide 22's last edit:
  the same.
- Suite (`suite_run.py green`, 4 GiB cap): `suite run: green — 3862 cases, 3886152 assertions` (unchanged: the boot
  line's pin is a string). Briefing regenerated on the built binary, `check`: `generated block current` (3,862 /
  3,886,152 at this worktree's path).
- Node: `# tests 173 # pass 173 # fail 0`.
- The engine rebuilt (pinned emsdk, `EM_FROZEN_CACHE=1`, `TMPDIR=build/tmp`; it reads `route_preview_model.hpp`, whose
  comment changed): exit 0, 0 warnings, the emsdk's git tree clean after. `test/wasm/*.test.mjs` under `MemoryMax=6G`:
  `# tests 15 # pass 15`.
- Page, a site built from the final tree. With the screens (`~/shulib-render-tmp/rmp1-c/shots-vfix/`, light/dark at
  1440 and 1024): Firefox `# tests 90 # pass 88 # skipped 2`, Chromium `# tests 90 # pass 85 # skipped 5` (+1: the
  1024 drawer case; the same skips). On a site built `--engine`: Firefox `# tests 63 # pass 62 # skipped 1` (THE REAL
  ENGINE passed); WG3 (`page/engine.test.mjs`) 1 of 1 in Firefox and in Chromium. No browser process left.
- ARM header gate: `TU includes 259 headers` → `ARM compile gate: CLEAN`. CI greps: PROS-free, sim-free, the route
  builder's files LF. `git diff -- include/` is empty.
- `ctest -R compile_refusal`: `100% tests passed, 0 tests failed out of 353`.
- `api_doc_tool`: self-test OK, check-coverage as above, check-fresh exit 0, check-examples `988 quoted lines, all
  verbatim`, check-removability `no public doc references docs/internal/`. `doc_staleness_audit.py`: self-test `OK
  (16 …)`, then `clean (37 live docs, 0 numeric claims checked)`.
- src gate `check`: `PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly their owed facts
  …)`. `self-test` (`timeout -s INT`, in its copy): `OK (398 detector cases, each against a real build)`. After it:
  no `(self-test plant` in `git diff -- Makefile src/`; untracked under src/include/test only GL20–GL23 and
  `test/mirrored_pair_numbers.hpp`.
- Not run here: the Windows/Linux CI (the coordinator's). Nothing committed.

## RMP1-E — the mechanisms in PREVIEW (the cascade rising, the claw opening)

*Executor: Claude Opus 5.5 (a subagent of the coordinator), worktree `.claude/worktrees/rmp1-e` (branch `rmp1-e`, from
`5e2c7f1`), started 2026-11-19. Brief: `docs/internal/chunks/RMP1-E-mechanisms-in-preview.md`. Host and browser only.*

### Log
- Started: read RESUMING's standing rules, the RMP1-E brief in full. Next: the engine's run record and the page's
  playback.
- THE ENGINE (D1): `test/route_maker_runner.hpp` — `MechTap`, a pacer below RPV1's recorder (through
  `onBuilt`'s inner slot, the hook RPV1 made for a tap), reads each slot every pace when the recorder does: a lift's or
  arm's `position()` (the library's own reading, in its units), a pneumatic's `commandedPosition()` (-1: none), a power
  mechanism's commanded volts' sign, a flywheel's target's sign. `mechanismsOf` writes the record's new `mechanisms`
  member (name, kind, a lift's/arm's units, bottom, top; a pneumatic's positions; RPV1's `placeholderFor` sentence; `at`
  aligned with `samples`). `runEntry` (the tap installed) is used by `preview` and by WG1's compiled twin
  (`route_maker_goldens.cpp`), so the two records stay byte-equal. `preview` split into `runParsed` + `recordOf` so
  the suite can run it. RPV1's `route_preview_run.hpp` untouched. Two suite cases (`route_maker_runner_test.cpp`): the
  brief's route on robot one, and a lift stalled by the plant's obstacle at 3 in. Both green. (The end action puts the
  claw back to its safe `closed`: the record shows it, so the case reads the claw at the route's end, not the run's.)
- Node, the engine: `test/wasm/engine.mjs`'s `compareRecords` compares `mechanisms` (a lift's/arm's states
  within POS_BOUND, the rest exact); a native case in `native.test.mjs` (the brief's route: heights, the claw on the
  pace after its step began, the toggle backward, the list in robot one's order). The module rebuilt (pinned emsdk
  read-only, `EM_FROZEN_CACHE=1`, `TMPDIR=build/tmp`): `test/wasm/` 16 of 16 (WG1 byte-equal with the twin's new member;
  WG2 23 routes, WG2s 500 routes). Fixtures re-recorded (`record.mjs`): only `owed` (robot one's facts typed since
  11-17), `mechanisms` and `madeBy` changed; `mechanisms.preview.json` new (the brief's route); the two PAIR fixtures
  stay without the member (an older record: no strip). Page: `mechanismsAt` (playback.mjs), `mech_strip.mjs`, wired in
  `preview.mjs` (the target is lift.mjs's PLANNED height of the step playing: the number the export writes), one line
  in About this run (`timeline.mjs`), CSS in ONE block `/* ── PREVIEW mechanisms (RMP1-E) ── */`.
- Page tests: `test/page/mechanisms.page.test.mjs` (new; the stand-in with the `mechanisms` fixture): the bar
  IS the record's height scrubbed (7 clicks on the track), stepped (Right, Shift+Right) and played (`frameAfter`); the
  claw closed until its step, open on the pace after, the piston drawn out; ONE accented per mechanism step, none at
  the end; the cascade's target marked only while its step plays (`to 0/6/4/0 in`); the toggle `backward`, only one
  motor running; "Robot one's mechanisms", the cascade's PLACEHOLDER, About this run's line; the field, the zoom and
  the dock clear of the strip; Esc puts it away, PREVIEW brings it back; a pre-RMP1-E record (pair_ok) shows none;
  1440 and 1280 (light and dark with SHOTS) with every mechanism in view unscrolled. Firefox, the whole page folder
  on the stand-in site: `# tests 68 # pass 66 # skipped 2` (the engine's two). First 1280 x 800 run: the strip
  scrolled (heading on two lines, 168 px lift): the heading made one line, the rows tighter, the lift 120 px least —
  re-measured: 406 of 406 px. `preview.test.mjs`'s THE REAL ENGINE compares `mechanisms` (lift/arm within 1e-5).
- Docs: guide 22 step 6 (one paragraph + `preview-mechanisms.png`), `docs/route-builder.md` (one clause), the
  changelog (2026-11-19 under API 2.9), `tools/route_maker/README.md` (the record's `mechanisms` row, `mech_strip.mjs`,
  the native/module bound's sentence twice). Gates so far: ARM header gate `TU includes 259 headers` → CLEAN; CI greps
  (PROS-free, sim-free, changed files LF) clean; `git diff -- include/ src/ Makefile` empty.
- Mutations (`.claude/coordinator/scratch/rmp1-e/mutate.py`: each file restored in a `finally` and on SIGTERM/SIGHUP/
  SIGINT, its timestamp bumped; the dependents rebuilt each time — the suite with `route_maker_runner_test.cpp`
  touched, the native program with `route_maker_main.cpp` touched, the site by `build.py`). Every one observed RED:
  - E1 the record takes the tap's FIRST n states → native `RMP1-E` RED ("up 6 in: 0 vs 6").
  - E2 a power mechanism's direction lost → suite RED (`stateAt(m, 1, r0 + 1) == -1.0`), native RED.
  - E3 a pneumatic's position never read → suite RED (`stateAt(m, 3, c0 + 1) == 1.0`), native RED.
  - E4 a lift records its target, not where it is → suite RED (the stall case: `std::abs(end - blockedAt) <= ...`).
  - E5 the states written under another mechanism's name → native RED.
  - P1 the strip a pace ahead → playback RED, page RED (2 cases). P2 every mechanism accented → playback RED, page RED.
    P3 the strip moves only while playing → page RED (2 cases). P5 a piston's words never change → playback RED, page
    RED. P6 the field under the strip (its width rule gone) → page RED ("the field and the zoom left of the strip").
    P7 a backward toggle said forward → playback RED, page RED.
  - P4 a target marked on a lift whose step is not playing: the first run NOT caught (robot one has ONE lift, so the
    mutant is the same on its page). A case added to playback.test.mjs (robot x's arm and lift: the arm's step marks
    the arm's target, never the lift's) → re-run: playback RED; the page stays green (one lift: equivalent there).
  - First pass: E2–E4 "BUILD BROKE" — not a mutant's fault: the doc gates (a dependency of `shulib_tests`) re-ran on
    my docs edits and compared the briefing's suite line, generated while unbuilt, with the built binary. Fixed by
    the rule's sequence (`rm -f build/test/shulib_tests`, generate, full build, generate, check); E2–E4 re-run, RED.
  - After: no mutant text in any file (grep), the suite and the native program rebuilt from the restored tree, the
    RMP1-E cases green again.

### Deltas from the brief (each with its reason)
*(Written after the review, which found them only in the builder's report — review finding D1 below.)*
1. **The bar's target is the page's PLAN** (`lift.mjs`'s `plannedHeights`, the step playing), not a library reading.
   D1's "never the page's guess" is kept for every STATE (height, position, motor): those are the library's. The target
   is what the route asked for — the number the export writes for that step; the slot's own `target()` is its move's
   or hold's aim (during a homing, where the push began: `holdAt(jointDeg())`, `service.hpp`, not the height the homing
   finds). Since the review it is the plan of the
   route that RAN (`run()` keeps it), never the route on screen.
2. **Flywheels recorded and drawn** (a motor chip), though D1/D2 name lifts, arms, pneumatics and power mechanisms:
   the record lists every mechanism in the robot's list, in its order, so the strip's rows are the list. No route
   step can spin a flywheel yet (the Mechanism menu: "Flywheel steps come later"), so its chip reads off; the tap's
   reading of one is tested in the suite (review finding T4).
3. **The PLACEHOLDER chip is RPV1's `placeholderFor`** — D3's "existing PLACEHOLDER rule" itself: the same sentence the
   steps carry (a mechanism on the library's starting gains, HA-182).
4. **The field narrows while the strip shows** (D5: "nothing else changes: the drive's playback"): D2 puts the strip
   BESIDE the field, so the field's drawing takes the pane less the strip; the playback itself (poses, path, timeline,
   steps) is unchanged.
5. **Robot one's facts changed under the fixtures:** the re-recorded fixtures' `owed` lists differ by facts typed since
   11-17 (another chunk's), not by anything RMP1-E did.
6. **The PAIR fixtures were the older-record case** (recorded before the member). Since the review the tests make
   their own older record (the stand-in's `omit`, a copy with the member deleted), so re-recording them as the README
   asks removes nothing.
7. **Readings from the library's own slots below RPV1's recorder,** through RPV1's `onBuilt` hook: D1's "from the
   library's simulated mechanisms", with `route_preview_run.hpp` untouched.

### Gates on the final tree (2026-11-19, `date` 21:00–21:15 EST)
- Build: `rm -f build/test/shulib_tests`, `briefing_status.py generate` (unbuilt), `cmake --build build/test -j8` →
  exit 0; doc gates (re-run with guide 22 touched after its last edit): `doc coverage: 6502 public entities across 215
  headers, all documented`, `check-nav: OK — 263 public pages`, `check-tone: OK — 48 pages`, `first_week: OK`,
  `build_to_program: OK`, `check-register: OK — 232 register rows`. Briefing regenerated on the built binary;
  `briefing_status.py check`: `generated block current`.
- Suite (`suite_run.py green`): `suite run: green — 3882 cases, 3775499 assertions`.
- Node: `node --test tools/route_maker/test/*.test.mjs` → `# tests 174 # pass 174` (173 + the RMP1-E case).
- The engine rebuilt from the final tree (emsdk read-only, its git tree clean after): `test/wasm/` under
  `MemoryMax=6G`: `# tests 16 # pass 16` (WG2 23 routes, largest 5.33e-14 in; WG2s 500 routes, 2.65e-7 in).
- Fixtures: a fresh `record.mjs --out` equals every committed one (result and project).
- Page, the stand-in site: Firefox `# tests 68 # pass 66 # skipped 2`, Chromium `# tests 68 # pass 63 # skipped 5`
  (the same skips as before). The `--engine` site (`preview.test.mjs` + `engine.test.mjs` +
  `mechanisms.page.test.mjs`): Firefox 28 of 28, Chromium 28 of 28 (THE REAL ENGINE and WG3 in both); THE REAL ENGINE
  also through CI's path (`ROUTE_MAKER_NATIVE_RECORDS`, no native program): pass. No browser process of this run left.
- Screens (`~/shulib-render-tmp/rmp1-e/shots/`, Firefox, at rest): the strip light/dark at 1440 and 1280, the cascade
  mid-move and the claw opening. Guide 22's `preview-mechanisms.png` from the light 1440 one by RMP1-A2u's `to_docs.py`
  (1200 x 750, 256 colours).
- ARM header gate `TU includes 259 headers` → CLEAN; CI greps clean; `git diff -- include/ src/ Makefile` empty.
- `api_doc_tool`: self-test OK, check-coverage as above, check-fresh exit 0, check-examples `988 quoted lines, all
  verbatim`, check-removability `no public doc references docs/internal/`. `doc_staleness_audit.py`: self-test OK (16),
  then `clean (37 live docs, 0 numeric claims checked)` (its note: the binary is older than `route_maker_main.cpp`,
  touched for the native program's last rebuild, which `shulib_tests` does not compile).
- src gate `check`: `PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly their owed facts
  ...)`. Its self-test not run (no src/ or Makefile change).

**Prose review (rule 4; greps run and hits read BEFORE this line, 20:53 and 21:07).** `route_maker_runner|
route_maker_goldens|playback\.mjs|preview\.mjs|timeline\.mjs|mech_strip|style\.css|record\.mjs|compareRecords|schema
1|runEntry|recordOf|runParsed` over docs/ (internal and api aside), README.md and the route builder's README: only the
README's hits (its table rows, the engine paragraph — `route_preview_run.hpp`, "unchanged", still true). Concepts
(`mechanism.*beside|the strip|About this run|from above|in step with|What the host test plant is not|within 1e-5|
re-record|native twin`): fixed the README's "every member exact, poses within 1e-5" (now with a lift's or arm's
position) and the WG2 sentence; guide 22 :157–162 (the plant is not the robot) read and true; guide 14 :66–75 and
:562–568 read and true (PREVIEW's limits; it never said mechanisms are not shown); every "strip" hit elsewhere is the
GPS strip. `preview|beside the field|robot from above` over where-your-code-goes, how-the-docs-stay-true,
hardware-assumptions, docs/README, README: none. Deliberately NOT touched: `roadmap.md`, `build-order.md` (the
coordinator's), `docs/api/` (no `///` changed), `ci.yml` (it runs `page/*.test.mjs` and `record.mjs --out`, which
pick up the new test and fixture).

**For the coordinator.**
- No file owned by RMP1-D or RMP1-F touched (`controls_tab.mjs`, `feel_tab.mjs`, `export_drawer.mjs`,
  `src/host/files.mjs`, `tools/upload_auton.py`); CSS in ONE block at the end of `style.css`
  (`/* ── PREVIEW mechanisms (RMP1-E) ── */`). Likely merge touch points: `docs/changelog.md` (a new 2026-11-19 entry at
  the top of API 2.9), guide 22, `tools/route_maker/README.md`, `style.css`'s end, and
  `docs/internal/PROJECT-BRIEFING.md` (regenerated from this base: its suite line will conflict with main's 06c32d3 —
  take main's, then regenerate on the merged tree after `rm -f build/test/shulib_tests` and a fresh build).
- The record grew about 19 % (the 60 s `cut` fixture 360 → 429 KB).
- Robot two (D4): PREVIEW plays robot one only today; the strip is robot one's and says "Robot one's mechanisms".
  When the pair mode lands, `mechanismsOf` already writes robot two's own list for its record.
- Not run here: the Windows/Linux CI. Nothing committed.

### RMP1-E review findings and their disposition *(2026-11-19, the builder again, 21:40–22:40 EST)*
Three reviewers (correctness, tests, docs), 17 findings; two pairs overlap (C3 = T1, the pace lag; the plan's
target is C1). Each re-checked in the tree before acting. Mutations by `scratch/rmp1-e/review-fix/mutate2.py`
(`mutate.py`'s pattern: restored in a `finally` and on SIGTERM/SIGHUP/SIGINT, timestamp bumped, dependents rebuilt —
the suite with `route_maker_runner_test.cpp` touched, the native program with `route_maker_main.cpp` touched, the
site by `build.py`); logs `eng-mut.log`, `page-mut.log`, `page-mut2.log` beside it. Every one observed RED.
- **C1 (major) — the bar's target from the route on screen, not the run's: FIXED at the root.** Re-checked: `open()`
  planned `app.route()`. Now `run()` keeps each step's `{name, target}` from the route that RAN (`plannedHeights(r,
  one)`), cached with the run; `playingMech` reads only that. Test: `mechanisms.page.test.mjs` "the targets are the
  plan of the route that RAN" (stand-in `delayMs: 2500`, step 1 deleted while running, the run opens stale, Ctrl+Z,
  the four targets `to 0/6/4/0 in`). F1 (the old code) RED in exactly that test.
- **C2 — a short window scrolls the strip and hid the accented row: FIXED.** `mech_strip.mjs` keeps the accented row
  in the strip's view when the accent changes (the strip's own `scrollTop`, never the page's; the first row brings
  the heading with it). Test at 1366 x 657 (the review's laptop case): every mechanism step's row in view, and the
  cascade brought back after the strip is scrolled to its end by hand. F2 RED. Not done: one-line piston and motor
  rows ("right toggle" + "backward" does not fit one 176 px line); the cascade's bar is short there but its number
  is in view.
- **C3 = T1 (major) — a one-pace lag (R1) and whole-inch rounding (R3) passed every gate; the native test's "BUG
  CAUGHT: a pace off" was false: FIXED.** The suite's RMP1-E case puts a pacer below the tap (through `runEntry`'s
  `then`, which is handed `tap.inner`) that reads the cascade's `position()` after each step of the world; the tap's
  column must equal it at EVERY pace (and the cascade must move at > 100 paces, so the check is not of a height
  standing still). R1, R3, E4 RED in the suite (`off == 0U`; E4 also in the stall case). The native comment now
  claims only what it catches.
- **C4 — a lift started by a start form ("keeps going") is never accented: KEPT, said in guide 22.** D2's rule is the
  step playing; a start form takes no time, so it never plays (`stepAt`), and the bar still moves truthfully through
  the drive. One sentence added to guide 22 step 6. Accenting the mechanism a start form began until its
  `waitUntilDone` changes D2's rule — the coordinator's call (after the 21st, if wanted).
- **T2 — the real-engine page check nearly empty: FIXED.** `preview.test.mjs` THE REAL ENGINE runs `ready` AND
  `mechanisms` (the cascade moves in the second), asserts each mechanism's `at` length before the per-pace loop.
  `native.test.mjs`'s RMP1-E case now compares its fresh record with the committed `mechanisms.preview.json` (steps
  and sample count exact, every state within POS_BOUND): CI's engine job runs it, so a stale committed fixture goes
  red there. R1, R3, E4, R7 RED in it.
- **T3 — no engine test of D3's PLACEHOLDER mark (R7): FIXED.** Native: the cascade's `placeholder` matches
  `^PLACEHOLDER`, the right toggle's is "". R7 RED.
- **T4 — flywheel, arm and no-position paths unexercised: FIXED.** Suite case "the tap reads a flywheel's
  direction, and a piston at none of its positions as -1" (`mech_rig::Rig` on robot x's list, line C refused so the
  hood is ABSENT): R4, R5 RED. Native case "RMP1-E on robot x" (kinds and units, the arm in degrees at its `score`,
  the hood's middle, the intake forward, the flywheel off throughout — no route step can spin one): R6 (an arm
  labelled a lift, in inches; the review's R9 is the same assert) RED.
- **T5 — page mutants Q1–Q5: FIXED but Q4.** Q1 (no clamp): a playback case past both stops, RED. Q2 (About's
  mechanisms line for a record with none): the older-record test opens About and finds none, RED. Q3 (half-out
  rod): the claw's rod 4 closed, 18 open, RED. Q5 (the accent lingering): the `ready` route accents cascade, none,
  none, claw, none at its five steps, RED. Q4 (rows never rebuilt) stays EQUIVALENT today: robots one and two have
  the same list and PREVIEW plays robot one only, so no page shows two records with different lists.
- **D1 (major) — the deltas only in the report: FIXED** ("Deltas from the brief" above, with reasons).
- **D2 — guide 22's PREVIEW pictures without the strip: RE-MADE.** `preview.png`, `preview-failed.png` and
  `preview-mechanisms.png` from a SHOTS-only test in `mechanisms.page.test.mjs` (light 1440, Firefox, robot two's
  question answered "one at a time": no red box, no error badge; toasts gone), by RMP1-A2u's `to_docs.py`; alt texts
  say what the strip shows. Shots: `~/shulib-render-tmp/rmp1-e/shots-review/` (11, the 1366 x 657 one included).
- **D3 — "the first line of About this run": FIXED** ("About this run says so"). My concept grep missed it: the
  guide wraps `**About` / `this run**` across a line break. This round's check is a multi-line regex
  (`About\**\s+\**this run`) over docs/ and both READMEs: two hits, both read, both true.
- **D4 — the README's "re-record the PAIR fixtures" vs two tests that needed them unre-recorded: FIXED at the
  tests.** `playback.test.mjs` deletes the member from a copy of `mechanisms`; the page test asks the stand-in to
  leave it out (`standin_engine.mjs`'s new `omit`; the README's stand-in sentence names it). Re-recording the PAIR
  fixtures now removes nothing.
- **D5 — the strip's PLACEHOLDER chip not in guide 22's PLACEHOLDER sentence: FIXED** (one sentence, linking
  chapter 18's Tune program).
- **D6 — two banners: FIXED** (`route_maker_runner_test.cpp`: whole routes on robot one since RMP1-E;
  `route_maker_runner.hpp`: `onBuilt` installs the tap, `runEntry`).
- **D7 — the README's bound in inches for an arm, a trig-only cause: FIXED** (an arm's 1e-5 deg; "round differently
  in the last bit").
- **D8 — PROJECT-BRIEFING.md missing from the merge touch points: FIXED** (added, with the regenerate sequence).
- Also: the stand-in registers a route with "both robots at the same time?" answered EITHER way (robot one's run is
  the same), so the docs pictures can answer "one at a time".

**Prose review (rule 4; greps run and hits read BEFORE this line, 21:49, the multi-line regex just after).** Names (`standin_engine|
mech_strip|mechanisms\.page|route_maker_runner_test|preview\.mjs|native\.test`) over docs/ (chunks and api aside),
README.md and the route builder's README: only the README's table rows and its stand-in paragraph (now naming
`omit`). Concepts (`first line of|stand-in.s options|Its options|within 1e-5|1e-5 in|starting gains|keeps going`,
and the multi-line "About this run" regex): the README's two bound sentences (fixed), guide 22's About sentences
(fixed and read), every other "starting gains" hit (guide 10, 14, roadmap) about the library, not the page.
Deliberately NOT touched: the changelog (its entry still true), `docs/route-builder.md` (one clause, still true),
`roadmap.md`, `build-order.md`, `docs/api/` (no `///`), `ci.yml` (it already runs every new test).

**Gates on the post-review tree.** Full build after `rm -f build/test/shulib_tests` and the unbuilt briefing: exit 0,
doc gates as before (`6502 public entities ... all documented`, `check-nav: OK — 263`, `check-tone: OK — 48`,
`check-register: OK — 232`); briefing regenerated on the built binary, `check`: current. Suite: `green — 3883
cases, 3775506 assertions`. Node: 174 of 174. The module rebuilt (pinned emsdk read-only): `test/wasm/` 17 of 17
(WG2 23 routes 5.33e-14 in; WG2s 500 routes 2.65e-7 in). Fresh `record.mjs --out` vs every committed fixture:
all 8 "same". Page, stand-in site: Firefox 72 / 69 pass / 3 skipped, Chromium 72 / 66 / 6 (the engine's two or five
and the SHOTS-only pictures). `--engine` site (preview + engine + mechanisms): Firefox and Chromium 32 / 31 / 1
(the pictures); THE REAL ENGINE through CI's path (`ROUTE_MAKER_NATIVE_RECORDS`, no program): pass. ARM header gate
`TU includes 259 headers` → CLEAN; PROS-free and sim-free greps clean; changed files LF; `git diff -- include/ src/
Makefile` empty. `api_doc_tool` self-test, check-coverage, check-fresh (exit 0), check-examples (`988 quoted
lines, all verbatim`), check-removability: OK. `doc_staleness_audit.py` self-test OK (16), run clean (37 live docs).
src gate `check`: `PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly their owed facts
...)` (its self-test not needed: no src/ or Makefile change). No browser process of this round left. Nothing committed.

## RMP1-D — the Controls tab, one thing at a time (layout and words only)

- **Started 2026-11-19** by an executor (Opus 5.5) in the `rmp1-d` worktree, base main `8b2d99e`. Brief:
  `RMP1-D-controls-tab-layout.md` (the coordinator's, uncommitted, lands with this run). The team lead: "THAT IS AN
  INFORMATION OVERLOAD" / "the formatting of the controls tab needs to be worked on". Scratch:
  `.claude/coordinator/scratch/rmp1-d/`. Host and browser only; no pros, no /dev/ttyACM*.
- **Added to the run by the coordinator (2026-11-19), found while recording the Controls tab's tutorial on the built
  site** — each to be fixed with a page test seen RED first: (1) the feel graph's dot stops dragging after the first
  drag (the graph's text gets selected; the browser starts its own drag); (2) changing a feel number (Curve 0.6 +
  Enter) scrolls the column back to the top, losing the graph; (3) giving a free button a job (robot one, Up) raises
  the "stale map" error and asks for "Remove that line" — it must succeed in one action, the export byte-for-byte the
  same for the same controls.
- **Baseline (base 8b2d99e, before any edit):** host build OK; node 156/156; Firefox `controls.page.test.mjs` 15/15.
- **Built:** `controls_tab.mjs` — the tab is a two-step switch (`1 · Buttons` / `2 · Driving feel`, role tablist, arrow
  keys) with the robot switch beside it; only one step is drawn. Step 1: the controller re-drawn on a 1000 × 462 viewBox
  (one label slot per button, two-line labels "R1 · cascade / up (hold)", 17/16 user units, pills measured by `getBBox`,
  leaders under the buttons; the selected button and its label the only accent) and ONE editor beside it: nothing
  chosen → "Give a button a job" with the mechanisms as a compact list, the no-button jobs as plain sentences, the
  ready banner and the source words; a free button → its mechanisms as large buttons, then that mechanism's jobs (the
  old popover's items, same words and accessible names); a used button → its keycaps, the mechanism, the job's words,
  the existing form ("Its words in the file" folded), "Make R1 and R2 free"; Esc / Close closes, the keyboard back on
  the button. `feel_tab.mjs` — the style as three cards, the axis's Dead zone and Curve beside the graph, everything
  else under "More settings" (closed). CSS in ONE block `/* ── Controls tab (RMP1-D) ── */ … /* ── end Controls tab ── */`;
  RMP1-B's old Controls layout rules (three-column grid, the mechanism cards, the popover groups, `.seg.wide`, the
  ≤1100 px drawers) removed from its block. `help.mjs` :47 one phrase ("is below it" → "is its step 2").
- **Found by Chromium (and fixed):** "More settings" closed itself when a typed number re-drew the feel right after
  it was opened — the open state was recorded on the `toggle` event, which comes a task later. Now recorded on the
  summary's click (a deterministic test re-draws inside the same task).
- **After:** node 156/156 (unchanged: no model or export file touched); Firefox controls page 21/21; Chromium 20 + 1 skipped
  (the download way is Firefox's).
- **Tests added (`test/page/controls.page.test.mjs`, Firefox + Chromium):** (1) two steps, ONE drawn at a time — click,
  arrow keys, a stick opens step 2, `aria-labelledby` follows; (2–4) at 1280 × 800, 1366 × 768 and 1440 × 900: all 14
  labels drawn at ≥ 13 px ON SCREEN (computed size × the drawing's scale), each label's words inside its pill and its
  pill inside the drawing, no two labels overlapping, and NOTHING CUT OFF (no element clipping its content or scrolling
  sideways, the tab ending above the status bar) with nothing chosen, the cascade's form, its words open, the feel, More
  open; the feel's essentials fit with no scroll; the last setting scrolls into view; (5) a click (or a label) opens ONE
  editor with the job now; the selected button and its label the ONLY accent-painted shapes; Make X free → give X the
  claw → drawn at once → two Ctrl+Z bring "ramp: toggle" back; Enter on a free button puts the keyboard on its first
  choice, Esc closes and returns it; (6) "More settings" closed at first, every setting under it hidden then reachable
  and enabled, the axis picked there titles the essentials, and a re-draw in the same task keeps it open. The old
  tests re-pointed (the popover → the editor's two-stage choices; `.pane.right` → `.ctl-editor`; feel settings via
  step 2). Screenshots: light/dark × 1440/1280 (`scratch/rmp1-d/shots/`); guide 22's five Controls pictures re-taken
  from the Firefox light-1440 ones (`controls-menu.png` is now the claw's three choices; the popover is gone).
- **Mutations — 15, every one CAUGHT on a FRESH site build, observed red for its own reason, restored with its
  timestamp bumped (`scratch/rmp1-d/mutate.py`, logs `mutlogs/`, `mutations.json`):** M1 a label's job line 12 units
  (10.06 px at 1280); M2 Right's label slot onto Left's (overlap); M3 the viewBox 1300 wide (10.31 px); M4 used buttons
  accent-filled ("no accent" fails); M5 the controller mounted under the feel (one step); M6 More open at first; M7 Esc
  does nothing; M8 an export byte (`_volt` → `_volts` in `controls.mjs`) — the node goldens red, 5 tests ("…is stale");
  M9 the editor `overflow: hidden` (clips at 1366); M10 a chosen job not committed; M11 the graph column 600 px (the
  essentials 737 > 603 at 1366 × 768); M12 More's state only on `toggle` (closes on a same-task re-draw); M13 the label
  not marked selected; M14 the pills measured 44 units short (words outside the pill); M15 Enter leaves the keyboard on
  the button.
- **The export is unchanged:** no model, export or driver-file module touched (`git diff -- tools/route_maker/src/model
  src/ include/` empty); the node goldens 156/156; M8 proves they see one byte.
- **Docs:** guide 22 — the Controls part's opening (two steps; the robot pick "at the right of the steps"; the date in
  the panel), "Step 1 · Buttons" (was "What each button does"), "Step 2 · Driving feel" (essentials, then More
  settings); the changelog — one bullet under the 2026-11-16 Controls entry; `tools/route_maker/README.md` row 13;
  `help.mjs` :47. NOT touched: `docs/route-builder.md` :16 (still true), guide 18 / 21 / cookbook 06 (no layout words),
  `docs/api/` (no `///` changed), `mkdocs.yml`.
- **PROSE REVIEW (rule 4), run 18:19, BEFORE this line:** `grep -rn -i -E "controls_tab|feel_tab|popover|Remove this
  control|With no button|short menu|menu lists|above the list on the left|the control on the left|under a mechanism|below
  the controller|Under the controller|the date is on the right|feel is below" docs README.md tools/route_maker/README.md
  test/README.md --include=*.md` (internal and api aside) → guide 22 :74 (the Route tab's Mechanism menu: true), :213
  and :231 (this run's / true), the README :13 (this run's). Also read: `docs/route-builder.md` :16, the changelog :59–77,
  cookbook 06 :282, `test/README.md` :102, `RMP1-A1-manual-checks.md` (no Controls step) — all true.
- **FINAL TESTS AND GATES (18:00–, on the final tree; `scratch/rmp1-d/gates.sh`, logs `gates/`):** the site re-built
  from the restored sources; Firefox Controls page **21/21**, Chromium **20 + 1 skipped** (the download way is
  Firefox's), A1's `page.test.mjs` (Firefox) **18/18**, node **156/156**. Gates: the suite binary unlinked, configure,
  briefing generated, host build **0 warnings** (no C++ changed), briefing generated again — **no tracked change**; suite
  green **3,856 cases / 3,886,386 assertions** (`suite_run.py green`, 4 GiB cap); briefing check and self-test;
  api_doc_tool self-test / coverage (6,485 entities, 215 headers) / fresh / examples / removability; staleness
  self-test and audit; doc_readability check-nav / check-tone / self-test; the PROS-free and sim-free greps clean; the
  ARM header gate exit 0 (260 lines: 259 headers + main).
  The src gate self-test **OK (398 detector cases)** and check **PASS (14 builds)** (`timeout -s INT 7000`, `nice`);
  after both, `git diff -- Makefile src/ include/` EMPTY and no untracked file under `src include test`. No browser,
  driver or test process left.
- **Deltas from the brief:** (1) a free button's choices come in TWO short steps — its mechanism, then that
  mechanism's jobs (each a large button; "Another mechanism" goes back) — not one list of ~16; one idea at a time.
  (2) A used button's editor offers "Make R1 free" as its large-button choice; changing to another job stays the
  existing Mechanism / What it does fields (no second list repeating them). (3) The axis picker sits INSIDE More
  settings ("The dead zone and curve above shape"), and the essentials' heading names the axis shown. (4) "Its words in
  the file" folded too (open by itself when a word there is stale). (5) Extra docs: one changelog bullet, the route
  builder README row, `help.mjs` :47. (6) RMP1-B's old Controls rules REMOVED from its block (not left dead beside the
  new block): `.controls`/`.controls-mid` grid, `.seg.wide`, the controller (SVG) group, `.mechs`/`.mech-card`/`.mech-head`,
  `.menu-group*`/`.mech-chip`, the ≤1100 px `.controls .pane` drawers — the new rules are all in the RMP1-D block.
- **For the coordinator's merge:** `help.mjs` and `style.css` are RMP1-C's files too — one phrase in `help.mjs` :47;
  in `style.css` the RMP1-B block lost the rules above (hunks around old :385–435 and the last `@media`) and the new
  block is appended at the end. `app.mjs` NOT touched. The tutorial video (`tools/docs_video/controls_tab/`) was
  recorded on the old layout: the Controls tab's role/name and every controller button's accessible name are
  unchanged, but the popover, the left Mechanisms column and the right Control pane are gone — it needs re-recording.
  **Amended in the fix round:** `.mech-head` and `.seg.wide` stay (RMP1-C uses both). RMP1-C's tree still has RMP1-B's
  OLD controller rules (rmp1-c `style.css` :415–430: `.callout { pointer-events: none }`, `.callout text {
  text-anchor: middle }`, `.callout.used rect` accent, `.cbtn*`, `.pad*`, the `.controls` grid): after the merge NONE
  may survive outside the RMP1-D block — kept, they stop label clicks, pull the words out of their pills and accent
  every used label. Re-run `controls.page.test.mjs` (Firefox and Chromium) on the merged site.

### RMP1-D review findings and their disposition (the fix round, started 2026-11-19 19:50, the builder again, Opus 5.5)

- Three reviewers (correctness, tests, docs) returned 19 findings; overlaps merged below (A–P). Each re-checked on this
  tree before its fix. **Interrupted once:** this round's first pass (19:50–19:57: the code and test edits, a copy of
  the reviewed `controls_tab.mjs` / `feel_tab.mjs` / `style.css` / page test in `scratch/rmp1-d/prefix/`, byte-equal
  to the 18:19 mutation backups) stopped with no log line and no process left; it RESUMED 20:02, kept those edits,
  re-checked each and wrote everything below.
- **How each was proven (`scratch/rmp1-d/mutate_fix.py`, logs `fixlogs/`, verdicts `fixmut*.json`):** every entry
  applies its edits, rebuilds the site, checks the mutant is LIVE in the built site, runs the WHOLE Controls page file
  (Firefox under `ulimit -v` 4 GiB + timeout; Chromium in a 6 GiB cgroup), and restores in `finally` / on
  SIGTERM-SIGHUP with the timestamp bumped. **RED first:** `PREFIX` puts the reviewed tree's three sources back under
  the NEW tests — Chromium 9 fail, Firefox 11–13 fail (both Firefox runs then lost the browser: each failed test
  leaves its page open, and the 4 GiB cap ends Firefox; so `PREFIX2` re-ran the four RMP1-D editor/feel tests alone:
  21, 23, 24 RED, 22 green as expected — its fix was already in the reviewed tree, R11 proves its test).
- **A — the three items the coordinator added (correctness 1, tests 8, docs 15).**
  (3) **Giving a free button a job raised the stale-map error — FIXED.** `controls_tab.mjs` `edit()`: a "free" map
  line that THIS edit makes lie is removed in the same edit (one undo entry), and the toast says so ("The button map's
  "d-pad" line said free, so it went too (Ctrl+Z undoes both)."). Only lines this edit makes stale: a line already
  stale in a file or saved project keeps its error and **Remove that line**. No model change (it calls the model's
  `checkMap` and `removeFreeLine`, as **Remove that line** does), so D5 holds. The export is byte-for-byte the two-step
  result: the export test now builds that result with the model (`addControl`, then `removeFreeLine`) and compares the
  three written parts with it. Tests: test 2 re-pointed (0 errors, `banner.free` empty, the toast, ONE Ctrl+Z brings
  the job and the line back, redo), the export test, and the saved-project test (another edit leaves a loaded stale
  line). RED on the reviewed tree (test 2: `[mapStale]`; test 4: "one action"). Mutants X1 (nothing removed) and X1b
  (EVERY stale line removed, not only this edit's) CAUGHT.
  (1) **The graph's dot stopped after its words were selected — FIXED** by `.graph { user-select: none }` (RMP1-D
  block) and a `pointercancel` that lets the dot go. Test: four drags, each ends at its target ± 1 %, nothing
  selected; a cancelled drag leaves the dot. RED on the reviewed tree in both browsers (Chromium: "-100 -100 -50 …"
  selected, Firefox: " % (pushed back ← → pushed forw"). Playwright's mouse never made the dot itself stop (the
  reviewers saw the same); the selection — the cause the coordinator named — is what turns red. The first pass also
  added `preventDefault()` on pointerdown: X9a / X9b showed each alone passes, so it was REMOVED (one mechanism; the
  CSS keeps a click focusing the dot). X9 / X9c (the rule gone) CAUGHT in both browsers; X12 (no `pointercancel`)
  CAUGHT.
  (2) **A typed feel number scrolled the step to the top — already fixed** by the reviewed tree's mounted-once
  `#ctl-body`. Test added (More open, scrolled to the end at 1280 × 800, Curve 0.6 + Enter, `scrollTop` unchanged). It
  cannot be RED on the reviewed tree (fixed there) nor on base `8b2d99e` (no `#ctl-body`, no More settings); X10
  re-makes the step's scroller on every draw (the old cause) — CAUGHT in both browsers (64 → 0).
- **B — Export's "Go there" on a stale map line opened "Up is free" (correctness 2) — FIXED.** `selAt()`: a selected
  button that has a job opens that job's form (its stale-map box and **Remove that line** in it). New test: a saved
  project with the stale line → Export → Go there → the claw's form, no mechanism offered, Up selected, Remove that
  line → 0 errors. RED on the reviewed tree ("Up is free"); X2 CAUGHT.
- **C — Esc after a mouse click on a label, a job or a mechanism (correctness 3) — FIXED.** Esc is heard on the
  document while step 1 is shown, a button is chosen, no box is typed in and no drawer / dialog / popover is open.
  Enter on a USED button now puts the keyboard on its first field. Tests: label click + Esc, a job chosen by mouse +
  Esc, a mechanism row + Esc, Esc over the Export drawer closes only the drawer, Enter on A → its Mechanism field, Esc →
  back on A. RED on the reviewed tree (21: "ramp" stays; 1: focus not on Mechanism). X3 (back on the tab) CAUGHT in
  both browsers; X3b (no drawer guard) SURVIVED the first set, so the drawer case was added — then CAUGHT; X4 CAUGHT.
- **D — a chosen AND refused button lost its red (correctness 4, docs 16) — FIXED.** `.cbtn.sel.bad`,
  `.callout.sel.bad` (pill and words) and `.leader.sel.bad` keep `--failed`; the fill stays the accent. Test 2 reads
  B's outline, its pill, its words and its leader as `--failed`. X5, X6, X6b, X6c CAUGHT. Guide 22: the caption is now
  "B and its label turn red"; :204 now "the only one highlighted on the controller (a button with a problem stays
  red)"; `controls-refused.png` re-taken (B red-ringed, its label red).
- **E — labels under 13 px from 1101 to ~1250 px (correctness 5) — FIXED.** The editor stacks under the controller
  up to 1260 px (the controller then at most 920 px wide; `select()` scrolls the editor into view up to 1260 too), and
  the button letters are 17 units. Size tests now at 1251 × 800 (stacked) and 1261 × 800 (the narrowest side by side)
  besides 1280 / 1366 / 1440, and they measure the button letters. RED on the reviewed tree: letters 12.13 px (1251),
  12.29 (1261), 12.57 (1280). X7 (stack at 1100: 12.89 px) CAUGHT in both browsers; X7b (stack at 1250: a label at
  12.94 px at 1251) CAUGHT; X8 (letters 15 units: 12.29 px) CAUGHT.
- **F — merge hazard: RMP1-C's `style.css` still has RMP1-B's old controller rules (correctness 6) — confirmed, a
  merge note** (not a defect in this tree): the rmp1-c worktree (base `88c8513`) has them at its `style.css` :415–430
  (`.callout { pointer-events: none }`, `.callout text { text-anchor: middle }`, `.callout.used rect` accent, `.cbtn*`).
  Added to the merge notes below.
- **G — guide 22's "Make R1 free" never appears for R1 (correctness 7) — FIXED:** "**Make X free** (both buttons of a
  pair: **Make R1 and R2 free**)".
- **H — the one-accent test saw only rects and circles (tests 9) — FIXED.** It now reads EVERY painted element under
  `svg.pad` against `--accent` and `--accent-strong`: none with nothing chosen; X chosen → X, its label and ONE leader.
  R2 (every leader accent: 14) and R3 (the stick knobs) now CAUGHT.
- **I — no check of labels against the controller (tests 10) — FIXED:** each pill must be apart from every button
  shape, both sticks and the controller's body. R12 (label columns moved in) CAUGHT ("off the controller's body").
- **J — "free" marking untested (tests 11) — FIXED:** each d-pad button's computed dash is not `none` and its label
  reads "Up · free". R4, R7 CAUGHT.
- **K — `fitLabels()` deletable (tests 12) — FIXED:** each pill at most 28 units + 2 px wider than its words. R1
  CAUGHT ("its pill is 46.7 px wider than its words").
- **L — the words fold's Chromium fix untested (tests 13) — FIXED:** opened and re-drawn in the same task, it stays
  open. R6 CAUGHT in Firefox and Chromium.
- **M — paths never exercised (tests 14) — FIXED:** ArrowRight, a mechanism's "No button: autons only", the re-zero
  button's "Open Driving feel" (step 2, More open), and Export's Go there on the ramp (More opens, the ramp in view;
  no `DEFAULT_ONLY` gate). R5, R9, R10, R11 CAUGHT.
- **N — removing `.mech-head` changes RMP1-C's pages after the merge (docs 17) — FIXED, and wider than reported:**
  `.seg.wide` too — RMP1-C uses it (`inspector.mjs` :117, :208) as well as `.mech-head` (`app.mjs` :502). Both rules
  are back in RMP1-B's block exactly as at base (no diff there). Departure (6) above is amended by this line.
- **O — the second copy is not in the log (docs 18) — recorded here:** from the coordinator's NOTES (11-19 18:16,
  18:21): its SendMessage started a SECOND copy of this run's builder in this worktree (two writers); the coordinator
  stopped the copy at ~18:15; the original continued at 18:19, keeping the copy's edits as the tree's state. Which
  edits were the copy's is not known here (the tree has no record of it). Its message carried the three items of A,
  which no line of the build recorded until this round.
- **P — toasts said "on the right" (docs 19) — FIXED:** "in the panel" in all three (the panel can sit under the
  controller). Test 2 reads the whole toast.
- **Q — found while re-checking `edit()` (not a reviewer's; pre-existing since RMP1-B) — FIXED:** on a read-only robot
  (a newer builder's controls), choosing a job or **Make … free** was refused, but the next toast replaced the refusal
  with "Added: …" / "… is free now". `edit()` now returns `null` when it refuses and those two callers stop. Test (the
  read-only test): a free button given a job, then a used one made free — the toast says "cannot be read here", the
  list unchanged. RED first ("Added: Up and Down. Change either, …"); X13 / X13b (either guard gone) CAUGHT.
- **All mutations, this round (32 mutants besides the three PREFIX runs; every one LIVE in its built site; X9, X9a,
  X9b ran before `preventDefault` was removed, X9c after):** CAUGHT — R1–R7, R9–R12, X1, X1b, X2,
  X3 (both), X3b (after its test), X4, X5, X6, X6b, X6c, X7 (both), X7b (after 1251 was added), X8, X9 / X9c (both),
  X10 (both), X12, X13, X13b. SURVIVED and acted on — X3b and X7b (tests added, then CAUGHT), X9a / X9b (the redundant
  `preventDefault` removed). None killed by the cap or the timeout.
- **The export is unchanged:** `git diff -- tools/route_maker/src/model src/ include/ Makefile` empty; node 156/156; the
  export test's byte comparison above.
- **Docs:** guide 22 (:202–207 the panel "under it, in a narrow window", one highlight, a problem stays red; :213 Make
  X free; :236–242 the map line goes in the same click, **Remove that line** only for a line already in the file;
  :244 the caption), the changelog's RMP1-D bullet (the map line in the same click), five pictures re-taken from the
  final tree (Firefox light 1440; light / dark × 1440 / 1280 in `scratch/rmp1-d/shots2/`, looked at).
- **PROSE REVIEW (rule 4), run 20:35 on the final words, BEFORE this line:** `grep -rn -i -E "Remove that line|on the
  right\b|Make R1 free|Make X free|turns? red|only thing coloured|only one highlighted|d-pad FREE|map line|stale map|Esc\*\*
  or|controls_tab|feel_tab|1100 px|1260|narrow window" docs README.md tools/route_maker/README.md test/README.md --include=*.md`
  (internal and api aside) → guide 22 :36 / :38 / :48 / :54 / :75 / :82 / :91 / :106 and `route-builder.md` :14 (the
  Route tab's panel, still on the right: true), guide 22 :204–205 / :213 / :237 / :239 / :244 (this round's), the changelog :342 (robot
  one's map: true), guide 02 :202 and `hardware-assumptions.md` :952 (not about this), the README :13 ("the editor
  beside it": true from 1261 px; kept short). The page's own words: `grep -n -i "on the right|beside|at the right"` in
  `controls_tab.mjs` / `feel_tab.mjs` / `help.mjs` → code comments only (:2, :324, :332; feel :2).
- **FINAL TESTS AND GATES (fix round, on the final tree; `scratch/rmp1-d/gates_fix.sh`, logs `gates_fix/`, `fixlogs/`):**
  Controls page **Firefox 25/25, Chromium 24 + 1 skipped** (the download way is Firefox's); A1's `page.test.mjs`
  (Firefox) 18/18; node **156/156**. The suite binary unlinked, configure, briefing generated, host build **0
  warnings**, briefing generated again — no tracked change; suite green **3,856 cases / 3,886,386 assertions** (4 GiB
  cap); briefing check / self-test; api_doc_tool self-test / coverage / fresh / examples / removability; staleness
  self-test and audit; doc_readability check-nav / check-tone / self-test (the doc checks re-run after the last doc
  edit); the PROS-free and sim-free greps clean; the ARM header gate exit 0 (260 lines). The src gate self-test **OK
  (398 detector cases)** and check **PASS (14 builds)**; after both, `git diff -- Makefile src/ include/` EMPTY and no
  untracked file under `src include test`. No browser, driver or test process of this run left. Not committed.

### Verification-round findings and their disposition (started 2026-11-19 21:44 EST, `date`, the builder again, Opus 5.5)

- **One finding (minor): four paths the fix round added had no test** — the verifier's mutants N1, N3, N7, N8 each
  passed the whole Controls page file in Firefox (`scratch/rmp1-d/verify/vmut_all.json`). Re-checked: real (each is a
  test gap, not a defect in today's behaviour; the verifier's probe and this round's runs agree). **FIXED with tests:**
  - **N7 (`edit()`'s own toast)** — new test 24: the claw's button moved from A onto Left with its Button field, and
    the re-zero moved onto Down in Driving feel. Each: 0 errors, `banner.free` empty, the toast is the "d-pad" sentence
    (read, then removed, so the next check reads a new one), one Ctrl+Z brings back the change AND the line.
  - **N1 (`.reverse()`)** — the same test, last part: a saved project whose map has three free lines ("Up", "Down",
    "Left / Right"); the right toggle given Up forward / Down back takes the first two in ONE edit; "Left / Right"
    stays, no `mapStale`, one Ctrl+Z brings the job and both lines back.
  - **N3 (Esc while typing)** — test 21: L1, "9" typed in Volts (not entered), Esc: the editor stays ("right
    toggle") and the box still says 9. Placed before the fold check, so that test's undo count is unchanged.
  - **N8 (`select()`'s scroll)** — the size tests: after the R1 click the editor's whole box is inside `#ctl-body`'s
    visible box. At 1251 × 800 the editor (483 px) fits the step (635 px) but starts at 642 px with no scroll —
    measured first (`scratch/rmp1-d/n8_probe.mjs`: the same at 1100 × 700 and 1024 × 768). Kept, not dropped: on a
    stacked screen the click would otherwise open a form mostly below the fold.
- **W — found while writing N1's test (not the verifier's): two lines gone said the sentence twice** ("The button
  map's "Up" line said free, so it went too (Ctrl+Z undoes both). The button map's "Down" line …") — FIXED in `edit()`:
  one sentence, `"Up" and "Down" lines … so they went too`. One line reads exactly as before (test 2 unchanged; no doc
  quotes it).
- **Mutations (`scratch/rmp1-d/vfix/vfixmut.py`, the verifier's driver with this round's list; logs `vfix/mutlogs/`,
  verdicts `vfix/vfixmut_all.json`):** each applied, the site re-built, the mutant checked LIVE in the built site, the
  WHOLE Controls page file run, restored in `finally` / on SIGTERM-SIGHUP-SIGINT with its timestamp bumped (the
  source byte-equal to a copy taken before, the built site re-built from it). ALL CAUGHT, each for its own reason:
  N1 (Firefox: `'Down'` left, not `'Left / Right'`), N3 (Firefox and Chromium: "Give a button a job", not "right
  toggle"), N7 (Firefox: no toast — the locator's 5 s wait), N8 (Firefox and Chromium, 1251 × 800: the editor at y 642,
  483 tall, the view 131–766), W1 (the old per-line words back: the toast says the sentence twice). None killed by the
  cap or the run's timeout.
- **Before the mutations, on the final tree:** Controls page Firefox **26/26**, Chromium **25 + 1 skipped** (the
  download way is Firefox's).
- **The export is unchanged:** no model, export or driver-file module touched this round.
- **Docs:** none changed — guide 22 :238 ("goes in the same click — the page says so, and one **Ctrl+Z** brings both
  back") is now tested on every path; :205 ("**Esc** or **×** closes it") stays (Esc in a text box is the box's).
- **PROSE REVIEW (rule 4), run 21:53, BEFORE this line:** `grep -rn -i -E "said free|went too|undoes both|the page says
  so|Esc\*\* or|Esc or|in view|scrolls? (it|the editor)" docs README.md tools/route_maker/README.md test/README.md
  --include=*.md` (internal and api aside) → guide 22 :205 and :238 (true, above); guide 16 :478, guide 18 :880, guide
  22 :129, `hardware-assumptions.md` :1792 ("the page says so": other pages, true); the rest "in view" of sensors and
  cameras (not this). The page's words: only `controls_tab.mjs` :133.
- **FINAL TESTS AND GATES (this round, on the final tree, 22:00–23:04; `scratch/rmp1-d/vfix/gates.sh`, logs
  `vfix/gates/`):** the suite binary unlinked, configure, briefing generated, host build **0 warnings** (no C++
  changed), briefing generated again — no tracked change; suite green **3,856 cases / 3,886,386 assertions** (4 GiB
  cap); briefing check / self-test; api_doc_tool self-test / coverage / fresh / examples / removability; staleness
  self-test and audit; doc_readability check-nav / check-tone / self-test; the PROS-free and sim-free greps clean; the
  ARM header gate exit 0 (260 lines). The src gate self-test **OK (398 detector cases)** and check **PASS (14
  builds)**; after both, `git diff -- Makefile src/ include/` EMPTY and no untracked file under `src include test`.
  Then the site re-built from the fresh describe tool: node **156/156**, A1's `page.test.mjs` (Firefox) **18/18**,
  Controls page **Firefox 26/26, Chromium 25 + 1 skipped**; `git diff -- tools/route_maker/src/model src/ include/
  Makefile` empty. After this line was written: the staleness audit, doc_readability check-nav / check-tone and
  the briefing check re-run, all exit 0. No browser, driver or test process of this run left. Not committed.

## RMP1-F — export → robot: where the code goes, whether it worked, what to do next

*(executor: Claude Opus 5.5, in the worktree `.claude/worktrees/rmp1-f`, branch `rmp1-f` from `5e2c7f1`; started
2026-11-19 at about 20:32 EST — the worktree and its base commit are both stamped 20:32:27 (corrected by the review:
this line first said ~20:20, a time never measured). Brief: `chunks/RMP1-F-export-to-robot.md`. Nothing committed by
the executor.)*

- Started: read RESUMING's standing rules, the brief in full. Next: the export page's code (`src/ui/export*`,
  `src/io/*`), RMP1's design system where touched.
- ~20:40 — host build from the base: green (`doc coverage: 6502 … all documented`). Node: 173 before.
- ~21:00 — code in: `region.mjs` readBack; `files.mjs` exportToFile reads back and returns a refusal for a write the
  folder throws on (`failed`); `model/to_robot.mjs` (the commands, the summary); `ui/to_robot.mjs` (result card, Will it
  build?, the paste picture, Next); `export_drawer.mjs` rebuilt around ONE numbered card per browser; CSS in one
  delimited block; `tools/upload_auton.py` with its `self-test` (14 cases against fake make/pros). Node 179/179;
  `export_robot.page.test.mjs` 6/6 Firefox, 5/5 + 1 skipped Chromium. Next: the whole page suite, docs, mutations.

### RMP1-F — what was built (2026-11-19, about 20:32–21:31 EST: the worktree's stamp to this file's last write before the review)

- **D1** `src/ui/export_drawer.mjs` (rebuilt): ONE card "Put it into your robot's code", numbered, the step to do now
  the ONE primary button. Chrome/Edge (`files.canWriteFolder`): 1 Connect your shulib folder (the one with src inside),
  2 Write into src/team/skills_route.cpp (disabled until 1). Any other browser: 1 Pick skills_route.cpp and download the
  updated one, 2 Replace src/team/skills_route.cpp with the downloaded file (Copy beside the path). Both: "Or Copy the
  code". The code is one click away (`<details>` "See the code: N lines of C++"), not a wall.
- **D2** the read-back lives in the FILE LAYER, for every host: `region.mjs` `readBack(text, regionLF, wrote)` ('gone',
  a findRegion why, 'differs' — the region AND the whole file compared); `files.mjs` `exportToFile` reads the file back
  after writing (and when nothing was written), and a write the folder throws on is `{why: 'failed'}` (a refusal, never
  an unhandled rejection). The UI (`ui/to_robot.mjs` `resultCard`): green "Written and checked: src/team/skills_route.cpp"
  + "2 routes (skills, left 1), 3 steps"; "Downloaded skills_route.cpp — now replace src/team/skills_route.cpp with it"
  (+ Copy the path, + Save route_maker.json); "Copied — paste it over the two ROUTE MAKER lines … and everything between
  them" + a small picture of the two lines selected. Red: the reason + "The fix:" with its one button (Connect / Write
  again / Copy); on a red card the fix is the ONE primary button. A hand edit: Keep their change → red "Not written:
  their change was kept"; Overwrite → writes.
- **D3** "Will it build?" in every green card: the steps checked by the library's rules; the code between the two
  lines (read back after a write); after a write, both autons files' addTeamRoutes lines (✓ both, or ✗ per robot);
  "The C++ compiler checks the rest when you build: that is the next step."
- **D4** `nextCard`: Robot one | Robot two; 1 `make ROBOT=one PROGRAM=autontest` (two: `ROBOT=tank`) + Copy, "It worked
  when make ends with no line saying "error"."; 2 `pros upload --slot 2 --name "shulib Auton Tester"` + Copy, "It worked
  when pros says "Finished uploading"."; 3 On the brain: shulib Auton Tester (slot 2) → the routes → RUN; the one command
  `python3 tools/upload_auton.py one|two` (`python` on a Windows browser) + Copy. Commands from `model/to_robot.mjs`.
- **D5** `tools/upload_auton.py`: make for the robot, `pros lsusb` → the V5 Brain's own (not User, not Controller) port,
  `pros upload --slot 2 --name "shulib Auton Tester" --port <it>` (stdin closed: a pros prompt fails instead of hanging;
  300 s cap); ✓ BUILT / ✓ UPLOADED to slot 2 + the route names from the region, or ✗ with: the OWED facts (deduped, in
  make's order) and their facts file; a compile error's file:line + message, inside the ROUTE MAKER region → "re-export
  from the route builder", in skills_route.cpp's plumbing → "undo the change there", elsewhere → "fix that line";
  make/pros missing → the PROS terminal or the install page; no brain / controller only / two brains; upload exit 0
  without "Finished uploading" → ✗ (R3a's silent-upload lesson). Full output: build/upload_auton.log. `self-test`: 14
  cases against fake make/pros on PATH (POSIX sh fakes; `.cmd` fakes on Windows) + a scan for POSIX-only calls. CI runs
  it on every row of the route-builder browsers job (Windows included).
- **REAL make, fake pros (a live probe, scratch/rmp1-f/realmake):** `upload_auton.py one` → `✓ BUILT` (robot one's
  Auton Tester links today) → `✓ UPLOADED … skills`; `upload_auton.py two` → `✗ NOT BUILT`, robot two's 31 OWED facts,
  "Type them in src/robots/robot_two_facts.hpp" — parsed from the real compiler's output.
- **D6** the exported C++ is untouched (`export.mjs` not edited; the goldens' node tests pass unchanged).

**Deltas from the brief, each with its reason.**
- Copy's words: "paste it OVER the two ROUTE MAKER lines and everything between them" (the brief: "between"): the copied
  code carries both marker lines (`exportRegion` is BEGIN..END inclusive), so pasting between them makes two pairs and
  the next Write refuses (`markerCount`). The picture shows both lines selected.
- The download way keeps the PICK (1 "Pick skills_route.cpp and download the updated one"): the page does not own the
  plumbing outside the region, so it cannot make the whole file; the pick keeps every byte outside the region.
- After a write the autons files' addTeamRoutes lines move into "Will it build?" (✓ both / ✗ per robot): a missing line
  means the routes never reach the brain's menu, which is the question the team lead asked. `page.test.mjs`'s one
  assertion of the old hint after a write (`Robot two registers every route`) became the new line (that test's folder
  lacks robot one's autons file: "Robot one will not list the routes … — tell the coordinator").
- A write that read back different leaves what is on disk NOT the builder's: "Write again" then ASKS (the hand-edit
  question) and Overwrite writes — never a silent overwrite of what something else put there.
- The code is collapsed under "See the code" (one idea at a time); guide 22's "Reading the code" points there.
- Upload success needs exit 0 AND "Finished uploading" (the recorded real line, R3d G.1): R3a 13.14's silent failure.
- Guide 23 step 5: NOT in this tree (guide 23 exists only on the `tutorial` branch) — for the coordinator.
- CI: one step added to the route-builder browsers job (every row, Windows included): the script's self-test.
- *(added by the review)* The card titles carry no ✓ or ✗ text (the brief: "✓ Written and checked …", "✓ Downloaded",
  "✓ Copied"): the card's icon carries it (a check, or the error mark), so the title is the words alone.
- *(added by the review)* The script printed `->` where D5 shows `→` (an ASCII-only console); since the review it
  prints `→` where the console can, `->` where it cannot (the same test as ✓/✗ → OK/X).
- *(added by the review)* Public docs touched also include `docs/changelog.md` (the API 2.9 entry, 2026-11-19).

**Tests added (adversarial; every fault the test's own).**
- Node `test/to_robot.test.mjs` (6): readBack (CRLF same region passes; gone; no markers; one character in the region;
  one character OUTSIDE it; the old region); exportToFile with a writeText that changes the text → 'differs', no export
  hash; one that deletes the file → 'gone'; one that throws NotAllowedError → 'failed', the file untouched; a good write
  is read back (reads counted: read, write, read back; unchanged: still read back); the summary; the commands, pinned to
  the Makefile's own words and to the script's SLOT/NAME.
- Page `test/page/export_robot.page.test.mjs` (Firefox 6, Chromium 5 + the download case skipped): D1 per browser (and
  that Chromium HAS the API and Firefox does not), one primary button; Write → green names "2 routes (skills, left 1), 3
  steps", the file's region equals the drawer's code, Will it build?, Next for robot one then robot two, Copy puts the
  command on the clipboard; a planted mismatch → red, no Next, nothing saved, ONE primary (the fix), Write again → the
  question → Overwrite → green; markers removed after opening → red + "use Copy instead", its Copy copies the code; a
  folder that refuses → red + Connect; Keep their change → red, file untouched; Copy → green + the picture; the download
  (Firefox) → the downloaded file's region is the code, Copy the path, Save.
- `tools/upload_auton.py self-test` (14): ok one / ok two (exact argv of make and pros), OWED (deduped, in order, no
  upload), compile error in the region (a Windows `C:\…` path), in the plumbing, elsewhere, no brain, controller only,
  brain + user port + controller (picks ttyACM0), two brains, upload exit 0 without "Finished uploading", permission
  denied, make/pros not on PATH, a wrong robot word; and a scan for POSIX-only calls.

**Mutations (driver `scratch/rmp1-f/mut/mut.py`: restores in finally and on SIGTERM/SIGHUP/SIGINT, bumps the restored
file's timestamp; page mutants each on a FRESH site build; site rebuilt from the restored sources after).** All RED:
M1 region compare dropped → readBack test; M2 whole-file compare dropped → "outside the region"; M3 no read-back → 3
node tests; M4 the catch rethrows → "never a throw"; M5 robot two `ROBOT=two` → the commands test; M6 first route's steps
only → the summary test; S1 first port, not the brain's → `--port /dev/ttyACM2`; S2 OWED not deduped → SURVIVED the
first cut, the case strengthened (the header line, then exactly two facts), then RED; S3 region check off → "re-export"
missing; S4 exit 0 = uploaded → "quiet" case; S5 two → `ROBOT=two` → argv; S6 no backslash normalising → the Windows
path case; S7 the User port counted as a brain → "ports" case. Chromium page: P1 the browser test swapped → D1, D2, the
mismatch test; P2 red shown green → the mismatch + markers tests; P3 no Next → D2-D4 + Copy tests; P4 the robot switch
dead → `make ROBOT=one` where `ROBOT=tank` expected; P5 the fix not the one highlight → "one highlight: the fix"; P6 no
picture → the Copy test; P7 no autons lines in Will it build → my D2-D4 test AND `page.test.mjs`'s; Firefox: P8 the
download's green card not shown → the download test (first cut used `return;` — Firefox's "unreachable code" console
warning made EVERY test red: re-cut as `void (…)`, then only the download test red).
After the runs: every mutant's `old` text is in its file exactly once (S4's `new`, `if code != 0:`, is also make's own
check — a true line, not a leftover).

**Prose review (greps run and hits read BEFORE this line).**
- `export_drawer|files\.mjs|region\.mjs|to_robot|upload_auton|exportToFile|readBack` over docs (internal excluded),
  README, src, test, tools: the route_maker README rows (updated), guide 16 :58 (new); `readBack`/`readBackup` in
  `test/record_store_test.cpp`, `calibration_record_test.cpp`, `fake_file_store_test.cpp` are C++ locals of other code.
- `Copy the code|Write into src/team|Pick skills_route|Kept\. Nothing|Only the lines between|in place of everything|C++ with
  Copy|export page|export drawer|checked once the folder|build and upload as always|the drawer shows`: guide 22 :168–177
  (new), :203 "The export page checks both lines" (still true: the step card when connected, Will it build after a
  write), :343 (the Controls tab's, not mine); changelog :178 (a dated entry). Fixed: guide 22's "the picture above"
  (the code is under See the code now), "build and upload as always" → the Next card.
- The page's own words: `capabilities.mjs` :22–23 and the top-bar chip "Save downloads · Export copies" stay true (Copy is
  still there); `app.mjs` :691 true. `docs/where-your-code-goes.md` :35 — added the helper line (its picture check: OK).
  `docs/route-builder.md` :28 — "(and reads it back)". `RMP1-A1-manual-checks.md` step 6 — the Windows check now covers
  the green card, the Next card's build command and the script against the real pros.
- NOT here: guide 23 (the `tutorial` branch only) — its step 5 should point at `tools/upload_auton.py` at the merge.

### Gates (2026-11-19, the final tree; the gate logs' own stamps: 21:08–21:30 EST)

- Build: `rm -f build/test/shulib_tests`, briefing generate (unbuilt), `cmake --build build/test -j8` → exit 0: `doc
  coverage: 6502 public entities across 215 headers, all documented`, `check-nav: OK — 263 public pages`, `check-tone:
  OK — 48 pages`, `first_week: OK`; re-run `--target shulib_doc_gates` after the last doc edit: the same.
- Suite (`suite_run.py green`, 4 GiB cap): `suite run: green — 3880 cases, 3775113 assertions` (no C++ changed).
  Briefing regenerated on the built binary; `check`: `generated block current`.
- Node: `# tests 179 # pass 179 # fail 0` (173 + 6). `upload_auton self-test: OK (14 cases against fake make and pros)`.
- Page, the full suite on a site built from the final tree (no `--engine`), under `MemoryMax=6G`: Firefox `# tests 69
  # pass 67 # skipped 2`, Chromium `# tests 69 # pass 63 # skipped 6` (RMP1-C's skips + the Firefox-only download case).
  Screens: `~/shulib-render-tmp/rmp1-f/shots/` — xr-step, xr-written, xr-read-back-differs, xr-copied in light/dark at
  1440 and 1280 in both browsers, + firefox-xr-downloaded. Docs pictures re-made from Chromium light 1440 (1200 × 750,
  256 colours, `scratch/rmp1-a2u/to_docs.py`): `export.png` (re-taken), `export-written.png` (NEW).
- ARM header gate: `TU includes 259 headers` → `ARM compile gate: CLEAN`. CI greps: PROS-free, sim-free, the route
  builder's files LF (and the new ones). `git diff -- include/ src/ Makefile test/` is empty.
- `api_doc_tool`: self-test OK, check-coverage as above, check-fresh exit 0, check-examples `988 quoted lines, all
  verbatim`, check-removability `no public doc references docs/internal/`. `doc_staleness_audit.py`: self-test `OK (16 …)`,
  then `clean (37 live docs, 0 numeric claims checked)`.
- src gate `check`: `PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly their owed facts
  (6 refused; …))`. Its self-test NOT run (no src/ or Makefile change). `ctest -R compile_refusal` not run (no C++).
- Not run here: the Windows CI rows (the script's self-test now runs there), the real `pros` (never). Nothing committed.

### RMP1-F review findings and their disposition

*(the builder again, Claude Opus 5.5, same worktree; started 2026-11-19 21:49 EST by `date`. Three reviewers'
findings (correctness, tests, docs), deduped; each re-checked against the source or a probe before it is fixed or
answered. Probes and mutation logs: `scratch/rmp1-f/fix/`. Nothing committed.)*

- 21:49 — started: read RESUMING's rules again, the brief, and every finding. Checked pros-cli 3.5.6's own source on
  this laptop (`~/.local/lib/python3.12/site-packages/pros/cli/upload.py`, `common.py`, `serial/devices/vex/v5_device.py`)
  before touching the script. Next: the script.
- 22:00 (the edited files' own stamps: the page test 21:57, the docs 21:59; the first mutant's log 22:00:40) — the script rewritten at the root (positional `[PATH] [PORT]`; the port from pros's own "V5 System Ports"
  section; header errors traced to the region; ASCII consoles; timeouts and Ctrl+C as a plain ✗) and its self-test's
  fake pros given pros-cli 3.5.6's grammar: `upload_auton self-test: OK (30 cases …)`. The page: titles, fixes, the
  wrong pick named, the download's newest file, "Not checked here", the PROS terminal and Python 3 beside the one
  command. Node 181/181; `export_robot.page.test.mjs` Chromium 8 + 1 skipped, Firefox 9/9. Docs: guide 22 step 7 cut,
  guide 16, changelog, where-your-code-goes, README, manual check 6. Next: the mutations.
- 22:06 (`date`) — the mutations done (below); the host build and the suite green. Next: the disposition, the gates.

**The findings, deduped (19 → 16), each re-checked before it was fixed.**

1. **BLOCKER — `--port` is not a pros-cli option (tests lens).** CONFIRMED twice: pros-cli 3.5.6's own source on this
   laptop (`pros/cli/upload.py`: `@click.argument('path')`, `@click.argument('port')`; no `--port` anywhere), and a
   probe that copies only those decorators into click 8.1.8 (`scratch/rmp1-f/fix/click_argv.py`; pros NOT imported or
   run): the old argv → `No such option: --port`, exit 2; `… . /dev/ttyACM0` and `… . COM3` → `path='.'`,
   `port=<it>`. FIXED: `pros upload --slot 2 --name "shulib Auton Tester" . <port>` (cwd = the repository, which
   upload.py resolves to the project), no port → no path either (pros finds both). The self-test's fake pros now takes
   pros-cli's grammar (an unknown option → `Error: No such option`, exit 2; `[PATH]` must exist; at most two
   positionals) and prints lsusb in pros's own shape (`VEX EDR V5 System Ports:` / `There are no connected …`, from
   `ls_usb`'s `PortReport.__str__`). Mutant F1 (the old `--port` command) → RED in ok/one, ok/two, ports, win, winusb.
2. **The brain found only by the word "Brain" (correctness; and docs 15's "choose by System Ports").** CONFIRMED by
   reading `find_v5_ports`: pros sorts by USB interface (location `…0` system, `…2` user, `…1` controller) and falls
   back to names; `lsusb` prints that sorting as sections, and its System section also lists a controller
   (`system_ports + joystick_ports`). FIXED: the port is the System section's one port whose name does not say
   "Controller"; none → "Only the controller …" or "No brain found …"; two that both say "Brain" → "Two brains …";
   two unnamed (Windows: a brain and a controller with the same driver name) → "More than one V5 port … keep only the
   brain's USB cable in"; no System header at all (another pros's shape) → no port, pros picks. New cases: `win`
   (COM3 "VEX Robotics Communications Port (COM3)" + a User section → `. COM3`, ✓), `winusb` ("USB Serial Device"
   names → `. COM3`, ✓), `winboth`, `oddformat`. **For the coordinator — a belief, not a measurement:** Windows'
   `pros lsusb` names are still unseen (the fake's Windows lines are made up); the choice now depends only on pros's
   own sections and the word "Controller". Manual check 6 asks for the real output to be pasted into this log.
3. **No markers / the wrong file picked: an out-of-date reason and a dead-end fix (correctness).** CONFIRMED (the
   markers are in `src/team/skills_route.cpp` :37/:69 in this tree; Copy's card says to paste over lines that are not
   there). FIXED: `region.mjs`'s noMarkers words no longer mention robot two's update ("someone removed them, or this
   is not the skills-route file"); markerCount/markerOrder lose their "fix it by hand" tail (the page gives the fix);
   Chrome — on Write and in the banner when the drawer opens — "The fix: ask the coordinator to put back its one pair
   of ROUTE MAKER lines (git checkout src/team/skills_route.cpp undoes changes not yet committed)", no Copy offered;
   Firefox — `exportByDownload` returns `picked` (the file's name) with a refusal, and a wrong pick says "You picked
   autons.cpp: the routes go into src/team/skills_route.cpp. The fix: pick src/team/skills_route.cpp" with the Pick
   button as the one highlight. "Robot two's update" stays only for a folder with NO skills-route file (an old copy).
4. **"Not written" for a write that read back different (correctness).** CONFIRMED. FIXED: the title is "Written, but
   it read back different" ('differs') or "Written, then the file was gone" ('gone'). Found while fixing: `readBack`
   returned findRegion's own why when the markers were lost IN the write, so that case would also have said "Not
   written" with the put-them-back fix; it is now 'differs' too (the page wrote those markers).
5. **The download renamed `skills_route(1).cpp`; "✓ The file has exactly this code" after a download
   (correctness + docs 19b).** FIXED: "Use the NEWEST skills_route.cpp in your Downloads: a second download gets a
   number, like skills_route(1).cpp." (its own plain line: the first cut, inside the bold instruction, read as a wall
   in the screenshot) and "The downloaded file has exactly this code between its two ROUTE MAKER lines: replace
   yours with it."
6. **Python not mentioned; three places name different terminals (correctness minor + docs major).** CONFIRMED (the
   page: "Terminal → New Terminal"; the script and manual check 6: the PROS terminal). FIXED with ONE wording, from
   the script's own constant `TERMINAL`: "the PROS terminal (VS Code: PROS, then Integrated Terminal)" — the Next
   card, the script's missing-tool line, guide 16, manual check 6. Beside the one command: "it needs Python 3, from
   python.org; without it, use 1 and 2" (also guide 16 and the changelog). Unverified, and now asked by manual check
   6: whether that terminal has `make`, `pros` and `python` on the team's Windows laptop.
7. **A header error caused by the region said "Fix that line" (correctness).** FIXED: `from_region` reads GCC's
   context lines around the first error (back to the file's `Compiled …` line, on to the next error) and, when a
   "required from" / "In file included from" line names a line inside the region, gives the region's advice. Both
   region boundaries now share one test, `inside()`: cases `edge` (an error on the END line) and `header` (required
   from the BEGIN line) pin `<=` on each side (mutants F7, F8 RED); `headerplain` keeps "Fix that line".
8. **A long build prints nothing; Ctrl+C ends in a traceback (correctness).** FIXED: Ctrl+C → "✗ STOPPED (Ctrl+C).
   What make and pros printed so far: build/upload_auton.log", exit 130 (a POSIX self-test case sends SIGINT while
   the fake make sleeps; mutant F16 RED); the build line says "(a few minutes the first time; Ctrl+C stops it)" and
   is flushed. NOT done: echoing make's `Compiled …` lines (the finding's optional half) — a hundred lines scrolling
   is the dense screen the team lead asked us to avoid; the line says how long it takes.
9. **The page test expects `python3` on Windows CI rows (tests).** CONFIRMED by reading CI (windows-latest rows run
   `test/page/*.test.mjs`). FIXED: D2-D4 takes the word from the page's own userAgent through `robotCommands`; a new
   test sets a Windows Chrome userAgent in the page (no harness change: `navigator.userAgent` is read when Next is
   built) and asserts `python tools/upload_auton.py one`. Mutant P16 (`windows = false`) RED.
10. **The whole-file half of the read-back untested end to end (tests).** CONFIRMED (R1 shape). FIXED: a node test
    through `exportToFile` whose write changes only `#include` → 'differs', no export hash. Mutant N2 (R1) RED.
11. **Never-silent and one-highlight mutants survive (tests: R2, R4, R6).** FIXED: a Copy the browser refuses
    (Clipboard API and `execCommand` both) → red "Not copied", no Next, the code opened (P9 RED); route_maker.json
    changed in the folder before Write → the green card's `.xr-warn` line, theirs kept (P10 RED); after connecting,
    exactly one `.btn.primary`, the Write button (P11 RED).
12. **What the cards claim, untested (tests: R3, R5, R7, R8).** FIXED: Copy's build list has "cannot see your editor"
    and not "read back"/"has exactly this code" (P12 RED); the download's has "The downloaded file …" and not "read
    back" (P13, Firefox, RED); one route → "→ skills → RUN" (P14 RED); a refused command copy → "Select it and copy"
    (P15 RED).
13. **Seven surviving script mutants (tests: T1–T7).** FIXED with cases, each mutant re-cut against the fixed script
    and RED: T1 pros's own no-port words (`nodevice`, F9); T2 a linker error (`link`, F10); T3 both boundaries (F7,
    F8); T4 no markers / two pairs → "→ your route → RUN", no traceback (F11); T5 a cut-short lsusb that fails →
    pros picks (`lsusbfail`, F12); T6 an ASCII console (`curly`, PYTHONIOENCODING=ascii, GCC's ‘r’ quotes) → "X NOT
    BUILT", "?r?" (F13) — which found a real crash: printing a compiler message the console cannot encode raised
    UnicodeEncodeError, so stdout now replaces what it cannot print (F14 RED); T7 a hung lsusb under a short wait
    (`--wait 0.05`, the self-test's hidden option) → "✗ NOT UPLOADED: pros gave no answer in 3 s" (F15; F18 for the
    lsusb-timeout branch). Found while reading pros's source: with stdin closed, any pros question (several brains,
    too many files on the brain, a wireless upload) ends "Aborted!"; it now says "pros stopped to ask a question. Run
    the upload yourself to answer it: pros upload --slot 2 --name "shulib Auton Tester"" (`question`, F17 RED).
14. **Public docs state the helper's real-brain behaviour as fact (docs).** FIXED in place, as CAL2's entry does:
    guide 16 and the changelog say "It has run against stand-ins for `make` and `pros` only: not yet against the real
    `pros`, or on Windows", and that it needs Python 3. The lsusb belief is item 2 above.
15. **The log's times were never measured (docs).** CONFIRMED (the worktree and 5e2c7f1 are both 20:32:27; the gate
    logs 21:08–21:30). FIXED in the three places. And caught in my own first entry above (it said 22:20 before 22:00
    had passed): corrected to the files' stamps. Every time in this section is `date` or a file's stamp.
16. **Small wording (docs 17, 18, 19; deltas).** FIXED: "every write is READ BACK" → "the export into
    skills_route.cpp is READ BACK" in `region.mjs`, `files.mjs`'s header and the README (and the README's "14 cases"
    → its cases against a fake pros with pros-cli's grammar); region.mjs's rule (a) "a student's Copy replaces the
    marked lines with a copy that carries them; never a new pair"; where-your-code-goes adds "`python` on Windows";
    guide 22 :203 "With the folder connected (Edge or Chrome), the export page checks both lines", :204 "Edge or
    Chrome says so and offers Copy"; "Will it build?" after a Copy or a download says "Not checked here: the
    addTeamRoutes line in each robot's autons file (Edge or Chrome checks it once the folder is connected)" (and a
    Chrome page not yet connected no longer counts unread lines as present: P22 RED). The deltas list gains the
    titles-without-✓ and the arrow (the script now prints → where the console can) and names the changelog. Guide 22
    step 7: the command block and "Or both at once" cut to one sentence pointing at Next and chapter 16; 713 words
    (base 617, the reviewed tree 836) — the rest is the three browser bullets, which are this chunk's point.

**Mutations, this round** (driver `scratch/rmp1-f/fix/mut2.py`, the same discipline: restore in finally and on
SIGTERM/SIGHUP/SIGINT, the restored file's timestamp bumped, every page mutant on a FRESH site build). All 34 RED:
script F1–F18 (self-test), node N1 (lost markers → 'differs'), N2, N3 (`picked`), page P9–P12, P14–P19, P22 in
Chromium and P13, P20 (the wrong pick named) in Firefox. Each failure was read: it is the assertion the mutant aims
at (e.g. P15 `expected: 'Select it and copy'`, P16 `expected: 'python tools/upload_auton.py one'`). Logs:
`scratch/rmp1-f/fix/mut/`.

**Prose review, this round (greps run and every hit read BEFORE this line).** `--port`, `✓ Written`, `✓ Downloaded`,
`✓ Copied`, `New Terminal`, `PROS terminal`, `Integrated Terminal`, `robot two's update`, `NEWEST`, `Not checked here`,
`someone removed`, `Python 3`, `every write`, `no student pastes`, `14 cases`, `use Copy instead`, `has no ROUTE MAKER
lines yet`, `checks both lines`, `upload_auton`, `Not written into`, `read back after` over docs (internal and api
excluded), README, `tools/route_maker` and the script. Every hit is this round's wording, or another subsystem's
(`every write` in guide 11, the changelog's old entries, hardware-assumptions; `14 cases` in the roadmap is another
test), or the one kept case (no skills-route file: `use Copy instead`, "robot two's update"). **For the coordinator:**
the Controls tab's own export (`controls_export.mjs` :113, `driver_file.mjs` :151 — RMP1-D's files, not edited) still
says the driver file "gets them [its markers] with robot two's update"; `src/team/robot_one_driver.hpp` already has
them (`// [control words one]` :22), so that wording has the same staleness finding 3 fixed here.

### Gates, after the review (2026-11-19; `date` 22:05–22:32 EST)

- `rm -f build/test/shulib_tests`, briefing generate (unbuilt), configure, `cmake --build build/test -j8` → exit 0:
  `doc coverage: 6502 public entities across 215 headers, all documented`, `check-nav: OK — 263 public pages`,
  `check-tone: OK — 48 pages`, `first_week: OK`; after the last doc edit `--target shulib_doc_gates` → the same.
- Suite (`suite_run.py green`, 4 GiB cap): `suite run: green — 3880 cases, 3775113 assertions`. Briefing regenerated on
  the built binary; `check`: `generated block current`.
- Node: `# tests 181 # pass 181 # fail 0` (179 + 2). `upload_auton self-test: OK (30 cases against fake make and pros)`.
- Page, the full suite on a site built from the final tree, under `MemoryMax=6G`: Firefox `# tests 72 # pass 70
  # skipped 2`, Chromium `# tests 72 # pass 66 # skipped 6` (69 + the three new tests; Chromium skips the download way).
  Screens re-taken: `~/shulib-render-tmp/rmp1-f/shots-review/` (light/dark at 1440 and 1280, both browsers, +
  firefox-xr-downloaded; the full suite with screens: Firefox 111/109/2, Chromium 111/105/6). `export.png` re-made
  from them is byte-identical to this tree's; `export-written.png` re-made (the Next card's words changed).
- ARM header gate: `TU includes 259 headers` → `ARM compile gate: CLEAN`. CI greps: PROS-free, sim-free, LF (the
  changed and new route-builder files and the script). `git diff -- include/ src/ Makefile test/` empty.
- `api_doc_tool`: self-test OK, check-fresh exit 0, check-examples `988 quoted lines, all verbatim`, check-coverage as
  above, check-removability `no public doc references docs/internal/`. `doc_staleness_audit.py`: `clean (37 live docs …)`.
- src gate `check`: `PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly their owed facts
  (6 refused; …))`. Self-test not run (no src/ or Makefile change).
- Not run here: the Windows CI rows, the real `pros` (never). Nothing committed.

### Verification-round findings and their disposition (2026-11-19 from 22:44 EST, `date`)

*(the builder again, Claude Opus 5.5, same worktree. A fresh verifier's two minor findings; each re-checked against the
source and its probe before it is fixed. Probes and mutation logs: `scratch/rmp1-f/vfix/`. Nothing committed.)*

- 22:44 — started: read RESUMING's rules, the brief, both findings and the verifier's probes. Next: re-check each.
- 22:47 (the edited files' own stamps: the page test 22:46; the drawer and the script restored by the mutation driver
  22:54, so their stamps are its) — both fixed; the self-test `OK (31 cases …)`; the export page tests Chromium 8 + 1
  skipped, Firefox 9/9. 22:51–22:54 (`date`) — the mutations (below). 22:56 — guide 22's one clause. 22:57–23:08 — the
  gates (below).

**The findings, each re-checked before it was fixed.**

1. **Markers gone when the drawer opens: Copy still offered, focused, and it led to a green "Copied" that sends the
   student to lines that are not there (minor).** CONFIRMED by reading `export_drawer.mjs` (the Copy line was always
   appended and took the focus when nothing was primary) and by the verifier's probe. Also found while re-checking:
   after a refused **Write** for the same reason, the red card had no Copy, but the steps card's "Or Copy the code"
   line stayed (the old test looked only inside the card). FIXED at the root: `putBack()` (the one fix for markers
   gone, doubled or out of order) now removes the Copy line wherever it is used — the banner on open, the refused
   Write, Firefox's refused download — and on open, with that banner, the Copy line is never added and focus goes to
   the banner (`tabindex="-1"`). No skills-route file at all (an old copy) keeps Copy and its focus: that banner says
   "use Copy". Tests: the markers-gone test now asserts no Copy anywhere in the drawer after the refused Write; on open
   (no markers, and two pairs) no Copy, no primary button, focus on the banner; no file: Copy focused. Firefox's
   download test: a wrong pick keeps Copy; a skills_route.cpp without markers → red with the put-back fix and no Copy;
   then the right file downloads. The verifier's probe, re-run on the fixed page: `PRIMARY COUNT: 0`, `COPY BUTTONS
   IN STEPS: 0`, focus on the banner. Screens: `~/shulib-render-tmp/rmp1-f/shots-vfix/*-xr-markers-gone-*` (both
   browsers, light/dark at 1440 and 1280); every other export screen byte-identical to the review round's, so
   `export.png` and `export-written.png` stand. **Not changed, for the coordinator:** the no-file case's own Copy
   ("it arrives with robot two's update. Until then, use Copy") also ends at "paste it over the two ROUTE MAKER
   lines in src/team/skills_route.cpp", a file that folder lacks: an old copy, or the wrong folder connected. Left as
   the review round decided; worth a look if the team ever connects the wrong folder.
2. **Ctrl+C names build/upload_auton.log, but make's output reached the log only when make finished (minor).**
   CONFIRMED by reading `run()` (`subprocess.run` captured everything, the log was written after it returned) and
   by the verifier's probe (`LOG BYTES: 0`). FIXED at the root: `run()` starts the command with `Popen` and a reader
   thread copies each line into the log as it is printed (flushed); Ctrl+C, a timeout or the end writes the
   trailer — `(stopped: Ctrl+C)`, `(stopped: no answer in N s)` or `(exit N)` — then Ctrl+C goes on to the plain
   ✗. A timed-out command is killed; the reader is waited for at most 5 s (a child's child can hold the pipe:
   probe `vfix/probe_hold.py`, 6.0 s for a 1 s timeout). The verifier's probe re-run (SIGINT to the process group,
   a shell `make` that prints two lines then sleeps): `EXIT 130`, the log has both `Compiled …` lines and
   `(stopped: Ctrl+C)`. Self-test: the ctrl-c case's fake make prints a line first; the case asserts that line is in
   the log WHILE make still runs, and `(stopped: Ctrl+C)` after; the `hang` fake now writes a call line if it outlives
   the script's timeout (POSIX: the `.cmd` stand-in's python outlives `cmd.exe` on Windows, so that row skips the
   calls check); a new case, `late`, has pros end with its last line still in the pipe (its own child prints
   "Finished uploading" 0.5 s later), which only reading to the end gets right. The docstring says the log is
   written as make and pros print. No walrus or other new syntax: the script still parses on any Python 3.

**Mutations, this round** (driver `scratch/rmp1-f/vfix/mut3.py`: restore in finally and on SIGTERM/SIGHUP/SIGINT,
the restored file's timestamp bumped; every page mutant on a FRESH site build in `vfix/site_mut`; checksums of
both files checked clean after each batch). All 13 RED, each failure read: PM1 Copy appended with the banner (:194
`1 !== 0`), PM2c Copy kept after a refused Write (:187), PM2f the same in Firefox's download (:324), PM3 focus not on
the banner (:196 'focus on the fix'), PM4 two pairs of markers keep Copy (:194 via the markerCount call :201), PM5 no
file takes Copy too (:206), PM6 a wrong pick takes Copy (:318 'the real file has its lines: Copy stays'), PM7 the
banner without tabindex (:196); script SM1 the old `run()` (ctrl-c: 'not written as make printed'), SM2 no
`(stopped: Ctrl+C)` (ctrl-c, the log live but no trailer), SM3 no flush (ctrl-c, not live), SM4 no kill on a
timeout (hang: `pros lsusb ran past its timeout`), SM5 `reader.join(0)` (late: exit 1, wanted 0). SM5 first
SURVIVED (the reader always won the race against the fakes' small outputs); the `late` case was added for it and it
is RED. Logs: `scratch/rmp1-f/vfix/mut/`.

**Prose review, this round (greps run and every hit read BEFORE this line).** `someone removed`, `put back`, `no
ROUTE MAKER lines`, `Or Copy`, `use Copy`, `Copy the code`, `upload_auton`, `upload_auton.log`, `Ctrl+C` over docs
(internal chunk logs and docs/api excluded), README, `tools/route_maker`: guide 22 :176 said Copy without the
exception → one clause added ("A file that has lost those lines gets no Copy: the page says how to put them back
first."); guide 16 :59 "Everything make and pros printed is in build/upload_auton.log" — now true after Ctrl+C too,
unchanged; the changelog, where-your-code-goes, the README and manual check 6 describe neither behaviour, unchanged.

### Gates after the verification-round fixes (2026-11-19, `date`: 22:57–23:14 EST; every one on the final tree)

- `rm -f build/test/shulib_tests`, briefing generate (unbuilt), configure, `cmake --build build/test -j8` → exit 0:
  `doc coverage: 6502 public entities across 215 headers, all documented`, `check-nav: OK — 263 public pages`,
  `check-tone: OK — 48 pages`, `first_week: OK`.
- Suite (`suite_run.py green`, 4 GiB cap): `suite run: green — 3880 cases, 3775113 assertions`. Briefing regenerated on
  the built binary; `check`: `generated block current` (re-checked after this log's last edit); self-test OK.
- Node: `# tests 181 # pass 181 # fail 0`. `upload_auton self-test: OK (31 cases against fake make and pros)`.
- Page, the full suite on a site built from the final tree, under `MemoryMax=6G`: Firefox `# tests 72 # pass 70
  # skipped 2`, Chromium `# tests 72 # pass 66 # skipped 6`. Screens (`export_robot.page.test.mjs` with
  ROUTE_MAKER_SHOTS, Firefox 21/21, Chromium 20 + 1 skipped) in `~/shulib-render-tmp/rmp1-f/shots-vfix/`.
- ARM header gate: `TU includes 259 headers` → `ARM compile gate: CLEAN`. CI greps: PROS-free, sim-free, LF (the
  changed route-builder files, the script, the goldens). `git diff -- include/ src/ Makefile test/` empty.
- `api_doc_tool`: self-test OK, check-fresh exit 0, check-examples `988 quoted lines, all verbatim`, check-coverage as
  above, check-removability `no public doc references docs/internal/`. `doc_staleness_audit.py`: self-test OK,
  `clean (37 live docs …)`.
- src gate `check`: `PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly their owed facts
  (6 refused; …))`. Self-test not run (no src/ or Makefile change).
- Not run here: the Windows CI rows, the real `pros` (never). Nothing committed.

## RMP1-G — steps you can hide, lock and focus on (overlapping steps stop getting in the way)

Executor: Claude Opus 5.5 (subagent), worktree `.claude/worktrees/rmp1-g` (branch `rmp1-g`, from `200f491`). Brief:
`docs/internal/chunks/RMP1-G-lock-hide-focus-steps.md`. Started 2026-11-19. Nothing committed.

- Log opened before any code. Reading the brief, RESUMING's standing rules, then field_view / step_list / inspector /
  project / app.
- 21:34–21:48 — built. `src/model/step_view.mjs` (new, pure): a step's `view` = {hidden, locked, section, folded}, only
  what is set; sections are by ORDER (a head's `view.section` runs to the step before the next head), so a reorder
  never leaves a stale range; `focusSet` = the step and the one before and after (the start is step 1's neighbour).
  `route.mjs`: `view` in STEP_KEYS, normalized LAST in a step, dropped by a duplicate. `project.mjs`: a `view` with a
  key or a type this builder cannot write is refused by name (D3's never-half-read rule). `field_view.mjs`: each step
  in its own `<g class="g-step">` — hidden not drawn, locked/faded points `pointer-events: none` (a click falls to what
  is under it, natively); the handles under the pointer from `elementsFromPoint`; a PRESS takes the one picked last on
  that spot (a drag moves it), a CLICK without a move picks the next one down — first cut cycled on the press, which
  made "cycle to it, then drag it" grab the one after: re-cut before any test. Hover tip names the step ("3 here").
  `app.mjs`: one delimited block (calls, the Focus tool button, `viewKey`) + `pick()` carries `here` and moves the
  focus anchor + the nudge refuses a hidden/locked step + focus off on a route switch / an open. `step_list.mjs`: eye,
  lock, ... on hover; section head rows (fold, hide/lock together, rename/remove); "N steps hidden · Show all".
  `inspector.mjs`: one line each for "k of n here", hidden, locked. `help.mjs`: the keys. `dom.mjs`: six icon paths.
- **Delta from the brief: Focus is `O`, not `F`.** `F` is Straight since RMP1-C (guide 22 :82 and :354, the changelog
  :59, `named_routes.page.test.mjs:366`, the tutorial); moving it two days before the event breaks a learned key and a
  shipped doc. `O` ("only this spot") is free; the Focus button shows it.
- 21:49 — docs: guide 22 "### 10. Crowded spots: hide, lock, focus" (a 3-row table, click-again, sections, the
  screenshot `docs/assets/route-builder/focus.png` = Firefox light 1440 focus shot), two keyboard rows, the project
  file's `view`; `docs/route-builder.md` one sentence; `docs/changelog.md` an API 2.9 entry (no API change; an older
  route builder refuses a project with a `view`, by name).
- 21:52–22:00 — mutations (driver `.claude/coordinator/scratch/rmp1-g/mutate.py`: backup under `backups/`, restore in
  `finally` and on SIGTERM/SIGHUP, `os.utime` on restore; each page mutant on a FRESH site build; Chromium under
  `systemd-run --user --scope -p MemoryMax=6G`, node under `ulimit -v 4 GiB` + `timeout 600`). All 18 CAUGHT, each by
  its intended assertion: N1 view not stored (round trip / normalize order), N2 the export skips a hidden step (D6: "gl12_fail_carry_on"
  differs from its committed .cpp), N3 the view check gone (`{"hidden":true,"colour":"red"}` opened), N4 a section's end
  off by one, N5 a duplicate keeps the view, N6 focus on step 1 loses step 1 itself; P1 hidden still drawn ("not
  drawn"), P2 a locked point takes the pointer ("the click passes through to step 1"), P3 the nudge guard gone ("no
  nudge"), P4 no cycling, P5 the press always takes the top one (the drag moves step 1, not the picked 3), P6 faded
  points clickable (step 5 selected), P7 Esc keeps focus, P8 section hide never toggles back ("all hidden: one click
  shows them all"), P9 a folded section still lists its steps, P10 a hidden step's number still on the field, P11 no
  "k of n" line, P12 focus not following the selection. A first driver run was stopped (my `pkill -f` matched its own
  shell); the restore was checked (every cut present once, tree as before) and the whole set re-run.
- Found and fixed in my own test: `here()` used `locator.textContent()`, which WAITS 30 s for an absent line — the
  "alone there now" assertion passed only after a timeout. Now a plain `querySelector`.
- Prose review (grep run, hits read, 22:05): `hidden|\block|focus|overlap|crowded|on top of each other|Duplicate|Esc\b|
  newer route builder|step list|route_maker.json` over docs/ (not internal, not api) and README.md; and every changed
  file's name (`field_view|step_list|inspector.mjs|project.mjs|route.mjs|help.mjs|dom.mjs|style.css|step_view`): no
  public doc names one. Read: guide 22 :428 (`V or Esc`: still true — Esc leaves focus only when nothing is being
  added, the new row says so), :433 (Ctrl+D: a copy now shows and is unlocked — left, it is what "duplicate" means),
  :422 (a newer project file: `view` is "a step field", still true), :449 (the project file: `view` added), guide 17
  :132 and changelog :67 (`#9-more-routes`: the anchor is unchanged). No key edit in guide 23 (no route-builder
  keys; its :253 edit, below, is the staleness gate's: corrected after the review) or the cookbook (no step-list UI).
- **Gate found on the base tree:** `doc_staleness_audit.py` exit 1 on `docs/guide/23-set-up-your-robot-first.md:253`
  ("Bench Tests tile 5 tests it" read as a claim of 5 test cases) — from `200f491` (the tutorial), not this run.
  Reworded to "Bench Tests' tile 5 checks it": `clean (38 live docs, 0 numeric claims checked)`. (After the review:
  main's `f480e08` fixed it the same evening as "Bench Tests' tile 5 checks the card"; this line is now that text.)

### Gates (2026-11-19, `date`: 21:58–22:24 EST; every one on the final tree)

- Build: `rm -f build/test/shulib_tests`, `briefing_status.py generate` (unbuilt), `cmake --build build/test -j8` → exit
  0: `doc coverage: 6502 public entities across 215 headers, all documented`, `check-nav: OK — 264 public pages`,
  `check-tone: OK — 49 pages`, `first_week: OK`; re-run (`--target shulib_doc_gates`) after guide 23's edit: the same.
- Suite (`suite_run.py green`, 4 GiB cap): `suite run: green — 3893 cases, 3775836 assertions`. Briefing regenerated
  on the built binary; `check`: `generated block current`; `self-test`: OK (38 cases).
- Node (`node --test test/*.test.mjs`, 4 GiB cap): `# tests 181 # pass 181 # fail 0` (+8: `step_view.test.mjs`).
- Page, every page test file on a site built from the final tree: Chromium `# tests 69 # pass 64 # skipped 5`,
  Firefox `# tests 69 # pass 67 # skipped 2` (the skips as before: engine-only and Windows-only cases).
  `view.page.test.mjs` with the screens (`~/shulib-render-tmp/rmp1-g/shots/`, light/dark at 1440 and 1280, both
  browsers): `# tests 9 # pass 9` in each. No browser process left.
- ARM header gate: `TU includes 259 headers` → `ARM compile gate: CLEAN`. CI greps: PROS-free, sim-free, the changed
  and new files LF. `git diff -- include/` empty.
- `api_doc_tool`: self-test OK, check-coverage as above, check-fresh exit 0, check-examples `988 quoted lines, all
  verbatim`, check-removability `no public doc references docs/internal/`. `doc_staleness_audit.py`: self-test OK (16),
  then `clean (38 live docs, 0 numeric claims checked)` (after the guide 23 rewording above).
- src gate `check`: `src build gate: PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly
  their owed facts …)`. `self-test` not run (no src/ or Makefile change, as the task allows).
- Not run: the Windows CI, the engine build (no `wasm/` or `route_preview_model.hpp` change). Nothing committed.

### Deltas from the brief, and for the coordinator

- **D6: focus is not saved in the project file** (only hide, lock and sections are): it is for now, and is turned off
  on a route switch, an open, a change to the set of routes, or an undo to another route (added after the review).
- Focus is **O**, not F (above). The row's eye / lock / ... show on hover or keyboard focus only; a hidden or locked
  row says so in its sub line ("Hidden on the field · ", "Locked · "), so the list stays quiet (the team lead's
  "overstimulating"). The nudge also refuses a HIDDEN step (it would move a point nobody can see). A duplicate drops
  the view. Sections also rename and remove; folding is a section edit (saved, undone). Focus stays on its step when
  the background is clicked, follows any selection, and is turned off by a route switch or an open. In focus, robot
  two's overlay fades too, and the start counts as step 1's neighbour. The press / click split of D4 (above).
- **RMP1-E's file, not touched:** `src/model/playback.mjs:12` `routeKey` canonicalises the whole normalized route, `view`
  included, so hiding or locking a step while a PREVIEW is shown puts the run away (it comes back on an undo or on
  showing the step again). One line there (drop each step's `view` before `canon`) keeps it; their call.
- Outside my listed files: `dom.mjs` (six icon paths appended to PATHS, after `mirror`), `help.mjs` (four rows),
  `route.mjs` (`view` in STEP_KEYS / normalizeStep / duplicateStep), guide 23 :253 (the staleness false positive),
  `PROJECT-BRIEFING.md` (regenerated). `app.mjs`: the delimited block plus small edits outside it (import, `ui`'s three
  fields, `select`/`selectHandle`'s `here`, `pick()`, the nudge guard, `ui.focus = false` in `switchRoute` and
  `adoptProject`, the Focus button state and hint in `render`, one `viewKey(e)` call in the key handler).
- An older route builder (a site or an offline file from before this) refuses a project with any `view` by name. The
  team should use the published site once a hidden/locked step or a section is saved.

### RMP1-G review findings and their disposition (2026-11-19, from ~22:45 EST; the same executor, the same worktree)

Three reviewers (correctness, tests, docs) returned 21 findings; deduped below, each re-checked against the tree before
any change. Started: reading them, the reviewers' probes (`scratch/rmp1-g/review-correctness/`, `review-tests/`).

Each finding, re-checked, then what was done (file:line as of the base `200f491` only where it says "base"):

1. **Click-through: a drag could move a step neither selected nor on top** (correctness, major) — REAL: `lastPick` was
   reset only by a drag or a pan/zoom, so a step chosen in the list, or a click on the empty field, left a stale pick.
   **Fixed at the root:** `lastPick` is gone. `field_view.mjs` pointerdown: the press takes the SELECTED point when it
   is in the stack under the pointer (a drag moves what is highlighted), else the top one; a click without a move picks
   the one under the selected (the top one when the selected is not there). The start and its heading knob match a
   selected start; a curve point matches its step when no point is picked, or the picked point. Guide 22 no longer
   says "without moving the mouse"; it says a drag moves the highlighted one.
2. **The stale-pick guard (`lastPick.keys === keys`) untested** (tests, major) — the guard no longer exists (1). The
   reviewer's scenario (cycle to step 1 of three, H, drag the spot: step 5, the top one shown, moves) is now in D4.
   R19 / R20 (the `lastPick` resets) are moot: the code is gone.
3. **Hide / lock / fold while a PREVIEW shows puts the run away, "The route changed"** (correctness major; tests
   minor; docs minor) — REAL (`routeKey` canonicalises `view` too). **`playback.mjs` is RMP1-E's: not edited.** A
   ready patch for the coordinator: `scratch/rmp1-g/routekey/routekey-for-rmp1e.patch` — `routeKey` drops each step's
   `view` before `canon`; `playback.test.mjs` asserts a fully hidden / locked / sectioned route has the key of the
   bare one and a moved number still changes it; guide 22's sentence below goes. Checked: `git apply --check` clean
   on this worktree; the new assertion RED on the unpatched `routeKey` and the playback file 19/19 GREEN patched (a
   copy under `scratch/rmp1-g/routekey/copy/`). Safe: the engine never sees `view` (engine_backend.mjs `preview` gets
   `routeLines(route, robot)`, the exported lines D6 pins byte for byte), so the key only stops counting page state.
   Until it lands, guide 22's Hide row says it as a plain fact: hiding, locking or folding while a PREVIEW shows puts
   the run away (Ctrl+Z brings it back).
4. **Focus stays on, with a stale step, after New / Delete / Duplicate route and an undo across routes**
   (correctness, minor) — REAL. Fixed: `editSet` turns focus off (the selection is dropped there too), `travel` keeps
   it only when the route shown is the same. Pinned in the new D3 test (a new route; a switch; an undo back across
   routes; and R4, opening a project: the folder's file changed under a non-empty route, "Open the folder's route").
5. **"k of n here" stale after H, L or a nudge** (correctness, minor) — REAL. Fixed in `edit()`: any edit clears it
   (what is on the clicked spot may have changed). Pinned: D4, after H "two left there now: the line goes".
6. **Deleting a section's first step drops its steps into a folded section above** (correctness, minor) — REAL, and
   the same root cause on a MOVE (Alt+Down on a section's first step put the next one in the section above). Fixed in
   the model, not the list: `step_view.mjs passSection` — the step that takes the leaving one's place takes its
   section name and fold, unless it starts a section itself; `route.mjs removeStep` and `moveStep` use it; a move to
   where the step already is returns the route unchanged. A one-step section deleted goes; moved, it goes with its
   step. The base run's node assertion "a head moved changes what its section holds" WAS this defect: replaced by the
   new node test (delete, move away, one down, a one-step section, a no-op move).
7. **The arrow keys said "move it" for a locked / hidden step with no point** (correctness, minor) — REAL. Fixed:
   `nudge` resolves the point first and returns false (as at the base) when there is none; only then the guard.
   Pinned: a locked Wait step, ArrowUp, no "unlock it" message.
8. **In focus a shared number pill stayed full and listed faded steps' numbers** (correctness, minor) — REAL (the
   builder's own focus shot: "2, 4, 6, 8, 11"). Fixed: in focus a spot's pill lists only the steps in focus (its red /
   amber from those); a spot with none in focus lists all, faded. Pinned: [A, B, A, ...] focus on step 1 → A's pill
   says "1", not "1, 3".
9. **The hidden half of the nudge guard untested (R2)** (tests, major) — added (step 2 hidden, ArrowUp: no move, "Step
   2 is hidden: show it (H) to move it").
10. **The row's eye / lock never pressed (R9, R10, R25), the step menu's Hide / Focus and the section's Rename (R21,
    R15, R16)** (tests, major) — added: a click on the eye (the row not selected by it), Tab to the lock + Enter; the
    menus in "D5 from the menus".
11. **Focus's one-highlight visuals unpinned (R5–R8)** (tests, major) — D3 now asserts `.focusing`, robot two faded,
    the lit and faded pill numbers, and that the faded start's heading knob cannot be turned.
12. **null / 5 / true views not in the refusal list (R14)** (tests, minor) — added.
13. **Other untested paths** (tests, minor) — R23 (Show all keeps a lock: D1), R11 (Esc in a curve in focus), R17 (a
    folded section keeps the selected step listed), R12 / R22 (the inspector's hidden line; Unlock unlocks), R3 (a
    switch ends focus), R4 (an open ends it), R13 (the start's "1 of 2 here"), R24 (Shift+H / Shift+L do nothing):
    each pinned.
14. **Gate and mutation records predate the final test edit** (tests minor; docs minor) — TRUE (the 21:50–21:54
    mutation run was before the `here()` fix at 21:54:26; "21:52–22:00" was loose). Every mutant re-run below on the
    final files, each page mutant on a fresh site build, with the driver's own clock times; the screens re-taken.
    Line refs: "guide 22 :82 and :354" above mixed trees: :82 is the same in both; base :354 is :377 now.
15. **Changelog "What you must do: nothing" vs an offline copy downloaded earlier** (docs, major) — REAL. The
    changelog's "What you must do" and guide 22 step 10 now say: an offline copy from before this refuses a project
    using hide / lock / sections, download it again. **For the coordinator:** the between-parts release marker
    (RMP1-PROGRESS.md line 3) is yours to declare: not touched here; the publish has to serve the new builder before
    the team saves a view.
16. **Brief D6 says focus is saved; the build keeps it only for now — an unlogged delta** (docs, minor) — TRUE; added
    to the deltas below.
17. **Merge hazard: guide 23 and PROJECT-BRIEFING against main** (docs, minor) — TRUE. Guide 23 :253 is now the SAME
    line as main's `f480e08` ("Bench Tests' tile 5 checks the card"), so the two merge as one change; the earlier
    bullet "Reworded to ... checks it" is superseded (found on the base tree; main's f480e08 fixed it the same
    evening), and "Not touched: guide 23" in the prose-review bullet was wrong. `PROJECT-BRIEFING.md` here is
    regenerated for this worktree's gates (its assertion count follows the source path's length, d79d257): **at the
    merge take main's and regenerate it in the main tree after the build.**
18. **step_list.mjs's header promised the selected card shows its buttons; the CSS did not** (docs, minor) — made the
    comment TRUE rather than weaker: `.step.selected .step-acts` shows them (one row at a time; a touch screen, the
    ≤1100 px drawer, now reaches the eye, the lock and the ... menu, which had no other way in). Pinned: the selected
    row shows them with the pointer on the field, an unselected one does not.
19. **Mutation verdicts observed against the test file before its `here()` fix** (docs, minor) — see 14.

Found while re-checking, NOT fixed (for the coordinator): a card dropped onto a section's FIRST card (the drop line
shows above that card, under the section's row) lands before that step, so in the section ABOVE; when that one is
folded, the step leaves the list once it is deselected (it still runs; unfold shows it). Sections start at their first
step by design (D5); a drop meaning "first in this section" is a list rule to decide, not a defect in the C++ (the
export never reads a section).

- Prose review after the fixes (grep run 23:08, hits read): `without moving|click (the spot|it) again|2 of 3|hover (it|the
  row)|put away|offline|Download for offline|highlighted|section` over docs/ (not internal, not api) and README.md, kept to
  route / step / preview / offline / click hits. Read: guide 22 :26 (download beforehand: still true; step 10 now says to
  download again), :140 ("the one playing is highlighted": PREVIEW's, unrelated), :160 ("Change the route and the run
  is put away (an undo brings it back)": consistent with the Hide row's new sentence), :249–:250 (rewritten: no
  "without moving the mouse"; a drag moves the highlighted one), :241 ("hover or select it": the selected row now shows
  its buttons), :257–:259 (the offline copy); `docs/route-builder.md` :42–:46 "Offline, at the event": one sentence
  added (download again after an update; an older copy cannot open a project that uses something added since, and
  names it — true of every newer-file refusal, not only `view`); changelog :26 and :30–:32. No other public doc
  describes the click-through, the row buttons or the offline copy's age.

**Mutations after the review** (driver `scratch/rmp1-g/mutate2.py`: backup under `backups2/`, restore in `finally` and
on SIGTERM / SIGHUP / SIGINT, `os.utime` on restore; node mutants: every node test file under `ulimit -v 4 GiB` +
`timeout 600`; page mutants: `view.page.test.mjs` on a FRESH site build each, Chromium under `systemd-run --user
--scope -p MemoryMax=6G` + `timeout 900`; the site rebuilt from the restored tree at the end, and every cut checked
present exactly once afterwards). First pass 22:52:43–22:57:33: 51 of 52 CAUGHT, **R4 SURVIVED** (no test opened a
project with focus on) → the "opening a project ends it" step added → R4 CAUGHT at 23:08:22. **The record: the whole
set re-run on the FINAL test files, 23:08:30–23:13:42, 52 of 52 CAUGHT**, each by the assertion meant for it:

- Node: N1 view not stored (round trip / view LAST); N2 the export skips a hidden step (D6, `gl12_fail_carry_on`); N3
  the view check gone (`{"hidden":true,"colour":"red"}` opened); N4 a section's end off by one; N5 a duplicate keeps the
  view; N6 focus on step 1 loses the start; **N7** delete without the hand-off ("step 4 still in Back, not in the folded
  Claw 1"); **N8** move without it; **N9** the hand-off onto a step that starts its own section; **N10** a move to where
  it is shifts the section (reference-equal); **N11** the fold not handed on ("the fold goes with the name"); R14
  `!plainObject` gone (`view: null` throws).
- Page: P1 hidden still drawn; P2 a locked point takes the pointer (the locked step 2 moved by a drag); P3 the nudge
  guard gone ("no nudge"); **P4** a click never cycles; **P5** the press always takes the top one ("steps 1 and 5
  stay"); P6 faded points take the pointer (the faded start's knob turned); P7 Esc keeps focus; P8 a section's hide
  never shows again; P9 a folded section lists its steps; P10 a hidden step's number drawn; P11 no "k of n"; P12 focus
  not following the selection; R2 the hidden half of the guard; R3 / R4 focus kept on a switch / an open; R5–R8 focus's
  visuals; R9 / R10 / R25 the row's eye and lock; R11 Esc in a curve; R12 / R22 the inspector's lines (R12 by the 30 s
  wait on the absent line); R13 the start's "k of n"; R15 / R16 / R21 the menus; R17 the selected step in a folded
  section; R18 focus kept on a background click; R23 Show all unlocking; R24 Shift+H; **F3a / F3b** focus kept after a
  new route / an undo to another route; **F4** "k of n" kept after H; **F6** the "move it" message for a Wait; **F7**
  the shared pill listing faded steps; **F8** the selected row without its buttons; **F10** a selected start not
  matched in a stack. R19 / R20 (the reviewer's `lastPick` resets) have no code left to cut.

**Gates after the review** (2026-11-19, `date`: 23:13–23:35 EST, every one on the final tree; logs under
`scratch/rmp1-g/`: `gates2.log`, `build2.log`, `src_gate2.log`, `page_chromium.log`, `page_firefox.log`):

- Node: `# tests 182 # pass 182 # fail 0` (+1: the section hand-off test; the refusal list +3 shapes).
- Page, every page test file, on the site rebuilt from the restored tree: Chromium `# tests 73 # pass 68 # skipped 5`,
  Firefox `# tests 73 # pass 71 # skipped 2` (+4 tests in `view.page.test.mjs`; the skips as before).
  `view.page.test.mjs` with the screens (`~/shulib-render-tmp/rmp1-g/shots2/`, light/dark at 1440 and 1280, both
  browsers): `# tests 13 # pass 13` in each. `docs/assets/route-builder/focus.png` re-taken (Firefox light 1440): the
  crowded spot's pill now says "6", not "2, 4, 6, 8, 11". No browser process of mine left (the `chrome-headless-shell`
  and `ffmpeg` running at 23:15 are the `integrate` worktree's: not touched).
- Build: `rm -f build/test/shulib_tests`, briefing generated (unbuilt), `cmake --build build/test -j8` exit 0 (`doc
  coverage: 6502 public entities across 215 headers, all documented`, `check-nav: OK — 264 public pages`,
  `check-tone: OK — 49 pages`, `first_week: OK`). Suite (`suite_run.py green`, 4 GiB cap): `green — 3893 cases,
  3775836 assertions`. Briefing regenerated on the built binary; `check`: `generated block current`; `self-test` OK.
- src gate `check`: `PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly their owed facts …)`
  (self-test not run: no src/ or Makefile change).
- ARM header gate: `TU includes 259 headers` → `CLEAN`. CI greps: PROS-free, sim-free; every changed / new TEXT file LF
  (the script's first pass flagged `focus.png`, a binary PNG: re-run with `grep -I`, clean). `git diff -- include/`
  empty.
- `api_doc_tool`: self-test OK, check-coverage as above, check-fresh exit 0, check-examples `988 quoted lines, all
  verbatim`, check-removability `no public doc references docs/internal/`. `doc_staleness_audit.py`: self-test OK (16),
  `clean (38 live docs, 0 numeric claims checked)`.
- Not run: the Windows CI, the engine build (no `wasm/` or `route_preview_model.hpp` change). Nothing committed.

**For the coordinator, after the review:** (a) `scratch/rmp1-g/routekey/routekey-for-rmp1e.patch` — RMP1-E's
`playback.mjs` (`routeKey` without `view`), its test, and the guide 22 sentence it makes untrue; route it to RMP1-E or
land it as a logged cross-file edit. (b) The between-parts release marker (line 3) is yours to declare before the
publish; the changelog now tells teams with an old offline copy to download it again. (c) At the merge: take main's
`PROJECT-BRIEFING.md` and regenerate it in the main tree; guide 23 :253 is already main's exact line. (d) Open, not
fixed: a card dropped onto a section's first card lands in the section above (see the list above). (e) Docs moved in
this pass: guide 22 (step 10: the click-again and drag words, the PREVIEW sentence, the offline copy; "hover or select
it"), the changelog's "What you must do", `docs/route-builder.md`'s offline section, guide 23 :253.

### Verification-round findings and their disposition (2026-11-19, from ~23:50 EST; the same executor, the same worktree)

A fresh verifier returned 4 minor findings (its probes under `scratch/rmp1-g/verify/`). Started: re-checking each
against the tree before any change.

Each finding, re-checked, then what was done:

1. **Fix 7's buttons covered the selected card's title** — REAL (the base fix made `.step.selected .step-acts`
   visible, but they stayed `position: absolute` over the words). **Fixed in the RMP1-G CSS block:** the selected card
   (not the start card) gets its own grid column for them (`grid-template-columns: 22px 32px 1fr auto auto`) and its
   `.step-acts` turn `position: static` with no surface box or shadow; a hovered card that is not selected keeps the
   overlay (only while the pointer is on it). The selected card's title is now cut with an ellipsis where it meets them
   ("Back up ..." at the 300 px list), never drawn under them; the inspector shows the full title right under "Step N",
   and the card's tooltip has it. Pinned in "D1 + D2 from the row": the selected card's `.step-main` ends at or before
   `.step-acts` starts, and the grip stays on the row after them. `focus.png` re-taken (Firefox light 1440).
2. **Alt+Down then Alt+Up on a section's first step left it in the section above (gone under a fold)** — REAL. A
   symmetric index rule does not exist: after Alt+Down on a head ([a | N(head) s]) and after Alt+Down on the step
   before a head ([a | N(head) s] again, s now in N's section) the route is the same shape, yet putting each back needs
   a different answer (s heads N's section / s ends the section above), so no move rule restores both; Ctrl+Z restores
   either exactly. The harm is the step leaving the list unseen, so **fixed at that root, in the model:**
   `step_view.mjs unfoldAt` and `route.mjs moveStep` — a folded section a moved step lands in unfolds (one edit, one
   undo). It covers the keys and a drop alike, so it also closes open item (d) (a card dropped onto a section's first
   card). A move inside a folded section opens it too (you see what the step passed). Pinned by a node test over every
   move (8 steps, 4 sections incl. two one-step ones, all 16 fold patterns, every from/to: 1024 moves): the moved step
   is listed afterwards and every step listed before still is; plus the verifier's own keys (Claw 1 folded, Back's first
   step Down then Up: the same order, the step listed, Claw 1 open). The existing assertion "Claw 1's first step one
   down" now expects the fold open. Guide 22's Sections paragraph says it.
3. **The hover tip named the top point while a drag moved the selected one under it** — REAL. **Fixed:** one rule for
   both, `field_view.mjs selectedIn()`: the press takes the selected point when it is in the stack, and the tip names
   that one first: "Step 1: ... (selected · 2 here: click for the next)"; with the selected point not there, the tip is
   as before ("Step 3: ... (2 here: click again for the next)"). The selected group is not raised (that would reorder
   the stack under the pointer on every pick and break cycling). Pinned in D4: on [A, B, A] with step 1 picked under
   step 3, hovering A names step 1, "selected". Guide 22 step 10: hovering names the step a drag would move.
4. **The focused pill's colour was untested** — TRUE (the verifier's M3 survived). D3 now gives step 3 (on A with step
   1) an error (`onFail: skipTo` a label nobody has): no focus, A's pill "1, 3" is red; focus on step 1, the lit "1" is
   NOT red; focus on step 3, "3" is red.

Mutations of this round (driver `scratch/rmp1-g/mutate_v.py`, the review's 52 cuts plus 9 new, backups under
`backups3/`, the same restore-on-signal and fresh-site rules; Chromium): 00:00:52–00:08:44, **61 of 61 CAUGHT**; the
tree afterwards byte-identical to a copy taken before (`verify_round/src_before_mutations`, `diff -r` empty). New:
**V1a** both new CSS lines gone (the overlay back: "the buttons sit beside its words", words to x 252, buttons from
172); **V1b** the column gone (the grip wraps: "the grip stays on the row"); **V1c** `position: static` gone (covered
again); **V2a** moveStep without `unfoldAt`, **V2b** unfolding where the step LEFT, **V2c** `unfoldAt` blind to a
section's head (a one-step folded section moved: "mask 4, 6 -> 0") — node, the new property test; **V3a** the tip
naming the top one, **V3b** the tip never saying "selected" — D4's new hover line; **V4** the verifier's M3 (the
pill's colour from every step on the spot) — D3's "not the faded step 3's red".

- Prose review of this round (grep run 00:09, hits read): `hover|names (its|the) step|fold|click (again|for the next)|
  highlighted one|its row|select it\)|step.s row` over docs/ (not internal, not api) and README.md. Read: guide 22 :241
  ("hover or select it": still true, the selected row shows them, now beside its words), :250–:251 and :254–:255 (this
  round's two sentences), :435 (the keys table's "a click again on one spot": still true), changelog :27 ("sections
  that fold, hide and lock together": still true); guide 16 :416 is PREVIEW's hover, unrelated; every other hit is
  another meaning of "fold"/"folder"/"row". In the page: `help.mjs`'s "Click again on one spot" row still true;
  `field_view.mjs`'s header now says hovering names the one a press would take (it said "names the step");
  `step_list.mjs`'s header ("on a card (hovered, focused or selected)") still true.

**Gates of this round** (2026-11-20, `date`: 00:09–00:27 EST, every one on the final tree; logs under
`scratch/rmp1-g/`: `gates3.log`, `build3.log`, `src_gate3.log`, `page3_chromium.log`, `page3_firefox.log`,
`verify_round_view_*.log`, `verify_round_shots_*.log`):

- Node (4 GiB cap): `# tests 183 # pass 183 # fail 0` (+1: the every-move property test).
- Page, every page test file, on a site built from the final tree: Chromium `# tests 73 # pass 68 # skipped 5`,
  Firefox `# tests 73 # pass 71 # skipped 2` (the new assertions are inside existing tests; the skips as before).
  `view.page.test.mjs` with the screens (`~/shulib-render-tmp/rmp1-g/shots3/`, light/dark at 1440 and 1280, both
  browsers): `# tests 13 # pass 13` in each (taken before `field_view.mjs`'s header comment was reworded: comment only).
  `docs/assets/route-builder/focus.png` = `shots3/firefox-rmp1g-focus-light-1440.png`: step 6's row "Back up ..." with
  the eye, the lock and ... beside it, nothing over its words. No browser process of mine left (the
  `chrome-headless-shell` processes running at 00:27 are `rmp1-h`'s and `integrate`'s: not touched).
- Build: `rm -f build/test/shulib_tests`, briefing generated (unbuilt), `cmake --build build/test -j8` exit 0 (`doc
  coverage: 6502 public entities across 215 headers, all documented`, `check-nav: OK — 264 public pages`, `check-tone:
  OK — 49 pages`, `first_week: OK`). Suite (`suite_run.py green`, 4 GiB cap): `green — 3893 cases, 3775836 assertions`.
  Briefing regenerated on the built binary; `check`: `generated block current`; `self-test` OK (38).
- src gate `check`: `PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly their owed facts
  …)` (self-test not run: no src/ or Makefile change).
- ARM header gate: `TU includes 259 headers` → `CLEAN`. CI greps: PROS-free, sim-free; every changed / new TEXT file LF
  (`grep -I`; the script's plain pass flags the binary `focus.png`, as last round). `git diff -- include/` empty.
- `api_doc_tool`: self-test OK, check-coverage as above, check-fresh exit 0, check-examples `988 quoted lines, all
  verbatim`, check-removability `no public doc references docs/internal/`. `doc_staleness_audit.py`: self-test OK (16),
  `clean (38 live docs, 0 numeric claims checked)`.
- Not run: the Windows CI, the engine build (no `wasm/` or `route_preview_model.hpp` change). Nothing committed.

**For the coordinator, after this round:** open item (d) above (a card dropped onto a section's first card lands in
the section above) is now harmless: that section opens if folded, so the step is never put away unseen; WHICH section
a boundary drop means is still the list's rule (index moves cannot be exactly reversible across a section's edge,
finding 2). (a)–(c) of the review stand unchanged (the `routeKey` patch for RMP1-E, the release marker, the briefing at
the merge).

## RMP1-H — the field's solid structures: drawn, and every step checked against them

*(Started 2026-11-19, ~23:15 — the Start entry's time was not recorded; the first timed entry is D1's 23:16; the builder, Opus 5.5, in the worktree
`.claude/worktrees/rmp1-h` from `518717a`. Brief: `RMP1-H-field-obstacles.md`, uncommitted, landing with this run.)*

- **Start.** Read RESUMING's standing rules and the brief in full. Plan: D1 first (the manual) — if it cannot be
  fetched, STOP before any number.
- **D1 — the manual, read (23:16–23:35).** v2.0's PDF (`.claude/coordinator/scratch/autoflip/dl/override-2.0.pdf`,
  2026-09-03) Appendix A rendered (`scratch/rmp1-h/manual/a-100..111.png`: A5–A16); **v2.1 READ this time**: the online
  manual's Appendix A page (manuals.vex.com/v5/override/v2.1/appendix-a/) links its drawings, downloaded to
  `scratch/rmp1-h/v21dl/` (FieldElementLocations, GoalSpecs, LoaderSpecs: release 2026-04-27, every number the same as
  v2.0's). The solid floor structures: the **nine Goals** (A10 places them; A7 the base: 142.5 mm = 5.61 in flat to flat,
  R81.9 mm = 3.22 in corners — "octagonal", the glossary; the black 5.77-in and centre 8.77-in goals' bases are drawn
  332 / 330 px on the sheet where the red's 5.61 in spans 332 px) and the **four Loaders** (A9: 4.02 in wide, 3.74 in
  into the field; A10: 58.764 in either side of centre on the red and blue walls — the library's HA-212 numbers). NOT:
  the Toggles (on the wall's top, brackets 10.6 in up), the tape, the Pins and Cups. F1 from A10's audience view:
  x = 70.205 - a_y, y = a_x - 70.205 — each goal's colour checked against A10 and HA-243's Quadrants.
- **The picture (field.webp) is a PERSPECTIVE render** (found measuring it, `scratch/rmp1-h/probe/`): the walls' inner
  faces show as a strip, and the Loaders' 23-in tops are drawn ~3 in outward of their floor footprints — the floor
  footprint sits on the Loader's faint clear base (`probe/loader_zoom.png`). The goals (3–9 in tall) sit on their
  outlines: 92–97 % of each outline's inside is not floor, 0 % of a 0.6–1.6 in ring outside it is (`probe/scores.py`).
- **Built:** `test/override_field_obstacles.hpp` (the data, each number cited), `route_maker_describe.hpp` emits
  `field.obstacles` (id, name, kind, x, y, source, footprint on the 0.001 grid), fixtures robots/x|y|z.json re-made;
  `src/model/obstacles.mjs` (signedGap: SAT, the push-out depth; the sweep = pair.mjs's stepPoses with each two
  neighbours' hull; RCO1 red > 0.5 in in, RCO2 amber < 1 in, RCO3 robot two's red; detour; runThrough for PREVIEW);
  checks.mjs one line + `detour` fix + RCO3 in ROBOT_TWO_ONLY; route.mjs `touches` (STEP_KEYS, normalized sorted);
  playback.mjs (routeKey ignores `touches`; the page's THROUGH a structure when it comes first); preview.mjs passes the
  structures; timeline.mjs "THROUGH the <element>." (was "a wall"); `src/ui/field_obstacles.mjs` (outlines + the one
  highlight + the "on purpose" switches); hooks: field_view.mjs (import + `parts = [obstacleShapes(app, P)]`),
  inspector.mjs (import + `touchOnPurpose(step, i, app)`) — both RMP1-G's files (coordinator). CSS: one delimited block.
- **What the check found in the TREE's own test routes (true, each):** the routes starting at (-36, -55.7) and driving
  along y = -55.7 or to (-12, -36) go 1.7–10 in through the black goal by the red wall; the old ready route (-36, -60)
  -> (-24, -36) the same; four recorded PREVIEW fixtures (ready, failed, pair_fail, through) through goals; 12 of the
  goldens' TEST routes (`probe/goldens_rco.mjs`) — those are left as they are (the C++ goldens stay byte-for-byte;
  they are the exporter's test routes, nothing checks their page issues). Test routes moved clear: checks.test.mjs
  (clean route along y = -60), named_routes.test.mjs D1, page.test.mjs (the ready route 24 in right; the clicked
  route's start (-9, -55.7); the ONE FILE leg through the LEFT wall), named_routes.page.test.mjs ((24, -48) is IN the
  red goal by the red wall: (24, -60)). The PREVIEW page files (preview.test.mjs, mechanisms.page.test.mjs) are baked
  without the structures (`harness.mjs noStructures()`, said why): re-recording those fixtures is for after the 21st.
- **RESUMED 2026-11-20 01:43 EST (`date`)** after the 5-hour cut (~00:25). Found: `git diff -- src/ Makefile` EMPTY (no
  gate plant left); every mutated file byte-equal to its pre-mutation md5 (`scratch/rmp1-h/mut/md5-before.txt`,
  `md5-ui-before.txt`: `md5sum -c`, all OK). Done before the cut but not logged: the JS mutations M1–M19 (all CAUGHT,
  `mut/verdicts-js.json`), U1–U4 and C1–C4 (all CAUGHT, `mut/log-*.txt`), the screens (both browsers, `shots/`), the
  final page run (stopped in Firefox's mechanisms file). Missing: the guide's picture (`structures.png` is cited, not
  in the tree), U1–U4 re-run (the page test was edited at 23:59, after them), the gates, the prose review. Plan: a
  fresh build from source (`rm -f build/test/shulib_tests` first), then those.
- **Built and run fresh (01:44–02:05).** `rm -f build/test/shulib_tests build/test/shulib_route_maker`, briefing
  regenerated, `cmake -S test -B build/test` + `cmake --build build/test -j8` + `--target shulib_route_maker`. Suite
  (capped): **3,900 cases / 3,776,969 assertions, 3 skipped — SUCCESS** (RMP1-H's 4 cases: 732 assertions). Node:
  **188/188**. The page files, both browsers, on a site re-built from source (`scratch/rmp1-h/pagelogs/final-*.log`):
  obstacles with the screens 10/10 each; every file green — Firefox controls 26, mechanisms 8 (+1 skip), named_routes 7,
  obstacles 4, page 18, preview 21 (+1); Chromium controls 25 (+1), mechanisms 8 (+1), named_routes 7, obstacles 4,
  page 16 (+2), preview 21 (+1); engine skipped in both (every skip a browser branch or the real engine, as before).
  The native twin re-records every PREVIEW fixture from the new robots.json **byte-identical** (`record.mjs --out
  scratch/rmp1-h/native_records`): the engine ignores `field.obstacles`. The wasm tests skip (no emsdk here: CI's).
- **Mutations — every one observed RED, restored byte-equal (md5), the file's time bumped.** JS (`mut/jsmut.py`, node
  capped, before the cut, against today's files: md5 equal): M1 TOUCH_IN 0.6, M2 CLEAR_IN 0.9, M3/M4 amber/red not
  judged on the step, M5 no hull between samples, M6 overlap length for the push out, M7 the start unchecked, M8 robot
  two unchecked, M9 any touch THROUGH, M10 the page's THROUGH always wins, M11 the key carries `touches`, M12 not
  sorted, M13 the way around may leave the field, M14 a turn worded as a drive, M15 no way around, M16 on purpose
  ignored, M17 RCO3 waits PREVIEW, M18 the way around checks no other structure, M19 no structure checked — all CAUGHT
  (`mut/verdicts-js.json`). Page (`mut/uimut.py`, the site re-built from each mutant, Firefox in a 6G scope):
  **U1–U4 RE-RUN at the resumption** (the page test changed at 23:59, after their first run): U1 no highlight, U2 not
  drawn, U3 no on-purpose switch, U4 PREVIEW given no structures — all CAUGHT. C++ (the dependents touched, the fast
  targets re-built, the suite capped): C1 a goal 2 in off the drawing (suite: the golden, the A10 grid and mirror
  checks; page: "red-goal-right-wall: 65 % of its inside and 24 % of the ring around it is NOT floor"), C2 the Loaders
  3.0 in deep (the plant's Loaders), C3 the corners R90 mm (A7's R81.9), C4 the description drops the Loaders — all
  CAUGHT.
- **Gates (final):** src_build_gate check PASS (14 builds; self-test not needed: no src/ or Makefile change); the ARM
  header gate CLEAN (259 headers); PROS-free and sim-free greps hold; api_doc_tool self-test / check-coverage /
  check-fresh / check-examples / check-removability OK (no `///` under include/ changed: docs/api not regenerated);
  briefing self-test + check OK; doc_staleness_audit self-test + run clean; doc_readability self-test / check-nav /
  check-tone OK. `git diff -- include/ src/ Makefile` EMPTY.
- **Docs:** guide 22's picture `docs/assets/route-builder/structures.png` (the Chromium light 1440 screen); guide 22's
  checks table now says "at the start or anywhere on a step" (RCO1 checks the start too).
- **Prose review (greps run 01:58, `scratch/rmp1-h/prose/greps.txt`, hits read):** each changed file's name
  (`override_field_obstacles`, `route_maker_describe`, `obstacles.mjs`, `field_obstacles`, `checks.mjs`, `playback.mjs`,
  `route.mjs`, `field_view.mjs`, `inspector.mjs`, `preview.mjs`, `timeline.mjs`, `style.css`) — only
  tools/route_maker/README.md, already updated; the concepts `THROUGH a wall`, `through a wall`, `no walls`,
  `Loader`, `touches`, `newer builder`, `field.obstacles`, `solid structure`: guide 16 l.454/518 and the glossary's
  "route preview" describe `tools/route_preview.py` (RPV1's walls and Loaders: still true — the CLI does not check
  goals), guide 14 l.70 (the plant has no walls: true), guide 22 l.180 (true), README l.85/191 (true: an older page
  refuses `touches` by STEP_KEYS), cookbook 09 l.117 (Loaders do not mirror: agrees with RCO3). Stale: guide 22's
  table row (the start) — fixed.
- **Deltas from the brief, each with its reason.** (1) D1's footprint-on-picture test is the PAGE's (the browser
  decodes field.webp; the suite has no image decoder); the suite pins the footprints to A7/A9/A10 instead and to the
  two other copies of the Loaders (the library's keep-outs, the host test plant's field). (2) The Loaders' picture
  threshold is looser than the goals' (≥ 15 % not floor inside, vs 85 %): the render is a perspective, their 23-in tops
  drawn ~3 in outward. (3) Added RCO3, robot two's mirrored path red (the Loaders are not mirror images); PREVIEW does
  not wait for it, as for RCT1/RCP1/RCM1. (4) A step is judged on what IT does: one that starts touching and backs
  away is clean (placing a cone, then leaving). (5) "Go around: add a point" only on a drive or back-up to a typed
  point; a curve says "move a curve point", a turn "move the step before it". (6) PREVIEW's THROUGH a structure is the
  page's (`runThrough`, RPV1's 1 in allowance) because D5 keeps the engine; robot one's square is used for robot
  two's run too (both 15 in this season). (7) Two hook lines in RMP1-G's `inspector.mjs` (D4's checkbox needs one),
  beside the one in `field_view.mjs`. (8) preview.test.mjs and mechanisms.page.test.mjs baked without the structures
  (`harness.mjs noStructures`): four recorded fixtures drive through a goal; re-recording them clear is after the 21st.
- **Stopped here: the run is complete, uncommitted, for the coordinator's verification.** For the coordinator: RMP1-G
  merged on main (`e982165`) after this base — `field_view.mjs` (import + `parts = [obstacleShapes(app, P)]`) and
  `inspector.mjs` (import + `touchOnPurpose(step, i, app)`) need a merge by hand there.

### RMP1-H review findings and their disposition

*(Started 2026-11-20 02:27 EST (`date`); the builder again, Opus 5.5, same worktree. Three reviewers' 24 findings,
deduplicated below; each re-checked before it is fixed or answered.)*

Probes and drivers: `.claude/coordinator/scratch/rmp1-h/fix/` (`probe/`, `mut/rmut.py`, `mut/verdicts.json`, logs).

1. **A cone placed on a red or blue goal is red, and Export waits (correctness, major) — REAL; FIXED, a decision for
   the coordinator's ruling (change 9 below).** A7 re-read (`v21dl/GoalSpecs.png`): the red goal's base is 142.5 mm
   with a 10 mm lip; the body is ~132 mm wide just above the lip and narrows to 88.8 mm at its 82.5 mm top, so a front
   at 1 / 2 / 3.25 in up sits 0.39 / 0.68 / 1.06 in inside the floor footprint. The black and centre goals stand full
   width to 64 / 140.2 mm; the Loaders straight (A9). Fix: `override_field_obstacles.hpp` `kColourGoalOverhang` =
   (142.5 - 88.8) / 2 mm, cited, a per-structure `overhang` (the four red and blue goals; 0 for the rest) emitted as
   `field.obstacles[].overhangIn`; a step that ENDS at a structure it touches on purpose may go max(0.5, overhangIn)
   in; unsaid, it stays red (the default is unchanged), and the red then says "If its front reaches over the base on
   purpose (placing a cone), say so below" with the switch offered. Rejected: amber up to the overhang without the
   switch (a mistaken 1 in end overlap would only be amber). Tests: node "a red or blue goal…" (0.9 in unsaid red,
   said clear, 1.1 in said red, a black goal said still red); C++ "the overhang is A7's" and the description's counts.
2. **Guide 22's pictures and quoted line show the old ready route (correctness minor + docs major, the same) — REAL;
   FIXED but two.** Line 218 now quotes `DRIVE to (0, -36) in: about 26.8 in`. `route.png`, `route-dark.png`,
   `export.png` (the `export-copy-only` screen) and `saved.png` re-taken from the moved ready route (Firefox, light /
   dark 1440, RMP1-A2u's `to_docs.py`: 1200 x 750, 256 colours). **KNOWN-STALE: `preview.png` and
   `preview-failed.png`** — they play the recorded `ready` and `failed` fixtures, made before the structures and both
   through a goal (the PREVIEW pages run baked without the structures, `harness.mjs noStructures`). After the 21st:
   move those fixtures' routes clear as `readyRoute` was, re-record (`test/fixtures/record.mjs`), drop
   `noStructures`, re-take both pictures. Also found: `structures.png` was committed at 1440 x 900 RGB, not the
   convention — re-made by `to_docs.py`.
3. **The merge with RMP1-G conflicts in more files than the handoff named (correctness minor + docs minor, the same)
   — REAL.** A dry run (`patch -p1 --dry-run` of `git diff 518717a` onto shulib-v2's copies, `fix/mergecheck/`):
   hunks FAIL in `route.mjs` (STEP_KEYS), `style.css` (the end), `field_view.mjs`, `inspector.mjs`, the README (rows
   11/13) and guide 22 (the project-file paragraph); the changelog, guide 16 and page.test.mjs apply with offsets.
   The handoff below is rewritten. A malformed `touches` (not a list of strings) is dropped by `normalizeStep`, not
   refused: `project.mjs` is RMP1-G's file (not edited here) — the follow-up for after the merge is beside RMP1-G's
   `view` check in `newerParts`: `if (s.touches !== undefined && !(Array.isArray(s.touches) && s.touches.every((t) =>
   typeof t === 'string' && t))) return \`a "touches" of ${JSON.stringify(s.touches)} (${at})\`;`.
4. **The checks are 100–1000x slower; a drag re-checks (correctness minor) — REAL; FIXED.** Measured on the
   reviewer's 21-step routes (`fix/probe/perf.mjs`; the pre-review code on a copy, `perf_base.mjs`): before 10.1 ms
   (the "clean" one, which has two reds), 45.7 ms (red), 49.5 ms (mirrored), a drag 46.3 ms a move; after 2.4, 3.1,
   4.1, 3.3 ms (no structures: 0.14). How: the way around is searched only for a red ON THE WAY (no point clears one
   where a step ends) and remembered per field by everything it depends on (from, to, kind, turned first, the
   structure, size, wall, `touches`); the sweep skips a structure the step's whole box is far from; `signedGap`
   returns a bound once a gap passes CLEAR_IN + 1 in; the projections allocate nothing.
5. **Wording (correctness minor) — REAL; FIXED.** (a) A Loader: "taking a load", in the amber and on the switch;
   (b) a red forward / backward: "Change the inches."; (c) a red's depth rounds UP, an amber's DOWN, and a limit met
   exactly is not past it (0.5 in is amber, says 0.5; 0.499 says 0.4; 0.51 red says 0.6) — so the guide's "more than
   0.5 in" and the number agree; (d) the C++ test comment says "centre".
6. **PREVIEW's turned square unpinned (R2) — REAL; test added** (a sample turned 45 deg 12 in from the centre goal:
   THROUGH 1.41 in; square to it: clear).
7. **The "on purpose" offer untested (N1, N2) — REAL; tests added, and the rule sharpened:** a step with a red no
   switch clears offers none (one idea at a time); else one switch per structure it comes close to. Node:
   `nearStructures` on an amber (exactly the centre goal), a red (none), a clean step (none), a liftable end (that
   goal). Page: exactly one switch on the amber step, none on the red one or on a clean one.
8. **The gap one way only (G1) — REAL; tests added:** square-on, 0.5 in short of the flat (amber "passes 0.5 in");
   `signedGap` corner to corner is the diagonal (sqrt 2), never an axis's bound.
9. **"Go around" never turns the second leg (D1) — REAL; test added** (a drive not turned first: both legs turned
   after the fix, no error).
10. **Robot two's mirrored start in a Loader (S1) — REAL; test added** (RCO3 at the start, robot two only).
11. **Robot two's PREVIEW run (P2) — REAL; test added** (`p.two.through`).
12. **The deepest named (W1) — REAL; test added** (centre goal 4.x in, not the clip).
13. **The start's 0.5 in edge (T0) — REAL; test added** (0.49 clear, 0.51 and 1.0 red).
14. **Untick, the highlight's step, the moment's square (T1, T2, V1) — REAL; page assertions added** (untick brings
    the amber back; the wait selected lights nothing, the drive its goal; the moment's polygon equals
    `squareCorners(issue.obstacle.at)` within 0.01 px).
15. **The way around's preferences (I1, D3, D5, D6, D2) — I1 and D3 REAL, tests added** (a back-up detours as two
    back-ups; a line just above the goal's centre goes round above it, below it below). **D5, D6, D2 deferred to
    after the 21st, with the reason:** they choose WHICH point is offered or whether one is; the point applied is
    re-checked like every step (anything it comes near shows at once), so none can let a collision through.
16. **The logged doc gates predate the last guide edit (tests minor) — REAL (a log inaccuracy, the gates were
    green).** Every gate is re-run this round AFTER the last doc edit (below).
17. *(= 2.)*
18. **The 0.5 in allowance and the switch applied anywhere on a step, not only where it ends (docs major) — REAL: an
    unlogged departure from D3/D4. FIXED to the brief:** on the way any overlap (past 0.01 in) is red and the switch
    never lifts it; the 0.5 in touch and the switch's overhang are for where the step ends (`sweep`'s `end`: its last
    pose is its deepest). The switch still silences an amber anywhere (D4 does not limit it). Tests: "on the way, ANY
    overlap is red" (0.3 in mid-pass red, said or not; 0.02 in red reads 0.1; flush amber).
19. **Brief §1 was wrong about the Loaders (docs minor) — REAL; logged here.** The host test plant's field
    (`route_preview_model.hpp`, Loader = `BoxRole::Solid`), the library's wall keep-outs and RPV1 all had them, and
    PREVIEW already marked THROUGH more than 1 in into a Loader. The changelog now says "PREVIEW now marks a run
    THROUGH a goal (it already did for the Loaders)". RPV1's own Loader words ("past its face: the host test plant
    has no walls") are left: RMP1-A2u's, pinned by `playback.test.mjs` and `preview.test.mjs`, and true (the plant
    has neither).
20. **Guide 16 "the only field parts shulib knows" (docs minor) — REAL; FIXED** there and in the RPV1 page's own
    sentence (`test/route_preview_render.hpp` `kFieldLine`; no golden holds it): "the field parts the host test
    plant's field model has … the goals … not drawn or checked here (the route builder checks the goals)".
21. **The data header's 5.77 / 8.77 in read as widths (docs minor) — REAL; FIXED** ("5.77 in tall", "8.77 in tall").
22. **Guide 22's section broader than the page (docs minor) — REAL; FIXED:** "Select the step: …"; "A drive or a
    back-up to a point offers Go around … when there is a clear way round … both legs turn to face their point"; the
    depths (item 5c).
23. *(= 3.)*
24. **The PROGRESS header promised a `date` (docs minor) — REAL; FIXED** (the header now says the Start entry's time
    was not recorded; the first timed entry is D1's 23:16).

**Change (9), for the coordinator's ruling — the on-purpose overhang (item 1).** **Change (10) — not a change: item
18 restores the brief's D3/D4** (the first round's mid-path allowance was the unlogged departure).

**Mutations this round (`fix/mut/rmut.py`, every one observed RED, each file restored byte-equal — `md5sum -c
mut/md5-before.txt` all OK — its time bumped, and its dependents re-built from the restored file):** node (all
`tools/route_maker/test/*.test.mjs`, capped): R2, N1, N2, G1, D1, S1, P2, W1, T0, I1, D3, and the new rules' O1
(overhang ignored), O2 (0.5 in allowed on the way), O3 (the switch lifts a red on the way), O4 (every step judged as
ending there), O5 / O6 (depths rounded to nearest), O7 (a Loader for a cone), O8 (a straight drive told to move its
point), O9 (a limit met exactly is past it), O12 (the step's box from its first pose), O13 (every gap an axis's bound)
— all CAUGHT. Page (the site re-built from each mutant, Firefox in a 6G scope, `obstacles.page.test.mjs`): T1 (cannot
untick: the uncheck never takes, Playwright's 30 s), T2, V1, N3 (the page offers all 13) — all CAUGHT. C++ (the
including tests touched, the fast targets re-built, `-tc="RMP1*"` capped): C5 (no overhang), C6 (the lip's width read
as the top's) — CAUGHT by the stale-fixture check and the description's overhang count.

**The merge onto main, for the coordinator (replaces the first round's two-file note).** From the dry run (item 3),
by hand: `tools/route_maker/src/model/route.mjs` — STEP_KEYS keeps BOTH `'view'` (RMP1-G) and `'touches'`, and
`normalizeStep` keeps both RMP1-G's `view` block (its copy loop skips `view`) and this run's `touches` block, or a
saved project with either is refused as "from a newer builder"; `src/style.css` — both delimited blocks at the end,
RMP1-G's and `/* ── Field obstacles (RMP1-H) ── */`; `src/ui/field_view.mjs` — the import and `parts =
[obstacleShapes(app, P)]`; `src/ui/inspector.mjs` — the import and `touchOnPurpose(step, i, app)`;
`tools/route_maker/README.md` rows 11 and 13 (RMP1-F's and RMP1-G's words beside this run's); guide 22 (the
project-file paragraph's `touches` beside RMP1-G's view words). With offsets: the changelog, guide 16,
`page.test.mjs`; and `RMP1-PROGRESS.md`, `PROJECT-BRIEFING.md` (regenerate). After it: the node suite
(`named_routes` "saves to the same bytes", `obstacles` "the flag is page-only"), the page tests, and the `touches`
shape check in `project.mjs` `newerParts` (item 3).

**Docs this round:** guide 22 (the section: red on the way / where a step ends, "Select the step", Go around on a
drive or a back-up when there is a clear way, both legs turned, the Loader's "take a load", the red and blue goals'
1.1 in with the switch, PREVIEW's 1 in; the checks table's row; line 218's quoted comment), its pictures `route.png`,
`route-dark.png`, `export.png`, `saved.png`, `structures.png` (re-taken, item 2; `preview.png` and
`preview-failed.png` KNOWN-STALE, item 2); the changelog's entry; guide 16 and the RPV1 page's `kFieldLine` (item
20); the data header's comments (items 1, 21). Not touched, with why: guide 02's line (true: the goals' places and
the Loaders'); `docs/route-builder.md` (its `route.png` re-taken; its words true); the route builder README (its
rows name the modules, not the rules; true); `test/README.md` (RPV1's THROUGH and TOUCH: true).
**Prose review (greps at 02:50, `fix/prose-greps.txt`, every hit read):** `overhang`, `on the way`, `more than 0.5
in`, `0.5 in into`, `taking a load`, `placing a cone`, `Go around`, `only field parts`, `field parts`, `-24, -36`,
`-36, -60`, `Change the inches`, `route_preview_render`, `kFieldLine`, `Touches the`, `touches it on purpose`,
`THROUGH a goal`, `THROUGH it`, `solid structure`, `Loader`, and each re-taken picture's name, over `docs/` (less
`docs/internal/`, `docs/api/`), `README.md`, the route builder's and `test/`'s READMEs. Stale: none left (guide 21
l.366's (-36, -60) is the twin-mirror picture's own example leg, not the route builder's ready route; guide 16
l.459-519, `test/README.md` l.268, cookbook 09 l.117, guide 02 l.206 describe RPV1 or the mirror: true).

**Gates, final, on the final tree (02:47–03:04, every doc gate after the last public doc edit at 02:46; logs
`fix/gates/`, `fix/pagelogs/`):** `rm -f build/test/shulib_tests build/test/shulib_route_maker`, the briefing
regenerated, `cmake -S test -B build/test` + `cmake --build build/test -j8` (0 warnings), the briefing regenerated
on the fresh binary. Suite (capped, 36 s, 66 MB): **3,901 cases / 3,776,984 assertions, 3 skipped — SUCCESS** (the
one new case: the overhang). Node: **198/198**. Pages on a site re-built from the final sources, every file, both
browsers: Firefox controls 26, engine 0 (+1 skip), mechanisms 8 (+1), named_routes 7, obstacles 4, page 18, preview
21 (+1); Chromium controls 25 (+1), engine 0 (+1), mechanisms 8 (+1), named_routes 7, obstacles 4, page 16 (+2),
preview 21 (+1) — the same skips as the first round (browser branches, the real engine); no Playwright browser left
(`pgrep -af pw-browsers`: none). The screens: `fix/shots/` (obstacles light / dark at 1440 and 1280 in both browsers,
10/10 each; page.test's in Firefox, 33/33). ARM header gate: 259 headers, CLEAN. PROS-free and sim-free greps hold.
`git diff 518717a -- include/ src/ Makefile`: EMPTY. api_doc_tool self-test / check-coverage (6,502 entities) /
check-fresh / check-examples (988 lines) / check-removability OK (no `///` changed: docs/api not regenerated);
briefing self-test + check OK; doc_staleness_audit self-test + run clean; doc_readability self-test / check-nav /
check-tone OK. src_build_gate check: **PASS** (14 builds; self-test not needed: no src/ or Makefile change).
`git status`: nothing committed; the worktree's `bin/` and `.d/` are the gate's (gitignored).

**Stopped here: the review round is complete, uncommitted, for the coordinator's verification.** Open for the
coordinator: change (9)'s ruling (item 1); the merge by hand (above); `preview.png` / `preview-failed.png` and the
four fixtures to re-record after the 21st (item 2); D5, D6, D2 tests after the 21st (item 15); the `touches` shape
check after the merge (item 3).

### Verification-round findings and their disposition

*(Started 2026-11-20 03:19 EST (`date`); the builder again, Opus 5.5, same worktree. A fresh verifier's three findings;
each re-checked before it is fixed or answered. Probes and drivers: `.claude/coordinator/scratch/rmp1-h/vfix/`.)*

1. **Change 9 does not work on a mirrored route, the page's default (major) — REAL; FIXED.** Re-checked: the
   verifier's `v1_mirror_overhang.mjs` re-run on the tree — said, robot two's RCO3 "goes 0.9 in into the red goal by
   the red wall … (the Loaders are not mirror images)", an error Export refuses; no switch clears it. Root: robot two's
   check took no "on purpose" at all. The mirror ((x, y) -> (-y, -x)) maps every goal onto a goal of its own colour
   (red by the right wall <-> red by the red wall, blue <-> blue, black <-> black, the centre onto itself) and no
   Loader onto a Loader. Fix (`obstacles.mjs`): `twinsOf` — each structure the step touches on purpose, its whole
   footprint reflected, matched vertex for vertex (0.01 in) to the structure standing there; that set is robot two's
   "on purpose" (a Loader has no twin). RCO3 says "(the Loaders are not mirror images)" only when it IS a Loader. After:
   said, only RCT1 (the "together?" question) is left; "one at a time" said, nothing. Test "the overhang on a mirrored
   route (the page's default)": `newRoute()` pinned mirrored; said 0.9 in → none; unsaid → RCO1 with the switch, not
   RCO3; 1.1 in said → RCO1; a 16 in robot two (0.5 in deeper): unsaid → RCO3, its exact words with no Loader clause;
   said → none; the twin's own id on robot one's step → RCO3. The Loader test now pins its clause. Guide 22: "the
   switch carries over: robot two touches the mirror-image goal".
2. **The docs' "1.1 in" vs the page (minor) — REAL; FIXED at the root, which is wider than the finding.** Re-checked
   (`v2_words.mjs`: said, 1.06 / 1.08 / 1.1 in all read "1.1 in into … it is solid"). Also found beside it: an
   UNSAID end 1.0–1.057 in read "1.1 in into the … base … say so below" (`v3.out`'s third case) — the same number for
   a red the switch clears and one it does not; and PREVIEW marks a run THROUGH past 1 in (`THROUGH_IN`, RPV1's), so a
   step said and ending 1.0–1.057 in over a base was clear in the check while its run, on the plan, is THROUGH. Three
   limits sat within 0.06 in of each other. Fix: the check's on-purpose limit is `overOf` = min(overhangIn,
   THROUGH_IN) = **1 in** — the walls' precedent (`checks.mjs` `WALL_SLACK_IN` is RPV1's THROUGH too); the data keeps
   A7's 1.057 (no C++ change). Now: said, ≤ 1 in clear, past it red reading ≥ 1.1; unsaid, ≤ 1 in reads ≤ 1.0 with
   the switch, past it reads "it is solid" and promises nothing. The base words take the same floating-point NOISE as
   the check (a synthetic goal whose depth computes as 1.0000000000000013: the words promise the switch and the switch
   clears it). Docs: guide 22 l.101 and the checks table, the changelog — "1 in", now exactly the page's number.
   **Change (9), amended for the coordinator's ruling:** the on-purpose limit over a red or blue goal's base is 1 in
   (PREVIEW's THROUGH), not A7's 1.057 in.
3. **"Say so below" with no switch below (minor) — REAL; FIXED as suggested.** Re-checked (`v3_promise.mjs`: 3 of
   368). A red no switch lifts (`goesIn(h, ALL)`) is now named first; a liftable one only when there is none, which is
   exactly when `nearStructures` offers switches. After: the same search, run to 19,852 drives, 8,338 promising the
   switch, 0 without it. Test: the verifier's route (the turn in place clips the black goal by the left wall: that red
   is named, `nearStructures` []), and the verifier's generator in the node suite (seed 7, 1,000 drives): every "say
   so below" has its structure's switch; over 100 reached.

**Mutations this round (`vfix/mut/vmut.py`; every node test file, capped; `obstacles.mjs` restored byte-equal —
`md5sum -c` OK — its time bumped):** V1 robot two never on purpose, V2 robot two given robot one's ids (the
verifier's V-two-touches), V3 robot two everything on purpose, V4 a twin by vertex count only, V5 a twin is itself,
V6 the Loader clause always, V7 never, V8 the liftable red first, V9 the overhang past PREVIEW's THROUGH, V10 the base
words without NOISE (SURVIVED at first: the exact 1 in end computes as exactly 1; the synthetic float-noise goal
added; re-run CAUGHT), V11 the base words past the cap, V12 the overhang ignored — **all CAUGHT** (`vfix/mut/verdicts.json`).

**Docs this round:** guide 22 (the overhang's "1 in" twice; the mirrored bullet: the switch carries over), the
changelog ("1 in"). Not touched, with why: `test/override_field_obstacles.hpp` (states A7's physical 1.057 in, still
true; the page's cap is the page's rule, said in `obstacles.mjs`); the route builder README (its rows name modules,
not limits); `structures.png` (a drive through the centre goal: its words did not change).
**Prose review (greps at 03:26, `vfix/prose-greps.txt`, every hit read):** `1.1 in`, `overhang`, `1.057`, `mirror
images`, `mirror-image goal`, `say so below`, `switch carries`, `up to 1 in`, `touches it on purpose`, `on purpose`,
`robot two's mirrored path`, `RCO3`, over `docs/` (less `docs/internal/`, `docs/api/`), `README.md`, the route
builder's and `test/`'s READMEs. Stale: none (`1.1 in`: gone; `overhang` in guide 14 / hardware-assumptions is the
Toggle's; "mirror images" in guides 02 / 17 / 19, cookbook 09, HA-258 are the Quadrants' or a tank's; guide 02 l.207
"A step at a Loader has no mirror" agrees with `twinsOf`).

**Gates on the final tree (03:27–03:45 EST, `date`; logs `vfix/gates/`, `vfix/pagelogs/`):** `rm -f
build/test/shulib_tests build/test/shulib_route_maker`, the briefing regenerated, `cmake -S test -B build/test` +
`cmake --build build/test -j8` (0 warnings; no C++ changed this round, so it re-linked), the briefing regenerated on
the fresh binary. Suite (capped, 36 s, 66 MB): **3,901 cases / 3,776,984 assertions, 3 skipped — SUCCESS**. Node:
**201/201**. Pages on a site re-built from the final sources, every file, both browsers: Firefox controls 26, engine 0
(+1 skip), mechanisms 8 (+1), named_routes 7, obstacles 4, page 18, preview 21 (+1); Chromium controls 25 (+1), engine
0 (+1), mechanisms 8 (+1), named_routes 7, obstacles 4, page 16 (+2), preview 21 (+1) — the same skips as before; no
Playwright browser left. The screens (`vfix/shots/`: obstacles light / dark at 1440 and 1280, both browsers, 10/10
each). ARM header gate: 259 headers, CLEAN. PROS-free and sim-free greps hold. `git diff 518717a -- include/ src/
Makefile`: EMPTY. src_build_gate check: **PASS** (14 builds; self-test not needed: no src/ or Makefile change).
api_doc_tool self-test / check-coverage (6,502 entities) / check-fresh / check-examples (988 lines) /
check-removability OK (no `///` changed: docs/api not regenerated); briefing self-test + check, doc_staleness_audit
self-test + run, doc_readability self-test / check-nav / check-tone: all OK, and re-run after this entry (below).

**Stopped here: the verification round is complete, uncommitted, for the coordinator.** Open for the coordinator:
change (9)'s ruling, now with the 1 in limit (item 2 above); the merge by hand onto main (the review round's list;
this round touched only `obstacles.mjs`, its node test, guide 22 and the changelog — no file RMP1-G or RMP1-F owns);
`preview.png` / `preview-failed.png` and the four fixtures after the 21st; D5, D6, D2 tests after the 21st; the
`touches` shape check after the merge.
