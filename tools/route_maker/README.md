# The route builder — for developers

The route builder is a web page (static files, no server, no framework, no runtime dependency) where the team builds
the skills route by clicking and exports the C++. Students: read
[guide chapter 22](../../docs/guide/22-making-your-first-route.md). This file is for whoever changes it.

## The layout

| Where | What |
|---|---|
| `src/model/` | PURE ES modules (no DOM, no file API), each with its node test in `test/`: the route and its edits (`route.mjs`), the library's curve ported line for line (`curve.mjs`, pinned by CV1), every check (`checks.mjs`), the export (`export.mjs`), the ONE number formatter (`numbers.mjs`), the marker / hand-edit / line-ending rules (`region.mjs`), the project file (`project.mjs`), undo, the field and snapping, the library's quoted sentences (`library_words.mjs`, checked against the headers) |
| `src/host/` | the four host interfaces the screens call: `files.mjs` (over `files_fsaccess.mjs` — Chrome/Edge's folder — or `files_download.mjs` — every browser), `engine.mjs` (the preview; not wired to the page yet), `robots.mjs` (the descriptions baked into the page), `capabilities.mjs`; and `engine_module.mjs`, ONE instance of the preview engine (below). A later VS Code panel or local helper swaps these, never the screens or a rule |
| `src/ui/` | the screens: `app.mjs` (the shell and the store), `field_view.mjs`, `step_list.mjs`, `inspector.mjs`, `export_drawer.mjs`, `help.mjs`, `words.mjs` (what the page calls each step), `dom.mjs` (helpers and the icons); the Controls tab: `controls_tab.mjs` (the controller), `feel_tab.mjs` (the driving feel and its graph), `controls_export.mjs` |
| `src/model/` (the Controls tab) | `controls.mjs` (the controls as data, one check per build refusal of `controls()`, the `controls` and `control words` regions), `feel.mjs` (the library's stick curve in the C++'s order, `checkFeel`, the `feel` region), `driver_file.mjs` (the driver file's three marked regions: found, read, written all or none) |
| `controls_cases.mjs` | the CT agreement's data: one configuration per build refusal on the fixture robots; `goldens.mjs --write` turns each into `test/compile_refusals/rm_ct_*.cpp` |
| `src/app.html`, `src/style.css`, `src/browser_check.js` | the page's template, its design tokens and stylesheet, and the script that runs first and refuses an old browser in plain words |
| `assets/field.webp`, `assets/field.json` | the real field picture, red at the bottom, and its MEASURED transform and landmarks — made by `python3 tools/docs_figures/route_maker_field.py` (Pillow), committed |
| `build.py` | stdlib Python: `--out DIR --describe build/test/shulib_route_maker` writes the SITE form (`app.html` + `src/`) and the ONE FILE (`shulib-route-builder.html`); refuses any URL but the XML namespaces and any laptop or runner path (RG1). Its output is never committed |
| `goldens.mjs` | `node tools/route_maker/goldens.mjs --write` re-makes `test/goldens/route_maker/*.cpp` from their `.route.json` (the C++ suite compiles and runs them) — never `gl0_empty.cpp`, the contract's empty region kept by hand (no `.route.json`; its hash is pinned in `export.test.mjs`) — and the Controls tab's goldens and `rm_ct_*.cpp` refusals |
| `src/model/route_lines.mjs` | the route → the LINES the preview engine runs (below) |
| `wasm/CMakeLists.txt` | the preview engine: the REAL library in WebAssembly (below) |
| `test/` | the node tests (`node --test tools/route_maker/test/*.test.mjs`); `test/page/` the browser tests; `test/wasm/` the engine's |

## Build and test it

```sh
cmake -S test -B build/test && cmake --build build/test --target shulib_route_maker
python3 tools/route_maker/build.py --out build/route_maker_site --describe build/test/shulib_route_maker
node --test tools/route_maker/test/*.test.mjs            # the model (any OS; no browser)
(cd tools/route_maker && npm ci)                          # the tests' ONE dev dependency: Playwright 1.64.1
ROUTE_MAKER_BROWSER=firefox node --test tools/route_maker/test/page/*.test.mjs   # firefox | chromium | msedge | chrome
```

Open `build/route_maker_site/shulib-route-builder.html` by double-click to use it.

**On the coordinator's laptop** (the 2026-10-15 memory rule): every `node` and `npx` runs as
`(ulimit -v 4194304; timeout ...)` with `NODE_OPTIONS=--disable-wasm-trap-handler` (Node 20 refuses WebAssembly
under the cap without it); Playwright's OWN Firefox, with `PLAYWRIGHT_BROWSERS_PATH` and every screenshot and
temporary file under `~/shulib-render-tmp/` (a non-hidden folder: the snap Firefox cannot write into a hidden one or
/tmp, and is not used). Chromium reserves more ADDRESS SPACE than `ulimit -v` allows (it dies with SIGTRAP at
launch), so Playwright's Chromium runs here under a memory cap of the same size on its real use instead:
`systemd-run --user --scope -p MemoryMax=4G -p MemorySwapMax=0 node --test ...`; Edge and Chrome run in CI.

## The preview engine (RMP1-A2e)

The engine behind the page's PREVIEW (which RMP1-A2u wires into the page) runs the REAL library on the host test
plant: `test/route_maker_runner.hpp` (the runner) plus RPV1's engine (`test/route_preview_run.hpp`, unchanged),
compiled to WebAssembly from the ONE main the native `shulib_route_maker` is built from (`test/route_maker_main.cpp`).
The page turns its route into lines (`route_lines.mjs`); the runner makes the SAME library calls `export.mjs` writes,
with the same numbers, on the robot's own mechanism list, and returns a record. It runs the builder's route, never the
team's autons (`src/team/` is not compiled in: `test/route_maker_no_team_autons.cpp`). Nothing here runs on a robot.

**Emscripten 6.0.13, pinned:** the emsdk checkout at its release commit `35659964bc27` (`./emsdk install 6.0.13`), as CI
installs it. Build and test:

```sh
source <emsdk>/emsdk_env.sh
emcmake cmake -S tools/route_maker/wasm -B build/wasm && cmake --build build/wasm       # ~10 s, ~300 MB
cmake --build build/test --target shulib_route_maker                                    # the native twin (WG2)
ROUTE_MAKER_WASM=build/wasm ROUTE_MAKER_NATIVE=build/test/shulib_route_maker node --test tools/route_maker/test/wasm/
ROUTE_MAKER_BROWSER=firefox ROUTE_MAKER_WASM=build/wasm node --test tools/route_maker/test/page/engine.test.mjs
```

The one list of allowed console warnings is `test/page/allowed_warnings.mjs` (today Firefox's, about the module's
legacy exception instructions); `engine.test.mjs` fails on any other warning or error. A1's `page.test.mjs` still has
its own (empty) list; it and A2u's PREVIEW test import this one at A2u's merge.

On the coordinator's laptop add `TMPDIR=build/tmp` (Emscripten writes its temporary objects there, not /tmp) and the
memory rule above; the shared emsdk is used read-only (`EM_FROZEN_CACHE=1`). `ROUTE_MAKER_SWEEP=3000` runs WG2s's
full sweep (300 by default).

`build/wasm/` (never committed): `preview.mjs` + `preview.wasm` (the SITE form), `preview_single.mjs` (the ONE FILE
form, the `.wasm` inside as base64) and `preview_test.mjs` + `.wasm` (also every golden's exported C++ and a test-only
looping step: WG1, WG7). Linked for `web,worker,node`, at most 256 MB, a 1 MB C stack whose overflow aborts.

**Running it** — always through `src/host/engine_module.mjs`: `engineInstance(createPreview)` then
`.preview(lines)` → `{rc, text, record, words}` (`text` is the record as written). The module's `callMain` leaves
its arguments on its C stack each call; the wrapper restores the stack, or some hundreds of PREVIEWs overflow it (679
of a 1.4 KB route). `rc` 0: it ran — also when a step failed, was skipped or jumped, or the 57 s cut ended it (the
record's `word` and each page step's `verdict` say so); 1: the auton threw (`threw`), went THROUGH a wall or a
Loader (`through`), was refused and never ran (`refused`), or hit the no-pace stop (`looped`); 2: the lines were refused
(`words` say why; the page's checks refuse such a route first); 3: the program itself failed (`words`). No record
for 2 or 3. A route that loops without letting time pass is stopped with
`std::_Exit` mid-route (P-W1f; the record's `looped`): the instance runs on correctly, but what that run built stays
on its heap — a fresh instance frees it. `.preview()` THROWS if the module aborts (a C stack overflow: never seen on a
route, a run needs 16–32 KB of the 1 MB); that instance is dead: make a new one.

**The lines** (`route_lines.mjs`): `shulib-route 1`, then `route robot=… name=… park=… stop=…`, `start x=… y=… h=…`
(or `start relative`), and one `step do=<kind> key=value …` per page step. Every number is `numbers.mjs`'s text; a
name is %-encoded; a mechanism goes by its `.name` and a ledger level by its preset's name (from the robot's
description). The runner refuses, by name, any line the export could not have written.

**The record — schema 1** (one JSON object, one top-level member per line; times are seconds from the arming, `s`
are indices into `samples`):

| Member | What |
|---|---|
| `schema`, `api` | 1; the library's API version |
| `robot`, `robotLine`, `owed`, `ledgerFile` | which robot; RPV1's line saying what is pretend; its facts still OWED (PLACEHOLDERS) |
| `auton`, `kind`, `period`, `park`, `stop`, `start` | what ran: its name, SKILLS, 60, 57, 59, where it started (words) |
| `notices` | RPV1's lines for the top of the page |
| `word`, `refused`, `why`, `threw`, `looped`, `loopSentence`, `loopHead`, `cut`, `scoringEnded`, `endActionEnded`, `exit` | how it ended (the Auton Tester's word, the refusal's or throw's sentence, the no-pace stop, the 57 s cut) |
| `pageSteps` | per PAGE step: `n`, `do`, `verdict` (`ok`, `failed`, `failedWentOn`, `cut`, `skipped`, `jumped`, `threw`, `looping`, `notReached`), `first`/`last` (its first and last sample — exact), `lib` (the brain's log's step numbers for it, `[from, to]`) |
| `steps` | RPV1's steps seen from below: `n`, `kind`, `what`, `s0`, `s1`, `t0`, `t1`, `verdict`, `mark`, `sentence`, `endAction`, `background`, `openAtThrow`, `placeholder`, `target` (`[x, y, heading]` or null) |
| `lines` | every log line from the arming on: `s`, `t`, `sub`, `text`, `mark` |
| `field`, `through`, `throughAt` | RPV1's field check: `kind` (THROUGH, TOUCH, PIECE), `element`, `s`, `t`, `step`, `depth` |
| `samples` | every 10 ms pace: `[t, x, y, theta]` — the plant's TRUE pose (inches, radians unwrapped) |

A non-finite number is written `null`. Native and WebAssembly records agree exactly but for the poses (within
1e-5 in and 1e-6 rad: the two maths libraries round sin, cos and atan2 differently in the last bit — WG2).

## The rules that keep it true to the library

- **The page refuses what the library refuses**, in the library's own words: a check's twin is a golden the C++
  suite compiles and runs (`test/route_maker_golden_test.cpp`).
- **One number formatter** (`numbers.mjs`): every number is stored rounded to 3 decimals as it is edited, and the
  export prints exactly that.
- **The export depends on the route alone** — no time, no hash, no path — and an empty route writes the contract's
  empty region byte for byte (GL0). The route's NAME is the contract's too (`"skills"`): the page does not rename
  it.
- **A project file is never half-read**: one from a newer builder — a newer `schemaVersion`, or a step kind or a
  field this builder cannot write — is refused by name (`project.mjs`), and a number the student has not typed is
  left OUT, never stored as `NaN` (which JSON turns into 0).
- **Only between the markers, only under `src/team/`, the file's own line endings kept, a hand edit never
  overwritten silently** — all in `region.mjs`, for every host.
- **Nothing is invented for the students** (G4): a one-click fix adds a step they see, and leaves its time limit
  empty for them to type. The Controls tab starts from the driver file's own map, never a made-up one.
- **The Controls tab refuses what the build refuses, by the build's name**: each of those refusals has
  compile-refusal twins (`rm_ct_*.cpp`) the build must refuse with the page's name, and a test fails when the
  library gains a refusal the page has no case for — all but one: any feel but the default, refused while "shulib
  Drive" names the default feel when it starts (`drive_program.cpp`'s `static_assert`), is pinned by its sentence
  read from that source (`feel.test.mjs`) instead. It also asks for what is unfinished and for words the map would
  get wrong (no build refusal: no twin). Its stick curve equals the C++ bit for bit on the host (`feel_curve.txt`).
- **CI proves it on Windows and Linux** (`.github/workflows/ci.yml`: `route-builder`, `route-builder-browsers`);
  the scoped `.gitattributes` here and in `test/goldens/route_maker/` keep these files LF on a Windows checkout.
