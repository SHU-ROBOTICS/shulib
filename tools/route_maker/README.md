# The route builder — for developers

The route builder is a web page (static files, no server, no framework, no runtime dependency) where the team builds
the skills route by clicking and exports the C++. Students: read
[guide chapter 22](../../docs/guide/22-making-your-first-route.md). This file is for whoever changes it.

## The layout

| Where | What |
|---|---|
| `src/model/` | PURE ES modules (no DOM, no file API), each with its node test in `test/`: the route and its edits (`route.mjs`), the library's curve ported line for line (`curve.mjs`, pinned by CV1), every check (`checks.mjs`), the export (`export.mjs`), the ONE number formatter (`numbers.mjs`), the marker / hand-edit / line-ending / read-back rules (`region.mjs`), what Export says next (`to_robot.mjs`: the build and upload commands), the project file (`project.mjs`), undo, the field and snapping, the field's solid structures (`obstacles.mjs`, RMP1-H: the goals and the Loaders from the description's `field.obstacles` — `test/override_field_obstacles.hpp`, cited to the game manual — every step's swept square checked against them), the library's quoted sentences (`library_words.mjs`, checked against the headers) |
| `src/host/` | the four host interfaces the screens call: `files.mjs` (over `files_fsaccess.mjs` — Chrome/Edge's folder — or `files_download.mjs` — every browser), `engine.mjs` (PREVIEW's engine, in a Worker — below), `robots.mjs` (the descriptions baked into the page), `capabilities.mjs`. A later VS Code panel or local helper swaps these, never the screens or a rule |
| `src/ui/` | the screens: `app.mjs` (the shell and the store), `field_view.mjs` (and `field_obstacles.mjs`: the structures' outlines, the one highlight, a step's "touches it on purpose"), `step_list.mjs`, `inspector.mjs`, `export_drawer.mjs` (with `to_robot.mjs`: the result, "Will it build?", Next), `help.mjs`, `words.mjs` (what the page calls each step), `dom.mjs` (helpers and the icons); the Controls tab, one step on screen at a time: `controls_tab.mjs` (step 1: the controller and the editor beside it), `feel_tab.mjs` (step 2: the driving feel and its graph, "More settings" closed at first), `controls_export.mjs` |
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

The page tests drive the page as built — robot one as its facts describe it today (its cascade has NO levels since
ROB1-C2: the menu says "No levels yet: record them at Bench Tests station 13"). The tests that move to a level put a
FIXTURE robot in robot one's place first (`window.routeBuilder.useRobots`): robot x's committed description with a
cascade of facts-ledger levels, still owed (`ledgerRobot()` in `test/helpers.mjs`).

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
| `mechanisms` | (RMP1-E) each mechanism in the robot's list, in its order: `name`, `kind` (`lift`, `arm`, `pneumatic`, `power`, `flywheel`), a lift's or arm's `units`, `bottom`, `top`, a pneumatic's `positions`, `placeholder` (RPV1's PLACEHOLDER sentence, or ""), and `at` — its state at every sample, read from the library's own mechanism each pace: a lift's or arm's position in its units, a pneumatic's position index (-1: none), a power mechanism's or flywheel's direction (-1, 0, 1). An older record has no such member: the page shows no strip |
| `samples` | every 10 ms pace: `[t, x, y, theta]` — the plant's TRUE pose (inches, radians unwrapped) |

A non-finite number is written `null`. Native and WebAssembly records agree exactly but for the poses and a lift's
or arm's position (within 1e-5 in — an arm's 1e-5 deg — and 1e-6 rad: the two maths libraries round differently in
the last bit — WG2).

## PREVIEW in the page (RMP1-A2u)

PREVIEW runs the route on screen through the engine and plays the record back. The pieces:

| Where | What |
|---|---|
| `src/host/engine.mjs` | the page's side: one Web Worker (made at the first PREVIEW, kept), each run's phases, **Cancel** (the Worker terminated), a robot's run that has not answered in **20 s** (the engine's load: 60 s) terminated and said so, a new Worker after any failure |
| `src/worker.mjs` | the Worker's side, after the engine's own code in ONE script: an engine instance (`self.shulibBackend`), remade after any run that returns non-zero (P-W1f) |
| `src/host/engine_backend.mjs` | the real engine for that script: RMP1-A2e's `engine_module.mjs` and `route_lines.mjs` around the WebAssembly module |
| `src/model/playback.mjs` | the record (schema 1) → what the screen draws: each robot's pose at time t (only the record's own, straight between paces), each page step's span and verdict in plain words beside the library's own lines, the PLACEHOLDER marks, a run's key (the route it belongs to; a step's `touches` is not part of it), and (RMP1-H) a run's THROUGH a solid structure, the page's own (`obstacles.mjs` `runThrough`: past RPV1's 1 in) when it comes before RPV1's |
| `src/ui/preview.mjs`, `src/ui/timeline.mjs` | the button, the keys and the dock: progress and Cancel, then the scrubber; the run on its own field layer (`field_view.mjs`'s `drawRun`), the step cards lit (`step_list.mjs`) |
| `src/ui/mech_strip.mjs` | (RMP1-E) the mechanisms beside the field, from the record's `mechanisms` (`playback.mjs`'s `mechanismsAt`) at the pace playing: a height bar per lift or arm (its stops, its height, the planned target of its step — `lift.mjs`), a piston per pneumatic, a motor per power mechanism or flywheel; the mechanism the step playing uses accented |

`build.py --engine build/wasm` (the built module) writes `engine/worker.mjs` + `engine/preview.wasm` beside the SITE
form's page (a `<meta name="shulib-engine">` points at it) and the same script, with the single-file module, inside
the ONE FILE (`<script type="text/plain" id="shulib-engine">`, run as a `blob:` Worker). The Worker is a CLASSIC
script in both forms: Chromium refuses a module Worker made from a `blob:` URL on a page opened from disk, so
`build.py` turns the module's glue into a classic script (`import.meta.url` → `self.location.href`, the Worker's own
URL) and refuses a glue it cannot (any other `import.meta`, code outside its one function). Without `--engine` the page
says PREVIEW comes next. A record with no paces (the library refused the route before it moved) is said, not played.

**Robot two** plays only when BOTH are true: robot two's update is in the build (`sharedRoute`, so the page asks for
the pair), and the runner has a PAIR MODE — today it has none (no chunk owns it yet), so the page plays robot one and
says so. That mode is the backend's `pair(route, robots) -> {rc, record, verdict}`: `record` robot two's run of
`competition::mirrored(entry, kSkillsMirror)` from ITS seated start (`competition::seatedStart` — RPV1's
`plantStartFor` seats a mirrored entry at robot one's start today, and MIRROR1-A's `checkMirroredPair` needs the
routine run inside `previewEntry`), as schema 1 with `robot: "two"`; `verdict` MIRROR1-A's `PairVerdict` as JSON — the
shape the page reads:

| Member | What |
|---|---|
| `checked`, `ok` | the check ran (false: skipped, `sentence` says why); it never came under the gap |
| `gapIn` | the gap kept clear: `kMirrorGapIn` (6) — the band drawn is 3 in each side of the mirror line |
| `leastGapIn`, `leastGapAt` | the closest the two squares came, and when (s) |
| `failAt`, `failGapIn`, `one`, `two` | the FIRST pace under the gap: when, how close, each robot's `[x, y, theta]` — `null` when `ok` |
| `sentence` | the check's own words |

**The stand-in** (`test/standin_engine.mjs`): the page's own `worker.mjs` behind RECORDED runs
(`test/fixtures/preview/*.preview.json` — `madeBy`, a project file and the engine's result for it), injected by
`window.routeBuilder.useEngine` as the file tests inject a folder; a route with no recording is answered "no recorded
run", never another's. Its options: a delay, a run that never returns, a load that fails, an exit code, a REFUSED
record as RPV1's `refusedModel` writes it, a member of the record changed (`patch`), robot two's record and verdict
(`two`, `pairRc`), members left out as an older engine's record lacks them (`omit`); every record it sends carries
the instance and the run it came from. The robot-one records are the
engine's native twin's (`shulib_route_maker preview`), made by `test/fixtures/record.mjs` (the routes and the
command) — re-record them when the record's schema, robot one's facts or the API change. Four of their routes (ready, failed, pair_fail, through) were made before the field's structures were checked and drive through a goal, so the pages that play them are baked without the structures (`test/page/harness.mjs` `noStructures`); re-recording them clear lets that go. The two PAIR fixtures' robot
two and verdict came from a PROTOTYPE of the pair mode on a copy of RPV1's header with `plantStartFor` patched (their
`madeBy` says so): re-record them when the runner has the mode.

```sh
ROUTE_MAKER_BROWSER=firefox node --test tools/route_maker/test/page/preview.test.mjs   # every PREVIEW state (stand-in)
```

On a site built with `--engine` the same file also runs the REAL module through the page, in both forms, and
compares its record with the native twin's of the same route, from the same tree — made at test time when the
program is there (`ROUTE_MAKER_NATIVE`, or `build/test/shulib_route_maker`), else read from
`ROUTE_MAKER_NATIVE_RECORDS` (a folder `test/fixtures/record.mjs --out` wrote: CI's Linux job makes it for the
browser rows, which have no native program) — every member exact, poses (and a lift's or arm's position) within 1e-5
in (an arm's 1e-5 deg) and 1e-6 rad; for the `ready` route and for `mechanisms`, the strip's. Off CI with neither, it compares with the committed fixture less the API version and the facts
ledger's lists; that run is robot one's facts', so it goes red when they change until the fixtures are re-recorded.
In CI (`CI` set) a tree with `tools/route_maker/wasm/` must have built its site with `--engine` and must give the
native twin's record: that test fails, not skips, when either is missing.

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
  overwritten silently** — all in `region.mjs`, applied by `files.mjs` for every host. **The export into
  `skills_route.cpp` is READ BACK** (a file that reads back different is a red refusal, never a green).
- **Export is never silent** (RMP1-F): one numbered step for this browser, then green (✓ written and checked,
  downloaded, copied) or red with its reason and its fix; then the commands that build and upload, or
  `tools/upload_auton.py one|two`, which does both and prints ✓ or ✗ (`python3 tools/upload_auton.py self-test`:
  its cases against fake `make` and a fake `pros` that takes pros-cli's own grammar).
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
