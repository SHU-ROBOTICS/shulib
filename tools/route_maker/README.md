# The route builder — for developers

The route builder is a web page (static files, no server, no framework, no runtime dependency) where the team builds
the skills route by clicking and exports the C++. Students: read
[guide chapter 22](../../docs/guide/22-making-your-first-route.md). This file is for whoever changes it.

## The layout

| Where | What |
|---|---|
| `src/model/` | PURE ES modules (no DOM, no file API), each with its node test in `test/`: the route and its edits (`route.mjs`), the library's curve ported line for line (`curve.mjs`, pinned by CV1), every check (`checks.mjs`), the export (`export.mjs`), the ONE number formatter (`numbers.mjs`), the marker / hand-edit / line-ending rules (`region.mjs`), the project file (`project.mjs`), undo, the field and snapping, the library's quoted sentences (`library_words.mjs`, checked against the headers) |
| `src/host/` | the four host interfaces the screens call: `files.mjs` (over `files_fsaccess.mjs` — Chrome/Edge's folder — or `files_download.mjs` — every browser), `engine.mjs` (the preview; none yet), `robots.mjs` (the descriptions baked into the page), `capabilities.mjs`. A later VS Code panel or local helper swaps these, never the screens or a rule |
| `src/ui/` | the screens: `app.mjs` (the shell and the store), `field_view.mjs`, `step_list.mjs`, `inspector.mjs`, `export_drawer.mjs`, `help.mjs`, `words.mjs` (what the page calls each step), `dom.mjs` (helpers and the icons) |
| `src/app.html`, `src/style.css`, `src/browser_check.js` | the page's template, its design tokens and stylesheet, and the script that runs first and refuses an old browser in plain words |
| `assets/field.webp`, `assets/field.json` | the real field picture, red at the bottom, and its MEASURED transform and landmarks — made by `python3 tools/docs_figures/route_maker_field.py` (Pillow), committed |
| `build.py` | stdlib Python: `--out DIR --describe build/test/shulib_route_maker` writes the SITE form (`app.html` + `src/`) and the ONE FILE (`shulib-route-builder.html`); refuses any URL but the XML namespaces and any laptop or runner path (RG1). Its output is never committed |
| `goldens.mjs` | `node tools/route_maker/goldens.mjs --write` re-makes `test/goldens/route_maker/*.cpp` from their `.route.json` (the C++ suite compiles and runs them) — never `gl0_empty.cpp`, the contract's empty region kept by hand (no `.route.json`; its hash is pinned in `export.test.mjs`) |
| `test/` | the node tests (`node --test tools/route_maker/test/*.test.mjs`); `test/page/` the browser tests |

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
  empty for them to type.
- **CI proves it on Windows and Linux** (`.github/workflows/ci.yml`: `route-builder`, `route-builder-browsers`);
  the scoped `.gitattributes` here and in `test/goldens/route_maker/` keep these files LF on a Windows checkout.
