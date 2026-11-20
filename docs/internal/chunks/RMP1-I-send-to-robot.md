# RMP1-I — the route builder on the team laptop: draw, then ONE button sends it to the robot

> Brief by the coordinator, 2026-11-20 ~06:45. **The team lead, 2026-11-20:** "we really have to make making, building
> and uploading routes easier for people using the builder" (and 11-19: "IT IS SO HARD", "confusing to know where to
> put the exported route code", "the person needs to know if it successfully exported and can be built and
> uploaded"). One run, **RMP1-I-A**. The route builder stays open to the 21st (competition: SATURDAY Nov 21). NO
> library (`include/`) change, NO `src/` change. Base: integrate `ae87076` (main `7d24984` + upload_auton finding make
> and pros in the PROS toolchain's folder).

## 1. What a student does today (five places, two apps)

1. Open the route builder on the docs site. 2. Chrome/Edge: connect the shulib folder (a folder dialog, then an
"Allow" each visit); Firefox: Export hands back a file or a copy to put in `src/team/skills_route.cpp` BY HAND.
3. Export; the page reads the file back (Chrome). 4. Switch to VS Code: Terminal → Run Task → "Upload routes to
robot one" (`tools/upload_auton.py`), read ✓/✗ in the terminal. 5. The brain: Auton Tester → the route → RUN.
Steps 2 and 4 are where students get lost. The page was built for this: `src/host/capabilities.mjs` already reserves
`build`, `upload`, `brainLog` ("a later … panel ADDS capabilities"), and `files.mjs`'s host list names "a local
helper". This run is that helper.

## 2. Decisions

- **D1 — One command opens it:** `python3 tools/route_builder.py` (`python` on Windows; Python 3 standard library
  ONLY — no pip) and a VS Code task **"Open the route builder"** (in `.vscode/tasks.json`, first in the list; a
  background task, so the terminal shows the URL and stays). It starts a small HTTP server on `127.0.0.1` (a free
  port; never 0.0.0.0), prints `Route builder: http://127.0.0.1:<port>/…  (Ctrl+C stops it)` and opens the default
  browser there. Ctrl+C stops it cleanly (no traceback). A second start while one runs opens the running one.
- **D2 — The page it serves is THIS checkout's:** the route builder's modules from `tools/route_maker/src` (so the
  page and the helper always match), built by `tools/route_maker/build.py`'s own code (import it; do not copy its
  rules). The robots' descriptions (`robots.json`) and PREVIEW's engine: this laptop's build when present
  (`build/route_maker_site`), else the PUBLISHED site's (`https://docs.shurobotics.com/route-builder/…`) downloaded
  into `build/route_builder_cache/` when online and kept, so it opens with NO internet at the venue after one run at
  home. NEVER a C++ compiler or Emscripten on the team laptop. When neither is there (first run, offline): say so in
  one plain line and still open (no PREVIEW; the robots from… say what it can do). The page says, as today, which
  robots' facts it knows.
- **D3 — Files without a folder dialog, in ANY browser:** a new host `src/host/files_local.mjs` (kind `'local'`) over
  the helper: `readText`/`writeText` of `src/team/…` through same-origin requests; `connected()` always true;
  `folderName()` the checkout's folder name. EVERY rule stays `region.mjs`'s through `files.mjs` (markers, the hand-edit
  hash, line endings, the read-back after export). The helper ALSO refuses any path that is not a file under
  `<checkout>/src/team/` after resolving `..` and symlinks, and writes all-or-nothing (a temp file, then replace).
  `app.mjs` picks `files_local` when the page was served by the helper (the helper puts
  `<meta name="shulib-local" content="<token>">` into the page it serves — the site never has it, so the site never
  probes the network).
- **D4 — Send to robot (the point of this run):** when the page is local, Export's result (green **Written and
  checked**) shows ONE primary button: **Send to robot one** (robot two for robot two's routes; both when the export
  wrote both) — and the same button in the top bar (it exports first if the route changed since the last export,
  then sends). It runs `tools/upload_auton.py <one|two>` (a subprocess — ONE implementation of build+upload, the one
  the VS Code task runs) and streams its lines into a small panel AS THEY COME: "Building…", **✓ BUILT**, "Uploading
  to slot 2…", **✓ UPLOADED** + "On the brain: shulib Auton Tester → <route> → RUN" — or **✗** with upload_auton's
  plain words (a compile error's file:line; inside the ROUTE MAKER lines: "re-export"; OWED facts; "plug the BRAIN in
  with USB (not the controller)"), and a "Show everything" fold with `build/upload_auton.log`. One send at a time (the
  button says "Sending…" and is disabled; a second request is refused by the helper too). Closing the tab never leaves
  an orphan make or pros: the helper owns the child and kills it on Ctrl+C.
- **D5 — Safe on a laptop:** bound to 127.0.0.1 only; every request must carry the token (header for the API; the
  token in the first URL, then a same-origin cookie or the meta) — a request from another website or another local
  page without it gets 403; the `Host` header must be `127.0.0.1:<port>` or `localhost:<port>` (DNS rebinding); only
  GET for files of the page and `src/team/` reads, POST for writes and send; no directory listing; nothing outside the
  checkout served.
- **D6 — The website keeps working as today** (Chrome folder / Firefox download). On the site, Export's Next adds ONE
  line at its top: "Easiest: open the route builder from VS Code — Terminal → Run Task → Open the route builder — then
  Send to robot is one button." Nothing else on the site changes. The exported C++ is byte-for-byte (the RMP1-C
  goldens).

## 3. Tests (adversarial — each seen RED against a mutant)

- **Python self-test** (`python3 tools/route_builder.py self-test`, like upload_auton's): starts the helper on a free
  port against a stand-in checkout (a temp copy with `src/team/`), and checks: the page served with the meta; a read
  and a write under `src/team/` work; `../`, an absolute path, `src/robots/…`, a symlink out of `src/team/`, a
  directory → refused (403/404) and nothing written; a missing or wrong token → 403; a wrong `Host` → 403; a write is
  all-or-nothing (a failure mid-write leaves the old file); Send with upload_auton's fake make/pros (reuse its stand-ins:
  PATH) streams ✓ BUILT and ✓ UPLOADED in order and a ✗ case streams the plain error; a second Send while one runs is
  refused; Ctrl+C (SIGINT) ends the child make. Capped: `ulimit -v 4194304` + `timeout`.
- **Node:** `files_local.mjs` against a stand-in fetch (the request shapes, a refusal surfaces as the same error kinds
  the other hosts use).
- **Page (Playwright, Firefox AND Chromium):** the helper started against the stand-in checkout and fake tools; the
  page opens WITHOUT a folder dialog; Export writes `src/team/skills_route.cpp` (read back green); **Send to robot
  one** shows the lines in order and ✓ UPLOADED with the route's name; a fake compile error shows ✗ with its line; the
  site form (no meta) behaves as before. Screenshots light/dark at 1440 and 1280 (Export's result with Send; the panel
  mid-send; ✓; ✗).

## 4. Docs (one idea per screen; the team lead finds dense screens "overstimulating")

- Guide 22: the route-to-robot path rewritten LOCAL-FIRST: "Open the route builder from VS Code (Run Task → Open the
  route builder) → draw → Export → Send to robot one → on the brain: Auton Tester → your route → RUN", each with a
  screenshot; the website path as "On a computer without the shulib folder". Guide 23 step 5 one line; `route-builder.md`
  (the browser table gets a first row "On the team laptop: `python3 tools/route_builder.py` — any browser, everything,
  and Send to robot"); `where-your-code-goes.md` one line; the changelog; `tools/route_maker/README.md`;
  RMP1-PROGRESS (`## RMP1-I`).

## 5. Landmines

- Nothing else in flight on the page right now; a later run (the mechanisms' menu) will touch `inspector.mjs` and the
  mechanism steps: keep this run's UI in `export_drawer.mjs`/`to_robot.mjs`, a NEW `src/ui/send_panel.mjs`, the top
  bar (one button) and ONE delimited block of `style.css` (`/* ── Send to robot (RMP1-I) ── */`).
- `tools/upload_auton.py` is the ONLY build+upload code: the helper runs it; do not duplicate its parsing. If it needs
  a change (e.g. a flag to print machine-readable stages), keep its self-test green and add cases.
- The helper must work on Windows (paths, `python`, no `fork`, Ctrl+C) — the CI Windows rows run the self-test.
- Never run the REAL pros, never touch /dev/ttyACM* (the self-test and page tests use the fake tools). The
  coordinator does the one real upload, with the team lead's OK.
