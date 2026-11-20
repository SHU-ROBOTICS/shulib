# RMP1-F — Export → robot: where the code goes, whether it worked, what to do next

> Brief by the coordinator, 2026-11-19 ~20:10. **The team lead, 2026-11-19:** "make it easier to add a route because
> it's confusing to know where to put the exported route code — also a person using Chrome needs to know as well";
> "we need to make it so that the person knows if it successfully exported and can be built and uploaded". One run,
> **RMP1-F-A**. The route builder stays open to the 21st. NO library (`include/`) change. Base: main AFTER RMP1-C
> (named routes; its export writes every route into the one region) merges — the page files are shared.

## 1. Why

After Export today a student sees the C++ and a button whose result is quiet: "Write into src/team/skills_route.cpp"
(Edge/Chrome, folder connected), a download (other browsers), or Copy. Nobody knows (a) where the code goes, (b) whether
it landed, (c) whether it will build, (d) what to type next. The team builds on Windows and Linux.

## 2. Decisions

- **D1 — Where it goes, per browser, as THE step on the export page.** One numbered card (one idea at a time — the team
  lead: dense screens are "overstimulating"): **Chrome or Edge** → "1 Connect your shulib folder (the one with `src`
  inside) · 2 Write into src/team/skills_route.cpp"; **other browsers** → "1 Download · 2 Replace
  `src/team/skills_route.cpp` with it" (the exact path, a Copy button for it), or Copy → "paste between the two
  `ROUTE MAKER` lines" with a small picture of the two lines. The page detects which one applies (File System Access
  API present or not) and shows only that one.
- **D2 — A clear result, read back.** After Write: the page READS THE FILE BACK and shows a big green **"✓ Written and
  checked: src/team/skills_route.cpp — N routes (names), M steps"** when the region equals what it exported, or a red
  card with the exact reason and the one fix (no folder access, the markers missing, a hand edit kept, a write that
  read back different). After a download: "✓ Downloaded — now replace src/team/skills_route.cpp with it". After Copy:
  "✓ Copied — paste it between the two ROUTE MAKER lines". Never a silent success.
- **D3 — "Will it build?" in plain words.** Beside the result: what the page checked — every step valid by the
  library's own rules (the export is only offered at "Ready to export"), the region's contract — and the one thing only
  the build checks (the C++ compiler), so the next step is the build.
- **D4 — Next: build and upload, with Copy.** A "Next" card: the two commands for the robot shown
  (`make ROBOT=one PROGRAM=autontest` then `pros upload --slot 2 --name "shulib Auton Tester"`; `ROBOT=tank` for robot
  two), each with Copy, and **how you know it worked**: make ends with no `error` line; pros says the upload finished;
  on the brain, shulib Auton Tester (slot 2) → the route's name → RUN. And the one-command helper (D5).
- **D5 — One command that builds, uploads and says plainly whether it worked:** `python3 tools/upload_auton.py one`
  (or `two`): runs the Auton Tester's make for that robot, then `pros upload --slot 2 --name "shulib Auton Tester"`,
  and prints **✓ BUILT** / **✓ UPLOADED to slot 2 — on the brain: shulib Auton Tester → <route names> → RUN**, or **✗**
  with the first real error in plain words (the robot's OWED facts list as make prints it; a compile error's file:line
  and message — e.g. inside the ROUTE MAKER region of skills_route.cpp → "re-export from the route builder"; no brain
  found → "plug the brain in with USB (not the controller)"; several ports → it picks the V5 Brain's). Windows and
  Linux (Python 3, no shell-only syntax; finds `make` and `pros` on PATH; says how to install PROS if missing). Tested
  with fake `make`/`pros` on PATH (success, OWED refusal, compile error, no brain, two ports).
- **D6 — Nothing else changes:** the exported code byte-for-byte (the RMP1-C goldens), the project file, PREVIEW.

## 3. Tests (adversarial — each seen RED against a mutant)

Page (Playwright, Firefox + Chromium): Chromium shows the Chrome/Edge card and Firefox the other one, never both; Write
then read-back → the green card names the routes and steps; a write whose read-back differs → the red card (a planted
mismatch); a file without markers → red with its fix; Copy/download → their result lines; the Next card's commands for
robot one and robot two. Node: the read-back comparison. D5: the script against fake tools for every case above, on
Linux, and its syntax on Windows paths (no POSIX-only calls). Screenshots light/dark at 1440 and 1280.

## 4. Docs

Guide 22 step 7 (Export) rewritten around the new cards, short; guide 16 (the Auton Tester) and guide 23 step 5 point
at `tools/upload_auton.py`; RMP1-PROGRESS (`## RMP1-F`). A narrated tutorial video is made from the real page AFTER
this lands (the coordinator).

## 5. Landmines

- RMP1-E (mechanisms in PREVIEW) may run beside you on the same page: do not edit `preview.mjs`, `timeline.mjs`,
  `tools/route_maker/wasm/`; CSS in one delimited block of `style.css` (`/* ── Export → robot (RMP1-F) ── */`).
- Never run the real `pros` or touch /dev/ttyACM* (the fake tools only).
