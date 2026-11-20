# RMP1 — THE ROUTE MAKER: one offline app in a browser where a student who does not code builds the skills route by clicking on the real field, presses PREVIEW and watches the REAL library run it (RPV1's engine) on a timeline they can scrub, sees both robots when robot two runs it mirrored (MIRROR1), and exports commented C++ for the autons file; a CONTROLS tab (a picture of the V5 controller → the driver file's controls lines, refused exactly where the build refuses) and a DRIVING FEEL section (a live stick-to-speed graph from the library's own formula → the driver file's `kFeel`)

> **Status 2026-11-13 (written ~13:25–14:00 by the coordinator's brief writer, resuming a run the usage limit stopped at
> ~03:40 — that run left only scratch trees, no brief text; this one starts from them; RE-CHECKED ~16:10–17:30 against
> the checker's 19 findings — "Re-check — amendments" R1–R19 below, which override the body; every body change is
> marked *[Rn]*): DRAFT for the team lead's go. WAITS for his answers to D1–D16, then for a go per run (RMP1-A1
> first). Nothing is built. The coordinator's queued Windows / web-app AMENDMENT (NOTES 11-13) may change D1, D5's
> helper and D8 after this.** Written from a READ-ONLY pass. *(amended 2026-11-13 late) **The Windows / web-app
> amendment — "Amendment 2026-11-13 (late)" right below this header, newest first — was written ~16:35–17:55,
> STOPPED by the laptop's restart at ~23:28 with the body half-amended, and FINISHED after it (from ~23:30): every body
> section it changes is now marked *(amended 2026-11-13 late)*, and one more probe ran (P-W1f). It overrides the
> re-check and the body where they differ: D1 → (e) a web app on the docs site, PREVIEW = the real library compiled to
> WebAssembly (PROBED: viable, no library change); the helper is gone; the runs are re-planned (A1, A2e, A2u, A1b, B,
> C); **D15 ANSWERED (a)** by the team lead; new D17 (publishing), D18 (Chromium on this laptop) and D19 (is the route
> builder itself inside the freeze?); D16 settled with a proven recipe.***
> ***[W1–W16] The amendment RE-CHECKED 2026-11-14 (~00:00–01:30):** the checker's 15 findings, each re-verified by
> re-running its probes and a few new ones — all real, fixed in place and marked *[Wn]*; one more found while
> re-verifying (W16). The record is **A.12**; A.7 (dates), A.9 (new **D20**: the pushes that prove Windows) and A.11
> (plain words) are updated.*
> **Base: main is `a27e2bc`, not `d33d25d`.** The task named `d33d25d`; main has since taken STEP1-B (`8df58f3`, API 2.8),
> VIS1-A2 (`2c3aa34`), AF1S-A1 (`5f6633b`), the release script, the front page, guide 2's field picture and the CI pin
> (`git log d33d25d..a27e2bc`). `src/` is byte-identical between the two (`git diff --stat d33d25d a27e2bc -- src/` is
> empty), so ROB1-INT's `src/` (on `d33d25d`) lays over `a27e2bc` cleanly. **Every `file:line` below was OPENED at
> `a27e2bc`** unless it names a worktree: `rpv1-a` (RPV1-A, in its fix round, on `d33d25d`), `rob1-int` (ROB1-INT, done,
> merging after the 15:50 reset), `mirror1-a` (MIRROR1-A, built, in review, on `5f6633b`), `g3a-p1` (unmerged). Every
> worktree was read, never edited. **Probed, not argued:** five probes in `.claude/coordinator/scratch/brief-RMP1/`
> (§0.4) over `tree-main/` (`git archive a27e2bc`), `tree-rob1/` (that + ROB1-INT's `src/`) and `tree-d33/` (`d33d25d` +
> RPV1-A's preview files); every binary ran under `(ulimit -v 4194304; timeout 300 …)`.
>
> **THE TEAM LEAD'S WORDS THIS BRIEF HOLDS (BINDING), as given to this writer:**
> - **2026-11-13: "i really want to get the route builder and preview working so that people can start coding autons"**
>   — the route maker and the preview are **the TOP priority after robot one**.
> - **2026-11-13: "i really want the builder and previewer to be really modern looking interactive and user friendly
>   like they should be well built products not just things tacked on. user feel is important"** — ONE product: build the
>   route on the real field, press PREVIEW, the robot plays the route right there from the REAL library's run, with a
>   timeline scrubber; drag and snap, zoom and pan, undo and redo, keyboard shortcuts, hover help in plain words, inline
>   checking as you edit, one design system in light and dark, smooth animation, offline at the event.
> - **2026-11-13 (driver controls):** "is there an easy way for non coders to handle creating the driver controls?" →
>   **"Yes, add it (Recommended)"** — a CONTROLS tab: a picture of the V5 controller; click a button, pick the mechanism and
>   the action; refuse in plain words what the library refuses (the C++ static_asserts are the final authority); export
>   the driver file's controls lines; save in the same project file the panel reuses later.
> - **2026-11-13 (the feel): "they need to be able to have enough control over coding the driver controls where they can
>   do something like make it so that moving their stick slightly will go slower but still move forward and push it down
>   will go faster"** — a DRIVING FEEL section: a slider per stick with a LIVE GRAPH from the library's own formula, the
>   dead zone, how quickly it may speed up, the drive mode; exports `kFeel`. A "slow mode while a button is held" is OUT
>   unless the library has it — **it does not** (§0.2, item 9).
> - **The situation:** first competition **Nov 21, SKILLS ONLY; BOTH robots run the skills route.** "robot 1 and robot 2
>   are the same build and just the auton routes are different", "IT IS LITERALLY THE SAME ROBOT BUILD JUST A MIRRORED
>   ROUTINE", "we can do everything including the second robot do not sell yourself short", "i really want the route
>   maker and/or panel so that it is easier to code the bots for non coders". **Code changes stop ~Nov 18. At most 3 build
>   jobs at a time.**
> - **MIRROR1 has his GO** (2026-11-13 ~03:30, D1–D11 as recommended): the route is written ONCE in robot one's numbers;
>   robot two's autons file carries ONE line, `registry.add(competition::mirrored(skillsRoute(),
>   competition::override2026::kSkillsMirror));` — **never a reflected copy**; the pair test on the host (a 6 in gap);
>   `competition::isMirrored(ctx)` for what lies on the diagonal; the mirror is the diagonal y = −x:
>   **(x, y, h) → (−y, −x, −90° − h)**. *[R2] The first print said "Robot two's line is typed by the STUDENTS (MIRROR1
>   D2 [R11])" — that was MIRROR1's recommendation BEFORE the go.* **The GO's D2 (MIRROR1 brief :7-9): robot two's
>   starter autons file registers the shared skills route MIRRORED — one line, PLUMBING, written by ROB1-BT or by the
>   coordinator at MIRROR1-A's merge, whichever is second; the ROUTE itself stays the students'.** The page only
>   checks that the line is there (D6).
> - **The docs rules (2026-11-13):** no "honest" disclaimers or status preambles in public pages (limits stay as plain
>   facts where they apply); VISUALS — the real field picture, `tools/docs_figures/source/override-field-top.png` turned
>   red-at-bottom as `docs/assets/field-coordinates.png` does it. They bind the page's own help text too.
> - **G4: the students author the routes, the driver map and the feel; a tool may turn THEIR choices into code; it never
>   writes a route (or a map) for them.**
> - *History, superseded:* on 2026-11-10 (RPV1's answers) he chose "(b) Code + preview only" over a standalone planner
>   page. His 2026-11-13 words above reverse that for the route maker; RPV1 stays the engine the route maker plays.
>
> **Nine things in the task as given that the tree and the probes do not support — corrected here:**
> 1. **"main d33d25d"** — main is `a27e2bc` (above). The citations are `a27e2bc`'s.
> 2. **"the checked Toggle step `competition::checkAndFlipToggle` (AF1S-A1)" as a route step.** Main has only the EXPLICIT
>    form, which takes a camera handle (`competition/toggle_visit.hpp` :11-16, :1490) that no auton can reach
>    (`AutonContext` carries no camera, `auton_registry.hpp` :213-240; AF1S brief :484 "No auton can reach the camera").
>    The form a route can call, `checkAndFlipToggle(ctx, request)`, is **AF1S-A2's** (queued, AF1S brief :284-286). So the
>    page offers the CHECKED Toggle step only once AF1S-A2 has merged (shown greyed, "arrives with the robot's camera
>    step", until then). The BLIND flip is a plain mechanism step on the flipper — and **robot one's flipper is not
>    declared yet** (its kind is owed: `rob1-int/src/robots/mechanism_table.hpp` :150-155), so it appears in the menu the
>    day the builder's facts add it — the page reads the robot's real list (D2).
> 3. **"a wall reset with its wall and sensor".** Robot one's robot file declares **no side distance sensors yet** (no
>    walls block in `rob1-int/src/robots/robot_one_facts.hpp`), so on robot one every reset is `REFUSED no sensor` until
>    that block is typed; and the preview's rig builds no Robot, so **in the preview a reset is always `REFUSED no sensor`**
>    (P1: `[WALL] red wall y REFUSED no sensor (RIGHT sensor): this robot declares no wall sensors`) until RPV1-B2 (after
>    L6-B). The page says both, on the step.
> 4. **"a trigger at N percent of a drive".** STEP1-B's triggers ride drives and turns, **not curves**
>    (`chassis/routine.hpp` :62 "Partway triggers (chunk STEP1-B, below) do not cover curves yet"; `ArmedLeg` :1218-1281
>    has no `curveTo`). The page offers a trigger on drive and turn steps only.
> 5. **"it refuses what DriverProfile::valid() refuses".** The PAGE can; **the BUILD does not**: an out-of-range feel
>    compiles and runs today — P1b built `DriverFile<…, kBadFeel{.forward = {.expo = 1.5}}, …>` against `a27e2bc` + ROB1's
>    `src/` with `-Werror` (`src/common/driver_file.hpp` :81-84 checks only the re-zero button and the tick). G3a's Part 5
>    planned the one-line `static_assert` (G3a brief :281-282) and never landed. D10 asks.
> 6. **A "wait" step is not `r.pause(t)`.** `Routine::pause` is a frozen `Chassis::wait` (`routine.hpp` :639-646) that the
>    run's deadline cannot cut (`auton_registry.hpp` :218-220); the export uses the run guard's pause (G3a's C2 fix), as P1
>    compiled and ran.
> 7. **A wall reset's "if it fails: carry on" is not `r.onFail(carryOn)`.** A refused reset that is not `required`
>    already continues (`auton_wall_reset.hpp` :38-40): P1's refused reset under `onFail(carryOn)` counted `completed=6,
>    failedOn=0`. The choice maps onto `.required` (G3a D14's mapping: stop = required; carry on = not required; skip =
>    required + `onFail(skipTo(...))`).
> 8. **"real screenshots of every main screen captured headless (Firefox is on this laptop)".** Firefox is the SNAP
>    (`/usr/bin/firefox` is the snap's wrapper; Firefox 159.0.1). From this agent's sandbox, five `--headless
>    --screenshot` spellings exited 0 and wrote **no file**, and a WebDriver session through the snap's own geckodriver
>    timed out (P3). The build proves a working capture path in its FIRST hour (§2.5) — or the coordinator captures. *A
>    stray `geckodriver --port 4471` (pid 3381105) from P3 is still running; the sandbox refused to stop it (`kill:
>    Permission denied`) — the coordinator stops it.* *[R18] AND a headless `firefox --marionette` it started, **pid
>    3381195** (~270 MB), both still running at 16:20 (`ps`). The checker's own CLI try also exited 0 and wrote no
>    file (`check/ff.log`). There is no other headless renderer here — so a capture path is asked of the team lead
>    NOW (D16), not left to the build's first hour alone.* *(amended 2026-11-13 late) Both strays ended with the
>    laptop's restart at ~23:28 (`ps` after it: no geckodriver, no Firefox); D16 is settled (Amendment A.5).*
> 9. **"slow mode while a button is held"** — the library has none (item 5's file, `teleop/`, `driver_inputs.hpp`: no
>    such word; `DriverLoopConfig` holds `tickPeriod`, `rezeroButton`, `releaseBrakeMode`, `protection` only,
>    `teleop/teleop_loop.hpp` :196-223). OUT (§5).
>
> **Executor model: `opus`. Do not commit. Live log `docs/internal/chunks/RMP1-PROGRESS.md`, created FIRST** (a heading
> per run). Say "host test plant", never "simulator" — in code, comments, the page, its help and the docs. **No hardware
> claim anywhere:** nothing here runs on a robot. **Hardware Assumptions: none expected** (§4; next free is HA-261 after
> MIRROR1's HA-258..HA-260 — a belief found ⇒ stop and tell the coordinator). **Memory rule (2026-10-15):** every binary
> built — the page's helper, probes, the suite, any test, `node`, the browser — runs as `(ulimit -v 4194304; timeout …)`;
> the suite only through `python3 tools/suite_run.py`. **RMP1 changes no robot program** (tools/, test/, docs/ only) —
> except D10's one optional `static_assert`, and the students' own exports, which are their code. *(amended 2026-11-13
> late) There is no helper any more. Under the memory rule every capped `node` / `npx` also gets
> `NODE_OPTIONS=--disable-wasm-trap-handler` (Node 20 refuses WebAssembly under the cap without it — A.1), every
> `em++` / `emcmake` gets `TMPDIR` inside the run's `build/` (Emscripten writes its temporary objects to /tmp
> otherwise — P-W1f), and Chromium never runs on this laptop (D18). The files RMP1 touches grow by
> `.github/workflows/ci.yml` and `pages.yml`, `mkdocs.yml` (the tab) and `tools/route_maker/package.json` + its lockfile
> (the browser tests' one pinned dev dependency) — still no robot program.*
>
> **Shape:** the house style of `docs/internal/chunks/L6-wall-reset.md`.

---

## ANSWERS (the team lead, 2026-11-14 ~01:45)

- **"Go, accept all (Recommended)"** — every decision (D1–D20) as this brief recommends; **A1 GO** (each later run still gets its go).
- **D20: "Yes, standing OK (Recommended)"** — the coordinator pushes the route builder's own branches (`rmp1-*`) to GitHub so the
  Windows CI runs on them before they merge. Only those branches.
- **D17: "Yes, publish each piece (Recommended)"** — a docs release when each route-builder piece merges before the 21st.
- **D19: "Keep it open to the 21st (Recommended)"** — the route builder itself stays open; the library and robot setup freeze on the 18th.
- **D15** (answered 2026-11-13 late): (a) keep routes open.

## Amendment 2026-11-13 (late) — Windows: a web app, the preview in WebAssembly

> Written ~16:35–17:55 by the coordinator's brief writer: a READ-ONLY pass on the repository and every worktree, plus
> probes in `.claude/coordinator/scratch/brief-RMP1/wasm/` (browser screenshots through `~/shulib-render-tmp/rmp1-amend/`,
> copied to `wasm/shots/`). Every binary ran as `(ulimit -v 4194304; timeout …)` — with ONE exception, owned here: one
> diagnostic run of Node 20 on the 6.5 s route WITHOUT the cap (`timeout 120` only, 44 ms) to tell "out of address
> space" from "unsupported"; it was not repeated. **This amendment OVERRIDES the re-check and the body where they
> differ; every body change below is marked *(amended 2026-11-13 late)*.** Nothing is built. RMP1 still waits for
> the team lead's go per run. *Stopped by the laptop's restart at ~23:28 with the body half-amended; FINISHED after it
> (from ~23:30): the emsdk re-checked (6.0.13 still builds), every output re-hashed (the three groups in P-W1b stand:
> 3 native files identical, 7 WebAssembly files identical, `throws` identical in all 10), one screenshot re-opened,
> and one more probe run — **P-W1f**, RPV1's no-pace stop inside the module — which CORRECTED the first print's A.3.10
> (it claimed a hook RPV1 does not offer). D15's answer (the team lead, late 11-13) is recorded and the dates planned
> on it (A.7, D15, new D19).*

**THE TEAM LEAD'S WORDS (2026-11-13, verbatim, BINDING):**
- "it should work on windows too btw that is very important. if anything maybe we implement it in our webapp or
  something i dont know whatever is easiest but also gives the best result"
- "we do shurobotics.com/build? i dont know"
- "maybe we dont do a vscode extension and instead do the webapp add on?"
- "no i want it to build and upload the program i think that would be better ... because then the app can also debug
  AHHHHHHHH whatever we just need the preview and route builder ready and able to work on windows and linux"

**What they make binding here.** (1) **THE ACCEPTANCE BAR FOR RMP1: the route builder and PREVIEW work on Windows AND
Linux — proven, not assumed:** the page's headless tests (`node --test` for the model; a headless browser for the page
and the WebAssembly PREVIEW) run in CI on a Linux runner AND a `windows-latest` runner (A.4), plus a short manual check
on a team Windows laptop before the freeze (A.4's checklist). (2) **Build, upload and debug** are what he wants for the
full panel AFTER the 21st; whether that is a VS Code extension, a desktop app or this web app with a small local helper
is decided THEN. Nothing of it is built here; the builder's code is HOST-AGNOSTIC so that panel reuses it (A.6).
(3) **Static files only.** The app is a folder of static files that works from the docs site, from shurobotics.com/build
if the team later puts it there (that server is NOT inspected and nothing depends on it), and from one downloaded file
with no network (A.3.6).

### A.1 The probes (`scratch/brief-RMP1/wasm/`; run BEFORE any design)
**The tree** `wasm/tree/` = the checker's composed tree (`check/t1`: `a27e2bc` + ROB1-INT's `src/` + RPV1-A's
`auton_tester_rig.hpp`, `fakes_by_kind.hpp` and `fake_telemetry_sink.hpp`) + RPV1-A's `test/route_preview_*.hpp`
(worktree `rpv1-a`, read only). **The program** `wasm/probe/pw1_route.cpp` = RPV1-A's own engine, unchanged —
`route_preview::previewEntry` (the Auton Tester's `TesterStack`: the 50 idle ticks, the countdown, `runAuton`, the run
guard, the mechanism service; a Chassis on the host test plant's stand-in 12 in tank) — on **robot x's switch-homed
fixture lift** (`test/mechanism_fixtures.hpp`, on RPV1-A's fakes by kind), three routes: **minimal** (re-check P6 (4)'s
route: `home`, a wall reset — REFUSED no sensor on the rig —, `driveTo` with a 60 % trigger `startMove` on the lift
(the library's `MoveToPose` underneath), `curveTo`, `waitUntilDone`, `onFail(skipTo)`, a 2.5 s drive, `label`, the
run guard's pause); **sixty** (30 rounds of drive + trigger, curve, face, drive back, turn: the guard cuts it at 57 s —
a full skills run, 5 715 paces); **throws** (`waitUntilDone` on a mechanism never started: a `PreconditionError`
THROWN inside the library and caught by the Auton Tester's sequence). It prints every pace's true pose and estimate as
hex doubles, every step from below, every log line, RPV1's table and an FNV-1a hash of RPV1's page.

| # | What | Result |
|---|---|---|
| **P-W1a** the toolchain | `git clone` emsdk (commit `708dcbc`) into `wasm/emsdk`, `./emsdk install 6.0.13` (released 2026-10-19; 7.0.0 of 11-06 skipped — under two weeks old) | **Installed in user space, no sudo:** 312 MB + 32 MB (its Node 24.19) downloaded, **1.7 GB** unpacked. Emscripten 6.0.13 = clang; its defaults (`src/settings.js`): `MIN_CHROME_VERSION` 85, Firefox 79, Safari 15.0, Node 18.3, raised automatically by `-fwasm-exceptions` (`tools/feature_matrix.py` :96-110) |
| **P-W1b** the build | `wasm/build_wasm.sh`: `em++` with **the suite's exact flags** (`-std=c++20 -Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion`, the suite's include roots and `SHULIB_HOST_PROS_SHIM`) + `-fwasm-exceptions`; also `-fexceptions` (JavaScript-based) and `-sWASM_LEGACY_EXCEPTIONS=0` (exnref); `-O0/-O2/-O3/-Os`. Natively: GCC 13.3 `-O0` (the suite's level — `cmake -S test` sets no build type) and `-O2`, Clang 18 `-O2` | **Zero warnings, zero source changes** — the library, the host test plant, RPV1-A's engine and robot two's autons file compile with `-Werror` under Emscripten's clang (and Clang 18 natively) as they are. **`.wasm`: `-O2` 576 KB (200 KB gzip), `-Os` 419 KB (163 KB gzip), `-O3` 593 KB, `-O0` 910 KB; glue 11 KB.** JS exceptions: 617 KB + 22 KB glue. Build 8–12 s, **290 MB** peak (native g++ `-O0`: 6 s, 458 MB) |
| **P-W1b** exceptions | the **throws** route | **Works in every build:** `PROGRAM ERROR` with the library's own sentence (`r.waitUntilDone(...): this routine started no step on that mechanism -- start one first ...`), the page and the table identical to native. **Legacy Wasm exceptions** recommended (Chrome/Edge 95, Firefox 100, Safari 15.2, Node 17); exnref needs Chrome 137 / Firefox 131 / Safari 18.4 and **Node 20 refuses it** (`invalid value type 'noexternref'`) |
| **P-W1b** speed (the 60 s route) | 5 runs, `quiet`; the library's run timed inside the program | **WebAssembly in Node 24: 112–129 ms** for 57 s of auton time (+23 ms for RPV1's page), **0.25–0.27 s for the whole process** (Node's start and the compile included), ~95 MB peak RSS. **Firefox 157: 169–192 ms** (P-W2). Native GCC `-O2` 45 ms; GCC `-O0` (the suite's) 149–164 ms. The helper design's PREVIEW was ~5 s (P4: 4.4 s to compile the route) |
| **P-W1b** fidelity — **byte for byte** | `wasm/tools/compare.py`; 9 builds × 3 routes, `md5sum` groups | **Three groups, nothing else:** all 3 native builds identical to each other (GCC `-O0`, GCC `-O2`, Clang `-O2`); all 6 WebAssembly builds identical to each other (`-O0/-O2/-O3/-Os`; legacy, JS and exnref exceptions) **and across engines** — Node 24, Node 20 and Firefox 157 (P-W2) give the same bytes; **throws** (a straight drive) identical in all 9. **Native vs WebAssembly:** every STEP (number, kind, verdict, start/end time, start/end pace, the library's sentence), every log line with its time, RPV1's table and RPV1's page hash: **IDENTICAL**. The **poses are NOT byte-identical**: the first differing pace is **209 (T+2.09 s, inside the curve)**; **minimal**: 316 of 655 paces differ, by **≤ 2.1e-14 in**; **sixty**: 5 376 of 5 715 paces, by **≤ 4.1e-8 in and 2.4e-9 rad** after 57 s |
| **P-W1b** — exactly where the floating point differs | `wasm/probe/pw1_libm.cpp`: 100 000 seeded inputs per math function the library calls (a grep of `include/shulib`: `cos` ×70, `hypot` ×65, `sin` ×62, `sqrt` ×22, `atan2` ×15, …), native (glibc 2.39) vs WebAssembly (Emscripten's musl-derived libm) | **The math library, nothing else.** Every arithmetic operation agrees (IEEE 754; no fused multiply-add on either side). **1-ulp differences** in: `atan2` 17.7 % of inputs, `hypot` 14.0 %, `acos` 7.7 %, `asin` 6.0 %, `tan` 3.8 %, `log10` 3.2 %, `cos` 3.1 %, `sin` 3.0 %, `atan` 2.3 %, `exp` 0.07 %; **identical:** `sqrt`, `log`, `remainder`. A last-bit difference grows slowly through the closed loops — to 4.1e-8 in after 57 s — and changed no step, verdict or pace |
| **P-W1c** the numbers a route carries | `wasm/probe/pw1_literals.cpp`: every field number from −72 to 72 in with 1–3 decimals (159 843) through the unit literals' `long double` operator (`units/literals.hpp` :22 — 80-bit on x86-64, 128-bit in WebAssembly) vs `strtod` (what a runner parsing the page's numbers would do) | **0 of 159 843 differ**, natively and in WebAssembly |
| **P-W1d** the RUNNER — the route as DATA | `wasm/probe/pw1d_runner.cpp`: the minimal route as nine text lines (what the page would hand the module), each mapped at run time to the SAME library calls — a mechanism found by NAME through the robot's table (`MechanismTable::declared`, `wiring.hpp` :122-124), a preset turned into the SAME `PositionGoal` the compile-time `lift["low"]` builds (`kinds.hpp` :870-877: the declaration, its name, the preset's own name, the value, not computed, inches), `CurvePoint`s in a vector, `Angle::degrees` as `_deg` calls it (`literals.hpp` :64-66) — vs the same route COMPILED | **BYTE-IDENTICAL, natively (`a42ef8f4…` both) AND in WebAssembly (`5a00134a…` both)**: every pose, step, line and the page. So **"the page's model run by a runner" and "the exported C++" are one run** when built the same way |
| **P-W1e** a CMake target | `wasm/cmake-probe/CMakeLists.txt` (refuses to configure without Emscripten), `emcmake cmake -S … -B build -DCMAKE_BUILD_TYPE=Release`, `cmake --build` | **Builds** (12 s, 290 MB) an ES module + `.wasm` (593 KB); run under Node exactly as the page runs it (`callMain`, `print` captured): **byte-identical** to P-W1b's `-O2` on all three routes |
| **P-W1f** RPV1's no-pace stop inside the module *(after the restart)* | `wasm/probe/pw1f_loop.cpp` = P-W1's program + RPV1's OWN looping case (`route_preview_run.hpp` :34-40: a drive that times out, so the routine stops, then `for (;;)` drives — each logs "skipped" and no pace passes), run through `previewEntry` with `opt.onLoop` set; GCC `-O2` and `em++ -O2 -fwasm-exceptions` (the site form, `-sEXIT_RUNTIME=0`); `wasm/out/pw1f/run_loop.mjs`: the loop, then `minimal` on the SAME instance, then on a FRESH one | **No RPV1 change needed.** Natively: `onLoop` printed `LOOPS WITHOUT LETTING TIME PASS`, the process exited 1. **In WebAssembly RPV1's `std::_Exit(1)` ends only the call: `callMain` RETURNS 1** in 16–25 ms, nothing thrown (Emscripten's `proc_exit` throws its `ExitStatus`, which its `callMain` catches), `onLoop` having handed over the loop's record; **the same instance then ran `minimal` byte-identical to a fresh instance.** Node 24 under the cap: PASS. **Node 20 + the flag: the loop passed, then V8 died out of memory making the SECOND instance** (most likely each instance reserving address space for the module's 2 GiB maximum); **linked with `-sMAXIMUM_MEMORY=256MB`, Node 20 and Node 24 both PASS**, and the 60 s route still runs in 125 ms with P-W1's body. So the module links with that ceiling (A.3.3) |
| **P-W1** Node under the memory cap | Node 24.19 (emsdk's) and Node 20.20 (this laptop's) under `ulimit -v 4194304` | Node 24 runs WebAssembly under the cap. **Node 20 refuses** (`WebAssembly.instantiate(): Out of memory: Cannot allocate Wasm memory`: V8 reserves guard regions beyond 4 GiB) **unless `--disable-wasm-trap-handler`** — then it runs, byte-identical. **Node 20's own `fetch` fails the same way** (`npx playwright install` died in `lazyllhttp`): **every capped `node` / `npx` gets `NODE_OPTIONS=--disable-wasm-trap-handler`** |
| **P-W2** the page, from the site AND from one file | `wasm/build_pw2.sh` + `wasm/tools/make_pw2_pages.py`: (1) the SITE form — `preview.mjs` (10.5 KB, ES module) + `preview.wasm` (575 KB), served by a static server shaped like GitHub Pages (`.wasm` as `application/wasm`); (2) ONE downloaded file — the module with the `.wasm` inside (`-sSINGLE_FILE=1`, base64) inlined into one HTML page, opened over `file://` with every http(s) request refused. The page runs the three routes, hashes each output and compares it with Node's run of the same build | **Both PASS in Playwright's Firefox 157 under the 4 GiB cap: all three routes byte-identical to Node** (`wasm/shots/firefox-site.png`, `firefox-offline-file.png`); the offline page made no request at all. Instantiate 9–216 ms; the 60 s route 169–192 ms. **The one file: 780 KB (281 KB gzip)**; its base64 `.wasm` 777 KB (Emscripten's UTF-8 "binary" encoding: 649 KB, but it needs the file served as UTF-8 — not recommended). The single-file loader decodes base64 in JavaScript and never fetches (so `file://` works) |
| **P-W2** the page's Content-Security-Policy (R11) | the one-file page with R11's policy as written (`default-src 'none'`; inline script and style) and with `'wasm-unsafe-eval'` added | **R11's policy as written BLOCKS the preview:** `CompileError: call to WebAssembly.instantiate() blocked by CSP`. **With `script-src 'unsafe-inline' 'wasm-unsafe-eval'`: PASS.** (`wasm/tools/pw2_csp.mjs`) *[W1] That was the ONE FILE only; the SITE form under the same policy is BLOCKED (its `preview.mjs` and Worker are separate files) and passes with `'self'` added — A.3.9 now has two policies (A.12)* |
| **P-W3** which browsers | Emscripten 6.0.13's feature table; the page's own check; Firefox with WebAssembly switched off; V8 with the legacy exception instructions switched off | **The floor:** legacy Wasm exceptions — Chrome/Edge 95, Firefox 100, Safari 15.2; with the CSP's `'wasm-unsafe-eval'` (Chrome/Edge 97, Firefox 102, Safari 16 — an older browser blocks WebAssembly under that policy) **the page states Chrome or Edge 97, Firefox 102, Safari 16 — any browser updated since mid-2022.** **Windows:** 10 and 11 with any current Chrome, Edge or Firefox; 7 and 8.1 reach it too (Chrome 109 and Edge 109 were their last, Firefox ESR 115). **Writing straight into the shulib folder** (the File System Access API, `showDirectoryPicker`): **Chrome and Edge on a desktop only** (Windows, Linux, macOS, ChromeOS) — never Firefox or Safari. **The refusal, PROVEN:** a classic inline script runs first and asks `WebAssembly.validate()` about a 34-byte module that uses the legacy exception instructions: **true** in Node 20 and 24; **false** with V8's `--no-experimental-wasm-legacy-eh` (the real `preview.wasm` is then invalid too); a Firefox with WebAssembly off shows the plain-words box (`wasm/shots/firefox-no-wasm-refusal.png`): *"This browser is too old to run the route preview. The preview needs Chrome or Edge … Please update the browser (or use another one); the route builder itself still opens, saves and exports."* (the probe printed 95/100/15.2; the app prints 97/102/16). Safari NOT tried (no Safari or WebKit here). *[W16] The site form's Worker is an ES module; module Workers need Firefox 114 (MDN; not run) — the app prints **Chrome or Edge 97, Firefox 115, Safari 16**, and the check also tests module-Worker support. [W6] Legacy exceptions are deprecated: Firefox 157 warns on every instance (A.3.3)* |
| **P-W4** a `windows-latest` runner | from documentation — **a run needs a push, which is the coordinator's**: actions/runner-images' readmes (fetched to `wasm/docs-fetched/`) and the installed Playwright 1.64.1's own files | **`windows-latest` = Windows Server 2025** (image 20261103.298.1): **Chrome 156.0.8078.80, Edge 156.0.4314.58, Firefox 159.0.1, Node 22.23.4 and 24.22.2** preinstalled. **`ubuntu-24.04`** (20261111.370.1): Chrome 157, **Chromium 157**, Edge 157, **Firefox 160**, Node 22.23.4. **Playwright 1.64.1** (2026-10-14; 1.65.x skipped — under two weeks): Chromium 156.0.8078.4, Firefox 157.0, WebKit 27.2 on **Linux, macOS AND Windows** (its README); its code maps channel `"msedge"` on win32 to `\Microsoft\Edge\Application\msedge.exe` and `"chrome"` to the installed Chrome — so the Windows job drives the runner's OWN Edge and Chrome, plus Playwright's Firefox. **Not run on a Windows runner yet: A1's first CI run is the proof** |
| **D16** the screenshot path | Playwright 1.64.1 pinned in `~/shulib-render-tmp/rmp1-amend/pw/`, its own browsers in `PLAYWRIGHT_BROWSERS_PATH=~/shulib-render-tmp/pw-browsers` (588 MB), `TMPDIR` under `~/shulib-render-tmp/` | **Firefox (Playwright's own 157): WORKS under the 4 GiB cap** — 4 screenshots, page text read, no stray process left (`ps` after each run). **Chromium (Playwright's own headless shell 156): DUMPS CORE at start under the 4 GiB cap** (its address-space reservations); a higher cap was NOT used (refused, rightly — the 10-15 rule). The snap Firefox was NOT re-tried: Playwright's Firefox avoids the snap, and P3's two strays (pids 3381105, 3381195) were still running then — no third was risked (the 23:28 restart ended both) |

**Verdict: WebAssembly is VIABLE in time, with NO library change.** The library, the host test plant and RPV1's engine
compile to WebAssembly as they are, exceptions work, the module is ~165–200 KB over the network, a 60 s route runs in
~0.2 s in a browser, a downloaded single file works with no network, and the step results agree with the native build
exactly (the poses to the last bits, for a stated reason). No fallback is needed. *(For the record, the two fallbacks
weighed and not needed: a Windows build of the preview program from CI, run by a local helper — needs Python on every
laptop and a library never built on Windows; or a JavaScript port of the motion code — a second robot model that is
not the library, the landmine this chunk exists to avoid.)*

### A.2 What "the preview is the library" means now — and the golden test that holds it
- **In the browser** the route the student sees is turned into a short list of steps and handed to **the runner**: a
  small C++ file compiled INTO the WebAssembly module with the library, the host test plant and RPV1's engine. The
  runner makes the **same library calls the export writes** (P-W1d's mapping: mechanisms and presets by name through the
  robot's own table, the same `PositionGoal`, `CurvePoint`s, `Angle::degrees`, the trigger chain built by a small
  template over the leg kinds — `if constexpr` keeps an impossible chain from being compiled; the page refuses it first).
  The student's route is never C++ in the browser.
  *[W7] **And with the SAME NUMBERS:** the export and the line format print every number through ONE formatter
  (`model/numbers.mjs`), and the model stores every number already rounded at edit time to what the export prints (at
  most 3 decimals — P-W1c's proven range; a dragged point snaps, a typed value is rounded as it is entered and shown
  rounded), so PREVIEW never runs a `12.3456` the file prints as `12.35`; test NF1 checks every number in the line
  format equals the export's, character for character. [W8] **The runner never parses inside a call's argument
  list:** C++ leaves the order of a call's arguments unspecified and GCC and Clang differ (`recheck2/argorder/`:
  `driveTo(L{num()}, L{num()})` swaps x and y under GCC, not under Clang or Emscripten — so WG1, all in WebAssembly,
  would NOT see it); every value is read into a named variable first (`const double x = num(in), y = num(in);` —
  declarators are sequenced), and golden WG8 makes every pair of numbers in every step kind different so a swap shows.
  [W15] The record never prints a NaN's sign or payload (the one place WebAssembly arithmetic differs between x86 and
  ARM): a non-finite value is written as one fixed word.*
- **The golden test (A2e):** for every golden route — **WG1** the runner on the route's JSON and the EXPORTED C++
  compiled with `em++`, both run in WebAssembly under Node: **byte-identical** (P-W1d's result, now for every step kind);
  **WG2** the same golden's native run ~~in the suite~~ *[W9] by the native program `shulib_route_maker preview` (GCC,
  the suite's flags; built in the route-builder CI job — A.3.3) and the module, both driven by ONE Node test that
  compares the two JSON records*: **every step
  (number, kind, verdict, start and end pace), every log line, the table: EXACT; positions within ~~1e-6 in~~ *1e-5 in*
  and headings within ~~1e-7 rad~~ *1e-6 rad*** — ~~25× the largest difference measured over a full 57 s run (4.1e-8 in,
  2.4e-9 rad)~~ *[W2] set from a SWEEP, not three routes: 3,000 seeded random routes of every step kind (timeouts,
  carry-on, triggers, curves; 40,551 steps) gave 0 step differences and at most 3.5e-7 in and 2.5e-8 rad — the bound
  is ~28× and ~40× that, still a hundred-thousandth of an inch (the first print's 1e-6 / 1e-7 was only ~3× / ~4×). WG2
  prints the measured maximum beside the bound on every run, and **WG2s** re-runs a seeded sweep in CI (300 routes; the
  full 3,000 in A2e's report)*. A golden whose step ends one pace apart FAILS by name — never loosened (landmine 12's
  rule). *[W2] The EXACT rule is empirical (0 of 3,000), not a theorem: a last-bit difference CAN flip a threshold or
  a printed decimal. Written now: only when the run shows the first difference IS such a flip (a comparison within
  ~1e-9 of its threshold, or a printed decimal on its rounding boundary) is that golden or seed REPLACED by another,
  logged by name; anything else is a bug, and no rule is ever loosened;* **WG3** the same `.wasm` in Node, and in
  every CI browser (A.4), gives byte-identical records (proven here for Node 20, Node 24 and Firefox 157).
- **Why poses cannot be byte-identical to the native build, in plain words:** the computer's sine, cosine and arctangent
  are rounded slightly differently by Linux's maths library and by the one inside the WebAssembly module — in the last
  of ~16 digits, for a few percent of inputs. On the robot a third maths library is used, so neither copy is ever the
  brain's bit for bit; the preview is the host test plant's run either way, as RPV1 labels it. **What CAN be promised,
  and is tested: the same steps, verdicts and times as the native build, and the same bytes on every browser and OS** —
  the maths library travels inside the module.

### A.3 The design (D1 (e)) — every body decision this touches is marked *(amended 2026-11-13 late)*
1. **Where it runs (D1 (e), RECOMMENDED):** a web app — static files — at **docs.shurobotics.com/route-builder/**, a
   **"Route builder" tab** on the docs site (a short docs page — what it is, *Open the route builder*, *Download for
   offline use*, which browsers — and the app beside it). Windows, Mac, Linux, Chromebooks; nothing installed. **The
   helper (`tools/route_maker.py`) is DROPPED** — every job it had moves (below). *[W3] The docs page is
   `/route-builder/`; the app itself is **`/route-builder/app.html`** — NEVER `route-builder/index.html`: `mkdocs build
   --strict` renders `docs/route-builder.md` to that same path and silently overwrites the app (re-proved with mkdocs
   1.6.1: exit 0, no warning, the app gone; as `app.html` both survive and the page's link becomes `href="app.html"`).*
2. **The pieces.** (a) The page: D8's plain JavaScript + SVG, the model modules unchanged (they were pure); (b) the
   **robot descriptions** (`robots.json`: each robot's real mechanism list, presets, homing, the field's numbers) made
   by the C++ describe program (§1.1's `route_maker_describe.hpp`) run NATIVELY in CI and baked into the page — **A1
   needs no WebAssembly**; (c) **the preview engine** (A2e): one WebAssembly module = the library + the host test plant
   + RPV1's engine + the runner + the describe function, run in a **Web Worker** so the page never freezes.
3. **How the WebAssembly is produced.** NEW `tools/route_maker/wasm/CMakeLists.txt` — its own CMake project like
   `test/`, refusing to configure without Emscripten (P-W1e's shape), the suite's warning flags with `-Werror` (proven
   clean), `-fwasm-exceptions` (legacy instructions), `-Os` (419 KB; `-O2` gave the same bytes of output at the same
   speed), *`-sMAXIMUM_MEMORY=256MB` (P-W1f: two instances fit under the 4 GiB cap; the whole Node process peaked at
   ~95 MB on the 60 s route)*, two link outputs: the SITE form (ES module + `.wasm`) and the ONE-FILE form (`-sSINGLE_FILE=1`, base64).
   *[W15] Both linked with **`-sENVIRONMENT=web,worker,node`** — the module runs in a Worker (the probe linked the one
   file `web` only, `build_pw2.sh` :15-16, and it ran in a Worker only because a release build skips the check), and
   `node` for the tests, so WG3 compares the SAME glue and `.wasm` everywhere. [W10] Compiled with
   **`-ffile-prefix-map=<repo root>/=`** (trailing slash, empty replacement): the mechanism declarations keep
   `std::source_location` file names (`kinds.hpp` :105-106, `wiring.hpp` :623-624), so without it the module carries
   the builder's absolute path (the probe's `.wasm` holds `/home/gonzei/…/src/robots/mechanism_table.hpp`; CI's would
   hold `/home/runner/work/…`). Proven (`recheck2/srcloc/`): GCC and Emscripten then print the SAME relative name;
   `=<root>=.` would print `./inc/…` in GCC but `inc/…` in Emscripten. The native `shulib_route_maker` target takes
   the same flag. [W6] **Legacy Wasm exceptions are DEPRECATED:** Firefox 157 logs "The WebAssembly exception handling
   'try' instruction is deprecated … recompile to use the 'try_table' instruction" on every instance (re-run, A.12);
   Emscripten 6.0.13 still defaults to them (`settings.js` :803). Kept for the 21st — exnref would drop Chrome/Edge
   before 137 (mid-2025), Firefox before 131 and Safari before 18.4, and Node 20 refuses it (Node 22's support NOT
   checked); no browser has announced a removal date. The tripwire: CI's rows run the runners' LATEST Edge, Chrome
   and Firefox on every push, and WG4 shows the plain refusal (never a broken page) if a browser drops them; the page
   tests allow exactly this one warning. Moving to exnref (`-sWASM_LEGACY_EXCEPTIONS=0`, byte-identical in P-W1b) is
   RMP1-C's.*
   Configured with `emcmake cmake -S tools/route_maker/wasm -B build/wasm`. **Emscripten pinned to 6.0.13** (emsdk
   checkout pinned; the SDK's own release commit `3565996` is the natural pin), installed in CI on the pinned
   `ubuntu-24.04` runner with `actions/cache` (~345 MB download, 1.7 GB unpacked). **Never committed:** neither the
   `.wasm` nor the one-file page enters the repository; `ci.yml` builds them on every push (an artifact the Windows job
   tests) and `pages.yml` builds them on every publish. *(This supersedes R11's "`route_maker.html` COMMITTED so a
   double-click works" and its `build.py --check` freshness gate — the downloadable copy comes from the site; a developer
   with emsdk builds it locally with `python3 tools/route_maker/build.py`.)* *The runner is ONE host header,
   `test/route_maker_runner.hpp` (beside RPV1's engine, which it includes unchanged), compiled natively by the suite
   (WG2's native half) AND into the module, so the two runs share one source. On this laptop every `em++` / `emcmake`
   runs with `TMPDIR` inside `build/` (P-W1f).* *[W9] Its `main` is ONE file, `test/route_maker_main.cpp` (`describe`
   AND `preview`), compiled natively as `shulib_route_maker` (the route-builder CI job builds it; WG2's native half;
   what a later helper runs — A.6) AND as the module's `main` — `test/route_maker_wasm_main.cpp` is dropped. [W14]
   **A second compiler gate, from A2e's merge on:** Emscripten's Clang builds the library with `-Werror`, and Clang's
   warnings differ from GCC 13's, so a later chunk verified only by the GCC suite can turn the route-builder job red and
   block a publish. Every chunk that touches `include/`, `src/robots/` or a host-test-plant header the module compiles
   runs the module build (`emcmake cmake -S tools/route_maker/wasm -B build/wasm && cmake --build build/wasm`, ~12 s,
   290 MB) in its done list, with the shared emsdk's path in its prompt — the coordinator's to add.*
   **For the coordinator:** one shared emsdk for
   every worktree at a fixed non-hidden user path (e.g. `~/shulib-tools/emsdk`, or the probe's), named in each run's
   prompt — not 1.7 GB per worktree.
4. **How it stays in step with the library:** built from the SAME tree on every CI run and every publish (the robot
   tables, the facts ledger and the library in the module are exactly that commit's); WG1–WG3 (A.2) on every CI run;
   the page shows "library API 2.x, built from `<commit>` on `<date>`" and the project file's `seen` (D3) records it. **The
   consequence the team must know:** the published builder knows the robots AS OF THE LAST PUBLISH — a fact the builder
   types into `src/robots/robot_one_facts.hpp` reaches the builder's mechanism list and its PREVIEW at the next publish
   (D17), never silently.
5. **The site integration:** `pages.yml` gains three steps after "Prepare the site source": install the pinned emsdk
   (cached), build the module and the app (`tools/route_maker/build.py --out site_src/route-builder/`), so `mkdocs build
   --strict` copies them as static files and the tab's relative links validate; its `paths:` filter gains
   `tools/route_maker/**`, `include/**`, `src/robots/**`, `test/**` (or drops the filter — main moves only at a release);
   ~~pin its runner to `ubuntu-24.04` as `ci.yml` already is (a chore for the coordinator)~~ *[W14] its runner pinned to
   `ubuntu-24.04` (both jobs, `pages.yml` :37 and :100 say `ubuntu-latest` today) **IN A1's OWN COMMIT** — A1 adds a
   native GCC build of the describe program there, and `ubuntu-latest` is GCC 15.2: the `-Werror` trap that kept CI red
   from mid-October to 11-13*. Its guard "the built site
   contains no internal documentation" stays — **checked: the probe's `.wasm` and glue contain none of its patterns and no
   URL**; the app's sources must never mention an internal path. No server code; GitHub Pages serves `.wasm` as
   `application/wasm` (and if a host did not, Emscripten's loader falls back to compiling the downloaded bytes).
   *[W3] **A check AFTER `mkdocs build`** (the CI browser jobs test `build.py`'s output, never the mkdocs-built site,
   so only this step catches a collision before a publish): `site/route-builder/app.html` and the module's `.wasm`
   exist, and `app.html` carries the app's own marker (`<meta name="shulib-app" content="route-builder">`). [W10]
   **RG1:** `build.py` (and this step) fail if the site folder or the one file contains `/home/`, `/Users/`, `C:\`,
   `/runner/` or `.claude` — checked on the `.wasm` itself and on the one file's DECODED module: its base64 hides a path
   from a plain grep (the probe's base64 one file shows no `/home/`; its binary-encoded twin shows the two paths).*
6. **Where it can be hosted, and what each needs:** **docs.shurobotics.com** — nothing more than A.3.5. **shurobotics.com/build**
   (the team's own server behind Cloudflare — NOT inspected) — copy the same folder (or only the one file) under `/build/`;
   it needs HTTPS (for writing into a folder), no server-sent Content-Security-Policy that forbids inline script or
   `'wasm-unsafe-eval'`, and `.wasm`/`.mjs` served with sane types (the one-file form needs none); a version in the file
   names so Cloudflare's cache never serves an old engine. *[W15] And, for `/build/` only, Cloudflare's script
   rewriting OFF — **Rocket Loader, Email Obfuscation and Web Analytics' automatic injection** (a Configuration Rule
   for `/build/*` is enough): each rewrites the page's scripts or injects one, which the page's CSP then blocks or which
   breaks the module order. Checked when the server is (it is NOT inspected).* **A downloaded copy** — nothing at all:
   one file, double-click, no network (P-W2).
7. **Saving and export on Windows (D3, D4).** One small module, `files.mjs`, with two implementations:
   - **Chrome or Edge (the recommended browsers; Edge is on every Windows laptop):** "Connect your shulib folder" ONCE
     (`showDirectoryPicker`, read and write; the handle remembered in the browser, so later visits need one "Allow"
     click). Then **Save** writes `src/team/route_maker.json` in place, and **Export** writes ONLY between the existing
     markers of `src/team/skills_route.cpp` (§6.3's contract), with the re-check's rules unchanged — never a file
     outside `src/team/`, never a region with a hand edit (the hash in the project file, R4/R12) unless the student says
     "overwrite", nothing outside the region touched (the file's writable stream commits whole on close, or not at all).
     **Line endings kept:** this repository has no `.gitattributes`, and Git for Windows checks files out with CRLF by
     default — the export reads the file's own line ending and final newline and writes them back (a CRLF case in the
     tests; GitHub's Windows runner checks out CRLF too). Robot two's mirrored line (D6, R2) is checked by READING robot
     two's autons file through the same folder. *[W12] The marker, hash and line-ending rules are `model/region.mjs`'s
     (pure, node-tested — A.6); `files_fsaccess.mjs` only reads and writes the text. [W11] The builder's OWN test files
     are kept LF on every checkout by two scoped `.gitattributes` — `tools/route_maker/.gitattributes` and
     `test/goldens/route_maker/.gitattributes`, each `* text=auto eol=lf` — so the Windows job's byte-for-byte golden
     compares hold; the students' files keep Git's default (that is what FA3 protects); FA3's CRLF case is BUILT in the
     test from the LF fixture, never committed as a CRLF file.*
   - **Firefox, Safari, anything else:** Save = a download of `route_maker.json`; Open = pick the file; Export = **Copy**
     (always there) or "open `skills_route.cpp` → download the updated file" for the student to put back; robot two's
     line shown as "not checked in this browser". The page says once which of these this browser offers.
8. **Offline at the event:** the **one-file download** (~1.2–1.4 MB with the app's code and the field picture: the
   `-Os` module is ~560 KB of base64). A service worker is NOT built for the 21st (an old cached version is a support
   problem; the one file already covers offline) — RMP1-C may add one.
9. **The page's Content-Security-Policy (R11, amended):** `default-src 'none'; script-src 'unsafe-inline'
   'wasm-unsafe-eval'; style-src 'unsafe-inline'; img-src data: blob:; worker-src blob:; connect-src 'self'` — the
   `'wasm-unsafe-eval'` is REQUIRED (P-W2: without it the preview is blocked); `connect-src 'self'` lets the site form
   fetch its own `.wasm`; still no request to any other host (HS8's test re-pointed). `127.0.0.1` leaves the policy.
   *[W1] **That policy is the ONE FILE's only.** The SITE form loads `preview.mjs` and `worker.mjs` as separate files,
   which `script-src` without `'self'` BLOCKS (re-run: "blocked a script (script-src-elem) at …/preview.mjs", the page
   never ran). **Two exact policies, each written by `build.py` into its own form:**
   **ONE FILE** — `default-src 'none'; script-src 'unsafe-inline' 'wasm-unsafe-eval'; style-src 'unsafe-inline';
   img-src data: blob:; worker-src blob:; connect-src 'self'` (as above; proven twice in Firefox over `file://`,
   main thread and a `blob:` Worker);
   **SITE** — `default-src 'none'; script-src 'self' 'unsafe-inline' 'wasm-unsafe-eval'; style-src 'unsafe-inline';
   img-src 'self' data: blob:; worker-src 'self' blob:; connect-src 'self'` (proven in Firefox with the design's MODULE
   Worker: byte-identical, no CSP message — A.12).
   Both pages carry `<link rel="icon" href="data:,">` (else the browser asks for `/favicon.ico`, the policy blocks it and
   logs an ERROR — seen on both served site pages; with it, no request at all; harmless in the one file). WG5 pins BOTH policies exactly; every page test
   fails on any CSP violation or console error (A.4). [W16] The site form's Worker is an ES module — Firefox 114 or
   newer (MDN; not run) — so the page states **Firefox 115** (Chrome/Edge 97 and Safari 16 unchanged) and
   `browser_check.js` feature-tests module Workers too.*
10. **PREVIEW (D5, D13).** PREVIEW runs the route ON SCREEN (the model, never the last export) through the runner in
    the Worker: ~0.2 s for a 60 s route (P-W1, P-W2) instead of ~5 s. **D13's matching is replaced:** the runner
    executes the page's steps one by one, so it records each page step's first and last pace exactly — a wait or a label
    included — and a trigger's mechanism by its MECH lines as RPV1 already does; no aligner, no RPV1-A2 library
    listener. RPV1's no-pace stop (`std::_Exit` on a looping route) cannot be reached from the page's model (it has no
    loops). *Corrected after the restart (P-W1f): the first print said the build "replaces that hook with a returned
    flag" — RPV1 offers no such hook (the stop is a lambda inside `previewEntry` that always ends in `std::_Exit`,
    `rpv1-a` `test/route_preview_run.hpp` :850-858). None is needed: the runner passes `opt.onLoop`, which receives
    the loop's record; RPV1's `std::_Exit(1)` then ends only the call (`callMain` returns 1); the Worker posts that
    record and RE-CREATES the module instance after any non-zero return (9–216 ms — P-W1f showed the same instance
    would also run on byte-identically, but a stopped run never frees what it built). A run that never returns (a
    loop that logs nothing) is ended by the page: the Worker is terminated after 20 s and the page says so; a 60 s
    route takes ~0.2 s.* Because PREVIEW is this fast, re-running it as the student edits ("live preview") is possible —
    RMP1-C, not now.

### A.4 Proven on Windows AND Linux — the CI jobs and the manual check
**`ci.yml` gains two jobs (A1 adds both; A2e/A2u add their tests to them):**
- **`route-builder` (ubuntu-24.04):** `node --test tools/route_maker/test/` (the model); the describe program → `robots.json`;
  from A2e: the pinned emsdk (cached), the module, WG1–WG2 and WG7 (the no-pace stop, P-W1f); `build.py` makes the
  site folder and the one file; both uploaded as an artifact.
- **`route-builder-browsers` (a matrix):** **`ubuntu-24.04`** — Playwright's Chromium (Chrome for Testing 156) and
  Playwright's Firefox 157; **`windows-latest`** — **the runner's own Edge (channel `msedge`) and Chrome (channel
  `chrome`)**, and Playwright's Firefox 157. Each downloads the artifact, runs `npm ci` (one dev dependency,
  `playwright` **1.64.1** exact, with its lockfile — the app itself has none) and `node --test` over the page tests:
  the SITE form from a tiny static server and the ONE FILE over `file://` with every request refused; build a route by
  clicking (start, drive, curve, cascade level with "home it first"); export to a fake folder (the file layer's tests
  use an in-memory directory handle — the real folder dialog is the manual check's); PREVIEW, and the record's hash equal
  to Node's run of the same artifact (WG3); the old-browser refusal; the CSP present and allowing WebAssembly. The
  Windows job runs `node --test tools/route_maker/test/` too (paths, line endings). Safari: not in CI (no macOS runner
  planned); its floor is stated, not proven.
- ***[W9] `route-builder` also builds the native `shulib_route_maker`** (the same CMake project as the suite, that
  target only) so WG2 and WG2s compare the native program and the module on the same runner.*
- ***[W15] The matrix gains rows:** **`ubuntu-24.04-arm`** — Playwright's Chromium and Firefox (Playwright 1.64.1
  ships both for Linux arm64 — its registry) — because Apple-silicon Macs, ARM Chromebooks and Windows-on-ARM laptops
  run the module on ARM engines (the repository is public, so ARM runners cost nothing — GitHub's documentation, not
  fetched here); and in the **`windows-latest`** row **Playwright's own Chromium (pinned, 156)** beside the runner's
  `msedge` and `chrome` channels — those change with every runner image (Chrome 156 → 157 between the two fetched
  readmes) while Playwright stays at 1.64.1, so a red Edge or Chrome row next to a green pinned Chromium reads as a
  tooling mismatch, not an app fault.*
- ***[W1, W6] Every page test** fails on a CSP violation, a page error or a console ERROR; console WARNINGS are allowed
  only from an exact list — today ONE: Firefox's "The WebAssembly exception handling 'try' instruction is deprecated…".
  WG5 checks the SITE form's policy on the served site AND the ONE FILE's over `file://`.*
- ***[W11] On `windows-latest`** the job first asserts that a golden read from disk has no `\r` (the scoped
  `.gitattributes` working), then runs `node --test`; FA3's CRLF case is made in the test itself.*

**How a run gets its Windows proof *[W5] (new)*.** `ci.yml` runs on a push to ANY branch, and only a push reaches
GitHub's runners. So: when a run's builder reports done and the coordinator's local checks (§7) pass, the coordinator
**pushes the run's branch** (`rmp1-a1`, `rmp1-a2e`, `rmp1-a2u`, `rmp1-a1b`) to GitHub — BEFORE the merge, so the chunk
loop's "verified before merge" holds for Windows too — waits for the whole CI run (~1 h: the ARM gate is the long
pole; the route-builder jobs finish sooner), READS both route-builder jobs green on every row, and only then merges. A
red Windows row is fixed on the branch and pushed again. A run's branch carries every `shulib-v2` commit under it, so
that push also puts those commits on GitHub (on that branch). Pushes are "on the team lead's word" today, and the
permission system has blocked one (NOTES 11-13) — **D20** asks for a standing OK for exactly these pushes.
- **This laptop:** the page tests and screenshots run in **Playwright's Firefox** (A.5); Chromium-based ones in CI only
  (D18).

**The manual check on a team Windows laptop — before the freeze (the team lead, ~10 minutes):**
1. Press Win+R, type `winver`, note the Windows version; open Edge, go to `edge://version`, note the version.
2. Open docs.shurobotics.com, click the **Route builder** tab, then *Open the route builder*. No yellow "too old" box.
3. Drag robot one onto a red start tile; add a **Drive**, a **Curve** and a **Mechanism → cascade → a level**; accept
   "home it first" and type a time.
4. **(from A2u)** Press **PREVIEW**: the robot plays within about a second; drag the timeline.
5. **Connect your shulib folder** (the repository folder) and **Save**: `src/team/route_maker.json` appears in File Explorer.
6. **Export** (after robot two's update has merged): in VS Code, `src/team/skills_route.cpp` changed only between the
   ROUTE MAKER markers; the status bar shows the same line ending (CRLF or LF) as before; `git diff` shows only the region.
7. **Download for offline use**, turn Wi-Fi off, double-click the downloaded file: it opens; PREVIEW works (A2u).
8. Open the builder in **Firefox** if installed: build and Copy work; it says folder writing needs Chrome or Edge.
9. Tell the coordinator anything that looked wrong, with steps 1's two versions.

### A.5 D16 — SETTLED: the screenshot recipe (proven 2026-11-13, A.1)
```bash
# once (network): Playwright pinned, its OWN Firefox -- no snap, nothing in a hidden folder or /tmp
mkdir -p ~/shulib-render-tmp/<run>/pw ~/shulib-render-tmp/<run>/tmp && cd ~/shulib-render-tmp/<run>/pw
# package.json: {"private": true, "type": "module", "dependencies": {"playwright": "1.64.1"}}
export PLAYWRIGHT_BROWSERS_PATH=$HOME/shulib-render-tmp/pw-browsers
export NODE_OPTIONS=--disable-wasm-trap-handler        # Node 20 under the cap (A.1, P-W1)
export TMPDIR=$HOME/shulib-render-tmp/<run>/tmp
(ulimit -v 4194304; timeout 300 npm install --no-audit --no-fund)
(ulimit -v 4194304; timeout 600 npx playwright install firefox)
# every capture: firefox.launch(); page.goto(file://... or http://127.0.0.1:<port>/...);
# page.waitForFunction(<the page's own "done" flag>); page.screenshot({path}); browser.close() in a finally
(ulimit -v 4194304; timeout 600 node shots.mjs)       # wasm/tools/pw2_browsers.mjs is a working example
cp ~/shulib-render-tmp/<run>/shots/*.png <the run's report folder>
```
Chromium-based screenshots (the Edge look) come from the CI job's artifact (D18). §2.5.1's "first hour" proof is now
"run this recipe on a test page" — it worked here in seconds.

### A.6 Host-agnostic — the door to build, upload and debug after the 21st
The route tab (and B's controls and feel) never call a browser API for files or the engine directly; they call four
small interfaces in `tools/route_maker/src/host/`:
- **`files.mjs`** — open/save the project, read a team file, write a marked region ~~(with the hash and line-ending
  rules)~~ *[W12] (the hash and line-ending rules are `model/region.mjs`'s, below; the host only reads and writes text)*.
  Now: `files_fsaccess.mjs` (Chrome/Edge), `files_download.mjs` (everywhere). Later: a VS Code webview's `postMessage`
  version, or a local helper's.
- **`engine.mjs`** — `describe(robot)`, `preview(route, robot) → record`. Now: `engine_wasm.mjs` (the Worker). Later: a
  helper running the SAME runner natively against the team's WORKING tree — so a robot fact typed five minutes ago is in
  the preview before any publish.
- **`robots.mjs`** — where descriptions come from (now: the baked `robots.json`; later: the helper's describe).
- **`capabilities.mjs`** — what this host can do (`writeFolder`, `preview`, and later `build`, `upload`, `brainLog`); the
  shell shows a button only when its capability is there. A later Build / Upload / Debug panel ADDS capabilities and
  pages; the route tab is not rewritten. **What a local helper would add then (NOT built now):** running the robot
  program's build (`make` in the PROS project), uploading it to the brain, reading the brain's log over USB live, the
  working-tree preview above, and writing files without the browser's folder prompt. (Chrome and Edge can also read a
  USB serial port from a page — Web Serial — a possible helper-free way to READ the brain's log; to be investigated then,
  not promised.) The same pages can be hosted in a VS Code webview (its CSP must then allow `'wasm-unsafe-eval'`) or a
  desktop shell; the D7 decision of WHICH is after the 21st.
- ***[W12] The rules live in ONE pure module, not in each host:** `tools/route_maker/src/model/region.mjs` (no DOM, no
  file API; node-tested) finds the markers, checks the hand-edit hash, splices the region and keeps the file's own line
  ending and final newline. A host gives `files.mjs` only primitives — `readText(path)`, `writeText(path, text)`,
  `pickFolder()`, plus save/open for the project file — so a VS Code `postMessage` host or a local helper later
  re-implements a few calls, never the marker, hash or CRLF rules. FA1–FA4 test `region.mjs` directly AND through the
  in-memory host. **How a later helper reaches the page:** it SERVES the same static build from
  `http://127.0.0.1:<port>` (same origin, so the page's `connect-src 'self'` lets it call the helper with no CSP
  change), rather than the public `https://docs.shurobotics.com` page calling localhost — which the page's own
  `connect-src` forbids and which Chrome's local-network-access rules meet with a permission prompt or a block (from
  Chrome's documentation; NOT tried here). [W9] The helper's native preview is exactly `shulib_route_maker preview`
  (A.3.3) — the program WG2 already runs.*

### A.7 The runs, sizes and dates — re-planned *(supersedes D11 (a′), §2.1 and §6.2's dates)*
**The earliest usable thing — build and export a route on Windows and Linux — is RMP1-A1, cut down to the step kinds a
first skills route needs;** what it no longer carries moves to A1b. The preview is split into its engine (A2e, which
needs no page code and can run ~~BESIDE A1~~ *[W4] in its own slot*) and its screen (A2u). Pace as the re-check measured (a builder-day ≈ ½ a
calendar day; one full-day usage stop moves every date a day); the team lead's go assumed Sat 11-14. **Planned on
D15 as ANSWERED (a)** (the team lead, late 11-13): the students' skills-route region and the driver files'
controls/feel regions stay open after the ~Nov 18 freeze, so a route EXPORTED from the builder up to the robots'
packing (~Nov 20 evening) still reaches the robots — through the full checks and an Auton Tester run on the robot.
**Whether the builder ITSELF may still merge and publish after the 18th is new D19:** it is not robot code, but D15
(a)'s own wording froze "the library, the programs and the tools" — so the table after this one gives both answers.

| Run | What | Size (builder-days) | Needs at its tip | Lands |
|---|---|---|---|---|
| **RMP1-A1** — the web route builder, FIRST USABLE | the shell and design tokens; the field (the real picture, pan, zoom, snap); **start, drive (turn-first), turn, curve, mechanism (with RC25's home rule), wait until done, wait, if it fails + labels**, with their checks (RC1–RC3, RC15–RC21, RC24–RC26) on both robots' step lists; undo/redo; the export (D4) with its goldens and the C++ golden test; save/open; `files.mjs` (Chrome/Edge folder + downloads + Copy, CRLF kept); the describe program → `robots.json` in CI; the site tab + the one-file download; **CI on Linux AND Windows (A.4)**; guide 22's first steps; *[W3, W7, W10–W12, W14] `app.html` + the post-build check, `numbers.mjs` + NF1, `region.mjs`, RG1, the scoped `.gitattributes`, `pages.yml` pinned* | **~5–6 + ~1 review** *[W] ~5¼–6¼ + ~1* | ROB1-A merged (robot one's list — staged now); writing into the folder: ROB1-BT with §6.3's contract (Copy until then) | **merged ~Tue 11-17 to Wed 11-18; on the site the same day with D17 (a)** |
| **RMP1-A2e** — the preview ENGINE | the WebAssembly CMake project + the pinned emsdk in CI; the runner (every A1 and A1b step kind); the record as JSON (was RPV1's `--json`); exact step attribution (A.3.10); WG1–WG3 and WG7 (the no-pace stop, P-W1f); *[W2, W8–W10, W15] the native `preview`, WG2s's seeded sweep, WG8, the prefix map, `web,worker,node`, the ARM and pinned-Chromium rows* | **~2½–3 + ~½** *[W] ~3–3½ + ~½* | RPV1-A merged; D3's schema (this brief) | ~~beside A1 if a slot frees Sun 11-15 or Mon 11-16 → merged ~Tue 11-17 to Wed 11-18~~ *[W4] **in slot 3 from AF1S-A2's merge (~Tue 11-17) → merged ~Wed 11-18 evening to Thu 11-19** (RECOMMENDED: no 4th job, nothing displaced); in one slot after A1: ~Thu 11-19 to Fri 11-20* |
| **RMP1-A2u** — PREVIEW in the page | the Worker, progress and cancel; playback with the timeline scrubber; the step list lit from the runner's spans; the two-robot PREVIEW (when MIRROR1-A has merged); guide 22's PREVIEW section | **~1½–2 + ~½** | A1 merged to START *[W4] (against a stand-in engine: a schema-1 record behind `engine.mjs`)*; A2e merged before ITS merge | ~~was ~Wed 11-18 to Thu 11-19 (A2e beside A1); ~Thu 11-19 to Fri 11-20 (in series)~~ *[W4] **~Thu 11-19 to Fri 11-20** (started at A1's merge, beside A2e); all in one slot after A1: ~Fri 11-20 to Sat 11-21 — may miss the event* |
| **RMP1-A1b** — the rest of the route tab | triggers partway (RC4–RC10); the wall reset (RC11–RC14 — robot one declares no wall sensors yet, so every reset is refused on it today); the mirror ghost and band (RC22); the time bar (D12); the Toggle steps greyed until AF1S-A2 / robot one's flipper | **~2–2½ + ~½** | A1 merged | ~~was ~Thu 11-19 to Fri 11-20, or after the 21st~~ *[W4] after the 21st, unless a slot frees by ~Wed 11-18 (then ~Fri 11-20): with A2u in slot 1 and A2e in slot 3, no slot is free before ~Thu 11-19* |
| **RMP1-B** — Controls and Driving feel | as §1.2, writing through `files.mjs` | ~2½–3 + ~½ | A1's shell | after the 21st unless a slot frees (as before) |
| **RMP1-C** — polish | as §1.3, plus live preview while editing and (if wanted) a service worker | ~1½–2 + ~½ | everything above + the usability session | after the 21st |

**In all ~15–18½ builder-days + ~3½ review** *[W] — **~15¾–19¼ + ~3½** after the amendment's re-check (A.12)* (the re-check said ~12–15 + ~2½). **What the increase buys:** the site, the
one-file download and the Windows CI (~1), the file layer for browsers (~¾), the WebAssembly engine with its golden
test (~2½–3, replacing RPV1's `--json` and the step aligner, ~1½); minus the helper (~¾); and ~1 more from splitting
into smaller runs (each its own start-up and review) so the first usable piece lands first.

**Is PREVIEW sooner or later than the helper design? Honestly: about the same date, and only this design reaches
Windows.** The helper design's PREVIEW was ~2½–3½ + ½ after A1 (~Thu 11-19 to Fri 11-20) — and it worked only where
Python, CMake and a C++ compiler are installed: this Linux laptop. On the team's Windows laptops it would have needed a
C++ toolchain on each one and a library that has never been built on Windows (MSVC never tried) — not before the 21st.
~~The WebAssembly PREVIEW is ~1½ builder-days MORE work, but its engine half needs no page code, so: **with A2e in its own
slot beside A1 (from ~Sun 11-15 or Mon 11-16), PREVIEW lands ~Wed 11-18 to Thu 11-19 — about a day SOONER; in one slot
after A1, ~Thu 11-19 to Fri 11-20 — the same or a day later.**~~ Either way it runs on Windows, Mac, Linux and
Chromebooks, in ~0.2 s instead of ~5 s. ~~The slot it needs pushes AF1S-A2 or the docs amendment by ~1 day — the team
lead's call (§6.2's slot table otherwise stands).~~

*[W4] Re-planned within the THREE-job rule (11-11). The WebAssembly PREVIEW is ~2–2½ builder-days MORE work than the
helper's (A2e + A2u ≈ 5½–6½ with review, against ~3–4), but it splits: the engine (A2e) needs no page code, and the
screen (A2u) can be built against a stand-in engine behind `engine.mjs` (A.6) and wired to the real one at the end.
The free slot: on the re-check's table Tue–Wed's slot 3 was RMP1-B's, and RMP1-B is now after the 21st.*
- ***RECOMMENDED: A2e in slot 3 from AF1S-A2's merge (~Tue 11-17); A2u in slot 1 from A1's merge, merging after A2e.
  PREVIEW ~Thu 11-19 to Fri 11-20 — about the SAME date as the helper design.** Three jobs at every moment; AF1S-A2 and
  the docs untouched. If AF1S-A2 runs late, A2e waits for its slot.*
- ***All in one slot after A1:** ~Fri 11-20 to Sat 11-21 — a day LATER, and it can miss the event.*
- ***A day SOONER (~Wed 11-18 to Thu 11-19) only by starting A2e on Sun 11-15 beside A1** — and Sun–Mon's three slots
  are A1, ROB1-BT and AF1S-A2, so that is either a FOURTH job (against the 11-11 rule) or AF1S-A2 a day later. AF1S-A2
  is the camera in autons — library and robot-setup code, which FREEZES ON THE 18TH under the D15 answer — and the
  auto-flipper is a before-the-21st item. **NOT recommended**; the team lead's call if he wants PREVIEW a day sooner at
  that price.*
- ***Every date also carries** ~1 h of CI on the run's pushed branch before its merge (W5, A.4) and, for the team to
  see it on the site, a publish (D17: ~30–45 min and the team lead's OK for the push). A full-day usage stop moves every
  date a day; with PREVIEW at ~Fri 11-20, one stop puts it past the robots' packing.*

**What the team has for the 21st, by D19's answer** (D15 answered; dates as above, one usage stop = a day later):

| | **D19 (a) RECOMMENDED — the builder stays open to the 21st** | **D19 (b) — the builder freezes on the 18th too** |
|---|---|---|
| Build and export a route on Windows and Linux (A1) | from ~Nov 17–18, published at its merge (D17 (a)) | the same IF A1 merges AND is published by the 18th; a day's slip and it misses — the route is then typed by hand (guide 21) |
| PREVIEW in the page (A2e + A2u) | ~~was ~Nov 18–19 (A2e beside A1) or ~Nov 19–20 (in series)~~ *[W4] **~Nov 19–20** (A2e in slot 3 from ~Nov 17, A2u from A1's merge), published the same day (D17 (a)) or taken from CI's run page (D17 (b)); in one slot ~Nov 20–21 — may miss* | not expected: A2u needs A1 merged first, so it lands at or after the 18th. PREVIEW before and at the event = RPV1's command, run by the coordinator on this laptop (Linux only) |
| Triggers, wall resets, the mirror view (A1b) | ~~was ~Nov 19–20, or~~ *[W4]* after the 21st *(unless a slot frees by ~Nov 18; until then typed by hand inside the region)* | after the 21st (until then typed by hand inside the region; the builder never overwrites a hand edit, D4) |
| Students' route changes reaching the robots | until packing, ~Nov 20 evening (D15) | the same (D15) — with whatever builder was published by the 18th |

### A.8 Tests and checks that change
- **Gone with the helper:** HS1 (bind 127.0.0.1), HS2 (launch token, Origin), HS7 (the helper's bounds), RM12; RM13 and
  HS6 become a page test ("PREVIEW runs the model on screen, not the last export"); RM22 (the aligner) goes with D13 (a).
- **Moved to `files.mjs` (node, with an in-memory directory handle):** HS3–HS5 and HS9/HS10 become **FA1** only between
  existing markers under `src/team/`; **FA2** a hand edit refused unless "overwrite"; **FA3** nothing outside the region
  changes, **CRLF and the final newline kept** (a CRLF fixture); **FA4** robot two's mirrored line found / missing;
  HS9's "the export in a scratch copy of the tree keeps the suite green" stays (Linux job).
- **New:** **WG1–WG3** (A.2); **WG4** the old-browser refusal (V8 without legacy exceptions; a browser without
  WebAssembly); **WG5** the CSP present, exact, and allowing WebAssembly; **WG6** the one file works over `file://`
  with every request refused (A.4); **WG7** the no-pace stop inside the module (P-W1f's looping route as a fixture
  route the runner accepts only in its test build): `callMain` returns 1 with the loop's record, the Worker re-creates
  the instance, the next PREVIEW equals a fresh instance's; and a Worker that does not answer in 20 s is terminated with
  the page's sentence (a fake engine that never returns). HS8 re-pointed: no URL except the XML namespace URIs; no
  request to another host (checked on the probe's glue and one file: none).
- **New mutations:** RM27 the runner maps `startMove` to `move` (blocking) → WG1 red; RM28 the runner builds a preset goal
  with `computed = true` → WG1 red; RM29 the CSP without `'wasm-unsafe-eval'` → WG5 red (P-W2 showed the failure);
  RM30 the export writes LF into a CRLF file → FA3 red; RM31 one byte of the browser-check module changed → WG4 red; RM32 the runner leaves `opt.onLoop` unset (the loop's
  record lost) → WG7 red.
- **§7's checks gain:** `NODE_OPTIONS=--disable-wasm-trap-handler` on every capped `node`; `emcmake cmake -S
  tools/route_maker/wasm -B build/wasm && cmake --build build/wasm` (under the cap: 290 MB); WG1–WG2 under Node; the
  Firefox page tests through A.5's recipe; the CI run's Linux AND Windows browser jobs READ, green.
- ***[W1–W16] Added by the amendment's re-check (A.12):***
  - *FA1–FA4 run against `model/region.mjs` directly AND through the in-memory host (W12); FA3's CRLF fixture is made
    in the test from the LF one (W11).*
  - ***WG2** compares `shulib_route_maker preview` and the module under Node, positions within 1e-5 in and headings
    within 1e-6 rad, the measured maximum printed (W2, W9); **WG2s** a seeded sweep — 300 random routes of every step
    kind per CI run (the generator in Node, the same seeds every run; the full 3,000 once in A2e's report), steps,
    verdicts, paces and lines EXACT, poses within WG2's bounds; a seed that lands on a threshold or printed-decimal
    boundary is replaced only after the run shows that, logged by number (W2).*
  - ***WG4** also: a browser without module Workers gets the refusal for PREVIEW (W16). **WG5** BOTH policies exact —
    the SITE form's on the served site, the ONE FILE's over `file://` — and the `data:` favicon; every page test fails
    on a CSP violation, page error or console error, warnings only from the exact list (W1, W6).*
  - ***WG8** a golden in which every pair of numbers in every step kind differs (x ≠ y, a curve's points all distinct,
    a timeout ≠ a speed cap), run by WG1 AND WG2 (W8). **NF1** every number in the line format equals the export's,
    character for character, over every golden; numbers typed with more decimals are stored rounded (W7). **RG1** no
    `/home/`, `/Users/`, `C:\`, `/runner/` or `.claude` in the site folder, the `.wasm` or the one file's DECODED module
    (W10). **PG1** (in
    `pages.yml`, after `mkdocs build`) `site/route-builder/app.html` and the `.wasm` exist and `app.html` carries the
    app's marker (W3). **CR1** (Windows job) a golden read from disk has no `\r` (W11).*
  - ***Mutations:** RM29 becomes two — the ONE FILE's policy without `'wasm-unsafe-eval'` → WG5 red; **RM33** the SITE
    policy without `'self'` in `script-src` → WG5 and every SITE-form page test red (W1); **RM34** `route_lines.mjs`
    prints a number with one more decimal than `export.mjs` → NF1 red (W7); **RM35** the runner reads two numbers inside
    one call's argument list → WG2 red on WG8 natively (WG1 stays green — the reason WG8 runs in WG2) (W8); **RM36** the
    app written as `route-builder/index.html` → PG1 red (W3); **RM37** the prefix map dropped → RG1 red (W10).*

### A.9 New decisions
**D17 — Publishing the route builder before the 21st. RECOMMENDED (a).** The site publishes only from `main`, at a
release; two releases are pre-authorized (after the 11-11 merges; at the ~Nov 18 freeze).
- **(a) RECOMMENDED: a site publish at each RMP1 merge before the 21st (A1, A2u, A1b if it lands) and whenever robot
  one's facts change** — ~~the release tool, the docs gates, ~1 minute of publishing each; the robot code is not touched by
  a publish~~. *[W13] **What a publish really is:** a full RELEASE — `python3 tools/release.py check → stage → push`
  (`docs/internal/RELEASING.md` :11-14; the 11-13 release took ~45 min from the decision to the push, with the team
  lead's OK for the push) — so **~30–45 min of the coordinator's gates plus his OK each time**, and it carries EVERY
  change merged to `shulib-v2` by then (RPV1, MIRROR1, ROB1-BT, AF1S-A2 …) to the public `main` and the public docs. No
  robot is touched until someone builds and uploads, but it is a library release, not "a page update".* Then the team
  uses the builder from ~Nov 17–18 and PREVIEW from ~Nov ~~18–20~~ *19–20*. *A publish after the 18th needs D19 (a) as
  well.*
- **(b) Only the pre-authorized Nov 18 release.** **Cost:** A1 reaches the team on Nov 18 at the earliest; PREVIEW and any
  later robot fact do not reach the published builder before the 21st (a coordinator-built one file passed around is
  the only way). *[W13] A cleaner way than a passed-around file: **every CI run uploads the ONE FILE as an artifact**
  (A.4), so the team lead (signed in to GitHub) can download it from that run's page and copy it to a team laptop — no
  release, works offline; it carries no laptop path (RG1, W10).*

**D18 — Chromium-based page tests on THIS laptop. RECOMMENDED (a) — the coordinator's call unless the team lead wants it.**
- **(a) RECOMMENDED: in CI only** (Linux Chromium, Windows Edge and Chrome); locally, Firefox (proven under the cap).
- **(b) A real-memory cap for Chromium only** (a cgroup, `systemd-run --user --scope -p MemoryMax=4G …`) — it bounds
  memory used rather than address space reserved, which Chromium needs far beyond 4 GiB. **Cost:** it changes how the
  2026-10-15 memory rule is enforced, for one program — needs the team lead's OK.

**D19 — Is the route builder ITSELF inside the ~Nov 18 freeze? RECOMMENDED (a).** D15's answer keeps the students'
regions open; D15 (a) as written also froze "the tools" — here, the builder's code (`tools/route_maker/**`, its
`test/route_maker_*` files), its CI jobs, its docs page and the site publish. None of it is compiled into a robot.
- **(a) RECOMMENDED: the builder stays open to the 21st, like the students' regions** — each change through its own
  checks (the Linux AND Windows CI jobs, the suite with the goldens, `git diff -- src/ Makefile include/` EMPTY in every
  run's done list), a publish changes only the website (D17) *[W13] — strictly, a publish is a full release of
  everything merged by then (D17); after the 18th the library and robot setup are frozen, so it carries only the
  builder and the students' regions*, and the robot code it writes is the students', under D15.
  Then PREVIEW in the page reaches the team before the event (A.7's table).
- **(b) The builder freezes on the 18th too.** **Cost:** PREVIEW in the page does not reach the team before the 21st;
  A1 itself misses if it slips a day; the routes are then exported by the last builder published by the 18th, or
  typed by hand.

**D20 — The pushes that prove Windows. RECOMMENDED (a).** *[W5] new.* "Works on Windows" is proven only by GitHub's
Windows runner, which runs only on a push (`ci.yml`: `on: push:`, any branch). Today a push waits for the team lead's
word (and the permission system has blocked one).
- **(a) RECOMMENDED: a standing OK for the coordinator to push each RMP1 run's own branch** (`rmp1-a1`, `rmp1-a2e`,
  `rmp1-a2u`, `rmp1-a1b`) to GitHub when the local checks pass, BEFORE its merge, and to re-push it after a CI fix.
  Each push also carries the `shulib-v2` commits under the branch (on that branch). `main`, `release/v2` and the site
  stay under D17 and the release rules. If the permission system still asks, the coordinator asks him to click.
- **(b) Each push on his word.** **Cost:** every run's Windows proof waits for him (~1 h of CI after it); a run that
  finishes at night merges the next day — at freeze week's pace that is about a day per run.

### A.10 What stays unverified
A run on a real `windows-latest` runner (A1's first CI run); Edge and Chrome running the module (Chromium could not start
under the cap here — the same V8 as Node, so expected, not proven); writing into a folder from the ONE FILE opened over
`file://` in Edge (the File System Access API needs a secure context; whether Edge counts `file://` as one is the
manual check's step 7 and A1's first-hour proof — else the one file offers Copy and downloads); a Web Worker created from
a `blob:` URL in the one file on `file://` (A2u's first-hour proof; else the run happens on the page itself, ~0.2 s)
*[W15] — PROVEN in Firefox 157 under the ONE FILE's policy (byte-identical; the checker's run, re-run here); Edge and
Chrome still open*;
Safari; the real docs site serving the app (no publish from here); every robot-one time (placeholder facts, as before).
*(after the restart)* The no-pace stop was proven in Node (P-W1f), not yet inside a browser Worker (A2u's WG7 in the
browser jobs); WG2's tolerances were measured on three routes (P-W1b), not yet on every step kind (A2e's goldens).
*[W1–W16] (after the amendment's re-check) Now measured: WG2's margin over 3,000 seeded routes of every step kind
(W2) — but the EXACT rule stays empirical, and the sweep ran on robot x's fixture, not on robot one or two. Still
unverified: the SITE policy in Edge and Chrome (proven in Firefox only, with the module Worker); Firefox 114–156 with a
module Worker (the floor is MDN's, W16); any ARM engine (no ARM machine here — the CI row is the first run, W15); that
GitHub's ARM runners are free for this public repository (documentation, not fetched); whether Node 22 (CI's) accepts
exnref (W6, only relevant when moving off legacy exceptions); Chrome's local-network-access behaviour for a later
helper (W12, documentation); every Cloudflare setting on shurobotics.com (the server is not inspected, W15); the
`-ffile-prefix-map` form on the REAL module build (proven on a small probe in both compilers, W10).*

### A.11 The changed decisions in plain words (for the team lead)
- **The route builder becomes a web page on the docs site** (a "Route builder" tab) — open it in Edge or Chrome on any
  Windows laptop, a Mac, Linux or a Chromebook, with nothing to install. There is also a **one-file download** for the
  event with no internet. (Was: a small program started from a terminal on this laptop.)
- **PREVIEW runs the REAL library inside the web page.** I compiled the library, the host test plant and the preview's
  engine for the browser today: no change to the library, about 200 KB to download, and a full 60-second skills route
  plays in about a fifth of a second. It gives the same steps, results and times as the normal build; the robot's
  position can differ in the ~15th digit because the computer's sine and cosine are rounded slightly differently. It
  comes out exactly the same in every browser tried so far (on Linux); CI checks Windows once it is built. *[W2] A
  stress test of 3,000 random routes since: the same steps and results every time; positions apart by less than a
  millionth of an inch.*
- **Saving and exporting:** in **Edge or Chrome**, you connect the shulib folder once and the builder saves the route and
  writes the code into the right place itself (keeping Windows line endings). In other browsers it downloads the file or
  copies the code.
- **"Works on Windows" is tested on every change:** CI runs the builder in Edge, Chrome and Firefox on Windows, and
  Chromium and Firefox on Linux; plus a 10-minute check list for you on a team Windows laptop (A.4). *[W5, W15] Also
  on an ARM Linux machine (what Apple-silicon Macs and ARM Chromebooks use), and with one fixed Chromium beside the
  ever-updating Edge and Chrome. Each piece is pushed to GitHub and has to pass these checks BEFORE it is merged — a
  push needs your word today, so D20 asks for a standing OK for exactly those pushes.*
- **The order of work:** first a usable builder (the common steps, export, save — ~Nov 17–18), ~~the preview engine built
  beside it if a slot is free, then PREVIEW in the page (~Nov 18–20); triggers, wall resets and the mirror view right
  after~~ *[W4] then PREVIEW in the page **~Nov 19–20**: its engine is built in the third job slot once the auto-flipper's
  camera piece (AF1S-A2) has merged (~Nov 17), and its screen starts as soon as the builder merges. We stay at three jobs
  at once and the auto-flipper is not delayed. PREVIEW could come about a day sooner only by running a fourth job or by
  pushing the auto-flipper's camera piece a day closer to the Nov 18 library freeze — I do not recommend either.
  Triggers, wall resets and the mirror view most likely after the 21st (typed by hand until then)*; driver controls and
  polish after the 21st. **Total work is ~3 builder-days more than before** *[W] (~3¾ after this re-check's fixes)* —
  the price of Windows with nothing installed — but PREVIEW lands about the same day, and only this way reaches
  Windows. *A usage stop of a full day puts PREVIEW past the robots' packing (~Nov 20 evening).*
- **Build, upload and debug** stay for after the 21st, as you said; the builder is written so that panel (VS Code, a
  desktop app, or this web app plus a small helper on the laptop) reuses it without a rewrite. Nothing depends on
  shurobotics.com; it can be copied there later.
- **Your D15 answer is in:** the students' route, and the driver controls and feel, can keep changing after the
  ~Nov 18 freeze — each change through the full checks and an Auton Tester run on the robot; the library and the robot
  setup freeze on the 18th. The dates above are planned on it.
- ~~**Two questions for you:**~~ *[W5, W13] **Three questions for you:*** **D17** — may the coordinator publish the docs
  site each time a route-builder piece merges before the 21st? (Otherwise the team only sees it on Nov 18, and PREVIEW
  not before the event.) *Each publish is a full release — about 30–45 minutes of checks, your OK for the push, and it
  puts everything merged by then on the public site and `main`. If a publish cannot happen in time, the one-file
  builder can also be downloaded from any CI run's page on GitHub and copied to a team laptop.* **D19** — does the
  route builder ITSELF freeze on the 18th? It never goes on a robot. If it stays open (recommended), PREVIEW in the page
  reaches the team ~Nov ~~18–20~~ *19–20*; if it freezes, the builder they get is whatever is published by the 18th, and
  PREVIEW at the event is the coordinator running it on this laptop. *[W5] **D20** — may the coordinator push each
  route-builder piece's own branch to GitHub (before it merges) so the Windows checks run on it? Recommended yes, as a
  standing OK for those branches only.* **D18** is the coordinator's (Chromium tests run on GitHub, not on this
  laptop).

### A.12 Re-check of this amendment (2026-11-14 ~00:00–01:30; the checker's 15 findings W1–W15, each RE-VERIFIED, and W16 found while re-verifying)

**How it was checked.** The checker's probes were opened and RE-RUN, not taken on its word, every binary under
`(ulimit -v 4194304; timeout …)`: its CSP harness in Playwright's Firefox 157 (copied to
`~/shulib-render-tmp/rmp1-amend/recheck2/check.mjs`, its own screenshots in `recheck2/shots/`); its fuzz comparison
over its stored outputs (`wasm/checker/cmp_fuzz.py` on `out/fz*_{n,w}.txt`); its exact double-rounding count
(`checker/src/double_round.py` 3 and 4); the mkdocs collision with its mkdocs 1.6.1 (`wasm/recheck2/mk/`). New small
probes: `wasm/recheck2/srcloc/` (the prefix map, GCC and Emscripten), `wasm/recheck2/argorder/` (argument order, GCC 13,
Clang 18, Emscripten) and `~/shulib-render-tmp/rmp1-amend/recheck2/site-final/` (the corrected SITE policy with a
MODULE Worker and a `data:` favicon, Firefox; `recheck2/check2.mjs`). The harnesses, the page and the screenshots are
copied to `wasm/recheck2/browser/` and `wasm/recheck2/shots/`. Read: `ci.yml` / `pages.yml`, `gh run list` and
`gh repo view` (read only), NOTES, `docs/internal/RELEASING.md`, the usage-pacing rule, Emscripten's `settings.js`,
Playwright 1.64.1's browser registry, the AF1S chunk file.

| # | Finding (severity) | Re-verification | Disposition — where fixed (marked *[Wn]*) |
|---|---|---|---|
| **W1** | A.3.9's CSP blocks the SITE form; only the one file was tested (major) | **REAL.** Re-run: the site page with A.3.9's policy — `Content-Security-Policy: The page's settings blocked a script (script-src-elem) at …/site-csp/preview.mjs`, no verdict in 20 s; with `'self'` in `script-src` and `worker-src` — PASS, the three routes byte-identical to Node; the favicon blocked by `img-src` (a console ERROR) in both. **New here:** the brief's ACTUAL site design — a MODULE Worker (`worker.mjs` importing `preview.mjs`) — under `script-src 'self' 'unsafe-inline' 'wasm-unsafe-eval'; img-src 'self' data: blob:; worker-src 'self' blob:` with `<link rel="icon" href="data:,">`: **PASS**, byte-identical; the server saw only the page, `worker.mjs`, `preview.mjs` and `preview.wasm` (no favicon request); no CSP message, only W6's warning (`wasm/recheck2/shots/site-final-module-worker.png`, opened) | **FIXED.** A.3.9: TWO exact policies — the ONE FILE keeps the proven one, the SITE form adds `'self'` —, the `data:` favicon in both; WG5 pins both; every page test fails on a CSP violation or a console error (A.4); RM29 per form, new RM33; §1.1, §7, landmine 21 |
| **W2** | WG2's margin overstated: 3 routes, not a sweep (major) | **REAL.** `cmp_fuzz.py` re-run: seeds 1–300 and 301–3000 — **0 of 3,000 routes** differ in any step, verdict, pace, summary or line (4,068 + 36,483 = **40,551 steps**); poses at most **3.17e-7 in, 2.24e-8 rad**; the 25 worst seeds at every pace: **3.51e-7 in, 2.49e-8 rad**. So 1e-6 / 1e-7 was ~2.9× / ~4×, not 25× | **FIXED.** WG2's bounds **1e-5 in / 1e-6 rad** (~28× / ~40× the sweep's maximum; a hundred-thousandth of an inch), the measured maximum printed beside the bound on every run; new **WG2s** (a seeded sweep in CI, 300 routes; the full 3,000 in A2e's report); the remedy written: a golden or seed that sits on a threshold or a printed-decimal boundary is REPLACED, logged by name, only after the run shows it is that — never loosened (A.2); landmine 17; §4 |
| **W3** | The docs page silently overwrites the app at publish (major) | **REAL.** Re-run, mkdocs 1.6.1: `route-builder.md` + the app at `route-builder/index.html`, `--strict` → **exit 0, no warning**, `site/route-builder/index.html` = the rendered docs page, the app's marker gone. The app as `route-builder/app.html`: both survive, the page's link rewritten to `href="app.html"` | **FIXED.** The app is **`route-builder/app.html`**, never `index.html` (A.3.1, A.3.5, §1.1); a `pages.yml` step AFTER `mkdocs build` checks `app.html` and the `.wasm` exist and that `app.html` carries the app's marker (A.3.5, §7); landmine 26 |
| **W4** | "A2e beside A1" is a 4th job or pushes AF1S-A2 — library code — toward the freeze (major) | **REAL.** The usage rule (11-11): **MAX 3 jobs**; the re-check's slot table: Sun–Mon = A1, ROB1-BT, AF1S-A2; AF1S-A2 = "the camera in autons" (`docs/internal/chunks/AF1S-skills-toggle-check-flip.md` :3-6) — library and robot-setup code, frozen on the 18th by D15's answer; the auto-flipper is a before-the-21st item. **What the finding missed:** Tue–Wed's slot 3 was RMP1-B's, and A.7 moved RMP1-B after the 21st — that slot is FREE from AF1S-A2's merge | **FIXED.** A2e planned **in slot 3 from AF1S-A2's merge (~Tue 11-17)** — no 4th job, AF1S-A2 and the docs untouched; **A2u starts at A1's merge against a stand-in engine** (a schema-1 record behind `engine.mjs`) and merges after A2e. "Beside A1 from Sun" stays only as the NOT-recommended way to be a day sooner, with its cost said. A.7, A.11, §6.2, D11, §9 |
| **W5** | "Proven on Windows" needs pushes nobody approved; §6.1's CI-pin note stale (major) | **REAL.** `ci.yml` runs `on: push:` (ANY branch) — the proof IS a push; NOTES: pushes "on jal's word", one blocked by the permission classifier (11-13). **Stale, confirmed:** `origin/shulib-v2` = `a27e2bc`; `gh run list`: run 43481975300 on `a27e2bc` **success** (2026-11-13 18:22Z, 58 min — the ARM gate is the long pole); `d684aa7`, `49822e5` local only | **FIXED.** Each RMP1 run's branch is pushed for CI BEFORE its merge (the chunk loop's "verified before merge" kept), under a standing OK asked as **new D20**; ~1 h of CI per run in the dates; §6.1 corrected. A.4, A.7, A.9, A.11, §7 |
| **W6** | Legacy Wasm exceptions are deprecated (minor) | **REAL.** Re-run: Firefox 157 logged "The WebAssembly exception handling 'try' instruction is deprecated and should no longer be used. Please recompile to use the 'try_table' instruction instead." once per instance; `settings.js` :803 `WASM_LEGACY_EXCEPTIONS = true` | **FIXED (recorded; the choice kept for the 21st).** A.3.3: why legacy stays, the tripwire (CI's latest-browser rows + WG4's refusal), the page tests allow exactly this one warning, exnref in RMP1-C; A.10 |
| **W7** | Nothing makes PREVIEW run the numbers the export PRINTS (minor) | **REAL.** No rule tied `route_lines.mjs` to `export.mjs`. The literal-vs-`strtod` evidence holds: P-W1c (1–3 decimals, both builds) and the checker's exact count, re-run: **0 of 64,800** (3 decimals) and **0 of 648,000** (4 decimals) double-round differently through x86-64's 80-bit `long double` | **FIXED.** ONE formatter, `model/numbers.mjs`, for the export AND the line format; every number stored already rounded at edit time (≤ 3 decimals — P-W1c's proven range); test **NF1** (every number in the line format equals the export's, character for character) and **RM34**. A.2, §1.1, landmine 25 |
| **W8** | GCC and Clang evaluate call arguments in opposite orders (minor) | **REAL.** `recheck2/argorder/`: `driveTo(L{num()}, L{num()})` → `x=2 y=1` under GCC 13 `-O0`, `x=1 y=2` under Clang 18 and Emscripten; `const double x = num(), y = num();` → `x=1 y=2` in all three. **Worse than stated:** in WebAssembly (left to right) the bug is INVISIBLE to WG1 — only the native GCC run (WG2) shows it | **FIXED.** The rule in A.2 and landmine 24; golden **WG8** with x ≠ y (and every pair of numbers different) in every step kind; **RM35** |
| **W9** | WG2's mechanics: where the native and WebAssembly records meet (minor) | **REAL.** §1.1 DROPPED the native `preview`, so WG2's native record had no producer | **FIXED.** `test/route_maker_main.cpp` = ONE `main` (`describe`, `preview`) for the native program `shulib_route_maker` AND the module (`route_maker_wasm_main.cpp` dropped); the route-builder CI job builds the native program; WG2 under Node runs both on each golden. The later helper runs the same program (A.6). A.2, A.3.3, A.4, §1.1 |
| **W10** | The module embeds absolute source paths (minor) | **REAL.** `strings` on `wasm/out/eh_Os/pw1.wasm`: `/home/gonzei/…/wasm/tree/src/robots/mechanism_table.hpp` and `…/test/mechanism_fixtures.hpp` — `std::source_location` defaults (`kinds.hpp` :105-106, `wiring.hpp` :623-624). `recheck2/srcloc/`: the base64 one file HIDES them from a plain grep (its binary-encoded twin shows both). **`-ffile-prefix-map=<root>/=`** (trailing slash, empty) removes them in BOTH compilers and both print the SAME name (`inc/decl.hpp:5`); the checker's `=<root>=.` printed `./inc/decl.hpp` in GCC but `inc/decl.hpp` in Emscripten — a WG2 split if a record line ever quoted a location (none did in the probes) | **FIXED.** The trailing-slash form in the module AND the native target; **RG1**: `build.py` and `pages.yml` fail if the site folder or the one file contains `/home/`, `/Users/`, `C:\`, `/runner/` or `.claude`. A.3.3, A.3.5, §7, landmine 27 |
| **W11** | The Windows job's `node --test` would read CRLF goldens (minor) | **REAL.** No `.gitattributes` (`ls -a` at the root); A.3.7 itself says the Windows runner checks out CRLF; every byte-for-byte golden compare would fail there | **FIXED.** Two SCOPED `.gitattributes` (`tools/route_maker/` and `test/goldens/route_maker/`: `* text=auto eol=lf`) — the students' files untouched; FA3's CRLF case BUILT in the test from the LF fixture (no CRLF file committed); the Windows job asserts a golden read from disk has no `\r` (a lost `.gitattributes` is caught). A.4, §1.1, landmine 19 |
| **W12** | Region rules belong in ONE pure module; a later helper should serve the page (minor) | **REAL** (a design gap; nothing to run). A.6 put the splice, hash and line-ending rules inside each `files_*.mjs`; a public https page calling `http://127.0.0.1` meets its own `connect-src 'self'` and Chrome's local-network-access rules (documentation, not tried) | **FIXED.** `model/region.mjs` (pure, node-tested); hosts give only read/write/pick; the later helper SERVES the same static build from 127.0.0.1. A.3.7, A.6, A.8 |
| **W13** | D17 understates a publish (minor) | **REAL.** `pages.yml` publishes only from `main` (its header); `RELEASING.md` :11-14 `check → stage → push`; NOTES 11-13: release #1 decided ~11:45, pushed ~12:30, with the team lead's push OK | **FIXED.** D17 (a)'s cost restated (~30–45 min, his OK for the push, and every change merged by then goes to public `main`); D17 (b) gains the CI-artifact route. A.9, A.11 |
| **W14** | A second compiler gate for every library chunk; `pages.yml`'s pin belongs in A1 (minor) | **REAL.** `pages.yml` :37, :100 `runs-on: ubuntu-latest` (GCC 15.2) and A1 adds a native GCC build there; the module is built `-Werror` by Clang, whose warnings differ from GCC 13's | **FIXED.** The pin in A1's own commit (A.3.5); the module build named as a check for every chunk touching `include/`, `src/robots/` or the host test plant from A2e's merge — the coordinator's to put in those prompts (A.3.3, landmine 28) |
| **W15** | Flags and open questions (minor) | (a) **REAL** — `build_pw2.sh` :15-16 link the one file `-sENVIRONMENT=web`; it ran in a Worker only because a release build skips the check (`settings.js` :687 default `web,webview,worker,node`). (b) **REAL** — `gh repo view`: PUBLIC; the runner readme lists `ubuntu-24.04-arm` (:26) and Windows 11 Arm64 (:38); Playwright 1.64.1's registry ships Chrome-for-Testing `linux-arm64` and Firefox `ubuntu-24.04-arm64` (that ARM runners are free for public repositories is GitHub's documentation, not fetched). (c) **REAL** — the two fetched readmes: Chrome 156 → 157. (d) **REAL in kind** — Rocket Loader, Email Obfuscation and Web Analytics inject or rewrite scripts; the team's server is NOT inspected. (e) **CONFIRMED** — re-run: the ONE FILE over `file://` under A.3.9's policy, the module in a `blob:` Worker: PASS, byte-identical (Firefox) | **FIXED.** (a) `-sENVIRONMENT=web,worker,node` (A.3.3); (b) one ARM row + the NaN rule (A.4, A.2); (c) Playwright's own Chromium in the Windows row as the pinned baseline (A.4); (d) A.3.6; (e) A.10 |
| **W16** | *(found while re-verifying W1)* A MODULE Worker raises Firefox's floor | The site form's Worker is an ES module (proven above in Firefox 157). Module Workers arrived in **Firefox 114** (Chrome 80, Safari 15) — from MDN's compatibility data, NOT run (no older Firefox here) — above the page's stated "Firefox 102" | **FIXED.** The page states **Firefox 115** (the ESR that Windows 7 and 8.1 end on — Windows reach unchanged); `browser_check.js` also feature-tests module Workers before offering PREVIEW (WG4). A.1's P-W3 note, A.3.9 |

**What changed in the sizes.** A1 +~¼ builder-day (the two `.gitattributes`, `region.mjs`, `numbers.mjs` + NF1, the
`pages.yml` pin and post-build check, RG1); A2e +~½ (the native `preview`, WG2s's seeded sweep, WG8, the prefix map,
the environment flag, the ARM and Chromium-baseline rows). In all **~15¾–19¼ builder-days + ~3½ review**. Plus ~1 h of
CI per run (W5) and ~30–45 min per publish (W13) — calendar time, not builder time.

---

## Re-check — amendments (2026-11-13 ~16:10–17:30; the checker's 19 findings, each RE-VERIFIED, not taken on its word)

**How this was checked.** Every citation in the findings was OPENED (main's tree at `a27e2bc` + the index, which now
carries ROB1-A's merge STAGED; worktrees `rob1-int`, `rpv1-a`, `mirror1-a` read only), the checker's own outputs read
(`scratch/brief-RMP1/check/`), and **four new probes run** in `scratch/brief-RMP1/recheck/` (the checker's composed tree
`check/t1` = `a27e2bc` + ROB1-INT's `src/` + RPV1-A's rig and any-robot fakes; the suite's flags with `-Werror`; every
binary under `ulimit -v 4194304` + `timeout`):
- **P5** `p5_units.cpp` — `.maxLinearSpeed = 30_in / 1_s` compiles as a `Velocity` (the library divides quantities,
  `units/quantity.hpp` :127); `units::AngularVelocity{270.0 * math::Angle::kPi / 180.0}` = 4.712 rad/s; and the trap:
  `(270_deg).radians()` = **−1.5708** (an `Angle` wraps, `math/angle.hpp` :27, :49-52) — so a turn cap must never be
  spelled through `_deg`.
- **P6** `p6_homing.cpp` — (1) robot one, `r.startHome(cascade, 2_s)` then a 60 % trigger on the first leg:
  `'cascade': CANCELLED in 0.31s (replaced by a newer command)` → `NOT_HOMED` → `STOPPED at step 2 (driveTo)`; (2) the
  same with `r.waitUntilDone(cascade, 2_s)` after the start: homed in 0.32 s, then the move to `lv(1)` **TIMED_OUT in
  2.01 s** (the placeholder gearing); (3) robot x's switch-homed lift, NO home step: **6/6, ok** — the fakes press its
  switch at start (`rpv1-a/test/fakes_by_kind.hpp` :101-106), so the preview CANNOT catch a missing home on a
  switch-homed mechanism; (4) robot x with `r.home(lift, 3_s)` first: **7/7, ok, end (11.96, −20.23) 88.7°** — the
  corrected D4 route on a fixture whose numbers the test owns (`out`: `recheck/p6_*.txt`).
- **P7** `p7_empty_region.cpp` — the EMPTY route region of the contract with ROB1-BT (§6.3) compiles with `-Werror` and
  runs as a skills auton with no steps (`steps=0 ok=1`, start RELATIVE, park T+57, stop T+59).
- **P8** `p8_tank_turnfirst.cpp` — a 24 in tank `driveTo` from (−36, −55.7) facing 0° to a point at bearing B: **B = 0,
  ±2, ±5, 8° arrive; B = ±10, 14° TIME OUT 0.59–0.81 in short** (inside the 15° steering, no Warn logged); 16–90° time
  out 2.75–9.06 in short with the library's own `tank moveTo not steered: it starts 16.0 deg off its target heading
  (steers within 15.0 deg) -- turn first: face()`; **with `r.face(x, y)` first, 45° and 10° arrive** (0.24 in). Host
  test plant, the suite's 12 in stand-in tank — a fact about the plant, not the robot.

| # | Finding (severity) | Re-verification | Disposition — where fixed (body marked *[Rn]*) |
|---|---|---|---|
| **R1** | D4's sample, GL15 and the usability task make a route the library stops at step 2: NOT_HOMED (blocker) | **REAL.** `rob1-int` `mechanism_table.hpp` :174-190 (`.home = homeByStall(...)`); `service.hpp` :59-62 (zero at bind only by restsAtStop, an absolute sensor, or a switch pressed on a fresh read), :1139-1154 (`if (!homed_) … NotHomed`); a new step REPLACES a running one (`service.hpp` :399-402 `cancelRunningStep(t, "replaced by a newer command")`). The checker's `check/d4r1_nohome.txt` re-read; P6 (1) adds that guide 21's own advice (`rob1-int` guide 21 :224, "`r.startHome(cascade, 2_s)` homes it while the first leg drives") fails the same way when that leg carries a cascade trigger. P6 (3): the preview passes a switch-homed route that never homes. | **FIXED.** New **RC25** (an ERROR): a position move — `move`, `startMove`, a trigger's `startMove` — on an arm or lift homed by STALL or SWITCH needs a FINISHED home before it on every path (`r.home`, or `startHome` then `waitUntilDone` with no other command on it between); a never-homed one cannot move to a level at all; a home whose failure is "carry on" warns on the move after it. One-click fixes "home it first" / "home it while the first leg drives (and wait for it before the move)", the time limit left EMPTY for the student to type (no invented number, §4). D4's sample and D3's JSON gain step 1 `r.home(cascade, 3_s)`; GL15's behaviour moves to robot x (R8); the C++ twin **GL18** = P6 (1)'s NOT_HOMED run; usability card note (§2.5.4). **For the coordinator (not RMP1's file):** guide 21 :224 and `robot_one_autons.cpp`'s banner (`r.startHome(cascade, 2_s); // home the cascade while the first leg drives`) need "…and `r.waitUntilDone(cascade, …)` before the cascade's first move" — ROB1-A's merge is staged now. |
| **R2** | Robot two's mirrored line misreports the MIRROR1 GO (blocker) | **REAL.** MIRROR1 brief :7-9 (the GO: "robot two's starter autons file registers the shared skills route MIRRORED (one line, plumbing …; it lands with ROB1-BT or MIRROR1-A, whichever merges second); the ROUTE itself stays the students'"); NOTES 11-13 the same; `drafts/make_rob1bt.py` :16-18 (ROB1-BT writes it, or the coordinator at MIRROR1's merge). This brief quoted the PRE-GO [R11] recommendation. | **FIXED.** BINDING block, D6 (a), the plain words, §5, landmine 5, §9: the line is plumbing written by ROB1-BT or the coordinator; the page only CHECKS that robot two's autons file carries it (the helper reads the file; `GET /api/status`) and says "robot two runs this route mirrored ✓" or "the line is missing — tell the coordinator", showing the line with Copy ONLY in that fallback. |
| **R3** | Where the export goes conflicts with ROB1-BT's args; the before-ROB1-BT fallback does not work (major) | **REAL.** `make_rob1bt.py` :16 ("between markers the route maker (RMP1) will write" in robot two's file), :33 ("the markers for the route maker in both autons files"), :42; `rob1-int` `robot_one_autons_test.cpp` :162-166 (`count(code, "registry.add(") == 2`); `robot_one_autons.cpp` :24-31 includes no `auton_wall_reset.hpp` / `override_walls.hpp`. | **FIXED.** **New §6.3, THE CONTRACT with ROB1-BT** (one marked region, in `src/team/skills_route.cpp` only; the header list OUTSIDE it; names that resolve to the BUILDING robot's declarations; the EMPTY region's exact text — compiled and run, P7; both autons files register `skillsRoute()`; ROB1-BT re-points robot one's autons pin from 2 registrations to 3). **The coordinator writes it into ROB1-BT's args before either run launches.** Before ROB1-BT merges, the page offers Copy only (the write-to-file button greyed, "the skills-route file arrives with robot two's update"); no marker-pasting by students (§6.1). |
| **R4** | The page's exports turn existing suite tests red (major) | **REAL.** `rob1-int` `robot_one_driver_test.cpp` :161-197 (T11: the default map, Y R1 R2 X A free), :199-226 (GM22: the banner's words AND the exact `preset(cascade[facts::lv(i).name], …)` lines); `test/robot_two_driver_test.cpp` :1071-1110 (MECH-A5: the `[driver file two]` region is the heading plus the fixture's statement, "nothing else", its BEGIN marker pinned as `"// [driver file two]\n"`). They pin the STARTER content, so any change the students make — by hand or by the page — turns them red. | **FIXED.** New **§6.4, what happens to the pin tests when the team takes a file over:** T11 and GM22 are re-pointed from "this map" to the rules they protect (Y free — the build already refuses it, `driver_file.hpp` :81; every level by the ledger's name; the banner's map EQUALS the controls — now generated, below); MECH-A5's region rule kept (robot two's region rewritten by ROB1-BT anyway); the autons pin re-pointed by ROB1-BT (R3). **The header map becomes a generated region** (G3a's `control words` region, its brief :262-273), so GM22's bug (banner and file disagree) cannot happen. **No hash in any source file:** the hand-edit check keeps the hash of what it last wrote in the PROJECT file (also R12). New **HS9 / CT-HS:** every golden export written into a scratch copy of the tree, and the suite run there, green. |
| **R5** | PREVIEW cannot light up page steps: RPV1 numbers steps from below; RPV1-A2 not named (major) | **REAL.** `rpv1-a/test/route_preview_model.hpp` :111-113 (`number` = "the from-below count"); RPV1 brief P8 :370 (a pause and a `then` leave no trace), A2 :395, :535, D13 :236 (after the 21st). This brief's only mention was §0.2 item 10. | **FIXED.** **New D13:** (a) RECOMMENDED — the exporter emits a **step map** beside the scratch file (each page step → its Routine step numbers PER ROBOT, and its expected trace from below: a motion of a kind, a mechanism step in front or behind, a WALL line, or none), and the playback aligns it with RPV1's steps in time order, the RTN stop / skip / jump lines (Routine numbers) and the WALL lines; a page step with no trace (a wait, a label) gets NO span of its own — the list shows it ran or was skipped from the RTN lines; a failed alignment says so and plays the run unnamed, never a guess. Goldens **SM1–SM4** (skip, carry on, the 57 s cut, robot two's numbering with a one-robot block). (b) bring RPV1-A2 (~½ day, a `routine.hpp` change) forward into freeze week — not recommended. §1.1 `stepmap.mjs`; RM22. |
| **R6** | The page draws legs and curves the library will not drive (major) | **REAL, and worse than stated.** `routine.hpp` :482-490, `chassis.hpp` :100-125 (TANK STEERING) read; `curve_path.hpp` :27-43 (checkCurve's refusals), :154 (`at()`, `length()`, `checkCurve()` "are what a tool may rely on"), :181 ("a refused path is still built finite, so a tool can draw it"). **P8:** on the host test plant a tank leg 10° off its bearing already times out — inside the 15° the library steers. | **FIXED.** D2's Drive row: on a tank, **"turn to face it first" is part of the Drive step and ON by default** — it exports `r.face(x, y)` (reverse: `r.faceAway`) before the drive, as the library asks (`routine.hpp` :484-487 "face() is YOUR turn"); OFF only when the planned leg starts within 0.5° of its line, or by the student's choice. **RC24 → an ERROR** above the robot's own `tankSteer.maxSteer` (15°, from the description) with the one-click fix; a WARNING from 0.5° up to it ("may stop short — P8; PREVIEW shows it"). **New RC26** — `checkCurve`'s rules ported to `curve.mjs` (StartBehind, every Invalid case, TightBend as a warning), each with its C++ twin through `checkCurve`; **RC2** now checks the SAMPLED Hermite path, not the polyline; the field draws the library's own curve geometry, proved by **CV1** (a golden table from `CurvePath::at`). Landmine 1 and §1.4's Curve row rewritten. |
| **R7** | Size and dates optimistic; students' exports land at or after the freeze (major) | **REAL.** MIRROR1 :321-323 (3–4 builder-day runs took ~1½–2 calendar days); G3a :1201-1202 (R1 4½–6½ + R2 4–5½ for a narrower route tool); NOTES 11-13: usage stops at 03:24 and 13:03; the slot order "RMP1 brief → … → DOCS3 amendment → docs phase 1"; **and a newer entry: the RMP1 AMENDMENT (Windows — a web app, PREVIEW in WebAssembly) runs right after this re-check and RMP1-A waits for it.** | **FIXED.** D11 re-sized and re-dated (below and §2.1, §6.2, §9): RMP1-A split into **A1** (the route tab, checks, export, save — ~5½–6½ + ~1) and **A2** (PREVIEW in the page — ~2½–3½ + ~½); A1 merged **~Tue 11-17 to Wed 11-18**, A2 **~Thu 11-19 to Fri 11-20**; B after A1's shell files freeze (its file list in §1.2), **~Nov 19–20 or after the 21st**, and it displaces the DOCS3 amendment / docs phase 1 / ROB1-C by ~2 days unless the team lead orders otherwise; C after the 21st. **New D15:** are the students' route and driver regions in `src/team/` open after the ~Nov 18 freeze? (Without it, the tool's output cannot reach the robots before the 21st.) |
| **R8** | Goldens tied to robot one's PLACEHOLDER facts (major) | **REAL.** `check/d4r1_home.txt`; P6 (2) (the cascade times out on the placeholder gearing); `rob1-int` `robot_one_facts.hpp` OWED levels. | **FIXED.** Every golden that ASSERTS behaviour (counts, verdicts, end pose) runs on the fixture robots whose numbers the test owns (`test/mechanism_fixtures.hpp`: robot x's switch-homed lift, robot y's stall-homed arm, … — already RPV1-A's, `rpv1-a/test/route_preview_fixtures.hpp` :11-17); robot one and robot two get a compile-and-run-without-throwing SMOKE, no verdicts asserted. `describe --robot x|y` added. §2.2 GL rewritten. |
| **R9** | RC18 names the wrong refusal for a lift preset (minor) | **REAL.** `kinds.hpp` :517 `no_such_preset_on_this_mechanism` raised for an Arm :776, a Lift :876, a Flywheel speed :992; :519 `no_such_position_on_this_mechanism` for a Pneumatic only :1138; fixture `k_misspelt_lift_preset.cpp`. | **FIXED** (§0.2 item 7, RC18, §1.2): each kind its own name. |
| **R10** | UN1 cannot be met: speed caps have no literal; the turn cap is rad/s (minor) | **REAL.** `units/literals.hpp` (`_in _tile _s _ms _volt _deg _rad` only); `chassis.hpp` :289-291. P5 found the unit-carrying spelling. | **FIXED.** Export spelling: `.maxLinearSpeed = 30_in / 1_s` (the compiler checks the dimension; P5) and `.maxAngularSpeed = units::AngularVelocity{270.0 * math::Angle::kPi / 180.0}  // at most 270 deg/s` — never through `_deg` (P5's wrap). UN1's whitelist names exactly these two forms; goldens **GL19** (each, compiled and run). |
| **R11** | The no-network check trips on the SVG namespace; `build.py` cannot be stdlib (minor) | **REAL.** `tools/docs_figures/field_coordinates.py` :14 `from PIL import Image` (Pillow 10.2.0 here); `http://www.w3.org/2000/svg` is required by `createElementNS`. The robot descriptions baked into the committed page would go stale or redden the gate at every facts edit. | **FIXED.** The field image is made ONCE by a Pillow script beside the docs figures (`tools/docs_figures/route_maker_field.py`) and committed; `build.py` (stdlib) only inlines committed files, so `--check` is Pillow-free. NO robot description in the page: the project file carries the last description the helper read (with its API version and date); double-click mode uses it and says so. No-network: a **Content-Security-Policy** in the page (`connect-src` 127.0.0.1 only; no outside script, style, image or font) plus a test — no URL outside a whitelist of XML namespace URIs, no `fetch` / `src` / `href` / `url()` to any host but 127.0.0.1 (HS8, §7). |
| **R12** | The export is not reproducible from the project file; the trigger ordering rule unstated (minor) | **REAL.** D4's header carried a save time; the JSON sample lacked the trigger's name; `routine.hpp` :1098-1104 (anything but the line itself while armed is a precondition), :748 / :783 (`onFail` and `label` call `requireUnarmed()`), :2019-2026 (`afterTurning`: an angle wraps, so 270° is −90° and refused). | **FIXED.** No time and no hash inside the region (both in the project file); trigger names stored in the JSON; the exporter's order on one leg is fixed: `label` → `onFail` → the arms → the leg (**GL17**: label + `onFail(skipTo)` + a trigger on one leg, compiled and run); RC5 refuses "after more than 180°" as well as 0. |
| **R13** | "Who runs it" needs path-aware checks or robot two's run THROWS (minor) | **REAL.** `routine.hpp` :978-980 (`waitUntilDone` on a mechanism this routine never started is a PRECONDITION — a throw ends the auton), while a start a SKIP jumped answers a counted failure (:970-976). A start inside `if (!competition::isMirrored(ctx)) {…}` never runs on robot two. | **FIXED.** RC15 and RC17 evaluate BOTH robots' step lists (robot one's, and robot two's with the one-robot blocks applied); a skip's label inside a one-robot block is checked for the other robot too. **GL16b**: a one-robot start and a shared wait — the page refuses it, and its C++ twin throws on the mirrored run (with MIRROR1-A). |
| **R14** | Controls and feel storage diverges from G3a's design of record (minor) | **REAL.** G3a :1763 (`controls.whileHeld[]`, `.toggles[]`, `.presets[]` … each with `words`, in the ROBOT file), :256-268 (`driving`: `curve`, `strafe`, `turn`, `rampPerSecond`, `onRelease`), :600-622 (R.4: `onFail` inside `care`). A VS Code webview reaches an extension by `postMessage`, not by `fetch` to 127.0.0.1. | **FIXED, and asked.** RMP1 stores everything in G3a's shapes: routes as R.4 (`onFail` back inside `care`; `mirroredFor` replaces `drawnFor` / `mirror`, MIRROR1), and `controls` / `driving` exactly as G3a's robot-file blocks, kept under `robots.<id>` in `route_maker.json` until G3a's robot file exists — then moved verbatim. **New D14** asks the team lead (or the coordinator) to confirm. The page talks to the helper through ONE module (`api.mjs`), so the panel swaps that one module for `postMessage` — "hosted unchanged" corrected to "the model and screens reused; the transport swapped" (D7). |
| **R15** | Most controls refusals cannot be reached on robot one's or two's real lists (minor) | **REAL.** Both robots will have only the cascade (a Lift) after ROB1-BT; the pneumatic, flywheel, power and never-homed refusals need other kinds. | **FIXED.** The CT agreement runs on FORCED configurations over the fixture robots x, y and z (every kind, every homing method), described by the same describe program; the per-robot list is only what the UI offers. |
| **R16** | The feel graph is wrong in TankSticks; D12 needs a top speed robot one lacks (minor) | **REAL.** `drive_modes.hpp` :9-11, :89-91 (tank sticks become forward = (l + r)/2, yaw = (r − l)/2 BEFORE `shapeRequest`, `input_shaping.hpp` :215-219); `rob1-int` `chassis_table.hpp` :203-210 (nothing about robot one's drive is known); `rpv1-a/test/route_preview_run.hpp` :188-191 (the stand-in 12 in tank). | **FIXED.** The graph's axes are "stick" → "the forward (or turn) REQUEST, −100…100 %", with a line under it in TankSticks: "both sticks are first averaged into a forward and a turn request; the curve shapes those". D12's "about the fastest" uses the route's own speed caps and the robot's motion budget FROM ITS DESCRIPTION; where the robot states none (robot one today) the page shows only "the slowest allowed" and says the fastest needs robot one's calibration record — no stand-in number. |
| **R17** | `kFeel`'s new marked place has no owner (minor) | **REAL.** No markers around THE FEEL in either driver file (`rob1-int` `robot_one_driver.hpp` :62-69; `src/team/robot_two_driver.hpp` :56-63); ROB1-BT's args place markers only in the autons files. | **FIXED.** D9: three regions per driver file, G3a's names — `controls` (the existing `[driver file one/two]`), `control words` (the header map) and `feel` (`kFeel` and `kLoop`'s two team lines). **Their marker lines are placed by ROB1-BT** (its args gain them — the coordinator; the content between them unchanged byte for byte), or, if ROB1-BT has launched without them, by RMP1-B as its one named `src/team/` edit (marker lines only; the src gate re-run). Until then the Controls tab offers Copy only. |
| **R18** | Screenshot acceptance not achievable from the sandbox (minor) | **REAL.** `ps`: BOTH strays still run — `geckodriver --port 4471` **pid 3381105** and a headless `firefox --marionette` **pid 3381195** (~270 MB); `check/ff.log` (a CLI screenshot that exits 0 and writes nothing). No chromium or playwright here. | **FIXED, and asked now.** **New D16:** a capture path the team lead allows (run `probes/p3_shot.py` outside the sandbox, or let the build run it) — asked BEFORE RMP1-A1 launches; §2.5.1 keeps the first-hour proof with "stop and ask" if it fails; both pids named for the coordinator to stop (header item 8). |
| **R19** | Small citation and count slips (minor) | **REAL.** 280 fixtures at `a27e2bc` (289 with ROB1-A's staged merge), not 283; the `refused::` names are DECLARED at `driver_inputs.hpp` :141-206 — **29**, not 30; `fakes_by_kind.hpp` is RPV1-A's (A0 and B1 ran as one run); P1's route differs from D4's listing (`onFail(carryOn)` before the reset, robot two's pivot and `r.set(claw)`); P1b only compiled. | **FIXED** in §0.2, §0.4, §1.1, D4. |

### Re-check — where it stands (plain words for the team lead)

**What changed in the design.** (1) The route maker makes the students **home the cascade before the route moves it**
— the first try at the sample route stopped at step 2 because the cascade had not found its zero. (2) On a tank, every
**Drive step turns to face its point first** (switchable): the library's preview showed a leg only 10° off its line
stopping short. Curves are checked by the library's own curve rules, and drawn as the library's real curve. (3)
**Robot two's one line is not the students' job** — your MIRROR1 go made it plumbing; the page only checks it is there.
(4) The route goes into **one shared skills-route file** whose shape is agreed with the robot-two run BEFORE either
starts. (5) The goldens that check behaviour run on **test robots the suite owns**, so the builder typing robot one's
real numbers this week never breaks them.

**The decisions as I now recommend them:** D1 (a) a small helper + one offline page — *to be revisited by the queued
Windows / web-app amendment* *(amended 2026-11-13 late)* **— revisited: D1 (e), a web app with PREVIEW in WebAssembly; see the Amendment
above, whose A.11 replaces this paragraph where they differ**; D2 (a) the start and ten step kinds, with "turn to face it first" on by default on a tank
and the home rule; D3 (a) one project file, in the panel's own shapes (D14); D4 (a) one marked region in the shared
skills-route file, written by the helper, Copy until robot two's update lands; D5 (a) PREVIEW runs the route on screen
through the real library; D6 (a) robot one's route once, robot two's line checked, not typed; D7 (a) the panel reuses
this app later; D8 (a) plain JavaScript and SVG, one file; D9 (a) three marked regions per driver file (controls,
the button map, the feel), markers placed by the robot-two run; D10 (a) the build refuses an out-of-range feel — only
if RMP1-B lands before the freeze; D11 (a) **four runs: A1, A2, B, C** (below); D12 (a) two arithmetic times plus
the preview's; **D13 (a)** PREVIEW names the steps by matching the run to the route (no library change in freeze
week); **D14 (a)** store controls and feel in the panel's own shapes; **D15 (a)** keep the students' route and driver
regions open after the ~Nov 18 freeze, every change through the full checks and the Auton Tester on the robot;
**D16** please allow a way to take the page's screenshots (run one script outside the sandbox).
*(amended 2026-11-13 late) Since then: D1 → (e) the web app; D4's writer is the page through the connected folder
(A.3.7); D11 → six runs (A.7); D13 → (c), exact from the runner; **D15 ANSWERED (a)** by the team lead; D16 settled
(A.5); new D17–D19 (A.9). A.11 is the current plain-words summary.* *[W5] And new D20 (the pushes that prove
Windows), after the amendment's re-check (A.12).*

**The runs, sizes and honest dates** (measured pace: a 3–4 builder-day run took 1½–2 calendar days; one full-day usage
stop moves every date a day; the Windows amendment runs first): *(amended 2026-11-13 late) — superseded by Amendment
A.7 (six runs, planned on D15 as answered and D19).*

| Run | Size (builder-days) | Lands |
|---|---|---|
| **RMP1-A1** — the app, the field, every step kind with its checks, export into the skills-route file, save / open | ~5½–6½ + ~1 review | merged **~Tue 11-17 to Wed 11-18** |
| **RMP1-A2** — PREVIEW in the page: the real library's run played on the field, the timeline scrubber, both robots | ~2½–3½ + ~½ | merged **~Thu 11-19 to Fri 11-20** |
| **RMP1-B** — the Controls tab and Driving feel | ~2½–3 + ~½ | **~Nov 19–20, or after the 21st** (it pushes the docs amendment and ROB1-C ~2 days) |
| **RMP1-C** — polish from the review and the builder's session | ~1½–2 + ~½ | **after the 21st** |

In all **~12–15 builder-days + ~2½ review** (the first print said 8½–10½; it left out the curve rules, the home
rule, the step matching and the pin tests). **What the students can do before the 21st, if D15 is (a):** from
~Nov 17–18 build and export the skills route by clicking and run it on the robots with the Auton Tester; from ~Nov
19–20 press PREVIEW in the page (until then the preview is RPV1's one command, run by the coordinator); the driver
controls stay robot one's starting map, edited by hand (guide 21), unless RMP1-B lands in time.

**What stays unverified:** a headless screenshot from this laptop (D16); the export compiled inside the real shared
file (it does not exist until ROB1-BT — P7 compiled the region in a stand-in file); the two-robot PREVIEW and the
one-robot blocks (MIRROR1-A unmerged); the checked Toggle step (AF1S-A2 unmerged); the step-matching design (D13 —
argued from RPV1's model and the RTN lines, not yet built); whether the Windows amendment keeps D1 / D5 / D8 as they
stand; the usability task (needs the builder's time); every robot-one time (placeholder facts). *(amended 2026-11-13
late) The Windows question is answered (D1 (e)); what the amendment leaves unverified is A.10.*

---

## DECISIONS FOR THE TEAM LEAD (a recommendation for each; nothing below decides them silently)

**D1 — Where it runs and how it is opened. RECOMMENDED (e).** *(amended 2026-11-13 late)*
- **(e) RECOMMENDED *(amended 2026-11-13 late)*: a WEB APP — static files on the docs site (a "Route builder" tab at
  docs.shurobotics.com/route-builder/), plus a one-file download for offline use; PREVIEW runs the real library
  compiled to WebAssembly in the page.** Windows, Mac, Linux, Chromebooks, nothing installed. Probed (Amendment A.1):
  no library change, ~165–200 KB over the network, a 60 s route in ~0.2 s, the step results equal to the native build.
  Saving and export through the browser (Chrome/Edge write into the shulib folder; elsewhere download and Copy —
  A.3.7). The design: Amendment A.3; the runs: A.7.
- *(a) — superseded by (e): it worked only on a laptop with Python, CMake and a C++ compiler — this Linux laptop, not
  the team's Windows laptops.* **(a) (first recommendation): one command starts a small helper on the laptop and opens the app in the browser:**
  `python3 tools/route_maker.py` (Python's standard library only — no install, no internet). The helper serves ONE
  self-contained page (`tools/route_maker/route_maker.html`: its code, styles, icons and the field picture all inside the
  file) on `http://127.0.0.1:<port>` (this laptop only, never the network), and does the four things a browser page
  cannot: reads the robot's real mechanism list (a small host program built from the robot's own tables), runs PREVIEW
  (builds the current route into RPV1's preview program and runs it on the host test plant — ~5 s here, P4), saves the
  project file, and writes the export into the marked place in the autons file. The same HTML file opened by
  double-click (no helper) still builds, checks, saves (a download) and copies the export; PREVIEW and "write to the
  file" then say, in one banner, that they need the helper. Nothing needs the internet at the event.
  **Who can PREVIEW:** any laptop where `python3 tools/route_preview.py` works today — Python 3, CMake and a host C++
  compiler (RPV1's needs; this laptop has them). Elsewhere the page works without PREVIEW.
- **(b) A page only, no helper:** build and export in the browser; to preview, run `route_preview.py` in a terminal and
  drag its result file onto the page. **Cost:** no PREVIEW button — two windows and a terminal for every look, which is
  what a non-coder cannot do unaided; saving means a download the student must move into `src/team/`.
- **(c) Inside the VS Code panel (G3a).** After the 21st by his ruling (the panel is unmerged, `g3a-p1`). D7 makes RMP1
  the panel's route page later, unchanged.
- **(d) The library compiled to WebAssembly so the page runs it itself.** No toolchain here (no emscripten), an unknown
  size, and a second build of the library to keep equal — weeks, not days. Rejected. *(amended 2026-11-13 late)* *Re-opened and PROBED:
  Emscripten installs in user space; the size is ~420–580 KB (~165–200 KB gzip); the same tree builds both ways with
  no change; a golden test keeps them equal (A.2). It is now (e).*

**D2 — Which step kinds are in the first version (the fewest that write a whole skills route). RECOMMENDED (a).**
*(amended 2026-11-13 late) Unchanged as the first version; it is now built in two runs so the first usable builder
lands sooner (A.7): **A1** — start, drive (turn-first), turn, curve, mechanism (with the home rule), wait until done,
wait, "if it fails" and labels; **A1b** — triggers partway, the wall reset, the Toggle steps (greyed until their
chunks), and "who runs it" with the mirror view.*
- **(a) RECOMMENDED — the start and ten step kinds, each the library's own verb:**

  | In the page | Its options | The library call it exports |
  |---|---|---|
  | **Start** (the robot dragged onto a red-Quadrant tile, a heading handle) | x, y, heading | `e.start = competition::StartPose::field(x, y, h)` (`auton_registry.hpp` :171-173) |
  | **Drive to** (click a point) | forwards / backwards; speed cap, time limit, arrival tolerance; *[R6]* on a tank **"turn to face it first"**, ON by default (OFF only when the planned leg already starts within 0.5° of its line, or by the student's choice) | *[R6]* `r.face(x, y)` (backwards: `r.faceAway(x, y)`) then `r.driveTo(x, y, opts)` / `r.reverseTo(x, y, opts)` (`routine.hpp` :491, :516; :484-487 "face() is YOUR turn") — P8: a leg 10° off its line already stops short on the host test plant |
  | **Turn** | to a heading, or to face a point (or face away) | `r.turnTo(h)` / `r.face(x, y)` / `r.faceAway(x, y)` (:533, :541, :556) |
  | **Curve** (click 2–16 points, the last may state a heading) | forwards / backwards | `r.curveTo({...})` / `r.reverseCurveTo({...})` (:596-617; CURVE1-A, API 2.7) |
  | **Mechanism** (from the robot's real list) | a lift or arm to a level/preset; a pneumatic to a position; a power mechanism for a time; home; each "wait until done" or "start it and keep going"; *[R1]* an arm or lift homed by a stall or a switch must have FINISHED homing before any move (RC25) | `r.move(m["p"], t)` / `r.startMove(…)`, `r.set(m["p"])`, `r.run(m, V, t)` / `r.startRun(…)`, `r.home(m, t)` / `r.startHome(…)` (:848-909) |
  | **Wait until a mechanism is done** | which mechanism, the most extra time | `r.waitUntilDone(m, t)` (:959) |
  | **Trigger partway** (on a drive or a turn) | at N % / after N in / after N° ; starts a mechanism | `r.atPercent(N).startMove(…).driveTo(…)` (:1293; STEP1-B, API 2.8) |
  | **Wall reset** | the wall, the side sensor, the most it may move the robot | `r.then([&]{ return competition::resetFromWall(ctx, {...}).continues(); }, "…")` (`auton_wall_reset.hpp` :8-12) |
  | **Toggle — checked flip** | which Toggle, its visit numbers (Step 0's) | `competition::checkAndFlipToggle(ctx, …)` — **only once AF1S-A2 merges** (correction 2) |
  | **Toggle — blind flip** | the flipper's push and back | two mechanism steps on the flipper — **once robot one's flipper is declared** (correction 2) |
  | **Wait** | seconds | the run guard's pause (correction 6) |

  Plus, on EVERY step: **if it fails** — stop the route / carry on / skip ahead to a named step (`r.onFail(...)`,
  `r.label(...)`, :747, :782; STEP1-A) — and, on a mirrored route, **who runs it** — both robots / robot one only /
  robot two only (`competition::isMirrored(ctx)`, MIRROR1-A, `mirror1-a/.../field_mirror.hpp` :41, :195).
- **(b) Fewer: drive, turn, mechanism, wait** (no curve, no trigger, no wall reset, no Toggle, no "if it fails").
  **Cost:** half the route the students will write is then hand code beside the page's, which is what the page exists
  to avoid.
- **(c) More: strafe (holonomic only), move to a pose, through points, wait for a sensor, flywheel steps, a custom
  action.** **Cost:** ~1–1½ more builder-days; none is needed on a tank skills route with a cascade (both robots are
  tanks, `rob1-int/src/robots/chassis_table.hpp` :210). Later, additive (§5).

**D3 — How a route is saved. RECOMMENDED (a).**
- **(a) RECOMMENDED: ONE project file beside the autons, `src/team/route_maker.json`, committed with the code** — the
  routes, and each robot's controls and feel, in the field names G3a already designed (its R.4 route schema, its D11
  `driving` block, its controls vocabulary), so the panel reads the SAME file after the 21st (D7) *[R14] — its routes;
  the `controls` and `driving` blocks move into G3a's robot file verbatim when that file exists (D14)*. Inches, degrees,
  seconds, volts, percent 0–100 — the units the student sees. A sample:
  ```json
  {
    "schemaVersion": 1,
    "madeWith": "shulib route maker 1.0 (API 2.8)",
    "routes": [ {
      "name": "skills", "kind": "skills", "robot": "one", "mirroredFor": "two",
      "start": {"x": -36, "y": -55.7, "headingDeg": 0},
      "endActionAtS": 57, "hardStopAtS": 59,
      "steps": [
        {"do": "home", "mechanism": "cascade", "boundS": 3},
        {"do": "wallReset", "wall": "red", "sensor": "right", "maxCorrectionIn": 3, "care": {"onFail": "carryOn"}},
        {"do": "driveTo", "x": -12, "y": -55.7, "faceFirst": false,
         "during": [{"name": "raise", "at": {"percent": 60},
                     "then": {"do": "move", "mechanism": "cascade", "preset": 2, "boundS": 2}}]},
        {"do": "curveTo", "points": [{"x": 0, "y": -50}, {"x": 12, "y": -36, "headingDeg": 90}]},
        {"do": "waitUntilDone", "mechanism": "cascade", "boundS": 1},
        {"do": "driveTo", "x": 12, "y": -20, "faceFirst": false,
         "care": {"timeoutS": 2.5, "onFail": {"skipTo": "finish"}}},
        {"label": "finish", "do": "wait", "seconds": 0.5}
      ] } ],
    "robots": { "one": { "controls": { "whileHeld": [ … ], "presets": [ … ], … }, "driving": { … },
                         "seen": { "api": "2.8", "readAt": "…", "description": { … } } } },
    "exports": { "src/team/skills_route.cpp#skills": "sha256:…" }
  }
  ```
  *[R1] step 1 homes the cascade (the first print had none and the library stopped the route at step 2, NOT_HOMED).
  [R12] the trigger's name is stored; no time is written into the code. [R14] `onFail` sits inside `care`, and
  `controls` / `driving` use G3a's robot-file shapes (its brief :1763, :256-268), under `robots.<id>` (D14). [R11]
  `seen` is the last robot description the helper read, so a double-clicked page works from it and says how old it
  is. [R4] `exports` holds the hash of each region the helper last wrote — the hand-edit check, kept OUT of the
  source files. `faceFirst` is false on both drives because each already starts on its line (R6).*
  *(amended 2026-11-13 late)* *`seen` is now the description BAKED into the published page (its API version and commit), not one a
  helper read; saving writes `src/team/route_maker.json` through the connected folder in Chrome/Edge, or downloads it
  elsewhere (A.3.7).*
  Re-opening a saved file shows exactly what was saved (a round-trip test, §2.2 RT3). A file from a NEWER route maker
  is refused by name, never half-read. Unsaved work is kept in the browser between sessions as a draft — a convenience,
  never instead of the file.
- **(b) One file per route** (`src/team/skills.route.json`). **Cost:** controls and feel need another file; the panel's
  design (one route file per robot, one robot file) maps less directly.
- **(c) Only the C++ (the page reads the code back).** **Cost:** a C++ reader for hand-edited code — out of reach and
  fragile.

**D4 — What the export looks like. RECOMMENDED (a).**
- **(a) RECOMMENDED: commented `Routine` code in ONE marked place of the shared skills-route file** — MIRROR1's layout
  (`src/team/skills_route.{hpp,cpp}`, a function returning the entry; built by ROB1-BT, MIRROR1 §6.3) — written by the
  helper between two marker lines that already exist in the file (placed by ROB1-BT's starter *[R3] — THE CONTRACT,
  §6.3: the marker lines, the empty region's exact text, the headers outside it; until ROB1-BT merges the page offers
  Copy only, and no student pastes markers*); the helper never creates a file under `src/` and never touches a line
  outside the markers. *(amended 2026-11-13 late)* *There is no helper: the page writes the region through `files.mjs` in Chrome or Edge
  (the connected shulib folder; the same rules, and the file's own line endings kept — A.3.7); in other browsers Copy,
  or a download of the updated file.* A "Copy" button always works too. **A hand edit inside the markers is detected** *[R4, R12] by
  the hash of what the helper last wrote, kept in the PROJECT file (`exports`), never in the source* and the helper
  refuses to overwrite it without the student saying so, in the page. **Deleting the two marker lines makes the route
  hand-written code** the page then leaves alone. Every step is commented in plain words, numbered as in the page.
  *[R19] The first print said "its shape compiled (P1b) … the same shape … RAN (P1)". In fact P1 ran a SIMILAR route on
  robot two (an `onFail(carryOn)` before the reset, a pivot `startRun`, a claw `set`) and P1b only compiled robot one's.
  The checker RAN this listing verbatim on robot one (`check/d4r1_nohome.txt`): `NOT_HOMED` → `STOPPED at step 2`.
  [R1] So step 1 now homes the cascade. Re-check P6 (4) ran this corrected shape on robot x's fixture lift (`lift["low"]`
  for the cascade's level): 7/7, ok, end (11.96, −20.23) 88.7°. On robot one's PLACEHOLDER gearing the move still times
  out (P6 (2)) — a fact the builder's numbers will change, not a page error.* The sample:
  ```cpp
  // ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
  // Made in the route maker from src/team/route_maker.json. Change the route there and export again:
  // the route maker will not overwrite a hand edit between BEGIN and END.
  // To make this hand-written code instead, delete the BEGIN and END lines.
  namespace {
  // The skills route, in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
  // the field's diagonal (robot two's autons file says so in one line).
  chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
      chassis::Routine r{ctx, "skills"};

      // Step 1 — HOME the cascade: it pushes gently down until it stalls on its bottom stop, which
      //   becomes its zero. At most 3 s. Nothing can move it to a level before this.
      r.home(cascade, 3_s);

      // Step 2 — WALL RESET against the RED wall, read by the robot's RIGHT side sensor.
      //   It may move the robot's y by at most 3 in. If it is refused, the route carries on.
      r.then([&] { return competition::resetFromWall(ctx,
                       {.wall = localization::override2026::kRedWall,
                        .sensor = localization::WallSide::Right,
                        .maxCorrection = 3_in}).continues(); }, "red wall y");

      // Step 3 — DRIVE to (-12, -55.7) in: 24 in straight ahead.
      //   At 60 % of the way, START the cascade toward its third level ("L2"); it has 2 s.
      r.atPercent(60).startMove(cascade[facts::lv(2).name], 2_s, "raise")
          .driveTo(-12_in, -55.7_in);

      // Step 4 — CURVE through (0, -50) in, ending at (12, -36) in facing 90 deg (toward blue).
      r.curveTo({{0_in, -50_in}, {12_in, -36_in, 90_deg}});

      // Step 5 — WAIT until the cascade's move from step 3 is done: at most 1 s more.
      r.waitUntilDone(cascade, 1_s);

      // Step 6 — DRIVE to (12, -20) in, at most 2.5 s. If it fails, skip to "finish".
      r.onFail(chassis::skipTo("finish"));
      r.driveTo(12_in, -20_in, {.timeout = 2.5_s});

      // Step 7 — "finish": WAIT 0.5 s. (The run's own wait: at 57 s the run cuts it at once.)
      r.label("finish");
      r.then([&] { return ctx.guard.pause(0.5_s) == sequence::GuardedWaitResult::Satisfied; },
             "wait 0.5 s");

      return r.result();
  }
  }  // namespace

  // The entry both robots register: robot one's start, the 60 s skills schedule.
  competition::AutonEntry skillsRoute() {
      competition::AutonEntry e{};
      e.name = "skills";                                  // what the brain's menu shows
      e.kind = competition::AutonKind::Skills;            // the 60 s skills run
      e.routine = &skillsSteps;                           // the steps above
      e.start = competition::StartPose::field(-36_in, -55.7_in, 0_deg);  // robot one: red wall's Quadrant
      e.endActionAt = 57_s;  // still running at 57 s? it is stopped, and the end action (a brake) starts
      e.hardStopAt = 59_s;   // everything stops, 1 s before the run's 60 s
      return e;
  }
  // ═══ ROUTE MAKER — route "skills" — END ═════════════════════════════════════════════════
  ```
  A level is named through the facts ledger (`facts::lv(2).name`, as robot one's own driver file does,
  `rob1-int/src/team/robot_one_driver.hpp` :52-58), so renaming a level in the ledger never breaks a route; the comment
  carries the name the page showed. Every number carries its unit (`_in`, `_deg`, `_s`, `_volt`); a percent is a plain
  number 0–100, as the library takes it. *[R10] A speed cap has no literal of its own: it exports as `.maxLinearSpeed =
  30_in / 1_s` (the library divides quantities; the compiler checks the result is a speed — P5), and a turn cap as
  `.maxAngularSpeed = units::AngularVelocity{270.0 * math::Angle::kPi / 180.0}  // at most 270 deg/s` — NEVER through
  `_deg`, which wraps (P5: `(270_deg).radians()` is −1.5708).* *[R12] On one leg the exporter writes, in this order:
  `label`, `onFail`, the triggers' arms, the leg — the only order STEP1-B accepts (`routine.hpp` :1098-1104, :748,
  :783).* A step the page marks "robot one only" exports as
  `if (!competition::isMirrored(ctx)) { … }` with a comment (MIRROR1-A's form; compiled once MIRROR1-A merges).
- **(b) Copy only** (the student pastes it). **Cost:** a non-coder pasting into the right place of a C++ file, every
  time the route changes — the most common way to break a build.
- **(c) A separate generated file the student never opens.** **Cost:** the students stop reading their own route
  (G4: they must be able to explain every line); the build needs a new file in its lists.

**D5 — How it pairs with the preview. RECOMMENDED (a), amended.** *(amended 2026-11-13 late)* *The route on screen is now run by the
RUNNER inside the WebAssembly module (Amendment A.2, A.3.10) — the same library calls the export writes, ~0.2 s for a
60 s route, no build step, no helper, no scratch file; the record keeps RPV1's content (its Model, as JSON). The text
below describes the helper's way and stands only for what the page DOES with the record.*
- **(a) RECOMMENDED: PREVIEW runs THE ROUTE ON SCREEN — not the file on disk — through RPV1's engine, and the page plays
  what the library did.** The helper writes the current export to a scratch file under `build/route_maker/`, builds it
  into a preview program next to RPV1's (`route_preview_main.cpp` + that one file; P4: 4.4 s to build, ~0 s to run),
  runs it under the suite's bounds and hands back RPV1's record as JSON (a small addition to RPV1: `--json`, §1.1 — *[R7] in RMP1-A2*): the
  robot's true pose on every 10 ms pace, every step's start and end time and verdict, each failed / skipped / cut step
  with the library's own sentence, every RTN/MECH/WALL/COMP line, the field check's THROUGH and TOUCH notes, the total.
  The page then animates the robot along that record on the same field, under a **timeline scrubber** (play / pause,
  drag, each step's span marked and named, the 57 s cut and the 60 s line), the step list highlighting the step playing.
  *[R5] RPV1 numbers steps FROM BELOW (motions and mechanism steps only); how the page names them is D13 — a step map
  from the exporter, aligned with the run; a wait or a label has no span of its own.*
  The path the student DREW stays visible as a dashed "planned" line; the path the library DROVE is the solid one — the
  page never animates its own guess as if it were the robot (§8.1). RPV1's own page stays one click away ("open the
  full report").
- **(b) PREVIEW of the saved file only** (export first). **Cost:** a student cannot try an idea without changing their
  code; every look is an export.
- **(c) The page animates its own plan** (no library). **Cost:** a page that draws what the library will not do — the
  landmine this whole chunk exists to avoid. Rejected.

**D6 — How a mirrored route is exported (with MIRROR1). RECOMMENDED (a) — MIRROR1's own D10 (a).**
- **(a) RECOMMENDED: robot one's route ONCE (D4's region) plus robot two's ONE line — never a reflected copy of the
  numbers.** `registry.add(competition::mirrored(skillsRoute(), competition::override2026::kSkillsMirror));` — *[R2]
  the first print had the students type or paste it, with a Copy button. The GO (MIRROR1 brief :7-9) made it PLUMBING:
  ROB1-BT writes it, or the coordinator at MIRROR1-A's merge. So the page **only CHECKS** robot two's autons file (the
  helper reads it; `GET /api/status` — *(amended 2026-11-13 late)* *now `files.mjs` reads it through the connected folder; in a browser
  without folder access the page says "not checked in this browser"*) and shows "robot two runs this route mirrored ✓" — or "the line that runs it
  mirrored is missing from robot two's autons file: tell the coordinator", with the line and Copy ONLY in that case.
  The helper never writes robot two's file.* While the student edits, the page draws
  robot two's mirrored ghost with MIRROR1's formula, checked line by line against MIRROR1-A's shared table
  (`mirror1-a/test/goldens/mirror/skills_mirror_vectors.txt`, "read … by the route maker (RMP1) by splitting each line
  on whitespace", its header :8-10), shades the 6 in band along the diagonal, marks every step whose robot comes within
  3 in of the diagonal (MIRROR1 D11's rule), offers "robot one only" on it, and warns where a mirrored step lands at a
  bare wall instead of a Loader or in the BLUE Load Zone (MIRROR1 D4's "cannot carry" list). PREVIEW of a mirrored route
  runs BOTH robots — robot one's entry and the mirrored one — and MIRROR1-A's pair check (`mirror1-a/test/
  mirrored_pair_check.hpp` :64 `kMirrorGapIn = 6.0`, :272 `comparePair`), and plays both robots together, the first
  moment their squares come within 6 in marked on the timeline in red with the check's own sentence. *[R13] Every
  path check (RC15, RC17, RC25) runs on BOTH robots' step lists — robot one's, and robot two's with the "robot one
  only" / "robot two only" blocks applied — because a `waitUntilDone` on a mechanism the run never started THROWS
  (`routine.hpp` :978-980) and ends robot two's auton.*
- **(b) The page writes a reflected second route.** **Cost:** two copies drift at the first edit (MIRROR1 D10 (b)); only
  the fallback if MIRROR1-A misses the freeze (MIRROR1 D9 (b)).
- **If MIRROR1-A has not merged when RMP1-A is built:** the mirror option ships greyed ("arrives with MIRROR1"), the
  ghost and the band still drawn from the shared table (a pure drawing aid), and the line and the two-robot PREVIEW land
  in RMP1-C.

**D7 — What is later, and why nothing is redone. RECOMMENDED (a).** *(amended 2026-11-13 late)* *The panel after the 21st may be a VS
Code extension, a desktop app, or this web app plus a small local helper (the team lead's words; decided then). The
app reaches files and the engine only through four host interfaces (`files`, `engine`, `robots`, `capabilities` —
Amendment A.6), so any of those hosts swaps those modules, not the screens; `api.mjs` is replaced by them.*
- **(a) RECOMMENDED: the panel's route page (G3a R2) BECOMES this app after the 21st** — the same HTML and JavaScript
  hosted in the panel's VS Code webview, reading the same `src/team/route_maker.json` *[R14] — "unchanged" was too
  strong: a webview reaches its extension by `postMessage`, not by `fetch` to 127.0.0.1, so the ONE module that talks
  to the helper (`api.mjs`) is swapped for the panel's; the model and the screens are reused; the controls and feel
  blocks move to G3a's robot file (D14)* — and G3a's planned Python
  route generator (R1's `tools/robot_routes.py`) is replaced by RMP1's JavaScript exporter (one generator, already
  golden-tested). G3a's R1 + R2 (~8½–12 builder-days in its plan, G3a brief :1201-1202) shrink to hosting, its robot-file
  hooks and the cut items below. What RMP1 CUTS from G3a's route design, each additive later: the F8 / VexBuilder export
  (R.10; D13 (c)'s second half); RELATIVE starts and MATCH (30 s) routes; strafe, move-to-a-pose, through-points,
  sensor waits and flywheel steps (D2 (c)); the per-robot `[routes]` region inside `registerAutons()` (R.11 — replaced by
  MIRROR1's shared skills-route file); the robot file's footprint field (RMP1 uses RPV1's 15 in square,
  `rpv1-a/test/route_preview_model.hpp` :48); the head-to-head red/blue pair (G3a D17 — after the 21st with ALLY1;
  MIRROR1 §6.6 corrects its axis). What RMP1 KEEPS from G3a: R.4's schema (adapted), R.5's verb table, R.6's care and
  D14's three failure choices, R.7 / D15's trigger rules as STEP1-B built them, R.9 / D18's arithmetic time bounds, R.12's
  test ideas (S37–S47, S53) and mutations (M36–M58) where they still apply (§2).
- **(b) A separate panel route page built after the 21st.** **Cost:** the same page twice.

**D8 — How the app is built (the UI approach). RECOMMENDED (a).**
- **(a) RECOMMENDED: plain modern JavaScript and SVG, no framework, no package downloads, bundled into ONE HTML file.**
  *(amended 2026-11-13 late)* *Kept, with three changes: the one file is BUILT in CI and on publish (never committed — A.3.3), it
  carries the WebAssembly preview inside it (base64), and the site form is the same app as a folder (ES modules + the
  `.wasm`). The "CI checks the committed file is fresh" gate goes with the commit.*
  The model (routes, checks, export, mirror, time bounds, controls, feel, the project file, undo/redo) is pure ES
  modules with no screen code — tested headless with `node --test` (Node 20 is on this laptop and on CI's runner); the
  screen is plain DOM + SVG over a small store (one state object, render functions, `requestAnimationFrame` for
  playback); one stylesheet of design tokens (type scale, spacing, radius, colours, light and dark, motion). A ~60-line
  standard-library Python script inlines everything (the modules, the CSS, the icons, the field picture as a compressed
  image) into `route_maker.html`; CI checks the committed file is fresh. Why: offline by construction, nothing to
  install at the event, no third-party code to vet or vendor, every line readable by a reviewer, and the model
  testable without a browser.
- **(b) A small vendored library (Preact + htm, ~10 KB).** **Cost:** vendored third-party code under the project's
  review rules, for a gain plain DOM gives at this size (~3–4k lines).
- **(c) A compiled framework (Svelte, React + a bundler).** **Cost:** `npm install` and a build toolchain the team's
  laptops do not have; a supply chain to audit. Rejected.

**D9 — Where the Controls tab writes. RECOMMENDED (a).** *[R4, R17] rewritten.*
- **(a) RECOMMENDED: three marked regions per driver file — G3a's own three (its brief :258-273, :1763):**
  `controls` = the EXISTING `// [driver file one]` … `// [/driver file one]` (`rob1-int/src/team/robot_one_driver.hpp`
  :50, :60) and `[driver file two]` (`src/team/robot_two_driver.hpp` :48-54) — the WHICH BUTTON heading and the
  `kControls = controls(…)` statement, one commented line per control, nothing else (MECH-A5's rule); `control words`
  = the header's stick-and-button map (robot one's :20-37), GENERATED from the same choices, so the banner can never
  disagree with the file (GM22's bug); `feel` = THE FEEL's comment, `kFeel` and `kLoop`'s two team lines (re-zero
  button, what the drive does on release; never `.tickPeriod` or `.protection`). **Who places the new markers:**
  ROB1-BT, in both robots' driver files (marker lines only, the content between them unchanged byte for byte — the
  coordinator adds this to its args with §6.3's contract); if ROB1-BT has launched without them, RMP1-B places them
  as its one named `src/team/` edit, the src gate re-run. Each marker line holds ONLY its marker (MECH-A5 pins
  `"// [driver file two]\n"`). The hand-edit check is D4's (the hash in the project file). *(amended 2026-11-13 late)* *Written
  through `files.mjs`, as D4.* **The pin tests that
  freeze today's map are re-pointed (§6.4).** Until the markers exist, Copy only.
- **(b) Copy only.** As D4 (b).

**D10 — The build refusing an out-of-range feel. RECOMMENDED (a).**
- **(a) RECOMMENDED: add `static_assert(Feel.valid(), "...")` to `DriverFile` (`src/common/driver_file.hpp`, beside its
  two at :81-84) with its compile-refusal fixture — in RMP1-B, only if RMP1-B lands BEFORE the ~Nov 18 freeze**; else
  after the 21st. Then a hand-typed `kFeel{.forward = {.expo = 1.5}}` stops the build by name, as the page stops it. G3a
  planned exactly this line (Part 5, G3a brief :281-282). One `src/` line; every robot build is re-run by the src gate.
- **(b) The page refuses; the build stays as it is.** **Cost:** a hand edit can still ship a curve that overshoots full
  speed (`input_shaping.hpp` :127-129: outside [0, 1] the promise is void, silently).

**D11 — The runs, their sizes, and what lands for robot week. RECOMMENDED (a′′) — Amendment A.7.** *(amended 2026-11-13 late)* *A1
(cut to the first usable step kinds, on the site, CI on Windows and Linux), A2e (the WebAssembly engine, ~~beside A1 if a
slot frees~~ *[W4] in slot 3 from AF1S-A2's merge, ~Tue 11-17*), A2u (PREVIEW in the page, *started at A1's merge
against a stand-in engine, ~Thu 11-19 to Fri 11-20*), A1b (triggers, wall resets, the mirror view, the time bar), B, C —
~15–18½ + ~3½ *[W] (~15¾–19¼ + ~3½ after A.12)*. The (a′) plan below is superseded where A.7 differs.*
- *[R7]* **(a′) RECOMMENDED (re-check): FOUR runs, each its own go — RMP1-A split so the route building lands first.**
  - **RMP1-A1 — the app and the ROUTE tab, without PREVIEW (~5½–6½ builder-days + ~1 review/merge).** The shell and
    design system (~1); the field — the real picture, pan, zoom, snapping (~1); every D2 step kind with its inspector
    and inline checks, including RC25's home rule, R6's turn-first and the ported curve rules, and both robots' path
    checks (~2); the export, its goldens on the fixture robots and the C++ golden test (~1¼); the helper's describe,
    save and export (with §6.3's contract) (~¾); guide 22's first half (~½). Merged **~Tue 11-17 to Wed 11-18** from a
    launch on Sat 11-14 (measured pace, §6.2).
  - **RMP1-A2 — PREVIEW in the page (~2½–3½ + ~½).** RPV1's `--json` and the "maker" preview row (~¾), the step map
    and its alignment with goldens (D13, ~¾), the playback and timeline scrubber (~1), the two-robot PREVIEW when
    MIRROR1-A has merged (~½), its docs (~¼). After RPV1-A merges and after A1 (it shares A1's screen files). Merged
    **~Thu 11-19 to Fri 11-20.** Until then a route's preview is RPV1's one command, run by the coordinator.
  - **RMP1-B — Controls and Driving feel (~2½–3 + ~½)**, after A1's shell files are merged (it may touch only §1.2's
    list plus `app.mjs`'s tab entry). **~Nov 19–20 or after the 21st**; in a slot it pushes the DOCS3 amendment, docs
    phase 1 and ROB1-C back ~2 days — the team lead's call.
  - **RMP1-C — polish (~1½–2 + ~½): after the 21st.**
  - In all **~12–15 builder-days + ~2½ review.** The first print's 8½–10½ left out the curve rules, the home rule,
    the step matching, the pin tests and the contract work; G3a sized a narrower route tool at 8½–12 (its brief
    :1201-1202).
- *Superseded by (a′) — the first print's plan:*
- **(a) three runs, each its own go:**
  - **RMP1-A — the app and the ROUTE tab with PREVIEW (~5–6 builder-days + ~1 review/merge).** The shell, the design
    system, the field (the real picture, pan, zoom, snapping), every D2 step kind with its inline checks, undo/redo and
    shortcuts, the time bounds, save/open, the export and its goldens, the helper, RPV1's `--json` and the PREVIEW
    playback with the scrubber, the mirror ghost (and the two-robot PREVIEW if MIRROR1-A has merged), first-run help,
    guide 22. **Launch: the first free slot after your go** — its page-and-model half needs nothing unmerged; its PREVIEW
    half rebases onto RPV1-A's merge and ROB1-INT's (robot one's list).
  - **RMP1-B — the CONTROLS tab and DRIVING FEEL (~2–2½ + ~½).** Beside RMP1-A from the day a second slot frees, in its
    own files against the design tokens §1.1 fixes; merges after A.
  - **RMP1-C — polish from the UX review and the usability session (~1½–2 + ~½).** The review lens's findings, the
    builder's session's findings, motion and accessibility passes, every main screen's screenshots for the docs, and
    whatever of the mirror (D6) or the checked Toggle step (AF1S-A2) landed late.
  - **Honest dates (§6.2):** a go on Fri 11-13 → RMP1-A launched Sat 11-14 → **merged ~Mon 11-16 to Tue 11-17** (the
    team's robot week: Nov 17–20); RMP1-B merged ~Nov 18–19; RMP1-C ~Nov 19–20 or after the 21st. RMP1 changes no robot
    program, so the ~Nov 18 freeze does not stop it (D10's line excepted). One full-day usage stop moves each date a day.
    *[R7] These dates were optimistic (measured pace puts A's merge ~Nov 17–18), and "the freeze does not stop it"
    misled: the TOOL may merge late, but what it WRITES is robot code in `src/team/` — D15 asks whether that stays
    open.*
- **(b) RMP1-A only before the 21st;** B and C after. **Cost:** controls and feel stay hand-typed for the event (robot
  one's starting map exists, `robot_one_driver.hpp` :52-58).
- **(c) One big run.** **Cost:** ~9–10 builder-days in one go — no usable landing before ~Nov 19, and one review of
  everything at once.

**D12 — The time shown against 60 s. RECOMMENDED (a) — G3a D18 (a), plus the preview's time.**
- **(a) RECOMMENDED: while editing, two arithmetic numbers** — "about the fastest" (each leg's length over its speed cap
  or the robot's top speed, each turn's angle over its turn cap, every wait and timed run *[R16] — the top speed and
  turn rate are the robot's motion budget AS ITS DESCRIPTION STATES IT; a robot that states none (robot one today: no
  geometry, `rob1-int` `chassis_table.hpp` :203-210) gets no "fastest" — the page shows only the slowest and says
  "the fastest needs robot one's calibration record (shulib Calibrate)"; never the preview's stand-in drive's
  numbers*) and "the slowest allowed" (the
  sum of every step's own time limit, capped at the 57 s cut) — with their words, never one predicted time; **after
  PREVIEW, the host test plant's time** from the library's run, labelled as RPV1 labels it ("IDEAL host test plant, not
  the robot … time it on the robot", P4's output). The 57 s cut and the 60 s line are drawn on the bar; a route whose
  fastest passes 57 s is marked red, one whose slowest does amber.
- **(b) One "likely" time** from an assumed acceleration. **Cost:** a second, unchecked model of the robot — what he
  ruled out on 2026-10-08 (G3a R.9) — that will disagree with the preview's.

**D13 — How PREVIEW names the steps it plays. RECOMMENDED (c).** *(amended 2026-11-13 late)*
- **(c) RECOMMENDED *(amended 2026-11-13 late)*: exact, from the runner.** The runner executes the page's steps one at a time inside the
  WebAssembly module, so it records each page step's first and last pace itself — a wait and a label included — and a
  trigger's mechanism by its MECH lines as RPV1 does. No aligner, no step map, no library change (Amendment A.3.10).
  Goldens SM1–SM4 stay, as the runner's spans. *(a) and (b) below are superseded.*

*[R5] new:*
- **(a) RECOMMENDED: a step map from the exporter, matched to the run — no library change.** RPV1's record numbers
  only what it sees from below (motions and mechanism steps; `rpv1-a/test/route_preview_model.hpp` :111-113), and a
  pause or a `then` leaves no trace (RPV1 brief P8 :370). The exporter already knows what each page step becomes, so
  it writes a step map beside the scratch file: for each page step, its Routine step numbers ON EACH ROBOT (a "Drive
  to" with turn-first is two; a blind flip two; a one-robot block none on the other robot) and its expected trace
  (a motion of a kind; a mechanism step in front, or behind the drive for a trigger; a WALL line; or none). The page
  walks the run in time order and matches: RPV1's steps, the RTN lines (they carry the Routine's own numbers for a
  stop, a skip and a jump) and the WALL lines. A step with a trace gets its span; a wait or a label gets NO span of
  its own (the list shows "ran" or "skipped" from the RTN lines); a failed step is red from its RTN line. If the
  match fails anywhere, the page says so and plays the run without step names — never a guess. Goldens SM1–SM4 (a
  skip, a carry-on, the 57 s cut, robot two's numbering with a one-robot block).
- **(b) Bring RPV1-A2 forward** (the library's step listener, exact numbers, ~½ builder-day; RPV1 brief :395, :535):
  a `routine.hpp` change in freeze week that every robot program recompiles. **Cost:** a library edit for a picture;
  RPV1's D13 put it after the 21st. Later, A2 replaces the matching with exact numbers.

**D14 — Where controls and feel are stored. RECOMMENDED (a). *[R14] new — the team lead's or the coordinator's call.***
- **(a) RECOMMENDED: in G3a's own shapes** — the route as G3a's R.4 (`onFail` inside `care`; `mirroredFor` in place
  of `drawnFor` / `mirror`, MIRROR1), and `controls` / `driving` exactly as G3a's robot-file blocks (its brief :1763,
  :256-268), kept under `robots.<id>` in `route_maker.json` until G3a's robot file exists — then moved there
  verbatim. G3a inherits at its re-plan: R1/R2 onto RMP1's model, Part 5's `controls` / `driving` emitter replaced
  by RMP1's, Part 6's FEEL page onto RMP1-B's, `api.mjs` swapped for `postMessage`. *(amended 2026-11-13 late) There
  is no `api.mjs`: a panel swaps the host interfaces of Amendment A.6 (`files`, `engine`, `robots`, `capabilities`);
  the storage shapes this decision is about are unchanged.*
- **(b) RMP1's own shapes** (the first print: `onFail` beside `care`, a flat `controls` list). **Cost:** G3a redesigns
  its robot file or translates forever.

**D15 — The freeze and the students' files. ANSWERED (a) — the team lead, 2026-11-13 (late): "(a) Keep routes
open".** *(amended 2026-11-13 late)* *As given to this writer: after the ~Nov 18 freeze ONLY the students' skills-route
region and the driver files' controls/feel regions may change, each change through the full checks plus an Auton
Tester run on the robot; the library and the robot setup freeze on the 18th. Two consequences, written down here:
(1) the driver files' `control words` region (the header's button map, GENERATED from the controls, D9) must change
WITH the controls region — read as part of it — or a controls change after the 18th leaves the banner disagreeing
with the code (GM22's bug); (2) whether the route BUILDER itself (not robot code) is inside the freeze is new D19
(Amendment A.9). The dates are planned on this answer (A.7). The options as they were put:*
*[R7] new:*
- **(a) RECOMMENDED: the ~Nov 18 freeze covers the library, the programs and the tools; the students' route region
  (`src/team/skills_route.cpp`) and their driver regions stay OPEN until the robots are packed (~Nov 20 evening),**
  every change through the full gates (the suite, the src gate's builds) and the Auton Tester on the robot. Without
  this the route maker's output cannot reach the robots before the 21st: RMP1-A1 merges ~Nov 17–18.
- **(b) Everything freezes ~Nov 18.** **Cost:** the skills route is whatever is typed by then — by hand, since the
  route maker lands at the freeze.

**D16 — A way to take the page's screenshots. SETTLED by the coordinator; the recipe PROVEN.** *(amended 2026-11-13 late)* *Playwright
1.64.1 pinned, its OWN Firefox under `~/shulib-render-tmp/` (no snap, nothing hidden, nothing in /tmp), under the
4 GiB cap — Amendment A.5. Chromium-based screenshots from CI (D18). The text below is the first ask, superseded.*

*[R18] new — please answer before RMP1-A1 launches:*
From this agent's sandbox the snap Firefox writes no screenshot (P3, and the checker's try), and nothing else here
renders a page. The screenshots are an acceptance item (§2.5). **Recommended:** let the coordinator run
`probes/p3_shot.py` (a WebDriver session through the snap's own geckodriver) OUTSIDE the sandbox — you run one command,
or allow it once. Two stray processes from P3 are still running and need stopping: geckodriver **pid 3381105** and a
headless Firefox **pid 3381195**. *(amended 2026-11-13 late) Both ended with the laptop's restart at ~23:28.*

---

## The design in plain words (for the team lead)

*(amended 2026-11-13 late)* A student opens **docs.shurobotics.com** in Edge or Chrome on any laptop — Windows, a
Mac, Linux or a Chromebook, with nothing to install — and clicks the **Route builder** tab (at the event, with no
internet, they double-click the one-file copy downloaded beforehand). The page opens on **the real skills
field, seen from behind the red station** — the same picture as guide 2. They drag robot one onto its start tile and
turn its arrow to the way it faces. Then they build the route the way they would explain it: click **Drive** and click a
spot (on a tank the robot turns to face the spot first — the step says so, and it can be switched off); click
**Curve** and click two or three spots; pick **Mechanism → cascade → third level** (the page asks them to home the
cascade first if they have not — one click adds the home step, they type how long it may take); tick "start it 60 %
into the drive"; add a **Wall reset** against the red wall with the right sensor. Each step appears in a list on the left,
numbered, and on the field as a line and a ghost of the robot where it ends. Anything the library would refuse is
underlined in red **as they type**, in the library's own words and plain English beside them — a point the robot's
square cannot reach without going through a wall; a trigger at 120 %; a wall reset where the robot's side sensor points
45° away from the wall ("turn to 0° first?", one click inserts the turn). A bar along the bottom shows about the fastest
and the slowest the route can take against the 57 s cut and the 60 s line.

They press **PREVIEW**. *(amended 2026-11-13 late)* In well under a second the robot plays the route on the same
field — the real library, the same code that will run on the brain, compiled to run inside the web page and driven on
the host test plant — with a timeline under it: play, pause, drag to any
moment; each step's span is marked, the step playing lights up in the list, a step that failed turns red with the
library's sentence. If the route is mirrored for robot two, both robots play together, robot two in the right wall's
Quadrant, and the first moment they come within 6 in of each other is marked in red.

They press **Export**. *(amended 2026-11-13 late)* In Edge or Chrome, once they have connected the shulib folder, the
route is written as plain, commented C++ into the marked place in the skills-route file, keeping the file's own
Windows or Linux line endings; in another browser they copy it, or download the updated file to put back — the
same lines a student would type, every step explained in a comment. Robot two already runs that same route mirrored
(one line in its own file, put there by the robot-two update); the page checks the line is there and says so. The
route is saved in `src/team/route_maker.json` beside the code (downloaded, in a browser that cannot write to a
folder), so anyone can re-open it and carry on.

The **Controls** tab shows the V5 controller. Click L1: pick "cascade", then "move up while held"; L2 becomes "down" with
it. Click d-pad Up: "cascade → fourth level". Y shows "the library keeps Y for re-zeroing — pick another button". Pick a
button that is already used and the page says which job it already has. The **Driving feel** section has a slider per
stick and a live graph: stick position along the bottom, the speed the driver asks for up the side. Slide "curve" up and the line sags —
a small push gives a slow, controllable speed, a full push still gives full speed, which is exactly what the drivers
asked for. Export writes the driver file's lines — the buttons, the map of buttons at the top of the file, and the
feel.

What the page will not do: invent a route, guess what the robot will do (only the library's run is drawn solid), or
let a student export something the library would refuse.

*(amended 2026-11-13 late)* After the 21st the same pages can sit inside a tool that also builds the robot program,
uploads it to the brain and reads its log — a VS Code panel, a desktop app, or this web page with a small helper on
the laptop, decided then. Nothing in the builder has to be rewritten for that (Amendment A.6).

---

## 0. Why — what exists, what is reused, and the probes

### 0.1 Why this chunk, and why now
Robot one's skills route is still the comment "THE SKILLS ROUTE GOES HERE (the students')"
(`rob1-int/src/team/robot_one_autons.cpp` :50), and the code freeze is ~Nov 18. Every library verb a skills route needs
is on main (§0.2); what is missing is a way for the team's non-coders to turn their plan into those verbs, see what the
library does with it, and change it at the field without a programmer. The preview exists (RPV1-A, finishing) but runs
code that is already written. The panel that was to do this (G3a R1/R2) is after the 21st. This chunk is the route
maker the team lead asked for, built so the panel can host it unchanged later (D7). *(amended 2026-11-13 late) And it
must work on the team's Windows laptops — the team lead's bar — so it is a web page on the docs site with the library
compiled into it (Amendment).*

### 0.2 What exists in the tree (opened at `a27e2bc` unless a worktree is named)
1. **The route verbs** — `chassis/routine.hpp`: `startAt` :468, `moveTo` :478, `driveTo` :491, `reverseTo` :516,
   `strafeTo` :527, `turnTo` :533, `face` :541, `faceAway` :556, `curveTo` / `reverseCurveTo` :596-617 (CURVE1-A, up to
   16 points, `{x, y}` or `{x, y, heading}` — `motion/curve_path.hpp` :99-110), `brake` :621, `hold` :627, `pause`
   :639-646 (a frozen `Chassis::wait`), `waitFor` :655, `then` :699, `onFail` :747 (two in a row and an empty skip target
   are preconditions, :748-756), `label` :782 (two in a row, an empty name: preconditions, :783-788), the mechanism steps
   :848-959 (`move`, `startMove`, `home`, `startHome`, `run`, `startRun`, `set` — no start form —, `spinUp`, `stop`,
   `waitUntilDone`), the STEP1-B triggers :1119-1312 (`atPercent` / `afterDistance` / `afterTurning`, the four start forms,
   at most 4 per leg, one per mechanism, drive and turn legs only), and the trigger ranges' sentences at :2003-2027 —
   e.g. `"r.atPercent(...): the percent must be above 0 and at most 100 (\"at 0 %\" is a start step written BEFORE the
   drive: r.startMove(...).driveTo(...))"`. `OnFail`: `stopChain`, `carryOn`, `skipTo("…")` :316-334. A settled leg fires
   a trigger whose mark is on the leg plus its tolerance as it arrives; a mark past that never fires, and a later
   `waitUntilDone` answers "the trigger never fired" (:1040-1090).
2. **The care on a step** — `MotionOptions` (`chassis/chassis.hpp` :284-311): `timeout`, `maxLinearSpeed`,
   `maxAngularSpeed`, `positionTolerance`, `headingTolerance` (STEP1-A); 0 = the robot's default.
3. **The auton entry** — `competition/auton_registry.hpp`: `StartPose::field` :171-177, `AutonEntry` :249-284 (name,
   kind, routine | route, start, `endActionAt`, `hardStopAt`, end action); `checkAutonEntry`'s refusals :300-390 (a name
   of printable ASCII without '%', within `kMaxAutonNameChars`, not "NONE"; the schedule; `hardStopAt` within the
   period); `AutonContext` :213-240 (no camera).
4. **The wall reset** — `competition/auton_wall_reset.hpp` :8-12 (the everyday form), :20-42 (refused unless a FIELD
   start; `no sensor` when the robot declares none; `continues()` true unless `required`), :105-129 (the call);
   `localization/wall_reset.hpp` :300-316 (`WallResetRequest{wall, sensor, maxCorrection (no default), required, name}`),
   :250 (`squareWithin` 5° by default), :343-436 (every verdict and its word, e.g. `"REFUSED not square"`, `"REFUSED
   keep-out"`); `localization/override_walls.hpp` :8-10 (walls at ±70.205 in), :34-40 (Loaders, corners, VEX U Load
   Zones as keep-outs), :162-179 (`kLeftWall` … `kWalls`).
5. **The Toggle step** — `competition/toggle_visit.hpp` :1-16 (the explicit form, over a camera handle), :220-262 (the
   request: which Toggle, the flipper's push and back, eleven numbers, every one the team's from Step 0, none with a
   default), :422-440 (`toggleVisit({...})` refuses an unset number at build); `vision/override_elements.hpp` :92-144
   (the four Toggles and their names). The everyday form `checkAndFlipToggle(ctx, …)` is AF1S-A2's.
6. **Robot one** (`rob1-int`, merging): the cascade is a `Lift` whose six presets come from the facts ledger
   (`src/robots/mechanism_table.hpp` :174-195; `src/robots/robot_one_facts.hpp` :425-426, PLACEHOLDER levels `floor`,
   `L1`, `L2`, `L3` at 0/8/16/24 in, OWED; `lv(i)` :517); no flipper until its kind is typed (:150-155); no wall sensors;
   its driver file `src/team/robot_one_driver.hpp` :50-60 (the controls region), :65 (`kFeel{.mode = ArcadeBody}`).
   *[R1] The cascade is homed by a STALL (`.home = homeByStall(...)`, :185-186): it finds its zero only by a homing
   step, and every move before that is `NOT_HOMED` (`mechanism/service.hpp` :59-62, :1151-1155); a new command on it
   REPLACES a running home (`service.hpp` :399-402) — P6 (1). ROB1-A's merge is now STAGED on main.*
7. **The driver controls** — `mechanism/driver_inputs.hpp` :8-17 (one list, checked at build), :212-321 (`whileHeld`,
   `toggle`, `preset`, `homeOnPress`, `homeAtDriverStart`, `autonOnly` — "jog" is `whileHeld` on an arm or lift),
   :384-395 (the rules), and their refusals by name (`refused::whileHeld_is_for_…` …
   `refused::a_preset_on_a_mechanism_that_is_never_homed`, DECLARED :141-206 — **29 names** *[R19: the first print
   said 30, at :327-509, which is where they are RAISED]*); a preset a mechanism lacks — *[R9] by kind:*
   `no_such_preset_on_this_mechanism` for an Arm, a Lift or a Flywheel's named speed (`mechanism/kinds.hpp` :517,
   raised :776, :876, :992; fixture `k_misspelt_lift_preset.cpp`), `no_such_position_on_this_mechanism` for a
   Pneumatic only (:519, :1138); the re-zero button and the tick:
   `src/common/driver_file.hpp` :81-84; `DriverControls::kCap = 24` (driver_inputs.hpp :363). Every refusal has a
   compile-refusal fixture judged by `run_refusal.py` (`test/compile_refusals/`, *[R19]* 280 files at `a27e2bc`, 289
   with ROB1-A's staged merge; e.g.
   `n57_a_control_on_the_rezero_button.cpp`, its `// expect:` line).
8. **The feel** — `teleop/input_shaping.hpp`: `continuousDeadband` :104-114, `expoCurve` :130-135 (`axis * (expo*axis^2 +
   (1 - expo))`, the identity at expo 0), `AxisShaping{deadband, expo}` and `valid()` :142-157 (deadband in [0, 1), expo
   in [0, 1]; NaN fails), `shapeAxis` :171-173, `DriverProfile{forward, left, yaw, slew, mode}` and `valid()` :187-213;
   `SlewConfig{ratePerSecond}` (`teleop/slew_limiter.hpp` :40-45; ≤ 0 = off); `DriveMode` (`teleop/drive_modes.hpp`
   :51-63: `ArcadeBody`, `ArcadeField`, `TankSticks`). Robot two's worked example: `src/team/robot_two_driver.hpp` :48-63.
   Nothing in the build checks `Feel.valid()` (correction 5).
9. **No slow mode** — `DriverLoopConfig` (`teleop/teleop_loop.hpp` :196-223) and `driver_inputs.hpp`: no "slow",
   "precision" or speed-scale word (grep, 2026-11-13).
10. **The preview** (`rpv1-a`, in its fix round): `tools/route_preview.py` (409 lines; builds `shulib_route_preview`,
    runs it under `suite_run.py`'s bounds, exit codes 0/1/2/3); `test/route_preview_model.hpp` — `kPreviewFootprintIn`
    :48 (15 in), `Sample` :84 (true pose per pace), `Step` :111 (from-below number, kind, t0/t1, verdict, mark, the
    library's sentence), `LogLine` :133, `FieldNote` :145, `Model` :155; `test/route_preview_run.hpp` — `PreviewRobot`
    :131 (`registerAutons`, the mechanism table, the autons file), `kRobots[]` :167 (robot two; robot one "NO BUILD until
    ROB1-A lands"), `runPreview` :917; `test/route_preview_render.hpp` — the page, the field at `kPxPerIn` 5 :51-62.
    It writes HTML and a table; **no machine-readable record yet** (§1.2 adds it). *[R19] Its any-robot fakes,
    `test/fakes_by_kind.hpp`, and the fixture robots' preview rows (`test/route_preview_fixtures.hpp` :11-17, robot x
    and y) are RPV1-A's — A0 and B1 ran as ONE run. [R5] Its step `number` is the from-below count: what that means
    for the page is D13.* *[R16] Every preview runs on a STAND-IN drive, the suite's 12 in tank
    (`route_preview_run.hpp` :188-191).*
11. **The mirror** (`mirror1-a`, built, in review): `competition::mirrored(entry, override2026::kSkillsMirror)`
    (`competition/field_mirror.hpp` :177), `isMirrored(ctx)` (:195), `kSkillsMirror` (`competition/override_mirror.hpp`
    :56), the pair check (`test/mirrored_pair_check.hpp` :64, :272), the shared vectors
    (`test/goldens/mirror/skills_mirror_vectors.txt`, `FIELD x y h -> x' y' h'` lines), cookbook 09's recipe
    (`docs/cookbook/09-one-route-two-robots.md` — the shared route function, the entry function, the two registrations).
12. **The field picture** — `tools/docs_figures/source/override-field-top.png` (1138 × 1138, the manual's Figure VURS1 —
    byte-identical to MIRROR1's `manual/vurs1_field.png`, `cmp`), turned 90° counterclockwise so red is at the bottom,
    7.5 px per inch, the centre goal at (565, 570) source px (`tools/docs_figures/field_coordinates.py` :1-26, :34-36).
13. **No JavaScript tooling in the repository** — no `package.json`, no `.mjs`, no `node` in CI (`.github/workflows/
    ci.yml`); MIRROR1 left the node set-up to RMP1 (MIRROR1 D10 [R10]). *(amended 2026-11-13 late) RMP1 adds them:
    `tools/route_maker/package.json` with ONE pinned dev dependency for the browser tests (`playwright` 1.64.1, its
    lockfile committed — the app itself has none), and two CI jobs running node on Linux and Windows (A.4).*

### 0.3 G3a's design — what RMP1 reuses, and what it cuts (D7)
| G3a (brief of record, `docs/internal/chunks/G3a-robot-profile-generator-panel.md`) | In RMP1 |
|---|---|
| R.4 the route file (:600-634) | **Reused, adapted:** one project file (D3); `drawnFor` / `mirror: true` → `mirroredFor` (MIRROR1); `0` / absent in care = the robot's default; no `endAction` |
| R.5 the verb table and its refusals (:636-694) | **Reused** for D2's kinds; every run-time precondition the page can predict is a refusal in the page (§8.4) |
| R.6 / D14 care and the three failure choices (:695-739) | **Reused**, with the wall reset's mapping onto `required` (correction 7) |
| R.7 / D15 triggers (:740-777) | **Reused as STEP1-B BUILT them** (drive and turn legs; the settle rule) |
| R.7a / D16 curves | **Superseded:** CURVE1-A built the curve follower; the page offers it |
| R.8 / D17 mirroring (midline / half turn; red/blue pairs) | **Replaced by MIRROR1** (the diagonal, one line, the pair check); red/blue head-to-head pairs after the 21st |
| R.9 / D18 the timeline's two arithmetic bounds | **Reused** (D12), plus the preview's plant time |
| R.10 the F8 / VexBuilder export | **Cut** (later, additive) |
| R.11 the `[routes]` region above `registerAutons()` | **Replaced** by MIRROR1's shared skills-route file and its marked region (D4) |
| R.12 tests S37–S47, S53; mutations M36–M58 | **Reused where they apply** (§2.2, §2.3 cite them) |
| Part 5's `controls` vocabulary and D11's `driving` block (:225-310) | **Reused** as the project file's `drivers` section, field for field (D3) *[R14] — the first print's sample did not keep G3a's shapes (a flat `controls` list, `onFail` beside `care`); D3's sample now does, under `robots.<id>`, and D14 asks to confirm* |
| *[R4, R17]* G3a's three driver-file regions — `controls`, `control words`, `feel` (:258-273, :1763) | **Reused** (D9): the header map is generated, never pasted by hand |
| *[R6]* (not in G3a) | The library's own curve rules (`checkCurve`) and curve geometry (`CurvePath::at`), ported to the page and pinned by a golden (CV1) |
| Part 6's FEEL page ("the panel draws the curve, it never computes it") | **Changed:** the page computes the curve with a JavaScript copy of the formula, proved bit-identical to the C++ at 9 630 points (P2) and pinned by a golden table in the suite |

### 0.4 The probes (`.claude/coordinator/scratch/brief-RMP1/`, run by this writer 2026-11-13; every binary under `ulimit -v 4 GiB` + `timeout`)
| # | What | Result |
|---|---|---|
| **P1** | `probes/p1_route_export.cpp`: a 6-step SKILLS route written *[R19: in D4's FORM, not its listing — see the row after P4]* (a wall reset; a drive with a trigger at 50 %; a curve; a mechanism step; a drive with `onFail(skipTo("finish"))` and a 2.5 s limit; a labelled guard wait), on robot two's real mechanisms (the rig is robot two's), compiled against `tree-main` with the suite's flags (`-Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion`), run through the Auton Tester's own sequence (`test/auton_tester_rig.hpp`'s `TesterStack`) placed at its FIELD start | **Compiled first time in 4.3 s; ran: `steps=6 completed=6 failedOn=0 ok=1`**, the trigger's pivot `SUCCEEDED -- time in 0.41s`, the claw `SUCCEEDED -- time in 0.51s`, true end pose (11.96, −20.23) 88.7°, **705 paces (7.05 s of auton time)**, `ROUTINE_RAN; finished in time, end action succeeded`. The reset: `REFUSED no sensor` — and under `onFail(carryOn)` it counted as no failure at all (correction 7). `out/p1.txt` |
| **P1b** | `probes/p1b_robot_one.cpp`: the same route on ROBOT ONE's cascade (`startMove(cascade[facts::lv(2).name], 2_s)` as a 60 % trigger, `waitUntilDone`, `move` to level 3, `home`, a 1 in tolerance), plus the Controls tab's export (`controls(kRobotOneMechanisms, whileHeld …, preset … ×2, homeOnPress, homeAtDriverStart)`) and a feel (`{.deadband = 0.08, .expo = 0.6}`, slew 4.0), and an OUT-OF-RANGE feel (`expo = 1.5`) through `DriverFile`, against `tree-rob1` (`a27e2bc` + ROB1-INT's `src/`), host flags, `-Werror` | **All of it compiles — the bad feel included** (correction 5; `static_assert(!kBadFeel.valid())` holds, so only the build's silence is the gap). `out/p1b.txt` |
| **P2** | `probes/p2_feel.cpp` prints `shapeAxis(x, {deadband, expo})` as exact hex doubles for 6 dead zones × 5 curves × (257 grid + 64 off-grid) points; `probes/p2_feel.mjs` is the page's JavaScript copy of `continuousDeadband` and `expoCurve`, compared with `Object.is` (the sign of zero counts) | **9 630 points, 0 differ, 0 sign-of-zero differences.** The golden-table method works bit for bit (x86-64, no FMA contraction — §8.12). `out/p2.txt`, `out/p2_golden.txt` |
| **P3** | Headless screenshots of a one-file test page with the snap Firefox: `--headless --screenshot` in five spellings (with and without a profile, `file://` and `data:` URLs, 4 GiB and 16 GiB caps), then a WebDriver session through `/snap/bin/geckodriver` driven by stdlib Python (`probes/p3_shot.py`) | **No screenshot in any attempt** (every CLI run exited 0 and wrote no file; the WebDriver run timed out at 180 s). Cause not isolated (the snap's confinement or this agent's sandbox; a desktop Firefox was also running). A stray geckodriver (pid 3381105, port 4471) could not be stopped from the sandbox *[R18] — nor the headless Firefox it started, pid 3381195*. §2.5 makes a working capture the build's first task; *[R18] D16 asks for a path now.* |
| **P4** | RPV1-A's preview program (its `test/route_preview_*` files and `fake_telemetry_sink.hpp` copied over `tree-d33`), built and run on `"40in forward"` | **Build 4.42 s (451 MB peak), run ~0 s**; the table (`1 | drive | (0.0, 0.0) 0.0° | (39.8, 0.0) 0.0° | 0.00 | 1.80 | SETTLED`) and an 18 847-byte page. So PREVIEW's round trip is ~5 s on this laptop. `out/rpv_40in.html` |
| *[R19]* | *P1's route is NOT D4's listing: it has `onFail(carryOn)` before the reset, robot two's pivot `startRun` and a claw `set` where D4 has the cascade and `waitUntilDone`. P1b (robot one) only COMPILED. The checker's `check/d4r1` RAN D4's first listing verbatim on robot one: `NOT_HOMED`, stopped at step 2 (R1).* | |
| **P5–P8** *(re-check)* | `recheck/`: P5 the unit spellings; P6 the home rule (four variants); P7 the empty region; P8 a tank leg off its line | See "Re-check — amendments" above |
| **P-W1a–f, P-W2–P-W4** *(amended 2026-11-13 late)* | `wasm/`: Emscripten in user space; the library, the host test plant and RPV1's engine built to WebAssembly and compared with the native build; the route as data; a CMake target; the no-pace stop; the page from the site and from one file; the CSP; browsers and the old-browser refusal; GitHub's Windows runner; the screenshot recipe | See "Amendment 2026-11-13 (late)", A.1 |

---

## 1. Scope — IN (every new and changed file, by run)

### 1.1 RMP1-A — the app, the ROUTE tab, PREVIEW, save and export
*(amended 2026-11-13 late) **The file list below is the helper design's. The web app changes it as follows** (Amendment
A.3, A.6; the runs that own each piece, A.7):*
- ***DROPPED:** `tools/route_maker.py` (the helper; HS1, HS2, HS7 and RM12 go with it); `test/route_maker_preview_stub.cpp`
  ~~and the `preview` subcommand of `route_maker_main.cpp` (it keeps `describe`)~~ *[W9] — `route_maker_main.cpp` KEEPS
  `preview` (now the runner's, reading the line format): it is the ONE `main` of the native `shulib_route_maker` AND of
  the module, so WG2 has a native record and a later helper a native preview; `test/route_maker_wasm_main.cpp` is
  dropped instead*; RPV1's `--json` flag, the "maker"
  preview row and `--pair` — the runner records the run itself, and RPV1's files are READ, never changed (P-W1, P-W1f);
  `stepmap.mjs` (D13 (c)); `api.mjs` (replaced by `src/host/`); the COMMITTED `route_maker.html` and `build.py --check`
  (built in CI and at publish — A.3.3).*
- ***NEW, the page (A1):** `tools/route_maker/src/host/` — `files.mjs`, `files_fsaccess.mjs`, `files_download.mjs`,
  `engine.mjs` (A1: PREVIEW shown as "arrives with the next update"; A2u adds `engine_wasm.mjs`), `robots.mjs`,
  `capabilities.mjs` (A.6); `tools/route_maker/src/browser_check.js` — the classic inline script that runs FIRST and
  refuses an old browser in plain words (P-W3) *[W16] — module-Worker support included; the page states Chrome/Edge 97,
  Firefox 115, Safari 16*; *[W12] `tools/route_maker/src/model/region.mjs` (the marker, hand-edit hash, splice and
  line-ending rules — pure); [W7] `tools/route_maker/src/model/numbers.mjs` (THE number formatter and edit-time
  rounding, used by `export.mjs` AND `route_lines.mjs`); [W11] `tools/route_maker/.gitattributes` and
  `test/goldens/route_maker/.gitattributes` (`* text=auto eol=lf`)*; `tools/route_maker/package.json` + `package-lock.json` (`"type":
  "module"`, scripts `test` and `test:page`, ONE dev dependency, `playwright` 1.64.1 exact); `tools/route_maker/test/page/`
  (the browser tests, A.4); `docs/route-builder.md` (the tab's page) and its `mkdocs.yml` nav entry; `ci.yml`'s two
  jobs and `pages.yml`'s steps (A.3.5, A.4).*
- ***NEW, the engine (A2e):** `test/route_maker_runner.hpp` — THE RUNNER: reads the route in P-W1d's line format (one
  step per line, `key=value` words — no JSON reader is vendored; `test/vendor/` holds only doctest), maps each step to
  the library call the export writes (P-W1d's mapping), runs it through RPV1's `previewEntry` with `opt.onLoop` set
  (P-W1f), and WRITES the record as JSON (schema 1: the samples as `[t, x, y, theta]` rows; each PAGE step's first and
  last pace and verdict; RPV1's steps, lines and field notes; the exit word) — compiled natively by the suite (WG2) and
  into the module; ~~`test/route_maker_wasm_main.cpp` — the module's `main`~~ *[W9] `test/route_maker_main.cpp` — the
  module's `main` AND the native program's* (`callMain(["preview", <lines>])`,
  `["describe", <robot>]`); `tools/route_maker/wasm/CMakeLists.txt` (A.3.3); `tools/route_maker/src/model/route_lines.mjs`
  — the page's model → the line format (WG1 proves the runner reads it as the export writes it);
  `tools/route_maker/test/wasm/` (WG1–WG3, WG7 under Node). **(A2u):** `tools/route_maker/src/worker.mjs` and the
  playback in `timeline.mjs` / `field_view.mjs`.*
- ***CHANGED:** `build.py` — `--out DIR` writes the SITE form (the modules, `preview.mjs` + `preview.wasm`,
  `robots.json`) and the ONE FILE (everything inlined, the single-file module base64) — stdlib still; `refusals.json`
  generated at build, its node test unchanged; the page's CSP is A.3.9's.* *[W1, W3, W10] The SITE form's page is
  **`app.html`** (never `index.html` — A.3.1) with the SITE policy; the ONE FILE gets the ONE-FILE policy; both carry
  `<link rel="icon" href="data:,">` and `<meta name="shulib-app" content="route-builder">`; `build.py` refuses an
  output containing an absolute path (RG1).*
- ***Where the text below names the helper** (`GET /api/robots`, `POST /api/save`, `POST /api/export`, `GET
  /api/status`), read the host interface that replaces it — `robots.mjs`, `files.saveProject`, `files.writeRegion`,
  `files.readTeamFile` (A.6) — under the same rules (only between existing markers under `src/team/`, the hand-edit
  hash in the project file, and now the file's own line endings kept).*
*[R7] Split in two runs (D11 (a′)): **A1** = everything below EXCEPT what is marked **(A2)**; **A2** = the PREVIEW
half — RPV1's `--json` and "maker" row, `--pair`, `preview` in `route_maker_main.cpp`, `POST /api/preview`,
`stepmap.mjs`'s aligner, the scrubber and playback in `timeline.mjs` / `field_view.mjs`, SM1–SM4, HS6–HS7.*
*[R1, R6, R13] `checks.mjs` gains RC25 (the home rule) and RC26 (the curve rules), and every path check runs on both
robots' step lists. [R6] NEW `curve.mjs` — `CurvePath`'s geometry (Catmull-Rom tangents, chord-length Hermite
segments, `curve_path.hpp` :15-25) and `checkCurve`'s rules (:27-43), the planned curve and RC2's off-field check
taken from ITS samples, pinned by CV1. [R5] NEW `stepmap.mjs` — the exporter's step map (A1) and the aligner (A2),
D13. [R14] NEW `api.mjs` — the ONLY module that talks to the helper (the panel later swaps it for `postMessage`).*
**The app's files (all NEW):**
- `tools/route_maker/src/model/` — PURE ES modules, no DOM (each with its `node --test` file under
  `tools/route_maker/test/`):
  - `route.mjs` — the route model (start, steps, labels, triggers, care, failure choice, who-runs-it); every edit a pure
    function returning a new state; `plannedPoses(route)` (each step's planned end pose from the one before: a drive ends
    at its point facing its bearing — reversed for `reverseTo` —, a turn at its heading, a curve at its last point and
    heading or its last segment's bearing, a mechanism step, a wait or a reset where it stood).
  - `checks.mjs` — every refusal and warning, each returning `{level: "error" | "warn", stepIndex, field, words,
    library}` where `library` is the library's OWN sentence (§1.1 `refusals.json`), `words` the plain English beside it.
    The list: §2.2 RC1–RC24, *[R1, R6] RC25–RC26*.
  - `export.mjs` — route → D4's commented C++ (deterministic, byte-stable *[R4, R12]: the region carries no time and
    no hash — its hash goes in the project file's `exports`*); the robot-two line (D6) *[R2] — only for the "missing"
    fallback*; the controls and feel blocks are RMP1-B's. *[R3] An EMPTY route exports §6.3's empty region byte for
    byte (GL0).*
  - `mirror.mjs` — MIRROR1's two maps; `bandHits(route)` (steps whose planned square comes within 3 in of y = −x);
    `mirrorNotes(route)` (a step at a Loader or in a Load Zone).
  - `timebounds.mjs` — D12's two numbers (G3a R.9's arithmetic; the robot's top speed, turn rate and default timeout from
    the robot description).
  - `project.mjs` — load / save / migrate `route_maker.json` (schema 1; a newer schema refused by name); `undo.mjs` — a
    bounded history of states (every edit, drag-end and import undoable; Ctrl+Z / Ctrl+Shift+Z).
  - `field.mjs` — the field's numbers FROM the robot description (walls, keep-outs, Loaders, Load Zones, Toggle places,
    start pieces — emitted by the C++, never hand-copied), the 15 in square's off-field check (RPV1's THROUGH rule: a
    corner more than 1 in past a wall's inner face), snapping targets.
- `tools/route_maker/src/ui/` — the screen (DOM + SVG): `app.mjs` (the shell, the store, routing between tabs, toasts),
  `field_view.mjs` (the field picture under an SVG layer; pan with drag / two fingers / space-drag, zoom with the wheel,
  pinch and +/−, "fit" with 0; the robot's 15 in square drawn to scale with its front marked; drag handles for points
  and headings; snapping), `step_list.mjs` (numbered cards with icons, drag to reorder, the step playing highlighted),
  `inspector.mjs` (the selected step's form: every number with its unit, the failure choice, the triggers, "who runs it";
  inline checks under each field), `timeline.mjs` (the bounds bar; after PREVIEW, the scrubber), `export_drawer.mjs`,
  `help.mjs` (first-run tour, `?` overlay of shortcuts, hover help on every control).
- `tools/route_maker/src/style.css` — THE DESIGN TOKENS, shared by RMP1-B (fixed here so B can build beside A):
  `--font-ui` (the system UI stack), type scale `--t-xs 12px / --t-sm 13px / --t-md 14px / --t-lg 16px / --t-xl 20px /
  --t-2xl 24px`, spacing `--s-1 4px … --s-8 48px`, `--radius-sm 6px / --radius-md 10px`, colour roles
  (`--bg`, `--surface`, `--surface-2`, `--border`, `--text`, `--text-muted`, `--accent` (the docs figure's violet,
  #7a3db8), `--red-alliance`, `--blue-alliance`, and RPV1's status marks `--ok` green, `--unconfirmed` teal, `--failed`
  red, `--cut` amber, `--grey`), each defined for light and dark (`prefers-color-scheme` and a toggle), `--motion-fast
  120ms / --motion 200ms` (zero under `prefers-reduced-motion`), hit targets ≥ 32 px, a visible focus ring.
- `tools/route_maker/assets/` — the icon set (inline SVG symbols, drawn for the app) and the controller drawing (B).
  *[R11]* and `field.webp` (or `.jpg`): the field picture turned red-at-bottom exactly as `field_coordinates.py` turns
  it, cropped to the perimeter, ≤ 250 KB — made ONCE by `tools/docs_figures/route_maker_field.py` (Pillow, like the
  docs figures, `field_coordinates.py` :14) and COMMITTED.
- `tools/route_maker/build.py` — stdlib: inlines the modules (in dependency order, as one `<script type="module">`), the
  CSS, the icons, the field picture *[R11] (the committed file, base64 — `build.py` never decodes an image, so its
  `--check` needs no Pillow)*, `refusals.json` (below) ~~and the robot descriptions last generated~~ *[R11] — NO robot
  description: it would go stale at every facts edit; the project file carries the last one the helper read (D3's
  `seen`)*, into `tools/route_maker/route_maker.html` (COMMITTED, so a double-click works); `--check` rebuilds in
  memory and fails if the committed file differs (CI and the suite). The page makes **no network request of any kind**
  *[R11]: a Content-Security-Policy in the page itself (`default-src 'none'`; inline script and style; images and
  fonts from `data:` only; `connect-src` only `http://127.0.0.1:*`), so the browser refuses any other request; and a
  test (HS8): no URL in the file except the XML namespace URIs (`http://www.w3.org/2000/svg`, `…/1999/xlink`,
  `…/1999/xhtml`) and 127.0.0.1, no `fetch` / `src` / `href` / `url(` naming any other host, the CSP present and
  exact.*
- `tools/route_maker/refusals.json` — GENERATED by `build.py` from the headers: every `SHULIB_PRECONDITION` sentence the
  route steps can raise, every `refused::` name the controls can hit, every wall-reset verdict word; the page shows
  these strings verbatim. A node test fails when a refusal the page uses is missing from the extraction (the library's
  words changed and the page did not follow).
- `tools/route_maker.py` — THE HELPER (stdlib `http.server`, bound to `127.0.0.1` only, a fixed default port with a
  fallback; opens the browser; a per-launch token every write must carry; the `Origin` checked). Endpoints:
  `GET /` (the page), `GET /api/robots` (each robot's description, from the describe program), `POST /api/preview`
  (D5), `POST /api/save` (writes `src/team/route_maker.json`, atomically, after a backup), `POST /api/export` (D4: writes
  ONLY between existing markers in a file under `src/team/`; refuses a symlink, a path outside `src/team/`, a region
  whose hash shows a hand edit unless the request says "overwrite"), `GET /api/status` *[R2] (also: does robot two's
  autons file carry the mirrored line? does the skills-route file carry the markers?)*. Exit codes and bounds as
  `route_preview.py`'s (it imports `suite_run.py` by path, as RPV1 does). `self-test` subcommand (§2.2 HS1–HS9).

**The C++ side (NEW, host only — `test/`; nothing a robot program compiles):**
- `test/route_maker_describe.hpp` — one robot's description as JSON from its REAL tables: every mechanism (name, kind,
  presets / positions with values and units, homing method, `neverHomed`, `unhomedVolts`, piece sensor, stall
  signature, a two-position pneumatic or not), the wall sensors (declared or not, their sides, `squareWithin`), the
  chassis's top speed / turn rate / default timeout, the API version, and the field (`override_walls.hpp`'s walls and
  keep-outs, `override_elements.hpp`'s Toggle places, the Loaders and Load Zones). The page's field numbers are these.
  *[R6] Also: is it a tank (no strafe authority), and its `tankSteer.maxSteer`; the curve config `checkCurve` reads.
  [R16] A number the robot's tables do not state is written as ABSENT, never filled from the stand-in drive.*
- `test/route_maker_main.cpp` — the program `shulib_route_maker`: `describe --robot one|two` *[R8] `|x|y|z` (the
  fixture robots, `test/mechanism_fixtures.hpp`)* and `preview` **(A2)** (RPV1's
  `runPreview` with a third preview robot row, "maker", whose `registerAutons` is the scratch file's
  `registerRouteMakerPreview`, and the robot's table and rig — *[R19] RPV1-A's* any-robot fakes, `test/fakes_by_kind.hpp`).
- `test/route_maker_preview_stub.cpp` — the committed stand-in the build compiles until the helper writes
  `build/route_maker/preview_route.cpp`.
- `test/CMakeLists.txt` — the `shulib_route_maker` target (beside `shulib_route_preview`), and the golden test's sources.

**RPV1's files (CHANGED by RMP1-A, after RPV1-A merges):**
- `test/route_preview_render.hpp` (or a new `test/route_preview_json.hpp`) — `renderJson(const Model&)`: the Model as
  JSON (schema 1: the samples as `[t, x, y, theta]` rows, the steps, the lines, the field notes, the exit code, the
  stamp). `test/route_preview_run.hpp` — the `--json FILE` flag and the "maker" row hook. Additive: RPV1's page, table
  and exit codes unchanged (its 40 cases stay green).
- With MIRROR1-A merged: a `--pair` mode — runs an entry and its mirrored copy, each on its own plant, and MIRROR1-A's
  `comparePair` — output `{one, two, pair}`.

**Goldens and tests (NEW):** `test/goldens/route_maker/*.route.json` and their exports `*.cpp` (one per step kind and
per failure choice — §2.2 GL0–GL19); `test/route_maker_golden_test.cpp` (compiles and RUNS every golden on the host test
plant *[R8] — the behaviour asserted on the fixture robots; robot one and two a smoke run*); *[R6]*
`test/goldens/route_maker/curves.txt` (CV1: `CurvePath::at` samples and `checkCurve` verdicts, made by the C++, read by
both suites); `tools/route_maker/test/*.test.mjs`; `package.json` (`"type": "module"`, one script `test`, **no
dependencies**).

**CI:** `.github/workflows/ci.yml` gains one step — `node --test tools/route_maker/test/` and `python3
tools/route_maker/build.py --check` (shared with ROB1-INT's ci.yml edit — §6). *(amended 2026-11-13 late) Replaced by
A.4's two jobs — `route-builder` (ubuntu-24.04) and `route-builder-browsers` (ubuntu-24.04 AND windows-latest); no
`--check`. And `package.json` gains its one pinned dev dependency (above).*

### 1.2 RMP1-B — the CONTROLS tab and DRIVING FEEL
*[R7] B starts after A1's shell files are merged and touches ONLY this list plus one tab entry in `app.mjs`. [R4, R17]
It writes THREE regions per driver file (D9) and re-points the pin tests (§6.4).* *(amended 2026-11-13 late) B writes
through `files.mjs` like A1 (Copy where a browser cannot write to a folder); its controls agreement and feel golden are
unchanged (node + the suite); its tab joins the Linux and Windows browser jobs (A.4). After the 18th its exports are
the students' controls/feel changes D15 allows — the `control words` region moving with them (D15 (1)).*
- `tools/route_maker/src/model/controls.mjs` — the controls model from the robot description; `checkControls` — one
  rule per `refused::` name *[R15, R19] — all 29 declared (`driver_inputs.hpp` :141-206), the agreement run on FORCED
  configurations over the fixture robots x, y and z (every kind and homing method), since robot one and two will have
  only the cascade* (§2.2 CT-series), plus the re-zero button
  (`driver_file.hpp` :81-83) and a preset the mechanism lacks (*[R9]* `no_such_preset_on_this_mechanism` for an arm, a
  lift or a flywheel speed; `no_such_position_on_this_mechanism` for a pneumatic — `kinds.hpp` :517, :519);
  `exportControls` — the `kControls = controls(kRobotOneMechanisms, …)` list, one commented line per control, in the
  order the build reads; *[R4] `exportControlWords` — the header's stick-and-button map, the `control words` region,
  from the same choices and the drive mode.*
- `tools/route_maker/src/model/feel.mjs` — `continuousDeadband`, `expoCurve`, `shapeAxis` (the C++'s operations in the
  C++'s order, P2), the slew's time to full speed (`1 / ratePerSecond` s from rest; off at ≤ 0), `checkFeel` (what
  `AxisShaping::valid()` refuses, with its sentence; a negative ramp refused as the page's own stricter rule — the C++
  reads it as off, G3a D11), `exportFeel` (the `kFeel{…}` block, only the stated fields, in declaration order *[R17]
  — and `kLoop`'s two team lines, the `feel` region, D9*).
- `tools/route_maker/src/ui/controls_tab.mjs` — the V5 controller drawn in SVG (L1 L2 R1 R2, the d-pad, X B Y A, both
  sticks): a used button shows its job as a label; click → a popover: mechanism → the actions its KIND allows (a Power
  mechanism: run while held, as a forward/back pair; an Arm or Lift: move while held (jog), go to a level/preset, home
  now, home when driver control starts; a two-position Pneumatic: toggle, go to a position; a Flywheel: toggle at a
  named speed, go to a speed; any: "autons only") → the second button of a pair, the volts, the time limit. The stick
  map follows the drive mode. Re-zero's button shown as taken.
- `tools/route_maker/src/ui/feel_tab.mjs` — the drive mode; per stick axis (forward, turn; strafe only on a holonomic
  robot) a dead-zone slider (0–0.5, typed up to 0.99) and a curve slider (0–1); THE GRAPH (stick position −1…1 along
  the bottom, requested speed −100…100 % up the side, the library's curve drawn from 201 points, the straight line
  faint for comparison, a draggable "try it" stick marker reading "30 % stick → 12 % speed"); the ramp with its "full
  speed from rest in 0.25 s" readout; what the drive does on release; the export block. *[R16] The side axis is
  labelled "the forward (or turn) REQUEST, −100…100 %" — what the library shapes, not a measured robot speed. In
  TANK STICKS the axes are per REQUEST, not per stick: the page says "both sticks are first averaged into a forward
  request (left + right) / 2 and a turn request (right − left) / 2; the curve shapes those" (`drive_modes.hpp` :9-11,
  :89-91; `shapeRequest` after it, `input_shaping.hpp` :215-219); in arcade the graph shows one axis with the other
  stick still.*
- `src/common/driver_file.hpp` — **D10 only:** `static_assert(Feel.valid(), "...")` beside :81-84, and its fixture
  `test/compile_refusals/rm01_a_feel_out_of_range.cpp` (`// expect:` its sentence).
- Goldens: `test/goldens/route_maker/controls_*.json` → `*.hpp` (every control word on robot one and robot two, and the
  defaults reproducing each robot's current file byte for byte), `feel_*.json` → `*.hpp`; `test/route_maker_golden_test.cpp`
  gains them (compiled through `DriverFile`); the feel's golden table `test/goldens/route_maker/feel_curve.txt` (P2's
  form, hex doubles) checked by BOTH the C++ suite and node; generated refusal fixtures
  `test/compile_refusals/rm_*.cpp` (§2.2 CT-series).
- *[R4]* `test/robot_one_driver_test.cpp` (T11, GM22) and, if ROB1-BT has not, `test/robot_two_driver_test.cpp`
  (MECH-A5) — re-pointed from "today's map" to the rules they protect (§6.4).
- *[R17]* `src/team/robot_one_driver.hpp` / `robot_two_driver.hpp` — ONLY if ROB1-BT launched without them: the
  `control words` and `feel` MARKER LINES (the content between them unchanged byte for byte); the src gate re-run.

### 1.3 RMP1-C — polish
The UX review's and the usability session's findings (§2.5); the motion pass (playback easing, panel transitions,
the robot's turn animation interpolated by the shortest way, matching F3's angle rule); the accessibility pass
(keyboard path through every action, ARIA labels, contrast ≥ 4.5:1 in both themes, focus order); the screenshots for
the docs; and whatever of D6's two-robot PREVIEW or the checked Toggle step arrived late. No new step kind.
*(amended 2026-11-13 late) Plus: live PREVIEW while editing (fast enough now — A.3.10); a service worker for offline
use, only if the one file is not enough; the Chromium/Edge screenshots from the CI artifact (D18).*

### 1.4 The interaction model (RMP1-A unless marked B)
**Information architecture.** A top bar: the project's name and its save state ("Saved" / "Unsaved changes" / "Saved
as a draft in this browser"), the robot picker (one / two), the tabs **Route** and **Controls** (B), the theme toggle,
help (`?`). The Route tab: the step list on the left; the field in the middle with its toolbar (the step kinds, snapping
on/off, zoom, fit); the inspector on the right (the selected step); the timeline across the bottom (the bounds bar;
after PREVIEW the scrubber); **PREVIEW** and **Export** as the two primary buttons, top right. At ≤ 1100 px the step list
and the inspector become drawers. The Controls tab (B): the controller in the middle, the selected button's form on the
right, the Driving feel section below with its graph; Export in the same place.

| Step kind | Added by | Edited by | Drawn as |
|---|---|---|---|
| Start | the empty field's first prompt ("Place robot one's start") | drag the square; drag the heading arrow (snaps 15°/45°/90°, Shift = 1°) | the 15 in square, front marked, in the robot's colour |
| Drive to | toolbar **D**, then click | drag the end point; inspector fields; "backwards" switch; *[R6]* "turn to face it first" (on a tank: ON by default) | a straight dashed planned leg, the end square as a ghost, an arrowhead; *[R6]* the turn drawn as an arc at the start when on; when off and the leg starts off its line, the leg amber (warn) or red (past the robot's 15° — RC24) with "turn to face it first?" |
| Turn | **T** — "to a heading" (drag a dial) or "face a point" (click it) | the dial / the point | an arc at the robot showing the sweep and its direction |
| Curve | **C**, then click 2–16 points, Enter / double-click to finish | drag any point; add a point on a segment; delete one; the last point's optional heading | ~~a dashed line through the points (never a smoothed shape the follower might not take — §8.1)~~ *[R6] the LIBRARY's own curve geometry (`CurvePath`, ported and pinned by CV1), dashed as planned; a point that breaks `checkCurve`'s rules red with its sentence; the tightest bend marked when under the warn radius* |
| Mechanism | **M** → the robot's mechanisms → the actions its kind allows | inspector: level / position / volts / time limit; "wait until done" or "start it and keep going"; *[R1]* on a stall- or switch-homed arm or lift with no finished home before it: RC25's error with "home it first" / "home it while the first leg drives" (the time limit left empty for the student) | an icon chip on the step card; on the field a badge at the step's spot |
| Wait until done | from a started mechanism's chip ("collect it") or **M** | the extra time allowed | a chip |
| Trigger | on a drive or turn: "+ start something partway" | the mark (%, in, °) as a slider ON the leg (drag the tick along it), the start form | a tick on the leg with the mechanism's badge; "fires only if the robot gets there" |
| Wall reset | **W** → the wall (click it on the field) → the side sensor | max correction; if refused: carry on / stop / skip | the sensor's beam from the robot to the wall, green when square, red when not |
| Toggle — checked | **G** → click a Toggle (enabled once AF1S-A2 merges) | the visit numbers (Step 0's) | the look spot and push spot on the Toggle's normal |
| Toggle — blind | **G** → "blind flip" (when the robot has a flipper) | the flipper's push and back | two chips |
| Wait | **P** | seconds | a clock chip |

Every step: **if it fails** (stop / carry on / skip to…; the skip drawn as an arrow to its label), **a label**, and on a
mirrored route **who runs it**. Reordering re-plans every later pose at once; a check that a reorder breaks shows on the
moved step. **Snapping:** tile lines and centres (24 in, half-tiles 12 in), a wall (the square flush to it), the Toggle
look spots, the start tiles; Alt holds snapping off. **Shortcuts:** D T C M W G P to add, Delete, Ctrl+D duplicate,
Ctrl+Z / Ctrl+Shift+Z, Ctrl+S save, Ctrl+E export, Ctrl+Enter PREVIEW, Space play/pause, ←/→ step through the preview
(Shift = 0.1 s), Home/End, +/−/0 zoom, Esc cancel, `?` the list. **Empty state:** the field with one prompt and a
three-line "your first route" coach (place the start → add a drive → press PREVIEW), dismissible, re-openable from help.
**Error states:** each in place, in plain words with what to do: the helper not running (one banner, the command to
start it, PREVIEW and write-to-file greyed); a build that failed (a drawer with the compiler's own words — never hidden);
a preview that was refused, threw, went THROUGH a wall or looped (RPV1's sentences, the step marked); the robot
description out of date (re-read it); a hand edit inside the export's markers (overwrite / keep, the student's choice);
a project file from a newer route maker (refused by name). Never a wall of errors at export: Export is disabled while
any step has an error, and its tooltip lists the first ones with "go to step". *[R2, R3] Two more states: the
skills-route file has no markers yet (before ROB1-BT) — "Write to the file" greyed, Copy offered, one line saying
the file arrives with robot two's update; robot two's mirrored line missing — said once, in the mirror panel.
[R5] A preview whose steps could not be matched to the route plays unnamed, with one line saying so.*
*(amended 2026-11-13 late) The helper's states are replaced by: **a browser too old** — the yellow box before the app
loads (P-W3: "This browser is too old to run the route preview…"); **a browser that cannot write to a folder**
(Firefox, Safari) — said once: "Save downloads the project; Export copies the code; to write into the shulib folder,
open the builder in Chrome or Edge"; **the folder not connected, or its permission not granted again** — one
"Connect your shulib folder" button; **the preview engine failed to load** (a damaged one file, a blocked `.wasm`) —
PREVIEW greyed with the reason, everything else working; **a preview that did not finish in 20 s** — the Worker
stopped, the page says so. Gone: "the helper not running", "a build that failed" (nothing is compiled in the browser)
and "could not be matched" (D13 (c)). **The robot description out of date** now means the builder was published
before the robot's facts changed: the page shows the commit and date it was built from (A.3.4).*

---

## 2. Tests, mutations, UX acceptance, definition of done

### 2.1 The runs (each its own go; verified and committed before the next merges)
*(amended 2026-11-13 late) Superseded by Amendment A.7 — six runs (A1, A2e, A2u, A1b, B, C), planned on D15 as
answered and on D19. The table below is the re-check's.*
| Run | Needs at its tip | Size (builder-days) | Lands |
|---|---|---|---|
| *[R7]* **RMP1-A1** | its model and page: nothing; robot one's description: ROB1-A merged (staged on main now); write-to-file: ROB1-BT merged with §6.3's contract (else Copy only); the mirror ghost: MIRROR1-A's vectors file | **~5½–6½** (shell + design system ~1; field view, drag/snap/pan/zoom ~1; the step kinds, inspector, checks incl. RC25, RC26, turn-first, both robots' paths, planned poses, undo ~2; export + goldens on the fixture robots + the C++ golden test ~1¼; helper describe/save/export + the describe program ~¾; guide 22's first half ~½) **+ ~1 review/merge** | **~Tue 11-17 to Wed 11-18** |
| *[R7]* **RMP1-A2** | A1 merged; RPV1-A merged; the two-robot PREVIEW: MIRROR1-A merged (else RMP1-C) | **~2½–3½** (RPV1 `--json` + the maker row ~¾; the step map's aligner + SM goldens ~¾; playback + scrubber + timeline ~1; `--pair` and the two robots ~½; docs ~¼) **+ ~½** | **~Thu 11-19 to Fri 11-20** |
| RMP1-B | *[R7]* A1's shell merged (B touches only §1.2's list + one tab entry); the driver markers (D9) | **~2½–3** (controls model + the 29-name agreement on fixture robots ~1¼; the controller UI ~½; feel model + graph + golden ~½; the regions, pin re-points, D10 + docs ~½) **+ ~½** | **~Nov 19–20, or after the 21st** |
| RMP1-C | A1, A2 and B merged; the usability session held | **~1½–2 + ~½** | **after the 21st** |
| *First print, superseded:* RMP1-A ~5–6 + ~1 (~Nov 16–17); B ~2–2½ + ~½ (~Nov 18–19); C ~Nov 19–20 | | | |

### 2.2 Tests — ADVERSARIAL: each tries to break ONE rule (FAILS first on a deliberately broken model, then passes)
**The export compiles AND runs (the C++ suite, `test/route_maker_golden_test.cpp`; the goldens are the JS exporter's
output, committed, and node asserts they are fresh):**
- *[R8] Behaviour is asserted ONLY on the fixture robots (`test/mechanism_fixtures.hpp` — x: a restsAtStop arm, a
  switch-homed lift, a flywheel, an intake, a 3-position pneumatic; y: a stall-homed arm, a never-homed lift, …),
  whose numbers the test owns. Robot one and robot two each get a SMOKE golden: compiled with the suite's flags and
  run without throwing, no verdict asserted — the builder types robot one's facts this week.*
- **GL0 the empty region** *[R3]* — an empty route exports §6.3's empty region byte for byte; it compiles and runs
  (`steps=0 ok=1` — re-check P7).
- **GL1–GL11, one golden route per step kind** — start + drive *[R6] (turn-first on and off)*, reverse, turn to a
  heading, face, face away, curve (2 and 16 points, forwards and backwards, with and without an end heading), each
  mechanism action ~~on robot one's cascade and robot two's claw~~ *[R8] on the fixture robots' every kind* (blocking
  and started, `waitUntilDone`, *[R1] a home first*), a trigger by %, by inches and by degrees, a wall reset (each wall
  and side), a wait, the checked Toggle (only once AF1S-A2 has merged), a "robot one only" block (once MIRROR1-A has
  merged): each **compiles with the suite's flags AND runs on the host test plant**, its step counts, verdicts and end
  pose asserted (P1's method — the plant placed at the FIELD start).
- **GL12 the failure choices** — the same forced failure (a blocked leg; a mechanism stalled) under stop / carry on /
  skip: exactly the counts each predicts (`completed`, `failedOn`, `jumped`, `stoppedAt`) and the run never throws (G3a
  S44, S53).
- **GL13 a cut** — a route whose wait spans the 57 s cut: the guard pause ends AT the cut (mutation RM10's oracle; G3a
  M57).
- **GL14 the wall-reset mapping** — carry on → `required` false and no `onFail`; stop → `required` true; skip →
  `required` true + `onFail(skipTo)`: on the rig (no sensor) carry on goes on, stop stops, skip skips (correction 7).
- **GL15 the full skills golden** — D4's sample route, compiled and run, its export equal to D4's listing byte for byte
  (the brief's sample cannot drift from what the tool writes). *[R1, R8] D4's listing now homes first; on robot one it
  is a SMOKE (compiles, runs, no verdict — its placeholder gearing times the move out, P6 (2)); its behaviour twin is
  the same route on robot x (`lift["low"]` for the cascade's level), asserted 7/7, ok, end (11.96, −20.23) 88.7° ±
  the test's tolerance (re-check P6 (4)).*
- **GL17 one leg with everything** *[R12]* — `label` + `onFail(skipTo)` + two triggers on one drive, in the only order
  STEP1-B accepts; compiled and run; the skip taken when the leg is blocked.
- **GL18 the home rule's twin** *[R1]* — the route RC25 refuses (a trigger's move on robot y's stall-homed arm with
  no finished home; and `startHome` then a trigger on the same leg), forced past the page: the run stops
  `NOT_HOMED` at that step (re-check P6 (1); the checker's `d4r1_nohome`).
- **GL19 the caps' spellings** *[R10]* — a drive with `.maxLinearSpeed = 30_in / 1_s` and a turn with
  `.maxAngularSpeed = units::AngularVelocity{270.0 * math::Angle::kPi / 180.0}`: compiled, run, the caps read back
  (30 in/s; 4.712 rad/s).
- **HS9 / the export in the real files** *[R4]* — every golden written by the helper into a SCRATCH copy of the tree
  (`git worktree` or `git archive`), and the suite and the src gate's builds run there: green. (RMP1-B adds the driver
  files' three regions to it.)
- **GL16 the mirrored pair** (MIRROR1-A merged) — GL15 registered as robot one's and mirrored as robot two's: both run,
  and the pair check passes; a route kept 2 in off the diagonal FAILS it, naming the step. **GL16b** *[R13]* — a
  "robot one only" start and a shared `waitUntilDone`: the page refuses it (RC17 on robot two's list); forced past the
  page, robot two's run THROWS the precondition (`routine.hpp` :978-980).
- *(amended 2026-11-13 late) SM1–SM4 are now the RUNNER's spans (D13 (c)), in A2e; "a deliberately wrong map"
  becomes a runner whose span is one pace off → red.*
- **SM1–SM4 the step map (A2)** *[R5]* — each golden's step map matched to its preview run: SM1 a skip (the jumped
  steps "skipped", the target lit), SM2 carry on (the failed step red, the rest named), SM3 the 57 s cut (the cut step
  amber, every later one "not reached"), SM4 robot two's numbering with a "robot one only" block (MIRROR1-A). A
  deliberately wrong map makes the page play UNNAMED, never mis-named.
- **CV1 the curve golden** *[R6]* — 200 seeded curves (2–16 points, headed and not, forwards and backwards, start
  poses around them): `CurvePath::at` sampled at 64 arc lengths and `checkCurve`'s verdict and point, from the C++;
  `curve.mjs` equal — the verdict and point EXACTLY, the positions within 0.001 in (the library stores its arc-length
  tables as float, `curve_path.hpp` :23-24).

**A route saved and re-opened identically (node):**
- **RT1** every golden `route.json` → load → save → byte-identical; **RT2** load → edit → undo every edit → equal to the
  original (deep-equal) and the export byte-identical; **RT3** a randomized edit script (200 seeded edits of every
  kind) → save → load → equal state; **RT4** a file with `schemaVersion: 2` is refused by name, nothing loaded; **RT5** a
  file whose mechanism is not on the robot loads with that step marked, never dropped.

**The page refuses what the library refuses, in the library's words (node; each RC case = a route the page must mark
red, with `library` equal to the extracted sentence, AND — where the library refuses at build or at run time — a C++
twin proving the library does refuse it):**
- **RC1** a point whose 15 in square crosses a wall's inner face by more than 1 in (RPV1's THROUGH: x = 63.8 refused,
  62.7 allowed) — twin: RPV1's field check flags THROUGH on the same route; **RC2** a curve ~~point~~ off the field
  *[R6] — checked on the SAMPLED library curve (`curve.mjs`, CV1), not the polyline: a Hermite segment can bulge past a
  wall between two points that are both inside*; **RC3** a start outside the field;
- **RC4** a trigger at 0, at 100.5, at NaN (`percentMark`'s sentence) — twin: the C++ throws that precondition; **RC5**
  after 0 in / after 0° (`distanceMark` / `turnedMark`) *[R12] and after MORE than 180° (an angle wraps — 270° is −90°,
  refused by `turnedMark`'s own sentence, `routine.hpp` :2019-2026)*; **RC6** a trigger on a curve (no such leg — the page offers none;
  a hand-edited file with one is refused); **RC7** a 5th trigger on a leg; **RC8** two triggers on one mechanism on one
  leg; **RC9** a trigger with a blocking action (the five `static_assert` sentences — move, home, run, spinUp, set — `routine.hpp` :1164-1203); **RC10** an
  "after N in" longer than the planned leg plus its tolerance (warn: "never fires" — STEP1-B's settle rule);
- **RC11** a wall reset whose side sensor's beam, at the PLANNED pose, is not within `squareWithin` (5°) of the wall's
  normal or faces away (`"REFUSED not square"`, with "turn to 0° first?") — the robot facing 45° with its RIGHT sensor
  on the red wall is refused; facing 0° allowed; facing 180° allowed only with the LEFT sensor; **RC12** a reset whose
  beam lands in a keep-out (`"REFUSED keep-out"`, the span named); **RC13** `maxCorrection` 0 or absent ("maxCorrection
  UNSET…"); **RC14** a reset on a robot whose description has no wall sensors — a WARNING on the step (`"REFUSED no
  sensor"` on the robot; correction 3);
- **RC15** a skip to a label that is earlier or missing (a skip goes forward only — `warnBehind`); **RC16** two
  labels the same; **RC17** a `waitUntilDone` with no start of that mechanism on EVERY path to it (G3a R.5's
  definitely-started rule; a trigger's start counts, as STEP1-B defines) *[R13] — on BOTH robots' step lists (robot
  two's with the one-robot blocks applied), since a wait on a mechanism the run never started THROWS (`routine.hpp`
  :978-980); RC15 likewise checks a skip's label exists on each robot's path*; **RC18** a preset / level / position the
  mechanism lacks (*[R9]* `no_such_preset_on_this_mechanism` for an arm, a lift or a flywheel speed;
  `no_such_position_on_this_mechanism` for a pneumatic) — twin: the compile-refusal fixture (e.g.
  `k_misspelt_lift_preset.cpp`, `n23_misspelt_position.cpp`); **RC19** a mechanism time
  limit ≤ 0 or a run whose "or after" is longer than its limit (the service's preconditions); **RC20** homing a
  `neverHomed` mechanism; **RC21** a name over 29 characters, or over 25 when mirrored (MIRROR1 D5), with '%', or
  "NONE" (`checkAutonEntry`'s words); **RC22** a mirrored route with a step whose planned square comes within 3 in of the
  diagonal and is not "robot one only" (warn, MIRROR1 D11); **RC23** a route whose FASTEST bound passes 57 s (error) /
  whose SLOWEST does (warn); **RC24** a drive leg starting more than 15° off its bearing on a tank (warn, "turn first",
  G3a R.5's tank warnings) *[R6] — now: with "turn to face it first" switched OFF, an ERROR above the robot's own
  `tankSteer.maxSteer` (15°, from its description; the library's `tank moveTo not steered … turn first: face()`), a
  WARNING from 0.5° up to it ("it may stop short of the point — PREVIEW shows it"; re-check P8: on the host test plant
  a 24 in leg 10° off timed out, 8° arrived); one click turns it back on. Twin: P8's runs as goldens;*
- **RC25** *[R1]* a position move — `move`, `startMove` or a trigger's `startMove` — on an arm or lift homed by a
  STALL or a SWITCH with no FINISHED home before it on every path (on both robots' lists): an ERROR. A finished home
  is `r.home(m, t)`, or `r.startHome(m, t)` followed by `r.waitUntilDone(m, t)` with no other command on that mechanism
  between (a new command REPLACES a running home, `service.hpp` :399-402). A switch-homed one is included although
  it zeroes at power-up when its switch is already pressed: the page cannot know that, the host test plant always
  presses it (re-check P6 (3)), and a home on a pressed switch finishes at once (P6 (4): `reached in 0.00s`). A
  never-homed one cannot move to a level at all. A home whose failure is set to "carry on" puts a WARNING on the move
  after it. Fixes, one click each: "home it first" (`r.home`) or "home it while the first leg drives" (`startHome`
  before it, `waitUntilDone` before the move) — the time limit left EMPTY for the student. Twin: GL18;
- **RC26** *[R6]* `checkCurve`'s refusals, from the PLANNED start pose: **StartBehind** — the first point more than
  90° off the nose (the back, reversing): an ERROR with "turn to face the first point first?" (inserts `r.face` /
  `r.faceAway`); **Invalid** — a headed first point within 1 in of where the robot is; a point within 1 in of the one
  before it; a headingless point whose neighbours are within 1 in of each other; any point whose direction is more
  than 90° from the chord arriving at it or the chord leaving it: ERRORS naming the point, in `checkCurve`'s words;
  **TightBend** — a WARNING. Twin: the same curves through `checkCurve` (CV1).

**The export's numbers carry units (node):** **UN1** every number in every golden export is followed by `_in`, `_deg`,
`_s` or `_volt`, except a trigger's percent and `maxPushes` (a grep over the goldens with a whitelist) *[R10] — and
exactly two more forms: a speed cap `N_in / 1_s`, and a turn cap `units::AngularVelocity{N * math::Angle::kPi /
180.0}` followed by its `// at most N deg/s` comment; a turn cap spelled through `_deg` FAILS UN1 (it wraps)*; **UN2** the UI
never shows radians; **UN3** headings exported in (−180, 180] (F3), 180 written `180_deg`.

**The field picture tells the truth (node + the build):** **FP1** five landmarks — the centre goal, the four Toggle
places — map from the picture's pixels to within 1 in of the C++ positions (`override_elements.hpp`); **FP2** red is at
the bottom, +x to the right (a pixel of the red wall's tape is at y < −60).

**The mirror (node):** **MI1** every line of `skills_mirror_vectors.txt` through `mirror.mjs` — equal (and never −0);
**MI2** the band hits on a route kept 4.21 in off the diagonal vs 2 in.

**The time bounds:** **TB1** on the host test plant every golden's time lies between its two bounds (G3a S42, with its
fixtures built to break each bound — a robot with an 8 s default timeout).

*(amended 2026-11-13 late) The helper is gone: HS1, HS2 and HS7 drop; HS3–HS5, HS9 and HS10 become FA1–FA4 on
`files.mjs` with an in-memory folder (A.8), HS9's scratch-tree suite run kept (Linux job); HS6 becomes a page test
(PREVIEW runs the model on screen); HS8 is re-pointed to the built site folder and one file. New: WG1–WG7 (A.2, A.8).
The paragraph below is the helper design's.*
**The helper (`route_maker.py self-test`):** **HS1** binds 127.0.0.1 only; **HS2** a write without the launch token, or
with another Origin, is refused; **HS3** export refuses a path outside `src/team/`, a symlink, a file with no markers,
a region with a hand edit (unless "overwrite"); **HS4** export touches ONLY the bytes between the markers (the rest of
the file byte-identical); **HS5** save is atomic (a crash between write and rename leaves the old file); **HS6** PREVIEW
builds the route ON SCREEN, not the file on disk (a route that differs from the file previews as the screen's); **HS7**
PREVIEW under the suite's bounds (a looping route ends in RPV1's no-pace stop); **HS8** no network: the built page has
no `http(s)://` outside the helper's own calls *[R11] — re-written: no URL except the XML namespace URIs (whitelist)
and 127.0.0.1; no `fetch` / `src` / `href` / `url(` naming another host; the page's Content-Security-Policy present and
exact*; **HS9** *[R4]* the export into a scratch copy of the tree keeps the suite and the src gate's builds green (with
GL0–GL19); **HS10** *[R2]* `/api/status` reports robot two's mirrored line present / missing on a scratch file of
each kind.

**RMP1-B (node + the C++ suite):**
- *[R15, R19] Corrected: `driver_inputs.hpp` DECLARES 29 `refused::` names (:141-206), not 30; two cannot come from
  the picture (a mechanism the robot does not have; a thirteenth button — each a hand-edited project file, refused by
  the loader, its own case). The agreement runs on FORCED configurations over the fixture robots x, y and z (every
  kind and homing method), so every one of the 29 is exercised although robot one and two will have only the cascade.*
- **CT1–CT31, the controls refusal agreement** — one case per refusal: ~~the 29 `refused::` names the picture can reach (of the 30 in `driver_inputs.hpp`; …)~~ *[R19] the 29 declared names, plus the re-zero button and a missing preset (by kind, R9) = 31* (the 12 "does two
  jobs", the hold's volts, a toggle on the wrong kind or a flywheel without its speed, `whileHeld` on a pneumatic,
  homing on a power mechanism or a never-homed one, two controls of one kind, a mechanism with no control and not
  autons-only, autons-only with a control, a must-be-homed preset with no way to home, a preset on a never-homed one,
  a never-homed hold without `unhomedVolts`, more than 24) plus the re-zero button and a missing preset: the page's
  `checkControls` refuses the configuration, AND the exporter's output for that configuration (forced past the check) is
  a generated compile-refusal fixture the build refuses with THAT name (`run_refusal.py`'s `// expect:`) — the C++ is
  the final authority; **CT21** every allowed control word on each robot compiles through `DriverFile` (golden); **CT22**
  the defaults reproduce robot one's and robot two's current `kControls` byte for byte *[R4] — and the `control words`
  and `feel` regions byte for byte as they stand when the markers are placed*; **CT-HS** *[R4]* the three regions
  exported into a scratch copy of each real driver file keep the suite (with §6.4's re-pointed T11, GM22, MECH-A5)
  and the src gate green; **CT23** *[R4]* the `control words` region equals what `exportControlWords` makes from the
  file's own `kControls` (GM22's bug, now caught by construction).
- **FE1 the feel golden** — `feel_curve.txt` (P2's 9 630 points as hex doubles, made by the C++): the C++ suite re-makes
  it and compares; node compares `feel.mjs` with `Object.is`; **FE2** `checkFeel` refuses exactly what `valid()` refuses
  at the boundaries (deadband −1e−12, 0, 0.999…, 1; expo −1e−12, 0, 1, 1+1e−12; NaN) — twin: a `static_assert` per row;
  **FE3** the exported `kFeel` compiles through `DriverFile` (golden) and, with D10, an out-of-range one is refused at
  build (`rm01`); **FE4** the slew readout: 4.0 /s → "0.25 s".

### 2.3 Mutations (each applied, RED observed, restored, logged — a fresh run each)
RM1 `atPercent` exported as a fraction (60 → 0.6) → GL-trigger red · RM2 the mirror heading as −h → MI1 red · RM3
`expoCurve` without `(1 − expo)` → FE1 red · RM4 the trigger check accepts 100.5 → RC4 red and its twin throws · RM5
the controls check misses a button used twice → CT agreement red · RM6 the picture's transform off by one tile → FP1 red ·
RM7 save drops `onFail` → RT1 red · RM8 undo skips a drag-end → RT2 red · RM9 the wall reset's carry on exported as
`required = true` → GL14 red · RM10 the wait exported as `r.pause(t)` → GL13 red · RM11 a skip target exported without
its `label` → GL12 red (the skip runs to the end) · RM12 the helper binds 0.0.0.0 → HS1 red · RM13 PREVIEW builds the
file on disk → HS6 red · RM14 the square check uses the LEFT sensor's direction for RIGHT → RC11 red · RM15 the
off-field check ignores the square's half-width → RC1 red · RM16 a newer schema loaded silently → RT4 red · RM17 the
slew readout per tick, not per second → FE4 red · RM18 a curve's planned end pose taken from its FIRST point → RC11's
curve case red · RM19 the definitely-started check ignores skips → RC17 red (G3a M53) · RM20 the export writes past the
END marker → HS4 red (G3a M44) · *[R1]* RM21 RC25 counts a `startHome` still running as finished → RC25's
replaced-home case red · *[R5]* RM22 the aligner shifts by one after a skip → SM1 red · *[R6]* RM23 `curve.mjs` drops
the 90° chord rule → CV1 red · *[R6]* RM24 turn-first's face exported AFTER the drive → GL1's turn-first case red
(the leg times out) · *[R13]* RM25 the path checks read robot one's list only → GL16b red · *[R10]* RM26 a turn cap
exported as `(N_deg).radians()` → UN1 and GL19 red. *(amended 2026-11-13 late) RM12 (the helper's bind) drops; RM13
becomes "PREVIEW runs the last export, not the model on screen" → the page test red; RM22 becomes "a page step's span
shifted by one after a skip" → SM1 red, on the runner; new RM27–RM32 (A.8).*

### 2.4 Definition of done — every run
`python3 tools/suite_run.py green` green (the golden test inside it); `node --test tools/route_maker/test/` green;
`python3 tools/route_maker/build.py --check` clean; `ctest -R compile_refusal --no-tests=error` all pass (B adds
`rm_*`); `route_maker.py self-test` and `route_preview.py self-test` OK; the API doc gates, `doc_staleness_audit`,
`briefing_status` clean; the src gate — `git diff -- src/ Makefile` EMPTY (B: only D10's line *[R17] and, if ROB1-BT
did not place them, the driver files' marker lines — nothing between them changed*, then the src gate's
`check` PASS on every build); *[R4]* HS9 (and B's CT-HS) green; the ARM header gate unchanged (no `include/` change); the mutations RED; the prose review
(rule 4) logged; the screenshots (§2.5) in the run's report; nothing committed. *(amended 2026-11-13 late) For the
web app: `build.py --check` and `route_maker.py self-test` drop; ADD `python3 tools/route_maker/build.py --out
build/route_maker_site` (the site folder and the one file made); from A2e the module built with the pinned emsdk and
WG1–WG3 + WG7 under Node; the browser tests in Playwright's Firefox on this laptop (A.5); and THE BAR — **the CI run's
Linux AND Windows browser jobs green, READ by the coordinator** (a run needs a push, which is the coordinator's). The
src gate reads `git diff -- src/ Makefile include/` EMPTY for A1, A2e, A2u and A1b (§7).*

### 2.5 UX acceptance — not only correctness
1. **The screenshot path, FIRST.** Before any UI work the run makes one headless screenshot of a test page work and logs
   how (the snap Firefox's `--screenshot`, its geckodriver over WebDriver as in `probes/p3_shot.py`, with a profile
   under the worktree's `build/`; if the agent sandbox blocks the snap, the run says so in the first hour and the
   coordinator captures with the same script outside it). It never leaves a browser or driver process behind (P3's
   stray). Every capture under `(ulimit -v …; timeout …)` — if Firefox cannot start under 4 GiB, the run logs the cap it
   needed and asks the coordinator, never raises it silently. *[R18] The sandbox has failed this twice (P3; the
   checker's `check/ff.log`), and P3's WebDriver run left TWO strays (geckodriver pid 3381105, headless Firefox pid
   3381195). So the path is asked of the team lead BEFORE A1 launches (D16); the first-hour proof stays, and if it
   fails the run STOPS AND ASKS — it does not build the UI blind. Every driver session is ended with
   `DELETE /session` and the driver process killed in a `finally`, whatever happened.* *(amended 2026-11-13 late)
   SETTLED — Amendment A.5's recipe (Playwright 1.64.1's OWN Firefox under `~/shulib-render-tmp/`, under the 4 GiB
   cap) worked in seconds here; the first-hour proof is running it on a test page. The snap Firefox and geckodriver
   are not used. Chromium/Edge screenshots come from the CI artifact (D18).*
2. **The screens captured, light and dark, at 1440 × 900 and at 1024 × 768:** first run (the empty field and the coach);
   a 6-step route with the inspector open on a trigger; an inline error (RC11's wall reset, the "turn first?" fix
   offered); *[R1] RC25's "home it first" on a cascade step; [R6] a Drive step with turn-first, and a curve with a
   refused point;* PREVIEW playing mid-route with the scrubber (A2); a failed step in the preview (A2); the mirrored pair with the band
   (when MIRROR1-A has merged); the export drawer; (B) the Controls tab with a popover; (B) the Driving feel with a curve
   at 0.6. In the run's report and the brief's re-check. *(amended 2026-11-13 late) Plus, from the Windows job's
   artifact, the first run and PREVIEW in Edge, and the old-browser refusal.*
3. **A UX / polish review lens** in each run's review, beside correctness: the tokens used everywhere (no stray colour or
   size), visual hierarchy (one primary action per area), every control labelled with hover help in plain words, empty
   and error states as §1.4, keyboard-only path through "make a route, preview, export", focus visible, contrast ≥ 4.5:1
   in both themes, nothing under 32 px to hit, motion ≤ 200 ms and none under reduced motion, PREVIEW's wait shown
   (progress, never a frozen page), the words a non-coder reads (no "Routine", no "precondition" in the page's own
   sentences — the library's quoted words sit beside a plain one), no "honest" disclaimers.
4. **The usability task — a NON-CODER, unaided:** the coordinator puts the builder (or another non-coding team member) in
   front of the app at the next session with ONE card: *"Make a 3-step route: start on a red tile, drive somewhere,
   curve to somewhere else, and raise the cascade to a level. Preview it. Export it."* Pass: done in ≤ 10 minutes with
   no help and no terminal (*(amended 2026-11-13 late)* the builder opened from the docs site — on a team Windows
   laptop when one is at hand), the export compiles (the suite), and every place they hesitated
   is logged. Before that, the run's reviewer does the same task COLD, using only the page and its help, and logs every
   hesitation. Findings go to RMP1-C. *[R1] The card is unchanged on purpose: raising the cascade will meet RC25's
   "home it first" — finding and using that one-click fix UNAIDED is part of the pass (the time limit they type is
   theirs). The export must also RUN: through the suite's golden harness on robot x's fixture lift in place of the
   cascade (R8), and as a smoke on robot one. "Preview it" needs A2; with A1 alone the card ends at "Export it" and
   the preview is done by RPV1's command.*

---

## 3. Documents — in the same change as the code (pace rule 3), the prose review logged (rule 4), ≤ ~5 public each (rule 7)

**RMP1-A (public):** `docs/guide/22-the-route-maker.md` — NEW, the next free chapter after ROB1's 21: **"Make your first
route with the route maker"** first — starting it (one command; double-click without PREVIEW), placing the start,
three steps, PREVIEW, Export, where the code went and how to read it — **every step a real screenshot** (§2.5), the
field always the real picture; then a reference half (every step kind, the checks and what each means, the shortcuts,
the project file). No disclaimer preambles; limits stated where they apply ("the preview's rig has no side sensors: a
wall reset shows as refused there"). The guide README's row; the changelog (`## API 2.8`, "No API change — tools");
guide 16's preview section gains "or press PREVIEW in the route maker"; guide 14's list gains the route maker's limits
(the cut kinds, §5). Internal: RMP1-PROGRESS.md, `test/README.md`'s golden and node sections, `RELEASING.md` (the page's
`build.py --check`). *[R7] A1 writes guide 22 without PREVIEW; A2 adds the PREVIEW section and its screenshots.
[R1, R6] Guide 22 explains, with a picture each, why the page asks to home the cascade first and why a tank turns
to face its point first.* *(amended 2026-11-13 late) Guide 22's "starting it" becomes "opening it": the Route builder
tab, the one-file download for the event, which browsers (Chrome or Edge to write into the shulib folder; any updated
browser for the rest), connecting the folder once. NEW public page `docs/route-builder.md` — the tab: what it is,
*Open the route builder*, *Download for offline use*, the browsers, one screenshot. Internal: `RELEASING.md` gains "a
publish builds the route builder (`pages.yml`): open its page and press PREVIEW once"; `test/README.md` gains the
runner and WG1–WG7; `tools/route_maker/README.md` (for developers: the emsdk pin, building the module locally, the
`TMPDIR` and `NODE_OPTIONS` rules on this laptop). The "one command / double-click" wording goes everywhere.*

*[R1] For the coordinator, outside RMP1's files (ROB1-A's merge is staged on main now):* guide 21 :224 ("a route's
first line `r.startHome(cascade, 2_s)` homes it while the first leg drives") and `src/team/robot_one_autons.cpp`'s
banner (`r.startHome(cascade, 2_s); // home the cascade while the first leg drives`) are true only if NOTHING else
commands the cascade before the home ends — a trigger on that first leg cancels the home and the move is NOT_HOMED
(re-check P6 (1)). One line each: "…then `r.waitUntilDone(cascade, 2_s)` before the cascade's first move".

**RMP1-B (public):** guide 22 gains "Driver controls" and "Driving feel" (screenshots; the graph explained with the
drivers' own example — a small push slow, a full push fast); guide 18's driver-controls section links to it; the
changelog. With D10: guide 18's feel paragraph says the build refuses an out-of-range feel.

**RMP1-C:** guide 22's screenshots refreshed after the polish.

**DOCS3-A's beginner track:** `docs/start/04` (the skills route, written by hand) links to guide 22 ("or build it by
clicking"); DOCS3-A may move guide 22's first half into `docs/start/` under its stricter rules — its call (§6).

**The prose review's greps** (each run logs the hits it read): "simulat", "honest", "obviously", "just ", "simply",
"easy", "precondition" in the page's own words, "radian", every `http` in the page.

---

## 4. Hardware Assumptions — none expected

RMP1 makes no claim about a robot. The numbers the page shows are the robot files' (and their own HA rows: robot one's
OWED facts, HA-112's invented feel, HA-185's claw time) and RPV1's 15 in square (RPV1 D12). **One conditional:** if the
build finds the page must state a belief of its own (for example a default trigger tolerance), it stops and asks; the
next free number is **HA-261**. *[R1, R6, R16] Checked against the re-check's additions: the home fix leaves its
time limit EMPTY (no invented number); turn-first's ERROR line is the robot's own `tankSteer.maxSteer` from its
description, and its 0.5° warning edge is a display rounding, not a robot claim; P8's "10° times out, 8° arrives"
is quoted only as a HOST TEST PLANT fact on the suite's stand-in drive; D12's "fastest" shows nothing where the
robot states no speed. None of these is a belief about a robot.* *(amended 2026-11-13 late) The amendment adds none
either: its numbers (sizes, times, browser versions, WG2's ~~1e-6 in~~ *[W2] 1e-5 in*) are facts about the tool and
the host test plant, measured here or read from documentation.*

---

## 5. OUT, and explicitly rejected

- **A slow mode while a button is held** — not in the library (§0.2 item 9). The feel's curve is the library's answer
  to "a small push slow, a full push fast". A slow mode is a library chunk of its own (a `DriverLoopConfig` field and a
  control word) — after the 21st, if asked.
- **The F8 / VexBuilder export; RELATIVE starts; MATCH (30 s) routes; strafe; move to a pose; through points; sensor
  waits; flywheel steps; a custom `then` action** (D2 (c), D7) — additive later.
- **The checked Toggle step before AF1S-A2 merges** (correction 2) — greyed, never a fake.
- **A preview of a wall reset seating, or of the camera** — the rig has neither (correction 3); RPV1-B2 (after L6-B) and
  a host Toggle plant on the preview rig (AF1S's `sim/toggle_plant.hpp`) later.
- **Reading hand-written C++ back into the page** (D3 (c)).
- **Writing robot two's line** ~~(MIRROR1 D2 [R11]: the students')~~ *[R2] — PLUMBING by the MIRROR1 go: ROB1-BT
  writes it, or the coordinator at MIRROR1-A's merge; the page only checks it*; ~~**writing the driver file's header
  prose** (D9)~~ *[R4] — the header's button map is now GENERATED as G3a's `control words` region (D9); the rest of
  the header (the protections paragraph, the history) stays hand-written*.
- **Any route, map or feel written FOR the team** (G4) — the goldens and the guide's examples are placeholders, said so.
- **A WebAssembly library; a framework; an npm dependency; any network request** (D1 (d), D8). *[R7] The WebAssembly
  rejection is being re-opened by the coordinator's queued Windows / web-app amendment (NOTES 11-13) — its call.*
  *(amended 2026-11-13 late) Re-opened and now IN: the library compiled to WebAssembly IS the PREVIEW engine (D1 (e)).
  Still out: a framework, and any runtime dependency of the app (the TESTS have one pinned dev dependency,
  Playwright); the only request the app makes is the site form fetching its own `.wasm` from the same site.*
- *(amended 2026-11-13 late)* **Build, upload and debug** (the team lead's wish for the full panel) — after the 21st,
  its host decided then (A.6). **A local helper of any kind before the 21st. A service worker** (RMP1-C, if wanted).
  **Putting the app on shurobotics.com** (the team's server, not inspected — the static folder can be copied there
  later, A.3.6). **Safari in CI** (its floor stated, not proven). **Chromium on this laptop** (D18).
- **The panel's hosting of the page** (D7) — G3a, after the 21st.
- **Rejected: a "likely time" model** (D12 (b)); **animating the page's own plan as the robot** (D5 (c)).

---

## 6. Coordination — every in-flight run (read 2026-11-13 ~13:30–13:55; every worktree read only)

### 6.1 In flight, and what each means for RMP1
| Run | State (NOTES, 11-13) | Shared with RMP1 | Rule |
|---|---|---|---|
| **RPV1-A** (`rpv1-a`) | in FIX, stopped at 90 % usage | `test/route_preview_{run,render,model}.hpp`, `test/CMakeLists.txt` | RMP1-A's PREVIEW half starts after RPV1-A MERGES; its `--json` and "maker" row are additive on RPV1's files; RPV1's 40 cases stay green |
| **ROB1-INT** (`rob1-int`) | DONE, merging after the 15:50 reset | robot one's list and facts (read); `.github/workflows/ci.yml` (MM) | RMP1-A's robot-one description and goldens need it merged; the ci.yml step goes in after ROB1-INT's edit |
| **MIRROR1-A** (`mirror1-a`) | in REVIEW, stopped | the vectors file and `mirrored_pair_check.hpp` (read) | merged before RMP1-A's tip → the mirror (D6) in A; else greyed and finished in RMP1-C |
| **AF1S-A2** | queued (after AF1S-A1, L6-A2, VIS1-A2 — all on main) | none; the checked Toggle step reads its everyday form | the step lights up in whichever RMP1 run follows its merge |
| **ROB1-BT** | queued (waits for ROB1-INT's merge and the builder's word) | the shared skills-route file and its markers (D4); *[R17]* the driver files' markers (D9); *[R2]* robot two's mirrored line | ~~RMP1 writes only between markers ROB1-BT's starter places; before that, guide 22 shows the two marker lines to paste~~ *[R3] §6.3's CONTRACT goes into ROB1-BT's args (the coordinator, before either launches; its args today say "markers … in both autons files", `make_rob1bt.py` :16, :33, :42 — one region in the SHARED file instead). Before ROB1-BT merges: Copy only, no student pastes markers. ROB1-BT re-points robot one's autons pin (2 registrations → 3) and MECH-A5 (§6.4)* |
| *[R7]* **The RMP1 amendment** (Windows / a web app / PREVIEW in WebAssembly) | queued right after this re-check (NOTES 11-13); RMP1-A waits for it | D1, D5's helper, D8, the screenshots | its call; R1–R19's fixes are written to hold either way (the model is pure modules; `api.mjs` is the one transport; the goldens run in the suite) |
| *[R7]* **DOCS3 amendment, docs phase 1, ROB1-C** | queued after RMP1's brief (NOTES 11-13) | slots | RMP1-B in a slot before the 21st pushes them ~2 days — the team lead's call (D11) |
| **DOCS3-A** (+ its amendment) | queued | guide 22 ↔ `docs/start/04`; screenshots | the second to merge adds the cross-link |
| **G3a** (`g3a-p1`) | unmerged; after the 21st | `src/team/route_maker.json` (its future input) | at its re-plan: R1/R2 onto RMP1's model (D7); R.8/D17 corrected per MIRROR1 §6.6 |
| **The CI pin** | committed, push blocked (NOTES) *[W5] — STALE: pushed since; `origin/shulib-v2` = `a27e2bc`, CI run 43481975300 on it **green** (2026-11-13 18:22Z, 58 min)* | `ci.yml` | the node step lands on the pinned runner |

*(amended 2026-11-13 late) Changed rows: **RPV1-A** — A2e compiles RPV1's engine UNCHANGED into the module and runs
it natively in the suite: no `--json`, no "maker" row, no `--pair` (P-W1, P-W1f). A2e starts from RPV1-A's MERGE (its
preview files are still untracked in the worktree, on `d33d25d`, after the restart). **ROB1-INT** — `ci.yml` gains two
jobs, not one step. **The RMP1 amendment** — DONE (this). **ROB1-BT** — unchanged; §6.3's contract is what
`files.writeRegion` writes into. New: **the docs site** — `pages.yml` gains the builder's build steps (A.3.5; its
runner `ubuntu-latest` → `ubuntu-24.04`, a chore for the coordinator, as `ci.yml`'s pin), and every publish is a
release from `main` (D17). **GitHub's Windows runner** — A1's first CI run is the first proof on Windows (A.10); ~~the
CI pin's push is blocked today, so that push is the coordinator's to clear before A1's review~~ *[W5] the CI pin IS
pushed and CI is green again since 11-13 (`a27e2bc`); each RMP1 run's branch is pushed for CI before its merge (A.4,
D20). `pages.yml` is still on `ubuntu-latest` — pinned in A1's own commit (W14)*.*

### 6.2 The slots to the ~Nov 18 freeze (an honest plan, not a promise)
*(amended 2026-11-13 late) Superseded by Amendment A.7's runs and dates (planned on D15 as answered, with D19 open).
~~The slot to add: A2e beside A1 from ~Sun 11-15 or Mon 11-16 — it pushes AF1S-A2 or the DOCS3 amendment ~1 day (the
team lead's call).~~ *[W4] The slots, within the THREE-job rule (11-11): Sun–Mon stays A1 / ROB1-BT / AF1S-A2;
from ~Tue 11-17 slot 3 (RMP1-B's below — now after the 21st) takes **A2e** once AF1S-A2 has merged, and slot 1 takes
**A2u** at A1's merge (against a stand-in engine; it merges after A2e); slot 2 keeps the DOCS3 amendment / docs. A2e
beside A1 from Sunday would be a 4th job or push AF1S-A2 — library code under the 18th's freeze — toward it: not
recommended (A.7).* The tables below are the re-check's and the first print's.*
Measured pace (NOTES, MIRROR1 §6.1b): a ~3–4 builder-day run took ~5–8 hours of agent time over ~1½–2 calendar days.

*[R7] RE-DATED (the table below this one is the first print's, superseded). At that pace a builder-day is ~½ a
calendar day, and two usage stops hit on 11-13 alone (03:24, 13:03):*

| When (2026) | Slot 1 | Slot 2 | Slot 3 |
|---|---|---|---|
| Fri 11-13 evening | ROB1-A merge (staged, local) | RPV1-A fix → merge | MIRROR1-A review → merge; then the RMP1 amendment brief |
| Sat 11-14 — the team lead's go on D1–D16 | **RMP1-A1** | ROB1-BT (with §6.3's contract in its args) | AF1S-A2 / MIRROR1-A merging |
| Sun 11-15 – Mon 11-16 | RMP1-A1 → review → fix | ROB1-BT → merge | AF1S-A2 |
| **Tue 11-17 – Wed 11-18** | **RMP1-A1 merged** → RMP1-A2 | DOCS3 amendment / docs phase 1 | RMP1-B (if the team lead puts it before the docs) |
| **~Nov 18 freeze** (D15: the students' regions stay open) | | | |
| **Thu 11-19 – Fri 11-20** | **RMP1-A2 merged** | docs | RMP1-B merged, or after the 21st |

*A launch a day later, or a full-day usage stop, moves every row a day: A1 then merges ~Nov 18–19 — at the freeze,
which is why D15 matters.*

*First print (superseded):*

| When (2026) | Slot 1 | Slot 2 | Slot 3 |
|---|---|---|---|
| Fri 11-13 after the 15:50 reset | ROB1-INT merge (local) | RPV1-A fix → review → merge | MIRROR1-A review → merge |
| Fri evening — **the team lead's go on D1–D12** | **RMP1-A** (model and page half first) | RPV1-A merging | MIRROR1-A merging |
| Sat 11-14 | RMP1-A (PREVIEW half once RPV1-A has merged) | ROB1-BT (if the builder has said robot two is rebuilt) | **RMP1-B** |
| Sun 11-15 | RMP1-A | ROB1-BT → AF1S-A2 | RMP1-B → review |
| Mon 11-16 | RMP1-A review → **merged** | AF1S-A2 | RMP1-B merged after A |
| Tue–Wed 11-17–18 | DOCS3-A1 | AF1S-A2 → merged; **freeze ~Nov 18** | RMP1-C (after the usability session) |

**What slips first:** RMP1-B's D10 line (to after the 21st) and RMP1-C; never RMP1-A, which is the team lead's top
priority after robot one. If RPV1-A's merge is late, RMP1-A merges its page with PREVIEW wired to the stub and lights it
the day RPV1-A lands (a small follow-up, not a new run). *[R7] Now: A1 never waits for RPV1-A (A1 has no PREVIEW);
A2 does.*

### 6.3 THE CONTRACT with ROB1-BT — the shared skills-route file *[R3] new; the coordinator copies it into ROB1-BT's args*
> **Amended by RMP1-C (2026-11-17, named routes):** the region holds EVERY route of the project — each its own
> `<stem>Steps` and `<stem>Route()` in the region's anonymous namespace — and ends with ONE list, `const TeamRoute
> kTeamRoutes[]` (`{&<stem>Route, Mirror::…, Together::…}` per route, in the brain menu's order). The markers still name
> the region "skills". Items 1, 3 and 4 below are the amended contract; GL0 (`gl0_empty.cpp`, its hash pinned in
> `export.test.mjs`) was re-made with it.

1. **One file pair, ROB1-BT's:** `src/team/skills_route.hpp` declares the list's types (`Mirror`, `Together`,
   `TeamRoute`, `TeamRobot`), `teamRoutes()`, `registerRoutes()` and `addTeamRoutes()` (RMP1-C; it declared
   `skillsRoute()` before); `src/team/skills_route.cpp` defines them below the END line. It is compiled into BOTH robots' packages (MIRROR1 §6.3; gate step 8 as
   MIRROR1 says).
2. **Outside the markers, ROB1-BT's file carries** the includes the route maker's code needs — `robots/mechanism_table.hpp`
   (and through it the facts ledger), `shulib/chassis/routine.hpp`, `shulib/competition/auton_registry.hpp`,
   `shulib/competition/auton_wall_reset.hpp`, `shulib/localization/override_walls.hpp`,
   `shulib/sequence/run_guard.hpp`, `shulib/units/literals.hpp`, `shulib/math/angle.hpp`, and
   `shulib/competition/field_mirror.hpp` once MIRROR1-A has merged (and AF1S-A2's header when it lands) — the
   namespace the region sits in, `using namespace units::literals;`, and the lines that make `cascade`,
   `facts::lv(i)` and every mechanism the route names resolve to the **BUILDING robot's own** declarations (robot
   two's package: robot two's cascade and ledger). The route maker writes no `#include` and no `using` line (landmine 10).
3. **Inside, exactly one region**, its two marker lines each alone on their line, ROB1-BT's starter holding the
   route maker's output for an EMPTY route, byte for byte (compiled with `-Werror` and run: `steps=0 ok=1`, re-check
   P7; GL0 keeps the exporter equal to it):
   ```cpp
   // ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
   // Made in the route maker from src/team/route_maker.json. Change the route there and export again:
   // the route maker will not overwrite a hand edit between BEGIN and END.
   // To make this hand-written code instead, delete the BEGIN and END lines.
   namespace {
   // The route "skills", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
   // the field's diagonal (the list at the bottom says so).
   chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
       chassis::Routine r{ctx, "skills"};
       // NO STEPS YET: the team's route is made in the route maker and written here.
       return r.result();
   }

   // Its entry.
   competition::AutonEntry skillsRoute() {
       competition::AutonEntry e{};
       e.name = "skills";                                // what the brain's menu shows
       e.kind = competition::AutonKind::Skills;          // the 60 s skills run
       e.routine = &skillsSteps;                         // the steps above
       e.start = competition::StartPose::relative();     // NO START YET: the route maker writes robot one's start
       e.endActionAt = 57_s;  // still running at 57 s? it is stopped, and the end action (a brake) starts
       e.hardStopAt = 59_s;   // everything stops, 1 s before the run's 60 s
       return e;
   }
   }  // namespace

   // EVERY ROUTE, in the order the brain's menu lists them (addTeamRoutes, below the END line, registers them).
   // Robot one registers each as written; robot two MIRRORS one marked Mirror::ForRobotTwo. Together::Yes: both
   // robots run it at the SAME time, so the host suite checks they never come too close.
   const TeamRoute kTeamRoutes[] = {
       {&skillsRoute, Mirror::ForRobotTwo, Together::NotAnswered},  // "skills"
   };
   // ═══ ROUTE MAKER — route "skills" — END ═════════════════════════════════════════════════
   ```
   No route content (G4): no step, no field point; the relative start and the 57 / 59 s schedule are the plumbing a
   registration needs, replaced at the students' first export.
4. **The registrations (plumbing):** since RMP1-C one line in each autons file — robot one's
   `addTeamRoutes(registry, TeamRobot::One);` (every route as written), robot two's `addTeamRoutes(registry,
   TeamRobot::Two);` (each route marked `Mirror::ForRobotTwo` mirrored by `kSkillsMirror`, the others as written); the
   export page checks both (`region.mjs` `hasTeamRoutesLine`). (Before RMP1-C: `registry.add(skillsRoute());` and
   its mirrored twin.)
5. **The driver files (D9, R17):** in both robots' driver files, the `control words` and `feel` markers around the
   existing header map and THE FEEL block, content unchanged; each marker alone on its line.
6. **Nothing else in `src/team/` is written by the route maker.** Its hand-edit check keeps the last-written hash in
   `route_maker.json`, never in a source file.

### 6.4 The pin tests when the team takes a file over *[R4] new*
Today's tests pin the STARTER content, so the first change the students make — by hand or by the page — turns them
red. Each is re-pointed to the rule it protects, by the run named:

| Test | Pins today | Re-pointed to | By |
|---|---|---|---|
| `robot_one_driver_test.cpp` T11 (:161-197) | the default map: L1/L2, the d-pad levels, B, Y R1 R2 X A free | the RULES: the re-zero button free (already a build refusal, `driver_file.hpp` :81); every level named through the ledger (`facts::lv(i).name`, never a quoted string); the map equal to the project file's when one exists | RMP1-B |
| `robot_one_driver_test.cpp` GM22 (:199-226) | the banner's words and the exact `preset(...)` lines | the `control words` region equals what the exporter makes from the file's own `kControls` (CT23) — the banner can no longer disagree with the file | RMP1-B |
| `robot_two_driver_test.cpp` MECH-A5 (:1071-1110) | the region = the heading + the claw fixture's statement | the same region RULE (the heading, the statement, nothing else; markers alone on their lines), robot two's new statement | ROB1-BT (it rewrites robot two's driver file) |
| `robot_one_autons_test.cpp` (:162-166) | `registry.add(` exactly 2 | 3 (the shared skills route registered) — and the shared file's region checked by GL0 / HS9 | ROB1-BT |

---

## 7. Checks — the coordinator re-runs every one after each run (never the report's word)
*(amended 2026-11-13 late) For the web app, these replace `build.py --check`, `route_maker.py self-test` and the
grep over the committed page; the rest of the block below stands, with "the helper" read as the page's `files.mjs`
(HS9's export into a scratch copy of the tree, the suite there) and the end-to-end run done in the page and the CI
browser jobs:*
```bash
export NODE_OPTIONS=--disable-wasm-trap-handler         # every capped node / npx (A.1)
(ulimit -v 4194304; timeout 300 node --test tools/route_maker/test/)
python3 tools/route_maker/build.py --out build/route_maker_site       # the site folder + the one file
# from A2e: the module, with the ONE shared pinned emsdk named in the run's prompt; temporary files under build/
source <emsdk>/emsdk_env.sh && mkdir -p build/tmp && export TMPDIR=$PWD/build/tmp
(ulimit -v 4194304; timeout 900 emcmake cmake -S tools/route_maker/wasm -B build/wasm -DCMAKE_BUILD_TYPE=Release)
(ulimit -v 4194304; timeout 900 cmake --build build/wasm)
(ulimit -v 4194304; timeout 600 node --test tools/route_maker/test/wasm/)   # WG1-WG3, WG7
# the browser tests in Playwright's Firefox (A.5's recipe: PLAYWRIGHT_BROWSERS_PATH under ~/shulib-render-tmp)
(ulimit -v 4194304; timeout 900 env ROUTE_MAKER_BROWSER=firefox node --test tools/route_maker/test/page/)
# no network: the built site folder and the one file (the probe's glue and one file had no URL at all)
! grep -rn -o -E 'https?://[^"'"'"' )>]*' build/route_maker_site \
    | grep -v -E 'https?://www\.w3\.org/(2000/svg|1999/xlink|1999/xhtml)'
grep -l "wasm-unsafe-eval" build/route_maker_site/*.html        # the CSP allows the preview (WG5)
# THE BAR: the CI run's route-builder and route-builder-browsers jobs, ubuntu-24.04 AND windows-latest -- READ, green
git diff --stat -- src/ Makefile include/      # EMPTY for A1, A2e, A2u, A1b
# [W1-W16] added by the amendment's re-check (A.12):
test -f build/route_maker_site/app.html && ! test -e build/route_maker_site/index.html   # W3: the app is app.html
grep -c "script-src 'self' 'unsafe-inline' 'wasm-unsafe-eval'" build/route_maker_site/app.html   # 1 (W1: SITE policy)
grep -c 'rel="icon" href="data:,"' build/route_maker_site/*.html                                  # each 1 (W1)
! grep -rl -a -E '/home/|/Users/|C:\\|/runner/|\.claude' build/route_maker_site build/wasm/*.wasm   # RG1 (W10);
#   the one file's base64 module is checked DECODED by build.py's own RG1 (a plain grep cannot see inside it)
git check-attr eol -- tools/route_maker/test/x.test.mjs test/goldens/route_maker/x.cpp     # eol: lf (W11)
# from A2e: the native program beside the module; WG2 prints its measured maximum beside the bound -- READ it (W2, W9)
(ulimit -v 4194304; timeout 900 cmake --build build/test --target shulib_route_maker)
(ulimit -v 4194304; timeout 900 node --test tools/route_maker/test/wasm/)   # WG1-WG3, WG2s, WG7, WG8
# a library-touching chunk from A2e's merge on: the module build is its second compiler gate (W14)
# the CI run was on the run's PUSHED BRANCH, before the merge (W5, D20); the ARM row and the pinned-Chromium row green (W15)
```
```bash
# the suite, bounded (the golden test is inside it)
python3 tools/suite_run.py green
# the page's model, headless, and the built page's freshness
(ulimit -v 4194304; timeout 300 node --test tools/route_maker/test/)
python3 tools/route_maker/build.py --check
# the helper and the preview launcher
python3 tools/route_maker.py self-test
python3 tools/route_preview.py self-test
# the compile refusals (B adds rm_*)
(ulimit -v 4194304; timeout 900 ctest --test-dir build/test -R compile_refusal --no-tests=error)
# no network in the page  [R11: the first print's grep tripped on the SVG namespace URI]
! grep -n -o -E 'https?://[^"'"'"' )>]*' tools/route_maker/route_maker.html \
    | grep -v -E 'https?://(127\.0\.0\.1|www\.w3\.org/(2000/svg|1999/xlink|1999/xhtml))'
grep -c 'http-equiv="Content-Security-Policy"' tools/route_maker/route_maker.html   # 1
# the src gate: A changes nothing in src/; B only D10's line (and the driver markers if ROB1-BT did not place them)
git diff --stat -- src/ Makefile
python3 tools/src_build_gate.py check            # B with D10 or markers
# [R4] the exports in the real files: the helper writes every golden into a scratch copy of the tree; the suite there
#      (HS9; B: CT-HS) -- read its log, green
# the doc gates
python3 tools/api_doc_tool.py check-coverage && python3 tools/api_doc_tool.py check-fresh \
  && python3 tools/api_doc_tool.py check-examples && python3 tools/doc_staleness_audit.py
# end to end, through the real helper on robot one (after ROB1-INT): describe, preview the D4 golden, export to a
# scratch copy of the skills-route file, and the suite on the result
# a mutation subset: A1 — RM1, RM4, RM9, RM10, RM14, RM15, RM21, RM23, RM24, RM26; A2 — RM13, RM22, RM25;
#                    B — RM3, RM5, RM17
#   [W] amended: A1 also RM29 (one-file), RM30, RM33, RM34, RM36; A2e RM27, RM28, RM32, RM35, RM37; A2u RM31
# [R1, R6] re-run the re-check probes on the merged tree: P6 (the home rule) and P8 (turn-first) as goldens GL18, GL1
# the screenshots: open every one the report lists
```

---

## 8. Landmines

1. **A page that draws what the library will not do.** The only solid path, the only moving robot and the only time
   called "the preview's" come from the library's run (RPV1's record). The planned path is dashed and labelled; a curve
   is drawn ~~through its points, never as a smoothed shape the follower might not take~~ *[R6] as the LIBRARY's own
   curve geometry (`CurvePath`, ported and pinned by CV1) — the path the follower is told to follow, not a guess at the
   robot; where the robot actually went is PREVIEW's*; a trigger's tick says "fires
   only if the robot gets there". The time bounds are two labelled numbers, never one prediction. *[R6] And a planned
   pose is only drawn AT a point if the library would get there: a tank leg off its line without turn-first is drawn
   amber or red (RC24), a refused curve red (RC26) — P8 showed a 10° leg stopping short.*
2. **Units.** Inches, degrees, seconds, volts and percent at every human edge (the page, the project file, the export's
   literals); radians never shown; a trigger's percent is 0–100 as `atPercent` takes it (RM1); "after N°" is degrees.
   One conversion function per unit in the model, used everywhere.
3. **Heading conventions.** 0° faces +x — toward the RIGHT wall —, angles grow counterclockwise, (−180, 180], an exact
   antipode +180 (F3); the picture red-at-bottom (FP2); the robot's FRONT is the calibration record's front; the mirror
   heading is −90° − h, not −h (MIRROR1 correction 1). A student's mental "north" is the blue wall = 90°: the start's
   heading handle shows the number AND an arrow.
4. **A step the library refuses at run time.** A precondition throws and ends the auton (`match_runner.hpp`; G3a R.5
   C3). Every one the page can predict from the route and the robot's description is an ERROR in the page (RC4–RC21);
   the rest are run by the golden test (GL12, S53's "nothing generated throws"). *[R1, R13] Two that do not throw but
   stop or crash a run: a move before the mechanism is homed (NOT_HOMED stops the route — RC25), and a wait that only
   one robot's path started (THROWS on the other — RC17 on both robots' lists).*
5. **G4.** No "suggest a route", no template of a whole route, no auto-fill of a level; ~~robot two's line and~~ the
   driver map ~~are~~ *is* the students'; the goldens and the guide's example are placeholders and say so in a comment.
   *[R2] Robot two's line is plumbing by the MIRROR1 go. [R1, R6] The one-click fixes ("home it first", "turn to face
   it first") add a step the student SEES and can remove, the library's own requirement made visible; the home's time
   limit is left for the student to type.*
6. **The preview's rig is not the robot.** No walls (the field check stands in), no side sensors (every reset refused),
   no camera, an ideal plant, robot one's mechanism times from PLACEHOLDER facts (hatched, RPV1's PROVISIONAL rule; the
   cascade's gearing is owed — ROB1-INT's Tune finding: on the placeholder gearing a floor-to-L3 move times out). The
   page shows RPV1's own caption. *[R1, R8] Two more: the fakes press a switch-homed mechanism's switch at start, so
   the preview never shows a missing home on one (P6 (3)) — the page's RC25 must; and robot one's cascade moves time
   out on the placeholder gearing (P6 (2)), so no golden asserts robot one's behaviour.*
7. **Robot one's facts are OWED.** Level names are placeholders (`floor`, `L1`, `L2`, `L3`); the export names levels
   through `facts::lv(i)` so a renamed level never breaks a route; the robot description re-read after a facts edit.
8. **The snap Firefox** (P3) — screenshots are an acceptance item, so their path is proved first (§2.5.1).
9. **The helper writes into `src/`.** 127.0.0.1 only, a launch token, the Origin checked, only between existing markers
   in `src/team/`, never a symlink, an atomic save with a backup, a hand edit inside the markers never overwritten
   silently (HS1–HS5).
10. **The shared route's mechanism names.** The shared skills-route file names robot one's declarations; robot two's
    package compiles the same function — whether robot two's list is reached by the same names is ROB1-BT's layout
    (MIRROR1 §6.3); the export writes no `using namespace` line (outside the markers, the file's own). *[R3] Now part
    of the contract (§6.3 item 2): outside the markers the names resolve to the BUILDING robot's own declarations.*
11. **The mirror's limits.** The 6 in check is at the PLANNED timing only (MIRROR1 D11 (a) 3); the page says so beside
    the band; the suite's pair check is the judge.
12. **Float exactness.** The feel golden is bit-exact on x86-64 without fused multiply-add (P2). The suite builds without
    `-march=native`; a future flag that allows contraction would break FE1 loudly — keep it, never loosen it to a
    tolerance.
13. **Names.** ≤ 29 printable ASCII without '%', unique, not "NONE"; ≤ 25 when mirrored ("<name> SKILLS MIR" on one
    36-column line, MIRROR1 D8).
14. **The page and the library drifting.** The refusal words come from the headers (`refusals.json`, extracted at
    build); the goldens are compiled and run by the suite; `build.py --check` fails a stale page; the API version is
    stamped in the export and the page refuses a robot description of another major.
15. **One file, two ways to open it.** Opened by double-click there is no helper: the page must say which features are
    off, once, and never fail silently.
16. **The page's size.** The field picture compressed (≤ 250 KB), the whole file under ~1.5 MB, so it opens instantly
    on a team laptop.

*(amended 2026-11-13 late) 8 is settled (A.5: no snap, no geckodriver). 9 is now `files.mjs` — the BROWSER writes into
`src/team/`, under the same rules (FA1–FA4). 14's `build.py --check` is gone: the builder is built from the same commit
as the library at every CI run and every publish, and the page shows that commit. 15 becomes THREE ways to open it —
the site, the one file over `file://`, and browsers with or without folder writing — each says once which features it
has. 16: the one file is ~1.2–1.4 MB with the engine inside. New:*

17. **Native and WebAssembly poses differ in the last bits** — the maths library (A.1, P-W1b). Never compare a native
    pose to a WebAssembly pose byte for byte (WG2: within ~~1e-6 in / 1e-7 rad~~ *[W2] 1e-5 in / 1e-6 rad, from a
    3,000-route sweep whose maximum was 3.5e-7 in / 2.5e-8 rad*), and never loosen the STEP comparison
    (exact): a step that ends one pace apart is a real difference. *[W2] The exact rule is empirical: a golden or seed
    that the run SHOWS sits on a threshold or a printed-decimal boundary is replaced, logged by name — never loosened.*
18. **The published builder lags the tree** — it knows the robots as of its last publish (A.3.4, D17); the page shows
    its commit and date; a robot fact typed today reaches it at the next publish.
19. **Windows line endings** — Git for Windows checks files out with CRLF and this repository has no `.gitattributes`;
    the export keeps each file's own line ending and final newline (FA3, RM30). *[W11] The builder's OWN test files
    are pinned LF by two scoped `.gitattributes`, or every byte-for-byte golden compare fails on `windows-latest`;
    never a repository-wide `.gitattributes` (it would change how the students' files check out).*
20. **Node under the 4 GiB cap** — Node 20 needs `--disable-wasm-trap-handler`, and the module links with
    `-sMAXIMUM_MEMORY=256MB` so a second instance fits (P-W1f).
21. **The CSP must allow `'wasm-unsafe-eval'`** — without it the preview is blocked (P-W2); WG5 pins it. *[W1] And
    the SITE form needs `'self'` in `script-src` and `worker-src` (its module and Worker are separate files — without
    it nothing runs); the ONE FILE must not need it. Two policies, both pinned; a `data:` favicon in both.*
22. **`std::_Exit` inside the module ends only the call** (P-W1f) — the Worker re-creates the instance after any
    non-zero return, and terminates a Worker that does not answer in 20 s.
23. **`file://` may not count as secure** — writing into the folder from the ONE FILE may be refused in Edge (A.10);
    it then offers Copy and downloads, and says so once.

*[W1–W16] Added by the amendment's re-check (A.12):*

24. **C++ call arguments have no order** (W8) — GCC evaluates them right to left here, Clang and Emscripten left to
    right; `driveTo(L{num(in)}, L{num(in)})` swaps x and y natively ONLY, so WG1 (all WebAssembly) cannot see it. The
    runner never parses inside a call's argument list; every value is read into a named variable first. WG8 (every
    pair of numbers different) runs in WG2.
25. **PREVIEW must run the numbers the export prints** (W7) — one formatter (`numbers.mjs`) for the export and the
    line format, every number stored already rounded (≤ 3 decimals); NF1 pins it.
26. **mkdocs silently overwrites a static file with a rendered page** (W3) — `docs/route-builder.md` renders to
    `route-builder/index.html`, so the app is `app.html`; `--strict` does NOT warn; PG1 after `mkdocs build` is the
    only guard, because the browser jobs test `build.py`'s output.
27. **The module remembers where it was built** (W10) — `std::source_location` keeps file names; without
    `-ffile-prefix-map=<root>/=` a laptop-built copy carries the laptop's private path, and the base64 one file hides it
    from a plain grep. RG1 checks the decoded module.
28. **Two compilers judge the library from A2e on** (W14) — Emscripten's Clang with `-Werror` can refuse what GCC 13
    passes; a library chunk verified only with the GCC suite can turn the route-builder job red and block a publish.
    The module build is in every library-touching chunk's done list.
29. **The pushes are the Windows proof** (W5) — nothing runs on Windows until the run's branch is on GitHub; the merge
    waits for that CI run (~1 h), and the push needs D20's OK.
30. **Legacy Wasm exceptions are deprecated** (W6) — every Firefox instance logs a warning; the page tests allow
    exactly that one; a browser that drops them gets WG4's plain refusal, and CI's latest-browser rows see it first.

---

## 9. Summary

*(amended 2026-11-13 late)* **What, now:** a WEB APP on the docs site (a "Route builder" tab — Windows, Mac, Linux,
Chromebooks, nothing installed) plus a one-file download for the event, whose PREVIEW runs the REAL library compiled
to WebAssembly inside the page (~0.2 s for a 60 s route; the same steps, verdicts and times as the native build, the
poses to the last bits for a stated reason); saving and export through the browser (Chrome or Edge write into the
shulib folder, keeping its line endings; elsewhere download and Copy); proven on Windows AND Linux by CI on every
change, plus a 10-minute check on a team Windows laptop. **Evidence:** Amendment A.1 (P-W1a–f, P-W2–P-W4).
**Runs:** A1 (~5–6 + ~1 — the first usable builder, ~Tue 11-17 to Wed 11-18), A2e (~2½–3 + ~½, beside A1 if a slot
frees), A2u (~1½–2 + ~½ — PREVIEW in the page, ~Wed 11-18 to Fri 11-20), A1b, B, C — ~15–18½ + ~3½ in all (A.7).
*[W] After the amendment's re-check (A.12): A1 ~5¼–6¼ + ~1 (~Tue 11-17 to Wed 11-18); A2e ~3–3½ + ~½ in slot 3
from AF1S-A2's merge (~Tue 11-17 → ~Wed 11-18 evening to Thu 11-19); A2u ~1½–2 + ~½ from A1's merge → PREVIEW **~Thu
11-19 to Fri 11-20**; A1b most likely after the 21st; B, C after the 21st — ~15¾–19¼ + ~3½ in all; plus ~1 h of CI
per run and ~30–45 min per publish.*
**Waits for:** the team lead's go per run; D17 and D19 (D15 is answered: (a)) *[W5] and D20 (the pushes)*; the rest of "Waits for" below where it
still applies — the Windows amendment is done. The text below is the helper design's summary.

**What:** one offline app — `python3 tools/route_maker.py` opens it — where a non-coder builds the skills route on the
real field (the start, drives, turns, curves, cascade levels, triggers partway through a drive, wall resets, the
Toggle, waits, what to do when a step fails, robot one only), sees inline in the library's own words anything the
library would refuse, presses **PREVIEW** to watch the REAL library drive it on the host test plant with a timeline
scrubber (both robots when robot two runs it mirrored), and exports commented C++ into the marked place of the shared
skills-route file ~~plus robot two's one line to paste~~ *[R2] (robot two's one line is plumbing; the page checks it
is there) — homing the cascade before any move and, on a tank, turning to face each point first (R1, R6)*; a **Controls** tab (the V5 controller → the driver file's
controls, refused exactly where the build refuses) and a **Driving feel** section (a live stick-to-speed graph from the
library's own formula, bit-identical to the C++, → `kFeel`); everything saved in `src/team/route_maker.json`, the file
the panel reads after the 21st.

**Evidence:** P1 — a route in D4's FORM (not its listing, R19) compiled first time against main and ran 6 of 6 steps on
the host test plant in 7.05 s; P1b — robot one's cascade route, controls and feel compile (and so does an out-of-range
feel: D10); P2 — the page's feel formula equals the C++ at 9 630 points, bit for bit; P4 — PREVIEW's build-and-run is
~5 s; P3 — headless screenshots did NOT work from the sandbox. *[Re-check]* the checker's run of D4's first listing on
robot one — NOT_HOMED at step 2 (R1); P5 — the caps' unit spellings compile, and `_deg` wraps a turn cap; P6 — the home
rule (a `startHome` replaced by a trigger → NOT_HOMED; the corrected route 7/7 on robot x; the preview cannot see a
missing switch home); P7 — the empty region compiles and runs; P8 — a tank leg 10° off its line stops short, and
turn-first fixes it.

**Runs and honest dates:** *[R7] re-dated:* **RMP1-A1 ~5½–6½ builder-days + ~1** (merged ~Tue 11-17 to Wed 11-18);
**RMP1-A2 ~2½–3½ + ~½** (PREVIEW in the page, ~Thu 11-19 to Fri 11-20); **RMP1-B ~2½–3 + ~½** (~Nov 19–20, or after
the 21st); **RMP1-C ~1½–2 + ~½** (after the 21st). **In all ~12–15 builder-days + ~2½ review.** The tool changes no
robot program (D10's line and the driver markers excepted), but what it WRITES is the students' robot code in
`src/team/` — so whether those regions stay open after the ~Nov 18 freeze is D15. A full-day usage stop moves each date
a day. *(First print: A ~5–6 + ~1, ~Nov 16–17; B ~Nov 18–19; C ~Nov 19–20; in all ~8½–10½ — superseded.)*

**Waits for:** the team lead's answers to D1–D16, then a go per run; the coordinator's Windows / web-app amendment
(first); §6.3's contract in ROB1-BT's args BEFORE ROB1-BT launches; A1's write-to-file on ROB1-BT's merge (Copy
until then); A2 on RPV1-A's merge; the mirror on MIRROR1-A's; the checked Toggle step on AF1S-A2's; the usability
session on the builder's time; a screenshot path (D16).
