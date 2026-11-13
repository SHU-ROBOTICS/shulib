# VIS1, the AI Vision camera as library code — progress log (live)

- **Chunk:** VIS1 — the V5 AI Vision camera's colours and objects as shulib library code (brief:
  `VIS1-ai-vision-colours.md`, being copied into `docs/internal/chunks/` by the coordinator with the answers
  block; the executor reads the coordinator's copy, `.claude/coordinator/briefs/VIS1-ai-vision-colours.md`, the
  same text). Three runs — VIS1-A1, VIS1-A2, VIS1-B — each its own go (the brief's D1, §2.1).
- **Executor model:** opus.
- Appended as the work goes, a heading per run. Every time in a section header is a `date` reading taken when the
  section was written. **Nothing in this chunk has run on a robot, and no AI Vision evidence exists yet.** Say "host
  test plant" (and "the host camera model"), never "simulator".

---

## VIS1-A1 — the camera as a device

- **Started:** Sun Oct 18 10:46:51 PM EDT 2026 (`date`).
- **Tree at start:** `ceb17c0` (CAL1 Part A1 landed there; MECH-A1 `fd36e07`, SER1 `2506067`, MECH-A2 `98392b2`
  all merged below it), clean, in the git worktree `.claude/worktrees/vis1-a1` (branch `vis1-a1`) — NOT the main
  working tree.
- **Go:** the team lead, 2026-10-18: "Accept all (Recommended)" — D1–D11 ALL decided as the brief recommends (D6
  as re-checked: a read reporting 24 or more is "possibly full" and never evidence; D9 as re-checked: the lens about
  10–11 in up and level — a mount fact, not code). The coordinator's workflow run launched VIS1-A1 ONLY. Do not
  commit; no `pros` command; no `/dev/ttyACM*`. Mutation backups and probes under
  `.claude/coordinator/scratch/vis1-a1/` (never `/tmp`, never the repository root). Every test binary runs as
  `(ulimit -v 4194304; timeout 600 …)` (the 2026-10-15 memory rule).
- **Scope:** §2.1's VIS1-A1 row — §1.1 the frame seam, §1.2 `ProsAiVision`, §1.3 its fake, §1.4 the shim, §1.5
  `DeviceKind::AiVision`, §1.6 the provider's `aiVision()` in all four implementers, §1.7 `CameraConfig` + `valid()`,
  §1.8 the Robot builds it, §1.9 the geometry, §1.10 the season's data, §1.11 the host camera model; §2.2's VIS1-A1
  tests and mutations; §3.1's VIS1-A1 documents; HA-192..HA-203 registered. NOTHING of VIS1-A2 (tracker, piece
  finder, feed) or VIS1-B (robot two's camera entry, boot source, station 12, worksheet) — except the ONE-word test
  change in `test/boot_source_test.cpp` :159 (`Vision` → `AiVision`, re-check T5). No `src/` or `tools/` file.

### 22:46 — start
- Read `docs/internal/RESUMING.md` (the chunk loop, the standards, pace rules 1–8) and the brief in full: its header,
  "Re-check — amendments" (T1–T10, S1–S10, R7 — these OVERRIDE the body), the Decisions D1–D11, the plain words,
  §0–§9 and the landmines. This log created before any code.

### 22:47–23:20 — required reading, the prototypes against the base, the binary and the manual re-checked *(`date` 23:20)*
- **Read in the coordinator's scratch** (`scratch/brief-VIS1/v6/`, the newest drafts): `proposed/include/shulib/
  {hal/vision_frames.hpp, hal/pros/ai_vision.hpp, hal/fake/fake_vision_frames.hpp, config/camera_config.hpp,
  vision/camera_geometry.hpp, vision/override_elements.hpp, sim/camera_plant.hpp}` and `proposed/shim/pros/ai_vision.h`
  (A1's slice; `vision/{toggle_tracker,piece_finder,camera_feed}.hpp` are A2's and are NOT copied). `overlay/`: each
  hook diffed against this base (`ceb17c0`) — `hal/device_registry.hpp` (+2 lines), `hal/pros/device_registry.hpp`
  :128 (one word), `hal/pros/device_provider.hpp` (one `foundWords` case after :620), `config/robot_config.hpp` (the
  include, `SensorsConfig::camera`, `PortUse::Camera`, its `portUseWord` case), `test/boot_source_test.cpp` :159,
  `test/mechanism_devices_test.cpp` :66, :80 — every overlay hunk lands on the line the brief cites at this base (no
  drift since `2506067`: MECH-A2 and CAL1-A1 touched none of these lines). `overlay/src/robots/chassis_table.hpp`
  is VIS1-B's (`src/`) and is NOT applied — delta A1-D1 below.
- **Read in the tree at `ceb17c0`:** `hal/device_registry.hpp` (whole), `hal/device_provider.hpp` (whole),
  `hal/pros/device_provider.hpp` (whole), `hal/fake/fake_device_provider.hpp` (whole), `config/robot_config.hpp`
  :1-80, :300-340, :515-1180 (`SensorsConfig`, `PortUse`, `valid()` whole, `describe()`), `robot/robot.hpp` :55-200,
  :496-830, :1055-1188, `robot/mechanism_parts.hpp` :190-232 (the `copyReason` pattern), `test/robot_builder_test.cpp`
  :1770-1830 (`SilentRefusal`) and the `SilentLine` site, `test/mechanism_devices_test.cpp` :1-130,
  `test/pros_device_registry_adapter_test.cpp` :60-160, `test/pros_adapter_fence_test.cpp` (whole — its fence rules
  apply to the new adapter), `test/pros_shim/pros/{error.h, screen.h, shim_control.hpp}` (CAL1-A1's landed C-shim
  pattern), `test/CMakeLists.txt` (tests are globbed — no edit needed), the vendored `include/pros/ai_vision.h` (the
  declarations; :33-34, :57-76, :85-175, :209-492), `tools/api_doc_tool.py` (target exclusions :93-103 — `sim/` and
  `/fake/` are outside the reference; check-examples :1947 — a ```cpp listing in a public doc must be a line of a
  compiled example test, so guide 10's camera section uses ```text only).
- **Every test or doc that pins a word VIS1-A1 changes** (grep, not the brief's list): `"vision ABSENT"` is pinned by
  `test/robot_builder_test.cpp` :883, `test/xdrive_config_test.cpp` :234 and `test/boot_source_test.cpp` :393 (the
  banner) — the brief named only :159 of boot_source_test (delta A1-D2). `E_DEVICE_AIVISION`'s word: only
  `pros_device_registry_adapter_test.cpp` :143. `"a vision sensor"`: the provider and the test's own `foundWords`.
- **The binary, re-read by me (T1/T4 — load-bearing for the shim):** `ar x firmware/libpros.a vdml_ai_vision.c.o`,
  `arm-none-eabi-objdump -dr` (scratch `vis1-a1/objdump/ai_vision_r.dis`). CONFIRMED, each: `aivision_get_class_name`
  — `registry_validate_binding` fails → `mvn.w r0,#0x80000000` (PROS_ERR); `port_mutex_take` fails → errno 13 + PROS_ERR;
  otherwise `bl vexDeviceAiVisionClassNameGet`, `port_mutex_give`, **`movs r0,#1`** — the SDK's value discarded, every
  id 1. `aivision_get_object` — `memset(sp+6,0,16)`, `strb #127` to `sp+4` (id) and `sp+5` (type), THEN
  `claim_port_try`; a failed claim returns that record. `aivision_get_color` — a failed claim loads 127 into id/R/G/B and
  `0x7f800000` (+Inf) into both floats. `set_enabled_detection_types`, `enable_`, `disable_` → `vexDeviceAiVisionEnableSet`,
  each ends `movs r0,#1`; `get_enabled_detection_types` → `vexDeviceAiVisionEnableGet`; `get_object_count` →
  `vexDeviceAiVisionObjectCountGet` (PROS_ERR on a failed binding/mutex). The brief's §0.3 stands.
- **The manual (§1.10's re-check):** `override-2.1.pdf` fetched once more (2026-10-18 ~22:55): **HTTP 404** — so
  **VIS1-A1 uses v2.0 (2026-09-03)**. Re-checked against v2.0 myself: the glossary (v20.txt :4668-4669 "Toggles are 25.8”
  (656.2mm) long and each face of the triangle is approximately 2.05” (52.2mm) wide"; :4546 Pin 1.6”/6.5”; :4270 Cup
  3.15”/6.5”) and the appendix drawings, VIEWED (`scratch/autoflip/img/`): pA8 side view — the axle 13.20 in (335.3 mm),
  the length 25.99 in with the caps; pA8 end view — **51.6 mm (2.03 in) is a VERTICAL dimension, apex to the FLAT bottom
  face, the axle 30.2 mm (1.19 in) below the apex, the corners heavily rounded**; pA10 — 140.41 in, each Toggle centred
  at 70.20 on its wall; pA13 portable wall 11.54 in; pA14 metal 11.50 in; pA15 — "Yellow facing up and into the field".
  **A finding the brief's S10 did not state (delta A1-D3):** read with the drawing, both the glossary's 2.05 in and the
  12.61-in lower edge hold — a rounded triangle whose corner arcs' outermost points lie 30.2 mm from the axle has its
  colour boundaries (the corner arcs' middles, by the 120° symmetry) 52.3 mm apart and the field face's lower boundary at
  13.20 − 0.594 = 12.61 in, 1.03 in in front of the apex — the brief's numbers — but the FLAT bottom face sits at 13.20 +
  1.19 − 2.03 = **12.36 in**, ~¼ in BELOW that lower edge, joined to it by the rounded corner. v4's 12.36 was the bottom
  face's height, not the field face's lower edge. Carried in `override_elements.hpp` as `flatBottomIn` with the
  arithmetic, and in HA-201's claim; the host camera model's bottom face at the lower edge's height is therefore a
  ~¼-in approximation (stated in its banner and in HA-201 — A2's tracker is where it would matter).

### RESUMED — Thu Oct 22 12:44:28 AM EDT 2026 (`date`)
- **The stop.** The team lead paused all work at ~23:30 on 2026-10-18 (the session limit); this run was cut off after
  the 23:20 entry above, so the log is BEHIND the tree: it records the reading and the re-checks, not the edits that
  followed. Resumed 2026-10-22 by a new executor (opus) under the coordinator's continuation task. The coordinator's
  audit at resume: 19 files changed, no mutation backup under `scratch/vis1-a1/` (the run had not reached its
  mutations), `src/`, `tools/` and the `Makefile` untouched; nothing ran since.
- **Checked first, myself:** `git status` — 13 modified, 6 new (`docs/internal/chunks/VIS1-PROGRESS.md`,
  `config/camera_config.hpp`, `hal/fake/fake_vision_frames.hpp`, `hal/pros/ai_vision.hpp`, `hal/vision_frames.hpp`,
  `test/pros_shim/pros/ai_vision.h`); `git diff --stat -- src/ Makefile tools/` EMPTY (no gate self-test plant left
  behind by a killed run); `scratch/vis1-a1/` holds only the 22:51 baseline outputs, the objdump and the manual's
  layout text — no `.bak`, no mutant.
- **What the stopped run had written (read whole, each against `v6/proposed` / `v6/overlay`):** §1.1
  `hal/vision_frames.hpp`, §1.2 `hal/pros/ai_vision.hpp`, §1.3 `hal/fake/fake_vision_frames.hpp`, §1.4 the shim
  (+ `shim_control.hpp`'s include and `resetAll()` line), §1.5 (all four switches, `kEveryKind`, the registry
  mapping, the registry tests' rows, `boot_source_test.cpp` :159), §1.6 (all four implementers), §1.7
  (`config/camera_config.hpp` and `robot_config.hpp`'s hooks and `valid()`'s checks), §1.8 (`robot.hpp`: `camera()`,
  `cameraAbsence()`, the boot line). NOT YET: §1.9 `vision/camera_geometry.hpp`, §1.10 `vision/override_elements.hpp`,
  §1.11 `sim/camera_plant.hpp`, every new test file, the three `"vision ABSENT"` pins (A1-D2), every document, the
  mutations. Continued from there; nothing finished is redone.

### Thu Oct 22 01:09:45 AM EDT 2026 — the fresh build, the three pins, the remaining headers, the tests (`date`)
- **A fresh build first** (`rm -f build/test/shulib_tests`; configure; build): the doc gate stopped it twice — the
  API reference stale (the stopped run never regenerated it: `api_doc_tool.py generate` run) and the briefing's
  header count (227 — regenerated with no binary on disk). Then rc 0. **Suite, capped** (`ulimit -v 4194304; timeout
  600`): 2,477 cases, **3 failed — exactly the three `"vision ABSENT"` pins A1-D2 named** (`robot_builder_test.cpp`
  :883, `xdrive_config_test.cpp` :234, `boot_source_test.cpp` :393 — the banner now ends "camera ABSENT"; "vision" and
  "camera" are both six letters, so the 193-byte line and the 200-byte TermSink cut stay true). All three updated
  (builder test 9 now also pins the old word GONE and the camera's words LAST).
- **The manual, once more** (2026-10-22 00:49): `override-2.1.pdf` **HTTP 404**, `override-2.0.pdf` 200 — v2.0 stands.
  I re-read pA8's end view myself (`scratch/autoflip/img/toggle_end-103.png`): 51.6 mm apex to the FLAT bottom, the axle
  30.2 mm below the apex, the corners heavily rounded — A1-D3 confirmed (inradius 21.4 mm, circumradius 30.2 mm; the
  boundaries 52.3 mm apart = the glossary's 52.2; the field face's lower boundary 12.61 in, the flat bottom 12.36 in).
- **Written from v6/proposed** (each compared with v4/v5 first: v4→v6 87 / 16 / 70 differing lines, v5 identical to v4 —
  v6 is the newest; every PROBE marker, line-number citation and "the brief's §" reference stripped, every member its own
  `///`): `vision/camera_geometry.hpp` (v6), `vision/override_elements.hpp` (v6), `sim/camera_plant.hpp` (v6).
- **Tests written** (probes P1, P2, R3, R6 as the starting point; every `printf` a CHECK or a MESSAGE):
  `test/vision_frames_test.cpp`, `test/pros_ai_vision_adapter_test.cpp`, `test/camera_geometry_test.cpp`,
  `test/override_elements_test.cpp`, `test/sim_camera_plant_test.cpp`, `test/robot_camera_test.cpp`; CHANGED
  `test/pros_adapter_fence_test.cpp` (the AI Vision adapter's five banned calls pinned as text — the adapter's header
  already said they were) and `test/robot_builder_test.cpp` (the two forwarding providers' camera forwarding pinned).

### The deltas from the brief and its v6 prototypes (A1-D1…), each with its reason — written at the resumption
*(A1-D1..D3 were logged by the stopped run above; D4 onward were found by diffing its code, comments stripped, against
`v6/proposed`, and by this run's own work.)*
- **A1-D1** (the stopped run): `v6/overlay/src/robots/chassis_table.hpp` NOT applied — `src/` is VIS1-B's.
- **A1-D2** (the stopped run): three tests pin "vision ABSENT", not one — updated here (above).
- **A1-D3** (the stopped run; re-verified by me from the drawing): the Toggle's FLAT bottom face is 12.36 in, ~¼ in
  below the field face's lower boundary (12.61 in) — carried as `ToggleFace::flatBottomIn` and in HA-201; the host
  camera model's bottom face at the boundary's height is a stated ~¼-in approximation.
- **A1-D4** (the stopped run, `hal/vision_frames.hpp`): `kMaxVisionClassNames = 20` (the adapter `static_assert`s it
  equals `AIVISION_MAX_CLASSNAME_COUNT`); a NULL colour name refused; the objects-table check split into named refusals
  (no table / more than 20 names / an empty or null name — the prototype folded "more than 20" into "no table");
  `sameVisionText()` + `sameAiVisionSettings()` ADDED — §1.6's "the same object for a second request with EQUAL settings
  (field by field, strings by content)" needs one comparison both providers share.
- **A1-D5** (the stopped run, `hal/pros/ai_vision.hpp`): copy/move deleted (the provider holds it in place; its status
  strings point into it); `settings()` accessor (the provider compares a second request against it); a failed count
  read sets the set-up problem's words; the model problem SANITIZED to printable ASCII (the camera's bytes reach the
  boot line); `f.model` stamped before the early returns (a NotConfigured or ReadFault frame now says the model's state —
  neither is usable either way).
- **A1-D6** (the stopped run, `config/camera_config.hpp`): `finiteNumber()`; a NaN or infinite forward/left offset or
  tolerance refused (the prototype refused only a tolerance ≤ 0, so an infinite one — an interval of everything —
  passed as "measured").
- **A1-D7** (this run, `vision/camera_geometry.hpp`): `boundAtImage()` also refuses a NEGATIVE or NON-FINITE tolerance in
  the bound it widens, and on ANY bad input makes BOTH tolerances infinite (the prototype: the position tolerance only)
  — the brief's own rule, "never a tight bound from a bad number", applied to the heading tolerance a consumer (AF1's
  "in front of my arm" window, §6.5 rule 4) reads on its own; `math::Angle::kPi` used (no second π); "probe P4c" (a
  typo) → P4b's numbers; line-number citations into other files dropped (landmine 14); `project()` `[[nodiscard]]`.
- **A1-D8** (this run, `vision/override_elements.hpp`): `modelClassName()` reads `kOverrideClassNames` (the prototype
  kept a second copy of the five names in a switch — one table, never two; the test pins they are the same pointers);
  `modelClassSize()` through the enum; every number cited by the manual's page name (pA8, pA10, pA13–15, the glossary),
  not by lines of a scratch text file; one member per line.
- **A1-D9** (this run, `sim/camera_plant.hpp`): `quadCount()` / `quad(i)` added — §2.2's `addRolledToggle` anchors are
  the model's OWN geometry, so the test reads the quads it drew; the unused `hal/clock.hpp` include dropped; the
  banner states the missing wall and the flat-bottom approximation.
- **A1-D10** (this run, tests): R6.1/R6.2 (the brief's finder-level probes, A2's) re-expressed at the GEOMETRY level
  (bearing + ground range + field fix), so MG5/MG6 have a scene-level witness in A1 — with the truth taken as the Pin's
  NEAR RIM from the true lens at the image's time (what the ground range measures, HA-200's belief): with the centre and
  0.8 in of slack instead, the 60-in/s case sat 0.2 in INSIDE the un-widened bound at 150 ms and the finding was not
  pinned. P2.1's "width < 8 in" (mine, a guess) replaced by its honest bound (lo > 18, hi < 32; measured 10.1 wide).
- **A1-D11** (this run, tests): `test/pros_adapter_fence_test.cpp` pins the adapter's five banned calls as text (the
  adapter's header already named that test as the pin; §2.2's "source pin"); `test/robot_builder_test.cpp`'s two
  forwarding providers each pin the camera's forwarding (§2.2, "the forwarding two"); `test/robot_camera_test.cpp`
  also runs the Robot over the REAL provider and the shim (D4 and D5 end to end — not in the brief's list).
- **A1-D12** (this run): the Freeze Register row is **F15** (the next after F14; the stopped run's header already named
  it).
- **A1-D13** (this run): `docs/internal/build-order.md`'s VIS1/AF1 entry and `REMAINING-WORK.md` row 18d NOT edited in
  this tree — the coordinator's plan commit is rewriting exactly those entries on main as this runs (D1–D11's answers,
  the three runs); a second, stale-based rewrite here would only conflict. The status lines to add at the merge are in
  the run's report.
- **A1-D14** (this run): guide 14's tag-corrector sentence ("a camera, a tag map, and a task polling it, none of which
  exist yet") clarified — the camera EXISTS now for colours and objects; a camera that reads TAGS does not (prose
  review, below). Not in the brief's A1 document list (guide 14 is A2's), but made stale by A1.

### Thu Oct 22 01:22:55 AM EDT 2026 — the documents and the prose review (pace rule 4; every grep RUN and its hits READ before this was written)
- **Written:** `docs/changelog.md` (a new entry at the TOP of "## API 2.4": additive, no version bump, plus the two word
  changes — the census's "AI-VISION" and the boot line's "camera ..."; its census claim CORRECTED after reading
  `boot_check.hpp` :288-298 — the boot check names a kind only for a drive or IMU port holding the wrong device, so only
  Calibrate's STAGE 0 census, `calibrate_program.cpp` :1115, lists port 3 as AI-VISION); `docs/guide/10-the-api.md` (a
  section "The camera — a frame at a time" before the gotchas, ```text only — check-examples needs every ```cpp line in
  a compiled example test; its "Covers" line); `docs/guide/15-glossary.md` (ten terms: AI model / class table, bearing,
  colour descriptor, fix radius, frame (a camera's), ground range, one camera explains it, possibly full, size range,
  tolerance box); `docs/hardware-assumptions.md` (HA-192..HA-203: twelve Index rows, a "VIS1-A1's additions" section of
  twelve entries placed before MECH-A1's — away from where MECH-A3 appends —, the status line 177 → 189 and its
  paragraph, HA-70's "no library adapter reads it yet" gaining "(the camera's objects and colours are read since VIS1-A1,
  2026-10-22; its TAGS still are not)"); `docs/roadmap.md` (Freeze Register row F15, NOT FROZEN, said out loud);
  `test/README.md` (the camera's three test doubles — the tool keeps `hal/fake/` and `sim/` out of the reference);
  `docs/api/` + `mkdocs.yml` regenerated (five new pages: vision_frames, pros-ai_vision, camera_config,
  camera_geometry, override_elements).
- **Public grep** `grep -rn -i "vision\|camera\|AbsentVision\|DeviceKind::Vision" docs/ README.md` (docs/api, docs/internal,
  the register and the changelog read separately): 367 lines; every hit outside my own new text READ —
  `faq.md` :48, guide 01 :32, guide 03 :84-89 / :107 / :148-159 / :244-255 / :397, guide 11 :559-562, guide 12 :54 / :144,
  guide 13 :251, guide 16 :145 (the GPS's camera, the tag corrector's camera and its vision-rate task, the robot's
  camera END) — all still true: tags are not read, no camera has been pointed at a tag; `vexbuilder-integration.md` :115
  (the file format's `"vision"` sensor type — F7's, G3a's; not changed by A1). **STALE, fixed:** guide 14 :145-147 ("a
  camera, a tag map, and a task polling it, none of which exist yet") — the camera exists now for colours and objects;
  clarified to "a camera that reads TAGS" (A1-D14). guide README :18 (chapter 10 "Every verb ...") left as MECH-A1 left it.
- **Internal grep** for the statements §3.4 names: "no adapter yet" — `hal/device_registry.hpp` :57's doc FIXED by the
  stopped run; **`src/programs/drive_program.cpp` :212-213 and `src/programs/xdrive_program.cpp` :145 still say the camera
  "has no adapter yet" / "no camera adapter exists yet" in their boot lines** — `src/` is out of A1's reach (its diff must
  stay empty), and both are still TRUE in substance (no program declares or polls a camera); FOR THE COORDINATOR / VIS1-B,
  which edits the programs' boot words. "TAGS AND VISION ARE DELIBERATELY ABSENT" — gone from `robot_config.hpp` (the
  stopped run). "vision ABSENT" — only historical records (`G1-PROGRESS.md` :1167, :1174) and this log; the three test pins
  updated. "interprets nothing" — `R3a-BENCH-WORKSHEET.md` :1128 (station 12, VIS1-B's). "COLORS left as found" — station
  12 / the worksheet, VIS1-B's. HA-70 — done. `R2a-sensor-stations.md` :101 names the registry trap A1 fixed — a record, not
  rewritten. `HANDOFF-remaining-work.md` :265-269 (the tree's adapter list ended "**no AI Vision adapter.**") — FIXED in
  one line (the camera adapter named; "no AprilTag adapter"); its item L3 (:417-425, R2b's plan — "hal/pros/vision.hpp
  implementing hal::IVision") is PLAN text the coordinator's plan commit owns (A1-D13): VIS1 built a sibling seam
  instead (D11) — for the coordinator.

### RESUMED (second stop) — Thu Oct 22 08:44:12 AM EDT 2026 (`date`)
- **The stop.** The run was stopped a second time at ~01:45 on 2026-10-22 (the team lead dropped to one job near a
  usage limit) — after the 01:22 entry above, so the log is again BEHIND the tree. Resumed by a new executor (opus)
  under the coordinator's continuation task. The coordinator's audit at resume: 50 files changed; one mutation backup
  (M10 of `ai_vision.hpp`) matching the file — no mutation left applied; `src/`, `tools/` and the `Makefile` untouched;
  nothing ran since.
- **Checked first, myself:** `git status --short` — 50 entries (31 modified, 19 new, every one this chunk's);
  `git diff --stat ceb17c0 -- src/ tools/ Makefile` EMPTY (no gate self-test plant left by a killed run). Every backup
  under `scratch/vis1-a1/bak/` (M1–M10, M6b, M7b — all of `ai_vision.hpp`) is byte-identical (`cmp`) to the file, so no
  mutant is applied. The suite binary on disk is dated 01:39 — the M10 mutant's link, the run's last action — so it is
  REMOVED and the suite rebuilt from source (with `ai_vision.hpp` touched, so every object built from the M10 mutant
  recompiles) before any result is trusted.
- **What the stopped run did after its 01:22 entry, from its scratch outputs (not logged before):** `ctest -R
  compile_refusal` 200/200 (`compile_refusal1.txt`, 01:24); `src_build_gate.py self-test` OK, 235 detector cases
  (01:28) and `check` PASS, all 7 builds link (01:31); a full build (01:32) and the suite 2,551 cases, 0 failed, 3
  skipped (`resume_suite4.txt`, 01:33); then its mutation runner (`scratch/vis1-a1/mutate.py` — one exact anchor each,
  the line count kept, the binary deleted, a FULL `cmake --build`, the whole suite under the 4 GiB cap and 600 s, the
  file restored and touched in a `finally`) from 01:33: **M1–M9, M6b, M7b each RED** (`mutations.log`, below in the
  mutation table); M10 started at 01:39 and was cut off by the stop (no verdict — re-run here).

### Thu Oct 22 08:46–09:45 AM EDT 2026 — the fresh build, the mutation run finished, one survivor fixed, the last prose fixes (`date` 09:45)
- **The fresh build and suite (08:45, before anything else ran):** `rm -f build/test/shulib_tests`, the briefing
  regenerated with no binary, configure, build rc 0 (every object depending on `ai_vision.hpp` recompiled — it was
  touched first); the suite under `(ulimit -v 4194304; timeout 600 …)`: **2,551 cases, 2,551 passed, 0 failed, 3
  skipped; 2,795,631 assertions** — the stopped run's 01:33 numbers exactly, so its M1–M9 verdicts stand on this code
  (`find include test docs mkdocs.yml -newer scratch/vis1-a1/mutations.started`: only `ai_vision.hpp` (restored and
  touched) and two internal docs changed after the runner started).
- **The mutation runner continued** (`scratch/vis1-a1/mutate2.py` = the stopped run's `mutate.py` with the log path a
  parameter; one exact anchor each, the line count kept, the binary deleted, a FULL `cmake --build`, the WHOLE suite
  under the 4 GiB cap and 600 s, the file restored and touched in a `finally`): M10 onward, 08:46–09:10. Afterwards all
  50 backups compared byte-equal (`filecmp`) with their files — no mutant left applied; `git status` 50 entries as
  before.

#### The mutation table — every one RUN on a fresh full build, the whole suite, SEEN RED, restored
| Id | The break | Verdict (failing cases) | Who ran it |
|---|---|---|---|
| M1 | the mode mask leaves TAGS alone | RED (23) | stopped run, 01:33 |
| M2 | no re-set-up after a failed read | RED (2) | stopped run |
| M3 | reads index 24 and past | RED (1) | stopped run |
| M4 | a read with a skipped object still `Ok` | RED (2) | stopped run |
| M5 | no mode read-back after the write | RED (2) | stopped run |
| M6 / M6b | no retry rate limit / the retry's edge `>` (lands EXACTLY on 0.5 s) | RED (1) / RED (1) | stopped run |
| M7 / M7b | no 1 Hz verify / the verify's edge `>` (lands EXACTLY on 1 s) | RED (1) / RED (1) | stopped run |
| M8 | the model check skipped | RED (5) | stopped run |
| M9 | the model check compares only class 0 | RED (3) | stopped run, 01:39 |
| M10 | an unreadable table counted as a match | RED (2) | this run (the stopped run's M10 had no verdict) |
| M11 | an Object past the expected table passed on | RED (1) | this run |
| M12 | the name buffer not zeroed (it holds the expected name) | RED (2) | this run |
| M13 | a colour read-back not compared | RED (2) | this run |
| M14 | possibly full only ABOVE the cap (lands EXACTLY on 24) | RED (1) | this run |
| R1 | a `PROS_ERR` count treated as a number | RED (3) | this run |
| R2 | the provider makes a SECOND adapter for a second request | RED (1) | this run |
| R3 | `E_DEVICE_AIVISION` back to `Vision` | RED (6) | this run |
| R4 | `valid()` does not claim the camera's port | RED (1) | this run |
| R5 | the Robot refuses the build on a refused camera (D5 (b)) | RED (2) | this run |
| R6 | an all-zero descriptor accepted | RED (4) | this run |
| MG1–MG4 | the yaw's sign; the image's down read as up; square pixels; the nominal camera only | RED (1 / 7 / 3 / 3) | this run |
| MG5 / MG6 | `boundAtImage` drops the turn term / the speed term | RED (2) / RED (2) | this run |
| MG7 | `boundAtImage`'s bad-input guard removed | RED (1) | this run |
| MG8 | the horizon refusal removed — **BUILD FAILED, NOT A VERDICT**: the mutant left `minDepressionDeg` unused (`-Werror=unused-parameter`), the mutation's own breakage, not a test | — | this run |
| MG8b | the horizon's MARGIN dropped (`0.0 * minDepressionDeg`: rays just below level accepted) — **SURVIVED** at 09:10, then **RED (1)** at 09:12 after the test below was written | SURVIVED → RED (1) | this run (A1-D15) |
| MG9 | the size range's too-small edge `>=` (lands EXACTLY on 5 px) | RED (1) | this run |
| MC1 | a mount 36 in up accepted (lands EXACTLY on 36) | RED (1) | this run |
| MC2 / MC3 | `valid()` skips a SET mount's problem / the settings' problem | RED (1) / RED (1) | this run |
| MB1–MB4 | the boot line's words not sanitized; `cameraAbsence()` not empty for a built camera; a provider's `std::exception` / non-standard throw not caught | RED (1 / 2 / 1 / 1) | this run |
| MF1 / MF2 | the fake provider answers `readFaults` before serving / serves different settings | RED (2) / RED (1) | this run |
| MD1 | the census word back to "VISION" | RED (1) | this run |
| MS2 | `clipped()` strict at the margin (lands EXACTLY on it) | RED (1) | this run |
| MS3 / MS4 | `sameAiVisionSettings` ignores the class names / a pending model check worded "model OK" | RED (1) / RED (3) | this run |
| MFV1 | the fake camera's failing read holds the set frame | RED (1) | this run |
| MFENCE | the adapter starts a white balance | RED (22) — the shim's counters; and **the text pin ALONE** (`-tc "forbidden calls are textually absent*"`, the same mutant, a fresh build, 09:13): RED at `pros_adapter_fence_test.cpp` :202 | this run |
| MP1–MP4 | the host camera model: image upside down; possibly full only above the cap (EXACTLY 24); frozen frame not re-stamped; withheld objects not counted | RED (7 / 1 / 1 / 1) | this run |

- **Every brief §2.2 mutation is in it:** M1–M14, MG1–MG6, and the run's own six (R2 the second adapter, R3
  `E_DEVICE_AIVISION` → `Vision`, R4 the claim, R5 the Robot refusing, R6 the all-zero descriptor, R1 `PROS_ERR` as a
  number); the rest are this chunk's own. **No verdict rests on a compile-time refusal:** the only build failure (MG8)
  was the mutation's own breakage and is replaced by MG8b, whose red is a test's assertion. The equality edges each have
  a test that lands on them exactly (binary-exact clock steps 0.25 + 0.25 and 0.5 + 0.5; a count of exactly 24; a box
  exactly on the margin; a height of exactly 36; exactly 5 px).

### The deltas found at this resumption (continuing A1-D1…D14)
- **A1-D15** — `test/camera_geometry_test.cpp` P2.4 gains the MARGIN case: an 8-in level mount, row 131, where EVERY
  corner's ray descends (the shallowest between 1 and 2 degrees — `REQUIRE`d, computed through `eachCorner` + `ray`, so
  the case cannot drift out of the margin unseen): refused at the default 2 degrees, accepted with 1 degree, its far end
  past 200 in. Why: the existing horizon rows refuse because some corner looks UP, which MG8b's mutant still refuses; it
  SURVIVED until a row inside the margin was tested. The file's banner names MG8b.
- **A1-D16** — `docs/roadmap.md`'s R2 row (`[~] Vendor & extend … IVision/ITagSource adapter`) said "the adapter itself
  (R2b) is written afterwards from those records, not from beliefs" — VIS1-A1 made that stale for the objects-and-colours
  half. ONE amendment paragraph added under it (the half pulled forward, through the sibling seam F15, on beliefs
  HA-192..HA-203; still `[~]`: no tag adapter, nothing read on a robot). Found by this resumption's prose grep (the
  stopped run's review listed the guide's hits but not the roadmap's or the master plan's).
- **A1-D17** — `docs/guide/15-glossary.md`: two of the ten new terms were out of alphabetical order ("ground range" before
  "GPS", "one camera explains it" before "odometry") — moved. (Five older out-of-order entries are left as found — not
  this chunk's.)
- **A1-D18** — `test/robot_builder_test.cpp` ~:827: a comment still said "Tags and vision are ABSENT unconditionally in
  Part 1 — no camera adapter exists"; now says the frozen `IVision` stays absent and the camera is read through
  `IVisionFrames` (D11). Comment only.

### The prose review, re-run by this resumption (pace rule 4 — each grep RUN and its hits READ before this was written)
- **Public:** `grep -rn -i "vision\|camera\|AbsentVision\|DeviceKind::Vision" docs/ README.md --include=*.md` outside
  `docs/api/`, `docs/internal/`, the register and the changelog: 18 files. READ: README :131 (no camera pointed at a
  TAG — true); faq :48; guide 01 :32; guide 03 :84-89, :107, :148-159, :244-255, :397 (the GPS's camera, the TAG
  corrector — true: tags are not read); guide 05/06 (the word "provisional" only); guide 10 (this chunk's section);
  guide 11 :559-562 (the tag corrector's vision task — true); guide 12 :54, :144; guide 13 :251; guide 14 :105-112
  (the tag pipeline's host truth — true), :145-149 (A1-D14's clarification); guide 15 (this chunk's terms — A1-D17);
  guide 16 :145 ("the camera end" of robot two — a direction, unchanged); `legacy-command-vocabulary.md`,
  `diagnostics-plan.md` ("provisional"/"revision" only); `vexbuilder-integration.md` :115 (the file format's `"vision"`
  sensor type — F7/G3a's); **`shulib-v2-master-plan.md`** :93-97, :129, :220, :238, :409-410, :498-504 (decision #7 and
  the 2026-06 "RESOLVED" note that the wrapper "becomes the hal/pros IVision/ITagSource adapter") — a locked decision's
  record, still true in its own terms (D11: `ProsAiVision` is the class the tag part extends) — left; **`roadmap.md`** —
  F4's row (untouched, true), F15 (this chunk's), :242, :414-415, :457-465, :658, :679, :831 (dated records, true),
  :1103, :1121 (R1a/R1b's dated "NOT done: the vision adapter (R2's)" — records of their day, left), **:1124-1134 the
  R2 row — STALE, amended (A1-D16)**.
- **Internal / code** — `grep -rn "vision ABSENT\|VISION ARE DELIBERATELY\|no adapter yet\|no camera adapter\|interprets
  nothing\|INTERPRETS NOTHING\|COLORS left as found\|no library adapter reads it yet"` over the tree: the two test pins'
  own "was" comments; **`src/programs/xdrive_program.cpp` :145 and `src/programs/drive_program.cpp` :213** ("no camera
  adapter exists yet" / "has no adapter yet") — `src/` is out of A1's reach and both still true in substance (no
  program declares or polls a camera) — **VIS1-B's**; **`test/robot_builder_test.cpp` :827 — STALE, fixed (A1-D18)**;
  `robot.hpp` :786 (this chunk's "until VIS1-A1 the line ended …"); `R3a-BENCH-WORKSHEET.md` :1128 (station 12 —
  VIS1-B's); G1-PROGRESS :455, :1167, :1174 (records); the register :3930 (HA-70, amended by the stopped run as the
  brief's §4 words it). And `grep -rn -i "R2 owns\|camera adapter\|no adapter\|AI Vision" include/shulib test/*.cpp`
  outside this chunk's files: `hal/vision.hpp`, `vision_conversion.hpp` :97 (the TAG adapter's corners — R2b's, true),
  `robot_context.hpp` :98, `ekf_fusion_test.cpp` :922, `robot_context_test.cpp` :154 (tag sources — true).

### The register's checks (re-check T7), run myself
- Rows: 189 (`^| HA-n |`), = the status line's "7 of 189". HA-192..HA-203 each have a row AND an entry.
- **Direction 2** (each id this run registers has an `A4 register HA-n` site under `include/shulib`, wrapped lines
  included): HA-192 `hal/vision_frames.hpp`, `vision/camera_geometry.hpp`, `sim/camera_plant.hpp`; HA-193
  `hal/vision_frames.hpp`, `vision/camera_geometry.hpp`; HA-194 `vision/camera_geometry.hpp`; HA-195
  `hal/pros/ai_vision.hpp`; HA-196 `hal/pros/ai_vision.hpp`, `hal/device_provider.hpp`, `hal/pros/device_provider.hpp`;
  HA-197 `hal/pros/ai_vision.hpp`, `vision/override_elements.hpp`; HA-198 `hal/vision_frames.hpp`; HA-199
  `sim/camera_plant.hpp`; HA-200 `vision/camera_geometry.hpp` (`groundRange` and `sizeRange`); HA-201
  `vision/override_elements.hpp`; HA-202 `vision/camera_geometry.hpp`; HA-203 `hal/vision_frames.hpp`. **Direction 1:**
  every `A4 register HA-n` label under `include/shulib` has a row (none missing); `grep -rn 'PROVISIONAL (A4'
  include/shulib | grep -v 'HA-[0-9]'` empty.
- **The Held line:** in THIS tree (base `ceb17c0`) it never carried HA-192..HA-203 — the plan commit `71b4764` (on
  `shulib-v2`, after this base) added them. At the merge the Held line must DROP ", HA-192..HA-203" (all twelve are
  registered here — the brief's §3.1) — for the coordinator.

### FOR THE COORDINATOR — a hardware belief OUTSIDE HA-192..HA-203 (logged, not numbered: the brief's rule)
- **The registry reports an AI Vision sensor as `E_DEVICE_AIVISION` (29), never `E_DEVICE_VISION` (11).** VIS1-A1's
  D4 makes the provider REFUSE the camera on any other kind, so the camera now depends on it. The only robot evidence —
  "the calibrate program's own census confirmed VISION 3" (`R2a-sensor-stations.md` :14-16) — was printed through
  `deviceKindWord`, which until VIS1-A1 named BOTH values "VISION": it cannot tell 11 from 29. The code cites HA-141
  (`hal/device_provider.hpp`'s camera section), whose claim is scoped to an IMU, a rotation sensor and a GPS — so
  either HA-141's claim widens to the AI Vision kind or the belief takes a number (the next free is HA-204). **Settle,
  no new code:** Step 0.2 — Bench Tests station 12 scans the registry for `E_DEVICE_AIVISION` only and says "no
  AI-VISION sensor in the registry" otherwise; station 1's census prints "AI-VISION" for 29 (`bench_r3a.cpp` :512).
  **Blast radius if wrong:** the camera is refused at boot, safely (D5: the robot builds and drives), and the boot
  line names it — "camera port 3 ABSENT (port 3: a vision sensor (expected an AI Vision sensor))".

### The gates (final, after every edit — `date` ~09:45)
- The suite (capped, fresh binary 09:14): **2,551 cases, 2,551 passed, 0 failed, 3 skipped; 2,795,637 assertions**
  (+6: A1-D15's checks); 29.5 s, 62 MB peak.
- `ctest --test-dir build/test -R compile_refusal` (capped): **100% tests passed, 0 tests failed out of 200**.
- The ARM header gate (RESUMING's command, `scratch/vis1-a1/all.cpp`): **230 headers, rc 0**. CI guards: no `pros/`
  include outside `hal/pros/` (empty); no `shulib/sim/` include outside `sim/` (empty).
- `src_build_gate.py self-test`: **OK (235 detector cases, each against a real build)** (09:19–09:37); `check`: **PASS
  (all 7 builds link** — bench, xdrive, tank, tank-drive, tank-calibrate, tank-autontest, tank-competition: each
  11/11 TUs, 0 disallowed warnings; ~09:38–09:56); `git status -- src/ Makefile tools/` clean after both.
  The doc gates and the briefing check were re-run after both finished (09:57): every one as below.
- `api_doc_tool.py` self-test OK; check-coverage "4951 public entities across 188 headers, all documented";
  check-fresh rc 0; check-examples "541 quoted lines, all verbatim"; check-removability "no public doc references
  docs/internal/". `doc_staleness_audit.py` clean (29 live docs). `briefing_status.py check` "generated block current"
  (regenerated before the build with no binary, and again after the suite binary existed). *(One `briefing_status.py
  check` run while the src gate's self-test was planting its faults returned 1; re-run twice after, rc 0 — the
  self-test's plants, not the tree.)*
- The brief's own: `include/shulib/hal/vision.hpp` untouched since `ceb17c0`; `git diff --stat ceb17c0 -- src/ tools/`
  EMPTY; `grep 'PROBE\|prototype\|proposed'` over the new files EMPTY; the five banned calls appear in
  `hal/pros/ai_vision.hpp` only on comment lines (:29, :31, :33); `PROVISIONAL (A4` without an id EMPTY; the C7 grep
  EMPTY; no "simulator" in any added line.

### The tree at the end of this run — `date` Thu Oct 22 09:57 AM EDT 2026
- **Nothing committed** (branch `vis1-a1`, base `ceb17c0`): 31 modified, 19 new — this chunk's files only, plus the
  regenerated `docs/api/` and `PROJECT-BRIEFING.md`'s generated block. `src/`, `tools/` and the `Makefile` untouched.
  No mutant applied (50 + 1 backups under `scratch/vis1-a1/bak/` byte-equal to their files). The suite binary on disk
  is the fresh 09:14 link of this tree.
- **Not done here, by design:** `build-order.md` and `REMAINING-WORK.md` (A1-D13 — the plan commit `71b4764` rewrote
  VIS1's entries on `shulib-v2` after this base; the status lines are in the run's report); the register's Held line
  (drops HA-192..HA-203 at the merge); the programs' "no camera adapter" boot words (`src/`, VIS1-B's); VIS1-A2 and
  VIS1-B entirely.

---

## VIS1-A1 review findings and their disposition

- **Started:** Thu Oct 29 02:14:57 PM EDT 2026 (`date`). Executor: opus (the builder again, under the coordinator's
  workflow run). Tree: the uncommitted VIS1-A1 work in the worktree `vis1-a1` (base `ceb17c0`), 50 entries in
  `git status`, `git diff --stat ceb17c0 -- src/ tools/ Makefile` EMPTY, the suite binary on disk the 2026-10-22 09:14
  link (rebuilt from source before any result here is trusted).
- **The task:** three reviewers (correctness, tests, docs) returned 27 findings; each is re-checked here, fixed at the
  root when real or answered with evidence when not, every gate re-run. Nothing committed. Probes and mutation backups
  under `.claude/coordinator/scratch/vis1-a1/review-fixes/` only.

### Re-checked first, each finding against the tree (14:15–15:00)
Every finding was re-checked before anything was changed — the correctness reviewer's probes re-run on the tree
as it stood (compiled against `review-correctness/tree/include`, the reviewers' copy of it) and my own probes under
`scratch/vis1-a1/review-fixes/probes/`. **All 27 findings are REAL** (none answered "not a defect"); one is partly
out of this run's reach (`src/`, the docs reviewer's boot-line finding) and one is merge-time work for the coordinator.
One more defect was found while fixing them (A1-R1 below).

### The fixes, finding by finding — correctness
- **(1, major) `sizeRange()` returned the depth along the optical axis.** RE-CHECKED: my probe `size_probe.cpp`
  (thin upright lines and real Pins across the WHOLE width, five pitches −15..+12°, the true camera at 98 % of every
  tolerance in all sign combinations, the two focal lengths apart): on the old code **3,350 of 18,495 lines missed
  their true range (worst 17.2 % outside)**. **FIXED at the root:** for each corner the answer is now EXACT for a
  vertical line — the ray through row v, turned to the line's bearing b off the axis, rises
  tan(pitch − atan(yd)) × cos(b) per inch, so R = H / (cos(b) × (rise_top − rise_bottom)); widened over the
  corners, ±kBoxEdgePx on each edge and on the column. (The reviewer's suggested fix, axis depth × hypot(cp + yd·sp,
  xr), would still take the depth from H·fy/px, which is right only for a level camera — off by the factor
  cp + yd·sp, ~4 % at a 10° pitch; the exact form has no such term.) After: **0 of 18,495 lines miss**; the reviewer's
  own probes on the fixed code: `size_offaxis` 0 of 26 miss, `size_sweep` 0 at −10°, 0°, +10°. **A geometric fact
  the fix exposed, documented (header, guide 10, glossary, changelog):** a real Pin's box takes in its TOP FACE, so
  its size range reads NEARER than its near rim — at 23 in from a 10-in lens its box's line-equivalent range is
  21.48 in against a near rim of 22.20 (`size_probe` (b): 139 of 32,881 Pin intervals do not reach [near rim,
  centre]; (c): **0 of 32,881 miss the box's line-equivalent range**, computed by plain trigonometry from the scene).
  **FOR THE COORDINATOR (A2's brief, §1.13 rule 1):** "ON THE FLOOR when SOME corner makes the ground range of its box
  bottom (the near rim, + up to one radius to its centre) agree with the range its known height implies" must allow
  the size range to read up to about a radius-and-a-half NEARER than the near rim (the top face's depth), or a floor
  Pin seen from above could be called RAISED/UNSURE; the finder's own tests will show how much.
  Tests: P2.2 rewritten (below); a new property case — 0.3-in rods across the whole width, pitches −12/0/+10, all 32
  sign combinations, CONTAINMENT of each rod's line-equivalent range (8,978 measured, all held); the reviewer's hand
  anchor (24 in at 30°); a "jitter LANDED ON" case that puts the truth exactly on the interval's end.
- **(2, major) the 1 Hz re-ask of a DIFFERENT model was untested.** RE-CHECKED: the reviewer's mutant survived
  (2551/2551, their `mutA_suite.txt`). **FIXED:** a test — names that read wrong at boot (the binary's realistic path,
  `movs r0, #1`), not re-asked at 0.5 s, re-asked at EXACTLY 1.0 s (binary-exact steps), Matches, the Object passes.
  The reviewer's mutant re-run on the new code as MV5 (below).
- **(3, minor) the two focal lengths were tied.** RE-CHECKED: the reviewer's `p22_independent` — 10 groundRange and 1
  bearing misses of 3,824 with the factors apart. **FIXED:** `forEachCorner` takes sx and sy apart — 32 corners with
  the height, 16 without (eachCorner's doc, the header banner, `CameraOptics::fovTolerance`'s doc, HA-194, the
  glossary's "tolerance box"). After: `p22_independent` 0 / 0 / 0. P2.3 pins 32/16 and that (fx down, fy up) and
  (fx up, fy down) are each 8 corners; P2.2 perturbs them apart.
- **(4, minor) `fieldFix()`: a non-finite heading tolerance gave a TIGHT radius.** RE-CHECKED (the reviewer's
  `adapter_edges` (d): inf → 1.250 in). **FIXED:** a negative or non-finite position or heading tolerance gives an
  INFINITE radius (the nominal point still computed); an interval with a non-finite or reversed end, or a non-finite
  x or y, gives no fix; and the heading term is now EXACT per interval corner at ANY tolerance — the corner's point
  turns rigidly about the robot's centre, so its farthest is an end of the heading interval or, when the interval
  reaches it, the diametrically opposite point (|v| + |n|), which is now counted (the reviewer's "above 180 degrees
  under-bounds" — it also under-bounded slightly just below 180). A NaN heading cannot reach `fieldFix` at all:
  `math::Angle` throws on a non-finite input (pinned: `CHECK_THROWS`), so the guard reads x and y only. After:
  `adapter_edges` (d) inf → inf, NaN → inf. Tests: inf/NaN/−1/−inf on each tolerance; boundAtImage's own "not a
  bound" output; non-finite intervals and poses; 179/180/181/270/1000 degrees all give |v| + |n| (+ the mount's
  0.25) with a 0.6-rad bearing interval; monotone in the tolerance from 0 to 360.
- **(5, minor) an unplug first met by the 1 Hz read-back counted as a lost setting.** RE-CHECKED (`adapter_edges`
  (a): faultedReads 0, settingsLost 1). **FIXED:** the check returns Holds / Changed / **Unanswered** — the mode
  read's `PROS_ERR` or get_color's failure record is a FAILED READ: a ReadFault frame, `faultedReads()` +1, its own
  problem text ("the camera did not answer the 1 Hz check -- the set-up is re-written"), `settingsLost()` untouched.
  After: (a) status 2 (ReadFault), faultedReads 1, settingsLost 0. Test: the same unplug met by the check (at
  exactly 1.0 s) and by the count (at 0.9 s) — each ONE faulted read, settingsLost 0, then six NotConfigured reads
  and a recovery; and a colour read-back the check could not make.
- **(6, minor) T4/P1.18's second half untested; the shim could not produce it.** RE-CHECKED (the shim's `get_color`
  answered the failure record only when `disconnected`, which also fails the write first). **FIXED:** the shim gains
  `colourReadFails` (the claim fails, the write lands); the adapter now NAMES the failure record ("a colour read-back
  failed (the camera did not answer)" — an id outside 1..7, which no stored descriptor has); a test with RGB 40 AND
  RGB 127,127,127, the write made and landed, no mode written, the retry configuring it.
- **(7, minor) a match cached across a re-plug no failed read observed.** RE-CHECKED (by reading: the 1 Hz check
  compared the modes only; the model was re-asked only while not Matches). **FIXED (A1-D19 — a deliberate change to
  the brief's §1.2 rule 4, "re-asked once a second while it does not match"; landmine 6, "never cache a match
  across a re-plug", is what it serves):** the 1 Hz check reads the modes AND every descriptor back and then
  re-asks the model, matching or not — a match is never more than a second old, and a camera swapped while nothing
  read it is caught BEFORE the first read's objects pass. Its cost (1 + colours + expected names, ≤ 28 calls, once
  a second) is added to HA-203's claim; the residual limit (a camera swapped AND answering again between two reads
  under a second apart, with no read meeting it unplugged) is stated in the adapter's header. Tests: a swap with the
  modes right — another model (Different, withheld in that very read) and other descriptors (re-written before the
  read, settingsLost 1).
- **(8, minor) colour/code detections of descriptors this program never wrote passed as evidence.** RE-CHECKED
  (`adapter_edges` (b): status Ok, Colour id 1). **FIXED:** a Colour whose id is not one the settings wrote, and every
  Code (VIS1 writes none), is WITHHELD and counted in a new `VisionFrame::coloursWithheld` (additive on the F15 seam,
  NOT FROZEN); the frame stays Ok. After: (b) count 0. Test: the shim's stray slot 1, an unwritten id 7 and a CODE 2
  (descriptor 2 IS written, so only the code rule withholds it), against two written colours that pass.
- **(9, minor) the uint16 score narrowed to int16.** RE-CHECKED ((c): 65535 → −1). **FIXED:** `Detection::score` is
  `std::int32_t` (0..65535 raw, −1 "not an Object"). After: (c) 65535 → 65535. Tests: 65535, 40000, 0, 32767, 32768
  through the adapter; the type's range pinned in the seam's test.
- **(10, minor) the public docs named `vision::CameraFeed`, an A2 type.** RE-CHECKED (robot.hpp :140, :759).
  **FIXED:** both now say "one read per period, from its own loop" (the header: "the shared camera feed is the
  camera's next part"); `docs/api/` regenerated (no `CameraFeed` left outside docs/internal).
- **A1-R1 — FOUND WHILE FIXING (1) and (3), not in any finding: `bearing()` took ±kBoxEdgePx/2 of CENTRE jitter.**
  HA-202 bounds each EDGE by ~2 px; a box's centre is the mean of two edges, so it moves 2 px when both move the same
  way — half an edge under-widened it. Probe `bearing_jitter_probe.cpp` (each edge jittered 0.98 × kBoxEdgePx with the
  host model's rounding, the true camera at 98 % of every tolerance): the pre-fix code **45 of 32,201** bearings miss;
  with only the focal lengths taken apart **15 of 32,201**; fixed (±kBoxEdgePx) **0**. Fixed in `bearing()` and in the
  size range's column; pinned by the "jitter LANDED ON" test (the centre moved exactly kBoxEdgePx puts the true bearing
  exactly on the interval's end) and P2.2's jitter; mutation MZ7 (the old half-edge).

### The fixes — tests (the tests reviewer's survivors, each now with a test that lands on it)
- **X8/X9** (an Object's top and width): P1.2 checks all four box values and the angle. **X12/X12b/X13** (the lens's
  LEFT offset): `mount()` takes `left`; P2.2 mounts the lens 0 / +3 / −3 in with the robot at headings −2.5..+2.5 rad
  across the field; hand anchors (a point ahead of an offset lens is on the centre column; heading 90° puts the lens at
  (−left, forward)); the host camera model's own left term mutated too (MP5). **X3/X4/X5** (the read-back's red, green,
  id): one sub-case per field (only red / green / blue / the id off by one). **X1/X2** (DEL): 0x7F in the adapter's name
  case and the Robot's boot-line case. **X14/X15** (boundAtImage's ≥ 0): a pose stated EXACT (both 0) widens to exactly
  |v|L, |w|L, and stays 0 parked. **X19** (the margin): a point whose worst corner's column is computed through the
  same projection — a margin of exactly that is in view, `nextafter` it is not. **X7** (readAt): a `TickingClock`
  (1 ms per reading) — readAt equals the clock's LAST reading inside read(), for an Ok and a ReadFault frame. **X6** (the
  objects guard): P1.1b reads after 1 s — the check runs (2 mode reads), no name is read, the words stay "set up,
  objects off". **X16**: the radius moves by exactly the mount's tolerance change. **X17** is obsolete (the tilt term is
  gone with the rewrite) — replaced by MZ1/MZ2 and the property cases. **X20** (elevation's jitter): P2.2 checks the
  bottom row's elevation with every edge jittered, and the landed-on case. **X18**: an H-DRIVE Robot (tracking-wheel
  odometry — a holonomic drive has no two sides) with a camera — its banner ends with the camera's words, and "camera
  ABSENT" with none. **X10**: a CODE is now withheld (MW2). **X11**: a red-only (and a blue-only) change in
  `sameAiVisionSettings`. **X21**: the check's timer re-arms after a check that HELD — three reads inside the next
  second make no mode or colour read. **X23**: a 600-character refusal is cut to 159 characters, its first 159.
- **The progress log's own over-claim, corrected:** the 09:45 entry's "The equality edges each have a test that lands on
  them exactly" (above) was true only for the edges it listed (0.5 s, 1 s, 24, the clip margin, 36 in, 5 px); five
  more (DEL, boundAtImage's two ≥ 0, the in-view margin, and readAt's "completed") had none — each has one now.

### The fixes — documents (the docs reviewer)
- **(D1, major) "a screened read is answered with the last good value" was false for the camera.** FIXED in
  `hal/read_faults.hpp` (the class doc — a camera's count means "a read in between was not evidence", nothing held —
  a new header section, and the implementer list gains `ProsAiVision`), `docs/api/read_faults.md` (regenerated), the
  glossary's "read fault", guide 06's "Every adapter ... holds its last good value" (now the scalar sensors' rule, the
  camera's frame saying it failed). **FOR THE COORDINATOR — a LOCKED decision, not edited here (pace rule 2):** the
  master plan's decision #16 ("Every `hal/pros` adapter screens ... then holds its last good value ... and exposes a
  `faultedReads()` counter", locked 2026-08-13) is contradicted by `ProsAiVision` for its frames. Proposed amendment,
  for the team lead's word: *"... holds its last good value — except an adapter whose seam carries its own validity
  (the AI Vision camera's frame, VIS1-A1: a failed read is a frame that says so, with no detections, because a held
  frame would read as a fresh 'nothing changed') — and exposes a `faultedReads()` counter."* (The GPS already carries
  a validity bit, `hasFix()`, while holding its pose.)
- **(D2, major) the registry-kind belief stated as fact; HA-141 cited for it; "the next free is HA-204".** FIXED: the
  changelog's census sentence is conditional ("should list ... IF the brain's registry reports an AI Vision sensor as
  `E_DEVICE_AIVISION` (unmeasured ...)"), as is guide 10's "AI-VISION" sentence; `hal/device_provider.hpp`'s camera
  section no longer cites HA-141 — it states the belief as UNMEASURED ("no census has yet printed a word that tells
  the two apart"). **CORRECTION to the "FOR THE COORDINATOR" entry above:** the tip (`shulib-v2`, now `d8fdb8b`) holds
  HA-204..HA-215 for L6 and IMU1; **the next free number is HA-216** (the tip's PROJECT-BRIEFING :116). The choice stays
  the coordinator's: widen HA-141's claim to the AI Vision kind (and MECH-A1's distance and optical kinds, which it was
  already stretched to), or number the belief at HA-216 — then the provider's sentence gains the label.
- **(D3)** HA-70: "(a library adapter for the camera's objects and colours exists since VIS1-A1, host-tested only; no
  program reads robot two's camera until VIS1-B declares it; its TAGS are read by nothing)".
- **(D4)** `A4 register HA-193` now sits at the by-value copy in `ProsAiVision::read()` (the box's four values).
- **(D5)** guide 10 and the glossary: "the camera sends at most 24 and may hold more"; HA-197: "a name the SDK does
  not write reads as '' (the buffer is zeroed before each read); bytes it does write that are not the expected name
  read as Different — safe either way".
- **(D6)** the twelve index rows' Owner column and the twelve entries' *Settle* labels rewritten — not only HA-192 and
  HA-201: every "Station I 0.1/0.2" was a no-code step — "Step 0.1 / 0.2 (no new code ...)" and "Station I 0.N after
  VIS1-B" for 0.5–0.8; the four plain "invented" bolded (an awk count over the index: 104 bold, 2 plain — HA-15 and
  HA-111, older rows outside this chunk, left as found); the section's intro DEFINES Step 0 (0.1 tape and level, 0.2
  station 12 as it stands, 0.3 the AI Vision Utility, 0.4 the live picture; then Station I with VIS1-B, 0.5–0.8); the
  two runtime refusals no longer say "(Step 0.3)" / "(Step 0.1)" — "UNSET -- measure it with VEXcode's AI Vision
  Utility and type it", "the camera's mount is UNSET -- measure it from the robot's tracking centre and type it" (four
  test pins and two test comments updated).
- **(D7) the programs' boot lines ("has no adapter yet" / "no camera adapter exists yet") — NOT fixable here:** `src/`
  must stay untouched (`git diff --stat ceb17c0 -- src/ tools/` EMPTY is this run's rule). **Correction to the two
  entries above that called them "still TRUE in substance":** they are literally STALE — an adapter exists; no program
  reads a camera. **FOR THE COORDINATOR:** accept them until VIS1-B, or make a separate, approved one-line change in
  `src/programs/drive_program.cpp` :211-213 and `src/programs/xdrive_program.cpp` :144-146 at the merge ("no program
  reads the camera yet; its adapter exists since VIS1-A1").
- **(D8) merge-time staleness against the tip — FOR THE COORDINATOR (re-verified with `git show shulib-v2:` at
  `d8fdb8b`):** (1) the tip's changelog opens "## API 2.5" (:17) above "## API 2.4" (:152) — the entry moves under
  2.5 as "Added since 2.5", still additive; (2) the tip's register has **178** rows ("7 of 178") — after the merge
  **190**, not 189; (3) the tip's Held line ends "HA-192..HA-203, HA-204..HA-215" — drop ONLY "HA-192..HA-203";
  (4) "2026-10-22" as a landing date — `docs/changelog.md` :19 (the entry's heading), `docs/roadmap.md` :1135 (the R2
  amendment) and `docs/internal/HANDOFF-remaining-work.md` :269 (a grep of the worktree, 2026-10-29; HA-70's no longer
  carries one) — set to the real landing date; the register's "built 2026-10-18–22" is a build span, true as it
  stands, and the build continued on 2026-10-29 with these fixes; (5) the register's VIS1-A1 section sits
  before MECH-A1's (logged, to stay clear of MECH-A3's appends) — move it after MECH-A3's; (6) A1-D13's status lines
  (build-order, REMAINING-WORK row 18d) and the briefing regenerated from a fresh build.

### RESUMED (after the coordinator's stop at ~15:10 and the laptop's reboot at 17:58) — `date` Thu Oct 29 06:16:04 PM EDT 2026
Executor: opus (the builder again, the same workflow run continued). The fix run above was stopped while its mutation
runner was on **MV7**; nothing after "(D8)" above was written by it. What this resumption found, and did, in order:

- **The tree as left.** `git status`: 53 entries (34 modified, 19 untracked paths — the 50 at the stopped run's start, plus the three tracked files its D1 touched: `hal/read_faults.hpp`, `docs/api/read_faults.md`, guide 06); `git diff --stat ceb17c0 --
  src/ tools/ Makefile` EMPTY. **No mutant applied:** the stopped run's 33 backups
  (`scratch/vis1-a1/review-fixes/bak/`, all `ai_vision.hpp`, M1 … MV7) are each byte-equal to the live file (`cmp`);
  a check of BOTH mutation tables (`review-fixes/mutate_fix.py`, 98 rows, and the 10-22 `mutate3.py`, 51 rows) found
  every original anchor present and no mutant text, apart from three substring hits that are not mutants (M4's `f.status
  = FrameStatus::Ok;`, M6's `if (!configured_) {`, MF1's `return &cameras_[i];` each also occur, unmutated, elsewhere in
  their files — each row's original anchor is there exactly once). No gate self-test plant was left: `src/`, `tools/` and
  the `Makefile` have no diff.
- **The binary on disk WAS a mutant link:** `build/test/shulib_tests` 15:01:05 — after MV7's backup (15:00:55), before
  the restore (15:01:25). Removed; `cmake -S test -B build/test`; a full `cmake --build` (the ten objects that include
  `ai_vision.hpp` recompiled); the suite under `ulimit -v 4194304; timeout 600`: **2562 / 2562 passed, 2,796,105
  assertions, 0 failed** (61 MB peak; 86 s — four other jobs' suites were running, load ~29). THEN
  `briefing_status.py generate`.
- **`review-fixes/post_run_edits.py` had never run** (it was held for the end of the mutation run; all four old anchors
  were still in place). Its facts re-checked first (the host model's default 74 x 63, the shim's `colourReadFails` and
  `codeObject()` exist), then applied. Why it matters, re-measured: a STRICT direction-2 check (each id must appear in
  each Source file somewhere OTHER than as one end of a range like "HA-192..HA-198") found **HA-192 in
  `hal/pros/ai_vision.hpp` only as a range end** — the docs reviewer's HA-193 defect (D4) a second time. The held edit
  labels it at the read ("A4 register HA-192: no frame counter and no timestamp, so the frame is stamped when the read
  COMPLETES"). The strict check now: 0 missing (script `scratch/vis1-a1/review-fixes2/dir2_strict.py`); the plain
  direction-2 check: 0 missing; direction 1: nothing printed.
- **Three comment blocks the stopped run left ragged were re-flowed** (`ai_vision.hpp`'s READ paragraph,
  `camera_plant.hpp`'s independence and not-modelled paragraphs, `camera_geometry.hpp`'s tolerance paragraph and the
  `sizeRange()` / `fieldFix()` docs) — words unchanged. `docs/api/` regenerated (`api_doc_tool.py generate`;
  `check-fresh` 0).
- **Each disposition above RE-VERIFIED against the tree, not taken from the log.** The code read whole for (2), (5),
  (6), (7), (8), (9) and X7 (`hal/pros/ai_vision.hpp`: `verify()` → Holds / Changed / Unanswered, the model re-asked
  at every check that held, `colourReadFailed()`, `wroteColour()`, the `int32_t` score, `readAt` at completion) and
  for (1), (3), (4) and A1-R1 (`vision/camera_geometry.hpp`). **The size range re-derived by hand:** a point at
  horizontal range R and bearing β off the axis's vertical plane, Δz above the lens, has depth R cos β cos p + Δz sin p
  and image-down R cos β sin p − Δz cos p, so (v − cy)/fy = yd gives Δz = R cos β × (sin p − yd cos p)/(cos p + yd sin
  p) — the code's `rise(v)` — and H = R cos β (rise(top) − rise(bottom)): exact for a vertical line. Its column: a 3-D
  line projects to an image line, so the box's centre (the mean of its ends) lies on the line's image and its ray
  has the line's bearing — so `sideways = cos(b − yaw)` from the centre column is exact too. The fix radius's heading
  term: each corner's point turns rigidly about the robot's centre, its distance from the nominal point unimodal in
  the turn — farthest at an end of the interval or, when the interval reaches it, diametrically opposite (|v| + |n|):
  as coded. Every test the entries name exists (the test-case list read; the X-series lines grepped); D1–D6 grepped
  in `read_faults.hpp`, the glossary, guide 06, guide 10, the changelog, `hal/device_provider.hpp`, the register's
  index rows, its Step 0 intro, HA-70 and HA-197; no `CameraFeed` outside `docs/internal/`.
- **Every probe re-run on THIS tree** — compiled fresh against the worktree's headers (`-DSHULIB_HOST_PROS_SHIM` for
  the shim's), binaries under `scratch/vis1-a1/review-fixes2/probes/`, each under the 4 GiB cap: the reviewer's
  `adapter_edges` (a) ReadFault, faultedReads 1, settingsLost 0; (b) count 0; (c) 65535 → 65535, 40000 → 40000; (d)
  inf → inf, NaN → inf · `p22_independent` 0 / 0 / 0 of 3,824 · `size_offaxis` 0 of 26 · `size_sweep` 0 at −10°, 0°,
  +10° · the stopped run's `size_probe` (a) 0 of 18,495, (b) 139 of 32,881 (the top-face fact above), (c) 0 of 32,881
  · `bearing_jitter_probe` 0 of 32,201. Each matches the "after" the entries above give.
- **The mutations.** The stopped run's 32 verdicts above (M1–M14, M6b, M7b, M11b, X1, X3–X9, X21, MV1–MV6 — 14:45 to
  15:00, `review-fixes/mutations.log`) each ran on a deleted binary and a full `cmake --build`, every one RED; they
  stand for the final code (since then `ai_vision.hpp` changed only in comments; no test file changed between 14:42
  and the stop, and this resumption only ADDED one test case (A1-R2), which removes nothing those 32 failed on).
  MV7 was in flight at the stop. **The remaining 66 ran here** (`scratch/vis1-a1/review-fixes2/mutate_fix2.py` — the
  stopped run's table with its own backup and log paths; each: backup, ONE exact replacement keeping the line count,
  the binary deleted, a full `cmake --build`, the whole suite under `ulimit -v 4194304; timeout 600`, the file restored
  in a `finally` and touched): **65 RED, 0 survived**; the cases failed per mutation — MV7 1 · MC6 1 · MW1 1 · MW2 1 ·
  MW3 1 · MW4 1 · R1 5 · R2 1 · R3 6 · R4 1 · R5 2 · R6 4 · MG1 1 · MG2 8 · MG3 4 · MG4 5 · MG5 2 · MG6 2 · MG7 1 ·
  MG8b 1 · MG9 1 · X12 2 · X12b 2 · X13 1 · X14 1 · X15 1 · X16 1 · X19 1 · X20 2 · X22 2 · MZ1 3 · MZ2 2 · MZ4 1 ·
  MZ5 1 · MZ6 1 · MZ7 1 · MZ8 1 · MF3 1 · MF4 1 · MF5 1 · MF6 1 · MC1 1 · MC2 1 · MC3 1 · MB1 1 · X2 1 · MB2 2 · MB3 1
  · MB4 1 · X18 1 · X23 1 · MF1 2 · MF2 1 · MD1 1 · MS2 1 · MS3 1 · MS4 3 · X11 1 · MFV1 1 · MFENCE 30 · MP1 8 · MP2 1
  · MP3 1 · MP4 1 · MP5 2. **MI1 is NOT a verdict:** its mutant (`n.fy * sx`) left `sy` unused and `-Werror` refused
  the BUILD — the mutation's own breakage, not a test. Re-run as **MI1b** (`n.fy * (sx + 0.0 * sy)`: the factors tied,
  `sy` still read): RED by assertion — P2.2 holds 476 of 477, and P2.3's (fx down, fy up) and (fx up, fy down) corner
  counts read 0, not 8. **X23 re-run with its output kept (X23r):** RED by the ASSERTION (`kept.size() == 159U` read
  608 — the unbounded copy wrote 608 bytes into the 160-byte buffer without crashing; the test, not a crash, caught
  it). Every RED in this pass is a test's runtime assertion; none was counted from a build failure. The equality
  edges among them, each landed on exactly: X1/X2 (0x7F), X14/X15 (a tolerance of exactly 0), X19 (the margin, and
  `nextafter` it), M6b/M7b (0.5 s and 1.0 s, binary-exact), M11b (an id equal to the count), M14 (a count of 24), MG9
  (the size range's too-small edge), MC1 (36 in), MS2 (the clip margin). Restores: the 66 backups byte-equal to their files when that run ended, the 3
  supplementary ones when theirs did (`review-fixes2/bak/`, `cmp`); no "RESTORE MISMATCH" logged.
- **Two mistakes of mine in this resumption, logged.** (i) The first launch of the 66 came right after the comment
  re-flows and BEFORE `docs/api/` was regenerated: the build's doc gate (`check-fresh`) failed, so 19 mutations logged
  "BUILD FAILED (not a verdict)" — all VOID, none counted. (ii) Stopping that launch, `pkill -f mutate_fix2.py` also
  matched my own shell's command line; the runner died by SIGTERM in its handler, which restored X20's file (all 29
  backups byte-equal, checked). `docs/api/` regenerated, a fresh baseline built and run green (2562 / 2562), and the
  66 restarted from the top.
- **A1-R2 — FOUND IN THIS RESUMPTION, not in any finding: the host test plant could not emit the score range the seam
  now carries.** Finding (9) widened `hal::Detection::score` to `std::int32_t` (0..65535); `sim::Element::score`, which
  the host camera model copies into each Object's detection, was still `std::int16_t` — no scene could put a raw score
  above 32,767 in front of a consumer, the plant's side of the same defect. **FIXED:** `std::int32_t` ("its detection's
  raw score — 0..65535, the seam's range (HA-198)"); a new test (`test/sim_camera_plant_test.cpp`, "an element's score
  reaches the frame WHOLE") puts 0, 32767, 32768 and 65535 through the plant and checks the type's range — the value
  goes in through the field's own type (a no-op cast), so a narrowed field fails the test's CHECKs, not the build.
  Mutation **MP6** (the field back to `int16_t`): RED by assertion — the range read 32767, 32768 → −32768, 65535 → −1.
  Prose review for it: `grep -rn -i "camera_plant\|host camera model\|CameraPlant"` and a score grep over `docs/`,
  `README.md` and `test/README.md` — the hits (changelog :96-101 and :132-138, the register's Source fields,
  test/README :100-105) state nothing about the score's type; none stale. `sim/` has no `docs/api/` page.
- **(D8) re-checked against the tip as it is now — `shulib-v2` at `e5a69b5` (CAL1 Part A2a), not `d8fdb8b`:** the
  changelog opens "## API 2.5" at :17 and "## API 2.4" at :266; the register has 178 rows ("7 of 178" — 190 after the
  merge); its Held line ends "HA-192..HA-203, HA-204..HA-215" (drop ONLY the first); the briefing says the next free is
  HA-216 (:116); MECH-A3's register section is at :1265 (the VIS1-A1 section goes after it). Every other item of (D8)
  stands as written. The "2026-10-22" landing dates in this worktree: `docs/changelog.md` :19, `docs/roadmap.md` :1135,
  `docs/internal/HANDOFF-remaining-work.md` :269 (re-grepped).

### The gates on the final tree (this resumption, after every edit — `date` 19:15–19:40)
- **The suite**, from a binary REMOVED and relinked by a full `cmake --build` (18 objects recompiled, 0 warnings),
  under `ulimit -v 4194304; timeout 600`: **2563 / 2563 test cases passed (3 skipped, as before), 2,796,118
  assertions, 0 failed** — 36 s, 62.5 MB peak. THEN `briefing_status.py generate`; `check` current.
- **Doc gates:** `api_doc_tool.py` self-test OK · check-coverage (4,952 public entities across 188 headers, all
  documented) · check-fresh 0 · check-examples (541 quoted lines verbatim) · check-removability (no public doc
  references docs/internal/) · `briefing_status.py` self-test OK, check current · `doc_staleness_audit.py` self-test
  OK, audit clean.
- **The CI greps:** the library PROS-free outside `hal/pros/`; the core sim-free. **The register:** direction 1
  prints nothing; direction 2 (plain and STRICT) 0 missing for HA-192..HA-203.
- **The ARM header gate:** all 230 headers in one TU, `arm-none-eabi-g++ -std=gnu++20 … -Werror -mcpu=cortex-a9
  -mfpu=neon-fp16 -mfloat-abi=softfp` — rc 0 (the TU under `scratch/vis1-a1/review-fixes2/`).
- **Compile refusals:** `ctest -R compile_refusal` — 200 / 200 passed; 200 TUs = 200 CTests.
- **`src_build_gate.py self-test`:** OK (235 detector cases, each against a real build). **`check`:** PASS — all 7
  builds link, 0 disallowed warnings. Afterwards `git diff --stat ceb17c0 -- src/ tools/ Makefile` is EMPTY (no plant
  left).

### The tree at the end of this resumption — `date` Thu Oct 29 07:40:48 PM EDT 2026
- **Nothing committed** (branch `vis1-a1`, base `ceb17c0`): 34 modified, 19 untracked — this chunk's files only, the
  regenerated `docs/api/` and `mkdocs.yml`, and `PROJECT-BRIEFING.md`'s generated block. `src/`, `tools/` and the
  `Makefile`: no diff. **No mutant applied:** of this day's backups (33 under `review-fixes/bak/`, 69 under
  `review-fixes2/bak/`), 64 are byte-equal to their files and the other 38 differ ONLY by this resumption's
  intended edits made after they were taken — the 33 stopped-run backups from `ai_vision.hpp` by one identical
  diff of comment lines alone (the held labels and the re-flow), MP1–MP5's from `camera_plant.hpp` by A1-R2's score
  line alone (`diff`, read); both tables' anchors checked present, no mutant text. The suite binary on disk is the 19:15:34 fresh link of this tree.
- **Changed by this resumption:** `hal/pros/ai_vision.hpp` (the held HA-192/195/198/203 labels; the READ paragraph
  re-flowed), `sim/camera_plant.hpp` (the held HA-193/194/199/201 labels; two paragraphs re-flowed; A1-R2's
  `int32_t` score), `vision/camera_geometry.hpp` (three comment blocks re-flowed, words unchanged), `test/README.md`
  (the held shim paragraph: `colourReadFails`, `codeObject()`), `test/sim_camera_plant_test.cpp` (A1-R2's test),
  `docs/api/` regenerated, this log. Not touched, by design: the shared plan docs (A1-D13's status lines and the (D8)
  items are the coordinator's at the merge), `src/` (D7), VIS1-A2 and VIS1-B.
- **Still FOR THE COORDINATOR (each stated in full above):** decision #16's amendment (D1, waits for the team lead's
  word — pace rule 2); the registry-kind belief — widen HA-141 or number it HA-216 (D2); the programs' stale boot
  words (D7); the merge-time items (D8, re-checked at `e5a69b5`); and, for A2's brief (§1.13 rule 1), the size range
  of a Pin seen from above reading up to about a radius and a half NEARER than its near rim (finding (1)'s
  disposition).

---

## Verification-round findings and their disposition

- **Started:** Thu Oct 29 08:26:52 PM EDT 2026 (`date`). Executor: opus (the builder again, the same workflow run).
  Tree: the uncommitted VIS1-A1 work in the worktree `vis1-a1` (base `ceb17c0`); `git diff --stat ceb17c0 -- src/
  tools/ Makefile` EMPTY; the suite binary on disk the 19:15 link of the fix round's final tree.
- **The task:** a fresh verifier returned six findings (one major, five minor) against the fix round's tree; each is
  re-checked here, fixed at the root when real or answered with evidence when not, every gate re-run. Nothing
  committed. Probes and mutation backups under `.claude/coordinator/scratch/vis1-a1/vr2/` only.


### RESUMED (this verification-fix run was stopped partway; the session ended) — `date` Fri Oct 30 08:33:03 AM EDT 2026
Executor: opus (the builder again, the same workflow run continued). What this resumption found, and did, in order:

- **The tree as left.** `git status`: 53 entries (34 modified, 19 untracked), the same as at the round's start;
  `git diff --stat ceb17c0 -- src/ tools/ Makefile` EMPTY (no gate plant). The stopped run had written only
  this section's two header lines. Its work on disk, found by diffing the verifier's private copy of the tree
  (`scratch/vis1-a1/verify/tree/`, taken 2026-10-29 19:43, after the fix round ended at 19:40) against the
  worktree — the ONLY differences are five files, each an intended edit read in full here:
  `hal/pros/ai_vision.hpp` (20:28 — `colourReadFailed()` the EXACT failure record, `#include <cmath>`, the
  header's 1 Hz paragraph), `test/pros_ai_vision_adapter_test.cpp` (20:29 — three new test cases, the file's
  header, the P1.18 comment), `docs/hardware-assumptions.md` (20:29 — HA-196's claim), `hal/vision_frames.hpp`
  (20:30 — `ReadFault`'s doc, the seam's read() cost) and this log. It had then snapshotted `include/` to
  `scratch/vis1-a1/vr2/before_include/` (20:31; byte-equal to the worktree's `include/` now) and written a
  geometry brute-force probe (`vr2/probes/geom_brute.cpp`, its outputs `geom_before_doc.txt` and
  `geom_before_full.txt`, 20:36–20:41). **No mutation had run** (`vr2/bak/` empty).
- **No mutant applied.** The verifier's 51 backups (`verify/bak/`) are byte-equal to its tree (its own
  mutations restored), and its tree equals the worktree apart from the five intended edits above; the fix
  round's own backups (`review-fixes/bak/`, `review-fixes2/bak/`) equal the verifier's tree except the 33
  `ai_vision.hpp` and 5 `camera_plant.hpp` ones the previous resumption already logged as differing only by
  intended edits.
- **The binary on disk was the 19:15 link** — older than the stopped run's edits. Removed; `docs/api/`
  regenerated (`vision_frames.hpp`'s `///` text had changed); the briefing regenerated with the binary
  absent; `cmake -S test -B build/test`; a full `cmake --build` (0 warnings); the suite under `ulimit -v
  4194304; timeout 600`: **2566 / 2566 passed (3 skipped), 2,796,344 assertions, 0 failed** (18 s, 61.8 MB
  peak) — the stopped run's three new test cases among them; THEN the briefing regenerated.

### The six findings, each re-checked against the tree, then dispositioned
**All six are REAL.** Two (4, 5) were wider than stated; one more doc gap was found by this round's prose
review (under 6).

- **(1, major) The 1 Hz check's four-field compare was pinned by no single field.** RE-CHECKED: the only
  tests of the check's colour compare changed red, green AND blue together (slot 3 to {3, 10, 10, 10}) or
  green AND blue (the swap's stray). **FIXED (the stopped run's test, re-verified by reading it):** "the 1
  Hz check compares EACH field on its own" — descriptor 1 {1, 200, 30, 40} stored back with ONLY red +1,
  ONLY green +1, ONLY blue +1, ONLY the id (2): each a changed setting (settingsLost 1, setups 2, never a
  faulted read, the slot written again, every field); ONLY the two floats changed: not a change (HA-196:
  never compared). Mutations VR, VG, VB, VI (each drops one term of `verify()`'s compare): see below.
- **(2, minor) Any read-back id outside 1..7 was taken for "the camera did not answer".** RE-CHECKED in
  the disassembly (`scratch/vis1-a1/objdump/ai_vision.dis` :236-:276): the claim-fail branch (`cbz r0, 60`)
  loads 127 into the id, red, green and blue and 0x7f800000 (+Inf) into both floats; the success branch
  copies out whatever `vexDeviceAiVisionColorGet` wrote into an uninitialised stack record (`sub sp, #20`;
  `mov r2, sp`; `ldrb r6, [sp]`) — what the SDK reports for an empty slot is unread. **FIXED at the root
  (the stopped run's code, re-verified):** `colourReadFailed()` is true for EXACTLY that record — id, red,
  green, blue all 127 and both floats +Inf (finite or −Inf refused) — and every other answer, an empty slot
  included, is COMPARED (a changed setting in the 1 Hz check; "a colour read back different from what was
  written" in the set-up). No belief is added: HA-196's claim now says only the exact record is a failed
  read and "whatever the SDK reports for an empty slot … is unread, and no belief about it is needed"
  (re-read). The Settle text ("settingsLost — a camera re-plugged between two reads that came back without
  its settings") is now what the code does. Tests (the stopped run's, re-verified): every slot wiped with
  the modes kept → Ok, settingsLost 1, faultedReads 0, re-written; a write that "succeeds" into an EMPTY
  slot and stores nothing → "a colour read back different …", never "did not answer"; the record and each
  of its eight one-field-off variants (id/red/green/blue 126, a finite hue, a finite saturation, −Inf hue,
  −Inf saturation) met by the 1 Hz check — only the exact record a ReadFault; the set-up's read-back of the
  record ("did not answer") and of the record with green 126 ("different"). Mutations CRW (the old wide rule
  back), CRID, CRR, CRG, CRB, CRHI, CRHS, CRSI, CRSS (each term of the record dropped): see below. The
  verifier's probes re-run on this tree: see below.
- **(3, minor) Descriptor id 7 — the bound's equality edge — was never configured.** RE-CHECKED: no test
  wrote descriptor 7 (the only 7 a port). With (2)'s fix the adapter has no id range any more — the edge
  CRF7 mutated is gone — but id 7 still has to work end to end. **FIXED (the stopped run's test):**
  "descriptor 7 — the last id the camera has — configures, holds at the 1 Hz check and its blobs pass":
  a settings block {7, 1}, configured, slot 7 written, a colour-7 blob passed at 0 s, 1 s and 2 s (the check
  at exactly 1 s and 2 s), settingsLost 0, faultedReads 0, setups 1, six colour reads (two at the set-up,
  two per check, each held). Mutations CRF7b (id 7 taken for the failure record — the CRF7 defect expressed
  against the new code) and VF7 (the settings' own range check refusing 7): see below.
- **(4, minor) `fieldFix()`'s radius under-bounded at heading tolerances near and past 180 degrees.**
  RE-CHECKED, and the derivation redone whole rather than patched: the radius is the farthest point of the
  set {robot centre + R(h)(lens + r·dir(b)) : b, r, h in their intervals} from the nominal point. Along r
  (a ray from the lens) the distance is convex, so r is at an end; for a fixed turn h the farthest point
  of the bearing-range sector is a corner or the FAR arc's point straight away from the nominal point as
  seen from the lens (distance |nominal − lens(h)| + range.hi, when that bearing is inside the interval);
  for fixed (b, r) the farthest turn is an end of the heading interval or the turn diametrically opposite
  the nominal point. So the maximum is one of: a corner at a heading end (counted before); a corner at its
  opposite turn (counted since the review); the far arc's point at a heading END (NEW); or the far arc's
  point at a turn interior in both b and h — where it must also be opposite the nominal point, which puts
  the lens on the line through the robot's centre and the nominal point (NEW: those two turns, each when
  inside the heading interval; the verifier's scene is the lens-behind one, 7 + 31 + 37 = 75 in). Every
  candidate is a reachable point, so the maximum over them is EXACT, not a widened bound — the verifier's
  suggested |lens| + range.hi + |n| would have been a bound, but loose wherever the lens's direction is
  outside the bearing interval. **FIXED** (`vision/camera_geometry.hpp`, `fieldFix()`: a `farArc(h)` at the
  two heading ends and at the two lens-on-the-line turns; the doc now names all of them). **The test's
  expected value was wrong too** and is corrected: ≥ 180 degrees → 75 + 0.25 (and > the old corner value
  by more than 0.25 in); 179 degrees → the far arc at the interval's END, hypot(7 cos 179° − 37, 7 sin
  179°) + 31 + 0.25, landed on to 1e-12. **A new sweep test** ("the field fix's radius is EXACT"): 450 fixes
  — five lenses (ahead, ahead-and-left, behind-and-right, AT the centre, beside), three bearing intervals,
  three range intervals, ten heading tolerances 0..359 — each swept over 25 bearings x 3 ranges x 361 turns:
  the radius less the mount's tolerance holds the farthest swept point (worst short 7.1e-15 in — rounding)
  and is never more than 0.05 in beyond it (worst over 0.0029 in — the sweep's own step). Mutations FA1–FA7:
  see below.
- **(5, minor) The header said every output was monotone over the tolerance box "while the box does not
  straddle the horizon; one that does is refused" — nothing refused, and three outputs were not.**
  RE-CHECKED with the stopped run's brute-force probe (`vr2/probes/geom_brute.cpp`, the true camera on a
  grid INSIDE the box with each tolerance at 100 %, the truth from the probe's own projection) and my own
  probes (`vr2/probes/mono.cpp`, `mono2.cpp`). On the tree as it stood (`geom_before_doc.txt`, each
  function's documented jitter only): the BEARING missed near the level row (944 of 178,104 at pitch 0 ± 1°,
  worst 5.5e-5 rad; 2,652 at ± 5°, worst 1.4e-3 rad); the SIZE RANGE missed for boxes straddling the level
  (13,936 of 2,740,068 at pitch 0 ± 5°, worst 0.68 %) — AND, not in the finding, for a box whose jittered
  column reaches the principal point's (204 misses at pitch ± 44, column 160.4, rel 1.4e-4: the line of sight
  is square to the image there, the sideways factor's peak, interior to the column jitter); elevation and
  ground range 0 misses anywhere. Analysis (logged in the code's docs): the bearing's forward part cos p + yd
  sin p peaks at the pitch atan(yd) where row v's ray is level; the size range's product P = sideways x
  (rise(top) − rise(bottom)) has d ln P/dp = tan e_top + tan e_bottom + k f'/f (k = xr²/(f² + xr²)) — it is
  LOG-CONVEX in the pitch (d² ln P/dp² ≥ 1.6 over 400,000 random boxes at every pitch within ±55°,
  `mono.cpp`), so its maximum (the NEAR end) is at a pitch end and its minimum (the FAR end) at an end or
  where that slope crosses zero; for every pitch it is smallest with the box shrunk by an edge at each end
  (the rise falls with the top row's jitter and rises with the bottom's) and the column farther from the
  axis, and monotone in each focal length (0 interior extremes over 300,000 boxes at every |pitch| up to
  56.5°, `mono2.cpp`; at 57° an edge row's ray turns backward). **FIXED at the root — the intervals made
  EXACT, the claim corrected, a domain stated and enforced:** (a) `bearing()` also evaluates, per corner,
  the pitch atan(yd) when it is inside the tolerance; (b) `sizeRange()` also evaluates the principal point's
  column when the jittered column reaches it (`|u − cx| < kBoxEdgePx`), and searches the interior pitch —
  bisection (40 steps) on the log's slope, for the shrunk box at the column farther from the axis, at each
  focal-length corner, only when the slope changes sign across the tolerance — taking the convexity
  tangent's value at the bracket's far end, a bound from below on the product, so the far end is never
  short (the slope is a closed form: `sizeTerms()`); (c) the header says what is now true — every interval
  the EXACT range over the whole box, evaluated at the corners and at the named interior points, each
  function's doc naming its own — while the pitch WITH its tolerance stays inside ±`kMaxPitchSpanDeg` (55);
  (d) `vision::cameraModelProblem()` refuses a mount past that span ("the pitch with its tolerance reaches
  past 55 degrees -- …"): the GEOMETRY's own limit, so the robot file still validates and the camera is
  still built and read (D5; `config::cameraPlacementProblem()` unchanged — a test checks it still accepts
  that mount). The verifier's alternative, refusing a straddling box, would have refused exactly the raised
  piece near lens height that A2's floor-or-raised question needs. **New tests:** "every interval is the
  EXACT range over the WHOLE tolerance box" (48 points and 96 boxes over three pitches x two tolerances x
  four columns — two of them reaching the principal point's on the sweep's 0.5-px grid — x the level row and
  row 200; each focal length at both ends AND its middle, 41 pitches, both yaws, nine columns, three jitters
  per edge: each interval holds the swept extremes and reaches them, bearing within 1e-6 rad, elevation
  1e-9, size range 5e-6 relative); "a point on the lens's LEVEL" (the bearing's near-axis end landed on BY
  HAND, to 1e-12: yaw low, the column toward the axis, fx high, f = sqrt(1 + yd²) at fy low — and the
  pitch-ends-only value 1.2e-4 rad off it); "a box STRADDLING the lens's level near the picture's EDGE" (the
  size range's far end landed on to 1e-12 against this test's own golden-section search over the pitch; the
  rise alone's pitch falls 4.5e-7 short there, so a search ignoring the sideways factor fails it); "a thin
  line STRADDLING the lens's level" (1,944 lines through every true camera of the box — the pitch at nine
  points of each tolerance, inside included, the focal lengths at 100 %, both edges grown and the column
  moved toward the axis by exactly kBoxEdgePx — projected by the test's OWN pinhole at sub-pixel precision:
  every one inside its size range); `cameraModelProblem()`'s new edge (44 + 11 = 55 builds, binary-exact;
  the next double past 55 refuses, for ±44; 0 + 56 refuses). Mutations BI1, BI2, SZ1–SZ10, PS1–PS3: see
  below. The stopped run's probe re-run on the final tree: see below.
- **(6, minor) Docs left stale by the fix round's (5) and (7); no prose review logged for its source
  changes.** RE-CHECKED: `FrameStatus::ReadFault`'s doc said only "the count read failed (PROS_ERR)" and
  the seam's read() cost named only the count and the detections; `grep -n -i 'grep\|prose review'` over
  the fix round's part of this log finds only A1-R2's. **FIXED:** `ReadFault` — "a read failed — the count
  (PROS_ERR), or a read-back the adapter's once-a-second check could not make: NO detections — never a held
  frame" (the stopped run's); `IVisionFrames`' class doc and `read()`'s doc carry the once-a-second check
  (up to 28 more calls) — and, found by THIS round's prose review below, the SET-UP's cost, which read()
  pays too when it finds one due (two calls per colour, two for the modes, one per expected name: up to 36,
  at most every half second after a lost setting or a failed read) and no document named: added there,
  in `ai_vision.hpp`'s header, and in HA-203's claim and its index row. `docs/api/` regenerated.

### FOR THE COORDINATOR — from this round (each stated in full above)
- **A new refusal in the camera's contract (finding 5, (d)):** `vision::cameraModelProblem()` now also
  refuses a mount whose |pitch| + pitch tolerance is past `vision::kMaxPitchSpanDeg` (55) — the geometry's
  own limit, measured (an edge row's ray turns backward at about 56.7 degrees; the size range's focal-length
  monotonicity holds to 56.5). The robot file's `config::cameraPlacementProblem()` is unchanged, so no robot
  file that validated before stops validating, and the camera is still built and read (D5). Additive to the
  unreleased API 2.4 entry (the changelog says so). If the team lead would rather the robot file refuse it,
  that is a one-line move — logged, not done (pace rule 2).
- **For A2's brief (§1.13's corner-by-corner consistency):** `CameraModel::eachCorner()` visits the 16/32
  CORNERS only. This round made every INTERVAL exact by also evaluating interior points (the pitch that makes
  a row's ray level; a straddling box's interior pitch; the principal point's column), but a consumer that
  asks "is there one camera inside the tolerances that sees this box as X?" corner by corner does not visit
  those cameras — near the lens's level it can answer "no camera explains it" when an interior one does.
  That errs toward a refusal (UNSURE), the safe side; A2 decides whether to visit them.
- Every earlier item stands as logged (D1, D2, D7, D8 at `e5a69b5`, finding (1)'s top-face note for A2).

### A1-V1 — FOUND IN THIS ROUND (not in any finding): a box edge's jitter moves a point in BOTH coordinates; three functions carried one
- **What:** `bearing(u, v)` jittered only the column ("+-kBoxEdgePx of centre jitter" — a reader takes that as
  the centre, both coordinates); `elevation()` and `groundRange()` only the row. A box's centre and its bottom
  row's column move with the edges too. Measured by the stopped run's probe in its "full" mode (every
  coordinate jittered by 0 or exactly +-kBoxEdgePx, the true camera on the grid INSIDE the box at 100 %) on the
  tree after (4) and (5): at the realistic mounts (pitch −15..+10, tolerance 1..5 degrees) the bearing missed by
  up to 2.3e-3 rad, the elevation by up to 2.8e-3 rad and the ground range by up to 0.43 % (0.19 in at 45 in),
  all near the picture's corners; at pitch 44 ± 5 the bearing by 1.25e-2 rad. The header this round had just
  written ("the EXACT range … over that whole tolerance box", the box including "a box edge's jitter") would
  not have been true.
- **FIXED at the root:** `bearing()` now jitters the row too (the level pitch evaluated for each jittered row);
  `elevation()` and `groundRange()` jitter the column too, and — their own interior point — evaluate the
  principal point's column when the jittered column reaches it (the ray in the axis's vertical plane is the
  one farthest from level: |elevation| falls as the column moves off the axis); one private helper
  (`jitteredColumns()`) gives the column set to all three and `sizeRange()`. The header and each function's
  doc say so. Proved first, not assumed: the elevation is monotone in the row, the pitch, the focal lengths and
  |the column's offset| for fixed others (d e/d yd ∝ −(f + cos p · xr²) < 0; d|e|/d|xr| < 0), and the bearing
  monotone in the row through f = cos p + yd sin p (linear in yd for a fixed pitch).
- **Tests updated and added:** the landed-on test's elevation and ground range now move the bottom row AND
  the column by exactly kBoxEdgePx — only the corner that undoes both puts the truth on the end (to 1e-9) —
  and with the row alone moved the truth is now strictly INSIDE (so a column jitter dropped fails it); the
  bearing landed-on test's hand formula takes the row jittered away from the centre (and its row moved to
  122, putting the level pitch mid-tolerance: the pitch-ends-only value is 1.75e-4 rad off); the hull sweep
  now jitters the bearing's and the elevation's row AND column (9 x 5 per point) and sweeps the GROUND RANGE
  too, at both lens heights (24 of the 48 points below the level: each interval equal to the swept extremes to
  1e-12; the other 24 refused exactly where a swept ray is within 2 degrees of level). Probe "full" mode on
  the final tree: see the probes below.
- **Mutations:** BJ1, BJ2, X22r, MZ7r, X20r, EJ1, EJ2, GJ1, GJ2, GJ3, MG8br, JC (new or re-anchored), and every
  earlier geometry mutation whose code moved re-run on the final code (BI1, BI2r, SZ1, SZ2r, SZ4, SZ5, SZ7, SZ8,
  SZ9b, SZ10, MZ1r, MZ2r, MZ4r, MZ5r, MZ6r): see below.

### A mistake of mine in this round, logged
- Checking which earlier mutation anchors my edits had moved, I ran `review-fixes2/check_no_mutant.py` over
  four tables, among them `review-fixes2/mutate_supp.py` — which has NO `__main__` guard, so loading it RAN its
  three mutations (MI1b, MP6, X23r) on the live worktree, twice (the command ran the check twice), each with
  its backup, a full build, the capped suite and the restore in a `finally`; a wait loop of mine then matched
  its own command line with `pgrep -f` and had to be stopped. Effects, each checked: all six runs RED by
  assertion (MI1b 5 cases — this round's sweeps among them — MP6 1, X23r 1), so they stand as fresh verdicts on
  this round's tree; the three files byte-equal their backups after (`cmp`) and `robot.hpp` / `camera_plant.hpp`
  equal the verifier's copy (untouched by this round); the 2026-10-29 `review-fixes2/supp_*.txt` outputs were
  overwritten by these runs and `review-fixes2/mutations.log` has six extra lines. The binary left on disk was
  X23r's mutant link: removed and relinked from source, the suite green (2571 / 2571) before anything else ran.

### RESUMED (second stop: the first resumption above was stopped after its last mutation, 10:14, before it appended its own section) — `date` Fri Oct 30 02:23:53 PM EDT 2026
Executor: opus (the builder again, the same workflow run continued). The section above was the first resumption's DRAFT
(`scratch/vis1-a1/vr2/log_draft.md`, last written 09:59), appended here VERBATIM by this resumption; every claim in it was
re-checked below, and each "see below" it promises is answered below.

- **The tree as left.** `git status`: 53 entries (34 modified, 19 untracked), as at the round's start; `git diff
  --stat ceb17c0 -- src/ tools/ Makefile` EMPTY (no gate plant). What the stopped resumption had done, found on
  disk: its draft (appended above); every row of its mutation table run (`vr2/mutate_vr2.py`, 64 rows, 09:07 –
  10:14, `vr2/mutations.log`); its final-tree probes (`vr2/probes/final/`, 10:14 – 10:22). It stopped after
  those probes — before its gates and before appending its section (no worktree file is newer than 10:14 but
  `camera_geometry.hpp`, MZ6r's restore at 10:14:12).
- **No mutant applied.** Its 56 backups (`vr2/bak/`): 42 byte-equal to their live files (`cmp`); the other 14 —
  the `camera_geometry.hpp` backups taken before A1-V1 (BI2, FA1–FA7, PS1–PS3, SZ2, SZ6, SZ9a) — byte-equal to
  the pre-A1-V1 snapshot `vr2/camera_geometry.before-2d-jitter.hpp`, so they differ from the live file only by
  that intended edit. The live tree against the verifier's copy (`verify/tree/`, `diff -rq`): differences only
  in the files this round edited — `hal/pros/ai_vision.hpp`, `hal/vision_frames.hpp`, `vision/camera_geometry.hpp`,
  `config/camera_config.hpp` (`pitchToleranceDeg`'s doc names the 55-degree limit), the two test files, the
  changelog, guide 10, the glossary, the register, `docs/api/`, the briefing and this log — each diff read; every
  other file (`robot.hpp`, `camera_plant.hpp`, `mkdocs.yml` among them, touched only by restores) equal.
- **The binary on disk WAS a mutant link** — 10:13, MZ6r's build (its restore at 10:14). No suite was run on it:
  the gates below removed it first and relinked from source.

#### Each claim of the draft re-verified against the tree (not taken from it)
- **The code read whole for (1)–(3) and (6):** `ProsAiVision::read()`, `verify()`, `colourReadFailed()`,
  `configure()` — the check compares id, red, green and blue; Unanswered only on the mode read's `PROS_ERR` or
  the exact record. **The record re-read in the disassembly myself** (`scratch/vis1-a1/objdump/ai_vision.dis`
  :236-:276): `cbz r0, 60` on a failed claim; at :60 `movs r6, #127`, `mov.w r7, #0x7f800000`, both copied to
  id/red/green/blue and to both floats — exactly the code's `kFailed` and +Inf with the sign checked; the success
  path loads the stack record the SDK wrote (`mov r2, sp`; `ldrb.w r6, [sp]`). **No fast-math anywhere**
  (`grep` of `common.mk`, `Makefile`, `test/CMakeLists.txt` for `fast-math|finite-math|Ofast`: nothing), so
  `std::isinf` cannot be folded away on either build. The shim's colour slots (`std::array<…, 8>`, slots 1..7,
  an unset slot all zero). The four new adapter test cases read whole (each field on its own — five cases; the
  exact record — a wiped camera, an ignored write into an empty slot, the record and its eight one-field-off
  variants at the 1 Hz check, the set-up's read-back of the record and of green 126; descriptor 7 — configured,
  slot 7 written, three reads at 0, 1 and 2 s, six colour reads).
- **The code read whole for (4), (5) and A1-V1:** `vision/camera_geometry.hpp` end to end — `bearing()` (the
  level pitch per corner and per jittered row), `elevation()` / `groundRange()` (row AND column, the principal
  point's column via `jitteredColumns()`), `sizeRange()` (corners × columns × both edges; the interior search per
  focal-length corner on the shrunk box at the far column; the convexity tangent), `fieldFix()` (corners at the
  heading ends, the opposite turn, `farArc()` at the ends and at the two lens-on-the-line turns),
  `cameraModelProblem()` (|pitch| + tolerance > 55). The draft's monotonicity arguments re-derived for the
  bearing (f = cos p + yd sin p = sqrt(1 + yd²) cos(p − atan yd): largest at p = atan(yd), linear in yd for a
  fixed p; xr and (p, yd) independent, so the extremes are the corners plus that pitch), and `kMaxPitchSpanDeg`'s
  56.7 degrees (the jittered edge row at fy −5 %: atan(122 / 186.0) = 33.3 degrees; 90 − 33.3 = 56.7). The tests
  read: the field fix's far-side subcase (75 + 0.25 for 180, 181, 270 and 1000 degrees; 179 degrees by hand to
  1e-12; monotone in the tolerance), the 450-fix sweep, the level-point test, the whole-box hull sweep, the
  straddle tests and `cameraModelProblem`'s edge.
- **Every verifier probe re-run on THIS tree,** compiled fresh against the worktree's headers (the adapter ones
  with `-DSHULIB_HOST_PROS_SHIM -I test/pros_shim`), each under `ulimit -v 4194304; timeout 600`
  (`vr2/probes/resume/verifier_probes.out`): `wiped_slots` — status Ok, usable, faultedReads 0, settingsLost 1,
  setups 2 (the verifier's tree: ReadFault, faultedReads 1, settingsLost 0); `ignored_write` — "a colour read back
  different from what was written" (was "did not answer"); `fieldfix_testscene` — 180 and 270 degrees: radius
  75.25, the box's farthest point 75.0000; 179: 74.9991 = the farthest point; `fieldfix_brute` — 0 under-bound at
  every one of 12 heading tolerances, 10 to 359 degrees (400 fixes each; the verifier's tree: 53 / 90 / 81 at 170 /
  180 / 359); `bearing_horizon` — 0 of 78 miss at 1 and 3 degrees (was 38 of 78); `size_interior` — 0 miss in
  each of its nine blocks (2,619 checked; was 291 of 291 at pitch 0 ± 1).
- **A slip in the stopped run's probe summary, corrected:** its `vr2/probes/final/small.out` reads "size_interior:
  misses total 2619" — that is the sum of the CHECKED column (9 × 291); its own final binary, re-run here, prints 0
  miss in every block (column 9 sums to 0). Nothing was decided on the wrong line (the draft never quoted it).
- **The stopped run's brute-force probe re-run on this tree** (`vr2/probes/geom_brute.cpp`, compiled fresh;
  `vr2/probes/resume/geom_resume2_{doc,full}.txt`): the true camera on a grid INSIDE the box with every tolerance at 100 %
  (pitch −44..44 at tolerances 1, 3 and 5; yaw 0 and 20), the truth from the probe's own projection; **doc
  mode** (each function's documented jitter): bearing 5,216,244 · elevation 5,216,244 · ground range 2,964,572 ·
  size range 74,062,848 checked, **0 miss** in all 120 blocks; **full mode** (every edge jittered, the observed
  centre moved): bearing 15,648,732 · elevation 15,648,732 · ground range 8,893,728 · size range 74,062,848 checked,
  **0 miss** in all 120 blocks. (The verifier's tree, the stopped run's `geom_before_doc.txt`: 944 bearing misses
  at pitch 0 ± 1 and 13,936 size-range misses at pitch 0 ± 5, among others.)

#### The mutations the draft points to "below" — 64 rows, `vr2/mutate_vr2.py`, `vr2/mutations.log`
Each: the file backed up, ONE exact replacement keeping the line count, the suite binary deleted, a full `cmake
--build` (every object that includes the file recompiled), the WHOLE suite under `ulimit -v 4194304; timeout 600`,
the file restored in a `finally` and touched. 0 "BUILD FAILED", 0 "NOT RUN", 0 "RESTORE MISMATCH", 0 killed; every
RED below is a test's runtime assertion (cases failed in brackets):
- **Finding 1:** VR 1 · VG 1 · VB 1 · VI 1 — each one term of `verify()`'s compare dropped; each fails "the 1 Hz
  check compares EACH field on its own".
- **Finding 2:** CRW 1 (the old wide rule back) · CRID 1 · CRR 1 · CRG 1 · CRB 1 · CRHI 1 · CRHS 1 · CRSI 1 · CRSS
  1 (each term of the exact record dropped) — each fails "ONLY get_color's EXACT failure record is a failed read".
- **Finding 3 — the equality edge:** the verifier's CRF7 has no anchor any more (the adapter no longer has an id
  range); its defect re-expressed as **CRF7b** (id 7 taken for the failure record) RED 1, and **VF7** (the
  settings' own check `c.id > kMaxColourDescriptors` → `>=`, the edge landed on exactly by a block holding id 7)
  RED 2.
- **Finding 4:** FA1 4 · FA2 2 · FA3 2 · FA4 2 · FA5 2 · FA7 1; **FA6 SURVIVED** (below). They ran on the
  pre-A1-V1 header; A1-V1 touched neither `fieldFix()` nor the tests they failed (`diff` of the pre-A1-V1
  snapshots: every hunk of the header before its line 302 or at the helper, 542; of the test file at :486-502,
  :831-854, :876-975 and :1009 — none in the field-fix tests or `cameraModelProblem`'s), so they stand.
- **Finding 5 (before A1-V1):** PS1 1 (exactly 55 refused — the edge, landed on by 44 + 11) · PS2 1 · PS3 1 — they
  stand for the same reason; BI1, BI2, SZ1, SZ2, SZ4, SZ5, SZ7, SZ8, SZ9b, SZ10 all RED and then RE-RUN on the
  final code (below); **SZ6 and SZ9a SURVIVED** (below).
- **After A1-V1, on the final code:** BI1 2 · BI2r 2 · BJ1 2 · BJ2 2 · X22r 4 · MZ7r 3 (the bearing) · X20r 3 · EJ1 2
  · EJ2 1 (the elevation) · GJ1 2 · GJ2 1 · GJ3 4 · MG8br 1 (the ground range) · JC 1 (the shared column helper) ·
  SZ1 3 · SZ2r 1 · SZ4 3 · SZ5 3 · SZ7 3 · SZ8 2 · SZ9b 3 · SZ10 3 · MZ1r 6 · MZ2r 5 · MZ4r 3 · MZ5r 3 · MZ6r 3 (the
  size range).
- **The three survivors, shown EQUIVALENT here, not assumed** (`vr2/probes/resume/equiv.cpp`; each mutant applied
  to a PRIVATE copy of the live header under `vr2/probes/resume/inc_<id>/`, never the worktree; the same probe
  built against the real header and against each copy; 583,020 size ranges — nine pitches −44..44 × five
  tolerances 0.5..11 inside the span, columns 0..320, centres 4..236, heights 6..230 — and 20,000 field fixes,
  half of them with the heading tolerance set to EXACTLY the distance of a lens-on-the-line turn; every output
  printed `%.17g` and compared; `vr2/probes/resume/equiv.out`):
  - **SZ9a** (the interior search at fy's HIGH end only): 0 of 583,020 differ, bit for bit. The product falls
    with fy (the rise between two rows falls with fy wherever fy² − y² − 2 fy y tan(p − atan(y/fy)) > 0 — every
    row of the picture inside the span), so the fy-low search never sets the far end. KEPT: it costs two
    searches and keeps the far end safe if an optics' monotone direction ever differed.
  - **SZ6** (the convexity tangent's term dropped): 0 differ. After 40 bisections b − a ≤ 0.38 rad / 2⁴⁰ ≈ 3.5e-13
    and the slope at a is of that order too, so exp(slope (b − a)) rounds to exactly 1.0: the term makes the bound
    provable, below one ulp in value.
  - **FA6** (`<= dTh` → `< dTh` for the lens-on-the-line turns): 8 of 20,000 differ, by at most one ulp (2.4e-16
    relative): a turn exactly at the interval's edge is the same turn `farArc(th ± dTh)` already evaluates.
  - **Controls — the domain bites:** SZ1 (the interior search removed) changes 58,200 of the size ranges (the far
    end up to 4.6 %), FA3 (the lens-on-the-line turns removed) 677 of the field fixes — so the zero differences
    above are not a domain that never reaches the code.
- **Earlier verdicts this round, by the stopped run's mistake (logged in its draft):** MI1b 5 · MP6 1 · X23r 1, each
  run twice (`review-fixes2/mutations.log` :319-:324), all RED by assertion — fresh verdicts on this round's tree.

#### FOUND IN THIS RESUMPTION — checked, NOT a defect: the size range's sideways factor at the OBSERVED centre row
- **The question:** `sizeRange()` jitters the two edges independently, so a box whose edges both moved the same way
  has its true centre up to kBoxEdgePx from the observed one — but the line of sight's sideways factor is taken at
  the observed centre row for every edge combination. The header (A1-V1) says each function carries both
  coordinates of a box edge's jitter.
- **The answer:** g(dt, db) = ln sideways(centre + (dt + db)/2) + ln(rise(top + dt) − rise(bottom + db)). Where
  the rise term outweighs the sideways term in each edge's derivative, d g / d top < 0 and d g / d bottom > 0
  over the whole edge square, and BOTH ends of the interval come from dt = −db — boxes whose centre does not move.
  **Measured** (`vr2/probes/resume/centre_dominance.cpp`, `centre_dominance_fine.cpp`): 233,406,360 points
  (every pitch −55..55 by 1 degree, both focal lengths at both ends, columns 0..320, every box in the picture,
  each edge jittered) and then 608,236,200 on a finer grid with the jittered edges 2 px outside the picture and
  the smallest measurable box, plus 291,001,200 around the worst corner: **0 with a sign not dominated; the
  smallest ratio 5.863** — at pitch 55, the picture's top-left corner 2 px outside, both focal lengths at −5 %.
  So the code is exact at the ends; the brute-force probe's full mode (which moves the observed centre) agrees.
- **Documented, not changed:** `sizeRange()`'s doc now says so ("The sideways factor is taken at the box's
  OBSERVED centre row … at least 5.8 times …"), and the header's sentence names how the size range carries the
  row ("through its two edges and its column"). `docs/api/` regenerated (`camera_geometry.md`); `check-fresh` 0.

#### Edits by this resumption (all documentation; no code line changed)
- `include/shulib/vision/camera_geometry.hpp`: the header's tolerance paragraph re-flowed (the stopped run left
  two lines of 133 and 148 characters), its one added clause above; `sizeRange()`'s doc, the sentence above.
- `docs/api/camera_geometry.md` regenerated.
- `docs/changelog.md` — this chunk's own entry, its **TESTED** paragraph only: each colour field compared on its
  own, only the binary's exact failure record taken for a camera that did not answer (a wiped slot is a changed
  setting), descriptor 7 written and held; every interval swept against the WHOLE tolerance box at 100 % of each
  tolerance, and the fix radius against a sweep of bearing, range and heading — the round's tests were not in it.
  Re-flowed to 110 columns, words otherwise unchanged.
- `test/README.md`: the shim paragraph's "It / starts / ADVERSARIAL" (left ragged by the held edit) joined.
- This log (the draft above appended verbatim, and this section).

#### The prose review (pace rule 4) — RUN by this resumption, each grep's hits READ before this was written
For the source files this round changed — `hal/pros/ai_vision.hpp`, `hal/vision_frames.hpp`,
`vision/camera_geometry.hpp`, `config/camera_config.hpp` — over `docs/` (less the generated `docs/api/` and this
log), `README.md` and `test/README.md`:
- `grep -rn -i 'ReadFault\|faultedReads\|faulted read\|settingsLost\|did not answer\|failure record\|1 Hz
  check\|once a second\|once-a-second\|read-back\|read back'` (less mechanism/motor/IMU/rotation/GPS lines): the
  camera's hits read — guide 10 :568-:575 (a re-write after a read the camera does not answer, the once-a-second
  check: TRUE), the changelog :50-:68 ("a check the camera does not answer is a failed read … never a 'lost
  setting'": TRUE for the exact record and a mode read's PROS_ERR) and :132-:146 (TESTED: stale by omission —
  fixed above), the register's Step 0 line :161, HA-196's row :383 and entry :1236-:1262 (the exact record only;
  an empty slot compared — TRUE), HA-203's row :390 and entry :1363-:1373, test/README :86-:98 (the fake's
  `ReadFault`; the shim's 127s and +Inf, `colourReadFails` — TRUE; the ragged break fixed). The other hits are
  other devices' `faultedReads()` (faq :22, guide 10 :533, MECH-PANEL, R1a, DOCS2, …) — unaffected.
- **The call counts re-derived from the code:** `kMaxColourDescriptors` 7, `kMaxVisionClassNames` 20 — the check
  1 + 7 + 20 = **28**, a set-up 7 × 2 + 2 + 20 = **36**, `kRetrySeconds` 0.5: HA-203's row, its entry and
  `read()`'s doc give 28 and 36; its entry and `IVisionFrames`' class doc the half second (the class doc names the
  check and the re-write without figures). TRUE. (Each figure is a per-read "up to"; a read whose check finds a
  change writes the set-up in the same read — then at most 8 + 36 + 25 = 69 calls; a read whose check holds, at
most 28 + 25 = 53 — no document sums them, and none claims a total.)
- `grep -rn -i 'size range\|sizeRange\|fix radius\|fieldFix\|kMaxPitchSpan\|55 degrees\|cameraModelProblem\|tolerance
  box\|every corner\|monotone\|camera_geometry\|CameraModel\|boundAtImage\|ground range\|groundRange'`: the hits read
  — guide 10 :600-:616 (the exact range over every combination; 55 degrees refused by the geometry, the camera
  still read: TRUE), the changelog :72-:86 (TRUE), the glossary's fix radius :96, ground range :109, size range
  :230, tolerance box :268 (TRUE), the roadmap's F15 row :149 (names `camera_geometry.hpp`, NOT FROZEN: TRUE), the
  register's HA-192 / HA-194 / HA-200 / HA-202 Source fields :1148-:1353 (name the file and its functions, no
  claim this round changed); the rest ("monotone", "blast radius … fix") other chunks' words.
- `grep -rn -i 'pitchTolerance\|pitch tolerance\|cameraPlacementProblem\|CameraPlacement'` and `'empty
  slot\|wiped\|id outside\|1\.\.7'`: changelog :70, the roadmap :149, HA-196 :1246 — TRUE.

#### The gates on the final tree (`date` 14:28 – 14:53; `vr2/gates_r2.sh`, outputs `vr2/out_r2/`)
- **The suite**, from a binary REMOVED and relinked by a full `cmake --build` (0 warnings) after `docs/api/` and
  the briefing were regenerated with the binary absent, under `ulimit -v 4194304; timeout 600`: **2571 / 2571 test
  cases passed (3 skipped, as before), 2,802,533 assertions, 0 failed** — 33 s, 62.5 MB peak. THEN
  `briefing_status.py generate`. (2571 = the fix round's 2563 + the eight cases this round added: three adapter —
  each field, the exact record, descriptor 7 — and five geometry — the fix radius's sweep, the whole-box sweep, the
  level point, the two straddles; `grep '^TEST_CASE'` against the verifier's copy.)
- **Doc gates:** `api_doc_tool.py` self-test OK · check-coverage (4,953 public entities across 188 headers, all
  documented) · check-fresh rc 0 · check-examples (541 quoted lines verbatim) · check-removability (no public doc
  references docs/internal/) · `briefing_status.py` self-test OK, check current · `doc_staleness_audit.py`
  self-test OK, audit clean.
- **The CI greps:** the library PROS-free outside `hal/pros/`; the core sim-free. **The register**
  (`vr2/register_check.py`): direction 1 prints nothing; direction 2 STRICT — each of HA-192..HA-203 cited in
  every Source file other than as a range end — **bad: 0**; no unnumbered `PROVISIONAL (A4`.
- **The ARM header gate:** all 230 headers in one TU, `arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion
  -Wsign-conversion -Wshadow -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp` — rc 0.
- **Compile refusals:** `ctest -R compile_refusal` — 200 / 200 passed; 200 TUs under `test/compile_refusals/`.
- **`src_build_gate.py self-test`:** OK (235 detector cases, each against a real build). **`check`:** PASS — all 7
  builds link, 0 disallowed warnings. Afterwards `git diff --stat ceb17c0 -- src/ tools/ Makefile` is EMPTY.
- After this section was appended, the doc gates that read `docs/` re-run: see the last line below.

#### The tree at the end of this resumption — `date` Fri Oct 30 02:54:39 PM EDT 2026
- **The six findings: all REAL, all FIXED at the root** (the dispositions above, each re-verified here): (1) the
  check's compare pinned field by field; (2) only the binary's exact failure record is a failed read — a wiped
  camera is a lost setting, HA-196 unchanged in belief; (3) descriptor 7 configured and held, its edge landed on
  (VF7) and its defect re-expressed (CRF7b); (4) the fix radius EXACT over bearing, range and heading together, the
  test's expected value corrected; (5) every interval EXACT over the whole tolerance box (the interior points
  evaluated, the claim corrected, the 55-degree domain enforced by `cameraModelProblem()`); (6) `ReadFault`'s and
  `read()`'s docs corrected and the prose review run and logged (above). Found beside them: A1-V1 (fixed), and in
  this resumption the size range's centre row (checked, not a defect, documented).
- **Nothing committed** (branch `vis1-a1`, base `ceb17c0`): `git status` 53 entries (34 modified, 19 untracked) — this chunk's files only, the
  regenerated `docs/api/` and `mkdocs.yml`, and `PROJECT-BRIEFING.md`'s generated block. `src/`, `tools/` and the
  `Makefile`: no diff.
- **No mutant applied:** of the 56 backups under `vr2/bak/`, 15 (`ai_vision.hpp` 14, `vision_frames.hpp` 1) are
  byte-equal to their files; the 27 `camera_geometry.hpp` backups taken after A1-V1 differ from the live file in
  COMMENT lines only (`diff`: 0 changed lines that are not `//` comments — this resumption's re-flow and doc
  sentence); the 14 taken before A1-V1 equal the pre-A1-V1 snapshot. This resumption's own mutants were applied
  only to private copies under `vr2/probes/resume/inc_*/`, never on the worktree build's include path. The suite
  binary on disk is the 14:28:13 fresh link of this tree; nothing but this log changed after it.
- **Changed by this resumption:** `vision/camera_geometry.hpp` (comments only: the header paragraph re-flowed with
  one clause, `sizeRange()`'s centre-row sentence), `docs/api/camera_geometry.md` (regenerated), `docs/changelog.md`
  (this chunk's TESTED paragraph), `test/README.md` (one ragged break joined), this log. Not touched, by design: the
  shared plan docs (A1-D13's status lines and the (D8) items are the coordinator's at the merge), `src/` (D7),
  VIS1-A2 and VIS1-B.
- **Still FOR THE COORDINATOR:** this round's two items (the geometry's new 55-degree refusal in the camera's
  contract, and `eachCorner()` visiting corners only, for A2's brief — both stated in the draft above), and every
  earlier one as logged (D1 decision #16's amendment, D2 the registry-kind belief, D7 the programs' stale boot
  words, D8 the merge-time items re-checked at `e5a69b5`, finding (1)'s top-face note for A2). No hardware belief
  outside HA-192..HA-203 was added by this round.
- **After the append** (`date` Fri Oct 30 02:55:36 PM EDT 2026): `briefing_status.py check` current · `doc_staleness_audit.py` clean ·
  `check-removability` clean · `check-fresh` rc 0 · `check-examples` 541 verbatim; `git diff --stat ceb17c0 -- src/
  tools/ Makefile` EMPTY; `git status` 53 entries. The verification round is complete; nothing committed.

## The coordinator's merge verification — VIS1-A1 (2026-10-30 15:00 – 2026-11-03 13:50, main `eab0f89` + the squash of `vis1-a1`)
- **Audit before the merge:** `audit_wt.py vis1-a1` — 11 backed-up files, 0 suspicious; the two files newer than their
  backups (`config/camera_config.hpp`, `vision/camera_geometry.hpp`) diffed by hand: the last fix round's doc and wording
  edits, no live mutant. No gate plant; no stray root file. The transport commit `4bac494`, then `git merge --squash
  vis1-a1` (base `ceb17c0` — older than every chunk landed since).
- **Seven conflicts:** `docs/api/README.md`, `all-entities.md` and the briefing (regenerated); the changelog — the entry
  was written under API 2.4, so it moved to the top of `## API 2.5` as "Added since 2.5 (2026-10-30; built 2026-10-18–22,
  reviewed and fixed 2026-10-22–30)" (its "no version bump, every change ADDITIVE" still true); the register — the status
  **7 of 194** (main's 182 + HA-192..HA-203), the index rows in number order (HA-192..203 before HA-213..215; 194 rows,
  sorted), the "added" paragraphs in landing order with VIS1's last ("With them, 194"); guide 10 — both new sections kept
  (A4's "In an auton, on the buttons, on a route" and VIS1's "The camera — a frame at a time"); HANDOFF :~263 — the
  adapters sentence names both CAL1-A2a's shell and `ProsAiVision`. VIS1 had left build-order and REMAINING-WORK alone
  (A1-D13): the coordinator added its LANDED status there and a roadmap "Now" paragraph.
- **A SEMANTIC CONFLICT, caught by the build:** the squash had no text conflict in code, but the merged tree did not
  compile. VIS1-A1 added the pure virtual `hal::IDeviceProvider::aiVision()`; CAL1 Part A2a — landed AFTER VIS1's base —
  brought `test/calibrate_program_rig.hpp`'s `CountingProvider`, which implemented the interface without it ("cannot
  declare field … of abstract type"). Fixed in the merge, at the root: one forwarding override (`++requests;` then the
  inner provider's `aiVision`, the wrapper's pattern for every device; Calibrate never asks for a camera, so no count a
  test pins moves). No other `IDeviceProvider` implementer in any in-flight worktree lacks it (searched).
- **Gates on the merged tree** (`.claude/coordinator/logs/vis1_a1_merge_verify.out`, re-run 2026-11-03 after the pause):
  host build with the doc gates; suite **2,815 | 2,815 passed, 3 skipped**; `src_build_gate.py` self-test OK and check
  **PASS, all 7 builds**; ARM header gate over **235** headers clean; CI greps empty; api check-fresh, the briefing,
  removability and the staleness audit clean; the compile-refusal harness **255 of 255**; no plant, no stray root file.
- **The coordinator's hand mutations** (`muts_vis1_a1_main.json`; each on a fresh link, the whole suite under a 4 GiB /
  300 s cap, restored and hash-checked): **6 of 6 caught** — MV1 (a PROS_ERR count read as a number of detections), MV2 (a
  failed read keeps the settings as written — HA-196), MV3 (exactly 24 detections not "possibly full"), MV4 (objects
  counted before the model's names match), MV5 (a colour blob from a descriptor another program wrote is kept), MV6 (a
  provider that throws a `std::runtime_error` making the camera — the robot still builds, but the reason loses the
  provider's own words) RED. Restored tree green.
- **Next for VIS1:** VIS1-A2 (the Toggle tracker, the piece finder, the feed) — moved ahead by the team lead's ruling of
  2026-11-03 (the camera features, drive-to-piece and the auto-flipper before 2026-11-21, the panel after); then VIS1-B.

---

## VIS1-A2 — what the camera means (the Toggle tracker, the piece finder, the camera feed)

- **Started:** Tue Nov  3 01:53:07 PM EST 2026 (`date`).
- **Tree at start:** `9d77c9d` (VIS1-A1 merged, main), clean, in the git worktree `.claude/worktrees/vis1-a2` (branch
  `vis1-a2`) — NOT the main working tree. Executor: opus, under the coordinator's workflow run.
- **Go:** the team lead's answers of 2026-10-18 ("Accept all (Recommended)" — D1–D11 as recommended, D6 and D9 as the
  re-check changed them) and his ruling of 2026-11-03 that the camera features, drive-to-piece and the Toggle
  auto-flipper come BEFORE 2026-11-21, ahead of the panel ("Everything incl. flipper"). VIS1-A2 is on the
  auto-flipper's critical path (AF1 needs the Toggle tracker; L4 needs the piece finder). Do not commit; no `pros`
  command; no `/dev/ttyACM*`. Probes and mutation backups under `.claude/coordinator/scratch/vis1-a2/` only. Every
  test binary runs as `(ulimit -v 4194304; timeout 600 …)`.
- **Scope:** §2.1's VIS1-A2 row — §1.12 `vision/toggle_tracker.hpp`, §1.13 `vision/piece_finder.hpp`, §1.14
  `vision/camera_feed.hpp`; §2.2's VIS1-A2 tests and mutations; §3.1's VIS1-A2 documents (changelog, a NEW guide
  chapter "The camera" + its two hand-kept index entries, guide 14, HA-197/199/200/203's Source lines, `docs/api/`).
  NOTHING of VIS1-B (`src/`). The alliance colour is ALLY1's: the tracker reports colours, never "ours".
- **Reading (the team lead's 2026-10-22 ruling for executors):** RESUMING whole; the brief's header and answers
  block, every Re-check section (T1–T10, S1–S10, R7), D1–D11, the plain words, §1 preamble, §1.12–§1.14, §2.1's
  A2 row, §2.2's A2 tests and mutations, §2.3, §3 (A2), §4, §5, §6, §7, §8 in full; the rest by heading. This log's
  VIS1-A1 entries and the coordinator's merge record — the hand-ons to A2 collected below as they are met.


### RESUMED — Tue Nov  3 03:50:28 PM EST 2026 (`date`)
- **The stop.** The team lead dropped to ONE job at 93 % of the session limit at ~14:50 and the old session ended; the
  coordinator audited this worktree (no live mutant) and resumed the run after the reset. The stopped run logged only its
  start and reading line (above); its work is on disk, unlogged, and is accounted here from the files themselves.
- **Found on disk** (`git status`, file times 13:52–14:07): NEW `include/shulib/vision/{toggle_tracker,piece_finder,
  camera_feed}.hpp`, NEW `test/camera_scene_rig.hpp`; CHANGED `include/shulib/vision/camera_geometry.hpp` (two
  per-camera functions, `groundRangeAt()` / `sizeRangeAt()`, and a header paragraph). No test file, no
  `test/CMakeLists.txt` edit, no document. `git diff -- src/ Makefile`: EMPTY (no gate self-test plant). Scratch
  `.claude/coordinator/scratch/vis1-a2/` holds `configure0.txt`, `build0.txt` (a full build at 13:58, before the three
  headers were last written) and `hdr.cpp` (a compile probe of the three headers): no mutation backup, so no mutation
  was ever run by the stopped run. The suite binary on disk is removed and rebuilt from source before any result.
- **Reading re-done by this resumption** (the 2026-10-22 ruling's list): RESUMING whole; the brief's header and answers
  block, the Re-check (T1–T10, S1–S10, R7), D1–D11, the plain words, §1 preamble, §1.12–§1.14, §2.1–§2.3, §3, §4, §5,
  §6, §7, §8, §9; this log's VIS1-A1 entries and the coordinator's merge record. **The hand-ons to A2** found there:
  (H1) finding (1)'s note — a Pin seen from above has its size range up to ~1½ radii NEARER than its near rim (the box
  takes in the top face), so §1.13 rule 1's ON-THE-FLOOR agreement must allow it; (H2) `eachCorner()` visits the
  corners only — A2 decides whether a "one camera explains it" test needs the interior cameras (refusal is the safe
  side); (H3) A1-D3 — the Toggle's flat bottom face is 12.36 in, ~¼ in below the field face's lower boundary
  (`ToggleFace::flatBottomIn`), the host camera model's bottom face is at the boundary (a stated approximation); (H4)
  A1's review (10) — public docs must not name `vision::CameraFeed` before it exists (it exists now: A2 may name it);
  (H5) A1-D14 — guide 14's tag-corrector sentence already clarified by A1; (H6) the coordinator's merge record — the
  changelog's VIS1 entry sits under `## API 2.5`, the register at 194 rows.
- **Fresh build at the resumption** (`date` 15:52): `rm -f build/test/shulib_tests`, the briefing regenerated, configure,
  build — the doc gate failed on a STALE `docs/api/` (the stopped run never regenerated it: three new pages and
  `camera_geometry.md`'s two new members); `api_doc_tool.py generate` (coverage already 5,424 entities, all documented),
  rebuilt: only `camera_geometry_test.cpp.o` recompiled (the one object whose depfile names the changed header — checked
  in `build/test/CMakeFiles/shulib_tests.dir/*.d`); suite **green, 2,815 cases** (`suite_run.py green` under the cap) —
  the stopped run's geometry refactor breaks none of A1's tests.

### The stopped run's code, reviewed against the v6 prototypes (its deltas, logged here because it logged none)
- **A2-D1 (`toggle_tracker.hpp`):** the prototype's gates 2–4 split out as `detail::measureToggleFace()` so §1.12's bench
  helper `explainAsToggleFace()` (absent from the prototype) shares the tracker's own arithmetic (one path, never two);
  `ToggleFaceMeasure {gate, aspect, sideClipped, range, impliedLength}`; `toggleColourWord()` / `toggleWhyWords()` (ASCII,
  for a log line); `toggleTrackerConfigProblem()` also refuses an id past 7 and a non-finite number; gate 6 also refuses
  an INVALID bearing interval (the prototype read an invalid interval's zeros as "inside the window"); the read's count
  clamped to the array; `view()`'s alive test NaN-safe (`!(now - last <= alive)`); a frozen view keeps the bearing and
  range but clears `positionChecked`/`toggleIndex`; the same-colour-twice selection written without the prototype's
  `front = c` before the nearer test (the same answer: two colours are UNKNOWN anyway).
- **A2-D2 (`toggle_tracker.hpp`, hand-on H3):** the bottom-face gate also tries the FLAT bottom's plane
  (`flatBottomIn`, 12.36) beside the lower edge's interval — more explanations can only refuse more.
- **A2-D3 (`camera_geometry.hpp`, hand-ons H1/H2):** `groundRangeAt()` / `sizeRangeAt()` — the per-camera parts of
  `groundRange()` / `sizeRange()`, which now call them (the whole-box results unchanged: A1's tests green); a header
  paragraph: "some corner explains it" is an honest yes, "no camera explains it" is shown only from the whole-box
  intervals.
- **A2-D4 (`piece_finder.hpp`, hand-ons H1/H2):** the posture rule rewritten from the prototype's mixed per-corner
  arithmetic onto A1's exact geometry: ON THE FLOOR when SOME corner's `groundRangeAt` and `sizeRangeAt` meet, the size
  range allowed to read NEARER by the top face's depth (`topFaceAllowanceIn()`, 2r(a − H)/H, the supremum over every
  distance — derived by me, below); RAISED only when the WHOLE box's farthest size range plus that allowance is still
  nearer than its nearest ground range (corners + the principal point's column, any descending ray) — never concluded
  from the corners alone; a `kPostureSlackIn` of 0.25 in. Also: `pieceTargetProblem()` refuses a class bit past the
  model's five and a non-finite position or tolerance; the bottom-cut bound `nearerThanIn` taken at row 239 − kBoxEdgePx
  and both jittered columns (the prototype: row 242, one column — a bound that could be short); `nearestOfDisc()` and
  `bearingCouldBeInside()` also take off the mount's position tolerance and the lens's swing over the heading tolerance;
  the NOT-IN-VIEW slack gains the heading tolerance at the target's distance; the finder's words
  (`postureWord`, `pieceVerdictWords`, `pieceWhyWords`).
  *The allowance, derived (checked before relying on it):* a box spanning the floor's near rim (range R, height 0) and
  the top's FAR rim (R + 2r, height H) under a lens a > H high has a rise difference a/R − (a − H)/(R + 2r), so its
  line-equivalent range is R_eq = H R (R + 2r) / (H R + 2 r a) and R − R_eq = R · 2r(a − H) / (H R + 2 r a): increasing
  in R, with limit 2r(a − H)/H — so the allowance is a supremum, exact in the pitch (the rise is per inch along the
  camera's own vertical plane). With a ≤ H the box's top is the near rim and R_eq = R.
- **Found by this resumption's review, to be confirmed by tests before any fix:** (a) after an UNUSABLE read the
  finder's `answer()` runs `decide()` over an emptied sighting list — NONE from a fault (the prototype's defect, copied);
  (b) NOT IN VIEW tests the disc's centre and four axis points only — a disc near a corner of the view can be partly
  out of view with all five in.
- **A2-D2 REVERTED by this resumption (measured, `date` ~16:30).** The ported R5 sweep printed a D9 table that no longer
  matched the brief's: a 12-in level lens NEVER reported (brief: 42..22 in), the 12-in lens pitched up 10 degrees only
  74..40 in (brief: 72..12 — D9's "pitch it up ~10 degrees" rescue), 11.75 in 60..24 (brief 60..12). A scratch probe
  (`scratch/vis1-a2/probe_r5/r5why.cpp`, the live header against a copy without the flat plane) showed the reason
  letter by letter: "maybe the bottom face" at every near distance, from the 12.36 plane. Then the question that
  decides it — does the plane prevent a wrong colour? `probe_r5/realbottom.cpp` draws the Toggle's REAL lower shape (the
  colour boundary at 12.61, apex 1.02 behind; the bottom colour from that boundary round the corner to the flat bottom's
  far end at 12.36, 2.05 in back), seen square 80..4 in and turned -30..+30 at 14..48 in, lenses 6..12.25 in, both
  bottom colours: **0 wrong colours WITH the plane and 0 WITHOUT**; RED coverage without it equal or higher at every
  mount (12 in level 10/39 vs 0/39; 12 in up 10: 31/39 vs 17/39; turned 28/65 vs 12/65). So the plane cost range and
  bought nothing: removed, the reason in the gate's comment. The real-shape Toggle becomes a scene test (below).

### The tests so far, and what they found (`date` ~17:40)
- **`test/toggle_tracker_scenes_test.cpp`** (40 cases): P4 S1–S16 + S5b + S15b, P6 T1–T6, P7 T7–T10, P8/P8b as a
  second truth on the scenes that depend on the face (the believed 12.55/1.2 and the equilateral 12.61/1.02), the
  re-check's R2.S3/S3b/S4/S7/S7b/S8, R4, R5 (D9's table printed; pinned: a 10-in level lens far >= 60 and near <= 12,
  13.25-in far <= 44, the 12.25-in lens never, AND — new — the 12-in lens pitched up 10 near <= 12, the rescue D9
  advises, which A2-D2 had broken), R7, P4b, R6.4, and NEW "the real shape" (above). Numbers on this code, against the
  brief's v6 prototypes: R4 78 of 216 (brief 78); R7 0 wrong of 390, RED 106; P4b 70 of 540 (96-in window; brief 65)
  and 40 (36-in; brief 39), 0 with the position gate (brief 0) — A1's exact intervals are wider than the prototypes',
  so a few more lookalikes pass (HA-199's row and the tracker's header updated to 70); R2.S8 0 of 732; R2.S7b 90 of
  90 vetoed; T10 the plate RED at 2 distances (16–24 in), 0 at <= 14; S15b 3 outside refused, 18 inside RED.
- **Two of my ported expectations were WRONG, corrected (not the code):** S5b — the probe's "RESIDUAL RISK, stated"
  case only printed; the wall-top strip alone reads RED at 20 and 14 in on the prototypes too (v6 `p4_toggle_scenes.txt`)
  — now pinned as the stated limit; T7 — I first wrote "RED again at exactly index 4", but 4 x 0.033 = 0.132 < 0.15:
  index 5 is exact (the v6 output said "after 5 reads").
- **`test/toggle_tracker_test.cpp`** (13 cases, hand-made frames): every config refusal by its words with the first
  accepted value beside it; the words; `explainAsToggleFace` (the 4-px floor, the cut rows 1/2 and 239/238, the side
  clip spanning or not the centre column); every non-Ok status and a POSSIBLY FULL read; the hold and alive edges LANDED
  ON (equal passes, the next double does not); kind/id; the aspect floor landed on; the range window's OVERLAP and the
  bearing window's CONTAINMENT landed on (the window found by `nextafter`); two colours / same colour nearer; the
  colour-above reach landed on (2 x kBoxEdgePx vetoes, one pixel more does not, touching columns do not); the
  partial-band veto in and out of the window; the frozen move and turn thresholds landed on (the lens at the centre,
  so the pose's next double IS the lens's — with 7 in forward the last bit rounds away, which my first draft missed),
  the +-pi seam, recovery; the position gate. My own errors here, fixed in the test: the bottom cut row is 239 (the box's
  0..240 extent, `Detection::clipped`'s convention), not 238; the 30-in face's fix is ~9.2 in wide, so the gate's
  `maxFixRadiusIn` there is 12, not 8.
- **`test/piece_finder_scenes_test.cpp`** (20 cases): P5 F1–F9, P6 P1a/P1b/P2/P3/P3b, P7 for every non-Matches check,
  R6.1/R6.2 at the finder's level, and NEW properties swept through 22 true cameras at the EDGES of the tolerance box
  (0.95 of each), five mounts (6 level, 10 level, 10 down 10, 14 down 15, 18 down 20), 14 ranges x 9 bearings.
  **They found three defects in the finder, each fixed at its root:**
  - **A2-D5 — a fault read answered NONE.** (The review's (a), confirmed.) `answer()` ran `decide()` over the sighting
    list the fault had emptied. FIX: `lastUsable_`; a non-usable latest read answers UNKNOWN with the NEW
    `PieceWhy::ReadNotUsable` (the tracker's own reason, the brief's rule 5 "no usable read"). F8 pins it on a torn
    AND an unplugged read.
  - **A2-D6 — NOT IN VIEW looked at five points of the disc.** (The review's (b).) FIX: the eight corners of the octagon
    drawn AROUND the disc (circumradius slack / cos 22.5 degrees) — the picture's certain region on a plane is convex
    (four image-edge half-planes and "in front", per camera, intersected over the box), so those eight in view put the
    whole disc in view; the slack also gains the mount's position tolerance (the in-view test uses the nominal mount).
  - **A2-D7 — a Pin on a GOAL read ON THE FLOOR.** The property sweep: 73 of 2,720 Goal Pins (base 5.8 in) from a 10-in
    lens pitched down 10, 293 of 2,754 from 14 in, 639 of 2,772 from 18 in, each with a floor fix that MISSED its true
    centre (by up to twice its radius; from 18 in at 34 in out the radius was 6.0 and the miss 12.5 — a route with a
    13-in tolerance would have taken a Goal's Pin as a floor Pin at the wrong place). Cause (scratch `probe_pos/goal.cpp`):
    §1.13 rule 1 calls a Pin ON THE FLOOR when SOME corner agrees with the floor — and the corner opposite a true camera
    at the edge of its tolerance is TWICE the tolerance away (1.95 degrees, ~10 % in fy), through which a raised Pin far
    out does agree. Agreement says "the floor fits", not "nothing else does". The brief's own §2.2 test ("a Pin on a
    Goal is RAISED and never taken by an OnFloor route") fails on the brief's rule. FIX (measured first, scratch
    `probe_pos/posture2.cpp`, thresholds 2/3/4/5 in): ON THE FLOOR also needs NO camera of the box to be consistent with
    a raise of `raisedMinIn` or more — shown from the WHOLE box's exact intervals (a raise z0 puts the box bottom's floor
    point a/(a - z0) times the near rim, for a lens a high: no camera can when ground.hi + slack < size.lo x
    aMax / (aMax - raisedMinIn)); the raise that matters is a fact of the FIELD, so it is the TEAM's number,
    `PieceFinderConfig::raisedMinIn` (UNSET refused — the MECH core's no-library-default rule). With 5.0 (the scenes'
    Goal is 5.8): Goal Pins ON THE FLOOR 0 at every mount; floor Pins kept ON THE FLOOR 98 % (10 in down 10), 97 %
    (10 level), 100 % (6 level), 80 % (14 down 15), 11 % (18 down 20 — a high lens far out cannot rule a Goal out:
    UNSURE, the honest cost); every fix holds its true centre (0 misses of ~26,000). **FOR THE COORDINATOR: a new
    team number on the finder's config (L4 types it); the field fact it carries — "no piece rests between the floor
    and the lowest raise" — is the team's statement, not a library belief, so no HA number was taken; the coordinator
    may decide otherwise.**
  - **A2-D8 — the fixes' intervals made bounds (found with A2-D7, tightened where the evidence allows):** ON THE FLOOR,
    the centre is one radius beyond the near rim, bounded by the ground range AND (the floor now certain) the size
    range: [max(G.lo, Z.lo) + r, min(G.hi + r sec b, Z.hi + (A + r) sec b)] — off the camera's axis a depth along it
    is shorter than the horizontal range by cos b (b the bearing's farthest end plus the mount's yaw and tolerance, capped
    at 80 degrees), so the radius and the top face's allowance are divided by it; the stopped run's [G.lo, G.hi + r]
    could be short off the axis by r(sec b - 1); RAISED, the fix only when EVERY camera's box bottom looks down (the lens
    above the base, so the size range bounds the near rim), [Z.lo + r, Z.hi + (A + r) sec b]; LYING, [G.lo, G.hi +
    (L/2 + r) sec b] (the stopped run added the whole length L: the centre is at most half a length and a radius beyond
    the nearest end); the bottom-cut bound + r sec b.
- **The brief's F1 claim measured on the exact geometry:** the prototype's FOUND at 21 in (r 4.1) came from A1's
  pre-verification intervals; on A1's EXACT intervals the middle Pin's fix at 21 in has r 4.6, so with a 5-in tolerance
  and a +-1-in pose the scene goes AMBIGUOUS straight to NOT IN VIEW (the disc's near edge leaves the frame at ~20 in).
  With 6 in (half the Pins' spacing) it is FOUND at 22–23 in, never a neighbour (`probe_pos/f1b.cpp`; F1 pins it).
  F8/F9 moved to 23 in with 6 in. The lying Pin's fix is wide (half a length either way), so a lying route with 8 in
  never FOUND it in F4's sweep — characterised, its fix checked to hold the centre at every distance. The guide says
  all three.

### The remaining tests (`date` ~16:15)
- **`test/piece_finder_test.cpp`** (14 cases, hand-made frames from an EXACT host camera): every config and target
  refusal by its words (the new `raisedMinIn` among them; a class bit past five, 1<<31 too); the words; the top
  face's allowance at its edges (lens AT the top 0, the next double above it > 0); `sightPiece` interprets only an
  Object of a named class; a side-clipped box UNSURE with no bound; the aspect thresholds LANDED ON; EVERY non-Ok status
  UNKNOWN "read not usable" (A2-D5) and an empty usable read after it NONE again; every non-Matches model check; a
  POSSIBLY FULL read; the alive and hold edges landed on; the score floor landed on; INSIDE / BORDER / OUTSIDE landed on
  from the sighting's own fix (`dist + r == tol` FOUND, the next double smaller the border; `dist - r == tol` the
  border, the next double smaller NONE); the error larger than the tolerance; two inside; the class mask; an
  unmeasurable box in and out of the disc's cone of bearings; the wrong posture (and the lying route taking it); the
  hold restarted by a jump past the two radii; NOT IN VIEW for a disc whose five points are in view but which pokes
  out diagonally (A2-D6; the place found by search with `inViewForCertain`). My errors fixed in the test: the first Pin
  at 40 in was AMBIGUOUS at a 5-in tolerance (its fix ~6 in) — the cases moved to 30 in with 6 in; the lying helper
  first left the host model's lying heading at 0 (END-ON, so UNSURE) — set across the view.
- **`test/camera_feed_test.cpp`** (6 cases): the config's refusals by words; the period LANDED ON (1/32 s; the double
  before does not read); two consumers in one instant cost one read; the period from a read's START (a camera that
  takes 1/64 s on the feed's own clock); latest() a fault frame with no detections, never the last good one; the stats
  by status (`truncated` only on usable reads, the withheld counts summed); each read timed (1/256 s exactly), the
  slowest never lowered.
- **`test/vision_guide_example_test.cpp`** (5 cases, guide-19a..e): the chapter's listings verbatim; one loop ticked
  every 10 ms for a second reads 25..34 times; the tracker's RED after its hold (and NotHeldYet on the first read);
  the finder's FOUND, its fix holding the Pin; `boundAtImage`. Scene tuning logged: a level 10-in lens put the
  disc's near edge out of the frame at 23 in (NOT IN VIEW), so the example's camera is tipped down 10 degrees and
  its Pin 26 in out (`probe_pos/g19.cpp`).
- **Suite after all tests:** `suite_run.py show` -> 2,912 cases, 0 failed, 3 skipped (2,815 at the base: +97).
  The new tests' own time: ~4 s (`-d`: R5 0.8 s, the real shape 1.0 s, the Goal property 0.8 s).

### The documents (pace rule 3), `date` ~16:20–16:40
- NEW `docs/guide/19-the-camera.md` — the chapter §3.1 asks for: one loop and the feed; the team's numbers; freshness
  as time (frozen only while moving; the picture's age and `boundAtImage`); the tracker's gates and its UNKNOWNs; what
  a colour MEANS (the face SEEN, never set; a stuck Toggle's destination colour — R4's 78 of 216; colour confirmed AT
  THE PUSH POINT); what turning costs (R7); the lookalikes (70 of 540; the wall-top strip; the long strip from 22 in;
  0 with the position gate); D9's mount table from THIS run's R5; the finder's five verdicts; posture with
  `raisedMinIn` (A2-D7's evidence); how close is close enough (F1 measured: 22–23 in at 6 in, never at 5 in; the
  lying Pin's wide fix). Every ```cpp listing is lines of `vision_guide_example_test.cpp` (the build's example scan:
  654 quoted lines, all verbatim, 11 sources). Its two hand-kept index entries (re-check T8): a row in
  `docs/guide/README.md`'s table (19 — the next free number at this tip, after MECH-A3's 18) and a line in
  `mkdocs.yml`'s "User guide" nav.
- `docs/changelog.md` — the entry at the top of `## API 2.5` (additive, no bump; `raisedMinIn` named as beyond the
  brief; the suite count to be filled at the end).
- `docs/guide/14-what-it-cannot-do-yet.md` — a NEW section before "No easier tiers yet": the camera's honest limits
  (no sensor; a colour seen not set; lookalikes; frozen only while moving; near-square close in; the blind band; the
  picture's age; FOUND only close in and a Goal far out UNSURE; no tags — "no camera has been pointed at a tag" kept).
- `docs/hardware-assumptions.md` — the Source lines of HA-197 (the finder's model check), HA-199 (the tracker; the
  claim's lookalike count 65 -> 70 on this code, 65 named as the prototypes', plus the wall-top strip), HA-200 (the
  finder's posture and fixes), HA-203 (the feed's timing), and HA-202 (the tracker's band rows — the stopped run's A1
  text pointed at "VIS1-A2" there too); the VIS1 paragraph's "add their sites then" made present tense. No new number.
- `docs/api/` regenerated (three new pages; `camera_geometry.md`'s two members; README / all-entities / mkdocs nav).
- `test/README.md` — the scene rig documented for test authors (beside the host camera model's paragraph).
- Prose-review fixes outside the list (rule 4 — each a sentence this change made false): `docs/guide/10-the-api.md`'s
  camera section ("Nothing in the library READS it yet: the camera's consumers ... are the camera's next part" ->
  points to chapter 19); `docs/roadmap.md`'s Freeze Register row F15 (the three headers named as present; "the first
  two" -> the tracker, finder and feed are its first consumers, read by no program yet). Only those two cells/lines.
- Deliberately NOT touched: the glossary (chapter 19 defines its words; a glossary pass would be a sixth public
  document — rule 7 — and the docs redo after the panel is where it belongs; FOR THE COORDINATOR); the shared plan
  documents' status lines (`build-order.md`'s VIS1 entry and `**Next:**`, `REMAINING-WORK.md` row 18d, the roadmap's
  "Now" paragraph ~:438 "Nothing reads the camera yet ... next for the camera is VIS1-A2", HANDOFF) — the coordinator's
  at the merge, as A1-D13 left them; `src/`, `tools/` (VIS1-B's / nobody's).

### The prose review (pace rule 4) — each grep RUN and its hits READ before this was written
- `grep -rn -i "next part\|camera's next\|part A2\|VIS1-A2\|tracker\|piece finder\|CameraFeed\|camera feed\|no consumer\|reads it yet\|READS it yet" docs/*.md docs/guide docs/cookbook README.md`:
  hits read — guide 10 :603 (STALE, fixed), roadmap F15 :149 (STALE, fixed), roadmap :438 (the coordinator's "Now",
  stale at the merge — left, named above), changelog :76 (A1's entry: history, true when written), HA register :1194
  (fixed) and the Source lines (fixed); every other hit another chunk's "next part" (H1, CAL1, MECH — not VIS1's).
- §3.4's own grep, `grep -rn -i "vision\|camera\|AbsentVision\|DeviceKind::Vision\|VISION" docs/ README.md` (public,
  outside `docs/api`): 427 hits in 23 files, counted per file; read in full where VIS1-A2 could make a statement
  stale — guide 15 (glossary: frame, bearing, ground range, fix radius — still true; no A2 term claimed absent),
  guide 03 :84–255 (the GPS and tag correctors; "no camera has been pointed at a tag" stays true), guide 06 :78 (the
  camera answers a bad read with a frame that says so — true; the feed keeps it so), guides 01/11/12/13/16/18, faq,
  master plan (architecture; `IVision` stays as D11 says), README :136 (tags) — none stale. guide 10, 14, 19, the
  changelog and the register: edited above.
- The internal statements §3.4 names ("no adapter yet", "TAGS AND VISION ARE DELIBERATELY ABSENT", "vision ABSENT",
  "COLORS left as found", "interprets nothing", HA-70's "no library adapter") are A1's and VIS1-B's (`src/`); HA-70's
  sentence was already amended by A1 (:4263–4265, read). Nothing in them is made stale by A2.

### RESUMED (third stop) — Tue Nov 10 10:19:20 PM EST 2026 (`date`)
- **The stops.** This run was stopped at ~14:50 on 2026-11-03 (the one-job rule; the resumption above), then again at
  ~17:30 the same day (the 5-hour usage limit); the laptop was off until 2026-11-10. The resumption above logged
  everything to its prose review; the mutation run and the `src_build_gate.py` self-test it started at 16:38–16:40
  were running when it was stopped, unlogged — accounted here from the scratch files.
- **NEW SINCE THE STOP (the team lead, 2026-11-10, through the coordinator):** the first competition (Nov 21) is SKILLS
  ONLY; the auto-flipper is re-scoped to AF1-S (a skills route drives to a KNOWN Toggle, the camera checks its colour,
  the robot flips it only if it is not ours) — so the TRACKER's colour answer (RED / BLUE / YELLOW or UNKNOWN with its
  reason, from a fresh frame) is on the critical path and is finished first; the rest of the scope stays as briefed.
  ALLY1 and L4 are after the 21st. Main has moved past this base (L6-A2, MECH-S-A, STEP1-A, CAL1-B): the rebase is at
  the merge, not now.
- **FOUND: a gate self-test PLANT left in `src/programs/bench_r3a.cpp`** — 25 appended lines (`ser1ReviewPrintf` …
  `ser1ReviewSf`, `extern "C" std::FILE __sf[3]`), the file's mtime 17:08:27 on 11-03: `src_build_gate.py self-test`
  (its `SRC_PLANT`, :1875) killed mid-plant, so its `finally` never ran (a SIGTERM to a Python process without a
  handler skips `finally`). The coordinator's 11-10 audit did not see it. The diff was the plant ONLY (read whole);
  RESTORED by `git checkout -- src/programs/bench_r3a.cpp` + `touch` (this worktree's base copy: the file is
  MECH-A6's in flight, NOT edited by VIS1-A2); the planted copy kept as evidence at
  `scratch/vis1-a2/bak/LEFTOVER_srcgate_plant.bench_r3a.cpp`. `git diff -- src/ Makefile` now EMPTY.
  **FOR THE COORDINATOR:** `src_build_gate.py self-test` has no SIGTERM/SIGHUP handler, so a stopped run leaves its
  plant; worth adding one (a `tools/` change — not VIS1's) and worth checking every worktree whose run was stopped
  during a self-test.
- **The mutation run's state** (`scratch/vis1-a2/mutations.log`, `mutations_run2.txt`): MT1–MT25 and MA1–MA2 (27 of
  the table's 67) RED, each on a fresh link, every dependent object recompiled; MA3 was being built when the stop came —
  the driver's SIGTERM handler killed the build, restored `toggle_tracker.hpp` (17:08:45) and removed the binary (its
  last line: "signal 15: child killed, file restored, binary removed, exiting"). All 28 backups and the live file are
  byte-identical (md5 `cd5fa09f…`). No other header was ever mutated.
- **Fresh build** (`date` 22:19): `rm -f build/test/shulib_tests`, the briefing regenerated, configure, build — the
  five objects depending on `toggle_tracker.hpp` (the two tracker tests, the two finder tests, the guide example;
  checked in their `.o.d`) recompiled at 22:19:50, linked; the briefing regenerated with the binary present; the suite
  through `suite_run.py green` under the cap: **green — 2912 cases, 2959822 assertions** (the stopped run's count).
- **Continuing:** the mutation table from MA3 (40 left: MA3–MA8, MF1–MF9, MF1b, MA9–MA23, MD1–MD6, MG-A1–MG-A3) by the
  same driver; `src_build_gate.py` self-test and check beside it (no `src/` file includes the four mutated headers —
  grepped — so its builds cannot see a mutant); then the remaining gates on the restored tree.

### Tue Nov 10 22:21 – Wed Nov 11 00:05 EST 2026 — the mutation run finished, three survivors answered, one finding OPEN (`date`)
- **A MISTAKE OF MINE, caught by a build, logged:** I ran `src_build_gate.py self-test` BESIDE the mutation driver,
  reasoning that no `src/` file includes the four mutated headers. Wrong question: the self-test PLANTS into `src/`
  files the HOST suite compiles (`test/../src/robots/mechanism_table.hpp`, `src/programs/bench_r3a.cpp`), so a host
  build during a plant compiles the plant. Seen at MA9: "BUILD-BROKEN" on `'SHULIB_ROBOT_TANK_2026' was not declared`
  in `mechanism_table.hpp` — the self-test's plant, not the mutant. I stopped the driver (SIGTERM: its handler killed
  the build, restored `piece_finder.hpp` — byte-identical to all 15 backups — and removed the binary), let the
  self-test finish (OK, 269 detector cases; check PASS, all 7 builds; `src/` clean after), and RE-RAN every verdict
  that overlapped a plant. That includes the 11-03 run's MT1–MA2: its driver started at 16:40:03, the same minute as
  that day's self-test (`srcgate_selftest.txt`, 16:40) — the self-test whose plant the stop left behind. **Every
  mutation verdict below is from a run with no self-test alive** (`ps` checked; other worktrees' self-tests compile
  THEIR trees, not this one). FOR THE COORDINATOR: the self-test and a host build in the same tree must never
  overlap — worth a line in RESUMING's gate list.
- **The 40 from MA3** (`mutations_run4.txt`, quiet tree): RED — MA3–MA8; MF1–MF9, MF1b; MA9–MA11, MA13, MA16, MA18–MA23;
  MD1–MD6; MG-A1, MG-A3. **MG-A2 broke its own build** (`-Werror=unused-parameter`: the mutant left
  `minDepressionDeg` unused) — RE-CUT as `minDepressionDeg * 0.0 * kDegToRad` (still "never refuses"), never counted
  broken. **SURVIVED: MA12, MA14, MA15, MA17** (MA12 also survived the overlapped run).
- **Why they survived — one pattern:** each is a FAR-END term of a fix (a radius, the top face's allowance, half a
  length, r(sec b − 1)) that only decides when the fix's intervals are TIGHT; every scene so far used the stated
  tolerances (0.25 in, 1 degree, the optics' 5 %), whose width hides the term. A bound nobody can see go short is not
  tested. Answered with a camera MEASURED ON THE BENCH (`tightModel`: mount 0.01 in / 0.01 degree, optics 0.1 % — HA-194
  measured), each case first PINNING that its scene is on the edge (the true centre beyond the mutant's far end), then
  posture and "the fix holds the true centre" (`test/piece_finder_test.cpp`, three new cases + helpers
  `tightModel` / `tightTruth` / `tightBox`):
  - **MA12** (the floor fix's far end without the radius) — a 12-in lens pitched down 25, floor Pins 10..20 in at
    -10/0/+10 degrees: the ground range only 0.35–0.9 in wide, the centre 0.3–0.6 in past its far end. My first cut
    (a 10-in lens at 16–48 in) FAILED its own edge pin on the correct code — the optics' 5 % made the ground range
    1.2 in wide even at 10 in; the second (tight mount only) the same; measured, then the optics tightened.
  - **MA14** (the allowance dropped from RAISED) — an 18-in lens pitched down 30 over a Pin on the scenes' 5.8-in
    Goal, 10..14 in: the centre 0.4–0.6 in past the size range plus a radius (farther out the margin closes — measured
    to 24 in, pinned only where it holds).
  - **MA15** (a lying Pin's centre only a radius past its near end) — a 6-in lens, a Pin lying at ±45 degrees to the
    view, its near end 12..27 in out: LYING, its centre 1.0–1.3 in past the ground range plus a radius. Every lying
    scene before lay ACROSS the view, where a radius IS all the centre is past its nearest point.
  - **MA17** (the off-axis factor dropped) — NOT answered; see A2-F1: the scene that would show it (a Pin far off the
    axis) shows a LARGER, unrelated defect first.
  The re-run of MT1–MA2, MA12–MA15, MA17 and the re-cut MG-A2 on the final test file: (`mutations_run5.txt`, 2,915 cases, no self-test alive):
  **MT1–MT25, MA1, MA2 RED** (the same failing vision cases as 11-03; MT23 and MA1 each count ONE FEWER failing case
  than on 11-03 — the extra was not a vision case, so a plant did fail an unrelated test that day: the re-run was
  needed); **MA12, MA13, MA14, MA15 RED** (each on its tight case; MA13 also by the MA12 case); **MG-A2 (re-cut) RED**
  (1 case); **MA17 SURVIVED** — the one survivor, tied to A2-F1 below.
  **THE TABLE'S 67, FINAL: 66 RED, 1 SURVIVED (MA17), 0 broken.** Every run: the file backed up, the mutant written with
  its line count kept, the binary removed, the briefing regenerated, every object whose depfile names the file
  recompiled (checked), the WHOLE suite under the 4 GiB / 600 s cap, restored byte-exact in a `finally` and on
  SIGTERM/SIGHUP/SIGINT, and a clean rebuild at the end.
- **A2-F1 — OPEN FINDING (the finder; NOT fixed in this run — FOR THE COORDINATOR).** Hunting MA17 with a Pin far off the
  axis (a tight camera 12 in up pitched down 25; Pins 10..16 in at ±28 and ±31 degrees; the box's bottom edge ONE pixel
  low — inside HA-202's 2-px jitter, as a real box edge may be), the UNMUTATED finder's floor fix MISSED the true centre:
  11 in at ±31 degrees, error 0.438 in against a radius of 0.416 (reproducer: `scratch/vis1-a2/offaxis_reproducer.cpp.txt`).
  Diagnosed with the intervals printed: (1) the sighting's BEARING interval excludes the true bearing even on the
  plant's exact box (-32.46..-31.08 degrees for a Pin at -31: biased OUTWARD ~0.77 degree, the jitter's half-width
  0.69); (2) the floor fix's near end `max(G.lo, Z.lo) + r` = 11.09 in, beyond the true 11.00. ROOT CAUSE (one, in
  `sightPiece`): under a pitched camera a standing Pin's TOP circle is nearer the lens, so it is the widest
  cross-section in pixels and sets the box's centre column — the Pin's axis at the TOP row — but the finder evaluates
  the bearing at the box's MIDDLE row (`m.bearing(u, d.top + d.height / 2.0)`, `piece_finder.hpp` :289) and the
  ground range at its BOTTOM row (`m.groundRange(u, vBottom)`); off the axis a fixed column's azimuth changes with the
  row, so both are biased outward (the ground range by X (sec b_col − sec b) = 0.17 in here, against the r (sec b − 1)
  = 0.13 the bound allows). Shown: for a vertical cylinder whose cross-sections' widths are ordered along it, the box's
  centre column lies between the axis's columns at its top and bottom rows. PROPOSED FIX (not built): (a) the bearing
  as the hull of `bearing(u, top)` and `bearing(u, bottom)` (a fixed column's azimuth is monotone in the row; `bearing()`
  already handles the level-row interior point); (b) the floor fix's ground terms from the row's DEPTH, which is
  column-independent for a camera with no roll (a row's floor points share one forward distance X; at the principal
  column the ground range IS X): the centre's X in [X.lo + r, X.hi + r] per corner camera, its range X / cos α over the
  bearing interval relative to that camera's yaw — which also replaces the `sec` factor MA17 mutates. WHY NOT NOW: it
  changes §1.13's arithmetic in two places beyond the brief and moves documented numbers (F1's 22–23 in at 6 in — its
  side Pins sit ~27 degrees off the axis — and guide 19's), it wants a reviewer's eye on A1's geometry, and the finder
  is L4's, AFTER Nov 21 (the team lead, 11-10), while this run's critical path is the tracker. The finder's existing
  claims stay true where they were measured: the property sweeps (realistic tolerances, true cameras at the tolerance
  box's edges, 1-px jitter, ~26,000 sightings) never missed; the miss needs a near-exact camera AND a box edge pushed
  toward the jitter's limit at the picture's side, and is 0.02 in past a 0.42-in radius. **The tracker does not take
  this path for its colour** (a band's colour never depends on a bearing; its bearing feeds only gate 6's window and the
  optional position gate, and AF1-S faces the Toggle square), so the critical path is unaffected. Guide 14 and the
  guide's finder section gain one sentence each (below) so no reader takes the off-axis fix as proven.
- **Documents for A2-F1 and this round** (pace rule 3): `docs/changelog.md` (the count 2,815 → 2,915; "every
  mutation but one"; the open case named); `docs/guide/19-the-camera.md` (one paragraph after "every field position
  held the Pin"); `docs/guide/14-what-it-cannot-do-yet.md` (one bullet in the camera section). No `///` comment
  changed (no `docs/api` regeneration); no HA number (a library arithmetic gap, not a hardware belief — HA-202's 2-px
  jitter is what the reproducer stays inside).
- **The source says it too** (rule 3; found by this round's prose grep — the header's "What the fix is a bound on"
  claimed a bound A2-F1 breaks at the picture's side): `include/shulib/vision/piece_finder.hpp` gains an 8-line
  `//` paragraph "KNOWN GAP (VIS1-A2's open finding A2-F1)" in its header comment (no `///` changed; `docs/api`
  regenerated only for the moved line numbers; `mkdocs.yml`'s diff re-read: still only the chapter-19 row and the
  three API rows). Comment-only: every mutation anchor is unchanged, so the verdicts above stand on this code.
- **The prose review of this round** (pace rule 4 — each grep RUN and its hits READ before this was written):
  `grep -rn -i "every field position held\|every fix hold\|fix holding\|holds the true centre\|holding the true
  centre\|holds its true centre\|each turned the suite red\|2,9xx" docs/*.md docs/guide docs/cookbook README.md` —
  2 hits: guide 19 :261 and the changelog :69, each now followed by the open case (edited above); `grep -n -i
  "holds\|contains the\|bound on\|certainly" include/shulib/vision/piece_finder.hpp` — :58 "What the fix is a bound
  on" (QUALIFIED, above), the rest about FOUND/NOT IN VIEW's "certainly" (true: they rest on the fix's disc and
  inherit A2-F1, which the new paragraph names); the HA register's VIS1 Source lines (:1290–:1415) read — none claims a
  fix holds; the earlier round's stale-statement grep re-run (`next part\|camera's next\|part A2\|VIS1-A2\|reads it
  yet\|no consumer`) — the same hits as 11-03 (roadmap :438 the coordinator's "Now", left for the merge; the rest other
  chunks' or VIS1's own Source lines).

### The gates on the final tree (`date` Wed Nov 11 00:05–00:40 EST 2026, after every edit; nothing else running in this tree)
- Host build (binary removed, briefing regenerated, doc gates in the build): rc 0, no warning; the briefing
  regenerated with the binary present; `suite_run.py green` under the cap: **green — 2915 cases, 2960116 assertions**
  (2,815 at the base: +100).
- `ctest -R compile_refusal`: **100% tests passed, 0 tests failed out of 255**.
- `src_build_gate.py self-test`: **OK (269 detector cases, each against a real build)**; `check`: **PASS (all 7
  builds link …)**; `git diff --stat -- src/ Makefile tools/` EMPTY after (`scratch/vis1-a2/srcgate_final.txt`).
- ARM header gate over **238** headers (`-Werror`, cortex-a9): rc 0. CI greps (pros/ outside hal/pros; sim/ outside
  sim): empty. The brief's own: F4 `vision.hpp` untouched; `PROBE|prototype|proposed` in the vision headers: empty;
  `internal/|chunks/|RESUMING|build-order` in public docs: empty.
- `api_doc_tool.py` self-test OK; check-coverage 5,426 entities all documented; check-fresh rc 0; check-examples 654
  quoted lines all verbatim (11 sources); check-removability clean. `doc_staleness_audit.py` clean;
  `briefing_status.py check` current.
- A slip of mine: one gate loop above captured each tool's last line through a temporary file under `/tmp`
  (`/tmp/null_<pid>`, deleted in the same command) — against the scratch-only rule; nothing else touched `/tmp`.

### The tree at the end of this run — `date` Wed Nov 11 00:40 EST 2026
- NOT committed. CHANGED: `include/shulib/vision/camera_geometry.hpp` (A2-D3), `docs/{changelog,hardware-assumptions,
  roadmap}.md`, `docs/guide/{10-the-api,14-what-it-cannot-do-yet,README}.md`, `docs/api/{README,all-entities,
  camera_geometry}.md`, `mkdocs.yml`, `test/README.md`, the briefing (generated), this log. NEW: `include/shulib/
  vision/{toggle_tracker,piece_finder,camera_feed}.hpp`, `docs/guide/19-the-camera.md`, `docs/api/{toggle_tracker,
  piece_finder,camera_feed}.md`, `test/{toggle_tracker,toggle_tracker_scenes,piece_finder,piece_finder_scenes,
  camera_feed,vision_guide_example}_test.cpp`, `test/camera_scene_rig.hpp`. `src/`, `tools/`, `Makefile`,
  `test/CMakeLists.txt` untouched (the suite globs its sources). No mutant, no plant, no stray root file; every backup
  under `scratch/vis1-a2/bak/`.
- **Open, for the coordinator and the reviewers:** A2-F1 (the finder's off-axis bearing and floor range; MA17 survives
  until its fix); the self-test's missing SIGTERM handler; the self-test never beside a host build; `raisedMinIn`
  (A2-D7, a team number beyond the brief, no HA number taken); the glossary pass deferred (rule 7); the shared plan
  docs' status lines (build-order, REMAINING-WORK 18d, roadmap :438 "Now", HANDOFF) left for the merge.

### RESUMED (fourth stop) — Wed Nov 11 02:00 EST 2026 (`date`)
- **The stop.** The team lead stopped all work at ~00:30 on 11-11 (usage credits low) and resumed at ~01:30. This
  build run had ALREADY FINISHED at 00:40 (the section above: every test, the mutation table, the documents, the prose
  review and the gates); the stop caught the three REVIEWERS (scratch `review-correctness/`, `review-docs/`,
  `review-tests/`, `review-tests-gates/`, `review-tests-drv/`, 00:42–01:01 — each in its OWN copy of the tree, never
  this worktree: the reviewer's mutation driver's root is `scratch/vis1-a2/review-tests`; its last line "signal 15:
  child killed, file restored, binary removed, exiting"). Their findings are the coordinator's, not this log's.
- **Audit at the resumption:** `git diff -- src/ Makefile` EMPTY (no gate plant); no file of the tree newer than 00:36
  but the briefing and this log; the live headers' md5 equal to every backup's base (toggle_tracker `cd5fa09f`, 33;
  camera_feed `372c28ce`, 6; camera_geometry `4eee2bb1`, 4; piece_finder `1d41b8e6`, the reviewer's 21 — this run's
  25 are of the copy before the comment-only KNOWN GAP paragraph, `58d387d9`, as logged); no shulib_tests or
  src_build_gate process alive. Nothing to continue but a FRESH build, the suite and the gates on this tree (below):
  no finished work redone, none reverted.
- **The gates on this tree, re-run from source** (`date` 02:01–02:39; nothing else of this tree running; outputs in
  `scratch/vis1-a2/resume4_*.txt`): `rm -f build/test/shulib_tests`, the briefing regenerated, a CLEAN host build (`--target
  clean`, all 257 objects compiled from source; no warning), the briefing regenerated with the binary present;
  `suite_run.py green` under the 4 GiB / 600 s cap: **green — 2915 cases, 2960116 assertions** (the 00:35 count).
  `src_build_gate.py self-test` **OK (269 detector cases)**, then `check` **PASS (all 7 builds link)** — run ALONE, no
  host build beside it (the 11-10 lesson); `git diff --stat -- src/ Makefile tools/` EMPTY after. Because the
  self-test's restores re-stamp `src/` headers the suite compiles, the host build was run AGAIN after it (257 objects,
  rc 0), the briefing regenerated, and the suite re-run: **green — 2915 cases, 2960116 assertions**. `ctest -R
  compile_refusal`: **255 of 255**. ARM header gate over **238** headers (`-Werror`, cortex-a9; the TU under scratch):
  rc 0. CI greps (pros/ outside hal/pros; sim/ outside sim): empty. `api_doc_tool.py` self-test OK, check-coverage
  5,426 entities all documented, check-fresh rc 0, check-examples 654 quoted lines all verbatim (11 sources),
  check-removability clean; `doc_staleness_audit.py` clean (counts derived from the fresh binary); `briefing_status.py
  check` current. The prose review's stale-statement grep re-run: the same hits as at 00:05 (roadmap :438, the
  coordinator's "Now", left for the merge). No edit to any file but this log and the generated briefing.
- **State:** VIS1-A2's BUILD is complete; NOT committed. Open for the coordinator, unchanged: A2-F1 (MA17); the
  self-test's missing SIGTERM handler; the self-test never beside a host build; `raisedMinIn` (A2-D7); the glossary;
  the shared plan docs' status lines. The interrupted reviewers' scratch copies are theirs to resume or restart.

## VIS1-A2 review findings and their disposition

- **Started:** Wed Nov 11 04:00:37 AM EST 2026 (`date`), the builder again, in the worktree `.claude/worktrees/vis1-a2` (branch `vis1-a2`, base
  `9d77c9d`). Three reviewers' findings (correctness, tests, docs — 24 after dedupe), each re-checked against this tree
  before any edit; fixed at the root or answered with evidence. Probes and backups under `scratch/vis1-a2/fix1/`. Not
  committed; no `pros`; no `/dev/ttyACM*`; every test binary under the 4 GiB / 600 s cap.


### RESUMED (this fix run was stopped by the 5-hour usage limit at ~04:40; resumed by the builder) — `date` Wed Nov 11 07:44:59 AM EST 2026
- **What the stopped run left (it logged nothing past "Started"; reconstructed from the tree, never from memory):**
  `git diff -- src/ Makefile` EMPTY (no src-gate plant). Its mutation driver (`scratch/vis1-a2/fix1/mt/mutate.py`,
  restore in `finally` and on SIGTERM/SIGHUP/SIGINT) worked in a COPY of the tree (`fix1/mt/tree`), never in this
  worktree; its log ends "signal 15: child killed, file restored, binary removed, exiting" after RT2, RT3 and RT4 —
  each RED (1 of 2,937 cases, "a VETO read in the middle of a SURE hold breaks it"), on fresh binaries (04:38–04:41).
  This worktree's three edited headers against that copy: the CODE identical with comments stripped (`diff` of
  `sed 's://.*$::'`: empty for `toggle_tracker.hpp`, `piece_finder.hpp`, `camera_geometry.hpp`, `robot.hpp`); the
  differences are comment edits made at 04:38–04:39, after the copy was taken. The tests are byte-identical to the
  copy's. So no mutant is applied anywhere in the worktree, and RT2–RT4's verdicts stand for this code. The stopped
  run's backups `fix1/bak/*.orig` are the PRE-FIX headers (md5 `cd5fa09f` toggle_tracker, `1d41b8e6` piece_finder,
  `4eee2bb1` camera_geometry — equal to the 02:01 audit's bases and to the reviewers' copy `review-strength/tree`),
  not mutants.
- **What it had done** (the diff against `fix1/bak/*.orig` and `review-strength/tree`): every code fix below
  (the range bound, the same-band hold, the width check and the lying exclusion, the NoTarget answer, the −1
  UNSETs), the new tests in `toggle_tracker_test.cpp`, `toggle_tracker_scenes_test.cpp`, `piece_finder_test.cpp`,
  `piece_finder_scenes_test.cpp` and `vision_guide_example_test.cpp`, and the edits to guides 14 and 19, the changelog
  (still holding a `SUITE_COUNT` placeholder), `camera_geometry.hpp`'s `kBoxEdgePx` comment and `robot.hpp`. NOT done:
  the register (HA-192/198/199/202), the R7 header line's 40–48-in figures, `maxFixRadiusIn`'s edge, the mutation
  run past RT4, every disposition below, the gates.
- **Relinked from source before trusting any result** (`date` 07:46–07:49; `scratch/vis1-a2/fix2/rebuild.sh`):
  `rm -f build/test/shulib_tests`, `api_doc_tool.py generate`, the briefing regenerated (binary absent), configure, a
  CLEAN host build (`--target clean`; **257 objects compiled, no warning**), the briefing regenerated with the binary;
  `suite_run.py green` under the 4 GiB / 600 s cap: **green — 2,937 cases, 2,960,622 assertions**.

### The findings, each re-checked against this tree, then dispositioned (C = correctness, T = tests, D = docs)
Probes re-run on THIS code are under `scratch/vis1-a2/fix2/` (`pb.sh` compiles a probe against the worktree's
headers and runs it under the 4 GiB / 600 s cap; outputs in `probes1.txt`).

- **C1 (major) — a MISSED field-face detection lets its bottom face's colour through. REAL; answered by the finding's
  option (b), the dependency STATED and PINNED; option (a) measured and rejected.** Re-checked: the reviewer's
  `p1_noface` on this code still prints "cases 6240, a colour (wrong …) 658" — the same as reported; it is the
  rule, not a slip. **Option (a)** (every band some corner explains as a bottom face, with no band above it,
  answers "maybe the bottom face") rebuilt here as a scratch header (`fix2/optA/`, the one line in
  `bottomFace()`), the scene tests compiled against it and against the shipped header, both run capped: with (a),
  a 6-in level lens reads the face at **0 of 39** parkings (edges 12.55/12.61/12.65; shipped: 25–27), an 8-in lens
  at 2–7 (shipped: 27–29), the real shape **204** RED of 2,288 (shipped: 820), R7 70 of 390 (shipped: 106) — AND
  it does not close the hole: the missed-field-face scene still reads the bottom face's YELLOW sure in **16 of
  600** parkings with (a) (70 shipped), because a bottom face some corner does NOT explain as one is judged as a
  face. (The stopped run had measured the same coverage at 04:02, `fix1/trialA_msgs.txt`; reproduced here.) So (a)
  costs most of the coverage D9's mounts rest on and still is not safe; the real remedy is a fact about the camera.
  **Option (b), done:** `toggle_tracker.hpp` gate 5 "WHAT THIS RESTS ON"; HA-199's claim now INCLUDES "the field
  face's descriptor fires on every read on which its bottom face's does", its blast radius names the wrong colour,
  its Settle a bench step (a real Toggle from a lens below its face under event light, RECORD lines — the bottom
  face's band never present without the field face's), the index row amended; guide 19 (gate 5's paragraph) and
  guide 14 (a new bullet); the changelog. **Pinned:** `toggle scenes … the missed field face` (600 parkings, the
  field face's detection removed: `CHECK(removedWrong > 0)` — the dependency is real, so a change that removes it
  is SEEN; the face cut short by 0..6 px: `CHECK(shortWrong == 0)`).
- **C2 (major) — a Pin LYING near end-on read STANDING ON THE FLOOR. REAL; FIXED at the root.** ON THE FLOOR now
  also needs (1) the box's WIDTH to be an upright Pin's at the range the corner agreed on (`detail::uprightWidthFits`
  — bounds, not estimates: the true width is at least D·fx/zMax and at most the farthest-out and nearest-in points'
  spread; it never refuses on what it cannot bound) and (2) no camera of the box able to see it LYING on the floor
  (its ground range's far end plus the lying Pin's diagonal against the nearest range at which its top row meets a
  Pin's diameter). Re-run on this code: `p8b 0 1` (pose exact, 1-px jitter, 99 % of the tolerance box) — **0**
  ON-FLOOR lying Pins at all five mounts (reported 17 / 0 / 0 / 323 / 5,876) and **0** fixes missing; `p8b 0 0 0`
  (the camera exactly true, no jitter) — **0** (reported 54 at 14/−15, 1,200 at 18/−20); `p9b` — "159744 unclipped
  sightings of a lying Pin; **0** read STANDING ON THE FLOOR; **0** phantom FOUNDs" (reported 3,659 and 19);
  `p5_endon` (end-on on a Goal) — "128 sightings …; 0 … ON THE FLOOR; 0 phantom FOUNDs". **The cost, measured:**
  the floor-Pin posture counts of the property test on this code equal the PRE-FIX binary's at all five mounts
  (6/0: 2,750 ON-FLOOR of 2,753; 10/−10: 2,682; 10/0: 2,379; 14/−15: 2,073; 18/−20: 276 of 2,549 — the pre-fix
  binary `review-docsbrief/shulib_tests`, a clean 02:41 link, run capped with the same filter): no standing Pin
  was lost. Header (`Posture::Unsure`/`OnFloor` comments, the posture paragraph), guide 19, guide 14, changelog.
  Tests: `piece finder: a Pin LYING near END-ON is never 'standing on the floor'` (floor, 18-in lens pitched −20; Goal
  top, 10-in), `piece scenes PROPERTY: a Pin LYING on the floor at every heading …` (five mounts, true cameras at the
  box's edges, `onFloor == 0`, `missed == 0`). Mutations W1–W3, L1–L4 below.
- **C3 (minor) — the view's RANGE was a hull over the explaining corners only, not a bound. REAL; FIXED.**
  `measureToggleFace()` now takes the range over EVERY camera of the tolerance box: the near end from all corners'
  edge rays, the far end INFINITE when a ray at some corner cannot reach its plane and the length does not bound it
  (the convention `groundRange()` follows: refuse rather than report a false finite), then the length's window over
  the focal lengths' extremes; `impliedLength` from the same extremes. `ToggleView::range` and
  `ToggleFaceMeasure::range` documented as that bound. Re-run: `p11_range` — "11538 sure views; the true range
  outside the view's interval: **0**" (reported 11). Test: `toggle scenes the view's RANGE is a BOUND …` (lenses
  6/8/11, pitch 0/10, three lower edges, 6–90 in; `missed == 0`, `views > 500`; the review's 11-in, 64-in case
  exactly). Mutations NR1–NR4 below.
- **C4 (minor) — the hold was by COLOUR, the header said by BAND. REAL; FIXED in the code (the header was right).**
  `update()` restarts the hold when the front band is not the same band: its direction from the lens (the heading
  plus its bearing) and its range must overlap the last read's, widened only by what the lens's travel since then
  explains, and its Toggle place (with the position gate on) must match. Tests: a nearer band of the colour, in
  another direction, taking over mid-hold → "not held yet", then held again; the same with a camera measured to
  0.01 in/0.01° where the RANGE alone decides (bearings overlapping, `REQUIRE`d). Header, guide 19, changelog.
  Mutations SB1–SB5 below.
- **C5 (minor) — the finder with no target answered NONE "nothing there". REAL; FIXED.** `PieceWhy::NoTarget`
  ("no target"): `answer()` is UNKNOWN until `setTarget()` is given a target `pieceTargetProblem()` accepts (a refused
  one is no target). Re-run: `p14_notarget` — "UNKNOWN / no target" (reported "NONE / nothing there"). Test: fresh
  finder, a refused target, then a valid one (FOUND), the words. Mutations NT1–NT2 below.
- **C6 (minor) — `kBoxEdgePx` has no margin; HA-202 did not say whether the 2 px includes rounding. REAL as a
  documentation gap; not a library defect** (the reviewer's own words: the input exceeds the stated 2 px). HA-202's
  claim now bounds the REPORTED integer edge against the true silhouette — jitter, rounding and steady bias all
  inside 2 px — states the zero margin (1-px plant jitter: no standing Pin's fix missed; 2 px + rounding, ~3 px:
  missed by up to 1.8 in), and its Settle measures the BIAS (the box's edges against the edges the measured mount
  and the tape predict), not only the spread; the index row amended. `kBoxEdgePx`'s comment says the same; the
  finder header's "no miss in ~26,000 sightings" now names the 1-px plant jitter it was measured at; guides 19 and 14.
- **T1 (major) — a veto read DURING a hold was never tested. REAL; FIXED.** Test `toggle tracker: a VETO read in the
  middle of a SURE hold breaks it at that read …` — two colours, a cut band of another colour in the window,
  another colour directly above: UNKNOWN with the matching reason at that read, "not held yet" with
  `heldSeconds == 0` on the next clean read. RT2, RT3, RT4: RED on fresh binaries (the stopped run's record above).
- **T2 (major) — the "certainly outside" cut band never reached the window check. REAL; FIXED.** The test's cut boxes
  are now 30 × 8 (aspect 3.75 ≥ minAspect 3, so they reach the CUT gate), one certainly right of a 10° window
  (`REQUIRE`d from the model's bearing) and one inside; the reason ORDER is checked (maybe the bottom face beside a
  cut band → "maybe the bottom face"; the cut band alone → "cut"; nothing → "no band"). X14, X15 below.
- **T3 (major) — gate 5's boundaries and the invalid-bearing veto untested.** Test `gate 5's edges LANDED ON`: the
  box above's bottom row AT the band's middle row (bottom face) and one lower (not); its reach exactly 3 ×
  kBoxEdgePx above (bottom face) and one row higher (not); half the narrower box's columns overlapping and one fewer;
  the apex row just inside 2 × kBoxEdgePx of the top (an oracle written from the header, `REQUIRE`d); the depth's
  1-in floor decided by a 10°-up camera whose band no corner explains with the floor but one would without it. RT8,
  RT9, RT18, X7, X24 below. **RT16 is EQUIVALENT** (argued from the code, not run away from): `touchesWindow(b)`'s
  `!b.valid` is reached only with an invalid bearing; at :521 the bearing is `CameraModel::bearing()`, which widens
  a finite ray from every corner (`widen()` drops only a non-finite one), and a model `cameraModelProblem()` accepts
  (the tracker's PRECONDITION) gives finite rays for the integer box; at :531 `b` is a PASSED band's, and `gate()`
  passes only a valid bearing. No input the contract allows makes it invalid; the clause stays as the safe default.
- **T4 (major) — the finder's safety bounds largely untested. REAL; tests added; every mutation re-run here.**
  New tests land on each bound: the bottom-cut box dropped only when CERTAINLY nearer — each term of the disc's
  nearest point (the tolerance, the pose's position tolerance, the lens's swing, the mount's position tolerance)
  with the place moved out by less than that term → AMBIGUOUS, and the bound itself written again from the header
  (`==`), a box cut at the left as well bounding nothing; the unmeasurable box's bearing cone LANDED between the
  bound with and without the heading tolerance and the pose's position tolerance, and a lens INSIDE the disc
  counting anything; NOT IN VIEW checking the floor as well as a Pin's height, and its heading slack landed; the
  raise exclusion's slack and the lens's height tolerance landed; RAISED with the lens below the support (no fix)
  and its fix's near end through a near-exact camera; a truncated or unconfirmed-model read mid-FOUND breaking the
  hold. The verdicts, each survivor judged, are in the table below.
- **T5 (minor) — the position gate tested only at place 0. REAL; FIXED.** Test `the POSITION gate at EACH of the four
  Toggle places`: a Toggle on each wall, the robot facing it, `toggleIndex == i`; the pose bound exactly
  `maxPoseToleranceIn` checks and the next double skips; tolerance 0 is the gate OFF; a fix off the line by more
  than the tolerance but within it plus the fix's radius passes (and past it does not), across and along. **Added
  at this resumption:** `maxFixRadiusIn` LANDED ON — exactly the band's fix radius is placed (`toggleIndex == 0`),
  the next double down is too vague (UNKNOWN); mutation FR1 below. X9, X11b, X12, X13, RT11, RT22 below.
- **T6 (minor) — additions with no failing test. REAL; FIXED.** Tests: a frame whose count says 25 with a passing
  band of another colour placed in memory right after the array (`REQUIRE`d adjacent) — read only to 24, RED; a
  frozen view with the gate on (`positionChecked == false`, `toggleIndex == -1`, `heldSeconds == 0`); the length
  gate's minimum and maximum fractions LANDED ON at the jittered widths (the largest/smallest that pass found by
  bisection on an oracle of the per-corner edge agreement, one double past each refuses) and `impliedLength`
  exactly `wLo·lo/fxMax` and `wHi·hi/fxMin`; a new colour after a frozen view starts a fresh hold, a box that keeps
  its corner but changes size has moved. X3–X6, X18, X19, RT14, RT15, RT17, RT20, RT24 below.
- **T7 (minor) — "doc_staleness_audit clean (counts derived from the fresh binary)" (the fourth resumption, above)
  is not what the saved outputs show. REAL — my error.** `resume4_clean.txt` and `resume4_build2.txt` show the audit
  ran inside the build's doc gates with the binary deleted ("[cases/assertions NOT derived — the test binary is not
  built]"). The audit checked 0 numeric claims either way, so nothing it judged changes; the sentence above stays as
  written (this log is history) and is CORRECTED here. This round runs the audit AFTER the link, alone, and saves
  its output (`fix2/gates_*.txt`, below).
- **D1 (major) — the position gate documented as turning away a lookalike it cannot. REAL; FIXED.** The gate removes
  OFF-LINE bands (P4b: 70 → 0) and the wall-top LED strip (with the gate on: UNKNOWN at 48/30/20/14 in); it CANNOT
  remove a Toggle-length red strip ON a Toggle's face line, which reads RED from 22 to 30 in named as that Toggle.
  The reviewer's `gateprobe` re-run on this code: "T10 gate=1 d=22 … d=30 colour=1 why=0 checked=1 idx=0"; d ≤ 20
  "why=5" (no band); d = 32 "why=9" (not held yet: some reads' fixes are wider than 8 in); d ≥ 34 "why=5". Fixed in
  the header, HA-199 (claim — "from 20 in or CLOSER" and "about 22 in or FARTHER" now say which way — and index
  row), guide 19 and guide 14; S5b's title corrected and S5b now runs with the gate OFF and ON; NEW T10b pins the
  limit with the gate ON (RED, `positionChecked`, `toggleIndex == 0` at 22–30 in; UNKNOWN at 8–20 and 32–40).
- **D2 (major) — guide 19's R7 coverage was the brief's prototype table. REAL; FIXED.** The R7 test prints the
  coverage per lens, bottom face and distance (the brief's §2.2 "the coverage printed") and pins its two stated
  facts (`squareMissed == 0`, `past15 == 0`). Run here (`-tc="*R7*" -s`): 8-in lens — 14/20/30 in square only, 40 in
  ±5°, 48 in ±10°; 10-in — 14/20 square only, 30 ±5°, 40/48 ±15°; 11-in — 14 square only, 20 in −5..0, 30 in
  −15..+10, 40/48 ±15°; the same for both bottom colours. Guide 19's table matches it line for line; the header's
  sentence, which said "at 40–48 in up to 10–15", was loose for the 8-in lens (±5° at 40 in) and is CORRECTED at
  this resumption ("at 40 in 5 (8 in) to 15; at 48 in 10 (8 in) to 15"); guide 14's bullet agrees.
- **D3 (minor) — "about 30" reads a second; the code reads 25. REAL; FIXED.** Guide 19 says 25 and why (the period
  rounded up to the loop's tick: every fourth 10-ms tick, 40 ms); `guide-19b` checks `reads == 25` exactly, its
  title corrected.
- **D4 (minor) — the changelog overclaimed. REAL; FIXED.** "each term … but the off-axis factor"; the landed edges
  are listed by name (the hold, alive, aspect, window, colour-above, bottom-face, frozen, length, position-gate and
  the finder's in/border/out); the position gate's `maxFixRadiusIn` was the one listed edge NOT landed — landed at
  this resumption (T5). The suite count filled in (2,815 → 2,943, this round's tests included). The mutations sentence points at this log, where
  each survivor is judged.
- **D5 (minor) — `robot.hpp` called the feed "the camera's next part". REAL; FIXED** (`vision::CameraFeed`,
  `vision/camera_feed.hpp`) — `robot.hpp` is shared with L6: a one-phrase comment edit, logged here for the merge.
  The stale-statement grep re-run over `include/shulib` and `src` (prose review below).
- **D6 (minor) — `minScore` / `viewMarginPx` accepted UNSET. REAL; FIXED** (the delta is now logged, D9).
  Both default to −1 (UNSET) and are refused by name ("minScore UNSET (negative) -- the lowest model score
  believed, 0 for every score"; "viewMarginPx UNSET (negative) -- … 0 for none"); 0 keeps its meaning. Tests: the
  defaults are −1, refused at −1 and at −denorm_min (the edge). The header and guide 19 say which two numbers have
  a meaningful 0. CF1, CF2 below.
- **D7 (minor) — `raisedMinIn`, a field belief beyond HA-192..203. FOR THE COORDINATOR (unchanged: rule HA number
  or not before the merge).** The docs now agree on its SOURCE: the header and guide 19 say it is read from the
  game manual's field drawings (a fact of the field, not of the bench); guide 19 says the library's tests use 5.0
  against their own 5.8-in Goal, a stand-in, not the manual's figure; the changelog the same. No manual figure is
  cited because none was read for it — that read is the coordinator's ruling's to ask for.
- **D8 (minor) — public docs promised A2-F1's fix "is planned". REAL; FIXED in the public docs** — guide 14, guide 19,
  the changelog and the finder header now say the arithmetic "is not fixed yet" (no program uses the finder before
  drive-to-piece) and promise nothing. FOR THE COORDINATOR at the merge: an A2-F1 item (the finder's off-axis
  bearing and floor range, MA17) in REMAINING-WORK and build-order, ahead of L4 — the shared plan docs are not this
  run's to edit.
- **D9 (minor) — no re-check of the brief's citations at the base; unlogged deltas. REAL; done below.**
- **D10 (minor) — HA-192's and HA-198's Source lines; HA-198's claim. REAL; FIXED.** HA-192's Source gains the
  tracker (freshness is the read's time; the frozen check; the bearing paired with the pose at the image), the
  finder (the pose at the image's time) and the feed (the period's floor). HA-198 reworded: nothing depends on the
  ORDER; the score's raw scale is used only by the team's `minScore`, chosen at Step 0.2 from the scores the camera
  reports (0 takes every score); its Source gains the finder.

### D9 — the brief's citations for A2, re-checked at the base `9d77c9d` (`git show 9d77c9d:<file>`), and the deltas
- §1.14 `hal/pros/clock.hpp` :5-16 binds `pros::micros()` — TRUE at the base (the BINDS / CONVERTS block).
- §3.1 `tools/api_doc_tool.py` :1479-1491 — TRUE (`render_nav`: a page missing from the nav passes the strict build
  with an INFO line). ~:95-104 (A1's `hal/fake/` and `sim/` exclusion) — TRUE.
- §3.2 `build-order.md` "the VIS1/AF1 entry :1641-1659" — MOVED: at the base :1641-1659 is the MECH entry; the VIS1
  and AF1 entry is at :1745 (its status block :1746-1751, "Next: VIS1-A2"). Not edited (shared; the merge's).
- §3.4 `hal/device_registry.hpp` :57's "no adapter yet" — already rewritten by A1 ("the OLDER V5 Vision sensor …
  since VIS1-A1 the AI Vision sensor is AiVision"); HA-70's "no library adapter reads it yet" — MOVED to
  :4249-4255 at the base (the tag camera's mount; A1's amendment stands; not A2's).
- **Shipped differences from the brief's text, not logged before:** the feed's stats type is `CameraFeedStats`
  (brief: `Stats`) and carries `coloursWithheld` (A1's additive `VisionFrame::coloursWithheld`, summed); the finder's
  `minScore` and `viewMarginPx` are UNSET at −1, not 0 (D6 above — the brief's "UNSET refuses" now holds); a sixth
  reason the brief does not list, `PieceWhy::NoTarget` (C5); `ToggleView::range` is a bound over EVERY camera of the
  box with an infinite far end allowed (C3); the hold is the same BAND's (C4); ON THE FLOOR has two more conditions,
  the width and the lying exclusion (C2), beside A2-D7's raise exclusion; the R7 test now prints its coverage, as
  §2.2 asked (D2).

### The mutation record of this round (every verdict on a FRESH binary in a COPY of the tree; `scratch/vis1-a2/fix2/`)
Driver `fix1/mt/mutate.py` (and its copy `fix2/mt2/mutate.py`): each anchor found exactly once, the line count kept,
the file backed up, the mutant written and its mtime bumped, the suite binary removed and the briefing regenerated,
the host build (doc gates first), EVERY object depending on the file checked as recompiled, the WHOLE suite through
`suite_run.py` under the 4 GiB / 600 s cap, the failing cases named, the file restored byte-for-byte (finally, and
on SIGTERM/SIGHUP/SIGINT) and re-stamped; a final clean rebuild. Two copies ran side by side (`fix1/mt/tree` the
tracker's, `fix2/mt2/tree` the finder's), each synced to this worktree and checked green before every run; a
stale-object trap found and fixed on the way (below). Records: `fix2/run_tracker.txt`, `run_finder.txt`,
`rerun_*.txt`, `rerun2_*.txt`; the copies' own `mutations.log`.
- **The stopped run (04:38–04:41, its copy):** RT2, RT3, RT4 — RED (1 of 2,937).
- **Main run (07:52–08:42; 2,937 cases):** tracker, 36 — RED 29: RT8 RT9 RT11 RT14 RT15 RT17 RT18 RT20 RT22 RT24 X3
  X4 X5 X7 X9 X11b X12 X13 X14 X15 X18 X19 X24 RECHK-MG-A3 NR4 SB1 SB2 SB3 FR1; SURVIVED 6: RT16 X6 NR2 NR3 SB4 SB5;
  BUILD-BROKEN 1: NR1 (`-Werror` unused `inf` — re-cut, never counted). Finder, 41 — RED 25: RF1 RF2 RF3 RF4 RF6 RF9
  RF10 RF11 RF17 RF20 RF22 RF25 RF30 RF36 RF37 RF38 X33 W1 W2 L1 L3 NT1 NT2 CF1 CF2; SURVIVED 16: RF5 RF12 RF13 RF14
  RF15 RF16 RF18 RF19 RF34 X35 X36 X37 X38 W3 L2 L4.
- **After the new tests: every survivor below re-run on a fresh binary (`rerun*_*.txt`).** Each survivor judged; the ones that could make an answer WRONG or cost real coverage got a test that lands on
  them, found by a SEARCH, not by guessing:** `fix2/sweep/` — `sweep.cpp` (standing Pins on the floor and on 2 /
  3.5 / 5.8 / 8-in supports, seven mounts, the tight camera and eight true cameras at the box's edges, 8–60 in,
  ±30°, jitter 0 and 1: 339,146 sightings), `sweepL.cpp` (lying Pins at twelve headings: 355,015), `boxes.cpp` /
  `boxesR.cpp` (every narrow hand-made box on a grid through six camera models), `rangeprobe*.cpp` (the view's
  range for lenses below, inside and just under the face), each compiled against the clean header and against a
  mutant COPY (`inc_<name>/`, built from the same table entry) and DIFFED. The clean code on the way: 0 fixes
  missing the Pin among the ~203,000 standing sightings within the field belief (floor, 5.8, 8 in) — every miss
  of the clean code is a Pin on a 2- or 3.5-in support, below `raisedMinIn` (5), the documented cost of a lowest
  raise set above the field's own; 0 ON-FLOOR and 0 misses among the 355,015 lying sightings.
  - **NR1 (re-cut NR1r: the far end a hull of the corners that reach the plane)** — survived the 2,941-case run:
    no swept lens needed the infinity. `rangeprobe2` (lenses 10–12.4 in, pitch −5..10, 30–130 in): clean 0 of
    1,559 views missed, NR1r 107. The range test now also sweeps 11.5- and 12-in lenses pitched 2 and 10 out to
    120 in. **RED** (1 of 2,942).
  - **NR2 (the edges always taken together)** — `rangeprobe` with lenses INSIDE the face's band: clean 0 of
    1,698, NR2 42 (14 in, ±10°…). The range test now sweeps 13- and 14-in lenses and pitch −10. **RED** (1 of 2,941).
  - **NR3 (the near end from the explaining corners only) — SURVIVES, judged unreachable on every swept scene:**
    it drops from the near end's minimum the corners whose own edge agreement is empty; 0 misses on all 3,257
    views of both range probes. The hull over every corner is kept as the safer bound; no test claims it.
  - **SB4 (place not compared)** — new test: with the gate on, a read after the lens moved far (its travel widens
    direction and range to anything) whose band names ANOTHER Toggle → "not held yet", `toggleIndex == 1`. **RED.**
  - **SB5 (the lens's travel allowance dropped)** — new test: a camera measured to 0.01 in strafing past a Toggle
    2 in a read, the reads' bearing intervals `REQUIRE`d apart: it holds and becomes sure. (A first version
    driving straight at it did not separate the reads — the range intervals are 2–9 in wide — and was rewritten;
    its `REQUIRE(apart)` caught that.) **RED.**
  - **RF5 (the "error larger than the tolerance" edge inclusive)** — the REASON's edge: new check, the tolerance
    exactly the fix's radius (0.01 in off its centre) → "error larger than the tolerance", one double more → "on
    the border". **RED.**
  - **RF13 (ON THE FLOOR without `lo <= hi`)** — sweep: a Pin on a 2-in support, 44 in from a tight 10-in level
    lens, its size range (37.6–49.3) ending before its ground range (50.3–56.5): clean UNSURE, mutant ON-FLOOR with
    no fix. New check, the disjointness `REQUIRE`d. **RED.**
  - **RF16 (the RAISED fix's near end 1.5 radii)** — sweep: 824 sightings where the mutant's fix misses and the
    clean one holds; in-contract: a Pin on a 5.8-in Goal 8 in from a tight 8-in level lens (0.2 in inside a
    0.39-in fix). **RED.**
  - **RF34 (the top-face allowance from the highest lens)** — sweep: 540 UNSURE→ON-FLOOR, every one a Pin on a 2-
    or 3.5-in support; pinned: a 3.5-in support 45 in out through a true camera at the box's edge → UNSURE (the
    mutant: ON-FLOOR, its fix 12 in past the Pin). **RED.**
  - **X38 (the lying fix's near end + a radius)** — unsafe: the lowest ray of a lying Pin meets the floor NEARER
    than the line it rests on (the tangent argument: the floor contact is under the axis, the silhouette's lowest
    ray grazes the near side and lands short of it). Sweep: 1,313 misses; pinned: a Pin lying across the view 10 in
    from a 6-in lens at −24° keeps its centre in its fix. **RED.**
  - **W3 (the width check without its jitter)** — conservative (refuses more), but it COSTS coverage: 33 in-contract
    standing Pins ON-FLOOR→UNSURE. Pinned as coverage: a 6-px-wide Pin 52 in from a tight 6-in lens still stands. **RED.**
  - **L2 (the lying reach without the diameter, 0.19 in) and L4 (the bottom row not jittered)** — no plant scene
    of the 694,161 changed; the hand-made box search found the boxes each margin alone decides (146,92 3–4×6
    through the tight 10/−10 camera; 2,98 3–7×9 through the stated tolerances). New test: never ON THE FLOOR.
    **RED** (both, 1 of 2,942).
  - **RT16 (an invalid bearing counts as elsewhere) — EQUIVALENT** (argued with T3 above: no input the contract
    allows makes the bearing invalid).
  - **X6 (a frozen view keeps `heldSeconds`) — EQUIVALENT:** `update()` resets `last_` to `ToggleView{}` on every
    read and never writes `heldSeconds` into it; `view()` copies `last_`, so the field is already 0 there — the
    explicit `v.heldSeconds = 0.0` restates it. (The frozen test's `heldSeconds == 0` holds either way.)
  - **Conservative, judged and kept (each refuses more or widens a bound; none can make an answer wrong):** RF12,
    X36, X37 (the ON-FLOOR fix's near end without the size range / without the radius, its far end without the size
    range — each a WIDER interval, still a bound); RF18, X35 (the agreement's two slacks dropped — fewer agree:
    more UNSURE); RF19 (the FOUND hold's sameness without the held radius — stricter: the hold restarts more);
    RF15 (the RAISED test's slack dropped — the sweep: 2,250 sightings change, EVERY one a raised Pin going
    UNSURE→RAISED, correctly, and no floor Pin read RAISED: the slack costs coverage, it buys margin against box
    edges beyond the model).
  - **RF14 (the RAISED test's nearest ground without the principal column)** — no plant scene changed (the column
    matters only for a box spanning it, by a factor up to 1/cos(4/212 rad) ≈ 1.0002); the RAISED box search found
    the two boxes it alone decides (146,98 27×87 and 29×87 through the stated tolerances, centred on the column,
    `REQUIRE`d). New test: never RAISED. **RED** (1 of 2,943).
- **The stale-object trap, logged:** the finder's copy was made from the tracker's at 07:50 and built at ~07:52;
  a test edited in this worktree at 07:51 and then `rsync -a`'d over carried its 07:51 mtime — OLDER than the
  object — so the copy's first link was stale (2,960,622 assertions where 2,960,626 were due). Caught by the
  assertion count before any mutation ran; every vision header and test in the copy touched and relinked (10
  objects), green at 2,960,626, and from then on every sync TOUCHED the vision files before the build.

### The prose review of this round (pace rule 4 — each grep RUN and its hits READ before this was written)
- **What this round's writing touched:** HA-192 (Source), HA-198 (title, claim, Source), HA-199 (claim, Source,
  Settle, blast radius, index row), HA-202 (title, claim, Source, Settle, index row); `toggle_tracker.hpp`'s R7
  sentence and gate 5's "what this rests on" (option (a)'s leak added); guide 19's gate-5 paragraph (the same);
  the changelog (the count, `raisedMinIn`'s source, the overlong line wrapped); and the stopped run's edits, read
  here in full before being kept: guide 19 (the feed's 25 reads, the UNSET −1s, the same-band hold, gate 5, the R7
  table, the lookalikes, the finder's posture and its limits), guide 14 (four bullets), the changelog's A2 entry,
  `piece_finder.hpp`'s and `toggle_tracker.hpp`'s headers, `camera_geometry.hpp`'s `kBoxEdgePx`, `robot.hpp`:140.
  Each was read as a student would: every number against the run that produced it (R7's table against the test's
  printout line by line; "one parking in ten" against 70 of 600; "5,876 of 169,542" against the review's `p8b`,
  re-run here at 0; "1.8 in" against `p7_standmiss` at 2 px, re-run here: 177 misses, worst 1.775 in; "22 to 30 in"
  against `gateprobe`, re-run here), every "never" against a test that would see it fail.
- `grep -rn -i "vision\|camera\|AbsentVision\|DeviceKind::Vision\|VISION" docs/ README.md` (public, outside
  `docs/api` and `docs/internal`): 532 hits; read where A2's review changes could make one stale (guide 19, 14,
  10, the changelog, the register) — none stale beyond those edited above.
- The §3.4 internal statements and this round's own (`no adapter yet`, `TAGS AND VISION ARE DELIBERATELY ABSENT`,
  `vision ABSENT`, `COLORS left as found`, `interprets nothing`, `no library adapter reads it yet`, `camera's next
  part`, `is planned`) over `docs`, `README.md`, `include`, `src`: hits — HA-70 :4291 (the TAG camera's mount; A1's
  amendment stands), the changelog :192 (A1's history), `robot.hpp` :786 ("Until VIS1-A1 the line ended 'vision
  ABSENT'" — history, true), `bench_r3a.cpp` :2634 and `drive_program.cpp` :225 (VIS1-B's `src/`, not this run's),
  `fake_file_store.hpp` ("planned fault" — another meaning). **Nothing stale from A2; "the camera's next part" and
  "is planned" have no hit left.**
- **Finding D5's point — the grep over `include/shulib` too:** `grep -rn -i "camera's next\|part A2\|no
  consumer\|reads it yet\|is planned" docs/*.md docs/guide docs/cookbook README.md include/shulib`: every
  "no consumer" hit (`hal/vision.hpp` :61, `absent_vision.hpp` :12/:35, `robot_config.hpp` :334, `robot.hpp` :68)
  is about the FROZEN `IVision`, which D11 keeps unconsumed — true; every "part A2" hit is CAL1's or H1's.
- Roadmap :438 (the coordinator's "Now": "next for the camera is VIS1-A2") — stale at the merge, left for it
  (unchanged since the build's log).

### FOR THE COORDINATOR — from this round
1. **`raisedMinIn` (A2-D7, review D7):** an HA number or not, before the merge; the docs now say it is the
   manual's field fact, but no manual figure for the lowest raise is cited — the ruling may ask for that read.
2. **A2-F1 (MA17, the finder's off-axis bearing and floor range):** add its item to REMAINING-WORK and
   build-order, ahead of L4 (review D8); the public docs promise nothing.
3. **C1's choice is (b), measured:** AF1-S acts on the colour, so the dependency (the field face's descriptor
   fires on every read its bottom face's does) is a bench step BEFORE AF1 trusts the colour: HA-199's new Settle.
   Option (a) was measured and does not remove the risk (16 of 600 still wrong) while costing most of the
   coverage.
4. **NR3 survives,** judged unreachable on every swept scene (3,257 range views); no test claims it.
5. Unchanged from the build: the src gate self-test's SIGTERM handler; the self-test never beside a host build;
   the glossary; the shared plan docs' status lines; `robot.hpp` (shared with L6) carries one comment phrase.

### The gates the stopped run ran on its final tree (`date` 09:12–09:50; `scratch/vis1-a2/fix2/gates.sh`, outputs `fix2/gates_*.txt`) — written by the resumption below FROM those saved outputs, because the stop came before this section was appended
- Relink: `rm -f build/test/shulib_tests`, `api_doc_tool.py generate`, the briefing regenerated (binary absent), the
  build (5 objects, rc 0, no warning), the briefing regenerated (binary present). `suite_run.py green` capped:
  **green — 2,943 cases, 2,960,683 assertions**.
- Doc gates AFTER the link (T7's lesson): `api_doc_tool.py` self-test OK; check-coverage 5,427 public entities across
  196 headers, all documented; check-fresh rc 0; check-examples 654 quoted lines all verbatim (11 sources);
  check-removability clean; `doc_staleness_audit.py` clean (32 live docs, 0 numeric claims checked) — run with the
  fresh binary present, its output saved (`fix2/gates_staleness.txt`); `briefing_status.py check` current.
- `ctest -R compile_refusal`: **255 of 255**. ARM header gate over **238** headers (`-Werror`, cortex-a9): rc 0, no
  output. CI greps: empty.
- `src_build_gate.py self-test` **OK (269 detector cases)**, `check` **PASS (all 7 builds link)**, run ALONE;
  `git diff --stat -- src/ Makefile tools/` 0 lines after.
- After the self-test's re-stamps: rebuilt (257 objects, rc 0, no warning), the briefing regenerated, the suite
  **green — 2,943 cases, 2,960,683 assertions**; briefing check current; staleness audit clean (saved,
  `fix2/gates_staleness2.txt`).



### RESUMED (this fix round's SECOND stop: the 5-hour usage limit; work resumed ~13:00, this run started) — `date` Wed Nov 11 10:37:37 PM EST 2026
- **What the stopped run left, audited first (not from memory):** `git diff -- src/ Makefile` EMPTY and `git diff
  --stat -- src/ Makefile tools/` EMPTY (no src-gate plant). Its mutation drivers worked only in COPIES of the tree
  (`fix1/mt/tree`, `fix2/mt2/tree`); this worktree's vision headers are byte-identical to the latest pre-mutation
  backups each driver took (`toggle_tracker.hpp` md5 `2e5fc07f` = the NR1r and SB5 backups; `piece_finder.hpp`
  `e59cb708` = the RF14, L2 and L4 backups; `camera_geometry.hpp` `c4a980c2` = the RECHK-MG-A3 backup) and to both
  copies' restored files (`diff -rq` over `include/` and `test/`: equal, but `fix1/mt/tree`'s older
  `piece_finder_test.cpp`, synced later to `fix2/mt2/tree`, which is equal). Each copy's log ends "final clean
  rebuild rc 0". **No mutant and no plant anywhere in the worktree.** No process of this worktree alive (the only
  build running on the machine is the coordinator's, in the main tree — not touched).
- **Where it stopped:** after its gates (09:12–09:50, all green — the section just above, written now from the saved
  `fix2/gates_*.txt`) and BEFORE it appended anything to this log: the dispositions, the D9 block, the mutation
  record, the prose review and the coordinator's list were drafts in `scratch/vis1-a2/fix2/log_part{1,2,3}.md`.
  Every hand-written file of the tree was last written by 09:12 (the briefing, `docs/api` and `mkdocs.yml` are
  regenerated by every gate run). This run appended those drafts
  verbatim (above), each claim first checked against the tree, and wrote no new fix.
- **Checked against the tree before appending:** every disposition's code (`PieceWhy::NoTarget`, `minScore` and
  `viewMarginPx` at −1, the same-band hold, the range bound, the width and lying exclusions), every named test
  (the missed field face; the RANGE bound; the VETO mid-hold; gate 5's edges; the position gate at EACH place; the
  same band by PLACE and WHILE MOVING; LYING near END-ON; the LYING margins; the PRINCIPAL COLUMN), the documents
  (`robot.hpp` :140 names `vision::CameraFeed`; guide 19 says 25 reads; no `SUITE_COUNT` placeholder and no
  "is planned" / "camera's next part" left; the changelog's count 2,815 → 2,943); the mutation logs (every
  verdict the draft gives is the logs' line, the final reruns RED on fresh binaries; the survivors left are
  exactly RT16, X6, NR3 and the finder's seven conservative ones the draft judges); and guide 19's R7 table
  against `r7probe` re-run here, line for line.
- **The reviewers' probes, re-run on the FINAL tree with fresh probe binaries** (`fix2/pb.sh`, capped; output
  `scratch/vis1-a2/fix3/probes.txt`, 22:38–22:40): `p14_notarget` "UNKNOWN / no target"; `p11_range` "11538 sure
  views; the true range outside the view's interval: 0"; `p1_noface` "cases 6240, a colour (wrong …) 658" (the
  stated dependency, C1 (b), unchanged by design); `p5_endon` "128 … 0 … ON THE FLOOR; 0 phantom FOUNDs";
  `p9b_lying_phantom` "159744 …; 0 read STANDING ON THE FLOOR; 0 phantom FOUNDs"; `p8b 0 1` and `p8b 0 0 0`
  MISSED 0 and onfloor 0 at all five mounts; `gateprobe` T10 RED, `checked=1 idx=0` at 22–30 in, why=5 at ≤ 20
  and ≥ 34, why=9 at 32; `r7probe` as guide 19's table; `p7_standmiss` at 2-px jitter "6/0 … MISSED 177 (worst
  +1.775 in)", "10/−10 … MISSED 94 (worst +0.808 in)" (C6's documented zero margin).
- **Two corrections to this section's own record:** the "Started" line says "24 after dedupe"; the findings
  dispositioned are **23** (C1–C6, T1–T7, D1–D10), every one of the coordinator's list. And the coordinator puts
  the second stop at ~09:40; the saved gate outputs run to 09:50:03, so the stop came after the gates, before the
  log was written.
- **(Appended by the fourth resumption below:** the note above was written by that run at 22:41 to
  `scratch/vis1-a2/fix3/log_resume.md` and never appended — the stop came first. Copied here verbatim. Its gate run
  (`fix3/gates.sh`, outputs `fix3/gates_*.txt`, 22:41–23:18) got through steps 1–6 green — clean relink 257 objects,
  no warning; **suite green, 2,943 cases, 2,960,683 assertions**; every doc gate clean with the binary present;
  255 of 255 compile refusals; the ARM header gate over 238 headers rc 0; the CI greps empty — and the src gate
  self-test **OK (269 detector cases)**; it was stopped DURING `src_build_gate.py check` (`gates_srccheck.txt`
  empty), so steps 7b–8 never ran. Not counted as this round's final gates.)

### RESUMED (fourth stop: the team lead's pause at ~14:00; resumed by the builder) — `date` Thu Nov 12 10:20:00 PM EST 2026
- **Audited first, not from memory:** `git diff -- src/ Makefile tools/` EMPTY and `git status --ignored` shows
  nothing planted under `src/`, `include/`, `tools/` or `Makefile` (the src files newer than 22:41 yesterday are
  the stopped `check`'s and the self-test's re-stamps — mtimes only, contents equal to HEAD). The one
  `src_build_gate.py check` alive on the machine is ROB1-A's, in `.claude/worktrees/rob1-a` (its `/proc/…/cwd`),
  not this tree's. Every mutation backup the two drivers took (`fix1/mt/bak/`, `fix2/mt2/bak/`, 81 files) has one
  of five md5s; the LATEST of each header's equals this worktree's file (`toggle_tracker.hpp` `2e5fc07f`,
  `piece_finder.hpp` `e59cb708`, `camera_geometry.hpp` `c4a980c2`); the older two are earlier copies of the tracker
  that differ from it in COMMENTS only (`25913259`, the main run's: gate 5's "what this rests on" note, extended
  after; `d1ee62c3`, RT2–RT4's at 04:38: with comments stripped the `diff` is blank lines only). Both mutation copies' `include/` equal this
  worktree's (`diff -rq`: empty), so no mutant is left in a copy either; both drivers' logs end on a RED re-run
  (NR1r 09:00, RF14 09:11). **No mutant and no plant anywhere.**
- **The tree is the one every logged verdict was taken on:** no hand-written file changed since 09:13 yesterday
  (`find -newer fix2/log_part1.md` over every changed and new path: only the regenerated `docs/api/*`,
  `mkdocs.yml`, the briefing, and this log). So the third resumption's probe re-runs on fresh probe binaries
  (`fix3/probes.txt`) and the drivers' fresh-binary verdicts stand for THIS code; nothing is re-run for them.
- **Every finding is dispositioned** (the 23 above, C1–C6, T1–T7, D1–D10); this run writes no new fix. It relinks
  from source and re-runs EVERY gate on the final tree (below).

### The gates on the final tree (this resumption; `date` Thu Nov 12 10:20:33 – 10:56:52 PM EST 2026; `scratch/vis1-a2/fix4/gates.sh`, outputs `fix4/gates_*.txt`)
- **Relinked from source:** `rm -f build/test/shulib_tests`, `api_doc_tool.py generate`, the briefing regenerated
  (binary absent), configure, a CLEAN host build (`--target clean`): **257 objects, rc 0, no warning**; the
  briefing regenerated with the binary present. `suite_run.py green` under the 4 GiB / 600 s cap: **green —
  2,943 cases, 2,960,683 assertions** (the count the round's last added test, RF14's, brought it to; the stopped runs ended on the same).
- **Doc gates, AFTER the link:** `api_doc_tool.py` self-test OK; check-coverage 5,427 public entities across 196
  headers, all documented; check-fresh rc 0; check-examples 654 quoted lines all verbatim (11 sources);
  check-removability clean; `doc_staleness_audit.py` clean (32 live docs, 0 numeric claims checked — no "NOT
  derived" note: the binary was present; `fix4/gates_staleness.txt`); `briefing_status.py check` current.
- `ctest -R compile_refusal`: **255 of 255**. ARM header gate over **238** headers (`-Werror`, cortex-a9): rc 0, no
  output. CI greps: empty.
- `src_build_gate.py self-test` **OK (269 detector cases)**, `check` **PASS (all 7 builds link)**, run alone in this
  tree (the other `src_build_gate` processes on the machine were the main tree's, ROB1-A's and STEP1-B's, each in
  its own directory); `git diff --stat -- src/ Makefile tools/` **0 lines** after.
- After the self-test's re-stamps: the binary removed, the briefing regenerated, rebuilt (**257 objects, rc 0, no
  warning**), the briefing regenerated, the suite **green — 2,943 cases, 2,960,683 assertions**; briefing check
  current; staleness audit clean (`fix4/gates_staleness2.txt`).
- After appending this section: `briefing_status.py check`, `api_doc_tool.py check-fresh` and
  `doc_staleness_audit.py` re-run: current, rc 0, clean; `src/ Makefile tools/` diff still 0 lines (`fix4/gates_after_log.txt`).

### The tree at the end of this resumption — `date` Thu Nov 12 10:57:10 PM EST 2026
- Every one of the 23 findings dispositioned above (C1–C6, T1–T7, D1–D10); no new fix this resumption. Not
  committed; no `pros`; no `/dev/ttyACM*`; no mutant, no plant, no stale binary (the suite binary on disk is the
  step-8 link of the final tree). Guide 19's file name left as `19-the-camera.md` (the coordinator renumbers it at
  the merge: chapter 19 is CAL1's on main). The coordinator's open items are the list "FOR THE COORDINATOR — from
  this round" above, unchanged.

### Verification-round findings and their disposition — `date` Thu Nov 12 11:30 PM – Fri Nov 13 12:25 AM EST 2026
The fresh verifier returned two findings, both minor. Each re-checked here against the tree before any edit;
records under `scratch/vis1-a2/fix5/`.
- **V-1 — HA-198's index row still said "nothing in VIS1 depends on either" (the order AND the score's scale).**
  REAL: the round's earlier fix reworded the entry's title, claim and Source (nothing depends on the ORDER; the
  raw scale is used only through the team's `PieceFinderConfig::minScore`, set at Step 0.2) and left the index
  row at `docs/hardware-assumptions.md`:414 contradicting it; that round's prose review listed HA-198's edits as
  "title, claim, Source" and missed the row. A grep for the same stale claim found ONE more, which the verifier
  did not name: the register's VIS1-A1 summary at :189–190 ("the detections' order and score (HA-198, reasoned —
  nothing depends on them)"), a present-tense claim in the live register. **Fixed:** the row now says nothing
  depends on the order or on a count above 24, and that the scale is used in one place only, the finder's
  `minScore`, a TEAM number on the raw scale chosen at Step 0.2 — the entry's own words; the summary now says
  "nothing depends on the order; the score's scale only through VIS1-A2's team-set floor, `minScore`" (re-wrapped
  to the paragraph's width). The entry itself (:1325–1343) was already right and is unchanged.
- **V-2 — RF15 recorded as a SURVIVING "conservative" mutant; on the final tree it is RED, and "conservative"
  has its direction backwards.** REAL on both counts, and the re-check found the same error in two more verdicts
  and the same wrong reasoning behind a third.
  - **RF15 re-run on a fresh binary** (`fix5/rf15.sh`; restores on EXIT/TERM/HUP/INT; 23:32): 5 objects recompiled
    — every object that includes `piece_finder.hpp` (the four camera test files through `camera_scene_rig.hpp`
    and `vision_guide_example_test.cpp`); the suite **RED — 1 case, 2 assertions**: the PRINCIPAL COLUMN test
    (`piece_finder_test.cpp`:1083, `CHECK( 2 != 2 )` for w = 27 and 29). The main run's SURVIVED (`fix2/
    run_finder.txt`:12) was taken at 2,937 cases, before RF14's PRINCIPAL COLUMN test was added; RF15 was never
    re-run after it. And the slack is the RAISED verdict's MARGIN: dropping it lets more boxes read RAISED —
    the less cautious side. The "conservative" judgement above read the sweep's 2,250 UNSURE→RAISED changes as
    harmless because every one was a raised Pin; the hand-made boxes RF14's search found (consistent with a
    floor Pin through the stated tolerances) are read RAISED by it — a wrong answer the sweep never reached.
  - **Every logged survivor then re-run on the FINAL tree, fresh binary each** (`fix5/surv.py`, the fix2 tables'
    exact edits, restore in `finally` and on SIGTERM/SIGHUP/SIGINT, every vision header's md5 compared before and
    after: equal; 23:34–23:43, 5 objects each, `fix5/surv_run.txt`): RT16, X6, NR3, RF12, RF18, X35, RF19
    SURVIVED (2,943 cases); **X36 and X37 RED** (1 case, 2 assertions each) and RF15 RED. X36/X37's SURVIVED
    verdicts were also taken at 2,937 cases; what catches them now (`fix5/which.py`, `which_X36.txt`,
    `which_X37.txt`) is RF13's test, "the fix's near ends LANDED", its ON THE FLOOR block
    (`piece_finder_test.cpp`:1016–1017: a Pin on a 2-in support, 44 in from a 10-in lens — the two rim bounds
    disjoint, so UNSURE; X36/X37 widen the ON-FLOOR interval until it is not empty and read that Pin ON THE
    FLOOR, its fix missing it). So "each a WIDER interval, still a bound" was wrong reasoning: the interval
    also DECIDES the posture (`lo <= hi`), and widening it turns a refusal into a floor answer.
  - **RF12, by the same reasoning, was not conservative either — and no test landed on it.** RF12 drops the size
    range from the ON-FLOOR interval's near end. Searched, not guessed: `fix5/rf12/boxes12.cpp` (every box on a
    grid, w ≤ h, through eight camera models, capped at 1,500 s — models 0–2 compared in full, 3 in part, 4–7 not
    reached) against the clean header, RF12 and X36: through the stated tolerances (model 0) no box changes;
    through cameras measured to 0.01 in, RF12 reads **10,377** boxes (10-in lens, −10°) and **554** (18-in, −20°)
    ON THE FLOOR that the clean code reads UNSURE, none the other way. Every one probed (`fix5/rf12/probe.cpp`)
    is the MIRROR of RF13's case: the size range BEGINS after the ground range ends (e.g. 110,108 10×24 through
    the tight 10/−10 camera: ground [40.613, 44.781], size [44.954, 63.328]) — a box too short for a Pin standing
    where its bottom row is, which the clean code refuses and RF12 places by the ground range alone. The plant
    sweeps re-run for RF12 and X36 against a fresh clean build (`fix5/sweep/`, the fix2 sources unchanged): lying
    Pins — no line differs for either; standing Pins — RF12 changes 15,553 of 339,146 sightings, EVERY one an
    ON-FLOOR fix that still holds made wider (e.g. radius 1.562 → 1.864: coverage lost to the tolerance check,
    never a wrong posture); X36 changes 75,626, among them 54 UNSURE → ON-FLOOR-with-a-missing-fix (Pins on a
    2-in support — what RF13's test pins). **New test** (`piece_finder_test.cpp`, after the PRINCIPAL COLUMN
    one): "ON THE FLOOR only when the rim's two bounds INTERSECT -- the MIRRORED side" — three of the found boxes
    (110,108 10×24 and 11×24 through `tightModel(10, −10)`; 110,100 12×18 through `tightModel(18, −20)`), each
    `REQUIRE`d unclipped with `sizeRange.lo > groundRange.hi`, then UNSURE with no fix. Clean: green (18
    assertions). **RF12 on a fresh binary (00:20): RED — 1 case, 6 assertions**, the new test's :1109–1110.
  - **The PRINCIPAL COLUMN test's own title corrected:** it said its two boxes are ones "that only that column
    keeps from RAISED"; RF15 shows the RAISED slack keeps them from it too. Now: "that the column keeps from
    RAISED (and the RAISED test's slack too: drop either and they read RAISED)". The test's body is unchanged.
  - **The re-check of the three left — reasoned, then searched:** RF18 and X35 drop one of the per-camera
    agreement's slacks (`piece_finder.hpp`:444), so fewer cameras agree; a box that no longer agrees cannot pass
    the RAISED test instead (for RF18 its size range lies beyond every agreeing camera's ground range, so the
    rim's far end is past the nearest ground; for X35 the camera that agreed did so within the slack, which the
    RAISED test's own slack then rules out), so it reads UNSURE. Searched (`fix5/rf12/boxesP.cpp`: every box on
    the same grid through models 0–2, COMPLETE, every non-UNSURE posture printed; `cmpP.py` merges clean against
    each mutant): RF18 changes **25,487** boxes and X35 **50,884**, EVERY one ON THE FLOOR → UNSURE, none to
    RAISED, none from UNSURE. RF19 (the FOUND hold's sameness without the held radius): the hold restarts more
    often; it never reports a different piece as held. These three stay judged conservative.
  - **The corrected record (this replaces the main run's and the third resumption's survivor lists for the
    final tree):** tracker — RT16 and X6 EQUIVALENT (argued above), NR3 judged unreachable (3,257 range views),
    all three re-run SURVIVED on the final tree; finder — **three** conservative survivors, RF18, X35, RF19
    (re-run SURVIVED at 2,944 cases, 00:20–00:22); RF12, RF15, X36, X37 are **RED** on the final tree, and
    none of the four was ever conservative (each turns a refusal into an answer). The earlier sentences at
    "Conservative, judged and kept" and "the survivors left are exactly RT16, X6, NR3 and the finder's seven
    conservative ones" stand as what was written then; this entry corrects them.
- **Suite count:** 2,943 → **2,944** cases (2,960,683 → 2,960,701 assertions); the changelog's A2 entry
  (`docs/changelog.md`:70) now says 2,815 → 2,944. The briefing's line is regenerated by the gates.

### The prose review of the verification round (pace rule 4 — each grep RUN and its hits READ before this was written)
- **What this round's writing touched:** `docs/hardware-assumptions.md` HA-198's index row (:415) and the VIS1-A1
  summary (:189–191); `test/piece_finder_test.cpp` the PRINCIPAL COLUMN test's title and the new MIRRORED-side
  test (its title and its one comment); `docs/changelog.md`:70 (the count); this log. Each read as a student
  would, every number against the run that produced it (10,377 / 554 against `comm` over `fix5/rf12/out_*.txt`
  per model; 15,553 / 75,626 / 54 against `fix5/sweep/runS.txt` and the classified diff; the boxes' ranges
  against `probe.cpp`'s output; 6 assertions against `which_RF12.txt`).
- `grep -rn -i "nothing in VIS1 depends\|nothing depends on them\|depends on either"` over `docs/ include/ test/
  README.md` (outside `docs/api` and the VIS1 brief and log): only HA-198's row and entry, both now saying "the
  order" — none stale. `grep "minScore"` over guide 19, the register and the changelog: guide 19's example
  value (50.0) sits under its "yours come from the camera's bench station" note; none claims a scale.
- `grep -rn -i "conservative"` over guide 19, guide 14, the changelog, the vision headers and the finder tests:
  no public text calls any mutant or bound conservative except `camera_geometry.hpp`:152 (the fix's radius — a
  bound, true) and `toggle_tracker.hpp`:790 (the partial-band veto — refuses more, true); the changelog says only
  "the survivors are named there, each judged" — still true. `grep "2,943\|2943"`: only the briefing (regenerated)
  and this log. Guide 19's posture paragraph (:273–288) re-read: it states the per-camera agreement and the
  RAISED/lying exclusions; the whole-box intersection the new test pins is behaviour unchanged by this round, so
  no guide text changes.
- `grep -rn "only that column"` (the corrected title's old words): no other copy.

### The gates on the final tree (the verification round; `date` Fri Nov 13 12:25:54 – 01:04:29 AM EST 2026; `scratch/vis1-a2/fix5/gates.sh`, outputs `fix5/gates_*.txt`)
- **Relinked from source:** the binary removed, `api_doc_tool.py generate`, the briefing regenerated (binary
  absent), configure, a CLEAN host build: **257 objects, rc 0, no warning**; the briefing regenerated with the
  binary present. `suite_run.py green` under the 4 GiB / 600 s cap: **green — 2,944 cases, 2,960,701 assertions**.
- **Doc gates, AFTER the link:** `api_doc_tool.py` self-test OK; check-coverage 5,427 public entities across 196
  headers, all documented; check-fresh rc 0; check-examples 654 quoted lines all verbatim (11 sources);
  check-removability clean; `doc_staleness_audit.py` clean (32 live docs); `briefing_status.py check` current.
- `ctest -R compile_refusal`: **255 of 255**. ARM header gate over **238** headers (`-Werror`, cortex-a9): rc 0, no
  output. CI greps: empty.
- `src_build_gate.py self-test` **OK (269 detector cases)**, `check` **PASS (all 7 builds link)**, alone in this
  tree; `git diff --stat -- src/ Makefile tools/` **0 lines** after.
- After the self-test's re-stamps: the binary removed, the briefing regenerated, rebuilt (**257 objects, rc 0, no
  warning**), the briefing regenerated, the suite **green — 2,944 cases, 2,960,701 assertions**; briefing check
  current; staleness audit clean. The RF18/X35 box search ran beside the gates (scratch binaries only, compiled
  against the same headers; no worktree file touched).
- After appending this section: `briefing_status.py check`, `api_doc_tool.py check-fresh` and
  `doc_staleness_audit.py` re-run (`fix5/gates_after_log.txt`).
- **The tree at the end of this round:** both findings dispositioned (V-1 fixed, with the second stale copy it
  missed; V-2 corrected, with X36/X37's stale verdicts, RF12's missing test and the PRINCIPAL COLUMN title found
  on the way). Not committed; no `pros`; no `/dev/ttyACM*`; no mutant (every vision header's md5 equal to its
  pre-run copy after each driver), no plant, no stale binary (the suite binary on disk is step 8's link).

## VIS1-A2 — the coordinator's merge verification *(2026-11-13)*

- **Applied as its OWN diff** (`9d77c9d..` its transport) onto `8df58f3` (STEP1-B, API 2.8): every code file applied cleanly
  (`camera_geometry.hpp`, `robot/robot.hpp`, the new `vision/{toggle_tracker,piece_finder,camera_feed}.hpp` and their tests).
  Documents resolved by hand: guide 14 (L6's wall-reset section, then the camera's), the guide index and `mkdocs.yml`'s guide
  nav, the changelog (VIS1-A2's entry at the top of `## API 2.8`; no bump — vision is in no frozen row), and the generated
  pages (regenerated). **The guide chapter is renumbered 19 → 20** (`docs/guide/20-the-camera.md`): CAL1 Part B had taken 19.
  Every live link moved (guides 10 and 14, the changelog, the index, the nav) and the example test's case names
  `guide-19a..e` → `guide-20a..e`. G3a Part 1's unmerged `20-your-robot-file.md` renumbers at its own merge (after the 21st).
  Status words: build-order's VIS1 line, REMAINING-WORK row 18d, a roadmap "Now" paragraph.
- **Gates on the merged tree** (`.claude/coordinator/scratch/vis1_a2_merge_verify.sh`): the suite green (3,543 cases);
  `src_build_gate.py` self-test OK and check PASS (all 8 builds); the ARM header gate CLEAN; the doc gates and the briefing
  check clean; the compile-refusal harness 272 of 272; no plant, no stray root file.
- **The coordinator's own mutations** (`muts_vis1_a2_main.json`): **4 of 4 RED** — MV1 two colours in view held as the first,
  MV2 a camera that moved over an unmoved band never called frozen, MV3 a colour reported before it held for `holdSeconds`,
  MV4 a stale camera still answering. The restored tree relinked and ran green.
- **VIS1-A2 LANDED.** No AI Vision sensor has run this code. AF1S-A1 (built on this code) merges next.
