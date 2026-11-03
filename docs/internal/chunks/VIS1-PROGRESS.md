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
