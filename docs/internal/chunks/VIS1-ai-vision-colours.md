# VIS1 — the AI Vision camera as library code: its COLOURS (the Toggle) and its OBJECTS (Pins, Cups); a camera frame beside the frozen `IVision`; the geometry from a measured mount; the Toggle colour tracker; the piece finder (a piece named by its type and its field position); the host camera model; the Bench Tests camera page; and Step 0 at the robot

> **THE TEAM LEAD'S ANSWERS (2026-10-18)** to "Decisions for the team lead": **"Accept all (Recommended)."**
> - **D1 (a) — DECIDED, as recommended:** three runs, each its own go — VIS1-A1 (the camera as a device) once
>   MECH-A1 merges; VIS1-A2 (what the camera means) when A1's BUILD ends; VIS1-B (robot two and the bench page)
>   after MECH-A5 merges and before MECH-A6 launches (if A6 has launched, after A6 merges).
> - **D2 (a) — DECIDED, as recommended:** the camera's facts live in the robot file as `config::CameraConfig` (a NEW
>   `config/camera_config.hpp`; `robot_config.hpp` gains only its hooks), its port claimed through the one-owner
>   claim; robot two's values are a `.camera` member of its `ChassisTable`.
> - **D3 (a) — DECIDED, as recommended:** Bench Tests station 12 is extended — the colours, the model check's line,
>   one timed read, the library's measurements beside every raw record, and a LIBRARY page; no new station or program.
> - **D4 (a) — DECIDED, as recommended:** `DeviceKind::AiVision` is appended and `E_DEVICE_AIVISION` maps to it (word
>   "AI-VISION"); `Vision` keeps meaning the older sensor.
> - **D5 (a) — DECIDED, as recommended:** a declared camera the brain cannot make never refuses the robot —
>   `robot.camera()` is `nullptr`, `robot.cameraAbsence()` and the boot line say why, and every consumer that needs it
>   refuses to construct.
> - **D6 (a) — DECIDED, as recommended (as the re-check's S4 changed it):** a read reporting **24 or more** is
>   "possibly full" (`VisionFrame::truncated`) and **never evidence** — not of absence, not of "only one colour"; the
>   tracker and the finder say UNKNOWN "camera full".
> - **D7 (a) — DECIDED, as recommended:** the Override AI model is checked BY NAME — each expected class id's name
>   (0..4) read into a zeroed buffer, never a count; until every name matches, no Object is passed on (colours still
>   are), and the check is re-asked once a second.
> - **D8 (a) — DECIDED, as recommended:** the alliance colour picker is its own small chunk, **ALLY1** (~1–1½
>   builder-days, rough, its own brief), placed after MECH-A4 merges and before L4 and AF1; VIS1 builds none of it.
> - **D9 (a) — DECIDED, as recommended (as the re-check's S1 changed it):** a lens **about 10–11 in up and level**,
>   rigid, then measured (Step 0.1) — **never 11¾–12¾ in level** (the blind band; if the build can only put the lens
>   there, pitch it up ~10°).
> - **D10 (a) — DECIDED, as recommended:** the consumer's own loop reads the camera through one `vision::CameraFeed`
>   (`feed.tick()`, at most once per `minReadPeriod`, the same frame to every consumer) — never a library task.
> - **D11 (a) — DECIDED, as recommended:** the frozen `hal::IVision` stays `AbsentVision` in the Robot; every VIS1
>   consumer takes frames, and `hal::pros::ProsAiVision` is the class R2b's tag part later extends.
> - **The parts and sizes** (the final Re-check's — §2.1 and §9): VIS1-A1 (the camera as a device) ~2½–3½, VIS1-A2
>   (what the camera means) ~2¼–2¾, VIS1-B (robot two and the bench page) ~1¾–2¼ — **VIS1 ~6½–8½ builder-days**
>   (rough; verification ~half again on top); ALLY1 separate, ~1–1½.
> - **The go to build:** VIS1-A1 starts now, beside MECH-A3 (D1's condition, MECH-A1 merged, met at `fd36e07`);
>   VIS1-A2 when VIS1-A1's build ends; VIS1-B after MECH-A5 merges and before MECH-A6 launches (D1). Step 0's first
>   four steps need no new code: they run at the next bench session.
> - **Where this brief lives:** `docs/internal/chunks/VIS1-ai-vision-colours.md` (this file), NOT the
>   `VIS1-ai-vision-colours-and-objects.md` name §3.3 gives; every pointer to the brief links this path.

> **Status 2026-10-15: DRAFT for the team lead's go. Nothing is built.** Written by the coordinator's brief writer
> at tree `03a8081` (branch `shulib-v2`; no code change since `bdcbc7c` — `git diff --stat bdcbc7c 03a8081 --
> include test src tools` is empty), with **MECH-A1's working tree** (`.claude/worktrees/mech-a1`, uncommitted on
> `bdcbc7c`, its build done, its review running) laid over it in the probe copy, because VIS1's first run edits five
> of A1's files and starts only after A1 merges (D1, §6). In flight while this was written, NOT edited: MECH-A1
> (`mech-a1`), MECH-A2 (`mech-a2`, built from A1's snapshot `85e0285`), SER1 (`ser1`), CAL1-A1 (`cal1-a1`).
> This file replaces the stub the 2026-10-08 run left, the half-written draft of the 2026-10-15 00:35 run that the
> machine's reboot (00:58) cut off, and the 01:43 draft that the 09:45 reboot cut off after "The design in plain
> words" (this run kept that draft's decisions and plain words, re-checked each claim it could, and wrote §0–§9).
> The earlier probes (`scratch/brief-VIS1/{proposed,probes,out}/`, `v2/`, `v3/`) were read, rebuilt and re-run, not
> trusted: **every probe result below is from `scratch/brief-VIS1/v4/`** — v3's seven probe programs rebuilt from
> source and re-run at 10:00 (every scene line byte-identical to v3's), plus this run's P7 and P8 and the four fixes
> they forced into the prototypes (§0.4: a frozen-camera latch, the finder's NONE from withheld objects, the Toggle's
> face height as one guessed number, the bottom face's sliver read as a far face). `v4/tree` links to `v2/tree`, re-checked byte-equal (`diff -rq`) to `mech-a1`'s `include/
> test/ src/ tools/` at ~09:58 — A1's fix round had not yet changed a file. **Re-check (T10): superseded** — A1's fix
> round then changed `robot_config.hpp` and `mechanism/{kinds,spec,wiring}.hpp` (~10:33; no line cited here moved);
> A1 merged as `fd36e07` (14:49) and SER1 as `2506067` (21:53); every line cited is re-checked at `2506067`.
>
> **The team lead's words this brief holds (BINDING).**
> - 2026-10-08: "so my team wants an autofliper in combinaton with the ai vision sensor so that whenever a team flips
>   the side thingie that our robot will automatically flip it if it is close by and itll be able to tell by color
>   and it should be quick and seamless"; "we will have to plan and implement this all"; "we probbaly should do it
>   before the panel". His answers that evening: **both robots carry a flipper and both are 15"**; the flipper is a
>   **motor arm**; the camera faces **forward**, one distance sensor on **each side** (left and right, straight
>   out); whether an **Optical** sits at the flipper is decided **after a bench test (Step 0)**; VIS1 builds
>   **beside the MECH core** ("sensors after G3" superseded for the camera only); R2b's adapter is pulled forward
>   **scoped to OBJECTS and COLOURS — no AprilTags** (the field's duplicate tag ids are a separate problem).
> - 2026-10-14: "we definitely need distance sensors working and we need ai vision camera, objects" — the camera's
>   **OBJECTS** (Pins, Cups: the VEXcode V5 4.68 Override AI model) are a **first-class VIS1 deliverable** in their
>   own right, for **L4** (drive to a game piece, after VIS1). **L6** (the wall reset with the left/right distance
>   sensors) is a separate chunk beside the MECH core — VIS1 builds none of it.
> - 2026-10-14, piece choice: "like 2 or 3 yellow pins like if it knows to not touch one vs touch another in a
>   route" — VIS1 gives L4 what it needs to take ONE specific piece: its **TYPE** (the model's class) and its
>   **FIELD POSITION** (robot pose + the camera's bearing + a ground-plane range from the measured mount, each with
>   its error), so a route names a piece by **its expected position plus a tolerance**; when no candidate is inside
>   the tolerance, or two are too close to call, it reports **NONE / AMBIGUOUS — never a guess**; pieces **on goals
>   or in stacks** are told apart (elevation, size). The **alliance colour picker** moves next to VIS1 (L4's piece
>   choice needs it, not only AF1) — where it belongs is decision D8.
> - **AF1** (the automatic flip) is a LATER chunk, after MECH-A4: **VIS1 gives AF1 what it needs (§6.5) and builds
>   none of it.** 2026-10-15: "lets just keep going with the original roadmap but lets move faster OF COURSE THOUGH
>   THAT DOESNT MEAN TO SKIP STEPS" — the work is split into three runs that each verify and commit on their own, and
>   **Step 0 needs NO new code** for its first four steps (§2.0): it can run this week.
>
> **Executor model: `opus`. Do not commit. Live log `docs/internal/chunks/VIS1-PROGRESS.md`, created FIRST** (a
> heading per run). Say "host test plant", never "simulator". **No hardware claim anywhere:** nothing here has run on
> a robot, and no AI Vision evidence exists yet — Bench Tests stations 11 and 12 (worksheet Stations H and I) were
> uploaded and never run (`docs/internal/REMAINING-WORK.md` :166-167). **Hardware Assumptions: HA-192..HA-203 ONLY**
> (§4; the register's Held line gains them at the plan commit; a thirteenth belief ⇒ stop and tell the coordinator).
> Every `file:line` below was OPENED (`~:N` = within a few lines; MECH-A1's files marked "(A1)" — since its merge
> these ARE the tip's lines, re-checked at `2506067`; `bench_r3a.cpp` lines are `2506067`'s unless marked `fd36e07`,
> SER1 having moved every line after ~:180 by +4 — re-check T10).
>
> **The manual.** VEX V5RC *Override* Game Manual **v2.0** (2026-09-03), local text
> `.claude/coordinator/scratch/autoflip/v20.txt`. **v2.1** was scheduled for release 2026-10-08, effective today
> (2026-10-15; v20.txt :517). Checked once on 2026-10-15: `content.vexrobotics.com/docs/2026-2027/override/files/
> override-2.1.pdf` returned **HTTP 404**; the manuals page (vexrobotics.com/26-27-manuals) and the VEX Forum's
> manual-updates thread both returned 403 to the fetch; a web search shows the manuals page's table carrying a row
> dated October 8 (effective October 15, "a clarification or minor update") whose version number the snippet cuts off.
> Re-checked ~10:05 the same day: `override-2.1.pdf` still **404**, the manuals page still **403**; a search finds the
> obsolete 0.1.2 / 0.2 / 1.0 / 1.1 copies and no 2.1. v2.0's own schedule names the row (v20.txt ~:518: "October 8,
> 2026 · October 15, 2026 · Version 2.1 · Clarification / minor update").
> **So whether 2.1 is out, and whether it changed any Toggle rule or dimension, is UNVERIFIED.** Nothing in VIS1
> depends on a Toggle RULE (VIS1 reads colours and sizes; the rules are AF1's), but the Toggle's DIMENSIONS (pA8) and
> the Pin/Cup sizes are season data VIS1 carries (`vision/override_elements.hpp`, §1) — **the executor re-checks
> pA8, pA10, pA15 and the Pin/Cup glossary entries against the newest manual before VIS1-A1 copies them**, and says
> which version it used.

## Re-check — amendments (2026-10-15, ~16:20 onward; the brief writer, after two checkers)

Two checkers (lens "tree": 10 findings, T1–T10 below; lens "sensor": 10 findings, S1–S10 — one overlap, S9(c) with
S6 on a colour band's extent, merged into S6) were each re-verified here from source before anything changed.
**Every re-check probe is in `scratch/brief-VIS1/v6/`**: `v6/tree` is a `git checkout-index` of the INDEX of
`shulib-v2` at ~16:20 — HEAD **`fd36e07` (MECH-A1 MERGED, 14:49)** with **SER1's squash STAGED** (uncommitted; it
touches no file VIS1 edits) — so everything this brief marks "(A1)" is now the TIP (`git diff --stat bcb1cdb fd36e07
-- include test src tools` is empty: the merge is A1's final worktree byte for byte). `v6/proposed` started as
`v4/proposed` (`v5/`, an interrupted 11:05 revise, held a byte-identical copy and nothing else new; superseded). The
manual: v2.1 re-checked once more at ~16:25 — `override-2.1.pdf` still **HTTP 404**; the header's paragraph stands.
Each disposition says REAL or NOT, the evidence, what changed (in the prototypes AND in this brief's body), and the
probe that now shows it. The body below is corrected in place; a changed decision says "(re-check)".

**Second session (~21:00 onward, after two interrupted ones):** T1–T5 and T9 (above) were dispositioned at ~16:20-16:45
and their body edits re-checked as landed; the session at 16:44-16:51 had already prototyped S2 (the rolled-Toggle
host model, R1, R4), S3, S4, S7 and S8 in `v6/proposed/` without writing their dispositions — each was re-verified and
re-run here (the 20:32 `p1_adapter.txt` FAILURE in `v6/out/pre-s/` was a stale output: the adapter already said `>=`;
P1 rebuilt passes 20/20). This session dispositions T6, T7, T8, T10 and S1–S10 (S9(c) merged into S6) and **R7, a
major defect it found itself** (a turned robot read the Toggle's bottom face). The tree moved meanwhile: **SER1 is
COMMITTED as `2506067` (21:53)** and MECH-A2 was being merged in the main worktree (~21:55; no VIS1 code file — §6.1);
every SER1-dependent line was re-checked (§6.3, §1.15 item 6, §2.1's VIS1-B row, T10). Every probe and test binary ran
under `ulimit -v 4194304` and `timeout`. Pre-session outputs, prototypes and this brief are kept in `v6/out/pre-s/`.
The manual: `override-2.1.pdf` fetched once more this session — still **HTTP 404**; S10's narrowing rests on v2.0's
glossary (v20.txt :4668), and the executor's re-check of pA8 and the glossary against the newest manual (§1.10)
covers a v2.1 change.

**T1 (blocker) — the model check read a class COUNT that PROS never returns. REAL; fixed at the root.**
- *Re-verified:* extracted `vdml_ai_vision.c.o` from `firmware/libpros.a` (kernel 4.2.2 per `project.pros`) and
  disassembled it myself: `aivision_get_class_name` is `… bl vexDeviceAiVisionClassNameGet; mov r0,r4; bl
  port_mutex_give; movs r0,#1` — the SDK's value is discarded, every id (−1 included) returns `PROS_SUCCESS`;
  `PROS_ERR` (`mvn.w r0,#0x80000000`) only when the binding or the port mutex fails. The C++ wrapper tests `cmp r0,#1`.
  The header's ":375 -1 returns the count" is documentation only. Reproduced the failure: `v6/probes/classname_check.cpp`
  with the checker's binary-faithful shim gave `model=Different "the camera names 1 classes, the robot file expects
  5"` — on the robot every Object would have been withheld forever while every host test passed.
- *Fixed:* the adapter (`v6/proposed/…/hal/pros/ai_vision.hpp`) reads ONLY the expected ids' names, each into a
  zeroed buffer (`PROS_ERR` → `Unreadable`; '' or a wrong name → `Different`, re-asked at 1 Hz) and withholds any
  Object whose id is past the expected table; its header comment cites the disassembly. The shim
  (`v6/proposed/shim/pros/ai_vision.h`) now answers 1 for every id and writes nothing for an id the table lacks, and
  counts id −1 asks. P1.15–P1.17 added; mutation M9 is now "compares only class 0"; M11 (an id past the table passed
  on) and M12 (the buffer not zeroed) new — **13/13 RED** (`v6/out/p1_mutations.v6.txt`). `classname_check` gives
  `Matches` under BOTH shims (the brief's and the checker's binary one).
- *Body changed:* D7 (a) and its probe line; §0.3's class-name bullet (the binary, with the method); §0.5 item 5
  (copy the names from the RECORD lines); §1.2 rule 4; §1.4 (the shim follows the binary where it and the header
  differ); §1.15 item 2 (VIS1-B's `logClassNames` prints ids 0..19 by name and says the 1 is a success code); Step
  0.2 (expect "-> 1" and one class line; all five names from RECORD with one piece of each type in view — the five
  types are named); HA-197 (no count through PROS 4.2.2; Settle via RECORD lines); §2.2's mutation list; landmine 6.


**T4 (minor) — the binary settles HA-195's failure byte and backs HA-196's mask reading. REAL; folded in.**
- *Re-verified* in my own disassembly: `aivision_get_object` does `memset(sp+6,0,16)`, stores 127 to the id and
  type bytes, then `claim_port_try` (a failed claim returns that record); `aivision_get_color`'s failure path loads
  127 into id/R/G/B and 0x7f800000 (+Inf) into both floats; `set_enabled_detection_types` → `EnableSet(dev, bits,
  mask)`, `enable_` → `EnableSet(dev, t, t)`, `disable_` → `EnableSet(dev, 0, t)`; overlay / tag family / white
  balance → `ModeSet`. Every setter ends `movs r0,#1`.
- *Fixed:* the shim's failure records are the binary's (127/127; 127s and +Inf), citing it; P1.18 pins them through
  the adapter (a 127/127 object is skipped → `PartialFault`; a 127/+Inf read-back refuses the set-up). A new shim
  knob, a colour write that "succeeds" and stores nothing (PROS returns 1 regardless), and P1.19 — written because
  the run's new mutation M13 (the colour read-back not compared) SURVIVED without it; now RED.
- *Body changed:* §0.3 (the method and the four facts), HA-195 and HA-196 rows (the binary as evidence; the SDK half
  still unread), §1.4.


**T2 (major) — the seam the Robot holds could not report the model check its boot line prints. REAL; fixed.**
- *Re-verified:* v4's `IVisionFrames` had one member, `read()`; `model()`/`modelProblem()`/`configured()`/
  `setupProblem()` existed only on `ProsAiVision`, and §1.3's fake had none; the builder never reads a frame. So the
  §1.8 boot line could only have been produced by a downcast (breaking the fake path and pulling the PROS adapter
  into `robot.hpp`).
- *Fixed:* `IVisionFrames` gains `[[nodiscard]] virtual CameraStatus status() const noexcept` (the seam is new and NOT
  frozen, so a second pure virtual costs nothing), with `CameraStatus {configured, setupProblem, model, modelProblem}`
  and `cameraStatusWords()` (ASCII words for each state); the adapter returns its own fields; `FakeVisionFrames` (now
  prototyped) has `setStatus()`. Probe R3.1 pins the adapter's three states through `status()` with no device call
  and every word < 0x80; R3.2 the fake. §1.8's boot line is built from the seam; §2.2's Robot test drives every
  state through the fake.
- *Body changed:* §1.1, §1.2 item 6, §1.3, §1.8, §2.2 (the Robot), landmine 7.

**T9 (minor) — the fake provider's read-fault answer departed from A1's pattern without saying so. REAL (unstated);
chosen and stated.**
- *Re-verified:* A1's fake answers `readFaults(port)` with its own `readFaults_[port]` (a `FakeReadFaults`), only
  where `distance()`/`optical()` served (`fake_device_provider.hpp` :401-408, banner :62-70).
- *Chosen:* the fake answers with the `FakeVisionFrames` itself, only once served — because the F4 distance/optical
  fakes cannot count (A1 left them unmodified, so it needed a companion counter), while the camera's fake is an
  `IReadFaults` as `ProsAiVision` is, and its count must move with the fault frames it returns. Stated in §1.6 with
  the reason; the provider test pins nullptr-until-served and a failing fake moving `readFaults(3)`; R3.2 shows the
  fake's count moving with its frames.


**T3 (minor) — "every shape compiles" was not true; one never-compiled change would have broken the build; two
`tick` signatures. REAL; fixed.**
- *Re-verified:* v4/proposed held 7 headers + the shim; `camera_config.hpp`, the fake, the feed, `bench_camera.hpp`
  and every hook into A1's files had never been compiled. Reproduced the break myself: a designated initializer that
  omits a member with no default initializer is `-Werror=missing-field-initializers` on g++ 13.3 AND
  arm-none-eabi-g++ 13.2.1; `ChassisTable` (`chassis_table.hpp` :69-91) has no default initializers and its eight
  initializers name every field.
- *Fixed:* prototyped and compiled the missing NEW headers (`config/camera_config.hpp` — the placement moved there
  from the geometry header, with `vision::cameraModelProblem()`; `hal/fake/fake_vision_frames.hpp`;
  `vision/camera_feed.hpp`) — probe R3 (4 cases) runs them; the ARM header gate over the tip + the ten new headers
  is 225 headers, rc 0; the CI PROS-free and sim-free greps are clean. Compiled the HOOKS as an overlay of the tip
  (`v6/overlay/`): with `ChassisTable::camera{}` (a default member initializer — the only one, commented why) all
  three tables compile on ARM and every test initializer stays valid, so `test/chassis_table_test.cpp` needs no edit
  for the build (VIS1-B still adds a case pinning robot two's block). One signature: `bool tick()` (the feed holds the
  clock it times reads with); D10's text corrected. §1's preamble now says exactly what is prototyped, what was
  compiled as an overlay, and what is not prototyped.
- *Body changed:* §1 preamble, D10, §1.7, §1.15 (the member, the B test), §2.1's VIS1-B row, §2.2 VIS1-B tests.

**T5 (minor) — two more breaks from appending `DeviceKind::AiVision`. REAL; both named, one moved.**
- *Re-verified by compiling, not by grep:* the overlay appends `AiVision` and `syntax-only`-compiles all 218 test
  files: exactly THREE switches fail `-Wswitch -Werror` — `deviceKindWord`, the provider's `foundWords` (:612), and
  the test's own `foundWords` (`mechanism_devices_test.cpp` :69); with a case in each, all 218 compile. The same run
  found a FOURTH break the checker did not name: `PortUse::Camera` breaks `portUseWord` (`robot_config.hpp` :539).
- *Fixed:* landmine 13 names all four switches and `kEveryKind`; §1.7 names `portUseWord`; the one-word flip of
  `boot_source_test.cpp` :159 moves into VIS1-A1 (a `test/` file — A1's `src/ tools/` diff stays empty), because A1 is
  the run that changes what `Vision` means and MECH-A5 rewrites that test in between. §0.2's DeviceKind row re-cited
  at the tip with all users.

**T6 (minor) — L6 was missing from the coordination section, though it shares VIS1's files. REAL; added (§6.7).**
- *Re-verified* at the committed tip `2506067`: `build-order.md`'s ruling row "The distance sensors and the camera's
  objects BEFORE THE PANEL" (:2177) puts L6 beside the MECH core, "before 2026-11-23 if it can"; `REMAINING-WORK.md`
  row 25 (:104). No L6 brief exists. To declare and build the left/right distance sensors L6 must touch
  `SensorsConfig` (`robot_config.hpp` :329-332), the Robot's three boot-line sites (`robot.hpp` :770, :787, :798),
  robot two's table (ports 4/5 — R2a :14-17) and `boot_source` — VIS1-A1 edits the first two, VIS1-B the last two.
  §6 named MECH, SER1 and CAL1 only; L6 appeared solely as "VIS1 builds none of it".
- *Fixed:* NEW §6.7 — the four shared files, the rule (whichever of L6 and VIS1-A1/B lands second re-reads and
  rebases them at its tip; never a hand-merge of a stale copy), and the TEMPLATE L6's sensors follow so the robot
  file does not grow two styles: D2/D5's pattern (the block's type in its own `config/` header, only hooks in
  `robot_config.hpp`; the one-owner claim with an appended `PortUse` and its `portUseWord` case — landmine 13; built
  by the provider's existing `distance()` (MECH-A1); a refused sensor never refuses the robot, an `…Absence()`
  sentence; ASCII boot-line words beside the camera's). §5's L6 bullet and §6.2's order line point to it.
- *Body changed:* §5 (the L6 bullet), §6.2 (the order line), §6.7 (new).

**T7 (minor) — HA-200 was cited by no prototype, so the register's reconciliation had no code site for it. REAL;
cited.**
- *Re-verified:* `grep -rno 'HA-[0-9]*' v6/proposed` found every id 192..203 EXCEPT 200 (HA-199 only in
  `sim/camera_plant.hpp`, which the reconciliation's grep over `include/shulib` counts).
- *Fixed:* `A4 register HA-200` now sits in `vision/camera_geometry.hpp`'s `groundRange` doc ("that a model box's
  bottom row IS the piece's base") and `sizeRange` doc ("that a model box spans the WHOLE piece") — VIS1-A1 — and at
  the finder's posture agreement (`piece_finder.hpp`, VIS1-A2); the re-grep finds all twelve ids. §4's HA-200 row
  names the sites; §2.3's DoD gains the direction-2 check (every id a run registers has at least one code site,
  named in the log).
- *Body changed:* §4 (HA-200), §2.3.

**T8 (minor) — VIS1-A2's new guide chapter needs two hand-kept index entries the brief did not list. REAL; listed.**
- *Re-verified* at `2506067`: `docs/guide/README.md` lists every chapter by hand (the table :9-25, chapters 1-17);
  `mkdocs.yml`'s "User guide" nav is hand-kept too (`api_doc_tool.py` generates ONLY the API reference's nav); and
  the tool's own `render_nav` doc (:1479-1491) says a page missing from the nav makes `mkdocs build --strict` emit an
  INFO line and PASS — an unlisted chapter would ship with no link and no failing gate.
- *Fixed:* §3.1's VIS1-A2 list names both (one table row, one nav line, at the chapter number the run takes at its
  tip; MECH-A3's mechanisms chapter may take 18 first). Index lines, not documents: rule 7's ≤ ~5 is unchanged.
- *Body changed:* §3.1 (VIS1-A2).

**T10 (minor) — line references mixed two trees; the "byte-equal" premise was stale; non-ASCII on screen. REAL;
re-cited at the committed tip.**
- *Re-verified at `2506067`* (SER1, committed 21:53 on `fd36e07`; MECH-A2's merge was in progress in the main
  worktree at 21:55 and touches none of VIS1's code files — its staged list read): every "(A1)" line is now the TIP's,
  and each was re-opened there — `hal/device_provider.hpp` :142, :178, :183, :196, :201, :222, the IMU banner :91;
  `hal/pros/device_provider.hpp` :183, :441, :612 (`foundWords`), :632 (`expectKind`); `hal/fake/fake_device_provider
  .hpp` :62-70, :103, :401-408, :477; `robot.hpp` :114-117, :543/:571/:1153, :646-696, :770/:787/:798;
  `robot_config.hpp` :325-332, :539; `robot_builder_test.cpp` :1782, :2170; `mechanism_devices_test.cpp` :8-9,
  :63-66, :69; `boot_source_test.cpp` :159; `device_registry.hpp` :50-80; `hal/pros/device_registry.hpp` :124, :128.
  THREE were wrong: §1.5's `foundWords` ":488-500" (pre-A1 — now :612-626); D11 and §0.2's `robot.hpp`
  ":535, :563, :1157" (pre-A1 — now :543, :571, :1153); §1.6's IMU note "(:91-100 (A1))" is in `hal/device_provider
  .hpp`, not the PROS provider. SER1 moved `bench_r3a.cpp` by +1 after :106 and +4 after ~:180 (one include; `emit`
  → `hal::pros::serialWrapped`): the T1-era citations (:2320-2327, :2426-2447) were already the staged SER1 tree's
  (= `2506067`); the v4-era ones (§0.2's row, D3, §0.3, §1.5) were `fd36e07`'s — §1.5's and §0.3's re-cited, the
  long §0.2 row and D3 marked `fd36e07`; §6.4's `calibrate_program.cpp` :1106 (pre-SER1) is :1115. `build-order.md`'s VIS1/AF1 entry is :1666-1683 and the two rulings
  :2176-2177 at `2506067` (§0.1 said :1641-1659 / :2151-2152 — `03a8081`'s); the MECH-A2 merge moves them again, so
  §0.1 names the headings too. `REMAINING-WORK.md`'s rows are unchanged at `2506067`. The header's "byte-equal at
  ~09:58" premise: A1's fix round changed `robot_config.hpp` and `mechanism/{kinds,spec,wiring}.hpp` at ~10:33 — no
  line this brief cites moved, and the ARM gate over the tip `2506067` + the ten new headers + the hooks is now
  **225 headers, rc 0** (`v6/armgate2/`, rebuilt this session; the CI PROS-free and sim-free greps clean). The non-ASCII
  check mark went at T2 (a grep for it over the brief: none left).
- *Body changed:* the header, the "every file:line" line, §0.1, §0.2 (title, the absent-sources row, the station-12
  row), D3, D11, §0.3, §1.5, §1.6, §6.1 (a re-check row), §6.3 (SER1 committed), §6.4, landmine 14.

**S1 (major) — D9 recommended a mount band where the tracker is blind. REAL; D9 rescored by the tracker, the
recommendation changed.**
- *Re-verified:* rebuilt and re-ran the checker's `c6_mount_range` / `c6c_nominal_truth` / `c6d_no_bottom_blob` on
  this session's prototypes (`v6/out/c6*.txt`; the pre-session outputs kept in `v6/out/pre-s/`): the v4 table scored
  a mount only by "whole face in frame", and a lens near the face's lower edge reports no colour — before this
  session the 12-in level lens read RED at 0 of 18 distances (12.36 truth), the 12¾-in one only inside 24 in.
- *Fixed:* NEW probe **R5** (`v6/probes/r5_mount_coverage.cpp`) scores 16 mounts × the lower edge at 12.55 / 12.61 /
  12.65 (the narrowed interval, S10) and 12.36 (out of belief), parked every 2 in from 80 to 4 in: below the face
  (6–11½ in level) the colour is reported from ~64–72 in down to ~10–18 in; **the blind band is ~11¾–12¾ in level**
  (12 in: 42..22 in at 10 of 39 distances; 12¼ in: never; 12½: 24..4 with gaps); above the face (13–14 in) to ~4 in
  but first seen only ~14–38 in out; pitching up 10° rescues a 12-in lens (72..12) at the cost of floor pieces near
  the robot; never a wrong colour in 2,496 parkings. D9 rewritten from R5: the table (far..near for each mount, P3's
  Pin column), the blind band named and explained, **(a) a lens ~10–11 in, level** (the colour ~70→12 in; a Pin 24 in
  ahead to ±~3.9 in), never 11¾–12¾ level (pitch up 10° if the build must put it there); "as high as the robot
  allows" dropped; the trade restated as far range (and floor pieces) against near range; what any mount means for
  AF1 and the Optical decision (S5). Step 0.1 stops at a lens height inside the band. R5's CHECKs (a 10-in lens first
  reports at ≥ 60 in — 68; a 13¼-in one at ≤ 44 in — 40; the 12¼-in one never; no wrong colour) join §2.2's tracker
  tests; mutation MT2 (no bottom-face gate) now also goes red in R5.
- *Body changed:* D9, the plain words (the tracker bullet), Step 0.1, §2.2 (VIS1-A2 tracker), §9's D9 line.

**S2 (major) — a Toggle stuck mid-roll is reported "sure" in the colour it is rolling toward; `<SC4>`a calls an
unseated Toggle neutral. REAL; stated, characterised, partly reduced; seating stays AF1's.**
- *Re-verified:* v20.txt :949-956 (`<SC4>`: set only when fully seated, a face parallel with its mounts, AND no robot
  touching; otherwise neutral, yellow); v4's header claimed a face mid-roll would read "two colours". The checker's C3
  (rebuilt): 56 cases "sure" between 20° and 100° of roll on v4-era prototypes (only the DESTINATION face is visible
  from below, so the two-colour veto never fires); research F2 (Q&A 3202) makes a stuck Toggle real.
- *Fixed / decided* (the host model and R1/R4 written in the interrupted session at 16:48-16:51, re-run now): the host
  model draws a ROLLED Toggle (`addRolledToggle(…, rollDeg, lift, …)`, a sharp equilateral prism, faces ~2.06 in, the
  axle lifted mid-roll). **Probe R1** asks whether a seating gate is possible: the face's lean the camera can measure
  (the apex's set-back interval) is never narrow enough — 0 of 21 seated RED faces resolved as seated, 0 rolled ones
  resolved as seated, at 8/10/12-in cameras and 6–30 in — so a seating gate, and a "lean" field in `ToggleView`, are
  REJECTED with that evidence (AF1 would get nothing usable). **Probe R4** (roll −120..+120 by 10°, cameras 8/10/12
  in, 8–30 in) characterises it: colours reported between 20° and 100° in 88 of 216 cases (16:51), 98 after S10's
  narrowing, **78 after R7's colour-above veto** (which catches a Toggle showing two faces stacked); the checker's C3
  now 35 (was 56). The header, §1.12 and the plain words say what a colour MEANS (the face SEEN where a seated face
  could be, never `<SC4>`'s set state), the mid-roll claim is corrected, §6.5 makes seating and contact AF1's to
  establish at the push point (S5), and Step 0.6 wedges a real Toggle at ~45° and ~90°.
- *Body changed:* the plain words, §1.11, §1.12, §2.0 Step 0.6, §2.2 (R4 as a characterisation; `addRolledToggle`'s
  anchors), §6.5.

**S3 (major) — the frozen-camera check compared translation only: a robot turning in place kept a stale colour.
REAL; fixed — the LENS's travel or the robot's turn.**
- *Re-verified:* v4 flagged frozen only when the robot CENTRE moved more than `frozenMoveIn`; the checker's C1: a
  frozen camera, the robot turning in place at ~90°/s, RED "sure" from 15° to 90° (a live camera there sees nothing).
- *Fixed* (prototyped in the interrupted session at 16:45, re-run now): the tracker computes the LENS's field position
  through the placement and the robot's heading, and flags frozen when the lens moved more than `frozenMoveIn` OR the
  robot turned more than the new `frozenTurnDeg` (UNSET refuses — `toggleTrackerConfigProblem`) and the box is
  pixel-identical. C1 now UNKNOWN "camera frozen?" at every step; **R2.S3** (T7b): frozen by the third 1° step, a live
  camera turning slowly never called frozen; **R2.S3b (new this session)**: with the turn check set to 45°, the lens
  7 in forward is flagged after 17° of turn (2 in of travel) — written because mutation **MT22** (the robot centre
  instead of the lens) SURVIVED without it; MT22 and **MT19** (the turn ignored) RED. The plain words and landmine 5
  now say what the check covers, and that parked nothing can tell.
- *Body changed:* the plain words, §1.12 (the config list, the freshness rule), §2.2, landmine 5, §6.5 item 6.

**S4 (major) — D6's "camera full" guard probably never fired: the device reports at most 24, and a read of exactly
24 was treated as complete. REAL; fixed — 24 or more is "possibly full".**
- *Re-verified:* `include/pros/ai_vision.h` :33 `#define AIVISION_MAX_OBJECT_COUNT 24` under "Parameters given by
  VEX"; v4's adapter set `truncated = count > 24` and §2.2 pinned "exactly 24 is NOT truncated"; the host model
  marked truncation only on pushes past 24, so every scene test shared the belief. The checker's C4 (rebuilt, this
  session's prototypes): a read AT the cap, its contradicting face never sent, built by hand with `truncated =
  false` — RED "sure"; it still fails as written, because it bypasses the adapter: the fix is the flag's source.
- *Fixed* (prototyped in the interrupted session at 16:43, re-run now): the adapter `truncated = count >= 24` (reads
  0..23), the host model the same; P1.20 (23 complete; 24 and 25 truncated, all 24 read) passes — the 20:32
  `p1_adapter.txt` failure in `v6/out/pre-s/` was a stale run, the source already said `>=`; R2.S4 (the host model:
  23 detections → the colour holds, 24 → UNKNOWN "camera full"); mutation **M14** (back to `>`) RED. HA-198
  reworded so the design no longer rests on whether a count above 24 can be reported; Step 0.2 counts how often a
  crowded field reads 24.
- *Body changed:* D6, §0.4's P1 row, §1.1 (`truncated`), §1.2 rule 3, §1.4, §1.11, §2.2 (the adapter's cap test,
  the host model), HA-198, landmine 2.

**S5 (major) — §6.5 let a side distance sensor be AF1's "second evidence"; it cannot corroborate COLOUR. REAL;
the contract rewritten.**
- *Re-verified:* §6.5 said "a second, close-range one (an Optical at the flipper, or the side distance sensor)"; the
  sensors face left and right (the team lead, 2026-10-08), the camera forward; a distance reading says something is
  at range X — an opponent in front of our Toggle, or a stuck Toggle, satisfies it. The camera stays the only colour
  source, and it reports a destination colour for an unseated Toggle (S2), is blind at some mounts (S1) and loses the
  colour turned close in (R7). Research F1/F10's fix is an Optical at the flipper (colour, reach, the post-push check).
- *Fixed:* §6.5's AF1 contract is six numbered rules: before any push the face's colour must be corroborated AT THE
  PUSH POINT by a sensor that sees colour there (in this plan an Optical at the flipper); a distance sensor gates
  reach and presence only, never colour; the camera is the far-range cue whatever Step 0.4 shows. **The Optical is
  NOT decided here** — the team lead's ruling puts it after Step 0 — so D9's last bullet and Step 0.4 state the
  consequence for that decision: without a colour sensor at the push point, AF1 cannot push on colour evidence.
- *Body changed:* the plain words ("What it does not do"), D9, Step 0.4, §6.5.

**S6 (major) — three beliefs the geometry's "bound" rests on were unregistered: the image's age, a colour box's
extent, the optical axis against the housing. REAL; folded into HA-192, HA-202 and HA-194 as explicit claims (no
thirteenth number — this return tells the coordinator, who may split them); a widening helper and its tests.**
- *Re-verified:* a grep for latency/lag/delay over the brief and the prototypes found nothing; `VisionFrame::readAt`
  is the read's COMPLETION; the tracker's position gate, the finder's fix and AF1's window all pair a frame with the
  pose at completion. The checker's C5 (rebuilt): turning 180°/s, a Pin 30 in ahead, 10-in camera — at 66 ms the true
  Pin is 6.0 in from the fix whose radius is 3.4 (OUTSIDE), at 100 ms 8.8 vs 3.0; the brief called the interval "a
  bound". Gate 3 takes a colour band's top and bottom rows as the apex and the lower edge (HA-202 covered only a
  still object's box); Step 0.1 levels the camera's BACK FACE, and that the optical axis is square to it was no HA.
- *Decided — folded, not a thirteenth entry:* each is a claim of an entry that already owns the quantity, stated
  explicitly with its own settle: **HA-192** gains "the image is older than the read's completion by an unmeasured
  `L`", settled by Step 0.7's LATENCY check (the LIBRARY page paints a taught colour on the brain's own screen at a
  logged `pros::micros()`; the first read reporting it bounds `L` from above — the screen's delay included, the
  conservative direction); **HA-202** gains the colour band's rows (glare, rounded corners — merging S9(c)), settled
  at Step 0.5; **HA-194** gains the axis against the housing, settled at Step 0.2 (a Pin on the centreline at 24 and
  48 in). **The coordinator decides whether to split them into new numbers** (the brief's rule: "a thirteenth
  belief ⇒ stop and tell the coordinator" — this is that telling; HA-192..203 stay the only numbers held).
- *Fixed in the prototypes:* `vision::boundAtImage(atRead, speed, turnRate, maxImageAge)` in `camera_geometry.hpp`
  (the position tolerance + |v|·L, the heading tolerance + |ω|·L; a negative or non-finite input → an infinite
  position tolerance, never a tight bound); `PoseBound`'s and `readAt`'s docs say "at the image's time"; the header
  paragraph on the axis. VIS1 invents no `L`: the consumers type the measured one (AF1, L4 — §6.5; UNSET ⇒ field
  positions only with the robot stopped). **Probe R6:** R6.1 (turning 180°/s, a Pin 36 in ahead, lag 0–150 ms):
  un-widened, OUTSIDE at 66/100/150 ms (7.0 vs 4.4, 10.5 vs 3.8, 15.4 vs 5.2 in); widened, inside at every lag;
  R6.2 (60 in/s past a Pin 14 in aside): un-widened OUTSIDE at 150 ms, widened inside throughout; R6.3 (the helper's
  arithmetic and refusals); R6.4 (the tracker with its position gate on, a lagged image while turning 60°/s, up to
  150 ms: never a wrong colour or Toggle — lag can only cost a refusal). Mutations **MG5** (the turn term dropped)
  and **MG6** (the speed term) RED. The checker's C5 still fails as written — it pairs the frame with the pose at
  completion and does not widen, which is the finding; R6 is the fix's test.
- *Body changed:* the plain words (freshness), §1.1 (`readAt`), §1.9 (`PoseBound`, `boundAtImage`, the interval's
  "stated tolerances", the axis), §1.15 (the LATENCY check), Step 0.2 / 0.5 / 0.7, HA-192 / HA-194 / HA-202, §2.2,
  §6.5 (AF1 rule 4; L4), landmine 19, §2.1 (VIS1-B's size).

**S7 (minor) — "two colours → UNKNOWN" held only among PASSING bands; a cut or bottom-face-ambiguous band of another
colour never vetoed. REAL; fixed.**
- *Re-verified:* v4 skipped a band whose gate said Cut, or whose bottom-face check said CannotTell, so `two` was set
  only between passing bands; the plain words promised "two colours at once, a band cut by the picture's edge —
  UNKNOWN". The checker's C2: a passing RED face 30 in out + a BLUE band touching the image top in the window → RED.
- *Fixed* (prototyped in the interrupted session at 16:45, re-run now): a band of ANOTHER Toggle colour that is Cut,
  or CannotTell, whose bearing touches the window → UNKNOWN "ConflictingPartialBand". C2 now UNKNOWN; **R2.S7** (a
  cut BLUE → UNKNOWN, a cut RED → RED); **R2.S7b (new this session)** isolates the CannotTell half — 78 hand-made
  left-clipped BLUE bands the bottom-face check alone cannot judge, beside a far RED face with NO column overlap, all
  vetoed — written because mutation **MT24** (that half removed) SURVIVED: every host-model scene where it fires is
  also caught by R7's colour-above veto (a CannotTell band is a near band near the frame's top, overlapping and above
  anything that passes), so only a hand-made frame reaches it. MT20 and MT24 RED. The plain words corrected.
- *Body changed:* the plain words, §1.12 (the rule), §2.2.

**S8 (minor) — side-clipped bands skipped the length gate; only the bearing window stood in the way, and nothing
bounded it. REAL; fixed — a side-clipped band is judged only if it spans the image's centre column.**
- *Re-verified:* v4 skipped LENGTH for any side-clipped band and accepted any `bearingWindowDeg > 0`; the checker's
  `c7b`: with a 35° window, 219 (8-in) and 274 (10-in) of 9,054 short red bands (4–6.5 in — `<R4>`g plates are
  4.0–4.5 in, v20 :2301-2305; a lying Pin 6.5) read RED "sure". T10's lookalike is a 26-in plate on the face's line,
  not a legal plate.
- *Fixed* (prototyped in the interrupted session at 16:45, re-run now): a side-clipped band that does not span the
  image's centre column is "cut" (a real face clipped at the flip position, wider than the picture, does span it) —
  chosen over a derived cap on the window, which would cap AF1's reach geometry by a number the team cannot see.
  `c7b` now 0 of 9,054 at 25° AND 35°, both cameras; **R2.S8**: 0 of 732; mutation **MT21** RED. That change left
  **MT12** (the bearing window ignored) ALIVE — P4's S15 face, 31° aside, is now "cut" before the window gate — so
  **P4/S15b (new this session)** pins the window's containment on WHOLE, uncut faces (centre outside 25° → UNKNOWN,
  3 cases; inside → RED, 18) and MT12 is RED again; **MT25** (the S8 rule and the window together) RED. Step 0.6 now
  uses a REAL 4.5-in plate (R2.S8's case) AND a long red strip (T10's case), and says which is which.
- *Body changed:* §1.12 (gate 3), §2.0 Step 0.6, §2.2.

**S9 (minor) — Step 0's gaps: (a) the wrong criterion at 0.4, (b) an undefined reference point, (c) a jitter settle
on the wrong box, (d) no versions, (e) the arm in the picture, (f) no event-day check. REAL, each; each fixed.**
- *(a) Re-verified:* 0.4 judged "whole face in the picture" through the USB video, but R5 shows whole-in-frame is not
  "reported" (a 12-in level lens has the whole face from ~4¾ in yet reports at 10 of 39 distances), and the video's
  crop against the 320 × 240 detection frame is unverified. *Fixed:* 0.4 records station 12's RECORD lines at the
  flip position and at 12/24/36/48 in; station 12 turns on TAGS and OBJECTS only (`setUpCamera` → `aivision_enable_
  detection_types`, which the binary shows is `EnableSet(t, t)` — COLORS left as found, `bench_r3a.cpp` :2388-2410
  at `2506067`), so COLOR boxes appear only if the camera kept the Utility's colours (HA-196) — when they do, the
  coordinator runs them and 0.1's mount through the v6 tracker and host model before the mount is fixed; when they
  do not, the video stands and the tracker's verdict waits for 0.5. Stated, not assumed.
- *(b) Re-verified:* `ChassisTable` (:69-91) has no fore-aft centre; the library's pose is the TRACKING CENTRE (guide
  02 :128). *Fixed:* 0.1 measures from the tracking centre, and says where it is on robot two (a tank on drive-encoder
  odometry, no pods — `boot_source.hpp` :235, :309: the point it turns about in place, found over a taped cross).
- *(c)* merged into S6 (HA-202 now names colour bands; settled at Step 0.5).
- *(d) Fixed:* §2.0's preamble — every provenance line names VEXos, the sensor's firmware and the AI model as the
  Utility shows them, VEXcode's version and the PROS kernel; 0.2 records any update the Utility makes.
- *(e) Fixed:* 0.4 moves the arm by hand (motors off) through its stroke: in the picture? what colour? does it cover
  the face? 0.6 adds the arm to the lookalikes.
- *(f) Fixed:* NEW Step 0.8 — on the event's practice field under its light, the three faces at 24 and 48 in; the
  re-teach path (the Utility → robot two's table → rebuild and re-upload: the descriptors are compiled in, so there
  is no other path), in Station I and the camera chapter's bench section (§1.15).
- *Body changed:* §2.0 (the preamble, 0.1, 0.2, 0.4, 0.5, 0.6, 0.8 new), §1.15 (the worksheet bullet).

**S10 (minor) — HA labels outside the register's vocabulary, HA-192 mislabelled, HA-201 ignored the glossary's
face width. REAL; relabelled, and HA-201 narrowed to the glossary.**
- *Re-verified:* the register defines Confidence as measured elsewhere / reasoned / invented (`hardware-assumptions
  .md` :15-20); it uses "reported" only for the TEAM LEAD's own reports (HA-124, HA-153, HA-159), never a VEX
  snippet; "read" and "unknown" are not its words. HA-192's ~30/s and HA-194's 74° × 63° come from the SAME KB
  article seen only through search snippets (research `check` F1). The glossary (v20.txt :4668): "each face of the
  triangle is approximately 2.05” (52.2mm) wide" — three such faces make an equilateral prism whose lower edge is
  ≈12.61 in; v4's 12.36 (a 51.6-mm VERTICAL span) implies 2.34-in sloped faces.
- *Fixed:* HA-192 and HA-194 are both **reasoned** (snippet-only VEX figures; HA-192's no-counter half from the
  header and binary); HA-198 **reasoned** (its claim reworded at S4); HA-197 unchanged ("measured elsewhere" for the
  names, from the api.vex.com page fetched directly). HA-201 cites the glossary and pA8's circumradius, and the
  prototype's `ToggleFace` is narrowed — the lower edge **[12.55, 12.65]** (was [12.36, 12.65]), the set-back
  **[0.9, 1.2]** (was [0.9, 1.5]; side/2 ≈ 1.03) — and every probe re-run on it: all cases pass; P4b's lookalikes
  75 → 65 of 540 (96-in window), 46 → 39 (36-in), still 0 with the position gate; T10's plate from 5 to 2 distances;
  S1's approach first RED at 70 in (72 before); P6/T6's 12¾-in lens to 3 in. 12.36 stays in R5 as an out-of-belief
  truth: never a wrong colour. Step 0.1 still measures a real Toggle.
- *Body changed:* §0.5 item 2, §1.10, HA-192 / HA-194 / HA-198 / HA-201, D9's footnote, the plain words ("~12.6–14.4
  in").

**R7 (MAJOR — found BY this re-check, in neither checker's list) — with the robot TURNED to a seated Toggle, the
tracker reported the BOTTOM face's colour "sure". REAL; fixed at the root of the wrong answer; a coverage limit
stated.**
- *Found* by R6.4's first draft: every earlier Toggle scene faced the wall square (heading 0). An 8-in camera, lens 30
  in from a RED Toggle with a YELLOW bottom face, the robot turned +15°: YELLOW "sure" — and the same on the
  prototypes as they stood before this session (`v6/out/pre-s/proposed`) and on v4's own (`v6/probes/r7b_v4_turned
  .cpp`, built against `v4/proposed`: 16 wrong colours of 78 turned parkings), so it is not a side-effect of S10. Cause:
  off the optical axis a horizontal edge projects SLANTED; the field face's box grows (21 rows against 12 square),
  "one camera explains it" refuses it, and the thin bottom face — explained as neither a bottom face (the slant) nor
  refused — passed as the only face.
- *Fixed:* a band with a box of ANOTHER Toggle colour directly ABOVE it (columns overlapping; the other's top above
  the band's, its bottom within 2 × `kBoxEdgePx` of the band's top or lower; whatever ITS gates said) is never
  reported — `ToggleWhy::ColourAbove`. A seated field face has nothing of another Toggle colour directly above it
  (its bottom face is below); a bottom face does, and so does a Toggle showing two faces mid-roll. **Probe R7**
  (cameras 8/10/11 in, the bottom face BLUE or YELLOW, the lens 14–48 in, heading −30..+30 by 5°, 390 cases): **0
  wrong colours; with the veto removed (mutation MT23, RED) 38 of 390 were the bottom face's colour, "sure"**. Every
  other probe unchanged by it (all cases pass); R4's mid-roll colours fall 98 → 78.
- *Stated (the cost, which AF1 must design for):* close in the colour holds only near-square — at 14–20 in only
  within ±5°, ±10–20° at 30–48 in (R7's table, printed). A turned approach reads UNKNOWN, never a wrong colour; AF1
  squares up before it judges (§6.5 rule 5). Station I's 0.5 records ±5/10/20° to measure it.
- *Body changed:* the plain words, D9, §1.12 (the rule, `ToggleWhy`), §2.0 Step 0.5, §2.2, §6.5 rule 5, landmine 18.

---

## Decisions for the team lead (a recommendation for each; nothing below decides them silently)

**D1 — Three runs, and when each starts. RECOMMENDED (a).**
- **(a)** **VIS1-A1, the camera as a device** (the frame seam, the PROS adapter, the provider accessor, the robot
  file's camera, the Robot, the geometry, the season's data, the host camera model) starts **when MECH-A1 merges** —
  it edits five of A1's files (`hal/device_provider.hpp`, `hal/pros/device_provider.hpp`,
  `hal/fake/fake_device_provider.hpp`, `config/robot_config.hpp`, `robot/robot.hpp`) and A1's
  `test/robot_builder_test.cpp`. **VIS1-A2, what the camera means** (the Toggle tracker, the piece finder, the camera
  feed) starts when A1's BUILD ends (the overlap speedup MECH-A2 used: A2 shares no file with A1's code, only
  documents). **VIS1-B, robot two and the bench page** (`src/`) starts **after MECH-A5 merges and before MECH-A6
  launches** (A5 rewrites the programs and `boot_source`; A6 writes `bench_r3a.{hpp,cpp}`); if A6 has already
  launched, B waits for A6 to merge. A1 and A2 build beside MECH-A2–A4: they share no code file with them (§6.2).
- **(b)** One run after MECH-A5. **Cost:** nothing of the camera until ~A5 (weeks); the library half never needed to
  wait for A5.
- **(c)** Start A1 now from MECH-A1's snapshot (`85e0285`), as MECH-A2 did. **Rejected:** A2 shares no file with
  A1's code, VIS1-A1 shares five that A1's review is changing right now; porting a fix round into files both edit is
  exactly the merge this project has learned to avoid (the coordinator's PORT RULE for A2 already carries that risk
  once).

**D2 — Where the camera's facts live: the robot's own table, as config data. RECOMMENDED (a).**
- **(a)** `config::CameraConfig` (a NEW header `config/camera_config.hpp`, so the A1-shared `robot_config.hpp`
  gains only its hooks: the include, the appended `CameraConfig camera{};` in `SensorsConfig`, `PortUse::Camera`,
  and the camera's checks in `valid()`): the
  port; the adapter's settings (the measured colour descriptors with their names and provenance, OBJECTS on/off, the
  expected class table); the MEASURED mount with its tolerances and provenance. `RobotConfig::valid()` claims the
  camera's port through the one-owner claim (a motor typed on the camera's port is refused naming both). Robot two's
  values go in `src/robots/chassis_table.hpp` as a member of its `ChassisTable` (`.camera = {...}`; the table already
  carries the IMU's port), so the `src` gate's two table files stay the only files that name a robot's tables (G3a's
  coupling, `build-order.md`'s G3a entry). The robot file is the one place a robot's facts live (MECH Q1 (a)); the
  panel (G3a) later writes the same block.
- **(b)** A struct the team's program code builds and hands the trackers. **Cost:** a second place a robot's facts
  live; G3a could not write it; the boot line could not report it.

**D3 — The bench page: Station 12 gains the colours. RECOMMENDED (a).**
- **(a)** **Extend Bench Tests station 12** (`src/programs/bench_r3a.cpp` ~:2133-2870 at `fd36e07`): it turns COLORS on with the
  robot table's descriptors when they are SET (it says "colours UNSET — teach them (Step 0.3)" when not), shows the
  model check's line, times one read (HA-203), beside every blob's RAW record prints the library's MEASUREMENTS of it
  (a colour band: its aspect, the range at which the camera explains it as a Toggle face, its implied length; an
  object: its posture, ground and size ranges, bearing) — numbers, never a verdict that needs a threshold the team has
  not typed — and gains a LIBRARY page (one more state on the FAMILY button) where the library's own adapter reads the
  camera (§1.15). One camera page, one worksheet station (I), no new program. Its edits are inside station 12's own
  functions plus a NEW PROS-free `bench_camera.hpp`; MECH-A6's are the menu (`bench_r3a.hpp` `kMenu`) and a new
  station 13 — textually disjoint, and A6 re-reads its tip anyway (MECH brief §2).
- **(b)** A new station 14 after A6's 3 × 5 menu. **Cost:** waits for A6 (the last MECH run); two camera pages to
  keep true.
- **(c)** A new program "shulib Camera" (`PROGRAM=camera`). **Cost:** an eighth build — `Makefile`,
  `src/build_selection.hpp`, `src/main.cpp`, the `src` gate's table, which CAL1 Part B, G3a Part 2 and MECH-A5 all
  edit — and another upload slot for the team to keep straight.

**D4 — `DeviceKind` learns the AI Vision sensor. RECOMMENDED (a).**
- Today the brain's registry answers `E_DEVICE_VISION` (the older Vision sensor) and `E_DEVICE_AIVISION` with the
  SAME `DeviceKind::Vision` (`hal/pros/device_registry.hpp` :124, :128; `hal/device_registry.hpp` :57), so a
  registry check cannot refuse an older Vision sensor on the camera's port — R2a named the trap and left it
  (`R2a-sensor-stations.md` :101-102). The older sensor is legal (Q&A 3157) and a team may own one.
- **(a)** Append `DeviceKind::AiVision` (after `Other`: the value is compared in memory only — no record stores
  it, checked: every `DeviceKind` use is a comparison or a word), map `E_DEVICE_AIVISION` to it, word "AI-VISION";
  `Vision` keeps meaning the older sensor. The standing rule: "a chunk that finds a flaw in an earlier chunk fixes it
  there, not around it" (RESUMING, standards).
- **(b)** Keep the enum; the provider's camera accessor reads the PROS type itself. **Cost:** the census
  (Calibrate's, the boot check's) keeps calling the camera "VISION", and the next sensor that needs the difference
  rediscovers it.

**D5 — A camera the robot file declares but the brain cannot make. RECOMMENDED (a).**
- **(a)** The robot **builds and drives without it**: `robot.camera()` is `nullptr`, `robot.cameraAbsence()` says
  why ("port 3: nothing plugged in (expected an AI Vision sensor)"), the boot line says it, and every consumer that
  needs the camera refuses to construct — exactly as a mechanism's refused piece sensor (MECH §1.5: "refused = it
  runs WITHOUT one"). A dead camera must never stop a robot from driving a match.
- **(b)** Refuse the whole build, as a drive motor is. **Cost:** a camera cable out at an event costs the match.

**D6 — A read that may have been cut off at the camera's 24. RECOMMENDED (a).**
- The camera reports at most 24 (`ai_vision.h` :33, `AIVISION_MAX_OBJECT_COUNT 24` under "Parameters given by VEX"
  — so a camera holding more may still REPORT 24: a read of EXACTLY 24 cannot show the band that contradicts a colour
  was not the 25th; re-check S4); a field of 63 Pins can fill it (each Pin fires two colour
  blobs AND one object), and the order the 24 come in under PROS is UNVERIFIED (VEXcode's own snapshot sorts
  "largest first", by width in one paragraph and by area in another — api.vex.com, read 2026-10-15; whether the raw
  list PROS reads is sorted at all is unknown, HA-198).
- **(a)** A read reporting **24 or more** is "possibly full" (`VisionFrame::truncated`; re-check S4 — v4 marked only
  MORE than 24, and the checker's C4 showed a read at the cap, its contradicting face never sent, reported RED
  "sure") — never evidence of absence and never evidence of "only one colour": the tracker says UNKNOWN "camera
  full", the finder UNKNOWN "camera full" (probes T2, P2, P1.20, R2.S4; mutations MT1, MF2, M14). Safe whether or
  not the camera can report more than 24 (HA-198 no longer matters to it); the cost is a rare crowded read. The team keeps the
  working set small — only the colours and the model the robot needs — and Step 0.5 counts how often a real field
  fills it.
- **(b)** Trust a size order. **Rejected until Step 0 shows it:** a band the camera dropped could be the one that
  contradicts the colour reported.

**D7 — The Override AI model: checked BY NAME before an object id means anything. RECOMMENDED (a).**
- **Settled as far as reading can settle it (2026-10-15):** a PROS program **cannot choose the AI model** — the
  vendored PROS 4.2.2 API has no such call (`include/pros/ai_vision.h` read whole: modes, colours, codes, tag
  families, class names, the overlay, white balance, reads — nothing selects a model; `ai_vision.hpp` likewise).
  VEX's API page says the sensor "processes visual information using an onboard AI model selected in the AI Vision
  Utility within VEXcode", and for VS Code "the AI model must first be configured in VEXcode before it can be used in
  your program" (api.vex.com, V5 C++ AI Vision Sensor page, read 2026-10-15); its Override table is 0 `redBluePin`,
  1 `blueYellowPin`, 2 `redYellowPin`, 3 `yellowPin`, 4 `cup`. So the model is chosen in VEXcode V5 ≥ 4.68's AI
  Vision Utility and — **UNVERIFIED, HA-197** — kept on the sensor, where a PROS program can read it. **What a PROS
  program CAN do is read the camera's class names one id at a time** (`aivision_get_class_name`, `ai_vision.h`
  :371-389) — **but NOT how many there are (re-check, T1):** the header's "-1 returns the number of class names"
  (:375) is not what the vendored binary does — `libpros.a` 4.2.2's `aivision_get_class_name` discards the SDK's
  value and returns `PROS_SUCCESS` (1) for EVERY id, -1 included (§0.3). Bench Tests station 12 already prints the
  name of every OBJECT it records, beside its id (`bench_r3a.cpp` :2320-2327), so Step 0.2 still settles HA-197
  with NO new code — from the RECORD lines with each piece type in view, not from the entry log (which will say
  "get_class_name(-1) -> 1" and print class 0 only, `logClassNames` :2426-2447).
- **(a)** The adapter reads the name of each EXPECTED class id (0..4) at set-up, into a zeroed buffer, and compares it
  **by name** with the robot file's table (`override2026::kOverrideClassNames`) — never asking for a count, which
  PROS cannot give (re-check, T1); until every name matches, **no Object detection is passed on** (counted in
  `objectsWithheld`), colours still are; an Object whose id is PAST the expected table is never passed on (its name
  was never read). Re-asked once a second while it does not match (the names "may not
  correctly report … for the first several hundred milliseconds of being plugged in", `ai_vision.h` :373-374). The
  shim's DEFAULT model is VEX's Classroom table, whose class 3 is `blueRing` — the trap made real (probe P1.11; P1.14:
  one wrong name of five is caught; P1.15: a camera with no model names '' and is `Different`; P1.16: id -1 is never
  asked; P1.17: an id past the table is withheld; mutations M8–M12 red — `v6/out/p1_mutations.v6.txt`).
- **(b)** Trust the ids. **Rejected:** a camera carrying the Classroom model reports `blueRing` as id 3 — the
  Override table's `yellowPin` — and a route would chase a ring.

**D8 — The alliance colour picker: its own small chunk, ALLY1, next to VIS1 — not inside it. RECOMMENDED (a).**
- Nothing knows our colour today (every program sets `SessionInfo::alliance = ""`; the auton menu has no colour row
  — the research's library map §5). L4 needs it to choose "our" piece; AF1 needs it for the Toggle's target colour
  (RED / BLUE / SKILLS, UNSET until chosen, no arming without it — F11).
- **(a)** **ALLY1** (~1–1½ builder-days, rough, its own brief): `include/shulib/competition/alliance.hpp` (the
  value — `Alliance {Unset, Red, Blue}`, `MatchKind {Unset, HeadToHead, Skills}` — kept across autonomous → driver
  control in static storage, and its words) and the brain-screen picker's colour row (`competition/auton_menu.hpp`);
  its `src/` half (the menu row in the competition program and the Auton Tester, `SessionInfo`, a driver override
  button) after MECH-A5. **Placed after MECH-A4 merges** (A4 rewrites `competition/{auton_registry,match_runner}.hpp`
  beside the menu) **and before L4 and AF1**, which both consume it. VIS1 builds none of it and needs none of it: the
  tracker reports a face's colour and the finder a piece's class; "which is ours" is the consumer's.
- **(b)** Inside VIS1. **Cost:** VIS1 would wait for MECH-A4 (the menu's neighbours), delaying the camera for a
  feature the camera does not use.
- **(c)** Inside AF1, as the research designed it. **Cost:** L4 needs it first if L4 lands before AF1.

**D9 — The camera's mount, for the build team (a recommendation from the probes, not a library choice). RECOMMENDED
(a). (re-check S1: rescored by what the TRACKER reports, near AND far; the v4 recommendation "as high as the robot
allows, ~10–12¾ in" put the lens in a blind band.)**
- The library models any height, pitch and yaw (nothing assumes a level camera on this path; HA-70's level camera
  stays the TAG corrector's belief). But WHERE it sits decides what it can see. Probe R5 (`v6/probes/r5_mount_coverage
  .cpp`) parks the robot square to a seated Toggle (RED field face, YELLOW bottom face) every 2 in from 80 in to 4 in,
  the true camera inside the stated tolerances (0.2 in high, 0.8° down, 0.7° yawed, field of view 75.5 × 62), and
  records where the tracker says RED "sure"; P3 gives the floor-Pin column (HA-194's optics throughout):

  | Lens height, pitch | Toggle colour reported from (far) .. to (near), lens to wall — lower edge 12.61 (12.55 / 12.65) | A Pin on the floor 24 in ahead: fix radius |
  |---|---|---|
  | 6 in, level | 68 .. 18 in (70 / 66 .. 18) | ~5.3 in |
  | 8 in, level | 68 .. 14 in (70 / 66 .. 14) | ~4.4 in |
  | 8 in, up 10° | 68 .. 10 in | ~4.8 in |
  | **10 in, level** | **68 .. 12 in** (72 / 68 .. 12) | **~3.9 in** |
  | 10 in, up 10° | 70 .. 8 in | not in view (below the frame) |
  | **11 in, level** | **70 .. 12 in** (72 .. 10 / 68 .. 12) | (~3.7 in, between P3's 10 and 12) |
  | 11½ in, level | 64 .. 12 in | — |
  | **11¾ in, level — the BLIND BAND starts** | 50 .. 14 in (60 .. 12 with 4 blind distances) | — |
  | **12 in, level** | 42 .. 22 in, colour at only 10 of 39 distances | ~3.6 in |
  | **12¼ in, level** | **never** (all three believed truths) | — |
  | **12½ in, level** | 24 .. 4 in, with gaps | — |
  | **12¾ in, level — the band ends** | 14 .. 4 in | — |
  | 13 in, level | 24 .. 4 in | — |
  | 13¼ in, level | 38 .. 4 in | — |
  | 14 in, level | 20 .. 4 in | — |
  | 12 in, up 10° | 72 .. 12 in | not in view |

  **The blind band, ~11¾–12¾ in level:** a lens within about an inch of the face's lower edge looks along it — the
  lower edge's ray is too near the horizontal to range (refused under 0.5°), and the bottom-face guard cannot see
  above the band — so the colour is lost at every distance or most (the checker's c6 showed it first; R5 pins it:
  the 12¼-in lens never reports a colour). Below the band the far range is kept (~64–72 in) and the near limit is
  ~10–18 in; above it the near limit is ~4 in but the far range collapses to ~14–38 in. Pitching up 10° rescues a
  12-in lens (72 .. 12 in) but loses floor pieces near the robot. **Turned to the Toggle** (probe R7, re-check: a
  defect found there, fixed — §1.12's colour-above veto), the colour holds only near-square close in: at 14–20 in only within ±5° of
  square, ±10–20° at 30–48 in. (Typical tolerances ±1° pitch/yaw, ±¼ in, the optics ±5 %; every number rests on
  HA-194's optics and HA-201's Toggle geometry — the lower edge now the INTERVAL [12.55, 12.65], re-check S10; with
  v4's 12.36 as the truth, outside that belief, R5 never reports a wrong colour either.)
- **(a)** **A lens ~10–11 in up, level, rigid, then measured (Step 0.1)** — the Toggle's colour from ~70 in down to
  ~12 in when the robot is square to it, and a floor Pin 24 in ahead fixed to ±~3.9 in (L4). **Never 11¾–12¾ in
  level**; if the build can only put the lens there, pitch it up ~10°. The trade is **far range (and floor pieces)
  against near range**: below the face the camera is the far-range "Toggle ahead" cue and loses the colour inside
  ~12 in; above it (13–14 in) the colour holds to ~4 in but is first seen only ~14–38 in out.
- **(b)** Above the face (~13¼ in, level): the colour to ~4 in, seen first at ~38 in; costs the far cue.
- **(c)** Leave it where it is (robot two: "front, level, roughly centred", height unmeasured — R2a :15-17): Step 0.1
  measures it first — inside the blind band it must move.
- **What any mount means for AF1 (and the Optical decision — the team lead's after Step 0, NOT decided here; re-check
  S5):** no mount below the face reads the colour at a flipping position closer than ~8–12 in, none reads it turned
  more than ~5° close in, and NONE can tell a seated Toggle from one stuck mid-roll (S2) or see whether a robot is
  touching it (`<SC4>`a/b). So the camera's colour is the far-range cue whatever Step 0.4 shows; AF1's push needs the
  face's colour corroborated AT THE PUSH POINT, which in this plan only an Optical at the flipper gives (§6.5). If
  the Optical is decided against, AF1 cannot push on colour evidence — VIS1 states that consequence for the decision.

**D10 — Who reads the camera, and how often: the consumer's own task, through one feed, never a library task.
RECOMMENDED (a).**
- A read is 1 + N device calls (the count, then each detection — `ai_vision.h` :463, :479; up to 25), and the
  library is single-task by contract (`hal/mechanism.hpp` :44-54; `vision.hpp` :19-21 already says vision is read OFF
  the 10 ms hot path, "at a lower rate").
- **(a)** `vision::CameraFeed` (§1, VIS1-A2): the program's loop (the driver loop's `perTick`, an auton step's wait
  predicate) calls `feed.tick()` once a tick (the feed holds the clock it times each read with — re-check T3: one
  signature, §1.14's, prototyped); it reads the camera at most once per `minReadPeriod` (the team's
  number — VEX states ~30 frames a second, HA-192, so ~33 ms is the natural floor), times each read (HA-203), and
  hands the SAME frame to every consumer (the tracker, the finder, later AF1 and L4) — two consumers never cost two
  reads, and no consumer can read faster than the camera changes.
- **(b)** A vision task of the library's own. **Rejected:** a second task breaks the single-task contract every
  device in the tree is written to (the claim model is "a plain flag, not a mutex"), and a task PROS deletes on a
  disable mid-read leaves nothing to clean up but adds the deletion exposure SER1's HA-190 describes.

**D11 — The frozen `hal::IVision` stays `AbsentVision` in the Robot. RECOMMENDED (a).**
- `RobotContext::vision` points at `hal::AbsentVision` today (`robot.hpp` :543, :571, :1153 at the tip `2506067` — re-check T10). `IVision` cannot carry what
  VIS1 needs (no type — a colour descriptor 1 and a model class 1 are both `classId` 1; no box, no time —
  `vision.hpp` :52-63), returns a `std::vector` (allocates), and has no consumer in the tree.
- **(a)** VIS1 installs nothing behind `IVision`; every VIS1 consumer takes frames. Decision #7 ("ONE adapter
  implements both", `vision.hpp` :3-6) still holds: `hal::pros::ProsAiVision` is the class R2b's tag part later
  extends with `ITagSource` (and `IVision`, if a consumer of it ever appears) — additively.
- **(b)** An `IVision` view over the frames (bearings only). **Cost:** a second, lossy path to the same camera
  with no reader; the opaque-id trap the sibling exists to remove.

---

## The design in plain words (for the team lead)

- **The camera becomes a device the library can open.** The robot's table says "an AI Vision camera on port 3,
  these colours, the Override model, mounted here". At boot the library writes the colours and the detection modes to
  the camera (it never trusts what an earlier program left on it), checks by NAME that the camera carries the
  Override model, and from then on reads it a **frame** at a time: every coloured blob and every model object, each
  with its box in the picture, and the time the read finished. A read that fails is a frame that SAYS it failed —
  never the last good picture held over. Tags stay off.
- **Freshness is a clock, not a counter.** PROS gives no frame number and no timestamp (F5), so the library can only
  say "this read finished at 12.430 s". Everything downstream asks: has the same answer held for T seconds, and did a
  good read arrive in the last X seconds? A camera that freezes looks exactly like a still scene; while the CAMERA
  moves — the robot drives OR turns (re-check S3: a turn in place barely moves the robot's centre) — a box that does
  not move by a single pixel is reported "camera frozen?". And the picture is OLDER than the read by an amount nobody
  has measured (re-check S6): a robot turning fast has moved since; anything that pairs a picture with where the robot
  was must allow for it (§1.9's `boundAtImage`), and Step 0.7 measures it.
- **Pixels become directions and distances, with their error.** From the measured mount (height, offset, pitch, yaw
  — each typed with how well it was measured) and the camera's published field of view, a box's column is a
  **bearing**, its row an **elevation**, and for a thing of known size or known height a **range** — every one an
  interval, never a single number. Where two quantities come from ONE camera (the Toggle's two edges; a Pin's base and
  its height), they are checked against each other **one camera at a time**: "is there a camera, inside the stated
  tolerance, that sees this box as a Toggle?" — never by overlapping two separately widened intervals, which
  double-counts the camera's pitch error and let half again as many lookalikes through (P4b: 129 vs 86 of 540).
- **The Toggle tracker says RED, BLUE, YELLOW — or UNKNOWN with the reason.** It believes a colour only when a long,
  flat band of that colour, at the Toggle's height (~12.6–14.4 in), of a Toggle's length, where a Toggle can be, has
  held for T seconds with the camera alive. Two colours that both look like a face; a face beside a band of ANOTHER
  colour the tracker cannot judge (cut by the picture's edge, or maybe a bottom face — re-check S7); a band with
  another Toggle colour directly ABOVE it (re-check R7, below); a band cut by the picture's edge; a camera that is
  full, silent or frozen — UNKNOWN; a camera that stops being frozen is believed again only after a NEW hold (a latch
  the 01:43 prototype had, found by P7). **The bottom face:** the Toggle's BOTTOM face (a different colour) is visible
  from a camera below it, and close in it looks almost exactly like the face that counts — on the host test plant an
  8-in camera reported the bottom face's YELLOW as "sure" 8 in from a RED Toggle. The tracker drops a band that could
  be the bottom face unless it can see where the real face would be (probe T1; mutation MT2), and a band no taller
  than its edges' jitter is not measured at all (the bottom face's far-off sliver — P7/T8). **A second way to the same
  wrong answer, found in this re-check (R7):** with the robot TURNED 15° to the Toggle, the face's band slants in the
  picture, stops looking like a face, and the thin bottom face was reported instead — fixed (a band with another
  Toggle colour directly above it is never reported); the cost is that close in the colour holds only when the robot
  is near-square to the Toggle. **What a colour means (re-check S2):** the colour of the face the camera SEES where a
  seated face would be — NOT that the Toggle is set: a Toggle stuck mid-roll can show the colour it is rolling toward,
  and nothing the camera measures tells seated from rolled, or whether a robot is touching it (`<SC4>`). **Where the
  camera sits decides what it sees (D9, re-check S1):** a lens ~10–11 in up reports the colour from ~70 in to ~12 in;
  a lens within about an inch of the face's lower edge (~11¾–12¾ in, level) is blind to it.
- **The piece finder takes ONE named piece or none.** A route says "the yellow Pin expected at (60, 0) within 5 in,
  standing on the floor". The finder answers FOUND (exactly one candidate certainly inside, none on the border),
  NONE (nothing of that type there — it may be gone, or hidden), AMBIGUOUS (two inside, one on the border, one it
  cannot measure, or its own error larger than the tolerance), NOT IN VIEW (the place is not wholly in the picture),
  or UNKNOWN (no fresh read, not held yet, the camera full, or the camera's AI model not confirmed by name — a read
  whose objects were withheld is never "nothing there", P7). Never the nearest of several. A Pin on a Goal or in a
  stack has its base above the floor: its box sits too high for its size, so it is RAISED, and a route that wants a
  floor Pin never takes it; a lying Pin is LYING. From 4 ft with a ±1° mount, two Pins a foot apart are AMBIGUOUS
  by design; at ~21 in the named one is FOUND (P5/F1) — drive closer, or measure the mount tighter.
- **The host camera model** draws Toggles (all their faces), Pins, Cups, licence plates and LED strips through a TRUE
  camera that differs from what the library believes (inside the stated tolerance), with jitter, dropped blobs,
  freezes, tears and over-full reads — written with its own geometry so a sign error in the library's cannot cancel
  out in the tests (the A2 rule).
- **What it does not do.** It moves nothing, flips nothing, drives nowhere: AF1 (the automatic flip), L4 (driving
  to a piece), L6 (the wall reset) and ALLY1 (our colour) use it later. It does not read AprilTags. It cannot make the
  camera immune to lookalikes: an opponent's red plate at bumper height near a wall, or red lights on the wall top,
  can still read as a Toggle when the position check is off (P4/S5b, P4b, P7/T10); it reports the face it sees, not
  whether the Toggle is seated or touched (S2) — which is why AF1 must have the face's colour corroborated AT THE PUSH
  POINT before any push, by a sensor that sees colour there (an Optical at the flipper); a side distance sensor says
  something is there, never which colour (re-check S5; F1, F10).
- **Step 0 at the robot comes first, and four of its steps need no new code** (§2.0): measure the mount; read which
  model the camera carries; teach the three colours in VEXcode's AI Vision Utility; hold a Toggle where the arm will
  push it and see whether it is in the picture. Step 0's answers decide the Optical question and the mount.

---

## 0. Why — what exists, what the camera offers, and the probes (every line cited was OPENED)

### 0.1 Why this chunk, and why now
- The team lead's rulings above (2026-10-08, 2026-10-14) put the camera before the panel: the Toggle colours for AF1,
  the OBJECTS for L4, both resting on one adapter. `build-order.md` :1666-1683 (the VIS1/AF1 entry, "### VIS1 and AF1") and
  :2176-2177 (the deviations table, both rulings) — at `2506067`; the MECH-A2 merge moves them (re-check T10); `REMAINING-WORK.md` :89-90 (rows 18d, 18e), :103 (row 24: R2b's objects part
  now in VIS1; its tags stay after G3), :105 (row 26: L4 after VIS1).
- The first competition is 2026-11-23 (the team lead, 2026-10-14; the coordinator's NOTES). Nothing in VIS1 is on that date's critical path (the
  competition track is robots, mechanisms on buttons, student autons — MECH-A5 is when autons can be written), so
  VIS1 is sized and split for correctness, not squeezed; Step 0's four no-code steps can run at any bench session.
- **No AI Vision evidence exists.** Stations H and I (Bench Tests 11/12) are "uploaded, not run"
  (`REMAINING-WORK.md` :166-167); session S6 ("vision on the bench", :168-169) was planned for after R2b and is now
  this brief's Step 0 + VIS1-B's worksheet station.

### 0.2 What exists in the tree (re-checked at the tip `2506067` — SER1 on `fd36e07`, MECH-A1 merged; "(A1)" marks MECH-A1's files, whose lines ARE the tip's — re-check T10)
| Piece | Where (opened) | What it gives VIS1 / what it lacks |
|---|---|---|
| The frozen camera seams | `include/shulib/hal/vision.hpp` :3-22 (decision #7: one adapter, both seams; detections by value, "OFF the 10 ms control hot path"; "the consumer timestamps"), :52-63 (`ObjectObservation`: `classId` opaque, a BEARING only, `confidence` read by nobody), :99-100 (`objects()` returns `std::vector`) | F4, LOCKED 2026-06-19 (`roadmap.md` :138). No type, no box, no time — VIS1 adds a sibling and changes nothing here (D11) |
| Tag geometry | `hal/vision_conversion.hpp` (`CameraMount` encodes HA-70's LEVEL camera — HA-70, `hardware-assumptions.md` :3473-3484) | Tags only; left untouched (OUT, §5) |
| The absent sources | `hal/absent_vision.hpp`; `robot.hpp` :543, :571, :1153 (`vision_{}`, `.vision = &vision_`, `AbsentVision vision_` — at the tip; :535/:563/:1157 were pre-A1, re-check T10) | Stays (D11) |
| The registry's kinds | `hal/device_registry.hpp` :50-62 (`DeviceKind::Vision` "including AI Vision — no adapter yet; R2 owns the camera"), :66-80 (`deviceKindWord`); `hal/pros/device_registry.hpp` :124 and :128 (`E_DEVICE_VISION` AND `E_DEVICE_AIVISION` → `Vision`) | The trap R2a named (`R2a-sensor-stations.md` :101-102, landmine 3) — D4 |
| `DeviceKind` users (re-check T5/T10: lines at the TIP `fd36e07`, A1 merged) | `hal/device_registry.hpp` :66-80 (`deviceKindWord`, a switch); `hal/pros/device_provider.hpp` :612-626 (`foundWords`, a switch), :632 (`expectKind`); `calibration/boot_check.hpp` :294, :348 (words); `src/programs/calibrate_program.cpp` :1111-1120 (comparisons), :1115 (a word); tests `pros_device_registry_adapter_test.cpp` :76, :139, :143, `boot_source_test.cpp` :159 (robot two's port 3 as `Vision`, commented "the AI Vision camera on 3"), `mechanism_devices_test.cpp` :63-66 (`kEveryKind`) and :69-82 (its OWN `foundWords`, a switch) | Every use is a comparison or a word: no record stores the byte (`calibration_record.hpp` :200 stores the DRIVETRAIN kind) — appending an enumerator is safe. **Compiled (re-check, `v6/overlay/`): with `AiVision` appended, exactly THREE exhaustive switches break under `-Wswitch -Werror` — `deviceKindWord`, the provider's `foundWords`, and the test's own `foundWords`; all 218 test files compile once each gains its case** |
| The device provider | `hal/device_provider.hpp` (A1) :142-222 — `distance()`/`optical()` :178/:183, `readFaults(port)` :196, `lineReadFaults` :201, `lastRefusal()` :222; four implementers: `hal/pros/device_provider.hpp` (A1) :183 (`ProsDeviceProvider`; `distance()` :441-463 is the model — range check, cache by port, `expectKind`, construct in a `try`, refuse naming the port), `hal/fake/fake_device_provider.hpp` (A1) :103, and the two forwarding providers in `test/robot_builder_test.cpp` (A1) :1782 (`SilentRefusal`), :2170 (`SilentLine`) | VIS1 adds ONE accessor in A1's style and answers `readFaults(cameraPort)` |
| The read-fault channel | `hal/read_faults.hpp` (A1) :1-61 — `IReadFaults::faultedReads()`, monotonic, "a change means a value it was given in between was held" | A frame cannot be held (§1.1), but the camera adapter implements it too, so a consumer written for A1's channel works unchanged |
| The robot file | `config/robot_config.hpp` (A1) :325-332 ("TAGS AND VISION ARE DELIBERATELY ABSENT … no camera adapter exists (R2 owns it)"), `SensorsConfig { GpsConfig gps; }`; A1 appends `PortUse::MechanismSensor` and claims sensor ports through the one-owner `claim` (MECH brief §1.5, ~:1413-1417) | VIS1 appends `CameraConfig camera{}` (D2) and `PortUse::Camera` |
| The Robot's boot line | `robot.hpp` (A1) :770, :787, :798 — "… GPS %s tags ABSENT vision ABSENT" | VIS1 words the camera's state there |
| Bench Tests station 12 | `src/programs/bench_r3a.cpp` (this row's lines are `fd36e07`'s: +4 at `2506067` after ~:180 — SER1, re-check T10) :31-47 (it "INTERPRETS NOTHING … the AI Vision adapter (R2b) is written afterwards from these records"), :2135-2148 (tag families), :2184-2197 (`className`), :2200-2213 (`CamSnap`, `snapCamera` — count, then `get_object(i)` up to 24), :2243-2275 (`decode`, fields copied out of the packed struct), :2332-2379 (`ChangeTimer`, the only frame-rate probe), :2384-2419 (`setUpCamera`: TAGS+OBJECTS on, overlay on, "colours left as found"), :2422-2444 (`logClassNames` — the class-name table logged at entry), :2632-2877 (`aiVisionStation`) | Steps 0.2 and 0.4 need NO new code (§2.0); VIS1-B gives it colours (D3) |
| Robot two's sensors | `R2a-sensor-stations.md` :14-17 (camera port 3 "front, level, roughly centred"; distance 4 left, 5 right); `src/robots/chassis_table.hpp` :69-91 (`ChassisTable`; `imuPort` :76, robot two's `.imuPort = 2` :131) | The camera's port and mount go in the table (D2, VIS1-B) |
| Our colour | `diag/session_info.hpp` :50 (free text); every program sets `""` (research `lib` §5) | ALLY1 (D8) — not VIS1 |

### 0.3 What the vendored PROS 4.2.2 API gives, and what it does not (`include/pros/ai_vision.h`, read whole; `ai_vision.hpp` :39-597)
- **Modes** `TAGS 1, COLORS 2, OBJECTS 4, COLOR_MERGE 16` (:70-76). `aivision_set_enabled_detection_types(port, bits,
  mask)` (:252) — its ONLY semantics are the doc's one example (:242-244: bits `0b010`, mask `0b101` "would disable
  the detection of tags and objects, and leave the setting of colors alone"): masked bits are set from `bits`, the
  rest kept (HA-196). `enable_`/`disable_detection_types` (:278, :303); the read-back `get_enabled_detection_types`
  (:232).
- **Colours** `aivision_color_s_t {id 1-7, r, g, b, hue_range 1-40, saturation_range 0.1-1}` (:85-92);
  `aivision_set_color` (:352; "The color detection type must be separately enabled"; a second write to an id
  "will be overwritten", :340); `aivision_get_color` (:369 — "If you attempt to get a color configuration that has
  not been previously used, the behavior is not defined", :355-357). Codes (:102-110, :421-450) exist; VIS1 sets
  none.
- **Reads** `aivision_get_object_count` (:463) then `aivision_get_object(i)` (:479; "an invalid object type if the
  operation failed", :476-477) — **two calls per detection's worth of state, not one atomic frame** (HA-195). At most
  `AIVISION_MAX_OBJECT_COUNT` 24 (:33). A record is `{id, type, union {color: x, y, w, h, angle (tenths of a degree);
  tag: 4 corners; element: x, y, w, h, score}}`, packed (:128-175); `xoffset` is the "left edge (from camera's
  view)", `yoffset` the "top edge" (:129-130 — HA-193).
- **No frame counter, no timestamp, no frame rate, no field of view, no range** anywhere in the header (finding F5;
  research `lib` §1a). The only freshness a program can know is when ITS read finished (HA-192).
- **The AI model cannot be chosen** — nothing in the C or C++ API selects one. **Its class names can be read, one id
  at a time, but their COUNT cannot (re-check, T1 — read in the BINARY, not the header):** the header says id -1
  "will return the number of class names" (:375), but `aivision_get_class_name` in the vendored `firmware/libpros.a`
  (`vdml_ai_vision.c.o`, disassembled with `arm-none-eabi-objdump -d`; kernel 4.2.2, `project.pros`) is `bl
  vexDeviceAiVisionClassNameGet` → `port_mutex_give` → `movs r0, #1`: the SDK's value is DISCARDED and every id, -1
  included, returns `PROS_SUCCESS` (`error.h` :40); `PROS_ERR` only when the port claim fails (errno EACCES). PROS's
  own C++ wrapper treats exactly 1 as success (`get_class_name[abi:cxx11]`: `cmp r0, #1`). What the SDK writes for
  an id the camera does not name is unknown — so a reader zeroes its buffer (buffer "length >=20") and reads ''
  there. Names "may not correctly report … for the first several hundred milliseconds of being plugged in"
  (:371-374). **The same method settles two smaller beliefs** (T4): a failed `aivision_get_object` returns `{id 127,
  type 127, the rest 0}` (the record is zeroed and both bytes set to 127 BEFORE the claim), a failed
  `aivision_get_color` returns id/R/G/B 127 and both floats +Inf (0x7f800000); `aivision_set_enabled_detection_types`
  passes `(bits, mask)` straight to `vexDeviceAiVisionEnableSet`, `enable_` passes `(t, t)` and `disable_` `(0, t)`,
  while the overlay, white balance and tag family go through `vexDeviceAiVisionModeSet` — so the all-modes MASK
  write cannot disturb them; every setter returns 1 whatever the SDK did (the read-back is the only check).
  api.vex.com (V5 C++ AI Vision Sensor page, fetched 2026-10-15 ~10:07): "The sensor processes visual information
  using an onboard AI model selected in the AI Vision Utility within VEXcode"; "When using VS Code, the AI model must
  first be configured in VEXcode before it can be used in your program"; Override 0 `redBluePin`, 1 `blueYellowPin`,
  2 `redYellowPin`, 3 `yellowPin`, 4 `cup`; Classroom 0 `blueBall` … 3 `blueRing` … 8 `redCube`; 320 × 240,
  `originX/Y` the box's top-left; objects "ordered from largest to smallest (by width)" in one paragraph and "(by
  area)" in another; a score "between 1 and 100" (VEXcode's — PROS's raw `uint16_t score` scale is unstated, HA-198).
- **Never to be called by the library:** `aivision_reset` (:209 — "the initial state" is undocumented and might drop
  the model the Utility chose), `aivision_start_awb` (:418 — a white balance taken mid-match moves every colour the
  team measured), any tag-family call (no tags), `aivision_set_usb_bounding_box_overlay` (:404 — a bench aid).
- **The C++ wrapper is not used:** `pros::AIVision::get_all_objects()` returns `std::vector` (`ai_vision.hpp` :595 —
  allocates), and the header includes `<expected>` (:26), a C++23 header that is empty under the tree's `gnu++20`.
  The adapter binds the C API exactly as station 12 does (`bench_r3a.cpp` :88, :2207-2216 — `snapCamera`).

### 0.4 The probes (`scratch/brief-VIS1/v4/`; every one rebuilt from source and run by this writer, 2026-10-15)
Built with the suite's flags (`-std=c++20 -Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion
-Wdouble-promotion`, `test/CMakeLists.txt` :14, :22-24), the host shim first on the include path as the suite does
(:117-118), doctest from `tree/test/vendor`. `proposed/` holds the seven prototype headers and the shim (2,227 lines) the parts
below copy from; `probes/` the probes; `out/` every output.

| Probe | What it checks | Result |
|---|---|---|
| **P1** `p1_adapter.cpp` | `ProsAiVision` over the shim (`proposed/shim/pros/ai_vision.h`, adversarial: TAGS+OBJECTS on as station 12 leaves them, a stray descriptor in slot 1, VEX's CLASSROOM model): exactly COLORS+OBJECTS written with the all-modes mask, TAGS off; descriptors written and read back; never `reset`/`awb`/tag calls; a count of 24 or more truncates (re-check S4; v4 said "> 24"); a torn read is `PartialFault`; a failed count is `ReadFault` with NO detections and the set-up re-written (rate-limited); the 1 Hz read-back finds settings changed under the program; the model check BY NAME (Classroom → objects withheld; one wrong name of five caught; an unreadable table never counts as a match). **Re-check (v6):** the shim answers as the vendored BINARY does (`get_class_name` → 1 for every id; failures 127/127, 127/+Inf) and the check reads names only — P1.15–P1.19 added (no model → `Different`; id -1 never asked; an id past the table withheld; the binary's failure records; a colour write that stores nothing) | v4: 14 cases, 10 mutations RED. **v6: 20 cases pass (P1.20, the cap, re-check S4); 14 mutations (M1–M14) each RED** (`v6/out/p1_mutations.v6b.txt`; M13 survived until P1.19) |
| **P2** `p2_geometry.cpp` | bearing/elevation by hand anchors; 400 random placements: every true point inside the stated interval (widest fix radius 37.1 in); ground range at row 200, 8 in, pitch 5°: ±1° → [21.5, 31.3] in, ±3° → [19.4, 36.4]; the "whole face in frame" table | 6 cases, 17 checks pass |
| **P3** `p3_mount_table.cpp` | D9's table: for lens height × pitch, the nearest lens-to-wall distance with the WHOLE face in frame, and a Pin at 24/48 in: its range interval and fix radius, typical vs tight tolerances | `out/p3_mount_table.txt` (D9 quotes it) |
| **P4** `p4_toggle_scenes.cpp` | the tracker against the INDEPENDENT host camera model: approach from 72 in (8-in level camera: RED from 72 to 14 in, then UNKNOWN "bottom face?" / "cut" / "no band"), the length gate, two colours, a frozen camera while moving, unplugged, an LED strip (S5b: read RED from 20 in with the position gate OFF), a short strip mid-field (position gate ON refuses it), the position gate on a real face (S13: RED from 28 in to 14 in), a Toggle to the side, a band at 9–11 in | 17 cases, 220 checks pass |
| **P4b** `p4b_sweep.cpp` | 540 red lookalike bands (heights 4–18 in, lengths 4–26 in, 10–48 in away) 30 in in front of a wall: how many read RED | final prototypes: **75** with a 96-in window, **46** with 36 in, **0** with the position gate (pose ±2 in / 2°) — **v6, after re-check S10's narrowed lower edge: 65 / 39 / 0**; before the minimum band height (below): 86 / 46 / 0; with the two edges' intervals OVERLAPPED instead of tested corner by corner (v2's prototype, `v2/out3/p4b_sweep.txt`): 129 / 88 / 5 — why §1.9 tests per corner. **Without a trusted pose, the camera alone lets lookalikes through** — the honest limit AF1 designs around (F10) |
| **P5** `p5_piece_scenes.cpp` | the finder: a named yellow Pin with a second one a foot away, approached from 53 in (AMBIGUOUS "error larger than the tolerance" to ~25 in, FOUND at 21 in r 4.1 in, NOT IN VIEW at 17 in); nothing there; a Pin on a Goal (Raised); a lying Pin; out of view; two inside | 9 cases, 16 checks pass |
| **P6** `p6_v3_scenes.cpp` | the WHOLE Toggle (field face RED, bottom face YELLOW): an 8-in level camera never reports YELLOW (T1); a truncated read → UNKNOWN "camera full" (T2); a fault's reason; the camera pitched up (T5: last BLUE at 14 in level, 7 in at +15°); the camera's height (T6: 12¾ in level → last RED at 3½ in; 10 in +10° → 6½; 8 in level → 13½); the finder's unmeasurable second candidate, truncated read, a Pin's top out of view | 11 cases, 370 checks pass |
| **P7** `p7_v4_checks.cpp` (NEW, this run) | **T7:** a camera that freezes while the robot drives, then recovers; **P7:** a read whose Object detections were withheld (the model check says `Different`); **T8:** the whole Toggle approached by an 8-in camera — RED unbroken from 48 to 14 in (lower edge 12.36) and from 56 to 14 in (12.61); **T9:** a 12¾-in camera, the true Toggle equilateral — RED in to 4 in; **T10:** a VERTICAL red plate exactly at the face's heights on the face's line | **T7 and P7 FAILED on the 01:43 prototypes:** T7 never recovered (the `frozen_` flag latched until the colour changed); P7 answered **NONE "nothing there"** — a confident negative from a read that carried no object evidence at all. **T8 failed** once the lower edge became an interval (the bottom face's 1–2-px sliver at 36–48 in passed as a far face → "two colours"): fixed by a **minimum band height** — a band no taller than its two edges' jitter (2 × `kBoxEdgePx`, HA-202) cannot be measured as a face (it also cut P4b's 86 to 75). All fixed in `v4/proposed/` (the tracker restarts the hold at the first moved box; the finder answers UNKNOWN "model not confirmed" unless the frame's `model` is `Matches`; the host camera model reports the model check and withholds objects as the adapter does). Final: T7 recovers after 5 reads (holdSeconds 0.15 s at 33 ms), P7 UNKNOWN, T8 passes, T9 last RED at 3 in, **T10: the plate is never a colour from 14 in or closer, but reads RED from 16 to 24 in** — the face's lean is visible only close in (**v6, S10: RED only at 22 and 24 in**). 5 cases, 142 checks pass |
| **P8** `p8_equilateral_truth_p6.cpp`, `p8b_…_p4.cpp` (NEW) | P6 and P4 with the TRUE Toggle an equilateral prism (lower edge 12.61 in, apex 1.02 in behind it) | All 28 cases pass (590 checks), on the final prototypes AND on the single-value 01:43 tracker; what moved is T6's 12¾-in camera (6 in with one value, 3 in with the interval) — D9 amended, T9 pins it |
| **Mutations** `mutate_all.sh` | 32 one-line breaks of the tracker (MT1–MT18), the finder (MF1–MF9, MF1b), the geometry (MG1–MG4), each rebuilt against every probe that should catch it | **32 of 32 RED** (`out/mutations_all.final.txt`) — MT14–MT18 and MF9 are this run's (the latch; recovery keeping the old hold; the finder ignoring the model check; no minimum band height; the lower edge back to one value; the set-back's lower bound dropped). MT17 and MT18 SURVIVED until T9 and T10 were written — a design choice with no test is not done |
| **Headers** | the 7 proposed headers + the 215 of `mech-a1`'s tree on the ARM gate (RESUMING :107-112, `arm-none-eabi-g++ 13.2.1`); the PROS-free ones on the host; the adapter over the shim on the host; the CI guards' greps (no `pros/` include outside `hal/pros/`; nothing outside `sim/` includes `sim/`) | **222 headers, rc 0**; host rc 0; rc 0; both greps clean |
| **Re-check (v6)** `scratch/brief-VIS1/v6/` | the two checkers' findings, each turned into a probe that FAILS on the v4-era prototypes and passes on v6's: **R1** (can the camera measure a face's lean? no — S2), **R2** (S3's turn, S3b's lens, S4's cap on the host model, S7/S7b's partial-band veto, S8's side-clipped bands), **R3** (the seam's `status()`, the fake, the feed — T2/T3), **R4** (a Toggle rolled −120..+120° — S2), **R5** (D9 scored by the tracker — S1), **R6** (the image's age — S6), **R7** (the robot turned to a Toggle — R7, found here), **P4/S15b** (the window on whole faces — MT12); the checkers' own `c1/c3/c6*/c7*` and `classname_check` rebuilt and re-run (`v6/out/`; the pre-session outputs in `v6/out/pre-s/`) | **14 suites, 114 cases pass**; 41 + 14 mutations RED (`v6/out/mutations_all.v6b.txt`, `mutations_recheck2.txt`, `p1_mutations.v6b.txt`); **225 headers rc 0** on the ARM gate over `2506067` (`v6/armgate2/`); the checkers' C1/C2/c7b now pass; C3 falls 56 → 35; C4 and C5 still fail AS WRITTEN — they bypass the fixes (a hand-built `truncated = false`; a pose not widened) — the fixes' own tests are P1.20/R2.S4 and R6 |

**What the probes cannot show** (each is a §4 belief, settled only at the robot): that PROS behaves as the shim
says (HA-195/196/197); the optics (HA-194); the colour classifier (the host camera model fires a descriptor because
the scene says so — HA-199); occlusion, lighting, motion blur (not modelled, stated in the model's banner); that
the model's box encloses the whole piece (HA-200).

### 0.5 Found while reading (each handled in scope or named)
1. **F1's arithmetic is for the face's LOWER edge;** the WHOLE face needs its apex (14.39 in): a level camera at
   height h sees the whole face only from (14.39 − h)/tan 31.5° away — 13.7 in at 6 in, 10.4 at 8, 7.2 at 10, 3.9 at
   12 (nominal; P2.5/P3 add the tolerances: 15½, 12, 8½, 4¾). Step 0.4 uses the whole-face figure.
2. **The Toggle's face span is arithmetic, and the glossary settles which reading** (HA-201; re-check S10): the
   glossary says "each face of the triangle is approximately 2.05” (52.2mm) wide" (v20.txt :4668), and pA8's
   axle-to-apex 30.2 mm is the circumradius of a 51.6–52.2-mm equilateral (29.8–30.1 mm) — an equilateral prism
   resting on a face, whose face spans 1.74–1.78 in vertically: the lower edge ≈12.61 in. v4's other reading, 12.36 =
   14.39 − 2.03 (pA8's 51.6-mm end view read as a VERTICAL size), implies 2.34-in sloped faces, against the glossary —
   dropped. `override_elements.hpp` carries the lower edge as the INTERVAL **[12.55, 12.65]** and the set-back (the
   apex behind the lower edge, side/2 ≈ 1.03) as **[0.9, 1.2]** in (VIS1-A1; prototyped and probed — P7/T8–T10,
   MT17/MT18, R5; narrowing cut P4b's lookalikes 75 → 65 of 540 and T10's plate from 5 to 2 distances, every probe
   case still passing; v4's 12.36 kept as an OUT-OF-BELIEF truth in R5: never a wrong colour), and Step 0.1 measures
   a real Toggle if one is reachable; a measured Toggle collapses both intervals.
3. **The bottom face** (P6/T1): the prism rests on a face, horizontal at the lower edge's height, a different
   colour, visible from below to a camera under it. The host camera model draws no wall, so a real wall top
   (11.50–11.54 in, pA13/pA14) may hide part of it — the probes are the worst case.
4. **api.vex.com contradicts itself on the read order** (width vs area) and VEXcode's score is 1–100: neither is
   PROS's statement (HA-198). Nothing in VIS1 depends on either: a truncated read is UNKNOWN (D6), and the score
   floor is the team's number from Station I.
5. **The class names as the camera spells them are unread** — `kOverrideClassNames` is VEX's API spelling. A
   different spelling on the camera fails SAFE (objects withheld, the boot line says which name differed); Step 0.2
   reads the camera's own spelling from station 12's RECORD lines (each OBJECT line carries `class '…'` for its id,
   `bench_r3a.cpp` :2320-2327 — one piece of each type in view gives all five; the entry log gives class 0 only, T1)
   and the executor copies the names from those lines if Step 0.2 has run (HA-197).
6. **`ai_vision.hpp` includes `<expected>`** (:26) — harmless under `gnu++20` (an empty header), but a reason never
   to include the C++ wrapper from a library header.

---

## 1. Scope — IN

**What is prototyped, and what is not (re-check T3 — v4's "every shape below is in `v4/proposed/` and compiles" was
not true).** PROTOTYPED AND COMPILED (`v6/proposed/`, host `-Werror` with the suite's flags AND the ARM header gate,
225 headers rc 0): `hal/vision_frames.hpp`, `hal/pros/ai_vision.hpp`, `hal/fake/fake_vision_frames.hpp`,
`config/camera_config.hpp`, `vision/{camera_geometry,override_elements,toggle_tracker,piece_finder,camera_feed}.hpp`,
`sim/camera_plant.hpp`, the shim. HOOKS INTO A1's FILES, compiled as an overlay of the tip (`v6/overlay/`: every one
of the 218 test files `-fsyntax-only` with the suite's flags, and robot two's three chassis tables on ARM): `DeviceKind
::AiVision` + its three switch cases + the registry mapping; `SensorsConfig::camera`; `PortUse::Camera` + its
`portUseWord` case; `ChassisTable::camera{}`; the test's `kEveryKind` and `foundWords`; `boot_source_test.cpp` :159.
NOT PROTOTYPED (each built from A1's own pattern, which the run re-reads at its tip): the provider's `aiVision()` in
the four implementers, `RobotConfig::valid()`'s camera checks, the Robot's `camera()`/`cameraAbsence()` and boot
line, `boot_source`'s copy, `src/programs/bench_camera.hpp` and station 12's edits. Each run copies its slice, strips every "PROBE" marker, gives
every public member its own `///` line (`api_doc_tool.py check-coverage` fails the BUILD on an undocumented entity,
`test/README.md` ~:20-27), and re-reads every cited file at its own tip.

### 1.1 `include/shulib/hal/vision_frames.hpp` — NEW: the frame seam, beside the frozen `IVision` (VIS1-A1)
An **F4-additive sibling**, exactly as `IDigitalOut` (F11), `IDigitalIn` (F14) and `IReadFaults` (MECH-A1) were;
`hal/vision.hpp` is untouched byte for byte. PROS-free, allocation-free. Verbatim from `v6/proposed/` (the re-check's prototypes supersede v4's — every §1 pointer below is to `v6/proposed/`; v4's line ranges dropped) (v4 :28-189):
- `kMaxVisionDetections = 24` (restated PROS-free; the adapter `static_assert`s it equals `AIVISION_MAX_OBJECT_COUNT`),
  `kMaxColourDescriptors = 7`, `kVisionImageWidthPx = 320`, `kVisionImageHeightPx = 240` (HA-193).
- `enum class DetectionKind : std::uint8_t { Colour = 1, Code = 2, Object = 3 }` — OURS, not PROS's bit flags; **no
  Tag** (a tag that arrives anyway is counted in `skipped`, never passed on).
- `struct Detection { kind; id; left; top; width; height; angleTenths; score = -1; constexpr bool clipped(int
  marginPx = 1) const; }` — image pixels, +x right, +y DOWN, the box's left/top edges + size (HA-193).
- `enum class FrameStatus { NotRead, Ok, ReadFault, PartialFault, NotConfigured }` — **only `Ok` is evidence.**
- `enum class ModelCheck { NotAsked, Matches, Different, Unreadable }`.
- `struct VisionFrame { status; units::Time readAt; std::uint32_t readSeq; std::int32_t reportedCount; bool
  truncated; std::uint8_t count, skipped; ModelCheck model; std::uint8_t objectsWithheld; Detection
  detections[24]; constexpr bool usable() const; }` — **`readAt` is when the READ COMPLETED, `readSeq` the
  adapter's own poll count — never a camera frame number** (F5, HA-192); **the IMAGE is older than `readAt` by an
  unmeasured age** (HA-192, re-check S6 — a consumer pairing a frame with a pose widens the pose's bound for it,
  §1.9). **`truncated` = "possibly full": the camera reported 24 or more** (re-check S4, D6). A failed read is a frame
  that SAYS it failed, with NO detections — never the last good frame held (a held frame would read as a fresh
  "nothing changed").
- `struct ColourDescriptor { id; red; green; blue; hueRange; saturationRange; const char* name; const char*
  provenance; }` + `constexpr const char* colourDescriptorProblem(const ColourDescriptor&)` — **an all-zero
  descriptor is UNSET and refuses** ("UNSET -- measure it with VEXcode's AI Vision Utility (Step 0.3) and type it");
  id outside 1..7, hue outside 1..40, saturation outside 0.1..1, an empty provenance — each refuses by name; nothing
  inside the vendored ranges is refused (`ai_vision.h` :85-92).
- `struct AiVisionSettings { ColourDescriptor colours[7]; colourCount; bool objects; const char* const*
  expectedClasses; expectedClassCount; bool colourMerge; }` + `aiVisionSettingsProblem()` — more than 7 colours, a
  colour problem, two colours on one id (the camera silently keeps the second, `ai_vision.h` :340), COLOR_MERGE with
  no colours, **nothing turned on**, **objects on with no expected class table** — each refuses.
- `class IVisionFrames { … [[nodiscard]] virtual VisionFrame read() noexcept = 0; [[nodiscard]] virtual CameraStatus
  status() const noexcept = 0; }` — read() HITS THE DEVICE (1 + N calls, HA-203), so it is not const; single-task by
  contract. **`status()` (re-check T2) touches nothing:** `struct CameraStatus { bool configured; const char*
  setupProblem; ModelCheck model; const char* modelProblem; }` — the adapter's own words, so the Robot's boot line
  and a bench page can report the set-up and the model check through the SEAM (the builder never reads a frame, and
  must not downcast to the PROS adapter). `constexpr const char* cameraStatusWords(const CameraStatus&)` gives the
  ASCII words: "NOT set up", "set up, objects off", "set up, model OK", "set up, model check pending" (`Unreadable`
  — re-asked at 1 Hz), "set up, model DIFFERENT" (prototyped in `v6/proposed/…/hal/vision_frames.hpp`; probe R3.1).

### 1.2 `include/shulib/hal/pros/ai_vision.hpp` — NEW: `ProsAiVision`, the adapter (VIS1-A1)
`final : public IVisionFrames, public IReadFaults`, over the C API (§0.3), constructed by the provider only
(`v6/proposed/…/hal/pros/ai_vision.hpp`). The rules, each a §2 test and most a mutation:
1. **Settings are WRITTEN, never trusted** (HA-196): at construction; again after ANY failed count read (a
   re-plugged camera may come back without them); again when the 1 Hz read-back of the modes disagrees (counted in
   `settingsLost()`). The mode write uses the ALL-MODES mask (TAGS|COLORS|OBJECTS|COLOR_MERGE) so it sets EXACTLY
   the wanted modes — **TAGS off** — whatever station 12 or VEXcode left on (the shim starts with TAGS+OBJECTS on).
   Each descriptor is written, then read back (id and RGB compared; the two floats NOT compared — whether the camera
   stores them exactly is HA-196's question, and refusing on a rounding would refuse a working camera).
2. **Until a set-up succeeds the camera reads `NotConfigured`** — never a frame from a camera configured by someone
   else; a failed set-up is retried at most every 0.5 s (`kRetrySeconds`; the camera may still be booting).
3. **The read:** the count (PROS_ERR or < 0 → `ReadFault`, no detections, `faultedReads()` +1, the set-up
   re-done); a count of **24 or more** → `truncated` ("possibly full" — re-check S4, D6), reads 0..23; each `get_object(i)` copied BY VALUE (never a pointer
   into the packed record — R2a landmine 2); an invalid type or a TAG → `skipped`, and a read with any skip is
   `PartialFault` (+1 fault) — a torn read is not evidence (HA-195).
4. **The AI model, checked BY NAME** (D7, HA-197): with `objects` on, the name of each EXPECTED id (0 ..
   `expectedClassCount`−1) is read into a zeroed buffer — **never the count: PROS 4.2.2 returns 1 for id -1** (§0.3,
   re-check T1) — and compared with `expectedClasses`: `PROS_ERR` → `Unreadable`, an empty or wrong name →
   `Different`; until it says `Matches`, every Object detection is WITHHELD (`objectsWithheld`), and an Object whose
   id is past the expected table is withheld always (its name was never read); colours still pass; re-asked once a second while it does not match (names "may not
   correctly report … for the first several hundred milliseconds", `ai_vision.h` :373-374) and again after every
   re-set-up (a re-plugged camera may carry another model). `model()`, `modelProblem()` ("class 3 is 'blueRing',
   the robot file expects 'yellowPin'").
5. **Never called** (pinned by a source test, as `test/` pins PROS-free text elsewhere): `aivision_reset`,
   `aivision_start_awb`, `aivision_set_tag_family*`, `aivision_set_usb_bounding_box_overlay`, any `aivision_set_code`.
6. Accessors: `status()` (the seam's, T2 — `{configured_, problem_, model_, modelProblem_}`), and the adapter's own
   `configured()`, `setupProblem()`, `setups()`, `settingsLost()`, `model()`, `modelProblem()`, `faultedReads()`
   (monotonic, A1's contract).

### 1.3 `include/shulib/hal/fake/fake_vision_frames.hpp` — NEW (VIS1-A1)
A settable `IVisionFrames` + `IReadFaults` IN ONE OBJECT, as the adapter is, for provider, Robot and consumer tests
(the F4 fakes stay unmodified): `setNext(frame)` / `setFailing(bool)` (a failing read returns `ReadFault` with no
detections and counts), `setStatus(CameraStatus)` (the boot line's states are tested through it — T2), `reads()`,
`faultedReads()` (fault frames returned — it moves WITH the frames). Prototyped and compiled
(`v6/proposed/…/hal/fake/fake_vision_frames.hpp`; probe R3.2).

### 1.4 `test/pros_shim/pros/ai_vision.h` — NEW: the host shim (VIS1-A1)
`v6/proposed/shim/pros/ai_vision.h`: every belief transcribed from the vendored header with its line;
**adversarial defaults** — TAGS+OBJECTS on (what station 12 leaves), a stray descriptor in slot 1, VEX's CLASSROOM
class table (whose id 3 is `blueRing`); knobs for a disconnected camera, a mode write that changes nothing, a colour
write that fails, a colour write that stores nothing, a count of 24 or more, a torn read from index k, class names that
fail; call counters for every forbidden call and for id -1. **The shim answers as the vendored BINARY does where the
header and the binary differ (re-check, T1/T4), citing the disassembly:** `get_class_name` returns 1 for every id
and writes nothing for an id the table lacks; a failed `get_object` is `{127, 127, 0…}`; a failed `get_color` is
127s and +Inf. Its banner says the shim tests the adapter against OUR BELIEF about PROS, never the belief (the
existing shims' wording, `test/pros_shim/pros/optical.hpp` :16-17). CAL1-A1 is adding C shims beside it
(`pros/misc.h`, `motors.h`, `screen.h`, `main.h` — `cal1-a1` worktree): follow whatever pattern has landed.

### 1.5 The registry learns the AI Vision sensor (VIS1-A1, D4)
- `hal/device_registry.hpp`: APPEND `AiVision` after `Other` ("the V5 AI Vision sensor — hal/pros/ai_vision.hpp");
  `Vision`'s doc becomes "the OLDER V5 Vision sensor (276-4850) — no adapter"; `deviceKindWord(AiVision)` =
  "AI-VISION" (station 12 and the worksheet already say AI-VISION, `bench_r3a.cpp` :512, :2656, :2671).
- `hal/pros/device_registry.hpp` :128: `E_DEVICE_AIVISION` → `AiVision`; `foundWords` (`hal/pros/device_provider.hpp`
  :612-626 at the tip — re-check T10) gains "an AI Vision sensor".
- Tests: `test/pros_device_registry_adapter_test.cpp` :143's row becomes `AiVision`/"AI-VISION", and a new row pins
  that the older sensor is still `Vision`/"VISION" and is REFUSED by the camera accessor by name.

### 1.6 The provider's camera accessor (VIS1-A1) — in all four implementers, A1's style
- `hal/device_provider.hpp`: `[[nodiscard]] virtual IVisionFrames* aiVision(std::uint8_t port, const
  AiVisionSettings& settings) = 0;` and `readFaults(port)`'s doc names the camera too.
- `hal/pros/device_provider.hpp`: like `distance()` (A1 :441-463): a port outside 1..21 refused; a settings problem
  refused with its sentence ("port 3: the camera's settings: …"); `expectKind(p, DeviceKind::AiVision, "an AI
  Vision sensor")` (the older Vision sensor refused by name); constructed inside a `try` with the provider's clock;
  cached by port — **the same object for a second request with EQUAL settings (field by field, strings by content),
  a refusal for different ones** ("port 3: the camera was already set up with other settings — one owner"); and
  `readFaults(port)` returns the camera where it made one. **Constructing it WRITES the camera's settings** — say
  so beside the IMU's "calibration starts here" note (`hal/device_provider.hpp` :91-100, the interface's banner —
  re-check T10).
- `hal/fake/fake_device_provider.hpp`: its own `FakeVisionFrames` per port (`fakeCamera(port)`), refused like its
  other sensors on a refused port. **`readFaults(port)` answers the `FakeVisionFrames` ITSELF, and only once
  `aiVision()` has SERVED it (nullptr before) — a deliberate difference from A1's distance/optical pattern (re-check
  T9):** A1 pairs each F4 fake (left unmodified) with the provider's own `FakeReadFaults` (`readFaults_[port]`,
  `fake_device_provider.hpp` :401-408, :477), because an F4 fake cannot count; the camera's fake IS an `IReadFaults`,
  as `ProsAiVision` is, so its count moves with the fault frames it returns — a separate counter would be a second
  knob a test must keep in step by hand. A1's "only where served" rule holds; the provider test pins nullptr-until-
  served for the fake as §2.2 does for the real provider, and that a failing fake camera moves `readFaults(3)`.
- `test/robot_builder_test.cpp` (A1) :1782 `SilentRefusal`, :2170 `SilentLine`: one forwarding line each.

### 1.7 `include/shulib/config/camera_config.hpp` — NEW, and its hooks in `config/robot_config.hpp` (VIS1-A1, D2)
```cpp
struct CameraPlacement { bool measured; units::Length forward, left, height; double yawDeg, pitchDeg;
                         units::Length positionTolerance, heightTolerance; double yawToleranceDeg,
                         pitchToleranceDeg; const char* provenance; };          // moved here from the prototype's
constexpr const char* cameraPlacementProblem(const CameraPlacement&);           // vision/camera_geometry.hpp
struct CameraConfig { std::uint8_t port = 0;                // 0 = this robot has no camera
                      hal::AiVisionSettings settings{};     // what the adapter writes (§1.1)
                      CameraPlacement placement{};          // UNSET until Step 0.1 measures it
                      const char* note = ""; };             // e.g. "front, centred"
```
(The placement is DATA the robot file carries and G3a writes, so it lives in `config/`; `vision/camera_geometry.hpp`
includes it.) `SensorsConfig` APPENDS `CameraConfig camera{};` (its banner :325-332 rewritten: the camera is now
declared here; tags still absent); `PortUse` APPENDS `Camera` (and `portUseWord` gains "the AI Vision camera" —
its switch is exhaustive, re-check T5). `RobotConfig::valid()`: with `port != 0` — the port
claimed through the one-owner `claim` (a motor or sensor typed on the camera's port is refused naming both); the
settings' problem refused; a placement that is SET (`measured`) but has a problem refused. **An UNSET placement is
NOT a config error** — the camera is built and read, and every geometry consumer refuses to construct
(`vision::cameraModelProblem(cfg)` names it) — so Step 0 can proceed a piece at a time (objects first, colours
after Step 0.3, the mount after Step 0.1). Every config valid before stays valid: the new fields are 0 by default.

### 1.8 `robot::Robot` builds the camera (VIS1-A1, D5)
When `sensors.camera.port != 0`: `provider.aiVision(port, settings)`. **A refusal never refuses the build:**
`camera()` is nullptr and `cameraAbsence()` carries the provider's own sentence ("port 3: nothing plugged in
(expected an AI Vision sensor)"); `cameraAbsence()` is "" EXACTLY when `camera()` is non-null, "no camera is
declared" for port 0 (the mechanism accessors' contract, `robot.hpp` (A1) :646-696). The boot line's "vision ABSENT"
(:770, :787, :798 — A1's, now the tip `fd36e07`) becomes "camera ABSENT" or "camera port 3 " + the seam's
`cameraStatusWords(camera()->status())` — e.g. "camera port 3 set up, model OK", "camera port 3 set up, model check
pending", "camera port 3 set up, model DIFFERENT (class 0 is 'blueBall', the robot file expects 'redBluePin')",
"camera port 3 NOT set up (…)" — read through `IVisionFrames::status()` (re-check T2), ASCII only (re-check T10: the
brain screen's `screenText()` turns any non-ASCII byte into '?', `src/common/screen_text.hpp` :14-16, :71); "tags
ABSENT" stays. `RobotContext::vision` stays `AbsentVision` (D11). The builder never reads
a frame (no tick, no poll — `robot.hpp` (A1) ~:114-117: "The builder still never ticks and never commands a motor").

### 1.9 `include/shulib/vision/camera_geometry.hpp` — NEW: pixels → directions and distances, with their error (VIS1-A1)
`v6/proposed/…/vision/camera_geometry.hpp`, PROS-free, allocation-free, pure:
- `Interval {valid, lo, hi; mid(), width(), contains()}` — every answer is one; **an interval is a bound from the
  stated tolerances, never a probability — and the stated tolerances include the caller's PoseBound AT THE IMAGE's
  time** (re-check S6, below).
- `CameraOptics` + `kAiVisionOptics{74.0, 63.0, 0.05, 320, 240}` — a DEVICE fact, a library constant registered as
  HA-194 (VEX's 74° × 63°, read only through search snippets — the KB page answers 403), **two focal lengths, never
  a square pixel** (fx = 160/tan 37° = 212.3 px, fy = 120/tan 31.5° = 195.8 px), principal point at the centre, no
  distortion, no roll, **the optical axis square to the housing's back face** (where Step 0.1 measures pitch and yaw
  — HA-194's claim since re-check S6); `kBoxEdgePx = 2.0` (HA-202 — for a model's box AND a colour band's rows,
  re-check S6).
- `PoseBound {pose, positionTolerance, headingToleranceDeg}` — **the CALLER's statement** of how far the pose may be
  off (VIS1 does not know robot two's drift — unmeasured, research `lib` §4) **at the time the IMAGE was taken, not
  when the read completed** (re-check S6: PROS gives no exposure time — HA-192); `FieldFix {valid, x, y, radius}`.
- **`PoseBound boundAtImage(atRead, speedInPerS, turnRateDegPerS, maxImageAgeSeconds)`** (re-check S6, prototyped):
  the position tolerance + |speed| × age, the heading tolerance + |turn rate| × age; a negative or non-finite input
  gives an INFINITE position tolerance — never a tight bound from a bad number. The age is a MEASURED number (Step
  0.7's latency check, HA-192), so the helper takes it from the consumer (AF1's and L4's configs, UNSET refuses
  there — §6.5); VIS1 does not invent one. Probe R6: turning 180°/s with a Pin 36 in ahead, the bound at READ
  completion excludes the true Pin once the image is 66 ms old (7.0 in off, radius 4.4) and at 100/150 ms; driving
  60 in/s past a Pin 14 in aside, at 150 ms; widened, it includes it at every lag (0–150 ms); the tracker with its
  position gate on, a lagged image while turning 60°/s, never reports a wrong colour or a wrong Toggle (lag can only
  cost a refusal). Mutations MG5 (the turn term dropped) and MG6 (the speed term) RED.
- `CameraModel(optics, placement)` + `constexpr const char* cameraModelProblem(const config::CameraConfig&)`
  (nullptr, or "no camera is declared" / the placement's problem — prototyped, probe R3.3; a consumer asks it first
  and refuses to construct, naming the reason): `ray()`, `bearing(u, v)`, `elevation(u, v)`, `groundRange(u, vBottom,
  minDepressionDeg)` (refused near the horizon — never infinity), `sizeRange(realHeightIn, u, vCentre, pxHeight)`,
  `fieldFix(robot, bearing, range)`, `inViewForCertain(fwd, left, up, marginPx)`, `project()`, `eachCorner()`,
  `rangeToPlane()`. **Every output is evaluated at every CORNER of the tolerance box** (optics ±, pitch ±, yaw ±,
  height ±) and the hull taken — exact while the output is monotone in each input over the box.
- **One camera explains it:** where two quantities come from ONE camera (a band's two edges; a Pin's base and its
  height), a consumer tests them CORNER BY CORNER ("is there one camera inside the tolerances that sees this box as
  X?") — never by overlapping two separately widened intervals, which double-counts the common pitch error (P4b,
  every other gate equal: 129 vs 86 lookalikes of 540 pass). The header says so where `eachCorner` is declared.
- `hal::CameraMount` (`vision_conversion.hpp`, HA-70's level camera for the tag PnP) is untouched.

### 1.10 `include/shulib/vision/override_elements.hpp` — NEW: the season's facts (VIS1-A1)
DATA ONLY, every number cited, one header per season (next season adds its own): `ToggleFace` — **the lower edge
as an INTERVAL [12.55, 12.65] in** (re-check S10: the glossary's 2.05-in faces; v4 had [12.36, 12.65]), the apex
14.39 in (pA8: axle 13.20 in + 30.2 mm), the set-back of the apex behind the lower edge [0.9, 1.2] in (v4: [0.9, 1.5]), the length 25.8 in (glossary; pA8's 25.99 includes the caps), the wall top
11.50/11.54 in (pA14/pA13) — and HA-201 named on each inferred number (§0.5 item 2); `kTogglePlaces[4]` (one centred
on each wall at the 70.20-in centreline, pA10, F1 field frame, with the into-field unit vector; how far the face
sits from the wall's inner face is UNVERIFIED, HA-201); `PieceSize` and `kPinSize{1.6, 6.5}`, `kCupSize{3.15, 6.5}`
(glossary, v20.txt ~:4546, ~:4270); `ModelClass` and `kOverrideClassNames` {"redBluePin", "blueYellowPin",
"redYellowPin", "yellowPin", "cup"} (api.vex.com, 2026-10-15 — replaced by the camera's own spelling if Step 0.2's log
exists, §0.5 item 5); `modelClassSize(id)`. **The executor re-checks pA8, pA10, pA13–pA15 and the Pin/Cup glossary
entries against the newest manual it can fetch and names the version it used** (v2.1's status is UNVERIFIED).

### 1.11 `include/shulib/sim/camera_plant.hpp` — NEW: the host camera model (VIS1-A1)
`v6/proposed/…/sim/camera_plant.hpp`. A scene (coloured quads: Toggle field faces AND bottom faces, plates,
LED strips, cloth; model elements: Pins and Cups, standing, lying or raised, each half firing a colour) seen
through a **TRUE camera** that a test sets DIFFERENT from what the library believes, inside the stated tolerances,
from a TRUE robot pose — and a Toggle ROLLED, or stuck, at any angle about its axle (`addRolledToggle(…, rollDeg,
lift, …)`: a sharp equilateral prism, faces ~2.06 in, the axle lifted mid-roll — re-check S2). Failure modes: edge jitter (seeded, deterministic), every n-th detection dropped, unplugged
(`ReadFault`), FROZEN (the last frame again, re-stamped — indistinguishable from a still scene), torn
(`PartialFault`), a model check other than `Matches` (objects withheld, as the adapter does), 24 or more (re-check S4)
(`truncated`). **Independence (the A2 rule):** it projects with its own vector code in the FIELD frame and includes
nothing from `vision/` — a sign error in the library's projection cannot cancel out (MG1/MG2 prove the tests see
one). Its banner states what it does NOT model: occlusion, lighting, the colour classifier itself (a quad IS its
descriptor), motion blur, the wall (it may hide a bottom face from a low camera). `sim/` may include `hal/`; the core
never includes `sim/` (`ci.yml` :71-78).

### 1.12 `include/shulib/vision/toggle_tracker.hpp` — NEW: RED / BLUE / YELLOW or UNKNOWN with the reason (VIS1-A2)
`v6/proposed/…/vision/toggle_tracker.hpp` (v4: the frozen latch fixed; the re-check: S2's meaning, S3's turn, S7's
veto, S8's side-clip rule, R7's colour-above veto). `ToggleColour {Unknown, Red, Blue, Yellow}`; `ToggleWhy {Sure,
NoRead, CameraSilent, ReadNotUsable, CameraFull, NoBand, CutByImage, MaybeBottomFace, TwoColours, NotHeldYet,
CameraFrozen, ConflictingPartialBand, ColourAbove}`; `ToggleTrackerConfig` — **every number the team's, from Station I; every
default 0 = UNSET and refused** (`toggleTrackerConfigProblem`, the MECH core's no-library-default rule): the three
descriptor ids (distinct), `holdSeconds` (T), `aliveSeconds` (X), `minAspect`, `minLengthFraction` /
`maxLengthFraction`, `minRangeIn` / `maxRangeIn`, `bearingWindowDeg`, `frozenMoveIn`, `frozenTurnDeg` (re-check S3), and the optional position gate
(`positionToleranceIn`, `maxPoseToleranceIn`, `maxFixRadiusIn`). `update(frame, PoseBound)`; `view(now)` →
`ToggleView {colour, why, heldSeconds, bearing, range, positionChecked, toggleIndex}`. The gates, in order:
1. **Kind and id:** a Colour detection with one of the three ids, else ignored.
2. **SHAPE:** width/height ≥ `minAspect` (a standing Pin half is tall; a Cup is not a band), and taller than
   2 × `kBoxEdgePx` (a library rule from HA-202, not a team number: a band within its edges' jitter cannot be measured
   — without it the bottom face's sliver read as a far face, P7/T8; MT16).
3. **CUT:** a band touching the image's top or bottom is the face leaving the frame → "cut by the image", never a
   colour from a partial face. **A band cut at a SIDE has no measurable length, so it is judged only when it spans
   the image's centre column** (a real face at the flip position, wider than the picture, does); otherwise it is
   "cut" too (re-check S8: v4 skipped the length gate for any side-clipped band, and with a 35° window let 219 / 274
   of 9,054 short red bands — plates, a lying Pin — through; now 0, the checker's `c7b` re-run and R2.S8).
4. **ONE CAMERA EXPLAINS IT** (per corner, §1.9): the lower edge on the face's lower-edge plane, the upper edge on
   the apex plane, at ranges that put the apex 0..set-back behind the lower edge, AND (a band not cut at a side) a
   width that is `minLengthFraction..maxLengthFraction` × 25.8 in at that range — with the v4 change that the lower
   edge and the set-back are INTERVALS (§1.10), so the corner loop gains those two dimensions.
5. **NOT THE BOTTOM FACE** (P6/T1): a band some corner explains as the horizontal bottom face is kept only when the
   place its field face would occupy, directly above, is certainly in the image AND holds no other Toggle-coloured
   band; a band there → this one IS the bottom face (dropped); that place out of frame → UNKNOWN "too close (bottom
   face?)".
6. **WHERE:** its bearing interval wholly inside ±`bearingWindowDeg`, and its range interval OVERLAPPING
   [`minRangeIn`, `maxRangeIn`] (the window drops bands that are certainly elsewhere — a far wall's Toggle; the view
   carries the range interval so a consumer can demand more). Both semantics pinned by tests.
7. **POSITION (optional):** with a pose bound tighter than `maxPoseToleranceIn`, the band's field fix (radius ≤
   `maxFixRadiusIn`) must lie on a Toggle's face line (`kTogglePlaces`, ± the tolerance); `toggleIndex` says which.
- **Two different colours passing at once → UNKNOWN** (never "pick one"); the same colour twice → the nearer band.
- **A passing band beside a band of ANOTHER Toggle colour the gates could not judge → UNKNOWN
  "ConflictingPartialBand"** (re-check S7): a band cut by the image, or one the bottom-face check cannot judge,
  whose bearing touches the window may be the real face — v4 skipped such bands, so a passing RED with a cut BLUE
  beside it read RED (the checker's C2; now UNKNOWN).
- **A band with a box of ANOTHER Toggle colour directly ABOVE it → UNKNOWN "ColourAbove"** (re-check R7 — a defect
  this re-check found, in neither checker's list): columns overlapping, the other box's top above the band's and its
  bottom within 2 × `kBoxEdgePx` of the band's top or lower, whatever ITS gates said. A seated field face has nothing
  of another Toggle colour directly above it (its bottom face is BELOW it); a bottom face does (its field face), and
  so does a Toggle mid-roll. Without it, a robot TURNED 15° to a RED Toggle 30 in away read the YELLOW bottom face
  "sure" (the face's band slants off the optical axis, its box grows, gate 4 refuses it, and the thin bottom face was
  explained as neither a bottom face nor refused) — on the v4 prototypes too (16 wrong of 78 turned parkings; 38 of
  390 in R7 with the veto removed, 0 with it).
- **A read reporting 24 or more ("possibly full", `truncated`) → UNKNOWN "camera full"** (D6, re-check S4); a
  non-usable read breaks the hold.
- **Freshness is time** (HA-192): a colour is reported only when it has held for `holdSeconds` on usable reads AND
  a usable read is within `aliveSeconds` of now. **While the CAMERA moves** — its LENS (the robot pose through the
  placement, not the robot's centre) travels more than `frozenMoveIn`, OR the robot turns more than `frozenTurnDeg`
  (re-check S3: v4 compared the centre's translation only, so a robot turning in place with a frozen camera kept a
  stale RED "sure" through 90°, the checker's C1) — a band whose box has not moved by one pixel is UNKNOWN "camera
  frozen?"; **the first read whose box moved again clears it
  and RESTARTS the hold** (v4 — the prototype latched until the colour changed, P7/T7). Parked, nothing can tell a
  frozen camera from a still scene — said in the header, and why AF1's post-push check is not the camera's (F1, F5).
- **What a colour MEANS (re-check S2), said in the header and the guide:** the colour of the face the camera SEES
  where a seated face could be, within the stated tolerances — NOT `<SC4>`'s "set" (fully seated, a face parallel
  with its mounts, AND no robot touching; otherwise neutral, v20.txt :949-956). A Toggle stuck mid-roll shows its
  DESTINATION face from below and can read as that colour (probe R4, §2.2), and the camera's measure of a face's lean
  is never fine enough to tell seated from rolled at any mount and range modelled (probe R1: 0 of 21 seated faces
  resolved, 0 rolled ones resolved as seated) — so the tracker exposes no "seated" flag and no lean: seating and
  contact are the consumer's to establish (AF1, §6.5).
- **A pure helper for the bench:** `explainAsToggleFace(detection, model)` → the range interval at which some
  corner explains it (or invalid, with which gate), and the implied length — so station 12 can PRINT what the
  tracker measures without the team's thresholds (§1.15).

### 1.13 `include/shulib/vision/piece_finder.hpp` — NEW: take ONE named piece, or none (VIS1-A2)
`v6/proposed/…/vision/piece_finder.hpp` (v4: the model check; the re-check: the HA-200 site). `Posture {Unsure, OnFloor, Raised, Lying}`;
`sightPiece(detection, model, robot, cfg)` → `PieceSighting {classId, posture, bearing, groundRange, sizeRange, fix,
nearerThanIn, box}`; `PieceTarget {classMask, x, y, toleranceIn, want}` — **the ROUTE's, never a library default**
(`pieceTargetProblem`: no class, tolerance UNSET, wanting UNSURE); `PieceFinderConfig {holdSeconds, aliveSeconds,
minScore, lyingAspect, standingAspect, viewMarginPx}` — the team's, UNSET refuses. `PieceVerdict {Unknown, Found,
None, Ambiguous, NotInView}` + `PieceWhy` (… `ModelNotConfirmed`, v4). The rules:
1. **Posture:** a clipped box is UNSURE (a box cut ONLY by the image's bottom edge carries `nearerThanIn`, the
   floor distance at the bottom row — its base is nearer than that); wider than `lyingAspect` LYING (ground range at
   its near edge, widened by its length); narrower than `standingAspect` standing — ON THE FLOOR when SOME corner
   makes the ground range of its box bottom (the near rim, + up to one radius to its centre) agree with the range its
   known height implies; RAISED when EVERY corner puts the box bottom too high for its size (a Pin on a Goal, in a
   Cup, on a stack); between → UNSURE.
2. **NOT IN VIEW** unless the expected place (its tolerance disc, at the floor AND at a Pin's height) is certainly
   in the image for every corner.
3. **Inside** only when a candidate's WHOLE error disc lies inside the tolerance disc; **on the border** when it
   could be inside; a candidate of the right class it cannot measure whose BEARING could point inside makes the
   verdict AMBIGUOUS — never silently dropped (unless its `nearerThanIn` bound puts it certainly short of the disc).
4. **FOUND** = exactly one inside, none on the border, held for `holdSeconds` (the same candidate: its fix within
   the two radii); **NONE** = none inside or on the border (with "wrong posture inside" when a candidate of the
   class stood there in another posture); **AMBIGUOUS** = two inside, one on the border, one unmeasurable, or its own
   error larger than the tolerance ("drive closer, or measure the mount tighter" — P5: AMBIGUOUS to ~25 in, FOUND at
   21 in); never the nearest of several.
5. **UNKNOWN** when no usable read, no read within `aliveSeconds`, the read was truncated (D6), FOUND not yet held —
   **or the frame's `model` is not `Matches`** (v4, P7: the prototype answered NONE from a read whose objects were
   all withheld).

### 1.14 `include/shulib/vision/camera_feed.hpp` — NEW: one read per period, for every consumer (VIS1-A2, D10)
`CameraFeedConfig { double minReadPeriodSeconds = 0.0; }` (UNSET refuses); `CameraFeed(hal::IVisionFrames&, const
hal::IClock&, CameraFeedConfig)`; `bool tick()` — reads when at least `minReadPeriodSeconds` has passed since the
last read (never twice in one call), returns whether a new frame was read; `const hal::VisionFrame& latest()`;
`Stats {reads, usable, readFaults, partialFaults, notConfigured, truncated, objectsWithheld, lastReadSeconds,
slowestReadSeconds}` — each read timed with the clock before and after (µs on the robot — `hal/pros/clock.hpp` :5-16
binds `pros::micros()`), the measurement HA-203 needs. It owns no task, allocates nothing, and is ticked from the
caller's loop (D10).

### 1.15 `src/` — robot two's camera, its boot, and Bench Tests station 12 (VIS1-B; after MECH-A5, before A6 — D1)
- **`src/robots/chassis_table.hpp`** (D2): `ChassisTable` gains `shulib::config::CameraConfig camera{};` — **the one
  member WITH a default member initializer (re-check T3):** the struct has none today and every initializer — the
  three tables and the five in `test/chassis_table_test.cpp` (e.g. :55-70) — names every field, and GCC 13 refuses a
  designated initializer that omits a member WITHOUT one (`-Wmissing-field-initializers` under `-Werror`; reproduced
  on g++ 13.3 and arm-none-eabi-g++ 13.2.1, `v6/overlay/`); with `camera{}` the bench bot's and the no-chassis
  tables and the five tests stay untouched and compile (checked). Robot two's table names `.camera = {...}`: port 3 (R2a :14-17), **objects ON with `kOverrideClassNames`, colours
  NONE until Step 0.3's numbers exist, the placement UNSET (`measured = false`) until Step 0.1's** — no value is ever
  invented (HA-111's lesson, `chassis_table.hpp` :26-31). If Step 0 has run, its numbers go in with their provenance
  line.
- **`src/common/boot_source.{hpp,cpp}`** (MECH-A5's form): the camera block copied into `RobotConfig::sensors.camera`;
  a case where port 3 carries the OLDER sensor boots WITHOUT the camera, naming it. (`test/boot_source_test.cpp`
  :159's one-word flip `Vision` → `AiVision` moved to VIS1-A1 — re-check T5: A1 is the run that changes what
  `Vision` MEANS, and MECH-A5 rewrites boot_source and its tests in between; a fixture left saying "the older sensor
  is on robot two's port 3" for that span would carry a wrong fact into A5's rewrite.)
- **The programs** (Drive, Competition, Auton Tester — MECH-A5's forms): nothing but what the Robot's boot line now
  says (§1.8); no program reads a frame in VIS1 (AF1 and L4 add the feed to the loops that need it).
- **`src/programs/bench_r3a.cpp` station 12** (D3; its own functions only — `aiVisionStation`, `setUpCamera`,
  `recordCamera`, the sensors namespace's helpers; never `kMenu`, `menu_grid`, `bench_r3a.hpp` — MECH-A6's):
  1. **COLORS:** when robot two's table carries descriptors, station 12 writes them and turns COLORS on beside
     TAGS+OBJECTS, logging every write and read-back as it does today; when it carries none: "colours UNSET — teach
     them (Step 0.3)".
  2. **The model check line** at entry: the camera's names for ids 0..4 vs `kOverrideClassNames`, by name — "model
     OK" or the first difference (the adapter's words). **And `logClassNames` stops trusting the count (re-check, T1):**
     it prints the name of EVERY id 0..19 (`AIVISION_MAX_CLASSNAME_COUNT`), each into a zeroed buffer, and its first
     line says "get_class_name(-1) -> 1 (PROS 4.2.2 returns its success code here, not a count)".
  3. **Read timing:** each `snapCamera` timed with `pros::micros()` before and after — the panel's period line adds
     "read ~N µs (max M)"; RECORD logs it (HA-203).
  4. **Measurements, never verdicts that need the team's numbers:** each COLOR blob's RECORD line gains its aspect,
     whether it is cut, `explainAsToggleFace`'s range interval (or which gate failed) and implied length; each
     OBJECT's gains its posture by the bench's own printed aspect bounds, ground range, size range and bearing — all
     computed through the library's pure functions from the table's placement, or "mount UNSET (Step 0.1)" when it
     is not measured. These are the numbers Station I's trials turn into the team's thresholds.
  5. **The library's own read path, once:** the FAMILY cycle (`g_tagFamily`, -1 → the four families) gains one more
     state, **LIBRARY**: the station constructs a `ProsAiVision` from the table's settings (TAGS off — logged), reads
     it through a `CameraFeed` at the station's own printed period (33 ms — HA-192's ~30 a second), and shows the
     frame AS THE LIBRARY SEES IT (status, count,
     withheld, model check, the feed's stats); the next press returns to "as found", re-running `setUpCamera`
     (logged). The adapter's first on-robot run is therefore a bench station, not a match. **The LATENCY check
     (re-check S6, HA-192):** on the LIBRARY page a press paints a solid block in a taught colour on the brain's
     screen at a logged `pros::micros()` and logs the first read whose frame reports it, ten trials, the largest
     printed as "image age <= N ms" — the bound AF1 and L4 widen their pose bound with (§1.9's `boundAtImage`).
  6. Every line through the file's own `emit`/`emitf` (since SER1, `2506067`: `emit` → `hal::pros::serialWrapped`,
     one `write()` per line; the gate's step 9 refuses stdout stdio in `src/`); `%f` lines only on a RECORD press,
     never in the per-read loop (SER1's `char_sink.hpp` :76-77: `%f`/`%e`/`%g` allocate); the RECORD format keeps
     R2a's fields first (the 18 raw bytes stay).
- **`docs/internal/chunks/R3a-BENCH-WORKSHEET.md` Station I** rewritten from the new source (:1119, :1136-1138 say
  colours need teaching first): Step 0.5–0.8's trials (§2.0) as its steps, what each line means, the LIBRARY page
  and its latency check, and the event-day check with the re-teach and re-upload path (re-check S9(f)).

---

## 2. Step 0, the runs, their tests and mutations, and the definition of done

### 2.0 STEP 0 at the robot — a worksheet for the team lead (steps 0.1–0.4 need NO new code; run them first)
Nothing here moves the robot under power. Bring: a tape measure, a phone with a level app (or a protractor), a laptop
with VEXcode V5 **4.68 or later** (the Override model arrived in 4.68, 2026-07-29 — plc.pd.vex.com release note) and
PROS, a USB-C cable for the CAMERA's own port, a real Toggle or a mock built from pA8 (25.8 in long, three 2.05-in
faces — red, blue, yellow — in pA15's order), three Pins and a Cup, a REAL opponent-coloured licence plate (4.5 in
long, `<R4>`g) AND a long red strip (tape or cloth, ~24 in × 2 in), a wedge to hold a Toggle part-rolled, and a
roll of masking tape. Record everything in `docs/internal/chunks/VIS1-PROGRESS.md` under "Step 0" (or hand the
coordinator the photos and numbers). **Every provenance line names the versions (re-check S9(d)):** VEXos on the
brain, the AI Vision sensor's firmware and the AI model's name/version as VEXcode's AI Vision Utility shows them,
VEXcode's version, and the PROS kernel (4.2.2, `project.pros`) — HA-192/197/198 depend on them, and a firmware update
offered at 0.2 changes them.

**0.1 — Measure the camera's mount (each robot that carries one; robot two first).** With the robot on field tiles:
- the LENS height above the tiles (to the lens centre), ± how well (a tape: ~¼ in). **If it is between ~11¾ and
  ~12¾ in and the camera is level, STOP and tell the coordinator before anything else: that is the blind band, where
  the tracker loses the Toggle's colour at most or all distances (D9, probe R5 — re-check S1); the mount moves (to
  ~10–11 in) or pitches up ~10° before Step 0.4;**
- the lens's position from the robot's **TRACKING CENTRE** — forward and left, ± how well. The tracking centre is the
  point the library's pose IS (guide chapter 2, `02-the-field-and-coordinates.md` :128; re-check S9(b) — `ChassisTable`
  has no fore-aft centre). On robot two (a tank on drive-encoder odometry, no tracking pods — `boot_source.hpp` :235,
  :309) it is the point the robot turns about in place: midway between the left and right wheels' contact lines AND
  midway along the wheelbase — find it by turning the robot in place by hand over a taped cross and marking the point
  that does not move. (If pods are added later, it is the point their offsets are typed from: re-measure the camera.)
- yaw — is the camera square to the chassis? (a square against the frame; ± how well, ~1°);
- pitch — the level app on the camera's flat back face (up = +; ± how well, ~1°). Roll — the same app turned 90°;
  the library does not model roll (§1.9): if it is more than ~1°, re-mount the camera. (That the optical axis is
  square to that back face is HA-194's claim; 0.2 checks it.)
- who, when, how, and the versions → the provenance line. **If any number cannot be measured, leave the mount
  UNSET** — the camera still reads; only the geometry refuses (§1.7).
- **If a real Toggle is reachable:** its face's LOWER edge and APEX heights above the tiles, how far (horizontally)
  the field face's lower edge sits from the wall's inner face, and which colour sits on which face (HA-201 — the
  library carries the lower edge as [12.55, 12.65] in from the glossary's 2.05-in faces; a measured Toggle replaces it).

**0.2 — Which AI model is on the camera, how fast it reports, and where its axis points.** Upload "shulib Bench Tests"
as it stands, run **12 AI VISION**. Its entry log (serial and `/usd/r3a_log.txt`) will read `[R3A] aiv port 3 class
names: get_class_name(-1) -> 1` and ONE line, `class 0 '…'` — **the 1 is PROS's success code, not a count** (PROS
4.2.2 returns 1 for every id, §0.3; re-check T1), so the loop stops after class 0 (`logClassNames`, `bench_r3a.cpp`
:2426-2447). Class 0 alone tells the two known tables apart (Override `redBluePin`, Classroom `blueBall`). **For all
five names:** put one piece of each type in view — a red/blue Pin, a blue/yellow Pin, a red/yellow Pin, a yellow
Pin, a Cup — and press RECORD: every OBJECT line carries `id N class '…'` (:2320-2327). **Expected for the Override
model:** 0 `redBluePin`, 1 `blueYellowPin`, 2 `redYellowPin`, 3 `yellowPin`, 4 `cup` (api.vex.com).
- If it names another table (the Classroom model starts `blueBall`): connect the camera's own USB-C to the laptop,
  open VEXcode V5's **AI Vision Utility**, choose the Override model (UNVERIFIED: whether this needs a sensor firmware
  update, and whether the Utility reaches the camera through its own USB-C port or through the brain), power-cycle,
  and run station 12 again: the names must read back **after a power cycle** (HA-197 — the model is kept on the
  sensor). Record any firmware update the Utility made (the versions, above).
- With a Pin moving in view: the panel's "data changes every ~N ms" line (the change timer) — the camera's report
  rate with tags + objects on (HA-192). Stand a Pin still at 12, 24 and 36 in straight ahead and RECORD 5 times at
  each: the box's spread is the edge jitter (HA-202), its bottom row vs the Pin's base is HA-200, its `s` the score
  scale (HA-198). Lay one Pin down; put one on a Goal; RECORD each. **With many pieces in view, note how often the
  count line reads 24** — the cap the tracker and finder treat as "possibly full" (D6, re-check S4).
- **The optical axis (HA-194, re-check S6):** a Pin straight ahead ON the robot's centreline (the tracking centre's
  line, 0.1) at 24 and at 48 in: its box's centre column should be within the yaw tolerance of 160 (and the lens's
  own sideways offset), its bottom row where the measured pitch and height put the floor at that range. A
  consistent offset is an axis not square to the housing — a yaw or pitch to type, not a tolerance to widen.
- **Tag family** (for the tags chunk after G3, free while here): with a field AprilTag in view, press FAMILY through
  the four families and note which one reports it.

**0.3 — Teach the three Toggle colours.** In the AI Vision Utility, under event-like light (the bright, even light of
a gym, not a window): for each of the Toggle's red, blue and yellow faces, set a colour on the face and adjust its
hue and saturation ranges until the face's box is solid and a red/blue/yellow PIN beside it is NOT boxed (if a Pin
cannot be excluded, note it — the tracker's shape gate is the next defence). Save, and copy the six numbers of each
— id, R, G, B, hue range, saturation range — from the colour definition VEXcode writes into the project's device
configuration (**UNVERIFIED** where VEXcode displays them: the Utility's panel or the generated `colordesc` line; the
executor confirms against VEXcode 4.68). Record the light. **These numbers are typed into robot two's table (VIS1-B)
— the library writes them to the camera every boot (HA-196); nothing the Utility leaves on the camera is trusted.**

**0.4 — Can the camera see the Toggle where the arm pushes it? (F1's question; it feeds the Optical decision.)** Put
the robot square to a wall in the position the motor arm would push the Toggle from. Watch the camera's live picture
(the AI Vision Utility over the camera's USB-C, or station 12 running with its USB box overlay on). Record:
(a) the lens's horizontal distance to the wall's inner face; (b) whether the WHOLE field-facing face is in the
picture — all of it, part, or none; (c) back the robot off 1 in at a time until the whole face is in the picture,
and record that distance. **The prediction (a level camera, 63° tall view, HA-194):** the whole face is in the
picture only from **(14.39 − lens height) / tan(31.5° + pitch)** inches away — 13.7 in for a 6-in lens, 10.4 for
8 in, 7.2 for 10 in, 3.9 for 12 in, level; with ±1°/±¼ in and the optics ±5 %: 15½, 12, 8½, 4¾ (P3).
- **"In the picture" is not "the tracker reports it" (re-check S9(a)).** At the flip position and at 12, 24, 36 and
  48 in (square), press station 12's RECORD: its AS FOUND line says whether COLORS is on, and if the camera kept the
  Utility's colours from 0.3 (UNVERIFIED, HA-196 — station 12 turns on TAGS and OBJECTS only and leaves COLORS as
  found), each COLOR blob's raw box is logged. The coordinator runs those boxes and 0.1's mount through the
  prototype tracker and the host camera model (`v6/`) BEFORE the mount is fixed, against R5's prediction (D9). If no
  COLOR line appears, the whole-face judgement stands on the video and the tracker's verdict waits for 0.5.
- **The arm in the picture (re-check S9(e)):** with the motors off, move the flipper arm by hand through its stroke
  (rest, mid, full): is any of it in the picture, what colour is it, and does it cover the Toggle's face at any point?
  A red or blue arm in view is a lookalike the tracker has never seen.
- **What the answer feeds (AF1's, not VIS1's — re-check S5):** whatever 0.4 shows, the camera's colour is the
  far-range "Toggle ahead" cue: it cannot tell a seated Toggle from one stuck mid-roll, or see whether a robot is
  touching it (`<SC4>`a/b; S2), so AF1's push needs the face's colour corroborated AT THE PUSH POINT by a sensor that
  sees colour there — in this plan an Optical at the flipper (F1's fix). A side distance sensor gates reach and
  presence only, never colour. The Optical decision stays the team lead's, after this step (D9's last bullet).

**After VIS1-B lands (worksheet Station I, rewritten; §1.15):**
- **0.5 — The colours on the library's measurements.** Station 12 with COLORS on: each face at 12, 24, 36, 48 in,
  straight and ±5°, ±10°, ±20° (R7 predicts the colour is lost close in beyond ~5° from square), RECORD each — every
  blob's line now carries its aspect, cut, the range at which the camera explains it as a Toggle face, and its implied
  length. These become the team's tracker numbers (minimum aspect, length fractions, range window, hold and alive
  times). **Five RECORDs of a still face at each distance settle HA-202 for COLOUR bands (re-check S6 / S9(c)):** the
  band's top and bottom rows against the rows the measured mount puts the apex and the lower edge at, and their spread
  — glare on the back-leaning face, rounded corners.
- **0.6 — Lookalikes (F10) and a Toggle that is not seated.** RECORD with: the three Pin colours standing near the
  wall; a Pin stack ~2 ft from the wall; **the REAL 4.5-in plate AND the long red strip**, each held at 12–14 in on
  the face's line near the wall (re-check S8: the short plate is R2.S8's case — a short band, now refused at any
  side-clip; the long strip is T10's — from ~22 in out a strip at the face's height on the face's line is NOT
  distinguishable, closer it is); coloured clothing or an LED strip behind the Toggle; a neighbouring field's Toggle
  in view; **the robot's own arm** (0.4). Which read as a band at Toggle height, and what the measurements say about
  each (HA-199). **A Toggle that is not seated (re-check S2):** wedge it part-rolled at ~45° and at ~90° toward each
  neighbouring face and RECORD from 14, 20 and 30 in — probe R4 predicts the destination face can be reported, which
  is why "seated" is AF1's to establish, not the camera's.
- **0.7 — Timing and the library's own read.** The read time (µs, max) with COLORS + OBJECTS on (HA-203); the
  change rate (HA-192); then the FAMILY button to **LIBRARY**: the library's adapter writes its settings (TAGS off),
  checks the model, reads frames — its page shows the status, the withheld count and the feed's stats. Unplug the
  camera for 2 s and plug it back: the page must show the fault, the re-set-up, and the model re-checked (HA-196).
  **The image's age (HA-192, re-check S6):** the LIBRARY page's LATENCY check — point the camera at the brain's own
  screen (hold the brain in front of it, cable permitting, or unmount the camera and re-do 0.1 after); the station paints a solid block in a taught colour at a logged `pros::micros()` and logs the first read
  that reports it: that difference bounds the image's age at read completion FROM ABOVE (the screen's own delay is
  included — conservative, which is the direction a bound needs). Ten trials; the largest is the `L` AF1 and L4 widen
  with (§1.9's `boundAtImage`).
- **0.8 — On the event's practice field, before the first match (re-check S9(f)).** Station I's short check under the
  EVENT's light: the three faces at 24 and 48 in, RECORD. If a face does not box solidly, re-teach that colour in the
  Utility (0.3), type its six numbers into robot two's table, rebuild and re-upload (`make`, `pros upload`) — the
  descriptors are compiled in, so there is no other path; the camera chapter's bench section documents it.

### 2.1 The runs — three, each its own go, each verified and committed on its own (D1)

| Run | What (§1) | Files it writes | Waits on | Size (rough) | Public docs |
|---|---|---|---|---|---|
| **VIS1-A1 — the camera as a device** | §1.1 frame seam; §1.2 `ProsAiVision`; §1.3 its fake; §1.4 the shim; §1.5 `DeviceKind::AiVision`; §1.6 the provider's `aiVision()` in all four implementers; §1.7 `CameraConfig` + `valid()`; §1.8 the Robot builds it; §1.9 the geometry; §1.10 the season's data; §1.11 the host camera model | NEW `hal/vision_frames.hpp`, `hal/pros/ai_vision.hpp`, `hal/fake/fake_vision_frames.hpp`, `config/camera_config.hpp`, `vision/camera_geometry.hpp`, `vision/override_elements.hpp`, `sim/camera_plant.hpp`, `test/pros_shim/pros/ai_vision.h`; CHANGED `hal/device_registry.hpp`, `hal/pros/device_registry.hpp`, `hal/device_provider.hpp`, `hal/pros/device_provider.hpp`, `hal/fake/fake_device_provider.hpp`, `config/robot_config.hpp`, `robot/robot.hpp`; tests: NEW `vision_frames_test.cpp`, `pros_ai_vision_adapter_test.cpp`, `camera_geometry_test.cpp`, `override_elements_test.cpp`, `sim_camera_plant_test.cpp`, `robot_camera_test.cpp`; CHANGED `pros_device_registry_adapter_test.cpp`, `robot_builder_test.cpp` (the two forwarding providers), A1's `mechanism_devices_test.cpp` (its kind check walks "EVERY kind the registry can report", :8-9 — `AiVision` joins `kEveryKind` :63-66 and gains a case in the test's OWN `foundWords` switch :69-82, re-check T5; it is refused by `distance()`/`optical()`), `boot_source_test.cpp` :159 (ONE word, `Vision` → `AiVision` — a `test/` file, so A1's `src/ tools/` diff stays empty; re-check T5) | **MECH-A1 merged** | **~2½–3½** | changelog; guide 10 (the API: the camera accessor, the frame); glossary; **HA-192..HA-203 all registered here** (A1's code cites every one — the register's reconciliation is bidirectional, a label with no row fails it); roadmap's Freeze Register (a row for the frame seam, NOT FROZEN, stated out loud); `docs/api/` + `mkdocs.yml` nav (generated) |
| **VIS1-A2 — what the camera means** | §1.12 the Toggle tracker; §1.13 the piece finder; §1.14 the camera feed | NEW `vision/toggle_tracker.hpp`, `vision/piece_finder.hpp`, `vision/camera_feed.hpp`; tests: NEW `toggle_tracker_test.cpp`, `toggle_tracker_scenes_test.cpp`, `piece_finder_test.cpp`, `piece_finder_scenes_test.cpp`, `camera_feed_test.cpp`, `vision_guide_example_test.cpp` (the chapter's listings, verbatim) | VIS1-A1's BUILD ends (it shares no code file with A1's review-and-fix — the overlap MECH-A2 used); it MERGES after VIS1-A1 | **~2¼–2¾** (re-check: +¼ — the S3/S7/R7 rules and R2, R4–R7, S15b as tests) | changelog; the guide's NEW camera chapter (the next free number at its tip — MECH-A3 takes 18 if it lands first); guide 14 (what it cannot do yet: lookalikes, a parked frozen camera, no tags); HA-197, 199, 200, 203's Source lines gain the tracker, finder and feed sites; `docs/api/` |
| **VIS1-B — robot two and the bench page** | §1.15 entire | `src/robots/chassis_table.hpp`, `src/common/boot_source.{hpp,cpp}`, `src/programs/bench_r3a.cpp` (station 12's functions only), NEW `src/programs/bench_camera.hpp` (station 12's PROS-free measurement lines — never `bench_r3a.hpp`, A6's); tests: NEW `bench_camera_test.cpp`; CHANGED `boot_source_test.cpp`, `chassis_table_test.cpp` (robot two's camera block pinned — no other change: the member's default initializer keeps every existing initializer valid, re-check T3) | **MECH-A5 merged, MECH-A6 NOT launched** (else A6 merged); SER1 merged (`2506067`, done); Step 0.1–0.3's numbers typed if they exist (else UNSET — never invented) | **~1¾–2¼** (re-check: +¼ — the LATENCY check, Step 0.8 and the turned/unseated trials in Station I) | the camera chapter's bench section; changelog (the programs' boot line words); `R3a-BENCH-WORKSHEET.md` Station I (internal); HA settle steps on each entry |
| **VIS1, all three** | | | | **~6½–8½** (re-check: was ~6–8; + verification ~half again, `REMAINING-WORK.md` :236) | |

**Why above row 18d's ~4¾–6½** (the 10-08 estimate, colours only): the 10-14 OBJECTS scope — the piece finder, the
model check by name, the ground and size ranges and their per-corner agreement (~1–1½); the camera feed (D10, ~¼);
the registry fix (D4, ~¼); the re-check's rules, tests and bench additions (~½). ALLY1 (D8) is NOT in it (~1–1½, its
own brief).

### 2.2 Tests (adversarial) and mutations — every test tries to break one rule; every mutation is RUN and seen red
The probes are the starting tests (copied, tightened, every `printf` turned into a `CHECK` or a doctest `MESSAGE`).
**A test that only passes on the prototype is not enough:** each rule below has a test that FAILS when the rule is
removed — the mutation lists below say how that was shown, and the run re-shows it on its own code.

**VIS1-A1**
- **The seam:** every `colourDescriptorProblem` / `aiVisionSettingsProblem` refusal by name and its boundaries (hue
  1 and 40 accepted, 0.99 and 40.01 refused; saturation 0.1 and 1 accepted; id 0 with ranges set refused as "id
  outside 1..7", all-zero refused as UNSET; duplicate ids; nothing on; objects with no table; a 21st class);
  `usable()` true for `Ok` only; `clipped()` at each edge and margin.
- **The adapter over the shim** (P1's 14 cases, plus): a camera RE-PLUGGED carrying another model → after the
  re-set-up the model check reads `Different` and objects are withheld again; `get_object_count` returning
  `PROS_ERR` (= `INT32_MAX`, the vendored `error.h`) is a fault, never "2,147,483,647 detections, truncated"; a
  count of 23 is complete, **24 and 25 are truncated with all 24 read** (re-check S4 — v4 said "exactly 24 is NOT
  truncated"; P1.20, mutation M14); the forbidden calls' counters stay 0 through every case, AND a source
  pin: `aivision_reset`, `aivision_start_awb`, `aivision_set_tag_family`, `aivision_set_usb_bounding_box_overlay`,
  `aivision_set_code` appear in `hal/pros/ai_vision.hpp` only inside comments.
- **The registry:** `E_DEVICE_AIVISION` → `AiVision` / "AI-VISION"; `E_DEVICE_VISION` stays `Vision` / "VISION";
  every `v5_device_e_t` value still named (the existing walk).
- **The provider (both shipped ones and the forwarding two):** port 0 and 22 refused; an empty port, a MOTOR, and the
  OLDER Vision sensor on the port each refused naming what was found; a settings problem refused with its sentence;
  the same object for a second request with equal settings, a refusal for different ones; `readFaults(port)` is the
  camera after it is made and nullptr before (it never makes one).
- **The config:** a motor typed on the camera's port refused naming both; a settings problem refused; an UNSET
  placement ACCEPTED (the camera builds, `cameraModelProblem` names it); a SET placement with a zero tolerance, a height
  outside (0, 36) in, a pitch outside ±45° or no provenance refused; every config in the suite still `valid()`.
- **The Robot:** the camera built and `cameraAbsence()` empty; a refused camera → nullptr, the provider's sentence,
  AND the drivetrain still built (D5); port 0 → "no camera is declared"; the boot line's words in EVERY state the
  seam can report, driven through the fake provider's `FakeVisionFrames::setStatus` (model OK / check pending /
  DIFFERENT with the first differing name / NOT set up with its problem / objects off) and "camera ABSENT" — each
  line ASCII-only (re-check T2, T10).
- **The geometry** (P2, plus): the hand anchors (a point straight ahead projects to the centre; a point ABOVE the
  axis projects to a SMALLER row — the anchor that caught the plant's first sign error); 400 random placements and
  true points all inside their intervals; the horizon refusal (`groundRange` invalid, never infinite); fx ≠ fy;
  **`boundAtImage` (re-check S6, R6.3): no motion changes nothing, motion adds exactly |v|·L and |ω|·L, a negative or
  non-finite input gives an infinite position tolerance**.
- **The host camera model:** its own anchors (independent of the library's: a field point straight ahead at the
  lens height lands on the centre; up is up); frozen returns the last frame re-stamped; `setModel(Different)`
  withholds every object and counts it; 24 or more → truncated (R2.S4: 23 → the colour holds, 24 → "camera full");
  **`addRolledToggle` (re-check S2): at roll 0 the field face's lower vertex lies on the wall line and its apex at the
  axle + the circumradius; ±120° brings the next face into the field (its own anchors, not the library's)**.
- **Mutations (each RUN, seen red, restored):** M1–M14 (P1's: the mask leaving TAGS alone; no re-set-up after a
  failed read; reading index 24+; a skipped object still `Ok`; no mode read-back; no retry rate limit; no 1 Hz
  verify; the model check skipped; the model check comparing only class 0; an unreadable table counted as a
  match; an Object past the expected table passed on; the name buffer not zeroed (a camera that writes nothing
  reads as the expected name); a colour read-back not compared; truncation only ABOVE the cap — M11–M14 are the
  re-check's; M13 SURVIVED until P1.19 was written), MG1–MG6 (yaw sign; image down read as up; square pixels; nominal
  only; `boundAtImage` dropping the turn term; dropping the speed term — MG5/MG6 the re-check's, red in R6), and the
  run's own: the provider
  making a SECOND adapter for a second request (two writers); `E_DEVICE_AIVISION` back to `Vision` (the older sensor
  accepted as the camera); `valid()` not claiming the camera's port; the Robot refusing the build on a refused
  camera; an all-zero descriptor accepted; `PROS_ERR` count treated as a number.

**VIS1-A2**
- **The tracker** (P4, P6, P7/T7–T10, P8 as scene tests on the host camera model; plus unit cases on hand-made
  frames): each gate refuses its lookalike (shape, minimum height, cut, explain, bottom face, where, position); two
  colours → UNKNOWN; a truncated read → UNKNOWN "camera full"; a non-usable read breaks the hold; silent past
  `aliveSeconds`; frozen while moving → UNKNOWN, and RECOVERY restarts the hold (T7); **frozen while TURNING in place
  → UNKNOWN within `frozenTurnDeg`, a live camera turning slowly never "frozen" (R2.S3, re-check S3), and the LENS's
  travel flags it with the turn check set high (R2.S3b)**; **a passing band beside a CUT band of another colour →
  UNKNOWN "ConflictingPartialBand", beside a cut band of its own colour → the colour (R2.S7), and beside a band the
  bottom-face check cannot judge with no column overlap → the same veto (R2.S7b, re-check S7)**; **side-clipped short
  bands (4–6.5 in, a 35° window) never a colour (R2.S8, re-check S8)**; **a seated Toggle with the robot TURNED −30..
  +30° (cameras 8/10/11 in, bottom BLUE or YELLOW, 14–48 in): never a wrong colour (R7, re-check R7) — the coverage
  printed**; **a Toggle rolled −120..+120° (R4, re-check S2) as a CHARACTERISATION: never claimed as set, the count of
  colours reported between 20° and 100° printed (78 of 216 on the v6 prototypes) so a change to it is seen**; **a
  lagged image while turning with the position gate on: never a wrong colour or Toggle (R6.4, re-check S6)**; the
  range window's OVERLAP and the bearing window's CONTAINMENT each pinned (a band whose range straddles `maxRangeIn`
  passes; a WHOLE, uncut face whose centre's bearing lies outside the window does not — P4/S15b, re-check S8: S15's
  side-clipped face is now "cut" before the window, which left MT12 alive until S15b); the T1 approach and T8's unbroken RED runs; T9 (12¾-in camera, 3 in); **R5 (re-check S1)
  as a characterisation: every D9 mount × the lower edge's two ends, its middle and v4's 12.36 (out of belief),
  parked every 2 in from 80 to 4 in — the far..near table printed, never a wrong colour, and pinned: a 10-in level
  lens first reports at ≥ 60 in, a 13¼-in one at ≤ 44 in, the 12¼-in one never (the blind band — a change to it is
  seen)**; T10 (the
  plate never a colour at ≤ 14 in); the P4b sweep as a CHARACTERIZATION — its counts printed and the position-gate
  count pinned at 0; every config refusal by name.
- **The finder** (P5, P6's piece cases, P7): FOUND only with one inside and none on the border, held; NONE /
  AMBIGUOUS / NOT IN VIEW / UNKNOWN each from its scene; an unmeasurable candidate whose bearing could be inside →
  AMBIGUOUS; a bottom-cut box certainly short of the disc ignored; a Pin on a Goal is RAISED and never taken by an
  OnFloor route; a lying Pin LYING; a truncated read and a model not `Matches` → UNKNOWN, never NONE (P7); **a lagged
  image — turning 180°/s, or driving 60 in/s past a Pin — with the bound widened by `boundAtImage`: the true Pin
  always inside the fix; un-widened, outside at the longer lags (R6.1, R6.2 — the finding pinned, re-check S6)**;
  every target and config refusal by name.
- **The feed:** never two reads in one `tick()`; no read before `minReadPeriodSeconds` (a fake clock); `latest()` is
  the latest READ including a fault frame — never the last good one held; the stats count each status; a fake camera
  that advances the fake clock during `read()` gives the read time (`lastReadSeconds`, `slowestReadSeconds`).
- **Mutations:** MT1–MT25 and MF1–MF9, MF1b (the probes' 35, all seen red on the v6 prototypes —
  `v6/out/mutations_all.v6b.txt` and `v6/out/mutations_recheck2.txt`; MT19–MT25 are the re-check's: the turn ignored,
  the partial-band veto removed, the side-clip rule back to v4, the robot centre instead of the lens, the colour-above
  veto removed, the CannotTell half of the partial veto removed, the side-clip rule and the window together; MT22 and
  MT24 SURVIVED until R2.S3b and R2.S7b were written, MT12 until P4/S15b; re-run on the run's own code), and the feed's: reading every tick (the period ignored); `latest()`
  keeping the last GOOD frame across a fault; the slowest read never updated.

**VIS1-B**
- `bench_camera_test.cpp`: station 12's measurement lines from hand-made detections and a table placement —
  "mount UNSET (Step 0.1)" when it is not measured (never a number); "colours UNSET — teach them (Step 0.3)" when the
  table has none (never an invented descriptor written); the model-check line in both states; every line within the
  log's width and through the one sanitising path (R2a's rule).
- `chassis_table_test.cpp` (CHANGED, re-check T3): robot two's camera block pinned field by field (port 3, objects
  on with the five Override names, no colours, placement UNSET) and the other two tables' `camera.port == 0`.
- `boot_source_test.cpp`: robot two's camera block reaches `RobotConfig` unchanged; port 3 carrying the OLDER sensor
  boots WITHOUT the camera, naming it; a table with no camera block builds no camera.
- `python3 tools/src_build_gate.py check` and `self-test` (all builds link; SER1's object check: no stdout stdio).
- **Mutations:** the station writing a descriptor when the table has none; a measurement line printing a range from
  an UNSET mount; boot_source dropping the camera block.
- **On the robot (Step 0.5–0.7, the worksheet):** the LIBRARY page's re-plug check is the only test of the adapter's
  fault path on hardware — recorded, never claimed before it runs.

### 2.3 Definition of done — every run (the MECH core's §2.1, adapted)
- [ ] `docs/internal/chunks/VIS1-PROGRESS.md` exists (VIS1-A1 creates it FIRST), with this run's heading, its tip,
      each step as it happened, every mutation with its observed red, the prose review's greps and hits.
- [ ] The host suite builds and passes (`cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests`) and
      `ctest --test-dir build/test -R compile_refusal` passes — counts in the log.
- [ ] The ARM header gate (RESUMING :107-112) and the CI PROS-free and sim-free guards pass.
- [ ] `python3 tools/src_build_gate.py check` passes every run (A1/A2 change headers `src/` includes); B also
      `self-test`.
- [ ] `tools/api_doc_tool.py` (coverage, fresh, examples, removability), `doc_staleness_audit.py`,
      `briefing_status.py` checks pass.
- [ ] `grep -rn "PROBE" include/shulib/vision include/shulib/hal/vision_frames.hpp include/shulib/hal/pros/ai_vision.hpp
      include/shulib/sim/camera_plant.hpp test/pros_shim/pros/ai_vision.h` is empty; no "prototype"/"proposed" text.
- [ ] `git diff --quiet <tip> -- include/shulib/hal/vision.hpp` (F4's file untouched); A1 and A2: `git diff --stat --
      src/ tools/` empty.
- [ ] Every §2.2 test of the run exists and every mutation was RUN, seen red, restored (the log names each).
- [ ] Docs in the same change (§3), the prose review logged (pace rule 4), the C7 removability grep empty.
- [ ] **No hardware claim:** every HA entry this run registers says what settles it; nothing says "works on the
      robot".
- [ ] **The register's direction-2 check (re-check T7):** every HA id this run registers or cites has at least one
      `A4 register HA-nn` code site under `include/shulib` (A1: all twelve — HA-200 at `groundRange`/`sizeRange`;
      A2: HA-197/199/200/203 gain the tracker, finder and feed sites), each named in the log.
- [ ] Nothing committed; the tree's state stated in the log.

---

## 3. Documents — in the same change as the code (pace rule 3), the prose review logged (rule 4), ≤ ~5 public each (rule 7)

### 3.1 Per run (public)
- **VIS1-A1:** `docs/changelog.md` — "Added since 2.4: the AI Vision camera as a device" (no version bump — every
  change ADDITIVE on surfaces in no Freeze Register row: `hal::IDeviceProvider`, `config::RobotConfig`,
  `robot::Robot`, `hal::DeviceKind` (an appended enumerator); the F4 seams untouched — the MECH-A1 entry's wording is
  the model) **plus one BEHAVIOUR CHANGE (words): a census line that said "VISION" for an AI Vision sensor now says
  "AI-VISION"**; `docs/guide/10-the-api.md` (the camera accessor, the frame, "only `Ok` is evidence"); the glossary
  (frame, colour descriptor, AI model / class table, bearing, ground range, size range, fix radius, tolerance box,
  "one camera explains it"); `docs/hardware-assumptions.md` (all twelve, HA-192..HA-203, rows and entries — A1's
  code cites every one; the Held line drops them; the status line's count); `docs/roadmap.md`'s Freeze Register — a row for `hal::IVisionFrames` /
  `VisionFrame` / `vision/*`: **NOT FROZEN, stated out loud** ("silence in this register reads as frozen" — D2's
  lesson; MECH-A1's coordinator note (4) asked the same for `IReadFaults`). `docs/api/` regenerated, `mkdocs.yml`'s
  nav gains five pages (`vision_frames`, `pros-ai_vision`, `camera_config`, `camera_geometry`, `override_elements`;
  `hal/fake/` and `sim/` are outside the reference by the tool's own rule, `tools/api_doc_tool.py` ~:95-104 — so
  `test/README.md` documents `FakeVisionFrames`, the shim and the host camera model for test authors, as that rule
  says).
- **VIS1-A2:** changelog; **a NEW guide chapter, "The camera"** (the next free number at its tip), **AND its two
  hand-kept index entries — a row in `docs/guide/README.md`'s chapter table and a line in `mkdocs.yml`'s "User guide"
  nav** (re-check T8: `api_doc_tool` generates only the API nav, and a page missing from the nav passes `mkdocs build
  --strict` with an INFO line, `tools/api_doc_tool.py` :1479-1491 — so nothing else would catch it): what it sees and
  what it cannot (the frame, freshness as time, the tolerance box, the tracker's gates and its UNKNOWNs with their
  reasons, the finder's five verdicts and why it never guesses, the mount trade and the blind band of D9, the lookalike
  limit, why the camera's colour is the face SEEN, never a set Toggle, and why AF1 needs the colour corroborated at
  the push point — re-check S1/S2/S5; what turning costs close in — R7; the image's age and `boundAtImage` — S6),
  every code listing held verbatim by `vision_guide_example_test.cpp`
  (`check-examples`); `docs/guide/14-what-it-cannot-do-yet.md` (the camera's honest limits; "no camera has been
  pointed at a tag" stays true — tags are not VIS1's); HA-197/199/200/203's Source lines; `docs/api/`.
- **VIS1-B:** the camera chapter's bench section; changelog (the programs' boot line words); each HA entry's
  "Settle" names its Station I step. Internal: `docs/internal/chunks/R3a-BENCH-WORKSHEET.md` Station I rewritten
  from the new source (its :1119-1140 block and the menu row :101).

### 3.2 Internal, every run
`VIS1-PROGRESS.md`; `build-order.md` (the VIS1/AF1 entry :1641-1659 and the `**Next:**` block); `REMAINING-WORK.md`
row 18d (and rows 24, 26 where they name VIS1); `PROJECT-BRIEFING.md`'s generated block (`briefing_status.py`).
At the end of VIS1-B: `VIS1-COMPLETED.md` and the pointer moves (pace rule 5).

### 3.3 The plan commit (the coordinator's, on the team lead's go, BEFORE VIS1-A1)
`build-order.md`'s VIS1/AF1 entry gains the three runs, D1–D11's answers, and the objects scope of 10-14 and the
piece choice; `REMAINING-WORK.md` row 18d's text and size (~6½–8½); **ALLY1 added** (D8 (a): a row and a build-order
entry, the chunk count +1); `docs/hardware-assumptions.md`'s Held line gains **HA-192..HA-203** (`tools/
briefing_status.py` reads it); this brief moved to `docs/internal/chunks/VIS1-ai-vision-colours-and-objects.md`.

### 3.4 The prose review's greps (each run runs them and logs the hits it read)
`grep -rn -i "vision\|camera\|AbsentVision\|DeviceKind::Vision\|VISION" docs/ README.md` (public), and the same over
`docs/internal/` for statements VIS1 makes stale: "no adapter yet" (`hal/device_registry.hpp` :57's doc; HANDOFF's
R2b item), "TAGS AND VISION ARE DELIBERATELY ABSENT" (`robot_config.hpp`), "vision ABSENT" (the boot line's
documented form), "COLORS left as found" / "colours … none set here" (station 12, the worksheet), "interprets
nothing" (`bench_r3a.cpp` :36-47 — station 12 now prints the library's measurements beside the raw records: the
banner says so), HA-70's "no library adapter reads it yet" (`hardware-assumptions.md` ~:3478-3479).

---

## 4. Hardware Assumptions — HA-192..HA-203 ONLY (the register's Held line gains them at the plan commit)
Each entry in the register's form: *Claim · Source (`A4 register HA-nn` beside the code) · Confidence · Settle ·
Blast radius if wrong*. **None is measured.** A thirteenth belief ⇒ stop and tell the coordinator.

| # | Belief | Confidence | Cited in code by | Settled by |
|---|---|---|---|---|
| **HA-192** | The data PROS returns is the camera's LATEST frame, refreshed at ~30 a second (VEX's figure, read only through search snippets), with NO counter and NO timestamp; a read's completion time is the only freshness a program can know, and a frozen camera is indistinguishable from a still scene. **The IMAGE is older than the read's completion by an age `L` nobody has measured** (re-check S6) — so a pose paired with a frame must be bounded at the image's time (`vision::boundAtImage`, §1.9), and L4/AF1 refuse field positions while moving until `L` is measured. (HA-72's and HA-90's "~20 Hz" are the TAG pipeline's and are not this.) | **reasoned** — the rate is VEX's figure seen only through search snippets (the KB page answers 403), the same source as HA-194; the lack of a counter or timestamp from the vendored header (whole) and binary; `L` UNKNOWN (re-check S10: v4 said "invented" and "read", neither the register's word for these) | A1 (`vision_frames.hpp`'s `readAt`, the adapter, `camera_geometry.hpp`'s `PoseBound`/`boundAtImage`) | Step 0.2 / 0.7: the change timer's interval with a piece moving, colours + objects on; **`L`: Step 0.7's latency check** (the LIBRARY page paints a taught colour on the brain's screen at a logged `pros::micros()`; the first read reporting it bounds `L` from above, ten trials) |
| **HA-193** | The report is 320 × 240 with (0, 0) at the top-left, +y DOWN, and a box is its left edge, top edge, width and height (`ai_vision.h` :129-130 "left edge (from camera's view)", "top edge"; api.vex.com's `originX/Y`) | **reasoned** | A1 | Step 0.2: a Pin moved left/right and up/down — the numbers move the stated way; a box at the image's edge reads 0 / 319 / 239 |
| **HA-194** | The optics: 74° × 63° (two focal lengths, fx 212.3 px, fy 195.8 px), the principal point at the centre, no distortion worth modelling, ±5 % on each focal length — **and the optical axis square to the housing's back face** (where Step 0.1 measures pitch and yaw; re-check S6) within the mount's stated tolerance | **reasoned** — VEX's figures seen only through search snippets (the KB article answers 403); "reported" is the register's word for the team lead's own report, not a VEX snippet (re-check S10) | A1 (`kAiVisionOptics`; the geometry header's axis paragraph) | Step 0.2: a Pin straight ahead on the robot's centreline at 24 and 48 in — its box centre at column ~160 and its bottom row where the measured pitch puts the floor (the axis); Station I: a Pin at known offsets (straight, ±20°, at 12/24/36 in) — the computed bearings and ranges against the tape (the optics) |
| **HA-195** | A read is NOT atomic: the count and each object are separate calls, the camera may update between them; a `get_object` whose port claim fails returns `{id 127, type 127, the rest 0}` (the vendored `libpros.a`: the record is zeroed and both bytes set to 127 BEFORE `claim_port_try` — re-check T4); what the SDK returns for an index past its own count is UNKNOWN (the adapter's skip rule — any type other than COLOR 1 / CODE 2 / OBJECT 4 is skipped — covers 0, 127 and TAG 8 alike) | **reasoned** (the API's two-call shape; the 127 record READ in the binary, its SDK half not) | A1 (the adapter's `PartialFault`; the shim's failure record) | Step 0.7: the LIBRARY page's partial-fault count while pieces move; unplugging mid-read (station 12's RECORD prints any non-1/2/4/8 type as `TYPE 0x..` with its raw bytes) |
| **HA-196** | `set_enabled_detection_types(bits, mask)` sets EXACTLY the masked mode bits from `bits` and keeps the rest; `get_enabled_detection_types` reads the modes back; `get_color` reads back the RGB exactly as written. (Whether the camera KEEPS modes and colours across programs, power cycles and re-plugs is NOT assumed either way: the adapter writes them at every construction, after every failed read and on any read-back mismatch.) | **reasoned** — the doc's one example (:242-244), AND the vendored binary (re-check T4): `set_enabled_detection_types` passes `(bits, mask)` straight to `vexDeviceAiVisionEnableSet`, PROS's own `enable_` / `disable_` call it as `(t, t)` / `(0, t)` (a set-and-mask reading), and the overlay, white balance and tag family use `vexDeviceAiVisionModeSet` instead — so the all-modes mask cannot disturb them; every setter returns 1 whatever the SDK did. The SDK's own semantics are unread | A1 | Step 0.7: the LIBRARY page's `settingsLost`/`setups` across a re-plug; station 12's AS FOUND line after another program ran; the mode line after the adapter's write reads exactly COLORS+OBJECTS |
| **HA-197** | The AI model is chosen in VEXcode's AI Vision Utility and KEPT on the sensor; a PROS program reads each class id's NAME with `get_class_name` (the vendored binary returns 1 for every id — **the class COUNT cannot be obtained through PROS 4.2.2**, `libpros.a` disassembly, §0.3); the Override model names its classes exactly `redBluePin`, `blueYellowPin`, `redYellowPin`, `yellowPin`, `cup` (ids 0–4); for an id the camera does not name, the SDK writes nothing a zeroed buffer would not show | **measured elsewhere** (api.vex.com's table, 2026-10-15) for the names; **reasoned** for "kept on the sensor" and the PROS read path (the binary's behaviour is read, its SDK half is not) | A1 (the adapter's check; the finder relies on it from A2) | Step 0.2: station 12's RECORD lines (`class '…'` per OBJECT) with one piece of each type in view, before and after a power cycle; the entry line `get_class_name(-1) -> 1` confirms the binary's return |
| **HA-198** | The order of up to 24 detections is UNVERIFIED (VEX's page says largest-first, by width in one place and by area in another), and the Object score's raw scale is UNKNOWN (VEXcode reports 1–100) — and NOTHING in VIS1 depends on either, or on whether a count above 24 can be reported: a read reporting 24 or more is "possibly full" and never evidence (D6; re-check S4 reworded this row so the design no longer rests on it) | **reasoned** (the independence, from the code; the order and the scale themselves unknown — re-check S10's vocabulary) | A1 | Step 0.2: RECORD with several pieces in view — the order against their sizes, the `s` values, and how often a crowded field reads 24 |
| **HA-199** | The three descriptors separate the Toggle's red, blue and yellow faces under event light, and the lookalikes (Pins, Goals, plates, LEDs, clothing) either fire no band at Toggle height or fail a gate — EXCEPT those the probes show cannot be told apart without a trusted pose (P4b: 65 of 540 bands since re-check S10 — 75 before; T10: a 26-in strip on the face's line from ~22 in out — 16 before; a legal 4.5-in plate clipped at the picture's side never, since S8) | **invented** | A1 (the host camera model cites it); the tracker from A2 | Step 0.5 / 0.6, then a field visit under event lights |
| **HA-200** | The model's box encloses the WHOLE visible piece: its bottom row is where a standing Pin's base meets the floor (or its support), its height the Pin's 6.5 in — which the finder's ground/size agreement and RAISED rest on | **invented** | A1 (`camera_geometry.hpp`'s `groundRange` and `sizeRange` docs — `A4 register HA-200`, re-check T7); the finder's posture agreement from A2 | Step 0.2: a still Pin at 12/24/36 in, standing, lying, on a Goal — the box against the tape |
| **HA-201** | The Toggle's geometry: the field-facing face's lower edge **12.55–12.65 in** up, the apex 14.39 in and **0.9–1.2 in** behind the lower edge, the bottom face horizontal at the lower edge's height and a different colour, the face's distance from the wall's inner face UNVERIFIED — ARITHMETIC from pA8 (axle 13.20 in, apex 30.2 mm above it — the circumradius of a ~52-mm equilateral) and the glossary ("each face of the triangle is approximately 2.05” (52.2mm) wide", v20.txt :4668): an equilateral prism resting on a face, lower edge ≈12.61 (§0.5 item 2; re-check S10 narrowed v4's [12.36, 12.65] — 12.36 read pA8's 51.6 mm as a VERTICAL size, which implies 2.34-in sloped faces, against the glossary) | **reasoned** | A1 (`override_elements.hpp`) | Step 0.1: a real Toggle measured (and the newest manual's pA8 and glossary) |
| **HA-202** | A box edge's jitter is at most ~2 px for a still object (`kBoxEdgePx`; the minimum band height is twice it) — **for a model's object box AND for a COLOUR band, whose top and bottom rows are taken as the face's apex and lower edge within it** (glare on the back-leaning face, rounded corners; re-check S6, merging S9(c)) | **invented** | A1 (`camera_geometry.hpp`); the tracker's band rows from A2 | Step 0.2: five RECORDs of a still Pin at each distance — the spread; **Step 0.5: five RECORDs of a still Toggle face at 12/24/36/48 in — the band's rows against the rows the measured mount predicts for the apex and the lower edge, and their spread** |
| **HA-203** | One read (1 + N device calls, N ≤ 24) costs little enough to make once per `minReadPeriod` inside the driver loop's 10 ms tick without stretching it | **invented** | A1 (`IVisionFrames::read()`'s doc); the feed from A2 | Step 0.7: station 12's read time (µs, max) with colours + objects on; later the driver loop's own tick monitor under AF1 |

Every entry's code site carries `A4 register HA-nn`; the register's direction-1 grep (no `PROVISIONAL (A4` without an
HA id) stays empty. HA-70, HA-72 and HA-90 are NOT changed (tags) — HA-70's "no library adapter reads it yet" gains
"(the camera's objects and colours are read since VIS1; its TAGS still are not)".

---

## 5. OUT, and explicitly rejected

**OUT (with the chunk that owns it):**
- **AprilTags** — detection, families, PnP, the duplicate-id field map (`TagMap` refuses duplicates,
  `tag_map.hpp` :104): R2b's tag part, after G3 (the team lead, 2026-10-14). The adapter keeps TAGS off; station 12
  keeps its FAMILY discovery for that chunk's evidence.
- **The automatic flip** — the action, its state machine, the second push, the end-of-period lockout, the period
  clock, the Toggle model in the host test plant, the TOGGLE bench station, the Optical's LED, the driver arbitration:
  **AF1** (after MECH-A4). VIS1 gives AF1 an interface (§6.5) and builds none of it.
- **Our colour** (RED / BLUE / SKILLS, UNSET until chosen): **ALLY1** (D8).
- **Driving to a piece** — turning to the bearing, the drive, the claw's confirmation, the route steps that name a
  piece: **L4** (after VIS1). The pieces' expected positions are the ROUTE's (the manual's start layout typed by the
  students in their route — the project's guardrail); VIS1 ships no piece map.
- **The wall reset with the side distance sensors: L6** (beside the MECH core) — it shares VIS1's robot-file, Robot
  and robot-two files; who rebases, and the pattern its sensors follow, are §6.7's (re-check T6).
- **`hal::IVision` behind the Robot** (D11); **a library vision task** (D10).
- **Colour codes, COLOR_MERGE on robot two** (allowed by the settings, unused), **white balance**, **lens distortion,
  roll**, **occlusion and lighting in the host camera model**, **the 24-in robot** (both robots are 15" this
  season), **the AI Vision's Wi-Fi under `<R13>`c** (a rules question for the team, noted in the research — not a
  library belief), **the drivers' Toggle lookup card** (F16 — the team's, free).

**Explicitly rejected:** trusting the detection order (D6 (b)); trusting class ids without the names (D7 (b));
a separate camera program (D3 (c)); overlapping two separately widened intervals (P4b: 129 vs 86 lookalikes); a
held frame on a failed read (it reads as a fresh "nothing changed"); a library default for any of the team's
thresholds (the MECH core's rule); refusing the whole robot when the camera is refused (D5 (b)); calling
`aivision_reset` or `aivision_start_awb` from the library (§0.3).

---

## 6. Coordination — shared files, who waits for whom, and what the consumers get

### 6.1 In flight while this was written (2026-10-15 ~10:00; NOT edited — every worktree read only)
| Job | Worktree | State | VIS1's relation |
|---|---|---|---|
| MECH-A1 | `mech-a1` (on `bdcbc7c`, uncommitted) | fix round + verify (run `wf_e4501e23-3d5`) | **VIS1-A1 waits for its merge** — five shared code files + `test/robot_builder_test.cpp` |
| MECH-A2 | `mech-a2` (from A1's snapshot `85e0285`) | building | no shared code file (its files: `mechanism/{until,service}.hpp`, `sim/mechanism_plant.hpp`, `manipulation/mechanism_outcome.hpp`, `sequence/run_guard.hpp`, its tests); docs only (`README.md`, `mkdocs.yml`, the changelog, the generated API) |
| SER1 | `ser1` | reviews | `src/` stdout writers, `hal/pros/char_sink.hpp`, `tools/src_build_gate.py`'s object check — VIS1-B builds on SER1's rule (no stdout stdio in `src/`) |
| CAL1-A1 | `cal1-a1` (on `aaea302`) | building | C shims beside VIS1's (`test/pros_shim/pros/{misc,motors,screen}.h`, `main.h`) and `test/pros_shim/pros/rtos.hpp`; no census word pinned ("VISION" appears in none of its files — checked) |

**Re-check, 2026-10-15 ~21:55 (T10):** MECH-A1 merged (`fd36e07`, 14:49); SER1 COMMITTED (`2506067`, 21:53); MECH-A2
was being merged in the main worktree — its staged files (`mechanism/{service,until}.hpp`, `sim/mechanism_plant.hpp`,
`sequence/run_guard.hpp`, `manipulation/mechanism_outcome.hpp`, their tests, and docs: the changelog, guide 10, the
glossary, the HA register, `mkdocs.yml`, the roadmap) include no VIS1 code file; the docs are §6.2's serial merges.
CAL1-A1 unchanged. Nothing else VIS1 touches moved.

### 6.2 The MECH core's six runs (MECH brief §2's table, :2127-2135) against VIS1's files
| VIS1 file(s) | MECH run that writes them | Rule |
|---|---|---|
| `hal/device_provider.hpp`, `hal/pros/device_provider.hpp`, `hal/fake/fake_device_provider.hpp`, `config/robot_config.hpp`, `robot/robot.hpp`, `test/robot_builder_test.cpp` | **A1** | VIS1-A1 starts from the tip AFTER A1 merges, re-reads each (line numbers here are A1's working tree) |
| `hal/device_registry.hpp`, `hal/pros/device_registry.hpp`, `test/pros_device_registry_adapter_test.cpp` | none (A1–A6 lists checked) | — |
| `sim/camera_plant.hpp` (new) | A2/A3 write `sim/mechanism_plant.hpp` | different files; the sim-free guard covers both |
| `vision/*.hpp`, `config/camera_config.hpp`, `hal/vision_frames.hpp`, `hal/pros/ai_vision.hpp`, the shim (all new) | none | — |
| the guide's new chapter number | **A3** creates "the mechanisms chapter" at the next free number (18 unless G3a Part 1 took it) | VIS1-A2 takes the next free number AT ITS TIP; whichever lands second renumbers nothing — it simply takes the next |
| changelog, `docs/api/` (generated), `mkdocs.yml`, guide 10, the glossary, the HA register, the roadmap, build-order, REMAINING-WORK, the briefing | every run | serial merges: the generated files are REGENERATED after a merge (`api_doc_tool.py`, `briefing_status.py`), never hand-merged; the HA register's rows merge by number (MECH holds 182..186, VIS1 192..203) |
| `src/robots/chassis_table.hpp`, `src/common/boot_source.{hpp,cpp}`, `test/boot_source_test.cpp` | **A5** rewrites boot_source, the programs, the team files, `tools/src_build_gate.py` | **VIS1-B after A5 merges** |
| `src/programs/bench_r3a.cpp` (station 12's functions) | **A6** writes `bench_r3a.{hpp,cpp}` (the 3 × 5 menu, station 13) and `test/bench_menu_grid_test.cpp` | **VIS1-B before A6 launches; if A6 has launched, after A6 merges.** VIS1-B never touches `bench_r3a.hpp`, `kMenu`, `menu_grid` or the station row (its pure parts go in the NEW `bench_camera.hpp`); A6 re-reads its tip anyway (MECH brief §2) |
| `competition/{auton_registry,match_runner}.hpp` (and `auton_menu.hpp` beside them) | **A4** writes the first two | VIS1 touches none; **ALLY1** (D8) writes `auton_menu.hpp`'s colour row, after A4 |

**Order, all told:** MECH-A1 merges → VIS1-A1 (beside MECH-A2/A3) → VIS1-A2 starts when VIS1-A1's BUILD ends,
merges after it → … MECH-A5 merges → VIS1-B → MECH-A6. With ~5 jobs allowed at once (the team lead, 2026-10-08) and
ONE when the budget runs low (the team lead, 2026-10-01), VIS1 is never on the MECH critical path: if a slot is
short, VIS1 waits, never MECH. **L6** (the wall reset) also runs beside the MECH core and shares four of VIS1's files —
§6.7 (re-check T6).

### 6.3 SER1 (committed `2506067`, 2026-10-15 21:53 — re-check T10)
VIS1-B's station lines go through the file's own `emit`/`emitf` (since SER1, `emit` → `hal::pros::serialWrapped`: one
`write()` per line; no `printf`/`puts`/`std::cout` in `src/` — the `src` gate's step 9 fails a build whose objects name
stdout stdio); `%f` only on a RECORD press (SER1's `char_sink.hpp` :76-77: `%f`/`%e`/`%g` allocate). Nothing in `include/shulib/vision/` prints: the tracker, finder and feed
return values and words; a consumer logs them.

### 6.4 CAL1
No shared file. The census word change (D4) reaches Calibrate's census through `deviceKindWord` (`calibrate_program.cpp`
:1115 at `2506067` — :1106 before SER1, re-check T10, `calibration/boot_check.hpp` :294, :348) — robot two's port 3 will read "AI-VISION". CAL1-A2a (the standard
Calibrate's program) writes its own census: it should use `deviceKindWord` and pin nothing about port 3's word.

### 6.5 What AF1 (and L4, and ALLY1) get from VIS1 — an interface, not an implementation
**AF1 (the automatic flip), after MECH-A4:**
- A camera: `robot.camera()` (nullptr ⇒ AF1 does not arm; `cameraAbsence()` says why) and the model:
  `vision::cameraModelProblem(robot.config().sensors.camera)` (non-null — e.g. an UNSET mount ⇒ AF1 refuses to
  construct, naming Step 0.1), then `vision::CameraModel{vision::kAiVisionOptics, ….placement}`.
- A feed it ticks from the loop it already runs in (D10): `vision::CameraFeed{*robot.camera(), clock, {period}}`.
- A colour belief: `vision::ToggleTracker{cfg, model}` fed `feed.latest()` with the robot's `PoseBound`; `view(now)`
  → `ToggleView {colour, why, heldSeconds, bearing, range, positionChecked, toggleIndex}`. **AF1's contract with it
  (rewritten at re-check S5/S2/S6/R7):**
  1. Only a colour is evidence; UNKNOWN (any `why`) never triggers.
  2. **A colour is the face the camera SEES, not a set Toggle** (S2): it cannot tell a seated Toggle from one stuck
     mid-roll (which can show the colour it is rolling toward — probe R4) or see whether a robot is touching it
     (`<SC4>`a/b). The camera's colour is therefore the FAR-RANGE "Toggle ahead" cue, whatever Step 0.4 shows.
  3. **Before any push, the face's colour must be corroborated AT THE PUSH POINT by a sensor that sees colour there**
     (S5) — in this plan an Optical at the flipper (F1's fix). A side distance sensor may gate REACH and PRESENCE
     only; it never counts as colour evidence (an opponent robot in front of the Toggle, or a stuck Toggle, satisfies
     a distance reading). Whether the Optical is fitted is the team lead's decision after Step 0 (D9's last bullet):
     without it, AF1 cannot push on colour.
  4. **The bearing in a view is the robot's at the IMAGE's time** (S6): a robot that has turned since is off by
     turn rate × the image's age; AF1 widens its own "in front of my arm" window by it, and with the position gate on
     passes `vision::boundAtImage(bound, speed, turnRate, L)` with the MEASURED `L` (Step 0.7; UNSET in AF1's config
     refuses to arm while moving).
  5. **Close in, the colour holds only near-square to the Toggle** (R7: at 14–20 in only within ±5°; ±10–20° at
     30–48 in) — AF1's approach squares up before it judges; a turned approach reads UNKNOWN, never a wrong colour.
  6. A frozen camera is caught only while the camera MOVES (drives or turns, S3); parked at the Toggle — exactly the
     post-push check — nothing can tell (F5): the post-push check is not the camera's. `bearingWindowDeg` and the range window are where AF1 says "in front of my arm"; `toggleIndex` (with
  the position gate on) says which wall's Toggle — the Skills target colour (the Quadrant's, `<RSC3>`a-iii-1) is
  AF1's policy, not VIS1's.
- **What VIS1 does NOT give AF1:** the target colour (ALLY1), the policy (the students' code), reach, the push, the
  post-push check (the camera cannot see it parked — F5), the period clock, the Toggle's roll direction (UNVERIFIED,
  a TOGGLE-station fact).
**L4 (drive to a piece), after VIS1:** `vision::PieceFinder{cfg, model}` with `setTarget(PieceTarget{classMask
(from `override2026::ModelClass`), x, y, toleranceIn, want})` fed the same feed; `answer(now)` → FOUND (the
`sighting`: bearing, ground range, field fix with its radius — what L4 turns to and drives at; **the fix is only a
bound if L4's pose bound is the one AT THE IMAGE: L4 passes `vision::boundAtImage(…, L)` with Step 0.7's measured `L`,
and while `L` is UNSET it takes a piece only with the robot stopped** — re-check S6) or NONE / AMBIGUOUS /
NOT IN VIEW / UNKNOWN, each a reason for the route's step to be SKIPPED, never to grab another piece (the team
lead's 10-14 rule: "not touch one vs touch another"); "don't touch" stays route geometry (no obstacle avoidance is
planned). ALLY1's colour tells L4 which Pin class is "ours".
**ALLY1 (our colour), after MECH-A4, before L4 and AF1:** VIS1 needs nothing from it and gives it nothing; its own
brief (D8).

### 6.6 G3a (the robot panel) — what its robot file must adopt
The camera block is robot-file DATA (D2): `CameraConfig {port, settings {colours[], objects, expectedClasses,
colourMerge}, placement {measured, forward, left, height, yawDeg, pitchDeg, tolerances, provenance}, note}`. G3a's
generator writes it like every other block; the descriptors and the mount carry their provenance strings (who
measured, how, when, under what light). G3a's brief is told by the coordinator at the plan commit.

### 6.7 L6 (the wall reset with the side distance sensors) — beside the MECH core, sharing VIS1's files (re-check T6)
The team lead's 2026-10-14 ruling puts **L6 before the panel, beside the MECH core, before 2026-11-23 if it can**
(`build-order.md` :2177 at `2506067`; `REMAINING-WORK.md` row 25, :104); it has no brief yet. Declaring robot two's
left and right distance sensors (ports 4 and 5, R2a :14-17) and building them touches four files VIS1 also edits:

| File | VIS1's edit | L6's (expected) |
|---|---|---|
| `config/robot_config.hpp` | VIS1-A1: `SensorsConfig::camera`, `PortUse::Camera`, `valid()`'s camera checks | the side sensors' block in `SensorsConfig`, a `PortUse`, `valid()`'s claims |
| `robot/robot.hpp` | VIS1-A1: `camera()`/`cameraAbsence()`, the boot line's camera words (:770, :787, :798) | the side sensors' accessors and their boot-line words |
| `src/robots/chassis_table.hpp` | VIS1-B: `ChassisTable::camera{}` and robot two's block | robot two's ports 4/5 |
| `src/common/boot_source.{hpp,cpp}` | VIS1-B: the camera block copied into `RobotConfig` | the side sensors copied the same way |

- **Rule:** whichever of L6 and VIS1-A1 (or VIS1-B) lands SECOND re-reads those files at its tip and rebases onto
  them — never a hand-merge of a copy taken before the other merged (the MECH-A2 PORT RULE's lesson). Neither waits
  for the other: they share files, not a design.
- **The template L6's sensors follow (so the robot file does not grow two styles):** D2/D5's pattern — the block's
  type in its own `config/` header with only hooks in `robot_config.hpp` (as `config/camera_config.hpp`); each port
  claimed through the one-owner `claim` with an APPENDED `PortUse` (and its case in the exhaustive `portUseWord` —
  landmine 13); built by the provider's EXISTING `distance()` (MECH-A1, `hal/device_provider.hpp` :178 — L6 needs no
  provider change); a refused sensor NEVER refuses the robot (`nullptr` and an `…Absence()` sentence, the
  mechanism accessors' contract, `robot.hpp` :646-696); ASCII boot-line words beside the camera's ("camera port 3 set
  up, model OK" / L6's own). The coordinator's L6 brief cites this section.

---

## 7. Checks — the coordinator re-runs every one after each run (RESUMING's step 3: never the report's word)
```sh
cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -6
ctest --test-dir build/test -R compile_refusal --output-on-failure | tail -3
# the ARM header gate (RESUMING :107-112) — every v2 header, VIS1's ten new ones included
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > <scratch>/all.cpp
echo "int main(){return 0;}" >> <scratch>/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c <scratch>/all.cpp -o /dev/null -Iinclude
# the CI guards (ci.yml :55-62, :71-78)
grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib | grep -v '^include/shulib/hal/pros/'   # empty
grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib              # empty
python3 tools/src_build_gate.py check            # every run; VIS1-B also: self-test
python3 tools/api_doc_tool.py self-test && python3 tools/api_doc_tool.py check-coverage \
  && python3 tools/api_doc_tool.py check-fresh && python3 tools/api_doc_tool.py check-examples \
  && python3 tools/api_doc_tool.py check-removability
python3 tools/doc_staleness_audit.py && python3 tools/briefing_status.py check
# the brief's own
git diff --quiet HEAD -- include/shulib/hal/vision.hpp && echo "F4 vision seam untouched"
git diff --stat -- src/ tools/                    # VIS1-A1, VIS1-A2: empty
grep -rn 'PROBE\|prototype\|proposed' include/shulib/vision include/shulib/hal/vision_frames.hpp \
  include/shulib/hal/pros/ai_vision.hpp include/shulib/sim/camera_plant.hpp test/pros_shim/pros/ai_vision.h   # empty
grep -n 'aivision_reset\|aivision_start_awb\|aivision_set_tag_family\|bounding_box_overlay\|aivision_set_code' \
  include/shulib/hal/pros/ai_vision.hpp           # comment lines only
grep -rn 'PROVISIONAL (A4' include/shulib | grep -v 'HA-[0-9]'   # empty (the register's direction-1 grep)
grep -rln 'internal/\|chunks/\|RESUMING\|build-order' docs/*.md docs/guide docs/cookbook README.md   # empty (C7)
```
Plus: spot-check the run's single most load-bearing rule in the source (A1: a failed read returns NO detections and
re-writes the settings; A2: UNKNOWN never becomes a colour — the hold, the alive check, two colours, the model check;
B: no descriptor is written that the table does not carry), and re-run two of the run's mutations by hand.

---

## 8. Landmines
1. **The packed record** (`ai_vision.h` :128-175): copy each field OUT by value; a pointer or reference to a packed
   member is undefined on the ARM (R2a landmine 2).
2. **`PROS_ERR` is `INT32_MAX`** (`pros/error.h` :32): compare the count with `PROS_ERR` FIRST — a "count >= 24"
   check alone reads a failure as a full, truncated frame. And the cap is `>=`, not `>` (re-check S4, D6): a read of
   exactly 24 may have been cut off.
3. **Never `aivision_reset`, never `aivision_start_awb`** (§0.3): the first may drop the model the Utility chose; the
   second moves every measured colour mid-match. The source pin catches a regression; a review must too.
4. **A held frame is a lie.** Every other sensor adapter answers a fault with its last good value (A1's channel
   exists because of it); a frame must not — a held frame reads as a fresh "nothing changed". The fault frame has
   no detections, and the feed's `latest()` is the fault frame.
5. **A frozen camera looks like a still scene** (HA-192). The tracker can catch it only while the CAMERA moves — its
   LENS travels (the robot pose through the placement, never the robot's centre) or the robot TURNS (re-check S3:
   v4 compared the centre's translation only, and a robot turning in place kept a stale colour through 90°;
   mutations MT19, MT22). Parked at a Toggle — exactly AF1's "check after the push" — nothing can. Never claim
   otherwise in a doc.
6. **The class ids mean nothing until the NAMES match** (D7): the Classroom model's id 3 is `blueRing`; the
   Override's id 3 is `yellowPin`. The names may read wrong "for the first several hundred milliseconds" — re-ask,
   never cache a failure as a mismatch forever, never cache a match across a re-plug. **And there is NO class
   count** (re-check, T1): `aivision_get_class_name(port, -1, …)` returns 1 in the vendored binary whatever the
   header says — read each expected id's name into a ZEROED buffer, never compare a "count", and withhold an Object
   whose id is past the expected table. Where the PROS header and `libpros.a` disagree, the binary is the fact
   (`arm-none-eabi-objdump -d` on the extracted `.o` — SER1's method for HA-190/191).
7. **The camera's spelling of the class names is unread** (§0.5 item 5) — a wrong expected spelling withholds every
   object (safe, but dark): the boot line must say which name differed, not just "model DIFFERENT" — through
   `IVisionFrames::status().modelProblem`, never by downcasting to the PROS adapter (re-check T2).
8. **Two focal lengths.** 74° × 63° over 320 × 240 is NOT square pixels (212.3 vs 195.8 px); a single "FOV" constant
   is MG3's mutation and moves every range by ~8 %.
9. **Per corner, never overlapped intervals** — the pitch error is common to both edges of one box (P4b: 129 vs 86).
10. **The host camera model must stay independent** (the A2 rule): it includes nothing from `vision/` and projects in
   the field frame with its own code. A shared helper "to avoid duplication" would hide a sign error in both (MG1,
   MG2 prove the tests see one today).
11. **The station is not the library.** Station 12 writes the camera's settings its own way (TAGS on, FAMILY); the
   LIBRARY page constructs `ProsAiVision`, which writes TAGS OFF and re-checks its settings every second — the two
   must never run at once on one camera (a constant re-write fight). Switching pages re-writes and logs.
12. **`api_doc_tool` fails the BUILD** on an undocumented public entity: the prototypes have plain `//` comments on
   some members (`CameraCorner`'s fields) and several declarators per line — one `///` per member.
13. **`-Wswitch` under `-Werror`:** appending `DeviceKind::AiVision` breaks every exhaustive switch over it until each
   gains a case — exactly three at the tip (compiled, re-check T5): `deviceKindWord` (`hal/device_registry.hpp`), the
   provider's `foundWords` (`hal/pros/device_provider.hpp` :612), and **the test's own `foundWords`**
   (`test/mechanism_devices_test.cpp` :69, whose `kEveryKind` :63-66 must also gain it). Appending `PortUse::Camera`
   breaks one more: `portUseWord` (`config/robot_config.hpp` :539). The provider's word for `Vision` stays "a vision
   sensor"; `AiVision`'s is "an AI Vision sensor" (the test mirrors both).
14. **Line numbers drift.** Everything marked (A1) was MECH-A1's working tree and is now the tip's (re-checked at
   `2506067`, re-check T10); MECH-A2 (merging as this was re-checked), A3–A5 and SER1's successors move them again.
   Re-read; never paste a line number from this brief into code or a doc.
15. **"Simulator" is not this project's word** — the host test plant; the host camera model.
16. **No invented value, ever** (HA-111's lesson): robot two's descriptors and mount stay UNSET until Step 0
   measures them; a placeholder "so the tests pass" is the defect class this project has paid for before.
17. **Season data is dated.** `override_elements.hpp` is v2.0's (or v2.1's, if the executor finds it); a manual
   change to pA8/pA10/pA15 or the Pin/Cup sizes is a one-header change with its citation — never a silent edit.

18. **A slanted band is not a short one (re-check R7).** Off the optical axis a horizontal edge projects SLANTED, and
   the box of a slanted band is taller than the band: a turned robot's field face fails "one camera explains it" while
   the thin bottom face under it can pass. The colour-above veto is what keeps that from becoming a wrong colour —
   never remove it to "get coverage back" when turned; square up instead (AF1's approach).
19. **A frame is older than its `readAt`** (re-check S6). Never pair a frame with the pose at read completion while
   the robot moves without widening the bound by the measured image age (`boundAtImage`); an UNSET age means "only
   when stopped", never "assume zero".
---

## 9. Summary

**What VIS1 is:** the AI Vision camera as library code — read a FRAME at a time (beside the frozen `IVision`, which
is untouched), configured by the library every boot (colours the team measured, the Override AI model checked BY
NAME, tags off), turned into directions and distances that carry their error, and two consumers that say only what
they are sure of: the **Toggle tracker** (RED / BLUE / YELLOW or UNKNOWN with the reason) and the **piece finder**
(FOUND / NONE / AMBIGUOUS / NOT IN VIEW / UNKNOWN for ONE named piece), plus the **camera feed** that reads once per
period for both. AF1, L4 and ALLY1 build on it; VIS1 builds none of them.

**Decisions for the team lead (each with a recommendation):** D1 three runs (A1 after MECH-A1 merges; A2 overlapping;
B between MECH-A5 and A6) · D2 the camera's facts in the robot file (`CameraConfig`) · D3 Bench Tests station 12
gains colours, the library's measurements and a LIBRARY page · D4 `DeviceKind::AiVision` (the registry stops calling
two sensors one name) · D5 a refused camera never refuses the robot · D6 a read reporting the camera's cap (24) or more is "possibly full", never evidence (re-check S4) · D7 the
AI model checked by name before an id means anything · D8 our colour is its own small chunk, ALLY1 · D9 the mount:
a lens ~10–11 in up, level — never 11¾–12¾ in level, the blind band (re-check S1) · D10 read from the consumer's loop through one feed, no library task ·
D11 `IVision` stays absent.

**Sizes (rough builder-days; verification ~half again on top):** VIS1-A1 ~2½–3½ · VIS1-A2 ~2¼–2¾ · VIS1-B ~1¾–2¼ —
**VIS1 ~6½–8½** (re-check: was ~6–8; row 18d said ~4¾–6½ before the objects of 10-14) · ALLY1 ~1–1½ (separate). **Step 0's first four
steps need no code and can run at the next bench session.**

**Files:** NEW `hal/vision_frames.hpp`, `hal/pros/ai_vision.hpp`, `hal/fake/fake_vision_frames.hpp`,
`config/camera_config.hpp`, `vision/{camera_geometry,override_elements,toggle_tracker,piece_finder,camera_feed}.hpp`,
`sim/camera_plant.hpp`, `test/pros_shim/pros/ai_vision.h`, `src/programs/bench_camera.hpp`, ~13 test files;
CHANGED `hal/{device_registry,device_provider}.hpp`, `hal/pros/{device_registry,device_provider}.hpp`,
`hal/fake/fake_device_provider.hpp`, `config/robot_config.hpp`, `robot/robot.hpp`, `src/robots/chassis_table.hpp`,
`src/common/boot_source.{hpp,cpp}`, `src/programs/bench_r3a.cpp` (station 12 only), the registry/builder/boot-source
tests; docs per §3.

**Probes (all re-run by this writer, `scratch/brief-VIS1/v4/`):** P1–P8 — 10 probe programs (8 test suites, 90 cases; 2 sweeps); 32 + 10
mutations, every one seen RED; 222 headers clean on the ARM gate. **Re-check (`scratch/brief-VIS1/v6/`):** P1–P8 and
R2–R7 — 14 test suites, 114 cases, all passing, plus the sweeps P3, P4b, R1; 41 tracker/finder/geometry mutations
(MT1–MT25, MF1–MF9, MF1b, MG1–MG6) and 14 adapter mutations (M1–M14), every one seen RED (four survived until a test
was written: M13, MT12, MT22, MT24); 225 headers clean on the ARM gate over `2506067`. **The re-check found and
fixed:** the model check reading a class count PROS never returns (T1); the boot line's model words needing a
downcast (T2); a never-compiled `ChassisTable` change (T3); a read at the camera's cap of 24 trusted as complete (S4);
a frozen camera undetected while turning (S3); a cut band of another colour never vetoing (S7); short side-clipped
lookalikes (S8); the image's unmeasured age breaking the fix's bound (S6); a blind band where D9 put the lens (S1);
and — in neither checker's list — the bottom face's colour reported "sure" with the robot turned to a Toggle (R7). **This run found and fixed in the prototypes:** a
frozen-camera latch (the tracker never believed a recovered camera — T7); the finder answering NONE from a read whose
objects were withheld (P7); the Toggle's face geometry carried as one guessed number (now an interval — T9); and a
sliver of the Toggle's bottom face measured as a far face (a minimum band height — T8).

**UNVERIFIED (each a §4 entry or a Step 0 question):** whether manual v2.1 is out and changed any Toggle fact; the
optics (74° × 63°), the frame rate, the read cost; whether a PROS program sees the Override model's names as VEX's
page spells them, and whether the model and the settings persist on the sensor; the detection order and the score
scale; whether the model's box encloses the whole piece; the Toggle's real face heights and its distance from the
wall; whether the three colours separate under event light and which lookalikes fire; where VEXcode shows a colour's
six numbers; whether the AI Vision Utility reaches the camera over its own USB-C; the box jitter. **Nothing in VIS1
has run on a robot, and no AI Vision evidence exists yet.**
