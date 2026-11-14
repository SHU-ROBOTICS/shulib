# G1 — ROBOT ONE (and its twin) IN THE CODE: a robot id, five builds, its tables and a FACTS LEDGER that compiles and is tested on the host test plant with marked placeholder numbers but REFUSES to build an upload, by name, until each owed fact is typed — the hand-written path, with robot two untouched

> **THE TEAM LEAD'S ANSWER (2026-11-11 ~02:00): "Go, but keep the queue"** — D1–D13 accepted as recommended, EXCEPT D11: robot
> one's build does NOT jump the queue; it follows the checked Toggle step (AF1S-A1) and curves (CURVE1-A). He was told that
> kept in the queue, robot one's first Drive likely lands at or after the ~Nov 18 freeze. Filed as ROB1-* (G1 is a finished
> chunk). ROB1-C (station 13 applies the cap) follows ROB1-A. The facts form goes to the builder now.

> **THE TEAM LEAD'S RULINGS THIS BRIEF HOLDS (BINDING), as given to this writer (2026-11-10):**
> - **No panel before the first competition (2026-11-21; code changes stop ~Nov 18).**
> - **SKILLS ONLY at it.**
> - **BOTH robots run the skills route** — "both robots but they will be structurally similar so building one will be
>   like building the other".
> - **Robot one is built ~Nov 14.** The team's builder, 2026-11-05: **both robots identical, 15"**; robot one has the
>   **4-stage string CASCADE lift — strings both ways, no limit switch, homed by stalling gently DOWN into its bottom
>   stop under MECH-S-A's current cap; preset levels by stack height**; its **motors, ports, gears, spool and heights
>   NOT KNOWN YET**; **sensors: only VEX visual sensors (AI Vision, one at the front) and rotation sensors**; **a flipper
>   whose kind — motor arm, pneumatic or passive bar — the builder still owes**; **no intake this competition**; **a
>   passive toggle is mechanical**.
> - **The panel's robot file generator (G3a Part 1, `tools/robot_tool.py`, branch `g3a-p1`) is UNMERGED and out of
>   scope — this is the HAND-WRITTEN path.**
> - **Robot two** (`src/robots/chassis_table.hpp`, `mechanism_table.hpp`, `src/common/driver_file.hpp`,
>   `src/team/autons.{hpp,cpp}`, the 7 builds of `tools/src_build_gate.py`) **is the worked example to follow, not to
>   change.**
> - **G4 (the guardrail):** the team's competition code — their autons and their driver choices — is the students'; the
>   coordinator's builder may write the robot's DECLARATIONS from facts the team gives and the plumbing (ids, build
>   targets, the gate's builds), and **must leave autons as the team's (a starter skills auton file with the 40 in
>   drive only, or none).**
> - **The order (AF1-S's answers, 2026-11-10):** the queue is kept — the RPV1 brief, then this brief; robot one's
>   BUILD waits for CAL1-B's merge (the same robot-selection files).

> **Status 2026-11-11 ~01:00: DRAFT for the team lead's go — WAITS for his answers to D1–D13. Nothing is built.**
> **RE-CHECKED 2026-11-11:** a checker's eleven findings were each re-verified (four new probes, P3–P6); all eleven
> are real and each is fixed in place — every changed passage is marked **[R1]**…**[R11]**, and the list with its
> evidence is **"## Re-check — amendments"** at the end. New: **D13** (station 13 and the current cap) and run **G1-C**.
> Written by the coordinator's brief writer from a READ-ONLY pass over branch `shulib-v2` at **`16c01e0`** (main's tip,
> L6-A2, committed 2026-11-10 23:12). Every `file:line` below was OPENED at `16c01e0` (`git show 16c01e0:<path>`;
> `~:N` = within a few lines) unless it says **(MECH-S-A, staged)** — the main worktree's INDEX carries the MECH-S-A
> squash, staged and uncommitted (`git show :<path>`) — or **(CAL1-B, done)** — the worktree `.claude/worktrees/cal1-b`
> on `858b675`, finished, merging after MECH-S-A (NOTES :965). Worktrees read, never edited: `cal1-b`, `mech-s-a`,
> `step1-a` (done, merging third — its `src/` changes are `auton_tester_panel.hpp`, `auton_tester_run.hpp`,
> `competition_run.hpp`, `competition_screen.hpp`), `tune1-a` (no `src/` change), `vis1-a2` (library only), `g3a-p1`
> (unmerged; it changes `chassis_table.hpp`, `test/CMakeLists.txt`, the gate, and adds `src/robots/robot_two.robot.json`).
> Briefs read: AF1-S (`briefs/AF1S-skills-toggle-check-flip.md`, its "Mechanism gaps" table :500-516 and D9 :306-322),
> RPV1 (`briefs/RPV1-route-preview.md`, D3 :78-92, §1.2 :374-387), CURVE1 (HA numbers). **Probes** in
> `.claude/coordinator/scratch/brief-G1/` (§0.4, `PROBES.txt`), every binary under `(ulimit -v 4194304; timeout 300 …)`.
>
> **Executor model: `opus`** (never `fable` unless the team lead says so). **Do not commit.** Say "host test plant",
> never "simulator". **No hardware claim anywhere: robot one does not exist in this project's records** — no census,
> no reading, no photo; robot two's claws and the drive program have never run the library on a robot. **Hardware
> Assumptions: HA-252..HA-257 ONLY** (§4). **Memory rule (2026-10-15):** every binary built — probes, the suite, the
> gate's builds — runs as `(ulimit -v 4194304; timeout 300 …)`; the suite only through `python3 tools/suite_run.py`.
>
> **A NAME COLLISION, found while writing (D1):** "G1" is already a finished chunk — `docs/internal/chunks/G1-PROGRESS.md`,
> `G1-COMPLETED.md`, `G1-robot-config-and-builder.md`, `G1-part2-pros-provider-and-main.md` (the robot config and
> builder, 2026-09; "G1 is complete", commit `37706ce`). A run of this chunk writing `G1-PROGRESS.md` would append to
> that record. **This brief calls the runs G1-A / G1-B / G1-T (the coordinator's queue name) but files them as
> `ROB1`** — live log `docs/internal/chunks/ROB1-PROGRESS.md`, created FIRST, a heading per run (D1 (d)).

---

## DECISIONS FOR THE TEAM LEAD (a recommendation for each; nothing below decides them silently)

**D1 — The names. RECOMMENDED (a).**
- **(a)** Robot one is **`RobotId::RobotOne`**, built with **`make ROBOT=one`** (+ `PROGRAM=drive|calibrate|autontest|
  competition`); its code names are `kRobotOneChassis`, `namespace robot_one` (`kRobotOneMechanisms`),
  `src/team/robot_one_driver.hpp` (`RobotOneDriver`), `src/team/robot_one_autons.{hpp,cpp}`
  (`registerRobotOneAutons`), its facts ledger `src/robots/robot_one_facts.hpp`; its build define
  `SHULIB_ROBOT_ONE_2026`; its beacons `shulib-robot-variant=one`, `one-drive`, `one-calibrate`, `one-autontest`,
  `one-competition` and `shulib-robot=robot-one`; the name on its screens "2026 CASCADE ROBOT (robot one)". **Robot two
  keeps every name it has** — `ROBOT=tank`, `RobotTwo`, `robot_two` — so its seven builds' commands, docs and gate rows
  do not move.
- **(b)** Rename robot two's `ROBOT=tank` to `ROBOT=two` at the same time, for symmetry. **Cost:** every doc, command,
  gate row and the team's notes change five days before the freeze, for no new ability. After the 21st, if wanted.
- **(c)** `ROBOT=cascade`. **Cost:** the twin will have a cascade too; the name would stop meaning one robot.
- **(d) The chunk's FILES** (not a choice about the robot): `ROB1-PROGRESS.md` / `ROB1-COMPLETED.md`, because G1's
  names are taken (status note above). **What I need from you:** do the students call the robots anything else?
  The screen name is one string (the chassis table's `robot`), changed in a minute.

**D2 — One shared declaration, or two copies? RECOMMENDED (a).** *(You said: "building one will be like building the
other".)*
- **(a) One region PER ROBOT, written the way the guide teaches (chapter 18's literal declarations), whose every
  robot-specific number comes from THAT robot's facts ledger — plus a TWIN TEST that proves the two regions are the same
  text except for the robot's name and its ledger.** The day robot two is rebuilt as the twin (D9), its region is robot
  one's region copied with `robot_one` → `robot_two` and its own ledger; the test then fails on any other difference
  (a preset renamed on one robot only, a motor added to one, a `Hold` turned into `Coast`). What students read stays
  the guide's form, and what G3a's generator will later write stays the same form.
- **(b) One declaration in code, made by a function of the facts** (`makeCascade(kRobotOneFacts)`), or a template over
  the facts. **Cost:** a student reading the robot file no longer sees a declaration the guide shows; the src build
  gate's per-robot rules (which recognise a robot by `namespace robot_<name>` — `src_build_gate.py` :680-760) would need
  rework; G3a's generator writes literal declarations.
- **(c) Two free copies.** **Cost:** nothing stops them drifting — one robot's "L3" quietly becomes 1 in higher.

**D3 — How the code is written BEFORE the facts exist: a FACTS LEDGER that REFUSES the upload build, by name.
RECOMMENDED (a).**
- **(a)** Every number about robot one that is not known yet is written ONCE, in `src/robots/robot_one_facts.hpp`, as
  `owed(<a placeholder>, "<what it is, in plain words>")`; when the builder gives it, it becomes
  `typed(<the placeholder, kept>, <the value>, "<what>", "<who, how, when>")` — **[R10]** the value in its own place, a
  value still equal to the placeholder refused (P3b). The declarations read ONLY these. **On the host** everything
  compiles and the host test plant runs robot one's cascade, homing and levels on the placeholders (they are chosen to
  pass every rule the library checks — probes P2, P3). **For the robot**, each fact says which programs need it, and
  `make ROBOT=one PROGRAM=…` **fails** while any fact that program needs is owed, with one line per owed fact naming it
  (probe P1b: `static assertion failed: robot one: OWED -- the cascade's top-stage height at its top stop
  (src/robots/robot_one_facts.hpp)`). **[R1] That refusal only fires if `make` recompiles.** Today it does not when
  only `ROBOT=` or `PROGRAM=` changes: objects depend on sources and headers, not on the flags (`common.mk` :26-30,
  :266), so after another build `make` compiles nothing and re-links the previous robot's or program's package —
  guide 16 :56-59 tells people to type `rm -rf bin .d` for exactly this. **So G1-A adds a VARIANT STAMP to the
  Makefile** (§1.1; probe P5): `make` writes `ROBOT=`, `PROGRAM=` and the build's defines into `.d/shulib-variant` and,
  when they differ from the last build's, deletes `bin/` and `.d/` before compiling. With it, every change of robot or
  program recompiles everything, and a REFUSED build leaves NO package in `bin/` to upload. With the stamp, **no
  placeholder can reach robot one's brain in a program that uses it, from a clean tree or not.** (Left: a same-variant
  rebuild that fails leaves that SAME variant's last good package — the right robot and program, the previous facts.)
  Two stages (§2.0):
  - **Stage A — what is plugged where** (each mechanism's ports, cartridge, gear teeth, and its motors' directions AS
    THE BUILDER SEES THEM — station 13 then confirms; **every motor port the Devices screen lists that is not a drive
    motor** — the build refuses while any of them is undeclared (D6, [R9]); the flipper's KIND, which may be typed as
    "not on the robot yet"; and the cascade's **STARTING** current cap and stall signature, **chosen, not measured**
    (landmine 17's words) at the bench from chapter 18's order rule, because station 13 cuts out on the stall
    signature and, under D13, writes the cap — **[R2]** they cannot wait for station 13's RECORD, which needs Bench
    Tests built): needed by **all five** programs, Bench Tests included (it powers declared mechanisms).
  - **Stage B — how it moves** (the cascade's measured height rows, its top, its levels, its hold volts, its homing push;
    the flipper's angles or move time): needed by **Drive, the Auton Tester and Competition** only — so Bench Tests and
    Calibrate run on the robot as soon as Stage A is typed, and Stage B is MEASURED with them. **[R2]** The cap and
    signature MEASURED at station 13 are typed later over the starting ones (a G1-F edit; every program rebuilt).
  - **[R9] The camera's port gates nothing:** no G1-A code reads it (D12), so it is recorded in the ledger as "read by
    VIS1-B" and never stops a build.
  - The drivetrain's facts need no ledger: the chassis table already leaves an unknown value UNSET and every station
    and program that needs it refuses by name (`chassis_table.hpp` :24-28, `describeMissing` :298-311); Calibrate
    discovers the drive ports, their directions, the sides, the scale and the track width itself.
  - **The src build gate proves both halves before Nov 14:** on the real tree each of robot one's five builds either
    links with nothing it needs owed, or fails naming exactly its owed facts; and **[R4] on a COPY of the working tree
    under `build/src-gate-facts/`** (gitignored; never the repo's own files) with every owed fact typed with the source
    `"gate copy"`, all five link and pass every check the seven builds pass. The repo's ledger and `bin/` are never
    touched by that half, so a crash, an out-of-memory kill or a power loss mid-proof leaves nothing behind in the
    repo; the gate deletes the copy when it finishes and sweeps a leftover one at its start; and the ledger's own
    check refuses a `"gate copy"` source at `make` (the gate turns that one check off in the copy only), so a copied-back
    ledger can never link. So on Nov 14 typing the facts is the only thing between the team and a working upload.
- **(b) Refuse at BOOT instead:** the build always links; each robot-one program's first act paints the owed facts and
  builds nothing. **Cost:** a package carrying placeholder numbers reaches the brain; the refusal is one `if` that a
  later edit can move below a device's construction; the builder learns of an owed fact only after an upload.
- **(c) Both.** **Cost:** (a)'s work plus a boot check that can never fire in a build that compiled; little gained.
- **(d) No placeholders — declare each mechanism only once measured** (AF1-S's G6, `briefs/AF1S-…` :513). **Cost:**
  nothing of the cascade is host-tested before Nov 14; Bench Tests' station 13 cannot jog the cascade to MEASURE it
  (it powers declared mechanisms only); and an undeclared mechanism motor is a drive CANDIDATE for Calibrate, which
  POWERS a candidate that turns with the push (`calibration/calibrate_program.hpp` :8-16, :132-138 (CAL1-B, done)).

**D4 — Which programs robot one gets, and the src build gate's builds. RECOMMENDED (a).**
- **(a) All five, in the same slots as robot two's** — **Bench Tests** (slot 3: the census, MOTOR WATCH, station 13's
  direction check and the cascade's measurement), **shulib Calibrate** (slot 4: CAL1-B's standard Calibrate needs NO
  line for robot one — `src/robots/calibrate_facts.hpp` maps any robot's tables (CAL1-B, done)), **shulib Drive**
  (slot 1), **shulib Auton Tester** (slot 2), **shulib Competition** (slot 6). The gate grows **7 → 12 builds** (`one`,
  `one-drive`, `one-calibrate`, `one-autontest`, `one-competition`) — **[R11]** the five new rows ARE the facts proof's
  real-tree half (their verdict accepts "fails naming exactly its owed facts") — **plus the copy's five builds**:
  ten more clean builds at ~8 s (P0) plus the copy, so the gate grows by **about 1½ minutes** (about 2 if the builder
  finds the rows cannot serve as the real-tree half and builds them twice).
- **(b) Drop the Auton Tester.** **Cost:** the team loses the bench route runner on the robot that runs the cascade
  route; saves one gate row.
- **(c) Drive and Competition only.** **Cost:** no station 13 → no direction check and no way to MEASURE the cascade;
  no Calibrate → Drive has no record to boot from.

**D5 — The cascade's declaration. RECOMMENDED (a).**
- **(a)** A **`Lift` with its motors (two to four, `{Forward, Reversed}` when mirrored — `kinds.hpp` :126-150,
  `kMaxMotorRoles` 4), its cartridge, `gears(motor, spool)`, a MEASURED shape — `measured({...})` with a row at the
  bottom, at every level and at the top (three to eight rows, `kinds.hpp` :390-402) — `Hold` when stopped, a stall
  signature, MECH-S-A's `.currentCap` (MECH-S-A, staged), `range` = bottom..top, `constantGravity(V)`,
  `homeByStall(0_in, −V, signature)` (your builder's ruling: stall gently DOWN; no switch), and up to six named levels
  by stack height (`kMaxPresets` 6)** — every number from the ledger. Probe P2 compiled this SHAPE (host and ARM) and
  saw the library refuse, by name, a stall at or above the cap, a homing push UP, a level above the top, rows out of
  order. **[R8]** P2 itself typed its ports, teeth and levels as literals; **probe P3 (re-check)** compiled it with
  EVERY number read from a ledger — ports, directions, cartridge, teeth, rows, top, stall (amps, rpm, time), cap, hold
  volts, homing push, unhomed volts, the levels — host and ARM `-Werror`. The levels cannot be passed as one array
  (`.presets = kLevels.value.p` fails: "array must be initialized with a brace-enclosed initializer"), so the
  declaration spells **all six preset slots** from the ledger, `{lv(i).name, lv(i).at}`, an unused slot `{nullptr,
  0_in}`; the rows' count is the ledger array's size. **The motor COUNT is structural**: a third or fourth motor is
  a declaration edit, not a ledger edit. **Why rows at every level, not two:** a string winds onto a spool in layers, so the height per turn changes as
  it climbs (HA-254); the shape interpolates straight between rows, so a row AT each level makes each level exact
  whatever the curve between.
- **(b) Two rows only** (bottom and top). **Cost:** a level between them is off by however much the spool's layering
  bends the line — unmeasured.
- **(c) `spool(d)`.** **Cost:** wrong for a cascade — `spool` is "carriage inches = string inches" (`kinds.hpp`
  :373-376); a 4-stage cascade's top stage moves a multiple of that.
- **Homing for a skills run** (not a code choice — said so the team writes it): the route's first line
  `r.startHome(cascade, 2_s)` homes while the first leg drives; `homeAtDriverStart(cascade, …)` in the driver file
  homes at driver skills' start (both exist — `routine.hpp`, `driver_inputs.hpp` :295-303).

**D6 — The flipper, whose kind is still owed. RECOMMENDED (a).**
- **(a) G1-A writes NO flipper into robot one's file** but ships all three forms ready and PROVEN: a host test compiles
  and runs robot one's list with each — **a motor arm** (`Arm`: `{"stow", 0}, {"flip", F}`, `restsAtStop(0)` or
  `homeByStall`, a stall signature, a cap — `wire(flipper, motors(p))`), **a pneumatic** (`Pneumatic`: `"stow"` /
  `"flip"`, start and safe `"stow"`, the measured `.moveTime` — `wire(flipper, lines('X'))`), **a passive bar**
  (nothing to declare) — with each one's default control and its fit on the Drive screen's row. The day the builder
  names it, the matching block is pasted (the coordinator's ~½-hour facts edit, G1-F). **[R9]** A flipper PORT typed in
  the ledger protects nothing — CAL1-B's `calibrateFactsFor` reads only the mechanism LIST (`calibrate_facts.hpp`
  :60-80 (CAL1-B, done)). What protects Calibrate is a Stage A fact, **"every motor port on the Devices screen that is
  not a drive motor"**, and a build check that each one is DECLARED in robot one's list (probe P4: refused, by name,
  when port 10 is listed and undeclared). A flipper motor on the robot therefore stops every robot-one build until its
  block is pasted; a flipper not yet on the robot is typed as "not on the robot yet". A typed kind "motor arm" with no
  arm declared is refused too.
- **(b) Ask now and write the one form in G1-A.** **Cost:** G1-A waits for an answer the builder may not have until
  the robot exists.
- **(c) Write the arm (the likeliest) and change it if wrong.** **Cost:** a wrong declaration on the robot is worse
  than none — Calibrate and the boot would take an arm's port for a mechanism that is a pneumatic.

**D7 — The controller map (your builder: "buttons can come later"). RECOMMENDED (a).**
- **(a) A DEFAULT driver file, marked in the file and on Drive's boot line as "a default — the team's to change"**,
  because the build REFUSES a mechanism with no control (`refused::a_mechanism_has_no_driver_control…`, P2;
  `driver_file.hpp` :18-24): **left stick drive, right stick turn (robot two's feel); L1 / L2 the cascade up / down
  while held (12 V, soft-stopped once homed); the d-pad Down / Left / Up / Right = the bottom and the first three
  levels; B = home the cascade; the cascade homes once at driver start; the flipper on R1 / R2 (arm: while held; pneumatic:
  R1 toggles); Y left free (the drive loop's re-zero).** Probe P2 compiled all three flipper variants of it. Under G4
  this is the one piece of "driver choice" the builder writes, only because nothing builds without it; the students
  own it from the day it lands.
- **(b) The students write it from guide chapter 21 before robot one drives.** **Cost:** no Drive or Competition build
  for robot one until they do — on a ~2-day window (§9).
- **(c) Drive and the cascade jog only; every level and the flipper `autonOnly`.** **Cost:** driver skills without
  level buttons or a flipper.

**D8 — Robot one's autons file. RECOMMENDED (a).**
- **(a) A starter `src/team/robot_one_autons.cpp`** holding robot two's **"40in forward" copied, registered as a
  SKILLS auton** (60 s period, `autons.hpp` :24-25 — the only competition is skills) and the library's acceptance run
  — and a comment where the team's skills route goes. Nothing else: the skills route is the students' (G4).
- **(b) None** — an empty `registerRobotOneAutons`. **Cost:** the first upload of the Auton Tester shows an empty menu;
  nothing proves robot one's auton path end to end on the host.
- **(c) The 40 in as a MATCH auton, exactly robot two's.** **Cost:** it cannot be picked for a skills run.

**D9 — Robot two, and the twin. RECOMMENDED (a).**
- **(a) Robot two stays exactly as it is in G1-A** (its tables, driver file, autons and seven builds unchanged — §2.2
  T6 compares machine code, strings AND constant data, [R5]). **The twin** — the second robot built like robot one — is a later, small run,
  **G1-T**, the day your builder says robot two has been rebuilt: robot two's region becomes robot one's copied with its
  own ledger (D2's twin test then passes or names the difference), its claws retire (or stay, if kept), its driver
  file and its record are re-made (Calibrate again: the drive may have changed).
- **(b) Convert robot two now** to the twin with every fact owed. **Cost:** robot two's five builds would refuse until
  its rebuild — a working robot made unusable before it is rebuilt.
- **What I need from your builder:** is "both identical" robot two REBUILT to robot one's design, and when? If before the
  freeze, G1-T must land by ~Nov 17 (§9).

**D10 — A calibration record names its robot. RECOMMENDED (a).**
- **The finding:** both robots read their calibration from the SAME file name on their SD card
  (`calibration_record.hpp` :92, `shulib_drivetrain.bin`), and the boot check recognises a card carried from one robot
  to the other only by its PORTS (`boot_check.hpp` :6-12 names exactly that case) — on two near-identical robots wired
  the same, **robot two's card in robot one's brain passes**, and robot one drives on robot two's measured directions,
  scale and track width without a word. Calibrate writes `"shulib Calibrate <stamp>"` as the record's provenance
  (`include/shulib/calibration/calibrate_program.hpp` :685 at `16c01e0`, :684 in CAL1-B's worktree) — no robot.
- **(a) G1-B:** Calibrate writes the robot's name into the record's provenance (the 96-byte field,
  `drivetrain_description.hpp` :145); the boot (`src/common/boot_source.hpp`) refuses a record that names ANOTHER robot,
  by name ("this card's record is robot two's — run shulib Calibrate on THIS robot"), accepts its own, and — for robot
  two's existing records, which name nobody — accepts a nameless record on robot two with a warning and refuses it on
  robot one. ~½–¾ builder-day, after CAL1-B merges (its library file and goldens).
- **(b) Procedure only for the 21st:** one SD card per robot, labelled, never moved. **Cost:** HA-257 (a belief), and
  the failure is silent when it happens.
- **(c) A per-robot file name.** **Cost:** a library constant both robots share becomes per-robot — a bigger change
  than (a).

**D11 — When G1 builds, against the queue. RECOMMENDED (a).**
- **(a) G1-A takes the NEXT free build slot after CAL1-B merges — ahead of AF1S-A1, TUNE1-B1 and STEP1-B** (NOTES :963,
  :965 list them first). Nothing of robot one — the cascade first — can be built, driven, tuned or tested without it
  (AF1-S :512), and robot one is built ~Nov 14 with the freeze ~Nov 18 (§9's dates).
- **(b) Keep the queue** (G1-A after AF1S-A1, TUNE1-B1, STEP1-B, CURVE1, RPV1). **Cost:** G1-A merges ~Nov 17–18 at
  best ([R1]–[R10] grew it); robot one's first Drive lands at or after the freeze.
- **[re-check] G1-C** (D13) is a separate small run in a SECOND slot beside G1-A (its files are Bench Tests' only); it
  needs only MECH-S-A merged.

**D12 — Robot one's camera block. RECOMMENDED (a).**
- **(a)** G1-A records the camera's port and mount in robot one's ledger (**[R9]** recorded, gating NO build — nothing in
  G1-A reads them), and the code that puts
  a camera into a booted robot is **VIS1-B's** (not built yet — it designs that path for robot two: the table's camera
  block and `boot_source`) — VIS1-B then wires BOTH robots' blocks in one run. AF1S-A2 (the camera in autons) needs it.
- **(b) G1-A designs the camera path for robot one now.** **Cost:** G1-A invents the template VIS1-B is briefed to
  design (VIS1 §1.15), +~½ day, and VIS1-B then re-does it.

**D13 — [R3] Bench Tests' station 13 and the current cap. NEW at the re-check. RECOMMENDED (a).**
- **The finding:** station 13 is where robot one's cascade is powered FIRST (the direction check, then jogging it up
  to each level and the top stop to measure it), and it **never applies MECH-S-A's cap**: the cap is written into the
  motors only when the mechanism SERVICE binds (`service.hpp` :111-137 (MECH-S-A, staged)); the parts "write no limit
  itself" (`mechanism_parts.hpp` :33-35 (MECH-S-A, staged)); station 13 deliberately builds the parts with NO service
  (`bench_r3a.hpp` :237-240); `grep` finds no `currentCap`/`currentLimit` anywhere in `src/` at the tip, in CAL1-B or
  in STEP1-A. Station 13's own cut-outs are 2.4 A for 250 ms and the mechanism's stall signature (:262-266); its
  ceiling goes to 12 V. On a string cascade pulled both ways, that is the over-pull MECH-S-A exists to bound.
- **(a) Run G1-C:** when station 13 first powers a mechanism that STATES `.currentCap`, it reads each motor's limit,
  writes the cap through the same hal call the service uses, reads it back and shows it on the page ("cap 1.20 A per
  motor — set"); on EVERY exit (the station's own and bench_r3a.cpp's guard) it writes back what it read. Robot two's
  claws state no cap (`mechanism_table.hpp`, robot two's region), so robot two's station 13 behaves exactly as today;
  its three packages that link Bench Tests (bench, xdrive, tank) change by design, listed in G1-C's log. ~½ builder-day
  (+~¼), in a second slot beside G1-A, after MECH-S-A merges.
- **(b) Procedure only:** guide 21 and the form say station 13 is UNCAPPED; the ceiling stays at 3 V on the cascade;
  the top stop and the high levels are approached by hand, never jogged into. **Cost:** the first powered moves of the
  string cascade run without the cap; measuring by hand through the gearing may not be possible; a press of R1 lifts the
  ceiling to 6 V.

---

## The design in plain words (for the team lead)

- **What it adds.** A sixth robot id, **robot one**, chosen by the same one switch as every build
  (`src/build_selection.hpp`'s one chain, `make ROBOT=one PROGRAM=…`); its drivetrain table (mostly "unknown — Calibrate
  will measure it"); its mechanism list (the cascade, and the flipper once its kind is known); a default driver file; a
  starter autons file; and five builds the gate checks the way it checks robot two's.
- **The facts it does not have yet.** Every unknown number is written once, in one file, marked **OWED** with a plain
  name and a placeholder. The host suite runs robot one on those placeholders, so everything is tested before the
  robot exists. **The robot cannot get them:** building a robot-one program that needs an owed fact stops with a line
  naming each one ("OWED — the cascade's height at level 2"), and **[R1]** `make` now starts fresh by itself whenever
  the robot or the program changes, so a stopped build never leaves an old package behind to upload. When the builder
  measures a fact, the coordinator types it with who measured it and when — **[R10]** the real value goes in its own
  place beside the placeholder, and the build refuses a "typed" fact still equal to its placeholder — and that
  program builds.
- **The order on the robot (§2.0).** Day one: read the ports off the brain's own Devices screen (no program needed) →
  the Stage A facts typed → **Bench Tests** (the census; station 13 jogs each mechanism once to check which way it
  moves) and **Calibrate** (it measures the drive by itself) → the cascade measured with station 13 (tape height and
  the recorded shaft angle at the bottom, each level, the top) → the Stage B facts typed → **Drive, Auton Tester,
  Competition**.
- **Robot two does not change.** Not a line of its tables, driver file or autons; its seven builds produce the same
  packages (§2.2 T6 compares the machine code, the strings and the constant data — **[R5]** the constant data matters:
  each mechanism records the LINE it is written on, so even one line added above robot two's block changes its
  packages). Under D13 (a), G1-C changes Bench Tests on purpose; robot two's claws behave exactly as before.
- **[R3] Bench Tests caps the cascade** (D13): station 13 writes the cascade's current cap while it jogs it, and puts
  the motors back as they were when it leaves.
- **"The two robots differ only where stated."** Each robot's mechanism block is written the same way from its own
  facts file, and a test fails if the two blocks ever differ anywhere else (once the twin exists).
- **What it does not do.** No panel, no route (the students'), no wall reset on robot one (it has no distance
  sensors), no tracking wheels from its rotation sensors, no camera wiring (VIS1-B's — D12).

---

## 0. Why — what exists, what adding robot two touched, and the probes (every line cited OPENED at `16c01e0`)

### 0.1 Why this chunk, and why now
AF1-S's coverage check found that **robot one has no build at all** (`briefs/AF1S-…` :489, :512): no `RobotId`, no
tables, no driver or autons file, no `ROBOT=` value — "the largest gap for the 21st, and it blocks EVERY other row on
robot one" (the cascade, TUNE1's live tuning, the Toggle steps, RPV1's preview of robot one). Robot one is built ~Nov
14; the freeze is ~Nov 18. Every program after the bench tester is `ROBOT=tank`-only by a Makefile `$(error)` and a
`#error` (below).

### 0.2 What exists in the tree
| What | Where (`16c01e0`) | What G1 does with it |
|---|---|---|
| The one chain | `src/build_selection.hpp` :113-117 `RobotId` {BenchBot, RobotTwo, InventedXDrive}; :130-158 seven branches, each setting `kProgram`, `kRobotId`, `kVariantBeacon`; :167-177 `robotBeacon()` (a case per robot, no default) | +`RobotOne`; +5 branches; +1 beacon case |
| The impossible combinations | :79-81 two robot defines; :82-85 two programs; **:86-97 `drive`/`calibrate`/`autontest`/`competition` need `ROBOT=tank`** | widened to "tank or one" (words kept for robot two's message) |
| The Makefile's switches | `Makefile` :95-104 `ROBOT?=bench` (bench, xdrive, tank, else `$(error)`); :166-193 `PROGRAM`, each of the four `ifneq ($(ROBOT),tank)` → `$(error)` (:170-172, :177-179, :182-184, :187-189) | +`one`; the four guards accept `one` |
| Robot two's drive table | `src/robots/chassis_table.hpp` :69-91 `ChassisTable`; :124-145 `kRobotTwoChassis`; :206-216 `chassisTableFor` (no default); :221 `kChassis`; :227-240 the selection restated by address; :298-311 `describeMissing` (UNSET named) | +`kRobotOneChassis` (UNSET where unknown); +case; +restatement |
| Robot two's mechanisms | `src/robots/mechanism_table.hpp` :105-134 `namespace robot_two` with the `[robot two mechanisms]` markers (:107, :133 — G3a's generator wraps that region); :137 `kNoMechanisms`; :144-157 `mechanismTableFor`; :160 `kMechanisms`; :261-276 the drive check and the selection restated | +`namespace robot_one` region (own markers); +case; +restatement |
| The library's refusals | `kinds.hpp` :40-48 (an UNSET field refuses at build, by name), :303-309 `homeByStall`, :390-402 `measured`, :811-914 `Lift`; `.currentCap` (MECH-S-A, staged); `driver_inputs.hpp` :8-17, :212-325 the controls | used as is — no library change in G1-A |
| A tank needs an IMU | `hal/drivetrain_description.hpp` :183-189 — a description with IMU port outside 1..21 is refused ("0 = UNSET") | a Step 0 question (HA-253): the builder named only AI Vision and rotation sensors |
| The boot from the record | `src/common/boot_source.hpp` :1-62 (record, else table, else refuse); `boot_source.cpp` :1-57 (reads `kChassis`, `kMechanisms`) — robot-generic; **:105-109 `kDeclaredMotorLabel = "claw pivots (declared, not drive motors)"`** and step 5's "a claw" (:440-447) — robot two's words | the label per robot (robot two's bytes kept) |
| Calibrate | `src/programs/calibrate_program.cpp` at `16c01e0` says "robot two" in four strings (:1049-1081); **CAL1-B replaces it with a glue over `src/robots/calibrate_facts.hpp`, which maps ANY robot's `kChassis`/`kMechanisms`** (CAL1-B, done: "absent `robot two MEASURES` (0 each)", its log) | nothing — robot one's Calibrate is free after CAL1-B |
| The driver file pattern | `src/common/driver_file.hpp` :1-24 (a robot's file states only `kControls`, `kFeel`, `kLoop`; `controls(...)` refuses a mechanism with no control, a button twice); `src/team/robot_two_driver.hpp` :46-67 | +`src/team/robot_one_driver.hpp` |
| Programs that name robot two's driver | `drive_program.cpp` :120 (include), :177, :247, :274, :350, :410, :487, :581-585, :594-595, :605, :616-618 (`RobotTwoDriver`, "robot two's driver file" in 8 strings); `competition_program.cpp` :153, :253, :552, :828 | a per-build alias `ThisRobotDriver` and per-robot words (§1.1) |
| Programs that call the autons | `auton_tester_program.cpp` :97, :456 (a string naming `src/team/autons.cpp`), :523; `competition_program.cpp` :152, :745; `src/common/dropped_autons.hpp` :69-72 ("fix src/team/autons.cpp") | a per-build `registerThisRobotsAutons()` and per-robot path words |
| The team's autons | `src/team/autons.hpp` :19-33 (`kAutonPeriods`: match 30 s, skills 60 s; `registerAutons`); `autons.cpp` :1-91 (robot two's: the 40 in, the acceptance run; `using namespace robot_two`) | robot one's own file beside it; robot two's untouched |
| The Drive screen's mechanism row | `src/common/driver_panel.hpp` :391-463 — the row begins `CLAW` (:437), `kNoClawsRow` (:424); static_asserts on robot two's list (:460) and the build's (:462) against the SMALL budget | the row's first word per robot; robot one's list budget-checked |
| The host build | `test/CMakeLists.txt` :83-90 — ONE `src/` `.cpp`, `src/team/autons.cpp` | +`src/team/robot_one_autons.cpp` |
| The gate | `tools/src_build_gate.py` :409-470 (VARIANTS, BUILD_ARGS, VARIANT_DEFINES, VARIANT_BEACONS, BUILD_ROBOT, ROBOT_BEACONS), :531-555 (BUILD_PROGRAM, PROGRAM_ENTRY_SYMBOLS), :675-678 `TEAM_FILE_ROBOT`, :680-760 the per-robot namespace rule (a `src/team/<ns>_*` file may name `<ns>`), :1005-1017 the host CMake rule; "269 detector cases" in `self-test` (CAL1-B's log) | +5 builds; the facts proof (D3); new step-0 rules (§1.1) |
| Station 13's record | `src/programs/bench_r3a.hpp` :1382-1432 — RECORD prints each motor's position (rad and deg) and **the output angle** `output %+.2f deg` (:1430) beside the units reading | the cascade is MEASURED from these (Step 0 C4) — no change |
| RPV1 (brief) | `briefs/RPV1-route-preview.md` :78-92 (D3: robot two in RPV1-A; **robot one in RPV1-B "the day its file exists"**), :385-387 (`--robot two|one` once G1 adds `RobotId::RobotOne`) | G1 keeps robot one's autons PROS-free and host-compiled |

### 0.3 What adding robot two touched — the checklist robot one follows (from the tree at `16c01e0`; robot two arrived over R3b Part 0 `e16f336`, T3, T4 `8d89df7`, COMP-t `0f56301`, G1 Part 2c `37706ce`, COMP-b `237c4fe`, MECH-A5)
- [ ] `src/build_selection.hpp`: an enum value; one branch per program; the guards; `robotBeacon()`'s case.
- [ ] `Makefile`: the `ROBOT=` value and its define; the `PROGRAM=` guards.
- [ ] `src/robots/chassis_table.hpp`: the table, `chassisTableFor`'s case, the selection restated.
- [ ] `src/robots/mechanism_table.hpp`: the namespace region with markers, `mechanismTableFor`'s case, the drive check
      and the selection restated.
- [ ] `src/robots/calibrate_facts.hpp` (after CAL1-B): nothing — it maps the build's tables.
- [ ] `src/team/<robot>_driver.hpp` + the alias the programs use.
- [ ] `src/team/` autons + the call the two auton programs make; `test/CMakeLists.txt`'s `target_sources`.
- [ ] The robot-two words in the programs' strings (Drive's boot lines, the claw row, the boot label, the "fix
      src/team/autons.cpp" sentence).
- [ ] `tools/src_build_gate.py`: the builds' rows, `TEAM_FILE_ROBOT` (if a team file is not `<ns>_*`), self-test cases.
- [ ] Host tests: `build_selection_test`, `chassis_table_test`, `mechanism_table_test`, `boot_source_test`,
      `driver_panel*_test`, the driver file's test, the autons test, the source-text tests that pin `RobotTwoDriver`
      (`drive_program_source_test.cpp` :157, :273-274, :360-361, :453, :486; `competition_source_test.cpp` :396, :405,
      :765-766), **[R6] and the `registerAutons(` pins** (`auton_tester_source_test.cpp` :43 `count(code,
      "registerAutons(") == 1`, :56 the DropEntry-then-register sequence, :120 `find("registerAutons(")`;
      `competition_source_test.cpp` :54, :112 — the same three kinds) and, if Drive's LCD word moves, **[R7]**
      `drive_program_source_test.cpp` :381's `"CLAW ABSENT"`.
- [ ] **[R7]** The boot's step-5 words (`boot_source.hpp` :482-500 "port %d is both a claw (%.23s)…", "never takes a
      declared claw port", the headlines "a CLAW port is a DRIVE port in the record / … typed table") and the LCD's
      `kLcdClawAbsent` "CLAW ABSENT" (`driver_panel.hpp` :549) — robot two's words, false on robot one.
- [ ] Docs: guide 07 (:144-145 the build commands), 16 (:52), 17 (:27), README (:207), `src/team/README.md`, the
      worksheet, the HA register.

### 0.4 The probes (`.claude/coordinator/scratch/brief-G1/`, `PROBES.txt`)
- **P0 — can "robot two's seven packages unchanged" be checked to the byte?** *([R5]: bytes yes; but a moved string
  forces the fallback, and the fallback needs P6's constant-data compare too.)* `git archive 16c01e0` into `tree/`;
  `make ROBOT=bench` twice from clean: `hot.package.bin` differs in **7 bytes**, every one inside a `__DATE__ __TIME__`
  string or `_PROS_COMPILE_TIMESTAMP_INT`; `objdump -d --no-show-raw-insn` identical. An archive copy has no `.git`, so
  no build hash enters. **So the before/after check is real** (T6). A clean `make ROBOT=tank PROGRAM=competition`: 8 s.
- **P1 — the obvious ledger check is wrong twice.** A `consteval` "no fact owed" under `kRobotId != RobotOne || …`
  **fails EVERY build** (an immediate invocation is evaluated even where `||` short-circuits), and GCC 13's message
  names the refusal function and `f.name` — **not the fact**. (P1c: plain `constexpr` short-circuits but still names
  nothing.)
- **P1b — what works:** ONE `static_assert` PER FACT, `kRobotId != RobotId::RobotOne || fact.typed`, its message naming
  the fact: host bench OK, ARM robot two OK, ARM robot one → `static assertion failed: robot one: OWED -- the
  cascade's top-stage height at its top stop (…)`. (G1-A adds the program to the condition — Stage B — §1.1.)
- **P2 — robot one's declarations as G1 would write them** (a ledger feeding a 2-motor stall-homed `Lift` with a 3-row
  `measured` shape and MECH-S-A's `.currentCap`; each flipper kind; the wirings; D7's default map in three variants):
  compile host (`-Wall -Wextra -Wconversion -Wshadow`) and ARM (`-Werror -Wsign-conversion …`) against main's tree
  with MECH-S-A staged. Six mutations, each refused BY NAME by the library: stall ≥ cap, homing pushing UP, a level
  above the top, rows not increasing, the flipper with no control, one button twice. (P2 typed ports, teeth and levels
  as literals — [R8].)

**Re-check probes (2026-11-11; `.claude/coordinator/scratch/brief-G1/chk2/`, every binary under `(ulimit -v 4194304;
timeout 300 …)`):**
- **P3 — [R8] every number from the ledger** (`p3_full_ledger.cpp`, `p3c_every_number.cpp`): the cascade with its
  ports, directions, cartridge, teeth, rows, top, stall amps/rpm/time, cap, hold volts, homing push, unhomed volts and
  six preset slots all read from `Fact`s, and D7's map naming its levels THROUGH the ledger (`cascade[lv(1).name]`):
  host `-Wall -Wextra -Wconversion -Wshadow` and ARM `-Werror -Wsign-conversion` clean. Robot one + Bench Tests →
  exactly the four Stage A lines; robot one + Drive → all nine owed lines, each by name.
- **P3b — [R10] [R4] the typed fact** (`p3b_*.cpp`): a `Fact` holding the placeholder and the typed value apart;
  `typed(1.0, 1.0, …)` (value = placeholder) → `static assertion failed: robot one: TYPED but still the placeholder --
  the cascade's stall signature, A`; the same for a struct fact (the ports); a source `"gate: pretend"` → `… a typed
  fact needs a real source …` (the probe searched for `pretend`; the design's marker is `gate copy`, same check). The library's `HeightAt` has no `==`, so the ledger carries its own field-wise `same()`.
- **P4 — [R9] the undeclared motor** (`p4_undeclared_motor.cpp`): a "non-drive motor ports" fact {8, 9} against a
  list wiring 8 and 9 → clean; {8, 9, 10} → `static assertion failed: robot one: a motor the Devices screen lists is not
  a drive motor and is NOT DECLARED in src/robots/mechanism_table.hpp -- shulib Calibrate would take it for a drive
  candidate and power it` (through `mechanism::declaredMotorPorts`, constexpr).
- **P5 — [R1] the variant stamp** (`chk2/stamp/`, `git archive 16c01e0` + a 9-line stamp before `-include
  ./common.mk`; GNU Make 4.3): `make ROBOT=tank` 11 files compiled; then `make ROBOT=tank PROGRAM=drive` **11** compiled
  (the checker's run without the stamp: 0), beacon `tank-drive`; the same again: 0; back to `ROBOT=tank`: 11, beacon
  `tank`.
- **P6 — [R5] constant data** (`chk2/t6_rodata.py` on the checker's `chk_t6/base.elf` vs `shift.elf` — one comment
  line added above `namespace robot_two`): disassembly and sorted strings showed NO difference (the checker's), but
  `.rodata` compared byte for byte with every word that points into the image masked shows **26 differing bytes**
  (the mechanisms' recorded line numbers 161→162 … and the timestamp); base vs itself: 0. A per-symbol compare
  (`chk2/t6_masked.py`) caught only the timestamp — the mechanism list has no symbol of its own — so T6 compares the
  whole sections.

---

## 1. Scope — IN (every new and changed file, by run)

**Does one run fit? Yes — for robot one, ONE build run (G1-A) is the whole of what the 21st needs from code**, with
**G1-C** (D13, [R3]) beside it in a second slot. The record's robot name (G1-B, D10) is a third small run —
**[R1] recommended and NOT the first cut**: it is the only boot-side check that catches a robot-two PACKAGE uploaded to
robot one (robot one's card names robot one, so robot two's boot refuses it) as well as a card moved between them. The twin (G1-T) exists only when your builder rebuilds robot two. Typing the facts (G1-F) is the
coordinator's own edit each time facts arrive — no builder run.

### 1.1 G1-A — robot one in the code (after CAL1-B and MECH-S-A merge; D11)
**NEW**
- `src/robots/robot_one_facts.hpp` — THE LEDGER. **[R10]** `template <class T> struct Fact { T placeholder; bool
  typed; T typedValue; Stage needed; const char* name; const char* source; constexpr const T& value() const; }` —
  the placeholder and the typed value are SEPARATE fields; `value()` is the typed value when typed, else the
  placeholder; `owed(p, stage, name)` / `typed(p, v, stage, name, source)` (typing KEEPS the placeholder and adds the
  value, so flipping `owed(` to `typed(` without writing a value does not compile — P3b). Every robot-one number not
  known today. Below each fact ONE check macro (P1b's form, P3b's additions):
  `static_assert(!(kRobotId == RobotId::RobotOne && needs(kProgram, f.needed)) || f.typed, "robot one: OWED -- <name>
  (src/robots/robot_one_facts.hpp)")`; a typed fact needs a non-empty source not containing `gate copy` ([R4]); a typed
  value equal to its placeholder is refused unless the source says `same as the placeholder, checked` ([R10]; the
  ledger's own field-wise `same()` — the library's `HeightAt` has no `==`). `needs(Program, Stage)`: Stage A for all
  five programs; Stage B for Drive, AutonTester, Competition; **[R9]** a third value, `Recorded`, needed by no program
  (the camera's port and mount). **NOT** `consteval` (P1). **[R9]** One more Stage A fact, `kNonDriveMotorPorts` (the
  form's B6: every motor on the Devices screen that is not a drive motor), and the check that each is in
  `mechanism::declaredMotorPorts(kRobotOneMechanisms)` (P4) — it lives in `mechanism_table.hpp` below robot one's list
  (it needs the list); and the flipper's kind (`Arm` / `Pneumatic` / `Passive` / `NotOnTheRobotYet`), refused when
  typed `Arm` and no arm is declared. A constexpr `kRobotOneFactCount` / `kRobotOneOwedCount` for the boot line and
  the tests. PROS-free (the host includes it). Every placeholder chosen to pass the library's rules (P2/P3 — the
  placeholders are P3's). **[R11] Its names:** the ledger opens NO namespace called `robot_one` — the gate's step-0
  per-robot rule (`src_build_gate.py` :680-735) derives robot namespaces from the mechanism table file and fails any
  other `src/` file that NAMES one as a whole identifier; the ledger is not a table file nor a `src/team/` file. Its
  facts live in `namespace shulib::bench::robot_one_facts` (a different identifier, which the rule does not match) and
  robot one's region reads them as `robot_one_facts::kCascadePorts.value()`; a gate self-test case proves a ledger
  naming `robot_one` is refused. Banner: what OWED means, how to type a fact, who may (the coordinator from the
  builder's form; G4).
- `src/team/robot_one_driver.hpp` — D7's default map, `RobotOneDriver = DriverFile<…>`, its banner saying "a DEFAULT —
  the team's to change" and listing every button (robot two's banner's form, :1-42).
- `src/team/robot_one_autons.hpp` / `.cpp` — `registerRobotOneAutons(registry)`; D8's starter (the 40 in as SKILLS +
  the acceptance run); `using namespace robot_one;`; PROS-free.
- `src/common/this_robot.hpp` — THE BUILD'S TEAM FILES, selected by `kRobotId` (never by a switch): `ThisRobotDriver`
  (a specialisation per robot that has a driver file; for the bench and X-drive builds — whose programs never run a
  driver — robot two's, exactly today's bytes); `registerThisRobotsAutons(registry)` (a `switch (kRobotId)`, a case per
  robot, no default — `-Wswitch` fails the gate); the per-robot WORDS the programs print (`kDriverFileWords` "robot two's
  driver file" / "robot one's driver file", `kDriverFilePath`, `kAutonsFilePath`, the mechanism row's first word
  `CLAW` / `MECH`, the boot label `kDeclaredMotorLabel`, **[R7] the LCD's `CLAW ABSENT` / `MECH ABSENT`, and step 5's
  refusal words and headlines** — "a claw (…)", "never takes a declared claw port", "a CLAW port is a DRIVE port in
  the record / typed table" — "a mechanism (…)" / "a MECHANISM MOTOR port …" for robot one) — robot two's strings
  byte for byte. **How, so robot one's words are host-testable:** the words are `wordsFor(RobotId)` (a constexpr
  function, a case per robot), and `decideBootSourceFrom` and the panel take them as a LAST, DEFAULTED parameter
  (`= wordsFor(kRobotId)`): every existing call — robot two's tests included — is unchanged, and T10 passes
  `wordsFor(RobotId::RobotOne)`. **The format strings:** `-Wformat=2` is on (`Makefile` :53), so a format may not come
  from a variable; each call picks between two LITERALS with `?:` on the words' robot (GCC checks both arms), robot
  two's literal verbatim (landmine 18).
- `docs/guide/21-adding-your-robot-by-hand.md` and `docs/robot-facts-form.md` (§3).
- Tests (§2.2): `test/robot_one_facts_test.cpp`, `test/robot_one_tables_test.cpp`, `test/robot_one_driver_test.cpp`,
  `test/robot_one_autons_test.cpp`, `test/robot_one_flipper_forms_test.cpp`, `test/this_robot_test.cpp`; compile
  refusals `test/compile_refusals/s74_robot_one_typed_fact_without_a_source.cpp` (and the next free numbers).

**CHANGED**
- `src/build_selection.hpp` — `RobotOne`; five branches (`SHULIB_ROBOT_ONE_2026` alone → BenchTests; `+PROGRAM_*` → each
  program); the guards :79-81 (three robot defines, any two), :86-97 ("needs ROBOT=tank or ROBOT=one"); `robotBeacon()`'s
  case; the banner's lists. Every branch still sets exactly the three constants (the gate's step 0).
- `Makefile` — `else ifeq ($(ROBOT),one)` → `-DSHULIB_ROBOT_ONE_2026`; the `$(error)` lists; the four `PROGRAM` guards
  accept `one`; the comment block's build list. **[R1] THE VARIANT STAMP** (P5's 9 lines, just before `-include
  ./common.mk`): `ROBOT=$(ROBOT) PROGRAM=$(PROGRAM) $(EXTRA_CXXFLAGS)` compared with `$(file <.d/shulib-variant)`;
  when different (and the goal is not `clean`): a `$(info)` line saying so, `rm -rf bin .d`, the new stamp written.
  It covers robot two's builds too (their packages do not change — the stamp is not compiled in) and the build hash
  (a new commit recompiles everything, ~8 s). The builder confirms the gate's parser ignores the `$(info)` line.
- `src/robots/chassis_table.hpp` — `kRobotOneChassis` (`.robot = "2026 CASCADE ROBOT (robot one)"`, ports
  `leftCount = rightCount = 0` (Calibrate discovers them), `cartridge = Unset`, `imuPort = 0`, `measured = false`, the
  provenance "nothing typed — Step 0 (docs/robot-facts-form.md)") — the table's own UNSET rule, NOT the ledger (D3);
  `chassisTableFor`'s case; `static_assert(kRobotId != RobotId::RobotOne || &kChassis == &kRobotOneChassis, …)`.
- `src/robots/mechanism_table.hpp` — `namespace robot_one { // [robot one mechanisms] … // [/robot one mechanisms] }`:
  the cascade (D5), every number from the ledger (six preset slots, [R8]); `kRobotOneMechanisms = mechanisms(wire(cascade,
  motors(<ledger ports>)))`; the non-drive-motor check (P4); `mechanismTableFor`'s case;
  `clearOfItsDrive(robot_one::kRobotOneMechanisms, kRobotOneChassis)`; the selection restated for robot one (:270-276's
  form). **[R5] WHERE:** every line G1-A adds to this file goes **BELOW** robot two's `// [/robot two mechanisms]`
  marker — the ledger's `#include`, robot one's region, its checks — and **no line above the end of robot two's region
  is added or removed**: each declaration and `wire(...)` records its LINE (`std::source_location`, `kinds.hpp` :65-70,
  `wiring.hpp` :623-647) into the list robot two's packages carry, so one line moved shifts them (P6). The banner's
  "who reads it" gains robot one only by re-wording within its existing lines, or in a note below robot two's region.
  (`chassis_table.hpp` records no lines — `grep source_location` finds only `kinds.hpp`, `spec.hpp`, `wiring.hpp` — but
  robot one's table goes below robot two's there too.)
- `src/programs/drive_program.cpp`, `src/programs/competition_program.cpp`, `src/programs/auton_tester_program.cpp`,
  `src/common/dropped_autons.hpp` — `RobotTwoDriver` → `ThisRobotDriver`, `registerAutons` → `registerThisRobotsAutons`,
  the robot-two strings → `this_robot.hpp`'s words (§0.2's lines). Drive's boot gains ONE line: `robot one: 31 facts, 31
  typed` (the ledger's counts — provenance, not a refusal).
- `src/common/driver_panel.hpp` (the row's first word from `this_robot.hpp`; `clawRowWorstLength` takes it; a
  static_assert on robot one's list as on robot two's :460), `src/common/boot_source.hpp` (:109's label from
  `this_robot.hpp`).
- `test/CMakeLists.txt` — `target_sources(… ../src/team/robot_one_autons.cpp)` beside :90.
- `tools/src_build_gate.py` — the five builds in every table (:409-470, :531-555; robot one's entry symbols as robot
  two's programs'); **the facts proof (D3 (a))**: (i) the five robot-one ROWS on the real tree: each either links with
  no needed fact owed or FAILS with exactly the `robot one: OWED -- …` lines of the owed facts its program needs
  (compared as a SET — `sort -u`; under `-j` a header's line repeats per file) and no other error (the gate computes
  the expected set from the ledger's text); (ii) **[R4] in a COPY, never in the repo:** the working tree's files
  (`git ls-files -co --exclude-standard`, so the uncommitted change is what is proven) copied to
  `build/src-gate-facts/` (gitignored); in the copy only, every `owed(p, …)` → `typed(p, p, …, "gate copy -- same as
  the placeholder, checked")` and the ledger's `gate copy` refusal turned off; all five built there and checked like
  the seven (the hash rule is proven on the real-tree rows; in the copy the gate's `git describe` runs with the copy's
  working directory, as `make`'s does, so both agree); the copy deleted in a `finally`, and a leftover one swept at the
  gate's start. The repo's `bin/`, `.d/` and ledger are never touched by (ii). **New step-0 rules**: a program naming
  `RobotTwoDriver` or `RobotOneDriver` (choosing a robot outside the chain — the per-robot table rule's sibling;
  `this_robot.hpp` exempt); a `typed(` in the real ledger whose source is empty or contains `gate copy`; self-test cases
  for each, for the robot-one namespace rule (a program naming `robot_one`; robot two's team file naming `robot_one`;
  **[R11]** the ledger naming `robot_one`), **[R4]** a proof interrupted by an exception after the copy is edited (the
  repo's ledger hash and `bin/` unchanged; the next run sweeps the copy), and **[R1]** the stamp (two different variants
  built in a row with NO `rm` between: the second recompiles every file, and for a robot-one variant with an owed fact
  it refuses and leaves no `bin/hot.package.bin`).
- Tests changed: `build_selection_test.cpp` (:35-36 `kEveryRobot` gains RobotOne; the beacon cases), `chassis_table_test`,
  `mechanism_table_test`, `boot_source_test` (robot one's list through `decideBootSourceFrom`), `driver_panel*_test`,
  `drive_program_source_test.cpp` / `competition_source_test.cpp` / **[R6] `auton_tester_source_test.cpp`** — the
  robot-two test text G1-A changes, ALL listed in §0.3, each re-pinned with the same force: `RobotTwoDriver` →
  `ThisRobotDriver`; `count(code, "registerAutons(") == 1` → `count(code, "registerThisRobotsAutons(") == 1` (and
  `count(code, "registerAutons(") == 0`, so a second, direct call cannot slip in); :56 / :112's
  "DropEntry); registerAutons(*st.registry);" sequence → the same sequence with the new name; :120's position check →
  the new name's position; **[R7]** :381's `"CLAW ABSENT"` → the words' constant if the LCD word moves. Robot two's
  other tests (`boot_source_test`'s claw strings, `boot_check_test`'s label) stay byte for byte — the words are a
  defaulted parameter.
- Docs (§3).

**NOT changed by G1-A** (T6 checks): `src/team/robot_two_driver.hpp`, `src/team/autons.{hpp,cpp}`, robot two's regions
of both table files, `include/shulib/` (no library change), `src/programs/bench_r3a.{hpp,cpp}` (its "robot two: ports 4
and 5" hints stay — true facts about robot two; landmine 11), `src/main.cpp` (it switches on `kProgram` only).

### 1.2 G1-B — the record names its robot (D10; after G1-A and CAL1-B)
`include/shulib/calibration/calibrate_program.hpp` (the provenance: `"shulib Calibrate <stamp> robot=<name>"` from
`CalibrateFacts::robot`, bounded to the 96-byte field), `src/robots/calibrate_facts.hpp` (a per-robot short name),
`src/common/boot_source.hpp` (the check before step 1's port check; its refusal words), CAL1-B's goldens
(`test/goldens/calibrate/*`), `test/boot_source_test.cpp`, `test/calibrate_*_test.cpp`; guide 19's "one card per robot"
sentence; the changelog. ~½–¾ builder-day.

### 1.3 G1-T — the twin (D9; only when your builder says robot two is rebuilt)
Robot two's region in `mechanism_table.hpp` replaced by robot one's copied (`robot_one` → `robot_two`), reading a new
`src/robots/robot_two_facts.hpp`; `src/team/robot_two_driver.hpp` re-made from robot one's (the students' choice which);
robot two's chassis table re-checked; the twin test armed (T7). Robot two's builds' packages then change, by design.
~¾–1 builder-day.

### 1.4 G1-F — typing facts (the coordinator's edit, as each batch arrives)
Only `src/robots/robot_one_facts.hpp` (and, for the flipper, D6's chosen block pasted into robot one's region and its
control into the driver file), the worksheet's filled form, then the gate and the suite. ~1–2 h each. **[R8]** That
holds because the declaration spells six preset slots and the driver file names levels THROUGH the ledger (P3) — a
renamed level is a ledger edit. **Not** ledger-only: a cascade with a third or fourth motor (`.motors` and `wire`), a
seventh level (refused — `kMaxPresets`), the flipper's block. **[R2]** The measured cap and stall signature, typed over
the starting ones after station 13, are a G1-F edit like any other.

### 1.5 G1-C — [R3] station 13 writes the cap (D13 (a); after MECH-S-A merges; a second slot beside G1-A)
`src/programs/bench_r3a.hpp` / `.cpp` only: when the station first powers a mechanism whose spec states `.currentCap`,
it reads each of its motors' limits (the parts' `Devices::currentLimits`, MECH-S-A's), writes the cap, reads it back
and shows "cap X.XX A per motor — set" (or "this provider cannot cap a motor — NOT CAPPED" in the page's warning
colour); on every exit path (the station's own exit and bench_r3a.cpp's guard, a thrown precondition included) it
writes back what it read. Tests in `test/bench_mech_station_test.cpp` over the fake provider: a capped mechanism's
motors hold the cap while powered and their old limit after exit (each exit path); robot two's claws (no cap) never
have a limit written; mutations: the restore skipped on the guard's path; the cap written to one motor only. Robot
two's three Bench-Tests packages change by design (listed). Docs: guide 21's station-13 paragraph (G1-A's) and the
changelog. ~½ builder-day (+~¼).

---

## 2. Step 0, the runs, their tests and mutations, and the definition of done

### 2.0 STEP 0 at the robot — THE FACTS FORM (one page; `docs/robot-facts-form.md`, the builder fills it in)
*Write "don't know" rather than guess. Each answer: who, how (read off / counted / tape / station 13 RECORD), date.
"The robot's centre" = midway between the left and right wheels, midway along the wheels.*

**A. What the robot is** (before any program — needed for Calibrate and the boot)
1. Is the drive a TANK — wheels on two sides, it cannot drive sideways? *(If NO: stop and tell the coordinator — every
   program G1 builds supports only a tank. HA-252.)*
2. How many drive motors per side; their cartridge colour (read the insert on ONE motor: red / green / blue).
3. Wheel size (in, across the tread); motor-to-wheel gearing (tooth counts, or "direct").
4. **Is there an INERTIAL SENSOR (IMU)? Which port?** *(The library cannot drive a tank without one. HA-253.)*
5. Which end is the front (where is the AI Vision camera)?

**B. What is plugged where — read off the brain's own Devices screen (VEXos; no program needed)**
6. Every port 1–21: the device (motor / rotation / vision / IMU / radio / empty) and what it is for.
7. Every 3-wire port A–H: what is plugged in (a solenoid? nothing?).

**C. The cascade** — *[R2] C7 needs Bench Tests' station 13 on robot one, and C6's MEASURED pair and C8 come from the
bench too, so they come AFTER the coordinator has typed A, B, C1–C5 and C6's STARTING pair*
8. *(C1)* Its motor ports; their cartridge colour; do two motors face each other (mirrored)? *(Typed as you see them;
   station 13's direction check then confirms each one's way — HA-255 — and a correction is a G1-F edit.)*
9. *(C2)* Gearing from motor to spool: count the teeth (motor gear, spool gear).
10. *(C3)* A rotation sensor on the cascade? Its port; on which shaft.
11. *(C4)* Where the cascade's HEIGHT is measured from (pick one point on the top stage — say which) to the floor.
12. *(C5)* Its levels, by stack height: a NAME (≤ 15 letters) and the height for each, at most six.
13. *(C6)* **[R2] Two steps.** (i) **STARTING** current cap and stall signature, typed BEFORE Bench Tests is built:
    **chosen, not measured**, with the coordinator, from chapter 18's order rule (hold < signature < cap − 0.2 A; the
    cap ≤ the string's safe pull) — written "chosen, not measured" as their source. (ii) **MEASURED** later from station
    13's RECORD while the cascade presses gently down on its bottom stop at 3 V, and typed over the starting pair (G1-F).
14. *(C7 — station 13)* With the cascade on its bottom stop at power-up: RECORD. Then jog up and stop at each level and
    at the top stop: tape the height (11.'s point) and RECORD. **Read the RECORD's `output … deg`, not its inches** —
    until the shape is typed, station 13's inches come from placeholders (landmine 6). Three to eight rows.
    **[R3]** Under D13 (a) station 13 writes the starting cap while it jogs (check the page says "cap … — set"); under
    D13 (b) it is UNCAPPED: keep the ceiling at 3 V on the cascade and do not jog into the top stop.
15. *(C8)* The smallest volts that hold the empty cascade halfway without sagging (TUNE1's page if it reaches robot one;
    otherwise chosen with the coordinator and typed as "chosen, not measured").

**D. The flipper**
16. Kind: motor arm / pneumatic / passive bar (AF1-S's D7).
17. Motor arm: port, cartridge, gear teeth; at station 13, RECORD at stowed and at full flip (angles). Pneumatic: the
    3-wire letter; one cable or two from the valve; which way is "out" (station 13's fire); how long a full stroke takes.
    Passive: nothing.

**E. Sensors**
18. The AI Vision camera: port; how far forward and left of the centre, how high, tilted how much.
19. Each rotation sensor not on the cascade: where, and what it measures (tracking wheels are NOT used by G1 — §5).

**F. The twin**
20. Is robot two being rebuilt like robot one? When? *(Then this form again for robot two.)*

*[R2] [R9] Stage A (all five programs) = 2, 4 (the chassis table's own UNSET rule, not the ledger), **6's motors that are
not drive motors** (each must be DECLARED — a flipper motor's block included), 8, 9, 13 (i) — the STARTING pair —,
16 (the kind; "not on the robot yet" is an answer). Stage B (Drive, Auton Tester, Competition) = 11, 12, 14, 15, the
rest of 17. 13 (ii) re-types facts already typed — it gates nothing, and should land before the skills runs. Recorded,
gating nothing = 18 (the camera — VIS1-B reads it), 19.*

### 2.1 The runs
| Run | What | Waits on | Size (builder-days) | Public docs |
|---|---|---|---|---|
| **G1-A — robot one in the code** | §1.1 | CAL1-B merged (the same files: `build_selection.hpp`, `Makefile`, `mechanism_table.hpp`, `boot_source.hpp`, the gate); MECH-S-A merged (`.currentCap`); your go on D1–D13 | **~3¾–4½** (+~1 verification) [re-check: was ~3–3¾] | guide 21 (new), the facts form (new), guide 07, changelog, `src/team/README.md`, **[R1] guide 16 :56-59** (the `rm` sentence: `make` now starts fresh by itself; the `rm` stays harmless) — **six**, one over rule 7's ~5, flagged |
| **G1-C — station 13 writes the cap** [R3] | §1.5 | MECH-S-A merged; your go on D13 (a); a second slot | **~½** (+~¼) | changelog (guide 21's paragraph is G1-A's) |
| **G1-B — the record names its robot** | §1.2 | G1-A merged | **~½–¾** (+~½) | guide 19, changelog |
| **G1-T — the twin** | §1.3 | the builder: robot two rebuilt | **~¾–1** (+~½) | guide 21, changelog |
| **G1-F — facts** (the coordinator) | §1.4 | each batch of the form | ~1–2 h each | the worksheet |

**Sized against** AF1-S's first estimate for G1 (~1½–2½, `briefs/AF1S-…` :512 — "a `RobotId`, its tables, the Makefile's
branch, the gate's builds, a driver file and an autons file, their tests"), plus what it did not count: the ledger and
its per-program refusals (~½), the gate's facts proof and new step-0 rules with self-test cases (~¾), the per-robot
words and the alias through four programs and their source-text pins (~½), D6's three proven flipper forms (~¼).
**Re-check additions:** the variant stamp and its self-test case (~¼, [R1]); the boot's step-5 words and the LCD word
per robot with the defaulted words parameter and T10's robot-one cases (~¼, [R7]); the facts proof in a copy with its
sweep and interrupted-proof case instead of in place (~⅛ over the in-place form, [R4]); the separate placeholder, the
`same()` checks and the non-drive-motor check (~⅛, [R9] [R10]); T6's constant-data compare (~⅛, [R5]) — ~¾ in all.
None of these numbers is measured.

### 2.2 Tests (ADVERSARIAL — each tries to break ONE rule) and the mutations each must turn RED
Every mutation is RUN and seen RED, then restored; logged in `ROB1-PROGRESS.md` with the failing test's name.

| # | Test (the rule it tries to break) | Mutations it must turn RED |
|---|---|---|
| T1 | **Every robot is every selection's case**: for each `RobotId`, `chassisTableFor`, `mechanismTableFor`, `robotBeacon`, the autons dispatcher and the driver selection return THAT robot's object (by ADDRESS, never a count); the four beacons differ and none is a prefix of another; robot one's variant beacons are not prefixes of robot two's (`one` vs `tank…`) | GM1 `robotBeacon(RobotOne)` returns robot two's; GM2 `mechanismTableFor(RobotOne)` returns `kNoMechanisms`; GM3 the dispatcher's RobotOne case calls robot two's `registerAutons` |
| T2 | **The ledger is complete and honest** (source + constexpr): every `Fact` in the ledger has exactly one `static_assert` whose message contains its `name` verbatim; every number in robot one's regions (both table files, the driver file's volts and times excepted — they are the team's, D7) comes from the ledger (a scan of the marked region: a digit outside a comment, a ledger name, `0_in`/`0` home, or **[R8] a preset slot's index `lv(0)`…`lv(5)`** fails); a `typed(` has a non-empty source; **[R10] a typed value equal to its placeholder is refused unless its source says so (P3b)**; `kRobotOneOwedCount` = the number of `owed(` | GM4 one fact's static_assert deleted; GM5 a literal `30_in` typed into the cascade's `.range`; GM6 `typed(…, "")`; GM7 a fact's message misspelt (no longer contains its name); **[R10] GM7b the `same()` refusal deleted** (a `typed(p, p, …)` compile-refusal case turns red); **[R4] GM7c the `gate copy` refusal deleted** (a compile-refusal case with that source) |
| T3 | **The stages are right** (constexpr over `needs`): Stage A needed by all five programs; Stage B by Drive, AutonTester, Competition only — and NOT by Bench Tests or Calibrate (they must run to MEASURE Stage B) | GM8 Stage B needed by Bench Tests; GM9 Stage A not needed by Calibrate |
| T4 | **The upload refuses by name** (the gate, real tree): each robot-one build links when its program needs no owed fact, else fails with exactly the owed facts' lines and no other error; the bench / xdrive / tank builds never print a `robot one: OWED` line | GM10 a Stage B fact's static_assert drops `needs(kProgram, …)` (Bench Tests fails — the gate's expected set differs); GM11 the condition written with `&&` for `||` (every build fails) |
| T5 | **Typed, every robot-one build works** (**[R4]** the gate's COPY under `build/src-gate-facts/`): all five link, their variant and robot beacons are in and every other robot's out, the program's entry symbols linked, the stdio rule held; **the repo untouched** — the real ledger's hash before = after, the repo's `bin/` holds no package carrying the `gate copy` source, no `build/src-gate-facts/` left after `check` | GM12 robot one's driver file missing the cascade's control (only the copy's build sees it — proves that build is real); **GM13 [R4] the proof pointed at the REPO instead of the copy** (the ledger-hash / `bin/` checks fail); **GM13b the `finally` deletion removed** (the leftover check fails) |
| T6 | **Robot two unchanged** — (a) `git diff 16c01e0 -- src/team/robot_two_driver.hpp src/team/autons.hpp src/team/autons.cpp` empty and robot two's two regions (between its markers, and :124-145) byte-identical, **[R5] and no line of `mechanism_table.hpp` above the end of robot two's region added or removed** (its line numbers are compiled in — P6); (b) the seven builds from `git archive` of the base and of the change (P0's method, same scratch path): **bench and xdrive** `hot.package.bin` identical except bytes inside `__DATE__ __TIME__` strings and `_PROS_COMPILE_TIMESTAMP_INT`; **the five tank builds** the same, OR — if a string's address moved — disassembly identical modulo addresses AND the sorted `strings -a` set identical **AND [R5] `.rodata` and `.data` of `hot.package.elf` identical byte for byte with every aligned word pointing into the image masked (P6's `t6_rodata.py` method; a section whose SIZE differs is NOT provable by it — every such build is listed and explained by hand, never counted as passed)**; every other difference is a finding to remove or explain in the log; (c) every robot-two host test unchanged except the §0.3 pins ([R6]: the `RobotTwoDriver` and `registerAutons(` pins, and :381 if the LCD word moves) | GM14 robot two's row word `CLAW` → `MECH` (the strings set differs); GM15 `kDeclaredMotorLabel` for robot two re-worded; **[R5] GM15b one blank line added above robot two's region** (the constant-data compare — not the disassembly, not the strings — turns red) |
| T7 | **The two robots differ only where stated** — before the twin: robot one's region names nothing of robot two's (no `robot_two`, no robot-two ledger), robot one's packages carry no robot-two beacon, driver path or `registerAutons` symbol and robot two's none of robot one's (gate step 8's `nm`); **from G1-T: the twin test** — the two regions' token streams equal after `robot_one`→`robot_two` and the ledger's name normalised | GM16 robot one's cascade wired to `robot_two::` anything; (G1-T) GM17 robot two's `"L3"` renamed |
| T8 | **The cascade on the host test plant** (placeholders; `sim::ArmPlant` with constants from the declaration, RPV1's §1.2 pattern): `r.home` drives DOWN under the cap and stops on the stall signature at 0 in; each level reached within the service's tolerance and held; a target past the top is a compile refusal (a `test/compile_refusals/` case); the measured shape has ≥ 3 rows; the cap is written into BOTH motors | GM18 the homing push's sign flipped (build refusal, by name — P2); GM19 `wire(cascade, motors(a))` with one port for a two-motor declaration (the wiring's refusal) |
| T9 | **Calibrate never powers a mechanism**: `calibrateFactsFor(kRobotOneChassis, robot one's list)` declares EVERY cascade motor port (and, per D6 form, the flipper's), named by its mechanism (CAL1-B's helper); **[R9] the non-drive-motor check**: a ledger listing a motor port robot one's list does not declare is a compile refusal, by name (P4's form, a `test/compile_refusals/` case) | GM20 one cascade port dropped from the wiring's declared ports; **[R9] GM20b the non-drive-motor check deleted** (its compile-refusal case turns red) |
| T10 | **The boot refuses a record that drives a mechanism port**, in robot one's words (`decideBootSourceFrom` with robot one's list and `wordsFor(RobotId::RobotOne)`; the label is robot one's, never "claw pivots"); **[R7] the step-5 refusal's line and headline say "mechanism", never "claw" / "CLAW", for a cascade port the record lists as a drive port (record and typed table); the LCD's absent word is robot one's**; robot two's calls (the default words) byte for byte as today | GM21 robot one's label left as robot two's; **[R7] GM21b robot one's headline arm returning robot two's literal** |
| T11 | **The default driver file** (host, through the PROS shim, robot two's `robot_two_driver_test` pattern): every button of the banner does what it says on the host; Y is free; L1+L2 together = neither; the row fits the SMALL budget for each D6 form | GM22 a level's button swapped in the file but not the banner (the banner-vs-file case) |
| T12 | **The starter autons**: `registerRobotOneAutons` registers the 40 in as SKILLS (schedule inside 60 s) and the acceptance run; each runs on the host test plant through the tester's run (`autons_test.cpp`'s chain) | GM23 the 40 in's kind Match (the skills-kind check) |
| T13 | **The flipper's three forms** (`robot_one_flipper_forms_test.cpp`): robot one's list + each form compiles, binds over the fake provider, its default control works, a blind flip (`r.move(flipper["flip"], …)` / `r.set(…)`) runs on the host test plant | GM24 the pneumatic form's `.moveTime` removed (the build refuses it by name) |
| T14 | **The Makefile**: `make ROBOT=one PROGRAM=x` for each x puts exactly its defines on every compile line (gate step 5); `ROBOT=bench PROGRAM=drive` and `ROBOT=xdrive PROGRAM=competition` are still `$(error)`s | GM25 `ROBOT=one` setting the tank define |
| T15 | (G1-B) **The record names its robot**: a record naming robot two on robot one → refused by name before any port check; its own → accepted; a nameless record → robot two accepts with a Warn, robot one refuses | GM26 the name check skipped for nameless records on robot one |
| T16 | **[R1] The variant stamp** (the gate's self-test, `make` run directly, NOT through `run_build`'s delete): `ROBOT=tank` then `ROBOT=tank PROGRAM=drive` with no `rm` → the second recompiles every file and its package carries `tank-drive`; the same variant twice → 0 files; a robot-one variant with an owed fact after a linked robot-two build → refused, and NO `bin/hot.package.bin` remains | GM27 the stamp's comparison always "equal" (the second build compiles 0 files); GM28 the stamp written but `bin/` not deleted (the refused build leaves robot two's package) |
| T17 | (G1-C) **[R3] Station 13 caps what states a cap**: §1.5's tests | §1.5's two mutations |

### 2.3 Definition of done — every run
- [ ] `ROB1-PROGRESS.md` created FIRST, appended as the run goes; nothing committed.
- [ ] The host suite green through `python3 tools/suite_run.py green`; every listed mutation RUN and seen RED, then
      restored; the compile-refusal harness green.
- [ ] `python3 tools/src_build_gate.py check` (all TWELVE builds; the facts proof) and `self-test` (new cases red→green).
- [ ] The ARM header gate clean; the CI PROS-free and sim-layering greps empty.
- [ ] T6's base-vs-change package comparison logged with its numbers — the bytes, the disassembly, the strings AND
      the masked `.rodata` / `.data` compare ([R5]).
- [ ] The doc gates (`api_doc_tool.py` check-fresh / coverage / examples / removability; `doc_staleness_audit.py`;
      `briefing_status.py check` against a FRESH build).
- [ ] §3's documents IN THE SAME CHANGE; the prose review logged.
- [ ] Every status line under-claims: "host- and build-gated; robot one does not exist in our records; nothing has run
      on it".

---

## 3. Documents — in the same change as the code (pace rule 3), the prose review logged (rule 4), ≤ ~5 public each (rule 7)

**G1-A (public):**
1. **NEW `docs/guide/21-adding-your-robot-by-hand.md`** — "Adding your robot by hand": the checklist of §0.3 in the
   students' words; the facts ledger (what OWED means, how a fact is typed, what the build says, why it refuses);
   the five programs and their slots; `make ROBOT=one …`; the order on the robot (§2.0's day one); robot one's default
   controls (a table) and that they are the team's; the flipper's three forms (paste-ready, from T13); "the two robots
   differ only in their facts". Chapter **21**, because the unmerged G3a-P1 holds chapter 20 ("Your robot file",
   `g3a-p1`'s `docs/guide/20-your-robot-file.md`); `mkdocs.yml` nav.
2. **NEW `docs/robot-facts-form.md`** — §2.0, printable, one page; linked from guide 21 and 07.
3. `docs/guide/07-getting-set-up.md` (:144-145 and the build list) — robot one's builds beside robot two's.
4. `docs/changelog.md` — no API change (the library is untouched); the new builds.
5. `src/team/README.md` — robot one's two files; which robot each file is.
Guides 16 (:52) and 17 (:27) show `ROBOT=tank` commands — still TRUE for robot two; a one-line "robot one: `ROBOT=one`"
each only if the prose review finds a sentence that is now FALSE ("only robot two…"), else left.
6. **[R1]** `docs/guide/16-testing-an-auton-on-the-bench.md` :56-59 — "The `rm -rf bin .d` is not optional … it does
   not notice that the `PROGRAM=` word changed" becomes FALSE with the stamp: re-worded to "`make` now notices a
   changed `ROBOT=` or `PROGRAM=` and starts fresh by itself; the `rm` is harmless". (The `rm` in the commands stays.)
   That is six public files, one over rule 7's ~5 — flagged for the team lead, not hidden.

**Internal, every run:** the worksheet (`docs/internal/chunks/R3a-BENCH-WORKSHEET.md`, a new station "Robot one — the
facts form", the form's answers as they come), `REMAINING-WORK.md`, `roadmap.md` ("you are here"), `build-order.md`,
`docs/hardware-assumptions.md` (HA-252..257), the briefing (regenerated, never by hand).

**The prose review's greps** (each run logs the hits it read): `grep -rn -i "only robot two\|robot two's autons are\|ROBOT=tank" docs/guide docs/*.md README.md src/`;
`grep -rn -i "simulator" docs/guide/21-* docs/robot-facts-form.md`; `grep -rn "G1-PROGRESS" docs/internal/chunks/ROB1-*` (empty —
the collision).

---

## 4. Hardware Assumptions — HA-252..HA-257 ONLY
**The next free number:** the register's Held line ends at HA-227 (`docs/hardware-assumptions.md` :39 — MECH-S-A's
staged copy registers HA-219..223); held in briefs and owed to the plan commit: AF1's HA-228..236, ALLY1+L4's
HA-237..242, AF1-S's HA-243..245, CURVE1's HA-246..250 (`briefs/CURVE1-…` :39-42); RPV1 "none expected", its first
contingent number HA-251 (`briefs/RPV1-…` :80, :771-772 — [R11]; :692-702, cited before, is a mutation table). **So G1 holds HA-252..HA-257** (labels R1-H1..H6 kept; a
mechanical rename if another brief reserves first). A seventh belief ⇒ stop and tell the coordinator.

| # | Belief | Code that acts on it | How it is checked |
|---|---|---|---|
| **HA-252** (R1-H1) | Robot one is a TANK (two sides, no strafe) — the builder's "identical" | robot one's Calibrate (`calibrate_facts.hpp`: kind Tank), the boot (a tank description) | Step 0 A1 |
| **HA-253** (R1-H2) | Robot one carries an IMU (the builder named only AI Vision and rotation sensors) | the boot refuses a tank description without one (`drivetrain_description.hpp` :183-189) | Step 0 A4; Bench Tests' census |
| **HA-254** (R1-H3) | Between two measured rows, the cascade's height is close to straight in its output angle (string layering on the spool, stretch) | the `measured` shape's interpolation (D5) | Step 0 C7 rows at every level; a mid-row check at the bench |
| **HA-255** (R1-H4) | The cascade's motors face each other (`{Forward, Reversed}`) until station 13's direction check | robot one's declaration | station 13's direction check |
| **HA-256** (R1-H5) | The twin is structurally identical (same kinds, motor counts, sensors), differing only in ports and measured numbers | D2's twin test; G1-T's copy | the builder's form for the twin (F20) |
| **HA-257** (R1-H6) — only if D10 (b) | Each robot keeps its own SD card; a card never moves between them | the boot (no robot check) | a label on each card; withdrawn by G1-B |

---

## 5. OUT, and explicitly rejected
- **The panel** (G3a), **`tools/robot_tool.py`** and its `.robot.json` (branch `g3a-p1`, unmerged) — robot one's
  regions carry `[robot one mechanisms]` markers so the generator can take them over after the 21st.
- **ALLY1** (the alliance colour) and L4 — after the 21st.
- **A skills route, or any auton beyond D8's starter** — the students' (G4); the AF1-S Toggle steps are AF1S-A1/A2.
- **The camera's wiring into a booted robot** — VIS1-B (D12).
- **Tracking wheels from robot one's rotation sensors** — the boot builds drive-encoder odometry only
  (`boot_source.hpp` ~:126-127); a later chunk if the builder has them.
- **A wall reset on robot one** — it has no distance sensors (the builder's list); `competition::resetFromWall` refuses
  "no sensor" there, as on robot two before L6-B.
- **More than six named levels** (AF1-S's G8, `kMaxPresets`) — constants `cascade.to(h)` serve autons.
- **The string-snap watch** (MECH-S-B, after the 21st).
- **Renaming `ROBOT=tank`** (D1 (b)) — after the 21st, if at all.
- **Rejected:** a `consteval` ledger check (P1 — it fails every build and names nothing); a Makefile "rehearsal" switch
  that links placeholders into an uploadable package (D3 (a)'s gate copy does the same proof with nothing left behind).

---

## 6. Coordination — shared files, who waits for whom
- **CAL1-B** (done, merging after MECH-S-A — NOTES :965): edits `src/build_selection.hpp` (:90's message),
  `Makefile` (:126-134, :155-163, :178), `src/main.cpp`, `src/robots/mechanism_table.hpp` (its banner; deletes
  `dropDeclaredMechanismPorts`), `src/common/boot_source.hpp` (:106-108's comment), the gate (:36-44, :138-146, :548-555,
  :1893, :2595-2599); adds `src/robots/calibrate_facts.hpp`. **G1-A starts from CAL1-B's merge** and never re-adds
  what it deletes. G1-B edits CAL1-B's library Calibrate and its goldens — after CAL1-B.
- **MECH-S-A** (staged on main, merging first): robot one's cascade uses `.currentCap`; G1-A starts after it.
- **STEP1-A** (done, merging third): its `src/` files are `auton_tester_panel.hpp`, `auton_tester_run.hpp`,
  `competition_run.hpp`, `competition_screen.hpp` — G1-A edits `competition_program.cpp` and
  `auton_tester_program.cpp` beside them; whichever merges second re-reads the other. STEP1-B: later, its own brief.
- **TUNE1-A** (building; no `src/` change): live tuning reaches only robots with a build — robot one after G1-A.
  TUNE1-B1 (queued) puts its page into a program: if it lands after G1-A, it serves both robots by `this_robot.hpp`;
  if before, G1-A carries it to robot one.
- **VIS1-A2** (in flight, library only): no shared file. **VIS1-B** (not built): designs the camera path for robot two;
  under D12 (a) it wires robot one's block too, from G1's ledger.
- **AF1-S**: AF1S-A1 (the checked flip, library) shares no file. AF1S-A2 (the camera in autons) needs robot one's
  camera (VIS1-B) and flipper (G1-F after D6's answer). AF1-S's Step 0.1 and this form's D share their questions — one
  visit to the builder.
- **L6-B** (queued): robot two's side sensors in `chassis_table.hpp` (robot two's table) and `boot_source.{hpp,cpp}`
  — the same files as G1-A: never two builders at once; the second re-reads the tip. Robot one gets no wall block.
- **RPV1**: RPV1-A previews ROBOT TWO; **RPV1-B previews robot one "the day its file exists"** (`briefs/RPV1-…` :78-92,
  :385-387): G1-A gives it `RobotId::RobotOne`, `robot_one::kRobotOneMechanisms`, `registerRobotOneAutons` in a
  PROS-free file the host compiles, and the ledger's placeholders — which RPV1-B must hatch as PLACEHOLDER (its D5's
  words fit: "this time is what its declaration predicts").
- **CURVE1**: no shared file.
- **[R3] G1-C** edits `src/programs/bench_r3a.{hpp,cpp}` and `test/bench_mech_station_test.cpp` only — no file of
  G1-A's; it needs MECH-S-A merged (the parts' `currentLimits`). If TUNE1-B1 or another run edits Bench Tests in the
  same days, never two builders at once; the second re-reads the tip.
- **G3a-P1** (unmerged): changes `chassis_table.hpp`, `test/CMakeLists.txt`, the gate; it will re-read G1's tip when
  it merges after the 21st, and its chapter 20 stays 20.

---

## 7. Checks — the coordinator re-runs every one after each run (never the report's word)
```sh
cmake --build build/test -j"$(nproc)"
python3 tools/suite_run.py green
ctest --test-dir build/test -R compile_refusal --no-tests=error --output-on-failure | tail -3
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > <scratch>/all.cpp
echo "int main(){return 0;}" >> <scratch>/all.cpp
(ulimit -v 4194304; timeout 300 arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c <scratch>/all.cpp -o /dev/null -Iinclude)
grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib | grep -v '^include/shulib/hal/pros/'   # empty
grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib              # empty
python3 tools/src_build_gate.py check && python3 tools/src_build_gate.py self-test   # TWELVE builds + the facts proof
python3 tools/api_doc_tool.py self-test && python3 tools/api_doc_tool.py check-coverage && python3 tools/api_doc_tool.py check-fresh \
  && python3 tools/api_doc_tool.py check-examples && python3 tools/api_doc_tool.py check-removability
python3 tools/doc_staleness_audit.py && python3 tools/briefing_status.py check
# ── the brief's own ──
git diff --stat 16c01e0 -- include/shulib                                          # G1-A: empty (no library change)
git diff 16c01e0 -- src/team/robot_two_driver.hpp src/team/autons.hpp src/team/autons.cpp   # G1-A: empty
git show 16c01e0:src/robots/mechanism_table.hpp | sed -n '/\[robot two mechanisms\]/,/\[\/robot two mechanisms\]/p' > <scratch>/r2a
sed -n '/\[robot two mechanisms\]/,/\[\/robot two mechanisms\]/p' src/robots/mechanism_table.hpp | diff <scratch>/r2a -   # empty
grep -n 'consteval' src/robots/robot_one_facts.hpp                                 # empty (P1)
grep -c 'static_assert' src/robots/robot_one_facts.hpp; grep -c ' owed(\| typed(' src/robots/robot_one_facts.hpp   # equal (+ the fixed ones the banner names)
grep -rn 'RobotTwoDriver\|registerAutons(' src/programs src/common | grep -v this_robot.hpp   # empty
grep -rn 'robot_one' src --include=*.hpp --include=*.cpp | grep -v 'src/robots/\|src/team/robot_one_\|src/common/this_robot.hpp'   # empty
(ulimit -v 4194304; timeout 300 make ROBOT=one PROGRAM=drive -j"$(nproc)") 2>&1 | grep -o 'robot one: OWED -- [^"(]*' | sort -u | wc -l   # [R11] = the owed Stage A+B facts (one line per fact; per-file repeats removed)
ls bin/hot.package.bin   # [R1] after that refused build: "No such file" (the stamp deleted the last variant's package)
ls docs/internal/chunks/G1-PROGRESS.md && git diff --stat -- docs/internal/chunks/G1-PROGRESS.md   # no change (the collision)
# T6 — base vs change, robot two's seven packages (P0's method; <scratch> is the coordinator's scratch, never /tmp)
for rev in 16c01e0 HEAD-OR-THE-WORKTREE; do rm -rf <scratch>/t6 && mkdir <scratch>/t6 && (git archive $rev | tar -x -C <scratch>/t6) \
  && for v in "ROBOT=bench" "ROBOT=xdrive" "ROBOT=tank" "ROBOT=tank PROGRAM=drive" "ROBOT=tank PROGRAM=calibrate" \
     "ROBOT=tank PROGRAM=autontest" "ROBOT=tank PROGRAM=competition"; do (cd <scratch>/t6 && rm -rf bin .d \
     && (ulimit -v 4194304; timeout 300 make $v -j"$(nproc)") >/dev/null && cp bin/hot.package.bin <scratch>/$rev-${v// /_}.bin); done; done
# then per build: cmp -l <base> <change> — every differing byte inside a "__DATE__ __TIME__" string or
# _PROS_COMPILE_TIMESTAMP_INT (P0: 7 bytes); else objdump -d --no-show-raw-insn modulo addresses + sorted strings -a
# [R5] AND (keep bin/hot.package.elf beside each .bin) the masked constant-data compare, P6's script:
#   (ulimit -v 4194304; timeout 300 python3 -I .claude/coordinator/scratch/brief-G1/chk2/t6_rodata.py <base>.elf <change>.elf)
#   -> ".rodata … differing after masking: N" — every N byte inside the timestamp string, else a finding;
#      "SIZE DIFFERS" -> not provable by this compare: list and explain by hand
# [R1] the stamp: after `make ROBOT=tank`, `make ROBOT=tank PROGRAM=drive` (no rm) prints the stamp's line and compiles every file
```
(For G1-A the base is `16c01e0` + MECH-S-A + CAL1-B + STEP1-A as merged — the tip G1-A started from — not `16c01e0`
itself.) **Spot checks:** G1-A — `make ROBOT=one PROGRAM=competition` fails naming a Stage B fact; flip that one fact
in a scratch copy and see the next named; re-run **GM10** and **GM12** by hand. G1-B — re-run **GM26**. G1-T — the twin
test red on GM17.

---

## 8. Landmines
1. **The name G1 is taken** (`G1-PROGRESS.md`, `G1-COMPLETED.md` — the 2026-09 chunk). File as `ROB1-*` (D1 (d)).
2. **`consteval` + `||` fails EVERY build** (P1): an immediate invocation is evaluated in the short-circuited operand.
   The ledger's checks are plain `static_assert`s (P1b).
3. **GCC names a refusal function, not a string argument** (P1): the fact's name must be IN the static_assert's
   message. T2 pins name = message.
4. **Every robot-one mechanism MOTOR must be declared before Calibrate runs on robot one** — an undeclared motor is a
   drive candidate, POWERED if it turns with the push (`calibrate_program.hpp` :13-16 (CAL1-B)). **[R9]** A port in
   the ledger does not declare it — Calibrate reads only the list; the Stage A "non-drive motor ports" fact and its
   build check (P4) are what refuse an undeclared one (D6).
5. **Bench Tests must build on Stage A alone** — it is the tool that MEASURES Stage B. A Stage B check on Bench Tests
   (GM8) locks the team out of measuring. **[R2]** Nothing in Stage A may need station 13 to be measured: the cap and
   signature enter Stage A as a STARTING pair, chosen, not measured.
6. **Station 13's inches are the placeholder shape's until Stage B is typed** — Step 0 reads the RECORD's
   `output … deg` (`bench_r3a.hpp` :1430). Say it on the form and in guide 21; never "correct" a placeholder from an
   inches reading.
7. **Placeholders must pass the library's own rules** (stall < cap − 0.2 A, rows increasing, levels inside the range,
   a homing push DOWN — P2). A placeholder set that fails turns every HOST build red, not just robot one's.
8. **A tank needs an IMU** (`drivetrain_description.hpp` :183). If robot one has none, nothing of the library drives it
   — stop and tell the coordinator; no workaround in G1.
9. **Two robots, one record file name, a port-only boot check** (D10): until G1-B, a card moved between near-identical
   robots boots silently.
10. **The bench and X-drive builds compile every program's `.cpp`** (PROS builds all of `src/`) — `ThisRobotDriver` must
    have a type for EVERY `RobotId` (robot two's for those two, exactly today's), or the bench build breaks.
11. **Robot-two words that are TRUE stay** — bench_r3a's "(robot two: ports 4 and 5, 2026-09-24)" hints (:2988, :3150)
    describe robot two; changing them moves robot two's tank packages (T6). Only words that would be FALSE on robot one
    (the driver file's path, "robot two's driver file", the claw row's word, the boot label, **[R7] step 5's "claw"
    refusals and headlines, the LCD's "CLAW ABSENT"**) go per robot. G1-A does not touch `bench_r3a`; **[R3] G1-C does**,
    on purpose, and lists the three packages that change.
12. **String literals for robot one in robot-two builds** — a robot-one string the compiler keeps in a shared section
    shifts robot two's addresses; T6's fallback (disassembly modulo addresses + the strings set **+ [R5] the masked
    constant-data compare**) is the honest test then — never loosen T6 to "it links".
13. **[R4] The gate's typed proof runs in a COPY, never in the repo** (it used to say "edits the real ledger in place";
    a kill or a power loss would have left every fact reading typed). The copy lives under `build/src-gate-facts/`, is
    deleted in a `finally` and swept at the next start; the real ledger's `gate copy` refusal makes a copied-back
    ledger fail at `make`; step 0 refuses that source in the repo too.
14. **The driver file's volts and times are the team's** (D7), not facts — the ledger scan (T2) excludes the driver
    file; the cascade's `.unhomedVolts` and homing push ARE ledger facts (they are the robot file's).
15. **RPV1-B and robot one's placeholders** — a preview drawn on OWED numbers must say so (RPV1's hatch); G1 exposes
    `kRobotOneOwedCount` so it can.
16. **The twin test is meaningless before the twin** — do not arm it against today's robot two (a different robot);
    T7's before-twin half checks separation instead.
17. **Robot one's facts are typed by the coordinator from the builder's form** — never "corrected" to match a belief,
    never invented to unblock a build; a starting value chosen at the bench is typed as "chosen, not measured".
18. **[R7] `-Wformat=2` is on** (`Makefile` :53): a per-robot `snprintf` format cannot come from a variable — pick
    between two literals with `?:` (GCC checks both arms), robot two's literal verbatim.
19. **[R5] Robot two's mechanisms carry their LINE NUMBERS** (`std::source_location` in every declaration and
    `wire(...)`): a line added or removed anywhere above the end of robot two's region in `mechanism_table.hpp` changes
    all five tank packages while the disassembly and strings stay the same (P6). Everything new goes below.
20. **[R1] `make` used to re-link the last variant's objects** (guide 16 :56-59): without the stamp, the facts refusal
    holds only from a clean tree. Never remove the stamp to "speed up" a build.

---

## 9. Summary (amended at the re-check — [R1]–[R11])

**What G1 is:** robot one in the code by hand — a robot id and five builds (`make ROBOT=one …`: Bench Tests, Calibrate,
Drive, Auton Tester, Competition), its drivetrain table (unknowns UNSET; Calibrate measures the rest), its cascade
declared from a FACTS LEDGER whose unknown numbers are marked OWED with placeholders, a default driver file, a starter
autons file — everything host-tested on the placeholders, and **every robot-one upload build refusing, one line per
owed fact by name, until the builder's facts are typed** (Stage A for every program; Stage B for the three that drive the
cascade), with `make` starting fresh whenever the robot or program changes so a refused build leaves no package
([R1]). Robot two is untouched — its packages compared by machine code, strings and constant data ([R5]). Beside it,
**G1-C** makes Bench Tests' station 13 apply the cascade's current cap while it jogs it ([R3]).

**Decisions as I now recommend them (plain words):**
- **D1** — call it robot one, `make ROBOT=one`; robot two keeps every name; the chunk's files are `ROB1-*` because "G1"
  is already a finished chunk. **(a)**
- **D2** — each robot's mechanisms written the way the guide teaches, every number from that robot's own facts file; a
  test that the two robots' blocks match once the twin exists. **(a)**
- **D3** — unknown numbers live in one facts file; building a robot-one program that needs an unknown one stops and
  names it. **Changed:** `make` now notices a robot or program switch and starts clean, so the stop cannot be skipped by
  an old package ([R1]); the cascade's current cap and stall signature start as a careful CHOSEN pair so Bench Tests
  can be built, then are re-typed once measured ([R2]); the gate's "everything typed" rehearsal runs in a throwaway
  copy, never on the real file ([R4]); a typed fact must carry its own value, never just keep the placeholder ([R10]).
  **(a)**
- **D4** — all five programs for robot one; the gate grows from 7 to 12 builds, about 1½ minutes longer ([R11]). **(a)**
- **D5** — the cascade as a lift with a measured row at every level, homed by stalling gently down under the cap; every
  number — including the level names — in the facts file ([R8]). **(a)**
- **D6** — no flipper written until the builder says which kind; all three kinds ready and tested. **Changed:** the
  thing that stops Calibrate from powering a mechanism motor is now "every motor that is not a drive motor must be
  declared", checked at build ([R9]). **(a)**
- **D7** — a default controller map, marked as the team's to change. **(a)**
- **D8** — a starter skills auton (the 40 in) plus the acceptance run; the route is the students'. **(a)**
- **D9** — robot two unchanged now; the twin is a small later run when the builder rebuilds it. **(a)**
- **D10** — the calibration record names its robot (G1-B). **Changed:** no longer the first thing to cut — it is the
  only check at power-up that catches robot two's program uploaded onto robot one ([R1]). **(a)**
- **D11** — G1-A in the next free build slot after CAL1-B merges, ahead of the queue; G1-C in a second slot beside it.
  **(a)**
- **D12** — the camera's wiring is VIS1-B's; robot one's camera port is written down but stops no build ([R9]). **(a)**
- **D13 (new)** — Bench Tests' mechanism page (station 13) applies a mechanism's current cap while it powers it, and
  puts the motors back as they were when it leaves — so the cascade's first powered moves, done there, are capped
  ([R3]). Robot two's claws state no cap and behave exactly as today. **(a)**

**Runs and sizes (builder-days, estimates — none measured; verification on top):** **G1-A ~3¾–4½ (+~1)** (was ~3–3¾;
the re-check's fixes add ~¾) · **G1-C ~½ (+~¼)**, in a second slot beside G1-A · G1-B ~½–¾ (+~½) · G1-T ~¾–1 (+~½),
only when robot two is rebuilt · G1-F ~1–2 h per batch of facts.

**Honest dates** (today Wed Nov 11; robot one built ~Sat Nov 14; freeze ~Wed Nov 18; competition Sat Nov 21; one
builder-day per slot per calendar day — the plan's arithmetic, never measured; usage stops have cut whole days twice
since Nov 3):
- **D11 (a), G1-A launched Nov 11** (after your go and CAL1-B's merge) **with G1-C beside it**: G1-C merges ~Nov 12–13;
  **G1-A merges ~Nov 15–16**. Robot one's Stage A facts (the Devices screen, the teeth, the flipper's kind, the
  starting cap and signature) typed the same day → **Bench Tests and Calibrate on robot one ~Nov 15–16**; the cascade
  measured at station 13 → Stage B typed → **Drive, Auton Tester, Competition on robot one ~Nov 16–17**; TUNE1 (if
  TUNE1-B1 has reached it) and the students' skills route **~Nov 17–18. That is one to two robot days before the
  freeze, at best** (it was two to three before the re-check grew G1-A).
- **D11 (b), the queue kept:** G1-A starts ~Nov 13 → merges ~Nov 17–18 → robot one's first Drive **at or after the
  freeze**.
- **G1-B** with a slot after G1-A: ~Nov 17; else the 21st runs on a labelled card per robot, said plainly as a risk.
- **G1-T**: ~1½ days after the builder says robot two is rebuilt; after ~Nov 16 it misses the freeze.
- **If cuts are needed** ([R1] re-ordered): first D6's three flipper forms down to the one the builder names (if he
  has); then robot one's Auton Tester build (D4 (b)); then G1-B, last, and only with labelled cards and the risk said.
  **Never cut** the facts refusal, the variant stamp, robot two's package check, the gate's five builds, or G1-C
  (under D13 (b) the cascade's first moves are uncapped).

**Probes:** P0 (the base-vs-change package check works: 7 timestamp bytes on a same-tree rebuild; an 8-s build), P1 /
P1c (the `consteval` ledger check fails every build and names nothing), P1b (a per-fact `static_assert` names the fact
on ARM robot one and passes elsewhere), P2 (robot one's cascade shape, three flipper forms and the default map compile
host and ARM; six bad facts refused by the library by name); **re-check:** P3 (every cascade number and the map's level
names from the ledger, six preset slots — host and ARM clean; Bench Tests names exactly its four Stage A facts, Drive
all nine), P3b (a typed value equal to its placeholder, and a `pretend`/copy source, refused by name), P4 (an
undeclared non-drive motor refused by name), P5 (the variant stamp: a program switch recompiles all 11 files where
`make` compiled 0), P6 (a one-line shift above robot two's region: invisible to disassembly and strings, 26 bytes in the
masked `.rodata` compare).

**Found while writing (each handled):** the G1 name collision (D1 (d)); the `consteval` trap (P1); a robot-two card
boots robot one silently (D10); a tank needs an IMU and the builder named none (HA-253); undeclared mechanism motors are
Calibrate drive candidates (landmine 4); Bench Tests must build on Stage A alone (landmine 5); station 13's inches come
from the placeholder shape (landmine 6); robot-two words in four programs (§0.2); `ThisRobotDriver` needs a type for
every robot (landmine 10). **Found by the re-check:** [R1]–[R11] below.

**THE FACTS THE BUILDER MUST SEND** (one list; the form in §2.0 asks them in this order):
1. Is robot one a tank (wheels on two sides, cannot strafe)? (HA-252)
2. Drive motors per side and their cartridge colour; wheel size; motor-to-wheel gearing.
3. Is there an inertial sensor (IMU), and on which port? (HA-253 — without one, nothing drives)
4. Which end is the front (where the AI Vision camera is).
5. The brain's Devices screen: every port 1–21 (what is plugged in, what it is for) and every 3-wire port A–H —
   **and which motors are NOT drive motors** (each must be declared before Calibrate runs).
6. The cascade: its motor ports, cartridge colour, whether two motors face each other; the gear teeth from motor to
   spool; a rotation sensor on it (port, shaft); the point on the top stage its height is measured from.
7. The cascade's levels: a name (≤ 15 letters) and the stack height for each, at most six.
8. With the coordinator: a STARTING current cap and stall signature (chosen, not measured) — then, at station 13, the
   measured pair.
9. At station 13: the RECORD's output angle and a taped height at the bottom stop, each level and the top stop (three to
   eight rows); the smallest volts that hold the cascade halfway.
10. The flipper: motor arm, pneumatic, passive bar — or "not on the robot yet"; then its port or 3-wire letter, gearing,
    angles or stroke time.
11. The AI Vision camera's port and where it sits (forward, left, height, tilt) — recorded for VIS1-B.
12. Any other rotation sensors: where, and what they measure.
13. Is robot two being rebuilt like robot one, and when? (D9, HA-256)
14. Do the students call the robots anything else? (D1)

**UNVERIFIED:**
- **robot one itself** — its kind (HA-252), its IMU (HA-253), every port, cartridge, tooth count, height and level: no
  record of the robot exists;
- the cascade's height-to-angle line between rows (HA-254), its motors' directions (HA-255);
- whether robot two will be rebuilt as the twin, and when (HA-256, D9);
- the flipper's kind (AF1-S's D7, still owed);
- CAL1-B's and STEP1-A's FINAL merged form (both done, not merged) — G1-A re-reads the tip it starts from;
- the sizes and dates above — estimates against other briefs' estimates;
- that robot two's tank packages stay unchanged (T6) — P0 and P6 show the checks work, not that G1-A will pass them;
- **[R1]** the stamp was probed on a `git archive` copy with no `.git` (no build hash); with the hash, every new commit
  recompiles everything — believed harmless, not run; whether the gate's output parser ignores the stamp's `$(info)`
  line — not run; whether `pros upload` or the VS Code PROS extension runs `make` itself — not checked;
- **[R3]** G1-C is not probed: that station 13 can write a cap through the parts' `currentLimits` as the service does,
  and whether a V5 motor keeps a written limit after the program exits (MECH-S-A's boot already reads limits other
  programs may have left, `service.hpp` :126-131) — unmeasured; a starting cap low enough to protect the strings may
  also be too low to lift the cascade — found at the bench;
- **[R4]** the copy-based proof is designed, not built: that `git ls-files -co --exclude-standard` captures every file
  a build reads;
- **[R5]** P6's masked compare treats any 4-byte word whose value falls in the image's address range as a pointer —
  a data constant that happens to fall in that range would be masked (a missed difference, never a false one);
- **[R7]** that GCC folds the defaulted words parameter so robot one's literals do not land in robot two's packages —
  T6 will show it;
- **nothing in G1 has run on a robot.**

---

## Re-check — amendments (2026-11-11; the checker's eleven findings, each re-verified)

| # | Finding (checker's severity) | Re-verified how | Disposition | Where fixed |
|---|---|---|---|---|
| **R1** | The refusal only works from a clean tree: `make` does not recompile when `ROBOT`/`PROGRAM` change; G1-B was the first cut (major) | **Real.** `common.mk` :26-30, :266 — objects depend on sources and `.d` files only; guide 16 :56-59 says so; the checker's `chk_t6/a` built 0 files on a program switch. **P5:** a 9-line stamp before `-include ./common.mk` → the switch recompiles all 11 files, the same variant 0. | **Fixed.** A variant stamp in G1-A (deletes `bin/` and `.d/` when robot, program or defines change); D3 states the clean-tree condition and what the stamp leaves (a same-variant failed rebuild keeps that variant's last good package); T16 + GM27/GM28; §7 checks `bin/` is empty after a refusal; guide 16's sentence re-worded (six public docs, flagged); G1-B moved to the LAST cut. | D3, plain words, §1 intro, §1.1 Makefile, §1.1 gate, §2.1, T16, §3 item 6, §7, landmine 20, §9 |
| **R2** | Stage A is circular: the cap and signature block Bench Tests but are chosen from Bench Tests' station 13; the C labels are wrong (major) | **Real.** D3 put the pair in Stage A; form item 13 chose it "from station 13's RECORD"; the italic note said "C8–C11" where the items run C1–C8. | **Fixed** with the checker's first option: a STARTING pair, "chosen, not measured", in Stage A; the measured pair re-typed later (G1-F). **Not** moved to Stage B: station 13 cuts out on the stall signature (`bench_r3a.hpp` :262-266) and, under D13, writes the cap, so a placeholder there would reach a program that uses it. Directions likewise typed as the builder sees them, confirmed at station 13. Labels corrected. | D3, §1.4, §2.0 (C note, C1, C6, the stage line), landmine 5 |
| **R3** | Station 13 never applies MECH-S-A's cap (major) | **Real.** `service.hpp` :111-137 (staged) writes the cap at bind; `mechanism_parts.hpp` :33-35 (staged) "writes no limit itself"; `bench_r3a.hpp` :237-240 no service; `grep -rn 'currentCap\|currentLimit\|set_current_limit' src/` empty at the tip and in `cal1-b`, `step1-a`, `mech-s-a`; robot two's claws state no cap. | **Fixed** as a new decision **D13** (a src change to Bench Tests is a scope change the team lead should see): recommended (a) run **G1-C** — station 13 writes a stated cap, reads it back, restores on every exit; (b) the procedure (uncapped, 3 V, no jogging into the top stop) stated as the alternative. D3's "station 13 acts on those two" corrected. Form C7 says which. | D3, D11, D13, plain words, §1 intro, §1.5, §2.0 C7, §2.1, T17, landmine 11, §9 |
| **R4** | The pretend proof edits the real ledger in place and can leave a linked placeholder package (major) | **Real.** `run_build` :1581-1592 deletes `bin/`/`.d/` before each build in `REPO`, never after; `cmd_check` :1834-1840 leaves the last build. A kill skips any restore. | **Fixed:** the typed proof runs in a COPY of the working tree under `build/src-gate-facts/` (gitignored), deleted in a `finally` and swept at start; the repo is never touched; the real ledger refuses a `gate copy` source at `make` (P3b), the copy alone turns that off; step 0 refuses it too. T5 now checks the repo is untouched; GM13 re-aimed, GM13b and GM7c added; an interrupted-proof self-test case. A `git archive` copy was rejected for this half: it would miss the uncommitted change being gated. | D3, §1.1 gate, T2, T5, landmine 13 |
| **R5** | T6 cannot see robot two's recorded line numbers (major) | **Real.** `kinds.hpp` :65-70 and `wiring.hpp` :623-647 take `std::source_location`; `grep` finds it nowhere else in `include/` or `src/`, and no `assert`/`__LINE__`. The checker's `chk_t6` pair: `cmp` 26 bytes. **P6:** a per-symbol compare caught only the timestamp (the list has no symbol), but a whole-section `.rodata` compare with image pointers masked shows the 26 bytes; base vs itself 0. | **Fixed:** every G1-A line in `mechanism_table.hpp` goes below robot two's region; T6 (a) checks no line above it moved; T6 (b) adds the masked constant-data compare (a size change = not provable, explained by hand); GM15b; "checked to the byte" dropped from the plain words. | D9, plain words, §0.4 P6, §1.1 table file, T6, §2.3, §7, landmines 12 and 19, §9 |
| **R6** | More robot-two tests turn red: the `registerAutons(` pins (minor) | **Real.** `auton_tester_source_test.cpp` :43, :56, :120; `competition_source_test.cpp` :54, :112 at `16c01e0`. The other `registerAutons` callers (`autons_test`, rigs, `competition_screen_test` …) call robot two's own function, which stays. | **Fixed:** listed in §0.3 and §1.1 with how each is re-pinned (count 1 of the new name AND count 0 of the old; the same sequence and position checks). | §0.3, §1.1 tests changed, T6 (c) |
| **R7** | Robot one's boot refusals would still say "claw" (minor) | **Real.** `boot_source.hpp` :482-500 (the line and both headlines); `driver_panel.hpp` :549 `kLcdClawAbsent`; pinned in `boot_source_test` :1102-1236 and `drive_program_source_test` :381. | **Fixed:** these words join `this_robot.hpp`'s per-robot words as `wordsFor(RobotId)`, passed as a LAST DEFAULTED parameter so robot two's calls and tests are untouched and T10 can test robot one's words on the host; `-Wformat=2` (`Makefile` :53) forces literal formats — `?:` between two literals. T10 extended; GM21b. | §0.3, §1.1 this_robot, T10, landmines 11 and 18 |
| **R8** | The levels cannot come from the ledger as one array; P2 hard-coded ports, teeth and levels (minor) | **Real.** The checker's `ledger_full.cpp` / `ledger_six.cpp`; `p2_robot_one_shape.cpp` :30-48. **P3/P3c:** every number (ports, directions, cartridge, teeth, rows, top, stall amps/rpm/time, cap, hold, homing push, unhomed volts, six slots) and the map's level names from the ledger — host and ARM `-Werror` clean. | **Fixed:** D5 and §1.1 spell six preset slots; T2's scan exempts the slot indices; G1-F says what is NOT ledger-only (motor count, a flipper block). | D5, §0.4, §1.1, §1.4, T2 |
| **R9** | The camera port gates builds nothing reads; the flipper port's gate does not protect Calibrate (minor) | **Real.** No G1-A code reads the camera port (D12). CAL1-B's `calibrateFactsFor` (`calibrate_facts.hpp` :60-80, worktree) reads only the mechanism list. **P4:** a "non-drive motor ports" fact checked against `declaredMotorPorts` refuses an undeclared port 10 by name. | **Fixed:** the camera is `Recorded` (gates nothing); Stage A gains "every motor that is not a drive motor" with P4's check, which covers the flipper whatever its kind; a typed kind `Arm` without an arm declared is refused; "not on the robot yet" is a valid kind. | D3, D6, D12, §1.1 ledger, §2.0, T9, landmine 4 |
| **R10** | Typing a fact can leave its placeholder in place (minor) | **Real** for the design as written (`Fact { value; typed; … }`). **P3b:** separate fields; `typed(1.0, 1.0, …)` and a struct fact typed equal to its placeholder are refused by name; the library's `HeightAt` has no `==`, so the ledger has its own `same()`. | **Fixed:** `Fact` keeps `placeholder` and `typedValue` apart; typing must state the value; equal-to-placeholder refused unless the source says "same as the placeholder, checked". T2 + GM7b. | plain words, §1.1 ledger, T2 |
| **R11** | Slips: the `grep -c` count; the gate's time; the RPV1 citation; the ledger's namespace vs the gate's rule (minor) | **Real**, each: `grep -c` counts per file and varies under `-j`; the copy's five builds were not counted; RPV1 :692-702 is a mutation table, HA-251 is at :771-772 (and :80); the gate's `_robot_namespace_problems` (:680-735) fails any non-table, non-team `src/` file naming a robot namespace. | **Fixed:** `grep -o … | sort -u | wc -l`; ~1½ minutes (10 builds + the copy; ~2 if the rows cannot double as the real-tree half); the citation; the ledger uses `namespace shulib::bench::robot_one_facts` (a different identifier) and opens no `robot_one`, with a self-test case. | D4, §1.1 ledger and gate, §4, §7 |
