# MECH, the core — any mechanism described as data, bound to ports after it is made, run by one per-tick service: four control kinds, auton steps, driver presets, the boot binding check, the bench page and the direction check

> **THE TEAM LEAD'S GO (2026-10-08): "Accept all, go" — N1–N13 all as recommended (a).** The core is built right
> after T4 Part 3 lands (N12), in six runs A1–A6, each its own executor run, review round and coordinator
> verification, strictly in order; CAL1 (the standard Calibrate) lands before A5 (N9). Committed to the plan the same
> day as `docs/internal/chunks/MECH-core-mechanism-library.md` (this file). HA-182..HA-186 as §4.3 assigns.

> **Execution brief (DRAFT for the team lead's go).** Written 2026-10-06; **re-checked 2026-10-07 → 10-08** (the
> section right below this quote: 35 findings dispositioned, the design amended in place, the 2026-10-08 route-id
> ruling added). **The tree:** main `2fce38a` (2026-10-08) and **T4 Part 3 as it lands**, read from
> `.claude/worktrees/t4-3`'s working tree copied to `scratch/brief-MECH-core/tree_t43d/` (2026-10-07: both steps
> built, its review fixes, main `fab1ebb` merged and staged, uncommitted; R5a Part A's code from `083e232` laid
> over). Its `include/` is main's PLUS Part 3's 18 changed headers and its new `diag/log_copy_sink.hpp` — five of
> them headers A1 and A4 rewrite (§6.1, N12). **Every file:line below was re-opened in that copy** (`~:N` = within a
> few lines); Part 3's fix round, running as this is written, has since changed four of its files (the
> amendments' "What moved"). Requirements line numbers are main `2fce38a`'s.
>
> **History.** Five drafting runs (2026-09-29 → 10-06) were stopped by usage limits; the 10-06 run re-ran every
> probe against `tree_t43c` (`out4/`); the re-check re-ran every probe in the amended words against `tree_t43d`
> (`out5/`, byte-identical on every re-run) and added three: R (piece sensors whose reads fail), the
> never-tripping stall signature, and P14 (mechanism route ids). **Status (2026-10-08): GO given, N1–N13 answered as
> recommended (the note above). Nothing is built.**
>
> **The team lead's words this brief holds:** "I feel like it shouldnt matter what mechanism a robot has? We
> should be able to make any mechanism that we need to"; "mechanisms should be port agnostic ... bounddd to ports
> after they are created"; "lets do this so we have it, we can do the rest after the panel" (the core first,
> 2026-09-28); "It's complete, write the brief" (2026-09-29); **2026-10-04: "the new programs are kinda
> confusing looking and look hard to write"** — so every line a TEAM writes (a mechanism, its wiring, an auton
> step, a driver control) is short, reads as English, and carries nothing but the team's own decision (§1.1
> shows every one verbatim, compiled, and counted); and **2026-10-08: every mechanism publishes its actions as
> route ids automatically, built in the MECH core** (requirements §4 item 7b; N13, §1.16). His §5 answers of
> 2026-09-29 are RULINGS and are not re-asked: **Q1 (a)** bindings only in the robot file; **Q2 (c)** a sensorless
> grab counted done by time, reported UNCONFIRMED; **Q3** mechanism sensors with the mechanisms; **Q4 (b)** PTO:
> name the later change; **Q5 (a)** no 3-wire expanders; **Q6** robot two's pivots' homing undecided; **Q8 (a)**
> the most recent input wins, auton-only when marked; **Q13 (a)** the team states each position's levels; **Q18
> (a)** typed handles checked at build; **Q25** speedup C (its "alongside" — N12); **Q28** F3/F4 re-scope yes;
> **Q30** the bench page and the direction check in the core.
>
> **Runs:** SIX executor runs (§2), each its own go, each in an isolated worktree (`.claude/worktrees/mech-<run>`,
> branch `mech-<run>`, from the tip at launch), **all after T4 Part 3 lands** (N12) and **CAL1 before A5** (N9).
> A1–A3 are library-only; A4 is library-only but for two one-line `case`s in `src/` (T4); A5–A6 are `src/`.
> **Executor model: `opus`. Do not commit.** Live log: **`docs/internal/chunks/MECH-PROGRESS.md`, created FIRST**
> by MECH-A1, appended as you go, a heading per run. **Hardware Assumptions numbers: HA-182 to HA-186 ONLY** (the
> register's held-numbers line, `docs/hardware-assumptions.md` :39; §4.3 says which is whose; a sixth belief ⇒
> stop and tell the coordinator). Say "host test plant", never "simulator". No hardware claim anywhere: nothing
> here has run on a robot.
>
> **Probes** (evidence, never project code) — `.claude/coordinator/scratch/brief-MECH-core/`: `tree_t43d/` (the
> tree above); **`proposed4/` — the library half exactly as §1 writes it: start from it, it compiles** (29
> headers: 16 changed tree headers, 13 new; strip every `PROBE_*` switch and prototype marker when copying — §2.1);
> `proposed4_src/` (A4's two `src/` lines); `proposed_test3/robot_builder_test.cpp` (the one existing test file
> the library half must edit, §3.1); `team5/` (the team-facing files of §1.1 — the table and driver file in the
> spelling MECH-A5 writes, robot x's files, `counted_steps.cpp` with every counted step and control); `p5/` (every
> probe, regenerated by `tools5/regen.sh` from the 10-06 probes and the checkers'); `neg5/` (66 negative compiles
> + 1 positive, `run5.sh`); `b5.sh` (build one probe against `tree_t43d` + `proposed4`), `run_all5.sh` (re-run
> all), `build_suite5.sh` (the host suite by hand, loud on any failed TU), `gate5/` + `tools5/gate_*.py` (the
> `src` gate's step 0, today's and proposed); outputs in **`out5/`**. Older sets (`proposed/`–`proposed3/`,
> `team/`, `team4/`, `out/`–`out4/`, `neg/`–`neg3/`, `tree*/` other than `tree_t43d`) are history; the checkers'
> probes are in `check-control/`, `check-team-view/`, `check-tree-feasibility/`.

---

## Re-check 2026-10-04 — amendments

> **Run 2026-10-07 → 10-08** (the heading keeps the coordinator's name for this re-check). Three checkers (tree,
> team-facing, design) returned **35 findings** (T1–T12, U1–U12, D1–D11; three pairs overlap: T5≡U7, T9≈D9,
> D3→N10). **Every one was re-verified against the tree as it is now** and **every real one is fixed in the
> design and re-probed**; one sub-claim is rejected with evidence (T9's "captured line"). The tree: main is
> **`2fce38a`** (it moved four times under this run: `fda5351` CAL1 + R5a Part A; **`6517f2d` 2026-10-08 09:32,
> the team lead's ruling that every mechanism publishes its actions as ROUTE IDS, built in the MECH core** — N13;
> `3c643f1` and `2fce38a`, the panel's routes: no simulation in the panel, a geometric visual instead);
> T4 Part 3 as `scratch/brief-MECH-core/tree_t43d/` (the `t4-3` worktree's working tree of 2026-10-07, review
> fixes included, R5a Part A's code laid over). **The evidence:** `proposed4/` (the amended library half — 29
> headers: 16 changed tree headers, 13 new; `proposed3/` is history), `proposed4_src/` (A4's only two `src/`
> lines), `team5/` (the team-facing files, verbatim in §1.1), `p5/` (every probe in the amended words, regenerated
> by `tools5/regen.sh`), `neg5/` (66 build refusals + 1 positive, `neg5/run5.sh`), `gate5/` + `tools5/gate_*.py`
> (the `src` gate's step 0 with the proposed rule), `run_all5.sh` → **`out5/`**, `build_suite5.sh`. **Results,
> re-run 2026-10-08 10:02 on the final overlay:** the host suite **2231/2231 (2 783 067 assertions) pristine AND
> with the overlay** — identical (R5a Part A's larger suite; the F8 signature pins and the byte-exact manifest
> golden included); the ARM header gate **220 headers PASS, 29/29 overlay headers alone**; **66/66 negatives fail
> naming their reason**, the positive compiles; P1, P2a–i, P3, P4, P5b, P5d, P6, P8, P9, P10, P13 re-run with the
> verdicts below; the gate's planted cases **9/9 right under the proposed rule, 5/9 under today's**; three new
> probes: **R** (piece sensors whose reads fail, through the REAL adapters), the never-tripping stall signature,
> and **P14** (mechanism route ids through the REAL `PathRunner` in the REAL `runAuton`). Every `out5/` file
> re-ran byte-identical after each later change. Nothing here has run on a robot. **The body below is amended in
> place**; where it and this section disagree, this section wins and the body is the error.

### What moved under the brief (read first)
- **The route-id ruling (2026-10-08, `6517f2d`; MECH-PANEL-requirements §4 item 7b :515-517 at `2fce38a`,
  REMAINING-WORK row 12):**
  "every mechanism publishes its actions as route ids AUTOMATICALLY (an arm's presets as `arm_stow`,
  `arm_score`; an intake's `intake_run`, `intake_until_piece`), in shulib's manifest that VexBuilder's marker
  picker reads — a coder no longer registers each id by hand", **built in the MECH core**; the panel's route
  editor (G3a Part R, row 20b) reads those ids. This brief now builds it (§1.16, N13 for its open details). The
  same day (`3c643f1`, `2fce38a`) the panel lost its route PREVIEW ("simulating it in the panel is not good and
  should be kept to vexbuilder") and gained a geometric VISUAL; it still "refuses a marker naming a route id the
  robot does not have" — so G3a's validator must derive the same ids from the robot file (§6.3 gives it the rule
  and the four refusals). Nothing in the core simulates a route outside the library's own tests.
- **CAL1** (row 18b, ruled 2026-10-06): Calibrate moves into the library "right after T4 Part 3, beside the
  mechanism core"; CAL1's draft brief (`briefs/CAL1-standard-calibrate.md`, 10-08 00:56) recommends the
  declared-port WATCH in CAL1 (its D4) and DELETES `dropDeclaredMechanismPorts` from `mechanism_table.hpp` in its
  Part B (its §1.8) → **N9**; its §5 lists the amendments it asks of this brief, each made here.
- **T4 Part 3's fix round is running** in `t4-3` as this is written: since `tree_t43d` it has changed
  `src/programs/drive_program.cpp`, `competition_program.cpp`, `auton_tester_program.cpp` and
  `include/shulib/hal/pros/device_provider.hpp` (a comment and `serialCopyDroppedLines()`); nothing the core's
  design reads changed, and A1/A5 re-read every Part 3 file at their tip regardless (§6.1).

### New decisions for the team lead (a recommendation for each; N1–N8 below stand — N6 and N8 amended here, N2's hold now clamped and watched, D2/N10)

**N9 — Calibrate's declared-port drop and its push WATCH belong to CAL1. RECOMMENDED (a).**
- **Why it is a question now.** CAL1 moves Calibrate into the library, "stating only drive kind, motor cartridge
  and its mechanisms' ports" (`fda5351`). Its draft carries both the drop (`CalibrateFacts::mechanisms[]`,
  `{port, name}`, `kMaxMechanismPorts = hal::kMaxSmartPorts` = 21 — CAL1 §1.1) and the WATCH (its D4 (a),
  HA-187, measured on its host rig: `WARNING: port 6 is declared as claw pivot 6 but TURNED WITH THE PUSH`), maps
  today's table in a new `src/robots/calibrate_facts.hpp` (`calibrateFactsFor(const ChassisTable&, const
  MechanismTable&)`, CAL1 §1.1), and deletes `dropDeclaredMechanismPorts` (CAL1 §1.8). The 10-06 draft of
  this brief had A5 edit `calibrate_program.cpp` :1119-1134 and add the watch — in a program CAL1 replaces.
- **(a) RECOMMENDED:** CAL1 owns the drop and the watch (G3a re-check #10 moves with them; CAL1 D4 (a)). MECH-A5
  edits **no Calibrate code**: it rewrites `calibrateFactsFor` (CAL1's file) to read the core's list — for each
  mechanism `i`, one fact per motor role `r < specs[i].motorCount`: `{wiring[i].motors[r], specs[i].name}` (a
  two-motor lift gives two facts with one name) — and its host test pins robot two's facts unchanged (ports 6, 7;
  "claw pivot 6", "claw pivot 7"). CAL1's facts are vocabulary-free, so neither chunk needs the other's types.
  **Order:** CAL1 before MECH-A5 (CAL1 starts when Part 3 lands; A1–A4 are ≥ 13 builder-days). If A5 lands first,
  A5 re-points today's drop call (`calibrate_program.cpp` :1129-1130) at the new list — one line — and CAL1 maps
  the core's list. Either way `declaredMotorRole()` (the 10-06 draft's) is not built: CAL1's §5 asks for names like
  "claw pivot 6 motor 1" through it, but its facts hold a `const char*`, and a per-role name would need a constexpr
  string table of its own — the mechanism's name is what CAL1's measured WARNING already prints ("declared as claw
  pivot 6"). If CAL1 wants the motor's number, it is one `std::uint8_t role` in its `MechanismPort` — CAL1's call,
  carried by the coordinator.
- (b) The core keeps them (A5 edits `calibrate_program.cpp`). **Cost:** an edit to a file CAL1 replaces with 24
  lines; the watch built twice; A5 +½ day.

**N10 — A STALL SIGNATURE is REQUIRED on every Arm, Lift and Flywheel, and every stated one must be able to trip.
RECOMMENDED (a).**
- **What the checkers found, re-probed** (`out5/adv_control.txt`; the 10-06 overlay's numbers in
  `check-control/`): an arm holding after SUCCEEDED, then loaded past what the hold can carry, sat **30 s at
  11.94 V / 2.50 A**, no fault, the verdict still SUCCEEDED (case A); a jammed flywheel sat **30 s at 12.00 V /
  2.50 A** after TIMED_OUT (case H). The service watched a stated stall only during a MOVE; `Speed` had no
  `.stall`; and N7 said "stall signatures stay REQUIRED" while §1.1 made `.stall` optional.
- **(a) RECOMMENDED:** `.stall` is REQUIRED on the three kinds whose behaviour goes on after the step (a hold, a
  spin) — refused at build (`refused::position_the_stall_signature_is_unset…`, `speed_the_stall_signature_is_
  unset…`; `neg5/` n46, n47) — and a stated signature watches EVERY behaviour of a motor kind (move, hold, jog,
  spin, a Power's run and its hold-to-run; homing by stall keeps its own): on a trip, STALLED, `MECHANISM_STALLED`
  once, the declared safe state, the claim released, the command number bumped (a held button is superseded until
  released). **And a signature that cannot trip is refused** (found by this run, the same class): `.amps` must be
  BELOW the motor's 2.5 A current limit (the vendored `pros/motors.h` :1255-1257: "The default value is 2500 mA";
  a limited motor never reads more) — `refused::a_stall_signatures_amps_must_be_below_the_motors_2_5_A_current_
  limit` (n58); `.belowMotorRpm` must be above 0 (`StallDetector` trips on `|speed| <= threshold`, and a stalled
  motor reads a little creep, HA-93) — `refused::a_stall_signature_needs_belowMotorRpm_above_0` (n59). **Measured:**
  case A → STALLED once the load had dragged the arm to its stop (1.7 s at ≥ 2.4 A), then 0 V; case H → STALLED,
  0 V; a jog into an obstacle → STALLED after 0.94 s held; a Power roller's hold-to-run into a jam (its optional
  signature stated) → STALLED at 0.30 s (`out5/adv_extra.txt`); P2h's sensor-against-its-mounting arm → STALLED
  at 1.31 s at the hard stop instead of TIMED_OUT at 3.01 s; **a hand-built 3.0 A signature on case A sat 29.9 s at
  ≥ 2.4 A, 12.00 V commanded, MECHANISM_STALLED 0 — the 2.0 A one STALLED once** (`out5/adv_stallsig.txt`); P2a
  still 143/144. **The cost:** +1 line in every arm, lift and flywheel (§1.1), a number the team states before it
  measures (the guide's recipe: a bit below 2.5 A, a low motor speed, about half a second). A DELIBERATE squeeze
  in a hold (an arm pressing a goal down) trips it too: the team states a higher `.amps` or a longer `.lasting`
  — said in the guide. The declared SAFE state after a trip is the team's: an arm whose safe mode is Hold is then
  held by the motor's firmware at whatever current that draws (HA-144, HA-93) — the rest's current caps, named.
- (b) A library starting signature (HA-182), printed at boot. **Cost:** an invented protective threshold — P6.
- (c) Optional, as drafted. **Cost:** cases A and H (30 s at stall current) — P4.
- A Power mechanism's `.stall` stays OPTIONAL (every step is bounded by its time; the driver's hold-to-run by the
  finger), and a Power with none gets a BOOT LINE saying so: `'claw pivot 6': no stall signature -- held against
  its stop it is cut only by the driver letting go (HA-154)`.

**N11 — The team-facing words, amended for a student reading them. RECOMMENDED (a): adopt all.** *(His words of
2026-10-04; each change answers a slip the team-facing checker compiled, or a message that misled.)*
- `.gearing = gears(12, 60)` (tooth counts, the motor's gear first; `direct`; stages multiply with `*`) replaces
  `.gearRatio = 5`. A bare number is refused BY NAME (n41: the deleted constructor's template parameter is named
  `Write_the_gearing_as_tooth_counts_like_gears_12_60`, because a consteval refusal inside a converting
  constructor printed no reason). *Why:* the drive geometry states its ratio the other way round
  (`hal/drive_geometry.hpp` :67-70, `motorToWheelRatio`, wheel turns per motor turn), and `.gearRatio = 1.0/5`
  compiled and moved a 95° preset 3.8°, reporting SUCCEEDED (`check-team-view/s02`).
- `Flywheel` replaces the `Speed` declaration (read as a quantity); the internal kind stays SPEED.
- `whileHeld(...)` replaces the `hold(...)` control (three meanings of "hold" in one file).
- `toggle(flywheel["shoot"], Button::X)` names the speed it toggles to (the old "its first preset" changed
  silently when presets were reordered); `toggle(flywheel, …)` is refused by name (n51).
- `Stall::belowMotorRpm` replaces `belowRpm` (every other rpm a team writes is the output's).
- `seesPiece.orAfter(2.5_s)` ("… or after 2.5 s, whichever comes first", SUCCEEDED by time) — a missed piece no
  longer has to end the auton.
- ONE list type, `MechanismTable` (the library's own name, the `src` gate's word): robot x's list is
  `kRobotXMechanisms`, robot two's `kRobotTwoMechanisms`; no `RobotMechanisms`, no alias.
- The driver file's and the auton's library names become words: `Button`, `DriverProfile`, `DriverLoopConfig`,
  `ArcadeBody`/`ArcadeField`, `Routine`, `RoutineResult`, `AutonContext`; ONE `Coast`/`Brake`/`Hold` (a
  `BrakeWord`, converting both ways) serves a mechanism's `.safe` and the loop's `.releaseBrakeMode`; the driver
  file states no `.tickPeriod` (§1.10).
- The members are written in ONE order per kind, which C++ requires (`check-team-view/s16`, `s17`: "designator
  order … does not match declaration order"): what it IS (name, motors, cartridge, gearing, shape, angle / piece
  sensor), what it does STOPPED (safe, stall), where it GOES (range, gravity, home, presets / at-speed, presets),
  the extras (unhomedVolts, tuning). The guide prints each kind's order.
- (b) Keep the 10-06 words. **Cost:** the slips above keep compiling.

**N12 — The core's library half waits for T4 Part 3 to LAND; Q25's "alongside" is not kept. RECOMMENDED (a).**
- Q25 (speedup C) said "its LIBRARY half builds alongside T4 Part 3". T4 Part 3 changes 18 library headers (`git
  show --stat 6abcac8 -- include`, plus the worktree's uncommitted `auton_registry.hpp`, `auton_menu.hpp`, and a new
  `diag/log_copy_sink.hpp`), and five are ones A1 and A4 rewrite: `hal/pros/device_provider.hpp` (+87/−5, and the
  fix round is editing it now), `config/robot_config.hpp` (+8/−8), `robot/robot.hpp` (+15/−12),
  `competition/auton_registry.hpp` (+56/−6), `teleop/teleop_loop.hpp` (+10/−9). REMAINING-WORK §3 already
  sequences the core after Part 3.
- **(a) RECOMMENDED:** A1 starts when Part 3 has landed (its fix round, verification and merge are next).
  **(b)** A1 starts now from main in its own worktree and rebases. **Cost of (b):** five header conflicts,
  re-solved at A1 and again at A4.

**N13 — Mechanism ROUTE IDS (his ruling of 2026-10-08): how a marker runs. RECOMMENDED (a).** *(The ruling
decides THAT the ids exist, automatically; these four details are what it leaves open.)*
- **The rule the core builds (not a question):** the ids are DERIVED at build from the robot's list — the same
  declarations autons and driver files name, so an id cannot drift from its mechanism — one rule per kind (§1.16):
  an arm's or lift's `<name>_<preset>` and `<name>_home`; a flywheel's `<name>_<preset>` and `<name>_stop`; a
  power mechanism's `<name>_run`, `<name>_until_piece` / `<name>_until_gone` (when it declares a piece sensor) and
  `<name>_stop`; a pneumatic's `<name>_<position>` (the name in route-id words: "claw pivot 6" → `claw_pivot_6`).
  F8's vocabulary holds for every id; an id longer than 23 characters, two ids alike (a preset named "home"), an id
  that is a built-in (`wait_ms`) and more than 32 ids are each REFUSED AT BUILD at the robot's list, by name
  (n60–n63) — never a silently missing id. ONE call registers them all (`runner::MechanismRoutes`).
- **(a) RECOMMENDED:** (1) a marker **BLOCKS at its waypoint** until the step's verdict — markers fire only after a
  leg settles (F8's ruling R2), so a route marker never runs while driving; a step started mid-leg is the rest's
  "triggers partway through a drive" (row 14) — and it runs THROUGH the Routine (one verdict policy: SUCCEEDED and
  UNCONFIRMED-counted continue, everything else stops the route as `MarkerFailed`); (2) **the time limit is the
  marker's own arg** (`arm_score [1500]`; `intake_until_piece [12, 3000]` = volts, ms) — P4 bounded, P6 the route
  author's number; a marker missing it, or volts outside ±12 or 0, is REFUSED with one line naming the id and what
  it needs, and the route stops; (3) **each id's one-line doc in the manifest** (an F8-ADDITIVE `CommandEntry::doc`
  and `on(id, handler, argCount, doc)`; the manifest's `doc` field already exists and a registry without docs
  writes the same bytes as today), so the picker shows "args[0] = volts, args[1] = how long, ms" instead of a bare
  "2 args"; (4) **capacity stays F8's 32** (`runner::kMaxCommands`): a robot's ids ≤ 32 at build (n63), and the
  team's own registrations share the rest (a registration past it is the registry's precondition, host-tested
  per robot in A5).
- **Measured (P14, `out5/probe5_routes.txt`):** robot x publishes 17 ids, robot two 6 (`claw_pivot_6_run`,
  `claw_pivot_6_stop`, `claw_pivot_7_run`, `claw_pivot_7_stop`, `claw_closed`, `claw_open`), each with its doc, by
  the REAL `writeManifest`; a 3-waypoint route of 7 generated markers through the REAL `PathRunner` inside the REAL
  `runAuton` under the guard, the service in the chain: ROUTE_RAN, 7 fired, every one SUCCEEDED (the intake saw a
  piece at 0.97 s, the arm reached 'score' in 0.90 s, the hood 0.31 s, the flywheel at speed 0.64 s, the feed
  0.51 s), the end action succeeded; robot two's route: the sensorless grab UNCONFIRMED, counted done, the route
  went on (4 fired); `arm_score` with no args and `intake_run [13, 500]` each REFUSED with its line, the route
  stopped `MarkerFailed`, the drive safed, the end action ran. ARM `-Os`: `MechanismRoutes` 4 804 B (static
  storage), `CommandRegistry` 1 156 → 1 284 B.
- (b1) A marker STARTS its step and returns (the step runs through the next leg), a `<name>_wait` id collecting
  it. **Cost:** two ids per action (32 fill twice as fast); a failed step surfaces a marker late. (b2) One time limit
  per mechanism in the robot file (`.routeTime`). **Cost:** a field per mechanism holding a number the team
  cannot know per use. (b3) No docs. **Cost:** the picker shows "minArgs 2" and nothing else. (b4) `kMaxCommands`
  32 → 64 (+1.3 KB). **Cost:** a change to a constant F8's register calls a design constant.
- **Cost of (a):** A1 +¼ (the derivation, its four refusals, their tests), A4 +¾ (the binder, the F8-additive doc
  and its pins, P14 as a test, mutations M44–M45), A5 +¼–½ (a registry in the competition program and the Auton
  Tester, registered at boot after the service binds and passed to `runAuton`; each robot's manifest pinned by a
  host test). Where the manifest FILE lives for the panel and VexBuilder (a generated file, or the panel computing
  it from the robot file) is G3a Part R's; the core gives `routeIdsOf()` (constexpr), the binder and the docs.

**N8, amended (the question stands; its scope is corrected).** A piece sensor is read as POSITIVE evidence: YES /
NO / UNKNOWN (§1.7). UNKNOWN satisfies NEITHER condition, and is: a sensor declared but missing this boot; a
distance reading held near with confidence 0; and **any sample taken while the sensor's read-fault count moved**
(the read-fault channel, decided below). **(a)'s counted-done rule applies only to a sensor DECLARED but MISSING
this boot** (keyed on the declaration vs the devices, never on the devices alone). A condition on a mechanism that
declares NO sensor is a mistake refused at the step (§1.7). A sensor that stops answering MID-STEP ends the step at
its bound UNCONFIRMED, NOT counted ("its piece sensor stopped answering"). Measured: `out5/adv_control.txt` J →
UNCONFIRMED after 2.00 s; `out5/adv_readfaults.txt` R2/R3/R8 → UNCONFIRMED after 1.00 s (were false SUCCEEDED);
`out5/probe4_missing_sensor.txt` → both conditions UNCONFIRMED (counted) after their 1.00 s with the motor run.

**Decided here, overrulable (added to the header's list):**
- **RoutineStopCause gains `MechanismStepFailed`** (appended; F10's sanctioned path, F1's precedent;
  `MechanismFailed` keeps its F1 meaning and its doc comment untouched; pinned `== 5` beside the existing pins in
  `test/routine_signature_pin_test.cpp` :347-350). The context constructor is a "new overload" under `version.hpp`
  :24-28; the 2.5 changelog and the F10 register row say both. It costs A4 two one-line `case`s, in
  `src/programs/competition_run.hpp` :559 and `auton_tester_run.hpp` :271 (their `-Wswitch` would otherwise fail
  the gate) — the only `src/` lines A1–A4 touch.
- **A declaration's provenance is a COMMENT**, no longer build-checked (today's check proves a string is non-empty,
  never that it is true; the facts that matter stay in the region's comments AND the HA register; G3a emits notes
  as comments). Overrulable by an optional `.note` the list requires non-empty.
- **The `src` gate's rule is per ROBOT** (§1.13).
- **`driverHeading()` moves to its own `src/common/driver_heading.hpp`** (the X-drive uses it and has no driver
  file).
- **Done-conditions that can never be met are refused at the STEP, not the build** (the condition and the
  declaration are two arguments of one run-time call; a build check would need `seesPiece(intake)` everywhere);
  the host test of every registered auton (A5, U3) runs each step against the robot's REAL list, so the refusal
  fails `ctest` before any upload.
- **NEW — the READ-FAULT channel** (found by this run's R probe: the 10-07 positive-evidence rule caught a sensor
  held NEAR, not one held FAR). The F4 sensor seams have no validity channel by design, and a PROS adapter
  answers a failed read with its last good value — so a distance sensor that failed while it read "far" (or from
  boot: its hold starts at the far no-object distance) is byte-for-byte the 9999 "no object" reading, and an
  optical or a switch holds its last level. **Fix:** `hal::IReadFaults` (one member, `faultedReads()`; an
  F4-ADDITIVE sibling like `IDigitalIn`), implemented by `ProsDistance`, `ProsOptical`, `ProsDigitalIn` (their
  existing `faultedReads()` gains `override`); `IDeviceProvider::readFaults(port)` and `lineReadFaults(line)`
  (pure — all four implementers answer; the test-local two forward); `mechanism::Devices::pieceFaults` and
  `homeSwitchFaults`; and the service's rule: **a sample taken while that count moved since the service's
  previous tick is UNKNOWN** (a piece sensor) or **not pressed** (a home switch). Measured through the REAL
  adapters made by the REAL `ProsDeviceProvider` over the PROS shim (`out5/adv_readfaults.txt`): R2 (cable out while
  the intake was empty, then a piece loaded, eject), R3 (cable out from boot) and R8 (an OPTICAL that failed while
  empty) each SUCCEEDED "stopped seeing a piece" at 0.00 s with the motor never run under the 10-07 rule; with the
  channel each runs its 1.00 s and ends UNCONFIRMED, not counted; R1 (working: SUCCEEDED at 0.50 s), R6 (`orAfter`:
  SUCCEEDED by time at 0.50 s) and R7 (a 0.2 s drop-out, then the piece gone: SUCCEEDED on the fresh read at
  0.40 s) are unchanged; S — a home switch dead from boot, in the world where HA-184 is wrong (pressed = low),
  homed the arm at bind wherever it stood; with the channel it does not. The suite (2231/2231) and the ARM gate
  pass with it. Cost: A1 +¼, A2 +¼. What it cannot see: an UNPLUGGED 3-wire switch, which reads a level, not
  PROS_ERR (HA-184 settles what it reads).
- **NEW — a stated stall signature must be able to trip** (n58, n59; N10).

### Dispositions — one per finding (T = tree, U = team-facing, D = design; ⇒ where the fix lives in the body)

- **T1 (major) Brief predates main's tip; CAL1, SAFE-SAVE B2, L9 Part B not named; R5a "in review".** REAL.
  Re-based (main `2fce38a`; R5a Part A landed as `083e232`, library-only — `sim/drive_plant.hpp`, not the core's
  `mechanism_plant.hpp`). Calibrate's drop + watch → **N9** (CAL1's cap is 21 in its current draft, not 12 — no
  cross-brief item remains). §6.1 now names CAL1 (`src/robots/calibrate_facts.hpp`), SAFE-SAVE B2
  (`src/common/boot_source.*`, the record read: A5 edits `bootConfigFrom` and the declared-port rule — different
  functions) and L9 Part B (moves `decideBootSourceFrom` into the library and DERIVES the declared ports from the
  base config's mechanisms — that derivation must include A1's sensor ports `rotationPort` / `distancePort` /
  `opticalPort`); order CAL1 → … → MECH-A5 → L9 Part B, each re-reading the others' hunks; requirements lines
  refreshed against `2fce38a` (§4 item 7b :497-517, §5 :537, §6 :728, §7.1 :752, §7.2 :773, §7.3 :795).
  ⇒ header, §0.1, §6.1, N9.
- **T2 (major) False "include/ equals fab1ebb's"; five shared headers; the Q25 departure unstated.** REAL
  (re-measured: 18 differ + `diag/log_copy_sink.hpp` new). ⇒ header corrected; §6.1 lists the five and requires
  A1/A4 to re-read them at their tip; **N12**.
- **T3 (major) The `src` gate does not protect the per-robot declarations; the `src/team` exception admits any
  robot.** REAL — re-probed (`gate5/`, `tools5/gate_cases.py`, `out5/gate_cases.txt`): today's step 0 PASSES a
  program naming `robot_two::claw`, a program's `using namespace robot_two;`, and `src/common` re-opening robot
  two's namespace. ⇒ the proposed rule (`tools5/gate_patch.py`, §1.13): step 0 derives each robot's NAMESPACE from
  the mechanism table file (a namespace that declares a `MechanismTable`) and refuses `<ns>::`, `using namespace
  <ns>` and re-opening `namespace …<ns>` outside the table files, `test/`, a `static_assert` and that robot's own
  team files (`src/team/<ns>_*`, or `TEAM_FILE_ROBOT` — today `src/team/autons.{hpp,cpp}` → `robot_two`, until G3a
  Part 4's per-robot auton files); the per-robot-table exception is tied to the same robot. Planted: 9/9 right (5/9
  today). A5 +½; self-test cases for each, mutations M33/M34.
- **T4 (major) F10: a frozen stop cause reused; the constructor outside routine.hpp's additive list.** REAL. ⇒
  `MechanismStepFailed` appended (Decided here); its `src/` consequence measured (`proposed4_src/`: the two
  programs' `-Wswitch`); the 2.5 note cites `version.hpp` :24-28 for the constructor. ⇒ §1.9, §2, §7.
- **T5 ≡ U7 (major) "Refused at the line that is wrong, naming the reason" overclaims; 11–12 generic C++
  errors.** REAL. ⇒ (1) the type-shaped slips get NAMED reasons (`neg5/`): a plain number for volts → "volts are
  written with their unit: 12_volt, not 12"; `r.run(arm, …)` / `r.move(intake, …)` / `r.stop(arm)` → a sentence
  saying which step to use; `lift.to(18)` → `a_lift_height_needs_inches_like_to_18_in`; `intake["fast"]` →
  `a_power_mechanism_has_no_presets_run_it_with_r_run`; `motors('A')` / `lines(6, 7)` / `homeSwitch(66)` /
  `angleSensor('B')` → named; `wire(hood, motors(…))` / `wire(arm, lines(…))` → named; `whileHeld(hood, …)` → named;
  `.gearing = 5` → named. (2) A port, line or button used twice NAMES IT (`smart_port_4_is_used_twice`,
  `three_wire_port_C_is_used_twice`, `button_Right_does_two_jobs`). (3) The true locations, measured
  (`out5/neg5_summary.txt`): 12 of 66 errors land ON the team's line; the other 54 land in a library header with the
  team's line in the first "in 'constexpr' expansion of" / "required from here" note; a cross-entry refusal (a
  port, line, button or name twice) points at the list's FIRST line (`neg5/m1`, `m2`: line 2) and names the
  port/button; a declaration's refusal points at its `wire(...)` line (`m3`: line 15, declared on 12). Still
  generic, said so: `90_deg` → "call to non-'constexpr' function operator""_deg" (names `_deg`, at the line); a
  run-time value where a constant is checked → "is not a constant expression"; a misspelt mechanism → "was not
  declared in this scope". ⇒ "The design in plain words", §1.2/§1.4's rule rewritten, §3.0's harness gets the
  multi-line TUs and fixed-string reasons, the guide's "reading a refusal" lesson.
- **T6 (minor) A5 deletes `driver_controls.hpp` but the X-drive, a 589-line test and the README use it.** REAL, and
  wider (grep of `tree_t43d`): `src/programs/drive_program.cpp` :91, `src/programs/xdrive_program.cpp` :54 and
  :237, `src/common/driver_panel.hpp` :75 and :128, `src/team/robot_two_driver.hpp` :43, `test/driver_controls_test.cpp`
  (589 lines, its :542-574 heading-rule pin), `test/drive_program_source_test.cpp` :282 (a source pin naming the
  file), `test/robot_two_driver_test.cpp` :13 and :36, `test/driver_panel_claws_test.cpp` :13, :41, :614-615,
  `test/mechanism_table_test.cpp` :90 and :729, `src/common/boot_source.hpp` :98, `src/robots/mechanism_table.hpp`
  :120, :139, :213, :417 (comments the rewrite removes), `src/team/README.md` :10, `docs/cookbook/06-driver-control.md`
  :292, `docs/roadmap.md` :1785-1786. ⇒ `driverHeading()` → `src/common/driver_heading.hpp` (the X-drive includes
  it, so it is no longer "untouched"); `test/driver_controls_test.cpp` splits — its heading cases move to
  `test/driver_heading_test.cpp`, its `HoldControl` / `ToggleControl` / `DriverBindings` cases are DELETED with the
  types (their rules are now `controls(...)`'s, tested in A4 and by `neg5/`); every file named is in A5's list.
- **T7 (minor) "All eight builds link": Part 3 leaves seven.** REAL (`tree_t43d/tools/src_build_gate.py` :464-472).
  ⇒ "all seven builds link (the gate's table at the tip)".
- **T8 (minor) The overlay carries probe-only switches and stale words.** REAL (`PROBE_NO_MECH_CUT`,
  `PROBE_SETTLE_SPEED`, `PROBE_TIMEOUT_HOLDS_TARGET` — still in `proposed4/`, 4 uses, because the probes need
  them; "(proposed)" / "PROTOTYPE" markers). ⇒ §2's DoD: strip every `PROBE_*` and prototype marker from copied
  code; §7 adds `grep -rn PROBE_ include/` = empty; the direction check's thresholds are HA-182's page entries.
- **T9 ≈ D9 (minor/major) HA-184 and HA-186 mis-cited; requirements lines; the captured-line rule.** HA-184 and
  HA-186: REAL — vendored `pros/adi.hpp` :711-712 is inside `get_new_press()`'s doc (:705-729); `get_value()`'s is :731-757 ("Gets the
  digital value (1 or 0) of a pin"); `pros/rotation.hpp` :244-266 is `get_angle()`, `get_position()` is :194-216.
  ⇒ §4.3 re-cited; HA-184's confidence is **unknown** (not "reasoned"): no vendored text says which level means
  pressed or what an UNPLUGGED input reads; HA-121 (`get_value()` is a level) is its neighbour; station 13 reads
  pressed, released AND unplugged; the guide states what a wrong HA-184 does (every switch-homed mechanism zeroes
  wherever it stands at boot) and the nets now measured: the move into the hard stop STALLS (case F: 1.70 s, the
  required signature), and a switch that FAILS from boot no longer homes it (S). HA-186's settle adds a PROGRAM
  restart without a power cycle after moving the arm. **The captured-line sub-claim is REJECTED with evidence:**
  `team/robot_x.hpp` :25 IS the lift's `.name = "lift",` line (:24 `inline constexpr Lift lift{`, :26 `.motors`) —
  every role points at the `.name` line, as the brief said; the amended P7 prints `team5/robot_x.hpp:28`, again the
  `.name` line (`out5/probe3_direction.txt`). Requirements lines: T1.
- **T10 (minor) The size basis calls estimates "measured" and mislabels one.** REAL. ⇒ §2 re-worded: the briefs'
  ESTIMATES (T4 Part 2 BEFORE its 2a/2b split ~2–2½, Part 2a a subset of it; TANK1 Part A ~1¼ for one behaviour;
  SAFE-SAVE Part A ~1–1½, its own brief :42); the sizes re-derived (the final runs, below).
- **T11 (minor) The compile-refusal harness is not in CI; A1–A4 never run the `src` gate.** REAL (`ci.yml` :102 runs
  only `shulib_tests`; the suite's `-iquote`/defines are PRIVATE, `test/CMakeLists.txt` :103, :118, :124). ⇒ A1
  adds `ci.yml`'s ctest step and an INTERFACE target carrying the suite's flags (the harness links it); every run's
  DoD runs `python3 tools/src_build_gate.py check` (A1–A4 change headers `src/` includes).
- **T12 (minor) Provenance becomes an unchecked comment unlisted; the `mechanismsDisjointFromChassis` rename not in
  §6.3.** REAL. ⇒ Decided here; §6.3 adds the rename (G3a cites the old name at :298, :2689).
- **U1 (major) `gearRatio`'s direction is the library's opposite; a swapped value compiles; nothing on the bench
  shows the mechanism in its units.** REAL. ⇒ N11's `gears(driving, driven)`; §1.14: station 13 shows each
  position mechanism's reading IN ITS DECLARED UNITS from power-up (degrees through the gearing, inches through the
  shape), so a wrong gearing is a wrong angle on the screen and the team can read off `.range`, presets and the
  level angle by moving the arm by hand; §6.3's JSON `gearing: [[12, 60]]`.
- **U2 (major) A position's level count is never checked against the wired lines.** REAL. ⇒ `Levels` records how
  many levels each position states; `wire(Pneumatic, lines(…))` refuses a mismatch BEFORE the spec's own checks, by
  its own name (`a_position_states_a_different_number_of_levels_than_the_wiring_has_lines`, n42, n43 — n43 was
  refused for the wrong reason before).
- **U3 (major) The team's auton workflow is not wired for mechanism steps.** REAL (`src/team/autons.cpp` :53
  `Routine r{ctx.chassis, …}`; `test/autons_test.cpp` :143-168 runs every registered auton through
  `test/auton_tester_rig.hpp` :247-262 → `runTesterOnce` → `runAuton` with no service). ⇒ A5: `autons.cpp` :53 →
  `Routine r{ctx, "40in-forward"}` (the only spelling the guide teaches); the tester rig builds a
  `MechanismService` in its chain (guard → service → watch), bound to the build's list over the §1.12 models, and
  `runTesterOnce` takes the service overload; `test/autons_test.cpp` gains a case that registers a claws auton and a
  claws ROUTE and asserts each step's VERDICT (not only "ended safe"); the three files are in A5's list and §3.5.
- **U4 (major) Switch- and stall-homing never check the push direction.** REAL. ⇒ spec refusal
  `position_the_homing_push_points_away_from_its_home_end` (home at the bottom limit pushes −, at the top +; a home
  strictly inside the range may push either way): n44, n45.
- **U5 (major) A switch- or stall-homed mechanism with no driver-side homing has dead buttons after a restart.**
  REAL. ⇒ `controls(...)` refuses it (`a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control`, n48)
  unless the file states `homeAtDriverStart`, `homeOnPress` or the declaration `.unhomedVolts` — robot x's driver
  file now carries `homeAtDriverStart(lift, 1.5_s)`; and the never-homed cases too: a hold with no `.unhomedVolts`
  (n49), a preset on a never-homed mechanism (n50). The boot prints each position mechanism's homed state (§1.13);
  the guide's homing section states the restart case for `restsAtStop` (it trusts the boot pose).
- **U6 + U12 (major/minor) N6's driver file drops today's tick check; `.tickPeriod` is not a team decision;
  plumbing uncounted.** REAL. ⇒ `DriverFile` supplies the brain's tick (`ProsTickPacer::kTickMs`) and
  `static_assert`s that a stated one is the brain's (n56 — today's sixth check kept); the team writes no
  `.tickPeriod`, no `using Button`, no library namespace (N11's words); N6 now says where each of the six checks
  lives (§1.1); the count shows each auton's fixed cost (signature, `Routine r{ctx, …}`, `return r.result();`, and
  COMP-t's 8-line registration, unchanged) and `autons.cpp`'s new `#include "robots/mechanism_table.hpp"`.
- **U8 (major) Robot two's rewrite reaches "13 lines" by deleting its caveats.** REAL. ⇒ the region keeps one
  comment per caveat (HA-153 the unseen ports, the unmeasured signs, HA-154 the driver as the only cut-off, HA-156
  the unmeasured loaded hold, HA-159 the guessed level and single/double, HA-155 the invisible swap with a drive
  cable): **10 comment lines + 13 code lines**, counted honestly (§1.1); plus N10's boot line for a Power with no
  stall signature.
- **U9 (minor) Robot x teaches the list name the table file forbids.** REAL. ⇒ `inline constexpr MechanismTable
  kRobotXMechanisms` (N11); the guide says a driver file names its own robot's list.
- **U10 (minor) "Until a piece, else carry on" has no word.** REAL. ⇒ `seesPiece.orAfter(2.5_s)` (N11), compiled
  in P5b and the positive TU, run in P1 (robot x's `scoreThree`, 10/10) and R6.
- **U11 (minor) Names a student will misread.** REAL. ⇒ N11 (`whileHeld`, `Flywheel`, `belowMotorRpm`,
  `toggle(flywheel["shoot"], …)`); "no name typed twice" corrected (a declaration types its identifier and its
  `.name` — the run-time lookup is by `.name`); §6.3 adds G3a's identifier rule: the C++ identifier is the JSON
  `name` in lowerCamelCase, the JSON may override it with `id`, and the generator refuses two that collide.
- **D1 (major) Driver → auton handover: a PROS-deleted driver task leaves its behaviours running into the auton.**
  REAL (re-probed: `out5/adv_handover.txt` — 257 of 269 paces at 12 V before; **0 of 269** with the fix, with or
  without `leaveDriverControl()`). ⇒ `MechanismService::enterAutonomous()` (clears the re-entrancy flag a deleted
  task left; releases a foreign leftover claim WITHOUT its `cancel()`, counted; motor kinds safe; pneumatics keep
  their level; a running step CANCELLED), called by `runAuton`'s service overload before the guard's run; landmine
  2 covers both directions; test + mutation M35.
- **D2 (major) The runtime soft stop is open-loop.** REAL. ⇒ at or past a limit, pushing outward, the HOLD loop
  holds the LIMIT (P+I+D + gravity), entered at the stopping distance at the profile's own acceleration (no new
  number); the release never targets past a limit (clamped), and N2's hold-where-stopped is clamped too — **a
  latent defect this re-check found**: `out4/probe3_driver.txt` held a target of −1.30° on a 0..160 arm; now 0.00.
  Measured (`out5/adv_softstop.txt`, `adv_extra.txt`): the checker's case (plant kG 1×/1.5×/2×) lowest 18.32 /
  17.94 / 16.06° at the 20° limit (was 13.16 / −3.99 / −10.00 = the hard stop); both limits arriving at 8 V: worst
  0.97–4.06° past, the release 0.00–3.56° (a 2× heavier arm sags under the hold until its integral catches up — a
  gravity mismatch the team tunes out with `.gravity`). Case C (`restsAtStop` wrong): the jog now stops at the
  believed zero (true 29.20) instead of −1.16. The guide: limits sit short of the hard stops by more than the
  measured overshoot. M16 is red for both limits.
- **D3 (major) A blocked HOLD and a jammed flywheel sit at stall current.** REAL. ⇒ **N10** (with the two
  never-trip refusals this run added).
- **D4 (major) "Reached" can never be met at a table's top or a linkage's maximum.** REAL. ⇒ judged in the
  mechanism's OWN units (`|angleDegToPosition(jd) − target| ≤ tolerance`): case G → SUCCEEDED at 1.91 s (was
  TIMED_OUT); §3.3 tests a target AT each limit for every geometry; mutation M36.
- **D5 (major) The prose says the N1 cut never runs inside the end action; the code (rightly) cuts on an abandon.**
  REAL. ⇒ all five places reworded ("the DEADLINE cut never runs inside the end action; an ABANDON cuts claimed
  mechanisms everywhere, the end action included — B1"); §3.2's test split; mutation M37. Measured
  (`out5/adv_endaction.txt`): a disable mid end action → the step CANCELLED, returned at T+1.52, 0.1 V·s.
- **D6 (major) The direction check gives the wrong fix for a swap and misnames a fight of equals.** REAL. ⇒ three
  answers (its + way / the other way / it did NOT move or something ELSE moved), the third naming the mechanism's
  `wire(...)` line (captured where `motors(...)` / `lines(...)` is written: P7 prints `team5/robot_x.hpp:75`, the
  `wire(arm, motors(1))` line); "every motor still, every one drawing" → "its motors FIGHT (one declared the wrong
  way) or it is JAMMED"; a motor that neither moved nor drew → its cable and its wire line; a pneumatic's "nothing
  moved" → its line / tubing / air. The page says "ONLY '<name>' should move" before asking. Verbatim in
  `out5/probe3_direction.txt` and `adv_control.txt` K (K's own `team/robot_x.hpp:66` is a hand-set stand-in; the
  real capture is P7's).
- **D7 (major) A done-condition that can never be satisfied compiles; `stopsSeeingPiece` with no sensor
  SUCCEEDED.** REAL. ⇒ refused at the STEP (Decided here): `out5/adv_conditions.txt` — all three now a
  precondition naming the field to add, nothing commanded; P5's claim narrowed to `seesPiece(other)` (the only
  build-time form, n39, n40).
- **D8 (major) A distance sensor lost mid-run gives a false "stopped seeing".** REAL — and wider than the checker's
  case. ⇒ N8 amended (positive evidence) fixes the checker's J (held NEAR) → UNCONFIRMED, not counted; this run's R
  probe found the held-FAR cases (R2, R3, R8) still false-SUCCEEDED under that rule, so the READ-FAULT channel
  (Decided here) closes them: UNCONFIRMED after the bound, measured through the real adapters. **Probe consequence:**
  `FakeDistance`'s default (0 in, confidence 0) reads UNKNOWN; a test's "no piece" sets the adapter's 393.66 in
  (`tools5/patch_far.py`) — landmine 21.
- **D9 ≈ T9** (above).
- **D10 (minor) Homing into a jam sets a wrong zero silently.** REAL. ⇒ every (re-)home logs where zero went
  (`'arm': homed -- 2.0 deg from where its encoders read zero at boot`; a re-home: `homed again -- its zero moved
  25.0 deg`, Warn past the tolerance); the guide says stall-homing trusts the first stall and `restsAtStop` the boot
  pose.
- **D11 (minor) A starved service leaves the motors at their LAST voltage.** REAL (case I: 11.74 V held 1.5 s, 0.89 s
  against the hard stop at 2.45 A). Not fixable inside the service (nothing runs when nothing ticks). ⇒ §1.9 and the
  guide say it plainly: a starved service leaves each mechanism at its last command — not held, not limited, not
  safe; use `r.pause()` / `waitFor`, never `pros::delay`, in an auton, and a `then()` action must not block.

### The final runs (each its own go, strictly in order, after T4 Part 3 lands — N12; CAL1 before A5 — N9)
| Run | Size (builder-days) | 10-06 draft |
|---|---|---|
| **MECH-A1 — the binding and the devices** (+ N11's words, U2, U4, N10's refusals and the two never-trip ones, the named refusals, the read-fault seam and the provider's two accessors, the route-id derivation and its four refusals, the harness + CI) | **~3½–4½** | ~2½–3½ |
| **MECH-A2 — the service, power, pneumatic, the guard's cut** (+ the evidence rule and the read-fault rule, the step preconditions, `enterAutonomous`) | **~2¾–3¾** | ~2–3 |
| **MECH-A3 — position and speed** (+ the closed-loop soft stop, the stall watch on every behaviour, reached in units, the clamps, the zero log) | **~3½–5** | ~3–4½ |
| **MECH-A4 — autons, the driver loop, the direction judgement, the route binder** (+ `MechanismStepFailed` and its two `src/` lines, the refusal templates, the words, U5's refusals, the three answers, `runner::MechanismRoutes` and the registry's doc) | **~3¾–4¾** | ~2½–3½ |
| **MECH-A5 — robot two and the programs** (− Calibrate → CAL1; + the gate's per-robot rule, `driver_heading.hpp`, the tester rig's service, the boot lines, the route registry in two programs) | **~3¾–5** | ~3–4 |
| **MECH-A6 — station 13 and the direction page** (+ the readings in declared units, the three answers) | **~2–3** | ~2–3 |
| **The core** | **~19¼–26** | ~15–21½ |
---

## Decisions for the team lead (NEW — a recommendation for each; nothing below decides them silently)

**N1 — The run guard's deadline also CUTS a running mechanism step. RECOMMENDED (a).**
- **What the probe found (P6, `probe3_deadline.cpp`, built twice).** A Routine's mechanism step waits through
  `Chassis::waitUntil` — a FROZEN wait the guard cannot cut (`sequence/run_guard.hpp` ~:103: "THE UNEXPIRED
  REMAINDER OF THE WAIT'S OWN TIMEOUT"); the guard's latch cuts only the active MOTION (`cutActiveMotion()`
  ~:653). An intake step `r.run(intake, 12_volt, seesPiece, 5_s)` still running when the auton's `endActionAt`
  arrives keeps running to its own bound:

  | the step's bound; what arrives | the tree's guard | (a) the latch also cuts claimed mechanisms |
  |---|---|---|
  | 2 s; the deadline at 1 s | scoring returns **1.01 s late**; the intake ran **12.0 V·s** past the cut; the end action ran | 0.01 s late; 0.0 V·s; step CANCELLED; end action ran |
  | 5 s; the deadline at 1 s | **2.02 s late**, 24.1 V·s; the HARD STOP fired during scoring and **the end action FAILED** (the park never happened) | 0.01 s late; 0.0 V·s; end action succeeded |
  | 2 s / 5 s; the field DISABLES at 0.5 s | returns **1.51 / 2.52 s late** — COMP-b's `opcontrol()` waits for its auton task that long; 18.0 / 30.1 V·s | 0.02 s late; 0.1 V·s |
- **(a) RECOMMENDED:** in the two branches that cut the motion — the DEADLINE (never inside the end action, which
  stays exempt from it) and an ABANDON (everywhere, the end action included — B1; re-check D5) — the latch also calls `cancel()` on every listed mechanism whose claim has a
  registered claimant (a service slot, an F1 operation), so a step in a frozen wait ends CANCELLED at the next
  pace. Unclaimed mechanisms are not re-safed every pace (a repeated Hold re-captures its point, HA-144);
  `run()`'s cancel-all still safes every one once, as today. `RunGuardReport::mechanismCuts` (appended) counts
  them. F12 is unfrozen ("FREEZES NOTHING", `run_guard.hpp` ~:163): no version bump, but a BEHAVIOUR CHANGE in the
  changelog (an F1 operation with a registered claimant now stops at the deadline, not at its own bound). The
  whole suite passes with it (2148/2148 on `tree_t43c`; 2231/2231 on `tree_t43d`, the re-check).
- (b) Leave the guard alone and document that a mechanism step's bound is its lateness. **Cost:** the table — a
  slow step can cost the park, and after a disable it delays driver control by up to its bound.

**N2 — A position move that runs out of time HOLDS WHERE IT STOPPED. RECOMMENDED (a).**
- **The probe (P2c, `probe2_position.cpp`, built twice).** An arm blocked at 60° while moving to 120°, with no
  stall signature stated (a new robot file, before the team measures one): the move ends TIMED_OUT at 3.01 s;
  over the next 2 s —
  - **(a) hold where it stopped:** mean 1.68 V (worst 1.77), 0.37 A, the arm held at 59.7°;
  - **(b) keep holding the unreached target:** **12.00 V and 2.50 A — the stall signature — indefinitely**, until
    another command, the guard or a disable. P4: "nothing sits at stall current beyond its stated cap".
- **The cost of (a), measured (P2a).** Of 144 moves over a deliberately mismatched grid (2 cartridges × 2
  gearings × plant kV ×0.8/1/1.25 × kS 0.3/0.6 × kG 1.2/1.6/2.4, with the library's starting values), **143
  SUCCEEDED**; the one that timed out (Green 7:1, kS 0.6, kG 2.4 — a plant slower than its own cartridge) ends
  1.74° short under (a) and 0.95° short under (b). Either way the step says TIMED_OUT and the auton stops on it.
- A SPEED mechanism is the exception on purpose: a flywheel that times out spinning up keeps its target (it is
  meant to keep spinning); a jammed one is the rest's current protection.

**N3 — Robot two's claw MOVE TIME: a provisional 0.5 s, registered HA-185, timed at Station J. RECOMMENDED (a).**
- A pneumatic step ends after the cylinder's move time; with no sensor on the claw, that is when a grab is
  counted done, UNCONFIRMED (Q2 (c); P9b: `'claw': UNCONFIRMED -- time in 0.51s`, the chain goes on). The core
  REFUSES a pneumatic mechanism whose `moveTime` is not stated (P6: no invented default; `neg5/` refuses it at
  build). Nobody has stated robot two's.
- **(a) RECOMMENDED:** `.moveTime = 0.5_s` in the claw's declaration, marked PROVISIONAL the way the claw's open
  level already is (`src/robots/mechanism_table.hpp` ~:159-171: a stated guess, one place, a host pin, an HA
  entry, the Station that settles it). Station J times the claw (closed → open, a phone's slow-motion video); the
  number, its pin and HA-185 change together. It affects only an auton's `set()` step; the driver's toggle does
  not wait on it.
- (b) He states a number now. (c) No number until measured: robot two's table does not compile until Station J,
  so MECH-A5 cannot land.

**N4 — T3's driver bindings after the core: kept, marked superseded, removed later on his word. RECOMMENDED (a).**
- After MECH-A5, robot two's buttons run through the mechanism service (`mechanism::DriverInputs`, §1.10). T3's
  `teleop::MechanismBinding` / `teleop::ToggleBinding` (`teleop/mechanism_binding.hpp`) and their
  `runDriverControl` overloads (`teleop/teleop_loop.hpp` ~:256-410) then have no user in the tree, and
  `src/common/driver_controls.hpp`'s `HoldControl` / `ToggleControl` / `DriverBindings` are replaced.
- **(a) RECOMMENDED:** the library keeps T3's two binding types and their overloads, documented "superseded by
  the mechanism service; for a program with no service", and the loop refuses a program that hands it both
  paths at once (one driver path per mechanism — a precondition, probed). `src/common/driver_controls.hpp` is
  retired in MECH-A5 (program glue, not library API). Removing the library types later is one small chunk.
- (b) Delete them in MECH-A4. **Cost:** a documented API disappears (guide 10, cookbook 06, `docs/api/`
  `mechanism_binding.md`), and whether removing an unfrozen documented type needs a MAJOR bump is itself a ruling
  (`include/shulib/version.hpp` ~:10-41 is written for frozen members; TANK1's brief named the same gap).

**N5 — The Bench Tests menu is full: where station 13 MECHANISMS goes. RECOMMENDED (a): a 3 × 5 grid.**
- Bench Tests has twelve stations on a 3 × 4 grid (`src/programs/bench_r3a.hpp` :145, `kMenu{12, 3, 6, 38, 153,
  46, 4}`), and the usable height is 240 px, not 272 (`bench_r3a.cpp` ~:207-212, the file's own lesson).
- **(a) RECOMMENDED:** five rows of 36 px (38 + 5 × 40 − 4 = 234 ≤ 240), 153 px wide — room for 15, so the rest's
  pages fit later. Host-tested pixel for pixel like today's grid (`test/bench_menu_grid_test.cpp`).
- (b) A second menu page behind a MORE button (the twelfth slot becomes MORE; one extra touch for station 12).
- (c) Replace a station. **Cost:** every station is somebody's worksheet step.

**N6 — A driver file holds ONLY the team's decisions; how it runs is written once, in `src/common/`.
RECOMMENDED (a).** *(New since 09-29: his words of 2026-10-04.)*
- **Today** (`src/team/robot_two_driver.hpp` as T4 Part 3 leaves it, 142 lines): the button map in words (:1-39),
  then 18 lines that are decisions (the controls :68-76, the feel and the loop :78-88) and ~65 lines that are not
  — a class, `bind()`, `run()`, `bindings()`, `unbind()`, three private members (:59-123) and six
  `static_assert`s (:125-140). A student copying it for a second robot copies all of it.
- **(a) RECOMMENDED:** MECH-A5 moves the HOW-IT-RUNS half into `src/common/driver_file.hpp` — one class template,
  `DriverFile<kControls, kFeel, kLoop>`, written once (it binds the inputs, runs the library loop, holds the
  controller, the feel and the inputs, and refuses a control on the re-zero button at compile time) — so robot
  two's driver file is its button map in words plus **12 lines, 9 of them code** (re-check U6/U12: no
  `.tickPeriod`, no `using Button`, no library namespace): the controls (4), the feel and the loop (5), two headings
  and a blank (§1.1; P10 compiles it in static storage; the programs' spelling `RobotTwoDriver driver{};` is kept by
  one `using` line after the region). The six `static_assert`s' checks are all still made at build: the controls'
  by `controls(...)`, the re-zero button and the brain's tick by `DriverFile`'s two `static_assert`s (n56, n57),
  the claw row's widths in `driver_panel.hpp` (§1.1). It changes the final form of a file he approved (T4 brief
  §2) and the three listings cookbook 06 quotes; both move in MECH-A5's change.
- (b) Keep the class in the driver file, with the core's controls inside it. **Cost:** ~65 lines a team must
  copy and never change, in the file they are told to edit.

**N7 — The "reached" tolerance and the smooth move's speed are STARTING VALUES, like the gains. RECOMMENDED (a).**
- P6 says "no threshold is invented as a default", and also "gains ship with starting values and are MEASURED on
  the robot later". Three kinds of number sit between those two: the controllers' gains, the smooth move's top
  speed and acceleration, and "reached" (within 1.5° — 0.25 in for a lift — for 0.1 s).
- **(a) RECOMMENDED:** all three are the library's STARTING values, derived from the stated cartridge and
  gearing (75 % of the free speed; reach it in 0.25 s; kV = 12 V over the free speed), every one PROVISIONAL
  under HA-182, every one overridable by `.tuning = {...}` in the declaration, and printed in the boot line so
  nobody mistakes them for measurements. P2a: with no tuning at all, 143 of 144 mismatched moves arrive. Every
  number the team CAN know (cartridge, gearing, limits, presets, the gravity hold, homing, stall signatures, the
  at-speed band, a valve's move time) stays REQUIRED and refused when missing (`neg5/` n14, n25, n36, n46, n47, …;
  re-check N10).
- (b) The tolerance and the profile are required fields. **Cost:** three more lines in every arm and lift,
  holding numbers the team cannot know before it tunes — the case P6's "starting values" exist for.

**N8 — A piece sensor DECLARED but MISSING this boot: its steps run their time and are counted done, UNCONFIRMED.
RECOMMENDED (a).** *(Found by P13; Q2 (c) applied to a sensor lost at boot.)*
- **What the probe found (P13, `probe4_missing_sensor.cpp`).** A missing piece sensor is "it runs WITHOUT it"
  (P4), and the overlay then reads it as "sees nothing" — so `r.run(intake, -6_volt, stopsSeeingPiece, 1_s)`
  (eject, until the piece is gone) ends **SUCCEEDED "stopped seeing a piece" at the first tick, the motor never
  run**: a false success. `seesPiece` runs its whole bound and ends TIMED_OUT, which stops the auton. The false
  success is a DEFECT and is fixed either way: a condition on a missing sensor is UNKNOWN — never satisfied in
  either direction.
- **(a) RECOMMENDED:** the step runs to its bound (the motor really runs), then ends **UNCONFIRMED, counted done**,
  its line naming the missing sensor (`'intake': UNCONFIRMED -- its piece sensor is missing this boot (port 9);
  ran 1.00 s`) — exactly his Q2 (c) rule for a grab with no sensor ("counted done after a set time and reported
  Unconfirmed"): a cable that fell out costs the step's time, not the auton.
- (b) UNCONFIRMED, NOT counted: the chain stops after the bound. **Cost:** a loose sensor cable ends the auton at
  its first sensor step.

**Carried, not new — for his go (requirements §5 "not asked, carried to the brief as recommended"):**
**Q26 — the freeze triggers.** Recommended: "MECH on hardware" becomes the trigger for F11 (the mechanism seam),
F12 (the guard — the service is its claimant, and N1 changes it), F13 (driver input) and F14 (digital input), and
F14's open "switch or stall" question is recorded answered "both" (the core builds both, P2e). **Q28's drafts**
(the deviations row, open dependency #6, F4's generic items) are ready for the plan commit in §9.

**Decided here, overrulable (each with its reasoning in §1; the re-check's seven are in the amendments' list above):**
- **The declaration IS the typed handle (Q18 (a)).** A team writes `inline constexpr Arm arm{...}` once; an auton
  names `arm`, a preset `arm["score"]`, a constant `arm.to(120)`. A removed mechanism is `'arm' was not declared
  in this scope` at the auton's line; a misspelt preset, a constant past the limits, a speed past the free speed,
  a pneumatic position the claw does not have, and a step of the wrong kind each FAIL THE BUILD (66 negative
  compiles, P5 — most name their reason, three stay plain C++ errors, said in §1.2). No second "handles" file
  exists to drift: G3a's generator writes these same declarations. (The 09-29 draft's `mech::arm` handles are
  withdrawn.)
- **A robot's mechanisms live in its own namespace** (`robot_two::claw`), so two robots' `claw`s never collide in
  the one table file every build compiles; an auton file says `using namespace robot_two;` once — the team's
  statement of which robot it drives.
- **The `src` gate keeps every robot's mechanism facts in that robot's files** (re-check T3). Step 0 derives each
  robot's NAMESPACE from the mechanism table file and refuses `robot_two::…`, `using namespace robot_two` and
  re-opening it outside the table files, `test/`, a `static_assert` and robot two's OWN team files
  (`src/team/robot_two_*`, or the reviewed `TEAM_FILE_ROBOT` map — today `src/team/autons.{hpp,cpp}`); a robot's own
  team file may name its own robot's list (a driver file checks its controls against its robot — its job) and no
  other's; a chassis table in `src/team/` and any table in `src/programs/`, `src/common/` or `include/` stay
  refused (P11, `out5/gate_cases.txt`: 9/9). The team lead's ruling it serves — "no program may name another
  robot's table" — is unchanged and now covers the declarations too.
- **Gains are per OUTPUT-SHAFT degree** for every position mechanism, a lift's included (its spool's degrees).
- **A TOGGLE works a two-position pneumatic or a flywheel's NAMED speed** (`toggle(flywheel["shoot"], …)` — on at
  "shoot" / off) — the rule G3a's Flywheel template waits on.
- **A CONSTANT target is checked at build** (`arm.to(95)`, `arm["score"]`); **a COMPUTED one is clamped at run
  time and logged** (`arm.clampedTo(x)`), the runtime soft stop.
- **`RobotConfig::valid()` owns every port collision at run time**, sensor roles included (it sees the
  calibration record's drive); the robot file's build check mirrors it for the typed drive.
- **A missing ANGLE sensor or actuator makes the mechanism ABSENT; a missing PIECE sensor leaves it running
  without one** (its sensor steps then behave as N8 says, and say why).
- **The service sits BETWEEN the guard and the program's watch** in each auton program's pacer chain
  (Chassis → RunGuard → MechanismService → watch → tick pacer): the guard checks first, then the service ticks,
  then the world — the requirements' order — and the boot's chain proof also proves the service ticks (§1.13).
- **Route ids are derived, never typed** (his 2026-10-08 ruling; §1.16): one rule per kind, checked at the robot's
  list; N13 holds what the ruling left open.

---

## The design in plain words (for the team lead)

A mechanism is written ONCE, in the robot file, as **what it is**: its kind (an `Arm`, a `Lift`, a `Flywheel`,
a `Power` roller, a `Pneumatic`), which way each motor is mounted, its cartridge, its gearing as tooth counts
(`gears(12, 60)`), what it does when stopped and what a jam looks like, and the numbers only the team knows — its
limits, its presets, the volts that hold it level, how it finds zero. **Where it is plugged in** is a separate
block of one-line `wire(...)` entries — `wire(arm, motors(1))`, `wire(claw, lines('A'))` — so re-cabling changes
only that block. A port number is a number and a 3-wire port is a letter in quotes, so the two cannot be confused.
The build refuses a port used twice (naming it: `smart_port_4_is_used_twice`), a mechanism on a drive port, a
preset past a limit, a missing number, a misspelling, a homing push that points away from home, a jam signature
that could never trip — 66 mistakes, each with its reason in words. GCC names your line: on the error itself for a
few, in its first "in 'constexpr' expansion of" note for the rest; a mistake between two entries (a port twice)
points at the list's first line and names the port (§1.2 says exactly where each lands).

**One mechanism service runs every mechanism, every tick** — inside a drive in an auton, and every tick of
driver control. So an arm HOLDS 95° while the robot drives 40 in (worst error 1.16° on the host test plant), an
intake started before a drive stops the moment its sensor sees a piece mid-drive, and a flywheel spins up while
the robot drives to the goal. Nothing it runs can sit at stall current: an arm, a lift and a flywheel must state
what a jam looks like, and a jam ends the behaviour, said. A sensor that stops answering is never read as an
answer. At the end of an auton the guard makes every mechanism safe (and, with N1, at the deadline too); driver
control takes back whatever an auton left running, and an auton takes back whatever driver control left.

**Autons** say one step per line, each with its time limit: `r.move(arm["score"], 1.5_s)`, `r.run(intake,
12_volt, seesPiece, 3_s)`, `r.set(claw["closed"])`, `r.spinUp(flywheel["shoot"], 2_s)`, `r.home(lift, 1.5_s)`.
A `start...` form starts a step and goes on (the robot drives while it runs) and `waitUntilDone(...)` collects it.
Every step says how it ended (SUCCEEDED, TIMED_OUT, STALLED, NOT_HOMED, ABSENT, REFUSED, CANCELLED, or
UNCONFIRMED for robot two's sensorless claw grab — counted done, as you ruled). **Every mechanism's actions are
also ROUTE IDS, automatically** (your ruling of 2026-10-08): `arm_score`, `intake_until_piece`, `claw_closed`
appear in the manifest a route editor reads, and a route marker runs exactly the step an auton line runs. The
**driver file** adds PRESETS beside hold-to-run and toggle (`preset(arm["score"], Button::Right, 1.5_s)`); the most
recent input wins. With N6 the driver file is only those lines and the feel.

**At every boot** each role gets one log line — `arm: motor 1 -> port 1 ok`, or `wrist: angle sensor -> port 10
ABSENT: port 10: a distance sensor (expected a rotation sensor)` (that mechanism's buttons and steps then do
nothing; the drive runs) — and every 3-wire role says "not checkable". Two cables of the SAME kind swapped
between mechanisms look fine at boot; that is what the new **MECHANISMS station in Bench Tests** is for: jog
each mechanism under a dead-man button, watch its reading in its own units (degrees, inches), fire each valve on
your confirm, and the **direction check** ("did ONLY 'arm' move, and which way?") names the exact robot-file line
to change — the declaration's for a direction, the `wire(...)` line for a cable on the wrong mechanism. It cannot
save a flip on the brain — you edit the file and rebuild (your Q1 (a)).

Robot two keeps working exactly as it does: its claws are re-written in the new words (13 lines of code and 10 of
its caveats where there are 43 and 35 today) and the result is checked **field by field** against today's
configuration. Nothing has run on a robot; the arm, lift and flywheel numbers come from new host test plant models
whose constants are invented until a robot session measures them.

---

## 0. Why — the requirements, re-read, and what exists (every line cited was opened in `tree_t43d`)

### 0.1 The requirements this brief builds (`docs/internal/MECH-PANEL-requirements.md`, main `2fce38a`)
- §0 his words (:17-41); §1 the principles P1–P8 (:43-86) — P2 port-agnostic, P4 "every action is bounded and
  says how it ended ... nothing sits at stall current beyond its stated cap", P5 safety carries over, P6 "no
  threshold is invented as a default", P7 PTO later, P8 the robot file is the `.vexbot` profile.
- §2 what exists (:88-112); §3.0 the service (:116-155) — "a pacer decorator composed with RunGuard, in this
  order: the guard checks its deadlines first ..., then the service ticks, then the world steps"; in driver
  control "after the bindings read the buttons, before perTick"; "a claimant"; "exactly once per tick, even
  where both seams run in one loop".
- §3.1 the kinds (:157-215; the cycle kind is the rest's); §3.2 done-conditions (:219-220; counting, sorting,
  jam, interlocks, caps, tip are the rest's); §3.3 presets and arbitration (:263-276); §3.4 autons (:301-341;
  mid-drive triggers are the rest's); §3.5 port binding (:343-363); §3.6 the bench page and the direction check
  (:367-371), the plant models (:391-393); §3.7 PTO (:395-407); §3.8 what stays out (:409-421); §3.9 Q1's
  consequences (:423-431).
- §4 item 7b the panel's routes and **the mechanism route ids, "built in the MECH core"** (:497-517, ruled
  2026-10-08; §1.16, N13); §5's answers (:537-570, quoted in the header); §6 surfaces touched (:728-749); §7.1 sizes
  (:752-772); §7.2 placement (:773-794); §7.3 the F3/F4 drafts (:795-805).
- `docs/internal/REMAINING-WORK.md` §3 (:49-78): row 12 the core (:63, the route ids added 2026-10-08), the rest rows
  13–15b; row 18b CAL1 (N9); row 20b G3a Part R (the routes, which read §1.16's ids).

### 0.2 What exists, and what each piece gives the core (as T4 Part 3 leaves it)
| Piece | Where (opened) | What the core uses / changes |
|---|---|---|
| The mechanism seam (F11, open) | `hal/mechanism.hpp`: `ICancellable` :101; `IMechanism` :125 (`applySafeState` :153, `tryClaim(claimant)` :178); `MotorMechanism` :209 (`motors()` :282); `PneumaticMechanism` :295 (`set(bool)` fans ONE level) | the service's slots are each mechanism's `ICancellable`; ADDS `MotorMechanism::meanPosition()` and a per-line `PneumaticMechanism` form (`setLevels`, a per-line safe level) — three positions need different levels per line |
| Bounded operations (F1) | `manipulation/mechanism_op.hpp` (the blocking idiom; `RunUntilConfirmed`; `ActuateAndConfirm`) | untouched; they keep working beside the service (one ticker per claim) |
| Outcomes (append-only) | `manipulation/mechanism_outcome.hpp` :33, :50-61 | APPENDS `Absent = 6`, `NotHomed = 7`, `Refused = 8` |
| Stall detection | `manipulation/stall_detector.hpp` (`StallConfig` :37) | a blocked move, a stall-home, `stalls` |
| PID / profile | `control/pid.hpp` (`PidConfig` :31, `integralLimit` :45, `update` :78); `control/trapezoid_profile.hpp` | the move loop (P + D on the profile), the hold loop (P + I + D), the smooth move |
| The guard (F12, open) | `sequence/run_guard.hpp`: frozen waits ~:92-107; "FREEZES NOTHING" ~:163; `RunGuardReport` :241; `pace()` check-then-step :312; `run()` :473; `cutActiveMotion` :653; `safeMechanisms` :701 | N1: the latch also cuts claimed mechanisms; `mechanismCuts` appended |
| The Routine (F10, LOCKED, additive-open) | `chassis/routine.hpp`: "ADDITIVE growth — new steps ..." :20; `RoutineStopCause::MechanismFailed` :172; `RoutineResult` :183; the frozen constructor :209; `recordStop` :564 | a second constructor (from the auton's context), the mechanism steps, `lastMechanism()`, `RoutineResult::mechanismVerdict` appended; `RoutineStopCause::MechanismStepFailed` APPENDED (T4) |
| Autons (COMP-a, not frozen) | `competition/auton_registry.hpp` `AutonContext` :212-226 ("For the guard's safing only (B25): a routine that commands a mechanism looks it up by name from the Robot" :221); `competition/match_runner.hpp` `runAuton` :140 (ctx :180) | `AutonContext::mechanismService` APPENDED with a default; a `runAuton` overload taking the service |
| The driver loop | `teleop/teleop_loop.hpp`: the tick order :18-59 (perTick 7, pace 8); the bindings overload :256-385; the forwarding overloads :388-410; T4 Part 2a's protections inside it | ONE loop body; a service overload that ticks the service after the inputs, before perTick |
| The Robot | `robot/robot.hpp`: MECHANISMS banner :90-120; `BuildOptions` :187 (`pacer` :207); `mechanisms()` :735; `MechanismSlot` :1189; `buildMechanisms` :1212; `buildPneumatic` :1249 | builds the SENSOR roles too, through a library piece the bench station reuses (§1.5); per-role boot facts |
| The config | `config/robot_config.hpp`: `kMaxMechanisms` :60 (8), per-mechanism caps :63, :66; `MechanismConfig` :443; `PortUse` :493; `valid()` :578, the one-owner `claim` :585 | APPENDS the sensor roles' ports and lines and per-line levels; `valid()` claims them (`PortUse::MechanismSensor` appended) |
| Devices by port | `hal/device_provider.hpp`: the 3-wire exception :45-47, `IDeviceProvider` :100, `rotation` :141, `digitalOut` :154; `hal/pros/device_provider.hpp` (`ProsDeviceProvider(registry)` :181, `rotation` :357, `digitalOut` :387, `expectKind` :496); `hal/fake/fake_device_provider.hpp` :88; two test-local providers in `test/robot_builder_test.cpp` ~:1797, ~:2175 | THREE new accessors — `distance`, `optical`, `digitalIn` — and the read-fault pair `readFaults` / `lineReadFaults`, answered by all four |
| Sensor adapters | `hal/pros/{distance,optical,digital_in,rotation}.hpp` (`ProsDigitalIn` screens PROS_ERR to the last good level, reads a LEVEL, never `get_new_press`) | made by the new accessors |
| The drive's boot check | `calibration/boot_check.hpp` `DeclaredMotorPorts` :185 | fed from the wiring (`declaredMotorPorts()`) |
| Robot two's mechanisms | `src/robots/mechanism_table.hpp` (525 lines after T4 Part 3): `kClawOpenCommand` :159-171; entry types :173-228; `kRobotTwoMechanisms` :230-293; `kNoMechanisms` :296; `mechanismTableFor()` :304-314; `kMechanisms` :319; checks :321-402; polarity :404-434; Calibrate's drop :436-489; pins :491-523 ("only robot two declares mechanisms" :522-523) | MECH-A5 rewrites it into declarations + wiring (§1.1, §1.9); the buttons already left it (T4 Part 3) |
| Table → config | `src/common/boot_source.hpp` `mechanismTableProblem` :129, `mechanismConfigIndexOfPneumatic` :158, `toMechanismConfig` ×2 :201-217, `bootConfigFrom` :290-316 | replaced by the library's ONE projection `mechanism::toMechanismConfig(spec, wiring)` — proved IDENTICAL for robot two (P9a, P10) |
| The driver file | `src/team/robot_two_driver.hpp` (142 lines; controls :68-76, feel/loop :78-88, how it runs :90-123, six checks :125-140); `src/common/driver_controls.hpp` (533 lines: `HoldControl`/`ToggleControl`, the predicates, `DriverBindings`, `driverHeading()` :268) | N6: the decisions stay, the rest moves to `src/common/driver_file.hpp`, `driverHeading()` to `src/common/driver_heading.hpp`; `driver_controls.hpp` retires (every user re-pointed, T6) |
| The programs | `src/programs/drive_program.cpp` (claw boot lines :583-592, claw row :378, run :695); `competition_program.cpp` (the chain :640-644, the proof :672-680, claw boot lines :752-758, claw row :542); `auton_tester_program.cpp` (the chain :468-473, the proof :498); `src/common/guarded_run.hpp` (`pacerChainProven` :72-79); `src/common/driver_panel.hpp` (claw boot lines :152-190, claw row :238-320) | the service in each auton chain and the proof; one boot line per role; the claw row read from the service |
| Calibrate's drop | `src/programs/calibrate_program.cpp` :1119-1134 (the `claw pivot (declared)` line); its push's own "moved" threshold (`c.movedThreshold`, :752-755) | **CAL1's now (N9)**: CAL1 deletes the drop and builds the WATCH (its D4); A5 only maps the new list into CAL1's facts |
| Bench Tests | `src/programs/bench_r3a.hpp` `kMenu` :145; `bench_r3a.cpp` usable height :207-212, station 10's six safety gates :1317-1360 | station 13 MECHANISMS (N5), with station 10's gates as its model |
| The `src` gate | `tools/src_build_gate.py`: the per-robot names derived from `TABLE_FILES` :449-457, `per_robot_table_names` :577, the refusal outside tables/test/static_assert :734-758, self-test case 28 ~:2324-2363 | step 0 per ROBOT: each robot's namespace and list stay in that robot's files (T3, P11) |
| Route commands (F8, LOCKED) | `runner/command_registry.hpp` (`CommandRegistry::on` :129, `kMaxCommands` 32 :69, `CommandEntry` :95); `runner/manifest.hpp` (`writeManifest` :90; a registered id's doc printed as `""`); `runner/path_runner.hpp` (markers fire after a leg settles; a non-Succeeded handler stops the route); `competition/match_runner.hpp` (a route entry needs a registry, :156-158) | §1.16: the mechanism route ids, registered by one call; `CommandEntry::doc` and a doc'd `on()` overload APPENDED (F8-additive) |

### 0.3 The probes — every one re-run against `tree_t43d` + `proposed4/` in the amended words (outputs in `out5/`)
| # | What | Result |
|---|---|---|
| **P1** | the service under the REAL guard in the REAL Chassis's pacer chain (guard → service → world), an auton through the REAL `competition::runAuton` (its service overload), the host test plant's 12 in tank (`probe3_auton.cpp`) | **The arm HOLDS 95° through a 1.80 s `driveTo`: worst error 1.16°.** 180 paces = **180 service ticks** (exactly one per pace). The intake, STARTED before the drive, ran at 12 V during it and went to 0 V at T+2.00, when its sensor saw a piece — **the drive still running**; the lift (started before the drive) was at 9.70 in when the drive ended and SUCCEEDED at 18.02 in after 3.42 s; the flywheel at speed in 0.63 s; the hood in its 0.31 s move time; 13/13 steps; after the run every claim released, the arm at 0 V in Hold, the hood at its safe position. **The students' auton as written** (`team5/autons_x.cpp` `scoreThree`, with `seesPiece.orAfter(2.5_s)`): 10/10. **Control — the service NOT in the chain:** the chain stops at step 4, "the mechanism service never ticked -- it is not in the chassis pacer's chain" |
| **P2a** | position moves with the library's STARTING values (N7) on the new ARM model; controller kG 1.6 against plant kG 1.2/1.6/2.4, kV ×0.8/1/1.25, kS 0.3/0.6, Red/Green × 5:1/7:1; 0→90 then 90→150 (`probe2_position.cpp`) | **143/144 SUCCEEDED** in 0.69–3.15 s; overshoot ≤ 7.22° (≤ 3.59° with a plant no faster than its cartridge — a real motor cannot beat it); final error ≤ 2.55°. The one miss: a plant slower than its own cartridge (N2) |
| **P2b** | holding at 90° under a +0.8 V piece load | worst 1.66–2.41°, within 1.25° after 2 s, still holding. **Nobody ticking** (Hold at 0 V only, the plant's invented stiffness, HA-183): 3.05° of sag in 2 s — why the hold must be ticked |
| **P2c** | a BLOCKED arm (obstacle at 60°, moving to 120°) | **the robot file's signature (2.0 A below 5 rpm for 0.5 s): STALLED at 1.00 s** at 59.9°, MECHANISM_STALLED raised once, 0 V, claim released (1.6 A / 3 rpm / 0.15 s: 0.65 s). A hand-built signature that CANNOT trip (2.6 A — now refused at build, n58): TIMED_OUT at 3.01 s, then N2's hold-where-stopped (mean 1.68 V, 0.37 A, held at 59.7°) |
| **P2d** | the LIMITS | a target AT the limit (160°): SUCCEEDED (160.36°, overshoot 2.79°); a COMPUTED 175° → clamped to 160°, SUCCEEDED, the soft stop logged once; a +8 V jog AT the limit for 1.5 s peaks at 160.24° (hard stop 170°) and is held AT the limit by the closed-loop hold (D2); never homed with no `unhomedVolts`: a jog commands 0 V and a move is NOT_HOMED; with `unhomedVolts` 4: the jog is capped at 4 V |
| **P2e** | EVERY HOMING MOMENT | before homing a move is NOT_HOMED; stall-home from an unknown 30°: 0.51 s, zero at the stop; switch-home from 40°: 0.48 s; a switch already pressed at boot: homed at bind, 0 V, no motion; rests at its stop: homed at bind. At AUTON START (`r.home(lift, 1.5_s)`, P1): costs its time, logged. At DRIVER START and ON REQUEST: P2i. Every (re-)home logs where zero went (D10) |
| **P2f** | a two-motor LIFT in inches (Red direct, 1.5 in spool; plant kG 0.8–1.4 against controller 1.0, kV ×0.8/1.25) | 18 in SUCCEEDED in 3.42–3.58 s (its starting cruise is 5.89 in/s), within 0.05 in; under +0.5 V worst 0.091 in, ≤ 0.01 in after 2 s |
| **P2g** | the NON-LINEAR shapes, declared as a team would | a DR4B `linkage(14_in, 10_in, 2)` round-trips height → angle → height within 3.55e-15 in over −10..34 in; a 4-pair `measured(...)` table within 1.78e-15 in (10 in → 50.00°); a non-increasing table is refused at build (n37); a target AT a table's top end SUCCEEDS (D4, case G: 1.91 s) |
| **P2h** | an ABSOLUTE rotation sensor (the arm at 40° at power-up) | homed at bind, reads 40.00, then to 90° SUCCEEDED (89.95°); mounted reversed and declared `rotationSensor(Reversed)`: the same. **Declared Forward but mounted reversed: reads −40°, drives the arm into its hard stop — now STALLED at 1.31 s** (the required signature) instead of TIMED_OUT at 3.01 s; the direction check's "sensor against its motors" is the other net; the boot cannot see it |
| **P2i** | homing from the driver file, through the REAL `runDriverControl` (`probe3_driver.cpp`) | `homeAtDriverStart(arm, …)`: homes once when the auton did not; skipped when it did. `homeOnPress(arm, Button::Left, …)`: SUCCEEDED |
| **P3** | a FLYWHEEL (Blue `gears(72, 12)`, free 3600 rpm) to "shoot" = 3000 rpm in 2 s; plant loss kV ×1.0/1.1/1.2 × friction kS 0.2/0.4/0.8 × spin-up τ 0.1/0.3/0.5 s (`probe2_speed.cpp`) | **feedforward + PI, the integral acting only within 10 % of the target: 20/21 of the cases the plant can reach are at speed** (0.31–1.87 s, overshoot ≤ 49.7 rpm, a 20 % shot recovered in ≤ 1.35 s); the slowest reachable one and the 6 unreachable ones TIMED_OUT honestly and keep spinning (an unreachable wheel spins; a JAMMED one STALLS — N10, case H) |
| **P4** | the BOOT binding check through the REAL `ProsDeviceProvider` (its three new accessors) over a faked census; then `valid()` with a calibration record (`probe3_wiring.cpp`) | clean: one line per role, `ok`; every 3-wire role "not checkable". **Two same-kind cables swapped: invisible** (the bench station's job). A rotation and a distance sensor swapped: the wrist ABSENT (`port 10: a distance sensor (expected a rotation sensor)`), the intake runs WITHOUT its piece sensor. A MOTOR where a piece sensor is bound: runs without. Pivot 7 unplugged: ABSENT, "nothing plugged in (expected a motor)". An actuator port holding an optical sensor: ABSENT, its own sensor "not asked". **`valid()`**: a record that measured the intake's motor port as a LEFT drive port → REFUSED "port 5 is claimed by the LEFT drive and by a mechanism motor"; a sensor's port → "... and by a mechanism sensor"; a limit switch on the hood's line → REFUSED naming both mechanisms |
| **P5** | the build checks (`neg5/`, `run5.sh`) against the real headers | **66 negative compiles, each REFUSED, each matched by its reason's text** (`-F`), and 1 positive (every step form at the exact limits). Where GCC puts it: 12 ON the team's line; 54 in a library header with the team's line in the first note; a cross-entry refusal at the list's first line, naming the port or button (m1, m2); a declaration's at its `wire(...)` line (m3). Three stay plain C++ errors, said: `90_deg` (names `_deg`), a run-time value where a constant is checked, a misspelt mechanism. The list (`out5/neg5_summary.txt`): mechanisms (misspelt or removed; a misspelt preset or position; past the limits or the free speed; a heading for an arm angle; a number for a lift height; `run()` on an arm, `move()` / `set()` / a preset on a roller, `stop()` on an arm; volts without a unit); wiring (a port or line twice, NAMED; a number for a letter and the reverse; the wrong motor count; a sensor unplugged; port 22; a mechanism twice; two names alike; motors on a pneumatic, lines on an arm; a drive port; levels vs lines, both ways); declarations (no cartridge, safe state, at-speed band, stall signature on an arm or a flywheel; a signature that cannot trip, two ways; a homing push away from home, two ways; a preset past the limits; a start naming no position; a name past 23; a non-increasing table; a bare-number gearing); controls (a button twice, NAMED; no control and not `autonOnly`; another robot's mechanism; 13 V; two holds; a preset with no time limit; `whileHeld` on a pneumatic; a flywheel toggle with no speed; a switch-homed mechanism with no driver-side homing; a never-homed one with no `unhomedVolts`; a preset on a never-homed one; a stated tick that is not the brain's; the re-zero button); route ids (too long, two alike, a built-in, more than 32); `seesPiece(arm)` on a mechanism with no piece sensor |
| **P5b** | EVERY optional word (§1.1) in one robot — an absolute-sensor wrist with a reversed rotation sensor, `unhomedVolts` and `.tuning`; a stall-homed arm; a DR4B `linkage`; a `measured` tilt that is `neverHomed`; an optical roller; a limit-switch catapult; a pneumatic grip with `grabs` / `releases` and a distance sensor — wired, with 10 driver controls (presets on a pneumatic, `homeOnPress`, `homeAtDriverStart`, `autonOnly`) and an auton using every step form (`probe4_forms.cpp`) | compiles; every derived config is as meant (the reversed lift motor −4; the switch on 'E' line 5; the optical on 9; the distance on 8); `.tuning` replaces exactly what it names (kP 0.30, within 2.00) and the rest keep the starting values (kP 0.25, within 1.50) |
| **P5d** | robot x's driver file through the REAL `runDriverControl` (`probe3_driver.cpp`) | a press ON the entry tick is ignored (a press must be SEEN); preset `score` → 95°; a hold pressed mid-move wins, and its release HOLDS HERE (41.80°); a new preset while the hold is STILL held wins, and the held button commands nothing until released; the intake runs while held and stops (Coast) on release; the flywheel toggles to 2996 rpm "at speed" and stops. **The disable: motors safe, the hood KEEPS its level, every claim released**; no hold target ever outside the range (D2's clamp) |
| **P6** | a mechanism step when the deadline or a disable arrives (`probe3_deadline.cpp`, built with and without the cut) | N1's table (unchanged) |
| **P7** | the DIRECTION CHECK's judgement (`probe3_direction.cpp`) | a fight: "'lift' motor 2 turns AGAINST its command (its mate drags it): in the declaration at team5/robot_x.hpp:28, .motors: make motor 2 Forward, then rebuild and upload"; the GENERATED form names the robot file's JSON path; both motors still and drawing: "they FIGHT … or it is JAMMED"; a motor that neither moved nor drew: its cable and its `wire(...)` line (:76); the operator says the OTHER way: swap every Forward and Reversed at the declaration's line; **"it did not move / something else moved": "a cable of 'arm' is on ANOTHER mechanism's port … check the wire(...) line at team5/robot_x.hpp:75"**; confirmed; a pneumatic whose levels are swapped; a pneumatic where nothing moved: its line, tubing, air |
| **P8** | EXACTLY ONCE per driver tick, with the loop handed the raw pacer, the service, or `guard{service}` | 600 iterations → 600 service ticks → 600 world paces, all three |
| **P9** | **robot two's claws, the proving case** (`probe3_robot_two.cpp`, reading the tree's REAL `src/`) | the derived config is **IDENTICAL field by field** to today's `boot_source` conversion (+6 Red Hold; +7 Red Hold; line A, start and safe Low); declared motor ports 6 7. A claws auton: pivots by time SUCCEEDED; **the grab UNCONFIRMED, counted done (no sensor)**; with a distance sensor mounted, **SUCCEEDED "sees a piece"** — Q2 (c) both ways. Robot two's controls through `runDriverControl`: the claw toggles; the pivot at 12 V while held; the claw keeps its level through the disable, entry after entry |
| **P10** | robot two in the spelling MECH-A5 writes (`team5/mechanism_table_core.hpp`: its own namespace, the list `kRobotTwoMechanisms`, the library's `MechanismTable`), the slim driver file (N6) in static storage, an auton with `using namespace robot_two;` — host (bench selection) AND `-DSHULIB_ROBOT_TANK_2026` (`probe4_robot_two.cpp`) | compiles both ways with every pin; `kMechanisms` is `kNoMechanisms` on the host and robot two's under the tank define; the derived config again identical; each declaration records its own `.name` line (the direction check's "change it here") |
| **P11** | the REAL `src` gate's step 0 over planted files (`gate5/`, `tools5/gate_cases.py`) | today's rule: 5/9 right — a program naming `robot_two::claw`, a program's `using namespace robot_two;` and `src/common` re-opening robot two's namespace all PASS, and robot two's own second team file is refused; **the proposed per-robot rule: 9/9** (§1.13) |
| **P12** | a declaration written WITHOUT `inline`, used from two translation units (`diag/tu/`) | the service, bound in one TU, finds the mechanism named in the other BY NAME: yes (an address lookup would have ended that step ABSENT, silently) |
| **P13** | a power mechanism whose DECLARED piece sensor is MISSING this boot, a piece in front of it (`probe4_missing_sensor.cpp`) | present: `seesPiece` SUCCEEDED at once, `stopsSeeingPiece` TIMED_OUT after 1.00 s (the piece stayed); **missing: both run their 1.00 s with the motor running and end UNCONFIRMED, counted done (N8)** — the 10-06 overlay's false success ("stopped seeing" at the first tick) is gone |
| **R** | piece sensors whose reads FAIL, through the REAL `ProsDistance` / `ProsOptical` / `ProsDigitalIn` made by the REAL `ProsDeviceProvider` over the PROS shim, each case with and without the read-fault channel (`adv_readfaults.cpp`) | without it: a distance sensor that failed while the intake was empty (R2), one dead from boot (R3) and an optical that failed while empty (R8) each SUCCEEDED "stopped seeing a piece" at 0.00 s, the motor never run — false successes; **with it: each runs its 1.00 s and ends UNCONFIRMED, not counted**; a working sensor (R1), `orAfter` (R6) and a 0.2 s drop-out (R7: SUCCEEDED on the fresh read) unchanged; a home switch dead from boot with pressed = low (S, HA-184 wrong) homed the arm at bind without it, not with it |
| **ST** | a blocked HOLD with a hand-built stall signature of 2.0 A vs 3.0 A (`adv_stallsig.cpp`) | 2.0 A: STALLED once, 0 V; **3.0 A: 29.9 s at ≥ 2.4 A, 12.00 V commanded, never STALLED** — why n58 refuses amps at or above the motor's 2.5 A limit |
| **P14** | **mechanism ROUTE IDS** (his ruling of 2026-10-08): robot x's and robot two's ids derived from their lists, registered by ONE call, published by the REAL `writeManifest`, fired by the REAL `PathRunner` inside the REAL `runAuton` under the guard (`probe5_routes.cpp`) | robot x 17 ids, robot two 6, each with its doc; a 7-marker route: ROUTE_RAN, every marker SUCCEEDED (the intake saw a piece at 0.97 s, the arm reached 'score' in 0.90 s, the flywheel at speed in 0.64 s), the end action succeeded; robot two's route: the sensorless grab UNCONFIRMED, counted done, the route went on; a marker with no time limit, and 13 V: each REFUSED with its line, the route stopped `MarkerFailed` |
| **S1** | the whole host suite by hand with CMake's flags (`build_suite5.sh`, `-O0`, loud on any failed TU) | pristine `tree_t43d`: **2231/2231** (2 783 067 assertions, 3 skipped); with the overlay (+ `proposed_test3/robot_builder_test.cpp`, + `proposed4_src/`): **2231/2231**, identical counts — the F8 signature pins and the byte-exact manifest golden among them |
| **S2** | the ARM header gate (RESUMING ~:107-112) on tree + overlay, and each overlay header alone | **220 headers PASS; 29/29 alone PASS** |
| **S3** | sizes (ARM, `arm-none-eabi-g++ -Os`) | (10-06, unchanged in kind) `MechanismSpec` 1 136 B; a robot's list 9 400 B (`constexpr` — flash); `MechanismService` 8 048 B (static storage); `DriverControls` 1 352 B (flash); `DriverInputs` 176 B; an `Arm` declaration 416 B (flash); a `PositionSlot` 536 B; **new: `runner::MechanismRoutes` 4 804 B (static storage); `CommandRegistry` 1 156 → 1 284 B** |

### 0.4 Found while probing (each handled in scope, or named as out)
1. **The guard cannot cut a mechanism step** → N1.
2. **A timed-out move would hold its unreached target at stall current** → N2.
3. **Gains are per OUTPUT-SHAFT degree** for every geometry, stated as a rule (a lift's per degree of its spool).
4. **A profile the plant cannot follow must TIME OUT honestly**, never report SUCCEEDED — a test (§3.3).
5. **An angle sensor declared against its mounting drives the arm into its hard stop** (P2h). The nets are the
   direction check and a stated stall signature; a running cross-check of the sensor against the motor encoders
   is protection — the rest's.
6. **A 3-wire port typed as a number is the smart port it looks like** (an early census probe read "claw line =
   6" as 3-wire F): `motors(...)` takes only numbers, `lines(...)`/`homeSwitch(...)` only quoted letters (n11,
   n26, n27).
7. **`AutonContext` needs only a forward declaration** of the service: `auton_registry.hpp`'s include graph does
   not grow.
8. **After an auton, driver entry drops a position HOLD to its safe state** (the ruled T3 entry rule; in a
   match the field's disable between the periods cuts the motors anyway, HA-145): stated in the guide, unchanged.
9. **The first prototype re-zeroed an absolute sensor wherever the arm stood**; fixed as a FIXED offset (the
   sensor reads `sensorReadsDeg` when the mechanism is at `at`), P2h.
10. **A declaration copied without `inline` has one address per TU** (P12): a step finds its mechanism by NAME
    and KIND (and, for a pneumatic, the position's name at its index), never by address. Two robots that both
    have a `claw` are therefore told apart only by which robot's build runs the auton — landmine 6.
11. **The `src` gate refuses a driver file naming its own robot's list** (P11) → the one-case exception (decided
    here).
12. **The stopped runs named robot two's list `kMechanisms` inside its namespace**: step 0 would then derive the
    BUILD's selection name as a per-robot name and refuse every program. The list keeps today's name,
    `kRobotTwoMechanisms` (P10, P11).
13. **The overlay's `mechanism_outcome.hpp` comment says REFUSED also covers "a target the build could not
    check"** — wrong: a computed target is CLAMPED (the soft stop), never refused. The executor writes REFUSED =
    "another owner holds the mechanism's claim" only.
14. **A declaration's recorded file name is the path the compiler opened**: `./src/robots/mechanism_table.hpp`
    on the brain (`common.mk` :150 turns the Makefile's root into `-iquote"./src"`), absolute on the host. Host
    tests pin the suffix `src/robots/mechanism_table.hpp:<line>`.
15. **Calibrate already has a "moved" threshold for its push** (`c.movedThreshold`): the declared-port watch
    reuses it — no new number.
16. **The suite has no compile-fail harness**, and 40 build refusals could not be mutation-tested without one →
    §3.0 adds `test/compile_refusals/` (one CTest per refusal, passing only when the compiler names the reason).
17. **`ProsRotation::position()` binds `get_position()` — CUMULATIVE** (`hal/pros/rotation.hpp` :7, :78-79; HA-11),
    not `get_angle()`. `absoluteSensor()` homing therefore rests on the belief that, at power-up, the cumulative
    position starts at the shaft's absolute angle within one turn; the vendored `pros/rotation.hpp` :194-216
    (`get_position()`; re-check T9 — :244-266 is `get_angle()`) says only "current position". That is HA-186,
    settled at station 13 (power-cycle at two marked angles, and a PROGRAM restart). P2h's plant
    publishes an absolute reading by construction, so the host proves the arithmetic only; `IRotation` is F4
    LOCKED, so binding `get_angle()` instead would be a sibling seam — not the core's unless HA-186 fails.
18. **A sensor condition on a MISSING piece sensor was a false success** (P13: `stopsSeeingPiece` SUCCEEDED at
    the first tick, the motor never run) → fixed (the condition is UNKNOWN, never satisfied) and N8 for the
    verdict.

19. **(re-check) A piece sensor whose reads fail holds its last value, and a held "far" is the 9999 "no object"
    reading byte for byte** (R) → the read-fault channel (`hal::IReadFaults`; the amendments' "Decided here").
20. **(re-check) A REQUIRED stall signature that can never trip is no signature** (amps at or above the motor's
    2.5 A limit; `belowMotorRpm` 0) → two build refusals (n58, n59; N10).
21. **(re-check) N2's hold-where-stopped and a driver's release could target past a limit** (the 10-06 driver
    probe held −1.30° on a 0..160 arm) → every hold target is clamped to the range (D2).
22. **(re-check) Main moved under the brief four times** (`fda5351`, `6517f2d`, `3c643f1`, `2fce38a`); one of them
    put the mechanism ROUTE IDS into the core → §1.16, N13.
23. **(re-check) `CommandRegistry` publishes no doc for a registered id** (`manifest.hpp` writes `""`), so a
    picker would show a mechanism id's args unlabelled → the F8-additive `CommandEntry::doc` (N13 (a)(3)).
---

## 1. Scope — IN (every shape below is VERBATIM from `proposed4/` or the probes, and compiles)

### 1.1 What a team writes — verbatim, compiled against the real headers, counted

The rule (his words of 2026-10-04): a team file carries the team's decisions and nothing else. Each block below is
quoted from a probe file that compiles against `tree_t43d` + `proposed4/` (P1, P5, P10, `team5/counted_steps.cpp`)
and runs (P1, P5d, P9, P14). A field the team does not write is UNSET and refused by the build when the kind needs
it (P5); the library's only own numbers are the starting values of N7, printed at boot. **The members are written
in one order per kind — C++ requires it** (N11): what it IS, what it does STOPPED, where it GOES, the extras.

**The robot file — what each mechanism IS** (`team5/robot_x.hpp`, an invented robot; one `using namespace
mechanism::words;` at the top of the robot's namespace gives every word below):
```cpp
// An ARM with three presets and gravity compensation (12 lines)
inline constexpr Arm arm{
    .name = "arm",
    .motors = {Forward},
    .cartridge = Green,
    .gearing = gears(12, 60),         // a 12-tooth gear on the motor drives a 60 on the arm
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 10, .lasting = 0.5_s},
    .range = {0, 160},                // degrees: down on its stop .. just past vertical
    .gravity = cosine(1.6_volt, 45),  // 1.6 V holds it out level; it is level at 45 degrees
    .home = restsAtStop(0),           // it rests on its stop at 0 when the program starts
    .presets = {{"stow", 0}, {"score", 95}, {"high", 140}},
};
// A two-motor LIFT in inches (13 lines)
inline constexpr Lift lift{
    .name = "lift",
    .motors = {Forward, Reversed},    // the right-hand motor is mounted mirrored
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = homeBySwitch(0_in, -3_volt),  // down at 3 V until its switch is pressed: that is 0 in
    .presets = {{"floor", 0_in}, {"low", 8_in}, {"high", 22_in}},
};
// A FLYWHEEL with "at speed" (10 lines)
inline constexpr Flywheel flywheel{
    .name = "flywheel",
    .motors = {Forward},
    .cartridge = Blue,
    .gearing = gears(72, 12),         // geared UP 6:1, so 3600 rpm free
    .safe = Coast,
    .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s},
    .atSpeed = within(60, 0.1_s),     // "at speed": within 60 rpm for 0.1 s
    .presets = {{"shoot", 3000}},
};
// An INTAKE that can run until its sensor sees a piece (7 lines)
inline constexpr Power intake{
    .name = "intake",
    .motors = {Forward},
    .cartridge = Blue,
    .pieceSensor = distanceSensor(2_in),  // it "sees a piece" closer than 2 in
    .safe = Coast,
};
// A THREE-POSITION pneumatic (7 lines)
inline constexpr Pneumatic hood{
    .name = "hood",
    .positions = {{"down", {Low, Low}}, {"middle", {High, Low}}, {"up", {High, High}}},
    .start = "down",
    .safe = "down",
    .moveTime = 0.3_s,
};
```
**The robot file — where each one is plugged in** (one line per mechanism; re-cabling changes only these):
```cpp
inline constexpr MechanismTable kRobotXMechanisms = mechanisms(
    wire(arm, motors(1)),
    wire(lift, motors(2, 3), homeSwitch('B')),
    wire(flywheel, motors(4)),
    wire(intake, motors(5), pieceSensor(9)),
    wire(hood, lines('C', 'D')));
```
*(Every robot names its list `k<Robot>Mechanisms` — `MechanismTable` is the library's one type name; never
`kMechanisms`, which is the BUILD's selection, landmine 13.)* The other words a declaration may use, each optional
unless its kind needs it: `.angleSensor = rotationSensor()` / `rotationSensor(Reversed)`; `.pieceSensor =
opticalSensor(0.5)` / `limitSwitch()`; a Power's optional `.stall = {.amps = 1.6, .belowMotorRpm = 3, .lasting =
0.15_s}` (REQUIRED on an Arm, a Lift and a Flywheel, N10); `.home = homeByStall(0, -3_volt, {…})` /
`absoluteSensor(0, 180)` / `neverHomed`; `.unhomedVolts = 4_volt`; a lift's `.shape = linkage(14_in, 10_in, 2)` or
`measured(kTable)` with `HeightAt kTable[] = {{0_in, 0}, {6_in, 30}, …}`; `.gravity = noGravity`; gearing stages
`gears(12, 36) * gears(12, 60)`; a pneumatic position's `grabs` / `releases` and `.pieceSensor` + `.confirmWithin`;
`.tuning = {...}` (N7's overrides); the wiring's `angleSensor(10)`, `pieceSensor('E')` for a switch.

**An auton** (`team5/autons_x.cpp`, a function the registry runs; `using namespace robot_x;` once per file):
```cpp
RoutineResult scoreThree(AutonContext& ctx) {
    Routine r{ctx, "score-three"};
    r.startSpinUp(flywheel["shoot"], 2_s);         // the flywheel spins up while the robot drives
    r.startRun(intake, 12_volt, seesPiece.orAfter(2.5_s), 3_s);  // until it sees a piece (or 2.5 s)
    r.driveTo(24_in, 0_in);
    r.waitUntilDone(intake, 1_s);
    r.move(arm["score"], 1.5_s);                    // the arm HOLDS 95 degrees from here on
    r.move(lift.to(18_in), 4_s);
    r.set(hood["up"]);
    r.waitUntilDone(flywheel, 1_s);
    r.run(intake, 12_volt, 0.5_s);                  // feed the piece into the flywheel
    r.stop(flywheel);
    return r.result();
}
```
and the homing moments an auton chooses: `r.home(lift, 1.5_s);` (at auton start: costs match time) or
`r.startHome(lift, 1.5_s).driveTo(12_in, 0_in).waitUntilDone(lift, 0.5_s);` (while driving); a computed target
`r.move(arm.clampedTo(x), 1_s);` (a lift's takes a length); a reverse until the piece is gone `r.run(intake,
-6_volt, stopsSeeingPiece, 1_s);`; until ANOTHER mechanism's sensor `r.run(roller, 12_volt, seesPiece(grip),
2_s);`; a squeeze, done when it stalls `r.run(catapult, 6_volt, stalls, 1_s);`. **Every optional word in this
section is compiled together, wired, controlled and stepped in one probe (`probe4_forms.cpp`, P5b).** An auton
written with today's `Routine r{ctx.chassis, …}` compiles but its first mechanism step is a loud precondition
naming the fix — so MECH-A5 changes `src/team/autons.cpp` :53 to `Routine r{ctx, …}`, the only spelling the guide
teaches (U3).

**The driver file's controls** (`team5/driver_x.hpp`; presets beside hold-to-run and toggle; the most recent input
wins):
```cpp
inline constexpr DriverControls kControls = controls(kRobotXMechanisms,
    whileHeld(arm, Button::R1, Button::R2, 6_volt),  // up / down while held; let go: it holds there
    preset(arm["stow"], Button::Down, 1.5_s),
    preset(arm["score"], Button::Right, 1.5_s),      // the most recent button wins (Q8)
    preset(arm["high"], Button::Up, 1.5_s),
    whileHeld(lift, Button::L1, Button::L2, 12_volt),
    homeAtDriverStart(lift, 1.5_s),                  // after a restart it may not be on its switch
    toggle(flywheel["shoot"], Button::X),            // on at "shoot" / off
    whileHeld(intake, Button::A, Button::B, 12_volt),
    autonOnly(hood));                                // the hood is the autons' alone
```
(also `preset(hood["up"], Button::Up)` — a pneumatic position needs no time limit, its move time is the robot
file's; `homeOnPress(lift, Button::Left, 1.5_s)` in place of `homeAtDriverStart`; both compiled in
`team5/counted_steps.cpp`. The lift's homing control is REQUIRED: a switch-homed mechanism held by a button with no
way to home in driver control is refused, n48 — its buttons would be dead after any restart off its switch, U5.)

**Robot two, as MECH-A5 leaves it** — `src/robots/mechanism_table.hpp`'s region (P10 compiles exactly this; G3a
wraps it, §6.2):
```cpp
namespace robot_two {
using namespace mechanism::words;
// [robot two mechanisms]
// The claw pivots (the team lead, 2026-09-26): RED; each holds its angle when let go. Ports 6/7 are
// as REPORTED -- no census has seen them (HA-153); each + is its motor's own forward, and which way
// that turns the claw is unmeasured (the bench page's direction check). No stall signature: held
// against its stop, a pivot is cut only by the driver letting go (HA-154); a loaded hold is
// unmeasured (HA-156).
inline constexpr Power clawPivot6{.name = "claw pivot 6", .motors = {Forward}, .cartridge = Red, .safe = Hold};
inline constexpr Power clawPivot7{.name = "claw pivot 7", .motors = {Forward}, .cartridge = Red, .safe = Hold};

// The pneumatic claw (the team lead, 2026-09-26): CLOSED at the start and when made safe. Which
// level OPENS it, and whether it is single or double, are GUESSES (HA-159; Station J.9): to flip
// it, swap Low and High below.
inline constexpr Pneumatic claw{
    .name = "claw",
    .positions = {{"closed", {Low}, grabs}, {"open", {High}}},
    .start = "closed",
    .safe = "closed",
    .moveTime = 0.5_s,  // PROVISIONAL (HA-185): Station J times it
};

// Where each one is plugged in. Re-cabling changes only these lines. A claw cable swapped with a
// drive cable is invisible to the boot (HA-155): label the cables.
inline constexpr MechanismTable kRobotTwoMechanisms = mechanisms(
    wire(clawPivot6, motors(6)),
    wire(clawPivot7, motors(7)),
    wire(claw, lines('A')));
// [/robot two mechanisms]
}  // namespace robot_two
```
its driver file under N6 (`src/team/robot_two_driver.hpp`, after today's button map in words; `team5/
robot_two_driver_core.hpp`):
```cpp
namespace shulib::bench::robot_two {
// [driver file two]
// ═══ WHICH BUTTON DOES WHAT ═══════════════════════════════════════════════════════════════
inline constexpr DriverControls kControls = controls(kRobotTwoMechanisms,
    whileHeld(clawPivot6, Button::R1, Button::R2, 12_volt),  // forward / back, full power
    whileHeld(clawPivot7, Button::L1, Button::L2, 12_volt),  // forward / back, full power
    toggle(claw, Button::A));                                // each press opens or closes it

// ═══ THE FEEL ═════════════════════════════════════════════════════════════════════════════
inline constexpr DriverProfile kFeel{.mode = ArcadeBody};
inline constexpr DriverLoopConfig kLoop{
    .rezeroButton = Button::Y,     // field-centric's "away from me"
    .releaseBrakeMode = Coast,     // let go of the sticks: the drive coasts
};
// [/driver file two]
}  // namespace shulib::bench::robot_two

namespace shulib::bench {
using RobotTwoDriver = DriverFile<robot_two::kControls, robot_two::kFeel, robot_two::kLoop>;
}  // namespace shulib::bench
```
and a claws auton (`team5/autons_x.cpp`):
```cpp
RoutineResult grabAndLift(AutonContext& ctx) {
    Routine r{ctx, "grab-and-lift"};
    r.set(claw["open"]);
    r.driveTo(20_in, 0_in);
    r.set(claw["closed"]);                     // no sensor: UNCONFIRMED, counted done (Q2 (c))
    r.run(clawPivot6, 12_volt, 0.4_s);         // pivot 6 forward for 0.4 s
    r.run(clawPivot7, 12_volt, 0.4_s);
    return r.result();
}
```
**And its ROUTE IDS, which nobody writes** (his ruling of 2026-10-08; §1.16): robot x publishes `arm_stow`,
`arm_score`, `arm_high`, `arm_home`, `lift_floor`, `lift_low`, `lift_high`, `lift_home`, `flywheel_shoot`,
`flywheel_stop`, `intake_run`, `intake_until_piece`, `intake_until_gone`, `intake_stop`, `hood_down`, `hood_middle`,
`hood_up`; robot two `claw_pivot_6_run`, `claw_pivot_6_stop`, `claw_pivot_7_run`, `claw_pivot_7_stop`,
`claw_closed`, `claw_open` (P14). A route marker carries its time limit: `arm_score [1500]`, `intake_until_piece
[12, 3000]`.

**The count** (lines a team writes; a declaration's count includes its braces; every step and control in the table
is compiled verbatim in `team5/counted_steps.cpp`):

| Mechanism | its declaration | its wiring | an auton step | its driver controls | its route ids |
|---|---|---|---|---|---|
| an arm, three presets, gravity compensation | 12 | 1 | 1 — `r.move(arm["score"], 1.5_s);` | 4 — one hold-to-run, three presets | 0 (4 published) |
| a two-motor lift in inches | 13 | 1 | 1 — `r.move(lift.to(18_in), 4_s);` | 2 — a hold-to-run, its driver-start homing | 0 (4) |
| a flywheel with "at speed" | 10 | 1 | 1 — `r.spinUp(flywheel["shoot"], 2_s);` (or a start + a wait: 2) | 1 — `toggle(flywheel["shoot"], …)` | 0 (2) |
| an intake run until its sensor sees a piece | 7 | 1 | 1 — `r.run(intake, 12_volt, seesPiece, 3_s);` (or `.orAfter(2.5_s)` to carry on without one) | 1 — a hold-to-run | 0 (4) |
| a three-position pneumatic | 7 | 1 | 1 — `r.set(hood["up"]);` | 1 — `autonOnly(hood)` (or a preset per position: 3) | 0 (3) |
| robot two's claw pivots | 1 each | 1 each | 1 — `r.run(clawPivot6, 12_volt, 0.4_s);` | 1 each — a hold-to-run | 0 (2 each) |
| robot two's claw | 7 | 1 | 1 — `r.set(claw["closed"]);` | 1 — a toggle | 0 (2) |

Once per file, not per mechanism: the robot file's `using namespace mechanism::words;` and its list's `mechanisms(`
line; the driver file's `controls(<list>,` line, its feel and loop (5 lines of code: `kFeel` 1, `kLoop` 4), and the
3-line `using RobotTwoDriver = DriverFile<…>;` in its namespace; the auton file's `#include
"robots/mechanism_table.hpp"` (new in `autons.cpp`) and `using namespace robot_two;`. **Per auton, beyond its
steps:** the signature, `Routine r{ctx, "name"};`, `return r.result();` and the closing brace (4), plus COMP-t's
8-line `AutonEntry` registration in `registerAutons` (unchanged, `autons.cpp` :62-70). **Robot two today vs
after:** its mechanisms are 43 lines of code today (`kClawOpenCommand` :171 and the entries :252-293) with 35 of
comment above them; **13 of code and 10 of comment (one per caveat — HA-153, the unmeasured signs, HA-154, HA-156,
HA-159 and single/double, HA-155) after**; its driver file's code below the button map is ~100 lines today
(:41-142), **9 lines of decisions (controls 4, feel 1, loop 4), 2 headings, 1 blank and 5 namespace / `using` lines
after** (N6). **Where today's six `static_assert`s of the driver file went** (U6): the controls' own checks (a button
twice, 13 V, a mechanism the robot lacks) are `controls(...)`'s, at build; the re-zero button and the tick (now the
brain's, supplied by `DriverFile`; a stated one must equal it — n56) are `DriverFile`'s two `static_assert`s; the
width checks of the claw row stay in `driver_panel.hpp`. **What a team does NOT type twice:** a port (only the
wiring), a mechanism's facts (only its declaration), an id (route ids are derived). A declaration types its C++
identifier and its `.name` (`clawPivot6` / "claw pivot 6"): steps find mechanisms by `.name` at run time (P12), and
the build refuses two declarations with one `.name` (n38).

### 1.2 `include/shulib/mechanism/kinds.hpp` — NEW: the five declarations and the words (MECH-A1)

THE DECLARATION IS THE HANDLE (Q18 (a)). Every field has an UNSET value; a number the team owns is never
invented (P6). The words are reached by ONE `using namespace mechanism::words;`. Verbatim public surface (bodies
in `proposed4/shulib/mechanism/kinds.hpp`):
```cpp
namespace shulib::mechanism {
struct Name {  // `.name = "arm"` -- also records the LINE it was written on (std::source_location)
    const char* text = nullptr;  const char* file = nullptr;  std::uint32_t line = 0;
    constexpr Name() noexcept = default;
    consteval Name(const char* s, std::source_location at = std::source_location::current()) noexcept;
};
struct Motor { bool present = false; bool reversed = false; const char* changeAt = nullptr; };
inline constexpr Motor Forward{.present = true};
inline constexpr Motor Reversed{.present = true, .reversed = true};
struct Motors {  // `.motors = {Forward, Reversed}` (1..4); records its line
    Motor m[kMaxMotorRoles]{};  std::uint8_t count = 0;  bool tooMany = false;
    const char* file = nullptr;  std::uint32_t line = 0;
    constexpr Motors() noexcept = default;
    consteval Motors(std::initializer_list<Motor> list, std::source_location at = std::source_location::current()) noexcept;
};
struct Sensor {
    SensorKind kind = SensorKind::None;  bool reversed = false;
    bool pressedLevel = true;        // Switch: the level that means PRESSED (HA-184)
    double seesWithinIn = 0.0;       // Distance
    double seesProximity = 0.0;      // Optical
    const char* changeAt = nullptr;  const char* file = nullptr;  std::uint32_t line = 0;
};
[[nodiscard]] constexpr Sensor rotationSensor(Motor mounted = Forward, std::source_location at = std::source_location::current()) noexcept;
[[nodiscard]] constexpr Sensor distanceSensor(units::Length within, std::source_location at = std::source_location::current()) noexcept;
[[nodiscard]] constexpr Sensor opticalSensor(double proximity, std::source_location at = std::source_location::current()) noexcept;
[[nodiscard]] constexpr Sensor limitSwitch(std::source_location at = std::source_location::current()) noexcept;
// A stall signature: at least `amps` while the MOTOR shaft turns slower than `belowMotorRpm`, for `lasting`.
// REQUIRED on an Arm, a Lift and a Flywheel (N10); amps < 2.5 and belowMotorRpm > 0 (n58, n59).
struct Stall { double amps = 0.0; double belowMotorRpm = 0.0; units::Time lasting{0.0}; };
// THE GEARING as tooth counts (N11): gears(12, 60) = the motor's 12-tooth gear drives a 60; stages multiply.
struct Gearing {
    double motorTurnsPerTurn = 0.0;   // 0 = UNSET
    constexpr Gearing() noexcept = default;
    template <class Write_the_gearing_as_tooth_counts_like_gears_12_60>
        requires std::is_arithmetic_v<Write_the_gearing_as_tooth_counts_like_gears_12_60>
    Gearing(Write_the_gearing_as_tooth_counts_like_gears_12_60) = delete;   // `.gearing = 5` names the fix (n41)
    [[nodiscard]] constexpr Gearing operator*(Gearing o) const noexcept;
};
[[nodiscard]] consteval Gearing gears(int driving, int driven);   // refuses a gear with < 1 tooth
inline constexpr Gearing direct = /* motorTurnsPerTurn 1 */;
struct GravityTerm { Gravity kind = static_cast<Gravity>(0); double volts = 0.0; double levelAtDeg = 0.0; };
[[nodiscard]] constexpr GravityTerm cosine(units::Voltage holdLevel, double levelAt) noexcept;
[[nodiscard]] constexpr GravityTerm constantGravity(units::Voltage volts) noexcept;
inline constexpr GravityTerm noGravity{Gravity::None, 0.0, 0.0};
struct Homing { HomeMethod method; double at; double sensorAtZeroDeg; double volts; Stall stall; };
[[nodiscard]] constexpr Homing restsAtStop(double at) noexcept;            // + units::Length overload
[[nodiscard]] constexpr Homing homeBySwitch(double at, units::Voltage toward) noexcept;   // + Length
[[nodiscard]] constexpr Homing homeByStall(double at, units::Voltage toward, Stall signature) noexcept;  // + Length
[[nodiscard]] constexpr Homing absoluteSensor(double at, double sensorReadsDeg) noexcept;  // HA-186
inline constexpr Homing neverHomed{.method = HomeMethod::None};
struct DegreeRange { double min = 0.0; double max = 0.0; };
struct InchRange { units::Length min{0.0}; units::Length max{0.0}; };
struct DegreePreset { const char* name = nullptr; double value = 0.0; };
struct InchPreset { const char* name = nullptr; units::Length value{0.0}; };
struct RpmPreset { const char* name = nullptr; double rpm = 0.0; };
struct LiftShape { Geometry geometry; double spoolDiameterIn; double armLengthIn, pivotHeightIn; std::uint8_t stages;
                   double table[kMaxTablePoints][2]; std::uint8_t points; };
[[nodiscard]] constexpr LiftShape spool(units::Length diameter) noexcept;
[[nodiscard]] constexpr LiftShape linkage(units::Length armLength, units::Length pivotHeight, int stages) noexcept;
struct HeightAt { units::Length height{0.0}; double degrees = 0.0; };
template <std::size_t N> [[nodiscard]] constexpr LiftShape measured(const HeightAt (&pairs)[N]) noexcept;
struct PositionTuning { double kP = 0.0, kI = 0.0, kD = 0.0, kS = 0.0, kV = 0.0, kA = 0.0;
                        double maxSpeed = 0.0, maxAcceleration = 0.0; double within = 0.0; units::Time settle{0.0}; };
struct SpeedTuning { double kS = 0.0, kV = 0.0, kP = 0.0, kI = 0.0; };

// GOALS -- made only by a declaration (arm["score"], arm.to(120), flywheel["shoot"], claw["closed"]) or by the
// route binder from a spec (§1.16); the service finds a goal's mechanism by NAME and KIND (P12)
struct PositionGoal { const void* mechanism; const char* mechanismName; const char* presetName; double value; bool computed; };
struct SpeedGoal { const void* mechanism; const char* mechanismName; const char* presetName; double rpm; bool computed; };
struct PneumaticGoal { const void* mechanism; const char* mechanismName; std::uint8_t position; const char* positionName; };

// THE FIVE KINDS -- members in the order C++ requires them written: what it IS, what it does STOPPED,
// where it GOES, the extras (N11)
struct Power {      // run at a voltage (intakes, rollers, conveyors, claw pivots)
    Name name{};  Motors motors{};  hal::MotorCartridge cartridge = …;
    Sensor pieceSensor{};  config::SafeBrakeMode safe = …;  Stall stall{};   // stall OPTIONAL here (N10)
    units::Voltage maxVolts{12.0};
    template <class T> [[nodiscard]] consteval int operator[](T) const;   // `intake["fast"]` refused BY NAME
    [[nodiscard]] constexpr MechanismSpec spec() const noexcept;
};
struct Arm {        // an angle in DEGREES, held
    Name name{};  Motors motors{};  hal::MotorCartridge cartridge = …;  Gearing gearing{};  Sensor angleSensor{};
    config::SafeBrakeMode safe = …;  Stall stall{};
    DegreeRange range{};  GravityTerm gravity{};  Homing home{};  DegreePreset presets[kMaxPresets]{};
    units::Voltage unhomedVolts{0.0};  PositionTuning tuning{};
    [[nodiscard]] consteval PositionGoal operator[](const char* preset) const;  // a misspelling FAILS THE BUILD
    [[nodiscard]] consteval PositionGoal to(double degrees) const;              // past the limits FAILS THE BUILD
    template <class T> requires(!std::is_arithmetic_v<T>)
    [[nodiscard]] consteval PositionGoal to(const T&) const;                    // `arm.to(90_deg)` refused BY NAME
    [[nodiscard]] constexpr PositionGoal clampedTo(double degrees) const noexcept;  // computed: clamped, logged
    [[nodiscard]] constexpr MechanismSpec spec() const noexcept;
};
struct Lift {       // a height in INCHES, held (spool, linkage, measured)
    Name name{};  Motors motors{};  hal::MotorCartridge cartridge = …;  Gearing gearing{};  LiftShape shape{};
    Sensor angleSensor{};  config::SafeBrakeMode safe = …;  Stall stall{};
    InchRange range{};  GravityTerm gravity{};  Homing home{};  InchPreset presets[kMaxPresets]{};
    units::Voltage unhomedVolts{0.0};  PositionTuning tuning{};
    [[nodiscard]] consteval PositionGoal operator[](const char* preset) const;
    [[nodiscard]] consteval PositionGoal to(units::Length height) const;
    template <class T> requires std::is_arithmetic_v<T>
    [[nodiscard]] consteval PositionGoal to(T) const;                           // `lift.to(18)` refused BY NAME
    [[nodiscard]] constexpr PositionGoal clampedTo(units::Length height) const noexcept;
    [[nodiscard]] constexpr MechanismSpec spec() const noexcept;
};
struct AtSpeed { double rpm = 0.0; units::Time lasting{0.0}; };
[[nodiscard]] constexpr AtSpeed within(double rpm, units::Time lasting) noexcept;
struct Flywheel {   // an RPM of its output, "at speed" (the SPEED kind)
    Name name{};  Motors motors{};  hal::MotorCartridge cartridge = …;  Gearing gearing{};  Sensor angleSensor{};
    config::SafeBrakeMode safe = …;  Stall stall{};
    AtSpeed atSpeed{};  RpmPreset presets[kMaxPresets]{};  SpeedTuning tuning{};
    [[nodiscard]] constexpr double freeRpm() const noexcept;  // the cartridge through the gearing
    [[nodiscard]] consteval SpeedGoal operator[](const char* preset) const;
    [[nodiscard]] consteval SpeedGoal at(double rpm) const;  // past freeRpm() FAILS THE BUILD
    [[nodiscard]] constexpr SpeedGoal clampedTo(double rpm) const noexcept;
    [[nodiscard]] constexpr MechanismSpec spec() const noexcept;
};
struct Levels {     // `{Low}` or `{High, Low}`: records HOW MANY were written (U2)
    bool l[kMaxLineRoles]{};  std::uint8_t count = 0;  bool tooMany = false;
    constexpr Levels() noexcept = default;
    consteval Levels(std::initializer_list<bool> list) noexcept;
};
struct NamedPosition { const char* name = nullptr; Levels levels{}; Confirm confirm = Confirm::None; };
struct Positions {  // `.positions = {{"closed", {Low}, grabs}, {"open", {High}}}`; records its line
    NamedPosition p[kMaxPneumaticPositions]{};  std::uint8_t count = 0;  bool tooMany = false;
    const char* file = nullptr;  std::uint32_t line = 0;
    constexpr Positions() noexcept = default;
    consteval Positions(std::initializer_list<NamedPosition> list, std::source_location at = std::source_location::current()) noexcept;
};
inline constexpr bool High = true;
inline constexpr bool Low = false;
inline constexpr Confirm grabs = Confirm::Sees;           // a piece sensor confirms; none: UNCONFIRMED, counted done (Q2)
inline constexpr Confirm releases = Confirm::StopsSeeing;
struct Pneumatic {  // two or three named positions, each the team's line levels (Q13 (a))
    Name name{};  Positions positions{};  const char* start = nullptr;  const char* safe = nullptr;
    units::Time moveTime{0.0};  Sensor pieceSensor{};  units::Time confirmWithin{0.0};
    const char* changeAt = nullptr;  // generated robot files only: the positions' JSON path (G3a)
    template <class T> [[nodiscard]] consteval PneumaticGoal to(const T&) const;   // refused BY NAME
    [[nodiscard]] consteval PneumaticGoal operator[](const char* position) const;
    [[nodiscard]] constexpr MechanismSpec spec(std::uint8_t lines = 1) const noexcept;  // the WIRING says how many
};
namespace words {  // what `using namespace mechanism::words;` brings, and nothing else
using namespace units::literals;
using mechanism::Arm; using mechanism::Lift; using mechanism::Pneumatic; using mechanism::Power; using mechanism::Flywheel;
using mechanism::Forward; using mechanism::Reversed;
using mechanism::rotationSensor; using mechanism::distanceSensor; using mechanism::opticalSensor; using mechanism::limitSwitch;
using mechanism::Stall; using mechanism::cosine; using mechanism::constantGravity; using mechanism::noGravity;
using mechanism::restsAtStop; using mechanism::homeBySwitch; using mechanism::homeByStall; using mechanism::absoluteSensor;
using mechanism::neverHomed; using mechanism::spool; using mechanism::linkage; using mechanism::measured;
using mechanism::HeightAt; using mechanism::within; using mechanism::High; using mechanism::Low;
using mechanism::grabs; using mechanism::releases; using mechanism::gears; using mechanism::direct;
inline constexpr hal::MotorCartridge Red = …, Green = …, Blue = …;
struct BrakeWord { config::SafeBrakeMode safe; hal::BrakeMode drive;      // ONE word for `.safe` and the loop's
    constexpr operator config::SafeBrakeMode() const noexcept;             // `.releaseBrakeMode` (N11)
    constexpr operator hal::BrakeMode() const noexcept; };
inline constexpr BrakeWord Hold{…}, Brake{…}, Coast{…};
}  // + wiring.hpp, until.hpp, driver_inputs.hpp, teleop_loop.hpp (Button, DriverProfile, DriverLoopConfig,
   //   ArcadeBody, ArcadeField), chassis/routine.hpp (Routine, RoutineResult) and competition/auton_registry.hpp
   //   (AutonContext) add their words to the same namespace
}
```
Rules the executor keeps (each pinned by a test, §3): a `consteval` member refuses by CALLING a non-constexpr
function whose NAME is the reason (`detail::no_such_preset_on_this_mechanism`, `target_is_outside_the_travel_
limits`, `speed_is_past_the_free_speed`, `no_such_position_on_this_mechanism`, the four type-shaped ones above);
**where GCC prints it, measured (`out5/neg5_summary.txt`, T5/U7):** 12 of the 66 negatives put the `error:` ON the
team's line (a deleted or named overload, `90_deg`, a misspelt mechanism); the other 54 put it in a library header
with the team's line as the first "in 'constexpr' expansion of" / "required from here" note — the guide's "reading a
refusal" lesson says so, with an example; a refusal BETWEEN entries (a port, line, button or name twice) points at
the list's first line and names the port or button; a declaration's own refusal points at its `wire(...)` line. Still
plain C++ errors, said in the guide: `arm.to(90_deg)` ("call to non-'constexpr' function operator""_deg" — `_deg` is
a WRAPPING heading, 270 reads −90, never accepted for an arm angle), a run-time value where a constant is checked
("is not a constant expression": use `clampedTo`), a misspelt mechanism ("was not declared in this scope"). The
library's starting values (N7, HA-182) are `positionStart()`'s: kP 0.25, kI 0.3 (the HOLD loop only), kD 0.004, kV =
12 V over the output's free speed, cruise 75 % of it, reached in 0.25 s, within 1.5° (0.25 in) for 0.1 s; the speed
loop's kV = 12 V over the free speed, kP 0.05, kI 0.2. The captured line is the `.name` line (GCC 13 reports an
initializer list's capture at the declaration's closing line — measured — so every declaration role points at the
`.name` line, which is exact; the WIRING line is captured where `motors(...)` / `lines(...)` is written — §1.4).

### 1.3 `include/shulib/mechanism/spec.hpp` — NEW: one mechanism as data, and THE build check (MECH-A1)

The device-independent form every declaration becomes (`spec()`), which the service, the Robot's projection,
the direction check, the route ids and G3a's agreement test read. Fixed-width, constexpr, no allocation. Verbatim
public surface:
```cpp
inline constexpr std::size_t kMaxMechanisms = config::kMaxMechanisms;              // 8
inline constexpr std::size_t kMaxMotorRoles = config::kMaxMotorsPerMechanism;      // 4
inline constexpr std::size_t kMaxLineRoles = config::kMaxPneumaticsPerMechanism;   // 2
inline constexpr std::size_t kMaxPresets = 6, kMaxPneumaticPositions = 3, kMaxTablePoints = 8, kNameBytes = 24;
enum class Kind : std::uint8_t { Power = 1, Position = 2, Speed = 3, Pneumatic = 4 };
[[nodiscard]] constexpr const char* kindWord(Kind k) noexcept;
enum class SensorKind : std::uint8_t { None = 0, Rotation = 1, Switch = 2, Distance = 3, Optical = 4 };
struct MotorRole { bool reversed = false; const char* source = nullptr; const char* file = nullptr; std::uint32_t line = 0; };
struct SensorRole { SensorKind kind; bool reversed; bool pressedLevel = true; double seesWithinIn; double seesProximity;
                    const char* source; const char* file; std::uint32_t line; };
enum class Units : std::uint8_t { Degrees = 1, Inches = 2 };
enum class Geometry : std::uint8_t { Rotary = 1, Spool = 2, Arm = 3, Table = 4 };
enum class Gravity : std::uint8_t { None = 1, Cosine = 2, Constant = 3 };
enum class HomeMethod : std::uint8_t { None = 1, RestsAtStop = 2, Switch = 3, Stall = 4, Absolute = 5 };
struct Preset { char name[16]{}; double value = 0.0; };
struct StallSpec { double amps, speedRadS, time; [[nodiscard]] constexpr bool stated() const noexcept; };
struct PowerSpec { double maxVolts; StallSpec stall; };
struct PositionSpec { Units units; Geometry geometry; double gearRatio, spoolDiameterIn, armLengthIn, pivotHeightIn;
    std::uint8_t armStages; double table[kMaxTablePoints][2]; std::uint8_t tablePoints;
    double minPosition, maxPosition, maxVelocity, maxAcceleration, tolerance, settleTime;
    double kP, kI, kD, kS, kV, kA, kG;  // per OUTPUT-SHAFT degree, whatever the units
    Gravity gravity; double horizontalAt; HomeMethod home; double homePosition, homeVolts; StallSpec homeStall, stall;
    double unhomedVolts, absoluteSensorAtZeroDeg; Preset presets[kMaxPresets]; std::uint8_t presetCount; };
struct SpeedSpec { double gearRatio, maxRpm, kS, kV, kA, kP, kI, atSpeedBandRpm, atSpeedTime;
                   Preset presets[kMaxPresets]; std::uint8_t presetCount; StallSpec stall; };   // stall: N10
enum class Confirm : std::uint8_t { None = 0, Sees = 1, StopsSeeing = 2 };
struct PneumaticPositionSpec { char name[16]{}; bool levels[kMaxLineRoles]{}; Confirm confirm = Confirm::None;
                               std::uint8_t levelCount = 0; };   // how many levels the team WROTE (U2)
struct PneumaticSpec { PneumaticPositionSpec positions[kMaxPneumaticPositions]; std::uint8_t positionCount, start, safe;
    double actuationTime, confirmWindow; const char* levelsSource; const char* levelsFile; std::uint32_t levelsLine;
    bool startNamed = true, safeNamed = true; };
struct MechanismSpec { char name[kNameBytes]{}; Kind kind; MotorRole motors[kMaxMotorRoles]; std::uint8_t motorCount, lineCount;
    SensorRole angleSensor, homeSwitch, pieceSensor; hal::MotorCartridge cartridge; config::SafeBrakeMode safeMode;
    PowerSpec power; PositionSpec position; SpeedSpec speed; PneumaticSpec pneumatic;
    const char* declaredFile; std::uint32_t declaredLine; bool presetNameTooLong; };
[[nodiscard]] constexpr std::string_view nameOf(const MechanismSpec& s) noexcept;
[[nodiscard]] inline double angleDegToPosition(const PositionSpec& p, double outDeg) noexcept;  // degrees -> units
[[nodiscard]] inline double positionToAngleDeg(const PositionSpec& p, double pos) noexcept;     // units -> degrees
enum class Refusal : std::uint8_t { None = 0, /* 60 reasons, one property each, in check order */ };
[[nodiscard]] constexpr const char* refusalText(Refusal r) noexcept;   // the sentence
namespace refused { /* one non-constexpr function per reason, named in words */ }
consteval void refuse(Refusal r);                                        // stops the build, naming it
[[nodiscard]] constexpr Refusal specRefusal(const MechanismSpec& s) noexcept;   // THE build check
[[nodiscard]] constexpr const char* specProblem(const MechanismSpec& s) noexcept;  // the same, as a sentence
```
The 60 refusals (`proposed4/.../spec.hpp` :281-345: the 10-06 draft's 55 plus, appended and pinned,
`PositionTheStallSignatureIsUNSET`, `SpeedTheStallSignatureIsUNSET` (N10), `PositionHomingPushesAwayFromItsHomeEnd`
(U4), `AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit` and `AStallSignatureNeedsBelowMotorRpmAbove0`
(N10's never-trip pair — checked for EVERY stated signature: a power's, a position's, its homing's, a flywheel's))
are G3a's validator's list too (§6.3): every one has a sentence (`refusalText`), a named function, and a test
(§3.1). Remove `MechanismSpec::autonOnly` (the overlay still carries it, unused, :205: auton-only is a DRIVER-file
mark, `autonOnly(hood)`, never a robot-file field — §6.3).

### 1.4 `include/shulib/mechanism/wiring.hpp` — NEW: the binding, checked at build; the projection; the boot line (MECH-A1)

The wiring is ONLY in the robot file, compiled in (Q1 (a)). Verbatim public surface:
```cpp
enum class Line : std::uint8_t { None = 0, A = 1, B, C, D, E, F, G, H };  // its own type: never a smart port
[[nodiscard]] constexpr char lineLetter(Line l) noexcept;
struct MechanismWiring { char mechanism[kNameBytes]{}; std::uint8_t motors[kMaxMotorRoles]{}; Line lines[kMaxLineRoles]{};
    std::uint8_t angleSensor = 0; Line homeSwitch = Line::None; std::uint8_t pieceSensorPort = 0; Line pieceSwitch = Line::None;
    const char* file = nullptr; std::uint32_t line = 0; };   // its wire(...) line (captured by motors()/lines())
struct MechanismTable {  // ONE robot's mechanisms: definitions + wiring, same order -- the ONE team-facing type (N11)
    MechanismSpec specs[kMaxMechanisms]{};  MechanismWiring wiring[kMaxMechanisms]{};  std::uint8_t count = 0;
    const void* declared[kMaxMechanisms]{};  // the declarations, in order (a control's build-time check)
    [[nodiscard]] constexpr int indexOf(const void* d) const noexcept;
    [[nodiscard]] constexpr std::span<const MechanismSpec> specSpan() const noexcept;
    [[nodiscard]] constexpr std::span<const MechanismWiring> wiringSpan() const noexcept;
};
[[nodiscard]] constexpr const char* wiringProblem(std::span<const MechanismSpec> specs, std::span<const MechanismWiring> wiring,
                                                  std::span<const std::uint8_t> drivePorts, std::uint8_t imuPort) noexcept;
[[nodiscard]] constexpr config::MechanismConfig toMechanismConfig(const MechanismSpec& s, const MechanismWiring& w) noexcept;  // THE ONE PROJECTION
struct MotorPorts { std::uint8_t port[kMaxMotorRoles]{}; std::uint8_t count = 0; bool bad = false;
                    const char* file = nullptr; std::uint32_t line = 0; };
template <class A> [[nodiscard]] consteval MotorPorts motors(A a, std::source_location at = std::source_location::current());  // + 2, 3, 4 ports
                                                                       // NUMBERS: motors('A') refused BY NAME (n27)
struct LinePorts { Line line[kMaxLineRoles]{}; std::uint8_t count = 0; bool bad = false;
                   const char* file = nullptr; std::uint32_t lineNo = 0; };
template <class A> [[nodiscard]] consteval LinePorts lines(A a, std::source_location at = std::source_location::current());     // + 2 lines
                                                                       // LETTERS: lines(6, 7) refused BY NAME (n11)
struct RolePort { enum class Role : std::uint8_t { Angle, Piece, HomeSwitch } role; std::uint8_t port = 0; Line line = Line::None; bool bad = false; };
template <class P> [[nodiscard]] consteval RolePort angleSensor(P port);   // a number; angleSensor('B') refused BY NAME
template <class P> [[nodiscard]] consteval RolePort pieceSensor(P port);   // a number (distance / optical) or a letter (switch)
template <class C> [[nodiscard]] consteval RolePort homeSwitch(C line);    // a letter; homeSwitch(66) refused BY NAME (n26)
struct Wired { MechanismSpec spec{}; MechanismWiring wiring{}; const void* declared = nullptr; };
template <class... R> [[nodiscard]] consteval Wired wire(const Power& d, MotorPorts m, R... r);   // + Arm, Lift, Flywheel
template <class... R> [[nodiscard]] consteval Wired wire(const Pneumatic& d, LinePorts l, R... roles);
template <class... R> consteval Wired wire(const Pneumatic&, MotorPorts, R...);                    // refused BY NAME (n32)
template <class D, class... R> requires(!std::same_as<D, Pneumatic>) consteval Wired wire(const D&, LinePorts, R...);  // (n52)
template <class... W> [[nodiscard]] consteval MechanismTable mechanisms(const W&... wired);
[[nodiscard]] consteval MechanismTable mechanisms();                     // a robot with none
[[nodiscard]] constexpr bool clearOfDrive(const MechanismTable& r, std::span<const std::uint8_t> drivePorts, std::uint8_t imuPort) noexcept;
[[nodiscard]] constexpr std::size_t declaredMotorPorts(const MechanismTable& r, std::span<int> out) noexcept;
enum class RoleState : std::uint8_t { Present, Refused, NotCheckable, NotAsked };
inline void roleLine(char* out, std::size_t cap, const char* mechanismName, const char* role, std::uint8_t port,
                     Line line, RoleState st, const char* refusal, bool degradeOnly);   // ONE boot line per role
namespace words { using mechanism::MechanismTable; using mechanism::mechanisms; using mechanism::wire;
                  using mechanism::motors; using mechanism::lines; using mechanism::angleSensor;
                  using mechanism::pieceSensor; using mechanism::homeSwitch; }
```
`wire()` refuses, at the robot file's line, every spec refusal (§1.3) and: a port outside 1..21 or more than four
motors; not a letter A..H or more than two lines; a different motor count than the declaration; a pneumatic
position stating a different number of levels than the wiring has lines (`a_position_states_a_different_number_of_
levels_than_the_wiring_has_lines`, checked BEFORE the spec's own checks, both ways — n42, n43; U2); a sensor the
declaration has but the wiring leaves unplugged, or the reverse; a smart sensor given a letter, a switch given a
number; motors on a pneumatic, lines on a motor kind. `mechanisms()` refuses a smart port or a 3-wire port used twice
**naming it** (`smart_port_4_is_used_twice`, `three_wire_port_C_is_used_twice`), the same declaration twice, two
declarations sharing a name, more than eight, and **any route-id problem** (§1.16: n60–n63). 18 reasons in all (the
"used twice" pair speaks through 29 named variants). `toMechanismConfig()` is the ONE projection to the device-level
config (signed ports: a reversed role is a negative port, reversed once by its adapter; per-line start and safe
levels when the positions need different levels per line), PROVED identical to today's conversion for robot two
(P9a, P10). The DRIVE check is the robot table file's (`mechanismsClearOf(list, chassisTable)`, §1.13) — the typed
drive at build — and `RobotConfig::valid()` at run time, against the drivetrain the boot chose.

### 1.5 The devices: five provider accessors, the read-fault channel, the config's sensor roles, the Robot builds every role (MECH-A1)

- **`hal/read_faults.hpp` — NEW, an F4-ADDITIVE SIBLING** (like `IDigitalIn`, F14; F4's locked row is untouched),
  and `hal/fake/fake_read_faults.hpp` (a test's settable counter; the F4 fakes stay unmodified — a test pairs one
  with one of them). Verbatim:
  ```cpp
  class IReadFaults {   // how many reads a sensor adapter has SCREENED (answered with its last good value)
  public:
      virtual ~IReadFaults() = default;   // + the five defaulted special members, IDistance's way
      [[nodiscard]] virtual int faultedReads() const noexcept = 0;   // monotonic
  };
  ```
  `hal::pros::ProsDistance`, `ProsOptical` and `ProsDigitalIn` derive from it beside their seam (their existing
  `faultedReads()` gains `override`; adapters are implementations, not contracts). WHY (the re-check's R probe):
  the locked seams carry no validity channel, so a held value is indistinguishable from a fresh one, and a
  distance sensor held "far" is byte-for-byte the 9999 "no object" reading (`hal/pros/distance.hpp`'s T7 note).
- **`hal/device_provider.hpp` (seam)** — five pure virtuals (`proposed4/` diff, verbatim):
  ```cpp
  [[nodiscard]] virtual IDistance* distance(std::uint8_t port) = 0;    // a piece sensor; nullptr + refusal on a wrong port
  [[nodiscard]] virtual IOptical* optical(std::uint8_t port) = 0;      // likewise
  [[nodiscard]] virtual IDigitalIn* digitalIn(std::uint8_t line) = 0;  // 1..8; refused ONLY outside 1..8 (HA-158);
                                                                       // PRECONDITION: not also made a digitalOut
  [[nodiscard]] virtual const IReadFaults* readFaults(std::uint8_t port) const noexcept = 0;      // the distance /
                                                       // optical made at `port`; nullptr = none, or never screens
  [[nodiscard]] virtual const IReadFaults* lineReadFaults(std::uint8_t line) const noexcept = 0;  // the digitalIn's
  ```
  answered by `hal::pros::ProsDeviceProvider` (registry-checked through `expectKind` — kind DISTANCE / OPTICAL;
  the 3-wire input made on first request, never asked of the registry; `readFaults` returns its own adapter),
  `hal::fake::FakeDeviceProvider` (its own `FakeReadFaults` per port and line, `ownReadFaults(port)` for a test),
  and the two test-local providers in `test/robot_builder_test.cpp` (forwarding; `proposed_test3/` is that edit,
  10 lines). Idempotent, like `rotation()`. PURE on purpose: a provider that forgot the read-fault pair would
  silently re-open the false "piece gone" it exists to stop.
- **`config/robot_config.hpp`** — `MechanismConfig` APPENDS `std::uint8_t rotationPort, distancePort, opticalPort;
  std::uint8_t switchLines[2]; PneumaticLevel pneumaticSafeByLine[kMaxPneumaticsPerMechanism],
  pneumaticStartByLine[kMaxPneumaticsPerMechanism];` (all 0 = today's form); `PortUse` APPENDS
  `MechanismSensor`; `valid()` claims the sensor ports through the one-owner `claim` (so a record that drives a
  sensor's port is refused naming both, P4) and the switch lines against the pneumatic lines, refuses a distance
  AND an optical piece sensor on one mechanism, and a switch line outside 1..8.
- **`hal/mechanism.hpp`** — `MotorMechanism::meanPosition()` (the group's mean output-shaft angle, radians,
  cumulative); `PneumaticMechanism`'s PER-LINE form: a constructor taking one safe level per line, `setLevels
  (std::span<const bool>)`, `commandedLevel(i)`, and `applySafeState()` applying the per-line safe levels — so a
  three-position mechanism has a declared safe POSITION the guard applies with no mechanism knowledge.
- **`include/shulib/mechanism/devices.hpp`** — NEW, HAL pointers only (so `robot.hpp` may produce it without an
  upward include):
  ```cpp
  struct Devices {
      hal::MotorMechanism* motors = nullptr;  hal::PneumaticMechanism* pneumatic = nullptr;
      hal::IRotation* angleSensor = nullptr;  hal::IDigitalIn* homeSwitch = nullptr;
      hal::IDistance* pieceDistance = nullptr;  hal::IOptical* pieceOptical = nullptr;  hal::IDigitalIn* pieceSwitch = nullptr;
      const hal::IReadFaults* pieceFaults = nullptr;       // the piece sensor's read-fault count
      const hal::IReadFaults* homeSwitchFaults = nullptr;  // the home switch's
      const char* absence = "";  // why the ACTUATOR was not built; "" when it was
  };
  ```
- **The Robot builds every role** — the builder `probe3_wiring.cpp` :58-108 prototypes (P4 ran it through the
  real `ProsDeviceProvider`), moved into a library class the Robot OWNS and Bench Tests' station 13 constructs
  on its own (it builds no Robot). *Its home is designed here, not in the overlay: the overlay left
  `robot.hpp` untouched and P4 ran the builder as a free function; the class is that function plus the storage
  `robot.hpp` :1189-1300 already holds, moved —*
  `robot::MechanismParts` in `include/shulib/robot/mechanism_parts.hpp` — `void build(std::span<const
  config::MechanismConfig>, hal::IDeviceProvider&)`; `std::span<const mechanism::Devices> devices() const`
  (each piece sensor's and home switch's read-fault counter filled from `readFaults(port)` /
  `lineReadFaults(line)`); `std::span<hal::IMechanism* const> built() const` (today's `mechanisms()` span,
  unchanged for the guard); and per role the facts a boot line needs (`RoleState`, the provider's refusal
  sentence). Order, per mechanism: the actuators first (today's code, `robot.hpp` :1212-1270, moved); then the
  angle sensor (refused = the mechanism ABSENT); the switches (3-wire: always made, "not checkable"); then the piece
  sensor (refused = it runs WITHOUT one); once ABSENT, the rest of its roles are "not asked". `robot::Robot` keeps
  every public accessor it has (`mechanismCount`, `builtMechanismAt`, `mechanismAt`, `pneumaticMechanismAt`,
  `mechanismAbsence`, `mechanism(name)`, `pneumaticMechanism(name)`, `mechanisms()`) by forwarding, and gains
  `const robot::MechanismParts& mechanismParts() const`. The builder still never ticks and never commands a motor
  (`robot.hpp` :114-118's rule).

### 1.6 `include/shulib/mechanism/until.hpp` and `service.hpp` — NEW: done-conditions and THE SERVICE (MECH-A2, A3)

**Done-conditions** — any combination, ALWAYS beside the step's bound (P4; `Until` never replaces it). Verbatim:
```cpp
enum class DoneBy : std::uint8_t { None = 0, Time, Reached, Stalled, Sees, StopsSeeing };
[[nodiscard]] constexpr const char* doneByWord(DoneBy d) noexcept;  // "time", "reached", "stalled", "sees a piece", ...
class Until {
public:
    static constexpr const char* kSelf = nullptr;
    [[nodiscard]] static constexpr Until time(units::Time t) noexcept;
    [[nodiscard]] static constexpr Until stalled() noexcept;      // a squeeze: SUCCEEDED, not a fault
    [[nodiscard]] static constexpr Until sees() noexcept;         // THIS mechanism's piece sensor
    [[nodiscard]] static constexpr Until stopsSeeing() noexcept;
    template <class Declared> [[nodiscard]] static constexpr Until sees(const Declared& other) noexcept;  // ANOTHER's, by name
    template <class Declared> [[nodiscard]] static constexpr Until stopsSeeing(const Declared& other) noexcept;
    [[nodiscard]] static constexpr Until bound() noexcept;        // only the bound ends it (TIMED_OUT)
    [[nodiscard]] constexpr Until operator|(const Until& o) const noexcept;  // either ends the step
    [[nodiscard]] constexpr Until orAfter(units::Time t) const noexcept;    // `seesPiece.orAfter(2.5_s)`: SUCCEEDED
                                                                            // by time if no piece (U10); t < the bound
    [[nodiscard]] constexpr bool has(std::uint8_t bit) const noexcept;
    [[nodiscard]] constexpr double seconds() const noexcept;
    [[nodiscard]] constexpr const char* sensor() const noexcept;
    static constexpr std::uint8_t kTime = 1, kStalled = 4, kSees = 8, kStops = 16;
};
struct SeesPiece : Until { template <class Declared> [[nodiscard]] consteval Until operator()(const Declared& other) const; };
struct StopsSeeingPiece : Until { template <class Declared> [[nodiscard]] consteval Until operator()(const Declared& other) const; };
inline constexpr SeesPiece seesPiece{};                 // r.run(intake, 12_volt, seesPiece, 3_s)
inline constexpr StopsSeeingPiece stopsSeeingPiece{};   // seesPiece(claw): another's sensor -- refused AT BUILD if it has none
inline constexpr Until stalls = Until::stalled();
namespace words { using mechanism::seesPiece; using mechanism::stopsSeeingPiece; using mechanism::stalls; }
```
**The service** — ONE object, ticked once per control tick, running every mechanism's behaviour so it goes on
inside a blocking drive verb and every driver tick. Each mechanism's SLOT is its claimant (`hal::ICancellable`),
so the guard's cancel-all reaches every running behaviour; one behaviour per mechanism; a new command REPLACES
the running one (Q8); a mechanism another owner has claimed is never commanded (REFUSED). Verbatim surface:
```cpp
struct StepReport { MechanismOutcome outcome = MechanismOutcome::Running; DoneBy doneBy = DoneBy::None;
                    bool countedDone = false;   // UNCONFIRMED only because no sensor exists (Q2): the chain goes on
                    double elapsed = 0.0; };
class Slot : public hal::ICancellable {          // non-copyable, non-movable
public:
    [[nodiscard]] Kind kind() const noexcept;  [[nodiscard]] const MechanismSpec& spec() const noexcept;
    [[nodiscard]] const char* name() const noexcept;  [[nodiscard]] std::uint8_t index() const noexcept;
    [[nodiscard]] bool built() const noexcept;  [[nodiscard]] const char* absence() const noexcept;
    [[nodiscard]] const StepReport& lastStep() const noexcept;
    [[nodiscard]] bool running() const noexcept;          // the STEP is still working (a hold after it is not)
    [[nodiscard]] std::uint32_t commandSeq() const noexcept;  // bumped by EVERY command: "most recent wins"
    [[nodiscard]] bool holdsClaim() const noexcept;
    enum class Seen : std::uint8_t { Yes, No, Unknown };  // POSITIVE evidence only (N8 amended, §1.7)
    [[nodiscard]] Seen seen() const;  [[nodiscard]] bool sees() const;   // sees() == (seen() == Yes)
    [[nodiscard]] bool homeSwitchPressed() const;        // a FRESH pressed read (a held level is never a home)
    [[nodiscard]] bool declaresPieceSensor() const noexcept;   // the DECLARATION has one
    [[nodiscard]] bool hasPieceSensor() const noexcept;        // it was built this boot
    [[nodiscard]] bool pieceSensorMissing() const noexcept;    // declared, missing this boot (N8)
    void cancel() override;   // THE GUARD'S REACH: inert + declared safe state + claim released; a running step CANCELLED
};
class PowerSlot final : public Slot { public:
    void run(units::Voltage volts, Until until, units::Time bound);  // clamped to maxVolts; returns at once;
                                                                     // a condition that can never be met: PRECONDITION (D7)
    void stop();                                                     // safe state, claim released
    void drive(units::Voltage volts);                                // the driver's hold-to-run (open-ended; a
                                                                     // stated stall signature watches it, N10)
    [[nodiscard]] double commandedVolts() const; };
class PositionSlot final : public Slot { public:
    void moveTo(double pos, units::Time bound, bool computed = true);  // NOT_HOMED before homing; a target past a limit CLAMPED, logged once
    void moveToPreset(std::uint8_t i, units::Time bound);
    void holdHere();  void home(units::Time bound);  void jog(units::Voltage volts);  void release();
    [[nodiscard]] bool homed() const noexcept;  [[nodiscard]] double position() const;
    [[nodiscard]] double target() const noexcept;  [[nodiscard]] int softStops() const noexcept;
    [[nodiscard]] double jointDeg() const; };
class SpeedSlot final : public Slot { public:
    void spinUp(double rpm, units::Time bound, bool computed = true);  // SUCCEEDS at speed; keeps spinning
    void spinUpPreset(std::uint8_t i, units::Time bound);  void stop();
    [[nodiscard]] bool spinning() const noexcept;  [[nodiscard]] double rpm() const;  [[nodiscard]] bool atSpeed() const noexcept; };
class PneumaticSlot final : public Slot { public:
    void set(std::uint8_t p);   // the lines NOW; the verdict after moveTime (+ confirmWithin with a sensor)
    [[nodiscard]] std::uint8_t commandedPosition() const noexcept;  [[nodiscard]] double lastStepBound() const noexcept; };

class MechanismService final : public motion::ITickPacer {
public:
    explicit MechanismService(motion::ITickPacer& inner) noexcept;   // non-copyable, non-movable
    // boot: one slot per spec over its devices; homing WITHOUT motion happens here
    void bind(std::span<const MechanismSpec> specs, std::span<const Devices> devices,
              hal::IClock& clock, diag::FaultLatch& faults, hal::ITelemetrySink& telemetry);
    void bind(const MechanismTable& robot, std::span<const Devices> devices, hal::IClock& clock,
              diag::FaultLatch& faults, hal::ITelemetrySink& telemetry);
    void pace() override;          // THE PACER SEAM (autons): tick once, then the inner pacer; composed UNDER the guard
    void tickFromDriverLoop();     // THE DRIVER LOOP'S STATED PLACE; a pace() right after it does NOT tick again
    void enterDriverControl();     // drop auton-era behaviour; a leftover claim released WITHOUT its cancel() (T3 Part 2)
    int enterAutonomous();         // (D1) drop driver-era behaviour a PROS-deleted driver task left: the re-entrancy
                                   // flag cleared, a foreign claim released WITHOUT its cancel() (counted), motor kinds
                                   // safe, pneumatics KEEP their level, a running step CANCELLED; returns the count
    void leaveDriverControl();     // the disable: motor kinds safe; pneumatics KEEP their level (HA-157)
    [[nodiscard]] std::size_t size() const noexcept;
    [[nodiscard]] Slot& slot(std::size_t i);  [[nodiscard]] const Slot* slotPtr(std::size_t i) const noexcept;
    [[nodiscard]] PowerSlot& power(std::size_t i);  [[nodiscard]] PositionSlot& position(std::size_t i);
    [[nodiscard]] SpeedSlot& speed(std::size_t i);  [[nodiscard]] PneumaticSlot& pneumatic(std::size_t i);
    // A DECLARATION's (or a goal's) slot: by KIND (the overload) and NAME (unique on a robot); nullptr = not on this robot
    [[nodiscard]] PowerSlot* find(const mechanism::Power& d);  [[nodiscard]] PositionSlot* find(const Arm& d);
    [[nodiscard]] PositionSlot* find(const Lift& d);  [[nodiscard]] SpeedSlot* find(const mechanism::Flywheel& d);
    [[nodiscard]] PneumaticSlot* find(const mechanism::Pneumatic& d);
    [[nodiscard]] PositionSlot* find(const PositionGoal& g);  [[nodiscard]] SpeedSlot* find(const SpeedGoal& g);
    [[nodiscard]] PneumaticSlot* find(const PneumaticGoal& g);   // also checks the POSITION's name at its index
    [[nodiscard]] const Slot* slotFor(const char* name) const noexcept;
    [[nodiscard]] std::span<hal::IMechanism* const> mechanisms() const noexcept;  // every BUILT one: the guard's span
    [[nodiscard]] long ticks() const noexcept;  [[nodiscard]] int leftoverClaimsReleased() const noexcept;
    [[nodiscard]] hal::IClock& clock() const;  [[nodiscard]] diag::FaultLatch& faults() const;
    [[nodiscard]] hal::ITelemetrySink& telemetry() const;
};
```
Re-entrancy (requirements :142-152), each pinned: the tick is never re-entered (a precondition); it never calls a
blocking verb, `waitUntil` or `scheduler.cancel()`; a predicate may READ a slot, never tick it; **exactly once
per tick** whichever pacer the driver loop is handed (P8: raw / the service / `guard{service}` — 600/600/600);
unbound, `pace()` ticks nothing and forwards (a program builds the chain before the Robot and binds after); at the
END of every tick each slot records its sensors' read-fault counts (the read-fault rule, §1.7). A step's verdict is
ONE `[MECH]` line (`'arm': SUCCEEDED -- reached in 0.88s`), Warn unless SUCCEEDED; STALLED raises
`MECHANISM_STALLED` once; every (re-)home says where zero went (D10). Sizes (S3): `MechanismService` 8 048 B,
static storage in every program.

### 1.7 The four behaviours — the rules the probes measured (MECH-A2: power, pneumatic; MECH-A3: position, speed)
- **Piece sensors are POSITIVE evidence (N8 amended; D7, D8; the read-fault channel).** `seen()` answers YES only
  on a fresh reading that sees (a distance closer than its threshold with confidence above 0; an optical at or
  above its proximity; a switch at its pressed level), NO only on a fresh reading that does not (a distance at or
  beyond the threshold — 9999's honest 393.66 in included; an optical below; a switch released), and UNKNOWN
  otherwise: no sensor built, a distance held NEAR with confidence 0, and **any sample taken while the sensor's
  read-fault count moved since the service's previous tick**. UNKNOWN satisfies NEITHER condition. A condition on a
  mechanism whose DECLARATION has no piece sensor (or `stalls` with no `.stall`) is a PRECONDITION at the step,
  naming the field to add — nothing is commanded (`out5/adv_conditions.txt`); `seesPiece(other)` on a mechanism
  with none is refused at build (n39, n40). At the bound: a declared sensor MISSING this boot → UNCONFIRMED,
  counted done (N8 (a)); a sensor that stopped answering → UNCONFIRMED, NOT counted ("its piece sensor stopped
  answering"); otherwise TIMED_OUT. The home switch reads the same way: a held level is never a home, at bind or
  while homing (S). What the channel cannot see: an UNPLUGGED 3-wire switch reads a level, not PROS_ERR (HA-184).
- **Power.** `run(volts, until, bound)`: volts clamped to `maxVolts`; done by time (SUCCEEDED), by its own or
  another's piece sensor (SUCCEEDED, `sees a piece`), by `stalls` (SUCCEEDED), by `orAfter`'s time (SUCCEEDED,
  `time`) — or STALLED by a stated stall signature that `stalls` did not ask for (fault raised), or as above at the
  bound; every exit applies the safe state and releases the claim. `drive(volts)` is the driver's open-ended
  hold-to-run (the release is its stop); a stated signature watches it too (STALLED, the command number bumped:
  the held button is superseded until released — `out5/adv_extra.txt` (3), 0.30 s). With NO signature (allowed on
  a Power, N10), the boot says a held button is cut only by its release (HA-154).
- **Pneumatic.** `set(p)` writes the position's level to EACH line now (per-line levels; a three-position
  mechanism on two lines); the step ends after `moveTime` — SUCCEEDED when the position confirms nothing; a
  `grabs` / `releases` position with a piece sensor SUCCEEDS when the sensor agrees within `confirmWithin`, else
  UNCONFIRMED (not counted); with NO sensor it is UNCONFIRMED and **counted done** (Q2 (c)) — the chain goes on.
  The claim is released at the verdict; the level persists. Driver entry sends the START position on the FIRST
  entry only (T4's ruling: "starts closed — whatever autonomous left it at"); a disable changes nothing.
- **Position.** Before homing a move is NOT_HOMED and a jog runs at most `unhomedVolts` (0 = nothing: no limits
  known, no power invented); homing per §1.2's methods, with no motion at bind for `restsAtStop`, `absoluteSensor`
  and a home switch already pressed on a fresh read (P2e); every (re-)home logs where zero went, Warn when a
  re-home moves it past the tolerance (D10: stall-homing trusts the first stall, `restsAtStop` the boot pose — the
  guide says both). A move is a trapezoid profile in OUTPUT degrees (a lift's inches converted at the move's start:
  exact for a spool, a conservative scale for a linkage or a table) tracked by P+D plus feedforward (kS, kV, kA, and
  gravity — `kG·cos(angle − level)` for an arm, `kG` for a lift); when the profile ends a FRESH hold loop (P+I+D,
  integral clamped to 3 V) takes over, so the integral never winds up behind the profile (P2a without it: up to
  3.6° of steady error under a gravity mismatch). **REACHED = within the tolerance IN THE MECHANISM'S OWN UNITS**
  (`|angleDegToPosition(jd) − target| ≤ tolerance`, D4 — a degree-converted tolerance is 0 at a table's end or a
  linkage's top) for the settle time → SUCCEEDED, and the HOLD goes on. **The stated stall signature watches the
  move, the hold that follows, a jog and the release's hold** (N10): STALLED, 0 V, claim released (P2c, case A).
  Out of time → TIMED_OUT and HOLD WHERE IT STOPPED (N2). **No hold target is ever outside the travel limits** (the
  release, N2's hold-where-stopped and the soft stop are all clamped — a latent defect this re-check found). The
  two limit rules (P2d): a constant target past a limit never compiles; a computed one is clamped and logged once
  (`'arm': target 175.00 clamped to the travel limit 160.00 (soft stop)`); **a jog pushing past a limit is HELD AT
  THE LIMIT by the closed-loop hold** (P+I+D + gravity), entered at the stopping distance at the profile's own
  acceleration — never the open-loop gravity estimate the 10-06 draft cut to (D2: worst 0.97–4.06° past at 8 V,
  plant kG 1×–2×; was down to the hard stop). The guide: limits sit short of the hard stops by more than the
  measured overshoot. `holdHere()` holds where it is (gravity included, clamped) — a driver's release.
- **Speed.** Feedforward (kS + kV·ω) plus P on the speed error, plus an I term that acts ONLY within 10 % of the
  target (P3: an integral from the start overshot +219 rpm matched, +467 with a lighter wheel) and is clamped to
  3 V; "at speed" = within the band for the time → SUCCEEDED, and it KEEPS spinning until stopped; out of time →
  TIMED_OUT and still spinning (N2's exception: an UNREACHABLE wheel spins). A JAMMED wheel trips its REQUIRED
  stall signature → STALLED, 0 V (N10, case H). The output speed is the motor velocity through the gearing, or the
  angle sensor when one is wired.

### 1.8 The guard and the outcomes (MECH-A2)
- **`manipulation/mechanism_outcome.hpp`** APPENDS (the file's own append-only rule, :33): `Absent = 6` (not
  built: nothing moved), `NotHomed = 7` (a position move before homing), `Refused = 8` (another owner holds the
  claim, or a route marker's args are wrong: nothing moved) and their words `ABSENT`, `NOT_HOMED`, `REFUSED`
  (finding 13: never "a target the build could not check" — a computed target is CLAMPED).
- **`sequence/run_guard.hpp`** (N1 (a)) — verbatim:
  ```cpp
  /// MECH (appended): claimed mechanisms the latch cancelled after the deadline or an abandon
  int mechanismCuts = 0;   // in RunGuardReport, after pacesSeen
  // in pace(): in the DEADLINE branch (never inside the end action), and in the ABANDON branch (everywhere,
  // the end action included -- B1), right after cutActiveMotion():
  cutClaimedMechanisms();
  /// The latch reaches MECHANISMS too: every listed mechanism whose claim has a registered claimant is
  /// cancelled at the cut -- inert, safe, released. Unclaimed ones are NOT re-safed every pace (HA-144).
  void cutClaimedMechanisms() {
      for (hal::IMechanism* m : mechanisms_) {
          if (hal::ICancellable* c = m->claimant(); c != nullptr) { c->cancel(); ++mechanismCuts_; }
      }
  }
  ```
  **The DEADLINE cut never runs inside the end action (`!inEndAction_`, unchanged); an ABANDON (a disable) cuts
  claimed mechanisms everywhere, the end action included — B1's "exempts NOTHING"** (D5; the overlay's
  `run_guard.hpp` :322-327 already does; measured `out5/adv_endaction.txt`: a disable mid end action → the
  in-end-action step CANCELLED, returned at T+1.52, 0.1 V·s; the tree's guard, in the checker's run
  `check-control/adv_endaction.txt`: T+3.92, TIMED_OUT, 28.9 V·s after the disable). The banner gains a paragraph;
  the check-then-step ordering pin is untouched.

### 1.9 Autons: ONE eager call per step, the `AutonContext` lookup, run while driving (MECH-A4)

**`chassis/routine.hpp`** (F10 LOCKED; ADDITIVE growth is legal by its own banner :20 — "new steps, new observers,
new RoutineResult fields, appended RoutineStopCause enumerators" — and the second constructor is a "new overload"
under `include/shulib/version.hpp` :24-28; a minor API bump, **2.4 → 2.5**, whose changelog and the F10 register row
say both — T4). Verbatim additions:
```cpp
// RoutineStopCause, APPENDED (F1's precedent; pinned == 5 beside test/routine_signature_pin_test.cpp :347-350):
MechanismStepFailed,  ///< a mechanism STEP (move, run, set, spinUp, home, waitUntilDone) ended other than SUCCEEDED or
                      ///< counted UNCONFIRMED -- the verdict is RoutineResult::mechanismVerdict. MechanismFailed keeps
                      ///< its F1 meaning (a then()-action's verdict), its doc comment untouched.
// RoutineResult, APPENDED after `exit`:
/// MECH: the failing MECHANISM step's verdict; `Running` = no mechanism verdict here (read `cause` first).
manipulation::MechanismOutcome mechanismVerdict = manipulation::MechanismOutcome::Running;

// A SECOND constructor -- the frozen one (`explicit Routine(Chassis&, const char*)`, :209) is untouched:
template <class Context>
    requires requires(Context& c) {
        { c.chassis } -> std::convertible_to<Chassis&>;
        { c.mechanismService } -> std::convertible_to<mechanism::MechanismService*>;
    }
explicit Routine(Context& ctx, const char* name = "routine") noexcept;   // `Routine r{ctx, "score"}`

// THE STEPS -- each runs the moment it is chained; the blocking form waits for its verdict within `bound`:
Routine& move(mechanism::PositionGoal goal, units::Time bound, const char* name = nullptr);
Routine& startMove(mechanism::PositionGoal goal, units::Time bound, const char* name = nullptr);
Routine& home(const mechanism::Arm& m, units::Time bound, const char* name = nullptr);        // + Lift
Routine& startHome(const mechanism::Arm& m, units::Time bound, const char* name = nullptr);   // + Lift
Routine& run(const mechanism::Power& m, units::Voltage volts, mechanism::Until until, units::Time bound, const char* name = nullptr);
Routine& run(const mechanism::Power& m, units::Voltage volts, units::Time time, const char* name = nullptr);
Routine& startRun(const mechanism::Power& m, units::Voltage volts, mechanism::Until until, units::Time bound, const char* name = nullptr);
Routine& startRun(const mechanism::Power& m, units::Voltage volts, units::Time time, const char* name = nullptr);
Routine& set(mechanism::PneumaticGoal goal, const char* name = nullptr);      // its move time is the robot file's
Routine& spinUp(mechanism::SpeedGoal goal, units::Time bound, const char* name = nullptr);
Routine& startSpinUp(mechanism::SpeedGoal goal, units::Time bound, const char* name = nullptr);
Routine& stop(const mechanism::Power& m, const char* name = nullptr);         // + Flywheel
template <class Declared> requires requires(const Declared& d) { d.name.text; }
Routine& waitUntilDone(const Declared& m, units::Time bound, const char* name = nullptr);
[[nodiscard]] const mechanism::StepReport& lastMechanism() const noexcept;
// THE WRONG STEP FOR THE KIND, refused BY NAME where the team calls it (a static_assert in a constrained template):
template <class D, class... Rest> requires(/* Arm, Lift, Flywheel, Pneumatic */) Routine& run(const D&, Rest&&...);   // "r.run(...) is for a POWER mechanism -- ..."
template <class V, class... Rest> requires std::is_arithmetic_v<V> Routine& run(const mechanism::Power&, V, Rest&&...);  // "volts are written with their unit: 12_volt, not 12"
template <class D, class... Rest> requires(/* Arm, Lift, Power, Flywheel: a declaration where a target belongs */) Routine& move(const D&, Rest&&...);   // "r.move(...) takes a TARGET: arm["score"], ..."
template <class D, class... Rest> requires(/* Arm, Lift, Pneumatic */) Routine& stop(const D&, Rest&&...);    // "r.stop(...) is for a power mechanism or a flywheel -- ..."
// and the auton's words (beside the mechanism words, so an auton needs no library namespace):
namespace shulib::mechanism::words { using chassis::Routine; using chassis::RoutineResult; }
```
The verdict policy (each pinned): SUCCEEDED continues; UNCONFIRMED **counted done** (no sensor, Q2 (c))
continues; every other verdict STOPS the chain as **`MechanismStepFailed`** with the verdict in the stop line, in
`mechanismVerdict` and in `lastMechanism()`; a START form stops the chain only if the start itself is ABSENT /
REFUSED / NOT_HOMED; a step on a mechanism this robot's list does not have is ABSENT, said (`'claw' is not on this
robot (its wiring list does not have it): the step did nothing`), and stops the chain; a blocking wait whose
service NEVER ticked stops the chain with "the mechanism service never ticked -- it is not in the chassis
pacer's chain" (P1's control). The wait's backstop is the bound + 0.25 s; the service's own deadline ends the
step. A mechanism step on a Routine built from `Chassis&` (the frozen constructor) is a loud precondition naming
the fix (`construct it as Routine r{ctx, "name"}`) — which is why A5 changes `src/team/autons.cpp` :53 (U3).
`routine.hpp` includes `mechanism/service.hpp` (S1: the whole suite rebuilt with it). **The two `src/` lines A4
writes:** the new enumerator's word in `src/programs/competition_run.hpp` :559 and `auton_tester_run.hpp` :271
(`case chassis::RoutineStopCause::MechanismStepFailed: return "mechanism step FAILED";` — their `-Wswitch` would
otherwise fail the `src` gate; `proposed4_src/`), and `test/competition_run_test.cpp` :1369-1370 gains its word.

**`competition/auton_registry.hpp`** — `AutonContext` APPENDS (COMP-a, not frozen; positional
`AutonContext{chassis, guard, telemetry, mechanisms, entry}` still compiles, finding 7):
```cpp
namespace shulib::mechanism { class MechanismService; }   // only a pointer is held here
/// MECH: the robot's mechanism service -- THE LOOKUP an auton reaches its mechanisms through
/// (`Routine r{ctx, "name"}` takes it). nullptr in a program that runs autons without one.
mechanism::MechanismService* mechanismService = nullptr;
// and, beside the routine's words: namespace shulib::mechanism::words { using competition::AutonContext; }
```
and the comment at :221 ("For the guard's safing only (B25): a routine that commands a mechanism looks it up by
name from the Robot") is rewritten: an auton reaches its mechanisms through `mechanismService`, by declaration.

**`competition/match_runner.hpp`** — the body moves to `detail::runAutonImpl(..., mechanism::MechanismService*
service, ...)`; COMP-a's `runAuton(chassis, guard, entry, commands, telemetry, std::span<hal::IMechanism* const>
mechanisms, period)` forwards with `nullptr`, unchanged; ADDED:
```cpp
[[nodiscard]] inline AutonRunResult runAuton(chassis::Chassis& chassis, sequence::RunGuard& guard,
                                             const AutonEntry& entry, const runner::CommandRegistry* commands,
                                             hal::ITelemetrySink& telemetry, mechanism::MechanismService& mechanisms,
                                             units::Time period);   // the guard's span = mechanisms.mechanisms();
                                                                    // calls mechanisms.enterAutonomous() first (D1)
```
**Run while driving** is the START forms plus the service in the chassis's pacer chain (P1: the intake stopped
on its sensor mid-drive; the lift rose during the drive; the arm held). The service is ticked ONLY inside a
shulib motion, wait or the driver loop. **A student's own busy loop starves it, and a starved service leaves each
mechanism at its LAST command — not held, not limited, not safe** (D11, case I: 11.74 V held 1.5 s, 0.89 s against
the hard stop at 2.45 A) — the limit `RunGuard` already states ("nothing preempts pure user code"); the guide and
the cookbook say: in an auton use `r.pause()` / `waitFor`, never `pros::delay`, and a `then()` action must not
block.

### 1.10 The driver file's controls and the loop (MECH-A4)

**`include/shulib/mechanism/driver_inputs.hpp`** — NEW. Verbatim surface:
```cpp
enum class ControlKind : std::uint8_t { Hold, Toggle, Preset, HomeOnPress, HomeAtDriverStart, AutonOnly };
struct Control { ControlKind kind; Kind mechanismKind; const void* mechanism; const char* mechanismName;
                 hal::ControllerButton button, back; double volts, bound, value; std::uint8_t position; const char* presetName; };
[[nodiscard]] constexpr Control whileHeld(const Power& m, hal::ControllerButton forward, hal::ControllerButton back, units::Voltage volts) noexcept;
[[nodiscard]] constexpr Control whileHeld(const Arm& m, hal::ControllerButton up, hal::ControllerButton down, units::Voltage volts) noexcept;
[[nodiscard]] constexpr Control whileHeld(const Lift& m, hal::ControllerButton up, hal::ControllerButton down, units::Voltage volts) noexcept;
template <class D> requires(std::same_as<D, Pneumatic> || std::same_as<D, Flywheel>)
consteval Control whileHeld(const D&, hal::ControllerButton, hal::ControllerButton, units::Voltage);   // refused BY NAME (n24)
[[nodiscard]] constexpr Control toggle(const Pneumatic& m, hal::ControllerButton b) noexcept;   // two positions
[[nodiscard]] constexpr Control toggle(SpeedGoal g, hal::ControllerButton b) noexcept;          // `toggle(flywheel["shoot"], X)`
template <class D> requires std::same_as<D, Flywheel>
consteval Control toggle(const D&, hal::ControllerButton);   // `toggle(flywheel, X)`: name its speed (n51)
template <class D> requires(std::same_as<D, Power> || std::same_as<D, Arm> || std::same_as<D, Lift>)
consteval Control toggle(const D&, hal::ControllerButton);   // refused BY NAME
[[nodiscard]] constexpr Control preset(PositionGoal g, hal::ControllerButton b, units::Time bound) noexcept;
[[nodiscard]] constexpr Control preset(SpeedGoal g, hal::ControllerButton b, units::Time bound) noexcept;
[[nodiscard]] constexpr Control preset(PneumaticGoal g, hal::ControllerButton b) noexcept;
template <class PositionKind> [[nodiscard]] constexpr Control homeOnPress(const PositionKind& m, hal::ControllerButton b, units::Time bound) noexcept;
template <class PositionKind> [[nodiscard]] constexpr Control homeAtDriverStart(const PositionKind& m, units::Time bound) noexcept;
template <class Declared> [[nodiscard]] constexpr Control autonOnly(const Declared& m) noexcept;
struct DriverControls { static constexpr std::size_t kCap = 24; Control c[kCap]{}; std::uint8_t count = 0;
    [[nodiscard]] constexpr std::span<const Control> all() const noexcept;
    [[nodiscard]] constexpr bool avoids(hal::ControllerButton b) const noexcept; };   // the re-zero
template <class... C> [[nodiscard]] consteval DriverControls controls(const MechanismTable& robot, C... list);
class DriverInputs {   // static storage, like T3's bindings
public:
    void bind(MechanismService& svc, const DriverControls& controls);  // initialize(): after the service is bound
    [[nodiscard]] bool avoids(hal::ControllerButton b) const noexcept;  [[nodiscard]] bool bound() const noexcept;
    void enter();                                      // loop entry: every level unknown -- a press must be SEEN
    void tick(const hal::IController& c, bool connected);   // read, and command the service
};
namespace words { using mechanism::DriverControls; using mechanism::controls; using mechanism::whileHeld; using mechanism::toggle;
                  using mechanism::preset; using mechanism::homeOnPress; using mechanism::homeAtDriverStart; using mechanism::autonOnly; }
// teleop/teleop_loop.hpp adds the driver file's words to the same namespace:
namespace shulib::mechanism::words { using Button = hal::ControllerButton; using teleop::DriverProfile;
    using teleop::DriverLoopConfig; inline constexpr teleop::DriveMode ArcadeBody = …, ArcadeField = …; }
```
`controls(...)` refuses AT BUILD (P5; 17 reasons, a button twice NAMED — `button_Right_does_two_jobs`): a control
naming a mechanism this robot does not have; a button doing two jobs; hold volts outside (0, 12]; a preset or
homing with no time limit; two controls of one kind on one mechanism (a hold-to-run beside presets is fine — Q8); a
toggle on anything but a two-position pneumatic or a flywheel's named speed; homing on a never-homed mechanism; **a
hold-to-run or preset on a switch- or stall-homed mechanism with no way to home it in driver control —
`homeAtDriverStart`, `homeOnPress` or the declaration's `.unhomedVolts` (U5, n48: its buttons would be dead after
any restart off its home); a hold-to-run on a never-homed mechanism with no `.unhomedVolts` (n49); a preset on a
never-homed one (n50)**; a mechanism with no control unless marked `autonOnly`, and one marked `autonOnly` that also
has a control; more than 24 controls. Arbitration, Q8 (a), by the slot's command number: an input that sees a
number it did not write is SUPERSEDED and commands nothing until its button is seen released (T3's "an outside stop
is never undone by a finger that never left the button", generalized — P5d; a stall trip bumps the number too,
N10). HOLD-TO-RUN on a power mechanism: `drive(±V)`, release → stop (safe); on an arm or lift: `jog(±V)`, release
→ HOLD HERE (clamped to the range); both buttons = neither (T3's R2). `homeAtDriverStart` homes once at entry when
nothing homed it (P2i). A controller that drops: every hold-to-run released that tick.

**`src/common/driver_file.hpp`'s `DriverFile`** (N6, A5; `team5/driver_file_core.hpp`) carries what is not a team
decision: the loop's tick is the BRAIN's (`ProsTickPacer::kTickMs`), supplied — a driver file states no
`.tickPeriod`, and one that states another is refused (`static_assert`, n56 — today's sixth check, kept: the
shaper's dt and T4 Part 2a's 250 ms / 1 s protection windows depend on it, `teleop_loop.hpp` :307, :319); and a
control on the re-zero button is refused (n57).

**`teleop/teleop_loop.hpp`** — the body becomes ONE `detail::driverLoop(...)` taking the T3 spans AND an optional
service + inputs; every public overload forwards to it; ADDED (verbatim):
```cpp
template <typename IsDisabled, typename HeadingSource, typename PerTick>
void runDriverControl(chassis::Chassis& chassis, const hal::IController& controller,
                      motion::ITickPacer& pacer, DriveShaper& shaper, const DriverLoopConfig& cfg,
                      mechanism::MechanismService& mechanisms, mechanism::DriverInputs& inputs,
                      IsDisabled&& disabled, HeadingSource&& heading, PerTick&& perTick);
```
with two preconditions (the service and its BOUND inputs come together and replace the T3 spans — one driver
path per mechanism; no control takes the re-zero button), and three places in the tick: at ENTRY,
`mechanisms.enterDriverControl()` then `inputs.enter()` (a released leftover claim is logged, as today); at a
DISABLE, `mechanisms.leaveDriverControl()` beside the bindings' `makeSafe()` (motor kinds safe; pneumatics keep
their level); each tick, after the bindings and toggles read their buttons and BEFORE `perTick`,
`inputs.tick(controller, connected)` then `mechanisms.tickFromDriverLoop()` (requirements :134-135). T4 Part 2a's
protections are untouched (they wrap the drive, not the mechanisms). The `ONE TICK, IN THIS ORDER` banner (:18-59)
gains step 6½.

### 1.11 `include/shulib/mechanism/direction_check.hpp` — NEW: the judgement (MECH-A4; its page is MECH-A6)

Pure, host-tested, PROS-free. What the encoders can see (a motor whose encoder moves AGAINST its command — its
mate drags it; a motor that neither moved nor drew; every motor still while every one draws — two declared
against each other, or a jam; an angle sensor reading against the motors) and what only a person can say (which
mechanism moved, and which way). It REPORTS and names the place to change; it cannot save a flip (Q1 (a)).
Verbatim (re-check D6: THREE answers, the third naming the WIRING line):
```cpp
enum class PersonSaw : std::uint8_t { NotAsked = 0, ItsPlusWay, TheOtherWay, NothingOrSomethingElseMoved };
struct DirectionObservation { double commandVolts = 0.0; double motorDeltaRad[kMaxMotorRoles]{};
    double motorPeakAmps[kMaxMotorRoles]{}; bool hasAngleSensor = false; double sensorDeltaRad = 0.0;
    PersonSaw person = PersonSaw::NotAsked; };
enum class DirectionFinding : std::uint8_t { Ok, DidNotMove, MotorFights, FightOrJam, SensorReversed, MechanismReversed,
                                             CableOnAnotherMechanism, AwaitingPerson };
inline void changeHere(char* out, std::size_t cap, const char* source, const char* file, std::uint32_t line, const MechanismSpec& s);
inline void wireHere(char* out, std::size_t cap, const MechanismWiring& w, const MechanismSpec& s);   // its wire(...) line
struct DirectionVerdict { DirectionFinding finding = DirectionFinding::AwaitingPerson; std::uint8_t role = 0; char fix[384]{}; };
[[nodiscard]] inline DirectionVerdict judgeDirection(const MechanismSpec& s, const MechanismWiring& w,
                                                     const DirectionObservation& o, double minMoveRad, double drawAmps);
inline constexpr std::uint8_t kNothingOrSomethingElseMoved = 0xFF;   // a pneumatic's third answer
[[nodiscard]] inline DirectionVerdict judgePneumatic(const MechanismSpec& s, const MechanismWiring& w,
                                                     std::uint8_t commanded, std::uint8_t sawPosition);
```
"Change it HERE" (G3a's re-check item 3): the role's `changeAt` (a generated robot file's JSON path) when stated,
else the file and line the declaration was written on (captured), else the declaration's line; a cable on the
wrong mechanism is fixed at its `wire(...)` line (captured where `motors(...)` / `lines(...)` is written). So the
hand-written robot two needs NO string typed by hand. Verbatim (`out5/probe3_direction.txt`):
- `'lift' motor 2 turns AGAINST its command (its mate drags it): in the declaration at team5/robot_x.hpp:28,
  .motors: make motor 2 Forward, then rebuild and upload` (generated: `at src/robots/x.robot.json: mechanisms >
  "lift" > roles > "motor 2" > reversed make it false, then regenerate, rebuild and upload`);
- `'lift' did not move but its motors drew current: they FIGHT (one is declared the wrong way -- check each
  Forward/Reversed at team5/robot_x.hpp:28) or it is JAMMED / at a stop`;
- `'lift' motor 2 neither moved nor drew current: check its cable, and its port in the wire(...) line at
  team5/robot_x.hpp:76`;
- `'arm' moved the OTHER way: in the declaration at team5/robot_x.hpp:14, .motors: swap EVERY Forward and Reversed
  (and the angle sensor's), then rebuild and upload`;
- `'arm' did not move although its motors turned: a cable of 'arm' is on ANOTHER mechanism's port (a swap the boot
  cannot see) -- check the wire(...) line at team5/robot_x.hpp:75 and the wire line of what moved`;
- `'claw' went 'closed' when told 'open': its line levels are swapped -- in the declaration at
  team5/robot_two.hpp:24, .positions: swap each position's levels, then rebuild and upload`;
- `'claw' did not move when told 'open': its valve is not on the line the wiring says (a 3-wire swap, or an
  unplugged or empty valve -- the boot cannot see either): check the wire(...) line at team5/robot_two.hpp:36, the
  tubing and the air`.
`minMoveRad` and `drawAmps` are the page's STATED thresholds (HA-182's page entries). The page says "ONLY '<name>'
should move" before it asks.

### 1.12 `include/shulib/sim/mechanism_plant.hpp` — NEW: the arm, lift and flywheel models (MECH-A2 base, MECH-A3 tuned)

In the A2 plant's own style — no mass, no torque curve: the INVERTED feedforward relation with a gravity LOAD in
volts (`V_eff = V − G(θ)`, `G = kG·cos(θ − level)` for an arm, `kG` for a lift), a static band, a first-order
approach (τ = kA/kV, exact exponential step), plus what a drive wheel lacks: HARD STOPS (zero speed and
stall-grade current pushing into one), an OBSTACLE that appears mid-run, and the BRAKE MODE at 0 V (Hold pulls back
to where it stopped, Brake damps, Coast is free — invented stiffness). Current `I = |V − kV·ω|·(stallAmps/12)`,
clamped at the stall current (2.5 A at a 12 V stall, HA-93's shape). `ArmPlant` (one joint, N fake motors
fanned like `MotorMechanism`; a lift is an arm with a constant load through a spool) with `addLoadVolts`,
`setObstacleDeg`, `attachRotation`; `FlywheelPlant` with `shot(fraction)`. **The controller's constants and the
plant's are set separately on purpose** — every adversarial case mismatches them (the A2 truth-integrator
lesson): a test that gives both the same kG proves only the algebra. Every constant invented: HA-183.

### 1.13 `src/` — robot two's tables, the driver file, the programs, the tester rig, the gate (MECH-A5)

Against the tree as T4 Part 3 leaves it (the buttons already left the table; Teleop is retired; "shulib Drive",
the Auton Tester and the competition program all run the driver file or the autons). Calibrate is CAL1's (N9).
- **`src/robots/mechanism_table.hpp`** — rewritten (`team5/mechanism_table_core.hpp` is its code, P10): the banner
  says what the file now holds (each robot's mechanisms as declarations + wiring, in that robot's namespace; the
  claw's open level is a guess written ON its positions line); `using mechanism::MechanismTable;` (the LIBRARY's
  type name, so the `src` gate's step-0 regexes `constexpr MechanismTable <name>` / `constexpr const
  MechanismTable& <fn>(RobotId …)` match with no alias, P11); robot two's region exactly as §1.1, in `namespace
  robot_two`; `kNoMechanisms = mechanism::mechanisms();`; `mechanismTableFor()` kept as a case per robot with NO
  default (G3a item 1 — Part 2's `[mechanism selection]` markers go inside it; the core adds no second per-robot
  selection function); `kMechanisms`; `mechanismsClearOf(const MechanismTable&, const ChassisTable&)` — today's
  check 4 (:395-402) EXTENDED from motors to every smart role (motors, angle sensors, piece sensors), both drive
  sides and the IMU (renamed from `mechanismsDisjointFromChassis`, §6.3); the pins of P10 (§6.2 items 4 and 6: the
  address, no counts; the bench bot and the X-drive by name). REMOVED: the entry types, `PneumaticPosition`, the old
  `kMaxMechanisms = 6` and `kMaxPneumatics`, `kClawOpenCommand` (the guess moves onto the claw's `.positions` line;
  its 12-line note becomes the comment above it), checks 1-3 and 5 (the build checks them; provenance becomes a
  comment — Decided here), `pneumaticCommandFor/StartCommand/SafeCommand`, `motorEntryCount` / `pneumaticEntryCount`
  (their users — `competition_program.cpp` :759, `drive_program.cpp` :583, `driver_panel.hpp` :239-299,
  `boot_source.hpp` :136-566 — move to the list; `absPort` is `chassis_table.hpp`'s and stays), and the comments naming `driver_controls.hpp` (:120, :139, :213, :417). KEPT,
  re-implemented over the new list: `isDeclaredMechanismPort(t, port)` (the boot's declared-port rule). **Calibrate's
  drop (`dropDeclaredMechanismPorts`, :436-489) is CAL1's**: CAL1 Part B deletes it (CAL1 §1.8) and maps the table
  in `src/robots/calibrate_facts.hpp`; A5 rewrites that mapping over the new list (N9) — or, if A5 lands first,
  re-points today's call (`calibrate_program.cpp` :1129-1130) at the new list, one line.
- **`src/common/boot_source.{hpp,cpp}`** — `bootConfigFrom` fills `c.mechanisms[i] =
  mechanism::toMechanismConfig(list.specs[i], list.wiring[i])` in the LIST's order (robot two's order is today's:
  pivot 6, pivot 7, claw — P9a); `mechanismTableProblem` (:129), both `toMechanismConfig` (:201-217) and
  `mechanismConfigIndexOfPneumatic` (:158) are removed (the build checks the list; an index IS the list's); the
  declared-port rule of `decideBootSourceFrom` (:459-471: a record that drives a declared mechanism port refuses
  the boot, "recalibrate") reads `declaredMotorPorts(list)` and now also the smart SENSOR ports. Its two
  `static_assert`s (:152-155) go (the list's own checks replace them). SAFE-SAVE B2 edits this file's record READ
  and L9 Part B moves `decideBootSourceFrom` into the library (§6.1) — different functions; whichever is second
  rebases, and L9 Part B's derivation of declared ports must include the sensor ports.
- **N6: `src/common/driver_file.hpp` NEW** (`team5/driver_file_core.hpp` is it: `DriverFile<Controls, Feel, Loop>`
  with `bind(MechanismService&)` and `run(robot, mechanisms, pacer, disabled, perTick)`, the controller, the shaper
  and the inputs as members; the brain's tick supplied and a stated one checked; the re-zero `static_assert`).
  **`src/common/driver_heading.hpp` NEW** — `driverHeading()` moved unchanged (the X-drive uses it and has no
  driver file; T6). `src/common/driver_controls.hpp` is RETIRED (its types replaced by
  `mechanism::DriverControls`/`DriverInputs`, its five predicates by `controls(...)`'s build checks); every user
  re-pointed: `drive_program.cpp` :91, `xdrive_program.cpp` :54 and :237 (→ `driver_heading.hpp` — so the X-drive
  program is no longer untouched), `driver_panel.hpp` :75 and :128, `robot_two_driver.hpp` :43;
  `src/team/robot_two_driver.hpp` becomes §1.1's block under today's button-map comment (whose words stay true, plus
  one line for presets); `src/team/README.md` :10 re-worded. Under (b) of N6 the class stays in the driver file with
  the same members.
- **`src/team/autons.cpp`** — :53 `chassis::Routine r{ctx.chassis, …}` → `Routine r{ctx, "40in-forward"}` (U3: the
  only spelling the guide teaches; the frozen constructor would make every copied auton's first mechanism step a
  precondition); `#include "robots/mechanism_table.hpp"` and `using namespace robot_two;` for the steps to come.
- **The programs.** Each builds ONE `mechanism::MechanismService` in static storage, constructed in the pacer
  chain BEFORE the Robot and bound AFTER it (`service.bind(kMechanisms, robot.mechanismParts().devices(),
  robot.context().clock(), robot.faults(), sink)` — unbound it ticks nothing):
  - `competition_program.cpp` — the chain (:640-644) becomes watch → service → guard: `st.watch.emplace(
    devices->pacer(), …); st.mechanisms.emplace(*st.watch); st.guard.emplace(*st.mechanisms);` — so
    Chassis → RunGuard → MechanismService → CompetitionWatch → tick pacer; the auton request calls the `runAuton`
    service overload (`competition_run.hpp` ~:436) with **the route registry** (below); `opcontrol()` calls
    `driver.run(robot, *st.mechanisms, devices->pacer(), …)` (the driver loop ticks the service itself; its pacer
    stays the raw one, :28-29).
  - `auton_tester_program.cpp` — the same insertion between the guard and the AbortWatch (:468-473), and the
    registry.
  - **The route registry (N13, §1.16)** — in the competition program and the Auton Tester: one
    `runner::CommandRegistry` and one `runner::MechanismRoutes` in static storage, `routes.registerInto(registry)`
    at boot after the service binds (inside the boot's try), `&registry` passed to `runAuton` (route autons fire the
    generated ids; a team's own `registry.on(...)` lines come after, sharing the 32). Boot line: `ROUTES: 6 mechanism
    route ids published (claw_pivot_6_run … claw_open)`.
  - `src/common/guarded_run.hpp` — `pacerChainProven(chassis, guard, watch)` gains the service: one idle tick
    must pace the guard, the service AND the watch (`service.ticks()` advanced), or the boot is refused naming
    the missing layer — the proof that P1's control case (the service left out of the chain) cannot ship.
    And `stopAfterProgramError()` (:84-99), after safing every PRESENT mechanism, also calls each one's
    registered claimant's `cancel()` (the service's slot) — so a slot that held a behaviour when the throw came
    is inert and cannot re-command at a later tick (the "half-safe" failure, `mechanism.hpp`).
  - `drive_program.cpp` ("shulib Drive") — no guard: the service is constructed around the raw pacer and is
    ticked by the driver loop only; `driver.run(robot, mechanisms, pacer, …)` (:695).
  - **Boot lines, all three programs**: one `[MECH]` line per ROLE through `mechanism::roleLine` (P4's lines),
    then one per CONTROL (`claw pivot 6: hold-to-run R1 / R2, 12 V`; `arm: preset 'score' on Right`), one per
    position mechanism naming its HOMED state (`lift: NOT HOMED -- its buttons do nothing until it is homed
    (homeAtDriverStart)`, U5), one per Power with no stall signature (`'claw pivot 6': no stall signature -- held
    against its stop it is cut only by the driver letting go (HA-154)`, N10), and one per position or speed
    mechanism naming its STARTING values (`arm: starting values kP 0.25 kI 0.30 kD 0.004 kV 0.050, cruise 180 deg/s,
    reached within 1.5 deg for 0.10 s -- PROVISIONAL (HA-182); .tuning overrides` — N7), replacing
    `motorClawBootLine`/`pneumaticClawBootLine` (`driver_panel.hpp` :152-190; drive :583-592; competition
    :752-758). The claw ROW (`clawRowText`, :277) reads the service (a pneumatic's commanded position, a motor
    mechanism's state) and FREED reads `service.leftoverClaimsReleased()`; its width `static_assert`s stay
    against `kRobotTwoMechanisms` inside `static_assert` (allowed by step 0).
  - `xdrive_program.cpp` — only its include (`driver_heading.hpp`); `src/main.cpp` (Part 3's final form) untouched.
- **The tester rig and the auton tests** (U3) — `test/auton_tester_rig.hpp` (:247-262) builds a
  `MechanismService` in its chain (guard → service → watch) bound to the build's list over the §1.12 models, and the
  route registry; `runTesterOnce` takes the service overload; `test/autons_test.cpp` (:143-168, "EVERY registered
  auton") then runs every registered auton with its mechanism steps live, and gains a case that registers a claws
  auton AND a claws route and asserts each step's VERDICT (not only "ended safe") — so a step that can never be met
  (D7) or a marker with bad args fails `ctest` before any upload.
- **`tools/src_build_gate.py`** — step 0 per ROBOT (T3; decided here; `tools5/gate_patch.py` prototyped it): a new
  `_robot_namespaces(repo)` (each namespace of the mechanism table file that declares a `MechanismTable`) and
  `_robot_namespace_problems(repo, rel)` (refuses `<ns>::`, `using namespace <ns>` and re-opening `namespace …<ns>`
  outside the table files, `test/`, a `static_assert` and that robot's own team files — `src/team/<ns>_*`, or the
  reviewed `TEAM_FILE_ROBOT` map, today `src/team/autons.{hpp,cpp}` → `robot_two`, until G3a Part 4's per-robot
  auton files); `_stray_selection_problems` (:734-758) lets a robot's own team file name its OWN mechanism list and
  nothing else. Self-test: case 28's chassis-table-in-`src/team/` refusal still holds, and NEW cases pin
  `out5/gate_cases.txt`'s nine (a program naming `robot_two::claw`, a program's `using namespace robot_two`,
  `src/common` re-opening the namespace, robot three's driver file naming robot two's list qualified and unqualified,
  a chassis table in a team file — each refused; robot two's own second team file, the auton file via
  `TEAM_FILE_ROBOT`, a `static_assert` in a program — each passing).
- **The tests over the retired header** (T6): `test/driver_controls_test.cpp` splits — its heading cases (:542-574)
  move to `test/driver_heading_test.cpp`, its `HoldControl` / `ToggleControl` / `DriverBindings` cases are DELETED
  with the types (their rules are `controls(...)`'s, tested in A4 and `test/compile_refusals/`);
  `test/drive_program_source_test.cpp` :282's pin names `driver_file.hpp`; `test/robot_two_driver_test.cpp` and
  `test/driver_panel_claws_test.cpp` re-point (§3.5).
- **HA-159's text and Station J.9** (`docs/internal/chunks/R3a-BENCH-WORKSHEET.md`): "flip `kClawOpenCommand`"
  becomes "swap `Low` and `High` on the claw's `.positions` line"; HA-185 registered (N3).

### 1.14 Bench Tests station 13 MECHANISMS and the direction check page (MECH-A6)
- **The menu** becomes 3 × 5 (N5 (a)); stations 1–12 keep their numbers, their logged names and their places.
- **What it builds.** `robot::MechanismParts` over the `ProsDeviceProvider` from `kMechanisms` (projected by
  `toMechanismConfig`) — no Robot, no drivetrain, no service (Bench Tests builds no Robot today; a mechanism's
  direction must be checkable before it is homed, so the station commands the hardware directly, as station 10
  does — its own "ONLY code path that calls setVoltage" note grows to two). Each role's boot line is shown first
  (P4's words); an ABSENT mechanism's page explains why and offers nothing powered.
- **Live readings** per mechanism, painted on change at most 5 Hz (the PROS print-leak lesson, `bench_r3a.cpp`
  ~:254): each motor's position, velocity, current and temperature (temperature "unknown" when its reads have
  failed, never "cool"); **each position mechanism's reading IN ITS DECLARED UNITS, from power-up** — degrees
  through its gearing, inches through its shape (U1: a wrong gearing is then a wrong angle on the screen, and the
  team reads `.range`, the presets and the level angle off the robot by moving the arm by hand); the angle sensor;
  each switch's level — **pressed, released AND with its cable out** (the screen asks for all three: HA-184's
  settle); the piece sensor's reading and "sees a piece" (or "not answering" when its read-fault count moves).
- **Powered, behind station 10's six gates adapted** (`bench_r3a.cpp` :1317-1360, every threshold stated as
  invented on screen): (1) REFUSE unless the mechanism was built; (2) a two-stage CLEAR confirm ("nothing in its
  path, hands away" — and for a pneumatic, "FIRE 'claw' → 'open'? hands clear") before the first volt or the
  first line in a power cycle; (3) DEAD-MAN: L1 held — release or a controller drop puts every mechanism in its
  safe state that tick; (4) a volts ceiling starting at 3 V, R1 steps 3 → 6 → 9 → 12; (5) cut-outs: two motors of
  one mechanism disagreeing (the shared coupled-side evaluator station 10 uses), any member above 2.4 A for
  250 ms, the mechanism's own stall signature when it has one, and a jog's stated maximum time — each cuts THAT
  mechanism to its safe state, named, until re-entry; (6) EXIT → every mechanism safe, an RAII guard on every way out.
- **The direction check** (Q30; Q1 (a); D6): the page says "ONLY '<name>' should move", then for a motor mechanism
  one short "+" jog at the ceiling for a stated time (HA-182) records each motor's encoder change and peak current
  and the angle sensor's → `judgeDirection`; a motor dragged against its command, a motor that neither moved nor
  drew, every motor still while drawing (a fight of equals or a jam), or a sensor against its motors is reported at
  once; otherwise it asks **"did '<name>' move its + way / the other way / it did NOT move or something ELSE moved"**
  and shows the verdict, naming the robot-file line — the declaration's for a direction, the `wire(...)` line for a
  cable on another mechanism's port (§1.11). For a pneumatic: fire a position (gate 2), ask "which position did it
  go to?" (its position names as buttons, plus "nothing / something else moved") → `judgePneumatic`. Nothing is
  saved; the screen says "edit the robot file, rebuild, upload". RECORD writes `[R3A] mech` lines like the other
  stations (the verdict included), to serial and the SD log. **Station 13 also settles HA-184** (a switch read
  pressed, released and unplugged) and **HA-186** (an absolute sensor read at two marked angles after a POWER cycle
  and after a PROGRAM restart).

### 1.15 PTO-ready, as ruled (Q4 (b)): nothing built; the places a later PTO changes, named
"A port is never shared by the drive and a mechanism" is enforced in exactly these places, and nothing else in
the core depends on it: (1) `RobotConfig::valid()`'s one-owner claim (`config/robot_config.hpp` :585, and the
`MechanismSensor` claims the core adds); (2) G3a's generator ("a claw port that is a drive port"); (3) the
VexBuilder contract §3.2 ("may not share a port with the drivetrain"); and, in the core's own code, (4) the robot
table file's build check `mechanismsClearOf` and `wiringProblem`'s drive-port refusal — compile-time mirrors of
(1); plus (5) the motor group, which has no ownership hand-over (`markAbsent` is permanent and keeps commanding).
A later PTO chunk changes those five; §9 drafts the plan's record of them.


### 1.16 Mechanism ROUTE IDS — every mechanism's actions, published automatically (his ruling of 2026-10-08; MECH-A1 the derivation, MECH-A4 the binder, MECH-A5 the programs)

The ruling (MECH-PANEL-requirements §4 item 7b :515-517 at `2fce38a`; REMAINING-WORK row 12): "every mechanism
publishes its actions as route ids AUTOMATICALLY (an arm's presets as `arm_stow`, `arm_score`; an intake's
`intake_run`, `intake_until_piece`), in shulib's manifest that VexBuilder's marker picker reads — a coder no longer
registers each id by hand". The panel's route editor (G3a Part R) drops these ids on a route and refuses one the
robot does not have; nothing here simulates a route (the panel's preview was reversed the same day, `3c643f1`).

**`include/shulib/mechanism/route_ids.hpp` — NEW (A1; constexpr, PROS-free).** THE RULE, one per kind (the stem is
the mechanism's `.name` in route-id words — lower case, every run of other characters one `_`: "claw pivot 6" →
`claw_pivot_6`):

| kind | its ids | a marker's args |
|---|---|---|
| Arm, Lift | `<stem>_<preset>` per preset (move there, hold); `<stem>_home` when it has a homing method | `[time limit ms]` |
| Flywheel | `<stem>_<preset>` per preset (spin up, at speed, keep spinning); `<stem>_stop` | `[time limit ms]`; none |
| Power | `<stem>_run`; `<stem>_until_piece` and `<stem>_until_gone` when it declares a piece sensor; `<stem>_stop` | `[volts, ms]`; `[volts, time limit ms]`; none |
| Pneumatic | `<stem>_<position>` per position | none (its move time is the robot file's) |

Verbatim surface (`proposed4/shulib/mechanism/route_ids.hpp`):
```cpp
enum class RouteAction : std::uint8_t { GoToPreset = 1, Home, SpinUp, Run, RunUntilPiece, RunUntilGone, Stop, SetPosition };
inline constexpr std::size_t kMaxRouteIds = 32;   // = runner::kMaxCommands, F8's design constant
struct RouteId { char id[config::kMaxMarkerIdBytes]{}; std::uint8_t mechanism = 0; RouteAction action = RouteAction::Stop;
                 std::uint8_t index = 0; std::uint8_t argCount = 0;
                 [[nodiscard]] constexpr std::string_view view() const noexcept; };
enum class RouteIdProblem : std::uint8_t { None = 0, TooLong, Duplicate, Builtin, TooMany };
struct RouteIds { RouteId ids[kMaxRouteIds]{}; std::uint8_t count = 0; RouteIdProblem problem = RouteIdProblem::None;
                  char offending[48]{}; };
[[nodiscard]] constexpr RouteIds routeIdsOf(std::span<const MechanismSpec> specs) noexcept;   // THE RULE
[[nodiscard]] constexpr const char* routeIdProblemText(RouteIdProblem p) noexcept;              // the sentence
namespace refused { /* a_route_id_does_not_fit_23_characters_shorten_the_mechanism_or_preset_name,
                       two_route_ids_are_the_same_rename_a_mechanism_a_preset_or_a_position,
                       a_route_id_is_one_of_the_librarys_own_wait_ms_brake_hold_ms_log,
                       more_route_ids_than_a_route_registry_holds_32 */ }
consteval void refuseRouteIds(RouteIdProblem p);
```
`mechanisms(...)` (§1.4) calls `refuseRouteIds(routeIdsOf(list).problem)`: an id that would break F8's vocabulary
(1–23 bytes of `[a-z0-9_]`), two ids alike (a preset named "home" beside a homing method; "claw pivot" and
"claw_pivot"), an id that is a built-in (`wait_ms`, `brake`, `hold_ms`, `log`), or more than 32 ids each FAILS THE
BUILD at the robot's list (n60–n63) — never a silently missing id. Robot x publishes 17, robot two 6 (§1.1).

**`include/shulib/runner/mechanism_routes.hpp` — NEW (A4; in `runner/` because it needs the Routine and the
registry, and `mechanism/` stays below `chassis/`).** Verbatim:
```cpp
class MechanismRoutes {   // static storage beside its registry (the registry keeps pointers into it)
public:
    MechanismRoutes(chassis::Chassis& chassis, mechanism::MechanismService& service,
                    const mechanism::MechanismTable& robot, hal::ITelemetrySink& telemetry) noexcept;
    void registerInto(CommandRegistry& registry);    // ONE call: every id, in the list's order, each with its doc
    [[nodiscard]] const mechanism::RouteIds& ids() const noexcept;
};
```
A marker runs EXACTLY the step an auton's one line runs, through a `chassis::Routine` built on the same
chassis and service (one verdict policy, not two): it BLOCKS at its waypoint until the step's verdict within the
marker's time limit (markers fire only after a leg settles, F8's ruling R2 — a step started mid-leg is the rest's
"triggers partway through a drive"); SUCCEEDED and UNCONFIRMED-counted continue the route; every other verdict
returns to `PathRunner`, whose policy stops the route (`MarkerFailed`, the verdict in its line). A marker with the
wrong args — no time limit, a time of 0, volts 0 or outside ±12 — is DATA a route author wrote: REFUSED with one
Warn naming the id and what it needs (`route marker 'arm_score' REFUSED: it needs its time limit in ms, above 0
(args[0])`), and the route stops. A flywheel's spin and an arm's hold go on after their marker (the service keeps
them); the guard's end of run makes every mechanism safe.

**`runner/command_registry.hpp` + `runner/manifest.hpp` — F8-ADDITIVE (A4; "new registry members", "new manifest
fields" under `kApiMinor`):** `CommandEntry` APPENDS `const char* doc = nullptr;` (non-owning) and `CommandRegistry`
gains `void on(std::string_view id, Handler handler, std::uint8_t argCount, const char* doc)` (a doc free of `"` and
`\`, the manifest's rule); `writeManifest` prints a registered id's doc where it printed `""` — a registry without
docs writes the same bytes as before (S1: `test/manifest_test.cpp`'s byte-exact golden and
`test/f8_signature_pin_test.cpp` pass unchanged). The 2.5 changelog and the F8 register row say so; A4 adds pins for
the new member and overload. Robot two's manifest lines, verbatim (P14):
```
{"id":"claw_pivot_6_run","builtin":false,"minArgs":2,"maxArgs":2,"doc":"claw pivot 6: run; args[0] = volts (-12..12), args[1] = how long, ms"}
{"id":"claw_closed","builtin":false,"minArgs":0,"maxArgs":0,"doc":"claw: go to 'closed'"}
```
**Where the manifest FILE lives** (a generated file per robot, or the panel computing it from the robot file) is
G3a Part R's; the core gives `routeIdsOf()` (constexpr — a host test or tool can write any robot's manifest without
a brain), the binder and the docs. The programs' registry is §1.13's. Sizes (S3): `MechanismRoutes` 4 804 B;
`CommandRegistry` 1 156 → 1 284 B. N13 holds the four details the ruling left open.
---

## 2. The executor runs — six, each its own go, strictly in order, all after T4 Part 3 lands (N12), CAL1 before A5 (N9)

Speedup C's split survives as the order: A1–A3 touch `include/`, `test/` and `docs/` only (the evidence is `git
diff --stat -- src/ tools/`, empty); A4 the same plus its two one-line `case`s in `src/programs/` (T4); A5–A6 are
`src/`. Each run starts from the tip at its launch, re-reads every file this brief cites (line numbers drift — and
the five Part 3 headers of N12 especially), copies its slice of `proposed4/` as the starting point (stripping every
`PROBE_*` switch and prototype marker), and stops when its DoD (§2.1) is met. **Sizes are builder-days, priced
against the briefs' ESTIMATES of comparable runs** (T4 Part 2 before its 2a/2b split ~2–2½ — Part 2a was a subset;
TANK1 Part A ~1¼ for one behaviour; SAFE-SAVE Part A ~1–1½, its brief :42) — none of those is a measured duration —
and against the overlay (it exists and compiles: what remains is the tests, the docs, the review and the mutations,
which in this project are most of a run).

| Run | What (§1) | Files it writes | Waits on | Size | Public docs |
|---|---|---|---|---|---|
| **MECH-A1 — the binding and the devices** | §1.2 `kinds.hpp`, §1.3 `spec.hpp`, §1.4 `wiring.hpp`, §1.5 `devices.hpp` + the read-fault seam; §1.16's `route_ids.hpp`; the five provider accessors in all four implementers and the three adapters' `IReadFaults`; the config's sensor roles + `valid()`; `MotorMechanism::meanPosition`, the per-line `PneumaticMechanism`; `robot::MechanismParts` and the Robot forwarding to it; THE COMPILE-REFUSAL HARNESS (§3.0) and its CI step | `include/shulib/mechanism/{kinds,spec,wiring,devices,route_ids}.hpp`, `hal/read_faults.hpp`, `hal/fake/fake_read_faults.hpp`, `robot/mechanism_parts.hpp` (new); `hal/device_provider.hpp`, `hal/pros/{device_provider,distance,optical,digital_in}.hpp`, `hal/fake/fake_device_provider.hpp`, `hal/mechanism.hpp`, `config/robot_config.hpp`, `robot/robot.hpp`; `test/robot_builder_test.cpp` (the two local providers, `proposed_test3/`); new tests; `test/compile_refusals/`, `test/CMakeLists.txt` (an INTERFACE target carrying the suite's flags), `.github/workflows/ci.yml` | T4 Part 3 landed | **~3½–4½** | changelog; HA-184, HA-186; `docs/api/` (generated); guide 10 (the API); glossary (role, wiring, declaration, route id) |
| **MECH-A2 — the service, power, pneumatic, the guard's cut** | §1.6 `until.hpp`, `service.hpp` (the slot, power, pneumatic, the service's seams, claimant, entry, `enterAutonomous`, leave, lookup; the evidence rule and the read-fault rule; the step preconditions); the outcome appends; N1 with D5's wording; §1.12's plant scaffolding | `include/shulib/mechanism/{until,service}.hpp`, `sim/mechanism_plant.hpp` (new); `manipulation/mechanism_outcome.hpp`, `sequence/run_guard.hpp`; new tests | A1 | **~2¾–3¾** | changelog (N1's BEHAVIOUR CHANGE); `docs/api/`; guide 11 (the `[MECH]` lines, `MECHANISM_STALLED`, `mechanismCuts`); HA-183 |
| **MECH-A3 — position and speed** | §1.7's position and speed behaviours (every homing moment and its zero log, the move and hold loops, reached in units, both limit rules with the closed-loop soft stop, the clamps, N2, the stall watch on every behaviour, the speed loop), N7's starting values and the library formatter of their boot line (beside `roleLine`); §1.12's arm, lift and flywheel models; P2a–P3 and the re-check's cases as tests | `include/shulib/mechanism/service.hpp`, `kinds.hpp` (the starting values), `sim/mechanism_plant.hpp`; new tests | A2 | **~3½–5** | changelog; `docs/api/`; HA-182; the guide's NEW mechanisms chapter (the next free number at its tip — 18 unless G3a Part 1 took it): kinds, units, gearing, limits, homing, jam signatures, tuning |
| **MECH-A4 — autons, the driver loop, the direction judgement, the route binder** | §1.9 (the Routine's steps, constructor and refusal templates, `MechanismStepFailed`, `RoutineResult::mechanismVerdict`, `AutonContext::mechanismService`, the `runAuton` overload — API **2.5**); §1.10 (`driver_inputs.hpp`, the loop's one body + the service overload); §1.11 `direction_check.hpp`; §1.16's `runner/mechanism_routes.hpp` and the registry's doc; N4's "superseded" notes | `include/shulib/chassis/routine.hpp`, `competition/{auton_registry,match_runner}.hpp`, `teleop/teleop_loop.hpp`, `teleop/mechanism_binding.hpp` (notes), `runner/{command_registry,manifest}.hpp`, `include/shulib/version.hpp`, `mechanism/{driver_inputs,direction_check}.hpp`, `runner/mechanism_routes.hpp` (new); `src/programs/{competition_run,auton_tester_run}.hpp` (one `case` each); new tests; `test/routine_signature_pin_test.cpp`, `test/f8_signature_pin_test.cpp` (the appended pins), `test/competition_run_test.cpp` (the word) | A3 | **~3¾–4¾** | changelog (2.5; the F8-additive doc); `docs/api/`; the mechanisms chapter (auton steps, route ids, driver controls); guide 10; a cookbook recipe (an arm with presets, held while driving); `docs/vexbuilder-integration.md` §4.2.3 (the mechanism ids and their docs) |
| **MECH-A5 — robot two and the programs** | §1.13 entire: the table, `boot_source`, N6's `driver_file.hpp` + robot two's driver file, `driver_heading.hpp`, `driver_controls.hpp` retired, `autons.cpp`, the service and the route registry in each program (chain, proof, boot lines, claw row), CAL1's facts mapping (N9), the tester rig, the gate's per-robot rule, HA-185, HA-159's text | `src/robots/mechanism_table.hpp`, `src/robots/calibrate_facts.hpp` (CAL1's, if landed), `src/common/{boot_source.hpp,boot_source.cpp,driver_file.hpp,driver_heading.hpp,driver_controls.hpp (deleted),driver_panel.hpp,guarded_run.hpp}`, `src/team/{robot_two_driver.hpp,autons.cpp,README.md}`, `src/programs/{competition_program,competition_run,auton_tester_program,auton_tester_run,drive_program,xdrive_program}.*`, `tools/src_build_gate.py`; tests (§3.5) | A4; T4 Part 3's files at their landed form; CAL1 (N9) | **~3¾–5** | cookbook 06 (the driver-file listings and their `example-source` directives); guide 16 and 17 (the boot lines, a mechanism step and a mechanism route in an auton); HA-185 + HA-159; `R3a-BENCH-WORKSHEET.md` J.9; README (mechanisms) |
| **MECH-A6 — station 13 and the direction check page** | §1.14 entire: the 3 × 5 menu (N5), station 13's readings in declared units, its six gates, the direction check page with its three answers, RECORD lines, HA-184/186's settles | `src/programs/bench_r3a.{hpp,cpp}`; `test/bench_menu_grid_test.cpp` and new host tests of the station's pure parts | A5 | **~2–3** | `R3a-BENCH-WORKSHEET.md` (station 13, its steps, what each verdict means); the mechanisms chapter (the bench page); changelog |
| **The core, all six** | | | | **~19¼–26** | |

**Why above the requirements' ~10½–18 (REMAINING-WORK row 12)** (+ Q30's 1½–2½ for the bench page and the direction
check): what the requirements did not price, each found by this brief or ruled since — N6's driver file and its
cookbook listings (~½); the compile-refusal harness for 116 named build refusals and its CI step (~½–1); the re-check's
fixes — N10/N11/U2/U4/U5's refusals and words (~½), D1/D2/D4/D8's behaviours (~1), the read-fault channel (~½), the
gate's per-robot rule (~½), the tester rig and the auton tests (~¼) — and the 2026-10-08 route ids (~1¼–1½). Calibrate's
watch, priced in the 10-06 draft, moved to CAL1 (−½, N9).

### 2.1 Definition of done — every run
- [ ] `docs/internal/chunks/MECH-PROGRESS.md` exists (A1 creates it FIRST) and carries this run's heading, its
      tip, each step as it happened, every mutation with its observed red, and the prose review's greps and hits.
- [ ] The host suite builds and passes (`cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests`),
      AND `ctest --test-dir build/test -R compile_refusal` passes (§3.0) — counts in the log; CI runs both (A1 adds
      the step).
- [ ] The ARM header gate (RESUMING :107-112) passes; the CI PROS-free guard passes.
- [ ] **Every run:** `python3 tools/src_build_gate.py check` passes — all seven builds link (the gate's table at
      the tip), zero disallowed warnings (A1–A4 change headers `src/` includes — T11); A5, A6 also `self-test`.
- [ ] `grep -rn "PROBE_" include/` is empty, and no copied file says "PROTOTYPE" or "proposed" (T8).
- [ ] Every §3 test of the run exists and every §3 mutation was RUN, observed red, and restored.
- [ ] Docs in the same change (§4), the prose review logged (pace rule 4), the C7 removability grep empty.
- [ ] Nothing committed; the tree's state stated in the log.


---

## 3. Tests (adversarial) and mutations — each test tries to break one rule; each mutation is RUN and seen red

Every plant test MISMATCHES the controller's constants against the plant's (the A2 truth-integrator lesson): a
test that gives both the same kG proves only the algebra. Every threshold a test asserts is the probe's measured
number plus a stated margin, quoted beside it. A mutation is a one-line change to the code under test, built
FRESH, run, observed red, and restored — logged with the failing test's name.

### 3.0 The compile-refusal harness (MECH-A1, grown by every run)
The core's front door refuses at BUILD; a refusal no test can see is a refusal a mutation can delete. So:
`test/compile_refusals/<reason>.cpp` — one tiny TU per refusal (the 66 of `neg5/` to start, the three MULTI-LINE
ones `m1`–`m3` among them so the LOCATION GCC reports is pinned too, then one per reason not yet covered: every
`refused::` function of `spec.hpp`, `wiring.hpp`, `route_ids.hpp` and `driver_inputs.hpp`, every `detail::`
refusal of `kinds.hpp` and `until.hpp`, the Routine's four named `static_assert`s, `DriverFile`'s two, and the
table file's build-check `static_assert`s) — and a CMake function `shulib_compile_refusal(<name> <file> <text>)`
that makes each TU an `EXCLUDE_FROM_ALL` object target linked to an INTERFACE target carrying the suite's own
flags (`-iquote src`, the defines, the warning set — today PRIVATE to `shulib_tests`, `test/CMakeLists.txt` :103,
:118, :124; T11), and adds a CTest that builds it (`cmake --build <dir> --target <it>`), passing only when the
compile FAILS and its output contains the reason's text as a FIXED string (`neg5/run5.sh` uses `grep -F`; a CMake
`PASS_REGULAR_EXPRESSION` escapes the text — a TU that compiles prints no reason and fails). The three that stay
plain C++ errors (`90_deg`, a run-time value where a constant is checked, a misspelt mechanism) match GCC's own
words, said so in the harness. One POSITIVE TU (`neg5/p01.cpp` + `probe4_forms.cpp` + `team5/counted_steps.cpp`:
every form at the exact limits, every counted line) is compiled into the suite itself. **CI runs `ctest -R
compile_refusal`** — A1 adds the step to `.github/workflows/ci.yml` (today :102 runs only `shulib_tests`).
**Mutation M0:** delete `Arm::to()`'s limit check → `compile_refusal_target_is_outside_the_travel_limits` red.


### 3.1 MECH-A1
- **Each spec refusal at run time**: a hand-built `MechanismSpec` wrong in exactly ONE property returns exactly
  that `Refusal` (so the check ORDER is pinned too), and its sentence; the same reason at build through a
  declaration (§3.0). The agreement G3a will mirror is this table.
- **The words → the spec**: a table over every word (P5b's robot y) asserting exactly the fields it sets; the
  starting values (N7) pinned for Red/Green/Blue × 1:1/5:1/7:1 (P2a's header lines); `.tuning` replaces exactly
  what it names (kP 0.30, within 2.00) and nothing else; a `.name` past 23 characters, a preset name past 15.
- **Where it was written**: each declaration's `.name` line recorded (P10: 24, 25, 30); a host test pins the
  SUFFIX `src/robots/mechanism_table.hpp:<line>` (finding 14).
- **The conversions** (P2g): spool, linkage (1–3 stages) and measured tables round-trip within 1e-12 across and
  past the limits; table ends clamp; the linkage's asin domain clamps; a non-increasing table and a linkage whose
  limits reach past its formula are refused.
- **The wiring**: `wiringProblem` and `mechanisms()` refuse the same inputs (the mirror, both directions);
  `toMechanismConfig` field by field — a Reversed role a negative port, per-line levels only when the positions
  differ per line, each sensor role's port or line; robot two's projection **identical** to the tree's
  `bootConfigFrom` of today's table (P9a — read through `src/common/boot_source.hpp`, so the test fails if either
  side moves); `declaredMotorPorts` physical and in order.
- **The provider accessors**, PROS (over a `FakeDeviceRegistry`) and fake: kind DISTANCE / OPTICAL accepted; every
  other kind and an empty port refused with the sentence `port N: a <kind> (expected a <kind>)`; 0 and 22
  refused; idempotent (the same pointer twice); `digitalIn` refused only outside 1..8; a line made as an output
  then asked as an input (and the reverse) is a precondition.
- **`valid()`**: a sensor port colliding with each other owner (a drive side, the IMU, a pod, the GPS, another
  mechanism's motor, another's sensor) refused naming both; a distance AND an optical on one mechanism; a switch
  on a pneumatic's line (P4b); a switch line 0 or 9.
- **`MechanismParts`** — P4's six census cases as tests, each role's `RoleState` and its boot line VERBATIM
  (clean; a same-kind swap invisible; a rotation/distance swap → ABSENT + "runs WITHOUT"; a motor on a
  piece-sensor port; an unplugged pivot; an actuator port holding an optical sensor → its sensor "not asked"); the
  Robot's accessors unchanged (every existing `robot_builder_test.cpp` case passes, the two local providers
  extended).
- **Mutations:** M1 `mechanisms()` stops refusing a smart port twice → `compile_refusal_a_smart_port_is_used_twice`
  red; M2 `toMechanismConfig` drops a Reversed role's sign → the identity and the reversed-lift tests red; M3 the
  PROS `distance()` skips `expectKind` → the wrong-kind test red; M4 a refused ANGLE sensor treated as degrade-only
  → the ABSENT test red; M5 `valid()` stops claiming sensor ports → the record-collision test red; M6 `motors()`
  accepts a `char` → `compile_refusal` n27 red; M7 two checks of `specRefusal` swapped → the one-property test red.
- **(re-check) The words' refusals and the stall pair**: `.gearing = 5` (n41); levels vs lines both ways (n42,
  n43); a homing push away from its home end at either limit, both methods (n44, n45; a home strictly inside the
  range pushes either way and compiles); `.stall` unset on an arm and a flywheel (n46, n47); amps 2.5 and above
  refused, 2.49 accepted; `belowMotorRpm` 0 refused, 0.1 accepted (n58, n59) — each as a one-property `specRefusal`
  test AND a compile-refusal TU.
- **(re-check) The read-fault seam**: each PROS adapter's `faultedReads()` reached through `IReadFaults` equals
  its own count (shim `disconnected` / `broken`); `ProsDeviceProvider::readFaults(port)` returns the distance or
  optical adapter it made there and nullptr elsewhere; `lineReadFaults(line)` the input; the fake's counters move
  only while `failing(true)`; `MechanismParts` fills `pieceFaults` / `homeSwitchFaults`.
- **(re-check) Route ids** (§1.16): `routeIdsOf` over robot x gives exactly P14's 17 ids in order with their arg
  counts, robot two's 6; the stem rule ("Claw  Pivot-6!" → `claw_pivot_6`); an id of exactly 23 characters accepted
  and 24 refused; a duplicate, a built-in and the 33rd id refused (n60–n63), each naming itself in `offending`.
- **Mutations (re-check):** M38 `routeIdsOf` stops checking `findBuiltinCommand` → n62 red; M40 the stall pair's
  `<` becomes `<=` on 2.5 → the 2.5-refused test red; M41 `ProsDistance` loses `IReadFaults` (`readFaults` returns
  nullptr) → the provider test and R2 red.

### 3.2 MECH-A2
- **Once per tick** (P8): the raw pacer, the service and `guard{service}` handed to the loop — N iterations, N
  service ticks, N world paces, all three; `pace()` unbound forwards and ticks nothing; a re-entrant tick is a
  precondition; `tickFromDriverLoop()` then `pace()` ticks once.
- **The claimant**: the guard's cancel-all reaches a running behaviour (inert, safe, released, CANCELLED), and the
  NEXT tick commands nothing — the "half-safe" failure `mechanism.hpp` measured; another owner's claim → every
  step REFUSED and `drive()` silent; an ABSENT slot → every step ABSENT, nothing commanded.
- **Power**: every done-condition alone and OR-ed (time; its own sensor; ANOTHER mechanism's sensor; stops
  seeing; `stalls` → SUCCEEDED); a stated stall without `stalls` → STALLED and `MECHANISM_STALLED` raised once;
  the bound → TIMED_OUT; volts clamped to `maxVolts`; the safe state on EVERY exit.
- **Pneumatic**: robot x's hood on two lines — each position's level on each line; the guard's safe state applies
  the PER-LINE safe position (from "middle" to "down" = low/low); the start position at bind; robot two's claw:
  `grabs` with NO sensor → UNCONFIRMED and counted done; with a sensor that sees → SUCCEEDED "sees a piece"; with
  one that never sees → UNCONFIRMED, NOT counted, after `confirmWithin`; driver entry sends START on the FIRST
  entry only; a disable keeps the level.
- **A declared piece sensor missing this boot** (P13): `seesPiece` and `stopsSeeingPiece` each run their full
  bound with the motor running and end per N8, the line naming the sensor — never SUCCEEDED; the same step with the
  sensor present and seeing still SUCCEEDS at once.
- **Driver entry**: a leftover claim of ANOTHER owner is released WITHOUT its `cancel()` (a spy proves it never
  ran) and counted; the service's own behaviours dropped; a Running step → CANCELLED.
- **N1** (P6): the four cases (deadline / disable × a 2 s / 5 s step) — late by at most one pace, ~0 V·s past
  the cut, the end action ran and succeeded, `mechanismCuts` 1; the DEADLINE does not cut a mechanism step INSIDE
  the end action (an ABANDON does — the re-check's split below);
  an unclaimed mechanism is not re-safed each pace (`applySafeState` counted: once, at the run's end).
- **Mutations:** M8 remove `cutClaimedMechanisms()` → the N1 tests red; M9 the tick skips its `holdsClaim()` check
  → the half-safe test red; M10 `pace()` ticks after `tickFromDriverLoop()` → P8 red; M11 `enterDriverControl()`
  calls the foreign claimant's `cancel()` → the spy red; M12 a sensorless `grabs` not counted done → the claws
  test red; M13 `applySafeState()` drives every line to line 0's level → the hood's safe test red; M13b a missing sensor read as "sees nothing" again → P13's test red (the false success returns).
- **(re-check) The evidence rule and the read-fault rule (N8 amended; D7, D8):** the R probe's eight cases as tests
  through the REAL adapters over the shim (`adv_readfaults.cpp`): R2, R3, R8 → UNCONFIRMED not counted after the
  bound, the motor run; R1, R6, R7 unchanged; the home switch S → not homed at bind; a condition on a mechanism with
  no declared sensor, and `stalls` with no signature → the step's precondition, nothing commanded
  (`adv_conditions.cpp`); case J (held near) → UNCONFIRMED. A FakeDistance's "no piece" is 393.66 in (landmine 21).
- **(re-check) `enterAutonomous()` (D1):** `adv_handover.cpp` — the driver's intake at 12 V and a toggled flywheel,
  the driver task deleted without `leaveDriverControl()`, then an auton that never names them: 0 of 269 paces at
  12 V (257 without the call); a foreign claim released without its `cancel()` (a spy), counted; the re-entrancy flag
  cleared.
- **(re-check) N1 split (D5):** the DEADLINE does not cut a step inside the end action; an ABANDON does
  (`adv_endaction.cpp`: CANCELLED, T+1.52, 0.1 V·s).
- **Mutations (re-check):** M35 `runAuton`'s service overload stops calling `enterAutonomous()` → the handover test
  red; M37 the abandon cut gated on `!inEndAction_` → the abandon-in-end-action test red; M42 `seen()` ignores
  `pieceFaults` → R2/R3/R8 red (a false SUCCEEDED returns); M43 a distance read at confidence 0 counted as "gone"
  again → case J red.

### 3.3 MECH-A3
- **P2a's grid as a test** (144 moves): ≥ 143 SUCCEEDED, overshoot ≤ 7.5°, final error ≤ 2.6°, and ≤ 3.8° overshoot
  where the plant is no faster than its cartridge; the miss is the stated case. **A profile the plant cannot
  follow ends TIMED_OUT, never SUCCEEDED** (finding 4).
- **P2b** hold under +0.8 V: worst ≤ 2.6°, within 1.4° after 2 s; the CONTROL (nobody ticking, Hold at 0 V) sags
  ≥ 2.5° — so the test can see a sag.
- **P2c** blocked: with the robot file's signature (2.0 A / 5 rpm / 0.5 s) STALLED within 1.2 s (1.6 A / 3 rpm /
  0.15 s: within 0.8 s), fault once, 0 V, released; with a hand-built signature that cannot trip (2.6 A — the build
  refuses it, n58) TIMED_OUT, then N2's hold-where-stopped: mean |V| < 2.5 V and current < 1 A over the next 2 s.
- **P2d** the limits (at the limit; a computed target clamped and logged ONCE; a jog HELD at the limit by the
  closed-loop hold (D2); never homed: a jog at 0 V or at most `unhomedVolts`, a move NOT_HOMED). **P2e** every homing
  moment (stall, switch, pressed-at-boot, rests-at-stop; NOT_HOMED before). **P2h** absolute: at 40° reads 40;
  mounted reversed and declared so: fine; the MISMATCH drives into the hard stop and ends STALLED (the required
  signature; TIMED_OUT before N10) — asserted, the hazard the direction check exists for. **P2f** the lift; **P2g** (§3.1).
- **The integral's job**: the hold loop WITHOUT its integral leaves ≥ 2° of steady error under a 25 % gravity
  mismatch (the ablation that proves the integral earns its place); a fresh hold loop at the profile's end (no
  windup carried in).
- **P3** the flywheel's 27 cases: every reachable one at speed but the stated slowest; the unreachable TIMED_OUT
  and still commanded; a 20 % shot recovered; the integral-from-the-start ablation overshoots > 150 rpm.
- **Mutations:** M14 the hold loop's kI 0 → the gravity-mismatch hold red; M15 the speed integral's 10 % window
  removed → the overshoot test red; M16 the soft stop removed → the jog-at-limit test red (it reaches the hard
  stop); M17 a move before homing proceeds → NOT_HOMED red; M18 a computed target not clamped → P2d red; M19
  absolute homing zeroes "where it is" (the first prototype's bug) → P2h red; M20 the gravity term's sign flipped →
  the hold red; M21 N2 reverted (hold the target) → the stall-current assertion red.
- **(re-check) N10:** case A (a blocked hold after SUCCEEDED) → STALLED once, then 0 V; case H (a jammed flywheel)
  → STALLED, 0 V; a jog into an obstacle → STALLED, the held button superseded until released; a Power's
  hold-to-run into a jam with a stated signature → STALLED at ~0.30 s; P2h's mismatched sensor → STALLED at the hard
  stop; a hand-built 3.0 A signature (`adv_stallsig.cpp`) never trips — the test that justifies n58.
- **(re-check) D2:** both limits, arriving at jog speed, plant kG 1.0×/1.5×/2× the controller's (`adv_extra.cpp`
  (1)): overshoot within the measured 4.06° + 1° margin; the release's hold target never outside the range; N2's
  hold-where-stopped clamped; case C's jog stops at the believed zero.
- **(re-check) D4:** a target AT each limit SUCCEEDS for every geometry — rotary, spool, a linkage at its formula
  maximum, a measured table at both ends (case G: 1.91 s).
- **(re-check) D10:** a stall-home into a wedged piece logs `homed -- 25.0 deg from where its encoders read zero`;
  a re-home that moves zero past the tolerance logs a Warn.
- **Mutations (re-check):** M16 now red at BOTH limits; M36 "reached" back in converted degrees → the table-top test
  red; M39 the hold's stall watch removed → case A red (30 s at 2.5 A).

### 3.4 MECH-A4
- **P1 as a test** through the REAL `runAuton` (service overload) on the A2 tank plant and the §1.12 models: the
  arm holds within 1.5° through a 1.8 s `driveTo`; the intake stops on its sensor while the drive still runs; one
  service tick per pace; every claim released and every safe state applied after; the CONTROL (the service not
  in the chain) stops with "the mechanism service never ticked".
- **Every step form** (p01, P5b) in a routine on the plant: SUCCEEDED continues; counted UNCONFIRMED continues;
  TIMED_OUT, STALLED, NOT_HOMED, ABSENT, REFUSED each stop the chain as `MechanismFailed` with the verdict in the
  stop line, `mechanismVerdict` and `lastMechanism()`; a step naming another robot's mechanism → ABSENT with the
  sentence; `waitUntilDone` of a finished step returns at once; a START form's failed start stops the chain; the
  frozen constructor + a mechanism step → the precondition naming the fix.
- **COMP-a unchanged**: positional `AutonContext{...}` still compiles; the old `runAuton` behaves as before (its
  tests untouched).
- **The driver loop** (P5d, P2i, P8, P9c as tests): a press on the entry tick ignored; a preset; a hold that wins
  mid-move and holds HERE on release; a newer preset beats a still-held hold, which then commands nothing until
  released; both buttons = neither; `homeAtDriverStart` once, skipped when homed; the disable (motor kinds safe,
  pneumatics kept, claims released); both paths handed at once → precondition; a control on the re-zero →
  precondition; every T4 Part 2a protection test passes unchanged.
- **The direction judgement** (P7): the eight verdicts, strings VERBATIM; `changeHere`'s three sources.
- **Mutations:** M22 the Routine stops on a counted UNCONFIRMED → the claws auton red; M23 the SUPERSEDED state
  removed → P5d red (the held button re-commands); M24 the service ticked AFTER `perTick` → an order pin red (a
  spying `perTick` sees last tick's command); M25 `judgeDirection` ignores the command's sign → the fight test red.
- **(re-check) Route ids through the REAL `PathRunner` (P14, `probe5_routes.cpp`):** robot x's 7-marker route →
  ROUTE_RAN, every marker SUCCEEDED, the end action succeeded; robot two's route → the sensorless grab UNCONFIRMED,
  counted done, the route goes on; `arm_score` with no args, with 0, and `intake_run [13, 500]` / `[0, 500]` → REFUSED,
  `MarkerFailed`, the drive safed; the manifest's lines for both robots byte-exact (docs included); a registry with
  NO docs writes today's golden unchanged; `registerInto` twice → the registry's duplicate precondition.
- **(re-check) The stop cause (T4):** every failing mechanism step stops the chain as `MechanismStepFailed`, never
  `MechanismFailed`; a `then()`-action's failure is still `MechanismFailed`; the appended pin `== 5`.
- **(re-check) The driver file's homing rule (U5):** n48–n50 compile refusals; a switch-homed lift with
  `homeAtDriverStart` after a restart off its switch: the first tick homes it, its buttons then move it.
- **(re-check) The direction check's three answers (D6):** every verdict string of §1.11 verbatim; a same-kind swap
  answered "it did NOT move" → `CableOnAnotherMechanism` naming the `wire(...)` line; two equal motors stalled
  against each other → `FightOrJam`.
- **Mutations (re-check):** M44 the route binder maps a counted UNCONFIRMED to Unconfirmed → robot two's route red;
  M45 the marker's arg check removed → the no-time-limit route runs `arm_score` with a 0 s bound (the step's own
  precondition throws) — red; M46 `judgeDirection` answers "the OTHER way" for `NothingOrSomethingElseMoved` → the
  swap test red.

### 3.5 MECH-A5
- `test/mechanism_table_test.cpp` (rewritten): robot two's facts pinned literally — pivots +6/+7, Red, Hold; the
  claw on A, closed = Low (the guess, HA-159), `moveTime` 0.5 (HA-185); the selection for EVERY robot (its loop
  :384 iterates `kEveryRobotId` once G3a Part 2 has landed — G3a item 8 — today's three otherwise);
  `mechanismsClearOf` refuses a planted SENSOR on a drive port and on the IMU's; `isDeclaredMechanismPort`
  (today's cases); Calibrate's drop and watch are CAL1's (N9) — A5's test is the facts mapping (below).
- `test/boot_source_test.cpp`: robot two's derived config equals today's numbers, written literally.
- The programs' source pins (`test/source_pins.hpp`, the competition, drive and tester source tests): each chain's
  order (guard → service → watch), the three-layer proof, the per-role boot lines; `test/guarded_run_test.cpp`: a
  chain without the service FAILS the proof.
- `test/driver_panel_claws_test.cpp` → the per-role and per-control boot lines and the claw row read from the
  service; `test/robot_two_driver_test.cpp` through the PROS shim against robot two's REAL list (R1 → pivot 6 at
  12 V; A toggles; the first entry sends CLOSED; a disable keeps the claw).
- The gate's new self-test cases (the nine of `out5/gate_cases.txt`).
- **Mutations:** M26 `bootConfigFrom` loses the per-line levels → the hood fixture red (robot two's one line
  cannot see it — say so); M27 the gate exception admits CHASSIS tables → the new self-test case red; M28 the
  proof ignores the service → `guarded_run_test` red; M29 `mechanismsClearOf` checks motors only → the planted
  sensor red.
- **(re-check) The tester rig and the autons (U3):** every registered auton runs with the service live; the claws
  auton's and the claws ROUTE's verdicts asserted step by step; `autons.cpp`'s `Routine r{ctx, …}` pinned by a source
  test. **The route registry:** each program registers the build's ids at boot after binding (a source pin), passes
  `&registry` to `runAuton`, and its boot line names the count. **The gate (T3):** the nine planted cases of
  `out5/gate_cases.txt` as self-test cases. **The retired header (T6):** `test/driver_heading_test.cpp` (the moved
  cases), and a grep test that no `src/` file includes `driver_controls.hpp`. **N9:** `calibrateFactsFor` over the new
  list gives robot two's facts unchanged (ports 6, 7; names) — or, if A5 lands before CAL1, today's drop call reads
  the new list with `test/mechanism_table_test.cpp`'s drop cases unchanged.
- **Mutations (re-check):** M33 the gate's namespace rule skips `using namespace` → its planted case red; M34 the
  `src/team` exception admits another robot's list → robot three's case red; M47 a program forgets
  `registerInto` → the route test's `unknown ids` count red.

### 3.6 MECH-A6
- `test/bench_menu_grid_test.cpp`: 15 buttons inside 240 px, no overlaps, stations 1–12 at their numbers and
  places (pixel-for-pixel against the 3 × 4 grid it replaces, like R2a's).
- The station's pure parts: the gate state machine (refuses an ABSENT mechanism; no volt before the two-stage
  confirm; the dead-man's release safes that tick; the ceiling 3 → 12; each cut-out; the exit guard), the
  direction page's flow (jog → observation → `judgeDirection` → the question → the verdict), the RECORD line.
- **Mutations:** M30 the dead-man ignored → red; M31 the ceiling starts at 12 V → red; M32 a valve fired without
  its confirm → red.
- **(re-check):** the readings in declared units (a 12:60 arm turned one output turn by hand reads 360°; a spool lift
  one spool turn reads π × 1.5 in); the switch's three states shown; the page's three answers drive `judgeDirection`
  and `judgePneumatic` as §1.11; the mechanism's own stall signature cuts a jog (M48: removed → red).

---

## 4. Documentation (same change — pace rule 3; prose review logged — rule 4)

### 4.1 Per run (public docs; each ≤ ~5, pace rule 7)
- **A1:** `docs/changelog.md` ("Added": the mechanism vocabulary, the provider's three accessors, the config's
  sensor roles; "Changed": none in behaviour); `docs/hardware-assumptions.md` (HA-184, HA-186 rows + entries);
  `docs/api/` regenerated (`tools/api_doc_tool.py`) for the new headers and `device_provider` / `robot` /
  `robot_config` / `mechanism`; `docs/guide/10-the-api.md` (the mechanism half of the API); `docs/guide/
  15-glossary.md` (declaration, role, wiring, piece sensor, "not checkable").
- **A2:** changelog (the service; N1 as a BEHAVIOUR CHANGE: "an operation with a registered claimant now stops
  at the run's deadline, not at its own bound"); `docs/api/`; `docs/guide/11-reading-the-diagnostics.md` (the
  `[MECH]` step lines, `MECHANISM_STALLED`, `mechanismCuts` in the guard's report); HA-183.
- **A3:** changelog; `docs/api/`; HA-182 (row + entry, every starting number listed); **the guide's new
  MECHANISMS chapter** (the next free number at A3's tip — 18 unless G3a Part 1 took it; G3a then takes the next):
  the five kinds and their words, units, limits and the soft stop, every homing moment and which to choose, the
  starting values and how to tune them (`.tuning`), what "held" costs (N2), and the honest limit — the service
  runs only inside shulib's waits, motions and driver loop.
- **A4:** changelog (**API 2.5**: the Routine's steps and constructor, `RoutineResult::mechanismVerdict`,
  `AutonContext::mechanismService`, the `runAuton` and `runDriverControl` service overloads; T3's bindings
  "superseded" — N4); `include/shulib/version.hpp`'s history line; `docs/api/`; the mechanisms chapter (auton
  steps, run while driving, the verdicts; driver presets, "the most recent input wins"); a cookbook recipe (an arm
  with presets held while the robot drives).
- **A5:** `docs/cookbook/06-driver-control.md` (its three listings of robot two's driver file become N6's, each
  with its `<!-- example-source: src/team/robot_two_driver.hpp -->` directive if G3a Part 4's rule has landed, the
  unbroken-run rule otherwise); `docs/guide/16-testing-an-auton-on-the-bench.md` and `17-the-competition-program.md`
  (the per-role boot lines, a mechanism step in an auton); HA-185 + HA-159's text; `docs/internal/chunks/
  R3a-BENCH-WORKSHEET.md` J.9 (the flip's place) — internal; `README.md` (mechanisms in the feature list).
- **A6:** `R3a-BENCH-WORKSHEET.md` (station 13: its gates, each step, what each verdict means, the direction
  check's "edit, rebuild, upload"); the mechanisms chapter (the bench page); changelog.
- **Every run, internal:** `MECH-PROGRESS.md`; `docs/internal/build-order.md` (MECH's entry, the first bold
  `Next:`); `docs/roadmap.md` ("You are here", the MECH rows, `[~]` until A6); `REMAINING-WORK.md` §3 row 12;
  `HANDOFF-remaining-work.md`; at A6's close `MECH-COMPLETED.md`. The C7 removability grep (no public doc links
  into `docs/internal/`) before every hand-back.
- **(re-check) additions to the docs above:** A1 — the glossary's "route id" and "read fault"; guide 10's
  `IReadFaults` and the provider's two accessors. A3 — the mechanisms chapter teaches the member ORDER per kind,
  tooth-count gearing, the jam signature recipe (a bit below 2.5 A, a low motor speed, ~½ s; a deliberate squeeze
  needs a higher one), that limits sit short of the hard stops by more than the measured overshoot, what
  `restsAtStop` and stall-homing trust, and **"reading a refusal"** (where GCC puts your line, with an example).
  A4 — the chapter's route ids (the rule, the args, what blocks), `seesPiece.orAfter`, `whileHeld`, the
  never-satisfiable conditions refused at the step, and the starved service (D11); `docs/vexbuilder-integration.md`
  §4.2.3 gains the mechanism ids and their docs (a public doc VexBuilder reads). A5 — guide 16/17 show `Routine
  r{ctx, …}` only, and a route auton with mechanism markers.
- **(re-check) prose-review greps added:** A1 `gearRatio|Speed\b|RobotMechanisms|faultedReads`; A3 `soft stop|stall
  signature|belowRpm`; A4 `hold(|toggle(|MechanismFailed|CommandRegistry|manifest`; A5 `driverHeading|
  ctx.chassis|kMechanisms`.

### 4.2 Prose review (pace rule 4) — the greps each run runs and logs
`grep -rn` over `docs/` and `README.md` for the names it changed: A1 `MechanismConfig|digitalOut|rotation(|
IDeviceProvider|mechanisms()`; A2 `RunGuard|cancel-all|safeMechanisms|MechanismOutcome`; A3 `hold|gravity|
homing|travel limit`; A4 `Routine|then(|AutonContext|runAuton|runDriverControl|MechanismBinding|ToggleBinding`;
A5 `mechanism_table|kClawOpenCommand|driver_controls|HoldControl|ToggleControl|DriverBindings|claw pivot|
robot_two_driver|Calibrate.*declared`; A6 `Bench Tests|station 1[0-3]|3 × 4|twelve stations`. Every hit read;
every stale sentence fixed or listed with why it stays.

### 4.3 Hardware Assumptions — HA-182..HA-186 ONLY (the register's held line, :39)
| # | The belief | Confidence | Settled by | Owner |
|---|---|---|---|---|
| **HA-182** | The core's STARTING numbers (N7) hold a real mechanism well enough to be tuned from: position kP 0.25, kI 0.3 (hold loop), kD 0.004, kV = 12 V over the output's free speed, the smooth move at 75 % of the free speed reached in 0.25 s, "reached" within 1.5° / 0.25 in for 0.1 s, the hold's integral clamp 3 V; speed kV = 12 V over the free speed, kP 0.05, kI 0.2 acting within 10 % of the target; the driver toggle's 5 s at-speed window; station 13's jog volts and time, its "did not move" angle and its "drawing" current (the direction check's two thresholds) | **invented** | tuning each mechanism on the robot (MECH on hardware); station 13 for its own numbers | A3 (A6 adds the page's) |
| **HA-183** | The host test plant's mechanism models stand in for real ones closely enough that P2/P3's verdicts transfer: the inverted-feedforward arm/lift/flywheel, their kS/kV/kG/τ ranges, the Hold-at-0 V stiffness, the stall current's shape, a hard stop's ZERO speed (a real stalled motor creeps — HA-93) | **invented** | the first robot session that logs a real arm's and a real flywheel's step response beside the model's | A2 (A3 tunes) |
| **HA-184** | A VEX limit switch read through `ProsDigitalIn::state()` (`get_value()`) reads 1 — TRUE — when PRESSED, so `pressedLevel = true` is right; and an UNPLUGGED switch line reads RELEASED | **unknown** — the vendored `pros/adi.hpp` :731-757 (`get_value()`) says only "Gets the digital value (1 or 0) of a pin"; :711-712, cited by the 10-06 draft, is inside `get_new_press()`'s doc; HA-121 (`get_value()` is a level) is its neighbour, not its answer. **What a wrong one does:** every switch-homed mechanism zeroes wherever it stands at boot (the nets, measured: the required stall signature stops the move into the hard stop — case F, STALLED at 1.70 s; a switch that FAILS from boot no longer homes it — S) | station 13: read the switch pressed, released AND with its cable out | A1 |
| **HA-185** | Robot two's claw moves between closed and open within 0.5 s, so a `set()` step's verdict at 0.5 s is not early (N3) | **invented** | Station J: a slow-motion video of closed → open and back | A5 |
| **HA-186** | At power-up a V5 rotation sensor's CUMULATIVE `get_position()` (what `ProsRotation::position()` reads, HA-11) starts at the shaft's absolute angle within one turn, so `absoluteSensor()` homing needs no motion — and the same holds after a PROGRAM restart without a power cycle | reasoned (VEX's description of the sensor as absolute within a turn); the vendored `pros/rotation.hpp` :194-216 (`get_position()`) says only "current position" (:244-266, cited by the 10-06 draft, is `get_angle()`) | station 13: with the shaft at two marked angles, power-cycle the brain and read; then move the arm, restart only the PROGRAM, and read again | A1 |

A sixth belief ⇒ stop and tell the coordinator. Every HA cited in a header comment (`HA-182` beside each starting
value, etc.) is reconciled both ways, the register's own rule. The re-check added no HA: the never-trip stall pair
rests on the vendored 2500 mA default (`pros/motors.h` :1255-1257) and HA-93 (existing); the read-fault channel on
the adapters' own documented screens.


---

## 5. OUT, and explicitly rejected

- **THE REST (after the panel, his ruling of 2026-09-28):** the cycle kind (catapult/puncher), piece counting,
  colour sorting, jam auto-reverse, interlocks, loaded gains, the low-power hold, current caps and heat derating,
  tip protection, preset cycling, stick control, long-press and double-tap, rumble, the controller's mechanism
  row, mechanism macros, triggers partway through a drive, guided port binding, auto-tuning, the per-match health
  log, and the panel's extension for them. None is stubbed. The core's shapes leave room for each without a
  breaking change (a new kind is a new declaration type and slot; a control kind is appended to `ControlKind`; a
  done-condition is a bit in `Until`) — and nothing more is promised.
- **A PTO** (Q4 (b)): §1.15 names the five places; nothing is built.
- **3-wire expanders** (Q5 (a)): a `Line` is the brain's A..H only.
- **A route preview or simulation** (reversed 2026-10-08, `3c643f1`): the core derives and runs route ids; it
  simulates no route outside the library's own tests. **Mid-leg mechanism triggers** stay the rest's (row 14).
- **An SD binding record or a saved flip** (Q1 (a)): the direction check REPORTS; the robot file is the only place a
  binding or a direction lives.
- **Robot two's pivots gaining positions or homing** (Q6, undecided): they stay `Power`; the day they become an
  `Arm`, the robot file's `.home` says which moment, and nothing in the library changes.
- **The AI Vision adapter, the distance-sensor wall reset, driving to a piece** — after G3 (his ruling of
  2026-09-28); the core's distance and optical sensors are MECHANISM piece sensors only (Q3).
- **Rejected:** a deferred auton chain (`r.run(...).until(...).within(...)` — F10 froze EAGER: "a chain built
  but never run, which compiles and does nothing"); strings as auton names (Q18 (a)); a second "handles" file
  (the declaration is the handle); a running cross-check of an angle sensor against the encoders (protection — the
  rest's; P2h's nets are the direction check and a stall signature); a library default for any number the team
  owns (P6).

---

## 6. Coordination — shared files, who re-reads whom, and what G3a must adopt

### 6.1 The tree and what is in flight (re-checked 2026-10-08)
- **Landed since the 09-29 draft** (main `2fce38a`): TANK1 (complete), COMP-t, L9 Part A1 (API 2.4), DIAG1
  (complete), H1 Part A1, T4 Part 2a, COMP-b, SAFE-SAVE Part A, **R5a Part A** (`083e232`: library-only,
  `sim/drive_plant.hpp` — not the core's `mechanism_plant.hpp`), and four plan commits (`fda5351` CAL1 and the
  front; `6517f2d` the route ids — §1.16; `3c643f1`, `2fce38a` the panel's routes without simulation). None is re-done
  here; every one is read as landed.
- **T4 Part 3** (`.claude/worktrees/t4-3`, both steps built, its fix round RUNNING as this is written,
  uncommitted) — **the core starts after it lands** (N12). Its library headers the core rewrites (A1, A4 re-read
  each at their tip): `hal/pros/device_provider.hpp` (+87/−5; the fix round is editing it now),
  `config/robot_config.hpp` (+8/−8), `robot/robot.hpp` (+15/−12), `competition/auton_registry.hpp` (+56/−6),
  `teleop/teleop_loop.hpp` (+10/−9). Its `src/` and `test/` files the core then edits (A5): `src/robots/
  mechanism_table.hpp` (Part 3 took the buttons out), `src/team/robot_two_driver.hpp` (Part 3's final form),
  `src/team/autons.cpp`, `src/common/driver_panel.hpp` (COMP-b's shared panel, Part 3's PROTECTION row),
  `src/common/{boot_source.hpp,boot_source.cpp,guarded_run.hpp,driver_controls.hpp}`, `src/programs/
  {drive_program,competition_program,competition_run,auton_tester_program,auton_tester_run,xdrive_program}.*`
  (the fix round has changed `drive_program.cpp`, `competition_program.cpp`, `auton_tester_program.cpp` since
  `tree_t43d`), `tools/src_build_gate.py` (Part 3 edited it), and the tests over them (`source_pins.hpp`,
  `drive_program_source_test.cpp`, `competition_source_test.cpp`, `driver_panel_claws_test.cpp` — renamed from
  `teleop_claws_test.cpp` by Part 3 —, `robot_two_driver_test.cpp`, `driver_controls_test.cpp`,
  `boot_source_test.cpp`, `mechanism_table_test.cpp`, `auton_tester_rig.hpp`, `autons_test.cpp`). This brief's line
  numbers are `tree_t43d`'s. Part 3's new `src/common/dropped_autons.hpp` and `log_copy_line.hpp` are not touched.
- **CAL1** (row 18b; its brief is being drafted, `briefs/CAL1-standard-calibrate.md` 10-08 00:56) — "after T4 Part
  3, beside the mechanism core". **N9 (a): CAL1 lands before MECH-A5** and owns the declared-port drop and the push
  WATCH; CAL1 Part B deletes `dropDeclaredMechanismPorts` from `src/robots/mechanism_table.hpp` and adds
  `src/robots/calibrate_facts.hpp`; A5 rewrites `calibrateFactsFor` over the core's list. If A5 lands first: A5
  re-points `calibrate_program.cpp` :1129-1130 (one line) and CAL1 maps the core's list. CAL1 Part A (library only)
  shares nothing with A1–A4 (its `calibration/calibrate_program.hpp`, `hal/brain_screen.hpp`, … are new files).
- **SAFE-SAVE Part B2** (the boot's fallback) edits `src/common/boot_source.*` (the record READ); A5 edits
  `bootConfigFrom` and the declared-port rule — different functions; whichever lands second rebases.
- **L9 Part B** (the boot as library code; COMP-b, its prerequisite, has landed) moves `decideBootSourceFrom` into
  the library and DERIVES the declared ports from the base config's mechanisms — **its derivation must include the
  sensor ports A1 adds** (`rotationPort`, `distancePort`, `opticalPort`), or a record that drives a mechanism's
  SENSOR port would boot. Order: MECH-A5 before L9 Part B (L9 then moves the rule A5 wrote); if L9 Part B goes first,
  A5 extends its derivation.
- **G3a** (the amended brief waits for the team lead's D6′–D10): its Parts 1 → 2 → 3 build BESIDE A5/A6 (speedup
  D) — Part 1 starts "after T4 Part 3 lands, when the core's `src/` half starts". Shared with A5: Part 2's two
  marker lines inside `mechanismTableFor()` and its one-line loop change in `test/mechanism_table_test.cpp`
  (:384); `tools/src_build_gate.py` (Part 2 edits the gate's tables and regions; A5 edits step 0's per-robot
  functions — different functions); `src/build_selection.hpp` (G3a only). Whichever lands second rebases; each
  conflict is a few lines. Calibrate is no longer shared with G3a through the core (N9). Parts 4–6 and Part R (the
  routes) follow the core; Part R reads the core's route ids (§1.16).


### 6.2 G3a's eight requests (its §5 "what its brief should carry for G3a", re-check #6, #7, #10) — each honoured
1. **Keep the per-robot selection switch.** `mechanismTableFor(RobotId)` stays a case per robot with no default,
   returning `const MechanismTable&` (now the library's `mechanism::MechanismTable`, N11), where Part 2 puts its
   `[mechanism selection]` region. The core adds NO second per-robot selection function (no handles file exists).
2. **Robot two's data in a form a region wraps byte for byte.** One contiguous block, `// [robot two mechanisms]`
   … `// [/robot two mechanisms]` (§1.1, P10): the declarations then the wiring list, no hand-written line
   inside. There is no separate "typed handles", "binding" or "polarity" block to wrap — the declarations are the
   handles, the wiring is the list's `wire(...)` lines, and the polarity is the claw's `.positions` line. So G3a's
   four robot-two regions (`[pneumatic polarity]`, `[mechanisms]`, `[control words]`, `[controls]`) become TWO:
   this one, and the driver file's `[driver file two]` (§1.1). If G3a wants declarations and wiring as two regions,
   A5 puts `// [/robot two declarations]` / `// [robot two wiring]` around the comment line between them — say so
   before A5 runs. (Re-check U8: the region now holds ten caveat comment lines too — G3a's emitter writes them from
   the robot file's notes, so the region still round-trips byte for byte.)
3. **A per-role "change it here" string.** Every role has the field — `Motor::changeAt`, `Sensor::changeAt`,
   `Pneumatic::changeAt` (the levels) — which G3a's generator fills with the robot file's JSON path (P7's
   generated message). A HAND-WRITTEN file leaves it null and the declaration's captured `file:line` is named
   instead, so the core types no string by hand (better than asked: it cannot go stale when a line moves).
4. **No counts in the mechanism pin.** The rewritten pins check the ADDRESS only (§1.13, P10).
5. **The Calibrate edit separate from Part 2's — now CAL1's (N9).** The core edits NO Calibrate code: CAL1 owns the
   drop and the watch (its D4 (a)); A5 maps the new list into CAL1's facts (`src/robots/calibrate_facts.hpp`), so
   G3a Part 2's Calibrate hunks (the banner and refusal words, the front lines) meet CAL1's file, not the core's.
6. **(re-check #7) "only robot two declares mechanisms"** (now `mechanism_table.hpp` :522-523) is rewritten
   `chassis_table.hpp`'s way: the generic `&kMechanisms == &mechanismTableFor(kRobotId)` and the hand-written
   robots that declare none BY NAME ("the bench bot and the invented X-drive declare no mechanisms") — P10.
7. **(re-check #10) Calibrate WATCHES the declared mechanism ports during the push** — in CAL1 (N9; CAL1 D4 (a),
   HA-187, measured on CAL1's host rig): raw reads, nothing commanded, the push's own threshold, one line per port
   that moved. G3a Part 5 does not need its +~½ day, and neither does MECH-A5.
8. **(re-check #6) the per-robot loop** in `test/mechanism_table_test.cpp` (:384) iterates `kEveryRobotId` once
   Part 2 has landed; A5 keeps a per-robot loop either way.
(9.) The first free HA number was HA-182: the core uses HA-182..HA-186 and no other.

### 6.3 What G3a's robot file must adopt — EXACTLY this vocabulary (its §1.14 re-check, before Part 5)
| G3a §1.14 (proposed 09-29) | The core's (this brief, as re-checked) | Change for G3a |
|---|---|---|
| `kind`: power / position / speed / pneumatic, with `units` | FIVE declaration types: `power`, `arm` (degrees), `lift` (inches), `flywheel`, `pneumatic` | `kind` takes the five words; `units` is dropped (implied by arm / lift) |
| `roles[] {role, device, reversed}` (free-form roles) | ordered `motors` (each forward / reversed) and three NAMED sensor slots: `angleSensor` (rotation, its mounting), `pieceSensor` (distance + "sees within" in / optical + proximity / limit switch), and the home switch implied by `home: homeBySwitch` | `motors: ["forward", "reversed"]`; `angleSensor`, `pieceSensor`; no free role names; the boot line's role words are "motor N", "angle sensor", "home switch", "piece sensor", "line N" |
| `cartridge`, `safe` | the same (`red/green/blue`; `hold/brake/coast`; a pneumatic's `safe` names a position) | none |
| `autonOnly` (a mechanism field) | a DRIVER-file mark: `autonOnly(hood)` in `controls(...)` | moves to `controls.autonOnly[]` |
| position: `gearing`, `limits`, `gravity`, `homing`, `profile`, `gains`; inches: `spoolDiameterIn` / `linkage` / `table` | **`gearing`: tooth pairs, the motor's gear first, stages in order — `[[12, 60]]`, `[[12, 36], [12, 60]]`, or `"direct"` (never a bare ratio, N11)**; `range {min, max}`; `gravity`: `cosine {holdLevelVolts, levelAtDeg}` / `constant {volts}` / `none`; `home`: `restsAtStop {at}` / `homeBySwitch {at, towardVolts}` / `homeByStall {at, towardVolts, stall}` / `absoluteSensor {at, sensorReadsDeg}` / `neverHomed`; a lift's `shape`: `spool {diameterIn}` / `linkage {armLengthIn, pivotHeightIn, stages}` / `measured [[in, deg], …]`; `presets [{name, value}]`; **`stall {amps, belowMotorRpm, lastingS}` REQUIRED (N10; amps < 2.5, belowMotorRpm > 0)**; optional `unhomedVolts`, `tuning {kP, kI, kD, kS, kV, kA, maxSpeed, maxAcceleration, within, settleS}` | rename to these; `profile` and `gains` become the optional `tuning` (N7: starting values, never required); homing needs `at` and `towardVolts`, and the push must point at its home end (U4) |
| speed: `atSpeed {bandRpm, forS}`, `gains` | `kind: "flywheel"`; `gearing` as above; `atSpeed {rpm, lastingS}` (REQUIRED), `presets [{name, rpm}]`, **`stall` REQUIRED**, optional `angleSensor`, `tuning {kS, kV, kP, kI}`; the free speed is DERIVED (cartridge through the gearing) | rename; no `maxRpm` field |
| power | `stall` OPTIONAL (amps < 2.5, belowMotorRpm > 0 when stated); `pieceSensor` optional | — |
| pneumatic: `openLevel`, `openLevelVerified`, `start`, `safe` (two), or `positions {name: [levels]}` (three) | ONE form for two and three: `positions [{name, levels: [high/low per line], confirm: grabs / releases / none}]` — **every position states one level per wired line (U2)**, `start`, `safe` (names), `moveTimeS` (REQUIRED, P6), optional `pieceSensor`, `confirmWithinS` | `openLevel` / `openLevelVerified` go; "unverified" becomes the region's comment (robot two: HA-159) |
| `wiring[] {mechanism, role, port}` | the same facts, emitted as `wire(<mechanism>, motors(…), angleSensor(n), pieceSensor(n or 'X'), homeSwitch('X'))` / `wire(<m>, lines('A', …))` in the mechanisms' order, inside the robot's namespace, after the declarations, in a list named `k<Robot>Mechanisms` of type `MechanismTable` | the JSON may stay; the emitter writes `wire(...)` |
| `controls.holds / toggles / presets` | **`whileHeld(m, fwd, back, volts)`**; `toggle(m, button)` (a two-position pneumatic) / **`toggle(m["preset"], button)` (a flywheel's NAMED speed)**; `preset(m["name"], button, timeLimitS)` (a pneumatic position takes no time limit); `homeOnPress(m, button, timeLimitS)`; `homeAtDriverStart(m, timeLimitS)`; `autonOnly(m)`; **a switch- or stall-homed mechanism with a hold or preset needs one of the two homing controls or `unhomedVolts` (U5)** | rename `holds` → `whileHeld`; add `timeLimitS` to presets; add the two homing controls; the Flywheel template's toggle names its preset |
| the handles file (its item 13: a SET region of `mech::…` handles) | none: the declarations ARE the handles (Q18 (a)); a removed mechanism fails the build at each auton line that names it (`'arm' was not declared in this scope`) | drop the handles file and its SET region; N5's probe message changes |
| (absent) the C++ identifier of a declaration | **the JSON `name` in lowerCamelCase (`"claw pivot 6"` → `clawPivot6`), overridable by an `id` field; two that collide are refused by the generator** (U11) | add the rule and `id` |
| (absent) route ids | **DERIVED, never stored** (§1.16): the generator and the panel's route editor (Part R) compute a robot's ids by `routeIdsOf`'s rule from the robot file — `<name>_<preset>` / `_home` / `_stop` / `_run` / `_until_piece` / `_until_gone` / `<name>_<position>` — and refuse what the build refuses (too long, alike, a built-in, more than 32); a marker naming an id the robot does not have is refused before saving (`3c643f1`) | mirror the rule and the four refusals, by sentence |
| `provenance` per mechanism | a comment above each declaration (robot two: one line per caveat) — no longer build-checked (Decided here) | the emitter writes it as that comment |
| `draft` | not the core's | G3a's own (a draft generates nothing) |
| `mechanismsDisjointFromChassis` (G3a :298, :2689) | **`mechanismsClearOf(list, chassisTable)`**, extended to every smart role and the IMU (§1.13) | rename the citations |

The templates map one to one: Intake → `power` (+ optional `pieceSensor`); Arm → `arm`; Lift → `lift`; Claw →
`pneumatic`, two positions, `grabs` on closed; Flywheel → `flywheel`; Catapult → `power` with a `limitSwitch()` piece
sensor ("run until loaded" is `r.run(cata, V, seesPiece, t)`); Conveyor → `power`. Every refusal G3a mirrors is
`spec.hpp`'s 60, `wiring.hpp`'s 18, `driver_inputs.hpp`'s 17 and `route_ids.hpp`'s 4 (§1.3, §1.4, §1.10, §1.16), by
their sentences; the G3a template for an arm, a lift and a flywheel must carry a `stall` (the generator cannot
invent one — P6), and its panel page asks for it with the guide's recipe beside the field.


### 6.4 The robot file is the `.vexbot` profile block (P8); F7 freezes at G3, after the rest
Every word in §6.3 becomes `.vexbot` schema the moment G3a Part 5 emits it, and F7 freezes it at G3 — after the
rest of MECH, so the rest's kinds and controls join the vocabulary before anything is frozen. Until G3a Part 5
the core's C++ is the vocabulary's only home, and a rename is a C++ change; from Part 5 to G3, a rename is a
generator + agreement-test change (G3a's validator mirrors every refusal by sentence, so a reworded refusal is
also a G3a change); after G3, a `schemaVersion` migration. So: the words above are chosen to be read by a student
in the robot file AND in a JSON field, and the brief's §6.3 table is the record G3a's re-check works from. The
VexBuilder contract §3.1 revision (roles plus a binding block) is the coordinator's note to VexBuilder (G3a §1.11),
not an executor's.

---

## 7. Checks (the coordinator re-runs every one after each run)
```sh
cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -6        # counts into the log
ctest --test-dir build/test -R compile_refusal --output-on-failure | tail -3         # every refusal names its reason
# the ARM header gate, RESUMING :107-112, unchanged (every include/shulib header, -Werror)
python3 tools/src_build_gate.py check                                                # EVERY run: seven builds link
git diff --stat -- src/ tools/                                                       # A1–A3: EMPTY; A4: the two `case` lines only
python3 tools/src_build_gate.py self-test                                            # A5, A6
grep -rn "PROBE_" include/                                                           # empty (T8)
grep -rln "internal/\|chunks/\|RESUMING\|build-order" README.md docs --include=*.md | grep -v "^docs/internal/"  # C7: empty
```
Spot checks, the single most load-bearing rule of each run, in the source: **A1** `wire()` refuses a port twice
naming it, `toMechanismConfig` signs a Reversed role (robot two's identity test), the stall pair refuses 2.5 A and
0 rpm, and `routeIdsOf` refuses a built-in; **A2** the guard's latch calls `cutClaimedMechanisms()` in the deadline
branch but never inside the end action, and in the abandon branch everywhere; `seen()` returns UNKNOWN when the
read-fault count moved; **A3** a timed-out move holds where it stopped, clamped (N2), a computed target is clamped,
never refused, and the stall watch runs during the hold; **A4** UNCONFIRMED-counted continues the chain and every
other non-success stops it as `MechanismStepFailed`; the loop ticks the service BEFORE `perTick`; a route marker
blocks through a Routine; **A5** each auton chain is guard → service → watch and the boot proof checks all three;
the gate's namespace rule refuses `robot_two::` in a program; **A6** no volt before the two-stage confirm, and the
dead-man's release safes that tick.


---

## 8. Landmines
1. **The service is in the Chassis's pacer chain, which a Chassis takes ONCE** (`robot.hpp` `BuildOptions::pacer`
   :207): construct the service BEFORE the Robot, bind it AFTER. Unbound it ticks nothing, and every step finds
   no mechanism (ABSENT) — so each program binds it inside its boot's try, before any auton can run, and a source
   pin says so.
2. **Static storage, and a task deleted mid-tick — in BOTH directions** (D1). PROS deletes the driver task on a
   mode change (HA-161); the service, `DriverInputs` and `DriverFile` live in the program's static storage. A task
   deleted mid-tick leaves the re-entrancy guard set and its behaviours running: `enterDriverControl()` clears both
   on the way into driver control, and `enterAutonomous()` (called by `runAuton`'s service overload) on the way into
   an auton — `adv_handover`: 257 of 269 auton paces at the driver's 12 V without it, 0 with it. Both pinned.
3. **One task ticks the service at a time.** The competition program's `opcontrol()` waits for its auton task
   (COMP-b); keep it. A service ticked from two tasks is a data race no test catches.
4. **One ticker per operation.** A student's `then()`-ticked F1 operation and a service step on the same
   mechanism: the claim refuses the second (REFUSED). Never "fix" it by taking the claim over.
5. **`_deg` is a wrapping heading** (270 reads −90): an arm angle is a plain number; `arm.to(90_deg)` must stay a
   build error (n07).
6. **Lookup is by NAME and KIND** (P12). Two robots that both have a `claw`: an auton for robot two compiled into
   robot X's build would command robot X's claw (same name, same kind, same position name). Autons are per robot
   — `using namespace robot_two;`, and G3a Part 4's `autonsFor(RobotId)`; do not "fix" it with an address lookup
   (a declaration without `inline` would then end steps ABSENT silently).
7. **The recorded path differs host vs brain** (finding 14): pin the suffix, never the whole path.
8. **Calibrate's watch is CAL1's (N9)**, and its rule stays theirs: never construct a `ProsMotor` for a declared
   port with a gearset (it WRITES it — T3's rule for a claw); CAL1's HA-187 holds the read. MECH-A5 only maps the
   list into CAL1's facts.
9. **The guard's DEADLINE cut never runs inside the end action; its ABANDON cut runs everywhere, the end action
   included (B1)** (D5) — and neither re-safes an UNCLAIMED mechanism each pace (HA-144: a repeated `brake()` under
   Hold re-captures the point and lets a loaded shaft creep).
10. **The per-line `PneumaticMechanism` form must leave the single-level constructor byte-for-byte as it is**
    (robot two's claw, T3); one test per form.
11. **"A pneumatic keeps its level through a disable" rests on HA-157** (whether the brain keeps a 3-wire output
    while disabled). Say it with the HA, never more.
12. **The gate's rule is PER ROBOT** (T3) — a robot's namespace and list in that robot's own files only. A broader
    one re-opens HA-111's defect class (a build carrying another robot's facts) without a single compile error; a
    new team file that belongs to one robot but does not start with its namespace goes in `TEAM_FILE_ROBOT`, by hand.
13. **Never name a robot's list `kMechanisms`** inside its namespace (finding 12): step 0 would derive the BUILD's
    selection name as a per-robot one and refuse every program.
14. **`consteval` refusals are read in GCC's words** (`call to non-'constexpr' function '…refused::…'`); the
    harness matches the function NAME, which every GCC since 10 prints. The project's compilers are GCC (host) and
    arm-none-eabi-g++ 13.2.1; a clang build is not a target, and the brief claims nothing for it.
15. **`routine.hpp` now includes `mechanism/service.hpp`.** Log the suite's and the ARM gate's compile time before
    and after; if it grows noticeably, the fallback is the steps in a small header over a forward-declared
    service (the slots' types are then needed only in the step bodies).
16. **After an auton, driver entry drops a position HOLD to its safe state** (finding 8). That is the ruled T3
    entry rule — keep it, and say it in the guide.
17. **The direction check cannot see a motor mounted the wrong way from its own encoder** (a V5 motor reverses
    command and encoder together, `pros/motors.hpp` :51-52 per station 10's note). It sees a fight, a still motor,
    a sensor against its motors — and the person's answer. The screen claims no more.
18. **Station 13 commands hardware directly; its six gates are the only protection** (an unhomed mechanism has no
    soft stop). Every gate has a test; the station is labelled POWERED, like station 10.
19. **N1 is a behaviour change for F1 operations too**: an operation with a registered claimant now stops at the
    deadline. Search the suite for a test that relied on an operation outliving the deadline; there should be
    none (S1 ran the whole suite with the cut: 2231/2231 on `tree_t43d`) — confirm at the tip.
20. **The service binds over `robot.mechanismParts().devices()`** — the SAME devices the guard's span lists. A
    program that builds its own devices for the service would leave the guard safing objects the service does not
    command. One source of devices; a source pin.
21. **(re-check) `FakeDistance`'s default (0 in, confidence 0) reads UNKNOWN** under the evidence rule (a held
    near value): a test's "no piece" sets the adapter's honest 393.66 in (`tools5/patch_far.py` did it for every
    probe). A test that leaves the default sees every `stopsSeeingPiece` run to its bound — not a defect.
22. **(re-check) The read-fault rule compares against the count at the END of the previous service tick** — not a
    before/after pair inside one read — so a fake that bumps between ticks and a real adapter that counts every
    failed read both read UNKNOWN for the tick a failure touched, and FRESH again once the device answers (R7). Do
    not "simplify" it to a per-call comparison: two reads of one sensor in one tick would then disagree.
23. **(re-check) A route marker BLOCKS its route** (it runs through a Routine and waits for the verdict): a long
    time limit on a marker is a long stop at that waypoint. Run-while-driving belongs to autons' `start…` forms and,
    later, the rest's mid-leg triggers — never to a marker that returns early and leaves a step unreported.
24. **(re-check) Two rulings moved under this brief within hours** (route ids in, the panel's preview out). The
    executor of A4 re-reads MECH-PANEL-requirements §4 item 7b at its tip before building §1.16; a further change
    there is the coordinator's to bring back, not the executor's to absorb.

---

## 9. The plan commit's drafts (Q26, Q28, the core's runs) — for the coordinator, on his go

- **Q28 — a deviations-table row** (requirements §7.3 :773-777, unchanged): "**F3/F4 re-scoped; MECH added**
  *(2026-09-28, the team lead: "I feel like it shouldnt matter what mechanism a robot has? ...")* | F3 the scoring
  primitives, F4 skills motion + reference routines, Phase F′ gated on hardware and the final mechanism decisions |
  **MECH: the generic mechanism library, not gated; F3/F4 = the students' season strategy (examples optional);
  F4's generic items placed** | P1: what a robot has decides its settings, never the library's work".
- **Q28 — open dependency #6** (:778-779): "Final mechanism / lift / role decisions — Build team — gates the season
  strategy (the students') and each robot's MECH settings, not MECH's library work."
- **Q28 — F4's generic items, placed** (requirements §3.8 :413-417): `fieldCentricStrafe` / `strafeTrim` → the
  holonomic track (R3c on the spring X-drive; I1's H-drive hardening); `moveToPoseProfiled` → tip protection (the
  rest); `endInMidfield`'s 18″ lockout → an interlock (the rest) plus the students' end action. The Phase F′
  banner gains one line pointing at the re-scope.
- **Q26 — the freeze register**: F11 (the mechanism seam), F12 (the run guard), F13 (driver input) and F14 (digital
  input) each freeze on "**MECH on hardware**" (today: F11 "F3's scoring primitives, on hardware", F12 "F4's
  routines", F13 "T3", F14 "F3's homing" — the re-scope orphans three of them); F14's open question "switch or
  stall" is recorded **answered: both** (the core builds both, P2e).
- **Q4 (b) — PTO, recorded**: the five places of §1.15, as a named later change in the plan (not a chunk).
- **build-order / REMAINING-WORK**: MECH-core becomes six entries, MECH-A1 … MECH-A6, with §2's sizes (the core
  ~19¼–26, replacing row 12's ~10½–18 — the route ids of 2026-10-08 included) and §2's order (after T4 Part 3, N12;
  CAL1 before A5, N9); the rest (rows 13–15b) unchanged, after the panel.
- **The F8 register row** (on A4's landing): the additive `CommandEntry::doc` and its `on()` overload, under API 2.5.

---

## Found while reading (not the core's to fix; recorded so nothing is silent)
- **`include/shulib/version.hpp`'s policy** (~:10-41) is written for frozen members; whether removing an UNFROZEN
  documented type needs a major bump is unruled (N4 (b) would need that ruling; TANK1's brief named the same gap).
- **G3a's brief cites `mechanism_table.hpp :624-625`** for the "only robot two" pin; after T4 Part 3 it is
  :522-523. Its §1.14 is superseded by §6.3 here; its N5 probe message (`'claw' is not a member of 'mech'`) becomes
  `'claw' was not declared in this scope`. The coordinator's §1.14 re-check (before G3a Part 5) carries both.
- **The requirements' §7.1 sizes** for the core are superseded by §2's split (the briefs' estimates, T10).
- **`runner/command_registry.hpp`'s banner** says "a V5 brain has 128 KB of user RAM" and "asynchronous mechanism
  operations are chunk T3's" — both older than the core; A4 refreshes the second (§1.16's binder), the first is
  not the core's to judge.
- **T4 Part 3's worktree** holds a staged merge of `fab1ebb` (MERGE_HEAD) on top of its early-step commit; the core
  waits for it to land and rebases nothing onto the worktree.

---

*Rewritten 2026-10-06 against `tree_t43c`; re-checked 2026-10-07 → 10-08 against `tree_t43d` and main `2fce38a`
(the amendments at the top). Probes: `.claude/coordinator/scratch/brief-MECH-core/` (`out5/`). Nothing is built; it
waits for the team lead's go and his answers to N1–N13.*
