# L9 (the rest) — reverse driving in a recipe, and booting a `robot::Robot` from the SD card as library code

> **Caller gained at COMP-t (2026-09-28; noted here, the body not edited):** "shulib Auton Tester" builds
> its Robot with `opts = boot.built.buildOptions(); opts.pacer = &*st.guard;` — the one-line boot of Part B
> must pass `options` WHOLE, `.pacer` included (a pacer is borrowed for the Robot's whole life;
> `BootConfig::buildOptions()`'s doc says so since COMP-t).

> **Paths moved at SRC1 (2026-09-27):** the `src/` files this brief names each moved to a folder, names unchanged — `src/robots/` ← `chassis_table.hpp`, `mechanism_table.hpp`; `src/programs/` ← `teleop_claws.hpp`, `teleop_program.{hpp,cpp}`; `src/common/` ← `boot_source.{hpp,cpp}`. `src/main.cpp` and `src/build_selection.hpp` stay where they were. The full mapping is in the [SRC1 brief §1](SRC1-organize-src.md); every include of a `src/` file is now written from `src/` (`#include "robots/chassis_table.hpp"`). The body below is NOT rewritten — it describes the tree it was written against; its executor re-checks every path against the tree.

> **THE TEAM LEAD'S ANSWERS (2026-09-26, ~23:10): every recommendation accepted** — "Accept all" on
> D1–D9 as written below. Builders start after tonight's claw and auton merges (his ruling of ~14:15:
> "briefs now, build after tonight"); each part still starts on its own go, at its turn in the order
> the brief gives. Committed by the coordinator with its plan entries.

> **Execution brief.** Written 2026-09-26 (evening) from a read-only pass over `shulib-v2` at `a01e3a2`:
> `chassis/routine.hpp`, `version.hpp`, the F6/F8/F10 pin tests, `runner/path_runner.hpp` and its tests,
> `motion/move_to_pose.hpp`, `math/angle.hpp`, the D1 recipe tests, the cookbook and guide chapter 9,
> `src/boot_source.{hpp,cpp}`, `src/teleop_program.cpp`, `robot/robot.hpp`, and the T3 Part 2, COMP and
> G1 Part 2 briefs. **Re-checked the same evening** against the tree and the `t3-part2` and
> `g1-2c-final` worktrees (amendments below). **STATUS: WAITS FOR THE TEAM LEAD'S GO, and decisions
> D1–D9 below need his answer first.** Two parts, each with its own executor run, verification and
> commit:
> - **Part A — reverse driving** (library + tests + docs, no `src/`). It runs in an isolated worktree
>   off `shulib-v2` **only on the team lead's explicit yes to D4, which is an exception to pace rule 1**
>   ("one thing in flight"). If D8 (a) is taken, Part A runs as **A1** (the two steps) and then **A2**
>   (the cookbook recipe), each verified and committed on its own. About ½–1 day in all.
> - **Part B — the boot from the SD card as library code.** **NOT runnable until T3 Part 2, G1 Part 2c's
>   final run and COMP-b have all merged** (each edits or calls `src/boot_source.*`). Its scope below is
>   a design; the coordinator re-checks it against the tree before its go. About 1 day.
>
> **Executor model: `opus`. Do not commit.** Live log: `docs/internal/chunks/L9-PROGRESS.md`, created
> FIRST and appended as you go (a heading per part). Reserved Hardware Assumptions number: **HA-167 only**
> (in this wave COMP-b holds HA-162, G1 Part 2c HA-163/164, DIAG1 Part B HA-165, SAFE-SAVE HA-166, H1
> HA-168/169 and L7 HA-170..174; the coordinator renumbers at merge if needed). **L9 has no entry in
> `build-order.md`** (its only mention is T3's deviations row ~:1943, which took "handoff L9's piece",
> the mechanism lookup). The handoff (§5 ~:283-286) requires an entry and a deviations row in the brief's
> commit, before building: the coordinator adds both when this brief is committed to
> `docs/internal/chunks/`.

## Re-check 2026-09-26 (evening) — amendments

The checker's ten findings (`findings-L9-REST.json`), each re-opened at its cited lines, then five more
found while re-checking. Probe programs are in `docs/internal/evidence/brief-probes-2026-09-26/` (`L9-*.cpp`).

1. **(major) "No test has ever reversed a tank" — REAL, fixed in §0 and in the handoff sentence §3 asks
   for.** `motion_primitives_test.cpp` ~:238-273 drives a tank to d = −18 in, `motion_sweep_test.cpp`
   ~:154-168 runs signed trials, and `drive_encoder_odometry_test.cpp` ~:169 tracks a reverse segment.
   What is true: a reversed APPROACH (bearing + π) has run on an X-drive only.
2. **(major) D5's rationale — REAL, D5 rewritten.** G3a emits the `src/` table shapes
   (`build-order.md` ~:1555-1557). `src/boot_source.hpp` ~:7-12 is shared by design. In `g1-2c-final`,
   `kChassis = chassisTableFor(kRobotId)` (`src/chassis_table.hpp` ~:200-214). So robot one and G3a are
   served in this repo without Part B. D5 now names the real beneficiaries and gives (c) its true cost.
3. **(major) Byte-identical lines versus generic library code — REAL, new decision D9.** The
   `t3-part2` diff adds `(src/mechanism_table.hpp)`, `src/chassis_table.hpp and src/mechanism_table.hpp
   disagree`, the claw wording and `kDeclaredMotorLabel`, and it adds `calibration::DeclaredMotorPorts`
   as an input. One narrowing: "run shulib Calibrate" is ALREADY library text (`boot_check.hpp`
   ~:278-380), so the program-specific words are "(slot 4)", the `src/` file names and the claw
   vocabulary. `DeclaredMotorPorts` is now a named `planBoot` input, derived from the base config.
4. **(minor) Surviving mutations — REAL, fixed in §2.** (a) `plantConfig()` leaves `initialPose` at the
   origin (`motion_test_rig.hpp` ~:47-51, ~:111), and runner 10 runs from the origin at bearing 0. The
   coordinator's probe (`wrap_probe.cpp`) shows that after `Angle`'s wrap, `b + π` and `b − π` are
   bit-identical at bearing 0 and at about 59 % of a uniform sweep, and differ at the rest. One case
   where they differ is the acceptance route's own first leg, (−24, −12) → (0, 24) (`geom_probe.cpp`).
   There is now a geometry rule. (b) A starved `faceAway` was added to A6. (c) There is a new options
   case, A12. Mutations 13–14 were added.
5. **(minor) The memcmp of `RobotConfig` — REAL, fixed.** `boot_source_test.cpp` ~:5-14 forbids it
   (padding) and carries member-count pins (~:236-255). Part B reuses them.
6. **(minor) The PROS shim "call log" does not exist — REAL, fixed at the root by D7 (a).** `UsdState`
   holds only `installed` (`pros_shim/pros/misc.hpp` ~:106-114). `RegistryState` holds `kinds`,
   `queries` and `lastIndex` (`apix.h` ~:68-78). `ProsBlockSource` uses real `fopen`/`fclose`. The
   planner now takes an already-read record, so "the card is closed before the registry is asked" is
   structural, and it is checked by reading. The mutation that could not be observed is dropped.
7. **(minor) `hal/pros/` → `robot/` include direction — REAL, now a choice in D7.** Every `hal/pros/`
   header includes only `core`, `diag`, `hal`, `math`, `motion` and `units` (grep tally). The master
   plan §5 says "Dependencies point down / inward only".
8. **(minor) F10 pin count and the version comment — REAL, fixed in §3 and A9.** The file has 38
   `SHULIB_F10_PIN(` plus 12 `SHULIB_F10_NOT_NOEXCEPT(` invocations, 50 in all (`briefing_status.py`
   `pin_counts`; `PROJECT-BRIEFING.md` ~:144 "F10 50"). The roadmap's "37 pins" has been stale since
   F1. A9 may reword the comment at ~:388-389 (see item 11).
9. **(minor) Documentation gaps — REAL, all five fixed in §3.** The cookbook README has a "4 —
   Drivetrains" table (~:66-71), not a mapping line. Cookbook 04 ~:7-8 names only 04a/04b. Guide 09's
   stability notice is at ~:10-20. The `path_runner.hpp` comments at ~:24-27 and ~:187-188 are outside
   the `docs/` grep. The document count is PAST pace rule 7, not at it, so there is new decision D8.
10. **(minor) Collisions, rules, checks — REAL, all six fixed.**
    - D4 is now framed as an exception to pace rule 1, with its precedent.
    - T3 Part 2's file list is completed from its `git diff`, including `boot_check.hpp`.
    - H1's `version.hpp` ~:38-41 edit is noted in D3 and in the landmines.
    - `docs/api/all-entities.md` and `docs/api/README.md` are regenerated at every merge.
    - The subcommand is `check-coverage`. `git diff --stat -- src/` is the no-`src/` evidence.
    - A2's per-tick truth recording is specified.
11. **(found in the re-check) G1 Part 2c-final edits `test/routine_signature_pin_test.cpp` ~:385-386**
    (and the F6 twin ~:415-420). That is directly above the comment A9 must reword. It also edits
    `docs/vexbuilder-integration.md` ~:61, a different section from Part A's conditional sentence.
    So Part A shares a test file with in-flight work. D4 now weighs this.
12. **(found) HA-167's Source cited the wrong entries.** It cited HA-137 (a coupled train's mean-volts
    model) and HA-45 (the feedforward placeholders) for "direction-symmetric, no backlash". Neither
    entry says that. The plant's own model does: `sim/motor_model.hpp` ~:21-26 (V = kS·sign(v) + kV·v,
    with no backlash term). The Source is re-cited.
13. **(found) `routine.hpp` is also included by three `competition/` headers** (COMP-a:
    `match_runner.hpp`, `acceptance_run.hpp`, `auton_registry.hpp`). So the stale-binary touch set is
    larger than "the tests that include it". Landmine 4 is widened.
14. **(found) The handoff words the ask as "an ADDITIVE option on `Routine::driveTo`"** (§5 ~:449-452).
    D1 now says which spelling that is, and why (a) departs from it. D1 (b)'s address-of cost is also,
    read strictly, the policy's own definition of BREAKING (`version.hpp` ~:14-16).
15. **(found) Part B's pins and neighbours grew.**
    - T3 Part 2 adds boot tests 6s–6v, and the claw lines are pinned there. "(slot 4)" is already
      pinned at HEAD (6n ~:747, 6r ~:813).
    - COMP B8 may itself reword `BootConfig::buildOptions()` or add `buildOptions(ITickPacer&)`.
    - Guide chapter 8 is also 2c-final's file.

    All three are folded into D6, D9 and Part B.

Verified as the checker stated, and unchanged: every other file:line citation, D1 (b)'s
overload-resolution argument, D3's reading of the changelog, and that HA-167 is free.

## Decisions for the team lead (a recommendation for each; nothing below decides them silently)

**Part A — reverse driving**

- **D1 — How reverse is spelled.** `Routine` is frozen (F10), and `version.hpp` ~:14-28 allows only
  additive growth without a major bump. The handoff (§5 ~:449-452) asks for "an ADDITIVE option on
  `Routine::driveTo`". Of the four spellings weighed below, only (b) is literally an option on `driveTo`
  that the policy calls additive. The literal reading, (c), is BREAKING.
  - **(a) RECOMMENDED: two new steps, `reverseTo(x, y, options)` and `faceAway(x, y, options)`.**
    - The F10 banner (`routine.hpp` ~:17-20) names "new steps" as the intended growth path.
    - `driveTo` and `face` stay byte-identical, and so do their pins.
    - The stop and skip lines name the step, so a transcript says `(reverseTo)`, not `(driveTo)`.
    - Other names are fine if he prefers them (`backUpTo`, `driveBackwardTo`). Rejected:
      `driveBackTo`, because "drive back to the start" reads as "return to".
  - (b) An overload taking an enum, `driveTo(x, y, Direction::Reverse, options)`, plus the same for
    `face`. The policy counts a new overload as additive, but it has two costs.
    - First, an existing call `r.driveTo(x, y, {})` would silently re-resolve to the NEW overload. An
      enum from `{}` is an identity conversion, and an aggregate from `{}` is a user-defined one
      ([over.ics.list]). The re-resolved call behaves the same only while `Forward` is the zero
      enumerator. No call in the tree is spelled that way today (grepped).
    - Second, `auto p = &Routine::driveTo;` compiles today and would not. Read strictly, that meets the
      policy's own definition of BREAKING ("any change that can make a previously-compiling user
      program fail to compile", `version.hpp` ~:14-16), so (b) sits on the line between the policy's
      two bullets.
    - It also reads worse: `.face(x, y, chassis::Direction::Reverse)`.
  - (c) Rejected: a defaulted fourth parameter on `driveTo`. It changes the member's TYPE, and the F10
    pin (`routine_signature_pin_test.cpp` ~:100-105) fails naming F10. That is a BREAKING change
    (kApiMajor 3).
  - (d) Rejected: a `reverse` field on `MotionOptions`. That grows F6 (`chassis.hpp` ~:184), and six of
    the eight verbs that take the struct would silently ignore the field.
- **D2 — Include `faceAway`. Recommended: yes.** On a tank, `moveTo` does not plan turns. The body's
  sideways command is clamped to zero, and the forward component is the projection of the field
  command on the heading (`move_to_pose.hpp` ~:196-220). So a reversed approach is a straight line only
  if the robot has first turned its BACK to the point. That is the author's turn (C1's D12; the tank
  cookbook recipe ~:30-37). Without `faceAway`, a tank author has to write
  `turnTo(atan2(...) + 180°)` by hand, which is exactly the arithmetic `face()` exists to remove.
- **D3 — The version moves to 2.3. Recommended: yes.** This is the first growth of a FROZEN surface
  since G2. `version.hpp` ~:24-28 says additive growth of a frozen surface bumps `kApiMinor`, as F1
  did for F10 (2.0 → 2.1). The "since 2.2, no bump" entries in the changelog (DIAG1, COMP-a, T3 Part 1)
  each grew an UNFROZEN surface. So H1's D7 question (policy versus practice) does not arise here: both
  readings bump. Only the three pin files read the version (grepped: no golden, tool or doc prints
  "2.2").
  - **Merge consequences:**
    - The changelog gains a `## API 2.3` heading. Any chunk that merges AFTER Part A (whichever of
      T3 Part 2, COMP-b, H1 and L7 are still out) must have its "since 2.2" entry re-headed "since
      2.3" under the new heading.
    - H1 Part B edits `version.hpp` ~:38-41 (F9's comment), and Part A edits ~:50-78. These are two
      hunks of one file, merged by hand.

    The coordinator does both at merge.
- **D4 — Part A runs before the in-flight work lands, as an EXCEPTION to pace rule 1. Recommended:
  (a).** Pace rule 1 (RESUMING) says "one thing in flight". The precedent is the team lead's
  2026-09-26 "start all three" (build-order's deviations row for G1 Part 2c, DIAG1 and COMP-a).
  Part A touches no `src/` file, no `tools/` file, no worksheet and no header another chunk has in
  flight. It does share one test file: G1 Part 2c-final edits `routine_signature_pin_test.cpp` ~:385-386,
  directly above the comment A9 rewords (~:388-389).
  - **(a) RECOMMENDED:** yes, as an explicit exception, branched off `shulib-v2` AFTER G1 Part
    2c-final merges. 2c-final is the only in-flight change to a file Part A edits, and it is next in
    line. Cost: Part A starts when 2c-final lands, not tonight.
  - (b) Yes, now, off `a01e3a2`. The coordinator hand-merges one adjacent hunk in the pin test's
    version comment. It is small, but it is the comment that explains the version pin, so it must be
    read, not auto-merged. Cost: one hand merge. The shared-document blocks need reconciling in any
    case.
  - (c) No: Part A waits until T3 Part 2, 2c-final and COMP-b have landed (the rule as written). Cost:
    routine authors have no recipe-layer reverse until then, including for the first autonomous
    (COMP-b's Station K).
- **D8 — Part A's documentation load is past pace rule 7. Recommended: (a).** Rule 7 (RESUMING
  ~:220) says to split a chunk whose documentation runs past about five documents. Part A as drafted
  touches six public documents, seven if A4 requires the VexBuilder sentence, plus three internal
  ones.
  - **(a) RECOMMENDED: split.**
    - **A1** is the two steps, the version, the pins, tests A1–A10 and A12, guide 09, the changelog,
      the roadmap, HA-167, the conditional VexBuilder sentence, and the internal documents. That is
      four public documents, or five.
    - **A2** is the cookbook recipe "Back into a point": cookbook 04, the cookbook README's table row,
      and the cookbook test's banner and case (A11). That is two public documents.
    - Each part gets its own verification and commit.
    - Cost: a second verify and commit cycle. Guide 09 links to the recipe only when A2 lands (A2
      adds the link).
  - (b) Accept the overrun in one commit and say so in the commit body. Cost: rule 7 exists because
    a documentation pass that runs long goes stale (RESUMING's own example, GATE1).

**Part B — the one-line boot**

- **D5 — Does the library expose the boot, and in what shape? Recommended: (a).** Nothing is lost by
  taking this decision at Part B's re-check instead, because Part B cannot start before then (D6).
  - **(a) RECOMMENDED:** two PROS-free library pieces.
    - `robot/boot.hpp` holds today's decision, moved: the SD record, verified against the registry →
      else the typed fallback, verified → else refuse, with the cross-check.
      - It is parameterised by a BASE `config::RobotConfig`. The program states everything in the
        base (identity, odometry, sensors, mechanisms, tuning), and the base's tank description is
        the typed fallback. The card's record replaces that description when it is chosen.
      - It takes an already-read record and an `hal::IDeviceRegistry&` (D7).
    - `robot::buildInto(std::optional<Robot>&, cfg, devices, options) -> BuildVerdict` does
      validate → emplace, and catches `PreconditionError` into the verdict. This is the "graph at its
      boundary" block that Teleop writes (`teleop_program.cpp` ~:848-867) and COMP-b will write
      again.
    - A program's boot becomes: read the card, plan (which prints its own decision lines), then
      build. COMP inserts its `RunGuard` between plan and build (COMP brief B8 ~:77-82).
    - **Who actually gains:**
      - any program that is NOT in this repository: another team using shulib, and the guide's own
        teaching path;
      - G3's `.vexbot` → `robot_config.hpp` codegen (`build-order.md` ~:1576), whose output is a
        `RobotConfig`, which is exactly the base `planBoot` takes.
    - **Who does not gain:** robot one and G3a in this repository.
      - After 2c, a new robot is a `RobotId` case plus its tables (`chassisTableFor`, `g1-2c-final`
        `src/chassis_table.hpp` ~:200-214).
      - G3a generates those `src/` tables (`build-order.md` ~:1555-1557).
      - So both reuse `src/boot_source` unchanged.
    - Cost:
      - The decision code moves into a public header with its own `docs/api` page. That is `src/`'s
        382 + 55 lines today, and T3 Part 2 adds about 230 more.
      - 6a–6v are re-pointed at it.
      - D9's wording question follows.
      - It creates one more unfrozen public surface to maintain.
  - (b) A literal one-liner that builds the provider AND the Robot into caller storage. Rejected:
    - COMP must build its guard AFTER the provider and BEFORE the Robot (`robot.hpp` ~:188-199: the
      Chassis takes its pacer once), so COMP could not use it.
    - It would also hide the lifetimes: the provider must outlive the Robot, and the pacer is borrowed
      for the Robot's life.
  - (c) Expose nothing. A guide section shows the boot assembled from the library pieces that are
    already public (`calibration::decodeRecord`, `verifyAgainstRegistry`, `chooseBootSource`).
    - Zero risk to work in flight, and no new surface.
    - True cost: a program outside this repository re-writes the flow, or copies
      `src/boot_source.hpp`. The flow is the order, the refusal that never falls back to the table,
      the cross-check and the absent-port door, about 150 lines of glue. The copy would be a file in
      `namespace shulib::bench` that includes `src/` tables.
    - Inside this repository: no cost.
  - **Never a `Robot::fromCard()` factory that returns a Robot.** `robot.hpp` ~:50-59 already rejected
    returning a Robot by value (it is non-movable; "a Robot is constructed WHERE IT WILL LIVE"), and
    this brief does not re-open that ruling.
- **D6 — When Part B runs. Recommended: after T3 Part 2, G1 Part 2c-final and COMP-b have merged, with
  the coordinator re-checking this section against the tree first.**
  - T3 Part 2 is rewriting the pure core right now. It takes mechanisms from `src/mechanism_table.hpp`,
    adds a `DeclaredMotorPorts` input to the port check, refuses a claw port that the chosen
    description lists as a drive port, and adds tests 6s–6v (the `t3-part2` diff).
  - 2c's follow-ups edit `boot_source.cpp` ~:13. 2c-final also edits guide chapter 8, which is Part
    B's listing.
  - COMP-b calls `decideBootSource`, and its B8 may reword `BootConfig::buildOptions()` or add
    `buildOptions(ITickPacer&)`. Part B adopts whichever landed.
  - Rejected: building the library copy now and collapsing `src/` later. That leaves two owners of
    the boot rules for as long as the copy lives, and the first copy would already be stale.
  - **Also:** L7 Part D edits `src/boot_source.*` (the L7 brief ~:114). Part B should land before
    L7 Part D, or L7's brief must be re-checked after Part B.
- **D7 — Names, and where the card-and-registry glue lives. Recommended: (a).** The names: `robot/boot.hpp`
  holds `robot::readCalibrationRecord`, `robot::planBoot`, `robot::BootPlan`, `robot::BootWording` (D9)
  and `robot::buildInto`. He may prefer other names.
  - **(a) RECOMMENDED: no new PROS header.**
    - `robot::readCalibrationRecord(hal::IBlockSource&, path)` reads the card.
    - `robot::planBoot(const RecordRead&, const hal::IDeviceRegistry&, base, wording, printLine)`
      decides.
    - The program writes today's `boot_source.cpp` ~:36-41 block: a scope that constructs
      `hal::pros::ProsBlockSource`, reads it and closes it, then `hal::pros::ProsDeviceRegistry`.
    - The order (card closed before the registry is asked) is then structural, and visible in six
      lines.
    - There is no new include direction and no CI change, and the guide's listing compiles on the
      host against the fakes.
    - Cost: every program writes those six lines. Robot two's two programs share one copy, in
      `src/boot_source.cpp`.
  - (b) `hal/pros/card_boot.hpp` with `planBootFromCard(base, ...)`: one call per program.
    - Cost: the first include from `hal/pros/` UP into `robot/` and `calibration/` (master plan §5:
      dependencies point down only).
    - And its "card closed before the registry" claim needs an order seam that `test/pros_shim/`
      does not have, which would be a test-infrastructure change of its own.
  - (c) A new PROS-bound directory (for example `robot/pros/`). It keeps `hal/pros/` at its layer, but
    CI's PROS-include grep exempts `hal/pros/` by exact path (`ci.yml` ~:50-51), so CI changes. The CI
    file is G1 Part 2c-final's, in flight.
- **D9 — The words the boot prints. Recommended: (i).**
  - What is program-specific today, counting T3 Part 2's version:
    - "(slot 4)" in the TYPED TABLE warning, pinned by 6n and 6r (~:747 and ~:813 at HEAD);
    - `(src/mechanism_table.hpp)` in the mechanism-table refusal;
    - "claw", `recalibrate: shulib Calibrate (slot 4) never takes a declared claw port` and
      `src/chassis_table.hpp and src/mechanism_table.hpp disagree` in the claw/drive refusal, pinned
      by 6v;
    - `kDeclaredMotorLabel`, "claw pivots (declared, not drive motors)", in the port-check line. 6u
      pins it, and the worksheet quotes it after T3 Part 2.
  - What is not in question: "run shulib Calibrate" is already library text (`boot_check.hpp`
    ~:278-380). The 2026-09-25 capture's lines (6k) carry none of the words above.
  - **(i) RECOMMENDED: the program supplies them.**
    - A `robot::BootWording` carries the calibrate hint, the typed table's name, the mechanism table's
      name, the declared-motor label and the mechanism noun. Its library defaults are generic.
    - Robot two's `src/` passes today's words, so every line stays byte-identical. 6a–6v and the
      worksheet's quotes then stay pinned, unchanged.
    - Cost: a wording struct in a public API, and every future line needs a slot in it.
  - (ii) Generic library lines.
    - Cost: the expected lines of 6n, 6r, 6u and 6v, and the worksheet's quoted boot lines (Stations
      G and J after T3 Part 2), are re-derived and re-edited.
    - Robot two's printed boot changes text between two sessions.
    - The move's "byte-identical" pin is lost for exactly the lines that changed.

## Do not touch — work in flight

- **T3 Part 2** (worktree `t3-part2`, from its `git diff`):
  - source: `src/teleop_program.*`, `src/boot_source.*`, `src/teleop_claws.hpp` (new),
    `src/mechanism_table.hpp`, `include/shulib/teleop/mechanism_binding.hpp` and `teleop_loop.hpp`,
    and `include/shulib/calibration/boot_check.hpp`;
  - tests: `boot_check`, `boot_source`, `mechanism_binding`, `mechanism_table`, `toggle_binding` and
    `teleop_claws` (new);
  - documents: `docs/changelog.md`, the worksheet (`R3a-BENCH-WORKSHEET.md`), `README.md`,
    `docs/api/*`, guide chapters 7, 10, 11 and 14, cookbook 06, the hardware assumptions, the roadmap,
    `build-order.md`, the handoff and the briefing.
- **G1 Part 2c's final run** (worktree `g1-2c-final`, diffed against its base `cc513a7`):
  - source: `src/main.cpp`, the new `src/build_selection.hpp` (it picks the program AND the robot), the
    two robot tables and every `src/*_program.*`, `tools/src_build_gate.py`, `Makefile` and
    `.github/workflows/ci.yml`;
  - headers: `robot/robot.hpp`, `config/robot_config.hpp`, `diag/session_info.hpp`,
    `hal/pros/device_provider.hpp` and `teleop/stick_mapping.hpp`;
  - tests: **`test/routine_signature_pin_test.cpp` ~:385-386** and `test/f6_signature_pin_test.cpp`
    ~:415-420;
  - documents: guide chapters 7, 8 and 13, `docs/vexbuilder-integration.md` ~:61, and the shared
    documents.
- **COMP-b** (next): the competition program in `src/`, `main.cpp`'s chain, and the worksheet's Station K.
- **H1** (briefed, not started): `version.hpp` ~:38-41, among others.

So **Part A touches no `src/` file, no `tools/` file, no worksheet, no CI file and no header in
flight.** It shares `test/routine_signature_pin_test.cpp` with 2c-final (D4). Part B touches
`src/boot_source.*` and the two programs' boot blocks, but only after all three have merged (D6).

Shared documents: `changelog.md`, `hardware-assumptions.md`, `roadmap.md`, `build-order.md`, and
`HANDOFF-remaining-work.md`. T3 Part 2 edits the handoff's named-mechanism bullet, which comes right
after the L9 reverse bullet. Write L9's changes as self-contained blocks:
- a new changelog section;
- the F10 row;
- one HA entry, its index row, and ONE appended status sentence;
- L9's own build-order entry;
- one appended sentence on the handoff's L9 reverse bullet.

Do not rewrite anyone else's paragraph. The coordinator reconciles the first `**Next:**` block and the
roadmap's "You are here" at merge. The generated `docs/api/all-entities.md` (its entity count) and
`docs/api/README.md` are regenerated again at every merge, because T3 Part 2 and 2c-final regenerate
them too.

## 0. Why

- **Reverse driving.** Handoff §5 L9 (~:448-458) wants the readable routine to back into a point.
  - The data path already can. `Waypoint::reverse` (`route.hpp` ~:281) makes `PathRunner` aim at
    `bearing + π` (`path_runner.hpp` ~:236-245).
  - The recipe layer cannot. `Routine::driveTo` always arrives FACING the point (`routine.hpp`
    ~:227-239), and `face` always turns toward it (~:253-263).
  - So today a student who writes routines as C++ cannot say what a route file can.
  - **Found while reading (corrected at the re-check):** the handoff's "which the runner honours" is
    proven for a reversed APPROACH (arriving at bearing + π) on an X-DRIVE only
    (`path_runner_test.cpp` ~:714-750, `xDrive(7 in)`, from the origin at bearing 0).
    - Tanks HAVE driven backwards in tests: C1's along-axis legs to a target behind the robot
      (`motion_primitives_test.cpp` ~:238-273, d = −18 in; `motion_sweep_test.cpp` ~:154-168, signed
      trials), and odometry's reverse segment (`drive_encoder_odometry_test.cpp` ~:169). But every one
      of them holds the start heading. None arrives turned around onto a bearing + π.
    - Both fall robots are tanks. On a tank, the runner's tank routes carry their turns as authored
      `TurnTo` waypoints (~:184-233), because `moveTo` never plans a turn.
    - Part A adds the first reversed-APPROACH evidence on a tank (A2, A3).
- **The one-line boot.** The handoff (~:456) says "may be L1's `bootConfig()` exposed; decide in
  L1/L2". Neither G1 nor COMP decided it:
  - COMP-b's brief boots through `src/boot_source.hpp`'s `decideBootSource(kTag)` (B8 ~:77-82).
  - That file's banner says two programs need it (~:7-12). It is the SHARED `src/` header by design,
    so inside this repository every program already reaches it (D5).
  - Its decision flow is PROS-free and host-tested (`boot_source_test.cpp` 6a–6r, ~:320-802; 6s–6v
    after T3 Part 2). But it lives in `src/`, in `namespace shulib::bench`, so no program outside this
    repository can reach it.

## 1. Scope — IN

### Part A — reverse driving (`include/shulib/chassis/routine.hpp`, `include/shulib/version.hpp`)

1. **`Routine& faceAway(units::Length x, units::Length y, const MotionOptions& options = {})`** —
   rotate in place so the robot's BACK points at the field point (x, y).
   - It is exactly one `Chassis::turnTo`, to `bearingTo(x, y) + math::Angle::radians(math::Angle::kPi)`.
   - Like `face`, the bearing is computed when the step RUNS, from the live estimate.
   - Written in the shape of `face` (~:257-263): `skipIfStopped("faceAway")` first, then
     `runMotion("faceAway", ...)`. That is TWO name literals, and both are tested (A6).
2. **`Routine& reverseTo(units::Length x, units::Length y, const MotionOptions& options = {})`** —
   drive BACKWARDS to (x, y), arriving with the robot's back toward the approach.
   - It is exactly one `Chassis::moveTo`, to `Pose2d{x, y, bearing + π}`, in the shape of `driveTo`
     (~:232-239).
   - **The `+ π` expression is character-for-character `path_runner.hpp` ~:241**, so a route and a
     recipe that name the same leg stay bit-identical. That is the G2 rule (`path_runner.hpp`
     ~:20-27), and test A3 pins it.
   - Both steps pass the caller's `options` through WHOLE: every field, not just the timeout (A12).
3. **Share `bearingTo` (~:445-453); do not duplicate it.** Its precondition message may be reworded to
   name all four field-vocabulary steps. Wording is not frozen, and no test pins the text (grepped).
4. **The F10 banner** (~:6-20) changes from ELEVEN to THIRTEEN steps. It says the two were APPENDED at
   L9 (API 2.3) by the documented additive path. The "field vocabulary" paragraph (~:67-72) changes
   from two steps to four, each still exactly one Chassis call. Carry over, unchanged, the sentence
   saying that a hand-written `turnTo(atan2(...))` is bit-identical.
5. **`version.hpp`:**
   - `kApiMinor = 3`, `kApiVersionString = "2.3"`.
   - A `/// 3 = chunk L9's additive growth (<date>)` note naming the two steps, in the style of the 1 and
     2 notes (~:52-74).
   - **Keep the Q6 sentence ("whose major-bump question is open", ~:71-74) exactly as open as it is.**
     The 2.3 note must not resolve it by implication.
   - Do not touch ~:38-41 (H1's).
6. **`path_runner.hpp`, COMMENTS ONLY** (no code line moves; the runner's behaviour is pinned by its
   tests):
   - ~:24-27 says the bearing is duplicated because `Routine::bearingTo` is private, which stays true.
     But "`routine.hpp` is FROZEN — F10's pins fail the build if it is touched" was already wrong: F1
     grew it additively, and L9 does too. Say instead that F10 is frozen and grows only additively.
   - ~:187-188: `reverse` is now also exactly `Routine::reverseTo`. This is a `///` comment, so
     `docs/api/path_runner.md` regenerates.
7. **Forbidden:** `driveTo`, `face`, `Chassis`, `MotionOptions` and `PathRunner` do not change shape
   or behaviour. The existing D1 twin, D1 sugar and G2 differential cases are the pin: green, and
   unedited.

### Part B — the boot from the card (design; re-checked at its go, D6; written for D5 (a), D7 (a), D9 (i))

1. **`include/shulib/robot/boot.hpp`** (PROS-free). Move the following from `src/boot_source.hpp` (as
   T3 Part 2 leaves it), with every decision and every printed line's text and order UNCHANGED:
   - `RecordRead` / `readRecord` (~:216-255 at HEAD), renamed `readCalibrationRecord`;
   - `countPortDisagreements` (~:174-183);
   - `BootOutcome` + `BootConfig` (~:104-131, ~:193-213), merged into `BootPlan`, keeping the absent
     ports behind ONE door (~:44-53);
   - `decideBootSourceFrom` (~:295-374), renamed `planBoot`. It takes a BASE `config::RobotConfig` and
     a `BootWording` in place of `fromTable`, the mechanism table and `robotName`.

   In `planBoot`:
   - A non-tank base is refused BY NAME, because a calibration record describes only a tank.
   - The chosen description replaces `base.drivetrain.tank.description`, and nothing else in the base
     changes. That replaces the hard-coded "drive encoders, no GPS" of `bootConfigFrom`
     (~:133-166).
   - The port check's `calibration::DeclaredMotorPorts` is DERIVED from the base: every motor port of
     every base mechanism, as magnitudes, labelled `wording.declaredMotorLabel`.
   - T3 Part 2's refusal of a declared mechanism motor port that the chosen description lists as a
     DRIVE port becomes a GENERIC rule over the base's motor mechanisms, worded by `BootWording`.
   - T3's table-to-config conversion and `mechanismTableProblem` STAY in `src/`: they read `src/`'s
     `MechanismTable` type, which the library does not know. They run before `planBoot`, which moves
     the "mechanism table: REFUSED" line ahead of the record line. That refusal is `static_assert`ed
     unreachable in every build (the two asserts after `mechanismTableProblem`, `t3-part2`), so only
     test 6t sees the order change. The coordinator confirms this at the re-check.
   - `BootPlan::buildOptions(motion::ITickPacer* pacer = nullptr)` covers COMP B8's pacer. Its doc
     comment says the pacer is borrowed for the Robot's life (`robot.hpp` ~:188-199). If COMP-b
     already changed `buildOptions`, keep its form.
2. **`robot::buildInto(std::optional<Robot>& storage, const config::RobotConfig&, hal::IDeviceProvider&,
   const BuildOptions& = {}) -> BuildVerdict`:**
   - The precondition is that `storage` is empty.
   - It runs `validate()`, then `emplace`, and catches a `PreconditionError` into the verdict.
   - On any refusal, storage is left EMPTY.
3. **No library PROS header (D7 (a)).** `src/boot_source.cpp` keeps the card-and-registry block
   (~:34-53 today): a scope that constructs `ProsBlockSource`, calls `robot::readCalibrationRecord` and
   closes the card; then `ProsDeviceRegistry`; then `robot::planBoot`. The order is checked by reading
   the six lines, and the PROGRESS log records that reading. *(If he picks D7 (b) instead:
   `hal/pros/card_boot.hpp`, and first an order seam in `test/pros_shim/`, logged as test
   infrastructure.)*
4. **`src/boot_source.{hpp,cpp}` shrinks** to what is robot two's own:
   - `kChassis` (through `tableToDescription`, the only path from the table into a graph) and T3's
     mechanism table, converted into the base config;
   - `kChassis.robot` as the base's identity;
   - robot two's `BootWording`, which is today's words.

   Teleop and the competition program then boot through the library calls. Their printed lines must
   be byte-identical to today's.

## 2. Tests (adversarial) and mutations

Grade motion on GROUND TRUTH (`h.truePose()`), never on the estimate the motions read. Part A's new
cases go in a new `test/routine_reverse_test.cpp` (tests are globbed, `test/CMakeLists.txt` ~:77).
The one exception is A3, which goes beside G2 test 10 in `path_runner_test.cpp`, reusing its
`RecordingPacer` and `requireIdentical` (~:80-134).

**The geometry rule (every Part A motion case: A1, A2, A3, A5, A6, A12).** Start OFF the origin with an
off-axis bearing, as G2 runner 9 does ("deliberately not 0").
- The default is `startPose()`'s position (−24, −12) toward (0, 24). That is the acceptance route's
  first leg, at bearing 0.98279… rad.
- At that exact bearing, `b + π` and `b − π` DIFFER bitwise after `Angle`'s wrap (−2.1587989303424644
  against −2.158798930342464; `docs/internal/evidence/brief-probes-2026-09-26/L9-geom_probe.cpp`). At bearings 0,
  0.4636… and π/4 they are identical.
- Another geometry is fine if the executor probes it the same way and logs both values.
- Why this matters: from the origin, mutation 3 (the bearing taken from the origin) IS the live
  bearing, and at bearing 0, mutation 7 is exactly equivalent. `plantConfig()` defaults to the origin
  (`motion_test_rig.hpp` ~:47-51).

**Part A:**
- **A1 — the oracle, on a tank.** `faceAway(x, y).reverseTo(x, y)` must equal a FRESHLY hand-written
  `turnTo(atan2(dy, dx) + π)` followed by `moveTo({x, y, atan2' + π})`, bit for bit. Compare truth
  and clock, as D1's sugar case does (`chassis_recipe_test.cpp` ~:358-394). Pass identical, non-default
  options to both arms (a speed cap on each step).
  - Add the truth anchors: the robot arrives within 1.0 in, its final heading is within 0.10 rad of
    bearing + π, and it moved the way the field says.
- **A2 — it really drives BACKWARDS (tank).** Sample truth on every tick of the `reverseTo` leg.
  - Record with a pacer LOCAL to `routine_reverse_test.cpp`, in `RecordingPacer`'s shape: sample
    after the tick and before the plant steps, hard-capped.
    - `PlantPacer` (`motion_test_rig.hpp` ~:148-170) records only a path length, and
      `RecordingPacer` lives in `path_runner_test.cpp`'s anonymous namespace.
    - Moving `RecordingPacer` into `motion_test_rig.hpp` instead is acceptable if logged. That header
      has 35 includers, all of which join the stale-binary set.
  - The true heading stays within ±10° of bearing + π on EVERY tick, so it never turned around.
  - The backward displacement along the heading is at least the leg length minus 1 in.
  - The total forward movement is at most 1 in.
  - The executor sets the exact bounds from a measurement and logs them. The ±10° band is the
    discriminating assertion.
- **A3 — route ≡ recipe, bit-identical (memcmp truth, exact volts, the same tick count).**
  - On a TANK: a route [Start, TurnTo(bearing + π computed at authoring time, as `tankAcceptanceRoute`
    does ~:197-233), DriveTo reverse] against [`startAt`, `turnTo(same angle)`, `reverseTo`].
  - On the X-drive: [Start, DriveTo reverse] against [`startAt`, `reverseTo`].
- **A4 — a tank WITHOUT `faceAway`.** From a robot FACING the point, `reverseTo` asks for a 180° heading
  change while translating. `Angle::errorTo` resolves an exact antipode to +π (`angle.hpp` ~:9-12).
  - MEASURE what the host test plant does: the verdict, the time, the largest distance from the
    straight line, and the heading excursion. Log it.
  - Pin exactly the claim the docs will make, and nothing the docs do not say.
- **A5 — the X-drive:** `reverseTo` alone arrives on truth with heading bearing + π. On a holonomic
  drive, rotating in flight is expected.
- **A6 — the error policy.**
  - A `reverseTo` starved of time STOPS the chain: cause `MotionFailed`, exit `TimedOut`,
    `stoppedName == "reverseTo"`, and the Warn line contains `(reverseTo)`.
  - **A `faceAway` starved of time does the same, naming `faceAway`.** Each step has two name
    literals, and only a starved step reaches the `runMotion` one.
  - Later steps are skipped, one Info line each. `faceAway` and `reverseTo` on a stopped chain are
    skipped and named in their own Info lines, and the drive is safed.
- **A7 — misuse.** `faceAway` / `reverseTo` at the robot's own position throw `PreconditionError`, and
  the counters stay untouched (as in D1 misuse ~:811-830). A NaN timeout throws through.
- **A8 — compile-time.** In `chassis_recipe_test.cpp` ~:186-206 style: `(double, double)` does not
  compile and `(Length, Length)` does.
- **A9 — the F10 pins** (`routine_signature_pin_test.cpp`) are ADDED, and no existing PIN is edited:
  - member-pointer casts for both steps;
  - `SHULIB_F10_NOT_NOEXCEPT` for both;
  - both terse spellings in `F10TerseSpellings`;
  - the non-const pin extended;
  - in the TEST_CASE (~:390-396), `CHECK(shulib::kApiMinor >= 3)`, with the comment in F8's words
    (`f8_signature_pin_test.cpp` ~:473-476, floor ~:488-489): "F10 grew additively at 2.3".
    - **The one existing-TEXT edit allowed in this file:** the comment at ~:388-389 ("kApiMinor is
      deliberately unpinned") becomes "not pinned to a value; it has a floor".
    - The lines right above it (~:385-386) are G1 Part 2c-final's (D4).
- **A10 — doc fidelity** (`api_reference_fidelity_test.cpp` ~:141-197): `docs/api/routine.md` renders
  both signatures, and both are in the documented-members list.
- **A11 — cookbook-04c** (below; Part A2 if D8 (a)) compiles and runs, and asserts every claim its
  prose makes: arrival, backwards the whole leg, and any time claim checked against the host clock
  with a hand-computed bound (`cookbook_examples_test.cpp` ~:20-24).
- **A12 — options forwarded, one case per step, in D1's shape** (`chassis_recipe_test.cpp`
  ~:782-806). A capped leg must take genuinely longer than its uncapped twin: a `maxAngularSpeed` cap
  on `faceAway`, and a `maxLinearSpeed` cap on `reverseTo`. A dropped cap still settles, so only time
  sees it.

**Part A mutations.** Use a fresh binary for each, with the stale-binary recipe (HANDOFF ~:208-216):
`routine.hpp` is included by 14 test files directly and by three `competition/` headers, so touch every
test that reaches it either way. Log each mutation RED or GREEN:
1. `reverseTo` drops the `+ π`.
2. `faceAway` drops the `+ π`.
3. The bearing is taken from the field origin instead of the live pose. This is killed only off the
   origin (the geometry rule).
4. `reverseTo`'s stop and skip name is `"driveTo"`.
5. `skipIfStopped` is removed from `reverseTo`, so a stopped chain drives.
6. `faceAway` delegates to an in-place `moveTo` instead of `turnTo`.
7. **Probe:** `bearing − π` in place of `+ π` (spelled `bearing - math::Angle::radians(math::Angle::kPi)`;
   `Angle::radians(-kPi)` wraps to +π and would be the SAME code).
   - It is an EQUIVALENT mutant at bearing 0 and at about 59 % of bearings.
   - At the geometry rule's bearing it is NOT equivalent for `faceAway`, or for `reverseTo` on the
     X-drive, where the bearing is read at an exact seeded pose.
   - Record whether A1 or A3 sees the 1-ulp heading difference, and say which.
8. `kApiMinor` is left at 2 (with the string "2.2").
9. `kApiMinor` is 3 while the string is "2.2".
10. A new step is marked `noexcept`. The build must fail NAMING F10; copy the message.
11. A `reverseTo(double, double)` overload is added. The build must fail on the negative pin.
12. In `path_runner.hpp`, DriveTo's `+ π` is dropped. A3's tank case must go RED as well as test 10.
    This proves the new case also guards F8's behaviour on a tank. Restore.
13. `faceAway`'s `runMotion` name is `"face"` (its `skipIfStopped` name stays `"faceAway"`). A6's
    starved `faceAway` must go RED.
14. Either new step forwards only `.timeout` (a fresh `MotionOptions` with the timeout copied). A12,
    and A1 with its caps, must go RED.

**Part B** (at its go). Every expectation of 6a–6r, and of T3 Part 2's 6s–6v, carries over UNCHANGED,
re-pointed at the library API. They are the pin that no decision or line moved (under D9 (i)).
- Capture the `RobotConfig` the old core produces for 6b, 6d, 6l, 6n and 6s BEFORE the move.
- Require equality after it FIELD BY FIELD, reusing `boot_source_test.cpp`'s longhand comparison and
  its member-count pins (~:5-14, ~:236-255).
- **Never memcmp.** The structs hold bools, a uint32 mask and a pointer, so a memcmp compares padding.

New cases:
- a non-tank base is refused by name;
- a base with mechanisms keeps them, with the record's description substituted;
- a base with a motor mechanism on port 6 names it under the wording's label in the port-check line,
  never as NEW;
- the claw/drive refusal fires from a base mechanism for BOTH sources, the record and the table;
- `BootWording`'s defaults print generic words, and robot two's wording prints today's lines byte for
  byte;
- `buildInto`: a refusal leaves the optional empty and names the port; a success emplaces once; a
  second call on a full optional is a precondition.

Part B mutations:
- a record that fails its port check falls back to the table;
- the cross-check runs when the table was chosen;
- `absent()` spans the whole array;
- `buildInto` emplaces before it validates;
- the substitution keeps the base's typed description;
- the declared ports are built over the whole mechanism array (the unused zeros);
- the claw/drive refusal is skipped when the table was chosen;
- a hard-coded label ignores `BootWording`.

## 3. Documentation (same change)

Read `docs/internal/guide-maintenance.md` first. Guide and cookbook listings are compiled and quoted
VERBATIM.

**Part A1** (all of Part A if D8 (b)). Four public documents, five if the VexBuilder sentence is
needed.
- `docs/guide/09-the-recipe-api.md`:
  - two rows in the steps table (~:96-109);
  - the "argument sugar" paragraph (~:111-117) now covers four steps;
  - the **stability notice (~:10-20)** names 2.3's two appended steps as the second real use of the
    additive path;
  - one short paragraph in "Tank drives" (~:194-222): to back into a point on a tank, `faceAway`
    first, then `reverseTo`, with A4's measured outcome of skipping `faceAway`, stated as measured
    on the host test plant.
- `docs/changelog.md`: a NEW `## API 2.3` heading above `## API 2.2`, with one entry:
  - "ADDED: `Routine::faceAway`, `Routine::reverseTo`";
  - "What you must do: nothing — every 2.2 program compiles and behaves the same";
  - nothing has run on a robot.
- `docs/roadmap.md`:
  - The F10 row (~:144): thirteen steps, naming the two added at 2.3.
  - Its pin count is **recounted from the file**. It is 50 today (38 `SHULIB_F10_PIN` + 12
    `SHULIB_F10_NOT_NOEXCEPT`), and `tools/briefing_status.py`'s `pin_counts` is the counter. The
    row's "37 pins" has been stale since F1; say so in the PROGRESS log.
  - The row still says LOCKED 2026-08-12.
  - The F8 row's "`version.hpp` (API **2.2**, carrying the G2 additive note)" (~:142) becomes "API
    2.2 at the lock". It must not claim 2.2 is current.
- `docs/hardware-assumptions.md`: HA-167 (below) under an "L9's addition" heading in Group R3, placed
  as T3's additions are (~:1638). Its index row goes in numeric order: HA-162–166 may land first. Add
  ONE appended status sentence ("L9 added ONE on <date> — HA-167, invented").
- `docs/vexbuilder-integration.md` §4.1.1 (~:245-255): READ it. Add one sentence to the `"drive"` row's
  note ONLY IF A4 shows that a reversed leg on a tank does not settle without a preceding turn. The F8
  vocabulary does not change. (2c-final edits ~:61 of this file, a different section.)
- Header comments that the `docs/` grep cannot see: `path_runner.hpp` ~:24-27 and ~:187-188 (§1 item 6).
- Internal documents:
  - `build-order.md`: L9's own entry, with its status;
  - `HANDOFF-remaining-work.md`: one sentence appended to the L9 reverse bullet (~:449-452). It says
    that a reversed approach (bearing + π) had run only on an X-drive, that tank along-axis reverse
    legs were already tested at C1, and that L9 adds the first reversed approach on a tank;
  - `L9-PROGRESS.md`.
- `docs/api` regenerated (never hand-edited); the briefing regenerated LAST.

**Part A2** (the cookbook recipe; inside Part A if D8 (b)). Two public documents.
- `docs/cookbook/04-drivetrains.md`: a new recipe, **"Back into a point"** (`cookbook-04c`), written
  as the existing recipes are ("Use this when" / listing / "Why" / "Watch out for"). The case list at
  ~:7-8 gains `cookbook-04c`. The routine is generic and is not season content (the guardrail).
- `docs/cookbook/README.md`: a new row in the "4 — Drivetrains" table (~:66-71).
- The test file's banner mapping line (~:6-12): `cookbook-04a/b/c`.
- Guide 09's Tank drives paragraph gains its link to the recipe here.

**Prose review, logged.**
- Grep `docs/` and `README.md` for `driveTo`, `face(`, `reverse`, `backwards`, `kApiMinor`, `2.2`,
  `F10` and `37`, and read every hit.
- Grep `include/` for `routine.hpp is FROZEN` and `exactly \`Routine::driveTo\``.
- Two hits are historical and stay as they are: roadmap ~:466 ("Twelve step verbs") and ~:511 ("37
  compile-time pins").
- Public documents cite evidence by date, never by a `docs/internal` path (C7).

**Part B:**
- `docs/guide/08-your-first-routine.md` (~:274-314; also 2c-final's file, so re-read it after 2c
  lands): a short "from the calibration record" section with a compiled listing. The listing shows
  reading the card, the plan, then `buildInto`, all through the fakes. Its "one program in `src/`"
  sentence is updated.
- `README.md` ~:242-245 (prose review; T3 Part 2 and 2c-final both edit this file).
- `changelog.md`: an entry under whatever heading the H1 D7 ruling implies.
- `build-order.md`, HANDOFF §4 and L9, and COMP's brief B8 wording if it is still open.
- `docs/api` regenerated (a new `boot.md` page), and the briefing LAST.
- No hardware-assumptions entry is expected. The card and the registry already carry HA-122 and
  HA-138..141. If Part B finds a new belief, stop and ask the coordinator for a number.

**HA-167 — a tank drives backwards the way its calibration measured it forwards.**
- *Claim:*
  - shulib Calibrate measures each drive side's travel scale (inches per radian of motor rotation)
    from ONE unpowered 48 in push, with either end first (`drivetrain_calibrator.hpp` ~:26-30,
    ~:53-58: BACK-first inverts every sign).
  - That scale holds within 1 % when the robot drives itself BACKWARDS under power.
  - A change of direction, where each side's gear train takes up its backlash the other way, adds less
    than 0.25 in of travel that the chassis did not make.
- *Source:*
  - `chassis/routine.hpp` `reverseTo` / `faceAway`, and `runner/path_runner.hpp` DriveTo with
    `reverse`. Both now command reverse legs.
  - The host test plant models every wheel as odd-symmetric, with no backlash term
    (`sim/motor_model.hpp` ~:21-26: V = kS·sign(v) + kV·v). So no host test can see either effect.
- *Confidence:* **invented.**
  - The scale was never measured under powered reverse. The push is unpowered. The spin phase does
    drive one side backward under power in each spin direction, but it measures WIDTH
    (W = (dR − dL) / dθ), not scale.
  - The one known anomaly on robot two came during reversing: on 2026-09-25, an implausible-odometry
    fault fired right after ~0.35 s of hard reversing (HA-160). That says reversing is where trouble
    was seen, not that the scale is asymmetric.
- *Settle (the first autonomous session — COMP-b's Station K, or S8):*
  - On the floor, along a tape, run a routine that drives 48 in forward, stops, `faceAway`s the start
    and `reverseTo`s it. Record the estimate and the tape at each end, three times.
  - The forward and backward scale errors must agree within 1 %.
  - Record the raw IMU heading through every reversal (HA-160's settle).
- *Blast radius if wrong:*
  - A reversed leg ends short or long by the asymmetry, and every later leg starts from that error.
    Position only: heading is IMU-owned.
  - A take-up at each direction change adds a phantom offset along the heading. Nothing corrects it
    until a wall reset (L6) or GPS.
  - An IMU glitch during a hard reverse (HA-160) ENDS the motion (ODO_STUCK), rather than bending it.
- **Contained?** Only partly. A measured asymmetry needs a per-direction scale, which is a calibration
  RECORD change (a version bump of the 180-byte record), not a config constant.

## 4. OUT

- Any `src/` change in Part A.
- A facade `driveTo`, or any change to `Chassis` (F6), `MotionOptions`, or `PathRunner` / `Route` (F8)
  beyond §1 item 6's comments.
- A face-away verb in the route data (F8: authored `TurnTo` waypoints carry it).
- Automatic turn planning on a tank: forbidden by C1's D12.
- Per-direction calibration scales.
- A literal one-call boot (D5 (b)), and a `Robot::fromCard()` factory.
- Part B before D6's three merges.
- Any hardware claim: nothing here has run on a robot.

## 5. Checks

- The suite is green, with case and assertion counts before and after logged.
- Every mutation is RED on a FRESH binary. Require `Linking CXX executable shulib_tests` in the output
  (HANDOFF ~:208-216).
- `python3 tools/src_build_gate.py self-test && check`. This proves that `src/` still builds. For
  Part A, the evidence that nothing in `src/` moved is `git diff --stat -- src/`, which must be empty.
- The ARM header gate (`-Wconversion -Wsign-conversion -Werror`).
- Both CI greps. Part B's `robot/boot.hpp` must include no `pros/`, and under D7 (a) no new file
  includes `pros/` at all.
- `api_doc_tool` `self-test`, `check-coverage`, `check-fresh`, `check-examples` and
  `check-removability`.
- `doc_staleness_audit.py`.
- The briefing regenerated LAST and checked.
- The coordinator's command set, HANDOFF ~:218-233.

## 6. Landmines

1. Never add a parameter to an existing F10 step. The member's type changes, the pin fails naming F10,
   and that is a BREAKING (major) change. Grow by ADDING steps, and ADD pins; edit no existing pin.
2. The `+ π` must be spelled exactly as in `path_runner.hpp` ~:241, or A3's memcmp fails, or drifts
   later.
3. `kApiMinor` and `kApiVersionString` move together, and three pin files compare them. H1 edits the
   same file (~:38-41): leave that block alone.
4. The stale-binary trap: `routine.hpp` reaches tests directly (14 files) and through
   `competition/match_runner.hpp`, `acceptance_run.hpp` and `auton_registry.hpp`. Touch them all.
5. A tank test that grades the estimate proves nothing about direction. Grade truth, and use A2's
   heading band.
6. Never grade a reversed leg from the origin or at bearing 0. Two mutants (3 and 7) hide there
   (the geometry rule).
7. `Angle::errorTo` gives +π at an exact antipode (`angle.hpp` ~:11-12). A4's outcome can depend on
   that; say so if it does.
8. Say "host test plant", never "simulator", in everything new. (The cookbook test's existing
   "simulated" comments, ~:2 and ~:20, are not L9's to rewrite.)
9. Shared documents take self-contained blocks only. T3 Part 2 edits the handoff bullet right after
   the one L9 appends to.
10. `routine_signature_pin_test.cpp` ~:385-386 are G1 Part 2c-final's lines. Reword only ~:388-389
    (A9).
11. Use HA-167 only.
12. Part B: never return a Robot by value, never use a `static Robot` (`robot.hpp` ~:50-59), and keep
    the absent ports behind one door (`boot_source.hpp` ~:44-53). A span over the whole array hands
    `validate()` port 0. The same trap applies to the declared ports.
13. Part B: `robot/` may include `calibration/`, but nothing in `calibration/` may include `robot/`,
    and nothing in `hal/` may include either (D7).

## Found while reading (not L9's to fix)

- `version.hpp` ~:71-74 and the changelog's 2.2 entry leave Q6 (the `MotionDeps` ruling) open. The
  2.3 bump must carry it forward unresolved. It is the team lead's decision.
- H1's D7 (whether additive growth of an UNFROZEN surface bumps `kApiMinor`) decides only Part B's
  changelog heading, not Part A's.
- `version.hpp`'s policy lists "new overloads" as additive (~:24-28), but its first bullet calls any
  change that can stop a previously-compiling program from compiling BREAKING (~:14-16). Taking the
  address of a member that has become overloaded is such a program (D1 (b)). This is the team lead's
  to reword, if ever.
