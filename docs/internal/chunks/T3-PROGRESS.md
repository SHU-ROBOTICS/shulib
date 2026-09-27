# T3 — live progress log (both halves, joined at the merge)

> **Coordinator's note (the merge, 2026-09-26).** T3 was built in two isolated git worktrees in
> parallel, each with its own live log: the **library half** (`.claude/worktrees/t3-lib` — run B,
> then B2, the pneumatic claw's library side) and the **program-side prep**
> (`.claude/worktrees/t3-prog-prep` — run E, then E2, the pneumatic claw's table entry). Both logs
> are joined here VERBATIM — the library half first (the foundation), then the program half —
> with every heading one level lower so each part sits under its own heading; the text is
> otherwise unchanged, so "this worktree", "the coordinator at merge" and the line numbers inside
> each part mean what they meant when written. The coordinator's merge record follows both.
> T3's Teleop wiring (its Part 2) is a later run, logged below that.

## Part 1 — the library half (worktree `t3-lib`, runs B and B2)

> **Chunk:** T3 — the claw pivots: button bindings and mechanisms built from the robot's config.
> **This run: PHASE 1, the library half** (brief §2 items 1-3; tests 2-12 and 14 at the library level;
> the documentation for the files touched). Phase 2 (`src/`: the claw table, Teleop wiring, the panel
> row, Calibrate's filter; tests 1 and 13; the worksheet's Station J; build-order / roadmap / handoff /
> the COMP brief) is a separate run.
> **Date:** 2026-09-26. **Tree at start:** `b622d9b`, in the git worktree
> `.claude/worktrees/t3-lib` (branch `t3-lib`), run in parallel with G1 Part 2b (main working tree) and
> DEFECTS2 (another worktree; it owns `hal/pros/motor.hpp`, the PROS shim's brake behaviour, and a
> release-brake-mode field in `DriverLoopConfig`).
> **Executor model:** opus. **Brief:** held outside the repo by the coordinator (it is committed with
> the chunk). Appended in real time. Nothing in this chunk is committed by the executor.

---

### Order of work

1. This log; required reading; the clean-tree suite count at `b622d9b` in a fresh build dir.
2. `include/shulib/teleop/mechanism_binding.hpp` + its tests (brief tests 2-11 at the binding level).
3. The `runDriverControl` bindings overload in `teleop/teleop_loop.hpp` (additive; the old signature
   forwards with an empty span) + loop-level tests (5, 7, 8, 11's re-zero check, 12).
4. `config::MechanismConfig`'s cartridge + safe mode (UNSET representations), `valid()`'s refusals after
   the port-claim loop, the drive-port collision refusal; `robot::Robot` building, naming and exposing
   mechanisms with the absent-mechanism path (test 14).
5. Documentation: changelog, guide 10, a cookbook recipe compiled in `test/cookbook_examples_test.cpp`,
   `vexbuilder-integration.md`'s mechanisms schema, header comments, `docs/api` regenerated,
   hardware-assumptions (HA-147..HA-152 reserved, only if a library-level belief needs one), the
   briefing regenerated.
6. All gates in the worktree; the prose review; the mutation table; the report.

---

### Journal

#### 1. Log created; baseline (fresh build dir `build/test` in the worktree)

**Clean-tree suite at `b622d9b`:** `1425 cases / 2,386,871 assertions, 3 skipped — SUCCESS`. The same
number the committed briefing carries, so the worktree is the tree the brief was written against.

The first `cmake --build` went RED on the briefing gate, not on a test — creating this log adds `T3` to
the briefing's OPEN CHUNK(S) list by existing (G1-PROGRESS's journal records the same trap). Regenerated
with `briefing_status.py generate` (twice: once so the gate would pass and build the binary, once after
the binary existed so the suite line carries the count). `PROJECT-BRIEFING.md` is regenerated again
LAST, and again by the coordinator at merge time.

#### 2. Required reading done; the design, before any code

Read: the brief (in full), `RESUMING.md`, `hal/mechanism.hpp`, `hal/controller.hpp`,
`hal/fake/fake_controller.hpp`, `teleop/teleop_loop.hpp`, `config/robot_config.hpp`,
`robot/robot.hpp`, `hal/device_provider.hpp`, `hal/fake/fake_device_provider.hpp`,
`hal/pros/device_provider.hpp`'s `motor()`, `test/mechanism_test_rig.hpp`, `test/teleop_loop_test.cpp`,
`test/robot_config_test.cpp`, `test/robot_builder_test.cpp`'s rigs, `tools/api_doc_tool.py`'s example
gate, the cookbook README, the changelog's "Added since 2.2", `vexbuilder-integration.md` §3.

**Merge hygiene (the parallel runs).** A read-only `diff` of the shared files against the main working
tree (G1 Part 2b's uncommitted edits) shows 2b rewording `robot.hpp` banner lines ~18-23, ~73-75,
~325-346, ~373, ~690, ~846, ~866, ~920; `robot_config.hpp` ~8-9 and ~325; `teleop_loop.hpp` ~51-53;
`docs/guide/10-the-api.md` ~186-187; `docs/guide/14-what-it-cannot-do-yet.md` ~330-333. Every edit
below is placed with unchanged lines between it and those hunks, so the coordinator's merge should
be mechanical. DEFECTS2 adds a field to `DriverLoopConfig`; this run does not touch that struct.

**Design decisions (each with the alternative rejected):**

1. **`config::SafeBrakeMode : uint8_t { Coast = 1, Brake = 2, Hold = 3 }`, byte 0 = UNSET** — the same
   no-`Unset`-member convention `hal::MotorCartridge` uses. Rejected: storing `hal::BrakeMode` with a
   sentinel (`static_cast<BrakeMode>(-1)`) — `BrakeMode` is an `int` enum whose value-initialised
   state is `Coast` (landmine 7), it is not fixed-width (the config is laid out to be generated and
   serialized), and a sentinel of an enum the HAL owns is a second meaning for a type the config does
   not own. The cartridge reuses `hal::MotorCartridge` (it already has the UNSET-is-0 convention).
   New fields sit after `motorCount` so every existing designated initializer still compiles.
2. **`valid()`'s new refusals run in a SECOND pass after the whole mechanism claim loop** — so every
   collision message a test pins is unchanged even when the same config also has an unset cartridge.
   Plus two refusals the brief did not name, both because `Robot` now borrows the name as a C string:
   a name not NUL-terminated inside its 24 bytes (checked in the first pass, BEFORE any `'%s'` of it —
   the existing messages already print it with `%s`), and two mechanisms with one name (`mechanism(name)`
   would be ambiguous).
3. **`Robot` builds mechanisms in its constructor BODY, after `chassis_`** — the drive graph's
   construction order and its provider requests are untouched (the bit-identity pin), and
   `MotorMechanism` is non-movable so an initializer-list factory returning it by value is not
   possible. Mechanism motors are NOT asked for in `validate()`: a refused mechanism motor makes that
   mechanism ABSENT, never the whole robot (the dead-port rule's spirit); the provider's own sentence
   is copied into the Robot (160 bytes, `ProsDeviceProvider::kRefusalBytes`). A mechanism with no
   motors (pneumatic-only) is reported as not built: the provider has no digital-out seam, and
   pretending otherwise would be a promise.
4. **`MechanismBinding::tick(connected, forwardDown, reverseDown)`** — a pure state machine over two
   levels plus the connected flag (the loop reads the buttons only while connected; the binding also
   ignores a level handed to it while disconnected, so a lying caller cannot drive it).
5. **The outside-stop latch needs an OBSERVED release** (a connected tick with both buttons up). A
   disconnect is a release for the level rules (safe once; a still-held button drives again on
   reconnect — as the brief says), but it does NOT clear an outside stop, and an outside stop that
   lands while the controller is disconnected latches too — because the binding cannot know the
   button is up. Rejected: "disconnect clears the latch", which lets a controller blip undo an
   outside stop with the driver's finger still on the button. **Flagged for the coordinator** as a
   refinement of the brief's "disconnect = release".
6. **A mechanism someone else has `claimed()` is never commanded, `makeSafe()` included**; a press
   seen during someone else's claim latches the binding inert (it never carries over after the claim
   ends). Rejected: resuming a held button the moment the claim is released.
7. **`teleop::validBindings(bindings, rezeroButton, why, cap)`** — PURE, total, `noexcept`: null
   entry, a button used twice across the set, a binding button equal to the re-zero button, one
   mechanism bound twice (two bindings on one mechanism would fight, and bindings take no claim).
   The binding's own constructor refuses forward == reverse, an unknown button byte and a voltage
   outside (0, 12] or not finite. The loop overload runs `validBindings` as a precondition before
   tick 1 (belt and braces); phase 2 calls it inside `teleopInitialize`'s `try`.

#### 3. The library code, written; the existing suite still green

- `include/shulib/teleop/mechanism_binding.hpp` (new): `MechanismBinding` (state machine, the rules
  in its banner), `validBindings()`, `controllerButtonWord()`, `isControllerButton()`.
- `include/shulib/teleop/teleop_loop.hpp`: the existing function gained the
  `std::span<MechanismBinding* const> bindings` parameter in place (body not moved: entry
  reset+makeSafe, makeSafe after `chassis.cancel()` on the disable, step 6b ticks the bindings
  after `drive()`), and a NEW 2a-signature overload after it forwards an empty span. Banner: one
  inserted paragraph after step 8; `DriverLoopConfig` untouched (DEFECTS2 owns it).
- `include/shulib/config/robot_config.hpp`: `SafeBrakeMode`, `safeBrakeModeWord`,
  `isSafeBrakeMode`, `toBrakeMode`; `MechanismConfig::cartridge` / `safeMode`; `valid()`'s name
  terminator check (first pass, before any `%s`) and its second pass (cartridge, safe mode,
  duplicate names) after the whole claim loop; `terminatedName` / `sameName` helpers.
- `include/shulib/robot/robot.hpp`: banner MECHANISMS section, `validate()` doc note, the
  constructor body `buildMechanisms(devices)`, `mechanismCount / mechanismAt / mechanismAbsence /
  mechanism(name) / mechanisms()`, the slot storage after `chassis_`.
- `test/robot_config_test.cpp` 1i: its "legal mechanism set" now states a cartridge and a safe mode
  (a mechanism with motors and neither is no longer legal — that is the point of the chunk); title
  and banner comment reworded ("Part 1 builds none" went stale).

`docs/api` regenerated (`api_doc_tool.py generate`: the new page, the index, the A-Z list, the
mkdocs nav line), briefing regenerated. **Suite: 1425 cases / 2,386,874 assertions, 3 skipped —
SUCCESS** (no new tests yet). The +3 assertions: two are `pros_adapter_fence_test.cpp`'s recursive
scan of every `include/shulib` header (two CHECKs per header; one new header) — read in its source;
the third was not traced to a test, and is noted rather than guessed at.

#### 4. The tests — 24 new cases, all green

- `test/mechanism_binding_test.cpp` (new, 16 cases): B1 hold-to-run every tick at ±V; B2 (brief
  test 2) release = [Brake Hold, 0 V] once then silence over 50 ticks, from both directions, and the
  mechanism's OWN declared mode; B3 (test 3) both held = hold, releasing one drives the other; B4
  the Idle rule and a held button on the first tick; B6 (test 6, binding half) a disconnect is a
  release and TRUE levels handed over while disconnected are ignored; B9 (test 9) the outside-stop
  latch; B9b the disconnect/latch refinement (three subcases); B10 (test 10) a claimed mechanism is
  never commanded, makeSafe() included, and a press never carries over a claim; B11 (test 11) the
  construction refusals and the edges; B11b validBindings (valid set, the DEFAULT re-zero L1 caught,
  a reverse button caught, a button on two mechanisms, a mechanism bound twice, a null). Loop, on
  the host test plant: L4/L5 (tests 4, 5) R1+L2 in one tick and each release in its own tick; L6
  (test 6) the lying disconnected controller (no read while disconnected, resume on reconnect, the
  exact read count); L7 (test 7) the disable — cancel FIRST (a snooping motor sees the drive already
  braked when the pivot's safe state lands), both pivots safe inside the disable tick, no pace;
  L8 (test 8) safe at entry before the first poll, disabled-on-first-poll, re-entry forgets the
  latch; L11 (test 11, loop half) the loop refuses the default-L1 clash and a duplicate button
  before any poll and before touching a mechanism; L12 (test 12) 120 ticks of sticks, buttons and
  drops with and without bindings — drive volts and commands BIT-identical, and the run proves both
  that the drive moved and that the bindings drove.
- `test/robot_config_test.cpp` (4 cases, 1m..1m'''): the claw reference config; UNSET is never
  Coast (and `toBrakeMode`'s floor is Hold); unset/unknown cartridge and safe mode refused by name;
  every real mode x cartridge accepted; pneumatic-only exempt; the new refusals come AFTER every
  port claim (two orderings) and a claw on a drive port is refused; duplicate names (including
  garbage after the NUL, a prefix is a different name, a pneumatic-only name counts); a 24-byte
  name with no NUL refused and 23 characters accepted.
- `test/robot_builder_test.cpp` (4 cases, T3 builder M1..M4, appended at the end — G1 Part 2b
  edits only comments in the middle of this file): M1 validate() never asks for mechanism motors,
  refuses a claw on a drive port / a pod port / an unset cartridge or mode before asking for ANY
  device; M2 two single-motor mechanisms, names borrowed from the Robot's own copy (the caller's
  copy is changed after construction), RED and the stored sign at the provider while the drive
  keeps BLUE, Hold, the guard span, nothing commanded at build, commanding one does not reach the
  other, lookups that must fail; M3 a refused claw is ABSENT with the provider's exact sentence,
  the drive still drives BIT-identically to the hand-wired graph, a two-motor mechanism with its
  second motor refused is absent as a whole, a pneumatic-only one is reported not built; M4 a tank
  WITH claws is bit-identical to the hand-wired graph with none.

**Suite: 1449 cases / 2,389,324 assertions, 3 skipped — SUCCESS** (was 1425 / 2,386,871).

#### 5. The mutation campaign, first attempt — stopped, and what it taught

Harness (scratchpad, not the repo): exact-once text replacement, `touch test/*.cpp`, `/fast`
rebuild REQUIRING the "Linking CXX executable shulib_tests" line, the WHOLE suite (5 s), restore
and hash-verify. The first run was stopped after four mutants for two reasons, both fixed:

1. **MB3 (a binding that trusts levels handed over while disconnected) SURVIVED.** B6 lied with
   BOTH buttons — and a trusting binding reads "both held" and holds anyway, so the test could not
   see the defect. B6 now lies with ONE button at a time (forward, then reverse) and also asserts no
   non-zero voltage after the transition. Re-run: MB3 RED on B6.
2. **The harness's failing-case list was polluted** (doctest prints a `TEST CASE:` header for any
   test that emits messages, not only failures); it now keeps only blocks with `ERROR:`/`FATAL`.
   It also restores the file on SIGTERM now — stopping the first run left MB5 applied to
   `mechanism_binding.hpp`; it was reverted by inverting that exact mutant and verified against the
   pre-campaign SHA-256 of all four headers (all OK) before anything else ran.

Also before the second run: the `std::isfinite` term in the binding's voltage precondition was
REMOVED as logically redundant (NaN fails `> 0`, +inf fails `<= 12`, so no test could ever kill a
mutant of it — B11 pins NaN and +inf refused by the remaining comparisons); B4 now pins `reset()`
alone returning to Idle (mutant MB19 would otherwise be equivalent at loop level); L6 pins that the
loop hands the binding the real connected flag (a disable that lands while disconnected latches);
L7's snoop now also pins "bindings tick AFTER drive()" (the drive's volts are already on the motor
when the pivot's first +12 V lands). The cookbook recipe's compiled case `cookbook-06a` was added
before the second run too, so every mutant runs against the final test set.

#### 6. Documentation (written while the second campaign ran; none of it touches the headers or tests)

- `docs/changelog.md` — "Added since 2.2 (2026-09-26)": the binding, the loop overload, the
  config fields and `SafeBrakeMode`, the Robot's mechanisms; the one behaviour change (valid() now
  refuses a motor mechanism with an unset cartridge or safe mode, an unterminated name, a duplicate
  name) with its migration. No version bump; F7/F11/F13 stay unfrozen.
- `docs/guide/10-the-api.md` — a "Buttons for mechanisms" subsection after the driver-control
  paragraph (inserted after the blank line that follows the sentence G1 Part 2b rewords).
- `docs/guide/14-what-it-cannot-do-yet.md` — "Button bindings and driver macros are still future
  work" went STALE with this chunk; now: mechanism buttons exist (host-only, no driver has used
  one), driver macros are still future work. (Not in the launch prompt's list, but the sentence
  became false — the standing rule is that docs move with the code. Edited two lines below 2b's
  hunk in the same bullet, with two unchanged lines between.)
- `docs/guide/13-extending-the-library.md` — one sentence: a config-built Robot hands you the
  `IMechanism*` list (`Robot::mechanisms()`) and each mechanism by name.
- `docs/cookbook/06-driver-control.md` (new), `docs/cookbook/README.md` (the chapter's table),
  `mkdocs.yml` (the cookbook nav line), `test/cookbook_examples_test.cpp` (case `cookbook-06a`, its
  mapping comment). Every `cpp` line in the chapter is quoted from that case.
- `docs/vexbuilder-integration.md` — §3.1's `mechanisms` schema (motors as {port, reversed},
  `cartridge`, `safeMode`, name rules) and a §3.2 field note on why neither has a default and why
  two claws are two mechanisms.
- `README.md` — the Mechanisms bullet (buttons for driver control; a config-built robot builds its
  mechanisms; host-only) and the `teleop/` line of the tree map.
- `docs/hardware-assumptions.md` — **NOT touched; the reserved HA-147..HA-152 block is unused in
  this phase.** The library half introduces no new hardware belief of its own: "Hold at 0 V holds a
  loaded mechanism" is HA-92 (the mechanism layer's), "a repeated hold re-captures its hold point"
  belongs to DEFECTS2's brake-mode entries, the controller-read beliefs are HA-103/HA-104, and the
  robot-two facts (RED claws on 6 and 7, the loaded-pivot hold, the stall heat, a claw/drive cable
  swap) are the program half's.

**Prose review (grep run and hits read before this entry):** `MechanismConfig`, "no mechanism",
"builds no mechanism", "constructs NO mechanism", "types only", `interruptible`, `binding`, `claw`,
`teleop`, `driver control`, `mechanism` across `docs/*.md`, `docs/guide/`, `docs/cookbook/`,
`README.md`, `include/shulib/`. Stale and fixed: `robot_config.hpp`'s "TYPES ONLY IN PART 1 ...
`robot::Robot` constructs NO mechanism from it" (rewritten), guide 14's "button bindings ... future
work", `robot_config_test.cpp`'s 1i title/banner ("Part 1 builds none"). Hits read and left, with
why: guide 14:204 ("with no mechanism at all" — about `then()` running an operation, still true);
roadmap.md:1609 ("no mechanism hardware has ever moved" — still true) and :1671-1674 (the
"Button bindings / driver macros" item and its stick-interrupt sentence — R1's amendment and the
checkbox are the PLAN documents' and the program half's; `roadmap.md` is outside this run);
`hardware-assumptions.md` "binding" hits (HA-03/11/104 bindings of PROS calls — a different sense
of the word); `vexbuilder-integration.md` §2's RobotBuilder status row (G1 Part 2b rewords it; no
mechanism claim in it).

#### 7. The mutation campaign — 60 mutants, every one RED on a fresh binary

Each mutant: one exact-once text change, `touch test/*.cpp`, `cmake --build build/test --target
shulib_tests/fast` with the "Linking CXX executable shulib_tests" line REQUIRED, the WHOLE suite,
restore, SHA-256 verified. Pre- and post-campaign hashes of the four headers match
(`sha256sum -c` all OK). Killing cases are the first failing ones the harness recorded.

| # | Mutant | Verdict | Killed by |
|---|---|---|---|
| MB1 | safe state re-applied on EVERY non-driving tick | RED (6) | B2, B3, B4, B6, L4/L5 |
| MB2 | both held: forward wins | RED (2) | B3, L6 |
| MB3 | levels trusted while disconnected | RED (1) — **survived the first B6; B6 fixed** | B6 |
| MB4 | a disconnect counts as an observed release | RED (2) | B9b, L6 |
| MB5 | makeSafe() never latches | RED (5) | B9, B9b, L6, L7, L8 |
| MB6 | the inert branch removed | RED (3) | B9, B9b, B10 |
| MB7 | tick() ignores claimed() | RED (1) | B10 |
| MB8 | makeSafe() ignores claimed() | RED (1) | B10 |
| MB9 | a press seen during a claim carries over it | RED (1) — **survived the first B10; B10 extended** | B10 |
| MB10 | reverse commands +V | RED (4) | B1, B3, L4/L5, L12 |
| MB11 | the release stops with 0 V only (no brake mode) | RED (7) | B2, B3, B4, B6, B9 |
| MB12 | constructor accepts forward == reverse | RED (1) | B11 |
| MB13 | constructor drops the 12 V ceiling | RED (1) | B11 |
| MB14 | constructor accepts 0 V | RED (1) | B11 |
| MB15 | constructor refuses exactly 12 V | RED (17) | B11, B1, cookbook-06a, … |
| MB16 | constructor accepts a non-button byte | RED (1) | B11 |
| MB17 | isControllerButton() off by one | RED (1) | B11 |
| MB18 | reset() keeps the last observation | RED (1) | L8 (re-entry after a disable) |
| MB19 | reset() keeps the state | RED (1) | B4 (reset() alone → Idle) |
| MB20b | validBindings(): no re-zero check (`&& false`, compiles) | RED (3) | B11b, L11, cookbook-06a |
| MB21 | validBindings(): no duplicate-button check | RED (2) | B11b, L11 |
| MB22 | validBindings(): no same-mechanism check | RED (1) | B11b |
| MB23 | validBindings(): re-zero checked on the FORWARD button only | RED (1) | B11b |
| MB24 | validBindings(): no null check | RED (1) | B11b |
| ML1 | no makeSafe() at loop entry | RED (1) | L8 |
| ML2 | no reset() at loop entry | RED (1) | L8 |
| ML3 | the disable does not safe the bindings | RED (4) | L4/L5, L6, L7, L8 |
| ML4 | the disable safes the bindings BEFORE chassis.cancel() | RED (1) | L7 (the snooping motor) |
| ML5 | buttons read while disconnected | RED (1) | L6 (read counter) |
| ML6 | the binding always told "connected" | RED (1) | L6 (a disable while disconnected latches) |
| ML7 | bindings ticked BEFORE drive() | RED (1) | L7 (the snoop's drive volts at the pivot's first command) |
| ML8 | perTick BEFORE the bindings | RED (5) | L4/L5, L6, cookbook-06a, … |
| ML9 | the loop skips validBindings() | RED (1) | L11 |
| ML10 | only the first binding ticks | RED (3) | L4/L5, L6, L12 |
| MC1 | no cartridge check | RED (2) | 1m', builder M1 |
| MC2 | no safe-mode check | RED (2) | 1m', builder M1 |
| MC3 | cartridge checked BEFORE the port claims | RED (5) | 1m'', G1 config 1b, builder M1, … |
| MC4 | no name-terminator check | RED (1) | 1m''' |
| MC5 | no duplicate-name check | RED (1) | 1m''' |
| MC6 | sameName() compares past the NUL | RED (1) | 1m''' (garbage after the NUL) |
| MC7 | toBrakeMode()'s floor is Coast | RED (1) | 1m |
| MC8 | a pneumatic-only mechanism must state a cartridge | RED (3) | 1m', builder M3, … |
| MC9 | Hold converts to Brake | RED (2) | 1m, builder M2 |
| MC10 | isSafeBrakeMode() accepts UNSET | RED (3) | 1m, 1m', builder M1 |
| MR1 | mechanism motors asked with the DRIVE's cartridge | RED (1) | builder M2 |
| MR2 | the mechanism port's sign dropped | RED (1) | builder M2 |
| MR3b | an absent mechanism is built anyway (compiles) | RED (2) | builder M3, M5 |
| MR4 | validate() asks for mechanism motors and refuses | RED (3) | builder M1, M2, M3 |
| MR5 | safe mode hard-coded Coast | RED (1) | builder M2 |
| MR6 | name lookup by prefix | RED (1) | builder M2 |
| MR7 | the absence sentence not copied | RED (1) | builder M3 |
| MR8 | the guard span counts declared, not built | RED (1) | builder M3 |
| MR9 | a mechanism named with a literal | RED (1) | builder M2 |
| MR10 | a motorless mechanism not skipped | RED (1) | builder M3 |
| MR11 | no fixed sentence for a reasonless refusal | RED (1) | builder M5 |
| MR12 | mechanismAt() unbounded | RED (1) | builder M5 (index 99 — the mutant reads out of bounds) |
| MR13 | mechanismAbsence() unbounded | RED (2) | builder M2, M5 (same) |

Notes, honestly: MB20 and MR3 as first written were killed only by `-Werror` (an unused parameter,
an unused variable) — a compile error is not a behavioural kill, so both were re-expressed to
compile (MB20b, MR3b) and re-run: RED. MR12/MR13 are killed by an out-of-bounds read the correct
code prevents — the kill is real but it is undefined behaviour in the mutant, not a clean assertion.
Not mutated: `controllerButtonWord()`'s labels beyond those the messages assert ("L1", "R2");
"mechanisms built after the drive graph" (moving construction ahead of the drive changes no
observable of the fake provider — the ordering is a documented discipline, and M4's bit-identity
is the behavioural pin); the claim-free property of the binding (asserted directly in B10:
`claimed()` is false after it drives, and the only claim in the test is the test's own).

**Suite after the campaign's test additions: 1451 cases / 2,389,361 assertions, 3 skipped —
SUCCESS** — 26 new cases over the 1425 baseline: 16 in `mechanism_binding_test.cpp`, 4 in
`robot_config_test.cpp` (1m..1m'''), 5 in `robot_builder_test.cpp` (T3 builder M1..M5) and
`cookbook-06a`. (Counted: `grep -c TEST_CASE` on the new file gives 16.)

#### 8. Final gates (all run in this worktree, after the last edit to any header or test)

- Host suite (fresh build dir, `cmake --build build/test` with every doc gate inside it):
  **1451 cases / 2,389,361 assertions, 3 skipped — SUCCESS** (baseline 1425 / 2,386,871).
- Doc gates, run explicitly too: api_doc_tool self-test OK; coverage "2636 public entities across
  149 headers, all documented"; check-fresh exit 0; examples "421 quoted lines, all verbatim (4
  source files)"; removability "no public doc references docs/internal/"; briefing "generated
  block current"; staleness self-test OK and "clean (27 live docs ...)".
- `tools/src_build_gate.py self-test`: "OK (33 detector cases, each against a real build)";
  `check`: "PASS (all 6 builds link ...)", every build 0 disallowed warnings, beacons checked.
- ARM header gate (arm-none-eabi-g++, -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow
  -Werror, cortex-a9 softfp, 185 headers): "CLEAN (all v2 headers)".
- CI greps: "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds: core
  is sim-free".
- The briefing was regenerated LAST from the green binary. (An earlier regeneration had read the
  last MUTANT's binary and recorded "RED — 2 cases failing"; caught by `briefing_status.py check`
  after the rebuild, and regenerated. Worth knowing for anyone who runs a campaign and then
  generates the briefing: rebuild clean first.)

#### 9. What this phase did NOT do (for the coordinator and phase 2)

- Anything under `src/` (the claw table, Teleop wiring, the panel row and fault-row merge,
  Calibrate's candidate filter, the boot-check wording), tests 1 and 13, the worksheet's Station J,
  `build-order.md` (the T3 entry, R1's deviation row), `roadmap.md` (Phase T's "Button bindings /
  driver macros" item still reads `[ ]` with the stick-interrupt sentence R1 amends),
  `HANDOFF-remaining-work.md` (L5, L9, Q1, §7), the COMP brief, `hal/pros/motor.hpp`, the PROS
  shim — all outside this run by the launch prompt.
- `docs/hardware-assumptions.md`: HA-147..HA-152 left unused (§6 says why).
- **One flagged refinement of the brief** (design decision 5): a disconnect is a release for the
  level rules but NOT an observed release for the outside-stop latch. Pinned by B9b; mutant MB4.
- **Merge notes:** `teleop_loop.hpp`'s change is additive (a parameter on the existing function,
  four inserted blocks, a new forwarding overload after it; `DriverLoopConfig` untouched).
  `docs/guide/10-the-api.md`'s new subsection is inserted after the blank line that follows the
  sentence G1 Part 2b rewords — a diff tool may still anchor it one line higher, so a trivial
  conflict there is possible. `README.md`, `docs/guide/14`, `robot.hpp`, `robot_config.hpp` and
  `robot_builder_test.cpp` are also edited by 2b, in hunks separated from these by unchanged lines.
  `docs/api/*`, `mkdocs.yml`'s API nav and `PROJECT-BRIEFING.md` are generated — regenerate after
  the merge rather than merging them by hand.
- Nothing was committed, pushed, or run on a robot; no `pros` command; no serial device touched.

---

#### 10. Review findings and their disposition (review round 1, 2026-09-26)

A reviewer reported nine findings on this phase (2 major, 7 minor). Each is re-checked below
against the tree and, where it is real and inside phase 1, fixed at the root; the rest say why
not, with evidence. Appended as the work goes; the gates are re-run after the last edit (§10.10).

**Before any fix, the three reported surviving mutants were RUN on a fresh binary** (same harness
as §7: exact-once replacement, `touch test/*.cpp`, `/fast` rebuild with the "Linking CXX executable
shulib_tests" line required, whole suite, restore, SHA-256 verified): MR14 (Robot builds Hold
whatever the config says), MR15 (a mechanism built over its first motor only) and MB25 (the claim
branch sets Idle, not Stopped, when nothing is held) — **all three SURVIVED (green)**. So findings
3, 4 and 5 were real before anything was changed.

##### 10.1 Finding 1 (major) — merge conflicts with DEFECTS2, and two steps named "6b"

**Re-checked: real.** A read-only `git diff b622d9b` in `defects2-lib` shows it inserting at two of
the same anchors in `runDriverControl`: after `hal::ButtonEdge rezeroEdge{}` (its entry
`applyReleaseMode`) and next to `chassis.drive(...)`; editing the `cfg` bullet next to T3's
`bindings`/`disabled` bullets; and adding a section at the top of the changelog's `## API 2.2`,
where T3 adds one too. DEFECTS2's snapshot MOVED while I watched: at my first read its
`applyReleaseMode()` sat BEFORE `drive()`; at the trial merge it sat AFTER it again, as
"`// 6b. The release mode again`" (the reviewer's version). Its banner names its step "6b. release
mode" in both. T3's code comment also said "6b", so a mechanical merge would have carried two
different 6b steps, one of them silently (no textual conflict in the variant where DEFECTS2's code
precedes `drive()`).

**Fixed at the root, inside phase 1:**
- **No step label T3 can collide on.** T3's three loop additions are now placed by POSITION, not
  numbered: code comments "Bindings, before tick 1", "…and every mechanism, in the same tick",
  "Bindings, after drive() and before the panel"; the banner paragraph says "after step 6's
  drive(), before step 7's perTick … placed by position, not numbered: they touch only the
  mechanisms, never the drive". Rejected: the reviewer's "6c" — it presupposes DEFECTS2's 6b and
  where it sits, which changed twice in one hour; position words stay true in T3 alone and after a
  merge in either order. `grep -rn "6b"` over `include/shulib/teleop/` and T3's docs: none.
- **The entry block no longer shares DEFECTS2's anchor.** It moved from after `rezeroEdge` to
  directly after T3's own `validBindings` precondition, before `budgets` — three unchanged lines
  away from DEFECTS2's insertion. Behaviour unchanged: entry safe still lands before the first
  `disabled()` poll (L8), and the precondition still refuses before any mechanism is touched (L11).
  Re-run as mutants on the moved block: ML1b (no makeSafe at entry) RED on L8 and L10; ML2b (no
  reset at entry) RED on L8 and L10.
- **The merge notes now carry the DEFECTS2 overlap and its resolution** (§10.9 supersedes §9's
  merge notes, which said nothing about DEFECTS2 beyond `DriverLoopConfig`).

**Left as conflicts, because they are the same logical edit in two chunks, and resolved in a
trial:** (b) both chunks add a step between `drive()` and `perTick` — resolution: DEFECTS2's
release mode FIRST, then T3's bindings (the order is immaterial to behaviour — drive motors vs
mechanism motors — and the trial proves it); (c) the doc bullets — DEFECTS2's `cfg` bullet, then
T3's `bindings` and `disabled` bullets (moving T3's bullet out of parameter order to dodge an
adjacency conflict would trade a one-time merge step for a permanently misordered doc — rejected);
(d) the changelog — T3's section ABOVE DEFECTS2's (newest first; T3 lands after). Plus one edit
with no textual conflict: DEFECTS2's function-doc order line gains "the mechanism bindings"
between "the release mode" and "`perTick`".

**The trial merge (evidence that the resolutions are right, not only plausible).** In the
scratchpad (`t3r/tree`, never in any worktree): `git archive 9325490` (shulib-v2 HEAD — G1 Part
2b committed), then every DEFECTS2 source file merged 3-way against `b622d9b` from its working
tree (READ-ONLY), then every T3 file merged 3-way the same way; generated pages regenerated.
- 2b × DEFECTS2: 2 conflicts, neither T3's — `teleop_loop.hpp`'s `DriverLoopConfig` paragraph (2b
  and DEFECTS2 both rewrite it) and `hardware-assumptions.md`'s HA-141 row next to DEFECTS2's
  HA-142..146. Resolved by hand for the trial only.
- T3 onto that: **3 conflicts, exactly (b), (c) and (d) above.** Every other T3 file —
  `robot.hpp`, `robot_config.hpp`, guide 10, guide 14, `vexbuilder-integration.md`, `README.md`,
  `robot_builder_test.cpp`, `mkdocs.yml` — merged CLEAN against 2b's committed hunks (the
  hunk-separation claim in §9 was checked, not assumed).
- Resolved as above; fresh host build: **doc gates passed; 1492 cases, 1490 passed, 2 failed**
  — both are "no git identity" cases (`session_header_test.cpp`: `SHULIB_BUILD_HASH is not
  defined`; `run_report_e2e_test.cpp`: the `[SES] run start · build` line), because the trial tree
  is a `git archive`, not a checkout. **L7, L10, L12, cookbook-06a and T3 builder M1..M6 all pass
  on the merged tree** (`-tc` run: 10 cases, 2315 assertions, 0 failed) — the reviewer's "re-run
  L7 and L12 after the merge", done ahead of the merge. ARM header gate on the trial tree: CLEAN
  (185 headers).
- Caveat, honestly: DEFECTS2's tree is a moving snapshot; the coordinator's real merge is the one
  that counts. The resolved trial files are at `scratchpad/t3r/tree/include/shulib/teleop/
  teleop_loop.hpp` and `…/docs/changelog.md` for reference.

##### 10.2 Finding 2 (major) — a claim stranded by a deleted autonomous task

**Re-checked: real, and latent.** `hal/mechanism.hpp`: the claim is a plain flag released only by
`releaseClaim()`; the binding's claim branch (`tick()`) and `makeSafe()` both skip a claimed
mechanism; the loop's entry does only `reset()` + `makeSafe()`. An operation autonomous was
running when the field ended it keeps its claim into driver control; the bound mechanism then
gets no command at all — not the entry safe state, not a button, not the disable's safe state —
and the binding's state reads Stopped/Inert. Latent: the brief keeps the claws driver-only (§0),
so no autonomous operation touches a bound mechanism in phase 1 or 2; COMP is where it becomes
reachable.

**Not fixed in behaviour, and why.** Every fix changes the brief's rule "never commands a
mechanism someone else has `claimed()`" (§2 item 1), which is the team lead's, and the candidate
fixes differ in WHAT they break — so it is a ruling, and standing rule 2 says propose and wait.
The options, with evidence:
- **(a) the loop force-releases a claim on every BOUND mechanism at entry, before `makeSafe()`**
  (never calling the claimant's `cancel()` — it may have lived on the deleted task's stack).
  Covers every stranded claim, including an early field disable, and no program can forget it.
  Costs: (i) a program that starts an operation on a bound mechanism BEFORE entering the loop
  and ticks it from `perTick` loses its claim — the operation keeps commanding and a press fights
  it (workaround: start such an operation from `perTick`, after entry); (ii) the stale operation
  object keeps `holdsClaim_ == true`, so a later `cancel()` of that same object would release a
  claim someone else then holds (`mechanism_op.hpp` ~:392-396) — the same exposure RunGuard's
  anonymous force-release already accepts (`run_guard.hpp` ~:560-569); (iii) the loop has no
  telemetry sink, so "log a Warn" needs a new loop parameter or a flag on the binding for the
  panel.
- **(b) COMP releases every autonomous claim before the field's disable** (a RunGuard hard stop
  earlier than the field's): cannot cover a disable that comes early (a match stopped by the
  field) — the stranded claim then happens anyway.
- **(e) the program releases the claims on its bound mechanisms at the top of `opcontrol()`**,
  before `runDriverControl`: covers an early disable; can be forgotten, so COMP's brief must carry
  it with a test.
- **Recommendation: (a)** with a flag on the binding the panel can show, because it is the only
  option a program cannot forget and the one pattern it breaks has a stated alternative. If the
  team lead prefers (e), COMP's brief carries it. Either way the case below is the one that flips.

**Done inside phase 1 (true whichever way the ruling goes):**
- **L10 (new): a mechanism claimed BEFORE `runDriverControl` is entered** — zero events on that
  pivot over 8 held ticks (no entry safe state, no +12 V, no disable safe state), the claim still
  there when the loop returns, the other pivot safe at entry and driving every tick, the drive
  unaffected, the binding Inert, `mechanism().claimed()` true (what a panel reads); after the
  claim is released, the next call makes it safe at entry and a held button drives at once.
  Mutants: **ML11 (the loop releases a claim at entry — the gap closed silently) RED on L10;
  MB8b (`makeSafe()` paints a claimed mechanism) RED on B10 and L10.**
- **The header states the gap and today's obligation** (`mechanism_binding.hpp`, the NO CLAIM
  rule: "THE GAP THIS LEAVES … Until the library owns that case, a program whose autonomous
  routine commands a bound mechanism releases any claim still on it (`releaseClaim()` — never the
  claimant's cancel() …) before it starts driver control, and its panel shows the claim"). The
  changelog entry, guide 10's "Buttons for mechanisms" and the cookbook (a new watch-out, "An
  operation's claim wins — even a dead one") say the same.

##### 10.3 Finding 3 (minor) — surviving mutant: Robot ignoring the declared safe mode

**Re-checked: real** (MR14 SURVIVED, above). **Fixed:** builder **M6** (new, appended after M5)
builds three mechanisms side by side — "intake" (two motors, 16 and -17, GREEN, **Coast**), "arm"
(18, RED, **Brake**), "claw" (-19, RED, **Hold**) — and asserts each one's `safeBrakeMode()` and,
at RecordingMotors, that `applySafeState()` lands [Brake(its mode), 0 V] on every motor.
**MR14: RED (M6).**

##### 10.4 Finding 4 (minor) — surviving mutant: a multi-motor mechanism built over one motor

**Re-checked: real** (MR15 SURVIVED, above; the only two-motor case, M3's lift, is refused).
**Fixed in M6:** the intake's `motors().size() == 2`, in config order (`&m16`, `&m17`), both signed
ports (16, -17) and both GREEN cartridges at the provider, one `setVoltage(6 V)` reaching both
motors and no other. **MR15: RED (M6).**

##### 10.5 Finding 5 (minor) — surviving mutant: the claim branch returning Idle

**Re-checked: real** (MB25 SURVIVED, above). **Fixed:** B10's third part now asserts `Stopped`
after the claimed tick with nothing held, then five idle ticks after `releaseClaim()` with ZERO
events and `Stopped` still, then the press drives. **MB25: RED (B10).**

##### 10.6 Finding 6 (minor) — the cookbook printed an empty refusal branch

**Re-checked: real** — the chapter's listing had `if (!validBindings(...)) { // comment }` and the
test's `FAIL(why);` was the unquoted line. **Fixed:** the test declares a `showOnScreen` stand-in
(a `FAIL_CHECK`, so a refused set can never pass the case vacuously), and the branch — quoted
WHOLE — is `showOnScreen(why); return;` ("say what is wrong while the program still can… and never
start the loop, which would refuse it by throwing"). The chapter says the branch must stop the
program before `runDriverControl`, and lists `showOnScreen` among the names the test declares.
**MB26 (validBindings refuses every non-empty set): RED on cookbook-06a** (and 7 others) — because
the `return` skips every later CHECK, cookbook-06a going red proves the `FAIL_CHECK` inside the
branch fired. `check-examples`: 422 quoted lines, all verbatim.

##### 10.7 Finding 7 (minor) — the header promised a park guard could make a stop stick

**Re-checked: real** — `RunGuard::safeMechanisms()` walks `IMechanism*` only (cancel a registered
claimant, force-release an anonymous claim, then `applySafeState()`, `run_guard.hpp` ~:555-570);
a binding takes no claim, so the guard cannot reach it, and a held button re-drives next tick.
**Fixed:** the header's OUTSIDE STOP rule now says makeSafe() is for code that HOLDS the binding;
"Only a stop that goes THROUGH the binding sticks"; a guard holding `IMechanism*` repaints and a
held button drives again next tick; a program that runs driver control under such a guard makes
the loop's `disabled()` true once the guard expires (`guard.expired()`), so the loop's own disable
path stops every binding. (The reviewer's "make makeSafe() part of the end action" was checked
and rejected: `runDriverControl` returns only when `disabled()` says so, and the guard cannot
pre-empt caller code — its own banner — so the end action would never run while the loop does.)
Changelog, guide 10 and the cookbook reworded to "through the binding". **For the COMP brief
(outside this run):** never run `runDriverControl` under a RunGuard unless its `disabled()`
includes the guard's expiry.

##### 10.8 Findings 8 and 9 (minor) — text that goes false at merge; the HA numbering

- **8(1) — 2b's `DriverLoopConfig` sentence** ("Making this default refuse too is owed before any
  program binds a mechanism to a shoulder button") is committed at `7f9b9a1`, not in this tree
  (`b622d9b`), and both 2b and DEFECTS2 rewrite that very paragraph (the trial's 2b × DEFECTS2
  conflict). Editing it here would add a third conflicting rewrite, so it is a merge step — the
  replacement text, tested in the trial build, is in §10.9.
- **8(2) — `ORIENTATION.md`** — **fixed here**: "Not built yet" no longer lists "button bindings
  for mechanisms"; it lists "the claws' buttons in a robot program (the library's own mechanism
  buttons, and a robot that builds its mechanisms from its config, exist since 2026-09-26 — host
  test plant only)". Checked first that no other run touches the file: the main tree's copy is
  byte-identical to `b622d9b`'s (`cmp`), and `defects2-lib` / `t3-prog-prep` do not modify it.
  Phase 2 moves it again. Seen, not T3's: its "shulib Teleop … never run" (:95-96) is already
  stale since `4611b22`.
- **8(3) — `HANDOFF-remaining-work.md`** — out of this run by instruction. Two lines, not one:
  :437 (L9, "`Robot` constructs no mechanism from `MechanismConfig` today") AND :258
  ("`MechanismConfig` exists as data; **nothing…"). The prose review now covers `docs/internal/`:
  `grep -rn -i "button binding\|bindings for mechanisms\|builds no mechanism\|constructs NO
  mechanism\|no mechanism from\|MechanismConfig" docs/internal` — hits read; the stale ones are
  ORIENTATION:100 (fixed), HANDOFF :258/:437 (merge), build-order :1432 and the COMP brief :180
  (out of this run); the rest are dated PROGRESS/brief records, left as history.
- **9 — HA-147..HA-152 unused while `t3-prog-prep` numbers its claw beliefs HA-153..156.**
  Re-checked: real (read-only diff of `t3-prog-prep`'s register). Not fixable here (another
  worktree). Recommendation: renumber `t3-prog-prep`'s four down to **HA-147..HA-150** — T3's own
  reserved block, and it is T3's program half — so the register stays contiguous, then
  regenerate the briefing. This phase still registers no belief (§6).

##### 10.9 Merge notes, corrected (supersede §9's "Merge notes" bullet)

- **Order:** 2b (committed) → DEFECTS2 → T3 phase 1 → phase 2.
- **`teleop_loop.hpp` × DEFECTS2 — two conflicts:** the doc bullets (take DEFECTS2's `cfg`
  bullet, then T3's `bindings` and `disabled` bullets); after `chassis.drive(...)` (DEFECTS2's
  release mode first, then T3's bindings block). Then, no conflict marks it: DEFECTS2's order line
  ("…`drive(Body)`, the release mode, `perTick`, `pace`") gains "the mechanism bindings" before
  `perTick`. T3's entry block (after the `validBindings` precondition) and DEFECTS2's (after
  `rezeroEdge`) both stay.
- **`teleop_loop.hpp` × 2b × T3 — the `DriverLoopConfig` paragraph** (2b and DEFECTS2 conflict
  there; T3 makes 2b's last sentence false). Tested replacement: "State the first TWO, with
  designated initializers — `{.tickPeriod = …, .rezeroButton = …}`. The period's default is
  deliberately not a working configuration: a zero period trips `runDriverControl`'s precondition
  before anything moves, so a config nobody wrote can never run. The button's default, `L1`, is a
  shoulder button, which robot two's claw bindings take: `validBindings()` and
  `runDriverControl`'s precondition refuse a binding on the re-zero button by name, before the
  first tick, but a program with NO binding on L1 still gets it quietly — so name the button
  ("shulib Teleop" names `A`). The third, `releaseBrakeMode`, has a working default (Coast, the
  team lead's ruling)."
- **`docs/changelog.md` × DEFECTS2 — one conflict:** T3's "Added since 2.2 (2026-09-26)" section
  first, then DEFECTS2's "Fixed since 2.2 (2026-09-26)", one blank line between.
- **Clean against 2b (measured in the trial, §10.1):** `robot.hpp`, `robot_config.hpp`, guide 10,
  guide 14, `vexbuilder-integration.md`, `README.md`, `robot_builder_test.cpp`, `mkdocs.yml`.
- **`T3-PROGRESS.md` is an add/add with `t3-prog-prep`** (each worktree created one; that log says
  the coordinator concatenates them).
- **Regenerate, never hand-merge:** `docs/api/*`, `mkdocs.yml`'s API nav, `PROJECT-BRIEFING.md`.
- **Carried for the coordinator / phase 2:** HANDOFF :258 and :437 (8(3)); the HA renumbering (9);
  the COMP brief's RunGuard note (10.7) and the stranded-claim ruling (10.2); ORIENTATION's
  "the claws' buttons in a robot program" moves again in phase 2.

##### 10.10 This round's mutation table (fresh binary each: `touch test/*.cpp`, `/fast`, the Linking line required, whole suite, restore + SHA-256 verified — all headers `OK` after)

| # | Mutant | Before the new tests | After | Killed by |
|---|---|---|---|---|
| MR14 | Robot builds Hold whatever the config says | **SURVIVED** | RED (1) | builder M6 |
| MR15 | a mechanism built over its FIRST motor only | **SURVIVED** | RED (1) | builder M6 |
| MB25 | claim branch, nothing held: Idle, not Stopped | **SURVIVED** | RED (1) | B10 |
| ML1b | no makeSafe() at loop entry (the MOVED block) | — | RED (2) | L8, L10 |
| ML2b | no reset() at loop entry (the moved block) | — | RED (2) | L8, L10 |
| ML11 | the loop releases a claim at entry (the gap closed silently) | — | RED (1) | L10 |
| MB8b | makeSafe() paints a claimed mechanism | — | RED (2) | B10, L10 |
| MB26 | validBindings() refuses every non-empty set | — | RED (8) | cookbook-06a (the refusal branch fired), B11b, L4/L5, L6, L7, … |

§7's 60 mutants were not re-run: this round's code change is the entry block's move (re-run
above as ML1b/ML2b) and comments; every test change ADDS assertions, and the cookbook's
`FAIL(why)` became `FAIL_CHECK` + `return`, which fails the case on exactly the same inputs.

**A trap met again, recorded so the next session does not lose an hour to it.** After the
campaign, a full `cmake --build` went RED on the BRIEFING gate (the suite count had changed), and
the gate runs BEFORE the test binary relinks — so the binary left in `build/test` was the LAST
MUTANT's (MB26), and L10 "threw" a precondition with an empty message (MB26's `validBindings`
refuses without writing `why`). Fixed by a fresh `/fast` relink (Linking line seen), then
`briefing_status.py generate`, then the full build (exit 0). §8 noted the same trap from the
other side; the rule is: after a mutation campaign, relink before believing any result.

##### 10.11 Final gates, round 1 (all in this worktree, after the last edit to any header, test or public doc)

- Host suite (`cmake --build build/test` exit 0 with every doc gate inside it; binary freshly
  relinked): **1453 cases / 2,389,412 assertions, 3 skipped — SUCCESS** (round 0: 1451 /
  2,389,361; baseline `b622d9b`: 1425 / 2,386,871). +2 cases: L10 and builder M6.
- Doc gates, explicitly: api_doc_tool self-test OK; coverage "2636 public entities across 149
  headers, all documented"; check-fresh exit 0; examples "422 quoted lines, all verbatim (4 source
  files)"; removability "no public doc references docs/internal/"; briefing "generated block
  current"; staleness self-test OK (10 cases) and "clean (27 live docs …)". `docs/api` regenerated
  after the last header edit; the briefing regenerated after the last relink.
- `tools/src_build_gate.py self-test`: "OK (33 detector cases, each against a real build)";
  `check`: "PASS (all 6 builds link …)", every build 0 disallowed warnings, beacons checked.
- ARM header gate (arm-none-eabi-g++, -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow
  -Werror, cortex-a9 softfp): CLEAN, 185 headers.
- CI greps: "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds:
  core is sim-free". New runtime strings: ASCII only (checked by grep over the diff).
- Nothing committed or pushed; no `pros`; no serial device; nothing under `src/` touched;
  `defects2-lib`, `t3-prog-prep` and the main tree only READ.

---

### Addendum B2: the pneumatic claw

> **Brief:** `docs/internal/chunks/T3-addendum-pneumatic-claw.md` §2 (committed in the main repo at
> `9325490`; read from there, never edited). **This run: B2 only — the library half** (the toggle
> binding, the loop carrying it, the provider's digital out, pneumatic mechanisms from the config,
> their tests and mutations, the docs this worktree owns, HA-157 and HA-158). E2 (the program half,
> `src/`, Station J, HA-159) is another run in another worktree. **Executor model:** opus.
> **Date:** 2026-09-26. **Tree:** the T3 library half above, uncommitted, at base `b622d9b`.
> Nothing is committed; no `pros`; no serial device; only this worktree is edited (the main repo's
> addendum and T3 brief were READ).

#### B2.1 Log first; required reading; the baseline

Read in full: the addendum, the T3 brief, this log (§1-§10.11), `teleop/mechanism_binding.hpp`,
`teleop/teleop_loop.hpp`, `hal/mechanism.hpp` (PneumaticMechanism ~:287-338), `hal/controller.hpp`
(ButtonEdge), `hal/digital_out.hpp`, `hal/pros/digital_out.hpp`, `hal/fake/fake_digital_out.hpp`,
`hal/device_provider.hpp`, `hal/fake/fake_device_provider.hpp`, `hal/pros/device_provider.hpp`,
`config/robot_config.hpp` (MechanismConfig, `valid()`), `robot/robot.hpp` (the MECHANISMS section
and `buildMechanisms`), `test/pros_shim/pros/adi.hpp`, the vendored `pros/adi.hpp` DigitalOut
(~:530-615), `test/mechanism_binding_test.cpp`, `test/pros_device_provider_test.cpp`,
`test/robot_config_test.cpp` 1i/1m*, `test/robot_builder_test.cpp` T3 M1-M6, the cookbook
chapter 6 and its compiled case, the T3 edits to the changelog, guide 10/13/14,
`vexbuilder-integration.md`, `README.md`, `ORIENTATION.md`, and the register's HA-119..HA-141
rows and format.

**Baseline (this worktree, `build/test`, before any B2 edit): 1453 cases / 2,389,412 assertions,
3 skipped — SUCCESS** (the §10.11 number).

#### B2.2 The design, before any code (each with the alternative rejected)

1. **`teleop::ToggleBinding` lives in `mechanism_binding.hpp`, beside the hold binding** — so the
   ONE set check (`validBindings`) can see both kinds, and the hold-only `validBindings` becomes a
   forward to it (one owner for "one button, one job"). Rejected: a sibling header — the set check
   would need both types, so either the hold-only check would be a second implementation of the
   same rules (two owners drift) or the headers would include each other.
2. **Two spans, not a common interface** (the addendum leaves it to the executor): the loop takes
   `std::span<MechanismBinding* const>` AND `std::span<ToggleBinding* const>`. The two kinds differ
   exactly where the loop must be explicit — on `disabled()` the hold bindings are made safe and the
   toggles are NOT — and two loops over two spans make that difference readable at the one place it
   happens. Rejected: an `IDriverBinding` with virtual enter/tick/onDisable — a vtable to hide the
   one difference that matters (mechanism.hpp's anti-abstraction rule), and a `span<Base*>` cannot
   be made from T3's `MechanismBinding*` arrays, so every T3 caller and test would have to change.
   The body is the NEW overload (the T3 function gains the toggle span); T3's 9-argument signature
   forwards with an empty toggle span; the 2a signature forwards as before. One body.
3. **A press is a SEEN press.** The toggle flips on a rising edge of its OWN `hal::ButtonEdge`, and a
   level the binding could not see — at loop entry, and on every disconnected tick — counts as HELD
   (the edge is re-seeded "down"). So a button already down at entry, or down when the controller
   comes back, is not a press: it must be seen up, then down, both on connected ticks. That is how
   "an edge that happened while disconnected is never replayed" is made structural, and it is the
   opposite of the hold binding's "a button held at the first tick drives at once" on purpose: a
   hold stops when released, a toggle's flip is permanent — a phantom flip opens the claw on what
   it holds. Cost, stated: a press landing on the very tick the controller reconnects (or the first
   tick of driver control) is not counted; the driver presses again.
4. **A flip flips what the line was last COMMANDED** — `set(!mechanism.commanded())`, the
   PneumaticMechanism's own readback of its first line — never a private copy of the state. So if
   anything else set the claw (an operation, an end-of-run guard's `applySafeState()`), the next
   press still does the visible opposite. Rejected: a `bool open_` in the binding (it goes stale the
   moment anyone else commands the mechanism).
5. **The initial state is commanded ONCE IN THE BINDING'S LIFE — at its first loop entry — not on
   every entry.** PROS calls `opcontrol()` again after a field disable; re-commanding the initial
   state there would move the claw on the re-enable, which is exactly what "a disable commands
   nothing, the claw keeps its state" exists to prevent. **Flagged for the coordinator** as my
   reading of the addendum's "commanded once at loop entry". Consequence for COMP (flagged): if
   autonomous commands the claw, the FIRST driver-control entry still re-commands the stated
   initial state once. A claim on the mechanism at that first entry means the initial state is not
   commanded (never commands a claimed mechanism), and it is not deferred to later either — a start
   state landing mid-driver-control, whenever a claim happens to end, would be a movement nobody
   pressed for.
6. **No claim; a claimed mechanism is never commanded** — the edge keeps following the button
   during a claim, so a press seen then is DROPPED and never carried over (the button must be seen
   up again after the claim). No `makeSafe()` exists on a toggle at all: nothing in the toggle's
   life ever commands the safe value (that is the mechanism's `applySafeState()`, for an end-of-run
   guard), and the loop's disable path does not touch it.
7. **`IDeviceProvider::digitalOut(line, initialState)` is PURE VIRTUAL.** Additive for every caller;
   an implementer must add it (the seam is unfrozen; the tree's implementers are the two providers
   and one test double, `SilentRefusal` in builder M5). Rejected: a default body returning nullptr —
   it cannot write the implementer's `lastRefusal()`, so the Robot would report the PREVIOUS
   refusal's sentence as the reason a line was refused: a lie with a port number in it.
   **The initial state is a PARAMETER** because making the line drives it (HA-119: `ProsDigitalOut`
   has no default, deliberately) — so the provider cannot make a line without being told the level.
   Same line again ⇒ the same object, NOT driven again; the same line with a different initial
   state ⇒ precondition (one line, one configuration — the motor's rule). Lines are 1..8 only (the
   brain's own A..H; a letter byte is refused, not translated — one spelling at the seam; an
   expander is not on the seam). **No registry check, and none is possible** (HA-158): the only
   refusal the on-robot provider can make is the range. No `try` around the construction: the
   vendored `pros::adi::DigitalOut` constructor reports a bad port through errno (ENXIO), never an
   exception, and the range is checked first — a catch nothing can reach would be dead code.
8. **`config::PneumaticLevel : uint8_t { Low = 1, High = 2 }`, byte 0 = UNSET**, for both
   `pneumaticSafe` and `pneumaticStart` (new fields after `pneumaticCount`, so every existing
   designated initializer compiles). Rejected: `bool` fields — a value-initialised bool is `false`,
   an implicit "off" whose meaning (claw open or closed) only the plumbing knows.
9. **A mechanism with motors AND 3-wire lines is REFUSED by `valid()`**, by name ("make them two
   mechanisms"). One name is one object and one lookup: a `MotorMechanism` cannot fan a level and a
   `PneumaticMechanism` cannot take a voltage, and a binding needs one concrete type. This changes
   one T3/G1 test fixture (1i's "legal mechanism set" had a motor intake with a line on it). Also
   new in the claim pass: **a 3-wire line claimed twice** (by two mechanisms, or twice by one) is
   refused, naming the port letter — two mechanisms fanning to one line is a wiring contradiction,
   and the provider would hand both the same object.
10. **`Robot` builds a `PneumaticMechanism` for every mechanism with lines and no motors**, asking
    the provider for each line with the mechanism's START level (the program-start state; on a
    brain that DRIVES the line — the one device write the builder now causes, stated in its
    banner), declaring its SAFE level, naming it from the Robot's own config copy; a refused line ⇒
    that mechanism ABSENT with the provider's sentence, the drive untouched. `validate()` does not
    ask for lines (asking would DRIVE them, and the only refusal possible is the range, which
    `valid()` already makes). **Lookup:** `mechanism(name)` / `mechanismAt(i)` stay the MOTOR
    lookups (T3's signatures, unchanged — the program half is written against them), and
    `pneumaticMechanism(name)` / `pneumaticMechanismAt(i)` are the pneumatic ones: same key, same
    name rules, typed because a binding needs the concrete type and the library does not downcast.
    `mechanisms()` (the RunGuard span) carries both kinds in config order. **Deviation, stated:**
    the addendum says "the same `mechanism(name)` lookup"; a single untyped lookup would return
    `hal::IMechanism*`, which no binding can use.
11. **Two lines in one mechanism are two valves fired TOGETHER at the same level** —
    `PneumaticMechanism` fans one level to every line. That is NOT a double-coil solenoid (whose two
    coils need OPPOSITE levels); the library does not build one of those, and the docs say so. (For
    E2: "a double solenoid would use the second pneumatic port" is true only for two single valves
    moved together.)

#### B2.3 The library code, written; the existing suite still green

- `teleop/mechanism_binding.hpp`: the banner's TOGGLE section (every rule); `ToggleBinding`
  (`enter()`, `tick(connected, down)`, `entered()`, `button()`, `initialState()`, `mechanism()`;
  the private `forgetLevel()` re-seeds its `hal::ButtonEdge` "down"); `validBindings(bindings,
  toggles, rezero, why, cap)` — the ONE set check, its order documented (nulls, re-zero, one button
  one job across both kinds, a mechanism bound twice); the T3 hold-only `validBindings` now forwards
  to it (every T3 message byte-identical — B11b/L11 unchanged and green).
- `teleop/teleop_loop.hpp`: the body gained `std::span<ToggleBinding* const> toggles` after
  `bindings`; entry `enter()`s every toggle after the hold bindings' reset+makeSafe; the disable
  path touches no toggle (a comment says so); toggles tick after the hold bindings. A new
  forwarding overload carries T3's 9-argument signature (empty toggle span); the 2a overload is
  unchanged. Banner paragraph and the `toggles` / `disabled` doc bullets. `DriverLoopConfig`
  untouched (DEFECTS2 and 2b own that paragraph).
- `hal/device_provider.hpp`: `hal::kBrainAdiLines = 8`; pure virtual `digitalOut(line,
  initialState)`; banner paragraph "A 3-WIRE LINE IS THE EXCEPTION". `hal/pros/device_provider.hpp`:
  `digitalOut()` over `std::optional<ProsDigitalOut> lines_[9]` + `lineInitial_[9]`, range-only
  refusal, conflict precondition, banner section. `hal/fake/fake_device_provider.hpp`:
  `digitalOut()` driving the line when made, `bindDigitalOut`, `fakeDigitalOut`,
  `refuseLine`/`allowLine`, `digitalOutRequestCount`, `digitalOutMade`, `digitalOutInitialState`.
  `hal/pros/digital_out.hpp`: comments only — "the boot level must agree with the safe state"
  now also admits a config's stated START level (the Robot hands the provider `pneumaticStart`).
- `config/robot_config.hpp`: `PneumaticLevel`, `pneumaticLevelWord`, `isPneumaticLevel`,
  `toLineValue`; `MechanismConfig::pneumaticSafe` / `pneumaticStart`; `valid()`'s line-owner table
  (first pass) and the second pass's mixed refusal and level refusals; docs.
- `robot/robot.hpp`: banner (the one device write the builder now causes; the MECHANISMS
  section), `validate()` doc, constructor doc, `pneumaticMechanismAt(i)`, `pneumaticMechanism(name)`
  (+const), the typed-lookup note on `mechanism(name)` / `mechanismAt(i)`, slot storage
  (`lines[2]`, `std::optional<hal::PneumaticMechanism> pneumatic`), `buildPneumatic()`, and
  `copyAbsence()`'s fallback sentence as a parameter (motor default unchanged).
- Existing tests changed because the behaviour they pinned changed ON PURPOSE: 1i's legal set (its
  motor intake had a line on it — now a separate "intake flap" pneumatic mechanism); 1m' and 1m'''
  (their pneumatic-only mechanisms now state their levels); builder M3's pneumatic-only subcase
  (built now — it became "a mechanism with neither motors nor lines is not built"); builder M5's
  `SilentRefusal` test double answers `digitalOut()` (the new pure virtual).

**Suite after the code, before new tests: 1453 cases / 2,389,414 assertions, 3 skipped — SUCCESS.**

#### B2.4 The tests — 21 new cases

- `test/toggle_binding_test.cpp` (new, 11 cases): **T1** one press = one flip, held 50 ticks = one,
  press-release-press = two, ten taps alternate; **T2** the flip is the opposite of the COMMANDED
  level (a guard's `applySafeState()` in between, and an outside `set()`); **T3** the initial level
  is the stated one, sent at the FIRST `enter()` only — never at construction, a tick, or a later
  `enter()` — plus `static_assert`s that a ToggleBinding without the initial level does not compile
  (the "no default" property is a signature, so it is pinned at compile time) and is neither
  copyable nor movable, and the non-button refusal; **T4** the lie (A "down" while disconnected,
  still down at the reconnect — no flip), pressed-and-released while gone, down at entry, pressed
  while the loop was not running and down at the re-entry; **T5** a claimed first entry never
  sends the initial level (nor a later one), a press under a claim is dropped and not carried over;
  **T6** `validBindings` across both kinds — robot two's set with the re-zero on Y valid; the claw
  on A with the re-zero on A; the claw on a pivot's button; two claws on one button; one claw
  toggled twice; a null toggle; the documented order (a hold binding's clash reported first); the
  hold-only form. Loop, on the host test plant beside robot two's two pivots: **TL1** the initial
  level lands before the first poll, the press at tick k flips in tick k after `drive()` AND after
  the pivots (a snooping line records the drive's volts and pivot 6's event count at the flip),
  held = nothing, the exact button-read count; **TL2** the lying controller (no read while
  disconnected, no flip, no replay, the next seen press flips); **TL3** a disable while OPEN
  commands nothing to the claw while both pivots go safe in the same tick, and the re-entry after
  it (A pressed during the pause) commands nothing either; **TL4** A touches only the claw, the
  pivot buttons only the pivots, and the drive's volts and commands are BIT-identical across the
  two-span, one-span and no-span signatures (and the pivots identical with or without a toggle);
  **TL5** the loop refuses a toggle on the re-zero button or on a pivot's button before the first
  poll, before the initial level, before any pivot (its poll ends the loop after three polls, so a
  lost refusal is red, not a hang).
- `test/robot_config_test.cpp` (4 cases, 1n..1n'''): UNSET levels are never Low and the helpers;
  unset/unknown safe and start levels refused by name (one- and two-line), all four pairs
  accepted, a motor mechanism not asked for levels; the mixed mechanism refused (for the mix, even
  with its levels unset); a 3-wire line claimed by two mechanisms or twice by one (exact
  sentences), a line past the count not read, and the line claims before the level checks.
- `test/robot_builder_test.cpp` (3 cases): **FP1** the fake provider's digital out (same object,
  driven once when made and never again, a precondition on a second level, out-of-range incl. 'A'
  refused by number, a refused line counted and never driven, a bound line served and driven);
  **P1** the Robot builds and names the claw (made at START=High, safe Low — deliberately different;
  the name borrowed from the Robot's copy; the motor lookup does not hand it out and the pneumatic
  lookup does not hand out a pivot; the span in config order; the guard's `applySafeState()` lands
  Low; `validate()` never asks for the line; the drive BIT-identical to the hand-wired graph; a
  two-line clamp built in config order at the start level); **P2** a refused line ⇒ that claw
  ABSENT with the provider's sentence, the drive bit-identical; a second line refused ⇒ absent as a
  whole; a silent refusal ⇒ the line-specific fallback sentence.
- `test/pros_device_provider_test.cpp` (2 cases): **1l** through the shim — made = driven once at
  the stated level (the shim's per-line write history), the same object, never re-driven, the
  registry never asked (and every smart port reads NONE), the conflict precondition's exact words,
  `0, 9, 'A', 'a', 255` refused by number with line A untouched; **1m** a Robot with the claw over
  the REAL provider: `validate()` drives nothing, construction drives port A once at START, the
  mechanism's commands and its safe level reach the line, no other 3-wire line touched.
- `test/cookbook_examples_test.cpp`: **cookbook-06b** (the recipe, compiled; see B2.6).

Before the campaign, a dry pass of every planned mutant against these tests predicted four
survivors and one hang, and the tests were strengthened FIRST: T5's up-tick is now taken with
nobody claiming (else a toggle that skips the edge during a claim — MT10 — hides behind the entry's
"held" seed); T4's and TL3's re-entry now has A last seen UP and pressed during the pause (else
MT11, "enter() keeps the old level", is invisible); T6's order subcase puts BOTH kinds on the
re-zero (else MV5, "toggles checked first", is invisible); 1n''' puts the unset-level mechanism
BEFORE the colliding one (else MC18, "level checks in the claim pass", is invisible); TL5's poll
ends the loop (else ML26 hangs the suite instead of failing it). These were predictions from
reading the code, not measurements; the campaign below is the measurement.

**Suite with the new tests: 1474 cases / 2,392,200 assertions, 3 skipped — SUCCESS** (+21 cases).

#### B2.5 Documentation (written while the campaign ran; nothing under `include/` or `test/` was touched meanwhile)

- `docs/changelog.md` — inside T3's "Added since 2.2 (2026-09-26)" section (not a new section, so
  the DEFECTS2 conflict stays the one §10.9 resolved): the note now says the provider gained a
  member every implementer must answer; bullets for `ToggleBinding` and the two-set
  `validBindings`, the loop's toggle span, `PneumaticLevel` and the two fields, the provider's
  `digitalOut()` (both implementations, the fake's hooks), the Robot's pneumatic mechanisms; the
  "Pneumatic lines are still not built" sentence replaced. Behaviour changes with migrations: the
  new refusals (unset levels, mixed mechanism, a line listed twice) and the new pure virtual.
- `docs/guide/10-the-api.md` — a paragraph after "Buttons for mechanisms": the pneumatic claw on
  one button, the seen-press rule, the stated start level sent once, the disable that leaves the
  claw alone, the one-set check, the config fields and `pneumaticMechanism()`, the honest status
  (HA-157 named).
- `docs/cookbook/06-driver-control.md` — the chapter intro names both compiled cases; a new recipe
  "Open and close a pneumatic claw with one button" whose two listings are quoted verbatim from
  `cookbook-06b` (one polarity constant, the claw beside a pivot, the re-zero on Y, the two-list
  check, the two-span loop call), with the tick-by-tick result the test checks, why, and
  watch-outs (the re-zero, the polarity guess, two valves move together — not a double-coil
  solenoid — the config route, HA-157). `docs/cookbook/README.md` — its table row.
  `test/cookbook_examples_test.cpp` — `cookbook-06b` and the mapping comment (06a/b).
- `docs/vexbuilder-integration.md` — §3.1's `mechanisms` schema gains `pneumatics: ["A".."H"]`,
  `pneumaticSafe`, `pneumaticStart` and "motors OR pneumatics"; §3.2's field note on the two levels
  (no default, why), the letter-to-number mapping, one port once, two ports = two valves together,
  nothing on the brain can confirm a solenoid; the stale "Pneumatic lines are checked but not built
  yet" removed.
- `docs/hardware-assumptions.md` — index rows and full entries for **HA-157** (UNKNOWN: a 3-wire
  output's level while the brain is disabled) and **HA-158** (reasoned: nothing the program can read
  sees a solenoid on a brain 3-wire port), a paragraph in the intro, and the status line's total
  (141 → 143 in this tree; see the merge notes). Only these two numbers were used.
- Stale sentences found by the prose review and fixed (docs move with the code):
  `docs/guide/13-extending-the-library.md` (the Robot hands you "the motor mechanisms" → motor and
  pneumatic, and `pneumaticMechanism()`), `docs/guide/14-what-it-cannot-do-yet.md` (mechanism
  buttons now include the toggle), `docs/guide/08-your-first-routine.md` ("before it opens
  anything it asks the brain what is plugged in" — true of smart ports only now; the 3-wire
  exception stated), `docs/faq.md` ("make the boot level the mechanism's safe state" — a
  config-built mechanism's boot level is its `pneumaticStart`), `README.md` (the Mechanisms bullet),
  `docs/internal/ORIENTATION.md` ("Not built yet" line), and `hal/pros/digital_out.hpp`'s two
  comments saying the same thing as the FAQ.

#### B2.6 Resumed by a second executor run — what was found in the tree first

The first B2 run stopped after B2.5 (no campaign table, no gates, no prose-review entry in this
log). This run started by checking, not trusting:

- **No mutant left applied.** The first run's pre-campaign SHA-256s (its scratchpad
  `b2mut/pre.sha256`, 05:34) were checked against the six headers it mutated: all `OK`, and
  `robot.hpp` is byte-identical to the copy the first run saved before its last (sanitizer) mutant.
- **Its campaign was on disk but not here:** 55 mutants, every one RED except **MR28
  (`pneumaticMechanismAt()` without its bound) — SURVIVED** — and MT13 (a default initial level),
  which does not compile: the `static_assert` in `toggle_binding_test.cpp` is the test. After that
  the first run added builder **P3** (a heap-placed Robot asked for index `kMaxMechanisms`) and
  built an AddressSanitizer tree, where MR28 aborted in P3; neither was logged. So B2.4's count is
  one case short: B2's new cases are 22 (11 toggle, 4 config, FP1/P1/P2/P3, 1l/1m, cookbook-06b).
- **Stale generated files:** `docs/api` had never been regenerated for B2 (`check-fresh` exit 1:
  `PneumaticLevel`, `pneumaticSafe`/`pneumaticStart`, `digitalOut`, `pneumaticMechanism*` missing)
  and the briefing still said 1,425 cases and HA-142 next. So a full `cmake --build` of the tree as
  found would have gone RED on the doc gates.
- **Suite as found (fresh relink):** 1475 cases / 2,392,217 assertions, 3 skipped — SUCCESS.

The cookbook's compiled case is `cookbook-06b` (B2.5 said "see B2.6"; B2.5's own bullet is the
account of it — this entry adds nothing to it).

#### B2.7 What this run changed (each re-checked against the code first)

- **The "say how" for a double solenoid** — the addendum (§0) asks for it and the first run did
  not give it: `config::MechanismConfig`'s banner (`robot_config.hpp` ~:434-441) now says how a
  two-coil valve WOULD be built (its two lines stay one mechanism; each line gains a stated
  polarity; an inverted line is asked for at the opposite start level and handed to the mechanism
  through a one-line `IDigitalOut` adapter that writes and reports the opposite level — so the
  mechanism, the toggle and the guard's span are untouched), and says it is not built. The
  cookbook watch-out links to it, and `vexbuilder-integration.md` §3.2 says no polarity field
  exists yet, so none is emitted. This answers E2's `mechanism_table.hpp` note ("the library half,
  B2, says how config::MechanismConfig's second pneumatic line would carry it"): **as the library
  stands, a second line in one mechanism cannot carry a second coil** — it gets the SAME level.
- **`Robot::buildPneumatic`'s doc** (`robot.hpp` ~:1150-1157) now states the partial case P2 pins: a
  line made before a refused one stays made (driven to the start level) and belongs to no
  mechanism; on a brain it cannot happen (the only on-robot refusal is a line outside 1..8, which
  `valid()` refuses first).
- **One new subcase, before the campaign:** T4 "ticked before any enter()" — the constructor's own
  "held" seed. Predicted from the code that mutant MT16 (the constructor does not seed the edge)
  would survive, then MEASURED: MT16 on a fresh binary with this subcase excluded
  (`-sce="ticked before any enter*"`) — 1475 cases green, SURVIVED; with it — RED (T4).
- **Docs:** guide 13's hand-wired `ProsDigitalOut` paragraph said "pass it the same value you
  declare as the mechanism's safe state" — true only for hand-wiring since B2; it now says "the
  level the mechanism must be at boot", naming `pneumaticStart` for a config-built robot (the FAQ
  and `digital_out.hpp` already did). Guide 14's "Not proven" bullet gains HA-157 and HA-158 (the
  honest-status chapter did not mention either). The cookbook: "the claw is closed from the moment
  the robot is built" held only for a claw whose start level is closed — reworded, plus "give the
  toggle the same starting level, or the claw moves when driver control first starts" (the Robot
  drives `pneumaticStart` at build; the toggle sends its own initial level at the first entry).
  Reflowed: `hal/pros/digital_out.hpp`'s two comments, guide 13 and ORIENTATION's lines left
  ragged by the first run.
- **Generated:** `docs/api` regenerated after the last header edit (`check-fresh` exit 0); the
  briefing regenerated LAST, from a freshly relinked binary.

#### B2.8 The mutation campaign — 67 mutants, every one RED on a fresh binary (one at compile time)

Harness as §7 (scratchpad `b2fin/`, a copy of the first run's): per mutant an exact-once text
change (every anchor checked to occur exactly once before the run), `touch test/*.cpp`, `cmake
--build build/test --target shulib_tests/fast -j8` with the "Linking CXX executable shulib_tests"
line REQUIRED, the WHOLE suite, restore, SHA-256 verified; all six mutated headers `OK` after the
campaign (against hashes taken after this run's last header edit). The first run's 55 mutants were
re-run on the final code — not carried over from its log — plus 12 new ones (ML28, ML29, MR30,
MR31, MR32, MR35, MC19, MF5, MF6, MP6, MP7, MT16) for gaps found by re-reading the code. "Killed
by" lists the failing cases the harness recorded (at most six). Test IDs: T*/TL* are
`toggle_binding_test.cpp`, config 1n* `robot_config_test.cpp`, builder FP1/P* `robot_builder_test.cpp`,
pros 1l/1m `pros_device_provider_test.cpp`.

**The addendum's four named mutations:** flip on level instead of edge = **MT1**; replay an edge
across a disconnect = **MT2** (and MT2b, MT2c, MT3); command safe on disable = **ML21**; default the
start value = **MT13** (the binding's), **MC12** (the config's check) and **MR21** (the builder's).

| # | Mutant | Verdict | Killed by |
|---|---|---|---|
| MT1 | flip on the LEVEL instead of the edge | RED (7) | cookbook-06b, T1, T4, T5, TL1, TL2 |
| MT2 | no re-seed on a disconnect (an edge replayed across it) | RED (2) | T4, TL2 |
| MT2b | an unknown level re-seeded as UP, not held | RED (3) | T4, TL2, TL3 |
| MT2c | a disconnected tick feeds 'up' to the edge (the naive reading) | RED (2) | T4, TL2 |
| MT3 | a level handed over while disconnected is trusted | RED (1) | T4 |
| MT4 | the flip comes from a private belief, not the line's commanded level | RED (1) | T2 |
| MT5 | the initial level re-sent on EVERY entry | RED (4) | T3, T4, T5, TL3 |
| MT6 | the initial level never sent | RED (9) | cookbook-06b, T1, T2, T3, T4, TL1 |
| MT7 | enter() ignores a claim | RED (1) | T5 |
| MT8 | a claimed first entry defers the initial level (sent late) | RED (1) | T5 |
| MT9 | tick() ignores a claim | RED (1) | T5 |
| MT10 | a press under a claim is carried over (edge not fed during a claim) | RED (1) | T5 |
| MT11 | enter() keeps the old level (a finger held across a re-entry is a press) | RED (2) | T4, TL3 |
| MT12 | the constructor accepts a non-button byte | RED (1) | T3 |
| MV1 | no re-zero check for toggles | RED (3) | cookbook-06b, T6, TL5 |
| MV2 | toggle buttons left out of one-button-one-job | RED (2) | T6, TL5 |
| MV3 | a claw toggled twice accepted | RED (1) | T6 |
| MV4 | no null check for toggles | RED (1) | T6 |
| MV5 | toggles checked BEFORE hold bindings for the re-zero (order) | RED (1) | T6 |
| ML20 | toggles never entered | RED (5) | cookbook-06b, TL1, TL2, TL3, TL4 |
| ML21 | the disable makes the toggles safe (commands the claw) | RED (4) | cookbook-06b, TL1, TL3, TL4 |
| ML22 | a toggle's button read while disconnected | RED (1) | TL2 |
| ML23 | the toggle always told 'connected' | RED (1) | TL2 |
| ML24 | toggles ticked BEFORE drive() | RED (1) | TL1 |
| ML25 | toggles ticked before the hold bindings | RED (1) | TL1 |
| ML26 | the loop's precondition checks the hold bindings only | RED (1) | TL5 |
| ML27 | the toggle loop removed (toggles never ticked) | RED (5) | cookbook-06b, TL1, TL2, TL3, TL4 |
| MF1 | fake: a line re-driven on every request | RED (1) | builder FP1 |
| MF2 | fake: making a line does not drive it | RED (2) | builder FP1, builder P1 |
| MF3 | fake: no one-line-one-configuration precondition | RED (1) | builder FP1 |
| MF4 | fake: a bound line ignored | RED (2) | builder FP1, builder P1 |
| MP1 | pros: a fresh line made (and driven) per request | RED (1) | pros 1l |
| MP2 | pros: no upper bound on the line (a letter byte reaches PROS) | RED (1) | pros 1l |
| MP3 | pros: the registry consulted for a 3-wire line | RED (2) | pros 1l, pros 1m |
| MP4 | pros: no one-line-one-configuration precondition | RED (1) | pros 1l |
| MP5 | pros: the stated level dropped for PROS's default LOW | RED (2) | pros 1l, pros 1m |
| MC11 | valid(): no safe-level check | RED (2) | config 1n', config 1n''' |
| MC12 | valid(): no start-level check (the start value defaulted) | RED (1) | config 1n' |
| MC13 | isPneumaticLevel() accepts UNSET (UNSET reads as a level) | RED (3) | config 1n, config 1n', config 1n''' |
| MC14 | valid(): a mixed motor+line mechanism accepted | RED (1) | config 1n'' |
| MC15 | valid(): a 3-wire line listed twice accepted | RED (1) | config 1n''' |
| MC16 | toLineValue() inverted | RED (3) | pros 1m, builder P1, config 1n |
| MC17 | valid(): a line collision's owner mis-named (same-mechanism branch always) | RED (1) | config 1n''' |
| MC18 | valid(): the level checks moved AHEAD of the line claims (first pass) | RED (2) | config 1n', config 1n''' |
| MR20 | the line made at the SAFE level | RED (2) | pros 1m, builder P1 |
| MR21 | the line made at a default (false) — the start value defaulted | RED (2) | pros 1m, builder P1 |
| MR22 | the safe level taken from the start level | RED (2) | pros 1m, builder P1 |
| MR23 | the pneumatic mechanism left out of the guard's span | RED (2) | pros 1m, builder P1 |
| MR24 | only the first line built | RED (1) | builder P1 |
| MR25 | the provider's sentence not copied for a refused line | RED (1) | builder P2 |
| MR26 | no line-specific fallback sentence (the motor one used) | RED (1) | builder P2 |
| MR27 | a refused line skipped and the mechanism built anyway | RED (1) | builder P2 |
| MR28 | pneumaticMechanismAt() unbounded | RED (1) | builder P3 |
| MR29 | a motorless mechanism never built as pneumatic | RED (5) | pros 1m, T3 builder M3, builder P1, builder P2, builder P3 |
| MT13 | a DEFAULT initial level (expected: the T3 static_assert refuses to compile) | RED at compile time | compile time: the `static_assert` at toggle_binding_test.cpp:201 |
| ML28 | toggles entered BEFORE the set check (the claw moves, then the loop refuses) | RED (1) | TL5 |
| ML29 | toggles entered on the first TICK, after the first disabled() poll | RED (1) | TL1 |
| MR30 | validate() asks the provider for every 3-wire line (and so DRIVES it) | RED (3) | pros 1m, builder P1, builder P2 |
| MR31 | the pneumatic mechanism named with a literal, not the Robot's config copy | RED (1) | builder P1 |
| MR32 | a BUILT pneumatic mechanism still reports an absence | RED (1) | builder P1 |
| MR35 | every line of a mechanism asked for at its FIRST port | RED (2) | builder P1, builder P2 |
| MC19 | valid(): the mixed-mechanism refusal moved AFTER the level checks | RED (1) | config 1n'' |
| MF5 | fake: a REFUSED line request not counted | RED (2) | builder FP1, builder P2 |
| MF6 | fake: line 0 served | RED (1) | builder FP1 |
| MP6 | pros: the made line's level never recorded (the conflict check compares against low) | RED (1) | pros 1l |
| MP7 | pros: line 0 accepted | RED (1) | pros 1l |
| MT16 | the constructor does not seed the edge 'held' (only enter() does) | RED (1) | T4 |

Notes, honestly:
- **MT13 is a compile-time kill, and that is the test**: "the initial level has no default" is a
  property of the constructor's SIGNATURE, pinned by `static_assert(!std::is_constructible_v<
  ToggleBinding, PneumaticMechanism&, ControllerButton>)` — unlike §7's MB20/MR3, which died only on
  `-Werror` and were re-expressed.
- **MR28 (`pneumaticMechanismAt()` without its bound) is RED here by undefined behaviour, not by a
  promise.** For an index between the declared count and the capacity the slots are empty anyway,
  so only an index at or past `kMaxMechanisms` separates the bound from its absence, and there the
  mutant reads past the array. Measured three ways: (1) the first run's campaign, before builder
  P3 existed — SURVIVED; (2) this campaign — RED on P3 (the heap-placed Robot asked for index
  `kMaxMechanisms`): the stray byte happened to be non-zero in this binary; (3) an
  AddressSanitizer build of the whole suite (scratchpad `b2fin/asan`, `-fsanitize=address
  -fno-omit-frame-pointer`, Debug), MR28 applied: **heap-buffer-overflow at
  `robot_builder_test.cpp:2157`** (P3), exit 1 — and with P3 excluded (`-tce="B2 builder P3*"`),
  1474 cases GREEN under the sanitizer: P1's index-99 check on a stack Robot does not catch it
  (its far neighbour is valid stack memory). So P3 is the case that exercises the bound, the
  sanitizer is what makes the catch certain, and the ordinary suite can only be lucky. §7's
  MR12/MR13 (the motor accessors) are the same shape. The final code under the sanitizer:
  **1475 cases / 2,392,219 assertions, 3 skipped — SUCCESS, no sanitizer report.**
- Not mutated, with why: the `forgetLevel()` helper's `ButtonEdge{}` reset (it is what makes the
  seed exact; every variant of "seed wrongly" is MT2b/MT2c/MT11/MT16); `pneumaticLevelWord()`'s
  labels beyond those 1n asserts; the fake's `checkedLine()` test-hook bound (a precondition on
  test code, not library behaviour).

#### B2.9 Prose review (grep run and hits read before this entry)

Greps over `docs/*.md`, `docs/guide/`, `docs/cookbook/`, `README.md`, `include/shulib/` and
`docs/internal/ORIENTATION.md`: `pneumatic`, `solenoid`, `digital.out`, `digitalOut`, `3-wire`,
`three-wire`, `no digital-out`, `provider has no`, `pneumatic lines are`, `lines are checked but`,
`not built yet`, `pneumatic.*not built`, `toggle.*future`, `button bindings`, `bindings for
mechanisms`, `L9`, `mechanism` (guide 14, guide 08, the FAQ), and `simulator` in every line B2
added. Stale and fixed: guide 13's `ProsDigitalOut` boot-level sentence; guide 14's "Not proven"
bullet (silent on the 3-wire beliefs); the cookbook's "closed from the moment the robot is built".
Hits read and left, with why: `docs/cookbook/README.md` "What is not in here — Named mechanisms"
(still true: no `Intake` type); `docs/shulib-v2-master-plan.md` :264/:346/:701/:713 (the plan's
history of F1 and the `.vexbot` schema sketch, dated); `docs/legacy-command-vocabulary.md` (the
legacy robot's pneumatics, history); `docs/vexbuilder-integration.md` :51 (the VexBuilder
electrical UI gap — about the app, still true); `docs/roadmap.md` :1671 "Button bindings / driver
macros" (outside this run; R1's amendment and the checkbox are the program half's and the
coordinator's); guide 10 :186 "on the simulator" (not B2's line — G1 Part 2b has already
reworded it to "host test plant" at `7f9b9a1`, and the trial merge below takes that). No
`simulator` in any line B2 added; no non-ASCII byte in any string literal B2 added (grep over the
header diff).

#### B2.10 A trial merge onto shulib-v2 HEAD (evidence for the coordinator, scratchpad only)

`git archive ea58fbc` (HEAD now: G1 Part 2b, DEFECTS2 at `a8540e6`, the DIAG1/COMP brief commit)
into `b2fin/trial`, then every file this worktree changes (T3 phase 1 + B2) 3-way merged over
`b622d9b` with `git merge-file` (read-only on both sides), `docs/api` regenerated. **Conflicts: 3
files, 7 hunks — all the "both sides added" kind:**
- `teleop/teleop_loop.hpp`, 2 hunks — exactly §10.9's two, now carrying B2's lines too: (1) the
  parameter bullets — DEFECTS2's `cfg` bullet (it names `releaseBrakeMode`), then T3's `bindings`,
  B2's `toggles` and T3/B2's `disabled` bullet ("every hold binding's `makeSafe()` — no toggle is
  touched"); (2) after `chassis.drive(...)` — DEFECTS2's `applyReleaseMode()` FIRST, then the hold
  bindings, then the toggles. Plus the no-conflict edit §10.9 names, now "…`drive(Body)`, the release
  mode, the mechanism bindings and toggles, `perTick`, `pace`". (`applyReleaseMode()` walks the
  DRIVE motors only, `chassis.deps().ctx->driveMotors()`, so it never touches a mechanism.)
- `docs/changelog.md`, 1 hunk — T3's "Added since 2.2" section (with B2 in it) above DEFECTS2's
  "Fixed since 2.2", as §10.9 says.
- `docs/hardware-assumptions.md`, 4 hunks — the status total (146 on HEAD; 148 with HA-157/158,
  more once E2's land), the intro paragraph (DEFECTS2's sentence, then B2's), the index rows and
  the entry sections (both, in number order). At merge HA-157 can name DEFECTS2's **HA-145** (what a
  motor stopped under Hold does while the robot is disabled) as its motor sibling — not written
  here, because HA-145 does not exist in this tree.
- Clean: `README.md`, guide 08/10/14, `vexbuilder-integration.md`, `robot_config.hpp`,
  `pros/device_provider.hpp`, `robot.hpp`, `pros_device_provider_test.cpp`,
  `robot_builder_test.cpp`; every other file changed on one side only.

Resolved as above (a script, for the trial only), `git init` + one commit so the two build-hash
cases have an identity, fresh host build (the worktree's build type): **1514 cases / 2,392,881
assertions, 3 skipped — SUCCESS**; doc gates in the full build: coverage "2667 public entities
across 149 headers", examples "437 quoted lines, all verbatim", removability OK, staleness clean;
ARM header gate CLEAN (185 headers). Caveat: E2 (`t3-prog-prep`) is not in the trial, and the
coordinator's real merge is the one that counts. (Seen in passing, not B2's: the same trial built
with `-DCMAKE_BUILD_TYPE=Release` fails `-Werror=format-truncation` on `validate()`'s existing
`"%s cannot be made: %s"` snprintfs, `robot.hpp` ~:291/351/356 — present at `b622d9b`; the
worktree's and CI's host builds use no build type.)

#### B2.11 For the coordinator — flagged, not decided here

1. **The lookup is TYPED, a stated deviation** (B2.2 #10): the pneumatic claw is
   `robot.pneumaticMechanism("claw")` (`robot.hpp` ~:649), not `robot.mechanism("claw")`, which
   stays the motor lookup T3's program half is written against. One untyped lookup would return
   `hal::IMechanism*`, which no binding can use without a downcast. Same key, same name rules; both
   kinds are in `mechanisms()`, the RunGuard span.
2. **"Commanded once at loop entry" is read as once in the binding's LIFE** (B2.2 #5): a re-entry
   after a field disable sends nothing (that is what keeps the claw's state through a disable).
   For COMP: if autonomous moves the claw, the FIRST driver-control entry still sends the stated
   initial level once.
3. **The Robot drives `pneumaticStart` at build, and the toggle sends its own initial level at the
   first entry** — the program states the same level in both (the cookbook says so), or the claw
   moves when driver control starts.
4. **A double solenoid is not "the second pneumatic port"** (B2.7): E2's note expects B2 to say how
   the second line carries it; it cannot as the library stands. If Station J finds two cables, the
   polarity design in `MechanismConfig`'s banner is the change (a library chunk), not a table edit.
5. **`IDeviceProvider::digitalOut()` is pure virtual** (B2.2 #7): every implementer must add it —
   the tree's three are done; nothing in `src/` implements the provider (checked, here and at
   HEAD).
6. **HA-157 is UNKNOWN and the claw's disable behaviour rests on it**; Station J (E2's) settles it.
7. The stranded-claim gap (§10.2) applies to a toggle exactly as to a hold binding (the header
   says so) — the same ruling covers both.

#### B2.12 Final gates (all in this worktree, after the last edit to any header, test or public doc)

- Host suite, freshly relinked (the "Linking CXX executable shulib_tests" line seen) and then
  `cmake --build build/test` exit 0 with every doc gate inside it: **1475 cases / 2,392,219
  assertions, 3 skipped — SUCCESS** (T3 phase 1's end: 1453 / 2,389,412; +22 cases).
- Under AddressSanitizer (B2.8): 1475 cases, SUCCESS, no report.
- Doc gates: api_doc_tool self-test OK; coverage "2666 public entities across 149 headers, all
  documented"; `check-fresh` exit 0; examples "437 quoted lines, all verbatim (4 source files)";
  removability "no public doc references docs/internal/"; briefing "generated block current";
  staleness self-test OK (10 cases) and "clean (27 live docs …)".
- `tools/src_build_gate.py self-test`: "OK (33 detector cases, each against a real build)";
  `check`: "PASS (all 6 builds link …)", 0 disallowed warnings in every build, beacons asserted.
- ARM header gate (arm-none-eabi-g++ 13.2.1, the CI command): "CLEAN (all v2 headers)", 185.
- CI greps: "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds:
  core is sim-free".
- Nothing committed or pushed; no `pros`; no serial device; nothing under `src/`; the main tree and
  every other worktree only READ.

#### B2.13 Not done in B2 (outside it, by the launch prompt or the addendum)

E2's everything (`src/mechanism_table.hpp`'s claw, Station J, HA-159); moving Teleop's re-zero
button to Y and wiring the claw into shulib Teleop (the coordinator, at merge); `build-order.md`,
`roadmap.md`, `HANDOFF-remaining-work.md`, the worksheet and the COMP brief; `hal/pros/motor.hpp`
and the shim's brake behaviour (DEFECTS2's). A double-coil valve is described, not built.

#### B2.14 B2 review findings and their disposition (review round 1, 2026-09-26)

A reviewer reported five findings on B2, all minor. Each is re-checked below against the tree and,
where real and inside B2, fixed at the root; the rest say why not, with evidence. Nothing under
`src/`, the worksheet, `build-order.md`, `roadmap.md`, `HANDOFF-remaining-work.md` or the COMP
brief was touched; `t3-prog-prep` and the main tree were only READ. The gates were re-run after
the last edit (B2.14.7). **Suite before this round: 1475 cases / 2,392,219 assertions** (fresh
relink, the B2.12 number).

##### B2.14.1 Finding 1 — "never replayed" rests on an unregistered belief

**Re-checked: real.** `ToggleBinding::tick()` has one discriminator: `if (!connected) {
forgetLevel(); return; }`, then `edge_.update(down)`. So the FIRST connected tick's level is
believed. HA-103 claims only that a disconnected controller reads 0 and that `is_connected()` is
the only validity signal; nothing in the register says the flag and the button data change
TOGETHER. The failure is concrete: a stale "up" on the first reconnected tick while the driver
still holds A, then the live "down", is a rising edge — a flip nobody asked for (the claw opens on
what it holds). E2's J.9.5 (`t3-prog-prep` worksheet ~:1342, read-only) presses A while unplugged
but never HOLDS A across the plug-in, so it could not show this. The header's "structural", the
cookbook's "never opens by itself when a cable blips", guide 10's "nothing … is ever replayed" and
the changelog's "never replayed" were all stronger than the evidence.

**Fixed inside B2:**
- `mechanism_binding.hpp`'s A PRESS MUST BE SEEN rule: "structural" is gone; the rule now says
  the guarantee holds PROVIDED the flag and the levels change together (unmeasured, unregistered),
  names the exact failure (a stale "up", or a level bouncing across two 10 ms samples), and points
  at the test that pins the boundary. `tick()`'s doc says a CONNECTED tick's level is taken as
  live, the first after a reconnect included.
- Docs qualified the same way: the cookbook (plus the driver's workaround until it is measured —
  let go of A before pressing again after a controller drop), guide 10, the changelog, and the
  test file's banner.
- **A new T4 subcase, "WHAT THE GUARANTEE RESTS ON"**, pins the boundary in the direction of the
  hazard: connected-up, a dropped tick, a stale connected-up, then the live down → the claw FLIPS.
  Pinned so the boundary cannot move silently, not because the behaviour is safe. **MT17** (a
  one-tick reconnect settle — the obvious mitigation) is RED on exactly that subcase.

**Not done, and why:**
- **Registering the belief** — this run may use HA-157/158 only. **For the coordinator, at
  merge, under the next free number:** *"A V5 controller's connected flag and its button levels
  change together: on the first tick `is_connected()` is true after a disconnect,
  `get_digital()` already returns the live level; and a button's level does not bounce across two
  10 ms samples."* Confidence: invented (unmeasured). Source: `teleop/mechanism_binding.hpp` (A
  PRESS MUST BE SEEN), `test/toggle_binding_test.cpp` T4 ("WHAT THE GUARANTEE RESTS ON"). Blast
  radius: a phantom flip of the claw when the driver holds A across a reconnect (or on a bouncing
  release). Contained: `ToggleBinding::tick()`. (The code then gets its `A4 register HA-nnn` tag.)
- **The Station J step — for the coordinator (the worksheet is E2's):** add to J.9.5, after the
  existing steps: *"Open the claw. Hold A down. Pull the cable; plug it back in STILL holding A;
  keep holding for two seconds, then let go. Nothing may move until A is released and pressed
  again; then one press closes it."* → moved while A was held across the reconnect? YES / NO.
- **A code mitigation — flagged, not built.** A reconnect settle window (treat the first N
  connected ticks after a disconnect as unknown too; MT17 is its N = 1 shape) would change the
  addendum's "each rising edge flips exactly once" — it drops real presses inside the window — its
  N is a magnitude nobody has measured, and it does not cover a bounce during an ordinary release.
  That is the team lead's ruling, best taken after the J.9.5 step above has measured whether the
  failure exists at all.

##### B2.14.2 Finding 2 — HA-157 did not cover a level commanded while the brain is disabled

**Re-checked: real, and wider than reported.** `robot.hpp`'s `buildPneumatic()` writes
`pneumaticStart` the moment the Robot is built. At HEAD (`ea58fbc`, read-only), `initialize()`
calls `teleopInitialize()`, whose `st.robot.emplace(...)` (`src/teleop_program.cpp` :861, from
`7f9b9a1`) builds the Robot — so on a field the start level is written while the brain is
DISABLED. **It is a bench case too:** HA-128 (MEASURED 2026-09-10) says a program started from the
CONTROLLER's own menu comes up in competition mode, DISABLED. HA-157 claimed only that a level
already commanded is KEPT through a disable. E2's J.9.2 says "start slot 2" without saying how,
and a laptop start runs enabled.

**Fixed inside B2 (HA-157 is B2's):**
- HA-157's index row, title and claim now have two halves — (1) a commanded level is kept
  through a disable; (2) a level commanded WHILE disabled reaches the valve, at once or at the
  latest at the enable — with the Robot-in-`initialize()` and HA-128 cases named. Its Source adds
  `robot.hpp` and the two host tests that pin the library's half (TL3, pros 1m); its Settle adds a
  step for (2) — leave the claw at the other level, start slot 2 from the CONTROLLER's menu (the
  brain comes up disabled, HA-128), and record whether the claw moves at the start, at the enable,
  only when driver control begins, or never — and says a laptop start cannot answer (2). Its Blast
  radius adds (2): the claw's FIELD-START state rests on it (a preload not gripped from program
  start through autonomous, or the claw moving at the enable), bounded by the toggle's own
  initial level at the first driver-control entry (every `ProsDigitalOut::set()` writes the line —
  checked, no skip-if-equal), with the containment (re-command the start level at the top of
  `autonomous()` and `opcontrol()`, program-side).
- **Reconciliation direction 2, a gap B2's first run left:** HA-157's Source cited
  `teleop_loop.hpp`, which did not carry the ID. Now tagged there (the disable path), in
  `robot.hpp` (the MECHANISMS section and `buildPneumatic()`), and in the TL3 and pros 1m
  comments; `grep -rn "HA-157"` finds every file the entry cites.
- Docs: guide 10 and guide 14 (both halves), the cookbook (the "From a config" bullet now says
  the claw is TOLD to close when the robot is built; the watch-out covers both halves and what the
  driver sees if (2) is false), `vexbuilder-integration.md` §3.2's `pneumaticStart` note, the FAQ's
  boot-level entry and guide 13's pointer to it.
- **For the coordinator:** add HA-157's step (2) to Station J (the step text is in its Settle).
  Also noted: J.9.2's own reading of "at the start" depends on HOW slot 2 was started — the sheet
  should say.

##### B2.14.3 Finding 3 — a boot report written against the typed lookup calls the claw ABSENT

**Re-checked: real.** Before this round `mechanismAt(i)` and `mechanism(name)` were nullptr for a
BUILT pneumatic mechanism and `mechanismAbsence(i)` was "" for it, so the natural loop — "if
`mechanismAt(i)` is null, print ABSENT: `mechanismAbsence(i)`" — prints the working claw as
"claw ABSENT: ". No such loop exists yet (checked read-only: nothing in `t3-prog-prep`'s `src/`,
`include/` or `test/` calls any of these accessors), so this is the moment to close it.

**Fixed at the root — the reviewer's option (a), plus (b)'s documentation:**
- **`Robot::builtMechanismAt(i)`** (both constnesses) → `hal::IMechanism*`: the i-th declared
  mechanism if it was built, of EITHER kind. `IMechanism` has `name()`, so a boot line needs
  nothing else.
- **`mechanismAbsence(i)` is empty EXACTLY when `builtMechanismAt(i)` is non-null.** For an index
  past the declared count it now answers "no mechanism is declared at this index" — it answered
  "", which is the answer that means "built". (This moves T3 phase 1's own, unmerged, out-of-range
  answer; T3 M2 and M5 were updated with it.)
- Docs: `mechanismAt()` and `pneumaticMechanismAt()` now say "a TYPED lookup … null here does NOT
  mean absent — `builtMechanismAt()` answers that"; `mechanismCount()`; the banner's MECHANISMS
  section (a boot or panel line asks `builtMechanismAt()` and `mechanismAbsence()`, never
  `mechanismAt()`); a changelog bullet; guide 10; the cookbook's "From a config" bullet.
- Tests: a `checkReportContract()` helper asserts, for every index 0..kMaxMechanisms+1, that
  built ⇔ empty absence, that the two typed lookups never both answer, that `builtMechanismAt()` is
  whichever one did, and the sentence past the count — called from T3 M2, M3 (and both its
  subcases), M5, and B2 P1, P2, P4. **New builder P4:** a report loop written the way the header
  says, over one robot holding every case (a built pivot, a refused pivot, the built claw, a
  pneumatic mechanism with a refused line, one with nothing to build), pins the five report lines
  exactly, and shows the trap (`mechanismAt(2)` null, `builtMechanismAt(2)` not).
- Mutants: MR42 (non-const `builtMechanismAt()` blind to a pneumatic mechanism — the trap itself)
  RED on P1; MR43 (blind to a motor one) RED on M2; MR44 (the const overload blind to a pneumatic
  one) RED on P1 and P4; MR45 ("" past the count — the old behaviour) RED on 7 cases.
- **Still the coordinator's to ratify:** the typed lookup itself (B2.11 item 1). What changed is
  that the deviation no longer leaves a trap.

##### B2.14.4 Finding 4 — MR28 caught only under AddressSanitizer

**Re-checked: real** — B2.8's own note says so: for an index in [count, capacity) the slots are
empty anyway, so each accessor's separate bound was observable only past the array, by undefined
behaviour. The same held for T3's MR12/MR13.

**Fixed at the root — the reviewer's second option, made observable without UB:**
- **One private bound, `declaredSlot(self, i)`** — a static member template, so the const and
  non-const accessors share ONE expression (`i < cfg_.mechanismCount ? &slots[i] : nullptr`).
  `builtMechanismAt`, `mechanismAt`, `pneumaticMechanismAt` and `mechanismAbsence` all go through
  it; `mechanism(name)` / `pneumaticMechanism(name)` became `mechanismAt(mechanismIndex(name))`
  (the index is the count on a miss, which the bound turns into null — the four redundant
  `i < count ?` guards are gone).
- **The bound's loss is now a wrong ANSWER inside the array.** Because an undeclared index
  answers the sentence (finding 3) while an unused slot holds "", index == count — still inside the
  array — tells the bound from its absence deterministically. Every test that probes past the
  capacity checks an in-array index FIRST (M2 and P1 gained one; M5, P3, P4 and the helper already
  start there), so the first thing a lost bound meets is an assertion.
- **Measured, by hand, on a fresh binary (MR40, the bound deleted):** the first failure is
  `robot_builder_test.cpp:1671: ERROR: CHECK( std::string{robot.mechanismAbsence(2)} == "no
  mechanism is declared at this index" )` — the in-array assertion — and only AFTER it does M2's
  index-99 probe crash the run (SIGSEGV at :1605's case; the campaign's harness recorded "RED (1)"
  because doctest stops at the crash). **MR41** (the bound against the CAPACITY instead of the
  count — no UB possible) is RED on 7 cases by assertion alone.
- **The sanitizer gate — not added; recommended to the coordinator.** `.github/workflows/ci.yml`
  is shared infrastructure, outside B2's library half, and with the bound observable by assertion
  it is no longer what stands between a lost bound and CI. It would still catch what no assertion
  can (any other stray read). Measured cost in B2.14.6.

##### B2.14.5 Finding 5 — two loose ends in the register

1. **"The per-confidence counts below predate all eighteen"** — re-checked: literally true of the
   eighteen it names (HA-124..HA-141), but a reader can take it as covering B2's two. **Fixed in
   B2's own sentence, not in that one:** "The per-confidence counts below predate these two as
   well, and have no bucket for UNKNOWN, which is HA-157's confidence: they still sum to 122"
   (78 + 41 + 2 + 1, checked). Why not rewrite "all eighteen": DEFECTS2 at HEAD (`ea58fbc`,
   read-only) kept that sentence and added "The per-confidence counts below predate these five as
   well … they still sum to 122" to its own; B2 follows the committed convention, so the merge
   stays the take-both it already is (B2.10) and no third rewrite of an untouched line appears.
   DEFECTS2 also calls its UNKNOWN entries "UNKNOWN and labelled invented"; the recount it says is
   due is the place to decide one convention.
2. **HA-158's settle named a Station J step that does not exist** — re-checked: real (J.9.0 only
   LOOKS at the valve's cable, J.9.5 pulls the controller's). **Fixed:** HA-158's Settle now says
   it is a step the coordinator adds to J.9 and gives it: no hand in the claw, pull the valve's
   cable out of A, start slot 2, confirm the same boot line and no refusal, look at the Devices
   screen plugged and unplugged, plug it back in before the rest of J.9. HA-157's step (2) is
   marked the same way.

##### B2.14.6 This round's mutation table, gates and ASan

Harness as B2.8 (scratchpad `b2rev/`): exact-once replacement (every anchor counted first),
`touch test/*.cpp`, `/fast -j8` with the "Linking CXX executable shulib_tests" line REQUIRED, the
WHOLE suite, restore, SHA-256 verified (`robot.hpp` and `mechanism_binding.hpp`: OK after). 8 new
mutants, and every earlier `robot.hpp` mutant whose anchor survived this round's rewrite RE-RUN on
the final code (B2.8's MR20-MR27, MR29-MR32, MR35; T3's MR1, MR2, MR3b, MR4-MR10) — 31 in all,
**every one RED.** MR28, MR12 and MR13 no longer exist as separate mutants: their three bounds are
now MR40's one.

| # | Mutant | Verdict | Killed by |
|---|---|---|---|
| MR40 | `declaredSlot()` without its bound (the one bound every indexed accessor shares) | RED (1) | M2 — by the in-array assertion first (B2.14.4), then the run crashes at the index-99 probe |
| MR41 | `declaredSlot()` bounded by the CAPACITY, not the declared count | RED (7) | M2, M3, M5, P1, P2, P3, … |
| MR42 | `builtMechanismAt()` blind to a built PNEUMATIC mechanism (the boot-report trap) | RED (1) | P1 |
| MR43 | `builtMechanismAt()` blind to a built MOTOR mechanism | RED (1) | M2 |
| MR44 | `builtMechanismAt()` const overload blind to a pneumatic mechanism | RED (2) | P1, P4 |
| MR45 | `mechanismAbsence()` answers "" past the declared count (the old answer) | RED (7) | M2, M3, M5, P1, P2, P3, … |
| MR46 | `mechanismIndex()` maps a null name to index 0 | RED (1) | M2 |
| MT17 | a one-tick reconnect settle (the first connected tick after a drop is also unknown) | RED (1) | T4 ("WHAT THE GUARANTEE RESTS ON") |
| MR20-27, 29-32, 35 | B2.8's builder mutants, re-run | all RED | as B2.8, plus P4 on MR25/27/29/32 |
| MR1, 2, 3b, 4-10 | T3's builder mutants, re-run | all RED | as §7, plus P4 on MR3b/4/7/9/10 |

**AddressSanitizer (scratchpad `b2fin/asan`, `-fsanitize=address -fno-omit-frame-pointer`,
Debug; every test TU recompiled):** the final code, the WHOLE suite — **1476 cases / 2,392,653
assertions, 3 skipped, SUCCESS, no sanitizer report**, 9.2 s (the ordinary binary: 4.9 s); an
incremental rebuild of the ASan tree after touching every test file took 21 s wall at `-j8`. MR40
under the sanitizer: **heap-buffer-overflow at `robot_builder_test.cpp:2211`** (P3, index
`kMaxMechanisms`) — the second witness P3's comment promises. **For the coordinator:** a CI job
"host suite under AddressSanitizer" (configure `test/` with those flags, build, run the whole
suite) costs about that much, and would catch stray reads no assertion can; this round did not add
it (B2.14.4).

**Two traps met, recorded so the next run does not lose time to them.** (1) After a mutant is
applied BY HAND, `build/test`'s binary IS that mutant until it is relinked: the first timing run
after the manual MR40 run segfaulted for exactly that reason, and the suite was green again after
`touch test/*.cpp` and a `/fast` relink with the header's SHA-256 verified. (2) The `/fast`
target skips CMake's depend step, so a header change reaches only the test TUs whose own mtime
moved: an ASan rebuild after touching ONE test file linked some TUs with the mutant's inline
`Robot` accessors and some with the restored ones (an ODR mix; a spurious stack-buffer-overflow
in M5 with the correct header on disk). The harness's `touch test/*.cpp` is not ceremony — it is
what makes a `/fast` build honest. Both ASan results above are from runs with every test file
touched.

**Prose review, this round (greps run and every hit read before this entry):** over `docs/*.md`,
`docs/guide/`, `docs/cookbook/`, `README.md`, `docs/internal/ORIENTATION.md` and
`include/shulib/`: `never replayed`, `is ever replayed`, `never opens by`, `cable blip`,
`structural`, `reconnect`, `moment the robot is built`, `from the moment`, `closed from`, `3-wire.*
disabled` / `disabled.*3-wire`, `boot drives the line`, `mechanismAt`, `mechanismAbsence`,
`pneumaticMechanismAt`, `HA-157`, `src/teleop_program`, and `simulator` in every file touched.
Stale or too strong, and fixed: the four "never replayed" claims (header, cookbook, guide 10,
changelog — B2.14.1); the cookbook's "closed from the moment the robot is built" → "told to close";
the FAQ's and guide 13's "boot drives the line straight there" (true only of an ENABLED brain —
both now point at HA-157's open half); `vexbuilder-integration.md`'s "the moment the robot is
built" (same qualification); guide 14's HA-157 sentence (one half → both). Hits read and left, with
why: every other `structural` (other subsystems' design claims, not the toggle's); `reconnect` in
`mechanism_binding.hpp`'s HOLD rules ("a still-held button drives again after a reconnect" — true
of the hold binding, which drives on levels, not edges); `mechanismAt` in `docs/api/*` (generated,
regenerated); `src/teleop_program.cpp` in `HANDOFF-remaining-work.md`, `build-order.md` and the G1
brief (internal planning documents that name the file on HEAD, not this tree — outside this run,
and correct at merge). The cookbook's pointer "`mechanismAt(i)` … is empty" → "returns null" (a
pointer is not empty).

##### B2.14.7 Final gates, this round (all in this worktree, after the last edit to any header, test or public doc)

- Host suite, freshly relinked (the "Linking CXX executable shulib_tests" line seen; headers'
  SHA-256 verified against the pre-mutation copies): **1476 cases / 2,392,653 assertions, 3
  skipped — SUCCESS** (B2.12: 1475 / 2,392,219; +1 case, builder P4; the rest of the new
  assertions are the report-contract helper's and T4's new subcase). `cmake --build build/test`
  exit 0 with every doc gate inside it.
- Doc gates: api_doc_tool self-test OK; coverage "2668 public entities across 149 headers, all
  documented" (+2: `builtMechanismAt` and its const overload); `check-fresh` exit 0 after
  `api_doc_tool.py generate`; examples "437 quoted lines, all verbatim (4 source files)";
  removability "no public doc references docs/internal/"; briefing "generated block current";
  staleness self-test OK (10 cases) and clean — after one catch: HA-157 had named
  `src/teleop_program.cpp`, which does not exist in this tree (it is G1 Part 2b's, at `7f9b9a1`);
  the entry now names the commit instead.
- `tools/src_build_gate.py self-test`: "OK (33 detector cases, each against a real build)";
  `check`: "PASS (all 6 builds link …)", 0 disallowed warnings in every build, beacons asserted.
- ARM header gate (the CI command, arm-none-eabi-g++ 13.2.1): 185 headers, "CLEAN (all v2
  headers)".
- CI greps: "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds:
  core is sim-free".
- ASCII: the one new runtime string ("no mechanism is declared at this index") is ASCII; no
  non-ASCII byte in any string literal in the touched headers. No `simulator` in any line this
  round added.
- The briefing was regenerated LAST, from the freshly relinked binary.
- Nothing committed or pushed; no `pros`; no serial device; nothing under `src/`; the main tree
  and every other worktree only READ.

##### B2.14.8 For the coordinator (adds to B2.11; nothing here decided by this run)

1. **Register, at merge, the controller belief of B2.14.1** under the next free number (the text
   is there), and tag `mechanism_binding.hpp`'s A PRESS MUST BE SEEN rule with it.
2. **Station J (E2's worksheet), three steps this round names but cannot write:** the J.9.5
   hold-A-across-the-reconnect step (B2.14.1); HA-157's step (2), a start from the CONTROLLER's
   menu with the claw at the other level (B2.14.2) — and J.9.2 should say how slot 2 is started;
   HA-158's boot-with-the-valve-cable-out step (B2.14.5).
3. **Ratify the typed lookup** (B2.11 item 1) — now with `builtMechanismAt()` as the one
   kind-neutral "is it built?"; Teleop's boot line and panel row for the claw (the program half,
   at merge) should be written against `builtMechanismAt()` / `mechanismAbsence()`, never against
   `mechanismAt()`.
4. **A reconnect settle window for the toggle** is a ruling for the team lead after J.9.5's new
   step has measured whether the failure exists (B2.14.1; MT17 is its shape).
5. **An AddressSanitizer CI job** — recommended, not added (B2.14.4, B2.14.6: ~9 s of suite).
6. Merge notes (B2.10) unchanged in shape: this round's `robot.hpp`, `teleop_loop.hpp` (one
   comment inside B2's own disable-path block), `hardware-assumptions.md` (HA-157/158's own rows,
   entries and B2's own intro sentence), guides 10/13/14, the FAQ, the cookbook, the changelog's
   B2 bullets and `vexbuilder-integration.md` edits all sit inside hunks B2 already owns;
   `docs/api/*` and the briefing are regenerated, never hand-merged.

---

## Part 2 — the program-side prep (worktree `t3-prog-prep`, runs E and E2)

- **Chunk:** T3 — the claw pivots. THIS log covers only the part of T3's PROGRAM half that depends
  on neither the library half (worktree `t3-lib`, its own `T3-PROGRESS.md`), nor DEFECTS2 (worktree
  `defects2-lib`), nor G1 Part 2b's Teleop (the main tree): `src/mechanism_table.hpp`, Calibrate's
  candidate filter, their host tests, HA-153..HA-156, worksheet Station J. The coordinator
  concatenates this log with the library builder's at merge.
- **Brief:** `T3-claw-pivots-bindings-and-mechanisms.md` (held outside the repo by the coordinator
  on purpose; read in full, rulings R1-R4 and the "stated without objection" defaults included).
- **Started:** Sat Sep 26 03:30:41 AM EDT 2026
- **Tree at start:** `b622d9b` on branch `t3-prog-prep` (a git worktree), clean
- **Executor model:** opus
- **Rules:** do not commit, do not push, no `pros`, nothing on `/dev/ttyACM*`; edit only files in
  this worktree. "host test plant", never the other word. ASCII only in runtime strings.

Appended in real time. **Nothing in this log has run on a robot.** *(The section times were first
written as estimates and ran ahead of the clock; corrected at 04:06 from the files' own mtimes.)*

---

### Log

#### 03:30 — start
- `date` above; `git status --short` empty; HEAD `b622d9b`. This log created before any code.
- Read `docs/internal/RESUMING.md` (standing rules: docs in the same change, prose review logged,
  tests try to break the code, mutations on a fresh binary) and the T3 brief in full.

#### 03:30–03:33 — required reading
- `src/chassis_table.hpp` (all): the rule "a value is never invented", the Unset values, the
  provenance fields, the variant `#if` at :77 (`SHULIB_ROBOT_TANK_2026`; bench and X-drive both take
  the `#else`), `absPort` / `tableSideOf` (constexpr). `test/chassis_table_test.cpp`: it RE-TYPES
  robot two's table because the tank block is invisible on the host (its header, :9-14).
- `src/calibrate_program.cpp`: the census loop (:1114-1129 at b622d9b) is where registry motors
  become candidates (`candidates[n].port`, `ports[n]`); step 4 (:1174-1195) builds one `ProsMotor`
  per candidate (BLUE gearing written, then `stopAll` = 0 V + Coast); `emit`/`emitf` print the
  line after an `[R3D]` prefix. The file has no `#if`: it compiles in all six builds, so a header
  it includes is compiled in every variant.
- `include/shulib/hal/controller.hpp` (`ControllerButton`'s first enumerator is **L1**),
  `hal/motor.hpp` (`BrakeMode`'s first is **Coast**; `kMaxMotorVoltage` 12 V),
  `tools/src_build_gate.py` header (six builds, the TU list globbed from `src/`),
  `test/CMakeLists.txt` (`SHULIB_SOURCE_DIR` is defined for the host suite: tests may read source
  text — precedent `calibrate_panel_test.cpp:643`, `drivetrain_calibrator_test.cpp:1538`).
- The pros shim has no `pros/screen.h` / `pros/misc.h`: `calibrate_program.cpp` cannot be compiled
  on the host, so its call site can only be pinned by reading its source.
- **Evidence re-read, not taken on trust:** `evidence/r3d-session-2026-09-24b-calibrate-run1.log`'s
  census lists MOTOR on 11-18 and 20 only (`9 motor(s), 1 IMU(s)`); the 2026-09-25 Teleop capture's
  port check says `NEW motors not in the calibration: 14` — no 6, no 7. No capture has seen a claw.
- **Merge hygiene, read-only:** `diff` of this worktree's `src/calibrate_program.cpp`,
  `hardware-assumptions.md` and the worksheet against the main tree's in-progress copies (G1 Part
  2b): 2b edits calibrate at :17-18, :44, adds `#include "screen_text.hpp"` right after
  `#include "chassis_table.hpp"`, and replaces :240-296; the worksheet up to :894 (Station G);
  hardware-assumptions at :741-749, :969, :1070, :1401-1406. Station J (after Station I, ~:1105)
  and the candidate site (~:1124) are clear of all of it. The include: a scratch git repo showed
  that inserting `mechanism_table.hpp` directly AFTER `chassis_table.hpp` conflicts with 2b's
  insertion there, and inserting it directly BEFORE merges cleanly — so it goes before (out of
  alphabetical order, with a comment; the coordinator may reorder after the merge).
  *(INCOMPLETE — the main tree kept changing after this look, and the "clear of all of it" did not
  hold for `hardware-assumptions.md`: see "Review findings and their disposition", finding 5.)*

#### 03:33–03:40 — the table, the filter, the call site, the tests
- Baseline, fresh configure + build at b622d9b (clean): **1425 cases (3 skipped), 2,386,871
  assertions**, all pass.
- `src/mechanism_table.hpp` (new). Design decisions, each with its alternative:
  - **Robot two's table is defined in EVERY build** (`kRobotTwoMechanisms`); only the selector
    `kMechanisms` (a reference) depends on the define. Alternative: the chassis table's one-name-
    per-variant `#if`. Rejected because then the host suite could only test a re-typed COPY, and
    the brief's own mutation "swap R1/R2" in the real table would stay green; and defining the tank
    macro in a test TU would give `kChassis` two definitions across the suite (an ODR violation
    whose COMDAT merge could hand another TU the wrong table).
  - **`std::optional` for the safe mode and both buttons.** Alternative: the plain
    `hal::BrakeMode` / `hal::ControllerButton` the coordinator's task text names. Rejected because a
    member left out of a designated initializer is value-initialised: Coast (drops a loaded claw)
    and L1 (the port-7 claw's button and the loop's default re-zero button) — an invented value in a
    table whose rule is that none is. The coordinator converts with `*entry.forward` etc. at merge.
    The cartridge reuses the chassis table's `Cartridge` (its first enumerator IS `Unset`).
  - Eight named constexpr predicates, one property each (count, complete, ports, forward != reverse,
    unique buttons, avoid the reserved button, volts in (0, 12] NaN-proof, disjoint from the same
    variant's chassis table incl. its IMU port); static_asserts: checks 1-7 on robot two's table in
    EVERY build, all eight on the build's own pair. `kTeleopRezeroButton = A` restated with its
    dependency on G1 Part 2b stated (Teleop must take its re-zero button from it at merge).
  - **The selection restated as a static_assert** under the same define (`&kMechanisms ==
    &kRobotTwoMechanisms` for tank, `&kNoMechanisms` otherwise): a misspelt macro in the `#if` would
    otherwise compile clean and hand robot two an EMPTY table — Calibrate would take its claws again.
  - `isDeclaredMechanismPort(t, port)` matches on |entry| (a flipped pivot is still its port) and
    never matches a port outside 1..21 (an entry left at 0 must not make "port 0" a mechanism).
  - `splitDriveCandidates(t, span)` → `CandidateSplit{kept[], keptCount, dropped[], droppedCount}`,
    both in registry order (the calibrator's port list and motor list must stay index-aligned).
    *(REPLACED at review by the in-place `dropDeclaredMechanismPorts` — findings 1 and 2, below.)*
- `src/calibrate_program.cpp`: ONE block right after the census count line — the split, one
  `emitf("port %d: claw pivot (declared) -- not a candidate, not touched")` per dropped port, the
  candidate/port arrays re-synced to `kept`, `n = keptCount` — before step 4 builds any adapter.
  Also: the include; the "WHY THE CANDIDATES COME FROM THE REGISTRY" header paragraph (it said every
  registry motor is a candidate — now false; the new text says why the claws are the exception and
  that it is not a fourth human fact about the drivetrain); and the `n < 2` refusal's words, which
  would otherwise say "the registry found 1 motor(s)" on a brain with three (two of them claws):
  now "the registry found %u motor(s) besides the declared mechanisms; a drivetrain needs at least
  two". MEASURED, not estimated: a scratch program through the library's own
  `calibration::wrapWords<kMediumCols, 3>` gives 94 chars for n = 0/1 (95 for n = 21) and three
  complete lines — `the registry found 1 motor(s)` / `besides the declared mechanisms; a` /
  `drivetrain needs at least two` — so the refusal screen shows it whole (no "more on serial").
- Header syntax-checked alone with host g++ (the suite's warning flags) AND arm-none-eabi-g++ (the
  ARM header gate's flags) under no define, `-DSHULIB_ROBOT_TANK_2026` and
  `-DSHULIB_ROBOT_XDRIVE_INVENTED`: all six clean.
- `test/mechanism_table_test.cpp` (new): 11 cases, 210 assertions. clangd flagged
  `-Wmissing-designated-field-initializers` (an error under -Werror with clang) on a test entry that
  left fields out ON PURPOSE; rebuilt by assignment instead (same property: omitted = value-init).
- Suite after: **1436 cases (3 skipped), 2,387,081 assertions**, all pass (+11 / +210).

#### 03:40–03:47 — mutations (first battery, in the background), each on a FRESH binary
Runner: a scratch script (outside the repo) that, per mutation, backs the file up, applies an
exact replacement (each old string must occur once), `touch test/*.cpp`, rebuilds
`shulib_tests/fast` and requires the `Linking CXX executable shulib_tests` line, runs the
`mechanism table*` cases, and — where the tank build matters — syntax-checks the header under
`-DSHULIB_ROBOT_TANK_2026` with host g++ and arm-none-eabi-g++; then restores byte-exact (asserted).
Every one RED:

| # | Mutation | Where | Observed |
|---|---|---|---|
| M1 | swap R1/R2 on port 6 (the brief's) | table | RED, 2 cases: the contents case (forward/reverse) and the buttons case |
| M2 | port 7 on L1/R2 (the brief's) | table | host build FAILS: `static assertion failed: robot two: one button is bound to two mechanisms` (no binary); tank host+ARM the same |
| M3 | drop the filter call (the brief's) | calibrate | RED: the call-site case (the call is not found) |
| M3b | move the filter block after step 4's adapters | calibrate | RED: the call-site case (`newCount < firstAdapter` fails) |
| M4 | a claw port on 11 (the brief's) | table | RED, 5 cases (contents, predicate, filter, ports, disjointness); tank host+ARM: `static assertion failed: a mechanism port is a DRIVE port (or the IMU port) of this build's chassis table` |
| M5 | predicate compares the SIGNED entry (no absPort) | predicate | RED: the predicate case (a flipped -6 no longer drops port 6) |
| M6 | predicate loses its 1..21 guard | predicate | RED: the predicate case (an entry at 0 makes "port 0" a mechanism) |
| M7 | volts check as `v <= 0 \|\| v > 12` | volts | RED: the volts case (NaN passes) |
| M8 | the filter keeps declared ports | filter | RED: the filter case (8 assertions) |
| M9 | the selector's macro misspelt (`SHULIB_ROBOT_TANK2026`) | selection | host suite GREEN (expected — the host has no define); tank host+ARM: `static assertion failed: ROBOT=tank must select robot two's mechanism table` |
| M10 | disjointness forgets the IMU port | check 8 | RED: the disjointness case |
| M11 | the reserved-button check ignores `reverse` | check 6 | RED, 2 cases (buttons; the full-table case, whose sixth entry binds A as reverse) |
| M12 | uniqueness forgets forward-vs-reverse across entries | check 5 | RED: the buttons case |
| M13 | completeness forgets the safe mode | check 2 | RED: the completeness case |
| M14 | `mechanismCount` not clamped | count | RED: the count case |

After the battery: restored, rebuilt, `1436 passed / 2,387,081 assertions`; tank syntax clean.

**Why the call site is pinned by reading source, and how to check it for real.** `runCalibrate()`
is a whole PROS program: `pros/screen.h`, `pros/misc.h` and the screen/touch/SD code have no host
shim, so the call cannot be RUN on the host. The last case reads `src/calibrate_program.cpp` and
requires, in order: the census's `STAGE 0` line < `splitDriveCandidates(kMechanisms, {ports, n})`
< the log literal < the re-sync `candidates[i].port = split.kept[i];` < `n = split.keptCount;` <
the first `.motor.emplace(` — and exactly one call. It sees the call deleted (M3), moved (M3b),
fed another table, or its result ignored. *(WRONG as written — the review showed the lower bound
was the census HEADER, so the block moved ABOVE the census loop stayed green, and statement edits
inside the block survived; the site was rebuilt and re-pinned, findings 1 and 2 below.)* It cannot
see the line print on the robot: **the
coordinator's check is the tank-calibrate build (src gate) plus Station J.2 — the two `[R3D] port
6/7: claw pivot (declared)` lines in the first Calibrate capture with the claws plugged in.**

#### 03:41–03:49 — documentation (while the battery ran)
- `docs/hardware-assumptions.md`: index rows and a new subsection "T3's additions — robot two's claw
  pivots" (after HA-141, inside the Robot 2 group) with **HA-153** (RED on 6 and 7 + the
  unmeasured direction — reported, never seen), **HA-154** (a pivot stalled at 12 V heats at the
  vendored 2500 mA default, `include/pros/motors.h`; shulib's MOTOR_OVER_TEMP watches the drive
  only, `health_monitor.hpp` "over the drive"; timing invented and not stated), **HA-155** (a
  claw/drive cable swap is invisible to the registry — and what it does to Calibrate and Teleop:
  the calibrator's rule 5 means a claw that swings on the push is PULSED), **HA-156** (Hold holds
  a LOADED pivot — invented; claimed nowhere until Station J.5). Tags `A4 register HA-15x` added
  at each cited source (`src/mechanism_table.hpp` for 153/154/155/156, `src/calibrate_program.cpp`
  for 155). Status line: 7 of **145** in THIS tree, with a sentence saying the other chunks own
  HA-142..152 — **the coordinator sets the merged total** (and the briefing's "next free").
- `docs/internal/chunks/R3a-BENCH-WORKSHEET.md`: **Station J** (new, after Station I): J.0 before
  (Station F first; labels on the claw cables), J.1 census + cartridge colours, J.2 Calibrate's two
  lines quoted EXACTLY (final) + the push count "of 9" + nothing moves under power, J.3 Teleop's
  boot lines / banner / claw row (each `<<FROM TELEOP AT MERGE>>`, said in the intro: a copy with a
  marker is not ready to run), J.4 which way R1/R2/L1/L2 move which claw (a flip goes in the sign,
  never the buttons), J.5 the load test (sag at 0/10/30 s, the hand push that tells Hold / Brake /
  Coast apart), J.6 both buttons (R2), J.7 the controller off, J.8 heat against the stop (stop at 50
  °C, HA-44's 55 °C step), what to send back. Also two small true-ups in Station G that 2b's hunks
  do not touch: G.2.1 (the census shows ELEVEN motors once the claws are in) and G.7's order list
  (the claw lines come right after the census).
- `docs/internal/PROJECT-BRIEFING.md` regenerated (`briefing_status.py generate`): T3 in the open
  chunks, suite 1436 / 2,387,081, HA 145 / next free HA-157 (this tree only).

#### Prose review (grep first, then read the hits)
- `grep -rn "every motor the registry|every port the registry|every registry motor|takes every|..."
  docs README.md`: **`docs/internal/HANDOFF-remaining-work.md:111`** says "shulib Calibrate takes
  every motor the brain's registry reports as a candidate" — now true except for declared mechanism
  ports. **Not edited: HANDOFF is out of this worktree's scope** — flagged for the coordinator.
  `docs/internal/chunks/R3d-drivetrain-self-calibration.md:85/296` (a brief — history, not edited).
- `grep -rn -i "calibrat" docs/guide docs/*.md README.md | grep -i "registry|candidate|..."`:
  `docs/faq.md:196-198` "Calibrate builds its candidate list from the ports that actually carry a
  motor" — still TRUE (a subset of them now); not edited.
- `grep -rn -i "claw|port 6|port 7|ports 6|6 and 7"` over docs: no stale claw statement outside the
  files written here (`docs/api/robot_config.md:1001`'s "port 7 is claimed by the LEFT drive and by
  a mechanism motor" is a generated example, not a robot fact).
- `grep -rn "Station J|mechanism_table|kMechanisms"`: only this change's files.
- `src/calibrate_program.cpp`'s own header: the "WHY THE CANDIDATES COME FROM THE REGISTRY"
  paragraph said every registry motor is a candidate — fixed in the same change (above). Its "THREE
  facts" paragraph stays true (the new text says why the claw ports are not a fourth).
- Station G's G.2.1 ("nine MOTORs") and G.7's print order — fixed (above). G.4's "9 of 9 tonight"
  stays true: with the filter the claws are not candidates, so the denominator is the drive motors.

#### 03:49–04:06 — gates, a formatting pass, the battery again on the final code
- Full host build with the doc gates (the register changed, so they re-ran) after
  `briefing_status.py generate`: self-test OK; coverage 2592 entities / 148 headers; check-fresh
  exit 0; 408 example lines verbatim; removability clean; briefing current; staleness audit
  self-test OK and clean. Suite **1436 / 1436 passed (3 skipped), 2,387,081 assertions**.
- `tools/src_build_gate.py self-test`: **OK (33 detector cases, each against a real build)**.
  `check`: **PASS, all 6 builds link** (bench, xdrive, tank, tank-drive, tank-library,
  tank-calibrate: 4/4 TUs each, 0 disallowed warnings, hash and beacon asserted) — so the new
  header, with its static_asserts, compiled in every variant, robot two's pairing in the four tank
  builds. The tank-calibrate ELF carries `port %d: claw pivot (declared) -- not a candidate, not
  touched` and the new refusal words (`strings bin/hot.package.elf`).
- ARM header gate (every `include/shulib` header, one TU, the CI flags): CLEAN, 184 headers. CI
  greps: library PROS-free outside `hal/pros/`; core sim-free. `src/mechanism_table.hpp` includes
  no `pros/` header; no `SHULIB_RUNS_*` outside `src/main.cpp`. Every string literal in the two
  changed `src/` files is ASCII (checked by a script, not by eye).
- A formatting pass to the sibling's width (chassis_table.hpp has no line over 100 characters):
  three header lines, one calibrate comment line, four test lines re-wrapped; the split's bound
  hoisted into a `constexpr cap`. No behaviour change — and so the WHOLE battery was run again on
  the final code rather than assumed: **all fifteen reproduce exactly** (same REDs, same compile
  errors, M9 red only under the tank define), then restored, rebuilt, 1436 green, tank syntax
  clean. The src gate `check` was run again AFTER the pass: PASS, all 6 builds.

#### For the coordinator at merge (not done here, by scope)
1. **Convert** `bench::MechanismEntry` into the library's `config::MechanismConfig` (cartridge and
   safe mode from the entry; `*entry.forward` / `*entry.reverse` / `volts` into the bindings). The
   optionals are refused at compile time when unset, so the dereferences are safe for any table that
   compiles. *(E2 renamed it `bench::MotorMechanismEntry`, read from `kMechanisms.motors[0 ..
   motorCount)`; the pneumatic claw converts separately — E2's coordinator list, item E2-2.)*
2. **Tie the re-zero button:** G1 Part 2b's `src/teleop_program.cpp` names its own `kRezeroButton =
   hal::ControllerButton::A` (read-only look at the main tree, :141, with a static_assert against
   L1/L2/R1/R2). Make one refer to the other — e.g. `static_assert(kRezeroButton ==
   bench::kTeleopRezeroButton)` — so the two cannot drift. *(SUPERSEDED in part by E2: the team
   lead's ruling moved the re-zero to **Y** — `bench::kTeleopRezeroButton` is Y now, and Teleop's
   constant moves A -> Y at the same merge; E2's coordinator list, item E2-1.)*
3. **Station J's `<<FROM TELEOP AT MERGE>>` markers** (J.3.1, J.3.2, J.3.3, J.4.6, and — added at
   review — J.7.1): fill each from Teleop's code once its claw boot lines, banner sentence and
   panel row exist. J.7.1's marker also asks whether Teleop prints a serial line when the
   controller drops; if it does not, strike "and the serial line" from J.7.1.
4. **The include order** in `src/calibrate_program.cpp`: `mechanism_table.hpp` sits BEFORE
   `chassis_table.hpp` on purpose (a clean merge against 2b's `screen_text.hpp` line); reorder
   after the merge if wanted.
5. **Hardware-assumptions status line** ("7 of 145" here) and the briefing's "next free HA-157" are
   this tree's numbers only — set the merged total once HA-142..152 land.
6. **`docs/internal/HANDOFF-remaining-work.md:111`** ("Calibrate takes every motor the brain's
   registry reports as a candidate") is now true except for declared mechanism ports — out of this
   worktree's scope; worth a clause at merge.
7. **Teleop's boot check** ("NEW motors not in the calibration: ...") should name 6 and 7 as claw
   pivots, not unknown motors (brief item 6, second half) — it lives in `src/boot_source.*`, which
   is out of this worktree's scope.
8. Bench Tests (slot 3) is unchanged: its census will list 6 and 7 as plain MOTORs, and its DRIVE
   station never touches them (it drives only the chassis table's ports).
9. *(Added at review.)* **`docs/hardware-assumptions.md`'s index needs a HAND merge** at the HA-141
   row: G1 Part 2b's in-progress tree rewrites that row and this tree adds HA-153..156 on the lines
   right after it (`git merge-file` against the main tree at 04:20, 04:25 and 04:42: ONE conflict, exactly there;
   HA-142..152 from the other chunks will land in the same spot). *(E2 review, finding 3: against
   the main tree's `a8540e6`, with DEFECTS2 landed, it is THREE conflict hunks — see E2-5.)* The status paragraph's per-chunk
   sentences and its "7 of 145" total are re-checked at the same time. **`PROJECT-BRIEFING.md`**
   also conflicts; it is generated — regenerate it after the merge, never hand-merge it.
10. *(Added at review.)* `src/calibrate_program.cpp` now changes in THREE places besides the
    include and the header paragraph: the census loop (it fills `ports[]` only), the filter block,
    and step 4's first lines (it builds from `ports[i]`). All three merge cleanly with 2b's hunks
    as of 04:42; `test/mechanism_table_test.cpp`'s last case pins their exact text, so the suite
    is the check that the merge kept them whole.

**Nothing here has run on a robot.** Calibrate's claw lines are the only claw text that is final;
everything about the claws' behaviour under Teleop waits for the library half and Station J.

---

### Review findings and their disposition (04:17–04:43)

A reviewer read the change and reported one major and seven minor findings. Each was re-checked
against the tree (not taken on trust), then fixed at the root or answered with evidence. **All
eight were real; all eight are fixed.**

**1 (major) — the call-site test was anchored on the census HEADER. REAL, fixed at the root.**
Re-checked: `emit("STAGE 0  DEVICE CENSUS ...")` prints BEFORE the loop that fills the list, so
the block cut and pasted after the `PORT  device` line (ahead of the loop) still compiled, kept
every `find()` in order, filtered an EMPTY list (n = 0) — and the loop then added 6 and 7 back as
candidates, to be geared BLUE, set to Coast and possibly pulsed, with 1436/1436 green. Fixed in
three parts, because moving the anchor alone would still have left logic untested at the site
(finding 2):
- **The logic went into the table, where the suite RUNS it**: `splitDriveCandidates` /
  `CandidateSplit` are replaced by `[[nodiscard]] constexpr std::size_t
  dropDeclaredMechanismPorts(const MechanismTable&, std::span<std::int8_t> ports,
  DroppedMechanismPorts& dropped)` — it compacts the census's own port list IN PLACE (survivors
  to the front, census order), sets every slot past them to 0 (never a port, so a caller that
  kept the old length reads no stale duplicate), resets `dropped` first, and returns the kept
  count. `[[nodiscard]]`: a thrown-away count is the bug of finding 2(b).
- **Calibrate keeps ONE list.** The census loop now fills `ports[]` only (it used to fill
  `candidates[n].port` too), and step 4 builds each adapter from `ports[i]` (`const int port =
  ports[i];`, also used by its WARNING line). Before, the filter had to re-sync two copies — the
  loop finding 2(a) broke. Now there is no second copy to go stale: `candidates[].port` is
  written only by step 4, and nothing reads it before then (grep: its only other uses were the
  census write and the re-sync, both gone). What is left at the call site is one assignment,
  `n = dropDeclaredMechanismPorts(kMechanisms, {ports, n}, dropped);`, and one log loop over
  `dropped`.
- **The pin is exact, comments stripped.** The last test case reads the program through
  `codeOnly()` (every comment removed, literals kept whole, whitespace squeezed; its own case
  pins those four behaviours) and requires, CONTIGUOUSLY and exactly: the census loop's motor
  branch (filling `ports[]` only), the loop's end, the census's CLOSING count line
  (`"  %u motor(s), %d IMU(s)"` — the reviewer's anchor), then the filter block. Also exactly:
  step 4's first lines (building from `ports[i]`), after the block, and the file's FIRST
  `.motor.emplace(`; nothing between the block and step 4 writes `n` or `ports[` (nor takes
  `&n` or a span); exactly one call; `candidates[i].port` / `candidates[n].port` appear nowhere.
  Any edit to those lines turns the case red — on purpose; its comment says whoever edits them
  updates the case in the same change.

**2 (minor) — the re-sync and log loops were untested logic. REAL, fixed** (the design above).
The re-sync loop no longer exists; the list rewrite is `dropDeclaredMechanismPorts`, tested on
the WHOLE array after the call (not only the kept prefix), the returned count, the dropped list
(the test pre-fills it with garbage: 99s and count 3), a claw in the middle and at the END, the
empty census, a full brain, and a 25-entry flood of port 6 (the dropped list caps at 21 and
every copy still leaves the candidates). The reviewer's three survivors are now RED: (a) the
compaction miswritten (F2a), (b) the count assignment commented out (F2e) or discarded (F2f),
(c) the log loop walking the kept list (F2g). The first round's claim that the test saw the
result "ignored" is marked WRONG where it was written (above).

**3 (minor) — the 1..21 guard's boundary mutations survived. REAL, fixed.** New case: entries on
+21 and -1 (flipped, physical port 1; the table passes every check): `isDeclaredMechanismPort` is
true for 21 and 1, false for 0, 2, 6, 7, 20, 22; the filter drops `{1, 21}` from
`{1, 2, 11, 20, 21}`; an entry at 22, -22 or 0 never makes 22 or 0 a port. `port >= kMaxPort`
(F3a) and `port < 2` (F3b) are RED.

**4 (minor) — "predate all eighteen" went stale. REAL, fixed.** Reworded to "predate every entry
from HA-124 on", which stays true however many entries land after (the coordinator still
re-checks the paragraph's total at merge, item 9).

**5 (minor) — the HA index conflicts with 2b; the merge-hygiene claim was incomplete. REAL.**
Verified, not assumed: `git merge-file` of base `b622d9b`, the main tree's in-progress copies and
this worktree's, run at 04:20, after this round's code edits (04:25) and at the end (04:42):
`calibrate_program.cpp` and the worksheet merge CLEAN (and the MERGED `calibrate_program.cpp`
still holds both pinned texts, checked with the same normalisation); `hardware-assumptions.md` has ONE conflict, at the HA-141 index
row (2b rewrites it; HA-153..156 follow it); `PROJECT-BRIEFING.md` conflicts (generated). The
main tree's `calibrate_program.cpp` also gained a hunk at base :640 (mtime 03:50, after the first
look). The conflict cannot be avoided from this side: a Markdown table row cannot be padded with
a blank line, and HA-142..152 will land in the same spot anyway. Recorded for the coordinator
(items 9 and 10); the first look's "clear of all of it" is marked INCOMPLETE where it was written.

**6 (minor) — J.8 stated the 55 °C cut as fact. REAL, fixed.** Now: "the motor is believed to
start derating at 55 °C — A4 register HA-44, whose onset nobody has measured; 50 °C leaves room
for that belief to be wrong". (HA-154's own Settle already said "HA-44's 55 °C first step".)

**7 (minor) — HA-156's Source described the library binding as present. REAL, fixed.** Now: "The
T3 brief specifies that the library's hold-to-run binding applies the safe state ONCE on release
...; that binding is written in T3's library half and merged separately — it is not in the tree
this entry was written in."

**8 (minor) — J.7.1 could not tell "stopped because disconnected" from "stopped at its stop".
REAL, fixed.** J.7 now has an intro saying why, and J.7.1 is passed on what Teleop COMMANDS: the
claw row going FWD → HOLD at the disconnect while the driver still holds R1 and L1
(`<<FROM TELEOP AT MERGE>>`, with the serial line if Teleop prints one — coordinator item 3
amended). The link is cut by pulling a CABLE, at a moment the team chooses: the tether (as for
pairing, Station 5.5), or the radio's cable from brain port 1 (the radio's port per the worksheet's
own census record, :327 and :700); the USB serial capture is on its own cable. The claws start at
mid travel on a spoken count; a claw that reaches its stop first still counts (the row reads FWD
until the link goes), but if the pull has not happened within about three seconds of a stop, the
driver lets go and they restart (HA-154's heating). A row still reading FWD, or a claw still
moving or pushing after the pull, is a FAIL: the second person pulls the battery. J.7.2 and "What
to send back" (item 5) follow the change.

#### The battery, again, on the final code — every mutation on a FRESH binary (04:27–04:36)

Runner: `t3prep_mutate_r1.py` (scratch, outside the repo), the first battery's method: exact
replacements (each old string must occur once) or a cut-and-paste, `touch test/*.cpp`, rebuild
`shulib_tests/fast` requiring `Linking CXX executable shulib_tests` (seen for every mutation that
compiled), run `mechanism table*`; the table's tank-relevant ones also syntax-checked under
`-DSHULIB_ROBOT_TANK_2026` with host g++ and arm-none-eabi-g++; restored byte-exact (sha256 of
the three files checked after the run: OK). **All 28 RED:**

| # | Mutation | Observed |
|---|---|---|
| M1 | swap R1/R2 on port 6 (the brief's) | RED: the contents case + the buttons case |
| M2 | port 7 on L1/R2 (the brief's) | host build FAILS: `static assertion failed: robot two: one button is bound to two mechanisms`; tank host + ARM the same |
| M3 | drop the filter block (the brief's) | RED: the call-site case |
| M4 | a claw port on 11 (the brief's) | RED, 5 cases; tank host + ARM: `a mechanism port is a DRIVE port (or the IMU port) of this build's chassis table` |
| F1 | the block moved ABOVE the census loop (finding 1) | RED: the call-site case |
| F1b | the block moved after step 4's adapters | RED: the call-site case |
| F2a | the helper compacts to `ports[i]`, not `ports[kept]` | RED: the filter case + the edges case |
| F2b | the helper leaves the stale tail | RED: the filter case + the edges case |
| F2c | the helper does not reset `dropped` | RED, 3 cases (the garbage pre-fill shows) |
| F2d | the helper returns the old length | RED: the filter case + the edges case |
| F2e | Calibrate's `n = ...` commented out (finding 2b) | RED: the call-site case |
| F2f | Calibrate discards the count | RED: the call-site case |
| F2g | the log loop walks the kept list (finding 2c) | RED: the call-site case |
| F3a | guard `port >= kMaxPort` (finding 3) | RED: the edges case |
| F3b | guard `port < 2` (finding 3) | RED: the edges case |
| S1 | two copies again: the census fills `candidates[n].port`, step 4 reads it | RED: the call-site case |
| S2 | `ports[0] = 6;` inserted between the filter and step 4 | RED: the call-site case (the "between" check) |
| S3 | the filter fed `kNoMechanisms` | RED: the call-site case |
| M5 | predicate on the SIGNED entry | RED: predicate + edges |
| M6 | predicate loses its 1..21 guard | RED: predicate + edges |
| M7 | volts check `v <= 0 \|\| v > 12` | RED: the volts case (NaN) |
| M8 | the filter keeps declared ports | RED: filter + edges (18 assertions) |
| M9 | the selector's macro misspelt | host GREEN (expected: no define on the host); tank host + ARM: `ROBOT=tank must select robot two's mechanism table` |
| M10 | disjointness forgets the IMU port | RED: the disjointness case |
| M11 | the reserved-button check ignores `reverse` | RED: buttons + the full-table case |
| M12 | uniqueness forgets forward-vs-reverse | RED: the buttons case |
| M13 | completeness forgets the safe mode | RED: the completeness case |
| M14 | `mechanismCount` not clamped | RED: the count case |

Restored: rebuild linked, **1438 / 1438 passed (3 skipped), 2,387,130 assertions**; tank syntax
clean on both compilers.

#### Gates after the fixes (04:36–04:41, from the run files' mtimes)
- `briefing_status.py generate`, then the full host build with the doc gates: api_doc_tool
  self-test OK; coverage 2592 entities / 148 headers; check-fresh exit 0; 408 example lines
  verbatim; removability clean; briefing current; staleness audit self-test OK and clean. Suite
  **1438 / 1438 passed (3 skipped), 2,387,130 assertions** (+2 cases, +49 assertions over the
  first round: the edges case and the source-reader case; 13 `mechanism table` cases, 259
  assertions).
- `tools/src_build_gate.py self-test`: **OK (33 detector cases, each against a real build)**.
  `check`: **PASS, all 6 builds link**, each build's line identical to the first round's (hash
  aside) — no new warning from `[[nodiscard]]`, the span or the new locals. The tank-calibrate
  ELF still carries the log literal.
- ARM header gate (the CI command, 184 headers): CLEAN. CI greps: the library PROS-free outside
  `hal/pros/`; the core sim-free. `src/mechanism_table.hpp` has no `pros/` include; no
  `SHULIB_RUNS_*` outside `src/main.cpp`; every string literal in the two changed `src/` files is
  ASCII (a script: 23 + 238 literals).

#### Prose review for this round (grep first, then read the hits)
- `grep -rn "splitDriveCandidates|CandidateSplit|dropDeclaredMechanismPorts|DroppedMechanismPorts"`
  over the tree: the header, the test, this log (the old names only in entries marked REPLACED).
- `grep -rn "candidates\[i\]|candidates\[n\]|candidate list" docs README.md`: `docs/faq.md:197` and
  `docs/api/device_registry.md:319` ("builds its candidate list from the ports that carry a
  MOTOR") — still true, the declared mechanisms are a subset taken out; `docs/api/` is generated
  and out of scope anyway. `R3d-PROGRESS.md:880` — history.
- `grep -n "55 °C"` in the worksheet and the register: J.8 fixed; HA-154's Settle already
  attributed; `hardware-assumptions.md:1794` is another entry's own story, not a claim here.
- `grep -n "J.7"` over this change's files: J.7.1, J.7.2 and "What to send back" updated
  together; this log's first-round summary of Station J ("J.7 the controller off") is history.

**Nothing here has run on a robot.**

---

### Addendum E2: the pneumatic claw

- **Brief:** `docs/internal/chunks/T3-addendum-pneumatic-claw.md` (committed in the main repo at
  `9325490`; read from there, in full), its §3 "E2 — the program half" ONLY; the T3 brief re-read
  for context. B2 (the library half: the toggle binding, the provider's digital out, pneumatic
  mechanisms from config, HA-157/158) is another worktree's.
- **Started:** Sat Sep 26 04:47 AM EDT 2026, on this worktree's uncommitted phase-2 prep (above).
- **Executor model:** opus. Same rules: no commit, no push, no `pros`, nothing on `/dev/ttyACM*`;
  edit only this worktree; "host test plant", never the other word; ASCII only in runtime strings.

Appended in real time. **Nothing in this section has run on a robot.**

#### 04:47 — start
- Log section written before any code. Read: the addendum (all of it), the T3 brief, this log,
  `src/mechanism_table.hpp`, `test/mechanism_table_test.cpp`, Station J, HA-153..156;
  `hal/mechanism.hpp`'s `PneumaticMechanism` (`set`, `applySafeState()` = `set(safe)`),
  `hal/pros/digital_out.hpp` (CONSTRUCTING a `ProsDigitalOut` drives the line to its REQUIRED
  `initialState`; lines 1-8 = 'A'-'H'), `config/robot_config.hpp` (`pneumaticPorts[2]`, ADI 1..8),
  `hal/controller.hpp` (`ControllerButton`: L1 L2 R1 R2 Up Down Left Right X B Y A).
- Read-only look at the main tree: `src/teleop_program.cpp:141` (committed, `7f9b9a1`) is
  `constexpr hal::ControllerButton kRezeroButton = hal::ControllerButton::A;`, with a
  static_assert against the four shoulder buttons at :146-149. `grep -rn "DigitalOut|adi::"
  src/` in this worktree: nothing — no program here builds a 3-wire output.
- Baseline (this worktree, the phase-2 prep as it stood): `cmake --build build/test -j8`, suite
  **1438 / 1438 passed (3 skipped), 2,387,130 assertions**.

#### 04:49–04:52 — the table: the pneumatic claw, two entry types, the re-zero on Y
`src/mechanism_table.hpp`. Design decisions, each with its alternative:
- **Two entry types in two arrays, not one tagged struct.** `MotorMechanismEntry` (renamed from
  `MechanismEntry`) and the new `PneumaticMechanismEntry`; `MechanismTable` is now `motors[6]` /
  `motorCount` + `pneumatics[8]` / `pneumaticCount` (renamed from `entries` / `count`), mirroring
  `hal::MotorMechanism` / `hal::PneumaticMechanism` and `config::MechanismConfig`'s `motorCount` /
  `pneumaticCount`. Alternative: one entry with a kind tag — rejected: every check would branch
  on the tag, and every entry would carry the other kind's fields, silently ignored or silently
  read. With two types a pneumatic entry has no `.signedPort` at all, and the confusion that
  would matter most — 3-wire line 1 ('A') read as smart port 1 by Calibrate's drop — is
  impossible: `isDeclaredMechanismPort` reads the motor array only (pinned for every line 1..8
  against every port 1..21). The rename of the motor type and fields is why the old names appear
  only in this log's history above; `mechanismCount` / `mechanismEntryComplete` became
  `motorEntryCount` / `motorEntryComplete` beside the new `pneumaticEntryCount` /
  `pneumaticEntryComplete`. `src/calibrate_program.cpp` uses none of the renamed names (checked),
  so its pinned text is unchanged.
- **The claw's fields, each `std::optional` where a missing field would default silently:** `name`
  ("claw"), `adiLine` (1 = 'A'; 0 = UNSET), `openCommand` (`optional<bool>` — a value-initialised
  bool is `false`, an invented polarity), `start` and `safe` (`optional<PneumaticPosition>` —
  `Closed` is the enum's first enumerator, plausible and still invented), `toggle`
  (`optional<ControllerButton>` — L1 otherwise), `provenance` ("team lead 2026-09-26 (port A;
  polarity unknown): ..."; it says the safe = CLOSED is the addendum's statement, not the team
  lead's words).
- **The polarity is ONE constant, the positions are POSITIONS.** `kClawOpenCommand = true`
  (UNVERIFIED — Station J confirms; flip here only) is read by the entry (`.openCommand =
  kClawOpenCommand`); `start` and `safe` are `PneumaticPosition::Closed`, never levels; the one
  place a position becomes a level is `pneumaticCommandFor(entry, position)` (with
  `pneumaticStartCommand` / `pneumaticSafeCommand`), so flipping the constant moves the start
  command and the safe command together and nothing else is edited. Alternative: store the start
  and safe LEVELS (`.startCommand = !kClawOpenCommand`) — rejected: a later edit typing `false`
  there would silently detach it from the constant, and the team lead's facts are positions
  ("closed at start"), not levels.
- **Safe = CLOSED, and what "safe" means here, stated in the field's comment:** the position a
  LIBRARY-forced safe state commands (`hal::PneumaticMechanism`'s `safe`, COMP's end-of-run
  guard later) — NOT what a disable does (the toggle commands nothing on a disable or a
  disconnect, addendum §0).
- **No check that start == safe.** `hal/pros/digital_out.hpp` asks that a line's boot level agree
  with its mechanism's safe state; robot two's do (both CLOSED, pinned by the contents case). A
  compile-time equality check was considered and rejected: the addendum gives the library half
  (B2) SEPARATE start and safe values, so equality is B2's rule to make or not, not this table's.
- **ONE line, the double solenoid said, not built** (header "ONE LINE"): a second line needs its
  letter read off the brain AND its own level per position — `hal::PneumaticMechanism::set` drives
  EVERY line with the SAME level, so a second line driven the opposite way cannot be added to the
  same mechanism; the design is made when Station J.9.0's look says TWO.
- **`kTeleopRezeroButton` = Y** (the team lead's ruling 2026-09-26), its comment REPLACED: A is the
  claw's; Teleop's own `kRezeroButton` (main tree, committed `7f9b9a1`, `src/teleop_program.cpp`
  :141, still A) is moved A -> Y by the coordinator at merge and static_assert-tied to this
  constant then; until then the two DISAGREE, and the claw must not be wired into a Teleop whose
  re-zero is still A. (Teleop's existing "never a shoulder button" static_assert stays true for Y.)
- **The checks.** Check 1 covers both counts; check 2 both kinds (`pneumaticEntryComplete`: every
  field above); check 5 (uniqueness) now compares motor vs motor, the claw's toggle vs every
  pivot button, and toggle vs toggle; check 6 (reserved) reads the toggles too; NEW check 9
  `pneumaticLinesValid` (1..8, none twice); check 8 (chassis disjointness) stays motors-only, with
  the reason (the chassis table names no 3-wire port). The pivots' checks are unchanged. The
  static_asserts: all of checks 1-7 and 9 on robot two's table in every build; the build's own
  table the same plus check 8; the tank selection now also asserts `pneumaticCount == 1`.
- `kMaxMechanisms` stays 6; its comment now gives the shared arithmetic (2 x motors + pneumatics
  <= 11 buttons); `kMaxPneumatics` = `kMaxAdiLine` = 8 (one per 3-wire port).
- Header syntax-checked alone (no define, tank, X-drive) with host g++ under the suite's warnings
  and arm-none-eabi-g++ under the gate's: all six clean; clang 18 (`-Wall -Wextra
  -Wmissing-field-initializers -Werror`, host and tank) clean.

#### 04:52–04:56 — the tests *(times from the files' mtimes; this section was drafted at 05:00)*
`test/mechanism_table_test.cpp`: the prep's cases renamed mechanically (`entries` -> `motors`,
`count` -> `motorCount`); the buttons case now pins **Y** as the re-zero (`kTeleopRezeroButton ==
Y`) and its reserved-button tables put a pivot on **Y** (forward and reverse — each still UNIQUE, so
only check 6 catches it); `allChecksPass` includes check 9. Five NEW cases:
1. **the claw entry exactly** — line 1, `openCommand == kClawOpenCommand`, start CLOSED, safe
   CLOSED, toggle A, name "claw", provenance non-empty and carrying "team lead 2026-09-26 (port A;
   polarity unknown)", the four levels against the constant, smart port 1 NOT a mechanism port.
   The constant's VALUE is deliberately not pinned (the case says why: "flip here only");
   *(REVERSED at review, finding 1: the value is now pinned, as the pivots' signs are.)*
2. **the polarity is one constant** — both polarities: each level follows it, the positions do
   not; each level helper reads its OWN position; and the table's code (comments stripped) holds
   exactly one `kClawOpenCommand` definition and exactly one `.openCommand = `, which is the
   constant (a literal cannot hide);
3. **pneumatic completeness** — each of the seven fields unset => refused; the four optionals
   left out come back unset; a SECOND entry's gap is refused; a garbage entry past the count is
   not judged;
4. **3-wire lines** — 0, 9, 10, 22, 255 refused; 1, 2, 7, 8 fine; two entries on one line
   refused; a second entry's range read; a line past the count not judged; and for EVERY line
   1..8 the table passes, is disjoint from robot two's chassis, and Calibrate's predicate is true
   for exactly 6 and 7 of the 21 ports;
5. **the buttons across pivots and claw** — the claw on R1, R2, L1, L2 (the addendum's mutation
   and its three siblings) => not unique; a pivot on A (forward, reverse) => not unique; two
   toggles on A => not unique, on A/B fine, the second on L2 => not unique; the claw (or a second
   toggle) on Y => refused by check 6; the old re-zero A now collides with the table; an unset
   toggle collides with nothing (completeness refuses it); a toggle past the count not judged.
The count case also refuses `pneumaticCount` past its array (and clamps it) and pins the shared
arithmetic: five motor entries + the claw on A pass; any second pneumatic on any of the twelve
buttons fails.
- Suite after: **1443 / 1443 passed (3 skipped), 2,387,443 assertions** (+5 cases, +313
  assertions; 18 `mechanism table` cases, 572 assertions).

#### 04:57–05:10 — documentation, while the battery ran
*(The battery touches only the header, Calibrate and the test; the docs were safe to edit.)*
- **Worksheet Station J** (`R3a-BENCH-WORKSHEET.md`): the title now names both kinds ("THE CLAWS:
  the two pivots and the pneumatic claw"); the intro gains the pneumatic claw as the team lead
  stated it, the re-zero on **Y**, which programs drive 3-wire port A (only slot 2; no census lists
  it — Bench Tests' census reads the 21 smart ports, checked in `src/bench_r3a.cpp` STAGE 1 — and
  nothing in shulib drives the line in slots 1, 3, 4), and "nothing has ever driven 3-wire port A".
  NEW **J.9 — the PNEUMATIC claw on A**, the addendum's §3 item 3 step for step: J.9.0 air, no hand
  in the claw, and **the look — one cable or two from the valve into the brain, which letters** (TWO
  => run only J.9.1-J.9.2 and send it); J.9.1 the claw's boot line (`<<FROM TELEOP AT MERGE>>`);
  J.9.2 **at the start the claw CLOSES** — before/after/when, and "OPENED AT THE START" =>
  `kClawOpenCommand` is backwards, the coordinator flips that ONE constant (never the button or the
  start position); J.9.3 A opens, A again closes, nothing else moves, the panel's claw word
  (`<<FROM TELEOP AT MERGE>>`), Y leaves the claw alone; J.9.4 holding A does nothing more (one flip at the
  press, none while held, none at release); J.9.5 the controller unplugged while OPEN => it stays
  open, presses while unplugged do nothing, nothing moves at the reconnect, one press closes; J.9.6
  the disable (field control / Timed Run, when available, else "not available") => it keeps its
  state, open and closed, with the brain's own 3-wire level while disabled named as UNMEASURED.
  "What to send back" gains item 7 (J.9). Teleop's claw quotes are `<<FROM TELEOP AT MERGE>>`
  markers, as the addendum says. *(AMENDED at review: J.9.0 now stops on one cable not in A and
  the polarity verdict moved to J.9.3's first press — finding 2; J.9.6 no longer points at the
  controller-menu launch — finding 4.)*
- **`docs/hardware-assumptions.md`: HA-159 only** — an index row after HA-156; the entry, with a
  lead paragraph, after HA-156 in T3's subsection (retitled "robot two's claw pivots and its
  pneumatic claw"): the claim (ONE solenoid line on 3-wire A, `kClawOpenCommand`'s level opens
  it), the source (tagged `A4 register HA-159` in `src/mechanism_table.hpp`; the test pins the
  typing and NOT the polarity's value — *REVERSED at review, finding 1*), confidence
  **reported** (the port) with single/double and
  polarity **unknown**, settled by J.9.0 / J.9.2 / J.9.3, blast radius (a wrong polarity opens the
  claw at the start — the output drives its line when it is constructed, HA-119 — and would make a
  forced safe state OPEN it; a wrong port or a second cable is invisible to software). Status
  line: **7 of 146** in this tree; its parenthesis now says the addendum's library half owns
  HA-157 and HA-158.
- Written at first and corrected before this entry: HA-159 and J.9 said "the brain's registry
  reports smart ports only" / "cannot see a 3-wire device" — a hardware claim, and the library
  half's to register. Both now say what the CODE does: shulib's census reads the 21 smart ports,
  and nothing in shulib can ask the brain what is on a 3-wire line. J.9.6 first cited Station A
  as recording "a Timed Run"; Station A records a program launched from the controller's own menu
  running as a pretend match — reworded to exactly that. *(Still WRONG: that launch restarts the
  program and is forbidden, so it can never be the disable J.9.6 needs — review finding 4.)*
- This log's prep items 1 and 2 ("For the coordinator at merge") were stale after the rename and
  the Y ruling: each now carries an *(E2: ...)* note pointing at E2's list below, rather than being
  rewritten.

#### 04:57–05:24 — mutations, each on a FRESH binary
Runner `t3e2_mutate.py` (scratch, outside the repo), the prep's method: exact replacements (each
old string must occur once), `touch test/*.cpp`, rebuild `shulib_tests/fast` requiring `Linking
CXX executable shulib_tests` (seen for EVERY mutation that compiled: 53 of 53 in the first run,
27 of 27 in the second), run `mechanism table*`; table mutations also syntax-checked under
`-DSHULIB_ROBOT_TANK_2026` with host g++ and arm-none-eabi-g++; restored byte-exact (sha256 of
the header, Calibrate and the test equal before and after each run: True). **Run 1 (04:57–05:15):
the E-series and the prep's whole battery, 59 mutations.** **Run 2 (05:16–05:24): the E-series
again plus M9, on the FINAL code** (after a comment-only change to the header, below), with E22
replaced by E22b.

| # | Mutation (the addendum's four first) | Observed (identical in both runs) |
|---|---|---|
| E1 | **the claw on R1** | host build FAILS: `static assertion failed: robot two: one button is bound to two mechanisms (pivots and claw together)`; tank host + ARM the same |
| E2 | **a pivot on Y** (port 6's forward) | host build FAILS: `robot two: a mechanism takes shulib Teleop's re-zero button (kTeleopRezeroButton)`; tank host + ARM the same |
| E3 | **the claw starts OPEN** | RED, 2 cases (the claw's contents; the polarity case) — no static_assert can call it wrong, a test must |
| E4 | **a claw line 9** | host build FAILS: `robot two: a pneumatic line is outside 1..8 (3-wire A..H) or declared twice`; tank host + ARM the same |
| E5 | the re-zero back to A | host build FAILS (the claw's A is the reserved button); tank host + ARM the same |
| E6 | the claw's safe position OPEN | RED, 2 cases |
| E7 | the claw toggled by B (valid, wrong) | RED, 4 cases |
| E8 | the claw on line 2 (valid, wrong) | RED, 2 cases |
| E9 | the name "Claw" | RED: the contents case |
| E10 | the polarity typed as a literal (`.openCommand = true`) | RED: the polarity case (its source pin) |
| E11 | the provenance loses "(port A; polarity unknown)" | RED: the contents case |
| E12 | the claw not declared (`pneumaticCount = 0`) | RED, 4 cases; tank host + ARM: `ROBOT=tank must select robot two's mechanism table (the claw pivots on 6 and 7, the pneumatic claw on 3-wire A)` |
| E13 | uniqueness skips toggle vs pivot | RED, 2 cases (the cross-kind buttons case; the count case's arithmetic) |
| E14 | uniqueness skips toggle vs toggle | RED, 2 cases |
| E15 | the reserved check skips the toggle | RED, 2 cases |
| E16 | lines: upper bound `> kMaxAdiLine + 1` | RED: the lines case |
| E17 | lines: lower bound dropped | RED: the lines case |
| E18 | lines: a repeated line passes | RED: the lines case |
| E19 | completeness never reads the pneumatic entries | RED, 2 cases |
| E20 | completeness forgets the polarity | RED: the pneumatic completeness case |
| E21 | completeness forgets the safe position | RED: the pneumatic completeness case |
| E21b | completeness forgets the toggle | RED, 2 cases |
| E21c | completeness forgets the line | RED: the pneumatic completeness case |
| E22 | the level ignores the polarity (`return position == Open;`) | **INVALID in run 1**: it did not compile (`unused parameter 'e'`, -Werror) — proves nothing, so it was re-written |
| E22b | the same, compiling (`static_cast<void>(e);` first) | RED: the polarity case (the `false` polarity) |
| E23 | the start level ignores the start position | RED: the polarity case |
| E24 | the safe level reads the start position | RED: the polarity case |
| E25 | Calibrate's predicate reads the 3-wire lines too | RED, 4 cases (the predicate sweep, the lines case's every-line sweep, the full-brain filter, the claw's contents) |
| E26 | the count check ignores `pneumaticCount` | RED: the count case |
| E27 | `pneumaticEntryCount` not clamped | RED: the count case |
| E28 | `kClawOpenCommand = false` | **GREEN — BY DESIGN**: the suite pins that every level FOLLOWS the constant and deliberately not its value ("flip here only"); the value is Station J.9.2's to settle. Tank host + ARM clean. *(REVERSED at review, finding 1: RED now — see the review's battery)* |

The prep's battery on the renamed code (run 1), every one as before: **M1** RED 2 cases; **M2**
(port 7 on L1/R2) host build FAILS on the uniqueness static_assert (message now "... (pivots and
claw together)"); **M3, F1, F1b, F2e, F2f, F2g, S1, S2, S3** RED (the call-site case); **M4** RED
6 cases + tank host/ARM `a mechanism port is a DRIVE port ...`; **F2a-F2d, F3a, F3b, M5, M6, M7,
M8, M10** RED as before; **M11** RED (now caught by the pivot-reverse-on-**Y** table — the
six-entry table's reverse is A, no longer reserved); **M12** (uniqueness forgets the other entry's
reverse-side pairings) RED; **M12b** (`motorEntryBinds` reads only the forward button) RED 3
cases; **M13, M14** RED; **M9** (the selector's macro misspelt) host GREEN as expected (no define
on the host), tank host + ARM FAIL on the selection static_assert (both runs).

Restored after each run: rebuild linked, **1443 / 1443 passed (3 skipped), 2,387,443
assertions**; tank syntax clean on both compilers.

#### 05:15–05:16 — one comment corrected in the header, then run 2
The robot-two doc comment said "the brain's registry cannot see a 3-wire device at all" — the same
hardware claim corrected in HA-159 and J.9 (above), and the library half's to register. Now:
"Nothing has seen it: shulib's census reads the 21 smart ports only, and no program in this tree
drives a 3-wire line." Comment-only; run 2 re-ran the E-series on the result anyway. Also in the
register's new lead paragraph: "no census can show it" -> "shulib's census cannot show it (it reads
the 21 smart ports)".

#### 05:26–05:32 — gates, on the final code *(times from the run files' mtimes)*
- `briefing_status.py generate`: T3 in the open chunks; suite 1,443 / 2,387,443; HA **146**
  registered, next free **HA-160** (this tree's numbers — B2 owns 157/158).
- Full host build (`touch test/*.cpp`, `cmake --build build/test -j8`) with the doc gates:
  api_doc_tool self-test OK; coverage 2592 entities / 148 headers; 408 example lines verbatim;
  briefing current; staleness audit self-test OK and clean. Suite **1443 / 1443 passed (3
  skipped), 2,387,443 assertions**.
- The CI documentation gates run one by one: `self-test`, `check-coverage`, `check-fresh`,
  `check-examples`, `check-removability` all exit 0; `briefing_status.py check` exit 0;
  `doc_staleness_audit.py self-test` OK, audit clean (exit 0).
- `tools/src_build_gate.py self-test`: **OK (33 detector cases, each against a real build)**.
  `check`: **PASS, all 6 builds link**; each build's line identical to the prep's final check
  (hash aside — diffed, not eyeballed): 4/4 TUs, the same allowed `-Wunused-function` counts, 0
  disallowed. So the pneumatic entry, the Y re-zero and the new static_asserts compile in every
  variant, robot two's table in all four tank builds.
- ARM header gate (the CI command, 184 headers): CLEAN. CI greps: library PROS-free outside
  `hal/pros/`; core sim-free. `src/mechanism_table.hpp` includes no `pros/` header; no
  `SHULIB_RUNS_*` outside `src/main.cpp`. Every string literal in the two changed `src/` files is
  ASCII (a script: 30 + 238 literals).

#### Prose review (grep first, then read the hits)
- `grep -rn "MechanismEntry|kTeleopRezeroButton|.entries|mechanismCount|mechanismEntryComplete"`
  over src/test/docs: the old names only in this log's history (and `RobotConfig::mechanismCount`
  in `test/robot_config_test.cpp` — a different struct's field, untouched).
- `grep -rn -i "re-zero|rezero"` over docs (this worktree): no statement of the re-zero BUTTON
  outside history logs. **In the MAIN tree** (read-only): `docs/roadmap.md:1690` says Teleop's
  "loop binds the re-zero to button A" — out of scope (roadmap), stale once the coordinator moves
  Teleop to Y (item E2-6). The main tree's worksheet quotes no re-zero button. *(INCOMPLETE —
  review finding 5: the main tree also names the re-zero as A at `build-order.md:1409`,
  `HANDOFF-remaining-work.md:396`, `include/shulib/teleop/teleop_loop.hpp:90` and its generated
  `docs/api/teleop_loop.md:34`; all four added to E2-6.)*
- `grep -rn "HA-159"`: the three header tags (`A4 register HA-159`), the test, the register's
  index row / status / entry, the worksheet — the Source field's "tagged in
  `src/mechanism_table.hpp`" holds (reconciliation, direction 2).
- `grep -rn "HA-157|HA-158"`: only as "owned by the addendum's library half" (register status
  line, the E2 lead paragraph) and this log.
- `grep -rn "of 145|of 146"`: the register's status line (146); "7 of 145" only in the prep's
  history items (its item 5 already says the coordinator sets the merged total).
- `grep -rn "Station J"` over docs/src/test: HA-153..156 still point at the right steps (J's
  lettering J.0-J.8 unchanged; J.9 is new).
- `grep -rn -i "pneumatic claw|3-wire port A|port A"` over docs: only this change's text.
- Station J's intro: "It POWERS TWO MORE MOTORS" stays true (the pneumatic claw is not a motor);
  G.2.1's "ELEVEN motors" stays true (the claw is not in the census).

#### For the coordinator at merge (E2 — not done here, by scope)
- **E2-1. The re-zero button, A -> Y, tied.** Main tree `src/teleop_program.cpp:141`
  (`kRezeroButton = hal::ControllerButton::A`, committed `7f9b9a1`): set it to Y (or FROM
  `bench::kTeleopRezeroButton`) and add `static_assert(kRezeroButton ==
  bench::kTeleopRezeroButton)`; replace its comment ("No ruling picks it: A ...") with the team
  lead's ruling of 2026-09-26. Its boot line names the button from the constant (`buttonWord`),
  so it will say Y by itself; its "never a shoulder button" static_assert stays true. **Until this
  lands, the claw must not be wired into Teleop** (A would re-zero AND toggle the claw).
  *(Added at review, finding 5a:)* in the same change, REWRITE the `kTeleopRezeroButton` doc
  comment in `src/mechanism_table.hpp` — its "NOT YET TIED TO TELEOP ... still
  `hal::ControllerButton::A` ... Until then the two DISAGREE" paragraph describes the tree before
  this merge and says so — to state that the two are tied and name the static_assert's place in
  `src/teleop_program.cpp`.
- **E2-2. Convert the claw.** `kMechanisms.pneumatics[0 .. pneumaticCount)` into B2's pneumatic
  `config::MechanismConfig` (name; `pneumaticPorts[0] = adiLine`, `pneumaticCount = 1`; B2's start
  value = `pneumaticStartCommand(e)` and safe value = `pneumaticSafeCommand(e)` — LEVELS, never
  the positions); the toggle binding on `*e.toggle`. The 3-wire output is driven when it is
  constructed (HA-119): whatever B2 constructs it with must be the START level, or the claw
  twitches at boot. The motor entries convert as the prep's item 1 says, from
  `kMechanisms.motors[0 .. motorCount)`.
- **E2-3. Station J's new markers:** J.9.1 (the claw's boot line) and J.9.3 (the panel's claw word)
  — `<<FROM TELEOP AT MERGE>>`, filled from Teleop's code with the prep's J.3 / J.4.6 / J.7.1.
- **E2-4. B2's HA numbers.** HA-159's Confidence refers to "the library half's register entries"
  (a 3-wire output's level while the brain is disabled; what the brain can report about a 3-wire
  line) WITHOUT numbers, and J.9.6 calls the brain's 3-wire level while disabled UNMEASURED
  without citing the entry that registers it: the addendum gives B2 HA-157 and HA-158, and which
  is which is B2's text. Cite them in both places at the merge. *(Corrected at the E2 review's
  second sitting: this item first said J.9.6 also names "the library half's register entries";
  it never did.)*
- **E2-5. The register's hand merge grows** (prep item 9): HA-159's index row sits right after
  HA-156's, and B2's HA-157/158 rows land between; the status line reads "7 of 146" (this tree
  only) and its parenthesis names B2's two; T3's subsection is retitled "... and its pneumatic
  claw". The briefing ("146 registered, next free HA-160") is regenerated after the merge.
  *(REVISED at review, finding 3 — re-measured, not carried over:)* against the main tree's
  `a8540e6` (DEFECTS2 landed: HA-142..146), a dry-run `git merge-file` of base `b622d9b`, this
  worktree's copy and `a8540e6`'s gives **THREE conflict hunks** in the register. *(Re-measured
  at the second sitting, 09:29, main tree still at `a8540e6` — its next commit, `ea58fbc`, touches
  none of the register, the worksheet or Calibrate: the same three — and NOT all three are keep-both; a plain
  keep-both of each would duplicate text:)* (1) the status paragraph's per-chunk sentences (this
  tree's T3 / E2 sentences vs DEFECTS2's "added FIVE ... HA-142..HA-146" and its note that the
  per-confidence counts "still sum to 122, and a recount is due") — a HAND merge of one sentence:
  both sides rewrote the same base line, so each side of the hunk opens with "earlier screens
  instead of measured." and closes with "**No *v2* robot exists**, and"; keep each once,
  DEFECTS2's sentences before T3's and E2's, and this tree's "The same night's other chunks own
  HA-142 to HA-152 ... the total above counts only the entries in this tree" rewritten (HA-142..146
  are then in; only HA-147..152 are elsewhere); (2) the index rows from HA-141's on — keep-both
  EXCEPT HA-141's row, which is on BOTH sides (`a8540e6` carries G1 Part 2b's rewrite of its last
  column; this tree never touched it): take `a8540e6`'s once, then HA-142..146, then HA-153..156
  and HA-159 (a plain keep-both lists HA-141 twice — the dry run's keep-both gives 152 index rows
  over 151 entries); (3) the body, after HA-141's entry (T3's subsection vs DEFECTS2's) —
  keep-both, no entry on both sides; DEFECTS2's subsection first, in number order. **And one line merges
  CLEANLY and WRONG:** both sides changed "7 of 141" to "7 of **146**" (this tree: +4 T3, +1
  HA-159; DEFECTS2: +5), so git takes 146 with no conflict — the merged register holds 141 + 5
  (DEFECTS2) + 4 (T3) + 1 (HA-159) = **151**, 153 with B2's HA-157/158, plus whatever lands from
  HA-147..152 (held by "a parallel chunk", per `DEFECTS2-PROGRESS.md:15` at `a8540e6`; no entry
  there yet). Set the total by hand, then do the
  per-confidence recount DEFECTS2 flagged. Calibrate and the worksheet merge cleanly against
  `a8540e6`, and the merged `calibrate_program.cpp` still holds the test's pinned text (checked
  with the test's own normalisation: the census + filter block, step 4's loop, one call, the
  include, no `candidates[i|n].port`); `PROJECT-BRIEFING.md` conflicts (generated: regenerate).
- **E2-6. Stale at merge, out of this worktree's scope:** main tree `docs/roadmap.md:1690` ("its
  loop binds the re-zero to button A") becomes Y with E2-1; HANDOFF's Q1 (the claws) gains the
  pneumatic claw. *(Added at review, finding 5b — every other main-tree statement that the
  re-zero is A, at `a8540e6`:)* `docs/internal/build-order.md:1409` ("re-zero button A");
  `docs/internal/HANDOFF-remaining-work.md:396` ("\"shulib Teleop\" names `A`");
  `include/shulib/teleop/teleop_loop.hpp:90` (`DriverLoopConfig`'s comment, the same words — the
  library's, out of this worktree) and its GENERATED `docs/api/teleop_loop.md:34` (regenerate,
  never hand-edit). All four say Y once E2-1 lands.
- **E2-7. The polarity is Station J.9's.** `kClawOpenCommand = true` is a guess. *(REVISED at
  review, finding 1 and 2:)* the suite now PINS its value (`CHECK(kClawOpenCommand == true)` in
  the claw's contents case, as the pivots' signs are pinned — E28 is RED). The verdict is J.9.3's
  first press read with J.9.2, and only with J.9.0's one cable in A: open after the start AND the
  first press CLOSES it => "OPENED AT THE START", backwards. Then flip that one constant (the only
  code edit — never the button, never a position) and, in the SAME change, the test's pin,
  HA-159 (its index row and its claim) and J.9's intro quote. A press that moves nothing is not a
  polarity answer. *(Completed at the second sitting — `grep` for every place that states the
  value:)* the same change also rewrites the header's two comments that call `true` a guess
  (the file header's "THE POLARITY" paragraph and `kClawOpenCommand`'s own doc comment) — after
  J.9 the value is measured, and "UNVERIFIED" must go with the guess — and the test comment above
  the pin ("while HA-159 and Station J.9 still say `true`"). If J.9 says RIGHT, nothing flips,
  but the same places lose "UNVERIFIED" / "guess" and HA-159 is settled as the register says.
- **E2-8.** *(Added at review, finding 4.)* The worksheet's DO NOT list gained "Do not start a
  program from the CONTROLLER's own menu" — G.0.5 already sent the tester there for that rule,
  and the list did not carry it (the same gap is in the main tree's copy at `a8540e6`; the list
  is unchanged there since `b622d9b`, so the addition merges cleanly).

**Nothing in this section has run on a robot.** The claw's entry, its checks and Station J.9 are
text and host tests; the polarity, the port and one-wire-or-two wait for Station J.9.

---

#### E2 review findings and their disposition (05:40–06:03, then 09:26–10:06; the builder again)

A reviewer read E2 and reported five minor findings. Each was re-checked against the tree and, for
3 and 5, against the main tree's `a8540e6` (read-only), not taken on trust. **All five were real;
four are fixed at the root in this worktree, and the fifth (3) is a merge fact, re-measured and
handed to the coordinator (E2-5).**

**1 — the claw's polarity was the one fact left unpinned; E28 passed. REAL, fixed.** Re-checked:
the pivots' signs are "flip here only" too (worksheet J.4: "the fix is ONE sign"), yet the
contents case pins `+6` / `+7` ("a flip is a measurement"); and the value `true` is already
written in two docs (HA-159's index row, J.9's intro), so under the register's reconciliation
rule a flip was never a one-place edit — only a one-place CODE edit. An unrecorded flip left those
docs contradicting the code with the suite green, and on the robot it opens the claw at program
start. The first round's reason (keep the flip a one-place edit) confused the two. Fixed:
`CHECK(kClawOpenCommand == true)` in the claw's contents case, with a comment saying the flip is
Station J.9's measurement and changes this pin, HA-159 (index row and claim) and J.9's quote in the
same change; the case's own "NOT PINNED, ON PURPOSE" paragraph removed; the header's two polarity
comments now say "the only CODE edit is here" and name the pin and the two quotes; HA-159's Source
says the test pins the value "as it pins the pivots' signs". The code edit is still one line —
the polarity case still proves every level follows the constant. E28 is RED now (battery below).

**2 — J.9.2 called a backwards polarity from one observation with other causes, and J.9.0 never
checked the port was A. REAL, fixed.** Re-checked: J.9.0 recorded the letter but stopped only on
TWO cables, and J.9.2's rule ("If it is OPEN after the start, `kClawOpenCommand` is BACKWARDS")
had no condition — a valve that is not driven (one cable in another port, no air, a loose cable)
and rests open would have been reported as "OPENED AT THE START", and a correct polarity flipped.
Fixed: J.9.0 now stops on ONE cable NOT in A ("send the letter; the table says A"), and says the
TWO-cable case gives no polarity answer. J.9.2 records where the claw is and says "do not call the
polarity yet". The verdict moved to **J.9.3's FIRST press**, read with J.9.2 and only with one
cable in A: closed after the start and the press OPENS it => right; open after the start and the
press CLOSES it => backwards ("OPENED AT THE START"; the coordinator flips the constant and
records it); the press moves NOTHING => not a polarity answer, stop and send. The J.9 intro,
"What to send back" item 7, HA-159's Settle and the header's comments follow the same rule.
(The polarity cannot be misread the other way either: a claw closed after the start whose first
press does nothing is also "not called".)

**3 — the register's merge now has three conflicts, not one. REAL (a merge fact, not a code
defect) — re-measured and recorded, E2-5.** A dry-run `git merge-file` (scratch copies; base
`b622d9b`, this worktree, `a8540e6`) at 05:45, and again after this round's doc edits: the
register has THREE conflict hunks (the status paragraph, the index rows after HA-141, the body
after HA-141's entry), all keep-both *(WRONG for two of the three — the second sitting's
re-measure, below: the status hunk is a hand merge of one sentence, and the index hunk carries
HA-141's row on both sides)*; the worksheet and `calibrate_program.cpp` merge cleanly; the
merged Calibrate still holds the test's pinned text (checked with a script that reproduces the
test's `codeOnly()` normalisation: the census + filter block, step 4's loop, one call, the
include, no `candidates[i|n].port`); the briefing conflicts (generated). **One thing the reviewer
did not report, found while re-measuring:** the status line merges CLEANLY and WRONG — both sides
changed "7 of 141" to "7 of 146" (this tree +5, DEFECTS2 +5), so git takes "146" with no conflict
while the merged register holds 151 (153 with B2's). E2-5 now says so, with the arithmetic, and
that DEFECTS2's per-confidence recount is still due. Prep item 9's "ONE conflict" carries a note.

**4 — J.9.6 pointed to a disable that restarts the program and that the worksheet forbids. REAL,
fixed.** Re-checked: the only example J.9.6 named was Station A's 2026-09-10 pretend match — a
launch from the controller's own menu, which (HA-128, measured) restarted the program and left it
DISABLED before driver control; A could never open the claw first, and the restart would command
CLOSED again (so "closed through the disable" would pass for the wrong reason). Fixed: J.9.6 now
says what counts — a disable that arrives AFTER slot 2 is in driver control, in the SAME run (a
field controller or competition switch disabling the running program; a Timed Run started from
the BRAIN's screen only if it ends in a disable of the same run — nobody here has seen what one
does at its end, so no claim is made); the controller-menu launch is named as NOT counting and
forbidden; and **the serial log decides** (Teleop's boot lines printed a second time = a restart =
the check does not count). **Found while fixing it:** G.0.5 sends the tester to "the DO NOT list"
for the controller-menu rule, and the DO NOT list did not carry it (the main tree's copy has the
same gap; that list is unchanged between `b622d9b` and `a8540e6`). The rule is added there, citing
Station A step 1 and HA-128 (E2-8), so J.9.6's and G.0.5's pointers are both true.

**5 — notes that go stale at merge; main-tree re-zero mentions missing from the coordinator list.
REAL, both parts, fixed within scope.** (a) The `kTeleopRezeroButton` comment's "NOT YET TIED ...
Until then the two DISAGREE" is true only before the merge: E2-1 now tells the coordinator to
rewrite it in the same change (naming the static_assert's place), and the comment itself now says
it describes the tree before that merge. (b) `git grep` of `a8540e6` for every statement of the
re-zero button: besides `roadmap.md:1690`, `build-order.md:1409` ("re-zero button A"),
`HANDOFF-remaining-work.md:396` ("\"shulib Teleop\" names `A`"), and one the reviewer did not
list — `include/shulib/teleop/teleop_loop.hpp:90` (`DriverLoopConfig`'s comment, the same words;
library code, out of this worktree) with its generated `docs/api/teleop_loop.md:34`. All four are
in E2-6; the E2 prose review's "only roadmap:1690" is marked INCOMPLETE where it was written. None
of these files is in this worktree's tree in that form (`b622d9b` predates Part 2b's text), so
none could be edited here.

##### The second sitting (09:26–10:06): the round finished, and every finding re-checked again

**Why a second sitting.** The first sitting's battery (`t3e2_review_mutate.py`, scratch) was cut
off at 06:03, in E28's ARM tank check: its run file ends there, with no restore line, and none of
that sitting's gates ran. So nothing above the heading "E2 review findings" was trusted as
finished. On resuming: the three code files carried the final code (`kClawOpenCommand = true`;
the battery's own exact-once check found every one of its 60 unmutated strings), a fresh full
build linked and the suite was **1443 / 1443 passed (3 skipped), 2,387,444 assertions** — one
more than E2's 2,387,443: the new pin.

**Each finding re-checked against the tree, not against the first sitting's account.**
- **1:** `CHECK(kClawOpenCommand == true)` is in the claw's contents case with the comment the
  reviewer asked for; the "NOT PINNED" paragraph is gone; the header's two polarity comments and
  HA-159's Source say the value is pinned and name the places that quote it. **E28 is RED**
  (battery below). *Considered and not done:* a test that reads HA-159's row and J.9's intro and
  compares their `true` with the constant — it would pin prose wording, and the register's own
  reconciliation is a grep rule, not a unit test. The pin plus E2-7's list is the fix.
- **2:** J.9.0 stops on one cable not in A; J.9.2 says "do not call the polarity yet"; the verdict
  is J.9.3's first press, only with one cable in A, and "did NOTHING" is "not called". The intro,
  "What to send back" item 7, HA-159's Settle and the constant's comment all say the same. Walked
  every case by hand (claw open or closed before the start; valve driven or not; polarity right
  or backwards): no combination ends in RIGHT or BACKWARDS unless the valve moved on the press.
- **3:** re-measured at 09:29 (main tree still `a8540e6`, its working copy clean): THREE register
  hunks, the worksheet and Calibrate clean, the briefing one conflict. **Two things the first
  sitting got wrong, now fixed in E2-5 and marked where they were written:** the hunks are NOT all
  keep-both — (1) the status hunk is a hand merge of one sentence (both sides rewrote the same base
  line, so both open with "earlier screens instead of measured." and close with "**No *v2* robot
  exists**, and"), and (2) the index hunk carries HA-141's row on BOTH sides (`a8540e6` has Part
  2b's rewrite of it; this tree never touched it), so a plain keep-both lists HA-141 twice (the
  dry run's keep-both: 152 index rows over 151 entries). The body hunk is a true keep-both. The
  clean-and-wrong "7 of 146" and the 151 total hold (counted: base 141 rows / 141 entries; each
  side 146 / 146; merged 151 entries). **The merged Calibrate is now checked with the REAL test,
  not a reproduction of it:** the dry-run merge of `calibrate_program.cpp` swapped into this
  worktree and the call-site case run on the built binary — **GREEN, 25 assertions**; the control,
  `a8540e6`'s own copy (no filter), **RED** at `REQUIRE(filter != std::string::npos)`, which shows
  the case read the swapped file; restored byte-exact (sha256), the case green again.
- **4:** J.9.6 names what counts (a disable of the SAME run, after driver control) and what does
  not (the controller-menu launch, forbidden), and the serial log decides; the DO NOT list carries
  the controller-menu rule, sourced to HA-128's own claim ("launching (Run / Timed Run / Match)
  from the controller's own program menu") and Station A step 1. The main tree's DO NOT list at
  `a8540e6` still lacks it and G.0.5 is unchanged there, so the gap and the clean merge both hold.
- **5:** (a) the `kTeleopRezeroButton` comment says it describes the tree before the merge, and
  E2-1 tells the coordinator to rewrite it (by name, not by line number); `a8540e6`'s
  `src/teleop_program.cpp:141` is still `kRezeroButton = hal::ControllerButton::A`. (b) `git grep`
  of `a8540e6` again: `roadmap.md:1690`, `build-order.md:1409`, `HANDOFF-remaining-work.md:396`,
  `include/shulib/teleop/teleop_loop.hpp:90`, `docs/api/teleop_loop.md:34` — exactly E2-6's list;
  none of the five exists at `b622d9b` or in this worktree (grepped), and all are out of scope.

**Found while re-checking (not in the review), fixed:**
- **E2-4** said J.9.6 names "the library half's register entries". It never did (checked against
  the worksheet as E2 first wrote it); it calls the brain's 3-wire level while disabled
  UNMEASURED with no register number. E2-4 now says so, and asks for B2's number in both places.
- **E2-7** listed the pin, HA-159 and J.9's intro as the places a flip must change. A grep for
  every statement of the value found three more: the file header's "THE POLARITY" paragraph and
  `kClawOpenCommand`'s own doc comment (both call `true` a guess, UNVERIFIED) and the test comment
  above the pin. E2-7 now lists them, and says what changes when J.9 says RIGHT (no flip; the
  "UNVERIFIED" / "guess" wording goes, HA-159 is settled).

**The battery on the final code, every mutation on a FRESH binary (09:33–09:52).** The first
sitting's runner, unchanged: exact replacements (each old string must occur once — all 60 did),
`touch test/*.cpp`, rebuild `shulib_tests/fast` requiring `Linking CXX executable shulib_tests`
(**seen for all 54 mutations that compiled**; the other 6 are compile failures, no binary), run
`mechanism table*`; table mutations with a tank meaning also syntax-checked under
`-DSHULIB_ROBOT_TANK_2026` with host g++ and arm-none-eabi-g++; each file restored byte-exact
(sha256 of the header, Calibrate and the test equal before and after the run: True). **Every
mutation RED, or refused at compile time, except the two expected:**

| # | Observed |
|---|---|
| E1 the claw on R1; E2 a pivot on Y; E4 a claw line 9; E5 the re-zero back to A; M2 port 7 on L1/R2 | host build FAILS on the static_assert (uniqueness; re-zero; lines; re-zero; uniqueness); tank host + ARM the same |
| E3 the claw starts OPEN | RED, 2 cases; tank host + ARM clean (no static_assert can call it wrong — a test must) |
| **E28 `kClawOpenCommand = false`** | **RED** — the claw's contents case (the pin, finding 1); tank host + ARM clean. *Was GREEN in E2's own battery.* |
| E6-E11, E13-E21c, E22b-E27 | RED, 1-4 cases each, as in E2's run 2 (E7 4, E12 4 + tank host/ARM FAIL on the selection, E25 4) |
| M1, M3, M4 (6 cases + tank host/ARM FAIL), F1, F1b, F2a-F2g, F3a, F3b, S1-S3, M5-M8, M10-M14, M12b | RED, as in E2's run 1 |
| E22 (the uncompilable form, kept for the record) | does not compile (`unused parameter`) — INVALID as before; E22b is its valid form, RED |
| M9 the selector's macro misspelt | host GREEN — expected, the host has no define; tank host + ARM FAIL on the selection static_assert |

Compared by a script, mutation by mutation (the outcome, the number of red cases, and each tank
compile): identical to E2's run 1 (59 in common) and run 2 (31 in common) **except E28 alone**,
GREEN then, RED 1 now.

Restored: rebuild linked, **1443 / 1443 passed (3 skipped), 2,387,444 assertions**; tank syntax
clean on both compilers.

**Gates on the final code (09:53–10:02).**
- `briefing_status.py generate` (the block is unchanged by this round: T3 open, suite 1,443 /
  2,387,444, 146 registered, next free HA-160 — this tree's numbers), then `touch test/*.cpp` and
  `cmake --build build/test -j8` with the doc gates: api_doc_tool self-test OK; coverage 2592
  entities / 148 headers; 408 example lines verbatim; removability clean; briefing current;
  staleness audit self-test OK and clean. Suite **1443 / 1443 passed (3 skipped), 2,387,444
  assertions**.
- The CI documentation gates one by one: `self-test`, `check-coverage`, `check-fresh`,
  `check-examples`, `check-removability`, `briefing_status.py check`, `doc_staleness_audit.py
  self-test` and the audit — every one exit 0.
- `tools/src_build_gate.py self-test`: **OK (33 detector cases, each against a real build)**.
  `check`: **PASS, all 6 builds link** — each build's line identical to E2's final check (diffed):
  4/4 TUs, the same allowed `-Wunused-function` counts, 0 disallowed. The last-built ELF
  (tank-calibrate) still carries `claw pivot (declared) -- not a candidate, not touched`.
- ARM header gate (the CI command, 184 headers): CLEAN. CI greps: the library PROS-free outside
  `hal/pros/`; the core sim-free. `src/mechanism_table.hpp` has no `pros/` include; `SHULIB_RUNS_`
  appears only in `src/main.cpp`; every string literal in the two changed `src/` files is ASCII
  (a script, comments stripped: 30 + 238 literals, 0 non-ASCII).

**Prose review for this round (grep first, then read the hits).**
- `grep -rn -i "keep-both"` over docs/src/test: only E2-5 and this section, and finding 3's first
  account, now marked WRONG where it was written.
- `grep` for every statement of the polarity's value (`` `true` ``, `= true`, "ON = open", HIGH
  near the claw) over the header, the test, the register and the worksheet: the seven places E2-7
  now lists (the pin, HA-159's row and claim, J.9's intro, the header's two comments, the test's
  comment), plus the constant itself — no other.
- `grep -rn "library half"` over the worksheet, the register and the header: HA-159's Confidence
  is the only "the library half's register entries" (E2-4 corrected to match).
- `grep -rn "OPENED AT THE START|J.9.0|J.9.2|J.9.6"`: J.9's intro, J.9.0, J.9.2, J.9.3, J.9.6,
  "What to send back" item 7, HA-159's Settle and the header agree on the one rule; this log's
  E2 history carries AMENDED / WRONG notes where it described the old rule.
- `grep -n "E28"`: E2's table row carries its REVERSED note; the battery above is the RED run.

**The main tree moved during this sitting** — `ea58fbc` (committed 09:34, after the 09:29 dry run:
the DIAG1 brief, COMP's binding re-check B1-B27, `build-order.md`, the briefing). Checked at 10:05
rather than assumed: `git diff --stat a8540e6 ea58fbc` touches none of the register, the worksheet,
`calibrate_program.cpp` or `teleop_program.cpp`, so every merge result above stands as measured;
E2-6's five places are unchanged (`build-order.md:1409` is still line 1409, and `git grep` of
`ea58fbc` finds no new statement that the re-zero is A). COMP's brief is consistent with this
table: B11 names the re-zero "Teleop's constant (Y)", its cancel-all applies each listed
mechanism's safe state (for the pneumatic claw, the CLOSED command — E2-2's
`pneumaticSafeCommand`), and its own HA entries start after HA-160. The briefing conflicts at merge
as before (generated: regenerate).

**Nothing in this section has run on a robot.** The claw's polarity, its port and one wire or two
are still Station J.9's.

---

## The coordinator's merge — T3 Part 1 (2026-09-26, ~12:00–13:20)

**What was merged.** Both halves' uncommitted trees, each as a transport commit on its own branch —
`t3-prog-prep` at `f11ace6` (E + E2) and `t3-lib` at `e512327` (B + B2) — squash-merged in turn into
a staging worktree (`.claude/worktrees/t3-merge`, branch `t3-merge`) based on `ea58fbc`, so the main
tree was never half-merged; the result is ONE commit on `shulib-v2`.

**Conflicts and how each was resolved.**
- `include/shulib/teleop/teleop_loop.hpp` (two hunks — the one real code conflict): the library half
  made ONE loop body (bindings + toggles, older signatures forwarding); DEFECTS2 (`a8540e6`) had added
  the release mode at entry and after every `drive()`. Resolved: `drive()` → DEFECTS2's release mode
  (drive motors only, `driveMotors()`) → the hold bindings → the toggles → `perTick` → `pace`; the
  function doc carries both sides' `cfg` / `bindings` / `toggles` / `disabled` bullets; the header's
  placement sentence says "after step 6b"; `DriverLoopConfig`'s comment no longer says refusing the
  L1 default is "owed" — `validBindings()` now refuses a binding on the re-zero button before tick 1,
  so the collision cannot run silently (the default itself is still L1).
- `docs/changelog.md`: T3's "Added since 2.2" section above DEFECTS2's "Fixed since 2.2" (newest
  first), both whole.
- `docs/hardware-assumptions.md` (by hand, in two passes — E2's own dry run had predicted three hunks
  against `a8540e6`, and the library half added four more): the status line is **7 of 154** (141 +
  DEFECTS2's 5 + T3's HA-153..156 + HA-157..159 + HA-147); the status paragraph keeps DEFECTS2's
  sentences, then T3's, the addendum's and the merge's; the index rows run HA-141 (main's, once) →
  HA-147 → HA-153..159 in number order; the body keeps DEFECTS2's subsection, then T3's (retitled
  intro; HA-147 first). **HA-147 is NEW at this merge** — B2's review asked for the controller belief
  its toggle rests on to be registered: "a V5 controller's connected flag and its button levels
  change together" (invented), tagged in `mechanism_binding.hpp`'s A PRESS MUST BE SEEN rule. It takes
  a number from T3's own reservation, so nothing can collide; **HA-148..HA-152 were reserved for T3 and
  stay unused** (never reissued — the status paragraph says so). **HA-157's second half** (a level
  written while the brain is disabled) is now settled at COMP's station, not Station J: Station J's DO
  NOT list forbids the controller-menu start it needs, and COMP's station makes that start on purpose
  (COMP brief amendment **B28**, added here).
- `docs/internal/chunks/R3a-BENCH-WORKSHEET.md` (merged cleanly; then two additions B2 asked for):
  J.9.5 gains the held-A reconnect step (HA-147), and **J.9.7** pulls the valve's cable (HA-158).
- `docs/internal/chunks/T3-PROGRESS.md` (add/add): this file — both logs joined verbatim above.
- `docs/api/*`, `mkdocs.yml`, `docs/internal/PROJECT-BRIEFING.md`: regenerated, never hand-merged
  (the briefing twice — before the build, then after it, the stale-binary trap).
- `build-order.md` (T3's status line, the first `Next:`, a deviations row for this merge's rulings),
  `roadmap.md` ("Now"), and the COMP brief (B28).

**B2's decisions, ratified by the coordinator:** (1) `robot.pneumaticMechanism(name)` for a pneumatic
mechanism, `mechanism(name)` staying the motor lookup — a shared lookup would hand a binding a base
pointer it cannot use without a downcast (a recorded deviation from the addendum's `mechanism(name)`);
(2) the toggle's initial level is sent ONCE in its life, so a re-enable commands nothing — the team
lead's "keeps its state on disable"; (3) the config's `pneumaticStart` and the toggle's initial level
must be the one table value — Part 2's brief now requires a test pinning them equal; (4) a double
solenoid is a library change (every line of a mechanism gets one level), not a table edit — Station J
says which the robot has; (5) `IDeviceProvider::digitalOut()` is pure virtual (all three implementers
have it). `builtMechanismAt(i)` is the boot report's lookup over both kinds (Part 2's brief says so).

**The coordinator's verification (the merged tree, clean run 12:22–13:13; the laptop moved rooms
mid-run).** Suite **1533 | 1533 passed | 0 failed | 3 skipped**, 2,393,888 assertions;
`src_build_gate.py` self-test OK (33 detector cases) and check PASS on all six builds (0 disallowed,
beacons); the ARM header gate CLEAN (185 headers); both CI greps empty; `check-fresh` 0; the briefing
current; removability empty; the staleness audit clean. Hand mutations, each on a freshly linked
binary, restored by hash, the restored tree 1533/1533: **T1** DEFECTS2's release mode dropped from the
merged loop body → RED (`brake_stop_stack_test.cpp:294`, the drive no longer coasts); **T2** hold
bindings read their buttons while disconnected → RED; **T3** a hold binding on the re-zero button
allowed → RED; **T4** a toggle on the re-zero button allowed → RED; **T5** the 3-wire line made at the
SAFE level instead of the start level → RED (`pros_device_provider_test.cpp:598`); **T6** the disable
no longer safes the hold bindings → first form did not compile (an unused variable — not counted),
**T6b** `(void)binding;` → RED, 6 cases; **T7** Calibrate's claw-port predicate never matches → RED.
Read by the coordinator: the claw table against the team lead's rulings (ports, buttons, RED, 12 V,
Hold, the claw on A starting Closed, the polarity one UNVERIFIED constant), the merged loop body, the
toggle's press rule.

**Not done here — T3 Part 2 (the Teleop wiring), next:** the claws are built and bindable but no
program drives them yet. Its brief (committed with Part 2) carries every item both halves handed on:
the table → `MechanismConfig` conversion with the start LEVEL (no boot twitch), the bindings built and
validated in `teleopInitialize`, the re-zero A → Y with its `static_assert`, the claim release at loop
entry, the claw panel row, Station J's `<<FROM TELEOP AT MERGE>>` markers, and every place that still
says the re-zero is A. **Open for the team lead after Station J.9.5:** a reconnect settle window for
the toggle (B2's finding 1 — flagged, not built). **Recommended, not done:** an AddressSanitizer CI job
(B2's finding 4; ~9 s of suite time). **Nothing of T3 has run on a robot.**

---

## Part 2: Teleop wiring (worktree `t3-part2`, 2026-09-26)

> **Brief:** `docs/internal/chunks/T3-part2-teleop-wiring.md` (read in full, with the T3 brief, its
> addendum and every earlier section of this log). **Tree at start:** `cc513a7` on branch `t3-part2`
> (a git worktree), clean. **Executor model:** opus. **Started:** Sat Sep 26 01:33 PM EDT 2026.
> **Rules:** do not commit, do not push, no `pros`, nothing on `/dev/ttyACM*`; edit only this
> worktree; nothing in `src/main.cpp`, and nothing about how `src/mechanism_table.hpp` or
> `src/chassis_table.hpp` choose the robot (G1 Part 2c's final run owns those); "host test plant",
> never the other word; ASCII only in runtime strings; only a literal `"%s"` reaches a screen print.
> HA numbers, if needed: HA-148 or HA-149 only. The team lead's rulings for this run: R1/R2 drive
> port 6, L1/L2 port 7, 12 V, HOLD on release, both buttons of one pivot held => hold; A toggles the
> pneumatic claw, starts CLOSED, keeps its state on a disable; the re-zero button moves A -> Y; a
> claim left by a deleted autonomous task is RELEASED by `runDriverControl` at loop entry, and said so.

Appended in real time. **Nothing in this section has run on a robot.**

### P2.1 Log created; reading
Read in full: the Part 2 brief, the T3 brief, its addendum, this whole log (both halves, both
reviews, the coordinator's merge record), `RESUMING.md`. Code read: `src/teleop_program.cpp` (all),
`src/boot_source.{hpp,cpp}`, `src/mechanism_table.hpp`, `src/chassis_table.hpp` (the parts the
boot uses), `teleop/mechanism_binding.hpp`, `teleop/teleop_loop.hpp`, `robot/robot.hpp`'s
MECHANISMS section and slot storage, `config/robot_config.hpp`'s `MechanismConfig` /
`SafeBrakeMode` / `PneumaticLevel`, `hal/mechanism.hpp` (claim token, both mechanism types),
`calibration/boot_check.hpp`, `core/check.hpp`, the PROS shim's motors / rtos / misc,
`hal/pros/tick_pacer.hpp`, `test/boot_source_test.cpp` (fixtures and 6k-6r),
`test/mechanism_binding_test.cpp` (the loop rig, L4-L12), `test/pros_device_provider_test.cpp`
(1k, 1l, 1m), worksheet Stations F and J.

**Baseline (this worktree, fresh `build/test`, before any edit but this log):** `1533 cases /
2,393,888 assertions, 3 skipped — SUCCESS` — the coordinator's merged number, so the worktree is
the tree the brief was written against.

### P2.2 The design, before any code (each with the alternative rejected)

1. **The conversion lives in `src/boot_source.hpp`'s pure core** — `bootConfigFrom()` and
   `decideBootSourceFrom()` gain the mechanism table as an EXPLICIT parameter (no default: a caller
   that forgot it would silently build a claw-less robot). Config order is the table's: the motor
   entries in table order, then the pneumatic ones — so `robot.mechanismAt(i)` is motor entry `i`
   and `robot.pneumaticMechanismAt(motorCount + j)` is pneumatic entry `j`. Every field converted:
   name (copied whole — a name that does not fit `MechanismConfig::name` is REFUSED, never
   truncated), `motorPorts[0] = signedPort` / `motorCount = 1`, the cartridge (table enum ->
   `hal::MotorCartridge`, Unset -> the refused byte 0), the safe mode (`hal::BrakeMode` ->
   `config::SafeBrakeMode`), `pneumaticPorts[0] = adiLine` / `pneumaticCount = 1`, START and SAFE
   as LEVELS from `pneumaticStartCommand(e)` / `pneumaticSafeCommand(e)`. Buttons and volts are not
   config fields; they go to the bindings, from the same table entry. Rejected: a converter in
   `mechanism_table.hpp` (it would make the PROS-free table include the library's config and the
   robot builder), and a default `= kNoMechanisms` (hides the question the brief asks).
2. **The drive-port collision refusal is a BOOT decision** (`decideBootSourceFrom`, after the source
   is chosen, before anything is built): a declared mechanism motor port that the chosen description
   lists as a drive port refuses the boot by name. Rejected: leaving it to `RobotConfig::valid()` —
   it refuses too, but its words ("port 6 is claimed by the LEFT drive and by a mechanism motor")
   do not say what to DO; the brief's words do ("recalibrate").
3. **The boot check names declared claw ports** — a LIBRARY change, additive:
   `calibration::verifyAgainstRegistry(desc, registry, declared)` with
   `DeclaredMotorPorts{ports, label}`; a registry MOTOR on a declared port that the description
   does not list is counted in its own class and named with the caller's label ("claw pivots"),
   never as a NEW motor. The two-argument form forwards with no declared ports, and its prose is
   byte-identical (every existing pinned sentence stays). Rejected: post-editing the verdict's
   `reason` string in the program (fragile), and a registry wrapper that hides ports 6/7 (it would
   lie about the brain).
4. **A new PROS-free header, `src/teleop_claws.hpp`**, host-tested: the bindings' static storage
   and their construction from the table over a built `robot::Robot` (validated as one set), the
   claws' boot lines, the banner's claw sentence, the claw panel row, the controller's absent word,
   and the MERGED fault row (ruling R3 — the change that made room for the claw row). Everything
   the worksheet quotes is produced here, so a host test can pin it word for word;
   `src/teleop_program.cpp` (which the host cannot compile: no screen shim) only calls it.
   Rejected: all of it inside `teleop_program.cpp` (untestable text and logic).
5. **An absent mechanism has NO binding** — its buttons do nothing (the brief's "inert"); the
   binding set handed to the loop holds only built mechanisms. Rejected: a binding over a stand-in
   mechanism (a fake motor on the brain would be a lie in the panel's temperature).
6. **The claim ruling in the library loop:** after the `validBindings` precondition (a refused set
   still touches nothing) and BEFORE the entry safe state / `enter()`, each binding's NEW
   `releaseLeftoverClaim()` releases a claim still on its mechanism (never the claimant's
   `cancel()` — it may have lived on the deleted task's stack) and remembers it
   (`releasedLeftoverClaim()`, for a panel); the loop writes ONE Warn line per released claim
   through the chassis's own sink (`chassis.deps().ctx->telemetry()`, tag `TEL`), so no new loop
   parameter is needed and no program can forget to pass a sink. Hold AND toggle bindings (B2.11
   item 7: the same gap). A claim taken DURING the loop is still never commanded (unchanged).
   Rejected: a sink field in `DriverLoopConfig` (a nullptr default would let a program be silent);
   releasing inside `reset()`/`enter()` (a binding used outside the loop must keep the old rule).
7. **The panel:** the two fault rows become one — `faults: none`, or `FAULT n=<n> first <A> last
   <B>` when it fits the 50-character SMALL budget, else `FAULT n=<n> last <B>` (the latest is the
   one to act on; serial has both) — and the freed row is the claw row, `CLAW 6 HOLD 34C | 7 HOLD
   33C | CLAW CLOSED`, with ` FREED` at its end (amber) while a claim released at this entry is
   remembered. A pivot's word is what its motor was last COMMANDED (`FWD` > 0 V, `REV` < 0 V, else
   its declared safe mode's word, `HOLD`), not the binding's state — so an operation driving it
   would show too. Worst-case lengths checked at compile time for the build's table.
8. **The re-zero:** `kRezeroButton = Y`, `static_assert(kRezeroButton ==
   bench::kTeleopRezeroButton)`, and the shoulder-button assert extended by
   `mechanismButtonsAvoid(kMechanisms, kRezeroButton)` (never a binding button).

### P2.3 The code, written; the suite green

- **Library** (`include/shulib/`): `teleop/mechanism_binding.hpp` — `releaseLeftoverClaim()` /
  `releasedLeftoverClaim()` on both binding kinds; the header's "THE GAP THIS LEAVES" paragraph
  replaced by the ruling (what it releases, how it says so, what it costs). `teleop/teleop_loop.hpp`
  — the entry block (after the `validBindings` precondition, before `reset()`/`makeSafe()` and
  `enter()`): each binding's and toggle's leftover claim released and one Warn line per claim on
  `chassis.deps().ctx->telemetry()`, tag `TEL`; banner and function-doc bullets.
  `calibration/boot_check.hpp` — `DeclaredMotorPorts`, the three-argument `verifyAgainstRegistry()`
  (the two-argument form forwards), `BootVerdict::declaredPorts/declaredCount`. The NEW-motors
  clause is printed with precision 110 — `-Wformat-truncation` in the host build refused the first
  version (a 191-byte `%s` into a 320-byte reason); the arithmetic that it never cuts is in the
  comment.
- **Program** (`src/`): `boot_source.hpp` (the conversion, `mechanismTableProblem()`, the declared
  ports, the collision refusal; the table passed explicitly), `boot_source.cpp` (passes
  `kMechanisms`), NEW `teleop_claws.hpp` (`TeleopClaws` + every word), `teleop_program.cpp` (Y, the
  two static_asserts, `TeleopClaws` in `TeleopState`, bound inside the try with `unbind()` in the
  catch, the banner's claw sentence, one boot line per claw, the merged fault row, the claw row, the
  LCD's `CLAW ABSENT`, the bindings overload, the exit line), `teleop_program.hpp` (docs),
  `mechanism_table.hpp` (comments only: `kTeleopRezeroButton`'s doc now says TIED and where; "WHO
  READS IT" names Teleop). `kRunStatus` gained a sentence (the claws have never been driven), which
  made it 422 bytes: the src gate's `-Wformat-truncation` caught `status[400]` — now 480.
- **Tests:** `mechanism_binding_test.cpp` — L10 REWRITTEN to the ruling (it pinned the gap), L13
  (a claim taken during the loop), B10b (the binding half), L11 extended (a refused set releases no
  claim), `ClawLoop` now carries a `FakeTelemetrySink` on its chassis; `toggle_binding_test.cpp` —
  T5b, TL6, TL5 extended; `boot_check_test.cpp` — 6f'; `boot_source_test.cpp` — 6s-6v, `runBoot`
  takes the table (default none), the G1 2b cases pass `kNoMechanisms` explicitly; NEW
  `teleop_claws_test.cpp` — test 1 (the pivots through the shim, the REAL table), 1b-1j.
- Suite after: **1552 cases / 2,396,838 assertions, 3 skipped — SUCCESS** (baseline 1533 /
  2,393,888; +19 cases). Noted: `teleop_claws_test.cpp`'s runs go through the REAL provider, whose
  sink is a `TermSink` on stdout, so the suite now prints ~300 `[LOC]` lines while those cases run
  — the robot's own per-tick telemetry, harmless, not muted (muting it would need a provider seam
  the brief does not own).
- `src_build_gate.py check`: PASS, all six builds (6/6 TUs), 0 disallowed — after the `status[480]`
  fix.

### P2.4 RESUMED AFTER THE POWER LOSS (a second executor run, opus, started 20:22 EDT)

The machine lost power during the first run; nothing was committed and `/tmp` was wiped by the
reboot. This run read this log to its end, `git status` / `git diff` of the worktree (24 files
modified, `src/teleop_claws.hpp` and `test/teleop_claws_test.cpp` untracked), and — the one record
that survived — the first run's own session transcript (`~/.claude/projects/…/subagents/workflows/
wf_399b7be9-2da/agent-a07d87ee11490fa81.jsonl`), before touching anything.

**Where the first run really stopped.** Its last tool call was at **14:20:36 EDT** (reading the end
of Station J's send-back list), and it never returned: the transcript records it as a permission
request that was aborted when the session died. Everything between P2.3 (14:11) and that call is
NOT in this log, so it is listed here from the transcript and the tree:
- tests added after P2.3: `teleop_claws_test.cpp` **1k** (each toggle's initial level is the
  config's START level, with start != safe and both polarities) and **1l** ("shulib Teleop"'s
  wiring, read from its source) — the suite went 1552 → **1554** cases;
- documentation: `docs/cookbook/06-driver-control.md`, `docs/guide/10-the-api.md`, `11`, `14`,
  `docs/changelog.md` (a "Changed since 2.2" section: the claim ruling's behaviour change and the
  boot check's declared ports), worksheet Station F (the one fault row, the claw row, the LCD's
  `CLAW ABSENT`) and Station J (every `<<FROM TELEOP AT MERGE>>` replaced by a quote from the code);
- **a mutation battery, launched at 14:18 in the background** — a harness (`harness.py`: exact-once
  replacements, `touch test/*.cpp`, the `/fast` target, the `Linking CXX executable shulib_tests`
  line required, the whole suite, restore, SHA-256 compared) over **56 mutants** (`mutants.py`),
  both under `/tmp`. Both files were recovered verbatim from the transcript into
  `.claude/coordinator/scratch/t3p2/prev_{harness,mutants}.py`. Only its first three verdicts reached
  the transcript (B1 swap R1/R2 RED 6 cases; B2a drop the hold release RED 3; B2b drop the toggle
  release RED 2); the rest were written to `/tmp` and are GONE — none is claimed here.

**Was a mutation left applied?** The battery's harness restored each file after each mutant, and
the power loss could have caught one mid-run. Checked, not assumed:
1. **The file times** match the battery's order to its LAST mutant: `robot.hpp` 14:21:34 (B6),
   `teleop_loop.hpp` 14:23:30 (L1–L5), `mechanism_binding.hpp` 14:26:12 (L6–L12), `boot_check.hpp`
   14:27:40 (K1–K4), `boot_source.hpp` 14:31:35 (S1–S11), `teleop_claws.hpp` 17:01:51 (C1–C16),
   `teleop_program.cpp` 17:03:22 (P1–P3), `boot_source.cpp` 17:03:52 (P4, the last). (Why C1–C16
   took until 17:01 is not known — the machine may have slept; it does not change the check below.)
2. **Every one of the 56 mutants' anchors** is in the tree EXACTLY ONCE and no mutant's replacement
   text is present (a script over `prev_mutants.py`, 20:27): `56 mutants; suspects: 0`.
3. `include/shulib/robot/robot.hpp` (B6's file, the only battery file outside this chunk's diff) is
   byte-identical to `HEAD` (`git diff --quiet`).
4. **Every source change was read against P2.2/P2.3** (the whole `git diff` of `include/`, `src/`
   and `test/`, and both new files): each matches what the log says was built; nothing unexplained.
5. **The suite on a fresh full build** (the doc gates regenerated first; `touch test/*.cpp`; the
   Linking line seen): **1554 cases | 1554 passed | 0 failed | 3 skipped; 2,396,871 assertions**.

**Verdict: no mutation was left in the tree; the code is the first run's intended code.** This run
continues from where the transcript stops: the rest of §1 item 7's documents, the battery again on
the FINAL code (its evidence kept under `.claude/coordinator/scratch/t3p2/`, never `/tmp`), the
prose review, and the gates.

### P2.5 This run's changes to code and tests (20:27–20:35), then the battery launched

**Comments only, in `src/mechanism_table.hpp`** (stale since the wiring; no value moved):
`pneumaticStartCommand()`'s doc said "what the coordinator hands the library ... at merge" — now it
names BOTH its users (the config's `pneumaticStart` in `src/boot_source.hpp`, and the toggle's
initial level in `src/teleop_claws.hpp`); `pneumaticSafeCommand()`'s "at merge" likewise; robot
two's doc block said "no program in this tree drives a 3-wire line" (Teleop does now — not run on
the robot) and "nothing in shulib watches a mechanism motor's temperature yet" (Teleop's claw row
SHOWS it now; nothing ACTS on it — both said). `test/mechanism_table_test.cpp`: one comment's
"the coordinator's merge step" → where the tie now sits.

**Tests strengthened — each one kills a mutant this run added (run 2 below):**
- `teleop_claws_test.cpp` **1h** — FREED from EACH kind alone (a released claim on only pivot 7,
  then on only the claw): the first run's case claimed a pivot AND the claw at once, so a panel
  that read only one kind passed it (C17 / C18). Also: that release fired nothing at the claw.
- **1k** — the claw row for a table robot two's cannot imitate: `CLAW 9 BRAKE 0C | CLAMP CLOSED |
  WING OPEN` (a stopped mechanism's OWN mode, not a hard-coded HOLD — C19; a LOW-opens polarity
  read right — C10 again, from the other side).
- NEW **1m** — `bind()` with a 13 V entry THROWS `PreconditionError` (why Teleop binds inside its
  boot boundary's try), leaves pivot 6 bound (bind() does not clean up after a throw), and the
  catch's `unbind()` leaves nothing — the contract `teleop_program.cpp`'s catch relies on.
- `boot_source_test.cpp` **6u** — a pivot FLIPPED in the table (`-6`, Station J's fix for a claw
  that moves the wrong way) is still declared port 6 to the port check, and converts as `-6`
  (S14); **6v** — the flipped pivot still collides with a record's drive port 6 (S15).

Suite: **1555 cases | 1555 passed | 0 failed | 3 skipped; 2,396,896 assertions** (from 1554 /
2,396,871: +1 case, +25 assertions), `/fast` after `touch test/*.cpp`, the Linking line seen.

**The battery, run 2 (launched 20:35, background):** `.claude/coordinator/scratch/t3p2/
harness.py` (the first run's harness, byte for byte) over `mutants.py` = the first run's 56
(`prev_mutants.py`) + 10 new (B4c the TABLE's claw started open; S12 Brake converted as Hold; S13
every level inverted; S14 the declared ports keep their sign; S15 the collision on the signed
port; C17 / C18 FREED blind to a kind; C19 the stop word hard-coded; L14 / L15 a claim released
every tick instead of only at entry). All 66 anchors checked present exactly once before the
launch. Output: `run2.jsonl` beside it. No file under `include/`, `src/` or `test/` is edited
while it runs (the harness restores by rewriting the whole file).

### P2.6 The rest of §1 item 7's documents (20:36–20:50, while the battery ran)

What the first run had already written was re-read against the code before anything was added:
the Station J quotes (J.3.1 BUILT/ABSENT lines, the port-check clause, the collision refusal;
J.3.2 banner; J.3.3 row; J.4.6; J.7.1; J.9.1; J.9.3) against `src/teleop_claws.hpp`, the
`[R3B]` / `[WARN][R3B]` prefixes against `diag/term_sink.hpp`'s `log()` (Info carries no level
tag), the 196-column wrap against `kLogPieceCols`, "the row repaints at most five times a second"
against `kPanelEveryTicks = 20`, and J.7.1's "Teleop prints NO serial line when the controller
drops" against `teleop_loop.hpp` (its one `log()` is the claim release), `teleop_program.cpp`
(`connected` reaches only the panel and the LCD) and `hal/pros/controller.hpp` (no log) — all true.
The re-zero line's end, `re-zero button Y; field-centric: OFF (selectable)`, against `kDriverLine`
+ `buttonWord(kRezeroButton)`. Station F never named the re-zero button (grep of F, and of F at
`cc513a7`): nothing to move A → Y there; its fault rows (the table, the LCD) were done by the first
run.

Written by this run:
- **Station G** (`R3a-BENCH-WORKSHEET.md` ~:834): its `NEW motors not in the calibration: 14`
  quote now says the line goes on `; claw pivots (declared, not drive motors): 6 7` with the claw
  motors plugged in (a reader at G, before J, would otherwise log it as a finding).
- **`build-order.md`:** the first `Next:` is now G1 Part 2c, then COMP; a "T3 PART 2 LANDED"
  sentence ahead of Part 1's (which no longer says Part 2 is next); T3's status line (both parts
  in, **chassis macros still open**, this entry's DoD theirs); T2's status line (re-zero A → Y);
  a deviations row for the claim ruling (with the alternatives rejected).
- **`roadmap.md`:** "Now" (next: 2c, then COMP; T3 landed, both parts, not run); the
  field-centric row (re-zero A → Y); the "Button bindings / driver macros" row `[ ]` → `[~]`
  (mechanism buttons done host/build-gated; macros not started — Part 1 had left it `[ ]`).
- **`HANDOFF-remaining-work.md`:** robot one's paragraph (Calibrate takes every motor EXCEPT a
  declared mechanism port); L5 (done for the claws; the stick rule amended; the claim ruling; still
  open: macros, presets; the re-zero default GUARDED by `validBindings()`, Teleop on Y — replacing
  the "owed ... names A" paragraph); L9 (the named-mechanism lookup DONE at T3 Part 1 — it still
  said Robot builds no mechanism); Q1 (answered in part for robot two, the pneumatic claw
  included); §7 (DEFECTS2 and T3 landed; next 2c, then COMP).
- **`DEFECTS2-COMPLETED.md`:** its last NOT-done bullet (the L1 default) — what guards it now.
- **`hardware-assumptions.md`:** no new entry (HA-148..HA-152 stay unused); the status paragraph
  says so; HA-153 (Teleop builds/binds/names the pivots; a record listing 6/7 as drive refuses),
  HA-154 (the claw row now exists and SHOWS the temperature; nothing acts on it), HA-156 (its
  "binding not in the tree" sentence), HA-159 and the addendum paragraph ("no program drives a
  3-wire line" — Teleop does, not on the robot), the T3 intro ("drives them once Part 2 lands"),
  and the "not registered" table's pneumatics row (the claws' beliefs ARE registered now; the air
  budget is still not).
- **`README.md`** (the layout's `src/` line) and **`docs/guide/07-getting-set-up.md`** (the `src/`
  file list): Teleop's claw wiring and robot two's mechanism table named.
- Checked, NOT changed: `include/shulib/teleop/teleop_loop.hpp`'s `DriverLoopConfig` comment (the
  brief's "~:90": it no longer says A — "a program should still name its button, and 'shulib
  Teleop' does"); the COMP brief (`:89` already says T3 frees leftover claims; `:92` already says
  Y); `docs/vexbuilder-integration.md` (Part 2 changes no schema); guide 12 / 13 and the master
  plan (no Teleop, claw or claim-at-entry text); HANDOFF L1's "2c waits ... after DEFECTS2 and T3"
  (2c's line — its final run, in another worktree, owns it).

### P2.7 Prose review (the greps run and their hits read BEFORE this entry)

1. `grep -rn -i "re-zero\|rezero"` over `docs/ README.md include/ src/` (all `.md`, then the
   headers separately), minus generated `docs/api/`, PROGRESS logs and transcripts — every hit
   read. Stale ones fixed: roadmap `:1693`, build-order `:1409`, HANDOFF `:395-396`,
   DEFECTS2-COMPLETED `:56`. The rest are the loop's generic re-zero, library tests that use A as
   an arbitrary re-zero in their own rigs, the cookbook's L1/A/Y examples (compiled, correct), and
   history in briefs.
2. `grep -rn -i` for the changed files' names and concepts — `verifyAgainstRegistry`, `NEW motors`,
   `bootConfigFrom`, `decideBootSourceFrom`, `boot_source`, `teleop_claws`, `mechanism_table`,
   `kTeleopRezeroButton`, `kRezeroButton`, `releaseLeftoverClaim`, `releasedLeftoverClaim`,
   `DeclaredMotorPorts`, `fault rows`, `no mechanism(s)`, `builds no mechanism` — 54 hits
   (`scratch/t3p2/prose_grep1.txt`), each read: fixed the register's T3 intro (`:1626`) and its
   pneumatics exclusion row (`:2954`); the rest are current or history.
3. `grep -rn -i "claw\|interruptible\|MechanismConfig\|\bbinding"` over the public docs, README,
   HANDOFF, RESUMING and build-order — 302 hits (`prose_grep2.txt`), filtered for tense words
   (not yet / until / once ... lands / no program / owed / later) and those read: nothing stale left
   (the cookbook's and guide 10's "until" are the binding's inert rule).
4. `grep -rn "shulib Teleop\|Teleop"` over README and the public docs (minus register, changelog,
   roadmap, done above) — README `:244-248`, `:282`; faq `:204`; guide 07 `:124-126`, 08 `:310`, 10,
   14; vexbuilder-integration `:64` — read: README `:282` and guide 07 `:126` updated (the `src/`
   lists); the rest true.
5. `grep -rn "MechanismBinding\|ToggleBinding\|runDriverControl\|claimed()\|\bclaim\b"` over README
   and public docs — the cookbook, guide 10/11/12/13/14 hits read: the claim-at-entry rule is stated
   the same way everywhere it appears (cookbook 06, guide 10, guide 11's Warn line, changelog);
   guide 12's `anonymous claim force-released` is RunGuard's, unrelated.
6. Source comments: `grep -rn -i` for "until part 2 / no program drives / not yet / at merge /
   FROM TELEOP / coordinator.*merge" over `include/ src/ test/` — fixed `mechanism_table.hpp`
   (`pneumaticStartCommand`/`pneumaticSafeCommand`'s "at merge", robot two's "no program in this
   tree drives a 3-wire line" and "nothing ... watches a mechanism motor's temperature") and one
   test comment (P2.5). Every other hit is unrelated ("not yet" of other subsystems).
7. The C7 removability grep on public docs (`internal/`, `chunks/`, `RESUMING`, `build-order`) is
   run with the gates (P2.9). "Station J" in public prose has a precedent in the register's settle
   lines, and names no path.

### P2.8 The mutation battery, run 2 — 66 mutants, every one RED on a fresh binary (20:35–21:00)

Run on the FINAL code (no file under `include/`, `src/` or `test/` changed after 20:35 — the last
edits were P2.5's). Each mutant: exact-once replacement (anchor checked), `touch test/*.cpp`,
`cmake --build build/test --target shulib_tests/fast -j8` with the `Linking CXX executable
shulib_tests` line REQUIRED, the WHOLE suite, then the file restored by rewriting its saved text.
Harness end lines: `{"hashes_ok": true}` (every mutated file's SHA-256 equal before and after the
battery) and `{"final_relink": true, "rc": 0}`; afterwards all 66 anchors were re-found exactly
once and `robot.hpp` still equals `HEAD`. Evidence: `.claude/coordinator/scratch/t3p2/run2.jsonl`
(one JSON line per mutant: verdict, doctest summary, failing cases), `harness.py`, `mutants.py`,
`prev_mutants.py`, `mutation_table.md`. **Result: 66 of 66 RED, none survived, none failed to
compile.** Per-mutant build + suite: 19–28 s.

The brief's six (§1 item 6) are B1–B6 plus B4c: swap R1/R2 (B1); drop the claim release (B2a hold
bindings, B2b toggles); convert a pivot with Coast (B3); start the claw open (B4a the config's start
level, B4b the toggle's initial level, B4c the TABLE's start position); hand the library the claw's
POSITION instead of its level (B5a) or the safe level as the start (B5b); construct the 3-wire
output at the opposite level before setting the start (B6, in `robot.hpp`'s builder — caught by
Part 1's `pros_device_provider_test.cpp` 1m and this part's test 1c).

The five this run added a test for (P2.5) — **C17, C18, C19, S14, S15** — are RED only because of
those tests; that each would have SURVIVED the first run's tests is REASONED from what those tests
checked (1h claimed a pivot and the claw together; no row test used a non-Hold mechanism; 6u/6v
used only `+6`/`+7`), not run — the old tests no longer exist to run them against.

| # | Mutant | File | What it breaks | Verdict | Failing cases | First failing case |
|---|---|---|---|---|---|---|
| 1 | `B1-swapR1R2` | `src/teleop_claws.hpp` | the brief: swap R1/R2 in the wiring | **RED** | 6 | T3 P2 test 1 |
| 2 | `B2a-noHoldRelease` | `include/shulib/teleop/teleop_loop.hpp` | the brief: drop the claim release (hold bindings) | **RED** | 3 | T3 L10 |
| 3 | `B2b-noToggleRelease` | `include/shulib/teleop/teleop_loop.hpp` | the brief: drop the claim release (toggles) | **RED** | 2 | T3 P2 test 1h |
| 4 | `B3-pivotCoast` | `src/boot_source.hpp` | the brief: convert a pivot with Coast | **RED** | 6 | T3 P2 test 6s |
| 5 | `B4a-startOpenConfig` | `src/boot_source.hpp` | the brief: start the claw open (the config start level) | **RED** | 4 | T3 P2 test 6s |
| 6 | `B4b-startOpenToggle` | `src/teleop_claws.hpp` | the brief: start the claw open (the toggle initial level) | **RED** | 5 | T3 P2 test 1c |
| 7 | `B5a-positionAsLevel` | `src/boot_source.hpp` | the brief: the claw POSITION handed over instead of its level | **RED** | 2 | T3 P2 test 6s |
| 8 | `B5b-safeAsStart` | `src/boot_source.hpp` | the brief: the SAFE level handed over as the start | **RED** | 2 | T3 P2 test 6s |
| 9 | `B6-oppositeThenStart` | `include/shulib/robot/robot.hpp` | the brief: the 3-wire output constructed at the opposite level, then set to the start | **RED** | 5 | ProsDeviceProvider 1m |
| 10 | `L1-releaseBeforeCheck` | `include/shulib/teleop/teleop_loop.hpp` | claims released BEFORE the set check | **RED** | 6 | T3 L10 |
| 11 | `L2-holdReleaseAfterSafe` | `include/shulib/teleop/teleop_loop.hpp` | hold release AFTER the entry safe state | **RED** | 2 | T3 L10 |
| 12 | `L3-toggleReleaseAfterEnter` | `include/shulib/teleop/teleop_loop.hpp` | toggle release AFTER enter() | **RED** | 2 | T3 P2 test 1h |
| 13 | `L4-notSaid` | `include/shulib/teleop/teleop_loop.hpp` | the release is not logged | **RED** | 3 | T3 L10 |
| 14 | `L5-saidAsInfo` | `include/shulib/teleop/teleop_loop.hpp` | logged at Info, not Warn | **RED** | 3 | T3 L10 |
| 15 | `L6-holdFlagNeverSet` | `include/shulib/teleop/mechanism_binding.hpp` | hold binding: the panel flag never set | **RED** | 3 | T3 B10b |
| 16 | `L7-holdCallsClaimant` | `include/shulib/teleop/mechanism_binding.hpp` | hold binding: the dead claimant's cancel() called | **RED** | 2 | T3 B10b |
| 17 | `L8-holdFlagSticky` | `include/shulib/teleop/mechanism_binding.hpp` | hold binding: the flag never clears | **RED** | 3 | T3 B10b |
| 18 | `L9-holdReleaseSafes` | `include/shulib/teleop/mechanism_binding.hpp` | hold binding: the release also commands the safe state | **RED** | 2 | T3 B10b |
| 19 | `L10-toggleFlagNeverSet` | `include/shulib/teleop/mechanism_binding.hpp` | toggle: the panel flag never set | **RED** | 3 | T3 P2 test 1h |
| 20 | `L11-toggleReleaseFires` | `include/shulib/teleop/mechanism_binding.hpp` | toggle: the release fires the valve | **RED** | 3 | T3 P2 test 1h |
| 21 | `L12-toggleReleaseEnters` | `include/shulib/teleop/mechanism_binding.hpp` | toggle: the release spends the first entry | **RED** | 3 | T3 P2 test 1h |
| 22 | `K1-declaredIgnored` | `include/shulib/calibration/boot_check.hpp` | declared ports ignored (claws reported as NEW) | **RED** | 3 | BootCheck 6f' |
| 23 | `K2-labelDropped` | `include/shulib/calibration/boot_check.hpp` | the caller's label dropped | **RED** | 3 | BootCheck 6f' |
| 24 | `K3-clauseAlways` | `include/shulib/calibration/boot_check.hpp` | the declared clause printed with no declared motor | **RED** | 5 | BootCheck 6f' |
| 25 | `K4-drivePortDeclared` | `include/shulib/calibration/boot_check.hpp` | a listed drive port no longer skipped in the NEW/declared sweep | **RED** | 9 | BootCheck 6a |
| 26 | `S1-safeFromStart` | `src/boot_source.hpp` | the SAFE level taken from the start | **RED** | 2 | T3 P2 test 6s |
| 27 | `S2-signDropped` | `src/boot_source.hpp` | a pivot's sign dropped | **RED** | 2 | T3 P2 test 6s |
| 28 | `S3-cartridgeRedAsBlue` | `src/boot_source.hpp` | RED converted as the drive's BLUE | **RED** | 2 | T3 P2 test 6s |
| 29 | `S4-pneumaticsFirst` | `src/boot_source.hpp` | the config order: pneumatics before motors | **RED** | 13 | T3 P2 test 6s |
| 30 | `S5-noMechanismsInConfig` | `src/boot_source.hpp` | the mechanisms converted and then dropped | **RED** | 13 | T3 P2 test 6s |
| 31 | `S6-noCollisionCheck` | `src/boot_source.hpp` | the drive-port collision never refuses | **RED** | 1 | T3 P2 test 6v |
| 32 | `S7-collisionAlwaysRecord` | `src/boot_source.hpp` | the collision names the record for a typed table | **RED** | 1 | T3 P2 test 6v |
| 33 | `S8-nameFitOffByOne` | `src/boot_source.hpp` | a 23-character name refused | **RED** | 1 | T3 P2 test 6t |
| 34 | `S9-countFitOffByOne` | `src/boot_source.hpp` | eight mechanisms refused | **RED** | 1 | T3 P2 test 6t |
| 35 | `S10-noTableRefusal` | `src/boot_source.hpp` | a table that does not convert is not refused by the boot | **RED** | 1 | T3 P2 test 6t |
| 36 | `S11-recordNotDeclared` | `src/boot_source.hpp` | the record's port check not given the declared ports | **RED** | 2 | T3 P2 test 6u |
| 37 | `C1-toggleInitialFromSafe` | `src/teleop_claws.hpp` | the toggle's initial level = the SAFE level | **RED** | 1 | T3 P2 test 1k |
| 38 | `C2-toggleInitialFromPosition` | `src/teleop_claws.hpp` | the toggle's initial level = the start POSITION | **RED** | 1 | T3 P2 test 1k |
| 39 | `C3-rezeroIgnored` | `src/teleop_claws.hpp` | bind() validates against Y, not the loop re-zero it was given | **RED** | 1 | T3 P2 test 1d |
| 40 | `C4-refusedLeavesBindings` | `src/teleop_claws.hpp` | a refused set leaves its bindings behind | **RED** | 1 | T3 P2 test 1d |
| 41 | `C5-noNameCheck` | `src/teleop_claws.hpp` | bind() binds by index without the name check | **RED** | 1 | T3 P2 test 1i |
| 42 | `C6-noCountCheck` | `src/teleop_claws.hpp` | bind() ignores a count mismatch | **RED** | 1 | T3 P2 test 1i |
| 43 | `C7-kindAsAbsent` | `src/teleop_claws.hpp` | a built mechanism of the other kind read as ABSENT | **RED** | 1 | T3 P2 test 1i |
| 44 | `C8-rowFwdRev` | `src/teleop_claws.hpp` | the row: FWD/REV swapped | **RED** | 3 | T3 P2 test 1e |
| 45 | `C9-rowNoFreed` | `src/teleop_claws.hpp` | the row never says FREED | **RED** | 1 | T3 P2 test 1h |
| 46 | `C10-rowOpenClosed` | `src/teleop_claws.hpp` | the row: OPEN/CLOSED inverted | **RED** | 4 | T3 P2 test 1e |
| 47 | `C11-noTempClamp` | `src/teleop_claws.hpp` | the temperature not clamped | **RED** | 1 | T3 P2 test 1g |
| 48 | `C12-absentNoSentence` | `src/teleop_claws.hpp` | the ABSENT line without the provider's sentence | **RED** | 1 | T3 P2 test 1e |
| 49 | `C13-faultFallbackFirst` | `src/teleop_claws.hpp` | the short fault row names the FIRST fault | **RED** | 1 | T3 P2 test 1j |
| 50 | `C14-faultNoFallback` | `src/teleop_claws.hpp` | the full fault row always, cut at 50 | **RED** | 1 | T3 P2 test 1j |
| 51 | `C15-absentNeverSaid` | `src/teleop_claws.hpp` | anyClawAbsent() never true | **RED** | 1 | T3 P2 test 1e |
| 52 | `C16-bannerNoRezero` | `src/teleop_claws.hpp` | the banner forgets the re-zero button | **RED** | 1 | T3 P2 test 1g |
| 53 | `P1-loopWithoutBindings` | `src/teleop_program.cpp` | the loop called without the bindings | **RED** | 1 | T3 P2 test 1l |
| 54 | `P2-rezeroA` | `src/teleop_program.cpp` | the re-zero left on A | **RED** | 1 | T3 P2 test 1l |
| 55 | `P3-noUnbindInCatch` | `src/teleop_program.cpp` | the catch leaves the bindings | **RED** | 1 | T3 P2 test 1l |
| 56 | `P4-bootWithoutTable` | `src/boot_source.cpp` | the boot source handed no mechanisms | **RED** | 1 | T3 P2 test 1l |
| 57 | `B4c-tableStartOpen` | `src/mechanism_table.hpp` | the brief: start the claw open (robot two's TABLE) | **RED** | 7 | T3 P2 test 6s |
| 58 | `S12-brakeAsHold` | `src/boot_source.hpp` | a Brake safe mode converted as Hold | **RED** | 2 | T3 P2 test 6s |
| 59 | `S13-levelInverted` | `src/boot_source.hpp` | every pneumatic level inverted | **RED** | 4 | T3 P2 test 6s |
| 60 | `S14-declaredSigned` | `src/boot_source.hpp` | the declared ports keep their sign (a flipped pivot -6 becomes a NEW motor) | **RED** | 1 | T3 P2 test 6u |
| 61 | `S15-collisionSigned` | `src/boot_source.hpp` | the drive-port collision matched on the SIGNED port (a flipped pivot slips through) | **RED** | 1 | T3 P2 test 6v |
| 62 | `C17-freedIgnoresToggles` | `src/teleop_claws.hpp` | the panel's FREED ignores a released claim on the claw | **RED** | 1 | T3 P2 test 1h |
| 63 | `C18-freedIgnoresHolds` | `src/teleop_claws.hpp` | the panel's FREED ignores a released claim on a pivot | **RED** | 1 | T3 P2 test 1h |
| 64 | `C19-pivotWordFromSafeOnly` | `src/teleop_claws.hpp` | the row's stop word hard-coded HOLD, not the pivot's declared mode | **RED** | 1 | T3 P2 test 1k |
| 65 | `L14-holdReleaseEveryTick` | `include/shulib/teleop/teleop_loop.hpp` | hold bindings: a claim released every tick, not only at entry | **RED** | 3 | T3 L10 |
| 66 | `L15-toggleReleaseEveryTick` | `include/shulib/teleop/teleop_loop.hpp` | toggles: a claim released every tick, not only at entry | **RED** | 2 | T3 P2 test 1h |

**What no host test can reach** (so no mutant was run on it, stated rather than implied): the parts
of `src/teleop_program.cpp` that only the brain runs — the claw row's COLOUR (amber on ABSENT or
FREED), the LCD's `CLAW ABSENT` precedence, a boot line's Info/Warn level, the exit line. They are
compiled and linked by the src gate in all six builds (P2.9), and test 1l pins by READING the
source the four wiring lines whose loss would still compile (P1–P4). Everything they print comes
from `src/teleop_claws.hpp`, which the host suite does run.

### P2.9 Final gates (all in this worktree, 21:00–21:10, after the last edit to any header, source or test)

- `python3 tools/api_doc_tool.py generate` then `python3 tools/briefing_status.py generate` BEFORE
  the build; `touch test/*.cpp && cmake --build build/test -j8` → exit 0, the doc-gate target ran
  (coverage: 2679 public entities across 149 headers, all documented; examples: 437 quoted lines,
  all verbatim; removability clean; briefing current; staleness audit clean), `[100%] Linking CXX
  executable shulib_tests`.
- **Suite: `1555 | 1555 passed | 0 failed | 3 skipped`; `2396896 | 2396896 passed | 0 failed`;
  `Status: SUCCESS!`** (baseline at `cc513a7`: 1533 / 2,393,888 — **+22 cases, +3,008 assertions**
  for T3 Part 2).
- `briefing_status.py generate` AGAIN after the binary existed (it reads the suite's counts) →
  `check`: `briefing status: generated block current`. Its pointer now reads `Next up: G1 PART 2c
  ...`; suite 1,555 / 2,396,896; honest partials 10 → 11 (the Phase T row now `[~]`).
- `api_doc_tool.py`: `self-test` OK; `check-coverage` all documented; `check-fresh` exit 0;
  `check-examples` 437 lines verbatim; `check-removability`: no public doc references
  `docs/internal/` (and the manual C7 grep for `internal/ chunks/ RESUMING build-order` over the
  public docs: empty).
- `doc_staleness_audit.py self-test`: OK (10 cases); `doc_staleness_audit.py`: clean (27 live docs).
- `src_build_gate.py self-test`: OK (33 detector cases, each against a real build);
  `src_build_gate.py check`: **PASS — all 6 builds link, 6/6 TUs each, 0 disallowed** (2 allowed
  `-Wunused-function` in five builds, 0 in xdrive — the same counts as the first run's 14:09 gate),
  hash asserted `v0.1.1-335-gcc513a7-dirty`, beacons checked.
- ARM header gate (185 headers, `arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion
  -Wsign-conversion -Wshadow -Werror -Os -mcpu=cortex-a9 ...`): exit 0, clean.
- CI greps: `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core
  is sim-free`. No `SHULIB_RUNS_*` outside `src/main.cpp`.
- ASCII: no string literal in any changed runtime file carries a byte above 0x7F (a scan of the
  code with comments stripped); the panel still paints only through `printText()`'s literal `"%s"`.

### P2.10 Not done here, and for the team lead / coordinator

- **Nothing of T3 has run on a robot.** Every claim above is the host test plant, the PROS shim, or
  the six builds. Station J is the first sight of every claw.
- Not committed (by the rules). `T3-COMPLETED.md` is the coordinator's, after the commit.
- **Chassis macros** (the stick-interrupt case) and angle presets stay open under T3 (build-order's
  T3 entry says so now).
- Still open from Part 1, unchanged: a reconnect settle window for the toggle (B2's finding 1 — for
  the team lead after Station J.9.5), an AddressSanitizer CI job (recommended, not done).
- The library default `DriverLoopConfig::rezeroButton = L1` was NOT moved — GUARDED by
  `validBindings()` (DEFECTS2-COMPLETED's bullet says so now).
- A throw from `bind()` (a binding's precondition) is reported under the headline `the library
  graph REFUSED to build`, not `the claw buttons REFUSED` — the catch is shared; the reason
  sentence names the binding's own precondition. Left as is (cosmetic; noted for the coordinator).
- HANDOFF L1's "2c waits ... after DEFECTS2 and T3" is 2c's to update (its final run, another
  worktree). Merge note: this run's `build-order.md`, `roadmap.md`, `HANDOFF-remaining-work.md` and
  `hardware-assumptions.md` edits touch the same "Next"/"Now" blocks 2c, DIAG1 and COMP-a will; the
  briefing and `docs/api/` must be REGENERATED at the merge, never hand-merged.
- The suite prints ~300 `[LOC]` lines while `teleop_claws_test.cpp` runs (the REAL provider's
  TermSink on stdout) — harmless, not muted (a provider seam this chunk does not own).

### P2.11 Part 2 review findings and their disposition (review round 1; the builder again, started 21:30 EDT)

**Start (21:30).** Nine findings from the reviewer (one major, eight minor). Before anything: this
section was absent, so no earlier attempt at this round had written anything; every file under
`include/`, `src/` and `test/` is older than P2.9's gates (the newest, `test/*.cpp`, 21:02 — the
final gate's `touch`; `Makefile`, `src/bench_r3a.cpp` and a vendored lvgl header carry 21:07 mtimes
from the src gate's rebuild, and `git diff` shows no change to any of them); all 66 anchors of
`scratch/t3p2/mutants.py` are in the tree exactly once with no replacement text present
(`scratch/t3p2/review/anchors_check.py`: `66 mutants; suspects: 0`); `robot.hpp` equals `HEAD`.
So the tree is P2.9's. Scratch for this round: `.claude/coordinator/scratch/t3p2/review/`.

**Baseline for this round (21:31, a fresh `/fast` link, the Linking line seen):** `1555 | 1555
passed | 0 failed | 3 skipped; 2,396,896 assertions` — P2.9's numbers, so the binary matches the
tree the reviewer read.

#### P2.11.1 Finding 1 (major) — Station F's fault-row example could not appear: REAL, fixed

Re-checked against the code: `faultRowText()` (`src/teleop_claws.hpp`) prints the full form only
when it is at most `calibration::kSmallCols` (50) characters. `FAULT n=1 first MOTOR_GROUP_DISAGREE
last MOTOR_GROUP_DISAGREE` is 62 (16 + 20 + 6 + 20), so robot two shows `FAULT n=1 last
MOTOR_GROUP_DISAGREE`. The row's second example had the same flaw in a milder form: `FAULT n=3 last
MECHANISM_STALLED` appears only when the FIRST name is long enough to push the pair past 50 — and
MECHANISM_STALLED is not a fault Teleop raises. P2.6's "its fault rows ... were done by the first
run" was a statement about who wrote them, not a check; it should not have been read as one.
- **Fixed** (worksheet row 9): the leading example is now the realistic line, `FAULT n=1 last
  MOTOR_GROUP_DISAGREE`; the full form is quoted with `FAULT n=1 first IMU_LOST last IMU_LOST`
  (test 1j already pinned it) and, at the very edge, `FAULT n=2 first IMU_LOST last
  MOTOR_GROUP_DISAGREE` (exactly 50); the row says MOTOR_GROUP_DISAGREE's 20 characters usually
  leave room for one name. Grep: no other doc quotes the row (`grep -rn "FAULT n="` over `docs/`,
  `README.md`, `src/`, `include/` — only row 9, the code and old logs).
- **Tests** (1j): both quoted lines are now pinned — `FAULT n=1 last MOTOR_GROUP_DISAGREE`, and the
  50-character edge (`<=`, not `<`: a strict comparison would shorten it — mutant C23).
- **Found while re-checking Station F (not in the findings):** F.2.7 quoted the loop's exit line as
  `... cancel() (0 V + Brake); N screen prints so far` — but this chunk CHANGED that line (the
  first run's `teleop_program.cpp` edit: `each claw pivot made safe, the pneumatic claw left as it
  was;` before the count). Nobody had re-read F.2.7 against it. Fixed in the worksheet; test 1l now
  pins the line (mutant P13).

#### P2.11.2 Finding 2 (minor) — the typed-table branch's declared ports untested: REAL, fixed

Re-checked: `decideBootSourceFrom()` calls `verifyAgainstRegistry(..., declared)` twice (the
record, `src/boot_source.hpp` ~:476; the typed table's re-check, ~:506). 6u reaches only the
first; 6v's typed-table part ends in the collision refusal and reads only its headline and why.
Every `teleop_claws_test.cpp` boot uses a decoded record. So dropping `declared` from the second
call would have kept the suite green — reasoned, then RUN below as **S16** (RED on the new test
only).
- **New test 6w** (`boot_source_test.cpp`): no card, the usable typed table, a census with MOTORs
  on 6 and 7 — the WARNING line is pinned whole (`... NEW motors not in the calibration: none; claw
  pivots (declared, not drive motors): 6 7. NOT CHECKED: ...`), `newCount == 0`, `declaredCount ==
  2`, the config carries the three mechanisms; and the same boot WITHOUT the table prints `NEW
  motors not in the calibration: 6 7` (so the case cannot pass by accident).

#### P2.11.3 Finding 3 (minor) — Teleop's claw call sites deletable with the suite green; the log overstated 1l: REAL, fixed

Re-checked `src/teleop_program.cpp`: the banner's claw `printf` (:889), both boot-line loops in
`logBootLines()` (:862-872) and the claw-row paint (:743-746) were read by no test; test 1l pinned
only the re-zero, the static_asserts, the static storage, the try/catch, the loop call and the boot
source's table. P2.8's sentence "test 1l pins by READING the source the four wiring lines whose
loss would still compile (P1–P4)" implied those were the only such lines: they were not.
- **Test 1l extended** — each pinned WHOLE (so its arguments, a boot line's Info/Warn level, the
  claw row's amber condition and the LCD's order of urgency are pinned with it): the banner's claw
  sentence right after the BUILD STAMP `printf`; the motor boot-line loop; the pneumatic boot-line
  loop; the `logBootLines(r, log, st.boot)` call after the session header; the fault row and the
  claw row painted together; the LCD's third line (`... DEGRADED see brain` → `CLAW ABSENT` → `ok`);
  and the exit line (F.2.7, finding 1's extra).
- **Mutants P5–P14** (below) delete or weaken each; every one RED on test 1l. Whether the ARM
  build would ALSO have refused a deletion (an unused function or member) was not measured here —
  the reviewer reasoned it would compile; it no longer matters, because the host suite now refuses
  each.
- **The P2.8 paragraph, corrected here** (the log is append-only, so it is superseded, not
  edited): what no host test can reach in `teleop_program.cpp` is now only what the BRAIN does with
  the calls — the screen's rendering of a row and its colour, the controller LCD's rendering, the
  timing of the panel's repaint. Every claw-related STATEMENT in the file is pinned by test 1l's
  reading of the source, and every word it prints is produced and pinned by
  `src/teleop_claws.hpp`'s tests.

#### P2.11.4 Finding 4 (minor) — the claw row's compile-time width check not tied to a row: REAL, fixed

Re-checked: both `static_assert`s and test 1g said only `clawRowWorstLength(...) <= 50`, so an
under-count passed all three. For robot two the true worst row is exactly 50 characters:
`CLAW 6 HOLD 999C | 7 HOLD 999C | CLAW CLOSED FREED` (4 + 12 + 14 + 14 + 6).
- **Test 1g:** `clawRowWorstLength(kRobotTwoMechanisms) == 50` (exact); that worst row RENDERED by
  the code — both pivots stopped with the shim at 5000 C and 1000 C (clamped to 999), the claw
  CLOSED, a claim on pivot 7 released at entry — equals the string above and its length equals
  `clawRowWorstLength()`, so every character (FREED included) is kept; and
  `clawRowWorstLength(kNoMechanisms)` equals the no-claws row's length.
- **Mutants W1** (` FREED` not counted), **W2** (a 3-character temperature), **W3** (a
  2-character separator) — each still compiles (every sum stays <= 50) and each is RED on 1g.

#### P2.11.5 Finding 5 (minor) — a reversed pivot's words, and the 23-character name: REAL, fixed

Re-checked: `motorClawBootLine()`'s BUILT branch, `clawBannerLine()` and `clawRowText()`'s ABSENT
branch print `absPort(e.signedPort)`, and no test used a negative port in any of them (1k's `-9`
lift reached only the row's BUILT branch). `copyMechanismName()`'s bound was pinned only through
`mechanismTableProblem()` at 23/24.
- **New test 1n** (`teleop_claws_test.cpp`; the fixture now takes a table): pivot 6 flipped to `-6`
  — the BUILT line says `port 6 (typed -6)`, the banner `R1/R2 port 6`, the row `CLAW 6 HOLD ...`
  and, with R1 held, `CLAW 6 FWD ...` while the shim's port 6 receives `-12000 mV` (the flip reaches
  the motor through the port entry, as Station J's fix intends); with that port EMPTY, the ABSENT
  line (`port 6: nothing plugged in (expected a motor)`, the provider's own magnitude) and the row
  `CLAW 6 ABSENT | ...`. A 23-character pivot name boots, binds (bind()'s name check compares it
  WHOLE), and is carried whole into the config, the built mechanism and the boot line.
- **Test 6t:** `bootConfigFrom()` with 23-character names of BOTH kinds, compared whole.
- **Mutants C20 / C21 / C22** (the signed port in each text) and **S17** (`copyMechanismName`
  keeps 22) — RED.

#### P2.11.6 Finding 6 (minor) — the row's temperature can be stale; `--` never appears on the brain: REAL; documented, the code fix is NOT in scope

Re-checked: `hal::pros::ProsMotor::temperature()` (`include/shulib/hal/pros/motor.hpp` ~:269-282)
returns the last good value on a failed read (counted in `faultedReads()`) and 20 °C before any
good read; `hal::IMotor` has no validity channel and does not expose `faultedReads()`; the row reads
through `IMotor`. So on robot two a pivot that stops answering after boot shows its last
temperature, frozen and unmarked; one that never answered shows `20C`; `--` is unreachable there.
(A pivot unplugged AT BOOT is ABSENT — the census refuses it — so the realistic case is a cable
worked loose mid-run.) J.8 is where this bites: a frozen number while the driver holds a pivot
against its stop reads as "not heating".
- **Documented:** `src/teleop_claws.hpp` (`clawTemperatureText()`'s doc: what the number is on the
  brain, why `--` never shows there, why the branch stays); worksheet J.3.3 (last good reading,
  freezes unmarked, `20C` if never answered, no `--`) and J.8 (a number that has not moved for 30 s
  of holding is not proof the motor is cool — let go, check the cable, write it down); A4 register
  HA-154's settle text (the same, with the source).
- **Not done, and why:** showing `--` once reads fail needs the read's validity to cross the
  `hal::IMotor` seam — its header says it deliberately has none ("without this seam growing a
  validity channel F4 does not have", `pros/motor.hpp`'s `faultedReads()` doc) — or an accessor on
  `ProsDeviceProvider` for its concrete motors (it has none: `motors_` is private). Either is a
  library/HAL design change outside T3 Part 2's brief. **For the team lead** (P2.11.12).

#### P2.11.7 Finding 7 (minor) — the merge notes left out docs/changelog.md: REAL, fixed (merge notes below, P2.11.11)

Re-checked: `git diff cc513a7 shulib-v2 -- docs/changelog.md` — main added DIAG1's `### Changed
since 2.2 (2026-09-26) — an ODO_STUCK line names ...` and COMP-a's `### Added since 2.2
(2026-09-26) — the competition library ...` directly under `## API 2.2`, exactly where this
worktree adds its `### Changed since 2.2 (2026-09-26, later) — driver control releases a leftover
claim ...`. The files changed on BOTH sides since `cc513a7` (`comm` of the two `--name-only`
lists): `README.md`, `docs/api/README.md`, `docs/api/all-entities.md`, `docs/changelog.md`,
`docs/guide/11-reading-the-diagnostics.md`, `docs/hardware-assumptions.md`,
`docs/internal/HANDOFF-remaining-work.md`, `docs/internal/PROJECT-BRIEFING.md`,
`docs/internal/build-order.md`, `docs/roadmap.md` — P2.10 named four of the hand-merged ones, and
missed the changelog, guide 11 and the README. A trial merge (P2.11.11) says which conflict.

#### P2.11.8 Finding 8 (minor) — the claim release lets the claw's start level go out at the first entry: REAL; stated everywhere, the choice is the team lead's

Re-checked: `runDriverControl` releases leftover claims, then `reset()`/`makeSafe()`, then
`enter()` (`teleop_loop.hpp` ~:219-243); `ToggleBinding::enter()` sends the initial level on the
first entry unless the mechanism is claimed; TL6 and 1h pin that it goes out after a release. So a
claw an autonomous operation left claimed at the OTHER level now moves to its start level
(CLOSED, on robot two) when driver control first starts. Precisely what changed: an UNCLAIMED claw
autonomous had moved already did that (Part 1's rule — "the FIRST driver-control entry still sets
the initial level once"); the ruling extends it to a CLAIMED one, which Part 1 left where the
operation put it. The header's "a claim at the first entry means it is never sent" was still
there, true only for a program that calls `enter()` itself — misleading beside the next sentence.
- **Fixed:** `mechanism_binding.hpp` — the NO CLAIM bullet rewritten (enter() over a claim sends
  nothing; inside the loop the claim is released first, so the first entry always sends; the
  movement stated; only a program that calls enter() itself gets "never sent"), and `enter()`'s
  doc says the same in one sentence. `docs/changelog.md` — a "What it moves, for a toggle"
  sentence in the behaviour-change bullet, and a migration line (state the level autonomous leaves
  as the toggle's initial level; there is no "keep what is there" start). `docs/cookbook/
  06-driver-control.md` — the same in the claim bullet. The COMP brief's B10 note (the one that
  says "T3 also frees leftover claims") now carries the consequence and says COMP-b re-checks the
  team lead's answer before it wires the claw.
- **Not decided here:** today's Teleop runs no autonomous, so nothing on robot two can do this
  yet; COMP's will. **For the team lead** (P2.11.12).

#### P2.11.9 Finding 9 (minor) — a departure from the brief not logged with its alternative: REAL, logged now

The brief (§1 item 1) names `robot.builtMechanismAt(i)` / `robot.pneumaticMechanism(name)` as the
boot report's and the panel's claw lookups. What was built:
- the **boot lines** use `builtMechanismAt(i)` for both kinds (as the brief says) and
  `mechanismAbsence(i)` for an absent one's sentence;
- **`TeleopClaws::bind()`** binds a pivot through `robot.mechanismAt(i)` and the claw through
  `robot.pneumaticMechanismAt(k)` (`k = mechanismConfigIndexOfPneumatic(table, j)`), each only after
  checking the robot's mechanism COUNT equals the table's and the built mechanism's NAME at that
  index equals the entry's — a mismatch is refused, by name, and leaves nothing bound;
- the **claw row** reads the same indices (`mechanismAt(i)` / `pneumaticMechanismAt(k)`), relying
  on bind()'s check at boot (the robot never changes after it).

**Chosen: index lookup + name check. Rejected: `pneumaticMechanism(name)` (and `mechanism(name)`)
alone.** Why: a name lookup binds to whatever mechanism has that name in the robot — built from a
DIFFERENT table, or in another order, it would still find "claw" and bind A to it, and a name the
robot does not have would come back null and be indistinguishable from an ABSENT claw (the button
silently inert). The index + name check REFUSES a robot not converted from this table in its
order (test 1i: swapped pivots, a missing entry, the other kind at an index — mutants C5, C6, C7),
and it keeps ONE source of truth for the order (`bootConfigFrom()`'s CONFIG ORDER) shared by the
bindings, the boot lines and the row. Cost: `bind()` depends on the conversion's order, which
`mechanismConfigIndexOfPneumatic()` states and 6s pins (`== 2` for robot two's claw). This entry
should have been in P2.2 (it was only in the first builder's report); it is here now.

#### P2.11.10 Prose review for this round (the greps run and their hits read BEFORE this entry)

- `grep -rn "FAULT n="` over `docs/`, `README.md`, `src/`, `include/` (minus PROGRESS logs): row 9
  of Station F, the code and its tests only — row 9 fixed (P2.11.1).
- `grep -rn "screen prints so far"` over the docs: F.2.7 only — fixed (P2.11.1's extra).
- `grep -rn -i "never sent\|initial level"` over `docs/`, `README.md`, `include/shulib/teleop/`
  (minus generated `docs/api/`, logs): the cookbook, the changelog (the Part 1 toggle bullet at
  ~:100 — "sent once, at the binding's first loop entry ... never commands a claimed mechanism" —
  still true: the loop releases the claim first, which the 2.2 "Changed" section above it now
  says), the COMP brief, HA-157's settle text (~:1817-1825: consistent — the toggle's initial level
  at driver-control start), the Part 2 brief, and `mechanism_binding.hpp` / `teleop_loop.hpp`
  (the loop's "its initial level, once in its life" lines: consistent). Fixed: the header's NO
  CLAIM bullet and `enter()`'s doc (P2.11.8).
- `grep -rn -i "claw row"` with "temp" over the docs: HA-154's settle and blast-radius lines,
  worksheet J.6's table heading, J.8, Station F's row 10 — each now says the row shows the LAST
  GOOD reading (HA-154's blast radius and Station F row 10 amended too, beyond the two places the
  finding named). Guides 11 and 14 do not mention the claw row's temperature (grepped: no hit).
- `grep -rn "clawRowWorstLength\|copyMechanismName\|faultRowText"` over the docs: none outside
  logs — they are src/-internal names; nothing public to update.
- The C7 removability grep over the public docs: run with the gates (P2.11.14).

#### P2.11.11 Merge notes, corrected (supersede P2.10's merge-note sentence)

**A trial merge, objects only** (`git stash create` of this worktree's tracked changes → commit
`abfdd8a`, no ref written; `git merge-tree --write-tree --name-only shulib-v2 abfdd8a`; the two
untracked new files, `src/teleop_claws.hpp` and `test/teleop_claws_test.cpp`, do not exist on
`shulib-v2`, so they cannot conflict). Run after the battery, on this round's tree before its last
changelog edit (the two cross-references named by title — see the changelog bullet below). **Seven files conflict:**
- **Regenerate, never hand-merge:** `docs/api/README.md`, `docs/api/all-entities.md` (and every
  `docs/api/` page, `mkdocs.yml`), `docs/internal/PROJECT-BRIEFING.md` — the briefing twice, before
  the build and after the suite binary exists.
- **`docs/changelog.md`** — the finding: three sections now sit directly under `## API 2.2`.
  **Order, newest first by landing:** T3 Part 2's `### Changed since 2.2 (2026-09-26, later) —
  driver control releases a leftover claim; ...` FIRST (it lands after COMP-a), then main's COMP-a
  `### Added since 2.2 (2026-09-26) — the competition library ...`, then main's DIAG1 `### Changed
  since 2.2 (2026-09-26) — an ODO_STUCK line names ...`, then T3 Part 1's `### Added since 2.2
  (2026-09-26) — mechanism buttons ...`, then DEFECTS2's `### Fixed since 2.2`. Each section
  whole. The two cross-references between T3's sections name their target section by title (fixed
  in this round), so they survive the sections landing between them.
- **`docs/hardware-assumptions.md`** — hand-merge: main's COMP-a/DIAG1 register changes (HA-05
  amended, HA-161 new, the status line/paragraph) with this chunk's HA-153/154/156/159 and T3-intro
  edits; the status line's count must be recomputed from the merged index, not taken from either
  side.
- **`docs/internal/build-order.md`**, **`docs/roadmap.md`** — hand-merge the "Next"/"Now" blocks
  and status lines (both sides moved them; the FIRST bold `Next:` must be true for the merged tree —
  the briefing derives its pointer from it).
- **Merged cleanly in the trial, but re-read after the merge:** `README.md` (main: the layout's
  `competition/` and `ui/` lines; here: the `src/` line), `docs/guide/11-reading-the-diagnostics.md`
  (main: the ODO_STUCK section; here: the released-claim Warn line), `docs/internal/
  HANDOFF-remaining-work.md`.
- **Also changed here since P2.10, for the merge record:** `docs/internal/chunks/COMP-competition-
  program.md` (B10's note, P2.11.8 — main has not touched the brief since `cc513a7`),
  `docs/cookbook/06-driver-control.md`.

#### P2.11.12 For the team lead (nothing here decided by this run)

1. **The claw at the first driver-control start after an autonomous (finding 8).** Since the claim
   ruling, the first `runDriverControl` entry always sends the toggle's initial level — robot two's
   claw is commanded CLOSED when driver control starts, whatever autonomous left it at, claimed or
   not (before the ruling, a claw an operation still had claimed stayed where it was; an unclaimed
   one already closed). Today's Teleop runs no autonomous, so it cannot happen yet; COMP's
   autonomous will command the claw. Is "the claw closes when driver control starts" wanted? If
   not, the toggle needs a "keep what is there" start (a library change: the first entry would adopt
   `mechanism.commanded()` instead of sending a level). The COMP brief's B10 note carries the
   question to COMP-b.
2. **A stale claw temperature (finding 6).** The claw row can show a frozen last-good temperature
   (or 20 °C) with no mark, because `hal::IMotor` carries no read validity. Documented at J.3.3,
   J.8 and HA-154 (let go and check the cable if the number does not move). A real fix — `--` once a
   pivot's reads fail — needs either a validity signal across the `IMotor` seam (its header says it
   deliberately has none) or a `ProsDeviceProvider` accessor for its motors' `faultedReads()`. Worth
   a chunk before J.8 is trusted for heat numbers? Not decided here.
3. Still open from P2.10, unchanged: the reconnect settle window (after J.9.5), an AddressSanitizer
   CI job, chassis macros and angle presets, and the shared-catch headline for a `bind()` throw.

#### P2.11.13 The battery on this round's FINAL code — 85 mutants, every one RED on a fresh binary (22:02–22:32)

Run after the last edit to any file under `include/`, `src/` or `test/` (the comment edits of
P2.11.6 / P2.11.8 and the tests of P2.11.1–P2.11.5 all came first). The harness is run 2's,
byte for byte (`review/harness.py`, `cmp` identical to `../harness.py`): exact-once replacements,
`touch test/*.cpp`, the `/fast` target with the `Linking CXX executable shulib_tests` line
REQUIRED, the WHOLE suite, the file restored from its saved text. The set (`review/mutants_r1.py`)
= run 2's 66 (`../mutants.py`, unchanged) + 19 new for this round. Suite before the battery:
**1557 | 1557 passed | 0 failed | 3 skipped; 2,396,948 assertions**. Harness end lines:
`{"hashes_ok": true}` (every mutated file's SHA-256 equal before and after) and
`{"final_relink": true, "rc": 0}`; afterwards every anchor was re-found exactly once
(`anchors_check.py`: its one flag, P12's replacement text, is a false positive — that exact text
is an unrelated pre-existing call at `teleop_program.cpp:800`), and `robot.hpp` still equals
`HEAD`. **Result: 85 of 85 RED, none survived, none failed to compile;** 20–21 s each. Evidence:
`.claude/coordinator/scratch/t3p2/review/run_r1.jsonl`, `mutation_table_r1.md`.

Rows 1–66 are run 2's mutants again on the final code: every verdict RED again, and none fails
FEWER cases than in run 2 (compared line by line against `../run2.jsonl`); 24 fail one or two
MORE — wherever the recorded list shows which (the harness keeps the first eight failing cases),
a case this round added (1g, 1n, 6t, 6w). The new ones — 67–85 — are each RED ONLY
because of this round's tests: that each would have SURVIVED the P2.9 suite is reasoned from
what those tests read (the finding texts, re-checked in P2.11.1–P2.11.5), not run — those tests
no longer exist to run them against.

| # | Mutant | File | What it breaks | Verdict | Failing cases | First failing case |
|---|---|---|---|---|---|---|
| 1 | `B1-swapR1R2` | `src/teleop_claws.hpp` | the brief: swap R1/R2 in the wiring | **RED** | 7 | T3 P2 test 1 |
| 2 | `B2a-noHoldRelease` | `include/shulib/teleop/teleop_loop.hpp` | the brief: drop the claim release (hold bindings) | **RED** | 4 | T3 L10 |
| 3 | `B2b-noToggleRelease` | `include/shulib/teleop/teleop_loop.hpp` | the brief: drop the claim release (toggles) | **RED** | 2 | T3 P2 test 1h |
| 4 | `B3-pivotCoast` | `src/boot_source.hpp` | the brief: convert a pivot with Coast | **RED** | 7 | T3 P2 test 6s |
| 5 | `B4a-startOpenConfig` | `src/boot_source.hpp` | the brief: start the claw open (the config start level) | **RED** | 5 | T3 P2 test 6s |
| 6 | `B4b-startOpenToggle` | `src/teleop_claws.hpp` | the brief: start the claw open (the toggle initial level) | **RED** | 6 | T3 P2 test 1c |
| 7 | `B5a-positionAsLevel` | `src/boot_source.hpp` | the brief: the claw POSITION handed over instead of its level | **RED** | 2 | T3 P2 test 6s |
| 8 | `B5b-safeAsStart` | `src/boot_source.hpp` | the brief: the SAFE level handed over as the start | **RED** | 2 | T3 P2 test 6s |
| 9 | `B6-oppositeThenStart` | `include/shulib/robot/robot.hpp` | the brief: the 3-wire output constructed at the opposite level, then set to the start | **RED** | 5 | ProsDeviceProvider 1m |
| 10 | `L1-releaseBeforeCheck` | `include/shulib/teleop/teleop_loop.hpp` | claims released BEFORE the set check | **RED** | 7 | T3 L10 |
| 11 | `L2-holdReleaseAfterSafe` | `include/shulib/teleop/teleop_loop.hpp` | hold release AFTER the entry safe state | **RED** | 2 | T3 L10 |
| 12 | `L3-toggleReleaseAfterEnter` | `include/shulib/teleop/teleop_loop.hpp` | toggle release AFTER enter() | **RED** | 2 | T3 P2 test 1h |
| 13 | `L4-notSaid` | `include/shulib/teleop/teleop_loop.hpp` | the release is not logged | **RED** | 3 | T3 L10 |
| 14 | `L5-saidAsInfo` | `include/shulib/teleop/teleop_loop.hpp` | logged at Info, not Warn | **RED** | 3 | T3 L10 |
| 15 | `L6-holdFlagNeverSet` | `include/shulib/teleop/mechanism_binding.hpp` | hold binding: the panel flag never set | **RED** | 4 | T3 B10b |
| 16 | `L7-holdCallsClaimant` | `include/shulib/teleop/mechanism_binding.hpp` | hold binding: the dead claimant's cancel() called | **RED** | 2 | T3 B10b |
| 17 | `L8-holdFlagSticky` | `include/shulib/teleop/mechanism_binding.hpp` | hold binding: the flag never clears | **RED** | 3 | T3 B10b |
| 18 | `L9-holdReleaseSafes` | `include/shulib/teleop/mechanism_binding.hpp` | hold binding: the release also commands the safe state | **RED** | 2 | T3 B10b |
| 19 | `L10-toggleFlagNeverSet` | `include/shulib/teleop/mechanism_binding.hpp` | toggle: the panel flag never set | **RED** | 3 | T3 P2 test 1h |
| 20 | `L11-toggleReleaseFires` | `include/shulib/teleop/mechanism_binding.hpp` | toggle: the release fires the valve | **RED** | 3 | T3 P2 test 1h |
| 21 | `L12-toggleReleaseEnters` | `include/shulib/teleop/mechanism_binding.hpp` | toggle: the release spends the first entry | **RED** | 3 | T3 P2 test 1h |
| 22 | `K1-declaredIgnored` | `include/shulib/calibration/boot_check.hpp` | declared ports ignored (claws reported as NEW) | **RED** | 4 | BootCheck 6f' |
| 23 | `K2-labelDropped` | `include/shulib/calibration/boot_check.hpp` | the caller's label dropped | **RED** | 4 | BootCheck 6f' |
| 24 | `K3-clauseAlways` | `include/shulib/calibration/boot_check.hpp` | the declared clause printed with no declared motor | **RED** | 6 | BootCheck 6f' |
| 25 | `K4-drivePortDeclared` | `include/shulib/calibration/boot_check.hpp` | a listed drive port no longer skipped in the NEW/declared sweep | **RED** | 10 | BootCheck 6a |
| 26 | `S1-safeFromStart` | `src/boot_source.hpp` | the SAFE level taken from the start | **RED** | 2 | T3 P2 test 6s |
| 27 | `S2-signDropped` | `src/boot_source.hpp` | a pivot's sign dropped | **RED** | 3 | T3 P2 test 6s |
| 28 | `S3-cartridgeRedAsBlue` | `src/boot_source.hpp` | RED converted as the drive's BLUE | **RED** | 2 | T3 P2 test 6s |
| 29 | `S4-pneumaticsFirst` | `src/boot_source.hpp` | the config order: pneumatics before motors | **RED** | 15 | T3 P2 test 6s |
| 30 | `S5-noMechanismsInConfig` | `src/boot_source.hpp` | the mechanisms converted and then dropped | **RED** | 15 | T3 P2 test 6s |
| 31 | `S6-noCollisionCheck` | `src/boot_source.hpp` | the drive-port collision never refuses | **RED** | 1 | T3 P2 test 6v |
| 32 | `S7-collisionAlwaysRecord` | `src/boot_source.hpp` | the collision names the record for a typed table | **RED** | 1 | T3 P2 test 6v |
| 33 | `S8-nameFitOffByOne` | `src/boot_source.hpp` | a 23-character name refused | **RED** | 2 | T3 P2 test 6t |
| 34 | `S9-countFitOffByOne` | `src/boot_source.hpp` | eight mechanisms refused | **RED** | 1 | T3 P2 test 6t |
| 35 | `S10-noTableRefusal` | `src/boot_source.hpp` | a table that does not convert is not refused by the boot | **RED** | 1 | T3 P2 test 6t |
| 36 | `S11-recordNotDeclared` | `src/boot_source.hpp` | the record's port check not given the declared ports | **RED** | 2 | T3 P2 test 6u |
| 37 | `C1-toggleInitialFromSafe` | `src/teleop_claws.hpp` | the toggle's initial level = the SAFE level | **RED** | 1 | T3 P2 test 1k |
| 38 | `C2-toggleInitialFromPosition` | `src/teleop_claws.hpp` | the toggle's initial level = the start POSITION | **RED** | 1 | T3 P2 test 1k |
| 39 | `C3-rezeroIgnored` | `src/teleop_claws.hpp` | bind() validates against Y, not the loop re-zero it was given | **RED** | 1 | T3 P2 test 1d |
| 40 | `C4-refusedLeavesBindings` | `src/teleop_claws.hpp` | a refused set leaves its bindings behind | **RED** | 1 | T3 P2 test 1d |
| 41 | `C5-noNameCheck` | `src/teleop_claws.hpp` | bind() binds by index without the name check | **RED** | 1 | T3 P2 test 1i |
| 42 | `C6-noCountCheck` | `src/teleop_claws.hpp` | bind() ignores a count mismatch | **RED** | 1 | T3 P2 test 1i |
| 43 | `C7-kindAsAbsent` | `src/teleop_claws.hpp` | a built mechanism of the other kind read as ABSENT | **RED** | 1 | T3 P2 test 1i |
| 44 | `C8-rowFwdRev` | `src/teleop_claws.hpp` | the row: FWD/REV swapped | **RED** | 4 | T3 P2 test 1e |
| 45 | `C9-rowNoFreed` | `src/teleop_claws.hpp` | the row never says FREED | **RED** | 2 | T3 P2 test 1g |
| 46 | `C10-rowOpenClosed` | `src/teleop_claws.hpp` | the row: OPEN/CLOSED inverted | **RED** | 5 | T3 P2 test 1e |
| 47 | `C11-noTempClamp` | `src/teleop_claws.hpp` | the temperature not clamped | **RED** | 1 | T3 P2 test 1g |
| 48 | `C12-absentNoSentence` | `src/teleop_claws.hpp` | the ABSENT line without the provider's sentence | **RED** | 2 | T3 P2 test 1e |
| 49 | `C13-faultFallbackFirst` | `src/teleop_claws.hpp` | the short fault row names the FIRST fault | **RED** | 1 | T3 P2 test 1j |
| 50 | `C14-faultNoFallback` | `src/teleop_claws.hpp` | the full fault row always, cut at 50 | **RED** | 1 | T3 P2 test 1j |
| 51 | `C15-absentNeverSaid` | `src/teleop_claws.hpp` | anyClawAbsent() never true | **RED** | 1 | T3 P2 test 1e |
| 52 | `C16-bannerNoRezero` | `src/teleop_claws.hpp` | the banner forgets the re-zero button | **RED** | 2 | T3 P2 test 1g |
| 53 | `P1-loopWithoutBindings` | `src/teleop_program.cpp` | the loop called without the bindings | **RED** | 1 | T3 P2 test 1l |
| 54 | `P2-rezeroA` | `src/teleop_program.cpp` | the re-zero left on A | **RED** | 1 | T3 P2 test 1l |
| 55 | `P3-noUnbindInCatch` | `src/teleop_program.cpp` | the catch leaves the bindings | **RED** | 1 | T3 P2 test 1l |
| 56 | `P4-bootWithoutTable` | `src/boot_source.cpp` | the boot source handed no mechanisms | **RED** | 1 | T3 P2 test 1l |
| 57 | `B4c-tableStartOpen` | `src/mechanism_table.hpp` | the brief: start the claw open (robot two's TABLE) | **RED** | 8 | T3 P2 test 6s |
| 58 | `S12-brakeAsHold` | `src/boot_source.hpp` | a Brake safe mode converted as Hold | **RED** | 2 | T3 P2 test 6s |
| 59 | `S13-levelInverted` | `src/boot_source.hpp` | every pneumatic level inverted | **RED** | 5 | T3 P2 test 6s |
| 60 | `S14-declaredSigned` | `src/boot_source.hpp` | the declared ports keep their sign (a flipped pivot -6 becomes a NEW motor) | **RED** | 1 | T3 P2 test 6u |
| 61 | `S15-collisionSigned` | `src/boot_source.hpp` | the drive-port collision matched on the SIGNED port (a flipped pivot slips through) | **RED** | 1 | T3 P2 test 6v |
| 62 | `C17-freedIgnoresToggles` | `src/teleop_claws.hpp` | the panel's FREED ignores a released claim on the claw | **RED** | 1 | T3 P2 test 1h |
| 63 | `C18-freedIgnoresHolds` | `src/teleop_claws.hpp` | the panel's FREED ignores a released claim on a pivot | **RED** | 2 | T3 P2 test 1g |
| 64 | `C19-pivotWordFromSafeOnly` | `src/teleop_claws.hpp` | the row's stop word hard-coded HOLD, not the pivot's declared mode | **RED** | 1 | T3 P2 test 1k |
| 65 | `L14-holdReleaseEveryTick` | `include/shulib/teleop/teleop_loop.hpp` | hold bindings: a claim released every tick, not only at entry | **RED** | 4 | T3 L10 |
| 66 | `L15-toggleReleaseEveryTick` | `include/shulib/teleop/teleop_loop.hpp` | toggles: a claim released every tick, not only at entry | **RED** | 2 | T3 P2 test 1h |
| 67 | `S16-tableNotDeclared` | `src/boot_source.hpp` | the TYPED TABLE's port check not given the declared ports (claws NEW with no SD card) | **RED** | 1 | T3 P2 test 6w |
| 68 | `S17-nameCopyShort` | `src/boot_source.hpp` | copyMechanismName cuts a 23-character name to 22 | **RED** | 2 | T3 P2 test 6t |
| 69 | `W1-worstNoFreed` | `src/teleop_claws.hpp` | clawRowWorstLength forgets " FREED" (44 for robot two: the guard passes, checking nothing) | **RED** | 1 | T3 P2 test 1g |
| 70 | `W2-worstTempShort` | `src/teleop_claws.hpp` | clawRowWorstLength counts a 3-character temperature | **RED** | 1 | T3 P2 test 1g |
| 71 | `W3-worstSepShort` | `src/teleop_claws.hpp` | clawRowWorstLength counts a 2-character pivot separator | **RED** | 1 | T3 P2 test 1g |
| 72 | `C20-bootLineSignedPort` | `src/teleop_claws.hpp` | the BUILT boot line prints the SIGNED port (port -6 after a flip) | **RED** | 1 | T3 P2 test 1n |
| 73 | `C21-bannerSignedPort` | `src/teleop_claws.hpp` | the banner prints the SIGNED port | **RED** | 1 | T3 P2 test 1n |
| 74 | `C22-absentRowSignedPort` | `src/teleop_claws.hpp` | the row's ABSENT pivot printed with its SIGNED port | **RED** | 1 | T3 P2 test 1n |
| 75 | `C23-faultEdgeStrict` | `src/teleop_claws.hpp` | a full fault row of exactly 50 characters falls back to the short form | **RED** | 1 | T3 P2 test 1j |
| 76 | `P5-noBanner` | `src/teleop_program.cpp` | the banner's claw sentence never printed (J.3.2) | **RED** | 1 | T3 P2 test 1l |
| 77 | `P6-noMotorBootLines` | `src/teleop_program.cpp` | the pivots' boot lines never printed (J.3.1) | **RED** | 1 | T3 P2 test 1l |
| 78 | `P7-noPneumaticBootLines` | `src/teleop_program.cpp` | the claw's boot line never printed (J.9.1) | **RED** | 1 | T3 P2 test 1l |
| 79 | `P8-noClawRow` | `src/teleop_program.cpp` | the claw row never painted (J.3.3) | **RED** | 1 | T3 P2 test 1l |
| 80 | `P9-noFaultRow` | `src/teleop_program.cpp` | the merged fault row never painted (Station F row 9) | **RED** | 1 | T3 P2 test 1l |
| 81 | `P10-lcdNoClawAbsent` | `src/teleop_program.cpp` | the controller never says CLAW ABSENT | **RED** | 1 | T3 P2 test 1l |
| 82 | `P11-clawRowNeverAmber` | `src/teleop_program.cpp` | the claw row never amber (ABSENT / FREED) | **RED** | 1 | T3 P2 test 1l |
| 83 | `P12-bootLineAlwaysInfo` | `src/teleop_program.cpp` | an ABSENT pivot's boot line at Info, not Warn ([WARN] lost) | **RED** | 1 | T3 P2 test 1l |
| 84 | `P13-exitLineOld` | `src/teleop_program.cpp` | the exit line without its claw clause (F.2.7) | **RED** | 1 | T3 P2 test 1l |
| 85 | `P14-noBootLinesCall` | `src/teleop_program.cpp` | logBootLines() never called (every boot line after the header lost) | **RED** | 1 | T3 P2 test 1l |

#### P2.11.14 Final gates, this round (all in this worktree, 22:42–22:47, after the last edit to any header, source, test or public doc)

A first full run (22:37–22:41, every gate green) was REPEATED because the changelog's two
cross-references were renamed after its doc gates had run (P2.11.11). The script
(`review/gates.sh`, its output `review/gates_out_final.txt`) runs, in order:
- `api_doc_tool.py generate` (wrote `docs/api/all-entities.md`, `mkdocs.yml`; `mechanism_binding.md`
  carries this round's header wording) and `briefing_status.py generate` BEFORE the build;
  `touch test/*.cpp && cmake --build build/test -j8` → exit 0, the Linking line seen, the
  doc-gate target ran: coverage 2679 public entities across 149 headers, all documented; 437
  quoted lines, all verbatim; removability clean; briefing current; staleness audit clean.
- **Suite: `1557 | 1557 passed | 0 failed | 3 skipped`; `2396948 | 2396948 passed | 0 failed`;
  `Status: SUCCESS!`** (P2.9: 1555 / 2,396,896 — this round +2 cases (1n, 6w), +52 assertions;
  from `cc513a7`'s 1533 / 2,393,888, T3 Part 2 is +24 cases, +3,060 assertions).
- `briefing_status.py generate` AGAIN after the binary existed, then `check`: `generated block
  current` (suite 1,557 / 2,396,948; `Next up: G1 PART 2c ...`; honest partials 11).
- `api_doc_tool.py`: `self-test` OK; `check-coverage` all documented; `check-fresh` exit 0;
  `check-examples` 437 verbatim; `check-removability` clean. The manual C7 grep (`internal/`,
  `chunks/`, `RESUMING`, `build-order` over `README.md` and `docs/` outside `docs/internal/`):
  empty.
- `doc_staleness_audit.py self-test`: OK (10 cases); `doc_staleness_audit.py`: clean (27 live docs).
- `src_build_gate.py self-test`: OK (33 detector cases, each against a real build — it plants into
  `src/bench_r3a.cpp` while it runs and restores it byte-exact; `git diff` of that file after: empty);
  `src_build_gate.py check`: **PASS — all 6 builds link, 6/6 TUs each, 0 disallowed** (2 allowed
  `-Wunused-function` in five builds, 0 in xdrive — unchanged from P2.9), hash asserted
  `v0.1.1-335-gcc513a7-dirty`, beacons checked.
- ARM header gate (185 headers, the RESUMING flags): `ARM compile gate: CLEAN`.
- CI greps: `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core
  is sim-free`. `SHULIB_RUNS_` outside `src/main.cpp`: two COMMENT lines only
  (`boot_source.cpp:13`, `teleop_program.cpp:75`, both saying they do not read it), no code.
- ASCII: comments stripped (test 1l's normaliser, in Python), no byte above 0x7F in the code of
  `src/teleop_claws.hpp`, `teleop_program.cpp`, `boot_source.{hpp,cpp}`, `mechanism_table.hpp`,
  or the three changed library headers. This round changed no runtime string.

**This round, in one line:** 9 findings re-checked — all 9 real; 7 fixed in code/tests/docs, 1
(finding 6) documented with the code fix flagged as out of scope, 1 (finding 8) stated everywhere
with the choice left to the team lead; one more stale quote found and fixed (F.2.7); 85 of 85
mutants RED; every gate green. **Not committed. Nothing here has run on a robot.**

---

## The coordinator's merge — T3 Part 2 (2026-09-26, ~22:50–23:15)

**What was merged.** The t3-part2 worktree's tree (the builder's run from 13:33, cut off by the
afternoon's power loss at ~14:35, resumed at 20:20 — P2.4 — then its review round and fixes, P2.11),
as the transport commit `1a86103`, squash-merged onto `a01e3a2` (T3 Part 1, the Part 2 brief, DIAG1
Part A, COMP-a and the G3a plan change on top of `cc513a7`). No code conflict. Documentation, all as
the builder's trial merge predicted (P2.11.11): `docs/changelog.md` (T3 Part 2's section on top,
newest first); `docs/hardware-assumptions.md` (DIAG1's and COMP-a's status sentences kept, then T3
Part 2's "added no entry" sentence); `docs/internal/build-order.md` (the `Next:` block now reads G1
Part 2c, then COMP; T3 Part 2's LANDED sentence inserted before Part 1's, whose "Part 2 is next"
became "Part 2 then wired them in"; the deviations table keeps the G3a and `.shupaths` rows and adds
Part 2's claim-release row); `docs/roadmap.md` ("Now": Part 2's text plus DIAG1's landed sentence);
`docs/api/*` and the briefing regenerated (the briefing again after the suite binary existed).

**Not complete.** A `T3-COMPLETED.md` was drafted and withdrawn before commit: the build-order entry
keeps chassis MACROS and angle presets under T3 (the team lead's rulings kept macros in T3 — the
stick-interrupt rule applies to them), so T3's DoD is not met; only the claws' part is, on the host.

**Verification (the coordinator, on the merged tree).** Suite **1649 | 1649 passed | 0 failed | 3
skipped**, 2,466,733 assertions; `src_build_gate.py` self-test OK (33) and check PASS on all six
builds (0 disallowed, beacons); ARM header gate clean (191 headers); both CI greps empty;
`check-fresh` 0; the briefing current; removability empty; the staleness audit clean. Four hand
mutations aimed at the integration points, each on a freshly linked binary, restored by hash, the
restored tree 1649/1649: the claim release at loop entry dropped → RED (4 cases); the claw's line
handed over at the SAFE level as its start → RED; a claw port that is a drive port no longer refused
→ RED; a pivot's forward/reverse buttons swapped in Teleop's wiring → RED (7 cases). The builder's
own battery: 85 of 85 RED (P2.8, P2.11). Read by the coordinator: the review's lens-1 checks against
the code (no boot twitch, ports, hold on release, disable and controller drop, nothing throws after
driver control starts), the fix report, the claim-release and collision code.

**For the team lead (from the review round, open):** whether the first driver-control start should
keep a claw level autonomous left instead of sending the start level (finding 8); a read-validity
signal so a silent pivot does not show a frozen temperature (finding 6); the reconnect settle window
after J.9.5; an AddressSanitizer CI job. **Nothing of T3 has run on a robot — Station J next.**
