# T4 — the driver program: ONE driver file for robot two, run by the new "shulib Drive" and the competition program's `opcontrol()`; the old drive's protections move into the library; "shulib Teleop" retires

> **THE TEAM LEAD'S ANSWERS (2026-09-27, ~03:40) to the new decisions:** **N7 (c)** — stop driving
> mid-match only when MORE THAN TWO drive motors are dead in total ("Only when 3+ are dead in total");
> **N1 (a)** — the over-current cut stays as ruled, ON by default with one config switch to turn it off,
> and a push check is a required robot test before any match; **N4 (a)** — the Auton Tester takes slot
> 7 until Part 3 retires Teleop, then slot 2; **N5 (a)** — every library program gets an optional SD
> copy of its log. **N2 (a)** as amended (the 5-read frozen rule, the command-floor gate on over-current)
> proceeds as recommended (a technical detection rule; the alternative amends the locked IMotor, F4).
> N3 and N6 are the coordinator's / the brief's, as recommended. SRC1 (`0fab866`) moves `src/` into
> robots/, team/, programs/, common/ before any part is built: the driver file is
> `src/team/robot_two_driver.hpp`, its helpers `src/common/driver_controls.hpp`, the programs under
> `src/programs/` — the coordinator maps every path at SRC1's merge.
>
> **PART 0 LANDED (2026-09-27)** with the coordinator's plan commit after SRC1 (`1237c91`): COMP's B5, B29
> and §3 item 3 name slot 6; the build order's T4 entry, count and deviations rows; G3a's line on the driver
> file's marked region; worksheet Station A's slot map; the roadmap's row; HANDOFF's lines. The auton tester
> (COMP-t) now waits for TANK1 (the team lead, 2026-09-27: "Fix the library first" — `moveTo` on a tank
> steers back onto its line), so T4 Part 1a and TANK1 are the builds in flight, not 1a and COMP-t.

> **Execution brief.** First written 2026-09-27 (00:15–01:30 EDT) from a read-only pass over `shulib-v2` at
> `f6274e1` and the `g1-2c-final` worktree. Re-checked the same night against `37706ce` (G1 Part 2c final
> merged) after an independent checker's nine findings (the 2026-09-26 amendments, kept below). **Amended
> 2026-09-27 (~02:30) to the team lead's rulings R1–R7 (~00:55–01:45)**; see "Team lead's rulings
> applied, 2026-09-27". **Re-checked the same night (~03:30) against a checker's fourteen findings on
> that amendment**; see "Re-check of the rulings amendment". The driver file (§2) and its helper were
> compiled, and run through the PROS shim, at `37706ce` (probe, §0.4). **Part 2 (the protections) is
> designed from the code; only its dead-port rule was probed, as a pure simulation over the REAL
> `coupled_side_monitor.hpp` (the rulings probe, §0.4).** Every claim it rests on is a test in §3 that
> must go red under its mutation.
>
> **Status: the team lead's rulings of 2026-09-27 applied. WAITS for his go on each part at its turn.
> Nothing is built.** The amendments raised seven NEW decisions, N1–N7 (§ Decisions). Each is answered
> before the part that first needs it: N4 before Part 0; N1, N2, N3 and N7 before Part 2; N5 and N6
> before Part 3.
>
> **The parts, in the ruled order (R2).** Each is its own go, executor run, verification, documentation
> pass and commit. Sizes are estimates from the pace of T3 Part 2 and COMP-a, not measurements.
>
> | # | Part | What | Lands | Size |
> |---|---|---|---|---|
> | 0 | **the slots and the plan** | docs only: COMP moves to slot 6, the final slot map, T4's plan entry and its deviations rows | before COMP-b | ~1–2 h (coordinator) |
> | 1a | **the driver file** | `src/team/robot_two_driver.hpp` + `src/common/driver_controls.hpp`, tied at compile time to the table's buttons; no T3 rework, no new build | before COMP-b | ~½ day |
> | — | *COMP-b (not T4's)* | *its `opcontrol()` calls the driver file, written once (B11 as Part 1a replaces it)* | | |
> | 1b | **one owner** | the buttons leave `src/robots/mechanism_table.hpp`; "shulib Teleop" runs the driver file until Part 3 retires it | after COMP-b | ~1 day |
> | 2 | **the drive protections, in the library** | the old drive's mid-match absence rule and its two re-arming cuts, applied inside `teleop::runDriverControl` for every program, with a dead port's frozen readings kept out of both cuts; host-tested on the plant's member faults and on hand-stepped reads | after 1b, BEFORE 3 | ~1½–2 days (the ruling said ~½ day; see Part 2's note) |
> | 3 | **the new "shulib Drive"; "shulib Teleop" retired** | `src/programs/drive_program.*`'s content REPLACED by the thin program around the driver file (slot 1, `PROGRAM=drive`, beacon `tank-drive`); Teleop's diagnostics become its DETAILS page; `PROGRAM=library` removed; every remaining word naming Teleop or slot 2 swept from `src/` and `include/shulib/` | after 2 | ~2 days (~1½ without N5 (a)) |
>
> **Executor model: `opus`. Do not commit.** Live log: `docs/internal/chunks/T4-PROGRESS.md`, **created
> FIRST** and appended as you go, with a heading per part. **Hardware Assumptions numbers: HA-175 and
> HA-176 ONLY.** N3 recommends re-assigning HA-175 from the program-name belief, which R3 made moot, to
> the mid-match absence rule. HA-176 is the over-current cut. HA-177/178 are COMP-t's (the Auton Tester),
> not T4's. Take no other number: an executor who finds another hardware belief stops and tells the
> coordinator. The team lead authorized the coordinator to write this competition code (2026-09-26). VEX's
> rule is that students can explain and defend every line. So the driver file is given VERBATIM in §2 and
> the executor writes it as shown. Any change the tree forces is logged with its reason, and every
> document quotes the result, never the proposal.
>
> **Paths are SRC1's layout (`1237c91`, 2026-09-27).** The coordinator mapped every `src/` path in this
> brief at SRC1's merge: the driver file is `src/team/robot_two_driver.hpp`; `driver_controls.hpp`,
> `driver_panel.hpp` and `boot_source.*` are under `src/common/`; the tables under `src/robots/`; every
> program (the competition program included) under `src/programs/`. Includes are written from `src/`
> (`#include "common/driver_controls.hpp"`), which the gate's `[include-root]` rule enforces; a string
> that names a `src/` file must name one that exists (`[printed-path]`). **Line numbers are still
> `37706ce`'s** (SRC1 moved files and rewrote include lines, it changed no line count) — re-check each
> against the tree. The §2 driver file was probed at `37706ce` with the flat includes; the executor
> compiles it with the mapped ones.

---

## Re-check of the rulings amendment, 2026-09-27 (~03:30) — amendments

A checker returned fourteen findings on the rulings amendment below. Each was re-verified in the tree at
`37706ce`. The first two were also run through a probe written for them (the rulings probe, §0.4). **All
fourteen are real, and each is fixed at the root.** The rulings R1–R7 are unchanged.

1. **(major) Part 2's absence rule lost the race to the fight monitor — REAL, redesigned (N2 amended).**
   - **Verified.** `CoupledSideMonitor::update` counts a PRESENT dead member in `fastestRadS` (:113-118),
     so a member frozen at +v keeps its side judged by itself. After a reversal it is opposite from the
     reversal tick (:127-128). The plant degrades from the NEXT synthesis (`drive_plant.hpp` :397-400), so
     the first frozen read is the live one.
   - **Probed, with the REAL monitor.**
     - P2-6 on the plant CUTS at the reversal + 24, and latches ABSENT one tick later.
     - A port that dies one tick after a reversal, on hardware's last-good screen, does the same.
     - A port that dies while turning, then a released stick and a 3 V reversal on a lagging train: ten
       cuts in 13 s, and the port is never marked.
   - **A fourth failure the checker did not name.** A LIVE train that hits a wall at full stick, with a
     velocity report lagging 80 ms, had a live member latched ABSENT. The positions stop, while the last
     command and the lagging velocities still say "turning".
   - **Fixed at the root.** N2 (a) is now a rule SHORTER than the monitor's that does not need the mates to
     turn: a member whose position repeats for 5 reads while it, or a moving mate, reports motion is
     FROZEN. It leaves the monitor and the over-current streak that tick, and is ABSENT 25 ticks later.
     Under it the probe's scenarios give zero cuts for every death, a real fight is still cut at tick 25,
     and no live member is ever marked. Each of five named mutations goes red.
   - **Tests.** P2-6 is kept exactly as specified, with a new expectation. P2-6b (the order), P2-6c (the
     finding's test (a): a reversal at the re-arm), P2-6d (its test (b): a deceleration through zero,
     hand-stepped) and P2-19 (the wall) are added. Landmine 16 is rewritten.
2. **(major) A port's frozen current cut the drive forever — REAL, fixed inside N2.**
   - **Verified.** `hal/pros/motor.hpp` :250-258 holds `lastCurrent_` on PROS_ERR, and Part 2's
     over-current rule counted every present, not-absent member.
   - **Probed.** A member that dies at 2.5 A in a stalled push, a 2 s rest, then driving away: four cuts
     (the push's own, then one every 1.25 s at rest, then more) before it is marked.
   - **The old program avoided it.** It marked the member within 250 ms from faulted reads, with no motion
     gate (:495), and skipped absent members' current (:558-561).
   - **Fixed.** The over-current streak skips a FROZEN member (N2). It counts only while the member's side
     is commanded above the monitor's 1 V command floor. The probe then gives one cut, the push's own.
   - **Raised** inside N2, the team lead's, before Part 2. Test P2-4b; mutations "a FROZEN member's
     current counted" and "over-current counted at or below 1 V", each red in the probe.
3. **(major) The mid-match REFUSE stops robot two at the SECOND dead port on one side — REAL.**
   - **Verified.** Robot two's record is four per side: worksheet F.1's `4/4 present` rows (:569-570) and
     F.2.1's eight drive ports (:625). The floor is `drivetrain_degradation.hpp` :46, and its header
     :31-32 says "3/4 drives, 2/4 refuses".
   - **What was wrong.** HA-175's blast radius said "a third death on a side … until the field disables
     it". That is wrong on the count and on the duration: P2-7 re-applies the REFUSE at every later entry.
     §2's paragraph had the same duration error.
   - **Fixed.** **N7** is the team lead's decision, needed before Part 2. It gives three options and each
     one's cost on four per side, and RECOMMENDS (c). HA-175's blast radius, §2's paragraph, P2-7 and P2-18
     are written per option, with the real duration: the rest of the run.
4. **(major) Runtime words naming Teleop and slot 2 outside the deleted files — REAL.**
   - **Verified.** `calibrate_program.cpp` :896-897 and `xdrive_program.cpp` :174; no test, document or
     README quotes either (grep). SAFE-SAVE A's `formatSaveReport` names Teleop in the library (SAFE-SAVE
     :612-642), and B1 lands before any Calibrate rerun (NOTES :73), so almost certainly before Part 3.
   - **Fixed.** Part 3 item 3 rewords both strings, or B1's replacement, and `formatSaveReport`'s sentences
     if they have landed. §5 allows that library TEXT change. Landmine 25 names it. Part 3's prose review
     greps `src/` and `include/shulib/` too.
5. **(major) R5 misquoted, and its four rows put on MAIN by default — REAL.**
   - **Fixed.** R5 is quoted verbatim and applied. The four rows are on DETAILS, and "shulib Drive"'s MAIN
     keeps the header bar, the PROTECTION row, the controller row and the DETAILS button. D4's "open to
     you" paragraph is gone.
   - **COMP's page.** COMP-b composes its page from the same pure rows (B11 as replaced); R5 governs the
     driver program.
   - **The claw row's safety argument, restated honestly.** On the field the driver reads the controller,
     not the brain, so a brain row was never the driver's cut-off. On the bench, DETAILS is one touch away.
6. **(minor) SAFE-SAVE B2's `BACKUP see brain` missing from the ONE shared chooser — REAL.**
   - It is added at B2's place: … > CLAW ABSENT > BACKUP > ok, in D4 and P3-T2 (pinned there if B2 has
     landed).
   - B2 writes it into the shared chooser, never `teleop_program.cpp`. Landmine 25, §8 and Part 0's HANDOFF
     line tell B2's re-check.
7. **(minor) N1 was scheduled after the default it decides goes live — REAL.**
   - N1 is now needed before Part 2.
   - If it is unanswered at Part 2's go, Part 2 ships `overCurrentCuts = true`, the cut R4 ruled. The
     REQUIRED push check is then written into Station K, the first station that drives COMP's
     `opcontrol()` with the cut.
8. **(minor) Five named mutations survived — REAL, all five fixed.** Each has its mutation row:
   - T2c: a table with the claw starting OPEN and safe CLOSED;
   - P2-1: every member at about 1 A, so a side's total is above 2.4 A;
   - P2-3 and one P2-4 case on the RIGHT group;
   - P2-20: the config's entry check;
   - `holdFor`/`toggleFor` asserted in T1/T2, and `anyReleasedLeftoverClaim()` in T9.
9. **(minor) The G3a line put the re-zero in WHICH BUTTON — REAL.**
   - Part 0's G3a line now says the region holds `kHolds` and `kToggles`.
   - Where the re-zero is generated is G3a's to decide (R1). Today it is `kLoop.rezeroButton`, under THE
     FEEL, its one owner since the 2026-09-26 re-check.
   - §2's code does not change.
10. **(minor) Gate cases 12–13, re-pointed, would copy 10–11 — REAL** (`src_build_gate.py` :1372-1419).
    - 12–13 are RETIRED, with a logged mapping to 10–11, and the new retirement pair stands in their place.
    - 16, 19's define plants, 21, 24, 26 and 27's two plants are re-pointed. A re-pointed plant that would
      copy an existing case is retired the same way.
    - N6, Part 3 item 3 and P3-T1 say so.
11. **(minor) Part 3's list of comments naming Teleop or the old Drive was incomplete — REAL.**
    - Verified by grep: every line the checker named, plus `coupled_side_monitor.hpp` :171 (`tankSideVolts`,
      which goes false only at Part 3).
    - All are listed in §4 Part 3. The grep's terms and scope are the rule, and every hit is accounted for
      in the log.
12. **(minor) F.5 compared the feel against programs Part 3 deletes — REAL.**
    - F.5 now compares against Bench Tests' 10 DRIVE (Station D), the raw-volts baseline that survives, with
      its differences stated. Guide 14's update names it.
    - Rejected: scheduling it before Part 3. The old Drive and a driver-file Teleop drive toward OPPOSITE
      ends: the back-to-back hazard the worksheet already removed (F.2.1).
13. **(minor) N5 (a) was underspecified and contradicted P3-T3 — REAL.**
    - N5 (a) now names the pieces: PROS-free `hal::TeeCharSink` and `hal::BlockCharSink`, host-tested with
      the existing `FakeCharSink`/`FakeBlockSink`.
    - The provider owns them and one `ProsBlockSink`, given a bare file name.
    - P3-T3's pin is reworded to match.
14. **(minor) HA-172, the `[BLD]` tell, and Part 0's HA entries — REAL, all three.**
    - HA-172 is R5a's reserved, unissued number; the register ends at HA-161. N1 now says "R5a's reserved
      HA-172 (once registered)", and HA-176's register text drops the reference.
    - Landmine 6 says why `[BLD]` is not a tell: `main.cpp` :131-132 prints the same line in every build
      since `37706ce`.
    - Part 0 says R2's HA entries are none, after N3.

## Team lead's rulings applied, 2026-09-27 — amendments

What changed in this brief, ruling by ruling. The team lead's words are in § Decisions.

1. **R1 (D1 RULED: (a), with G3a path (i)).** The driver file owns which button does what, and the table
   keeps what the robot IS. The decision text is replaced by the ruling. G3a's build-order entry is amended
   in **Part 0** (it was Part 1a) to say that the robot panel writes a MARKED REGION inside the driver
   file, not the tables. Parts 1a and 1b are unchanged in substance.
2. **R2 (D2 RULED: (iv)).** The order is Part 0 → Part 1a → COMP-b → Part 1b → **Part 2 (the protections,
   NEW)** → **Part 3 (the replacement, formerly "Part 2 — the driver program")**. Every cross-reference
   is renumbered.
3. **R3 (D3 RULED: replace "shulib Drive" now).** There is no eighth build and no new name.
   - "shulib Sticks", `PROGRAM=sticks`, `tank-sticks`, `STK` and `src/programs/sticks_program.*` are dropped
     everywhere.
   - The new program IS "shulib Drive": slot 1, `PROGRAM=drive`, beacon `shulib-robot-variant=tank-drive`,
     `Program::Drive`, and the entry names `driveInitialize`/`runDrive` (it adds
     `driveAutonomous`/`kDriveAutonomousReason`).
   - `src/programs/drive_program.{hpp,cpp}` keep their names and their content is REPLACED. Part 3 item 2 lists
     exactly what is deleted and what moves.
   - The one-letter hazard is gone, and so are the required L.1 banner check and landmine 6's (a) branch.
     A new hazard takes their place. A brain that still holds the OLD package has the same name, slot and
     beacon, but drives toward the OTHER end. Landmine 6 and Station F.0 cover it.
   - HA-175 is re-judged in N3.
4. **R4 (NEW) — Part 2.** The old drive's protections move into the library, host-tested on the plant, and
   land before Part 3. §0.5 says what the library covers and lacks, §1 Part 2 designs the change, and §3
   tests it. N1, N2, N3 and N7 are the decisions it raised (N2 amended and N7 added at the re-check).
5. **R5 (NEW) — Teleop retires in Part 3's change.**
   - Its diagnostics become the new program's DETAILS page, and Station J moves to the new program.
   - `PROGRAM=library` is REMOVED, not re-pointed (N6, with the build-gate consequences).
   - **D6 is SUPERSEDED.** Teleop runs the driver file from Part 1b only until Part 3.
6. **R6 — the final slots.** 1 "shulib Drive" (new), 2 "shulib Auton Tester" (COMP-t, a separate chunk
   built beside Part 1a), 3 Bench Tests, 4 shulib Calibrate, 5 held for the team's old-code fallback,
   6 "shulib Competition", 7–8 free.
   - Every slot mention is amended to this map.
   - N4 covers slot 2 in the days between COMP-t landing and Teleop retiring.
   - T4 creates NOTHING of COMP-t's: no students' auton file, no tester program, no tester build or slot
     text.
7. **R7 — the robot console, later.** T4 prepares for it without building it:
   - The panel's rows and the controller-screen lines come from PURE, PROS-free functions in ONE shared
     header, host-tested. The program only paints them.
   - A page flip takes a `ui::touch_panel` touch sample, so a later remote touch is just another source.
   - The console itself is not T4's (§5).
8. **Unchanged, and going ahead as recommended (no objection was received).**
   - D4: a minimal honest panel lives in the program, never in the driver file. In "shulib Drive" it is
     now a MAIN page (header, PROTECTION, controller, DETAILS button) and R5's DETAILS page (R5's four
     rows and the rest of Teleop's panel). COMP composes its own page from the same pure rows (re-check
     item 5).
   - D5: the shared `src/common/boot_source` is used now; L9 Part B's library boot comes later.
9. **Found while applying the rulings.**
   - SRC1's path move (header).
   - COMP-t holds HA-177/178.
   - SAFE-SAVE B2 and Part 3 both edit where Teleop's words live (landmine 25).
   - Part 2's honest size.
   - A per-tick read cost that is unmeasured on the brain (landmine 26).

## Re-check 2026-09-26 (late evening) — amendments

*Kept as the record. Where an item conflicts with the rulings above, the rulings win: item 4's D2 text, and
item 9's "shulib Sticks" (superseded by R3).*

Each finding re-verified in the tree at `37706ce`; all nine are real and fixed at the root. The v2 probe
(§0.4) is the evidence for the redesign.

1. **(major) "The claw does not move when driver control starts" — REAL, fixed.** `mechanism_binding.hpp`
   ~:98-102 sends the initial level at the first entry whatever autonomous left, which is the ruling
   ("starts closed — whatever autonomous left it at", status board ~:597). The driver file now says so and
   cites it. The toggle's initial level is read from the Robot's CONFIG start level, not `commanded()`,
   so bind() no longer depends on call order (landmine 3). New test T2b pins the ruling; three T3
   comments carrying the old claim are corrected in 1a.
2. **(major) Test-plan gaps — REAL, all five fixed.**
   - T4b: an absent claw, through a FakeDeviceProvider.
   - T7 split so each heading term flips alone.
   - The compile mutation is now "port 7's line removed", which fails ONLY `controlsCoverTable`. An empty
     control array is not expressible; a robot with none passes `{}` (stated).
   - The re-zero tie is removed at the root: `kLoop.rezeroButton` is its ONE owner, and the checks read
     it, so `kRezero` is gone.
   - The tick is compile-tied. Per-run feel and the constant heading are marked EQUIVALENT while
     field-centric is off, with a field-centric test owed.
3. **(major) Half the driver file was plumbing, and "the one rule" was untrue — REAL, redesigned.** The
   binder, `driverHeading()` and the predicates now live in `src/common/driver_controls.hpp` (PROS-free,
   host-tested once). The driver file keeps only:
   - the controls, the feel and the loop constants;
   - a one-line `bind()`;
   - `run()` as the library call itself.

   It uses `12_volt` and `10_ms`. Teleop and the X-drive switch to the one heading rule in 1a, and a
   source pin proves no copy is left.
4. **(major) D1/D2 missing the off-critical-path option, and the G3a conflict understated — REAL.**
   - D2 gains (iv), now RECOMMENDED: Part 1a (the final driver file, its buttons tied at compile time to
     the table's) → COMP-b → Part 1b → Part 2. This is better than the checker's table-read variant,
     because the file and its public quote are final from their first commit.
   - D1 now states plainly that (a) reverses G3a's `controls` generation target. It offers "a generated
     region inside the driver file" and asks for an explicit confirmation.
5. **(minor) Stale against `37706ce` — REAL.**
   - Re-anchored throughout.
   - B29 is amended alongside B5 and B11.
   - The teleop_loop.hpp banner item is reduced to "and the driver file".
   - The mechanism-table lines are updated (`kTeleopRezeroButton` :159, entry fields :183-185 and :211,
     asserts :568-582).
6. **(minor) Slot and HA-175 timed after COMP needs them — REAL.** The slot fix and HA-175 move to a
   docs-only Part 0 that lands before COMP-b in every order. Station A gains the slot 5 hold.
7. **(minor) Bind by name lost T3's refusal (test 1i) — REAL, fixed.** `DriverBindings::bind()` resolves
   each name against the Robot's CONFIG. It refuses these three cases by name:
   - a name the robot does not declare;
   - a name of the wrong kind;
   - a declared mechanism that no control names.

   It skips only a mechanism that is declared but not built. The probe saw the first two refusals. T10
   keeps 1i's intent.
8. **(minor) Part 1's lists had gaps — REAL, all six fixed.**
   - The storage rule is reworded.
   - `modeWord(kFeel.mode)` is added.
   - The lines that are not host-compiled get a check that no string literal changed, plus a
     serial-capture comparison.
   - Part 2 prints no slot-2 banner sentence.
   - §8 re-points T3's macros and presets and the parked draft.
   - Guide 07 :130 and HA-153's source text are listed, along with boot_source.hpp :92-94.
9. **(minor) D3 kept a one-letter hazard — REAL.** D3 now RECOMMENDS (c): a name that does not start with
   "shulib Drive" (proposed "shulib Sticks", `PROGRAM=sticks`). If the team lead keeps (a), the banner
   check at Station L.1 becomes REQUIRED before the wheels touch the ground. *(Superseded by R3.)*

---

## Decisions

### RULED by the team lead (2026-09-27, ~00:55–01:45)

**D1 → R1. RULED (a), reached in two steps with a compile-time tie between them, and G3a path (i).**
- **The ruling.** "The driver file" owns which button does what.
  - The buttons and volts live in `src/team/robot_two_driver.hpp`.
  - `src/robots/mechanism_table.hpp` keeps what the robot IS: the ports, the cartridges, Hold, the polarity, and
    the start and safe positions.
- **G3a.** G3a's robot panel writes a MARKED REGION INSIDE the driver file, not the table.
  - The region is the WHICH BUTTON block. It is freshness-gated like `check-examples`, and a hand edit
    inside it fails the gate with "edit the robot file or the panel".
  - Whether the region also covers the feel and the loop is G3a's to decide.
  - The C++ checks stay the final authority.
- **The cost it carries.** Part 1b re-points just-landed T3 code (~1 day). Declined, for the record:
  (b) the table keeps the buttons; (c) both, tied for good.

**D2 → R2. RULED (iv), with the tester running beside Part 1a (R6).**
- **The order.** Part 0 (docs only) → Part 1a (the driver file + `src/common/driver_controls.hpp`) → COMP-b
  (its `opcontrol()` calls the driver file, written once) → Part 1b (the buttons move from the table into
  the driver file; Teleop runs the driver file) → **Part 2** (the protections) → **Part 3** (the
  replacement).
- **The costs it carries.**
  - The first auton waits about ½ day more than it would with COMP-b first.
  - For about 1–2 days the buttons are stated twice, tied at compile time.
  - Station K, not J, is the driver file's first run on the robot.
  - **Until Part 2 lands, the competition program's `opcontrol()` drives with no mid-match absence rule
    and no cuts.** MOTOR_GROUP_DISAGREE is still raised. The OLD "shulib Drive" in slot 1 keeps its
    protections until Part 3 replaces it, so at no moment is the only protected program gone.

**D3 → R3 and R6. RULED: replace "shulib Drive" now; the slots final.**
- **The question.** The team lead asked: "why cant we just replace it? that shulib drive is just the old one
  no?"
- **The choice.** "Retire shulib Drive now". The new driver program IS "shulib Drive": the same name,
  slot 1, the SAME build switch `PROGRAM=drive` and variant beacon (`tank-drive`). There is no eighth build
  and no new name.
- **What the old program takes with it.**
  - It is the only shulib program known to have driven robot two (2026-09-17, his report).
  - It is the only one that drives with NO calibration record. The new one refuses without a record, as
    Teleop does, and the slot 5 fallback is the team's no-record answer.
  - Its open-loop feel. Ruling M4's raw-volts baseline survives in Bench Tests' 10 DRIVE (Station D).
  - Its SD text log (N5).
  - Its protections MOVE; they are not lost (R4, Part 2).

| Slot | Now (`37706ce`) | Until T4 Part 3 | Final (R6) |
|---|---|---|---|
| 1 | "shulib Drive": the OLD program (typed table; it drives toward the 15/16 end) | the same OLD program | **"shulib Drive": the NEW driver program** (it drives toward the record's front, the camera end) |
| 2 | "shulib Teleop" | "shulib Teleop" (runs the driver file from Part 1b) | **"shulib Auton Tester"** (COMP-t) |
| 3 | "Bench Tests" | the same | the same |
| 4 | "shulib Calibrate" | the same | the same |
| 5 | HELD: the team's old-code fallback | the same | the same |
| 6 | — | **"shulib Competition"** (COMP-b; B5/B29 amended in Part 0) | the same |
| 7 | — | "shulib Auton Tester" if N4 (a) | free |
| 8 | — | free | free |

**R4 (NEW). RULED: carry the old drive's protections into the library.**
- **His words.** "Moved into the library, so the competition program's opcontrol() and Teleop get them
  too, not just this program: the mid-match absence rule and the two re-arming cuts. With tests on the host
  test plant. Adds about half a day to T4, after the driver file itself, so the auton is not delayed
  further."
- **What they rest on.**
  - The degradation rule, his words of 2026-09-10: "Some ports might die during comp; 1-2 dead ports
    shouldn't stop driving, just let it be a warning that gets logged".
  - The two cuts, as `src/programs/drive_program.cpp` :27-36 states them. FIGHT: a member opposite its side's
    command above 0.5 rad/s, or under 25 % of the side's fastest, while the side is commanded above 1 V and
    moving above 1 rad/s, for 250 ms. OVER-CURRENT: any member above 2.4 A for 250 ms. On either, both
    sides go to 0 V for 1 s; the cut is logged and counted, then the protection re-arms.
- **Where it is designed.** Part 2 (§1); what the library already covers is §0.5. **Honest size:** ~1½–2
  days, not ½. The ½-day estimate did not see four things:
  - MotorGroup has no runtime absence today (its own header calls that "a later chunk").
  - A log line cannot name a port without group labels.
  - Without ProsMotor's faulted-read count (F4), a dead port is seen only by its FROZEN readings. They
    must leave the fight monitor and the over-current streak faster than the monitor's own 250 ms, or
    they cut the drive (N2, amended at the re-check; the rulings probe).
  - That order has to be proved, and so does "a live train at a wall marks nobody".

  Part 2 stays after the driver file and before Part 3, so the auton is not delayed by it.

**R5 (NEW). RULED: retire "shulib Teleop" when the driver program lands.**
- **The question.** The team lead asked: "why do we still need teleop ... if we have everything that we
  need with calibration bench test, driver program and the auton tester?"
- **The choice.** "Retire it when the driver program lands".
- **The ruling, verbatim (2026-09-27 ~01:40):** "its useful diagnostics (the fault row, the claw row, where
  the drivetrain numbers came from, the run status) become a DETAILS page in the new driver program".
- **What it means.**
  - Those four rows are on "shulib Drive"'s DETAILS page (D4). Its MAIN page keeps only the header bar,
    the PROTECTION row, the controller row and the DETAILS button.
  - Part 3 retires Teleop in the same change (N6: `PROGRAM=library` REMOVED).
  - Station J (the claw test) moves to the new program.
  - Until then Teleop keeps working, and Part 1b moves it onto the driver file so its buttons never have
    two owners.

**R6. RULED: the slots are final** (the table above). The team lead added:
- "shulib Auton Tester" is a NEW, separate chunk (COMP-t), built in parallel with T4 Part 1a.
- It runs a registered auton on the bench through the same `competition::runAuton` path: touch RUN, a
  3 s countdown, and any button aborts.
- T4 must NOT create the students' auton file or anything of the tester's.

**R7. NOTED: the robot console, planned AFTER G3a, on H1's wire.**
- **His words (relayed 2026-09-27):** "it would be nice to have the logging be visible and the whatever
  else on the brain screen not only visible on the brain screen but be able to see it on the computer, it
  would make things very much easier maybe even you can interact with the brain screen from the computer
  so you dont even have to be by the robot you can just set it down and plug in the micro usb, is there a
  way we can design in create that after the panel/vs code extension for creating robots?"
- **What it asks of T4.** The panel and DETAILS page must not preclude streaming the screen rows later. So:
  - every row and LCD line is produced by pure functions a mirror can reuse;
  - the log already goes through the one telemetry sink;
  - the page flip takes its touch as data.
- The console chunk is the coordinator's plan entry, not T4's.

**D4 — going ahead as recommended (no objection), with R5 and R7 applied.**
- **Where the code lives.** It lives in the program and in one shared PROS-free panel header, never in the
  driver file. That header is COMP-b's shared file (B11 as Part 1a replaces it); proposed name
  `src/common/driver_panel.hpp`. It holds pure ROW functions and the controller-screen chooser. Each program
  composes its own page from them, so a page layout is a list, never a copy.
- **"shulib Drive"'s MAIN, the driver's page (R5 applied):**
  - the header bar: `shulib DRIVE` + `BUILD hh:mm:ss`;
  - **the PROTECTION row** (Part 2): `drive OK`, or `DEGRADED dead 13 18`, or `CUT 1 s: FIGHT LEFT port 13
    (#2)`, or the REFUSE's words under N7 (for example `REFUSED: 3 drive motors dead`);
  - the controller row (`NO CONTROLLER -- the drive gets 0 V`);
  - a `DETAILS` touch button.
- **DETAILS (R5: "a DETAILS page in the new driver program").**
  - R5's four rows: the run status; the two SOURCE rows (where the drivetrain numbers came from); the CLAW
    row with the pivot temperatures; the ONE fault row.
  - The rest of Teleop's panel, so nothing Teleop showed is lost: cmd, est, the driver mode, the L and R
    group rows (present/members, disagreeing, and the ports dead since boot), the heading cross-check with
    the battery, and the travel.
  - The protections' counters: cuts, the last cut's words, each side's peak current this entry.
  - A `BACK` button.

  If that does not fit one page, DETAILS gets a second page (`MORE`), with R5's four on its first.
  Stations F.2.5 and G.6.3 read `est` here.
- **The claw row, off MAIN.** The first proposal kept it on MAIN as "safety, not diagnostics", because the
  driver is the only cut-off for a pivot held against its stop (HA-154). That argument fails once you ask
  who reads the brain. On the field the driver reads the CONTROLLER, so a brain row was never the driver's
  cut-off; on the bench, DETAILS is one touch away. The controller's `CLAW ABSENT` word stays in the
  chooser.
- **COMP's `opcontrol()` page** is COMP-b's, composed from the same pure rows (B11 as Part 1a replaces
  it): the header, the claw row, the fault row, the controller row, and the PROTECTION row once Part 2
  lands. R5 governs the driver program; it does not move COMP's rows.
- **The controller screen** (shared by both programs):
  - row 0: `DRIVE B12.9V`;
  - row 1: the dead ports (`dead 13 18`), or empty;
  - row 2: ONE shared chooser, in order of urgency: `REFUSED see brain` > `CUT see brain` > the last
    fault by name > `no controller 0V` > `DEGRADED see brain` > `CLAW ABSENT` > `BACKUP see brain` >
    `ok`.
    - `BACKUP see brain` is SAFE-SAVE B2's word, at the place B2 gave it (after CLAW ABSENT). B2 adds it to
      THIS shared chooser once COMP-b's header exists, never to `teleop_program.cpp` (landmine 25).
    - Under N7 (b) there is no `REFUSED`.

  A state in force outranks a latched history, so a cut is never hidden behind an old fault.
- **The serial log.** The boot decisions, the session header, the run status, one line per claw, the
  driver line (generated from the driver file's constants), the protections line (generated from
  `DriverLoopConfig::protection`, never retyped), and the live and disable lines. There is NO banner claw
  sentence.

**D5 — going ahead as recommended.** The driver FILE never boots. The program uses the shared
`src/common/boot_source`, as Teleop and COMP do, and L9 Part B's one-line boot changes only the programs' boot
lines, later.

**D6 — SUPERSEDED by R5.** Teleop runs the driver file from Part 1b until Part 3 retires it.

### NEW decisions the amendments raised (a recommendation for each, and the real cost of every option)

**N1 — The over-current cut during a push (the team lead's; needed before Part 2, which ships the default
into the competition program's `opcontrol()`). RECOMMENDED (a): as ruled, plus a REQUIRED push check
before the competition program drives a match, and one config field to change it.**
- **What happens in a push.** A V5 motor pushing a wall or a robot sits at its current limit, which is the
  PROS default of 2500 mA; nothing in shulib changes it (HA-154). That is above 2.4 A, so the cut fires
  after 250 ms and repeats every 1.25 s while the push lasts.
- **Why the old program had it.** It is a bench tool on a robot with typed signs. A wrong sign LOCKS the
  train, and nothing moves. The fight monitor is judged only while the side turns above 1 rad/s
  (`coupled_side_monitor.hpp` :117-118), so it cannot see that locked fight. Only the current can, and in
  current a locked fight looks exactly like a push. From Part 2, this cut runs in the competition program's
  `opcontrol()`, in a match.
- **The other failure.** If VEXos lowers each motor's limit when more than eight are plugged in (the open
  question of R5a's reserved HA-172, once registered; robot two has twelve), 2.4 A may never be reached and
  the cut protects nothing.
- **(a) RECOMMENDED.** Keep it as ruled.
  - Part 2 makes the action ONE field, `DriveProtectionConfig::overCurrentCuts` (default `true`).
  - A REQUIRED push check before the competition program drives a match: on the ground, full stick into a
    wall for 3 s, recording whether it cut and the highest current. **Part 2** writes it into Station K
    (COMP's), the first station that drives the competition program's `opcontrol()` with the cut, reading
    the current from serial's disable line. **Part 3** adds it to Station F.6 (ii), reading DETAILS.
  - You decide with that number. **Cost:** until the check, a push on match day may cut.
- **If you have not answered by Part 2's go,** Part 2 ships (a)'s default, `overCurrentCuts = true`: the
  cut R4 ruled, with the push check in Station K. Answering later changes one field.
- **(b)** Over-current WARNS in every program: logged, counted and shown, but it never cuts. The fight cut
  stays. **Cost:** a locked fight is caught only by MOTOR_OVER_TEMP (55 °C, HA-44), minutes later. The
  record's MEASURED signs are then all that stands between robot two and that fight.
- **(c)** Warn in the competition program and cut in "shulib Drive". **Cost:** the one driver file would
  behave differently per program. That needs a per-program argument through `run()`, which is the
  duplication R1 ruled out.
- **(d)** A longer window, for example 2 s. **Cost:** one guess replaces another, and a real locked fight
  strips gears in less time than that.

**N2 — How the library sees a port die mid-match, and keeps its frozen readings out of BOTH cuts (the team
lead's: F4 is locked; needed before Part 2). RECOMMENDED (a), amended at the re-check.**
- **How the old program sees it.** `ProsMotor::faultedReads()` advances on every tick for 250 ms, with no
  motion gate (`coupled_side_monitor.hpp` :189-191, :214-218; `drive_program.cpp` :495). Then it skips the
  absent member's current (:558-561).
- **Why the library cannot do the same.** The library holds its drive motors as `hal::IMotor*`, and IMotor
  has NO faulted-read count: "this seam has no validity channel" (`hal/motor.hpp` ~:59-66). ProsMotor's
  own doc says `faultedReads()` exists "without this seam growing a validity channel F4 does not have"
  (`hal/pros/motor.hpp` ~:283-290). F4 is LOCKED since 2026-06-19 (`roadmap.md` :138).
- **What a dead port looks like through IMotor.** Its adapter repeats its LAST readings: position,
  velocity AND current (`hal/pros/motor.hpp` :230-258). The plant's `MemberFault::Frozen` models that
  (`sim/drive_plant.hpp` ~:196-197), and a test can hold a `FakeMotor`'s current by hand.
- **Why the first proposal failed (the re-check; the rulings probe, §0.4).** It proposed this: a position
  bit-identical for 25 ticks while the train turned (the last command above 1 V and the mates above
  1 rad/s) is ABSENT.
  - The monitor counts a present member's STALE velocity in its side's fastest
    (`coupled_side_monitor.hpp` :113-118). So a member frozen at +v keeps its side judged by itself, and
    after a reversal it is opposite from the reversal tick.
  - A 25-tick streak that waits for the mates to turn starts later than that. On a real train the mates
    pass below 1 rad/s in every reversal.
  - Probed with the REAL monitor:
    - P2-6 CUT one tick before the absence latched;
    - a death while turning, then a released stick and a slow reversal, gave a 1 s cut every 1.25 s and
      was never marked;
    - a LIVE train that hit a wall at full stick, with a velocity report lagging 80 ms, had a live member
      marked dead.
  - **Any rule as long as the monitor's loses, and so does one that waits for the mates.**
- **(a) RECOMMENDED — a FROZEN member, judged per side through IMotor alone** (a new pure
  `teleop::FrozenReadingDetector`, one per group; Part 2 item 4). Each tick, BEFORE `drive()`:
  - **SUSPECT.** The member's `position()` has been bit-identical for 5 consecutive reads
    (`DriveProtectionConfig::frozenSuspectReads`, HA-175), AND one of these holds:
    - its OWN reported |velocity| is above 0.5 rad/s. The motor says it turns and its encoder does not,
      which a live motor cannot do.
    - a side-mate whose position IS changing reports above 1 rad/s. The train turns and this encoder does
      not.

    The two floors are the group's `SideMonitorConfig` (`oppositeFloorRadS`, `movingFloorRadS`): no new
    threshold.
  - **FROZEN.** SUSPECT while at least one other present member of its side is NOT. A whole side stopping
    at once is every member alike, never a death: a wall, a stall, or a velocity report lagging a stop.
  - **What FROZEN does, from that tick on:**
    - the group's monitor is told `present = false` for it (`MotorGroup::setMonitorExcluded`, Part 2
      item 3), so its stale velocity can neither start a fight streak nor make its mates look stalled;
    - the over-current streak skips it (its stale current);
    - it stays in the group's reads and is still commanded until it is ABSENT.
  - **ABSENT.** FROZEN for 25 consecutive ticks (the monitor's `persistTicks`, 250 ms) latches it for the
    run: `markAbsent`, one Warn line, and the degradation re-judged (N7).
  - **The over-current streak also counts only while the member's side is commanded above the monitor's
    1 V command floor.** On a drive in Coast at 1 V or less there is nothing a cut would stop, and a frozen
    current at rest can no longer cut a standing robot every 1.25 s (the re-check's finding 2).
  - **Why it cannot lose the race.** The monitor can judge a dead member only until its 5th repeated read.
    After that it never judges it again while its stale reading could matter:
    - a stale velocity above 0.5 rad/s makes it SUSPECT by itself;
    - a stale velocity at or below 0.5 rad/s cannot keep its side judged (the monitor needs 1 rad/s);
    - and a side judged by its mates makes it SUSPECT through them.

    `frozenSuspectReads < persistTicks` is a precondition. In the probe the dead member's monitor streak
    peaked at 5 in every death scenario, against the 25 a cut needs.
  - The old "velocity exactly 0" signature stays beside it (`MemberAbsenceDetector`, unchanged).
    Registered, with the new rule, as HA-175 (N3).
  - **Cost:**
    - A LIVE member whose position really repeats for 5 reads while it, or a moving mate, reports above
      those floors leaves the fight monitor for those ticks (harmless). If that lasts 30 reads while a mate
      still answers, it is dropped. That is reasoned impossible on robot two's BLUE cartridge unless its
      data stops arriving, which is what "stops answering" means. BLUE counts 300 per output turn: about
      one count every 42 ms at 0.5 rad/s, and every 21 ms at 1 rad/s. HA-175 settles it.
    - **A whole side that stops answering at once is not seen,** because every member looks alike. The old
      faulted-read count would have seen it. The robot shows it by not driving that side.
    - A stale reading stays in the group's median for up to 250 ms before it is ABSENT (today it stays
      forever).
    - New library API, additive: the detector, `setMonitorExcluded`, and one config field.
- **(b) Amend F4: IMotor gains `faultedReads()`, default 0.** It fixes the race and the frozen current
  directly, because "not answering" would be READ, not inferred. **Cost:** a locked interface reopened.
  Every implementer must answer it (FakeMotor, ProsMotor, MotorGroup, the plant's members), and the freeze
  review is yours.
- **(c) Velocity exactly 0 only** (the old second signature). **Cost:** a port that dies while TURNING
  reads its last good velocity forever, which is not zero, so it is never marked absent. On the driver's
  next reversal it reads as a FIGHT: a 1 s cut every 1.25 s for the rest of the match. Its frozen current
  can do the same through the over-current cut.
- **Withdrawn:** the first proposal (above). The probe shows it cuts before it marks, and marks a live
  train at a wall.

**N3 — HA numbers (the coordinator's; needed before Part 2). RECOMMENDED: re-assign HA-175, use HA-176 as
reserved, and add no third number.**
- **HA-175's old belief is not needed.** R3 removes the belief HA-175 was reserved for: that the brain
  shows a program's upload name whole. Its hazard was "shulib Drive" and "shulib Driver" being cut to one
  prefix. Now:
  - no two of our names share more than "shulib C" (Calibrate and Competition);
  - every document names the slot NUMBER beside the name;
  - Station A keeps its photograph of the program list as a plain step.

  It is not issued.
- **R4 needs one registered belief per protection.**
  - The fight: HA-132/133 already (AMENDED in Part 2).
  - **The mid-match absence rule → HA-175:** N2's FROZEN rule (its 5 reads and its 250 ms latch), the
    velocity-exactly-0 signature, and the floor as N7 rules it. The floor has been unregistered since R3b
    Part 0b, and N7 decides how much of it now applies MID-MATCH.
  - **The over-current cut → HA-176:** 2.4 A for 250 ms, unregistered today.
- **Alternatives.**
  - HA-176 alone for both new beliefs: one entry with two claims whose settle tests differ.
  - Or a third number. HA-177/178 are COMP-t's, so it would be the next free past them; the coordinator
    picks.
- The policy constants (the 1 s cut, the 50-line log bound) are not hardware beliefs and get no entry.

**N4 — Slot 2 between COMP-t landing and Teleop retiring (the team lead's or the coordinator's; needed
BEFORE Part 0, which writes Station A). RECOMMENDED (a).**
- **The conflict.** COMP-t lands beside Part 1a. Teleop keeps working until Part 3 (R5), because Stations
  F, G.6 and J run it. Both programs cannot be in slot 2.
- **(a) RECOMMENDED.** "shulib Auton Tester" uploads to **slot 7** until Part 3. Part 3's upload block
  then moves it to slot 2. **Cost:** the tester's documents name slot 7 for a few days, then slot 2. That
  is docs-only, if COMP-t prints no slot number in any code string; this is asked of its brief.
- **(b)** Teleop moves to slot 7 at once. **Cost:** Teleop's own banner says `THE CLAWS are in THIS
  program (slot 2), NOT shulib Drive (slot 1)`. That is `src/programs/teleop_claws.hpp` :294, pinned by
  `test/teleop_claws_test.cpp` :496 and :791, and quoted at J.3.2. Moving Teleop means either a code change
  to a retiring program's pinned words, or a program that prints a false slot.
- **(c)** The tester takes slot 2 at once, and Teleop is no longer uploaded. **Cost:** Stations F, G.6 and
  J lose their program until Part 3, and the claws have no bench test. Station K is a practice match, not a
  claw test.

**N5 — The old program's SD text log (the team lead's; needed before Part 3). RECOMMENDED (a).**
- **What the old program does.** The old "shulib Drive" writes every line it prints to
  `/usd/drive_log.txt`, one file per boot (`drive_program.cpp` :117-144, :275-276; worksheet E.2.5). A
  practice run with no laptop still leaves the dead ports, every cut and the per-port peaks.
- **What the library programs do.** They log to USB serial only: the provider's sink is a `TermSink` over
  stdout (`hal/pros/device_provider.hpp` ~:373-376). The replacement therefore loses the only SD text log a
  run leaves.
- **(a) RECOMMENDED — keep it, as a copy the PROVIDER owns.** `TermSink` takes ONE `hal::ICharSink&`
  (`diag/term_sink.hpp` :81), and no `ICharSink` writes to the SD card today. `ProsBlockSink` is a BLOCK
  sink (`hal/pros/block_sink.hpp` :71). So (a) is three pieces, all additive (no frozen interface):
  - **`hal::TeeCharSink`** (PROS-free, `include/shulib/hal/`): an `ICharSink` over two `ICharSink&`. Each
    `write()` goes to the first, then the second, whole. It never throws.
  - **`hal::BlockCharSink`** (PROS-free): an `ICharSink` over an `IBlockSink&`. Each `write()` is one
    block `write()` then `flush()`. A refusal is dropped and counted (`droppedWrites()`), never thrown: a
    full or missing card must not stop a drive.
  - **`ProsDeviceProvider`** gains an optional constructor argument, `const char* serialCopyFile`
    (default `nullptr`: today's provider, byte-identical).
    - Given a BARE file name, the provider owns a `ProsBlockSink` on it (truncated at construction: one
      file per boot, as the old `drive_log.txt` was), the `BlockCharSink` over it, and the `TeeCharSink`
      over stdout's `ProsCharSink` and that. Its `TermSink` writes to the tee.
    - A missing card leaves a refusing sink, and serial is unchanged.
  - The two sinks are host-tested with the existing `FakeCharSink`/`FakeBlockSink`; the provider's
    argument through the shim (the copy holds exactly the serial bytes; a refusing card drops and counts).
  - The new "shulib Drive" passes `"drive_log.txt"`; COMP may do the same, at COMP's choice. The copy
    begins when the provider is built. So the banner and the boot-source lines printed before it are on
    serial only, and F.7 says so.
  - It is done in Part 3, about 3 h more. **Cost:** a library change in Part 3, and one SD write + flush
    per log line. Log lines are events only, never one per tick.
- **(b)** Accept the loss now. The robot console (R7) covers the bench, and the field record is COMP's to
  decide. **Cost:** a practice run without a laptop leaves nothing but the screen.

**N6 — `PROGRAM=library`: REMOVED, not re-pointed (R5 asks "say which").** This is the brief's choice,
stated so it can be objected to.
- A re-pointed `PROGRAM=library` would silently build a different program under a retired name. A stale
  command in someone's notes would then upload the wrong thing, without a word.
- Removed, it fails loudly. The Makefile's `$(error)` names the retirement and the right command, and the
  gate keeps a RETIRED list so the define and the beacon cannot come back.
- Part 3 item 3 gives the build-gate consequences:
  - the builds go down by one;
  - self-test cases 12–13 are RETIRED onto 10–11, which already test the same things for `tank-drive`;
  - six cases are re-pointed (16, 19's define plants, 21, 24, 26 and 27's two plants);
  - a new pair proves the retirement.

**N7 — Whether a runtime death may STOP the drive mid-match (the team lead's; needed before Part 2).
RECOMMENDED (c).**
- **What the floor does on robot two.** It refuses a side with fewer than 3 answering, or more than 2 dead
  in total (`drivetrain_degradation.hpp` :46, :50).
  - It was written for five per side, where 3/5 drives, and for the bench bot's four: "3/4 drives, 2/4
    refuses" (:31-32).
  - Robot two's record is FOUR per side (worksheet F.1's `4/4 present`; F.2.1's eight drive ports).
  - So the SECOND death on one side refuses, and so does the third anywhere.
- **What Part 2 would do with it, as first written.** It would carry the old bench program's behaviour
  into every driver-control program. The drive goes to 0 V from that tick, and again at every later
  driver-control start, because the marks live in the groups (P2-7). That is the rest of the run, in a
  match, in the competition program's `opcontrol()`.
- **Your words it has to meet** (2026-09-10): "Some ports might die during comp; 1-2 dead ports shouldn't
  stop driving, just let it be a warning that gets logged". Under the rule as it stands, two deaths on one
  side of robot two stop driving.
- **(a) Refuse mid-match, as the old program did.** **Cost:** on robot two the second death on one side
  ends driving for the rest of the match. So does one real death plus one live member wrongly marked. That
  goes against your words for two deaths on one side.
- **(b) Never refuse mid-match: warn and drive.** DEGRADED is counted and shown, whatever the count. The
  refusal stays a BOOT decision (the boot check, unchanged). **Cost:**
  - a side with half its motors drags the robot into a curve at speed;
  - a systemic failure (a harness, a port bank, a sagging battery) keeps driving, and the driver sees it
    and decides;
  - the `Refused` mode, `REFUSED see brain` and P2-7's refusal leave Part 2.
- **(c) RECOMMENDED — refuse mid-match only on "more than two dead in total".** The per-side floor is not
  applied mid-run; the total cap is.
  - One or two dead, in any arrangement, drive with a warning: your words exactly.
  - The third death is the "systemic problem" the policy's header names (a harness, a port bank, a
    battery), and it stops the drive for the rest of the run.
  - **Cost:** a side at 2/4 drags into a curve, and the driver steers against it. Three deaths, or three
    wrongly marked members, end the match. It needs one new pure function beside the policy,
    `evaluateRuntimeDegradation(left, right)` (the total cap only), so the boot check's rule does not
    change.
- **The boot.** The same rule refuses at BOOT today: 2/4 on one side is refused before the match starts
  (`boot_check.hpp` ~:277). Changing that is not T4's. Say if you want the boot to match.
- **What the choice changes:**
  - Part 2 item 1's re-judgement in `enter()` and `beforeDrive()`, and item 5;
  - P2-7, and §2's protections paragraph with P2-18;
  - HA-175's blast radius;
  - the chooser's `REFUSED see brain` (absent under (b)).

---

## 0. Why

### 0.1 The requests
The team lead, 2026-09-26 ~23:25, verbatim: *"rather than putting the claw and stuff in teleop please make
a driver program and also put anything driver under the official opcontrol as well with of course the fully
finished library code running it so that I can see how the driver control can actually look like"*. His
answers:
1. "do both" — a SEPARATE driver program AND the same driver control under the competition program's
   official `opcontrol()`;
2. keep the claws in "shulib Teleop" as the bench test (superseded by R5 on 2026-09-27: Teleop retires and
   the claws move to the driver program);
3. "a short, readable driver file".

His rulings of 2026-09-27 (§ Decisions) add four things:
- the driver program REPLACES the old "shulib Drive";
- the old drive's protections move into the library for every program;
- "shulib Teleop" retires;
- the slots are final.

### 0.2 The design (the coordinator's principle, checked here)
ONE readable driver file is the team's driver code. Every program that drives robot two for a driver runs
it, so nothing is duplicated: the new "shulib Drive" (slot 1) and the competition program's `opcontrol()`
(slot 6). "shulib Teleop" runs it from Part 1b until Part 3 retires it.
- **What it declares:** each control in one line, the feel and the loop. The feel is Teleop's today:
  the default `DriverProfile`, field-centric off, Coast on release, re-zero on Y.
- **What it runs:** the library's loop, `teleop::runDriverControl`, called in the file itself so the
  reader sees it.
- **What it sits on:** the FINISHED front door — `robot::Robot` over `hal::pros::ProsDeviceProvider`,
  booted from the SD record by the shared boot source — never hand-wired.
- **What is kept out:** everything generic — resolving a control to a mechanism BY NAME, the refusals,
  the storage and the one heading rule. That lives in `src/common/driver_controls.hpp` (PROS-free, host-tested
  once), so a second robot's driver file would copy only its own controls.
- **What protects the drive is the library's** (R4, Part 2): `runDriverControl` applies the old drive's
  mid-match absence rule and its two re-arming cuts for EVERY caller, with no wiring in the driver file.
  The driver file's code does not change when they land; only its header gains a sentence saying what they
  do.
- **Diagnostics** stay OUT of the driver file: they belong to the program's panel, whose rows are pure
  functions (R7).

### 0.3 What exists (every anchor re-read at `37706ce`)
| Piece | Where | What matters |
|---|---|---|
| The loop | `include/shulib/teleop/teleop_loop.hpp` | `DriverLoopConfig{tickPeriod, rezeroButton = L1, releaseBrakeMode = Coast}` :132-151; `HeadingSample` :155; the two-span `runDriverControl(chassis, controller, pacer, shaper, cfg, holds, toggles, disabled, heading, perTick)` :207-310 — `validBindings` as a PRECONDITION before tick 1 (:216-219), a leftover claim released at entry and logged Warn tag `TEL` through the chassis's sink (:223-240), toggles left alone on a disable (:262-270), `drive()` then the release mode (:291-294), `perTick(command, connected, shaper.state())` (:307); the banner already names Teleop AND the X-drive (:92-96) |
| The bindings | `include/shulib/teleop/mechanism_binding.hpp` | the TOGGLE rules :98-118 (initial level sent ONCE in the binding's life, at its FIRST entry); `MechanismBinding` :180, ctor :205-216 (throws on a bad volts); `ToggleBinding` :329, ctor :337-343, `enter()` ~:361-369; `validBindings` :453 |
| The feel | `teleop/driver_control.hpp` (`DriveShaper` :83, ctor `noexcept` :88; `DriveShaperState` :62-75), `input_shaping.hpp` (`DriverProfile` :187, `mode = ArcadeBody` default), `units/literals.hpp` (`_volt`, `_ms` :45-58) | arcade = left stick forward, right stick X yaw, left stick X strafe (a tank ignores it) |
| The robot | `include/shulib/robot/robot.hpp` | absent drive ports decided by the boot check, never by the builder (:69-80); `chassis()` :579; `faults()` :590; `health()` :595; `config()` :606; `motorGroupCount()` :615; `mechanismCount()` :629; `builtMechanismAt(i)` :637; `mechanism(name)` / `pneumaticMechanism(name)` :703/:714 — **null for "not declared", "not built" AND "wrong kind" alike**; `describe()` :745 (its absent list read from the groups' slots, `appendAbsent` :1072-1101); the groups built by `makeSideGroup` ~:922 |
| Teleop today | `src/programs/teleop_program.cpp` | storage rule :64-73; `kRunStatus` :142; `kFieldCentric` :154; `kRezeroButton` :160 + asserts :163-176; `teleopProfile()` :201, `driverLineMatchesProfile()` :211; TermSink's 200 bytes and `logWrapped` :224-249; the boot-line literals :251-278 — **`kDeadPortLine1/2` :258-263 say runtime absence is "AT BOOT ONLY" and "still shulib Drive's (slot 1)"**; the screen and `PanelRow` (twelve rows, full) :284-321; `text::` checks :323-377; `sourceRows()` :392; `paintHeaderBar` :475; `TeleopState` :489-508; `paintRefusal` :520, `refuse` :547; `headingSample()` :559; `qualityName`/`modeWord`/`fieldCentricWord` :567-598; `TeleopPanel` :611-781; `logBootLines` :783-875; `teleopInitialize` :877; `teleopAutonomous` :955; `runTeleop` :965 |
| Teleop's claws | `src/programs/teleop_claws.hpp` | header's "the claw cannot move when driver control starts" (:21-25); `TeleopClaws::bind()` :95-137 (by config INDEX, refuses another table — test 1i); `motorClawBootLine` :236 (ends `-- in shulib Teleop only`, :245); `pneumaticClawBootLine` :260; `clawBannerLine` :285 (`THE CLAWS are in THIS program (slot 2), NOT shulib Drive (slot 1):` :294; pinned `test/teleop_claws_test.cpp` :496, :791); `clawRowText` :385; `kLcdClawAbsent` :445; `faultRowText` :452 |
| The table | `src/robots/mechanism_table.hpp` | `kTeleopRezeroButton` :159; entry fields :183-185 (motor), :211 (toggle); `kRobotTwoMechanisms` :255; `mechanismTableFor()` :311, `kMechanisms` :326; `pneumaticStartCommand()` :498 (its comment :493-497); robot two's asserts :568-582, the build's :589-596, the selection asserts :604-609 |
| The boot | `src/common/boot_source.{hpp,cpp}` | `decideBootSource(tag)` (cpp :34); the header's conversion note :86-95; `toMechanismConfig(pneumatic)` :203-212 sets `pneumaticStart` from `pneumaticStartCommand()` |
| The X-drive | `src/programs/xdrive_program.cpp` | `headingSample()` :237-248, "Written twice, deliberately for now … the merge as owed" |
| **The OLD "shulib Drive"** | `src/programs/drive_program.cpp` (687 lines), `.hpp` (43) | header: the degradation rule and the two non-fatal cuts :17-36, the all-stop guard :43-49; constants and the `SideMonitorConfig` pins :89-108 (`kCurrentLimitA = 2.4` "INVENTED for a first run" :96); the SD text log :117-144, :275-276; `Member`/`stopAll`/`AllStopGuard` :146-176; `deadPortsLine`/`evaluate` :178-201; typed-table refusals :320-336; a `ProsMotor` per port in its own try :338-370; boot degradation :372-391; open-loop `mapSticks`→`tankSideVolts` at 12 V :434-445; the cut and re-arm :447-461; **runtime absence, then the fight, then over-current, in that order** :482-520, :522-555, :556-568; the cut :569-586 (0 V at once, count, `at the cut:` per member, 50-line bound); ten-row panel :589-641; LCD :643-662; exit summary :667-684 |
| The protections' pure pieces | `include/shulib/teleop/coupled_side_monitor.hpp`, `drivetrain_degradation.hpp` | `SideMonitorConfig` :67-73 (HA-132/133); the monitor judges only above both floors and RESETS every streak on a tick it does not judge (:117-125); `MemberAbsenceDetector` :197-252 (faulted-reads signature :214-218, velocity-exactly-0 :221-227, LATCHED); `evaluateDegradation` :81-115 (`kMinAnsweringPerSide = 3` :46, `kMaxDeadTotal = 2` :50) |
| The coupled group | `include/shulib/hal/motor_group.hpp` | the shared monitor inside (:46-60), evaluated ONCE per tick in `evaluateDisagreement()` :244 by the health tick; **absent members are BOOT holes only — "Runtime death INSIDE the library graph — promoting the drive program's MemberAbsenceDetector in here — is a later chunk"** (:82-90); ctor precondition "at least one present" :159-173; `setVoltage` skips holes :179-189; `resetDisagreement` :273; `presentCount`/`absentMask`/`isPresent`/`member` :283-300 (`member(i)` on a hole is a precondition THROW) |
| The health tick | `include/shulib/motion/motion.hpp`, `diag/health_monitor.hpp` | `MotionDeps::motorGroups` :190-195; `tickHealthObservables` :258-277 (every group's disagreement evaluated once; MOTOR_GROUP_DISAGREE raised per EPISODE, `health_monitor.hpp` :241-253, CONTINUE not abort); MOTOR_OVER_TEMP at 55 °C (HA-44, :47-48) |
| The drive path | `include/shulib/chassis/chassis.hpp` `drive()` :385-420; `motion/command_pipeline.hpp` :142-149; `control/feedforward.hpp` :77-80 | `drive()` updates the Localizer FIRST, then the pipeline, then the health tick (so the group monitors are judged INSIDE `drive()`); the pipeline uses the CRUISE feedforward (acceleration 0), so a ZERO command is exactly 0 V |
| The adapter's screen | `include/shulib/hal/motor.hpp` :55-66; `hal/pros/motor.hpp` :74-84, :213-290 | a sentinel read HOLDS the last good position/velocity/current/temperature; `faultedReads()` is ProsMotor's, NOT IMotor's (F4 LOCKED, `roadmap.md` :138) |
| The provider's log | `include/shulib/hal/pros/device_provider.hpp` :373-376 | the telemetry sink is a `TermSink` over USB serial only (N5) |
| The host test plant | `include/shulib/sim/drive_plant.hpp`; `hal/fake/fake_motor.hpp` :39; `test/robot_builder_test.cpp` | `MemberFault::{Frozen, SignFlipped, Reversed, Decoupled}` :191-219, `setMemberFault` :404-420; `FakeMotor::setCurrent` (the plant synthesizes no current); `BuiltRobot`'s `wire()` hangs each tank side's extra members on the plant's coupled train :225-262; case 5f (a Frozen member raises MOTOR_GROUP_DISAGREE) :678-698; case A4 (dead ports DRIVE, no false fight) :1270-1320 |
| The build | `src/build_selection.hpp` | guards :69-84 (`PROGRAM=drive` "needs ROBOT=tank -- only robot two's chassis table carries measured signs" :76-78; `PROGRAM=library` :79-81); `enum class Program` :89-95; ONE chain :115-139 (Drive :119-122, Teleop :123-126). `src/main.cpp`: its build table :20-36; five `switch (kProgram)` at :134, :160, :178, :202, :236; **`autonomous()`'s Drive case shares the tester's sentence** (:203-208). `Makefile` :91-147 (`drive` :131-135, `library` :136-140, the valid-values message :147) |
| The gate | `tools/src_build_gate.py` | `VARIANTS` :275, `BUILD_ARGS` :276, `VARIANT_DEFINES` :284, `VARIANT_BEACONS` :293, `BUILD_ROBOT` :307, `BUILD_PROGRAM` :380, `PROGRAM_ENTRY_SYMBOLS` :388-395; step 0's per-robot rule :592-642. **Self-test cases built on `tank-library` / `SHULIB_PROGRAM_LIBRARY`:** 12–13 :1400-1419, 16 :1451-1466, the define plants :1536-1552 and :1600-1604, 21 :1648-1666, 24 :1711-1729, and :1870, :1909; case 20 (`runDrive` in the tester's build) :1634-1641; **case 26 mutates `src/programs/drive_program.cpp`** (`DRIVE_PLANT` :1179, `tableSignedPort(kChassis, p)` :1802) |
| COMP | `docs/internal/chunks/COMP-competition-program.md` | B4 :55; B5 slot 5 :56-66; B10 :89-94; **B11** :95-100; B28 :155-161; **B29** :163-176; §3 item 3's "Slot **5**" :307-313 |
| The quoting gate | `tools/api_doc_tool.py` | `EXAMPLE_SOURCE_GLOB = "test/*example*_test.cpp"` :1811, used ~:1868 (becomes `EXAMPLE_SOURCE_GLOBS`, Part 1a item 6) |
| The worksheet | `docs/internal/chunks/R3a-BENCH-WORKSHEET.md` | A :12-108; D (Bench Tests' 10 DRIVE); **E — the OLD "shulib Drive"** :409-496; **F — "shulib Teleop"** :498-686 (the dead-port paragraph :616-621 says runtime death is slot 1's); G.1's uploads :752-770 (:761-762); G.6 :905-937; **J — the claws, in Teleop, slot 2** :1209-1531 (J.3.2 :1311-1312); DO NOT :1533-1557 |
| The ruling | status board ~:597; T3-PROGRESS P2.11.12 item 1 | "Driver control starts with the claw closed — whatever autonomous left it at; revisit when an auton first moves the claw" |
| SRC1 | `docs/internal/chunks/SRC1-organize-src.md` (`0fab866`) :17-33 | the folders every path above moves into before this is built |

### 0.4 The probe (evidence, not project code)
- **v1 (at `f6274e1`).** It compiled the first proposal and saw these results:
  - R1 put +12000 mV on port 6;
  - L2 put −12000 mV on port 7;
  - line A's first level was CLOSED;
  - one press of A opened the claw.
- **v2 (at `37706ce`, `.claude/coordinator/scratch/brief-T4/v2/`), the §2 file with a SKETCH of
  `driver_controls.hpp`.**
  - **Compile.** Clean on the host (`g++ -std=c++20 -Wall -Wextra -Wpedantic -Wconversion
    -Wsign-conversion -Wshadow -Werror`, through the PROS shim). Clean on ARM (`arm-none-eabi-g++ 13.2.1`,
    the `src/` set + `-Wformat-nonliteral -Wformat-truncation -Wunused-function -Werror`) under four
    define sets: none, `TANK_2026+PROGRAM_LIBRARY`, `TANK_2026+PROGRAM_DRIVE` and `XDRIVE_INVENTED`.
  - **Run.** Through `ProsDeviceProvider` with the REAL `kRobotTwoMechanisms` and the 2026-09-25 record:
    - `bind()` bound 2 holds and 1 toggle, and line A's first level was 0 (CLOSED);
    - R1 put +12000 mV on port 6 in the tick it was read;
    - **T2b:** with the claw set OPEN after `bind()` (as an auton would), the first `run()` wrote CLOSED
      exactly once (writes 2 → 3), and a second `run()` wrote nothing;
    - `bind()` AFTER the line was set OPEN still started the toggle CLOSED (the config read);
    - a Robot built from a table without "claw pivot 7" was REFUSED: `a control names 'claw pivot 7',
      which the robot does not declare`;
    - one where "claw pivot 7" is pneumatic was REFUSED: `'claw pivot 7' is not a motor mechanism: a
      hold control needs one`;
    - a FakeDeviceProvider refusing line A gave `bind()` true with 2 holds and 0 toggles.
  - **Compile mutations, each named message seen:**
    - `rezeroButton = A` → "a mechanism's button is the re-zero button";
    - port 7's line removed → "one of robot two's mechanisms has no control";
    - `20_ms` → "one tick is not the brain's";
    - `13_volt` → the volts message;
    - R1 twice → "a button does two jobs";
    - "claw pivot 66" and a hold naming "claw" → the naming message.
  - **An emptied `kToggles[]`** does not compile: a zero-size array, and no span conversion.
  - **Not project code.** The SKETCH's refusal words use one non-literal format under a pragma. The real
    file writes each refusal with its own literal format (landmine 13).
- **The rulings probe (the re-check, 2026-09-27; `.claude/coordinator/scratch/brief-T4/rulings/race_probe.cpp`).**
  - **What it is.** A pure simulation, not project code, of one four-member side over the REAL
    `teleop::CoupledSideMonitor`. It runs two trains: the plant (no inertia, the plant's Frozen
    semantics), and a hardware-like one (an 80 ms lag, BLUE's 300-count encoder, a lagging velocity
    report).
  - **What it compares.** Part 2's first absence rule (B) against N2 (a) as amended (N), over the same
    scenarios:

  | Scenario | B (first proposal) | N (N2 (a) amended) |
  |---|---|---|
  | P2-6 on the plant (freeze and reverse on one tick) | CUT at +24, ABSENT at +25 | 0 cuts, ABSENT at +29 |
  | dies one tick after a reversal (last-good screen) | CUT | 0 cuts |
  | dies while turning, stick released, reversed at 3 V (lagging train) | 10 cuts in 13 s, never ABSENT | 0 cuts, ABSENT |
  | a cut forced 2 ticks after the death, reversed at the re-arm | a second cut | none |
  | dies at 2.5 A in a push, 2 s at rest, drives away | 4 cuts (the push's + 3) | 1 (the push's own) |
  | healthy sweeps for 20 s, quantized | 0 | 0 |
  | a LIVE train hits a wall at full stick (velocity lag 80 ms) | a live member ABSENT | nothing |
  | a real fight (a member reads negated) | cut at tick 25 | cut at tick 25 |

  - **Five mutations of N, each red on a scenario:**
    - a FROZEN member's current counted;
    - over-current counted at or below 1 V;
    - no other-member guard (the wall marks a live member);
    - SUSPECT after 25 reads (P2-6 cuts);
    - the mates' fastest counting members whose position repeats (the latch moves from +28 to +128 in the
      cut-then-reverse case).
  - **It is a simulation, not the library.** The executor's tests in §3 are the proof.
- **Not probed in the tree: Part 2.** Its design rests on the anchors in §0.3 and §0.5 and on the probe
  above. It adds an optional `DriverLoopConfig` field appended last, which the §2 file's designated
  initializer must still accept. The first thing Part 2's executor does is compile the §2 file against the
  new `teleop_loop.hpp`.

### 0.5 What the library already covers, and what it lacks (R4)
**It covers:**
- **A dead drive port at BOOT.** The boot check reads the brain's registry before any adapter exists and
  applies `teleop::evaluateDegradation` once (`boot_check.hpp` ~:199, :277). The Robot builds each MISSING
  port as a `nullptr` hole in its side's MotorGroup (`robot.hpp` :69-80; `motor_group.hpp` :72-100). Reads
  are over the present members, and the monitor is told `present = false`. Test A4 proves no false fight
  (`robot_builder_test.cpp` :1270-1320).
- **A fighting or frozen member, as a FAULT.** Each group's coupled-side monitor is judged once per tick
  inside `drive()`. A persisted disagreement raises MOTOR_GROUP_DISAGREE per episode, on the CONTINUE side
  (`health_monitor.hpp` :241-253; test 5f :678-698).
- **MOTOR_OVER_TEMP at 55 °C** (HA-44), **the disable path** (`cancel()`: 0 V + Brake, `teleop_loop.hpp`
  :262-270), **Coast on release** (:254-259, :294), and **a disconnected controller → exactly zero**
  (HA-103).

**It lacks (each was only the old program's):**
- **A port that dies AFTER boot.** Nothing marks it. Its adapter holds a frozen last-good reading forever.
  The group's median still protects position while a minority is dead, but the monitor keeps judging the
  member: a frozen one flags MOTOR_GROUP_DISAGREE, and nothing drops it. The degradation verdict is never
  re-judged mid-run. Teleop says so at boot (`teleop_program.cpp` :261-263).
- **Any CUT.** A fight is told to the driver (the fault), never stopped. The MotorGroup header chose that
  for aborting a motion, because a slow member must not end a match (:63-71). The old program's 1 s
  non-fatal cut is a different trade that R4 now rules in.
- **Any over-current watch on drive motors.**
- **A runtime REFUSE:** 0 V for the rest of the run, once the dead pass the floor mid-run. Whether the
  library should refuse mid-match at all, and on which count, is N7.
- **A way to keep a dead port's FROZEN readings (its stale velocity and current) out of the fight
  monitor and the over-current watch** (N2).
- **A way for a panel to see a cut or a runtime death.**

---

## 1. Scope — IN

### Part 0 — the slots and the plan (docs only; lands before COMP-b)
The coordinator does this at N4's answer, or it is folded into Part 1a's run if the team lead prefers one
step.
- **COMP B5** (:56-66): "slot 5" becomes "slot 6", with one line on why (slot 5 is held for the team's
  old-code fallback). **B29's closing sentence** (:174-176) becomes: "decided by T4 (the team lead's
  rulings of 2026-09-27, R6): slot 6 "shulib Competition"; `opcontrol()` per B11 as T4 Part 1a replaces
  it". **§3 item 3's "Slot 5"** (:313, replaced by B29 but still readable) gets "(slot 6 — B5 as
  amended)".
- **`build-order.md`:**
  - :1472's COMP DoD: "slot 5" becomes "slot 6".
  - **The T4 entry** under Phase T after T3: the parts 0/1a/1b/2/3, the ruled order, the DoD and BRIEFED
    status. It includes one sentence: its screen rows are pure functions a later robot console will stream
    (R7). The console's own entry is the coordinator's plan commit, not T4's.
  - The chunk count +1. Count the rows, since SRC1's entry may land first.
  - The first `**Next:**` block names T4 Part 0/1a beside COMP-b.
  - **The deviations rows**, each with the team lead's words:
    - T4 added (2026-09-26 ~23:25);
    - the controls leave the mechanism table (R1);
    - G3a's `controls` target is a marked region inside the driver file (R1);
    - the order (R2);
    - **replace "shulib Drive"** (R3);
    - **the protections into the library** (R4);
    - retire "shulib Teleop" (R5);
    - the final slots (R6).
  - **One line in G3a's entry** (:1552-1583). The `controls` block's hold and toggle bindings (`kHolds`,
    `kToggles`) are generated into a MARKED REGION inside `src/team/robot_two_driver.hpp`'s WHICH BUTTON
    block, not into the tables. Where the re-zero button is generated is G3a's to decide (R1). Today it is
    `kLoop.rezeroButton`, under THE FEEL, its ONE owner since the 2026-09-26 re-check. The DoD gains "and
    the driver file's region, byte for byte".
- **Worksheet Station A** (:12-108): the slot list, as the table in § Decisions under N4's answer:
  - slot 1 is the OLD "shulib Drive" until T4 Part 3 replaces it in place;
  - slot 5 is HELD for the team's fallback (do not upload over it);
  - slot 6 is reserved for "shulib Competition";
  - the tester's interim slot per N4.

  The photograph of the brain's program list stays as a plain step, with no register entry (N3).
- **`docs/roadmap.md`:** T4's Phase T row, PLANNED.
- **`HANDOFF-remaining-work.md`:**
  - T4's parts, and N1–N7 as open questions.
  - One line for SAFE-SAVE B2's re-check: its `BACKUP see brain` word goes into the shared chooser (COMP-b's
    header), at its place after CLAW ABSENT, never into `teleop_program.cpp`.
  - One line for SAFE-SAVE A/B1's re-check: `formatSaveReport`'s sentences name Teleop, which T4 Part 3
    retires (landmine 25).
- **R2's "HA entries": none remain for Part 0.** HA-175 was reserved for the program-name belief, which R3
  made moot, and N3 re-assigns it to Part 2's absence rule. HA-176 is Part 2's too. So no HA entry is
  issued in Part 0.

### Part 1a — the driver file (before COMP-b; no T3 rework)
1. **`src/common/driver_controls.hpp`** (PROS-free; the executor writes the real one, with its prose, from the
   §0.4 sketch):
   - **The data.** `HoldControl{mechanism, forward, back, units::Voltage volts}` and
     `ToggleControl{mechanism, button}`.
   - **Five constexpr predicates, one per property,** so a failing `static_assert` names the real fault:
     - `controlButtonsDistinct` — no button twice across holds and toggles, a pivot whose forward is its
       back included;
     - `controlsAvoid(…, reserved)`;
     - `holdVoltsValid` — (0, 12], written so NaN fails;
     - `controlsNameTable(…, table)` — each control names a declared entry of the right KIND, and no
       entry twice;
     - `controlsCoverTable(…, table)` — every declared entry has a control.
   - **`class DriverBindings`** (non-copyable, non-movable; static storage). Its capacity is
     `config::kMaxMechanisms` per kind. Its members:
     - **`bind(robot, holds, toggles, rezero, why, cap)`** resolves each control against
       `robot.config().mechanisms[0..mechanismCount())`, BY NAME:
       - it refuses (false, `why` filled, nothing bound) a name the robot does not declare, a name of
         the wrong kind, and a declared mechanism no control names;
       - it SKIPS a declared mechanism that was not built (ABSENT: its buttons do nothing);
       - it gives each toggle its initial level from the CONFIG,
         `config::toLineValue(mechanisms[i].pneumaticStart)`, the level the Robot made the line at;
       - it runs `teleop::validBindings` last.
     - `unbind()`; `holds()` / `toggles()` (spans); `holdFor(name)` / `toggleFor(name)`;
       `anyReleasedLeftoverClaim()`.
   - **`driverHeading(const robot::Robot&)`** — THE heading rule (Teleop's and the X-drive's today,
     byte-identical logic).
   - **Every refusal a literal format**; each sentence is host-pinned.
2. **`src/team/robot_two_driver.hpp`** — §2, verbatim.
   - The header's caller sentence is TRUE at every commit, and each part rewrites it in its own change:
     - **At 1a:** "No program runs this file yet: the competition program's opcontrol() will (chunk
       COMP-b), and so will the driver program that replaces "shulib Drive" (T4)."
     - **COMP-b:** "The competition program's opcontrol() runs THIS file; the driver program that replaces
       "shulib Drive" will (T4)."
     - **1b** adds: "so does "shulib Teleop", inside its bench panel, until the driver program retires
       it".
       - **1b also RESTORES §2's "To change what a button does, change it HERE and nowhere else."**
         *(Added 2026-09-27 by Part 1a's review.)* Part 1a could not write that sentence: while the
         interim tie stands it is false. So 1a wrote, in its place: "To change what a button does,
         change it HERE -- and, until T4 moves the buttons out of src/robots/mechanism_table.hpp,
         there too (the re-zero also in "shulib Teleop"): `make` refuses a build in which they
         disagree." 1b removes the table's copy and the tie, which makes the 1a clause false; 1b
         deletes it and writes §2's sentence back. In the same change, 1b deletes cookbook chapter
         6's "Watch out for" bullet that 1a wrote, "Until the buttons leave the mechanism table,
         change a button in both files …" (see §4, Part 1b).
     - **Part 2** adds the protections paragraph.
     - **Part 3** writes the final form shown in §2.
   - It is a HEADER, not a `.cpp`: the host suite compiles only `test/*.cpp` and includes `src/` headers
     by relative path (the pattern of `teleop_claws.hpp` and `boot_source.hpp`).
3. **The interim tie (the two steps' tie, until 1b), compile-time, in the RETIRING code:**
   - **In `src/programs/teleop_claws.hpp`**, a `controlsMatchTable(holds, toggles, table)` predicate: each
     control's buttons and volts equal the table entry's of that name.
   - **In `src/programs/teleop_program.cpp`**, two asserts: `static_assert(controlsMatchTable(RobotTwoDriver::kHolds,
     RobotTwoDriver::kToggles, kRobotTwoMechanisms), "the driver file and the mechanism table disagree
     about a button -- change both until T4 Part 1b")` and `static_assert(RobotTwoDriver::kLoop.rezeroButton
     == kRezeroButton, …)`. The same two go in `test/robot_two_driver_test.cpp`, so the host suite sees
     them too.
   - Both retire in 1b. Naming `kRobotTwoMechanisms` inside a `static_assert` is step 0-legal.
4. **The shared words, for COMP-b:** `clawRowText(t, robot, bool freed)`. The `TeleopClaws&` form stays
   as a one-line forwarder until 1b, so no call site and no 1l pin changes. Byte-identical.
5. **One heading rule:** `src/programs/teleop_program.cpp`'s `headingSample()` (:559) and
   `src/programs/xdrive_program.cpp`'s (:237-248) are DELETED, and both call `bench::driverHeading`. The X-drive's
   "written twice … the merge as owed" comment goes (G1's owed item closed; say so in G1-COMPLETED's
   follow-ups if it lists it).
6. **`tools/api_doc_tool.py` `check-examples`** also reads the driver files, `src/team/*_driver.hpp`: the
   kind of `src/team/` file a public document quotes whole. **One shape, shared with COMP-t** (the auton
   tester's brief, re-check finding 2): `EXAMPLE_SOURCE_GLOB` (:1811, used ~:1868, reasoned :47) becomes
   `EXAMPLE_SOURCE_GLOBS`, a LIST of globs, each naming one KIND of file a public document may quote whole
   and each required to match at least one file: `test/*example*_test.cpp`, `src/team/*_driver.hpp` (T4),
   `src/team/autons.cpp` (COMP-t). Whichever chunk lands first creates the list with its own entry; the
   second appends. Neither calls its entry "the one/only exception"; the guide README says "the team's
   files under `src/team/` that a chapter quotes (today: …), never the rest of `src/`", and the second chunk
   extends that sentence. It is commented in the tool's header beside the list's reasoning, with a
   self-test case: a line only a driver file holds passes, and a drifted one fails. It is never widened to
   all of `src/`. Both chunks also edit `SHULIB_DOC_INPUTS` (`test/CMakeLists.txt` ~:172) and
   `boot_source.hpp` (T4 :92-94); whoever lands second re-reads.
7. **The three T3 comments that say the claw "cannot move when driver control (first) starts"** are
   reworded to the rule: the first entry in a run SENDS the start level; nothing moves unless something
   commanded the line in between; after an autonomous the ruling closes it. They are:
   - `teleop_claws.hpp` :21-25;
   - `mechanism_table.hpp` :493-497;
   - `boot_source.hpp` :92-94 (prose only; SAFE-SAVE B2 edits this file later — whoever lands second
     re-reads).

### Part 1b — one owner (after COMP-b)
1. **`src/robots/mechanism_table.hpp`:**
   - remove `forward`, `reverse`, `volts` (the motor entry), `toggle` (the pneumatic entry),
     `kTeleopRezeroButton`, checks 4-7 and their asserts (robot two's and the build's), and update
     completeness;
   - rewrite the header's WHAT IT DECLARES, WHY std::optional (its button half), COMPILE-TIME CHECKS
     (renumbered) and WHO READS IT;
   - robot two's doc comment says where the buttons live now;
   - the provenance strings keep the team lead's words and add "(buttons: src/team/robot_two_driver.hpp since
     T4)".

   **Nothing about ports, signs, cartridges, safe modes, the line, the polarity or the positions
   changes.** `boot_source`'s conversion and Calibrate's drop are the pins, their tests unchanged but for
   hand-built tables losing four fields.
2. **`src/programs/teleop_claws.hpp`:**
   - `TeleopClaws`, `controlsMatchTable` and the forwarder retire;
   - the boot lines and the banner take the controls (`std::span<const HoldControl>`,
     `std::span<const ToggleControl>`, the re-zero button) beside the table;
   - **every sentence stays byte-identical** — J.3.1, J.3.2, J.9.1, the claw row, the fault row, the
     banner.
3. **`src/programs/teleop_program.cpp`** (it retires in Part 3; until then it must stay exactly right):
   - **The state.** `TeleopState` holds `RobotTwoDriver driver{}` in place of `master`, `shaper` and
     `claws`. The storage rule (:64-73) is REWORDED: the static holds empty optionals and members whose
     constructors cannot throw and touch no hardware. `ProsController`'s touches none
     (`hal/pros/controller.hpp` :63-69); `DriveShaper`'s is `noexcept`.
   - **The boot.** `teleopInitialize` calls `st.driver.bind(*st.robot, why, sizeof why)` where
     `st.claws.bind(…)` is, inside the try, with `st.driver.unbind()` first in the catch.
   - **The loop.** `runTeleop` becomes `st.driver.run(r, st.devices->pacer(), [] { return
     pros::c::competition_is_disabled() != 0; }, panel);`.
   - **What goes.** `kRezeroButton`, `kFieldCentric`, `teleopProfile()` and the tie asserts, in favour of
     `RobotTwoDriver::kLoop.rezeroButton` and `RobotTwoDriver::kFeel`. `kFeel.mode == ArcadeField` is
     "field-centric ON".
   - **Each read of a retired name** is re-pointed: the driver line (:855-857), the banner (:889), and
     `modeWord(state().shaper.profile().mode)` (:702) → `modeWord(RobotTwoDriver::kFeel.mode)`. The
     profile/driver-line assert reads `RobotTwoDriver::kFeel`.
   - **Every line Teleop prints stays byte-identical**, verified two ways, because this TU is not
     host-compiled:
     - (1) `git diff -U0 -- src/programs/teleop_program.cpp | grep '^[-+].*"'` lists every changed line holding a
       string literal. Each hit is accounted for in the log, and no literal's TEXT may change.
     - (2) If Station J has been run on a pre-1b build, the next Teleop capture's serial lines are
       diffed against it (the worksheet says so); any difference is a finding.
4. **`include/shulib/teleop/teleop_loop.hpp`'s banner** (:92-96, which already names Teleop and the
   X-drive) adds only "through robot two's driver file (src/team/robot_two_driver.hpp)". `docs/api` is
   regenerated; there are no declaration changes, so no changelog entry (say so in the commit body).

### Part 2 — the drive protections, in the library (after Part 1b; BEFORE Part 3)
The team lead's ruling R4. Everything here is library code (`include/shulib/`, PROS-free), apart from the
few `src/` lines in item 6 that would otherwise go false. It is host-tested on the plant's member faults,
and on hand-stepped `FakeMotor` reads where the plant cannot shape the case (a train with inertia, a wall).
N1, N2, N3 and N7 are answered before it starts. **No line of the driver file's CODE changes.**
- The loop reads its protection config from `DriverLoopConfig`, whose new field is appended last and
  defaults to the old program's numbers.
- It hands its state to a `perTick` that asks for it.
- D2 (iv)'s "the driver file is final from its first commit" therefore holds. Only the file's header
  comment gains the protections paragraph (§2).

1. **`include/shulib/teleop/drive_protection.hpp`** (NEW; PROS-free; allocation-free):
   - **`struct DriveProtectionConfig`.** The old program's numbers are its defaults, each with its source:
     - `double overCurrentAmps = 2.4;` (`drive_program.cpp` :96; HA-176);
     - `int overCurrentTicks = 25;` (:94). It is pinned EQUAL to `SideMonitorConfig{}.persistTicks` by a
       static_assert, as :104 pins it today: "the 250 ms window equals the over-current window by choice"
       (HA-132).
     - `int cutTicks = 100;` (:97, the 1 s cut);
     - `int maxCutsLogged = 50;` (:100);
     - `bool overCurrentCuts = true;` — N1's one field: false turns the over-current CUT into a logged,
       counted WARNING;
     - `int frozenSuspectReads = 5;` — N2 (a)'s one new number (HA-175): the consecutive bit-identical
       position reads that make a member SUSPECT. A static_assert pins the default BELOW
       `SideMonitorConfig{}.persistTicks`, because the FROZEN rule must be faster than the monitor or a
       dead member is cut on before it is excluded.

     It is checked at the loop's entry, before anything moves: every number finite and positive, and
     `frozenSuspectReads` below every group's `persistTicks`. A violation is a precondition error, and
     nothing is commanded (P2-20). The fight and absence floors are NOT here. They are each group's
     `SideMonitorConfig` (HA-132/133): one set of numbers, as today.
   - **`struct DriveProtectionState`** (what a panel reads; trivially copyable):
     - `mode`: `Driving` / `Degraded` / `Cut` / `Refused` (`Refused` never occurs under N7 (b));
     - `cutTicksLeft`, `cutCount`;
     - `lastCut` (a bounded char array holding the log's own words);
     - `dead` (the runtime-dead ports as text, `"13 18"`);
     - `frozen` (the ports FROZEN this tick, as text; empty almost always);
     - the `DegradationVerdict` in force;
     - each side's peak current this entry.
   - **`class DriveProtection`.** There is ONE per `runDriverControl` call (per driver-control entry). It
     works over `chassis.deps().motorGroups`, `deps().ctx->driveMotors()` and `deps().ctx->telemetry()`,
     with one `teleop::FrozenReadingDetector` per group (item 4):
     - **`enter()`** re-judges the degradation from each group's `memberCount()` / `presentCount()`, by
       N7's rule. Runtime marks live in the GROUPS, which sit in the program's static storage. So under N7
       (a) or (c) a robot whose marks meet the rule is REFUSED at the next entry, 0 V from tick 1.
     - **`beforeDrive()`: the FROZEN rule and RUNTIME ABSENCE, first** (N2 (a)). This keeps the old order
       (absence :482-520, THEN the fight :522-555), and it is the order that wins the race.
       - Every group's detector is handed each PRESENT member's fresh `position()` and `velocity()`.
       - The verdict's FROZEN members go to `group.setMonitorExcluded(frozenMask)` for this tick, and are
         remembered for the over-current check.
       - A member FROZEN for 25 consecutive ticks latches → `group.markAbsent(i)`, plus ONE Warn line in
         the old words (:499-501). The line names the port from the group's labels, with the reason `reads
         frozen for 250 ms while its side answered`.
       - The old velocity-exactly-0 signature runs beside it: `MemberAbsenceDetector::update(0, velocity,
         groupCommand, matesFastest)`. The faulted-reads argument is a constant 0, because the seam has no
         count; `groupCommand` is the group's `commandedVoltage()` from the last `drive()`.
       - After a latch, the degradation is re-judged by N7's rule, with its line (:505-512). Under N7 (a)
         or (c), a REFUSE latches `Refused` with its line (:513-518). The marks keep it refused at every
         later entry.
       - **Why before `drive()`:** the group monitors are judged INSIDE `drive()` (`chassis.hpp` :385-420).
         A member found FROZEN here is therefore already `present = false` to them THIS tick (P2-6b pins the
         order).
     - **`commandAllowed()`** is false while `Cut` or `Refused`. The loop then hands `drive()` a ZERO
       command, so the Localizer and the health tick keep running. The cruise feedforward makes zero
       exactly 0 V (`command_pipeline.hpp` :142-149, `feedforward.hpp` :77-80); a test pins exactly 0.0,
       so a future kA or slew term that leaked volts into a cut would fail it.
     - **`afterDrive()`:**
       - **If `Cut`:** count down. At zero it RE-ARMS: the over-current streaks are cleared, each group's
         `resetDisagreement()` is called, and one line is logged (`re-armed after cut #n (1 s at 0 V)`,
         :457). The FROZEN detectors are NOT reset: a dead member keeps counting toward ABSENT through a
         cut, because its own stale speed keeps it SUSPECT while its mates stand (P2-6c).
       - **Otherwise, unless `Refused`,** it checks two things:
         - **the FIGHT:** any group's `lastVerdict().persistedCount > 0`. That is the group's own monitor,
           judged once in this tick's `drive()`, with runtime-absent and FROZEN members already excluded.
           There is no second monitor.
         - **the OVER-CURRENT:** a present member that is neither runtime-absent nor FROZEN this tick, and
           whose group is commanded above its `commandFloorV` (1 V) this tick, reading `current()` above
           `overCurrentAmps` for `overCurrentTicks` CONSECUTIVE ticks. Every other member's streak is 0
           that tick: an absent one (:558-561), a FROZEN one (its current is its last good reading), and
           any member of a side at 1 V or less (a drive in Coast there has nothing a cut would stop). It is
           judged per member, never on the side's `totalCurrent()`.
       - **On either, it CUTS NOW:**
         - 0 V is written to every drive motor in THIS tick. `drive()` has already commanded the stick's
           volts, and the old `stopAll` (:570) did the same.
         - `cutTicksLeft = cutTicks`, `++cutCount`.
         - The old words are logged (:547-553, :565-566, :574), plus one `at the cut:` line per member
           (:576-580), within `maxCutsLogged` and its once-only notice (:581-583).
       - With `overCurrentCuts == false`, an over-current is logged and counted as a WARNING and nothing is
         cut.
     - **On the disable** (the loop's step 1, unchanged: `cancel()`, 0 V + Brake, return): one summary line
       per group, giving each member's peak current and temperature this entry and the cut count. This
       MOVES the old exit summary (:667-684) into the evidence HA-176 and HA-132 need.
     - Every line goes through the chassis's sink with tag `TEL` (the loop's existing tag,
       `teleop_loop.hpp` :229). The old words keep their text, re-pointed from "port" to the label.
2. **`include/shulib/teleop/teleop_loop.hpp`:**
   - **`DriverLoopConfig` gains `DriveProtectionConfig protection{}`, APPENDED LAST.** Designated
     initializers must follow declaration order. The driver file's `kLoop` names the first three fields
     and keeps compiling, with the defaults.
   - **`runDriverControl`** constructs one `DriveProtection` after the precondition checks and `enter()`s
     it. Each tick, after shaping, it runs `beforeDrive()` → `chassis.drive(allowed ? command : zero, Body)`
     → `afterDrive()` → step 6b (the release mode) → the bindings → the toggles → `perTick` → `pace`. The
     disable path is untouched, and a cut in force never delays it.
   - **`perTick` may take a FOURTH argument, `const DriveProtectionState&`.** It is detected with one
     `if constexpr (std::is_invocable_v<…>)`. The three-argument form still compiles and runs unchanged:
     Teleop's panel, the X-drive's, and every existing test use it.
   - **It applies to EVERY caller.** That is the driver file, so COMP's `opcontrol()`, and Teleop until
     Part 3. It also covers the invented X-drive, which has no groups. There it runs the over-current cut
     only, per bare drive motor, named by wheel index, and counted while that motor is commanded above 1 V.
     With no side to compare against, a bare motor's frozen current is NOT screened: stated, not solved.
     The X-drive refuses at boot on every robot.
   - The header's tick order (:18-53) and its banner (:92-96) gain the protections, in place.
   - **Rejected:**
     - (i) a protection object each program builds and passes (another overload): a program that forgets
       it drives unprotected, and passing it through the driver file changes the file's code after its
       first commit;
     - (ii) the driver file owning a `DriveProtection` member for its panel: the same objection;
     - (iii) the state carried in `DriveShaperState`: a feel type carrying safety state.
3. **`include/shulib/hal/motor_group.hpp` — RUNTIME absence.** This is the chunk the header's "a later
   chunk" (:82-90) points at.
   - **`bool markAbsent(std::size_t i) noexcept`** latches member i ABSENT for the group's life.
     - From then on it is excluded from EVERY read (position, velocity, current, temperature, and
       `brakeMode()`'s first member). The monitor is told `present = false`, exactly like a boot hole.
     - It is **STILL COMMANDED**: `setVoltage` and `setBrakeMode` still reach it. That is the old
       program's rule ("still commanded (harmless)", :24-25): a dead port ignores the command, and a member
       whose reads failed while its motor still turns keeps pulling.
     - It **REFUSES** — returns false, changes nothing, never throws — when i is out of range, is already
       absent, or is the LAST present member. A precondition throw mid-match would end driver control. The
       degradation policy refuses the drive long before a side reaches zero.
   - `presentCount()` / `isPresent()` reflect it. A new `runtimeAbsentMask()` says which members are
     runtime-absent. `absentMask()` stays the BOOT holes. `Robot::describe()` reads `isPresent`, so it
     names a runtime death too; its doc says so.
   - **`void setMonitorExcluded(std::uint32_t mask) noexcept`** (N2 (a)). Members whose bit is set are fed
     to the monitor as `present = false` on every `evaluateDisagreement()` until the mask changes. They
     are the protection's FROZEN members, re-set every tick; `monitorExcludedMask()` reads the mask.
     - It changes NOTHING else: an excluded member stays in every read and is still commanded.
     - Bits past `memberCount()`, and bits on holes or runtime-absent members, are ignored.
     - The default 0 is bit-identical to today.
     - Only the protections set it. Motion code never does, so MOTOR_GROUP_DISAGREE in an autonomous
       motion (test 5f) is unchanged.
   - **Labels:** `MotorGroup(members, thresholds = {}, labels = {})`, an optional span giving one port
     number per slot, so a log line can name the port. `robot::Robot` passes the description's port
     magnitudes (`makeSideGroup` ~:922). A group with no labels says "member i". A two-group tank names its
     sides LEFT and RIGHT by the Robot's order (`makeGroupArray`: left, then right).
     - **Rejected:** naming a slot by index (useless to a pit crew); handing the Robot to the loop (the
       loop takes a Chassis — F6's shape).
   - **All-present groups with no marks are BIT-IDENTICAL.** The whole suite is the pin, with its
     assertion count recorded before and after, as at R3d Part 2.
4. **`include/shulib/teleop/coupled_side_monitor.hpp`: N2 (a)'s `FrozenReadingDetector`** (NEW, additive;
   pure, allocation-free, sized `kMaxSideMembers` like the monitor; one per side):
   - **Construction.** `FrozenReadingDetector(SideMonitorConfig floors, int suspectReads)`, with the
     precondition `0 < suspectReads < floors.persistTicks`.
   - **Its tick.** `FrozenVerdict update(std::span<const MemberReading> members)`, with
     `MemberReading{positionRad, velocityRadS, present}` in the side's fixed order, once per tick. It
     returns `FrozenVerdict{suspectMask, frozenMask, latchedMask}`, where "latched" is newly ABSENT this
     tick. Also `reset()`, and `repeatReads(i)` for a panel.
   - **The rules, exactly.**
     - `repeat[i]` counts the consecutive reads bit-identical to member i's previous one. It is 0 on its
       first read and whenever the reading changes, and a member that is not present resets.
     - **The mates' fastest for i** is the largest |velocity| among the OTHER present, unlatched members
       whose `repeat` is below `suspectReads`. A member whose own encoder has stopped is no evidence that
       the train turns.
     - **SUSPECT(i)** = `repeat[i] >= suspectReads` AND (`|v[i]| > oppositeFloorRadS` OR the mates'
       fastest `> movingFloorRadS`).
     - **FROZEN(i)** = SUSPECT(i) AND some other present, unlatched member is not SUSPECT.
     - FROZEN for `persistTicks` consecutive ticks LATCHES the member, for the detector's life. The reason
       is `"reads frozen for 250 ms while its side answered"`.
   - `MemberAbsenceDetector` is UNCHANGED. Its velocity-exactly-0 signature runs beside the new detector.
     Its faulted-reads form stays for a caller that has the count; say that no `src/` caller does after
     Part 3.
   - The header's "the drive program" sentences about the monitor and the detector (:15, :40, :65, :195)
     are re-pointed to the library's protections. :171 (`tankSideVolts`, still the old program's until
     Part 3) is Part 3's.
5. **`include/shulib/teleop/drivetrain_degradation.hpp`:**
   - **Prose.** Its caller is now the boot check AND the library's protections mid-run (:38-40 names
     `src/programs/drive_program.cpp`). WHY THIS EXISTS keeps the team lead's words. The DOMAIN paragraph (:30-35)
     says that robot two is now four per side.
   - **Under N7 (c)**, one additive pure function, `evaluateRuntimeDegradation(left, right)`. It REFUSES
     only when the total dead exceeds `kMaxDeadTotal`, and otherwise gives Drive / DriveDegraded with the
     counts. It is host-tested at every boundary row beside `evaluateDegradation`, which does not change
     (it is the boot check's rule).
   - Under N7 (a) the protections call `evaluateDegradation` as it is. Under (b) they call nothing that can
     refuse.
6. **`src/`: only what would go false, and the one row that makes a cut visible.**
   - The SHARED driver panel header (COMP-b's; `src/common/driver_panel.hpp` as proposed): the PROTECTION row
     function and the LCD chooser's new words (`REFUSED see brain` under N7 (a)/(c), `CUT see brain`, and
     row 1's `dead <ports>`), at their places in D4's order. Pure and host-tested. COMP's page gains the
     PROTECTION row.
   - `src/programs/competition_program.cpp`: its `perTick` takes the fourth argument.
   - `src/programs/teleop_program.cpp`: `kDeadPortLine1/2` (:258-263) are reworded to say that runtime absence is
     now the library's, in every program. They are logged as the ONLY literal changes, by 1b's diff check.
     Teleop's screen shows no cut: its twelve rows are full (:302-316) and it retires in Part 3. Its serial
     log shows the cut, and its fault row shows a fight.
   - The driver file's header (§2's protections paragraph, and `run()`'s doc), with the cookbook's quote
     of it.
7. **HA** (texts in §4, numbers per N3):
   - **HA-175** — the mid-match absence rule: N2's FROZEN rule, the velocity-exactly-0 signature, and the
     floor as N7 rules it;
   - **HA-176** — the over-current cut;
   - **HA-132/133 AMENDED**: their consumers now include the library's protections in every driver-control
     program; the blast radius is that a false fight now CUTS the drive for 1 s, in a match.

### Part 3 — the new "shulib Drive"; "shulib Teleop" retired (after Part 2; ONE change, R5)
1. **`src/programs/drive_program.{hpp,cpp}` — content REPLACED.** The names stay (R3). It is in
   `namespace shulib::bench`, with the serial tag **`DRV`**. The tag is new so that a capture tells this
   program from the old `[R3B] booted: shulib DRIVE`.
   - **The entry points:** `driveInitialize(beacon, bits)`, `kDriveAutonomousReason` (≤ 50 characters,
     e.g. `No motion here, ever: Drive has no autonomous.` — 46), `driveAutonomous()`, `runDrive()`.
   - **The boot, in Teleop's order** (`teleopInitialize` :877-953):
     - the banner (beacon, build stamp, the run status);
     - the shared boot source;
     - the provider (given `"drive_log.txt"` under N5 (a), so its log is copied to the card from here on);
     - `validate`, `emplace` and `st.driver.bind()` in ONE try, with `st.driver.unbind()` first in its
       catch;
     - `attachFaultLatch`;
     - the session header;
     - the boot lines — Teleop's `logBootLines` MOVED, without `clawBannerLine()`, with the protections
       line generated from `RobotTwoDriver::kLoop.protection`.
   - **Any refusal** paints the shared refusal screen (header word `DRIVE REFUSED AT BOOT`) and builds
     nothing further. `bind()`'s `why` is the refusal's reason.
   - **The entry points' bodies.** `autonomous()` commands nothing. `opcontrol()`: if refused, repaint and
     idle; otherwise `st.driver.run(r, st.devices->pacer(), disabled, screen)`, then the disable line.
   - **Storage:** ONE function-local static (1b's reworded rule).
   - **The screen** is D4's MAIN (header, PROTECTION, controller, `DETAILS`) and DETAILS (R5's four rows
     and the rest), both composed from the pure rows in the shared header. A touch on the drawn `DETAILS` /
     `BACK` button flips the page, using `ui::touch_panel`'s
     release edge and arming delay. Only the visible page is painted, a row is repainted only when it
     changed, at most 5 Hz, every string through `printText()`. The LCD uses the shared chooser.
2. **What is DELETED from the old `drive_program.cpp`, and what MOVES:**

   | Old `drive_program.cpp` | Lines | Fate |
   |---|---|---|
   | header: typed-table signs, "adapters, NOT the motion stack", the all-stop guard's reasoning | :1-49 | DELETED — the new header says what the new program is |
   | `kMaxDriveV = 12`, open-loop volts; `mapSticks` → `tankSideVolts` | :89-91, :434-445 | DELETED — the library's DriveShaper, feedforward and battery ceiling drive it. Ruling M4's raw-volts baseline survives in Bench Tests' 10 DRIVE (`bench_r3a.cpp` :1629-1642, Station D) |
   | `kPersistTicks`, `kCurrentLimitA`, `kCutTicks`, `kMaxCutsLogged` and the `SideMonitorConfig` pins | :93-108 | MOVED (Part 2) → `DriveProtectionConfig` |
   | screen constants and colours; `paintRefusal`, `drawHeader`, `rowText` | :110-115, :203-250 | DELETED — the shared panel and Teleop's `paintRefusal` (moved) replace them |
   | `emit`/`emitf`/`flushCard` and `ProsBlockSink "drive_log.txt"` | :117-144, :275-276 | DELETED — **N5** decides whether an SD copy of the log comes back, owned by the provider (item 4) |
   | `Member`, `stopAll`, `AllStopGuard` | :146-176 | DELETED — the Robot's groups, `cancel()` on the disable, and Part 2's zero command plus the 0 V write at a cut replace them |
   | `deadPortsLine`, `evaluate` | :178-201 | MOVED (Part 2) → the protections' dead list and the mid-run re-judgement. The BOOT judgement was already the library's (`boot_check.hpp` :277) |
   | the old banner and boot lines (typed table, "IMU not used", the policy line) | :254-318 | DELETED — Teleop's boot lines MOVE in, with the protections line |
   | the typed-table refusals (no signs / unset / contradicts itself) | :320-336 | DELETED — the boot source's record check. The typed table cannot describe robot two (UNSET track width) |
   | a `ProsMotor` per port inside its own try | :338-370 | DELETED — the boot check's registry read and the Robot's absent ports (G1 2a) |
   | boot degradation | :372-391 | already the library's (the boot check) |
   | controller, LCD, battery | :393-401 | DELETED — the driver file's controller and the shared LCD chooser |
   | the ten per-member rows | :403-415, :589-641 | DELETED — DETAILS' two group rows (present/members, disagreeing, ports dead since boot), DETAILS' peak currents, and MAIN's PROTECTION row. Per-member live amps and °C stay in Bench Tests' 10 DRIVE |
   | the cut and re-arm; runtime absence and the runtime REFUSE; the fight and over-current cuts; `at the cut:` lines; the log bound | :447-461, :482-520, :522-587 | MOVED (Part 2) |
   | LCD rows | :643-662 | MOVED in spirit (Part 2): the shared chooser's REFUSED / CUT / dead words |
   | exit: `stopAll`, LCD, the summary table | :667-684 | the summary MOVED (Part 2's disable line: per-member peaks and the cut count); "driven seconds" DELETED |

   **What MOVES from Teleop** (`src/programs/teleop_program.cpp`, deleted). Every item is moved, never copied:
   - `kTag`, `kBuildStamp` and `kBuildTimeText` (:133-139); the tag becomes `DRV`.
   - `kRunStatus` (:142), rewritten for this program.
   - `logWrapped` and the 200-byte checks (:224-278).
   - The boot-line literals (:251-278), reworded where false. `kProvisionalLine`'s "feel different from
     shulib Drive" goes.
   - The screen constants, `PanelRow` and the `text::` checks (:284-377), into the shared model split into
     MAIN and DETAILS.
   - `sourceRows()` with its three static_asserts (:379-470); `paintHeaderBar`/`rowY` (:475-486).
   - `TeleopState` (:489-508), which becomes the program's state.
   - `paintRefusal`/`refuse` (:520-555).
   - `qualityName`/`modeWord`/`fieldCentricWord` (:567-598), for DETAILS.
   - `TeleopPanel` (:611-781), split into the pure MAIN/DETAILS rows and a thin painter.
   - `logBootLines` (:783-875).
   - `teleopInitialize` / `teleopAutonomous` / `runTeleop`, which become `driveInitialize` /
     `driveAutonomous` / `runDrive`.
3. **"shulib Teleop" RETIRED (R5), everywhere it is named:**
   - **Deleted:** `src/programs/teleop_program.{hpp,cpp}`. The remaining words in `src/programs/teleop_claws.hpp` MOVE to the
     shared panel header: the claw row, the fault row, the LCD word and the claw boot lines.
     `clawBannerLine()` is DELETED, because only Teleop printed it. `test/teleop_claws_test.cpp` becomes the
     shared panel's test:
     - every surviving word pin is UNEDITED and green;
     - each pin whose sentence names Teleop or slot 2 is reworded, logged one-for-one.
   - **The runtime words that name Teleop or slot 2 OUTSIDE the deleted files.** After Part 3, slot 2 is
     "shulib Auton Tester". No test, document or README quotes either string (grep), so only the source
     changes:
     - `src/programs/calibrate_program.cpp` :896-897, `SAVED -- ... shulib Teleop (slot 2) will boot from it.`,
       becomes `... shulib Drive (slot 1) and the competition program boot from it.` If SAFE-SAVE B1 has
       replaced that line with `formatSaveReport`'s output, that function's sentences are reworded instead
       (next bullet).
     - `src/programs/xdrive_program.cpp` :174, `kRobotTwo = "Robot two runs shulib Teleop (slot 2) instead."`,
       becomes `"Robot two runs shulib Drive (slot 1) instead."`: 45 characters against 46, so its
       `screenText` static_assert holds.
     - **SAFE-SAVE A's `formatSaveReport`** (`include/shulib/calibration/record_store.hpp`, if landed)
       says "Teleop boots from the record file", "this Teleop does not read the backup and will REFUSE",
       "boot Teleop and read its `calibration record:` line" (SAFE-SAVE :612-642).
       - Each sentence names "shulib Drive and the competition program" (or "the driver programs") in
         place of Teleop, and each is refitted to Calibrate's `emitf`.
       - Its tests' expected strings are reworded one-for-one, logged.
       - This is a library TEXT change, which §5 allows; no declaration changes.
     - The COMMENTS that name Teleop, the old Drive or slot 2 are §4 Part 3's sweep (every file listed
       there).
   - **`src/build_selection.hpp`:** `Program::Teleop`, the tank-library branch (:123-126) and the
     `PROGRAM=library` `#error` (:79-81) are removed. `SHULIB_PROGRAM_LIBRARY` leaves the switch list and
     joins a RETIRED list, which the gate fails anywhere. The `PROGRAM=drive` `#error` (:76-78) is reworded:
     "needs ROBOT=tank -- only robot two has a driver file and a calibration record".
   - **`src/main.cpp`:**
     - The Teleop case goes from each of the five switches.
     - The Drive case gets its own `autonomous()` sentence, `kDriveAutonomousReason`, where it now shares
       the tester's "the tester's DRIVE station lives under driver control" (:203-208).
     - The build table (:20-36) and the header's run-status notes (:38-49) are updated.
   - **The `Makefile`:**
     - `PROGRAM=library` → `$(error PROGRAM=library was retired at T4: "shulib Teleop"'s diagnostics are
       "shulib Drive"'s DETAILS page -- use make ROBOT=tank PROGRAM=drive)`.
     - `PROGRAM=drive`'s error is reworded (:133).
     - The valid-values message (:147) and the programs comment (:91-127) are updated.
   - **`tools/src_build_gate.py`:**
     - `tank-library` is removed from `VARIANTS`, `BUILD_ARGS`, `VARIANT_DEFINES`, `VARIANT_BEACONS`,
       `BUILD_ROBOT`, `BUILD_PROGRAM`, `PROGRAM_ENTRY_SYMBOLS` and the header table (:26-104).
     - `PROGRAM_ENTRY_SYMBOLS["Drive"]` gains `kDriveAutonomousReason` and `driveAutonomous`.
     - **Cases 12–13 are RETIRED, not re-pointed** (:1397-1419). Re-pointed to `tank-drive`, they would be
       exact copies of 10–11 (:1372-1395: the `PROGRAM=drive` append dropped, and the real `tank-drive`
       build alone). Each is logged with its mapping: 12's intent (a dropped append is caught by the define
       AND the beacon) lives on in 10, and 13's (the prefix, and "the library graph must link") in 11,
       because `tank-drive` now links the library graph. The NEW retirement pair (below) stands in their
       place.
     - **The self-test cases built on tank-library that add coverage are RE-POINTED to `tank-drive` /
       `SHULIB_PROGRAM_DRIVE`:** 16, case 19's define plants (:1536-1552, :1600-1604), 21, 24, and case 27's
       plants at :1870 and :1909.
       - Each is logged one-for-one with its intent.
       - A re-pointed plant that would copy an existing one is RETIRED the same way, with its mapping. :1539
         already plants `#ifndef SHULIB_PROGRAM_DRIVE`, so check each of case 19's plants against it.
     - **Case 26's mutation of `src/programs/drive_program.cpp`** (`tableSignedPort(kChassis, p)`, :1802, which the
       new file does not hold) is re-pointed to a line the new file holds that names `kMechanisms`.
     - **A NEW pair:** `SHULIB_PROGRAM_LIBRARY` planted in a `src/` file, and `make ROBOT=tank
       PROGRAM=library`. Both must FAIL, naming the retirement.
   - **`.github/workflows/ci.yml`** (:8, :109-112) and **`RESUMING.md`**'s build list (:114-140).
   - **The build-gate consequences:**
     - The builds go down by one (six today; add COMP-b's seventh and COMP-t's if they have landed).
     - `tank-drive`'s package grows: it now links `robot::Robot`, `runDriverControl` and the protections,
       where it held ten adapters.
     - Step 8 proves that `tank-drive` links `driveInitialize`, `kDriveAutonomousReason`, `driveAutonomous`
       and `runDrive`, and none of Calibrate's, COMP's, COMP-t's, the tester's or the X-drive's.
     - No build can link the `tank-library` beacon any more, because its literal is gone. The RETIRED list
       is what stops it coming back.
4. **N5 (a), if ruled** (the pieces as N5 (a) names them):
   - `include/shulib/hal/tee_char_sink.hpp` (`hal::TeeCharSink`) and `include/shulib/hal/block_char_sink.hpp`
     (`hal::BlockCharSink`), both PROS-free. They are host-tested with `FakeCharSink`/`FakeBlockSink`:
     - both outputs get every line whole and in order;
     - a refusing block sink drops and counts, and never throws;
     - nothing allocates.
   - `ProsDeviceProvider`'s optional `serialCopyFile` (default `nullptr`, byte-identical). Given a bare
     name, the provider owns the `ProsBlockSink`, the `BlockCharSink` and the tee, and its `TermSink` writes
     through them. It is host-tested through the shim: the copy holds exactly the serial bytes, and with no
     card, serial is unchanged.
   - The program passes `"drive_log.txt"` (one file per boot, as the old program's was). It holds NO
     `ProsBlockSink` of its own (P3-T3).

---

## 2. The driver file — PROPOSED, verbatim (the final form, after Part 3)

```cpp
#pragma once
//
// robot_two_driver.hpp — ROBOT TWO'S DRIVER CONTROL: what every stick and button does.
//
// This is the team's driver code (chunk T4). Every program that hands robot two to a driver runs
// THIS file for it: "shulib Drive" on its own, and the competition program's opcontrol(). To
// change what a button does, change it HERE and nowhere else. What the robot IS -- its ports, its
// cartridges, that a claw pivot is put in Hold when let go, that the claw starts CLOSED and which
// line level closes it -- is in src/robots/mechanism_table.hpp, and is not the driver's to choose.
//
// It runs on the library's front door. The program builds a robot::Robot from the calibration
// record on the SD card and hands it to bind(), once, in initialize(); opcontrol() calls run(),
// which is the library's driver loop. Nothing here draws on a screen or writes a log line: the
// program around it does both.
//
//   left stick   up / down: drive forward / back. Forward is the end the calibration record
//                calls the front (the camera end, on robot two's record of 2026-09-25).
//                Left / right does nothing: a tank cannot strafe.
//   right stick  left / right: turn
//   R1 / R2      the claw pivot on port 6: forward / back at full power while held. Let go and
//                it is put in Hold (whether Hold keeps a loaded claw up is unmeasured, HA-156)
//   L1 / L2      the claw pivot on port 7: the same
//   A            the claw: each press opens or closes it. The FIRST time driver control starts
//                in a run, the claw is told CLOSED, whatever autonomous left it at (the team
//                lead's ruling, 2026-09-26); a program with no autonomous has left it CLOSED,
//                so there it does not move. After that, a disable and the next start tell it
//                nothing. Which line level closes it is a guess (HA-159)
//   Y            field-centric re-zero. Field-centric is OFF, so Y changes nothing yet
//   let go of the sticks and the drive coasts (Coast is set on every drive motor)
//
// The library protects the drive under this file, in every program (teleop/drive_protection.hpp):
// a drive motor that stops answering is dropped, the robot drives on, and the driver is told --
// but once more than two are dead, the drive stays at 0 V for the rest of the run, every later
// start of driver control included, until the program is started again. A drive motor fighting
// its side, or drawing more than 2.4 A while it is driven, for 250 ms stops BOTH sides for 1 s,
// and then driving resumes. None of those numbers has been measured (HA-132, HA-175, HA-176).
//
// NOTHING HERE HAS RUN ON A ROBOT. It is tested on the host, through the PROS shim, against
// robot two's real mechanism table (test/robot_two_driver_test.cpp).

#include <cstddef>

#include "common/driver_controls.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/literals.hpp"

namespace shulib::bench {

using namespace units::literals;  // 12_volt, 10_ms

/// Robot two's driver: the buttons, the feel, and the loop.
///
/// ONE lives in the program's static storage for the whole run -- never on opcontrol()'s stack:
/// PROS deletes the driver-control task when the field disables the robot, and the loop holds
/// these bindings by address.
class RobotTwoDriver {
public:
    using Button = hal::ControllerButton;

    // ═══ WHICH BUTTON DOES WHAT ═══════════════════════════════════════════════════════════
    // One line per mechanism; the names are src/robots/mechanism_table.hpp's.
    static constexpr HoldControl kHolds[] = {
        {"claw pivot 6", Button::R1, Button::R2, 12_volt},  // forward / back, full power
        {"claw pivot 7", Button::L1, Button::L2, 12_volt},  // forward / back, full power
    };
    static constexpr ToggleControl kToggles[] = {
        {"claw", Button::A},  // each press opens or closes it
    };

    // ═══ THE FEEL ═════════════════════════════════════════════════════════════════════════
    // The library's default: arcade (left stick drives, right stick turns), a smooth deadband
    // of 0.05, no curve, no ramp -- numbers no driver has felt yet (HA-112). Field-centric is
    // OFF: ArcadeBody, not ArcadeField.
    static constexpr teleop::DriverProfile kFeel{.mode = teleop::DriveMode::ArcadeBody};

    static constexpr teleop::DriverLoopConfig kLoop{
        .tickPeriod = 10_ms,                        // the brain's tick
        .rezeroButton = Button::Y,                  // field-centric's "away from me"
        .releaseBrakeMode = hal::BrakeMode::Coast,  // let go of the sticks: the drive coasts
    };

    // ═══ HOW IT RUNS -- nothing below changes when a button does ══════════════════════════

    /// initialize(), once, right after the robot is built. False, with the reason in `why` and
    /// nothing bound, when the robot is not the one these controls are for. A mechanism it
    /// declares but could not build (a motor unplugged) gets no binding: its buttons do
    /// nothing, and the drive still runs. Call it inside the boot's try: a binding can throw.
    bool bind(robot::Robot& robot, char* why, std::size_t cap) {
        return bindings_.bind(robot, kHolds, kToggles, kLoop.rezeroButton, why, cap);
    }

    /// opcontrol(): drive until the field disables the robot, then return (PROS calls
    /// opcontrol() again when driver control comes back). `pacer` is the brain's tick,
    /// `disabled` asks the field, and `perTick` is the program's screen (it may do nothing; one
    /// that takes a fourth argument is also handed the drive protections' state).
    template <class IsDisabled, class PerTick>
    void run(robot::Robot& robot, motion::ITickPacer& pacer, IsDisabled&& disabled,
             PerTick&& perTick) {
        teleop::runDriverControl(robot.chassis(), controller_, pacer, feel_, kLoop,
                                 bindings_.holds(), bindings_.toggles(), disabled,
                                 [&robot] { return driverHeading(robot); }, perTick);
    }

    /// What bind() bound, for a program's screen and boot lines.
    const DriverBindings& bindings() const noexcept { return bindings_; }

    /// Forget every binding. A program whose boot caught a throw calls this before it
    /// destroys the robot, so nothing is left pointing at it.
    void unbind() noexcept { bindings_.unbind(); }

private:
    hal::pros::ProsController controller_{hal::pros::ControllerId::Master};  // the driver's
    teleop::DriveShaper feel_{kFeel};  // kept for the run: field-centric's zero survives a disable
    DriverBindings bindings_;
};

// ═══ THE CHECKS: `make` refuses a set of controls that could not work ═════════════════════
static_assert(controlButtonsDistinct(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles),
              "a button does two jobs (or a pivot's forward is its back)");
static_assert(controlsAvoid(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles,
                            RobotTwoDriver::kLoop.rezeroButton),
              "a mechanism's button is the re-zero button");
static_assert(holdVoltsValid(RobotTwoDriver::kHolds), "a pivot's volts are not in (0, 12]");
static_assert(controlsNameTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles,
                                kRobotTwoMechanisms),
              "a control names a mechanism robot two does not have, of the wrong kind, or twice");
static_assert(controlsCoverTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles,
                                 kRobotTwoMechanisms),
              "one of robot two's mechanisms has no control");
static_assert(RobotTwoDriver::kLoop.tickPeriod
                  == units::Time{hal::pros::ProsTickPacer::kTickMs / 1000.0},
              "one tick is not the brain's");

}  // namespace shulib::bench
```

**What changed against the 2026-09-26 proposal, and why (the rulings only):**
- **The caller sentence** names "shulib Drive" and the competition program (R3, R5). No slot numbers are
  given, because the file is quoted in a public document.
- **The protections paragraph** is new (R4). It is added by Part 2 and kept true by a source pin, P2-18.
  - It is written for N7 (c), the recommendation.
  - Under N7 (a), its second clause reads: "-- but once a side has fewer than three left (on this robot,
    the second dead on one side) or more than two are dead in total, the drive stays at 0 V for the rest
    of the run, …".
  - Under N7 (b): "a drive motor that stops answering is dropped, the robot drives on however many are
    dead, and the driver is told."
  - The duration is the REAL one, "the rest of the run", because the marks live in the groups (P2-7). The
    first draft's "until the field disables the robot" was wrong (the re-check's finding 3).
  - "While it is driven" is N2's command-floor gate on the over-current cut.
  - If N1 is ruled (b) or N3 is numbered otherwise, this paragraph changes with it.
- **`run()`'s doc** names the optional fourth argument (Part 2).
- **No code line changed.**

**What a student defends, top to bottom:**
- The header says what every control does, and what the library's protections do to the drive.
- `kHolds` and `kToggles` are the only lines that change when a button does; `kLoop.rezeroButton` too,
  if the re-zero moves.
- `kFeel` and `kLoop` are the feel. `kLoop` leaves `protection` at the library's defaults: the protections
  are the library's, not this file's.
- `bind()` hands the controls to the library-side binder, which finds each mechanism BY NAME. It refuses
  a robot these controls are not for, skips an unplugged one, and starts the claw at the level the robot
  was built at. So nothing here restates CLOSED, and a polarity flip at Station J.9 needs no edit here.
- `run()` IS the library's loop, with its arguments in plain sight.
- The six `static_assert`s make `make` refuse a set that could not work.

The generic half — the binder, the refusals, the storage, `driverHeading()` — is in
`src/common/driver_controls.hpp` and never changes when a robot does. **A robot with no toggle (or no hold)**
drops that array and passes `{}` in its place, in `bind()` and in the checks. That is a file change the
compiler walks you through: C++ has no empty `[]` array.

---

## 3. Tests (adversarial) and mutations

**Part 1a — `test/robot_two_driver_test.cpp`**, through `ProsDeviceProvider` + the PROS shim, with the
REAL `kRobotTwoMechanisms` and the 2026-09-25 record (the fixture of `test/teleop_claws_test.cpp`
:91-230, binding through `RobotTwoDriver`):
- **T1 the right ports.**
  - R1 alone ⇒ port 6 at +12000 mV in the tick read, port 7 untouched; R2 ⇒ −12000; L1 ⇒ port 7 at
    +12000; L2 ⇒ −12000.
  - Each release is ONE `brake()` under Hold. The shim's `lastVoltageMv` keeps the last value after a
    brake, so assert `brakeCalls`, not 0 mV.
  - The eight drive ports get bit-identical mV to a run with the same sticks and no button, and the
    drive really moved.
  - `bindings().holdFor("claw pivot 6")` is the binding that moves port 6, and `holdFor("claw pivot 7")`
    the one that moves port 7: press through each and see ITS port move. `holdFor("claw")` and
    `holdFor("intake")` are null.
- **T2 the claw.**
  - Line A's first level EVER is the start level, and the first driver-control entry sends the same
    level.
  - A press flips it once; holding does nothing more.
  - A disable writes nothing, and a second `run()` after a disable writes nothing at entry.
  - `bindings().toggleFor("claw")` is the binding that drives line A; `toggleFor("claw pivot 6")` is null.
- **T2c the START level, not the SAFE one.** Boot from a COPY of the table whose claw entry has `.start =
  Open` and `.safe = Closed` (through `decideBootSourceFrom`, T10's rig). The real table has both CLOSED
  (`mechanism_table.hpp` :289-290), so a binder reading `pneumaticSafe` passes T2, T2b and T3; T3's
  `.openCommand` flip moves both levels together. Here:
  - line A's first level EVER, and the first entry's level, are OPEN;
  - the first press closes it.
- **T2b the ruling's pin** (the team lead, 2026-09-26: "starts closed — whatever autonomous left it at").
  - After `bind()`, set the claw to the OTHER level through `robot.pneumaticMechanism("claw")->set()`,
    as an auton would. The first `run()` writes CLOSED exactly ONCE, and a second `run()` after a
    disable writes nothing.
  - And: set the line OPEN BEFORE `bind()`. The toggle still starts CLOSED. This pins the config read,
    and makes the call-order hazard of the old landmine 3 a test, not a rule.
- **T3 the claw follows the robot, not a restated CLOSED.** Boot from a COPY of the table whose claw
  entry's `.openCommand` is flipped (Station J.9's fix). The first level and the entry level are both
  the flipped one, and the first press still flips once.
- **T4 an ABSENT pivot** (port 7 unplugged). `bind()` is true with one hold. L1/L2 command nothing
  anywhere; R1 still drives 6; the drive runs. The same with port 6 unplugged.
- **T4b an ABSENT claw.** A `FakeDeviceProvider` with `refuseLine(1)` (the brain's provider refuses only
  a line outside 1..8, which the config refuses first; test 1f's rig). `bind()` is true with 2 holds and
  0 toggles; A commands nothing; the drive runs. This carries 1f's intent before 1f retires.
- **T5 the feel is the library's default, field-centric OFF.**
  - The drive ports' mV equal a direct `runDriverControl` with `DriverProfile{}` on the same inputs.
  - With the shim IMU at 90° and trusted, a forward stick still drives both sides forward (body frame).
- **T6 release = Coast, disable = Brake.** After the sticks are let go, every drive motor is in coast at
  0 V, every tick. On a disable, the library's `cancel()` brakes (0 V + Brake, DEFECTS2).
- **T8 `bind()` twice** ⇒ exactly 2 holds and 1 toggle, no stale entry; **`unbind()`** ⇒ empty spans.
- **T9 through the driver file, T3's rules hold.**
  - A controller drop with R1 held ⇒ Hold once, and R1 held after the reconnect drives again.
  - A claim left on pivot 6 before `run()` is released at entry with one `[WARN][TEL]` line, and R1
    drives from tick 1.
  - `bindings().anyReleasedLeftoverClaim()` is TRUE after that entry, and FALSE after an entry with no
    leftover claim. COMP's claw row reads it.
- **T10 a robot these controls are not for is REFUSED** (1i's intent; each through `decideBootSourceFrom`
  with a table COPY). Every case leaves nothing bound, and a clean `bind()` works afterwards:
  - without "claw pivot 7" → `a control names 'claw pivot 7', which the robot does not declare`;
  - "claw pivot 7" as a pneumatic entry → `'claw pivot 7' is not a motor mechanism: a hold control
    needs one`;
  - an extra motor entry "intake" on port 8 with no control → `the robot declares 'intake', and no
    control names it`.
- The interim tie's two `static_assert`s (Part 1a item 3).

**`test/driver_controls_test.cpp`** (PROS-free: `FakeDeviceProvider`, hand-built controls):
- **Every predicate broken one way at a time**, each of these:
  - a button in two holds;
  - a pivot's forward = its back;
  - a toggle on a hold's button;
  - a control on the re-zero button;
  - volts 0 / 12.0001 / NaN / −1;
  - a name absent from the table;
  - a hold naming a pneumatic entry;
  - a toggle naming a motor entry;
  - two controls on one name;
  - a table entry with no control;
  - and robot two's real set passing all five.
- **`DriverBindings` directly:**
  - a clash the driver file's asserts would refuse (a toggle on the re-zero button), handed in at run
    time ⇒ `bind()` false with `validBindings`' sentence;
  - a 13 V hold THROWS `PreconditionError` (1m's intent — why every program binds inside its try), and
    `unbind()` then leaves nothing;
  - more controls than `config::kMaxMechanisms` ⇒ refused.
- **T7 `driverHeading()`, one term flipping at a time** (`FakeImu::setReady`, `robot.faults().raise`):
  - (a) the localizer `Uninitialized` with the IMU READY ⇒ untrusted;
  - (b) the localizer live, then the IMU set NOT ready ⇒ untrusted;
  - (c) live, ready, one IMU_LOST raised ⇒ untrusted;
  - (d) live, ready, no fault ⇒ trusted, and `radians` equals the pose's heading.

  The heading rule has NO test today (both copies live in TUs the host cannot compile). This is its
  first.

**Part 1a mutations** (each on a FRESH binary, observed red, restored, logged):

| Mutation | Caught by |
|---|---|
| swap R1/R2 in `kHolds[0]` | T1 |
| swap the two holds' names | T1 |
| toggle initial level `!toLineValue(start)` | T2 |
| initial level = `p->commanded()` | T2b's bind-after-set case |
| a hard-coded `false` initial level | T3 |
| toggle initial level from `pneumaticSafe` | T2c |
| `holdFor()` returns the first hold whatever the name | T1 |
| `toggleFor()` returns null | T2 |
| `anyReleasedLeftoverClaim()` returns false | T9 |
| drop the null check before the hold `emplace` | T4, crash |
| drop the null check before the toggle `emplace` | T4b, crash |
| treat an undeclared name as absent | T10 |
| drop the kind check | T10: the hold falls to `mechanismAt()` == null and is skipped |
| drop the coverage check | T10 "intake" |
| `kFeel.mode = ArcadeField` | T5 |
| `releaseBrakeMode = Brake` | T6 |
| drop EACH `driverHeading()` term | T7 a / b / c, one each |
| drop `unbind()` at the top of `bind()` | T8 |
| `run()` handing `{}` for the toggles | T2 |
| drop `validBindings()` from `DriverBindings::bind()` | driver_controls_test's run-time clash |
| a word reading the wrong entry | the J.3.1 pin |

**Compile mutations** — the build must FAIL with the named message:

| Mutation | Named message |
|---|---|
| `rezeroButton = Button::A` | "a mechanism's button is the re-zero button" |
| `"claw pivot 66"` | the naming message |
| `13_volt` | "a pivot's volts are not in (0, 12]" |
| port 7's line removed | "one of robot two's mechanisms has no control" — the ONLY one |
| a hold naming `"claw"` | the naming message |
| R1 twice | "a button does two jobs" |
| `.tickPeriod = 20_ms` | "one tick is not the brain's" |
| `kHolds[0].forward = Button::X` with the table unchanged | the interim tie's message |

**Stated EQUIVALENT, not counted, each logged with its reason:**
- A per-run `DriveShaper` (constructed inside `run()`), and a constant heading lambda. Both are
  unobservable while `kFeel` is ArcadeBody with no slew: the heading is ignored, and the zero offset is
  not in `DriveShaperState` (:62-75). **A field-centric test through the driver file is OWED the day
  field-centric becomes selectable** (§8).
- `validBindings()` is unreachable for the REAL set: the compile-time checks refuse everything it would.
  It is reachable and tested through `DriverBindings` directly.

**Part 1b** — re-pointed, never weakened:
- **`test/mechanism_table_test.cpp`:** checks 4-7's cases move to `driver_controls_test`; the rest is
  unchanged.
- **`test/boot_source_test.cpp`:** hand-built tables lose four fields; every decision pin is unchanged.
- **`test/teleop_claws_test.cpp`:**
  - **every WORD pin is unedited and green**;
  - the fixture binds through the driver;
  - test 1l's SOURCE pins are rewritten one-for-one for the new wiring, each old pin mapped to its new
    one in the log;
  - 1f, 1i and 1m retire only with a logged account of what each covered and what covers it now (T4b,
    T10, driver_controls_test).
- **Mutations:** `st.driver.bind(` moved outside the try (1l red); the re-zero word read from a constant
  other than `kLoop.rezeroButton` (1l red); an edited string literal in `teleop_program.cpp` (the diff
  check).

**Part 2 — the protections, on the host test plant** (`test/drive_protection_test.cpp`, plus cases in
`test/motor_group_test.cpp` and `test/coupled_side_monitor_test.cpp`).
- **The fixture.** A `robot::Robot` built by `robot_builder_test.cpp`'s `BuiltRobot` pattern: a config
  over the `FakeDeviceProvider`, with each tank side's extra members hung on the plant's coupled train
  (:225-262). It is driven by `runDriverControl` with a `FakeController` and the plant-stepping pacer
  (`teleop_loop_test.cpp`'s).
- **Where the faults come from.** Member faults are the plant's `setMemberFault`. Currents are
  `FakeMotor::setCurrent`, because the plant synthesizes none; state that in the file.
- **The hand-stepped rig** (for what the plant cannot shape). The plant has no mass (`drive_plant.hpp`
  :172): its train reverses in one tick and can never hit a wall. So P2-6b, P2-6d and P2-19 use no plant.
  A `perTick` writes every member's next `position`/`velocity`/`current` into its `FakeMotor` by hand, from
  a table in the test, with the pacer not stepping anything.
- **What each case carries.** The bug it catches, and the mutation that turns it red.
- **P2-1 healthy.** Run 20 s of full-stick forward, turn and reverse sweeps with every member healthy.
  - Every member draws about 1 A (`setCurrent`), so each side's TOTAL is about 4 A, above 2.4 A, while
    every member is below it.
  - Expect zero cuts, zero FROZEN, zero absent members and no protection line.
  - **And the whole pre-existing suite is unchanged**: its assertion count is recorded before and after in
    the log.
- **P2-2 a FIGHT.** Put `MemberFault::Reversed` on one member of the LEFT side under +V.
  - The group's monitor persists at its 25th tick, and that tick writes 0 V to every drive motor.
  - The drive motors read exactly 0.0 V from that tick through the 100th tick after it. The stick drives
    again on the 101st.
  - ONE cut line names the port, the cut count is 1, and one re-arm line is logged.
  - With the fault kept, cut #2 comes exactly `persistTicks` after the stick drives again: the streak
    starts over. Pin the exact tick numbers.
- **P2-3** `MemberFault::SignFlipped` (read side only) on a member of the RIGHT side: the same. So a
  protection that judges only the first group fails here.
- **P2-4 over-current, at its boundaries** (the side driven above 1 V throughout, unless stated).
  - One member at 2.41 A for 24 ticks: no cut. For 25 ticks: a cut.
  - 2.40 A for 1000 ticks: never (the rule is strictly above).
  - One tick at 2.39 A resets the streak, and the streak starts over at a re-arm.
  - The 25-tick case once more on a member of the RIGHT side: a cut.
  - A runtime-ABSENT member at 2.5 A never cuts.
  - A member at 2.5 A with the stick RELEASED (its side at 0 V) for 1000 ticks never cuts: the streak
    counts only above the 1 V command floor.
  - With `overCurrentCuts = false`: logged and counted as a WARNING, never a cut (N1).
- **P2-4b a FROZEN current** (the re-check's finding 2; the probe's scenario G).
  - Every member is set to 2.5 A under full stick for 25 ticks: cut #1 (N1's push cut, as ruled).
  - During that cut, member X freezes (the plant's Frozen) with its current held at 2.5 A. The mates drop
    to 0.6 A.
  - After the re-arm, the stick is released for 2 s, then the robot drives away at half stick.
  - Expect cut #1 only. There is no cut while it stands (the command floor), and none when it drives away:
    X is FROZEN on the first tick a moving mate passes 1 rad/s, and ABSENT 24 ticks later. Pin the ticks.
- **P2-5 runtime absence, a member frozen AT REST and then driven.**
  - It is FROZEN on the first tick a moving mate reports above 1 rad/s, because its position has repeated
    far more than 5 reads. It is ABSENT 24 ticks later, with ONE line naming the port. Pin both ticks.
  - **No cut, and `MOTOR_GROUP_DISAGREE` raiseCount == 0**: it left the monitor on the first tick its mates
    turned. Run at least 50 ticks past the latch, so a `markAbsent` that left it in the monitor would
    raise.
  - The group's reads exclude the member, and its FakeMotor still receives the side's volts.
- **P2-6 THE RACE** (kept exactly as specified; the first proposal failed it). The member freezes while
  TURNING (the plant's Frozen at +v), and the stick is reversed on the same tick.
  - Expect ZERO cuts. The member's monitor streak (`disagreeTicks`) is never above `frozenSuspectReads`.
  - It is ABSENT 24 ticks after it was first FROZEN. On the plant that is the reversal + 29 (the probe);
    pin the exact ticks.
- **P2-6b THE ORDER** (hand-stepped).
  - Member X reads OPPOSITE to its side's command (a real fight) for 20 ticks. Then its reads freeze:
    position, velocity and current held, as ProsMotor's last-good screen holds them.
  - At its 5th repeated read, its monitor streak would reach 25.
  - Judged before `drive()`, X is FROZEN and out of the monitor THAT tick: no cut, and ABSENT 24 ticks
    later. With the FROZEN exclusion applied after `drive()`, the monitor persists that tick and CUTS.
- **P2-6c after a cut, reversed at the re-arm** (the re-check's finding 1, test (a); the probe's H).
  - X freezes at +v two ticks before an over-current cut on a mate (set above 2.4 A for 25 ticks while
    driving). The train stands for the cut, while X's stale speed stays above 0.5 rad/s.
  - Expect X ABSENT DURING the cut, at the pinned tick: SUSPECT through its own stale speed, FROZEN
    because its standing mates are not.
  - The driver reverses at the re-arm: no second cut.
- **P2-6d a deceleration through zero** (hand-stepped; the re-check's finding 1, test (b)).
  - X is held at +v. The command is reversed, and its mates decelerate linearly through zero over 30 ticks
    (below 1 rad/s for at least 6 of them), then on to −v.
  - Expect X FROZEN from its 5th repeated read and on EVERY tick after, zero cuts, and ABSENT at the pinned
    tick. While its mates pass below 1 rad/s, its own stale speed keeps it SUSPECT.
- **P2-7 the floor, per N7.**
  - **(c):** a four-member side loses two members mid-run (2/4). It is DEGRADED, drives on, and logs no
    REFUSE line. Then a third death, on the other side, is REFUSED: exactly 0 V from that tick until the
    disable, with ONE `REFUSED` line.
  - **(a):** the 2/4 loss itself is REFUSED, as just described.
  - **(b):** three deaths are DEGRADED, and the robot still drives.
  - **(a) and (c):** after a disable, the next entry is REFUSED at entry, 0 V from tick 1, because the
    marks live in the groups. That is the "rest of the run" §2 states.
- **P2-8 port 18's slip.** `MemberFault::Decoupled` at ~80 % (HA-130/133): never absent and never a cut,
  over a long run.
- **P2-9 the median when a member drops.** A Decoupled member spreads the side, then a member is marked
  absent mid-run.
  - The group's position moves by no more than the gap between its two middle members.
  - The odometry raises NO `ODO_STUCK`.
  - If this goes red, the fix is a continuity offset inside MotorGroup at the change, never a looser gate.
- **P2-10 `markAbsent` itself** (`motor_group_test`):
  - the member is excluded from every read;
  - it is still commanded (`FakeMotor::commandedVoltage` follows `setVoltage`);
  - the monitor is told `present = false`;
  - it REFUSES (false, nothing changed, no throw) for out of range, a second call, and the LAST present
    member;
  - a group with no marks is bit-identical;
  - **`setMonitorExcluded`:** an excluded member is fed `present = false`, stays in every read, and is
    still commanded. Bits past the range, and on holes, are ignored. A mask of 0 is bit-identical.
- **P2-10b the `FrozenReadingDetector` alone** (`coupled_side_monitor_test`), each rule at its boundary:
  - 4 repeated reads vs 5;
  - own speed 0.50 vs 0.51 rad/s;
  - a moving mate at 1.00 vs 1.01 rad/s;
  - a mate whose own position repeats is NOT counted as moving;
  - the whole side SUSPECT means nobody is FROZEN;
  - FROZEN for 24 ticks vs 25;
  - a latched member stays latched;
  - its precondition (`suspectReads` 0, and `suspectReads == persistTicks`).
- **P2-11 a disable during a cut.** `cancel()` (0 V + Brake) returns at once. The next entry has cut
  count 0 and is not cut; the marks are kept.
- **P2-12 the log bound.** 60 cuts give 50 per-port blocks and the notice exactly once; the count reads
  60.
- **P2-13 the X-drive (no groups).** Over-current cuts per bare drive motor. There is no absence or fight
  path, nothing crashes, and the line names the wheel index.
- **P2-14 perTick.** A four-argument `perTick` receives the state: mode `Cut`, with `cutTicksLeft`
  counting down, during P2-2. A three-argument `perTick` compiles and sees exactly what it saw before.
- **P2-15 the controller disconnected during a cut.** Still 0 V, and nothing is double-counted.
- **P2-16 the disable summary.** Each member's peak current and temperature this entry, and the cut count.
- **P2-17 labels.** A builder-made group names its members by port. A hand-made group with no labels says
  "member i".
- **P2-18 the driver file's protections paragraph** (a source pin over `src/team/robot_two_driver.hpp`). Each
  number it states is checked against its source:
  - "2.4 A" against `DriveProtectionConfig{}.overCurrentAmps`;
  - "250 ms" against `overCurrentTicks` × the tick;
  - "1 s" against `cutTicks` × the tick;
  - "more than two" against `kMaxDeadTotal` (N7 (a) and (c));
  - "three" against `kMinAnsweringPerSide` (N7 (a) only);
  - "the rest of the run" present whenever the paragraph names a stop (N7 (a) and (c)).

  A changed default fails it.
- **P2-19 THE WALL** (hand-stepped; the probe's own finding). Every member of a side stops at once under
  full stick, while each reported velocity decays over 300 ms. Expect nobody FROZEN, nobody ABSENT, no
  DEGRADED line and no cut. A live train that stops is never a death.
- **P2-20 the config's entry check.** Each of these is a `PreconditionError` before tick 1, with nothing
  commanded (every drive `FakeMotor`'s write count is 0):
  - a NaN `overCurrentAmps`;
  - `cutTicks = 0`;
  - `overCurrentTicks = -1`;
  - `frozenSuspectReads = 25` (not below `persistTicks`).
- **The shared panel (host):**
  - the PROTECTION row's words for every mode;
  - the LCD chooser gives REFUSED > CUT > fault for every pair of states in force at once;
  - every literal is within its budget, ASCII, with no `%`.

**Part 2 mutations** (each on a fresh binary, observed red, restored, logged):

| Mutation | Caught by |
|---|---|
| the FROZEN exclusion applied AFTER `drive()` | P2-6b (a cut) |
| SUSPECT after `persistTicks` reads (the first proposal's length) | P2-6 (a cut) |
| the mates' fastest counting a member whose position repeats | P2-6c (X is not ABSENT during the cut; the latch tick moves) |
| SUSPECT only through the mates (the own-speed branch dropped) | P2-6d (X leaves FROZEN at the turnaround; the latch tick moves) |
| FROZEN without the other-member guard | P2-19 (live members marked ABSENT) |
| a FROZEN member's current counted | P2-4b (a second cut) |
| over-current counted at or below the 1 V command floor | P2-4's released-stick case, P2-4b (a cut at rest) |
| over-current judged on the side's `totalCurrent()` | P2-1 (a cut) |
| the protections judged on the first group only | P2-3, P2-4's RIGHT case |
| the config's entry check dropped | P2-20 |
| over-current `>=` | P2-4's 2.40 A case |
| `overCurrentTicks` 24 | P2-4 |
| over-current streaks not reset at re-arm | P2-4 / P2-2 spacing |
| `cutTicks` 99 | P2-2 |
| no 0 V write on the deciding tick | P2-2 (one tick of stick volts) |
| the cut handing `drive()` the stick command | P2-2 |
| an absent member's current counted | P2-4's absent case |
| `markAbsent` leaves the member in the monitor | P2-5 (MOTOR_GROUP_DISAGREE) |
| `markAbsent` leaves it in the median | P2-5's reads |
| `markAbsent` stops commanding it | P2-5 / P2-10 |
| `markAbsent` allows the last member | P2-10 |
| the REFUSE not re-applied at entry (N7 (a)/(c)) | P2-7 |
| the per-side floor applied mid-run under N7 (c) | P2-7's 2/4 case |
| the frozen-reading rule dropped | P2-6 (the member is missed, and it cuts) |
| the log bound off by one | P2-12 |
| the fourth `perTick` argument never passed | P2-14 |
| `overCurrentCuts` ignored | P2-4's warn case |
| the defaults' paragraph drifted (write "2.5 A") | P2-18 |

**Stated EQUIVALENT, not counted:** each group's `resetDisagreement()` at the re-arm. The monitor already
resets every streak on each tick the side is commanded 0 V (`coupled_side_monitor.hpp` :117-125), so
dropping it is unobservable. It is kept for the old program's stated intent (:452-456).

**Part 3** (the program TU is not host-compiled: `pros/screen.h` has no shim):
- **P3-T1 the gate.**
  - The re-pointed cases (16, case 19's define plants, 21, 24, 26, case 27's plants at :1870 and :1909)
    fail or pass exactly as their tank-library forms did, each logged.
  - 12–13 are RETIRED with their mapping to 10–11 logged, and 10–11 are still green. The same holds for
    any plant retired as a copy.
  - The NEW pair: `SHULIB_PROGRAM_LIBRARY` planted in `src/` makes step 0 red, naming the retirement; `make
    ROBOT=tank PROGRAM=library` hits the `$(error)`, naming the retirement.
  - `main.cpp`'s Drive case calling `runR3a`: step 8 red.
  - The tank-drive branch carrying `tank`'s beacon: the NUL match fails it.
- **P3-T2 the pure panel model** (host):
  - MAIN and DETAILS rows from snapshots: every PROTECTION state, the source rows, the claw row and the
    fault row;
  - MAIN holds exactly the header, the PROTECTION row, the controller row and `DETAILS`. R5's four rows are
    on DETAILS' first page (R5);
  - the LCD chooser's full order, every pair: REFUSED > CUT > fault > no controller > DEGRADED > CLAW
    ABSENT > BACKUP > ok. BACKUP is pinned if SAFE-SAVE B2 has landed; otherwise B2 pins its pairs when it
    adds the word;
  - the page flip happens on the RELEASE edge only; a press held across a flip does nothing; the arming
    delay holds;
  - every literal within budget, ASCII, no `%`.
- **P3-T3 source pins** (code-only text, T3 Part 2's test 1l pattern):
  - **In `src/programs/drive_program.cpp`:**
    - `st.driver.bind(` appears once, inside the try, right after the Robot's `emplace`;
    - `st.driver.unbind()` is first in its catch;
    - `st.driver.run(` appears once, in `runDrive`;
    - the refusal path idles;
    - there is no `chassis()` in `driveAutonomous` and no `clawBannerLine(`;
    - **none of the old program survives by accident**: no `mapSticks(`, `tankSideVolts(`, `ProsMotor`,
      or `ProsBlockSink`. Under N5 (a) the SD copy is the provider's, reached only by passing the bare
      name `"drive_log.txt"` to its constructor, which the pin also checks. Under (b) there is no copy
      and no file name;
    - `competition_is_disabled` appears only in the disabled lambda.
  - **Across `src/`:**
    - `teleop::runDriverControl(` is called only by the driver file and `src/programs/xdrive_program.cpp`;
    - `MechanismBinding`/`ToggleBinding` are constructed only in `src/common/driver_controls.hpp`;
    - the heading rule's text appears only in `src/common/driver_controls.hpp`;
    - `teleop_program`, `TeleopClaws`, `runTeleop`, `teleopInitialize` and `SHULIB_PROGRAM_LIBRARY` appear
      nowhere (the RETIRED list);
    - no string LITERAL in `src/` or `include/shulib/` says "shulib Teleop" or "(slot 2)" (the re-check's
      finding 4: Calibrate's save line, the X-drive's refusal, `formatSaveReport`).
- **P3-T4 the moved words.** Every surviving word pin from `teleop_claws_test` is unedited and green in its
  new home, and each reworded pin is logged one-for-one.
- **Part 3 mutations:**
  - each P3-T1 plant is run for real;
  - the program's `run` swapped for a direct `runDriverControl` (P3-T3 red);
  - `bind` moved outside the try (P3-T3 red);
  - the flip on PRESS (P3-T2 red);
  - CUT ranked below a fault in the chooser (P3-T2 red);
  - a leftover `#include "programs/teleop_program.hpp"` in `main.cpp` (the build and P3-T3 red).

---

## 4. Documentation (same change, per part)

Every part's prose review greps `docs/` and `README.md` for the changed files' names and concepts, and
reads every hit. **Part 3's also greps `src/` and `include/shulib/`**, because it retires a program that
runtime strings and header comments name. It is logged BEFORE the commit body claims it. `docs/api` and
the briefing are regenerated LAST.

**Part 0:** as listed in §1. That is COMP B5/B29/§3 item 3; `build-order.md` (:1472, the T4 entry, the
count, the first `Next:`, the deviations rows, G3a's line); worksheet Station A; roadmap; HANDOFF. **Five
documents.**

**Part 1a:**
- **`docs/cookbook/06-driver-control.md`** — a closing section, "A whole driver file: robot two's". It
  quotes §2's controls, feel, `bind()` and `run()` VERBATIM, checked by `check-examples` through item 6.
  It says what each part is for, that the binder lives in `src/common/driver_controls.hpp`, and that nothing of
  it has run on a robot.
- **`COMP-competition-program.md` — B11 REPLACED** with: *"opcontrol() IS the driver file.
  - The program holds a `RobotTwoDriver` (`src/team/robot_two_driver.hpp`) in its static state.
  - `initialize()` calls `st.driver.bind(*st.robot, why, sizeof why)` inside the ONE try, right after
    the Robot's emplace. False ⇒ the refusal screen with `why`, and nothing further built. The catch
    calls `st.driver.unbind()` before it resets the Robot.
  - `opcontrol()`, after B2's wait, calls `st.driver.run(r, <pacer>, disabled, panel)`, and nothing else
    drives.
  - The re-zero, the feel and Coast-on-release are the driver file's constants, never restated here.
  - The pacer is COMP-b's choice (the provider's, or the guard, which passes through outside a run —
    say which).
  - **The driver-control screen is built from T4's shared PURE rows.**
    - Its rows and the controller-screen lines come from PURE, PROS-free functions in one shared `src/`
      header (proposed `src/common/driver_panel.hpp`), host-tested.
    - COMP's page is the header, the claw row, the fault row and the controller row. The PROTECTION row
      joins at T4 Part 2.
    - The new "shulib Drive" composes its MAIN and DETAILS pages from the same rows (T4 Part 3, the team
      lead's R5), and a later robot console can stream them (the team lead, 2026-09-27). The program only
      paints.
    - The claw row is `clawRowText(t, robot, st.driver.bindings().anyReleasedLeftoverClaim())`.
    - The controller screen's ONE chooser moves here, and so does any piece COMP needs that is still in
      `teleop_program.cpp`. They MOVE to the shared header; they are never copied. SAFE-SAVE B2 later adds
      its `BACKUP` word to that chooser.
  - The claw-port-in-record refusal stays in the shared boot source.
  - Teleop's claw boot line ends `-- in shulib Teleop only`, which is false once this program binds the
    claws: reword it and re-quote worksheet J.3.1 in the same change.
  - B10's question is ANSWERED by the team lead's ruling: the first driver-control entry closes the claw
    whatever autonomous left, and the driver file's T2b pins it. Station K's B28 step still observes it
    on the robot.
  - The driver file's header caller sentence gains this program.
  - A host source pin proves the wiring (T4's pattern).
  - The drive protections (T4 Part 2, which lands after this) reach opcontrol() through
    runDriverControl with no wiring. Until then this program's driver control has no mid-match absence
    rule and no cuts, and Station K says so."*
- **`build-order.md`:** T4 status and the first `**Next:**` if it moves. **`docs/roadmap.md`** Phase T.
  **`HANDOFF-remaining-work.md`** (L5, §7).
- **The prose review** terms: "headingSample", "cannot move when driver control", "which button".

**Part 1b:**
- **Guide chapter 10** ~:177-214: the driver file is the worked example, and "shulib Teleop" runs it
  until the driver program replaces it (:210's L1 default sentence stays true).
- **Chapter 14** ~:331-350 (status).
- **Guide 07 :130**, which says `src/programs/teleop_claws.hpp` is Teleop's claw wiring.
- **HA-153's source text** (`hardware-assumptions.md` ~:1695-1700: "binds R1/R2 and L1/L2 to them
  (`src/programs/teleop_claws.hpp`)") → the driver file.
- **Worksheet Station J:** J.4's "the fix is ONE sign … never swapping R1 and R2" now also says where the
  buttons live. Station F row 4's "a constant in the code" names `kFeel`. The serial-capture comparison of
  1b item 3.
- **The parked `.claude/coordinator/drafts/T3-COMPLETED.draft-for-when-macros-land.md`** re-pointed
  (§8).
- **The interim sentences Part 1a wrote** go false in 1b, so 1b removes each one *(added 2026-09-27 by
  Part 1a's review; T4-PROGRESS "Part 1a")*: the driver file header's "there too" clause (§1 Part 1a
  item 2, the 1b bullet); cookbook chapter 6's "Watch out for" bullet, "Until the buttons leave the
  mechanism table, change a button in both files — and the re-zero button in three …"; and HANDOFF
  L5's "robot two's buttons are ALSO stated in its driver file … tied to the table's at compile time
  in Teleop until T4 Part 1b".
- `build-order.md` / roadmap / HANDOFF status. The prose review terms: "TeleopClaws",
  "kTeleopRezeroButton", "re-zero", "which button", "teleop_claws".

**Part 2:**
- **`docs/api`** is regenerated, with `check-coverage` run. The new `drive_protection.hpp` is documented
  member by member, as are `motor_group.hpp`, `coupled_side_monitor.hpp`, `teleop_loop.hpp` and
  `drivetrain_degradation.hpp`.
- **`docs/changelog.md`:** one additive entry covering `DriverLoopConfig::protection`
  (`frozenSuspectReads` included), `DriveProtection` and its State,
  `MotorGroup::markAbsent`/`runtimeAbsentMask`/`setMonitorExcluded` and its labels,
  `teleop::FrozenReadingDetector`, N7 (c)'s `evaluateRuntimeDegradation` if ruled, and `perTick`'s
  optional fourth argument. Follow the version rule `version.hpp` states. Say
  that NO frozen interface changes: IMotor (F4) is untouched (N2).
- **`docs/hardware-assumptions.md`:** HA-175, HA-176, HA-132/133 amended, and the status paragraph (:60-84).
- **Guide 10** (the driver-control section ~:177-214): what the protections do, and what a driver sees.
  **Guide 12:** a new symptom, "The drive stops for a second, or a motor is reported dead", written from
  the code: what the screen and the log say, and what to check. **Guide 14** (~:331-350): runtime absence
  is no longer "later".
- **The cookbook:** the driver file quote's header paragraph; `perTick`'s fourth argument in the loop
  recipe.
- **COMP:** B11's last bullet is replaced by what now holds.
- **Worksheet Station K** (COMP-b has landed, by R2's order). It gains two things:
  - one line: what a CUT, a FROZEN port or a dead port looks like in the competition program;
  - **THE PUSH CHECK, REQUIRED before the competition program drives a match** (N1 (a)). On the ground,
    full stick into a wall for 3 s. Record whether it CUT, and the highest current on serial's disable
    line. This is HA-176. From this part on, the competition program's `opcontrol()` cuts on
    over-current.
- **Worksheet Station F** (still Teleop's): the dead-port paragraph (:616-621) is corrected, since runtime
  death is now detected in every program, and a CUT shows only on serial and the fault row in Teleop.
- `build-order.md`, roadmap, HANDOFF.
- **The prose review** terms: "MemberAbsenceDetector", "dies AFTER boot", "AT BOOT ONLY", "later chunk",
  "2.4 A", "cut", "re-arm", "drive program", "runtime".

**Part 3:**
- **The worksheet:**
  - **Station F is REWRITTEN for the new "shulib Drive" (slot 1)**, from the code at the merge. It is
    already the library-drives station. **Station E**, the OLD program's, becomes a two-line pointer: the
    program it described was replaced in place at T4, and its WARNING and CUT paragraphs moved into F.1,
    reworded for the library.
  - **F.0**
    - The upload: `rm -rf bin .d && make ROBOT=tank PROGRAM=drive && pros upload --slot 1 --name "shulib
      Drive"`.
    - **REQUIRED before any wheel moves: is this the OLD or the NEW program?**
      - If the header's second line reads `adapters, NOT the motion stack`, or serial says `[R3B] booted:
        shulib DRIVE`, it is the OLD program. It drives toward the OTHER end. Stop and re-upload.
      - The NEW one says `[DRV]` and shows the run-status row.
      - The BUILD stamp is today's.
    - The refusals, re-quoted from the code.
  - **F.1** MAIN and DETAILS row by row (touch DETAILS / BACK), and the LCD. WHICH END IS FORWARD: the
    record's front is the camera end. The typed-table 180° note stays for Bench Tests' MOTOR WATCH. "Do not
    edit the table … flipping the table flips shulib Drive" goes, because the table drives nothing but
    Bench Tests now.
  - **F.2** wheels up, as the old F.2.2–F.2.4. **F.3** the claws: see Station J. **F.4** the ground, as the
    old F.2.5.
  - **F.5 the feel**, against Bench Tests' 10 DRIVE (Station D): ruling M4's raw-volts baseline, the only
    one left once Part 3 deletes Teleop and the old Drive. Its differences are stated so the comparison
    is fair:
    - it runs WHEELS UP only;
    - it needs L1 held (the dead-man);
    - it starts at a 3 V ceiling that R1 steps to 12 V;
    - it drives toward the TYPED TABLE's front, the 15/16 end, the opposite end from the record's front.

    So F.5 compares feel, wheels up, at the 12 V ceiling, and never direction. Doing the comparison before
    Part 3 was rejected: the old Drive and a driver-file Teleop drive toward opposite ends, the back-to-back
    hazard F.2.1 already removed.
  - **F.6 the protections:**
    - (i) With the team lead's OK: wheels up, half stick, pull one drive motor's cable. Expect it FROZEN
      within about 50 ms and `DEAD` about 250 ms later, logged once, and no cut. Plug it back in: it stays
      DEAD until the restart (latched). This is HA-175.
    - (ii) **THE PUSH CHECK — REQUIRED before the competition program drives a match (N1)**, as Station K
      has it since Part 2, here reading the highest A on DETAILS. This is HA-176.
    - (iii) No fight is staged on purpose. The host plant covers it.
  - **F.7** the disable line. What to send back. Under N5 (a): what the card's `drive_log.txt` holds, which
    is everything from the provider's construction on. The banner and the boot-source lines are on serial
    only.
  - **Station J** moves to "shulib Drive", slot 1:
    - the header;
    - J.0.1 ("Station F passes first");
    - J.3.1 re-quoted from the code;
    - **J.3.2 RETIRED**, because the program prints no banner claw sentence (D4);
    - every "slot 2" becomes "slot 1", J.9 included.
  - **Station G:**
    - G.1's upload block: the Drive line now builds the new program, the Teleop line goes, and the
      tester's line is COMP-t's (N4);
    - G.6 becomes "shulib Drive from the record": G.6.1's boot lines, and G.6.3 reads `est` on DETAILS.
  - **Station A**'s program list: the final map.
  - **The DO NOT list:** "Station F's wheels-up run", and the OLD-package warning.
  - **N4 (a):** the tester's slot-7 lines become slot 2.
- **`README.md`** (:194, :244-257, :292); **guide 07** (:122-131); **guide 10** (:186-190's honest status);
  **guide 14** (:337-347). Its "until a driver has compared the two on the robot" now names the
  comparison F.5 makes, against Bench Tests' 10 DRIVE.
- **`docs/hardware-assumptions.md`:** the entries naming Teleop (HA-153/154/156/159's program text) or the
  old typed-table program (:811, :1171, :1548, :1954).
- **The comments that name "shulib Teleop", "shulib Drive", "the drive program", "teleop_program" or
  "slot 2".** Each hit is reworded, or kept as TRUE past-tense history, with the reason logged.
  - **The library headers** (`docs/api` is regenerated from them):
    - `hal/pros/motor.hpp` :34; `calibration/boot_check.hpp` :10, :22 and :200;
    - `calibration/calibration_record.hpp` :10 and :81 ("shulib Teleop reads it at boot");
    - `hal/pros/device_provider.hpp` :100; `hal/device_registry.hpp` :11; `hal/pros/block_source.hpp` :6;
    - `diag/session_info.hpp` :27; `config/robot_config.hpp` :9;
    - `teleop/stick_mapping.hpp` :104 and :134; `teleop/teleop_loop.hpp` :92-96;
    - `teleop/coupled_side_monitor.hpp` :171 (`tankSideVolts` is now the tester's only);
    - `hal/motor_group.hpp` :48 and :86; `robot/robot.hpp` :18, :115 and :393.
  - **`src/`:**
    - `fault_latch_hook.hpp` :10; `state_screens.hpp` :14 and :32; `screen_text.hpp` :7;
    - `calibrate_program.cpp` :10, :47, :107 and :255;
    - `xdrive_program.cpp` :8, :31 and :98; `xdrive_config.hpp` :20;
    - `boot_source.hpp` :10 and :58; `bench_r3a.cpp` :240;
    - what 1b left in `mechanism_table.hpp`; the Makefile, gate, `ci.yml`, `main.cpp` and
      `build_selection.hpp` comments.
  - **The rule, not only the list:** `grep -rn 'shulib Teleop\|teleop_program\|Teleop\|drive program\|shulib
    Drive\|slot 2' src/ include/shulib/` at the merge. Every hit is accounted for in the log. The line
    numbers above are `37706ce`'s, and SRC1 moves the `src/` files.
- **`RESUMING.md`** (the build list :114-140) and **`ORIENTATION.md`**'s hits.
- `build-order.md` (T4's status and the first `Next:`), roadmap, and HANDOFF. HANDOFF notes that R5a Part D
  and L9 Part B now have "shulib Drive" and COMP as the callers, where they had Teleop. SAFE-SAVE B2's words
  land in "shulib Drive" and COMP. Their briefs are re-read at their go, not edited now.
- `T4-COMPLETED.md`, then the briefing LAST.
- **The prose review terms:** "shulib Teleop", "PROGRAM=library", "tank-library", "teleop_program",
  "teleop_claws", "shulib Drive", "drive_program", "drive_log", "slot 1", "slot 2", "runDrive", "adapters,
  NOT the motion stack", "typed table". They are run over `docs/`, `README.md`, `src/` and
  `include/shulib/`.
- **More than five documents in Parts 2 and 3.** It is proposed, as with COMP-b's B21, that each part's
  list is accepted as one step under pace rule 7. They can be split at his word.

**HA-175 (Part 2; per N3 — the mid-match absence rule):**
`| HA-175 | A robot-two drive port that stops answering mid-run shows through the IMotor seam as a FROZEN reading — position(), velocity() and current() repeating their last good values, ProsMotor's screen — or as a velocity of exactly 0 while its train turns; a LIVE member's position is never bit-identical for 5 consecutive 10 ms reads while its own reported speed is above 0.5 rad/s or a moving side-mate's is above 1 rad/s, nor for 30 while a side-mate still answers (a whole side that stops together — a wall, a stall, a lagging velocity report — is never one member frozen); and a side still drives acceptably with its dead dropped, up to the mid-run floor N7 rules | reasoned (the adapter's screen, hal/pros/motor.hpp ~:224-280; BLUE's 300 counts per output turn: one about every 42 ms at 0.5 rad/s, every 21 ms at 1 rad/s) + invented (the 5 reads, the 250 ms latch; the floor, R3b Part 0b) | T4 Part 3's Station F.6 (i); the DEGRADED boot with one port unplugged per side |`
- **The blast radius, in the long form.**
  - **A live member wrongly marked absent** drops out of the reads, which is harmless to the median. But
    it counts as dead, so any mid-run stop comes one death sooner.
  - **A dead member missed** stays in the fight monitor: its stale velocity can read as a FIGHT, a 1 s cut
    that repeats. Its stale current can do the same through the over-current cut.
  - **Since T4 Part 2 the floor applies MID-MATCH, in every driver-control program**, as N7 rules it. On
    robot two (four per side), the stop comes at:
    - (a) the SECOND death on one side, or the third in total;
    - (c) the third in total;
    - (b) never.
  - A stop lasts the REST OF THE RUN. Every later driver-control start is refused too, until the program
    is started again, because the marks live in the groups.
  - Too permissive a floor keeps a robot driving that drags into a curve.
- **Contained:** one config field (`DriveProtectionConfig::frozenSuspectReads`), the monitor's two floors
  it reuses, and two library constants (`drivetrain_degradation.hpp` :46, :50).

**HA-176 (Part 2 — the over-current cut):**
`| HA-176 | On robot two a drive motor above 2.4 A for 250 ms while its side is driven above 1 V means something is WRONG (a fight on the coupled train, or a stall) and not ordinary driving — and a drive motor CAN draw more than 2.4 A (the per-motor limit stays at the PROS default 2500 mA with twelve motors plugged in, HA-154) | invented (R3b Session 2's first-run number, drive_program.cpp :96; no current trace of robot two under power exists) | Station K's push check (from T4 Part 2); T4 Part 3's Station F: the highest A at full stick wheels-up, then F.6 (ii) |`
- **The blast radius, in the long form.** Too tight, and a push against a wall or a robot trips it: the
  drive is cut for 1 s of every 1.25 s, IN A MATCH, since Part 2. If VEXos lowers the per-motor limit below
  2.4 A, the cut can never fire, and a locked fight is caught only by MOTOR_OVER_TEMP (55 °C, HA-44),
  minutes later.
- **Contained:** one config field and one switch (`DriveProtectionConfig::overCurrentAmps`,
  `overCurrentCuts`, N1).

**HA-132 / HA-133 AMENDED (Part 2):**
- **Consumers.** "consumed by "shulib Drive", the tester's DRIVE station, and `hal::MotorGroup`" becomes:
  the library's drive protections in every driver-control program ("shulib Drive", the competition
  program's `opcontrol()`, and Teleop until it retires); the tester's DRIVE station; `hal::MotorGroup`; and
  the calibrator (`drivetrain_calibrator.hpp` :1811).
- **HA-132's blast radius** gains: "since T4 Part 2 a false fight CUTS the drive for 1 s, in a match; a
  stick reversal at full speed that outlasts 250 ms is the first thing to watch for".

---

## 5. OUT
- chassis macros and angle presets (T3's still-open half; §8 says where they land);
- field-centric ON;
- any change to the feel's numbers (HA-112 stays invented until a driver has felt them);
- a partner controller;
- COMP's autonomous, selector, task and watch (COMP-b's);
- **everything of COMP-t's** (R6): the students' auton file, the tester program, its build, its slot and
  its documents. T4 only names slot 2's final owner and N4's interim;
- the one-line boot (L9 Part B, D5);
- SAFE-SAVE's BACKUP words (T4 only keeps the chooser's place for B2's word, D4);
- R5a's gains line;
- G3a's generator and its marked region (R1: G3a writes it; T4 leaves the WHICH BUTTON banner for G3a to
  turn into the region's start);
- **the robot console** (R7: after G3a, on H1's wire; the coordinator's plan entry);
- a second robot's driver file;
- **the protections during AUTONOMOUS motions.** The scheduler's fault policy covers a motion
  (MOTOR_GROUP_DISAGREE continues). A runtime mark made in driver control does persist into a later
  autonomous in the same run: stated, and not acted on;
- **over-current on MECHANISM motors.** HA-154 stands: the driver is the only cut-off for a pivot held
  against its stop, and MAIN's claw row shows its temperature;
- **a new FaultCode** for a cut or a runtime death. The vocabulary is wire-stable and read by the
  blackbox; the log and the panel carry both;
- **any change to IMotor (F4)** — N2;
- moving `DriverBindings` and `driverHeading()` into the library (still a candidate for L9 Part B);
- **any other library API change.** Part 2's list and N5 (a) are exhaustive. Part 2's list includes N2's
  `FrozenReadingDetector`, `setMonitorExcluded` and `frozenSuspectReads`, and N7 (c)'s
  `evaluateRuntimeDegradation`. Part 3 may change library TEXT only, with no declaration changes:
  - the header comments §4 Part 3 lists;
  - SAFE-SAVE A's `formatSaveReport` sentences that name Teleop, if SAFE-SAVE A has landed (landmine 25);
- **any hardware claim.** Nothing here has run on a robot until Station K's log (COMP) or Station F's
  (Part 3) says so, with J's for the claws.

## 6. Checks
- The host suite green, with counts before and after in the log. **Part 2 records the whole suite's
  assertion count before any change** (the bit-identity pin).
- Every mutation RED on a fresh binary; every compile mutation's named message seen.
- `python3 tools/src_build_gate.py self-test` and `check`:
  - Parts 1a, 1b and 2 on every build that exists at the time (six, plus COMP-b's and COMP-t's once
    landed);
  - Part 3 on that set **minus tank-library**;
  - 0 disallowed warnings, every beacon, step 8.
- The ARM header gate (RESUMING's command, which Part 2's new header must pass); the CI greps (the new
  header is PROS-free).
- `api_doc_tool.py` self-test / check-coverage / check-fresh / check-examples / check-removability;
  `doc_staleness_audit.py`.
- `git status` shows nothing committed.
- The briefing regenerated LAST and checked: `build-order.md`'s first `**Next:**`, the roadmap's "You
  are here".

## 7. Landmines
1. **Teleop's words are pinned; the pins are the proof.**
   - Edit a word pin in `test/teleop_claws_test.cpp` and the "byte-identical" claim is gone. Only the
     fixture and test 1l's SOURCE pins change in 1b. A call expression changing inside a word pin (the
     `clawRowText` forwarder going) is logged, with its expected string untouched.
   - Part 2 changes exactly two Teleop literals (`kDeadPortLine1/2`), logged.
   - Part 3 rewords only the pins whose sentence names Teleop or slot 2.
2. **`-- in shulib Teleop only`** (`teleop_claws.hpp` :245) goes false the moment a second program binds
   the claws. COMP-b is first under R2: it rewords the line and re-quotes J.3.1.
3. **The claw at the first driver-control entry** — the team lead's ruling, 2026-09-26: "Driver control
   starts with the claw closed — whatever autonomous left it at; revisit when an auton first moves the
   claw."
   - The toggle's initial level is the Robot's CONFIG start level, and the first entry in a run SENDS
     it, whatever an autonomous left. T2b pins both halves.
   - When a student auton first moves the claw, the ruling is revisited. Station K's B28 step observes
     it on the robot. Changing it is a LIBRARY change (a "keep what is there" start), never a driver-file
     edit.
4. **A refused `bind()` means `run()` is never called.** The loop re-checks the set as a PRECONDITION and
   would throw inside driver control (`teleop_loop.hpp` :216-219).
5. **Static storage only, and `unbind()` BEFORE the Robot is reset in any catch** (T3's rule). The
   storage rule's text changes in 1b (no throwing constructor, no hardware touched); its intent does not.
6. **The name (R3).** The NEW "shulib Drive" has the OLD one's name, slot, `PROGRAM=drive` and beacon.
   - A brain that still holds the old package drives toward the OTHER end (the typed table's front, the
     15/16 end) on the same stick.
   - Three things tell them apart: the header's second line (`adapters, NOT the motion stack` = OLD), the
     serial tag (`[R3B] booted: shulib DRIVE` = OLD, `[DRV]` = NEW), and the BUILD stamp.
   - **Not the `[BLD]` line**, which R3's words named. `main.cpp` :131-132 prints `[BLD] robot tables: …`
     in EVERY build since `37706ce`. So an old Drive built at or after `37706ce` prints the same `[BLD]`
     line as the new one; only a package built before `37706ce` lacks it.
   - Station F.0 makes this a REQUIRED check before the wheels touch the ground.
   - Every document names the slot, the name AND this check.
7. **`…=tank` is a prefix of `…=tank-drive`.** The gate's NUL match covers it, and the self-test plants
   it rather than assuming it.
8. **Step 8 cannot see the driver file** (inline, templated); only the source pins can.
9. **The driver file is robot two's.** Only a `ROBOT=tank` build may RUN it: the chain's `#error`s keep
   every program that runs it there, and at run time `bind()` refuses any other robot by name (T10).
   It COMPILES in every build (the probe compiled it under the bench and X-drive defines), so it must stay
   free of anything that breaks another build. A second robot gets its own driver file, never this one
   reused (2c-final's "a build carrying another robot's facts", HA-111's class).
10. **The `static_assert`s name `kRobotTwoMechanisms`.** Step 0 allows that only inside a `static_assert`
    (`src_build_gate.py` :592-642). Keep it there and nowhere else in the file. The same applies to the
    interim tie in `teleop_program.cpp`.
11. **`pros::c::competition_is_disabled()` is not in the PROS shim.** The disabled predicate stays in the
    program TUs, never in the driver file.
12. **The driver file is quoted in a PUBLIC document.**
    - Its comments carry no `docs/internal` path and no claim past the evidence: "holds" is never claimed
      (HA-156), and CLOSED is a command (HA-159).
    - The protections paragraph says its numbers are unmeasured, and P2-18 keeps them equal to the
      config's.
13. **`src/` compiles with `-Wformat-nonliteral`/`-Wformat-truncation` as errors** (the gate). Each
    refusal sentence in `driver_controls.hpp` has its OWN literal format with a precision on every `%s`
    (the §0.4 sketch's shared format under a pragma is NOT acceptable). ASCII on screen; every string
    through `printText()`; only a literal `"%s"` to a PROS print.
14. **`using namespace units::literals;` sits at `shulib::bench` scope in a header.** It brings only the
    unit literal operators into every includer's `bench` namespace, which is accepted for the cookbook's
    `12_volt` / `10_ms` spelling. No other using-directive goes in the file.
15. **HA-175 and HA-176 only**, per N3. HA-177/178 are COMP-t's.
16. **(Part 2) A dead member must leave the monitor BEFORE the monitor can cut on it.**
    - The monitor counts a present member's STALE velocity, which can be the side's fastest. So an absence
      rule as long as the monitor's 25 ticks, or one that waits for the mates to turn, loses the race. The
      first proposal did (the rulings probe).
    - N2 (a)'s FROZEN exclusion takes 5 reads, and it needs neither the command nor the mates when the
      stale velocity itself says "turning".
    - It is judged BEFORE `drive()`, because the monitors are judged inside it.
    - `frozenSuspectReads < persistTicks` is a precondition.
    - P2-6, P2-6b, P2-6c and P2-6d are the proof.
17. **(Part 2) Nothing mid-match may throw.**
    - `markAbsent` refuses rather than throws. `MotorGroup::member(i)` on an absent slot IS a precondition
      throw (:299-300), so the protections read only `isPresent` members.
    - Every protection line is a bounded literal format.
18. **(Part 2) F4 is locked.** No IMotor change (N2).
19. **(Part 2) The cut's deciding tick.** `drive()` has already commanded this tick's stick volts, so the
    0 V write is what makes the cut start in that tick. A zero command alone would start it one tick late.
20. **(Part 2) The median when a member drops** (P2-9).
21. **(Part 2) Part 3 must NOT land before Part 2** (R4). Station K run before Part 2 drives unprotected,
    and the OLD "shulib Drive" keeps its protections until Part 3, so no window exists in which the only
    protected program is gone.
22. **(Part 3) A retired build must not come back silently:** the RETIRED list in step 0, and the
    Makefile's named `$(error)`.
23. **(Part 3) COMP-t's files are not T4's; slot 2 per N4.**
24. **SRC1 paths** (the header): SRC1 must land first, or the coordinator re-maps the brief.
25. **SAFE-SAVE and Part 3 both edit where Teleop's words live.**
    - **B2's source-words function.** Best: Part 3 lands first, so B2's function is written once, for
      "shulib Drive" and COMP. If B2 lands first, Part 3 MOVES what B2 wrote, never copies it.
    - **B2's `BACKUP see brain` LCD word** goes into the ONE shared chooser in COMP-b's header, at its place
      after CLAW ABSENT. It never goes into `teleop_program.cpp`: B2's brief (:706-708) says
      `teleop_program.cpp`, written before COMP-b's shared header existed.
    - **SAFE-SAVE A's `formatSaveReport`** (`calibration/record_store.hpp`) prints "Teleop boots from …"
      sentences in the LIBRARY. B1 wires Calibrate to them before any Calibrate rerun, almost certainly
      before Part 3. Part 3 rewords them (a text change §5 allows). If SAFE-SAVE A lands after Part 3, it
      is written with the new names.
    - Whoever lands second re-reads.
26. **(Part 2) The per-tick reads grow.** Each present member's position, velocity and current is read
    every tick: about 30 adapter reads per 10 ms tick on robot two. The old program read as many
    (:472-480). The loop's real cost on the brain is unmeasured (HA-32). Log it; do not claim it.
27. **(Part 2) A dead port's CURRENT is frozen too** (`hal/pros/motor.hpp` :250-258). The over-current
    streak skips a FROZEN member, and counts only while its side is commanded above 1 V. Otherwise one port
    that dies at the current limit re-cuts the drive every 1.25 s for the rest of the match. P2-4b is the
    proof.
28. **(Part 2) A whole side stopping at once is not a death.** FROZEN needs another member of the side
    still answering. A live train at a wall, with its velocity reports lagging, must mark nobody. P2-19 is
    the proof. The cost is stated in N2: a whole side that dies at once is not seen.

## 8. Found while reading (not T4's to fix unless named above)
- **COMP's slot 5 collided with the fallback's.** Part 0 re-points B5/B29/§3 and build-order.
- **R5a Part D's program list** (R5a ~:376) and **L9 Part B's re-check** (L9 ~:240-252, ~:449-456) named
  Teleop. After Part 3 their callers are "shulib Drive" and COMP. SAFE-SAVE B2's one source-words function
  (~:690-708) covers both (landmine 25). Their briefs are re-read at their go.
- **T3's still-open half — chassis macros and angle presets — lands in the driver file** when it comes: a
  macro is "which button does what", so it gets a line in WHICH BUTTON DOES WHAT, and its binder goes in
  `driver_controls.hpp`. The parked `.claude/coordinator/drafts/T3-COMPLETED.draft-for-when-macros-land.md`
  describes the claws "in shulib Teleop" with the table's buttons. The coordinator re-points it at Part 1b's
  merge, and again at Part 3's (Teleop retired). T3's build-order entry gains a line saying where its
  macros will live.
- **A field-centric test through the driver file is owed** the day field-centric becomes selectable (the
  EQUIVALENT mutations of §3 become real then).
- **`DriverBindings` and `driverHeading()` are library-shaped** (they depend only on `robot::Robot` and
  `teleop`). Promoting them to `include/shulib/` is a candidate for L9 Part B's "boot as library code",
  with its API docs and changelog. It is not T4's.
- **The invented X-drive gets the protections by default** (Part 2: over-current only). Its program never
  runs, because it refuses at boot on every robot. Its three-argument panel shows nothing of them, which is
  correct.
- **A pushing match is the protections' real test** (N1). Nothing in shulib has pushed anything yet.
- **SAFE-SAVE B2's re-check** (the re-check's finding 6). B2 (:706-708) puts its `BACKUP see brain` choice
  in `teleop_program.cpp`, written before COMP-b's shared header existed. At B2's go, the word goes into the
  ONE shared chooser at its place after CLAW ABSENT (landmine 25). Part 0's HANDOFF line carries this.
- **The boot refuses 2/4 on one side today** (`boot_check.hpp` ~:277; N7's note). Robot two is four per
  side, so two dead ports on ONE side keep it from driving at all, from the first tick. "1-2 dead ports
  shouldn't stop driving" was said of a match. Whether the boot should match N7's answer is the team
  lead's, and not T4's change.
- **The coupled-side monitor also runs in the calibrator** (`drivetrain_calibrator.hpp` :1505, :1811).
  Part 2 does not change the monitor's own logic, only who acts on its verdict. The calibrator's cases are
  in the unchanged suite.
