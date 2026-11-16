# Hardware Assumptions Register

> **What this is.** Every claim about physical hardware that shulib v2 currently relies on but
> **cannot check without a robot** — inventoried, falsifiable, and owned. Three chunks of work
> (A1–A3) plus the M1/M2 conversion layers were built against these claims; this document is what
> converts that from a silent risk into a plan. Created by chunk A4
> (2026-08-06); it becomes **Phase R's checklist**: R3 walks the
> R3 group top to bottom on hardware day one, R4 the characterization group, R5/R6 the rest.
>
> **How to read an entry.** Each is a claim stated so it can be proven *false* — a number with
> units, or a definite assertion. Fields:
> - **Claim** — the falsifiable statement.
> - **Source** — where the tree currently assumes it (file:line at A4 close; the `HA-nn` tag
>   embedded at that spot is the durable anchor if lines drift).
> - **Confidence** — one of:
>   - **measured elsewhere** — a documented VEX/PROS spec we have not confirmed on our units;
>   - **reasoned** — derived from documentation, community reports, or arithmetic, unverified;
>   - **invented** — a number we made up to have *a* number. Invented is a legitimate and useful
>     value: a guess *labeled* as a guess is exactly what this register exists to protect. A guess
>     dressed as a measurement is the failure mode.
> - **Settle** — the specific bench measurement or test that decides it, and the owning chunk
>   (R3 conventions/geometry · R4 noise/drift/characterization · R5 gains · R6 plant back-fit).
> - **Blast radius if wrong** — the highest-value field: what breaks and how far it propagates.
>   Where the damage is contained *by design* (behind the `hal/pros` seam, or behind a single
>   config constant), the entry says so — that containment is a real, deliberate result of the
>   architecture, worth recording alongside the risk.
>
> **Rules of this document.**
> 1. **Nothing here is resolved on the host.** If an entry could be settled without a robot, its
>    presence here is a bug — settle it and note that instead. (No such entry was found at A4.)
> 2. **Entries are settled only with cited evidence** (the R-phase measurement, logged), then
>    marked `[x]` with the measured value recorded next to the guess it replaces. A corrected
>    value is the system *working* — the register predicted and localized the defect.
> 3. **The list is supposed to be long.** Its length is the honest cost of building without
>    hardware. Do not shrink it by quiet resolution.
> 4. Labels in code: `PROVISIONAL (A4: HA-nn)` on config fields; `A4 register HA-nn` in prose
>    comments. Reconciliation is bidirectional and grep-verified (see §Reconciliation).
>
> **Held numbers** (reserved by briefs, not yet registered; never reissued): HA-162..HA-176, HA-181..HA-186, HA-188 (CAL1's second number, released unused — never reissued), HA-192..HA-203, HA-204..HA-215, HA-216..HA-218 (STEP1's: HA-216 registered by STEP1-A, HA-217..HA-218 by STEP1-B — all three landed), HA-224..HA-227 (MECH-S/TUNE1's — its HA-219..HA-223 registered by MECH-S-A), HA-228..HA-230, HA-233 and HA-236 (AF1's — the driver-control auto-flipper's; its HA-231, HA-232, HA-234 and HA-235 registered by AF1S-A1, which reuses them), HA-237..HA-242 (ALLY1's and L4's), HA-256 (ROB1's — the twin's, for its run G1-T; ROB1-A registered HA-252..HA-255), HA-257 (ROB1's — HELD for its run G1-B: "each robot keeps its own SD card" is the procedure chapter 21 gives until the record names its robot (D10 (a)); registered if G1-B misses the freeze, released unused when it lands).
> The briefing names the next free number above every row AND every held one (`tools/briefing_status.py`
> reads the line above); a brief that reserves numbers adds them to it, and a number that lands as a row
> may stay in its range.
>
> **Status: 7 of 230 settled** (HA-94/95/96/97/99/100/101, all measured on the old competition bot
> 2026-08-13 — one robot, once; not proof of portability). HA-98 partially settled. *(The total read
> "122" while HA-123 existed — a one-entry drift, corrected 2026-09-14 when robot two's section,
> HA-124–HA-133, was added: of those ten, four are MEASURED on robot two (HA-127/128/129/130),
> one is reported (HA-124), two are UNSET and refuse at boot (HA-125/126), one reasoned (HA-131),
> two invented (HA-132/133). R3d Part 1 added FOUR more on 2026-09-17 — HA-134/135 reasoned,
> HA-136/137 invented — and not one of the four is measured on anything: each is a belief the host
> plant cannot settle, which is exactly why it is an entry. **R3d Part 2 added THREE more on
> 2026-09-18 — HA-138/139/140, all reasoned** — and the same is true of them: the registry's
> zero-indexing and fopen's `"rb"` are beliefs about PROS that only a brain can settle, and the CRC
> note is a design decision recorded so the next person does not re-litigate it. **G1 Part 2a added
> ONE more on 2026-09-24 — HA-141, reasoned:** that the registry's KIND answer is right for an IMU, a
> rotation sensor and a GPS, which the on-robot device provider now refuses ports on. The
> per-confidence counts below predate every entry from HA-124 on. **R3d Part 2F-1 (2026-09-24) added no entry and
> REWROTE three:** HA-134 now describes a RAMPED pulse and what happens when no level turns the
> robot, HA-136 lists the calibrator's new and removed numbers and the screen model's estimates, and
> HA-137 carries the plant's two new rules — coast-follow and the slipping member — because the first
> night on robot two showed the plant could not fail the way the robot did. **R3d Part 2F-2** (the
> same day) added no entry; HA-136 records how the screen's character budgets were checked against
> earlier screens instead of measured. **DEFECTS2 added FIVE on 2026-09-26 — HA-142..HA-146** — the
> beliefs about how a brake mode reaches a V5 motor, found when the library's Brake and Hold stops
> turned out never to have reached one (HA-142 and HA-146 reasoned from the vendored text, HA-144
> invented, HA-143 and HA-145 UNKNOWN and labelled invented), and AMENDED three: HA-53 (the drive's
> "0 V + Brake" coasted on a V5 until then), HA-92 (its premise that the declaration reaches the
> motor was false at the wire) and HA-98 (brake mode is device state too). **T3 (2026-09-26) added
> FOUR — HA-153 to HA-156**, robot two's claw pivots, which no capture has seen yet: HA-153
> reported, HA-154 and HA-155 reasoned (HA-154's timing invented), HA-156 invented. **T3's addendum
> (E2, the same day) added ONE — HA-159**, robot two's pneumatic claw on 3-wire port A: the port
> reported, its polarity and whether its valve is single or double unknown. **T3's addendum B2 (the library half, the same day) added TWO — HA-157, UNKNOWN, and HA-158,
> reasoned** — about the brain's 3-wire ports that the pneumatic claw depends on — **and the
> merge added ONE — HA-147, invented:** the controller belief the claw's toggle rests on (B2's
> review). HA-148 to HA-152 were reserved for T3 and are UNUSED: the numbers are not reissued, so
> the register has 154 entries up to HA-159 (156 with DIAG1's HA-160 and COMP-a's HA-161, below). **DIAG1 added ONE on 2026-09-26 — HA-160** — that the V5 IMU's heading is continuous tick to
> tick while ready, an assumption the library always made and never wrote down, now QUESTIONED by
> the 2026-09-25 robot evidence — and AMENDED HA-123: extended to drive-encoder sides, and its
> blast radius corrected (a spurious trip ends the active motion). **COMP-a added ONE on 2026-09-26 — HA-161, reasoned** (PROS deletes its competition tasks on
> a mode change and not the program's own — the belief the competition program's own auton task
> rests on), and AMENDED HA-05 (the start heading's owner is the chosen auton's start pose). **T3 Part 2 (the Teleop wiring, the same day) added no
> entry** — HA-148..HA-152 stay unused — and brought HA-153, HA-154, HA-156 and HA-159 up to the
> tree: "shulib Teleop" now builds and drives the claws (not on a robot yet), so their lines saying
> no program drives them, or that the claw row does not exist, were corrected. **TANK1 (2026-09-27)
> added TWO — HA-179, invented, and HA-180, reasoned** — the constants `moveTo`'s new tank steering
> runs on (a bend toward the target's line and a cancelled weak-side yaw, tuned on the host test
> plant's placeholder dynamics), and the belief it rests on that robot two's sideways drift is its
> HEADING being pulled, not the chassis sliding sideways; the register then holds 158 entries.
> **COMP-t (2026-09-28) added TWO — HA-177, invented in its VEXos half, and HA-178, reasoned and
> invented** — what the bench auton tester's SD run log assumes an APPEND open does on the card, and
> what its abort and its RUN gate read from PROS (the touch's press count, a button's level, the
> competition status with nothing connected, a controller switched off); the register then holds
> 160 entries. **L9 Part A1 added ONE on 2026-09-29 — HA-167, invented** — that a tank drives backwards
> the way its calibration measured it, in one unpowered push (the recipe layer's new `reverseTo` /
> `faceAway` and a route's `reverse` DriveTo both command backwards legs); with it, 161.
> **DIAG1 Part B added ONE on 2026-10-01 — HA-165, invented** — that the members of one drive side
> stay close enough in cumulative position that leaving members out of the side's median (members
> whose reading is not finite, now left out like absent ones) is a small step, and AMENDED HA-08 (what
> a motor group and the drivetrain calibrator do with a non-finite reading that gets past the adapter's
> screen); with it, 162.
> **T4 Part 2a (the drive protections' library half) added TWO on 2026-10-01 — HA-175, invented on
> reasoned mechanics and host-probed only, and HA-176, invented** — the frozen-reading rule by which the
> library's driver loop drops a drive motor that stops answering mid-run (and the mid-run stop at more
> than two dead in total), and the over-current cut — and AMENDED HA-132 and HA-133 (their consumers now
> include those protections in every driver-control program, and a false fight now CUTS the drive for
> 1 s); with them, 164.
> **COMP-b, the competition program, added ONE on 2026-10-02 — HA-162, its reservation, unknown** —
> what the controller's Timed Run does after it starts a program disabled (the bench's only way to run a
> pretend match); with it, 165.
> **SAFE-SAVE Part A added ONE on 2026-10-04 — HA-166**, the six beliefs about the brain's SD card that
> the calibration record's safe save rests on (no rename, no delete, no read-write open, `"wb"` empties a
> file at the open — reasoned from the vendored binaries; that a crash spares every other file and that a
> read-back proves the medium — invented; that two names differing only in letter case or trailing dots
> are one file — FAT's rule, its VEXos half invented), and AMENDED HA-122 with two cross-references (no
> status change); with it, 166.
> **R5a Part A added FIVE on 2026-10-06 — HA-170 to HA-174** — the beliefs shulib Calibrate's GAINS
> and REST steps rest on (built as library and proven on the host test plant only; no program runs
> them yet): HA-170 their every threshold, invented; HA-171 that a drive side follows the fitted law
> outside the fitted range, invented; HA-172 the current limit as the ramp's only non-linearity,
> reasoned and invented; HA-173 the IMU drift's per-boot nature and its noise STRUCTURE, reasoned and
> invented; HA-174 that measured gains hold for a match, invented and reasoned; with them, 171.
> **T4 Part 3 (the new "shulib Drive", Teleop retired; built 2026-10-04/06, reviewed 2026-10-07/08)
> added no entry** — and brought the tree's words up to it: HA-175's claim (b) re-worded to the team
> lead's ruling of 2026-10-01 (the ENCODER check: a side-mate is evidence the train turns only while its
> encoder moved), its known gaps re-stated (the any-wall case closed at the cost of one fight cut; port
> 18's two kept); HA-112, HA-124, HA-131, HA-132, HA-141, HA-143, HA-153..HA-159, HA-161 and HA-176's
> program text moved from "shulib Teleop" and the old "shulib Drive" to the new "shulib Drive" and the
> driver file; and HA-60 names its second consumer, the programs' SD log copy, written inside a tick.
> One belief it met is left for the coordinator, unnumbered: whether PROS serializes two tasks' writes
> to one FILE* (stdout; the card copy no longer depends on it — `diag::LogCopySink` keeps one writer).
> (Numbered HA-189 the same day, below — reasoned FALSE — and fixed in shulib's code by SER1.)
> HA-161 gained a consequence line at its verification (2026-10-08): as built, a deletion that lands
> mid-copy ends the card's copy for that boot. A second belief is named there and left unnumbered: it
> would be needed only by a guard that recovers from such a deletion (`task_get_state()` on a deleted
> task's handle).
> **The coordinator added ONE on 2026-10-08 — HA-189** — T4 Part 3's hand-over left one belief unnumbered (do two
> tasks' writes to stdout stay whole?); reading the vendored C library answers it the wrong way: reasoned FALSE. With
> it, 172.
> **MECH-A1 (the mechanism core's first part; built 2026-10-08, landed 2026-10-15) added TWO — HA-184, UNKNOWN, and HA-186,
> reasoned** — the two beliefs about the sensors the core's declarations name: that a limit switch reads
> 1 when pressed and an unplugged switch line reads released (the core's `pressedLevel = true`), and that
> a rotation sensor's cumulative position starts at the shaft's absolute angle (`absoluteSensor()`
> homing). In this part the core only DECLARES those roles and builds their devices; nothing reads a
> switch or homes a mechanism yet (the core's service, its next parts). With them, 174. *(Since MECH-A2, 2026-10-15, the service reads a limit switch used as a PIECE
> sensor — HA-184 matters from there; homing, and so HA-186 and the home-switch half of HA-184, from MECH-A3.)*
> **SER1 (the serial log's lines kept whole; built 2026-10-08, landed 2026-10-15) added TWO — HA-190 and HA-191** — what its fix, ONE `write()`
> per line, leaves in PROS's hands: a task deleted inside that write holds PROS's serial mutex for ever (HA-190), and
> one write must fit PROS's 2047-byte serial stream (HA-191); both reasoned from the vendored `libpros.a`. HA-189's
> entry now names the fix; it stays unsettled until the bench — its dedicated two-task test is owed, and Station
> K.7.3 is evidence toward it (SER1's review, 2026-10-15, corrected an over-claim there). With them, 176.
> *(2026-10-18: the team lead accepted K.7.3, in writing, as HA-189's robot test — no dedicated station; HA-189
> stays unsettled until K.7.3 runs.)*
> **MECH-A2 (the mechanism core's second part; built 2026-10-08–15, landed 2026-10-15) added ONE — HA-183, invented** — that the host
> test plant's mechanism models (`sim::ArmPlant`, `sim::FlywheelPlant`: an inverted-feedforward joint
> with a gravity load, hard stops, a jam, the brake modes at 0 V, and a stall current capped at 2.5 A)
> stand in for a real arm, lift, roller and flywheel closely enough that the core's host verdicts
> transfer. Every constant in them is invented; the core's next part (MECH-A3, the position and speed
> behaviours) leans on them far more than this one. With it, 177.
> **MECH-A3 (the mechanism core's third part, the position and speed behaviours; built 2026-10-15) added ONE —
> HA-182, invented** — that the library's STARTING numbers for an arm, a lift and a flywheel (the gains, the smooth
> move's speed and ramp, "reached", the hold's integral clamp, the speed integral's window and clamp) hold a real
> mechanism well enough to be tuned from. The gains, the profile and "reached" are per mechanism: worded for a
> boot line, PROVISIONAL (the programs print it from MECH-A5 on; none does yet), and replaced one by one by a
> declaration's `.tuning`. The integral clamps and window are LIBRARY constants: no `.tuning` field, not on that
> line — a wrong one is a library change for every mechanism (MECH-A3's review, 2026-10-22, split the claim). It AMENDED three: HA-183 (the plant gained a mounted angle sensor, a limit switch and a
> wedge, and now carries the position and speed verdicts), HA-184 (the service reads a HOME switch since this part
> — at boot and through a homing push — through the read-fault rule) and HA-186 (absolute homing runs: the
> reading's turn chosen nearest the middle of the travel, a reversed sensor's mounting applied, and a travel of a
> whole sensor turn or more refused at build). With it, 178.
> **IMU1 Part A (the IMU's turn rate once per control tick; built 2026-10-29) added THREE — HA-213, reasoned,
> HA-214, invented, and HA-215, reasoned** — the beliefs `ProsImu`'s 5 ms rate hold rests on: that the IMU's
> readings are copied into the brain's buffer every 10 ms (sample-and-hold), that the copy's phase against the
> tick is unknown and a copy next to a read (or between two positions of a tick's first read) gives a pair of
> ticks 0 and about double, and that a tick's reads fall within 5 ms of its first with no reader between ticks.
> None is measured; IMU1 Part B's bench stage (Station N; built 2026-11-03, never run on a robot) is their first look. It AMENDED HA-179 (its "The IMU
> read" paragraph: robot two's Localizer rate was not clean before IMU1; Station M.6 counts only on an IMU1
> build), HA-22 (a REST record from a pre-IMU1 build is not its evidence; one doubled sample can remain), HA-32
> and HA-102 (the hold groups reads by time). With them, 181.
> **MECH-A5 (the mechanism core's fifth part, robot two and the programs; built 2026-10-29) added ONE — HA-185,
> invented** — that robot two's pneumatic claw moves between closed and open within 0.5 s, so an auton's `set()`
> step on it does not count the move done early (the claw's `.moveTime = 0.5_s`, PROVISIONAL, in
> `src/robots/mechanism_table.hpp`). It AMENDED one: HA-159's flip is now "swap `Low` and `High` on the claw's
> `.positions` line" (the named constant `kClawOpenCommand` is gone with the old table). With it, 182.
> **VIS1-A1 (the AI Vision camera as a device; built 2026-10-18–22) added TWELVE — HA-192..HA-203**, the numbers
> VIS1's brief held: what the camera's library code believes about the camera and PROS — the frame's
> freshness and the image's unmeasured age (HA-192, reasoned), the image's axes (HA-193, reasoned), the optics
> and the axis against the housing (HA-194, reasoned), a non-atomic read (HA-195, reasoned), the mode mask and
> the read-backs (HA-196, reasoned), the AI model's names and no class count through PROS (HA-197, measured
> elsewhere for the names, reasoned for the rest), the detections' order and score (HA-198, reasoned — nothing
> depends on the order; the score's scale only through VIS1-A2's team-set floor, `minScore`) — and about the
> field: the colours and their lookalikes (HA-199, invented), the model's box
> (HA-200, invented), the Toggle's geometry (HA-201, reasoned — the flat bottom face at 12.36 in read from pA8
> at VIS1-A1), a box edge's jitter (HA-202, invented) and a read's cost (HA-203, invented). Where the vendored
> PROS header and its `libpros.a` disagree, the binary was read and followed. With them, 194.
> **L6-A1 (the wall reset's library core; built 2026-10-29) added EIGHT — HA-204..HA-209, HA-211 and HA-212**
> (HA-210, robot two's side-sensor ports and unmeasured mounts, stays HELD for L6-B, the part whose code cites it) —
> the beliefs the wall reset rests on: the V5 Distance Sensor's accuracy (HA-204, reasoned), its beam's spread and
> what it reports with two surfaces in view (HA-205, invented), the wall answering at the sensor's height — clear
> panels and the GPS code strip included (HA-206, invented with contrary evidence), the field's wall positions
> (HA-207, reasoned), the Toggle's overhang and brackets (HA-208, reasoned and invented), the sensor's update
> interval and latency (HA-209, invented), every gate default (HA-211, invented) and the Loaders, the start pieces
> and VEX U's loads (HA-212, reasoned). **None is measured; no distance-sensor reading of the team's robot exists.**
> It amended HA-113, HA-114, HA-115 and HA-180 by one sentence each (their blast radius now names the wall reset).
> With them, 202.
> **CAL1 Part A2b (the standard Calibrate's typed reference and its proof through the real PROS shell; built
> 2026-10-30) added ONE — HA-187, reasoned (1) and invented (2)–(3)** — that the standard Calibrate's WATCH reads
> a DECLARED mechanism motor with PROS's three C reads by port and changes nothing: the reads write nothing and
> make no motor object (reasoned from the vendored `libpros.a`, disassembled for the CAL1 brief on 2026-10-08),
> the position they give is the shaft's turn within the reported gearing's scale, and reading does not disturb
> it. CAL1 held HA-187..HA-188 for its beliefs; HA-188 (a travel ratio to judge a turn by) was never needed and
> is RELEASED, never reissued. With it, 203.
> **MECH-A6 (the core's sixth and last part, Bench Tests' station 13; built 2026-10-30) added NONE.** It AMENDED
> HA-182 (the bench page's own invented numbers — its jog ceiling, its 1 s announcement and 0.4 s check, the
> direction check's two thresholds, its cuts — joined the belief, as the core's brief assigned), HA-184 and HA-186
> (their settle names the built page and worksheet Station P; HA-184's covers a switch PIECE sensor as well as a
> home switch, and HA-186's page names the RAW reading `absoluteSensor(at, reads)` takes), HA-153 and HA-159 (the
> pivots' direction and the claw's open level are Station P's to check first), and HA-189's note (the menu is
> 3 × 5 now). Still 203.
>
> **MECH-S-A (a mechanism's current cap; built 2026-11-05) added FIVE — HA-219..HA-223** of the nine
> MECH-S/TUNE1's brief held (HA-224..HA-227, the slack watch's, stay HELD for MECH-S-B): that a V5 motor
> enforces the limit PROS sets by itself, within a tick, under any command and with nothing ticking (HA-219,
> reasoned · speed invented); that a limit can outlive the program that set it and a motor that loses power
> comes back at its 2500 mA default (HA-220, unverified); that VEXos may lower each motor's limit with more
> than 8 motors plugged in (HA-221, unverified); the string's pull per amp and the momentum no cap limits
> (HA-222, reasoned · number unverified); and that a stalled capped motor reads within ~0.1 A of its cap
> (HA-223, invented). **None is measured.** With them, 208.
>
> **STEP1-A (the arrival tolerance and the on-failure choice; built 2026-11-05) added ONE — HA-216, reasoned** —
> that a leg which ends under a LOOSENED arrival tolerance, still moving at up to the scaled speed bound (4 in/s
> at 2 in; a turn up to 1.31 rad/s, about 75°/s, at 5°), comes to rest within that tolerance on robot two. On the
> host test plant it stops dead on the exit tick; robot two coasts on its own momentum, unmeasured. STEP1 held
> HA-216..HA-218; HA-217 (a partway trigger's latency) and HA-218 (the estimate's "arrived" against the tape) stay
> HELD for STEP1-B, the part whose code cites them. With it, 209.
>
> **CURVE1-A (the curve follower, `curveTo` / `reverseCurveTo`; built 2026-11-11) added FIVE — HA-246..HA-250**,
> the five its brief reserved: that the curve's PROVISIONAL tuning (look-ahead 8 in, bend grip 60 in/s², the
> hand-off, the end speed) lets robot two — and robot one on robot two's numbers — follow a curve and settle
> at its end, and that guide 05's run-up rule (a sideways miss up to 6 / 12 in removed from 2½ / 3 ft back)
> holds on the robot (HA-246, invented); that robot two does not slide sideways at 60 in/s² (HA-247,
> reasoned); that a healthy robot never trips the "lost" guards while a dead side does within ~2 s (HA-248,
> invented); that drive-encoder odometry stays as true on a curve as on a straight (HA-249, reasoned); and
> that a curve fits the auton task's stack and its 10 ms tick (HA-250, reasoned). **None is measured;**
> Station C (after Station M) is their first look. With them, 214.
>
> **STEP1-B (triggers partway through a drive; built 2026-11-11) added TWO — HA-217 and HA-218, both reasoned** —
> that a partway trigger's mechanism starts within about 1–2 ticks of the estimate crossing its mark (the command
> is issued in the drive's tick; the service acts on it at the next pace), and that a trigger fired as a leg
> ARRIVES (the team lead's D6 (i)) puts the mechanism within the leg's tolerance of the author's mark only as far
> as the position estimate's "arrived" is true. On the host test plant the service answers the same tick and the
> estimate tracks truth to about 0.01 in; robot two's motor response, task timing and odometry drift are
> unmeasured. With them, 216.
>
> **AF1S-A1 (the Toggle flip for a skills run, the step; built 2026-11-11) added SEVEN.** Four of AF1's held
> numbers are REUSED — they mean the same thing here: one flipper stroke rolls a Toggle exactly one face, the
> research's order (HA-231, reasoned); a FULL stroke seats it, which the camera cannot check (HA-232, invented);
> the run's end safes the flipper, it does not retract it (HA-234, reasoned); a sticky Toggle's push is cut by
> the flipper's stall signature or its bound before it damages the Toggle (HA-235, invented). Three are new: each
> Toggle's Quadrant colour, read from two drawings (HA-243, reasoned); the step's straight drive in and out puts
> the flipper within reach because the ROUTE arrived — the camera's spot check refuses only gross errors (HA-244,
> invented); and a real field's camera names the colour from the look spot as the host camera model predicts
> (HA-245, invented). AF1's HA-228..HA-230 (the Optical), HA-233 (the field timer) and HA-236 (the flipper's
> ports — robot one's, with G1) stay held. **None is measured.** With them, 223.
>
> **ROB1-A (robot one in the code; built 2026-11-11) added FOUR — HA-252..HA-255** of the six the ROB1 brief
> held (HA-256, the twin's, stays HELD for its run G1-T; HA-257 stays HELD for G1-B): that robot one is a TANK
> like robot two (HA-252, reported — "identical"), that it carries an IMU (HA-253, unknown — the builder named
> only AI Vision and rotation sensors), that its cascade's height is close to straight in its output angle
> between two measured rows (HA-254, reasoned), and that its two cascade motors face each other (HA-255, a
> guess until station 13's direction check). **None is measured — robot one does not exist in this project's
> records.** With them, 227.
> **MIRROR1-A (one route, two robots: robot two runs robot one's skills route MIRRORED; built 2026-11-13) added
> THREE — HA-258..HA-260** (its brief held them past ROB1's numbers): the event's field is set up to Appendix A
> and Figure VURS1, so the two red Quadrants are mirror images across the skills diagonal — with the centre goal,
> two yellow Pins and the red/right Load Zone ON it (HA-258, reasoned); robot one's build is left-right
> symmetric for every part a route uses at the field (HA-259, unknown); the drivetrain's own left/right
> differences are not mirrored, and each robot's real position near the diagonal stays within half the pair
> check's 6 in gap of its plan, the robot inside the check's 15 in square about its tracking centre (HA-260,
> invented). **None is measured.** With them, 230.
> The rest of HA-162..HA-176 (all but HA-162, HA-165, HA-166, HA-167, HA-170..HA-174, HA-175 and HA-176) is not in it: numbers other chunks hold (briefed, not landed) — never
> reissued — and neither is **HA-181** (G3a, the robot panel: its amended brief), held the same way (the
> mechanism core's five, HA-182..HA-186, have all landed: HA-184 and HA-186 with its first part, HA-183 with
> its second, HA-182 with its third, HA-185 with its fifth, above). The
> per-confidence counts below predate every entry from HA-124 on (2026-09-26): they still sum to
> 122, have no bucket for UNKNOWN (HA-143, HA-145, HA-157 and HA-162's confidence), and a recount is due.)* **No *v2* robot exists**, and
> the platform layer has now been validated on the team's old competition bot — real adapters
> commanded real motors and read real sensors on 2026-08-13 — but **no control loop has ever
> closed and nothing has driven.** Counts: **78 invented · 41 reasoned · 2 measured elsewhere · 1 mixed** (HA-44:
> documented shape, unmeasured onset). HA-50–52 added by chunk C1,
> HA-53 by chunk C2 (the cancel safe state), HA-54–55 by chunk C3 (the H-drive's strafe derate
> and stand-in geometry), HA-56–57 by chunk C5 (the D-5 plausibility envelope and the D-4
> controller-screen grid), HA-58–60 by chunk E1 (the blackbox's flight-recorder depth, RAM
> budget and assumed SD write cost — every magnitude in that chunk is a guess until R4),
> HA-61–67 by chunk E2 (the GPS corrector's tuning set), **HA-68–82 by chunk E3** (the
> AprilTag path — of which three, HA-68/69/70, are about the physical world rather than tuning
> and are the most dangerous entries in this document: each produces a *confidently* wrong fix
> rather than a degraded one), HA-83–91 by chunk E4 (the EKF's noise model — every number
> invented), HA-92–93 by chunk F1 (the mechanism layer's physics), and **HA-94–112 by chunk
> R1a** (the beliefs about PROS itself that the `hal/pros` adapters and their host shim are
> built on — the shim tests the adapters against these beliefs; ONLY a bench tests the
> beliefs), and **HA-113–122 by chunk R1b** (the same class of belief for the mechanism-sensor
> adapters: distance, optical, ADI digital lines, and the SD card — including two flagged-weak
> halves the vendored source does not state: proximity's polarity, HA-117, and fopen's `/usd/`
> prefix, HA-122), per the Maintenance convention.
> *(This status line was found stale at R1a — it read "0 of 82" while the register held 93
> entries: E4's and F1's additions never updated it. Corrected here; the per-chunk narrative
> above is the part a tool cannot regenerate, so it is the part that must be tended.)*

---

## Index

| ID | Claim (short) | Confidence | Owner |
|---|---|---|---|
| HA-01 | GPS position axes: +X = East, +Y = North | **invented** | R3 |
| HA-02 | IMU as-mounted heading is CW-positive | **CONFIRMED** 2026-08-19 on the bench bot: operator turned the robot LEFT/CCW and RAW `get_rotation()` went NEGATIVE (-0.073 -> -87.241 deg) while CANONICAL rose (+0.073 -> +87.241 deg). Raw decreasing on a CCW turn IS raw being CW-positive, exactly as claimed; and the adapter's negation therefore yields the CCW-positive canonical F1 requires. Evidence: `evidence/r3a-session-2026-08-19.log` run 2 | R3 |
| HA-03 | `get_rotation()` is cumulative/unbounded (the required binding) | reasoned | R1/R3 |
| HA-04 | IMU yaw-rate source sign (`get_gyro_rate().z` undocumented) | **invented** | R3; IMU1 Part B's bench stage (Station N) |
| HA-05 | Post-cal tare invalidates bootHeading; offset applied once | reasoned — **AMENDED** 2026-09-26 (COMP-a): the start heading's one owner is the chosen auton's start pose, applied at autonomous start through the Localizer's bias (`reseatHeading`) — bootHeading stays 0 and the IMU is never re-zeroed | R1/R3 |
| HA-06 | Firmware GPS offset unset ⇒ `get_position()` = SENSOR pose | reasoned | R1/R3 |
| HA-07 | GPS `get_error()` returns meters | reasoned | R3 |
| HA-08 | Failed reads return PROS_ERR/PROS_ERR_F; adapters screen to stale-finite | reasoned — **AMENDED 2026-09-29** (DIAG1 Part B: what a motor group and the calibrator do if one gets through) | R1/R3 |
| HA-09 | `northHeadingDeg = 90°` matches our field setup | reasoned | R3 |
| HA-10 | GPS lever arm value (body inches) as configured | **invented** | R3 |
| HA-11 | Rotation `get_position()` cumulative; int32 centidegree never wraps in a match | reasoned | R1/R3 |
| HA-12 | Tracking-wheel offsets/signs as configured (−3.0″/−4.5″ are stand-ins) | **invented** | R3 |
| HA-13 | Tracking-wheel effective diameter = nominal 2.0″ | **invented** | R3 |
| HA-14 | Drive wheel 3.25″, wheel↔shaft gearing 1:1 | **CORRECTED** 2026-08-19: wheels measured **2.75″**, not 3.25″ — build team, read off the robot. The 3.25″ stand-in is WRONG and every distance derived from it is short by 15.4%. Gearing half still open (B1.2, per side) | R3 |
| HA-15 | Drive cartridge GREEN → 900 ticks/rev at output | **CORRECTED** 2026-08-19: cartridges are **BLUE** (6:1, 600 rpm), read visually off the motors — the only way this is knowable, since a V5 motor cannot sense its own cartridge. GREEN was invented and is WRONG. **The brain is currently CONFIGURED green while the hardware is blue** (`motor_get_gearing()` returned GRN 200 on all eight), so position and velocity are mis-scaled by the 18:1-vs-6:1 ratio until something sets it correctly | R3 |
| HA-16 | Rotation sensor 36000 ticks/rev (centidegree) | measured elsewhere | R3 |
| HA-17 | Built drivetrain matches the frozen preset geometry | **invented** | R3 |
| HA-18 | F5 numbers match on-V5 (host ≡ robot, swapping `RobotContext`) | reasoned | R3 |
| HA-19 | Brownout kills motors first; CPU survives; the run continues | reasoned | R3 |
| HA-20 | IMU per-boot drift ≤ 1°/min (typical 0.1–0.5) | **invented** | R4 |
| HA-21 | IMU heading noise σ ≈ 0.05° | **invented** | R4 |
| HA-22 | IMU yaw-rate noise σ ≈ 0.5°/s | **invented** | R4 |
| HA-23 | IMU calibration ≈ 2 s, emits garbage-that-moves while not-ready | reasoned | R3/R4 |
| HA-24 | IMU end-to-end heading latency ≈ 10 ms | **invented** | R4 |
| HA-25 | Encoder refresh ≈ 1 tick (10 ms) | reasoned | R4 |
| HA-26 | GPS on-strip position noise σ ≈ 0.7 in/axis | **invented** | R4 |
| HA-27 | GPS heading noise σ ≈ 1° | **invented** | R4 |
| HA-28 | GPS fresh-fix cadence ≈ 50 ms | **invented** | R4 |
| HA-29 | GPS claims rms ≈ 1.0″ healthy / ≈ 99″ no-fix; claim ≠ truth | **invented** | R4 |
| HA-30 | GPS end-to-end latency ≈ 50 ms | **invented** | R4 |
| HA-31 | Real off-strip/no-fix stale-pose behaviour (model: origin) | **invented** | R3/R4 |
| HA-32 | ~100 Hz loop sustained under full stack load | reasoned | R3 |
| HA-33 | PROS sensor reads are non-blocking (µs-class) | **invented** | R4 |
| HA-34 | Loop jitter ±20%, 2% spikes at ×5 | **invented** | R4 |
| HA-35 | `bootSettleTime` 0.1 s covers the worst data-path latency | **invented** | R4 |
| HA-36 | `driftHorizon` 12″ matches real dead-reckon drift decay | **invented** | R4 |
| HA-37 | Traction breaks above ≈ 80 in/s² wheel accel | **invented** | R4 |
| HA-38 | A slipping wheel still propels ≈ 70% of its spin | **invented** | R4 |
| HA-39 | Field foam is traction-uniform (one slip pair suffices) | **invented** | R4 |
| HA-40 | Pack sag ≈ 0.02 V per commanded volt | **invented** | R4 |
| HA-41 | Pack discharge ≈ 0.005 V/s under match load | **invented** | R4 |
| HA-42 | Brownout trips ≈ 10.5 V, recovers ≈ 10.8 V | **invented** | R3/R4 |
| HA-43 | Motor thermal: 0.0023 °C/(V²s) heat, 0.01 /s cool, 25 °C ambient | **invented** | R4 |
| HA-44 | Throttle steps 55/60/65 °C → 50/25/12.5% | mixed | R4 |
| HA-45 | Wheel FF placeholders: kS = 1.0 V, kV = 12/70, kA = 0 | **invented** | R5 |
| HA-46 | Fresh pack nominal ≈ 12.6 V | measured elsewhere | R4 |
| HA-47 | `move_voltage` is true voltage control (sag only bites at the ceiling) | reasoned | R5 |
| HA-48 | FF-inversion + first-order lag is an adequate plant shape | **invented** | R6 |
| HA-49 | Unmodeled 2.5 A current limiting changes no host-phase conclusion | **invented** | R6 |
| HA-50 | C1 motion gains + speed budget (kP 3.0/4.0, 60 in/s, 6 rad/s, 60 in/s wheels) | **invented** | R5 |
| HA-51 | C1 settle tolerances (0.5″/1.15° + rate floors; brake 1.2 in/s; 5 s timeout) | **invented** | R4/R5 |
| HA-52 | Stall cross-check thresholds (0.3 s window, 1.0″ spin, 25% ratio, stand-in radii) | **invented** | R3/R4 |
| HA-53 | Cancel safe state: 0 V + BrakeMode::Brake stops the drive promptly from speed | reasoned — **AMENDED** 2026-09-26 (DEFECTS2): on a V5 this stop COASTED until DEFECTS2 (the adapter sent 0 V, which ignores the mode — HA-142); it brakes only since | R3/R5 |
| HA-54 | H-drive strafe traction derate ≈ 0.35 (⇒ authority 0.35 at 1:1 gearing) | **invented** | R5 |
| HA-55 | 15″ H-bot stand-in geometry (11″ track, strafe wheel 4″ aft, 1:1 strafe gearing) | **invented** | R3 |
| HA-56 | D-5 plausibility envelope defaults (maxSpeed 150 in/s, maxYawRate 20 rad/s, ×1.5 margin) | reasoned | R3/R5 |
| HA-57 | V5 controller text grid is 3 rows × 19 columns (`ILineDisplay` geometry) | reasoned | R1 |
| HA-58 | Flight-recorder depth: 200 ticks (~2 s) reaches back past a fault's cause | **invented** | R4 |
| HA-59 | 64 KiB of RAM is spendable on the blackbox staging buffer | **invented** | R4 |
| HA-60 | An SD flush of tens of KB costs single-digit ms (fine at a boundary, not in a tick) | **invented** | R4 |
| HA-61 | GPS `rmsError()` must be inflated ×2 to be used as sigma (claim ≠ truth) | **invented** | R4 |
| HA-62 | 0.5″ floor on the GPS measurement sigma | **invented** | R4 |
| HA-63 | A fix claiming > 6″ of error is not worth folding | **invented** | R4 |
| HA-64 | 3 rad/s (≈172°/s) is where a GPS fix stops being trustworthy | **invented** | R4 |
| HA-65 | 4σ normalized-innovation gate width | **invented** | R4 |
| HA-66 | The estimate is ≈1″ uncertain immediately after a fix is folded | **invented** | R4/E4 |
| HA-67 | Dead-reckon sigma grows ≈0.02″ per inch travelled (the anti-lockout term) | **invented** | R4 |
| HA-68 | The field's AprilTag layout is knowable and a team's map is right to ≈0.5″ | **invented** | R3 |
| HA-69 | The tag detector reports its four corners in a consistent, knowable WINDING | reasoned | R2/R3 |
| HA-70 | The tag camera is mounted level (no pitch/roll) to within ≈1° | **invented** | R3 |
| HA-71 | Tag pipeline latency ≈ 80 ms (exposure + detect + PnP + transport) | **invented** | R4 |
| HA-72 | A tag frame older than 0.25 s means the vision task has stalled | **invented** | R4 |
| HA-73 | A tag fix is trustworthy only between 6″ and 72″ of range | **invented** | R4 |
| HA-74 | A tag detection below 0.35 confidence is not worth folding | **invented** | R4 |
| HA-75 | 2 rad/s (≈115°/s) is where motion blur kills a tag fix | **invented** | R4 |
| HA-76 | Tag position 1σ ≈ 1.0″ + 0.02″ per inch of range | **invented** | R4 |
| HA-77 | 4σ normalized-innovation gate width for tag fixes | **invented** | R4 |
| HA-78 | The estimate is ≈1″ uncertain immediately after a tag fix | **invented** | R4/E4 |
| HA-79 | Dead-reckon sigma grows ≈0.02″ per inch since this source's last fix | **invented** | R4 |
| HA-80 | A heading innovation above 15° is a fault, not drift | **invented** | R4/E4 |
| HA-81 | 0.15 of the heading innovation per tick at confidence 1 | **invented** | R4/E4 |
| HA-82 | 10°/s is a safe upper bound on estimator heading-bias change | **invented** | R4/E4 |
| HA-83 | EKF position process noise: σ grows 2% of distance travelled, + a 0.5 in/s floor | **invented** | R4 |
| HA-84 | EKF heading process noise: σ grows 1% of rotation, + HA-20's 1°/min drift | **invented** | R4 |
| HA-85 | The drive can change body velocity by 200 in/s² (the EKF's velocity process noise) | **invented** | R4/R5 |
| HA-86 | One tick's odometry displacement is 1σ ≈ 0.01 in + 2% of the tick's travel | **invented** | R4 |
| HA-87 | 3σ is the right Mahalanobis gate width for the EKF tier | **invented** | R4 |
| HA-88 | An absolute heading measurement is 1σ ≈ 2°, flat | **invented** | R4 |
| HA-89 | The EKF's prior when it knows nothing: 24 in, 30°, 24 in/s | **invented** | R4 |
| HA-90 | 50 consecutive gate rejections with a mean innovation over 6 in means "lost" | **invented** | R4 |
| HA-91 | 5 s is long enough between re-init declarations to tell recovery from a storm | **invented** | R4 |
| HA-92 | `BrakeMode::Hold` at 0 V holds a LOADED cascade lift against back-drive | **invented** — **AMENDED** 2026-09-26 (DEFECTS2): the premise that the declaration reaches the motor was FALSE at the wire until DEFECTS2 (HA-142) | R3/R5 |
| HA-93 | A jammed 11 W mechanism motor reads ≈ 2.5 A at a full 12 V command, with ≈ 0.05 rad/s residual creep | **invented** | R4 |
| HA-94 | Motor `move_voltage()` takes millivolts, ±12000 | **settled** measured-on-comp-bot 2026-08-13: 2.0 V commanded → device reported 1823–1990 mV under load | R3 |
| HA-95 | Motor `get_position()` with `MotorUnits::degrees` reports output-shaft degrees | **settled** measured 2026-08-13: deg/rad = 57.296 on all 8 motors (57.2958 exact) | R3 |
| HA-96 | Motor `get_actual_velocity()` returns output-shaft RPM | **settled** measured 2026-08-13: rpm/(rad/s) = 9.549 (9.5493 exact) | R3 |
| HA-97 | Motor `get_current_draw()` returns mA | **settled** measured 2026-08-13: mA/A = 1000.0 exactly, 8 motors | R3 |
| HA-98 | Motor units/gearing — and the BRAKE MODE (added to this row by DEFECTS2) — live in the DEVICE and persist across programs; explicit set + read-back holds | **partial** 2026-08-13: set+read-back accepted on 8 motors (no precondition fired); PERSISTENCE across programs NOT tested, so this stays open | R3 |
| HA-99 | Battery voltage/current are mV/mA — units NOT in the vendored source (website only) | **settled** measured 2026-08-13: raw 13039 → 13.04 V, raw 919 → 0.92 A. R1a's weakest belief, now observed | R3 |
| HA-100 | `battery_get_capacity()` returns percent 0–100 | **settled** measured 2026-08-13: raw 91.0 → 0.91 | R3 |
| HA-101 | `micros()` is µs since program start, monotonic, uint64 | **settled** measured 2026-08-13: 999784 µs over a nominal 1000 ms delay (0.02% low) | R3 |
| HA-102 | `Task::delay_until(prev, 10)` yields an anchored 100 Hz cadence | reasoned | R3 |
| HA-103 | Controller axes are [-127, 127]; disconnected reads 0, not a sentinel | reasoned | R3 |
| HA-104 | `get_digital()` is a level; `get_digital_new_press()` CONSUMES the press | reasoned | R3 |
| HA-105 | Rotation `get_velocity()` returns centidegrees/second | reasoned | R3 |
| HA-106 | `gps_status_s_t.yaw` is the CW-from-North heading in degrees | reasoned | R3 |
| HA-107 | Controller LCD columns: 19 (HA-57) vs the vendored doc's [0-14] ⇒ 15 — CONFLICT | **invented** | R3 |
| HA-108 | `Imu::reset(false)` starts calibration; one boot-time call is not a post-cal tare | reasoned | R3 |
| HA-109 | `imu_gyro_s_t` is deg/s and z is the yaw axis (sign is HA-04's entry) | **invented** | R3; IMU1 Part B's bench stage (Station N) |
| HA-110 | IMU as-mounted pitch/roll sign conventions | **invented** | R3 |
| HA-111 | The invented X-drive's port map and motor direction signs match the robot (`src/robots/xdrive_config.hpp` since G1 Part 2c; `src/main.cpp` before) | **invented — and MEASURED WRONG for the comp bot** 2026-08-13: real map is drive LEFT 15/16/17/18, RIGHT 11/12/14, IMU 4, no rotation sensors, no GPS. The xdrive build still carries the invented map, now as a config constant that refuses at boot on any robot that exists | R3 |
| HA-112 | Teleop stick mapping signs + the feel layer's defaults are drivable (0.05 deadband, applied as a continuous rescale since T2; expo 0; slew off; the field-centric fallback latch) | **invented** | T2/R3 |
| HA-113 | Distance `get_distance()` returns millimeters | reasoned | R3 |
| HA-114 | Distance "no object" is the IN-BAND integer 9999, not PROS_ERR | reasoned | R3 |
| HA-115 | Distance `get_confidence()` is 0–63, only available above 200 mm; its value at/below 200 mm is UNKNOWN | reasoned | R3 |
| HA-116 | Optical hue is 0–359.999°, saturation/brightness are 0–1.0 | reasoned | R3 |
| HA-117 | Optical `get_proximity()` 0–255, LARGER = CLOSER — polarity NOT in the vendored doc | **invented** | R3 |
| HA-118 | Optical sentinels: PROS_ERR_F on the double channels, PROS_ERR on proximity | reasoned | R3 |
| HA-119 | ADI `DigitalOut` DRIVES THE LINE AT CONSTRUCTION (init_state, PROS default LOW); `set_value` is 1/0, PROS_ERR on refusal | reasoned | R3 |
| HA-120 | ADI addressing: 1–8 ≡ 'a'–'h' ≡ 'A'–'H'; expander via {smart, adi} pairs; whether OUR robot has an expander is UNKNOWN | reasoned | R3 |
| HA-121 | ADI `DigitalIn::get_value()` is a level (PROS_ERR on refusal); `get_new_press()` CONSUMES the press | reasoned | R3 |
| HA-122 | SD: `usd_is_installed()` returns 1/0; fopen NEEDS the /usd/ prefix (list_files FORBIDS it); fflush is the strongest persist | **partial** 2026-08-18: beliefs (1) and (2) CONFIRMED on hardware — `usd_is_installed()` returned 1 with a card and 0 without, and `fopen("/usd/probe.txt","wb")` succeeded WITH the prefix; a 17-byte fwrite + `fflush`==0 landed. Belief (3)'s DURABILITY half (yank the card after a flush, count what survived) NOT tested, so this stays open | R3 |
| HA-123 | A per-tick travel above 36 in — from one tracking pod, or one DRIVE SIDE of a drive-encoder odometry — is corruption, not motion | **invented** — **AMENDED** 2026-09-26 (DIAG1): extended to drive-encoder sides (the bound has applied to them since 2026-09-14; this entry had not said so); blast radius corrected — a spurious trip ENDS the active motion | R3 |
| HA-124 | Robot two's drive wheels are 2.75 in in diameter | **reported** (team lead, 2026-09-14; "standard wheels") — not yet rulered on this robot | R3b/R3d |
| HA-125 | Robot two's track width (contact line to contact line) | **UNSET** — no value exists in the tree; the library program refuses to boot until a tape measurement is typed in | R3b/R3d |
| HA-126 | Robot two's motor→wheel external ratio | **UNSET** — "600 rpm" was reported, which is a cartridge, not a tooth count; the library program refuses until direct drive or the counts are stated | R3b/R3d |
| HA-127 | Robot two's brain runs VEXos 1.1.5 (b18) under PROS kernel 4.2.2 | **measured** 2026-09-10 (the kernel banner over USB) | — |
| HA-128 | Starting a program from the controller's own menu arms a pretend field-control MATCH on the brain (status 0x0f) that persists until the controller leaves that mode; a linked controller on its home screen does not | **measured** 2026-09-10 (two sessions, the decisive test with hands off the controller) | — |
| HA-129 | The brain's USB CDC ports renumber between plug-ins (ttyACM0/1 alone, ttyACM1/2 beside the controller) and must be re-resolved from `pros lsusb` for every read | **measured** 2026-09-10 | — |
| HA-130 | Robot two's port 18 travels ~20 % short of its side-mates on a hand push, reproducibly (137 vs ~175 deg, then 171 vs ~210 deg) — a member SLOW, not a fight | **measured** 2026-09-10 (twice); cause UNKNOWN — watch it under power | R3b/R3d |
| HA-131 | On robot two, ODO_STUCK cannot be detected by the wheels-vs-odometry cross-check: the odometry IS the drive encoders, so the check is a tautology and is deliberately not wired to report a verdict | reasoned (structural) | R3d/R4 |
| HA-132 | The coupled-side agreement thresholds — evaluate above 1 V and 1 rad/s; opposite sign above 0.5 rad/s; under 25 % of the side's fastest; 25 consecutive 10 ms ticks — separate a real fight from a transient | **invented** | R3d/R4 |
| HA-133 | A coupled member under 25 % of its side's fastest is disagreeing, and one at 80 % (port 18's signature) is NOT — the disagreement FLOOR tolerates a slow member and catches a frozen one | **invented** | R3d/R4 |
| HA-134 | On a tank, driving ONE side forward alone turns the robot toward the other: LEFT forward → CLOCKWISE → NEGATIVE canonical yaw; RIGHT forward → positive. The calibrator reads a drive port's SIDE off that sign — from a pulse that RAMPS 3 → 6 → 9 V until the turn clears the floor, classifying a port no level turns (since R3d Part 2F-1) | reasoned (rigid-body) | R3d Part 3 |
| HA-135 | The EFFECTIVE track width a powered spin measures is LARGER than the tape's contact-line-to-contact-line width, because a turning tank scrubs; the effective one is what the kinematics and the heading cross-check need | reasoned | R3d Part 3 |
| HA-136 | Every number in `calibration::CalibratorConfig` — the push threshold, the rest floors (motor velocity and, since 2F-1, IMU yaw rate)/settle/timeout, the pulse levels/time/yaw floor, the decoupled minimum, the push distance (a CHOICE), the straightness bound, the minimum push, the spin levels/start check/wrong-way bound/turns/timeout/agreement, the IMU-ready and missed-tick timeouts — plus the two reproducibility tolerances the host suite pins, and the calibration screen model's text budgets, line heights, hit padding and arming delay | **invented** | R3d Part 3 (the screen's: Bench Tests' SCREEN RULER) |
| HA-137 | A coupled train whose members are commanded DIFFERENT volts turns at the MEAN of their effective volts (the A2 plant's model; torque-free); since R3d Part 2F-1 also: an all-coasting wheel may FOLLOW the driven one by a fraction κ (default 0 = pinned), and a slipping (`Decoupled`) member's volts never reach its train | **invented** (sim shape) | R6 |
| HA-138 | `registry_get_plugged_type(i)` is ZERO-indexed: registry index i is PHYSICAL PORT i+1, and indices run 0–20 | reasoned (vendored text) — and the ONE conversion lives in `hal/pros/device_registry.hpp` | R3d Part 3 |
| HA-139 | `fopen(path, "rb")` under `/usd/` reads back the exact bytes `"wb"` wrote, byte for byte, on the brain's FatFS | reasoned, flagged — the host CANNOT test the mode (POSIX does not translate) | R3d Part 3 |
| HA-140 | A CRC-32 over a 180-byte record is worth its 4 bytes and its microseconds on a V5, where the blackbox's own format deliberately carries none | reasoned (design note under HA-122) | R3d Part 3 |
| HA-141 | The registry's KIND answer is right for an IMU, a rotation sensor and a GPS (not only for a motor): a port carrying one reads that kind, and an empty one reads NONE | reasoned (vendored enum) — the on-robot provider refuses a port on this answer ALONE for the three kinds whose adapters cannot fail loudly | IMU half: G1 Part 2b's first boot; the provider's kind refusal: the first program that asks it for a GPS or rotation port / R3 |
| HA-142 | `move_voltage()` (and `move()`) IGNORE the brake mode; only `brake()` applies it — so a stop is "mode + `brake()`", never "mode + 0 V" | reasoned (vendored text, stated twice) — the DEFECTS2 fix rests on it | R3 (an ENABLED Brake stop vs a Coast release — a disable is confounded by HA-145) / T3's claw station |
| HA-143 | At `move_voltage(0)` a V5 motor free-wheels — the "coast" `IMotor` promises, which HA-137's coast-follow and Calibrate's pulses rest on | **invented** (UNKNOWN — no PROS text says what 0 V does; robot observations are qualitative) | R3 / R4 |
| HA-144 | A REPEATED `brake()` under Hold re-captures the held position (so one per tick under load lets a shaft creep); the adapter issues one per transition into zero. Second half: whether a same-mode `set_brake_mode()` written to a motor already held does the same (still sent every tick by a caller that re-states it) | **invented** | T3's claw station |
| HA-145 | What a motor stopped under Hold (or Brake) does while VEXos has the robot DISABLED, and after re-enable | **invented** (UNKNOWN) — the adapter's stop record cannot see a disable | T3's claw station / COMP |
| HA-146 | After `brake()`, the next `move_voltage(v ≠ 0)` drives the motor at once (the stop is replaced, not latched); a `set_brake_mode()` repeated every tick changes nothing but the register of a RUNNING motor (a held one: HA-144) | reasoned (vendored text) | R3 / T3's claw station |
| HA-147 | A V5 controller's connected flag and its button levels change together: on the first tick `is_connected()` is true after a disconnect, `get_digital()` already returns the LIVE level (never a stale one from before the drop), and a button's level does not bounce across two 10 ms samples | **invented** (unmeasured; HA-103 claims only that a disconnected controller reads 0) — the pneumatic claw's toggle takes a connected tick's level as live | Station J.9.5 (robot two's claw) |
| HA-153 | Robot two's two claw pivots are separate V5 motors with RED (100 rpm) cartridges on smart ports 6 (R1 forward / R2 back) and 7 (L1 / L2), and a POSITIVE command moves each the way the team lead calls "forward" | **reported** 2026-09-26 (team lead); NO capture has seen a motor on 6 or 7, and the way each turns is not even reported | worksheet Station P (Bench Tests' station 13, MECH-A6), then Station J |
| HA-154 | A claw pivot held at 12 V against its mechanical stop draws up to the motor's DEFAULT current limit (2500 mA — nothing in shulib changes it while the pivot's declaration states no `.currentCap`, as robot two's do; since MECH-S-A a stated cap lowers it, HA-219) for as long as the button is held, and heats; shulib watches only the DRIVE motors' temperatures | reasoned (vendored `motors.h`; a stalled motor heats) — every TIMING **invented** (none is stated) | T3 / worksheet Station J |
| HA-155 | A cable swapped between a claw pivot and a drive motor is invisible to the brain's registry (a MOTOR on both ports either way), so no check before power can see it | reasoned (the registry reports a device's KIND, never which motor) | T3 / worksheet Station J (a person, not the registry) |
| HA-156 | A claw pivot released in Hold (0 V, after DEFECTS2 makes a Hold reach the motor) keeps its angle under its load — its own weight and a game piece — with a sag over 30 s too small to matter | **invented** — no load, sag or hold current measured; "holds" is claimed nowhere until Station J logs it | T3 / worksheet Station J |
| HA-157 | While the brain is DISABLED (between autonomous and driver control, on a field or competition-switch disable) a 3-wire digital output KEEPS the level the program last commanded, and still has it after the re-enable; and a level the program commands WHILE disabled (a Robot built in `initialize()` on a field, or a program started from the controller's menu, HA-128) reaches the valve — at once, or at the latest at the enable | **UNKNOWN** — nothing vendored says; the pneumatic claw's driver-control toggle commands NOTHING on a disable and relies on the first half; the claw's start level on a field relies on the second | Station J (robot two's claw) |
| HA-158 | Nothing the program can read sees a pneumatic solenoid on a brain 3-wire port: the registry is indexed by SMART port, and a valve on A..H is a passive load — a connected line and an empty one are indistinguishable | reasoned (vendored registry + ADI shape) — the on-robot provider refuses a 3-wire line only for being outside 1..8 | Station J (robot two's claw) |
| HA-159 | Robot two's pneumatic claw is driven by ONE solenoid line on the brain's 3-wire port A (line 1), and that line HIGH OPENS the claw (the claw's `.positions` line: closed = `Low`, open = `High`) | **reported** 2026-09-26 (team lead: "port A") — single or double solenoid and the polarity **UNKNOWN** (the team lead said so); open = `High` is a starting guess | worksheet Station P (P.5, Bench Tests' station 13), then Station J.9 |
| HA-160 | The V5 IMU's heading is CONTINUOUS tick to tick while it reports ready: two reads ~10 ms apart differ by no more than the robot turned in between — never by tens of degrees | **invented** (assumed since M2, unregistered until DIAG1) — **QUESTIONED** by 2026-09-25: in one start, during a hard reverse, the reconstructed heading swung up to 37° per 10 ms tick — 8 of the last 9 ticks at 16.9° or more, alternating in sign; cause UNKNOWN | R3 / R4 |
| HA-161 | PROS runs `competition_initialize()` / `disabled()` / `autonomous()` / `opcontrol()` each on a task it DELETES on a mode change (a deleted task runs no destructor) — and does NOT delete a task the program created itself | reasoned (the belief code comments have carried since R3b Part 0b; no vendored PROS text says either half, and neither has been observed directly) — the competition program's own auton task rests on the second half | COMP-b's practice-match station (K) |
| HA-162 | Started from the controller's own menu with **Timed Run**, a program comes up DISABLED (HA-128) and VEXos then ENABLES `autonomous()` after a disabled window of __ s, for __ s, then __ — every blank unmeasured | **unknown** (no vendored text; HA-128 measured only the start) | COMP-b Station K, K.3 (its first match step) |
| HA-165 | The present members of one drive side stay close enough in CUMULATIVE position, through a match's driving and slip, that leaving members out of the side's median (one or several whose reading is not finite, DIAG1 Part B) moves the side by far less than `maxTickTravel` (36 in) — at worst, the member farthest from the median read alone | **invented** — the members' spread over a match has never been recorded; HA-130's port 18 is the evidence it can grow; exercised only when a member read is non-finite, which HA-08 says never reaches a group on a V5 | R4 |
| HA-166 | On the brain's SD card a program has no rename, no delete, no stat and no read-write open — the safe save is built from a whole-file read and a truncating replace: fopen "wb" empties an existing file at the open (an "ab" open goes to a different VEXos call, not measured, which the save does not use); rename(), remove()/unlink(), stat() and mkdir() fail (ENOSYS); a read-write open fails (EINVAL); a crash while one file is written leaves every other file intact; a file flushed, closed and read back whole is on the medium before the next file is opened; two names that differ only in letter case or trailing dots and spaces are ONE file | **reasoned** (1)-(3), and (5)'s sync call, from the vendored libpros.a/libc.a disassembled 2026-09-26; (6) from the FAT rule, **invented** that VEXos follows it; **invented** (4) and what (5)'s sync commits; none measured | Station G, once SAFE-SAVE B1 is in the build (a) / Bench Tests' SD probe (b, unowned) / a deliberate power cut (c, unowned) |
| HA-167 | A tank drives BACKWARDS the way its calibration measured it: each side's travel scale, from ONE unpowered push made one end first, holds within 1 % under power in BOTH directions, and a change of direction adds under 0.25 in of travel the chassis did not make (no backlash take-up) | **invented** (the scale comes from one UNPOWERED push in one direction; the host test plant has no backlash and is direction-symmetric by construction) | the first on-robot autonomous session (COMP-t's auton tester, or COMP-b's Station K) |
| HA-170 | Every number the GAINS and REST steps judge by — `FitConfig`, `CharacterizerConfig` and `RestCaptureConfig`: the 2 V/s ramp to 10 V, the 6 V step, the 36 in / 8 s segment, the curve guard's 0.30 asymmetry, the 12 in stop bound, 45 °C, the 2.3 A current guard, the fit's admission and ranges, the 60 s capture and its bounds — is a first-run choice | **invented** | R5a's robot sessions (gains S2, rest S3) |
| HA-171 | A V5 drive side follows V = kS·sgn(v) + kV·v + kA·a closely enough that gains fitted over the ramp's range hold outside it (full stick, turning); `velocity()` is current within a tick and unfiltered; a command takes effect within one tick. The fit's PoorFit verdict is NOT a mismatch detector — a quadratic drag never reaches it, and noise on `velocity()` biases kA low and kS high under an Ok verdict (measured on synthetic data generated off the law) | **invented** | S2 (R5) |
| HA-172 | Inside the ramp's range the only non-linearity is the per-motor current limit (the PROS default, 2500 mA — whether VEXos lowers it with 12 motors plugged in is UNVERIFIED), `current()` reports it within a tick, and the firmware adds no slew of its own to `move_voltage` | reasoned (the limit) · **invented** (the rest) | S2 (R5) |
| HA-173 | A stationary V5 IMU's drift is a PER-BOOT bias (HA-20), never a correction for a later boot — and whether one 60 s capture separates it from noise depends on the noise's STRUCTURE (white heading noise, or integrated rate noise), which the host cannot show | reasoned (per-boot) · **invented** (the structure) | S3, ≥ 10 boots (R4) |
| HA-174 | Gains measured once hold for the match: the same foam tiles, a charged pack (HA-47), the same drive members — a member absent at a later boot leaves kV roughly unchanged and raises kS roughly as 1/(members present) | **invented** (surface) · reasoned (members) | S2 (R5) |
| HA-175 | A robot-two drive port that stops answering mid-run shows through the IMotor seam as a FROZEN reading — position(), velocity() and current() all repeating their last good values bit for bit (ProsMotor's screen); a LIVE member is never FROZEN for 25 consecutive 10 ms ticks — FROZEN being position AND velocity bit-identical for 5 reads while its own report says it turns (above 0.5 rad/s) or a side-mate whose ENCODER moved within the last 5 reads says the train does (above 1 rad/s), with at least as many of its side answering as not — because a live motor's velocity report changes while it decays and a stall stops its mates' encoders too (brief spells as a train coasts to a stop are expected and harmless), EXCEPT at a wall beside a slipping side-mate (HA-130's port 18), whose encoder keeps turning, where the guard holds only while every stalled member reads alike (the known gaps, in the long form; at a plain wall, one report settling sooner now costs one 1 s fight cut instead of a mark — the team lead's encoder check, 2026-10-01); and a side still drives acceptably with its dead dropped, up to the mid-run stop at more than two dead in total | reasoned (the adapter's screen, `hal/pros/motor.hpp` `position()`, `velocity()`, `current()`; BLUE's 300 counts per output turn) + **invented** (the 5 reads, the 250 ms latch, the majority-or-tie guard; the floor, R3b Part 0b) + host-probed only (a slipping member at a wall, with and without one stalled member reading unlike its mates; a plain wall with one report settling sooner; a coast-down with per-member encoder phase; 2026-09-28/30, 2026-10-01) | T4 Part 3's Station F.6 (i); the DEGRADED boot with one port unplugged per side; a full-stick wall push watched for a false DEAD on any drive port, port 18 (HA-130) included — the push check's push (HA-176): read its log for a `went ABSENT` line and a `CUT: FIGHT` line |
| HA-176 | On robot two a drive motor above 2.4 A for 250 ms while its side is driven above 1 V means something is WRONG (a fight on the coupled train, or a stall) and not ordinary driving — a full-stick reversal at top speed and a full-stick launch from rest included, each believed to hold the current limit for less than 250 ms — and a drive motor CAN draw more than 2.4 A (the per-motor limit stays at the PROS default 2500 mA with twelve motors plugged in, HA-154) | **invented** (R3b Session 2's first-run number, the old "shulib Drive"'s `kCurrentLimitA`; no current trace of robot two under power exists) | the competition program's push check (its practice-match station, K), with a full-stick reversal at top speed and a full-stick launch from rest; T4 Part 3's Station F: the highest A at full stick wheels-up, then F.6 (ii) |
| HA-177 | An APPEND open under `/usd/` (`fopen(path, "ab")`, which PROS hands to VEXos's `vexFileOpenWrite`) writes after an EXISTING file's last byte and keeps every earlier byte; on an ABSENT file it either fails or creates (the adapter handles both); a line written and flushed survives a power-off right after it | (1) reasoned (the vendored `libpros.a`/`libc.a`, disassembled 2026-09-26); (2)–(3) **invented** (VEXos's); (4) reasoned for the call, invented for what the sync commits | COMP-t Station M.7 |
| HA-178 | The Auton Tester's abort and RUN gate read the right things from PROS: `press_count` rises when a finger LANDS; a controller button's level turns true within ~50 ms; with nothing connected and the tester started from the brain, `competition_get_status()` reads 0x00; a controller switched OFF reads `is_connected()` false within ~1 s — each read from a task the program created | (a) reasoned (the vendored text); (b), (d) **invented**; (c) reasoned (HA-128's measurement) | COMP-t Stations M.2–M.4 |
| HA-179 | TANK1's tank-steering constants (bend 0.1 rad/in capped at 15°, fade 6 in, lead 0.06 s; yaw observer 0.1 s, limit 1.5 rad/s) remove robot two's weak-side drift on a tank `moveTo` within the unchanged 0.5 in / 0.02 rad rule, without heading oscillation — its drivetrain follows a yaw command much faster than 0.1 s | **invented** (tuned on the host test plant's placeholder dynamics; with unmodelled motor lag the plant's margin shrinks — measured) | COMP-t Station M.6 / R5 |
| HA-180 | Robot two's sideways error on a straight tank `moveTo` comes from its HEADING being pulled (the IMU sees it), not from the chassis sliding sideways (which no sensor on robot two sees) | reasoned (the 2026-09-25 veer was a heading change read on the IMU; a skid was never measured) | COMP-t Station M.6 |
| HA-182 | The mechanism core's STARTING numbers hold a real arm, lift and flywheel well enough to be TUNED from: position kP 0.25 V/deg, kI 0.3 (the hold loop only; its I-term clamped to 3 V), kD 0.004, kS 0, kA 0, kV = 12 V over the output's free speed, the smooth move at 75 % of the free speed reached in 0.25 s, "reached" within 1.5° (0.25 in) for 0.1 s; speed kS 0, kV = 12 V over the free speed, kP 0.05, kI 0.2 acting only within 10 % of the target, its I-term clamped to 3 V — the gains, profile and "reached" per mechanism (a boot line words them; `.tuning` replaces each), the two integral clamps and the window LIBRARY constants (no `.tuning`); the driver flywheel toggle's 5 s at-speed window (`kToggleAtSpeedWindowSeconds`, a LIBRARY constant, MECH-A4); the bench page's own numbers (Bench Tests station 13, MECH-A6 — `src/programs/bench_r3a.hpp`'s `mech_station`): the jog ceiling starting at 3 V (R1: 6 / 9 / 12), the direction check's + jog of 0.4 s at the ceiling, the 1 s announcement before that jog (nothing driven; its last 0.4 s a settle window — a motor that turned 0.035 rad or more in it abandons the check), its "did not move" 0.035 rad and its "drawing" 0.40 A (judgeDirection's two thresholds), a jog's 3 s maximum, the 2.4 A / 250 ms cut | **invented** (the host test plant's 144 mismatched moves and 27 flywheels are its only evidence — HA-183; the bench page's numbers have none) | tuning each mechanism on the robot (MECH on hardware) — the toggle's window by timing a real flywheel's spin-up to its named speed; the bench page's at its first run (worksheet Station P) |
| HA-183 | The host test plant's mechanism models (`sim::ArmPlant`, `sim::FlywheelPlant`) stand in for a real V5 arm, lift, roller and flywheel closely enough that the mechanism core's host verdicts transfer: the inverted-feedforward joint (static band kS, speed per volt kV, first-order approach τ = kA/kV), gravity as a load in volts (kG·cos(θ − level) for an arm, kG for a lift), the brake modes at 0 V (Hold's restoring stiffness, Brake's damping, Coast free), the current's shape (\|V − kV·ω\|·2.5/12, capped at 2.5 A — HA-93's), and a hard stop or a jam reading ZERO speed at stall-grade current | **invented** (no constant is measured; a real stalled motor creeps — HA-93 — the plant's does not) | the first robot session that logs a real arm's and a real flywheel's step response beside the model's (MECH on hardware) |
| HA-184 | A VEX limit switch read through `ProsDigitalIn::state()` reads 1 (TRUE) when PRESSED, so the mechanism core's `pressedLevel = true` is right; and an UNPLUGGED switch line reads RELEASED | **unknown** (the vendored `get_value()` doc says only "the digital value (1 or 0) of a pin") | MECH-A6's Bench Tests station 13: the switch read pressed, released AND with its cable out |
| HA-185 | Robot two's pneumatic claw moves between closed and open — either way — within 0.5 s of its line's level changing, so an auton's `set()` step on it (which, with no sensor on the claw, ends when the declared `.moveTime` has passed — the open SUCCEEDED, the grab UNCONFIRMED and counted done) does not count the move done before the claw has moved | **invented** (nobody has stated or timed it; the core refuses a pneumatic with no `.moveTime`, so a number had to be written) | Station J (J.9): a phone's slow-motion video of closed → open and back |
| HA-186 | At power-up a V5 rotation sensor's CUMULATIVE `get_position()` (what `ProsRotation::position()` reads, HA-11) starts at the shaft's ABSOLUTE angle within one turn — and so it does after a PROGRAM restart without a power cycle — so `absoluteSensor()` homing needs no motion | reasoned (VEX's description of the sensor as absolute within a turn; the vendored doc says only "current position") | MECH-A6's station 13: power-cycle at two marked angles; move the arm, restart only the program, read again |
| HA-187 | Calibrate's WATCH reads a DECLARED mechanism motor without changing it: PROS's C reads motor_get_position(), motor_get_encoder_units() and motor_get_gearing() by port write nothing and construct no motor object; the position, converted by the units and gearing they report, gives the shaft's turn between two reads of one push | **reasoned** (1) from the vendored libpros.a disassembled 2026-10-08; **invented** (2) and (3); none measured | Station G |
| HA-189 | Two tasks writing the V5 USB serial (stdout) at once never mix or lose each other's lines — `ProsCharSink`'s "fwrite is atomic per call at this layer" — believed FALSE: the vendored newlib's per-FILE locks are no-ops, stdout is line-buffered (`ser_isatty_r` returns 1) through ONE shared buffer, and only PROS's device write below it takes a mutex. **FIXED in shulib's code by SER1 (2026-10-08):** every line is ONE direct `write()`, which that mutex keeps whole — unconfirmed until the bench | **reasoned** (FALSE — from the vendored `libc.a`/`libpros.a`, disassembled 2026-10-08) | the coordinator, T4 Part 3's hand-over; SER1's fix; Station K.7.3 (HA-189's robot test — accepted by the team lead 2026-10-18 in place of the dedicated two-task test; unsettled until it runs) |
| HA-190 | A task PROS deletes while it is INSIDE `ser_write_r` (one serial `write()`) leaves PROS's serial write mutex held, and every later serial write of that boot — stdout and stderr alike — blocks its writer for ever | **reasoned** (the vendored `libpros.a`: `ser_write_r` holds the mutex across `stream_buf_send`'s unbounded waits; `task_delete` releases no mutex) | SER1; Station K.7.3 (Timed-Run mode changes while two tasks print) |
| HA-191 | One serial `write()` must fit PROS's 2047-byte serial stream (about 2,035 bytes of text: a 4-byte stream id, COBS framing and a delimiter around it); a larger one waits for ever holding PROS's write mutex | **reasoned** (the vendored `libpros.a`: `ser_driver_initialize` +0x38, `stream_buf_send` waits until the WHOLE frame fits) | SER1 (no shulib call offers one `write()` more than 512 bytes — `kSerialWriteMaxBytes`; `serialLine` 256) |
| HA-192 | The AI Vision camera's data is its LATEST frame, ~30 a second (VEX's figure), with NO counter and NO timestamp: a read's completion time is the only freshness, a frozen camera looks like a still scene — and the IMAGE is older than the read by an UNMEASURED age `L` | reasoned (VEX's rate through search snippets only; the vendored header and binary have no counter; `L` unknown) | Step 0.2 (no new code: Bench Tests station 12 as it stands — the change timer); Station I 0.7 after VIS1-B (the LIBRARY page's latency check) |
| HA-193 | The camera reports 320 x 240 with (0, 0) at the top-left, +y DOWN, a box as its left edge, top edge, width and height | reasoned | Step 0.2 (station 12 as it stands: a Pin moved left/right and up/down; a box at the edge reads 0 / 319 / 239) |
| HA-194 | The camera's optics: 74 x 63 degrees (two focal lengths, 212.3 and 195.8 px), the principal point at the centre, no distortion worth modelling, +-5 % — and the optical axis square to the housing's back face | reasoned (VEX's figures through search snippets only) | Step 0.2 (a Pin on the centreline at 24 and 48 in); Station I after VIS1-B (Pins at known offsets) |
| HA-195 | A camera read is NOT atomic (the count, then each object); a `get_object` whose port claim fails returns {id 127, type 127, the rest 0}; what the SDK returns past its own count is unknown | reasoned (the 127 record read in the vendored `libpros.a`; its SDK half not) | Station I 0.7 after VIS1-B (the LIBRARY page's partial-fault count; unplugging mid-read) |
| HA-196 | `set_enabled_detection_types(bits, mask)` sets exactly the masked mode bits; the modes and a colour's RGB read back as written (whether the camera KEEPS them is not assumed — they are written every boot) | reasoned (the vendored doc's one example and the binary; the SDK's own semantics unread) | Station I 0.7 after VIS1-B (`setups` and the fault count across a re-plug, `settingsLost` for a setting found changed; the mode line after the write) |
| HA-197 | The AI model is chosen in VEXcode's AI Vision Utility and KEPT on the sensor; PROS reads each class id's NAME (and no count — `get_class_name` returns 1 for every id); the Override model names ids 0–4 `redBluePin`, `blueYellowPin`, `redYellowPin`, `yellowPin`, `cup` | measured elsewhere (the names, api.vex.com); reasoned (kept on the sensor; the PROS path) | Step 0.2 (station 12's RECORD lines with each piece in view, before and after a power cycle) |
| HA-198 | The order of the 24 detections and the Object score's raw scale are UNKNOWN — nothing in VIS1 depends on the order, or on whether a count above 24 can be reported (24 or more is "possibly full", never evidence); the scale is used in one place only, the piece finder's score floor (`PieceFinderConfig::minScore`, VIS1-A2), a TEAM number on the raw scale chosen at Step 0.2 — the library assumes no scale | reasoned | Step 0.2 (the order against the sizes, the scores, how often a crowded field reads 24) |
| HA-199 | The three taught colours separate the Toggle's faces under event light, the field face's band present on every read its bottom face's is, and the lookalikes fire no band at Toggle height or fail a gate — except those VIS1's tests name (a strip on a Toggle's own face line passes even the position gate) | **invented** | Station I 0.5 / 0.6 after VIS1-B, then the event field (0.8) |
| HA-200 | The AI model's box encloses the WHOLE piece: its bottom row is where the piece's base meets the floor (or its support), its height the piece's | **invented** | Step 0.2 (a still Pin at 12 / 24 / 36 in — standing, lying, on a Goal) |
| HA-201 | The Toggle's geometry: the field face's lower edge 12.55–12.65 in up, the apex 14.39 in and 0.9–1.2 in behind it, the FLAT bottom face 12.36 in, a different colour; its distance from the wall's inner face unknown | reasoned (arithmetic from the manual's pA8 and glossary, v2.0) | Step 0.1 (a real Toggle, a tape measure; the newest manual) |
| HA-202 | A REPORTED box edge is within ~2 px of the true edge for a still object — rounding and bias included — for a model's box AND a colour band's top and bottom rows | **invented** | Step 0.2 (a still Pin, five reads); Station I 0.5 after VIS1-B (a still Toggle face) |
| HA-203 | One camera read (1 + N device calls, N up to 24; once a second up to 28 more for the adapter's settings-and-model check; up to 36 more for a set-up written again) is cheap enough to make once a period inside the driver loop's 10 ms tick | **invented** | Station I 0.7 after VIS1-B (station 12's read time); the driver loop's tick monitor under AF1 |
| HA-204 | The V5 Distance Sensor reads a flat, wall-sized target to ±15 mm below 200 mm and ~5 % above, over 20–2000 mm, from its front face — the wall reset assumes max(15 mm, 5 % of the reading) at every range until a measured `rangeAccuracy` replaces it | **reasoned** (VEX's published figures; the zero point and the real spread unknown) | L6 Step 0.3 / 0.7 |
| HA-205 | The beam is a cone of about ±12° (long range) to ±18° (short range); within 5° of square to a flat wall the reading is the beam-centre range; with two surfaces in the cone the sensor reports the NEAREST | **invented** (a community post seen through search summaries; VEX publishes no angle and no multi-target behaviour) | L6 Step 0.5 / 0.6 |
| HA-206 | The field wall's inner face returns a reading at the sensor's height — the portable perimeter's opaque walls, the metal perimeter's clear panels and the GPS code strip's black and white squares (9.06–11.44 in) alike | **invented, with CONTRARY evidence** (a forum report of the beam passing through clear panels) | L6 Step 0.3 / 0.6 |
| HA-207 | The perimeter's inner faces are ±70.205 in from the field's centre (A10/A13; the metal perimeter A14 ±70.25) and a real field is within ±0.5 in of its drawing | **reasoned** (the drawings) · **invented** (the tolerance) | L6 Step 0.6 |
| HA-208 | Each Toggle's BODY sits on top of its wall (overhang ≤ ~1 in), so a beam below the wall top (11.50 / 11.54 in, printed) reads the wall beneath it; its two BRACKETS (\|along\| ≈ 10.9–13.0 in) hang to ≈ 10.6 in | **reasoned** (the printed heights; the bracket's bottom off A8's scale) · **invented** (the overhang, the brackets' depth) | L6 Step 0.6 |
| HA-209 | The sensor's update interval and latency are UNKNOWN; repeated reads within one update repeat it, and a `distance()` / `confidence()` pair can straddle an update | **invented** (the interval, the latency) · **reasoned** (the two reads) | L6 Step 0.7 |
| HA-211 | The wall reset's gate defaults (range 2–16 in, square 5°, spread 0.5 in, ≥ 5 samples over ≥ 0.15 s, confidence 0.5, still ≤ 1 in/s and 3°/s, bound 1.0 in, keep-out margin 3 in + 10 % of the along-wall travel, heading tolerance 2° + 2 % of the rotation, the 6.5-in piece + 0.5 in and an 18° cone for the beam-height ceiling; stillness as the estimate's displacement over `window`; the Localizer's turn and travel counters in 1° / 0.5-in steps, which assume a still robot's IMU heading and odometry path never move a full step) | **invented** (each derived on paper, none observed) | L6 Step 0.3–0.8; a 60-s still-robot capture |
| HA-212 | The Loaders (4.02 in wide, 3.74 in deep, ±58.77 in on the red and blue walls), A12's start pieces (6.5 in tall, against every wall at \|along\| 18.81–28.28 in), VEX U's match loads in the Load Zones during the auton, and no single piece taller than 6.5 in | **reasoned** (A9/A10/A12, FO-1, VEXU-1, VURS1) | L6 Step 0.6 |
| HA-213 | The V5 IMU's readings (`get_rotation()`, `get_gyro_rate()`) are copied into the brain's shared buffer every 10 ms, so every read between two copies returns the SAME value (sample-and-hold) | reasoned (the vendored `pros/imu.hpp`, `set_data_rate`'s doc); never observed on our brains | IMU1 (Part B's bench stage, Station N) |
| HA-214 | The phase of that 10 ms copy against the motion tick's wake is unknown (fixed per boot, or drifting); a copy within a read's jitter of the tick's ONE rate read, or between two positions of the tick's FIRST read, makes a pair of ticks read 0 and about double — IMU1's hold does not cure it | **invented** (the phase) · reasoned (the consequence) | IMU1 (Part B's bench stage, Station N) |
| HA-215 | Every rate read of one control tick falls within 5 ms of the tick's first, consecutive ticks' first reads are 5 ms or more apart, and no reader outside the loop's tick reads the rate — what `ProsImu::kRateHoldUs` (5 ms) rests on | reasoned (HA-32/HA-102, themselves unmeasured under load) | IMU1 (Part B's bench stage, a lower bound); Bench Tests STAGE 5 |
| HA-216 | A leg that ends under a loosened arrival tolerance — still moving at up to the scaled speed bound (4 in/s at 2 in; a turn up to 1.31 rad/s, ~75°/s, at 5°) — comes to rest within that tolerance on robot two | **reasoned** (the host test plant stops dead on the exit tick; robot two's coast unmeasured) | an Auton Tester run of the 40 in leg at 2 in, tape-measured; the 90° turn at 5°, the END heading against the IMU at rest |
| HA-217 | A partway trigger's mechanism starts within ~1–2 ticks (10–20 ms) of the position estimate crossing its mark: the command is issued inside the drive's tick, and the mechanism service acts on it at the next pace | **reasoned** (the host test plant's service and motors answer the same tick; at 40 in/s a tick is 0.4 in of drive) | a bench run logging the mark's tick against the mechanism's first motion |
| HA-218 | A trigger fired as a leg ARRIVES (D6 (i): its mark ON the leg, the leg ended within its tolerance) puts the mechanism within the leg's tolerance of the author's mark, because the position ESTIMATE says the leg arrived — true only as far as the estimate is (odometry drift over the leg) | **reasoned** (on the host test plant the estimate and truth agree to ~0.01 in) | the 40 in floor run of HA-216, the estimate's END against the tape |
| HA-219 | A V5 motor given `motor_set_current_limit(L)` draws at most about L from then on — enforced by the motor itself within one 10 ms control tick, under any command, under Hold at 0 V, during a disable and with nothing ticking | reasoned (the vendored `pros/motors.h`: "The motor will reduce its output at 1000 mA instead of the default 2500 mA") · the speed **invented** | the Tune program (a later part): a capped lift pushed into its top stop, the current trace |
| HA-220 | A limit set by one program can persist in the motor into the next, and a motor that loses power mid-run comes back at its 2500 mA default — so the library READS every mechanism motor's limit at every boot (writing only a stated cap) and re-checks capped ones each second | **unverified** (HA-98's open question, extended to the limit) | MECH-S Step 0.4: a cap set, the program stopped from the brain, the next program's boot read-back; a capped motor's cable pulled and re-seated |
| HA-221 | With more than 8 motors plugged in, VEXos may lower each motor's limit below what was set (robot two runs 10); a cap below VEXos's own share behaves as stated — and writing 2500 mA over a share at every boot would fight it, so a mechanism with no cap is only READ | **unverified** (HA-172's open question) | MECH-S Step 0.4: robot two's boot read-back with all 10 motors plugged in |
| HA-222 | The string's pull is the motor's current times its torque per amp through the gearing over the spool's radius — about 10 lbf per amp per motor for a red cartridge on a 1.5 in spool; the rotor's and the carriage's MOMENTUM at a hard stop add a pull no current cap limits | reasoned (physics) · the number **unverified** (the published stall torque, from memory) | a spring scale on the string at a held current |
| HA-223 | A stalled motor held at its cap reads within ~0.1 A of the cap, so a stall signature MORE than ~0.2 A below the cap trips (the boot's Warn for one not more than 0.2 A below — exactly 0.2 A included — rests on it) | **invented** (the host test plant reads exactly the cap — it is not a motor) | the Tune program: a capped lift held against its top stop, the reading |
| HA-231 | One flipper stroke rolls a Toggle exactly ONE face, always the same way round, and the retract does not roll it back; the research's per-side order (a red-side Toggle Y -> B -> R, a blue-side the mirror) — so from yellow every skills Toggle needs TWO pushes | **reasoned** (pA15 + geometry; AF1's research) | AF1S Step 0.1, on a red-side and a blue-side mounting |
| HA-232 | The Toggle seats (`<SC4>`a) within `settle` after the retract — after EVERY FULL stroke, never left stuck part-way — because the camera cannot check it: a Toggle stuck part-way reads its destination colour "sure" | **invented** | AF1S Step 0.1: twenty full strokes, each checked by eye |
| HA-234 | At the run's end a V5 motor in its declared safe brake mode does not move the flipper toward `back` — a flipper out at 0:00 stays out (why the step's time check and its retract-now rule exist) | **reasoned** (the guard safes, it does not retract) | a timed run with the flipper out at its end |
| HA-235 | A push stopped by a sticky Toggle is cut by the flipper's stall signature (or `flipBound`) before it damages the Toggle (`<R18>`a); a PNEUMATIC flipper has neither — nothing senses its rod, so its "full stroke" and "back" are only "the move time passed" | **invented** | AF1S Step 0.1 with the Toggle held |
| HA-243 | Each Toggle's Quadrant colour in F1: the RED wall's and the RIGHT wall's (x = +70.2) RED, the BLUE wall's and the LEFT wall's BLUE; every Toggle starts a skills run yellow into the field, its Quadrant's colour out | **reasoned** (Figure Q-1 with pA15, read 2026-11-10) | AF1S Step 0.7, on the event's field |
| HA-244 | The step's straight drive in and out of `lookFrom - pushFrom` puts the flipper within its reach of the face and the robot back within `offBy` of the look spot, on a square robot — because the ROUTE arrived within its allowance: the camera's spot check refuses only GROSS arrival errors | **invented** | AF1S Step 0.4, 0.5, 0.6 |
| HA-245 | On a real field the camera names the Toggle's colour from the look spot as the host camera model predicts (about 12–24 in out, within ~3 degrees of square, for a lens 6–12 in up — only the 10-in lens's figures are re-run and pinned by this release's tests; the 6-, 8- and 12-in ones come from an earlier build of the tracker, not re-run since): the wall below hides the bottom face from a low lens, the taught colours hold under event light, the flipper at stow is not in the picture | **invented** | AF1S Step 0.3, 0.4 |
| HA-246 | `CurveConfig`'s look-ahead (8 in), bend grip (60 in/s²), accelerations (200 / 120 in/s², the plan rising from 12 in/s), hand-off (12 in, 5°, 0.3 in; last at ⅓ inside TANK1's window) and end speed (×0.6) let robot two follow a curve and settle at its end within the default rule — and robot one on robot two's numbers until its own check — and guide 05's run-up rule (a miss up to 6 / 12 in from 2½ / 3 ft back, nose within 20°) holds on the robot within ~6 in | **invented** (host test plant: 12–17 of the 17 drivable routes under every weak side and lag tried; the run-up rule pinned by CV23; no slip in the plant, neither robot's lag measured) | Station C.1–C.3, C.5 (C.5b the run-up) on robot two; C.1 on robot one |
| HA-247 | Robot two does not slide sideways in a bend taken at sqrt(bendAccel / curvature) — its tyres hold 60 in/s² (≈ 0.16 g) of sideways push | **reasoned** (VEX tyres commonly grip well above it; never measured; the plant has no slip) | Station C.3 |
| HA-248 | A healthy robot two never points more than 60° off its curve for 0.25 s nor makes under 1 in of progress in 0.75 s outside the last 12 in, and a dead side does one or the other within ~3 s | **invented** (host: 0 trips in 153 healthy runs, worst 25.9° off; a dead side stopped within 2.7 s, early deaths included) | every Station C log (no false "curve lost"); the dead-side half host-only |
| HA-249 | Robot two's odometry (drive encoders + IMU heading) is as true on a curve as on a straight — the scrub of a skid-steer turn does not move the end by more than ~1 in on a 60 in curve | **reasoned** (never measured) | Station C.4 |
| HA-250 | A `curveTo` fits the auton task: its motion (≤ 6 KB by `static_assert`, 5,904 B measured on the host) on the default 32 KiB stack, its per-tick work well inside the 10 ms tick, and its one-off path build (at its first live tick) inside that tick for a field-sized path | **reasoned** (the ARM `-fstack-usage` frames of a 16-point `curveTo`, CURVE1-A: the verb's 6,528 B + the follower's tick 792 B; the deepest chain about 9.2 KB, about 28 % of the stack; the build ~0.1 ms on the host for the 143 in snake, ~0.76 ms for a 2,092 in 16-point path — the brain's speed and headroom unmeasured) | Station C's loop-monitor overrun count |
| HA-252 | Robot one is a TANK — wheels on two sides, no strafe — like robot two (the builder: "both robots identical") | **reported** (never seen) | the facts form's A1, at the robot |
| HA-253 | Robot one carries an inertial sensor (IMU) on a smart port — the library drives a tank only with one | **unknown** (the builder named only AI Vision and rotation sensors) | the facts form's A4; Bench Tests' census on robot one |
| HA-254 | Between two measured rows, robot one's cascade height is close to straight in its output angle (a string winds onto its spool in layers, and stretches) | reasoned · **unmeasured** | the facts form's C7: a row at every level; a mid-row tape check at station 13 |
| HA-255 | Robot one's two cascade motors face each other (`{Forward, Reversed}`) | **guess** (the builder's look, not yet given) | Bench Tests' station 13 direction check on robot one |
| HA-258 | The event's field is set up to Appendix A and Figure VURS1, so the red wall's and the right wall's Quadrants are mirror images across F1's y = -x (goals, Toggles, Pins, tape) — each element within `<T5>`a's +-1.0 in, so a mirrored target can land up to ~2 in from robot two's element where robot one's was tuned to its own — and the centre goal, two yellow Pins and the red/right corner's Load Zone sit ON the diagonal | **reasoned** (the manual's drawings A10, A11, VURS1 — the Pins by pixel, about +-1 in; v2.1's drawings not re-measured) | On the event's field: tape the two red goals and two Toggles from their walls |
| HA-259 | Robot one's build is left-right symmetric for every part a route uses at the field: a part off the centre line (an intake, a flipper, a claw, the camera's `left`, the two side sensors' `forward`) lands twice its offset from the reflection of robot one's | **unknown** | MIRROR1 Step 0.2 (every part's sideways offset), Step 0.3 (both side sensors' mounts) |
| HA-260 | The drivetrain's own left/right differences are NOT mirrored: robot two's arrivals differ from the reflection of robot one's by its own asymmetry (host test plant, a 9 % weak side: 0.56 in, 0.44 s) — and each robot's real position near the diagonal stays within half the pair check's gap (3 in) of its plan, and its whole robot inside the check's 15 in square centred on its tracking centre, so a route that passes the 6 in check keeps the two robots apart on the field | **invented** (the host plant's magnitudes; the 3 in is a chosen margin, unmeasured; the footprint's place about the tracking centre unmeasured) | MIRROR1 Step 0.4 on both robots; Step 0.5 (both together, slowly first); the tracking centre's place inside each robot's footprint, taped with Step 0.2 |

---

## Group R3 — conventions, bindings, geometry (hardware day one)

Walk top to bottom with the robot on a bench and then on a field tile. Every entry here is either
a **frame/sign convention** (a wrong guess mirrors or rotates the world), a **binding contract on
the R1 adapters** (a wrong binding is a bug the adapter must not have), or a **geometry constant**
(a wrong value is a measured correction). All of them are contained behind the `hal/pros` seam or
a config constant **by design** — a correction here never touches the core.

- [ ] **HA-01 — GPS position axes: VEX +X = East, +Y = North.**
  *Claim:* `pros::Gps::get_position()`'s x axis is compass East and y is compass North. False if
  the axes are swapped, negated, or bound to the field rather than the compass.
  *Source:* `include/shulib/hal/gps_conversion.hpp:15–21` (the flagged ASSUMPTION block);
  skipped oracle `test/gps_conversion_test.cpp:164`.
  *Confidence:* **invented** — PROS does not document the position-axis→compass binding at all.
  The oldest entry in this register (flagged "validate-on-field" since June).
  *Settle (R3):* the field-cal oracle: place the GPS at a known +1 m-East / +1 m-North point at a
  known heading, read raw `get_position()`/`get_heading()`, replace the oracle's expected values
  with the measured mapping, **unskip it**. It stays green forever after.
  *Blast radius if wrong:* a wrong axis label silently **MIRRORS** every GPS pose, and
  `northHeadingDeg` cannot recover it (a rotation knob cannot undo a reflection). Downstream: the
  E2 corrector would propose reflected fixes that fight odometry — Mahalanobis gating would reject
  most (quality decay, GPS effectively dead) and accept the rest near the mirror axis (real pose
  corruption). **Contained by design:** the fix is a few lines in `gpsSensorPose()` — one
  conversion function, zero core impact. The oracle exists precisely so this is found on a bench,
  not mid-match.

- [ ] **HA-02 — the as-mounted IMU heading is CW-positive, per its own doc strings.**
  *Claim:* a physical +90° clockwise spin increases `get_rotation()` by ~+90 (and therefore
  decreases canonical heading by 90°).
  *Source:* `include/shulib/hal/imu_conversion.hpp:9–13, 29–32`.
  *Confidence:* reasoned — both `get_rotation()` and `get_heading()` document "clockwise rotations
  are positive" in the vendored PROS headers; unverified against a physical sensor as mounted.
  *Settle (R3):* bench: rotate the robot +90° CW against a wall/protractor; canonical heading must
  decrease by 90°.
  *Blast radius if wrong:* globally negated heading — odometry arcs mirror left/right, every turn
  goes the wrong way. Caught within seconds by the R3 push/spin test. **Contained:** the sign of
  one subtraction in `imuHeadingToCanonical` is the documented line to flip.

- [ ] **HA-03 — `pros::Imu::get_rotation()` is cumulative and unbounded.**
  *Claim:* `get_rotation()` continues past ±360° without wrapping (unlike `get_heading()`), so the
  adapter's required binding preserves revolution continuity.
  *Source:* `include/shulib/hal/imu_conversion.hpp:16–19` (binding contract).
  *Confidence:* reasoned — documented "theoretically unbounded"; untested across many revolutions
  on a real unit.
  *Settle (R1 review + R3):* adapter code review pins the binding; bench: spin > 360° and confirm
  no discontinuity in canonical heading.
  *Blast radius if wrong (or mis-bound):* a ±360° step once per revolution; `PilonsOdometry`'s
  trust gate flags the tick (|Δθ| > π/2) but heading authority is corrupted from then on.
  **Contained:** adapter-only fix; the conversion function is already cumulative-correct.

- [ ] **HA-04 — the IMU yaw-rate source's sign convention.**
  *Claim:* differentiating `get_rotation()` yields a CW-positive rate (provably, since HA-03);
  `get_gyro_rate().z`'s sign — the cheaper source — is **undocumented** and assumed CW-positive if
  ever used.
  *Source:* `include/shulib/hal/imu_conversion.hpp:52–61` (the adapter caveat).
  *Confidence:* **invented** for `get_gyro_rate().z` (no documentation exists); reasoned for the
  differentiate-`get_rotation()` path.
  *Settle (R3):* bench: spin CW at a steady rate; the canonical yaw rate must be negative. Do it
  for whichever source the R1 adapter picks. **Since IMU1 Part B (2026-11-03): Bench Tests button 2's
  STAGE 2c, worksheet Station N** — the gyro's z integrated (trapezoid on `micros()` stamps) over one hand
  turn, beside the raw rotation's change between the first and last change instants of the same window,
  both raw PROS degrees, never `ProsImu::yawRate()` (canonical, and held 5 ms — IMU1's re-check R18): the
  same sign means z shares `get_rotation()`'s convention, so `GyroRateZ` and the differentiated rate agree
  in sign through `ProsImu`'s one negate (whether that convention is CW is HA-02's). The stage judges the
  sign whenever z is about the yaw axis — the two magnitudes agreeing within 10 % in deg/s, OR in rad/s (a
  sign does not depend on units) — and not when z is neither (another axis says nothing about this sign; the
  printed `z / rotation` line still shows it); built, never run on a robot — not settled until the bench
  prints it.
  *Blast radius if wrong:* a negated yaw rate breaks the drift-observability property Phase E
  depends on (heading and rate must move together — `imu_hostility.hpp` models them consistently
  for exactly this reason) and inverts E2's high-yaw-rate rejection. Since TANK1 (2026-09-27) it would
  also turn a tank `moveTo`'s yaw observer into positive feedback, bounded at its 1.5 rad/s limit
  (HA-179's blast radius has the measurement). **Contained:** one negate in
  `imuYawRateToCanonical` / adapter source choice.

- [ ] **HA-05 — any post-calibration tare desyncs bootHeading; the additive offset is applied
  exactly once.**
  *Claim:* `tare()`/`set_rotation()`/`set_heading()`-family calls after calibration re-zero the
  sensor independently of shulib's bootHeading offset (silently shifting canonical heading), so
  the adapter must never call them; bootHeading's ONE owner is the robot's canonical start pose.
  *Source:* `include/shulib/hal/imu_conversion.hpp:20–26`.
  *Confidence:* reasoned — follows from PROS's documented tare semantics; the *discipline* is a
  contract on R1 code that does not exist yet.
  *Settle (R1 review + R3):* adapter review (no tare-family calls anywhere); bench: confirm a
  deliberate mid-run `tare_rotation()` on a scratch program shifts the raw stream (documenting the
  hazard is real), and that our adapter path never does.
  *Blast radius if wrong:* a silent constant heading offset from the tare instant onward — eats
  the < 1° budget invisibly, unexplainable from telemetry. **Contained** only by discipline plus
  R3's bench check; this is why it is registered rather than assumed.
  *Amended (COMP-a, 2026-09-26) — who owns the START heading.* The claim above stands (never tare
  after calibration). What changed is the owner of the start heading: it is the CHOSEN AUTON's
  declared start pose — `StartPose::field(x, y, heading)`, or `StartPose::relative()` (the robot's
  own placement is heading 0) — applied at the instant the auton starts by
  `competition::runAuton`, through `Localizer::reseatHeading`: the published heading is
  `imu.heading() + bias`, and the seed sets the bias. The IMU's boot heading stays 0
  (`hal/pros/device_provider.hpp`), the sensor is never re-zeroed, and the IMU remains the only
  source of heading CHANGE. Two reasons it cannot be the boot heading: the auton is chosen on the
  brain screen AFTER the robot boots (the boot cannot know which start pose will run), and the
  seed also discards whatever the IMU drifted while the robot waited for the match. The seed is
  refused while the IMU is still calibrating (the estimate must be live first), and it re-baselines
  the estimator's next tick exactly as `setPose` does. `Routine::startAt` (frozen, F10) still seeds
  position only. *Settle (COMP-b's practice match):* place the robot at a known field heading,
  wait a minute on the selector, run the acceptance auton; it must drive along its PLACED heading,
  not the heading the IMU read at boot.

- [ ] **HA-06 — with no firmware offset configured, `get_position()` reports the SENSOR, not the
  robot center.**
  *Claim:* constructing `pros::Gps` port-only (never `set_offset()`/`initialize_full()`) leaves
  `get_offset() == (0,0)` and `get_position()` = sensor pose, so shulib's own lever-arm removal
  (`gpsRemoveLeverArm`) is applied exactly once.
  *Source:* `include/shulib/hal/gps_conversion.hpp:27–34` (binding contract).
  *Confidence:* reasoned — from PROS's documented offset semantics; untested against firmware.
  *Settle (R1 review + R3):* boot-check `get_offset() == (0,0)` in the adapter; bench: compare a
  raw fix against tape-measured sensor position.
  *Blast radius if wrong:* the lever arm subtracted twice → a constant bias that *rotates with
  heading* (inches, direction-dependent — nastier than a fixed offset). **Contained:** adapter
  construction path + the boot check that exists to catch exactly this.

- [ ] **HA-07 — `pros::Gps::get_error()` returns METERS.**
  *Claim:* the device's self-reported rms error is in meters and must scale ×39.37 to inches at
  the HAL edge.
  *Source:* `include/shulib/hal/gps_conversion.hpp` — `gpsRmsErrorToCanonical()`.
  **Changed at E2:** from A4 until E2 this obligation existed only as PROSE in that header, with
  no function performing it and no test pinning it — an instruction addressed to a future adapter
  author, guarding the most dangerous silent factor in the GPS path. E2 gave it a function and an
  independent test. The test asserts against the DEFINITION of the inch (0.0254 m ≡ 1 inch)
  rather than against `kMetersToInches`, because the pre-E2 conversion tests imported that
  constant from the header under test — so a wrong constant would have satisfied both sides of
  the comparison. That hole is closed; the ASSUMPTION (that PROS really returns metres) is not,
  and still needs R3.
  *Confidence:* reasoned — PROS documents meters; unverified against a live device.
  *Settle (R3):* read `get_error()` on-strip; a healthy fix should report ~0.01–0.05 (meters), not
  ~1–2 (inches already).
  *Blast radius if wrong (scale skipped or double-applied):* E2's measurement covariance R is ~39×
  off. Too small → good fixes gated out, GPS silently dead (quality decay — honest but wasteful);
  too large → lies accepted too readily. **Contained:** one multiply in the R1 adapter.

- [ ] **HA-08 — failed/disconnected reads return the documented PROS sentinels, and screening
  them at the edge yields stale-finite values in the core.**
  *Claim:* a dead/absent device read returns `PROS_ERR`/`PROS_ERR_F` (= +∞ for floats) rather
  than garbage or a hang; R1 adapters screen these so the core only ever sees stale-but-finite
  values (the F4 finiteness contract) — for GPS specifically, screening to `hasFix() == false`
  BEFORE conversion.
  *Source:* `include/shulib/hal/gps_conversion.hpp:35–38`; breach modeled on purpose in
  `include/shulib/sim/hostile/encoder_hostility.hpp` (sentinel window).
  *Confidence:* reasoned — sentinels are PROS-documented; the full unplug matrix is unverified.
  *Settle (R3):* the unplug session — disconnect each sensor live (IMU, GPS, Rotation, motor) and
  log the raw returns; confirm every adapter screens to the F4 contract.
  *Blast radius if wrong:* a leaked +∞ reaches the estimators. **Contained twice, and proven at
  A3:** the conversion functions throw on sentinels (fail-loud backstop), and `PilonsOdometry`'s
  last-resort finite guard freezes position rather than integrating — the A3 breach attack
  measured ~1.05″ of bounded damage and zero NaNs, with `HealthMonitor` raising `ODO_STUCK`.
  (`math::Angle` cannot even carry a NaN, by construction.) **Amended 2026-09-29 (DIAG1 Part B):**
  a drive side that is a `hal::MotorGroup` now also leaves a member whose reading is not finite
  out of that read — its median, its mean current, its max temperature — like an absent member
  (the side reads NaN only when no member is finite; the membership step that costs is HA-165),
  and the drivetrain calibrator no longer makes a candidate whose push reading is not finite into
  a drive port (it names it `NO READING` instead). Host test plant only; nothing has run on a
  robot, and through `ProsMotor`'s screen neither is expected to fire.

- [ ] **HA-09 — `northHeadingDeg = 90°`: VEX-North points toward canonical +Y (away from red) on
  our field setup.**
  *Claim:* the default rotation parameter matches how our field/alliance is physically laid out
  (the alternative canonical value is 270°).
  *Source:* `include/shulib/hal/gps_conversion.hpp:22–26`.
  *Confidence:* reasoned — a setup convention with exactly two sane values, owned by the same
  authority as the robot's start pose; unverifiable until a strip is hung.
  *Settle (R3):* on-field: a fix at a known corner must land at that corner's canonical
  coordinates, not rotated.
  *Blast radius if wrong:* every fix rotated 180° (or 90°) — instantly obvious on the R3 field
  check, gated out wholesale before that. **Contained:** one config value with one owner.

- [ ] **HA-10 — the GPS lever arm equals the configured value.**
  *Claim:* the sensor's body-frame mounting position (forward/left inches) used by
  `gpsRemoveLeverArm` matches the physical mount.
  *Source:* `include/shulib/hal/gps_conversion.hpp:28–34` (ownership contract; the value itself
  cannot exist yet — no robot).
  *Confidence:* **invented** until a robot exists to measure.
  *Settle (R3, re-checked at E5):* tape-measure the mount; verify with a spin test (a wrong lever
  arm makes stationary-spin GPS fixes trace a circle of radius = the error).
  *Blast radius if wrong:* heading-dependent position bias in every fix, worst during rotation —
  degrades E2's gains from GPS exactly when correction is most needed. **Contained:** one config
  constant; E5's calibration routine re-measures it after any rebuild.

- [ ] **HA-11 — Rotation-sensor binding: cumulative `get_position()`, centidegree int32, no wrap
  within a match.**
  *Claim:* `pros::Rotation::get_position()` is cumulative (never wraps at 360°), and its int32
  centidegree range (~6×10⁴ revolutions ≈ miles of travel) is unreachable in a match.
  *Source:* `include/shulib/localization/tracking_wheel.hpp:12–16`.
  *Confidence:* reasoned — documented behavior + arithmetic; unverified on-device.
  *Settle (R1 review + R3):* adapter review; bench: roll a tracking wheel through > 5 revolutions
  both directions, confirm monotone cumulative readings through reversals.
  *Blast radius if wrong (mis-bound to a wrapping angle):* travel deltas spike by a full
  revolution at each wrap → position jumps. **Contained:** adapter-only; `TrackingWheel` is
  already written against the cumulative contract.

- [ ] **HA-12 — tracking-wheel offsets and direction signs match the configured geometry.**
  *Claim:* the physical mounts' perpendicular offsets (forward wheel: +LEFT coordinate; lateral
  wheel: +FORWARD coordinate) and rolling-direction signs equal the robot config. The harness
  defaults (−3.0″, −4.5″) are **stand-ins for a robot that has not been designed**.
  *Source:* `include/shulib/sim/scenario.hpp:99–108`; roles/axes contract in
  `include/shulib/localization/tracking_wheel.hpp:17–24`.
  *Confidence:* **invented** — there is no robot, not even a CAD-final mount.
  *Settle (R3):* tape-measure both offsets; then the in-place spin test — a pure rotation (both
  directions) must yield ~zero center travel (the classic odom-offset bug detector, already pinned
  host-side in the pure-rotation tests); then the push test (shove a measured distance, odometry
  must agree).
  *Blast radius if wrong:* phantom translation of Δθ·(offset error) **every tick of every turn** —
  the same mechanism A3's boot-poison attack exploited (10.8″ from garbage swings), at
  smaller-but-permanent scale; dead-reckoning cannot heal it. **Contained:** two config constants;
  every line of offset-correction *math* is already host-pinned, so R3 measures values, not logic.

- [ ] **HA-13 — tracking-wheel effective diameter = nominal (2.0″ in the harness).**
  *Claim:* the wheel's effective rolling diameter equals its nominal spec within ~1%.
  *Source:* `include/shulib/sim/scenario.hpp:102`; `tracking_wheel.hpp:15–16`.
  *Confidence:* **invented** — wear, compression and manufacturing tolerance are real and unknown.
  *Settle (R3, re-run at E5):* roll-calibration — push the robot a tape-measured distance (≥ 100″
  for 1% resolution); scale = measured/read. A2 proved the harness *detects* exactly this class:
  a deliberate 2% diameter lie read 51.0″ against a true 50.0″.
  *Blast radius if wrong:* a uniform scale error on ALL travel (1% diameter = 1% of every
  distance) — bounded, corrector-healable, and the single most common odometry defect in practice.
  **Contained:** one calibration constant, persisted by E5.

- [ ] **HA-14 — drive wheels are 3.25″ and wheel↔motor-shaft gearing is 1:1.**
  *Claim:* the plant's drive-encoder synthesis (shaft ← ∫spin/r, no external ratio term) matches
  the eventual drivetrain: 3.25″ wheels driven 1:1 from the cartridge output.
  *Source:* `include/shulib/sim/drive_plant.hpp:149–151` (re-pointed at R5a Part A, 2026-09-29: the
  line numbers had drifted since A4; the HA-13/HA-14 tag is the anchor).
  *Confidence:* **invented** — the drivetrain is not designed.
  *Settle (R3):* count teeth / read the build; one wheel revolution by hand must read one motor
  revolution × ratio on the encoder.
  *Blast radius if wrong:* drive-encoder-derived speeds/distances scale wrong. **Limited by
  design at M2:** odometry reads *tracking* wheels, not drive encoders, so pose is untouched;
  affects R5 sysid bookkeeping and any future drive-encoder fallback odometry. Config constants.
  *Amended 2026-09-14 (R3b Parts 1–3):* that containment no longer holds for ROBOT TWO, whose
  odometry IS the drive encoders (`localization/drive_encoder_odometry.hpp`). There the scale
  comes from `hal::DriveGeometry`, instantiated from the chassis table's measured values
  (HA-124/125/126), never from this stand-in; the plant still bakes 1:1 (the team lead has
  reported no other ratio), so a host test and the V5 model the same gearing only while that
  stays true.

- [ ] **HA-15 — drive motors carry the GREEN cartridge: 900 ticks/rev at the output shaft.**
  *Claim:* our drive uses green (200 RPM) cartridges — red is 1800, blue 300.
  *Source:* `include/shulib/sim/hostile/encoder_hostility.hpp:63` (`driveTicksPerRev`).
  *Confidence:* **invented** — the cartridge choice belongs to a build team that has not made it.
  *Settle (R3):* read the cartridge color; spin one output revolution, confirm the tick count.
  *Blast radius if wrong:* same family as HA-14 (scale on drive-encoder quantities; M2 pose
  untouched). Also invalidates the ≈70 in/s free-speed figure inside HA-45's kV placeholder —
  the two move together. One constant.

- [ ] **HA-16 — the V5 Rotation Sensor resolves 36000 ticks/rev (centidegree).**
  *Claim:* tracking-encoder quantization is at the centidegree grid.
  *Source:* `include/shulib/sim/hostile/encoder_hostility.hpp:64` (`trackingTicksPerRev`).
  *Confidence:* measured elsewhere — VEX-documented spec, unconfirmed on our units.
  *Settle (R3):* one careful hand revolution against an index mark.
  *Blast radius if wrong:* tracking travel scale error (folds into HA-13's roll-calibration,
  which measures the *product* of diameter and resolution — so R3's push test catches both at
  once). One constant.

- [ ] **HA-17 — the built drivetrain matches the frozen preset's idealized geometry.**
  *Claim:* the physical X-drive has true 45° symmetric mounts at equal radii (making the
  `xDrive()` coefficient columns orthogonal and `forward()` an exact inverse); the eventual H-bot
  likewise matches its C3 kinematics when built.
  *Source:* `include/shulib/kinematics/x_drive.hpp:1–34` (geometry derivation + the new register
  note).
  *Confidence:* **invented** — the robot is not built; the preset encodes the *intended* design.
  *Settle (R3/R5):* R3's push/strafe/spin signature checks (forward at V ⇒ wheels at V/√2;
  in-place spin ⇒ all wheels equal; strafe symmetric with forward); residual asymmetry falls out
  of R5's sysid fit.
  *Settle path note (R5a Part A's review, 2026-10-04):* R5's fit became shulib Calibrate's GAINS step
  (`calibration/drive_characterizer.hpp`), which is built for a TANK's two sides only — an X-drive's
  mount asymmetry does not fall out of it. This entry needs its own measurement when an X-drive is
  built.
  *Blast radius if wrong:* small mount asymmetry → small systematic twist error, absorbed by
  closed-loop control and R5 tuning; gross mismatch → a corrected coefficient table
  (`MatrixKinematics` exists precisely so a drivetrain is *data*, and relaxing orthogonality is
  the already-planned C3 pseudo-inverse — F5-safe by design).

- [ ] **HA-18 — F5's numbers match on the V5.**
  *Claim:* the same twist produces identical wheel commands in a host test and on the robot,
  swapping only `RobotContext` (M1's open on-robot clause; F5 is host-frozen).
  *Source:* roadmap.md F5 row ("on-V5 number-match pending"); owned by R3, hardware day one.
  *Confidence:* reasoned — pure IEEE-double arithmetic with no libm in `toWheels()`; ARM
  cross-compilation of the exact code is CI-verified (this chunk), but execution on a V5 is not.
  *Settle (R3):* run the prepared number-match routine; compare printed wheel commands
  bit-for-bit against the host test's.
  *Blast radius if wrong:* would indicate toolchain-level floating-point divergence — motion
  still *works* (F5's math is host-proven and closed-loop-corrected); the number-match is M1's
  evidence bar, so a mismatch blocks the badge and triggers a toolchain investigation, not a
  redesign.

- [ ] **HA-19 — brownout kills the motors first; the CPU survives and the program keeps
  running.**
  *Claim:* at pack collapse the brain cuts motor power while the CPU rides the supercap — the run
  itself continues.
  *Source:* `include/shulib/sim/hostile/power_hostility.hpp:17–21`; `HealthMonitor`'s latched
  brownout marker assumes the monitor is still alive to latch it.
  *Confidence:* reasoned — VEX-documented behavior and community experience; unverified on our
  hardware/firmware version.
  *Settle (R3):* controlled brownout on a bench pack (or current-limited supply): confirm motors
  die, program continues, telemetry keeps flowing.
  *Blast radius if wrong:* **the largest single blast radius in this register.** The
  guaranteed-park design (F2 chunk — §14 calls it non-negotiable, plausibly the
  highest-expected-value code in the library) *presupposes a CPU that outlives the motors*. If
  the brain instead resets, the park guard cannot exist as designed and F2's scheduling approach
  needs a rethink (e.g., park earlier on a voltage trend, not at collapse). NOT contained by a
  constant — this is why it must be settled at R3, before F2's design is trusted on a field.

### R1a's additions — the beliefs about PROS the adapters are built on

Every row of the `hal/pros` unit table and every semantic in the host shim
(`test` tree, PROS stand-ins) is one of these. The shim can only prove the adapters agree
with these beliefs — it can never prove the beliefs. The R1a bench runbook walks them in
dependency order (battery scale before anything drives, signs before any closed loop).

- [x] **HA-94 — `move_voltage()` takes MILLIVOLTS in [-12000, 12000]. SETTLED.**
  *Claim:* the argument is mV; 6 V of command is `move_voltage(6000)`.
  *Source:* vendored `include/pros/motors.hpp:234` ("from -12000 to 12000 in millivolts");
  `include/shulib/hal/motor_conversion.hpp` (`motorVoltageToMillivolts`, the one ×1000).
  *Confidence:* **settled** — was: reasoned, documented in the vendored source, unverified.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* 2.0 V commanded through the real `ProsMotor`; the
  device reported 1823–1990 mV under load across eight motors. The shortfall is not an error —
  `commandedVoltage()` reports what was *applied*, and a motor under load on a sagging battery
  applies less.
  *(Settled by R3 runbook step 4.)*
  *Blast radius if wrong:* every command 1000× off — the robot either hums in place or
  saturates instantly. **Contained:** one conversion function; caught in the first minute of
  the bench.

- [x] **HA-95 — `get_position()` with `MotorUnits::degrees` reports OUTPUT-SHAFT degrees. SETTLED.**
  *Claim:* after the adapter sets degrees, position is the output shaft's cumulative degrees
  (gearset already accounted for by firmware).
  *Source:* vendored `motors.hpp:628` ("absolute position in its encoder units");
  `hal/pros/motor.hpp` (the degree→radian binding).
  *Confidence:* **settled** — was: reasoned, with *output-shaft* the weaker half.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* deg/rad = **57.296** on all eight motors (57.2958 exact).
  The output-shaft half is confirmed: the ratio carries no cartridge factor.
  *(Settled by R3 runbook step 4.)*
  *Blast radius if wrong:* odometry scaled by the cartridge ratio (6:1/18:1/36:1) — silently,
  nothing crashes. **Contained:** adapter-only.

- [x] **HA-96 — `get_actual_velocity()` returns output-shaft RPM. SETTLED.**
  *Claim:* the double is RPM at the output shaft (gearset-corrected by firmware).
  *Source:* vendored `motors.hpp:404`; `motor_conversion.hpp` (`motorRpmToCanonical`, ×2π/60).
  *Confidence:* **settled** — was: reasoned.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* rpm/(rad/s) = **9.549** (9.5493 exact).
  *(Settled by R3 runbook step 4.)*
  *Blast radius if wrong:* feedforward and the stall cross-check read speeds ~9.5× off.
  **Contained:** one conversion.

- [x] **HA-97 — `get_current_draw()` returns milliamps. SETTLED.**
  *Claim:* the int32 is mA; a stalled 11 W motor reads ~2500.
  *Source:* vendored `motors.hpp:426` ("in mA"); `motor_conversion.hpp`
  (`motorMilliampsToCanonical`).
  *Confidence:* **settled** — was: reasoned.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* mA/A = **1000.0** exactly, across eight motors.
  *(Settled by R3 runbook step 5.)*
  *Blast radius if wrong:* stall/capture detection fires always or never — every
  sensor-confirm operation lies. **Contained:** one conversion.

- [ ] **HA-98 — motor units/gearing are DEVICE state, persist across programs, and an explicit
  set + read-back holds. PARTIALLY SETTLED — the persistence half is still open.**
  *Claim:* the ctor-default `invalid` means "leave as is"; a previous program's configuration
  survives; setting degrees+gearset then reading both back proves the device accepted them.
  *Source:* vendored `motors.hpp:74-75`; `hal/pros/motor.hpp` (ctor read-back preconditions);
  the shim's adversarial rotations/red defaults model exactly this hazard.
  *Confidence:* **partially settled** — was: reasoned.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* the explicit set + read-back was accepted on all
  eight motors with no precondition firing, so that half holds. **Persistence across programs
  was NOT tested** and stays open — which is the half the adapter's whole ctor discipline
  exists for, so it is the half that matters. Do not read this row as closed.
  *Settle (R3, runbook steps 0/1/4):* boot with a correct map (read-backs pass); boot with one
  wrong port (read-back faults loudly).
  *Blast radius if wrong (adapter skips the discipline):* a motor left in rotations reads 1/360
  — odometry wrong by 360× with nothing crashing. **Contained:** ctor discipline + mutation-
  proven test (campaign M5).
  *Amended DEFECTS2 (2026-09-26) — the BRAKE MODE is device state too:* `get_brake_mode()` reads
  back whatever mode the port was last given, by any program, and the ctor deliberately commands
  none (choosing a mode for a mechanism it knows nothing about is not the adapter's call). Since
  DEFECTS2 that inherited mode would matter at the motor if anything keyed on it, because a zero
  under Brake/Hold is now sent as `brake()` (HA-142). So `ProsMotor` keys the stop on the mode THIS
  PROGRAM commanded through `setBrakeMode()` — never on the device's read-back, which only seeds
  the T7 fallback (DEFECTS1 E2) — and a motor this program never gave a mode stays on
  `move_voltage(0)` at zero (`pros_motor_adapter_test.cpp` DEFECTS2 a7, the seeded-mode trap,
  mutation-proven). The persistence of the brake mode across programs is untested, like the rest
  of this row's persistence half.

- [x] **HA-99 — battery voltage/current are mV/mA. SETTLED — and it was R1a's weakest belief.**
  *Claim:* `battery_get_voltage()` ≈ 12600 on a fresh pack (mV); `battery_get_current()` is mA.
  *Source:* vendored `misc.h:718-750` — which documents **NO UNIT AT ALL** ("the current
  voltage of the battery"); the mV/mA belief is from PROS's website documentation only.
  `hal/pros/battery.hpp` carries the warning in its header.
  *Confidence:* **settled** — was: reasoned and flagged as the only load-bearing unit in R1a
  that the vendored source does not state at all.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* raw **13039 → 13.04 V**, and raw 919 → 0.92 A.
  The website-only belief was right. *(Settled by R3 runbook step 2, before anything drove.)*
  *Blast radius if wrong:* a 1000× error silently destroys brownout compensation, which scales
  EVERY motor command. **Contained:** one adapter, but only if checked before driving — hence
  the runbook ordering.

- [x] **HA-100 — `battery_get_capacity()` returns percent 0–100. SETTLED.**
  *Claim:* the double is a percentage; the adapter's ÷100 lands `IBattery::capacity()` in [0,1].
  *Source:* vendored `misc.h:771-787` (unit again undocumented; "Battery Level" example);
  `hal/pros/battery.hpp`.
  *Confidence:* **settled** — was: reasoned.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* raw **91.0 → 0.91**.
  *(Settled by R3 runbook step 2.)*
  *Blast radius if wrong:* capacity() pegged at 1.0 by the clamp (a fraction read as percent)
  — misleading telemetry, no control impact today. **Contained.**

- [x] **HA-101 — `micros()` is microseconds since program start, monotonic, uint64. SETTLED.**
  *Claim:* `IClock::now()` = micros()×1e-6 is a monotonic seconds stream; uint64 µs cannot wrap
  in a robot's lifetime (~584 000 years).
  *Source:* vendored `rtos.h:241`; `hal/pros/clock.hpp` (which also records WHY not millis():
  1 ms quantization is 10% of the 10 ms tick — a 10% error in every PID derivative).
  *Confidence:* **settled** — was: reasoned.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* **999 784 µs** over a nominal 1000 ms delay — 0.02% low.
  Monotonicity and the uint64 width were not separately exercised.
  *(Settled by R3 runbook step 3.)*
  *Blast radius if wrong:* every dt in every controller — global, but impossible to miss on
  the bench. **Contained:** one adapter.

- [ ] **HA-102 — `Task::delay_until(prev, 10)` yields an anchored 100 Hz cadence.**
  *Claim:* wake at `*prev + delta` with `*prev` updated to the wake instant, so tick processing
  time does not stretch the period (unlike `delay(10)`, which drifts by the work done).
  *Source:* vendored `rtos.hpp:742-747`; `hal/pros/tick_pacer.hpp`.
  *Confidence:* reasoned — documented semantics; unmeasured under our stack's load (that half
  is HA-32/HA-34's).
  *Settle (R3, runbook step 12):* LoopMonitor dt stats over 60 s idle ≈ 10 ms mean.
  *Blast radius if wrong:* the loop runs slow by the tick body's cost and every profile
  integrates the error. **Contained:** one adapter. *(IMU1 Part A, 2026-10-29:)* `ProsImu`'s rate hold
  groups a tick's reads by TIME — 5 ms, half this cadence (HA-215) — so a cadence far off 100 Hz, or a
  wake 5 ms or more late, moves which reads share an answer (`hal/pros/imu.hpp`'s three limits).

- [ ] **HA-103 — controller axes are [-127, 127]; a DISCONNECTED controller reads 0, not a
  sentinel.**
  *Claim:* full deflection is ±127 (÷127 → exactly ±1), and disconnection is reported ONLY by
  `is_connected()` — the channels just read 0.
  *Source:* vendored `misc.hpp:85-86` (both halves documented); `controller_conversion.hpp`;
  `hal/pros/controller.hpp`.
  *Confidence:* reasoned.
  *Settle (R3, runbook step 10):* full stick = ±1.00 in telemetry; pull the controller
  mid-stick — the robot must stop via isConnected(), not keep driving on a stale value.
  *Blast radius if wrong:* the range half is cosmetic (clamped); the zero-on-disconnect half is
  the dangerous one — without the validity signal a dropped controller looks like centred
  sticks. **Contained:** the seam carries isConnected() precisely for this.

- [ ] **HA-104 — `get_digital()` is a level; `get_digital_new_press()` CONSUMES the press.**
  *Claim:* the new-press read clears per-device edge state, so with two consumers one silently
  loses; binding levels + per-consumer `ButtonEdge` gives every consumer the press.
  *Source:* vendored `misc.hpp:173-212`; `hal/controller.hpp` (ButtonEdge);
  `hal/pros/controller.hpp` (the ban, guard-tested).
  *Confidence:* reasoned.
  *Settle (R3, runbook step 10):* hold a button — pressed() stays true (a level).
  *Blast radius if wrong (adapter binds new_press):* the second consumer of any button misses
  every press — an auton selector and a mechanism toggle fighting invisibly. **Contained:**
  mutation-proven (campaign M10) + a textual guard.

- [ ] **HA-105 — rotation `get_velocity()` returns centidegrees per second.**
  *Claim:* same centidegree scale as position; π/18000 converts both.
  *Source:* vendored `rotation.hpp:219-242`; `rotation_conversion.hpp`.
  *Confidence:* reasoned.
  *Settle (R3, runbook step 6):* hand-roll at a counted rate; velocity matches.
  *Blast radius if wrong:* the odometry stall cross-check compares spin to travel with a wrong
  scale on one side. **Contained:** one conversion.

- [ ] **HA-106 — `gps_status_s_t.yaw` is the CW-from-North heading in degrees.**
  *Claim:* the atomic status read's yaw field is the same quantity `get_heading()` documents
  ([0,360) CW from North), so one status read gives position AND heading from one sample.
  *Source:* vendored `gps.h:53-64` (field named "yaw", convention undocumented) +
  `gps.hpp:598-608` (get_heading's convention); `hal/pros/gps.hpp` binds the status read.
  *Confidence:* reasoned — the WEAKER half is that yaw ≡ heading's convention; the wrap range
  is immaterial (the conversion wraps).
  *Settle (R3, runbook step 13, strip required):* compare status.yaw against get_heading() and
  against a physically known orientation.
  *Blast radius if wrong:* every GPS pose's heading component wrong the same way — the E2
  corrector's heading channel poisoned. **Contained:** one binding choice in one adapter.

- [ ] **HA-107 — the controller LCD's real column count: HA-57 says 19, the vendored doc
  implies 15. A CONFLICT, found at R1a, that only a screen can settle.**
  *Claim (as shipped):* 19 columns (ILineDisplay::kCols, HA-57) is right and the vendored
  `misc.hpp:322` col-parameter range "[0-14]" is a stale doc comment.
  *Source:* `hal/line_display.hpp:18-21` (HA-57); vendored `misc.hpp:322`;
  `hal/pros/line_display.hpp` (which truncates at kCols and records the conflict).
  *Confidence:* **invented** — two written sources disagree and neither is a measurement.
  *Settle (R3, runbook step 11):* display the 19-char ruler `0123456789ABCDEFGHI`; count
  what shows. 15 visible → shrink kCols (one constant; the content layer already truncates
  through the seam) and mark HA-57 settled-false.
  *Blast radius if wrong:* the last 4 characters of every status row silently never display —
  degraded, not dangerous (the same rows ride the serial log in full). **Contained:** one
  constant.

- [ ] **HA-108 — `Imu::reset(false)` starts calibration; a single boot-time call is the
  calibration itself, not a post-cal tare.**
  *Claim:* reset() begins the ~2 s calibration (HA-23), `is_calibrating()` covers it, and one
  boot-time reset does not fall under HA-05's tare ban (which is about POST-calibration
  re-zeroing under a live bootHeading).
  *Source:* vendored `imu.hpp:141-145`; `hal/pros/imu.hpp` (calibrate(), single-call
  precondition).
  *Confidence:* reasoned.
  *Settle (R3, runbook step 7):* time isReady() false→true from boot.
  *Blast radius if wrong (e.g., reset() is itself a tare-after-autocal):* a constant heading
  offset from boot — caught by step 7's protractor check. **Contained:** one call site.

- [ ] **HA-109 — `imu_gyro_s_t` is in deg/s and z is the yaw axis.**
  *Claim:* the raw gyro struct's units are deg/s and its z component is rotation about
  vertical. (The SIGN is HA-04's entry — undocumented, assumed CW-positive.)
  *Source:* vendored `imu.hpp:438-468` + `imu.h:84-87` (neither documents units);
  `hal/pros/imu.hpp` (the GyroRateZ branch).
  *Confidence:* **invented** — units asserted from community usage, not from the source.
  *Settle (R3, runbook step 8):* steady ~90°/s CW spin → |z| ≈ 90 raw and canonical ≈
  −1.57 rad/s. **Since IMU1 Part B (2026-11-03): Bench Tests button 2's STAGE 2c, worksheet Station N**
  — the same comparison as HA-04's (the gyro's z integrated over one hand turn of at least 2 s and 30°,
  beside the raw rotation's change over the same window, both raw PROS degrees): HA-109 holds if the two
  magnitudes agree within 10 %; a ratio within 10 % of one degree's radians (0.0175) is its own verdict, z in
  rad/s (HA-109 wrong on its units, the sign still judged for HA-04); anything else, near 0 among it, says z
  is not the yaw axis. Built, never run on a robot — not settled until the bench prints it.
  *Blast radius if wrong:* the alternate yaw-rate branch is garbage — but it is OFF by default
  (differentiation is the default source) and the bench measures before anyone flips it.
  **Contained by the default.**

- [ ] **HA-110 — the IMU's as-mounted pitch/roll sign conventions.**
  *Claim:* none yet, honestly: the adapter passes `get_pitch()`/`get_roll()` through unnegated
  BECAUSE the convention is unmeasured; whatever the bench records becomes the convention the
  tip detector is written against.
  *Source:* `hal/pros/imu.hpp` (pitch()/roll(), the unnegated pass-through note).
  *Confidence:* **invented** (a deliberate placeholder convention).
  *Settle (R3, runbook step 9):* nose-up 10° on a book → record pitch's sign; left side up →
  record roll's.
  *Blast radius if wrong:* tip detection reads a climb as a dive — but no tip detector ships
  yet, so today the radius is zero. Settle it before one does.

- [ ] **HA-111 — the invented X-drive's port map and motor direction signs match the robot.**
  *Claim:* FL1 BL2 BR−3 FR−4 ROTF5 ROTL6 GPS9 IMU10, green cartridges — ALL INVENTED.
  *Source:* `src/robots/xdrive_config.hpp` (`kInventedXDrive`, a `constexpr config::RobotConfig`, its
  name carrying this entry's number) since G1 Part 2c, 2026-09-26 — the SAME numbers, pinned
  against a transcription of the old constants by `test/xdrive_config_test.cpp`; until then
  `src/main.cpp` (the `k*Port` block, labelled), which is now a dispatcher and carries no port.
  *Confidence:* **invented — and MEASURED WRONG on 2026-08-13.** The real map on the old
  competition bot is: drive **LEFT 15/16/17/18, RIGHT 11/12/14** (green cartridges), **IMU on
  port 4**, and **no rotation sensors and no GPS at all**. Only "there are motors on low ports"
  survived. The xdrive build still carries the invented map, deliberately — it also assumes an
  X-drive with two tracking wheels, which this robot is not and does not have, so fixing the
  ports alone would not make it correct. Since G1 Part 2c it is built by `robot::Robot` over the
  on-robot device provider, which asks the brain's registry before opening a port: on this robot,
  and on robot two, it refuses at boot naming port 1 and powers nothing (host-proven through the
  PROS shim only — `test/xdrive_config_test.cpp`; no X-drive build has been uploaded since). The wiring belongs to whichever chunk runs the shipped
  stack on this robot.
  *Also measured:* port 13 spins FREE (18 mA vs its neighbours' 500–950 mA, and it does not move
  when its wheels are turned by hand) — a mechanical disconnect, not a software matter. Port 16
  under-reports its side-mates by ~20% at an inconsistent ratio — probable slip, undiagnosed.
  *Settle (R3, runbook steps 0 and 4):* device inventory, then per-wheel spin direction —
  inventory and side-split DONE 2026-08-13; per-wheel DIRECTION SIGNS still open (the 2 V bursts
  moved several motors too little to read a sign).
  *Blast radius if wrong:* ports: loud at boot (by design). Signs: a mirrored wheel fights the
  other three — caught in the first open-loop spin, BEFORE any closed loop (runbook order).
  **Contained:** constants + boot checks.

- [ ] **HA-112 — the teleop mapping AND the feel layer's defaults are drivable.**
  *Claim:* (a) the axis signs — left-stick-up = +X forward, left-stick-left = +Y left,
  right-stick-right = CW (−ω); (b) a **0.05 deadband** kills centred-stick creep without
  killing fine control; and, since T2 built the feel layer, (c) **the shipped
  `DriverProfile` is drivable as it stands** — deadband 0.05 applied as a CONTINUOUS
  rescale rather than a hard cut, **expo 0** (no response curve at all), **slew OFF**
  (no rate limit, so no added lag), mode **ArcadeBody**, and a field-centric fallback that
  **latches** until the driver re-zeroes rather than flickering between frames.
  *Source:* `include/shulib/teleop/input_shaping.hpp` (`kDefaultDeadband`, `kDefaultExpo`,
  `DriverProfile`), `slew_limiter.hpp` (`SlewConfig`), `drive_modes.hpp` (`DriveMode`),
  `field_centric.hpp` (the latch), `driver_control.hpp` (`DriveShaper`, the order); the
  original signs and the hard-cut deadband remain in `teleop/stick_mapping.hpp`, whose
  `mapSticks` the bench tester's DRIVE station still calls (the old "shulib Drive" did too, until T4
  Part 3 replaced it). Robot two's driver file runs the feel layer instead
  (`src/team/robot_two_driver.hpp`, through `teleop::runDriverControl`, `teleop/teleop_loop.hpp`),
  with this default profile, and "shulib Drive" runs that file since T4 Part 3
  (`src/programs/drive_program.cpp`; "shulib Teleop" ran the feel layer from G1 Part 2b until T4
  Part 3 retired it); since G1 Part
  2c the invented X-drive does too (`src/programs/xdrive_program.cpp` — it replaced `src/main.cpp`'s R1a
  loop, the last caller of `mapSticksToChassisSpeeds`), and it refuses at boot on every robot that
  exists, so no driver has felt it there either.
  *Confidence:* **invented — every number of it.** Widened at T2 from "the mapping" to "the
  feel layer's defaults"; no new register number was minted, because nothing new was
  measured. **No claim about how any of it feels is made anywhere**, and none can be: the
  defaults were chosen to reproduce the shipped mapping as closely as a continuous deadband
  allows (identical at centre and at both rails; inside that range the rescale asks for at
  most 0.05 less), which is a compatibility argument, not a feel argument. The feel layer has
  never been driven by a person: a previous "shulib Teleop" drove robot two under a driver on
  2026-09-25 (eight starts, open-loop), but through the OLD raw mapping; the feel layer's first
  driver is the new "shulib Drive"'s first run (worksheet Station F), which has not happened.
  *Settle (T2/R3, runbook step 10):* a human drives it. Signs that feel backwards get flipped
  at the mapping; the deadband, the expo and any slew rate get tuned by feel and recorded as
  a named profile rather than edited into a constant.
  *Blast radius if wrong:* annoying, visible, and shallow — exactly the kind of wrong that is
  fine to ship provisionally. **Contained:** the `teleop/` headers and one `DriverProfile`
  value; the mode functions, the curve and the rate limit are each off or identity by
  default, so a wrong default is a wrong *number*, never a wrong *shape*.

### R1b's additions — the beliefs about PROS the mechanism-sensor adapters are built on

Same rule as R1a's block: every semantic in the extended host shim (distance, optical, ADI,
usd) is one of these; the shim proves the adapters agree with the beliefs, never the beliefs.
Bench runbook steps 16–20 walk them. None of these sensors is known to be ON the available
robot — the steps say what can be done with a sensor on a loose cable at the table.

- [ ] **HA-113 — `Distance::get_distance()` returns MILLIMETERS.**
  *Claim:* the int32 is mm; a game piece 10 in away reads 254.
  *Source:* vendored `include/pros/distance.hpp:63,89` ("measured distance from the sensor in
  mm"); `include/shulib/hal/distance_conversion.hpp` (`distanceMmToCanonical`, the one ÷25.4).
  *Confidence:* reasoned — documented in the vendored source; unverified against firmware.
  *Settle (R3, runbook step 18):* an object at a tape-measured 500 mm reads ~19.7 in canonical.
  *Blast radius if wrong:* every capture/dock-confirm threshold off by 25.4× — confirm never
  fires (or always does). **Contained:** one conversion function.
  L6's wall reset also rests on this (its readings are inches through the same conversion).

- [ ] **HA-114 — the distance sensor's "no object" is the IN-BAND integer 9999, not a sentinel.**
  *Claim:* with nothing in range, `get_distance()` returns exactly 9999 (mm) — a plain value
  that converts to a plausible-looking 393.66 in — and NOT `PROS_ERR`, and not an errno state.
  *Source:* vendored `distance.hpp:71,98` ("Will return 9999 if the sensor can not detect an
  object"); `hal/pros/distance.hpp` (the T4 rule: raw 9999 → `confidence() == 0`).
  *Confidence:* reasoned — documented twice in the vendored source; the EXACTNESS of the value
  (is it always precisely 9999, never 9998 or a range?) is the unverified half.
  *Settle (R3, runbook step 18):* empty intake, raw value logged over ~30 s — confirm it is
  constant 9999; through the adapter, confirm confidence() reads 0.0 the whole time.
  *Blast radius if wrong (value differs on firmware):* the screen misses and dock-confirm gets
  a wall 33 feet away that thresholds as "no object anyway" (far), so the practical damage is
  a confidence that flickers non-zero on an empty intake — visible in step 18's log.
  **Contained:** one named constant + the adapter rule, mutation-proven (campaign M2).
  L6's wall reset also rests on this (a 9999 reads confidence 0 and is refused `no return`).

- [ ] **HA-115 — `get_confidence()` is 0–63, "only available when distance is > 200mm"; its
  value at or below 200 mm is UNKNOWN.**
  *Claim:* above 200 mm the int32 is 0–63 (63 = high); at or below 200 mm the vendored doc
  says the channel does not exist, and we have NO belief about what it returns there — the
  adapter refuses to consult it in that range (a valid close reading reports confidence 1.0:
  the returned distance IS the detection).
  *Source:* vendored `distance.hpp:133-135`; `hal/pros/distance.hpp` (the close-range rule);
  `distance_conversion.hpp` (`distanceConfidenceToCanonical`, the ÷63).
  *Confidence:* reasoned — both documented halves; the below-200 behaviour is a stated unknown,
  not a guess.
  *Settle (R3, runbook step 18):* log raw `get_confidence()` with an object walked from 400 mm
  to 50 mm — record what the channel actually does below 200 (settles the unknown either way),
  and confirm the adapter's confidence() stays 1.0 through the close range.
  *Blast radius if wrong (0–63 wrong):* thresholds shift but stay monotone. If the adapter had
  instead passed the below-200 raw through and it reads 0 there: capture-confirm REFUSES the
  grab it most needs — the exact failure the close-range rule exists to prevent.
  **Contained:** adapter-only, mutation-proven (campaign M3).
  L6's wall reset also rests on this: inside 200 mm (7.87 in) its confidence gate checks nothing, and the range,
  spread, error-bound and agreement gates carry those readings.

- [ ] **HA-116 — optical hue is 0–359.999°, saturation/brightness are 0–1.0 doubles.**
  *Claim:* the three double channels are already in shulib's canonical ranges — the adapter
  applies NO scale, only the defensive clamp.
  *Source:* vendored `optical.hpp:79-80,103-104,127-128`; `optical_conversion.hpp`.
  *Confidence:* reasoned.
  *Settle (R3, runbook step 19):* a known-red object reads hue ≈ 0–20°, a known-blue one
  ≈ 210–230°, saturation well above 0.5 under room light.
  *Blast radius if wrong:* color windows match nothing (or everything) — loud at the first
  bench read. **Contained:** one conversion header.

- [ ] **HA-117 — optical `get_proximity()` is 0–255 and LARGER means CLOSER. THE POLARITY IS
  NOT IN THE VENDORED SOURCE.**
  *Claim:* the int32 range is 0–255 (documented) and 255 is "object at the lens" (NOT
  documented — the vendored comment states only the range; larger-is-closer is what community
  usage assumes).
  *Source:* vendored `optical.hpp:151-152` (range only); `optical_conversion.hpp`
  (`opticalProximityToCanonical`); `hal/optical.hpp` ("proximity ≈ 1 means an object is
  close").
  *Confidence:* **invented** — the polarity half has no written source; flagged the way HA-99
  was, and like HA-99 it must be measured before anything thresholds on it.
  *Settle (R3, runbook step 19 — BEFORE any capture threshold uses proximity()):* raw value
  with the sensor covered vs. facing open air. Covered ≫ open confirms the polarity; the
  reverse means the adapter needs a documented 1−x flip IN THE CONVERSION with this entry
  corrected.
  *Blast radius if wrong:* proximity-based capture-confirm fires on an EMPTY intake and never
  on a full one — inverted, silently plausible in any single reading. **Contained:** one
  conversion function; caught by one covered/uncovered pair at the bench.

- [ ] **HA-118 — optical failure sentinels: PROS_ERR_F on the double channels, PROS_ERR on
  proximity.**
  *Claim:* a dead/misconfigured optical port returns INFINITY from hue/saturation/brightness
  and INT32_MAX from proximity — screenable, and IN-BAND for nothing physical.
  *Source:* vendored `optical.hpp:87-88,111-112,135-136,159-160`; `hal/pros/optical.hpp`
  (per-channel screen → hold-last-good + faultedReads()).
  *Confidence:* reasoned.
  *Settle (R3, runbook step 19 + step 14's unplug matrix):* unplug the optical mid-read —
  values hold, faultedReads climbs, nothing propagates an infinity.
  *Blast radius if wrong:* an unscreened INFINITY in hue() breaks the F4 finiteness contract
  the estimator layer trusts; unscreened proximity clamps to 1.0 = "object present" from a
  DEAD sensor. **Contained:** adapter screens, tested against the shim's sentinel model.

- [ ] **HA-119 — ADI `DigitalOut` DRIVES THE LINE AT CONSTRUCTION; `set_value` is 1/0 with
  PROS_ERR on refusal.**
  *Claim:* constructing `pros::adi::DigitalOut(port, init_state)` physically drives the line
  to `init_state` immediately (PROS defaults it to LOW — on a pneumatic, the cylinder MOVES at
  object construction); `set_value(1/0)` drives it thereafter and returns PROS_ERR if the port
  is not configured as a digital output.
  *Source:* vendored `adi.hpp:546-547,564,596` ("The initial state for the port"),
  `adi.hpp:598-610`; `hal/pros/digital_out.hpp` (initial state REQUIRED, no default — the T3
  ruling).
  *Confidence:* reasoned — the ctor parameter is documented; "drives immediately at
  construction" (vs. at the first scheduler tick) is the unverified half, and it is exactly
  the half that decides whether a mismatched boot state physically moves a cylinder.
  *Settle (R3, runbook step 20 — AIR DISCONNECTED first):* construct with each initial state
  and watch the solenoid LED / listen for the click; then with air, confirm a matched
  declared-safe construction produces NO motion at boot.
  *Blast radius if wrong:* a clamp that flings its game piece (or opens on the goal) at every
  boot — physical, repeated, and blamed on "flaky pneumatics". **Contained:** the required
  ctor argument + the PneumaticMechanism agreement test.

- [ ] **HA-120 — ADI addressing: 1–8 ≡ 'a'–'h' ≡ 'A'–'H' on the brain; {smart, adi} pairs on
  an expander. Whether OUR robot HAS an expander is UNKNOWN.**
  *Claim:* the three spellings name the same 8 built-in ports; the pair form addresses an
  expander's bank identically; and — separately — the ADI expander R1a's session reported is
  UNVERIFIED (it was read from registry index 21, outside the documented 0–20 range, and never
  re-checked).
  *Source:* vendored `adi.hpp:59,62,91-93`; `hal/pros/digital_out.hpp` / `digital_in.hpp`
  (T6: one class, two ctors). The expander question: the R1a bench session record.
  *Confidence:* reasoned (addressing); the expander's existence is an open QUESTION, not a
  belief — recorded here so it cannot silently become one.
  *Settle (R3, runbook step 17):* the brain's Devices screen + a corrected 0–20 registry scan.
  A switch on port 'a' vs 1 vs 'A' reads identically (addressing); the expander question gets
  a yes/no with a photo.
  *Blast radius if wrong:* a mechanism wired to a bank that does not exist — loud at the first
  actuation test, not on the field. **Contained:** construction-time facts.

- [ ] **HA-121 — ADI `DigitalIn::get_value()` is a plain level (PROS_ERR on refusal);
  `get_new_press()` CONSUMES the press.**
  *Claim:* `get_value()` returns 1/0 for the line's level and PROS_ERR when the port is not a
  digital input — and PROS_ERR ≠ 0, so an unscreened read maps a DEAD port to "pressed";
  `get_new_press()` clears per-line edge state on read, so with two consumers one silently
  starves (the ADI sibling of HA-104).
  *Source:* vendored `adi.hpp:697-712,732-757`; `hal/pros/digital_in.hpp` (binds get_value
  ONLY — guard-pinned; PROS_ERR screened to last-good).
  *Confidence:* reasoned — the consume semantics are documented; the exact refusal value of
  get_value (PROS_ERR vs 0) is the weaker half.
  *Settle (R3, runbook step 20):* hold a switch — state() stays true (a level); unplug the
  wire mid-run and log the RAW get_value — settles the refusal value either way.
  *Blast radius if wrong (refusal is 0, not PROS_ERR):* the screen never fires and a dead port
  reads "released" instead of holding — safer than the pressed direction the screen exists
  for, but the faultedReads() observability goes blind. **Contained:** adapter-only,
  mutation-proven (campaign M6, M9).

- [ ] **HA-122 — SD card: `usd_is_installed()` returns 1/0; `fopen` NEEDS the `/usd/` prefix
  while `usd_list_files` FORBIDS it; `fflush` is the strongest persist available.**
  *Claim:* three beliefs about one subsystem: (1) `pros::usd::is_installed()` is a reliable
  1/0 card probe; (2) newlib file IO on the brain reaches the card only through paths
  prefixed `/usd/` — while `usd_list_files()` documents "DO NOT PREPEND YOUR PATHS WITH
  /usd/" (two conventions in ONE API, the registry-indexing shape R1a met); (3) there is no
  fsync in PROS's exposed surface, so a flushed write is only as durable as FatFS makes it.
  *Source:* vendored `misc.h:790-844` / `misc.hpp:555-612` (the is_installed contract and the
  list_files warning are vendored text; the fopen-needs-/usd/ half is PROS's file-IO
  documentation, NOT in the vendored headers — the weaker half, flagged);
  `hal/pros/block_sink.hpp` (owns the prefix in one place; bare-name precondition).
  *Confidence:* reasoned, flagged — belief (2)'s fopen half and belief (3) are not in the
  vendored source.
  *Settle (R3, runbook step 16):* boot with a card, write a blackbox, pull the card and read
  the file on a laptop (settles the prefix and the write path); boot WITHOUT a card and
  confirm the robot drives with isOpen()==false reported once (settles the probe); yank the
  card after a flushed write and count surviving bytes (bounds belief 3).
  *Blast radius if wrong:* a blackbox that never exists while everything reports healthy —
  the exact silent failure E1's bool-returning seam was built to surface. **Contained:** one
  adapter; the no-card path is mutation-proven (campaign M11).
  *Cross-references (SAFE-SAVE Part A, 2026-09-29; no status change):* belief (3) stays true — no
  fsync is EXPOSED — but PROS's own `usd_write_r` calls `vexFileSync` after every write it passes on,
  ignoring what it returns (HA-166 (5), read from the vendored `libpros.a`). And the calibration record's
  safe save reads belief (1)'s `usd_is_installed()` again at the END of a save, while the program runs —
  which was confirmed only at boot, never for a card pulled mid-run. If a pulled card still read 1, the
  save's report would fall back on its two other checks (nothing on the card could be opened, or a file
  did not read back the same way twice) and say SAVE UNCONFIRMED, so no false claim rests on it.


### VIS1-A1's additions — the AI Vision camera as a device (built 2026-10-18–22)

The camera chunk's first part reads the V5 AI Vision sensor a FRAME at a time through the library's own
adapter (`hal/pros/ai_vision.hpp`), checks the camera's AI model BY NAME, writes the colours and modes it
needs every boot, turns a box into bearings and ranges that carry their error (`vision/camera_geometry.hpp`),
and carries the season's Toggle, Pin and Cup data (`vision/override_elements.hpp`). Twelve beliefs are new.
**None is measured: no AI Vision sensor has been read by this code**, and no AI Vision evidence exists yet —
Bench Tests station 12 was uploaded and never run. Each is settled at the bench — first by **Step 0**, the
steps that need no new code (0.1 a tape measure and a level on the mount, and a real Toggle; 0.2 "shulib
Bench Tests" station 12 as it stands; 0.3 VEXcode's AI Vision Utility, teaching the colours; 0.4 the
camera's live picture at the wall), then by the camera's worksheet station (**Station I**, written with
VIS1's part B: 0.5–0.8, on the library's own adapter). Where the
PROS header and the vendored `libpros.a` 4.2.2 disagree, the binary was read (`arm-none-eabi-objdump` on its
`vdml_ai_vision.c.o`) and the binary is what the adapter follows. *Three beliefs the brief's re-check (S6) found
unregistered are FOLDED into the entries that own the quantity rather than numbered on their own — the image's
age (HA-192), the optical axis against the housing (HA-194), a colour band's rows (HA-202); the coordinator may
split them.* The consumers — the Toggle tracker, the piece finder and the camera feed (VIS1-A2, built
2026-11-03) — lean on HA-197, HA-199, HA-200 and HA-203 far more than the device does; each cites them at its
own site (the Source lines below).

- [ ] **HA-192 — the camera's data is its LATEST frame, about 30 a second, with no counter and no time; the
  IMAGE is older than the read by an unmeasured age.**
  *Claim:* what PROS returns is the camera's most recent frame, refreshed about 30 times a second (VEX's
  figure); PROS gives NO frame counter and NO timestamp, so the time a read COMPLETED is the only freshness a
  program can know, and a frozen camera is indistinguishable from a still scene. The image itself is older
  than that completion by an age `L` nobody has measured. (HA-72's and HA-90's "~20 Hz" are the TAG
  pipeline's and are not this.)
  *Source:* `include/shulib/hal/vision_frames.hpp` (`VisionFrame::readAt` — the read's completion — and
  `readSeq`, the adapter's poll count, never a frame number), `include/shulib/hal/pros/ai_vision.hpp` (the
  read), `include/shulib/vision/camera_geometry.hpp` (`PoseBound` must hold AT THE IMAGE; `boundAtImage()`
  widens a moving robot's bound by speed and turn rate times `L`), `include/shulib/sim/camera_plant.hpp` (the
  host camera model's FROZEN state, re-stamped); VIS1-A2: `include/shulib/vision/toggle_tracker.hpp` (freshness
  is the read's time; a frozen camera caught only while the lens moves; the bearing paired with the pose at the
  image), `include/shulib/vision/piece_finder.hpp` (the pose must be the robot's at the IMAGE's time) and
  `include/shulib/vision/camera_feed.hpp` (the read period's natural floor, about 0.033 s, from the ~30 a second).
  *Confidence:* reasoned — the rate is VEX's knowledge-base figure seen only through search snippets (the
  page answers 403), the same source as HA-194; the absence of a counter or a time from the vendored header,
  read whole, and the binary; `L` unknown.
  *Settle (Step 0.2; Station I 0.7 after VIS1-B):* station 12's change timer with a piece moving (the
  rate); for `L`, the LIBRARY page's latency check — a taught colour painted on the brain's own screen at
  a logged `pros::micros()`, the first read that reports it, ten trials, the largest kept (an upper bound,
  the screen's own delay included).
  *Blast radius if wrong:* a consumer that pairs a frame with the pose at read completion while the robot
  moves places a piece wrong by the distance moved in `L` — on the host test plant, turning 180 deg/s with a
  Pin 36 in ahead, the fix misses the Pin from a 66-ms-old image (`test/camera_geometry_test.cpp`, R6.1).
  **Contained:** `boundAtImage()` is the one widening, and the consumers (AF1, L4) take a MEASURED `L` or use
  field positions only with the robot stopped. A camera frozen while the robot is PARKED cannot be caught by
  anything that reads it — which is why a check after a push is not the camera's.

- [ ] **HA-193 — the camera reports 320 x 240, (0, 0) at the top-left, +y DOWN, a box as its left and top
  edges and its size.**
  *Claim:* every detection's box is in image pixels, 320 wide and 240 tall, the origin at the top-left, +x
  right and +y DOWN, given as the box's left edge, top edge, width and height (the vendored header's "left
  edge (from camera's view)" and "top edge"; api.vex.com's `originX/Y`).
  *Source:* `include/shulib/hal/vision_frames.hpp` (`Detection`, `kVisionImageWidthPx`/`HeightPx`),
  `include/shulib/vision/camera_geometry.hpp` (its frames), `include/shulib/hal/pros/ai_vision.hpp` (the
  copy out of the packed record).
  *Confidence:* reasoned.
  *Settle (Step 0.2):* a Pin moved left and right, up and down — the numbers move the stated way; a box
  at the image's edge reads 0 / 319 / 239.
  *Blast radius if wrong:* a flipped axis flips every bearing or every elevation — and the host camera model
  SHARES this belief (it is the one thing its independence cannot catch), so no host test can see it.
  **Contained:** two constants and the adapter's one copy.

- [ ] **HA-194 — the camera's optics are 74 x 63 degrees, the axis square to the housing's back face.**
  *Claim:* a 74-degree by 63-degree field of view over 320 x 240 — two focal lengths, 212.3 and 195.8 px, never
  a square pixel — the principal point at the image's centre, no lens distortion worth modelling, each focal
  length within +-5 % on its own (the geometry's tolerance box takes the two apart); and the optical axis square to the housing's back face, where the team measures pitch
  and yaw (Step 0.1), within the mount's stated tolerance.
  *Source:* `include/shulib/vision/camera_geometry.hpp` (`kAiVisionOptics`, the header's axis paragraph);
  the host camera model (`sim/camera_plant.hpp`) takes the same numbers as its default true camera.
  *Confidence:* reasoned — VEX's figures, read only through search snippets (the knowledge-base article
  answers 403).
  *Settle (Step 0.2, then Station I's offsets after VIS1-B):* a Pin straight ahead on the robot's centreline at 24 and
  48 in — its box centre near column 160 and its bottom row where the measured pitch and height put the floor
  (a consistent offset is an axis not square to the housing: a yaw or pitch to type, not a tolerance to
  widen); then Pins at known offsets (straight, +-20 degrees, at 12 / 24 / 36 in) against the tape.
  *Blast radius if wrong:* every bearing and range scales with the focal lengths (one "field of view" for both
  axes moves every range by about 8 %); an axis off the housing is a pitch or yaw error no stated tolerance
  covers — a confidently wrong range. **Contained:** one constant, and the mount's typed pitch and yaw.

- [ ] **HA-195 — a camera read is not atomic; a failed object read is {127, 127, 0...}.**
  *Claim:* the count and each object are separate calls and the camera may update between them; a
  `get_object` whose port claim fails returns `{id 127, type 127, the rest 0}` (the vendored `libpros.a`
  zeroes the record and sets both bytes to 127 BEFORE the claim); what the SDK returns for an index past its
  own count is unknown.
  *Source:* `include/shulib/hal/pros/ai_vision.hpp` (a record with no valid type, or a tag, is skipped and the
  frame is a `PartialFault` — counted, never evidence); `test/pros_shim/pros/ai_vision.h` (the failure
  record).
  *Confidence:* reasoned — the API's two-call shape; the 127 record read in the binary, its SDK half not.
  *Settle (Station I 0.7, after VIS1-B):* the LIBRARY page's partial-fault count while pieces move; unplugging mid-read
  (station 12's RECORD prints any type other than 1 / 2 / 4 / 8 with its raw bytes).
  *Blast radius if wrong:* a torn read passed as evidence could lose the band that contradicts a colour.
  **Contained:** any skip makes the whole frame a `PartialFault`; if the SDK answered a VALID-looking type
  for a stale index, a torn read would pass undetected — the bench's partial-fault count is how that shows.

- [ ] **HA-196 — the mode mask sets exactly the masked bits; the modes and a colour's RGB read back as
  written.**
  *Claim:* `aivision_set_enabled_detection_types(bits, mask)` sets EXACTLY the masked mode bits from `bits` and
  keeps the rest; `get_enabled_detection_types` reads the modes back; `get_color` reads back the RGB as
  written. Whether the camera KEEPS modes and colours across programs, power cycles and re-plugs is NOT
  assumed either way — the adapter writes them at every construction, after every read the camera does
  not answer, and whenever its once-a-second check reads one back DIFFERENT (the check reads the modes and
  every colour back); a read-back the camera does not ANSWER — the mode read's `PROS_ERR`, or `get_color`'s
  failure record (the binary: id, red, green, blue 127 and both floats +Inf) — is a failed read, never a
  changed setting. ONLY that exact record: any other answer is compared, and a mismatch is a changed
  setting — an empty slot included, whatever the SDK reports for one (the binary's success path copies out
  whatever the SDK wrote into an uninitialised record; that is unread, and no belief about it is needed).
  *Source:* `include/shulib/hal/pros/ai_vision.hpp` (the set-up, the all-modes mask, the 1 Hz check),
  `include/shulib/hal/pros/device_provider.hpp` and `include/shulib/hal/device_provider.hpp` (making the
  camera writes it).
  *Confidence:* reasoned — the vendored doc's one example, AND the binary: `set_enabled_detection_types`
  passes `(bits, mask)` straight to the SDK, PROS's own `enable_` / `disable_` call it as `(t, t)` / `(0, t)`,
  and the overlay, white balance and tag family use another SDK call — so the all-modes mask cannot disturb
  them; every setter returns 1 whatever the SDK did. The SDK's own semantics are unread.
  *Settle (Station I 0.7, after VIS1-B):* the LIBRARY page's `setups` and fault count across a re-plug
  (the adapter re-writes after any failed read), and its `settingsLost` — a setting the once-a-second check
  found CHANGED: a camera re-plugged between two reads that came back without its settings, or another
  writer; station 12's AS FOUND line after another program ran; the mode line after the adapter's write
  reads exactly COLORS + OBJECTS.
  *Blast radius if wrong:* a mask that means something else leaves tags on (one of the 24 slots taken) or a
  mode off — the read-back refuses the set-up and the camera reads `NotConfigured` (the boot line: "NOT set
  up") rather than a wrongly configured camera; a `get_color` that reads back other numbers refuses every
  set-up — the camera dark, the robot still drives (VIS1's D5). **Contained:** one adapter.

- [ ] **HA-197 — the AI model is kept on the sensor and named, id by id, as VEX spells it; PROS gives no
  count.**
  *Claim:* the AI model is chosen in VEXcode's AI Vision Utility and KEPT on the sensor; a PROS program reads
  each class id's NAME with `get_class_name` — and no class COUNT, because the vendored binary returns 1 (its
  success code) for every id, -1 included, whatever the header says; the Override model names ids 0–4
  `redBluePin`, `blueYellowPin`, `redYellowPin`, `yellowPin`, `cup`; for an id the camera does not name, the
  SDK writes nothing a zeroed buffer would not show.
  *Source:* `include/shulib/hal/pros/ai_vision.hpp` (the check by name, each expected id into a zeroed buffer,
  after every set-up and at every once-a-second check; every Object withheld until it matches), `include/shulib/vision/override_elements.hpp`
  (`kOverrideClassNames`), `include/shulib/hal/device_provider.hpp` and `hal/pros/device_provider.hpp`;
  `include/shulib/vision/piece_finder.hpp` (VIS1-A2: a frame whose model check is not Matches is UNKNOWN "model
  not confirmed", never NONE — its withheld Objects are no evidence of absence).
  *Confidence:* measured elsewhere for the names (api.vex.com's table, read 2026-10-15); reasoned for "kept
  on the sensor" and the PROS read path (the binary's behaviour read, its SDK half not).
  *Settle (Step 0.2):* station 12's RECORD lines (`class '...'` per OBJECT) with one piece of each type in
  view, before and after a power cycle; its entry line `get_class_name(-1) -> 1` confirms the binary's return.
  *Blast radius if wrong:* a different spelling withholds every object — safe, but dark; the boot line names
  the first name that differed. A model not kept on the sensor withholds objects until it is chosen again.
  A name the SDK does not write reads as '' (the buffer is zeroed before each read); bytes it does write
  that are not the expected name read as Different — safe either way. **Contained:** one
  check, one table; the colours never depend on it.

- [ ] **HA-198 — the detections' order and the score's scale are unknown; nothing depends on the order, and
  the score only through the team's own floor.**
  *Claim:* the order of up to 24 detections is UNVERIFIED (VEX's page says largest first, by width in one
  place and by area in another) and the Object score's raw scale is UNKNOWN (VEXcode reports 1–100).
  NOTHING in VIS1 depends on the order, or on whether a count above 24 can be reported: a read reporting 24
  or more is "possibly full" and never evidence. The SCALE is used in one place only: the piece finder's
  score floor (`PieceFinderConfig::minScore`, VIS1-A2) is a TEAM number on that raw scale, chosen at Step
  0.2 from the scores the camera actually reports (0 takes every score) — the library assumes no scale.
  *Source:* `include/shulib/hal/vision_frames.hpp` (`VisionFrame::truncated`, the detections' order,
  `Detection::score`), `include/shulib/hal/pros/ai_vision.hpp` (24 or more reads 0..23, truncated);
  `include/shulib/vision/piece_finder.hpp` (VIS1-A2: `minScore` on the raw scale; a truncated read is
  UNKNOWN "camera full", never NONE).
  *Confidence:* reasoned — the independence from the code; the order and the scale themselves unknown.
  *Settle (Step 0.2):* RECORD with several pieces in view — the order against their sizes, the scores,
  and how often a crowded field reads 24.
  *Blast radius if wrong:* by design none for the order or the cap; a field that reads 24 often costs answers
  (the consumers say "camera full"), so the team keeps the working set small. The score floor is the team's
  number from the bench.

- [ ] **HA-199 — the taught colours separate the Toggle's faces under event light; the lookalikes fail a
  gate.**
  *Claim:* the three colour descriptors separate the Toggle's red, blue and yellow faces under event light,
  and the lookalikes — Pins, Goals, licence plates, LEDs, clothing — either fire no band at Toggle height or
  fail one of the tracker's gates, EXCEPT those VIS1's tests show pass every gate. With the position check
  OFF: 70 of 540 red bands near a wall (65 on the brief's prototypes, before VIS1-A1's exact intervals), a
  red LED strip on the wall's top seen from 20 in or CLOSER, and a red strip about a Toggle's length on a
  Toggle's face line seen from about 22 in or FARTHER; a legal 4.5-in plate clipped at the picture's side
  never. With the position check ON and a pose the team trusts, the 540 bands and the wall-top strip are
  turned away (they are not where a Toggle's face is) — but the strip ON a face line is not: it reads RED
  from 22 to 30 in, named as that Toggle (farther, its fix is too vague for an 8-in limit). And the colours
  tell the field face from the BOTTOM face only when both are SEEN: the tracker reads an empty place above a
  band as "no field face there", so this claim includes that **the field face's descriptor fires on every
  read on which its bottom face's does** — on the host camera model, the field face's detection removed,
  the bottom face's colour read sure in about one parking in ten; a field face detected but cut short by up
  to 6 px, never.
  *Source:* `include/shulib/sim/camera_plant.hpp` (the host camera model: a quad IS its descriptor's colour,
  and a lookalike fires because the test says it does); `include/shulib/vision/toggle_tracker.hpp` (VIS1-A2:
  its gates, its header's paragraph on what it cannot tell apart, and gate 5's "what this rests on");
  `test/toggle_tracker_scenes_test.cpp` (S5b and T10b with the gate on and off; the missed field face).
  *Confidence:* invented.
  *Settle (Station I 0.5 / 0.6 after VIS1-B, then the event field, 0.8):* the lookalikes in front of the
  camera; and, at a real Toggle from a lens BELOW its face under event light (glare on the back-leaning
  face), RECORD lines over every parking — the bottom face's band never present on a read without the
  field face's.
  *Blast radius if wrong:* a lookalike read as a Toggle's colour; a missed field face lets its bottom
  face's colour — the WRONG one — read as sure. **Contained:** by the tracker's gates (A2), its optional
  position check, and AF1's contract — no push on the camera's colour alone.

- [ ] **HA-200 — the AI model's box encloses the WHOLE piece.**
  *Claim:* an Object's box spans the whole visible piece: its bottom row is where a standing Pin's base meets
  the floor (or its support) and its height the Pin's 6.5 in — what the ground range, the size range, and the
  piece finder's agreement between them and its RAISED verdict rest on.
  *Source:* `include/shulib/vision/camera_geometry.hpp` (`groundRange()` and `sizeRange()`'s documentation);
  `include/shulib/vision/piece_finder.hpp` (VIS1-A2: `sightPiece()`'s posture — the floor agreement, the
  raise excluded, RAISED — and the fixes built on the box's bottom row and its height).
  *Confidence:* invented.
  *Settle (Step 0.2):* a still Pin at 12 / 24 / 36 in — standing, lying, on a Goal — the box against the
  tape.
  *Blast radius if wrong:* a box short of the base reads the piece farther by the ground range and nearer by
  the size range, so the two disagree — the finder answers UNSURE (a refusal), or RAISED for a floor piece.
  **Contained:** two functions.

- [ ] **HA-201 — the Toggle's geometry, from the manual's drawing.**
  *Claim:* the field-facing face's lower colour boundary is 12.55–12.65 in above the floor, its apex 14.39 in
  and 0.9–1.2 in behind the lower boundary (the face LEANS BACK); the FLAT bottom face sits at 12.36 in, about
  a quarter inch below that boundary, joined to it by the rounded corner, and is a different colour; the
  face's distance from the wall's inner face is unknown. ARITHMETIC from the Game Manual v2.0's pA8 (the axle
  13.20 in up; the apex 30.2 mm above it; 51.6 mm from the apex to the flat bottom) and its glossary ("each
  face of the triangle is approximately 2.05 in (52.2 mm) wide"): the colour boundaries, at the rounded
  corners' middles, lie 30.2 mm from the axle and so 52.3 mm apart — the glossary's width — and the field
  face's lower one is 15.1 mm below the axle. (VIS1's brief first read 12.36 as the field face's lower edge,
  then took 12.61 from the glossary; VIS1-A1 read the drawing: both hold, 12.36 is the flat bottom.) v2.1's
  PDF answered 404 on 2026-10-15, -18 and -22.
  *Source:* `include/shulib/vision/override_elements.hpp` (`ToggleFace`, `kTogglePlaces`); the host camera
  model's Toggles (`sim/camera_plant.hpp`), whose bottom face is drawn at the lower boundary's height — a
  quarter-inch approximation its banner states.
  *Confidence:* reasoned.
  *Settle (Step 0.1):* a real Toggle measured with a tape — the lower boundary's and the apex's heights,
  the flat bottom's, the distance from the wall's inner face, which colour is on which face — and the newest
  manual's pA8 and glossary.
  *Blast radius if wrong:* heights off by more than the interval make "one camera explains it" refuse a real
  face (UNKNOWN), and a wider interval lets more lookalikes through; the blind band of mounts (lens
  11¾–12¾ in, level) moves with the lower boundary, so a mount chosen from it could be blind.
  **Contained:** one header.

- [ ] **HA-202 — a still object's REPORTED box edge is within about 2 px of its true edge — a model box's and
  a colour band's.**
  *Claim:* each edge of a box AS THE CAMERA REPORTS IT (whole pixels) is within ~2 px of the true
  silhouette's edge for a still object (`kBoxEdgePx`; the minimum measurable band height is twice it) — the
  read-to-read jitter, the integer rounding and any steady BIAS all inside that 2 px — for a model's object
  box AND for a colour band, whose top and bottom rows are taken as the face's apex and lower edge (glare on
  the back-leaning face, rounded corners). The bound has no margin past it: on the host test plant, edges
  jittered by 1 px then rounded kept every standing floor Pin's fix on the Pin; jittered by 2 px then
  rounded (up to ~3 px off), the fixes missed by up to 1.8 in.
  *Source:* `include/shulib/vision/camera_geometry.hpp` (`kBoxEdgePx`, the jitter term in every interval);
  `include/shulib/vision/toggle_tracker.hpp` (VIS1-A2: a band's rows taken as the face's apex and lower edge,
  and the minimum band height); `include/shulib/vision/piece_finder.hpp` (VIS1-A2: every posture and fix
  built on the box's rows and width).
  *Confidence:* invented.
  *Settle (Step 0.2; Station I 0.5 after VIS1-B):* five RECORDs of a still Pin at each distance — the spread
  AND the bias: its box's edges against the edges the measured mount and the tape predict; five of a still
  Toggle face at 12 / 24 / 36 / 48 in — its rows against the rows the measured mount predicts, and their
  spread.
  *Blast radius if wrong:* larger jitter makes every interval too narrow — a true range outside its "bound",
  a confident wrong fix; a glare-shifted band edge refuses a real face. **Contained:** one constant.

- [ ] **HA-203 — one camera read is cheap enough for the driver loop's tick.**
  *Claim:* one read — the count and then one call per detection, up to 25 device calls, and once a second
  the adapter's check (the modes, every colour and every expected class name read back: up to 28 more), and
  after a lost setting or a failed read, at most every half second, the set-up written again (each colour
  written and read back, the modes written and read back, every expected class name: up to 36 more) —
  costs little enough to make once per read period inside the driver loop's 10 ms tick without stretching
  it.
  *Source:* `include/shulib/hal/vision_frames.hpp` (`IVisionFrames::read()`'s documentation),
  `include/shulib/hal/pros/ai_vision.hpp`; `include/shulib/vision/camera_feed.hpp` (VIS1-A2: one read per
  period, each timed with the loop's own clock — `CameraFeed::stats()`'s last and slowest read).
  *Confidence:* invented.
  *Settle (Station I 0.7, after VIS1-B):* station 12's read time (microseconds, the maximum) with colours
  and objects on, and the LIBRARY page's on a read that carries the once-a-second check;
  later the driver loop's own tick monitor under AF1.
  *Blast radius if wrong:* a read that costs milliseconds stretches the tick (LOOP_OVERRUN faults, worse
  motion). **Contained:** the feed reads at most once a period, a period the consumer chooses.

### MECH-A1's additions — the mechanism core's beliefs about the sensors it names (2026-10-08)

The mechanism core's robot file declares a mechanism's sensor ROLES — a limit switch (a home switch or
a piece sensor) and a rotation sensor (an angle sensor, possibly homed by its absolute reading) — and
its first part builds their devices. Two beliefs about what those sensors READ are new; both sit
beside R1b's (HA-121, HA-11), and neither is settled by any host test. In this part nothing reads a
switch or homes a mechanism: the service that will (the core's later parts) is where each belief
starts to matter, and Bench Tests' station 13 (the core's last part) is where each is settled. *(Since
MECH-A2, 2026-10-15, the service reads a limit switch used as a PIECE sensor — HA-184 matters from
there; homing, and so HA-186 and the home-switch half of HA-184, from MECH-A3.)*

- [ ] **HA-184 — a VEX limit switch reads 1 (TRUE) when PRESSED, and an UNPLUGGED switch line reads
  RELEASED.**
  *Claim:* `ProsDigitalIn::state()` (the vendored `pros::adi::DigitalIn::get_value()`, a level —
  HA-121) returns 1 while a VEX limit switch or bumper is pressed and 0 while it is released, so the
  core's `pressedLevel = true` (every switch role's, today) is right; and a 3-wire line with NOTHING
  plugged in reads 0 — released — never pressed.
  *Source:* `include/shulib/mechanism/kinds.hpp` (`Sensor::pressedLevel`, `limitSwitch()`),
  `include/shulib/mechanism/spec.hpp` (`SensorRole::pressedLevel`), `include/shulib/hal/pros/digital_in.hpp`
  (the adapter's read-fault note); since MECH-A2 (2026-10-15) `include/shulib/mechanism/service.hpp`
  (`Slot::seen()` reads a limit switch used as a PIECE sensor against `pressedLevel`, so every `seesPiece`
  / `stopsSeeingPiece` on one leans on this belief), and since MECH-A3 (2026-10-15) its HOME switch too
  (`Slot::homeSwitchPressed()`: a position mechanism homed by switch is homed at boot when its switch
  reads pressed on a fresh read, and a homing push ends when it does); since MECH-A6 (2026-10-30) Bench Tests' station 13
  (`src/programs/bench_r3a.hpp`, `mech_station::switchText()`: a level shown as `PRESSED (if HA-184)` or `released (if
  HA-184)`, the page asking for all three readings — for a home switch and, since its review round (2026-11-03), a
  switch PIECE sensor alike); vendored `include/pros/adi.hpp` :731-757 (`get_value()`: "Gets the
  digital value (1 or 0) of a pin" — nothing about which level a pressed switch gives, or what an empty
  line reads; :705-729, which an earlier draft of the core's brief cited, is `get_new_press()`'s doc).
  *Confidence:* **unknown** — no vendored text says which level means pressed or what an unplugged
  input reads. HA-121 (that `get_value()` is a level) is its neighbour, not its answer.
  *Settle (MECH-A6, Bench Tests station 13 — built 2026-10-30, not yet run; worksheet Station P, P.7.1):*
  read a switch PRESSED, RELEASED and with its CABLE OUT; the screen asks for all three (`RECORD it pressed,
  released, and cable OUT`, beside `home sw B: reads 1 = PRESSED (if HA-184)` or a piece switch's `piece sw E: …`),
  and each RECORD logs the raw level and its `pressedLevel`. No robot declares a switch yet: the first that does settles this.
  *Blast radius if wrong:* if pressed reads 0 — every switch-homed mechanism believes it is home
  wherever it stands at boot (the rule "a switch already pressed at bind: homed, no motion" fires on a
  released switch), and every switch piece sensor reads inverted. If an unplugged line reads 1 — the
  same, for a cable that fell out. The nets (built by MECH-A3, measured on the host test plant): the
  REQUIRED jam signature stops a move into the hard stop (case F: an arm homed at 60° by a switch reading
  "pressed", then moved to "140" — truly 200 — STALLED at 1.70 s at the stop, 170°;
  `test/mechanism_position_test.cpp`), and a switch whose reads FAIL from boot no longer homes it (the
  read-fault rule — case S, the same file). What no net sees:
  an unplugged 3-wire line reads a LEVEL, not PROS_ERR (`hal/read_faults.hpp` says so), so its
  read-fault count never moves. **Contained:** one field per switch role (`pressedLevel`), set in one
  place (`limitSwitch()` and the switch-homing words) — a measured inversion is a one-line change.

- [ ] **HA-186 — at power-up a V5 rotation sensor's cumulative position starts at the shaft's absolute
  angle within one turn, and so it does after a PROGRAM restart without a power cycle.**
  *Claim:* the vendored `pros::Rotation::get_position()` (which `ProsRotation::position()` binds —
  HA-11's binding; cumulative, never wrapped) reads, at a program's start, the shaft's absolute angle
  within one turn — not 0 — so `absoluteSensor(at, sensorReadsDeg)` homing needs no motion: the
  mechanism is at `at` when the sensor reads `sensorReadsDeg`, a FIXED offset. And restarting only the
  program (the brain powered throughout, the arm moved meanwhile) gives the arm's NEW angle, not the
  old count.
  *Source:* `include/shulib/mechanism/kinds.hpp` (`absoluteSensor()`), `include/shulib/mechanism/spec.hpp`
  (`HomeMethod::Absolute`, `PositionSpec::absoluteSensorAtZeroDeg`, the refusal of a reading outside
  [0, 360), and — MECH-A3 — the refusal of absolute homing over a travel of a whole sensor turn or more);
  since MECH-A3 (2026-10-15) `include/shulib/mechanism/service.hpp` (`PositionSlot`'s absolute zero: the
  output angle is home + (reading − the stated reading), the declared mounting applied, the reading's TURN
  chosen nearest the middle of the travel — a reading within one turn is what this belief says it is, so
  the travel plus twice the farthest the mechanism can rest outside a limit at power-up must stay under
  one turn: refusal 70 checks only the travel, and chapter 18 tells the team the rest — MECH-A3's
  review, 2026-10-22);
  `include/shulib/sim/mechanism_plant.hpp`'s `RotationMount` MODELS this belief for the host tests (it
  proves the arithmetic, never the belief); since MECH-A6 (2026-10-30) Bench Tests' station 13 shows the sensor's
  raw reading and logs it (`src/programs/bench_r3a.hpp`'s `mech_station`, its RECORD line citing this entry); `include/shulib/hal/pros/rotation.hpp` :79 binds `get_position()` (it carries HA-11, not this
  ID); vendored `include/pros/rotation.hpp` :194-216 (`get_position()`: "the Rotation Sensor's current
  position in centidegrees" — nothing about its value at power-up; :244-266, which an earlier draft of
  the core's brief cited, is `get_angle()`).
  *Confidence:* reasoned — VEX describes the sensor as absolute within a turn; the vendored text says
  only "current position", and the program-restart half is not described anywhere we have.
  *Settle (MECH-A6, station 13 — built 2026-10-30, not yet run; worksheet Station P, P.7.2):* with the shaft
  at two marked angles, power-cycle the brain and read; then move the arm, restart only the PROGRAM, and read
  again — the page shows the sensor raw and as mounted, the number `absoluteSensor(at, reads)` takes (the RAW
  reading in [0, 360)), and asks for the readings (`RECORD it at 2 marks: power-cycled, restarted`). No robot declares an angle sensor yet: the first that does settles this.
  *Blast radius if wrong:* an absolute-homed mechanism's zero is wherever the count started — every
  preset and limit offset by the arm's boot angle, the very bug the core's first prototype had ("zero
  wherever it stands"). The nets: the required jam signature stops a move into the hard stop, and the
  bench page's direction check reads the sensor against the motors. **Contained:** one homing method —
  `restsAtStop`, switch and stall homing never read it; and the F4 rotation seam is LOCKED, so reading
  `get_angle()` instead would be a sibling seam, built only if this fails.

### MECH-A2's addition — the host test plant's mechanisms (2026-10-15)

The mechanism core's second part runs power and pneumatic mechanisms through one service, and its tests
drive a power mechanism into a jam, a stall and a hold-to-run on a NEW part of the host test plant: a
mechanism joint and a flywheel. The plant is the only place any of this has run; what it stands in for
is a belief.

- [ ] **HA-183 — the host test plant's mechanism models stand in for real ones closely enough that the
  core's host verdicts transfer.**
  *Claim:* `sim::ArmPlant` (one joint driven by N fake motors — an arm, a lift through its spool, a
  roller or a claw pivot) and `sim::FlywheelPlant` behave like a real V5 mechanism in every way the
  core's host tests lean on: the INVERTED feedforward relation the drive plant uses (a static band kS,
  kV volts per rad/s of the output, a first-order approach with τ = kA/kV); gravity as a LOAD in volts
  (kG·cos(θ − level) for an arm, kG for a lift; a game piece adds to it); the brake modes at 0 V
  commanded — Hold pulls back toward where the command stopped, the angle at the END of the last
  commanded step (`holdStiffness` volts per radian of sag; captured one step earlier it drove a fast
  joint back a whole step of travel the moment it stopped — the MECH-A2 review's finding), Brake damps (`brakeDamping` volts per rad/s), Coast is free — each solved implicitly in the step
  (an explicit form rang on a fast joint, MECH-A2's own finding); the current I = |V − kV·ω|·(2.5/12),
  capped at 2.5 A at a 12 V stall (HA-93's shape); and a hard stop, an obstacle or a jam reading ZERO
  speed at stall-grade current. *(MECH-S-A, 2026-11-05:)* a joint also honours motor 0's own current
  limit when one BELOW 2.5 A is set — the applied volts limited so |V − kV·ω| at the end of each step
  stays within the limit's volts (its torque, not only its reading), the Hold brake included; a load the
  limit cannot carry falls as fast as −12 V drives it (no mass to fall with). At the default it is
  bypassed, so every earlier result stands — including the reading clamp a 12 V reversal pushes past
  (HA-219 holds whether a real motor limits like this).
  *Source:* `include/shulib/sim/mechanism_plant.hpp` (its banner names every constant invented),
  `test/sim_mechanism_plant_test.cpp` (the model's equations, each pinned against a number derived by
  hand), and the core's tests that run a mechanism through it: a power mechanism
  (`test/mechanism_service_power_test.cpp`) and, since MECH-A3 (2026-10-15), every position and speed
  verdict (`test/mechanism_position_rig.hpp`, `test/mechanism_position_test.cpp`,
  `test/mechanism_speed_test.cpp`, `test/mechanism_chapter_example_test.cpp`). MECH-A3 gave the joint an
  angle sensor MOUNTED on it (absolute within one turn at power-up — HA-186's belief, modelled — either way
  round), a limit SWITCH at an angle and a chosen pressed level (HA-184's belief is the test's to choose),
  and a stop BELOW the joint (a piece wedged under the arm).
  *Confidence:* **invented** — no constant is measured; the shape is the A2 drive plant's, not a measured
  mechanism's.
  *Settle (MECH on hardware):* the first robot session that logs a real arm's and a real flywheel's step
  response — volts, position, velocity and current every 10 ms — beside the model's, and a real
  roller pushed into a jam.
  *Blast radius if wrong:* a host verdict that rests on the model's dynamics may not transfer. In this
  part: the stall-signature timings (a 0.2 s signature trips at ~0.21 s here because the plant's jam
  reads zero speed at once; a real stalled motor CREEPS — HA-93 — so a `belowMotorRpm` below the creep
  would never trip — the reason `spec.hpp` refuses a signature whose `belowMotorRpm` is 0),
  and the hold-to-run into a jam. **Since MECH-A3 every position and speed verdict rests on it**, with
  these numbers (the brief's probes re-run on the built code, to the hundredth): 143 of 144 mismatched moves
  SUCCEEDED (overshoot ≤ 7.22°, ≤ 3.59° with a plant no faster than its cartridge; the miss a plant slower
  than its own cartridge); a hold under a +0.8 V piece within 2.41°; the hold's integral worth 3.20° of
  steady error under a 25 % gravity mismatch; a blocked move STALLED in 0.65 s / 1.00 s; the soft stop (as
  MECH-A3's review rebuilt it, 2026-10-22) at most 8.32° past a limit — a fast Blue 5:1 arm twice as heavy as
  its `.gravity`, pushed down at 12 V; 2.50° with the gravity right, 3.68° on the slower Green arm twice as
  heavy; 20 of 27 flywheels at speed, the other 7 out of reach or the slowest; a jam stopped in 0.51 s. Each
  transfers only as far as a real mechanism's step response matches the model's — and the starting values
  those tests run are HA-182's. MECH-A3 TUNED NO constant of the model: the brief planned "A3 tunes", and
  its probes reproduced on A2's plant to the hundredth, so none needed it (MECH-A3's review).
  **Contained:** test-only — `shulib/sim/` is
  unreachable from the library (the CI layering guard), no program runs any of it, and every adversarial
  case sets the controller's constants and the plant's SEPARATELY (the A2 truth-integrator lesson): a
  test that gave both the same numbers would prove only the algebra.

### MECH-A3's addition — the library's own numbers for an arm, a lift and a flywheel (2026-10-15)

The mechanism core's third part runs the position (an arm, a lift) and speed (a flywheel) behaviours. Every
number a team can know it requires (limits, presets, the gravity hold, how it homes, what a jam looks like,
the at-speed band); the numbers no team can know before the robot runs — the loops' gains, the smooth
move's speed and ramp, "reached" — the library STARTS, from the cartridge and gearing the team states.
Those are a belief about real mechanisms.

- [ ] **HA-182 — the library's starting numbers hold a real arm, lift and flywheel well enough to be tuned
  from.**
  *Claim:* with no `.tuning`, a position mechanism's move and hold — kP 0.25 V per output degree, kI 0.3
  V per degree-second in the HOLD loop only (its I-term clamped to 3 V, `mechanism::kHoldIntegralLimitVolts`),
  kD 0.004 V per degree/s, kS 0, kA 0, kV 12 V over the output's free speed (the cartridge's 100 / 200 /
  600 rpm through the stated gearing), the smooth move at 75 % of that free speed reached in 0.25 s,
  "reached" within 1.5° (a lift's 0.25 in) for 0.1 s — and a flywheel's spin — kV 12 V over its free
  speed, kS 0, kP 0.05 V per rad/s, kI 0.2 V per rad acting only within 10 % of the target
  (`kSpeedIntegralWindow`) and clamped to 3 V (`kSpeedIntegralLimitVolts`) — bring a real mechanism near
  enough to its target to be tuned from, rather than oscillating, stalling or crawling. **And (since
  MECH-A4, 2026-10-22) the driver's flywheel TOGGLE gives its spin-up 5 s** to report "at speed"
  (`mechanism::kToggleAtSpeedWindowSeconds`) — long enough for a real wheel to reach its named speed, so
  the step it logs is not a false TIMED_OUT (the wheel keeps spinning whatever that verdict says; it
  bounds only the logged step).
  *Source:* `include/shulib/mechanism/kinds.hpp` (`detail::positionStart()`, `detail::speedStart()`,
  `positionStartingValues()`, `speedStartingValues()`), `include/shulib/mechanism/service.hpp` (the three
  constants, and the loops that run every number), `include/shulib/mechanism/wiring.hpp`
  (`startingValuesLine()`, the boot line that words the per-mechanism ones "PROVISIONAL (HA-182)" — every
  program that boots a robot's list prints it since MECH-A5, 2026-10-29, for each position or speed
  mechanism; robot two declares none, so no boot prints one yet; it does not carry the three library
  constants),
  `include/shulib/mechanism/spec.hpp` (its banner), `include/shulib/mechanism/driver_inputs.hpp`
  (`kToggleAtSpeedWindowSeconds`, MECH-A4).
  *Confidence:* **invented** — the only evidence is the host test plant (HA-183): 143 of 144
  deliberately mismatched moves reached their target with these numbers (the miss: a plant slower than its
  own cartridge, TIMED_OUT honestly), 20 of 27 flywheels reached speed (the rest could not, or were the
  slowest), and a hold carried a 25 % gravity mismatch within 0.001°.
  *Settle (MECH on hardware):* tune each mechanism on the robot — log a move's and a hold's volts,
  position and current beside the starting values', and record the `.tuning` each mechanism ends with.
  MECH-A4 added the driver toggle's at-speed window (above). **MECH-A6 (built 2026-10-30) added the bench
  page's own** — Bench Tests station 13's `mech_station` constants in `src/programs/bench_r3a.hpp`: the jog
  ceiling starts at **3 V** for each mechanism and R1 steps it 3 → 6 → 9 → 12 (never past a power mechanism's
  declared `maxVolts`); the direction check first ANNOUNCES what should move and drives nothing for **1 s**
  (`kAnnounceMs`, added at its review round, 2026-11-03), a motor that turned **0.035 rad** or more in that
  second's last 0.4 s abandoning it as still moving, then jogs **+ at the ceiling for 0.4 s** (`kCheckJogMs`); its **"did not move" is an encoder change under 0.035 rad** (about 2° at the
  motor, `kMinMoveRad`) and its **"drawing" a peak current of 0.40 A or more** (`kDrawAmps`) — the two
  thresholds `judgeDirection` takes from the page and never invents; a hold-to-run jog is cut at **3 s**
  (`kJogMaxMs`) and any motor above **2.4 A for 250 ms** (station 10's numbers, `kCurrentLimitA`,
  `kPersistTicks`). None was measured: the page says so on its screens, and its first run (worksheet
  Station P) is where a 3 V check that is too weak to move a loaded claw (a BACK-DRIVEN or FIGHT-OR-JAM
  finding on a free mechanism) or a "drawing" threshold a free motor reaches would show.
  *Blast radius if wrong:* a mechanism that moves too fast (overshooting toward a hard stop — which is
  why the guide tells a team to set limits short of the stops by more than the measured overshoot), too
  slowly (a move TIMED_OUT, said, holding where it stopped), or that rings or sags in a hold (a hold's
  gravity number is the TEAM's; the integral covers only a mismatch). Never silent: every step says how
  it ended, and a jam trips the team's own stall signature. **Contained — in two parts** (MECH-A3's
  review split this, 2026-10-22): the gains, the smooth move's speed and ramp and "reached" are PER
  MECHANISM — a boot line words them (the programs print it since MECH-A5) and the declaration's
  `.tuning` replaces each one by one. The hold's integral clamp (3 V) and the speed integral's window
  (10 %) and clamp (3 V) are LIBRARY constants (`service.hpp`): no `.tuning` field and not on the boot
  line, so if one is wrong the fix is an edit to the library that changes EVERY mechanism — a wider
  blast radius than the rest of this entry (whether a team should be able to tune them is a design
  question for the coordinator, not this register's). Nothing outside the mechanism core reads any of
  them.

### IMU1's additions — the IMU's 10 ms copy and the timing the rate hold rests on (2026-10-29)

`ProsImu::yawRate()` differentiated the IMU's rotation between CALLS, and the library reads the rate two or
three times a tick; IMU1 Part A made the adapter keep its answer for 5 ms (`ProsImu::kRateHoldUs`), so every
read in one tick gets one value. The defect and the fix both rest on three beliefs no robot has shown —
the IMU's 10 ms copy, its phase against the loop, and the spacing of a tick's reads. **None is measured.**
The premise that IS measured: HA-101 (`micros()` is microseconds and monotonic), so two reads in one tick are
microseconds apart. IMU1 Part B (Bench Tests button 2's STAGE 2c, worksheet Station N — built 2026-11-03,
never run on a robot) is the first measurement of all three, and of the gyro's sign and units (HA-04,
HA-109).

- [ ] **HA-213 — the V5 IMU's readings are copied into the shared buffer every 10 ms (sample-and-hold).**
  *Claim:* `get_rotation()` and `get_gyro_rate()` return what PROS last copied into the brain's shared memory
  buffer, and that copy is made every 10 ms (PROS's default data rate; shulib never calls `set_data_rate`), so
  every read between two copies returns the SAME value.
  *Source:* `include/shulib/hal/pros/imu.hpp` (the banner's ONE ANSWER PER CONTROL TICK paragraph — "A4
  register HA-213"); `test/imu_buffer_model.hpp` (the host test plant's model of it); the PROS host shim's
  "Beliefs modeled" list (`test/pros_shim/pros/imu.hpp`).
  *Confidence:* **reasoned** — the vendored `include/pros/imu.hpp` says it in words (`set_data_rate`'s doc:
  "values are copied into the shared memory buffer only at 10ms intervals"); never observed on our brains.
  *Settle:* IMU1 Part B, Bench Tests button 2's STAGE 2c (worksheet Station N): a tight-loop read of the raw
  rotation while the robot is turned by hand, the intervals between its changes — HA-213 holds if they cluster
  at 10 000 µs (the stage's 2c.1 judges the MEDIAN of at least 20 intervals inside 9 000–11 000 µs, and prints
  how many fell inside and their mean; an IMU that answered PROS_ERR on every read is "no finding", never
  "no change").
  *Blast radius if wrong — "faster"* (the buffer refreshes more often than 10 ms): before IMU1 the second read
  was a noisy short-interval derivative rather than 0, and the baseline still moved; IMU1's hold is still
  right, and HA-214's residual is smaller. **"Slower"** (e.g. 20 ms): a 10 ms tick's ONE read is 0 on
  alternate ticks even after IMU1 — the rate then needs differentiating over a longer window, or the gyro's
  own rate (a new chunk). Visible at the bench (Part B) before any motion depends on it. A team's own loop at a
  period that is not a multiple of 10 ms reads 0 on a large share of its reads (on the host test plant a 4 ms
  loop read 0 on one read in five and never exactly — IMU1's L3) — said in the changelog and `ProsImu`'s header.

- [ ] **HA-214 — the copy's phase against the tick is unknown, and a copy next to a read gives 0 / double.**
  *Claim:* the phase of HA-213's copy against the motion tick's wake is UNKNOWN — fixed per boot, or drifting
  (whether the IMU's copy and the RTOS's 1 ms tick share a clock is unknown too). When the copy lands within a
  read's jitter of the tick's ONE rate read, that read alternates 0 / about double on the affected ticks (the
  IMU1 brief's probe 3, pinned on the real adapter by IMU1's L2: 73 zero and 73 double of 300 at ±0.05 ms of
  jitter, with one reader or two); and when the tick's FIRST read MOVES between code paths — a corrector's gate
  reads first only on a fresh-fix tick; Calibrate's REST capture reads the heading before its rate where its
  settle step did not — with the copy between the two positions, the pair of ticks around each change reads 0
  and about double (IMU1's T9 (b): a fresh fix every 5th tick, the Localizer exact on 180, 0 on 60 and double
  on 60 of 300 ticks — recorded, and held by the suite to a ceiling of 60 and 60; T10 (b): one doubled REST
  sample). IMU1's hold does not cure either. Before IMU1 the same phase decided whether robot two's Localizer read 0 on EVERY tick
  (the copy between the Localizer's read and the record's).
  *Source:* `include/shulib/hal/pros/imu.hpp` (the banner's limit (2) and its NOT CURED sentence — "A4 register
  HA-214"); `ProsImu::kRateHoldUs`'s doc.
  *Confidence:* **invented** (the phase, its drift) · **reasoned** (the consequence, from HA-213's model).
  *Settle:* IMU1 Part B's STAGE 2c, the copy's offset from the wake over 100 ticks and its drift per second (a
  drift means every boot passes through the bad band) — 2c.2 prints the first, median and last offset, the
  ticks that saw no change, the offsets' spread, and the least-squares drift with its standard error (each
  offset is measured from the actual wake, so the wake's jitter — unmeasured, STAGE 5 — enters it). It reads
  FIXED only when |drift| ≤ 10 µs/s AND 3 standard errors ≤ 10 µs/s, DRIFTING only when |drift| exceeds
  both 10 µs/s and 3 standard errors, and anything else as "no verdict: the wakes jitter too much to tell in
  one second"; it judges nothing unless 2c.1 found the 10 ms copy.
  *Blast radius if wrong:* on an affected boot the Localizer's ω alternates 0 / ×2 tick to tick (the mean right,
  the noise large): TANK1's yaw observer (time constant 0.1 s, HA-179) averages most of it; `DriveBrake`'s exit
  and the calibrator's rest gate (`|ω|` under a floor) can pass on a 0 tick or fail on a ×2 tick; a corrector's
  yaw-rate gate mis-gates on ×2 ticks; a REST capture's `yawRateSigma` carries one doubled sample (HA-22).
  **Contained:** the gyro's own rate (`YawRateSource::GyroRateZ`, one constructor argument) once HA-04 and
  HA-109 are measured — a later, separate change (IMU1's brief §5); for the corrected robot's first-read move,
  the Localizer reading its rate at the top of `update()` (the same brief's §5, OUT for now: no robot is GPS-
  or tag-corrected this season).

- [ ] **HA-215 — a tick's rate reads fall within 5 ms of its first, and no reader reads between ticks.**
  *Claim:* every rate read of one control tick falls within 5 ms of the tick's first, consecutive ticks' first
  reads are 5 ms or more apart, and no reader OUTSIDE the loop's tick reads the rate — the motion task's body
  reads the rate early and fast (the Localizer's `update()`, then a record — microseconds apart on the host test
  plant's arithmetic), and a wake is rarely 5 ms or more late. So `ProsImu::kRateHoldUs` = 5 ms (half of
  `ProsTickPacer::kTickMs`) groups a tick's reads exactly.
  *Source:* `include/shulib/hal/pros/imu.hpp` (`kRateHoldUs`'s doc and the banner's three limits — "A4 register
  HA-215"); `test/imu_rate_hold_pin_test.cpp` (the hold pinned to the tick).
  *Confidence:* **reasoned** — HA-32/HA-102's anchored 100 Hz cadence (both themselves reasoned, unmeasured
  under load); the tick body's span is unmeasured on a brain.
  *Settle:* IMU1 Part B's STAGE 2c, the tick span — a stand-in, a LOWER bound (the device reads between a
  motion tick's two IMU reads plus one serial line; 2c.3 judges the LONGEST of 100 such ticks: under 2.5 ms
  is "well under" — not proof — and 5 ms or more is HA-215 wrong on that brain); Bench Tests STAGE 5 (LOOP
  RATE) for the wake jitter, which has never run.
  *Blast radius if wrong:* a tick body whose span between its first and last rate read JITTERS around 5 ms (a
  slow SD write on some ticks, a long record build) damages the Localizer's OWN first read on most ticks — the
  later read is fresh on some ticks and held on others, so the next tick's first read is held or not by chance
  (on the host test plant a later reader at a random +4..+7 ms left 55 of 300 Localizer reads exact); a span
  STEADILY over 5 ms makes the Localizer answer the previous tick's late read — one partial tick late; a stall
  over 5 ms inside a tick costs two of the Localizer's ticks, and a wake 5 ms or more late one stale tick (three
  ticks off, where a 4 ms late wake, which the hold does not touch, costs two — the brief's probe 3b); a reader
  outside the tick (team code between routine steps, a screen task) 5–10 ms after a tick's first read costs two
  Localizer ticks per read (host: 60 of 300 for one read in 10 ticks; the adapter before IMU1, 30). Every host
  figure here was first a probe of the IMU1 brief's on a prototype of the hold; IMU1's L1 and L3 pin them on the
  real adapter — all but the 30, measured on the host test plant on 2026-10-29 with the adapter before IMU1
  put back (the same 30 at all three phases, on every run that day), which no suite test can hold: that
  adapter no longer exists. **Contained:** the constant (`kRateHoldUs`, pinned to the tick by IMU1's T3 (c))
  — or a tick counter the pacer bumps (the brief's D1 (c), rejected: a loop that forgets to bump it FREEZES
  the rate).

---

### MECH-A5's addition — robot two's claw's move time (2026-10-29)

The mechanism core's fifth part moved robot two onto the core: its two claw pivots and its pneumatic claw
are DECLARATIONS now (`src/robots/mechanism_table.hpp`), run by one mechanism service in every program
that boots robot two. A pneumatic declaration must state how long its cylinder takes to move
(`.moveTime`) — the core refuses one that does not, by name, and invents no default — because with no
sensor on the claw that time is when an auton's `set()` step counts the move done. Nobody has stated or
timed robot two's claw, so the number below is a written guess: the MECH brief's N3 (a), accepted by the
team lead with the brief, 2026-10-08.

- [ ] **HA-185 — robot two's pneumatic claw moves between closed and open within 0.5 s.**
  *Claim:* from the moment its line's level changes, robot two's claw finishes moving — closed → open,
  and open → closed — within 0.5 s, with its air at match pressure, so an auton step that sets it does
  not let the next step start while the claw is still moving. With nothing sensing the claw, the step
  ends when the declared `.moveTime` has passed: `r.set(claw["open"])` SUCCEEDED, `r.set(claw["closed"])`
  — a grab, `grabs` on its position — UNCONFIRMED and counted done ("no piece sensor to confirm it"). False if either move takes
  longer than 0.5 s at any pressure the robot plays at — a low tank late in a match included.
  *Source:* `src/robots/mechanism_table.hpp` (robot two's `claw`: `.moveTime = 0.5_s,  // PROVISIONAL
  (HA-185): Station J times it`); the step's verdict is the service's (`include/shulib/mechanism/service.hpp`,
  a pneumatic's `set()`: done when the move time has passed). `test/mechanism_table_test.cpp` pins the
  0.5 s, and `test/robot_two_claws_example_test.cpp` runs auton steps on it (`'claw': SUCCEEDED -- time in
  0.5…` for the open, `'claw': UNCONFIRMED -- time in 0.5…` for the grab).
  *Confidence:* **invented** — no one has stated it and nothing has timed it; no solenoid has run on
  either robot (HA-157..HA-159).
  *Settle (worksheet Station J, J.9):* film the claw with a phone's slow-motion video, closed → open and
  back, at least three times each way, with the tank full and again after the claw has been cycled until
  the gauge falls to the lowest pressure a match reaches; count the frames from the line's change (the
  press of A) to the claw at rest. The longest move, with a margin, is the number: it, its pin and this
  entry change together, in one change.
  *Blast radius if wrong:* too SHORT — an auton drives away, or pivots, while the claw is still closing
  on a piece, which it may then drop or never grip; the step's word is the same either way (the open's
  SUCCEEDED, the grab's UNCONFIRMED), so the log does not show it. Too LONG — every claw step in an auton waits longer than it needs, and a timed auton
  loses that time. The driver's toggle does not wait on it at all. **Contained:** one number, on one
  declaration; only an auton's claw steps read it.

---

### MECH-S-A's additions — a motor's own current cap (2026-11-05)

A string lift's string snaps when it is pulled too hard, and a V5 motor's current IS that pull (its torque
through the spool). MECH-S-A lets a mechanism state `.currentCap` — the most each motor may draw — and
writes it into the motors at boot through PROS's own per-motor limit (`hal::IMotorCurrentLimit`,
`hal/motor_current_limit.hpp`; the MECH-S brief's D1 (a): the MOTOR enforces it, not the library). Five
beliefs come with it; **none is measured**, and no motor on either robot has had a limit set by shulib.

- [ ] **HA-219 — a motor enforces the limit PROS sets, by itself, within a tick.**
  *Claim:* a V5 motor given `motor_set_current_limit(L)` draws at most about L from then on, enforced by the
  motor itself within one 10 ms control tick — under any command, under Hold at 0 V, during a disable and
  with nothing in the program ticking.
  *Source:* `include/shulib/hal/motor_current_limit.hpp` (the banner's WHY THE MOTOR — "A4 register
  HA-219"); `include/shulib/hal/pros/motor.hpp` (THE CURRENT LIMIT); `include/shulib/hal/motor_conversion.hpp`
  (`motorCurrentLimitToMilliamps`); `include/shulib/sim/mechanism_plant.hpp` models it (the limiter, its own
  invented constant — HA-183).
  *Confidence:* **reasoned** from one vendored sentence ("The motor will reduce its output at 1000 mA instead of
  the default 2500 mA" — `include/pros/motors.h`); how FAST, and whether with nothing ticking, **invented**.
  *Settle:* the Tune program ("shulib Tune", TUNE1-B1, landed 2026-11-11): a capped lift pushed into its top stop, its
  current trace; and once with the program held in a blocking loop.
  *Blast radius if wrong:* a cap that is slow or soft lets a string see more than the cap for a while — the
  cap is then a softer guard than the guide says; a cap that is not enforced while the program is starved
  is no better than the software cap the brief rejected (D1 (b)). **Contained:** nothing in the library's
  verdicts depends on it — only how hard the string is pulled.

- [ ] **HA-220 — a limit outlives its program; a motor that loses power comes back at 2500 mA.**
  *Claim:* the limit lives in the DEVICE, so one set by one program may still be there when the next program
  boots; and a motor that loses power mid-run (a cable, a brown-out) comes back at its 2500 mA default.
  *Source:* `include/shulib/mechanism/service.hpp` (the header's CURRENT CAP section: every mechanism
  motor's limit READ at boot; each capped motor re-checked once a second and written again when it reads the
  default — the Warn line names HA-220); `include/shulib/hal/motor_current_limit.hpp`.
  *Confidence:* **unverified** — HA-98's open question (device settings persisting across programs), extended
  to the limit.
  *Settle:* MECH-S Step 0.4 — a cap set by one program, the program stopped from the brain, the next
  program's boot read-back; and a capped motor's cable pulled and re-seated mid-run.
  *Blast radius if wrong:* if a limit does NOT outlive its program, the boot read-back never sees a stray one
  (harmless); if a powered-down motor does NOT come back at its default, the re-check's re-write never fires
  and the "other number" Warn says so instead. **Contained:** the read-back at every boot and the re-check
  each second cover either way; nothing is written to a mechanism with no cap.

- [ ] **HA-221 — VEXos may lower each motor's limit with more than 8 motors plugged in.**
  *Claim:* with more than 8 motors plugged in, VEXos may hold each motor's limit below what was set (robot
  two runs 10); a cap below VEXos's own share behaves as stated; and writing 2500 mA over a share at every
  boot would fight VEXos — which is why a mechanism with no cap is only READ (the MECH-S brief's D12 (b)).
  *Source:* `include/shulib/mechanism/service.hpp` (the no-cap boot Info line names HA-221; the re-check's
  "something else set it" Warn); `include/shulib/hal/motor_current_limit.hpp`.
  *Confidence:* **unverified** — HA-172's open question.
  *Settle:* MECH-S Step 0.4 — robot two's boot read-back with all 10 motors plugged in, right after MECH-S-A
  lands; if every motor reads 2500 mA, the brief's D12 (a) (reset at boot) is safe and comes back to the team
  lead as a one-line follow-up.
  *Blast radius if wrong:* a share that exists reads as "a cap left by another program" on the boot line —
  the line names both causes. **Contained:** nothing is written over it.

- [ ] **HA-222 — the string's pull per amp, and the momentum no cap limits.**
  *Claim:* the string's pull is the motor's current times its torque per amp through the gearing over the
  spool's radius — about 10 lbf per amp per motor for a red cartridge on a 1.5 in spool (VEX's published stall
  torque, believed 2.1 N·m at 2.5 A — not checked in this tree); friction adds little; the rotor's and the
  carriage's MOMENTUM at a hard stop add a pull no current cap limits.
  *Source:* `include/shulib/mechanism/kinds.hpp` (`CurrentCap`'s note: choosing the cap); guide chapter 18's
  current-cap section; `include/shulib/hal/motor_current_limit.hpp` (what a limit cannot be trusted to do).
  *Confidence:* reasoned (physics) · the number **unverified**.
  *Settle:* a spring scale on the string at a held current (the Tune program's HOLD at a known cap).
  *Blast radius if wrong:* a cap chosen from the guide's 10 lbf per amp pulls harder or softer than the team
  meant. **Contained:** the number is the team's to choose; the guide gives the arithmetic, not the cap.

- [ ] **HA-223 — a stalled capped motor reads within ~0.1 A of its cap.**
  *Claim:* a motor stalled at its cap reads within about 0.1 A of the cap, so a stall signature MORE than
  ~0.2 A below the cap trips.
  *Source:* `include/shulib/mechanism/service.hpp` (`kCapSignatureMarginAmps`, 0.2 A — the boot's Warn for a
  signature not more than 0.2 A below the cap, judged in whole milliamps — the cap as sent, the signature as the first whole-mA reading that trips it — so exactly 200 mA below warns); `include/shulib/mechanism/spec.hpp` and `kinds.hpp` (the build refuses only AT or above the cap: the
  margin is not refused, because it is invented).
  *Confidence:* **invented** — the host test plant reads exactly the cap (the brief's P10: 1.15 A under 1.2 A
  trips there), which proves only the algebra.
  *Settle:* the Tune program — a capped lift held against its top stop, the reading.
  *Blast radius if wrong:* a signature the boot did not warn about may never trip — a jam then sits at the
  cap until the step's time runs out (TIMED_OUT, not STALLED). **Contained:** the Warn names the margin; the
  cap still limits the pull.
>
### STEP1-A's addition — the coast after a loosened arrival tolerance (2026-11-05)

STEP1-A gave every drive and turn step an ARRIVAL TOLERANCE (`MotionOptions::positionTolerance` /
`headingTolerance`). A leg ends when the robot is close enough AND slow enough, and on a drive leg the speed
bound is what decides — the STEP1 brief's probe P3 found the distance alone changes nothing (1.80 s at 0.5,
1, 2 and 4 in alike, on the host test plant) — so a looser tolerance loosens both, in proportion
(`motion::loosenedSettle`, the team lead's D2 (a), 2026-11-03). A leg can therefore END while the robot is
still moving at a few in/s, or still turning. What it does next is the belief below.

- [ ] **HA-216 — a leg that ends under a loosened tolerance comes to rest within that tolerance.**
  *Claim:* a leg that ends under a loosened arrival tolerance — moving at up to the scaled speed bound (4 in/s
  at 2 in, 8 in/s at 4 in; a turn up to 1.31 rad/s, about 75°/s, at 5°) — comes to rest within that tolerance
  on robot two, drives and turns alike.
  *Source:* `include/shulib/chassis/chassis.hpp` (`MotionOptions::positionTolerance`'s doc — "A4: HA-216");
  `include/shulib/motion/motion_config.hpp` (`loosenedSettle`); `test/routine_tolerance_test.cpp` (ST3: the
  host test plant's times and arrivals).
  *Confidence:* **reasoned** — on the exit tick the motion sets the drive motors to 0 V; the A2 plant stops
  dead then (its kA 0), and the motor-lag plant (kA 0.02) ENDS the leg inside the tolerance (ST3, read at the
  verb's return — where that plant then comes to rest is not measured); robot two keeps moving on its own
  momentum under whatever brake mode its drive motors hold — unmeasured.
  *Settle:* an Auton Tester run of the 40 in leg at `{.positionTolerance = 2_in}`, tape-measured; and of the 90°
  turn at `{.headingTolerance = 5_deg}`, the END heading against the IMU at rest (a Station M-style floor run).
  *Blast radius if wrong:* a robot that coasts PAST the tolerance arrives further on than the step said it
  would (a drive) or turned further (a turn) — the next step starts from there, and a tight auton's next leg
  is off by the coast. Only steps that ASK for a looser tolerance are affected; a step with none uses the
  robot's own settle numbers, bit for bit. **Contained:** the author's own number on the step (a tighter
  tolerance, or none), or the formula's proportion (`loosenedSettle`, one function).


### CURVE1-A's addition — the curve follower (2026-11-11)

CURVE1-A added `curveTo` / `reverseCurveTo`: pure pursuit on a smoothed path, a speed plan, two "lost"
guards, and an end that is TANK1's `moveTo`. Every number below was chosen on the host test plant (a weaker
side to 15 %, motor lag to 0.18 s), never on a robot; the team's Station C session (after Station M) is the
first look at each.

- [ ] **HA-246 — the curve's tuning, and the run-up rule, hold on the robot.**
  *Claim:* `MotionConfig::curve`'s defaults — look-ahead 8 in, bend grip 60 in/s², speeding up at 200 and
  slowing at 120 in/s² (the plan rising from 12 in/s), the hand-off (12 in, 5°, 0.3 in; the last resort at ⅓
  of it inside TANK1's 15° window), the end speed ×0.6 — let robot two follow a curve and settle at its end
  within the default arrival rule; robot one runs on robot two's numbers until its own short check; and guide
  05's run-up rule (a one-point curve removes a sideways miss up to 6 in from 2½ ft back along the end's line,
  up to 1 ft from 3 ft, nose within 20°) holds on the robot within ~6 in.
  *Source:* `include/shulib/motion/motion_config.hpp` (`CurveConfig`, every field "A4: HA-246");
  `include/shulib/motion/curve_path.hpp` (`kPlanStartSpeed`); `test/motion_curve_test.cpp` (CV6, CV23).
  *Confidence:* **invented** — on the host test plant the 17 drivable routes settle 12–17 per cell under every
  weak side and lag tried, and the run-up rule's 180 runs all end within 0.5 in and 1.15°; the plant has no
  wheel slip, and neither robot's motor lag is measured.
  *Settle:* Station C.1–C.3 and C.5 on robot two (C.5b, optional, the run-up's hardest corner); C.1 on robot
  one once built.
  *Blast radius if wrong:* a curve that swings wide, times out at its end, or needs more run-up than the guide
  says. **Contained:** every curve sits behind the team's `kUseCurves` flag (off until Station C passes), its
  `.timeout` bounds it, and its end is an ordinary `moveTo` judged by the same rule.
- [ ] **HA-247 — robot two does not slide in a bend at 60 in/s².**
  *Claim:* a bend taken at sqrt(bendAccel / curvature) does not make robot two's tyres slip sideways.
  *Source:* `CurveConfig::bendAccel`. *Confidence:* **reasoned** — VEX tyres commonly grip well above 0.16 g;
  never measured; the plant has no slip. *Settle:* Station C.3 (the grip raised 60 → 90 → 120 while the back
  holds the 12 in bend). *Blast radius if wrong:* the robot skids wide in the tightest bends and the
  odometry with it (HA-249). **Contained:** one number, lowered in the robot file.
- [ ] **HA-248 — the "lost" guards: never on a healthy robot, always on a dead side.**
  *Claim:* a healthy robot two never points more than 60° off its curve for 0.25 s, nor makes under 1 in of
  progress in 0.75 s outside the last 12 in; a dead side does one or the other within ~3 s.
  *Source:* `CurveConfig::lostHeading / lostTime / stuckTime`; `test/motion_curve_test.cpp` (CV5, CV6).
  *Confidence:* **invented** — host test plant: 0 trips in 153 healthy runs (worst 25.9° off the path); a dead
  side stopped within 2.5 s of dying at 0.8 s, and — dying EARLY, at 0.1–0.5 s, either side, 7 routes —
  within 2.7 s and 9.5 in of travel (CV5; a robot still creeping forward while it pivots is stopped by the
  heading guard). *Settle:* every Station C log read for a FALSE "curve lost" (the healthy
  half); the dead-side half is host-only — no cable is pulled on the robot. *Blast radius if wrong:* a false
  "curve lost" fails a good step (the route's `onFail` decides what follows); a guard that never fires leaves
  a dead side driving until the `.timeout`. **Contained:** the timeout always ends it.
- [ ] **HA-249 — drive-encoder odometry is as true on a curve as on a straight.**
  *Claim:* the scrub of a skid-steer turn does not make robot two's encoder odometry move a curve's end by
  more than ~1 in on a 60 in curve. *Source:* `curve_follow.hpp` (reads the Localizer only).
  *Confidence:* **reasoned** — never measured. *Settle:* Station C.4 (tape the end pose, measure after the S).
  *Blast radius if wrong:* the curve "arrives" where the estimate says, off the real spot. **Contained:** the
  wall reset (L6) re-seats the estimate between legs.
- [ ] **HA-250 — a curve fits the auton task's stack and tick.**
  *Claim:* a `curveTo` motion (≤ 6 KB by `static_assert`; 5,904 B measured on the host) fits the auton task's
  32 KiB stack, and its per-tick work (a forward search over 12 in of path, a few Hermite evaluations) fits
  well inside the 10 ms tick. *Source:* `curve_follow.hpp` (the `static_assert`), `curve_path.hpp`.
  *Confidence:* **reasoned** — the first evidence, the ARM `-fstack-usage` frames of a TU calling a 16-point
  `curveTo` (CURVE1-A, its development log): the verb's frame 6,528 B (it holds the 5,840 B follower), the
  follower's tick 792 B; the deepest call chain from the verb about 9.2 KB, about 28 % of the 32 KiB stack
  (a `moveTo`'s verb frame is 2,160 B). The headroom left on the brain is not measured. The path is BUILT
  once, inside the curve's first live tick, and that work grows with the path's length (a curvature scan
  every 0.25 in, and a few evaluations per speed-plan sample): on the host (-Os) about 0.1 ms for the 143 in
  snake and 0.76 ms for a 16-point, 2,092 in path (14 field widths — no real route); a Cortex-A9 with
  soft-float doubles is plausibly 10–30× slower, so a long skills curve could make that one tick late
  (bounded, once per curve; review of 2026-11-12). *Settle:* Station C's loop-monitor overrun count —
  read the period of each curve's FIRST tick as well as the steady ones. *Blast radius if wrong:* a stack overflow on the brain (no message), or late ticks.
  **Contained:** the size is pinned at compile time.


### ROB1-A's addition — robot one in the code (2026-11-11)

ROB1-A added robot one — the cascade robot — to the code before anyone in this project has seen it: its id,
its tables, a facts file whose every unmeasured number is OWED and refuses the builds that need it, and its
builds (six since its integration onto TUNE1-B1, its Tune among them). The four beliefs below are what that
code rests on about the robot itself.

- [ ] **HA-252 — robot one is a tank.**
  *Claim:* robot one drives on two sides and cannot strafe, like robot two — the builder, 2026-11-05: "both robots
  identical, 15 in".
  *Source:* `src/robots/chassis_table.hpp` (`kRobotOneChassis` — a `ChassisTable` is a tank's);
  `src/robots/calibrate_facts.hpp` (`calibrateFactsFor()` states a tank for every table); the boot
  (`src/common/boot_source.hpp` builds a tank description).
  *Confidence:* **reported** — nobody in this project has seen robot one.
  *Settle:* the facts form's item A1 (`docs/robot-facts-form.md`), at the robot.
  *Blast radius if wrong:* every robot-one program that drives is wrong for it — Calibrate would refuse or
  measure nonsense. **Contained:** the form says to stop and tell the coordinator before anything is built.

- [ ] **HA-253 — robot one has an IMU.**
  *Claim:* robot one carries a V5 inertial sensor on a smart port.
  *Source:* `include/shulib/hal/drivetrain_description.hpp` (a tank description with no IMU port is refused —
  "0 = UNSET"); `src/robots/chassis_table.hpp` (`kRobotOneChassis.imuPort = 0`, UNSET).
  *Confidence:* **unknown** — the builder named only AI Vision and rotation sensors.
  *Settle:* the facts form's item A4; Bench Tests' census on robot one.
  *Blast radius if wrong:* no robot-one program can drive the robot (the boot refuses, by name). **Contained:**
  nothing moves on a guess; the team is told at Step 0.

- [ ] **HA-254 — between two measured rows, the cascade is close to straight in its output angle.**
  *Claim:* robot one's cascade height, between two of its measured rows, follows its output angle closely
  enough that straight-line interpolation is good to within the position tolerance (0.25 in).
  *Source:* `src/robots/mechanism_table.hpp` (robot one's `cascade`, `.shape = measured(...)`);
  `src/robots/robot_one_facts.hpp` (`kCascadeRows` — a row at the bottom stop, EVERY level and the top stop, so
  each level is exact whatever the curve between).
  *Confidence:* reasoned · **unmeasured** (a string winds onto a spool in layers and stretches).
  *Settle:* the facts form's C7 (a row at every level) and a mid-row tape check at station 13.
  *Blast radius if wrong:* a constant target BETWEEN rows (`cascade.to(h)`) lands off by the curve's bend; the
  named levels do not. **Contained:** the levels are rows.

- [ ] **HA-255 — robot one's cascade motors face each other.**
  *Claim:* the cascade's two motors are mirrored, so one is `Forward` and the other `Reversed`.
  *Source:* `src/robots/robot_one_facts.hpp` (`kCascadeDirections`, OWED: `{Forward, Reversed}` its placeholder,
  typed as the builder sees them).
  *Confidence:* **guess** until the builder's look, then station 13's direction check.
  *Settle:* Bench Tests' station 13 direction check on robot one (a correction is a typed fact).
  *Blast radius if wrong:* the two motors fight — the cascade does not move, and both stall. **Contained:**
  station 13 checks the direction before any program drives it, and at station 13 — which does NOT apply the
  cascade's current cap until ROB1-C — the containment is the station's own: the 3 V ceiling and its cut-outs
  (two motors fighting for 250 ms, 2.4 A for 250 ms, the stall signature, a 3 s jog). In the programs that bind
  the mechanism service (Drive, the Auton Tester, Competition, and shulib Tune — which may change the cap live
  and puts the robot file's back on RESET and EXIT, HA-220), the cascade's current cap bounds the pull.

---

### STEP1-B's additions — a partway trigger on a real robot (2026-11-11)

STEP1-B added TRIGGERS partway through a drive (`r.atPercent(60).startMove(lift["high"], 5_s).driveTo(...)`,
`chassis/routine.hpp`): the drive's observer reads the position estimate every tick and starts the
mechanism the first tick it reaches the mark; a leg that ARRIVES just short fires a mark that is on the leg
as it arrives (the team lead's D6 (i)). Two beliefs carry what the host test plant cannot show.

- [ ] **HA-217 — a partway trigger's mechanism starts within about 1–2 ticks of its mark.**
  *Claim:* the mechanism a trigger starts begins to move within about 1–2 control ticks (10–20 ms) of the
  tick on which the position estimate first reaches the mark: the START is commanded inside the drive's
  tick (`Routine::fire`), and the mechanism service acts on it at the next pace.
  *Source:* `include/shulib/chassis/routine.hpp` (the "STEP1-B: partway triggers" block — "A4: HA-217");
  `test/routine_trigger_test.cpp` (ST9: on the host test plant the command lands on the mark's own tick).
  *Confidence:* **reasoned** — on the host test plant the service and the motor models answer on the same
  tick; robot two's motor response and its task timing are unmeasured. At 40 in/s each tick is 0.4 in of
  drive.
  *Settle:* a bench run logging the tick the mark is reached against the mechanism's first measured motion
  (its encoder), on robot one's cascade.
  *Blast radius if wrong:* the mechanism starts LATER along the drive than the mark says — a lift raised at
  "60 %" may begin at 65 %. The author's mark absorbs it once measured. **Contained:** one trigger's
  timing; nothing about the drive changes (a trigger never touches the leg).
- [ ] **HA-218 — a trigger fired as its leg arrives is as close to its mark as the estimate is to the tape.**
  *Claim:* a trigger fired as a leg ARRIVES — its mark on the leg (no further than the leg's length plus its
  arrival tolerance), the leg settled within that tolerance — puts the mechanism within the leg's tolerance
  of the author's mark, because the position ESTIMATE says the leg arrived; that is true only as far as the
  estimate is (odometry drift over the leg).
  *Source:* `include/shulib/chassis/routine.hpp` (the "STEP1-B: partway triggers" block — "A4: HA-218");
  `include/shulib/motion/leg_progress.hpp` (progress is read from the estimate); `test/routine_trigger_test.cpp`
  (ST22 and ST36: the settle fire on the host test plant).
  *Confidence:* **reasoned** — on the host test plant the estimate tracks truth to about 0.01 in over the 40 in
  leg (the STEP1 brief's probe P3); robot two's drift is the odometry entries' question.
  *Settle:* HA-216's floor run of the 40 in leg — the estimate's END against the tape.
  *Blast radius if wrong:* a mark near a leg's end fires where the ESTIMATE arrived, which may be short of or
  past where the robot really is — a lift raised a little early or late at the end of a leg. **Contained:**
  only marks near a leg's end that the leg did not cross inside a tick; a mark crossed mid-leg is the same
  estimate every motion steers by.


---

### AF1S-A1's additions — the Toggle flip for a skills run (2026-11-11)

A skills route parks the robot at a LOOK SPOT square in front of a named Toggle; `competition::checkAndFlipToggle`
looks with the front camera, flips the Toggle with the flipper only if it is not already its Quadrant's colour,
and looks again after (`include/shulib/competition/toggle_visit.hpp`). Seven beliefs come with it — four of AF1's
(the driver-control auto-flipper's brief, which held HA-228..HA-236) reused because they mean the same thing, three
new. **None is measured: no Toggle, no flipper and no camera reading of a real Toggle exist in this project.**

- [ ] **HA-231 — one stroke rolls one face, always the same way round.**
  *Claim:* one full flipper stroke rolls a Toggle exactly ONE face, the same way round every time, and the
  retract does not roll it back; on the research's reading a red-side Toggle goes Y -> B -> R and a blue-side one
  the mirror, so from yellow every skills Toggle needs TWO pushes, passing through the other colour.
  *Source:* `include/shulib/competition/toggle_visit.hpp` (rule 6: push 2 only on the THIRD colour; the time check
  budgets two pushes); `include/shulib/sim/toggle_plant.hpp` (the host Toggle's roll order is the test's choice).
  *Confidence:* **reasoned** (pA15 and the geometry, AF1's research — no Toggle has been pushed).
  *Settle:* AF1S Step 0.1 — on a real or mock Toggle at field height, mounted as a red-side and as a blue-side one.
  *Blast radius if wrong:* the RULE holds either way (it looks after every push — the brief's P3); the TIME does
  not — a Toggle needing one push costs less than budgeted, one that over-rolls is caught by the look after.

- [ ] **HA-232 — a full stroke seats the Toggle; the camera cannot check it.**
  *Claim:* after every FULL stroke (the push SUCCEEDED; a passive bar reached the push spot) the Toggle sits flat
  within `settle` and is never left stuck part-way. The camera cannot tell: a Toggle stuck part-way shows the face
  it is rolling TO, and reads that colour "sure" (VIS1-A2's banner; on this project's host camera model, a Toggle
  jammed 50 or 100 degrees into its roll read as its destination — `test/toggle_visit_scenes_test.cpp`'s T14).
  *Source:* `include/shulib/competition/toggle_visit.hpp` (`ToggleVisitRequest::settle`; FLIPPED only after a full
  stroke — OURS AFTER A SHORT STROKE otherwise; every success line ends `seating not checked`).
  *Confidence:* **invented**.
  *Settle:* AF1S Step 0.1 — twenty full strokes, each checked by eye for a Toggle stuck part-way.
  *Blast radius if wrong:* a route counts a Toggle flipped that the game counts yellow. **Contained only by the
  words:** the line says `seating not checked`; nothing in the library can see it.

- [ ] **HA-234 — the run's end safes the flipper; it does not retract it.**
  *Claim:* at the run's end a V5 motor in its declared safe brake mode does not move the flipper toward `back`, so
  a flipper out at 0:00 stays out — touching the Toggle, which then counts yellow (`<SC4>`b).
  *Source:* `include/shulib/competition/toggle_visit.hpp` (rule 4's time check: a visit that may need two pushes
  is begun only when both fit; rule 5's retract-now).
  *Confidence:* **reasoned** — the run guard safes, it does not retract.
  *Settle:* a timed run with the flipper out at its end.
  *Blast radius if wrong:* none worse — the time rules are then more careful than needed.

- [ ] **HA-235 — a sticky Toggle's push is cut before it damages the Toggle.**
  *Claim:* a push stopped by a sticky Toggle is ended by the flipper's stall signature (STALLED) or by `flipBound`
  (TIMED_OUT) before it damages the Toggle (`<R18>`a); the step then retracts on the next tick.
  *Source:* `include/shulib/competition/toggle_visit.hpp` (rule 5: `back` on ANY verdict); the flipper's own
  `.stall` signature in the robot file.
  *Confidence:* **invented**.
  *Settle:* AF1S Step 0.1 — the push run against a Toggle held by hand; the `[MECH]` line's verdict and time.
  *Blast radius if wrong:* a stalled push pushes harder or longer than the Toggle takes. **Contained:** the bound.
  *A pneumatic flipper:* no stall signature and no sensed position — its step SUCCEEDS once the valve has held the
  commanded position for its move time. So a cylinder blocked part-way by a jammed Toggle counts as a FULL stroke,
  one caught on the Toggle on the way back counts as back (the robot then drives out with it), and the step's
  `OURS AFTER A SHORT STROKE` and `FLIPPER STUCK` come only from a motor flipper (both pinned on the host test
  plant). Settle it by eye at Step 0.1: twenty strokes against a Toggle held part-way, and on the way back.

- [ ] **HA-243 — each Toggle's Quadrant colour.**
  *Claim:* in F1 (+x to the right as seen from the red station; the red wall at y = -70.2) the RED wall's and the
  RIGHT wall's Toggles are RED Quadrants', the BLUE wall's and the LEFT wall's BLUE — Figure Q-1 (Red 1 on the red
  alliance's wall, Red 2 on the wall to the red station's right; Blue 1 on the blue alliance's, Blue 2 to the red
  station's left) read with pA15 (each Toggle starts yellow into the field, its Quadrant's colour facing out).
  *Source:* `include/shulib/vision/override_elements.hpp` (`kToggleQuadrantColour`, `toggles::`);
  `include/shulib/competition/toggle_visit.hpp` ("ours" is the named Toggle's Quadrant colour).
  *Confidence:* **reasoned** — two drawings of the Game Manual v2.0, read 2026-11-10 (v2.1's change log changes
  neither).
  *Settle:* AF1S Step 0.7 — standing at the red station on the event's field: is the Toggle on the wall to the
  RIGHT the one with RED facing out?
  *Blast radius if wrong:* every visit to a side wall's Toggle flips it to the colour that scores nothing.
  **Contained:** one table of four entries.

- [ ] **HA-244 — the drive in and out lands where the route arrived; the spot check is coarse.**
  *Claim:* the step's straight drive of `lookFrom - pushFrom` along the robot's heading puts the flipper within
  its reach of the face, and back within `offBy` of the look spot, on a square robot — because the ROUTE arrived
  within its allowance. The camera's at-the-spot check (the band's WHOLE range interval inside the window)
  refuses only GROSS arrival errors: on the host camera model, a 10-in lens 16 in out is accepted from a true 14
  to 17 in and refused at 18 in and farther; a lens near the Toggle's height hardly ranges (on the host camera
  model a 12-in lens accepted a true 22–33 in at a 24-in look spot — a host-model result, not pinned by a test).
  With a FIELD start the step also compares the estimate's lens distance from the wall's INNER face with
  `lookFrom`, which takes the Toggle's face to sit at that inner face — where it sits is HA-201's, unmeasured.
  *Source:* `include/shulib/competition/toggle_visit.hpp` (`ToggleVisitRequest::{offBy, pushFrom, driveBound}`,
  `bandAtLookSpotProblem`).
  *Confidence:* **invented**.
  *Settle:* AF1S Step 0.4 (park 3 in too far and 3 in too near: NOT AT THE SPOT?; and how far the Toggle's face
  sits out from the wall's inner face), 0.5 (the push spot), 0.6 (ten drives in and out: the worst time, where
  the robot ends, where the flipper lands).
  *Blast radius if wrong:* the flipper pushes air (NO CHANGE) or lands off the face. **Contained:** the look after.

- [ ] **HA-245 — the camera names the colour from the look spot on a real field.**
  *Claim:* on a real field the camera names a Toggle's colour from the look spot as the host camera model
  predicts — about 12–24 in out and within ~3 degrees of square, for a lens 6–12 in up (only the 10-in lens's
  figures are re-run and pinned by this release's tests; the 6-, 8- and 12-in ones come from an earlier build of
  the tracker and have not been re-run) — with the wall below the
  Toggle hiding its bottom face from a low lens (the model draws no wall), the taught colours holding under event
  light, and the flipper at stow out of the picture.
  *Source:* `include/shulib/competition/toggle_visit.hpp` (`ToggleVisitRequest::lookFrom`); guide chapter 19's
  "Checking and flipping a Toggle in a skills route".
  *Confidence:* **invented** — the camera's own beliefs (HA-192..HA-203) cover the device; this is the look
  spot's geometry on our robot.
  *Settle:* AF1S Step 0.3 (the colours on the real faces) and 0.4 (the look spot, square and turned).
  *Blast radius if wrong:* every visit ends UNKNOWN BEFORE (nothing pushed) or NOT AT THE SPOT. **Contained:** a
  doubt does nothing.

### MIRROR1-A's additions — one route, two robots (2026-11-13)

Robot two runs robot one's skills route MIRRORED across the field's skills diagonal
(`include/shulib/competition/field_mirror.hpp`, `competition/override_mirror.hpp`; the Chassis's mirrored run
in `chassis/chassis.hpp`). Three beliefs come with it. **None is measured: robot two as robot one's twin does
not exist yet, and nothing here has run on a robot.**

- [ ] **HA-258 — the two red Quadrants are mirror images on the event's field.**
  *Claim:* the field is set up to Appendix A and Figure VURS1, so the red wall's and the right wall's Quadrants
  are mirror images across F1's y = -x — the goals, the Toggles, the Pins and the tape — each element within
  `<T5>`a's +-1.0 in of nominal (so a mirrored target can land up to ~2 in from robot two's element where robot
  one's was tuned to its own); and the centre goal, two yellow Pins (about (23.5, -23.5) and (47, -47) in) and the
  red/right corner's Load Zone sit ON the diagonal.
  *Source:* `competition::override2026::kSkillsMirror` (`include/shulib/competition/override_mirror.hpp`).
  *Confidence:* **reasoned** — the manual's drawings A10, A11 and Figure VURS1 (the Pins by pixel, about +-1 in);
  v2.1's text re-read, its drawings not re-measured.
  *Settle:* on the event's field — tape the two red goals and the two red Quadrants' Toggles from their walls.
  *Blast radius if wrong:* robot two's mirrored targets miss its elements by the field's own error.
  **Contained:** the wall reset re-seats each robot against its own wall; a target is tuned on the bench.
- [ ] **HA-259 — robot one's build is left-right symmetric where the route uses it.**
  *Claim:* every part a route uses at the field — an intake, a flipper, a claw, the camera's `left`, the two side
  sensors' `forward` — sits on the robot's centre line (or symmetric about it); a part off it lands TWICE its
  offset from the reflection of robot one's, because the robot is not mirrored, only its route.
  *Source:* `competition::mirrored`'s documentation (`field_mirror.hpp`); guide chapter 17.
  *Confidence:* **unknown** — no one has measured robot one's parts.
  *Settle:* MIRROR1 Step 0.2 (every part's sideways offset from the centre line), Step 0.3 (both side sensors'
  mounts — robot two reads the side robot one's route never uses).
  *Blast radius if wrong:* robot two's mechanism meets its piece or Toggle off to one side; its wall sensor's
  beam lands elsewhere along the wall (the keep-outs may refuse). **Contained:** the bench shows which steps
  need robot two's own numbers.
- [ ] **HA-260 — the robots' own asymmetry is small, and 3 in a side keeps the pair apart.**
  *Claim:* the drivetrain's own left/right differences are NOT mirrored — robot two's arrivals differ from the
  reflection of robot one's by its own asymmetry (host test plant, a 9 % weak side: 0.56 in of path, 0.44 s of
  time) — and each robot's REAL position near the diagonal stays within half the pair check's gap (3 in) of its
  plan, and its whole robot inside the check's 15 in square CENTRED on its tracking centre (a footprint sitting
  d in ahead of or behind the tracking centre still mirrors, but the check then misjudges each robot's clearance
  by up to d, out of the 3 in), so a route that passes the suite's 6 in check keeps the two robots apart on the
  field. *(The footprint half was added at MIRROR1-A's review, 2026-11-13 — a belief the check rested on and no
  number named; widened here rather than a fourth number, as R1 widened this one for the pair check.)*
  *Source:* `kMirrorGapIn` in `test/mirrored_pair_check.hpp`; the Chassis's mirrored-run documentation
  (`chassis.hpp`).
  *Confidence:* **invented** — the host plant's magnitudes; the 3 in is a chosen margin, unmeasured; where the
  tracking centre sits inside each robot's footprint, unmeasured.
  *Settle:* MIRROR1 Step 0.4 (each robot's mirrored route against the tape), Step 0.5 (both robots run the pair
  TOGETHER on a field, the drive speed turned down first, someone at each robot's stop); and, with Step 0.2's
  tape, how far each robot's front and back reach from its tracking centre (7.5 in each way is the check's
  square).
  *Blast radius if wrong:* the two robots touch near the diagonal. **Contained:** the suite refuses a route
  whose plan comes within 6 in; `isMirrored(ctx)` sends one robot to what lies on the diagonal.
---

## Robot 2 — the 2026 tank chassis (added 2026-09-14, R3b Parts 1–3)

The register was written for one robot. Robot two — the season's tank chassis, five coupled
motors per side driving four wheels per side, an IMU on port 2 and nothing else — is described
by `src/robots/chassis_table.hpp` (ports, signs, cartridge, IMU port, geometry) and the entries below
carry the facts about it as they are MEASURED, REPORTED or UNSET, each with its source and date.
Nothing here is invented for the robot itself: the invented entries (HA-132/133, and HA-136/137
added by R3d Part 1 on 2026-09-17) are the LIBRARY's thresholds and one SIM model shape, not the
robot's dimensions. Eighteen entries now — four measured on robot two (HA-127/128/129/130), one
reported (HA-124), two UNSET and refusing at boot (HA-125/126), seven reasoned
(HA-131/134/135/138/139/140/141), four invented (HA-132/133/136/137). The bench bot's rows above
are not rewritten.

**R3d Part 2 (2026-09-18) added HA-138, HA-139 and HA-140.** All three are about the PLATFORM
rather than the robot — what PROS's registry means by a port, what newlib does with a binary file
on the card, and what a record on that card is worth protecting with. They sit here rather than in
the R1a/R1b groups because they were found and are settled by robot two's session, and because two
of them are the direct inputs to "shulib Calibrate" and to the programs that boot from its record
("shulib Teleop" when they were written; "shulib Drive" since T4 Part 3).

- [ ] **HA-124 — robot two's drive wheels are 2.75 in in diameter.**
  *Claim:* a ruler across the tread of any of robot two's eight drive wheels reads 2.75 in.
  *Source:* `src/robots/chassis_table.hpp` (`.wheelDiameterIn = 2.75`, tagged `A4 register HA-124`);
  consumed through `tableDriveGeometry()` into `tableToDescription()`, i.e. only when the library
  program falls back to the TYPED TABLE. *(Amended at R3d Part 2, 2026-09-18: "shulib Teleop" then —
  "shulib Drive" since T4 Part 3 — builds from a `hal::DrivetrainDescription`, and when that comes from the SD card's calibration
  record the wheel diameter is not read at all — the record carries a MEASURED inches-per-radian
  per side instead, which is the radius × ratio product this entry and HA-126 together stand in
  for. The table remains the fallback, so the entry stays live until a record exists on the
  robot.)*
  *Confidence:* **reported** — the team lead, 2026-09-14: "standard wheels", the bench bot's
  measured size. Not yet rulered on THIS robot (worksheet Station F.0.3).
  *Settle (R3b / **R3d Part 3**):* the ruler; then R3d's one-push-along-a-tape scale calibration
  replaces the typed number with a measured inches-per-radian. That calibration EXISTS since R3d
  Part 1 (2026-09-17) and recovers the A2 plant's own radius to 1e-9; it has never been run on a
  robot, so this entry moves only on Part 3's bench log.
  *Blast radius if wrong:* every odometry distance scales by the error (a 2 % diameter error is
  a 2 % distance error, pinned quantitatively by the drive-encoder odometry's test 10); the
  stall check's spin travel scales identically, so the two never disagree with each other, only
  with the world. **Contained:** one table field.

- [ ] **HA-125 — robot two's track width, contact line to contact line.**
  *Claim:* NONE YET. There is no value in the tree. `src/robots/chassis_table.hpp` carries
  `.trackWidthIn = 0.0` (UNSET, tagged `A4 register HA-125`). *(Amended at R3d Part 2: the
  library program now refuses only when NEITHER source can describe a drivetrain — a calibration
  record on the SD card supplies an EFFECTIVE track width measured by a powered spin, HA-135, and
  a robot with one never reads this field. Without a record the typed table is still the second
  source, still UNSET, and the program still refuses at boot naming it.)*
  *Source:* `src/robots/chassis_table.hpp`; `src/common/boot_source.hpp` (the boot-source decision,
  `decideBootSourceFrom()` over `calibration::chooseBootSource`) and `src/common/boot_source.cpp`
  (`describeMissingForDescription`'s words, printed in the `typed table: cannot describe a
  drivetrain -- UNSET:` boot line) — in `src/main.cpp` until G1 Part 2b moved them out, unchanged.
  *Confidence:* **UNSET** — deliberately. A guessed track width turns every commanded yaw into
  the wrong wheel-speed difference (`kinematics/tank.hpp`) and every encoder-implied heading
  into the wrong number, silently; the drivetrain kind was stated, the width was not.
  *Settle (R3b / **R3d Part 3**):* tape measure, centre of the left contact line to centre of the
  right, front and back pairs (Station F.0.1); typed with provenance. R3d then measures it from the
  IMU (spin in place: Δθ_enc vs Δθ_imu — the cross-check `lastHeadingDisagreement()` exposes). That
  spin EXISTS since R3d Part 1 and recovers the plant's true width both ways to 1e-6 — but note
  that what it will produce on a floor is the EFFECTIVE width (HA-135), which is NOT the tape's
  number and is not meant to be.
  *Blast radius if wrong:* turns under-/over-rotate by the ratio of the error; the heading
  cross-check reads a steady bias on every turn (that is how a wrong width will be SEEN).
  **Contained:** one table field.

- [ ] **HA-126 — robot two's motor→wheel external ratio.**
  *Claim:* NONE YET. `.externalRatio = 0.0` (UNSET, tagged `A4 register HA-126`); the library
  program refuses while it is 0 AND no calibration record exists (amended at R3d Part 2 — see
  HA-125). "600 rpm" was reported, which names the blue cartridge and
  says nothing about the gears between the output shaft and the wheel.
  *Source:* `src/robots/chassis_table.hpp`; `hal/drive_geometry.hpp` (the ratio's one home).
  *Confidence:* **UNSET** — deliberately.
  *Settle (R3b / **R3d Part 3**):* "direct drive" (→ 1.0) or the tooth counts on BOTH sides (the
  R3a B1.2 amendment: ask per side — an asymmetric drivetrain is representable, two
  `DriveGeometry` objects); then R3d's push-along-a-tape calibration, which measures the radius ×
  ratio PRODUCT and cannot separate the factors — so once it has run, this entry is answered by
  being made unnecessary rather than by being filled in
  (`hal::DriveGeometry::fromInchesPerRadian`, R3d Part 1).
  *Blast radius if wrong:* the whole odometry scale (a 5:3 reduction typed as direct drive is a
  67 % distance error). **Contained:** one table field; the plant stays at 1:1 until a ratio
  other than 1:1 is reported (R3b Parts 1–3 D2-7).

- [x] **HA-127 — robot two's brain runs VEXos 1.1.5 (b18) under PROS kernel 4.2.2.**
  *Claim:* the kernel banner printed over USB at boot reads PROS 4.2.2, VEXos 1.1.5 (b18).
  *Source:* no in-tree source (exempt from direction 2) — measured from the serial capture,
  recorded in the development log on the `shulib-v2` branch (R3b Session 2 §9.1).
  *Confidence:* **measured** 2026-09-10.
  *Settle:* settled. Re-read the banner after any VEXos update.
  *Blast radius if wrong:* n/a — a fact about the brain, recorded so a future "it worked on
  1.1.5" has a number beside it. `arm-none-eabi-g++ 13.2.1` builds for it.

- [x] **HA-128 — starting a program from the CONTROLLER's menu arms a pretend field-control
  match on the brain.**
  *Claim:* launching (Run / Timed Run / Match) from the controller's own program menu puts
  VEXos into a competition-system mode (status `0x0f`: CONNECTED | DISABLED | AUTONOMOUS |
  SYSTEM) that persists across `pros v5 stop` / `run` until the controller leaves that mode;
  a controller linked and left on its home screen does not, and the program then runs in
  driver control normally.
  *Source:* `src/common/state_screens.{hpp,cpp}` (the competition-state screens, moved out of
  `src/main.cpp` at G1 Part 2c, output unchanged) and each program's boot-time status print (the
  bits `src/main.cpp` formats and passes, `A4 register HA-128`); the worksheet's Station A rule.
  *Confidence:* **measured** 2026-09-10 — the program restarted and went black when the
  controller linked; turning the controller OFF cleared it within seconds; the decisive test
  (controller linked, hands off, program started from the laptop) ran driver control for
  minutes.
  *Settle:* settled; the rule is in the worksheet and Daniel's note.
  *Blast radius if wrong:* n/a — the screen now names the state, so the diagnosis cannot again
  be "the touchscreen does not work".

- [x] **HA-129 — the brain's USB CDC ports renumber between plug-ins.**
  *Claim:* the brain enumerates as `ttyACM0/1` alone and `ttyACM1/2` beside the controller;
  a serial read must re-resolve the port from `pros lsusb` every time. `pros terminal` cannot
  run from a non-interactive shell (`termios` on stdin); raw reads of the USER CDC port work,
  COBS-framed with a `sout` channel tag (`strings` recovers the text lines whole).
  *Source:* no in-tree source (exempt) — the development log, R3b Session 2 §9.1.
  *Confidence:* **measured** 2026-09-10.
  *Settle:* settled. *Blast radius if wrong:* a capture from the wrong device, nothing more.

- [x] **HA-130 — robot two's port 18 travels ~20 % short of its side-mates on a push.**
  *Claim:* on a whole-robot hand push, port 18's encoder advances about 20 % less than the
  other four motors on its side (137 vs ~175 deg on the first push, 171 vs ~210 deg on the
  second), reproducibly — a member that is SLOW, not one that opposes. Cause UNKNOWN.
  *Source:* `src/robots/chassis_table.hpp` (the table comment, `A4 register HA-130`);
  `hal/motor_group.hpp` (the header cites it as the reason the disagreement floor must
  tolerate a slow member); `test/motor_group_test.cpp` test 16 (a member at 80 % of its mates
  is NOT flagged at rest or under command; the same member opposite in sign IS — pinned, with
  the threshold mutation observed red); and, since R3d Part 1,
  `include/shulib/calibration/drivetrain_calibrator.hpp`, which cites it as the reason each side's
  distance scale is taken through `hal::medianInPlace` — the SAME function the group will report
  `position()` with at run time, so a member that travels short cancels EXACTLY between the
  calibration and the run rather than approximately.
  *Confidence:* **measured** 2026-09-10, twice. What it means mechanically is not.
  *Settle (R3b / R3d Part 3):* watch it under power (the panel's `disagree` counts and the group's
  median); if it persists, swap the motor and re-measure. R3d's calibration report gives a second,
  sharper reading for free: it prints each side's member SPREAD over its push (the one 48 in push
  since R3d Part 2F-1 — there is no separate tape push any more), so one run says how short port 18 is
  against its four mates without anyone watching a live panel; and since 2F-1 its per-pulse serial
  record says whether port 18's own pulse moves the robot at all (a slipping gear is LEFT OUT by name,
  HA-134).
  *Blast radius if wrong:* if it turned out to be a fight rather than a shortfall, the group's
  median would still ignore it (one of five) and the monitor would count it; if the shortfall
  grew past 75 %, the floor (HA-133) would report it as frozen. Either way it is visible.

- [ ] **HA-131 — on robot two, ODO_STUCK cannot be detected by the wheels-vs-odometry
  cross-check.**
  *Claim:* with the drive encoders as the only odometry source (no tracking wheels), the
  `OdoStallCheck` comparison "wheels rolled but the estimate did not move" is a tautology — the
  estimate IS the wheels — and would report "not stalled" forever; so the check is configured
  with `independentMotionSource = false` on this robot, never reports a verdict, and a stuck
  robot is INVISIBLE to the health monitor until a detector built on the encoder-vs-IMU heading
  cross-check exists.
  *Source:* `include/shulib/motion/odo_stall_check.hpp` (the §2 ruling; `A4 register HA-131`);
  `src/programs/drive_program.cpp` (`kNoIndependentStallSourceNote` logged once at boot, in pieces
  so the 200-byte log line does not cut it; `src/main.cpp` until G1 Part 2b, "shulib Teleop" until T4
  Part 3); `localization/
  drive_encoder_odometry.hpp` (`lastHeadingDisagreement()`, the future detector's input).
  *Confidence:* reasoned (structural — a fact about the sources, not a measurement).
  *Settle (**R4** — RE-HOMED from "R3d/R4" on 2026-09-17 by R3d Part 1):* a stuck/slip detector on
  the heading cross-check plus current, measured against real pushes; or tracking wheels, which
  restore the independent source. **Why it moved:** R3d builds a DESCRIPTION of a drivetrain —
  which ports, which signs, which sides, what scale, what width. A stuck/slip detector is not a
  description; it is a run-time detector, and it cannot be tuned without real pushes on a real
  floor (the host plant's slip is a declared scenario fact, not an emergent one), which is R4's
  whole subject. R3d Part 1 does hand that detector its input, ready and host-proven: the effective
  track width the cross-check has to read zero against.
  *Blast radius if wrong:* n/a in the honest direction — the alternative was a monitor that
  lies. The cost is the missing detection itself: the driver is the supervisor (the panel says
  so), and an auton that pushes a wall will not fault; its watchdog bounds the damage.

- [ ] **HA-132 — the coupled-side agreement thresholds separate a fight from a transient.**
  *Claim:* evaluating only while a side is commanded above 1 V and its fastest member turns
  above 1 rad/s; flagging a member opposite the command above 0.5 rad/s or under 25 % of the
  side's fastest; and requiring 25 consecutive 10 ms ticks (250 ms) — together catch a real
  fight (a wrong sign, a stripped gear, a frozen encoder) and ignore a stick reversal's coast.
  *Source:* `include/shulib/teleop/coupled_side_monitor.hpp` (`SideMonitorConfig`,
  `PROVISIONAL (A4: HA-132)`); consumed by the old "shulib Drive" (its own loop), the tester's DRIVE
  station, `hal::MotorGroup`, the self-calibrator (`calibration/drivetrain_calibrator.hpp`) and —
  **since T4 Part 2 (2026-09-29)** — the library's drive protections
  (`teleop/drive_protection.hpp`) in every program that drives through
  `teleop::runDriverControl` (robot two's driver file — so "shulib Drive", and the competition
  program's `opcontrol()`, which run it — and the X-drive; "shulib Teleop" until T4 Part 3 retired
  it), which CUT on the group
  monitor's verdict and
  reuse two of its floors for the frozen-reading rule (HA-175) — one evaluator, one set of numbers.
  *Confidence:* **invented** for a first run (R3b Session 2's own words). No fight has yet been
  observed under power; the 250 ms window equals the over-current window by choice.
  *Settle (R3d/R4):* the drive logs' cut lines and the library's MOTOR_GROUP_DISAGREE episodes
  over a season of practice; a real fight (one motor deliberately reversed, wheels up, 3 V)
  must persist; a stick reversal at full speed must not.
  *Blast radius if wrong:* too tight = a healthy drive cut every second (the old drive program)
  or a fault line every run (a library motion); too loose = a fight stalls one motor against the
  rest of its side (five against four on the typed table, one against three on the 2026-09-25
  record) until something strips. **Since T4 Part 2 a false fight CUTS the drive for 1 s, in a
  match**, in every driver-control program that drives through the library's loop — a stick
  reversal at full speed that outlasts 250 ms is the first thing to watch for. **Contained:** one
  config struct, five fields.

- [ ] **HA-133 — the disagreement FLOOR: under 25 % of the side's fastest is disagreeing; 80 %
  is not.**
  *Claim:* robot two's port 18 at ~80 % of its mates (HA-130) must NOT be reported as a fight,
  while a member at 0 rad/s (a frozen encoder) or turning backwards must; 25 % of the fastest
  is the line, and it holds on every real drivetrain.
  *Source:* `include/shulib/hal/motor_group.hpp` (the header's policy note, `A4 register
  HA-133`); `test/motor_group_test.cpp` test 16 (the 20 % case tolerated, the opposite case
  flagged; the threshold mutation 0.25 → 0.85 observed red).
  *Confidence:* **invented**; the 80 %/25 % gap is wide by design.
  *Settle (R3d/R4):* the distribution of member velocities on a healthy side under power (the
  panel shows every member); if a healthy member ever sits below 50 % of its mates, the floor
  moves.
  *Blast radius if wrong:* a slow-but-healthy member reported as a fight on every run (and a
  fault line per run — and, since T4 Part 2, a 1 s CUT of the drive in driver control, repeating
  every 1.25 s while it lasts), or a nearly-frozen member (25–50 %) missed. **Contained:** one field
  of HA-132's struct.

### R3d Part 1's additions — the beliefs the self-calibration is built on (2026-09-17)

Four entries, added by the chunk that made the robot able to describe its own drivetrain. **None
of the four has been measured on anything**: two are reasoned from mechanism, one is a set of
invented thresholds, and one is a belief about the host PLANT rather than about a robot. The
calibrator they belong to is proven against the A2 plant and has never moved a physical drivetrain.

- [ ] **HA-134 — on a tank, one side forward alone turns the robot toward the other, and the IMU's
  yaw SIGN is therefore what names a drive port's side.**
  *Claim:* driving ONE side of a tank forward while the other is unpowered turns the robot: the LEFT
  side forward turns it CLOCKWISE, which is NEGATIVE yaw in shulib's CCW-positive canonical frame
  (`hal/imu.hpp`); the RIGHT side forward is positive. The drivetrain calibrator pulses one member at
  a time and reads that member's side off that sign alone. **Amended at R3d Part 2F-1 (2026-09-24):**
  the claim is only about the SIGN; how MUCH one member turns the robot is not claimed at all. The
  pulse therefore RAMPS — 3, then 6, then 9 V, each for 0.4 s with the robot at rest between (motor
  velocities AND the IMU's yaw rate under their floors) — and stops at the first level whose turn
  reaches the 2° floor. A port that no level turns is CLASSIFIED rather than refused blind: its own
  shaft turned while nothing else moved → a slipping gear, LEFT OUT of the description with a
  warning; other drive ports moved but the robot did not turn → it ROLLED (or is off the floor),
  refused naming the port; nothing moved → refused naming the port.
  *Source:* `include/shulib/calibration/drivetrain_calibrator.hpp` (the header's phase table and
  `finishPulse()`, `A4 register HA-134`); `test/drivetrain_calibrator_test.cpp` case 6 — every LEFT
  pulse's Δθ negative, every RIGHT pulse's positive, the resulting sides equal to the plant's wheel
  membership exactly, with the rule-swapping mutation observed red (since 2F-1 it is caught ON THE
  ROBOT too: a flipped rule makes the spin turn the wrong way and the new wrong-way guard stops it at
  15°); cases 22–26 for the ramp and the classification, on a plant that can now roll (HA-137).
  *Confidence:* reasoned — rigid-body kinematics. Note honestly what the host suite does and does
  not prove: it AGREES with the frozen F5 contract (`kinematics/tank.hpp`'s `forward()` computes
  ω = (right − left)/track, so left alone gives ω < 0), and the plant turns the robot USING that
  same contract — so the plant cannot be an independent check of the claim. What it does check is
  that the calibrator reads the sign the same way round as the kinematics writes it, which is the
  half that a typo would break.
  *What the first bench session showed (2026-09-24, R3d Part 3):* five starts of the Part 1 version
  (one 3 V pulse): three stopped at port 11 (0.14°, 0.17°) or before any pulse, and one reached port
  18, so ports 11–17 each cleared the 2° floor in that start — but the program printed no per-pulse
  numbers, so neither the SIGN of those turns nor whether anyone touched the robot is on record, and
  the port-11 stops carry a POSITIVE yaw where this entry predicts negative (noise at that size, or a
  push made back-first while FRONT was answered — open). **Not settled.**
  *Settle (R3d Part 3):* the first floor run of the 2F version with its serial record: one line per
  pulse (port, level, yaw, own travel, others' travel, verdict). Every LEFT member's deciding yaw
  negative and every RIGHT member's positive settles the sign; the levels used say how strong a
  single-member pulse must be on robot two.
  *Blast radius if wrong:* the two sides swap, so every commanded turn goes the wrong way and the
  odometry's heading cross-check reads double instead of zero. **Contained:** one comparison in
  `finishPulse()`. Loud rather than silent — and since 2F-1 caught before any record is written: the
  spin's wrong-way guard refuses at 15°.

- [ ] **HA-135 — the EFFECTIVE track width from a powered spin is LARGER than the taped one.**
  *Claim:* on foam, a tank turning in place SCRUBS — the wheels slide sideways as well as rolling —
  so the drive encoders over-count the turn relative to the yaw the robot actually achieves, and
  `W = (ΔR − ΔL) / Δθ_imu` comes out LARGER than a tape measure's contact-line-to-contact-line
  width. The effective number is the one `TankKinematics` needs to command a turn that lands and
  the one the heading cross-check must read as zero; the taped number is the fallback's provenance
  and is a different quantity, not a better or worse estimate of the same one.
  *Source:* `include/shulib/calibration/drivetrain_calibrator.hpp` (T8, `finishSpin()`,
  `A4 register HA-135`); `include/shulib/hal/drivetrain_description.hpp`'s `trackWidth` field
  comment; `test/drivetrain_calibrator_test.cpp` test 13, which adds a turn-scrub model the plant
  does not itself have: 5 % scrub yields 13.157 in against a true 12.5 — larger, in the predicted
  direction, and within 8 %.
  *Confidence:* reasoned. The A2 plant scrubs NOTHING, so the host recovers the true width exactly
  (1e-6 in both directions) and can say nothing whatever about the size of the gap. The DIRECTION
  is reasoned from the mechanism; the RATIO is unmeasured and no number in the tree claims it.
  *Settle (R3d Part 3):* tape the width, then calibrate, and record both numbers and their ratio,
  on the same robot on the same floor. That ratio is the first real datum about this claim.
  *Blast radius if wrong:* if the effective width came out SMALLER than the tape's, the scrub model
  is backwards and the reason for preferring a measured width to a taped one is wrong — visible
  immediately, because Part 3 records the two numbers side by side. Either way the calibrated
  number remains the one the kinematics wants, because it is defined by the measurement it came
  from.

- [ ] **HA-136 — every number in `CalibratorConfig` is invented for a first run (and so is every
  number in the calibration screen model).**
  *Claim:* the calibration's thresholds — `movedThreshold` 5° (carried unchanged from the bench
  tester's MOTOR WATCH, where it was invented too), `restFloor` 0.05 rad/s, `restYawRateFloor` 2°/s,
  `restSettleTime` 0.5 s, `restTimeout` 3 s, `pulseLevels` 3 / 6 / 9 V (`pulseLevelCount` 3),
  `pulseTime` 0.4 s, `pulseYawFloor` 2°, `decoupledOwnMin` 20° of shaft, `pushDistance` 48 in,
  `straightBound` 3°, `minPushRad` 2π (one wheel turn), `spinLevels` 3 / 6 V (`spinLevelCount` 2),
  `spinStartTime` 2 s, `spinStartYaw` 5°, `spinWrongWay` 15°, `spinTurns` 2, `spinTimeout` 20 s,
  `spinAgreement` 5 %, `imuReadyTimeout` 5 s, `commandTimeout` 0.1 s — plus the two reproducibility
  tolerances the class CLAIMS (`kReproScaleTolerance` 2 %, `kReproWidthTolerance` 3 %) — together
  let a real drivetrain be described without either refusing a good run or accepting a bad one.
  `pushDistance` is a CHOICE, not a belief: two field tiles, stated on the screen, the only distance
  there is (there is no distance question any more, so a wrong one cannot be answered).
  **Changed at R3d Part 2F-1 (2026-09-24):** ADDED `restYawRateFloor`, `pulseLevels` +
  `pulseLevelCount`, `decoupledOwnMin`, `pushDistance`, `spinLevels` + `spinLevelCount`,
  `spinStartTime`, `spinStartYaw`, `spinWrongWay`; REMOVED `pulseVolts` (3 V — now the ramp's first
  level) and `spinVolts` (3 V — now the spin's first level); CHANGED `straightBound` (it now bounds the
  ONE push, which also captures the signs — a twist could otherwise roll one side backward and invert
  its signs) and `minPushRad` (checked at DONE over the moved ports AND per side once the sides are
  known). One relation is now a precondition, not a hope: `restYawRateFloor × pulseTime <
  pulseYawFloor` (0.8° < 2°), so a drift slow enough to pass the rest check can never, on its own,
  cross the floor during a pulse — and a side can only come out wrong through a leak at least as big
  as the floor. **The screen model's estimates** (`calibration/calibrate_panel.hpp`): characters per
  472-pixel line LARGE 18 / MEDIUM 36 / SMALL 50 (and two sub-budgets, the header word 24 and the note
  beside RUN AGAIN 18), line heights 34 / 20 / 16 px, touch padding ±4 px sideways and ±10 px up and
  down, and a 0.5 s arming delay after every layout change — none measured; every literal is checked
  against the budgets at compile time, so a measured number is a one-constant change. *(R3d Part
  2F-2, 2026-09-24: the three character budgets were checked against what earlier programs showed
  legibly on a V5 — the SCREEN RULER could not be run — and kept: SMALL 50 sits under the bench
  tester's 54-character lines, MEDIUM 36 equals the widest MEDIUM line read whole in a photograph of
  the panel (the tester's DRIVE row), and LARGE 18 lies between the 12 characters read whole (the IMU
  test's live number) and a 23-character state screen reported shown. They are DERIVED lower-bound
  evidence, not a measurement; the SCREEN RULER still settles them.)*
  *Source:* `include/shulib/calibration/drivetrain_calibrator.hpp` (`CalibratorConfig`, every field
  tagged `PROVISIONAL (A4: HA-136)`); `include/shulib/calibration/calibrate_panel.hpp` (each budget
  tagged ESTIMATE); `test/drivetrain_calibrator_test.cpp`, where the push threshold, the pulse yaw
  floor, the missed-tick bound and the spin agreement are each attacked from BOTH sides — a value
  just under the line and the same value just over it — and the ramp, the rest's yaw-rate floor, the
  decoupled minimum and the spin's start check and wrong-way bound each have a case that fails
  without them (each mutation run and observed red at R3d Part 2F-1).
  *Confidence:* **invented**, every one, for a first run. The host plant can only show that each
  threshold FIRES where it is set, never that it is set in the right place. The ramp's first level
  and the spin's inherit the DRIVE station's first ceiling (3 V), whose only history is "nothing
  broke at 3 V" — and the first bench session showed 3 V on one member of five is intermittent on
  robot two (HA-134).
  *Settle (R3d Part 3):* one calibration on the floor with the serial record printed: the push table,
  one line per pulse (the level each port needed), one line per spin direction (the level it
  finished at). `restFloor`, `restYawRateFloor` and `pulseYawFloor` are the most likely to move; all
  three are bounded below by sensor noise nobody has measured. The screen's numbers settle at Bench
  Tests' SCREEN RULER (the last digit visible on each size's row).
  *Blast radius if wrong:* too tight and a good drivetrain cannot be described — a refusal, loud,
  with the offending number in the message; too loose and a bad one is described confidently, which
  would bake a wrong sign or a wrong side into a record the competition program then trusts. The
  asymmetry is deliberate: every threshold is placed so that its failure mode is a REFUSAL — and the
  one new relaxation, a slipping member LEFT OUT instead of refused, says so on the screen and in the
  record's absence of that port. **Contained:** one config struct and one screen header.

- [ ] **HA-137 — a coupled train commanded unequal volts turns at the MEAN of its members'
  effective volts; an all-coasting wheel may follow the driven one; a slipping member drives nothing
  (beliefs about the SIM, not about a robot).**
  *Claim:* in the A2 plant, a wheel whose coupled members carry DIFFERENT commands turns as though
  it were commanded the mean of them (a `Reversed` member's volts negated first); when every member
  carries the SAME command — which is what a `hal::MotorGroup` always produces — the wheel takes
  that value, bit for bit. **Added at R3d Part 2F-1 (2026-09-24), both OFF by default:**
  (a) **coast-follow** — `DrivePlantConfig::coastFollow`, κ ∈ [0, 1], tank only: a wheel whose every
  coupled member is at exactly 0 V AND in `BrakeMode::Coast` turns at κ × the driven wheel's spin
  instead of being pinned (a Brake/Hold side stays pinned). One side driven at v then gives
  ω = (1 − κ)·v/W and vx = (1 + κ)·v/2: κ = 0 is the old pinned pivot, κ = 1 pure translation.
  (b) **`MemberFault::Decoupled`** — a slipping gear: the member's command is left out of its
  train's command, and its encoder reports its own free spin plus `decoupledDrag` (default 0) of the
  train's travel.
  **Why (a) and (b) exist — and why the calibrator no longer rests on the pinned wheel:** until 2F-1
  every unpowered wheel was pinned (an implicit perfect brake) and the calibrator's own test rig ran
  with no static friction, so a one-sided pulse could only PIVOT and always passed; at the tree's
  usual kS = 1.2 the same single 3 V pulse turns this plant by EXACTLY nothing
  (`test/drive_plant_coupled_test.cpp` case 25 keeps that as a regression), and on robot two it
  turned the robot 0.14–0.17°. The calibrator's cases now run at kS = 1.2 with κ stated in every one,
  including κ = 0.9 and 1 (the robot rolls and the pulse is refused ROLLED, naming the port) and a
  slipping member (left out, and the run still saves).
  *Source:* `include/shulib/sim/drive_plant.hpp` (`wheelCommand()`, `coupledOnlyCommand()`,
  `stepWheelsCoastFollow()`, `wheelCoasting()`, `MemberFault::Decoupled`, `A4 register HA-137`);
  `test/drive_plant_coupled_test.cpp` cases 18–26. Every other suite is bit-identical with both
  extensions at their defaults (assertion counts and every printed value, compared case by case
  before and after at R3d Part 2F-1).
  *Confidence:* **invented**, and deliberately SIM shapes rather than robot facts. Everything is
  torque-free like the rest of the plant (`motor_model.hpp`'s honesty boundary). The mean is
  OPTIMISTIC — a real motor back-driving four gearboxes reaches less than 1/N of its free speed. κ is
  a KINEMATIC stand-in, not derived from a wheel type, and the direction a real robot takes is not
  obvious: lateral scrub (traction wheels) pushes a one-sided push toward ROLLING (κ → 1), while a
  rigid body whose idle side rolls freely with no lateral force — the ideal of an all-omni robot like
  robot two — turns MORE than a pinned pivot (for a uniform square robot its idle side runs backward
  at about a fifth of the driven side's speed, a negative κ this field does not allow). So κ ∈ [0, 1]
  spans from the pass case to the failing one; which end robot two is nearer is exactly what the 2F
  serial record will show.
  *Settle (R6 — back-fit, beside HA-48's model-shape question):* compare one member's pulse on a
  real robot (the 2F record: the level it needed, the yaw it gave, how far the OTHER side's encoders
  moved) against the plant's prediction for the same command. If the robot needs materially more
  than 1/N, the mean becomes a weighted model; the other side's travel over a pulse is κ's first
  measurement.
  *Blast radius if wrong:* the host suite predicts pulses that turn a real robot more (or less) than
  it does — which the ramp and the classification now absorb: a pulse too weak steps up, and one that
  never turns the robot is refused naming the port and saying whether it rolled or nothing moved.
  It cannot make a calibration WRONG, only make one fail to finish. **Contained:** four functions in
  the plant, all behind switches that default off.

- [ ] **HA-138 — `registry_get_plugged_type(i)` is ZERO-indexed: registry index i is PHYSICAL
  PORT i + 1, and the valid indices are 0–20.**
  *Claim:* PROS's smart-port registry is the ONE API in the tree that is not numbered the way
  humans, motors, sensors and every other PROS call number ports. Index 0 is physical port 1;
  index 20 is physical port 21; index 21 does not exist.
  *Source:* vendored `include/pros/apix.h:640-660` — "Returns the type of the device plugged into
  the **zero-indexed** port … The V5 port number from 0-20", and `ENXIO` outside that range. The
  device type values are vendored `pros/device.h:48-63` and are NOT consecutive (NONE 0, MOTOR 2,
  ROTATION 4, IMU 6, DISTANCE 7, RADIO 8, VISION 11, ADI 12, OPTICAL 16, GPS 20, AIVISION 29,
  SERIAL 129, UNDEFINED 255). Bound in `include/shulib/hal/pros/device_registry.hpp`, where the
  conversion is ONE marked line; shimmed in `test/pros_shim/pros/apix.h`, which is indexed the
  PROS way so the adapter's conversion is the thing under test rather than a convention the shim
  shares with it; exercised by `test/pros_device_registry_adapter_test.cpp` from BOTH ends
  (physical 1 → index 0, physical 21 → index 20, asserted directly through the shim's recorded
  `lastIndex`), and out-of-range ports are asserted never to reach PROS at all.
  *Confidence:* **reasoned** — the vendored text states it plainly, and it already cost a session:
  `src/programs/bench_r3a.cpp`'s registry comment (just above `kMaxRegistryIndex` — find it by that name:
  lines 135-145 at R2a, 112-122 before it, and later chunks moved it again) records 2026-08-18, when the tester scanned 1..21 as if the index
  were physical, shifted every reading by one port and invented a device at index 21. HA-120 had
  PREDICTED exactly that (the 2026-08-13 "ADI expander" sighting came from index 21) and the
  prediction was not acted on. This entry exists so the mapping is a NAMED belief with a settle
  step rather than a comment in a tester.
  *Settle (R3d Part 3, Station G's census):* run "shulib Calibrate" beside the brain's own Devices
  screen. The program's `STAGE 0` census prints one line per non-empty PHYSICAL port; every line
  must name the same port and the same device as the brain does. Ten motors on 11–20, an IMU on 2
  and a radio on 1 is the expected answer for robot two.
  *Blast radius if wrong:* a calibration that pulses the wrong motors, or a boot check that
  compares a record against a port map shifted by one and refuses a healthy robot — confidently,
  and with a message naming the wrong ports. **Contained:** one line, one adapter, and the test
  that pins it from both ends.

- [ ] **HA-139 — `fopen(path, "rb")` under `/usd/` reads back exactly the bytes `"wb"` wrote.**
  *Claim:* the read half of HA-122. A calibration record written through `ProsBlockSink`
  (`fopen`+`fwrite`+`fflush`) and read back through `ProsBlockSource` (`fopen("rb")`+`fread`) is
  byte-identical on the V5's FatFS — no newline translation, no 0x1A end-of-file interpretation,
  no short read on a file that is entirely present.
  *Source:* `include/shulib/hal/pros/block_source.hpp`;
  `test/pros_block_source_adapter_test.cpp`, which writes NUL, LF, CR, CR+LF, 0x1A, DEL and 0xFF
  through the sink and reads them back through the source against a REAL temp directory.
  *Confidence:* **reasoned, and FLAGGED.** The host test proves the two adapters agree on Linux —
  where `"rb"` and `"r"` are the same thing, because POSIX does not translate. **No host test can
  fail if the mode is wrong**, and newlib on the brain is a different library. The mode is
  therefore a belief that only a brain can settle, and it is written down as one instead of being
  claimed by a test that could not see it.
  *Settle (R3d Part 3, Station G):* run "shulib Calibrate" to `SAVED` — which is itself the
  measurement, because the program writes, flushes, CLOSES, re-opens, reads back and compares the
  bytes before it says the word (ruling P5). Then pull the card and compare on a laptop: **since CAL1
  Part B (2026-11-05) robot two's Calibrate is the library's standard one, which saves through
  SAFE-SAVE's `saveRecordSafely` (`calibration/record_store.hpp`, over `hal/pros/file_store.hpp`'s
  `ProsFileStore`) — after a SAVED run BOTH files, `shulib_drivetrain.bin` and its backup
  `shulib_drivetrain.bak`, are 180 bytes and byte-identical (`cmp`, or `xxd`)**, which settles it
  independently of the program's own opinion (worksheet Station G, the RUN AGAIN paragraph).
  *Evidence so far:* a record written by R3d's own Calibrate (through `ProsBlockSink`) decoded at
  every one of eight boots of "shulib Teleop" on the evening of 2026-09-25 (`calibration record: DECODED`) — a
  read of what a write left, on the brain, by date; not yet the `cmp` above, and not yet through the
  library's save.
  *Blast radius if wrong:* `NOT SAVED` (or `SAVE UNCONFIRMED`) on the brain, with the reason — the
  read-back is precisely what makes this failure loud instead of silent. The worse variant, a file that
  reads back subtly changed and still passes a byte compare, cannot happen: the compare IS byte-exact,
  and the CRC would refuse the difference at the next boot. **Reaches:** the library's safe save
  (`saveRecordSafely`: its every step reads back, so a wrong mode is a refused save, never a silent
  one) and the boot's load (`loadRecord` / the drive program's `calibration record:` line), which would
  then refuse the record — the drive program boots from the typed table or refuses, never from a
  changed record.

- [ ] **HA-140 — a CRC-32 over the calibration record is worth its four bytes, where the
  blackbox's format deliberately carries none (a DESIGN note under HA-122).**
  *Claim:* the two on-card formats this tree writes take OPPOSITE decisions about integrity, and
  both are right. `diag/blackbox_format.hpp` has no checksum: it is read by a HUMAN after the
  fact, frame by frame, a truncated tail is expected and decodable, and the cost of a corrupt
  field is a misleading number in an investigation. `calibration/calibration_record.hpp` has a
  CRC-32: it is read by the ROBOT, once, at boot, and every field is immediately load-bearing — a
  flipped bit in a SIGN reverses a motor against its side-mates, one in a SCALE makes every
  odometry inch a lie, one in the track width's exponent makes every commanded turn wrong, and
  none of those is visible from outside.
  *Source:* `include/shulib/calibration/calibration_record.hpp` (`crc32Ieee`, computed BITWISE with
  no lookup table — 1 KiB of flash for 180 bytes of input is the wrong trade in a header a
  composition root includes); `test/calibration_record_test.cpp`, which pins the arithmetic against
  the published CRC-32/IEEE check values (`"123456789"` → 0xCBF43926) and flips one bit in each
  field class. **Since H1 Part A1 (2026-09-29) the bitwise CRC itself lives in
  `include/shulib/diag/crc32.hpp` (`diag::crc32Ieee`)**, because the SHUL/2 wire checks every frame
  with the same function and `diag/` may not include `calibration/`; the calibration record calls it
  through `calibration::crc32Ieee`, now a one-line forwarding function, so the record, its tests and
  this judgement are unchanged. (The wire's rate — a CRC on every tick frame — is a separate trade,
  judged with the wire's sink, not here.)
  *Confidence:* **reasoned.** The polynomial and its properties are standard; what is a JUDGEMENT
  is the trade — four bytes and a few microseconds per boot against the class of silent failure it
  removes. It is recorded here because it is exactly the kind of decision a later chunk would
  otherwise re-open without knowing why the blackbox differs.
  *Settle (R3d Part 3):* not a measurement so much as a confirmation — the record decodes on the
  brain and the boot log prints the description it spells. If the bitwise CRC ever shows up in a
  loop-rate measurement (it will not: 180 bytes × 8 iterations, once per boot), the table version
  is a two-line change. *(Since H1 Part A1 that one function is also the SHUL/2 wire's per-frame
  check, so a switch to a table is decided for both at once — with the wire's sink, which judges
  the per-tick rate.)*
  *Blast radius if wrong (the CRC is not worth it):* four wasted bytes. **If it were ABSENT:** a
  corrupt record drives a robot wrong, confidently, and the first symptom is a robot that will not
  drive straight. The asymmetry is the whole argument. **Contained:** one function
  (`diag/crc32.hpp`), shared since H1 Part A1 by two formats — the calibration record and the
  SHUL/2 wire. A change to it moves both, and both pin it: the record's golden and the wire's
  goldens each carry CRC values computed outside the code.

**G1 Part 2a (2026-09-24) added HA-141.** Like HA-138 it is about the PLATFORM, and it is the
belief the new on-robot device provider rests on: HA-138 says WHICH registry index is which port;
this says the KIND the registry reports on that port can be trusted for the three sensor kinds.

- [ ] **HA-141 — the registry's KIND answer is right for an IMU, a rotation sensor and a GPS, and
  an empty port reads NONE.**
  *Claim:* `registry_get_plugged_type()` on the index of a port carrying a V5 inertial sensor
  returns `E_DEVICE_IMU` (6), on a rotation sensor `E_DEVICE_ROTATION` (4), on a GPS
  `E_DEVICE_GPS` (20) — the same way it has been relied on to return `E_DEVICE_MOTOR` (2) for a motor
  since R3d Part 2 — and on a port with nothing plugged in, `E_DEVICE_NONE` (0). False if any of the
  three reports a different value, `E_DEVICE_UNDEFINED`, or NONE while plugged in (for example
  while the sensor is still enumerating after power-up).
  *Source:* `include/shulib/hal/pros/device_provider.hpp` (`ProsDeviceProvider::expectKind()`,
  tagged `A4 register HA-141`), which asks before constructing ANY port device and refuses on any
  other answer; the kind mapping is `hal/pros/device_registry.hpp`'s `kindOf()` over the vendored
  `pros/device.h:48-63` values; host-tested through `test/pros_shim/pros/apix.h` by
  `test/pros_device_provider_test.cpp` (cases 1b, 1c, 1k), which — like every shim test — tests the
  provider against this belief and cannot test the belief.
  *Confidence:* **reasoned** — the vendored enum names each kind, and for ONE of the three a census
  has already agreed with the robot: robot two's census read the IMU on port 2 (2026-09-10, and again
  2026-09-24), alongside MOTOR on its drive ports and the radio on port 1. **What is new is the
  DEPENDENCE:** until this part only the
  boot check read the registry, and only for motors. `ProsImu` and `ProsRotation` cannot throw and
  `ProsGps` defers on an unreadable device, so for those three kinds the registry's answer is now the
  ONLY thing that can refuse an empty or wrong port — which is why the belief gets its own entry.
  No rotation sensor or GPS has ever been plugged into either robot.
  *Settle (the first boot of a program that boots from the record — "shulib Drive" since T4 Part 3 —
  / R3) — reworded at G1 Part 2b:* on such a program the boot check reaches every IMU and drive-port fault FIRST, through the same
  registry call (`calibration/boot_check.hpp`'s `verifyAgainstRegistry`, which refuses a record
  whose IMU port no longer reads IMU), so a pulled IMU cable is refused by the BOOT CHECK and never
  reaches the provider — that boot cannot show "nothing plugged in (expected an IMU)". So the belief
  settles in two halves: the **IMU-present half** on the G1 Part 2b program's first boot (the
  provider constructs the IMU on the registry's word and starts its calibration); the **provider's
  kind refusal** on the first program that asks the provider for a port the boot check does NOT
  verify — a GPS or a rotation sensor, the day one first exists on a robot — with a pair of boots
  (plugged, then unplugged) for each. No worksheet step for it until then. The census beside the
  brain's own Devices screen (HA-138's settle step) reads the kinds at the same time.
  *Blast radius if wrong:* **refuses rather than lies.** A sensor the registry mis-kinds is REFUSED
  at boot, by port, with what was found — a robot that will not start, on a bench, with the reason on
  the screen; it cannot become a live adapter over an empty port (the failure this check exists to
  prevent). The one silent variant would be the registry reporting the right kind for an EMPTY port,
  which would put us back where the provider started — an adapter over nothing — and the IMU's own
  `isReady()` never going true is then the only tell. **Contained:** one check, in one provider.

**DEFECTS2 (2026-09-26) added HA-142–HA-146.** Found by the read-only research for T3 (claw pivots
that must HOLD their angle on release): every Brake and Hold stop in the library reached a V5 as a
0 V command, which PROS documents as ignoring the brake mode — so a declared Hold went limp and the
drive's "0 V + Brake" cancel coasted, while every host test passed (the fakes echo the mode, the A2
plant pins a wheel at 0 V, the shim had no `brake()`). The fix is one function in the PROS adapter
(`hal/pros/motor.hpp`, "THE STOP"); these five entries are what it — and everything that stops a
motor — now rests on. **None of them has been observed on a robot.**

- [ ] **HA-142 — `move_voltage()` and `move()` ignore the brake mode; only `brake()` applies it.**
  *Claim:* a V5 motor commanded `move_voltage(0)` is under VOLTAGE control at 0 V whatever its
  brake-mode register says; the mode acts only after `brake()`, which "sets motor velocity to zero,
  which will cause it to act according to the set brake mode". So a stop that must brake or hold is
  "the mode + `brake()`", never "the mode + 0 V". False if a motor set to Hold and commanded
  `move_voltage(0)` holds its position, or if `brake()` under Hold does not.
  *Source:* vendored `include/pros/motors.h:52` (`motor_move`) and `:253-254`
  (`motor_move_voltage`): *"This function will not respect brake modes, and simply sets the voltage
  to the desired value"*; `motors.h:82-86` and `motors.hpp:260-284` (`brake()`); the mode enum's
  own words, every one about a motor that is STOPPED (`motors.h:860-868`). In the tree:
  `include/shulib/hal/pros/motor.hpp` (THE STOP; `stopAtZero()`), which sends a zero under a
  COMMANDED Brake/Hold as `brake()`; the shim's control state (`test/pros_shim/pros/motors.hpp`);
  the comments that now say so in `hal/motor.hpp`, `hal/mechanism.hpp`,
  `manipulation/mechanism_op.hpp`, `motion/motion.hpp`, `sim/drive_plant.hpp` (each tagged
  `A4 register HA-142`); host-tested by `test/pros_motor_adapter_test.cpp` (DEFECTS2 a1–a11, a9b
  included) and `test/brake_stop_stack_test.cpp` (s1–s8) — which test the adapter against this
  belief and cannot test the belief. The same belief is why the adapter re-sends a mode write the
  port refused before its next `brake()`: `brake()` stops under the REGISTER, not under what the
  program meant (a9b).
  *Confidence:* **reasoned** — the vendored text is explicit, and says it twice. Nothing else in
  PROS's headers contradicts it.
  *Settle (R3, robot two — the first look; T3's claw station — the loaded one):* compare two stops
  from the same speed WHILE ENABLED — a stick release under the ruled Coast, and a library Brake
  stop (COMP's first autonomous `chassis.brake()`, or a driver-loop build with
  `DriverLoopConfig::releaseBrakeMode = Brake`): the Brake stop should be visibly shorter. Before
  DEFECTS2 the two were the same 0 V. **A DISABLE is a confounded test:** the loop's disable path
  issues its Brake stop only once the brain already reports disabled, and whether VEXos is by then
  cutting motor output is HA-145's open question — so what a disable shows is that entry as much
  as this one. For the hold: T3's claw
  pivot declared Hold, released with a load — it stays; turned by hand, it pushes back.
  *Blast radius if wrong:* if `move_voltage(0)` DID honour the mode, DEFECTS2 changed the wire call
  and not the behaviour — harmless. If `brake()` does NOT apply the mode, every Brake/Hold stop stays
  limp: a claw drops, a cancel coasts — the pre-DEFECTS2 robot, no worse. **Contained:** one
  function (`ProsMotor::stopAtZero()`), below every wrapper.

- [ ] **HA-143 — what a V5 motor PHYSICALLY does at `move_voltage(0)`: believed to free-wheel.**
  *Claim:* at `move_voltage(0)` the shaft is undriven and turns freely except for friction — the
  "coast" `hal::BrakeMode::Coast` promises and `IMotor`'s contract describes. False if a 0 V command
  short-circuits the windings (a dynamic brake), or holds.
  *Source:* `include/shulib/hal/motor.hpp` (the `BrakeMode::Coast` doc, tagged `A4 register
  HA-143`); `include/shulib/hal/pros/motor.hpp` (THE STOP: Coast, and every zero under no commanded
  mode, stays `move_voltage(0)` — exactly the pre-DEFECTS2 wire traffic, so Calibrate, the old
  shulib Drive (until T4 Part 3) and the tester's DRIVE station are unchanged); the plant's
  coast-follow (HA-137) and the
  calibrator's one-sided pulses assume it.
  *Confidence:* **invented — UNKNOWN.** PROS says only that the voltage "simply" is set; it does not
  say what an H-bridge at 0 V does. Two QUALITATIVE observations point toward a coast and neither is
  a measurement: the R3d Part 2F investigation of Calibrate's first night concluded robot two's
  idle side coasts during a one-sided pulse (the development log; HA-137 carries the model it led
  to), and the team lead describes the 2026-09-25 Teleop drive as coasting on stick release (the
  ruling that keeps it).
  *Settle (R3/R4):* robot two at a known speed, WHILE ENABLED, sticks released (0 V under Coast)
  versus an enabled Brake stop (`brake()` under Brake, HA-142 — COMP's first autonomous
  `chassis.brake()`, or a driver-loop build with `DriverLoopConfig::releaseBrakeMode = Brake`): the
  roll distances should differ clearly. Not against a DISABLE — HA-142's settle step says why that
  comparison is confounded (HA-145). And by hand, a motor at 0 V turns as easily with its register
  at hold as at coast (which also checks HA-142's first half).
  *Blast radius if wrong (0 V brakes):* the "coast" in Calibrate's pulses and the plant's κ is
  really a brake — the calibration's side test is modelled with the wrong sign of error — and the
  ruled driver-control release feel is a braking one. Nothing DEFECTS2 did makes it worse: the Coast path is
  byte-for-byte the wire traffic it was. **Contained:** it is a fact about the motor; the only code
  it steers is the plant's coast-follow model.

- [ ] **HA-144 — a REPEATED `brake()` under Hold re-captures the held position — and so, perhaps,
  does a `set_brake_mode()` of the SAME mode written to a motor already held.**
  *Claim:* each `brake()` under Hold re-targets the hold at the shaft's CURRENT position, so one
  issued every 10 ms tick under a load that sags between ticks lets the shaft creep, never springing
  back. False if a repeated `brake()` keeps the first target. *Second half (added in DEFECTS2's
  review):* whether a `set_brake_mode(hold)` written to a motor ALREADY stopped under Hold does the
  same. The adapter keeps sending that write every tick wherever a caller re-states an unchanged
  mode over a stop: `DriveBrake` (Brake, every tick since C1 — Brake holds no position, so only
  Hold is at stake) and the driver loop's release re-assert when `releaseBrakeMode` is Brake or
  Hold (no program does; the ruled Coast never stops). False if a same-mode write leaves the stop
  alone — which is what the shim assumes and cannot test.
  *Source:* `include/shulib/hal/pros/motor.hpp` (THE STOP, and `stopAtZero()`: the adapter issues
  `brake()` only on the TRANSITION into a stop — a zero repeated under the same mode sends nothing
  more; tagged `A4 register HA-144`); `include/shulib/hal/motor.hpp` (`BrakeMode::Hold`); pinned by
  `pros_motor_adapter_test.cpp` a8 and `brake_stop_stack_test.cpp` s7 (one `brake()` across ten
  ticks of the DriveBrake pattern / five released driver-loop ticks — mutation-proven). Both of those
  also send the same-mode `set_brake_mode()` every tick (a8 counts ten); the shim's header lists it
  among what it cannot model (`test/pros_shim/pros/motors.hpp`).
  *Confidence:* **invented.** PROS says only that Hold "may behave differently than calling
  motor_move_absolute(port, 0)", and nothing about a mode write to a stopped motor. The adapter is
  built so that the first half being TRUE costs nothing; for the second half it is NOT — the write
  is still sent. Not suppressed in the adapter (skip a same-mode write while stopped under it):
  that drops the per-tick re-write which heals a register changed underneath (a second adapter on
  the port, a replugged motor — HA-98), changes `DriveBrake`'s wire traffic from what it has been
  since C1, and today guards nothing a program does — the ruled release is Coast, T3's claws take
  their safe state once, and `DriveBrake` stops under Brake.
  *Settle (T3's claw station):* a loaded claw held by one `brake()` versus one re-issued every tick
  (a test build) — the sag over 30 s; then the same claw held by one `brake()` with a
  `set_brake_mode(hold)` written every tick (a test build) — the same sag test. **The opposite risk
  is the real one to look for:** if a hold LAPSES without a fresh command (a firmware timeout no PROS
  text mentions), issuing once is wrong — the claw would hold, then let go. Watch a held claw for a
  full minute.
  *Blast radius if wrong (it lapses):* a held claw lets go after some time with no command; the fix
  would be a periodic re-issue, in the same one function. *If the same-mode write re-targets:* only
  a Hold that a caller re-states every tick creeps — none today; the fix is the suppression above,
  in `setBrakeMode()`. **Contained:** `stopAtZero()` / `setBrakeMode()`.

- [ ] **HA-145 — Hold (or Brake) while VEXos has the robot DISABLED, and after re-enable.**
  *Claim:* UNKNOWN in both halves — whether a motor stopped under Hold keeps holding while the
  brain is disabled (between autonomous and driver control, after the match), and what state it is
  in when re-enabled. The belief this register can state: VEXos cuts motor output while disabled,
  so a hold does NOT survive it.
  *Source:* `include/shulib/hal/pros/motor.hpp` (THE STOP: the adapter's stop record is per object
  and cannot see a disable — after re-enable, a zero repeated under the same mode sends nothing new
  until the command leaves zero or the mode changes; tagged `A4 register HA-145`). The driver loop
  (`include/shulib/teleop/teleop_loop.hpp`) cancels into Brake on a disable and re-asserts its
  release mode every tick; neither re-issues a `brake()` for an unchanged mode. The loop header's
  step 1, which says the disable stop lands before VEXos cuts the motors, points here (tagged
  `A4 register HA-145`).
  *Confidence:* **invented — UNKNOWN.** No PROS or VEX text in the tree covers it.
  *Settle (T3's claw station, then COMP):* stop a loaded claw under Hold, disable (the controller's
  Timed Run ending), feel the shaft; re-enable WITHOUT commanding it and feel again. The same
  question covers the driver loop's disable stop: its Brake `brake()` is issued once the brain
  already reports disabled (the loop's header records, from R3b Part 0b, that this lands before
  VEXos cuts the motors itself — how long that window is, and whether a `brake()` issued in it
  outlasts the cut, is this entry).
  *Blast radius if wrong:* a claw holding a game piece at the end of autonomous drops it at the
  disable; a re-enabled claw stays limp until its first command. The re-arm would be one explicit
  "re-issue the stop" at the start of each competition phase — not built, because nothing yet says
  it is needed. **Contained:** the adapter plus one call site per phase.

- [ ] **HA-146 — leaving the stop: after `brake()`, the next `move_voltage(v ≠ 0)` drives the motor
  at once; a `set_brake_mode()` repeated every tick changes nothing but a RUNNING motor's register.**
  *Claim:* the velocity-zero stop `brake()` puts the motor in is a control mode the next
  `move_voltage()` REPLACES — not a latch — so a held claw moves the tick it is commanded, and a
  cancelled drive responds to the first stick after it. And a `set_brake_mode()` sent while the
  mode is unchanged (the driver loop's release re-assert, every tick since DEFECTS2; `DriveBrake`'s,
  every tick since C1) neither jolts nor resets a RUNNING motor. (The same write to a motor STOPPED
  under Hold is HA-144's second half, not this entry.)
  *Source:* vendored `motors.h:253-254` (`move_voltage` "simply sets the voltage") and
  `motors.hpp:260-284` (`brake()` is a velocity command); `include/shulib/hal/pros/motor.hpp`
  (`setVoltage()` clears the stop record on any non-zero command; tagged `A4 register HA-146`);
  `include/shulib/teleop/teleop_loop.hpp` (step 6b); pinned on the host by
  `pros_motor_adapter_test.cpp` a4 — against the shim, which leaves a stop at once.
  *Confidence:* **reasoned** — PROS's movement calls each set a new target; nothing says `brake()`
  is sticky.
  *Settle (R3 / T3's claw station):* hold a claw, then command it open — it moves on the first
  tick; on robot two, disable, re-enable, and drive — the first stick moves the robot.
  *Blast radius if wrong:* a held mechanism that ignores its next command (a claw that will not
  open); a drive sluggish after every library stop. **Contained:** `ProsMotor::setVoltage()`.

### T3's additions — robot two's claw pivots and its pneumatic claw (2026-09-26)

**T3 (the claw pivots, 2026-09-26) added HA-153 to HA-156**, its addendum HA-157 to HA-159 (the
pneumatic claw, below), and its merge HA-147 (the controller belief the claw's toggle rests on,
first below). HA-142 to HA-146 are DEFECTS2's (above); HA-148 to HA-152 were reserved for T3 and
are unused. HA-153 to HA-156 are beliefs about motors NOBODY HAS SEEN YET: the team lead reported
the two claw pivots on 2026-09-26, no census or capture has shown a motor on port 6 or 7, and
nothing has commanded either on a robot. `src/robots/mechanism_table.hpp` declares them;
the standard Calibrate acts on the declaration through `src/robots/calibrate_facts.hpp` (it never makes 6 or 7 a
drive candidate; robot two's own `src/programs/calibrate_program.cpp` did until CAL1 Part B, 2026-11-05);
"shulib Drive" builds them and robot two's driver file drives them (since T4 Part 3; "shulib Teleop"
did from T3's Teleop wiring, its Part 2, the same day, until then) — host- and build-gated, never
run. Worksheet Station J is their first run.

- [ ] **HA-147 — a V5 controller's connected flag and its button levels change together.**
  *Claim:* on the first tick `is_connected()` reads true after a disconnect, `get_digital()`
  already returns the button's LIVE level — never a stale one from before the drop — and a
  button's level does not bounce across two 10 ms samples on a press or a release. False if the
  flag can come back a tick (or more) before the button data does, or if a press or release reads
  down-up-down across consecutive ticks.
  *Source:* `include/shulib/teleop/mechanism_binding.hpp` (ToggleBinding's A PRESS MUST BE SEEN
  rule, tagged `A4 register HA-147`: the FIRST connected tick's level is taken as live);
  `test/toggle_binding_test.cpp` T4 ("WHAT THE GUARANTEE RESTS ON") pins the boundary in the
  direction of the hazard — a stale "up" on the first reconnected tick, then the live "down", FLIPS
  the claw. HA-103 claims only that a disconnected controller reads 0 and that `is_connected()` is
  the only validity signal; nothing claimed the two change together. Found by T3's B2 review and
  registered at the merge. **Since MECH-A4** the same rule runs every one-button control of a driver
  file — a toggle, a preset, a homing press: `include/shulib/mechanism/driver_inputs.hpp`
  (`DriverInputs`' A PRESS MUST BE SEEN paragraph, tagged HA-147); robot two's claw runs on it since
  MECH-A5 (2026-10-29: its driver file's `toggle(claw, Button::A)`).
  *Confidence:* **invented** — unmeasured; nothing vendored says either way.
  *Settle (worksheet Station J.9.5):* open the claw, hold A down, pull the controller's cable,
  plug it back in STILL holding A, keep holding for two seconds, then let go. Nothing may move
  until A is released and pressed again; then one press closes it. Record whether the claw moved
  while A was held across the reconnect.
  *Blast radius if wrong:* a phantom flip of the pneumatic claw when the driver holds A across a
  controller reconnect (or on a bouncing release) — an open flip drops what the claw holds. Until
  it is measured the driver's rule is: after a controller drop, let go of A before pressing it
  again (the driver-control cookbook says so). **Contained:** `ToggleBinding::tick()` and
  `DriverInputs::tick()`'s one-button controls; the mitigation (a reconnect settle window) is flagged for the team lead's ruling after J.9.5, not
  built.

- [ ] **HA-153 — robot two's claw pivots are RED motors on ports 6 and 7, and a positive command
  moves each the way the team lead calls "forward".**
  *Claim:* robot two carries two SEPARATE claw-pivot motors, each a V5 smart motor with a RED
  (100 rpm) cartridge: one on smart port **6** (forward while R1 is held, back while R2 is), one on
  port **7** (L1 / L2). A POSITIVE voltage — each pivot's `.motors = {Forward}` (until MECH-A5 the
  table's `+6` / `+7`), which nothing reverses —
  moves each pivot the way R1 (port 6) and L1 (port 7) are meant to move it. False if a census shows
  no motor, or another kind of device, on 6 or 7; if either cartridge insert is not red; or if R1 /
  L1 moves its claw the "back" way.
  *Source:* `src/robots/mechanism_table.hpp` (robot two's region: the declarations `clawPivot6` and
  `clawPivot7` — `.cartridge = Red`, `.motors = {Forward}` — and their `wire(..., motors(6))` /
  `motors(7)` lines; the file's banner is tagged `A4 register HA-153`, the declarations' comment
  `HA-153`): the ports, the cartridge and the signs (`Forward`, the motor's own forward; `Reversed`
  is the flip, since MECH-A5 — until then a `+6` / `+7` port); the standard Calibrate (robot two's
  since CAL1 Part B, its facts mapped by `src/robots/calibrate_facts.hpp`) drops 6 and 7 from its drive
  candidates on this belief, and every program that boots robot two
  from the record builds both pivots from it (`src/common/boot_source.hpp`, through
  `mechanism::toMechanismConfig`) and names them as claw pivots in its port check; robot two's
  driver file binds R1/R2 and L1/L2 to them by declaration (`src/team/robot_two_driver.hpp`'s
  `whileHeld(clawPivot6, Button::R1, Button::R2, 12_volt)` — the buttons are stated there and
  nowhere else since T4 Part 3), and "shulib Drive" and the competition program run that file.
  `test/mechanism_table_test.cpp` pins the declarations field by field — which tests the TYPING
  against the report, never the report.
  *Confidence:* **reported** by the team lead, 2026-09-26 — not measured. No capture has seen a
  motor on 6 or 7: the 2026-09-24 Calibrate census lists motors on 11–18 and 20 only, and the
  2026-09-25 Teleop capture's port check names only port 14 as a motor outside its record. The
  DIRECTION is not even reported: "forward" is the name of a button, not a measured rotation, and
  the `+` signs are the table's unmeasured starting point.
  *Settle (worksheet Station J):* Bench Tests' DEVICE CENSUS shows MOTOR on 6 and 7; the insert
  colour read off each motor; Calibrate prints `port 6: claw pivot 6 (declared) -- not a candidate,
  not touched` and the same for 7; **the bench page's direction check — Bench Tests' station 13
  (MECH-A6, built 2026-10-30, not yet run), worksheet Station P: a + jog of each pivot at 3 V and
  "which way did it go?", the verdict naming the declaration's line** — then J's direction check —
  hold R1, which way does the port-6 claw go; hold L1, the port-7 claw.
  *Blast radius if wrong:* a wrong PORT leaves that claw ABSENT in "shulib Drive" (the provider refuses a
  port with no motor on it, by name, and the drive still runs) — or, if a DRIVE motor is on 6 or
  7, Calibrate drops that drive motor (the side calibrates one short) and R1 spins a wheel. (A
  calibration RECORD that lists 6 or 7 as a drive port — one taken before Calibrate skipped the
  claws — makes every program that boots from it refuse at boot, "recalibrate", since T3 Part 2.) A wrong CARTRIDGE writes the
  wrong gearset: the volts still reach the motor and the claw still moves; only the motor's speed
  and position scaling — unused by a hold-to-run button, needed by a future angle preset — would be
  off. A wrong SIGN moves the claw opposite to what the driver expects, and is fixed on that pivot's
  declaration — `.motors = {Reversed}` (since MECH-A5; until then the entry's sign) — never by
  swapping its buttons. **Contained:** one table.

- [ ] **HA-154 — a claw pivot held at 12 V against its stop heats, at the motor's default current
  limit, and shulib does not watch it.**
  *Claim:* a claw pivot commanded 12 V while it is pressed against its mechanical stop (the driver
  keeps holding R1 after the claw has closed) draws current up to the V5 motor's current limit —
  PROS's documented DEFAULT, 2500 mA, which nothing in shulib changes while the pivot states no
  `.currentCap` (since MECH-S-A, 2026-11-05, a Power declaration MAY state one and the mechanism
  service then writes it into the motor at boot — HA-219; robot two's pivots state none, so its boot
  reads their limits and writes nothing) — for as long as the button
  is held, and heats; as it heats, the motor's own firmware derates it (HA-44's shape) until the
  claw weakens. A pivot in Hold against a load also draws current, and heats more slowly. **How
  long any of this takes is not known.** False if a stalled pivot barely warms in a minute at 12 V
  (then nothing is needed), or if it reaches the derating temperature within seconds (then a stall
  cut-off is not optional).
  *Source:* robot two's driver file, `src/team/robot_two_driver.hpp` (`whileHeld(clawPivot6,
  Button::R1, Button::R2, 12_volt)` and the same for port 7 — the full power is the DRIVER FILE's since
  MECH-A5; until then the table's `volts = 12.0`, tagged `A4 register HA-154`), and
  `src/robots/mechanism_table.hpp` (the pivots' declarations state no `.stall`, commented HA-154) — the
  ordered full power passes straight through while a button is held; a stall cut-off is OUT of T3 by
  the brief. Since MECH-A2 (2026-10-15) the mechanism service CAN watch it: a power mechanism's optional
  stall signature (`.stall`) watches the driver's hold-to-run and cuts it STALLED, said
  (`include/shulib/mechanism/service.hpp`, `PowerSlot::drive()`, tagged HA-154; host-tested) — but a
  power mechanism that states NONE, as robot two's pivots do in the core's plan, is still cut only by
  the driver letting go — since MECH-A5 robot two's pivots run on the service and state none (its boot
  says so per pivot: `'claw pivot 6': no stall signature -- held against its stop it is cut only by the
  driver letting go (HA-154)`) — and the claim below stands for them. The vendored `include/pros/motors.h` (`motor_get_current_limit`: "The default value is
  2500 mA."). `include/shulib/diag/health_monitor.hpp` raises `MOTOR_OVER_TEMP` from the hottest
  motor "over the drive" — a claw motor is not in it.
  *Confidence:* reasoned (the limit is the vendored default; a stalled DC motor heats) — every
  TIMING is **invented**, and none is written here on purpose: nothing may use one until Station J
  measures it.
  *Settle (worksheet Station J):* close one claw against its stop and hold the button for a timed
  interval, reading the motor's temperature before and after (the claw row — `CLAW 6 HOLD
  34C | ...`, since T3 Part 2; on "shulib Drive"'s DETAILS page since T4 Part 3); stop well below HA-44's 55 °C first step. The row shows the
  motor's LAST GOOD reading — it freezes, unmarked, if the motor stops answering, and reads 20 °C
  if it never answered (`hal/pros/motor.hpp`'s screen; the motor seam has no validity channel) —
  so a reading that does not move while the pivot is held means a cable check first.
  *Blast radius if wrong:* a claw that goes weak mid-match as its motor derates, or a motor that
  cuts itself out hot — and shulib would not say why: `MOTOR_OVER_TEMP` watches the drive only.
  The driver sees a claw that stopped gripping; the claw row SHOWS each pivot's temperature
  (T3 Part 2; its last good reading — see the settle text), but nothing acts on it. **Contained:** one mechanism; the drive is not
  affected.

- [ ] **HA-155 — a cable swapped between a claw pivot and a drive motor is invisible to the
  registry.**
  *Claim:* the brain's device registry reports a port's device KIND — MOTOR — never WHICH motor,
  so a claw-pivot cable and a drive-motor cable swapped with each other at the brain look identical
  to every check shulib runs before power: the census, Calibrate's declared-port drop, the boot
  check's port check against the record, the provider's kind check. False only if the registry could tell
  one motor from another, which its API (HA-138, HA-141) does not offer.
  *Source:* the standard Calibrate's declared-port drop (`include/shulib/calibration/calibrate_program.hpp`,
  over the facts `src/robots/calibrate_facts.hpp` maps from the mechanism list — since CAL1 Part B,
  2026-11-05; robot two's own `src/programs/calibrate_program.cpp`, tagged `A4 register HA-155`, until
  then): it drops PORTS, trusting that the cable in port 6 is the claw's; `src/robots/mechanism_table.hpp`. The
  boot check already says the same of two DRIVE cables swapped with each other ("NOT CHECKED: two
  drive cables swapped WITH EACH OTHER look identical here").
  *Confidence:* reasoned (what the registry reports).
  *Settle:* not by the registry, ever. By a person: label both ends of both claw cables; Station
  J's direction check is the first run that would show a swap (R1 must move the CLAW, not a wheel).
  *Blast radius if a swap happens:* Calibrate skips the drive motor now on 6 or 7 (that side
  calibrates one short) and takes the claw motor now on a drive port as a candidate — geared BLUE
  and set to Coast, so a loaded claw drops, and if the push swings it far enough to count as moved,
  it is taken for a drive motor and PULSED. On a record made before the swap, "shulib Drive" drives the claw
  motor as a member of a drive side — the coupled-side monitor catches that half
  (`MOTOR_GROUP_DISAGREE`: a claw at its stop cannot follow its side) — while R1 drives a wheel at
  12 V, which only a person sees. **Contained:** by labels and Station J; no software can contain
  it.

- [ ] **HA-156 — a claw pivot released in Hold keeps its angle under its load.**
  *Claim:* once DEFECTS2 has made a program-commanded Hold at 0 V reach the motor (its own
  register entries carry that part), a claw pivot released at an angle HOLDS that angle under its
  own weight and a game piece: the sag over 30 s is small enough that the claw still does its job.
  "Holds" is claimed nowhere — in the code, the worksheet or a commit — until Station J logs it.
  False if the claw droops visibly within 30 s, drops its piece, or the motor heats holding it
  (HA-154).
  *Source:* `src/robots/mechanism_table.hpp` (the pivots' declarations, `.safe = Hold`, commented
  HA-156; until MECH-A5 the entry's `safeMode = Hold`, tagged `A4 register HA-156`). The T3
  brief specifies that the library's hold-to-run control applies the safe state ONCE on release
  (never re-applied per tick, which could creep the hold point); that was `teleop::MechanismBinding`
  (T3 Part 1, `include/shulib/teleop/mechanism_binding.hpp`) until MECH-A5, and is the mechanism
  core's `whileHeld` control since (`include/shulib/mechanism/driver_inputs.hpp`, through the
  service): robot two's driver file binds both pivots with it, and "shulib Drive" and the
  competition program run that file — a release is ONE `brake()` under Hold at the PROS shim
  (`test/robot_two_driver_test.cpp`; the T3 bindings and the driver file command the same bytes,
  tick for tick — its MECH-A5 differential case), which says nothing about the load.
  *Confidence:* **invented** — the load, the hold's stiffness and the hold current are all
  unmeasured; the V5's Hold is a position hold whose strength under a claw's load nobody here has
  measured.
  *Settle (worksheet Station J):* move each pivot to a mid angle, loaded with a game piece if the
  claw carries one, release, and measure the sag at 0, 10 and 30 s against a mark; read the
  temperature after.
  *Blast radius if wrong:* a claw that sags or drops its piece after release, so the driver has to
  keep holding the button — and HA-154's heating then applies. **Contained:** one mechanism; the
  fix is a MEASURED one (a holding voltage or a hold stiffness), never a guess.

**T3's addendum (E2, the pneumatic claw, 2026-09-26) added HA-159** (its library half, B2, owns
HA-157 and HA-158). The team lead reported a PNEUMATIC claw the same night: its solenoid on the
brain's 3-wire port A, toggled by the controller's A button, closed at program start, keeping its
state when the robot is disabled. `src/robots/mechanism_table.hpp` declares it; "shulib Drive" drives
its line since T4 Part 3, through robot two's driver file ("shulib Teleop" did from T3 Part 2) — made
at the START level when the Robot is built, then A's toggle — not yet on the robot — and shulib's census cannot show it (it reads the 21 smart ports). Worksheet
Station J.9 is its first run.

**T3's addendum B2 (2026-09-26) added HA-157 and HA-158.** Both are about the brain's own eight
3-wire ports, which the library first USES here — for robot two's pneumatic claw on port A, toggled
by the driver's A button. Neither is about the claw itself (its port, its valve and which level
opens it are the program half's entry, from the team lead's report).

- [ ] **HA-157 — a 3-wire output keeps its level while the brain is disabled, and takes a level
  commanded while it is disabled.**
  *Claim:* two halves. (1) While the V5 brain is DISABLED — the gap between autonomous and driver
  control, a field disable, a competition-switch disable — a 3-wire digital output stays at the
  level the program last commanded, and is still at that level when the brain is enabled again.
  (2) A level the program commands WHILE the brain is disabled reaches the valve — at once, or at
  the latest when the brain is enabled. The second half is not a corner case: `robot::Robot` makes
  each pneumatic line at the config's `pneumaticStart` the moment it is built, and a program builds
  its Robot in `initialize()` ("shulib Drive" does, since T4 Part 3: its `driveInitialize()`;
  "shulib Teleop" did from G1 Part 2b, commit `7f9b9a1`), which on a field — and whenever the program is started from the
  controller's own menu (HA-128, measured: the brain then starts DISABLED) — runs while the brain
  is disabled. False if VEXos drives the brain's 3-wire outputs to any level of its own while
  disabled (as it cuts the motors) or on the way back to enabled, or drops (or holds back past
  the enable) a 3-wire write made while disabled.
  *Source:* `include/shulib/mechanism/service.hpp` (`leaveDriverControl()`: a pneumatic KEEPS its
  level on a disable, tagged HA-157) and `include/shulib/teleop/teleop_loop.hpp` (the disable path):
  driver control commands NOTHING to a pneumatic claw on a disable, on the team lead's rule that the
  claw keeps its state — robot two's claw on that path since MECH-A5 (its driver file's
  `toggle(claw, Button::A)`); until then through T3's `include/shulib/teleop/mechanism_binding.hpp`
  (ToggleBinding, the DISCONNECT/DISABLE rule, tagged `A4 register HA-157`), which keeps the rule for
  its superseded overloads. One consumer of the SECOND half: in "shulib
  Competition", a disable DURING a guarded auton makes the watch abandon the run, and the run guard's
  cancel-all then puts every present claw in its SAFE level while the brain is disabled
  ([guide 17](guide/17-the-competition-program.md), the watch) — a write made while disabled. Since MECH-A2 (2026-10-15) the mechanism
  service keeps the same rule for every pneumatic it runs (`include/shulib/mechanism/service.hpp`,
  `leaveDriverControl()` and `enterAutonomous()`: a pneumatic KEEPS its level, tagged HA-157). `include/shulib/robot/robot.hpp` (the
  MECHANISMS section and `buildPneumatic()`, tagged `A4 register HA-157`): the start level is
  written when the Robot is built, whatever the brain's state. `test/robot_two_driver_test.cpp` M5a
  pins that robot two's claw is commanded nothing on a disable (line A's every write identical to T3's
  ToggleBinding through a field disable and two re-entries) — `test/toggle_binding_test.cpp` TL3 the
  same for T3's superseded toggle — and
  `test/pros_device_provider_test.cpp` 1m
  that construction writes the start level once; no host test can pin what the brain does with
  either.
  *Confidence:* **UNKNOWN.** The vendored PROS headers document the 3-wire output's constructor
  and `set_value()` only (vendored `adi.hpp` ~:530-615) and say nothing about a disabled brain.
  VEXos is known to stop the MOTORS while disabled; whether it treats a 3-wire output the same
  way — its level, or a write to it — is exactly the open question. No solenoid has ever been on
  either robot.
  *Settle:* (1) at Station J (robot two's claw — J.9.6): close the claw, disable (the field
  control / Timed Run disable, or a competition switch), and watch it; then open it, disable, and
  watch again; re-enable and press A once. Record whether the claw moved on the disable, whether it
  moved on the re-enable, and whether that first press did the visible thing. (2) at COMP's
  station (the competition program's bench run), NOT Station J — Station J never starts a program
  from the controller's menu (its DO NOT list; HA-128), and COMP's station does so on purpose,
  through the controller's Timed Run, because the team has no competition switch (the coordinator's
  placement at T3's merge, 2026-09-26): leave the claw at the OTHER level from the start level (the
  run before ends with it there), start the program that way — the brain then comes up DISABLED,
  so `initialize()` builds the Robot and writes the start level while disabled — and watch: does
  the claw move at the start (the write reached the valve while disabled), only when the robot is
  enabled, only when driver control begins (the toggle's own initial level), or never? A bench
  start from the laptop runs enabled and cannot answer (2).
  *Blast radius if wrong:* (1) the claw moves by ITSELF on every disable — if that is open, it
  drops what it holds — and because the adapter still reports the level it last commanded, the
  first press after the re-enable commands the level the line already has: the driver sees a
  press that does nothing and has to press again. (2) The claw's FIELD-START state rests on it:
  if a write made while disabled is dropped, the claw sits wherever its valve rests from program
  start through autonomous — a preload it should grip is not gripped — until driver control
  starts and the toggle sends its initial level (which the program states equal to the start
  level); if the write is held back to the enable, the claw moves at the enable, on the field.
  **Contained:** one rule in the toggle and one line in the loop's disable path for (1) — the fix
  (re-assert the commanded level when driver control re-enters) is a small, testable change
  there; for (2), the fix is to command the start level again at the first ENABLED moment (the
  top of `autonomous()` and of `opcontrol()`), a program-side change.

- [ ] **HA-158 — nothing the program can read sees a solenoid on a brain 3-wire port.**
  *Claim:* the brain's device registry (`registry_get_plugged_type()`) is indexed by SMART port —
  indices 0..20 (HA-138) — and the brain's own eight 3-wire ports are served by its internal ADI
  (PROS's `INTERNAL_ADI_PORT`, 22, vendored `adi.h:136`), outside that range; a solenoid valve on
  one of them is a passive load with no digital identity, and a digital output has no read-back
  (hal/digital_out.hpp: no feedback, no validity). So a connected valve and an empty port are
  indistinguishable to the program. False if some API — a registry kind, an ADI read, a current
  sense — reports what is plugged into a 3-wire port.
  *Source:* `include/shulib/hal/device_provider.hpp` (the header's "A 3-WIRE LINE IS THE
  EXCEPTION"), `include/shulib/hal/pros/device_provider.hpp` (`digitalOut()` asks no registry,
  tagged `A4 register HA-158`; since MECH-A1, 2026-10-08, `digitalIn()` — a limit switch — asks none
  either, tagged HA-158), `include/shulib/hal/fake/fake_device_provider.hpp`;
  `test/pros_device_provider_test.cpp` 1l (the line is made with the registry reporting NONE on
  every port, and the registry is never asked).
  *Confidence:* **reasoned** — from the registry's documented index range, the internal-ADI port
  number, and the 3-wire output's documented API, which has no read of anything but the level the
  program set.
  *Settle (Station J — a step the station does not have yet; the coordinator adds it to J.9, whose
  J.9.0 only LOOKS at the valve's cable and whose J.9.5 pulls the CONTROLLER's):* with no hand in
  the claw, pull the valve's 3-wire cable out of port A and start "shulib Drive" (slot 1; slot 2's
  "shulib Teleop" until T4 Part 3) — the program must build
  the claw exactly as it does with it plugged in (the same boot line for the claw, no refusal, no
  difference on the screen) — and look at the brain's own Devices screen for any entry for the
  valve, plugged and unplugged. Confirming the belief means seeing NO difference. Plug the cable
  back in before anything else in J.9.
  *Blast radius if wrong:* in the safe direction — if something CAN see a solenoid, the provider
  could refuse an empty 3-wire port and does not, which is a missed check, not a wrong action. If
  RIGHT (the expected case), an unplugged or mis-plugged valve is invisible to every check in the
  library: the claw simply does not move, and the driver's eyes are the only tell — which is why
  the station tests it by pulling the cable. **Contained:** one provider member.

- [ ] **HA-159 — robot two's claw solenoid is on 3-wire port A; single or double, and which level
  opens the claw, are NOT KNOWN.**
  *Claim:* robot two's pneumatic claw is moved by a solenoid valve wired to the brain's 3-wire
  port **A** (line 1) through ONE cable, and commanding that line HIGH — the claw's `.positions`
  line, `{{"closed", {Low}, grabs}, {"open", {High}}}`, the addendum's starting guess, "ON = open"
  — OPENS the claw, while LOW CLOSES it. False if the valve's cable is in another lettered port; if the valve has TWO
  cables into the brain (a double solenoid, whose second line nobody has declared); or if HIGH
  closes the claw.
  *Source:* `src/robots/mechanism_table.hpp` (robot two's region: the declaration `claw` and its
  `wire(claw, lines('A'))`; the file's banner is tagged `A4 register HA-159`, the declaration's
  comment `HA-159`): line A, the polarity written ON the `.positions` line (closed = `Low`, open =
  `High`) and called a GUESS beside it, the start and safe positions the NAME "closed" — so they
  follow the levels. (Until MECH-A5, 2026-10-29, the polarity was one named constant,
  `kClawOpenCommand = true`.) `test/mechanism_table_test.cpp` pins the declaration against the report
  AND the two levels, as it pins the pivots' signs: the flip Station J.9 may call for is ONE line of
  code (swap `Low` and `High` on the claw's `.positions` line; the start and safe levels follow the
  names), and it is a measurement — so that line, that pin, this entry (its claim and its index row)
  and Station J.9's quote change together, in one change.
  *Confidence:* **reported** by the team lead, 2026-09-26 — the PORT only ("port A"). Whether the
  valve is single or double, and the polarity, are **unknown**: the team lead said so, and `true`
  is a guess recorded as one. Nothing has seen the claw move under a program: no program that drives
  its line has run on the robot — "shulib Drive" (since T4 Part 3; "shulib Teleop" from T3 Part 2) and
  "shulib Competition"'s driver control (robot two's driver file), and since MECH-A6 (built 2026-10-30)
  Bench Tests' station 13, which makes the line at its start level and fires it only on a confirm
  (worksheet Station P) — shulib's census
  reads the 21 smart ports only, and nothing in shulib
  can ask the brain what is on a 3-wire line (the library half's register entries carry what the
  brain can and cannot report there).
  *Settle (worksheet Station P, P.5 — Bench Tests' station 13, FIRST; then Station J.9):* P.5 fires `open` on a
  confirm, L1 held, and asks which position the claw went to: `OK` ⇒ the polarity is right; `REVERSED` (it went
  `closed`) names the `.positions` line to swap; `CABLE ELSEWHERE` (nothing moved) is no answer about the polarity
  (the air, the cable, the port, the valve). Station J.9 then checks it from the controller. J.9.0 — look at the valve: one cable or two into the brain,
  and which letters (one cable NOT in A stops the station: the port is wrong, not the polarity);
  J.9.2 — where the claw is after "shulib Drive" (slot 1) starts; J.9.3 — the FIRST press of A decides, with one
  cable in A: CLOSED after the start and the press OPENS it ⇒ the polarity is right; OPEN after
  the start and the press CLOSES it ⇒ it is backwards (swap `Low` and `High` on the claw's
  `.positions` line, the only code edit, and record it — the test's pin, this entry, J.9); the press moves NOTHING ⇒ no answer
  about the polarity (the air, the cable, the port, the valve). Then A opens it, A again closes
  it.
  *Blast radius if wrong:* a wrong POLARITY opens the claw at program start — constructing the
  3-wire output drives the line at once (HA-119) — so a preloaded piece drops as the program
  starts; every press still flips the claw, so the driver sees the button "backwards"; and a
  library-forced safe state (COMP's end-of-run guard, later) would OPEN the claw instead of keeping
  what it holds. Fixed by swapping `Low` and `High` on ONE line. A wrong PORT leaves the claw unmoved by A while
  the program drives a line with nothing (or something else) on it — and no software can see
  that: a 3-wire output has no feedback channel (`hal/digital_out.hpp`). A valve with TWO cables
  gets only the one in port A driven, never the other; what the claw does then is not known, and
  J.9.0's look is what catches it before power. **Contained:** one mechanism; the drive is not
  affected.

**COMP-a (2026-09-26) added HA-161.** The competition program (COMP-b) runs its autonomous routine on
a task it creates itself, and ends that routine itself — through `RunGuard::requestAbandon()` — when
the field disables the robot or leaves autonomous (the team lead's ruling of 2026-09-24: "our own
auton task"). That design, and code already in the tree, rest on a belief about PROS that no
vendored header states. It was registered at COMP-a because COMP-a built the library half of the
design; COMP-b's practice match settles it. (The number was COMP-a's reservation, taken while T3 and DIAG1
were built in parallel; both have since landed, with the numbers below it.)

- [ ] **HA-161 — PROS deletes its competition tasks on a mode change, and leaves the program's own
  tasks alone.**
  *Claim:* (a) PROS runs each of `competition_initialize()`, `disabled()`, `autonomous()` and
  `opcontrol()` on a task of its own and DELETES that task when the competition mode changes (the
  field disables the robot, or autonomous ends and driver control begins) — mid-statement, with no
  unwinding: a deleted task runs no destructor, releases nothing and finishes no loop. (b) A task
  the PROGRAM created (`pros::Task`) is not deleted on a mode change — it keeps running, and must
  end its own work.
  *Source:* half (a) is written, as a belief, in code comments only:
  `include/shulib/teleop/teleop_loop.hpp` (step 1: "PROS deletes the opcontrol task on a field
  disable, and a deleted task runs no destructor", so the driver loop polls the disable and
  returns, braking the drive before the task is torn down — the loop the X-drive runs too, since G1
  Part 2c moved it out of `src/main.cpp`, whose own X-drive loop said "PROS deletes the task on disable
  and a deleted task runs no destructor" until then; corrected at T4 Part 3's review, 2026-10-08),
  `src/programs/calibrate_program.cpp` until CAL1 Part B ("A FIELD DISABLE is NOT a C++ exit — PROS
  deletes the opcontrol task and a deleted task runs no destructors", for its all-stop guard; the
  standard Calibrate, `include/shulib/calibration/calibrate_program.hpp`, says it since) — the old
  `src/programs/drive_program.cpp` said it nearly word for word for its own guard until T4 Part 3
  replaced that program; the new one says it for its storage: the driver file is held in the
  program's static state, never on the opcontrol task's stack, "because the loop holds the inputs by
  address and PROS deletes the opcontrol task on a field disable" — "the bindings" until MECH-A5),
  `src/common/driver_controls.hpp` (deleted at MECH-A5, 2026-10-29; its Storage note: the bindings
  lived in static storage because PROS "runs no destructor" — the rule stands in `src/common/driver_file.hpp`, whose DriverFile and the
  mechanism service the programs hold in their static state), and `include/shulib/robot/robot.hpp` ("PROS calls opcontrol() again after
  every disable"). Two rules rest on half (a) as well: a claim a deleted task left must never lock a
  mechanism — T3's claw bindings took no claim for that reason; since MECH-A5 the claws run through
  the mechanism service, whose slots DO take their mechanism's claim while a step or a hold runs, and
  each mode change ends every slot's behaviour and releases its claim, and releases (and says) a
  claim ANOTHER owner left (`enterDriverControl()` / `enterAutonomous()`, `include/shulib/mechanism/
  service.hpp`) — and T3's static-state rule — a driver program ("shulib Drive" and the competition
  program since T4 Part 3; "shulib Teleop" until then) keeps its driver file (its bindings until
  MECH-A5, its inputs since), its mechanism service, its mechanisms and their `IMotor*` arrays in its
  static state, never on the driver task's stack, which a deleted task would leave dangling. Half (b) is what COMP's own-task design rests on
  (`competition/match_runner.hpp` runs on whichever task calls it; the program's task and its
  handshake are COMP-b's). **The Auton Tester (COMP-t, 2026-09-28) rests on half (b) too:** after
  boot it does everything — the idle ticks, the countdown, every run — on a `pros::Task` it creates
  in `initialize()` (`src/programs/auton_tester_program.cpp`), and `src/main.cpp`'s `disabled()`,
  `competition_initialize()`, `autonomous()` and `opcontrol()` return at once for it; on the bench
  nothing changes mode (its RUN gate refuses any competition bit), so the tester alone never tests
  (b). **The competition program (COMP-b, 2026-09-30) rests on both halves:** its auton task
  (`src/programs/competition_program.cpp`, created once in `initialize()`) serves every
  `autonomous()` request and ends a live auton itself, through its watch and the guard's abandon,
  when the robot is disabled or leaves autonomous — half (b); and its `autonomous()` only asks and
  waits, its driver loop ends on the autonomous bit and on a waiting request as well as on a
  disable, and the auton task does not serve a request while a driver loop holds the robot — not
  until that loop has LEFT, or its once-a-tick heartbeat has stood still for 0.1 s (a loop PROS
  deleted mid-tick never leaves: `DriverLoopPresence` in `src/programs/competition_run.hpp`, added at
  COMP-b's review, 2026-10-01) — so neither depends on being deleted at the mode change: half (a)'s
  failure is contained, not relied on. One REASONED margin is left: a driver-loop tick that took
  longer than 0.1 s while a request waited would be taken for a deleted loop. The vendored `include/pros/misc.h` (`competition_get_status`,
  `competition_is_disabled`, `competition_is_autonomous`, `competition_is_field`,
  `competition_is_switch`) says nothing about tasks.
  *Confidence:* **reasoned** — both halves are how PROS's competition template is generally
  understood to work, and half (a) is the belief this tree's code comments have carried since R3b
  Part 0b; neither half is stated in the vendored PROS text, and neither has been observed directly
  on this project's brain (R3b Part 0b's "finding" was a correction to a brief's REASONING — a
  deleted FreeRTOS task does not unwind its stack — not an observation of what PROS does).
  *Settle (COMP-b's practice-match station, K — from the competition program's OWN lines, no probe
  build; the claim is unchanged):* during the mid-auton disable step, with the auton still RUNNING
  when the mode changes, read the serial log in its order. The change's own mark is the NEXT phase
  function's `entry` line (`disabled() entry at <ms>` or `opcontrol() entry at <ms>`): PROS starts
  that function at the change and it logs that line first. **(a)** is confirmed if `autonomous()`'s
  `normal return` line NEVER appears for that run (an `autonomous()` PROS did not delete would log
  it within a tick of its request's terminal word, which the abandon publishes 1–3 ticks after the
  change); a `normal return` logged AFTER the next phase function's `entry` line refutes it; one
  logged BEFORE that line is AMBIGUOUS (the watch's abandon finished the run before PROS acted on
  the change) and settles nothing. **(b)** is confirmed if the run's `request N:` lines appear at all
  — the auton task writes them only after the run has ended, so after the change — and its
  `competition status ... at <ms>` lines continue at later changes. The station states one limit:
  the routine must still be running at the change — an auton that finished before it logs a normal
  return legitimately, so this rides on the mid-auton disable, never on an ordinary run. (The auton
  task's own status-change line cannot mark the change: it does not log while it serves a run, so a
  change during a run is logged after the run's lines, and only when the status then differs from
  the last one it logged — COMP-b's review, 2026-10-01, which moved the anchor from RC2's wording.)
  For `opcontrol()`, a request line `a driver loop still held the robot when it was served -- it
  LEFT` (logged only after a change straight from driver control to autonomous) shows a driver task
  PROS did NOT delete at that change; `its heartbeat STOPPED` is consistent with one it did.
  *Blast radius if wrong:* if (a) is wrong (PROS does not delete its tasks), an `autonomous()` that
  kept running would command the drive alongside driver control — COMP-b's design does not let it
  (the auton runs on the program's own task, which `opcontrol()` waits for, and a driver loop that
  survived a change straight to autonomous ends at its next tick and leaves before the auton task
  serves), so the damage is contained to the loops that assume the kill. If (b) is wrong (PROS deletes the program's tasks
  too), a routine could be killed mid-motion, leaving the scheduler's active motion on a freed
  stack — the failure the own-task design exists to avoid; the recovery would be a disable-aware
  pacer that ends the routine before the mode change lands, which the abandon latch already is.
  **Contained:** the program's task body and its watch pacer (COMP-b).
  *A consequence if (a) is RIGHT (T4 Part 3's verification, 2026-10-08):* the SD copy of a program's
  log (`diag/log_copy_sink.hpp`) lets one task at a time write the card, and only a task that
  RETURNS from its copy releases the guard. Both driver programs log a line right after their loop
  sees the disable ("shulib Drive"'s `opcontrol(): field DISABLED ...`; the competition program's
  `driver control ended` and `normal return`) — the moment (a) says PROS deletes the task. A deletion
  that lands while that line is being copied leaves the guard held, so the card's copy ENDS for the
  rest of that boot: every later line is counted and stays on serial, and no gap line is written.
  That is the chosen failure, not an oversight. A released guard would hand the next writer — the
  driver loop included — whatever the deleted task left half-done in newlib's FILE or the card's
  driver (unmeasured), and that writer could hang. How often the deletion lands inside a copy is
  unmeasured; worksheet F.7.2 and K.7.2 compare the card with serial at each disable. No new
  belief is added here. A guard that recovers from a dead holder would add one: that PROS's
  `task_get_state()` reads DELETED for a deleted task's handle, never the state of a later task built
  in its memory. That is left for the coordinator, with the design that would remove the case: one
  card-writer task that alone touches the file.

**COMP-b (2026-09-30) added ONE — HA-162, its reservation (the number was held for it since the COMP
brief's re-check of 2026-09-28).** The team owns no competition switch, so the bench's only way to run
the competition program through a pretend match — disabled, autonomous, driver control — is the
controller's own Timed Run. HA-128 measured how such a start BEGINS; nothing has measured what it does
next, and the bench's practice-match station is written around that measurement.

- [ ] **HA-162 — a controller Timed Run enables `autonomous()` after a disabled window, for a set time,
  then moves on.**
  *Claim:* a program started from the controller's own program menu with **Timed Run** comes up
  DISABLED in a pretend field-control match (HA-128, measured: status `0x0f`), and VEXos then ENABLES
  `autonomous()` after a disabled window of __ s, for __ s, then __ (a disable, driver control for
  __ s, or the end) — every blank UNMEASURED.
  *Source:* "shulib Competition" (`src/programs/competition_program.cpp`) is started this way at the
  bench's practice-match station; the auton is chosen on its menu during the disabled window, and
  its `autonomous()` runs only if the Timed Run enables it. The worksheet's rule "never start a
  program from the controller's menu" carries ONE exception for it (slot 6, at that station); the
  guide's chapter 17 says what is measured and what is not.
  *Confidence:* **unknown** — no vendored PROS or VEXos text describes the controller's Timed Run;
  HA-128 measured only its start.
  *Settle (Station K's K.3, its FIRST match step — after K.2's brain start, which only reads the boot
  and leaves the claw open for K.4):* start slot 6 with Timed Run and copy the program's own lines:
  each phase function's `entry` / `normal return` lines (the brain's milliseconds and the decoded
  status) and the auton task's `competition status ... at <ms>` lines give the disabled window, the
  autonomous period and what follows, to the tick. If Timed Run never enters `autonomous()`, record
  that and try **Match** from the same menu. The program's periods (30 s / 60 s, the game manual's)
  never come from this measurement.
  *Blast radius if wrong:* the bench procedure only. No disabled window before autonomous means no
  auton can be chosen on the bench before it starts (NONE runs: nothing moves — the safe failure),
  and how to choose one then is the team lead's to decide after the measurement; a Timed Run that
  never enters autonomous means the station needs Match or a field. A real field drives the modes
  itself. **Contained:** the practice-match station; the program's behaviour does not depend on the
  numbers.

**SAFE-SAVE Part A (landed 2026-10-04) added ONE — HA-166.** "shulib Calibrate" saved the calibration record
by opening it with `fopen "wb"` and writing it, and printed, after any failure, that the earlier record
"is still on the card". On this brain `"wb"` reaches VEXos's create-a-file call, which is taken to empty
an existing file at the OPEN (belief (1) below — reasoned from the vendored libraries, not measured), so
every failure after it is taken to leave no record at all (found 2026-09-26). The obvious fix — write a
temporary file and rename it over the old one — cannot be built here, because the brain has no rename.
The library's safe save (`calibration/record_store.hpp`) writes a BACKUP copy first, reads it back, only
then replaces the record, and measures both files at the end; this entry is every belief about the card
that design rests on.
Nothing of it has run on a robot: "shulib Calibrate" does not use it until SAFE-SAVE Part B1.

- [ ] **HA-166 — what a program can do to a file on the brain's SD card, and what the safe save
  assumes about it.**
  *Claim:* six beliefs that the calibration record's safe save is designed around.
  1. `fopen(path, "wb")` under `/usd/` on an **existing** file succeeds and empties it **at the open**.
     PROS's `usd_open_r` sends a write-only open without `O_APPEND` to `vexFileOpenCreate`; `O_CREAT` and
     `O_TRUNC` are never consulted, so what happens to an existing file is VEXos's. (A write-only open
     WITH `O_APPEND` — `"ab"` — goes to `vexFileOpenWrite` instead. What that call does to a file is
     VEXos's and is not a belief of this entry; the safe save does not use it, since it needs a
     whole-file replace and an append is not one.)
  2. There is **no rename and no delete.** newlib's `rename()` is `_link_r` followed by `_unlink_r`, and
     `remove()`/`unlink()` are `_unlink_r`. PROS 4.2.2's `file_system_stubs.c` defines `_link`, `_unlink`,
     `_stat`, `mkdir` and `chdir` (and `chmod`, `fchmod`, `fchmodat`, `pathconf`, `symlink`, `readlink`,
     `getcwd`) as `errno = 88 (ENOSYS); return -1`, and no other vendored archive defines `_link` or
     `_unlink`; the VEXos jump table exports no delete and no rename either. A program also cannot make a
     directory, and cannot ask `stat()` whether a file exists — an `"rb"` open answers that.
  3. A read-write open (`"r+b"`, `"w+b"`, `"a+b"`) fails with `EINVAL` (22): `usd_open_r` accepts only
     read-only or write-only, so a file cannot be updated in place.
  4. A power cut, a brownout or a card pulled while ONE file is being written leaves every OTHER file
     readable with its previous bytes. (FatFS writes the FAT and a directory sector for the file being
     written; two files' directory entries can share one 512-byte sector; whether the card tears a sector
     write is the card's.)
  5. A file whose bytes went through `fflush`, whose writer closed it, and whose read-back matched is **on
     the medium** — its data, its FAT chain and its directory entry — before the program's next open of
     another file, and a read-back reads what the medium holds. PROS's `usd_write_r` passes each write to
     `vexFileWrite` and then calls `vexFileSync`, ignoring what the sync returns; `usd_close_r` calls
     `vexFileClose` and returns 0 whatever it answers. So a sync is CALLED after every write the driver
     passes on; what `vexFileSync` commits (FatFS's `f_sync`, or less), and whether the card has a write
     cache of its own, is VEXos's and the card's.
  6. Two names that differ only in **letter case**, or in **trailing dots and spaces**, name ONE file on
     the card: `"gains.bin"`, `"GAINS.BIN"` and `"gains.bin."` are the same file, while Linux — and so
     every host test — holds three. That is FAT's rule (long names match without regard to case; FatFS
     drops trailing dots and spaces when it parses a name); that VEXos's file system follows it is
     invented. A FAT short-name alias such as `SHULIB~1.BIN` is not covered: no caller names a file
     that way.
  *Source:* the vendored `firmware/libpros.a` (`file_system_stubs.c.o`; `usd_driver.c.o`: `usd_open_r`,
  `usd_write_r`, `usd_close_r`; `v5_apijump.c.obj`'s `vexFile*` exports) and `firmware/libc.a`
  (`libc_a-renamer.o`, `libc_a-remove.o`, `libc_a-unlinkr.o`), read with `arm-none-eabi-objdump -dr` on
  2026-09-26. The design that rests on it: `include/shulib/calibration/record_store.hpp` (the save, the
  load, the report) and the store seam `include/shulib/hal/file_store.hpp`, which has, on purpose, no
  rename, no delete and no read-write open; `include/shulib/hal/pros/file_store.hpp` (the adapter) and
  `include/shulib/hal/fake/fake_file_store.hpp` (the host model, which ASSUMES (4) and (5) and says so).
  `test/record_store_platform_scan_test.cpp` scans the library and `src/` for rename, remove, unlink,
  link, stat, mkdir, freopen, fdopen, openat, creat, an `#include` of `<fcntl.h>`, `<fstream>` or
  `<filesystem>`, any `fopen` outside the two SD adapters and Bench Tests' probe, and any `+` mode. For
  (6): `hal::sameFileOnCard()` in the store seam, which the save's and the load's preconditions and the
  record's own two names (a compile-time check) go through.
  *Confidence:* (1)–(3), and the sync call in (5), are **reasoned** from the vendored binaries. **The host
  test cannot see them:** on Linux `std::rename` and `std::remove` work, so a rename-based save would pass
  every host test and fail on the brain — this entry, and the scan, exist so nobody simplifies the save
  back to a rename. (4), and what the sync in (5) commits, are **invented**. (6) is FAT's documented
  rule; that VEXos applies it is **invented**. None is measured.
  *Settle:* (a) Station G, once "shulib Calibrate" saves through the library (SAFE-SAVE Part B1): after a
  SAVED run, on a laptop, `shulib_drivetrain.bin` and `shulib_drivetrain.bak` both exist, 180 bytes each,
  byte-identical (`cmp`) — which settles (1)'s success half for these names and the `.bak` file name.
  (b) (2) and (3) by extending Bench Tests' existing SD probe (button 5, `probeSdCard` in
  `src/programs/bench_r3a.cpp`) to call `rename`, `remove`, `mkdir` and an `"r+b"` open on its own scratch
  file and print each return value and `errno`, and (6) the same way — write its scratch file, then try
  to read it back under the same name in capitals and with a trailing dot — unowned; the design holds
  either way. (c) (4) and (5) need a deliberate power cut during a save and right after one, then both
  files read on a laptop — unowned.
  *Blast radius if wrong:* (1) the other way — an existing file refuses `"wb"` — every re-save says NOT
  SAVED and the record file is untouched: loud. (2)/(3): nothing breaks; a simpler design was available.
  (4) or (5): a power cut in the save's few milliseconds could damage BOTH files, and the boot then
  REFUSES by name (CRC or length) — or leave a file that decodes but is not the one the save last read
  back (an older record), which the boot would use. So the boot never drives from a DAMAGED record; it may
  drive from the record in effect before the save, or an older backup copy (amber BACKUP once SAFE-SAVE
  Part B2 lands). (6) the other way — VEXos telling `a.bin` from `A.BIN` — nothing breaks: the check
  refuses a few name pairs it did not need to. Short of the limit below, the save's report also never
  claims more than it measured: a card that is gone, refuses every open, or does not read back the same
  way twice is reported SAVE UNCONFIRMED. **What the design does not defend** takes two faults in one
  save, the first a misread at the save's first reads that its checks cannot catch, and the second a
  failed write or a power cut. The misread is either the record file read as damaged at BOTH of its first
  two reads, or a read that returns a VALID record the file does not hold — a stale read (HA-139): the
  record file read as some other record than it holds (it is damaged, missing or different), or the backup
  (damaged, older or missing) read as identical to the record. That can lose the record, or leave the next
  boot a record other than the one in effect (an older one). The save's words rest on what its first reads
  saw: after the first kind they say the record file "did not read as usable"; after a stale read they
  call the record it returned "the earlier record", and can say it is still on the card. A single misread
  the checks CAN see — refused, short, or failing the CRC or the length check — followed by a crash or a
  write fault IS defended, and the host tests enumerate it (the record file is read again when its first
  read does not decode — added after the review of 2026-10-01). **Contained:** two files, one program's
  save, one boot read.

**TANK1 (2026-09-27) added TWO — HA-179 and HA-180.** A tank `moveTo` now steers back onto its line
while it drives (the changelog's API 2.3 entry; `motion/tank_steer.hpp`, `motion/move_to_pose.hpp`): the
heading it servoes to is bent toward the target's line, and the yaw the robot makes that the heading loop
did not ask for — a weaker side's pull — is measured and cancelled. Every constant in it was tuned on the
host test plant, whose motors respond instantly and whose tank never skids; these two entries are what
that leaves unproven. The numbers below are host-test-plant measurements of 2026-09-27, re-measured on
2026-09-28 after a review added the bend's lead; **none of it has run on a robot.**

- [ ] **HA-179 — TANK1's steering constants hold on robot two.**
  *Claim:* with a bend of 0.1 rad per inch off the line capped at 15°, steering on where the robot will
  be 0.06 s ahead, a 6 in fade, a 0.1 s yaw observer and a 1.5 rad/s correction limit, robot two's tank
  `moveTo` removes the drift of a side up to about 10 % weaker than the other before arrival, inside the
  unchanged 0.5 in / 0.02 rad arrival rule, without heading oscillation, and without making a leg that
  arrives with the steering off time out with it on — which requires the drivetrain to follow a yaw
  command much faster than 0.1 s (its motor time constant, kA/kV, well under about 0.2 s).
  *Source:* `include/shulib/motion/motion_config.hpp` (`TankSteerConfig`, every field PROVISIONAL (A4:
  HA-179)); `include/shulib/motion/move_to_pose.hpp` (the tank path, labelled HA-179). Host test plant,
  instant motors: the 40 in leg settles with one side up to 25 % weak (1.83–2.57 s, within 0.1 in of its
  line), the acceptance run finishes with either side 9 % weak, and a faced leg removes an offset from its
  line of up to 6 in over 40 in, 8 in over 50 in, about 1 in over 20 in. **The lead** (steering on the
  error 0.06 s ahead): without it the line loop was under-damped — a leg starting 10–15° off its target
  heading, turned toward its line, crossed the line near its target and timed out half an inch beside it
  (178 of the 1 861 legs of a 24 563-leg grid that arrive with steering off), and the lateral reach had
  holes (over 50 in, 4–6 in timed out while 7–8 in arrived). With it: none of them, at plant lags from 0
  to about 0.18 s; every lead from 0.05 to 0.10 s measured did the same, 0.04 s left 4, 0.25 s left 72–102
  under lag. **With a motor lag the controller does not model** (the plant's kA raised, the controller's
  left at 0, as R5a will leave it): at a time constant of about 0.12 s and about 0.18 s the 40 in leg still
  finishes with one side up to 25 % weak (1.37–1.82 s and 2.09–2.42 s; 30 % times out at every lag, as
  without lag), and a 6 in offset over 40 in is still removed (1.45 / 2.39 s); a 10 in one is not (1.6 in
  off the line at 0.12 s). **At about 0.24 s** 24 of the 1 867 grid legs that arrive with steering off
  time out with it on (all 48 in long, starting 10–14° off their heading) — the heading loop's own kP 4 is
  the limit there. Near-target yaw-command sign changes stay small: 0 on every drift case without lag;
  at most 4 per arriving leg at 0.12–0.18 s (7 and 11 on the 12 in offset that times out).
  *Confidence:* **invented** — tuned on the host test plant's placeholder dynamics (the HA-45/HA-50
  co-dependency: wrong kV or kA changes what every one of these numbers costs).
  *Settle:* COMP-t's floor run of the 40 in auton (its Station M.6: the RESULT, END y, how many ticks show
  TSTR, and the tape's sideways offset), and R5, whose characterization records kA (R5a) and so the motor
  time constant. **When that is known, re-run the host probe grids with the measured kA** — the review's
  24 563-leg grid for "no leg that arrived before times out", and the design sweep: the lead's band
  (0.05–0.10 s measured regression-free at 0–0.18 s) was chosen for an unknown lag, and the right
  `leadTime` grows with the robot's real heading lag; that choice is R5's. The same grids re-derive how
  often TSTR lights if R5a moves `maxAngularSpeed` (the flag's floor is 2 % of it).
  *Blast radius if wrong:* too weak — the drift is not removed, and the leg ends as it did before TANK1: an
  honest MOTION_TIMEOUT (bounded, visible, and the tick lines show TSTR to the end). Too strong, or a
  drivetrain too slow for it — heading wobble on steered tank legs, bounded by the 15° cap and the
  1.5 rad/s limit and visible as TSTR; at a stop a weak side's static friction can leave a small commanded
  yaw it does not follow. **While the robot cannot follow a yaw command** (pinned by a defender, wedged on a
  wall, motor lag, both sides saturated) the correction integrates the heading error, up to 1.5 rad/s, and
  unwinds within about 0.1 s once the robot turns again: measured, a 0.5 s pin of the 9 % weak 40 in leg
  still arrives (2.41 s, peak correction 0.62 rad/s) and so does a 1.0 s pin (3.11 s, the correction at its
  1.5 rad/s limit), and a 6 in offset leg pinned during its bend swings to 16.5° off its heading on release
  (19° with 0.18 s of lag) and still arrives (before the lead it timed out). **The cap is also the engage window:** raising
  `maxSteer` to remove larger offsets also steers un-faced legs further off — with no window at all an
  un-faced 40 in leg arrives up to 30° off its nose and times out from 40°, a cliff an author cannot see —
  so keep it well under 30°. **Since CURVE1 (2026-11-11) the curve verbs share it:** every curve runs the
  same yaw observer (CurveFollow's copy of its step, pinned bit-equal by `test/motion_curve_test.cpp` CV14)
  and ends in a TANK1-steered `moveTo`, so a wrong constant here reaches every curve too. **The IMU read:** the observer uses the rate the Localizer read that tick
  (`loc.twist()`). *(Rewritten 2026-10-29, IMU1 Part A.)* Before IMU1 that rate was NOT the tick's clean
  first read on robot two — DERIVED from the adapter's code and PROS's documented 10 ms copy (HA-213); no robot
  log has shown it (IMU1's brief §0.5): every motion tick's record read `ProsImu::yawRate()` a second time (robot two's
  serial sink wants every record), that read moved the adapter's differentiation baseline, and the NEXT
  tick's first read — the Localizer's — was about 3 % high, or, for one band of the IMU's copy phase
  (HA-213, HA-214), 0 on EVERY tick: measured on the host test plant with TANK1's own motor lag (kA 0.02 /
  0.03), the 40 in TIMED OUT where one reader finishes (IMU1's brief §0.4, probe 2; its T8 reproduces it —
  the Localizer's ω exactly 0 on 486–487 turning ticks). TANK1's "immune" (its completion record) rested on a
  model of the second read that left this out. *(TANK1's own model, kept as it was written 2026-09-28: on a
  robot with a GPS or tag corrector, until IMU1 lands, the corrector reads first on the ticks it reaches its
  yaw-rate gate and the Localizer's rate is about 0 (or a spike) there — modelled on every other tick: with
  zeros 15 of 15 probe cases still arrive, with zeros and a 3 % ×50 spike 8 of 15. The Localizer also reads the
  rate a second time on the first tick of every chained motion; the observer never runs on a motion's first
  tick.)* **Since IMU1** `ProsImu` answers once per tick, so every
  reader gets one value — the record, a corrector's gate, the second `update()` at a chained-motion boundary
  included — and records on and off are bit-identical through the 40 in and the acceptance run on the host
  test plant (IMU1's T8; one narrow band at a chained-motion boundary excepted, where a later HEADING read
  sees a newer copy). **On a GPS- or tag-corrected robot a residual remains** (the hold's limit (2),
  HA-214): when the IMU's copy lands between the gate's read and the Localizer's, the pair of ticks around
  each change between a fresh-fix tick and a stale one reads 0 and about double — the observer's input
  included (IMU1's T9 (b), a fix every 5th tick: the Localizer exact on 180, 0 on 60 and about double on 60
  of 300 ticks). No steering run has been measured through it; TANK1's model above (zeros on every other
  tick) is the nearest. The cure, the Localizer reading its rate at the top of `update()`, belongs to the
  first corrected robot's chunk (IMU1's brief §5); no robot is GPS- or tag-corrected this season. **Station
  M's floor run (M.6) counts as evidence for HA-179 only on an IMU1 Part A build** (the team lead's D4 (a),
  2026-10-22; the worksheet's check: `kRateHoldUs` in the tree's `hal/pros/imu.hpp`); a TIMEOUT with TSTR
  lit on an older build is not evidence against it. **The rate's SIGN:** TANK1's observer is the first control law in the
  library that closes a loop on the yaw rate's sign (before it, the rate fed records, rest checks and
  |ω| gates), so a rate whose sign disagrees with the heading's (HA-04 — `get_gyro_rate().z`'s sign is
  undocumented) makes the observer push the WRONG way, up to its 1.5 rad/s limit: measured with a negated
  rate on the 9 % weak 40 in leg, heading wobble to about 16° and a TIMEOUT, visible as TSTR lit and the
  correction pinned at its limit. Robot two's default rate (differentiated `get_rotation()`) has the
  heading's sign by construction. **Contained:** `maxSteer = 0` restores the pre-TANK1 tank's motion,
  records and log exactly — all but the result line's ` xt` column (TANK1 Part B, 2026-09-28), a
  report kept whatever `maxSteer` is; every number is config.

- [ ] **HA-180 — robot two's sideways error on a straight `moveTo` comes from its heading, not a skid.**
  *Claim:* the drift of a tank leg on robot two is the HEADING being pulled — which the IMU sees and the
  drive encoders integrate into the pose — not the chassis sliding sideways, which
  `TankKinematics::forward()` reports as zero and no sensor on robot two sees.
  *Source:* TANK1 steers on the ESTIMATE's distance off the line; robot two's odometry is its drive
  encoders and IMU (HA-131), with no tracking wheel and no GPS on the bench. On the host test plant the
  drift is all heading by construction (its tank never skids), and a drive-encoder rig wired the way the
  builder wires robot two settles the 5 % and 9 % weak 40 in legs exactly like the tracking-wheel rig.
  *Confidence:* **reasoned** — the 2026-09-25 veer (about 11° over 48 in, open loop) was a heading change,
  read on the IMU; a sideways skid component was never measured.
  *Settle:* COMP-t's Station M.6 — END y by the estimate against the tape's sideways offset from the line,
  three runs; agreement within 0.5 in settles it.
  *Blast radius if wrong:* the screen says FINISHED while the robot is off its line on the tape by the
  skid, and every later leg starts from that error. Not contained by config — only a GPS, a tracking wheel
  or a wall reset sees a skid. The wall reset (L6, host-gated since 2026-10-29) sees it on the axis across the
  wall it resets against.

**COMP-t (2026-09-28) added TWO — HA-177 and HA-178.** "shulib Auton Tester" runs one of the team's
autons on the bench from RUN on the brain's screen (a 3 s countdown, then `competition::runAuton`, the
competition program's own call), aborts on any controller button, a touch of the screen, the controller
dropping or the robot being disabled, and appends every run's summary lines to one file on the SD card
across reboots. Two things it relies on are beliefs about the brain that the host cannot check: what an
append open does on the card, and what the abort and the RUN gate read from PROS. **Neither has been
observed; the tester has not run on a robot.**

- [ ] **HA-177 — an APPEND open under `/usd/` keeps the file and writes after it.**
  *Claim:* (1) `fopen(path, "ab")` reaches VEXos as `vexFileOpenWrite`: newlib makes `"ab"` a write-only
  open with `O_APPEND`, and PROS's `usd_open_r` sends exactly that case to `vexFileOpenWrite` (every other
  write-only open to `vexFileOpenCreate`). (2) On an EXISTING file, `vexFileOpenWrite` writes after its
  last byte and keeps every earlier byte. (3) On an ABSENT file it may fail or create — unknown. (4) A line
  written and flushed survives a power-off right after it (PROS calls `vexFileSync` after every write it
  passes on).
  *Source:* `include/shulib/hal/pros/block_sink.hpp` — `ProsFileMode::Append` and `openForAppend()`, the
  ONE place the decision is made: probe with `"rb"`, open `"ab"`, and create with `"wb"` ONLY when the
  probe said the file was absent, so an existing log is never truncated whichever way (3) goes; (3) only
  changes which word the boot line prints (CREATED either way). `hal::BlockCharSink`
  (`include/shulib/hal/block_char_sink.hpp`) turns it into the text sink the tester's second log writes
  through; `src/programs/auton_tester_program.cpp` opens `shulib_auton_runs.txt` once at boot (a BARE
  name — the adapter owns the prefix, HA-122). Host-tested through the shim on a real directory (two
  constructions leave both lines, in order, CREATED then APPENDED) and with a scripted opener (all four
  branches; no `"wb"` when the file existed); no mode has a `+` (the brain refuses read-write opens).
  *Confidence:* (1) **reasoned** from the vendored `libpros.a` and `libc.a`, disassembled 2026-09-26 (the
  reading the calibrator's safe-save work made); (2) and (3) **invented** — VEXos's behaviour; (4)
  reasoned for the call, invented for what the sync commits. HA-122 confirmed `fopen` under `/usd/` for
  `"wb"` only.
  *Settle:* COMP-t Stations M.2 and M.7 — the first boot on a card without the file (M.2.2: the third boot
  line says `SD run log ON, CREATED /usd/shulib_auton_runs.txt`), the runs of M.4–M.6, then a power-cycle
  (M.7.1: `ON, APPENDED to /usd/shulib_auton_runs.txt`) and one more run; copy the file to a computer:
  every run since M.2 present, in order, across the reboot. Never a blank or formatted card: the
  calibration record is on it, and the tester refuses to boot without it (M.7.2 deletes only the log, for
  a CREATED on a card that already had one).
  *Blast radius if wrong:* (2) wrong — earlier boots' runs lost from the card (serial still has them when
  it was captured); the robot's motion is unaffected. The one remaining truncation path: the `"rb"` probe
  fails on a file that exists AND the `"ab"` open fails too — then `"wb"` empties it; its cost is the same
  log. **Contained:** the SD log only; a refused card is `SD run log OFF` and serial carries every line.

- [ ] **HA-178 — the tester's abort and RUN gate read the right things from PROS.**
  *Claim:* read from a task the program created: (a) the touch struct's `press_count` rises when a finger
  LANDS on the screen, not when it lifts; (b) a controller button's level (`get_digital`) turns true within
  ~50 ms of the press; (c) started from the brain's screen with nothing connected and the controller on its
  home screen, `competition_get_status()` reads 0x00 — there is no CONNECTED bit from the controller's own
  link; (d) a controller switched OFF reads `is_connected()` false within ~1 s.
  *Source:* `src/common/screen_touch.hpp` — `screenTouchFrom()` (the one place PROS's touch fields are
  read for the Auton Tester and the competition program since COMP-b moved it there, 2026-09-30 —
  shulib Calibrate and the bench tests read the status themselves: `press_count` is the abort's count,
  `release_count` the pages' release edge, an `E_TOUCH_ERROR` read never counts) — and
  `src/programs/auton_abort_watch.hpp`'s `testerTouchFrom()`, which adds the tester's one fact (a finger
  is "down" when `press_count > release_count`),
  `abortInputsFrom()` and `AbortWatch` (armed with the press count of the very read whose release touched
  RUN, so any landing after it counts — COMP-t's review); `src/programs/auton_tester_panel.hpp` (`runAllowed`: RUN refused
  with any of DISABLED, AUTONOMOUS, CONNECTED set, and without a connected controller); the program's
  two one-line PROS reads, pinned by `test/auton_tester_source_test.cpp`.
  *Confidence:* (a) **reasoned** from the vendored `pros/screen.h` text; (b) **invented**; (c) **reasoned**
  from HA-128's measurement (a controller-menu start arms a pretend match, status 0x0f); (d) **invented**.
  *Settle:* COMP-t Stations M.2–M.4, wheels up: M.2's first `competition status 0x00` line (c); M.4's
  button in the countdown (b), a press-and-hold of the screen that aborts at the touch-down (a), and the
  controller switched off in the countdown and mid-run (d) — note how long after the switch-off the screen
  changed.
  *Blast radius if wrong:* (a) — a touch aborts later (at the lift) or never; the buttons still abort, and
  the RESULT page's wait for a held finger falls back to its plain 0.5 s arming delay. (b) slow — the robot
  stops that much later. (c) wrong — RUN is refused on the bench, with the bits on the status line
  (fail-safe). (d) slow — a controller that dies mid-run is noticed that much later (a touch still aborts).
  **Contained:** the tester's watch and its two reads; the competition program's own watch is COMP-b's.

**L9 (2026-09-29) added ONE — HA-167.** A routine can now back into a point: the recipe layer's
`Routine::reverseTo` (with `faceAway`, the turn before it on a tank; the changelog's API 2.4 entry), and a
route's `"drive"` waypoint with `reverse` set, which the path runner has executed since G2 — the two are
bit-identical. Both command a tank to drive BACKWARDS under power for a whole leg, and everything that
turns a tank's wheel travel into inches was measured in ONE unpowered push, one end first — never under
power, and never in the other direction. The host test plant cannot see the difference: its motor model
is the same in both directions and has no backlash. The number is L9's
reservation (the numbers around it are other chunks'); **nothing here has run on a robot.**

- [ ] **HA-167 — a tank drives backwards the way its calibration measured it.**
  *Claim:* (a) shulib Calibrate measures each drive side's travel scale (inches of travel per radian of
  motor rotation) from ONE unpowered 48 in push, either end first (a BACK-first push inverts every sign),
  and that one scale holds within 1 % when the robot drives itself under power in BOTH directions — the
  direction the push went and the other one. (A reversed leg after `faceAway` drives backwards for the
  whole leg: the untested direction after the usual FRONT-first push; after a BACK-first one, the push's
  own direction, but under power.) (b) A change of direction — where each side's gear train takes up its
  backlash the other way — adds less than 0.25 in of travel that the chassis did not make.
  *Source:* `include/shulib/chassis/routine.hpp` (`reverseTo`, whose doc comment names HA-167) and
  `include/shulib/runner/path_runner.hpp` (the `DriveTo` with `reverse`, whose doc comment names
  HA-167) command the backwards legs; the scale they are measured in comes from
  `include/shulib/calibration/drivetrain_calibrator.hpp` (the one push, `PushCapture`, and BACK-first).
  The host test plant models every wheel as odd-symmetric with no backlash term
  (`include/shulib/sim/motor_model.hpp`: V = kS·sign(v) + kV·v + kA·a), so no host test can see either
  effect — the L9 tests that drive backwards (a 43 in reversed leg on a tank, graded on the plant's truth)
  prove the geometry, not this.
  *Confidence:* **invented.** The scale was never measured under power, in either direction: the push is
  unpowered, and it goes one way.
  Calibrate's spin phase does drive one side backwards under power in each spin direction, but it
  measures WIDTH (W = (dR − dL) / dθ), not scale. The one known anomaly on robot two came during
  reversing — on 2026-09-25 an implausible-odometry fault fired right after about 0.35 s of hard
  reversing (HA-160) — which says reversing is where trouble was seen, not that the scale is asymmetric.
  *Settle (the first on-robot autonomous session — COMP-t's auton tester runs any auton on the team's
  list, or COMP-b's Station K):* on the floor, along a tape, run a routine that drives 48 in forward,
  stops, `faceAway`s the start and `reverseTo`s it; record the estimate and the tape at each end, three
  times. (a) holds if the forward and the backward scale errors agree within 1 %; (b) if the estimate and
  the tape disagree by less than 0.25 in more after the reversal than before it. Record the raw IMU
  heading through every reversal as well (HA-160's settle).
  *Blast radius if wrong:* a reversed leg ends short or long by the asymmetry, and every later leg starts
  from that error — position only: the heading is the IMU's. A take-up at each direction change adds a
  phantom offset along the heading that nothing corrects until a wall reset or GPS. An IMU glitch during a
  hard reverse (HA-160) ENDS the motion (`ODO_STUCK`) rather than bending it. **Contained?** Only partly:
  a measured asymmetry needs a per-direction scale, which is a change to the calibration RECORD (a
  version bump of its 180-byte format), not a config constant.

**T4 Part 2a (its library half; landed 2026-10-01) added TWO — HA-175 and HA-176 — and AMENDED HA-132 and
HA-133.** The old "shulib Drive"'s protections moved into the library's driver loop
(`teleop::runDriverControl`, `teleop/drive_protection.hpp`; the changelog's "the drive protections,
inside the driver loop"), so every program that drives through it now drops a drive motor that stops
answering, stops the drive once more than two are dead, and cuts it for 1 s on a fight or an
over-current — in a match. The old
program saw a dead port through the adapter's faulted-read count; the library holds its drive motors
as `hal::IMotor`, which has no such count (F4, locked), so it recognises a dead port by the one thing
the seam does show — readings that stop changing. HA-175 is that rule and the stop it feeds; HA-176 is
the over-current cut, unregistered until now (the old program carried its number as "INVENTED for a
first run"). HA-175's number was first reserved for a belief about program names that the team lead's
ruling of 2026-09-27 made moot; it was never issued for it. **Nothing here has run on a robot.**

- [ ] **HA-175 — a dead drive port FREEZES its readings; a live one never freezes for 250 ms.**
  *Claim:* (a) a robot-two drive port that stops answering mid-run shows through the `hal::IMotor` seam
  as a FROZEN reading: its `position()`, `velocity()` and `current()` all repeat their last good values,
  bit for bit, for as long as it is dead. (b) A LIVE member is never FROZEN for 25 consecutive 10 ms
  ticks — FROZEN being its position AND velocity bit-identical for 5 consecutive reads while its own
  report says it turns (above 0.5 rad/s) or a side-mate whose ENCODER moved within the last 5 reads says
  the train does (above 1 rad/s), with at least as many of its side answering as not — because a live
  motor's velocity report changes while it decays, and a stall stops its mates' encoders too (a mate
  whose encoder has stopped is no evidence, however its lagging report still reads: the team lead's
  ruling of 2026-10-01, "Encoder check"). Brief FROZEN spells as a train coasts to a stop are expected
  and harmless. The EXCEPTION is a wall BESIDE A SLIPPING side-mate (HA-130's port 18), whose encoder
  keeps turning and so stays evidence: there the guard — a COUNT of suspect members against answering
  ones — holds only while every stalled member reads alike (the known gaps, below).
  (c) A side still drives acceptably with its dead dropped, up to the mid-run stop: more than two drive
  motors dead in total (the team lead's ruling N7 (c), 2026-09-27: "Only when 3+ are dead in total");
  the per-side floor (three answering) is applied at BOOT only.
  *Source:* `include/shulib/teleop/coupled_side_monitor.hpp` (`FrozenReadingDetector`: the rule);
  `include/shulib/teleop/drive_protection.hpp` (`DriveProtectionConfig::frozenSuspectReads` = 5,
  labelled A4 register HA-175, and the loop's use of the verdict: out of the fight monitor and the
  over-current watch at once, ABSENT after the monitor's 250 ms); `include/shulib/hal/motor_group.hpp`
  (`markAbsent`, `setMonitorExcluded`); `include/shulib/teleop/drivetrain_degradation.hpp`
  (`evaluateRuntimeDegradation`); `include/shulib/hal/pros/motor.hpp` (the screen: `position()`,
  `velocity()` and `current()` hold their last good values on a failed read). Host-tested in
  `test/drive_protection_test.cpp`: P2-5 and P2-6 to P2-6d (a death at rest or at speed, a fight that
  then freezes, a death just before a cut, a deceleration through zero — FROZEN within 5 reads, ABSENT
  24 ticks later, no cut); P2-19 (a live train at a wall, every report lagging alike, marks nobody);
  P2-19c (a coast-down with a quarter-count encoder phase per member and a 150 ms report lag: no
  FROZEN tick at all); P2-21 and P2-21b (a slipping member at a wall, drag 0 and 0.8, and coasting
  through a cut: no live member marked; 3 fight cuts during the push); P2-19d (a PLAIN wall with one stalled member reading unlike its
  mates — a 1-RPM report flicker, a one-count encoder dither, a report settling 30 ms beside 80 ms:
  nobody marked, nobody FROZEN, no cut); P2-19e (a plain wall, one report settling sooner at every
  spread the verification of 2026-10-01 probed: nobody marked or FROZEN, and ONE 1 s fight cut from 30
  beside 100 ms on — the encoder check's cost, pinned); P2-21c and P2-21d (the known gaps below,
  pinned); and `test/coupled_side_monitor_test.cpp`'s P2-10b cases, each rule at its boundary, forward
  and backward, the encoder check's window among them.
  *Confidence:* the numbers are **invented** (the 5 reads, the 250 ms latch, the majority-or-tie
  guard; the floor, R3b Part 0b), on **reasoned** mechanics (the adapter's screen, read from the code;
  BLUE's 300 counts per output turn — one count about every 42 ms at 0.5 rad/s and every 21 ms at
  1 rad/s, so a turning live encoder changes well inside 5 reads), and the rest is **host-probed
  only**: the 2026-09-28 re-check's probe and this chunk's hand-stepped rig model a train lag,
  whole-RPM velocity reports, a per-member encoder phase and a slipping member (robot two's port 18,
  HA-130). Two beliefs under the rule are unmeasured: that a dead V5 port holds its VELOCITY bit for
  bit (ProsMotor's screen does; that a pulled cable produces the failed read the screen catches is
  HA-98's family), and that a live motor's reported velocity CHANGES while it decays — a whole-RPM
  report can hold one value for a few ticks. A third is unmeasured and load-bearing beside the slipping
  member (the review of 2026-09-30): that the stalled members read ALIKE. At a plain wall it no longer
  bears on a mark — since the encoder check, no stopped encoder is evidence — but it bears on a CUT: no
  V5 motor's report lag has been measured (the model's 80 ms is invented), and in the model a member
  whose report settles 30 ms beside mates at 100 ms or more, or 10 ms beside 80 ms, gets the push one
  1 s fight cut (P2-19e; HA-132/133's monitor, which now sees it). Before the ruling the same model
  marked that member dead (30 ms beside 120 ms at full stick, 10 beside 100, 30 beside 150 at half
  stick — the verification of 2026-10-01). Beside the slipping member the same model with ONE stalled
  member perturbed marked live members
  every time; one stalled member's report reading 1 RPM instead of 0 on a random half of the ticks of
  one push, and a second push without it, REFUSED the drive in 20 of 20 seeds (11 of 20 at three ticks
  in ten, 8 of 20 at two, none at one) — host-probed only.
  *Settle:* T4 Part 3's Station F.6 (i): wheels up, half stick, pull one drive motor's cable — expect
  it FROZEN within about 50 ms and DEAD about 250 ms later, logged once, no cut, and still DEAD after it
  is plugged back in (latched until the restart); the DEGRADED boot with one port unplugged per side;
  and a full-stick wall push watched for a false DEAD on ANY drive port, port 18 (HA-130) included —
  the push check's push (HA-176) is such a push, so its log is read for any `went ABSENT` line too, and
  for a `CUT: FIGHT` line (the encoder check's cost when one report settles sooner). Whether this wall
  push is REQUIRED before a match, like the push check, is the team lead's to rule with the known gaps
  below.
  *Blast radius if wrong:*
  - **A live member wrongly marked absent** drops out of the reads, which is harmless to the median,
    but it counts as dead, so the mid-run stop comes one death sooner.
  - **A dead member missed** stays in the fight monitor: its stale velocity can read as a FIGHT — a 1 s
    cut that repeats every 1.25 s — and its stale current can do the same through the over-current cut
    (HA-176).
  - **Since T4 Part 2 the stop applies MID-MATCH, in every driver-control program that drives through
    the library's loop.** On robot two (four a side) it comes at the THIRD dead drive motor in total,
    anywhere; two dead on one side drive on, degraded (the boot would refuse that robot). A stop lasts
    the REST OF THE RUN: every later start of driver control is refused too, until the program is
    started again, because the marks live in the robot's motor groups.
  - **The known gaps — the guard is a COUNT of suspect members against answering ones (the team
    lead's decision D (a) of 2026-09-28), and beside a slipping member it is right only while every
    stalled member reads alike:**
    - **At ANY wall — CLOSED by the encoder check** (found by the verification of 2026-10-01; the team
      lead's ruling the same day, "Encoder check"; built at T4 Part 3): every encoder stops with the
      train, and until the ruling the mates' lagging velocity REPORTS still counted as the train
      turning, so a member whose report settled to 0 well before theirs was SUSPECT alone and marked
      dead, a LIVE motor (30 ms beside 120 ms in the host model). Now a mate counts only while its
      ENCODER moved within the last 5 reads, and nobody is marked or FROZEN at any spread the
      verification probed (P2-19e). **Its cost, accepted with the ruling:** that member, no longer
      FROZEN, stays in the fight monitor under a quarter of its mates' REPORTED speed while their
      reports read above 1 rad/s, so the push gets ONE 1 s FIGHT cut while its motors draw under 2.4 A
      (in the model, from 30 ms beside 100 ms, or 10 beside 80, on; P2-19e). With every motor over
      2.4 A at the wall the over-current cuts come first (HA-176) and it adds none.
    - **A side ALREADY at two of four** (D (a)'s accepted cost), whose slipping member keeps turning
      while its last live mate stalls at a wall, is one suspect against one answering — a tie — so the
      stalled LIVE member is marked, which is the third dead, and the drive is REFUSED (P2-21c: 1.6 s
      into the push). A strict majority would close it, and then a two-of-four side's next REAL death
      would never be marked and would cut the drive every 1.25 s instead.
    - **A FULL side beside the slipping member** (found by the review of 2026-09-30 — wider than
      decision D weighed, and the team lead's to rule on): ONE stalled member whose readings keep
      changing — a report flickering by 1 RPM, an encoder toggling a count on a humming train — counts
      as answering, a 2-2 tie that marks two LIVE members; one whose report settles sooner than its
      mates' is suspect alone, and is marked at a smaller difference than at a plain wall (30 ms beside
      80 ms: the slipping member's encoder keeps the evidence alive). The side drives on degraded; a
      second push, its stalled members now reading alike, marks the last one against the slipping
      member — the third dead, REFUSED for the rest of the run, on a side of four live motors (P2-21d).
    - **The other changes weighed, and what each costs:** a strict majority ends the ties but not the
      lone suspect, and costs what the two-of-four case says; counting as answering only a member whose
      own reading shows the train turning leaves a dead member in the monitor through a slow push, cut
      again and again. The encoder check (chosen, above) does not reach the two slipping-member gaps:
      the slipping member's own encoder keeps turning, so it stays evidence — the ruling keeps them
      until port 18's gear is fixed.
    - Port 18's slipping gear is the trigger of the two slipping-member gaps: fixing it ends them. And
      a whole side that stops answering at once is never seen — every member alike looks like a side
      that stopped; the robot shows it by not driving that side.
  - Too permissive a floor keeps a robot driving that drags into a curve.
  **Contained:** one config field (`DriveProtectionConfig::frozenSuspectReads`), the monitor's two
  floors it reuses (HA-132), and two library constants (`drivetrain_degradation.hpp`'s
  `kMinAnsweringPerSide`, `kMaxDeadTotal`).

- [ ] **HA-176 — above 2.4 A for 250 ms is a fault, not driving; and a drive motor can get there.**
  *Claim:* on robot two a drive motor drawing more than 2.4 A for 250 ms while its side is driven
  above 1 V means something is WRONG — a fight on the coupled train, or a stall — and not ordinary
  driving, a full-stick REVERSAL at top speed and a full-stick LAUNCH from rest included: each is
  expected to hold every drive motor near its current limit while the train turns round or spins up,
  and the belief is that this lasts LESS than 250 ms; and a drive motor CAN draw more than 2.4 A (the
  per-motor limit stays at the PROS default 2500 mA with twelve motors plugged in, HA-154).
  *Source:* `include/shulib/teleop/drive_protection.hpp` (`DriveProtectionConfig::overCurrentAmps`
  2.4, `overCurrentTicks` 25 — pinned equal to the fight monitor's window by a `static_assert`, HA-132 —
  and `overCurrentCuts` true; labelled A4 register HA-176), carried from the old "shulib Drive"'s
  `kCurrentLimitA` (in its `src/programs/drive_program.cpp`, "INVENTED for a first run"; that program was
  replaced at T4 Part 3 — Bench Tests' DRIVE station keeps its own `kCurrentLimitA`, 2.4). Judged per member,
  never on the side's total, only while the side is commanded above 1 V, and never for a FROZEN or
  ABSENT member (a dead port's current is its last good reading). Host-tested in
  `test/drive_protection_test.cpp`: P2-4 (2.41 A for 24 ticks no, 25 yes, driving forward and in
  reverse; 2.40 A never; the command floor; the warn switch), P2-4c (a side commanded exactly 1 V
  counts nothing), P2-4b (a member that dies at the current limit cuts once, never again), P2-13b (an
  X-drive wheel's streak starts over at the re-arm), P2-16b and P2-16c (the peak line, for a push
  forward and one backing up), P2-5c (a FROZEN member's current is in no peak). The host test plant
  synthesizes no current: every current in those tests is set by hand, so none of them shows how long
  a reversal or a launch holds the limit.
  *Confidence:* **invented** (R3b Session 2's first-run number; no current trace of robot two under
  power exists).
  *Settle:* the PUSH CHECK, required before the competition program drives a match (the team lead's
  ruling N1 (a), 2026-09-27): on the ground, full stick into a wall for 3 s, then let go; record whether
  it CUT, and the highest current — from the `drive: LEFT peak … A (port n) over a … s drive, …` line
  printed when the stick is let go, and from a cut's `at the cut:` lines (the disable summary may never
  print: the loop must see the disable before PROS deletes its task, HA-161, and the team has no
  competition switch). Then, on the ground and clear of everything, a full-stick REVERSAL at top speed
  (full forward until it is at speed, then full back at once) and a full-stick LAUNCH from rest, each
  let go after it: read
  the same peak line and look for any `CUT:` line. A reversal or a launch that cuts is the team lead's
  to decide before a match (N1: the number, or the switch). It belongs to the competition program's
  practice-match station (K.8, written at T4 Part 3's late step); T4 Part 3's Station F repeats it
  (F.6 (ii)) after reading the highest A at full stick wheels-up.
  *Blast radius if wrong:* too tight, and a push against a wall or a robot trips it: the drive is cut
  for 1 s of every 1.25 s, IN A MATCH, in every driver-control program since T4 Part 2 (a V5 motor
  pushing is believed to sit at its current limit — the PROS default 2.5 A, above 2.4 A; HA-154,
  unmeasured). And if a full-stick reversal or launch holds the limit for 250 ms, a HEALTHY drive is
  cut for 1 s in the middle of the manoeuvre — the case the push alone does not try. If VEXos lowers the
  per-motor limit below 2.4 A with this many motors plugged in, the cut can never fire, and a locked
  fight — which the fight monitor cannot see, since it judges a side only while it turns above
  1 rad/s — is caught only by MOTOR_OVER_TEMP (55 °C, HA-44), minutes later.
  **Contained:** one config field and one switch (`DriveProtectionConfig::overCurrentAmps`,
  `overCurrentCuts` — false makes an over-current a logged, counted warning; the team lead's N1 (a):
  ON by default, one switch, and the push check before any match).

### R5a Part A's additions — the beliefs shulib Calibrate's GAINS and REST steps are built on (landed 2026-10-06)

**T4 Part 3's hand-over (the coordinator, 2026-10-08) added ONE — HA-189.** The competition program logs from
more than one task (the auton task, PROS's phase tasks and the opcontrol task's driver loop), so around a mode change two
can write the serial log at once; "shulib Drive" has one writer today. The card copy got its own one-writer guard at T4
Part 3 (`diag::LogCopySink`); serial did not, on the belief below — which the vendored binaries contradict.

- [ ] **HA-189 — two tasks writing the USB serial (stdout) at once keep their lines whole.**
  *Claim:* `hal::pros::ProsCharSink::write()` (`include/shulib/hal/pros/char_sink.hpp`) did `std::fwrite` + `std::fflush`
  on stdout until SER1, and its banner said "fwrite is atomic per call at this layer, so lines never interleave" — i.e.
  the C library serializes two tasks' calls on one `FILE*`.
  *Source:* the vendored toolchain, read 2026-10-08 (`.claude/coordinator/scratch/ha189/`, and `nm`/`objdump` on the
  linked `bin/cold.package.elf` of the seven-build tree): (1) newlib's per-FILE locks are the default stubs — every
  `__retarget_lock_*` in `firmware/libc.a` (`libc_a-lock.o`) is `bx lr` (2 bytes, linked as such), and `libpros.a`
  overrides only `__malloc_lock` and `__env_lock`; `vfprintf`, `fwrite` and `fflush` call those stubs. (2) stdout is a
  TERMINAL to newlib — PROS's `ser_isatty_r` is `movs r0, #1` — so it is LINE-BUFFERED through one buffer every task
  shares; nothing calls `setvbuf` on it (the only `setvbuf` caller is libstdc++'s `basic_file`). (3) Below the buffer,
  PROS's `ser_write_r` DOES take a mutex around each device write (`mutex_take`/`mutex_give` around
  `ser_output_write`) — so one flushed chunk reaches the USB whole, but the buffer filling it is unguarded.
  *Confidence:* **reasoned — FALSE as stated.** Two tasks inside `fwrite` on stdout at once can interleave their bytes
  in the buffer, and newlib's unguarded updates of the buffer's pointer and count can lose or repeat bytes; whether the
  race can also write past the buffer's end is reasoned possible, never observed. How often it happens on the brain
  (PROS's preemption, each task's priority, the time-slice) is unmeasured.
  *Settle:* a bench program with two tasks of equal priority each printing numbered 120-byte lines as fast as they can
  for 60 s; capture `pros terminal` to a file and check every line is whole and every number present once. Then the
  competition program's own Station K log around each mode change. A fix that makes one line one device write (stdout
  unbuffered, or a mutex in `ProsCharSink` — SER1 chose neither: one direct `write()` per line, below) is settled by the
  same test passing.
  *Blast radius if wrong (as believed):* serial log lines mixed or lost around mode changes in the competition program —
  the log a match is diagnosed from; the robot's motion is unaffected unless the buffer race ever writes out of bounds
  (then anything, a crash included — not observed, not excluded). The SD card's copy is unaffected (its own guard).
  **Not contained:** every program that logs from more than one task — until SER1, below.
  *The fix (chunk SER1, 2026-10-08, the team lead's ruling "one-call writes"):* shulib never writes the serial through
  stdout's FILE any more. `ProsCharSink::write()` sends each line with ONE direct `write()` on the serial's descriptor
  (`hal::pros::writeWhole()`: the remainder after a short write, a stop on 0 or −1); `hal::pros::serialLine()` formats a
  line on the CALLER'S stack (256 bytes) and sends it the same way; `hal::pros::serialWrapped()` sends a longer text as
  pieces of at most 256 bytes, one write each — and all 20 `printf` sites and 14 `fflush(stdout)` lines in `src/` went
  through them. No call is offered more than 512 bytes (`kSerialWriteMaxBytes`, HA-191) — above every line shulib
  prints, so each stays one call. One `write()` is one frame under PROS's write mutex (claim (3) above), so the claim
  the register states is now what shulib's own lines DO — reasoned from the same disassembly, never seen on a brain.
  The src build gate's step 9 fails any `src/` object that names stdio's FILEs (`_impure_ptr`, what naming stdin,
  stdout or stderr compiles to, and `__sf`) or calls a printer that writes stdout or stderr — every one the vendored
  `libc.a` defines (`printf`, `puts`, `viprintf`, `putchar_unlocked`, `wprintf`, …; the list is checked against the
  archive's symbol table on every run) and the C++ standard streams. stdout itself stays line-buffered (unbuffered
  stdout, the first ruling, would have given every `printf` a ~1.2 KB stack frame — enough to overflow PROS's 2 KB
  serial daemon when `pros terminal` connects). What it
  does NOT cover: PROS's own stdio (the serial daemon's banners, the expander ADI's `puts("Error: validate_type")`, crash
  dumps, `task_fn_wrapper`'s `fprintf(stderr)`), each of which ends its lines and so meets shulib's at a line boundary;
  a team's own `printf` (the guide's chapter 11 gives the rule); and PROS's own mutex on a deletion (HA-190).
  *Settle (as fixed):* **the team lead accepted, in writing on 2026-10-18 ("Use Station K's K.7.3"), worksheet
  Station K.7.3 as HA-189's robot test — no new Bench Tests station.** Until then the dedicated two-task numbered-line
  test above was the settle, and it was OWED: no build carries a program that prints numbered lines from two tasks (the
  bench program's menu was full — twelve stations, three across and four down, with no room for a row on the panel — so
  it needed a station retired or a new layout; that call was the team lead's, and he chose neither. *(MECH-A6, 2026-10-30,
  made the menu 3 × 5 for its station 13 — room for fifteen, two slots free; the ruling stands, K.7.3 is HA-189's test.)*) K.7.3 is the
  competition program's own serial capture around every Timed-Run mode change, every expected line whole — with no
  sequence numbers, a lost line shows only as an expected line missing (the weakness he accepted); and K.7.4, `pros
  terminal` connected to a RUNNING program once (the banner prints, no halt). **HA-189 stays UNSETTLED until K.7.3
  runs.**

**SER1 (2026-10-08) added TWO — HA-190 and HA-191** — what one `write()` per line leaves in PROS's hands (reasoned from
the vendored `libpros.a` by SER1's brief checker, `.claude/coordinator/scratch/ser1-check/`; not caused, and not cured,
by SER1).

- [ ] **HA-190 — a task PROS deletes INSIDE one serial write leaves PROS's serial write mutex held.**
  *Claim:* `ser_write_r` takes PROS's serial write mutex (`mutex_take` +0x72), builds the frame on the caller's stack and
  hands it to `ser_output_write` → `stream_buf_send`, whose wait for room is unbounded, then gives the mutex back
  (`mutex_give` +0xa6); `task_delete` releases no mutex a task holds. So a task deleted between the take and the give
  leaves the mutex held, and every later serial write — stdout and stderr alike, PROS's own included — blocks its
  writer for the rest of the boot.
  *Source:* `include/shulib/hal/pros/char_sink.hpp`'s banner (THE LIMITS); `src/programs/competition_program.cpp`'s file
  banner: its `opcontrol()` logs two lines right after its loop sees a disable — the moment PROS deletes that task
  (HA-161 (a), reasoned). `diag/log_copy_sink.hpp` names the same exposure for a writer deleted inside the CARD
  copy: the copy ends there for the rest of the boot, but no task waits (its guard gives up rather than waits) — and it
  says a blocked serial ends the copy too.
  *Confidence:* **reasoned** (the disassembly; never observed). How often a deletion lands inside that window — a few
  microseconds per line — is unmeasured.
  *Settle:* worksheet Station K.7.3 — Timed-Run mode changes while the competition program logs. This case looks like
  a serial capture that STOPS at a mode change, and after the next enable the robot does NOT answer the sticks while
  the brain still shows the program: every phase function's FIRST line is a log line (`competition_program.cpp`:
  `disabled()`, `autonomous()` and `opcontrol()` each call `logPhase(…, "entry", …)` before anything else, `opcontrol()`
  before its driver loop), so the next phase task blocks on it and driver control never starts. Restart the program.
  *Blast radius if wrong (as believed):* every later serial line of that boot blocks its writer — the next phase
  function blocks at its entry line, so driver control never starts (the robot stops answering the sticks), and the
  auton task likewise. The SD copy STOPS AT THE SAME LINE: `diag::LogCopySink` writes the primary (serial) first and only
  then the copy (`log_copy_sink.hpp`), so a writer blocked on serial never reaches the card — `comp_log.txt` and
  `drive_log.txt` end where the serial capture ends (the card file itself is not damaged; the copy guard's give-up
  covers contention on the CARD, not a blocked primary). **Not contained by shulib:** a fix would be PROS's (a mutex
  released on deletion), or a single printing task — the team lead's "one printing task" option, not chosen (SER1's
  brief, §2).

- [ ] **HA-191 — one serial write must fit PROS's 2047-byte serial stream.**
  *Claim:* PROS's serial output is a FreeRTOS stream buffer of 2047 bytes (`ser_driver_initialize` +0x38,
  `movw r0, #2047`), and `stream_buf_send` waits until the WHOLE frame fits — a 4-byte stream id, the COBS-encoded data
  and its delimiter, so about 2,035 bytes of text at most; a larger write waits for ever, holding PROS's write mutex
  (HA-190's consequence without any deletion).
  *Source:* `include/shulib/hal/pros/char_sink.hpp`: `writeWhole()` — the one loop every shulib serial write goes
  through — offers each `write()` at most `kSerialWriteMaxBytes` = 512 bytes (`static_assert` ≤ 1024), so a longer text
  through `ProsCharSink` goes out in 512-byte pieces (SER1's review, 2026-10-15: before it, the sink offered a text of
  ANY length to one call; the team lead accepted the pieces on 2026-10-18, "Accept pieces"); `serialLine()`'s and
  `serialWrapped()`'s lines are at most `kSerialLineBytes` = 256.
  shulib's own lines are at most 384 bytes (`diag/line_format.hpp`'s `Line::kCapacity`, pinned ≤ 512 by the host
  suite) and TermSink's messages 200.
  *Confidence:* **reasoned** (the disassembly). No shulib call comes near it; a team's own `write()` on fd 1, or a
  `printf` of a long buffer, could.
  *Settle:* not needed for shulib's calls (a quarter of it at most); a team that must send more splits it
  (`serialWrapped()`, or the sink, which pieces it).
  *Blast radius if wrong:* if the stream were SMALLER than believed, a 512-byte write could hang the serial — the same
  as HA-190's. **Contained** for shulib's calls by `writeWhole()`'s 512-byte ceiling (and its `static_assert` ≤ 1024);
  a 512-byte piece is a quarter of the stream, so the margin is a factor of four.

**L6-A1 (2026-10-29) added EIGHT — HA-204..HA-209, HA-211 and HA-212.** The wall reset re-seats ONE axis of the
estimate from a side distance sensor parked beside a field wall (`localization/wall_reset.hpp`,
`chassis/wall_reset_call.hpp`, the season's walls in `localization/override_walls.hpp`; the host test plant's
field and sensors in `sim/field_plant.hpp`). Every gate it applies rests on a belief about the sensor or the field
that only a robot on a field can settle — the L6 brief's Step 0 (worksheet Station H). **None is measured; the
reset has not run on a robot.** HA-210 (robot two's side-sensor ports and their unmeasured mounts) is held for
L6-B, whose robot table cites it.

- [ ] **HA-204 — the V5 Distance Sensor's accuracy: max(15 mm, 5 % of the reading).**
  *Claim:* on a flat, wall-sized target the sensor reads to about ±15 mm below 200 mm and about 5 % above, over
  20–2000 mm, measured from its front face (the zero point is not published). The two figures are approximations
  that meet at 200 mm, not an optimum, so the reset assumes the LARGER at every range — never a step at 200 mm (a
  step would put a cliff in the precision gate) — until a measured `SideMount::rangeAccuracy` replaces it.
  *Source:* VEX's published figures (api.vex.com, fetched 2026-10-18); `localization/wall_reset.hpp`
  (`kPublishedCloseAccuracyInches`, `kPublishedFarAccuracyFraction`, `readingAccuracy`, `axisErrorBound`, the
  `rangeMin` floor's reason, `SideMount::rangeBias` / `rangeAccuracy`); since L6-A2 `config/wall_sensor_config.hpp`
  (`kWallRangeFloorInches` — the robot file's `rangeMin` may not go under VEX's 20 mm — and a measured accuracy's
  0.05–2-in span).
  *Confidence:* **reasoned** — published; our sensors' real spread and zero point unknown.
  *Settle:* L6 Step 0.3 (ten taped distances × five RECORDs per side: the bias and the spread) and Step 0.7.
  *Blast radius if wrong:* a sensor worse than published makes the reset seat with a larger error than its bound
  says (the bound is the precision gate); better than published makes it refuse `imprecise` more than it must.
  **Contained** by a measured `rangeAccuracy` / `rangeBias`, typed per side.

- [ ] **HA-205 — the beam is a ±12°–18° cone, square within 5° reads the beam-centre range, and two surfaces
  in the cone read as the NEAREST.**
  *Claim:* the beam spreads about ±12° (long range) to ±18° (short-range mode); within 5° of square to a flat wall
  the reading is the beam-centre range to within HA-204 (≤ 0.046 in at 12 in); with two surfaces in the cone the
  sensor reports the nearest — though VEX says object size comes from "how much of the sensor's view is taken up",
  so it may integrate over its view and report a BLEND.
  *Source:* `localization/wall_reset.hpp` (`WallResetLimits::coneHalfAngle` widening the keep-outs and setting the
  beam-height ceiling while the mount's cone is unmeasured; `SideMount::coneHalfAngle`; the 5° square gate's
  reason); `sim/field_plant.hpp` (the host plant's nearest-hit default and its `blend` mode); since L6-A2
  `config/wall_sensor_config.hpp` (a measured cone's 3–30° span in the robot file).
  *Confidence:* **invented** — a community post seen only through search summaries; VEX publishes no angle and no
  multi-target behaviour.
  *Settle:* L6 Step 0.5 (angles 0–20° at 8 and 12 in; a Pin slid in from the side and from below — WHERE the
  reading changes and HOW: a jump to the Pin, or a slide) and Step 0.6 (a person just behind an alliance wall
  while the cone's top passes over it).
  *Blast radius if wrong:* a WIDER real cone than typed lets the beam-height gate pass a reading a standing piece
  could make (the start pieces stay kept out until a cone is measured, which bounds it there); a blending sensor
  changes the known-limit magnitudes (an object at a plausible distance) and the claim that the cone's top edge
  over the wall is harmless. The beam-height gate itself does not depend on the multi-target behaviour (at or
  under the ceiling no piece is in the cone at all).

- [ ] **HA-206 — the wall's face returns a reading at the sensor's height: opaque walls, clear panels and the
  GPS code strip.**
  *Claim:* the portable perimeter's opaque walls AND the metal perimeter's clear panels return a reading; so does
  the VEX GPS code strip, which VURC and Autonomous Coding Skills fields MUST carry (manual v2.0, the VURC field
  setup; `<RSC4>` a): two rows of black and white squares on the wall at 9.06–11.44 in (A8 at 300 dpi) — where every
  useful mount's beam lands — a black square reading the same range as a white one, within HA-204, with a usable
  confidence.
  *Source:* `localization/wall_reset.hpp` (the file banner; the `LowConfidence` / `NoReturn` gates);
  `sim/field_plant.hpp` (the see-through boxes and the code strip's per-colour bias, default none).
  *Confidence:* **invented, with CONTRARY evidence** — a VEX Forum thread (seen only through search summaries)
  reports the sensor seeing through the metal perimeter's clear panels or dropping out beyond ~700 mm; no published
  behaviour on dark or patterned targets. The strip's place is manual v2.0's (A8); v2.1 (2026-10-08) exists and
  was not readable on 2026-10-29, so whether it moved the strip is UNVERIFIED (HA-207).
  *Settle:* L6 Step 0.6 on each perimeter type the team can reach, ON the strip where the field has one; Step 0.3's
  black and white squares (a printed full-scale sample if the team's field has no strip).
  *Blast radius if wrong:* on a clear panel the reset usually REFUSES (no return, out of range, disagrees) — but
  SEATS wrong by up to `maxCorrection` when something opaque stands within that distance behind the panel (the
  host test plant: 1 in behind → ~1 in wrong, 2 in → ~2 in; the wall's own frame, a hand). A biased black square
  shifts every reading on the strip. Not contained by code; a tight `maxCorrection` caps it.

- [ ] **HA-207 — the walls' inner faces are ±70.205 in from the centre, within ±0.5 in on a real field.**
  *Claim:* A10 places the field elements on inner faces 3566.4 mm apart (±70.205 in); A13's portable perimeter
  agrees (140.40 in); A14's METAL perimeter is 140.50 in (±70.25 — 0.045 in further out per wall). A real field is
  within ±0.5 in of its drawing.
  *Source:* `localization/override_walls.hpp` (`kWallInnerFace`, `kMetalPerimeterInnerFace` — stated, never
  averaged); the host plant's own copy agrees to 0.01 in (`test/sim_field_plant_test.cpp`).
  *Confidence:* **reasoned** (the drawings — manual v2.0's; v2.1, released 2026-10-08 as a "clarification /
  minor update", exists but could not be read on 2026-10-29, so whether it moved any of these numbers is
  UNVERIFIED) · **invented** (the tolerance — the manual says only that fields "vary slightly").
  *Settle:* L6 Step 0.6 — the field measured face to face at three places, and WHICH perimeter type it was.
  *Blast radius if wrong:* every seat is off by the wall's error — 1:1, silently, inside the gates. **Contained** to
  one constant per season header.

- [ ] **HA-208 — each Toggle's BODY sits on top of its wall; its BRACKETS hang to ≈ 10.6 in.**
  *Claim:* the Toggle's body sits ON TOP of its wall, overhanging the inner face by ≤ ~1 in, so a beam below the
  wall top (11.50 in metal, A14; 11.54 in portable, A13 — printed) reads the wall beneath the body; its two
  BRACKETS, at |along| ≈ 10.9–13.0 in, hang down to ≈ 10.6 in above the floor, into the beam's band; how far they
  reach into the field is not drawn.
  *Source:* `localization/override_walls.hpp` (the Toggle BODY span — `KeepOutKind::Toggle`, lifted per side only by
  a measured `SideMount::toggleChecked` — and the two BRACKET spans, `Fixed`); `sim/field_plant.hpp` (the plant's
  1-in overhang and 1-in-deep brackets); since L6-A2 `config/wall_sensor_config.hpp` (`kWallMountMaxHeightInches`:
  the robot file refuses a lens above the wall's printed top, 11.5 in).
  *Confidence:* **reasoned** (the wall heights are printed; the bracket's bottom measured off A8's printed scale —
  manual v2.0's drawings; v2.1 (2026-10-08) exists and was not readable on 2026-10-29: UNVERIFIED) ·
  **invented** (the overhang and the brackets' depth).
  *Settle:* L6 Step 0.6 (the overhang and the brackets' depth measured; readings at the mount's height at
  mid-Toggle AND at each bracket end) — then `toggleChecked` lifts the BODY for that side, never the brackets.
  *Blast radius if wrong:* with the body's keep-out lifted on a sensor whose cone reaches the overhang, the reset
  seats short by up to the overhang; a bracket in the beam does the same — on the host test plant, a beam whose
  ESTIMATE was 5.5 in off along the wall, clear of the bracket's span by the widening alone, seated more than
  0.5 in wrong where the truth's beam met a bracket, with the along-wall drift term switched off (with it on, the
  reset refused KEEP-OUT — `test/wall_reset_plant_test.cpp`, "the along drift beside a Toggle bracket").
  **Contained** by the body's keep-out staying ON until measured and the brackets' never lifting.

- [ ] **HA-209 — the sensor's update interval and latency are UNKNOWN.**
  *Claim:* PROS returns the sensor's latest value, so repeated reads within one update repeat it (N samples are
  fewer independent readings), and one `distance()` / `confidence()` pair can straddle an update (two device reads,
  `hal/pros/distance.hpp`).
  *Source:* `localization/wall_reset.hpp` (`minSamples` with `window` — the window's TIME is the real guard;
  `WallSampler`'s per-sample read-fault rule around both reads); `sim/field_plant.hpp` (`updateEvery`,
  `latencySteps`).
  *Confidence:* **invented** (the interval, the latency) · **reasoned** (the two reads, from the adapter's source).
  *Settle:* L6 Step 0.7 (a change timer on the bench page, a board moved slowly).
  *Blast radius if wrong:* a slow update makes the 0.15-s window hold fewer real readings than its count says — a
  spread of 0 proves less; a long latency reads where the robot was. Shortening the window "to speed the auton up"
  without the interval is the misuse to avoid.

- [ ] **HA-211 — the wall reset's gate defaults.**
  *Claim:* range **2–16 in** (2 in = VEX's 20-mm floor with room for the unpublished close-range behaviour; 16 in =
  where the published 5 % alone, 0.8 in, leaves no room under the 1.0-in bound); square within **5°**; spread
  **≤ 0.5 in**; **≥ 5 samples over ≥ 0.15 s** (window ≤ 0.5 s); confidence **≥ 0.5**; still **≤ 1 in/s, ≤ 3°/s**;
  error bound **≤ 1.0 in** (F2's pose target); keep-out margin **3 in** (the start pose's placement ~1 in + a field
  element's place 0.5 in + the beam centre's aim 2° × 16 in ≈ 0.56 in, summed ≈ 2.1, rounded up) **plus 10 % of the
  travel since the along-wall axis was last set** (robot two's saved distance scale reads ~8 % short until the
  Calibrate rerun; HA-83 believes 2 % σ on a calibrated robot); heading tolerance **2° + 2 % of the |rotation|
  since the heading was last set** (HA-20's floor, doubled; HA-84's 1 % of rotation, doubled); the beam-height
  ceiling's 6.5-in piece + 0.5 in and an **18°** cone while the mount's is unmeasured. The ONE policy number,
  `maxCorrection`, has no default — it is the author's. *Added by the L6-A1 review (2026-10-29):* "still" is the
  estimate's DISPLACEMENT over the trailing `window`, per second — not one tick's finite-difference velocity, which
  a still IMU's heading noise (HA-21) turns into 1.1–1.4 in/s of phantom speed on tracking wheels 3–4.5 in off the
  turning centre (the host test plant), refusing every reset; `window` at most **0.6 s** (the 64-sample buffer
  spans 0.63 s at 10 ms); and the Localizer counts turning in **1° steps** of the raw IMU heading and travel in
  **0.5-in steps** of the odometry's path (`kTurnCounterStepRadians`, `kTravelCounterStepInches`) — summed tick by
  tick, HA-21's jitter alone read ~335° of "turning" and ~32 in of "travel" per still minute (twice that at 0.1°),
  and a still robot's reset was refused IMPRECISE after ~45 s, KEEP-OUT after a minute at 0.1°. The steps assume
  two readings of a still IMU's heading never differ by a full degree (14 σ of the difference of two of HA-21's
  readings), and a still robot's odometry path never strays 0.5 in from a point it passed — whether real noise is
  white or integrated (HA-173) does not change that while its size is near HA-21's. *Where it stops holding*
  (measured on the host test plant at the L6-A1 verification round, 2026-10-30: a still minute on the default rig's
  offset wheels, twenty seeds, the hostile IMU's σ scaled): the travel counter reads 0 up to SIX times HA-21's σ.
  The turn counter reads 0 with noise alone up to THREE times; with HA-20's drift added (≤ 1°/min, counted as it
  should be) ~1° up to twice and up to ~3° at three times. Then a cliff — the step is 3.5 σ of the difference at four
  times, and a minute is 6,000 tries: at four times up to ~4° a still minute with noise alone and up to ~31° with
  the drift (a reading whose mean sits mid-step crosses it back and forth, two steps a trip); at five times up to
  ~100°; at six times 20–245° — and a still robot at six times refused IMPRECISE after 120 s in 8 of 10 seeds (a
  7-in reading, parked square). Rule 5's displacement on a still robot reads ~0.08 in/s at HA-21's σ (0.04–0.17
  over 600 resets, ten seeds), ~0.33 (up to 0.69) at four times, and reaches the 1-in/s limit near six times (2 of
  600 refused MOVING). An over-count only refuses — a wider heading tolerance, a larger error bound — and never
  seats a wrong reading.
  *Source:* `localization/wall_reset.hpp` (`WallResetLimits`, every field labelled `PROVISIONAL (A4: HA-211)`;
  `SideMount`'s tolerances; `kMaxWallWindowSeconds`); `localization/localizer.hpp` (`travelSinceSeat`,
  `turnedSinceHeadingSeed`, and their steps `kTurnCounterStepRadians` / `kTravelCounterStepInches`); since L6-A2
  `config/wall_sensor_config.hpp` (`WallSensorsConfig::limits` — the robot file's copy — and the spans
  `wallLimitsProblem` keeps a typed limit inside).
  *Confidence:* **invented** — each derived on paper from HA-204/205/209 and HA-20/21/83/84, none observed.
  *Settle:* L6 Step 0.3–0.8; the first field session's `[WALL]` lines; for the two steps, a still-robot IMU and
  odometry capture (60 s, the robot untouched): `turnedSinceHeadingSeed()` and `travelSinceSeat()` must stay at a
  step or two, and the capture reports the heading's σ (R5a's REST step, HA-21) against the turn counter's clear
  margin — 0 counted up to ~0.15° (three times HA-21) on the host test plant. A σ past that means the counter's
  input needs smoothing before it is stepped (a design change for a later chunk, not a constant).
  *Blast radius if wrong:* too tight refuses good resets (the auton goes on uncorrected — exactly as without the
  reset); too loose seats from worse readings. Overridable per robot in `WallResetLimits`.

- [ ] **HA-212 — the Loaders, the start pieces and VEX U's loads.**
  *Claim:* the Loaders are 4.02 in wide, protrude 3.74 in, centred ±58.77 in on the red and blue walls (A9/A10;
  the season header computes 58.764 from A10's 3275.8 − 1783.2 mm); A12's start pieces — Cup-Pin-Cup clusters 6.5
  in tall, centred 1.58 in off EVERY wall at |along| 18.81–28.28 in — stand there until moved; in VEX U, match loads
  enter through the Loaders during the AUTONOMOUS period (`<VUG4>`) into the 23.11-in Load Zones, where a robot
  taking them, or pieces it drops (nested or not), may stand at any time; no single Override piece is taller than
  6.5 in (a nested stack is); no other field element stands against a wall below its top.
  *Source:* `localization/override_walls.hpp` (the Loader, Load Zone and start-piece spans);
  `localization/wall_reset.hpp` (`WallResetLimits::pieceTop`); `sim/field_plant.hpp` (the plant's own copy).
  *Confidence:* **reasoned** — A9/A10/A12 (manual v2.0; v2.1, 2026-10-08, exists and was not readable on
  2026-10-29: whether it moved these is UNVERIFIED), FO-1, VEXU-1, VURS1.
  *Settle:* L6 Step 0.6 (a Loader's position and depth, and the start pieces' places, on a real field).
  *Blast radius if wrong:* a Loader or a piece outside its kept-out span can be read as the wall — the host plant:
  a Loader read as the wall with the keep-outs off seated 3.7 in wrong, inside a 4-in bound.

**R5a Part A added FIVE — HA-170 to HA-174.** The team lead ruled on 2026-09-26 that the drive's gains
are measured by the robot and kept on the SD card, never typed or committed as constants; R5a builds
that in four parts, and Part A is the measurements as library: the fit (`calibration/feedforward_fit.hpp`),
the GAINS step's ramp routine (`calibration/drive_characterizer.hpp`), the REST step's stationary capture
(`calibration/rest_capture.hpp`), the tick-timing statistics both record (`diag/tick_timing_stats.hpp`),
and per-wheel gains in the host test plant so a test can tell LEFT from RIGHT. **None of it has run on a
robot, and no program runs it yet** (shulib Calibrate gains the two steps in R5a's Part C). The host test
plant's motor model IS the law the fit fits, so recovering the plant's gains proves the arithmetic and
the safety rules — not physics. These five entries are what that leaves.

- [ ] **HA-170 — every number the GAINS and REST steps judge by is invented for a first run.**
  *Claim:* the values below let a healthy robot two finish both steps and refuse a sick one: the ramp at
  2 V/s to at most 10 V and the 6 V step; 36 in and 8 s per segment; the curve guard's asymmetry of 0.30
  (ψ_g = 36 × 0.30 / W, 49.5° at W = 12.5 in) against the 15 % asymmetry warning; a 12 in stop bound; a
  45 °C temperature guard (under HA-44's first throttle step); the fit's 2.3 A current guard, 1 in/s dead
  band, dt within ±10 % of 10 ms, τMin = dt/5, at least 100 samples a side, a residual of at most 3 in/s,
  and its ranges (kS 0–4 V, the implied free speed at 12 V 20–200 in/s, kA 0–0.2 V·s²/in); the REST
  capture's 60 s, 0.25 in of translation and 3° of rotation by the encoders, and the settle-margin warning
  at 2×. It also covers two numbers R5a's later parts add: the full-stick budget D3 keeps (a CHOICE, 79/7 V
  exactly, from HA-45 × HA-50; Part B) and the raw-sample buffer's size inside HA-59 (Part C).
  *Source:* `include/shulib/calibration/feedforward_fit.hpp` (`FitConfig`),
  `include/shulib/calibration/drive_characterizer.hpp` (`CharacterizerConfig`) and
  `include/shulib/calibration/rest_capture.hpp` (`RestCaptureConfig`) — every field PROVISIONAL (A4:
  HA-170). The rest floors, IMU wait and missed-tick ceiling both steps reuse are the drivetrain
  calibrator's and keep HA-136; the fight monitor keeps HA-132/133.
  *Confidence:* **invented.** The clear space the lane needs follows from them: 36 + 12 = 48 in ahead and
  behind, and 26.6 in to each side at W = 12.5 in (`CharacterizerConfig::sidewaysExcursion`: the arc of a
  segment curving up to the guard, 14.6 in, PLUS the whole 12 in stop bound). *Corrected 2026-10-04 (R5a
  Part A's review):* the 14.6 in first stated here left out the stop. The curve guard is judged only while
  a segment is powered; the stop that follows is not guarded and can keep curving, or curve harder than
  the powered part when one side brakes short — on the host test plant a run that finished normally,
  curving 46° a segment with stops near 11 in, swept 19.7 in sideways. Both numbers are per segment and
  assume three things the drive characterizer's header spells out: each backward segment drives back
  roughly along its forward one (nothing tracks the run's own position); the powered part curves at a
  roughly constant rate (a robot that turned the whole guard at once and then ran straight could go
  36·sin 49.5° ≈ 27.4 in sideways); and every stop stays inside `stopBound` — a stop past it is refused only
  after the robot has rolled it, so the lane's ends need room beyond the bound for that one stop.
  *Settle:* the serial logs of R5a's robot sessions (gains S2, rest S3); each number moves with a logged
  line, and a moved `stopBound` or curve guard moves the clear space with it.
  *Blast radius if wrong:* a bound too tight refuses a good run; too loose admits a bad one. Most of that
  is loud — every refusal names itself, and a finished run prints every number behind it — with ONE
  exception found on 2026-10-04: a REST capture admits a rotation under `turnBound` (3°), and a robot that
  really turns turns the IMU with it, so the drift's slope absorbs the turn — one nudge of size θ at
  mid-capture moves the drift by 1.5·θ/T, and a heading swung by up to `turnBound` moves it by up to
  3·`turnBound`/T: 4.5 to 9 °/min at the defaults, several times HA-20's whole bound, with no refusal.
  The capture now RECORDS what the encoders saw (`RestSummary::encoderTurnRad`, `encoderTurnMaxRad`,
  `encoderMoveMaxIn`) and states the bound (`driftTurnBiasBound()` = 3·`encoderTurnMaxRad`/T), so the bias
  is visible in the record, but it is not removed: whether `turnBound` should shrink, or the drift be
  regressed on the IMU heading minus the encoders' rotation, is open for the team lead. A `stopBound` or
  ψ_g sized wrong also changes the clear space the lane needs. **Contained:** config fields only.

- [ ] **HA-171 — a V5 drive side follows V = kS·sgn(v) + kV·v + kA·a closely enough that gains fitted
  over the ramp's range hold outside it.**
  *Claim:* the gains fitted over the volts a 36 in segment reaches — the range the fit admitted,
  recorded per side as `voltsMin`–`voltsMax`; on the host test plant about 1–6 V, because the ramp runs
  out of lane near 6 V and the steps hold 6 V — hold at the full-stick voltage and while turning; the
  fit's two channels are faithful — `IMotor::velocity()` (PROS
  `get_actual_velocity`, HA-96) is current within a tick with no filter or lag of its own, and a motor
  command takes effect within one tick (unregistered until now).
  *Source:* `include/shulib/calibration/feedforward_fit.hpp` (the discrete law it fits, v[k+1] =
  α·v[k] + β·u[k] + γ·sgn(v[k]); no differentiated velocity anywhere). **What the fit's PoorFit verdict
  does NOT catch** — measured on synthetic data generated off the law (2026-09-29; the reading noise
  2026-10-04), pinned by `test/feedforward_fit_test.cpp` M4–M7 (the host test plant has no drag, filter,
  latency or velocity-noise term of its own; its encoder-hostility run, the characterizer's C16, shows the
  noise bias on the plant as well):
  - **a quadratic drag never reaches PoorFit.** Swept from 0 to 0.011 V/(in/s)², the residual stays
    under 0.08 in/s (PoorFit is 3 in/s) while a larger kV and a smaller kS absorb the drag: at 0.0005 (a
    drag of 0.45 V at 30 in/s) kS reads 9.5 % low and kV 10.5 % high; the fit still says Ok at 0.0100
    (kS 0.024 against the true 1.0, kV 2.4× the truth); the first refusal is NonPhysical (kS < 0) at
    0.0105. PoorFit catches a reading the law cannot follow at all (white noise of 5 in/s), not a law
    that is subtly wrong;
  - a `velocity()` low-pass filtered over 1 / 2 / 3 ticks is fitted, not refused: kS +0.8 / +2.0 /
    +2.1 %, kV −0.25 / −0.6 / −1.0 %, kA +3.3 / +8.6 / +19 %;
  - a one-tick command latency is fitted, not refused: kS +2.8 %, kV −0.4 %, kA −0.6 %;
  - **noise on the speed reading is fitted, not refused, and biases the gains** — the one mismatch every
    robot has. A noisy speed is noise in a regressor (errors in variables): it pulls α toward 0, so kA
    reads LOW and kS HIGH, while the residual it leaves is only ~1.3·σ — the 3 in/s PoorFit bound calls a
    fit Ok up to a side-median σ of ~2.2 in/s. Over 40 seeds a level on the four segments' exact law
    (kS 1.0, kV 0.16, kA 0.02), every fit Ok, the mean relative error: σ 0.5 in/s — kS +3 %, kV +1 %,
    kA −18 % (single runs to kS +9 %); σ 1 in/s — kS +11 %, kV +1 %, kA −46 % (single runs to kS +26 %);
    σ 2 in/s — kS +6 %, kV +5 %, kA −72 % (single runs to kS +33 %, kV +11 %). The current guard (HA-172)
    makes it worse: excluding the step's current-limited opening ticks, which carry most of the
    excitation, multiplies the bias three- to fourfold (σ 0.5 in/s with a 100 in/s² limit: kS +14 %
    against +3 %, kA −56 % against −18 %). kS and kV are applied. The REST step's encoder speed σ at
    rest (`RestSummary::speedSigma`) is a floor on this σ, which turns the table into a bias estimate
    for S2.
  HA-48 is this entry's plant-side twin (the plant's shape is the same law).
  *Confidence:* **invented.**
  *Settle (S2, R5):* the residual per voltage band in S2's serial log; the laptop re-fit's comparison of
  the reported speed against the differenced position (R5a Part C writes both to the card); one
  full-stick run whose speed is predicted from the gains; the REST step's encoder speed σ read against
  the noise table above (whether `maxResidualRms` should be sized from a bias tolerance rather than 3 in/s
  is an open choice under HA-170).
  *Blast radius if wrong:* Teleop's top speed and every motion's feedforward are off by the bias; the
  PID absorbs it and settles slower. Only kS and kV reach a motor — a measured kA is recorded, not
  applied (the command pipeline calls the cruise form `Feedforward::calculate(v)`).

- [ ] **HA-172 — inside the ramp's range, the only non-linearity is the per-motor current limit.**
  *Claim:* the limit is the PROS default of 2500 mA (the vendored `motors.h` figure HA-154 also cites);
  `IMotor::current()` reports it within a tick, so a sample with any member at or above 2.3 A can be
  excluded; the firmware adds no slew of its own to `move_voltage`; the motors are below HA-44's first
  throttle step, which the 45 °C guard enforces. **Whether VEXos lowers the per-motor limit when more
  than eight motors are plugged in is not stated in the vendored text and is UNVERIFIED** — robot two
  carries 12 (10 drive, 2 claw pivots).
  *Source:* `include/shulib/calibration/feedforward_fit.hpp` (`FitConfig::currentGuard` and the
  OverCurrent exclusion, labelled HA-172); the GAINS step (`calibration/drive_characterizer.hpp`) feeds
  it each side's largest member current every tick and holds the temperature guard (HA-170).
  *Confidence:* reasoned (the limit) · **invented** (the timing, the motor-count question, no firmware
  slew).
  *Settle (S2, R5):* the step segments' current column and its plateau in S2's raw samples; one reading of
  `get_current_limit` on robot two.
  *Blast radius if wrong:* current-limited samples that slip past the guard bias kS and kV, which ARE
  applied (Teleop's top speed, every motion's feedforward); a torque-limited step also inflates kA, which
  is recorded only — it reaches the report and R6's plant back-fit (a too-sluggish plant), and would reach
  a motor only through a future acceleration channel. HA-49 is the plant-side twin.

- [ ] **HA-173 — a stationary V5 IMU's drift is a PER-BOOT bias, and one 60 s capture may not separate it
  from noise.**
  *Claim:* (a) the drift is HA-20's per-boot bias: one REST capture measures that boot, never a property
  of the robot — a stored drift is a sample for this register and is never applied as a heading
  correction at a later boot; (b) whether one capture separates the bias from the noise depends on the
  noise's STRUCTURE, which the host cannot show. With WHITE heading noise (the host's hostile IMU; HA-21's
  0.05°) the slope's σ is σ·√12/(√N·T) ≈ 0.002 °/min at 60 s and 100 Hz. With INTEGRATED rate noise (a
  gyro's heading is its integrated rate; HA-22's 0.5 °/s per 10 ms sample) it is ≈ 0.42 °/min at 60 s and
  0.19 °/min at 300 s — HA-20's typical drift itself, so one boot cannot separate them and only the spread
  across boots can. Under that structure the settle floor's derivation √2·σ/dt also overstates the rate
  noise `SettledUtil` sees (on the host test plant's scripted walks it reads 0.14–0.29 rad/s against a true
  ~0.009 rad/s, and would warn falsely against the 0.30 rad/s floor in seven walks of eight).
  *Source:* `include/shulib/calibration/rest_capture.hpp` — `RestCapture` reports the drift with its σ
  under BOTH models and the structure ratio ρ = σ_Δ/(√2·σ_line) (≈ 1 white, ≪ 1 a walk: 0.03–0.06 on the
  host test plant's walks); `settleFloorReport()` compares HA-51's floors with the MEASURED per-tick step,
  never √2·σ/dt. Nothing applies a drift anywhere. *Two ways a boot's numbers mislead, found 2026-10-04
  (R5a Part A's review), and how each is now visible in the summary:* (1) a robot nudged UNDER the
  capture's `turnBound` turns the IMU with it — the drift absorbs up to 3·`turnBound`/T (1.5·θ/T for one
  nudge of θ at mid-capture; a 2.1° pivot on the host test plant read as −3.2 °/min of drift on an IMU
  with none) and ρ drops toward 0, so the nudge also reads as random-walk evidence. The summary records
  the encoders' rotation (`encoderTurnRad`, `encoderTurnMaxRad`) and `driftTurnBiasBound()`; a boot whose
  drift is not well clear of that bound is a nudge, not a sample (HA-170). (2) ρ is undefined when the
  heading has no noise about its line (σ_line = 0 — an IMU that reports a constant): it is reported as 0,
  and `structureRatioDefined()` says it means nothing.
  *Confidence:* reasoned (the per-boot bias) · **invented** (the structure).
  *Settle (S3, R4):* REST on ≥ 10 separate boots; ρ says which structure holds, and the spread between
  boots is compared with the within-run slope σ of that structure — leaving out any boot whose
  `driftTurnBiasBound()` is not small against its drift, and reading ρ only where it is defined.
  *Blast radius if wrong:* if the bias were stable per robot, an easy correction is left unused — a missed
  gain, not an error. If the structure is a walk, HA-51's heading-rate floor was sized on the wrong
  formula, and the REST step's margin line shows by how much. A nudged boot left in the spread inflates
  it, and pushes the structure verdict toward "walk".

- [ ] **HA-174 — gains measured once hold for the match.**
  *Claim:* the gains measured on the same foam tiles (not carpet or a bench top), on a charged pack
  (HA-47), with the same drive members, hold through a match. A member ABSENT at a later boot leaves kV
  roughly unchanged (the motors on one train share the back-EMF) but raises kS roughly as 1/(members
  present); kS IS applied. kA rises the same way, and is recorded only.
  *Source:* `include/shulib/calibration/drive_characterization.hpp` (`SideCharacterization::members`
  keeps each side's member count at measurement, so R5a's Part B can say when fewer are present).
  *Confidence:* **invented** (the surface) · reasoned (the members).
  *Settle (S2, R5):* the GAINS step on tiles and again on another surface; one run with a member
  unplugged.
  *Blast radius if wrong:* creep at low speed, or a settle that stalls short. The member-count note R5a
  Part B adds makes the second half visible.

### CAL1's addition — the standard Calibrate's watch reads a declared motor and changes nothing (2026-10-30)

**CAL1 Part A2b (2026-10-30) added ONE — HA-187.** The standard "shulib Calibrate" (`calibration/
calibrate_program.hpp`, its robot shell `hal/pros/calibrate.hpp`) WATCHES every motor a team declares as a
mechanism: it reads each one as the push begins and again at DONE, and one that turned with the push like a
drive motor HOLDS the save on MECHANISM PORT TURNED (SAVE keeps it declared; RUN AGAIN saves nothing). It must
never command, configure or brake such a motor — a claw that is geared, set to coast or pulsed can drop or
strain (T3's rule) — so its read is PROS's C API by port, with no motor object of any kind. This entry is the
belief that read rests on.

- [ ] **HA-187 — the declared-port watch reads a mechanism motor and changes nothing.**
  *Claim:*
  1. The three C reads write nothing to the motor. In the vendored `firmware/libpros.a` (`vdml_motors.c.o`),
     `motor_get_position`, `motor_get_gearing` and `motor_get_encoder_units` each call only PROS's registry check
     (`registry_validate_binding`, `registry_get_device`), the port mutex and `__errno`, plus ONE firmware getter
     (`vexDeviceMotorPositionGet`, `…GearingGet`, `…EncoderUnitsGet`). There is no `*Set` call and no motor
     object. The getters are closed firmware, so "a Get writes nothing" is reasoned, not seen.
  2. On a motor this program never configured, `motor_get_position()` returns the shaft's position in the units
     `motor_get_encoder_units()` reports, scaled by the gearing `motor_get_gearing()` reports, whatever
     configuration the motor then has. Both stay the same between the watch's two reads (the start and the end
     of one push), so the difference is the shaft's turn, within the reported gearing's scale. The watch's 5°
     threshold (the push's own test for a drive port, HA-136) sits far below a 48 in push's ~1,700° for any
     cartridge.
  3. Reading a motor's position does not disturb it (a claw left where it was stays there).

  *Source:* the disassembly above (2026-10-08, `arm-none-eabi-objdump -dr`, for the CAL1 brief);
  `include/pros/motors.h`; `hal::pros::ProsCalibrateIo::readShaftDegrees()` in
  `include/shulib/hal/pros/calibrate.hpp` (degrees as reported; rotations × 360; counts × 360 over 1800, 900 or
  300 by the reported gearing; false on `PROS_ERR_F` or a unit or gearing it does not know). The host suite runs
  that read through the PROS shim and counts ZERO gearing, units, voltage, brake-mode and `brake()` writes on
  every declared port in every scenario, and two position reads per push the watch judges
  (`test/calibrate_golden_test.cpp`) — but the shim transcribes this belief, so it tests the shell against
  the belief, not the belief itself.
  *Confidence:* (1) **reasoned**; (2) and (3) **invented**. None measured.
  *Settle:*
  - (2) and (3) at Station G: with robot two's claw pivots declared, turn one by hand during the push. The watch
    names that port and only that port, the save waits on MECHANISM PORT TURNED, and the other pivot has not
    moved. Read an unconfigured claw's `motor_get_gearing` there too, before any program gears it — it decides
    the evidence line's scale (the blast radius below).
  - (1) needs a read of a pivot's gearing and units before and after a Calibrate run, with no other program in
    between. That is an extension of Bench Tests' raw probes, unowned (like HA-166's settle (b)). The design
    does not depend on it.

  *Blast radius if wrong:*
  - (1) a getter that wrote would leave the claw's motor with some configuration until the next program
    configures it (every program that drives the claws writes its own gearset at construction, `ProsMotor`). No
    motion results, and nothing is commanded.
  - (2) a missed watch line, or a false MECHANISM PORT TURNED hold. The hold costs one touch (SAVE), and a missed
    one is the state before the watch existed (no catch for a drive motor declared by mistake). **And the held
    screen's EVIDENCE depends on the gearing the motor REPORTS** (CAL1 A2a's review, 2026-10-29): nothing in
    Calibrate gears a declared motor, while the drive's travel comes from motors geared to the STATED cartridge,
    so an ungeared declared port that reports another gearing than its insert shows the MEDIUM `port N: X deg,
    drive Y deg` line off by the gearing ratio — a BLUE drive motor reporting green shows about ⅓ of the drive's
    travel, which reads as "not a drive motor" and invites SAVE. Detection is unaffected (5° against ~1,700°);
    the operator's judgement between SAVE and RUN AGAIN is what a wrong scale misleads.
  - (3) a claw that drifts while read: not expected of a read.

  **Contained:** one program's serial lines, one held screen and one NOTE. In Calibrate the read is one
  function (`readShaftDegrees`). It is not the only reader: Bench Tests' raw motor report and its motor
  watch read motors by port through the same C calls (`motor_get_position`, `motor_get_gearing`), so a getter
  that wrote would touch their reads too — still with no motion and nothing commanded.

---

## Group R4 — noise, drift, latency, timing, power, traction (characterization)

The numbers behind every hostile model. A3's convention: the failure **shapes** are confident and
test-pinned; the **magnitudes** are these entries. R4 replaces each with a measurement and re-runs
the suite (the models take the measured values by config — zero code motion). None of these are
settleable before hardware, and none block any host chunk.


- [ ] **HA-123 — a single tracking-pod or drive-side travel delta above 36 in in one tick is
  corruption, not motion.**
  *Claim:* `PilonsOdometry` treats |Δtravel| > `maxTickTravel` (default 36 in) from either pod
  in one `update()` as implausible, and `DriveEncoderOdometry` applies the same bound, with the
  same default, to each DRIVE SIDE's travel (since R3b Part 2, 2026-09-14 — its config field
  carried this entry's label from the start, but the entry itself spoke only of tracking wheels
  until DIAG1 extended it, 2026-09-26). The number is a SANITY bound, not a measurement: it is
  chosen to be unreachable by a real drivetrain at any plausible loop rate, so it can only
  catch corruption that is orders of magnitude out.
  *Source:* introduced at DEFECTS1 (item N1) — a tracking pod that is dead or not yet
  enumerated at construction reads 0, so `TrackingWheel` baselines at 0 and, on the tick the
  pod finally answers, differences its TRUE cumulative position against that 0. Measured at
  **28.4 in** of phantom translation for a pod waking at 1000°, and unbounded in general.
  Before this, `PilonsOdometry` gated |Δθ| and never |Δtravel|, so the jump passed in silence.
  The drive-encoder side has the same failure shape where a side is ONE motor: a drive port
  that enumerates late, or whose adapter holds a last-good value and then wakes
  (`localization/drive_encoder_odometry.hpp`). Where a side is a motor GROUP (robot two: four
  motors a side) the odometry reads the members' MEDIAN, so one such member is absorbed and the
  bound never sees it — a drive-side trip there means enough members jumped together to move
  the median (two of four). The group-level detector for one bad member is the coupled-side
  monitor (`MOTOR_GROUP_DISAGREE`), which samples member velocity; whether it catches a member
  whose velocity is held too is not established. (In DRIVER CONTROL, since T4 Part 2, the
  library's drive protections look for exactly that member — its position AND velocity repeating
  while its side turns — and mark it ABSENT after 250 ms, out of the median from then on: HA-175.
  In an autonomous motion nothing does.)
  *Confidence:* **invented.** Nobody has measured a real loop rate under load, and neither class
  holds a clock, so the bound is **dt-BLIND** — the same 36 in is 300 ft/s on a 10 ms tick and
  15 ft/s on a 200 ms one. That is why it is generous rather than tight, and why the gate
  REPORTS rather than withholds: a dt-blind bound that could stop odometry accumulating on a
  slow loop would be a worse failure than the jump it prevents.
  *Settle (R3/R4):* measure the real control-loop period under load, then re-express this as a
  velocity bound (`maxTickTravel = vMax × dt`) using the same clock the loop reads. R4 owns the
  loop-rate measurement; until then this stays a magnitude check. Since DIAG1 a trip names
  itself on the fault line — `implausible odometry tick why=TRAVEL … L=… R=…` (`fwd=`/`lat=` for
  pods) — so any trip seen on a robot says which side or pod, and by how much.
  *Blast radius if wrong (too tight):* legitimate fast motion on a slow loop is flagged
  implausible, which HealthMonitor turns into `ODO_STUCK` — and `ODO_STUCK` is the motion
  scheduler's DEFAULT ABORT fault, so a spurious trip **ENDS the active motion** (an autonomous
  stops where it is). **Corrected 2026-09-26 (DIAG1):** this entry said "noisy, not dangerous,
  because the delta is still integrated". The integration half is true; the "not dangerous"
  half was not, and was never true while `ODO_STUCK` aborted by default. **If too loose:** a
  smaller phantom jump stays invisible, which is the pre-DEFECTS1 behaviour. **Contained:** one
  config field and one comparison per odometry; the POSE is unaffected either way (the delta is
  integrated), the active MOTION is not.
- [ ] **HA-160 — the V5 IMU's heading is continuous tick to tick while it reports ready.**
  *Claim:* while `isReady()` is true, two heading reads taken one control tick (~10 ms) apart
  differ by no more than the robot actually turned in between — a few degrees at most, never
  tens. The library has always rested on this without saying so: heading is IMU-OWNED (the
  pose's heading is set equal to the reading every tick), and each tick's Δθ — the arc
  `arcStep` integrates along, and the drive-encoder heading cross-check — is the difference of
  two consecutive reads.
  *Source:* implicit since M2; registered by DIAG1 (2026-09-26). Consumed by
  `localization/drive_encoder_odometry.hpp` and `localization/pilons_odometry.hpp`. Policed only
  coarsely: the odometry trust gate trips at |Δθ| > π/2 per tick (`maxTickRotation`), and during
  a SCHEDULED motion the scheduler's pose-delta guard raises `IMPLAUSIBLE` above its yaw-rate
  envelope (that guard does not run under driver control) — so under driver control a swing
  smaller than 90° in one tick passes in silence.
  *Confidence:* **invented, and QUESTIONED by the 2026-09-25 robot evidence** (robot two, the
  library's graph under open-loop driver control). In one start, during ~0.35 s of reversing at
  up to ~34 in/s, the heading RECONSTRUCTED from the logged commands swung by up to 37° between two
  consecutive 10 ms records — of the last nine such steps, eight were 16.9° or more, alternating
  in sign, and one was 1.4° — and 37° in 10 ms would be a robot spinning at ~3700°/s; an
  `implausible odometry tick` fault fired on the first tick the command returned to zero. That
  is an INFERENCE, not a reading: the log of that day carried no heading field, so the heading
  was recovered from the field-frame command (assuming the tank's body command had no lateral
  part), and the fault line then named no check. The cause is UNKNOWN — an IMU data glitch, a
  mounting or vibration effect under a hard reverse, or something between the sensor and the
  read are all open.
  *Settle (R3/R4):* record the raw IMU heading every tick (not reconstructed) through hard
  reverses and stops, wheels up and on the floor; any consecutive-read step larger than the
  robot can turn in 10 ms falsifies the claim. Since DIAG1 a step over 90° also names itself on
  the fault line (`why=ROT dth=… h0=… h1=…`).
  *Amended R5a Part A (2026-09-29; worded 2026-10-04):* the GAINS step's StepBackward segment
  reproduces the condition of that hard reverse — a reverse from rest at a held 6 V (whether the
  2026-09-25 episode itself began from rest was not established). The step is built as library (R5a
  Part A; shulib Calibrate runs it from Part C, and no robot has run it): every segment records the
  largest tick-to-tick heading step and the IMU's accumulated yaw beside the encoders' curve (their
  difference is this entry's evidence), and the IMU never refuses the run — its curve guard reads the
  encoders, so a 37° jump is recorded, not obeyed (host-tested).
  *Blast radius if wrong:* the pose's heading jumps with the reading, and every tick's arc is
  integrated along the wrong average heading, so position is corrupted too; a step over 90°
  trips `ODO_STUCK`, which ENDS the active motion under the default fault policy (HA-123's
  corrected blast radius). **Contained:** detection only — the trust gate, the pose-delta guard
  and now the fault line's values. Nothing filters the IMU heading, deliberately: a filter would
  lag every real turn to hide a fault whose cause is not yet known.
- [ ] **HA-165 — the members of one drive side stay close enough in cumulative position that
  leaving members out of the side's median is a small step.**
  *Claim:* the PRESENT members of one drive side (robot two: four or five coupled motors) stay
  close enough in CUMULATIVE position, through a match's driving and slip, that leaving one or
  more of them out of the side's median moves the median by far less than `maxTickTravel` (36 in,
  HA-123) divided by the side's inches per radian. Since DIAG1 Part B (2026-09-29) a member whose
  reading is not finite (NaN, ±Inf) is left out of that read, like an absent member — so on the
  tick it drops out the side's reading can STEP with nothing moving, and step back on the tick it
  returns. The step, for ONE member left out: of an odd count (three or more), at most half the
  larger of the two gaps beside the middle value in sorted order; of an even count, half the gap
  between the two middle values; on a two-member side, half the gap between the two. For SEVERAL
  left out, the median of the rest can land anywhere between the lowest and the highest member
  position: the step is bounded only by the members' SPREAD, and its worst case is the member
  farthest from the side's median, read alone.
  *Source:* `include/shulib/hal/motor_group.hpp` (the label, in its "A READING THAT IS NOT A
  NUMBER" section); `test/median_nonfinite_test.cpp` case DIAG1 N7 — the exact step and its return
  on `[100, 100.5, 101, 101.5, 80]` (∓0.25); the MIDDLE member of `[80, 100, 100.5, 102, 103]` left
  out (+0.5, the midpoint of its two neighbours); a step past the bound on a two-member side
  (`[0, 100]` → `L=-50.000in`, `why=TRAVEL`, one fault line); and four of five members left out at
  robot two's 1.625 in/rad, the drifted one read alone — at 80, a -33.3125 in step, under the
  bound and NOT flagged (on the unchanged tree those four `+Inf` members reached the middle and the
  tick froze as `NONFINITE+TRAVEL`), and at 69, -51.1875 in, `TRAVEL`. Both files carry the string
  HA-165.
  *Confidence:* **invented.** The members' spread over a match has never been recorded. The
  evidence that it can grow is HA-130 — robot two's port 18 travelled about 20 % short of its
  side-mates on both hand pushes measured, so if that holds under power (unmeasured) its offset
  grows with the distance driven. It is exercised only when a member's reading is not finite,
  which HA-08's screen says never reaches a group on a V5.
  *Settle (R4):* after a match-length drive, read every member's cumulative position and compute,
  for the saved record's member set, the largest distance of any member from the side's median
  times the side's inches per radian: the worst step (that member read alone). The spread
  (highest minus lowest, times inches per radian) bounds it; with one member left out the step is
  at most half the larger gap beside the middle value.
  *Blast radius if wrong:* one tick carries the step and the return tick carries it back. They
  cancel in the side's travel, and in the pose only up to the heading turned between them. A side
  step under 36 in moves the pose (by half of it) SILENTLY — and that now includes a side whose
  infinite members used to reach the middle, which read non-finite and froze the tick as
  `NONFINITE+TRAVEL` (raising `ODO_STUCK`) and now integrates the rest's median; over 36 in,
  `why=TRAVEL` → `ODO_STUCK` → the active motion ends (HA-123's blast radius). **Contained:** HA-08
  must fail first — a non-finite reading has to get past the adapter's screen before this step
  exists at all.
- [ ] **HA-20 — IMU per-boot rate bias is ≤ 1°/min (typical 0.1–0.5°/min).**
  *Claim:* a calibrated V5 IMU's per-boot yaw-rate bias magnitude does not exceed 1°/min.
  *Source:* `include/shulib/sim/hostile/imu_hostility.hpp:71` (`rateBiasMax`); consumed by the
  live M2 acceptance test (`test/accuracy_spec_test.cpp:53`).
  *Confidence:* **invented** — a pessimistic community-folklore bound, not a measurement.
  *Settle (R4):* ≥ 10 boots, 60 s stationary each, log heading vs time; the fitted slope
  distribution replaces both the bound and the "typical" range. Build-order R4 names the 60 s
  drift number as a deliverable.
  *Settle path amended R5a Part A (2026-09-29; worded 2026-10-04):* that capture is built as shulib
  Calibrate's REST step (`calibration/rest_capture.hpp` — library only; Calibrate runs it from R5a
  Part C) — one boot per run, ≥ 10 boots; it reports the slope with its σ under both noise structures,
  and the encoders' rotation that bounds how much of the slope a nudge could be (HA-173). A measured
  drift is a sample for this entry, never a correction.
  *Blast radius if wrong:* **the F2 `< 1°` ceiling itself.** At exactly 1°/min the margin is zero
  by construction (1°/min × 60 s = 1°); A3 measured the stack adding ~nothing on top (worst
  end-of-60s 0.912°, worst instantaneous 1.065°). If the real bound is worse, the M2 acceptance
  fails on hardware **with the stack blameless**, and E3's heading correction stops being margin
  and becomes a requirement for F2. If better (likely), everything gains margin. Either way: a
  config value and re-run, no logic change — the acceptance test's scope note already says
  exactly this.

- [ ] **HA-21 — IMU heading white noise σ ≈ 0.05°.**
  *Source:* `imu_hostility.hpp:73`. *Confidence:* **invented**.
  *Settle (R4):* stationary high-rate log; σ of the detrended heading stream.
  *Amended R5a Part A (2026-09-29):* the REST step reports that σ (about the fitted line, never the
  mean). "σ" here presumes WHITE heading noise; if a V5 heading is integrated rate noise instead, this
  number is the wrong description of it — see HA-173 and the structure ratio the REST step reports.
  *Blast radius if wrong:* `SettledUtil` deadbands and E4's measurement covariance were sized
  against fiction — loops may chatter at settle (σ larger) or tolerances are needlessly loose
  (σ smaller). Constants only. *Added L6-A1 (2026-10-29):* the Localizer's wall-reset counters
  count in steps sized against this σ (1° of heading, 0.5 in of odometry path — HA-211). On the host
  test plant (a still minute, twenty seeds) the travel counter stays at 0 up to six times this σ, and
  the turn counter stays at 0 with noise alone up to THREE times it (with HA-20's drift added, ~1° up
  to twice it — the drift itself — and up to ~3° at three times). Past that it falls off a cliff: at
  four times up to ~4° a still minute (~31° with the drift), at five times up to ~100°, at six times
  20–245° (HA-211 has the numbers). An over-count only widens the reset's heading tolerance — it
  refuses more resets, never seats a wrong one.

- [ ] **HA-22 — IMU yaw-rate white noise σ ≈ 0.5°/s.**
  *Source:* `imu_hostility.hpp:75`. *Confidence:* **invented**.
  *Settle (R4):* same stationary log, rate channel.
  *Amended R5a Part A (2026-09-29):* the REST step reports the yaw-rate channel's mean and σ, and the
  σ of the per-tick heading step over dt — the rate noise `SettledUtil` actually sees (HA-51, HA-173).
  *Amended IMU1 Part A (2026-10-29):* a REST capture made on a build BEFORE IMU1 has a contaminated rate
  channel — DERIVED from the adapter's code and PROS's documented 10 ms copy (HA-213), never measured on a
  robot: the settle gate's read and the capture's first read share the tick the capture starts
  (`calibration/rest_capture.hpp`'s settle read, `beginCapture()`, the capture's read), so the first sample
  was a second read of the same tick, 0 or a spike of one 10 ms copy over a few microseconds: on the host test
  plant (the IMU's 10 ms copy modelled, HA-213) `yawRateSigma` read 4.5e-2 rad/s in the band and 2.25e-4 off
  it, against a clean 5.6e-7 (IMU1's T10). **A REST record from a pre-IMU1 build is not this entry's
  evidence.** On an IMU1 build ONE residual remains (HA-214, the hold's limit (2)): when the IMU's copy lands
  between the settle read's position in the tick and the capture read's (a band about one device read wide),
  exactly ONE capture sample is about twice the drift rate — σ 2.2e-4 rad/s on the host test plant against a
  clean 5.6e-7, still far below this entry's claimed 0.5°/s (8.7e-3 rad/s), so it cannot fake the verdict;
  it is named so a REST record is read knowing it (IMU1's T10 (b) pins it).
  *Blast radius if wrong:* E-phase rate weighting and any rate-based rejection thresholds
  (E2 high-yaw-rate gate) miscalibrated. Constants only.

- [ ] **HA-23 — IMU calibration takes ≈ 2 s and emits moving garbage while not-ready.**
  *Claim:* `isReady()` stays false ~2–3 s from power-on, and readings during the window are
  large-magnitude and *changing* (not frozen zeros).
  *Source:* `imu_hostility.hpp:69` (`calibrationEnd`) and `:79` (`calibrationGarbageRate` — shape
  only).
  *Confidence:* reasoned — VEX documents ~2 s calibration; the garbage-that-MOVES shape is
  community-observed and is the exact pathology that exposed the A3 boot-poison defect.
  *Settle (R3/R4):* log heading + `isReady()` from power-on across boots; characterize window
  length and in-window behavior.
  *Blast radius if wrong:* if real boots are *longer*, autons that don't wait violate the
  documented consumer contract (motion before quality leaves `Uninitialized` is unaccounted);
  if in-window readings are *frozen* rather than moving, the Localizer's boot guard is
  over-defensive (harmless). The guard itself is shape-robust — pinned against both a garbage
  window and a ready-from-construction boot.

- [ ] **HA-24 — IMU end-to-end heading latency ≈ 10 ms.**
  *Source:* `include/shulib/sim/hostile/latency_hostility.hpp:54`. *Confidence:* **invented**
  (smart-port refresh cadence used as a stand-in for an unmeasured pipeline).
  *Settle (R4):* step-response rig — snap the robot through a known angle, cross-correlate
  commanded/physical motion against the reported stream.
  *Blast radius if wrong:* transient heading error scales as ω·L (at 1.5 rad/s, 20 ms is already
  1.7° — transiently past the entire F2 budget), and the stop-drain phantom translation scales as
  ω·L·|offset| (A3 §3.6). E2's latency compensation consumes the real number; a wrong guess here
  mis-sizes it. Also feeds HA-35.

- [ ] **HA-25 — encoder refresh ≈ 1 tick (10 ms).**
  *Source:* `latency_hostility.hpp:56–57` (documented; defaulted 0 in the composed model,
  exercised explicitly by test). *Confidence:* reasoned — the smart-port refresh cadence is
  documented; the effective end-to-end value is not measured.
  *Settle (R4):* same rig as HA-24 on a driven wheel.
  *Blast radius if wrong:* wheel-travel/heading pairing skew inside each odometry tick — bounded,
  small, and precisely the class the settle window + E2 compensation absorb. Constants.

- [ ] **HA-26 — GPS on-strip position noise σ ≈ 0.7 in/axis.**
  *Source:* `include/shulib/sim/hostile/gps_hostility.hpp:73`. *Confidence:* **invented**
  (field-test folklore).
  *Settle (R4):* stationary on-strip logs at several field positions; per-axis σ.
  *Blast radius if wrong:* E2's R and gate widths; fusion weighting quality. Constants only —
  and the decimation/double-count *hazard* the corrector must survive is magnitude-independent
  and already pinned.

- [ ] **HA-27 — GPS heading noise σ ≈ 1°.**
  *Source:* `gps_hostility.hpp:74`. *Confidence:* **invented**.
  *Settle (R4):* same logs, heading channel.
  *Blast radius if wrong:* almost none structurally — heading is IMU-owned by decision #4 and no
  corrector can rotate the robot; the number only informs whether GPS heading is ever worth a
  cross-check. Recorded for honesty, not fear.

- [ ] **HA-28 — GPS fresh-fix cadence ≈ 50 ms.**
  *Source:* `gps_hostility.hpp:75` (`updatePeriod`). *Confidence:* **invented**.
  *Settle (R4):* timestamp distinct fixes in a long log; the inter-fix distribution.
  *Blast radius if wrong:* the double-counting hazard's severity for E2 (a ~100 Hz consumer sees
  each fix N times; N is this entry) and the information rate available for correction. Constants.

- [ ] **HA-29 — GPS claims rms ≈ 1.0″ when healthy / ≈ 99″ at no-fix — and its self-estimate is
  not its real error.**
  *Claim:* `get_error()` reports ~an-inch-class value on-strip and a sentinel-large value at
  no-fix; the gap between claimed and actual error is real and must be survived.
  *Source:* `gps_hostility.hpp:76–77` (`reportedRms`, `noFixRms` — deliberately decoupled from
  the actual noise σ in the model).
  *Confidence:* **invented** (both numbers and the size of the claim-vs-truth gap).
  *Settle (R4):* joint log of `get_error()` vs measured error on- and off-strip.
  *Blast radius if wrong:* E2's *adaptive* R maps claimed→trusted; a wildly different mapping
  changes gating behavior. The design premise (never trust the claim raw) already assumes this
  entry is only approximately right. Constants.

- [ ] **HA-30 — GPS end-to-end latency ≈ 50 ms.**
  *Source:* `latency_hostility.hpp:55`. *Confidence:* **invented** (capture+solve+transport
  guess).
  *Settle (R4):* move at a known velocity across the strip; cross-correlate reported vs true
  position streams.
  *Blast radius if wrong:* E2's lever-arm/latency compensation mis-sized (a v·L position skew per
  fix); **HA-35's adequacy directly depends on this number** — see there. Constants.

- [ ] **HA-31 — what the real GPS serves while it has no fix is unknown (the model serves
  origin-until-first-fix, then stale).**
  *Claim (falsifiable):* the real device's no-fix `get_position()` behavior — whatever it is —
  is finite and is *screened by the adapter* so no code path ever consumes it as a pose.
  *Source:* `gps_hostility.hpp:15–22` (model semantics + the register note).
  *Confidence:* **invented** (the model's origin choice is deliberately adversarial, chosen so
  code that trusts a no-fix pose gets dragged to (0,0) and caught — not a prediction of the
  device).
  *Settle (R3/R4):* observe raw off-strip/covered behavior; write the adapter screen against
  what is actually seen (sentinels → HA-08).
  *Blast radius if wrong:* none while the `hasFix` contract holds — the IGps contract
  (no-fix pose is unspecified-but-finite, callers must check) is exactly what makes the real
  behavior not-matter. This entry exists so R3 *verifies the contract holds on the real device*
  rather than assuming the model was prophetic.

- [ ] **HA-32 — a loaded V5 sustains the ~100 Hz loop.**
  *Claim:* the full stack (odometry + localizer + motion + telemetry) completes a tick in ≲10 ms
  sustained, keeping per-tick Δθ ≪ 1 rad — the premise behind arcStep's wrap-horizon
  precondition and the pilons trust-gate calibration.
  *Source:* `include/shulib/localization/arc_step.hpp:49–56` (precondition);
  `include/shulib/localization/pilons_odometry.hpp:33–39` (gate note; re-pointed at DIAG1, whose
  header lines moved it);
  `composed.hpp` `JitterScheduleConfig.nominal{0.01}`.
  *Confidence:* reasoned — 10 ms is the standard PROS task cadence and the M2 stack is small;
  unmeasured under our full eventual load (E4 EKF + sinks + vision).
  *Settle (R3, re-checked each phase):* `LoopMonitor` (A1) on the first real runs — it exists
  precisely to measure this; overruns raise `LOOP_OVERRUN` with the observed dt.
  *Amended R5a Part A (2026-09-29; worded 2026-10-04):* the GAINS and REST steps built for shulib
  Calibrate (library in R5a Part A; Calibrate runs them from Part C) record their loop's dt
  statistics (`diag/tick_timing_stats.hpp`: mean, σ, min, max, ticks at or over 15 ms, ticks over
  2× nominal) — PARTIAL evidence: Calibrate's loop does less work per tick than a motion loop, so its
  timing is a floor for a motion loop's, not a measurement of it.
  *Blast radius if wrong:* slower loops enlarge per-tick Δθ (aliasing horizon approaches — at
  even 20 ms the margin is still enormous for real yaw rates), coarsen control discretization
  (the derived instability thresholds in the closed-loop tests scale with dt), and inflate all
  latency-relative effects. Structure survives — the suite already passes under ×5 jitter
  stalls — but budgets get recalibrated. Watch, don't fear. *(IMU1 Part A, 2026-10-29:)*
  `ProsImu`'s rate hold groups a tick's reads by TIME (HA-215): a tick body whose span between its
  first and last rate read nears 5 ms, or a cadence far off 100 Hz, moves which reads share an answer.

- [ ] **HA-33 — PROS sensor reads are non-blocking (µs-class, no ms-scale stalls).**
  *Claim:* `get_position()`/`get_rotation()`/GPS reads return cached values quickly rather than
  blocking on the bus.
  *Source:* no in-tree code assumes a *number* — the assumption lives in the loop-budget math of
  HA-32 ("PROS call latency" was named as seed content when this register was planned; the R1
  adapters, when written, inherit this entry).
  *Confidence:* **invented** — PROS documents nothing about call cost.
  *Settle (R4):* microbench each adapter call with `IClock` timestamps over thousands of calls;
  record worst-case.
  *Blast radius if wrong:* HA-32 breaks from below — the loop budget is spent inside reads; the
  fix is batching/caching in adapters (an R1-layer change, core untouched).

- [ ] **HA-34 — loop jitter ±20% with ~2% spikes at ×5 nominal.**
  *Source:* `include/shulib/sim/hostile/composed.hpp:48–50, 205–207` (`JitterScheduleConfig`).
  *Confidence:* **invented** — PROS task-contention statistics are unmeasured.
  *Settle (R4):* dt histograms from real-run telemetry (the A1 `DebugRecord` carries per-tick
  dt; `LOOP_OVERRUN` counts spikes).
  *Amended R5a Part A (2026-09-29):* the GAINS and REST steps' timing statistics (HA-32's note) will be
  a partial settle once a robot session runs them — a lighter loop than a motion's.
  *Blast radius if wrong:* the robustness *envelope* host tests certified is mis-shaped — the
  suite proves survival under THIS schedule (Pid convergence through ×5 stalls is pinned); a
  wilder real distribution means re-running the suite under the measured schedule, which is a
  config swap in `JitterScheduleConfig`.

- [ ] **HA-35 — `bootSettleTime = 0.1 s` covers the worst sensor data-path latency.**
  *Claim:* 100 ms past first-ready is enough for a delayed heading stream to flush its
  boot-boundary garbage (2× the HA-30 guess, the worst modeled latency).
  *Source:* `include/shulib/localization/localizer.hpp:81–87`.
  *Confidence:* **invented** — derived from HA-24/HA-30, which are themselves invented.
  *Settle (R4):* set from the *measured* worst latency (HA-24/HA-30/HA-25) with margin; verify
  with instrumented real boots (the fused pose must not move while stationary through
  ready-transition — the A3 attack, run on hardware).
  *Blast radius if wrong:* too small → the §3.2 leak returns on real hardware (A3 measured
  3.65″ pre-fix; the settle window is what closed it); too large → wasted milliseconds at auton
  start. One constant, and the leak signature (pose jump at exactly ready+ε while stationary) is
  now known and greppable in telemetry.

- [ ] **HA-36 — `driftHorizon = 12″`: dead-reckon trust should decay to floor by about a foot of
  travel.**
  *Claim:* real odometry drift accumulates at a rate that makes ~12″ of uncorrected travel the
  right quality-decay scale.
  *Source:* `include/shulib/localization/localizer.hpp:75–78`.
  *Confidence:* **invented** — a policy constant expressing an unmeasured drift rate.
  *Settle (R4):* measured drift-vs-travel from repeated dead-reckon runs (push/figure-eight
  scripts against tape marks); set the horizon where error crosses the useful-fix threshold.
  *Blast radius if wrong:* quality *reporting* miscalibrated (gates too eager or too trusting);
  no pose math involved. One constant.

- [ ] **HA-37 — traction breaks above ≈ 80 in/s² commanded wheel acceleration.**
  *Source:* `include/shulib/sim/hostile/slip_hostility.hpp:61`. *Confidence:* **invented** —
  our wheels/foam combination has never existed.
  *Settle (R4):* commanded-accel ramps while comparing drive-encoder spin vs tracking-wheel
  truth; the divergence onset is the threshold.
  *Blast radius if wrong:* C-phase profile aggressiveness (max accel choices) and the realism of
  slip-scenario tests. Lower threshold → real launches slip more than modeled → tune profiles
  down (a constants change R5 owns anyway).

- [ ] **HA-38 — a slipping wheel still propels ≈ 70% of its spin.**
  *Source:* `slip_hostility.hpp:62`. *Confidence:* **invented**.
  *Settle (R4):* measured travel deficit during induced slip (the A3 exact-accounting test shape
  — truth 64″ vs 80″ encoder-implied — run on hardware).
  *Blast radius if wrong:* slip-window realism in scenarios; no estimator logic depends on the
  value (tracking odometry rides through drive slip *by architecture*, pinned at A3).

- [ ] **HA-39 — the field surface is traction-uniform: one (threshold, retain) pair suffices.**
  *Claim:* foam-tile traction does not vary enough across the field to need a per-region model.
  *Source:* `slip_hostility.hpp:28–31` (register note).
  *Confidence:* **invented** — the planning-time "field surface variation" seed, given claim form.
  *Settle (R4):* repeat HA-37's ramp at several field locations/orientations; compare onsets.
  *Blast radius if wrong:* location-dependent slip the model can't express without extension
  (slip windows already provide the escape hatch: declared per-region scenarios); affects skills
  route planning more than the library.

- [ ] **HA-40 — pack sag ≈ 0.02 V per commanded volt (≈1 V at four motors × 12 V).**
  *Source:* `include/shulib/sim/hostile/power_hostility.hpp:71`. *Confidence:* **invented**.
  *Settle (R4):* log battery voltage vs commanded load steps.
  *Blast radius if wrong:* brownout-margin planning and `Feedforward`'s battery compensation
  realism; constants. (The *shape* — sag follows load and recovers — is the pinned part.)

- [ ] **HA-41 — pack discharge ≈ 0.005 V/s under match load (≈0.3 V per 60 s run).**
  *Source:* `power_hostility.hpp:72`. *Confidence:* **invented**.
  *Settle (R4):* full-match logs, fresh vs tired packs.
  *Blast radius if wrong:* end-of-match brownout probability estimates; constants.

- [ ] **HA-42 — the brain cuts motor power at ≈ 10.5 V and recovers at ≈ 10.8 V.**
  *Source:* `power_hostility.hpp:73` (`cutoffVolts`) **and**
  `include/shulib/diag/health_monitor.hpp:96–99` (`brownoutVolts`/`brownoutRecoverVolts`; line numbers
  re-pointed 2026-09-26 at DIAG1, whose header note moved them — they already read 66–69 against 71–74) — two
  deliberately-aligned copies; they must move together when measured.
  *Confidence:* **invented** (community-folklore threshold; the hysteresis width is pure guess).
  *Settle (R3/R4):* bench brownout (HA-19's session): note the actual cutoff and recovery
  voltages under load.
  *Blast radius if wrong:* `HealthMonitor` brownout episodes fire early/late/chatter, and the
  F2 park guard's collapse-margin math is off. Two constants (cross-referenced in both headers).

- [ ] **HA-43 — motor thermal: heating ≈ 0.0023 °C/(V²·s), cooling ≈ 0.01 /s toward 25 °C
  ambient.**
  *Source:* `power_hostility.hpp:74–76`. *Confidence:* **invented** (chosen so full-drive
  reaches ~55 °C in ~90 s — a plausible story, not data).
  *Settle (R4):* `IMotor::temperature()` logs across sustained-load runs; fit both rates.
  *Blast radius if wrong:* when-does-droop-arrive predictions for skills route planning;
  constants.

- [ ] **HA-44 — V5 thermal throttling steps at 55/60/65 °C to 50/25/12.5% — and the onset on
  OUR motors is at the documented values.**
  *Source:* `power_hostility.hpp:77` (`throttleTempC`), model steps in the same header;
  `health_monitor.hpp:101` (`maxMotorTempC = 55` — deliberately equal to the first step; re-pointed
  at DIAG1 from a stale `:70–71`).
  *Confidence:* **mixed** — the step *shape* is measured elsewhere (VEX-documented current
  limiting); the onset temperatures on our units are unmeasured (invented until R4).
  *Settle (R4):* drive to droop; correlate `temperature()` with observed speed steps.
  *Blast radius if wrong:* `MOTOR_OVER_TEMP` fires at the wrong time relative to the actual
  droop (warning without droop, or droop without warning); constants in two aligned places.

---

- [ ] **HA-58 — 200 ticks (~2 s at 100 Hz) of flight recorder reaches back past the CAUSE of a
  fault.**
  *Claim:* when a fault fires, whatever caused it is visible within the preceding ~2 seconds of
  per-tick records, so a 200-tick ring is deep enough to diagnose from.
  *Source:* `include/shulib/diag/sd_sink.hpp` `kDefaultFlightRingTicks` (PROVISIONAL
  (A4: HA-58)); consumed by every caller that sizes `SdSinkBuffers`.
  *Confidence:* **invented** — it is diagnostics-plan D-6's own figure, chosen before any real
  failure had been recorded. Nobody has yet measured how far before a fault its cause sits.
  *Settle (R4):* once real runs exist, take the dumps that actually diagnosed something and
  measure how far back the useful evidence started. If the answer is "further", the ring grows
  (a template/argument change and more RAM); if "much less", the ring shrinks and the dump gets
  cheaper.
  *Blast radius if wrong (too small):* a dump that starts after the cause — the fault is
  recorded, its origin is not, and the run has to be reproduced to learn anything. *(Too
  large):* RAM spent and a slower dump, both harmless.

- [ ] **HA-59 — 64 KiB of RAM is spendable on the blackbox staging buffer.**
  *Claim:* a V5 program running the full stack can permanently give 64 KiB to a diagnostics
  buffer (plus ~88 KB for a 200-tick ring of records) without pressuring anything else.
  *Source:* `include/shulib/diag/sd_sink.hpp` `kRecommendedBufferBytes` (PROVISIONAL
  (A4: HA-59)); the storage is caller-owned, so this is a recommendation, not a hard size.
  *Confidence:* **invented** — no memory budget for the real program exists yet.
  *Settle (R4):* measure free heap/static space with the competition build loaded, then set the
  recommendation to something the real program can actually afford.
  *Blast radius if wrong:* the number is a caller-chosen span, so being wrong costs a
  one-line change — and being wrong SMALL is visible rather than silent (frames are dropped and
  counted, and the count is reported in the run summary and written into the file).

- [ ] **HA-60 — an SD flush of tens of kilobytes costs single-digit milliseconds.**
  *Claim:* writing ~64 KiB to `/usd/` on the V5 brain takes a few ms — affordable at a motion
  boundary or at auton end, and NOT affordable inside a 10 ms control tick.
  *Source:* `include/shulib/diag/sd_sink.hpp`, on `SdSink::flush()` (PROVISIONAL (A4: HA-60));
  it is the reason writes are caller-paced at all, and the reason the fault dump is the one
  exception that may write immediately.
  *Confidence:* **invented** — the V5's SD write path has never been timed by this team. The
  legacy code's experience with V5 SERIAL backpressure (~900-byte chunks with delays) is
  suggestive of the same class of problem, but it is a different device.
  *Settle (R4):* time `fwrite` + `fflush` of 4/16/64/256 KiB on a real brain, cold and warm,
  and put the numbers here.
  *Blast radius if wrong (much slower):* a flush at a motion boundary steals loop time; the
  LoopMonitor would report it as an overrun, so it is at least VISIBLE. The fix is a policy
  change — flush less often, or only at auton end — not a format change. R1 should re-check
  this the first time the adapter runs on hardware.
  *A second consumer, IN the tick (T4 Part 3, 2026-10-04; amended at its review, 2026-10-08):* the
  programs' SD copy of their log ("shulib Drive"'s `drive_log.txt`, the competition program's
  `comp_log.txt`; `diag/log_copy_sink.hpp` through `hal::BlockCharSink`) writes and flushes ONE
  LINE whenever a line is logged — so an event line in driver control or during an auton costs a
  small card write and a flush inside that tick, not at a boundary. Two things keep that to events
  rather than ticks, without this number: the copy never carries the per-tick records (decision A
  (a)), and a loop that stays slow now logs `LOOP_OVERRUN` once per EPISODE rather than once per tick
  (`diag/loop_monitor.hpp`; before the review a 20 ms-tick turn put 50 such lines, 50 flushes, on
  the card — each inside an already-late tick, the overrun's own report feeding it). Not bounded
  further: an INTERMITTENT overrun — a slow tick between healthy ones — is a new episode each time,
  so a loop slow every few ticks still writes a line every few ticks (as it did before the review). The cost of
  one small line's write and flush is the same unknown as this entry's; worksheet Station K reads
  it (K.7.2), and an overrun that the copy causes would itself be said once, not hidden.

### The E2 `GpsCorrector` tuning set (HA-61 … HA-67)

Seven constants, added when the first real corrector landed. They share one property worth
stating once instead of seven times: **E2 proved the corrector's LOGIC, and every one of these
numbers is a guess about a device nobody has plugged in.** Each is tested by an assertion about
*shape* — "a worse claim widens sigma", "the gate widens with dead-reckoned travel", "a spin
rejects the fix" — never by an assertion that the constant is right. Contained the same way as
the rest of Phase E's tuning: one config struct, one owner, no core impact.

They also share one settle: **HA-26 (the real on-strip noise) and HA-20 (the real dead-reckon
drift) decide whether folding GPS is worth doing at all.** E2's own measurements found the two
within the same order of magnitude in simulation, which is why the accuracy gain there is real
but modest (see `test/gps_corrector_accuracy_test.cpp`'s header). R4 settles both, and the
answer changes how these seven should be set.

- [ ] **HA-61 — the GPS's self-reported rms must be inflated ×2 before it is used as sigma.**
  *Claim:* the device's `get_error()` understates its true 1σ by roughly a factor of two, so
  trusting the claim raw makes the gate too tight and the pull too strong.
  *Source:* `include/shulib/localization/gps_corrector.hpp`, `GpsCorrectorConfig::rmsTrustFactor`
  (PROVISIONAL (A4: HA-61)). Follows from HA-29, which records that the claim and the truth are
  different numbers without saying by how much.
  *Confidence:* **invented** — the factor 2 is a placeholder for "more than one".
  *Settle (R4):* the same joint log HA-29 needs — `get_error()` against measured error, on
  strip, at several positions. The factor is the ratio of their standard deviations.
  *Blast radius if wrong (too small):* gate too tight, good fixes rejected, GPS quietly useless
  — visible as `RejectedNormalizedInnovation` filling the blackbox. (Too large): weak pull and
  a slack gate, so lies do more damage before the innovation bound catches them. Constant only.

- [ ] **HA-62 — 0.5 inch is a sane floor on the measurement sigma.**
  *Claim:* no GPS fix should ever be treated as better than half an inch, whatever the device
  claims.
  *Source:* `gps_corrector.hpp`, `GpsCorrectorConfig::minPositionStdDev` (PROVISIONAL (A4: HA-62)).
  *Confidence:* **invented**.
  *Settle (R4):* set to the best measured per-axis sigma observed on strip.
  *Blast radius if wrong:* the floor exists to stop a device reporting ~0 from producing an
  arbitrarily tight gate that rejects every fix including truthful ones — that failure mode is
  pinned by test regardless of the value. Constant only.

- [ ] **HA-63 — a fix claiming more than 6 inches of error is not worth folding.**
  *Claim:* a device asserting `hasFix()` while reporting a large self-error should be declined
  rather than folded weakly.
  *Source:* `gps_corrector.hpp`, `GpsCorrectorConfig::maxReportedRms` (PROVISIONAL (A4: HA-63)).
  *Confidence:* **invented**.
  *Settle (R4):* from the on-strip `get_error()` distribution — set above the healthy tail and
  below whatever the device reports when it is struggling.
  *Blast radius if wrong (too high):* a barely-usable fix is folded, contributing almost nothing
  while making the Localizer report quality class "Corrected" — a run that looks anchored and is
  not. That is the specific failure the ceiling exists for, and it is pinned by test. (Too low):
  usable fixes declined in poor conditions; visible as `RejectedSensorQuality`.

- [ ] **HA-64 — 3 rad/s (≈172°/s) is where a GPS fix stops being trustworthy.**
  *Claim:* above this yaw rate the fix's lever-arm reduction and its heading/position sampling
  skew make it worse than dead-reckoning for the tick.
  *Source:* `gps_corrector.hpp`, `GpsCorrectorConfig::maxYawRate` (PROVISIONAL (A4: HA-64)).
  *Confidence:* **invented** — chosen as "clearly a fast spin, not a normal arc"; it is not
  derived from any measurement of how the camera behaves under rotation.
  *Settle (R4):* spin in place on the strip at increasing rates and log reported vs. true
  position; the knee is the threshold. Re-check at E5 whenever the lever arm is re-measured
  (HA-10).
  *Blast radius if wrong (too high):* fixes folded during spins carry a heading-dependent bias —
  exactly the error HA-10 warns about, at its worst. (Too low): the corrector goes quiet during
  every turn, which is a loss of information, not a corruption. The asymmetry is why the default
  errs low.

- [ ] **HA-65 — 4 sigma is the right normalized-innovation gate width.**
  *Claim:* a residual beyond 4× the fix's own 1σ is more likely a lie than a truth.
  *Source:* `gps_corrector.hpp`, `GpsCorrectorConfig::gateSigma` (PROVISIONAL (A4: HA-65)).
  *Confidence:* **invented** — 4σ is the conventional engineering choice, not a measured one,
  and it is only as meaningful as the sigma it multiplies (HA-61, HA-66, HA-67).
  *Settle (R4):* from the measured residual distribution on a run with known truth.
  *Blast radius if wrong:* the classic gate trade — too tight rejects truthful corrections
  (stubbornness), too loose accepts lies (corruption). Both are VISIBLE in the blackbox as
  `RejectedNormalizedInnovation` counts, which is why E2 put the residual and the sigma on the
  wire. **Note the ceiling:** `ComplementaryFusion::innovationGate` (12 inches, fixed) applies
  after this gate, so widening `gateSigma` past ~5 with these sigmas changes nothing.

- [ ] **HA-66 — the estimate is ~1 inch uncertain immediately after a fix is folded.**
  *Claim:* the floor of the dead-reckoning sigma, which sets how hard a fresh fix pulls.
  *Source:* `gps_corrector.hpp`, `GpsCorrectorConfig::postFixStdDev` (PROVISIONAL (A4: HA-66)).
  *Confidence:* **invented**. It is the closest thing the complementary tier has to a prior,
  and the honest description is "a gain knob wearing the clothes of a covariance".
  *Settle (R4):* properly, this is E4's job — an EKF estimates it instead of asserting it. Until
  then, set from measured post-correction error.
  *Blast radius if wrong:* it and HA-61 together set the confidence
  `σ_dr²/(σ_dr² + σ_meas²)`, i.e. how fast the estimate chases the GPS and therefore how much
  sensor noise it inherits. Must stay > 0: a zero would make confidence zero, the Localizer
  screens a zero-confidence proposal out, and the corrector would look absent rather than weak.
  Guarded by precondition.

- [ ] **HA-67 — dead-reckoning uncertainty grows ≈0.02 inch of sigma per inch travelled.**
  *Claim:* 2% of distance travelled, as a 1σ position uncertainty, since this source's last fix.
  *Source:* `gps_corrector.hpp`, `GpsCorrectorConfig::driftStdDevPerInch` (PROVISIONAL (A4: HA-67)).
  *Confidence:* **invented**, and related to but not the same as HA-36 (the Localizer's
  `driftHorizon`, which decays a quality SCALAR rather than widening a gate).
  *Settle (R4):* the same measurement as HA-36 — drive a known path with the GPS covered and
  measure how error grows with distance.
  *Blast radius if wrong (too small):* GATE LOCKOUT — after a long blind stretch, a truthful fix
  looks outrageous and is rejected, and since nothing else can repair the estimate, so is every
  fix after it. The GPS dies exactly when it is worth the most. This is the failure the term
  exists to prevent and it is pinned by a dedicated test. (Too large): the gate stays wide open
  after one long dead-reckon and a lie is accepted because the corrector still believes it is
  lost — pinned by the companion test that the widening RESETS on an accepted fix.

### The E3 AprilTag set (HA-68 … HA-82)

Fifteen entries, added when absolute yaw correction landed. They divide into **three that are
about the physical world and are the dangerous ones** (HA-68, HA-69, HA-70) and **twelve tuning
constants** that behave exactly like E2's set — proved by shape, never by value, contained in one
config struct each.

The three structural ones deserve their own warning, because unlike a tuning constant **they do
not degrade gracefully**. A wrong tag map, a reversed corner winding or a pitched camera each
produce a fix that is *confidently* wrong with a small residual and a high confidence — i.e. it
looks exactly like a healthy fix, and no gate, no filter and no amount of averaging can reveal it.
E2's tuning constants make the corrector work better or worse; these three make it lie.

- [ ] **HA-68 — the field's AprilTag layout is knowable, and a team's map is accurate to ≈0.5 inch.**
  *Claim:* the tags' field poses can be obtained (from a published specification or by measuring
  the actual competition field) to within about half an inch and half a degree, and the team will
  in fact do so.
  *Source:* `include/shulib/localization/tag_map.hpp` (A4 register HA-68). **shulib deliberately
  ships NO built-in map** — the header explains why at length — so this is an assumption about the
  *user's input*, not about a number in the tree, which is unusual for this register and is the
  point: there is no citable table of AprilTag field poses in this project's sources today.
  *Confidence:* **invented** — nobody here has measured or cited anything.
  *Settle (R3):* obtain the published field layout if one exists; otherwise measure every tag's
  position and facing on the competition field with a tape and a square, and record the method in
  each `TagPlacement::source`.
  *Blast radius if wrong:* **the worst in this register.** A map entry two inches off yields a
  corrector that is confidently two inches wrong every time it sees that tag, with a small
  residual and a high confidence. Sensor noise averages out; **a wrong map does not**. It will
  also fight the GPS corrector, and the complementary tier has no way to tell which is lying.
  Partly contained: `TagMap::add()` refuses an entry with no provenance, and `anyInvented()` lets
  telemetry say the estimate is anchored to guessed geometry.

- [ ] **HA-69 — the tag detector reports its four corners in a consistent, knowable winding.**
  *Claim:* whatever order the V5 AI Vision sensor (or a Pi detector) reports corners in, it is
  stable, and R2 can map it onto the order `hal/vision_conversion.hpp` documents.
  *Source:* `include/shulib/hal/vision_conversion.hpp`, the corner-order contract (A4: HA-69).
  *Confidence:* **reasoned** — every AprilTag implementation documents an order; ours is not
  verified against a physical detector.
  *Settle (R2/R3):* point the real detector at a real tag at a known pose and check the recovered
  pose, ONCE. That is the only check that exists (see the blast radius).
  *Blast radius if wrong:* **measured at E3 and worse than expected.** A CYCLIC ROTATION of the
  order is harmless — the planar reduction discards that degree of freedom. A **REVERSAL** is
  catastrophic and completely silent: it mirrors the tag plane, the recovered heading is 180°
  wrong, and **the reprojection error stays at machine zero**, because a mirrored pose reprojects
  onto the very same four pixels. No self-check anywhere in the pipeline can detect it; only a
  physical tag can. Pinned by three subcases in `test/vision_conversion_test.cpp`.

- [ ] **HA-70 — the tag camera is mounted level, to within about a degree.**
  *Claim:* the camera's optical axis is horizontal (no pitch, no roll), so a tag's height above
  the camera can be discarded without affecting its horizontal position.
  *Source:* `hal/vision_conversion.hpp`, the planar-reduction note (A4: HA-70). `CameraMount`
  carries position and yaw only, which encodes the assumption in the type.
  *Confidence:* **invented** — robot two has carried an AI Vision camera on port 3 since
  2026-09-24, but nobody has measured how it is mounted, and no library adapter reads it yet (R2)
  (a library adapter for the camera's objects and colours exists since VIS1-A1, host-tested only; no
  program reads robot two's camera until VIS1-B declares it; its TAGS are read by nothing).
  *Settle (R3):* measure the mounted camera's pitch and roll; if either is significant, the
  reduction needs the full 3-D transform and `CameraMount` grows two fields.
  *Blast radius if wrong:* a pitched camera turns a tag's HEIGHT into a RANGE error — a tag
  mounted 15 inches above a camera pitched 5° reads several inches nearer or further than it is —
  and nothing downstream can see that it happened. Contained to one function and one struct.

- [ ] **HA-71 — the tag pipeline delivers a fix describing a moment ≈80 ms earlier.**
  *Claim:* exposure + detection + PnP + transport totals about 80 ms, longer than the GPS's ~50 ms
  (HA-30) because a tag pipeline does more work per frame.
  *Source:* `apriltag_corrector.hpp`, `AprilTagCorrectorConfig::latency` (PROVISIONAL (A4: HA-71)).
  *Confidence:* **invented**.
  *Settle (R4):* the same measurement HA-30 needs — flash a known event and timestamp when the
  reduced detection becomes readable.
  *Blast radius if wrong:* an uncompensated 80 ms drags position backwards along the direction of
  travel AND, new at E3, drags HEADING backwards along the direction of rotation — at 180°/s that
  is 14°, fourteen times the entire heading budget. Both compensations are pinned by their own
  exact-case tests; only the magnitude is at risk.

- [ ] **HA-72 — a tag frame older than 0.25 s means the vision task has stalled.**
  *Claim:* a healthy vision task at ~20 Hz never leaves a gap this long, so a gap means failure.
  *Source:* `apriltag_corrector.hpp`, `maxObservationAge` (PROVISIONAL (A4: HA-72)).
  *Confidence:* **invented**.
  *Settle (R4):* log inter-frame intervals on a real coprocessor under load; set to a few times
  the observed 99th percentile.
  *Blast radius if wrong (too small):* healthy frames declined as `RejectedObservationAge` —
  visible, and the estimator falls back to dead-reckoning rather than doing anything wrong.
  (Too large): a dead vision task keeps folding the last frame it saw for a quarter of a second
  longer than it should. Constant only.

- [ ] **HA-73 — a tag fix is trustworthy only between 6 and 72 inches of range.**
  *Claim:* nearer than 6 inches the tag overfills the frame and is likely clipped; further than
  72 inches planar PnP's near-ambiguity makes the ORIENTATION untrustworthy well before the
  position is.
  *Source:* `apriltag_corrector.hpp`, `minRange` / `maxRange` (PROVISIONAL (A4: HA-73)).
  *Confidence:* **invented** — the *shape* of the claim (heading degrades with range faster than
  position) is real geometry; both numbers are made up.
  *Settle (R4):* park the robot at a known pose and sweep range; plot recovered position error and
  recovered heading error against distance. The band is where heading error crosses the budget.
  *Blast radius if wrong:* this band is E3's substitute for a separate heading noise model, so a
  band that is too wide folds heading fixes that are worse than the IMU — the one way this chunk
  could make heading accuracy WORSE. Too narrow and the corrector is simply idle more often.
  E4's EKF replaces the band with a real R_heading, which is the principled fix.

- [ ] **HA-74 — a tag detection below 0.35 confidence is not worth folding.**
  *Claim:* the detector's own score below this indicates a detection too poor to trust.
  *Source:* `apriltag_corrector.hpp`, `minConfidence` (PROVISIONAL (A4: HA-74)).
  *Confidence:* **invented** — and it assumes the V5 sensor's score is even comparable across
  detections, which is itself unverified.
  *Settle (R4):* log the score against measured pose error across a range of lighting and angles.
  *Blast radius if wrong:* the same shape as E2's D7 — without a floor a 0.05-confidence detection
  is still folded with a microscopic pull, and the Localizer reports quality class **Corrected** on
  a run with no usable anchor. That failure mode is pinned by test regardless of the value.

- [ ] **HA-75 — 2 rad/s (≈115°/s) is where motion blur kills a tag fix.**
  *Claim:* above this yaw rate the tag smears across the frame and a rolling shutter skews it into
  a different quadrilateral, which PnP solves happily into a confidently wrong pose.
  *Source:* `apriltag_corrector.hpp`, `maxYawRate` (PROVISIONAL (A4: HA-75)). Tighter than the
  GPS's HA-64 (3 rad/s) because a camera integrating over an exposure is more motion-sensitive
  than a strip reader.
  *Confidence:* **invented**.
  *Settle (R4):* spin at increasing rates in front of a fixed tag and find where the recovered
  pose degrades.
  *Blast radius if wrong (too high):* blurred fixes folded — and unlike a noisy fix, a skewed one
  is biased, so it does not average out. (Too low): the corrector goes quiet during every turn,
  which is when the estimate needs it most.

- [ ] **HA-76 — tag position 1σ is about 1.0 inch, growing 0.02 inch per inch of range.**
  *Claim:* the linear-in-range noise model, at confidence 1.
  *Source:* `apriltag_corrector.hpp`, `baseStdDev` / `stdDevPerInch` (PROVISIONAL (A4: HA-76)).
  *Confidence:* **invented** — the linear form is a modelling choice as much as the coefficients.
  *Settle (R4):* the range sweep from HA-73 gives both.
  *Blast radius if wrong:* sets both the gate width and the pull strength. Tested by shape only —
  "farther is worse", "less certain is worse" — never by value.

- [ ] **HA-77 — 4 sigma is the right normalized-innovation gate width for tags.**
  *Source:* `apriltag_corrector.hpp`, `gateSigma` (PROVISIONAL (A4: HA-77)). Same value and same
  reasoning as E2's HA-65; kept separate because the sigma it multiplies is a different model.
  *Confidence:* **invented**.
  *Settle (R4):* choose from the measured innovation distribution once HA-76 is real.
  *Blast radius if wrong:* as HA-65 — too tight rejects truthful fixes, too slack admits lies.

- [ ] **HA-78 — the estimate is ≈1 inch uncertain immediately after a tag fix is folded.**
  *Source:* `apriltag_corrector.hpp`, `postFixStdDev` (PROVISIONAL (A4: HA-78)).
  *Confidence:* **invented**. Like HA-66 this is **a gain knob wearing the clothes of a
  covariance**; E4's EKF estimates it instead of asserting it.
  *Settle (R4/E4):* superseded by the filter's own covariance.
  *Blast radius if wrong:* sets the floor of σ_dr and therefore the confidence of every fix.

- [ ] **HA-79 — dead-reckon sigma grows ≈0.02 inch per inch since this source's last fix.**
  *Source:* `apriltag_corrector.hpp`, `driftStdDevPerInch` (PROVISIONAL (A4: HA-79)). The tag
  corrector's own copy of HA-67, per-source by design: each corrector tracks how long *it* has
  been blind.
  *Confidence:* **invented**.
  *Settle (R4):* as HA-67.
  *Blast radius if wrong:* **gate lockout**, exactly as HA-67 describes, and pinned by the same
  pair of tests (a fix a zero-growth build rejects is accepted after a long dead-reckon; the
  widening RESETS on an accepted fix).

- [ ] **HA-80 — a heading innovation above 15 degrees is a fault, not drift.**
  *Claim:* a raw V5 IMU drifts ≈1°/min (HA-20), so over a 60-second match the expected heading
  error is about a degree. An innovation fifteen times that is far more likely to be a mirrored
  corner winding (HA-69), a wrong map entry (HA-68) or a misidentified tag id than real drift.
  *Source:* `complementary_fusion.hpp`, `ComplementaryFusionConfig::headingGate`
  (PROVISIONAL (A4: HA-80)).
  *Confidence:* **invented**, and it inherits HA-20's uncertainty: if real IMU drift is much worse
  than 1°/min, this gate becomes the thing that prevents recovery.
  *Settle (R4/E4):* set from the measured IMU drift distribution (HA-20) once that exists; E4's
  EKF replaces it with a Mahalanobis test.
  *Blast radius if wrong (too tight):* a genuinely drifted IMU can never be corrected — the
  correction locks out exactly when it is needed, the heading analogue of HA-67's failure.
  (Too slack): a mirrored tag rotates the robot's entire idea of the field, and every
  field-relative command afterwards inherits it.

- [ ] **HA-81 — 0.15 of the heading innovation is pulled per tick at confidence 1.**
  *Source:* `complementary_fusion.hpp`, `maxHeadingGain` (PROVISIONAL (A4: HA-81)). Same value as
  the position gain, which is a convenience, not a derivation.
  *Confidence:* **invented**.
  *Settle (R4/E4):* superseded by the EKF's Kalman gain.
  *Blast radius if wrong:* sets the settling time. Measured in simulation: a 4° error closes to
  0.5° in about 3 seconds. Too high and the correction fights the turn controller; too low and a
  60-second match ends before the bias is learned.

- [ ] **HA-82 — 10°/s is a safe upper bound on how fast the estimator's heading bias may change.**
  *Claim:* the robot's idea of which way it is pointing may be revised at up to 10°/s without
  disrupting a motion in progress.
  *Source:* `complementary_fusion.hpp`, `maxHeadingNudgeRate` (PROVISIONAL (A4: HA-82)).
  *Confidence:* **invented**.
  *Settle (R4/E4):* drive a heading-holding motion while injecting a bias correction and find the
  rate at which the controller visibly reacts.
  *Blast radius if wrong (too high):* a large correction arrives fast enough to fight an active
  turn — this is the number that stands between "nudge" and "snap" for yaw, and a yaw snap
  poisons every field-relative command after it. (Too low): a real drift takes longer than a match
  to correct. **This bound is what makes never-snap structural for heading**, and it is audited
  on every tick from the blackbox (`correctionDTheta`), so a violation is visible after the fact.

- [ ] **HA-83 — the EKF's position process noise: 1σ grows by 2% of the distance travelled since
  the last accepted fix, plus a 0.5 in/s floor while standing still.**
  *Claim:* dead-reckoning error is dominated by a SYSTEMATIC scale and alignment error, so σ grows
  LINEARLY with distance rather than as its square root, and something small but non-zero
  accumulates even at rest.
  *Source:* `ekf_fusion.hpp`, `posNoisePerInch` / `posNoiseRate` (PROVISIONAL (A4: HA-83)).
  *Confidence:* **invented**. The 2% figure is E2's `driftStdDevPerInch` (HA-67) reused so the two
  layers model the same physics with the same number; the floor is a numerical-health term.
  *Settle (R4):* drive a known distance with no correction, repeatedly, and plot the error
  distribution against distance — the SHAPE of that plot (linear vs √d) settles the model, and its
  slope settles the constant.
  *Blast radius if wrong (too small):* **gate lockout.** Measured during E4 with a random-walk
  form in place: after 360 inches of dead-reckoning the filter believed it was within half an inch
  and rejected a truthful fix 20 inches away. (Too large): the gate never closes and a lie walks
  in.

- [ ] **HA-84 — the EKF's heading process noise: 1σ grows by 1% of the rotation actually
  performed, plus HA-20's ≈1°/min of raw IMU drift.**
  *Source:* `ekf_fusion.hpp`, `headingNoisePerRad` / `headingDriftRate` (PROVISIONAL (A4: HA-84)).
  *Confidence:* **invented**, and it INHERITS HA-20 entirely — the drift term is HA-20 restated as
  a variance, so if HA-20 is wrong this is wrong by the same factor.
  *Settle (R4):* the same boot-drift experiment HA-20 needs; the scale-factor term needs a
  known-rotation test (spin exactly ten turns, compare).
  *Blast radius if wrong (too small):* a genuinely drifted heading is never corrected, because the
  filter is certain about a heading that is wrong. (Too large): a mirrored tag moves the bias fast.

- [ ] **HA-85 — the drivetrain can change its body velocity by about 200 in/s².**
  *Claim:* this is roughly a hard V5 drive launch, and it is the EKF's process noise on the
  velocity states — i.e. how far the constant-velocity model is allowed to be wrong in one tick.
  *Source:* `ekf_fusion.hpp`, `velNoise` (PROVISIONAL (A4: HA-85)).
  *Confidence:* **invented**; co-depends on HA-45's feedforward constants.
  *Settle (R5):* falls straight out of the sysid ramp — the measured peak acceleration IS this
  number.
  *Settle path note (R5a Part A's review, 2026-10-04):* that ramp became shulib Calibrate's GAINS step
  (`calibration/drive_characterizer.hpp`; built as library, not yet run on a robot). Its two STEP
  segments start from rest at a held 6 V, so their first ticks carry the drive's hardest acceleration
  in the run, and R5a Part C writes every tick's speed to the card; the fitted gains say the same
  thing, (u − kS)/kA at the step's start. A 6 V step is not a full-power launch, so the measured peak
  is a lower bound for this number, not the number.
  *Blast radius if wrong (too small):* the filter lags the wheels through every acceleration, and
  the position estimate lags with it. (Too large): encoder noise passes straight through, which is
  mostly harmless.

- [ ] **HA-86 — one tick's odometry displacement is 1σ ≈ 0.01 inch plus 2% of that tick's travel.**
  *Claim:* the constant term is encoder quantization and tracking-wheel jitter; the proportional
  term is slip.
  *Source:* `ekf_fusion.hpp`, `odomStdDev` / `odomStdDevPerInch` (PROVISIONAL (A4: HA-86)).
  *Confidence:* **invented**. The quantization half is checkable on paper from HA-16 and HA-13 and
  is the more trustworthy of the two.
  *Settle (R4):* push the robot a measured distance by hand with the motors off and compare
  odometry to a tape measure, repeatedly.
  *Blast radius if wrong:* sets how much the filter smooths the wheels. Too small and it follows
  encoder noise; too large and it lags. Measured at the default: the filter's dead-reckoning
  differs from raw odometry by under 0.7 inch over two minutes of stop-start driving, and does not
  accumulate.

- [ ] **HA-87 — 3σ is the right Mahalanobis gate width for the EKF tier.**
  *Claim:* on a 2-degree-of-freedom position innovation, ν > 3 means a fix is an outlier rather
  than noise — about a 1.1% false-reject rate IF the noise model is right, which is the caveat
  that matters.
  *Source:* `ekf_fusion.hpp`, `gateSigma` (PROVISIONAL (A4: HA-87)).
  *Confidence:* **invented**, and note it is a DIFFERENT number from E2's `gateSigma` = 4 (HA-65)
  and E3's = 4 (HA-77) on purpose: those normalise by an assumed constant and must be slacker to
  survive being wrong; this one normalises by an estimated covariance and can afford to be tighter.
  *Settle (R4):* plot the measured innovation distribution once HA-83…HA-86 are real, and choose
  the width from it rather than from a table.
  *Blast radius if wrong (too tight):* honest fixes are refused and the filter re-inits more often
  than it should. (Too slack): the outlier protection stops working, which is the whole reason the
  gate exists.

- [ ] **HA-88 — an absolute heading measurement is 1σ ≈ 2°, flat.**
  *Claim:* a tag-derived heading is about two degrees uncertain, and using one number for every
  such measurement is better than inventing a per-proposal relationship.
  *Source:* `ekf_fusion.hpp`, `headingStdDev` (PROVISIONAL (A4: HA-88)).
  *Confidence:* **invented**, and it is the WEAKEST entry in this group: `CorrectionProposal`
  carries no heading σ, so this one constant stands in for every heading-providing source at every
  range. E3 handled the same gap with a trusted-range BAND (HA-73) instead.
  *Settle (R4):* the range sweep HA-73 already needs — park at a known pose, sweep range, and plot
  recovered heading error separately from position error. That sweep produces σ(range), and a
  corrector that can state it makes this constant unnecessary.
  *Blast radius if wrong (too small):* heading fixes are trusted more than they deserve, and a bad
  tag moves the bias faster. (Too large): the heading correction converges too slowly to matter
  inside a match.

- [ ] **HA-89 — when the EKF knows nothing, it is 24 inches, 30° and 24 in/s uncertain.**
  *Claim:* "somewhere within a tile, facing roughly the right way" is the right ignorance to start
  from, and to return to after a discontinuity or a re-init.
  *Source:* `ekf_fusion.hpp`, `initialPosStdDev` / `initialHeadingStdDev` / `initialVelStdDev`
  (PROVISIONAL (A4: HA-89)).
  *Confidence:* **invented**; the 24 inches is one VEX tile, which is a convenient scale and not a
  measurement.
  *Settle (R4):* it is a prior, so it is not measurable in the way the others are — what R4 can
  settle is whether it is large enough that a first fix is always accepted, and small enough that
  the first few ticks after a re-init are not dominated by it.
  *Blast radius if wrong (too small):* the first fix after a re-init is rejected and the estimator
  cannot recover at all. (Too large): the tick after a re-init accepts almost anything — bounded
  in practice by the per-tick nudge budget, which is why this errs large.

- [ ] **HA-90 — 50 consecutive gate rejections with a mean innovation above 6 inches means the
  estimator is lost.**
  *Claim:* at the ~20 Hz cadence a real corrector achieves, that is about 2.5 seconds of a sensor
  insisting the estimate is wrong — long enough that noise cannot produce it and short enough to
  recover inside a match.
  *Source:* `ekf_fusion.hpp`, `reinitRejectCount` / `reinitInnovation` (PROVISIONAL (A4: HA-90)).
  *Confidence:* **invented**.
  *Settle (R4):* count how often a healthy run produces consecutive rejections at all, and set the
  bar above the observed maximum.
  *Blast radius if wrong (too low):* the filter throws away good confidence on ordinary noise —
  measured as zero occurrences across 8 seeds of a 60-second hostile run at this setting.
  (Too high): a shoved robot takes longer to recover, bounded below by the 12 in/s nudge rate
  anyway.

- [ ] **HA-91 — 5 seconds is long enough between re-init declarations to tell recovery from a
  storm.**
  *Source:* `ekf_fusion.hpp`, `reinitCooldown` (PROVISIONAL (A4: HA-91)).
  *Confidence:* **invented**. It is bounded below by the time a rate-limited recovery actually
  takes: 30 inches at 12 in/s is 2.5 seconds, so a cooldown shorter than that could re-declare
  while the previous recovery is still in progress.
  *Settle (R4):* measure the real recovery time once the nudge rate is settled, and set this to a
  comfortable multiple of it.
  *Blast radius if wrong (too short):* a re-init storm, which in the record is indistinguishable
  from a filter that is working. (Too long): a second genuine displacement inside the window is
  not recovered from until the window expires.

- [ ] **HA-93 — a jammed 11 W mechanism motor reads ≈ 2.5 A at a full 12 V command (scaling
  linearly with commanded voltage), with ≈ 0.05 rad/s residual reported creep.**
  *Claim:* the F1 hostile jam model's device signature is the right order of magnitude for a real
  V5 motor stalled by a wedged ring or a hard stop — stall current ∝ V/R, a spec-ballpark 2.5 A
  at full command, and a not-exactly-zero encoder reading from belt chatter and backlash (included
  deliberately so a detector keyed on EXACT zero is caught by the fake).
  *Source:* `include/shulib/sim/hostile/mechanism_hostility.hpp` (`JammedMotorConfig` defaults,
  PROVISIONAL (A4: HA-93)).
  *Confidence:* **invented** — V5 11 W stall current is commonly cited near this figure; no
  measurement of ours, and 5.5 W motors differ.
  *Settle (R4):* jam a real intake against a block at several commanded voltages; log `current()`
  and `velocity()` through the F4 seam; set per-mechanism `StallConfig` thresholds from the
  measured separation between spin-up and jam (the library ships NO default thresholds for
  exactly this reason).
  *Blast radius if wrong:* hostile-suite realism only — the operation layer's thresholds are
  required per-mechanism parameters, so no library behavior rests on these numbers; a wrong
  magnitude here mis-calibrates the *worst day* the suite rehearses, not the robot.
  *Since MECH-A3 (2026-10-15):* the host test plant's mechanism current has this shape (HA-183), and
  since MECH-A3's review (2026-10-22) chapter 18's advice for a `homeByStall` signature leans on its
  linear scaling: a gentle homing push stalls at about |V|/12 of the 2.5 A limit (0.62 A at 3 V), so the
  homing signature's amps belong below that (0.45 A to start) — stated as the host test plant's number,
  to be checked on the bench, and a stall-home that never trips logs what its push drew when stopped. No library
  behaviour and no build refusal rests on it: a refusal of a homing signature above |V|/12 × 2.5 A was
  proposed and NOT built — if the scaling is wrong (a real motor drawing more at low volts) it would
  refuse a signature that works.

---

## Group R5 — gains and actuation constants

- [ ] **HA-92 — `BrakeMode::Hold` at 0 V holds a LOADED cascade lift against back-drive.**
  *Claim:* the V5 firmware's active position hold, commanded through the F1 mechanism seam's
  declared safe state, keeps a lift carrying game pieces at its height when its operation exits
  or is cancelled — i.e. "declare Hold and the stack does not come down" is physically true, not
  just commanded. The whole T4 design (per-mechanism declared safe states) makes the *declaration*
  reach the motor provably; whether the *motor* then wins against gravity plus a loaded cascade's
  back-drive torque is this claim.
  *Source:* `include/shulib/hal/mechanism.hpp` (banner + `MotorMechanism::applySafeState`,
  PROVISIONAL (A4: HA-92)).
  *Confidence:* **invented** — no lift exists; V5 Hold is documented as an active position hold,
  its holding torque under a real cascade's load and friction is not.
  *Settle (R3/R5):* build-team lift on a stand: raise under load, cancel the operation, watch the
  stack. Measure sag over 30 s at several loads; if Hold alone is insufficient, F3's `liftToLevel`
  needs its sag-comp PID hold (already in its spec) and the declared safe state stays Hold as the
  best-available floor.
  *Amended DEFECTS2 (2026-09-26) — the premise was false at the wire:* "the declaration reach[es]
  the motor provably" was true only down to `IMotor::setBrakeMode()` and the device's mode
  REGISTER. The stop itself went out as `move_voltage(0)`, which PROS documents as ignoring the
  mode (HA-142), so on a V5 a declared Hold went LIMP — the question this entry asks (does Hold win
  against the load?) could not even have been posed on hardware before DEFECTS2. Since then
  `ProsMotor` sends that zero as `brake()`, pinned at the bottom of `MotorMechanism` and a
  mechanism operation over real `ProsMotor`s (`test/brake_stop_stack_test.cpp` s1, mutation-proven).
  Nothing about the holding TORQUE changed; that is still this entry, and still invented. The first
  loaded hold will be T3's claw pivots (ports 6 and 7), and two neighbours ride along: HA-144
  (whether a repeated stop re-captures the held position) and HA-145 (a hold across a disable).
  *Blast radius if wrong:* a cancelled/parked lift descends at the buzzer — scored stack lost and
  a possible entanglement; F2's park guard inherits the same exposure. The failure is visible the
  first time hardware exists, which is why this is registered rather than assumed silently.

- [ ] **HA-45 — the plant's wheel feedforward placeholders: kS = 1.0 V, kV = 12/70 V·s/in
  (≈ 70 in/s free speed at 12 V), kA = 0.**
  *Claim:* the real drive's sysid constants are within order-of-magnitude of these.
  *Source:* `include/shulib/sim/drive_plant.hpp:142–148` (declared PLACEHOLDERS since A2; re-pointed
  at R5a Part A, 2026-09-29 — the lines had drifted).
  *Confidence:* **invented** — right order of magnitude for a V5 drive by construction, nothing
  more. (Free speed co-depends on HA-14/HA-15: 200 RPM green × 3.25″ ≈ 34 in/s at the wheel for
  1:1 — the 70 in/s figure is deliberately generous; R5 measures reality.)
  *Settle (R5):* `tools/sysid` least-squares over one on-robot ramp routine → committed
  constants; R6 feeds them back into this config and re-runs the whole suite.
  *Settle path amended R5a Part A (2026-09-29; the team lead's ruling of 2026-09-26):* the measurement
  moved onto the robot. shulib Calibrate's GAINS step (`calibration/drive_characterizer.hpp`, fitted on
  the brain by `calibration/feedforward_fit.hpp`) measures kS/kV/kA for each side, and R5a's later parts
  keep them on the SD card beside the drivetrain record — never committed as constants. Built as
  library and proven on the host test plant only; no program runs it yet, so every drive still runs on
  these placeholders. A measured kA is RECORDED, NOT APPLIED: the command pipeline calls the cruise form
  `Feedforward::calculate(v)`. The host test plant can now give each wheel its own gains
  (`DrivePlantConfig::wheelFfOverride`, empty by default — a default plant is unchanged).
  *Blast radius if wrong:* every sim-tuned gain and every open-loop distance in sim is
  quantitatively off — **which is already the declared position**: the plant proves logic, not
  constants; C-phase gains are provisional until R5 *by explicit rule* (the Phase C rule: the
  sim proves control logic; real gain values are established on hardware). No conclusion currently drawn from the suite depends on these values being right.

- [ ] **HA-46 — a fresh pack reads ≈ 12.6 V.**
  *Source:* `include/shulib/sim/drive_plant.hpp:154` (`batteryVoltage` default; re-pointed at R5a
  Part A, 2026-09-29).
  *Confidence:* measured elsewhere — documented Li-ion pack behavior.
  *Settle (R4):* trivially, with the first battery log.
  *Blast radius if wrong:* negligible — a sim default; the F4 `IBattery` path reads reality.

- [ ] **HA-47 — `move_voltage` is true voltage control: a 6 V cruise on a sagged 11.6 V pack is
  untouched (sag only bites at the ceiling).**
  *Claim:* V5 firmware applies commanded voltage without a hidden internal compensation loop, so
  partial-throttle behavior is pack-independent until demand exceeds the sagged ceiling (A3's
  §3.7 modeling insight, promoted to a testable claim).
  *Source:* `power_hostility.hpp` shape 2 (pack ceiling); A3's completion record §3.7, in the
  development log.
  *Confidence:* reasoned — matches PROS documentation of `move_voltage` semantics; PWM-level
  firmware behavior unverified.
  *Settle (R5):* measure velocity at fixed partial commands across pack states during sysid.
  *Settle path note (R5a Part A's review, 2026-10-04):* shulib Calibrate's GAINS step (built as
  library; not yet run on a robot) holds a fixed 6 V command through each step segment and records
  every segment's battery minimum and mean — run at two pack states, the step segments' speeds against
  their battery columns are this measurement.
  *Blast radius if wrong:* gains tuned at partial throttle would be pack-dependent (R5 would
  discover this in its own data); the plant's power model semantics get adjusted at R6.

- [ ] **HA-50 — the C1 motion defaults: translation kP = 3.0 (1/s), heading kP = 4.0 (1/s),
  kI = kD = 0; maxLinearSpeed = 60 in/s, maxAngularSpeed = 6 rad/s, maxWheelSpeed = 60 in/s.**
  *Claim:* gains of this magnitude converge without oscillation on the real drive, and the speed
  budget keeps `Feedforward(maxWheelSpeed)` inside the 12 V rail with margin.
  *Source:* `include/shulib/motion/motion_config.hpp` (every field labeled PROVISIONAL (A4:
  HA-50)); the values converge on the A2 plant across the C1 sweep/routine suites — which by
  Phase C's own rule proves logic, not constants.
  *Confidence:* **invented** — tuned against placeholder plant dynamics (HA-45 co-dependency:
  wrong kV shifts what "60 in/s" costs in volts).
  *Settle (R5):* re-tune against measured kS/kV/kA; commit real values; R6 re-runs the suite.
  *Blast radius if wrong:* convergence speed and overshoot on hardware — contained by design:
  every C1 exit is watchdog-bounded and settle-verified, so a bad gain degrades to slow/TimedOut,
  never to divergence-with-a-green-light. No frozen contract encodes these numbers.

- [ ] **HA-51 — the C1 settle tolerances: translation 0.5 in / 1.0 in/s / 0.1 s; heading
  0.02 rad / 0.30 rad/s / 0.1 s; brake 1.2 in/s / 100 in/s² / 0.1 s (on a 5-tick averaged
  twist); default watchdog 5 s.**
  *Claim:* these thresholds sit ABOVE the real sensors' noise floors (else motions never settle
  on hardware) and BELOW competition-useful accuracy (else settling is meaningless). The
  heading-rate floor (0.30 rad/s) clears the HA-21-noise-differentiation floor (~0.12 rad/s at
  100 Hz) by ~2.5×; the brake threshold clears the measured M2 fused-twist noise floor at a
  physical dead stop under composed hostility (0.5–1.5 in/s raw, ~0.3–0.9 averaged — measured
  at C1, `drive_brake.hpp` header).
  *Source:* `include/shulib/motion/motion_config.hpp` (PROVISIONAL (A4: HA-51));
  `drive_brake.hpp` (the averaging + floor note).
  *Confidence:* **invented** — the floors they must clear are themselves provisional (HA-21/22).
  *Settle (R4 noise floors, R5 tolerances):* measure real estimator twist noise at a dead stop
  and real settle behavior; tighten or loosen with data.
  *Amended R5a Part A (2026-09-29):* the REST step's `settleFloorReport()`
  (`calibration/rest_capture.hpp`) reports how far the heading-rate floor clears the MEASURED per-tick
  heading step over dt, and the brake floor the larger side's encoder-speed σ at rest, each as a margin
  with a warning under 2× — it changes nothing. The "~0.12 rad/s" above is √2·σ/dt, right only for
  white heading noise (HA-173); the report never uses it.
  *Blast radius if wrong:* too tight ⇒ motions time out on a healthy robot (visible, bounded);
  too loose ⇒ arrival accuracy degrades toward the tolerance (bounded by it). Either failure is
  loud; neither corrupts state.

- [ ] **HA-52 — the spin-vs-motion cross-check thresholds: 0.3 s window, 1.0 in minimum spin
  travel, 25% motion ratio, wheel radius 1.625 in, rotation radius 7.0 in.**
  *Claim:* on real hardware the window/thresholds separate the three regimes they must —
  honest driving (ratio ≈ 1), A3-class slip (ratio ≈ 0.7, no fault), and a dead encoder or
  blocked drivetrain (ratio ≈ 0, ODO_STUCK) — with the stand-in radii close enough that the
  ratio comparison is not distorted.
  *Source:* `include/shulib/motion/odo_stall_check.hpp` (PROVISIONAL (A4: HA-52)); radii
  co-depend on HA-14 (wheel/gearing) and HA-17 (built geometry).
  *Confidence:* **invented** — the margins are provisional-vs-provisional (HA-40's 70% slip
  propulsion is itself a guess).
  *Settle (R3 geometry; R4 slip/noise):* measure real slip ratios and encoder noise; verify a
  hand-held wheel-stall raises ODO_STUCK within one window on the bench.
  *Blast radius if wrong:* too eager ⇒ spurious ODO_STUCK during aggressive maneuvers — and
  since C2 that is no longer only a log smell: the scheduler's default fault policy ABORTS the
  active motion on a new ODO_STUCK (brakes, records the cause, run continues), so a false
  positive costs one motion, loudly and safely, never a crash; too lax ⇒ a dead encoder is
  caught by the watchdog instead (slower, still bounded). The E-phase estimator-side detector
  supersedes this check's load-bearing role.

- [ ] **HA-53 — the C2 cancel safe state: 0 V + `BrakeMode::Brake` brings the drive from full
  speed to rest promptly (visibly faster than coast), with no adverse firmware interaction from
  re-commanding it repeatedly.**
  *Claim:* commanding zero volts under Brake mode on all drive motors is the correct and
  sufficient "safe state" for every cancel path (user cancel, pre-emption, fault abort, panic
  stop): the real V5 drivetrain decelerates at least as fast as the passive/back-EMF coast the
  host plant models, and short-circuit braking does not misbehave when the very next motion
  immediately commands voltage again.
  *Source:* `include/shulib/motion/motion.hpp` `applyCancelSafeState()` (PROVISIONAL (A4:
  HA-53)); consumed by every `IMotion::cancel()` and by `MotionScheduler`'s pre-empt / fault
  abort / panic paths. Host evidence is deliberately conservative: the A2 plant does not model
  brake TORQUE (it pins a wheel at 0 V whatever its mode, and reads the mode only for R3d Part
  2F-1's coast-follow, off by default), so tests prove the 0 V dynamics reach rest (9.7 in coast
  from 57 in/s on the lagged plant) and pin the Brake command by state inspection only.
  *Confidence:* reasoned — Brake mode's short-circuit behaviour is VEX-documented; its stopping
  distance from competition speeds on our robot's mass/wheels is unmeasured.
  *Settle (R3 bench, R5 at speed):* command a cancel from cruise on hardware; measure
  stop distance vs the coast prediction; verify motion-after-cancel resumes cleanly.
  *Amended R5a Part A (2026-09-29):* the GAINS step ends each normal segment with 0 V + Brake and
  measures each side's stopping distance by the encoders, from the end tick to rest (refusing a stop
  past 12 in, HA-170) — four measured Brake stops per run, each from the segment's end speed (about
  30 in/s at the host tests' gains). The plant models no brake torque, so its stopping distances are its τ = kA/kV decay only.
  *Blast radius if wrong:* the robot rolls further than expected after any cancel/abort —
  bounded by the coast physics either way (never re-energized: the 0 V command is proven by
  test), so the failure mode is "stops like coast", not "keeps driving". If Brake proves
  harmful (e.g. brownout interaction), the safe state is ONE function in ONE place.
  *Amended DEFECTS2 (2026-09-26) — until then this stop COASTED on a V5:* the PROS adapter sent
  the zero as `move_voltage(0)`, which ignores the brake mode (HA-142), so every cancel path —
  a user cancel, a pre-emption, a fault abort, the panic stop, and so the driver loops' disable —
  was the "stops like coast" failure mode above, silently, while the host tests pinned "Brake" by
  state inspection. Since DEFECTS2 the adapter sends that zero as `brake()`
  (pinned through a whole `robot::Robot` over the shim: `test/brake_stop_stack_test.cpp` s2, s4,
  s5), so the claim can now be tested at all. Three consequences for the settle step: a library
  stop while ENABLED now brakes while a stick release still COASTS (the team lead's ruling —
  `teleop::DriverLoopConfig::releaseBrakeMode`, default Coast, re-asserted every tick because a
  `drive()` pre-emption cancels into Brake), so the two should look different on robot two; a
  DISABLE is not a clean test of this entry, because the loop's Brake stop goes out once the brain
  already reports disabled, and what VEXos does to motor output by then is HA-145; and "re-commanding it repeatedly" is now one `brake()` per
  transition into the stop plus a `set_brake_mode()` per tick, not a `brake()` per tick (HA-144,
  HA-146).

- [ ] **HA-54 — the H-drive's strafe traction derate ≈ 0.35: a single, lightly-loaded strafe
  omni sustains ≈ 35% of its free surface speed pushing the whole robot laterally on foam.**
  *Claim:* `strafeAuthority = strafeSpeedRatio × strafeTractionDerate` with derate 0.35 is
  achievable AND not grossly conservative on the built 15″ H-bot — the robot really can crab at
  ≈ 0.35 × the linear speed budget sustained, and cannot sustain much more. The RATIO half is
  derivable geometry (gearing × wheel radius, 1.0 for same hardware — not registered); the
  DERATE is the part no geometry can supply: normal force on one omni, foam scrub, single-motor
  torque. C3's motion suites prove the CONTRACT at 0.35 (clamp, fallback, visibility, routine
  accuracy); nothing on the host can prove the NUMBER.
  *Source:* `include/shulib/kinematics/h_drive.hpp` `HDriveConfig::strafeTractionDerate`
  (PROVISIONAL (A4: HA-54)); the master plan §13 #5's locked "HDrive ≈ 0.35 sysid-measured
  default, not a hardcoded constant" is this entry's ancestor.
  *Confidence:* **invented** — the 0.35 is the master plan's placeholder; no strafe wheel
  exists to load.
  *Settle (R5):* sysid the built H-bot's sustained lateral speed at full strafe command on
  field foam (loaded, battery-nominal); set derate = measured/ceiling; re-run the C3 suites.
  *Blast radius if wrong:* too HIGH ⇒ the motion layer commands lateral speed the wheel cannot
  deliver — the robot crabs slower than commanded, closed-loop still converges (slower, may
  TimedOut on tight budgets); the fallback flag under-fires. Too LOW ⇒ lateral legs are
  needlessly slow. Either way ONE config field on ONE preset; no frozen contract carries the
  number (F5 carries the QUERY, not the value — by design).

- [ ] **HA-55 — the 15″ H-bot stand-in geometry: 11″ track width, strafe wheel 4″ aft of
  centre, strafe wheel same cartridge/diameter as the drive (ratio 1.0), 3 kinematic wheels
  (left gang / right gang / strafe).**
  *Claim:* the built H-bot's geometry is close enough to these stand-ins that the C3 host
  results (routine accuracy, fallback behaviour, conditioning margin relDet ≈ 0.94) carry over
  qualitatively. The values are INVENTED — chosen to be plausible for a 15″ chassis and
  deliberately OFF-CENTRE so the non-orthogonal pseudo-inverse path is what the suites
  exercise.
  *Source:* `test/motion_test_rig.hpp` `hBotKinematics()` (PROVISIONAL (A4: HA-55));
  co-depends on HA-14 (drive wheel/gearing) and HA-17 (built-vs-preset geometry, which now
  covers the H-bot too).
  *Confidence:* **invented** — no 15″ robot exists, even on paper.
  *Settle (R3):* measure the built chassis (track width, strafe wheel position/diameter/
  gearing); replace the stand-ins; re-run the C3 suites — any newly-failing bound was resting
  on the guess.
  *Blast radius if wrong:* geometry constants only — `hDrive()` makes the drivetrain DATA, so a
  corrected measurement is a config edit, not a redesign. The one structural sensitivity is the
  strafe wheel drifting NEAR centre (relDet → 1, orthogonal fast path takes over — benign) or
  absurdly far aft (relDet falls; the conditioning guard would reject long before accuracy
  degrades silently).

- [ ] **HA-56 — the D-5 plausibility envelope defaults: maxSpeed 150 in/s, maxYawRate
  20 rad/s, margin ×1.5 (per-tick pose-delta invariant).**
  *Claim:* no build of our robots can move its ESTIMATE faster than these bounds by physics —
  a 600 rpm 4″ drive tops out near 125 in/s and ~3 rev/s is past any real chassis — so a delta
  beyond envelope × margin × dt means the estimate is lying, never that the robot is fast.
  The ×1.5 margin absorbs legitimate non-physical estimate motion (never-snap-clamped fusion
  nudges, discretization) and is a logic constant, not part of this claim. One caller-made
  exception (L6-A1, 2026-10-29): a wall reset's one-axis seat over 2.25 in, seen by a Tier-3
  loop that ticks the scheduler itself, raises one advisory IMPLAUSIBLE while the estimate is
  right — a deliberate jump, not a lie (the guard sees only dt; every verb's dt-0 first tick
  forgives it).
  *Source:* `include/shulib/diag/plausibility_guard.hpp` `PlausibilityConfig` (PROVISIONAL
  (A4: HA-56)); consumed by the scheduler's per-tick check (`MotionSchedulerConfig.plausibility`).
  *Confidence:* reasoned — derived from motor/wheel physics with deliberate headroom; the
  actual drivetrains' top speeds are unmeasured.
  *Settle (R3/R5):* measure top linear speed and yaw rate during sysid; tighten the envelope
  toward measured × margin so the invariant gains sensitivity (today it only catches gross
  lies; a tight envelope catches subtle ones).
  *Settle path note (R5a Part A's review, 2026-10-04):* the GAINS step that R5's ramp became does NOT
  reach top speed — its ramp runs out of its 36 in lane near 6 V on the host test plant — so the free
  speed its gains imply, (12 − kS)/kV, is an extrapolation (HA-171), not a measurement of this
  envelope. Top linear speed and yaw rate still need a full-stick run.
  *Blast radius if wrong (too low):* false IMPLAUSIBLE faults — advisory only (never aborts,
  never rewrites the pose), and episode-gated, so the damage is log noise; the C5 suites prove
  zero false positives across every clean AND hostile run at the defaults. *(Too high):* subtle
  estimate lies pass — exactly today's honest state, which R3/R5 tightening fixes.

- [ ] **HA-57 — the V5 controller text grid is 3 rows × 19 columns.**
  *Claim:* `pros::Controller::set_text` addresses 3 text lines and ~19 visible columns; a row
  written longer than 19 columns must be truncated by our adapter (never wrapped into the next
  row).
  *Source:* `include/shulib/hal/line_display.hpp` `ILineDisplay::kRows/kCols` (PROVISIONAL
  (A4: HA-57)); consumed by `diag::ControllerFaultDisplay`, whose row-1 content ("flt " + the
  longest FaultCode spelling) fits 19 columns exactly, pinned by test.
  *Confidence:* reasoned — matches PROS documentation of the controller LCD; the visible
  column count on real hardware/firmware is unverified.
  *Settle (R1):* display a 25-char test row through the real adapter; count what shows; set
  kCols to the measured value (and re-check the row-1 exact-fit pin — if kCols measures
  smaller, the longest fault names truncate, which the seam contract already handles).
  *Blast radius if wrong:* display cosmetics only — content truncates at the seam by contract,
  so a smaller real grid clips characters, never corrupts rows or logic.

---

## Group R6 — model-shape adequacy (settled by back-fit)

- [ ] **HA-48 — the FF-inversion + first-order-lag plant shape is adequate.**
  *Claim:* with R5's measured constants installed, the A2 plant reproduces a recorded real run
  within R6's documented tolerance — no torque curve, stiction model, or inertia tensor needed
  for control-relevant fidelity.
  *Source:* `include/shulib/sim/motor_model.hpp:6–17` (the honesty boundary + register note).
  *Confidence:* **invented** (a structural bet, deliberately conservative: the plant refuses to
  model what it cannot measure).
  *Settle (R6):* the back-fit — feed measured parameters in, replay a recorded run, compare
  trajectories; every newly-failing suite test is a real defect the invented parameters had been
  hiding (R6's definition of done, verbatim).
  *Amended R5a Part A (2026-09-29):* its robot-side twin is HA-171 — the GAINS step fits exactly this
  law, so the same shape is now a belief about the fit's extrapolation as well as the plant's.
  *Blast radius if wrong:* the model grows exactly where the residual says (that is R6's job) and
  the suite re-runs against the improved plant. Host *logic* conclusions stand — they were
  constructed not to depend on plant constants; what would change is sim-vs-real *quantitative*
  agreement.

- [ ] **HA-49 — leaving the 2.5 A stall current limit unmodeled changes no host-phase
  conclusion.**
  *Claim:* no result certified by the host suite (M2 acceptance, survival matrix, closed-loop
  convergence) would change if stall-current limiting were modeled.
  *Source:* `power_hostility.hpp:28–31` (the honest gap note).
  *Confidence:* **invented** — the gap is deliberate (a current model needs the load model the
  honesty boundary forbids), the no-consequence claim is the assumption.
  *Settle (R6):* record stall-adjacent events (wall pushes, jams) on hardware; back-fit; check
  which host results move.
  *Amended R5a Part A (2026-09-29):* the GAINS step's two step segments record each side's largest
  member current every tick (`CharacterizerSample::maxCurrent`, the current column of R5a Part C's raw
  samples) — once a robot session (S2) runs it, the first measured evidence of where the limit bites;
  nothing has been measured yet. HA-172 is its robot-side twin.
  *Blast radius if wrong:* stall-adjacent behavior in sim is optimistic — affects F′-phase
  mechanism sequencing realism most (jam handling), where F1's hostile mechanism fakes model
  jams *independently* of the drive plant, by design.

---

## Legacy-measured reference points (salvaged at C6, before the C7 deletion)

**Scope, stated carefully:** these numbers were measured or configured on the **previous robot**
("pookster", a 10-motor tank from the pre-v2 era) and are recorded here as **evidence of realistic
magnitudes and as R-phase measurement provenance — NOT as values for any HA entry.** The Override
robots are different machines; every HA stand-in stays exactly as it is until R3–R5 measure *those*
robots. What this table changes is confidence: where a stand-in and the legacy measurement agree in
order of magnitude, the stand-in is corroborated; where they diverge, R-phase should look first.

| Legacy fact | Value (source) | Bears on | Note |
|---|---|---|---|
| Tracking-wheel offsets, as built | L −6.5″ / R +6.5″ / B +2.5″ (`main.cpp:34-36`, corroborated by the stuck-detection log text in `odometry.cpp`) | HA-12 | HA-12's −3.0″/−4.5″ stand-ins are for a *different* (two-wheel + IMU) layout; the legacy numbers show real offsets run ~2–7″ — same order, no change needed |
| Tracking-wheel diameter, as built | 2.75″ (`main.cpp:34-36`) | HA-13 | The harness's nominal 2.0″ is a stand-in; the last real robot used 2.75″ wheels. R3 measures whichever is actually built |
| Rotation sensor ticks/rev | 36000 (centidegree) — `odomUnit.cpp:17` computes travel as `position·πd/36000` | HA-16 | **This is the "measured elsewhere" provenance:** the conversion ran on a real robot all season. HA-16 confidence is effectively settled-by-prior-use; R3 still confirms on current sensors |
| Minimum drive command to move | 20/127 linear, 25/127 rotational (`main.cpp:838-839`, measured by `test_min_output()`'s +1-per-500 ms ramp) | HA-45 (kS) | ≈ 1.9 V equivalent linear kS on that robot vs the 1.0 V placeholder — **R5 should expect kS nearer 2 V** on a comparable drivetrain; the placeholder is likely low, not high |
| Drive geometry, as built | trackWidth 15″, wheel 3.25″, 400 rpm cartridge-equivalent (`main.cpp:38`) | HA-14/HA-15/HA-17 | Corroborates 3.25″ as this team's habitual wheel size (HA-14's guess); rpm differs from HA-15's GREEN/900-ticks stand-in — cartridge is an R3 read-off either way |
| Port map, last season | Drive L {11,−12,13,−14,−15} / R {16,−17,18,−19,20}; Rotation L(−8)/R(10)/B(9); IMU 6; ADI arm 'B', lever 'C'; intake {2,−3}, conveyor {4,−5}, releaser {−6,7} (`main.cpp:23-62`) | R1/R3 runbook | Historical only — since G1 the session header's port map is generated from a robot's config (`robot::Robot::portMap()`), and since G1 Part 2c every program built on `robot::Robot` prints it that way ("shulib Teleop" then, and the invented X-drive, whose map is now a config constant; since T4 Part 3, which retired Teleop, "shulib Drive", the Auton Tester and the competition program); the typed table the bench tester and the boot source read (`src/robots/chassis_table.hpp`) is still typed by hand. Recorded so the *shape* of a real port map (5-motor sides, reversed-port minus convention) informs R1's adapter tests |
| Calibration-routine specimens | `test_min_output()` (kS ramp), `rotation_calibration()` (iterative IMU-vs-wheel θ-scale comparison, ×3 passes) (`main.cpp:101-217`) | M3 calibration routines, R5 | The *procedures* are the salvage: both are legitimate bench routines to re-derive cleanly at M3/R5. The θ-scale factor itself is obsolete (v2 heading is IMU-owned) |

Diagnostic-culture evidence from the same files (stuck-wheel detection, L/R travel-imbalance and
motor veer/temp diagnosis) is recorded in [`diagnostics-plan.md`](diagnostics-plan.md)'s C6 note
rather than here — it is backlog input, not a hardware assumption.

---

## Considered and excluded (with reasons)

Recorded so "exhaustive" is checkable — these were examined and deliberately NOT registered:

| Item | Why excluded |
|---|---|
| F2 accuracy targets (< 1° heading, ~1″ pose, ~0.25″ docked) | **Requirements, not assumptions** — frozen at M0 (F2). The register tracks claims about hardware, not goals for it. HA-20 tracks whether the sensor can *meet* one. |
| The F4 ±12 V motor clamp / mV command units | Documented V5 API contract, encoded and tested as the F4 contract; nothing to falsify beyond HA-47's semantics claim. |
| GPS strip present in Autonomous Skills, absent in Driving Skills | Competition rules, not hardware; already designed for (off-strip mode, HA-31 covers the device side). |
| `Localizer` `maxDt`/`minDt` (0.1 s / 0.1 ms) | Self-referential policy bounds on *our own* loop, not claims about hardware; the hardware-facing part is HA-32/HA-34. |
| Cross-libm bit-identity of sim runs | A documented A2 determinism caveat about *host toolchains*, not about the robot; no hardware measurement settles it. |
| Stochastic truth-side hostility (random per-tick traction) | A3 D10/§3.8: a possible model extension, not an assumption — nothing at M2 needs it; revisit only if R4's data shows slip is noise-dominated (HA-37..39 will say). |
| Pneumatics/air budget, mechanism-specific constants | When written, no mechanism existed even on paper (F′-phase, build-team-gated). *(Since T3, 2026-09-26, robot two's claws exist on paper and in robot two's driver file — their beliefs ARE registered, HA-153..HA-159 and HA-147; what stays excluded is the AIR budget (how many actuations a tank holds — nobody has stated the tank or the cylinders) and the constants of mechanisms not yet declared, which F3's primitives register when authored.)* |
| ~~AprilTag camera intrinsics/mount~~ | **No longer excluded — E3 authored the PnP path and registered HA-68…HA-82.** Camera INTRINSICS themselves remain out: `CameraIntrinsics` has no defaults to be wrong about, and R2 obtains them by calibration. The MOUNT is now HA-70. |

---

## Reconciliation (bidirectional, grep-verified)

Every provisional label in the tree names its register entry, and every register entry that came
from a header points back at a label carrying its ID. Verified at A4 close by:

```sh
# Direction 1 — no label without a register ID (must print nothing):
grep -rn "PROVISIONAL (A4" include/ test/ | grep -v "HA-[0-9]"

# Direction 2 — every register ID cited from this file exists in the tree where claimed
# (script: extract HA-nn per source file above, grep each file for the ID; must find all).
```

Both directions were run clean at A4 close (2026-08-06; the captured output is preserved in
the development log on the `shulib-v2` branch). Entries with no in-tree source (HA-18, HA-33,
HA-47 — sourced from the roadmap, planning-time seeds, and A3's hostile-fake findings
respectively) are exempt from direction 2 and say so in their Source field.

---

## Maintenance

- **Adding an assumption** (any later chunk that invents a magnitude): label it
  `PROVISIONAL (A4: HA-nn)` in-header and add the entry here — the A3→A4 pipeline is now the
  standing convention (C1 followed it: HA-50–52; E1 followed it: HA-58–60). Phase E's remaining
  chunks (EKF noise priors) and F1 (mechanism fakes) are the known next contributors.
- **Settling an entry** (R3/R4/R5/R6): check the box, record the measured value beside the
  guess, update the in-header constant, cite the measurement log. If the measured value breaks
  a test, the test was resting on the guess — fix forward per the R6 rule (a new failure is a
  real defect that had been hiding).
- **This register is the R3 runbook.** R3 — hardware day one — is committed to walking it
  top to bottom; keep the R3 group in bench-practical order.

---

*Created by chunk A4, 2026-08-06 — the chunk that closed Phase A. Companion document:
[roadmap.md](roadmap.md) (status; Phase R owns settling every entry here). The 25-claim seed
inventory this register subsumes and extends to 49 — and the full A4 working record — live in
the development log on the `shulib-v2` branch.*
