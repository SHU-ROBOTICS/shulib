# R3b Part 0b — the DRIVE PROGRAM: robot two drives from the sticks, through the adapters

> Addendum to [`R3b-session2-tank-chassis.md`](R3b-session2-tank-chassis.md), written 2026-09-10
> late, on the team lead's instruction: *"make a new program just for driving since we know things
> work and leave bench tests. Some ports might die during comp; 1–2 dead ports shouldn't stop
> driving, just let it be a warning that gets logged. We need to make the program now."* Also:
> *"make sure it is known that the IMU right now is on port 2."* Live log: append to
> [`R3b-PROGRESS.md`](R3b-PROGRESS.md) as **"Part 0b"**, created FIRST. Tree at start: `f4c3218`.

## 0. What is known, as of tonight (all in the tank table, `src/bench_r3a.cpp`)

| Fact | Value | Provenance |
|---|---|---|
| Ports, left, back → front | 11 12 13 14 15 | census, all ten answered, 2026-09-10 evening |
| Ports, right, back → front | 20 19 18 17 16 | census |
| **Per-motor signs** | **LEFT −11 +12 −13 +14 −15 \| RIGHT +20 −19 +18 −17 +16** | MOTOR WATCH, two front-first pushes, identical (Session 2 §10.3) |
| Front | the 15/16 end | team lead |
| Cartridge | blue | read off a motor (a belief that passes read-back; volts unaffected) |
| **IMU** | **port 2** | census; team lead confirmed |
| Radio | port 1 | census |
| Port 18 | travels ~20 % short of its side-mates on a push, twice | unexplained; watch under power |
| Station 10 DRIVE | ran on hardware: ten `ProsMotor`s constructed, gearset written/read back, all gates passed, L1/R1 read; **no motor ever commanded (L/R stayed 0.0 V)** | §10.4 — the stick/axis question is still open |

**The governing line is unchanged:** nothing here is the library's motion stack; this program drives
through `hal/pros` adapters with open-loop volts, and the banner says so. The library has still never
driven a robot.

## 1. RULING — shape: one binary, the drive program is the DEFAULT, the tester one tap away

The tank binary boots into **DRIVE** unless the operator taps **BENCH TESTS** on a two-button brain
chooser during a 3-second window at `opcontrol()` start. Under field control
(`pros::c::competition_is_connected()`), **no chooser**: straight into DRIVE, no seconds lost.

Rejected: a separate binary in another slot via a `PROGRAM=` make switch. It doubles the uploads the
build team has to keep straight, needs the `src/` build gate to learn a second axis (define SETS per
variant, a bigger change than the program), and forks two composition roots that would drift. The
chooser gives them "a program that just drives", keeps Bench Tests byte-for-byte reachable for
diagnosis at the field, and was the shape session 2's brief already planned (§6.3). Rename the
program in `project.pros` to **"shulib Drive"**; the tester is inside it.

> **§1 AMENDED the same night, on the team lead's call: a SEPARATE program in its own slot, no
> chooser.** *"It'll be a separate program just for driver controls, right?"* — yes. The operator
> picks a program by name from the brain's slot list, and a program called Drive that hides the
> tester behind a timeout is one more thing to explain at a field. Shape: a second build axis,
> `make ROBOT=tank PROGRAM=drive` (`-DSHULIB_PROGRAM_DRIVE`; `PROGRAM ?= tester`, validated;
> `drive` is an `$(error)` for `bench` and `xdrive`, which have no signed table), uploaded as
> **slot 1 "shulib Drive"** (`pros upload --slot 1 --name "shulib Drive"`); the tester stays
> **slot 3 "Bench Tests"** (`pros upload --slot 3`), `project.pros` unchanged. A fourth beacon
> (`shulib-robot-variant=tank-drive`) and the `src/` build gate learns the second axis: four
> builds, each with an expected define SET, its beacon, and the behavioural expectation
> (tank-drive like tank); the self-test gains the non-landing-`PROGRAM`-define case. The
> rejected alternative above stays recorded as what was first ruled and why it lost.

## 2. RULING — the signs live in the table now; MOTOR WATCH becomes the cross-check

The chassis table's port lists become **signed** for robot two (the measured signs above), with the
provenance line. The bench bot's table stays as it is (its signs were never captured into a table;
its DRIVE station keeps using the capture). Consequences:

- `driveStation()` uses the TABLE's signs when the table carries them, and the MOTOR WATCH capture
  otherwise; when both exist and disagree, it refuses and names the port — a table that lies about a
  sign is exactly the fight the cut-out exists for.
- MOTOR WATCH, after a qualifying push on a signed table, prints AGREES / DISAGREES per port.
- R3d makes this automatic and persisted; tonight it is a typed, measured value with a date.

## 3. The drive program — `src/drive_program.cpp` + `.hpp`, `bench::runDrive()`

Read the DRIVE station (`src/bench_r3a.cpp` `driveStation()`, `namespace drive`) first: the program
is that station with the menu, the capture dependency and the dead-man removed, and degradation
added. Move the chassis table and its helpers into `src/chassis_table.hpp` so both TUs share ONE
definition (the gate globs `src/` for TUs; three is fine).

**Boot (in `opcontrol()`, after the chooser):**
1. Banner to serial (+ SD if present): build stamp, hash, table with provenance, **"adapters, NOT the
   motion stack"**, the IMU port stated.
2. Construct one `ProsMotor{signedPort, Blue}` per table port **inside a try per motor**. A port that
   refuses (no device, wrong device) is a **WARNING, logged with the port and the side, shown on the
   brain panel and the controller LCD — and the program drives on without it.** The degradation
   policy is a PURE function in `include/shulib/teleop/drivetrain_degradation.hpp` (PROS-free,
   host-tested): given the table's per-side counts and the per-side answering counts, it returns
   DRIVE / DRIVE_DEGRADED (with the counts) / REFUSE. **Rule: refuse only if any side has fewer
   than three answering motors or the total dead exceeds two**; otherwise drive, degraded. State
   the reasoning in the header: two motors of five is a side that cannot pull its share and will
   drag the robot into a curve at speed; three of five drives.
3. `ProsController` master; `ProsLineDisplay` LCD; `ProsBattery`.
4. **Runtime dead-port detection:** a member whose `faultedReads()` (the adapter's counter) advances
   for 250 ms straight, or whose velocity reads exactly 0 while its side-mates exceed 1 rad/s under
   command for 250 ms, is marked ABSENT at runtime: logged once with the port, dropped from the
   fight detector's agreement set, its voltage still commanded (harmless), and shown as `ABSENT` on
   the panel. Re-evaluate the degradation policy; if it now says REFUSE, cut to 0 V and paint why.

**Loop, 10 ms:** sticks → `teleop::mapSticks()` (the one shared mapping; arcade, left Y forward,
right X yaw) → per side `volts = kMaxDriveV × (forward ∓ yawCcw)` clamped to ±12 V → the same volts to
every constructed member of the side. **No dead-man** — a driver program drives while the stick is
deflected; a disconnected controller commands 0 V (the mapping already does). `kMaxDriveV = 12.0`
with a comment that T2 owns curves, slew and per-driver limits.

**The two protections, kept from the station and made non-fatal:**
- **Fight cut:** the station's gate-5 logic, extracted into a PURE per-side evaluator in
  `include/shulib/teleop/coupled_side_monitor.hpp` (PROS-free, host-tested): inputs are the
  command sign and each member's velocity + present flag; output is which members disagree
  (opposite sign above 0.5 rad/s, or under 25 % of the side's fastest while the side is commanded
  above 1 V and moving above 1 rad/s), with the 250 ms persistence counted inside. On a persisted
  disagreement: **both sides to 0 V for 1 s**, log the port(s) and the reason, count it, re-arm.
  Not fatal, because a match must not end on a transient; visible, because a fight is damage.
- **Over-current:** any member above 2.4 A for 250 ms → the same 1 s cut and log.

**Panel (brain) and LCD:** a compact status — build stamp, mode, battery volts, `L ±V R ±V`, one
row per port (`signed port  rad/s  A  C`, colour: green moving / dim idle / amber disagreeing /
red cut / grey ABSENT), a WARNINGS line (dead ports listed), the last cut reason and the cut
count. LCD: row 0 `DRIVE 12V  Bxx.xV`, row 1 warnings (`dead: 18` or `all 10 ok`), row 2 last cut
or `ok`. LCD writes only on change, ≤ 5 Hz.

**Logging:** everything the station logged, plus one line per state change (a port absent, a cut,
a cut re-arm), to serial and to `/usd/drive_log.txt` when a card is present (the T5 no-card rule:
never stop for a missing card; say once at boot that logging is off).

**Competition behaviour:** `autonomous()` stays no-motion (unchanged); `disabled()` paints the state
screen (unchanged); `opcontrol()` runs DRIVE. Under field control the chooser is skipped. A
field-disabled state stops the loop the normal way (PROS kills the task) — the RAII all-stop guard
from the station runs on that exit too; keep it.

## 4. Tests — host, adversarial, mutations run

| # | Test | Bug it catches | Mutation → RED |
|---|---|---|---|
| 1 | Degradation policy: 5/5+5/5 DRIVE; 4/5+5/5 DEGRADED(1); 4/5+4/5 DEGRADED(2); 3/5+5/5 DEGRADED(2); 2/5+5/5 REFUSE; 4/5+4/5+1 more dead (3 total) REFUSE; the exact boundary rows | a side with two motors dragged into service; a healthy robot refused | `< 3` → `< 2`; total `> 2` → `> 3` |
| 2 | Coupled-side monitor: all agree → none; one opposite sign → that one after exactly 25 ticks, not 24; one near-zero while mates move → that one; NOT evaluated under 1 V or under 1 rad/s; an ABSENT member never flagged; a stick reversal (all members briefly opposite) clears before 25 ticks | false cuts on a reversal; a real fight missed; a dead port flagged forever | drop the persistence; drop the present-flag check; flip the sign test |
| 3 | The mapping in the drive loop is `teleop::mapSticks` (test 14's existing pin covers the mapping; pin the per-side arithmetic `12 × (f ∓ y)` clamped, both signs) | the driver's stick drives the wrong way at 12 V | swap ∓ |
| 4 | Signed-table consistency: a table with signs must have every port signed, no duplicates by absolute value, both sides non-empty | a half-signed table | remove a check |

The drive loop itself is PROS-bound: its verification is the ARM build through the gate, plus a
walk-through in the log quoting each guarding condition (boot construction with the try, the
degradation call, the cut and re-arm, the all-stop guard).

## 5. Definition of Done

- [ ] `make ROBOT=tank` boots into DRIVE after the 3 s chooser (BENCH TESTS still reachable); under
      field control no chooser; `project.pros` names it "shulib Drive"; bench and xdrive variants
      unchanged in behaviour (bench still boots the tester directly — it has no signed table)
- [ ] The tank table carries the measured signs with provenance, `imuPort = 2`; the station and
      MOTOR WATCH honour §2
- [ ] Dead-port tolerance per §3 with the policy pure and tested; runtime absence detection
- [ ] Fight cut and over-current as 1 s non-fatal cuts, logged and counted, the evaluator pure and
      tested; every mutation in §4 observed red then restored
- [ ] src build gate PASS (three variants, now three TUs), host suite green with the new tests,
      the eight doc gates (two new public headers → `///` on every member, `docs/api` regenerated,
      nav generated), ARM header gate
- [ ] Docs in the same commit: worksheet **Station E — DRIVE PROGRAM** (chooser, what the panel
      shows, what a WARNING and a CUT mean, the ground-run rule: 3 m clear, second person, stop on
      any red); Daniel's note updated (the program now boots into DRIVE; IMU port 2; the signs are
      in the table); R3b-PROGRESS "Part 0b"; roadmap "you are here" and build-order Next block:
      **robot two can be driven through the library's adapters; the motion stack still has not
      driven a robot**; `docs/README.md`/`docs-publishing.md` counts
- [ ] Nothing committed by the executor; `PART 0B READY FOR VERIFICATION` at the end of the log

## 6. Landmines

1. **Signs in ONE place.** The table's signed port goes to `ProsMotor`; the program never negates.
2. **A dead port must not become a silent zero.** Warn on the panel, the LCD and the log, every boot.
3. **The cut must re-arm.** A permanent cut is the station's rule (diagnosis); the program's rule is
   1 s and log, because a match must not end on a transient.
4. **Do not touch `include/shulib/` except the two new PROS-free teleop headers.** Frozen surfaces
   stay frozen; the PROS-free guard applies.
5. **`xdrive` must still emit 0 `-Wunused-function` from `src/main.cpp`** (the gate's behavioural
   detector); guard any tester-only helper exactly as `benchStateScreen` is guarded.
6. **The tester's DRIVE station stays as the wheels-up diagnosis tool**; do not remove its gates.
7. **The IMU is on port 2.** It is in the table; the drive program prints it at boot; it is not used
   by this program (no heading, no odometry — those are Parts 1–3).
8. Baseline: `f4c3218`, suite 1163 / 1,538,453 clean, 152 headers, gate PASS ×3.

*Created 2026-09-10. Companion to R3b session 2; supersedes its §6.3 wording for the chooser.*
