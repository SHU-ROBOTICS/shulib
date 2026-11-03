# MECH-S + TUNE1 — a string lift's SAFETY (a per-motor CURRENT CAP the motor itself enforces; a SLACK watch that sees a snapped string in the motor's current, with a bench step that measures its thresholds and refuses what it cannot tell apart) and LIVE TUNING on the robot (a mechanism's numbers changed on the brain screen inside the same rules as the build, watched, undone, and copied back into the robot file through the panel)

> **THE TEAM LEAD'S ANSWERS (2026-11-03)** to D1–D13: **"Accept all, go (Recommended)"** — MECH-S includes live
> tuning; the runs in the brief's order (MECH-S-A the cap → TUNE1-A → TUNE1-B1 → MECH-S-B1/B2 the snap watch, TUNE1-B2
> after the panel). **His answers to the brief's three questions:** the string lift is on **ROBOT ONE, which is BUILT**;
> the lift is **"strings both ways"** (a continuous or two-string pull, up AND down) — NOT the string-up / gravity-down
> lift this brief assumed. **So MECH-S-B (the slack watch and STRING CHECK) and D5's homing rule are re-checked and
> amended for a both-ways lift before MECH-S-B's go** (a snapped UP string drops the lift under gravity; a snapped DOWN
> string leaves it unable to descend under power; pushing down to home pays out no string); the cap (MECH-S-A) and live
> tuning (TUNE1) are unaffected. D6's correction stands: a rotation sensor on the SPOOL's shaft cannot see a snap; a
> carriage-driven sensor can — its own small run after the panel. Robot one's facts (ports, cartridge, mechanisms, the
> string's rating) are owed by the team. HA-219..HA-227 reserved.

> **Status 2026-11-03: DRAFT for the team lead's go, RE-CHECKED ~14:00 (16 findings — "## Re-check — amendments" at
> the end; every corrected place is marked *(re-check #N)*), the re-check FINISHED ~16:00 after the usage stop (its
> "Continuation" block: four more corrections, marked *(re-check C1–C4)*). WAITS for his answers to D1–D12 and three
> questions (D10), then for a go per run (six runs, five before the panel — §2.1). Nothing is built.**
> Execution brief written by the coordinator's brief writer from a READ-ONLY pass over branch `shulib-v2` at
> **`eab0f89`** (main; the G3a plan commit of 2026-10-30). *(re-check #11)* VIS1-A1 has since COMMITTED as
> **`9d77c9d`** (2026-11-03; it touches `hal/device_provider.hpp`, `hal/pros/device_provider.hpp`, `robot/robot.hpp`,
> `config/robot_config.hpp`, `test/calibrate_program_rig.hpp`, the HA register and the plan documents — no mechanism
> file), and L6-A1's merge is STAGED on main (localization, `hal/distance.hpp`, `sim/field_plant.hpp` — no file this
> brief cites by line). Worktrees read, never edited (§6.1): `mech-a6` (MECH-A6 building: `src/programs/bench_r3a.*`,
> guide 18, the HA register, the worksheet), `cal1-a2b`, `imu1-b`, `l6-a1`, `g3a-p1`, `vis1-a1`. STEP1's draft brief
> (`.claude/coordinator/briefs/STEP1-route-steps-library.md`) read for its HA reservation, its on-failure rule and its
> D8 check functions. *(re-check C2, ~16:00:)* **L6-A1 has since COMMITTED as `59d93bb`** (main's tip; no mechanism,
> `src/`, `Makefile` or plant file changed between `eab0f89` and `59d93bb` — `git diff --stat` empty for them; it
> moved lines in the HA register, `build-order.md` and `REMAINING-WORK.md`, so their citations stay at the commits
> named); **STEP1's brief is ACCEPTED** (the team lead, ~14:30, "Accept all, go": D1–D8, so D8 (a)'s check functions
> are in; HA-216..HA-218 reserved — `NOTES.md` :910-911; L6-A1's commit :918); and **AF1's draft brief**
> (`.claude/coordinator/briefs/AF1-toggle-auto-flipper.md`, 13:55, still under its checker) was read for its shared
> files — it does NOT edit `service.hpp` (§6.2).
> **Every `file:line` below was OPENED at `eab0f89`, EXCEPT in the files VIS1-A1 changed —
> `hal/device_provider.hpp`, `hal/pros/device_provider.hpp`, `hal/read_faults.hpp`, `docs/hardware-assumptions.md` —
> which are cited at `9d77c9d`** *(re-check #11: the draft said all at `eab0f89`; those lines were its staged tree's, identical to the
> commit's)* (`~:N` = within a few lines). The mechanism files (`mechanism/`, `sim/mechanism_plant.hpp`,
> `manipulation/`) are byte-identical at both commits.
>
> **CONTINUATION (2026-11-03).** The first writer run (2026-10-30, stopped at ~15:40 by the team lead's pause) left
> nine probes (P1–P9, `probes/p_string.cpp`, `probes/p_tuning_rules.cpp`) and no draft. This run REBUILT both at the
> current tree — byte-identical output (the first run's copies kept as `out/*.1030.txt`) — kept every one, found one of
> them (P4's cap model) unable to judge an impact and replaced its claim by P10, added P10–P12, and wrote the brief.
>
> **The team lead's words this brief holds (BINDING).**
> - 2026-10-29 ~20:05: *"additionally we will have things with pullies with strings that can snap so when making
>   mechanisms with strings we need to be precise and things needs to be able to be tuned"*; the string mechanism is
>   **A LIFT**; he RULED **three things in BEFORE THE PANEL LANDS (~Nov 21)**, as library work beside the panel lanes:
>   (1) a **per-mechanism CURRENT CAP** (pulled forward from MECH's rest — MECH-PANEL §3.1/§3.2; §5 Q15 "firmware or
>   software" still open: **D1**); (2) **SNAPPED-STRING DETECTION**; (3) **LIVE TUNING ON THE ROBOT** — change a
>   mechanism's gains, speeds and limits on the brain screen, watch the response, then copy the numbers into the panel
>   (`.claude/coordinator/NOTES.md` :845-849; `REMAINING-WORK.md` :110-111, rows 20e/20f; `build-order.md` :848, :855).
> - 2026-10-29 ~20:15, his question *"cant we detect string snapping by resistance on the motor?"* — he was told: yes,
>   by CURRENT (load), not resistance — a SLACK signature, the stall signature's mirror; thresholds measured at the
>   bench string-on vs unhooked; blind at the bottom and while lowering; the encoder still tracks the spool after a
>   snap; V5 current is noisy; current-based by default, an output rotation sensor optional and stronger. **This brief
>   CORRECTS one thing he was told** (D6): a rotation sensor on the lift's output shaft — the spool's shaft — is BLIND to
>   a snap (probe P3: it reads exactly what the motor reads); only a sensor the CARRIAGE moves can see one.
> - ~~2026-10-30 (G3a's D20, "Accept all"): the five slots' order until Nov 21 — the 11-23 track, make-and-drive, the
>   route maker, **then the string chunks**, then walls, then the camera (`G3a-robot-profile-generator-panel.md`
>   :1247-1330).~~ *(re-check #1: SUPERSEDED the same day this brief was drafted — next bullet.)*
> - **2026-11-03 (BINDING, `.claude/coordinator/NOTES.md` :905-906):** ~13:30 *"AUTONS FIRST"* — slot order:
>   competition work → the auton library (CAL1-B, L6-A2/B, STEP1-A/B, MECH-S) → panel parts; ~13:45 *"EVERYTHING INCL.
>   FLIPPER before the 21st; panel AFTER; within limits (no credits)"* — slot order: **competition in flight → AF1
>   brief, VIS1-A2, ALLY1(+L4) brief, VIS1-B, AF1 build (start by ~Nov 8) → L6-A2/B, STEP1, MECH-S, CAL1-B → CURVE1
>   brief+build → panel**; G3a Part 5 DEFERRED (to after the 21st). The first competition is **2026-11-23**; the code
>   freeze **~Nov 16**.
>
> **Executor model: `opus`. Do not commit. Live logs `docs/internal/chunks/MECHS-PROGRESS.md` and
> `docs/internal/chunks/TUNE1-PROGRESS.md`, created FIRST** (a heading per run). Say "host test plant", never
> "simulator". **No hardware claim anywhere: NO STRING LIFT EXISTS IN ANY ROBOT FILE** — robot two's list is two Power
> claw pivots and a pneumatic claw (`src/robots/mechanism_table.hpp` :113-135), robot one has no build, and nothing here
> has run on a robot. **Hardware Assumptions: HA-219..HA-227 ONLY** (§4 — reserved AFTER STEP1's HA-216..HA-218; a
> tenth belief ⇒ stop and tell the coordinator). **Memory rule (2026-10-15):** every binary — probes, the suite, any
> test — runs as `(ulimit -v 4194304; timeout 300 …)`; the suite only through `python3 tools/suite_run.py`.
>
> **Sizes (rough — no precedent for a live-tuning program; each against the nearest brief's; *re-check: re-sized*):**
> BEFORE THE PANEL — MECH-S-A (the cap) **~2¼–3**, TUNE1-A (live tuning's library) **~2½–3¼**, TUNE1-B1 (the Tune
> program on the brain) **~3½–4¼**, MECH-S-B1 (the slack watch) **~2¾–3¾**, MECH-S-B2 (its STRING CHECK page)
> **~1¼–1¾** — **~12¼–16**; AFTER THE PANEL — TUNE1-B2 (the copy-back into the robot file; it needs G3a Part 5)
> **~1¼–1¾**. **~13½–17¾ builder-days in all** (REMAINING-WORK: 20e "not sized", 20f "rough ~2–3"; G3a's D20 guessed
> "three runs ~1 build day each" — this brief is about four to five times that, §6.4). *(re-check #1, #3–#10: the
> draft's ~11¾–15¾ grew by the fixes; the rule 6 (i) shares are gone — Part 5 is after the panel and adopts the
> vocabulary itself.)*
> **Shape:** the house style of `docs/internal/chunks/L6-wall-reset.md`.

---

## DECISIONS FOR THE TEAM LEAD (a recommendation for each; nothing below decides them silently)

**D1 — The current cap: the MOTOR's own limit (firmware) or the library's (software)? MECH-PANEL §5 Q15. RECOMMENDED (a).**
- **(a) Firmware.** The library tells each motor of a capped mechanism its limit once at boot (PROS
  `motor_set_current_limit`, `include/pros/motors.h` :1146; "The default value is 2500 mA", :1255), reads it back, and
  the MOTOR enforces it from then on — every command, the Hold brake at 0 V, the homing push, a disable, the end of an
  auton, and a student's blocking loop that starves the service. **On the host test plant (P10, STARVED): a 12 V
  push into an obstacle with nothing ticking the service for 1.5 s pulled 2.50 A (for 1.03 s after contact) with no
  cap and 2.50 A under a software cap — through the REAL service, so that half is a measurement of the library: a
  starved software cap does nothing (the service's own D11 measured 11.74 V held for 1.5 s). Under a 1.2 A
  firmware-style cap it pulled at most 1.20 A — but that cap is the PROBE'S OWN clamp, applied every 1 ms
  (`probes/p_cap.cpp` :6-11), so that number is MODELLED, not measured: whether a real motor holds its limit like
  that is HA-219.** *(re-check #13: the draft called all three "measured".)* **Cost:** a new seam beside the LOCKED `IMotor` (F4 —
  `docs/roadmap.md` :138; MECH-PANEL §6 :732 planned exactly this: "a NEW sibling seam … the `IDigitalOut` / `IController`
  / `IDigitalIn` precedent — so F4's row is untouched and there is no F4 bump"): `hal::IMotorCurrentLimit`, implemented
  by `ProsMotor` and `FakeMotor`; one more question every device provider answers (`currentLimit(port)`); the host test
  plant learns to honour a limit. And three beliefs nobody has measured (§4): that the motor holds the limit within a
  tick (HA-219), that a limit set by one program does not silently carry into the next (HA-220 — the library therefore
  writes or checks it at every boot), and how it meets VEXos's own sharing with more than 8 motors (HA-221; robot two
  runs 10).
- **(b) Software.** The service clamps each tick's volts so the expected current stays under the cap. **Cost:** only
  while ticked — a blocking `then()` or a `pros::delay` loop leaves the motor at its last command, uncapped (P10
  STARVED above; the service's own "honest limit", `mechanism/service.hpp` :104-113); one tick late; it needs a model of
  each motor's back-EMF and resistance (invented, HA-183's kind) or a feedback loop on a noisy reading; and the Hold brake
  at 0 V is outside it (the motor's own hold pulls up to 2.5 A).
- **(c) Both, firmware as the floor.** **Cost:** (a) + (b) for nothing a string needs.
- **Not in this decision:** MECH-PANEL's "low-power hold" (Q15's other half — hold with just enough current) stays in
  MECH's rest (§5).

**D2 — The cap's form: PER MOTOR, the team's number, no default; optional on every motor kind; a string lift without one
is WARNED at boot, not refused. RECOMMENDED (a).**
- **(a)** `.currentCap = {.amps = 1.2}` beside `.stall` on a Power, an Arm, a Lift and a Flywheel. **Per motor** — the
  firmware's unit, the stall signature's unit (`maxCurrent()`, the most loaded motor — `hal/mechanism.hpp` :256-266)
  and the reading's unit; **a lift with two motors on one spool pulls its string with TWICE the cap**, and the boot line
  says both (`'lift': current cap 1.20 A per motor (2 motors: 2.40 A in all) -- set and read back`). No default: no cap
  stated = the motor's own 2500 mA. The build REFUSES a cap at or below 0 or above 2.5 A, and — because **a stall
  signature at or above the cap can never trip** (P10: a 2.0 A signature under a 1.2 A cap ended TIMED_OUT after 1.33 s
  pulling at the cap, no fault; P11: a 1.2 A homing signature under a 1.0 A cap pulled at the cap for 3.34 s and never
  homed) — **every stated stall signature's amps must be below the cap** (the move's, the homing's, a power or flywheel
  one's). *(re-check #3)* **A signature within 0.2 A of the cap boots with a Warn** (a real capped motor may read up to
  ~0.1 A under its cap — HA-223 — so a signature just under it may never trip; the build cannot refuse on an invented
  margin). A STRING lift (one that declares `.slack`, D3) with no cap boots with a Warn line. A cap the motor would not
  take (it reads back a different number) is a Warn naming the port and both numbers, and the lift still runs; the
  service re-checks each capped motor once a second and re-writes one that reads back the motor's DEFAULT 2500 mA (a
  motor that lost power comes back at its default — HA-220); one that reads back any OTHER number is a Warn, once, and
  is NOT re-written (something else set it — VEXos's sharing is HA-221's open question — and the library must not
  fight it every second). *(re-check #8: the draft re-wrote every change.)*
  **Choosing the number — the ORDERING the four currents must keep** *(re-check #3: the draft stated only "signature
  below the cap" and "cap above the need" — and the stall watch runs on the HOLD too, `service.hpp` :84-89, :985-995)*:
  **the heaviest HOLD current (with a game piece) < the stall signature's amps < the cap − 0.2 A (HA-223) — and the
  cap > the heaviest MOVE's peak current — and the cap ≤ what the string can take.** Measured with a firmware-style cap
  model (the checker's V1, re-run here byte for byte — `recheck/out/p_verify.txt`): cap 1.2 A, signature 1.0 A (the
  build's "below the cap" rule kept): a 3 V lift holding 0.56 A held 12 s clean; **the same lift after a 2.5 V piece
  was added drew 1.08 A holding and ended STALLED** (the piece pushed the hold above the signature); a 5.4 V lift (hold
  ~1.06 A) ended STALLED 0.52 s into its move and fell to −0.01 in (the cap left too little to get it moving faster than
  the signature's 5 rpm). The guide's recipe, from the Tune program's readings (TUNE1-B1): the TEST report shows each
  move's PEAK and the HOLD current after it settles (with and without a piece), and warns when the stall signature is
  not above the hold or the cap is not above the peak; the cap's ceiling is the string's rated strength over a safety
  margin, divided by the pull per amp — about 10 lbf per amp per motor for a red cartridge on a 1.5 in spool, HA-222,
  NOT measured. Too low a cap also limits acceleration — moves slow down, time out or stall at their start (§8
  landmine 18).
- **(b)** Required on a string lift (a build refusal). **Cost:** a lift cannot be built before its cap is measured, so
  the first number typed is invented — the opposite of "none invented as safe".
- **(c)** One number per MECHANISM, split among its motors. **Cost:** a unit nobody reads anywhere else (the stall and
  slack signatures and the reading are per motor); a 2-motor lift with one motor unplugged would get half the pull it
  was tuned for.

**D3 — The SLACK signature: what it watches and where its numbers come from — the two thresholds MEASURED, two numbers
DERIVED from the same bench run, two STARTING values said as such; a phase the bench cannot separate by a clear margin
is REFUSED. RECOMMENDED (a).** *(re-check #5: the draft's headline said "none invented, each measured" — only the two
amps thresholds were; corrected in place below.)*
- **(a)** A lift that hangs on a string declares it: `.slack = notMeasuredYet` (a string lift whose watch is OFF — said
  at boot) or, once measured, `.slack = {.cruisingBelowAmps = …, .aboveMotorRpm = …, .lasting = …, .holdingBelowAmps =
  …, .holdingLasting = …, .offBottom = …_in}`. The watch trips when **every motor of the lift draws at most
  `cruisingBelowAmps` while the smooth move CRUISES UP (neither speeding up nor slowing down) faster than
  `aboveMotorRpm`, continuously for `lasting`** — or **at most `holdingBelowAmps` while it HOLDS above `offBottom`,
  after the hold has lasted `holdingLasting`, continuously for `holdingLasting`**. A driver's push up at a steady volts
  counts as cruising; *(re-check #5)* **a driver's push held into the TOP soft stop counts as a HOLD** (the jog's
  soft-stop hold, `service.hpp` :1061-1085, is `Mode::Jog`, not `Mode::Hold`, and its speed is ~0, so neither window
  of the draft watched it — measured, `recheck/out/p_recheck.txt` R3: a +6 V push held at 24.00 in drew 0.146 A (1 V
  load) / 0.354 A (2 V) with the string whole and 0.043 / 0.062 A after a snap, under the bench's 0.104 / 0.208 A
  holding thresholds). **Why only those two phases, measured:** slowing down near the top, gravity does the braking and
  the current drops toward 0 with the string whole — a watch that also looked there tripped on intact moves (P8: the
  "naive" watch FALSE-TRIPPED in the deceleration at three of four loads); speeding up, an unhooked spool's own
  acceleration draws as much as a loaded cruise (P12: at a 1 V gravity load the UNHOOKED run's acceleration reached
  0.420 A against the string-on cruise's low of 0.229 A — not separable — while CRUISE alone separates, 0.193 vs
  0.229). **Where the numbers come from** *(re-check #5: every member now has a stated source)*: the **STRING CHECK**
  page in the Tune program (MECH-S-B2) runs the same moves THREE times with the string ON, then three times with the
  string UNHOOKED (the carriage resting on its stop, the string off the carriage — one UNHOOKED run per string whose
  snap matters, below), holding at the top, the middle and four LOW heights (0.5, 1, 1.5 and 2 in), each hold reached
  once from ABOVE and once from BELOW. Then:
  - **`cruisingBelowAmps`, `holdingBelowAmps` — MEASURED:** MIDWAY between the ON runs' lowest sample and the UNHOOKED
    runs' highest in that phase; **a phase is REFUSED unless that gap is at least the larger of 0.03 A and four times
    the reading's noise** (σ̂: the standard deviation of successive samples' differences over √2, the larger of the
    two runs' — the slow swing along a cruise is not noise), with the sentence ("cruising up: string ON drew at least
    0.21 A, UNHOOKED at most 0.19 A -- 0.02 A apart, the noise needs 0.08 A: current cannot tell them apart on this
    lift"). *(re-check #5: the draft accepted any gap above 0 — 0.001 A would have passed.)* The floor and the factor are
    library numbers, INVENTED (HA-224 gains them): P12's tightest gap with no false trip was 0.036 A at no noise
    (accepted) and 0.038 A at σ 0.02 (refused — 4σ̂ ≈ 0.08).
  - **`aboveMotorRpm` — DERIVED:** half the SLOWEST motor speed among the ON runs' cruise samples (every cruise the bench
    made is watched; a gentler push than the bench's may not be — the guide says so).
  - **`offBottom` — DERIVED:** the lowest test height whose holds (from above AND from below) all drew at least the ON
    runs' holding low, plus 0.5 in. Below it something other than the string may carry the carriage (the bottom stop
    under a stretched string or a drifted zero), so the hold half does not watch there.
  - **`lasting`, `holdingLasting` — STARTING values, INVENTED** (0.10 s and 0.30 s, P8/P12's; HA-182's kind): a
    midway threshold sits below every ON sample the bench saw, so the bench run cannot measure how long a whole string's
    dip may last — the boot line says "starting value" beside each, the page lets them be tuned live, and STRING CHECK
    reports its margin (the ON sample closest to each threshold).
  The numbers go into the robot file like any stated fact, with their provenance (D9).
  **What it buys, on the host test plant (P8, P12):** a snap while cruising up is seen 0.11–0.16 s later, one
  while holding 0.34–0.37 s later, with NO false trip in any of 20 intact three-leg cycles wherever a threshold was set
  — at a 2 V gravity load with a reading noise up to 0.05 A; at a 1 V load only up to 0.02 A (cruise) and 0 A (hold);
  at 0.10 A of noise neither load separates (the V5's noise is unmeasured — HA-224). (Those cycles held at the top and
  the middle only — not near the bottom, in a jog's soft stop or while homing; S-B1's SB4 adds them.) **What it cannot
  see:** a snap while LOWERING (gravity does the work either way — seen only at the next hold above `offBottom` or the
  next cruise up: 2.18–2.20 s later in P8 and P12 (at the next hold); **3.48–3.50 s when the hold half was refused** (at
  the next climb — P12, 1 V, σ 0.02) — and with the hold half refused, **a snap WHILE HOLDING is seen only at the next
  climb, 5.70–5.71 s later** *(re-check #12: the draft gave 5.7 s to lowering; it belongs to holding)*); anything at the
  BOTTOM (the carriage rests on its stop either way); a lift too light, or too noisy, for its current to be told apart
  (refused at the bench, and then that half stays OFF — said at boot); *(re-check #5)* **one of two strings in
  parallel, or one stage of a multi-stage lift** whose weight is a small share of the load (the current barely
  changes; HA-227 assumes ONE string — Step 0.1 asks how many). **What reads as SLACK though no string broke**
  *(re-check #5)*: a hold where something ELSE carries the carriage — a piece set on a goal, a field element under it, a
  robot pushing it up, or the bottom stop under a stretched string or a drifted zero above `offBottom` (the checker's
  V3, re-run byte for byte: a hold at 1.0 in with the carriage really resting at 1.3 in drew 0.062 A for 2.79–2.80 s,
  under the 1 V and 2 V loads' 0.104 / 0.249 A thresholds — a SLACK). The string IS slack there; it is not snapped —
  which is why D4 now recommends a response that does not cost the lift for the match.
- **(b)** Library default thresholds (a fraction of the gravity hold). **Cost:** an invented number on a safety
  feature; P8 shows a light lift's ON and UNHOOKED currents overlap — a default would false-trip there.
- **(c)** A watch in every phase with one threshold. **Cost:** P8's false trips in the deceleration.

**D4 — What a detected slack does: STOP, SAY, and forget the lift's height (UN-HOME it): auton steps on it fail, the
driver keeps it at its unhomed volts. RECOMMENDED (e)** *(re-check #6: the draft recommended (a), a latch until the
program restarts, and never weighed what a FALSE trip costs — D3's "reads as SLACK though no string broke" list)*.
- **(e) RECOMMENDED.** Everything (a) says up to the latch — the `SLACK` verdict, the one Warn line, `MECHANISM_SLACK`
  once, the safe state, the auton step FAILING, the drive carrying on — and then, instead of a latch, **the lift is
  UN-HOMED**: its zero is dropped (after a snap the motor's height is fiction — P1), so every later position step ends
  **NOT_HOMED** through the path that already exists (`service.hpp` :791-795; its note: "its string went slack at
  12.3 s: its height is unknown until it is homed again"), which stops an auton chain at its START exactly as REFUSED
  would (`routine.hpp` :955-965; STEP1's ST33 for a trigger's fire). **The driver's buttons still move it — at its
  `.unhomedVolts`**, the existing before-homing rule (`tickJog`, `service.hpp` :1043-1046: no soft stops, at most those
  volts), with the cap (firmware) and the stall watch still on and the slack watch off until it is homed again. It is
  re-armed by homing: a `homeBySwitch` lift by a HOME step (an auton's or a driver's home control); a `restsAtStop` lift
  cannot re-zero at run time (`service.hpp` :850-851) and stays driver-only at limited volts until the program
  restarts. **Why over (a):** on a REAL snap (e) costs nothing more — the carriage is on the floor, the driver sees it,
  and winding a broken string onto an unloaded spool at a few volts harms nothing; on a FALSE trip (a hold on a goal, a
  drifted zero — the checker's V3) (a) loses the lift for the rest of the match and (e) keeps it, by sight, at limited
  power. **Its condition:** `.unhomedVolts` defaults to 0 V (`kinds.hpp` :837), so a string lift must STATE one or its
  buttons do nothing after SLACK, as under (a) — the guide's string-lift section and the boot line say so
  ("'lift': .slack is declared but .unhomedVolts is 0: after a SLACK its buttons will do nothing").
- **(a)** *(the draft's recommendation)* The running step ends **`SLACK`** (a NEW verdict, appended — `manipulation/mechanism_outcome.hpp` :59-77 is
  append-only) with one Warn line naming the phase and the numbers — `'lift': SLACK -- the string went slack while
  cruising up: snapped, off its spool, unhooked, or its motor stopped answering (0.08 A at 45 rpm for 0.12 s; its slack
  signature is below 0.21 A)` — the fault **`MECHANISM_SLACK`** once (a NEW `FaultCode`, appended after
  `MotorGroupDisagree = 12`, `diag/fault.hpp` :78-104; on the CONTINUE side of the drive's abort mask, like
  `MECHANISM_STALLED`: the drive keeps going), the lift's declared safe state, and **a LATCH: every later step on that
  lift ends REFUSED** ("its string went slack at 12.3 s: every step is refused until the program restarts"), the
  driver's buttons on it do nothing, and the controller shows the fault's name (the driver panel already shows the last
  fault, `src/common/driver_panel.hpp` :624). A slack seen while HOLDING after a step (not a step) is reported the way a
  stall after a step is (`stalledAfterStep`, `service.hpp` :459) — `lastStep()` says SLACK, the command number bumps.
  **In an auton:** the step that saw it FAILS — today a mechanism step that is not SUCCEEDED stops the chain
  (`RoutineStopCause::MechanismStepFailed`, `chassis/routine.hpp` :943-950); when STEP1 lands, that step's on-failure
  choice (stop / skip to a named step / carry on) applies; a later step on the latched lift is REFUSED, which already
  stops a chain (`routine.hpp` :955-965) — **no change to `Routine`**. The robot's drive and other mechanisms carry on.
  Why latch: after a snap the motor's height is fiction (P1: the motor said 20.08 in, the carriage was on the floor at
  0.00 in) — every later command would wind the broken string on a spool that thinks it is somewhere else. **Cost**
  *(re-check #6)*: a FALSE trip (D3's list) costs the lift for the rest of the match, with no way back but a restart.
- **(b)** Report and carry on. **Cost:** an auton keeps scoring "with the lift up" that is on the floor, and the spool
  keeps winding.
- **(c)** Stop the whole auton. **Cost:** a robot that can still drive and score with its other mechanisms is parked.
- **(d)** Re-use STALLED. **Cost:** a student reading STALLED looks for a jam — the opposite fault.

**D5 — Homing a string lift: refuse a stall-home DOWNWARD; the cap protects a homing push UP; the homing signature must
be below the cap. RECOMMENDED (a).**
- **(a)** **Refused at build:** `homeByStall` pushing DOWN on a lift that declares `.slack` — measured (P5): on a
  gravity-return string lift the carriage stops on its bottom stop but the SPOOL does not; it unwound **492° past the
  bottom (6.4 in of string paid out)** and ended TIMED_OUT after 6.01 s ("it never slowed to its homing signature's
  speed"), where the same declaration on a rigid lift homed in 2.52 s. A string lift homes by `restsAtStop` (it rests on
  its bottom stop when the program starts — HA-226) or `homeBySwitch`. **A homing push UP into the top stop is protected
  by the cap** (P11: a 12 V homing push pulled 2.50 A uncapped and at most 1.00 A after contact under a 1.0 A cap, and
  still homed in 0.78 s) — and **refused when its homing signature is at or above the cap** (P11: it never homes, and
  pulls at the cap for the whole bound). The cap does NOT limit the carriage's momentum when it hits the stop
  (HA-222): the guide says push gently.
  *(re-check #4)* **A homing push DOWN by SWITCH on a `.slack` lift is GUARDED by the motor's own travel:** it ends
  TIMED_OUT, named ("its home switch never read pressed 1.0 in past where its bottom should be: unplugged, or its zero
  was off -- jog it down by sight and home again"), once the motor's height is 1 in below its home position (a lift that
  had a zero) or once the motor has turned 1 in of travel from where the push began (one that never had a zero; HA-226
  puts the carriage on its stop at program start, so a first homing push is rare). The draft offered switch homing as
  the safe one, but a switch that never reads pressed (a dead 3-wire switch reads a LEVEL, not an error — HA-184) leaves
  the push to its bound, and only the stall signature watches a switch-homing push (`service.hpp` :1110), which an
  unloaded spool never trips. Measured (`recheck/out/p_recheck.txt` R1; the checker's V2 at 1.2 V: 2.9 in): a −3 V push
  with a 4 s bound and a dead switch paid out **2.68 in (1 V load) and 3.64 in (2 V) from 6 in, and 7.02 in from the
  bottom**; with the guard, at most ~1 in plus whatever the zero was off. The 1 in is a library constant, INVENTED
  (HA-182's kind), said in the line. *(Considered and not chosen: stopping when the current falls to the unhooked level
  while turning — the carriage has landed. R1: it stopped 0.31–0.42 s after landing with 0.59–0.68 in paid out, but it
  needs a MEASURED hold threshold, and the descent itself dipped to 0.060 A at a 1 V load, under that load's 0.104 A
  threshold, for less than its 0.3 s — too close to be the guard. A "longer than the whole travel's time" bound was also
  measured useless: at −3 V the unloaded spool turns ~2 in/s, so 25 in takes longer than the 4 s bound.)* **A `spool()`
  lift with NO `.slack`** boots with an Info line ("'lift' winds on a spool: if it hangs on a string, declare .slack --
  chapter 18"), so a string lift that forgot the word is told it is unguarded.
  *(re-check #9)* **A string lift's acceleration must stay below gravity's.** The string pulls but never pushes: when
  an UP move slows faster than g (386 in/s²), the carriage keeps rising off the string and then drops back and SNATCHES
  it taut — a shock no current cap limits, exactly the snap the team lead named; a DOWN move that speeds up faster than
  g pays out slack the same way. Kinematics (`recheck/out/p_recheck.txt` R4 — no library, no friction; a blue cartridge
  on a 2 in spool, top speed 47.1 in/s): **at 0.5 g and 0.9 g no slack; at 1.25 g 0.58 in of slack and a snatch at
  21 in/s; at 2 g 1.44 in and 33 in/s**, up and down alike. The starting acceleration is the top speed in 0.25 s
  (`kinds.hpp` :605): 23.6 in/s² for a red cartridge on a 1.5 in spool, but 188 in/s² for a blue one on 2 in — four
  ×1.25 presses past g. So on a `.slack` lift: **the build REFUSES a stated `maxAcceleration` above g** (in the lift's
  units — physics, not an invented number), **the boot WARNS above 0.5 g** (the margin for friction and the hold loop's
  own corrections — invented), **the live page refuses above g and notes above 0.5 g**, and the starting value on a
  `.slack` lift is capped at 0.5 g. The jog's soft stop uses the same acceleration (`service.hpp` :759). A driver
  flipping the stick from full up to full down is outside it (the guide says so; a jog slew limit is MECH's rest, §5).
- **(b)** Warn, do not refuse. **Cost:** P5's paid-out string on the first homing.
- **(c)** Refuse every stall-home on a string lift. **Cost:** a lift with a hard stop at the TOP loses a homing method
  the cap makes safe.

**D6 — The "stronger" sensor: a sensor the CARRIAGE moves, not the spool's shaft — and not now. RECOMMENDED (a).**
- **What changed since 2026-10-29:** he was told "an output rotation sensor, optional and stronger (the lift's
  angle-sensor slot)". The lift's angle-sensor slot is "on the output shaft" (`kinds.hpp` :822-823) — on a spool lift,
  the SPOOL's shaft. **Probe P3: after a snap at 10 in, a sensor on the spool's shaft differed from the motor's height
  by 0.0000 in; a sensor the carriage moves differed by 20.11 in.** The spool keeps turning when the string breaks; only
  the carriage knows.
- **(a)** Current-only now (MECH-S-B). A **carriage-height check** — a V5 distance sensor aimed at the carriage (the
  library already reads distance sensors), compared with the motor's height: "they disagree by more than X in for Z s"
  sees a snap in every phase but the bottom, LOWERING included — becomes its own small run **MECH-S-C after the panel**
  (~1½–2, rough). It must NEVER be put in the angle-sensor slot: the position loop would then chase a fallen carriage and
  wind the spool (§8 landmine 5).
- **(b)** Build the carriage check now too. **Cost:** ~1½–2 more days in a lane that is already at the edge of Nov 21
  (§6.4).
- **(c)** The spool-shaft rotation sensor as told. **Rejected:** P3 — it cannot see a snap.

**D7 — Where live tuning runs: its OWN bench program, "shulib Tune", never a match program. RECOMMENDED (a).**
- **(a)** A sixth program, **"shulib Tune"** (`make ROBOT=… PROGRAM=tune`, **slot 7** — free since T4 Part 3 moved the
  Auton Tester to slot 2; `Makefile` :106-192: Drive 1, Auton Tester 2, Bench Tests 3, Calibrate 4, the old-code
  fallback 5, Competition 6). It builds **ONLY the robot's mechanisms** (`robot::MechanismParts` over the provider, the
  way a caller with no Robot may — `robot/mechanism_parts.hpp` :91-98): **no drive, no calibration record needed, and the
  drive cannot move**. It lives in the LIBRARY (a PROS-free program over `hal::IBrainScreen`, CAL1's pattern: the
  standard Calibrate lives in `calibration/` with a PROS shell in `hal/pros/calibrate.hpp`), so every robot — robot one,
  a robot made in the panel — gets the same Tune program, and G3a Part 4 can generate its build later. Its files touch
  **none of MECH-A6's** (`src/programs/bench_r3a.*`). The src build gate gains a check that the Competition, Drive and
  Auton Tester builds contain NO live-tuning code (their ELF's symbols, step 8's method). A match runs exactly the
  numbers in the robot file.
  *(re-check #10)* **Its build is ROBOT-GENERIC:** `PROGRAM=tune` is accepted for ANY `ROBOT` whose mechanism table is
  not empty (a robot with none is a `$(error)` naming why), never hard-gated on `ROBOT=tank` the way every program after
  `tester` is today (`Makefile` :169-189). The reason is the lift itself: **no robot file declares a string lift**
  (robot two's list is two Power claw pivots and a pneumatic claw — `src/robots/mechanism_table.hpp` :113-135), so the
  string lift is most likely ROBOT ONE's — and robot one has no build yet: its hand-written build plumbing is its own
  chunk (G3a's RC18 and D20's robot-one line: "launched by ~Nov 6 on the 11-23 track"), and G3a Part 4, which would
  generate it, is after the panel. With a generic row, the plumbing chunk gets Tune by registering robot one's mechanism
  table; with a tank-only row it would have to add a `PROGRAM=tune` branch of its own. **Until robot one builds, live
  tuning before Nov 21 can reach only robot two's claw pivots** (their maxVolts, stall signature and cap — a Power
  kind's live set).
  *(re-check #7)* **Its safety rules are the Auton Tester's, not only "any button"**: every mechanism is released to
  its safe state on ANY controller button, the controller DISCONNECTING, or VEXos reporting the robot DISABLED (the
  Auton Tester's abort causes — `src/programs/auton_abort_watch.hpp` :20-26, :50-56; the library's Calibrate already
  polls the competition status this way, `calibration/calibrate_program.hpp` :189-194 — no `src/` code is reused);
  TEST, HOME and STRING CHECK are REFUSED unless a controller is connected and VEXos reports no field or switch (the
  Auton Tester's RUN rule, `src/programs/auton_tester_program.cpp` :43-46); a **two-stage clear confirm** — stage 1
  before ANYTHING is made (making a 3-wire line drives it to its start level, HA-119: robot two's pneumatic claw moves
  at boot — MECH-A6's station 13 asks the same, `mech-a6`'s `bench_r3a.hpp` :244-247), stage 2 before a mechanism's
  first motion in the session; a TEST move's bound is the plan's own time at the live speed and acceleration, ×2, plus
  the settle time and 0.5 s, at most 10 s (a library rule — the report says TIMED_OUT when it runs out).
- **(b)** A page inside "shulib Drive". **Cost:** the driver's program grows a mode that changes the mechanisms'
  numbers — one stray tap during practice and the robot drives on numbers nobody wrote down; the Drive screen is the
  driver panel, already full.
- **(c)** A page of Bench Tests' station 13 MECHANISMS (MECH-A6, building in `mech-a6`: it re-grids the menu 3 × 5
  for it). **Cost:** station 13 runs NO mechanism service on purpose — it commands each mechanism directly behind six
  gates, so a direction can be checked before homing (`mech-a6`'s `src/programs/bench_r3a.hpp` :228-262); tuning needs
  the service (homed moves, the PIDs, the soft stop), so it would put a second way of driving the same motors into the
  station; it waits for A6 to merge and then edits a file A6 just wrote; and Bench Tests is this repository's own
  `src/` program (`Makefile` :74-75, :108), where a library program serves every robot, a panel-made one included.

**D8 — The Tune page: what it shows, what can change live, how a change is undone. RECOMMENDED (a).**
- **(a)** **On the brain screen** (480 × 240, the touch-panel budgets — HA-136): the mechanism (‹ › to pick one of the
  robot's), its state (homed?, where it is, its target, its last verdict), a list of its numbers with **[−] [+]** beside
  the selected one (gains ×/÷ 1.25 per press, FINE ×/÷ 1.05; volts ±0.05 V; amps ±0.05 A; speeds ±5 %; "reached"
  ±0.05 in or ±0.5°; settle ±0.05 s; a travel limit ±0.25 in or ±1°), **TEST A / TEST B** (move to one of two heights
  the page holds — two presets, or a quarter and three quarters of the range — and report the move: time, overshoot,
  settle, peak current, final error, verdict), **HOME**, **UNDO**, **RESET**, **COPY**, **STRING CHECK** (D3's bench
  page, string lifts only). **On the controller: ANY button STOPS every mechanism** (the Auton Tester's abort rule) — the
  screen is for numbers, the controller for safety; *(re-check #7)* so do the controller dropping and VEXos disabling
  the robot, nothing moves without a connected controller or with a field or switch plugged in, and nothing is built
  or moved before the clear confirm (D7 (a)).
  **What can change live** — position (Arm, Lift): the ten `.tuning` numbers (kP, kI, kD, kS, kV, kA, maxSpeed,
  maxAcceleration, within, settle), the gravity hold (kG), the stall signature, the homing push and its signature, the
  cap (D2), the slack signature (D3), and the travel limits **narrowed only** (never wider than the robot file's, never
  past a preset, *(re-check #7)* **and never past where the mechanism is HOLDING now** — "it holds at 20.00 in, above
  the new top 12.00 in: move it inside first"; the draft re-clamped such a hold with `holdAt`, `service.hpp`
  :1152-1161, a jump to a new target with no smooth move: 8 in × 76.4 °/in × kP 0.25 asks 153 V, so the motor drives
  down at full 12 V); speed (Flywheel): kS, kV, kP, kI, the at-speed band and time, the stall signature, the cap
  *(re-check #14: the draft listed a flywheel kA — `SpeedTuning` has only kS, kV, kP, kI, `kinds.hpp` :429-438, so the
  robot file cannot say one, and by this page's own rule it is not live)*; power: maxVolts, the stall signature, the
  cap. *(re-check #14)* **A number that starts at 0** (an Arm's or a Lift's kA — `positionStart`, `kinds.hpp` :603)
  cannot leave 0 by ×1.25: its first [+] sets kV × 0.01 s (a 10 ms lag's feedforward), then ×/÷ 1.25; its [−] at that
  value goes back to 0 (which the file CAN say: kA's starting value is 0). kS is in volts (±0.05 V) and needs no rule.
  **Not live:** presets (a goal finds its preset by VALUE — P7: change "high"
  from 22 to 21 in and `find(lift["high"])` answers nobody, so a step on it would end ABSENT); instead the page shows
  where the lift is, so a student jogs it to the right height and reads the number off. The drive's numbers (Part T's
  page shows R5a's measured ones) and pneumatics: not live (§5).
  **The rules:** a change is applied only **between steps** (a test move running greys every button but STOP), **only
  if the changed declaration passes the BUILD's own check** (`specRefusal`) **plus the live bounds the build lacks**
  (P9: today the build accepts a NaN kP, a negative kS or kA, kV = 1e9 and a 500 in/s top speed on a lift whose motor
  turns at most 7.85 in/s — the page refuses each, saying why), and **never to a value the robot file cannot say**: a
  `.tuning` number of 0 means "the starting value" (P9: `.kD = 0` builds as 0.004), so the page never sets kP, kI, kD,
  kV, maxSpeed, maxAcceleration, within or settle to exactly 0. **How a change is undone:** UNDO (the last change, up to
  16 back), RESET (this mechanism back to the robot file's numbers, bit for bit), and **nothing persists** — leaving the
  program or a reboot runs the robot file's numbers again; every program but Tune always does.
- **(b)** The controller's buttons for the numbers too. **Cost:** they would collide with "any button stops".
- **(c)** Live presets (find goals by name, move to the live value). **Cost:** a change to the lookup every auton uses,
  for a number a student can read off the page.

**D9 — How the numbers reach the panel: a COPY block in the log (serial AND the SD card), imported into the ROBOT FILE.
RECOMMENDED (a).**
- **(a)** **COPY** prints, for each mechanism changed, ONE block of `[TUNE]` lines — *(re-check #14: the draft wrote
  EVERY effective number, which turns derived starting values into stated ones: a later cartridge or gearing change
  would keep the old kV and top speed, and a top speed above the new free speed still builds — P9)* **the SMALLEST set
  of stated numbers whose declaration builds to exactly the live spec**: every number the robot file states, every
  number changed live, and every DERIVED number whose re-derivation from those would differ from the live value (found
  by running the file's own derivation, `positionStart`, on the candidate and comparing field by field — so a changed
  top speed carries the acceleration it was tuned with, §8 landmine 12, and an untouched kV stays derived). Each line
  names its member as the declaration does (`.tuning.kP = 0.312`), within the log's 200 bytes, the block closed by an
  end line carrying its count — into the program's log: the serial terminal AND the Tune program's own copy on the SD
  card (`/usd/tune_log.txt`, *(re-check #14)* **APPENDED across boots** — the Competition program's mode, for the same
  reason: a reboot back into Tune must not erase the last session's COPY; `src/common/log_copy_line.hpp` :7-10, the
  provider's `ProsFileMode`, `hal/pros/device_provider.hpp` :220-231 at `9d77c9d`; the draft said "a new file each
  boot"). **The cost the team carries:** a number stated by a COPY stays as it was tuned if the cartridge, gearing or
  spool changes later — the guide says to RESET and re-tune after such a change. On the laptop, *(re-check #1: AFTER
  THE PANEL — TUNE1-B2 needs G3a Part 5, §2.1)* the panel's MECHANISMS page has **IMPORT TUNING** (a button over
  `python3 tools/robot_tool.py import-tuning FILE` — the subcommand G3a's amendment reserved for this chunk,
  `G3a-robot-profile-generator-panel.md` :1138-1145): it takes the LAST complete block per mechanism from a saved
  terminal log or the card's file, refuses a cut one, and writes the numbers into the ROBOT FILE through the tool's one
  writer, each with its provenance ("tuned live on the brain, imported <date> from <file>"); then `generate` and a
  rebuild. **The robot file stays the ONE source** of what every match program runs (MECH-PANEL §5 Q24's option (b)).
  **Until the import lands (after the panel), a student copies the block's lines BY HAND into the robot's hand-written
  mechanism table** (robot two's `src/robots/mechanism_table.hpp`; robot one's when its plumbing exists) — the lines
  name each member the way the declaration writes it, so the copy is mechanical; the provenance is a comment the
  student writes ("tuned live, <date>").
- **(b)** An SD tuning store every program reads at boot (Q24 (a), R5a's pattern for drive gains). **Cost:** what the
  competition program runs depends on which card is in the brain, not on the reviewed file; two stores to compare; and
  he asked for the numbers "in the panel".
- **(c)** Copy by hand only. **Cost:** typos and no provenance — the fallback of (a), not a plan.

**D10 — Six runs, their order, and what lands by Nov 21 — RE-DERIVED on the team lead's order of 2026-11-03.
RECOMMENDED (a).** *(re-check #1: the draft dated everything on G3a's D20 (a) of Oct 30 — the string chunks fourth
behind the route maker, one lane from ~Nov 8 — which he REPLACED on Nov 3, ~13:45: "EVERYTHING INCL. FLIPPER before the
21st; panel AFTER"; the order competition in flight → AF1 brief, VIS1-A2, ALLY1(+L4) brief, VIS1-B, AF1 build (by
~Nov 8) → L6-A2/B, STEP1, MECH-S, CAL1-B → CURVE1 → panel; G3a Part 5 deferred. The draft also put its own S-B low end
at Nov 21 where its arithmetic gave ~Nov 22. Re-derived below from today's date, so the four days of the pause are in.)*
- **Two consequences of the new order, not choices:** (1) **the copy-back (TUNE1-B2) lands AFTER the panel** — the
  robot tool refuses `mechanisms` BY NAME until G3a Part 5 adopts robot two's mechanism regions (`g3a-p1`'s
  `tools/robot_tool.py` :29-30, :244-253), and the IMPORT TUNING button is Part 6's page; until then COPY's lines are
  copied by hand (D9 (a)). (2) **G3a Part 5 ADOPTS `.currentCap` and `.slack`** when it builds the mechanism vocabulary
  — G3a's amendment §6: "If they land BEFORE Part 5's tip, Part 5 adopts them as landed (+¼–½ per member set)"
  (`docs/internal/chunks/G3a-robot-profile-generator-panel.md` :1131-1132): MECH-S-A and MECH-S-B land before it, so
  neither extends `tools/robot_tool.py` — the draft's "rule 6 (i), likely" and its +¼–½ shares here are gone, and Part
  5 carries ~+½–1 for the two member sets (and the live-tuning provenance comment form) instead.
- **(a) RECOMMENDED order:** **MECH-S-A** (the cap) → **TUNE1-A** (live tuning's library) → **TUNE1-B1** (the Tune
  program) and, BESIDE it if a second build slot is free, **MECH-S-B1** (the slack watch: the windows, the refusals,
  the homing guard, D4's un-homing, the string carriage fixture, the bench's pure model) → **MECH-S-B2** (the STRING
  CHECK page inside the Tune program, after TUNE1-B1's build) — and **TUNE1-B2** after the panel. *(re-check #1: the
  draft's single MECH-S-B is SPLIT so the watch, which needs no Tune file, can build beside the Tune program — the only
  way it can reach Nov 21; one lane keeps it whole.)* Why the slack watch after live tuning: its thresholds can only be
  measured on the STRING CHECK page, which lives in the Tune program — landed alone, it would sit OFF
  (`notMeasuredYet`) on every robot; and the cap PREVENTS a snap while the watch only reports one.
  **Dates** (rough sizes from the header; each build starts when the previous BUILD ends; G3a's model: a review round
  2 days, a merge ½ day; **MECH-S-A's build starting ~Nov 8** — the earliest the 11-03 order gives MECH-S a slot, after
  the AF1 build starts): **the cap ~Nov 13–14**; **live tuning's library ~Nov 15–17**; **live tuning on the robot (the
  Tune program) ~Nov 19–21**; **the slack watch ~Nov 20–23 with the second slot** (S-B1 ~Nov 18–20, S-B2 ~Nov 20–23),
  **~Nov 22–26 in one lane**; **the copy-back after the panel**. If a slot frees for MECH-S-A by ~Nov 5, every date
  moves ~3 days earlier (the slack watch ~Nov 17–20 with two lanes). **So: the cap before the freeze; live tuning just
  before Nov 21; the slack watch by Nov 21 only with the second slot and an early start; the copy-back after the panel
  by construction.**
  **What the freeze (~Nov 16) means here:** the cap lands before it and changes a match build only for a mechanism that
  STATES one (robot two states none) — plus, under D12 (b), a read of each mechanism motor's limit at boot (an Info line
  only if one reads other than 2500 mA; nothing written); live tuning's library (~Nov 15–17) adds code no match program
  calls (TUNE1-B1's gate check proves the match ELFs carry none); the Tune program is a bench build; **the slack watch
  lands after the freeze** and reaches a match build only on your word — robot two declares no `.slack`, so its match
  build would not change; a string lift competing on Nov 23 without it is protected by the cap alone.
  **The shared file:** ~~`mechanism/service.hpp` is also edited by STEP1-B and by the AF1 build in the same weeks
  (§6.2). If the coordinator kept the draft's "never two builders in it" across chunks, MECH-S's three `service.hpp`
  runs would queue behind AF1's ~8¾–11½-day build …~~ *(re-check C1: AF1's own draft brief says "**AF1 does not edit
  it**" — `AF1-toggle-auto-flipper.md` §6.2 :747, its §7 diff empty; the ~14:00 fix took AF1's one-line description in
  `build-order.md` instead. AF1-A shares `mechanism/kinds.hpp` — its `Arm::pieceSensor` — not the service.)*
  `mechanism/service.hpp` is also edited by **STEP1-B** in the same weeks (its D8 (a), accepted ~14:30: the four start
  commands' precondition blocks extracted into check functions — "same checks, same order, same messages", STEP1's
  brief :879), in regions none of MECH-S's runs is planned to touch (§6.2; S-B1's switch-homing guard records where
  the push began inside `tickHome`, not in `home()`'s checks). A strict "never two builders in it" across chunks
  would hold whichever MECH-S run overlaps STEP1-B's build until it ends — up to its ~3–4 builder-days on every date
  after it, enough to push the cap toward the freeze and live tuning past Nov 21; this brief therefore names each run's
  REGION of the file and asks for the house rule instead — whoever lands second re-reads, the merged build and the
  full suite are the gate (VIS1-A1's lesson).
- **(b)** One lane, MECH-S-B whole. **Cost:** the slack watch ~Nov 22–26 — after Nov 21 even at the low end.
- **(c)** The slack watch BEFORE live tuning (the order of your three words). **Cost:** it lands ~Nov 17–19 with no page
  to measure it — OFF on every robot until the STRING CHECK page exists (~Nov 21–24); live tuning ~Nov 21–25.
- **Not an option:** the copy-back before the panel — it needs Part 5, which you deferred.
- **What I need from you** *(re-check #1, #5, #10)*: **(Q1) does "MECH-S" in today's order include TUNE1 — live
  tuning?** On Oct 29 you ruled all three in; today's list names MECH-S only. This brief assumes yes. **(Q2) which robot
  carries the string lift, and when does its first build exist?** (No robot file declares one; if it is robot one, its
  hand-written build plumbing — G3a's RC18, due ~Nov 6 — must exist before live tuning can reach it, D7 (a).) **(Q3) the
  lift itself:** pulled UP by string and lowered by GRAVITY (the design's assumption — a string loop driven both ways is
  OUT, §5)? How many strings in parallel, how many stages (one string's snap in two may not show — D3)? The string's
  material and rated strength (the cap's ceiling, HA-222)?

**D11 — Hardware Assumptions HA-219..HA-227, reserved AFTER STEP1's HA-216..HA-218. RECOMMENDED: accept.**
Nine beliefs (§4) — none measured; each with the bench session that settles it. ~~If STEP1's reservation changes before
this brief's plan commit, these shift by a mechanical rename (IMU1's precedent).~~ *(re-check C3: STEP1's HA-216..HA-218
were reserved with the team lead's "Accept all, go" at ~14:30 — no longer open. AF1's draft brief reserves the NEXT
block, HA-228..HA-236, AFTER this one (its D12, :263) — so if this block ever changed, AF1's would move with it: the
coordinator renames both at once, IMU1's precedent.)*

**D12 — A motor with NO cap whose limit reads something other than 2500 mA at boot: reset it, or only report it?
RECOMMENDED (b) until Step 0.4 measures.** *(re-check #8, NEW: the draft decided (a) silently and said "robot two's
boot does not change" — true only if every limit reads 2500 mA, which is HA-220's and HA-221's open question.)*
- **(a)** *(the draft's)* Every boot writes 2500 mA back, with a Warn ("a cap left by another program: reset to
  2.50 A"). **Cost:** HA-172 (`docs/hardware-assumptions.md` :391 at `9d77c9d`) and MECH-PANEL §3.2
  (`docs/internal/MECH-PANEL-requirements.md` :247-249) hold open whether VEXos lowers each motor's limit when more
  than 8 motors are plugged in; robot two runs 10. If the read-back shows that share, EVERY Competition boot writes
  2.5 A over it, with a Warn that names the wrong cause — a change to the match program on an unmeasured belief, days
  before the freeze.
- **(b) RECOMMENDED until measured:** a mechanism with no cap READS each motor's limit at boot and writes nothing; one
  that reads other than 2500 mA gets one Info line naming both possible causes ("'claw pivot 6': its motor's limit
  reads 1.80 A: a cap left by the Tune program, or VEXos's sharing with 10 motors (HA-221) -- nothing written"). The
  Tune program restores every limit IT changed to the robot file's value (2500 mA for none) on RESET, on leaving that
  mechanism's page and on its EXIT, and its screen says that a limit set there can stay in the motor if the program is
  stopped another way (HA-220). Robot two's boot then changes ONLY if a claw motor reads other than 2500 mA — and then
  only by that Info line. **Cost:** a Tune limit left by a program that was stopped from the brain rides into the next
  program until a power cycle — shown at its boot, not undone. **The programs a Tune limit can reach that never read
  one:** Bench Tests' station 13 (MECH-A6 — it commands the mechanisms directly and binds no service) and slot 5's
  old-code fallback; and the drive motors are never touched (Tune builds only mechanisms).
- **(c)** Reset only in the Tune program's own exit paths, never at a match program's boot. **Cost:** (b)'s gap,
  without (b)'s boot report.
- **How it is settled:** Step 0.4 (§2.0) — robot two's boot read-back with all 10 motors plugged in, right after
  MECH-S-A lands (HA-221); then, once the Tune program exists, a limit set in Tune, the program stopped from the brain,
  and shulib Drive's boot read-back (HA-220). If every motor reads 2500 mA with 10 plugged in, (a) is safe and the
  coordinator brings it back to you as a one-line follow-up.

---

## The design in plain words (for the team lead)

**The motor's current IS the string's pull.** A V5 motor turns its current into torque, and the spool turns the
torque into string tension — so "how many amps" is "how hard the string is pulled" (about 10 lbf per amp per motor for a
red cartridge on a 1.5 in spool, by the published numbers — HA-222, not measured). Three things follow.

**1. The cap (MECH-S-A).** The team writes `.currentCap = {.amps = 1.2}` on the lift. At boot the library tells each of
the lift's motors "never draw more than 1.2 A", checks the motor took it, and says so in one boot line. From then on the
MOTOR enforces it, by itself — even when the program is stuck in a student's loop, even during a homing push into the
top stop. A stall signature (the jam detector) must be set below the cap, or it could never
fire; the build refuses one that is not. No number is invented: no cap written means the motor's own 2.5 A.
*(re-check #3)* And the jam detector must sit ABOVE the heaviest hold — with a game piece — or holding that piece
reads as a jam: four currents in order, hold < jam detector < cap (with room) and the heaviest move's peak < cap. The
Tune program's test report shows the hold and the peak, and warns when the order breaks. *(re-check #8)* A motor
whose limit reads something other than 2.5 A at boot is only reported until the bench shows what a 10-motor robot
reads (D12).

**2. The slack watch (MECH-S-B).** When the string snaps, the motor suddenly has nothing to lift: its current drops while
it keeps turning. The library watches for that in the two moments it can be told apart — while the lift CRUISES UP at a
steady speed, and while it HOLDS off the bottom (a driver's push held at the top counts as a hold). A guided bench page
(STRING CHECK) runs the lift three times with the string on and three times unhooked, and sets each threshold halfway
between the two — or says plainly that this lift's current cannot tell them apart by a clear margin (too light, too
noisy) and leaves that half of the watch off. Two of its numbers (how long a dip must last) are starting values, said
as such. *(re-check #6)* When the watch trips: the lift stops, the log and the controller say SLACK, the auton step
fails, and the lift FORGETS its height — auton steps on it fail until it is homed again, while the driver can still
move it slowly, by sight. (A hold where something else carries the carriage — a goal, the floor under a stretched
string — reads as slack too; that is why it does not lock the lift for the rest of the match.) It cannot see a snap
while the lift is coming DOWN (gravity does the work either way), at the bottom, or of one string out of two; it sees a
snap at the next hold or the next climb. *(re-check #4, #9)* Two more guards: a homing push down onto a switch that
never clicks stops after about an inch instead of unwinding string, and a string lift may not speed up or slow down
faster than gravity (the carriage would leave the string and snatch it).

**3. Live tuning (TUNE1).** A new bench program, "shulib Tune", builds only the mechanisms — the drive cannot move. Pick
a mechanism, change a number with − / +, press TEST A or TEST B to watch the lift move, read how long it took, how far it
overshot and the most current it drew. UNDO and RESET put numbers back; nothing is kept after the program ends. A change
that the build itself would refuse — or one the robot file cannot even say — is refused on the spot, with the reason.
When it is right, COPY writes the numbers into the log (and onto the SD card); *(re-check #1)* until the panel lands a
student copies them into the robot's mechanism table by hand; after it, the panel's IMPORT TUNING puts them into the
robot file, marked "tuned live on the brain"; the next build runs them. Match programs never tune. *(re-check #7)*
Nothing moves without a connected controller, any button or a dropped controller stops everything, and the screen asks
"hands clear?" before anything is switched on.

**Three things the probes found that change what was said on 2026-10-29:**
- **Today, a snapped string is invisible:** the library reported the lift's move SUCCEEDED with the motor saying 20 in
  and the carriage on the floor (P1).
- **A rotation sensor on the spool's shaft cannot see a snap** (P3) — only a sensor the carriage moves can (D6).
- **A string lift must not home by pushing DOWN until it stalls** — the spool unwinds past the bottom (P5); and live
  tuning cannot be "edit the numbers and go" — half of them are copied at boot and would silently not change (P6).

---

## 0. Why — what exists, what the motor and the string give, and the probes (every line cited was OPENED at `eab0f89`; VIS1-A1's files at `9d77c9d` — header)

### 0.1 Why these chunks, and why now
The team's string lift (a carriage on a string wound onto a spool, lowered by gravity) breaks if pulled too hard and
cannot be tuned on the robot today without a rebuild per change. The team lead ruled all three pieces in before the panel
lands (header). The MECH core (A1–A5 landed, A6 building) runs a Lift completely — shape, gains, range with closed-loop
soft stops, gravity, the required stall signature, homing — but has **no current cap, no slack watch, and no way to change
a number on the robot**.

### 0.2 What exists in the tree
1. **The Lift** (`include/shulib/mechanism/kinds.hpp` :811-913): `.shape = spool(diameter)` ("A string or chain on a spool
   … on the output shaft", :373-376), `.stall` REQUIRED (:826-827), `.range`, `.gravity = constantGravity(…)`, `.home`,
   `.presets`, `.unhomedVolts`, `.tuning` (`PositionTuning`, :403-427 — ten numbers, **0 keeps the starting value**,
   `positionStart` :592-608). New members can sit anywhere in the declaration order without breaking a declaration that
   does not name them (designated initializers may skip members; only NAMED members must follow the order — :26-30).
2. **The spec** (`mechanism/spec.hpp`): `StallSpec` (:184-203), `PositionSpec` (:214-284), `MechanismSpec` (:364-412);
   the refusals, APPENDED ONLY ("G3a's validator mirrors this list by sentence", :479-480), and `specRefusal` (:1106-1423,
   "The ORDER is pinned by a test"); every written stall signature below 2.5 A (:1177-1192 — "a limited motor never reads
   more"). **It does not police gains** (P9).
3. **The service** (`mechanism/service.hpp`): one ticker (:6-20); the stall watch on every behaviour (:85-90,
   `stallTripped` :982-984); a position move's profile, PIDs, soft stop, hold and homing (:727-1297). **What it copies at
   bind** (the slot's constructor): the PID gains (:735-741), the stall and homing-stall detectors (:743-752 — the
   detector's config is COPIED, `manipulation/stall_detector.hpp` :71-110, "re-tuning means constructing a new
   detector"), the travel limits in degrees (:753-756) and the acceleration the jog's soft stop uses (:759). **What it
   reads every tick:** the feedforward and gravity (:1173-1190). `moveTo` reads the limits and the profile's speed from the
   spec at each move (:796, :812). `Slot::spec_` points at the robot's list's entry (:475). `find(const PositionGoal&)`
   matches a preset by NAME AND VALUE (:1783-1791). The honest limit: a starved service leaves a motor at its last
   command (:104-113). A stall after a step: `stalledAfterStep` (:459-464).
4. **The motor seam** — `hal::IMotor` (`hal/motor.hpp` :68-121), **F4, LOCKED** (`docs/roadmap.md` :138): voltage in;
   position, velocity, current, temperature out; nothing about a current limit. `hal::MotorMechanism::maxCurrent()` — the
   most loaded motor, by magnitude (`hal/mechanism.hpp` :256-266); `applySafeState()` (:240-249).
   **PROS:** `motor_set_current_limit(port, mA)` (`include/pros/motors.h` :1146; "The motor will reduce its output at
   1000 mA instead of the default 2500 mA", :1137-1140), `motor_get_current_limit` ("The default value is 2500 mA",
   :1255); `pros::Motor::set_current_limit` / `get_current_limit` (`include/pros/motors.hpp` :1237, :974). The host shim's
   `pros::Motor` (`test/pros_shim/pros/motors.hpp`) has `get_current_draw` (:244) and no limit calls.
   `ProsMotor` (`hal/pros/motor.hpp` :136ff) reads `get_current_draw` (:260).
5. **The device provider** (`hal/device_provider.hpp` :132-248): every implementer answers every accessor — three today:
   `ProsDeviceProvider`, `FakeDeviceProvider`, `test/calibrate_program_rig.hpp`'s `CountingProvider`. VIS1-A1 added the
   pure `aiVision()` and the merged tree did not build until `CountingProvider` answered it (NOTES 10-30 ~16:00 — the
   lesson for MECH-S-A, §8 landmine 14). The F4-additive sibling precedent: `hal::IReadFaults` (`hal/read_faults.hpp`
   :1-60) with the provider's `readFaults(port)` lookup (:216-221; `ProsDeviceProvider` :571-587 — distance, optical,
   camera; **not motors**).
6. **The host test plant** (`sim/mechanism_plant.hpp`): a joint with a gravity LOAD in volts, hard stops that zero the
   speed IN ONE STEP (:228-251), the brake modes at 0 V; **current = |V − kV·ω|·2.5/12, clamped at 2.5 A as a READING**
   (:32-34, :272) — no limit on torque; `addLoadVolts` (:139) — how P1 removes the carriage's weight at a snap. Every
   constant INVENTED (HA-183, `docs/hardware-assumptions.md` :401).
7. **The verdicts and faults** — `MechanismOutcome` (`manipulation/mechanism_outcome.hpp` :59-77, append-only: Absent,
   NotHomed, Refused appended by MECH-A2); `FaultCode` (`diag/fault.hpp` :58-104, append-only; the latch's per-code tally
   holds 32 codes, :223-226); `test/driver_panel_test.cpp` :57-63 lists EVERY code (a new one joins it).
8. **The Routine** treats any verdict but SUCCEEDED (or UNCONFIRMED counted done) as a failed step
   (`chassis/routine.hpp` :943-950) and a START that is ABSENT, REFUSED or NOT_HOMED as a stop (:955-965).
9. **The programs:** five builds per robot two plus Bench Tests (`Makefile` :106-192; `src/build_selection.hpp` :80-150,
   THE ONE CHAIN, read by `tools/src_build_gate.py`'s step 0); each program binds ONE `MechanismService` over the
   CONSTANT list `kMechanisms` (`src/programs/drive_program.cpp` :658-660) — `MechanismTable` is a plain struct of arrays
   (`mechanism/wiring.hpp` :115-147), so a program can COPY it into its own mutable storage and bind over the copy.
   The brain screen seam `hal::IBrainScreen` (`hal/brain_screen.hpp` :109, CAL1-A1) with `FakeBrainScreen`; the touch-panel
   budgets (`ui/touch_panel.hpp`, HA-136). "shulib Drive" keeps an SD copy of its log through the provider
   (`hal/pros/device_provider.hpp` :122ff, `src/common/log_copy_line.hpp` :7-10). TermSink cuts a message at 200 bytes
   (`diag/term_sink.hpp` :285).
10. **The plan:** MECH-PANEL §3.1 (:174-182 — the two ways to cap), §3.2 (:245-250 — "Each mechanism gets a current
    cap… An HA is to be registered" for the >8-motor sharing), §5 Q15 (:649-652), Q24 (:696-699), §6 (:732); G3a's
    amendment §6 (:1129-1145 — where the string vocabulary and the copy-back plug in), the rebase rule (:1792-1818, point
    6), D20 (:1247-1330).

### 0.3 What the motor, PROS and the string give (read 2026-11-03)
- **The limit is the MOTOR's**: PROS documents it as a per-motor setting with a 2500 mA default; how fast the motor
  enforces it, whether it survives into the next program, and whether VEXos lowers it further with many motors plugged in
  are not in the vendored text (HA-219, HA-220, HA-221; HA-98 :326 and HA-172 :391 already hold the latter two questions
  open for brake modes and the drive).
- **A capped motor's reading** sits at or just under its cap when stalled — how close is unmeasured (HA-223); a signature
  a little below the cap trips on the host test plant (P10: 1.15 A under 1.2 A), which proves only the algebra.
- **The string:** its pull = the motor's torque through the gearing over the spool's radius — plus the rotor's and the
  carriage's momentum when it hits a stop, which no current cap limits (HA-222).
- **The current's noise:** unmeasured. P12 shows what it decides: whether a given lift's string-on and unhooked currents
  can be told apart at all (HA-224).

### 0.4 The probes (`.claude/coordinator/scratch/brief-MECHS-TUNE1/`, built by `build.sh` against the main checkout; every binary under `ulimit -v 4 GiB` + `timeout 300`)
All on the host test plant through main's REAL `MechanismService` and the position tests' rig
(`test/mechanism_position_rig.hpp`); a 1.5 in spool, a red cartridge direct (free 7.85 in/s), range 0–24 in, the
controller's kG 1.0 V; the plant's own gravity varied. Outputs in `out/`.
- **P1** (`p_string`): a string snaps at 10.04 in during a move 0 → 20 in. **Verdict SUCCEEDED in 3.76 s; the motor's
  height 20.08 in; the carriage's TRUE height 0.00 in; 2 s later it holds at 0.062 A; two MECH lines, zero faults.**
- **P2:** the current by phase, string ON vs SNAPPED, plant gravity 0.5/1/2/3 V (controller 1 V): cruising up ON
  0.125–0.300 / 0.229–0.408 / 0.440–0.609 / 0.688–0.776 A vs SNAPPED 0.021–0.193 A; holding ON 0.042 / 0.146 / 0.474 /
  0.688 A vs SNAPPED 0.062 A; lowering ON 0.000–0.099 … 0.438–0.603 vs SNAPPED 0.020–0.207 A; decelerating ON down to
  0.006–0.081 A.
- **P3:** after a snap at 10 in, a sensor on the SPOOL's shaft vs the motor's height: worst 0.0000 in; a sensor the
  CARRIAGE moves: 20.11 in.
- **P4** (superseded by P10 for the cap; kept): a move into an obstacle; a stall signature at/above a cap → TIMED_OUT, no
  stall fault.
- **P5:** `homeByStall` DOWN at −3 V on a gravity-return string lift: **TIMED_OUT in 6.01 s, the spool 492° past the
  carriage's bottom = 6.4 in of string paid out**; the same declaration on the rigid plant: SUCCEEDED 2.52 s.
- **P6 — live tuning TODAY**, the slot bound over a MUTABLE spec copy: kP × 4 in the spec changes the hold's command by
  **+0.0000 V** (the PIDs copied it at bind) and the NEXT move by nothing (1.050 s both); kG + 1 V changes it by
  **+1.0000 V** (read each tick); halving maxVelocity 0.5 s into a move changes nothing (3.76 s both) but the next move
  (6.36 vs 3.42 s); narrowing max to 12 in clamps a computed move (12.02 in) but **a 6 V jog for 3 s ends at 20.71 in**
  (the jog's soft stop uses the limits converted at bind).
- **P7:** a preset's value changed 22 → 21 in: `find(lift["high"])` → nullptr (a step on it would end ABSENT).
- **P8:** a slack watch (probe-local), thresholds the bench way (midway between ON and SNAPPED cruise/hold): at 0.5 V
  gravity NOT separable (false trips); a "naive" watch in every phase FALSE-TRIPS in the deceleration at 0.5–2 V; the
  phase-aware watch at 1–3 V: **no trip on intact three-leg cycles; a snap cruising up seen 0.12–0.14 s later, holding
  0.34 s, lowering 2.18–2.20 s later (at the next hold)**.
- **P9** (`p_tuning_rules`): `.tuning = {.kP = −0.3, .kD = 0, .kS = −0.5, .kA = −0.01, .maxSpeed = 500}` builds as kP
  0.25 and kD 0.004 (0 or less = the starting value), kS −0.5, kA −0.01, maxVelocity 500 in/s (free 7.85) — **specRefusal
  accepts it**; it also accepts kP NaN, kI −1, kV 1e9, maxVelocity 1e6; it refuses maxVelocity 0, tolerance 0, settle
  −0.1, kG 13, stall 2.6 A and a preset outside a narrowed range.
- **P10** (`p_cap`, NEW): a move up into an obstacle at 15 in, with no cap / a "software" cap (once per 10-ms tick, from
  the speed the service read) / a "firmware" cap (re-clamped every 1 ms against the joint's speed at that moment, and
  while the service is starved). **Pushing after the impact: 2.50 A with no cap, at most 1.20 A under either 1.2 A cap**;
  a stall signature 1.0 or 1.15 A (below the cap) → STALLED at 3.22 s, exactly as uncapped; 2.0 A (above) → TIMED_OUT at
  4.02 s after 1.33 s at the cap, no fault. **STARVED** (a 12 V push, nothing ticking for 1.5 s): no cap 2.50 A for
  1.03 s; software 2.50 A; **firmware 1.20 A**. **The impact sample itself (2.19 A in both cap models) is the plant's
  instant stop** — no cap model can be judged at an impact on this plant (§8 landmine 4).
- **P11** (`p_cap`): `homeByStall` UP into the top stop: +6 V uncapped homes in 1.52 s at 1.25 A; +12 V uncapped 0.74 s at
  2.50 A; **+12 V under a 1.0 A cap homes in 0.78 s pulling at most 1.00 A after contact**; a 1.2 A homing signature
  under a 1.0 A cap never homes (TIMED_OUT 4.02 s, 3.34 s at the cap). (+3 V never reached the top in the 4 s bound — a
  slow push, not a finding.)
- **P12** (`p_slack_noise`, NEW): the P8 watch with Gaussian noise (σ 0/0.02/0.05/0.10 A, fixed seeds) on every sample the
  bench and the watch see; the bench rule "midway between the ON run's LOWEST and the UNHOOKED run's HIGHEST, refused
  where they overlap". **With acceleration samples included, the cruise phase never separates at 1 V** (UNHOOKED 0.420 vs
  ON 0.229). **Cruise only:** at 1 V separable to σ 0.02 (hold to σ 0); at 2 V to σ 0.05; at σ 0.10 neither. Wherever a
  threshold was set: **0 false trips in 20 intact cycles**; cruising-up snaps seen 0.11–0.16 s later, holding 0.34–0.37 s,
  lowering 2.18–2.19 s (at the next hold); *(re-check #12)* where the hold half was refused (1 V, σ 0.02): lowering
  3.48–3.50 s and holding 5.70–5.71 s — both seen only at the next climb (the draft gave the 5.7 s to lowering).
- *(re-check, NEW)* **R1, R3, R4** (`recheck/p_recheck.cpp`, outputs `recheck/out/`; the checker's `check/p_verify.cpp`
  and `check/p_plantlimit.cpp` rebuilt at the current tree and re-run — byte-identical to the checker's outputs):
  **R1** a switch-homing push DOWN at −3 V, bound 4 s, the switch dead: TIMED_OUT, **2.68 in (1 V) / 3.64 in (2 V) of
  string paid out from 6 in, 7.02 in from the bottom**; a "landed" current stop would end it 0.31–0.42 s after landing
  (0.59–0.68 in), but the 1 V descent dipped to 0.060 A under its 0.104 A threshold; a whole-travel bound never acts
  inside 4 s. **R3** a +6 V push held at the top soft stop: 0.146 / 0.354 A whole (1 V / 2 V), 0.043 / 0.062 A after a
  snap, speed ~0. **R4** a one-sided string (kinematics only): no slack at 0.5 g or 0.9 g; 0.58 in and a 21 in/s snatch
  at 1.25 g; 1.44 in and 33 in/s at 2 g — up and down alike. **V1** (the checker's) a 1.0 A signature under a 1.2 A cap:
  a piece raising the hold to 1.08 A → STALLED while holding. **V3** a hold at 1.0 in on a carriage really resting at
  1.3 in → 2.79–2.80 s at the unhooked level (a false SLACK). **p_plantlimit** a 12 V jog reversed to −12 V drives the
  plant past 2.5 A's torque on 2 ticks (|V − kV·ω| up to 18.26 V).

---

## 1. Scope — IN (every new and changed file, by run)

### 1.1 MECH-S-A — the current cap (D1 (a), D2 (a), D5's homing-signature half)
- **NEW `include/shulib/hal/motor_current_limit.hpp`** — `hal::IMotorCurrentLimit`, an **F4-ADDITIVE SIBLING** (the
  header says so in `IReadFaults`'s words; `IMotor` untouched byte for byte): `[[nodiscard]] bool setCurrentLimit(units::Current)`
  (true when the device took it) and `[[nodiscard]] units::Current currentLimit() const` (READ BACK from the device, never
  the request's echo — `IMotor::brakeMode()`'s rule). PROS-free.
- **CHANGED `hal/pros/motor.hpp`** — `ProsMotor` implements it (`motor_.set_current_limit(mA)`, `get_current_limit()`;
  A → mA rounded, a PROS_ERR → `false` / the last good reading, counted in its `faultedReads()`).
  **CHANGED `test/pros_shim/pros/motors.hpp`** (+ `shim_control.hpp` if the shim's state lives there) — the two calls,
  recording what was set, and a knob to make the device answer PROS_ERR or a different number.
- **CHANGED `hal/fake/fake_motor.hpp`** — `FakeMotor` implements it (stores the limit, default 2.5 A; a knob to refuse or
  to "lose power" — reset to 2.5 A — for the re-check's test).
- **CHANGED `hal/device_provider.hpp`** — `virtual IMotorCurrentLimit* currentLimit(std::uint8_t port) = 0;` the limiter
  of the motor THIS provider made on `port`, nullptr when none; never makes anything (`readFaults`' rule). **Every
  implementer:** `hal/pros/device_provider.hpp`, `hal/fake/fake_device_provider.hpp`, `test/calibrate_program_rig.hpp`
  (`CountingProvider` — forwarding, as for `aiVision`) — **and any other found by `grep -rn 'IDeviceProvider' include src
  test` at the run's tip** (VIS1-A1's lesson).
- **CHANGED `mechanism/devices.hpp`** — `Devices::currentLimits[config::kMaxMotorsPerMechanism]` (HAL pointers only).
  **CHANGED `robot/mechanism_parts.hpp`** — fills them from `currentLimit(port)` for each motor it built.
- **CHANGED `mechanism/spec.hpp`** — `MechanismSpec::currentCapAmps` (0 = none stated); **APPENDED refusals** (the
  enum's end, the check at the END of `specRefusal` so the pinned order of the old ones is untouched):
  `ACurrentCapMustBeAbove0AndAtMost25A` (written, but not in (0, 2.5]; a NaN refused — "not inside" form, :1229's rule),
  `AStallSignatureMustBeBelowTheCurrentCap` (any stated `power.stall`, `position.stall`, `position.homeStall`,
  `speed.stall` with amps ≥ the cap — "a capped motor never draws more: the signature could never trip"), each with its
  sentence in `refusalText` and its named `refused::` function.
- **CHANGED `mechanism/kinds.hpp`** — `struct CurrentCap { double amps = 0.0; }` and `.currentCap` right after `.stall` on
  `Power`, `Arm`, `Lift`, `Flywheel` (the "what it does stopped" group, :26-30); each `spec()` copies it.
- **CHANGED `mechanism/service.hpp`** — at bind, for every BUILT motor kind: **a stated cap is written to EVERY motor of
  the mechanism and read back**; *(re-check #8: D12 (b) — the draft wrote 2500 mA back here)* **no cap stated: each
  motor's limit is READ and NOTHING is written**; one that reads other than 2500 mA gets one Info line naming both
  possible causes (a cap a previous program left — HA-220 — or VEXos's sharing — HA-221). One Info line per capped
  mechanism (per motor and in all); a Warn when the motor would not take it (the port, the asked and the read-back
  numbers) or when the provider gave no limiter ("this provider cannot cap a motor"); *(re-check #3)* a Warn when a
  stated stall or homing signature is within 0.2 A of the cap (HA-223); a STRING lift (one that declares `.slack`, after
  MECH-S-B1) without a cap: a Warn. **Once a second** (on the service's own clock, inside a tick) each capped motor's
  limit is read; *(re-check #8)* one that reads back the motor's DEFAULT 2500 mA (it lost power — HA-220) is re-written
  with one Warn; one that reads any OTHER number is a Warn ONCE and is not re-written (never a fight with whatever set
  it). No step verdict changes; nothing is refused at boot. **The region of `service.hpp` this run edits** (for §6.2):
  the slots' constructors (bind) and one once-a-second step in the service's tick.
- **CHANGED `sim/mechanism_plant.hpp`** — the plant honours motor 0's `currentLimit()` (every motor of a group is set
  alike) **only when a limit BELOW 2.5 A is set** *(re-check #15: the draft limited at every limit and claimed the
  default changed nothing — false: the plant clamps only its READING at 2.5 A, and a 12 V jog reversed to −12 V drives
  |V − kV·ω| to 18.26 V, past 2.5 A's torque, on 2 ticks — the checker's `p_plantlimit`, re-run byte for byte)*: within
  each step the applied volts are limited so |V − kV·ω| ≤ limit·12/stallAmps, re-solved at the END speed (the way the
  12 V clamp is re-solved, :222-226) so a joint stopped at a hard stop publishes at most the limit and pushes with at
  most its torque. **At the default the limiter is bypassed, so every existing plant result is unchanged** — the run
  proves it by the whole suite, byte for byte in its numbers. (That the plant lets a reversal past 2.5 A's torque is an
  existing plant inaccuracy — HA-183's kind; the guide's chapter on the plant says so; fixing it would re-pin reversal
  results and is not this run's.)
- **Robot two:** its claw pivots state no cap; each boot reads two limits and writes nothing; *(re-check #8)* its boot
  lines stay byte-identical **IF both read 2500 mA** — a reading other than that adds one Info line each (Step 0.4
  measures it; HA-221). `src/` is otherwise untouched.
- ~~**Rule 6 (i) if G3a Part 5 has landed at this run's tip** (likely — Part 5 builds ~Nov 4–8): `tools/robot_tool.py`
  learns `.currentCap` … If Part 5 has not landed, Part 5 adopts it.~~ *(re-check #1: Part 5 is after the panel under
  the 11-03 order — Part 5 ADOPTS `.currentCap`; this run does not edit `tools/robot_tool.py`.)*
- **Tests** (§2.2): NEW `test/mechanism_current_cap_test.cpp`, `test/motor_current_limit_test.cpp` (the fake, the
  provider lookup), additions to `test/mechanism_spec_test.cpp` (the refusals and their order), `test/pros_motor_adapter_test.cpp`,
  `test/fake_motor_test.cpp`, `test/sim_mechanism_plant_test.cpp`, `test/mechanism_parts_test.cpp` (the limiters in
  `Devices`), and `test/compile_refusals/` (`.currentCap = {.amps = 3.0}` fails the build naming its reason).

### 1.2 TUNE1-A — live tuning's library half (D8's rules, D9's block)
- **NEW `include/shulib/mechanism/live_tuning.hpp`** (PROS-free, allocation-free): `mechanism::LiveTuning` — holds the
  robot's `MechanismTable` **COPY** (mutable, the program's static storage; the service is bound over it so the slots'
  `spec_` point INTO it) and the **declared** table (the robot file's, for "narrowed only" and RESET); `apply(index,
  Change) → Applied{ok, sentence}`; `undo()` (16 deep), `reset(index)`; `tuningRefusal(declared, tuned)` = **the build's
  `specRefusal(tuned)` FIRST, then the live bounds** (every number finite; kP > 0; kI, kD, kS, kA, kV ≥ 0; kP, kI, kD,
  kV, maxVelocity, maxAcceleration, tolerance, settleTime never exactly 0 — the file cannot say 0, P9; maxVelocity at
  most the output's free speed through the gearing in its units — the starting profile's own scale,
  `detail::unitsPerOutputDeg`; the range only narrowed and never past a preset, the home position *(re-check #7)* **or
  the target the mechanism is holding now**; homeVolts pointing at its home end (the build's rule); every amps below
  the cap; a flywheel's band > 0 …), each refusal one sentence; `Parameter` descriptors per kind (name, unit, step rule
  — *(re-check #14)* an additive first step for a number that starts at 0 (kA: kV × 0.01 s, then ×/÷ 1.25) —, how the
  robot file says it; *(re-check #14)* a flywheel has no kA: `SpeedTuning` cannot say one — the page renders them and
  the copy block prints them); `copyBlock(index, sink)` — the `[TUNE]` lines of D9 (*re-check #14:* **the smallest set
  of stated numbers whose declaration builds to exactly the live spec** — the file's stated ones, the live-changed ones,
  and each derived one whose re-derivation would differ; each line ≤ 200 bytes with a precision on every `%`, naming
  its member as the declaration does, an end line with the count).
- **CHANGED `mechanism/service.hpp`** — `MechanismService::rederive(std::size_t index) → bool`: REFUSED (false, nothing
  changes) inside a tick or while that slot's STEP runs; otherwise the slot re-reads its spec: the PIDs' gains
  (**bumpless**: a hold keeps its I-TERM in volts, not its accumulator), the stall and homing-stall detectors rebuilt
  (reset), the limits in degrees and the soft stop's acceleration recomputed, ~~a hold whose target is outside a
  narrowed range re-clamped (`holdAt`'s rule)~~ *(re-check #7: never needed — `tuningRefusal` refuses narrowing past the
  held target, so no re-derive ever moves a hold; `rederive` ASSERTS it, a precondition)*, the cap re-written (MECH-S-A's
  path), and one Info line (`'lift': retuned live -- kP 0.250 -> 0.312`). `PowerSlot` and `SpeedSlot` likewise for their
  numbers. Nothing else of the service changes. **Its region of `service.hpp`** (for §6.2): new `rederive` members on
  the slots and on `MechanismService`.
- **CHANGED `control/pid.hpp`** — `Pid::setGains(kP, kI, kD)` (additive; finite gains, a precondition): keeps the
  integral TERM — `integral_ = kI_old·integral_ / kI_new` when kI_new ≠ 0 — so a gain change makes no jump of its own.
  (C1's layer, not in the Freeze Register.)
- **Tests:** NEW `test/mechanism_live_tuning_test.cpp`, `test/mechanism_tuning_rules_test.cpp`,
  `test/mechanism_tuning_block_test.cpp`; an addition to `test/pid_test.cpp` (the bumpless rule).

### 1.3 TUNE1-B1 — "shulib Tune" on the brain (D7 (a), D8 (a))
- **NEW `include/shulib/mechanism/tune_panel.hpp`** — the PURE panel model over `ui/touch_panel.hpp` (CAL1-A1 / COMP-a's
  pattern): pages (TUNE, TEST REPORT, STRING CHECK — the last empty until MECH-S-B fills it), every rectangle and padded
  hit box, the arming delay, ASCII with no `%` (`ui::screenText`), the text budgets (HA-136).
- **NEW `include/shulib/mechanism/tune_program.hpp`** — the PROS-free program: builds ONLY the mechanisms
  (`robot::MechanismParts` over the provider from the copied table's `toMechanismConfig`, the configs kept in static
  storage beside the parts) — *(re-check #7)* **only after the stage-1 clear confirm** —, binds ONE `MechanismService`
  over the copy, ticks it at the pacer's 10 ms, reads the screen through `hal::IBrainScreen`, the controller through
  `hal::IController` and the competition status byte from the shell (CAL1's `kCompetitionDisabled`, `calibrate_program.hpp`
  :189-194) (**any button, the controller dropping (`isConnected()` false) or VEXos's DISABLED → every mechanism
  released to its safe state**, the page's STOP likewise; TEST/HOME/STRING CHECK refused with no connected controller or
  with a field or switch plugged in; stage 2 of the confirm before a mechanism's first motion), runs TEST A/B as
  ordinary steps (*re-check #7:* bounded by the plan's own time × 2 + the settle time + 0.5 s, at most 10 s; the verdict
  line logged) and measures each move's report from the samples (time to the verdict, the farthest past the target, the
  time from first inside "reached" to settled, the most current, the final error, *(re-check #3)* the HOLD current over
  the second after it settles, and a warning line when the stall signature is not above that hold or the cap not above
  the peak), HOME as the declaration's homing step, COPY as `LiveTuning::copyBlock`; *(re-check #8)* on RESET, on
  leaving a mechanism's page and on EXIT it writes back every motor limit it changed (D12 (b)). Never constructs a
  chassis, a drive motor or a Robot.
- **NEW `include/shulib/hal/pros/tune.hpp`** — the PROS shell (`hal/pros/calibrate.hpp`'s pattern): the provider with
  its SD log copy at `/usd/tune_log.txt` (*re-check #14:* `ProsFileMode::Append` — kept across boots, the Competition
  program's mode; the draft said a new file each boot), the screen, the controller, the competition status, the task.
- **NEW `src/programs/tune_program.{hpp,cpp}`** — a few lines: the robot's table (`kMechanisms`) and the shell.
- **The build:** `Makefile` (PROGRAM=tune, `-DSHULIB_PROGRAM_TUNE`, the slot-7 upload line; *(re-check #10)* **accepted
  for ANY `ROBOT` whose mechanism table is not empty** — an `$(error)` naming why for one with none — never a
  `ROBOT=tank`-only row like the other programs', so robot one's plumbing chunk gets Tune by registering its table),
  `src/build_selection.hpp` (a `Program::Tune` branch, its guard and beacon
  `shulib-robot-variant=tank-tune`), `src/main.cpp` (its case), `tools/src_build_gate.py` (the per-build tables; step 8:
  the Competition, Drive and Auton Tester ELFs carry NO `LiveTuning` / `tune_program` symbol and the Tune ELF no
  chassis), `.github/workflows/ci.yml` (the eighth build), `README.md` / guide 07's slot list.
- **Tests:** NEW `test/tune_panel_test.cpp`, `test/tune_program_test.cpp` (the program over `FakeBrainScreen`, a fake
  controller, `FakeDeviceProvider` and the host test plant: every page, every refusal shown, STOP, a test move's report
  against an independent computation from the recorded samples), `test/build_selection_test.cpp` additions.

### 1.4 TUNE1-B2 — the copy-back (D9 (a)) — AFTER THE PANEL *(re-check #1)*
**Starts when G3a Part 5 has merged** (it makes `mechanisms` a key the tool reads — until then `g3a-p1`'s
`tools/robot_tool.py` :29-30, :244-253 refuses it by name), Part 6 for the button. Under the 11-03 order that is after
Nov 21; until then COPY's lines are copied by hand (D9 (a)).
- **CHANGED `tools/robot_tool.py`** (rule 6 (i)) — `import-tuning FILE [--robot ID]`: reads a terminal log or
  `/usd/tune_log.txt`, takes the LAST complete `[TUNE]` block per mechanism, refuses a block that is cut (no end line, a
  count that does not match, a line the 200-byte cut shortened), a mechanism the robot file does not have, or a number
  the validator refuses; writes through the tool's ONE writer with provenance per value; prints what it changed. Its
  `self-test` gains the cases. `test/robot_tool_agreement_test.cpp`: an imported file `generate`s to C++ whose `spec()`
  equals the live spec the block came from, field by field.
- **CHANGED `tools/robot-panel/`** — the MECHANISMS page's IMPORT TUNING button (a file picker → the subcommand), if Part
  6 has landed at this run's tip (else Part 6 adds it — G3a :1138-1145).

### 1.5 MECH-S-B — the slack watch (B1) and its bench page (B2) (D3 (a), D4 (e), D5 (a))
*(re-check #1: SPLIT. **MECH-S-B1** is everything below except the STRING CHECK page — it touches no Tune file, so it
can build BESIDE TUNE1-B1 once TUNE1-A's build ends. **MECH-S-B2** is the STRING CHECK page in `tune_panel.hpp` /
`tune_program.hpp` and the live `.slack`/acceleration bounds in `live_tuning.hpp`, after TUNE1-B1's build.)*
- **CHANGED `mechanism/spec.hpp`** — `SlackSpec` in `PositionSpec` {`declared`, `cruisingBelowAmps`, `aboveMotorRadS`,
  `lasting`, `holdingBelowAmps`, `holdingLasting`, `offBottomIn`}; APPENDED refusals: a measured slack signature not
  whole (any number written, one missing); its amps not above 0, not below 2.5, not below the cap; its rpm or lastings
  not above 0; `offBottom` outside the travel; **`homeByStall` pushing toward the BOTTOM on a lift that declares a
  slack signature** (D5, P5's reason in the sentence); a slack signature on a position mechanism in DEGREES (an Arm) —
  Lifts only; *(re-check #9)* **a stated `maxAcceleration` above g (386.09 in/s² in the lift's units) on a lift that
  declares `.slack`** (R4's reason in the sentence: "the carriage would leave the string and snatch it").
  *(re-check #9)* On a `.slack` lift the STARTING acceleration is capped at 0.5 g (`positionStart`'s caller — the cap
  applies only when `.tuning.maxAcceleration` is 0).
- **CHANGED `mechanism/kinds.hpp`** — `struct Slack {…}`, the word `notMeasuredYet`, and `.slack` on `Lift` right after
  `.currentCap`.
- **CHANGED `manipulation/mechanism_outcome.hpp`** — `Slack = 9`, "SLACK" (appended). **CHANGED `diag/fault.hpp`** —
  `MechanismSlack = 13`, "MECHANISM_SLACK" (appended; on the CONTINUE side of the scheduler's default abort mask — the run
  reads the mask at its tip and pins it). `test/fault_test.cpp`, `test/driver_panel_test.cpp`'s list, any exhaustive
  switch the src gate's `-Wswitch` finds.
- **CHANGED `mechanism/service.hpp`** — the watch in `PositionSlot` (only homed, only with a measured signature):
  `tickMoveOrHold` — the CRUISE window (the profile's sample with zero acceleration and positive velocity toward
  `maxPosition`, the motor faster than `aboveMotorRadS`, `maxCurrent()` ≤ `cruisingBelowAmps`, continuously for
  `lasting`; any other sample resets it — the stall detector's rule) and the HOLD window (Mode::Hold for at least
  `holdingLasting`, the target above `minPosition + offBottomIn`, `maxCurrent()` ≤ `holdingBelowAmps`, continuously for
  `holdingLasting`); `tickJog` — a push UP whose volts have not changed for `lasting`, outside the soft stop's approach,
  counts as cruising, and *(re-check #5)* **the jog's soft-stop HOLD at a limit above `offBottom` counts as a hold**
  (R3). ~~A trip: D4 (a) — … **the latch** (`slackAt_`): `beginStep` ends every later step REFUSED with the note;
  `jog`/`holdHere` do nothing.~~ *(re-check #6)* **A trip: D4 (e)** — `finishStep(…, Slack, …)` or the after-step form,
  `MECHANISM_SLACK` once, the safe state, and **the lift UN-HOMED** (`homed_ = false`, the time kept for the note), so
  a later position step ends NOT_HOMED through the existing path (`service.hpp` :791-795) with "its string went slack
  at 12.3 s: its height is unknown until it is homed again", and a held button jogs it at `.unhomedVolts` (:1043-1046);
  a boot line when `.slack` is declared and `.unhomedVolts` is 0. (Under (a), if he chooses it: the draft's latch.)
  *(re-check #4)* `tickHome` — **the downward SWITCH-homing guard** on a `.slack` lift (D5 (a)): TIMED_OUT, named, once
  the motor is 1 in below its home position (it had a zero) or has turned 1 in from where the push began (it had none).
  A string lift with `notMeasuredYet` or a phase the bench refused: that half OFF, said at boot. *(re-check #4)* A
  `spool()` lift with no `.slack`: one Info line at boot. *(re-check #9)* A `.slack` lift whose acceleration is above
  0.5 g: a boot Warn. Nothing changes for a lift without `.slack` (every MECH-A3 test unchanged). **Its regions of
  `service.hpp`** (for §6.2): inside `tickMoveOrHold`, `tickJog` and `tickHome`; the trip's un-homing; boot lines in
  the slot's constructor. *(re-check #2)* **It adds NO start-command precondition** — so nothing goes into STEP1-B's
  check functions (D8 (a) of STEP1's brief): those hold the commands' preconditions, which THROW and read only
  "arguments + spec + peer(), none run-dependent" (STEP1's re-check row 9); the un-homing is run-time state answered by
  a VERDICT (NOT_HOMED — or REFUSED under (a), the way `beginStep` answers "another owner holds its claim",
  `service.hpp` :357-382), which STEP1's ST33 already handles at a trigger's fire (the leg completes, then the drive
  step fails `MechanismStepFailed`). A check that threw at arming on a run-time state would be the mid-match throw
  STEP1's own re-check #1 removed.
- **NEW `include/shulib/mechanism/slack_bench.hpp`** (pure) — `SlackBench`: collects the cruise and hold samples of
  THREE ON passes and THREE UNHOOKED passes (the same moves; holds at the top, the middle and 0.5 / 1 / 1.5 / 2 in, each
  approached from above and from below), proposes each threshold MIDWAY, *(re-check #5)* refuses a phase whose gap
  (ON low − UNHOOKED high) is below max(0.03 A, 4σ̂) (σ̂ from successive differences — D3; the sentence names the gap
  and the need), derives `aboveMotorRpm` (half the ON passes' slowest cruise motor speed) and `offBottom` (the lowest
  test height whose holds all drew the ON holding low, + 0.5 in), proposes `lasting` / `holdingLasting` as their
  starting values (said so), reports its margins, excludes the acceleration and deceleration samples (P12), and words
  the proposal as the `.slack` member and as `[TUNE]` lines.
- **(S-B2) CHANGED `mechanism/tune_panel.hpp`, `tune_program.hpp`** — the STRING CHECK page (guided: home, the ON
  passes, "now unhook the string at the carriage and take it off the spool or tape its end — §8 landmine 15" — *(re-check
  #5)* once per string whose snap matters, for a lift with more than one —, the UNHOOKED passes, the proposal or the
  refusal; APPLY puts it in the live spec, COPY prints it). **(S-B2) CHANGED `mechanism/live_tuning.hpp`** — the live
  bounds for `.slack` (the build's refusals above, plus the minimum gap) and *(re-check #9)* the acceleration bound on a
  `.slack` lift (refused above g, a page note above 0.5 g).
- **NEW `include/shulib/sim/string_carriage.hpp`** — the host test plant's carriage (P1's model, made a fixture): tied to
  the spool until it snaps, then free-falling to its bottom stop; at the snap it takes its weight off the joint
  (`addLoadVolts`); a snap at a height, a time or a phase; *(re-check #9)* **ONE-SIDED** — the string pulls and never
  pushes, so a spool that slows faster than g leaves the carriage rising on its own (R4's model) and the fixture can show
  the slack and the snatch; *(re-check #5)* a "supported" mode (something carries the carriage at a height — V3's case)
  and a second string (one of two snapping).
- ~~**Rule 6 (i)** — `tools/robot_tool.py` learns `.slack` …~~ *(re-check #1: Part 5 ADOPTS `.slack` and
  `notMeasuredYet` — Part 5 is after the panel; this run does not edit `tools/robot_tool.py`.)*
- **Tests:** NEW `test/mechanism_slack_test.cpp`, `test/mechanism_slack_bench_test.cpp`, `test/sim_string_carriage_test.cpp`;
  additions to `test/mechanism_routine_test.cpp` (a SLACK step stops the chain, a later start NOT_HOMED stops it),
  `test/mechanism_driver_loop_test.cpp` (after SLACK a held button jogs at `.unhomedVolts` — 0 V: nothing), `test/fault_test.cpp` and
  `test/driver_panel_test.cpp` (the new code), `test/tune_program_test.cpp` (the page — S-B2), `test/mechanism_spec_test.cpp`
  and `test/compile_refusals/`; *(re-check #2)* if STEP1-B has landed at the run's tip, `test/routine_trigger_test.cpp`
  (or STEP1-B's file) gains SB15.

### 1.6 Not touched by any run
`hal/motor.hpp` (F4), `chassis/routine.hpp` (F10), `chassis/chassis.hpp` (F6), `src/programs/bench_r3a.*` (MECH-A6's),
`src/robots/mechanism_table.hpp`'s regions (no robot declares a string lift; robot one's, when it exists, through the
robot file and `generate`), `include/shulib/version.hpp` (no frozen surface grows — L6's policy, its §1.8).

---

## 2. Step 0, the runs, their tests and mutations, and the definition of done

### 2.0 STEP 0 at the robot — what the team settles (no new code)
- **0.1** Which robot carries the string lift (and when its first build exists — D10 Q2), its motors (how many on the
  spool), cartridge, gearing, spool diameter, travel, and its rigging: pulled up by string, lowered by gravity (the
  design's assumption) or driven both ways (OUT); *(re-check #5)* **how many strings in parallel and how many stages**
  (one string's snap out of two may not show in the current — D3).
- **0.2** The string: material, diameter, rated strength (the cap's ceiling, HA-222).
- **0.3** Unpowered at program start, does the carriage rest on its bottom stop (HA-226 — `restsAtStop` depends on it)?
- **0.4** *(re-check #8 — settles D12)* After MECH-S-A lands, on robot two with ALL 10 motors plugged in: the boot's
  read-back of the claw motors' limits (2500 mA expected — HA-221). After TUNE1-B1 lands: a cap set on a claw pivot in
  Tune, the program stopped from the brain (no RESET), then shulib Drive's boot read-back (HA-220). The coordinator
  brings D12 back to the team lead with both numbers.
- **0.5** *(re-check #3)* In the Tune program, before choosing a cap: each TEST move's peak and the hold current after
  it, WITH and WITHOUT the heaviest game piece the lift carries — the four currents of D2's ordering.

### 2.1 The runs — six, each its own go, each verified and committed on its own (D10) *(re-check #1: re-ordered and re-sized; B split; B2 after the panel)*
| Run | What | Starts | Size (builder-days, rough) |
|---|---|---|---|
| **MECH-S-A** | the cap (§1.1) | on the go and a free slot (the 11-03 order: after the AF1 build starts, ~Nov 8 at the latest); VIS1-A1 has committed (`9d77c9d`) | ~2¼–3 |
| **TUNE1-A** | live tuning's library (§1.2) | when MECH-S-A's BUILD ends (it rebases on A's `service.hpp` at its merge) | ~2½–3¼ |
| **TUNE1-B1** | "shulib Tune" (§1.3) | when TUNE1-A's build ends | ~3½–4¼ |
| **MECH-S-B1** | the slack watch (§1.5, all but the page) | when TUNE1-A's build ends, BESIDE TUNE1-B1 if a second slot is free (no Tune file; `service.hpp` after T-A); else after TUNE1-B1's build | ~2¾–3¾ |
| **MECH-S-B2** | STRING CHECK and the live `.slack` bounds (§1.5, S-B2) | when TUNE1-B1's and MECH-S-B1's builds have both ended | ~1¼–1¾ |
| | | | **~12¼–16 before the panel** |
| **TUNE1-B2** | the copy-back (§1.4) | AFTER THE PANEL: when G3a Part 5 has merged (Part 6 for the button) | ~1¼–1¾ |
| | | | **~13½–17¾ in all** |

Each run: three reviewers, fixes, a fresh verifier, the coordinator's checks (§7) — the house's, not counted above.

### 2.2 Tests (ADVERSARIAL — each tries to break ONE rule) and the mutations each must turn red
**MECH-S-A**
- **CA1** the build refuses a written cap ≤ 0, > 2.5, NaN; accepts 2.5 exactly and none; the sentences pinned.
- **CA2** every stated signature (power, position, homing, flywheel) at the cap refused, 0.01 A below accepted (the
  build); *(re-check #3)* one 0.19 A below boots with the within-0.2 A Warn, one 0.21 A below without it.
- **CA3** the refusal ORDER: every old reason in its pinned place, the two new ones last.
- **CA4** bind writes a stated cap to EVERY motor of a 2-motor lift and reads it back; the boot line says per motor AND
  in all; *(re-check #8: D12 (b))* a mechanism with no cap whose motor reads 1.0 A gets one Info line naming both causes
  and is NOT written; one that reads 2.5 A is NOT written and logs nothing new (the fake counts writes: zero for every
  no-cap mechanism).
- **CA5** a motor that refuses (or reads back 2.0 for 1.2): a Warn naming the port and both numbers; the mechanism runs.
- **CA6** the plant: a joint stalled under a 1.2 A limit reads ≤ 1.2 A and an extra load above that limit's torque but
  below 2.5 A's stalls it (the limit limits TORQUE, not only the reading); *(re-check #15)* at the default the limiter is
  BYPASSED — every existing plant test unchanged, and the checker's reversal case (a 12 V jog flipped to −12 V) gives the
  same samples, byte for byte, as before the change.
- **CA15** *(re-check #3, NEW)* the ordering: a lift capped at 1.2 A with a 1.0 A signature holds a 3 V load clean; a
  piece that raises the hold to ~1.08 A ends it STALLED while holding (V1's case, pinned as the documented reason);
  the guide's sentence and the boot Warn's are pinned.
- **CA7** STARVED (P10): a 12 V push into an obstacle and 1.5 s with no tick — the current stays ≤ the cap throughout.
- **CA8** a move into an obstacle under a cap with the signature below it: STALLED at the uncapped time (3.22 s, P10);
  after contact the pull never exceeds the cap (the impact tick excluded and SAID so in the test's comment).
- **CA9** homing UP into the top stop under a cap: SUCCEEDED, ≤ the cap after contact (P11).
- **CA10** the once-a-second re-check: a fake motor that "loses power" (resets to 2.5 A) mid-run is re-written within 1 s,
  one Warn; one that keeps its cap is never re-written; *(re-check #8)* one whose limit changes to 1.8 A (neither its cap
  nor the default) gets ONE Warn and is never re-written, however many seconds pass.
- **CA11** `ProsMotor` through the shim: 1.2 A → 1200 mA; PROS_ERR → false and the fault count; the read-back in A.
- **CA12** `currentLimit(port)` on every provider: the motor made there, nullptr for an empty port, makes nothing.
- **CA13** `compile_refusal`: `.currentCap = {.amps = 3.0}` and a stall at 1.5 under a 1.2 cap fail the build naming
  their `refused::` functions.
- **CA14** robot two's boot lines byte-identical when both claw motors read 2500 mA (the existing pins re-run); `src/`
  diff empty.
- **Mutations (each must turn a test red):** ~~MA1 the default-reset write dropped (CA4)~~ *(re-check #8)* **MA1 the
  no-cap Info line dropped (CA4)**; MA2 the cap written to motor 0 only (CA4); MA3 `>=` → `>` in the signature-below-cap
  check (CA2); MA4 the plant ignores the limit (CA6, CA7); MA5 A → mA × 100 (CA11); MA6 the re-check never re-writes
  (CA10); MA7 a new refusal inserted mid-order (CA3); MA8 the boot line's total not × motors (CA4); MA9 a no-cap motor
  written every boot (CA4's write count); *(re-check)* MA10 the re-check re-writes ANY changed value (CA10's 1.8 A
  case); MA11 the plant limiter active at the default (CA6's reversal case); MA12 the within-0.2 A Warn dropped (CA2).

**TUNE1-A**
- **TA1** each live number takes effect — P6 INVERTED: kP × 4 changes the hold's command at the next tick by exactly
  ΔkP × the error; kI changed in a hold: the command jumps by nothing beyond ΔkP's share (bumpless); maxVelocity at the
  NEXT move; a narrowed max: a 6 V jog stops at the new limit (P6's 20.71 in case now ≤ 12 + the tolerance); a new stall
  number trips as the new number says; a new cap reaches the motors.
- **TA2** refused mid-step and mid-tick: the spec, the slot and the motors byte-identical (memcmp of the spec; the next
  ticks' commands equal an untouched twin's).
- **TA3** every refusal of the build (one case per `Refusal` reachable by a live change) AND each live bound (NaN, a
  negative kS, kV past the free speed's, a widened range, a range past a preset, kD exactly 0) refused with its sentence,
  nothing changed.
- **TA4** UNDO restores the previous spec bit for bit (16 deep, the 17th refused); RESET restores the declared spec bit
  for bit; a fresh `LiveTuning` holds the declared table.
- **TA5** the copy block: every line ≤ 200 bytes; the end line's count; a test-local parser of the block, building a
  declaration from it, gives a `spec()` equal to the live spec FIELD BY FIELD (maxAcceleration included when only
  maxSpeed changed — landmine 12); the printed precision round-trips each number the page can produce.
- **TA6** presets: after any live change `find(lift["high"])` still finds the slot (presets are never touched).
- ~~**TA7** a hold above a newly narrowed max is re-clamped inside it at the re-derive.~~ *(re-check #7)* **TA7** a
  narrowed max BELOW the target the lift is holding is REFUSED with its sentence (the hold, the spec and the motors
  untouched — the next ticks equal an untouched twin's); the same narrowing after a TEST move inside it is accepted.
- **TA8** a Flywheel's and a Power mechanism's live sets, each number taking effect; *(re-check #14)* a Flywheel offers
  no kA.
- **TA9** *(re-check #14, NEW)* an Arm's kA at 0: [+] sets kV × 0.01 s, the next ×1.25; [−] from that value gives 0; the
  copy block states kA only when it is not 0.
- **TA10** *(re-check #14, NEW)* the minimal block: a live change of maxSpeed alone gives a block with `.tuning.maxSpeed`
  AND `.tuning.maxAcceleration` (the acceleration it was tuned with) and NOT kV; a change of kP alone gives kP only;
  each block's declaration builds to the live spec field by field (TA5's parser).
- **Mutations:** MT1 the PIDs' gains not refreshed (TA1 — P6's +0.0000 V); MT2 `minDeg_`/`maxDeg_` not recomputed (TA1's
  jog); MT3 the live bounds skipped, `specRefusal` only (TA3); MT4 `setGains` keeps the accumulator, not the term
  (TA1's bumpless case); MT5 UNDO restores only the changed member's neighbour (TA4); MT6 the block omits a derived
  member whose re-derivation differs (TA5, TA10); MT7 rederive allowed mid-step (TA2); ~~MT8 the hold not re-clamped
  (TA7)~~ *(re-check #7)* MT8 the narrowing-past-the-hold refusal dropped (TA7); MT9 the block states every effective
  number (TA10's "NOT kV"); MT10 a ×1.25 step on a zero kA (TA9).

**TUNE1-B1**
- **TB1** the panel: every rectangle inside 480 × 236, every pair of buttons further apart than twice the hit pads, every
  text within its budget, ASCII, no `%` — over every page and every mechanism kind (names up to 23 characters).
- **TB2** ANY controller button → every mechanism released to its safe state within one tick, a test move CANCELLED.
- **TB3** a refused change shows its sentence and changes nothing; buttons greyed during a test move.
- **TB4** the test report equals an independent computation over the recorded samples (time, overshoot, settle, peak A,
  final error) on three plant mismatches.
- **TB5** the program never constructs a drive motor (the fake provider's request log holds only the mechanisms' ports).
- **TB6** the build gate: the Tune build's beacon; the three match ELFs contain no live-tuning symbol (a planted one turns
  step 8 red — the gate's self-test).
- **TB7** *(re-check #7, NEW)* the abort set: the controller disconnecting mid-TEST and VEXos's DISABLED bit each release
  every mechanism within one tick; TEST, HOME and STRING CHECK refused with no connected controller and with a field
  or switch status, each with its sentence.
- **TB8** *(re-check #7, NEW)* the clear confirm: before stage 1 the fake provider's request log is EMPTY (nothing made —
  a pneumatic line included); a mechanism's first motion waits for stage 2.
- **TB9** *(re-check #7, NEW)* a TEST move's bound: the plan's time × 2 + settle + 0.5 s, capped at 10 s; a plant that
  cannot reach the target reports TIMED_OUT at that bound.
- **TB10** *(re-check #8, #14, NEW)* RESET, leaving a page and EXIT write back every limit Tune changed (the fake counts
  them); the SD copy opens in Append mode (two boots' blocks both in the file).
- **TB11** *(re-check #3, NEW)* the report's hold current after settling, and its warning line when the stall
  signature is not above it (a piece on the plant raising the hold).
- **TB12** *(re-check #10, NEW)* the build row: `PROGRAM=tune` with a robot whose table is empty is an `$(error)`
  naming why; with robot two it builds; the row names no robot (the gate's table checks it).
- **Mutations:** MB1 the controller's STOP ignores one button (TB2); MB2 a page's button outside the panel (TB1); MB3 the
  report's overshoot measured from the start, not the target (TB4); MB4 the program builds the Robot (TB5); MB5 the gate's
  no-tuning check dropped (TB6); *(re-check)* MB6 a dropped controller ignored (TB7); MB7 the mechanisms made before
  stage 1 (TB8); MB8 EXIT leaves a changed limit (TB10).

**TUNE1-B2** (after the panel)
- **TC1** `import-tuning` takes the LAST complete block; refuses a block with no end line, a wrong count, a cut line, an
  unknown mechanism, a value the validator refuses — each with its sentence, the file untouched (byte-compare).
- **TC2** the agreement: imported → `generate` → `spec()` equal to the block's live spec, field by field.
- **TC3** provenance written per value; `check` passes; `self-test` green.
- **Mutations:** MC1 the first block taken, not the last (TC1); MC2 a cut block accepted (TC1); MC3 a value written
  without provenance (TC3).

**MECH-S-B1 and B2** (SB7's page half, SB14's page and SB16's live bound are B2's; the rest B1's)
- **SB1** the refusals (each its own case): incomplete slack; amps at/above the cap; `offBottom` outside the travel; a
  stall-home DOWN on a string lift (P5's numbers in the sentence); `.slack` on an Arm; `notMeasuredYet` accepted, its boot
  Warn pinned; *(re-check #9)* a stated `maxAcceleration` above g on a `.slack` lift refused, one at 0.6 g accepted with
  its boot Warn, the starting value capped at 0.5 g; *(re-check #4, #6)* the `spool()`-without-`.slack` Info line and the
  `.slack`-with-0-`.unhomedVolts` line pinned.
- **SB2** a snap cruising up (the string carriage at 10 in): SLACK within `lasting` + 2 ticks; `MECHANISM_SLACK` exactly
  once; the safe state; the line's words and numbers pinned.
- **SB3** a snap while holding: SLACK within 2 × `holdingLasting` + 2 ticks; a snap while LOWERING: NOT seen while
  lowering (the limit pinned) and seen at the next hold above `offBottom`; a snap with the target at the bottom: never
  seen (the limit pinned, the guide's sentence).
- **SB4** NO false trip: the four P8 loads with the bench's own proposal, 50 intact cycles each (up, down, holds at the
  top, the middle and the bottom; decelerations; holds right after a move; a driver's push up and its release;
  *(re-check #5)* a hold just above `offBottom`, a driver's push held at the top soft stop, a homing push), with seeded
  noise within the bench run's spread.
- ~~**SB5** the latch …~~ *(re-check #6: D4 (e))* **SB5** the un-homing: after SLACK every position step ends NOT_HOMED
  with the note; a held button jogs at `.unhomedVolts` (and at 0 V commands nothing); a `homeBySwitch` lift's HOME
  re-arms it (the watch on again); a `restsAtStop` lift's HOME answers its existing "kept" line and it stays un-homed;
  a Routine step that saw it stops the chain `MechanismStepFailed` "SLACK"; a later START on it stops the chain
  NOT_HOMED. (Under D4 (a): the draft's latch case.)
- **SB6** a driver's push up with a snapped string: SLACK; a push down: nothing.
- **SB7** the bench: midway thresholds from ON/UNHOOKED samples; P12's 1 V / σ 0.05 case refused with both numbers;
  acceleration and deceleration samples excluded (P12's accel-included overlap is NOT reproduced); a proposal never
  outside (UNHOOKED high, ON low).
- **SB8** the string carriage: tied until the snap; free fall after; the joint's load removed at the snap; *(re-check
  #9)* ONE-SIDED — a spool slowing at 1.25 g on R4's numbers leaves ~0.58 in of slack and snatches at ~21 in/s, at 0.9 g
  none; its "supported" mode and a second string.
- **SB9** a lift without `.slack` behaves byte-identically (MECH-A3's position tests re-run unchanged).
- **SB10** `MECHANISM_SLACK` on the CONTINUE side: a drive in the same Routine finishes its motion.
- **SB11** *(re-check #4, NEW)* the switch-homing guard: R1's case (−3 V, a 4 s bound, the switch never pressed) on a
  `.slack` lift ends TIMED_OUT with its sentence after ≤ 1 in past the old zero (from 6 in) and ≤ 1 in of travel (no
  zero), against R1's 2.68–7.02 in without it; a working switch homes as before; a lift without `.slack` unchanged.
- **SB12** *(re-check #5, NEW)* a snap while a driver's push is held at the top soft stop: SLACK within
  2 × `holdingLasting` + 2 ticks (R3: 0.043 / 0.062 A after vs 0.146 / 0.354 A before).
- **SB13** *(re-check #5, NEW)* the limits pinned as limits: a carriage SUPPORTED above `offBottom` (V3's case) reads
  SLACK (the guide's sentence); one string of two snapping with a small current change is NOT seen.
- **SB14** *(re-check #5, NEW)* the bench's sources: a gap below max(0.03 A, 4σ̂) refused with both numbers (P12's
  0.038 A at σ 0.02); `aboveMotorRpm` = half the slowest ON cruise speed; `offBottom` = the lowest full-load test height
  + 0.5 in (a supported-below-1.3 in plant gives 1.5 + 0.5 = 2.0 in); the two lastings proposed as starting values and
  said so.
- **SB15** *(re-check #2, NEW — only if STEP1-B has landed at the run's tip)* a trigger ARMED on an un-homed (or, under
  (a), latched) lift does NOT throw at arming; at its fire the start answers NOT_HOMED (REFUSED), the leg completes and
  the drive step then fails `MechanismStepFailed` — STEP1's ST33 form.
- **SB16** *(re-check #9, NEW — B2)* the live page refuses a `.slack` lift's acceleration above g and notes one above
  0.5 g.
- **Mutations:** MS1 the cruise window includes the deceleration (SB4 — P8's false trip); MS2 the bench keeps
  acceleration samples (SB7 — P12); MS3 the hold window ignores `offBottom` (SB4's bottom hold); ~~MS4 no latch (SB5)~~
  *(re-check #6)* MS4 SLACK leaves the lift homed (SB5); MS5 the fault raised every tick (SB2's "once"); MS6 the window
  not reset by a sample above the threshold (SB4 under noise); MS7 the watch active moving DOWN (SB4); MS8 the
  threshold at the ON low instead of midway (SB4 noise); MS9 the stall-home-down refusal dropped (SB1); MS10 the watch
  active before homing (a dedicated case: an unhomed jog with no string never trips); *(re-check)* MS11 the
  switch-homing guard dropped (SB11); MS12 the jog's soft-stop hold not watched (SB12); MS13 the minimum gap dropped
  (SB14); MS14 the acceleration refusal dropped (SB1).

### 2.3 Definition of done — every run
The suite green through `tools/suite_run.py`; `compile_refusal` green; the ARM header gate (every v2 header in one unit);
the CI guards — *(re-check #1)* once G3a Part 1 has merged they include `python3 tools/robot_tool.py check` and
`python3 tools/robot_tool.py self-test` (`ci.yml` :104-105 in `g3a-p1`), every run; `tools/src_build_gate.py check` (and
`self-test` for TUNE1-B1/B2); the API-doc tool's five checks; the
briefing check; every mutation of §2.2 seen RED by the builder and two re-run by the coordinator; the documents of §3 in
the same change, the prose review logged; the HA rows (§4) with a code site each; no hardware claim; the live log's
entry; `git diff --stat` matching §1's file list (anything else explained in the log).

---

## 3. Documents — in the same change as the code (pace rule 3), the prose review logged (rule 4), ≤ ~5 public each (rule 7)

### 3.1 Per run (public)
- **MECH-S-A:** guide 18 (`docs/guide/18-mechanisms.md`) — a section "**Current caps — how hard a string is pulled**"
  after "Jam signatures" (:247-273): the cap is per motor, the motor enforces it, the signature must be below it,
  *(re-check #3)* **the four currents' ORDER** (heaviest hold with a piece < signature < cap − 0.2 A; heaviest move's
  peak < cap; cap ≤ the string's safe pull) and V1's piece-on-a-hold example — and the existing recipe's "2.0 A" amended
  (under a cap it must be below the cap), the recipe for the number (D2), the momentum it does not limit; guide 11 (the
  cap's boot lines and Warns, *(re-check #8)* the no-cap Info line and its two causes); the glossary ("current cap");
  `docs/changelog.md`; the generated API pages (the new header; `api_doc_tool.py`).
- **TUNE1-A:** guide 18's "Tuning" section (:275-320) gains "**…or on the robot: live tuning**" (the rules, what is not
  live, why 0 cannot be set); the changelog; the API pages.
- **TUNE1-B1:** guide 18 — "**The Tune program**" (the page, the controller's STOP *(re-check #7: and a dropped
  controller, a disable, the clear confirm)*, COPY — *(re-check #1)* copied by hand into the mechanism table until the
  panel's import exists; *(re-check #14)* RESET and re-tune after a cartridge, gearing or spool change); guide 07's slot
  list and the README's program list (slot 7); guide 11 (the `[TUNE]` lines); the changelog.
- **TUNE1-B2 (after the panel):** G3a's robot-file guide chapter (its number at the tip — CAL1-A2b and G3a-P1 both add a
  chapter 19, RC17's "next free number at its tip") — "importing tuning"; the panel's README; the changelog.
- **MECH-S-B1/B2:** guide 18 — "**A string lift: the slack watch and STRING CHECK**" (what it sees, what it cannot —
  lowering, the bottom, a light lift, *(re-check #5)* one string of two; what READS as slack though no string broke — a
  carriage carried by a goal, a field element or the stop; *(re-check #6)* what SLACK does — un-homed, auton steps fail,
  the driver at `.unhomedVolts`, state one; homing a string lift — *(re-check #4)* the switch guard; *(re-check #9)* the
  acceleration below gravity, and the stick flipped from full up to full down); guide 11 (the SLACK line,
  `MECHANISM_SLACK`, the new boot lines); guide 12 (my lift says SLACK; STRING CHECK refused my lift; my homing stopped
  1 in past its bottom); guide 14 (what it cannot do yet: lowering, the bottom, one string of two, the carriage check
  after the panel); the glossary; the changelog.

### 3.2 Internal, every run
The live log (MECHS-PROGRESS / TUNE1-PROGRESS), `build-order.md` (the MECH phase row :848 and the chunk entries),
`REMAINING-WORK.md` rows 20e/20f, `HANDOFF-remaining-work.md`, `roadmap.md`'s Now paragraph, `PROJECT-BRIEFING.md`
(through `tools/briefing_status.py`), `docs/hardware-assumptions.md` (the run's HA rows and the Held line).

### 3.3 The plan commit (the coordinator's, on the team lead's go, BEFORE MECH-S-A)
This brief moves to `docs/internal/chunks/MECH-S-TUNE1-string-lift-safety-live-tuning.md` with his answers on top;
`build-order.md` gains the six runs and their order (D10 — *(re-check #1)* five before the panel, TUNE1-B2 after it, in
the deviations row the coordinator owes for the 11-03 order); REMAINING-WORK rows 20e/20f sized (and MECH-S-C added as
"after the panel" if D6 (a)); the HA register's Held line gains HA-219..HA-227; MECH-PANEL §5 Q15 marked answered (D1),
Q24 answered for mechanisms (the robot file — Q24's option (b), by D9 (a)); G3a's amendment §6 cross-referenced (the
copy-back is D9 (a); *(re-check #1)* Part 5 adopts `.currentCap` and `.slack`, +½–1 on Part 5); the 2026-10-29 "output
rotation sensor" wording corrected wherever it was written — REMAINING-WORK :110; build-order :855; *(re-check #16)*
**G3a's brief, `docs/internal/chunks/G3a-robot-profile-generator-panel.md` :218 and :1131** (and the coordinator's copy,
`.claude/coordinator/briefs/G3a-robot-profile-generator-panel.md` :193, :1106): under D6 (a) the "new named sensor slot
in `wire(...)`" for the string chunks goes away (a carriage-height sensor, if MECH-S-C is built, is its own wiring
then); the coordinator's memory note `robot-config-panel-g3a.md` :37 ("output rotation sensor optional and stronger")
is the coordinator's to correct; NOTES is history.

### 3.4 The prose review's greps (each run runs them and logs the hits it read)
`grep -rn -i 'simulator' docs/guide include/shulib/{mechanism,hal,sim}` (no new hit); `grep -rn 'internal/\|chunks/\|RESUMING'
docs/guide docs/cookbook README.md` (empty); `grep -rnE 'HA-(219|22[0-7])' include src test docs` (every id has a row and a
code site); every new log line's format in guide 11 equals the code's (a test pins it).

---

## 4. Hardware Assumptions — HA-219..HA-227 ONLY (the register's Held line gains them at the plan commit; reserved after STEP1's HA-216..HA-218)

| ID | Belief | Why we believe it | Status | Settled by |
|---|---|---|---|---|
| **HA-219** | A V5 motor given `motor_set_current_limit(L)` draws at most about L from then on — enforced by the motor itself within one 10 ms control tick, under any command, under Hold at 0 V, during a disable and with nothing ticking | the vendored text: "The motor will reduce its output at 1000 mA instead of the default 2500 mA" (`motors.h` :1137-1140); how fast is not stated | reasoned · speed **invented** | Tune program, a capped lift pushed into its top stop: the current trace (TEST report) |
| **HA-220** | A limit set by one program can persist in the motor into the next program, and a motor that loses power mid-run comes back at its 2500 mA default — so the library READS every mechanism motor's limit at every boot (writing only a stated cap — D12 (b)) and re-checks capped ones each second, re-writing one that reads the default | HA-98's open question (settings "live in the DEVICE", persistence untested — `docs/hardware-assumptions.md` :326 at `9d77c9d`), extended to the limit | **unverified** | Step 0.4: Tune sets 1.0 A, the program is stopped from the brain, then shulib Drive's boot read-back; a cable pulled and re-seated on a capped motor |
| **HA-221** | With more than 8 motors plugged in, VEXos may lower each motor's limit below what was set or read back (robot two runs 10); a cap below VEXos's own share behaves as stated; *(re-check #8)* if the read-back shows that share, writing 2500 mA over it at every boot would fight it — which is why D12 (b) writes nothing until this is measured | HA-172's open question (:391 at `9d77c9d`); MECH-PANEL §3.2 :247-249 | **unverified** | Step 0.4: robot two's boot read-back with all 10 motors plugged in, after MECH-S-A |
| **HA-222** | The string's pull is the motor's current times its torque per amp through the gearing over the spool's radius — about 10 lbf per amp per motor for a red cartridge (VEX's published stall torque, believed 2.1 N·m at 2.5 A — NOT checked in this tree) on a 1.5 in spool; friction adds little; the rotor's and the carriage's MOMENTUM at a hard stop add a pull no current cap limits | physics; the published number from memory | reasoned · number **unverified** | a spring scale on the string at a held current (Tune's HOLD at a known cap) |
| **HA-223** | A stalled motor held at its cap reads within ~0.1 A of the cap, so a stall signature at least ~0.2 A below the cap trips (the boot's within-0.2 A Warn rests on it — *re-check #3*) | the plant reads exactly the cap (P10: 1.15 under 1.2 trips) — the plant is not a motor | **invented** | Tune: a capped lift held against its top stop, the reading |
| **HA-224** | The V5 current reading's sample-to-sample noise is small enough that a lift's string-ON cruise current stays above its UNHOOKED cruise current (P12: below ~0.02 A for a 1 V gravity load, ~0.05 A for 2 V); *(re-check #5)* and a gap of at least max(0.03 A, 4σ̂) between them (σ̂ from successive differences) is wide enough that a midway threshold does not false-trip over a match | P12 on the host test plant with Gaussian noise (its tightest working gaps 0.036–0.038 A, over 20 cycles only); the V5's noise is unmeasured | **invented** | STRING CHECK's three-and-three passes on the real lift (the page reports both spreads and σ̂) |
| **HA-225** | A gravity-return string lift's cruising-up current carries the carriage's weight for the whole cruise and an unhooked spool's does not; lowering draws little either way (blind while lowering) | P2 on the plant; physics of a gravity-return lift | reasoned | STRING CHECK's ON/UNHOOKED runs; a deliberate unhook mid-hold at the bench |
| **HA-226** | Unpowered at program start, a gravity-return string lift's carriage rests on its bottom stop (a V5 motor does not hold after a program ends), so `restsAtStop` homes it correctly | VEXos stops the motors when a program ends; whether it brakes or holds is unmeasured | **unverified** | Step 0.3: end a program with the lift up and watch |
| **HA-227** | The STRING CHECK's UNHOOKED run stands for a snapped string: an unhooked spool's current equals a snapped one's (the broken end's weight and drag negligible) | the plant's snap removes all load (P1, P2); a real broken end drags a little | reasoned | a deliberate cut of a spare string at the bench (once, safely) |

---

## 5. OUT, and explicitly rejected
- **The carriage-height check** (a distance sensor aimed at the carriage) — MECH-S-C, after the panel (D6 (a)).
- **A spool-shaft rotation sensor as snap detection** — rejected: blind (P3).
- **A software cap** (D1 (b)); **both** (D1 (c)).
- **The low-power hold** (MECH-PANEL Q15's other half) and **heat warnings/derating** (§3.2) — MECH's rest.
- **Detecting a snap while lowering or at the bottom** by current — impossible (P8, P12; HA-225); the carriage check's.
- **A string lift driven both ways** (a loop with a down-string) — its down-moves carry tension the watch would have to
  learn; not the team's lift (D10's question).
- **A fraying string before it breaks** — no current signature known.
- **Slack on an Arm** (a string-pulled arm) — Lifts only until one exists.
- **Live presets** (D8 (c)); **live drive gains** (Part T shows R5a's measured ones; the drive's numbers are consumed at
  `Chassis` construction — another chunk); **live pneumatics**.
- **A stated-zero vocabulary** for `.tuning` (letting a file say kD = 0) — a meaning change to every declaration; MECH's
  rest if a lift needs it.
- **An SD tuning store read at boot** (D9 (b)); **live tuning in a match program** (D7 (b)).
- **Auto-tuning** (MECH-PANEL §3.6) and plotting a move's trace (the SHUL/2 wire, H1).
- **A motor's read-fault count for the watches** (a cable that fails leaves the motor's last good current — `IMotor`'s
  header :55-66; the provider's `readFaults` covers no motor): the SLACK line names "its motor stopped answering" as a
  possible cause; screening is its own change.
- **Controller rumble/text for SLACK** beyond the fault name the driver panel already shows — MECH C's controller output.
- *(re-check)* **The copy-back before the panel** — needs G3a Part 5 (D10); **a run-time "re-zero here" for a
  `restsAtStop` lift after SLACK** (a new control word — D4 (e) re-arms only by homing); **a jog slew limit** for a
  string lift (a driver flipping full up to full down — D5's acceleration note); **a build check on a stated top speed
  above the free speed** (P9 — the live page refuses one, the build still accepts it); **fixing the plant's torque past
  2.5 A on a reversal** (CA6 — an existing plant inaccuracy); **a "landed" current stop for switch homing** (R1 — not
  chosen, D5); **a strict cross-chunk serial rule on `service.hpp`** (§6.2 — the house rule instead).

---

## 6. Coordination — shared files, who waits for whom, and what the consumers get

### 6.1 In flight while this was written (2026-11-03 ~13:00 resume; NOT edited — every worktree read only)
MECH-A6 (`mech-a6`, on `81eb666`: `bench_r3a.{hpp,cpp}`, `test/bench_menu_grid_test.cpp`, NEW
`test/bench_mech_station_test.cpp`, guide 18, the HA register, the worksheet, README, the plan documents); CAL1-A2b
(`cal1-a2b`: `calibration/calibrate_{panel,program}.hpp`, `hal/pros/calibrate.hpp`, a NEW guide chapter 19, goldens);
IMU1-B (`imu1-b`: its log only so far); L6-A1 (`l6-a1`, awaiting merge: localization, `hal/distance.hpp` comments,
`sim/field_plant.hpp`); G3a Part 1 (`g3a-p1`, awaiting merge: `tools/robot_tool.py` NEW, `src/robots/chassis_table.hpp`,
`tools/src_build_gate.py`, `ci.yml`, a NEW guide chapter 19); VIS1-A1 (staged on main, uncommitted). STEP1's brief
(draft; HA-216..HA-218). *(re-check, ~14:00: VIS1-A1 COMMITTED as `9d77c9d`; L6-A1's merge staged on main; per the
11-03 order the AF1 brief, VIS1-A2, the ALLY1 brief, VIS1-B and the AF1 build (by ~Nov 8) come before MECH-S; G3a Part 1
still merges, Part 5 is deferred to after the panel — read from `NOTES.md` :905-906, no other source; NOTES records
no launch of robot one's build plumbing (RC18).)* *(re-check C2, ~16:00: L6-A1 COMMITTED `59d93bb`; STEP1's brief
ACCEPTED ~14:30 (D1–D8, HA-216..HA-218); the AF1 brief's draft read (§6.2); at ~14:50 the usage limit cut the lanes to
one until the reset (~16:00, five again) — `NOTES.md` :907-926; still no robot-one plumbing launch recorded.)*

### 6.2 Shared files — who writes them, who waits, who re-reads
| File | Others (when) | This chunk's run | Rule |
|---|---|---|---|
| `mechanism/service.hpp`, `spec.hpp`, `kinds.hpp` | MECH-A6 (none of them, at its tip); *(re-check #2)* **STEP1-B** (`service.hpp`: D8 (a) of its brief, ACCEPTED ~14:30 — the four start commands' precondition blocks — `run`, `moveTo`, `home`, `spinUp` — extracted into check functions; BEFORE or BESIDE MECH-S in the 11-03 order); ~~**the AF1 build** (from ~Nov 8, "a generic automatic action in the MECH service" — `build-order.md` :1783 …)~~ *(re-check C1)* **AF1-A** (`kinds.hpp` ONLY: `Arm::pieceSensor` after `.angleSensor` — its brief §1.4 :486, §6.2 :745; "**AF1 does not edit** `service.hpp`", :747; its new action lives in its own `mechanism/auto_action.hpp`); MECH's rest (after the panel) | S-A (slot constructors + a once-a-second tick step), T-A (new `rederive` members), S-B1 (inside `tickMoveOrHold` / `tickJog` / `tickHome`, the trip's un-homing, constructor boot lines) — **serial WITHIN this chunk** | *(re-check #2)* within this chunk: never two of its builders in these files. Across chunks: **the house rule — whoever lands second re-reads, and the merged tree's build and full suite are the gate** (VIS1-A1's lesson: a merge with no text conflict can still fail to build); each run names its region (above); ~~the coordinator serializes if AF1's brief names the same regions. A strict cross-chunk serial would queue all three runs behind AF1's ~8¾–11½-day build (D10).~~ *(re-check C1)* in `service.hpp` only STEP1-B overlaps — its check functions, none of MECH-S's regions; a strict cross-chunk serial would cost up to STEP1-B's ~3–4-day build (D10). In `kinds.hpp` AF1-A and MECH-S both edit the motor kinds' member lists and `spec()` (the Arm's: AF1's `.pieceSensor` after `.angleSensor`, MECH-S-A's `.currentCap` after `.stall`) — the second re-reads the member order at the tip; a declaration stating both follows it (AF1's landmine 12). Nothing MECH-S adds goes into STEP1-B's check functions (§1.5: no new start-command precondition) |
| `hal/device_provider.hpp` + its 3 implementers | VIS1-A1 (COMMITTED `9d77c9d`: `aiVision`); L6-A2 (if it adds an accessor) | S-A | whoever lands second greps every implementer (landmine 14) |
| `hal/pros/motor.hpp`, the PROS shim | none in flight | S-A | — |
| `sim/mechanism_plant.hpp` | none | S-A | the default-limit bypass and byte-identity is the guard (CA6) |
| *(re-check C1, NEW)* `src/common/driver_panel.hpp`; `test/driver_panel_test.cpp` | AF1-B (both), ALLY1-B (the panel). AF1's brief lists "MECH-S (a slack word)" against the panel (its §6.2 :751) | **the panel: NONE** — the controller's line 2 shows `diag::faultCodeName(lastFault)` (`driver_panel.hpp` :624), so `MECHANISM_SLACK` (15 characters; the line holds 24 bytes, :579) shows with no panel edit, and nothing in `src/` or `include/` switches over `FaultCode` or `MechanismOutcome` outside their own headers (grep, `59d93bb`). **The test: S-B1** appends `MechanismSlack` to its every-code list (:57-63) | the coordinator corrects AF1's row (MECH-S edits no panel file); `driver_panel_test.cpp`: whoever lands second re-reads the list |
| `manipulation/mechanism_outcome.hpp`, `diag/fault.hpp` | none in flight; STEP1-A reads verdicts | S-B1 (append) | append-only; STEP1's on-failure treats SLACK as any failure, and its ST33 already covers a NOT_HOMED / REFUSED start at a trigger's fire — no STEP1 change *(re-check #2: SB15 pins it if STEP1-B has landed)* |
| `tools/robot_tool.py`, `test/robot_tool_agreement_test.cpp` | G3a Parts 1 (merging), 5 (after the panel), 2, 4, T, R1 | *(re-check #1)* T-B2 only (after Part 5); S-A and S-B are ADOPTED by Part 5 | rebase rule 1–2; one tool lane at a time — the coordinator queues them |
| `tools/robot-panel/` (MECHANISMS page) | G3a Parts 3, 6 | T-B2 (the button) | Part 6 first; else Part 6 adds it |
| `Makefile`, `src/build_selection.hpp`, `src/main.cpp`, `tools/src_build_gate.py`, `ci.yml` | G3a Parts 1 (gate), 2 (selection, gate builds), 4; CAL1-B (calibrate messages, gate); *(re-check #10)* **robot one's build plumbing (RC18)** — a second hand-written robot | T-B1 | whoever lands second re-reads; T-B1's Tune branch is a new branch of the ONE chain and its `PROGRAM=tune` row is ROBOT-generic, so the plumbing chunk adds no Tune row (TB12) |
| `docs/guide/18-mechanisms.md`, the HA register, changelog, glossary, README | MECH-A6 (guide 18, register, README), every chunk | every run | the coordinator merges docs serially; a chapter number at the tip |

### 6.3 G3a — Parts 5 and 6 (where the vocabulary plugs in)
G3a's amendment §6 (:1129-1145): the cap and the slack are new members on the Lift (the cap on every motor kind here);
~~"AFTER (likely)" Part 5's tip, each run extends `tools/robot_tool.py` itself (rule 6 (i))~~ *(re-check #1: under the
11-03 order Part 5 is AFTER the panel, so both land BEFORE its tip and **Part 5 adopts them as landed** — :1131-1132,
"+¼–½ per member set")*; Part 6 renders every member FROM `describe`, so the MECHANISMS page shows `.currentCap` and
`.slack` with no page code; the copy-back is a TOOL subcommand and a MECHANISMS-page button, carried by this chunk
(TUNE1-B2, after Part 5). **G3a's question, answered here:** Q24 (b) — the robot file (D9 (a)); G3a works either way.
The bench thresholds enter the robot file "like any stated fact, with provenance" (:1136-1137) — STRING CHECK's COPY
block is that path (by hand until Part 5).

### 6.4 Where this lands in the 11-03 order, and whether it fits before Nov 21 *(re-check #1: re-derived — the draft's section dated D20 (a)'s superseded order)*
The team lead's order of 2026-11-03 ~13:45 puts MECH-S after "competition in flight → AF1 brief, VIS1-A2, ALLY1(+L4)
brief, VIS1-B, AF1 build (start by ~Nov 8)", in the group "L6-A2/B, STEP1, MECH-S, CAL1-B", before CURVE1 and the
panel; "EVERYTHING INCL. FLIPPER before the 21st". G3a's model guessed three runs of ~1 build day (:1240-1242); this
brief's runs are ~12¼–16 builder-days before the panel. From MECH-S-A's build starting ~Nov 8 (G3a's model: a review
round 2 days, a merge ½ day; each build starting when the previous BUILD ends):

| Run | Build (start → end) | Lands (+2½) |
|---|---|---|
| MECH-S-A (2¼–3) | Nov 8 → Nov 10¼–11 | **~Nov 13–14** (before the ~Nov 16 freeze) |
| TUNE1-A (2½–3¼) | → Nov 12¾–14¼ | **~Nov 15–17** |
| TUNE1-B1 (3½–4¼) | → Nov 16¼–18½ | **~Nov 19–21** |
| MECH-S-B1 (2¾–3¾), beside T-B1 | Nov 12¾–14¼ → Nov 15½–18 | ~Nov 18–20½ |
| MECH-S-B2 (1¼–1¾) | from both builds' end, Nov 16¼–18½ → Nov 17½–20¼ | **~Nov 20–23** |
| one lane instead (S-B whole, 3¾–5¼) | Nov 16¼–18½ → Nov 20–23¾ | ~Nov 22½–26¼ |
| TUNE1-B2 | after G3a Part 5 | after the panel |

A slot for MECH-S-A by ~Nov 5 moves every row ~3 days earlier (the slack watch ~Nov 17–20 with two lanes). Every lost
day moves each date about a day. The draft's own arithmetic put its slack watch's low end at ~Nov 22, not the 21 it
wrote. **So:** the cap before the freeze; live tuning just before Nov 21; the slack watch by Nov 21 only with the
second slot and an early start; the copy-back after the panel by construction. ~~Under a strict cross-chunk serial rule on
`service.hpp` (AF1's build from ~Nov 8 for ~8¾–11½ days), only the Tune program could land by Nov 21~~ *(re-check C1:
AF1 does not edit `service.hpp` — its own brief)*: under a strict cross-chunk serial rule on `service.hpp`, the one
other builder is STEP1-B (~3–4 builder-days, after STEP1-A's ~3–4); whichever MECH-S run overlaps its build waits up to
that long, and every later row moves with it — §6.2 asks for the house rule instead.

### 6.5 STEP1, AF1, robot one, L6, CAL1, VIS1, IMU1, MECH-A6
- **STEP1:** a SLACK step is a failed step — STEP1-A's per-step on-failure covers it with no change; *(re-check #2)*
  STEP1-B edits `service.hpp` (D8 (a) of its brief — the check functions); nothing of MECH-S goes into them (§1.5), and
  its ST33 covers a NOT_HOMED / REFUSED start at a trigger's fire (SB15 pins it from this side if STEP1-B has landed);
  its HA block is HA-216..HA-218, this one's HA-219..HA-227.
- ~~*(re-check #2)* **AF1:** its build (from ~Nov 8) is "a generic automatic action in the MECH service" — `service.hpp`;
  its brief (next in the order) should name its regions; whoever lands second re-reads (§6.2).~~ *(re-check C1)*
  **AF1:** its draft brief (13:55) keeps `service.hpp` untouched (its §6.2 :747) and shares `kinds.hpp` (AF1-A's
  `Arm::pieceSensor`) and `test/driver_panel_test.cpp` (AF1-B); its table's "MECH-S (a slack word)" in
  `src/common/driver_panel.hpp` is not so — MECH-S edits no panel file (§6.2). It reserves HA-228..HA-236 after this
  brief's block (D11).
- *(re-check #10)* **Robot one:** its hand-written build plumbing (RC18) is the competition track's; the Tune row is
  ROBOT-generic, so it needs only robot one's mechanism table registered; if the string lift is robot one's, live tuning
  reaches it only once that build exists.
- **L6:** no shared code file (localization/, distance comments); documents only.
- **CAL1:** CAL1-B maps the core's list (N9) — `wiring.hpp`'s readers, not the cap; `test/calibrate_program_rig.hpp`'s
  `CountingProvider` gains `currentLimit` in S-A (forwarding). If CAL1-B lands after S-A it re-reads the rig.
- **VIS1:** S-A follows VIS1-A1's commit (the provider); VIS1-A2/B do not touch the provider's accessors (re-checked at
  S-A's tip).
- **IMU1-B:** station 2's functions only — no overlap.
- **MECH-A6:** this chunk never edits `bench_r3a.*`; the Tune program is its own program and slot (D7).

---

## 7. Checks — the coordinator re-runs every one after each run (RESUMING's step 3: never the report's word)
Every binary under the memory rule. `<scratch>` is the coordinator's scratch folder — never `/tmp`.
```sh
cmake --build build/test -j"$(nproc)"
python3 tools/suite_run.py green
ctest --test-dir build/test -R compile_refusal --no-tests=error --output-on-failure | tail -3
# the ARM header gate — every v2 header in ONE TU
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > <scratch>/all.cpp
echo "int main(){return 0;}" >> <scratch>/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c <scratch>/all.cpp -o /dev/null -Iinclude
# the CI guards
grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib | grep -v '^include/shulib/hal/pros/'   # empty
grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib              # empty
python3 tools/src_build_gate.py check            # every run; T-B1, T-B2: also self-test
python3 tools/robot_tool.py check && python3 tools/robot_tool.py self-test   # every run once G3a-P1 has merged (CI's own guard; re-check #1)
python3 tools/api_doc_tool.py self-test && python3 tools/api_doc_tool.py check-coverage \
  && python3 tools/api_doc_tool.py check-fresh && python3 tools/api_doc_tool.py check-examples \
  && python3 tools/api_doc_tool.py check-removability
python3 tools/doc_staleness_audit.py && python3 tools/briefing_status.py check
# ── the brief's own ──
git diff -U0 HEAD -- include/shulib/hal/motor.hpp | grep -E '^[+-][^+-]'          # empty: F4's IMotor untouched (S-A)
git diff -U0 HEAD -- include/shulib/chassis/routine.hpp include/shulib/chassis/chassis.hpp | grep -E '^[+-][^+-]'   # empty (every run)
git diff -U0 HEAD -- include/shulib/manipulation/mechanism_outcome.hpp include/shulib/diag/fault.hpp | grep -E '^-[^-]'   # S-B: no removed line (append-only)
git diff --stat -- src/programs/bench_r3a.cpp src/programs/bench_r3a.hpp   # empty (MECH-A6's)
git diff --stat -- src/                                   # S-A, T-A, S-B1, S-B2: empty (re-check C4); T-B1: tune_program.* and main.cpp, build_selection.hpp only
grep -rn 'IDeviceProvider' include src test | grep -E 'public|final' # every implementer answers currentLimit (S-A)
grep -rnoE 'HA-(219|22[0-7])' include src test | sort -u           # every id has its row; every row a code site
grep -rn 'PROBE_\|p_string\|p_cap\|SlackWatch' include/shulib test  # empty (no probe code leaked)
# T-B1: the match builds carry no live tuning — the gate's step 8 (extended by T-B1) builds every program and reads
# each linked ELF's symbols (`arm-none-eabi-nm -C --defined-only bin/hot.package.elf`, tools/src_build_gate.py :128):
# Competition, Drive and Auton Tester: no LiveTuning / tune_program symbol; Tune: no chassis. Then by hand, once:
make -s ROBOT=tank PROGRAM=competition >/dev/null && arm-none-eabi-nm -C --defined-only bin/hot.package.elf | grep -c 'LiveTuning\|tune_program'   # 0
```
Plus the spot checks — two mutations per run re-run by hand (seen RED, restored, the suite green again):
- **MECH-S-A:** MA10 (the re-check fights any changed value — *re-check #8*) and MA4 (the plant ignores the limit);
  robot two's boot lines byte-identical when both claw motors read 2500 mA.
- **TUNE1-A:** MT1 (P6's +0.0000 V) and MT6 (the copy block's derived members).
- **TUNE1-B1:** MB1 (one button not a STOP) and MB6 (a dropped controller ignored — *re-check #7*); MB5 (the gate's
  no-tuning check) by the gate's self-test.
- **MECH-S-B1:** MS1 (the deceleration window) and MS11 (the switch-homing guard — *re-check #4*).
- **MECH-S-B2:** MS13 (the minimum gap — *re-check #5*).
- **TUNE1-B2 (after the panel):** MC2 (a cut block accepted).

---

## 8. Landmines
1. **The cap is PER MOTOR.** Two motors on one spool pull with twice the cap; the boot line says the total.
2. **A signature at or above the cap never trips** (P10, P11) — the build refuses it; near the cap a real reading may not
   reach it (HA-223): leave a margin.
3. **Always check the limit at boot** (HA-220): a Tune session's 1.0 A must never ride into the Competition program
   unseen; *(re-check #8, D12 (b))* a mechanism with no cap REPORTS a stray limit and writes nothing until Step 0.4 has
   measured what a 10-motor robot reads (HA-221) — the Tune program restores what it changed on its own exits.
4. **The host test plant stops a joint dead in one step** (`mechanism_plant.hpp` :228-251): the first sample after an
   impact reads what the volts commanded before it (P10's 2.19 A "impact sample" under a 1.2 A cap in BOTH models). No
   test may claim the pull at an impact; the cap does not limit momentum (HA-222).
5. **A rotation sensor on the spool's shaft is blind to a snap** (P3). And a sensor the CARRIAGE moves must never be the
   angle-sensor role: the position loop would chase a fallen carriage and wind the broken string.
6. **The slack watch's phases are load-bearing:** in the deceleration a whole string draws almost nothing (P8's false
   trips); in the acceleration an unhooked spool draws as much as a loaded cruise (P12). Cruise and hold only.
7. **Blind while lowering and at the bottom** (P8, P12; HA-225) — the guide says so in its first sentence about the watch.
8. **A stall-home DOWN on a gravity-return string lift pays out string** (P5: 6.4 in) — refused.
9. **Today a snapped string is invisible** (P1: SUCCEEDED, 20.08 in vs 0.00 in) — until MECH-S-B lands, the guide's
   string-lift section says so.
10. **The PIDs copy their gains at bind and the jog's soft stop its limits** (P6): a live change without `rederive` changes
    nothing — or worse, half of the numbers.
11. **Presets are found by value** (P7): never live.
12. **0 in `.tuning` means "the starting value"** (P9): the page never sets kP, kI, kD, kV, top speed, acceleration,
    within or settle to 0; and *(re-check #14)* the COPY block writes the SMALLEST set that builds to the live spec — a
    stated top speed alone makes the file derive a new acceleration, so the block carries the acceleration too; but it
    never states a derived number that would re-derive the same (a stated kV goes stale on a cartridge change).
13. **The build check does not police gains** (P9: NaN, negative kS/kA, kV 1e9, 500 in/s accepted); a NaN gain would stop
    the program at the PID's constructor (`control/pid.hpp`'s precondition). The live rules add the bounds.
14. **A pure virtual on `IDeviceProvider` breaks every implementer at merge** — VIS1-A1's merged tree did not build
    (2026-10-30) because `CountingProvider` lacked `aiVision`. List them at the tip; the merge's build is the gate.
15. **STRING CHECK's UNHOOKED run with a loose string end:** the spool whips it. The page says: take the string off the
    spool or tape its end, carriage on its stop, hands clear.
16. **A `[TUNE]` line over 200 bytes is CUT by TermSink** — the import refuses a block whose end line or count is wrong;
    the test pins every line's length at the longest names and numbers.
17. **The Tune program must never build the drive** (TB5): a robot on blocks is not assumed.
18. **The cap limits acceleration too:** too low a cap slows moves or times them out — the TEST report shows each move's
    peak; the guide's recipe sets the cap above it.
19. **`MechanismOutcome` and `FaultCode` are append-only**; `test/driver_panel_test.cpp`'s every-code list and any
    exhaustive switch gain the new ones.
20. **The once-a-second re-check runs INSIDE a tick** (never its own task); a starved service does not re-check — the
    firmware keeps the cap meanwhile (that is D1's point).
21. **Never two of THIS chunk's builders in `service.hpp`** — S-A, T-A and S-B1 are serial; a run that rebases re-reads
    the merged form. *(re-check #2, C1)* STEP1-B edits it in the same weeks (AF1 does not — it shares `kinds.hpp`):
    whoever lands second re-reads, the merged build and the full suite are the gate (§6.2).
22. *(re-check #3)* **Four currents in order:** heaviest hold (with a piece) < stall signature < cap − 0.2 A; heaviest
    move's peak < cap; cap ≤ the string's safe pull. A signature under the hold trips STALLED on an ordinary hold (V1).
23. *(re-check #9)* **A string pulls, never pushes:** an acceleration above g leaves slack and snatches it taut (R4:
    0.58 in and 21 in/s at 1.25 g) — refused at build on a `.slack` lift, warned above 0.5 g.
24. *(re-check #4)* **A switch that never reads pressed** (a dead 3-wire switch reads a level — HA-184) lets a downward
    homing push unwind string to its bound (R1: up to 7.02 in) — guarded at 1 in on a `.slack` lift.
25. *(re-check #5)* **A carriage something else carries reads as SLACK** (V3) — the string IS slack; D4 (e) keeps the
    lift usable; `offBottom` comes from the bench with a margin.
26. *(re-check #7)* **The Tune program's abort set is the Auton Tester's** (button, controller lost, disabled) and
    nothing is made before the clear confirm — robot two's pneumatic claw moves when its line is made.
27. *(re-check #7)* **Never re-clamp a hold by `holdAt`** — a jump to a new target asks up to 153 V; narrowing past the
    held target is refused instead.

---

## 9. Summary

**What MECH-S + TUNE1 are:** a string lift that cannot be pulled harder than the team says, that knows when its string
has gone slack, and whose numbers can be tuned on the robot and carried back into the robot file. **The cap** (MECH-S-A):
`.currentCap = {.amps = …}` per motor on any motor mechanism; the motor enforces it (PROS's per-motor limit, through a new
sibling seam beside the locked `IMotor`), checked at every boot and once a second; every stall signature must be below it.
**The slack watch** (MECH-S-B1/B2): on a lift declared `.slack`, low current while cruising up or holding off the
bottom (a push held at the top counts) — two thresholds measured by STRING CHECK (string on vs unhooked, three passes
each, midway, refused unless a clear gap), two numbers derived from the same run, two starting values said as such; a
trip ends the step SLACK, raises `MECHANISM_SLACK`, puts the lift in its safe state and UN-HOMES it (auton steps
NOT_HOMED, the driver at `.unhomedVolts`); blind while lowering, at the bottom and to one string of two; a carriage
something else carries reads as slack; a downward switch-home guarded at 1 in; acceleration kept below gravity's.
**Live tuning** (TUNE1-A/B1, and B2 after the panel): "shulib Tune" (slot 7, mechanisms only, any robot with a table)
changes gains, speeds, "reached", gravity, the stall, homing, cap and slack numbers and narrows the limits (never past
the held target) — only between steps, only inside the build's rules plus the live bounds, never to a value the file
cannot say; the Auton Tester's abort set and a clear confirm; UNDO/RESET; nothing persists; COPY prints the smallest
`[TUNE]` block to serial and the SD card (appended); copied by hand until IMPORT TUNING arrives with the panel.

**Decisions:** **D1** the motor's own limit (firmware) · **D2** per motor, the team's number, no default, a string lift
without one warned, every signature below it, *and the four currents in order* · **D3** the slack watch in two phases
(cruise up, hold off the bottom — a push held at the top counts), two thresholds measured, two numbers derived, two
starting values, a minimum gap · **D4** *(e)* SLACK: stop, say, UN-HOME; the auton step fails; the driver keeps it at
its unhomed volts; the drive carries on · **D5** no stall-home down on a string lift; a downward switch-home guarded at
1 in; the cap protects homing up; the homing signature below the cap; acceleration below g · **D6** a sensor the
CARRIAGE moves (a distance sensor), not the spool's shaft — after the panel (MECH-S-C) · **D7** its own bench program,
never in a match, robot-generic, the Auton Tester's abort set · **D8** the page, what is live (not presets, no flywheel
kA), UNDO/RESET, nothing persists · **D9** the smallest COPY block in the log (appended), by hand until the panel's
import · **D10** order cap → live tuning's library → the Tune program beside the slack watch → STRING CHECK; the copy-back
after the panel; the cap ~Nov 13–14, live tuning ~Nov 19–21, the slack watch ~Nov 20–23 with a second slot (~Nov 22–26
in one lane) · **D11** HA-219..HA-227 · **D12** *(b)* a no-cap motor's stray limit reported, not written, until Step 0.4.

**Sizes (builder-days, rough):** S-A ~2¼–3 · T-A ~2½–3¼ · T-B1 ~3½–4¼ · S-B1 ~2¾–3¾ · S-B2 ~1¼–1¾ — **~12¼–16 before
the panel**; T-B2 ~1¼–1¾ after it — **~13½–17¾ in all**.

**Files:** NEW `hal/motor_current_limit.hpp`, `mechanism/{live_tuning,tune_panel,tune_program,slack_bench}.hpp`,
`hal/pros/tune.hpp`, `sim/string_carriage.hpp`, `src/programs/tune_program.{hpp,cpp}`, ~12 test files; CHANGED
`hal/pros/motor.hpp`, `hal/fake/fake_motor.hpp`, `hal/device_provider.hpp` + `hal/{pros,fake}/…device_provider.hpp` +
`test/calibrate_program_rig.hpp`, `mechanism/{devices,spec,kinds,service}.hpp`, `robot/mechanism_parts.hpp`,
`sim/mechanism_plant.hpp`, `control/pid.hpp`, `manipulation/mechanism_outcome.hpp`, `diag/fault.hpp`, the PROS shim,
`Makefile`, `src/build_selection.hpp`, `src/main.cpp`, `tools/src_build_gate.py`, `ci.yml`; after the panel (T-B2)
`tools/robot_tool.py`, `tools/robot-panel/`; documents per §3.

**Probes** (`.claude/coordinator/scratch/brief-MECHS-TUNE1/`, `build.sh`, outputs in `out/`): P1 (a snap is SUCCEEDED
today), P2 (currents by phase), P3 (the spool-shaft sensor is blind), P5 (stall-homing down pays out string), P6 (half of
the numbers are copied at bind), P7 (presets by value), P8 (the phase-aware watch; the deceleration's false trips), P9 (the
build check and gains), P10 (the cap under a starved service; a signature above the cap never trips; the plant cannot
judge an impact), P11 (the cap during homing), P12 (noise; cruise-only; when a lift can be told apart); *(re-check)*
R1 (a dead switch's homing push pays out string), R3 (a push held at the top soft stop, before and after a snap), R4 (a
one-sided string above g), and the checker's V1/V3/p_plantlimit re-run (`recheck/`).

---

## The decisions, as I would put them to the team lead (plain words) *(re-checked 2026-11-03 ~14:00; point 10 ~16:00)*

1. **The cap: should the MOTOR enforce it, or our code?** I recommend the motor. Our code only runs when something ticks
   it. On the host test plant, a student's stuck loop let a 12 V push into an obstacle pull 2.5 A for over a second
   under our own cap. The motor's own limit keeps working when our code is stuck. (On the plant that half is a model of
   the motor, not a measurement; the bench checks it.) The cost is a small new interface beside the locked motor one,
   and three things to check at the bench: how fast the motor enforces it, whether a limit from one program carries
   into the next, and how it mixes with VEXos's own sharing on a 10-motor robot.
2. **The cap's number: per motor, yours, no default?** I recommend yes. Two motors on one spool pull twice as hard, and
   the boot line will say so. **New:** four currents have to stay in order. The heaviest hold, with a game piece, must
   be below the jam detector. The jam detector must be below the cap, with about 0.2 A to spare. The heaviest move's
   peak must be below the cap. On the test plant, a jam detector set below what the lift drew holding a piece stopped
   the lift as if it were jammed. The Tune program shows the hold and the peak, and warns when the order breaks.
3. **The snap detector: watch only while the lift climbs steadily and while it holds off the bottom?** I recommend yes.
   A button held at the top counts as holding. The STRING CHECK page runs the lift three times with the string on and
   three times unhooked, and sets each threshold halfway between. If the gap is too small for the noise, it refuses and
   leaves that half off. Two timing numbers can't be measured that way, so they start at library values and the boot
   line says so. **It cannot see** a snap while the lift is coming down, at the bottom, or of one string out of two.
   **New:** a carriage resting on something else (a goal, or the floor under a stretched string) looks like a snap.
4. **When it fires: CHANGED.** I now recommend: stop the lift, say SLACK, and make it **forget its height**. Auton steps
   on it fail until it is homed again, and the driver can still move it slowly, by sight. For that, a string lift must
   state its "unhomed volts". Before, I recommended locking it until the program restarts. But a false alarm (point 3)
   would then cost you the lift for the rest of the match. On a real snap, the new way loses nothing extra.
5. **Homing: forbid "push down until it stalls" on a string lift?** I recommend yes; the spool unwound 6.4 in on the
   test plant. **New:** a push down onto a switch that never clicks (broken or unplugged) unwound up to 7 in on the
   plant. It will now stop after about 1 in. Pushing UP into a top stop is safe under the cap. **Also new:** a string
   lift must not speed up or slow down faster than gravity, or the carriage leaves the string and then snatches it
   tight. The build refuses that on a string lift and warns above half of gravity.
6. **The "stronger" sensor: the correction stands.** A rotation sensor on the spool's shaft can't see a snap, because
   the spool keeps turning. Only a sensor the carriage moves can, for example a distance sensor aimed at the carriage.
   I recommend current-only now, and the carriage sensor as a small run after the panel.
7. **Live tuning: its own program, "shulib Tune" in slot 7, never part of a match program?** I recommend yes. **New:**
   it has the Auton Tester's safety. Nothing moves without a connected controller. Any button, a dropped controller or
   a disable stops everything. The screen asks "hands clear?" before anything switches on. It also builds for ANY robot
   with mechanisms. That matters because no robot file has a string lift yet: if it is robot one's, live tuning reaches
   it only once robot one has a build.
8. **The Tune page.** Pick a mechanism, change a number, then TEST to see the time, overshoot, settle, peak current and
   the hold current. Presets are not changed live, and a limit can't be narrowed past where the lift is holding (move
   it first). UNDO and RESET undo changes, and nothing is kept after the program ends.
9. **Getting the numbers into the robot file.** COPY writes the smallest set of numbers that rebuilds exactly what you
   tuned, so numbers you never touched stay automatic. The copy is kept on the SD card across reboots. **New:** the
   panel now comes after the 21st, so its import button does too. Until then, a student copies the lines into the
   mechanism table by hand.
10. **Order and dates, on today's order.** The order: the cap, then live tuning's library, then the Tune program. The
    snap detector builds beside the Tune program if a second slot is free, and its STRING CHECK page follows. The
    copy-back comes after the panel. If the cap starts ~Nov 8:
    - the cap ~Nov 13–14, before the freeze;
    - the Tune program ~Nov 19–21;
    - the snap detector ~Nov 20–23 with the second slot, ~Nov 22–26 without.

    Starting by ~Nov 5 makes each about 3 days earlier. That is about 12–16 builder-days before the panel.
    *(re-check C1)* STEP1's second run edits the same mechanism file in the same weeks, in different places. (The
    flipper's brief says it does not touch that file.) I recommend the usual rule: the second to land re-reads.
    Taking turns could add up to 3–4 days to every date after the overlap.
11. **Three questions.** (a) Does "MECH-S" in today's list include live tuning? I assumed yes. (b) Which robot has the
    string lift, and when will its first build exist? (c) Is it pulled up by string and lowered by gravity? How many
    strings and stages does it have, what string is it, and how strong is it rated?
12. **New: motor limits at boot.** Suppose a motor with no cap reads something other than its 2.5 A default at boot.
    Should the library reset it? I recommend only reporting it, for now. On a 10-motor robot like robot two, that
    reading might be the brain's own power sharing. Resetting it every boot would change the competition program on a
    guess. Robot two's boot with all 10 motors plugged in settles it. Meanwhile, the Tune program puts back any limit
    it changed when you leave it normally.
13. **Hardware beliefs HA-219..HA-227**, right after STEP1's. Nine of them, none measured, each with the bench check
    that settles it.

---

## Re-check — amendments (2026-11-03 ~14:00)

A checker returned 16 findings on the draft (13:18). Each was RE-VERIFIED here against the tree, the worktrees, the
notes and probes: the checker's `check/p_verify.cpp` and `check/p_plantlimit.cpp` rebuilt at the current tree and re-run
under the memory rule — **byte-identical** to the checker's outputs (`recheck/out/p_verify.txt`, `p_plantlimit.txt`) —
and a new probe `recheck/p_recheck.cpp` (R1, R3, R4; `recheck/out/p_recheck.txt`). Every correction is in the body,
marked *(re-check #N)*. 15 REAL and fixed; 1 PARTLY REAL (one half fixed, one half not adopted, with the evidence).

| # | Sev | Finding | Verdict | Evidence re-verified | Disposition |
|---|---|---|---|---|---|
| 1 | blocker | The schedule rests on the order replaced today | **REAL** | `NOTES.md` :905 (~13:30) and :906 (~13:45) post-date the draft (13:18); `g3a-p1`'s `tools/robot_tool.py` :29-30, :244-253 refuse `mechanisms` by name until Part 5; Part 5 deferred; the draft's own sum put S-B's low end at ~Nov 22 | FIXED: header (the 11-03 words binding, D20's struck); D10 re-derived (two consequences, Q1–Q3, the freeze, the shared file); §2.1 (six runs, S-B split, T-B2 after the panel); §6.4 (a dated table, from Nov 8 and Nov 5); §1.4; Part 5 adopts `.currentCap`/`.slack` (§1.1, §1.5, §6.2, §6.3, §3.3 — G3a :1131-1132); the rule 6 (i) shares dropped; `robot_tool.py check`/`self-test` in §2.3 and §7; sizes re-derived (header, §9) |
| 2 | major | STEP1-B's and AF1's `service.hpp` edits missing; the latch belongs in STEP1-B's check function | **PARTLY REAL** | STEP1's brief :193-197, :842; `build-order.md` :1783 (AF1 "in the MECH service"); the 11-03 order puts both before or beside MECH-S. BUT STEP1's check functions hold PRECONDITIONS that THROW (its ST30; its re-check row 9: "arguments + spec + peer(), none run-dependent"); the latch is run-time state answered by a VERDICT in `beginStep` (`service.hpp` :357-382); STEP1's ST33 already handles a REFUSED / NOT_HOMED start at a trigger's fire; STEP1's own re-check #1 removed exactly such a mid-match throw | FIXED: §6.2 rows (STEP1-B, AF1; each run's region named; within the chunk serial, across chunks the house rule), §6.5, landmine 21 — *the AF1 half CORRECTED at the continuation (C1): AF1's own brief says it does not edit `service.hpp`; it shares `kinds.hpp`*. **NOT ADOPTED:** the latch inside the check function — §1.5 says why; SB15 pins the trigger-at-fire behaviour if STEP1-B has landed |
| 3 | major | The hold < stall < cap order is missing; false STALLs follow | **REAL** | V1 re-run: a 2.5 V piece → hold 1.08 A over a 1.0 A signature → STALLED; 5.4 V → STALLED at 0.52 s, fell to −0.01 in; the stall watch runs on the hold (`service.hpp` :84-89, :985-995); guide 18 :258-262 says it only for a squeezing hold | FIXED: D2 (the ordering, V1's numbers); the boot Warn within 0.2 A (D2, §1.1, CA2, MA12, HA-223); the TEST report's hold current and warning (§1.3, TB11); Step 0.5; CA15; guide 18's text (§3.1); landmine 22; plain words 2 |
| 4 | major | Switch homing pays out string when the switch never reads pressed | **REAL** | V2 re-run (2.9 in at 1.2 V); R1: 2.68 / 3.64 in from 6 in, 7.02 in from the bottom; only the stall signature watches a switch push (`service.hpp` :1110); HA-184 | FIXED: D5 (a) — a deterministic 1 in travel guard on a `.slack` lift; the checker's current-based "landed" stop MEASURED (R1: 0.31–0.42 s, 0.59–0.68 in) and not chosen (needs a measured threshold; the 1 V descent dipped to 0.060 A under 0.104 A); its "bound > travel time" alternative measured useless (never acts inside 4 s); the `spool()`-without-`.slack` boot Info; SB11, MS11, landmine 24 |
| 5 | major | D3's numbers lack sources; a supported carriage false-trips; some snaps missed; no minimum gap | **REAL** | (a) the bench derived only the two amps; (b) V3 re-run: 2.79–2.80 s at the unhooked level; (c) the jog's soft-stop hold is `Mode::Jog` (`service.hpp` :1061-1085) — R3: 0.146 / 0.354 A whole vs 0.043 / 0.062 A snapped; (d) the draft's rule accepted any gap > 0 | FIXED: D3's headline and each member's source — two MEASURED, two DERIVED (`aboveMotorRpm`, `offBottom` with holds approached from above and below), two STARTING values said as such (the checker's "longest dip × k" cannot work: a midway threshold sits below every ON sample the bench saw — D3 says why); the minimum gap max(0.03 A, 4σ̂) (HA-224); three passes each; the soft-stop hold watched (SB12, MS12); the supported carriage and one string of two (D3, SB13, landmine 25, guide 18 / 14); Step 0.1; SB4 widened |
| 6 | major | The latch never recovers; a false trip's cost unweighed | **REAL** | D4 (a)'s text; a `restsAtStop` lift is not re-zeroed at run time (`service.hpp` :850-851); V3's false trips | FIXED: D4 **(e)** — un-home on SLACK (the existing NOT_HOMED path :791-795 and the unhomed jog :1043-1046) — now RECOMMENDED, weighed against (a) with the false-trip cases; its condition (`.unhomedVolts` defaults to 0 — `kinds.hpp` :837 — a boot line); §1.5, SB5, MS4, plain words 4 |
| 7 | major | Tune ignores a dropped controller, a disable, a field; no clear confirm; `holdAt`'s jump; no TEST bound | **REAL** | `auton_tester_program.cpp` :43-46; `auton_abort_watch.hpp` :20-26, :50-56; `mech-a6`'s `bench_r3a.hpp` :244-247; `holdAt` :1152-1161 (8 in × 76.4 °/in × 0.25 = 153 V) | FIXED: D7 (a) the abort set, the refusal without a controller or with a field, the two-stage confirm, the TEST bound; D8 narrowing never past the held target (§1.2 strikes the re-clamp; TA7 re-written; MT8); §1.3; TB7–TB9, MB6, MB7; landmines 26–27 |
| 8 | major | Resetting uncapped motors changes robot two's match program | **REAL** | HA-172 (`docs/hardware-assumptions.md` :391 at `9d77c9d`); MECH-PANEL :247-249; robot two runs 10 motors; the draft's "does not change" was unconditional | FIXED: NEW **D12** — (b) report, write nothing, until Step 0.4 (RECOMMENDED); Tune restores what it changed on its exits; the re-check re-writes only a motor back at its default; robot two's boot conditional (§1.1, CA14); the programs a Tune limit can reach named (station 13, slot 5); HA-220/221 rows; CA4, CA10, MA1, MA10, TB10; landmine 3 |
| 9 | major | Nothing keeps a string lift's acceleration below gravity's | **REAL** | `kinds.hpp` :605 (top speed in 0.25 s); R4 (kinematics only): none at 0.9 g, 0.58 in / 21 in/s at 1.25 g, 1.44 in / 33 in/s at 2 g; `service.hpp` :759 | FIXED: D5's acceleration paragraph; build refusal above g on a `.slack` lift, boot Warn above 0.5 g, the starting value capped at 0.5 g, the live refusal (S-B2); the one-sided string carriage (SB8); SB1, SB16, MS14; landmine 23. The host test plant's rigid joint cannot show slack — the fixture must |
| 10 | major | Live tuning cannot reach the string lift (robot one has no build) | **REAL** | `mechanism_table.hpp` :113-135; every program after `tester` is `ROBOT=tank`-only (`Makefile` :169-189); RC18 (G3a :1245-1246); NOTES records no plumbing launch | FIXED: D7 (a) — a ROBOT-generic `PROGRAM=tune` row (§1.3, TB12); §6.2's Makefile row; §6.5; D10 Q2; before Nov 21 live tuning reaches only robot two's claw pivots unless robot one builds |
| 11 | minor | Citations said `eab0f89` for files VIS1-A1 changed | **REAL** | `IDeviceProvider` :117 → :132, `readFaults` :196 → :221, `ProsDeviceProvider::readFaults` :516 → :571, HA rows :316/:381/:391 → :326/:391/:401 | FIXED: the header names the files cited at `9d77c9d` (VIS1-A1 has since COMMITTED); §0's heading |
| 12 | minor | P12's latency after a snap while lowering misread | **REAL** | `out/p_slack_noise.txt`, 1 V σ 0.02, cruise only, hold refused: holding 5.70 s (worst 5.71), mid-down 3.48 s (worst 3.50) | FIXED: D3 and §0.4's P12 line |
| 13 | minor | "Measured" overstated for the modelled firmware cap | **REAL** | `probes/p_cap.cpp` :6-11 — the probe's own 1-ms clamp | FIXED: D1 (a) (modelled; the measured half is that a starved software cap does nothing), plain words 1 |
| 14 | minor | Flywheel kA; zero-start numbers; COPY freezes derived values; the SD log truncated | **REAL** | `SpeedTuning` = kS, kV, kP, kI (`kinds.hpp` :429-438); `p.kA = t.kA` (:603); `log_copy_line.hpp` :7-10; `ProsFileMode` (`hal/pros/device_provider.hpp` :220-231 at `9d77c9d`) | FIXED: D8 (no flywheel kA; kA's additive first step), D9 (the smallest block, the staleness cost, Append), §1.2, §1.3, TA8–TA10, MT6, MT9, MT10, TB10, landmine 12 |
| 15 | minor | The plant's default-limit byte-identity is false on a reversal | **REAL** | `p_plantlimit` re-run: |V − kV·ω| 18.26 V, 2 ticks past 12 V | FIXED: the limiter is bypassed at the default (§1.1); CA6, MA11; §5 (the plant's reversal torque is an existing inaccuracy, not this run's) |
| 16 | minor | D6's correction not carried into G3a's plan | **REAL** | `docs/internal/chunks/G3a-robot-profile-generator-panel.md` :218, :1131; the coordinator's copy :193, :1106; memory `robot-config-panel-g3a.md` :37 | FIXED: §3.3 lists them for the plan commit; the memory note is the coordinator's to correct (this writer writes only the brief) |

**What stays UNVERIFIED (nothing here has run on a robot):** every HA-219..HA-227 belief — above all whether a real
motor holds its limit with nothing ticking (HA-219), whether a limit persists across programs (HA-220), what a
10-motor robot's read-back shows (HA-221, which decides D12), the V5 current's noise (HA-224, which decides whether any
real lift separates at all), and whether the carriage rests on its stop at program start (HA-226). The new library
numbers are INVENTED and labelled so: the 1 in homing guard, the 0.5 g margin, the 0.03 A / 4σ̂ minimum gap, the two
slack lastings, the 0.2 A cap Warn, the TEST bound. R4 is kinematics with no friction; R1, R3, V1, V3 are on the host
test plant, whose constants are invented (HA-183). The dates rest on rough sizes, on G3a's review model, on MECH-S-A
getting a slot by ~Nov 8, and on the coordinator not serializing `service.hpp` behind ~~AF1's build~~ STEP1-B's
*(C1)*. Whether the team lead's "MECH-S" includes TUNE1, and which robot carries the lift, are his answers (D10 Q1–Q3).

### Continuation — the fix round finished (2026-11-03 ~16:00, after the ~14:50 usage stop)

The stopped run had written every row above and every body correction by 14:07 (the brief's last write); nothing was
half-done in the file. This run RE-VERIFIED the dispositions rather than trusting them: the checker's V1/V2/V3 and
`p_plantlimit` outputs and the re-check's copies compared (`cmp`: identical); R1/R3/R4's numbers read against
`recheck/out/p_recheck.txt` (each as quoted; V2's load is 1.2 V, `check/p_verify.cpp` :141-145); every mechanism
`file:line` the fixes lean on re-opened — `service.hpp` :791-795, :848-852, :1043-1046, :1110, :1152-1161; `kinds.hpp`
:429-438, :601-606, :837 — and `git diff --stat eab0f89 59d93bb` is EMPTY for `include/shulib/mechanism/`,
`manipulation/`, `sim/mechanism_plant.hpp`, `src/programs/`, `src/robots/` and the `Makefile`, so every such citation
holds at main's tip; STEP1's brief :193-197, :842, :879, ST30, ST33 and its re-check row 9 as quoted; `g3a-p1`'s
`tools/robot_tool.py` :28-31, :244-253 and `ci.yml` :104-105; G3a :218, :1129-1133; the memory note's line 37. All 16
dispositions KEPT. Four corrections, from what changed or was missed:

| # | What | Evidence | Disposition |
|---|---|---|---|
| C1 | Finding 2's AF1 half rested on `build-order.md`'s one-line description ("a generic automatic action in the MECH service"); AF1's own draft brief says the opposite | `AF1-toggle-auto-flipper.md` (13:55): §6.2 :747 "**AF1 does not edit it**", §7 :812 its diff of `service.hpp` empty; AF1-A's files :614 (`kinds.hpp` one member, `driver_inputs.hpp`, `teleop_loop.hpp`, a NEW `auto_action.hpp`); its §6.2 :751 lists "MECH-S (a slack word)" in `src/common/driver_panel.hpp` — but line 2 shows `faultCodeName` (:624; 24-byte line, :579) and no `switch` over `FaultCode`/`MechanismOutcome` exists outside their headers | FIXED: D10's shared-file paragraph, §6.2 (the service row's AF1 → `kinds.hpp`; a NEW panel/test row), §6.4's last sentence, §6.5's AF1 bullet, landmine 21, plain words 10, the paragraph above. The strict-serial cost is now STEP1-B's ~3–4 days, not AF1's build; the recommendation (the house rule) stands on a smaller reason |
| C2 | The header and §6.1 said L6-A1 was staged and STEP1's brief a draft | `git log`: `59d93bb` L6-A1; `NOTES.md` :910-911 (STEP1 "Accept all, go", HA-216..HA-218), :918 | FIXED: header, §6.1 |
| C3 | D11 kept a contingency STEP1's acceptance closed, and missed that AF1 reserves the block after this one | `NOTES.md` :910-911; AF1's brief :41, :263 (HA-228..HA-236 "reserved AFTER … MECH-S/TUNE1's HA-219..HA-227") | FIXED: D11 |
| C4 | §7's `src/` check named S-A and T-A only | §1.5: S-B1/S-B2 touch no `src/` file (the page is the library's `tune_program.hpp`) | FIXED: §7 |

**No probe was re-run beyond the comparisons above** — no new evidence was needed for C1–C4 (each is a reading).
