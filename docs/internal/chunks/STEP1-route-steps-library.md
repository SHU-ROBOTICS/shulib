# STEP1 — care and timing on every auton step: an arrival tolerance and an on-failure choice per step (STEP1-A), and triggers partway through a drive (STEP1-B) — the route maker's library half

> **THE TEAM LEAD'S ANSWERS (2026-11-03)** to D1–D8: **"Accept all, go (Recommended)"** — STEP1-A on his go now,
> STEP1-B after A's build ends. So: D1 `r.onFail(carryOn)` / `r.onFail(skipTo("x"))` / `r.label("x")` before the step,
> locked the day they land (the trigger words stay unfrozen with the other mechanism steps); D2 an arrival tolerance
> loosens the distance AND the settle speed in proportion; D3 skips go forward only (a step behind stops the auton and
> warns at the skip's line; a name that never comes runs nothing more; time running out stops); D4 a wait for a start
> a skip jumped fails, counted, "its start was skipped" — never a throw, never an older run's answer; D5 "DONE, 2
> FAILED" in amber ("DONE, 9+ FAILED"), "FINISHED" for a clean run; D6 a drive that arrives just short fires a trigger
> only if its mark is on the drive (no further than its end plus its tolerance); D7 "60 %" is distance along the
> straight line from the drive's start (the angle turned on a turn), and a refused start lets the drive finish and
> counts as failed; D8 a trigger written wrong is caught on its own line, every run (STEP1-B edits
> `mechanism/service.hpp` — the second of STEP1-B, MECH-S and AF1 to land re-reads it). Plan change approved: G3a Part
> R1 builds without triggers; R1-T after STEP1-B merges. HA-216..HA-218 reserved. Since the same day's orderings, STEP1
> comes in the auton library, ahead of the panel.

> **Status 2026-10-30: DRAFT for the team lead's go. WAITS for his answers to D1–D8 below, then for a go per run
> (STEP1-A, then STEP1-B). Nothing is built.**
> **Re-checked 2026-11-03** against a checker's fifteen findings (five major, ten minor): all fifteen hold up, two of
> them only in part. Each is fixed in the body where it applies, marked *(re-check #n)*, and listed with its disposition
> in **"## Re-check — amendments"** at the end. What changed for the team lead: **D5's word is now "DONE, 2 FAILED"**
> (it was "FINISHED, 2 FAILED"), **D6 is narrowed**, and **D8 is new**. D1, D3, D4 and D7 are now spelled out more
> exactly.
> Execution brief written by the coordinator's brief-writer from a READ-ONLY pass over `shulib-v2` at **`eab0f89`**
> (main; the G3a plan commit of 2026-10-30), against the brief of record
> `docs/internal/chunks/G3a-robot-profile-generator-panel.md` — its answers block of 2026-10-30 (:3-26: D13 (c),
> D14 (a), D15 (a), "Accept all"), its evening amendment's R.1-R.14 (:482-1090; R.13 at :977-1074 is this chunk's
> scope as G3a wrote it), D13-D16 and the checker's C1-C16 table (:1475-1492; C2, C3, C9, C10 are this chunk's).
> Worktrees in flight were read, never edited (§6). **Probed on the host test plant, not argued:** seven probes on
> main's REAL `Routine`, `Chassis`, `MotionScheduler`, `RunGuard`, `runAuton` and `MechanismService` (§0.4,
> `.claude/coordinator/scratch/brief-STEP1/`). **Three probe findings change G3a's R.13 as written:** (1) the arrival
> tolerance as R.13 words it ("applies them to `translationSettle.maxError`") does NOTHING on a drive leg — the
> settle's SPEED bound, not its distance bound, decides when a leg ends (P3: 1.80 s at 0.5, 1, 2 and 4 in alike) —
> D2; (2) a motion TICKS once at the deadline instant before the guard cuts it (P6), so a partway trigger must ask
> the guard or it can start a mechanism after the buzzer — ST29; (3) a per-tick observer built the obvious Tier-3 way
> (`async` + `waitUntil`) paces once more after the exit tick than the blocking verb (P5), so it is not bit-identical
> — §1.2. Five smaller corrections to R.13 are in §0.3.
> **Sizes (rough):** STEP1-A **~3–4** builder-days (R.13: ~2¾–3½), STEP1-B **~3–4** (R.13: ~2¼–3); **~6–8**
> in all (REMAINING-WORK row 20c: ~5–6½). *(re-check: was ~3–3¾ / ~2½–3¼ / ~5½–7. STEP1-B's +½–¾ is the
> trigger's own fire path, the arms split by kind, and D8 (a); STEP1-A's +¼ at the top is the extra tests.)* **HA: HA-216..HA-218 reserved** (§4; next free HA-216 at `eab0f89`, and two
> in-flight runs are told the same — the collision rule is in §4).
> **Shape:** the house style of `docs/internal/chunks/IMU1-imu-rate-once-per-tick.md` and `L6-wall-reset.md`.

---

## DECISIONS FOR THE TEAM LEAD

The team lead's answers of 2026-10-30 already settled the WHAT: per step, "stop the auton / skip ahead to a named later step /
carry on", the deadline always stopping (D14 (a)); a trigger partway through a drive that never fires if the drive
ends early (D15 (a)); routes become generated `Routine` code (D13 (c)). These eight are the HOW questions those
answers leave open (D8 added at the re-check). Each has a recommendation; nothing below decides one silently.

**D1 — Where the per-step choice lives: a Routine member, an options struct, or both. RECOMMENDED (a).**

- **(a) RECOMMENDED — Routine members, written in front of the step they apply to:**
  ```cpp
  r.onFail(carryOn).driveTo(-24_in, -36_in, {.timeout = 2.5_s, .positionTolerance = 1_in});
  r.onFail(skipTo("third piece")).run(intake, 12_volt, seesPiece, 2_s);
  r.label("third piece").face(0_in, 48_in);
  ```
  `onFail(...)` applies to exactly the next step; `label("...")` names the next step (it is not a step of its own, so
  the step numbers on the brain's screen match the panel's list). No `onFail` = today's rule, unchanged. Both are
  NEW members — the frozen steps (F10, `docs/roadmap.md` :144) keep their exact signatures — and they are PINNED in
  F10 the day they land (L9's path), because the panel's generated code is written against them and must not drift.
  **Cost:** a modifier is a new idea in the chain (a line that does nothing to the robot by itself); its misuse rules
  are in §1.3.
- **What else is pinned, and what is not** *(re-check #4)*. Pinned with `onFail`/`label`: `OnFail`'s three spellings
  and `RoutineStopCause::MechanismNotStarted == 6` (MECH-A4 pinned `MechanismStepFailed` the same way). **NOT pinned:**
  STEP1-B's three trigger arms (`atPercent`, `afterDistance`, `afterTurning`) and the handles they return. They only
  lead to the START forms, and your ruling of 2026-10-29 keeps those open (F10 row, `docs/roadmap.md` :144: "Pinning
  them later is additive; un-freezing them would not be"). In F6 (the driving commands), STEP1-B's two new overloads
  and `LegObserver`'s two members are pinned, the way F6 pinned `MotionOptions`' fields. `motion::LegProgress`, the
  progress the observer receives, stays open to new fields (CURVE1 may add one); no pin covers its field list.
- **(b) An options field** (`MotionOptions::onFail`, or a new `StepOptions` on every step). **Cost:** `MotionOptions`
  is the facade's (F6, `chassis.hpp` :206-239) — a Tier-3 `chassis.moveTo(...)` has no chain to carry on or skip in,
  so the field would be a silent no-op there; and 15 of the Routine's step forms take no `MotionOptions` at all (the
  eleven mechanism steps, `pause`, `waitFor`, `then`, `startAt`) — each would need a NEW overload with a trailing
  options parameter (on a frozen step a defaulted parameter cannot be added: it changes the signature the F10 pin
  checks), about 20 new overloads, some beside the mechanism steps' trailing `const char* name`.
- **(c) Both.** Two ways to say one thing, and a rule for which wins. Rejected.

**D2 — What an "arrival tolerance" loosens. RECOMMENDED (a).**
A leg ends ("settles") when the robot is CLOSE ENOUGH (within 0.5 in, 1.15°) AND MOVING SLOWLY ENOUGH (under 1 in/s,
0.3 rad/s), both held 0.1 s (`motion_config.hpp` :172-178, HA-51 — invented). Probe P3 found that on a drive leg the
SPEED condition is the one that decides: the robot is already within 0.24 in when it first slows under 1 in/s.
- **(a) RECOMMENDED — the tolerance loosens BOTH, in proportion.** `positionTolerance = 2_in` means "within 2 in and
  slower than 4 in/s" (the robot's own 0.5 in / 1 in/s, scaled by 2 ÷ 0.5); `headingTolerance = 5_deg` the same for
  turning. A tolerance TIGHTER than the robot's own tightens only the distance (a smaller speed bound can sit inside
  the IMU's noise and never settle — `motion_config.hpp` :36-40). `0` = the robot's own settle numbers, bit for bit.
  Host test plant, the 40 in leg: 1.80 s → **1.35 s** at 2 in (it stops 0.93 in short), **1.12 s** at 4 in; with the
  motor lag TANK1's probes used, 1.40 → 1.25 / 1.14 s (P3). **Cost:** a leg can now end with the robot still rolling
  at a few in/s, or a turn still turning (at 5°, up to 1.31 rad/s, about 75°/s). On the host test plant it stops dead;
  on robot two it coasts a little further (HA-216, unmeasured, drives and turns, *re-check #11*). A robot whose own
  settle distance is 0 never settles at all; a tolerance on such a robot keeps the robot's own speed bound, because
  the scaling needs a distance above 0 to divide by. That rule is defined and tested, never a divide by zero
  *(re-check #11)*.
- **(b) The distance only** (R.13's text as written). **Cost:** on a drive leg it does nothing — P3: 1.80 s at 0.5,
  1, 2 and 4 in, identical to the hundredth (1.40 s under motor lag, identical too). The panel's "arrive within"
  inches box would change nothing a student can see. (On a TURN it does help: 1.18 → 1.05 → 0.87 s at 1.15°, 2°, 5°.)
- **(c) Two boxes per axis** — "within" and "slower than". **Cost:** two more numbers on every step of the panel's
  form, and a student must understand why both exist.
- **The angle's type** (any option): `headingTolerance` is a `math::Angle`, so it is written `2_deg` like every
  heading; a negative or wrapped value (`-5_deg`, `359_deg`) is refused at the call. (The alternative, the
  magnitude type `units::AngleDim`, has no degree literal — `units::AngleDim{0.035}` in radians.)

**D3 — "Skip ahead to a named step": what if the named step is BEHIND, or never comes? RECOMMENDED (a).**
A Routine runs its lines as they are reached; it never stores them (the "eager" ruling, `routine.hpp` :76-84). So a
skip can only go FORWARD: it passes over lines until it reaches the named one.
- **(a) RECOMMENDED.** The failing step parks the drive; every step until the named one is passed over (each logged,
  counted as "jumped"); the named step RUNS and the auton goes on from it. **Behind** — the name is on a step that
  already ran, including the failing step's own: the skip cannot go back, so the auton STOPS right there, exactly as
  "stop the auton", and the stop line says "its skip target 'X' is behind it". **Never comes** — a misspelled name:
  every remaining step is passed over (nothing more runs — the same as a stop), and the result reads "ROUTINE STOPPED
  … the skip to 'X' never landed". **The deadline:** a skip still looking for its step when the auton's time runs out
  stops there — a skip never resumes scoring after the buzzer. The library remembers the last 32 step names it passed
  (no heap) to tell "behind" from "never comes"; past 32, a behind name reads as "never comes" — the robot does the
  same either way. **A target that is already behind is also known when the line is WRITTEN** *(re-check #10)*:
  when `onFail(skipTo("X"))` runs, the routine has already passed a step labelled "X", or "X" is the step's own label.
  It logs one Warn there, on every run that reaches that line, so a bench run shows the mistake before a match does.
  Nothing throws, and the stop when the step fails is unchanged.
- **(b) As (a), but remember nothing:** behind reads as "never comes". **Cost:** the log cannot say "behind" when it
  happens; a student sees "never landed" for a name plainly written in their code.
- **(c) A backward skip re-runs the steps from the named one.** Impossible in an eager chain — the C++ lines have
  already run. Listed only so it is not asked again.
- (The panel's tool refuses a backward or unknown skip in a drawn route before any code is generated — G3a R.5/R.12;
  this is the library's answer for hand-written autons.)

**D4 — A START form that a skip jumped over, and the later "wait until it is done". RECOMMENDED (a).**
Example: `startRun(intake)` is skipped by a skip, and a later step waits for the intake. Today that wait THROWS out of
the auton (P1 (a): the auton ends, the program shows PROGRAM ERROR) or, if the intake ran earlier in the same auton,
answers that EARLIER run's verdict at once — "done" — though nothing was started (P1 (b): the wait took 0.000 s and
reported SUCCEEDED).
- **(a) RECOMMENDED — the wait FAILS, counted, saying why:** "the intake's start was skipped". It then follows the
  WAIT's own on-failure choice (default: stop). Never a throw, never an older step's answer. (A newer start of the
  intake that DID run clears the mark; the wait then collects that one.) The same answer, with "the trigger never
  fired: the robot did not reach the mark", serves STEP1-B (D6). *(re-check #2, stated exactly)* **What sets the mark:**
  only a skip jumping a step that would have STARTED the mechanism: the four start forms, and the blocking `move`,
  `home`, `run`, `spinUp` and `set`. A jumped `stop` or a jumped wait sets nothing. **What clears it:** only a step
  that actually starts the mechanism, or a trigger that fires. A wait never clears it, so a second wait gets the same
  "not started" answer rather than an older step's.
- **(b) The wait is skipped too, silently** ("nothing to wait for"). **Cost:** the auton goes on as if the intake had
  done its job; the next scoring step runs empty-handed and the result says nothing.
- **(c) Keep today's** throw / stale answer — the failure G3a's checker found (C3).

**D5 — How "carry on" shows in the result and on the brain's screen. RECOMMENDED (a).**
Today both result screens print **ROUTINE STOPPED** for any routine that had a failure (`auton_tester_run.hpp` :248,
`competition_run.hpp` :550) — wrong for a routine that carried on to its end.
*(re-check #12 and #5: rewritten. The draft recommended "FINISHED, 2 FAILED", now option (b).)*
- **(a) RECOMMENDED — "DONE, 2 FAILED", in AMBER**, on the Auton Tester's and Competition's result pages (the big
  word). It is 14 characters, and ten or more failures read "DONE, 9+ FAILED" (15). Both fit the driver page's
  existing 15-character "last auton" slot (`competition_screen.hpp` :552), so that line does not change. They also sit
  under the large text's limit of 18, `ui::kLargeCols` (`ui/touch_panel.hpp` :47), and that 18 is itself only an
  estimate: no one has measured it on a V5 (HA-136; the evidence puts it between 12 characters, read whole, and 23,
  reported shown). "FINISHED" stays the word for a run with no failures. The serial and SD log gets one extra line
  *(re-check #5)*, "first failed: step 7 grab-piece (mechanism step FAILED)", under the routine line, which says how
  many failed and how many a skip jumped. A skip that never landed reads **ROUTINE STOPPED** (nothing ran after it).
  Inside the library, the run's result keeps every frozen field's meaning ("where the auton STOPPED"), and NEW
  fields carry the rest (how many failed and the auton went on, the first of them, how many were jumped, a skip still
  looking for its step).
- **(b) "FINISHED, 2 FAILED"** (the draft's): 18 characters, exactly the estimated limit. If the real screen is
  narrower, the end is cut: at 16 it shows "FINISHED, 2 FAIL". The count is within the first 11 characters, so it
  shows at any width of 11 or more. The driver page's slot must widen from 15 to 18, which cuts auton names there
  from 27 to 24 characters, and ten or more failures need "FINISHED, 9+ FAILS" (P7).
- **(c) Keep ROUTINE STOPPED** for any routine with a failure. It says the routine stopped when it ran to its end.

**D6 — D15's second question: the drive ARRIVED, but stopped just short of the trigger's mark. Fire then?
RECOMMENDED (i), yes.**
A drive "arrives" when it is within its tolerance, not at the exact end (D2). On the host test plant the 40 in leg
arrives at 99.4 % of its length (0.24 in short) — a trigger at 100 %, or "after 39.9 in", would never fire, though the
drive succeeded; with motor lag the same leg overshoots to 100.5 % and the same trigger fires (P4). Whether a mark near
the end is reached depends on the robot, not the plan.
- **(i) RECOMMENDED — a drive that ARRIVES fires its unfired trigger as it arrives, if the mark is ON the drive.**
  The robot got there, within the tolerance the step accepts. *(re-check #3: narrowed. The draft fired every unfired
  trigger at the arrival, including marks the robot could never reach.)* "On the drive" means no further than the
  drive's length plus its arrival tolerance (for a turn: its angle plus its tolerance), measured from where the drive
  actually started. A mark past that, such as "after 50 in" on a 40 in drive, never fires, and a later wait says "the
  trigger never fired". A percent is always on the drive. A drive that ends any other way (timed out, failed, skipped,
  cut) never fires it — his D15 (a), unchanged. The library also refuses nonsense marks where they are written: a
  percent must be above 0 and at most 100; a distance or angle must be above 0; "after N inches" only on a drive;
  "after N degrees" only on a turn (a turn has no inches; the last two are refused when the code is compiled). The
  tool refuses a distance or angle longer than the planned leg too, as a typo guard (G3a R.7).
- **(ii) Never fire short;** the tool refuses a mark it can predict is past "planned length − tolerance". **Cost:** the
  live robot can still stop short of a mark the plan reached (P4: the same leg ends short on one plant, past on
  another); the drive then succeeds with the mechanism never started, and only the later wait notices.

**D7 — What "60 % of the drive" measures, and how a trigger counts. RECOMMENDED (a).**
- **(a) RECOMMENDED — the distance covered ALONG the straight line from where the leg started to its target**, from
  the robot's position estimate (for a turn: the angle turned toward the target), counted from the leg's first live
  tick. It fires the first tick the robot reaches the mark, at most once, and never on or after the deadline tick
  (P6). A trigger is PART of its drive step: arming it adds no step to the count; if its START is refused when it
  fires (the mechanism absent this boot, held by another owner, not homed), the drive is NOT interrupted — the drive
  step fails after its leg ends, under the drive step's own choice. *(re-check #1)* For this to hold, a trigger's
  start needs its OWN short path: command the mechanism, note it, keep its answer. The existing start steps cannot be
  reused. On a refusal they stop the drive, and the drive cannot be stopped from inside its own tick. The checker's
  probe X3 ran that path with a lift that was not homed: the drive ended "cancelled" 12.5 in into a 30 in leg, or,
  with B7's catch, a throw left the auton after the leg.
- **(b) Distance still to go** (60 % = when the robot is 40 % of the leg's length from the target). **Cost:** a robot
  that starts off its line gains "progress" by moving sideways.
- **(c) Distance actually travelled** (the odometry's path length). **Cost:** a wobbly leg reaches "60 %" early.

**D8 — A trigger written wrong (for example "until it sees a piece" on a mechanism that has no piece sensor): when is
the mistake caught? RECOMMENDED (a).** *(re-check #9, NEW. The draft decided this silently as (c), and rejected (a)
for a reason that was wrong: the reads it needs are already public.)*
Today a start step with such a mistake throws at its own line, on every run. A trigger's start runs later, partway
through the drive. A check made only there shows up only on runs whose drive REACHES the mark: a bench run whose drive
stops early never shows it, and a match can be the first run that does.
- **(a) RECOMMENDED — checked at the line that sets the trigger, every run, by the mechanism's OWN checks.** STEP1-B
  moves each start command's list of checks in `mechanism/service.hpp` (`run`, `moveTo`, `home`, `spinUp`) into a
  check function. The command and the trigger's line both call it, so there is one list and no copy to drift. **Cost:**
  STEP1-B edits `service.hpp`, which the string-lift chunks (MECH-S/TUNE1) also edit; whichever lands second re-reads.
  A check the string chunks add later goes into the check function, and triggers get it for free. About +¼ day.
- **(b) A copy of those checks in `routine.hpp`**, tied to the service's list by a paired test (each case given to
  the blocking step and to the trigger: both must throw). No `service.hpp` edit. **Cost:** two lists. A check the
  string chunks add later is missed by triggers until someone adds it to the copy, and the paired test covers only the
  cases it lists.
- **(c) Only when the trigger fires** (the draft's B7): caught inside the drive, held, and thrown after it. **Cost:**
  the visibility problem above.
In every option, a mistake caught at the fire is held until the drive ends and thrown then. It never becomes a fake
"cancelled" drive. Under (a) that cannot happen at all, because the fire runs the same checks the trigger's line
already passed (the checks read only the arguments and the robot file, never the run).

---

## The design in plain words (for the team lead)

Today an auton made of `Routine` steps has ONE rule for trouble: the first step that fails parks the robot and
everything after it is skipped. That rule stays, word for word, as the DEFAULT. STEP1 adds two things an author can put in
front of any step: "if this fails, carry on" and "if this fails, skip ahead to the step called X" — and a way to name a
step. A failure under "carry on" is still counted and logged; the robot is stopped where it is, and the next step
starts from there. Under "skip", the steps in between are passed over (counted, logged) and the auton resumes AT the
named step.

Two safety rules sit above every choice. **The deadline always stops.** When the auton's time runs out the library cuts
the moving step; that cut, and any failure after the deadline, stops the auton whatever the step chose — otherwise a
"carry on" auton would keep issuing steps past the buzzer and its park would be late. The probe shows exactly that
today: a carry-on chain cut at T+1.00 s ran a frozen 2 s pause past the deadline, its park started at T+3.04 s, failed,
and the hard stop fired (P2-B). **Nothing throws.** A "wait until the intake is done" whose start was skipped, or whose
trigger never fired, is a counted failure that says why — never a crash out of the auton and never an older answer.

Each drive or turn step also gets an **arrival tolerance** ("within 2 in", "within 5°"). The probe found that just
widening the distance does nothing for a drive, because the robot only counts as arrived once it is nearly stopped; so
the tolerance also lets the robot count as arrived while moving a little faster, in proportion (D2). A 40 in drive that
takes 1.80 s to finish within 0.5 in finishes in 1.35 s within 2 in, on the host test plant.

STEP1-B adds **triggers partway through a drive**: "at 60 % of this drive, start raising the lift". The library watches
the drive's progress every tick and starts the mechanism the first tick the robot reaches the mark. If the drive ends
early — timed out, failed, skipped, cut by the deadline — the trigger never fires (his D15 (a)). If the drive arrives
just short of the mark, it fires as it arrives, as long as the mark is on the drive (D6, narrowed at the re-check). A
later "wait until the lift is done" after a trigger that never fired fails, counted, saying "the trigger never fired".
A trigger written wrong is caught on the line that sets it, every run (D8).

What stays the same: every auton written today runs exactly as before (the first test of each run proves it); the
route file format VexBuilder reads (F8) does not change; routes run by the route runner keep their own rules. What is
not in STEP1: smooth curves (CURVE1, his D16 (c), after the route maker), and anything in the panel (G3a's R1 and R2
generate and draw what STEP1 makes possible).

---

## §0 Why and what exists (every line cited was OPENED at `eab0f89`)

### 0.1 Why this chunk, and why now

- The team lead, 2026-10-29 evening, on the panel's concept page: *"the route maker does not allow for care in what
  to do and is very rudimentary"*; he picked care on every step and partway triggers (G3a :195-220, ruling 4 at :209-213). His
  answers of 2026-10-30: "Accept all" for D13–D15 (G3a :3-26).
- G3a's parts write no `include/` (G3a :978-979), so the library half is its own chunk: `docs/internal/build-order.md`
  :845 (phase table row G: "G1, G2, CAL1, G3a, STEP1, CURVE1, G3, PKG1, G4"), :855 (75 chunks, STEP1 added
  2026-10-30); `docs/internal/REMAINING-WORK.md` :108 (row 20c: "before G3a Part R1", ~5–6½ days).
- The deadline: G3a's slot table puts STEP1-A's build at Nov 2–5, its review Nov 6–7, STEP1-B's build Nov 8–9 and its
  review Nov 11–13 (G3a :1288-1300); routes land ~Nov 18–22 against the Nov 21 target, "STEP1-A's slot by ~Nov 3"
  (:1310). The first competition is 2026-11-23; the code freeze ~Nov 16.

### 0.2 What exists in the tree

**The Routine** (`include/shulib/chassis/routine.hpp`, 1111 lines; F10 FROZEN, `docs/roadmap.md` :144):
- **Eager**: each step runs when it is chained; a deferred chain was rejected (:76-84). Every step is "exactly one
  Chassis call" (:86-88).
- **One error policy** (:109-133): the first failure safes the drive (`Chassis::cancel`), skips every later step
  (counted, Info-logged), Warns once, and `ok()` goes false. "A routine that must CONTINUE past a failure branches on
  the verbs directly" (:134-139). Precondition throws are not policy (:128-132).
- `skipIfStopped` (:821-839) is the door of every step; `recordStop` (:853-877) records, cancels, Warns;
  `ok()` is `stoppedAt_ == 0` (:750); `result()` (:753-765) copies nine fields.
- **`RoutineResult`** (:231-250): `ok`, `steps`, `completed`, `skipped` ("steps skipped after the stop"),
  `stoppedAt`, `stoppedName`, `cause`, `exit`, `mechanismVerdict` (appended at MECH-A4). **`RoutineStopCause`**
  (:210-225), append-only; `MechanismStepFailed` = 5 is the last.
- **Constructors**: the frozen `Routine(Chassis&, name)` (:262-263) and MECH-A4's `Routine(ctx, name)` over the
  `RoutineContext` concept — a `chassis` and a `mechanismService` (:197-201, :271-273). Neither knows the run guard.
- **MECH-A4's mechanism steps** (:522-745), NOT pinned (the F10 row: "OPEN … leave it open for now"). The verdict
  policy: SUCCEEDED and a counted UNCONFIRMED continue, everything else stops as `MechanismStepFailed` (:543-552); a
  START form stops the chain only when ABSENT, REFUSED or NOT_HOMED (:955-967).
- **`waitUntilDone`** (:666-696) answers "the step THIS routine last began on `m`"; a step that already ended
  "answers at once" (:667); its precondition throws when "this routine started no step on that mechanism"
  (:682-685); `startedSeq_` is written only by `noteStarted` (:918-922), which only a step that RUNS calls (every
  mechanism step's first line is `skipIfStopped`, e.g. `powerStep` :1049-1062).

**The facade** (`include/shulib/chassis/chassis.hpp`; F6 FROZEN, `docs/roadmap.md` :140):
- `MotionOptions` (:206-239): `timeout`, `maxLinearSpeed`, `maxAngularSpeed`; "the field SET is deliberately
  additive-open — a future knob is a new field with a 0/'config default' meaning" (:209-211). The F6 pin pins each
  existing field's type, not the set (`test/f6_signature_pin_test.cpp` :318-330).
- `effectiveConfig` (:571-580) copies the chassis config and overrides only the two speed budgets.
- The blocking verbs are `async` + `waitUntilSettled` through the private `runBlocking`/`DetachGuard` (:541-566).

**The settle rule**: `SettledUtil` settles when `|error| <= maxError && rate <= maxErrorRate`, held `settleTime`
(`control/settled_util.hpp` :72); `translationSettle {0.5 in, 1.0 in/s, 0.1 s}`, `headingSettle {0.02 rad, 0.30 rad/s,
0.1 s}` (`motion/motion_config.hpp` :172-178; HA-51, invented); the heading rate floor sits ~2.5σ above the IMU's
noise (:36-40). `MoveToPose` needs BOTH criteria (`move_to_pose.hpp` :230-248, settle utils at :447-448); `TurnTo`
reads `headingSettle` (`turn_to.hpp` :60); `StrafeTo` and `HoldPose` are the `MoveToPose` engine (:61).

**The scheduler's loop**: `waitUntilSettled` ticks and paces, "no trailing pace after the exit tick"
(`motion/motion_scheduler.hpp` :800-806); `waitUntil` checks its predicate, ticks, then ALWAYS paces (:819-848, the loop :833-848). The
task-boundary catch converts a `PreconditionError` thrown inside a motion's tick into a Cancelled exit with
`FaultCode::Precondition` (:87-96, :1123-1132).

**The run guard** (`include/shulib/sequence/run_guard.hpp`): at `endActionAt` it cuts the active motion from `pace()`,
"the blocking verb returns Cancelled … a Routine records the stop and skips the rest" (:60-75); the deadline checks
run in `pace()` BEFORE the world advances (:345-368); the frozen waits cannot be cut, and the lateness bound "assumes
… its first post-deadline motion is refused and stops the chain" (:94-108); `expired()` THROWS when no run is live
(:400-403); `waitFor`/`pause` return `RunExpired` with zero latency (:437-468). `AutonContext::guard` is "the only waits
the deadline … can cut" (`competition/auton_registry.hpp` :218-220); `runAuton` builds the context at
`match_runner.hpp` :196 and runs the routine inside `guard.run` (:210-262).

**The result screens**: `testerResultWord` (`src/programs/auton_tester_run.hpp` :214-249) and
`competitionResultWord` (`competition_run.hpp` :513-551) end in `ranOk && endActionSucceeded ? "FINISHED" : "ROUTINE
STOPPED"`; the routine line prints "routine STOPPED, … stopped at …" only when `stoppedAt > 0`, else "routine OK/STOPPED"
(:399-411 / :722-736); both lines live under 200 bytes (`kTesterLineBytes` :292, `kCompetitionLineBytes` :113). The
word's colour: `competitionResultTone` — FINISHED green, ABANDONED / CUT BY DEADLINE / TOO LATE amber, anything else
RED (`competition_screen.hpp` :353-359); the Tester's `resultTone` likewise (`auton_tester_panel.hpp` :612-616); the
driver page's last-auton line `"auton %.15s: %.27s"` (`competition_screen.hpp` :552); `competitionRunLevel` logs
anything but FINISHED/NONE at Warn (`competition_run.hpp` :555-565). `routineStopWord` / `competitionStopWord` switch
over every `RoutineStopCause` with no default (`auton_tester_run.hpp` :275-288, `competition_run.hpp` :568-).

**The route runner's precedent**: markers fire on Settled only — "a marker that fires at the wrong place is …
strictly worse than a marker that does not fire at all" (`runner/path_runner.hpp` :45-50, ruling R2); MECH-A4's route
ids block, and "a step that starts mid-leg is the mechanism core's later 'triggers partway through a drive'"
(`runner/mechanism_routes.hpp` :19-23). MECH-PANEL's §5 Q19 proposed the same rule for triggers
(`docs/internal/MECH-PANEL-requirements.md` :338) and sized "mid-drive triggers for routines ~1–2" in MECH C (:764).

### 0.3 What G3a's R.13 got right, and what this brief corrects

R.13 (G3a :977-1074) is right on the shape: two runs; `MotionOptions` grows two fields; the choice is opt-in and
F10-additive; a cut always stops; a jumped start and an unfired trigger answer a counted failure; STEP1-B adds a public
F6-additive per-tick observer seam (C10); the MECH C item is taken. Corrected here, each with its evidence:

1. **The tolerance as worded is inert on a drive leg** (P3) — D2 and SM16.
2. **A motion ticks AT the deadline instant before the cut** (P6: 101 ticks, the last at T+1.0000 with the deadline at
   T+1.0000) — so a trigger must not fire once `guard.expired()` reads true; the "never after the deadline" rule
   needs the guard's word, not the drive's verdict — ST29, SM24.
3. **The observer seam cannot be built from `waitUntil`** — it paces once after the exit tick (P5: the clock at the
   verb's return 1.800 s vs 1.810 s) — so ST19's "bit-identical" holds only for a seam that wraps the motion inside
   the same `waitUntilSettled` loop (§1.2) — SM23. *(The checker's probe X2 confirmed it: a decorator through
   `waitUntilSettled` matched the plain `moveTo`'s clock and pose exactly.)*
4. **A precondition thrown by a trigger's start inside the motion's tick becomes a CANCELLED drive** (the
   task-boundary catch, `motion_scheduler.hpp` :1123-1132) — a misleading verdict, and under the cut rule a stop.
   STEP1-B checks the start's arguments when the trigger is SET (D8 (a)), and keeps a catch around the fire's command
   that re-throws after the leg as a backstop (§1.3 B7) — ST30, SM26, SM32. *(re-check #1, #9)* **And the fire must not
   reuse the start steps' recording** (`startedMechanism` / `notOnThisRobot` → `recordStop` → `Chassis::cancel()`,
   `routine.hpp` :853-860, :955-1075): `MotionScheduler::cancel` refuses from inside a tick (`motion_scheduler.hpp`
   :857-859), so a NOT_HOMED, REFUSED or ABSENT start would cancel the drive or throw (the checker's X3) — §1.2's fire
   path, ST33, SM28.
5. **The cut rule must ask `guard.running()` first**: `expired()` throws with no run live (P2-D), and the tree already
   builds an `AutonContext` over a guard that is not running (`test/motion_tank_steer_test.cpp` :510, :526;
   `test/mechanism_auton_run_test.cpp` :357) — ST25, SM18.
6. **The result word reaches two more files** than R.13 lists — the colour (`competition_screen.hpp` :353-359,
   `auton_tester_panel.hpp` :612-616: an unknown word is RED) and the driver page's 15-character slot (:552) — and
   needs a ten-or-more form (P7) — D5, ST17, SM21.
7. **A skip pending at the deadline must stop**, or a landed label resumes scoring after the buzzer (a case the
   default can never reach, because the default stopped earlier) — D3 (a), ST15 (d), SM27.
8. **"Run no further step" (R.13's ST15) is not true even of today's default** after a frozen pause: a step that
   SUCCEEDS after the deadline runs in today's chain too. The invariant that is true, and the one tested: **from the
   deadline on, a chain with choices does exactly what the same chain without them does** (ST15).

### 0.4 The probes (`.claude/coordinator/scratch/brief-STEP1/`; `build.sh` compiles each against main's tree; every binary run under `ulimit -v 4194304` + `timeout 300`; logs in `out/`)

| # | What it ran | What it found | Log |
|---|---|---|---|
| **P1** | MECH-A4's robot-x rig: a `waitUntilDone(intake)` after a `startRun(intake)` that did not run (the `if` a skip would make) | (a) no earlier intake step: **THROWS** `PreconditionError` "this routine started no step on that mechanism"; (b) an earlier 0.3 s `run(intake)` succeeded: the wait took **0.000 s and reported SUCCEEDED** — the earlier step's verdict; (c) (a) inside `runAuton`: the throw leaves the auton (guard disarmed, drive at 0 V, intake released — a program shows PROGRAM ERROR) | `out/p1.log` |
| **P2** | `runAuton` with the park at T+1.00, hard stop T+2.50: drive 60 in (cut) → frozen pause 2 s → drive back → intake 1 s | **A** today's Routine: stops at the cut; scoring ends T+1.01; park succeeds T+1.12; 1 cut. **B** carry on by hand (each step its own Routine): the frozen pause runs to **T+3.00**; scoring ends **T+3.04**; the park **FAILED**; **HARD STOP fired**; 3 cuts. **C** as B with the guard's pause: it returns RunExpired at once, scoring ends T+1.03 — but steps 3 and 4 were still ISSUED after the deadline (step 3's drive cut on its first pace — 2 motion cuts in all with step 1's — and step 4's intake cut by the latch: 1 mechanism cut). **D** `guard.expired()` with no run live: **THROWS** "RunGuard::expired: no run is live" | `out/p2.log` |
| **P3** | 40 in `moveTo` and 90° `turnTo` on the 12 in tank (memoryless plant, and kA 0.02 motor lag) with the settle's `maxError` raised alone vs with `maxErrorRate` scaled in proportion | **Drive, distance alone: 1.80 s at 0.5, 1, 2, 4 in — identical** (0.237 in short each); 2 in + rate ×4: **1.35 s** (0.93 in short); 4 in + ×8: **1.12 s** (1.88 in short). Under lag: 1.40 s at every distance alone; 1.25 s / 1.14 s scaled. **Turn**: 1.18 → 1.05 → 0.87 s at 1.15°, 2°, 5° alone (0.82 s scaled); under lag 1.21 → 1.10 → 0.80 s (0.63 s scaled, and 0.93° past instead of 3.17°) | `out/p3.log` |
| **P4** | the same legs, the along-track progress (from the estimate) each tick | the 40 in leg **settles at 99.41 %**: 99 % reached at 1.64 s, **100 % never**; under lag it overshoots to **100.53 %** (100 % at 1.21 s) and settles at 1.40 s; at 2 in scaled it settles at 97.67 % (a 99 % mark never reached) | `out/p3.log` |
| **P5** | the blocking `moveTo` vs `scheduler().async(m)` + `waitUntil(!active)`, same leg | clock at return **1.800 s vs 1.810 s** — one extra pace after the exit tick | `out/p3.log` |
| **P6** | a stand-in motion that records the clock at each tick, under `guard.run` with the park at T+1.00 | **101 ticks, the last at T+1.0000** — one tick runs AT the deadline instant, before the cut | `out/p6.log` |
| **P7** | the candidate words against the screen widths | "FINISHED, 2 FAILED" 18 (fits `kLargeCols` 18); "FINISHED, 10 FAILED" 19 (does not); the driver line prints "FINISHED, 2 FAI"; widening it to 18 + a 24-character name keeps the line at 50 (`kSmallCols`). *(re-check: "DONE, n FAILED" 14, "DONE, 9+ FAILED" 15 — both fit the driver slot unchanged; `recheck/r5_final_forms.log`)* | `out/p7_words.log` |

Not probed (reasoned, and said so where used): that a tighter heading tolerance's smaller rate bound can sit inside
the IMU's noise (the header's own arithmetic, `motion_config.hpp` :36-40); the robot's coast after a leg that ends at a
few in/s (HA-216); a trigger's latency on the robot (HA-217).

---

## §1 Scope — IN (D1 (a), D2 (a), D3 (a), D4 (a), D5 (a), D6 (i) as narrowed, D7 (a), D8 (a) assumed; the alternatives' scope is in their option text)

### 1.1 STEP1-A — the arrival tolerance and the on-failure choice

**`include/shulib/chassis/chassis.hpp` — CHANGED (F6-additive)**
- `MotionOptions` APPENDS `units::Length positionTolerance{0.0}` and `math::Angle headingTolerance{}` (after
  `maxAngularSpeed`, so every designated initializer keeps compiling; 0 = the robot's settle config). `validate()`
  refuses a non-finite or negative value (a negative angle includes a wrapped one: `359_deg` stores −1°).
- `effectiveConfig` (:571-580) applies them under D2 (a): for each axis with a tolerance `t > 0` and the robot's own
  `{e, r}`: `maxError = t`; `maxErrorRate = (e > 0 && t > e) ? r × t / e : r` (loosen in proportion, never tighten
  the rate; *re-check #11:* `e ≤ 0` keeps `r`, never a division by 0 — the draft's `r × t / e` gave `inf` at `e = 0,
  r > 0`, which silently drops the speed bound, and `NaN` at `e = 0, r = 0`, which fails `SettledUtil`'s
  `maxErrorRate >= 0` precondition, `settled_util.hpp` :51, on every leg with a tolerance. `SettledUtil` allows
  `maxError = 0` (:50), and `MotionConfig::validate` does not look inside the settle configs, `motion_config.hpp` :214-216, :246-247);
  `settleTime` untouched; `brakeSettle` untouched. The arithmetic is one PURE function in
  `include/shulib/motion/motion_config.hpp` (CHANGED, additive; that header is its layer's, outside F6 —
  `chassis.hpp` :17-20), e.g. `motion::loosenedSettle(const control::SettleConfig& robot, double tolerance)`, which
  `effectiveConfig` calls and ST23 tests directly (`effectiveConfig` itself is private). It reaches every verb that reads the settle config — `moveTo`,
  `driveTo`/`reverseTo` (through `moveTo`), `followTrajectory`'s legs, `strafeTo`, `turnTo`/`face`/`faceAway`, and
  `hold` (Settled iff within the tolerance when its window ends) — said in the field's doc.
- The banner's Options note (:162-166) gains one sentence; the F6 register row one sentence (§3).

**`include/shulib/chassis/routine.hpp` — CHANGED (F10-additive), one delimited block "STEP1-A: the on-failure
choice — BEGIN/END", members declared after MECH-A4's (initializer order)**
- **Types:** `chassis::OnFail` (a small value: stop / carry on / skip to a label), `inline constexpr OnFail stopChain`,
  `inline constexpr OnFail carryOn`, `OnFail skipTo(const char* label)`. Re-exported into `mechanism::words` beside
  `Routine` (:1106-1111), so a generated auton needs no namespace.
- **Members:** `Routine& onFail(OnFail)`, `Routine& label(const char*)` — modifiers on the next step (§1.3 A1-A4).
- **The guard:** the context constructor (:271-273) also borrows `ctx.guard` when the context has one
  (`if constexpr (requires { ctx.guard; })`) as a `sequence::RunGuard*`; the frozen `Routine(Chassis&)` keeps null.
  `routine.hpp` includes `sequence/run_guard.hpp` (that header includes `chassis.hpp`, never `routine.hpp`
  (`run_guard.hpp` :209) — no cycle).
- **`RoutineStopCause::MechanismNotStarted`** APPENDED (= 6, pinned): "a `waitUntilDone` whose start never ran — a
  skip jumped it (STEP1-A) or its partway trigger never fired (STEP1-B)". `stopCauseText` (:879-902) gains it.
- **`RoutineResult` APPENDS six fields** (types pinned): `int failedOn` (steps that failed and the chain went on),
  `int firstFailedAt`, `const char* firstFailedName`, `RoutineStopCause firstFailedCause`, `int jumped` (steps a skip
  passed over), `const char* skipPendingTo` ("" = no skip looking for its step). The nine frozen fields keep their
  meanings, read as "the chain's STOP" (G3a R.6's definition): under carry on and skip `stoppedAt` stays 0 unless a
  stop happens; `skipped` stays "after a stop".
- `ok()` (:750) becomes `stoppedAt_ == 0 && failedOn_ == 0` — its frozen doc ("no step has failed") unchanged; under
  the default `failedOn_` is always 0, so `ok()` is bit-identical there.
- The banner's error-policy paragraph (:109-139) gains the opt-in paragraph and the two rules above every choice (the
  cut; no throw); "branches on the verbs directly" (:134-139) gains "or `onFail(carryOn)`".

**`include/shulib/version.hpp`** — `kApiMinor` 5 → **6** (:123), with its history paragraph.

**`src/programs/auton_tester_run.hpp`, `src/programs/competition_run.hpp` — CHANGED (D5 (a))**
- The result word: before the final `FINISHED`/`ROUTINE STOPPED`, a routine with `stoppedAt == 0`, no pending skip,
  `failedOn > 0` and its end action succeeded reads `"DONE, n FAILED"` (a static table of nine words + "DONE, 9+
  FAILED" — the functions keep returning `const char*`; every word ≤ 15 characters); a pending skip reads `ROUTINE
  STOPPED`. *(re-check #12: was "FINISHED, n FAILED", 18.)* That word can only occur with the first log line's short
  fields ("in time", "end action succeeded", no abort, no hard stop — the word's own conditions), so line 1's worst
  case does not grow.
- **The routine line** *(re-check #5: the draft's one-line form, `"run %d: routine FINISHED, %d failed (first: step %d
  %.24s, %s), %d jumped, %d of %d steps done; %.79s"`, is 225 bytes at its worst in the Tester and 229 in
  Competition — over the 199 the brief claimed; 210/214 even with 1-digit counts. The checker's
  `check/probes/x_line_budget.py`, re-run in `recheck/r5_line_budget.py`)*. Amended to TWO lines when the routine went
  on past a failure or a skip is pending, keeping each frozen form and the standard faults tail whole:
  - line 3, the summary: `"run %d: routine DONE, %d of %d steps done, %d failed, %d jumped; %.79s"` (pending skip:
    `routine STOPPED`, same fields) — **155 / 159 bytes** at the worst case (run/request 99999, 4-digit counts, the
    faults tail at its 79 cap);
  - line 3b, the first failure: `"run %d: first failed: step %d %.24s (%s); the routine went on"` — or `"…; a skip to
    '%.24s' never landed"`, or `"…; the routine went on, then stopped"` when a later step stopped the chain (line 3 is
    then today's STOPPED form, unchanged, 193 / 197) — **104–138 bytes** (`recheck/r5_final_forms.log`). Both new
    lines fit even with EVERY count, and the run/request number, at its `int`/`unsigned` extreme (189–195 / 131–150
    bytes, the same log) — unlike today's STOPPED form (L11).
  Neither program has a free line today: the Tester uses all four of `TesterRunLines::line[4]` on a routine run
  (`auton_tester_run.hpp` :295-300), and Competition uses four of `line[5]`, plus the driver-loop line
  (`competition_run.hpp` :585-589, :767-783). So `line[4]` → `line[5]` and `line[5]` → `line[6]` (+200 bytes each, a
  value struct). Line 3b is written ONLY when `failedOn > 0` or a skip is pending. Under the default no line count
  changes, and every existing golden holds (ST1).
- `routineStopWord` / `competitionStopWord` gain `MechanismNotStarted` → "mechanism NOT STARTED" (21 characters, the
  same as today's longest, "mechanism step FAILED": the STOPPED form's worst case is unchanged).
- `competitionRunLevel` unchanged in rule (anything but FINISHED/NONE is Warn): the new word logs at Warn.

**`src/programs/competition_screen.hpp`, `src/programs/auton_tester_panel.hpp` — CHANGED**: the new words are AMBER
in `competitionResultTone` (:353-359) and the Tester's `resultTone` (:612-616), matched by the prefix "DONE, ". This
is the one place a prefix match is right, because every such word is generated here. The driver line (:552) keeps
`"auton %.15s: %.27s"`: every DONE word fits its 15 characters *(re-check #12; under D5 (b) it would become
`"auton %.18s: %.24s"`)*.

**Tests — NEW** `test/routine_on_fail_test.cpp` (the choice, the labels, the counts, the modifiers' rules),
`test/routine_cut_rule_test.cpp` (the deadline under each choice, on the MECH-A4 rig), `test/routine_tolerance_test.cpp`
(D2 on the host test plant); **CHANGED** `test/routine_signature_pin_test.cpp` (F10: `onFail`, `label`, `OnFail`'s
three spellings, the six fields' types, `MechanismNotStarted == 6`), `test/f6_signature_pin_test.cpp` (the two fields'
types), `test/mechanism_auton_steps_test.cpp` (ST18's cases beside A4's wait tests), `test/auton_tester_run_test.cpp`,
`test/competition_run_test.cpp`, `test/competition_screen_test.cpp` (ST17), `test/cookbook_examples_test.cpp` (the
new recipes, §3.1), `test/CMakeLists.txt`.

### 1.2 STEP1-B — triggers partway through a drive (after STEP1-A's build ends; it re-reads STEP1-A's tip)

**`include/shulib/motion/leg_progress.hpp` — NEW, PURE** (no chassis, no sequence include): `struct LegProgress {bool
live; double fraction; units::Length along; double turnedRad;}`; `legProgressAlong(start, target, pose)` — the
projection of (pose − start) on the unit start→target vector, over the straight-line length; a length under 1e-6 in
reads `fraction = 0` with a flag (a zero-length leg has no "percent", §1.3 B5); `turnProgress(startHeading, target,
heading)` — the angle turned in the direction of the start's shortest signed error, over its size (exact ±180°
resolves CCW like `TurnTo`). Host-tested on its own.

**`include/shulib/chassis/chassis.hpp` — CHANGED (F6-additive), after STEP1-A's lines**
- `struct LegObserver { void (*fn)(void* context, const motion::LegProgress&) = nullptr; void* context = nullptr; }`
  — no heap (`runner::Handler`'s pattern).
- Two overloads: `moveTo(const Pose2d&, const MotionOptions&, LegObserver)` and `turnTo(Angle, const MotionOptions&,
  LegObserver)`. Each builds the same motion as today's verb and runs it inside a private decorator `IMotion` that
  forwards `start`/`tick`/`cancel`/`exitReason`/`state`/`name` (so records and `CompletedMotion` say "MoveToPose" /
  "TurnTo" exactly as today), captures the start pose at the inner motion's first LIVE tick, and after each inner tick
  that returns `Running` calls the observer once with the progress computed from the pose that tick used — never on
  the exit tick, never after it — through the SAME `runBlocking` (`waitUntilSettled`, no trailing pace; P5). The
  observer runs inside the scheduler's tick: it must not call a blocking verb or `cancel()` (the scheduler's own
  preconditions say so), and anything it throws is the observer's to catch (§1.3 B7).
- The observer overload IS the Tier-3 spelling the cookbook shows (`routine.hpp` :86-88's tier rule: a recipe is never
  the only way).

**`include/shulib/chassis/routine.hpp` — CHANGED (F10-additive), block "STEP1-B: partway triggers"**
- `atPercent(double)`, `afterDistance(units::Length)`, `afterTurning(math::Angle)` return a small handle whose ONLY
  members are the four START forms (`startMove`, `startHome`, `startRun`, `startSpinUp`, same parameters) and refusal
  templates for the blocking forms and `set` (a `static_assert` naming the start form — MECH-A1's compile-refusal
  harness, ST14). Each start form ARMS (commands nothing) and returns an `[[nodiscard]]` armed-leg handle whose only
  members are the leg steps and the three arms (up to 4 triggers per leg). *(re-check #7)* Discarding it is a
  warning, which **fails the gate and CI** (`tools/src_build_gate.py`, the ARM header gate's `-Werror`), and is
  **only a warning in a plain `make`**: the robot Makefile turns warnings on WITHOUT `-Werror` (`Makefile` :31-53).
  So the library also says it at run time. The next step on `r` while a trigger is armed is a precondition
  (deterministic: the bench finds it). A Routine destroyed with a trigger still armed (the discarded line was the
  auton's last) logs one Warn from its destructor: "routine 'x': N trigger(s) armed but no drive followed: they never
  fired". The destructor is `noexcept`, a log failure is swallowed, and nothing is logged when nothing is armed.
- **The handle is split by what the mark measures** *(re-check #3)*: after `atPercent` every leg step is offered;
  after `afterDistance` only the drive legs (`driveTo`, `reverseTo`, `moveTo`); after `afterTurning` only the turn
  legs (`turnTo`, `face`, `faceAway`). Arms chained on one leg narrow it (the handle carries a kinds mask). A leg of
  the wrong kind, or a chain that leaves no kind (`afterDistance(…)…afterTurning(…)`), is refused by a
  `static_assert` naming the reason ("after N inches is a drive's mark — a turn has no inches; use afterTurning or
  atPercent"), ST34.
- **Arming preconditions** *(re-check #3)*: `atPercent(p)`: `p` finite and in (0, 100] ("at 0 %" is the start step
  written before the leg; the message says so). `afterDistance(d)`: finite and > 0. `afterTurning(a)`: > 0 (a
  `math::Angle` is finite by construction and wrapped into (−180°, 180°], so `270_deg` stores −90° and is refused,
  said in the message). Plus D8 (a)'s check of the start's own arguments, B8, and the arming rules (§1.3 B9).
  ```cpp
  r.atPercent(60).startMove(lift["high"], 2_s).driveTo(0_in, 48_in);
  r.afterDistance(20_in).startRun(intake, 12_volt, seesPiece, 3_s).atPercent(90).startHome(arm, 1_s).driveTo(40_in, 0_in);
  r.afterTurning(45_deg).startSpinUp(flywheel["shoot"], 2_s).turnTo(90_deg);
  ```
- The leg step delegates to the observer overload — ONE Chassis call (`driveTo`/`reverseTo`/`face`/`faceAway` reach
  `moveTo`/`turnTo` through their one computed argument, as today).
- **The fire's own path** *(re-check #1)*. ARMING resolves the slot (`mechs().find(…)`, `noexcept`; null = not on this
  robot) and runs the start's checks (D8 (a)). THE FIRE, inside the motion's tick, does exactly this: the slot's
  command (`moveTo` / `home` / `run` / `spinUp`, with the start form's arguments) → `noteStarted` → keep the slot's
  start report in the trigger's record. A trigger whose mechanism is not on this robot fires as ABSENT: nothing is
  commanded, and `notOnThisRobot`'s sentence is logged with its logging half, factored out. **Inside the tick it never
  calls** `skipIfStopped`, `recordSuccess`, `recordStop`, `notOnThisRobot`'s recording half, `startedMechanism`,
  `awaitMechanism` or `Chassis::cancel()`. AFTER the leg returns, outside the tick, the drive step is recorded ONCE:
  the leg's own verdict if it failed (a refused trigger is then named in the line's detail); else, if a fired
  trigger's start was ABSENT / REFUSED / NOT_HOMED, `MechanismStepFailed` with the first such verdict (`mechanismVerdict`
  and `lastMechanism()` carry it); else a success. A SUCCEEDING trigger adds no step and no `completed` count
  (D7 (a)). After the leg, `lastMechanism()` reads the last fired trigger's start report.
- **`mechanism/service.hpp` — CHANGED under D8 (a)** *(re-check #9; the draft said "NOT edited")*: each start
  command's precondition block (`PowerSlot::run` :512-547, `PositionSlot::moveTo` :779-786, `home` :855-861,
  `SpeedSlot::spinUp` :1317-1325) moves, unchanged and in the same order with the same messages, into a `const` check
  function the command calls first (`checkRun`, `checkMoveTo`, `checkHome`, `checkSpinUp`). This is a pure extraction:
  every MECH-A1..A5 test passes unchanged, and the checks read only the arguments, the slot's `spec()` and `peer()`
  (bound at boot), never run state. Under D8 (b) the file is not edited; under D8 (c) neither, and B7 alone stands.

**`include/shulib/runner/mechanism_routes.hpp`** — the comment at :19-23 says triggers are now a Routine form
(STEP1-B) and that route markers stay blocking (F8 unchanged). Comment only.

**`include/shulib/version.hpp`** — `kApiMinor` 6 → **7** (or shared with a string chunk landing in the same window —
the coordinator's, at merge).

**Tests — NEW** `test/leg_progress_test.cpp`, `test/chassis_leg_observer_test.cpp` (ST19's bit-identity, the tick
count), `test/routine_trigger_test.cpp` (ST9–ST13, ST20–ST22, ST29–ST33, ST35, ST36 on the MECH-A4 rig),
`test/compile_refusals/` cases for ST14 and ST34; **CHANGED** `test/f6_signature_pin_test.cpp` (the two overloads,
`LegObserver`'s two members) — *(re-check #4: the draft also pinned the three arms in F10; they stay UNPINNED, like
the start forms they lead to — the 2026-10-29 ruling, D1)*; `test/mechanism_service_test.cpp`
(D8 (a): each check function throws exactly what its command threw, case for case); `test/CMakeLists.txt`.

### 1.3 The rules, as the runs build and test them

**STEP1-A**
- **A1 The default is today's.** A step with no `onFail` follows the frozen policy (a mechanism step: MECH-A4's verdict
  policy) bit for bit — the same counts, log lines, cancel calls and result (ST1).
- **A2 Modifiers.** `onFail` and `label` apply to exactly the next step and are consumed at its door — whether it runs,
  is skipped by a stop, is jumped by a skip, or throws a precondition. Two `onFail` (or two `label`) in a row,
  `skipTo(nullptr)` / `skipTo("")`, `label(nullptr)` / `label("")`: precondition throws (programming errors a host test
  meets every time; the counters untouched). Labels and skip targets are stored, not copied: string literals (the
  `name` rule, :259-261). Matching is whole-string and case-sensitive. *(re-check #10)* `onFail(skipTo("X"))` checks
  the remembered labels AT THE CALL: a label "X" already passed, or "X" pending as this step's own label, logs one
  Warn ("its skip target 'X' is behind it: a skip only goes forward"); no throw, nothing else changes (the STOP at
  failure, A4, stays). *(re-check #13)* Modifiers and triggers: `onFail`/`label` come BEFORE the first arm and pass
  through the arms to the leg, which consumes them at its door (`r.onFail(carryOn).label("lift").atPercent(60)
  .startMove(…).driveTo(…)`); an `onFail` or `label` call while a trigger is armed is a precondition (B9).
- **A3 Carry on.** A failed step is recorded (`failedOn`, the first one's index/name/cause), Warn-logged ("… FAILED —
  carrying on"), and the drive is safed (`Chassis::cancel`, the same call a stop makes) — the next step starts from rest
  where the robot is. `ok()` false.
- **A4 Skip.** As carry on, then every step until the next step `label`led with the target is passed over: counted in
  `steps` and `jumped`, one Info line each ("… jumped — skipping to 'X'"); the labelled step runs under its own choice.
  A target already seen (the last 32 names remembered) or the failing step's own label → a STOP at the failing step,
  said "its skip target 'X' is behind it" (D3 (a)). A target that never comes leaves `skipPendingTo` set; nothing more
  runs. A jumped mechanism step marks its mechanism (A6).
- **A5 Above every choice — the cut.** The step's choice is IGNORED and the chain STOPS when (a) the failed step's
  verdict is CANCELLED — a motion's exit or a mechanism step's verdict, whatever cancelled it (the deadline, an abandon,
  the fault policy's abort, `chassis.hpp` :86-90, a replaced step) — or (b) the Routine has a guard, `guard.running()`
  and `guard.expired()` (the deadline passed or the run was abandoned). A skip still pending when (b) becomes true stops
  at the next step's door (D3 (a)). A Routine with no guard keeps (a) only. *(re-check #8)* **(a) includes `then()`:**
  an action returning `control::ExitReason::Cancelled` or `manipulation::MechanismOutcome::Cancelled` is CANCELLED (a
  `bool` false is not). **A Routine with no guard** — `Routine r{chassis}`, `r{ctx.chassis}`, or a context struct
  without a `guard` member — has NO deadline rule beyond (a): a carry-on chain there goes on past a frozen wait that
  times out after the buzzer (P2-B's late park). So the FIRST `onFail(carryOn)` or `onFail(skipTo(…))` on a guard-less
  Routine logs one Warn: "routine 'x': on-failure choices with no run guard — the deadline cannot stop this routine;
  build it as Routine r{ctx, \"x\"} in an auton". A program with a `RunGuard` but no `AutonContext` gets the rule by
  passing a struct with `chassis`, `mechanismService` (may be null) and `guard` (the context constructor's concept,
  `routine.hpp` :195-201, plus the `requires { ctx.guard; }` test).
- **A6 A jumped start** *(re-check #2: rewritten — the draft marked on any jumped mechanism step and cleared on any
  step that ran, which gave a false stop after a jumped `stop`/wait and C3's stale verdict on a second wait)*. **SET:**
  only a skip jumping a step that would have BEGUN a step on that slot — the four start forms and the blocking `move`,
  `home`, `run`, `spinUp`, `set` — marks the slot "not started: its start was skipped". A jumped `stop` or a jumped
  `waitUntilDone` marks nothing. Marking happens only when the Routine has a bound service (`find()` is `noexcept` and
  returns null for an absent mechanism, `service.hpp` :1767-1806; a chassis-only Routine marks nothing and calls no
  `mechs()`). **CLEAR:** only where `noteStarted` runs — a step that began a step on that slot, or a trigger that fired
  (B4). A `waitUntilDone` NEVER clears the mark, whatever it answers; a `stop` does not either. **ANSWER:**
  `waitUntilDone` checks the mark BEFORE its precondition: a marked slot answers a counted failure
  `MechanismNotStarted` with the reason, under the wait's own choice — and a second wait answers the same, never the
  older step's verdict. For that answer `RoutineResult::mechanismVerdict` stays `Running` ("no mechanism verdict here":
  nothing ran — the field's doc, :244-248, says only `MechanismStepFailed` fills it), and `lastMechanism()` reads the
  default report (`Running`), never an older step's report. The precondition throw stays for a wait with no start
  anywhere in the routine's history and no mark (a programming error, :682-685).
- **A7 Nothing new throws on a run-dependent path**: no throw from the cut rule (it reads `running()` first), from a
  jumped mechanism step, from a wait after a jump, or from a choice applied at the deadline.

**STEP1-B**
- **B1 Fires once**, on the first observer call whose progress reaches the mark (`>=`), and never again on that leg.
- **B2 Never before the leg is live**, never on or after the exit tick (the observer is not called then), and **never
  when the guard reads `running() && expired()`** — the deadline tick P6 found.
- **B3 A leg that ends SETTLED fires its unfired triggers whose mark is ON the leg, as it returns** (D6 (i)) —
  outside the tick, before the next step, unless the guard has expired. *(re-check #3: narrowed.)* "On the leg": the
  mark ≤ the live leg's size + its arrival tolerance — for a drive mark, the straight-line length from the start pose
  captured at the first live tick to the target, plus the EFFECTIVE `translationSettle.maxError` (the robot's, or the
  loosened one, D2); for a turn mark, the size of the start's signed error plus the effective `headingSettle.maxError`.
  A percent mark (≤ 100 %) is always on the leg. A mark past that never fires: the slot is marked (B4), and the drive
  step's line says so once, after the leg ("trigger at 50.0 in did not fire: the drive was 40.0 in"). Any other end
  (TimedOut, Cancelled, the step jumped or skipped) fires none.
- **B4 An unfired trigger marks its slot** "not started: the trigger never fired — the robot did not reach the mark";
  a later `waitUntilDone` answers `MechanismNotStarted` with that reason (ST20), never the verdict of a step the routine
  began on that slot earlier (ST21; P1 (b)'s failure).
- **B5 A zero-size leg** — a `moveTo` whose target is where the robot already is, or *(re-check #13)* a
  `turnTo`/`face`/`faceAway` to the heading it already has — has no direction: nothing fires DURING it (no fraction,
  no along-distance, no turned angle). At its settle B3's bound applies, with no special case: a percent mark fires; a
  distance or angle mark fires only if it is within the leg's arrival tolerance (size 0 + tolerance).
- **B6 A trigger is part of its drive step** (D7 (a)): arming adds no step; a refused start at the fire (ABSENT /
  REFUSED / NOT_HOMED) does not interrupt the leg — the drive step then fails after the leg, `MechanismStepFailed`
  with the verdict, under the drive step's choice. *(re-check #1)* This holds ONLY through §1.2's fire path. The start
  steps' own helpers reach `Chassis::cancel()`, which the scheduler refuses inside a tick: the checker's X3 got a
  CANCELLED leg at 12.47 in with a PRECONDITION fault, or with B7's catch, a throw out of the step after the leg. A
  successful fire counts nothing; the drive step's one record after the leg is the only count.
- **B7 A precondition from the fire — the backstop** *(re-check #1, #9: narrowed)*. Under D8 (a) the start's
  argument checks ran at ARMING, so the fire's command cannot throw one: the checks read only the arguments and the
  robot file. The `try` wraps ONLY the slot's command call, not the bookkeeping, and catches `PreconditionError`
  only; the leg runs to its own verdict and the error is RE-THROWN when the leg returns (`std::exception_ptr`). It
  never reaches the scheduler's catch as a fake "Cancelled" drive (§0.3 item 4), and a programming error stays loud
  (`routine.hpp` :128-132). The panel's tool refuses every such precondition before code exists (G3a R.5). Its
  visibility cost, if D8 (c) is chosen instead: a bad trigger shows only on runs whose leg reaches the mark (D8).
- **B8 Two triggers on the same mechanism in one leg** are refused at arming (a precondition): the second would cancel
  the first mid-leg (the service's newer-command rule, `routine.hpp` :686-693).
- **B9 Arming rules** *(re-check #13, NEW)*. (1) `onFail`/`label` before the first arm pass through to the leg (A2).
  (2) Any call on `r` while a trigger is armed — a step, `onFail`, `label`, or a second `r.at…()` chain — is a
  precondition (the armed handle is the only way on). (3) A precondition thrown by an arm (range, kind, D8's checks,
  B8) or by the leg itself (e.g. `driveTo` to where the robot is, `bearingTo` :791-798; a bad `MotionOptions`) CLEARS
  every armed trigger and the pending modifiers before it propagates, so a host test that catches it finds the
  Routine unarmed and its counters untouched. (4) A Routine destroyed while armed: §1.2's destructor Warn.

---

## §2 The runs, the tests, the mutations, the definition of done

### 2.1 Runs and sizes (each its own go; verified and committed before the next)

| Run | What | Builder-days (rough) | Starts | Notes |
|---|---|---|---|---|
| **STEP1-A** | §1.1 + the rules A1–A7 + §3.1 | **~3–4** (R.13 ~2¾–3½; +¼ for D2's rate rule and the two screen files, P3/P7; label memory and the no-throw tests inside it; *re-check:* +¼ at the top for the mark rules' cases, the onFail-time Warn, the guard-less Warn, the then() rule and the two-line result — the driver line no longer changes) | on its go, after D1–D5 are answered; G3a's table gave it the Nov 2–5 slot | edits `src/programs/` — the seven-build gate |
| **STEP1-B** | §1.2 + B1–B9 + §3.2 | **~3–4** (R.13 ~2¼–3; +¼ for B2's guard check, B7 and the bit-identity test; *re-check:* +½–¾ for the fire's own path, the handle split by kind with its compile refusals, the narrowed settle rule, D8 (a)'s check functions and the arming rules) | when STEP1-A's BUILD ends (both edit `chassis.hpp`); re-reads STEP1-A's tip; after D6–D8 are answered | no `src/` change; under D8 (a) edits `mechanism/service.hpp` (§6) |

Sized against L9 Part A1 (two steps), MECH-A4's step block and MECH-PANEL's own "~1–2 (+~1 with routes)" for the
triggers (:764). Rough: no step modifier exists in this repository yet.

### 2.2 Tests — adversarial: each tries to break ONE rule (G3a's ST1–ST22 numbers kept; new ones from ST23)

**STEP1-A**
| # | The rule it attacks | How |
|---|---|---|
| ST1 | the default is bit-identical | every existing routine, recipe, cookbook and mechanism-step test unchanged; PLUS a golden chain with every step form and three failure kinds whose result fields and RTN lines are literals captured at `eab0f89` |
| ST2 | tolerance 0 changes nothing | `{.positionTolerance = 0_in, .headingTolerance = 0_deg}` vs `{}`: records, exit tick and final pose identical (the 40 in leg and the 90° turn) |
| ST3 | a looser tolerance ends a leg earlier, within it | P3's legs: 2 in → Settled before 1.80 s and within 2 in of the target (truth); 5° → before 1.18 s and within 5°; both plants |
| ST23 | a tighter tolerance never lowers the rate bound; brake untouched; no division by 0 | `loosenedSettle` for 0.25 in / 0.5° / 2 in / 5° (rate unchanged below the robot's own, scaled above); *(re-check #11)* a robot settle of `{e = 0, r = 1}` and `{e = 0, r = 0}` with a 2 in tolerance: rate stays `r` (finite, never `inf`/`NaN`), and a `MoveToPose` built from it does not throw; through the facade: a 0.5° turn on the host test plant settles (does not time out) and `brakeSettle` is the robot's |
| ST4 | skip jumps exactly the steps between and resumes AT the label | steps 3 (fails, skip to "B"), 4, 5, 6 (label "B"), 7: 4–5 jumped (2 Info lines), 6 and 7 run; `jumped == 2` |
| ST5 | a skip that never lands | the label misspelled: every later step jumped, `skipPendingTo == "B"`, nothing commanded after the failure (motor log) |
| ST24 | a skip target behind stops at once — and is said when written | target on step 2 / on the failing step itself: `stoppedAt` = the failing step, the stop line says "behind"; *(re-check #10)* the Warn is logged at the `onFail(skipTo(…))` call on a run where the step SUCCEEDS (nothing else changes: counts, result); a forward target logs no Warn |
| ST6 | carry on goes on and says so | `ok()` false, `failedOn == 1`, first-failed fields, `stoppedAt == 0`, the next step runs |
| ST27 | carry on still safes the drive | the cancel count rises at the failure; the next step starts from rest (wheel speed 0 at its first tick) |
| ST7 | A4's verdict policy under each choice | a counted UNCONFIRMED grab is NOT a failure under carry on (`failedOn` 0); a TIMED_OUT one is |
| ST26 | the modifiers bind to one step only | `onFail(carryOn)` then two failing steps: the second stops the chain; a modifier on a step that is skipped is consumed; two in a row throws with the counters untouched |
| ST28 | labels match whole and exactly | target "B" vs labels "B2", "b", "B " — none lands |
| ST15 | **the deadline always stops** | carry-on and skip chains vs the same chain with no choices, cut (a) mid-motion, (b) mid-frozen-pause, (c) mid-guard-wait, (d) with a skip pending across the deadline: every `RunGuardReport` field equal to the default chain's; P2-B's and P2-C's scripts as regressions (scoring ends within one tick of T+1.00 in (a) and (c); the park succeeds; no step issued after the deadline in (c) and (d)) |
| ST16 | a CANCELLED step stops a carry-on chain | the latch's cut, and an outside replacement (`routine.hpp` :686-693); *(re-check #8)* a `then()` returning `ExitReason::Cancelled`, and one returning `MechanismOutcome::Cancelled`, under `carryOn` and `skipTo`: the chain STOPS; a `then()` returning `false` carries on |
| ST25 | **no throw on any run-dependent path** | a Routine from an `AutonContext` whose guard is NOT running (the tank-steer test's shape) failing a step under each choice; a chassis-only Routine jumping a mechanism step; a wait after a jump; *(re-check #8)* a guard-less Routine logs the one Warn at its first `onFail(carryOn)` / `onFail(skipTo(…))`, once, and a context-built one never |
| ST18 | a wait after a jumped start is a counted failure — and nothing else marks or clears | P1 (a) and (b) as tests: `MechanismNotStarted`, no exception, never SUCCEEDED; a newer start after the jump clears the mark; *(re-check #2)* (c) a jumped `stop(intake)` and a jumped `waitUntilDone(intake)` after a start that RAN: the later wait collects that start's verdict (no false `MechanismNotStarted`); (d) under `carryOn`, two waits after one jumped start: BOTH answer `MechanismNotStarted` (the second never the older step's verdict, never a throw); (e) for that answer `mechanismVerdict == Running` and `lastMechanism().outcome == Running` |
| ST17 | the result as reported | every frozen and appended field under each choice; both programs' word, colour, routine lines and driver line: carry on past 2 → "DONE, 2 FAILED", amber; 10 → "DONE, 9+ FAILED"; a pending skip → ROUTINE STOPPED with its two lines; *(re-check #5, #12)* every word ≤ 15 (the driver slot) and so ≤ `kLargeCols`; the worst case of EACH new line pinned — run/request 99999, 4-digit counts, a 24-character name, a 21-character cause word, the faults tail at its 79 cap — under 199 bytes with `cut` false; the line count 5 (Tester) / 6 (Competition, with the driver-loop line) only when line 3b is written, and unchanged under the default |
| ST8 | the pins | the F10 and F6 pins gain the members D1 names (not the trigger arms — *re-check #4*); each new pin seen failing the build under its own mutation |

**STEP1-B**
| # | The rule it attacks | How |
|---|---|---|
| ST9 | fires once, on the tick the mark is reached | 60 % of a 40 in leg: the start command's tick equals the first tick with progress ≥ 0.60; once |
| ST10 | never before | a 60 % mark on a leg whose timeout ends it at ~58 % (the furthest progress logged): no start command on any tick |
| ST11 | never on a leg that ends early | timed out, cut, the step jumped, the chain stopped |
| ST31 | once, through an overshoot | the lagged plant's leg crosses 100 % then settles back (P4): one fire |
| ST12 | by inches and by degrees | `afterDistance(20_in)`, `afterTurning(45_deg)` on `turnTo` and `face` |
| ST13 | the mechanism runs while the drive goes on | the service ticks during the leg; the lift's plant moves before the leg ends |
| ST14 | a blocking form is refused at build | the compile-refusal harness: `atPercent(60).move(…)`, `.run(…)`, `.set(…)`; a discarded armed handle warns (`-Werror` in the harness, as in the gate and CI; *re-check #7*) |
| ST19 | **the seam adds nothing** | an observer that does nothing: records, exit tick, clock at return and final pose bit-identical to the plain verb; the observer called exactly (Running ticks) times, never on the exit tick |
| ST20 | an unfired trigger's wait is a counted failure | `MechanismNotStarted`, "the trigger never fired", no exception |
| ST21 | never an older verdict | the same mechanism started and finished earlier in the routine |
| ST22 | D15's second question as answered | P4's memoryless leg with a 100 % mark: fires at the settle (D6 (i)) |
| ST29 | **never on or after the deadline tick** | the mark reached exactly on the tick at the deadline (P6's shape): no start command |
| ST30 | a bad trigger is refused AT ITS LINE, never a fake Cancelled drive | *(re-check #9, under D8 (a))* `seesPiece` on a mechanism with no sensor, `stalls` with no signature, an `orAfter` longer than the bound, `startHome` on a `neverHomed` arm, a constant target past the limits: each throws at the ARM with the service's own message, before any motion (no record, the clock unmoved), counters untouched, nothing armed after; the same arguments to the blocking step throw the same message. (Under D8 (c): the draft's case — the leg's own verdict stands, no scheduler `FaultCode::Precondition`, the error thrown after the leg.) |
| ST32 | a jumped or stopped leg arms nothing | triggers armed on a leg the chain skips: no fire, the slot marked for a later wait |
| ST33 | a refused start fails the drive step after the leg — and never touches the drive | *(re-check #1: widened)* each run-time refusal — ABSENT at boot (built in the list, no device), not on this robot (`find()` null), REFUSED (another owner holds the claim), NOT_HOMED (the lift's switch not pressed at boot, the checker's X3 shape): the leg completes SETTLED (not Cancelled), no `FaultCode::Precondition` raised, no exception, the cancel count unchanged during the leg; then `MechanismStepFailed` with that verdict under the drive's choice (default: a stop; `carryOn`: counted, the next step runs). And a SUCCEEDING trigger: `steps` and `completed` rise by exactly one for the drive step |
| ST34 | *(re-check #3, NEW)* a mark of the wrong kind or out of range is refused | compile refusals: `afterDistance(20_in)…turnTo(90_deg)`, `afterTurning(45_deg)…driveTo(…)`, `afterDistance(…)…afterTurning(…)` on one leg; preconditions at the arm: `atPercent(0)`, `atPercent(-5)`, `atPercent(150)`, `atPercent(NaN)`, `afterDistance(0_in)`, `afterDistance(-3_in)`, `afterDistance(inf)`, `afterTurning(0_deg)`, `afterTurning(270_deg)` (stores −90°); `atPercent(100)` and `afterTurning(180_deg)` accepted |
| ST35 | *(re-check #13, NEW)* the arming rules | `r.onFail(carryOn).label("L").atPercent(60).startMove(…).driveTo(…)` failing: the leg is step N labelled "L", carried on; `r.onFail(…)` / `r.label(…)` / `r.pause(…)` / a second `r.atPercent(…)` while armed: precondition; an arm that throws (range, B8, D8) and a leg that throws (`driveTo` to the robot's own point): nothing armed and no modifier pending afterwards, counters untouched; a Routine destroyed while armed logs the one Warn, and one never armed logs none |
| ST36 | *(re-check #3, NEW)* the settle fires only marks ON the leg | the 40 in leg (settles 0.24 in short, P4): `atPercent(100)` fires at the settle; `afterDistance(40_in)` fires (≤ 40 + 0.5); `afterDistance(40.4_in)` fires; `afterDistance(41_in)` and `afterDistance(50_in)` never — a later wait answers "the trigger never fired", and the drive step's line names the mark and the length; the 90° turn: `afterTurning(90_deg)` fires, `afterTurning(120_deg)` never; a zero-size `turnTo` (the robot's own heading): `atPercent(50)` fires at the settle, `afterTurning(0.5_deg)` fires (≤ 0 + 1.15°), `afterTurning(5_deg)` never |

### 2.3 Mutations — each applied, the suite run, RED observed, restored (the coordinator re-runs a subset, §7)

STEP1-A: **SM1** the choice ignored (always stop) → ST4/ST6. **SM2** a label matched by prefix → ST28. **SM3** carry on
leaves `ok()` true → ST6. **SM4** the position tolerance applied to the heading → ST3. **SM9** carry on ignores a
CANCELLED exit → ST15 (a). **SM10** `expired()` not consulted → ST15 (c). **SM11** the word reads `ok` alone → ST17.
**SM12** a jumped start leaves the slot unmarked → ST18 throws. **SM16** the tolerance raises the distance only (R.13's
text) → ST3's drive leg. **SM17** a tighter tolerance lowers the rate bound → ST23. **SM18** the cut rule reads
`expired()` without `running()` → ST25 throws. **SM19** a target behind treated as never-comes → ST24. **SM20** a
modifier not consumed by a skipped step → ST26. **SM21** *(re-check #5, #12: was "the driver line keeps `%.15s`",
moot now that it does not change)* line 3b not written — a carried-on run's log names no failed step → ST17. **SM22**
carry on skips the cancel → ST27. **SM27** a pending skip ignores the deadline → ST15 (d). *(re-check, new:)* **SM29**
a `waitUntilDone` that runs clears the mark → ST18 (d) (the second wait answers the older verdict). **SM30** a jumped
`stop`/`waitUntilDone` marks the slot → ST18 (c) (a false `MechanismNotStarted`). **SM33** the `onFail`-time behind
check removed → ST24's Warn case. **SM34** a `then()` returning `Cancelled` follows the choice → ST16. **SM35** the
guard-less Warn removed → ST25. **SM36** `loosenedSettle` divides by `e` unguarded → ST23's `e = 0` cases.

STEP1-B: **SM5** `>` for `>=` at the mark → ST9. **SM6** fires on a timed-out leg → ST11. **SM7** fires twice →
ST9/ST31. **SM8** a blocking form accepted → ST14. **SM13** the wait after an unfired trigger answers the earlier
verdict → ST21. **SM14** the observer called once more on the exit tick → ST19. **SM15** the settle-short rule inverted
→ ST22. **SM23** the seam built on `waitUntil` → ST19's bit-identity. **SM24** the trigger does not ask the guard →
ST29. **SM26** the catch around the fire removed → ST30 — *(re-check #9)* under D8 (a) the fire cannot throw, so
SM26 is observed ON TOP OF SM32: with the arm's check removed, ST30 goes red the backstop way (the error thrown after
a SETTLED leg, no scheduler PRECONDITION fault — logged); with the catch removed too, it goes red the scheduler way (a
CANCELLED leg + a PRECONDITION fault — logged). *(re-check, new:)* **SM28** the fire reuses the start step's recording
(`startMove` → `positionStep` → `startedMechanism`/`notOnThisRobot` → `recordStop`) → ST33 (a Cancelled leg or a
throw, and a SUCCEEDING fire counts a step). **SM31** the settle fires every unfired trigger (the draft's B3) →
ST36 (`afterDistance(50_in)` fires). **SM32** the arm skips the D8 check → ST30. **SM37** the kind mask ignored (every
leg offered after every arm) → ST34's compile refusals compile. **SM38** an arm's precondition leaves its triggers
armed → ST35.

### 2.4 Definition of done — every run

1. The suite green through `python3 tools/suite_run.py green`; the count before and after in the report.
2. Every mutation of §2.3 for the run applied, RED seen, restored; the log names the test that turned red.
3. STEP1-A: `python3 tools/src_build_gate.py check` — the seven builds — green (it edits `src/programs/`); both runs:
   the ARM header gate, the CI guards, `api_doc_tool` coverage/fresh/examples (§7).
4. The F6/F10 pins carry every new member D1 names — the trigger arms and their handles excepted, unpinned like the
   start forms (*re-check #4*); the register rows and `version.hpp` in the same commit; the briefing's pin
   counts regenerate (F6 48, F10 59 at `eab0f89`, `docs/internal/PROJECT-BRIEFING.md` :144).
5. Documents in the same commit (§3), the prose review's greps logged (§3.3).
6. No hardware belief beyond HA-216..HA-218 without stopping to ask.
7. A report that lists every file changed, the counts, each mutation's red test, and anything left.

---

## §3 Documents — in the same change as the code (pace rule 3), the prose review logged (rule 4), ≤ ~5 public each (rule 7)

### 3.1 STEP1-A
- **Public:** `docs/guide/09-the-recipe-api.md` (the three choices, `label`, the two rules above them, the tolerance and
  what it loosens — in D2's words); `docs/cookbook/02-when-a-step-fails.md` (recipes "carry on past a missed piece" and
  "give up on this piece, go for the next" — compiled by `test/cookbook_examples_test.cpp`); `docs/changelog.md` (API
  2.6); `docs/guide/16-testing-an-auton-on-the-bench.md` and `17-the-competition-program.md` (the new word and its
  colour, where they list the words); `docs/roadmap.md` F6 and F10 rows (one sentence each; the F10 pin count). That
  is six files, two of them one-line quote edits — one over pace rule 7's "~5"; recommended: keep all six (a guide
  that lists the result words and misses the new one is wrong the day it lands). *(found on the re-check, not a
  checker finding)* **Plus `docs/guide/10-the-api.md`** :338-340, whose Options paragraph lists `MotionOptions`' fields
  one by one ("`timeout` …, `maxLinearSpeed` …, `maxAngularSpeed` …"): it gains the two tolerances in one sentence,
  or it is wrong the day they land. Seven files, three of them one-sentence edits.
- **The recipes are written in the context form** *(re-check #8)*: every NEW recipe and guide example that uses
  `onFail` is `Routine r{ctx, "…"}` inside an auton taking `AutonContext& ctx` — the chassis-only form
  (`Routine r{chassis, …}`, which cookbook 02 :19/:91, 03 :124/:158 and guide 09 :75/:292/:354 teach today, and
  `acceptance_run.hpp` :40 uses) has no deadline rule beyond a CANCELLED verdict. Guide 09 says that in one sentence,
  and that such a Routine Warns once at its first choice (A5). The existing recipes are not rewritten (they use no
  choice; ST1 holds them).
- **Generated:** `docs/api/` regenerated (`tools/api_doc_tool.py`).
- **Internal:** `docs/internal/chunks/STEP1-PROGRESS.md` (new), the brief → `docs/internal/chunks/` with the answers
  block (the coordinator's plan commit), `REMAINING-WORK.md` row 20c, `build-order.md`, `HANDOFF-remaining-work.md`;
  the plan commit also strikes MECH C's "mid-drive triggers for routines" (`MECH-PANEL-requirements.md` :764, G3a C10).

### 3.2 STEP1-B
- **Public:** guide 09 (triggers, D6/D7/D8 in plain words, the kinds, the ranges, the arming rules); `docs/cookbook/07-mechanisms.md` ("start the lift partway
  through a drive", compiled, in the context form); the changelog (API 2.7); the roadmap's F6 row (the overloads,
  `LegObserver`) and F10 row (one sentence: the three arms are added and stay OPEN like the start forms — *re-check
  #4*); guide 10's Tier-3 section (one sentence: the observer overloads); `docs/guide/14-what-it-cannot-do-yet.md` if it
  lists mid-drive triggers (the run greps — L6-A1, MECH-A6 and VIS1-A1 all edit guide 14 first, §6).
- **Code comment:** `runner/mechanism_routes.hpp` :19-23.

### 3.3 The prose review's greps (each run runs them and logs the hits it read)
`grep -rn -i 'simulator' docs/guide docs/cookbook` (no new hit: "host test plant"); `grep -rn 'ROUTINE STOPPED'
docs/guide docs/cookbook` (each still true); `grep -rn 'branches on the verbs directly\|continue past' include docs`
(each updated or still true); `grep -rln 'internal/\|chunks/\|RESUMING' docs/*.md docs/guide docs/cookbook` (empty);
`grep -rn 'partway\|mid-drive\|mid-leg' include docs/guide docs/cookbook` (STEP1-B: each true).

---

## §4 Hardware Assumptions — HA-216..HA-218 ONLY (the register's Held line gains them at the plan commit)

Next free at `eab0f89` is HA-216 (`docs/hardware-assumptions.md` :39's Held line ends at HA-215). **Collision rule:**
the in-flight CAL1-A2b and L6-A1 runs are told "next free HA-216" (`.claude/coordinator/drafts/chunk-build.cal1-a2b.js`,
`chunk-build.l6-a1.cont.js`), and the string-lift brief (MECH-S/TUNE1) is next in the queue. Whichever registers or is
accepted first keeps HA-216; this brief's three shift by a mechanical rename, its labels kept, as IMU1 did.

| ID | The belief | Why the host test plant cannot settle it | Confidence | Settles by |
|---|---|---|---|---|
| **HA-216** | A leg that ends under a loosened tolerance — moving up to the scaled speed bound (4 in/s at 2 in; *re-check #11:* a TURN up to 1.31 rad/s, ~75°/s, at 5°) — comes to rest within that tolerance on robot two. | On the exit tick the motion sets the drive motors to 0 V (`move_to_pose.hpp` :522-526); the A2 plant P3 used stops dead then (kA 0), robot two keeps moving on its own momentum under whatever brake mode its drive motors hold — unmeasured. | reasoned | an Auton Tester run of the 40 in leg at 2 in, tape-measured, and of the 90° turn at 5°, the END heading against the IMU at rest (a Station M-style floor run) |
| **HA-217** | A partway trigger's mechanism starts within ~1–2 ticks (10–20 ms) of the estimate crossing the mark: the command is issued in the tick, the service acts on it at the next pace. | The host's service and motors answer the same tick; robot two's motor response and task timing are unmeasured. At 40 in/s each tick is 0.4 in of drive. | reasoned | a bench run logging the mark tick against the mechanism's first motion |
| **HA-218** | D6 (i) puts the mechanism within the leg's tolerance of the author's mark because the ESTIMATE says the leg arrived — true only as far as the estimate is (odometry drift over the leg). | The plant's estimate tracks truth closely (P3: estimate and truth agree to 0.01 in); robot two's drift is the odometry entries' question. | reasoned | the same floor run, the estimate's END against the tape |

No other belief is expected. A run that finds one stops and asks.

---

## §5 OUT, and explicitly rejected

- **CURVE1** — a true curve follower (G3a D16 (c), the team lead's answer, :5-10): its own brief and go, AFTER the route
  maker; it also edits `MotionOptions` (after STEP1-A, which therefore keeps its two fields at the end of the struct).
- **Rolling through points** (D16 (b)) — not taken.
- **Anything of the panel** — the route file, the tool's refusals (the definitely-started check, backward skips, marks
  past the planned leg), the generator, the F8 export, the page: G3a Parts R1 and R2.
- **F8 / `config/route.hpp` and `PathRunner`** — untouched: markers stay blocking and fire on Settled (R2); the
  in-memory types do not change (D13 (c)).
- **A per-step choice in `MotionOptions`** — D1 (b), rejected.
- **A backward skip** — D3 (c), impossible in an eager chain.
- **A guard-aware `pause` step on the Routine** — now a few lines (the Routine knows the guard after STEP1-A), but a new
  verb the generator does not need (G3a R.5 emits `r.then([&]{ return ctx.guard.pause(t) == …; })`); a later small
  change on the team lead's word.
- **Triggers on `strafeTo` and on `followTrajectory`** (several legs) — not in the route maker's vocabulary for a tank;
  additive later.
- ~~**Validating a trigger's start at arming through the service** — would edit `mechanism/service.hpp`, which the
  string chunks edit; B7 (catch and re-throw after the leg) instead. Revisit after MECH-S.~~ *(re-check #9: WITHDRAWN —
  the reason was wrong: the reads a check needs are public (`Slot::declaresPieceSensor()` `service.hpp` :306, `spec()`
  :219, `MechanismService::slotFor()` :1818, `Until`'s queries), so only the no-drift option edits `service.hpp`. The
  real choice is now D8; (a), arming through the service's own check functions, is recommended.)*
- **The existing "ROUTINE STOPPED" for an ok routine whose PARK failed** (`ranOk && endActionSucceeded`,
  `auton_tester_run.hpp` :248) — a quirk found while reading, not STEP1's to change (§8, L12).

---

## §6 Coordination — in flight while this was written (read-only at 2026-10-30 ~evening; NOT edited)

| Work | Where | Shares a STEP1 file? |
|---|---|---|
| **G3a Part 1** (in review, `g3a-p1` at `4691757`) | tools, the robot file, docs | **no `include/`**; shared: `docs/roadmap.md`, `docs/guide/15-glossary.md`, READMEs, and *(re-check #14)* `test/CMakeLists.txt` (both STEP1 runs add test files to it). Part 5 next — no `include/` |
| **G3a Part R1** (after Part 5's build; its emitters need STEP1-A/B "by its tip", G3a R.14) | `tools/robot_routes.py`, `src/team/*autons.cpp` region | none — but it EMITS STEP1's spellings: D1's and §1.2's must be fixed BEFORE R1 is briefed. *(re-check #15: one path picked, not a fallback.)* By G3a's own table R1 BUILDS ~Nov 8–11 and STEP1-B merges ~Nov 14–15 (below), so R1's trigger emitters cannot be run by S53 ("nothing generated throws") during R1's build. **Recommended: R1 builds WITHOUT triggers** (stop / skip / carry on and the tolerances need only STEP1-A, merged before R1's build starts), and **a small follow-up, R1-T** — the trigger emitters and their S53 fixtures — **runs after STEP1-B merges** (G3a's to size; rough ~½–1 day; D20's slip order already lets triggers slip first). The alternative — R1's emitters written against §1.2's spellings, their S53 fixtures run in R1's review against STEP1-B's unmerged tip — couples R1's verdict to a branch still in review. REMAINING-WORK row 20c ("before G3a Part R1") is corrected at the plan commit to "STEP1-A before R1's build; STEP1-B before R1-T". R1-T, like STEP1-B, lands after ~Nov 16 only on the team lead's word (G3a R.11) |
| **MECH-A6** (building, `mech-a6` from `81eb666`) | `src/programs/bench_r3a.*`, the worksheet | no STEP1 code file; *(re-check #14)* shared docs: `docs/changelog.md`, `docs/roadmap.md`, `docs/guide/14-what-it-cannot-do-yet.md`, `docs/guide/10-the-api.md`, the HA register — whichever lands second rebases its own lines |
| **CAL1-A2b** (in review, `cal1-a2b`) | calibration headers, docs | changelog, roadmap, HA register — whichever lands second rebases its own lines |
| **L6-A1** (verify, `l6-a1`) | `localization/`, `chassis/wall_reset_call.hpp`, `sim/field_plant.hpp` | no STEP1 header; changelog/roadmap/HA register; *(re-check #14)* also `test/cookbook_examples_test.cpp` (which compiles STEP1-A's and STEP1-B's new recipes), `docs/cookbook/05-mixing-tiers.md`, `docs/guide/10-the-api.md` and `docs/guide/14-what-it-cannot-do-yet.md` (STEP1-B greps and may edit 14). **L6-A2** (next) edits `motion_scheduler.hpp` and `robot_context.hpp` — not STEP1's |
| **VIS1-A1** (committed on main as `9d77c9d` DURING the re-check, 2026-11-03; an L6-A1 merge was then in progress on main — main moves under this brief, re-check #6) | `vision/`, docs | changelog/roadmap/HA register, `docs/guide/10-the-api.md`, `docs/guide/14-what-it-cannot-do-yet.md` — on main before either STEP1 run branches |
| **IMU1-B** *(re-check #14: missing from the draft)* (`imu1-b`, from `eab0f89`) | `src/programs/bench_r3a.cpp` only | no overlap |
| **ALLY1, L6-B, AF1** (later) | `src/programs/{auton_tester_run,competition_run}.hpp` | **yes** — whichever lands second re-reads (G3a's shared-files table, :1363) |
| **MECH-S / TUNE1** (brief next) | `mechanism/service.hpp`, kinds | *(re-check #9)* under D8 (a) STEP1-B DOES edit `service.hpp` (the four check functions, a pure extraction): whichever lands second re-reads; a start command's NEW precondition from MECH-S goes INSIDE its check function, so triggers check it at arming. Under D8 (b)/(c) not edited. `version.hpp` shared — the coordinator numbers at merge |
| **CURVE1** (after the route maker) | `chassis.hpp` `MotionOptions` | after STEP1-A |

- **Order:** STEP1-A → STEP1-B (both `chassis.hpp` and `routine.hpp`); STEP1-B re-reads STEP1-A's tip. The rebase rule
  for every row above is unchanged: whichever lands second re-reads the shared file and rebases its own lines.
- **The code freeze (~Nov 16) and the first competition (Nov 23):** STEP1-A changes the Competition program's result
  screen (~Nov 8 by the table) — inside the freeze window's lead-in; ST1 and ST17 are its guard. STEP1-B (merge ~Nov
  14–15 by the table) changes no `src/`, but every library header change rebuilds robot two's packages: a STEP1-B that
  slips past Nov 16 lands only on the team lead's word (G3a R.11's freeze rule, applied to the library).
- **Dates (from G3a's table, :1286-1300, plus this brief's +½):** STEP1-A build Nov 2–5, review Nov 6–8, merge ~Nov 8–9;
  STEP1-B build ~Nov 9–12, review ~Nov 12–14, merge ~Nov 14–15. Each lost day moves each date a day. *(re-check: the
  table's dates are kept as the plan's; each run's real start is its go, and the sizes above run from there — the
  ORDER, not the pace, is what #15 fixes.)*

---

## §7 Checks — the coordinator re-runs every one after each run (RESUMING's step 3: never the report's word)

Every binary under the memory rule. `<scratch>` is the coordinator's scratch folder — never `/tmp`.
```sh
cmake --build build/test -j"$(nproc)"
python3 tools/suite_run.py green
ctest --test-dir build/test -R compile_refusal --no-tests=error --output-on-failure | tail -3   # STEP1-B's ST14
# the ARM header gate — every v2 header in ONE TU
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > <scratch>/all.cpp
echo "int main(){return 0;}" >> <scratch>/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c <scratch>/all.cpp -o /dev/null -Iinclude
grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib | grep -v '^include/shulib/hal/pros/'   # empty
grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib              # empty
python3 tools/src_build_gate.py check                       # STEP1-A (src/programs/ changed); STEP1-B too (headers)
python3 tools/api_doc_tool.py check-coverage && python3 tools/api_doc_tool.py check-fresh && python3 tools/api_doc_tool.py check-examples
python3 tools/doc_staleness_audit.py && python3 tools/briefing_status.py check
# ── the brief's own ── (re-check #6: every diff against the RUN'S OWN BASE, never eab0f89 — main moves under
#    MECH-A6, G3a Part 1, L6-A1, VIS1-A1 and the string chunks, and STEP1-A itself edits src/programs/)
BASE=<STEP1-A: `git merge-base step1-a <main at its go>`, its branch point | STEP1-B: STEP1-A's merged tip, its branch point>
git diff $BASE -- include/shulib/config/route.hpp include/shulib/runner/path_runner.hpp | wc -l   # 0 (F8 untouched)
git diff $BASE -- include/shulib/mechanism/service.hpp | wc -l   # STEP1-A: 0; STEP1-B: 0 under D8 (b)/(c), under D8 (a) READ it: only the four check functions extracted, same checks, same order, same messages
git diff $BASE -- test/routine_signature_pin_test.cpp test/f6_signature_pin_test.cpp | grep -E '^-[^-]' | grep -v '^-\s*//'   # empty: no existing pin edited
git diff $BASE -- test/routine_signature_pin_test.cpp | grep -E 'atPercent|afterDistance|afterTurning'   # STEP1-B: empty — the arms stay unpinned (re-check #4)
grep -n 'MechanismNotStarted' include/shulib/chassis/routine.hpp | head -2            # appended AFTER MechanismStepFailed, == 6 pinned
grep -n 'running()' include/shulib/chassis/routine.hpp                               # the cut rule's guard read (A5, SM18)
grep -n 'runBlocking\|sched_.waitUntil' include/shulib/chassis/chassis.hpp   # STEP1-B: the two observer overloads go through runBlocking (P5); sched_.waitUntil stays in wait() and waitUntil() only
grep -n 'auton %.15s: %.27s' src/programs/competition_screen.hpp                     # STEP1-A: still there — the DONE words fit it (re-check #12; under D5 (b): '%.18s: %.24s')
git diff --stat $BASE -- src/                                                         # STEP1-B: empty
```
Plus by hand: **STEP1-A** re-run SM9, SM16, SM18, SM29 (seen red, restored, green); read `effectiveConfig`'s new lines
against D2's formula, `e ≤ 0` included. **STEP1-B** re-run SM23, SM24, SM28, SM31 (and SM26 on top of SM32 under
D8 (a)); read the decorator: it forwards `name()` and never calls the observer when the inner tick returned anything
but `Running`; *(re-check #1)* read the fire function: no call to `skipIfStopped`, `recordSuccess`, `recordStop`,
`startedMechanism`, `awaitMechanism`, `notOnThisRobot`'s recording half or any `cancel()` is reachable from it.

---

## §8 Landmines

- **L1 The rate bound decides drive legs** (P3). A tolerance that moves `maxError` alone is a placebo on every drive
  leg — and every existing test would still pass. SM16 is the guard.
- **L2 A tighter tolerance must not tighten the rate** — the heading rate floor sits ~2.5σ above the IMU's noise
  (`motion_config.hpp` :36-40); a 0.5° tolerance with a proportionally smaller rate bound could never settle on a
  noisy IMU and would time out every turn. Reasoned, not probed (ST23 pins the formula).
- **L3 `expired()` throws with no run live** (P2-D, `run_guard.hpp` :401). The tree builds `AutonContext`s over idle
  guards (`motion_tank_steer_test.cpp` :510, :526; `mechanism_auton_run_test.cpp` :357). Read `running()` first.
- **L4 One tick at the deadline** (P6). A trigger that trusts the drive's verdict fires at T+deadline; ask the guard.
- **L5 The scheduler's catch** (`motion_scheduler.hpp` :1123-1132) turns any `PreconditionError` from inside a tick into
  a Cancelled drive with `FaultCode::Precondition` — and under A5 a Cancelled step stops the chain. B7's catch must be
  `PreconditionError` only (anything else stays the scheduler's).
- **L6 `waitUntil` paces after the exit tick** (P5). The seam must wrap the motion, not wait on it.
- **L7 The decorator must forward `name()`** or every record and result line renames the motion ("MoveToPose" is read
  by tests and by `lastCompleted().name`, e.g. `motion_tank_steer_test.cpp`).
- **L8 A jumped mechanism step in a chassis-only Routine** must not call `mechs()` (it throws without a service,
  `routine.hpp` :910-916). Use `mechs_ != nullptr` and `find()`.
- **L9 Labels and targets are stored pointers** — literals only, like the routine's name; a `std::string`'s `c_str()`
  dangles. Say it in the docs; the generator emits literals.
- **L10 `ok()` is a frozen observer** — change its body, not its meaning; under the default it must stay `stoppedAt_ ==
  0` in effect (ST1's golden chain).
- **L11 The 200-byte lines** — COMP-b's routine-stop line is already 214 B at its worst and CUT (NOTES 10-29 ~21:00);
  STEP1-A must not lengthen it and every new line must fit (ST17). *(re-check #5)* The draft's one-line carry-on form
  broke exactly this: 225 / 229 bytes at 4-digit counts. The amended two-line form (§1.1) fits at every extreme;
  computing the worst case BEFORE the go is the rule, never "tested" as a promise.
- **L12 Found, not fixed:** an ok routine whose park FAILED reads "ROUTINE STOPPED" (`auton_tester_run.hpp` :248,
  `competition_run.hpp` :550). Left as is; logged for the team lead's later word.
- **L13 The words are compared as strings** (`competitionRunLevel` :555-565, the two tone functions): a new word must
  be added to each place that names words, or it reads RED and logs as the wrong level.
- **L14 `-Wswitch`:** the two stop-word switches have no default — appending `MechanismNotStarted` fails their builds
  until they gain the case. Good; do not add a default.
- **L15 HA numbers** — see §4's collision rule.
- **L16 The fire must not reuse the start steps** *(re-check #1)*. `positionStep`/`homeStep`/`powerStep`/`speedStep`
  end in `startedMechanism`/`notOnThisRobot` → `recordStop` → `Chassis::cancel()`, which `MotionScheduler::cancel`
  refuses inside a tick (`motion_scheduler.hpp` :857-859). Reusing them turns a run-time state (not homed, held,
  absent) into a cancelled drive or a mid-match throw (the checker's X3), and a successful start into a counted step.
  §1.2's fire path; ST33; SM28.
- **L17 The not-started mark** *(re-check #2)*. Set only by a jumped STARTING step; cleared only where `noteStarted`
  runs; a wait never clears it. Any looser rule gives a false stop (a jumped `stop`) or C3's stale verdict (a second
  wait). ST18 (c)–(e); SM29, SM30.
- **L18 A settle fire is bounded by the leg** *(re-check #3)*. "Fire every unfired trigger at the settle" starts a
  mechanism where the author never meant — what R2 calls worse than not firing (`path_runner.hpp` :45-50). ST36; SM31.
- **L19 A guard-less Routine has half the cut rule** *(re-check #8)*. `Routine r{chassis}` and `r{ctx.chassis}` keep
  only the CANCELLED rule; teach the context form, and the one Warn says so on the bench. ST25; SM35.
- **L20 `-Werror` is the gate's, not `make`'s** *(re-check #7)*. A discarded armed handle uploads from a student's
  laptop; the next-step precondition and the destructor Warn are what make it visible there.

---

## §9 Summary

STEP1 gives every `Routine` step the care the team lead asked for: an arrival tolerance that actually shortens a leg
(the probe showed the distance alone does nothing — D2 widens the speed bound in proportion), and a per-step choice of
stop / carry on / skip ahead to a named step, written `r.onFail(carryOn)` / `r.onFail(skipTo("x"))` / `r.label("x")` in
front of a step (D1), with today's rule as the untouched default. Two rules sit above every choice: the deadline (or a
CANCELLED verdict, a `then()`'s included) always stops, and nothing on a run-dependent path throws — a wait after a
skipped start or an unfired trigger is a counted failure that says why (D4), set and cleared by exact rules (A6). STEP1-B
adds triggers partway through a drive through a new F6-additive per-tick observer that wraps the motion (bit-identical
when unused), firing once through its OWN short path (never the start steps' recording, which would cancel the drive
from inside its tick), never on the deadline tick, never on a leg that ends early, and — D6 (i), narrowed — as the leg
arrives if the mark is on the leg. Marks are checked where they are written: kind (inches on drives, degrees on
turns, at compile time), range, and the start's own arguments (D8). The brain's screens learn "DONE, 2 FAILED" in
amber, and the log names the first failed step on a line of its own (D5). Two runs, ~3–4 and ~3–4 builder-days
(rough), STEP1-A before G3a's R1 builds, STEP1-B before R1's small trigger follow-up (R1-T); HA-216..HA-218 reserved.

### The decisions, as I would put them to the team lead (plain words) — as amended at the re-check

1. **D1 — Where does "if this fails" go?** Recommended: a short line in front of the step — `r.onFail(carryOn)` or
   `r.onFail(skipTo("third piece"))` — and `r.label("third piece")` to name a step. Locked into the frozen surface the
   day it lands, because the panel writes code against it. (Not as a speed-and-timeout style option: most steps don't
   have those options, and the driving commands underneath have no auton to carry on in.) The trigger words
   ("at 60 %", "after 20 in", "after 45°") are NOT locked, matching your 2026-10-29 ruling that the mechanism steps
   stay open.
2. **D2 — What does "arrive within 2 inches" loosen?** Recommended: both the distance AND how slowly the robot must be
   moving, in proportion. The test showed the distance alone changes nothing on a drive (1.80 s at 0.5 in and at 4 in
   alike); with both, the 40 in drive finishes in 1.35 s instead of 1.80 s. A tolerance tighter than the robot's own
   tightens only the distance. Unmeasured on robot two: how far it coasts after a drive or turn that ends while still
   moving a little.
3. **D3 — "Skip to step X" when X is behind, or misspelled?** Recommended: skips only go forward. If X already ran,
   the auton stops there and says "behind", and the log warns on the line where the skip is written, so the bench sees
   it first. If X never comes, nothing more runs and the result says the skip never landed. A skip still looking for X
   when time runs out stops.
4. **D4 — A start that a skip jumped, then "wait until it's done"?** Recommended: the wait fails, counted, saying "its
   start was skipped". It never crashes (today it crashes the auton, or says "done" from an older run of that
   mechanism), and the wait then follows its own choice. Only a skipped START causes this, and only a start that
   actually runs clears it; a second wait gives the same answer.
5. **D5 — What does the screen say after carrying on?** Recommended (changed at the re-check): "DONE, 2 FAILED" in
   amber, and "DONE, 9+ FAILED" for ten or more. It fits every screen slot as it is today, including the driver
   page's, and the large-text width it must fit has never been measured on a V5. "FINISHED" stays the word for a
   clean run. The log names the first failed step on its own line. (The draft's "FINISHED, 2 FAILED" is the
   alternative: it sits exactly at the estimated width and needs the driver page widened.)
6. **D6 — The drive arrived but stopped just short of the trigger's mark: fire?** Recommended: yes, as it arrives, if
   the mark is on the drive (no further than its end plus its tolerance). A mark past the end, like "after 50 in" on a
   40 in drive, never fires. A drive that ends any other way still never fires it (your D15).
7. **D7 — What does "60 % of the drive" measure?** Recommended: how far the robot has come along the straight line
   from where the drive started to its target (for a turn, how far it has turned). A trigger is part of its drive: if
   the mechanism refuses to start (unplugged, not homed, busy), the drive finishes and then counts as failed. The
   trigger never stops the drive mid-way.
8. **D8 — A trigger written wrong (say, "until it sees a piece" on a mechanism with no sensor): when is it caught?
   (new)** Recommended: on the line that sets the trigger, every run, by the mechanism's own checks shared in one place
   (the string-lift chunk then shares that file). Otherwise it shows only on runs where the drive reaches the mark, and
   a match could be the first.

**Also asked (gos, not decisions):** the go for STEP1-A, and later for STEP1-B; leave for the coordinator to reserve
HA-216..HA-218 at the plan commit; and the plan change that R1 builds without triggers, with a small R1-T after
STEP1-B (row 20c corrected).

---

## Re-check — amendments (2026-11-03)

A checker read this brief against `eab0f89` and returned fifteen findings (five major, ten minor). I re-checked each one
against the code at `eab0f89` (READ-ONLY), the checker's probes, and my own. The checker's tree (`check/tree`) is
byte-identical to `eab0f89` for `routine.hpp`, `motion_scheduler.hpp`, `service.hpp` and `chassis.hpp` (checked with
`cmp`). I re-ran X3 and X1/X2 under the memory rule (`recheck/x3_rerun.log`, `recheck/x1_rerun.log`; same output as
the checker's) and recomputed the line budgets (`recheck/r5_line_budget.py`/`.log`, `recheck/r5_final_forms.py`/`.log`;
all under `.claude/coordinator/scratch/brief-STEP1/`). **All fifteen hold up; #3 and #12 only in part. Each is fixed in
the body, marked *(re-check #n)*.** No finding was rejected.

| # | Sev. | Finding (short) | Re-verified — evidence | Disposition — where fixed |
|---|---|---|---|---|
| 1 | major | A trigger's start through the existing start-step code becomes a mid-match throw on a run-time state | **REAL.** The only start path is `positionStep`/`homeStep`/`powerStep`/`speedStep` → `startedMechanism`/`notOnThisRobot` → `recordStop` → `chassis_->cancel()` (`routine.hpp` :853-860, :955-1075); `MotionScheduler::cancel` has `SHULIB_PRECONDITION(!inTick_, …)` (`motion_scheduler.hpp` :857-859). X3 re-run: a not-homed lift → leg CANCELLED at 12.47 in + 1 PRECONDITION fault (no catch); with B7's catch → leg Settled, then "MotionScheduler::cancel: cannot cancel from inside a tick" re-thrown, `stoppedAt=1` half-recorded. A successful start calls `recordSuccess` → counts a step | FIXED: §1.2 "The fire's own path" (find at arm → command → `noteStarted` → keep the report; none of the step machinery inside the tick; ONE record after the leg); B6, B7 narrowed; D7 (a) says why; §0.3 item 4; L16; ST33 widened to ABSENT-at-boot / not-on-this-robot / REFUSED / NOT_HOMED + the no-extra-step count; SM28; §7's by-hand read |
| 2 | major | A6's mark rules give false stops or C3's stale verdict | **REAL.** "start form or blocking" includes `stop` and `waitUntilDone`; "a later step that RUNS clears the mark" includes a wait that answers `MechanismNotStarted`, after which a second wait reads `startedSeq_` from the older step (`routine.hpp` :682-693): stale verdict, or the throw. ST18/SM12 tested neither; `mechanismVerdict`'s doc (:244-249) covers only `MechanismStepFailed` | FIXED: A6 rewritten (SET only by a jumped starting step; CLEAR only where `noteStarted` runs; a wait never clears; the values of `mechanismVerdict` (`Running`) and `lastMechanism()` (the default report) stated); D4 says it; L17; ST18 (c)–(e); SM29, SM30 |
| 3 | major | B3/D6 (i) fires at the settle triggers the robot never reached; kinds can mismatch; no range checks | **REAL** for the gaps: §1.2's armed handle offered every leg after every arm; B3 said "fires its unfired triggers"; no arming checks. **In part**: the fix's bound "length + arrival tolerance" is the checker's; I kept it over the stricter "≤ length" because it absorbs the planned-vs-live length gap (a leg that starts a little past the planned start) and depends only on the leg's geometry, not on where the robot stopped (P4's plant-to-plant variance) | FIXED: §1.2 handle split by kind (compile-time, kinds mask), arming preconditions; B3 narrowed (bound defined from the live start pose and the EFFECTIVE settle `maxError`); B5 now follows B3 (zero-size turns included); D6 reworded so he answers the narrow rule; L18; ST34, ST36; SM31, SM37 |
| 4 | major | The brief pinned the three trigger arms against the "leave the mechanism steps open" ruling | **REAL.** §1.2 Tests said "CHANGED the two pins (… the three arms)"; the F10 row (`docs/roadmap.md` :144) keeps the START forms OPEN and unpinned; nothing asked him | FIXED: D1 now says what is pinned (onFail/label/`OnFail`/`MechanismNotStarted == 6`; F6: the two overloads and `LegObserver`'s two members) and that the arms and handles are NOT pinned; `LegProgress` stays additive-open; §1.2 Tests; §3.2's F10 sentence; §7 grep (no arm in the F10 pin diff) |
| 5 | major | The proposed carry-on line cannot meet the 199-byte budget | **REAL.** Recomputed: 225 (Tester) / 229 (Competition) at 4-digit counts; 210/214 at 1-digit; today's STOPPED form 192/196 (the checker's script) or 193/197 at run 99999. Neither program has a free line slot: the Tester fills `line[4]` on a routine run, Competition fills `line[5]` with the driver-loop line (`auton_tester_run.hpp` :295-300, :315-436; `competition_run.hpp` :585-589, :767-782). Line 1's word growth is harmless: the new word requires "in time … succeeded" (the word's own conditions) | FIXED: §1.1 two lines (summary 155/159; first-failed 104–138; both fit at every `int` extreme, 189–195/131–150); `line[4]`→`[5]`, `line[5]`→`[6]`, written only when something carried on; L11; ST17 pins each worst case; SM21 redefined |
| 6 | minor | §7 diffs against `eab0f89` | **REAL.** STEP1-A edits `src/programs/`; MECH-A6, G3a Part 1, L6-A1, VIS1-A1 and the string chunks move main | FIXED: §7 — every diff against `$BASE`, the run's own branch point (STEP1-B: STEP1-A's merged tip); `service.hpp`'s check split by D8 |
| 7 | minor | "Fails the build (`-Werror`)" is true only in the gate and CI | **REAL.** `Makefile` :31-53: warnings on WITHOUT `-Werror`; the gate fails on warnings | FIXED: §1.2 wording; the next-step precondition stays; NEW: a destructor Warn for a Routine left armed (noexcept, swallowed log failure); B9 (4); L20; ST14 wording, ST35 |
| 8 | minor | A guard-less Routine gets half the cut rule; the docs teach that form; `then()` unstated | **REAL.** `Routine r{chassis, …}` in cookbook 02 :19/:91, 03 :124/:158, guide 09 :75/:292/:354; `acceptance_run.hpp` :40 uses `r{ctx.chassis, …}`; `then()` accepts `ExitReason` and `MechanismOutcome` (`routine.hpp` then()) with no CANCELLED rule stated | FIXED: A5 — `then()` returning either `Cancelled` is CANCELLED; the guard-less Routine's rule stated; one Warn at its first choice; how a guard-only program gets the rule; §3.1 recipes in the context form + one guide sentence; L19; ST16, ST25; SM34, SM35 |
| 9 | minor | B7's alternative was rejected for the wrong reason; B7's cost unstated | **REAL.** `declaresPieceSensor()` (`service.hpp` :306), `spec()` (:219), `slotFor()` (:1818) are public; every command precondition reads only arguments + spec + `peer()` (:512-547, :779-786, :855-861, :1317-1325) — none is run-dependent | FIXED: NEW **D8** (a) one list — the service's checks extracted into check functions the command and the arm both call (recommended); (b) a copy + paired test; (c) the draft's B7 alone, with its visibility cost stated. §1.2 service.hpp line; B7 narrowed to a backstop; §5's rejection withdrawn; §6 MECH-S row; ST30 rewritten; SM32; SM26 observed on top of SM32 |
| 10 | minor | A backward skip target is detectable when `onFail` is written | **REAL.** In straight-line code the labels passed before the line are fixed | FIXED: D3, A2 — one Warn at the `onFail(skipTo(…))` call (labels passed, or its own pending label); no throw; the STOP unchanged; ST24; SM33 |
| 11 | minor | `loosenedSettle` divides by a possible 0; HA-216 covers drives only | **REAL.** `SettledUtil` allows `maxError = 0` (`settled_util.hpp` :50) and needs `maxErrorRate >= 0` (:51) — `NaN` fails it; `MotionConfig::validate` does not descend into the settle configs (`motion_config.hpp` :214-216, :246-247); 5° → 0.30 × 5/1.146 = 1.31 rad/s | FIXED: §1.1 formula `(e > 0 && t > e) ? r·t/e : r`; D2's cost; HA-216 widened to turns (and its floor run); ST23 `e = 0` cases; SM36 |
| 12 | minor | D5's 18-character word sits on an unmeasured width | **REAL.** `ui::kLargeCols = 18` "ESTIMATE, never measured on a V5 (A4: HA-136)" (`ui/touch_panel.hpp` :44-47; the register: derived between 12 read whole and 23 reported shown). **In part:** I did not take "FINISHED, 2 FAIL" (16; not plain English); I took "DONE, 2 FAILED" (14), which also fits the driver page's 15 unchanged (no widening, no name cut) | FIXED: D5 rewritten — (a) "DONE, n FAILED" / "DONE, 9+ FAILED", HA-136 named, what a narrower screen shows stated for (b); §1.1 (the words, the screen files; the driver line unchanged); ST17 (every word ≤ 15); §7 grep; SM21 redefined |
| 13 | minor | Arming vs `onFail`/`label`, a throwing leg, zero-size turns — unspecified | **REAL.** §1.2 / A2 / B5 were silent on each | FIXED: A2 (modifiers before the first arm, passed through); NEW B9 (a modifier or any `r.` call while armed = precondition; an arm or leg that throws clears every armed trigger and the pending modifiers; the destructor Warn); B5 covers zero-size turns; ST35; SM38 |
| 14 | minor | The coordination table misses shared files | **REAL.** Worktrees read: L6-A1 also edits `test/cookbook_examples_test.cpp`, `docs/cookbook/05`, guides 10 and 14; MECH-A6 edits changelog, roadmap, guides 10/14/18, the HA register; G3a Part 1 edits `test/CMakeLists.txt`; IMU1-B only `src/programs/bench_r3a.cpp`; VIS1-A1 (on main, uncommitted) edits guides 10/14 | FIXED: §6 rows amended and added (IMU1-B; VIS1-A1's state); the rebase rule restated, unchanged |
| 15 | minor | R1 builds before STEP1-B lands; pick R1's path | **REAL.** G3a's table: R1 build Nov 8–11, STEP1-B review from ~Nov 11, R1 review Nov 12–16 (`G3a` :1295-1298); this brief's STEP1-B merge ~Nov 14–15 | FIXED: §6 R1 row — R1 builds WITHOUT triggers; R1-T (emitters + S53 fixtures) after STEP1-B merges; row 20c corrected at the plan commit. Ordering only |

**Found on the re-check, not a checker finding:** `docs/guide/10-the-api.md` :338-340 lists `MotionOptions`' fields one by
one, so STEP1-A adds the two tolerances there (§3.1), and STEP1-B adds one sentence for the observer overloads (§3.2).

### The runs, with sizes (rough, re-derived)

| Run | Builder-days | Was | Gate before it |
|---|---|---|---|
| **STEP1-A** — the arrival tolerance, stop / carry on / skip, labels, the cut rule, the not-started answer, "DONE, n FAILED" | **~3–4** | ~3–3¾ | D1–D5 answered + its go |
| **STEP1-B** — partway triggers: the observer seam, the fire's own path, kinds and ranges, the narrowed settle fire, D8's checks | **~3–4** | ~2½–3¼ | STEP1-A's build ended; D6–D8 answered + its go |
| In all | **~6–8** | ~5½–7 | (REMAINING-WORK row 20c says ~5–6½ — corrected at the plan commit) |
| **R1-T** (G3a's, named here) — the panel's trigger emitters after STEP1-B | ~½–1 (rough; G3a sizes it) | — | STEP1-B merged |

### What stays unverified

- **On robot two, nothing here is measured:** the coast after a leg that ends at the loosened speed bound, drives AND
  turns (HA-216); a trigger's latency from the mark to the mechanism moving (HA-217); whether the estimate's "arrived"
  matches the tape (HA-218).
- **The screen widths** (`kLargeCols` 18, `kSmallCols` 50) are estimates (HA-136). The DONE words sit under both and
  inside the driver page's 15, so a narrower real screen cuts nothing of them down to 15 characters.
- **Designed, not probed:** the fire's own path (X3 probed the failure it avoids, not the fix), the kinds split, the
  narrowed settle bound, D8 (a)'s extraction, the guard-less Warn and the destructor Warn. Each has its test and
  mutation in §2; none has run.
- **Reasoned, not probed:** that a tighter heading tolerance's smaller speed bound could sit inside the IMU's noise
  (L2); that the extra 200-byte log line per run is harmless on the V5's task stacks (a value struct; not measured).
- **The line budgets** come from a script that applies the same `printf` formats, not from compiling the `snprintf`
  calls. ST17 is the compiled proof, and it runs only in the build.
- **The sizes** are rough: no step modifier or partway trigger exists in this repository to size against. R1-T is
  G3a's to size.
