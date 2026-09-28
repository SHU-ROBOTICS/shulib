# Changelog

> **What this is.** Every user-visible change to shulib, newest first, one section per API
> version — each entry says **what changed**, **whether it is breaking or additive**, and
> **what you must do about it**. The version policy itself lives in
> [`include/shulib/version.hpp`](https://github.com/SHU-ROBOTICS/shulib/blob/main/include/shulib/version.hpp):
> the major number moves only for a breaking change to a frozen surface, always with a
> migration note; the minor number moves for additive growth. The
> [Freeze Register](roadmap.md#freeze-register) says which surfaces are frozen at all.
>
> This file starts existing partway through the 2.1 line — the project ran to API 2.1 with
> the version history recorded only in `version.hpp`'s comments, which an outside team on
> 2.0 had no reason to read. Entries below the line marked *(reconstructed)* were written
> after the fact from those comments and the project records; everything above it was
> written when the change landed.

## API 2.3

### Changed — `moveTo` on a tank steers back onto its line (TANK1, 2026-09-27)

> Written when it landed (chunk TANK1, Part A). **Minor bump, 2.2 → 2.3** — the team lead's ruling
> (2026-09-27): one sentence the FROZEN `Chassis` header carries ([Freeze Register](roadmap.md#freeze-register)
> row F6, its strafe-fallback semantics) said *"On tank, laterally-offset targets honestly exit
> TimedOut"*, and after this change some of them arrive. That relaxes a stated LIMIT — the same
> direction as 2.2's relaxed `Route::valid()` rule — and no signature moved, so the bump is minor;
> but tank programs move differently, so it is listed as a behaviour change with a one-line way
> back. **Tank only: the X-drive and H-drive are unchanged, byte for byte** (every record, log
> line and motion checked with the change on and off) — with one exception by design: an H-drive
> configured with NO strafe authority (`strafeTractionDerate` 0, a dead strafe wheel) cannot move
> sideways either, reports authority 0, and is steered like a tank. **Host test plant only —
> nothing here has run on a robot.**

**BEHAVIOUR CHANGE (tank drives — a drivetrain whose `strafeAuthority()` is exactly 0):**

- **A tank `moveTo` leg that starts within 15° of its target heading now steers back onto its
  line while it drives.** The line runs through the target point along the target heading (for a
  recipe `driveTo`, the line from where the step started to the point). The heading the leg servoes
  to is bent toward the line — 0.1 rad per inch off it, never more than 15°, fading to nothing over
  the last 6 in; the inches counted are where the robot WILL be 0.06 s ahead at its heading and
  commanded speed, so the bend eases while the robot is already closing on the line instead of
  carrying it across — and the turn the robot makes that the heading loop did not ask for (a
  weaker side's pull) is measured and cancelled, never more than 1.5 rad/s. It never aims at the
  target POINT and never plans a turn: `face()` is still your turn. The three changes you can see:
  1. **Legs that already arrived take slightly different paths.** Any engaged tank leg with a
     cross-track error or a yaw disturbance moves differently from before, including legs that
     used to arrive: on the host test plant the library's acceptance run on an even drive ended at
     (0.24, 0.12, 180.00°) in 5.42 s before and ends at (0.23, 0.04, −179.97°) in 5.44 s now. Legs
     that used to time out on a weak side now finish: the 40 in leg with one side 3 %, 5 % or 9 %
     weak timed out 0.93 / 1.56 / 2.86 in off its line and now finishes within 0.02 in of it.
     **A leg that arrived before still arrives** — checked on the host test plant over 24 563 legs
     that start up to 14° off their target heading toward points up to 60 in away, with motor lags
     from none to about 0.18 s: none that settles with the change switched off times out with it
     on. (A review of this change caught a draft that failed that check — a leg starting 10–15°
     off its heading and turned toward its line crossed the line near its target and timed out
     half an inch beside it; steering on where the robot will be, above, removed every one.) The
     limit, stated: with about 0.24 s of motor lag, 24 of the 1 867 such legs that arrive with the
     change off time out with it on — at that lag the heading loop itself is at its limit, which
     R5's measured tuning is for.
  2. **An un-faced tank leg logs one Warn.** A tank `moveTo` leg that starts more than 15° off its
     target heading is not steered at all — it runs exactly as before — and logs, once, at its
     first live tick: `[WARN][MOT] tank moveTo not steered: it starts 26.6 deg off its target
     heading (steers within 15.0 deg) -- turn first: face()`.
  3. **(When the result line's cross-track column lands — TANK1 Part B, not yet built)**
     translating tank result lines will gain an ` xt` column. Nothing in this entry adds it.
- The tick log line of a steered tick carries the new flag ` TSTR` (after ` SFB`); ` SFB` lights
  on far fewer tank ticks than before, because the sideways error it reports is being steered out.
- `strafeTo`, `hold`, `turnTo`, `face`, `brake` and `drive()` are unchanged on a tank.

**ADDED:**

- `motion::TankSteerConfig` (`crossGain`, `maxSteer`, `fadeDistance`, `leadTime`,
  `yawObserverTime`, `yawCorrectionLimit`; every value PROVISIONAL — HA-179 in the
  [Hardware Assumptions Register](hardware-assumptions.md)) and `MotionConfig::tankSteer`,
  appended last (every designated initializer still compiles); `MotionConfig::validate()` refuses
  an invalid one.
- `motion/tank_steer.hpp` — the pure geometry: `TankLineError`, `tankLineError()`,
  `tankLeadCross()`, `tankSteerAngle()`, `tankLegEngages()`, `TankSteerStatus`,
  `kTankSteerNoiseFraction`.
- `MoveToPose::tankSteerStatus()` — whether a motion may steer, whether this run does, and the last
  commanding tick's cross-track error, the predicted one its bend steered on, the bend and the yaw
  correction.
- `diag::DebugRecord::tankSteerActive` (appended last) — on the SD-card blackbox as tick-flags bit
  2 (the file format version stays 1: every earlier writer wrote that bit as 0, so an older file
  reads it as false, which is true of it); on the terminal as ` TSTR`.
- `PoseMotionOptions::tankSteer` (Tier 3; false unless you build a plain `MoveToPose`).

**Migration:** nothing to change to keep compiling. To get the old tank back exactly, set
`maxSteer` to 0 — `ChassisConfig.motion.tankSteer.maxSteer = 0` (on a robot built from a
`RobotConfig`, its `tuning.motion.tankSteer.maxSteer`): a tank's `moveTo` then moves, records,
logs and ends exactly as it did at 2.2, bit for bit, with no Warn. (Part B's ` xt` column, when it
lands, will be the one exception: a diagnostic on the result line, kept whatever `maxSteer` is.)
The documented limits of the steering —
what it removes and what still times out, and what motor lag does to it — are in the guide's
[chapter 14](guide/14-what-it-cannot-do-yet.md#motion-quality-boundaries).

## API 2.2

### Changed since 2.2 (2026-09-26, later) — driver control releases a leftover claim; the boot check names declared mechanism motors

> Written when it landed (T3 Part 2 — robot two's claws wired into "shulib Teleop"). **No version
> bump:** `teleop::runDriverControl`, the mechanism bindings and `calibration::verifyAgainstRegistry`
> are not in the [Freeze Register](roadmap.md#freeze-register), and every signature that existed
> still compiles and means what it meant, except the one behaviour change below. **Host test plant
> only; nothing here has run on a robot.**

**BEHAVIOUR CHANGE (the team lead's ruling, 2026-09-26):**

- **`teleop::runDriverControl` RELEASES a claim still on a bound mechanism when it starts.** An
  operation autonomous was running when the field ended autonomous never releases its own claim —
  PROS deletes that task wherever it is — and a binding never commands a claimed mechanism, so the
  driver was locked out of that mechanism for the whole of driver control ("Added since 2.2 —
  mechanism buttons in driver control", below, told a program to release it itself). Now the loop
  does it: after its set check (a refused set still touches nothing) and before the entry safe
  state and each toggle's `enter()`, every hold
  binding's and toggle's mechanism that is still claimed is released — the claimant's `cancel()` is
  NOT called, because it may have lived on the deleted task's stack — and the loop logs one Warn
  line per release on the chassis's own sink, tag `TEL`: `driver control: RELEASED a claim left on
  mechanism '<name>' at entry (an operation that outlived its task) -- its buttons work from the
  first tick`. A claim taken DURING the loop is untouched: the binding still defers to it.
  **What it moves, for a toggle:** the release comes before the toggle's first `enter()`, so at the
  FIRST driver-control start the toggle's initial level now goes out even when an operation had
  the mechanism claimed — a claw an autonomous operation left claimed at the OTHER level moves to
  the initial level (a claw that starts closed closes) the moment driver control starts, a
  movement nobody pressed a button for. Before, a claimed claw stayed where the operation left it;
  an unclaimed one already moved (the toggle's rule since it was added).
  **Migration:** a program that released the claim itself before the loop may keep doing so (the
  loop then finds nothing and says nothing) or drop it. A program that starts an operation on a
  bound mechanism BEFORE calling the loop, meaning to tick it from the per-tick hook, loses that
  claim at the loop's start: start the operation from the hook instead. The dead operation object
  still believes it holds the claim, so do not `cancel()` it later — that would safe the mechanism
  under whoever holds it then. If your autonomous leaves a toggle's mechanism where driver control
  must find it, state that level as the toggle's initial level — the toggle has no "keep what is
  there" start.

**ADDITIVE:**

- **`MechanismBinding::releaseLeftoverClaim()` / `releasedLeftoverClaim()`**, and the same pair on
  **`ToggleBinding`** — what the loop calls at entry, and what a panel reads afterwards ("a claim
  was released when driver control started").
- **`calibration::DeclaredMotorPorts`** and a three-argument **`verifyAgainstRegistry(desc,
  registry, declared)`**: motor ports the PROGRAM declares as mechanism motors, and the words for
  them. A registry motor on one of them that the description does not list is expected — counted in
  `BootVerdict::declaredPorts` / `declaredCount` and named in its own clause after the NEW motors
  (`NEW motors not in the calibration: 14; claw pivots (declared, not drive motors): 6 7`), never
  as a NEW motor. The two-argument form is unchanged, word for word.

### Added since 2.2 (2026-09-26) — the competition library: an auton registry, a selector, a runner, abandon, and the start-heading seed

> Written when it was built (chunk COMP-a, the LIBRARY half of the competition program; the
> program itself — `src/`, the seventh build — is COMP-b). **No version bump — every change is
> ADDITIVE:** nothing frozen changed shape. `RunGuard` (Freeze Register row F12) is not frozen;
> `Localizer` and `robot::BuildOptions` are outside every freeze; `Routine::startAt` (F10) and
> `Chassis::setPose` (F6) are untouched and still seed position only. **Nothing below has run on a
> robot** — it is proven on the host test plant only.

**ADDED:**

- **`robot::BuildOptions::pacer`** (default `nullptr` = the provider's pacer, so every existing
  graph is byte-for-byte unchanged). A competition program passes its `sequence::RunGuard` here —
  the guard reaches a running motion only through the pacer, and a Chassis takes its pacer once.
  **Unlike `absentDrivePorts`, the pacer is BORROWED FOR THE ROBOT'S WHOLE LIFE** (the Chassis
  keeps it): spell it `robot::BuildOptions{.absentDrivePorts = ..., .pacer = &guard}` with the
  guard constructed first and living as long as the Robot.
- **`sequence::RunGuard::requestAbandon()`** and `abandonRequested()`, plus
  `RunGuardReport::abandoned` — the third way a run ends (the first two are the deadline and the
  hard stop). One atomic store, safe from any task and from inside a pacer; from the next check on
  every motion is cut before the world advances and every guard wait returns `RunExpired`; when
  scoring returns, `run()` does the full cancel-all, SKIPS the end action, and disarms. It is a
  latch, never a disarm: the routine unwinds normally, on its own stack. A request made while no
  run is live is dropped when the next run arms. **What you must do:** nothing — no existing call
  changes; `RunGuardReport` gained a field.
- **`localization::Localizer::reseatHeading(math::Angle)`** — the heading's twin of `setPose`: the
  published heading becomes the given one on this tick (through the same bias the tag corrector
  moves; the IMU stays the only source of heading CHANGE), and the next tick re-baselines as after
  `setPose`. Preconditions: the estimate is live, and no motion is active. It is a CALLER seed —
  no sensor and no corrector can reach it. **What you must do:** nothing; `setPose` still ignores
  heading.
- **`include/shulib/competition/`** (new directory):
  `auton_registry.hpp` — `AutonRegistry`, `AutonEntry` (a name, MATCH or SKILLS, a routine OR a
  route, a FIELD or RELATIVE `StartPose`, `endActionAt`/`hardStopAt`, an optional end action),
  `AutonPeriods` (the program's two periods — required, never defaulted), `checkAutonEntry`;
  every unset or impossible field is refused at REGISTRATION, by name. A ROUTE entry's start is
  its route's `Start` waypoint — and that waypoint's heading is read only when it is `Fixed`: a
  `Free` Start states no heading, so the entry must declare a FIELD start at the same (x, y) or
  it is refused (a default 0 is never seated by accident).
  `match_runner.hpp` — `runAuton(...)`: refuses before arming, arms the guard, waits (with the
  guard's own wait) for a live estimate, re-seats position AND heading on one tick, runs the
  routine or the route, and returns a TAGGED `AutonRunResult`. A run whose estimate never went
  live (`NotStarted`) brakes at its end — the entry's own end action is never run in a frame
  that was never seated.
  `auton_menu.hpp` — `AutonMenu`, the brain-screen selector as a pure, host-tested model: NONE
  always present and the default, paged, one atomic choice that `lockChoice()` freezes.
  `acceptance_run.hpp` — `acceptanceRun`, the one routine the library ships: drive 24 in, turn
  90 degrees, face the start and drive back; not season content.
- **`include/shulib/ui/touch_panel.hpp`** (new directory) — the generic pieces of a screen model:
  the text budgets (held equal to the calibrate panel's at compile time), rectangles and padded hit
  boxes, `screenText`/`copyScreenText` (printable ASCII, no `%`), the release edge, the arming
  clock. `calibration/calibrate_panel.hpp` is unchanged.

**RE-WORDED (a guarantee stated more precisely, not weakened):** `localizer.hpp` said "there is no
code path … that ASSIGNS a heading"; it now says no CORRECTOR and no fusion policy can, and names
the one caller seed. Never-snap — what a sensor may do to a live estimate — is unchanged. The
claims that followed from the old sentence now name the seed too: the heading bias is exactly
zero, and the published heading is the raw IMU bit for bit, only on a tree with no
heading-providing corrector AND no `reseatHeading()` seed — so **`Localizer::headingBias()` is no
longer a pure drift measure once a competition auton has started** (it carries the start-heading
seed). `Localizer::Quality::Uninitialized`'s doc now says what the code has always done: the
estimate is never live on the FIRST `update()`, only from the second (behaviour unchanged).
`config::Route`'s docs (F8; wording only) and the VexBuilder verb table say that a `Start`'s
heading is the start heading a competition auton is seated to when the Start is `Fixed`.

**Hardware assumptions:** HA-05 is AMENDED (the start heading's one owner is the chosen auton's
start pose, applied at autonomous start through the localizer's bias — never the IMU's boot
heading); HA-161 is NEW (the belief that PROS kills its competition tasks on a mode change — the
reason the competition program will run its auton on a task of its own). See the
[hardware assumptions register](hardware-assumptions.md).

### Changed since 2.2 (2026-09-26) — an `ODO_STUCK` line names the check that fired and its values

> Written when it landed (chunk DIAG1). **No version bump — the decision, under
> [`version.hpp`](https://github.com/SHU-ROBOTICS/shulib/blob/main/include/shulib/version.hpp)'s
> policy:** every surface this touches — `localization::IOdometry`,
> `diag::HealthMonitor::Observations`, the fault line's text — is outside the
> [Freeze Register](roadmap.md#freeze-register), and the one frozen promise nearby (F6's fault
> policy: `ODO_STUCK` aborts the active motion by default) is unchanged. The odometry's
> VERDICT — which ticks are implausible — is bit-identical; only the words on the line grew.
> **Nothing below has run on a robot.** It exists because robot two raised
> `implausible odometry tick` twice on 2026-09-25 and the line could not say why — and the next
> one could end an autonomous.

**CHANGED — what you will see:**

- **The `implausible odometry tick` fault line says which check fired, with the numbers.**
  `fault=ODO_STUCK n=1 FIRST implausible odometry tick why=ROT dth=1.745rad h0=0.012 h1=1.757
  L=0.000in R=0.000in` — `why=` is one or more of `NONFINITE`, `ROT` (the heading moved more than
  π/2 in one tick), `TRAVEL` (a side or pod moved more than 36 in in one tick); `dth`, `h0`, `h1`
  are the tick's heading change and its start/end IMU headings in radians; `L=`/`R=` are the two
  drive sides' travel in inches (`fwd=`/`lat=` on a tracking-wheel robot). One line per episode,
  with the FIRST tick's values. `why=UNREPORTED` means the odometry gave no account; `+STALL`
  means the stall cross-check rose on the same tick (the stall-only line is unchanged). Read
  field by field in the guide's
  [Chapter 11](guide/11-reading-the-diagnostics.md#reading-an-odo_stuck-line).
- **What you must do:** nothing, unless you parse fault lines — the text after
  `implausible odometry tick` is new.

**ADDED:**

- **`diag/odom_tick_audit.hpp`** — `diag::OdomTickAudit` (the trust gate's account of one tick:
  three flags, five values, `tripped()`), `diag::OdomTravelLabels`, `diag::appendOdomTickAudit()`
  and `diag::kOdomAuditDetailMaxBytes` (140 — what the fault latch's line can carry whole).
- **`localization::IOdometry::lastTickAudit()`** — a new **pure virtual**. Both of the library's
  odometries implement it, and their verdict is now read FROM the audit, so the two cannot
  disagree. **What you must do:** if you wrote your own `IOdometry`, add it — returning
  `diag::OdomTickAudit{}` compiles and makes your lines read `why=UNREPORTED`; filling it from your
  own gate makes them say why. This is source-breaking for such a class; it is not a frozen
  surface, which is why it carries no version bump.
- **`localization::Localizer::lastOdomTickAudit()`** (forwards it) and
  **`diag::HealthMonitor::Observations::odomAudit`** (appended LAST, so every designated
  initializer still compiles; left default, the line reads `why=UNREPORTED`).
  `motion::tickHealthObservables` fills it, so `drive()`, every motion and the idle tick carry it
  with no change to your program.

### Added since 2.2 (2026-09-26) — mechanism buttons in driver control, and mechanisms built from the config

> Written when it landed. **No version bump:** nothing here grows a FROZEN surface — the mechanism
> seam (F11), the controller seam (F13) and the robot profile (F7) are all still unfrozen, `Chassis`
> (F6) and `Routine` (F10) are untouched, and `robot::Robot`, `config::RobotConfig` and
> `teleop::runDriverControl` are not in the [Freeze Register](roadmap.md#freeze-register), and
> neither is `hal::IDeviceProvider`. Some config rules are new and REFUSE configs they used to
> accept, and the device provider gained a member every implementer must answer — see below.
> **Everything here is proven on the host test plant only, and nothing has moved a mechanism on a
> robot.** (When this was written no program used it; since T3 Part 2, later the same day, "shulib
> Teleop" drives robot two's claws with it — still not run on a robot.) The pneumatic half (the toggle, the provider's 3-wire lines,
> pneumatic mechanisms from the config) landed the same day, from T3's addendum.

**ADDITIVE:**

- **`teleop::MechanismBinding`** (`teleop/mechanism_binding.hpp`) — two controller buttons driving
  one `hal::MotorMechanism`, hold-to-run: the forward button alone drives it at a STATED voltage
  (no default; in (0, 12] V) every tick, the reverse button alone at minus that, and neither, both,
  or a disconnected controller puts it in its declared safe state ONCE, on the transition — then it
  commands nothing until a button drives again. Releasing the button is the stop. An outside stop
  through the binding (`makeSafe()`) while a button is held leaves it inert until the driver lets
  go; a mechanism that something else has claimed is never commanded — including one an operation
  left claimed when the end of autonomous deleted its task, so a program whose autonomous commands
  a bound mechanism releases that claim before driver control. *(Superseded later the same day:
  `runDriverControl` now releases such a claim itself when it starts — "Changed since 2.2 (2026-09-26,
  later) — driver control releases a leftover claim", above.)*
  `teleop::validBindings(bindings, rezeroButton, why, cap)` checks a whole set (a button used
  twice, a button that is also the re-zero button, a mechanism bound twice) and names the problem;
  `teleop::controllerButtonWord()` prints a button.
- **`teleop::ToggleBinding`** (same header) — ONE button flipping one `hal::PneumaticMechanism`:
  each press flips the level the line was last commanded, exactly once (holding does nothing more);
  a press must be SEEN — a button already down when driver control starts, or when a dropped
  controller comes back, is not a press, so a press made while the controller was gone is not
  replayed (provided the controller's connected flag and its buttons come back together — not yet
  measured); the initial level is STATED (no default) and sent once, at the binding's first loop
  entry; and a disconnect, a reconnect or a field disable commands NOTHING — the claw keeps what it
  holds. It takes no claim and never commands a claimed mechanism.
  `teleop::validBindings(bindings, toggles, rezeroButton, why, cap)` checks hold bindings and
  toggles as ONE set (the hold-only form is the same check with no toggles).
- **`teleop::runDriverControl` takes the bindings** — a new overload with a
  `std::span<teleop::MechanismBinding* const>` after the loop config. Each binding is reset and made
  safe before the first tick, ticked after `drive()` with its buttons read only while the controller
  is connected, and made safe again on a field disable (after the drive's own stop). It refuses,
  before the first tick, a set `validBindings()` refuses. The old signature is the same loop with
  no bindings. **Watch the default:** `DriverLoopConfig::rezeroButton` defaults to L1, so a program
  that binds L1 must name another re-zero button. A second overload also takes a
  `std::span<teleop::ToggleBinding* const>` after the hold bindings: each toggle is entered before
  the first tick, ticked with its button read only while connected, and **left untouched on a
  field disable** (the hold bindings are made safe; the claw is not).
- **`config::MechanismConfig` gains `cartridge` and `safeMode`** (`config::SafeBrakeMode` —
  `Coast`, `Brake`, `Hold`, with the byte 0 meaning UNSET and no default), plus
  `config::safeBrakeModeWord()`, `config::isSafeBrakeMode()` and `config::toBrakeMode()` — and
  **`pneumaticSafe` and `pneumaticStart`** (`config::PneumaticLevel` — `Low`, `High`, the byte 0
  UNSET, no default), plus `config::pneumaticLevelWord()`, `config::isPneumaticLevel()` and
  `config::toLineValue()`.
- **`hal::IDeviceProvider::digitalOut(line, initialState)`** — the brain's own 3-wire ports, 1..8
  for A..H (`hal::kBrainAdiLines`). Making a line DRIVES it to the stated level (PROS's own
  constructor does), so the level is an argument; the same line again is the same object, never
  driven again; the same line at another level is a precondition. `hal::pros::ProsDeviceProvider`
  makes a `ProsDigitalOut` and can refuse only a line outside 1..8 — the brain's registry cannot
  see a solenoid, so nothing can tell it one is missing. `hal::fake::FakeDeviceProvider` drives its
  lines the same way and adds `bindDigitalOut()`, `fakeDigitalOut()`, `refuseLine()`/`allowLine()`,
  `digitalOutRequestCount()`, `digitalOutMade()` and `digitalOutInitialState()`.
- **`robot::Robot` builds the mechanisms** — one `hal::MotorMechanism` per mechanism with motors,
  its motors asked for with the MECHANISM's cartridge and signed ports, its declared safe mode, its
  config name. `Robot::mechanism(name)`, `mechanismAt(i)`, `mechanismCount()`,
  `mechanismAbsence(i)`, and `mechanisms()` — the `hal::IMechanism*` span an end-of-run guard
  walks. **A mechanism motor the device provider will not make leaves that mechanism ABSENT**, with
  the provider's sentence, and the robot still builds and drives; `validate()` does not ask for
  mechanism motors. **Since the addendum it also builds one `hal::PneumaticMechanism` per
  mechanism with 3-wire lines**: each line made at the config's start level (on a brain, the moment
  the Robot is built is the moment the claw takes it), the declared safe level, the config name —
  `Robot::pneumaticMechanism(name)` and `pneumaticMechanismAt(i)` find it (`mechanism(name)` stays
  the MOTOR lookup), and `mechanisms()` carries both kinds. A line the provider will not make leaves
  that mechanism ABSENT; `validate()` never asks for a line (asking would drive it). A robot with no
  mechanisms builds exactly the graph it built before.
- **`Robot::builtMechanismAt(i)`** — the `i`-th declared mechanism if it was built, of EITHER kind
  (a `hal::IMechanism*`). Because the other lookups are typed, `mechanismAt(i)` is null for a built
  pneumatic claw; a boot line or panel row that reports each declared mechanism asks
  `builtMechanismAt(i)`, and `mechanismAbsence(i)` is empty exactly when it is non-null — for an
  index past the declared mechanisms it now says "no mechanism is declared at this index" rather
  than the empty string that means "built".

**Behaviour you should know changed, without a signature moving:**

- **`RobotConfig::valid()` (and so `robot::validate()` and `Robot`'s constructor) now refuses a
  mechanism WITH motors whose `cartridge` or `safeMode` is unset**, a mechanism name with no NUL
  inside its 24 bytes, and two mechanisms with the same name — each by name. Port collisions are
  checked first and keep their messages. **Migration:** give every mechanism that has motors a
  `.cartridge` and a `.safeMode`; a purely pneumatic mechanism needs neither.
- **…and, since the addendum, a mechanism with 3-wire lines whose `pneumaticSafe` or
  `pneumaticStart` is unset, a mechanism with BOTH motors and lines, and a 3-wire line listed twice**
  (by two mechanisms, or twice by one) — each by name. **Migration:** give every pneumatic
  mechanism its two levels; split a mixed mechanism into a motor one and a pneumatic one.
- **`hal::IDeviceProvider` has a new pure virtual, `digitalOut()`.** Code that only CALLS a
  provider is unaffected; a class that IMPLEMENTS one must add it (the two in the library have).
  **Migration:** answer it — make the line at the stated level, or return nullptr and say why in
  `lastRefusal()`.

### Fixed since 2.2 (2026-09-26) — Brake and Hold now reach a V5 motor; the driver loop's release mode

> Written when it landed (chunk DEFECTS2). **No version bump — the decision, under
> [`version.hpp`](https://github.com/SHU-ROBOTICS/shulib/blob/main/include/shulib/version.hpp)'s
> policy:** no signature changed anywhere; `hal::IMotor` (Freeze Register row F4) already PROMISED
> what this fix delivers ("Hold is the active position hold the guaranteed park relies on"), and
> `Chassis::cancel()`'s documented safe state (F6: "0 V + Brake") is unchanged in the contract — it
> is the V5 adapter, an implementation outside every freeze, that now keeps them. The one new name,
> `teleop::DriverLoopConfig::releaseBrakeMode`, is on an unfrozen struct and its default keeps the
> stick-release behaviour a robot showed before. A behaviour change that makes an implementation
> match its frozen contract is neither the policy's BREAKING (the documented semantics did not
> move) nor its ADDITIVE (nothing frozen grew). **Nothing below has run on a robot**; the beliefs
> it rests on are A4 register HA-142–HA-146 in the
> [hardware assumptions register](hardware-assumptions.md).

**FIXED — what changes on a robot:**

- **Every Brake and Hold stop now applies its mode at the motor.** PROS applies a brake mode only
  through `pros::Motor::brake()`; `move_voltage()` "will not respect brake modes". Until now
  `hal::pros::ProsMotor` sent every zero as `move_voltage(0)`, so — by PROS's own documentation;
  nobody has watched it happen — on a V5 a mechanism declared `BrakeMode::Hold` went LIMP when its
  operation ended, and the drive's cancel safe state ("0 V + Brake") COASTED, through every path
  that uses it: `Chassis::cancel()` and its panic
  stop, a pre-emption, a fault abort, the `brake()` verb (`DriveBrake`), `RunGuard`'s forced-safe
  paths, and the driver loop's disable. Every host test passed throughout, because the fakes echo
  the mode and the host test plant pins a wheel at 0 V. Now a zero command (the millivolts SENT
  round to 0) under a Brake or Hold that the program set with `setBrakeMode()` goes out as
  `brake()` — once, on the way into the stop; the mode and the zero may come in either order.
  **Coast is unchanged**, byte for byte on the wire: a zero under Coast is still `move_voltage(0)`,
  so "shulib Calibrate", "shulib Drive" and the tester's DRIVE station behave exactly as before.
- **A motor your program never gave a mode is unchanged too.** The adapter keys the stop on the
  mode THIS program commanded, never on the mode the device remembers from an earlier program
  (which it still reads for `brakeMode()`'s fallback): a drive motor left in Hold by another
  program does not start holding at centred sticks.
- **A stop the port refused is retried, never believed.** `brake()` stops under whatever mode the
  motor's register holds, so if a port drops out as a mode or a stop is sent (PROS answers with an
  error), the adapter records no stop, re-sends the mode before its next `brake()`, and tries again
  at the next zero command. A caller that sends its zero only once — a mechanism operation's exit —
  gets no retry if the port is down at that instant, as with any refused command.
- **A mode, once set, governs every later zero on that motor — so it now matters on a robot.**
  The library's Brake stops (a cancel, the `brake()` verb, a pre-emption, a fault abort) leave the
  drive motors in Brake, and a later zero command that sets no mode of its own — a motion's settle
  stop, the zero volts a motion commands while it waits for the estimate — now BRAKES too, where
  before DEFECTS2 every such zero was a plain 0 V. That is `hal::IMotor`'s contract working as
  written, and it is why the driver loop now re-asserts its own release mode every tick (below).
- **What you must do:** nothing, unless your program relied on a Brake or Hold stop coasting —
  then set `BrakeMode::Coast` where you meant it. Expect a mechanism declared Hold to draw holding
  current at rest now (watch its temperature — HA-92), and expect the drive to brake on a cancel.

**ADDED:**

- **`teleop::DriverLoopConfig::releaseBrakeMode`** (default `BrakeMode::Coast`, the team lead's
  ruling of 2026-09-26). `runDriverControl` puts it on every drive motor at entry and again after
  every `drive()` — because a `drive()` that pre-empts a motion cancels it into Brake, and with the
  fix above that Brake would now make the next released stick brake the drive. The disable path is
  not governed by it: a disable still cancels into Brake. **What you must do:** nothing to keep
  coasting on release; set `.releaseBrakeMode = hal::BrakeMode::Brake` (or `Hold`) to make a
  release stop the drive.

**CORRECTED DOCUMENTATION:**

- `ProsMotor::faultedReads()` counts a screened `brakeMode()` read — it has since DEFECTS1 — but
  its comment and the class comment still said it did not. They now say five screened reads are
  counted.
- The `hal::BrakeMode` comments (`hal/motor.hpp`) now quote what PROS actually says each mode does
  and mark the rest as belief: which position Hold holds, and that Brake does not return to where
  it stopped, are not in PROS's text.

**For contributors:** the host PROS shim (`test/pros_shim/pros/motors.hpp`) gained `Motor::brake()`
and a per-port CONTROL STATE (`shim::MotorControl::Voltage` / `Stopped`, and the mode a stop was
issued under), so a test can tell "0 V, mode ignored" from "stopped, mode applied". Assert a stop
through that state, never through an adapter's `brakeMode()`, which read Hold all through this
defect. `MotorPortState::refuseSetBrakeMode` refuses the next N mode writes alone, for a dropout
that ends between two calls.

### Removed since 2.2 (2026-09-24, later the same day) — the calibrator's three retired names

> Written when it landed. **No version bump:** `calibration::DrivetrainCalibrator` is not in the
> [Freeze Register](roadmap.md#freeze-register); the calibration record's format and the boot check
> are unchanged. The section below retired these names and said they would be removed once the
> "shulib Calibrate" program no longer used them; that program now paints the screen model
> (`calibrate_panel.hpp`'s `viewFor()`) instead, so they are gone. **The rewritten program has not
> run on a robot.**

**REMOVED — what a caller must change:**

- **`Phase::ScalePush`** — the calibrator never entered it since the change below. **Migration:**
  delete the `case`; a `switch` over `Phase` now has seven values.
- **`Answer::Distance24`, `Answer::Distance48`, `Answer::Distance72`** — never accepted since the
  change below (there is one push, of `CalibratorConfig::pushDistance`). **Migration:** delete the
  distance buttons; the answers are `Done`, `FrontFirst`, `BackFirst` and `Abort`.
- **`PortObservation::tapeDeltaRad`** — always 0 since the change below. **Migration:** read
  `pushTravelRad` (a drive port's travel over the one push, sign-corrected) or `pushDeltaRad` (as
  read).

### Changed since 2.2 (2026-09-24) — the drivetrain calibrator rebuilt after its first night on a robot; a pure model of its screen

> Written when it landed. **No version bump:** `calibration::DrivetrainCalibrator` is not in the
> [Freeze Register](roadmap.md#freeze-register), and nothing frozen moved — the calibration record's
> format is unchanged (`kRecordVersion` is still 1) and so is the boot check. **The calibrator's
> public surface DID change**, so this section says what a caller must do. Why it changed: on
> 2026-09-24 the calibrator ran five times on a real robot and never finished — its one 3 V
> single-motor pulse was too weak for a five-motor gear train, its screen offered DONE before
> anything was pushed, and its host tests had passed on a test plant that could not fail the way the
> robot did. **Everything below is proven on the host test plant only; none of it has run on a
> robot.**

**CHANGED — what a caller must change:**

- **One push instead of two.** The operator pushes the robot forward `CalibratorConfig::pushDistance`
  (48 in) along a tape ONCE, then answers FRONT or BACK; that push gives the drive ports, their signs
  and each side's scale. There is no distance question: `Answer::Distance24/48/72` are **retired** —
  `accepts()` never accepts them and `answer()` refuses one as out of order — and `Phase::ScalePush`
  is **retired** (never entered). `PortObservation::tapeDeltaRad` is **retired** (always 0); read
  `pushTravelRad`. The three retired names still compile so the current "shulib Calibrate" program
  builds; **they will be removed** when that program is rewritten.
  **Migration:** drop the distance buttons and the `ScalePush` case; offer DONE only when the new
  `pushReady()` is true (see below).
- **`CalibratorConfig::pulseVolts` and `spinVolts` are gone**, replaced by ramps:
  `pulseLevels` / `pulseLevelCount` (3, 6, 9 V) and `spinLevels` / `spinLevelCount` (3, 6 V). New
  fields: `restYawRateFloor`, `decoupledOwnMin`, `pushDistance`, `spinStartTime`, `spinStartYaw`,
  `spinWrongWay`. A config whose ramp is not strictly increasing inside (0, 12] V, or whose
  `restYawRateFloor × pulseTime` is not under `pulseYawFloor`, is refused at construction.
  **Migration:** a config that set `pulseVolts = X` sets `pulseLevels[0] = X` (and the count), or
  keeps the defaults.
- **A drive port whose motor turns but moves nothing (a slipping gear) is LEFT OUT** of the
  description with a warning (`warning()`, `report().excludedPorts`), and the run can still finish:
  a description may now name fewer ports than the push moved. The boot check already treats such a
  present-but-unrecorded motor as a warning.
- **`status().prompt`** may now point into a buffer the calibrator owns (the push instruction names
  the configured distance); it is valid until the calibrator's next `tick()`, `answer()` or
  `abort()` — copy it if you keep it longer. Every prompt is at most 44 characters.
- **Behaviour you should know changed, without a signature moving:** rest between pulses now also
  requires the IMU's yaw rate under a floor; the side pulse ramps and stops at the first level that
  turns the robot; the spin refuses at once if it turns the WRONG way (15°), and steps up once and
  then refuses if it cannot start turning (instead of stalling 20 s); the spin-disagreement refusal
  says "pct" instead of the percent sign (PROS re-reads screen text as a format); the copy
  constructor and copy assignment are now explicitly deleted (they were already ill-formed).

**ADDITIVE:**

- **`calibration::Refusal`**, `refusal()` and `refusalPort()` — a code (and the port, where there is
  one) for every way a run can stop, so a screen can explain a stop without parsing prose;
  `refusalName()` for logs.
- **`pushReady()`** — the DONE gate, built from the same predicates DONE is judged on.
- **Numbers in `CalibratorStatus`** (moved count, push travel and far-enough, at-rest, drift, pulse
  ordinal/port/level, live yaw, spin direction/turns/level, the side counts, left-out count, seconds
  in the phase) and **a serial record** in the report — one `PulseRecord` per pulse at every level,
  one `SpinRecord` per spin direction — with `formatPushRecord()`, `formatPulseRecord()` and
  `formatSpinRecord()` printing each as one ASCII line.
- **`calibration/calibrate_panel.hpp`** — the calibration screen as a pure, host-tested model:
  `viewFor()`, `hitTest()`, `TouchGate`, `wrapWords()`, `stoppedWords()`, with compile-time checks
  that every piece of text fits its (estimated) line budget, is ASCII and carries no percent sign.
  No program uses it yet.

### Added since 2.2 (2026-09-24) — the on-robot device provider, dead ports in the builder, the port map, the driver-control loop

> Written when it landed. **No version bump, and none owed:** everything here is additive, and none
> of it grows a FROZEN surface — `IDeviceProvider` and `robot::Robot` are not in the Freeze
> Register, `Chassis` (F6), the route and runner (F8), `Routine` (F10), the HAL interfaces (F4) and
> the kinematics (F5) are untouched, and `motion::ITickPacer` gained nothing. **Nothing on a brain
> uses any of it yet** — no program in `src/` changed; "shulib Teleop" moves onto it in a later
> step — and everything below is proven on the host only: the provider through the host's
> stand-in for the PROS headers, the builder and the loop on the host test plant.

**ADDITIVE:**

- **`hal::pros::ProsDeviceProvider`** (`hal/pros/device_provider.hpp`) — the on-robot
  `IDeviceProvider`: real adapters made on first request, one per port, owned for the life of the
  program, with no heap. **Before it constructs anything it asks the brain's port registry what is
  plugged in**, and refuses an empty or wrong-kind port with a sentence naming the port, what it
  found and what it expected (`port 2: a rotation sensor (expected an IMU)`); a motor that is
  present but rejects its configuration is refused with the adapter's own reason. Asking again for
  a port returns the same object; asking with a different sign, cartridge or GPS lever arm is a
  precondition violation. `imu()` starts the IMU's calibration once, itself. (New hardware belief:
  HA-141, [register](hardware-assumptions.md).)
- **Dead drive ports in the builder:** `robot::BuildOptions{ .absentDrivePorts = … }` — port
  numbers the boot check found dead — accepted by `robot::validate(cfg, devices, options)` and
  `robot::Robot{cfg, devices, options}`. Each becomes an ABSENT member of its side's
  `hal::MotorGroup` and is never asked for; the list is refused, naming the port, if a port in it
  is not a drive port, if the drivetrain is not a tank, or if it would empty a side. The builder
  applies no dead-port policy of its own. **Default options build exactly what 2.2 built.**
  `Robot::describe()` adds `DEGRADED: absent …` when a port is absent (only then).
- **The session header's port map, generated:** `Robot::portMap(out, cap)` and the free function
  `robot::writePortMap(config, out, cap)` — the text `diag::SessionInfo::portMap` carries, in the
  same format the programs in `src/` print by hand today, bounded and always terminated.
- **`Robot::driveOdometry()`** — the drive-encoder odometry, for a panel's cross-check row
  (`nullptr` on a tracking-wheel robot).
- **`teleop::runDriverControl`** (`teleop/teleop_loop.hpp`) — the driver-control loop, PROS-free:
  a disable check that stops the drive into its safe state and returns, all four stick axes read
  only while the controller is connected, `DriveShaper`, the chassis's speed budgets,
  `drive(…, Frame::Body)`, a re-zero button with its own edge, a per-tick hook for a panel. The
  tick period, the disabled check and the heading source are arguments
  (`teleop::DriverLoopConfig`, `teleop::HeadingSample`).
- **`hal::fake::FakeDeviceProvider::motorRequestCount(port)`** — counts every request, refused ones
  included (test infrastructure).

**Behaviour you should know changed, without a signature moving:**

- **`teleop::FieldCentric::rezero()` (and so `DriveShaper::rezero()`) no longer accepts a heading
  that is not a finite number.** It used to store it as the zero and clear the fallback latch, after
  which the next trusted tick produced a NaN command that `Chassis::drive` refuses by precondition.
  Now the previous zero is kept and the fallback latches, so the controls stay robot-centric until
  the driver re-zeroes on a real heading. Every finite heading behaves exactly as before.
  **Migration:** none.

### 2026-09-18 — routines as data, a robot built from data, driver feel, and self-calibration; F8 frozen

> *Written 2026-09-24, late.* The bump to 2.2 landed on 2026-09-18 without this section, against
> this file's own promise. It covers everything user-visible between 2.1 (2026-08-15) and 2.2,
> reconstructed from the commits and the headers' own notes. **None of it has run under the
> library's motion stack on a robot** — every closed-loop claim below is proven on the host test
> plant.

**Nothing frozen changed shape.** Every change is additive except one tightened precondition,
listed first because it is the one a compiler will not show you.

**Behaviour you should know changed, without a signature moving:**

- **`motion::MotionDeps` now requires the drive-motor count to EQUAL the kinematics' wheel count**
  (it accepted *more* motors than wheels before). The old rule let eight raw motors sit on a
  two-wheel tank kinematics with six of them never commanded — no voltage, no brake mode, no
  fault. **Migration:** put the N motors on one side behind one `hal::MotorGroup` and hand the
  group to the context as that wheel's motor. A program that passed extra motors now refuses at
  construction with a sentence saying exactly this. **Under the version policy this is a breaking
  change** — the old documentation made "at least as many motors as wheels" valid, a program
  relying on it no longer runs, and `MotionDeps` is what the frozen `Chassis` constructor takes —
  and it shipped (2026-09-14) without a major bump or a migration note. Whether to bump, or to
  rule it a defect fix as the 2.1 entry did for unfrozen surfaces, is an open ruling for the
  project lead; this entry will say which when it is made.

**ADDITIVE:**

- **Routines as data (the change that bumped the minor and LOCKED register row F8):**
  `runner::PathRunner` runs a `config::Route` through the facade's verbs, `runner::CommandRegistry`
  binds a marker's command id to your code (`registry.on("intake_in", fn)`), and
  `runner::writeManifest` exports the id vocabulary for a path editor. `config/route.hpp` gains
  the id rules and the four library-owned built-ins (`wait_ms`, `brake`, `hold_ms`, `log`);
  `Route::valid()` gains rules and **relaxes one** — a `MoveToPose` may now use
  `HeadingMode::Free` ("keep the heading you have"). Relaxing is the additive direction: every
  route valid under 2.1 is valid under 2.2. On a tank, a route must carry its own turns — the
  facade never plans a turn you did not ask for.
- **A robot described as data and built from it:** `config/robot_config.hpp` (`RobotConfig`),
  `hal/device_provider.hpp` (`IDeviceProvider`, devices by port) with a host fake, and
  `robot/robot.hpp` (`robot::validate()` and `robot::Robot`, which owns the whole graph). The PROS
  device provider was not built at 2.2, so no program on a brain used this path. *(It has existed
  since 2026-09-24 — the entry above — and still no program on a brain uses the path.)*
- **Coupled drive motors:** `hal::MotorGroup` (N motors as one, median reads, a member may be
  ABSENT), `hal::DriveGeometry`, and `diag::FaultCode::MotorGroupDisagree = 12` — appended, per the
  append-only rule — raised when one member of a group persistently disagrees with its mates.
  `MotionDeps` gains `motorGroups` so the fault fires in `drive()` and in every motion.
- **Odometry from the drive motors:** `localization::IOdometry` (the seam) and
  `DriveEncoderOdometry`. `Localizer`'s constructor now takes an `IOdometry&` rather than a
  `PilonsOdometry&` — every existing call still compiles, because `PilonsOdometry` implements it.
  `MotionConfig`'s stall check gains per-wheel geometry and an `independentMotionSource` flag
  (off when the odometry IS the drive encoders, since a check against itself proves nothing).
- **Drivetrain self-calibration:** `hal::DrivetrainDescription`,
  `calibration::DrivetrainCalibrator` (the robot measures its own sides, signs, scales and track
  width), `calibration/calibration_record.hpp` (a 180-byte, CRC-checked record for the SD card)
  and `calibration/boot_check.hpp` (a record verified against the ports before any adapter
  exists); `hal::IBlockSource` and `hal::IDeviceRegistry` with their PROS adapters and fakes.
- **Driver feel:** `teleop/` — `input_shaping.hpp` (continuous deadband, expo), `slew_limiter.hpp`,
  `drive_modes.hpp` (arcade body/field, tank sticks), `field_centric.hpp` (with a latched fallback
  on an untrusted heading), `driver_control.hpp` (`DriveShaper`), plus `stick_mapping.hpp`,
  `coupled_side_monitor.hpp` and `drivetrain_degradation.hpp`. **No program uses the shaper yet.**
- **Absent devices:** `hal::AbsentGps`, `AbsentTagSource`, `AbsentVision` — for a robot without
  them, never polled, with no fake in a competition build.

## API 2.1

### 2026-08-15 — 59 defect fixes, mostly additive, with five surface changes to know about

An audit chunk (DEFECTS1) triaged the 84 API defects the documentation pass had reported and
left in place, and fixed 59 of them. **Most are invisible to a caller doing nothing wrong** —
added preconditions, corrected counters, corrected comments. Five change something a compiler
or a caller can see, and one of those is breaking under the version policy.

**BREAKING (unfrozen surfaces, no consumer in the tree):**

- **`control::TrapezoidProfile::isDone()` is no longer `noexcept`**, and both it and `sample()`
  now REJECT a non-finite `t`. `sample(NaN)` used to return a *partially* finite state —
  position and velocity NaN, acceleration a perfectly finite `-aMax` — so a caller screening
  only `acceleration` passed it and forwarded a plausible-looking down-ramp; `isDone(NaN)`
  returned false, so a follower loop terminating on it spun forever. The `noexcept` drop is
  required because the precondition handler throws and a `noexcept` frame would turn a caller
  bug into `std::terminate`. **Migration:** screen your clock, or catch `PreconditionError`.
  This class has no consumer in the library but its own test.
- **`hal::IMechanism` is now non-copyable and non-movable.** It holds the claim token as value
  state, so a copied mechanism arrived already `claimed()`, with `claimant()` pointing at an
  operation registered against the original. **Migration:** construct a mechanism where it lives
  and hand out `IMechanism&` / `IMechanism*` — which is what the header already told you to do.
- **`localization::EkfFusion::state()` and `covariance()` are no longer `noexcept`**; both now
  bounds-check. They are observability accessors, and an out-of-range index was silent UB.

**ADDITIVE, but you may notice:**

- **`diag::MotionOutcome::Unset = 5`**, appended and value-pinned, is the new default for
  `MotionResult::outcome`. An unpopulated result line used to render `✓ SETTLED` — the one
  value meaning success — for a motion that never happened; it now renders `✗ UNSET`.
- **`ProsDigitalOut(smartPort, adiPort)` with the boot state forgotten no longer compiles.**
  It used to select the brain-ADI constructor with `initialState = (bool)2 = true` and fire a
  solenoid HIGH at boot on the wrong port. **Migration:** state the boot level, which the
  required third argument always intended.

**Behaviour you should know changed, without a signature moving:** a `MotionScheduler` destroyed
with a motion still armed now forces the drive to its safe state instead of leaving the motors
at their last command; `MotionConfig` and `Watchdog` reject an *infinite* timeout (`> 0.0` is
satisfied by infinity, and an infinite watchdog can never expire); `Localizer` rejects a `minDt`
that is non-positive or not below `maxDt`; and a stale, finished mechanism operation can no
longer `cancel()` a mechanism a different live operation now owns.

Full triage with evidence for every item, including the 15 findings that did **not** hold and
the 6 that need a decision before they can be fixed, is in the development record.


### 2026-08-14 — the API reference now covers the whole public API — no API change

No library code changed: not a signature, not a default, not a behaviour. This entry is here
because what the documentation *promises* changed, and because one of those promises is
enforced by a build gate you will meet if you contribute.

**The reference went from 2 documented types to every public entity in every shipped header** —
117 pages covering **1,625** types, members, nested types, free functions, namespace-scope
constants and type aliases, plus an [A–Z index](api/all-entities.md) of all of them. The
generator's target list is now a glob over `include/shulib/`, so a header added later is covered
the moment it exists rather than when someone remembers to list it.

**The coverage gate grew with it.** A public entity anywhere under `include/shulib/` with no
`///` documentation now fails the build, naming the entity, its file and its line. If you are
adding to a shipped header, that is the one new obligation: say what it is *for*.

**Being documented does not freeze anything.** The distinction matters and is easy to
mis-read: a change to a *frozen* signature (`Chassis` = F6, `Routine` = F10) fails a
compile-time pin that names the register row, and costs a major version bump plus a migration
note. A change to a documented-but-unfrozen seam costs one comment edit and a regeneration.
Freeze Register rows F11–F14 previously said those seams were deliberately ungated for exactly
this fear; all four are **amended** rather than silently overridden, and their freeze triggers
are unchanged.

**Three locked contracts were not in the reference at all**, and nobody could have known from
reading it: the generator was structurally blind to types with a base-class list, enums with an
explicit underlying type, and everything declared at namespace scope — so the coordinate frame
(`math::Frame` and its two conversions), the accuracy targets (`spec/accuracy.hpp`) and the
units vocabulary (`units::Length`, `Time`, `Voltage`, …) produced no output whatsoever. They are
there now.

**The site is easier to move around**, which matters more at 117 pages than at 2: the top-level
areas are tabs, subsystem groups collapse, each section's overview is its own landing page, and
each header's design commentary folds when it runs long. The nav is generated from the same list
as the pages and byte-checked, because a page missing from the nav was measured to publish
*unreachable*, with exit code 0 and an INFO line.

**Breaking:** nothing. **What you must do:** nothing, unless you contribute to a header — then
write a `///` on anything public you add.

### 2026-08-14 — the mechanism-sensor adapters land (`hal/pros/`, second half) — additive

The other half of the hardware binding: PROS-backed adapters for the mechanism seams —
`ProsDistance`, `ProsOptical`, `ProsDigitalOut`, `ProsDigitalIn`, and `ProsBlockSink` (the
SD-card device behind the blackbox). Each applies its unit conversion exactly once at the
edge, through two new pure conversion headers (`distance_conversion.hpp`,
`optical_conversion.hpp`).

New HAL seam, **not frozen**: `IDigitalIn` (one member — the raw level of a digital input
line; no debouncing, no edge state, no validity channel, each by documented ruling) with
`hal::fake::FakeDigitalIn`. It is an additive sibling outside the F4 freeze, exactly as
`IDigitalOut` and `IController` were; the Freeze Register records the non-freeze out loud
(row F14). It was built ahead of its consumer — the lift-homing switch-or-stall question is
still open — and the register row says that too.

Three behaviours worth knowing before you use these:

- **The distance sensor's "no object" is 9999 mm, in-band.** PROS reports an empty field of
  view as a plain reading that converts to a plausible-looking 393.66 inches — not an error.
  `ProsDistance` maps it to `confidence() == 0.0` (the seam's documented "no usable return"
  channel) and keeps `distance()` finite and far. Threshold `confidence()`, always — the FAQ
  entry "Why does my distance sensor read 393 inches?" walks it.
- **Constructing a `ProsDigitalOut` PHYSICALLY DRIVES the line.** PROS actuates the port at
  construction and defaults it LOW; on a pneumatic that moves the cylinder at boot. The
  adapter refuses the default — the initial state is a required constructor argument, and it
  must agree with the owning `PneumaticMechanism`'s declared safe state (a test pins the
  pattern).
- **A missing SD card does not stop the robot.** `ProsBlockSink` constructs successfully with
  no card, refuses every write (`write()` returns false from the first call — the blackbox's
  drop-and-count design absorbs it), and reports the fact once through `isOpen()`.

**Breaking:** nothing. Every frozen surface (F3/F4/F6/F10) is untouched; the adapters are new
files implementing existing interfaces, and `IDigitalIn` is additive.

**What you must do:** nothing, unless a mechanism uses these sensors — then construct the
`Pros*` adapters and read the seam headers' design notes (each names which PROS call it
binds, which conversion it applies, and which Hardware-Assumptions entries its beliefs rest
on — HA-113 onward, none yet measured on hardware). Honest scope, unchanged from R1a: these
are host-tested against the programmable PROS stand-in, and **the library has still never
driven a robot.**

### 2026-08-13 — first hardware validation of the adapters — no API change

No code changed. Recorded here because it changes what the library's claims are worth.

The `hal/pros/` adapters were run against a physical V5 brain and a real robot for the first time.
Every unit conversion they perform was checked against a turning wheel:

| Conversion | Expected | Measured |
|---|---|---|
| motor degrees → radians | 57.2958 | 57.296 |
| motor RPM → rad/s | 9.5493 | 9.549 |
| motor mA → amps | 1000 | 1000.0 |
| battery raw → volts | — | 13039 → 13.04 V |
| battery capacity → [0,1] | — | 91.0 → 0.91 |
| `micros()` per 1000 ms | 1000000 | 999784 |

Seven previously-guessed hardware assumptions are now measured observations rather than reasoning.
The battery unit was the weakest of them — PROS's vendored headers document no unit for
`battery_get_voltage()` at all.

**What this does not mean:** the library still has never driven a robot. Nothing closed a control
loop, nothing followed a path, and no wheel turned under motion control. These measurements
establish that the platform layer reads and commands real hardware correctly, and nothing more.

### 2026-08-13 — the PROS hardware adapters land (`hal/pros/`) — additive

The library's first hardware binding: header-only adapters under
`include/shulib/hal/pros/` implement the frozen HAL interfaces over real V5 devices —
`ProsClock`, `ProsMotor`, `ProsRotation`, `ProsImu`, `ProsGps`, `ProsBattery`,
`ProsCharSink`, `ProsLineDisplay`, `ProsController`, plus `ProsTickPacer` (the real
10 ms tick over the PROS scheduler). Each adapter applies its unit conversion exactly
once, at the edge, through new pure conversion headers (`motor_conversion.hpp`,
`rotation_conversion.hpp`, `controller_conversion.hpp` — joining the existing IMU and
GPS ones).

New HAL seam, **not frozen**: `IController` (normalized [-1, 1] axes, button levels, a
positive `isConnected()` signal, master/partner support) with `ButtonEdge` for
per-consumer press detection and `hal::fake::FakeController` for tests. The Freeze
Register records the non-freeze out loud (row F13).

**Breaking:** nothing. Every frozen surface (F3/F4/F6/F10) is untouched; the adapters are
new files implementing existing interfaces.

**What you must do:** nothing, unless you want your robot code on real hardware — then
construct the `Pros*` adapters instead of fakes (the shipped `src/main.cpp` is the worked
example, and guide chapter 7 walks it). Honest scope, stated plainly: these adapters are
host-tested against a programmable stand-in for PROS, and **the library has still never
driven a robot** — the beliefs behind every conversion are catalogued in the
[Hardware Assumptions Register](hardware-assumptions.md) (HA-94 onward) and get their
first reality check on a bench, not in a test suite.

### 2026-08-13 — a guaranteed end of run *(reconstructed)* — additive

`sequence/run_guard.hpp` is new: `RunGuard`, plus `RunGuardConfig`, `RunGuardReport` and
`GuardedWaitResult`. It is an `ITickPacer` **decorator** — construct it around the real pacer
and hand it to the chassis — and it is completely **inert until `run()`**.

At the instant you name, it cuts the active motion with zero latency, then **refuses every
later motion**. The refusal is a *latch*, not a pause: cancel-only expiry was measured inert
and marginally counterproductive, so once the deadline passes nothing new starts. Cancel-all
runs strictly before your end action, and an unconditional `hardStopAt` floor fires regardless.

**The library refuses to know your strategy.** No field coordinate, no park pose, no default
lead time, no default match length — both instants and the end action are yours. A team that
ends somewhere else, or does not park at all, is not fighting the library.

**Four limits, stated because they bind:** it proves a *scheduling* property, not a timing
margin on a real brain (loop rate and PROS call latency are still invented register entries);
**nothing preempts pure user code** — there are no background tasks anywhere in this library, so
a `while (true)` loop that never lets a call return keeps the CPU and your end action waits;
the frozen `pause`/`waitFor`/`waitUntil` surfaces cannot see the deadline and pay their full
remainder (a tested limit, not a hidden one); and **"zero travel after the deadline" is a
simulator result** — the host plant is memoryless, so on a robot with mass the honest claim is
*"no new commanded motion"*. Budget lead time for a robot that is still moving when the command
stops.

**Breaking:** nothing. New header, new types; no frozen surface touched.

**What you must do:** nothing, unless you want the guarantee — then wrap your pacer. Note that
the routine-level watchdog (diagnostics item D-8) is the same primitive under a second policy.

### 2026-08-13 — mechanism seam growth *(reconstructed from `version.hpp`)* — additive, 2.0 → 2.1

The mechanism layer grew through the documented additive paths: `RoutineStopCause`
gained an appended `MechanismFailed` enumerator, `Routine::then()` accepts a fourth
return type (`manipulation::MechanismOutcome`), and `FaultCode` appended
`MechanismStalled`. No frozen member changed shape.

**What you must do:** nothing. If you `switch` exhaustively over `RoutineStopCause` or
`FaultCode`, add the new cases.

## API 2.0

> The four entries below all landed **after** the 2.0 freeze and **before** F1 opened 2.1. None
> of them moved the version number, and that is the policy working rather than an oversight:
> `version.hpp` tracks the growth of *frozen* surfaces, and these four added entirely new
> headers and types outside `Chassis` and `Routine`. Nothing previously compiling changed
> meaning. They are reconstructed here because they are user-visible surface an outside team
> would otherwise meet only by reading the source.

### 2026-08-13 — a Kalman tier, which is **not** the default *(reconstructed)* — additive

`localization/ekf_fusion.hpp` adds `EkfFusion` and `EkfFusionConfig` — a 5-state SE(2) extended
Kalman filter behind the unchanged `IFusionPolicy` seam. It can do three things the simpler
filter cannot: weigh two disagreeing correctors against each other by their stated accuracy
(matched against the inverse-variance weighted mean to better than 0.2%), recover from a
displacement the fixed gate makes permanent, and state its own uncertainty as a number.

**`ComplementaryFusion` remains the shipped default, and the measurement is why.** Raced over
eight seeded 60-second runs the EKF finished *less* accurate — 0.351″ vs 0.225″ mean final
error, losing on 7 of 8 seeds. It lost the comparison its own definition-of-done asked it to
win, and that result was kept rather than tuned away. In this simulation dead-reckoning is
already sub-inch and the modelled GPS is noisier than the drift it corrects, so the right move
is mostly to ignore the sensor — which a blunt fixed gain does slightly harder. **Anyone
quoting this as an accuracy improvement is quoting it wrong.** Every noise parameter in it
(HA-83…HA-91) is invented, so real sensor characterization could flip the result either way.

**Breaking:** nothing. **What you must do:** nothing — you get the complementary filter unless
you ask for the EKF.

### 2026-08-13 — absolute heading from AprilTags *(reconstructed)* — additive

`localization/apriltag_corrector.hpp` adds `AprilTagCorrector` and `AprilTagCorrectorConfig`;
`localization/tag_map.hpp` adds `TagMap`, `TagPlacement` and `TagProvenance`. This is the only
source of **absolute heading** in the library — a tag whose field position is known tells the
robot which way it is actually pointing, and it proposes with `providesHeading = true`.

**shulib ships no tag map, deliberately.** Nobody on the project can cite a table of AprilTag
field poses, so inventing one would be a confidently wrong answer rather than a missing one.
`TagMap::add()` refuses a placement whose provenance is `Unspecified` — `Invented` is a
perfectly legitimate answer, *unstated* is not — and an unknown tag id makes the corrector
decline with `RejectedNoTagMapEntry` rather than guess.

**Two things worth knowing before you trust it:** a map that is two inches off produces a
corrector that is *confidently* two inches wrong, and unlike noise that error does not average
out (HA-68). And a **reversed corner winding is catastrophic and silent** — it mirrors the tag,
puts the recovered heading 180° out, and leaves the solver's own reprojection error reading
machine zero. No software self-check can see it; only a physical tag can (HA-69).

**Breaking:** nothing. **What you must do:** nothing, unless you add a camera — then you must
supply the tag map, with provenance.

### 2026-08-12 — GPS position correction *(reconstructed)* — additive

`localization/gps_corrector.hpp` adds `GpsCorrector` and `GpsCorrectorConfig`, the first real
`ICorrector`: it bounds position drift while the field strip is in view. It corrects **position
only** — `providesHeading` stays false, so the IMU keeps owning heading.

The accuracy claim is sized to the evidence rather than rounded up: over eight seeded 60-second
runs the corrected estimate was better on 7 of 8 on final error and 6 of 8 on worst-case. A
shorter scenario that passed 8 of 8 was found and **deliberately not adopted**, as scenario
shopping.

**Driving Skills has no GPS strip**, so `hasFix()` is false for that entire event and the
estimator dead-reckons — a normal operating state, not a fault. The FAQ entry
"Why does `hasFix()` go false in Driving Skills?" is the full version.

**Breaking:** nothing. **What you must do:** nothing, unless you have a GPS.

### 2026-08-12 — the SD-card blackbox *(reconstructed)* — additive

`diag/sd_sink.hpp` adds `SdSink` (an `ITelemetrySink` writing through the `IBlockSink` device
seam) with `SdSinkConfig`, `SdSinkStorage` and `SdSinkBuffers`; `diag/blackbox_format.hpp` adds
the versioned, session-stamped, fixed-width binary format; `diag/blackbox_reader.hpp` adds
`BlackboxReader` and `ReadStatus`. **A run becomes recoverable after the fact, with no laptop.**

Two design points that are the whole value: the **decoder ships in the same chunk as the
encoder** — a format nothing can read is not a record — and it is held to the format by
byte-exact goldens rather than by agreeing with the encoder, so the two cannot drift into
agreeing on something wrong. And the default posture is a **RAM flight recorder** that costs the
card nothing until a fault fires, at which point it writes the triage block **first** and the
preceding ticks after it — in that order, because the fault may be the brownout that cuts the
write short.

**Breaking:** nothing. **What you must do:** nothing. If you want it, give `SdSink` a block
device; the V5 adapter for that arrived later (see the 2026-08-14 entry) and a missing card is
handled rather than fatal.

### 2026-08-12 — the v2 facade freezes *(reconstructed)* — the 2.0 baseline

The public `Chassis` API froze (Freeze Register row F6), followed the same day by the
`Routine` recipe layer (row F10). From this point, routines written against either tier
do not need rewriting: signatures and documented behaviour change only with a major
version bump and a migration note, enforced by compile-time signature pins that fail the
build if a frozen member drifts.

**Breaking, relative to everything before it:** shulib v2 is a ground-up rebuild; the
legacy v1 tree was deleted when `src/main.cpp` was rewired onto the v2 core. There is no
v1→v2 migration path — v1 was never released beyond the team.

**What you must do:** new users start at the [guide](guide/README.md); the
[cookbook](cookbook/README.md) and the [API reference](api/README.md) are the day-to-day
documents.
