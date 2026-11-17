# 15 — Glossary

> **Covers:** every term of art used in this guide, one ordinary sentence each, alphabetically.
> Each term links (→) to the page that explains it, and has its own link: `15-glossary.md#pose`.
> **Read this if:** you hit a word and want the one-line version.

- <a id="abort"></a>**abort (the auton tester's)** — any controller button, a touch of the screen, the controller
  dropping or the robot being disabled, ending the tester's countdown (nothing moves) or its run
  (stopped within one more 10 ms tick, the end action skipped). → [Try a route before the field](16-testing-an-auton-on-the-bench.md)
- <a id="adapter"></a>**adapter (`hal/pros`)** — the thin glue that implements one hardware interface over the real
  PROS SDK, converting units exactly once at the edge. The library core never touches PROS; the
  adapters are the only place that does. → [Extend the library](13-extending-the-library.md)
- <a id="ai-model"></a>**AI model (the camera's) / class table** — the object recogniser chosen on the AI Vision camera in
  VEXcode's AI Vision Utility; it numbers what it finds (its CLASSES), and the library checks the
  camera's name for each number against the robot file's table before a number means anything — VEX's
  Override model's class 3 is a yellow Pin, its Classroom model's a blue ring. → [The API in words](10-the-api.md)
- <a id="apriltag"></a>**AprilTag** — a printed black-and-white square marker, like a chunky QR code, that a camera
  can find and identify. Because its real size and shape are known, the four corners in the image
  are enough to work out how far away it is *and which way it is turned*. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="assertion"></a>**assertion** — one checked claim inside a test; the suite's counts are assertions passed.
  → [Build and upload the programs](07-getting-set-up.md)
- <a id="autonomous"></a>**autonomous** — the phase (or whole run) where the robot acts entirely on its own code and
  sensors, nobody driving. → [What is this?](01-what-is-this.md)
- <a id="auton-task"></a>**auton task** — the task the competition program creates once at boot and owns for the whole
  run. It serves every `autonomous()` request, so PROS deleting its own `autonomous()` task at a
  mode change never cuts a running auton off mid-statement. → [Match day](17-the-competition-program.md)
- <a id="auton-tester"></a>**auton tester ("shulib Auton Tester")** — the bench program that runs one of the team's autons
  from RUN on the brain's screen, through the same library call the competition program's
  `autonomous()` makes; only the trigger differs. Not "the tester" — that word is Bench Tests'.
  → [Try a route before the field](16-testing-an-auton-on-the-bench.md)
- <a id="bearing"></a>**bearing (a camera's)** — which way something is from the camera, as an angle from the robot's
  forward, left positive — always an interval, never one number. → [The API in words](10-the-api.md)
- <a id="bend-grip"></a>**bend grip** — how hard a curve may push the robot sideways in a bend (`MotionConfig::curve.bendAccel`,
  in/s²); the speed limit in a bend is the square root of the grip over the bend's curvature. One of
  the two numbers a team tunes for curves on the robot. → [Driving, turning and curves](05-getting-there.md)
- <a id="blackbox"></a>**blackbox** — the binary run record written to the SD card, so a run can be read back with no
  laptop attached. Its decoder ships with it. → [Reading the log](11-reading-the-diagnostics.md)
- <a id="blocking"></a>**blocking (call)** — a function that doesn't return until its job is done, which is what
  lets a routine read top-to-bottom like a list. → [A routine built by hand](08-your-first-routine.md)
- <a id="body-frame"></a>**Body frame** — see [robot-relative](#robot-relative). → [The field and coordinates](02-the-field-and-coordinates.md)
- <a id="brake-mode"></a>**brake mode** — a motor setting that actively resists rotation when commanded to stop,
  instead of coasting. → [How things fail](06-how-things-fail.md)
- <a id="brownout"></a>**brownout** — battery voltage collapsing under load far enough that motors weaken (and, in
  the worst case, electronics reset). → [How things fail](06-how-things-fail.md)
- <a id="chassis"></a>**chassis** — the robot's mobile base; in code, `Chassis` is the object routines command.
  → [A routine built by hand](08-your-first-routine.md)
- <a id="ci"></a>**CI (continuous integration)** — automation that builds and tests every commit, so a
  breakage is caught the day it's made. → [Build and upload the programs](07-getting-set-up.md)
- <a id="colour-descriptor"></a>**colour descriptor** — one colour the AI Vision camera is taught to box: an id, a red-green-blue value
  and how far the hue and saturation may stray, measured by the team in VEXcode's AI Vision Utility and
  written to the camera by the library at every boot. All zero is UNSET, never a colour. → [The API in words](10-the-api.md)
- <a id="command-id"></a>**command id** — the per-run serial number (1, 2, 3…) stamped on each motion and all its
  diagnostic lines, tying them together. → [Reading the log](11-reading-the-diagnostics.md)
- <a id="competition-program"></a>**competition program ("shulib Competition")** — the program uploaded for a match: the auton
  chosen on the brain's menu runs in `autonomous()` on the program's own auton task, then driver
  control runs the team's driver file. → [Match day](17-the-competition-program.md)
- <a id="control-loop"></a>**control loop** — the measure → compare → command cycle, repeated ~100 times a second.
  → [Driving, turning and curves](05-getting-there.md)
- <a id="corrector"></a>**corrector** — a source of *absolute* fixes that can tell the estimate it is wrong (the GPS
  corrector, the AprilTag corrector). Correctors only ever *propose*; how far the estimate moves
  is the fusion layer's decision. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="covariance"></a>**covariance** — the estimator's written-down answer to "how sure am I?". At its simplest a
  give-or-take distance on the position estimate; properly, a small table covering every pair of
  quantities the estimator tracks, so it can also say things like *"half an inch along my
  direction of travel, three inches sideways"* and record how the errors are linked. It grows
  while dead reckoning and shrinks when a fix is folded. Only the Kalman fusion tier has one.
  → [Where the robot is](03-knowing-where-you-are.md)
- <a id="cross-track-error"></a>**cross-track error** — how far to the side of a line the robot is, as opposed to how far
  along it. For a tank's `moveTo`, the line runs through the target point along the target
  heading; the result line's `xt` column is the cross-track error the leg finished with, + to the
  left of the target heading (tank only, since API 2.3). → [Reading the log](11-reading-the-diagnostics.md)
- <a id="current-cap"></a>**current cap (a mechanism's)** — `.currentCap = {.amps = …}`: the most current EACH of a
  mechanism's motors may draw, written into the motors at boot and enforced by the motors themselves (believed, not measured — HA-219) —
  how hard a lift's string is pulled. Per motor (two motors on one spool pull with twice it); none
  written means the motor's own 2.5 A; a stall signature must sit below it. → [Mechanisms](18-mechanisms.md)
- <a id="dead-reckoning"></a>**dead reckoning** — estimating position purely by accumulating your own measured movement,
  with no external reference (the `DR` flag). → [Where the robot is](03-knowing-where-you-are.md)
- <a id="declaration"></a>**declaration (a mechanism's)** — what a mechanism IS, written once in the robot file
  (`inline constexpr Arm arm{...}`): its kind, its motors and which way each is mounted, its numbers —
  never a port (that is the *wiring*). The same declaration is the handle an auton names
  (`arm["score"]`), checked at build. → [The API in words](10-the-api.md)
- <a id="desaturation"></a>**desaturation** — uniformly scaling down a command that asks more than the wheels can give,
  preserving its direction. → [Drivetrains](04-drivetrains.md)
- <a id="doctest"></a>**doctest** — the test framework the suite is written in. → [Build and upload the programs](07-getting-set-up.md)
- <a id="drift"></a>**drift** — the slow accumulation of estimate error over time, inherent to dead reckoning.
  → [Where the robot is](03-knowing-where-you-are.md)
- <a id="drivetrain"></a>**drivetrain** — the wheels-and-motors arrangement that moves the robot (tank, X-drive,
  H-drive). → [Drivetrains](04-drivetrains.md)
- <a id="encoder"></a>**encoder** — a sensor that counts a shaft's rotation; how wheels report distance. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="error"></a>**error (control)** — the gap between where you want to be and where you are; what control
  loops exist to shrink. → [Driving, turning and curves](05-getting-there.md)
- <a id="estimate"></a>**estimate** — the robot's continuously-updated belief about its pose; always a belief,
  never a fact. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="exit-reason"></a>**exit reason** — how a motion ended: `Settled`, `TimedOut`, or `Cancelled`. → [Driving, turning and curves](05-getting-there.md)
- <a id="fault"></a>**fault** — a detected, named problem (e.g. `ODO_STUCK`), logged and survived rather than
  crashed on. → [How things fail](06-how-things-fail.md)
- <a id="fault-latch"></a>**fault latch** — the recorder that keeps the run's fault history, preserving the *first*
  fault specially as the root cause. → [How things fail](06-how-things-fail.md)
- <a id="feedforward"></a>**feedforward** — the predictive half of control: compute the command physics says you'll
  need, and let feedback fix only the leftovers. → [Driving, turning and curves](05-getting-there.md)
- <a id="field-relative"></a>**field-relative (Field frame)** — directions fixed to the field, independent of which way
  any robot faces. → [The field and coordinates](02-the-field-and-coordinates.md)
- <a id="fix-radius"></a>**fix radius** — how far a camera's field position for a thing may be from where it really is, given
  every stated tolerance — a bound, not a probability; infinite when a stated tolerance is not a number
  that bounds anything. → [The API in words](10-the-api.md)
- <a id="footprint"></a>**footprint (a beam's)** — the stretch of wall a distance sensor's beam covers: its cone widens
  with range (about ±2.3 in at a 7-in reading, for the unofficial ±18° spread), so the wall reset keeps
  the whole footprint, not just the beam's centre, off a keep-out. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="frame"></a>**frame** — a point of view for describing directions; the field's or the robot's. → [The field and coordinates](02-the-field-and-coordinates.md)
- <a id="frame-a-cameras"></a>**frame (a camera's)** — one READ of the AI Vision camera: up to 24 detections, each with its box in
  the picture, the time the read finished, and how the read went — only an `Ok` frame is evidence; a
  failed read has no detections. Not the same word as the frame of reference above. → [The API in words](10-the-api.md)
- <a id="fusion"></a>**fusion** — combining multiple imperfect sensors into one best estimate. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="gain"></a>**gain** — a tuning constant that sets how strongly a controller reacts (e.g. `kP`). → [Driving, turning and curves](05-getting-there.md)
- <a id="gate"></a>**gate / gating** — the fusion layer's bouncer: corrections that disagree wildly with the
  estimate are rejected as probable sensor lies. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="gps"></a>**GPS (V5 GPS sensor)** — VEX's camera-based sensor that reads a coded strip on the field
  wall to report absolute position; no relation to satellites. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="ground-range"></a>**ground range** — how far away something on the floor is, from the camera's height and the picture
  row where its base meets the floor — an interval, refused near the horizon. → [The API in words](10-the-api.md)
- <a id="hal"></a>**HAL (hardware abstraction layer)** — the set of interfaces (`hal/`) standing between the
  library and any actual hardware, real or simulated. → [Extend the library](13-extending-the-library.md)
- <a id="h-drive"></a>**H-drive** — a tank layout plus one sideways wheel: strafes, but slower than it drives.
  → [Drivetrains](04-drivetrains.md)
- <a id="heading"></a>**heading** — the direction the robot faces, as a field angle (0° = +X, counterclockwise
  positive). → [The field and coordinates](02-the-field-and-coordinates.md)
- <a id="heading-bias"></a>**heading bias** — the estimator's learned answer to "how wrong is the IMU today", added to
  every IMU reading before the pose is published. The IMU still owns every actual rotation; the
  bias moves only a fraction of a degree per tick, and only when a tag says so — except once, when
  an auton starts, where it is SET so the heading equals the one the auton's start pose declares
  (`Localizer::reseatHeading`; your placement, not a sensor). → [Where the robot is](03-knowing-where-you-are.md)
- <a id="holonomic"></a>**holonomic** — able to move in any direction regardless of facing, while also rotating.
  → [Drivetrains](04-drivetrains.md)
- <a id="homing"></a>**homing (a mechanism's)** — finding where an arm's or a lift's zero is, before it may go to any
  position: at program start with no motion (it rests on its stop; an absolute angle sensor; a home
  switch already pressed) or by a homing step that pushes until its switch is pressed or it stalls
  against its stop. Every home logs where its zero went. → [Mechanisms](18-mechanisms.md)
- <a id="host-test-plant"></a>**host test plant** — the suite's model of a robot and its field, run on the laptop. It is ideal —
  motors that answer at once, wheels that never slip — unless a test adds motor lag, a weak side, wheel
  slip or sensor noise, and it has walls, pieces or a camera's view only where a test puts them. Every
  auton is run on it by the host tests. The route preview runs it IDEAL, with nothing on the field to stop
  or push the robot, and draws the run; its times there are the library acting on a perfect robot — not a
  floor or a ceiling for the real one. → [Driving, turning and curves](05-getting-there.md), [Build and upload the programs](07-getting-set-up.md), [Try a route before the field](16-testing-an-auton-on-the-bench.md)
- <a id="host-side"></a>**host-side** — running on an ordinary computer rather than the robot. → [Build and upload the programs](07-getting-set-up.md)
- <a id="imu"></a>**IMU (inertial measurement unit)** — the sensor that measures rotation directly; the owner
  of the heading estimate. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="kalman-filter"></a>**Kalman filter** — a fusion method that carries a covariance and uses it to decide how much to
  trust each new measurement. shulib ships one (an *extended* Kalman filter, meaning it copes with
  the fact that the robot's motion is not a straight line) as the optional second fusion tier.
  → [Where the robot is](03-knowing-where-you-are.md)
- <a id="keep-out"></a>**keep-out (the wall reset's)** — a stretch of a field wall a side sensor's beam must not land on
  (the Toggle, its brackets, a Loader, a corner, a VEX U Load Zone, the start pieces until the beam's
  spread is measured): something other than the wall may stand there. The reset refuses `keep-out`
  and prints the nearest clean stretch. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="kinematics"></a>**kinematics** — the geometry converting a desired chassis velocity into per-wheel speeds
  and back. → [Drivetrains](04-drivetrains.md)
- <a id="localization"></a>**localization** — the whole problem (and code layer) of knowing where the robot is. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="look-ahead"></a>**look-ahead** — how far along a curve's path, past its closest point, the robot aims each tick
  (`MotionConfig::curve.lookAhead`, 8 in by default): shorter tracks tighter bends and wobbles more,
  longer is steadier and cuts tight bends. The other number a team tunes for curves. → [Driving, turning and curves](05-getting-there.md)
- <a id="mahalanobis-distance"></a>**Mahalanobis distance** — how far a measurement disagrees with the estimate, measured in units
  of *how wrong the estimate could plausibly be* rather than in inches. Two inches is a lot when
  the estimator is sure of itself and nothing at all when it has been driving blind. The Kalman
  tier refuses anything past three. → [Where the robot is](03-knowing-where-you-are.md), [Reading the log](11-reading-the-diagnostics.md)
- <a id="mechanism"></a>**mechanism** — anything on the robot that is not the drivetrain (an intake, a lift, a clamp),
  behind a seam with one required verb: go to your safe state. The library ships five generic KINDS
  you declare yours with (`Arm`, `Lift`, `Flywheel`, `Power`, `Pneumatic`) and the grammar for
  commanding and confirming them — never a season's mechanism: what your robot has, and what you
  call it, is yours to write. → [The API in words](10-the-api.md), [Extend the library](13-extending-the-library.md)
- <a id="mechanism-service"></a>**mechanism service** — the one object, ticked once per control tick (inside shulib's waits and
  motions, and — from the core's driver controls on — every driver-loop tick), that runs every
  mechanism's behaviour, so an arm keeps
  holding and an intake keeps running while the robot drives; starved by a busy loop of your own, it
  leaves each mechanism at its last command. → [Mechanisms](18-mechanisms.md)
- <a id="declared-safe-state"></a>**declared safe state** — what a given mechanism does when a run ends or is cancelled, stated
  per mechanism rather than assumed. A lift holds; an intake coasts; a clamp that has
  successfully grabbed *keeps holding*, because a clamp whose safe state were "open" would fling
  its game piece the instant the grab succeeded. → [Extend the library](13-extending-the-library.md)
- <a id="mirror"></a>**mirror (a mirrored auton)** — robot two running robot one's route reflected across the field's
  skills diagonal (`(x, y)` facing `h` becomes `(-y, -x)` facing `-90° - h`), registered in one line with
  `competition::mirrored(entry, override2026::kSkillsMirror)`: every field number the route writes is
  reflected for that run, and the robot is seated at the reflected start. The robot itself is not
  mirrored. → [The field and coordinates](02-the-field-and-coordinates.md), [Match day](17-the-competition-program.md)
- <a id="mutation-check"></a>**mutation check** — deliberately breaking code to confirm a test actually notices; the
  antidote to tests that pass no matter what. → [Extend the library](13-extending-the-library.md)
- <a id="nan"></a>**NaN ("not a number")** — the poison value floating-point math produces from impossible
  operations; guarded against at boundaries. → [How things fail](06-how-things-fail.md)
- <a id="not-checkable"></a>**not checkable** — the boot line of a 3-wire role (a switch, a solenoid's line): the brain cannot
  see a 3-wire device, so the role is made and reported, never verified. → [The API in words](10-the-api.md)
- <a id="odometry"></a>**odometry** — tracking position by accumulating wheel-measured movement each tick. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="omniwheel"></a>**omniwheel** — a wheel with rollers around its rim, so it can slide freely along its axle
  direction; what makes X-drives possible. → [Drivetrains](04-drivetrains.md)
- <a id="one-camera-explains-it"></a>**one camera explains it** — the rule for checking two numbers measured from ONE camera against each
  other (a band's two edges; a Pin's base and its height): ask whether a single camera inside the
  stated tolerances sees the box that way, never whether two separately widened intervals overlap.
  → [The API in words](10-the-api.md)
- <a id="oscillation"></a>**oscillation** — over-aggressive control rocking back and forth around the target instead
  of settling. → [Driving, turning and curves](05-getting-there.md)
- <a id="overshoot"></a>**overshoot** — carrying past the target before coming back; the result line's `over`
  field. → [Driving, turning and curves](05-getting-there.md)
- <a id="pacer"></a>**pacer** — the small seam that advances the world one tick while a motion blocks (sim:
  step physics; robot: wait for the next tick). → [A routine built by hand](08-your-first-routine.md)
- <a id="pid"></a>**PID (proportional–integral–derivative)** — the classic feedback strategy: push
  proportionally to error, lean on stubborn leftovers, brake early on fast approaches. → [Driving, turning and curves](05-getting-there.md)
- <a id="piece-sensor"></a>**piece sensor** — a mechanism's sensor for "is a game piece here": a distance sensor (closer than
  its threshold), an optical sensor (its proximity) or a limit switch. One that is missing at boot
  leaves its mechanism running without it. → [The API in words](10-the-api.md)
- <a id="plant"></a>**plant** — control-engineering jargon for "the thing being controlled"; here, the simulated
  robot's physics. → [Build and upload the programs](07-getting-set-up.md)
- <a id="pnp"></a>**PnP (perspective-n-point)** — the geometry that turns a marker's corners in an image back
  into its position and orientation relative to the camera. In shulib it is a standalone,
  testable function, deliberately separate from the corrector that consumes its output. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="pose"></a>**pose** — position plus heading: (x, y, θ). → [The field and coordinates](02-the-field-and-coordinates.md)
- <a id="possibly-full"></a>**possibly full** — a camera read that reported the camera's most, 24 or more: the camera sends at
  most 24 and may hold more, and what was not sent may be what matters, so such a read is never
  evidence that something is absent. → [The API in words](10-the-api.md)
- <a id="process-noise"></a>**process noise** — how much confidence the estimator gives up per tick simply from time
  passing and distance covered — the thing that makes a covariance grow while dead reckoning.
  Scaling it with distance travelled rather than with time is what lets a truthful correction
  through after a long blind stretch. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="precondition"></a>**precondition** — a rule about inputs a function checks at its door, rejecting nonsense
  loudly before anything moves. → [The API in words](10-the-api.md)
- <a id="pros"></a>**PROS** — the open-source V5 runtime the robot build runs on; the library core never
  touches it directly. → [Build and upload the programs](07-getting-set-up.md)
- <a id="provenance"></a>**provenance** — where a number came from: a published specification, a measurement, or a
  guess. The tag map refuses an entry that does not say, because a guessed tag position and a
  specified one are indistinguishable in the arithmetic and completely different in their
  consequences. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="pure-pursuit"></a>**pure pursuit** — the way the curve verbs steer: every tick, aim along the arc that carries the
  robot through the point a look-ahead ahead of it on the path. LemLib's `follow` works the same way;
  in this library only `curveTo` / `reverseCurveTo` steer toward a point at all. → [Driving, turning and curves](05-getting-there.md)
- <a id="rate-limiting"></a>**rate limiting** — capping diagnostic output volume, with every dropped line counted and
  announced. → [Reading the log](11-reading-the-diagnostics.md)
- <a id="read-fault"></a>**read fault** — a sensor read the adapter screened because the device failed: a scalar sensor
  answers with its last good value instead, the camera with a frame that says the read failed;
  counted (`IReadFaults`), so that a reader — the mechanism core's service, which reads a piece
  sensor's count since the core's second part — can tell a held value from a measured one (for the
  camera: a read that was not evidence). → [The API in words](10-the-api.md)
- <a id="recipe"></a>**recipe** — a routine written as a `Routine` chain of steps that runs in exactly the order
  it reads and stops (parking the robot) at the first failed step — unless that step was written
  after `onFail(carryOn)` or `onFail(skipTo("..."))`. → [The steps of a route](09-the-recipe-api.md)
- <a id="result-line"></a>**result line** — the one-line verdict each motion leaves in the transcript. → [Reading the log](11-reading-the-diagnostics.md)
- <a id="robot-relative"></a>**robot-relative (Body frame)** — directions fixed to the robot ("forward" = wherever its
  front points right now). → [The field and coordinates](02-the-field-and-coordinates.md)
- <a id="role"></a>**role (a mechanism's)** — one part a mechanism declares: a motor ("motor 1"), a 3-wire line, an
  angle sensor, a home switch, a piece sensor. The declaration states the roles, the wiring their
  ports, and each gets one boot line. → [The API in words](10-the-api.md)
- <a id="route-id"></a>**route id** — the word a route marker names to make the robot do something at a waypoint. Every
  mechanism publishes its actions as route ids automatically, derived from its declaration
  (`arm_score`, `intake_until_piece`, `claw_closed`); one that would not fit, would repeat or is the
  library's own is refused at build. A program registers a robot's ids in one call, and a marker then
  runs exactly the step an auton's line runs, blocking at its waypoint until its verdict. → [The API in words](10-the-api.md), [Mechanisms](18-mechanisms.md)
- <a id="route-preview"></a>**route preview** — `tools/route_preview.py "<auton name>"`: one auton run on the host test plant and
  drawn as one offline page — the field, the path, every step numbered with its time, failures in red with
  the library's own sentence, and THROUGH (a wall or a Loader hit, a failure) told apart from TOUCH (a note).
  A plan, not a robot test. → [Try a route before the field](16-testing-an-auton-on-the-bench.md)
- <a id="routine"></a>**routine** — the function encoding one autonomous plan as a sequence of commands. → [A routine built by hand](08-your-first-routine.md)
- <a id="run-guard"></a>**run guard** — a wrapper around the tick pacer that gives a whole routine a hard deadline: it
  cuts the active motion — and every mechanism operation still running that registered its claim,
  as the library's own do — at the instant you name, *refuses* every motion started after it, and
  runs your end-of-run action before an unconditional floor — unless the program *abandons* the
  run (the field disabled the robot mid-auton), when the motion and those operations are cut at
  once and the end action is skipped. It holds no opinion about strategy — no default match length, no park pose; both
  instants and the action are yours. → [How things fail](06-how-things-fail.md), [What it cannot do yet](14-what-it-cannot-do-yet.md), [Match day](17-the-competition-program.md)
- <a id="end-of-run-action"></a>**end-of-run action** — the thing you guarantee happens before time runs out (park, retract,
  end legal). Supplied by you to the run guard, which performs it after cancelling everything
  else. → [How things fail](06-how-things-fail.md)
- <a id="run-summary"></a>**run summary** — the one-screen ledger printed at the end of a run. → [Reading the log](11-reading-the-diagnostics.md)
- <a id="seat"></a>**seat (an axis)** — set ONE coordinate of the estimate to a measured value and leave everything
  else as it was — the other coordinate, the heading, the drift record (`Localizer::seatAxis`). What
  a wall reset does when every check passes; unlike `setPose`, it never passes a one-coordinate
  measurement off as a fix of both. → [Where the robot is](03-knowing-where-you-are.md), [The API in words](10-the-api.md)
- <a id="session-header"></a>**session header** — the transcript's opening lines: which build, routine, ports, battery.
  → [Reading the log](11-reading-the-diagnostics.md)
- <a id="settling"></a>**settling** — the three-part test for "genuinely arrived": close enough, slow enough, for
  long enough. → [Driving, turning and curves](05-getting-there.md)
- <a id="side-sensor"></a>**side sensor** — a V5 Distance Sensor on the robot's left or right side, facing straight out,
  declared in the robot file with its MEASURED mount (`RobotConfig::sensors.walls`); what the wall
  reset reads. → [Where the robot is](03-knowing-where-you-are.md), [The API in words](10-the-api.md)
- <a id="sim-harness"></a>**sim harness** — an assembled simulated robot (`SimHarness`): plant, sensors, clock, and
  their wiring. The test suite is its heaviest user, but it is a shipped header and [chapter 8](08-your-first-routine.md) has
  you build one yourself. → [A routine built by hand](08-your-first-routine.md)
- <a id="size-range"></a>**size range** — how far away an upright thing of known height is, from how tall its box is in the
  picture and where in the picture it sits — an interval. A Pin's box takes in its top face too, so its
  size range reads a little nearer than its near rim. → [The API in words](10-the-api.md)
- <a id="snap"></a>**snap (and never-snap)** — jumping the estimate straight to whatever a sensor claims. shulib's
  correctors never do it, in position or in heading: corrections are applied as small bounded steps, so a
  single bad reading cannot teleport or spin the robot's idea of itself. The one exception is a step
  YOU call — the wall reset seats ONE position coordinate from a side sensor's reading, only with no
  motion running and only within the limit you give it; nothing ever snaps the heading. → [Where the robot is](03-knowing-where-you-are.md), [The API in words](10-the-api.md)
- <a id="soft-stop"></a>**soft stop** — a mechanism's travel limits enforced while it runs: a computed target past one is
  clamped to it (and said), and a button pushing toward one is taken over where it could no longer
  stop before it, brought to it and held AT it by the closed-loop hold — never pushed harder than the
  button pushes, one soft stop per push. A constant target past one never compiles. → [Mechanisms](18-mechanisms.md)
- <a id="stall-signature"></a>**stall signature** — what a jam looks like for one mechanism, stated in its declaration
  (`.stall`): at least this current while the motor turns slower than this, for this long. Required
  on an arm, a lift and a flywheel; a trip is STALLED, the safe state and a `MECHANISM_STALLED`
  fault. A `homeByStall` push has its own, far smaller one (a gentle push stalls at a small
  current). → [Mechanisms](18-mechanisms.md)
- <a id="starting-values"></a>**starting values** — the numbers the library starts a position or speed mechanism's loop with
  (gains, the smooth move's speed and ramp, "reached"), derived from its cartridge and gearing,
  invented and PROVISIONAL (HA-182) until `.tuning` replaces them; a boot line words them (every
  program that boots a robot's list prints it, since the core's fifth part). → [Mechanisms](18-mechanisms.md)
- <a id="step"></a>**step (recipe)** — one link in a recipe chain, plus the record of whether it ran, succeeded,
  or was skipped. Most steps delegate to a single chassis command, but not all: a wait step has
  no motion behind it, a `then()` step runs your own code or a mechanism operation and judges
  it on what that returns, and a mechanism step (`move`, `run`, `set`, ...) runs a declared mechanism
  through the mechanism service. → [The steps of a route](09-the-recipe-api.md), [Mechanisms](18-mechanisms.md)
- <a id="strafe"></a>**strafe** — to move sideways without turning. → [Drivetrains](04-drivetrains.md)
- <a id="strafe-authority"></a>**strafe authority** — a drivetrain's sustainable sideways speed as a fraction of its
  forward speed (X: 1.0, tank: 0.0, H: between). → [Drivetrains](04-drivetrains.md)
- <a id="tag-map"></a>**tag map** — your table of where each AprilTag sits on the field and which way it faces. It is
  input, not something the library knows: shulib ships no map, because nobody here can cite one.
  An error in it does not average out. → [Where the robot is](03-knowing-where-you-are.md)
- <a id="tank-drive"></a>**tank drive** — left wheels/right wheels; drives and turns, cannot strafe. → [Drivetrains](04-drivetrains.md)
- <a id="tank-steering"></a>**tank steering** — what a tank's `moveTo` does since API 2.3 to stay on the line you drew
  (through the target point, along the target heading): it bends the heading it aims for back
  toward the line, at most 15°, and cancels the turn a weaker side adds. Only for a leg that starts
  within 15° of its target heading; it never aims at the point. The `TSTR` flag marks it in the
  log. → [Drivetrains](04-drivetrains.md), [Reading the log](11-reading-the-diagnostics.md)
- <a id="telemetry-sink"></a>**telemetry sink** — anything that accepts the diagnostic stream: the terminal formatter, a
  test capture, a level filter, a rate limiter, or the SD-card blackbox (which is shipped, not
  future). → [Reading the log](11-reading-the-diagnostics.md)
- <a id="terminal-word"></a>**terminal word** — the competition program's auton task's signal that a run request is over
  (FINISHED), published on EVERY path — an auton that finished, NONE, a refusal, an abandon, a
  caught throw. Driver control waits for it, with no upper bound, before it touches the robot.
  → [Match day](17-the-competition-program.md)
- <a id="tick"></a>**tick** — one iteration of the control loop, nominally every 10 ms. → [Driving, turning and curves](05-getting-there.md)
- <a id="tier"></a>**tier** — one of the master plan's four levels of using shulib (zero-code hardware,
  data-driven auton, recipes, full API), each a strict superset of the one below. **Only the top
  two exist today**: recipes and the full C++ API. The two zero-code tiers are unbuilt.
  → [The steps of a route](09-the-recipe-api.md), [What it cannot do yet](14-what-it-cannot-do-yet.md)
- <a id="timed-run"></a>**Timed Run** — the controller's own way to start a pretend match from its program menu: the
  competition program's start on the bench. A program started that way comes up disabled; what the
  Timed Run does next is not yet measured (the Hardware Assumptions Register, HA-162). → [Match day](17-the-competition-program.md)
- <a id="tolerance"></a>**tolerance** — the "close enough" threshold in settling. → [Driving, turning and curves](05-getting-there.md)
- <a id="tolerance-box"></a>**tolerance box** — every combination of the camera's stated errors (its height, pitch and yaw, each
  of the optics' two focal lengths on its own, a box edge's jitter) up to their limits; every camera
  answer's interval is its exact span over the box — computed at each corner of it, and at the few points
  inside where an answer turns back (a box on the lens's level, the principal point's column). → [The API in words](10-the-api.md)
- <a id="tracking-center"></a>**tracking center** — the specific point on the robot whose position the estimate tracks.
  → [The field and coordinates](02-the-field-and-coordinates.md)
- <a id="tracking-wheel"></a>**tracking wheel** — a small unpowered wheel with an encoder, pressed to the floor purely to
  measure travel (it can't slip from motor torque, having none). → [Where the robot is](03-knowing-where-you-are.md)
- <a id="trajectory"></a>**trajectory** — an ordered list of waypoints driven as chained moves. → [The API in words](10-the-api.md)
- <a id="triage"></a>**triage** — the compact "what went wrong" block written first when a fault fires (and printed
  at run end), before the surrounding ticks — first because the fault may be the brownout that
  cuts the write short. → [Reading the log](11-reading-the-diagnostics.md)
- <a id="typed-units"></a>**typed units** — lengths, angles, velocities, and times as distinct compile-time types
  (`24_in`, `90_deg`), so unit mix-ups can't compile. → [The API in words](10-the-api.md)
- <a id="v5-brain"></a>**V5 brain** — the VEX controller-computer that runs the robot's code. → [What is this?](01-what-is-this.md)
- <a id="vex-u"></a>**VEX U** — the university division of the VEX Robotics Competition; two robots per team.
  → [What is this?](01-what-is-this.md)
- <a id="wall-reset"></a>**wall reset** — a step an auton calls: parked beside a field wall, nose along it and square, the
  side sensor facing that wall re-seats ONE coordinate of the estimate (x against the left or right
  wall, y against the red or blue wall) — or refuses, saying which check failed. Needs a FIELD start;
  never the heading; never in the background (`competition::resetFromWall`). → [Where the robot is](03-knowing-where-you-are.md), [The API in words](10-the-api.md)
- <a id="watch"></a>**watch (the competition program's)** — the pacer under the run guard that reads the
  competition status every tick and asks the guard to abandon a live auton the moment the robot is
  disabled or leaves autonomous. A controller button never ends an auton in a match (that is the
  auton tester's abort). → [Match day](17-the-competition-program.md)
- <a id="watchdog"></a>**watchdog** — the per-motion timer guaranteeing no motion can run (or block) forever.
  → [Driving, turning and curves](05-getting-there.md)
- <a id="waypoint"></a>**waypoint** — one target pose in a trajectory. → [The API in words](10-the-api.md)
- <a id="wiring"></a>**wiring (a robot's)** — the block of the robot file saying which port each mechanism's roles are
  plugged into, one `wire(...)` line per mechanism; re-cabling changes only these lines. A smart
  port is a number, a 3-wire port a letter in quotes, and each role is written once. → [The API in words](10-the-api.md)
- <a id="x-drive"></a>**X-drive** — four omniwheels at 45°; fully holonomic, strafes as fast as it drives. → [Drivetrains](04-drivetrains.md)
