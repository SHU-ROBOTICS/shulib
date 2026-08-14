# 15 — Glossary

> **Covers:** every term of art used in this guide, one ordinary sentence each, alphabetically.
> Each term is explained properly in the chapter noted.
> **Read this if:** you hit a word and want the one-line version.

- **adapter (`hal/pros`)** — the thin glue that implements one hardware interface over the real
  PROS SDK, converting units exactly once at the edge. The library core never touches PROS; the
  adapters are the only place that does. [Ch. 13]
- **AprilTag** — a printed black-and-white square marker, like a chunky QR code, that a camera
  can find and identify. Because its real size and shape are known, the four corners in the image
  are enough to work out how far away it is *and which way it is turned*. [Ch. 3]
- **assertion** — one checked claim inside a test; the suite's counts are assertions passed.
  [Ch. 7]
- **autonomous** — the phase (or whole run) where the robot acts entirely on its own code and
  sensors, nobody driving. [Ch. 1]
- **blackbox** — the binary run record written to the SD card, so a run can be read back with no
  laptop attached. Its decoder ships with it. [Ch. 11]
- **blocking (call)** — a function that doesn't return until its job is done, which is what
  lets a routine read top-to-bottom like a list. [Ch. 8]
- **Body frame** — see *robot-relative*. [Ch. 2]
- **brake mode** — a motor setting that actively resists rotation when commanded to stop,
  instead of coasting. [Ch. 6]
- **brownout** — battery voltage collapsing under load far enough that motors weaken (and, in
  the worst case, electronics reset). [Ch. 6]
- **chassis** — the robot's mobile base; in code, `Chassis` is the object routines command.
  [Ch. 8]
- **CI (continuous integration)** — automation that builds and tests every commit, so a
  breakage is caught the day it's made. [Ch. 7]
- **command id** — the per-run serial number (1, 2, 3…) stamped on each motion and all its
  diagnostic lines, tying them together. [Ch. 11]
- **control loop** — the measure → compare → command cycle, repeated ~100 times a second.
  [Ch. 5]
- **corrector** — a source of *absolute* fixes that can tell the estimate it is wrong (the GPS
  corrector, the AprilTag corrector). Correctors only ever *propose*; how far the estimate moves
  is the fusion layer's decision. [Ch. 3]
- **covariance** — the estimator's written-down answer to "how sure am I?". At its simplest a
  give-or-take distance on the position estimate; properly, a small table covering every pair of
  quantities the estimator tracks, so it can also say things like *"half an inch along my
  direction of travel, three inches sideways"* and record how the errors are linked. It grows
  while dead reckoning and shrinks when a fix is folded. Only the Kalman fusion tier has one.
  [Ch. 3]
- **dead reckoning** — estimating position purely by accumulating your own measured movement,
  with no external reference (the `DR` flag). [Ch. 3]
- **desaturation** — uniformly scaling down a command that asks more than the wheels can give,
  preserving its direction. [Ch. 4]
- **doctest** — the test framework the suite is written in. [Ch. 7]
- **drift** — the slow accumulation of estimate error over time, inherent to dead reckoning.
  [Ch. 3]
- **drivetrain** — the wheels-and-motors arrangement that moves the robot (tank, X-drive,
  H-drive). [Ch. 4]
- **encoder** — a sensor that counts a shaft's rotation; how wheels report distance. [Ch. 3]
- **error (control)** — the gap between where you want to be and where you are; what control
  loops exist to shrink. [Ch. 5]
- **estimate** — the robot's continuously-updated belief about its pose; always a belief,
  never a fact. [Ch. 3]
- **exit reason** — a motion's honest ending: `Settled`, `TimedOut`, or `Cancelled`. [Ch. 5]
- **fault** — a detected, named problem (e.g. `ODO_STUCK`), logged and survived rather than
  crashed on. [Ch. 6]
- **fault latch** — the recorder that keeps the run's fault history, preserving the *first*
  fault specially as the root cause. [Ch. 6]
- **feedforward** — the predictive half of control: compute the command physics says you'll
  need, and let feedback fix only the leftovers. [Ch. 5]
- **field-relative (Field frame)** — directions fixed to the field, independent of which way
  any robot faces. [Ch. 2]
- **frame** — a point of view for describing directions; the field's or the robot's. [Ch. 2]
- **fusion** — combining multiple imperfect sensors into one best estimate. [Ch. 3]
- **gain** — a tuning constant that sets how strongly a controller reacts (e.g. `kP`). [Ch. 5]
- **gate / gating** — the fusion layer's bouncer: corrections that disagree wildly with the
  estimate are rejected as probable sensor lies. [Ch. 3]
- **GPS (V5 GPS sensor)** — VEX's camera-based sensor that reads a coded strip on the field
  wall to report absolute position; no relation to satellites. [Ch. 3]
- **HAL (hardware abstraction layer)** — the set of interfaces (`hal/`) standing between the
  library and any actual hardware, real or simulated. [Ch. 13]
- **H-drive** — a tank layout plus one sideways wheel: strafes, but slower than it drives.
  [Ch. 4]
- **heading** — the direction the robot faces, as a field angle (0° = +X, counterclockwise
  positive). [Ch. 2]
- **heading bias** — the estimator's learned answer to "how wrong is the IMU today", added to
  every IMU reading before the pose is published. The IMU still owns every actual rotation; the
  bias moves only a fraction of a degree per tick, and only when a tag says so. [Ch. 3]
- **holonomic** — able to move in any direction regardless of facing, while also rotating.
  [Ch. 4]
- **host-side** — running on an ordinary computer rather than the robot. [Ch. 7]
- **IMU (inertial measurement unit)** — the sensor that measures rotation directly; the owner
  of the heading estimate. [Ch. 3]
- **Kalman filter** — a fusion method that carries a covariance and uses it to decide how much to
  trust each new measurement. shulib ships one (an *extended* Kalman filter, meaning it copes with
  the fact that the robot's motion is not a straight line) as the optional second fusion tier.
  [Ch. 3]
- **kinematics** — the geometry converting a desired chassis velocity into per-wheel speeds
  and back. [Ch. 4]
- **localization** — the whole problem (and code layer) of knowing where the robot is. [Ch. 3]
- **Mahalanobis distance** — how far a measurement disagrees with the estimate, measured in units
  of *how wrong the estimate could plausibly be* rather than in inches. Two inches is a lot when
  the estimator is sure of itself and nothing at all when it has been driving blind. The Kalman
  tier refuses anything past three. [Ch. 3, Ch. 11]
- **mechanism** — anything on the robot that is not the drivetrain (an intake, a lift, a clamp),
  behind a seam with one required verb: go to your safe state. The library ships the grammar for
  commanding and confirming them, never a named `Intake` or `Lift` — those change every season
  and are yours to write. [Ch. 13]
- **declared safe state** — what a given mechanism does when a run ends or is cancelled, stated
  per mechanism rather than assumed. A lift holds; an intake coasts; a clamp that has
  successfully grabbed *keeps holding*, because a clamp whose safe state were "open" would fling
  its game piece the instant the grab succeeded. [Ch. 13]
- **mutation check** — deliberately breaking code to confirm a test actually notices; the
  antidote to tests that pass no matter what. [Ch. 13]
- **NaN ("not a number")** — the poison value floating-point math produces from impossible
  operations; guarded against at boundaries. [Ch. 6]
- **odometry** — tracking position by accumulating wheel-measured movement each tick. [Ch. 3]
- **omniwheel** — a wheel with rollers around its rim, so it can slide freely along its axle
  direction; what makes X-drives possible. [Ch. 4]
- **oscillation** — over-aggressive control rocking back and forth around the target instead
  of settling. [Ch. 5]
- **overshoot** — carrying past the target before coming back; the result line's `over`
  field. [Ch. 5]
- **pacer** — the small seam that advances the world one tick while a motion blocks (sim:
  step physics; robot: wait for the next tick). [Ch. 8]
- **PID (proportional–integral–derivative)** — the classic feedback strategy: push
  proportionally to error, lean on stubborn leftovers, brake early on fast approaches. [Ch. 5]
- **plant** — control-engineering jargon for "the thing being controlled"; here, the simulated
  robot's physics. [Ch. 7]
- **PnP (perspective-n-point)** — the geometry that turns a marker's corners in an image back
  into its position and orientation relative to the camera. In shulib it is a standalone,
  testable function, deliberately separate from the corrector that consumes its output. [Ch. 3]
- **pose** — position plus heading: (x, y, θ). [Ch. 2]
- **process noise** — how much confidence the estimator gives up per tick simply from time
  passing and distance covered — the thing that makes a covariance grow while dead reckoning.
  Scaling it with distance travelled rather than with time is what lets a truthful correction
  through after a long blind stretch. [Ch. 3]
- **precondition** — a rule about inputs a function checks at its door, rejecting nonsense
  loudly before anything moves. [Ch. 10]
- **PROS** — the open-source V5 runtime the robot build runs on; the library core never
  touches it directly. [Ch. 7]
- **provenance** — where a number came from: a published specification, a measurement, or a
  guess. The tag map refuses an entry that does not say, because a guessed tag position and a
  specified one are indistinguishable in the arithmetic and completely different in their
  consequences. [Ch. 3]
- **rate limiting** — capping diagnostic output volume, with every dropped line counted and
  announced. [Ch. 11]
- **recipe** — a routine written as a `Routine` chain of steps that runs in exactly the order
  it reads and stops (parking the robot) at the first failed step. [Ch. 9]
- **result line** — the one-line verdict each motion leaves in the transcript. [Ch. 11]
- **robot-relative (Body frame)** — directions fixed to the robot ("forward" = wherever its
  front points right now). [Ch. 2]
- **routine** — the function encoding one autonomous plan as a sequence of commands. [Ch. 8]
- **run guard** — a wrapper around the tick pacer that gives a whole routine a hard deadline: it
  cuts the active motion at the instant you name, *refuses* every motion started after it, and
  runs your end-of-run action before an unconditional floor. It holds no opinion about strategy —
  no default match length, no park pose; both instants and the action are yours. [Ch. 6, Ch. 14]
- **end-of-run action** — the thing you guarantee happens before time runs out (park, retract,
  end legal). Supplied by you to the run guard, which performs it after cancelling everything
  else. [Ch. 6]
- **run summary** — the one-screen ledger printed at the end of a run. [Ch. 11]
- **session header** — the transcript's opening lines: which build, routine, ports, battery.
  [Ch. 11]
- **settling** — the three-part test for "genuinely arrived": close enough, slow enough, for
  long enough. [Ch. 5]
- **sim harness** — an assembled simulated robot (`SimHarness`): plant, sensors, clock, and
  their wiring. The test suite is its heaviest user, but it is a shipped header and chapter 8 has
  you build one yourself. [Ch. 8]
- **snap (and never-snap)** — jumping the estimate straight to whatever a sensor claims. shulib
  never does it, in position or in heading: corrections are applied as small bounded steps, so a
  single bad reading cannot teleport or spin the robot's idea of itself. [Ch. 3]
- **step (recipe)** — one link in a recipe chain, plus the record of whether it ran, succeeded,
  or was skipped. Most steps delegate to a single chassis command, but not all: a wait step has
  no motion behind it, and a `then()` step runs your own code or a mechanism operation and judges
  it on what that returns. [Ch. 9]
- **strafe** — to move sideways without turning. [Ch. 4]
- **strafe authority** — a drivetrain's sustainable sideways speed as a fraction of its
  forward speed (X: 1.0, tank: 0.0, H: between). [Ch. 4]
- **tag map** — your table of where each AprilTag sits on the field and which way it faces. It is
  input, not something the library knows: shulib ships no map, because nobody here can cite one.
  An error in it does not average out. [Ch. 3]
- **tank drive** — left wheels/right wheels; drives and turns, cannot strafe. [Ch. 4]
- **telemetry sink** — anything that accepts the diagnostic stream: the terminal formatter, a
  test capture, a level filter, a rate limiter, or the SD-card blackbox (which is shipped, not
  future). [Ch. 11]
- **tick** — one iteration of the control loop, nominally every 10 ms. [Ch. 5]
- **tier** — one of the master plan's four levels of using shulib (zero-code hardware,
  data-driven auton, recipes, full API), each a strict superset of the one below. **Only the top
  two exist today**: recipes and the full C++ API. The two zero-code tiers are unbuilt.
  [Ch. 9, Ch. 14]
- **tolerance** — the "close enough" threshold in settling. [Ch. 5]
- **tracking center** — the specific point on the robot whose position the estimate tracks.
  [Ch. 2]
- **tracking wheel** — a small unpowered wheel with an encoder, pressed to the floor purely to
  measure travel (it can't slip from motor torque, having none). [Ch. 3]
- **trajectory** — an ordered list of waypoints driven as chained moves. [Ch. 10]
- **triage** — the compact "what went wrong" block written first when a fault fires (and printed
  at run end), before the surrounding ticks — first because the fault may be the brownout that
  cuts the write short. [Ch. 11]
- **typed units** — lengths, angles, velocities, and times as distinct compile-time types
  (`24_in`, `90_deg`), so unit mix-ups can't compile. [Ch. 10]
- **V5 brain** — the VEX controller-computer that runs the robot's code. [Ch. 1]
- **VEX U** — the university division of the VEX Robotics Competition; two robots per team.
  [Ch. 1]
- **watchdog** — the per-motion timer guaranteeing no motion can run (or block) forever.
  [Ch. 5]
- **waypoint** — one target pose in a trajectory. [Ch. 10]
- **X-drive** — four omniwheels at 45°; fully holonomic, strafes as fast as it drives. [Ch. 4]
