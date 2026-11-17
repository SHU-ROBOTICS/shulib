# 8 — Skills routes

In a SKILLS run a Toggle helps only when it shows **its own Quadrant's colour** — a red Quadrant's Toggle wants
RED, a blue Quadrant's wants BLUE (the Game Manual's `<RSC3>`). One library call does a whole visit as a route
step — look with the front camera, nothing more if it already shows that colour, otherwise drive in, push,
retract, drive out and look again ([guide chapter 20](../guide/20-the-camera.md#checking-and-flipping-a-toggle-in-a-skills-route)
teaches the step). This chapter is the recipe: a whole skills route that visits the Toggles. Every
number below is a PLACEHOLDER for the ones your team measures at Step 0 (the
measuring done at the robot first, numbered 0.1–0.7 in
[guide chapter 20's list](../guide/20-the-camera.md#step-0-the-numbers-the-team-measures)), and the route
is a teaching example, not your team's: which Toggles to visit, in what order, from where, and what a failed
visit does are yours to write and defend.

The listings are compiled and run in [`test/cookbook_examples_test.cpp`](../../test/cookbook_examples_test.cpp),
case `cookbook-08a` (and its `FLIPPER STUCK` variant), on a real `robot::Robot` built from a robot file with a
camera, a flipper (a motor arm) and a two-motor cascade, through `competition::runAuton` — the competition
program's own call — on the host test plant, with three host Toggles: the right wall's already ours, the blue
wall's yellow, the red wall's yellow and sticky.

---

## A skills route that visits the Toggles

**Use this when:** your skills route should flip each Toggle to its Quadrant's colour — checking with the
camera first, so a Toggle that is already right is never pushed away.

![The cookbook route on the real field: the start at (24, 0) facing 0 degrees; look spot 1 in front of the right wall's Toggle (a red Quadrant's — already red, nothing pushed), its push spot 4 in from the face; a straight back-off to (40, 0) before the turn; look spot 2 in front of the blue wall's Toggle (yellow to blue in two pushes); a back-off to (0, 40); look spot 3 in front of the red wall's Toggle (sticky — no change, the route skips to backing off); the robot backs off to (0, -30). Each Toggle is outlined in its Quadrant's colour — a reading of the manual's drawings (HA-243), checked on the event's field at Step 0.7](../assets/skills-toggle-look-spots.png)

**First, the robot file.** The step reads the robot's camera through the auton's context. It needs the robot
file's **camera block** (its port, the colours taught on the Toggle's real faces, and its MEASURED mount) and,
beside it, the **Toggle block**: the tracker's numbers and how often to read the camera. Neither of the team's
robot files has a camera block yet — writing one is part of wiring the camera; until then every visit is NO
CAMERA, naming why. Each robot that runs the skills route needs its own pair of blocks, measured on that robot:

```cpp
inline constexpr vision::ToggleCameraConfig kToggleBlock{
    .tracker = {.ids = {1, 2, 3},             // Step 0.3: the ids the camera block teaches: RED, BLUE, YELLOW
                .holdSeconds = 0.15,          // Step 0.3: how long a colour must hold
                .aliveSeconds = 0.10,         // Step 0.3: how old the newest read may be
                .minAspect = 3.0,             // Step 0.3: a band's width over its height, at least
                .minLengthFraction = 0.6,     // Step 0.3: a band's length, at least this x 25.8 in
                .maxLengthFraction = 1.3,     // Step 0.3: ... and at most
                .minRangeIn = 2.0,            // Step 0.3: the range window, inches
                .maxRangeIn = 96.0,           // Step 0.3
                .bearingWindowDeg = 25.0,     // Step 0.3: the bearing window, degrees
                .frozenMoveIn = 2.0,          // Step 0.3: "camera frozen?" after the lens moves this far
                .frozenTurnDeg = 2.0},        // Step 0.3: ... or the robot turns this far
    .feed = {.minReadPeriodSeconds = 0.033}};  // Step 0.3: read the camera at most this often, seconds
```

It goes in the robot file as `.sensors = {..., .toggleCamera = kToggleBlock}`. If the step cannot use it,
the robot still boots and drives, and the reason is in two places: `robot.toggleCameraAbsence()`, and the
`[TGL]` line of every visit (`no camera -- <the reason>`). The robot's boot line (`Robot::describe()`) also
ends `, toggles ready` or `, toggles OFF (<the reason>)` — but the brain's programs print that line cut to
220 characters, which a long reason passes, and the competition program prints none.

**The numbers every visit uses,** checked when the program is BUILT:

```cpp
consteval competition::ToggleVisitRequest visitAt(vision::override2026::ToggleId which) {
    return competition::toggleVisit({.toggle = which,
                                     .flipper = flipper["flip"],
                                     .back = flipper["stow"],
                                     .lookFrom = 16_in,      // Step 0.4: lens to the face, at the look spot
                                     .offBy = 6_in,          // Step 0.4: how far off the look spot is accepted
                                     .pushFrom = 4_in,       // Step 0.5: lens to the face, at the push spot
                                     .lookWithin = 0.6_s,    // Step 0.3: the longest a look waits for a colour
                                     .flipBound = 1.2_s,     // Step 0.1: the worst push of five
                                     .backBound = 1.2_s,     // Step 0.1: the worst retract of five
                                     .settle = 0.3_s,        // Step 0.1: until the Toggle sits flat
                                     .driveBound = 1.5_s,    // Step 0.6: the worst drive in (or out) of ten
                                     .margin = 0.3_s,        // your spare time per visit (a choice, not measured)
                                     .maxImageAge = 0.1_s});  // Step 0.2: the camera's picture's age
}
```

The look spot and the push spot are checked against the Toggle block's `frozenMoveIn` too — two files, typed at
different stations — so a change to either that brings the spots too close stops the build:

```cpp
static_assert(competition::toggleVisitBlockProblem(visitAt(toggles::kRightWall), kToggleBlock) == nullptr);
```

**The route** — `flipper` and `cascade` are the robot file's mechanism declarations (made nameable with a
`using namespace` line for your robot's region, as robot one's autons file does), and `toggles` is
`vision::override2026::toggles`:

```cpp
chassis::RoutineResult skillsToggles(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills toggles"};
    r.startHome(cascade, 4_s);                  // home the cascade while the first leg drives...
    r.driveTo(47.2_in, 0_in, {.timeout = 3_s});  // the right wall's look spot: lens 16 in from the face
    r.turnTo(0_deg, {.timeout = 2_s});           // square to the wall
    r.waitUntilDone(cascade, 4_s);              // ...and wait for the home to FINISH: a move sent while it
    r.move(cascade["travel"], 3_s);             // is still homing replaces it, and stops NOT_HOMED

    r.onFail(skipTo("blue wall")).then([&] {
        return competition::checkAndFlipToggle(ctx, visitAt(toggles::kRightWall)).outcome();
    }, "right wall Toggle");

    r.label("blue wall").move(flipper["stow"], 2_s);  // stowed again, then STRAIGHT back off the wall
    r.reverseTo(40_in, 0_in, {.timeout = 3_s});        // before any turn: after a FLIPPER STUCK the robot
    r.face(0_in, 47.2_in, {.timeout = 3_s});           // is still at the push spot, too near to turn there
    r.driveTo(0_in, 47.2_in, {.timeout = 6_s});
    r.turnTo(90_deg, {.timeout = 3_s});
    r.onFail(skipTo("red wall")).then([&] {
        return competition::checkAndFlipToggle(ctx, visitAt(toggles::kBlueWall)).outcome();
    }, "blue wall Toggle");

    r.label("red wall").move(flipper["stow"], 2_s);
    r.reverseTo(0_in, 40_in, {.timeout = 3_s});
    r.face(0_in, -47.2_in, {.timeout = 3_s});
    r.driveTo(0_in, -47.2_in, {.timeout = 8_s});
    r.turnTo(-90_deg, {.timeout = 3_s});
    r.onFail(skipTo("back off")).then([&] {
        return competition::checkAndFlipToggle(ctx, visitAt(toggles::kRedWall)).outcome();
    }, "red wall Toggle");

    r.label("back off").move(flipper["stow"], 2_s);
    r.reverseTo(0_in, -30_in, {.timeout = 3_s});  // off the wall: no robot touches a Toggle at the end
    return r.result();
}
```

**And its entry** — a FIELD start:

```cpp
competition::AutonEntry skillsTogglesEntry() {
    competition::AutonEntry e{};
    e.name = "skills toggles";
    e.kind = competition::AutonKind::Skills;
    e.routine = &skillsToggles;
    e.start = competition::StartPose::field(24_in, 0_in, 0_deg);
    e.endActionAt = 55_s;
    e.hardStopAt = 58_s;
    return e;
}
```

On the host test plant this run takes about 31 s and leaves these three lines (and the flipper's own `MECH`
lines beside them):

```text
right wall Toggle: already RED (ours), nothing pushed, 0.2s, seating not checked
blue wall Toggle: YELLOW -> BLUE after 2 pushes, 9.5s (ours BLUE), seating not checked
red wall Toggle: NO CHANGE -- still YELLOW after 1 push (sticky, or a camera frozen since before)
```

— the right wall's Toggle untouched, the blue wall's flipped (the host Toggle scores it BLUE), the sticky one
still yellow, the route skipping on to `back off`, and the robot backed off the wall at the end.

**Why it is written this way.**

- **The cascade's home FINISHES before its first move.** `startHome` homes it while the first leg drives (a
  stall home is slow — it creeps down until it stalls on its stop), and `waitUntilDone` waits for that home to
  END before anything moves it. A move sent while it is still homing REPLACES the home, and that move then
  stops NOT_HOMED — on the host test plant the cascade, left 3 in up at boot, is still coming down when the
  first leg ends. `r.home(cascade, 4_s)` is the same thing in one line, without the overlap.
- **Each visit's failure skips to the next leg, never the whole route.** A Toggle the camera cannot read, one
  that is sticky, a robot not at the look spot — each fails its step; `onFail(skipTo(...))` counts it and
  carries on from the next label. The run's end still stops the route at once: a visit cut by the deadline
  is CANCELLED, which no on-failure choice goes on from.
- **Each leg starts by stowing the flipper, then backing STRAIGHT off before it turns.** After a good visit
  the flipper is already stowed (a fraction of a second) and the back-off is a few inches. After FLIPPER
  STUCK — the retract did not reach `stow`, and the step left the robot at the PUSH spot so as not to drag
  the Toggle — the stow is the recovery's second, longer try (if that fails too, the route stops there),
  and the back-off takes the robot out to where a turn cannot sweep its frame into the wall or the Toggle.
  A back-off point is BEHIND the robot on its line, a few inches past the look spot, so `reverseTo` drives
  it straight back. The case `cookbook-08a (FLIPPER STUCK)` runs this recovery on the host test plant: the
  flipper stowed before the robot moves, the robot more than 10 in back before it turns.
- **Every look spot is reached SQUARE to its wall.** The camera names a Toggle's colour only from about square
  (on the host camera model, turned 8 degrees at 16 in it reads nothing), so every approach ends with `turnTo`
  to the wall's heading.
- **A FIELD start.** With one, every visit also checks the robot faces the named Toggle's wall — the one
  check that catches a route naming the wrong Toggle. Under a RELATIVE start the step works, and its line says
  `wall not checked (relative start)` (and drops a pose bound, below).
- **Always return `.outcome()`.** `then()` judges the step by what the lambda returns; one that returns nothing
  "succeeds" whatever the visit did.
- **"Ours" is never typed.** `visitAt(toggles::kBlueWall)` names WHICH Toggle; its Quadrant's colour comes from
  the season's table — a reading of the manual's drawings (HA-243), checked on the event's field at Step 0.7.
  A typo cannot make the robot flip a Toggle to the wrong colour — naming the wrong Toggle is the visible
  mistake, and the FIELD start's wall check catches it.

**Watch out for:**

- **A Toggle block the camera cannot use** — the camera's mount not measured, a tracker number left at 0, a
  colour id the camera was never taught: every visit fails NO CAMERA with the reason in its `[TGL]` line
  (`robot.toggleCameraAbsence()` says it too), and the robot still drives. Nothing is pushed on a guess.
- **Spots too close.** A visit refuses — `spots too close`, nothing read or moved, the route's `onFail`
  applies — when `lookFrom - pushFrom` is twice the Toggle block's `frozenMoveIn` or less (the look after a push
  must move the camera past the frozen check's distance). The `static_assert` above catches it at build.
- **FLIPPED is not "set".** The camera cannot see whether a Toggle has settled flat: one stuck part-way reads
  as the colour it was rolling to. FLIPPED means "it shows our colour after a full stroke", and every success
  line says `seating not checked` ([chapter 14](../guide/14-what-it-cannot-do-yet.md#the-toggle-flip-in-a-skills-route-what-it-cannot-catch)).
- **Holding pieces.** A robot holding more than one Pin or one Cup may not touch a Toggle (`<SG6>`); the library
  cannot see what the robot holds, so the route's order is what respects it.
- **The time.** A visit begins a push only when the run's time left covers every push it may need —
  `competition::worstCaseSeconds(visitAt(...))`, 13.1 s for these numbers. Late in a run a visit says
  `not now` and pushes nothing; budget the visits early.
- **The robot's pose.** Under a FIELD start, right after a wall reset, pass the estimate's bound as a third
  argument — `checkAndFlipToggle(ctx, visitAt(...), {.poseTolerance = 2_in, .headingToleranceDeg = 3})` — and,
  when the Toggle block turns the camera's position check on, it turns MOST lookalikes away — not one within
  its tolerance of where a Toggle's face is ([chapter 14](../guide/14-what-it-cannot-do-yet.md#the-toggle-flip-in-a-skills-route-what-it-cannot-catch)).
  Without one, that check is skipped; under a RELATIVE start the bound is dropped (the estimate is not in the
  field's frame) and the line says `wall and position not checked (relative start)`.
- **The plan for the first competition** is the team's own blind two-push flip, written with the mechanism
  steps they already have; this checked step replaces it only once it has flipped a real Toggle.
