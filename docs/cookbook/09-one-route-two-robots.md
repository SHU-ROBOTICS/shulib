# 9 — One route, two robots

In a VEX U skills run both robots run at once, from the two red Quadrants — the red wall's and the right
wall's — and those two Quadrants are mirror images of each other across the field's diagonal
([guide chapter 2](../guide/02-the-field-and-coordinates.md#the-fields-mirror-two-robots-one-route)). So a
team whose two robots are the same build writes its skills route **once**, and the second robot runs it
mirrored. This chapter is that recipe.

Listings are compiled and run in
[`test/cookbook_examples_test.cpp`](../../test/cookbook_examples_test.cpp), case `cookbook-09a` — both
robots' registrations, the two robots run TOGETHER on the host test plant, and the wall reset on a robot with
side sensors. The ROUTE in it is a placeholder: every number is the test's. Your skills route is yours to
write and defend; the recipe is the shape around it.

---

## Write once, run mirrored

**Use this when:** robot two is robot one's build and should run robot one's skills route, mirrored, at the
same time.

The route, written ONCE, in robot one's numbers — robot one starts in the red wall's Quadrant:

```cpp
// THE SHARED SKILLS ROUTE, written ONCE, in robot one's numbers: robot one starts
// in the red wall's Quadrant. Robot two runs this same function MIRRORED.
chassis::RoutineResult skills(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};
    // Robot one: the red wall, RIGHT sensor. Robot two: the right wall, LEFT sensor.
    r.then([&] { return competition::resetFromWall(ctx,
                     {.wall = localization::override2026::kRedWall,
                      .sensor = localization::WallSide::Right,
                      .maxCorrection = 3_in}).continues(); }, "red wall y");
    r.driveTo(-12_in, -55.7_in);
    r.face(-12_in, -30_in);
    r.driveTo(-12_in, -30_in);
    // The centre goal sits ON the mirror line: ONE robot goes there.
    if (!competition::isMirrored(ctx)) {
        r.face(-6_in, -6_in);
        r.driveTo(-6_in, -6_in);
        r.reverseTo(-12_in, -30_in);
    } else {
        r.pause(4_s);  // robot two waits on its own side meanwhile
    }
    r.face(-30_in, -40_in);
    r.driveTo(-30_in, -40_in);
    return r.result();
}
```

the entry both robots register — robot one's start, the schedule:

```cpp
// The entry both robots register: robot one's start, its schedule.
competition::AutonEntry skillsRoute() {
    competition::AutonEntry e{};
    e.name = "skills";
    e.kind = competition::AutonKind::Skills;
    e.routine = &skills;
    e.start = competition::StartPose::field(-36_in, -55.7_in, 0_deg);
    e.endActionAt = 55_s;
    e.hardStopAt = 58_s;
    return e;
}
```

and the two registrations, one in each robot's autons file:

```cpp
// Robot one's autons file:
void registerRobotOneSkills(competition::AutonRegistry& registry) {
    registry.add(skillsRoute());
}

// Robot two's autons file -- the one line:
void registerRobotTwoSkills(competition::AutonRegistry& registry) {
    registry.add(competition::mirrored(skillsRoute(), competition::override2026::kSkillsMirror));
}
```

The compiled test registers both, runs the two robots together on the host test plant (they stay at least
20 in apart: the branch sends only robot one to the centre), and runs the route on a robot with side sensors
from each robot's start: robot one's wall reset seats the red wall with its RIGHT sensor, and robot two's —
the SAME line — seats the right wall with its LEFT sensor, its log saying
`right wall x SEATED (LEFT sensor, mirrored from red wall RIGHT)`.

**Why it is written this way.**

- **One function, two registrations.** The route lives once; robot two's file says only "mirrored". An edit
  to the route reaches both robots — there is no second copy to forget. (Where the two files live is your
  project's layout; the route function sits in a file of its OWN that both robots' builds compile — not
  inside either robot's autons file, so each robot's program carries only its own registrations.)
- **A FIELD start.** The mirror is the FIELD's (Override's skills diagonal), so the route's start must be a
  field pose: robot two is seated at its reflection, (55.7, 36) facing -90° — the result screen's
  `START MIR` line says where to place it. A RELATIVE start is mirrored in the robot's own frame instead
  (every left a right), and is right only if the drivers place robot two at the mirror image of robot one's
  spot.
- **The wall reset and the Toggle step swap for themselves.** Robot one's `red wall, RIGHT sensor` is robot
  two's `right wall, LEFT sensor` — the library makes the swap; write robot one's. A Toggle step (handed the
  auton's entry) visits the image Toggle — robot one's red wall Toggle is robot two's right wall Toggle, the
  same Quadrant colour, so "ours" does not change — `competition::checkAndFlipToggle(ctx, ...)` from a team's
  auton file included ([chapter 8](08-skills-toggles.md)).
- **`isMirrored(ctx)` for what lies ON the diagonal.** The centre goal, two yellow Pins of the skills setup
  and the red/right corner's Load Zone are their own mirror images: a pure mirror sends both robots there at
  the same moment. One branch sends one robot.

**Watch out for:**

- **Stay 3 in off the diagonal.** Every point of robot one's route keeps its WHOLE robot at least 3 in from
  the mirror line — then the two robots are at least 6 in apart whatever their timing. The suite checks it:
  a route that brings the robots within 6 in fails, naming the moment and the motion
  ([guide chapter 17](../guide/17-the-competition-program.md#both-robots-at-once)).
- **The branch changes the timing.** After an `isMirrored` branch the two robots are no longer in step: give
  the waiting robot a margin (here robot two waits 4 s, longer than robot one needs), because a real robot's
  timing drifts.
- **Loaders and the red/left Load Zone do not mirror** (robot two's spot has no Loader), and **the robot is
  not mirrored**: a part off its centre line — a side claw, an off-centre camera — lands on the other side of
  the reflected path. Check those steps on the bench.
- **A mirrored name is at most 25 characters** — its menu button reads `skills SKILLS MIR`.
- **An exact 180° turn** sweeps counterclockwise on both robots, not the mirror way (the log says so once) —
  in a `turnTo`, and in a `moveTo` / `driveTo` whose heading changes by 180°. Where it matters, write a turn
  as two 90° turns; on a tank, turn first (two 90° turns) before such a drive — the sweep moves where a tank
  ENDS, by more the longer the drive: 14 in on a 36 in drive on the host test plant
  ([guide chapter 17](../guide/17-the-competition-program.md#two-robots-one-route-a-mirrored-auton)).
- **Run the two robots together before the event** — slowly first, someone at each robot's stop. Nothing
  here has run on a robot.
