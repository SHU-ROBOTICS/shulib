// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
// Made in the route maker from src/team/route_maker.json. Change the route there and export again:
// the route maker will not overwrite a hand edit between BEGIN and END.
// To make this hand-written code instead, delete the BEGIN and END lines.
namespace {
// The route "skills", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (the list at the bottom says so).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};

    // Step 1 — SET the hood to "up" (it takes the hood's move time).
    r.set(hood["up"]);

    // Step 2 — LEAVE GENTLY: WAIT 0.5 s, then BACK UP 4 in, straight back, to (-40, -55.7) in, at most 5
    //   in/s. (The brain's log calls the wait step 2 and the back-up step 3.)
    r.then([&] { return ctx.guard.pause(0.5_s) == sequence::GuardedWaitResult::Satisfied; },
           "wait 0.5 s");
    r.reverseTo(-40_in, -55.7_in, {.maxLinearSpeed = 5_in / 1_s});

    // Step 3 — "out": LEAVE GENTLY: WAIT 1 s, then BACK UP to (-44, -50) in: about 7 in, its back toward
    //   the point. First it turns its back to the point (at the robot's own turn settings); the drive: at
    //   most 8 in/s. If it fails, carry on with the next step. (The brain's log calls the wait step 4, the
    //   turn step 5 and the back-up step 6.)
    r.label("out");
    r.onFail(chassis::carryOn);
    r.then([&] { return ctx.guard.pause(1_s) == sequence::GuardedWaitResult::Satisfied; },
           "wait 1 s");
    r.onFail(chassis::carryOn);
    r.faceAway(-44_in, -50_in);
    r.onFail(chassis::carryOn);
    r.reverseTo(-44_in, -50_in, {.maxLinearSpeed = 8_in / 1_s});

    // Step 4 — TURN to face 90 deg (toward blue). (The brain's log calls this step 7.)
    r.turnTo(90_deg);

    return r.result();
}

// Its entry: robot one's start, the 60 s skills schedule.
competition::AutonEntry skillsRoute() {
    competition::AutonEntry e{};
    e.name = "skills";                                // what the brain's menu shows
    e.kind = competition::AutonKind::Skills;          // the 60 s skills run
    e.routine = &skillsSteps;                         // the steps above
    e.start = competition::StartPose::field(-36_in, -55.7_in, 0_deg);  // robot one's start, facing 0 deg (toward the right wall)
    e.endActionAt = 57_s;  // still running at 57 s? it is stopped, and the end action (a brake) starts
    e.hardStopAt = 59_s;   // everything stops, 1 s before the run's 60 s
    return e;
}

// The route "gently slow", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (the list at the bottom says so).
chassis::RoutineResult gentlySlowSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "gently slow"};

    // Step 1 — SET the hood to "up" (it takes the hood's move time).
    r.set(hood["up"]);

    // Step 2 — LEAVE GENTLY: WAIT 0.5 s, then BACK UP 4 in, straight back, to (-40, -55.7) in, at most 5
    //   in/s. (The brain's log calls the wait step 2 and the back-up step 3.)
    r.then([&] { return ctx.guard.pause(0.5_s) == sequence::GuardedWaitResult::Satisfied; },
           "wait 0.5 s");
    r.reverseTo(-40_in, -55.7_in, {.maxLinearSpeed = 5_in / 1_s});

    return r.result();
}

// Its entry: robot one's start, the 60 s skills schedule.
competition::AutonEntry gentlySlowRoute() {
    competition::AutonEntry e{};
    e.name = "gently slow";                           // what the brain's menu shows
    e.kind = competition::AutonKind::Skills;          // the 60 s skills run
    e.routine = &gentlySlowSteps;                     // the steps above
    e.start = competition::StartPose::field(-36_in, -55.7_in, 0_deg);  // robot one's start, facing 0 deg (toward the right wall)
    e.endActionAt = 57_s;  // still running at 57 s? it is stopped, and the end action (a brake) starts
    e.hardStopAt = 59_s;   // everything stops, 1 s before the run's 60 s
    return e;
}
}  // namespace

// EVERY ROUTE, in the order the brain's menu lists them (addTeamRoutes, below the END line, registers them).
// Robot one registers each as written; robot two MIRRORS one marked Mirror::ForRobotTwo. Together::Yes: both
// robots run it at the SAME time, so the host suite checks they never come too close.
const TeamRoute kTeamRoutes[] = {
    {&skillsRoute, Mirror::ForRobotTwo, Together::No},      // "skills"
    {&gentlySlowRoute, Mirror::ForRobotTwo, Together::No},  // "gently slow"
};
// ═══ ROUTE MAKER — route "skills" — END ═════════════════════════════════════════════════
