// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
// Made in the route maker from src/team/route_maker.json. Change the route there and export again:
// the route maker will not overwrite a hand edit between BEGIN and END.
// To make this hand-written code instead, delete the BEGIN and END lines.
namespace {
// The route "skills", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (the list at the bottom says so).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};

    // Step 1 — HOME the cascade: it finds its zero the way its declaration says, at most 3 s. Nothing can
    //   move it to a level before this.
    r.home(cascade, 3_s);

    // Step 2 — SET the claw to "open" (it takes the claw's move time).
    r.set(claw["open"]);

    // Step 3 — DRIVE to (-12, -55.7) in: about 24 in.
    r.driveTo(-12_in, -55.7_in);

    // Step 4 — RUN the rightToggle at 12 V for 0.3 s, then stop it.
    r.run(rightToggle, 12_volt, 0.3_s);

    // Step 5 — SET the claw to "closed" (it takes the claw's move time).
    r.set(claw["closed"]);

    // Step 6 — CURVE through (0, -50) in, ending at (12, -36) in facing 90 deg (toward blue).
    r.curveTo({{0_in, -50_in}, {12_in, -36_in, 90_deg}});

    // Step 7 — "finish": WAIT 0.5 s. (The run's own wait: at 57 s the run cuts it at once.)
    r.label("finish");
    r.then([&] { return ctx.guard.pause(0.5_s) == sequence::GuardedWaitResult::Satisfied; },
           "wait 0.5 s");

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
}  // namespace

// EVERY ROUTE, in the order the brain's menu lists them (addTeamRoutes, below the END line, registers them).
// Robot one registers each as written; robot two MIRRORS one marked Mirror::ForRobotTwo. Together::Yes: both
// robots run it at the SAME time, so the host suite checks they never come too close.
const TeamRoute kTeamRoutes[] = {
    {&skillsRoute, Mirror::ForRobotTwo, Together::NotAnswered},  // "skills"
};
// ═══ ROUTE MAKER — route "skills" — END ═════════════════════════════════════════════════
