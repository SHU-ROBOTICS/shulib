// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
// Made in the route maker from src/team/route_maker.json. Change the route there and export again:
// the route maker will not overwrite a hand edit between BEGIN and END.
// To make this hand-written code instead, delete the BEGIN and END lines.
namespace {
// The route "skills", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (the list at the bottom says so).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};

    // Step 1 — HOME the lift: it finds its zero the way its declaration says, at most 3 s. Nothing can move
    //   it to a level before this.
    r.home(lift, 3_s);

    // Step 2 — MOVE the lift to "low" and wait until it is there: at most 2 s.
    r.move(lift["low"], 2_s);

    // Step 3 — START the arm toward "score" and keep going (at most 2 s; a WAIT UNTIL DONE collects it).
    r.startMove(arm["score"], 2_s);

    // Step 4 — WAIT until the arm's last step is done: at most 2 s more.
    r.waitUntilDone(arm, 2_s);

    // Step 5 — SET the hood to "up" (it takes the hood's move time).
    r.set(hood["up"]);

    // Step 6 — RUN the intake at 12 V for 0.5 s, then stop it.
    r.run(intake, 12_volt, 0.5_s);

    // Step 7 — START the intake at -6 V for 0.3 s and keep going.
    r.startRun(intake, -6_volt, 0.3_s);

    // Step 8 — WAIT until the intake's last step is done: at most 1 s more.
    r.waitUntilDone(intake, 1_s);

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
