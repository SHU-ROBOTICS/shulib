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

    // Step 2 — MOVE the lift UP 6 in, to 6 in, and wait until it is there: at most 2 s.
    r.move(lift.to(6_in), 2_s);

    // Step 3 — MOVE the lift UP 10.5 in, to 16.5 in, and wait until it is there: at most 4 s.
    r.move(lift.to(16.5_in), 4_s);

    // Step 4 — MOVE the lift DOWN 4.5 in, to 12 in, and wait until it is there: at most 3 s.
    r.move(lift.to(12_in), 3_s);

    // Step 5 — MOVE the lift to 20 in, and wait until it is there: at most 3.5 s.
    r.move(lift.to(20_in), 3.5_s);

    // Step 6 — START the lift BACK TO THE BOTTOM (0 in) and keep going (at most 4 s; a WAIT UNTIL DONE
    //   collects it).
    r.startMove(lift.to(0_in), 4_s);

    // Step 7 — WAIT until the lift's last step is done: at most 4 s more.
    r.waitUntilDone(lift, 4_s);

    // Step 8 — MOVE the arm UP 30 deg, to 30 deg, and wait until it is there: at most 3 s.
    r.move(arm.to(30), 3_s);

    // Step 9 — MOVE the arm to 95.5 deg, and wait until it is there: at most 3 s.
    r.move(arm.to(95.5), 3_s);

    // Step 10 — MOVE the arm DOWN 15.5 deg, to 80 deg, and wait until it is there: at most 3 s.
    r.move(arm.to(80), 3_s);

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
    {&skillsRoute, Mirror::ForRobotTwo, Together::No},  // "skills"
};
// ═══ ROUTE MAKER — route "skills" — END ═════════════════════════════════════════════════
