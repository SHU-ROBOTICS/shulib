// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
// Made in the route maker from src/team/route_maker.json. Change the route there and export again:
// the route maker will not overwrite a hand edit between BEGIN and END.
// To make this hand-written code instead, delete the BEGIN and END lines.
namespace {
// The route "skills", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (the list at the bottom says so).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};

    // Step 1 — DRIVE to (-19.029, -38.729) in: about 24 in. First it turns to face the point (at the
    //   robot's own turn settings); the drive: at most 4 s. (The brain's log calls the turn step 1 and the
    //   drive step 2.)
    r.face(-19.029_in, -38.729_in);
    r.driveTo(-19.029_in, -38.729_in, {.timeout = 4_s});

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
