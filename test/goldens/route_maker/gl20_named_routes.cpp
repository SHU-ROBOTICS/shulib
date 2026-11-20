// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
// Made in the route maker from src/team/route_maker.json. Change the route there and export again:
// the route maker will not overwrite a hand edit between BEGIN and END.
// To make this hand-written code instead, delete the BEGIN and END lines.
namespace {
// The route "skills", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (the list at the bottom says so).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};

    // Step 1 — DRIVE FORWARD 12 in, straight on, to (-24, -55.7) in.
    r.driveTo(-24_in, -55.7_in);

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

// The route "practice 1", in ROBOT ONE's numbers. Robot two runs it as written
// (the list at the bottom says so).
chassis::RoutineResult practice1Steps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "practice 1"};

    // Step 1 — DRIVE FORWARD 6 in, straight on, to (24, -42) in.
    r.driveTo(24_in, -42_in);

    // Step 2 — BACK UP 3 in, straight back, to (24, -45) in.
    r.reverseTo(24_in, -45_in);

    return r.result();
}

// Its entry: robot one's start, the 60 s skills schedule.
competition::AutonEntry practice1Route() {
    competition::AutonEntry e{};
    e.name = "practice 1";                            // what the brain's menu shows
    e.kind = competition::AutonKind::Skills;          // the 60 s skills run
    e.routine = &practice1Steps;                      // the steps above
    e.start = competition::StartPose::field(24_in, -48_in, 90_deg);  // robot one's start, facing 90 deg (toward blue)
    e.endActionAt = 57_s;  // still running at 57 s? it is stopped, and the end action (a brake) starts
    e.hardStopAt = 59_s;   // everything stops, 1 s before the run's 60 s
    return e;
}

// The route "40in test", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (the list at the bottom says so).
chassis::RoutineResult route40inTestSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "40in test"};
    // NO STEPS YET: the team's route is made in the route maker and written here.
    return r.result();
}

// Its entry.
competition::AutonEntry route40inTestRoute() {
    competition::AutonEntry e{};
    e.name = "40in test";                             // what the brain's menu shows
    e.kind = competition::AutonKind::Skills;          // the 60 s skills run
    e.routine = &route40inTestSteps;                  // the steps above
    e.start = competition::StartPose::relative();     // NO START YET: the route maker writes robot one's start
    e.endActionAt = 57_s;  // still running at 57 s? it is stopped, and the end action (a brake) starts
    e.hardStopAt = 59_s;   // everything stops, 1 s before the run's 60 s
    return e;
}
}  // namespace

// EVERY ROUTE, in the order the brain's menu lists them (addTeamRoutes, below the END line, registers them).
// Robot one registers each as written; robot two MIRRORS one marked Mirror::ForRobotTwo. Together::Yes: both
// robots run it at the SAME time, so the host suite checks they never come too close.
const TeamRoute kTeamRoutes[] = {
    {&skillsRoute, Mirror::ForRobotTwo, Together::Yes},        // "skills"
    {&practice1Route, Mirror::None, Together::No},             // "practice 1"
    {&route40inTestRoute, Mirror::ForRobotTwo, Together::No},  // "40in test"
};
// ═══ ROUTE MAKER — route "skills" — END ═════════════════════════════════════════════════
