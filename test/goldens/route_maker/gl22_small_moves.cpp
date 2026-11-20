// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
// Made in the route maker from src/team/route_maker.json. Change the route there and export again:
// the route maker will not overwrite a hand edit between BEGIN and END.
// To make this hand-written code instead, delete the BEGIN and END lines.
namespace {
// The route "skills", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (the list at the bottom says so).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};

    // Step 1 — DRIVE FORWARD 1 in, straight on, to (-35, -55.7) in.
    r.driveTo(-35_in, -55.7_in);

    // Step 2 — TURN to face 90 deg (toward blue).
    r.turnTo(90_deg);

    // Step 3 — DRIVE FORWARD 2.5 in, straight on, to (-35, -53.2) in, at most 20 in/s.
    r.driveTo(-35_in, -53.2_in, {.maxLinearSpeed = 20_in / 1_s});

    // Step 4 — BACK UP 1 in, straight back, to (-35, -54.2) in.
    r.reverseTo(-35_in, -54.2_in);

    // Step 5 — CURVE through (-35, -40) in, ending at (-20, -28) in facing 0 deg (toward the right wall).
    r.curveTo({{-35_in, -40_in}, {-20_in, -28_in, 0_deg}});

    // Step 6 — DRIVE FORWARD 1.25 in, straight on, to (-18.75, -28) in.
    r.driveTo(-18.75_in, -28_in);

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
