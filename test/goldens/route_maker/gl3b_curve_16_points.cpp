// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
// Made in the route maker from src/team/route_maker.json. Change the route there and export again:
// the route maker will not overwrite a hand edit between BEGIN and END.
// To make this hand-written code instead, delete the BEGIN and END lines.
namespace {
// The skills route, in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (robot two's autons file says so in one line).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};

    // Step 1 — CURVE through (-26, -50), (-20.5, -49.258), (-15, -48.562), (-9.5, -47.955), (-4, -47.476),
    //   (1.5, -47.153), (7, -47.008), (12.5, -47.048), (18, -47.272), (23.5, -47.666), (29, -48.205),
    //   (34.5, -48.855), (40, -49.577), (45.5, -50.325), (51, -51.052) in, ending at (56.5, -51.715) in
    //   facing 0 deg (toward the right wall).
    r.curveTo({{-26_in, -50_in}, {-20.5_in, -49.258_in}, {-15_in, -48.562_in}, {-9.5_in, -47.955_in}, {-4_in, -47.476_in}, {1.5_in, -47.153_in}, {7_in, -47.008_in}, {12.5_in, -47.048_in}, {18_in, -47.272_in}, {23.5_in, -47.666_in}, {29_in, -48.205_in}, {34.5_in, -48.855_in}, {40_in, -49.577_in}, {45.5_in, -50.325_in}, {51_in, -51.052_in}, {56.5_in, -51.715_in, 0_deg}});

    return r.result();
}
}  // namespace

// The entry both robots register: robot one's start, the 60 s skills schedule.
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
// ═══ ROUTE MAKER — route "skills" — END ═════════════════════════════════════════════════
