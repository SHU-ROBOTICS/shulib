// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
// Made in the route maker from src/team/route_maker.json. Change the route there and export again:
// the route maker will not overwrite a hand edit between BEGIN and END.
// To make this hand-written code instead, delete the BEGIN and END lines.
namespace {
// The skills route, in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (robot two's autons file says so in one line).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};

    // Step 1 — DRIVE to (-12, -55.7) in: about 24 in.
    r.driveTo(-12_in, -55.7_in);

    // Step 2 — SET the claw to "open" (it takes the claw's move time).
    r.set(claw["open"]);

    // Step 3 — RUN the leftToggle at 6 V for 0.3 s, then stop it.
    r.run(leftToggle, 6_volt, 0.3_s);

    // Step 4 — CURVE through (0, -50) in, ending at (12, -36) in facing 90 deg (toward blue).
    r.curveTo({{0_in, -50_in}, {12_in, -36_in, 90_deg}});

    // Step 5 — WAIT 0.5 s. (The run's own wait: at 57 s the run cuts it at once.)
    r.then([&] { return ctx.guard.pause(0.5_s) == sequence::GuardedWaitResult::Satisfied; },
           "wait 0.5 s");

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
