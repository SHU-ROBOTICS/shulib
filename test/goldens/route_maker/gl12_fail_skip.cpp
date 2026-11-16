// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
// Made in the route maker from src/team/route_maker.json. Change the route there and export again:
// the route maker will not overwrite a hand edit between BEGIN and END.
// To make this hand-written code instead, delete the BEGIN and END lines.
namespace {
// The skills route, in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (robot two's autons file says so in one line).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};

    // Step 1 — DRIVE to (0, -55.7) in: about 36 in, at most 0.3 s. If it fails, skip to "after".
    r.onFail(chassis::skipTo("after"));
    r.driveTo(0_in, -55.7_in, {.timeout = 0.3_s});

    // Step 2 — TURN to face 90 deg (toward blue).
    r.turnTo(90_deg);

    // Step 3 — "after": WAIT 0.2 s. (The run's own wait: at 57 s the run cuts it at once.)
    r.label("after");
    r.then([&] { return ctx.guard.pause(0.2_s) == sequence::GuardedWaitResult::Satisfied; },
           "wait 0.2 s");

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
