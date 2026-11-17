// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
// Made in the route maker from src/team/route_maker.json. Change the route there and export again:
// the route maker will not overwrite a hand edit between BEGIN and END.
// To make this hand-written code instead, delete the BEGIN and END lines.
namespace {
// The skills route, in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (robot two's autons file says so in one line).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};

    // Step 1 — DRIVE to (0, -55.7) in: about 36 in, at most 30 in/s, turning at most 270 deg/s, arriving
    //   within 2 in.
    r.driveTo(0_in, -55.7_in, {.maxLinearSpeed = 30_in / 1_s, .maxAngularSpeed = units::AngularVelocity{270 * math::Angle::kPi / 180.0}, .positionTolerance = 2_in});  // at most 270 deg/s

    // Step 2 — TURN to face 90 deg (toward blue), turning at most 90 deg/s, within 5 deg.
    r.turnTo(90_deg, {.maxAngularSpeed = units::AngularVelocity{90 * math::Angle::kPi / 180.0}, .headingTolerance = 5_deg});  // at most 90 deg/s

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
