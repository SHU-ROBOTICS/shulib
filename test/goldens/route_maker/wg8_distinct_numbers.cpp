// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
// Made in the route maker from src/team/route_maker.json. Change the route there and export again:
// the route maker will not overwrite a hand edit between BEGIN and END.
// To make this hand-written code instead, delete the BEGIN and END lines.
namespace {
// The route "skills", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (the list at the bottom says so).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};

    // Step 1 — HOME the lift: it finds its zero the way its declaration says, at most 3.25 s. Nothing can
    //   move it to a level before this.
    r.home(lift, 3.25_s);

    // Step 2 — "leg one": DRIVE to (-12.5, -50.25) in: about 24.3 in. First it turns to face the point (at
    //   the robot's own turn settings); the drive: at most 3.75 s, at most 30.5 in/s, turning at most
    //   200.25 deg/s, arriving within 1.5 in, within 2.25 deg. If it fails, carry on with the next step.
    //   (The brain's log calls the turn step 2 and the drive step 3.)
    r.label("leg one");
    r.onFail(chassis::carryOn);
    r.face(-12.5_in, -50.25_in);
    r.onFail(chassis::carryOn);
    r.driveTo(-12.5_in, -50.25_in, {.timeout = 3.75_s, .maxLinearSpeed = 30.5_in / 1_s, .maxAngularSpeed = units::AngularVelocity{200.25 * math::Angle::kPi / 180.0}, .positionTolerance = 1.5_in, .headingTolerance = 2.25_deg});  // at most 200.25 deg/s

    // Step 3 — BACK UP to (-30.75, -46.5) in: about 18.6 in, its back toward the point. First it turns its
    //   back to the point (at the robot's own turn settings); the drive: at most 2.5 s. If it fails, carry
    //   on with the next step. (The brain's log calls the turn step 4 and the drive step 5.)
    r.onFail(chassis::carryOn);
    r.faceAway(-30.75_in, -46.5_in);
    r.onFail(chassis::carryOn);
    r.reverseTo(-30.75_in, -46.5_in, {.timeout = 2.5_s});

    // Step 4 — TURN to face 45.5 deg, at most 1.75 s, turning at most 150.5 deg/s. If it fails, carry on
    //   with the next step. (The brain's log calls this step 6.)
    r.onFail(chassis::carryOn);
    r.turnTo(45.5_deg, {.timeout = 1.75_s, .maxAngularSpeed = units::AngularVelocity{150.5 * math::Angle::kPi / 180.0}});  // at most 150.5 deg/s

    // Step 5 — TURN to face (-10.25, -35.75) in, at most 1.25 s, within 3.5 deg. If it fails, carry on with
    //   the next step. (The brain's log calls this step 7.)
    r.onFail(chassis::carryOn);
    r.face(-10.25_in, -35.75_in, {.timeout = 1.25_s, .headingTolerance = 3.5_deg});

    // Step 6 — CURVE through (-20.5, -38.25) in, ending at (-5.75, -25.5) in facing 60.25 deg, at most 3.5
    //   s, at most 40.25 in/s. If it fails, carry on with the next step. (The brain's log calls this step
    //   8.)
    r.onFail(chassis::carryOn);
    r.curveTo({{-20.5_in, -38.25_in}, {-5.75_in, -25.5_in, 60.25_deg}}, {.timeout = 3.5_s, .maxLinearSpeed = 40.25_in / 1_s});

    // Step 7 — TURN its back to (-40.5, -20.25) in, at most 1.5 s. If it fails, carry on with the next
    //   step. (The brain's log calls this step 9.)
    r.onFail(chassis::carryOn);
    r.faceAway(-40.5_in, -20.25_in, {.timeout = 1.5_s});

    // Step 8 — CURVE BACKWARDS (its back leading) through (-15.25, -24.5) in, ending at (-30.5, -16.75) in
    //   facing -10.25 deg, at most 4.25 s. If it fails, carry on with the next step. (The brain's log calls
    //   this step 10.)
    r.onFail(chassis::carryOn);
    r.reverseCurveTo({{-15.25_in, -24.5_in}, {-30.5_in, -16.75_in, -10.25_deg}}, {.timeout = 4.25_s});

    // Step 9 — MOVE the lift to "low" and wait until it is there: at most 2.25 s. If it fails, carry on
    //   with the next step. (The brain's log calls this step 11.)
    r.onFail(chassis::carryOn);
    r.move(lift["low"], 2.25_s);

    // Step 10 — START the arm toward "score" and keep going (at most 1.75 s; a WAIT UNTIL DONE collects
    //   it). (The brain's log calls this step 12.)
    r.startMove(arm["score"], 1.75_s);

    // Step 11 — TURN to face -30.5 deg, at most 2.25 s. If it fails, carry on with the next step. (The
    //   brain's log calls this step 13.)
    r.onFail(chassis::carryOn);
    r.turnTo(-30.5_deg, {.timeout = 2.25_s});

    // Step 12 — WAIT until the arm's last step is done: at most 2.5 s more. If it fails, carry on with the
    //   next step. (The brain's log calls this step 14.)
    r.onFail(chassis::carryOn);
    r.waitUntilDone(arm, 2.5_s);

    // Step 13 — SET the hood to "middle" (it takes the hood's move time). If it fails, carry on with the
    //   next step. (The brain's log calls this step 15.)
    r.onFail(chassis::carryOn);
    r.set(hood["middle"]);

    // Step 14 — RUN the intake at 9.5 V for 0.75 s, then stop it. If it fails, carry on with the next step.
    //   (The brain's log calls this step 16.)
    r.onFail(chassis::carryOn);
    r.run(intake, 9.5_volt, 0.75_s);

    // Step 15 — START the intake at -7.25 V for 0.5 s and keep going. (The brain's log calls this step 17.)
    r.startRun(intake, -7.25_volt, 0.5_s);

    // Step 16 — TURN to face (-35.25, -20.75) in, at most 1.75 s. If it fails, carry on with the next step.
    //   (The brain's log calls this step 18.)
    r.onFail(chassis::carryOn);
    r.face(-35.25_in, -20.75_in, {.timeout = 1.75_s});

    // Step 17 — WAIT until the intake's last step is done: at most 1.25 s more. If it fails, carry on with
    //   the next step. (The brain's log calls this step 19.)
    r.onFail(chassis::carryOn);
    r.waitUntilDone(intake, 1.25_s);

    // Step 18 — START homing the lift and keep going (at most 2.75 s); a WAIT UNTIL DONE must finish the
    //   home before its first move. (The brain's log calls this step 20.)
    r.startHome(lift, 2.75_s);

    // Step 19 — TURN to face 15.25 deg, at most 2.5 s. If it fails, carry on with the next step. (The
    //   brain's log calls this step 21.)
    r.onFail(chassis::carryOn);
    r.turnTo(15.25_deg, {.timeout = 2.5_s});

    // Step 20 — WAIT until the lift's last step is done: at most 1.5 s more. If it fails, carry on with the
    //   next step. (The brain's log calls this step 22.)
    r.onFail(chassis::carryOn);
    r.waitUntilDone(lift, 1.5_s);

    // Step 21 — DRIVE to (-24.75, -40.5) in: about 24.4 in. First it turns to face the point (at the
    //   robot's own turn settings); the drive: at most 0.35 s. If it fails, skip to "end". (The brain's log
    //   calls the turn step 23 and the drive step 24.)
    r.onFail(chassis::skipTo("end"));
    r.face(-24.75_in, -40.5_in);
    r.onFail(chassis::skipTo("end"));
    r.driveTo(-24.75_in, -40.5_in, {.timeout = 0.35_s});

    // Step 22 — "end": WAIT 0.75 s. (The run's own wait: at 57 s the run cuts it at once.) (The brain's log
    //   calls this step 25.)
    r.label("end");
    r.then([&] { return ctx.guard.pause(0.75_s) == sequence::GuardedWaitResult::Satisfied; },
           "wait 0.75 s");

    return r.result();
}

// Its entry: robot one's start, the 60 s skills schedule.
competition::AutonEntry skillsRoute() {
    competition::AutonEntry e{};
    e.name = "skills";                                // what the brain's menu shows
    e.kind = competition::AutonKind::Skills;          // the 60 s skills run
    e.routine = &skillsSteps;                         // the steps above
    e.start = competition::StartPose::field(-36.25_in, -55.5_in, 10.5_deg);  // robot one's start, facing 10.5 deg
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
