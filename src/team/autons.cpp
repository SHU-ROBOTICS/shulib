// autons.cpp -- THE TEAM'S AUTONS for robot two: every autonomous routine the robot can run,
// and registerAutons() at the bottom, the one list the programs read.
//
// WHO RUNS THESE. "shulib Auton Tester" runs the one you pick on the brain's screen when you touch
// RUN (on the bench, no competition switch needed). "shulib Competition" runs the one picked on
// its screen when the match's autonomous period starts. Both hand the SAME entry to the SAME
// library call (competition::runAuton), so the auton you test on the bench is the auton that runs
// in the match. Only what STARTS it differs.
//
// THE FRAME. x is straight ahead of the robot, y is to its left, and angles grow counterclockwise
// (0 degrees faces +x). "Ahead" is the robot's FRONT as the calibration record defines it: the end
// that LED the push when "shulib Calibrate" ran. A RELATIVE start makes the spot where the robot
// is placed (0, 0), facing 0 degrees, at the moment the auton starts.
//
// THE DISTANCES are the robot's OWN estimate (its drive encoders and its IMU). When the tape
// disagrees with an auton, the calibration record is what is wrong, not the number written here:
// re-run "shulib Calibrate", and never change an auton's number to make the tape agree.
//
// PROVENANCE. The 40 in auton was written for the team on the team lead's go of 2026-09-26 ("no
// you can write the auton"). Every line is the team's to explain, defend and change.

#include "team/autons.hpp"

#include "shulib/chassis/routine.hpp"
#include "shulib/competition/acceptance_run.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/literals.hpp"

namespace shulib::bench {

using namespace units::literals;  // 40_in, 0_deg, 27_s

namespace {

// ── THE 40 IN AUTON ─────────────────────────────────────────────────────────────────────
// Drive straight forward 40 inches and stop, facing the way the robot started.
// ONE moveTo: the library drives to the POSE -- 40 in ahead, 0 in to the side, facing 0 degrees --
// and the step ends when the robot is there and still, or when the motion's own time limit ends
// it. Nothing is tuned here: the motion uses the robot's configured defaults.
// A TANK CANNOT DRIVE SIDEWAYS. If one side of the drive is weaker than the other, it turns the
// robot toward that side and the robot drifts off its straight line. While it drives, the library
// steers it back: it measures the turn it did not ask for and cancels it, and it aims a little
// back toward the line, never more than 15 degrees off the heading written here. (It steers only
// a step that starts within 15 degrees of that heading, as this one does; otherwise turn first
// with face().) On the serial log, TSTR marks each tick's line where it is steering noticeably.
// If the robot still cannot finish within half an inch of the spot and about a degree of the
// heading, the step ends on its time limit: the result reads ROUTINE STOPPED with the fault
// MOTION_TIMEOUT, and the brake at the end still runs. That is the library telling the truth
// about this robot, not a broken auton: read END x and y (how far short, how far beside) and the
// TSTR lines before changing anything.
chassis::RoutineResult fortyInchesForward(competition::AutonContext& ctx) {
    chassis::Routine r{ctx.chassis, "40in-forward"};
    r.moveTo(math::Pose2d{40_in, 0_in, 0_deg});
    return r.result();
}

}  // namespace

void registerAutons(competition::AutonRegistry& registry) {
    // 1. THE 40 IN AUTON.
    competition::AutonEntry forty{};
    forty.name = "40in forward";                      // what the brain's menu shows
    forty.kind = competition::AutonKind::Match;       // runs in the 30 s autonomous period
    forty.routine = &fortyInchesForward;              // the function above
    forty.start = competition::StartPose::relative(); // where we place it is (0, 0), facing 0
    forty.endActionAt = 27_s;  // still running at 27 s? it is stopped, and the end action starts
    forty.hardStopAt = 29_s;   // everything stops: 2 s after 27 s, 1 s before the field's 30 s
    // endAction is left out: the end action is chassis.brake() -- 0 V under Brake until still.
    registry.add(forty);

    // 2. THE LIBRARY'S ACCEPTANCE RUN (drive 24 in, turn to 90 degrees, face the start, drive
    //    back): the test that the whole library works on this robot, on the same schedule.
    registry.add(competition::acceptanceRunEntry(competition::AutonKind::Match, 27_s, 29_s));
}

}  // namespace shulib::bench
