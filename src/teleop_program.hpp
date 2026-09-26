#pragma once
//
// "shulib Teleop" — the program in which THE LIBRARY drives robot two, built on robot::Robot
// (G1 Part 2b, 2026-09-26; the program itself dates from R3b Parts 1-3, 2026-09-14, and booted from
// the calibration record since R3d Part 2). Built with `make ROBOT=tank PROGRAM=library`, uploaded
// to slot 2 as "shulib Teleop".
//
// Three entry points, because the tank-library branches of THREE of src/main.cpp's PROS entry
// points forward here: initialize() builds the graph, autonomous() reports that nothing moves, and
// opcontrol() drives. Like "shulib Drive" and "shulib Calibrate", the program IS the build: no
// chooser, picked by name from the brain's slot list (the team lead's ruling, R3b Part 0b).
//
// RUN STATUS, in the words the program itself prints (the G1 Part 2b brief's A1): THIS VERSION of
// "shulib Teleop" (G1 Part 2b, built on robot::Robot) has not run on hardware. The previous version
// (build v0.1.1-319-g2780462) drove robot two from the calibration record on 2026-09-25 -- eight
// starts, open-loop under driver control through the library's own graph; no library-commanded
// motion has run (R3b open).

namespace shulib::bench {

/// The reason every autonomous() of this program gives, on the brain's state screen (src/main.cpp)
/// and in the one serial line teleopAutonomous() writes. At most 50 characters (the screen's SMALL
/// budget; checked where it is printed) and a promise: this program NEVER moves in autonomous —
/// autonomous routines run only in the competition program's autonomous() (the team lead's
/// ruling, 2026-09-25).
inline constexpr const char kTeleopAutonomousReason[] = "No motion here, ever: Teleop has no autonomous.";

/// initialize()'s tank-library branch. Prints the boot banner (`booted: shulib TELEOP build
/// [<beacon>]`, the build stamp, the run status) and the `competition status at boot:` line from
/// the two arguments src/main.cpp formats — the BEACON stays a literal in main.cpp's #if chain, so
/// only the build that selected this program links it (the brief's A2) — then decides the boot
/// source, builds the robot::Robot ONCE over a hal::pros::ProsDeviceProvider inside the graph
/// boundary's try, hands its fault latch to main.cpp's precondition handler, and writes the session
/// header and the boot lines. On any refusal it paints the refusal screen, says it on serial,
/// constructs nothing further, and returns: nothing is powered.
void teleopInitialize(const char* variantBeacon, const char* competitionStatusBits);

/// autonomous()'s tank-library branch, called after src/main.cpp has painted the AUTONOMOUS state
/// screen with kTeleopAutonomousReason. Logs that same sentence once, as a warning, when the robot
/// was built; does nothing otherwise. Commands NO motion, and no routine is ever added here.
void teleopAutonomous();

/// opcontrol()'s tank-library branch: the library's driver-control loop
/// (teleop::runDriverControl — the DriveShaper feel layer, default profile ArcadeBody,
/// field-centric selectable but OFF) over the robot's Chassis, with the brain panel and the
/// controller screen. Returns when the field disables the robot, after cancel() (0 V + Brake);
/// PROS calls opcontrol() again when driver control resumes. When the boot refused, it repaints
/// the refusal and idles, powering nothing.
void runTeleop();

}  // namespace shulib::bench
