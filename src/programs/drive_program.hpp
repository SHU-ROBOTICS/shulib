#pragma once
//
// "shulib Drive" — robot two's DRIVER PROGRAM: the team's driver file, run on the library's front
// door (chunk T4 Part 3, 2026-10-01; the team lead's rulings R3 and R5 of 2026-09-27). Built with
// `make ROBOT=tank PROGRAM=drive`, uploaded to slot 1 as "shulib Drive".
//
// It REPLACED, in place — the same name, slot, build switch and beacon (R3, "why cant we just
// replace it?") — the old "shulib Drive" of R3b Part 0b (2026-09-10), which drove robot two
// open-loop through bare hal/pros adapters over the typed chassis table, toward the typed table's
// front (the 15/16 end). THIS ONE drives through robot::Robot, built from the calibration record on
// the SD card, toward the RECORD's front (the camera end): the SAME stick moves the robot the OTHER
// way. A brain that still holds the old package shows `adapters, NOT the motion stack` on its
// screen and `[R3B] booted: shulib DRIVE` on serial; this one says `[DRV]` (worksheet Station F.0
// makes telling them apart a REQUIRED step before a wheel moves). The old program's protections
// were not lost: they are the library's since T4 Part 2 (teleop/drive_protection.hpp), in every
// program that drives through teleop::runDriverControl, this one included.
//
// It also RETIRED "shulib Teleop" (R5): Teleop's diagnostics — "the fault row, the claw row, where
// the drivetrain numbers came from, the run status" — are this program's DETAILS page, with the
// rest of Teleop's panel on its MORE page (src/programs/drive_panel.hpp), and Teleop's boot lines
// are this program's.
//
// RUN STATUS: THIS VERSION has not run on hardware (the program says so at every boot).

namespace shulib::bench {

/// The reason every autonomous() of this program gives, on the brain's state screen (src/main.cpp)
/// and in the one serial line driveAutonomous() writes. At most 50 characters (the screen's SMALL
/// budget; checked in src/programs/drive_program.cpp) and a promise: this program NEVER moves in
/// autonomous — autonomous routines run only in the competition program's autonomous() and, on the
/// bench, the Auton Tester's RUN, through the same library call (the team lead's ruling,
/// 2026-09-25; COMP-t).
inline constexpr const char kDriveAutonomousReason[] = "No motion here, ever: Drive has no autonomous.";

/// initialize()'s tank-drive branch. Prints the boot banner (`booted: shulib DRIVE build
/// [<beacon>]`, the build stamp, the run status) and the `competition status at boot:` line from
/// the two arguments src/main.cpp formats — the BEACON stays a literal in src/build_selection.hpp's
/// one chain, so only the build that selected this program links it — then decides the boot source
/// (src/common/boot_source.hpp), and builds the robot::Robot ONCE over a
/// hal::pros::ProsDeviceProvider that keeps a copy of the log on the SD card (`drive_log.txt`,
/// events and summaries only — the team lead's N5 (a) and A (a)), binding robot two's driver file
/// to it inside the SAME try; then hands its fault latch to main.cpp's precondition handler and
/// writes the session header and the boot lines (one per claw, BUILT or ABSENT). On any refusal it
/// says so and constructs nothing further: nothing is powered.
void driveInitialize(const char* variantBeacon, const char* competitionStatusBits);

/// autonomous()'s tank-drive branch, called after src/main.cpp has painted the AUTONOMOUS state
/// screen with kDriveAutonomousReason. Logs that same sentence once, as a warning, when the robot
/// was built; does nothing otherwise. Commands NO motion, and no routine is ever added here.
void driveAutonomous();

/// opcontrol()'s tank-drive branch: robot two's driver file (src/team/robot_two_driver.hpp) —
/// which IS the library's driver loop, teleop::runDriverControl, with the library's drive
/// protections — over the robot's Chassis, with the program's screen. Returns when the field
/// disables the robot, after cancel() (0 V + Brake) and each claw pivot's safe state (the pneumatic
/// claw is left as it is); PROS calls opcontrol() again when driver control resumes. When the boot
/// refused, it says why again and idles, powering nothing.
void runDrive();

}  // namespace shulib::bench
