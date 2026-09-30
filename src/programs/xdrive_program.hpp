#pragma once
//
// The INVENTED X-DRIVE program — `make ROBOT=xdrive` (G1 Part 2c, 2026-09-26; the graph itself dates
// from R1a, 2026-08-13, and until Part 2c was hand-wired inside src/main.cpp).
//
// The robot is the constant in src/robots/xdrive_config.hpp, built by robot::Robot over
// hal::pros::ProsDeviceProvider; its driver control is the library's loop,
// teleop::runDriverControl, with the DriveShaper's default profile and the drive COASTING on stick
// release. EVERY PORT IS INVENTED (A4 register HA-111): on any robot that exists today the build
// refuses at boot, names the first port that is not what the config says, and powers nothing.
//
// Three entry points, because three of src/main.cpp's PROS entry points forward here; the
// X-drive's disabled() and competition_initialize() paint nothing, as they never did.
//
// RUN STATUS: this version has never booted, and cannot on any robot that exists. The one X-drive
// boot on record is the old hand-wired graph over FAKE devices, 2026-08-12 (it drove nothing);
// none since R1a gave it real adapters.

namespace shulib::bench {

/// initialize()'s X-drive branch. Prints the boot banner FIRST — the build's beacon (the literal
/// is in src/build_selection.hpp's one chain, so only this build links it) and `competitionStatusBits`
/// (state_screens.hpp's competitionBits(), formatted by main.cpp), exactly as the other programs
/// do — so even a refused boot says which build it was. Then builds the robot ONCE over a
/// ProsDeviceProvider inside the graph boundary's try — validate() first, then the construction —
/// hands its fault latch to src/main.cpp's precondition handler, and writes the session header and
/// the boot lines. On any refusal (on every robot that exists: the ports are invented) it paints
/// the refusal, says it on serial, and returns with nothing powered.
void xdriveInitialize(const char* variantBeacon, const char* competitionStatusBits);

/// autonomous()'s X-drive branch: one Warn line when the robot was built — NO motion here, ever
/// (autonomous routines run only in the competition program's autonomous() and, on the bench, the
/// Auton Tester's RUN through the same library call — the team lead's ruling of 2026-09-25) — and
/// nothing otherwise.
void xdriveAutonomous();

/// opcontrol()'s X-drive branch: teleop::runDriverControl over the robot's Chassis (default
/// profile ArcadeBody, field-centric OFF, release COAST), updating the controller's fault display
/// each tick. Returns when the field disables the robot, after cancel(); PROS calls opcontrol()
/// again when driver control resumes. When the boot refused, it repaints the refusal and idles,
/// powering nothing.
void runXdrive();

}  // namespace shulib::bench
