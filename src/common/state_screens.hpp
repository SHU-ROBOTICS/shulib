#pragma once
//
// state_screens.hpp — THE COMPETITION-STATE SCREENS (2026-09-10, robot two's brain; moved out of
// src/main.cpp at G1 Part 2c, 2026-09-26, so the dispatcher has no screen code at all).
//
// ═══ WHY THEY EXIST ═════════════════════════════════════════════════════════════════
// The bench tester runs inside opcontrol(), so it exists only while VEXos reports DRIVER CONTROL,
// ENABLED. The first time a controller was linked to a brain running it, the program restarted and
// then showed a BLACK screen for as long as the controller stayed linked: PROS never started the
// driver task, and nothing on the brain said why. Every non-driver entry point of every program
// that drives only in driver control now paints the screen with the state VEXos reports and what to
// do about it, and the raw status bits print at boot — so "the touchscreen does not work" cannot be
// the diagnosis again (A4 register HA-128: launching from the controller's own menu arms the pretend
// match). "Bench Tests", "shulib Drive", "shulib Calibrate" and "shulib Teleop" share them;
// "shulib Calibrate" also paints its OWN not-driver-control screen from inside its loop (it powers
// motors and must say so in its own words), but needs these for disabled() and
// competition_initialize(), which run before and after its loop exists. The invented X-drive
// (src/programs/xdrive_program.cpp) never painted them and still does not.
//
// ═══ WHAT MOVED, AND WHAT DID NOT ═══════════════════════════════════════════════════
// Both functions are src/main.cpp's, byte for byte in what they print (G1-PROGRESS "Part 2c"
// carries the before/after). One change of SHAPE, not of output: every screen call now passes its
// text through a literal "%s" — the rule every screen print in src/ has followed since G1 Part 2b
// (PROS hands the formatted text to VEXos as a format string, so a caller's text must never be
// the format). No literal here contains a '%', so the pixels are the ones the brain showed (this
// screen ran on robot two on 2026-09-10 — R3b-PROGRESS §9.3 — with no recorded word on whether
// every line fit). The state and reason strings callers pass are program literals, ASCII and
// '%'-free — keep them so. This path deliberately does NOT go through src/common/screen_text.hpp's
// printText(): its SMALL budget (50 characters, an estimate — A4 register HA-136) would cut three
// of the fixed lines below (53-54 characters) and all four of src/main.cpp's own reason literals
// (DISABLED, FIELD CONTROL, the tester's and Calibrate's AUTONOMOUS: 66-126 characters; only
// "shulib Teleop"'s own reason, kTeleopAutonomousReason, is checked to fit 50), so it would change
// what the brain shows, and G1 Part 2c changes nothing a robot shows.
//
// PROS-bound (pros/misc.h, pros/screen.h) in the .cpp, so it lives in src/. This header is
// PROS-free. Compiled in every build, like every .cpp under src/ (the G1 Part 2 brief's A12);
// --gc-sections drops benchStateScreen() from the X-drive's package, where nothing calls it
// (competitionBits() stays: src/main.cpp formats every program's boot status bits with it).

#include <cstddef>
#include <cstdint>

namespace shulib::bench {

/// The competition status byte `status` (pros::c::competition_get_status()) in words, written into
/// `buf` (`n` bytes, always terminated): `0x04 field=yes disabled=no auton=no`. Returns `buf`, so a
/// caller can format and print in one expression. 64 bytes always holds the whole line.
const char* competitionBits(std::uint8_t status, char* buf, std::size_t n);

/// Paint the whole brain screen for a competition state the program does not run in — `state` in
/// LARGE yellow (`DISABLED`, `AUTONOMOUS`, `FIELD CONTROL CONNECTED`), `why` beneath it, the raw
/// status bits, and the three fixed lines that tell a person what to do — and print one
/// `[R3A] competition state: <state> (<bits>) -- <why>` line on serial. `state` and `why` must be
/// ASCII literals with no '%' (header).
void benchStateScreen(const char* state, const char* why);

}  // namespace shulib::bench
