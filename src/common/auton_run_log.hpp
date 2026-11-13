#pragma once
//
// auton_run_log.hpp — THE SD RUN LOG both auton-running programs append to: its file name, how its
// open went, and the words the boot line says about it (moved here at chunk COMP-b, 2026-09-30).
//
// "shulib Auton Tester" (COMP-t, 2026-09-28) appends every run's lines to ONE file on the card,
// across boots, through the adapter's Append mode (hal/pros/block_sink.hpp — the adapter owns the
// /usd/ prefix and every open mode; A4 register HA-177). The competition program appends its match
// runs to the SAME file (the COMP brief's ND5, answered yes), so the name is stated ONCE, here — two
// programs typing it twice could drift into two files, and the tester's runs and the match's would
// no longer be one record. The boot line's words came with it for the same reason: the card says
// the same thing whichever program opened it.
//
// PROS-free, header-only (the name is BARE — no /usd/: the adapter owns the prefix). Host-tested
// through the tester's boot lines (test/auton_tester_run_test.cpp (23b)) and the competition
// program's (test/competition_run_test.cpp). NOTHING HERE HAS RUN ON A ROBOT.

#include <cstdint>

namespace shulib::bench {

/// The run log's file, at the card's root: ONE file across boots and across both programs.
inline constexpr const char* kAutonRunLogFile = "shulib_auton_runs.txt";

/// How the run log opened at boot, for the boot line.
enum class AutonRunLog : std::uint8_t {
    Off,       ///< no card, or the log would not open: serial only
    Created,   ///< the log file was absent and was created
    Appended,  ///< the log file was there, and this boot's lines follow its last byte
};

/// The boot line's words before the path: "OFF (no card, or the log would not open)", "ON, CREATED "
/// or "ON, APPENDED to " (a program prints the path after the last two).
[[nodiscard]] constexpr const char* autonRunLogWords(AutonRunLog state) noexcept {
    switch (state) {
        case AutonRunLog::Off: return "OFF (no card, or the log would not open)";
        case AutonRunLog::Created: return "ON, CREATED ";
        case AutonRunLog::Appended: return "ON, APPENDED to ";
    }
    return "OFF (no card, or the log would not open)";
}

}  // namespace shulib::bench
