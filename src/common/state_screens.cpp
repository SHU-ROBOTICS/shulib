// state_screens.cpp — the competition-state screens (header: why they exist, what moved at G1
// Part 2c). Moved from src/main.cpp; the output is unchanged.

#include "common/state_screens.hpp"

#include <cstdint>
#include <cstdio>

#include "pros/misc.h"
#include "pros/screen.h"

namespace shulib::bench {

static_assert(kStatusDisabled == static_cast<std::uint8_t>(COMPETITION_DISABLED)
                  && kStatusAutonomous == static_cast<std::uint8_t>(COMPETITION_AUTONOMOUS)
                  && kStatusConnected == static_cast<std::uint8_t>(COMPETITION_CONNECTED),
              "the PROS-free status bits (state_screens.hpp) are PROS's");

const char* competitionBits(std::uint8_t s, char* buf, std::size_t n) {
    std::snprintf(buf, n, "0x%02x field=%s disabled=%s auton=%s", static_cast<unsigned>(s),
                  (s & COMPETITION_CONNECTED) ? "yes" : "no",
                  (s & COMPETITION_DISABLED) ? "yes" : "no",
                  (s & COMPETITION_AUTONOMOUS) ? "yes" : "no");
    return buf;
}

void benchStateScreen(const char* state, const char* why) {
    char bits[64];
    competitionBits(pros::c::competition_get_status(), bits, sizeof bits);
    std::printf("[R3A] competition state: %s (%s) -- %s\n", state, bits, why);
    std::fflush(stdout);
    // "status bits " + the bits, formatted here so the screen call's format is the literal "%s".
    char statusLine[80];
    std::snprintf(statusLine, sizeof statusLine, "status bits %s", bits);
    pros::c::screen_set_eraser(0x000000);
    pros::c::screen_erase();
    pros::c::screen_set_pen(0xF0C000);
    pros::c::screen_print_at(pros::E_TEXT_LARGE, 10, 30, "%s", state);
    pros::c::screen_set_pen(0xFFFFFF);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 80, "%s",
                             "This program runs ONLY in DRIVER CONTROL, enabled.");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 100, "%s", why);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 130, "%s", statusLine);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 160, "%s",
                             "Look at the CONTROLLER's screen: it names this state.");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 176, "%s",
                             "Unplug any competition switch or field cable from the");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 192, "%s",
                             "controller's smart port. This screen clears by itself.");
}

}  // namespace shulib::bench
