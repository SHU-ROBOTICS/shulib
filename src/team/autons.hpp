#pragma once
//
// autons.hpp -- the team's autons: the one function both auton-running programs call, and the
// two periods every auton's schedule is checked against. The autons themselves are in
// src/team/autons.cpp.
//
// "shulib Auton Tester" and "shulib Competition" both call registerAutons() from initialize(),
// inside the try around building the robot: an entry the registry refuses throws a
// PreconditionError whose sentence names the auton, the program refuses its boot with that
// sentence on the screen, and nothing moves.
//
// PROS-free on purpose: the host suite compiles src/team/autons.cpp and runs every auton on the
// host test plant (test/autons_test.cpp).

#include "shulib/competition/auton_registry.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

/// The two periods an auton can run in, from the VEX U game manual (Override, v1.0, 2026-07-02):
/// a head-to-head match opens with a 30 s autonomous period (<VUT4>), and an autonomous coding
/// skills run lasts 60 s (Appendix B). An entry whose hardStopAt does not fit inside its kind's
/// period is refused by name when it is registered.
inline constexpr competition::AutonPeriods kAutonPeriods{.match = units::Time{30.0},
                                                         .skills = units::Time{60.0}};

/// Register every auton the team has written, in the order the brain's menu lists them. Called
/// ONCE, from a program's initialize(), inside its try: `registry.add()` throws a
/// PreconditionError naming the auton when an entry is refused.
void registerAutons(competition::AutonRegistry& registry);

}  // namespace shulib::bench
