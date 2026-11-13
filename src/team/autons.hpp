#pragma once
//
// autons.hpp -- the team's autons: the one function both auton-running programs call, and the
// two periods every auton's schedule is checked against. The autons themselves are in
// src/team/autons.cpp.
//
// "shulib Auton Tester" and "shulib Competition" both call registerAutons() from initialize(),
// inside the try around building the robot, on a registry that DROPS an entry it refuses (the team
// lead's ruling of 2026-10-02 for the competition program, "drop just that auton"; the tester does
// the same): that entry alone is left off the menu and named there in red and in the boot's log, by
// a sentence that names the auton, and every other auton still runs — and, in the competition
// program, driver control. Fix the entry in src/team/autons.cpp and upload again.
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
/// ONCE, from a program's initialize(), inside its try. An entry the registry refuses is dropped
/// by both programs' registries (the file banner); a registry built without asking for that
/// throws a PreconditionError naming the auton at its `registry.add()`.
void registerAutons(competition::AutonRegistry& registry);

}  // namespace shulib::bench
