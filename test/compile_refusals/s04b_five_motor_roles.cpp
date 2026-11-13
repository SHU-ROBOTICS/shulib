// s04b_five_motor_roles: five motor roles.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power bad{.name = "bad", .motors = {Forward, Forward, Forward, Forward, Forward}, .cartridge = Red, .safe = Hold};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1, 2, 3, 4)));
// expect: a_motor_mechanism_needs_one_to_four_motor_roles
