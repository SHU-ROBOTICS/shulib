// s04_no_motor_roles: a power mechanism with no .motors.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power bad{.name = "bad", .cartridge = Red, .safe = Hold};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_motor_mechanism_needs_one_to_four_motor_roles
