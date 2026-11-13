// w_port_twice_fourth_motor: a four-motor mechanism's FOURTH motor on the arm's port.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power quad{.name = "quad", .motors = {Forward, Forward, Forward, Forward}, .cartridge = Red, .safe = Coast};
inline constexpr MechanismTable m = mechanisms(wire(rx::arm, motors(1)), wire(quad, motors(2, 3, 4, 1)));
// expect: smart_port_1_is_used_twice
