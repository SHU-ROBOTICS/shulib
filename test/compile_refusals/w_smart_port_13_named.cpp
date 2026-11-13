// w_smart_port_13_named: smart port 13 used twice — NAMED as port 13.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power a{.name = "a", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power b{.name = "b", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr MechanismTable m = mechanisms(wire(a, motors(13)), wire(b, motors(13)));
// expect: smart_port_13_is_used_twice
