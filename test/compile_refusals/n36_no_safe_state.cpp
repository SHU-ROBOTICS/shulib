// n36_no_safe_state: no safe state (P6: never invented).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power bad{.name = "pivot", .motors = {Forward}, .cartridge = Red};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_motor_mechanisms_safe_brake_mode_is_unset
