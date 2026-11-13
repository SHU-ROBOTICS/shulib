// s01_no_name: a mechanism with no .name.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power bad{.motors = {Forward}, .cartridge = Red, .safe = Hold};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_mechanism_has_no_name
