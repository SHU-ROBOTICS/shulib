// n35_name_past_23: a name past 23 characters.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power bad{.name = "this name is far too long", .motors = {Forward}, .cartridge = Red, .safe = Hold};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_mechanism_name_does_not_fit_twenty_three_characters
