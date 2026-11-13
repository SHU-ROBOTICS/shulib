// s17_max_volts_13: a power mechanism allowed 13 V.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .safe = Coast, .maxVolts = 13_volt};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: power_maxvolts_is_not_in_above_0_and_at_most_12
