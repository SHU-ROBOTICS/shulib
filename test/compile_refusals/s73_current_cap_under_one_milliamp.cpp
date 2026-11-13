// s73_current_cap_under_one_milliamp: a current cap of 0.0004 A -- above 0, but a V5 motor holds its limit in whole
// milliamps, so it would be sent as 0 mA, a motor with no torque at all (MECH-S-A's review; refusal 73). run_refusal.py
// judges it.
#include "refusal_common.hpp"
inline constexpr Power bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .safe = Hold, .currentCap = {.amps = 0.0004}};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_current_cap_must_be_at_least_0_001_A_a_motor_holds_whole_milliamps
