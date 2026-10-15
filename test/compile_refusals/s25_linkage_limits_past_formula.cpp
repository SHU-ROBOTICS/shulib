// s25_linkage_limits_past_formula: a DR4B whose top limit (40 in) is past its reach (10 + 2 x 14 = 38).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Lift bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = linkage(14_in, 10_in, 2), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 40_in}, .gravity = noGravity, .home = restsAtStop(0_in)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_an_arms_travel_limits_reach_past_its_linkage_the_height_formula_has_no_inverse_there
