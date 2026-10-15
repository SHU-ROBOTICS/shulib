// s24_linkage_four_stages: a linkage of 4 stages.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Lift bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = linkage(14_in, 10_in, 4), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in}, .gravity = noGravity, .home = restsAtStop(0_in)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_an_arm_needs_one_to_three_stages
