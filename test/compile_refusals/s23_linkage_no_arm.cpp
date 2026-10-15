// s23_linkage_no_arm: a linkage with 0 in arms.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Lift bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = linkage(0_in, 10_in, 2), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in}, .gravity = noGravity, .home = restsAtStop(0_in)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_arm_length_must_be_more_than_zero
