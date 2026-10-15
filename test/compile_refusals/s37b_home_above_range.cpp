// s37b_home_above_range: a switch home ABOVE the range (25 in on a 0..24 in lift).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Lift bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in}, .gravity = noGravity, .home = homeBySwitch(25_in, 3_volt)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1), homeSwitch('B')));
// expect: position_the_home_position_is_outside_the_travel_limits
