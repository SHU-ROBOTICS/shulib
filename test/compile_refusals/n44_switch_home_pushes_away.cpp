// n44_switch_home_pushes_away: a switch-home at the BOTTOM pushing UP.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Lift bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = homeBySwitch(0_in, 3_volt)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(2), homeSwitch('B')));
// expect: position_the_homing_push_points_away_from_its_home_end
