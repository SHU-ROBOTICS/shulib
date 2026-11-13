// n45_stall_home_pushes_away: a stall-home at the bottom pushing + (it would zero on the TOP stop).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = homeByStall(0, 3_volt, {.amps = 1.5, .belowMotorRpm = 2, .lasting = 0.2_s})};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_the_homing_push_points_away_from_its_home_end
