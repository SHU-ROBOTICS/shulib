// n58_stall_at_3_amps: a stall signature at 3 A: a motor limited at 2.5 A never reads it.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 3.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = restsAtStop(0)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_stall_signatures_amps_must_be_below_the_motors_2_5_A_current_limit
