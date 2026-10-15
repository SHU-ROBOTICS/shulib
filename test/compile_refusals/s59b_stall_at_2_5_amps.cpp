// s59b_stall_at_2_5_amps: a stall signature at exactly 2.5 A (the bound is strict).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.5, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = restsAtStop(0)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_stall_signatures_amps_must_be_below_the_motors_2_5_A_current_limit
