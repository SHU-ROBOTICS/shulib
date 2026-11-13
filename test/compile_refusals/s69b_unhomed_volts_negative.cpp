// s69b_unhomed_volts_negative: a negative unhomedVolts (it is a cap, never a direction).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = restsAtStop(0), .unhomedVolts = -3_volt};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_unhomedvolts_must_be_from_0_to_12
