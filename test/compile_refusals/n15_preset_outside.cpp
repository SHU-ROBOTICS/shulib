// n15_preset_outside: a preset outside the travel limits.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 90}, .gravity = noGravity, .home = restsAtStop(0), .presets = {{"high", 120}}};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_a_preset_is_outside_the_travel_limits
