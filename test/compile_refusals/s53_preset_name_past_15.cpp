// s53_preset_name_past_15: a preset name of 16 characters.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = restsAtStop(0), .presets = {{"sixteen_chars___", 5}}};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_preset_or_position_name_does_not_fit_fifteen_characters
