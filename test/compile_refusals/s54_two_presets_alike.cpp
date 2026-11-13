// s54_two_presets_alike: two presets named alike.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = restsAtStop(0), .presets = {{"a", 1}, {"a", 2}}};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: two_presets_or_positions_share_a_name
