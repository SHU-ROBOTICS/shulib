// n61_route_ids_alike: a preset named "home" beside its homing: two route ids arm_home.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "arm", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = restsAtStop(0), .presets = {{"home", 0}, {"score", 95}}};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: two_route_ids_are_the_same_rename_a_mechanism_a_preset_or_a_position
