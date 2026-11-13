// n37_table_not_increasing: a measured table that is not strictly increasing.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr HeightAt kBad[] = {{0_in, 0}, {6_in, 30}, {5_in, 70}};
inline constexpr Lift bad{.name = "tilt", .motors = {Forward}, .cartridge = Red, .gearing = gears(12, 84), .shape = measured(kBad), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 6_in}, .gravity = noGravity, .home = restsAtStop(0_in)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_the_lookup_table_is_not_strictly_increasing
