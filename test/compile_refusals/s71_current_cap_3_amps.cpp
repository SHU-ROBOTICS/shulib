// s71_current_cap_3_amps: a current cap of 3.0 A -- past the motor's own 2.5 A (MECH-S-A, the MECH-S brief's CA13).
// run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Lift bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .currentCap = {.amps = 3.0}, .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_current_cap_must_be_above_0_and_at_most_2_5_A
