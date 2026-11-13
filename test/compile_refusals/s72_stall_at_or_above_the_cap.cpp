// s72_stall_at_or_above_the_cap: a 1.5 A stall signature under a 1.2 A current cap -- a capped motor never draws
// more, so it could never trip (MECH-S-A, the MECH-S brief's CA13). run_refusal.py judges it.
#include "refusal_common.hpp"
inline constexpr Lift bad{.name = "bad", .motors = {Forward, Reversed}, .cartridge = Red, .gearing = direct, .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 1.5, .belowMotorRpm = 5, .lasting = 0.5_s}, .currentCap = {.amps = 1.2}, .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1, 2)));
// expect: a_stall_signatures_amps_must_be_below_the_current_cap_it_could_never_trip
