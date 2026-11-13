// s19_no_gearing: an arm with no .gearing.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = restsAtStop(0)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_gearratio_must_be_more_than_zero
