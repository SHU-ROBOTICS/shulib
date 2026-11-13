// s68_gravity_hold_20_volts: a gravity hold of 20 V.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(20_volt, 0), .home = restsAtStop(0)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_the_gravity_hold_must_be_within_12_volts_either_way
