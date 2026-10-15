// s67_homing_push_30_volts: a homing push of -30 V.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Lift bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in}, .gravity = noGravity, .home = homeBySwitch(0_in, -30_volt)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1), homeSwitch('B')));
// expect: position_the_homing_push_must_be_within_12_volts_either_way
