// s36_stall_homing_no_signature: stall homing with no signature.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = homeByStall(0, -3_volt, {})};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_stall_homing_needs_its_stall_signature
