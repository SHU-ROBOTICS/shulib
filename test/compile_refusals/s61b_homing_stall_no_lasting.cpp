// s61b_homing_stall_no_lasting: a stall-HOME signature with no .lasting: it would zero on its own spin-up.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = homeByStall(0, -3_volt, {.amps = 1.5, .belowMotorRpm = 2})};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_stall_signature_needs_lasting_above_0
