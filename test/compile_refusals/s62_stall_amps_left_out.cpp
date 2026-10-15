// s62_stall_amps_left_out: a signature with its .amps left out (it would be IGNORED as 'no signature').
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .safe = Coast, .stall = {.belowMotorRpm = 5, .lasting = 0.5_s}};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_stall_signature_needs_its_amps_above_0
