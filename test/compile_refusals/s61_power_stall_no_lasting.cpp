// s61_power_stall_no_lasting: a power's signature with no .lasting: it would trip on every start.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .safe = Coast, .stall = {.amps = 2.0, .belowMotorRpm = 5}};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_stall_signature_needs_lasting_above_0
