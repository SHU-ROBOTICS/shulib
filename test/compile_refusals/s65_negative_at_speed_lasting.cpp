// s65_negative_at_speed_lasting: an at-speed lasting of -0.1 s (the band, 60, is correct).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Flywheel bad{.name = "bad", .motors = {Forward}, .cartridge = Blue, .gearing = gears(72, 12), .safe = Coast, .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s}, .atSpeed = within(60, -0.1_s)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: speed_the_at_speed_lasting_must_not_be_negative
