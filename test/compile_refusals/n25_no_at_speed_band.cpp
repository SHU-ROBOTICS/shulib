// n25_no_at_speed_band: a flywheel with no .atSpeed (P6: the team states it).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Flywheel bad{.name = "bad", .motors = {Forward}, .cartridge = Blue, .gearing = gears(1, 1), .safe = Coast, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .presets = {{"go", 300}}};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(4)));
// expect: speed_the_at_speed_band_must_be_more_than_zero
