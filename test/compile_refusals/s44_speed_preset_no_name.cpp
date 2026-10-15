// s44_speed_preset_no_name: a speed preset named "".
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Flywheel bad{.name = "bad", .motors = {Forward}, .cartridge = Blue, .gearing = gears(72, 12), .safe = Coast, .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s}, .atSpeed = within(60, 0.1_s), .presets = {{"", 3000}}};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: speed_a_preset_has_no_name
