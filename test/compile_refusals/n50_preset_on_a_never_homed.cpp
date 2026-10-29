// n50_preset_on_a_never_homed: a PRESET on a never-homed arm (always NOT_HOMED: a dead button).
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr Arm nh{.name = "nh", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = neverHomed, .presets = {{"up", 90}}, .unhomedVolts = 4_volt}; inline constexpr MechanismTable m = mechanisms(wire(nh, motors(1))); inline constexpr DriverControls c = controls(m, whileHeld(nh, Button::R1, Button::R2, 6_volt), preset(nh["up"], Button::Up, 1_s));
// expect: a_preset_on_a_mechanism_that_is_never_homed
// expect-at: 5
