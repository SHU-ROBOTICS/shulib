// n48d_preset_beside_unhomed_volts: a switch-homed lift with .unhomedVolts, held AND preset, no homing control — its holds move it unhomed, but a jog never homes it, so the preset would end NOT_HOMED for the match (the review's U5 finding).
// Written by MECH-A4's review round; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr Lift sl{.name = "sl", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = homeBySwitch(0_in, -3_volt), .presets = {{"high", 22_in}}, .unhomedVolts = 4_volt}; inline constexpr MechanismTable m = mechanisms(wire(sl, motors(1), homeSwitch('B'))); inline constexpr DriverControls c = controls(m, whileHeld(sl, Button::L1, Button::L2, 12_volt), preset(sl["high"], Button::Up, 4_s));
// expect: a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control
// expect-at: 5
