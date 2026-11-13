// n49_never_homed_held_no_unhomed_volts: a never-homed arm held with no .unhomedVolts (its buttons would command 0 V).
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr Arm nh{.name = "nh", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = neverHomed}; inline constexpr MechanismTable m = mechanisms(wire(nh, motors(1))); inline constexpr DriverControls c = controls(m, whileHeld(nh, Button::R1, Button::R2, 6_volt));
// expect: a_never_homed_mechanism_needs_unhomedVolts_for_its_buttons_to_move_it
// expect-at: 5
