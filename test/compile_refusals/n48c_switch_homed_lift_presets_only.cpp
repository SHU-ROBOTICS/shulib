// n48c_switch_homed_lift_presets_only: the switch-homed lift PRESET (no hold) with no way to home it in driver control.
// Written by MECH-A4's review round; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(rx::kRobotXMechanisms, whileHeld(rx::arm, Button::R1, Button::R2, 6_volt), preset(rx::lift["high"], Button::Up, 4_s), toggle(rx::flywheel["shoot"], Button::X), whileHeld(rx::intake, Button::A, Button::B, 12_volt), autonOnly(rx::hood));
// expect: a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control
// expect-at: 5
