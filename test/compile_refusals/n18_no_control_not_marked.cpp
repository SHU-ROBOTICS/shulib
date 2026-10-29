// n18_no_control_not_marked: a mechanism (the hood) with no control and not marked autonOnly.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(rx::kRobotXMechanisms, whileHeld(rx::arm, Button::R1, Button::R2, 6_volt), whileHeld(rx::lift, Button::L1, Button::L2, 12_volt), homeAtDriverStart(rx::lift, 1.5_s), toggle(rx::flywheel["shoot"], Button::X), whileHeld(rx::intake, Button::A, Button::B, 12_volt));
// expect: a_mechanism_has_no_driver_control_mark_it_autonOnly_if_that_is_meant
// expect-at: 5
