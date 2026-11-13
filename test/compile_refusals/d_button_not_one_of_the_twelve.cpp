// d_button_not_one_of_the_twelve: a control on a cast byte that is not one of the controller's twelve buttons.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(r2::kRobotTwoMechanisms, whileHeld(r2::clawPivot6, static_cast<Button>(12), Button::R2, 12_volt), whileHeld(r2::clawPivot7, Button::L1, Button::L2, 12_volt), toggle(r2::claw, Button::A));
// expect: a_control_names_a_button_that_is_not_one_of_the_twelve
// expect-at: 5
