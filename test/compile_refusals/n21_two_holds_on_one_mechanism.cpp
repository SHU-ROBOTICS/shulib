// n21_two_holds_on_one_mechanism: two holds on one mechanism (the arm).
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(rx::kRobotXMechanisms, whileHeld(rx::arm, Button::Up, Button::Down, 3_volt), whileHeld(rx::arm, Button::R1, Button::R2, 6_volt), whileHeld(rx::lift, Button::L1, Button::L2, 12_volt), homeAtDriverStart(rx::lift, 1.5_s), toggle(rx::flywheel["shoot"], Button::X), whileHeld(rx::intake, Button::A, Button::B, 12_volt), autonOnly(rx::hood));
// expect: two_controls_of_the_same_kind_on_one_mechanism
// expect-at: 5
