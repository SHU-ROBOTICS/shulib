// d_homing_a_never_homed_mechanism: homeOnPress on robot y's tilt, declared neverHomed.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(ry::kRobotYMechanisms, whileHeld(ry::wrist, Button::R1, Button::Y, 6_volt), homeAtDriverStart(ry::swing, 1.5_s), whileHeld(ry::swing, Button::Up, Button::Down, 6_volt), autonOnly(ry::dr4b), whileHeld(ry::tilt, Button::L1, Button::L2, 3_volt), homeOnPress(ry::tilt, Button::Left, 1_s), autonOnly(ry::roller), autonOnly(ry::catapult), autonOnly(ry::grip));
// expect: a_homing_control_on_a_mechanism_that_is_never_homed
// expect-at: 5
