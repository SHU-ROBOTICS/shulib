// d_button_Right_twice: button Right doing two jobs (pivot 6's hold and the claw's toggle) — NAMED as Right.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(r2::kRobotTwoMechanisms, whileHeld(r2::clawPivot6, Button::Right, Button::R2, 12_volt), whileHeld(r2::clawPivot7, Button::L1, Button::L2, 12_volt), toggle(r2::claw, Button::Right));
// expect: button_Right_does_two_jobs
// expect-at: 5
