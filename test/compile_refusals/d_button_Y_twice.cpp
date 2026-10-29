// d_button_Y_twice: button Y doing two jobs (pivot 6's hold and the claw's toggle) — NAMED as Y.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(r2::kRobotTwoMechanisms, whileHeld(r2::clawPivot6, Button::Y, Button::R2, 12_volt), whileHeld(r2::clawPivot7, Button::L1, Button::L2, 12_volt), toggle(r2::claw, Button::Y));
// expect: button_Y_does_two_jobs
// expect-at: 5
