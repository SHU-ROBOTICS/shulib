// d_button_L1_twice: button L1 doing two jobs (pivot 6's hold and the claw's toggle) — NAMED as L1.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(r2::kRobotTwoMechanisms, whileHeld(r2::clawPivot6, Button::L1, Button::R2, 12_volt), whileHeld(r2::clawPivot7, Button::L2, Button::R1, 12_volt), toggle(r2::claw, Button::L1));
// expect: button_L1_does_two_jobs
// expect-at: 5
