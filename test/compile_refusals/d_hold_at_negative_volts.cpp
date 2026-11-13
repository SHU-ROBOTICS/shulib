// d_hold_at_negative_volts: a hold at -6 V (the buttons say the direction; the volts are a size).
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(r2::kRobotTwoMechanisms, whileHeld(r2::clawPivot6, Button::R1, Button::R2, -6_volt), whileHeld(r2::clawPivot7, Button::L1, Button::L2, 12_volt), toggle(r2::claw, Button::A));
// expect: a_hold_needs_volts_above_0_and_at_most_12
// expect-at: 5
