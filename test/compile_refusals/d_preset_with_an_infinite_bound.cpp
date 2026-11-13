// d_preset_with_an_infinite_bound: a preset whose time limit is INFINITE (no limit at all — P4: every action bounded).
// Written by MECH-A4's review round; run_refusal.py judges it (CMakeLists.txt beside it).
#include <limits>

#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(rx::kRobotXMechanisms, whileHeld(rx::arm, Button::R1, Button::R2, 6_volt), preset(rx::arm["score"], Button::Right, shulib::units::Time{std::numeric_limits<double>::infinity()}), whileHeld(rx::lift, Button::L1, Button::L2, 12_volt), homeAtDriverStart(rx::lift, 1.5_s), toggle(rx::flywheel["shoot"], Button::X), whileHeld(rx::intake, Button::A, Button::B, 12_volt), autonOnly(rx::hood));
// expect: a_preset_or_a_homing_needs_a_time_limit_above_zero
// expect-at: 7
