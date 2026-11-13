// m2_button_twice_on_the_ninth_line: MULTI-LINE: Right used twice (a preset on the 3rd control line, the intake's back on the 9th) — refused at the list's FIRST line, naming the button.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls cc = controls(rx::kRobotXMechanisms,
    whileHeld(rx::arm, Button::R1, Button::R2, 6_volt),
    preset(rx::arm["stow"], Button::Down, 1.5_s),
    preset(rx::arm["score"], Button::Right, 1.5_s),
    preset(rx::arm["high"], Button::Up, 1.5_s),
    whileHeld(rx::lift, Button::L1, Button::L2, 12_volt),
    homeAtDriverStart(rx::lift, 1.5_s),
    toggle(rx::flywheel["shoot"], Button::X),
    whileHeld(rx::intake, Button::A, Button::Right, 12_volt),
    autonOnly(rx::hood));
// expect: button_Right_does_two_jobs
// expect-at: 5
