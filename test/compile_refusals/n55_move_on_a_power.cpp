// n55_move_on_a_power: r.move(intake, ...) — move() takes a TARGET.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.move(rx::intake, 1_s); }
// expect: r.move(...) takes a TARGET
// expect-at: 5
