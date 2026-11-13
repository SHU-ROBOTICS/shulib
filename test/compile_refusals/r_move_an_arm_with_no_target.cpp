// r_move_an_arm_with_no_target: r.move(arm, 1.5_s) — the arm named, its target forgotten (arm["score"]).
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.move(rx::arm, 1.5_s); }
// expect: r.move(...) takes a TARGET
// expect-at: 5
