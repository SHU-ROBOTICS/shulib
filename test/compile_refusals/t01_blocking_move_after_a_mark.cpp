// t01_blocking_move_after_a_mark: r.atPercent(60).move(...) — a partway trigger STARTS a step (startMove).
// Written by STEP1-B (the STEP1 brief's ST14); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.atPercent(60).move(rx::arm["score"], 1_s); }
// expect: a partway trigger STARTS a step: write .startMove(...)
// expect-at: 5
