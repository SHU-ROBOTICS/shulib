// t04_blocking_home_after_a_mark: r.afterDistance(10_in).home(...) — a partway trigger STARTS a step (startHome).
// Written by STEP1-B (the STEP1 brief's ST14); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.afterDistance(10_in).home(rx::lift, 1_s); }
// expect: a partway trigger STARTS a step: write .startHome(...)
// expect-at: 5
