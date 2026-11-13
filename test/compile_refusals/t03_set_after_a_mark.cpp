// t03_set_after_a_mark: r.atPercent(60).set(...) — a pneumatic's set has no start form to trigger.
// Written by STEP1-B (the STEP1 brief's ST14); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.atPercent(60).set(rx::hood["up"]); }
// expect: a partway trigger STARTS a step, and set() has no start form
// expect-at: 5
