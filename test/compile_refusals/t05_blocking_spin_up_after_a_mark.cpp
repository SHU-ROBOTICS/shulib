// t05_blocking_spin_up_after_a_mark: r.afterTurning(45_deg).spinUp(...) — a partway trigger STARTS a step.
// Written by STEP1-B (the STEP1 brief's ST14); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.afterTurning(45_deg).spinUp(rx::flywheel["shoot"], 2_s); }
// expect: a partway trigger STARTS a step: write .startSpinUp(...)
// expect-at: 5
