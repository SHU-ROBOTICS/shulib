// t02_blocking_run_after_a_mark: r.atPercent(60).run(...) — a partway trigger STARTS a step (startRun).
// Written by STEP1-B (the STEP1 brief's ST14); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.atPercent(60).run(rx::intake, 12_volt, 1_s); }
// expect: a partway trigger STARTS a step: write .startRun(...)
// expect-at: 5
