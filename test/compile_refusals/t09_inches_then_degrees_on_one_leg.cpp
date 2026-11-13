// t09_inches_then_degrees_on_one_leg: afterDistance(...)...afterTurning(...) on one leg leaves no kind of leg.
// Written by STEP1-B (the STEP1 brief's ST34, re-check #3); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { (void)r.afterDistance(20_in).startRun(rx::intake, 12_volt, 1_s).afterTurning(45_deg); }
// expect: after N degrees is a turn's mark -- this leg already has a drive's mark (afterDistance)
// expect-at: 5
