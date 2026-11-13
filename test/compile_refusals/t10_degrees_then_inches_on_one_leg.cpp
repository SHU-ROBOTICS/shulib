// t10_degrees_then_inches_on_one_leg: afterTurning(...)...afterDistance(...) on one leg leaves no kind of leg.
// Written by STEP1-B (the STEP1 brief's ST34, re-check #3); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { (void)r.afterTurning(45_deg).startRun(rx::intake, 12_volt, 1_s).afterDistance(20_in); }
// expect: after N inches is a drive's mark -- this leg already has a turn's mark (afterTurning)
// expect-at: 5
