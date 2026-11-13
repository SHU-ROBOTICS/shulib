// t07_inches_mark_on_a_turn: r.afterDistance(20_in)...turnTo(90_deg) — a turn has no inches.
// Written by STEP1-B (the STEP1 brief's ST34, re-check #3); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.afterDistance(20_in).startRun(rx::intake, 12_volt, 1_s).turnTo(90_deg); }
// expect: after N inches is a drive's mark -- a turn has no inches; use afterTurning or atPercent
// expect-at: 5
