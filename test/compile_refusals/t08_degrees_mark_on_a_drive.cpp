// t08_degrees_mark_on_a_drive: r.afterTurning(45_deg)...driveTo(...) — a drive has no degrees turned.
// Written by STEP1-B (the STEP1 brief's ST34, re-check #3); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.afterTurning(45_deg).startRun(rx::intake, 12_volt, 1_s).driveTo(40_in, 0_in); }
// expect: after N degrees is a turn's mark -- a drive has no degrees turned; use afterDistance or atPercent
// expect-at: 5
