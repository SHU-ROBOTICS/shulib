// t11_degrees_mark_on_face_away_after_inches: atPercent then afterDistance narrows the leg to a drive: faceAway refused.
// Written by STEP1-B (the STEP1 brief's ST34: arms chained on one leg NARROW it); run_refusal.py judges it.
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.atPercent(30).startRun(rx::intake, 12_volt, 1_s).afterDistance(5_in).startMove(rx::arm["score"], 1_s).faceAway(0_in, 40_in); }
// expect: after N inches is a drive's mark -- a turn has no inches
// expect-at: 5
