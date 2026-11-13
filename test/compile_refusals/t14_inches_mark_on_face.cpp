// t14_inches_mark_on_face: r.afterDistance(20_in)...face(x, y) — a turn has no inches.
// Written by STEP1-B's fix round (review: each leg's kind refusal has its own TU, so one static_assert removed is seen).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.afterDistance(20_in).startRun(rx::intake, 12_volt, 1_s).face(0_in, 40_in); }
// expect: after N inches is a drive's mark -- a turn has no inches; use afterTurning or atPercent
// expect-at: 5
