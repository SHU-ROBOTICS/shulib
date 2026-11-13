// t06_armed_handle_discarded: a trigger line with no drive after it — the armed handle is [[nodiscard]]: a
// warning, an ERROR under the suite's -Werror (as in the gate and CI; only a warning in a plain make — the
// STEP1 brief's re-check #7, which is why the Routine also says it at run time). Written by STEP1-B (ST14).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.atPercent(60).startMove(rx::lift["high"], 2_s); }
// expect: ignoring returned value of type
// expect: Routine::ArmedLeg<3>
// expect-at: 6
