// t12_degrees_mark_on_move_to: r.afterTurning(30_deg)...moveTo(pose) — a drive has no degrees turned.
// Written by STEP1-B's fix round (review: each leg's kind refusal has its own TU, so one static_assert removed is seen).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.afterTurning(30_deg).startRun(rx::intake, 12_volt, 1_s).moveTo(shulib::math::Pose2d{}); }
// expect: after N degrees is a turn's mark -- a drive has no degrees turned; use afterDistance or atPercent
// expect-at: 5
