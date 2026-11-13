// r_stop_on_a_pneumatic: r.stop(hood) — a pneumatic keeps its position.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.stop(rx::hood); }
// expect: r.stop(...) is for a power mechanism or a flywheel
// expect-at: 5
