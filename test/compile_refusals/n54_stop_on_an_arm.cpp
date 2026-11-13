// n54_stop_on_an_arm: r.stop(arm) — an arm HOLDS; stop() is a power mechanism's or a flywheel's.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.stop(rx::arm); }
// expect: r.stop(...) is for a power mechanism or a flywheel
// expect-at: 5
