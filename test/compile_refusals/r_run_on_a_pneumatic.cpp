// r_run_on_a_pneumatic: r.run(...) on a pneumatic — the same sentence.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.run(rx::hood, 12_volt, 1_s); }
// expect: r.run(...) is for a POWER mechanism
// expect-at: 5
