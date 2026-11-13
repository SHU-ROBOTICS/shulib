// n05_run_on_an_arm: r.run(...) on an arm (a power mechanism's step) — refused by a sentence naming the steps to use.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.run(rx::arm, 12_volt, 1_s); }
// expect: r.run(...) is for a POWER mechanism
// expect-at: 5
