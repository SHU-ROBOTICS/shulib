// n53_volts_without_their_unit: r.run(intake, 12, 1_s) — volts written without their unit.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/chassis/routine.hpp"
void f(shulib::chassis::Routine& r) { r.run(rx::intake, 12, 1_s); }
// expect: volts are written with their unit: 12_volt, not 12
// expect-at: 5
