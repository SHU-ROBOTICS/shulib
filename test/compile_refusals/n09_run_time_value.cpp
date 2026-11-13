// n09_run_time_value: a run-time value where a constant is checked — a plain C++ error, said (use clampedTo).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline double f(double x) { return rx::arm.to(x).value; }
// expect: is not a constant expression
