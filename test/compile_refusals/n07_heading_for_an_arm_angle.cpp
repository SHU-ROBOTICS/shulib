// n07_heading_for_an_arm_angle: a HEADING where an arm angle belongs — a plain C++ error, said: _deg wraps (270 reads -90).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::arm.to(90_deg);
// expect: operator""_deg
