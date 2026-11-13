// s30_no_tolerance: a 'reached' tolerance of 0 (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::arm, motors(1)), [](mech::MechanismSpec& s) { s.position.tolerance = 0.0; }));
// expect: position_tolerance_must_be_more_than_zero
