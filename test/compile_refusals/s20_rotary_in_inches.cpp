// s20_rotary_in_inches: a rotary arm in inches (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::arm, motors(1)), [](mech::MechanismSpec& s) { s.position.units = mech::Units::Inches; }));
// expect: position_a_rotary_mechanism_is_in_degrees
