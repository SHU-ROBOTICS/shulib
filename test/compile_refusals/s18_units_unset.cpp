// s18_units_unset: an UNSET units byte (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::arm, motors(1)), [](mech::MechanismSpec& s) { s.position.units = static_cast<mech::Units>(0); }));
// expect: position_units_unset
