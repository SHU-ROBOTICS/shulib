// s03_kind_unset: an UNSET kind byte (by hand: every declaration states its kind).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::arm, motors(1)), [](mech::MechanismSpec& s) { s.kind = static_cast<mech::Kind>(0); }));
// expect: a_mechanisms_kind_is_unset
