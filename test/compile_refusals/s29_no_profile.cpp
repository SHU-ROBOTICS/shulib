// s29_no_profile: a profile of 0 speed (by hand: the starting values always fill it).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::arm, motors(1)), [](mech::MechanismSpec& s) { s.position.maxVelocity = 0.0; }));
// expect: position_the_profile_needs_maxvelocity_and_maxacceleration_more_than_zero
