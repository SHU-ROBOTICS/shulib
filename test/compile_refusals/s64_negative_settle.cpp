// s64_negative_settle: a negative settle time (by hand: .tuning.settle replaces only a value above 0).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::arm, motors(1)), [](mech::MechanismSpec& s) { s.position.settleTime = -0.1; }));
// expect: position_the_settle_time_must_not_be_negative
