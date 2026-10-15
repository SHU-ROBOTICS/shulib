// s38_seven_presets: seven presets (by hand: the declaration holds six).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::arm, motors(1)), [](mech::MechanismSpec& s) { s.position.presetCount = 7; }));
// expect: position_more_presets_than_fit
