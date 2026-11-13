// s43_seven_speed_presets: seven speed presets (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::flywheel, motors(4)), [](mech::MechanismSpec& s) { s.speed.presetCount = 7; }));
// expect: speed_more_presets_than_fit
