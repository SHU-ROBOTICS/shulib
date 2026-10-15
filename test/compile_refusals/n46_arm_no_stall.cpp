// n46_arm_no_stall: an arm with NO stall signature (its hold could sit at stall current).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .range = {0, 160}, .gravity = noGravity, .home = restsAtStop(0)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_the_stall_signature_is_unset_a_held_mechanism_must_never_sit_at_stall_current
