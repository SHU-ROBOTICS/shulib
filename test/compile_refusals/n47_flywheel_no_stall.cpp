// n47_flywheel_no_stall: a flywheel with NO stall signature.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Flywheel bad{.name = "bad", .motors = {Forward}, .cartridge = Blue, .gearing = direct, .safe = Coast, .atSpeed = within(60, 0.1_s), .presets = {{"go", 300}}};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(4)));
// expect: speed_the_stall_signature_is_unset_a_jammed_wheel_must_never_sit_at_stall_current
