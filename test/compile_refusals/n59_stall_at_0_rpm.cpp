// n59_stall_at_0_rpm: a stall signature with no belowMotorRpm (0).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Flywheel bad{.name = "bad", .motors = {Forward}, .cartridge = Blue, .gearing = gears(72, 12), .safe = Coast, .stall = {.amps = 2.0, .lasting = 0.5_s}, .atSpeed = within(60, 0.1_s), .presets = {{"shoot", 3000}}};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: a_stall_signature_needs_belowMotorRpm_above_0
