// s41_flywheel_no_gearing: a flywheel with no .gearing.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Flywheel bad{.name = "bad", .motors = {Forward}, .cartridge = Blue, .safe = Coast, .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s}, .atSpeed = within(60, 0.1_s)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: speed_gearratio_and_its_free_speed_must_be_more_than_zero
