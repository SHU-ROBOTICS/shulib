// s34_absolute_no_angle_sensor: absolute homing with no angle sensor.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = absoluteSensor(0, 180)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_absolute_homing_needs_an_anglesensor_rotation_role
