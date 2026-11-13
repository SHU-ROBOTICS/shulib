// s55_absolute_reading_360: an absolute sensor reading 360 at zero.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 36), .angleSensor = rotationSensor(), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {-90, 90}, .gravity = noGravity, .home = absoluteSensor(0, 360)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1), angleSensor(10)));
// expect: absolute_homing_the_sensors_reading_at_zero_must_be_in_from_0_up_to_360
