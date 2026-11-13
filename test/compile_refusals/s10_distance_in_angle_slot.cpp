// s10_distance_in_angle_slot: a distance sensor as an arm's angle sensor.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .angleSensor = distanceSensor(2_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = noGravity, .home = restsAtStop(0)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1), angleSensor(10)));
// expect: the_angle_sensor_role_is_not_a_rotation_sensor
