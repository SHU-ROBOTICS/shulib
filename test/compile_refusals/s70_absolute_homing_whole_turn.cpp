// s70_absolute_homing_whole_turn: an arm homed by an absolute angle sensor over a whole output turn of
// travel (-180..180) -- a reading within one turn cannot tell the turns apart (MECH-A3).
// Written for the MECH-A1 harness by MECH-A3; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm bad{.name = "bad", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 36), .angleSensor = rotationSensor(), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {-180, 180}, .gravity = noGravity, .home = absoluteSensor(0, 180)};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1), angleSensor(10)));
// expect: position_absolute_homing_needs_a_travel_shorter_than_one_turn_of_its_angle_sensor
