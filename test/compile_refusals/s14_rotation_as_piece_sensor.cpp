// s14_rotation_as_piece_sensor: a rotation sensor as a piece sensor.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .pieceSensor = rotationSensor(), .safe = Coast};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1), pieceSensor(9)));
// expect: a_piece_sensor_cannot_be_a_rotation_sensor
