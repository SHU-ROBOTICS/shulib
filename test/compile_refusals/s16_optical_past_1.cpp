// s16_optical_past_1: an optical threshold past 1.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .pieceSensor = opticalSensor(1.5), .safe = Coast};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1), pieceSensor(9)));
// expect: an_optical_piece_sensor_needs_its_proximity_threshold_in_above_0_and_at_most_1
