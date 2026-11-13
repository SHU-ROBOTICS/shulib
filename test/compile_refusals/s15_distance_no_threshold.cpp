// s15_distance_no_threshold: a distance piece sensor seeing within 0 in.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .pieceSensor = distanceSensor(0_in), .safe = Coast};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1), pieceSensor(9)));
// expect: a_distance_piece_sensor_needs_its_sees_within_distance_more_than_zero
