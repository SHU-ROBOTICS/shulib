// n60_route_id_too_long: a route id past 23 characters (conveyor_belt_one_until_piece is 29).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power belt{.name = "conveyor belt one", .motors = {Forward}, .cartridge = Blue, .pieceSensor = distanceSensor(2_in), .safe = Coast};
inline constexpr MechanismTable m = mechanisms(wire(belt, motors(1), pieceSensor(9)));
// expect: a_route_id_does_not_fit_23_characters_shorten_the_mechanism_or_preset_name
