// n38_two_share_a_name: two DIFFERENT declarations with one name (the run-time lookup is by name).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power intake2{.name = "intake", .motors = {Forward}, .cartridge = Blue, .safe = Coast};
inline constexpr MechanismTable m = mechanisms(wire(rx::intake, motors(5), pieceSensor(9)), wire(intake2, motors(6)));
// expect: two_mechanisms_share_a_name
