// w_switch_given_number: a NUMBER for a limit-switch piece sensor.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(ry::catapult, motors(7), pieceSensor(5)));
// expect: a_limit_switch_is_a_three_wire_device_give_it_a_letter
