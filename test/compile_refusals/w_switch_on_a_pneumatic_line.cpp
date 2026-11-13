// w_switch_on_a_pneumatic_line: a piece switch on a pneumatic's line — NAMED.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::hood, lines('C', 'D')), wire(ry::catapult, motors(7), pieceSensor('D')));
// expect: three_wire_port_D_is_used_twice
