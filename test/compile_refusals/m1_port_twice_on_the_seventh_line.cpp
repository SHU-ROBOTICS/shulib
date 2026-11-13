// m1_port_twice_on_the_seventh_line: MULTI-LINE: port 4 used twice on the list's 7th line — refused at the list's FIRST line, naming the port.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable mm = mechanisms(
    wire(rx::arm, motors(1)),
    wire(rx::lift, motors(2, 3), homeSwitch('B')),
    wire(rx::flywheel, motors(4)),
    wire(rx::intake, motors(5), pieceSensor(9)),
    wire(rx::hood, lines('C', 'D')),
    wire(r2::clawPivot6, motors(4)));
// expect: smart_port_4_is_used_twice
// expect-at: 4
