// n12_motor_count: the lift wired with ONE motor port (it has two) — pinned at its wire(...) line: mechanisms()' own check would name it too.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(
    wire(rx::arm, motors(1)),
    wire(rx::lift, motors(2), homeSwitch('B')));
// expect: the_wiring_gives_a_different_number_of_motors_than_the_mechanism_has
// expect-at: 6
