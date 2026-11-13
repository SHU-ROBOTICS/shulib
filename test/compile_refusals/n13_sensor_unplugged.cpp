// n13_sensor_unplugged: the lift's home switch not plugged in — at its wire(...) line.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(
    wire(rx::lift, motors(2, 3)));
// expect: the_mechanism_has_a_sensor_the_wiring_does_not_plug_in
// expect-at: 5
