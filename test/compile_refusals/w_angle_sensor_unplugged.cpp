// w_angle_sensor_unplugged: the wrist's angle sensor not plugged in — at its wire(...) line.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(
    wire(ry::wrist, motors(6)));
// expect: the_mechanism_has_a_sensor_the_wiring_does_not_plug_in
// expect-at: 5
