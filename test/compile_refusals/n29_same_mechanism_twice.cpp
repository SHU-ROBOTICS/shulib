// n29_same_mechanism_twice: the same mechanism wired twice.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::arm, motors(1)), wire(rx::arm, motors(2)));
// expect: the_same_mechanism_is_wired_twice
