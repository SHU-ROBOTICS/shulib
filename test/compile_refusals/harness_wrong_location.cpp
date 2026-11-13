// harness_wrong_location: SELF-TEST (must FAIL): refused for its stated reason, but its `expect-at` names a line GCC does not cite (the list's THIRD line; GCC cites its first).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable mm = mechanisms(
    wire(rx::arm, motors(1)),
    wire(rx::flywheel, motors(4)),
    wire(r2::clawPivot6, motors(4)));
// expect: smart_port_4_is_used_twice
// expect-at: 6
