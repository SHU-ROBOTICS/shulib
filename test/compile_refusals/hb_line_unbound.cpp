// hb_line_unbound: HAND-BUILT (kLineUnbound): a line role unbound — mechanisms() holds an entry built by hand to wire()'s rules.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(hb::kLineUnbound);
// expect: the_wiring_gives_a_different_number_of_lines_than_the_mechanism_has
