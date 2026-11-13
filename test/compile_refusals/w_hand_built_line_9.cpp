// w_hand_built_line_9: a hand-built entry on 3-wire line 9.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenWiring(wire(rx::hood, lines('C', 'D')), [](mech::MechanismWiring& w) { w.lines[0] = static_cast<mech::Line>(9); }));
// expect: a_three_wire_port_is_not_a_letter_from_A_to_H_or_more_than_two_lines
