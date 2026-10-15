// hb_name_differs: HAND-BUILT (kNameDiffers): the wiring names another mechanism — mechanisms() holds an entry built by hand to wire()'s rules.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(hb::kNameDiffers);
// expect: the_wiring_names_another_mechanism_than_its_declaration
