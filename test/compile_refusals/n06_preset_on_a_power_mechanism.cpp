// n06_preset_on_a_power_mechanism: a preset on a power mechanism (it has none; the error is ON this line).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::intake["open"];
// expect: a_power_mechanism_has_no_presets_run_it_with_r_run
// expect-at: 4
