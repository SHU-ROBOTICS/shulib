// n33_preset_on_a_power: a preset on a power mechanism (it has no presets) — the declaration's own refusal.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr mech::Control k = preset(rx::intake["fast"], Button::Up, 1_s);
// expect: a_power_mechanism_has_no_presets_run_it_with_r_run
// expect-at: 5
