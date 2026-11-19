// s91_robot_two_cascade_past_its_top: robot two's cascade sent 1 in past its top stop (its facts' last measured
// row) — the library refuses a constant target outside the travel limits, by name (chunk ROB1-BT, T8's twin).
// run_refusal.py judges it.
#include "robots/mechanism_table.hpp"
#include "shulib/units/literals.hpp"
using namespace shulib::units::literals;
inline constexpr auto g = shulib::bench::robot_two::cascade.to(shulib::bench::robot_two_facts::cascadeTop() + 1_in);
// expect: target_is_outside_the_travel_limits
