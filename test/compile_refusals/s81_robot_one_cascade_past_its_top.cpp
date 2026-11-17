// s81_robot_one_cascade_past_its_top: robot one's cascade sent past its top stop — the top row of its measured
// rows (src/robots/robot_one_facts.hpp) — refused at build by the library (the ROB1 brief's T8). On the
// placeholders the top is 40 in. run_refusal.py judges it.
#include "robots/mechanism_table.hpp"
#include "shulib/units/literals.hpp"
using namespace shulib::units::literals;
inline constexpr auto g = shulib::bench::robot_one::cascade.to(shulib::bench::robot_one_facts::cascadeTop() + 1_in);
// expect: target_is_outside_the_travel_limits
