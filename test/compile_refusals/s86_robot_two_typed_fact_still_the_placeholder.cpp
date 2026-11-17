// s86_robot_two_typed_fact_still_the_placeholder: robot two's facts file (src/robots/robot_two_facts.hpp, chunk ROB1-BT) — a fact typed EQUAL to its placeholder, its source not saying so. Robot two's check
// is robot one's, for robot two (the twin). run_refusal.py judges it.
#include "robots/robot_two_facts.hpp"
namespace shulib::bench::robot_two_facts {
inline constexpr auto kProbe = typed(1.0, 1.0, Stage::A, "a probe fact", "the builder, tape, 2026-11-16");
ROBOT_TWO_CHECK(kProbe, "a probe fact");
}  // namespace shulib::bench::robot_two_facts
// expect: robot two: TYPED but still the placeholder
