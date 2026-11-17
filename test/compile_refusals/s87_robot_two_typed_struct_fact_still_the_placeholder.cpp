// s87_robot_two_typed_struct_fact_still_the_placeholder: robot two's facts file (src/robots/robot_two_facts.hpp, chunk ROB1-BT) — a STRUCT fact typed equal to its placeholder. Robot two's check
// is robot one's, for robot two (the twin). run_refusal.py judges it.
#include "robots/robot_two_facts.hpp"
namespace shulib::bench::robot_two_facts {
inline constexpr auto kProbe = typed(Teeth{12, 60}, Teeth{12, 60}, Stage::A, "a probe fact", "the builder, counted");
ROBOT_TWO_CHECK(kProbe, "a probe fact");
}  // namespace shulib::bench::robot_two_facts
// expect: robot two: TYPED but still the placeholder
