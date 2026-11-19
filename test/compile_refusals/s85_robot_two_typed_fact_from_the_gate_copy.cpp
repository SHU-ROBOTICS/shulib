// s85_robot_two_typed_fact_from_the_gate_copy: robot two's facts file (src/robots/robot_two_facts.hpp, chunk ROB1-BT) — a fact typed with the src build gate's COPY source. Robot two's check
// is robot one's, for robot two (the twin). run_refusal.py judges it.
#include "robots/robot_two_facts.hpp"
namespace shulib::bench::robot_two_facts {
inline constexpr auto kProbe = typed(1.0, 2.0, Stage::A, "a probe fact", "gate copy -- same as the placeholder, checked");
ROBOT_TWO_CHECK(kProbe, "a probe fact");
}  // namespace shulib::bench::robot_two_facts
// expect: robot two: a typed fact needs a real source
