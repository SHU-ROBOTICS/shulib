// s88_robot_two_check_misquotes_its_fact: robot two's facts file (src/robots/robot_two_facts.hpp, chunk ROB1-BT) — a check whose words are not its fact's name, verbatim. Robot two's check
// is robot one's, for robot two (the twin). run_refusal.py judges it.
#include "robots/robot_two_facts.hpp"
namespace shulib::bench::robot_two_facts {
inline constexpr auto kProbe = owed(1.0, Stage::A, "a probe fact");
ROBOT_TWO_CHECK(kProbe, "a probe fakt");
}  // namespace shulib::bench::robot_two_facts
// expect: robot two: the check below a fact must quote its name verbatim
