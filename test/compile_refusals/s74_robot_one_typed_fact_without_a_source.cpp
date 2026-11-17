// s74_robot_one_typed_fact_without_a_source: robot one's facts ledger (src/robots/robot_one_facts.hpp) — a fact
// typed with an EMPTY source. A typed fact says who measured it, how and when (the ROB1 brief's D3, T2, GM6).
// run_refusal.py judges it.
#include "robots/robot_one_facts.hpp"
namespace shulib::bench::robot_one_facts {
inline constexpr auto kProbe = typed(1.0, 2.0, Stage::A, "a probe fact", "");
ROBOT_ONE_CHECK(kProbe, "a probe fact");
}  // namespace shulib::bench::robot_one_facts
// expect: robot one: a typed fact needs a real source
