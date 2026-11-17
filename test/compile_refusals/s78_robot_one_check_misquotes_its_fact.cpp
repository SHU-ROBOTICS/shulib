// s78_robot_one_check_misquotes_its_fact: robot one's facts ledger — the check below a fact quoting OTHER words
// than the fact's name: its OWED line would then name something else (GCC names the static_assert's message,
// never the fact — the ROB1 brief's landmine 3, GM7). run_refusal.py judges it.
#include "robots/robot_one_facts.hpp"
namespace shulib::bench::robot_one_facts {
inline constexpr auto kProbe = owed(1.0, Stage::A, "a probe fact");
ROBOT_ONE_CHECK(kProbe, "a probe fakt");
}  // namespace shulib::bench::robot_one_facts
// expect: robot one: the check below a fact must quote its name verbatim
