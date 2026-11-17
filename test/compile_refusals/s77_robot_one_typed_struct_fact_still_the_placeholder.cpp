// s77_robot_one_typed_struct_fact_still_the_placeholder: as s76 for a STRUCT fact (the gear teeth) — the
// ledger's own field-wise same() (the library's HeightAt and friends have no ==). run_refusal.py judges it.
#include "robots/robot_one_facts.hpp"
namespace shulib::bench::robot_one_facts {
inline constexpr auto kProbe = typed(Teeth{12, 60}, Teeth{12, 60}, Stage::A, "a probe fact", "the builder, counted");
ROBOT_ONE_CHECK(kProbe, "a probe fact");
}  // namespace shulib::bench::robot_one_facts
// expect: robot one: TYPED but still the placeholder
