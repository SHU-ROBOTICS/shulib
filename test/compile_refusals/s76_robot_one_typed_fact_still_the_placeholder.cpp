// s76_robot_one_typed_fact_still_the_placeholder: robot one's facts ledger — `owed(` flipped to `typed(` with the
// value left EQUAL to its placeholder and no word that the builder confirmed it (the ROB1 brief's R10, GM7b).
// run_refusal.py judges it.
#include "robots/robot_one_facts.hpp"
namespace shulib::bench::robot_one_facts {
inline constexpr auto kProbe = typed(1.0, 1.0, Stage::A, "a probe fact", "the builder, tape, 2026-11-14");
ROBOT_ONE_CHECK(kProbe, "a probe fact");
}  // namespace shulib::bench::robot_one_facts
// expect: robot one: TYPED but still the placeholder
