// s75_robot_one_typed_fact_from_the_gate_copy: robot one's facts ledger — a fact typed with the src build gate's
// COPY marker as its source. Only the gate's throwaway copy under build/src-gate-facts/ may carry it (it turns
// this check off there); the repository's ledger refuses it at make, so a copied-back ledger can never link
// (the ROB1 brief's R4, GM7c). run_refusal.py judges it.
#include "robots/robot_one_facts.hpp"
namespace shulib::bench::robot_one_facts {
inline constexpr auto kProbe = typed(1.0, 2.0, Stage::A, "a probe fact", "gate copy -- same as the placeholder, checked");
ROBOT_ONE_CHECK(kProbe, "a probe fact");
}  // namespace shulib::bench::robot_one_facts
// expect: robot one: a typed fact needs a real source
