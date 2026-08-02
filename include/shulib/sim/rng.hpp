#pragma once
//
// sim::Rng — the ONE random source of the host plant (chunk A2, constraint 4:
// bit-reproducible determinism).
//
// Why not <random>: the standard's ENGINES are portable, but its DISTRIBUTIONS
// (uniform_real_distribution, normal_distribution, …) are implementation-defined —
// libstdc++ and libc++ produce different streams from the same engine state. A
// "same seed → byte-identical run" guarantee built on them would silently hold on
// one standard library and not another. So the generator AND the value mappings
// are pinned here, in ~20 lines whose output is a pure function of the seed.
//
// Algorithm: SplitMix64 (Steele/Lea/Flood 2014; public domain reference constants).
// Chosen over xoshiro/PCG because it is the simplest generator that passes BigCrush,
// needs no seed-quality preconditions (any 64-bit seed, including 0, is fine — it
// is itself the recommended SEEDER for the fancier generators), and is 3 lines.
//
// Who draws from this (documented so randomness can never sneak in elsewhere):
//   * the scenario layer — seeded random command schedules for sweep tests
//     (test/sim_*_test.cpp) and, at Phase E, thousands of seeded trajectories;
//   * the A3 degradation hooks — noise/dropout draws (the DegradationModel receives
//     THIS Rng; the A2 identity model draws nothing).
// The plant itself is deterministic given its inputs; nothing else in shulib::sim
// may hold a random source. No wall-clock seeding exists anywhere (constraint 4:
// a failing run must be re-runnable exactly).
//
// Concurrency: single-task by contract, like the rest of the harness.

#include <cstdint>

namespace shulib::sim {

class Rng {
public:
    /// Any 64-bit value is a valid seed (SplitMix64 has no weak seeds; 0 is fine).
    explicit constexpr Rng(std::uint64_t seed) noexcept : state_{seed} {}

    /// Next raw 64-bit draw (the SplitMix64 reference sequence for this seed).
    [[nodiscard]] constexpr std::uint64_t nextU64() noexcept {
        state_ += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = state_;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }

    /// Uniform double in [0, 1): the top 53 bits scaled by 2^-53 — the standard
    /// exact mapping (every result is a representable double; no rounding bias).
    [[nodiscard]] constexpr double nextUnit() noexcept {
        return static_cast<double>(nextU64() >> 11) * 0x1.0p-53;
    }

    /// Uniform double in [lo, hi). Caller guarantees lo <= hi (a harness-side
    /// convenience, not a validated API — this is test scaffolding, not the core).
    [[nodiscard]] constexpr double uniform(double lo, double hi) noexcept {
        return lo + (hi - lo) * nextUnit();
    }

private:
    std::uint64_t state_;
};

}  // namespace shulib::sim
