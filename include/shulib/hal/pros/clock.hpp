#pragma once
//
// ProsClock — IClock over the V5's real time (chunk R1a).
//
// BINDS: pros::micros() (uint64 µs since program start — HA-101), NOT
// pros::millis(). millis() quantizes to 1 ms, which is 10% of the 10 ms
// control tick: PID derivative terms and profile timing divide by dt, so a
// 10%-quantized dt is a 10% error in every derivative term — and the host
// plant has never seen it, because FakeClock is exact. micros() costs nothing
// and removes the whole question (brief T6; the rejected alternative was
// millis() on "the tick is 10 ms so ms is enough", which conflates the tick
// period with the measurement of it).
//
// CONVERTS: µs → canonical seconds, ×1e-6, exactly once, here (clock.hpp:9-10:
// "the V5's milliseconds are converted to seconds exactly once, in the
// hal/pros adapter" — this adapter converts microseconds instead, same rule).
//
// MONOTONICITY (clock.hpp:12): micros() is an unsigned counter from boot;
// uint64 µs wraps after ~584,000 years, so within any run now() never
// decreases. (millis()'s uint32 would wrap at ~49.7 days — irrelevant to a
// match, but the reason the wrap is stated rather than silently assumed.)
//
// Epoch: program start (whatever micros() counts from) — IClock requires "a
// fixed per-run epoch", not a particular zero.
//
// DELIBERATELY NOT here: no tick pacing (tick_pacer.hpp owns that), no
// timeouts, no dt bookkeeping — one seam, one job.
//
// HA register: HA-101 (docs/hardware-assumptions.md). PROVISIONAL until the
// bench session confirms micros() advances ~1:1 with wall time.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/rtos.hpp"
#pragma GCC diagnostic pop

#include "shulib/hal/clock.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::pros {

class ProsClock final : public IClock {
public:
    /// Seconds since program start. Monotonic non-decreasing (header note).
    [[nodiscard]] units::Time now() const override {
        return units::Time{static_cast<double>(::pros::micros()) * 1e-6};
    }
};

}  // namespace shulib::hal::pros
