#pragma once
//
// HOST SHIM for <pros/rtos.hpp> — programmable time for adapter tests.
//
// Beliefs modeled (each registered in docs/hardware-assumptions.md):
//  * millis() is uint32 ms since program start (vendored rtos.h:223; HA-101)
//  * micros() is uint64 µs since program start (vendored rtos.h:241; HA-101)
//  * Task::delay_until(prev, delta) wakes at *prev + delta and updates *prev to
//    the wake instant — constant-cadence pacing (vendored rtos.hpp:737-760; HA-102)
//
// THE ADVANCE HOOK (chunk CAL1, Part A1, 2026-10-08): a test may install
// shim::advanceHook(), called with the clock's value before and after every
// advanceUs() — so after every delay(), and after every Task::delay_until() that
// blocks — so a host test plant can step in step with a program's own waits,
// whichever of the two it paces with (the library's ProsTickPacer, which a
// program's 10 ms loop runs on, waits with delay_until). It is a TEST seam, not a
// PROS belief: with no hook installed (the default, and what resetTime()
// restores) every call here behaves exactly as it did before the hook existed —
// delay_until's wait is advanceUs(wake − now), which sets the clock to the wake
// instant as the direct assignment did, and a wake instant already past still
// moves nothing (and so calls no hook).
//
// HONEST LIMIT: this shim tests the adapter against OUR BELIEF about PROS; it
// cannot test the belief. Hardware tests the belief (bench runbook).

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

#include <cstdint>

namespace pros {

namespace shim {
/// Programmable clock state, shared by millis()/micros()/delay/delay_until.
struct TimeState {
    std::uint64_t nowUs = 0;
    int delayCalls = 0;
    int delayUntilCalls = 0;
    std::uint32_t lastDelayUntilDelta = 0;
};
inline TimeState& timeState() {
    static TimeState s;
    return s;
}
/// CAL1: called with the clock's value before and after every advanceUs() — every delay(), every
/// delay_until() that blocks (null = none, the default) — header. A host test steps a plant in it;
/// it must not call advanceUs(), delay() or delay_until() itself.
using AdvanceHook = void (*)(std::uint64_t fromUs, std::uint64_t toUs);
inline AdvanceHook& advanceHook() {
    static AdvanceHook h = nullptr;
    return h;
}
/// Reset the clock and its counters, and uninstall any advance hook.
inline void resetTime() {
    timeState() = TimeState{};
    advanceHook() = nullptr;
}
/// Advance the shim clock (µs) — stands in for real time passing — then call the advance hook,
/// if one is installed.
inline void advanceUs(std::uint64_t us) {
    const std::uint64_t from = timeState().nowUs;
    timeState().nowUs += us;
    if (advanceHook() != nullptr) {
        advanceHook()(from, timeState().nowUs);
    }
}
}  // namespace shim

inline std::uint32_t millis() {
    return static_cast<std::uint32_t>(shim::timeState().nowUs / 1000u);
}

inline std::uint64_t micros() { return shim::timeState().nowUs; }

inline void delay(const std::uint32_t milliseconds) {
    shim::timeState().delayCalls += 1;
    shim::advanceUs(static_cast<std::uint64_t>(milliseconds) * 1000u);
}

class Task {
public:
    static void delay(const std::uint32_t milliseconds) { ::pros::delay(milliseconds); }

    /// Belief (vendored rtos.hpp:742-747): "The task will be woken up at the
    /// time *prev_time + delta, and *prev_time will be updated to reflect the
    /// time at which the task will unblock." If the wake instant is already in
    /// the past the call does not block, but *prev_time still advances — the
    /// FreeRTOS xTaskDelayUntil catch-up semantics.
    static void delay_until(std::uint32_t* const prev_time, const std::uint32_t delta) {
        auto& s = shim::timeState();
        s.delayUntilCalls += 1;
        s.lastDelayUntilDelta = delta;
        const std::uint64_t targetMs =
            static_cast<std::uint64_t>(*prev_time) + static_cast<std::uint64_t>(delta);
        const std::uint64_t targetUs = targetMs * 1000u;
        if (targetUs > s.nowUs) {
            // "block" until the wake instant — through advanceUs(), so an installed advance
            // hook sees this wait as it sees a delay() (CAL1); with none, the clock is set to
            // the wake instant exactly as before.
            shim::advanceUs(targetUs - s.nowUs);
        }
        *prev_time = static_cast<std::uint32_t>(targetMs);
    }
};

}  // namespace pros
