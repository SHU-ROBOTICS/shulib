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
inline void resetTime() { timeState() = TimeState{}; }
/// Advance the shim clock (µs) — stands in for real time passing.
inline void advanceUs(std::uint64_t us) { timeState().nowUs += us; }
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
            s.nowUs = targetUs;  // "block" until the wake instant
        }
        *prev_time = static_cast<std::uint32_t>(targetMs);
    }
};

}  // namespace pros
