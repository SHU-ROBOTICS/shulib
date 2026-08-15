#pragma once
//
// ProsTickPacer — motion::ITickPacer over pros::Task::delay_until (chunk R1a):
// the ONLY seam that regains control mid-motion on the robot, replacing
// main.cpp's V5DelayPacer (which had to hand-advance a FakeClock).
//
// BINDS: pros::Task::delay_until(&prev, kTickMs) — NOT pros::delay(kTickMs).
// The difference is drift: delay(10) sleeps 10 ms from NOW, so each tick's
// processing time ADDS to the period (a 2 ms tick body makes a 12 ms loop —
// 20% slow, and the motion profiles integrate that error forever).
// delay_until wakes at prev + delta and updates prev to the WAKE instant
// (vendored rtos.hpp:742-747, HA-102), so the cadence is anchored to the
// timeline, not to the work: processing time is absorbed, and the loop runs
// at the true 100 Hz the motion layer assumes (HA-32).
//
// FIRST CALL: prev is initialized from millis() lazily on the first pace() —
// the pacer anchors to the moment pacing STARTS, not the moment the object
// was constructed (a Robot constructed at t=0 but first paced at t=3000 must
// not "catch up" 300 phantom ticks; FreeRTOS's catch-up semantics would run
// them back-to-back and the motion layer would see 300 zero-dt ticks).
//
// CADENCE: kTickMs = 10 (the 100 Hz motion tick, HA-32) — the same constant
// V5DelayPacer carried; one owner, here, until a config plumbs it.
//
// The real IClock (ProsClock) reads real time and needs no help — the
// fake-clock advance that V5DelayPacer had to do is gone, which is exactly
// the R1 note that pacer carried in main.cpp since C7.
//
// HA register: HA-102, HA-32.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/rtos.hpp"
#pragma GCC diagnostic pop

#include <cstdint>

#include "shulib/motion/motion_scheduler.hpp"

namespace shulib::hal::pros {

/// ITickPacer on the robot: blocks until the next tick boundary via pros::Task::delay_until, so
/// the tick body's own duration is ABSORBED by the wait instead of added to it. That is the whole
/// reason it is not pros::delay(kTickMs), which sleeps from NOW and would turn a 2 ms tick body
/// into a 12 ms loop — 20% slow, forever, with the motion profiles integrating the error.
/// The cadence anchors on the FIRST pace(), not at construction, so an object built long before
/// it is used does not try to catch up the ticks it "missed" while nothing was pacing.
class ProsTickPacer final : public motion::ITickPacer {
public:
    static constexpr std::uint32_t kTickMs = 10;  ///< the motion tick (HA-32's 100 Hz)

    /// Block until the next tick boundary (header: anchored cadence, lazy
    /// first-call anchor).
    void pace() override {
        const std::uint32_t now = ::pros::millis();
        if (!anchored_) {
            prevWakeMs_ = now;
            anchored_ = true;
        } else if (now - prevWakeMs_ > kTickMs) {
            // RE-ANCHOR after a tick body that overran a whole period. The lazy first-call
            // anchor above exists to stop FreeRTOS replaying missed ticks back-to-back, and
            // that hazard is not confined to construction: after a 50 ms body on a 10 ms
            // period the setpoint is already 40 ms in the past, so the next four pace() calls
            // return instantly and the motion layer sees four near-zero-dt ticks — arriving
            // precisely when the loop is already in trouble. anchored_ is set once and never
            // cleared, so nothing re-anchored mid-run. Unsigned arithmetic is deliberate: the
            // millis() wrap is modular, so `now - prevWakeMs_` stays correct across it.
            prevWakeMs_ = now;
        }
        ::pros::Task::delay_until(&prevWakeMs_, kTickMs);
    }

private:
    std::uint32_t prevWakeMs_ = 0;
    bool anchored_ = false;
};

}  // namespace shulib::hal::pros
