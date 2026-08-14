#pragma once
//
// ProsDigitalIn — IDigitalIn over pros::adi::DigitalIn (chunk R1b): a limit
// switch / bumper line behind the HAL.
//
// BINDS:
//  * get_value() [int32 level, 1/0; HA-121] → state() — a LEVEL, deliberately.
//    This adapter NEVER calls get_new_press(): PROS's ADI edge detection
//    CONSUMES the press when read ("1 if the button is pressed and had not
//    been pressed the last time this function was called", vendored
//    adi.hpp:711-712), so with two consumers one silently loses — the exact
//    trap the controller adapter already rules on (HA-104; HA-121 is its ADI
//    sibling). Per-consumer edge detection lives ABOVE the seam in
//    hal::ButtonEdge (controller.hpp), one instance per consumer. The fence
//    guard test greps this file to keep the forbidden binding structurally
//    absent.
//
// ADDRESSING (T6, HA-120): one class, two constructors — brain ADI port
// ('a'–'h', 'A'–'H', or 1–8) or an expander's {smartPort, adiPort} — same as
// ProsDigitalOut, same reasoning: the seam is identical, so where the wire
// lands is a construction fact, never a type.
//
// SENTINELS (T7): the seam has no validity channel (digital_in.hpp — a dead
// ADI port is indistinguishable from a working one). get_value() returns
// PROS_ERR when the port refuses (HA-121), and PROS_ERR != 0, so an
// UNSCREENED binding would read a dead port as PRESSED — a homing switch
// permanently "pressed" means a lift that believes it is homed while it
// climbs into the hard stop. Screened: hold the last good level, count in
// faultedReads(). Never map to false either (a hard "released" is as wrong
// as a hard "pressed"; last-good is what the consumer's own cross-checks —
// homing travel limits — are designed around).
//
// DELIBERATELY NOT here: debouncing (the seam's own ruling — a filter
// constant belongs to the consumer that knows what the switch is for) and
// any homing routine (F3's, season content the students author).
//
// HA register: HA-120, HA-121 (docs/hardware-assumptions.md).

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/adi.hpp"
#include "pros/error.h"
#pragma GCC diagnostic pop

#include <cstdint>
#include <utility>

#include "shulib/hal/digital_in.hpp"

namespace shulib::hal::pros {

class ProsDigitalIn final : public IDigitalIn {
public:
    /// Brain ADI port ('a'–'h', 'A'–'H', or 1–8).
    explicit ProsDigitalIn(std::uint8_t adiPort) : line_{adiPort} {}

    /// Expander form: {smartPort 1–21, adiPort as above}.
    ProsDigitalIn(std::uint8_t smartPort, std::uint8_t adiPort)
        : line_{::pros::adi::ext_adi_port_pair_t{smartPort, adiPort}} {}

    /// The raw level via get_value() — NEVER the consuming get_new_press()
    /// (header). PROS_ERR is screened to the last good level (T7).
    [[nodiscard]] bool state() const override {
        const std::int32_t raw = line_.get_value();
        if (raw == PROS_ERR) {
            faultedReads_ += 1;
            return lastState_;
        }
        lastState_ = raw != 0;
        return lastState_;
    }

    /// How many reads were screened to last-good (T7 observability).
    [[nodiscard]] int faultedReads() const noexcept { return faultedReads_; }

private:
    ::pros::adi::DigitalIn line_;
    // `mutable`: state() is const but the T7 hold-last-good screen must
    // remember the level — the cache is part of the read, not commanded state.
    mutable bool lastState_ = false;
    mutable int faultedReads_ = 0;
};

}  // namespace shulib::hal::pros
