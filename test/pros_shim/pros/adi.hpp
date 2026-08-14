#pragma once
//
// HOST SHIM for <pros/adi.hpp> — programmable pros::adi::DigitalOut /
// DigitalIn over a shared per-line state.
//
// Beliefs modeled (each registered in docs/hardware-assumptions.md):
//  * CONSTRUCTION ACTUATES: DigitalOut's ctor takes `bool init_state = LOW`
//    and drives the line at construction (vendored adi.hpp:546-547,564,596 —
//    "The initial state for the port"; HA-119). The shim faithfully keeps
//    PROS's dangerous default AND records every driven value in a per-line
//    history, so a test can SEE the boot glitch an adapter with the wrong
//    initial state would cause on a real cylinder.
//  * set_value(int32) drives "the digital value (1 or 0) of a pin"; returns
//    1 on success, PROS_ERR if the port is not configured as a digital
//    output (vendored adi.hpp:598-610; HA-119).
//  * ADI addressing has TWO forms, one device either way: the brain's own
//    8 ports as 1–8 or 'a'–'h' or 'A'–'H', and an expander's as
//    {smart_port, adi_port} (vendored adi.hpp:59,62,91-93; HA-120). The shim
//    normalizes letters to numbers, so an adapter that fails to support
//    either spelling fails its test. Whether OUR robot has an expander is
//    UNKNOWN (HA-120 — R1a's expander report came from an out-of-range
//    registry index).
//  * DigitalIn::get_value() is a level, 1 or 0; PROS_ERR when the port is
//    not configured as a digital input (vendored adi.hpp:732-757; HA-121).
//  * DigitalIn::get_new_press() CONSUMES the press (vendored adi.hpp:697-712:
//    "1 if the button is pressed and had NOT been pressed the last time this
//    function was called") — modeled with REAL per-line consume state, so an
//    adapter mis-bound to it starves a second consumer in the test on real
//    behaviour, not on a counter alone (HA-121, the ADI sibling of HA-104).
//
// HONEST LIMIT: this shim tests the adapter against OUR BELIEF about PROS; it
// cannot test the belief. Hardware tests the belief (bench runbook).

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include "pros/error.h"

namespace pros {
namespace adi {

/// Vendored adi.hpp:38 — {smart_port, adi_port} for expander-hosted lines.
using ext_adi_port_pair_t = std::pair<std::uint8_t, std::uint8_t>;

}  // namespace adi

namespace shim {

struct AdiLineState {
    std::int32_t value = 0;             ///< current line level (driven or sensed)
    bool broken = false;                ///< true → reads/writes return PROS_ERR
    std::vector<std::int32_t> writes;   ///< EVERY value driven, ctor included, in order
    bool newPressPrev = false;          ///< the consuming edge state (per LINE)
    int newPressCalls = 0;              ///< the forbidden binding, counted (HA-121)
};

/// 'a'–'h' / 'A'–'H' → 1–8; numbers pass through (vendored adi.hpp:59: the
/// three spellings name the same 8 ports).
inline std::uint8_t adiIndex(std::uint8_t adiPort) {
    if (adiPort >= 'a' && adiPort <= 'h') {
        return static_cast<std::uint8_t>(adiPort - 'a' + 1);
    }
    if (adiPort >= 'A' && adiPort <= 'H') {
        return static_cast<std::uint8_t>(adiPort - 'A' + 1);
    }
    return adiPort;
}

/// smart 0 = the brain's own ADI; 1–21 = an expander on that smart port.
inline std::array<std::array<AdiLineState, 9>, 22>& adiLines() {
    static std::array<std::array<AdiLineState, 9>, 22> lines{};
    return lines;
}
inline AdiLineState& adiLine(std::uint8_t smartPort, std::uint8_t adiPort) {
    return adiLines()[smartPort][adiIndex(adiPort)];
}
inline void resetAdi() {
    for (auto& smart : adiLines()) {
        for (auto& line : smart) {
            line = {};
        }
    }
}

}  // namespace shim

namespace adi {

class DigitalOut {
public:
    /// Vendored adi.hpp:564: `init_state = LOW` — the shim keeps PROS's real
    /// (dangerous) default; CONSTRUCTION DRIVES THE LINE (HA-119).
    explicit DigitalOut(std::uint8_t adi_port, bool init_state = false)
        : smart_{0}, adi_{adi_port} {
        (void)set_value(init_state ? 1 : 0);
    }

    /// Vendored adi.hpp:596 — expander form, same actuating ctor.
    explicit DigitalOut(ext_adi_port_pair_t port_pair, bool init_state = false)
        : smart_{port_pair.first}, adi_{port_pair.second} {
        (void)set_value(init_state ? 1 : 0);
    }

    /// 1 on success, PROS_ERR if the port refuses (HA-119). Every accepted
    /// value lands in the line's write history.
    std::int32_t set_value(std::int32_t value) const {
        auto& line = shim::adiLine(smart_, adi_);
        if (line.broken) {
            return PROS_ERR;
        }
        line.value = value;
        line.writes.push_back(value);
        return 1;
    }

private:
    std::uint8_t smart_;
    std::uint8_t adi_;
};

class DigitalIn {
public:
    explicit DigitalIn(std::uint8_t adi_port) : smart_{0}, adi_{adi_port} {}
    explicit DigitalIn(ext_adi_port_pair_t port_pair)
        : smart_{port_pair.first}, adi_{port_pair.second} {}

    /// The level, 1 or 0; PROS_ERR when the port refuses (HA-121).
    std::int32_t get_value() const {
        const auto& line = shim::adiLine(smart_, adi_);
        if (line.broken) {
            return PROS_ERR;
        }
        return line.value != 0 ? 1 : 0;
    }

    /// The CONSUMING edge read (HA-121): 1 only for the FIRST caller after a
    /// low→high transition — the next caller gets 0. Counted, so tests can
    /// also pin "the adapter never calls this at all".
    std::int32_t get_new_press() const {
        auto& line = shim::adiLine(smart_, adi_);
        line.newPressCalls += 1;
        if (line.broken) {
            return PROS_ERR;
        }
        const bool now = line.value != 0;
        const bool rising = now && !line.newPressPrev;
        line.newPressPrev = now;
        return rising ? 1 : 0;
    }

private:
    std::uint8_t smart_;
    std::uint8_t adi_;
};

}  // namespace adi
}  // namespace pros
