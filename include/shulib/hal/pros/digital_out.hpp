#pragma once
//
// ProsDigitalOut — IDigitalOut over pros::adi::DigitalOut (chunk R1b): the
// pneumatic solenoid line behind the HAL (F1's seam, finally on hardware).
//
// ── CONSTRUCTION IS A PHYSICAL ACTION (T3) ──────────────────────────────────────────
// pros::adi::DigitalOut drives the line the moment it is constructed — its
// ctor takes `init_state` and PROS defaults it to LOW (vendored
// adi.hpp:546-547,564,596; HA-119). On a pneumatic clamp that means THE
// CYLINDER MOVES WHEN THIS OBJECT IS BUILT. This adapter therefore REFUSES
// the default: `initialState` is a REQUIRED constructor argument, so the
// author must state what the line should be at boot — and that statement
// must AGREE with the owning PneumaticMechanism's declared safe state
// (mechanism.hpp), or there is a window at boot where the line is wrong and
// "wrong" means physically moving. A safety step a caller can forget is a
// safety step that WILL be forgotten (the legacy escapeJSONString lesson);
// this one cannot be skipped, only stated.
//
// BINDS:
//  * ctor(init_state) — the boot actuation, stated explicitly (HA-119)
//  * set_value(1/0) ← set(bool) — the one bool→int32 mapping (HA-119)
//
// ADDRESSING (T6, HA-120): one class, two constructors — the brain's own 8
// ADI ports ('a'–'h', 'A'–'H', or 1–8) or an expander's {smartPort, adiPort}.
// The seam is identical either way; where the wire lands is a construction
// fact, never a type. Whether OUR robot has an expander is UNKNOWN (R1a's
// expander report came from an out-of-range registry index — bench step).
//
// COMMAND, NOT WORLD: commanded() returns the value this adapter last
// COMMANDED (ctor's initial state until set() is called) — never a read of
// the device (digital_out.hpp:44: there IS no feedback channel; air can be
// exhausted, a linkage can bind, and the line reads what it was told either
// way). Confirmation comes from a SEPARATE sensor, by contract.
//
// SENTINELS (T7 shape, write-side): set_value() returns PROS_ERR when the
// port refuses (misconfigured/reconfigured — HA-119). The seam has no
// validity channel by design, so the refusal is EXPOSED, not raised:
// faultedWrites() counts refused writes; commanded() still reports the
// command (the caller's intent is the telemetry contract even when the wire
// refused — that is what makes a refused write VISIBLE as a divergence).
//
// HA register: HA-119, HA-120 (docs/hardware-assumptions.md).

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/adi.hpp"
#include "pros/error.h"
#pragma GCC diagnostic pop

#include <cstdint>
#include <utility>

#include "shulib/hal/digital_out.hpp"

namespace shulib::hal::pros {

/// IDigitalOut over `pros::adi::DigitalOut` — the pneumatic solenoid line, on real hardware.
/// CONSTRUCTING ONE IS A PHYSICAL ACTION: PROS drives the line from its own constructor, so on a
/// pneumatic clamp the cylinder moves the moment this object is built. That is why `initialState`
/// is a required argument with NO default — the author must state the boot level, and must state
/// one that agrees with the owning PneumaticMechanism's declared safe state, or there is a window
/// at boot where the line is wrong and "wrong" means physically moving. A write the port refuses
/// is COUNTED (faultedWrites), never raised: the seam has no validity channel by design, and
/// commanded() goes on reporting the caller's intent, which is exactly what makes a refused write
/// visible as a divergence instead of a device that silently agrees with itself.
class ProsDigitalOut final : public IDigitalOut {
public:
    /// Brain ADI port ('a'–'h', 'A'–'H', or 1–8). CONSTRUCTION DRIVES THE
    /// LINE to `initialState` — a physical action (header). `initialState`
    /// is required, no default: state what the boot level must be, and make
    /// it agree with the owning mechanism's declared safe state.
    ProsDigitalOut(std::uint8_t adiPort, bool initialState)
        : line_{adiPort, initialState}, commanded_{initialState} {}

    /// Expander form: {smartPort 1–21, adiPort as above}. Same actuating
    /// construction, same required initial state (T6: one class — where the
    /// wire lands is a construction fact, not a type).
    ProsDigitalOut(std::uint8_t smartPort, std::uint8_t adiPort, bool initialState)
        : line_{::pros::adi::ext_adi_port_pair_t{smartPort, adiPort}, initialState},
          commanded_{initialState} {}

    /// Command the line (bool → 1/0, HA-119). A refused write is counted in
    /// faultedWrites(); commanded() reports the command regardless (header).
    void set(bool value) override {
        commanded_ = value;
        if (line_.set_value(value ? 1 : 0) == PROS_ERR) {
            faultedWrites_ += 1;
        }
    }

    /// The value last COMMANDED (the ctor's initial state until set() runs) —
    /// a readback of the command, NEVER of the world (digital_out.hpp:44).
    [[nodiscard]] bool commanded() const override { return commanded_; }

    /// How many set() calls (ctor excluded) the device refused with PROS_ERR
    /// — exposure, not policy (raising stays with the loop layer).
    [[nodiscard]] int faultedWrites() const noexcept { return faultedWrites_; }

private:
    ::pros::adi::DigitalOut line_;
    bool commanded_;
    int faultedWrites_ = 0;
};

}  // namespace shulib::hal::pros
