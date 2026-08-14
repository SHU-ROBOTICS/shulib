#pragma once
//
// FakeMechanism — a deterministic IMechanism for host tests: it implements the
// minimal virtual surface directly (no motors, no lines) and counts safe-state
// applications. Two jobs:
//   * prove the interface is implementable with NO actuator at all — the H2
//     (hal/sim over VexBuilder joints) shape, where a "mechanism" may be a
//     physics joint rather than a device group;
//   * give F2's park guard a pure test double: "did applySafeState() reach
//     every mechanism, exactly once each" is a count assertion here, with no
//     device state to reason about.
// The claim token needs no faking — it is concrete in the base by design
// (mechanism.hpp: no implementation can get it wrong).

#include "shulib/hal/mechanism.hpp"

namespace shulib::hal::fake {

class FakeMechanism final : public IMechanism {
public:
    explicit FakeMechanism(const char* mechName = "fake-mech") : name_{mechName} {}

    void applySafeState() override { ++safeStateApplications_; }

    [[nodiscard]] const char* name() const noexcept override { return name_; }

    // --- test observation ---
    /// How many times applySafeState() has been commanded.
    [[nodiscard]] int safeStateApplications() const noexcept { return safeStateApplications_; }

private:
    const char* name_;
    int safeStateApplications_ = 0;
};

}  // namespace shulib::hal::fake
