#pragma once
//
// FakeDeviceRegistry — a programmable port census for host tests. The boot check
// (calibration/boot_check.hpp) is entirely a function of "what does the registry say about
// each port", so this fake is how every one of its verdicts gets exercised: a port that lost
// its motor, a port that now carries a rotation sensor, an IMU that moved, a motor that
// appeared where the description has none.
//
// ADVERSARIAL DEFAULT: every port is EMPTY. A test that forgets to populate the registry
// therefore sees a drivetrain with no motors at all and gets a refusal — never an accidental
// pass. (The shim's no-card default is the same idea, for the same reason.)
//
// Host-test only in spirit; it is header-only, allocation-free and would compile anywhere.

#include <cstddef>

#include "shulib/core/check.hpp"
#include "shulib/hal/device_registry.hpp"

namespace shulib::hal::fake {

class FakeDeviceRegistry final : public IDeviceRegistry {
public:
    /// What PHYSICAL port `port` carries. Out of range answers None, exactly as the real
    /// adapter must — so a test can pin the boundary against the fake as well.
    [[nodiscard]] DeviceKind pluggedKind(int port) const noexcept override {
        ++queries_;
        if (!isSmartPort(port)) {
            return DeviceKind::None;
        }
        return kinds_[static_cast<std::size_t>(port)];
    }

    /// Put `kind` on PHYSICAL port `port` (1..kMaxSmartPorts). Precondition: a real port —
    /// a test that programs port 0 or 22 is describing a robot that cannot exist, and
    /// silently ignoring it would make the test prove nothing.
    void setKind(int port, DeviceKind kind) {
        SHULIB_PRECONDITION(isSmartPort(port),
                            "FakeDeviceRegistry::setKind: port must be a physical port 1..21");
        kinds_[static_cast<std::size_t>(port)] = kind;
    }

    /// Put `kind` on every port of `ports` (a convenience for "these ten ports carry motors").
    template <typename Range>
    void setKinds(const Range& ports, DeviceKind kind) {
        for (const auto p : ports) {
            setKind(static_cast<int>(p < 0 ? -p : p), kind);
        }
    }

    /// Empty every port (the adversarial default).
    void clear() noexcept {
        for (auto& k : kinds_) {
            k = DeviceKind::None;
        }
        queries_ = 0;
    }

    /// How many pluggedKind() calls have been served — the "the boot check asks once per
    /// port" claim, and the way a test proves an out-of-range port never reached the platform.
    [[nodiscard]] int queries() const noexcept { return queries_; }

private:
    /// Indexed by PHYSICAL port; slot 0 exists so the index needs no arithmetic and is never
    /// returned (isSmartPort screens it).
    DeviceKind kinds_[kMaxSmartPorts + 1]{};
    mutable int queries_ = 0;
};

}  // namespace shulib::hal::fake
