#pragma once
//
// FakeController — a deterministic IController for host tests. It stores
// CANONICAL values (the −127…127 → [-1,1] conversion is the hal/pros adapter's
// job, tested separately through controller_conversion.hpp) and lets a test
// script stick deflections, button levels and connection drops — including the
// disconnected-reads-zero behaviour a real controller shows (HA-103), so the
// teleop layer's isConnected() gating is exercised off-robot.

#include <array>

#include "shulib/hal/controller.hpp"

namespace shulib::hal::fake {

class FakeController final : public IController {
public:
    [[nodiscard]] double axis(ControllerAxis a) const override {
        // Mirror the real device: a disconnected controller reads 0 on every
        // channel (HA-103) — so a test that forgets isConnected() sees exactly
        // what the robot would.
        return connected_ ? axes_[static_cast<std::size_t>(a)] : 0.0;
    }

    [[nodiscard]] bool pressed(ControllerButton b) const override {
        return connected_ && buttons_[static_cast<std::size_t>(b)];
    }

    [[nodiscard]] bool isConnected() const override { return connected_; }

    // --- test injection ---
    void setAxis(ControllerAxis a, double value) { axes_[static_cast<std::size_t>(a)] = value; }
    void setPressed(ControllerButton b, bool down) {
        buttons_[static_cast<std::size_t>(b)] = down;
    }
    void setConnected(bool connected) { connected_ = connected; }

private:
    std::array<double, 4> axes_{};     // canonical [-1, 1]
    std::array<bool, 12> buttons_{};
    bool connected_ = true;  // a fresh fake is connected by default (cf. FakeImu ready)
};

}  // namespace shulib::hal::fake
