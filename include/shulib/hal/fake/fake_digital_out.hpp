#pragma once
//
// FakeDigitalOut — a deterministic IDigitalOut for host tests. There is nothing
// to model (the real seam has no feedback — digital_out.hpp), so the fake is
// the contract plus bookkeeping a test can assert on: the commanded value and
// how many times set() was called (an idempotent re-command and a toggle storm
// look identical in `commanded()` alone; the count tells them apart).

#include "shulib/hal/digital_out.hpp"

namespace shulib::hal::fake {

class FakeDigitalOut final : public IDigitalOut {
public:
    void set(bool value) override {
        commanded_ = value;
        ++sets_;
    }

    [[nodiscard]] bool commanded() const override { return commanded_; }

    // --- test observation ---
    /// Total set() calls since construction (any value).
    [[nodiscard]] int setCount() const noexcept { return sets_; }

private:
    bool commanded_ = false;
    int sets_ = 0;
};

}  // namespace shulib::hal::fake
