#pragma once
//
// FakeDigitalIn — a deterministic IDigitalIn for host tests. Like
// FakeDigitalOut, there is nothing to model (the real seam is one raw level
// with no validity channel — digital_in.hpp), so the fake is the contract plus
// the bookkeeping a test can assert on: the injectable level and how many
// times state() was read (a consumer that polls every tick and one that
// forgot to poll at all look identical in the level alone; the count tells
// them apart — the same reason FakeDigitalOut counts set() calls).

#include "shulib/hal/digital_in.hpp"

namespace shulib::hal::fake {

class FakeDigitalIn final : public IDigitalIn {
public:
    [[nodiscard]] bool state() const override {
        ++reads_;
        return state_;
    }

    /// Drive the fake's level (what a physical press/release would do).
    void setState(bool value) noexcept { state_ = value; }

    // --- test observation ---
    /// Total state() reads since construction.
    [[nodiscard]] int readCount() const noexcept { return reads_; }

private:
    bool state_ = false;
    // `mutable`: state() is const (a read), but the read-count observation must
    // still tally — the counter is part of the observation, not device state.
    mutable int reads_ = 0;
};

}  // namespace shulib::hal::fake
