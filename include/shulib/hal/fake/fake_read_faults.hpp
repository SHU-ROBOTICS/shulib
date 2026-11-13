#pragma once
//
// FakeReadFaults — the host's hal::IReadFaults (chunk MECH-A1, 2026-10-08).
//
// A test's settable read-fault count. `failing(true)` models an adapter whose device has stopped
// answering, so that EVERY read it is asked for is screened: each faultedReads() call while failing
// counts one more, as each PROS read of a dead device would (the consumer — the mechanism service —
// reads the count once per sample, so "a sample while failing" is "the count moved"). `failing(false)`
// is the device answering again: the count stops moving and keeps its total (it is monotonic, like
// every adapter's).
//
// The F4 fakes (FakeDistance, FakeOptical, FakeDigitalIn) stay UNMODIFIED: a test pairs one of them
// with one of these, which is exactly how hal::fake::FakeDeviceProvider serves a piece sensor and
// its count. Host-test infrastructure; PROS-free, allocation-free.

#include "shulib/hal/read_faults.hpp"

namespace shulib::hal::fake {

/// A settable read-fault count (header): while `failing`, every faultedReads() call counts one more
/// read screened — the adapter's own behaviour while its device does not answer.
class FakeReadFaults final : public IReadFaults {
public:
    /// The device stops answering (true) or answers again (false). Changes nothing by itself: the
    /// count moves only when it is next read while failing.
    void failing(bool f) noexcept { failing_ = f; }
    /// Whether the device is currently not answering.
    [[nodiscard]] bool isFailing() const noexcept { return failing_; }
    /// Reads screened so far — one more on each call while failing (header); never decreases.
    [[nodiscard]] int faultedReads() const noexcept override {
        if (failing_) {
            ++count_;
        }
        return count_;
    }

private:
    bool failing_ = false;
    mutable int count_ = 0;
};

}  // namespace shulib::hal::fake
