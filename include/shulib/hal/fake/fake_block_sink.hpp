#pragma once
//
// FakeBlockSink — records every byte written through the IBlockSink seam so host tests
// can assert a blackbox file's EXACT bytes (byte-exact goldens, round trips through the
// decoder). The binary equivalent of FakeCharSink. Host-test only (uses std::vector).
//
// It also MODELS THE FAILURE THAT MATTERS: setCapacity(n) makes the device accept only
// the first n bytes of the run and then report failure, keeping the prefix — exactly
// what a full/yanked/dying SD card leaves behind. That is how the truncated-file case
// gets tested, and the truncated case is the one that will actually occur in the field.

#include <cstddef>
#include <span>
#include <vector>

#include "shulib/hal/block_sink.hpp"

namespace shulib::hal::fake {

class FakeBlockSink final : public IBlockSink {
public:
    /// Accept the bytes (up to the configured capacity) and remember them in order.
    /// Returns false once the capacity cuts a write short — the partial prefix is
    /// KEPT, which is what a dying device leaves on the card.
    [[nodiscard]] bool write(std::span<const std::byte> bytes) noexcept override {
        ++writeCalls_;
        const std::size_t room = capacity_ - (bytes_.size() < capacity_ ? bytes_.size() : capacity_);
        const std::size_t take = bytes.size() < room ? bytes.size() : room;
        try {
            bytes_.insert(bytes_.end(), bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(take));
        } catch (...) {
            return false;  // the seam is noexcept; an allocation failure is a device failure
        }
        if (take < bytes.size()) {
            ++shortWrites_;
            return false;
        }
        return true;
    }

    /// Count the flush and report the configured outcome.
    bool flush() noexcept override {
        ++flushCalls_;
        return flushSucceeds_;
    }

    /// Cap the total bytes this device will ever accept (default: unbounded). The
    /// write that crosses the cap keeps its prefix and returns false.
    void setCapacity(std::size_t bytes) noexcept { capacity_ = bytes; }

    /// Make flush() report failure (a device that accepted bytes but lost them).
    void setFlushSucceeds(bool ok) noexcept { flushSucceeds_ = ok; }

    /// Everything accepted so far, verbatim and in order.
    [[nodiscard]] const std::vector<std::byte>& bytes() const noexcept { return bytes_; }
    /// The accepted bytes as a span, for handing straight to the decoder.
    [[nodiscard]] std::span<const std::byte> view() const noexcept { return bytes_; }
    /// How many bytes the device holds.
    [[nodiscard]] std::size_t size() const noexcept { return bytes_.size(); }
    /// True when nothing has been accepted (the "no fault, nothing written" claim).
    [[nodiscard]] bool empty() const noexcept { return bytes_.empty(); }
    /// How many write() calls the sink made — the IO-count claim in the cost tests.
    [[nodiscard]] int writeCalls() const noexcept { return writeCalls_; }
    /// How many write() calls were cut short by the capacity.
    [[nodiscard]] int shortWrites() const noexcept { return shortWrites_; }
    /// How many flush() calls the sink made.
    [[nodiscard]] int flushCalls() const noexcept { return flushCalls_; }

    /// Forget everything (bytes and counters); the capacity setting is kept.
    void clear() noexcept {
        bytes_.clear();
        writeCalls_ = 0;
        shortWrites_ = 0;
        flushCalls_ = 0;
    }

private:
    std::vector<std::byte> bytes_;
    std::size_t capacity_ = static_cast<std::size_t>(-1);
    int writeCalls_ = 0;
    int shortWrites_ = 0;
    int flushCalls_ = 0;
    bool flushSucceeds_ = true;
};

}  // namespace shulib::hal::fake
