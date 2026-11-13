#pragma once
//
// FakeBlockSource — serves a canned byte vector through the IBlockSource seam so host tests
// can hand a decoder EXACTLY the bytes they mean, including the bytes a damaged card would
// hand it. The read sibling of FakeBlockSink. Host-test only (uses std::vector).
//
// It MODELS THE TWO FAILURES THAT MATTER, because both of them are how a calibration record
// gets misread rather than refused:
//   * setShortRead(n) — the device hands back only the first n bytes of what it holds. That
//     is a TRUNCATED file (a card yanked mid-write, a full card that took a prefix), and the
//     record format must refuse it BY NAME rather than decode the prefix and invent the rest.
//   * setOpen(false) — a refusing source: no card, or the open failed. readAll() then returns
//     0 with nothing copied, which must read as "no record" and never as "an empty record".

#include <cstddef>
#include <span>
#include <vector>

#include "shulib/hal/block_source.hpp"

namespace shulib::hal::fake {

class FakeBlockSource final : public IBlockSource {
public:
    /// An empty, OPEN source (a card with a zero-length file). setBytes() gives it content.
    FakeBlockSource() = default;

    /// An open source holding `bytes`.
    explicit FakeBlockSource(std::vector<std::byte> bytes) : bytes_{std::move(bytes)} {}

    /// Copy min(into.size(), held, shortRead) bytes from the start and return the count.
    /// 0 while refusing, exactly as a no-card adapter behaves.
    [[nodiscard]] std::size_t readAll(std::span<std::byte> into) noexcept override {
        ++readCalls_;
        if (!open_) {
            return 0U;
        }
        std::size_t available = bytes_.size();
        if (shortRead_ < available) {
            available = shortRead_;
        }
        const std::size_t take = into.size() < available ? into.size() : available;
        for (std::size_t i = 0; i < take; ++i) {
            into[i] = bytes_[i];
        }
        return take;
    }

    /// False = this source is refusing (no card / failed open).
    [[nodiscard]] bool isOpen() const noexcept override { return open_; }

    /// Replace what the source holds.
    void setBytes(std::vector<std::byte> bytes) { bytes_ = std::move(bytes); }

    /// Make the device hand back only the first `n` bytes of what it holds — a truncated
    /// file. Default: unbounded (everything it holds).
    void setShortRead(std::size_t n) noexcept { shortRead_ = n; }

    /// Make the source refuse (no card / failed open) or accept.
    void setOpen(bool open) noexcept { open_ = open; }

    /// Everything the source holds, verbatim and in order.
    [[nodiscard]] const std::vector<std::byte>& bytes() const noexcept { return bytes_; }

    /// How many readAll() calls the source has served — the "read exactly once at boot" claim.
    [[nodiscard]] int readCalls() const noexcept { return readCalls_; }

private:
    std::vector<std::byte> bytes_;
    std::size_t shortRead_ = static_cast<std::size_t>(-1);
    bool open_ = true;
    int readCalls_ = 0;
};

}  // namespace shulib::hal::fake
