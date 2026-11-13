#pragma once
//
// BlockCharSink — an ICharSink over an IBlockSink: text lines onto a block device such as the SD
// card (chunk COMP-t, 2026-09-28). The driver-program chunk (T4) specified it for its SD copy of
// serial; it is built first here, to that specification, for the Auton Tester's SD run log.
//
// WHY: diag::TermSink writes its lines to an ICharSink, and the SD card's adapter
// (hal/pros/block_sink.hpp) is an IBlockSink. This joins them — each write() is ONE block write
// then ONE flush — so a second TermSink can put the same readable lines on the card that serial
// carries.
//
// A REFUSAL IS DROPPED AND COUNTED, NEVER THROWN. No card, a full card or a yanked card makes the
// block sink say no; that must never stop a robot (the same rule the blackbox follows —
// block_sink.hpp's contract). droppedWrites() counts every line a write OR its flush refused, so
// a caller can say "the card lost N lines" instead of pretending it lost none.
//
// ICharSink's contract (char_sink.hpp): bytes verbatim, synchronous, one complete line per call,
// never throws. This sink allocates nothing and holds nothing but the borrowed block sink and a
// count. PROS-free, header-only; host-tested with hal::fake::FakeBlockSink
// (test/block_char_sink_test.cpp). NOTHING HERE HAS RUN ON A ROBOT.

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "shulib/hal/block_sink.hpp"
#include "shulib/hal/char_sink.hpp"

namespace shulib::hal {

/// An ICharSink over a BORROWED IBlockSink (file banner): each write() is one block write, then
/// flush(); a refusal of either is dropped and counted, never thrown. The block sink must outlive
/// this, and nothing else may write through it between this sink's lines.
class BlockCharSink final : public ICharSink {
public:
    /// Borrow `sink` for this object's whole life.
    explicit BlockCharSink(IBlockSink& sink) noexcept : sink_{&sink} {}

    /// `text` verbatim as ONE block write, then a flush — both attempted every time (a failed
    /// write still flushes what the device took). A refusal of either counts one dropped line.
    void write(std::string_view text) noexcept override {
        const std::span<const std::byte> bytes =
            std::as_bytes(std::span<const char>{text.data(), text.size()});
        const bool wrote = sink_->write(bytes);
        const bool flushed = sink_->flush();
        if (!wrote || !flushed) {
            ++dropped_;
        }
    }

    /// Lines the block sink refused (its write or its flush) since construction.
    [[nodiscard]] std::uint32_t droppedWrites() const noexcept { return dropped_; }

private:
    IBlockSink* sink_;
    std::uint32_t dropped_ = 0;
};

}  // namespace shulib::hal
