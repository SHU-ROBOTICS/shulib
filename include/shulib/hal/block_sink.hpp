#pragma once
//
// IBlockSink — where BINARY BLOCKS physically go (an SD-card file on the brain, a
// captured buffer in a test). The sibling of ICharSink, added at chunk E1 for the
// blackbox (SdSink): it is the same idea one layer over — the DEVICE is injected, so
// "the file has these exact bytes" is a testable claim instead of an eyeballed one.
//
// ── Why a NEW seam instead of reusing ICharSink (chunk E1, tension T2) ──────────────
// ICharSink's contract is text and LINE-oriented: "one write() call carries one
// complete line", which is what makes TermSink's framing proof possible (a sink that
// is atomic per call can never interleave lines). A blackbox is neither text nor
// line-oriented — it is fixed-width binary containing every byte value including 0x00
// and 0x0A. Redefining ICharSink to mean "bytes, maybe lines, maybe not" would keep
// one seam at the cost of that seam meaning nothing, and would silently invalidate
// the framing argument every TermSink golden rests on. ICharSink is deliberately NOT
// part of the frozen F4 ten (it is an additive diagnostics-output seam), so a SIBLING
// is cheap and honest. The alternative considered and rejected: base64/hex text over
// ICharSink — a text blackbox, explicitly rejected by the E1 brief, paying ~33% more
// bytes for a format that still needs a parser.
//
// Contract:
//   * write() takes the bytes VERBATIM — no framing, no escaping, no sanitization
//     (the FORMAT owns its own structure; see diag/blackbox_format.hpp).
//   * It is called SYNCHRONOUSLY on the caller's task and MUST NOT throw (noexcept).
//   * It returns FALSE if the device did not accept every byte. That return value is
//     the whole point of the seam being bool-valued: an SD card that fills up, is
//     yanked, or dies mid-write is the NORMAL failure of a blackbox, and a void
//     write() would make it invisible. A caller that ignores the result cannot notice
//     a truncated file, so the result is [[nodiscard]].
//   * A partial write may leave a PREFIX of the bytes on the device. The format is
//     designed for exactly that (a truncated blackbox decodes up to the cut and says
//     so) — see diag/blackbox_reader.hpp.
//
// R1 owns the on-robot /usd/ adapter (PROS FILE* behind this interface); E1 ships the
// interface and hal::fake::FakeBlockSink.

#include <cstddef>
#include <span>

namespace shulib::hal {

class IBlockSink {
public:
    virtual ~IBlockSink() = default;
    IBlockSink() = default;
    IBlockSink(const IBlockSink&) = default;
    IBlockSink(IBlockSink&&) = default;
    IBlockSink& operator=(const IBlockSink&) = default;
    IBlockSink& operator=(IBlockSink&&) = default;

    /// Write `bytes` verbatim to the device. Returns false if any byte was not
    /// accepted (a partial write may leave a prefix behind — header note). MUST NOT
    /// throw.
    [[nodiscard]] virtual bool write(std::span<const std::byte> bytes) noexcept = 0;

    /// Push any device-side buffering out to the medium (fflush/fsync on the robot).
    /// Returns false if the device reported a failure. NON-pure with a default
    /// success body, so an implementation with no buffering of its own — the common
    /// case, including the test fake — stays a two-line class. MUST NOT throw.
    virtual bool flush() noexcept { return true; }
};

}  // namespace shulib::hal
