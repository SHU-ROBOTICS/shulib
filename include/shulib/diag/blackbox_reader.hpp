#pragma once
//
// BlackboxReader — THE DECODER. It ships in the same chunk as the encoder, because a
// format nothing can read is not a record: the first time a blackbox genuinely matters
// is a competition afternoon, and a file that cannot be opened that afternoon is worth
// exactly nothing. (WS13, chunk E1; format in diag/blackbox_format.hpp.)
//
// It is also the thing that makes the encoder TESTABLE. The round trip — encode a known
// stream, decode it, compare field by field — is the chunk's central test, and it needs
// both halves to exist in the same suite.
//
// ── Three rules it will not bend ────────────────────────────────────────────────────
// 1. REFUSE, NEVER MISREAD. A file whose version this build does not know is rejected
//    whole; so is a file whose self-declared tick width disagrees with this build's.
//    A wrong number read confidently is worse than no number: it sends an investigation
//    somewhere false with full confidence. (The width cross-check exists for one very
//    human failure: changing the layout and forgetting to bump the version.)
// 2. NEVER THROW, NEVER READ PAST THE END. The files that matter most are the damaged
//    ones — a run that died mid-write, a card pulled at the wrong moment. Every read
//    goes through a bounds-latching cursor, and a non-finite heading (which the Angle
//    type refuses by precondition) decodes to zero with `corrupt()` raised rather than
//    aborting the program that is trying to read the evidence.
// 3. TRUNCATION IS A RESULT, NOT AN ERROR. A cut file decodes up to the cut and SAYS
//    SO: truncated() is true, the frames before the cut are all delivered, and sawEnd()
//    is false because the graceful-end frame never arrived. That combination — good
//    frames, no end stamp — is exactly what a brownout leaves behind.
//
// Forward compatibility: a frame type this build does not know is SKIPPED BY ITS
// DECLARED LENGTH and counted in skippedFrames(). That is what lets a v1 reader survive
// a later writer that appended a frame kind, without ever guessing at its content.
//
// ── The boundary of rule 1, stated so nobody over-trusts it ─────────────────────────
// "Refuse, never misread" is about INTERPRETATION, not INTEGRITY, and the difference
// matters to anyone building on this format (H1's SHUL/2 in particular).
//
// What is detected: a file that is not a blackbox (BadMagic), a version this build does
// not know (UnsupportedVersion), a layout whose declared widths disagree (LayoutMismatch),
// a cut (truncated()), a frame kind this build lacks (skippedFrames()), and a decoded
// value that is implausible on its face — a non-finite heading raises corrupt() rather
// than tripping Angle's precondition.
//
// What is NOT detected: there is deliberately NO per-frame checksum, so a bit flip inside
// a frame this build CAN interpret, landing on a value that is merely wrong rather than
// impossible, decodes silently as that wrong value. That was measured, not assumed:
// flipping a payload byte mid-file leaves status Ok and every frame delivered.
//
// This is a considered trade, not an oversight — a CRC costs bytes and cycles on every
// tick of a fixed-width budget, and the SD card already carries hardware ECC beneath us.
// Recorded because the honest scope of a guarantee is part of the guarantee: if a future
// consumer needs end-to-end integrity (a wire protocol over a lossy link is the obvious
// case, and F9 is exactly that), it must add its own frame check rather than inherit one
// from here that does not exist. (Boundary verified during E1's independent review.)
//
// Allocation-free and PROS-free, like everything in this tree: it reads a span the
// caller already holds (a whole file loaded into a buffer, or the bytes a test just
// captured), and iterates.

#include <cstddef>
#include <cstdint>
#include <span>

#include "shulib/diag/blackbox_format.hpp"

namespace shulib::diag::blackbox {

/// Why a file is or is not readable by THIS build. Anything other than Ok means no
/// frames are delivered at all — the refuse-don't-misread rule.
enum class ReadStatus : std::uint8_t {
    Ok = 0,                  ///< header parsed and this build can read this version
    Empty = 1,               ///< nothing at all was written (a run with nothing to say)
    HeaderTruncated = 2,     ///< fewer bytes than a complete header
    BadMagic = 3,            ///< not a shulib blackbox
    UnsupportedVersion = 4,  ///< a version this build was not written for — REFUSED
    LayoutMismatch = 5,      ///< right version, wrong record width — REFUSED (header note)
};

/// The §18.5 spelling of a ReadStatus, for messages. Never returns null.
[[nodiscard]] constexpr const char* readStatusName(ReadStatus s) noexcept {
    switch (s) {
        case ReadStatus::Ok: return "OK";
        case ReadStatus::Empty: return "EMPTY";
        case ReadStatus::HeaderTruncated: return "HEADER_TRUNCATED";
        case ReadStatus::BadMagic: return "BAD_MAGIC";
        case ReadStatus::UnsupportedVersion: return "UNSUPPORTED_VERSION";
        case ReadStatus::LayoutMismatch: return "LAYOUT_MISMATCH";
    }
    return "UNKNOWN";
}

class BlackboxReader {
public:
    /// One frame as delivered by next(): its type and a view of its payload, valid for
    /// as long as the caller's file buffer is.
    struct Frame {
        FrameType type = FrameType::Tick;         ///< the frame's kind (known types only)
        std::span<const std::byte> payload{};     ///< exactly the declared payload bytes
    };

    /// Parse the header of `file` and position at the first frame. Never throws; the
    /// verdict is in status(). The span must outlive the reader.
    explicit BlackboxReader(std::span<const std::byte> file) noexcept : file_{file} {
        if (file.empty()) {
            status_ = ReadStatus::Empty;
            return;
        }
        if (file.size() < kHeaderBytes) {
            status_ = ReadStatus::HeaderTruncated;
            truncated_ = true;
            return;
        }
        if (!decodeHeader(file, header_)) {
            status_ = ReadStatus::BadMagic;
            return;
        }
        if (header_.formatVersion != kFormatVersion) {
            status_ = ReadStatus::UnsupportedVersion;
            return;
        }
        if (header_.headerBytes != kHeaderBytes || header_.tickRecordBytes != kTickPayloadBytes) {
            status_ = ReadStatus::LayoutMismatch;
            return;
        }
        status_ = ReadStatus::Ok;
        at_ = kHeaderBytes;
    }

    /// Whether this build can read this file at all (header note, rule 1).
    [[nodiscard]] ReadStatus status() const noexcept { return status_; }
    /// Shorthand for status() == Ok.
    [[nodiscard]] bool usable() const noexcept { return status_ == ReadStatus::Ok; }
    /// The decoded header. Meaningful for Ok and UnsupportedVersion (a refused file
    /// still tells you which version it claims to be — that is how you find the build
    /// that wrote it).
    [[nodiscard]] const BlackboxHeader& header() const noexcept { return header_; }

    /// Deliver the next KNOWN frame, skipping (and counting) unknown ones. Returns
    /// false at the end of the file, on a cut, or when the file is not usable.
    [[nodiscard]] bool next(Frame& out) noexcept {
        if (status_ != ReadStatus::Ok) {
            return false;
        }
        while (true) {
            if (at_ >= file_.size()) {
                return false;  // clean end of data
            }
            if (file_.size() - at_ < kFrameHeaderBytes) {
                truncated_ = true;  // a frame prefix was cut in half
                return false;
            }
            ByteReader head{file_.subspan(at_, kFrameHeaderBytes)};
            const std::uint8_t rawType = head.u8();
            head.skip(1);
            const std::size_t payloadBytes = head.u16();
            const std::size_t frameBytes = kFrameHeaderBytes + payloadBytes;
            if (file_.size() - at_ < frameBytes) {
                truncated_ = true;  // the payload was cut short — stop, and say so
                truncatedType_ = rawType;
                return false;
            }
            const std::span<const std::byte> payload = file_.subspan(at_ + kFrameHeaderBytes, payloadBytes);
            at_ += frameBytes;
            if (!knownType(rawType, payloadBytes)) {
                ++skipped_;
                continue;  // forward compatibility: skip by length, never guess
            }
            const auto type = static_cast<FrameType>(rawType);
            if (type == FrameType::End) {
                sawEnd_ = true;
            }
            ++frames_;
            out = Frame{type, payload};
            return true;
        }
    }

    /// True when the file ended mid-frame — the run was cut short (brownout, pulled
    /// card, a program that never closed). The frames delivered before the cut are all
    /// valid; this says the story stops there.
    [[nodiscard]] bool truncated() const noexcept { return truncated_; }
    /// The raw frame-type byte of the frame that was cut, when truncated() (0 if the
    /// cut fell in a frame prefix).
    [[nodiscard]] std::uint8_t truncatedFrameType() const noexcept { return truncatedType_; }
    /// True once the graceful-end frame has been delivered. Its ABSENCE at the end of
    /// iteration is the honest signal that the run did not close cleanly.
    [[nodiscard]] bool sawEnd() const noexcept { return sawEnd_; }
    /// Known frames delivered so far.
    [[nodiscard]] std::uint32_t framesRead() const noexcept { return frames_; }
    /// Frames skipped because this build does not know their type, or because a known
    /// type carried a payload of the wrong size (a corruption signal that must not stop
    /// the rest of the file from being read).
    [[nodiscard]] std::uint32_t skippedFrames() const noexcept { return skipped_; }
    /// Bytes consumed so far, including the header.
    [[nodiscard]] std::size_t bytesConsumed() const noexcept { return at_; }

private:
    /// A known type whose payload is the size v1 says it must be. A known type with the
    /// wrong size is treated as unknown (skipped and counted) rather than decoded — a
    /// mis-sized frame is corruption, and decoding it would be guessing.
    [[nodiscard]] static bool knownType(std::uint8_t rawType, std::size_t payloadBytes) noexcept {
        switch (rawType) {
            case static_cast<std::uint8_t>(FrameType::Tick): return payloadBytes == kTickPayloadBytes;
            case static_cast<std::uint8_t>(FrameType::Summary): return payloadBytes == kSummaryPayloadBytes;
            case static_cast<std::uint8_t>(FrameType::Triage): return payloadBytes == kTriagePayloadBytes;
            case static_cast<std::uint8_t>(FrameType::End): return payloadBytes == kEndPayloadBytes;
            default: return false;
        }
    }

    std::span<const std::byte> file_;
    BlackboxHeader header_{};
    ReadStatus status_ = ReadStatus::Empty;
    std::size_t at_ = 0;
    std::uint32_t frames_ = 0;
    std::uint32_t skipped_ = 0;
    std::uint8_t truncatedType_ = 0;
    bool truncated_ = false;
    bool sawEnd_ = false;
};

}  // namespace shulib::diag::blackbox
