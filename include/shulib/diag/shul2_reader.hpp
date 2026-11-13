#pragma once
//
// Shul2Reader — the SHUL/2 reader over a WHOLE capture (chunk H1 Part A1, 2026-09-29; the
// format is diag/shul2_format.hpp). It ships in the same part as the format for the reason
// BlackboxReader shipped with the blackbox: a format nothing can read is not a record, and the
// reader is what makes the encoders testable at all.
//
// It borrows the span (a capture the caller already holds — a file, or the bytes a test just
// built), allocates nothing, never throws, and never reads past the end. It is built on
// scanFrame() and negotiate() and on nothing else, so a LIVE reader (H2's socket loop, which
// waits on NeedMore instead) composes the same two functions and meets the same rules. There
// is no socket code here.
//
// ── Where it differs from BlackboxReader, and why ───────────────────────────────────
// At the TAIL. BlackboxReader stops at a frame whose declared length runs past the end and
// calls the file truncated — right for a file with no check, where the length is all there is
// to believe. Here every frame carries a CRC, so a declared length is only a CLAIM until the
// CRC verifies, and a corrupted length near the end of a capture would otherwise throw away
// every good frame after it. So a whole capture, which never receives more bytes, treats a
// NeedMore at position p as a failed candidate and resumes scanning at p + 1:
//   * if a frame verifies later, p was a corrupted length — counted in lengthOverruns;
//   * `truncated` is set if and only if the capture ends inside a candidate no later frame
//     displaced: the EARLIEST NeedMore after the last verified frame (truncatedAt);
//   * a capture that ends exactly on a frame boundary is not truncated.
//
// ── Delivery ────────────────────────────────────────────────────────────────────────
// next() delivers verified frames {type, flags, seq, payload} of the five known types, ONLY
// from ACCEPTED sessions (Exact, OlderMinor, NewerMinor), and only when the frame's width
// agrees with its session's Hello: a Tick, Summary or End of exactly the declared width; a Log
// whose tag and message fit and whose non-text bytes equal the declared logFixedBytes. A frame
// that disagrees counts in wrongWidthSkipped. Bytes beyond this build's known width are counted
// in extensionBytesIgnored (Hello: payload − the known Hello width; Log: non-text − the known
// 14; the others: payload − the known width) — that is how a NEWER minor's appended fields are
// ignored without being lost from the account. The payload delivered is the WHOLE payload; the
// decoders in shul2_format.hpp read the known prefix of it.
//
// ── Sessions ────────────────────────────────────────────────────────────────────────
//   * A Hello with seq 0 starts a new session (a writer that re-opened, or the first).
//   * A Hello with seq > 0 while no session is open is a LATE JOIN; the session starts there.
//   * A Hello with seq > 0 inside a session must match that session's Hello byte for byte — a
//     repeat (helloRepeats), delivered like any frame. Otherwise it is a helloMismatch and
//     starts a new session.
//   * An End closes its session. A frame outside any open session (before the first Hello, or
//     after an End and before the next Hello) counts in framesOutsideSession.
//   * A REFUSED session's frames — its End and its repeats included — count in
//     framesInRefusedSessions until the next Hello that differs from its own, and are never
//     delivered: nothing in a stream this build cannot interpret is interpreted.
//   * Any role is accepted and reported; delivery is by type, never by role.
//
// ── Sequence ────────────────────────────────────────────────────────────────────────
// Every frame of an accepted session (unknown types and repeats included — the writer offered
// them) is checked against expected = the last accepted seq + 1, modulo 2^32:
//   d = 0 in order; 0 < d < 2^31 means d frames were lost (lostFrames); d ≥ 2^31 is a stale or
//   duplicate frame, skipped (staleFrames). A sequence that wraps 0xFFFFFFFF → 0 is in order
//   and starts nothing: only a HELLO at seq 0 starts a session.
//
// ── Link loss, per session ──────────────────────────────────────────────────────────
// linkLostFrames = max(0, lostFrames − End.framesFailed): the frames the LINK lost, as opposed
// to the ones the writer's device refused (which the writer counted, and which also appear as
// gaps). Known ONLY for a session read from its seq-0 Hello whose End was delivered; every
// other ACCEPTED session counts in sessionsLinkLossUnknown (a late join, a mismatch-started
// session, a session with no End). An End whose framesOffered differs from its own sequence
// number counts in endDisagreements; so does one of a seq-0 session whose framesFailed exceeds
// the frames the session lost — and the figure then saturates at 0 instead of wrapping. (For a
// late-joined session that comparison means nothing: the frames lost before the join were
// never seen.)
//
// ── What a live reader must hold, and what it must do when its stream ends ──────────
// At least kMaxFrameBytes (14 + 4096) bytes. A corrupted length stalls a live reader until that
// many bytes have arrived — about nine tick frames, ~90 ms at 100 Hz — and then the frames
// inside those bytes are recovered, because scanning resumes one byte on. A whole capture has
// no stall: it resumes at once. When a live stream ENDS — the socket closes, the file ends —
// what the live reader still holds IS a whole capture, and it must apply the tail rule above to
// it: a corrupted length in the last 4 KB would otherwise hold back every frame after it, the
// End included, for good, and that session's link loss would become unknowable. (Applying the
// rule after an idle timeout as well is a live reader's own policy — H2's: it releases a held
// End sooner, at the cost of dropping a frame whose bytes were merely late.)
//
// ── What reading costs ──────────────────────────────────────────────────────────────
// Linear in the capture for ordinary damage and random bytes (a random sync pair is about one
// position in 65,536). The worst case is bytes CRAFTED to hold as many candidates claiming an
// accepted length as a byte pattern can — about three per ten bytes — each costing up to 4104
// bytes of bitwise CRC before it fails: about 1,200 bytes of CRC per input byte, measured on a
// desktop host at roughly 14-23 s per MB of such input (shul2_format.hpp, the resync rule). There
// is no time budget here — this reader runs on a host; a reader on a UI thread should bound its own.
//
// Allocation-free and PROS-free, like everything in this tree.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#include "shulib/diag/shul2_format.hpp"

namespace shulib::diag::shul2 {

/// One named verdict for a whole capture. Final once Shul2Reader::finished() is true.
enum class CaptureStatus : std::uint8_t {
    Empty = 0,      ///< no bytes at all
    NoFrame = 1,    ///< bytes, but no frame verified: a bit-7-stripping or NUL-truncating
                    ///< transport, or not SHUL/2 at all
    NoSession = 2,  ///< frames verified, but no Hello: a newline-translating transport (the
                    ///< signature's CR LF kills every Hello), or a capture that ended before
                    ///< the first re-Hello
    Refused = 3,    ///< every session was refused; the latest session's verdict says why
    Ok = 4,         ///< at least one session was accepted
};

/// The §18.5 spelling of a CaptureStatus, for messages. Never returns null.
[[nodiscard]] constexpr const char* captureStatusName(CaptureStatus s) noexcept {
    switch (s) {
        case CaptureStatus::Empty: return "EMPTY";
        case CaptureStatus::NoFrame: return "NO_FRAME";
        case CaptureStatus::NoSession: return "NO_SESSION";
        case CaptureStatus::Refused: return "REFUSED";
        case CaptureStatus::Ok: return "OK";
    }
    return "UNKNOWN";
}

/// Capture-wide counts, as of the bytes consumed so far (final once finished()). Byte counts
/// are std::size_t; loss figures saturate rather than wrap.
struct ReaderCounters {
    std::size_t bytesSkipped = 0;             ///< every byte not inside a verified frame
    std::uint32_t framesVerified = 0;         ///< frames whose sync matched and CRC verified
    std::uint32_t crcFailures = 0;            ///< whole candidates whose CRC did not verify
    std::uint32_t lengthOverruns = 0;         ///< tail candidates (a NeedMore) a later frame displaced
    std::uint32_t sessions = 0;               ///< sessions started (accepted and refused)
    std::uint32_t refusedSessions = 0;        ///< sessions negotiate() refused
    std::uint32_t lateJoins = 0;              ///< sessions started at a Hello with seq > 0, none open
    std::uint32_t framesDelivered = 0;        ///< frames next() handed out
    std::uint32_t framesOutsideSession = 0;   ///< verified frames with no session open
    std::uint32_t framesInRefusedSessions = 0;  ///< verified frames of refused sessions
    std::uint32_t unknownTypeSkipped = 0;     ///< in-session frames of a type this build does not assign
    std::uint32_t wrongWidthSkipped = 0;      ///< known-type frames whose width disagrees with the Hello
    std::size_t extensionBytesIgnored = 0;    ///< bytes beyond this build's known widths, not read
    std::uint32_t staleFrames = 0;            ///< stale or duplicate frames (d ≥ 2^31), skipped
    std::uint32_t helloRepeats = 0;           ///< re-Hellos identical to their session's Hello
    std::uint32_t helloMismatches = 0;        ///< re-Hellos at seq > 0 that differed (new session)
    std::uint32_t lostFrames = 0;             ///< sequence gaps in accepted sessions, summed
    std::uint32_t linkLostFrames = 0;         ///< link loss, summed over sessions where it is known
    std::uint32_t sessionsLinkLossUnknown = 0;  ///< accepted sessions whose link loss is unknowable
    std::uint32_t endDisagreements = 0;       ///< Ends that disagree with what the reader saw
    std::uint32_t sessionsEnded = 0;          ///< accepted sessions whose End was delivered
    bool truncated = false;                   ///< the capture ends inside an undisplaced candidate
    std::size_t truncatedAt = 0;              ///< that candidate's offset, when `truncated`
};

/// What the reader knows about ONE session — Shul2Reader::latestSession() is the latest.
struct SessionReport {
    /// Its Hello, decoded (views into the capture). For a session of ANOTHER major only the
    /// signature and version are decoded (`hello.complete` false): the rest is that major's.
    Hello hello{};
    Verdict verdict = Verdict::BadSignature;  ///< negotiate()'s verdict on it
    std::uint32_t helloSeq = 0;            ///< the sequence number of the Hello that started it
    /// True when the session did not start at its seq-0 Hello — a late join or a mismatched
    /// re-Hello — so its link loss cannot be known.
    bool lateJoin = false;
    std::uint32_t lostFrames = 0;          ///< its sequence gaps (accepted sessions only)
    bool sawEnd = false;                   ///< its End was delivered
    EndInfo end{};                         ///< that End, when sawEnd
    bool linkLossKnown = false;            ///< linkLostFrames is meaningful (seq-0 start, End seen)
    std::uint32_t linkLostFrames = 0;      ///< max(0, lostFrames − end.framesFailed), when known
};

/// The reader of a whole SHUL/2 capture: point it at bytes the caller holds — it BORROWS the
/// span and allocates nothing — and pull frames with next() until it returns false. Frames are
/// delivered only from accepted sessions and only at the widths their Hello declared; every
/// byte and frame that is not delivered is counted by name in counters(). A corrupted frame
/// costs exactly that frame: scanning resumes one byte on, never at a claimed length. At the
/// tail, a candidate that runs past the end is truncation only if no later frame verifies
/// (header). The verdict for the whole capture is status(), final once finished().
class Shul2Reader {
public:
    /// One delivered frame: its type, envelope flags, sequence number and whole payload (a view
    /// valid while the capture is).
    struct Frame {
        FrameType type = FrameType::Tick;      ///< one of the five known types
        std::uint8_t flags = 0;                ///< envelope flags as they arrived (never judged)
        std::uint32_t seq = 0;                 ///< its sequence number
        std::span<const std::byte> payload{};  ///< the WHOLE payload, appended bytes included
    };

    /// Read `capture` with `profile` (this build's, unless a test stands up another). Both are
    /// borrowed: the capture and the profile's rows must outlive the reader — never hand it a
    /// temporary buffer (H1's own test suite did once, and read freed memory). Never throws.
    explicit Shul2Reader(std::span<const std::byte> capture,
                         const ReaderProfile& profile = kThisReader) noexcept
        : capture_{capture}, profile_{profile} {}

    /// Deliver the next frame, skipping and counting everything that is not delivered.
    /// Returns false at the end of the capture, which finishes it (finished() is then true).
    [[nodiscard]] bool next(Frame& out) noexcept {
        while (at_ < capture_.size()) {
            const ScanResult r = scanFrame(capture_.subspan(at_));
            if (r.kind == ScanKind::NeedMore) {
                // The tail rule: a whole capture never receives more bytes, so this candidate
                // failed. Remember the EARLIEST one since the last verified frame.
                if (!pendingTail_) {
                    pendingTail_ = true;
                    pendingTailAt_ = at_;
                }
                ++counters_.bytesSkipped;
                ++at_;
                continue;
            }
            if (r.kind == ScanKind::Skip) {
                if (r.reason == SkipReason::CrcMismatch) {
                    ++counters_.crcFailures;
                }
                counters_.bytesSkipped += r.consumed;
                at_ += r.consumed;
                continue;
            }
            if (pendingTail_) {
                ++counters_.lengthOverruns;  // a later frame displaced it: a corrupted length
                pendingTail_ = false;
            }
            ++counters_.framesVerified;
            at_ += r.consumed;
            if (onFrame(r, out)) {
                ++counters_.framesDelivered;
                return true;
            }
        }
        finish();
        return false;
    }

    /// Read to the end, discarding the frames — for a caller that wants only the verdict and
    /// the counts.
    void drain() noexcept {
        Frame f;
        while (next(f)) {
        }
    }

    /// True once next() has returned false: status() and counters() are then final.
    [[nodiscard]] bool finished() const noexcept { return finished_; }

    /// The capture's verdict (final once finished()).
    [[nodiscard]] CaptureStatus status() const noexcept {
        if (capture_.empty()) {
            return CaptureStatus::Empty;
        }
        if (counters_.framesVerified == 0) {
            return CaptureStatus::NoFrame;
        }
        if (counters_.sessions == 0) {
            return CaptureStatus::NoSession;
        }
        if (counters_.refusedSessions == counters_.sessions) {
            return CaptureStatus::Refused;
        }
        return CaptureStatus::Ok;
    }

    /// The capture-wide counts (final once finished()).
    [[nodiscard]] const ReaderCounters& counters() const noexcept { return counters_; }
    /// True once any Hello has started a session.
    [[nodiscard]] bool hasSession() const noexcept { return counters_.sessions > 0; }
    /// The latest session (a default report while hasSession() is false).
    [[nodiscard]] const SessionReport& latestSession() const noexcept { return session_; }
    /// Bytes of the capture consumed so far.
    [[nodiscard]] std::size_t bytesConsumed() const noexcept { return at_; }

private:
    enum class State : std::uint8_t { None, Accepted, Refused, Closed };

    static void saturatingAdd(std::uint32_t& total, std::uint32_t add) noexcept {
        total = (0xFFFFFFFFU - total < add) ? 0xFFFFFFFFU : total + add;
    }

    /// The sequence rule for a frame of an ACCEPTED session. False = stale (counted, skipped).
    [[nodiscard]] bool inSequence(std::uint32_t seq) noexcept {
        const std::uint32_t d = seq - expected_;  // modulo 2^32 by construction
        if (d >= 0x80000000U) {
            ++counters_.staleFrames;
            return false;
        }
        saturatingAdd(session_.lostFrames, d);
        saturatingAdd(counters_.lostFrames, d);
        expected_ = seq + 1U;
        return true;
    }

    /// An accepted session that ends without its End: its link loss is unknowable.
    void abandonOpenSession() noexcept {
        if (state_ == State::Accepted) {
            ++counters_.sessionsLinkLossUnknown;
        }
    }

    void startSession(const Hello& h, const ScanResult& r, bool lateJoin) noexcept {
        abandonOpenSession();
        ++counters_.sessions;
        session_ = SessionReport{};
        session_.hello = h;
        session_.helloSeq = r.seq;
        session_.lateJoin = lateJoin;
        session_.verdict = negotiate(h, profile_);
        helloBytes_ = r.payload;
        expected_ = r.seq + 1U;
        if (!accepted(session_.verdict)) {
            ++counters_.refusedSessions;
            state_ = State::Refused;
            return;
        }
        state_ = State::Accepted;
        const std::size_t newest = profile_.rows.size() - 1U;  // non-empty: negotiate accepted
        known_ = profile_.rows[h.versionMinor < newest ? h.versionMinor : newest];
    }

    [[nodiscard]] bool onHello(const ScanResult& r, Frame& out) noexcept {
        Hello h;
        (void)decodeHello(r.payload, h);  // a short Hello is judged BadSignature by negotiate()
        const bool open = state_ == State::Accepted || state_ == State::Refused;
        if (open && r.seq != 0U) {
            const bool same = r.payload.size() == helloBytes_.size()
                              && std::equal(r.payload.begin(), r.payload.end(), helloBytes_.begin());
            if (same) {
                if (state_ == State::Refused) {
                    ++counters_.helloRepeats;
                    ++counters_.framesInRefusedSessions;
                    return false;
                }
                if (!inSequence(r.seq)) {
                    return false;
                }
                ++counters_.helloRepeats;
                counters_.extensionBytesIgnored += r.payload.size() - known_.hello;
                out = Frame{FrameType::Hello, r.flags, r.seq, r.payload};
                return true;
            }
            ++counters_.helloMismatches;
            startSession(h, r, true);
        } else if (r.seq == 0U) {
            startSession(h, r, false);
        } else {
            ++counters_.lateJoins;
            startSession(h, r, true);
        }
        if (state_ != State::Accepted) {
            return false;
        }
        counters_.extensionBytesIgnored += r.payload.size() - known_.hello;
        out = Frame{FrameType::Hello, r.flags, r.seq, r.payload};
        return true;
    }

    /// Exactly the declared width, and at least the known one (negotiate guaranteed it).
    [[nodiscard]] bool fixedWidthOk(std::size_t payload, std::uint16_t declared,
                                    std::uint16_t known) noexcept {
        if (payload != declared) {
            ++counters_.wrongWidthSkipped;
            return false;
        }
        counters_.extensionBytesIgnored += payload - known;
        return true;
    }

    void onEnd(const ScanResult& r) noexcept {
        EndInfo e;
        (void)decodeEnd(r.payload, e);  // the width was checked: at least 32 bytes
        session_.sawEnd = true;
        session_.end = e;
        ++counters_.sessionsEnded;
        bool disagrees = e.framesOffered != r.seq;
        if (!session_.lateJoin) {
            std::uint32_t link = 0;
            if (e.framesFailed > session_.lostFrames) {
                disagrees = true;  // the writer claims more refusals than the reader saw gaps
            } else {
                link = session_.lostFrames - e.framesFailed;
            }
            session_.linkLossKnown = true;
            session_.linkLostFrames = link;
            saturatingAdd(counters_.linkLostFrames, link);
        } else {
            ++counters_.sessionsLinkLossUnknown;
        }
        if (disagrees) {
            ++counters_.endDisagreements;
        }
        state_ = State::Closed;
    }

    [[nodiscard]] bool onFrame(const ScanResult& r, Frame& out) noexcept {
        if (r.type == static_cast<std::uint8_t>(FrameType::Hello)) {
            return onHello(r, out);
        }
        if (state_ == State::None || state_ == State::Closed) {
            ++counters_.framesOutsideSession;
            return false;
        }
        if (state_ == State::Refused) {
            ++counters_.framesInRefusedSessions;
            return false;
        }
        if (!inSequence(r.seq)) {
            return false;
        }
        const WidthRow& d = session_.hello.declared;
        const std::size_t n = r.payload.size();
        FrameType type = FrameType::Tick;
        switch (r.type) {
            case static_cast<std::uint8_t>(FrameType::Tick):
                if (!fixedWidthOk(n, d.tick, known_.tick)) {
                    return false;
                }
                type = FrameType::Tick;
                break;
            case static_cast<std::uint8_t>(FrameType::Summary):
                if (!fixedWidthOk(n, d.summary, known_.summary)) {
                    return false;
                }
                type = FrameType::Summary;
                break;
            case static_cast<std::uint8_t>(FrameType::End):
                if (!fixedWidthOk(n, d.end, known_.end)) {
                    return false;
                }
                onEnd(r);
                type = FrameType::End;
                break;
            case static_cast<std::uint8_t>(FrameType::Log): {
                LogLine line;
                if (!decodeLog(r.payload, line) || line.nonTextBytes != d.logFixed) {
                    ++counters_.wrongWidthSkipped;
                    return false;
                }
                counters_.extensionBytesIgnored += line.nonTextBytes - known_.logFixed;
                type = FrameType::Log;
                break;
            }
            default:
                ++counters_.unknownTypeSkipped;
                return false;
        }
        out = Frame{type, r.flags, r.seq, r.payload};
        return true;
    }

    void finish() noexcept {
        if (finished_) {
            return;
        }
        finished_ = true;
        if (pendingTail_) {
            counters_.truncated = true;
            counters_.truncatedAt = pendingTailAt_;
        }
        abandonOpenSession();
    }

    std::span<const std::byte> capture_;
    ReaderProfile profile_;
    ReaderCounters counters_{};
    SessionReport session_{};
    std::span<const std::byte> helloBytes_{};
    WidthRow known_{};
    State state_ = State::None;
    std::uint32_t expected_ = 0;
    std::size_t at_ = 0;
    std::size_t pendingTailAt_ = 0;
    bool pendingTail_ = false;
    bool finished_ = false;
};

}  // namespace shulib::diag::shul2
