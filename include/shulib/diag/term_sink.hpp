#pragma once
//
// TermSink — the human-readable terminal stream, the PRIMARY dev/debug surface (master
// plan §18.3; WS13, chunk A1). A pretty-printer for humans: leveled, subsystem-tagged,
// column-aligned lines — never prose, never the JSON wire (that is Shul2Sink at H1).
//
// Injected dependencies (the pattern control/ set): the clock stamps leveled messages;
// the character sink is where bytes go — which is what makes every line here a golden-
// testable claim (test/term_sink_test.cpp pins exact output) instead of an eyeballed one.
//
// ── Output shape (§18.3 target; per-motion results + run summary are chunk C5's) ────
//   leveled message:  [t=  12.51] [WARN][SEQ] intakeUntilCapture retry 1/3 (optical=none)
//                     — Info lines carry NO level tag (the common case stays quiet);
//                       Error/Warn/Debug/Trace carry [LEVEL] butted against [TAG].
//   per-tick record:  [t=  12.34] [MOT] cmd#7▸1 tgt(  24.0,  36.0,  90.0°) err(  0.40",  0.20",  0.3°) v(  18.0,   4.0, 0.10) q=0.91
//                     — timestamped from the RECORD's t (not the clock), so replayed
//                       records render identically to live ones; idle ticks (no active
//                       command) are tagged [LOC]. Trailing flags appear only when set:
//                       " DR" (dead-reckoning), " SFB" (strafe fallback), " CLMP"
//                       (nudge clamped), " flt=NAME" (fault this tick). The line shows
//                       the HEADLINE fields; the full record belongs to SdSink/SHUL/2.
//
// Formatting decisions that keep "column-aligned" true (each pinned by a golden test):
//   * Fixed-width numeric columns ([t=%7.2f], %6.1f coords, …). Deviates from the §18.3
//     sketch's unpadded [t=12.34] deliberately: alignment across ticks is the actual
//     requirement; the sketch is a shape, not a byte spec.
//   * Non-finite values render as deterministic right-aligned tokens ("NaN", "+Inf",
//     "-Inf") — never libc's locale/sign-varying "nan"/"-nan(0x…)".
//   * A value too wide for its column (|v| pathological) re-renders compactly as %.3g
//     ("1e+300") — the column widens slightly rather than exploding to 300+ digits.
//   * Degrees appear here and only here: this is a DISPLAY edge, the one place F3
//     permits radians→degrees.
//
// Sanitization is UNAVOIDABLE BY CONSTRUCTION — the legacy escapeJSONString lesson (a
// helper the caller must remember is a helper that gets forgotten): every byte of
// caller-controlled text (subsystem, message) enters the line through ONE bounded,
// sanitizing append. Control bytes (< 0x20, 0x7F) become '?' so a stray '\n' or ESC
// sequence can never break the one-line-per-call framing or the terminal; bytes ≥ 0x80
// pass through (UTF-8 is welcome). Over-long messages truncate with '…' at a UTF-8
// boundary. There is no unsanitized path to the device.
//
// Concurrency contract (the legacy racing-flush, designed OUT rather than fixed): the
// sink holds NO mutable state — no buffer, no queue, no background flush task. Each call
// formats into a stack-local line and hands the device exactly one write(). Interleaving
// is therefore impossible at this layer; if multiple tasks share one TermSink, ordering
// is whatever the ICharSink's per-call atomicity provides. Nothing here allocates.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string_view>

#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/char_sink.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/math/angle.hpp"

namespace shulib::diag {

class TermSink final : public hal::ITelemetrySink {
public:
    /// Both references must outlive the sink. `clock` stamps log() lines; emit() lines
    /// are stamped from the record itself (see header).
    TermSink(hal::IClock& clock, hal::ICharSink& out) noexcept : clock_{clock}, out_{out} {}

    void log(hal::LogLevel level, std::string_view subsystem, std::string_view message) override {
        Line line;
        appendTimestamp(line, clock_.now().value());
        line.appendLiteral(levelTag(level));
        line.appendLiteral("[");
        line.appendSanitized(subsystem, kMaxTagBytes);
        line.appendLiteral("] ");
        line.appendSanitized(message, kMaxMessageBytes);
        line.appendLiteral("\n");
        out_.write(line.view());
    }

    /// TermSink consumes records — overridden as a pair with emit(), per the seam contract.
    [[nodiscard]] bool wantsRecord() const noexcept override { return true; }

    void emit(const DebugRecord& r) override {
        Line line;
        appendTimestamp(line, r.t.value());
        if (r.activeCommandId != 0) {
            line.appendLiteral("[MOT] cmd#");
            appendUnsigned(line, r.activeCommandId);
            line.appendLiteral("▸");  // ▸ id/state separator, per the §18.3 sketch
            appendUnsigned(line, r.activeCommandState);
            line.appendLiteral(" ");
        } else {
            line.appendLiteral("[LOC] idle ");
        }
        line.appendLiteral("tgt(");
        appendNum(line, r.targetPose.x().value(), 6, 1);
        line.appendLiteral(",");
        appendNum(line, r.targetPose.y().value(), 6, 1);
        line.appendLiteral(",");
        appendNum(line, r.targetPose.heading().degrees(), 6, 1);
        line.appendLiteral("°) err(");  // °
        appendNum(line, r.errorX.value(), 6, 2);
        line.appendLiteral("\",");
        appendNum(line, r.errorY.value(), 6, 2);
        line.appendLiteral("\",");
        appendNum(line, r.errorHeading.value() * kRadToDeg, 5, 1);
        line.appendLiteral("°) v(");
        appendNum(line, r.commanded.vx().value(), 6, 1);
        line.appendLiteral(",");
        appendNum(line, r.commanded.vy().value(), 6, 1);
        line.appendLiteral(",");
        appendNum(line, r.commanded.omega().value(), 5, 2);
        line.appendLiteral(") q=");
        appendNum(line, r.quality, 4, 2);
        if (r.deadReckoning) {
            line.appendLiteral(" DR");
        }
        if (r.strafeFallbackActive) {
            line.appendLiteral(" SFB");
        }
        if (r.clampedThisTick) {
            line.appendLiteral(" CLMP");
        }
        if (r.fault != FaultCode::None) {
            line.appendLiteral(" flt=");
            line.appendLiteral(faultCodeName(r.fault));
        }
        line.appendLiteral("\n");
        out_.write(line.view());
    }

private:
    static constexpr std::size_t kMaxTagBytes = 16;       ///< subsystem tags are short by design
    static constexpr std::size_t kMaxMessageBytes = 200;  ///< structured fields, not essays (§18)
    static constexpr double kRadToDeg = 180.0 / math::Angle::kPi;
    /// A plain %f rendering longer than this is pathological → compact %.3g re-render.
    /// 10 comfortably admits every sane field value (±144.00 coords, ±9999.99 t).
    static constexpr int kCompactThresholdBytes = 10;

    /// One output line: a bounded stack buffer (no heap, hot-path safe). Appends that
    /// would overflow truncate silently — unreachable with the fixed widths above, but
    /// the bound is enforced, not assumed.
    struct Line {
        static constexpr std::size_t kCapacity = 384;

        void appendLiteral(const char* s) { appendRaw(s, std::strlen(s)); }

        void appendRaw(const char* s, std::size_t len) {
            const std::size_t room = kCapacity - n;
            const std::size_t take = len < room ? len : room;
            std::memcpy(buf + n, s, take);
            n += take;
        }

        /// The ONLY entry point for caller-controlled text (header note): sanitizes
        /// control bytes to '?', truncates at `cap` with '…' on a UTF-8 boundary.
        void appendSanitized(std::string_view text, std::size_t cap) {
            const bool truncated = text.size() > cap;
            std::size_t take = truncated ? cap : text.size();
            if (truncated) {
                // Do not split a multi-byte UTF-8 sequence: back off continuation bytes.
                while (take > 0
                       && (static_cast<unsigned char>(text[take]) & 0xC0U) == 0x80U) {
                    --take;
                }
            }
            for (std::size_t i = 0; i < take && n < kCapacity; ++i) {
                const unsigned char c = static_cast<unsigned char>(text[i]);
                buf[n++] = (c < 0x20U || c == 0x7FU) ? '?' : static_cast<char>(c);
            }
            if (truncated) {
                appendLiteral("…");  // …
            }
        }

        [[nodiscard]] std::string_view view() const noexcept { return {buf, n}; }

        char buf[kCapacity];
        std::size_t n = 0;
    };

    static const char* levelTag(hal::LogLevel level) noexcept {
        switch (level) {
            case hal::LogLevel::Error: return "[ERROR]";
            case hal::LogLevel::Warn: return "[WARN]";
            case hal::LogLevel::Info: return "";  // the common case stays quiet (§18.3)
            case hal::LogLevel::Debug: return "[DEBUG]";
            case hal::LogLevel::Trace: return "[TRACE]";
        }
        return "[?]";
    }

    static void appendTimestamp(Line& line, double tSeconds) {
        line.appendLiteral("[t=");
        appendNum(line, tSeconds, 7, 2);
        line.appendLiteral("] ");
    }

    /// Fixed-width numeric column (header note): finite values via %*.*f; non-finite as
    /// deterministic right-aligned tokens; pathologically wide values compacted to %.3g.
    static void appendNum(Line& line, double v, int width, int prec) {
        if (std::isnan(v)) {
            appendPadded(line, "NaN", width);
            return;
        }
        if (std::isinf(v)) {
            appendPadded(line, v > 0.0 ? "+Inf" : "-Inf", width);
            return;
        }
        char tmp[40];
        int len = std::snprintf(tmp, sizeof tmp, "%*.*f", width, prec, v);
        if (len > kCompactThresholdBytes) {
            len = std::snprintf(tmp, sizeof tmp, "%.3g", v);
        }
        if (len > 0) {
            line.appendRaw(tmp, static_cast<std::size_t>(len) < sizeof tmp
                                    ? static_cast<std::size_t>(len)
                                    : sizeof tmp - 1);
        }
    }

    static void appendUnsigned(Line& line, unsigned long v) {
        char tmp[24];
        const int len = std::snprintf(tmp, sizeof tmp, "%lu", v);
        if (len > 0) {
            line.appendRaw(tmp, static_cast<std::size_t>(len));
        }
    }

    static void appendPadded(Line& line, const char* s, int width) {
        const int len = static_cast<int>(std::strlen(s));
        for (int i = len; i < width; ++i) {
            line.appendLiteral(" ");
        }
        line.appendRaw(s, static_cast<std::size_t>(len));
    }

    hal::IClock& clock_;
    hal::ICharSink& out_;
};

}  // namespace shulib::diag
