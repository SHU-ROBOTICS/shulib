#pragma once
//
// line_format — the ONE set of §18.3 text-formatting primitives (WS13, chunk C5).
//
// Extracted VERBATIM from TermSink (chunk A1) when C5 grew two more §18.3 renderers
// (the per-motion result line and the run-summary block): three renderers each
// hand-rolling NaN/±Inf tokens and pathological-magnitude compaction is exactly how
// one of them drifts — a result line printing libc's "-nan(0x400000)" while the tick
// stream prints "NaN" would break the column discipline §18.3 exists for. One
// definition; every §18.3 line is built from these.
//
// The formatting CONTRACT (each clause pinned by the A1 golden tests, which survived
// this extraction bit-identically — that is the refactor's proof):
//   * appendNum: finite values render fixed-width %*.*f; NaN/±Inf render as
//     deterministic right-aligned tokens ("NaN", "+Inf", "-Inf") — never libc's
//     locale/sign-varying spellings; a rendering longer than kCompactThresholdBytes
//     re-renders compactly as %.3g (the column widens slightly rather than exploding
//     to 300+ digits).
//   * Line: a bounded stack buffer (no heap, hot-path safe); appends that would
//     overflow truncate silently — the bound is enforced, not assumed.
//   * appendSanitized: the ONLY entry point for caller-controlled text — control
//     bytes (< 0x20, 0x7F) become '?' so a stray '\n'/ESC can never break the
//     one-line-per-write framing; truncation backs off UTF-8 continuation bytes and
//     marks itself with '…'. (The legacy escapeJSONString lesson: a sanitization
//     step callers must remember is a step that gets skipped — so there is no
//     unsanitized route into a Line.)
//
// Concurrency: everything here is stateless free functions plus a caller-owned
// stack value (Line). Nothing allocates; nothing is shared.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string_view>

namespace shulib::diag::lineformat {

/// A plain %f rendering longer than this is pathological → compact %.3g re-render.
/// 10 comfortably admits every sane field value (±144.00 coords, ±9999.99 t).
inline constexpr int kCompactThresholdBytes = 10;

/// One output line: a bounded stack buffer (no heap, hot-path safe). Appends that
/// would overflow truncate silently — unreachable with the fixed widths the §18.3
/// renderers use, but the bound is enforced, not assumed.
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

/// Right-pad-to-width helper for the non-finite tokens (and any literal that must
/// occupy a numeric column).
inline void appendPadded(Line& line, const char* s, int width) {
    const int len = static_cast<int>(std::strlen(s));
    for (int i = len; i < width; ++i) {
        line.appendLiteral(" ");
    }
    line.appendRaw(s, static_cast<std::size_t>(len));
}

/// Fixed-width numeric column (header contract): finite values via %*.*f; non-finite
/// as deterministic right-aligned tokens; pathologically wide values compacted to %.3g.
inline void appendNum(Line& line, double v, int width, int prec) {
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

inline void appendUnsigned(Line& line, unsigned long v) {
    char tmp[24];
    const int len = std::snprintf(tmp, sizeof tmp, "%lu", v);
    if (len > 0) {
        line.appendRaw(tmp, static_cast<std::size_t>(len));
    }
}

/// The §18.3 "[t=%7.2f] " stamp every timestamped line opens with.
inline void appendTimestamp(Line& line, double tSeconds) {
    line.appendLiteral("[t=");
    appendNum(line, tSeconds, 7, 2);
    line.appendLiteral("] ");
}

}  // namespace shulib::diag::lineformat
