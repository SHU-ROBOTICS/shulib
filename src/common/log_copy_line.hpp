#pragma once
//
// log_copy_line.hpp — THE ONE BOOT LINE that says where a program's SD-card copy of its log is, or
// that it did not open (chunk T4 Part 3, 2026-10-04; the team lead's N5 (a), A (a) and B (a)).
//
// ── Why here ────────────────────────────────────────────────────────────────────────
// Two programs keep a copy of their log on the card, both through the provider
// (hal/pros/device_provider.hpp, AN OPTIONAL COPY OF THE LOG): "shulib Drive" in
// /usd/drive_log.txt, a new file each boot, and the competition program in /usd/comp_log.txt,
// APPENDED across boots (his B (a): a reboot mid-match must not erase that match's log). Each says
// ONCE at boot where its copy is, or that it did not open (ProsBlockSink's T5 rule: a refusing card
// is said once, by the program, never per line). Two programs typing that sentence would drift, so
// it is made here, PROS-free and host-tested (test/log_copy_line_test.cpp), and each program hands
// in what its provider says (serialCopyOpen(), serialCopyPath(), serialCopyDroppedLines(),
// serialCopyAppendOpen()) mapped to a LogCopyOpen.
//
// What the copy holds is the provider's to say, and the line repeats it: the log's events and
// summaries (diag::LogCopySink: every log() line and summarize() block, never the per-tick record
// lines serial carries), from the moment the provider was constructed — so a banner a program
// printed before that is on serial only.
//
// A copy that did not open says WHY in the words a person holding the card needs (the T4-3
// review, 2026-10-08): no card at all, or a card that is in and still would not take the file
// (full, write-protected, a folder of that name) — the program asks the card once
// (pros::c::usd_is_installed(), A4 register HA-122's probe) and hands the answer in.
//
// AND A BOOT REFUSED AFTER THE COPY OPENED (logBootRefusal(), the same review): the provider opens
// the copy first thing in the boot's try — truncating "shulib Drive"'s file — so a refusal decided
// after that must be said THROUGH the provider's sink, before the provider is released, or the
// card keeps an empty file (or, appended, a boot with no ending) and the reason is on serial only.
//
// Every line is ASCII and at most kLogCopyLineMaxBytes (TermSink cuts a message at 200 bytes,
// diag/term_sink.hpp), with a precision on every %s. NOTHING HERE HAS RUN ON A ROBOT.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/hal/telemetry_sink.hpp"

namespace shulib::bench {

/// How a program's log copy opened at boot.
enum class LogCopyOpen : std::uint8_t {
    NotOpen,    ///< a card is in, but the file would not open: serial only, every line lost
    NoCard,     ///< no SD card at boot: serial only, every line lost
    Truncated,  ///< opened empty: one file per boot ("shulib Drive")
    Created,    ///< opened for APPENDING and did not exist: created (the competition program)
    Appended,   ///< opened for APPENDING and existed: this boot's lines follow its last byte
};

/// The longest line logCopyBootLine() makes: within TermSink's 200 bytes, so it is never cut.
inline constexpr std::size_t kLogCopyLineMaxBytes = 200;

/// One boot line, NUL-terminated.
struct LogCopyLine {
    char text[kLogCopyLineMaxBytes + 1]{};  ///< the line
    bool warn = false;                      ///< log it as a WARNING (the copy did not open)
};

/// The boot line for a copy at `path` (the provider's serialCopyPath(), "/usd/drive_log.txt"; at
/// most 40 characters of it are printed) that opened as `how`, with `dropped` lines lost so far
/// (the provider's serialCopyDroppedLines()). A null `path` prints as "".
[[nodiscard]] inline LogCopyLine logCopyBootLine(LogCopyOpen how, const char* path,
                                                 std::uint32_t dropped) noexcept {
    LogCopyLine out;
    const char* p = path != nullptr ? path : "";
    if (how == LogCopyOpen::NoCard) {
        std::snprintf(out.text, sizeof out.text,
                      "log copy: %.40s did NOT open: NO SD CARD -- serial only; %lu line(s) lost "
                      "so far",
                      p, static_cast<unsigned long>(dropped));
        out.warn = true;
        return out;
    }
    if (how == LogCopyOpen::NotOpen) {
        std::snprintf(out.text, sizeof out.text,
                      "log copy: %.40s did NOT open, though an SD card is in (full? "
                      "write-protected? a folder of that name?) -- serial only; %lu line(s) lost "
                      "so far",
                      p, static_cast<unsigned long>(dropped));
        out.warn = true;
        return out;
    }
    const char* mode = how == LogCopyOpen::Truncated ? "one file per boot"
                       : how == LogCopyOpen::Created ? "CREATED, kept across boots"
                                                     : "APPENDED, kept across boots";
    std::snprintf(out.text, sizeof out.text,
                  "log copy: %.40s (%.27s) -- its events and summaries, no per-tick record, from "
                  "the provider on (earlier lines: serial only)",
                  p, mode);
    return out;
}

/// How a program's copy opened, from what its provider says (`open`: serialCopyOpen();
/// `append`: true for an Append-mode copy, whose `created` says serialCopyAppendOpen() was
/// Created) and whether a card was in (`cardIn`: pros::c::usd_is_installed() != 0, asked once at
/// the boot line). Both programs map through this one function.
[[nodiscard]] constexpr LogCopyOpen logCopyOpenFrom(bool open, bool cardIn, bool append,
                                                    bool created) noexcept {
    if (!open) {
        return cardIn ? LogCopyOpen::NotOpen : LogCopyOpen::NoCard;
    }
    if (!append) {
        return LogCopyOpen::Truncated;
    }
    return created ? LogCopyOpen::Created : LogCopyOpen::Appended;
}

/// The longest piece logBootRefusal() hands the sink: TermSink's 200 bytes, a continuation's
/// "... " included.
inline constexpr std::size_t kRefusalPieceCols = 196;

/// A boot REFUSED after the provider (and its log copy) was built (file banner): `REFUSED:
/// <headline> -- <why>` through `sink` at ERROR, tagged `tag`, broken at spaces into pieces of at
/// most kRefusalPieceCols characters (each later piece starts "... "), so serial AND the copy hold
/// every word. Call it BEFORE the provider is released, and paint the refusal screen with no
/// serial line of its own (the sink already said it). Null texts print as "".
inline void logBootRefusal(hal::ITelemetrySink& sink, std::string_view tag, const char* headline,
                           const char* why) {
    char line[320];
    std::snprintf(line, sizeof line, "REFUSED: %.63s -- %.240s",
                  headline != nullptr ? headline : "", why != nullptr ? why : "");
    const auto pieces = calibration::wrapWords<kRefusalPieceCols, 3>(line);
    for (std::size_t i = 0; i < pieces.count; ++i) {
        if (i == 0) {
            sink.log(hal::LogLevel::Error, tag, pieces.line[0]);
            continue;
        }
        char piece[kRefusalPieceCols + 8];
        std::snprintf(piece, sizeof piece, "... %s", pieces.line[i]);
        sink.log(hal::LogLevel::Error, tag, piece);
    }
    if (!pieces.complete) {  // a word longer than a piece, never split: said, not dropped silently
        sink.log(hal::LogLevel::Error, tag, "... (the rest of this refusal did not fit three log lines)");
    }
}

}  // namespace shulib::bench
