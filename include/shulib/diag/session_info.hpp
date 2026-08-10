#pragma once
//
// SessionInfo + the §18.5 session header — provenance as the FIRST lines of every
// run (WS13, chunk C5).
//
// §18.5: "First record of every run: git build hash + routine id + alliance/side +
// port map + battery start — lets us compare/reproduce runs and confirm exactly
// which binary produced a given log." The header is what turns a pile of logs into
// an archive: without it, two traces that disagree cannot even be attributed to
// two binaries.
//
// The header rides the log() channel as [SES]-tagged Info lines (the FaultLatch
// precedent: the OWNER formats structured text, the sink sanitizes and frames it),
// which is what gives it the exact "[t=…] [SES] …" §18.3 shape through TermSink —
// and lets a message-only sink capture provenance too.
//
// THE MISSING-HASH PATH IS LOUD (build_info.hpp carries the full rationale): an
// empty buildHash emits an [ERROR][SES] line FIRST — before anything else — and
// the header line renders the literal token MISSING. Never a plausible value.
//
// Field notes:
//   * portMap is CALLER-AUTHORED text at C5 ("L1,2,3 R4,5,6 IMU10"): the core HAL
//     deliberately has no port numbers (fakes don't have ports), so the honest v1
//     is a pass-through string; G1's RobotBuilder generates it from the profile.
//   * alliance/side are free text ("red"/"left"/"skills"); empty fields render as
//     "-" so a blank never silently vanishes from a column.
//   * battery start is READ, not caller-typed — a typed 12.6 that was actually
//     11.9 is the lying-number class this chunk bans.

#include <string_view>

#include "shulib/diag/line_format.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

/// Provenance for one run. string_views: the caller keeps the storage alive for
/// the emitSessionHeader() call only (they are read synchronously, never retained
/// — RunSummary re-copies what it needs into bounded arrays).
struct SessionInfo {
    /// From diag::compiledBuildHash() in the app's own TU. EMPTY = missing = LOUD.
    std::string_view buildHash{};
    std::string_view routineId{};  ///< e.g. "redLeftTall"
    std::string_view alliance{};   ///< "red" / "blue" / "skills"
    std::string_view side{};       ///< "left" / "right"
    std::string_view portMap{};    ///< caller-authored at C5; G1 generates (header note)
};

/// Bounds for the header's caller-controlled fields (sanitized appends; the sink
/// itself re-sanitizes — defense in depth, and these keep one field from eating
/// the line).
inline constexpr std::size_t kMaxHashBytes = 47;     ///< full SHA + "-dirty"
inline constexpr std::size_t kMaxFieldBytes = 24;    ///< routine/alliance/side
inline constexpr std::size_t kMaxPortMapBytes = 96;  ///< the port map is the long one

/// Emit the §18.5 session header: three [SES] Info lines (plus the [ERROR][SES]
/// line FIRST when the hash is missing). `batteryStart` must be a live reading
/// (header note). Byte shapes pinned by test/session_header_test.cpp.
///
///   [t=   0.00] [SES] run start · build 0b4948a-dirty · routine "redLeftTall"
///   [t=   0.00] [SES] alliance red · side left · batt 12.40V
///   [t=   0.00] [SES] ports L1,2,3 R4,5,6 IMU10
inline void emitSessionHeader(hal::ITelemetrySink& sink, const SessionInfo& info,
                              units::Voltage batteryStart) {
    using lineformat::Line;
    const auto orDash = [](std::string_view v) { return v.empty() ? std::string_view{"-"} : v; };

    if (info.buildHash.empty()) {
        // Loud and FIRST: the header's one load-bearing field is absent.
        sink.log(hal::LogLevel::Error, "SES",
                 "build hash MISSING — define SHULIB_BUILD_HASH at build time; "
                 "a wrong hash is worse than none, so nothing is invented");
    }
    {
        Line line;
        line.appendLiteral("run start · build ");
        if (info.buildHash.empty()) {
            line.appendLiteral("MISSING");
        } else {
            line.appendSanitized(info.buildHash, kMaxHashBytes);
        }
        line.appendLiteral(" · routine \"");
        line.appendSanitized(info.routineId, kMaxFieldBytes);
        line.appendLiteral("\"");
        sink.log(hal::LogLevel::Info, "SES", line.view());
    }
    {
        Line line;
        line.appendLiteral("alliance ");
        line.appendSanitized(orDash(info.alliance), kMaxFieldBytes);
        line.appendLiteral(" · side ");
        line.appendSanitized(orDash(info.side), kMaxFieldBytes);
        line.appendLiteral(" · batt ");
        lineformat::appendNum(line, batteryStart.value(), 5, 2);
        line.appendLiteral("V");
        sink.log(hal::LogLevel::Info, "SES", line.view());
    }
    {
        Line line;
        line.appendLiteral("ports ");
        line.appendSanitized(orDash(info.portMap), kMaxPortMapBytes);
        sink.log(hal::LogLevel::Info, "SES", line.view());
    }
}

}  // namespace shulib::diag
