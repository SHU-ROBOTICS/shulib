#pragma once
//
// ITelemetrySink — the diagnostics output seam (master plan §18.1: "one record, many
// sinks"). Every sink — NullSink, TermSink, SdSink, Shul2Sink — sits behind this one
// HAL interface, so the same trace can go to the terminal, an SD blackbox, or the
// SHUL/2 wire without the core knowing which. NullSink (zero-cost) is the
// competition-build default.
//
// At M1 this is the LEVELED-MESSAGE channel (§18.3): leveled, subsystem-tagged lines.
// The per-tick DebugRecord emit (§18.2) is added behind this SAME seam at M2, once the
// DebugRecord schema exists — an additive, F9-versioned extension, never a break.

#include <string_view>

namespace shulib::hal {

/// Severity, high → low. TRACE is compile-time strippable off the hot path (§18.3).
enum class LogLevel { Error, Warn, Info, Debug, Trace };

class ITelemetrySink {
public:
    virtual ~ITelemetrySink() = default;
    ITelemetrySink() = default;
    ITelemetrySink(const ITelemetrySink&) = default;
    ITelemetrySink(ITelemetrySink&&) = default;
    ITelemetrySink& operator=(const ITelemetrySink&) = default;
    ITelemetrySink& operator=(ITelemetrySink&&) = default;

    /// Emit a leveled, subsystem-tagged message (§18.3). Implementations MUST NOT throw.
    virtual void log(LogLevel level, std::string_view subsystem, std::string_view message) = 0;
};

}  // namespace shulib::hal
