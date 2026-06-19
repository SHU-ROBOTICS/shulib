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
// DebugRecord schema exists, as a NON-pure virtual with a default (no-op) body — so every
// existing sink (NullSink/TermSink/…) keeps compiling. THAT is what makes it additive and
// F9-versioned, never a break (a pure-virtual addition would break all implementers).

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
