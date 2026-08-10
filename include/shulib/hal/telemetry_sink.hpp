#pragma once
//
// ITelemetrySink — the diagnostics output seam (master plan §18.1: "one record, many
// sinks"). Every sink — NullSink, TermSink, SdSink, Shul2Sink — sits behind this one
// HAL interface, so the same trace can go to the terminal, an SD blackbox, or the
// SHUL/2 wire without the core knowing which. NullSink (zero-cost) is the
// competition-build default.
//
// TWO channels ride this one seam:
//   * log()  — the LEVELED-MESSAGE channel (§18.3): leveled, subsystem-tagged lines.
//     Here since M1; pure virtual (every sink must decide what a message means to it).
//   * emit() — the per-tick DebugRecord channel (§18.2), added at M2 (chunk A1) exactly
//     as this header promised: as a NON-pure virtual with a default no-op body, so every
//     sink written against the M1 seam kept compiling untouched. THAT is what makes the
//     addition additive and F9-versionable, never a break — a pure-virtual emit() would
//     have broken NullSink, FakeTelemetrySink, and every future implementer at once.
//     (Pinned by the message-only-sink additivity test in test/telemetry_sink_test.cpp.)
//
// ── The null-sink COST mechanism (§18.2 "NullSink ≈ free"), and why this shape ──────
// The subtle failure this seam must prevent: if the tick loop always builds a
// DebugRecord and hands it to whatever sink is installed, a competition build pays the
// FULL population cost (~30 field writes, pose copies, per-wheel reads) every tick just
// to have NullSink discard it. So the seam carries a cheap consumer query:
//
//     wantsRecord()  — "would emit() do anything with a record?"
//
// and the emitRecord() helper below makes lazy population the DEFAULT idiom: the record
// is built by a callable that is INVOKED ONLY IF the sink wants it. With NullSink the
// per-tick cost is one devirtualizable bool query — population never happens at all
// (pinned by the builder-not-invoked test in test/debug_record_test.cpp).
//
// Alternatives considered and rejected (chunk A1 decision log):
//   * Always-build-and-emit — pays full population cost to discard; rejected outright.
//   * A compile-time sink policy (template / #ifdef) — truly zero-cost, but it bifurcates
//     the build, breaks the ONE runtime seam F4 froze (sinks must be swappable behind
//     ITelemetrySink& at runtime — dev builds switch sinks without recompiling the core),
//     and buys ~nothing: the residual cost is one virtual call per 10ms tick.
//   * wantsRecord() defaulting to TRUE (only NullSink opts out) — then every message-only
//     sink silently pays record population for a no-op emit(). The defaults must AGREE:
//     the default emit() is a no-op, so the default wantsRecord() is false. A sink that
//     overrides emit() MUST override wantsRecord() too — override them as a PAIR.
//
// Concurrency contract (explicit, because the legacy logger's flush raced a background
// task): this seam has no background anything. Every log()/emit() call runs synchronously
// on the CALLER's task, and implementations must document their own thread-safety (the
// shipped sinks are single-task by contract). Implementations MUST NOT throw.

#include <string_view>
#include <utility>

namespace shulib::diag {
// Forward declarations only: the full schemas live in shulib/diag/debug_record.hpp
// and shulib/diag/run_summary.hpp. The seam stays include-light (a reference
// parameter with a no-op body needs no complete type); implementers that actually
// READ a record include the schema header themselves.
struct DebugRecord;
struct RunSummary;
}  // namespace shulib::diag

namespace shulib::hal {

/// Severity, high → low. TRACE is compile-time strippable off the hot path (§18.3;
/// see shulib/diag/trace.hpp for the strip mechanism).
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

    /// Would emit() consume a DebugRecord? Callers use this (via emitRecord below) to
    /// skip record POPULATION entirely when nothing consumes it — the null-sink cost
    /// mechanism (header note). Default false, matching the default no-op emit():
    /// OVERRIDE THIS AND emit() AS A PAIR.
    [[nodiscard]] virtual bool wantsRecord() const noexcept { return false; }

    /// Consume one per-tick DebugRecord (§18.2). NON-pure with a default no-op body BY
    /// CONTRACT (header note) — a sink implementing only log() stays valid forever.
    /// Implementations MUST NOT throw.
    virtual void emit(const diag::DebugRecord& /*record*/) {}

    /// Consume the end-of-run summary (§18.3's one-screen block, as data — see
    /// shulib/diag/run_summary.hpp). Added at chunk C5 by the SAME additive recipe as
    /// emit(): non-pure, default no-op, so every sink written before it kept
    /// compiling untouched. Called ONCE per run (cold path — no wants-style query is
    /// needed; building one struct per run is not a cost). DECORATOR RULE: a sink
    /// that wraps another MUST forward this, like log()/emit() — a decorator with
    /// the default body silently eats the summary. Implementations MUST NOT throw.
    virtual void summarize(const diag::RunSummary& /*summary*/) {}
};

/// THE record-emission idiom: build the record lazily, ONLY if the sink consumes it.
///
///     hal::emitRecord(sink, [&] { DebugRecord r; …populate…; return r; });
///
/// This exists so skipping population is the path of least resistance — the same lesson
/// as the legacy escapeJSONString defect (a safety step a caller can forget is a safety
/// step that WILL be forgotten): callers who go through emitRecord() cannot accidentally
/// pay for a record nothing reads. `buildRecord` is any callable returning a DebugRecord
/// (by value; it binds to emit's const&). Calling sites need the full schema header.
template <typename BuildFn>
inline void emitRecord(ITelemetrySink& sink, BuildFn&& buildRecord) {
    if (sink.wantsRecord()) {
        sink.emit(std::forward<BuildFn>(buildRecord)());
    }
}

}  // namespace shulib::hal
