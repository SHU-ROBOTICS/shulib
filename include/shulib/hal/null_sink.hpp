#pragma once
//
// NullSink — the zero-cost default ITelemetrySink (§18.1). Drops everything; the
// competition build routes telemetry here so observability costs ≈nothing when off.
//
// DELIBERATELY does not override wantsRecord()/emit(): it inherits the seam's defaults
// (wantsRecord() == false, no-op emit()), which is what makes it genuinely ≈free — a
// tick loop using hal::emitRecord() never even POPULATES a DebugRecord for it (the
// §18.2 cost mechanism; see telemetry_sink.hpp's header note). The absence of overrides
// here is also the living proof that the M2 emit() addition was additive: this M1 sink
// compiles untouched.

#include <string_view>

#include "shulib/hal/telemetry_sink.hpp"

namespace shulib::hal {

/// The competition-build default sink: every channel is dropped. What matters is what it
/// does NOT override — wantsRecord() stays false and emit()/summarize() stay the seam's
/// no-ops, which is what makes it ≈free rather than merely fast. A tick loop going through
/// hal::emitRecord() never even POPULATES a DebugRecord, because the builder callable is
/// not invoked at all; the whole per-tick cost is one bool query the compiler can
/// devirtualize. The ONE load-bearing omission is wantsRecord(): overriding it is what would
/// reintroduce the per-tick population cost this class exists to remove. Overriding emit()
/// alone would not — emitRecord() gates solely on wantsRecord(), so the builder would still
/// never run, which is precisely why the seam says override the two as a PAIR — and
/// summarize() is a once-per-run cold path that costs nothing per tick either way.
class NullSink final : public ITelemetrySink {
public:
    /// Discards the message. The parameters are unnamed on purpose — nothing is read, so
    /// nothing is formatted or copied, and the body inlines to nothing.
    void log(LogLevel /*level*/, std::string_view /*subsystem*/, std::string_view /*message*/) override {}
};

}  // namespace shulib::hal
