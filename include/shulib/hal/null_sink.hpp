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

class NullSink final : public ITelemetrySink {
public:
    void log(LogLevel /*level*/, std::string_view /*subsystem*/, std::string_view /*message*/) override {}
};

}  // namespace shulib::hal
