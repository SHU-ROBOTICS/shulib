#pragma once
//
// NullSink — the zero-cost default ITelemetrySink (§18.1). Drops everything; the
// competition build routes telemetry here so observability costs ≈nothing when off.

#include <string_view>

#include "shulib/hal/telemetry_sink.hpp"

namespace shulib::hal {

class NullSink final : public ITelemetrySink {
public:
    void log(LogLevel /*level*/, std::string_view /*subsystem*/, std::string_view /*message*/) override {}
};

}  // namespace shulib::hal
