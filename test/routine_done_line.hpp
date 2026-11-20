// routine_done_line.hpp — RMP1-K D4 (API 2.11): is a sink line the per-step "done in" / "started in" line a SUCCEEDED
// Routine step logs? The bit-identity goldens captured before it (each against its merge base) leave it out with this:
// with that one log call disabled every such golden held at its old value (RMP1-PROGRESS, "RMP1-K", the D4 proof), so
// the line is the only change; routine_step_line_test.cpp pins the line itself.
#pragma once

#include <string_view>

#include "shulib/hal/telemetry_sink.hpp"

namespace routine_d4 {
[[nodiscard]] inline bool isStepDoneLine(shulib::hal::LogLevel level, std::string_view subsystem, std::string_view message) {
    return level == shulib::hal::LogLevel::Info && subsystem == "RTN" &&
           (message.find(") done in ") != std::string_view::npos || message.find(") started in ") != std::string_view::npos);
}
}  // namespace routine_d4
