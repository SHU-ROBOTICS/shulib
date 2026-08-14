#pragma once
//
// The D-7 TRIAGE BLOCK — "why did it break", rendered for a human (diagnostics-plan
// D-7; WS13, chunk E1).
//
// The end-of-run summary (C5) answers "how did the run go". This answers the other
// question, and it is the one asked at 2am: which fault fired, at what time, on which
// tick, what the robot's state was at that instant, and how many ticks of history the
// flight recorder captured before it.
//
// Same architecture as every other §18.3 renderer in this directory: diag/ owns the
// VOCABULARY (blackbox::TriageInfo — the SAME struct that goes into the blackbox file)
// and the FORMATTER; the glue that decides WHEN to print it lives with the data
// (motion/run_reporter.hpp calls this at run end, but only when the blackbox actually
// dumped). One record, many renderings: these exact fields are in the file too, so the
// terminal block and the blackbox can never disagree about what happened.
//
// Why it rides log() at ERROR rather than drawing a box on the character device like
// the run summary: this is a fault report, and §18.4's discipline is that faults are
// structured, leveled, greppable key=value lines — not decoration. It also means a
// message-only sink (no record channel) still receives the triage, and that D-2's rate
// limiter is forbidden from throttling it (Error is exempt by contract).

#include "shulib/diag/blackbox_format.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/math/angle.hpp"

namespace shulib::diag {

/// Render the D-7 triage block as two [ERROR][TRI] lines (byte shapes pinned by test).
/// Through TermSink they read:
///
///   [t=   4.20] [ERROR][TRI] fault ODO_STUCK @  4.20s tick 421 preceding 200 brownout no
///   [t=   4.20] [ERROR][TRI] state pos(  24.0,  36.0) hdg  90.0° q=0.91 DR cmd#7▸1 batt 11.9V
///
/// The lines carry no newline of their own: the sink frames them (one write() per line
/// — the A1 framing contract), and a stray newline in a message would be sanitized to
/// '?' anyway.
inline void emitTriageBlock(hal::ITelemetrySink& sink, const blackbox::TriageInfo& info,
                            const DebugRecord& faultTick) {
    using lineformat::Line;
    {
        Line line;
        line.appendLiteral("fault ");
        line.appendLiteral(faultCodeName(info.fault));
        line.appendLiteral(" @");
        lineformat::appendNum(line, info.faultTime, 6, 2);
        line.appendLiteral(" tick ");
        lineformat::appendUnsigned(line, info.tickIndex);
        line.appendLiteral(" preceding ");
        lineformat::appendUnsigned(line, info.precedingTicks);
        line.appendLiteral(" brownout ");
        line.appendLiteral(info.brownout ? "YES" : "no");
        sink.log(hal::LogLevel::Error, "TRI", line.view());
    }
    {
        Line line;
        line.appendLiteral("state pos(");
        lineformat::appendNum(line, faultTick.measuredPose.x().value(), 6, 1);
        line.appendLiteral(",");
        lineformat::appendNum(line, faultTick.measuredPose.y().value(), 6, 1);
        line.appendLiteral(") hdg ");
        lineformat::appendNum(line, faultTick.measuredPose.heading().degrees(), 5, 1);
        line.appendLiteral("° q=");
        lineformat::appendNum(line, faultTick.quality, 4, 2);
        if (faultTick.deadReckoning) {
            line.appendLiteral(" DR");
        }
        line.appendLiteral(" cmd#");
        lineformat::appendUnsigned(line, faultTick.activeCommandId);
        line.appendLiteral("▸");  // the §18.3 id/state separator
        lineformat::appendUnsigned(line, faultTick.activeCommandState);
        line.appendLiteral(" batt ");
        lineformat::appendNum(line, faultTick.batteryVoltage.value(), 4, 1);
        line.appendLiteral("V");
        sink.log(hal::LogLevel::Error, "TRI", line.view());
    }
}

}  // namespace shulib::diag
