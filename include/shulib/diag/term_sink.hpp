#pragma once
//
// TermSink — the human-readable terminal stream, the PRIMARY dev/debug surface (master
// plan §18.3; WS13, chunk A1). A pretty-printer for humans: leveled, subsystem-tagged,
// column-aligned lines — never prose, never the JSON wire (that is Shul2Sink at H1).
//
// Injected dependencies (the pattern control/ set): the clock stamps leveled messages;
// the character sink is where bytes go — which is what makes every line here a golden-
// testable claim (test/term_sink_test.cpp pins exact output) instead of an eyeballed one.
//
// ── Output shape (§18.3 target; result lines + summary added at chunk C5) ───────────
//   leveled message:  [t=  12.51] [WARN][SEQ] intakeUntilCapture retry 1/3 (optical=none)
//                     — Info lines carry NO level tag (the common case stays quiet);
//                       Error/Warn/Debug/Trace carry [LEVEL] butted against [TAG].
//   per-tick record:  [t=  12.34] [MOT] cmd#7▸1 tgt(  24.0,  36.0,  90.0°) err(  0.40",  0.20",  0.3°) v(  18.0,   4.0, 0.10) q=0.91
//                     — timestamped from the RECORD's t (not the clock), so replayed
//                       records render identically to live ones; idle ticks (no active
//                       command) are tagged [LOC]. Trailing flags appear only when set:
//                       " DR" (dead-reckoning), " SFB" (strafe fallback), " CLMP"
//                       (nudge clamped), " flt=NAME" (fault this tick). The line shows
//                       the HEADLINE fields; the full record belongs to SdSink/SHUL/2.
//   run summary:      summarize(RunSummary) renders the §18.3 one-screen block (chunk
//                     C5) — six lines, unstamped, byte-pinned by golden test. The
//                     per-motion RESULT LINE does not enter here: it rides log() as
//                     structured Info text (diag/motion_result.hpp formats it), which
//                     is what gives it the exact "[t=…] [MOT] …" §18.3 shape.
//
// Formatting decisions that keep "column-aligned" true (each pinned by a golden test):
//   * Fixed-width numeric columns ([t=%7.2f], %6.1f coords, …). Deviates from the §18.3
//     sketch's unpadded [t=12.34] deliberately: alignment across ticks is the actual
//     requirement; the sketch is a shape, not a byte spec.
//   * Non-finite values render as deterministic right-aligned tokens ("NaN", "+Inf",
//     "-Inf") — never libc's locale/sign-varying "nan"/"-nan(0x…)".
//   * A value too wide for its column (|v| pathological) re-renders compactly as %.3g
//     ("1e+300") — the column widens slightly rather than exploding to 300+ digits.
//   * Degrees appear here and only here: this is a DISPLAY edge, the one place F3
//     permits radians→degrees.
//
// Sanitization is UNAVOIDABLE BY CONSTRUCTION — the legacy escapeJSONString lesson (a
// helper the caller must remember is a helper that gets forgotten): every byte of
// caller-controlled text (subsystem, message) enters the line through ONE bounded,
// sanitizing append. Control bytes (< 0x20, 0x7F) become '?' so a stray '\n' or ESC
// sequence can never break the one-line-per-call framing or the terminal; bytes ≥ 0x80
// pass through (UTF-8 is welcome). Over-long messages truncate with '…' at a UTF-8
// boundary. There is no unsanitized path to the device.
//
// Concurrency contract (the legacy racing-flush, designed OUT rather than fixed): the
// sink holds NO mutable state — no buffer, no queue, no background flush task. Each call
// formats into a stack-local line and hands the device exactly one write(). Interleaving
// is therefore impossible at this layer; if multiple tasks share one TermSink, ordering
// is whatever the ICharSink's per-call atomicity provides. Nothing here allocates.

#include <string_view>

#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/hal/char_sink.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/math/angle.hpp"

namespace shulib::diag {

class TermSink final : public hal::ITelemetrySink {
public:
    /// Both references must outlive the sink. `clock` stamps log() lines; emit() lines
    /// are stamped from the record itself (see header).
    TermSink(hal::IClock& clock, hal::ICharSink& out) noexcept : clock_{clock}, out_{out} {}

    void log(hal::LogLevel level, std::string_view subsystem, std::string_view message) override {
        Line line;
        appendTimestamp(line, clock_.now().value());
        line.appendLiteral(levelTag(level));
        line.appendLiteral("[");
        line.appendSanitized(subsystem, kMaxTagBytes);
        line.appendLiteral("] ");
        line.appendSanitized(message, kMaxMessageBytes);
        line.appendLiteral("\n");
        out_.write(line.view());
    }

    /// TermSink consumes records — overridden as a pair with emit(), per the seam contract.
    [[nodiscard]] bool wantsRecord() const noexcept override { return true; }

    void emit(const DebugRecord& r) override {
        Line line;
        appendTimestamp(line, r.t.value());
        // A tick belongs to the MOTION channel when a command id is assigned OR
        // the motion-layer state is non-idle. The state clause was added at C1,
        // the first real producer: ids are assigned by the C2 scheduler, so a C1
        // motion running standalone has id 0 but state != 0 — discriminating on
        // id alone rendered an ACTIVE motion as "[LOC] idle" (defect found by
        // the first consumer; a genuinely idle record still has both zero).
        if (r.activeCommandId != 0 || r.activeCommandState != 0) {
            line.appendLiteral("[MOT] cmd#");
            appendUnsigned(line, r.activeCommandId);
            line.appendLiteral("▸");  // ▸ id/state separator, per the §18.3 sketch
            appendUnsigned(line, r.activeCommandState);
            line.appendLiteral(" ");
        } else {
            line.appendLiteral("[LOC] idle ");
        }
        line.appendLiteral("tgt(");
        appendNum(line, r.targetPose.x().value(), 6, 1);
        line.appendLiteral(",");
        appendNum(line, r.targetPose.y().value(), 6, 1);
        line.appendLiteral(",");
        appendNum(line, r.targetPose.heading().degrees(), 6, 1);
        line.appendLiteral("°) err(");  // °
        appendNum(line, r.errorX.value(), 6, 2);
        line.appendLiteral("\",");
        appendNum(line, r.errorY.value(), 6, 2);
        line.appendLiteral("\",");
        appendNum(line, r.errorHeading.value() * kRadToDeg, 5, 1);
        line.appendLiteral("°) v(");
        appendNum(line, r.commanded.vx().value(), 6, 1);
        line.appendLiteral(",");
        appendNum(line, r.commanded.vy().value(), 6, 1);
        line.appendLiteral(",");
        appendNum(line, r.commanded.omega().value(), 5, 2);
        line.appendLiteral(") q=");
        appendNum(line, r.quality, 4, 2);
        if (r.deadReckoning) {
            line.appendLiteral(" DR");
        }
        if (r.strafeFallbackActive) {
            line.appendLiteral(" SFB");
        }
        if (r.clampedThisTick) {
            line.appendLiteral(" CLMP");
        }
        if (r.fault != FaultCode::None) {
            line.appendLiteral(" flt=");
            line.appendLiteral(faultCodeName(r.fault));
        }
        line.appendLiteral("\n");
        out_.write(line.view());
    }

    /// Render the §18.3 one-screen run-summary block. UNSTAMPED by design — the
    /// block is a run artifact, not a timed event (the sketch shows no [t=]); each
    /// of its six lines is one write() (the framing contract). Field renderings
    /// (chosen at C5, each pinned by golden test):
    ///   * heading values render "n/a" when hasHeadingData is false — the block
    ///     never fabricates a 0.0° it has no data for
    ///   * an EMPTY buildHash renders the literal token MISSING (never a
    ///     plausible-looking placeholder — §18.5's loudness rule)
    ///   * the drop counters are ALWAYS shown, zeros included: "dropped 0 rec 0 ln"
    ///     is a positive health claim, not noise (D-2: silence is the bug)
    ///   * first fault carries its latch time ("ODO_STUCK@  4.2s") — the 2am
    ///     root-cause line
    void summarize(const RunSummary& s) override {
        {
            Line line;
            line.appendLiteral("── RUN SUMMARY ───────────────────────────────────────────\n");
            out_.write(line.view());
        }
        {
            Line line;
            line.appendLiteral(" motions ");
            appendUnsigned(line, static_cast<unsigned long>(s.motionsStarted));
            line.appendLiteral(" · settled ");
            appendUnsigned(line, static_cast<unsigned long>(s.motionsSettled));
            line.appendLiteral(" · timeout ");
            appendUnsigned(line, static_cast<unsigned long>(s.motionsTimedOut));
            line.appendLiteral(" · cancelled ");
            appendUnsigned(line, static_cast<unsigned long>(s.motionsCancelled));
            line.appendLiteral(" · aborted ");
            appendUnsigned(line, static_cast<unsigned long>(s.motionsAborted));
            line.appendLiteral("\n");
            out_.write(line.view());
        }
        {
            Line line;
            line.appendLiteral(" heading max ");
            if (s.hasHeadingData) {
                appendNum(line, s.headingMax.value() * kRadToDeg, 4, 1);
                line.appendLiteral("° final ");
                appendNum(line, s.headingFinal.value() * kRadToDeg, 4, 1);
                line.appendLiteral("°");
            } else {
                appendPadded(line, "n/a", 4);
                line.appendLiteral("  final ");
                appendPadded(line, "n/a", 4);
                line.appendLiteral(" ");
            }
            line.appendLiteral(" · gating rejects ");
            appendUnsigned(line, static_cast<unsigned long>(s.gatingRejects));
            line.appendLiteral(" · brownout ");
            line.appendLiteral(s.brownout ? "YES" : "no");
            line.appendLiteral("\n");
            out_.write(line.view());
        }
        {
            Line line;
            line.appendLiteral(" worst loop dt ");
            appendNum(line, s.worstLoopDt.value() * 1000.0, 6, 1);
            line.appendLiteral("ms · first fault ");
            if (s.firstFault == FaultCode::None) {
                line.appendLiteral("none");
            } else {
                line.appendLiteral(faultCodeName(s.firstFault));
                line.appendLiteral("@");
                appendNum(line, s.firstFaultTime.value(), 5, 1);
                line.appendLiteral("s");
            }
            line.appendLiteral(" · dropped ");
            appendUnsigned(line, s.droppedRecords);
            line.appendLiteral(" rec ");
            appendUnsigned(line, s.droppedLines);
            line.appendLiteral(" ln\n");
            out_.write(line.view());
        }
        {
            Line line;
            line.appendLiteral(" build ");
            if (s.buildHash().empty()) {
                line.appendLiteral("MISSING");  // loud, never plausible (§18.5)
            } else {
                line.appendSanitized(s.buildHash(), kMaxHashBytes);
            }
            line.appendLiteral(" · routine \"");
            line.appendSanitized(s.routineId(), kMaxTagBytes);
            line.appendLiteral("\" · batt ");
            appendNum(line, s.batteryStart.value(), 4, 1);
            line.appendLiteral("→");
            appendNum(line, s.batteryEnd.value(), 4, 1);
            line.appendLiteral("V\n");
            out_.write(line.view());
        }
        {
            Line line;
            line.appendLiteral(
                "──────────────────────────────────────────────────────────\n");
            out_.write(line.view());
        }
    }

private:
    static constexpr std::size_t kMaxTagBytes = 16;       ///< subsystem tags are short by design
    static constexpr std::size_t kMaxMessageBytes = 200;  ///< structured fields, not essays (§18)
    static constexpr std::size_t kMaxHashBytes = 47;      ///< full SHA + "-dirty" (RunSummary)
    static constexpr double kRadToDeg = 180.0 / math::Angle::kPi;

    /// The bounded line + fixed-width numerics moved to diag/line_format.hpp at C5
    /// (verbatim — the A1 goldens are the bit-identity proof) so the result line and
    /// the summary block share ONE formatting definition with the tick stream.
    /// Unqualified appendNum/appendTimestamp/… calls resolve by ADL on lineformat::Line.
    using Line = lineformat::Line;

    static const char* levelTag(hal::LogLevel level) noexcept {
        switch (level) {
            case hal::LogLevel::Error: return "[ERROR]";
            case hal::LogLevel::Warn: return "[WARN]";
            case hal::LogLevel::Info: return "";  // the common case stays quiet (§18.3)
            case hal::LogLevel::Debug: return "[DEBUG]";
            case hal::LogLevel::Trace: return "[TRACE]";
        }
        return "[?]";
    }

    hal::IClock& clock_;
    hal::ICharSink& out_;
};

}  // namespace shulib::diag
