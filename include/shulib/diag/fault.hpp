#pragma once
//
// Fault discipline (master plan §18.4; WS13, chunk A1) — the stable numeric fault-code
// enum and the latched first-fault capture. The rule this file enforces: FAULTS LOG AND
// RECOVER, THEY NEVER CRASH. A NaN pose, a sensor pathology, or a loop overrun raises a
// code and the run continues on a safe fallback; nothing in this file can abort an auton.
//
// Why the FIRST fault is latched distinctly from the cascade: one root cause (say a NaN
// pose) typically triggers a burst of follow-on faults (odom stuck, gate rejects, motion
// timeout). At 2am you need the ROOT CAUSE, and it is the first fault, not the loudest or
// the last. FaultLatch therefore records the first (code + time) immutably until clear(),
// while still counting and logging every subsequent fault in the cascade.
//
// Why the enum values are EXPLICIT and never reordered: these numbers go on the F9 wire
// (the SHUL/2 serialization of DebugRecord, frozen at H1) and into SdSink blackbox files
// (E1). A reorder would silently re-label historical logs. The numeric values are pinned
// by test/fault_test.cpp — reordering turns the suite red. New codes are ADDED at the
// end, never inserted.
//
// Concurrency contract (the legacy logger's racing flush, designed against): FaultLatch
// is owned and mutated by ONE task (the control loop). It has no background task, no
// buffering, and no flush — raise() formats into a stack buffer and hands one line to the
// sink synchronously on the caller's task. Cross-task use requires external serialization.
//
// raise() is noexcept BY CONTRACT — it is the error path, so it must be unconditionally
// safe to call from any failure handler. A sink or clock that throws violates its own
// interface contract ("implementations MUST NOT throw"); raise() swallows such a throw
// (after the latch state is already updated) rather than crashing the run over a broken
// diagnostic channel. The latch always latches, even if logging fails.

#include <array>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include "shulib/hal/clock.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

/// Stable numeric fault codes (§18.4). WIRE-STABLE: explicit values, append-only —
/// pinned by test. `None` (0) means "no fault" and is not raisable.
enum class FaultCode : std::uint16_t {
    None = 0,           ///< no fault (the DebugRecord default; never latched)
    Precondition = 1,   ///< SHULIB_PRECONDITION violated (routed here on-robot via the
                        ///< check.hpp policy seam; host builds throw instead — §18.4)
    NanPose = 2,        ///< a non-finite pose/quantity was caught and recovered from
    LoopOverrun = 3,    ///< a control tick blew its dt budget (corrupts PID dt → §18.4)
    OdoStuck = 4,       ///< odometry implausible / wheel stuck (raised by the C/E layers)
    ImuLost = 5,        ///< IMU not ready / lost mid-run
    GpsGateReject = 6,  ///< a GPS fix was rejected by the fusion gate (E2)
    Brownout = 7,       ///< battery collapsed below the brownout threshold
    MotionTimeout = 8,  ///< a motion hit its watchdog (FAULT_ABORT / TimedOut, C1/C2)
    MotorOverTemp = 9,  ///< a motor crossed the thermal-throttle threshold (~55 °C) —
                        ///< the droop corrupts kS/kV/kA, so it must be visible (§8/§18.4;
                        ///< APPENDED at chunk A3, per the append-only rule above)
    Implausible = 10,   ///< a physical-plausibility invariant fired: per-tick pose delta
                        ///< beyond the drivetrain's physical maximum, a commanded speed
                        ///< outside its budget, or a wheel volt inconsistent with the
                        ///< battery ceiling (diagnostics-plan D-5 — FiniteGuard's
                        ///< log-and-recover posture extended beyond finiteness; APPENDED
                        ///< at chunk C5, per the append-only rule above)
    MechanismStalled = 11,  ///< a mechanism's stall detector tripped: stall-grade current
                            ///< with the shaft not turning, held past the persistence
                            ///< window — a jam or mechanical bind (manipulation layer,
                            ///< T6: the one mechanism failure that IS a pathology; an
                            ///< operation merely timing out raises nothing — see
                            ///< manipulation/mechanism_op.hpp. Lands on the CONTINUE
                            ///< side of the C2 abort mask by default: a jammed intake
                            ///< must not abort a drive. APPENDED at chunk F1, per the
                            ///< append-only rule above)
};

/// The §18.4 spelling of each code, for TermSink lines and the run summary.
/// Never returns null; an out-of-range cast renders as "UNKNOWN" (never a crash).
[[nodiscard]] constexpr const char* faultCodeName(FaultCode code) noexcept {
    switch (code) {
        case FaultCode::None: return "NONE";
        case FaultCode::Precondition: return "PRECONDITION";
        case FaultCode::NanPose: return "NAN_POSE";
        case FaultCode::LoopOverrun: return "LOOP_OVERRUN";
        case FaultCode::OdoStuck: return "ODO_STUCK";
        case FaultCode::ImuLost: return "IMU_LOST";
        case FaultCode::GpsGateReject: return "GPS_GATE_REJECT";
        case FaultCode::Brownout: return "BROWNOUT";
        case FaultCode::MotionTimeout: return "MOTION_TIMEOUT";
        case FaultCode::MotorOverTemp: return "MOTOR_OVER_TEMP";
        case FaultCode::Implausible: return "IMPLAUSIBLE";
        case FaultCode::MechanismStalled: return "MECHANISM_STALLED";
    }
    return "UNKNOWN";
}

/// Latched first-fault capture + cascade counting (§18.4). See the header note for the
/// root-cause rationale and the noexcept/concurrency contracts.
class FaultLatch {
public:
    /// Both references must outlive the latch. The sink receives one Error-level line per
    /// raised fault; the clock timestamps the first fault.
    FaultLatch(hal::ITelemetrySink& sink, hal::IClock& clock) noexcept
        : sink_{sink}, clock_{clock} {}

    /// Raise a fault: latch it (first-fault immutably), count it, and log one structured
    /// Error line — `fault=<NAME> n=<count>[ FIRST] <detail>`. Raising FaultCode::None is
    /// a defensive NO-OP (it is "no fault", and the error path must never crash — a
    /// precondition throw here would turn a bad raise into a dead robot).
    void raise(FaultCode code, std::string_view subsystem, std::string_view detail) noexcept {
        if (code == FaultCode::None) {
            return;
        }
        // Latch FIRST, log second — the latch must survive even a throwing sink/clock.
        ++count_;
        last_ = code;
        const auto idx = static_cast<std::size_t>(code);
        if (idx < kCodeSlots && perCode_[idx] < UINT16_MAX) {
            ++perCode_[idx];  // per-code tally (see raiseCount below)
        }
        const bool isFirst = (count_ == 1);
        if (isFirst) {
            first_ = code;
        }
        try {
            if (isFirst) {
                firstTime_ = clock_.now();
            }
            char buf[176];
            std::snprintf(buf, sizeof buf, "fault=%s n=%d%s%s%.*s", faultCodeName(code), count_,
                          isFirst ? " FIRST" : "", detail.empty() ? "" : " ",
                          static_cast<int>(detail.size()), detail.empty() ? "" : detail.data());
            sink_.log(hal::LogLevel::Error, subsystem, buf);
        } catch (...) {
            // A throwing sink/clock violates its interface contract. The fault is already
            // latched; swallowing here keeps "faults never crash" true unconditionally.
        }
    }

    /// True once ANY fault has been raised since construction/clear(), and true for the rest of
    /// the run thereafter — this is a LATCH, not a live "is something wrong right now" query, and
    /// nothing but clear() lowers it. Raising FaultCode::None is a no-op and never sets it. For
    /// triage read firstFault(): the root cause is the first fault, not the last or the loudest.
    [[nodiscard]] bool hasFault() const noexcept { return count_ > 0; }
    /// How many times `code` has been raised since construction/clear(). ADDED at
    /// chunk C2 (additive, like the A3 MotorOverTemp append): the scheduler's fault
    /// policy must distinguish a fault raised DURING the current motion from one
    /// latched by an earlier motion — a since-clear bitmask cannot see a RE-raise
    /// (a dead encoder that faulted in motion 1 must still abort motion 2), so the
    /// latch keeps a per-code tally. Saturates at UINT16_MAX; codes beyond the
    /// fixed slot capacity (far past today's 11) count only in faultCount().
    [[nodiscard]] int raiseCount(FaultCode code) const noexcept {
        const auto idx = static_cast<std::size_t>(code);
        return idx < kCodeSlots ? static_cast<int>(perCode_[idx]) : 0;
    }
    /// The ROOT CAUSE: the first fault raised since construction/clear() (None if none).
    [[nodiscard]] FaultCode firstFault() const noexcept { return first_; }
    /// When the first fault was raised (Time{0} if none, or if the clock threw).
    [[nodiscard]] units::Time firstFaultTime() const noexcept { return firstTime_; }
    /// The most recent fault in the cascade (None if none) — display only, never triage.
    [[nodiscard]] FaultCode lastFault() const noexcept { return last_; }
    /// Total faults raised since construction/clear() (first + cascade).
    [[nodiscard]] int faultCount() const noexcept { return count_; }

    /// Reset between runs. The first-fault latch is immutable WITHIN a run by design;
    /// only an explicit new-run boundary may clear it.
    void clear() noexcept {
        first_ = FaultCode::None;
        last_ = FaultCode::None;
        firstTime_ = units::Time{0.0};
        count_ = 0;
        perCode_.fill(0);
    }

private:
    /// Per-code tally capacity: covers FaultCode values 0..31 — nearly triple
    /// today's 11, and the enum is append-only so growth is deliberate and
    /// visible.
    static constexpr std::size_t kCodeSlots = 32;

    hal::ITelemetrySink& sink_;
    hal::IClock& clock_;
    FaultCode first_ = FaultCode::None;
    FaultCode last_ = FaultCode::None;
    units::Time firstTime_{0.0};
    int count_ = 0;
    std::array<std::uint16_t, kCodeSlots> perCode_{};
};

}  // namespace shulib::diag
