#pragma once
//
// RestCapture — shulib Calibrate's REST step: the robot sits still, unpowered, for a minute while
// the IMU's drift and noise, the encoders' noise at rest and the loop's timing are measured (chunk
// R5a Part A, 2026-09-29 — the roadmap's R4 stationary capture, moved onto the robot). Also the
// input types and the side readings the GAINS step (drive_characterizer.hpp) shares with it, and
// settleFloorReport(), which says how far each settle floor of MotionConfig clears what was measured.
//
// ── PROVEN ON THE HOST TEST PLANT; IT HAS NEVER RUN ON A ROBOT ──────────────────────
// Every property below is shown against the A2 host test plant with the A3 hostile IMU (white
// heading noise, a drawn per-boot rate bias) and a test-local random-walk heading. The plant CANNOT
// show the V5 IMU's noise STRUCTURE — white heading noise or integrated rate noise — nor its real
// noise, drift or continuity (A4 register HA-20/21/22/160/173). Every number in RestCaptureConfig
// is INVENTED (HA-170); the rest floors it shares with the drivetrain calibrator keep their HA-136.
//
// ── What it measures, and the formula behind each (rad, rad/s, in/s, s) ──────────────
// Per tick of the capture: the heading, UNWRAPPED — accumulated through Angle::errorTo, never taken
// as end minus start (drivetrain_calibrator.hpp's rule 3: a wrapped heading crossing ±π would jump
// by 2π); the per-tick heading step Δh (the same errorTo); the IMU's yaw rate; each side's median
// encoder speed and travel; the time.
//   * DRIFT = the least-squares SLOPE of the unwrapped heading against time (online, Welford's
//     centred sums — no Σt² − (Σt)²/n cancellation). Its standard error is reported under BOTH noise
//     models, because which one holds is exactly what the host cannot show:
//       - WHITE heading noise (the host's hostile IMU; HA-21): σ_b = σ_line / √(Σ(t − t̄)²), which
//         for N evenly spaced ticks over T is σ_line·√12/(√N·T) — ≈ 0.002 °/min at 60 s and
//         100 Hz with HA-21's 0.05°;
//       - a RANDOM WALK (integrated rate noise — a gyro's heading is the integral of its rate;
//         HA-22): σ_b = σ_Δ·√(1.2/(dt·T)) — the slope of a Wiener process of diffusion σ_Δ²/dt
//         over T has variance 1.2·(σ_Δ²/dt)/T — ≈ 0.42 °/min at 60 s, 0.19 °/min at 300 s with
//         HA-22's 0.5 °/s per 10 ms sample. That is HA-20's typical drift itself: under that
//         structure one boot cannot separate its bias from its noise, and only the spread across
//         boots can (A4 register HA-173).
//     Neither is picked automatically: each overstates the other case badly.
//   * the STRUCTURE RATIO ρ = σ_Δ/(√2·σ_line): white heading noise gives Δ = n[k] − n[k−1], so
//     σ_Δ = √2·σ and ρ ≈ 1; a random walk has a small step and a large wander about any line, so
//     ρ ≪ 1 (about 0.04 at HA-22's numbers over 60 s). S3's evidence for HA-173's second half.
//     UNDEFINED when σ_line is 0 (no heading noise at all — the quiet host plant): reported as 0,
//     which is not random-walk evidence, so RestSummary::structureRatioDefined() says which.
//   * HEADING NOISE σ_line: the residual about the fitted LINE, never about the mean — a drift of
//     10 °/min inflates the latter to degrees.
//   * σ_Δ/dt, with σ_Δ the step's standard deviation about ITS mean and dt the mean tick: the rate
//     noise SettledUtil actually differentiates (control/settled_util.hpp: |error − previous| / dt).
//   * the yaw-rate channel's mean (a second drift estimate) and its σ;
//   * each side's median encoder speed σ; the largest |Δh| (HA-160 evidence); the tick timing;
//   * the encoders' rotation at the end and at its largest, and their largest translation (below).
//
// THE DRIFT IS NEVER APPLIED as a heading correction: it is a per-boot bias (HA-20), so one boot's
// number is wrong at the next boot (A4 register HA-173). Nothing here feeds a settle tolerance, the
// EKF or the heading either — recorded and reported only (R5a's decision D5).
//
// ── The phases ──────────────────────────────────────────────────────────────────────
//   WaitImu   wait for imu.isReady(), bounded by imuReadyTimeout.
//   Settle    the drivetrain calibrator's REST definition: every member under restFloor AND the
//             IMU's yaw rate under restYawRateFloor, held restSettleTime, bounded by restTimeout.
//   Capture   `duration` of samples. Refused if the robot MOVES or TURNS, judged from the
//             ENCODERS — never from the IMU, whose apparent rotation at rest IS the drift being
//             measured (a 10 °/min bias "turns" it 10° in a minute).
//   Done | Aborted.
// UNPOWERED throughout: start() and every refusal leave every member at 0 V + Coast, so the robot
// stays pushable (the calibrator's rule 6), and nothing here ever commands a volt.
//
// ── Moved and Turned, and why they are judged on the decomposed motion ──────────────
// Turned: the encoders' rotation (dR − dL)/W past turnBound. Moved: the encoders' TRANSLATION, the
// centre's travel (dL + dR)/2, past moveBound. Both are measured from the capture's FIRST sample,
// and both are judged in either direction (|·|). R5a's design wrote "either side's travel" for
// Moved; with its own numbers (W = 12.5 in, 0.25 in, 3°) a turn in place moves each side 0.25 in
// at 2.3° of turn, so Moved would always fire first and Turned — the refusal that names a turn —
// could never name one. Split into translation and rotation, each refusal says what happened.
// THE COST OF THAT SPLIT, kept on purpose (R5a's review of 2026-10-04): the rotation a capture
// ADMITS rose — a turn in place from 2.29° (refused as Moved under the design's rule) to 3.0°, a
// pivot about one wheel from 1.15° to 2.29° (it is still refused as Moved, at 0.25 in of centre
// travel). The bound that should decide this is turnBound itself, sized on purpose — not an
// accident of the other bound's wording; its size is the team lead's (A4 register HA-170).
//
// ── What a rotation UNDER turnBound does to the drift — not refused, but RECORDED ────────
// A robot that really turns turns the IMU with it, and the drift's slope cannot tell that from
// drift. A heading offset ψ(t) with |ψ| <= M over [0, T] moves the least-squares slope by
//   Σ(t − t̄)·ψ(t) / Σ(t − t̄)²  <=  M·Σ|t − t̄| / Σ(t − t̄)²  =  3·M·(N − 1)/(N·T)  <  3·M/T
// for N evenly spaced ticks (Σ|t − t̄| = h(N² − 1)/4 and Σ(t − t̄)² = h²N(N² − 1)/12 at spacing h,
// odd N; even N gives 3·M/((1 + 1/N)·T)) — approached by a robot turned to −M just after the
// capture starts and to +M at mid-capture. ONE step of size A at time t0 moves it by
// 6·A·t0·(T − t0)/T³: 1.5·A/T at mid-capture. At the defaults (3°, 60 s) that is up to 4.5 °/min
// for one nudge and 9 °/min worst case — 4.5 to 9 times HA-20's whole 1 °/min bound, with no
// refusal (the host test plant reproduces 1.5·A/T within 0.03 %: development log, shulib-v2
// branch, 2026-10-04). So the capture RECORDS what the encoders saw — RestSummary::encoderTurnRad
// (at the end), encoderTurnMaxRad and encoderMoveMaxIn — and RestSummary::driftTurnBiasBound() is
// 3·encoderTurnMaxRad/T: a drift not well clear of it says nothing. A nudge also drags the
// structure ratio toward 0 (its step inflates σ_line, not σ_Δ), so it can read as random-walk
// evidence (HA-173). Whether to shrink turnBound, or to regress the IMU heading MINUS the encoder
// rotation, is an open decision for the team lead; neither is done here.
//
// ── A side that gives no reading ─────────────────────────────────────────────────────
// A side's speed and travel are the median over its members (hal::medianInPlace). When EVERY
// member's reading is not finite on a tick, that median is NaN — and one NaN added to a running
// σ, or judged against a bound, would have finished a 60 s capture Done with a summary its own
// valid() refuses (found 2026-10-04, R5a's review: a whole side's velocity() NaN on ONE tick did
// exactly that). So such a reading is LEFT OUT and counted (unreadSides()): a non-finite speed is
// not added to that side's σ; a non-finite travel judges and records no Moved/Turned on that tick
// — the travel is measured from the capture's first sample, so a push made during the gap still
// shows at the next tick that reads. The characterizer's fit does the same with a non-finite sample
// (FitExclusion::NonFinite). The IMU's heading is not screened here: a non-finite heading is the
// IMU adapter's to screen (A4 register HA-08), as it is for every other heading consumer.
//
// Tick-driven and non-blocking (10 ms on the robot); allocation-free; PROS-free and sim-free;
// raises no faults; every runtime string is ASCII with no '%' (a panel paints them).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <span>

#include "shulib/calibration/drive_characterization.hpp"
#include "shulib/calibration/drivetrain_calibrator.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/tick_timing_stats.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::calibration {

/// One motor of a drive side, as the GAINS and REST steps read and command it.
struct SideMember {
    /// The motor — non-owning, must outlive the step. In shulib Calibrate its adapter is built with
    /// a POSITIVE port and `sign` is applied HERE, once (drivetrain_calibrator.hpp's rule 1); a
    /// program whose adapters already carry the sign in the port passes sign +1.
    hal::IMotor* motor = nullptr;
    /// +1 or −1: multiplies every command sent to the motor and every reading taken from it.
    std::int8_t sign = 1;
    /// The physical port, 1..21 — for the messages that name a port. Positive.
    std::int8_t port = 0;
};

/// One drive side: its members and its scale (inches of surface travel per radian of shaft) —
/// both from the drivetrain record (the description's DriveSideDescription).
struct DriveSide {
    /// The side's members, 1..hal::kMaxSmartPorts of them.
    std::span<const SideMember> members{};
    /// The side's inches per radian, > 0 — the description's measured scale. Every speed and travel
    /// the steps report, and so every gain the GAINS step fits, is in THIS scale's inches (R5a's
    /// landmine 7: a re-calibrated drivetrain changes every gain's units).
    units::Length inchesPerRadian{0.0};
};

/// The side's surface speed now, in/s: the MEDIAN over its members of sign × velocity() — the
/// statistic hal::MotorGroup reports, through the same hal::medianInPlace — times its scale.
[[nodiscard]] inline double sideSpeed(const DriveSide& side) {
    double v[hal::kMaxSmartPorts];
    const std::size_t n = side.members.size();
    for (std::size_t k = 0; k < n; ++k) {
        const SideMember& m = side.members[k];
        v[k] = static_cast<double>(m.sign) * m.motor->velocity().value();
    }
    return hal::medianInPlace(v, n) * side.inchesPerRadian.value();
}

/// The side's position now, in: the MEDIAN over its members of sign × position(), times its scale
/// — cumulative, as IMotor::position() is (the raw CSV's column; a travel is sideTravel()).
[[nodiscard]] inline double sidePosition(const DriveSide& side) {
    double v[hal::kMaxSmartPorts];
    const std::size_t n = side.members.size();
    for (std::size_t k = 0; k < n; ++k) {
        const SideMember& m = side.members[k];
        v[k] = static_cast<double>(m.sign) * m.motor->position().value();
    }
    return hal::medianInPlace(v, n) * side.inchesPerRadian.value();
}

/// The side's travel since `base` (each member's position() at the baseline, parallel to
/// side.members), in: the MEDIAN of the members' sign-corrected DELTAS, times the scale — the
/// calibrator's sideTravel(): members whose encoders started at different counts still agree.
[[nodiscard]] inline double sideTravel(const DriveSide& side, const double* base) {
    double v[hal::kMaxSmartPorts];
    const std::size_t n = side.members.size();
    for (std::size_t k = 0; k < n; ++k) {
        const SideMember& m = side.members[k];
        v[k] = static_cast<double>(m.sign) * (m.motor->position().value() - base[k]);
    }
    return hal::medianInPlace(v, n) * side.inchesPerRadian.value();
}

/// Check one side's shape — the constructors' shared precondition: 1..21 members, none null, sign
/// ±1, port 1..21, scale finite and > 0.
[[nodiscard]] inline bool driveSideValid(const DriveSide& side) noexcept {
    if (side.members.empty() || side.members.size() > hal::kMaxSmartPorts) {
        return false;
    }
    for (const SideMember& m : side.members) {
        if (m.motor == nullptr || (m.sign != 1 && m.sign != -1) || m.port < 1
            || m.port > static_cast<std::int8_t>(hal::kMaxSmartPorts)) {
            return false;
        }
    }
    return std::isfinite(side.inchesPerRadian.value()) && side.inchesPerRadian.value() > 0.0;
}

/// Every number the REST step uses. The capture's own are INVENTED for a first run
/// (PROVISIONAL, A4: HA-170); the rest definition and the IMU wait are the drivetrain calibrator's,
/// copied from CalibratorConfig's defaults so the two steps call the same stillness the same thing
/// (A4: HA-136).
struct RestCaptureConfig {
    /// How long the capture samples. 60 s — a CHOICE: under a random-walk heading a longer capture
    /// helps only as 1/√T (HA-173). PROVISIONAL (A4: HA-170).
    units::Time duration{60.0};
    /// The encoders' TRANSLATION (the centre's travel) past this refuses as Moved. PROVISIONAL
    /// (A4: HA-170).
    units::Length moveBound{0.25};
    /// The encoders' ROTATION past this refuses as Turned. 3°. PROVISIONAL (A4: HA-170).
    units::AngleDim turnBound{3.0 * kDegreesToRadians};
    /// settleFloorReport() warns when a floor clears the measured noise by less than this factor.
    /// PROVISIONAL (A4: HA-170).
    double settleMarginWarn = 2.0;
    /// |velocity| under this is a still motor (the calibrator's; A4: HA-136).
    units::AngularVelocity restFloor{CalibratorConfig{}.restFloor};
    /// |IMU yaw rate| under this is a still robot (the calibrator's; A4: HA-136).
    units::AngularVelocity restYawRateFloor{CalibratorConfig{}.restYawRateFloor};
    /// How long stillness must hold before the capture starts (the calibrator's; A4: HA-136).
    units::Time restSettleTime{CalibratorConfig{}.restSettleTime};
    /// How long to wait for stillness before refusing (the calibrator's; A4: HA-136).
    units::Time restTimeout{CalibratorConfig{}.restTimeout};
    /// How long to wait for the IMU's boot calibration (the calibrator's; A4: HA-136).
    units::Time imuReadyTimeout{CalibratorConfig{}.imuReadyTimeout};
    /// The loop timing's nominal period and late budget (HA-32; LoopMonitor's 15 ms).
    diag::TickTimingConfig timing{};

    /// True when every field is finite and positive where it must be (settleMarginWarn >= 1,
    /// restTimeout > restSettleTime), else false naming the field in `why` (may be null).
    [[nodiscard]] bool valid(char* why, std::size_t cap) const {
        auto fail = [&](const char* text) {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap, "%s", text);
            }
            return false;
        };
        auto pos = [](double v) { return std::isfinite(v) && v > 0.0; };
        if (!pos(duration.value())) {
            return fail("RestCaptureConfig: duration must be finite and > 0");
        }
        if (!pos(moveBound.value()) || !pos(turnBound.value())) {
            return fail("RestCaptureConfig: moveBound and turnBound must be finite and > 0");
        }
        if (!std::isfinite(settleMarginWarn) || !(settleMarginWarn >= 1.0)) {
            return fail("RestCaptureConfig: settleMarginWarn must be finite and >= 1");
        }
        if (!pos(restFloor.value()) || !pos(restYawRateFloor.value())
            || !std::isfinite(restSettleTime.value()) || !(restSettleTime.value() >= 0.0)
            || !pos(imuReadyTimeout.value())) {
            return fail("RestCaptureConfig: the rest floors, settle time and IMU wait must be "
                        "finite and positive");
        }
        if (!std::isfinite(restTimeout.value()) || !(restTimeout.value() > restSettleTime.value())) {
            return fail("RestCaptureConfig: restTimeout must exceed restSettleTime");
        }
        if (!pos(timing.nominal.value()) || !(timing.budget.value() > timing.nominal.value())) {
            return fail("RestCaptureConfig: timing.budget must exceed timing.nominal > 0");
        }
        return true;
    }
};

/// Where a capture has got to. A capture ends in exactly one of Done or Aborted.
enum class RestPhase : std::uint8_t {
    Idle,     ///< constructed, start() not yet called
    WaitImu,  ///< waiting for the IMU's boot calibration
    Settle,   ///< waiting for the rest definition to hold restSettleTime
    Capture,  ///< sampling for `duration`
    Done,     ///< summary() is the capture's
    Aborted,  ///< refused or aborted; reason() says why; every member at 0 V + Coast
};

/// WHY a capture ended in Aborted — a code a panel words and a test asserts. APPEND-ONLY.
enum class RestRefusal : std::uint8_t {
    None,         ///< not refused
    Aborted,      ///< abort() — the operator, or the program on the operator's behalf
    ImuNotReady,  ///< the IMU never became ready, or stopped being ready after it was
    RestTimeout,  ///< the robot never came to rest within restTimeout
    Moved,        ///< the encoders' translation passed moveBound during the capture
    Turned,       ///< the encoders' rotation passed turnBound during the capture
};

/// The stationary capture (header). Non-owning: the motors (through the sides' spans), the IMU and
/// the clock must outlive it. Not copyable (it holds references). Construct a new one to capture
/// again (the calibrator's RUN AGAIN rule: nothing carried over by construction).
class RestCapture {
public:
    /// Preconditions: both sides valid (driveSideValid), no port listed twice across them, the
    /// track width finite and > 0 (the description's effective width), config.valid().
    RestCapture(DriveSide left, DriveSide right, units::Length trackWidth, hal::IImu& imu,
                hal::IClock& clock, const RestCaptureConfig& config = {})
        : sides_{left, right}, width_{trackWidth.value()}, imu_{imu}, clock_{clock}, cfg_{config},
          timing_{config.timing} {
        SHULIB_PRECONDITION(driveSideValid(left) && driveSideValid(right),
                            "RestCapture: each side needs 1..21 non-null members, sign +-1, port "
                            "1..21 and a positive scale");
        SHULIB_PRECONDITION(std::isfinite(width_) && width_ > 0.0,
                            "RestCapture: the track width must be finite and > 0");
        SHULIB_PRECONDITION(cfg_.valid(nullptr, 0), "RestCapture: the RestCaptureConfig is invalid");
        SHULIB_PRECONDITION(portsUnique(), "RestCapture: a port is listed twice");
    }

    /// Not copyable: it holds references to hardware it does not own.
    RestCapture(const RestCapture&) = delete;
    /// Not copy-assignable, for the same reason.
    RestCapture& operator=(const RestCapture&) = delete;

    /// Begin: every member to 0 V + Coast (the robot stays pushable), then WaitImu. Precondition:
    /// the capture is Idle.
    void start() {
        SHULIB_PRECONDITION(phase_ == RestPhase::Idle, "RestCapture::start: already started");
        safeState();
        enter(RestPhase::WaitImu);
    }

    /// One tick. Cheap and non-blocking; the caller paces it (10 ms). A no-op before start() and
    /// after the capture ends.
    void tick() {
        if (phase_ == RestPhase::Idle || phase_ == RestPhase::Done || phase_ == RestPhase::Aborted) {
            return;
        }
        const units::Time now = clock_.now();
        switch (phase_) {
            case RestPhase::WaitImu: tickWaitImu(now); break;
            case RestPhase::Settle: tickSettle(now); break;
            case RestPhase::Capture: tickCapture(now); break;
            default: break;
        }
    }

    /// Stop now: every member to 0 V + Coast, Aborted with RestRefusal::Aborted and `why` as the
    /// reason. Allowed in any phase, including Done (it then discards the summary).
    void abort(const char* why = "aborted by the operator") {
        refuse(RestRefusal::Aborted, 0, why != nullptr ? why : "aborted");
    }

    /// Where the capture has got to.
    [[nodiscard]] RestPhase phase() const noexcept { return phase_; }
    /// Which refusal ended it (None while it runs or once it is Done).
    [[nodiscard]] RestRefusal refusal() const noexcept { return refusal_; }
    /// Why it was refused — empty while it runs or once it is Done. ASCII, no '%'.
    [[nodiscard]] const char* reason() const noexcept { return reason_; }
    /// The port the refusal names (a motor that would not come to rest), or 0.
    [[nodiscard]] int refusalPort() const noexcept { return refusalPort_; }
    /// Seconds of capture so far (0 before Capture) — a panel's live number.
    [[nodiscard]] double captureSeconds() const noexcept { return capturedS_; }
    /// How many (tick, side) readings the capture LEFT OUT because a side had no finite reading —
    /// its speed (then left out of that side's speed σ) or its travel (then that tick judges and
    /// records no encoder motion). The header's "A side that gives no reading". 0 on a V5, where
    /// the motor adapter screens a failed read first (A4 register HA-08); a program prints it
    /// beside the summary when it is not.
    [[nodiscard]] std::uint32_t unreadSides() const noexcept { return unread_; }
    /// The capture's summary. Precondition: phase() == Done.
    [[nodiscard]] const RestSummary& summary() const {
        SHULIB_PRECONDITION(phase_ == RestPhase::Done,
                            "RestCapture::summary: the capture did not finish -- read reason()");
        return summary_;
    }
    /// The config in force.
    [[nodiscard]] const RestCaptureConfig& config() const noexcept { return cfg_; }

private:
    void tickWaitImu(units::Time now) {
        if (imu_.isReady()) {
            enter(RestPhase::Settle);
            return;
        }
        if (now.value() - phaseStart_ > cfg_.imuReadyTimeout.value()) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg, "the IMU never became ready within %.1f s",
                          cfg_.imuReadyTimeout.value());
            refuse(RestRefusal::ImuNotReady, 0, msg);
        }
    }

    void tickSettle(units::Time now) {
        if (!imu_.isReady()) {
            refuse(RestRefusal::ImuNotReady, 0, "the IMU stopped being ready while waiting for rest");
            return;
        }
        const int moving = firstMovingPort();
        if (moving == 0 && std::abs(imu_.yawRate().value()) <= cfg_.restYawRateFloor.value()) {
            if (!resting_) {
                resting_ = true;
                restSince_ = now.value();
            }
            if (now.value() - restSince_ >= cfg_.restSettleTime.value()) {
                beginCapture(now);
            }
            return;
        }
        resting_ = false;
        if (now.value() - phaseStart_ > cfg_.restTimeout.value()) {
            char msg[kReasonCap];
            if (moving != 0) {
                std::snprintf(msg, sizeof msg,
                              "port %d is still turning after %.1f s of waiting for rest -- a motor "
                              "that reports motion while nothing is driven is a frozen reading or a "
                              "moving robot",
                              moving, cfg_.restTimeout.value());
            } else {
                std::snprintf(msg, sizeof msg,
                              "the robot is still rotating at %.1f deg/s after %.1f s of waiting for "
                              "rest -- hands off the robot, and keep it on the floor",
                              imu_.yawRate().value() / kDegreesToRadians, cfg_.restTimeout.value());
            }
            refuse(RestRefusal::RestTimeout, moving, msg);
        }
    }

    void beginCapture(units::Time now) {
        for (std::size_t s = 0; s < 2; ++s) {
            for (std::size_t k = 0; k < sides_[s].members.size(); ++k) {
                base_[s][k] = sides_[s].members[k].motor->position().value();
            }
        }
        t0_ = now.value();
        n_ = 0;
        enter(RestPhase::Capture);
        tickCapture(now);  // the first sample is taken on the tick that started the capture
    }

    void tickCapture(units::Time now) {
        if (!imu_.isReady()) {
            refuse(RestRefusal::ImuNotReady, 0, "the IMU stopped being ready during the capture");
            return;
        }
        (void)timing_.tick(now);
        const double t = now.value() - t0_;
        const math::Angle h = imu_.heading();
        if (n_ == 0) {
            heading_ = 0.0;  // the unwrapped heading is measured from the first sample
        } else {
            const double step = prevHeading_.errorTo(h);  // rule 3: per tick, never end minus start
            heading_ += step;
            stepStats_.add(step);
            maxStep_ = std::max(maxStep_, std::abs(step));
        }
        prevHeading_ = h;
        // The heading-vs-time line, by Welford's centred sums (header).
        ++n_;
        const double nn = static_cast<double>(n_);
        const double dtv = t - meanT_;
        meanT_ += dtv / nn;
        const double dh = heading_ - meanH_;
        meanH_ += dh / nn;
        ctt_ += dtv * (t - meanT_);
        cth_ += dtv * (heading_ - meanH_);
        chh_ += dh * (heading_ - meanH_);
        rateStats_.add(imu_.yawRate().value());
        const double travel[2] = {sideTravel(sides_[0], base_[0]), sideTravel(sides_[1], base_[1])};
        bool travelRead = true;
        for (std::size_t s = 0; s < 2; ++s) {
            // A side with NO finite reading this tick (the median of nothing: header, "A side
            // that gives no reading") is left out and counted — never added to a σ or judged.
            const double v = sideSpeed(sides_[s]);
            if (std::isfinite(v)) {
                speedStats_[s].add(v);
            } else {
                ++unread_;
            }
            if (!std::isfinite(travel[s])) {
                travelRead = false;
                ++unread_;
            }
        }
        capturedS_ = t;
        lastT_ = t;
        if (!travelRead) {
            // Nothing to judge Moved or Turned on, and nothing to record: the travel is measured
            // from the capture's first sample, so a push during the gap shows at the next tick
            // that reads.
            if (t >= cfg_.duration.value()) {
                finish();
            }
            return;
        }
        // ENCODERS, never the IMU (header): the IMU's apparent rotation at rest IS the drift.
        const double turn = (travel[1] - travel[0]) / width_;
        const double moved = 0.5 * (travel[0] + travel[1]);
        // Recorded before judged: what a rotation under the bound can do to the drift (header).
        lastTurn_ = turn;
        maxTurn_ = std::max(maxTurn_, std::abs(turn));
        maxMove_ = std::max(maxMove_, std::abs(moved));
        if (std::abs(turn) > cfg_.turnBound.value()) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the robot turned %.2f deg by its encoders during the capture (bound "
                          "%.2f deg): LEFT %+.3f in, RIGHT %+.3f in -- hands off, and capture again",
                          turn / kDegreesToRadians, cfg_.turnBound.value() / kDegreesToRadians,
                          travel[0] + 0.0, travel[1] + 0.0);
            refuse(RestRefusal::Turned, 0, msg);
            return;
        }
        if (std::abs(moved) > cfg_.moveBound.value()) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the robot moved %.3f in by its encoders during the capture (bound "
                          "%.3f in): LEFT %+.3f in, RIGHT %+.3f in -- hands off, and capture again",
                          moved, cfg_.moveBound.value(), travel[0] + 0.0, travel[1] + 0.0);
            refuse(RestRefusal::Moved, 0, msg);
            return;
        }
        if (t >= cfg_.duration.value()) {
            finish();
        }
    }

    /// Compute the summary from the online sums (the header's formulas) and end in Done.
    void finish() {
        RestSummary s;
        s.samples = n_;
        s.durationS = lastT_;
        s.timing = timing_.summary();
        const double nn = static_cast<double>(n_);
        s.driftRadPerS = ctt_ > 0.0 ? cth_ / ctt_ : 0.0;
        const double sse = std::max(0.0, chh_ - (ctt_ > 0.0 ? cth_ * cth_ / ctt_ : 0.0));
        s.headingSigmaRad = n_ > 2 ? std::sqrt(sse / (nn - 2.0)) : 0.0;
        s.driftSigmaWhite = ctt_ > 0.0 ? s.headingSigmaRad / std::sqrt(ctt_) : 0.0;
        const double stepSigma = stepStats_.sigma();
        const double dt = s.timing.meanS;
        s.stepRateSigma = dt > 0.0 ? stepSigma / dt : 0.0;
        s.driftSigmaWalk =
            (dt > 0.0 && lastT_ > 0.0) ? stepSigma * std::sqrt(1.2 / (dt * lastT_)) : 0.0;
        s.structureRatio = s.headingSigmaRad > 0.0
                               ? stepSigma / (std::sqrt(2.0) * s.headingSigmaRad)
                               : 0.0;
        s.yawRateMean = rateStats_.mean;
        s.yawRateSigma = rateStats_.sigma();
        s.maxHeadingStepRad = maxStep_;
        s.speedSigma[0] = speedStats_[0].sigma();
        s.speedSigma[1] = speedStats_[1].sigma();
        s.encoderTurnRad = lastTurn_;
        s.encoderTurnMaxRad = maxTurn_;
        s.encoderMoveMaxIn = maxMove_;
        summary_ = s;
        phase_ = RestPhase::Done;
    }

    /// Welford's online mean and spread of one channel.
    struct Welford {
        std::uint32_t n = 0;
        double mean = 0.0;
        double m2 = 0.0;
        void add(double x) {
            ++n;
            const double d = x - mean;
            mean += d / static_cast<double>(n);
            m2 += d * (x - mean);
        }
        /// The SAMPLE standard deviation (divisor n − 1); 0 below two values.
        [[nodiscard]] double sigma() const {
            return n >= 2 ? std::sqrt(m2 / static_cast<double>(n - 1)) : 0.0;
        }
    };

    [[nodiscard]] bool portsUnique() const {
        bool seen[hal::kMaxSmartPorts + 1]{};
        for (const DriveSide& side : sides_) {
            for (const SideMember& m : side.members) {
                const auto p = static_cast<std::size_t>(m.port);
                if (seen[p]) {
                    return false;
                }
                seen[p] = true;
            }
        }
        return true;
    }

    /// The first member turning faster than restFloor (its port), or 0 when every one is still.
    [[nodiscard]] int firstMovingPort() const {
        for (const DriveSide& side : sides_) {
            for (const SideMember& m : side.members) {
                if (std::abs(m.motor->velocity().value()) > cfg_.restFloor.value()) {
                    return m.port;
                }
            }
        }
        return 0;
    }

    void safeState() {
        for (const DriveSide& side : sides_) {
            for (const SideMember& m : side.members) {
                m.motor->setVoltage(units::Voltage{0.0});
                m.motor->setBrakeMode(hal::BrakeMode::Coast);  // never Brake: a person may push it
            }
        }
    }

    void enter(RestPhase p) {
        phase_ = p;
        phaseStart_ = clock_.now().value();
        resting_ = false;
    }

    /// Every refusal comes through here: the reason, the code, the port (0 = none), 0 V + Coast,
    /// Aborted — which makes "0 V + Coast on every refusal" one line, not a property of each site.
    void refuse(RestRefusal code, int port, const char* text) {
        std::snprintf(reason_, sizeof reason_, "%s", text);
        refusal_ = code;
        refusalPort_ = port;
        safeState();
        phase_ = RestPhase::Aborted;
    }

    static constexpr std::size_t kReasonCap = 256;

    DriveSide sides_[2];
    double width_;
    hal::IImu& imu_;
    hal::IClock& clock_;
    RestCaptureConfig cfg_;
    diag::TickTimingStats timing_;

    RestPhase phase_ = RestPhase::Idle;
    RestRefusal refusal_ = RestRefusal::None;
    int refusalPort_ = 0;
    char reason_[kReasonCap] = "";
    double phaseStart_ = 0.0;
    bool resting_ = false;
    double restSince_ = 0.0;

    double base_[2][hal::kMaxSmartPorts]{};
    double t0_ = 0.0;
    double lastT_ = 0.0;
    double capturedS_ = 0.0;
    std::uint32_t n_ = 0;
    std::uint32_t unread_ = 0;
    math::Angle prevHeading_{};
    double heading_ = 0.0;
    double meanT_ = 0.0;
    double meanH_ = 0.0;
    double ctt_ = 0.0;
    double cth_ = 0.0;
    double chh_ = 0.0;
    double maxStep_ = 0.0;
    double lastTurn_ = 0.0;
    double maxTurn_ = 0.0;
    double maxMove_ = 0.0;
    Welford stepStats_{};
    Welford rateStats_{};
    Welford speedStats_[2]{};
    RestSummary summary_{};
};

/// How far each settle floor of a MotionConfig clears what a REST capture measured.
struct SettleFloorReport {
    /// The MEASURED heading-rate noise SettledUtil sees: the summary's σ_Δ/dt, rad/s.
    double headingRateNoise = 0.0;
    /// MotionConfig::headingSettle.maxErrorRate, rad/s.
    double headingRateFloor = 0.0;
    /// floor / noise — how many times over the floor clears it (+infinity for zero noise).
    double headingMargin = 0.0;
    /// headingMargin under the warning factor: a heading settle may flap on noise alone.
    bool headingWarn = false;
    /// The larger side's encoder speed σ at rest, in/s.
    double speedNoise = 0.0;
    /// MotionConfig::brakeSettle.maxError, in/s.
    double speedFloor = 0.0;
    /// floor / noise (+infinity for zero noise).
    double speedMargin = 0.0;
    /// speedMargin under the warning factor.
    bool speedWarn = false;
};

/// Compare MotionConfig's settle floors with a REST capture's MEASURED noise (A4 register HA-51's
/// settle path): the heading-rate floor against σ_Δ/dt — the per-tick step SettledUtil actually
/// differentiates — and the brake floor against the larger side's encoder speed σ. Each margin is
/// floor/noise; a margin under `marginWarn` (RestCaptureConfig::settleMarginWarn, 2×) warns.
/// It NEVER uses √2·σ_line/dt, motion_config.hpp's derivation of 0.3 rad/s: that formula holds only
/// for white heading noise, and under a random walk (HA-22's numbers) it reads ~0.24 rad/s against
/// a true ~0.009 — a false warning against 0.30 (A4 register HA-173). Pure; changes nothing.
[[nodiscard]] inline SettleFloorReport settleFloorReport(
    const RestSummary& summary, const motion::MotionConfig& config,
    double marginWarn = RestCaptureConfig{}.settleMarginWarn) {
    auto margin = [](double floor, double noise) {
        return noise > 0.0 ? floor / noise : std::numeric_limits<double>::infinity();
    };
    SettleFloorReport r;
    r.headingRateNoise = summary.stepRateSigma;
    r.headingRateFloor = config.headingSettle.maxErrorRate;
    r.headingMargin = margin(r.headingRateFloor, r.headingRateNoise);
    r.headingWarn = r.headingMargin < marginWarn;
    r.speedNoise = std::max(summary.speedSigma[0], summary.speedSigma[1]);
    r.speedFloor = config.brakeSettle.maxError;
    r.speedMargin = margin(r.speedFloor, r.speedNoise);
    r.speedWarn = r.speedMargin < marginWarn;
    return r;
}

/// A short, stable, ASCII name for a REST refusal — the serial record's code.
[[nodiscard]] constexpr const char* restRefusalName(RestRefusal r) noexcept {
    switch (r) {
        case RestRefusal::None: return "NONE";
        case RestRefusal::Aborted: return "ABORTED";
        case RestRefusal::ImuNotReady: return "IMU_NOT_READY";
        case RestRefusal::RestTimeout: return "REST_TIMEOUT";
        case RestRefusal::Moved: return "MOVED";
        case RestRefusal::Turned: return "TURNED";
    }
    return "?";
}

}  // namespace shulib::calibration
